// Continuous collision from solver.c: b2ContinuousQueryCallback and
// b2SolveContinuous.
//
// The C callback mutates the fast body sim's flags and the context through
// pointers while the tree query holds the world; the Rust port collects those
// effects in the local context (ids and copies instead of pointers) and
// applies them after the queries.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_flags, make_relative_sweep};
use crate::constants::{linear_slop, speculative_distance};
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, time_of_impact, Sweep, ToiInput};
use crate::dynamic_tree::DEFAULT_MASK_BITS;
use crate::id::ShapeId;
use crate::math_functions::{
    aabb_contains, aabb_union, cross, get_length_and_normalize, lerp, make_world_transform, nlerp,
    offset_pos, rotate_vector, sub, to_relative_transform, transform_point, Aabb, Pos, Transform,
    Vec2,
};
use crate::sensor::SensorHit;
use crate::shape::{compute_shape_aabb, make_shape_distance_proxy, should_shapes_collide};
use crate::solver_set::AWAKE_SET;
use crate::types::BodyType;
use crate::world::World;

const MAX_CONTINUOUS_SENSOR_HITS: usize = 8;
const CORE_FRACTION: f32 = 0.25;

struct ContinuousContext {
    fast_body_id: i32,
    fast_shape_id: i32,
    fast_min_extent: f32,
    centroid1: Vec2,
    centroid2: Vec2,
    sweep: Sweep,
    base: Pos,
    fraction: f32,
    /// Deferred fastBodySim->flags |= b2_hadTimeOfImpact
    had_time_of_impact: bool,
    sensor_hits: [SensorHit; MAX_CONTINUOUS_SENSOR_HITS],
    sensor_fractions: [f32; MAX_CONTINUOUS_SENSOR_HITS],
    sensor_count: usize,
}

/// This is called from the dynamic tree query for continuous collision.
/// (b2ContinuousQueryCallback — the closure body; returns true to continue
/// the query)
fn continuous_query_callback(world: &World, ctx: &mut ContinuousContext, user_data: u64) -> bool {
    let shape_id = user_data as i32;

    let fast_shape = &world.shapes[ctx.fast_shape_id as usize];
    debug_assert!(fast_shape.sensor_index == NULL_INDEX);

    // Skip same shape
    if shape_id == fast_shape.id {
        return true;
    }

    let shape = &world.shapes[shape_id as usize];

    // Skip same body
    if shape.body_id == fast_shape.body_id {
        return true;
    }

    // Skip sensors unless the shapes want sensor events
    let is_sensor = shape.sensor_index != NULL_INDEX;
    if is_sensor && (!shape.enable_sensor_events || !fast_shape.enable_sensor_events) {
        return true;
    }

    // Skip filtered shapes
    if !should_shapes_collide(fast_shape.filter, shape.filter) {
        return true;
    }

    let body = &world.bodies[shape.body_id as usize];
    let body_sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
    debug_assert!(
        body.type_ == BodyType::Static || {
            let fast_body = &world.bodies[ctx.fast_body_id as usize];
            let fast_sim = &world.solver_sets[fast_body.set_index as usize].body_sims
                [fast_body.local_index as usize];
            fast_sim.flags & body_flags::IS_BULLET != 0
        }
    );

    // Skip bullets
    if body_sim.flags & body_flags::IS_BULLET != 0 {
        return true;
    }

    // Skip filtered bodies
    if !crate::body::should_bodies_collide(world, ctx.fast_body_id, shape.body_id) {
        return true;
    }

    // Custom user filtering
    if shape.enable_custom_filtering || fast_shape.enable_custom_filtering {
        if let Some(custom_filter_fcn) = world.custom_filter_fcn {
            let id_a = ShapeId {
                index1: shape.id + 1,
                world0: world.world_id,
                generation: shape.generation,
            };
            let id_b = ShapeId {
                index1: fast_shape.id + 1,
                world0: world.world_id,
                generation: fast_shape.generation,
            };
            if !custom_filter_fcn(id_a, id_b, world.custom_filter_context) {
                return true;
            }
        }
    }

    // Early out on fast parallel movement over a chain shape.
    if let crate::collision::ShapeGeometry::ChainSegment(chain_segment) = &shape.geometry {
        let transform = to_relative_transform(body_sim.transform, ctx.base);
        let p1 = transform_point(transform, chain_segment.segment.point1);
        let p2 = transform_point(transform, chain_segment.segment.point2);
        let mut length_ = 0.0;
        let e = get_length_and_normalize(&mut length_, sub(p2, p1));
        if length_ > linear_slop() {
            let c1 = ctx.centroid1;
            let separation1 = cross(sub(c1, p1), e);
            let c2 = ctx.centroid2;
            let separation2 = cross(sub(c2, p1), e);

            let core_distance = CORE_FRACTION * ctx.fast_min_extent;

            if separation1 < 0.0
                || (separation1 - separation2 < core_distance && separation2 > core_distance)
            {
                // Minimal clipping
                return true;
            }
        }
    }

    let mut input = ToiInput {
        proxy_a: make_shape_distance_proxy(shape),
        proxy_b: make_shape_distance_proxy(fast_shape),
        sweep_a: make_relative_sweep(body_sim, ctx.base),
        sweep_b: ctx.sweep,
        max_fraction: ctx.fraction,
    };

    let mut output = time_of_impact(&input);
    if is_sensor {
        // Only accept a sensor hit that is sooner than the current solid hit.
        if output.fraction <= ctx.fraction && ctx.sensor_count < MAX_CONTINUOUS_SENSOR_HITS {
            let index = ctx.sensor_count;

            // The hit shape is a sensor
            let sensor_hit = SensorHit {
                sensor_id: shape.id,
                visitor_id: fast_shape.id,
            };

            ctx.sensor_hits[index] = sensor_hit;
            ctx.sensor_fractions[index] = output.fraction;
            ctx.sensor_count += 1;
        }
    } else {
        let mut hit_fraction = ctx.fraction;
        let mut did_hit = false;

        if 0.0 < output.fraction && output.fraction < ctx.fraction {
            hit_fraction = output.fraction;
            did_hit = true;
        } else if 0.0 == output.fraction {
            // fallback to TOI of a small circle around the fast shape centroid
            let centroid = crate::shape::get_shape_centroid(fast_shape);
            let extent = crate::shape::compute_shape_extent(fast_shape, centroid);
            let radius = CORE_FRACTION * extent.min_extent;
            input.proxy_b = make_proxy(&[centroid], radius);
            output = time_of_impact(&input);
            if 0.0 < output.fraction && output.fraction < ctx.fraction {
                hit_fraction = output.fraction;
                did_hit = true;
            }
        }

        if let (true, true, Some(pre_solve_fcn)) = (
            did_hit,
            shape.enable_pre_solve_events || fast_shape.enable_pre_solve_events,
            world.pre_solve_fcn,
        ) {
            let shape_id_a = ShapeId {
                index1: shape.id + 1,
                world0: world.world_id,
                generation: shape.generation,
            };
            let shape_id_b = ShapeId {
                index1: fast_shape.id + 1,
                world0: world.world_id,
                generation: fast_shape.generation,
            };

            // TOI runs in the base frame, lift the hit point back to world for
            // the callback
            let world_point = offset_pos(ctx.base, output.point);
            did_hit = pre_solve_fcn(
                shape_id_a,
                shape_id_b,
                world_point,
                output.normal,
                world.pre_solve_context,
            );
        }

        if did_hit {
            ctx.had_time_of_impact = true;
            ctx.fraction = hit_fraction;
        }
    }

    // Continue query
    true
}

/// Continuous collision of dynamic versus static. (b2SolveContinuous)
pub(super) fn solve_continuous(world: &mut World, body_sim_index: i32) {
    let fast_body_sim = world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize];
    debug_assert!(fast_body_sim.flags & body_flags::IS_FAST != 0);

    // Re-center the sweep on the fast body so the TOI and the swept query stay
    // in float precision
    let base = fast_body_sim.center0;
    let sweep = make_relative_sweep(&fast_body_sim, base);

    let xf1 = Transform {
        q: sweep.q1,
        p: sub(sweep.c1, rotate_vector(sweep.q1, sweep.local_center)),
    };
    let xf2 = Transform {
        q: sweep.q2,
        p: sub(sweep.c2, rotate_vector(sweep.q2, sweep.local_center)),
    };

    let fast_body_id = fast_body_sim.body_id;

    let mut ctx = ContinuousContext {
        fast_body_id,
        fast_shape_id: NULL_INDEX,
        fast_min_extent: fast_body_sim.min_extent,
        centroid1: Vec2 { x: 0.0, y: 0.0 },
        centroid2: Vec2 { x: 0.0, y: 0.0 },
        sweep,
        base,
        fraction: 1.0,
        had_time_of_impact: false,
        sensor_hits: [SensorHit::default(); MAX_CONTINUOUS_SENSOR_HITS],
        sensor_fractions: [0.0; MAX_CONTINUOUS_SENSOR_HITS],
        sensor_count: 0,
    };

    let is_bullet = fast_body_sim.flags & body_flags::IS_BULLET != 0;

    let mut shape_id = world.bodies[fast_body_id as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        ctx.fast_shape_id = shape_id;
        let next_shape_id;
        let swept_box;
        {
            let fast_shape = &world.shapes[shape_id as usize];
            next_shape_id = fast_shape.next_shape_id;

            ctx.centroid1 = transform_point(xf1, fast_shape.local_centroid);
            ctx.centroid2 = transform_point(xf2, fast_shape.local_centroid);

            let box1 = fast_shape.aabb;

            // xf2 is in the base frame, compute the tight box near the origin
            // then lift to world
            let box2 = crate::aabb::offset_aabb(
                compute_shape_aabb(fast_shape, make_world_transform(xf2)),
                base,
            );

            swept_box = aabb_union(box1, box2);

            // Store this to avoid double computation in the case there is no
            // impact event
            world.shapes[shape_id as usize].aabb = box2;
        }

        // No continuous collision for sensors (but still need the updated
        // bounds)
        if world.shapes[shape_id as usize].sensor_index != NULL_INDEX {
            shape_id = next_shape_id;
            continue;
        }

        {
            let world_ref: &World = world;
            world_ref.broad_phase.trees[BodyType::Static as usize].query(
                swept_box,
                DEFAULT_MASK_BITS,
                |_proxy_id, user_data| continuous_query_callback(world_ref, &mut ctx, user_data),
            );

            if is_bullet {
                world_ref.broad_phase.trees[BodyType::Kinematic as usize].query(
                    swept_box,
                    DEFAULT_MASK_BITS,
                    |_proxy_id, user_data| {
                        continuous_query_callback(world_ref, &mut ctx, user_data)
                    },
                );
                world_ref.broad_phase.trees[BodyType::Dynamic as usize].query(
                    swept_box,
                    DEFAULT_MASK_BITS,
                    |_proxy_id, user_data| {
                        continuous_query_callback(world_ref, &mut ctx, user_data)
                    },
                );
            }
        }

        shape_id = next_shape_id;
    }

    // Apply the deferred time-of-impact flag (the C sets it inside the
    // callback through the sim pointer)
    if ctx.had_time_of_impact {
        world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize].flags |=
            body_flags::HAD_TIME_OF_IMPACT;
    }

    let speculative_distance_ = speculative_distance();

    if ctx.fraction < 1.0 {
        // Handle time of impact event
        let q = nlerp(sweep.q1, sweep.q2, ctx.fraction);
        let c = lerp(sweep.c1, sweep.c2, ctx.fraction);
        let origin = sub(c, rotate_vector(q, sweep.local_center));

        // Advance body, lifting the base frame result back to world
        {
            let sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize];
            sim.transform.q = q;
            sim.transform.p = offset_pos(base, origin);
            sim.center = offset_pos(base, c);
            sim.rotation0 = q;
            sim.center0 = sim.center;

            // Update body move event
            world.body_move_events[body_sim_index as usize].transform = sim.transform;
        }

        // Prepare AABBs for broad-phase. Even though a body is fast, it may
        // not move much. So the AABB may not need enlargement.
        let transform =
            world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize].transform;
        let mut enlarge_bounds = false;

        shape_id = world.bodies[fast_body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            let shape = &mut world.shapes[shape_id as usize];

            // Must recompute aabb at the interpolated transform
            let aabb = crate::geometry::compute_fat_shape_aabb(
                &shape.geometry,
                transform,
                speculative_distance_,
            );
            shape.aabb = aabb;

            if !aabb_contains(shape.fat_aabb, aabb) {
                let margin = shape.aabb_margin;
                let fat_aabb = Aabb {
                    lower_bound: Vec2 {
                        x: aabb.lower_bound.x - margin,
                        y: aabb.lower_bound.y - margin,
                    },
                    upper_bound: Vec2 {
                        x: aabb.upper_bound.x + margin,
                        y: aabb.upper_bound.y + margin,
                    },
                };
                shape.fat_aabb = fat_aabb;

                shape.enlarged_aabb = true;
                enlarge_bounds = true;
            }

            shape_id = shape.next_shape_id;
        }

        if enlarge_bounds {
            world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize].flags |=
                body_flags::ENLARGE_BOUNDS;
        }
    } else {
        // No time of impact event

        // Advance body
        {
            let sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize];
            sim.rotation0 = sim.transform.q;
            sim.center0 = sim.center;
        }

        // Prepare AABBs for broad-phase
        let mut enlarge_bounds = false;
        shape_id = world.bodies[fast_body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            let shape = &mut world.shapes[shape_id as usize];

            // shape->aabb is still valid from above

            if !aabb_contains(shape.fat_aabb, shape.aabb) {
                let margin = shape.aabb_margin;
                let fat_aabb = Aabb {
                    lower_bound: Vec2 {
                        x: shape.aabb.lower_bound.x - margin,
                        y: shape.aabb.lower_bound.y - margin,
                    },
                    upper_bound: Vec2 {
                        x: shape.aabb.upper_bound.x + margin,
                        y: shape.aabb.upper_bound.y + margin,
                    },
                };
                shape.fat_aabb = fat_aabb;

                shape.enlarged_aabb = true;
                enlarge_bounds = true;
            }

            shape_id = shape.next_shape_id;
        }

        if enlarge_bounds {
            world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index as usize].flags |=
                body_flags::ENLARGE_BOUNDS;
        }
    }

    // Push sensor hits on the task context for serial processing.
    for i in 0..ctx.sensor_count {
        // Skip any sensor hits that occurred after a solid hit
        if ctx.sensor_fractions[i] < ctx.fraction {
            let hit = ctx.sensor_hits[i];
            world.task_contexts[0].sensor_hits.push(hit);
        }
    }
}
