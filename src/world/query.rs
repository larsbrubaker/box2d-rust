// World queries from physics_world.c: overlap tests, ray casts, shape casts,
// and the mover (character controller) queries.
//
// C passes function pointers plus a void* context; the Rust port passes
// closures, matching the dynamic-tree callback style used across the crate.
// The tree traversal callbacks (TreeQueryCallback, RayCastCallback, ...) are
// inlined as closures because their only job in C is to unpack the context
// struct.
//
// Queries take &mut World like C takes a non-const world: an active
// recording session captures every query with its hits (the C trampolines
// become a tee inside the callback). The traversal itself only reads, via a
// scoped immutable reborrow.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::World;
use crate::aabb::offset_aabb;
use crate::body::get_body_transform_quick;
use crate::collision::PlaneResult;
use crate::collision::{Capsule, CastOutput, RayCastInput, ShapeCastInput};
use crate::constants::linear_slop;
use crate::distance::{shape_distance, DistanceInput, ShapeProxy, SimplexCache};
use crate::dynamic_tree::{BoxCastInput, TreeStats};
use crate::id::ShapeId;
use crate::math_functions::{
    is_normalized, is_valid_aabb, is_valid_position, is_valid_vec2, make_aabb, offset_pos, sub_pos,
    to_relative_transform, to_vec2, Aabb, Pos, Vec2,
};
use crate::recording::{
    rec_w_aabb, rec_w_bool, rec_w_capsule, rec_w_f32, rec_w_planeresult, rec_w_position,
    rec_w_queryfilter, rec_w_rayresult, rec_w_shapeid, rec_w_shapeproxy, rec_w_vec2,
    record_query_result, QueryRecorder, OP_QUERY_CAST_MOVER, OP_QUERY_CAST_RAY,
    OP_QUERY_CAST_RAY_CLOSEST, OP_QUERY_CAST_SHAPE, OP_QUERY_COLLIDE_MOVER, OP_QUERY_OVERLAP_AABB,
    OP_QUERY_OVERLAP_SHAPE,
};
use crate::shape::{
    collide_mover, make_shape_distance_proxy, ray_cast_shape, shape_cast_shape,
    should_query_collide, Shape,
};
use crate::types::{QueryFilter, BODY_TYPE_COUNT};

/// Make a user-facing shape id for a shape. Queries hand these to callbacks.
fn query_shape_id(world: &World, shape: &Shape) -> ShapeId {
    ShapeId {
        index1: shape.id + 1,
        world0: world.world_id,
        generation: shape.generation,
    }
}

/// Overlap test for all shapes that potentially overlap the provided AABB.
/// The callback receives each overlapping shape id and returns false to
/// terminate the query. (b2World_OverlapAABB + static TreeQueryCallback)
pub fn world_overlap_aabb(
    world: &mut World,
    origin: Pos,
    aabb: Aabb,
    filter: QueryFilter,
    mut fcn: impl FnMut(ShapeId) -> bool,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_aabb(aabb));

    let mut q = QueryRecorder::begin(world, |buf| {
        rec_w_position(buf, origin);
        rec_w_aabb(buf, aabb);
        rec_w_queryfilter(buf, filter);
    });

    {
        let world: &World = world;

        // Lift to a world float box with outward rounding so the conservative
        // tree test never misses
        let world_box = offset_aabb(aabb, origin);

        for i in 0..BODY_TYPE_COUNT {
            let tree_result =
                world.broad_phase.trees[i].query(world_box, filter.mask_bits, |_, user_data| {
                    // (static TreeQueryCallback)
                    let shape_id = user_data as i32;
                    let shape = &world.shapes[shape_id as usize];

                    if !should_query_collide(shape.filter, filter) {
                        return true;
                    }

                    let id = ShapeId {
                        index1: shape_id + 1,
                        world0: world.world_id,
                        generation: shape.generation,
                    };
                    let ret = fcn(id);
                    if q.active() {
                        // (b2RecOverlapTrampoline)
                        rec_w_shapeid(&mut q.buf, id);
                        rec_w_bool(&mut q.buf, ret);
                        q.hits += 1;
                    }
                    ret
                });

            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;
        }
    }

    q.commit(world, OP_QUERY_OVERLAP_AABB, Some(tree_stats));
    tree_stats
}

/// Overlap test for all shapes that overlap the provided shape proxy. The
/// callback returns false to terminate the query.
/// (b2World_OverlapShape + static TreeOverlapCallback)
pub fn world_overlap_shape(
    world: &mut World,
    origin: Pos,
    proxy: &ShapeProxy,
    filter: QueryFilter,
    mut fcn: impl FnMut(ShapeId) -> bool,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_position(origin));

    let mut q = QueryRecorder::begin(world, |buf| {
        rec_w_position(buf, origin);
        rec_w_shapeproxy(buf, proxy);
        rec_w_queryfilter(buf, filter);
    });

    {
        let world: &World = world;

        // Relative box lifted to world float with outward rounding,
        // conservative for the tree
        let aabb = offset_aabb(
            make_aabb(&proxy.points[..proxy.count as usize], proxy.radius),
            origin,
        );

        for i in 0..BODY_TYPE_COUNT {
            let tree_result =
                world.broad_phase.trees[i].query(aabb, filter.mask_bits, |_, user_data| {
                    // (static TreeOverlapCallback)
                    let shape_id = user_data as i32;
                    let shape = &world.shapes[shape_id as usize];

                    if !should_query_collide(shape.filter, filter) {
                        return true;
                    }

                    // Re-center on the query origin so the distance test
                    // stays in float precision far from the world origin
                    let body = &world.bodies[shape.body_id as usize];
                    let transform =
                        to_relative_transform(get_body_transform_quick(world, body), origin);

                    let input = DistanceInput {
                        proxy_a: *proxy,
                        proxy_b: make_shape_distance_proxy(shape),
                        transform,
                        use_radii: true,
                    };

                    let mut cache = SimplexCache::default();
                    let output = shape_distance(&input, &mut cache, None);

                    let tolerance = 0.1 * linear_slop();
                    if output.distance > tolerance {
                        return true;
                    }

                    let id = query_shape_id(world, shape);
                    let ret = fcn(id);
                    if q.active() {
                        rec_w_shapeid(&mut q.buf, id);
                        rec_w_bool(&mut q.buf, ret);
                        q.hits += 1;
                    }
                    ret
                });

            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;
        }
    }

    q.commit(world, OP_QUERY_OVERLAP_SHAPE, Some(tree_stats));
    tree_stats
}

/// The shared per-tree ray cast loop. CastRay records around it; the closest
/// variant runs it directly and records a self-contained result instead,
/// matching C where the two entry points duplicate this loop.
fn cast_ray_impl(
    world: &World,
    origin: Pos,
    translation: Vec2,
    filter: QueryFilter,
    mut fcn: impl FnMut(ShapeId, Pos, Vec2, f32) -> f32,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    // Tree traversal sees the origin truncated to float, displacing the ray
    // by up to one coordinate ULP, a graze sized miss tolerance at extreme
    // range. Per-shape casts re-difference against the full precision origin.
    let mut input = RayCastInput {
        origin: to_vec2(origin),
        translation,
        max_fraction: 1.0,
    };

    let mut fraction = 1.0f32;

    for i in 0..BODY_TYPE_COUNT {
        let tree_result = world.broad_phase.trees[i].ray_cast(
            &input,
            filter.mask_bits,
            |tree_input, _, user_data| {
                // (static RayCastCallback)
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(shape.filter, filter) {
                    return tree_input.max_fraction;
                }

                let body = &world.bodies[shape.body_id as usize];
                let xf = get_body_transform_quick(world, body);

                // Re-center on the body so the per-shape cast stays in float
                // precision far from the origin. The tree traversal already
                // used the truncated origin in input. Here we re-difference in
                // full precision against the body position.
                let base = xf.p;
                let transform = to_relative_transform(xf, base);
                let mut local_input = *tree_input;
                local_input.origin = sub_pos(origin, base);
                let output: CastOutput = ray_cast_shape(&local_input, shape, transform);

                if output.hit {
                    let id = query_shape_id(world, shape);
                    let point = offset_pos(base, output.point);
                    let user_fraction = fcn(id, point, output.normal, output.fraction);

                    // The user may return -1 to skip this shape
                    if (0.0..=1.0).contains(&user_fraction) {
                        fraction = user_fraction;
                    }

                    return user_fraction;
                }

                tree_input.max_fraction
            },
        );
        tree_stats.node_visits += tree_result.node_visits;
        tree_stats.leaf_visits += tree_result.leaf_visits;

        if fraction == 0.0 {
            break;
        }

        input.max_fraction = fraction;
    }

    tree_stats
}

/// Cast a ray into the world to collect shapes in the path of the ray. The
/// callback receives `(shape_id, point, normal, fraction)` and controls the
/// continuation like C's b2CastResultFcn: return -1 to ignore the hit, 0 to
/// terminate, a fraction to clip the ray, or 1 to continue without clipping.
/// (b2World_CastRay + static RayCastCallback)
pub fn world_cast_ray(
    world: &mut World,
    origin: Pos,
    translation: Vec2,
    filter: QueryFilter,
    mut fcn: impl FnMut(ShapeId, Pos, Vec2, f32) -> f32,
) -> TreeStats {
    debug_assert!(!world.locked);
    if world.locked {
        return TreeStats::default();
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec2(translation));

    let mut q = QueryRecorder::begin(world, |buf| {
        rec_w_position(buf, origin);
        rec_w_vec2(buf, translation);
        rec_w_queryfilter(buf, filter);
    });

    let tree_stats = cast_ray_impl(
        world,
        origin,
        translation,
        filter,
        |id, point, normal, fraction| {
            let ret = fcn(id, point, normal, fraction);
            if q.active() {
                // (b2RecCastTrampoline)
                rec_w_shapeid(&mut q.buf, id);
                rec_w_position(&mut q.buf, point);
                rec_w_vec2(&mut q.buf, normal);
                rec_w_f32(&mut q.buf, fraction);
                rec_w_f32(&mut q.buf, ret);
                q.hits += 1;
            }
            ret
        },
    );

    q.commit(world, OP_QUERY_CAST_RAY, Some(tree_stats));
    tree_stats
}

/// Cast a ray into the world to collect the closest hit. This is a
/// convenience function. Ignores initial overlap.
/// (b2World_CastRayClosest + static b2RayCastClosestFcn)
pub fn world_cast_ray_closest(
    world: &mut World,
    origin: Pos,
    translation: Vec2,
    filter: QueryFilter,
) -> crate::types::RayResult {
    let mut result = crate::types::RayResult::default();

    debug_assert!(!world.locked);
    if world.locked {
        return result;
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec2(translation));

    // C runs its own loop with b2RayCastClosestFcn and records one
    // self-contained result, never per-hit tails.
    let stats = cast_ray_impl(
        world,
        origin,
        translation,
        filter,
        |id, point, normal, fraction| {
            // Ignore initial overlap
            if fraction == 0.0 {
                return -1.0;
            }

            result.shape_id = id;
            result.point = point;
            result.normal = normal;
            result.fraction = fraction;
            result.hit = true;
            fraction
        },
    );

    result.node_visits = stats.node_visits;
    result.leaf_visits = stats.leaf_visits;

    record_query_result(
        world,
        OP_QUERY_CAST_RAY_CLOSEST,
        |buf| {
            rec_w_position(buf, origin);
            rec_w_vec2(buf, translation);
            rec_w_queryfilter(buf, filter);
        },
        |buf| rec_w_rayresult(buf, &result),
    );

    result
}

/// Cast a shape through the world. Similar to a cast ray except that a shape
/// is cast instead of a point. The callback contract matches
/// [`world_cast_ray`]. (b2World_CastShape + static ShapeCastCallback)
pub fn world_cast_shape(
    world: &mut World,
    origin: Pos,
    proxy: &ShapeProxy,
    translation: Vec2,
    filter: QueryFilter,
    mut fcn: impl FnMut(ShapeId, Pos, Vec2, f32) -> f32,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec2(translation));

    let mut q = QueryRecorder::begin(world, |buf| {
        rec_w_position(buf, origin);
        rec_w_shapeproxy(buf, proxy);
        rec_w_vec2(buf, translation);
        rec_w_queryfilter(buf, filter);
    });

    {
        let world: &World = world;

        // Origin relative input carried on the context in C
        // (WorldShapeCastContext)
        let cast_input = ShapeCastInput {
            proxy: *proxy,
            translation,
            max_fraction: 1.0,
            can_encroach: false,
        };

        let mut fraction = 1.0f32;

        // Bound the proxy in origin relative space then lift to a
        // conservative world float box. The tree node boxes use the same
        // directed rounding, so the swept box never clips a shape far from
        // the origin. Per shape casts re-difference at full precision against
        // the carried origin.
        let local_box = make_aabb(&proxy.points[..proxy.count as usize], proxy.radius);
        let box_ = offset_aabb(local_box, origin);
        let mut tree_input = BoxCastInput {
            box_,
            translation,
            max_fraction: 1.0,
        };

        for i in 0..BODY_TYPE_COUNT {
            let tree_result = world.broad_phase.trees[i].box_cast(
                &tree_input,
                filter.mask_bits,
                |box_input, _, user_data| {
                    // (static ShapeCastCallback)
                    let shape_id = user_data as i32;
                    let shape = &world.shapes[shape_id as usize];

                    if !should_query_collide(shape.filter, filter) {
                        return box_input.max_fraction;
                    }

                    // Rebuild from the origin relative input, taking only the
                    // advancing fraction from the tree. The tree input is
                    // world float and would lose the cast far from the origin.
                    let mut local_input = cast_input;
                    local_input.max_fraction = box_input.max_fraction;

                    let body = &world.bodies[shape.body_id as usize];
                    let transform = get_body_transform_quick(world, body);
                    let local_transform = to_relative_transform(transform, origin);

                    let output = shape_cast_shape(&local_input, shape, local_transform);

                    if output.hit {
                        let id = query_shape_id(world, shape);
                        let point = offset_pos(origin, output.point);
                        let user_fraction = fcn(id, point, output.normal, output.fraction);
                        if q.active() {
                            rec_w_shapeid(&mut q.buf, id);
                            rec_w_position(&mut q.buf, point);
                            rec_w_vec2(&mut q.buf, output.normal);
                            rec_w_f32(&mut q.buf, output.fraction);
                            rec_w_f32(&mut q.buf, user_fraction);
                            q.hits += 1;
                        }

                        // The user may return -1 to skip this shape
                        if (0.0..=1.0).contains(&user_fraction) {
                            fraction = user_fraction;
                        }

                        return user_fraction;
                    }

                    box_input.max_fraction
                },
            );
            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;

            if fraction == 0.0 {
                break;
            }

            tree_input.max_fraction = fraction;
        }
    }

    q.commit(world, OP_QUERY_CAST_SHAPE, Some(tree_stats));
    tree_stats
}

/// Cast a capsule mover through the world. This is a special shape cast that
/// handles sliding along other shapes while reducing clipping. Returns the
/// fraction of the translation that can be performed without a hit.
/// (b2World_CastMover + static MoverCastCallback)
pub fn world_cast_mover(
    world: &mut World,
    origin: Pos,
    mover: &Capsule,
    translation: Vec2,
    filter: QueryFilter,
) -> f32 {
    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec2(translation));
    debug_assert!(mover.radius > 2.0 * linear_slop());

    debug_assert!(!world.locked);
    if world.locked {
        return 1.0;
    }

    let mut fraction = 1.0f32;

    {
        let world: &World = world;

        // Origin relative input carried on the context in C
        // (WorldMoverCastContext)
        let mut cast_input = ShapeCastInput::default();
        cast_input.proxy.points[0] = mover.center1;
        cast_input.proxy.points[1] = mover.center2;
        cast_input.proxy.count = 2;
        cast_input.proxy.radius = mover.radius;
        cast_input.translation = translation;
        cast_input.max_fraction = 1.0;
        cast_input.can_encroach = true;

        // Bound the capsule in origin relative space then lift to a
        // conservative world float box
        let centers = [mover.center1, mover.center2];
        let box_ = offset_aabb(make_aabb(&centers, mover.radius), origin);
        let mut tree_input = BoxCastInput {
            box_,
            translation,
            max_fraction: 1.0,
        };

        for i in 0..BODY_TYPE_COUNT {
            world.broad_phase.trees[i].box_cast(
                &tree_input,
                filter.mask_bits,
                |box_input, _, user_data| {
                    // (static MoverCastCallback)
                    let shape_id = user_data as i32;
                    let shape = &world.shapes[shape_id as usize];

                    if !should_query_collide(shape.filter, filter) {
                        return fraction;
                    }

                    // Rebuild from the origin relative input, taking only the
                    // advancing fraction from the tree
                    let mut local_input = cast_input;
                    local_input.max_fraction = box_input.max_fraction;

                    let body = &world.bodies[shape.body_id as usize];
                    let transform =
                        to_relative_transform(get_body_transform_quick(world, body), origin);

                    let output = shape_cast_shape(&local_input, shape, transform);
                    if output.fraction == 0.0 {
                        // Ignore overlapping shapes
                        return fraction;
                    }

                    fraction = output.fraction;
                    output.fraction
                },
            );

            if fraction == 0.0 {
                break;
            }

            tree_input.max_fraction = fraction;
        }
    }

    record_query_result(
        world,
        OP_QUERY_CAST_MOVER,
        |buf| {
            rec_w_position(buf, origin);
            rec_w_capsule(buf, *mover);
            rec_w_vec2(buf, translation);
            rec_w_queryfilter(buf, filter);
        },
        |buf| rec_w_f32(buf, fraction),
    );

    fraction
}

/// Collide a capsule mover with the world, gathering collision planes that
/// can be fed to `solve_planes`. Useful for character controllers. The
/// callback returns false to terminate the query.
/// (b2World_CollideMover + static TreeCollideCallback)
pub fn world_collide_mover(
    world: &mut World,
    origin: Pos,
    mover: &Capsule,
    filter: QueryFilter,
    mut fcn: impl FnMut(ShapeId, &PlaneResult) -> bool,
) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    debug_assert!(is_valid_position(origin));

    let mut q = QueryRecorder::begin(world, |buf| {
        rec_w_position(buf, origin);
        rec_w_capsule(buf, *mover);
        rec_w_queryfilter(buf, filter);
    });

    {
        let world: &World = world;

        let r = Vec2 {
            x: mover.radius,
            y: mover.radius,
        };

        // Relative box lifted to world float with outward rounding,
        // conservative for the tree
        let rel_box = Aabb {
            lower_bound: crate::math_functions::sub(
                crate::math_functions::min(mover.center1, mover.center2),
                r,
            ),
            upper_bound: crate::math_functions::add(
                crate::math_functions::max(mover.center1, mover.center2),
                r,
            ),
        };
        let aabb = offset_aabb(rel_box, origin);

        for i in 0..BODY_TYPE_COUNT {
            world.broad_phase.trees[i].query(aabb, filter.mask_bits, |_, user_data| {
                // (static TreeCollideCallback)
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(shape.filter, filter) {
                    return true;
                }

                // Re-center on the query origin, the mover and the resulting
                // planes are origin relative
                let body = &world.bodies[shape.body_id as usize];
                let transform =
                    to_relative_transform(get_body_transform_quick(world, body), origin);

                let result = collide_mover(mover, shape, transform);

                // todo handle deep overlap
                if result.hit && is_normalized(result.plane.normal) {
                    let id = query_shape_id(world, shape);
                    let ret = fcn(id, &result);
                    if q.active() {
                        // (b2RecPlaneTrampoline)
                        rec_w_shapeid(&mut q.buf, id);
                        rec_w_planeresult(&mut q.buf, &result);
                        rec_w_bool(&mut q.buf, ret);
                        q.hits += 1;
                    }
                    return ret;
                }

                true
            });
        }
    }

    // CollideMover has no TREESTATS tail (returns void)
    q.commit(world, OP_QUERY_COLLIDE_MOVER, None);
}
