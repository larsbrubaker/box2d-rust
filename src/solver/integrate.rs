// Body integration and finalization from solver.c: b2IntegrateVelocitiesTask,
// b2IntegratePositionsTask, and b2FinalizeBodiesTask run as serial loops over
// the whole awake body range (one worker, one block).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::continuous::solve_continuous;
use super::StepContext;
use crate::body::body_flags;
use crate::constants::{speculative_distance, MAX_ROTATION, TIME_TO_SLEEP};
use crate::core::NULL_INDEX;
use crate::events::BodyMoveEvent;
use crate::id::BodyId;
use crate::math_functions::{
    aabb_contains, abs_float, add, dot, integrate_rotation, is_valid_float, is_valid_vec2, length,
    max_float, mul_add, mul_rot, mul_sv, neg, normalize_rot, offset_pos, rotate_vector, Aabb,
    ROT_IDENTITY, VEC2_ZERO,
};
use crate::solver_set::AWAKE_SET;
use crate::types::BodyType;
use crate::world::World;

/// Integrate velocities and apply damping. (b2IntegrateVelocitiesTask)
pub(super) fn integrate_velocities(world: &mut World, context: &StepContext) {
    let gravity = world.gravity;
    let h = context.h;

    let set = &mut world.solver_sets[AWAKE_SET as usize];
    let sims = &set.body_sims;
    let states = &mut set.body_states;

    for (state, sim) in states.iter_mut().zip(sims.iter()) {
        let v = state.linear_velocity;
        let w = state.angular_velocity;

        // Apply forces, torque, gravity, and damping
        // Apply damping.
        // Differential equation: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        let linear_damping = 1.0 / (1.0 + h * sim.linear_damping);
        let angular_damping = 1.0 / (1.0 + h * sim.angular_damping);

        // Gravity scale will be zero for kinematic bodies
        let gravity_scale = if sim.inv_mass > 0.0 {
            sim.gravity_scale
        } else {
            0.0
        };

        // lvd = h * im * f + h * g
        let linear_velocity_delta = add(
            mul_sv(h * sim.inv_mass, sim.force),
            mul_sv(h * gravity_scale, gravity),
        );
        let angular_velocity_delta = h * sim.inv_inertia * sim.torque;

        state.linear_velocity = mul_add(linear_velocity_delta, linear_damping, v);
        state.angular_velocity = angular_velocity_delta + angular_damping * w;
    }
}

/// (b2IntegratePositionsTask)
pub(super) fn integrate_positions(world: &mut World, context: &StepContext) {
    let h = context.h;
    let max_linear_speed = context.max_linear_velocity;
    let max_angular_speed = MAX_ROTATION * context.inv_dt;
    let max_linear_speed_squared = max_linear_speed * max_linear_speed;
    let max_angular_speed_squared = max_angular_speed * max_angular_speed;

    let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;

    for state in states.iter_mut() {
        let mut v = state.linear_velocity;
        let mut w = state.angular_velocity;

        // Motion locks - these can be viewed as a constraint that comes last
        if state.flags & body_flags::LOCK_LINEAR_X != 0 {
            v.x = 0.0;
        }
        if state.flags & body_flags::LOCK_LINEAR_Y != 0 {
            v.y = 0.0;
        }
        if state.flags & body_flags::LOCK_ANGULAR_Z != 0 {
            w = 0.0;
        }

        // Clamp to max linear speed
        if dot(v, v) > max_linear_speed_squared {
            let ratio = max_linear_speed / length(v);
            v = mul_sv(ratio, v);
            state.flags |= body_flags::IS_SPEED_CAPPED;
        }

        // Clamp to max angular speed
        if w * w > max_angular_speed_squared && state.flags & body_flags::ALLOW_FAST_ROTATION == 0 {
            let ratio = max_angular_speed / abs_float(w);
            w *= ratio;
            state.flags |= body_flags::IS_SPEED_CAPPED;
        }

        state.linear_velocity = v;
        state.angular_velocity = w;
        state.delta_position = mul_add(state.delta_position, h, state.linear_velocity);
        state.delta_rotation = integrate_rotation(state.delta_rotation, h * state.angular_velocity);
    }
}

/// (b2FinalizeBodiesTask — serial over the whole awake range on worker 0.
/// Bullet body sim indices collect into `bullet_bodies` instead of the C
/// atomic-indexed arena array.)
pub(super) fn finalize_bodies(
    world: &mut World,
    context: &StepContext,
    bullet_bodies: &mut Vec<i32>,
) {
    let enable_sleep = world.enable_sleep;
    let enable_continuous = world.enable_continuous;
    let time_step = context.dt;
    let inv_time_step = context.inv_dt;
    let world_id = world.world_id;
    let speculative_distance_ = speculative_distance();

    let awake_body_count = world.solver_sets[AWAKE_SET as usize].body_sims.len();
    debug_assert!(awake_body_count <= world.body_move_events.len());

    for sim_index in 0..awake_body_count {
        let mut state = world.solver_sets[AWAKE_SET as usize].body_states[sim_index];
        let mut sim = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];

        let v = state.linear_velocity;
        let w = state.angular_velocity;

        debug_assert!(is_valid_vec2(v));
        debug_assert!(is_valid_float(w));

        sim.center = offset_pos(sim.center, state.delta_position);
        sim.transform.q = normalize_rot(mul_rot(state.delta_rotation, sim.transform.q));

        // Use the velocity of the farthest point on the body to account for
        // rotation.
        let max_velocity = length(v) + abs_float(w) * sim.max_extent;

        // Sleep needs to observe position correction as well as true velocity.
        let max_delta_position =
            length(state.delta_position) + abs_float(state.delta_rotation.s) * sim.max_extent;

        // Position correction is not as important for sleep as true velocity.
        let position_sleep_factor = 0.5;
        let sleep_velocity = max_float(
            max_velocity,
            position_sleep_factor * inv_time_step * max_delta_position,
        );

        // reset state deltas
        state.delta_position = VEC2_ZERO;
        state.delta_rotation = ROT_IDENTITY;

        sim.transform.p = offset_pos(
            sim.center,
            neg(rotate_vector(sim.transform.q, sim.local_center)),
        );

        let body_id = sim.body_id;

        // cache miss here, however I need the shape list below
        {
            let body = &mut world.bodies[body_id as usize];
            body.body_move_index = sim_index as i32;
            world.body_move_events[sim_index] = BodyMoveEvent {
                transform: sim.transform,
                body_id: BodyId {
                    index1: body_id + 1,
                    world0: world_id,
                    generation: body.generation,
                },
                user_data: body.user_data,
                fell_asleep: false,
            };
        }

        // reset applied force and torque
        sim.force = VEC2_ZERO;
        sim.torque = 0.0;

        // If you hit this then it means you deferred mass computation but
        // never called b2Body_ApplyMassFromShapes
        debug_assert!(world.bodies[body_id as usize].flags & body_flags::DIRTY_MASS == 0);

        {
            let body = &mut world.bodies[body_id as usize];
            body.flags &= !body_flags::BODY_TRANSIENT_FLAGS;
            body.flags |=
                sim.flags & (body_flags::IS_SPEED_CAPPED | body_flags::HAD_TIME_OF_IMPACT);
            body.flags |=
                state.flags & (body_flags::IS_SPEED_CAPPED | body_flags::HAD_TIME_OF_IMPACT);
        }
        sim.flags &= !body_flags::BODY_TRANSIENT_FLAGS;
        state.flags &= !body_flags::BODY_TRANSIENT_FLAGS;

        // The state is fully updated; store it before continuous collision
        // reads the world.
        world.solver_sets[AWAKE_SET as usize].body_states[sim_index] = state;

        let body_flags_now = world.bodies[body_id as usize].flags;
        let body_type = world.bodies[body_id as usize].type_;
        let sleep_threshold = world.bodies[body_id as usize].sleep_threshold;

        if !enable_sleep
            || body_flags_now & body_flags::ENABLE_SLEEP == 0
            || sleep_velocity > sleep_threshold
        {
            // Body is not sleepy
            world.bodies[body_id as usize].sleep_time = 0.0;

            let safety_factor = 0.5;
            let max_motion = max_float(max_delta_position, max_velocity * time_step);
            if body_type == BodyType::Dynamic
                && enable_continuous
                && max_motion > safety_factor * sim.min_extent
            {
                // This flag is only retained for debug draw
                sim.flags |= body_flags::IS_FAST;

                if sim.flags & body_flags::IS_BULLET != 0 {
                    // Store in fast array for the continuous collision stage.
                    // This is deterministic because the order of TOI sweeps
                    // doesn't matter.
                    bullet_bodies.push(sim_index as i32);
                    world.solver_sets[AWAKE_SET as usize].body_sims[sim_index] = sim;
                } else {
                    // solve_continuous mutates the stored sim, so store first
                    world.solver_sets[AWAKE_SET as usize].body_sims[sim_index] = sim;
                    solve_continuous(world, sim_index as i32);
                }
            } else {
                // Body is safe to advance
                sim.center0 = sim.center;
                sim.rotation0 = sim.transform.q;
                world.solver_sets[AWAKE_SET as usize].body_sims[sim_index] = sim;
            }
        } else {
            // Body is safe to advance and is falling asleep
            sim.center0 = sim.center;
            sim.rotation0 = sim.transform.q;
            world.bodies[body_id as usize].sleep_time += time_step;
            world.solver_sets[AWAKE_SET as usize].body_sims[sim_index] = sim;
        }

        // Any single body in an island can keep it awake
        let sleep_time = world.bodies[body_id as usize].sleep_time;
        let island_id = world.bodies[body_id as usize].island_id;
        if sleep_time < TIME_TO_SLEEP {
            // keep island awake
            let island_index = world.islands[island_id as usize].local_index;
            world.task_contexts[0]
                .awake_island_bit_set
                .set_bit(island_index as u32);
        } else if world.islands[island_id as usize].constraint_remove_count > 0 {
            // Body wants to sleep but its island needs splitting first. Track
            // the sleepiest candidate. Break sleep time ties using the island
            // id to ensure determinism.
            let task_context = &mut world.task_contexts[0];
            if sleep_time > task_context.split_sleep_time
                || (sleep_time == task_context.split_sleep_time
                    && island_id > task_context.split_island_id)
            {
                task_context.split_island_id = island_id;
                task_context.split_sleep_time = sleep_time;
            }
        }

        // Update shapes AABBs. Continuous collision may have advanced the
        // stored sim; use it.
        let sim_now = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];
        let transform = sim_now.transform;
        let is_fast = sim_now.flags & body_flags::IS_FAST != 0;
        let mut shape_id = world.bodies[body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            if is_fast {
                // For fast non-bullet bodies the AABB has already been updated
                // in solve_continuous. For fast bullet bodies the AABB will be
                // updated at a later stage.

                // Add to enlarged shapes regardless of AABB changes.
                // Bit-set to keep the move array sorted
                world.task_contexts[0]
                    .enlarged_sim_bit_set
                    .set_bit(sim_index as u32);
            } else {
                let shape = &mut world.shapes[shape_id as usize];
                let aabb = crate::geometry::compute_fat_shape_aabb(
                    &shape.geometry,
                    transform,
                    speculative_distance_,
                );
                shape.aabb = aabb;

                debug_assert!(!shape.enlarged_aabb);

                if !aabb_contains(shape.fat_aabb, aabb) {
                    let margin = shape.aabb_margin;
                    let fat_aabb = Aabb {
                        lower_bound: crate::math_functions::Vec2 {
                            x: aabb.lower_bound.x - margin,
                            y: aabb.lower_bound.y - margin,
                        },
                        upper_bound: crate::math_functions::Vec2 {
                            x: aabb.upper_bound.x + margin,
                            y: aabb.upper_bound.y + margin,
                        },
                    };
                    shape.fat_aabb = fat_aabb;

                    shape.enlarged_aabb = true;

                    // Bit-set to keep the move array sorted
                    world.task_contexts[0]
                        .enlarged_sim_bit_set
                        .set_bit(sim_index as u32);
                }
            }

            shape_id = world.shapes[shape_id as usize].next_shape_id;
        }
    }
}
