// The solve driver from solver.c: b2Solve with the b2SolverTask stage
// sequence run serially.
//
// The C partitions each stage into blocks claimed by workers via atomic CAS;
// with one worker every stage is a single full-range block, so the sync
// machinery (b2SolverStage/b2SyncBlock/atomicSyncBits) disappears and the
// orchestrator loop in b2SolverTask becomes plain loops here. The stage
// order, the overflow-before-colors ordering, and the ascending color order
// are preserved exactly — they determine the float accumulation order.
//
// Per-color contact constraints live in a local Vec per color (the C uses
// arena scratch hung off b2GraphColor). All colors run the scalar kernels;
// see contact_solver.rs for why this matches the C wide path.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: called by the world step slice.
// The range loops index two parallel arrays (constraints and graph colors).
#![allow(clippy::needless_range_loop)]

use super::integrate::{finalize_bodies, integrate_positions, integrate_velocities};
use super::StepContext;
use crate::constants::GRAPH_COLOR_COUNT;
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::contact::contact_flags;
use crate::contact_solver::{
    apply_restitution, prepare_contacts, solve_contacts, store_impulses, warm_start_contacts,
    ContactConstraint,
};
use crate::core::NULL_INDEX;
use crate::events::{BodyMoveEvent, ContactHitEvent, JointEvent};
use crate::id::{BodyId, ContactId, JointId, ShapeId};
use crate::joint::{get_joint_reaction, prepare_joint, solve_joint, warm_start_joint};
use crate::math_functions::{make_world_transform, offset_pos, TRANSFORM_IDENTITY};
use crate::solver_set::AWAKE_SET;
use crate::timer::{get_milliseconds, get_milliseconds_and_reset, get_ticks};
use crate::types::BodyType;
use crate::world::World;

// (solver.c: ITERATIONS / RELAX_ITERATIONS)
const ITERATIONS: i32 = 1;
const RELAX_ITERATIONS: i32 = 1;

/// Prepare the joints of one graph color. The C prepare stage reads world
/// data while writing the sim, so the sims are copied out and back.
fn prepare_color_joints(world: &mut World, color_index: i32, context: &StepContext) {
    let count = world.constraint_graph.colors[color_index as usize]
        .joint_sims
        .len();
    for i in 0..count {
        let mut sim = world.constraint_graph.colors[color_index as usize].joint_sims[i];
        prepare_joint(world, &mut sim, context);
        world.constraint_graph.colors[color_index as usize].joint_sims[i] = sim;
    }
}

/// Solve with graph coloring. (b2Solve)
pub fn solve(world: &mut World, context: &StepContext) {
    // Only count steps that advance the simulation
    world.step_index += 1;

    // Are there any awake bodies? This scenario should not be important for
    // profiling.
    let awake_body_count = world.solver_sets[AWAKE_SET as usize].body_sims.len();
    if awake_body_count == 0 {
        world.broad_phase.validate_no_enlarged();
        return;
    }

    // Solver setup: buffers, move events, bit sets, island split enqueue/run.
    // (solver.c: setupTicks → profile.solverSetup)
    let mut setup_ticks = get_ticks();

    // Prepare buffer for bullets (arena in C)
    let mut bullet_bodies: Vec<i32> = Vec::with_capacity(awake_body_count);

    // prepare for move events
    world.body_move_events.resize(
        awake_body_count,
        BodyMoveEvent {
            user_data: 0,
            transform: make_world_transform(TRANSFORM_IDENTITY),
            body_id: BodyId::default(),
            fell_asleep: false,
        },
    );

    // Reset the per-step event bit sets (b2Solve does this per worker before
    // spawning the solver tasks)
    let joint_id_capacity = world.joint_id_pool.id_capacity();
    let contact_id_capacity = world.contact_id_pool.id_capacity();
    {
        let task_context = &mut world.task_contexts[0];
        task_context
            .joint_state_bit_set
            .set_bit_count_and_clear(joint_id_capacity as u32);
        task_context
            .hit_event_bit_set
            .set_bit_count_and_clear(contact_id_capacity as u32);
        task_context.has_hit_events = false;
    }

    // Split an awake island. The C enqueues b2SplitIslandTask to run
    // concurrently with the constraint solve and finishes it before body
    // finalization; the serial port runs it inline here. Timing still goes to
    // split_islands (b2SplitIslandTask) and is also nested in solver_setup
    // when the split runs synchronously during setup.
    if world.split_island_id != NULL_INDEX {
        let split_ticks = get_ticks();
        let split_id = world.split_island_id;
        crate::island::split_island(world, split_id);
        world.split_island_id = NULL_INDEX;
        world.profile.split_islands += get_milliseconds(split_ticks);
    }

    world.profile.solver_setup = get_milliseconds_and_reset(&mut setup_ticks);

    // Constraint solve wall time (prepare → store impulses), matching C's
    // profile.constraints around b2SolverTask.
    let mut constraint_ticks = get_ticks();
    let mut ticks = get_ticks();

    // === Prepare constraints ===
    // (stage b2_stagePrepareJoints: colored joints in ascending color order)
    for color_index in 0..OVERFLOW_INDEX {
        prepare_color_joints(world, color_index, context);
    }

    // (stage b2_stagePrepareContacts: colored contacts)
    let mut color_constraints: Vec<Vec<ContactConstraint>> =
        (0..GRAPH_COLOR_COUNT).map(|_| Vec::new()).collect();
    for color_index in 0..GRAPH_COLOR_COUNT as usize {
        // Overflow contacts prepare below to match the C stage order; the
        // constraint storage is sized here either way.
        let count = world.constraint_graph.colors[color_index]
            .contact_sims
            .len();
        color_constraints[color_index].resize(count, ContactConstraint::default());
    }
    for color_index in 0..OVERFLOW_INDEX as usize {
        let contacts = &world.constraint_graph.colors[color_index].contact_sims;
        let states = &world.solver_sets[AWAKE_SET as usize].body_states;
        prepare_contacts(
            &mut color_constraints[color_index],
            contacts,
            states,
            context,
        );
    }

    // Single-threaded overflow work. These constraints don't fit in the graph
    // coloring. (b2PrepareJoints_Overflow / b2PrepareContacts_Overflow)
    prepare_color_joints(world, OVERFLOW_INDEX, context);
    {
        let contacts = &world.constraint_graph.colors[OVERFLOW_INDEX as usize].contact_sims;
        let states = &world.solver_sets[AWAKE_SET as usize].body_states;
        prepare_contacts(
            &mut color_constraints[OVERFLOW_INDEX as usize],
            contacts,
            states,
            context,
        );
    }

    world.profile.prepare_constraints += get_milliseconds_and_reset(&mut ticks);

    // === Sub-step loop ===
    let sub_step_count = context.sub_step_count;
    for _sub_step_index in 0..sub_step_count {
        // Integrate velocities
        integrate_velocities(world, context);
        world.profile.integrate_velocities += get_milliseconds_and_reset(&mut ticks);

        // Warm start constraints: overflow joints, overflow contacts, then
        // each color (joints before contacts, matching the graph block
        // layout).
        {
            let world_parts = &mut *world;
            let graph = &mut world_parts.constraint_graph;
            let states = &mut world_parts.solver_sets[AWAKE_SET as usize].body_states;

            for joint in graph.colors[OVERFLOW_INDEX as usize].joint_sims.iter_mut() {
                warm_start_joint(joint, states);
            }
            warm_start_contacts(&mut color_constraints[OVERFLOW_INDEX as usize], states);

            for color_index in 0..OVERFLOW_INDEX as usize {
                for joint in graph.colors[color_index].joint_sims.iter_mut() {
                    warm_start_joint(joint, states);
                }
                warm_start_contacts(&mut color_constraints[color_index], states);
            }
        }
        world.profile.warm_start += get_milliseconds_and_reset(&mut ticks);

        // Solve constraints
        for _ in 0..ITERATIONS {
            let use_bias = true;
            let world_parts = &mut *world;
            let graph = &mut world_parts.constraint_graph;
            let states = &mut world_parts.solver_sets[AWAKE_SET as usize].body_states;
            let joint_state_bit_set = &mut world_parts.task_contexts[0].joint_state_bit_set;

            // Overflow constraints have lower priority. Typically these are
            // dynamic-vs-dynamic. (b2SolveJoints_Overflow does not report
            // joint events; only the colored b2SolveJointsTask does.)
            for joint in graph.colors[OVERFLOW_INDEX as usize].joint_sims.iter_mut() {
                solve_joint(joint, context, states, use_bias);
            }
            solve_contacts(
                &mut color_constraints[OVERFLOW_INDEX as usize],
                states,
                context,
                use_bias,
            );

            for color_index in 0..OVERFLOW_INDEX as usize {
                for joint in graph.colors[color_index].joint_sims.iter_mut() {
                    solve_joint(joint, context, states, use_bias);

                    if (joint.force_threshold < f32::MAX || joint.torque_threshold < f32::MAX)
                        && !joint_state_bit_set.get_bit(joint.joint_id as u32)
                    {
                        let (force, torque) = get_joint_reaction(joint, context.inv_h);

                        // Check thresholds. A zero threshold means all awake
                        // joints get reported.
                        if force >= joint.force_threshold || torque >= joint.torque_threshold {
                            // Flag this joint for processing.
                            joint_state_bit_set.set_bit(joint.joint_id as u32);
                        }
                    }
                }
                solve_contacts(
                    &mut color_constraints[color_index],
                    states,
                    context,
                    use_bias,
                );
            }
        }
        world.profile.solve_impulses += get_milliseconds_and_reset(&mut ticks);

        // Integrate positions
        integrate_positions(world, context);
        world.profile.integrate_positions += get_milliseconds_and_reset(&mut ticks);

        // Relax constraints
        for _ in 0..RELAX_ITERATIONS {
            let use_bias = false;
            let world_parts = &mut *world;
            let graph = &mut world_parts.constraint_graph;
            let states = &mut world_parts.solver_sets[AWAKE_SET as usize].body_states;

            for joint in graph.colors[OVERFLOW_INDEX as usize].joint_sims.iter_mut() {
                solve_joint(joint, context, states, use_bias);
            }
            solve_contacts(
                &mut color_constraints[OVERFLOW_INDEX as usize],
                states,
                context,
                use_bias,
            );

            for color_index in 0..OVERFLOW_INDEX as usize {
                for joint in graph.colors[color_index].joint_sims.iter_mut() {
                    solve_joint(joint, context, states, use_bias);
                }
                solve_contacts(
                    &mut color_constraints[color_index],
                    states,
                    context,
                    use_bias,
                );
            }
        }
        world.profile.relax_impulses += get_milliseconds_and_reset(&mut ticks);
    }

    // Restitution: overflow first, then each color (contacts only)
    {
        let world_parts = &mut *world;
        let states = &mut world_parts.solver_sets[AWAKE_SET as usize].body_states;

        apply_restitution(
            &mut color_constraints[OVERFLOW_INDEX as usize],
            states,
            context,
        );
        for color_index in 0..OVERFLOW_INDEX as usize {
            apply_restitution(&mut color_constraints[color_index], states, context);
        }
    }
    world.profile.apply_restitution += get_milliseconds_and_reset(&mut ticks);

    // Store impulses: overflow first (no hit-event flagging in the C overflow
    // path), then the colored contacts with hit-event flagging
    // (b2StoreImpulsesTask).
    {
        let world_parts = &mut *world;
        let graph = &mut world_parts.constraint_graph;
        let task_context = &mut world_parts.task_contexts[0];
        let neg_hit_threshold = -world_parts.hit_event_threshold;

        store_impulses(
            &color_constraints[OVERFLOW_INDEX as usize],
            &mut graph.colors[OVERFLOW_INDEX as usize].contact_sims,
        );

        for color_index in 0..OVERFLOW_INDEX as usize {
            store_impulses(
                &color_constraints[color_index],
                &mut graph.colors[color_index].contact_sims,
            );

            // Check for hit events to speed up serial processing later in the
            // step
            for contact_sim in &graph.colors[color_index].contact_sims {
                if contact_sim.sim_flags & contact_flags::SIM_ENABLE_HIT_EVENT != 0 {
                    for k in 0..contact_sim.manifold.point_count as usize {
                        let mp = &contact_sim.manifold.points[k];

                        // Need to check total impulse because the point may be
                        // speculative and not colliding
                        if mp.normal_velocity < neg_hit_threshold && mp.total_normal_impulse > 0.0 {
                            task_context
                                .hit_event_bit_set
                                .set_bit(contact_sim.contact_id as u32);
                            task_context.has_hit_events = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    world.profile.store_impulses += get_milliseconds_and_reset(&mut ticks);

    // Finish island split (already done inline above in the serial port).
    world.profile.constraints = get_milliseconds_and_reset(&mut constraint_ticks);

    // === Finalize bodies ===
    let transform_ticks = get_ticks();

    // Prepare contact, enlarged body, and island bit sets used in body
    // finalization.
    {
        let awake_island_count = world.solver_sets[AWAKE_SET as usize].island_sims.len();
        let task_context = &mut world.task_contexts[0];
        task_context.sensor_hits.clear();
        task_context
            .enlarged_sim_bit_set
            .set_bit_count_and_clear(awake_body_count as u32);
        task_context
            .awake_island_bit_set
            .set_bit_count_and_clear(awake_island_count as u32);
        task_context.split_island_id = NULL_INDEX;
        task_context.split_sleep_time = 0.0;
    }

    // Finalize bodies. Must happen after the constraint solver and after
    // island splitting.
    finalize_bodies(world, context, &mut bullet_bodies);

    world.profile.transforms = get_milliseconds(transform_ticks);

    // === Report joint events ===
    {
        let joint_event_ticks = get_ticks();
        let world_id = world.world_id;
        let word_count = world.task_contexts[0].joint_state_bit_set.block_count();
        for k in 0..word_count {
            let mut word = world.task_contexts[0].joint_state_bit_set.block(k);
            while word != 0 {
                let ctz = word.trailing_zeros();
                let joint_id = (64 * k + ctz) as i32;

                let joint = &world.joints[joint_id as usize];
                debug_assert!(joint.set_index == AWAKE_SET);

                let event = JointEvent {
                    joint_id: JointId {
                        index1: joint_id + 1,
                        world0: world_id,
                        generation: joint.generation,
                    },
                    user_data: joint.user_data,
                };

                world.joint_events.push(event);

                // Clear the smallest set bit
                word &= word - 1;
            }
        }
        world.profile.joint_events = get_milliseconds(joint_event_ticks);
    }

    // === Report hit events ===
    {
        let hit_ticks = get_ticks();
        debug_assert!(world.contact_hit_events.is_empty());

        if world.task_contexts[0].has_hit_events {
            let threshold = world.hit_event_threshold;
            let world_id = world.world_id;

            let word_count = world.task_contexts[0].hit_event_bit_set.block_count();
            for k in 0..word_count {
                let mut word = world.task_contexts[0].hit_event_bit_set.block(k);
                while word != 0 {
                    let ctz = word.trailing_zeros();
                    let contact_id = (64 * k + ctz) as i32;

                    let contact = world.contacts[contact_id as usize];
                    debug_assert!(
                        contact.set_index == AWAKE_SET && contact.color_index != NULL_INDEX
                    );

                    let contact_sim = &world.constraint_graph.colors[contact.color_index as usize]
                        .contact_sims[contact.local_index as usize];

                    let mut approach_speed = threshold;
                    let mut best_point: Option<usize> = None;
                    for p in 0..contact_sim.manifold.point_count as usize {
                        let mp = &contact_sim.manifold.points[p];
                        let point_approach_speed = -mp.normal_velocity;

                        // Need to check total impulse because the point may be
                        // speculative and not colliding
                        if point_approach_speed > approach_speed && mp.total_normal_impulse > 0.0 {
                            approach_speed = point_approach_speed;
                            best_point = Some(p);
                        }
                    }

                    if let Some(p) = best_point {
                        let best = contact_sim.manifold.points[p];
                        let normal = contact_sim.manifold.normal;

                        let shape_a = &world.shapes[contact_sim.shape_id_a as usize];
                        let shape_b = &world.shapes[contact_sim.shape_id_b as usize];

                        // World contact point reconstructed from a body center
                        // of mass and the matching anchor. The anchors were
                        // built with the manifold, so a body that has moved
                        // since drags the point with it. A static body has not
                        // moved, prefer one so the common case of a fast body
                        // striking the world stays exact.
                        let body_a = &world.bodies[shape_a.body_id as usize];
                        let body_b = &world.bodies[shape_b.body_id as usize];
                        let point = if body_a.type_ != BodyType::Static
                            && body_b.type_ == BodyType::Static
                        {
                            let body_sim_b = &world.solver_sets[body_b.set_index as usize]
                                .body_sims[body_b.local_index as usize];
                            offset_pos(body_sim_b.center, best.anchor_b)
                        } else {
                            let body_sim_a = &world.solver_sets[body_a.set_index as usize]
                                .body_sims[body_a.local_index as usize];
                            offset_pos(body_sim_a.center, best.anchor_a)
                        };

                        let event = ContactHitEvent {
                            shape_id_a: ShapeId {
                                index1: shape_a.id + 1,
                                world0: world_id,
                                generation: shape_a.generation,
                            },
                            shape_id_b: ShapeId {
                                index1: shape_b.id + 1,
                                world0: world_id,
                                generation: shape_b.generation,
                            },
                            contact_id: ContactId {
                                index1: contact.contact_id + 1,
                                world0: world_id,
                                padding: 0,
                                generation: contact.generation,
                            },
                            point,
                            normal,
                            approach_speed,
                        };

                        world.contact_hit_events.push(event);
                    }

                    // Clear the smallest set bit
                    word &= word - 1;
                }
            }
        }
        world.profile.hit_events = get_milliseconds(hit_ticks);
    }

    // === Refit broad phase ===
    {
        let refit_ticks = get_ticks();
        world.broad_phase.validate_no_enlarged();

        // Enlarge broad-phase proxies and build move array.
        // Apply shape AABB changes to broad-phase. This also creates the move
        // array which must be in deterministic order. Sim bodies are tracked
        // because the number of shape ids can be huge. This has to happen
        // before bullets are processed.
        let word_count = world.task_contexts[0].enlarged_sim_bit_set.block_count();
        for k in 0..word_count {
            let mut word = world.task_contexts[0].enlarged_sim_bit_set.block(k);
            while word != 0 {
                let ctz = word.trailing_zeros();
                let body_sim_index = (64 * k + ctz) as usize;

                let (body_id, sim_flags) = {
                    let body_sim = &world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index];
                    (body_sim.body_id, body_sim.flags)
                };

                let mut shape_id = world.bodies[body_id as usize].head_shape_id;
                if sim_flags
                    & (crate::body::body_flags::IS_BULLET | crate::body::body_flags::IS_FAST)
                    == (crate::body::body_flags::IS_BULLET | crate::body::body_flags::IS_FAST)
                {
                    // Fast bullet bodies don't have their final AABB yet
                    while shape_id != NULL_INDEX {
                        let proxy_key = world.shapes[shape_id as usize].proxy_key;

                        // Shape is fast. Its aabb will be enlarged in
                        // continuous collision. Update the move array here for
                        // determinism because bullets are processed below in
                        // non-deterministic order.
                        world.broad_phase.buffer_move(proxy_key);

                        shape_id = world.shapes[shape_id as usize].next_shape_id;
                    }
                } else {
                    while shape_id != NULL_INDEX {
                        // The AABB may not have been enlarged, despite the
                        // body being flagged as enlarged. For example, a body
                        // with multiple shapes may have not have all shapes
                        // enlarged. A fast body may have been flagged as
                        // enlarged despite having no shapes enlarged.
                        if world.shapes[shape_id as usize].enlarged_aabb {
                            let proxy_key = world.shapes[shape_id as usize].proxy_key;
                            let fat_aabb = world.shapes[shape_id as usize].fat_aabb;
                            world.broad_phase.enlarge_proxy(proxy_key, fat_aabb);
                            world.shapes[shape_id as usize].enlarged_aabb = false;
                        }

                        shape_id = world.shapes[shape_id as usize].next_shape_id;
                    }
                }

                // Clear the smallest set bit
                word &= word - 1;
            }
        }

        world.broad_phase.validate();
        world.profile.refit = get_milliseconds(refit_ticks);
    }

    // === Bullets ===
    if !bullet_bodies.is_empty() {
        let bullet_ticks = get_ticks();
        // Fast bullet bodies. Note: a bullet body may be moving slow.
        // (b2BulletBodyTask)
        for &sim_index in &bullet_bodies {
            super::continuous::solve_continuous(world, sim_index);
        }

        // Serially enlarge broad-phase proxies for bullet shapes.
        // This loop has non-deterministic order in C but it shouldn't affect
        // the result; the serial port follows the bullet array order.
        for &sim_index in &bullet_bodies {
            let (body_id, enlarge) = {
                let bullet_body_sim =
                    &world.solver_sets[AWAKE_SET as usize].body_sims[sim_index as usize];
                (
                    bullet_body_sim.body_id,
                    bullet_body_sim.flags & crate::body::body_flags::ENLARGE_BOUNDS != 0,
                )
            };
            if !enlarge {
                continue;
            }

            // Clear flag
            world.solver_sets[AWAKE_SET as usize].body_sims[sim_index as usize].flags &=
                !crate::body::body_flags::ENLARGE_BOUNDS;

            let mut shape_id = world.bodies[body_id as usize].head_shape_id;
            while shape_id != NULL_INDEX {
                if !world.shapes[shape_id as usize].enlarged_aabb {
                    shape_id = world.shapes[shape_id as usize].next_shape_id;
                    continue;
                }

                // Clear flag
                world.shapes[shape_id as usize].enlarged_aabb = false;

                let proxy_key = world.shapes[shape_id as usize].proxy_key;
                let proxy_id = crate::broad_phase::proxy_id(proxy_key);
                debug_assert!(crate::broad_phase::proxy_type(proxy_key) == BodyType::Dynamic);

                // all fast bullet shapes should already be in the move buffer
                debug_assert!(world.broad_phase.moved_proxies[BodyType::Dynamic as usize]
                    .get_bit(proxy_id as u32));

                let fat_aabb = world.shapes[shape_id as usize].fat_aabb;
                world.broad_phase.trees[BodyType::Dynamic as usize]
                    .enlarge_proxy(proxy_id, fat_aabb);

                shape_id = world.shapes[shape_id as usize].next_shape_id;
            }
        }
        world.profile.bullets = get_milliseconds(bullet_ticks);
    }

    // === Report sensor hits ===
    // This may include bullet sensor hits.
    {
        let sensor_hit_ticks = get_ticks();
        let hits = std::mem::take(&mut world.task_contexts[0].sensor_hits);
        for hit in hits {
            let sensor_index = world.shapes[hit.sensor_id as usize].sensor_index;
            let generation = world.shapes[hit.visitor_id as usize].generation;

            let shape_ref = crate::sensor::Visitor {
                shape_id: hit.visitor_id,
                generation,
            };
            world.sensors[sensor_index as usize].hits.push(shape_ref);
        }
        world.profile.sensor_hits = get_milliseconds(sensor_hit_ticks);
    }

    // === Island sleeping ===
    // This must be done last because putting islands to sleep invalidates the
    // enlarged body bits.
    if world.enable_sleep {
        let sleep_ticks = get_ticks();
        // Collect split island candidate for the next time step. No need to
        // split if sleeping is disabled.
        debug_assert!(world.split_island_id == NULL_INDEX);
        {
            let task_context = &world.task_contexts[0];
            if task_context.split_island_id != NULL_INDEX && task_context.split_sleep_time >= 0.0 {
                debug_assert!(task_context.split_sleep_time > 0.0);
                world.split_island_id = task_context.split_island_id;
            }
        }

        // Need to process in reverse because this moves islands to sleeping
        // solver sets.
        let count = world.solver_sets[AWAKE_SET as usize].island_sims.len();
        for island_index in (0..count).rev() {
            if world.task_contexts[0]
                .awake_island_bit_set
                .get_bit(island_index as u32)
            {
                // this island is still awake
                continue;
            }

            let island_id =
                world.solver_sets[AWAKE_SET as usize].island_sims[island_index].island_id;

            crate::solver_set::try_sleep_island(world, island_id);
        }

        world.validate_solver_sets();
        world.profile.sleep_islands = get_milliseconds(sleep_ticks);
    }
}
