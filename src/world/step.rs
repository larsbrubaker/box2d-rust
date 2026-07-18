// The world step from physics_world.c: b2World_Step. The narrow phase
// (b2Collide / b2CollideTask) lives in the sibling `collide` module.
//
// b2World_Step takes &mut World instead of a b2WorldId (no global world
// registry). Profiling timers match C (`b2GetTicks` / profile fields).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::collide::collide;
use super::World;
use crate::broad_phase::update_broad_phase_pairs;
use crate::math_functions::{is_valid_float, min_float};
use crate::sensor::overlap_sensors;
use crate::solver::{make_soft, solve, StepContext};
use crate::solver_set::STATIC_SET;
use crate::timer::{get_milliseconds, get_ticks};
use crate::types::BodyType;

/// Simulate a world for one time step. (b2World_Step — takes &mut World; the
/// C resolves the world from an id)
pub fn world_step(world: &mut World, time_step: f32, sub_step_count: i32) {
    debug_assert!(is_valid_float(time_step));
    debug_assert!(0 < sub_step_count);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    // (B2_REC(world, Step, ...))
    if let Some(mut rec) = world.recording.take() {
        let world_id = crate::id::WorldId {
            index1: world.world_id + 1,
            generation: world.generation,
        };
        crate::recording::write_step(&mut rec, world_id, time_step, sub_step_count);
        world.recording = Some(rec);
    }

    // Prepare to capture events. Ensure user does not access stale data if
    // there is an early return.
    world.body_move_events.clear();
    world.sensor_begin_events.clear();
    world.contact_begin_events.clear();
    world.contact_hit_events.clear();
    world.joint_events.clear();

    world.profile = super::Profile::default();

    world.locked = true;

    let step_ticks = get_ticks();

    {
        let static_shape_count = world.broad_phase.trees[BodyType::Static as usize].proxy_count();
        let dynamic_shape_count = world.broad_phase.trees[BodyType::Dynamic as usize].proxy_count();
        let static_body_count = world.solver_sets[STATIC_SET as usize].body_sims.len() as i32;
        // this includes kinematic bodies
        let total_body_count = world.body_id_pool.id_count();
        let total_contact_count = world.contact_id_pool.id_count();

        let c = &mut world.max_capacity;
        c.static_shape_count =
            crate::math_functions::max_int(c.static_shape_count, static_shape_count);
        c.dynamic_shape_count =
            crate::math_functions::max_int(c.dynamic_shape_count, dynamic_shape_count);
        c.static_body_count =
            crate::math_functions::max_int(c.static_body_count, static_body_count);
        c.dynamic_body_count = crate::math_functions::max_int(
            c.dynamic_body_count,
            total_body_count - static_body_count,
        );
        c.contact_count = crate::math_functions::max_int(c.contact_count, total_contact_count);
    }

    // Update collision pairs and create contacts
    {
        let pair_ticks = get_ticks();
        update_broad_phase_pairs(world);
        world.profile.pairs = get_milliseconds(pair_ticks);
    }

    let sub_step_count = crate::math_functions::max_int(1, sub_step_count);
    let mut context = StepContext {
        dt: time_step,
        sub_step_count,
        ..StepContext::default()
    };

    if time_step > 0.0 {
        context.inv_dt = 1.0 / time_step;
        context.h = time_step / sub_step_count as f32;
        context.inv_h = sub_step_count as f32 * context.inv_dt;
    } else {
        context.inv_dt = 0.0;
        context.h = 0.0;
        context.inv_h = 0.0;
    }

    world.inv_h = context.inv_h;
    world.inv_dt = context.inv_dt;

    // Hertz values get reduced for large time steps
    let contact_hertz = min_float(world.contact_hertz, 0.125 * context.inv_h);
    context.contact_softness = make_soft(contact_hertz, world.contact_damping_ratio, context.h);
    context.static_softness =
        make_soft(2.0 * contact_hertz, world.contact_damping_ratio, context.h);

    context.restitution_threshold = world.restitution_threshold;
    context.max_linear_velocity = world.max_linear_speed;
    context.contact_speed = world.contact_speed;
    context.enable_warm_starting = world.enable_warm_starting;

    // Narrow phase: update contacts
    {
        let collide_ticks = get_ticks();
        collide(world, &context);
        world.profile.collide = get_milliseconds(collide_ticks);
    }

    // Integrate velocities, solve velocity constraints, and integrate
    // positions.
    if time_step > 0.0 {
        let solve_ticks = get_ticks();
        solve(world, &context);
        world.profile.solve = get_milliseconds(solve_ticks);
    }

    // Update sensors
    {
        let sensor_ticks = get_ticks();
        overlap_sensors(world);
        world.profile.sensors = get_milliseconds(sensor_ticks);
    }

    world.profile.step = get_milliseconds(step_ticks);

    world.locked = false;

    // Swap end event array buffers
    world.end_event_array_index = 1 - world.end_event_array_index;
    world.sensor_end_events[world.end_event_array_index as usize].clear();
    world.contact_end_events[world.end_event_array_index as usize].clear();

    // Per-step StateHash + bounds growth for an active recording session.
    crate::recording::record_step_end(world);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{create_body, get_body_full_id, get_body_transform};
    use crate::geometry::make_box;
    use crate::math_functions::{abs_float, rot_get_angle, to_pos, Vec2};
    use crate::shape::create_polygon_shape;
    use crate::types::{default_body_def, default_shape_def, default_world_def};

    // Port of test_world.c HelloWorld: a dynamic box falls onto a large
    // static ground box and comes to rest at y ~= 1.
    #[test]
    fn hello_world() {
        // Construct a world object, which will hold and simulate the rigid
        // bodies.
        let mut world_def = default_world_def();
        world_def.gravity = Vec2 { x: 0.0, y: -10.0 };

        let mut world = World::new(&world_def);

        // Define the ground body.
        let mut ground_body_def = default_body_def();
        ground_body_def.position = to_pos(Vec2 { x: 0.0, y: -10.0 });

        let ground_id = create_body(&mut world, &ground_body_def);

        // Define the ground box shape. The extents are the half-widths of the
        // box.
        let ground_box = make_box(50.0, 10.0);

        // Add the box shape to the ground body.
        let ground_shape_def = default_shape_def();
        create_polygon_shape(&mut world, ground_id, &ground_shape_def, &ground_box);

        // Define the dynamic body. We set its position and call the body
        // factory.
        let mut body_def = default_body_def();
        body_def.type_ = crate::types::BodyType::Dynamic;
        body_def.position = to_pos(Vec2 { x: 0.0, y: 4.0 });

        let body_id = create_body(&mut world, &body_def);
        let body_index = get_body_full_id(&world, body_id);

        // Define another box shape for our dynamic body.
        let dynamic_box = make_box(1.0, 1.0);

        // Define the dynamic body shape. Set the box density to be non-zero,
        // so it will be dynamic. Override the default friction.
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.material.friction = 0.3;

        create_polygon_shape(&mut world, body_id, &shape_def, &dynamic_box);

        // Prepare for simulation. Typically we use a time step of 1/60 of a
        // second (60Hz) and 4 sub-steps. This provides a high quality
        // simulation in most game scenarios.
        let time_step = 1.0 / 60.0;
        let sub_step_count = 4;

        // This is our little game loop.
        for _ in 0..90 {
            // Instruct the world to perform a single step of simulation. It is
            // generally best to keep the time step and iterations fixed.
            world_step(&mut world, time_step, sub_step_count);
        }

        let transform = get_body_transform(&world, body_index);
        let position = transform.p;
        let rotation = transform.q;

        assert!(abs_float(position.x as f32) < 0.01);
        assert!(abs_float(position.y as f32 - 1.00) < 0.01);
        assert!(abs_float(rot_get_angle(rotation)) < 0.01);

        // Profile timings are filled each step (timer.c was dropped earlier).
        let profile = crate::world::world_get_profile(&world);
        assert!(profile.step > 0.0);
        assert!(profile.pairs >= 0.0);
        assert!(profile.collide >= 0.0);
        assert!(profile.solve >= 0.0);
        assert!(profile.sensors >= 0.0);
    }

    // Port of test_world.c EmptyWorld: stepping an empty world does nothing.
    #[test]
    fn empty_world() {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let time_step = 1.0 / 60.0;
        let sub_step_count = 1;

        for _ in 0..60 {
            world_step(&mut world, time_step, sub_step_count);
        }

        // b2Solve counts the step before the awake-body early out
        assert_eq!(world.step_index, 60);
    }
}
