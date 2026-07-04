// Ports of the body-focused subtests of test_world.c plus coverage of the
// b2Body_* public API.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::*;
use crate::geometry::{make_box, make_square};
use crate::id::BodyId;
use crate::math_functions::{abs_float, make_rot, to_pos, Vec2, PI};
use crate::shape::create_polygon_shape;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType, MotionLocks};
use crate::world::{world_step, World};

const BODY_COUNT: usize = 10;

// (test_world.c DestroyAllBodiesWorld) — create and destroy bodies while
// stepping.
#[test]
fn destroy_all_bodies_world() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut count = 0usize;
    let mut creating = true;

    let mut body_ids: Vec<BodyId> = Vec::new();
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let square = make_square(0.5);

    for _ in 0..(2 * BODY_COUNT + 10) {
        if creating {
            if count < BODY_COUNT {
                let body_id = create_body(&mut world, &body_def);

                let shape_def = default_shape_def();
                create_polygon_shape(&mut world, body_id, &shape_def, &square);
                body_ids.push(body_id);
                count += 1;
            } else {
                creating = false;
            }
        } else if count > 0 {
            destroy_body(&mut world, body_ids[count - 1]);
            body_ids.pop();
            count -= 1;
        }

        world_step(&mut world, 1.0 / 60.0, 3);
    }

    // (C: b2World_GetCounters().bodyCount == 0)
    assert_eq!(world.body_id_pool.id_count(), 0);
}

// (test_world.c TestIsValid)
#[test]
fn test_is_valid() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let body_def = default_body_def();

    let body_id1 = create_body(&mut world, &body_def);
    assert!(body_is_valid(&world, body_id1));

    let body_id2 = create_body(&mut world, &body_def);
    assert!(body_is_valid(&world, body_id2));

    destroy_body(&mut world, body_id1);
    assert!(!body_is_valid(&world, body_id1));

    destroy_body(&mut world, body_id2);
    assert!(!body_is_valid(&world, body_id2));
}

// Kinematics API: transforms, velocities, forces, and impulses behave like
// the C accessors.
#[test]
fn body_kinematics_api() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: 0.0 };
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 2.0, y: 3.0 });
    let body_id = create_body(&mut world, &body_def);

    let shape_def = default_shape_def();
    let box_poly = make_box(0.5, 0.5);
    create_polygon_shape(&mut world, body_id, &shape_def, &box_poly);

    // Point/vector conversions round trip.
    let world_point = body_get_world_point(&world, body_id, Vec2 { x: 1.0, y: 0.0 });
    assert!(abs_float(world_point.x as f32 - 3.0) < 1e-6);
    let local_point = body_get_local_point(&world, body_id, world_point);
    assert!(abs_float(local_point.x - 1.0) < 1e-6 && abs_float(local_point.y) < 1e-6);

    // Rotate and check vectors.
    body_set_transform(
        &mut world,
        body_id,
        to_pos(Vec2 { x: 2.0, y: 3.0 }),
        make_rot(0.5 * PI),
    );
    let world_vector = body_get_world_vector(&world, body_id, Vec2 { x: 1.0, y: 0.0 });
    assert!(abs_float(world_vector.x) < 1e-6 && abs_float(world_vector.y - 1.0) < 1e-6);

    // Velocities.
    body_set_linear_velocity(&mut world, body_id, Vec2 { x: 1.0, y: -2.0 });
    body_set_angular_velocity(&mut world, body_id, 0.25);
    assert_eq!(
        body_get_linear_velocity(&world, body_id),
        Vec2 { x: 1.0, y: -2.0 }
    );
    assert_eq!(body_get_angular_velocity(&world, body_id), 0.25);

    // Impulse at the center changes velocity by inv_mass * impulse.
    let mass = body_get_mass(&world, body_id);
    assert!(mass > 0.0);
    body_set_linear_velocity(&mut world, body_id, Vec2 { x: 0.0, y: 0.0 });
    body_apply_linear_impulse_to_center(&mut world, body_id, Vec2 { x: mass, y: 0.0 }, true);
    let v = body_get_linear_velocity(&world, body_id);
    assert!(abs_float(v.x - 1.0) < 1e-6);

    // Force accumulates on the sim and is cleared.
    body_apply_force_to_center(&mut world, body_id, Vec2 { x: 10.0, y: 0.0 }, true);
    body_apply_torque(&mut world, body_id, 5.0, true);
    body_clear_forces(&mut world, body_id);

    // Mass override then back to shapes.
    let auto_mass = body_get_mass_data(&world, body_id);
    body_set_mass_data(
        &mut world,
        body_id,
        crate::collision::MassData {
            mass: 2.0 * auto_mass.mass,
            center: auto_mass.center,
            rotational_inertia: 2.0 * auto_mass.rotational_inertia,
        },
    );
    assert_eq!(body_get_mass(&world, body_id), 2.0 * auto_mass.mass);
    body_apply_mass_from_shapes(&mut world, body_id);
    assert_eq!(body_get_mass(&world, body_id), auto_mass.mass);

    // Damping / gravity scale round trip.
    body_set_linear_damping(&mut world, body_id, 0.1);
    body_set_angular_damping(&mut world, body_id, 0.2);
    body_set_gravity_scale(&mut world, body_id, 0.5);
    assert_eq!(body_get_linear_damping(&world, body_id), 0.1);
    assert_eq!(body_get_angular_damping(&world, body_id), 0.2);
    assert_eq!(body_get_gravity_scale(&world, body_id), 0.5);

    // Name and user data.
    body_set_user_data(&mut world, body_id, 77);
    assert_eq!(body_get_user_data(&world, body_id), 77);
    body_set_name(&mut world, body_id, "kinematics");
    assert_eq!(
        body_get_name(&world, body_id).len(),
        crate::constants::NAME_LENGTH as usize
    );

    // AABB covers the box.
    let aabb = body_compute_aabb(&world, body_id);
    assert!(aabb.lower_bound.x < 2.0 && 2.0 < aabb.upper_bound.x);

    world.validate_solver_sets();
}

// State API: sleep, wake, disable/enable, type changes, locks, and flags.
#[test]
fn body_state_api() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    // Ground + a resting box that will fall asleep.
    let ground_def = default_body_def();
    let ground_id = create_body(&mut world, &ground_def);
    let shape_def = default_shape_def();
    create_polygon_shape(&mut world, ground_id, &shape_def, &make_box(20.0, 1.0));

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 0.0, y: 1.55 });
    let body_id = create_body(&mut world, &body_def);
    create_polygon_shape(&mut world, body_id, &shape_def, &make_box(0.5, 0.5));

    assert!(body_is_awake(&world, body_id));
    assert!(body_is_enabled(&world, body_id));
    assert!(body_is_sleep_enabled(&world, body_id));

    // Let it settle and sleep.
    for _ in 0..120 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }
    assert!(!body_is_awake(&world, body_id));

    // Wake it manually.
    body_set_awake(&mut world, body_id, true);
    assert!(body_is_awake(&world, body_id));

    // Force it back to sleep.
    body_set_awake(&mut world, body_id, false);
    assert!(!body_is_awake(&world, body_id));

    // Disable removes it from simulation; enable brings it back awake.
    body_disable(&mut world, body_id);
    assert!(!body_is_enabled(&world, body_id));
    world_step(&mut world, 1.0 / 60.0, 4);
    body_enable(&mut world, body_id);
    assert!(body_is_enabled(&world, body_id));
    assert!(body_is_awake(&world, body_id));

    // Motion locks zero the angular velocity.
    body_set_angular_velocity(&mut world, body_id, 3.0);
    body_set_motion_locks(
        &mut world,
        body_id,
        MotionLocks {
            linear_x: false,
            linear_y: false,
            angular_z: true,
        },
    );
    assert_eq!(body_get_angular_velocity(&world, body_id), 0.0);
    assert!(body_get_motion_locks(&world, body_id).angular_z);

    // Bullet + contact recycling flags round trip.
    body_set_bullet(&mut world, body_id, true);
    assert!(body_is_bullet(&world, body_id));
    body_enable_contact_recycling(&mut world, body_id, true);
    assert!(body_is_contact_recycling_enabled(&world, body_id));

    // Type change: dynamic -> static -> dynamic keeps the world consistent.
    body_set_type(&mut world, body_id, BodyType::Static);
    assert_eq!(body_get_type(&world, body_id), BodyType::Static);
    assert_eq!(body_get_mass(&world, body_id), 0.0);
    world_step(&mut world, 1.0 / 60.0, 4);

    body_set_type(&mut world, body_id, BodyType::Dynamic);
    assert_eq!(body_get_type(&world, body_id), BodyType::Dynamic);
    assert!(body_get_mass(&world, body_id) > 0.0);
    for _ in 0..30 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    // Shape/joint/contact accessors.
    assert_eq!(body_get_shape_count(&world, body_id), 1);
    assert_eq!(body_get_shapes(&world, body_id, 8).len(), 1);
    assert_eq!(body_get_joint_count(&world, body_id), 0);
    let contact_capacity = body_get_contact_capacity(&world, body_id);
    let contact_data = body_get_contact_data(&world, body_id, 8);
    assert!(contact_data.len() as i32 <= contact_capacity);
    assert!(!contact_data.is_empty(), "resting box should touch ground");
    assert!(contact_data[0].manifold.point_count > 0);

    world.validate_solver_sets();
    world.validate_contacts();
    world.validate_connectivity();
}
