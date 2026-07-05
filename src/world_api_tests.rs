// Ports of the world-focused subtests of test_world.c plus acceptance
// coverage of the world query API (physics_world.c has no dedicated C tests
// for the queries; the expectations below are derived from exact geometry).
//
// Not ported: TestWorldRecycle and TestSetWorkerCount exercise the global
// world registry and the task system, neither of which exist in the
// registry-less serial Rust port.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::*;
use crate::collision::{Capsule, Circle};
use crate::geometry::{make_box, make_square};
use crate::id::ShapeId;
use crate::math_functions::{abs_float, to_pos, Pos, Vec2};
use crate::shape::{create_circle_shape, create_polygon_shape};
use crate::types::{
    default_body_def, default_explosion_def, default_query_filter, default_shape_def,
    default_world_def, BodyType, MotionLocks,
};
use crate::world::*;

fn custom_filter(_shape_a: ShapeId, _shape_b: ShapeId, _context: u64) -> bool {
    true
}

fn pre_solve_static(
    _shape_a: ShapeId,
    _shape_b: ShapeId,
    _point: Pos,
    _normal: Vec2,
    _context: u64,
) -> bool {
    true
}

// (test_world.c TestWorldCoverage)
#[test]
fn test_world_coverage() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);
    assert!(world_is_valid(&world));

    world_enable_sleeping(&mut world, true);
    world_enable_sleeping(&mut world, false);
    assert!(!world_is_sleeping_enabled(&world));

    world_enable_continuous(&mut world, false);
    world_enable_continuous(&mut world, true);
    assert!(world_is_continuous_enabled(&world));

    world_set_restitution_threshold(&mut world, 0.0);
    world_set_restitution_threshold(&mut world, 2.0);
    assert_eq!(world_get_restitution_threshold(&world), 2.0);

    world_set_hit_event_threshold(&mut world, 0.0);
    world_set_hit_event_threshold(&mut world, 100.0);
    assert_eq!(world_get_hit_event_threshold(&world), 100.0);

    world_set_custom_filter_callback(&mut world, Some(custom_filter), 0);
    world_set_pre_solve_callback(&mut world, Some(pre_solve_static), 0);

    let g = Vec2 { x: 1.0, y: 2.0 };
    world_set_gravity(&mut world, g);
    let v = world_get_gravity(&world);
    assert_eq!(v.x, g.x);
    assert_eq!(v.y, g.y);

    let explosion_def = default_explosion_def();
    world_explode(&mut world, &explosion_def);

    world_set_contact_tuning(&mut world, 10.0, 2.0, 4.0);

    world_set_maximum_linear_speed(&mut world, 10.0);
    assert_eq!(world_get_maximum_linear_speed(&world), 10.0);

    world_enable_warm_starting(&mut world, true);
    assert!(world_is_warm_starting_enabled(&world));

    assert_eq!(world_get_awake_body_count(&world), 0);

    world_set_user_data(&mut world, 77);
    assert_eq!(world_get_user_data(&world), 77);

    world_step(&mut world, 1.0, 1);
}

// (test_world.c TestSensor) — a bullet sensor flies through a tall wall and
// must report exactly one begin and one end event.
#[test]
fn test_sensor() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    // Wall from x = 1 to x = 2
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = to_pos(Vec2 { x: 1.5, y: 11.0 });
    let wall_id = create_body(&mut world, &body_def);
    let box_poly = make_box(0.5, 10.0);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    create_polygon_shape(&mut world, wall_id, &shape_def, &box_poly);

    // Bullet fired towards the wall
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.is_bullet = true;
    body_def.gravity_scale = 0.0;
    body_def.position = to_pos(Vec2 { x: 7.39814, y: 4.0 });
    body_def.linear_velocity = Vec2 { x: -20.0, y: 0.0 };
    let bullet_id = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.1,
    };
    create_circle_shape(&mut world, bullet_id, &shape_def, &circle);

    let mut begin_count = 0;
    let mut end_count = 0;

    loop {
        let time_step = 1.0 / 60.0;
        let sub_step_count = 4;
        world_step(&mut world, time_step, sub_step_count);

        let bullet_pos = body_get_position(&world, bullet_id);

        let events = world_get_sensor_events(&world);

        if !events.begin_events.is_empty() {
            begin_count += 1;
        }

        if !events.end_events.is_empty() {
            end_count += 1;
        }

        if (bullet_pos.x as f32) < -1.0 {
            break;
        }
    }

    assert_eq!(begin_count, 1);
    assert_eq!(end_count, 1);
}

// (test_world.c DeferredMassFlagSyncTest)
#[test]
fn deferred_mass_flag_sync() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_id = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.update_body_mass = false;

    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.5,
    };
    create_circle_shape(&mut world, body_id, &shape_def, &circle);

    body_apply_mass_from_shapes(&mut world, body_id);

    world_step(&mut world, 1.0 / 60.0, 4);
}

// (test_world.c EnableSleepFlagSyncTest)
#[test]
fn enable_sleep_flag_sync() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = false;
    let body_id = create_body(&mut world, &body_def);

    assert!(!body_is_sleep_enabled(&world, body_id));

    body_enable_sleep(&mut world, body_id, true);
    assert!(body_is_sleep_enabled(&world, body_id));

    world_step(&mut world, 1.0 / 60.0, 4);
}

// (test_world.c EnableContactRecyclingTest)
#[test]
fn enable_contact_recycling() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    // Default is enabled
    let body_a = create_body(&mut world, &body_def);
    assert!(body_is_contact_recycling_enabled(&world, body_a));

    body_enable_contact_recycling(&mut world, body_a, false);
    assert!(!body_is_contact_recycling_enabled(&world, body_a));

    body_enable_contact_recycling(&mut world, body_a, true);
    assert!(body_is_contact_recycling_enabled(&world, body_a));

    // Per-def opt-out at creation
    body_def.enable_contact_recycling = false;
    let body_b = create_body(&mut world, &body_def);
    assert!(!body_is_contact_recycling_enabled(&world, body_b));

    // Stepping after toggling must not trip the flag-sync validator
    world_step(&mut world, 1.0 / 60.0, 4);
}

// (test_world.c SetBulletDriftTest)
#[test]
fn set_bullet_drift() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = false;
        let body_id = create_body(&mut world, &body_def);

        assert!(!body_is_bullet(&world, body_id));

        body_set_bullet(&mut world, body_id, true);
        assert!(body_is_bullet(&world, body_id));

        let locks = MotionLocks {
            linear_x: true,
            linear_y: false,
            angular_z: false,
        };
        body_set_motion_locks(&mut world, body_id, locks);

        assert!(body_is_bullet(&world, body_id));
    }

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        let body_id = create_body(&mut world, &body_def);

        assert!(body_is_bullet(&world, body_id));

        body_set_bullet(&mut world, body_id, false);
        assert!(!body_is_bullet(&world, body_id));

        let locks = MotionLocks {
            linear_x: true,
            linear_y: false,
            angular_z: false,
        };
        body_set_motion_locks(&mut world, body_id, locks);

        assert!(!body_is_bullet(&world, body_id));
    }
}

// Acceptance test for the world query API against exact geometry: a static
// 2x2 box centered at the origin.
#[test]
fn world_queries() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let body_def = default_body_def();
    let body_id = create_body(&mut world, &body_def);
    let shape_def = default_shape_def();
    let box_poly = make_box(1.0, 1.0);
    create_polygon_shape(&mut world, body_id, &shape_def, &box_poly);

    let filter = default_query_filter();

    // World bounds contain the (fattened) box.
    let bounds = world_get_bounds(&world);
    assert!(bounds.lower_bound.x <= -1.0 && 1.0 <= bounds.upper_bound.x);
    assert!(bounds.lower_bound.y <= -1.0 && 1.0 <= bounds.upper_bound.y);

    // Overlap AABB on the box finds it; far away finds nothing.
    let mut count = 0;
    world_overlap_aabb(
        &mut world,
        to_pos(Vec2 { x: 0.0, y: 0.0 }),
        crate::math_functions::Aabb {
            lower_bound: Vec2 { x: -0.5, y: -0.5 },
            upper_bound: Vec2 { x: 0.5, y: 0.5 },
        },
        filter,
        |_| {
            count += 1;
            true
        },
    );
    assert_eq!(count, 1);

    count = 0;
    world_overlap_aabb(
        &mut world,
        to_pos(Vec2 { x: 100.0, y: 0.0 }),
        crate::math_functions::Aabb {
            lower_bound: Vec2 { x: -0.5, y: -0.5 },
            upper_bound: Vec2 { x: 0.5, y: 0.5 },
        },
        filter,
        |_| {
            count += 1;
            true
        },
    );
    assert_eq!(count, 0);

    // Overlap shape: a small circle proxy centered on the box overlaps it.
    let circle_proxy = crate::distance::make_proxy(&[Vec2 { x: 0.0, y: 0.0 }], 0.25);
    count = 0;
    world_overlap_shape(
        &mut world,
        to_pos(Vec2 { x: 0.0, y: 0.0 }),
        &circle_proxy,
        filter,
        |_| {
            count += 1;
            true
        },
    );
    assert_eq!(count, 1);

    // Closest ray cast from the left hits the x = -1 face at fraction 0.4.
    let result = world_cast_ray_closest(
        &mut world,
        to_pos(Vec2 { x: -5.0, y: 0.0 }),
        Vec2 { x: 10.0, y: 0.0 },
        filter,
    );
    assert!(result.hit);
    assert!(abs_float(result.fraction - 0.4) < 1e-5);
    assert!(abs_float(result.point.x as f32 + 1.0) < 1e-5);
    assert!(abs_float(result.normal.x + 1.0) < 1e-5);

    // Callback-form ray cast sees the same single hit.
    let mut hits = 0;
    world_cast_ray(
        &mut world,
        to_pos(Vec2 { x: -5.0, y: 0.0 }),
        Vec2 { x: 10.0, y: 0.0 },
        filter,
        |_, _, _, fraction| {
            hits += 1;
            fraction
        },
    );
    assert_eq!(hits, 1);

    // Shape cast: the circle proxy stops when its surface touches the face,
    // fraction = (4 - 0.25) / 10. The cast solver converges to within a
    // linear-slop sized tolerance of the surface.
    let mut cast_fraction = 1.0f32;
    world_cast_shape(
        &mut world,
        to_pos(Vec2 { x: -5.0, y: 0.0 }),
        &circle_proxy,
        Vec2 { x: 10.0, y: 0.0 },
        filter,
        |_, _, _, fraction| {
            cast_fraction = fraction;
            fraction
        },
    );
    assert!(abs_float(cast_fraction - 0.375) < 1e-2);

    // Mover cast: a capsule mover stops near the face as well.
    let mover = Capsule {
        center1: Vec2 { x: 0.0, y: 0.0 },
        center2: Vec2 { x: 0.0, y: 0.5 },
        radius: 0.1,
    };
    let mover_fraction = world_cast_mover(
        &mut world,
        to_pos(Vec2 { x: -5.0, y: 0.0 }),
        &mover,
        Vec2 { x: 10.0, y: 0.0 },
        filter,
    );
    assert!(abs_float(mover_fraction - 0.39) < 1e-2);

    // Collide mover: standing just off the face produces a collision plane
    // pushing away from the box (-x).
    let mut plane_count = 0;
    let mut plane_normal_x = 0.0f32;
    world_collide_mover(
        &mut world,
        to_pos(Vec2 { x: -1.05, y: 0.0 }),
        &mover,
        filter,
        |_, plane| {
            plane_count += 1;
            plane_normal_x = plane.plane.normal.x;
            true
        },
    );
    assert_eq!(plane_count, 1);
    assert!(plane_normal_x < 0.0);

    // Counters see one static body and one shape.
    let counters = world_get_counters(&world);
    assert_eq!(counters.body_count, 1);
    assert_eq!(counters.shape_count, 1);
    assert_eq!(counters.contact_count, 0);
    assert_eq!(counters.joint_count, 0);
}

// Acceptance test for world_explode against exact scaling: a free dynamic box
// in the falloff band receives an outward impulse.
#[test]
fn explosion_applies_impulse() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: 0.0 };
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 3.0, y: 0.0 });
    let body_id = create_body(&mut world, &body_def);
    let shape_def = default_shape_def();
    create_polygon_shape(&mut world, body_id, &shape_def, &make_square(0.5));

    let mut explosion_def = default_explosion_def();
    explosion_def.radius = 2.0;
    explosion_def.falloff = 2.0;
    explosion_def.impulse_per_length = 10.0;
    world_explode(&mut world, &explosion_def);

    // The closest point (x = 2.5) is inside the falloff band, direction +x.
    let v = body_get_linear_velocity(&world, body_id);
    assert!(v.x > 0.0);
    assert!(abs_float(v.y) < 1e-6);
    assert_eq!(body_get_angular_velocity(&world, body_id), 0.0);

    // Out of range: no impulse.
    let mut far_def = default_body_def();
    far_def.type_ = BodyType::Dynamic;
    far_def.position = to_pos(Vec2 { x: 100.0, y: 0.0 });
    let far_id = create_body(&mut world, &far_def);
    create_polygon_shape(&mut world, far_id, &shape_def, &make_square(0.5));
    world_explode(&mut world, &explosion_def);
    let far_v = body_get_linear_velocity(&world, far_id);
    assert_eq!(far_v.x, 0.0);
}
