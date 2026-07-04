// Ports the shape-focused subtest of test_world.c (ChainSegmentShapeTest)
// plus acceptance coverage for the b2Shape_* / b2Chain_* public API, which
// has no dedicated C test file.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_get_linear_velocity, body_get_position, create_body};
use crate::collision::{Capsule, ChainSegment, Circle, Segment, ShapeType};
use crate::core::NULL_INDEX;
use crate::geometry::make_box;
use crate::math_functions::{to_pos, Vec2};
use crate::shape::*;
use crate::types::{
    default_body_def, default_chain_def, default_query_filter, default_shape_def,
    default_surface_material, default_world_def, BodyType,
};
use crate::world::{world_cast_ray_closest, world_step, World};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

// (test_world.c ChainSegmentShapeTest) — a stand-alone chain segment supports
// a resting circle and can be reshaped or created by converting another type.
#[test]
fn chain_segment_shape() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    let body_def = default_body_def();
    let ground_id = create_body(&mut world, &body_def);

    let cs = ChainSegment {
        ghost1: Vec2 { x: 2.0, y: 0.0 },
        segment: Segment {
            point1: Vec2 { x: 1.0, y: 0.0 },
            point2: Vec2 { x: -1.0, y: 0.0 },
        },
        ghost2: Vec2 { x: -2.0, y: 0.0 },
        chain_id: 99,
    };

    let shape_def = default_shape_def();
    let orphan_shape = create_chain_segment_shape(&mut world, ground_id, &shape_def, &cs);
    assert!(orphan_shape.index1 != 0, "orphan shape must be non-null");

    assert_eq!(
        shape_get_type(&world, orphan_shape),
        ShapeType::ChainSegment
    );

    let parent_chain = shape_get_parent_chain(&world, orphan_shape);
    assert!(parent_chain.index1 == 0, "orphan has no parent chain");

    let got = shape_get_chain_segment(&world, orphan_shape);
    ensure_small(got.ghost1.x - cs.ghost1.x, 1e-5);
    ensure_small(got.ghost1.y - cs.ghost1.y, 1e-5);
    ensure_small(got.segment.point1.x - cs.segment.point1.x, 1e-5);
    ensure_small(got.segment.point1.y - cs.segment.point1.y, 1e-5);
    ensure_small(got.segment.point2.x - cs.segment.point2.x, 1e-5);
    ensure_small(got.segment.point2.y - cs.segment.point2.y, 1e-5);
    ensure_small(got.ghost2.x - cs.ghost2.x, 1e-5);
    ensure_small(got.ghost2.y - cs.ghost2.y, 1e-5);
    assert_eq!(got.chain_id, NULL_INDEX);

    let mut dynamic_def = default_body_def();
    dynamic_def.type_ = BodyType::Dynamic;
    dynamic_def.position = to_pos(Vec2 { x: 0.0, y: 2.0 });
    let circle_body_id = create_body(&mut world, &dynamic_def);
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.5,
    };
    let circle_shape_def = default_shape_def();
    create_circle_shape(&mut world, circle_body_id, &circle_shape_def, &circle);

    for _ in 0..120 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    let circle_pos = body_get_position(&world, circle_body_id);
    assert!(circle_pos.y > 0.0, "circle rests on the chain segment");

    let cs2 = ChainSegment {
        ghost1: Vec2 { x: 3.0, y: 0.0 },
        segment: Segment {
            point1: Vec2 { x: 2.0, y: 0.0 },
            point2: Vec2 { x: -2.0, y: 0.0 },
        },
        ghost2: Vec2 { x: -3.0, y: 0.0 },
        chain_id: NULL_INDEX,
    };

    shape_set_chain_segment(&mut world, orphan_shape, &cs2);

    let got2 = shape_get_chain_segment(&world, orphan_shape);
    ensure_small(got2.segment.point1.x - cs2.segment.point1.x, 1e-5);
    ensure_small(got2.segment.point1.y - cs2.segment.point1.y, 1e-5);
    ensure_small(got2.segment.point2.x - cs2.segment.point2.x, 1e-5);
    ensure_small(got2.segment.point2.y - cs2.segment.point2.y, 1e-5);
    ensure_small(got2.ghost1.x - cs2.ghost1.x, 1e-5);
    ensure_small(got2.ghost1.y - cs2.ghost1.y, 1e-5);
    ensure_small(got2.ghost2.x - cs2.ghost2.x, 1e-5);
    ensure_small(got2.ghost2.y - cs2.ghost2.y, 1e-5);
    assert_eq!(got2.chain_id, NULL_INDEX);

    let parent_chain2 = shape_get_parent_chain(&world, orphan_shape);
    assert!(parent_chain2.index1 == 0);

    // Convert a circle shape into a chain segment.
    let conv_body = create_body(&mut world, &body_def);
    let conv_circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.25,
    };
    let conv_shape = create_circle_shape(&mut world, conv_body, &shape_def, &conv_circle);
    assert_eq!(shape_get_type(&world, conv_shape), ShapeType::Circle);

    shape_set_chain_segment(&mut world, conv_shape, &cs2);

    assert_eq!(shape_get_type(&world, conv_shape), ShapeType::ChainSegment);
    let got3 = shape_get_chain_segment(&world, conv_shape);
    ensure_small(got3.ghost1.x - cs2.ghost1.x, 1e-5);
    ensure_small(got3.ghost1.y - cs2.ghost1.y, 1e-5);
    ensure_small(got3.ghost2.x - cs2.ghost2.x, 1e-5);
    ensure_small(got3.ghost2.y - cs2.ghost2.y, 1e-5);
    assert_eq!(got3.chain_id, NULL_INDEX);

    let parent_chain3 = shape_get_parent_chain(&world, conv_shape);
    assert!(parent_chain3.index1 == 0);

    destroy_shape(&mut world, orphan_shape, true);
}

// Acceptance test for b2CreateChain/b2DestroyChain and the b2Chain_* API: an
// open chain floor supports a resting box; loop chains create one segment per
// point.
#[test]
fn chain_create_and_destroy() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    let body_def = default_body_def();
    let ground_id = create_body(&mut world, &body_def);

    // Open chain: n points make n - 3 solid segments (the ends are ghosts).
    // Segments are one-sided; winding right-to-left keeps the solid side up.
    let mut chain_def = default_chain_def();
    chain_def.points = vec![
        Vec2 { x: 6.0, y: 0.0 },
        Vec2 { x: 5.0, y: 0.0 },
        Vec2 { x: -5.0, y: 0.0 },
        Vec2 { x: -6.0, y: 0.0 },
    ];
    let chain_id = create_chain(&mut world, ground_id, &chain_def);
    assert!(chain_is_valid(&world, chain_id));
    assert_eq!(chain_get_segment_count(&world, chain_id), 1);

    let segments = chain_get_segments(&world, chain_id, 8);
    assert_eq!(segments.len(), 1);
    assert_eq!(
        shape_get_type(&world, segments[0]),
        ShapeType::ChainSegment
    );
    assert_eq!(shape_get_parent_chain(&world, segments[0]), chain_id);

    // Chain materials propagate to the segment shapes.
    assert_eq!(chain_get_surface_material_count(&world, chain_id), 1);
    let mut material = default_surface_material();
    material.friction = 0.9;
    chain_set_surface_material(&mut world, chain_id, material, 0);
    assert_eq!(shape_get_friction(&world, segments[0]), 0.9);
    assert_eq!(
        chain_get_surface_material(&world, chain_id, 0).friction,
        0.9
    );

    // A box dropped on the chain comes to rest above it.
    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = to_pos(Vec2 { x: 0.0, y: 2.0 });
    let box_id = create_body(&mut world, &box_def);
    let shape_def = default_shape_def();
    create_polygon_shape(&mut world, box_id, &shape_def, &make_box(0.5, 0.5));

    for _ in 0..120 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }
    assert!(body_get_position(&world, box_id).y > 0.0);

    // Loop chain: n points make n solid segments.
    let loop_body = create_body(&mut world, &body_def);
    let mut loop_def = default_chain_def();
    loop_def.is_loop = true;
    loop_def.points = vec![
        Vec2 { x: 10.0, y: 0.0 },
        Vec2 { x: 12.0, y: 0.0 },
        Vec2 { x: 12.0, y: 2.0 },
        Vec2 { x: 10.0, y: 2.0 },
    ];
    let loop_id = create_chain(&mut world, loop_body, &loop_def);
    assert_eq!(chain_get_segment_count(&world, loop_id), 4);

    // Destroying the chain destroys its segments.
    let shape_count_before = world.shape_id_pool.id_count();
    destroy_chain(&mut world, loop_id);
    assert!(!chain_is_valid(&world, loop_id));
    assert_eq!(world.shape_id_pool.id_count(), shape_count_before - 4);

    destroy_chain(&mut world, chain_id);
    assert!(!chain_is_valid(&world, chain_id));

    world_step(&mut world, 1.0 / 60.0, 4);
    world.validate_solver_sets();
    world.validate_contacts();
}

// Acceptance coverage for the b2Shape_* accessors against exact geometry.
#[test]
fn shape_accessors() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 1.0, y: 2.0 });
    body_def.gravity_scale = 0.0;
    let body_id = create_body(&mut world, &body_def);

    let shape_def = default_shape_def();
    let box_poly = make_box(0.5, 0.5);
    let shape_id = create_polygon_shape(&mut world, body_id, &shape_def, &box_poly);

    assert!(shape_is_valid(&world, shape_id));
    assert_eq!(shape_get_body(&world, shape_id), body_id);
    assert!(!shape_is_sensor(&world, shape_id));

    // User data and materials round trip.
    shape_set_user_data(&mut world, shape_id, 42);
    assert_eq!(shape_get_user_data(&world, shape_id), 42);
    shape_set_friction(&mut world, shape_id, 0.7);
    assert_eq!(shape_get_friction(&world, shape_id), 0.7);
    shape_set_restitution(&mut world, shape_id, 0.4);
    assert_eq!(shape_get_restitution(&world, shape_id), 0.4);
    shape_set_user_material(&mut world, shape_id, 7);
    assert_eq!(shape_get_user_material(&world, shape_id), 7);
    let material = shape_get_surface_material(&world, shape_id);
    assert_eq!(material.friction, 0.7);
    shape_set_surface_material(&mut world, shape_id, material);

    // Density change updates body mass.
    let mass_data_before = shape_compute_mass_data(&world, shape_id);
    shape_set_density(&mut world, shape_id, 2.0, true);
    assert_eq!(shape_get_density(&world, shape_id), 2.0);
    let mass_data_after = shape_compute_mass_data(&world, shape_id);
    assert_eq!(mass_data_after.mass, 2.0 * mass_data_before.mass);

    // Event flags round trip.
    shape_enable_sensor_events(&mut world, shape_id, true);
    assert!(shape_are_sensor_events_enabled(&world, shape_id));
    shape_enable_contact_events(&mut world, shape_id, false);
    assert!(!shape_are_contact_events_enabled(&world, shape_id));
    shape_enable_pre_solve_events(&mut world, shape_id, true);
    assert!(shape_are_pre_solve_events_enabled(&world, shape_id));
    shape_enable_hit_events(&mut world, shape_id, true);
    assert!(shape_are_hit_events_enabled(&world, shape_id));

    // Point tests in world space: body center is inside, far point is not.
    assert!(shape_test_point(
        &world,
        shape_id,
        to_pos(Vec2 { x: 1.0, y: 2.0 })
    ));
    assert!(!shape_test_point(
        &world,
        shape_id,
        to_pos(Vec2 { x: 3.0, y: 2.0 })
    ));

    // Direct ray cast hits the left face at x = 0.5.
    let output = shape_ray_cast(
        &world,
        shape_id,
        to_pos(Vec2 { x: -2.0, y: 2.0 }),
        Vec2 { x: 5.0, y: 0.0 },
    );
    assert!(output.hit);
    ensure_small(output.point.x as f32 - 0.5, 1e-5);

    // AABB covers the box.
    let aabb = shape_get_aabb(&world, shape_id);
    assert!(aabb.lower_bound.x < 0.6 && 1.4 < aabb.upper_bound.x);

    // Closest point from the left lands on the left face.
    let closest = shape_get_closest_point(&world, shape_id, to_pos(Vec2 { x: -2.0, y: 2.0 }));
    ensure_small(closest.x as f32 - 0.5, 1e-3);
    ensure_small(closest.y as f32 - 2.0, 1e-3);

    // Filter set/get (identical filter early-outs, new one sticks).
    let mut filter = shape_get_filter(&world, shape_id);
    shape_set_filter(&mut world, shape_id, filter);
    filter.group_index = -3;
    shape_set_filter(&mut world, shape_id, filter);
    assert_eq!(shape_get_filter(&world, shape_id).group_index, -3);

    // Geometry conversions: polygon -> circle -> capsule -> segment.
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.5,
    };
    shape_set_circle(&mut world, shape_id, &circle);
    assert_eq!(shape_get_type(&world, shape_id), ShapeType::Circle);
    assert_eq!(shape_get_circle(&world, shape_id).radius, 0.5);

    let capsule = Capsule {
        center1: Vec2 { x: -0.5, y: 0.0 },
        center2: Vec2 { x: 0.5, y: 0.0 },
        radius: 0.25,
    };
    shape_set_capsule(&mut world, shape_id, &capsule);
    assert_eq!(shape_get_type(&world, shape_id), ShapeType::Capsule);
    assert_eq!(shape_get_capsule(&world, shape_id).radius, 0.25);

    let segment = Segment {
        point1: Vec2 { x: -1.0, y: 0.0 },
        point2: Vec2 { x: 1.0, y: 0.0 },
    };
    shape_set_segment(&mut world, shape_id, &segment);
    assert_eq!(shape_get_type(&world, shape_id), ShapeType::Segment);
    assert_eq!(shape_get_segment(&world, shape_id).point1.x, -1.0);

    let polygon = make_box(0.5, 0.5);
    shape_set_polygon(&mut world, shape_id, &polygon);
    assert_eq!(shape_get_type(&world, shape_id), ShapeType::Polygon);
    assert_eq!(shape_get_polygon(&world, shape_id).count, 4);

    // World ray cast still sees the reshaped body where it stands.
    let ray = world_cast_ray_closest(
        &world,
        to_pos(Vec2 { x: -5.0, y: 2.0 }),
        Vec2 { x: 10.0, y: 0.0 },
        default_query_filter(),
    );
    assert!(ray.hit);

    // Wind on a dynamic body adds force that integrates into velocity.
    shape_apply_wind(&mut world, shape_id, Vec2 { x: 50.0, y: 0.0 }, 1.0, 0.0, true);
    world_step(&mut world, 1.0 / 60.0, 4);
    let velocity = body_get_linear_velocity(&world, body_id);
    assert!(velocity.x > 0.0, "wind pushes the body along +x");

    world.validate_solver_sets();
}

// Sensor introspection: b2Shape_GetSensorCapacity/GetSensorData see the
// overlapping shape after a step.
#[test]
fn sensor_data_accessors() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: 0.0 };
    let mut world = World::new(&world_def);

    let mut sensor_body_def = default_body_def();
    sensor_body_def.position = to_pos(Vec2 { x: 0.0, y: 0.0 });
    let sensor_body = create_body(&mut world, &sensor_body_def);
    let mut sensor_shape_def = default_shape_def();
    sensor_shape_def.is_sensor = true;
    sensor_shape_def.enable_sensor_events = true;
    let sensor_shape =
        create_polygon_shape(&mut world, sensor_body, &sensor_shape_def, &make_box(1.0, 1.0));
    assert!(shape_is_sensor(&world, sensor_shape));
    assert_eq!(shape_get_contact_capacity(&world, sensor_shape), 0);

    let mut visitor_def = default_body_def();
    visitor_def.type_ = BodyType::Dynamic;
    visitor_def.gravity_scale = 0.0;
    visitor_def.position = to_pos(Vec2 { x: 0.5, y: 0.0 });
    let visitor_body = create_body(&mut world, &visitor_def);
    let mut visitor_shape_def = default_shape_def();
    visitor_shape_def.enable_sensor_events = true;
    let visitor_shape = create_polygon_shape(
        &mut world,
        visitor_body,
        &visitor_shape_def,
        &make_box(0.25, 0.25),
    );

    world_step(&mut world, 1.0 / 60.0, 4);

    assert_eq!(shape_get_sensor_capacity(&world, sensor_shape), 1);
    let visitors = shape_get_sensor_data(&world, sensor_shape, 8);
    assert_eq!(visitors.len(), 1);
    assert_eq!(visitors[0], visitor_shape);
    assert!(shape_is_valid(&world, visitors[0]));
}
