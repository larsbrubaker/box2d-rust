// Coverage of the b2Contact_* id-keyed public API (IsValid / GetData).
// There is no dedicated C unit test for these; they are exercised by samples.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_get_contact_data, create_body, destroy_body};
use crate::contact::{contact_get_data, contact_is_valid, destroy_contact};
use crate::geometry::{make_box, make_square};
use crate::id::ContactId;
use crate::math_functions::{to_pos, Vec2};
use crate::shape::create_polygon_shape;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{world_step, World};

#[test]
fn contact_is_valid_rejects_null_and_orphaned_ids() {
    let mut world = World::new(&default_world_def());

    assert!(!contact_is_valid(
        &world,
        ContactId {
            index1: 0,
            world0: world.world_id,
            padding: 0,
            generation: 0,
        }
    ));
    assert!(!contact_is_valid(
        &world,
        ContactId {
            index1: 1,
            world0: world.world_id,
            padding: 0,
            generation: 0,
        }
    ));

    // Build a resting contact, then destroy it and confirm the id goes stale.
    let mut ground_def = default_body_def();
    ground_def.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
    let ground = create_body(&mut world, &ground_def);
    create_polygon_shape(
        &mut world,
        ground,
        &default_shape_def(),
        &make_box(10.0, 1.0),
    );

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 0.0, y: 1.0 });
    let body = create_body(&mut world, &body_def);
    create_polygon_shape(&mut world, body, &default_shape_def(), &make_square(0.5));

    for _ in 0..30 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    let data = body_get_contact_data(&world, body, 8);
    assert!(!data.is_empty());
    let contact_id = data[0].contact_id;
    assert!(contact_is_valid(&world, contact_id));

    let contact_index = contact_id.index1 - 1;
    destroy_contact(&mut world, contact_index, false);
    assert!(!contact_is_valid(&world, contact_id));

    // Wrong generation on an empty world is also invalid (no live slot).
    assert!(!contact_is_valid(
        &world,
        ContactId {
            index1: 1,
            world0: world.world_id,
            padding: 0,
            generation: 99,
        }
    ));

    destroy_body(&mut world, body);
}

#[test]
fn contact_get_data_matches_body_contact_data() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    let mut ground_def = default_body_def();
    ground_def.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
    let ground = create_body(&mut world, &ground_def);
    let ground_shape =
        create_polygon_shape(&mut world, ground, &default_shape_def(), &make_box(10.0, 1.0));

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 0.0, y: 1.0 });
    let body = create_body(&mut world, &body_def);
    let box_shape =
        create_polygon_shape(&mut world, body, &default_shape_def(), &make_square(0.5));

    for _ in 0..30 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    let from_body = body_get_contact_data(&world, body, 8);
    assert_eq!(from_body.len(), 1);
    let contact_id = from_body[0].contact_id;
    assert!(contact_is_valid(&world, contact_id));

    let from_id = contact_get_data(&world, contact_id);
    assert_eq!(from_id.contact_id, contact_id);
    assert_eq!(from_id.shape_id_a, from_body[0].shape_id_a);
    assert_eq!(from_id.shape_id_b, from_body[0].shape_id_b);
    assert_eq!(
        from_id.manifold.point_count,
        from_body[0].manifold.point_count
    );
    assert!(from_id.manifold.point_count > 0);

    // Shape ids resolve to the ground/box pair (order follows contact shape A/B).
    let shapes = [from_id.shape_id_a, from_id.shape_id_b];
    assert!(shapes.contains(&ground_shape));
    assert!(shapes.contains(&box_shape));
}
