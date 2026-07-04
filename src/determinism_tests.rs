// Port of test_determinism.c + the shared FallingHinges harness
// (shared/determinism.c). This is the acceptance test for cross-platform
// determinism: 80 hinged boxes fall, settle, and sleep; the step on which
// they all sleep and a djb2 hash over every final body transform must match
// the C reference bit-for-bit.
//
// C's MultithreadingTest re-runs the scene across worker counts to prove the
// multithreaded solver is deterministic; the Rust port is serial, so a single
// run against the expected values is the complete equivalent.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_get_transform, create_body};
use crate::core::{hash, HASH_INIT};
use crate::geometry::make_square;
use crate::id::BodyId;
use crate::joint::create_revolute_joint;
use crate::math_functions::{make_rot, to_pos, Vec2, WorldTransform, PI};
use crate::shape::create_polygon_shape;
use crate::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, default_world_def, BodyType,
};
use crate::world::{world_get_awake_body_count, world_get_body_events, world_step, World};

struct FallingHingeData {
    body_ids: Vec<BodyId>,
    step_count: i32,
    sleep_step: i32,
    hash: u32,
}

/// Hash a transform's raw bytes exactly like C's
/// `b2Hash(hash, (uint8_t*)&transform, sizeof(b2WorldTransform))`. The C
/// struct is {p, q} with no padding (p is 2 floats, or 2 doubles in the
/// double-precision build; q is always 2 floats), fields serialized here in
/// declaration order as little-endian, matching x86/wasm memory layout.
fn hash_transform(h: u32, transform: &WorldTransform) -> u32 {
    let mut bytes: Vec<u8> = Vec::with_capacity(24);
    bytes.extend_from_slice(&transform.p.x.to_le_bytes());
    bytes.extend_from_slice(&transform.p.y.to_le_bytes());
    bytes.extend_from_slice(&transform.q.c.to_le_bytes());
    bytes.extend_from_slice(&transform.q.s.to_le_bytes());
    hash(h, &bytes)
}

/// (shared/determinism.c CreateFallingHinges)
fn create_falling_hinges(world: &mut World) -> FallingHingeData {
    {
        let mut body_def = default_body_def();
        body_def.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
        let ground_id = create_body(world, &body_def);

        let box_poly = crate::geometry::make_box(40.0, 1.0);
        let shape_def = default_shape_def();
        create_polygon_shape(world, ground_id, &shape_def, &box_poly);
    }

    let column_count = 4usize;
    let row_count = 20usize;
    let body_count = row_count * column_count;

    let mut body_ids: Vec<BodyId> = Vec::with_capacity(body_count);

    let h = 0.25f32;
    // C builds a rounded box and immediately overwrites it with the square;
    // the dead store is kept out of the Rust port.
    let box_poly = make_square(h);

    let shape_def = default_shape_def();

    let mut joint_def = default_revolute_joint_def();
    joint_def.enable_limit = true;
    joint_def.lower_angle = -0.1 * PI;
    joint_def.upper_angle = 0.2 * PI;
    joint_def.enable_spring = true;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 1.0;
    joint_def.enable_motor = true;
    joint_def.max_motor_torque = 0.25;
    joint_def.base.local_frame_a.p = Vec2 { x: -h, y: h };
    joint_def.base.local_frame_b.p = Vec2 { x: -h, y: -h };
    joint_def.base.constraint_hertz = 60.0;
    joint_def.base.constraint_damping_ratio = 0.0;
    joint_def.base.draw_scale = 0.5;

    let offset = 0.4 * h;
    let dx = 10.0 * h;
    let x_base = -0.5 * dx * (column_count as f32 - 1.0);

    for j in 0..column_count {
        let x = x_base + j as f32 * dx;

        let mut prev_body_id = BodyId::default();

        for i in 0..row_count {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;

            body_def.position = to_pos(Vec2 {
                x: x + offset * i as f32,
                y: h + 2.0 * h * i as f32,
            });

            // this tests the deterministic cosine and sine functions
            let angle = if (i & 1) == 0 { -0.1 } else { 0.1 };
            body_def.rotation = make_rot(angle);

            let body_id = create_body(world, &body_def);

            if (i & 1) == 0 {
                prev_body_id = body_id;
            } else {
                joint_def.base.body_id_a = prev_body_id;
                joint_def.base.body_id_b = body_id;
                create_revolute_joint(world, &joint_def);
                prev_body_id = BodyId::default();
            }

            create_polygon_shape(world, body_id, &shape_def, &box_poly);

            body_ids.push(body_id);
        }
    }

    assert_eq!(body_ids.len(), body_count);

    FallingHingeData {
        body_ids,
        step_count: 0,
        sleep_step: -1,
        hash: 0,
    }
}

/// (shared/determinism.c UpdateFallingHinges)
fn update_falling_hinges(world: &World, data: &mut FallingHingeData) -> bool {
    if data.hash == 0 {
        let body_events = world_get_body_events(world);

        if body_events.is_empty() {
            let awake_count = world_get_awake_body_count(world);
            assert_eq!(awake_count, 0);

            data.hash = HASH_INIT;
            for body_id in &data.body_ids {
                let transform = body_get_transform(world, *body_id);
                data.hash = hash_transform(data.hash, &transform);
            }

            data.sleep_step = data.step_count;
        }
    }

    data.step_count += 1;

    data.hash != 0
}

#[cfg(feature = "double-precision")]
const EXPECTED_SLEEP_STEP: i32 = 313;
#[cfg(feature = "double-precision")]
const EXPECTED_HASH: u32 = 0xF7C3082A;

#[cfg(not(feature = "double-precision"))]
const EXPECTED_SLEEP_STEP: i32 = 294;
#[cfg(not(feature = "double-precision"))]
const EXPECTED_HASH: u32 = 0x006F0F5E;

// (test_determinism.c SingleMultithreadingTest, serial)
#[test]
fn falling_hinges() {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut data = create_falling_hinges(&mut world);

    let time_step = 1.0 / 60.0;
    let step_limit = 500;
    for _ in 0..step_limit {
        let sub_step_count = 4;
        world_step(&mut world, time_step, sub_step_count);

        let done = update_falling_hinges(&world, &mut data);
        if done {
            break;
        }
    }

    if data.sleep_step != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH {
        println!(
            "  sleepStep={} hash=0x{:08X}",
            data.sleep_step, data.hash
        );
    }

    assert_eq!(data.sleep_step, EXPECTED_SLEEP_STEP);
    assert_eq!(data.hash, EXPECTED_HASH);
}
