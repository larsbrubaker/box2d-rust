// Port of test_snapshot.c: snapshot/restore over a scene exercising every
// serialized subsystem — stacks, four joint types, a chain shape, a sensor,
// and an independently sleeping island.
//
// The recording-file portions of the C test (save mid-stream, replay from
// disk) land with the recording player; the snapshot invariants are complete
// here.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_is_valid, create_body};
use crate::geometry::make_box;
use crate::id::{BodyId, ChainId, JointId, ShapeId};
use crate::joint::{
    create_distance_joint, create_prismatic_joint, create_revolute_joint, create_weld_joint,
    joint_is_valid,
};
use crate::math_functions::{to_pos, Vec2, PI};
use crate::recording::{
    create_world_from_snapshot, hash_world_state, hash_world_state_deep, world_restore,
    world_snapshot,
};
use crate::shape::{chain_is_valid, create_chain, create_polygon_shape, shape_is_valid};
use crate::types::{
    default_body_def, default_chain_def, default_distance_joint_def, default_prismatic_joint_def,
    default_revolute_joint_def, default_shape_def, default_surface_material,
    default_weld_joint_def, default_world_def, BodyType,
};
use crate::world::{world_get_awake_body_count, world_get_body_events, world_step, World};

struct SnapshotIds {
    body: BodyId,
    shape: ShapeId,
    joint: JointId,
    chain: ChainId,
}

/// (test_snapshot.c BuildScene)
fn build_scene(out_ids: &mut Option<SnapshotIds>) -> World {
    let def = default_world_def();
    let mut world = World::new(&def);

    // Ground
    {
        let mut bd = default_body_def();
        bd.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
        let ground_id = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground_id, &sd, &make_box(40.0, 1.0));
    }

    // Main stack: 8 dynamic boxes
    let mut stack_top = BodyId::default();
    {
        let sd = default_shape_def();
        let box_poly = make_box(0.5, 0.5);
        for i in 0..8 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: 0.0,
                y: 0.5 + i as f32 * 1.1,
            });
            let body_id = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body_id, &sd, &box_poly);
            stack_top = body_id;
        }
    }

    // Joint bodies: one for each pair of joints
    let mut jb_def = default_body_def();
    jb_def.type_ = BodyType::Dynamic;
    jb_def.position = to_pos(Vec2 { x: 5.0, y: 2.0 });
    let jb_a = create_body(&mut world, &jb_def);
    jb_def.position = to_pos(Vec2 { x: 7.0, y: 2.0 });
    let jb_b = create_body(&mut world, &jb_def);
    jb_def.position = to_pos(Vec2 { x: 9.0, y: 2.0 });
    let jb_c = create_body(&mut world, &jb_def);
    jb_def.position = to_pos(Vec2 { x: 11.0, y: 2.0 });
    let jb_d = create_body(&mut world, &jb_def);

    let jbox = make_box(0.3, 0.3);
    let jsd = default_shape_def();
    let held_shape = create_polygon_shape(&mut world, jb_a, &jsd, &jbox);
    create_polygon_shape(&mut world, jb_b, &jsd, &jbox);
    create_polygon_shape(&mut world, jb_c, &jsd, &jbox);
    create_polygon_shape(&mut world, jb_d, &jsd, &jbox);

    // Revolute joint (mirrors determinism.c idiom)
    let held_joint;
    {
        let mut rd = default_revolute_joint_def();
        rd.enable_limit = true;
        rd.lower_angle = -0.1 * PI;
        rd.upper_angle = 0.2 * PI;
        rd.enable_spring = true;
        rd.hertz = 1.0;
        rd.damping_ratio = 1.0;
        rd.enable_motor = true;
        rd.max_motor_torque = 0.5;
        rd.base.body_id_a = jb_a;
        rd.base.body_id_b = jb_b;
        rd.base.local_frame_a.p = Vec2 { x: 0.3, y: 0.0 };
        rd.base.local_frame_b.p = Vec2 { x: -0.3, y: 0.0 };
        held_joint = create_revolute_joint(&mut world, &rd);
    }

    // Prismatic joint
    {
        let mut pd = default_prismatic_joint_def();
        pd.enable_limit = true;
        pd.lower_translation = -0.5;
        pd.upper_translation = 0.5;
        pd.base.body_id_a = jb_b;
        pd.base.body_id_b = jb_c;
        pd.base.local_frame_a.p = Vec2 { x: 0.3, y: 0.0 };
        pd.base.local_frame_b.p = Vec2 { x: -0.3, y: 0.0 };
        create_prismatic_joint(&mut world, &pd);
    }

    // Distance joint
    {
        let mut dd = default_distance_joint_def();
        dd.length = 2.0;
        dd.base.body_id_a = jb_c;
        dd.base.body_id_b = jb_d;
        dd.base.local_frame_a.p = Vec2 { x: 0.3, y: 0.0 };
        dd.base.local_frame_b.p = Vec2 { x: -0.3, y: 0.0 };
        create_distance_joint(&mut world, &dd);
    }

    // Weld joint
    {
        let mut wd = default_weld_joint_def();
        wd.linear_hertz = 5.0;
        wd.linear_damping_ratio = 0.7;
        wd.base.body_id_a = jb_d;
        wd.base.body_id_b = stack_top;
        wd.base.local_frame_a.p = Vec2 { x: 0.3, y: 0.0 };
        wd.base.local_frame_b.p = Vec2 { x: 0.0, y: 0.0 };
        create_weld_joint(&mut world, &wd);
    }

    // Chain shape on a static body
    let held_chain;
    {
        let mut cbd = default_body_def();
        cbd.position = to_pos(Vec2 { x: -10.0, y: 0.0 });
        let chain_body_id = create_body(&mut world, &cbd);

        let mut chain_mat = default_surface_material();
        chain_mat.friction = 0.4;
        let mut chain_def = default_chain_def();
        chain_def.points = vec![
            Vec2 { x: -4.0, y: 0.0 },
            Vec2 { x: -2.0, y: 0.0 },
            Vec2 { x: 0.0, y: 0.0 },
            Vec2 { x: 2.0, y: 0.0 },
            Vec2 { x: 4.0, y: 2.0 },
        ];
        chain_def.materials = vec![chain_mat];
        chain_def.is_loop = false;
        held_chain = create_chain(&mut world, chain_body_id, &chain_def);
    }

    // Sensor on a static body, overlapping the scene area
    {
        let mut sbd = default_body_def();
        sbd.position = to_pos(Vec2 { x: 0.0, y: 5.0 });
        let sensor_body_id = create_body(&mut world, &sbd);

        let mut sensor_def = default_shape_def();
        sensor_def.is_sensor = true;
        sensor_def.enable_sensor_events = true;
        create_polygon_shape(&mut world, sensor_body_id, &sensor_def, &make_box(3.0, 3.0));
    }

    // Isolated second stack far from the main scene — sleeps independently
    {
        let sd = default_shape_def();
        let box_poly = make_box(0.5, 0.5);
        for i in 0..6 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: 40.0,
                y: 0.5 + i as f32 * 1.1,
            });
            let body_id = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body_id, &sd, &box_poly);
        }
    }

    *out_ids = Some(SnapshotIds {
        body: stack_top,
        shape: held_shape,
        joint: held_joint,
        chain: held_chain,
    });

    world
}

/// (test_snapshot.c StepUntilSleep)
fn step_until_sleep(world: &mut World) -> i32 {
    for step in 0..600 {
        world_step(world, 1.0 / 60.0, 4);
        if world_get_body_events(world).is_empty() && world_get_awake_body_count(world) == 0 {
            return step;
        }
    }
    panic!("scene never slept");
}

// (test_snapshot.c SnapshotTest — recording-file portions deferred)
#[test]
fn snapshot_test() {
    let mut ids = None;
    let mut world_a = build_scene(&mut ids);
    let ids = ids.unwrap();

    // Multiple sleeping islands force sleeping solver sets into the image.
    step_until_sleep(&mut world_a);
    assert!(world_a.solver_sets.len() > 3);

    let image = world_snapshot(&world_a);
    assert!(!image.is_empty());

    // Restore into a fresh world: shallow and deep hashes must match.
    let mut world_b = create_world_from_snapshot(&image).expect("restore");
    assert_eq!(hash_world_state(&world_a), hash_world_state(&world_b));
    assert_eq!(
        hash_world_state_deep(&world_a),
        hash_world_state_deep(&world_b)
    );

    // Wake everything by stepping with a new body dropped on each stack; both
    // worlds must stay bit-identical.
    for world in [&mut world_a, &mut world_b] {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec2 { x: 0.0, y: 12.0 });
        let body = create_body(world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(world, body, &sd, &make_box(0.4, 0.4));
    }
    for step in 0..120 {
        world_step(&mut world_a, 1.0 / 60.0, 4);
        world_step(&mut world_b, 1.0 / 60.0, 4);
        assert_eq!(
            hash_world_state_deep(&world_a),
            hash_world_state_deep(&world_b),
            "diverged at step {step}"
        );
    }

    // Restore over a populated, diverged world: ids from snapshot time are
    // valid again; ids created after the snapshot are not.
    let snap_hash = {
        let restored = create_world_from_snapshot(&image).unwrap();
        hash_world_state_deep(&restored)
    };
    let mut world_r = create_world_from_snapshot(&image).unwrap();
    for _ in 0..30 {
        world_step(&mut world_r, 1.0 / 60.0, 4);
    }
    let bd = default_body_def();
    let post_body = create_body(&mut world_r, &bd);
    assert!(hash_world_state_deep(&world_r) != snap_hash);

    assert!(world_restore(&mut world_r, &image));
    assert_eq!(hash_world_state_deep(&world_r), snap_hash);
    assert!(body_is_valid(&world_r, ids.body));
    assert!(shape_is_valid(&world_r, ids.shape));
    assert!(joint_is_valid(&world_r, ids.joint));
    assert!(chain_is_valid(&world_r, ids.chain));
    assert!(!body_is_valid(&world_r, post_body));

    // Bad images are refused without touching the world.
    let pre_bad_hash = hash_world_state_deep(&world_r);
    assert!(!world_restore(&mut world_r, &[]));
    let mut corrupt = image.clone();
    corrupt[8] ^= 0xFF; // layout hash
    assert!(!world_restore(&mut world_r, &corrupt));
    assert_eq!(hash_world_state_deep(&world_r), pre_bad_hash);

    // Repeated restores are idempotent.
    assert!(world_restore(&mut world_r, &image));
    assert!(world_restore(&mut world_r, &image));
    assert_eq!(hash_world_state_deep(&world_r), snap_hash);

    world_r.validate_solver_sets();
    world_r.validate_contacts();
    world_r.validate_connectivity();
}
