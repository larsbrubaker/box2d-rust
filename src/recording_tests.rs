// Port of test_recording.c RecordingTest: a mutator storm across every op
// family — all shape types, every body/shape/joint/world mutator, chain and
// joint lifecycle including destroys, all nine query types issued pre-step
// and mid-loop — recorded, replayed, header-tolerance checked, and round
// tripped through a file.
//
// Not ported (viewer features, deferred with the incremental player):
// RecordingOutlinerTest, RecordingKeyframeTest, RecordingScrubTest,
// RecordingQueryScrubTest, ReplayFileScrubDiag. ReStepRaceTest is N/A in
// the serial port. The per-frame player checks collapse to replay_buffer's
// step/divergence/bounds results.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::*;
use crate::collision::{Capsule, ChainSegment, Circle, MassData, Segment};
use crate::distance::make_proxy;
use crate::geometry::make_box;
use crate::id::ShapeId;
use crate::joint::*;
use crate::math_functions::{
    offset_pos, to_pos, Aabb, Transform, Vec2, WorldTransform, POS_ZERO, ROT_IDENTITY,
};
use crate::recording::{
    load_recording_from_file, replay_buffer, save_recording_to_file, validate_replay,
    world_start_recording, world_stop_recording, Recording,
};
use crate::shape::*;
use crate::types::*;
use crate::world::*;

/// (test_recording.c IssueAllQueries — all nine query types, several from a
/// nonzero base so the origin plumbing records)
fn issue_all_queries(world: &mut World, ground_shape_id: ShapeId) {
    let filter = default_query_filter();

    let base_offset = Vec2 { x: 3.0, y: -2.0 };
    let base = to_pos(base_offset);

    let aabb = Aabb {
        lower_bound: Vec2 {
            x: -5.0 - base_offset.x,
            y: -15.0 - base_offset.y,
        },
        upper_bound: Vec2 {
            x: 5.0 - base_offset.x,
            y: 5.0 - base_offset.y,
        },
    };
    world_overlap_aabb(world, base, aabb, filter, |_| true);

    // OverlapShape (small box proxy) at zero origin
    let proxy = make_proxy(
        &[
            Vec2 { x: -0.5, y: -0.5 },
            Vec2 { x: 0.5, y: -0.5 },
            Vec2 { x: 0.5, y: 0.5 },
            Vec2 { x: -0.5, y: 0.5 },
        ],
        0.0,
    );
    world_overlap_shape(world, POS_ZERO, &proxy, filter, |_| true);

    // CastRay (all hits)
    let ray_origin = to_pos(Vec2 { x: 0.0, y: 10.0 });
    let ray_dir = Vec2 { x: 0.0, y: -20.0 };
    world_cast_ray(world, ray_origin, ray_dir, filter, |_, _, _, fraction| {
        fraction
    });

    // CastRayClosest
    world_cast_ray_closest(world, ray_origin, ray_dir, filter);

    // CastShape (circle proxy), cast from the nonzero base
    let circ_proxy = make_proxy(
        &[Vec2 {
            x: -base_offset.x,
            y: -base_offset.y,
        }],
        0.3,
    );
    world_cast_shape(
        world,
        base,
        &circ_proxy,
        ray_dir,
        filter,
        |_, _, _, fraction| fraction,
    );

    // CollideMover (capsule with radius > 2*B2_LINEAR_SLOP), mover relative
    // to the base
    let mover_cap = Capsule {
        center1: Vec2 {
            x: -0.3 - base_offset.x,
            y: -base_offset.y,
        },
        center2: Vec2 {
            x: 0.3 - base_offset.x,
            y: -base_offset.y,
        },
        radius: 0.5,
    };
    world_collide_mover(world, base, &mover_cap, filter, |_, _| true);

    // CastMover
    let mover_translation = Vec2 { x: 0.0, y: -5.0 };
    world_cast_mover(world, base, &mover_cap, mover_translation, filter);

    // Shape_TestPoint: inside the r=10 ground circle at y=-10, and outside
    shape_test_point(world, ground_shape_id, to_pos(Vec2 { x: 0.0, y: -10.0 }));
    shape_test_point(world, ground_shape_id, to_pos(Vec2 { x: 0.0, y: 100.0 }));

    // Shape_RayCast against the ground shape, ray starting above the ground
    let ray_start = offset_pos(
        base,
        Vec2 {
            x: -base_offset.x,
            y: 5.0 - base_offset.y,
        },
    );
    shape_ray_cast(world, ground_shape_id, ray_start, Vec2 { x: 0.0, y: -20.0 });
}

// (test_recording.c RecordingTest)
#[test]
fn recording_test() {
    // Record a session
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);
    assert!(world_is_valid(&world));

    // Record from before the first step so the whole session is captured
    assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

    // Static ground body with a circle shape
    let mut ground_def = default_body_def();
    ground_def.position = to_pos(Vec2 { x: 0.0, y: -10.0 });
    let ground_id = create_body(&mut world, &ground_def);
    assert!(body_is_valid(&world, ground_id));

    let ground_shape_def = default_shape_def();
    let ground_circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 10.0,
    };
    let ground_shape_id =
        create_circle_shape(&mut world, ground_id, &ground_shape_def, &ground_circle);
    assert!(shape_is_valid(&world, ground_shape_id));

    // Dynamic body with a circle shape. The name is intentionally longer
    // than B2_NAME_LENGTH so replay exercises the over-length name path.
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(Vec2 { x: 0.0, y: 4.0 });
    body_def.name = "testBodyWithLongName".to_string();
    let body_id = create_body(&mut world, &body_def);
    assert!(body_is_valid(&world, body_id));

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.5,
    };
    let shape_id = create_circle_shape(&mut world, body_id, &shape_def, &circle);
    assert!(shape_is_valid(&world, shape_id));

    // Polygon shape on the dynamic body
    let box_poly = make_box(0.25, 0.25);
    let mut box_def = default_shape_def();
    box_def.density = 2.0;
    let box_shape_id = create_polygon_shape(&mut world, body_id, &box_def, &box_poly);
    assert!(shape_is_valid(&world, box_shape_id));

    // Capsule on a second dynamic body
    let mut capsule_body_def = default_body_def();
    capsule_body_def.type_ = BodyType::Dynamic;
    capsule_body_def.position = to_pos(Vec2 { x: 2.0, y: 6.0 });
    let capsule_body_id = create_body(&mut world, &capsule_body_def);
    let capsule = Capsule {
        center1: Vec2 { x: -0.5, y: 0.0 },
        center2: Vec2 { x: 0.5, y: 0.0 },
        radius: 0.25,
    };
    let mut capsule_def = default_shape_def();
    capsule_def.density = 1.0;
    let capsule_shape_id =
        create_capsule_shape(&mut world, capsule_body_id, &capsule_def, &capsule);
    assert!(shape_is_valid(&world, capsule_shape_id));

    // Static segment extending the ground
    let segment = Segment {
        point1: Vec2 { x: -20.0, y: 0.0 },
        point2: Vec2 { x: 20.0, y: 0.0 },
    };
    let segment_def = default_shape_def();
    let segment_shape_id = create_segment_shape(&mut world, ground_id, &segment_def, &segment);
    assert!(shape_is_valid(&world, segment_shape_id));

    // Chain segment shape on the ground
    let chain_seg = ChainSegment {
        ghost1: Vec2 { x: -2.0, y: 1.0 },
        segment: Segment {
            point1: Vec2 { x: -1.0, y: 1.0 },
            point2: Vec2 { x: 1.0, y: 1.0 },
        },
        ghost2: Vec2 { x: 2.0, y: 1.0 },
        chain_id: -1,
    };
    let chain_seg_shape_id =
        create_chain_segment_shape(&mut world, ground_id, &segment_def, &chain_seg);
    assert!(shape_is_valid(&world, chain_seg_shape_id));

    // Exercise the recorded shape mutators
    shape_set_friction(&mut world, box_shape_id, 0.3);
    shape_set_restitution(&mut world, capsule_shape_id, 0.5);
    shape_set_density(&mut world, box_shape_id, 3.0, true);
    shape_set_user_material(&mut world, box_shape_id, 0x1234);
    let mut surface = default_surface_material();
    surface.friction = 0.7;
    surface.restitution = 0.1;
    shape_set_surface_material(&mut world, capsule_shape_id, surface);
    let mut filter = default_filter();
    filter.category_bits = 0x2;
    shape_set_filter(&mut world, box_shape_id, filter);
    shape_enable_contact_events(&mut world, capsule_shape_id, true);
    shape_enable_sensor_events(&mut world, capsule_shape_id, true);
    shape_enable_hit_events(&mut world, box_shape_id, true);
    shape_enable_pre_solve_events(&mut world, box_shape_id, true);
    shape_apply_wind(
        &mut world,
        capsule_shape_id,
        Vec2 { x: 1.0, y: 0.0 },
        0.1,
        0.0,
        true,
    );

    // Change geometry in place
    let new_circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.4,
    };
    shape_set_circle(&mut world, shape_id, &new_circle);

    // Throwaway shape to exercise DestroyShape
    let tmp_circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.1,
    };
    let tmp_shape_id = create_circle_shape(&mut world, capsule_body_id, &capsule_def, &tmp_circle);
    destroy_shape(&mut world, tmp_shape_id, true);

    // A kinematic body to exercise SetType and SetTargetTransform
    let mut kinematic_def = default_body_def();
    kinematic_def.type_ = BodyType::Kinematic;
    kinematic_def.position = to_pos(Vec2 { x: -3.0, y: 5.0 });
    let kinematic_id = create_body(&mut world, &kinematic_def);
    let kinematic_shape_def = default_shape_def();
    let kinematic_circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.3,
    };
    create_circle_shape(
        &mut world,
        kinematic_id,
        &kinematic_shape_def,
        &kinematic_circle,
    );

    // A body to exercise Disable/Enable
    let mut disable_def = default_body_def();
    disable_def.type_ = BodyType::Dynamic;
    disable_def.position = to_pos(Vec2 { x: 5.0, y: 5.0 });
    let disable_id = create_body(&mut world, &disable_def);
    let disable_circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.3,
    };
    create_circle_shape(&mut world, disable_id, &shape_def, &disable_circle);

    // Exercise the recorded body mutators
    body_set_transform(
        &mut world,
        body_id,
        to_pos(Vec2 { x: 1.0, y: 5.0 }),
        ROT_IDENTITY,
    );
    body_set_linear_velocity(&mut world, body_id, Vec2 { x: 0.5, y: 0.0 });
    body_set_angular_velocity(&mut world, body_id, 0.25);
    body_set_name(&mut world, body_id, "renamedBody");
    body_set_linear_damping(&mut world, body_id, 0.1);
    body_set_angular_damping(&mut world, body_id, 0.05);
    body_set_gravity_scale(&mut world, body_id, 0.9);
    body_set_sleep_threshold(&mut world, body_id, 0.02);
    body_enable_sleep(&mut world, body_id, false);
    body_set_bullet(&mut world, body_id, true);
    body_enable_contact_recycling(&mut world, body_id, false);
    body_enable_contact_events(&mut world, body_id, true);
    body_enable_hit_events(&mut world, body_id, true);
    body_set_motion_locks(
        &mut world,
        body_id,
        MotionLocks {
            linear_x: false,
            linear_y: false,
            angular_z: true,
        },
    );
    let md = MassData {
        mass: 2.0,
        center: Vec2 { x: 0.0, y: 0.0 },
        rotational_inertia: 0.5,
    };
    body_set_mass_data(&mut world, body_id, md);
    body_apply_mass_from_shapes(&mut world, body_id);
    body_set_type(&mut world, capsule_body_id, BodyType::Kinematic);
    body_set_type(&mut world, capsule_body_id, BodyType::Dynamic);
    body_set_target_transform(
        &mut world,
        kinematic_id,
        WorldTransform {
            p: to_pos(Vec2 { x: -2.0, y: 5.0 }),
            q: ROT_IDENTITY,
        },
        1.0 / 60.0,
        true,
    );
    body_disable(&mut world, disable_id);
    body_enable(&mut world, disable_id);
    body_set_awake(&mut world, body_id, true);
    body_wake_touching(&mut world, body_id);

    // Per-step forces and impulses applied before the first step
    body_apply_force(
        &mut world,
        body_id,
        Vec2 { x: 0.0, y: 50.0 },
        to_pos(Vec2 { x: 1.0, y: 5.0 }),
        true,
    );
    body_apply_force_to_center(&mut world, body_id, Vec2 { x: 5.0, y: 0.0 }, true);
    body_apply_torque(&mut world, body_id, 1.0, true);
    body_apply_linear_impulse(
        &mut world,
        body_id,
        Vec2 { x: 0.1, y: 0.0 },
        to_pos(Vec2 { x: 1.0, y: 5.0 }),
        true,
    );
    body_apply_linear_impulse_to_center(&mut world, body_id, Vec2 { x: 0.0, y: 0.1 }, true);
    body_apply_angular_impulse(&mut world, body_id, 0.05, true);

    // Chain shape on a static body, plus a material change and a throwaway
    // chain destroyed
    let mut chain_body_def = default_body_def();
    chain_body_def.position = to_pos(Vec2 { x: 0.0, y: -2.0 });
    let chain_body_id = create_body(&mut world, &chain_body_def);
    let mut chain_def = default_chain_def();
    chain_def.points = vec![
        Vec2 { x: -8.0, y: 0.0 },
        Vec2 { x: -4.0, y: 0.0 },
        Vec2 { x: 0.0, y: 0.0 },
        Vec2 { x: 4.0, y: 0.0 },
        Vec2 { x: 8.0, y: 0.0 },
        Vec2 { x: 8.0, y: 4.0 },
    ];
    chain_def.is_loop = false;
    let chain_id = create_chain(&mut world, chain_body_id, &chain_def);
    assert!(chain_is_valid(&world, chain_id));

    let mut chain_surface = default_surface_material();
    chain_surface.friction = 0.4;
    chain_set_surface_material(&mut world, chain_id, chain_surface, 0);

    let tmp_chain_id = create_chain(&mut world, chain_body_id, &chain_def);
    destroy_chain(&mut world, tmp_chain_id);

    // Joints: a row of dynamic bodies connected by each joint type
    let mut jb = Vec::new();
    for i in 0..8 {
        let mut jbd = default_body_def();
        jbd.type_ = BodyType::Dynamic;
        jbd.position = to_pos(Vec2 {
            x: -7.0 + i as f32,
            y: 8.0,
        });
        let b = create_body(&mut world, &jbd);
        let jc = Circle {
            center: Vec2 { x: 0.0, y: 0.0 },
            radius: 0.25,
        };
        create_circle_shape(&mut world, b, &shape_def, &jc);
        jb.push(b);
    }

    // Revolute joint with full setter coverage and the generic mutators
    let mut rev_def = default_revolute_joint_def();
    rev_def.base.body_id_a = jb[0];
    rev_def.base.body_id_b = jb[1];
    rev_def.base.local_frame_a.p = Vec2 { x: 0.5, y: 0.0 };
    rev_def.base.local_frame_b.p = Vec2 { x: -0.5, y: 0.0 };
    let rev_id = create_revolute_joint(&mut world, &rev_def);
    assert!(joint_is_valid(&world, rev_id));
    crate::revolute_joint::revolute_joint_enable_limit(&mut world, rev_id, true);
    crate::revolute_joint::revolute_joint_set_limits(&mut world, rev_id, -1.0, 1.0);
    crate::revolute_joint::revolute_joint_enable_motor(&mut world, rev_id, true);
    crate::revolute_joint::revolute_joint_set_motor_speed(&mut world, rev_id, 0.5);
    crate::revolute_joint::revolute_joint_set_max_motor_torque(&mut world, rev_id, 10.0);
    crate::revolute_joint::revolute_joint_enable_spring(&mut world, rev_id, true);
    crate::revolute_joint::revolute_joint_set_spring_hertz(&mut world, rev_id, 2.0);
    crate::revolute_joint::revolute_joint_set_spring_damping_ratio(&mut world, rev_id, 0.5);
    crate::revolute_joint::revolute_joint_set_target_angle(&mut world, rev_id, 0.25);
    joint_set_local_frame_a(
        &mut world,
        rev_id,
        Transform {
            p: Vec2 { x: 0.5, y: 0.0 },
            q: ROT_IDENTITY,
        },
    );
    joint_set_local_frame_b(
        &mut world,
        rev_id,
        Transform {
            p: Vec2 { x: -0.5, y: 0.0 },
            q: ROT_IDENTITY,
        },
    );
    joint_set_constraint_tuning(&mut world, rev_id, 60.0, 2.0);
    joint_set_force_threshold(&mut world, rev_id, 100.0);
    joint_set_torque_threshold(&mut world, rev_id, 50.0);
    joint_set_collide_connected(&mut world, rev_id, false);
    joint_wake_bodies(&mut world, rev_id);

    // Distance joint
    let mut dist_def = default_distance_joint_def();
    dist_def.base.body_id_a = jb[1];
    dist_def.base.body_id_b = jb[2];
    dist_def.length = 1.0;
    let dist_id = create_distance_joint(&mut world, &dist_def);
    crate::distance_joint::distance_joint_set_length(&mut world, dist_id, 1.2);
    crate::distance_joint::distance_joint_enable_spring(&mut world, dist_id, true);
    crate::distance_joint::distance_joint_set_spring_hertz(&mut world, dist_id, 3.0);
    crate::distance_joint::distance_joint_set_spring_damping_ratio(&mut world, dist_id, 0.4);
    crate::distance_joint::distance_joint_set_spring_force_range(&mut world, dist_id, -50.0, 50.0);
    crate::distance_joint::distance_joint_enable_limit(&mut world, dist_id, true);
    crate::distance_joint::distance_joint_set_length_range(&mut world, dist_id, 0.5, 2.0);
    crate::distance_joint::distance_joint_enable_motor(&mut world, dist_id, true);
    crate::distance_joint::distance_joint_set_motor_speed(&mut world, dist_id, 0.3);
    crate::distance_joint::distance_joint_set_max_motor_force(&mut world, dist_id, 5.0);

    // Prismatic joint
    let mut pris_def = default_prismatic_joint_def();
    pris_def.base.body_id_a = jb[2];
    pris_def.base.body_id_b = jb[3];
    let pris_id = create_prismatic_joint(&mut world, &pris_def);
    crate::prismatic_joint::prismatic_joint_enable_spring(&mut world, pris_id, true);
    crate::prismatic_joint::prismatic_joint_set_spring_hertz(&mut world, pris_id, 2.0);
    crate::prismatic_joint::prismatic_joint_set_spring_damping_ratio(&mut world, pris_id, 0.5);
    crate::prismatic_joint::prismatic_joint_set_target_translation(&mut world, pris_id, 0.1);
    crate::prismatic_joint::prismatic_joint_enable_limit(&mut world, pris_id, true);
    crate::prismatic_joint::prismatic_joint_set_limits(&mut world, pris_id, -1.0, 1.0);
    crate::prismatic_joint::prismatic_joint_enable_motor(&mut world, pris_id, true);
    crate::prismatic_joint::prismatic_joint_set_motor_speed(&mut world, pris_id, 0.2);
    crate::prismatic_joint::prismatic_joint_set_max_motor_force(&mut world, pris_id, 8.0);

    // Wheel joint
    let mut wheel_def = default_wheel_joint_def();
    wheel_def.base.body_id_a = jb[3];
    wheel_def.base.body_id_b = jb[4];
    let wheel_id = create_wheel_joint(&mut world, &wheel_def);
    crate::wheel_joint::wheel_joint_enable_spring(&mut world, wheel_id, true);
    crate::wheel_joint::wheel_joint_set_spring_hertz(&mut world, wheel_id, 4.0);
    crate::wheel_joint::wheel_joint_set_spring_damping_ratio(&mut world, wheel_id, 0.7);
    crate::wheel_joint::wheel_joint_enable_limit(&mut world, wheel_id, true);
    crate::wheel_joint::wheel_joint_set_limits(&mut world, wheel_id, -0.5, 0.5);
    crate::wheel_joint::wheel_joint_enable_motor(&mut world, wheel_id, true);
    crate::wheel_joint::wheel_joint_set_motor_speed(&mut world, wheel_id, 1.0);
    crate::wheel_joint::wheel_joint_set_max_motor_torque(&mut world, wheel_id, 6.0);

    // Weld joint
    let mut weld_def = default_weld_joint_def();
    weld_def.base.body_id_a = jb[4];
    weld_def.base.body_id_b = jb[5];
    let weld_id = create_weld_joint(&mut world, &weld_def);
    crate::weld_joint::weld_joint_set_linear_hertz(&mut world, weld_id, 5.0);
    crate::weld_joint::weld_joint_set_linear_damping_ratio(&mut world, weld_id, 0.6);
    crate::weld_joint::weld_joint_set_angular_hertz(&mut world, weld_id, 5.0);
    crate::weld_joint::weld_joint_set_angular_damping_ratio(&mut world, weld_id, 0.6);

    // Motor joint
    let mut motor_def = default_motor_joint_def();
    motor_def.base.body_id_a = jb[5];
    motor_def.base.body_id_b = jb[6];
    let motor_id = create_motor_joint(&mut world, &motor_def);
    crate::motor_joint::motor_joint_set_linear_velocity(
        &mut world,
        motor_id,
        Vec2 { x: 0.1, y: 0.0 },
    );
    crate::motor_joint::motor_joint_set_angular_velocity(&mut world, motor_id, 0.2);
    crate::motor_joint::motor_joint_set_max_velocity_force(&mut world, motor_id, 10.0);
    crate::motor_joint::motor_joint_set_max_velocity_torque(&mut world, motor_id, 10.0);
    crate::motor_joint::motor_joint_set_linear_hertz(&mut world, motor_id, 2.0);
    crate::motor_joint::motor_joint_set_linear_damping_ratio(&mut world, motor_id, 0.5);
    crate::motor_joint::motor_joint_set_angular_hertz(&mut world, motor_id, 2.0);
    crate::motor_joint::motor_joint_set_angular_damping_ratio(&mut world, motor_id, 0.5);
    crate::motor_joint::motor_joint_set_max_spring_force(&mut world, motor_id, 20.0);
    crate::motor_joint::motor_joint_set_max_spring_torque(&mut world, motor_id, 20.0);

    // Filter joint, plus a throwaway joint to exercise DestroyJoint
    let mut filter_def = default_filter_joint_def();
    filter_def.base.body_id_a = jb[6];
    filter_def.base.body_id_b = jb[7];
    let filter_id = create_filter_joint(&mut world, &filter_def);
    assert!(joint_is_valid(&world, filter_id));

    let mut tmp_joint_def = default_distance_joint_def();
    tmp_joint_def.base.body_id_a = jb[0];
    tmp_joint_def.base.body_id_b = jb[7];
    tmp_joint_def.length = 5.0;
    let tmp_joint_id = create_distance_joint(&mut world, &tmp_joint_def);
    destroy_joint(&mut world, tmp_joint_id, true);

    // Exercise world config mutators
    world_set_gravity(&mut world, Vec2 { x: 0.0, y: -9.8 });
    world_enable_sleeping(&mut world, true);
    world_enable_continuous(&mut world, true);
    world_enable_warm_starting(&mut world, true);
    world_enable_speculative(&mut world, true);
    world_set_restitution_threshold(&mut world, 1.5);
    world_set_hit_event_threshold(&mut world, 2.0);
    world_set_contact_tuning(&mut world, 30.0, 10.0, 3.0);
    world_set_contact_recycle_distance(&mut world, 0.05);
    world_set_maximum_linear_speed(&mut world, 100.0);
    world_rebuild_static_tree(&mut world);
    let mut explosion = default_explosion_def();
    explosion.position = to_pos(Vec2 { x: 0.0, y: 4.0 });
    explosion.radius = 3.0;
    explosion.falloff = 1.0;
    explosion.impulse_per_length = 5.0;
    world_explode(&mut world, &explosion);

    // Issue all 9 query types before the first step (pre-step path)
    issue_all_queries(&mut world, ground_shape_id);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for i in 0..60 {
        // Inject mutators mid-simulation to exercise interleaving with steps
        if i == 30 {
            body_apply_linear_impulse_to_center(
                &mut world,
                capsule_body_id,
                Vec2 { x: 2.0, y: 0.0 },
                true,
            );
            body_clear_forces(&mut world, body_id);
            body_set_gravity_scale(&mut world, body_id, 1.0);
        }

        // Also issue queries mid-loop to exercise recording across steps
        if i == 15 {
            issue_all_queries(&mut world, ground_shape_id);
        }

        world_step(&mut world, time_step, sub_step_count);
    }

    let rec = world_stop_recording(&mut world).expect("active session");

    // The recording buffer now holds the full session
    let rec_data = &rec.buffer;
    assert!(!rec_data.is_empty());

    // Replay from the buffer (the C worker-count sweep collapses serially)
    assert!(validate_replay(rec_data));

    // The reserved header bytes (offsets 8 and 16, formerly buildHash and
    // simdWidth) must stay ignored on read.
    {
        let mut patched = rec_data.clone();
        patched[8] = 0xAB;
        patched[9] = 0xCD;
        patched[10] = 0xEF;
        patched[11] = 0x12;
        patched[16] = 0x34;
        assert!(validate_replay(&patched));
    }

    // File round-trip: save the buffer, load it back, and replay the copy
    let path = std::env::temp_dir().join("box2d_rust_test_recording.b2rec");
    assert!(save_recording_to_file(&rec, &path));
    let loaded = load_recording_from_file(&path).expect("load recording");
    assert_eq!(loaded.buffer.len(), rec_data.len());
    assert!(validate_replay(&loaded.buffer));
    let _ = std::fs::remove_file(&path);

    // The per-frame player checks collapse to replay_buffer results: frame
    // count, divergence, and the recorded bounds framing the whole scene
    // (ground circle and segment bracket x in [-20, 20], circle bottom at
    // y = -20).
    let result = replay_buffer(rec_data);
    assert!(result.ok);
    assert!(!result.diverged);
    assert_eq!(result.steps, 60);
    assert!(result.have_bounds);
    assert!(result.bounds.upper_bound.x - result.bounds.lower_bound.x > 0.0);
    assert!(result.bounds.upper_bound.y - result.bounds.lower_bound.y > 0.0);
    assert!(result.bounds.lower_bound.x <= -20.0 && result.bounds.upper_bound.x >= 20.0);
    assert!(result.bounds.lower_bound.y <= -20.0);

    // Restart reproduces the same run without reloading the file
    let result2 = replay_buffer(rec_data);
    assert_eq!(result2.steps, 60);
    assert!(!result2.diverged);
}
