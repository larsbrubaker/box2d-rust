//! Native tests for SimWorld sample binding surface (real production methods).
//! Grab/draw are owned by the Phase 1 harness `interact/` module — not tested here.

use super::SimWorld;

#[test]
fn joint_types_create_and_track() {
    let mut sim = SimWorld::new(-10.0);
    let ground = sim.add_static_box(0.0, -1.0, 20.0, 1.0);
    let a = sim.add_box(-1.0, 2.0, 0.5, 0.5, 1.0);
    let b = sim.add_box(1.0, 2.0, 0.5, 0.5, 1.0);
    let c = sim.add_box(0.0, 4.0, 0.4, 0.4, 1.0);

    let revolute = sim.add_revolute_joint(
        ground, a, -1.0, 2.0, true, -0.5, 0.5, false, 0.0, 0.0, false, 0.0, 0.0, false,
    );
    let prismatic = sim.add_prismatic_joint(
        ground, b, 1.0, 2.0, 1.0, 0.0, true, -1.0, 1.0, false, 0.0, 0.0, false, 0.0, 0.0,
        false,
    );
    let weld = sim.add_weld_joint(a, c, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, false);
    let wheel = sim.add_wheel_joint(
        ground, b, 1.0, 1.0, 0.0, 1.0, false, 0.0, 0.0, false, 0.0, 0.0, true, 4.0, 0.7,
        false,
    );
    let motor = sim.add_motor_joint(ground, a, 4.0, 1.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
    let filter = sim.add_filter_joint(a, b);
    let dist = sim.add_distance_joint_ex(
        ground, c, 0.0, 4.0, 0.0, 5.0, 1.0, true, 5.0, 0.5, 100.0, 50.0, false, 0.5, 2.0, false,
    );

    assert_eq!(sim.joint_count(), 7);
    sim.distance_set_length(dist, 1.5);
    sim.revolute_enable_motor(revolute, true);
    sim.joint_wake_bodies(revolute);
    let ft = sim.joint_constraint_ft(motor);
    assert_eq!(ft.len(), 3);
    let _ = (revolute, prismatic, weld, wheel, motor, filter, dist);

    for _ in 0..10 {
        sim.step(1.0 / 60.0, 4);
    }
    assert_eq!(sim.body_count(), 4);
}

#[test]
fn shapes_segment_polygon_attach() {
    let mut sim = SimWorld::new(-10.0);
    let seg = sim.add_segment(-10.0, 0.0, 10.0, 0.0);
    let body = sim.add_body(0.0, 2.0, 0.0, 2);
    sim.attach_box(body, 0.5, 0.25, 0.0, 0.0, 0.0, 1.0, 0.3, 0.0);
    sim.attach_circle(body, 0.0, 0.5, 0.2, 1.0, 0.3, 0.1);
    let poly = sim.add_polygon(
        2.0,
        3.0,
        0.0,
        &[-0.5, -0.5, 0.5, -0.5, 0.0, 0.5],
        0.0,
        1.0,
    );
    let _ = (seg, poly);
    for _ in 0..20 {
        sim.step(1.0 / 60.0, 4);
    }
    assert!(sim.body_count() >= 3);
}

#[test]
fn body_ops_transform_impulse_type() {
    let mut sim = SimWorld::new(-10.0);
    sim.add_static_box(0.0, -1.0, 20.0, 1.0);
    let box_i = sim.add_box(0.0, 2.0, 0.5, 0.5, 1.0);

    sim.set_transform(box_i, 1.0, 3.0, 0.25);
    let pos = sim.positions();
    assert!((pos[box_i * 3] - 1.0).abs() < 1e-4);
    assert!((pos[box_i * 3 + 1] - 3.0).abs() < 1e-4);

    sim.apply_linear_impulse_to_center(box_i, 5.0, 0.0, true);
    let v = sim.get_linear_velocity(box_i);
    assert!(v[0] > 0.0);

    sim.set_body_type(box_i, 0);
    assert_eq!(sim.get_body_type(box_i), 0);
    sim.set_body_type(box_i, 2);
    assert_eq!(sim.get_body_type(box_i), 2);

    sim.disable_body(box_i);
    assert!(!sim.is_body_enabled(box_i));
    sim.enable_body(box_i);
    assert!(sim.is_body_enabled(box_i));
}

#[test]
fn destroy_body_keeps_demo_indices_stable() {
    let mut sim = SimWorld::new(-10.0);
    let ground = sim.add_segment(-10.0, 0.0, 10.0, 0.0);
    let a = sim.add_box(-1.0, 2.0, 0.5, 0.5, 1.0);
    let b = sim.add_box(1.0, 2.0, 0.5, 0.5, 1.0);
    assert_eq!(sim.body_count(), 3);
    sim.destroy_body(a);
    // Slot retained; later index still addresses body b.
    assert_eq!(sim.body_count(), 3);
    sim.set_linear_velocity(b, 1.0, 0.0);
    let v = sim.get_linear_velocity(b);
    assert!(v[0] > 0.0);
    // Idempotent on already-destroyed slot.
    sim.destroy_body(a);
    let _ = ground;
    for _ in 0..5 {
        sim.step(1.0 / 60.0, 4);
    }
}

#[test]
fn world_toggles() {
    let mut sim = SimWorld::new(-10.0);
    sim.add_static_box(0.0, -1.0, 10.0, 1.0);
    sim.add_box(0.0, 2.0, 0.5, 0.5, 1.0);

    sim.set_sleeping(false);
    sim.set_warm_starting(true);
    assert!(sim.is_warm_starting_enabled());
    sim.set_continuous_collision(true);
    sim.set_speculative(true);
    sim.set_contact_tuning(90.0, 10.0, 3.0);
    let g = sim.get_gravity();
    assert!((g[1] + 10.0).abs() < 1e-5);

    for _ in 0..5 {
        sim.step(1.0 / 60.0, 4);
    }
}

#[test]
fn destroy_body_marks_slot_not_alive() {
    let mut sim = SimWorld::new(-10.0);
    sim.add_static_box(0.0, -1.0, 20.0, 1.0);
    let a = sim.add_box(-1.0, 2.0, 0.5, 0.5, 1.0);
    let b = sim.add_box(1.0, 2.0, 0.5, 0.5, 1.0);
    assert_eq!(sim.body_count(), 3);
    assert!(sim.is_body_alive(a));
    sim.destroy_body(a);
    assert!(!sim.is_body_alive(a));
    assert!(sim.is_body_alive(b));
    // Slot retained (Bodies semantics) — body_count still includes the hole.
    assert_eq!(sim.body_count(), 3);
    for _ in 0..10 {
        sim.step(1.0 / 60.0, 4);
    }
}

#[test]
fn attach_polygon_and_rounded_box() {
    let mut sim = SimWorld::new(-10.0);
    let ground = sim.add_segment(-10.0, 0.0, 10.0, 0.0);
    let arch = sim.add_body(0.0, 0.0, 0.0, 2);
    sim.attach_polygon(
        arch,
        &[1.0, 0.0, 2.0, 0.0, 2.0, 1.0, 1.0, 1.0],
        0.0,
        1.0,
        0.6,
        0.0,
    );
    let rounded = sim.add_body(0.0, 2.0, 0.0, 2);
    sim.attach_rounded_box(rounded, 0.45, 0.45, 0.05, 1.0, 0.3, 0.0);
    let _ = ground;
    for _ in 0..20 {
        sim.step(1.0 / 60.0, 4);
    }
    assert!(sim.body_count() >= 3);
}

#[test]
fn shape_ops_material_filter_wind() {
    let mut sim = SimWorld::new(-10.0);
    let ground = sim.add_body(0.0, 0.0, 0.0, 0);
    let _seg = sim.attach_segment_filter(ground, -20.0, 0.0, 20.0, 0.0, 0x1, 0xffff_ffff);
    let body = sim.add_body(0.0, 2.0, 0.0, 2);
    let shape = sim.attach_box_mat(body, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, 0.8, 0.0, 0.1, 2.0);
    sim.shape_set_surface(shape, 0.5, 0.2, 0.05, 1.0);
    sim.shape_set_filter(shape, 0x2, 0x1 | 0x4);
    let f = sim.shape_get_filter(shape);
    assert_eq!(f[0], 0x2);
    let belt = sim.add_body(-5.0, 5.0, 0.0, 0);
    let _ = sim.attach_rounded_box_mat(belt, 10.0, 0.25, 0.25, 0.0, 0.8, 0.0, 0.0, 2.0);
    sim.apply_wind_to_body(body, 6.0, 0.0, 1.0, 0.75, true);
    let pts = [0.0_f32, 0.0, 10.0, 0.0, 10.0, 5.0, 0.0, 5.0, -1.0, 2.0];
    let mats = [
        0.6, 0.0, 0.0, -10.0, 0.6, 0.0, 0.0, -20.0, 0.6, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.6,
        0.0, 0.0, 0.0,
    ];
    let _chain = sim.add_chain_mat(&pts, true, &mats);
    sim.enable_odd_even_filter(true);
    for _ in 0..10 {
        sim.step(1.0 / 60.0, 4);
    }
    sim.enable_odd_even_filter(false);
}

#[test]
fn create_human_spawns_and_destroys() {
    // shared/human.c CreateHuman / DestroyHuman
    let mut sim = SimWorld::new(-10.0);
    sim.add_segment(-20.0, 0.0, 20.0, 0.0);
    let h = sim.create_human(0.0, 5.0, 1.0, 0.03, 5.0, 0.5, 1, false, 0);
    assert!(sim.human_is_spawned(h));
    // 1 ground + 11 bones
    assert_eq!(sim.body_count(), 12);
    // 10 revolute joints (hip has none)
    assert_eq!(sim.joint_count(), 10);
    for _ in 0..30 {
        sim.step(1.0 / 60.0, 4);
    }
    sim.human_set_joint_friction_torque(h, 0.1);
    sim.human_set_joint_spring_hertz(h, 2.0);
    sim.human_set_joint_damping_ratio(h, 0.5);
    sim.human_set_velocity(h, 1.0, 0.0);
    sim.human_apply_random_angular_impulse(h, 0.1);
    sim.human_set_scale(h, 1.5);
    sim.human_enable_sensor_events(h, true);
    sim.destroy_human(h);
    assert!(!sim.human_is_spawned(h));
    // Slot reuse
    let h2 = sim.create_human(0.0, 8.0, 1.0, 0.05, 1.0, 0.1, 2, true, 0);
    assert_eq!(h2, h);
    assert!(sim.human_is_spawned(h2));
    sim.destroy_human(h2);
}
