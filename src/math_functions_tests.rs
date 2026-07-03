// Port of box2d-cpp-reference/test/test_math.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::math_functions::*;

// 0.0023 degrees
const ATAN_TOL: f32 = 0.00004;

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        value.abs() < tolerance,
        "|{value}| >= tolerance {tolerance}"
    );
}

#[test]
fn cos_sin_atan2_over_angle_sweep() {
    let mut t = -10.0f32;
    while t < 10.0 {
        let angle = PI * t;
        let r = make_rot(angle);
        let c = angle.cos();
        let s = angle.sin();

        // The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
        ensure_small(r.c - c, 0.002);
        ensure_small(r.s - s, 0.002);

        let xn = unwind_angle(angle);
        assert!((-PI..=PI).contains(&xn));

        let a = atan2(s, c);
        assert!(is_valid_float(a));

        let mut diff = abs_float(a - xn);

        // The two results can be off by 360 degrees (-pi and pi)
        if diff > PI {
            diff -= 2.0 * PI;
        }

        // The approximate atan2 is quite accurate
        ensure_small(diff, ATAN_TOL);

        t += 0.01;
    }
}

#[test]
fn atan2_matches_std_atan2_on_grid() {
    let mut y = -1.0f32;
    while y <= 1.0 {
        let mut x = -1.0f32;
        while x <= 1.0 {
            let a1 = atan2(y, x);
            let a2 = y.atan2(x);
            let diff = abs_float(a1 - a2);
            assert!(is_valid_float(a1));
            ensure_small(diff, ATAN_TOL);
            x += 0.01;
        }
        y += 0.01;
    }
}

#[test]
fn atan2_axis_cases() {
    for (y, x) in [
        (1.0f32, 0.0f32),
        (-1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
        (0.0, 0.0),
    ] {
        let a1 = atan2(y, x);
        let a2 = y.atan2(x);
        let diff = abs_float(a1 - a2);
        assert!(is_valid_float(a1));
        ensure_small(diff, ATAN_TOL);
    }
}

#[test]
fn vector_ops() {
    let zero = VEC2_ZERO;
    let one = Vec2 { x: 1.0, y: 1.0 };
    let two = Vec2 { x: 2.0, y: 2.0 };

    let v = add(one, two);
    assert!(v.x == 3.0 && v.y == 3.0);

    let v = sub(zero, two);
    assert!(v.x == -2.0 && v.y == -2.0);

    let v = add(two, two);
    assert!(v.x != 5.0 && v.y != 5.0);
}

#[test]
fn transform_composition_and_inverse() {
    let two = Vec2 { x: 2.0, y: 2.0 };

    let transform1 = Transform {
        p: Vec2 { x: -2.0, y: 3.0 },
        q: make_rot(1.0),
    };
    let transform2 = Transform {
        p: Vec2 { x: 1.0, y: 0.0 },
        q: make_rot(-2.0),
    };

    let transform = mul_transforms(transform2, transform1);

    let v = transform_point(transform2, transform_point(transform1, two));
    let u = transform_point(transform, two);

    ensure_small(u.x - v.x, 10.0 * f32::EPSILON);
    ensure_small(u.y - v.y, 10.0 * f32::EPSILON);

    let v = transform_point(transform1, two);
    let v = inv_transform_point(transform1, v);

    ensure_small(v.x - two.x, 8.0 * f32::EPSILON);
    ensure_small(v.y - two.y, 8.0 * f32::EPSILON);
}

#[test]
fn rotation_between_unit_vectors() {
    let v = normalize(Vec2 { x: 0.2, y: -0.5 });
    let mut y = -1.0f32;
    while y <= 1.0 {
        let mut x = -1.0f32;
        while x <= 1.0 {
            if x == 0.0 && y == 0.0 {
                x += 0.01;
                continue;
            }

            let u = normalize(Vec2 { x, y });

            let r = compute_rotation_between_unit_vectors(v, u);

            let w = rotate_vector(r, v);
            ensure_small(w.x - u.x, 4.0 * f32::EPSILON);
            ensure_small(w.y - u.y, 4.0 * f32::EPSILON);

            x += 0.01;
        }
        y += 0.01;
    }
}

#[test]
fn nlerp_error_bound() {
    // NLerp of Rot has an error of over 4 degrees.
    // 2D quaternions should have an error under 1 degree.
    let q1 = ROT_IDENTITY;
    let q2 = make_rot(0.5 * PI);
    let n = 100;
    for i in 0..=n {
        let alpha = i as f32 / n as f32;
        let q = nlerp(q1, q2, alpha);
        let angle = rot_get_angle(q);
        ensure_small(alpha * 0.5 * PI - angle, 5.0 * PI / 180.0);
    }
}

#[test]
fn relative_angle_matches_unwound_difference() {
    let base_angle = 0.75 * PI;
    let q1 = make_rot(base_angle);
    let mut t = -10.0f32;
    while t < 10.0 {
        let angle = PI * t;
        let q2 = make_rot(angle);

        let rel = relative_angle(q1, q2);
        let unwound = unwind_angle(angle - base_angle);
        let tolerance = 0.1 * PI / 180.0;
        ensure_small(rel - unwound, tolerance);

        t += 0.01;
    }
}

#[test]
fn world_position_boundary_helpers() {
    // World position boundary helpers. With large world mode off these collapse to the float
    // ops, so the round trips hold in both builds.
    let d = Vec2 { x: 0.25, y: -0.5 };
    let base = to_pos(Vec2 { x: 10.0, y: -20.0 });
    let p = offset_pos(base, d);
    let back = sub_pos(p, base);
    ensure_small(back.x - d.x, 8.0 * f32::EPSILON);
    ensure_small(back.y - d.y, 8.0 * f32::EPSILON);

    let r = to_vec2(base);
    assert!(r.x == 10.0 && r.y == -20.0);

    assert!(is_valid_position(p));
    assert!(is_valid_position(POS_ZERO));
    assert!(is_valid_world_transform(WORLD_TRANSFORM_IDENTITY));

    let wt = WorldTransform {
        p: to_pos(Vec2 { x: 3.0, y: -4.0 }),
        q: make_rot(0.7),
    };
    assert!(is_valid_world_transform(wt));

    // Local to world to local round trip
    let local = Vec2 { x: 1.5, y: 2.5 };
    let world = transform_world_point(wt, local);
    let back_local = inv_transform_world_point(wt, world);
    ensure_small(back_local.x - local.x, 8.0 * f32::EPSILON);
    ensure_small(back_local.y - local.y, 8.0 * f32::EPSILON);

    // Relative transform of B in A matches a float reference at modest coordinates
    let a = WorldTransform {
        p: to_pos(Vec2 { x: -2.0, y: 3.0 }),
        q: make_rot(1.0),
    };
    let b = WorldTransform {
        p: to_pos(Vec2 { x: 1.0, y: 0.0 }),
        q: make_rot(-2.0),
    };
    let rel = inv_mul_world_transforms(a, b);
    let ref_a = Transform {
        p: to_vec2(a.p),
        q: a.q,
    };
    let ref_b = Transform {
        p: to_vec2(b.p),
        q: b.q,
    };
    let reference = inv_mul_transforms(ref_a, ref_b);
    ensure_small(rel.p.x - reference.p.x, 8.0 * f32::EPSILON);
    ensure_small(rel.p.y - reference.p.y, 8.0 * f32::EPSILON);
}

/// Far from the origin a float vector cannot resolve sub meter motion, but a double world
/// position can. This is the whole point of large world mode.
#[cfg(feature = "double-precision")]
#[test]
fn large_world_resolves_sub_meter_motion() {
    let d = Vec2 { x: 0.25, y: -0.5 };
    let base = Pos { x: 1.0e7, y: 0.0 };
    let p = offset_pos(base, d);
    let back = sub_pos(p, base);
    ensure_small(back.x - d.x, 1.0e-4);
    ensure_small(back.y - d.y, 1.0e-4);
}
