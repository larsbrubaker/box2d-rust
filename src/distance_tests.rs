// Port of box2d-cpp-reference/test/test_distance.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::distance::{
    make_proxy, segment_distance, shape_cast, shape_distance, time_of_impact, DistanceInput,
    ShapeCastPairInput, SimplexCache, Sweep, ToiInput, ToiState,
};
use crate::math_functions::{Vec2, ROT_IDENTITY, TRANSFORM_IDENTITY, VEC2_ZERO};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn v(x: f32, y: f32) -> Vec2 {
    Vec2 { x, y }
}

#[test]
fn segment_distance_test() {
    let p1 = v(-1.0, -1.0);
    let q1 = v(-1.0, 1.0);
    let p2 = v(2.0, 0.0);
    let q2 = v(1.0, 0.0);

    let result = segment_distance(p1, q1, p2, q2);

    ensure_small(result.fraction1 - 0.5, f32::EPSILON);
    ensure_small(result.fraction2 - 1.0, f32::EPSILON);
    ensure_small(result.closest1.x + 1.0, f32::EPSILON);
    ensure_small(result.closest1.y, f32::EPSILON);
    ensure_small(result.closest2.x - 1.0, f32::EPSILON);
    ensure_small(result.closest2.y, f32::EPSILON);
    ensure_small(result.distance_squared - 4.0, f32::EPSILON);
}

#[test]
fn shape_distance_test() {
    let vas = [v(-1.0, -1.0), v(1.0, -1.0), v(1.0, 1.0), v(-1.0, 1.0)];
    let vbs = [v(2.0, -1.0), v(2.0, 1.0)];

    let input = DistanceInput {
        proxy_a: make_proxy(&vas, 0.0),
        proxy_b: make_proxy(&vbs, 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    ensure_small(output.distance - 1.0, f32::EPSILON);
}

#[test]
fn shape_cast_test() {
    let vas = [v(-1.0, -1.0), v(1.0, -1.0), v(1.0, 1.0), v(-1.0, 1.0)];
    let vbs = [v(2.0, -1.0), v(2.0, 1.0)];

    let input = ShapeCastPairInput {
        proxy_a: make_proxy(&vas, 0.0),
        proxy_b: make_proxy(&vbs, 0.0),
        transform: TRANSFORM_IDENTITY,
        translation_b: v(-2.0, 0.0),
        max_fraction: 1.0,
        // The C test leaves canEncroach uninitialized stack memory; the
        // meaningful configuration is false.
        can_encroach: false,
    };

    let output = shape_cast(&input);

    assert!(output.hit);
    ensure_small(output.fraction - 0.5, 0.005);
}

#[test]
fn time_of_impact_test() {
    let vas = [v(-1.0, -1.0), v(1.0, -1.0), v(1.0, 1.0), v(-1.0, 1.0)];
    let vbs = [v(2.0, -1.0), v(2.0, 1.0)];

    let input = ToiInput {
        proxy_a: make_proxy(&vas, 0.0),
        proxy_b: make_proxy(&vbs, 0.0),
        sweep_a: Sweep {
            local_center: VEC2_ZERO,
            c1: VEC2_ZERO,
            c2: VEC2_ZERO,
            q1: ROT_IDENTITY,
            q2: ROT_IDENTITY,
        },
        sweep_b: Sweep {
            local_center: VEC2_ZERO,
            c1: VEC2_ZERO,
            c2: v(-2.0, 0.0),
            q1: ROT_IDENTITY,
            q2: ROT_IDENTITY,
        },
        max_fraction: 1.0,
    };

    let output = time_of_impact(&input);

    assert!(output.state == ToiState::Hit);
    ensure_small(output.fraction - 0.5, 0.005);
}
