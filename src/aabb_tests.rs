// Port of the AABB subtests from box2d-cpp-reference/test/test_collision.c
// (AABBTest and AABBRayCastTest). The LargeWorld subtests depend on geometry
// modules not yet ported and will be added with them.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::aabb::aabb_ray_cast;
use crate::math_functions::{aabb_contains, aabb_overlaps, is_valid_aabb, Aabb, Vec2};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn aabb(lx: f32, ly: f32, ux: f32, uy: f32) -> Aabb {
    Aabb {
        lower_bound: Vec2 { x: lx, y: ly },
        upper_bound: Vec2 { x: ux, y: uy },
    }
}

#[test]
fn aabb_validity_overlap_contains() {
    let mut a = aabb(-1.0, -1.0, -2.0, -2.0);
    assert!(!is_valid_aabb(a));

    a.upper_bound = Vec2 { x: 1.0, y: 1.0 };
    assert!(is_valid_aabb(a));

    let b = aabb(2.0, 2.0, 4.0, 4.0);
    assert!(!aabb_overlaps(a, b));
    assert!(!aabb_contains(a, b));
}

#[test]
fn aabb_ray_cast_cases() {
    // AABB centered at origin with bounds [-1, -1] to [1, 1]
    let a = aabb(-1.0, -1.0, 1.0, 1.0);
    let eps = f32::EPSILON;
    let v = |x, y| Vec2 { x, y };

    // Test 1: Ray hits AABB from left side
    let o = aabb_ray_cast(a, v(-3.0, 0.0), v(3.0, 0.0));
    assert!(o.hit);
    ensure_small(o.fraction - 1.0 / 3.0, eps);
    ensure_small(o.normal.x + 1.0, eps);
    ensure_small(o.normal.y, eps);
    ensure_small(o.point.x + 1.0, eps);
    ensure_small(o.point.y, eps);

    // Test 2: Ray hits AABB from right side
    let o = aabb_ray_cast(a, v(3.0, 0.0), v(-3.0, 0.0));
    assert!(o.hit);
    ensure_small(o.fraction - 1.0 / 3.0, eps);
    ensure_small(o.normal.x - 1.0, eps);
    ensure_small(o.normal.y, eps);
    ensure_small(o.point.x - 1.0, eps);
    ensure_small(o.point.y, eps);

    // Test 3: Ray hits AABB from bottom
    let o = aabb_ray_cast(a, v(0.0, -3.0), v(0.0, 3.0));
    assert!(o.hit);
    ensure_small(o.fraction - 1.0 / 3.0, eps);
    ensure_small(o.normal.x, eps);
    ensure_small(o.normal.y + 1.0, eps);
    ensure_small(o.point.x, eps);
    ensure_small(o.point.y + 1.0, eps);

    // Test 4: Ray hits AABB from top
    let o = aabb_ray_cast(a, v(0.0, 3.0), v(0.0, -3.0));
    assert!(o.hit);
    ensure_small(o.fraction - 1.0 / 3.0, eps);
    ensure_small(o.normal.x, eps);
    ensure_small(o.normal.y - 1.0, eps);
    ensure_small(o.point.x, eps);
    ensure_small(o.point.y - 1.0, eps);

    // Test 5: Ray misses AABB completely (parallel to x-axis)
    assert!(!aabb_ray_cast(a, v(-3.0, 2.0), v(3.0, 2.0)).hit);

    // Test 6: Ray misses AABB completely (parallel to y-axis)
    assert!(!aabb_ray_cast(a, v(2.0, -3.0), v(2.0, 3.0)).hit);

    // Test 7: Ray starts inside AABB
    assert!(!aabb_ray_cast(a, v(0.0, 0.0), v(2.0, 0.0)).hit);

    // Test 8: Ray hits corner of AABB (diagonal ray)
    let o = aabb_ray_cast(a, v(-2.0, -2.0), v(2.0, 2.0));
    assert!(o.hit);
    ensure_small(o.fraction - 0.25, eps);
    // Normal should be either (-1, 0) or (0, -1) depending on which edge is hit first
    assert!((o.normal.x == -1.0 && o.normal.y == 0.0) || (o.normal.x == 0.0 && o.normal.y == -1.0));

    // Test 9: Ray parallel to AABB edge but outside
    assert!(!aabb_ray_cast(a, v(-2.0, 1.5), v(2.0, 1.5)).hit);

    // Test 10: Ray parallel to AABB edge and exactly on boundary
    let o = aabb_ray_cast(a, v(-2.0, 1.0), v(2.0, 1.0));
    assert!(o.hit);
    ensure_small(o.fraction - 0.25, eps);
    ensure_small(o.normal.x + 1.0, eps);
    ensure_small(o.normal.y, eps);

    // Test 11: Very short ray that doesn't reach AABB
    assert!(!aabb_ray_cast(a, v(-3.0, 0.0), v(-2.5, 0.0)).hit);

    // Test 12: Zero-length ray (degenerate case)
    assert!(!aabb_ray_cast(a, v(0.0, 0.0), v(0.0, 0.0)).hit);

    // Test 13: Ray hits AABB at exact boundary condition (t = 1.0)
    let o = aabb_ray_cast(a, v(-2.0, 0.0), v(-1.0, 0.0));
    assert!(o.hit);
    ensure_small(o.fraction - 1.0, eps);
    ensure_small(o.normal.x + 1.0, eps);
    ensure_small(o.normal.y, eps);

    // Test 14: Different AABB position (not centered at origin)
    let offset_aabb = aabb(2.0, 3.0, 4.0, 5.0);
    let o = aabb_ray_cast(offset_aabb, v(0.0, 4.0), v(6.0, 4.0));
    assert!(o.hit);
    ensure_small(o.fraction - 1.0 / 3.0, eps);
    ensure_small(o.normal.x + 1.0, eps);
    ensure_small(o.normal.y, eps);
    ensure_small(o.point.x - 2.0, eps);
    ensure_small(o.point.y - 4.0, eps);
}
