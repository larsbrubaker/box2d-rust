// Port of box2d-cpp-reference/test/test_shape.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{Capsule, Circle, RayCastInput, Segment};
use crate::geometry::*;
use crate::hull::compute_hull;
use crate::math_functions::{distance, make_world_transform, Vec2, PI, TRANSFORM_IDENTITY};

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

fn capsule() -> Capsule {
    Capsule {
        center1: v(-1.0, 0.0),
        center2: v(1.0, 0.0),
        radius: 1.0,
    }
}

fn circle() -> Circle {
    Circle {
        center: v(1.0, 0.0),
        radius: 1.0,
    }
}

fn segment() -> Segment {
    Segment {
        point1: v(0.0, 1.0),
        point2: v(0.0, -1.0),
    }
}

const N: usize = 4;

#[test]
fn shape_mass_test() {
    let box_ = make_box(1.0, 1.0);

    {
        let md = compute_circle_mass(&circle(), 1.0);
        ensure_small(md.mass - PI, f32::EPSILON);
        assert!(md.center.x == 1.0 && md.center.y == 0.0);
        ensure_small(md.rotational_inertia - 0.5 * PI, f32::EPSILON);
    }

    {
        let capsule = capsule();
        let radius = capsule.radius;
        let length = distance(capsule.center1, capsule.center2);

        let md = compute_capsule_mass(&capsule, 1.0);

        // Box that fully contains capsule
        let r = make_box(radius + 0.5 * length, radius);
        let md_upper = compute_polygon_mass(&r, 1.0);

        // Approximate capsule using convex hull
        let mut points = [Vec2::default(); 2 * N];
        let d = PI / (N as f32 - 1.0);
        let mut angle = -0.5 * PI;
        for point in points.iter_mut().take(N) {
            point.x = 1.0 + radius * angle.cos();
            point.y = radius * angle.sin();
            angle += d;
        }

        angle = 0.5 * PI;
        for point in points.iter_mut().skip(N) {
            point.x = -1.0 + radius * angle.cos();
            point.y = radius * angle.sin();
            angle += d;
        }

        let hull = compute_hull(&points);
        let ac = make_polygon(&hull, 0.0);
        let md_lower = compute_polygon_mass(&ac, 1.0);

        assert!(md_lower.mass < md.mass && md.mass < md_upper.mass);
        assert!(
            md_lower.rotational_inertia < md.rotational_inertia
                && md.rotational_inertia < md_upper.rotational_inertia
        );
    }

    {
        let md = compute_polygon_mass(&box_, 1.0);
        ensure_small(md.mass - 4.0, f32::EPSILON);
        ensure_small(md.center.x, f32::EPSILON);
        ensure_small(md.center.y, f32::EPSILON);
        ensure_small(md.rotational_inertia - 8.0 / 3.0, 2.0 * f32::EPSILON);
    }

    {
        let offset = v(0.4, -0.7);
        let b1 = make_box(0.25, 0.5);
        let b2 = make_offset_box(0.25, 0.5, offset, crate::math_functions::ROT_IDENTITY);

        let m1 = compute_polygon_mass(&b1, 1.0);
        let m2 = compute_polygon_mass(&b2, 1.0);

        ensure_small(m1.mass - m2.mass, f32::EPSILON);
        ensure_small(m1.rotational_inertia - m2.rotational_inertia, f32::EPSILON);
        ensure_small(m2.center.x - offset.x, f32::EPSILON);
        ensure_small(m2.center.y - offset.y, f32::EPSILON);
    }
}

#[test]
fn shape_aabb_test() {
    let box_ = make_box(1.0, 1.0);
    let identity = make_world_transform(TRANSFORM_IDENTITY);

    {
        let b = compute_circle_aabb(&circle(), identity);
        ensure_small(b.lower_bound.x, f32::EPSILON);
        ensure_small(b.lower_bound.y + 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.x - 2.0, f32::EPSILON);
        ensure_small(b.upper_bound.y - 1.0, f32::EPSILON);
    }

    {
        let b = compute_polygon_aabb(&box_, identity);
        ensure_small(b.lower_bound.x + 1.0, f32::EPSILON);
        ensure_small(b.lower_bound.y + 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.x - 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.y - 1.0, f32::EPSILON);
    }

    {
        let b = compute_segment_aabb(&segment(), identity);
        ensure_small(b.lower_bound.x, f32::EPSILON);
        ensure_small(b.lower_bound.y + 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.x, f32::EPSILON);
        ensure_small(b.upper_bound.y - 1.0, f32::EPSILON);
    }
}

#[test]
fn point_in_shape_test() {
    let box_ = make_box(1.0, 1.0);

    let p1 = v(0.5, 0.5);
    let p2 = v(4.0, -4.0);

    {
        let hit = point_in_circle(&circle(), p1);
        assert!(hit);
        let hit = point_in_circle(&circle(), p2);
        assert!(!hit);
    }

    {
        let hit = point_in_polygon(&box_, p1);
        assert!(hit);
        let hit = point_in_polygon(&box_, p2);
        assert!(!hit);
    }
}

#[test]
fn ray_cast_shape_test() {
    let box_ = make_box(1.0, 1.0);

    let input = RayCastInput {
        origin: v(-4.0, 0.0),
        translation: v(8.0, 0.0),
        max_fraction: 1.0,
    };

    {
        let output = ray_cast_circle(&circle(), &input);
        assert!(output.hit);
        ensure_small(output.normal.x + 1.0, f32::EPSILON);
        ensure_small(output.normal.y, f32::EPSILON);
        ensure_small(output.fraction - 0.5, f32::EPSILON);
    }

    {
        let output = ray_cast_polygon(&box_, &input);
        assert!(output.hit);
        ensure_small(output.normal.x + 1.0, f32::EPSILON);
        ensure_small(output.normal.y, f32::EPSILON);
        ensure_small(output.fraction - 3.0 / 8.0, f32::EPSILON);
    }

    {
        let output = ray_cast_segment(&segment(), &input, true);
        assert!(output.hit);
        ensure_small(output.normal.x + 1.0, f32::EPSILON);
        ensure_small(output.normal.y, f32::EPSILON);
        ensure_small(output.fraction - 0.5, f32::EPSILON);
    }
}

// Capsule point containment is not covered by test_shape.c; lock it in here.
#[test]
fn point_in_capsule_test() {
    let c = capsule();
    assert!(point_in_capsule(&c, v(0.0, 0.0)));
    assert!(point_in_capsule(&c, v(1.9, 0.0)));
    assert!(!point_in_capsule(&c, v(2.5, 0.0)));
    assert!(!point_in_capsule(&c, v(0.0, 1.5)));

    // Degenerate capsule (zero length) behaves as a circle.
    let degenerate = Capsule {
        center1: v(1.0, 1.0),
        center2: v(1.0, 1.0),
        radius: 0.5,
    };
    assert!(point_in_capsule(&degenerate, v(1.25, 1.0)));
    assert!(!point_in_capsule(&degenerate, v(2.0, 1.0)));
}
