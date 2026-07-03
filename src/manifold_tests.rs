// Manifold tests. Ports the LargeWorldManifoldTest from
// box2d-cpp-reference/test/test_collision.c (the far-from-origin portion is
// gated on double-precision, as in C), plus focused coverage of the other
// collide functions which have no dedicated C unit test (upstream exercises
// them through the samples app and determinism tests).
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{Capsule, Circle, Segment};
use crate::geometry::make_box;
use crate::manifold::*;
use crate::math_functions::{
    inv_mul_world_transforms, make_world_transform, offset_pos, Transform, Vec2, POS_ZERO,
    ROT_IDENTITY, TRANSFORM_IDENTITY, VEC2_ZERO, WORLD_TRANSFORM_IDENTITY,
};

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

/// Port of LargeWorldManifoldTest (test_collision.c). The narrow phase
/// differences the two world positions in double then works in frame A, so a
/// manifold far from the origin must match the same manifold at the origin.
#[test]
fn large_world_manifold() {
    let box_a = make_box(0.5, 0.5);
    let box_b = make_box(0.5, 0.5);

    // Centers 0.9 apart so the boxes overlap by 0.1 along x
    let sep = v(0.9, 0.0);

    let xf_ao = WORLD_TRANSFORM_IDENTITY;
    let xf_bo = crate::math_functions::WorldTransform {
        p: offset_pos(POS_ZERO, sep),
        q: ROT_IDENTITY,
    };
    let m_origin = collide_polygons(&box_a, &box_b, inv_mul_world_transforms(xf_ao, xf_bo));

    assert_eq!(m_origin.point_count, 2);
    ensure_small(m_origin.points[0].separation + 0.1, 0.01);
    ensure_small(m_origin.points[1].separation + 0.1, 0.01);

    #[cfg(feature = "double-precision")]
    {
        // Same relative configuration shifted far from the origin. The relative
        // pose differences the world positions in double, so in double the
        // frame A manifold is preserved to float precision. In float it would
        // collapse since the offset is below the ULP.
        let base = offset_pos(POS_ZERO, v(1.0e7, 1.0e7));
        let xf_al = crate::math_functions::WorldTransform {
            p: base,
            q: ROT_IDENTITY,
        };
        let xf_bl = crate::math_functions::WorldTransform {
            p: offset_pos(base, sep),
            q: ROT_IDENTITY,
        };
        let m_large = collide_polygons(&box_a, &box_b, inv_mul_world_transforms(xf_al, xf_bl));

        assert_eq!(m_large.point_count, m_origin.point_count);
        ensure_small(m_large.normal.x - m_origin.normal.x, 1e-4);
        ensure_small(m_large.normal.y - m_origin.normal.y, 1e-4);
        for i in 0..m_large.point_count as usize {
            ensure_small(
                m_large.points[i].separation - m_origin.points[i].separation,
                1e-4,
            );
            ensure_small(m_large.points[i].point.x - m_origin.points[i].point.x, 1e-4);
            ensure_small(m_large.points[i].point.y - m_origin.points[i].point.y, 1e-4);
        }
    }

    // Silence unused warnings in single precision.
    let _ = make_world_transform(TRANSFORM_IDENTITY);
}

#[test]
fn circle_manifolds() {
    // Two unit circles overlapping by 0.5 along x.
    let a = Circle {
        center: VEC2_ZERO,
        radius: 1.0,
    };
    let b = Circle {
        center: VEC2_ZERO,
        radius: 1.0,
    };
    let xf = Transform {
        p: v(1.5, 0.0),
        q: ROT_IDENTITY,
    };

    let m = collide_circles(&a, &b, xf);
    assert_eq!(m.point_count, 1);
    ensure_small(m.normal.x - 1.0, f32::EPSILON);
    ensure_small(m.normal.y, f32::EPSILON);
    ensure_small(m.points[0].separation + 0.5, 1e-6);
    ensure_small(m.points[0].point.x - 0.75, 1e-6);

    // Far apart: no contact.
    let far = Transform {
        p: v(10.0, 0.0),
        q: ROT_IDENTITY,
    };
    assert_eq!(collide_circles(&a, &b, far).point_count, 0);

    // Polygon vs circle: circle resting on top of a box.
    let box_a = make_box(1.0, 1.0);
    let circle_above = Circle {
        center: VEC2_ZERO,
        radius: 0.5,
    };
    let xf_above = Transform {
        p: v(0.0, 1.45),
        q: ROT_IDENTITY,
    };
    let m = collide_polygon_and_circle(&box_a, &circle_above, xf_above);
    assert_eq!(m.point_count, 1);
    ensure_small(m.normal.x, 1e-6);
    ensure_small(m.normal.y - 1.0, 1e-6);
    ensure_small(m.points[0].separation + 0.05, 1e-5);
}

#[test]
fn capsule_manifolds() {
    // Two horizontal capsules stacked with overlap.
    let a = Capsule {
        center1: v(-1.0, 0.0),
        center2: v(1.0, 0.0),
        radius: 0.5,
    };
    let b = Capsule {
        center1: v(-1.0, 0.0),
        center2: v(1.0, 0.0),
        radius: 0.5,
    };
    let xf = Transform {
        p: v(0.0, 0.9),
        q: ROT_IDENTITY,
    };

    let m = collide_capsules(&a, &b, xf);
    assert_eq!(m.point_count, 2);
    ensure_small(m.normal.x, 1e-6);
    ensure_small(m.normal.y - 1.0, 1e-6);
    ensure_small(m.points[0].separation + 0.1, 1e-5);
    ensure_small(m.points[1].separation + 0.1, 1e-5);

    // Segment vs capsule takes the same path with radiusA = 0.
    let seg = Segment {
        point1: v(-2.0, 0.0),
        point2: v(2.0, 0.0),
    };
    let xf_seg = Transform {
        p: v(0.0, 0.4),
        q: ROT_IDENTITY,
    };
    let m = collide_segment_and_capsule(&seg, &b, xf_seg);
    assert_eq!(m.point_count, 2);
    ensure_small(m.normal.y - 1.0, 1e-6);
    ensure_small(m.points[0].separation + 0.1, 1e-5);
}

#[test]
fn polygon_capsule_and_segment_manifolds() {
    let box_a = make_box(1.0, 1.0);

    // Capsule lying above the box.
    let cap = Capsule {
        center1: v(-0.5, 0.0),
        center2: v(0.5, 0.0),
        radius: 0.25,
    };
    let xf = Transform {
        p: v(0.0, 1.2),
        q: ROT_IDENTITY,
    };
    let m = collide_polygon_and_capsule(&box_a, &cap, xf);
    assert_eq!(m.point_count, 2);
    ensure_small(m.normal.y - 1.0, 1e-5);
    ensure_small(m.points[0].separation + 0.05, 1e-5);

    // Box resting on a ground segment.
    let ground = Segment {
        point1: v(-5.0, 0.0),
        point2: v(5.0, 0.0),
    };
    let xf_box = Transform {
        p: v(0.0, 0.95),
        q: ROT_IDENTITY,
    };
    let m = collide_segment_and_polygon(&ground, &box_a, xf_box);
    assert_eq!(m.point_count, 2);
    ensure_small(m.normal.y - 1.0, 1e-5);
    ensure_small(m.points[0].separation + 0.05, 1e-5);
}

#[test]
fn chain_segment_manifolds() {
    use crate::collision::ChainSegment;
    use crate::distance::SimplexCache;

    // A flat chain segment with collinear ghosts; circle above (right side).
    let chain = ChainSegment {
        ghost1: v(-2.0, 0.0),
        segment: Segment {
            point1: v(-1.0, 0.0),
            point2: v(1.0, 0.0),
        },
        ghost2: v(2.0, 0.0),
        chain_id: 0,
    };

    // Note: the chain normal points to the right of point1->point2, which is
    // -y here, so approach from below to hit the collision side.
    let circle = Circle {
        center: VEC2_ZERO,
        radius: 0.5,
    };
    let below = Transform {
        p: v(0.0, -0.45),
        q: ROT_IDENTITY,
    };
    let m = collide_chain_segment_and_circle(&chain, &circle, below);
    assert_eq!(m.point_count, 1);
    ensure_small(m.normal.y + 1.0, 1e-6);
    ensure_small(m.points[0].separation + 0.05, 1e-5);

    // Approaching from the left side is a miss (one-sided collision).
    let above = Transform {
        p: v(0.0, 0.45),
        q: ROT_IDENTITY,
    };
    assert_eq!(
        collide_chain_segment_and_circle(&chain, &circle, above).point_count,
        0
    );

    // Chain segment vs box from the collision side.
    let box_b = make_box(0.5, 0.5);
    let mut cache = SimplexCache::default();
    let m = collide_chain_segment_and_polygon(
        &chain,
        &box_b,
        Transform {
            p: v(0.0, -0.45),
            q: ROT_IDENTITY,
        },
        &mut cache,
    );
    assert_eq!(m.point_count, 2);
    ensure_small(m.normal.y + 1.0, 1e-5);

    // And a miss from the non-collision side.
    let mut cache2 = SimplexCache::default();
    let m = collide_chain_segment_and_polygon(
        &chain,
        &box_b,
        Transform {
            p: v(0.0, 0.45),
            q: ROT_IDENTITY,
        },
        &mut cache2,
    );
    assert_eq!(m.point_count, 0);
}
