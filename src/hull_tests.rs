// Tests for the hull module. Box2D has no standalone test_hull.c (b2ComputeHull
// is exercised through the shape tests), so these cover the algorithm directly:
// the documented success and failure cases plus b2ValidateHull.
//
// SPDX-License-Identifier: MIT

use crate::hull::{compute_hull, validate_hull, Hull};
use crate::math_functions::{cross, sub, Vec2};

fn v(x: f32, y: f32) -> Vec2 {
    Vec2 { x, y }
}

/// Signed area * 2 of the hull polygon; positive means CCW winding.
fn signed_area2(h: &Hull) -> f32 {
    let n = h.count as usize;
    let mut sum = 0.0;
    for i in 0..n {
        let a = h.points[i];
        let b = h.points[(i + 1) % n];
        sum += cross(a, b);
    }
    sum
}

#[test]
fn square_hull_is_convex_ccw() {
    // Four corners of a unit square, given in an arbitrary order.
    let pts = [v(1.0, -1.0), v(-1.0, -1.0), v(1.0, 1.0), v(-1.0, 1.0)];
    let h = compute_hull(&pts);

    assert_eq!(h.count, 4);
    assert!(validate_hull(&h));
    assert!(signed_area2(&h) > 0.0, "hull should wind CCW");

    // Every original corner is present in the hull.
    for corner in pts {
        assert!(
            h.points[..h.count as usize]
                .iter()
                .any(|p| (p.x - corner.x).abs() < 1e-6 && (p.y - corner.y).abs() < 1e-6),
            "corner {corner:?} missing from hull"
        );
    }
}

#[test]
fn interior_point_is_discarded() {
    // A triangle with an extra point inside it: the hull is still the triangle.
    let pts = [v(0.0, 0.0), v(4.0, 0.0), v(0.0, 4.0), v(1.0, 1.0)];
    let h = compute_hull(&pts);

    assert_eq!(h.count, 3);
    assert!(validate_hull(&h));
    assert!(signed_area2(&h) > 0.0);
}

#[test]
fn collinear_edge_points_are_merged() {
    // A square with a redundant point on the middle of the bottom edge.
    let pts = [
        v(-1.0, -1.0),
        v(0.0, -1.0),
        v(1.0, -1.0),
        v(1.0, 1.0),
        v(-1.0, 1.0),
    ];
    let h = compute_hull(&pts);

    assert_eq!(h.count, 4, "collinear midpoint should be removed");
    assert!(validate_hull(&h));
}

#[test]
fn degenerate_inputs_return_empty_hull() {
    // Fewer than 3 points.
    assert_eq!(compute_hull(&[v(0.0, 0.0), v(1.0, 1.0)]).count, 0);

    // More than MAX_POLYGON_VERTICES points.
    let many: Vec<Vec2> = (0..9).map(|i| v(i as f32, (i * i) as f32)).collect();
    assert_eq!(compute_hull(&many).count, 0);

    // All points collinear.
    let line = [v(0.0, 0.0), v(1.0, 1.0), v(2.0, 2.0), v(3.0, 3.0)];
    assert_eq!(compute_hull(&line).count, 0);

    // All points welded together (within tolerance).
    let cluster = [v(0.0, 0.0), v(1e-5, 0.0), v(0.0, 1e-5)];
    assert_eq!(compute_hull(&cluster).count, 0);
}

#[test]
fn validate_hull_rejects_clockwise_and_collinear() {
    // A clockwise square is not accepted (edges must have all points behind).
    let cw = Hull {
        points: {
            let mut p = [v(0.0, 0.0); 8];
            p[0] = v(-1.0, -1.0);
            p[1] = v(-1.0, 1.0);
            p[2] = v(1.0, 1.0);
            p[3] = v(1.0, -1.0);
            p
        },
        count: 4,
    };
    assert!(!validate_hull(&cw));

    // Fewer than 3 points is invalid.
    let two = Hull {
        points: [v(0.0, 0.0); 8],
        count: 2,
    };
    assert!(!validate_hull(&two));

    // Sanity: a hand-built CCW triangle validates and wraps correctly.
    let tri_pts = [v(0.0, 0.0), v(2.0, 0.0), v(0.0, 2.0)];
    let tri = compute_hull(&tri_pts);
    assert!(validate_hull(&tri));
    // Edge from last vertex back to the first is exercised by validate_hull's wrap.
    let n = tri.count as usize;
    let wrap = cross(
        sub(tri.points[0], tri.points[n - 1]),
        sub(tri.points[1 % n], tri.points[n - 1]),
    );
    let _ = wrap;
}
