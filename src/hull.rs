// Port of box2d-cpp-reference/src/hull.c, plus the b2Hull type and
// B2_MAX_POLYGON_VERTICES from include/box2d/collision.h.
//
// quickhull:
// - merges vertices based on the linear slop
// - removes collinear points using the linear slop
// - returns an empty hull if it fails
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// This is a line-by-line port of a quickhull that relies on explicit index
// arithmetic — welding by comparing ps[i] against ps[j<i], swapping removed
// points with ps[n-1], and walking modular triples (i, i+1, i+2) % count.
// Rewriting these as iterators would obscure the correspondence to the C.
#![allow(clippy::needless_range_loop)]

use crate::constants::linear_slop;
use crate::math_functions::{
    aabb_center, cross, distance_squared, max, min, min_int, normalize, sub, Aabb, Vec2, VEC2_ZERO,
};

/// The maximum number of vertices on a convex polygon. Changing this affects
/// performance even if you don't use more vertices. (B2_MAX_POLYGON_VERTICES)
pub const MAX_POLYGON_VERTICES: usize = 8;

/// A convex hull. Used to construct convex polygons. (b2Hull)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hull {
    /// The final points of the hull
    pub points: [Vec2; MAX_POLYGON_VERTICES],
    /// The number of points
    pub count: i32,
}

impl Default for Hull {
    fn default() -> Self {
        Hull {
            points: [VEC2_ZERO; MAX_POLYGON_VERTICES],
            count: 0,
        }
    }
}

// quickhull recursion
fn recurse_hull(p1: Vec2, p2: Vec2, ps: &[Vec2]) -> Hull {
    let mut hull = Hull::default();

    let count = ps.len();
    if count == 0 {
        return hull;
    }

    // create an edge vector pointing from p1 to p2
    let e = normalize(sub(p2, p1));

    // discard points left of e and find point furthest to the right of e
    let mut right_points = [VEC2_ZERO; MAX_POLYGON_VERTICES];
    let mut right_count = 0;

    let mut best_index = 0;
    let mut best_distance = cross(sub(ps[best_index], p1), e);
    if best_distance > 0.0 {
        right_points[right_count] = ps[best_index];
        right_count += 1;
    }

    for i in 1..count {
        let distance = cross(sub(ps[i], p1), e);
        if distance > best_distance {
            best_index = i;
            best_distance = distance;
        }

        if distance > 0.0 {
            right_points[right_count] = ps[i];
            right_count += 1;
        }
    }

    if best_distance < 2.0 * linear_slop() {
        return hull;
    }

    let best_point = ps[best_index];

    // compute hull to the right of p1-bestPoint
    let hull1 = recurse_hull(p1, best_point, &right_points[..right_count]);

    // compute hull to the right of bestPoint-p2
    let hull2 = recurse_hull(best_point, p2, &right_points[..right_count]);

    // stitch together hulls
    for i in 0..hull1.count as usize {
        hull.points[hull.count as usize] = hull1.points[i];
        hull.count += 1;
    }

    hull.points[hull.count as usize] = best_point;
    hull.count += 1;

    for i in 0..hull2.count as usize {
        hull.points[hull.count as usize] = hull2.points[i];
        hull.count += 1;
    }

    debug_assert!(hull.count < MAX_POLYGON_VERTICES as i32);

    hull
}

/// Compute the convex hull of a set of points. Returns an empty hull if it
/// fails. (b2ComputeHull)
///
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - fewer than 3 points
/// - more than [`MAX_POLYGON_VERTICES`] points
pub fn compute_hull(points: &[Vec2]) -> Hull {
    let mut hull = Hull::default();

    let mut count = points.len() as i32;

    if count < 3 || count > MAX_POLYGON_VERTICES as i32 {
        // check your data
        return hull;
    }

    count = min_int(count, MAX_POLYGON_VERTICES as i32);

    let mut aabb = Aabb {
        lower_bound: Vec2 {
            x: f32::MAX,
            y: f32::MAX,
        },
        upper_bound: Vec2 {
            x: -f32::MAX,
            y: -f32::MAX,
        },
    };

    // Perform aggressive point welding. First point always remains.
    // Also compute the bounding box for later.
    let mut ps = [VEC2_ZERO; MAX_POLYGON_VERTICES];
    let mut n = 0;
    let slop = linear_slop();
    let tol_sqr = 16.0 * slop * slop;
    for i in 0..count as usize {
        aabb.lower_bound = min(aabb.lower_bound, points[i]);
        aabb.upper_bound = max(aabb.upper_bound, points[i]);

        let vi = points[i];

        let mut unique = true;
        for j in 0..i {
            let vj = points[j];

            let dist_sqr = distance_squared(vi, vj);
            if dist_sqr < tol_sqr {
                unique = false;
                break;
            }
        }

        if unique {
            ps[n] = vi;
            n += 1;
        }
    }

    if n < 3 {
        // all points very close together, check your data and check your scale
        return hull;
    }

    // Find an extreme point as the first point on the hull
    let c = aabb_center(aabb);
    let mut f1 = 0;
    let mut dsq1 = distance_squared(c, ps[f1]);
    for i in 1..n {
        let dsq = distance_squared(c, ps[i]);
        if dsq > dsq1 {
            f1 = i;
            dsq1 = dsq;
        }
    }

    // remove p1 from working set
    let p1 = ps[f1];
    ps[f1] = ps[n - 1];
    n -= 1;

    let mut f2 = 0;
    let mut dsq2 = distance_squared(p1, ps[f2]);
    for i in 1..n {
        let dsq = distance_squared(p1, ps[i]);
        if dsq > dsq2 {
            f2 = i;
            dsq2 = dsq;
        }
    }

    // remove p2 from working set
    let p2 = ps[f2];
    ps[f2] = ps[n - 1];
    n -= 1;

    // split the points into points that are left and right of the line p1-p2.
    let mut right_points = [VEC2_ZERO; MAX_POLYGON_VERTICES - 2];
    let mut right_count = 0;

    let mut left_points = [VEC2_ZERO; MAX_POLYGON_VERTICES - 2];
    let mut left_count = 0;

    let e = normalize(sub(p2, p1));

    for i in 0..n {
        let d = cross(sub(ps[i], p1), e);

        // slop used here to skip points that are very close to the line p1-p2
        if d >= 2.0 * slop {
            right_points[right_count] = ps[i];
            right_count += 1;
        } else if d <= -2.0 * slop {
            left_points[left_count] = ps[i];
            left_count += 1;
        }
    }

    // compute hulls on right and left
    let hull1 = recurse_hull(p1, p2, &right_points[..right_count]);
    let hull2 = recurse_hull(p2, p1, &left_points[..left_count]);

    if hull1.count == 0 && hull2.count == 0 {
        // all points collinear
        return hull;
    }

    // stitch hulls together, preserving CCW winding order
    hull.points[hull.count as usize] = p1;
    hull.count += 1;

    for i in 0..hull1.count as usize {
        hull.points[hull.count as usize] = hull1.points[i];
        hull.count += 1;
    }

    hull.points[hull.count as usize] = p2;
    hull.count += 1;

    for i in 0..hull2.count as usize {
        hull.points[hull.count as usize] = hull2.points[i];
        hull.count += 1;
    }

    debug_assert!(hull.count <= MAX_POLYGON_VERTICES as i32);

    // merge collinear
    let mut searching = true;
    while searching && hull.count > 2 {
        searching = false;

        for i in 0..hull.count as usize {
            let i1 = i;
            let i2 = (i + 1) % hull.count as usize;
            let i3 = (i + 2) % hull.count as usize;

            let s1 = hull.points[i1];
            let s2 = hull.points[i2];
            let s3 = hull.points[i3];

            // unit edge vector for s1-s3
            let r = normalize(sub(s3, s1));

            let distance = cross(sub(s2, s1), r);
            if distance <= 2.0 * slop {
                // remove midpoint from hull
                for j in i2..(hull.count as usize - 1) {
                    hull.points[j] = hull.points[j + 1];
                }
                hull.count -= 1;

                // continue searching for collinear points
                searching = true;

                break;
            }
        }
    }

    if hull.count < 3 {
        // all points collinear, shouldn't be reached since this was validated above
        hull.count = 0;
    }

    hull
}

/// Validate that a hull is convex, CCW, and has no collinear points.
/// (b2ValidateHull)
pub fn validate_hull(hull: &Hull) -> bool {
    if hull.count < 3 || (MAX_POLYGON_VERTICES as i32) < hull.count {
        return false;
    }

    let count = hull.count as usize;

    // test that every point is behind every edge
    for i in 0..count {
        // create an edge vector
        let i1 = i;
        let i2 = if i < count - 1 { i1 + 1 } else { 0 };
        let p = hull.points[i1];
        let e = normalize(sub(hull.points[i2], p));

        for (j, &point) in hull.points[..count].iter().enumerate() {
            // skip points that subtend the current edge
            if j == i1 || j == i2 {
                continue;
            }

            let distance = cross(sub(point, p), e);
            if distance >= 0.0 {
                return false;
            }
        }
    }

    // test for collinear points
    let slop = linear_slop();
    for i in 0..count {
        let i1 = i;
        let i2 = (i + 1) % count;
        let i3 = (i + 2) % count;

        let p1 = hull.points[i1];
        let p2 = hull.points[i2];
        let p3 = hull.points[i3];

        let e = normalize(sub(p3, p1));

        let distance = cross(sub(p2, p1), e);
        if distance <= slop {
            // p1-p2-p3 are collinear
            return false;
        }
    }

    true
}
