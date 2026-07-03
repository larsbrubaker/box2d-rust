// Capsule-vs-capsule manifold and wrappers from manifold.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::polygons::collide_polygons;
use super::{make_capsule_polygon, make_id};
use crate::collision::{Capsule, LocalManifold, Polygon, Segment};
use crate::constants::{linear_slop, speculative_distance};
use crate::math_functions::{
    add, clamp_float, distance_squared, dot, get_length_and_normalize, left_perp, lerp, mul_add,
    neg, normalize, sub, transform_point, Transform, VEC2_ZERO,
};

/// Compute the contact manifold between two capsules. (b2CollideCapsules)
///
/// Follows Ericson 5.1.9 Closest Points of Two Line Segments. Adds some logic
/// to support clipping to get two contact points.
pub fn collide_capsules(capsule_a: &Capsule, capsule_b: &Capsule, xf: Transform) -> LocalManifold {
    let origin = capsule_a.center1;

    // Shift to the origin in frame A for round-off, a pure translation in A's frame
    let xfs = Transform {
        p: sub(xf.p, origin),
        q: xf.q,
    };

    let p1 = VEC2_ZERO;
    let q1 = sub(capsule_a.center2, origin);

    let p2 = transform_point(xfs, capsule_b.center1);
    let q2 = transform_point(xfs, capsule_b.center2);

    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);

    let dd1 = dot(d1, d1);
    let dd2 = dot(d2, d2);

    let eps_sqr = f32::EPSILON * f32::EPSILON;
    debug_assert!(dd1 > eps_sqr && dd2 > eps_sqr);

    let r = sub(p1, p2);
    let rd1 = dot(r, d1);
    let rd2 = dot(r, d2);

    let d12 = dot(d1, d2);

    let denom = dd1 * dd2 - d12 * d12;

    // Fraction on segment 1
    let mut f1 = 0.0;
    if denom != 0.0 {
        // not parallel
        f1 = clamp_float((d12 * rd2 - rd1 * dd2) / denom, 0.0, 1.0);
    }

    // Compute point on segment 2 closest to p1 + f1 * d1
    let mut f2 = (d12 * f1 + rd2) / dd2;

    // Clamping of segment 2 requires a do over on segment 1
    if f2 < 0.0 {
        f2 = 0.0;
        f1 = clamp_float(-rd1 / dd1, 0.0, 1.0);
    } else if f2 > 1.0 {
        f2 = 1.0;
        f1 = clamp_float((d12 - rd1) / dd1, 0.0, 1.0);
    }

    let closest1 = mul_add(p1, f1, d1);
    let closest2 = mul_add(p2, f2, d2);
    let distance_sq = distance_squared(closest1, closest2);

    let mut manifold = LocalManifold::default();
    let radius_a = capsule_a.radius;
    let radius_b = capsule_b.radius;
    let radius = radius_a + radius_b;
    let max_distance = radius + speculative_distance();

    if distance_sq > max_distance * max_distance {
        return manifold;
    }

    let distance = distance_sq.sqrt();

    let (mut length1, mut length2) = (0.0, 0.0);
    let u1 = get_length_and_normalize(&mut length1, d1);
    let u2 = get_length_and_normalize(&mut length2, d2);

    // Does segment B project outside segment A?
    let fp2 = dot(sub(p2, p1), u1);
    let fq2 = dot(sub(q2, p1), u1);
    let outside_a = (fp2 <= 0.0 && fq2 <= 0.0) || (fp2 >= length1 && fq2 >= length1);

    // Does segment A project outside segment B?
    let fp1 = dot(sub(p1, p2), u2);
    let fq1 = dot(sub(q1, p2), u2);
    let outside_b = (fp1 <= 0.0 && fq1 <= 0.0) || (fp1 >= length2 && fq1 >= length2);

    if !outside_a && !outside_b {
        // attempt to clip
        // this may yield contact points with excessive separation
        // in that case the algorithm falls back to single point collision

        // find reference edge using SAT
        let mut normal_a;
        let separation_a;
        {
            normal_a = left_perp(u1);
            let ss1 = dot(sub(p2, p1), normal_a);
            let ss2 = dot(sub(q2, p1), normal_a);
            let s1p = if ss1 < ss2 { ss1 } else { ss2 };
            let s1n = if -ss1 < -ss2 { -ss1 } else { -ss2 };

            if s1p > s1n {
                separation_a = s1p;
            } else {
                separation_a = s1n;
                normal_a = neg(normal_a);
            }
        }

        let mut normal_b;
        let separation_b;
        {
            normal_b = left_perp(u2);
            let ss1 = dot(sub(p1, p2), normal_b);
            let ss2 = dot(sub(q1, p2), normal_b);
            let s1p = if ss1 < ss2 { ss1 } else { ss2 };
            let s1n = if -ss1 < -ss2 { -ss1 } else { -ss2 };

            if s1p > s1n {
                separation_b = s1p;
            } else {
                separation_b = s1n;
                normal_b = neg(normal_b);
            }
        }

        // biased to avoid feature flip-flop
        if separation_a + 0.1 * linear_slop() >= separation_b {
            manifold.normal = normal_a;

            let mut cp = p2;
            let mut cq = q2;

            // clip to p1
            if fp2 < 0.0 && fq2 > 0.0 {
                cp = lerp(p2, q2, (0.0 - fp2) / (fq2 - fp2));
            } else if fq2 < 0.0 && fp2 > 0.0 {
                cq = lerp(q2, p2, (0.0 - fq2) / (fp2 - fq2));
            }

            // clip to q1
            if fp2 > length1 && fq2 < length1 {
                cp = lerp(p2, q2, (fp2 - length1) / (fp2 - fq2));
            } else if fq2 > length1 && fp2 < length1 {
                cq = lerp(q2, p2, (fq2 - length1) / (fq2 - fp2));
            }

            let sp = dot(sub(cp, p1), normal_a);
            let sq = dot(sub(cq, p1), normal_a);

            if sp <= distance + linear_slop() || sq <= distance + linear_slop() {
                {
                    let mp = &mut manifold.points[0];
                    mp.point = mul_add(cp, 0.5 * (radius_a - radius_b - sp), normal_a);
                    mp.separation = sp - radius;
                    mp.id = make_id(0, 0);
                }
                {
                    let mp = &mut manifold.points[1];
                    mp.point = mul_add(cq, 0.5 * (radius_a - radius_b - sq), normal_a);
                    mp.separation = sq - radius;
                    mp.id = make_id(0, 1);
                }
                manifold.point_count = 2;
            }
        } else {
            // normal always points from A to B
            manifold.normal = neg(normal_b);

            let mut cp = p1;
            let mut cq = q1;

            // clip to p2
            if fp1 < 0.0 && fq1 > 0.0 {
                cp = lerp(p1, q1, (0.0 - fp1) / (fq1 - fp1));
            } else if fq1 < 0.0 && fp1 > 0.0 {
                cq = lerp(q1, p1, (0.0 - fq1) / (fp1 - fq1));
            }

            // clip to q2
            if fp1 > length2 && fq1 < length2 {
                cp = lerp(p1, q1, (fp1 - length2) / (fp1 - fq1));
            } else if fq1 > length2 && fp1 < length2 {
                cq = lerp(q1, p1, (fq1 - length2) / (fq1 - fp1));
            }

            let sp = dot(sub(cp, p2), normal_b);
            let sq = dot(sub(cq, p2), normal_b);

            if sp <= distance + linear_slop() || sq <= distance + linear_slop() {
                {
                    let mp = &mut manifold.points[0];
                    mp.point = mul_add(cp, 0.5 * (radius_b - radius_a - sp), normal_b);
                    mp.separation = sp - radius;
                    mp.id = make_id(0, 0);
                }
                {
                    let mp = &mut manifold.points[1];
                    mp.point = mul_add(cq, 0.5 * (radius_b - radius_a - sq), normal_b);
                    mp.separation = sq - radius;
                    mp.id = make_id(1, 0);
                }
                manifold.point_count = 2;
            }
        }
    }

    if manifold.point_count == 0 {
        // single point collision
        let mut normal = sub(closest2, closest1);
        if dot(normal, normal) > eps_sqr {
            normal = normalize(normal);
        } else {
            normal = left_perp(u1);
        }

        let c1 = mul_add(closest1, radius_a, normal);
        let c2 = mul_add(closest2, -radius_b, normal);

        let i1 = if f1 == 0.0 { 0 } else { 1 };
        let i2 = if f2 == 0.0 { 0 } else { 1 };

        manifold.normal = normal;
        manifold.points[0].point = lerp(c1, c2, 0.5);
        manifold.points[0].separation = distance_sq.sqrt() - radius;
        manifold.points[0].id = make_id(i1, i2);
        manifold.point_count = 1;
    }

    // Undo the origin shift so points are in frame A
    for i in 0..manifold.point_count as usize {
        manifold.points[i].point = add(manifold.points[i].point, origin);
    }

    manifold
}

/// Compute the contact manifold between a segment and a capsule.
/// (b2CollideSegmentAndCapsule)
pub fn collide_segment_and_capsule(
    segment_a: &Segment,
    capsule_b: &Capsule,
    xf: Transform,
) -> LocalManifold {
    let capsule_a = Capsule {
        center1: segment_a.point1,
        center2: segment_a.point2,
        radius: 0.0,
    };
    collide_capsules(&capsule_a, capsule_b, xf)
}

/// Compute the contact manifold between a polygon and a capsule.
/// (b2CollidePolygonAndCapsule)
pub fn collide_polygon_and_capsule(
    polygon_a: &Polygon,
    capsule_b: &Capsule,
    xf: Transform,
) -> LocalManifold {
    let poly_b = make_capsule_polygon(capsule_b.center1, capsule_b.center2, capsule_b.radius);
    collide_polygons(polygon_a, &poly_b, xf)
}
