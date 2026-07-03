// Port of box2d-cpp-reference/src/distance.c and the distance group of
// include/box2d/collision.h.
//
// Split to satisfy the 800-line file limit:
// - types.rs — proxies, simplex cache, distance/cast/TOI inputs and outputs
// - gjk.rs   — the GJK distance algorithm (b2ShapeDistance) and simplex solvers
// - cast.rs  — linear shape cast via conservative advancement (b2ShapeCast)
// - toi.rs   — time of impact via local separating axes (b2TimeOfImpact)
//
// This file holds the standalone helpers: sweep evaluation, segment-segment
// distance, and proxy construction.
//
// The experimental `b2ShapeCastMerged` in distance.c sits inside `#if 0` and is
// not ported. The B2_SNOOP_TOI_COUNTERS globals are profiling-only and disabled
// in normal builds; they are not ported.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

mod cast;
mod gjk;
mod toi;
mod types;

pub use cast::shape_cast;
pub use gjk::shape_distance;
pub use toi::time_of_impact;
pub use types::{
    DistanceInput, DistanceOutput, SegmentDistanceResult, ShapeCastPairInput, ShapeProxy, Simplex,
    SimplexCache, SimplexVertex, Sweep, ToiInput, ToiOutput, ToiState,
};

use crate::hull::MAX_POLYGON_VERTICES;
use crate::math_functions::{
    add, clamp_float, distance_squared, dot, min_int, mul_add, mul_sv, normalize_rot,
    rotate_vector, sub, transform_point, Rot, Transform, Vec2,
};

/// Evaluate the transform sweep at a specific time. (b2GetSweepTransform)
pub fn get_sweep_transform(sweep: &Sweep, time: f32) -> Transform {
    // https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
    let mut xf = Transform {
        p: add(mul_sv(1.0 - time, sweep.c1), mul_sv(time, sweep.c2)),
        q: normalize_rot(Rot {
            c: (1.0 - time) * sweep.q1.c + time * sweep.q2.c,
            s: (1.0 - time) * sweep.q1.s + time * sweep.q2.s,
        }),
    };

    // Shift to origin
    xf.p = sub(xf.p, rotate_vector(xf.q, sweep.local_center));
    xf
}

/// Compute the distance between two line segments, clamping at the end points
/// if needed. (b2SegmentDistance)
///
/// Follows Ericson 5.1.9 Closest Points of Two Line Segments.
pub fn segment_distance(p1: Vec2, q1: Vec2, p2: Vec2, q2: Vec2) -> SegmentDistanceResult {
    let mut result = SegmentDistanceResult::default();

    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);
    let dd1 = dot(d1, d1);
    let dd2 = dot(d2, d2);
    let rd1 = dot(r, d1);
    let rd2 = dot(r, d2);

    let eps_sqr = f32::EPSILON * f32::EPSILON;

    if dd1 < eps_sqr || dd2 < eps_sqr {
        // Handle all degeneracies
        if dd1 >= eps_sqr {
            // Segment 2 is degenerate
            result.fraction1 = clamp_float(-rd1 / dd1, 0.0, 1.0);
            result.fraction2 = 0.0;
        } else if dd2 >= eps_sqr {
            // Segment 1 is degenerate
            result.fraction1 = 0.0;
            result.fraction2 = clamp_float(rd2 / dd2, 0.0, 1.0);
        } else {
            result.fraction1 = 0.0;
            result.fraction2 = 0.0;
        }
    } else {
        // Non-degenerate segments
        let d12 = dot(d1, d2);

        let denominator = dd1 * dd2 - d12 * d12;

        // Fraction on segment 1
        let mut f1 = 0.0;
        if denominator != 0.0 {
            // not parallel
            f1 = clamp_float((d12 * rd2 - rd1 * dd2) / denominator, 0.0, 1.0);
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

        result.fraction1 = f1;
        result.fraction2 = f2;
    }

    result.closest1 = mul_add(p1, result.fraction1, d1);
    result.closest2 = mul_add(p2, result.fraction2, d2);
    result.distance_squared = distance_squared(result.closest1, result.closest2);
    result
}

/// Make a proxy for use in overlap, shape cast, and related functions. This is
/// a deep copy of the points. (b2MakeProxy)
pub fn make_proxy(points: &[Vec2], radius: f32) -> ShapeProxy {
    let count = min_int(points.len() as i32, MAX_POLYGON_VERTICES as i32);
    let mut proxy = ShapeProxy {
        points: [Vec2::default(); MAX_POLYGON_VERTICES],
        count,
        radius,
    };
    proxy.points[..count as usize].copy_from_slice(&points[..count as usize]);
    proxy
}

/// Make a proxy with a transform. This is a deep copy of the points.
/// (b2MakeOffsetProxy)
pub fn make_offset_proxy(
    points: &[Vec2],
    radius: f32,
    position: Vec2,
    rotation: Rot,
) -> ShapeProxy {
    let count = min_int(points.len() as i32, MAX_POLYGON_VERTICES as i32);
    let transform = Transform {
        p: position,
        q: rotation,
    };
    let mut proxy = ShapeProxy {
        points: [Vec2::default(); MAX_POLYGON_VERTICES],
        count,
        radius,
    };
    for (dst, src) in proxy.points[..count as usize]
        .iter_mut()
        .zip(&points[..count as usize])
    {
        *dst = transform_point(transform, *src);
    }
    proxy
}
