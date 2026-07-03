// Port of box2d-cpp-reference/src/aabb.h and src/aabb.c
//
// b2IsValidAABB is declared in math_functions.h and lives in math_functions.rs;
// the AABB inline queries (contains/center/extents/union/overlaps) live there
// too. This module holds the pieces that are specific to aabb.c/aabb.h: the
// perimeter, in-place enlarge, world-space offset, and ray cast.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// In double precision Pos coordinates are already f64, so the `as f64` promotions
// in offset_aabb are the identity; in single precision they are real promotions.
// Keeping the cast lets one expression compile in both modes.
#![allow(clippy::unnecessary_cast)]

use crate::collision::CastOutput;
use crate::math_functions::{
    abs, lerp, min_float, round_down_float, round_up_float, sub, Aabb, Pos, Vec2, VEC2_ZERO,
};

/// Get the surface area of an AABB (the perimeter length). (b2Perimeter)
pub fn perimeter(a: Aabb) -> f32 {
    let wx = a.upper_bound.x - a.lower_bound.x;
    let wy = a.upper_bound.y - a.lower_bound.y;
    2.0 * (wx + wy)
}

/// Enlarge `a` to contain `b`. (b2EnlargeAABB)
///
/// @return true if the AABB grew.
pub fn enlarge_aabb(a: &mut Aabb, b: Aabb) -> bool {
    let mut changed = false;
    if b.lower_bound.x < a.lower_bound.x {
        a.lower_bound.x = b.lower_bound.x;
        changed = true;
    }

    if b.lower_bound.y < a.lower_bound.y {
        a.lower_bound.y = b.lower_bound.y;
        changed = true;
    }

    if a.upper_bound.x < b.upper_bound.x {
        a.upper_bound.x = b.upper_bound.x;
        changed = true;
    }

    if a.upper_bound.y < b.upper_bound.y {
        a.upper_bound.y = b.upper_bound.y;
        changed = true;
    }

    changed
}

/// Translate a relative AABB into world space, rounding outward so the float box
/// always contains the true box far from the origin. (b2OffsetAABB)
///
/// Float ULP at 1e8 dwarfs the AABB margin, so plain truncation could clip a
/// shape out of its own box; the broadphase pair order rides on the
/// deterministic directed rounding. In single precision this collapses to a
/// plain sum since [`round_down_float`]/[`round_up_float`] are the identity.
pub fn offset_aabb(box_: Aabb, origin: Pos) -> Aabb {
    Aabb {
        lower_bound: Vec2 {
            x: round_down_float(origin.x as f64 + box_.lower_bound.x as f64),
            y: round_down_float(origin.y as f64 + box_.lower_bound.y as f64),
        },
        upper_bound: Vec2 {
            x: round_up_float(origin.x as f64 + box_.upper_bound.x as f64),
            y: round_up_float(origin.y as f64 + box_.upper_bound.y as f64),
        },
    }
}

/// Ray cast an AABB. Radius is not handled. (b2AABB_RayCast)
// From Real-time Collision Detection, p179.
pub fn aabb_ray_cast(a: Aabb, p1: Vec2, p2: Vec2) -> CastOutput {
    let mut output = CastOutput::default();

    let mut t_min = -f32::MAX;
    let mut t_max = f32::MAX;

    let p = p1;
    let d = sub(p2, p1);
    let abs_d = abs(d);

    let mut normal = VEC2_ZERO;

    // x-coordinate
    if abs_d.x < f32::EPSILON {
        // parallel
        if p.x < a.lower_bound.x || a.upper_bound.x < p.x {
            return output;
        }
    } else {
        let inv_d = 1.0 / d.x;
        let mut t1 = (a.lower_bound.x - p.x) * inv_d;
        let mut t2 = (a.upper_bound.x - p.x) * inv_d;

        // Sign of the normal vector.
        let mut s = -1.0;

        if t1 > t2 {
            core::mem::swap(&mut t1, &mut t2);
            s = 1.0;
        }

        // Push the min up
        if t1 > t_min {
            normal.y = 0.0;
            normal.x = s;
            t_min = t1;
        }

        // Pull the max down
        t_max = min_float(t_max, t2);

        if t_min > t_max {
            return output;
        }
    }

    // y-coordinate
    if abs_d.y < f32::EPSILON {
        // parallel
        if p.y < a.lower_bound.y || a.upper_bound.y < p.y {
            return output;
        }
    } else {
        let inv_d = 1.0 / d.y;
        let mut t1 = (a.lower_bound.y - p.y) * inv_d;
        let mut t2 = (a.upper_bound.y - p.y) * inv_d;

        // Sign of the normal vector.
        let mut s = -1.0;

        if t1 > t2 {
            core::mem::swap(&mut t1, &mut t2);
            s = 1.0;
        }

        // Push the min up
        if t1 > t_min {
            normal.x = 0.0;
            normal.y = s;
            t_min = t1;
        }

        // Pull the max down
        t_max = min_float(t_max, t2);

        if t_min > t_max {
            return output;
        }
    }

    // Does the ray start inside the box?
    if t_min < 0.0 {
        return output;
    }

    // Does the ray intersect beyond the segment length?
    if 1.0 < t_min {
        return output;
    }

    // Intersection.
    output.fraction = t_min;
    output.normal = normal;
    output.point = lerp(p1, p2, t_min);
    output.hit = true;
    output
}
