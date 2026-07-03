// AABB queries, plane separation, and the spring-damper helper.
// Part of the math_functions module.

use super::*;

/// Does a fully contain b
pub fn aabb_contains(a: Aabb, b: Aabb) -> bool {
    let mut s = true;
    s = s && a.lower_bound.x <= b.lower_bound.x;
    s = s && a.lower_bound.y <= b.lower_bound.y;
    s = s && b.upper_bound.x <= a.upper_bound.x;
    s = s && b.upper_bound.y <= a.upper_bound.y;
    s
}

/// Get the center of the AABB.
pub fn aabb_center(a: Aabb) -> Vec2 {
    Vec2 {
        x: 0.5 * (a.lower_bound.x + a.upper_bound.x),
        y: 0.5 * (a.lower_bound.y + a.upper_bound.y),
    }
}

/// Get the extents of the AABB (half-widths).
pub fn aabb_extents(a: Aabb) -> Vec2 {
    Vec2 {
        x: 0.5 * (a.upper_bound.x - a.lower_bound.x),
        y: 0.5 * (a.upper_bound.y - a.lower_bound.y),
    }
}

/// Union of two AABBs
pub fn aabb_union(a: Aabb, b: Aabb) -> Aabb {
    Aabb {
        lower_bound: Vec2 {
            x: min_float(a.lower_bound.x, b.lower_bound.x),
            y: min_float(a.lower_bound.y, b.lower_bound.y),
        },
        upper_bound: Vec2 {
            x: max_float(a.upper_bound.x, b.upper_bound.x),
            y: max_float(a.upper_bound.y, b.upper_bound.y),
        },
    }
}

/// Do a and b overlap
pub fn aabb_overlaps(a: Aabb, b: Aabb) -> bool {
    !(b.lower_bound.x > a.upper_bound.x
        || b.lower_bound.y > a.upper_bound.y
        || a.lower_bound.x > b.upper_bound.x
        || a.lower_bound.y > b.upper_bound.y)
}

/// Compute the bounding box of an array of points
pub fn make_aabb(points: &[Vec2], radius: f32) -> Aabb {
    debug_assert!(!points.is_empty());
    let mut a = Aabb {
        lower_bound: points[0],
        upper_bound: points[0],
    };
    for point in &points[1..] {
        a.lower_bound = min(a.lower_bound, *point);
        a.upper_bound = max(a.upper_bound, *point);
    }

    let r = Vec2 {
        x: radius,
        y: radius,
    };
    a.lower_bound = sub(a.lower_bound, r);
    a.upper_bound = add(a.upper_bound, r);

    a
}

/// Signed separation of a point from a plane
pub fn plane_separation(plane: Plane, point: Vec2) -> f32 {
    dot(plane.normal, point) - plane.offset
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += time_step * new_velocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
pub fn spring_damper(
    hertz: f32,
    damping_ratio: f32,
    position: f32,
    velocity: f32,
    time_step: f32,
) -> f32 {
    let omega = 2.0 * PI * hertz;
    let omega_h = omega * time_step;
    (velocity - omega * omega_h * position)
        / (1.0 + 2.0 * damping_ratio * omega_h + omega_h * omega_h)
}
