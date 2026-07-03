// Point-in-shape tests and ray casts from geometry.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::is_valid_ray;
use crate::collision::{Capsule, CastOutput, Circle, Polygon, RayCastInput, Segment};
use crate::distance::{
    make_proxy, shape_cast, shape_distance, DistanceInput, ShapeCastPairInput, SimplexCache,
};
use crate::math_functions::{
    add, clamp_float, cross, distance_squared, dot, get_length_and_normalize, length_squared, lerp,
    mul_add, mul_sub, mul_sv, neg, normalize, right_perp, sub, Vec2, TRANSFORM_IDENTITY,
};

/// Test a point for overlap with a circle in local space. (b2PointInCircle)
pub fn point_in_circle(shape: &Circle, point: Vec2) -> bool {
    let center = shape.center;
    distance_squared(point, center) <= shape.radius * shape.radius
}

/// Test a point for overlap with a capsule in local space. (b2PointInCapsule)
pub fn point_in_capsule(shape: &Capsule, point: Vec2) -> bool {
    let rr = shape.radius * shape.radius;
    let p1 = shape.center1;
    let p2 = shape.center2;

    let d = sub(p2, p1);
    let dd = dot(d, d);
    if dd == 0.0 {
        // Capsule is really a circle
        return distance_squared(point, p1) <= rr;
    }

    // Get closest point on capsule segment
    // c = p1 + t * d
    // dot(point - c, d) = 0
    // dot(point - p1 - t * d, d) = 0
    // t = dot(point - p1, d) / dot(d, d)
    let mut t = dot(sub(point, p1), d) / dd;
    t = clamp_float(t, 0.0, 1.0);
    let c = mul_add(p1, t, d);

    // Is query point within radius around closest point?
    distance_squared(point, c) <= rr
}

/// Test a point for overlap with a convex polygon in local space.
/// (b2PointInPolygon)
pub fn point_in_polygon(shape: &Polygon, point: Vec2) -> bool {
    let input = DistanceInput {
        proxy_a: make_proxy(&shape.vertices[..shape.count as usize], 0.0),
        proxy_b: make_proxy(&[point], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    output.distance <= shape.radius
}

/// Ray cast versus circle shape in local space. (b2RayCastCircle)
// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
pub fn ray_cast_circle(shape: &Circle, input: &RayCastInput) -> CastOutput {
    debug_assert!(is_valid_ray(input));

    let p = shape.center;

    let mut output = CastOutput::default();

    // Shift ray so circle center is the origin
    let s = sub(input.origin, p);

    let r = shape.radius;
    let rr = r * r;

    let mut length = 0.0;
    let d = get_length_and_normalize(&mut length, input.translation);
    if length == 0.0 {
        // zero length ray

        if length_squared(s) < rr {
            // initial overlap
            output.point = input.origin;
            output.hit = true;
        }

        return output;
    }

    // Find closest point on ray to origin

    // solve: dot(s + t * d, d) = 0
    let t = -dot(s, d);

    // c is the closest point on the line to the origin
    let c = mul_add(s, t, d);

    let cc = dot(c, c);

    if cc > rr {
        // closest point is outside the circle
        return output;
    }

    // Pythagoras
    let h = (rr - cc).sqrt();

    let fraction = t - h;

    if fraction < 0.0 || input.max_fraction * length < fraction {
        // intersection is point outside the range of the ray segment

        if length_squared(s) < rr {
            // initial overlap
            output.point = input.origin;
            output.hit = true;
        }

        return output;
    }

    // hit point relative to center
    let hit_point = mul_add(s, fraction, d);

    output.fraction = fraction / length;
    output.normal = normalize(hit_point);
    output.point = mul_add(p, shape.radius, output.normal);
    output.hit = true;

    output
}

/// Ray cast versus capsule shape in local space. (b2RayCastCapsule)
pub fn ray_cast_capsule(shape: &Capsule, input: &RayCastInput) -> CastOutput {
    debug_assert!(is_valid_ray(input));

    let mut output = CastOutput::default();

    let v1 = shape.center1;
    let v2 = shape.center2;

    let e = sub(v2, v1);

    let mut capsule_length = 0.0;
    let a = get_length_and_normalize(&mut capsule_length, e);

    if capsule_length < f32::EPSILON {
        // Capsule is really a circle
        let circle = Circle {
            center: v1,
            radius: shape.radius,
        };
        return ray_cast_circle(&circle, input);
    }

    let p1 = input.origin;
    let d = input.translation;

    // Ray from capsule start to ray start
    let q = sub(p1, v1);
    let qa = dot(q, a);

    // Vector to ray start that is perpendicular to capsule axis
    let qp = mul_add(q, -qa, a);

    let radius = shape.radius;

    // Does the ray start within the infinite length capsule?
    if dot(qp, qp) < radius * radius {
        if qa < 0.0 {
            // start point behind capsule segment
            let circle = Circle {
                center: v1,
                radius: shape.radius,
            };
            return ray_cast_circle(&circle, input);
        }

        if qa > capsule_length {
            // start point ahead of capsule segment
            let circle = Circle {
                center: v2,
                radius: shape.radius,
            };
            return ray_cast_circle(&circle, input);
        }

        // ray starts inside capsule -> no hit
        output.point = input.origin;
        output.hit = true;
        return output;
    }

    // Perpendicular to capsule axis, pointing right
    let mut n = Vec2 { x: a.y, y: -a.x };

    let mut ray_length = 0.0;
    let u = get_length_and_normalize(&mut ray_length, d);

    // Intersect ray with infinite length capsule
    // v1 + radius * n + s1 * a = p1 + s2 * u
    // v1 - radius * n + s1 * a = p1 + s2 * u

    // s1 * a - s2 * u = b
    // b = q - radius * ap
    // or
    // b = q + radius * ap

    // Cramer's rule [a -u]
    let den = -a.x * u.y + u.x * a.y;
    if -f32::EPSILON < den && den < f32::EPSILON {
        // Ray is parallel to capsule and outside infinite length capsule
        return output;
    }

    let b1 = mul_sub(q, radius, n);
    let b2 = mul_add(q, radius, n);

    let inv_den = 1.0 / den;

    // Cramer's rule [a b1]
    let s21 = (a.x * b1.y - b1.x * a.y) * inv_den;

    // Cramer's rule [a b2]
    let s22 = (a.x * b2.y - b2.x * a.y) * inv_den;

    let s2;
    let b;
    if s21 < s22 {
        s2 = s21;
        b = b1;
    } else {
        s2 = s22;
        b = b2;
        n = neg(n);
    }

    if s2 < 0.0 || input.max_fraction * ray_length < s2 {
        return output;
    }

    // Cramer's rule [b -u]
    let s1 = (-b.x * u.y + u.x * b.y) * inv_den;

    if s1 < 0.0 {
        // ray passes behind capsule segment
        let circle = Circle {
            center: v1,
            radius: shape.radius,
        };
        ray_cast_circle(&circle, input)
    } else if capsule_length < s1 {
        // ray passes ahead of capsule segment
        let circle = Circle {
            center: v2,
            radius: shape.radius,
        };
        ray_cast_circle(&circle, input)
    } else {
        // ray hits capsule side
        output.fraction = s2 / ray_length;
        output.point = add(lerp(v1, v2, s1 / capsule_length), mul_sv(shape.radius, n));
        output.normal = n;
        output.hit = true;
        output
    }
}

/// Ray cast versus segment shape in local space. Optionally treat the segment
/// as one-sided with hits from the left side being treated as a miss.
/// (b2RayCastSegment)
// Ray vs line segment
pub fn ray_cast_segment(shape: &Segment, input: &RayCastInput, one_sided: bool) -> CastOutput {
    if one_sided {
        // Skip left-side collision
        let offset = cross(
            sub(input.origin, shape.point1),
            sub(shape.point2, shape.point1),
        );
        if offset < 0.0 {
            return CastOutput::default();
        }
    }

    // Put the ray into the edge's frame of reference.
    let p1 = input.origin;
    let d = input.translation;

    let v1 = shape.point1;
    let v2 = shape.point2;
    let e = sub(v2, v1);

    let mut output = CastOutput::default();

    let mut length = 0.0;
    let e_unit = get_length_and_normalize(&mut length, e);
    if length == 0.0 {
        return output;
    }

    // Normal points to the right, looking from v1 towards v2
    let mut normal = right_perp(e_unit);

    // Intersect ray with infinite segment using normal
    // Similar to intersecting a ray with an infinite plane
    // p = p1 + t * d
    // dot(normal, p - v1) = 0
    // dot(normal, p1 - v1) + t * dot(normal, d) = 0
    let numerator = dot(normal, sub(v1, p1));
    let denominator = dot(normal, d);

    if denominator == 0.0 {
        // parallel
        return output;
    }

    let t = numerator / denominator;
    if t < 0.0 || input.max_fraction < t {
        // out of ray range
        return output;
    }

    // Intersection point on infinite segment
    let p = mul_add(p1, t, d);

    // Compute position of p along segment
    // p = v1 + s * e
    // s = dot(p - v1, e) / dot(e, e)

    let s = dot(sub(p, v1), e_unit);
    if s < 0.0 || length < s {
        // out of segment range
        return output;
    }

    if numerator > 0.0 {
        normal = neg(normal);
    }

    output.fraction = t;
    output.point = p;
    output.normal = normal;
    output.hit = true;

    output
}

/// Ray cast versus polygon shape in local space. (b2RayCastPolygon)
pub fn ray_cast_polygon(shape: &Polygon, input: &RayCastInput) -> CastOutput {
    debug_assert!(is_valid_ray(input));

    if shape.radius == 0.0 {
        // Shift all math to first vertex since the polygon may be far
        // from the origin.
        let base = shape.vertices[0];

        let p1 = sub(input.origin, base);
        let d = input.translation;

        let (mut lower, mut upper) = (0.0f32, input.max_fraction);

        let mut index = -1i32;

        let mut output = CastOutput::default();

        for edge_index in 0..shape.count as usize {
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            let vertex = sub(shape.vertices[edge_index], base);
            let numerator = dot(shape.normals[edge_index], sub(vertex, p1));
            let denominator = dot(shape.normals[edge_index], d);

            if denominator == 0.0 {
                // Parallel and runs outside edge
                if numerator < 0.0 {
                    return output;
                }
            } else {
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower > numerator.
                if denominator < 0.0 && numerator < lower * denominator {
                    // Increase lower.
                    // The segment enters this half-space.
                    lower = numerator / denominator;
                    index = edge_index as i32;
                } else if denominator > 0.0 && numerator < upper * denominator {
                    // Decrease upper.
                    // The segment exits this half-space.
                    upper = numerator / denominator;
                }
            }

            if upper < lower {
                // Ray misses
                return output;
            }
        }

        debug_assert!(0.0 <= lower && lower <= input.max_fraction);

        if index >= 0 {
            output.fraction = lower;
            output.normal = shape.normals[index as usize];
            output.point = mul_add(input.origin, lower, d);
            output.hit = true;
        } else {
            // initial overlap
            output.point = input.origin;
            output.hit = true;
        }

        return output;
    }

    let cast_input = ShapeCastPairInput {
        proxy_a: make_proxy(&shape.vertices[..shape.count as usize], shape.radius),
        proxy_b: make_proxy(&[input.origin], 0.0),
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: false,
    };
    shape_cast(&cast_input)
}
