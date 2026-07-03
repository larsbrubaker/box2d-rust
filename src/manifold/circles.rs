// Circle manifolds from manifold.c: circle-vs-circle, capsule-vs-circle,
// polygon-vs-circle, segment-vs-circle.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{Capsule, Circle, LocalManifold, Polygon, Segment};
use crate::constants::speculative_distance;
use crate::math_functions::{
    dot, get_length_and_normalize, lerp, mul_add, mul_sub, normalize, sub, transform_point,
    Transform,
};

/// Compute the contact manifold between two circles. (b2CollideCircles)
pub fn collide_circles(circle_a: &Circle, circle_b: &Circle, xf: Transform) -> LocalManifold {
    let mut manifold = LocalManifold::default();

    let point_a = circle_a.center;
    let point_b = transform_point(xf, circle_b.center);

    let mut distance = 0.0;
    let normal = get_length_and_normalize(&mut distance, sub(point_b, point_a));

    let radius_a = circle_a.radius;
    let radius_b = circle_b.radius;

    let separation = distance - radius_a - radius_b;
    if separation > speculative_distance() {
        return manifold;
    }

    let c_a = mul_add(point_a, radius_a, normal);
    let c_b = mul_add(point_b, -radius_b, normal);

    manifold.normal = normal;
    let mp = &mut manifold.points[0];
    mp.point = lerp(c_a, c_b, 0.5);
    mp.separation = separation;
    mp.id = 0;
    manifold.point_count = 1;
    manifold
}

/// Compute the contact manifold between a capsule and circle.
/// (b2CollideCapsuleAndCircle)
pub fn collide_capsule_and_circle(
    capsule_a: &Capsule,
    circle_b: &Circle,
    xf: Transform,
) -> LocalManifold {
    let mut manifold = LocalManifold::default();

    // Compute circle position in the frame of the capsule.
    let p_b = transform_point(xf, circle_b.center);

    // Compute closest point
    let p1 = capsule_a.center1;
    let p2 = capsule_a.center2;

    let e = sub(p2, p1);

    // dot(p - pA, e) = 0
    // dot(p - (p1 + s1 * e), e) = 0
    // s1 = dot(p - p1, e)
    let p_a;
    let s1 = dot(sub(p_b, p1), e);
    let s2 = dot(sub(p2, p_b), e);
    if s1 < 0.0 {
        // p1 region
        p_a = p1;
    } else if s2 < 0.0 {
        // p2 region
        p_a = p2;
    } else {
        // circle colliding with segment interior
        let s = s1 / dot(e, e);
        p_a = mul_add(p1, s, e);
    }

    let mut distance = 0.0;
    let normal = get_length_and_normalize(&mut distance, sub(p_b, p_a));

    let radius_a = capsule_a.radius;
    let radius_b = circle_b.radius;
    let separation = distance - radius_a - radius_b;
    if separation > speculative_distance() {
        return manifold;
    }

    let c_a = mul_add(p_a, radius_a, normal);
    let c_b = mul_add(p_b, -radius_b, normal);

    manifold.normal = normal;
    let mp = &mut manifold.points[0];
    mp.point = lerp(c_a, c_b, 0.5);
    mp.separation = separation;
    mp.id = 0;
    manifold.point_count = 1;
    manifold
}

/// Compute the contact manifold between a polygon and a circle.
/// (b2CollidePolygonAndCircle)
pub fn collide_polygon_and_circle(
    polygon_a: &Polygon,
    circle_b: &Circle,
    xf: Transform,
) -> LocalManifold {
    let mut manifold = LocalManifold::default();
    let speculative = speculative_distance();

    // Compute circle position in the frame of the polygon.
    let center = transform_point(xf, circle_b.center);
    let radius_a = polygon_a.radius;
    let radius_b = circle_b.radius;
    let radius = radius_a + radius_b;

    // Find the min separating edge.
    let mut normal_index = 0usize;
    let mut separation = -f32::MAX;
    let vertex_count = polygon_a.count as usize;
    let vertices = &polygon_a.vertices;
    let normals = &polygon_a.normals;

    for i in 0..vertex_count {
        let s = dot(normals[i], sub(center, vertices[i]));
        if s > separation {
            separation = s;
            normal_index = i;
        }
    }

    if separation > radius + speculative {
        return manifold;
    }

    // Vertices of the reference edge.
    let vert_index1 = normal_index;
    let vert_index2 = if vert_index1 + 1 < vertex_count {
        vert_index1 + 1
    } else {
        0
    };
    let v1 = vertices[vert_index1];
    let v2 = vertices[vert_index2];

    // Compute barycentric coordinates
    let u1 = dot(sub(center, v1), sub(v2, v1));
    let u2 = dot(sub(center, v2), sub(v1, v2));

    if u1 < 0.0 && separation > f32::EPSILON {
        // Circle center is closest to v1 and safely outside the polygon
        let normal = normalize(sub(center, v1));
        let separation = dot(sub(center, v1), normal);
        if separation > radius + speculative {
            return manifold;
        }

        let c_a = mul_add(v1, radius_a, normal);
        let c_b = mul_sub(center, radius_b, normal);
        let contact_point_a = lerp(c_a, c_b, 0.5);

        manifold.normal = normal;
        let mp = &mut manifold.points[0];
        mp.point = contact_point_a;
        mp.separation = dot(sub(c_b, c_a), normal);
        mp.id = 0;
        manifold.point_count = 1;
    } else if u2 < 0.0 && separation > f32::EPSILON {
        // Circle center is closest to v2 and safely outside the polygon
        let normal = normalize(sub(center, v2));
        let separation = dot(sub(center, v2), normal);
        if separation > radius + speculative {
            return manifold;
        }

        let c_a = mul_add(v2, radius_a, normal);
        let c_b = mul_sub(center, radius_b, normal);
        let contact_point_a = lerp(c_a, c_b, 0.5);

        manifold.normal = normal;
        let mp = &mut manifold.points[0];
        mp.point = contact_point_a;
        mp.separation = dot(sub(c_b, c_a), normal);
        mp.id = 0;
        manifold.point_count = 1;
    } else {
        // Circle center is between v1 and v2. Center may be inside polygon
        let normal = normals[normal_index];
        manifold.normal = normal;

        // cA is the projection of the circle center onto to the reference edge
        let c_a = mul_add(center, radius_a - dot(sub(center, v1), normal), normal);

        // cB is the deepest point on the circle with respect to the reference edge
        let c_b = mul_sub(center, radius_b, normal);

        // The contact point is the midpoint
        let mp = &mut manifold.points[0];
        mp.point = lerp(c_a, c_b, 0.5);
        mp.separation = separation - radius;
        mp.id = 0;
        manifold.point_count = 1;
    }

    manifold
}

/// Compute the contact manifold between a segment and a circle.
/// (b2CollideSegmentAndCircle)
pub fn collide_segment_and_circle(
    segment_a: &Segment,
    circle_b: &Circle,
    xf: Transform,
) -> LocalManifold {
    let capsule_a = Capsule {
        center1: segment_a.point1,
        center2: segment_a.point2,
        radius: 0.0,
    };
    collide_capsule_and_circle(&capsule_a, circle_b, xf)
}
