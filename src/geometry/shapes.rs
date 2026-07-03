// Polygon constructors and transforms from geometry.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::Polygon;
use crate::hull::{validate_hull, Hull};
use crate::math_functions::{
    add, cross, cross_vs, dot, is_valid_float, mul_add, normalize, rotate_vector, sub,
    transform_point, Rot, Transform, Vec2, VEC2_ZERO,
};

pub(crate) fn compute_polygon_centroid(vertices: &[Vec2]) -> Vec2 {
    let mut center = VEC2_ZERO;
    let mut area = 0.0f32;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    let origin = vertices[0];

    let inv3 = 1.0 / 3.0;

    let count = vertices.len();
    for i in 1..count - 1 {
        // Triangle edges
        let e1 = sub(vertices[i], origin);
        let e2 = sub(vertices[i + 1], origin);
        let a = 0.5 * cross(e1, e2);

        // Area weighted centroid
        center = mul_add(center, a * inv3, add(e1, e2));
        area += a;
    }

    debug_assert!(area > f32::EPSILON);
    let inv_area = 1.0 / area;
    center.x *= inv_area;
    center.y *= inv_area;

    // Restore offset
    add(origin, center)
}

/// Make a convex polygon from a convex hull. This will assert if the hull is
/// not valid. (b2MakePolygon)
///
/// @warning Do not manually fill in the hull data, it must come directly from
/// `compute_hull`.
pub fn make_polygon(hull: &Hull, radius: f32) -> Polygon {
    debug_assert!(validate_hull(hull));

    if hull.count < 3 {
        // Handle a bad hull when assertions are disabled
        return make_square(0.5);
    }

    let mut shape = Polygon {
        count: hull.count,
        radius,
        ..Default::default()
    };

    // Copy vertices
    for i in 0..shape.count as usize {
        shape.vertices[i] = hull.points[i];
    }

    // Compute normals. Ensure the edges have non-zero length.
    for i in 0..shape.count as usize {
        let i1 = i;
        let i2 = if i + 1 < shape.count as usize {
            i + 1
        } else {
            0
        };
        let edge = sub(shape.vertices[i2], shape.vertices[i1]);
        debug_assert!(dot(edge, edge) > f32::EPSILON * f32::EPSILON);
        shape.normals[i] = normalize(cross_vs(edge, 1.0));
    }

    shape.centroid = compute_polygon_centroid(&shape.vertices[..shape.count as usize]);

    shape
}

/// Make an offset convex polygon from a convex hull. This will assert if the
/// hull is not valid. (b2MakeOffsetPolygon)
pub fn make_offset_polygon(hull: &Hull, position: Vec2, rotation: Rot) -> Polygon {
    make_offset_rounded_polygon(hull, position, rotation, 0.0)
}

/// Make an offset rounded convex polygon from a convex hull. This will assert
/// if the hull is not valid. (b2MakeOffsetRoundedPolygon)
pub fn make_offset_rounded_polygon(
    hull: &Hull,
    position: Vec2,
    rotation: Rot,
    radius: f32,
) -> Polygon {
    debug_assert!(validate_hull(hull));

    if hull.count < 3 {
        // Handle a bad hull when assertions are disabled
        return make_square(0.5);
    }

    let transform = Transform {
        p: position,
        q: rotation,
    };

    let mut shape = Polygon {
        count: hull.count,
        radius,
        ..Default::default()
    };

    // Copy vertices
    for i in 0..shape.count as usize {
        shape.vertices[i] = transform_point(transform, hull.points[i]);
    }

    // Compute normals. Ensure the edges have non-zero length.
    for i in 0..shape.count as usize {
        let i1 = i;
        let i2 = if i + 1 < shape.count as usize {
            i + 1
        } else {
            0
        };
        let edge = sub(shape.vertices[i2], shape.vertices[i1]);
        debug_assert!(dot(edge, edge) > f32::EPSILON * f32::EPSILON);
        shape.normals[i] = normalize(cross_vs(edge, 1.0));
    }

    shape.centroid = compute_polygon_centroid(&shape.vertices[..shape.count as usize]);

    shape
}

/// Make a square polygon, bypassing the need for a convex hull. (b2MakeSquare)
pub fn make_square(half_width: f32) -> Polygon {
    make_box(half_width, half_width)
}

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
/// (b2MakeBox)
pub fn make_box(half_width: f32, half_height: f32) -> Polygon {
    debug_assert!(is_valid_float(half_width) && half_width > 0.0);
    debug_assert!(is_valid_float(half_height) && half_height > 0.0);

    let mut shape = Polygon {
        count: 4,
        ..Default::default()
    };
    shape.vertices[0] = Vec2 {
        x: -half_width,
        y: -half_height,
    };
    shape.vertices[1] = Vec2 {
        x: half_width,
        y: -half_height,
    };
    shape.vertices[2] = Vec2 {
        x: half_width,
        y: half_height,
    };
    shape.vertices[3] = Vec2 {
        x: -half_width,
        y: half_height,
    };
    shape.normals[0] = Vec2 { x: 0.0, y: -1.0 };
    shape.normals[1] = Vec2 { x: 1.0, y: 0.0 };
    shape.normals[2] = Vec2 { x: 0.0, y: 1.0 };
    shape.normals[3] = Vec2 { x: -1.0, y: 0.0 };
    shape.radius = 0.0;
    shape.centroid = VEC2_ZERO;
    shape
}

/// Make a rounded box, bypassing the need for a convex hull. (b2MakeRoundedBox)
pub fn make_rounded_box(half_width: f32, half_height: f32, radius: f32) -> Polygon {
    debug_assert!(is_valid_float(radius) && radius >= 0.0);
    let mut shape = make_box(half_width, half_height);
    shape.radius = radius;
    shape
}

/// Make an offset box, bypassing the need for a convex hull. (b2MakeOffsetBox)
pub fn make_offset_box(half_width: f32, half_height: f32, center: Vec2, rotation: Rot) -> Polygon {
    let xf = Transform {
        p: center,
        q: rotation,
    };

    let mut shape = Polygon {
        count: 4,
        ..Default::default()
    };
    shape.vertices[0] = transform_point(
        xf,
        Vec2 {
            x: -half_width,
            y: -half_height,
        },
    );
    shape.vertices[1] = transform_point(
        xf,
        Vec2 {
            x: half_width,
            y: -half_height,
        },
    );
    shape.vertices[2] = transform_point(
        xf,
        Vec2 {
            x: half_width,
            y: half_height,
        },
    );
    shape.vertices[3] = transform_point(
        xf,
        Vec2 {
            x: -half_width,
            y: half_height,
        },
    );
    shape.normals[0] = rotate_vector(xf.q, Vec2 { x: 0.0, y: -1.0 });
    shape.normals[1] = rotate_vector(xf.q, Vec2 { x: 1.0, y: 0.0 });
    shape.normals[2] = rotate_vector(xf.q, Vec2 { x: 0.0, y: 1.0 });
    shape.normals[3] = rotate_vector(xf.q, Vec2 { x: -1.0, y: 0.0 });
    shape.radius = 0.0;
    shape.centroid = xf.p;
    shape
}

/// Make an offset rounded box, bypassing the need for a convex hull.
/// (b2MakeOffsetRoundedBox)
pub fn make_offset_rounded_box(
    half_width: f32,
    half_height: f32,
    center: Vec2,
    rotation: Rot,
    radius: f32,
) -> Polygon {
    debug_assert!(is_valid_float(radius) && radius >= 0.0);
    let mut shape = make_offset_box(half_width, half_height, center, rotation);
    shape.radius = radius;
    shape
}

/// Transform a polygon. This is useful for transferring a shape from one body
/// to another. (b2TransformPolygon)
pub fn transform_polygon(transform: Transform, polygon: &Polygon) -> Polygon {
    let mut p = *polygon;

    for i in 0..p.count as usize {
        p.vertices[i] = transform_point(transform, p.vertices[i]);
        p.normals[i] = rotate_vector(transform.q, p.normals[i]);
    }

    p.centroid = transform_point(transform, p.centroid);

    p
}
