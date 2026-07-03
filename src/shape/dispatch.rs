// Per-shape-type dispatch helpers from shape.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{Shape, ShapeExtent};
use crate::broad_phase::{proxy_type, BroadPhase};
use crate::collision::ShapeGeometry;
use crate::collision::{CastOutput, MassData, PlaneResult, RayCastInput, ShapeCastInput};
use crate::constants::speculative_distance;
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, ShapeProxy};
use crate::geometry::{
    collide_mover_and_capsule, collide_mover_and_circle, collide_mover_and_polygon,
    collide_mover_and_segment, compute_capsule_aabb, compute_capsule_mass, compute_circle_aabb,
    compute_circle_mass, compute_fat_shape_aabb, compute_polygon_aabb, compute_polygon_mass,
    compute_segment_aabb, ray_cast_capsule, ray_cast_circle, ray_cast_polygon, ray_cast_segment,
    shape_cast_capsule, shape_cast_circle, shape_cast_polygon, shape_cast_segment,
};
use crate::math_functions::{
    abs_float, add, cross, dot, inv_rotate_vector, inv_transform_point, length, length_squared,
    lerp, max_float, min_float, mul_sv, rotate_vector, sub, transform_point, Aabb, Transform, Vec2,
    WorldTransform, PI,
};
use crate::types::BodyType;

/// Compute the shape AABB and fat AABB with speculative and movement margins.
/// (static b2UpdateShapeAABBs)
pub(crate) fn update_shape_aabbs(
    shape: &mut Shape,
    transform: WorldTransform,
    proxy_type_: BodyType,
) {
    // Compute a bounding box with a speculative margin
    let speculative = speculative_distance();
    let aabb_margin = shape.aabb_margin;

    let aabb = compute_fat_shape_aabb(&shape.geometry, transform, speculative);
    shape.aabb = aabb;

    // Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
    let margin = if proxy_type_ == BodyType::Static {
        speculative
    } else {
        aabb_margin
    };
    shape.fat_aabb = Aabb {
        lower_bound: Vec2 {
            x: aabb.lower_bound.x - margin,
            y: aabb.lower_bound.y - margin,
        },
        upper_bound: Vec2 {
            x: aabb.upper_bound.x + margin,
            y: aabb.upper_bound.y + margin,
        },
    };
}

/// (b2CreateShapeProxy)
pub fn create_shape_proxy(
    shape: &mut Shape,
    bp: &mut BroadPhase,
    type_: BodyType,
    transform: WorldTransform,
    force_pair_creation: bool,
) {
    debug_assert!(shape.proxy_key == NULL_INDEX);

    update_shape_aabbs(shape, transform, type_);

    // Create proxies in the broad-phase.
    shape.proxy_key = bp.create_proxy(
        type_,
        shape.fat_aabb,
        shape.filter.category_bits,
        shape.id,
        force_pair_creation,
    );
    debug_assert!((proxy_type(shape.proxy_key) as usize) < crate::types::BODY_TYPE_COUNT);
}

/// (b2DestroyShapeProxy)
pub fn destroy_shape_proxy(shape: &mut Shape, bp: &mut BroadPhase) {
    if shape.proxy_key != NULL_INDEX {
        bp.destroy_proxy(shape.proxy_key);
        shape.proxy_key = NULL_INDEX;
    }
}

/// (b2ComputeShapeMass)
pub fn compute_shape_mass(shape: &Shape) -> MassData {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_mass(capsule, shape.density),
        ShapeGeometry::Circle(circle) => compute_circle_mass(circle, shape.density),
        ShapeGeometry::Polygon(polygon) => compute_polygon_mass(polygon, shape.density),
        _ => MassData::default(),
    }
}

/// (b2ComputeShapeExtent)
pub fn compute_shape_extent(shape: &Shape, local_center: Vec2) -> ShapeExtent {
    let mut extent = ShapeExtent::default();

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let radius = capsule.radius;
            extent.min_extent = radius;
            let c1 = sub(capsule.center1, local_center);
            let c2 = sub(capsule.center2, local_center);
            extent.max_extent = max_float(length_squared(c1), length_squared(c2)).sqrt() + radius;
        }

        ShapeGeometry::Circle(circle) => {
            let radius = circle.radius;
            extent.min_extent = radius;
            extent.max_extent = length(sub(circle.center, local_center)) + radius;
        }

        ShapeGeometry::Polygon(poly) => {
            let mut min_extent = crate::constants::huge();
            let mut max_extent_sqr = 0.0f32;
            let count = poly.count as usize;
            for i in 0..count {
                let v = poly.vertices[i];
                let plane_offset = dot(poly.normals[i], sub(v, poly.centroid));
                min_extent = min_float(min_extent, plane_offset);

                let distance_sqr = length_squared(sub(v, local_center));
                max_extent_sqr = max_float(max_extent_sqr, distance_sqr);
            }

            extent.min_extent = min_extent + poly.radius;
            extent.max_extent = max_extent_sqr.sqrt() + poly.radius;
        }

        ShapeGeometry::Segment(segment) => {
            extent.min_extent = 0.0;
            let c1 = sub(segment.point1, local_center);
            let c2 = sub(segment.point2, local_center);
            extent.max_extent = max_float(length_squared(c1), length_squared(c2)).sqrt();
        }

        ShapeGeometry::ChainSegment(chain_segment) => {
            extent.min_extent = 0.0;
            let c1 = sub(chain_segment.segment.point1, local_center);
            let c2 = sub(chain_segment.segment.point2, local_center);
            extent.max_extent = max_float(length_squared(c1), length_squared(c2)).sqrt();
        }
    }

    extent
}

/// (b2ComputeShapeAABB)
pub fn compute_shape_aabb(shape: &Shape, xf: WorldTransform) -> Aabb {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_aabb(capsule, xf),
        ShapeGeometry::Circle(circle) => compute_circle_aabb(circle, xf),
        ShapeGeometry::Polygon(polygon) => compute_polygon_aabb(polygon, xf),
        ShapeGeometry::Segment(segment) => compute_segment_aabb(segment, xf),
        ShapeGeometry::ChainSegment(chain_segment) => {
            compute_segment_aabb(&chain_segment.segment, xf)
        }
    }
}

/// (b2GetShapeCentroid)
pub fn get_shape_centroid(shape: &Shape) -> Vec2 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => lerp(capsule.center1, capsule.center2, 0.5),
        ShapeGeometry::Circle(circle) => circle.center,
        ShapeGeometry::Polygon(polygon) => polygon.centroid,
        ShapeGeometry::Segment(segment) => lerp(segment.point1, segment.point2, 0.5),
        ShapeGeometry::ChainSegment(chain_segment) => lerp(
            chain_segment.segment.point1,
            chain_segment.segment.point2,
            0.5,
        ),
    }
}

/// (b2GetShapePerimeter)
pub fn get_shape_perimeter(shape: &Shape) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            2.0 * length(sub(capsule.center1, capsule.center2)) + 2.0 * PI * capsule.radius
        }
        ShapeGeometry::Circle(circle) => 2.0 * PI * circle.radius,
        ShapeGeometry::Polygon(polygon) => {
            let points = &polygon.vertices;
            let count = polygon.count as usize;
            let mut perimeter = 2.0 * PI * polygon.radius;
            debug_assert!(count > 0);
            let mut prev = points[count - 1];
            for &next in points.iter().take(count) {
                perimeter += length(sub(next, prev));
                prev = next;
            }

            perimeter
        }
        ShapeGeometry::Segment(segment) => 2.0 * length(sub(segment.point1, segment.point2)),
        ShapeGeometry::ChainSegment(chain_segment) => {
            2.0 * length(sub(
                chain_segment.segment.point1,
                chain_segment.segment.point2,
            ))
        }
    }
}

/// This projects the shape perimeter onto an infinite line.
/// (b2GetShapeProjectedPerimeter)
pub fn get_shape_projected_perimeter(shape: &Shape, line: Vec2) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let axis = sub(capsule.center2, capsule.center1);
            let projected_length = abs_float(dot(axis, line));
            projected_length + 2.0 * capsule.radius
        }

        ShapeGeometry::Circle(circle) => 2.0 * circle.radius,

        ShapeGeometry::Polygon(polygon) => {
            let points = &polygon.vertices;
            let count = polygon.count as usize;
            debug_assert!(count > 0);
            let value = dot(points[0], line);
            let mut lower = value;
            let mut upper = value;
            for point in points.iter().take(count).skip(1) {
                let value = dot(*point, line);
                lower = min_float(lower, value);
                upper = max_float(upper, value);
            }

            (upper - lower) + 2.0 * polygon.radius
        }

        ShapeGeometry::Segment(segment) => {
            let value1 = dot(segment.point1, line);
            let value2 = dot(segment.point2, line);
            abs_float(value2 - value1)
        }

        ShapeGeometry::ChainSegment(chain_segment) => {
            let value1 = dot(chain_segment.segment.point1, line);
            let value2 = dot(chain_segment.segment.point2, line);
            abs_float(value2 - value1)
        }
    }
}

/// (b2MakeShapeDistanceProxy)
pub fn make_shape_distance_proxy(shape: &Shape) -> ShapeProxy {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            make_proxy(&[capsule.center1, capsule.center2], capsule.radius)
        }
        ShapeGeometry::Circle(circle) => make_proxy(&[circle.center], circle.radius),
        ShapeGeometry::Polygon(polygon) => {
            make_proxy(&polygon.vertices[..polygon.count as usize], polygon.radius)
        }
        ShapeGeometry::Segment(segment) => make_proxy(&[segment.point1, segment.point2], 0.0),
        ShapeGeometry::ChainSegment(chain_segment) => make_proxy(
            &[chain_segment.segment.point1, chain_segment.segment.point2],
            0.0,
        ),
    }
}

/// (b2RayCastShape)
pub fn ray_cast_shape(input: &RayCastInput, shape: &Shape, transform: Transform) -> CastOutput {
    let local_input = RayCastInput {
        origin: inv_transform_point(transform, input.origin),
        translation: inv_rotate_vector(transform.q, input.translation),
        max_fraction: input.max_fraction,
    };

    let mut output = match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => ray_cast_capsule(capsule, &local_input),
        ShapeGeometry::Circle(circle) => ray_cast_circle(circle, &local_input),
        ShapeGeometry::Polygon(polygon) => ray_cast_polygon(polygon, &local_input),
        ShapeGeometry::Segment(segment) => ray_cast_segment(segment, &local_input, false),
        ShapeGeometry::ChainSegment(chain_segment) => {
            ray_cast_segment(&chain_segment.segment, &local_input, true)
        }
    };

    // The output point stays in the frame of the input transform, a caller
    // chosen frame that is typically re-centered near the origin.
    output.point = transform_point(transform, output.point);
    output.normal = rotate_vector(transform.q, output.normal);
    output
}

/// (b2ShapeCastShape)
pub fn shape_cast_shape(input: &ShapeCastInput, shape: &Shape, transform: Transform) -> CastOutput {
    let mut output = CastOutput::default();

    if input.proxy.count == 0 {
        return output;
    }

    let mut local_input = *input;

    for i in 0..local_input.proxy.count as usize {
        local_input.proxy.points[i] = inv_transform_point(transform, input.proxy.points[i]);
    }

    local_input.translation = inv_rotate_vector(transform.q, input.translation);

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            output = shape_cast_capsule(capsule, &local_input);
        }
        ShapeGeometry::Circle(circle) => {
            output = shape_cast_circle(circle, &local_input);
        }
        ShapeGeometry::Polygon(polygon) => {
            output = shape_cast_polygon(polygon, &local_input);
        }
        ShapeGeometry::Segment(segment) => {
            output = shape_cast_segment(segment, &local_input);
        }
        ShapeGeometry::ChainSegment(chain_segment) => {
            // Check for back side collision
            let mut approximate_centroid = local_input.proxy.points[0];
            for i in 1..local_input.proxy.count as usize {
                approximate_centroid = add(approximate_centroid, local_input.proxy.points[i]);
            }

            approximate_centroid =
                mul_sv(1.0 / local_input.proxy.count as f32, approximate_centroid);

            let edge = sub(chain_segment.segment.point2, chain_segment.segment.point1);
            let r = sub(approximate_centroid, chain_segment.segment.point1);

            if cross(r, edge) < 0.0 {
                // Shape cast starts behind
                return output;
            }

            output = shape_cast_segment(&chain_segment.segment, &local_input);
        }
    }

    // Same frame contract as ray_cast_shape, the point stays in the input
    // transform frame
    output.point = transform_point(transform, output.point);
    output.normal = rotate_vector(transform.q, output.normal);
    output
}

/// (b2CollideMover)
pub fn collide_mover(
    mover: &crate::collision::Capsule,
    shape: &Shape,
    transform: Transform,
) -> PlaneResult {
    let local_mover = crate::collision::Capsule {
        center1: inv_transform_point(transform, mover.center1),
        center2: inv_transform_point(transform, mover.center2),
        radius: mover.radius,
    };

    let mut result = match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => collide_mover_and_capsule(&local_mover, capsule),
        ShapeGeometry::Circle(circle) => collide_mover_and_circle(&local_mover, circle),
        ShapeGeometry::Polygon(polygon) => collide_mover_and_polygon(&local_mover, polygon),
        ShapeGeometry::Segment(segment) => collide_mover_and_segment(&local_mover, segment),
        ShapeGeometry::ChainSegment(chain_segment) => {
            collide_mover_and_segment(&local_mover, &chain_segment.segment)
        }
    };

    if !result.hit {
        return result;
    }

    result.plane.normal = rotate_vector(transform.q, result.plane.normal);
    result
}
