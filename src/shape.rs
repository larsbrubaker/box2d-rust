// Port of the shape data model from box2d-cpp-reference/src/shape.h.
// Logic from shape.c lands in a later bring-up commit. The pure-geometry
// helpers declared in shape.h (mass/AABB/point/ray/mover functions) are
// already ported in the geometry module.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{Circle, ShapeGeometry, ShapeType};
use crate::core::NULL_INDEX;
use crate::math_functions::{Aabb, Vec2, VEC2_ZERO};
use crate::types::{Filter, QueryFilter, SurfaceMaterial};

/// Internal shape. (b2Shape)
///
/// The C struct stores a `b2ShapeType type` tag plus a union of the concrete
/// geometries; the Rust port stores the tagged [`ShapeGeometry`] enum, and
/// [`Shape::shape_type`] recovers the tag.
#[derive(Debug, Clone)]
pub struct Shape {
    pub id: i32,
    pub body_id: i32,
    pub prev_shape_id: i32,
    pub next_shape_id: i32,
    pub sensor_index: i32,
    pub material: SurfaceMaterial,
    pub density: f32,
    pub aabb_margin: f32,
    pub aabb: Aabb,
    pub fat_aabb: Aabb,
    pub local_centroid: Vec2,
    pub proxy_key: i32,

    pub filter: Filter,
    pub user_data: u64,

    /// The shape geometry (C: type tag + union).
    pub geometry: ShapeGeometry,

    pub generation: u16,
    pub enable_sensor_events: bool,
    pub enable_contact_events: bool,
    pub enable_custom_filtering: bool,
    pub enable_hit_events: bool,
    pub enable_pre_solve_events: bool,
    pub enlarged_aabb: bool,
}

impl Shape {
    /// The shape type tag. (C: shape->type)
    pub fn shape_type(&self) -> ShapeType {
        self.geometry.shape_type()
    }

    /// (static inline b2GetShapeRadius)
    pub fn radius(&self) -> f32 {
        match &self.geometry {
            ShapeGeometry::Capsule(capsule) => capsule.radius,
            ShapeGeometry::Circle(circle) => circle.radius,
            ShapeGeometry::Polygon(polygon) => polygon.radius,
            _ => 0.0,
        }
    }
}

impl Default for Shape {
    fn default() -> Self {
        Shape {
            id: NULL_INDEX,
            body_id: NULL_INDEX,
            prev_shape_id: NULL_INDEX,
            next_shape_id: NULL_INDEX,
            sensor_index: NULL_INDEX,
            material: SurfaceMaterial::default(),
            density: 0.0,
            aabb_margin: 0.0,
            aabb: Aabb::default(),
            fat_aabb: Aabb::default(),
            local_centroid: VEC2_ZERO,
            proxy_key: NULL_INDEX,
            filter: Filter::default(),
            user_data: 0,
            geometry: ShapeGeometry::Circle(Circle::default()),
            generation: 0,
            enable_sensor_events: false,
            enable_contact_events: false,
            enable_custom_filtering: false,
            enable_hit_events: false,
            enable_pre_solve_events: false,
            enlarged_aabb: false,
        }
    }
}

/// Internal chain shape. (b2ChainShape)
///
/// The C `int* shapeIndices` / `b2SurfaceMaterial* materials` pointer+count
/// pairs are owned Vecs.
#[derive(Debug, Clone, Default)]
pub struct ChainShape {
    pub id: i32,
    pub body_id: i32,
    pub next_chain_id: i32,
    pub shape_indices: Vec<i32>,
    pub materials: Vec<SurfaceMaterial>,
    pub generation: u16,
}

/// (b2ShapeExtent)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ShapeExtent {
    pub min_extent: f32,
    pub max_extent: f32,
}

/// (static inline b2ShouldShapesCollide)
pub fn should_shapes_collide(filter_a: Filter, filter_b: Filter) -> bool {
    if filter_a.group_index == filter_b.group_index && filter_a.group_index != 0 {
        return filter_a.group_index > 0;
    }

    (filter_a.mask_bits & filter_b.category_bits) != 0
        && (filter_a.category_bits & filter_b.mask_bits) != 0
}

/// (static inline b2ShouldQueryCollide)
pub fn should_query_collide(shape_filter: Filter, query_filter: QueryFilter) -> bool {
    (shape_filter.category_bits & query_filter.mask_bits) != 0
        && (shape_filter.mask_bits & query_filter.category_bits) != 0
}

// ---------------------------------------------------------------------------
// Shape dispatch helpers from shape.c: thin dispatch over the geometry and
// distance modules by shape type.
// ---------------------------------------------------------------------------

use crate::broad_phase::{proxy_type, BroadPhase};
use crate::collision::{CastOutput, MassData, PlaneResult, RayCastInput, ShapeCastInput};
use crate::constants::speculative_distance;
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
    lerp, max_float, min_float, mul_sv, rotate_vector, sub, transform_point, Transform,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::default_filter;

    #[test]
    fn filter_logic() {
        // Same positive group always collides, same negative group never.
        let mut a = default_filter();
        let mut b = default_filter();
        a.group_index = 3;
        b.group_index = 3;
        assert!(should_shapes_collide(a, b));
        a.group_index = -3;
        b.group_index = -3;
        assert!(!should_shapes_collide(a, b));

        // Zero group falls back to category/mask.
        a.group_index = 0;
        b.group_index = 0;
        a.category_bits = 0x2;
        a.mask_bits = 0x4;
        b.category_bits = 0x4;
        b.mask_bits = 0x2;
        assert!(should_shapes_collide(a, b));
        b.mask_bits = 0x8;
        assert!(!should_shapes_collide(a, b));

        // Query filtering is symmetric category/mask with no groups.
        let shape_filter = default_filter();
        let query = crate::types::default_query_filter();
        assert!(should_query_collide(shape_filter, query));
    }

    #[test]
    fn shape_dispatch_matches_geometry() {
        use crate::collision::Circle;
        use crate::geometry::{compute_circle_mass, make_box};
        use crate::math_functions::{make_world_transform, Vec2, PI, TRANSFORM_IDENTITY};

        let circle = Circle {
            center: Vec2 { x: 1.0, y: 0.0 },
            radius: 1.0,
        };
        let shape = Shape {
            density: 2.0,
            geometry: ShapeGeometry::Circle(circle),
            ..Default::default()
        };

        // Mass dispatch matches the direct geometry call.
        let md = compute_shape_mass(&shape);
        assert_eq!(md, compute_circle_mass(&circle, 2.0));

        // Centroid, perimeter, extents.
        assert_eq!(get_shape_centroid(&shape), circle.center);
        assert_eq!(get_shape_perimeter(&shape), 2.0 * PI);
        let extent = compute_shape_extent(&shape, Vec2 { x: 0.0, y: 0.0 });
        assert_eq!(extent.min_extent, 1.0);
        assert_eq!(extent.max_extent, 2.0);

        // AABB through the world transform.
        let aabb = compute_shape_aabb(&shape, make_world_transform(TRANSFORM_IDENTITY));
        assert!((aabb.lower_bound.x - 0.0).abs() < 1e-6);
        assert!((aabb.upper_bound.x - 2.0).abs() < 1e-6);

        // Distance proxy carries the radius for round shapes.
        let proxy = make_shape_distance_proxy(&shape);
        assert_eq!(proxy.count, 1);
        assert_eq!(proxy.radius, 1.0);

        // Polygon dispatch: projected perimeter of a unit box on the x axis.
        let box_shape = Shape {
            geometry: ShapeGeometry::Polygon(make_box(1.0, 1.0)),
            ..Default::default()
        };
        let proj = get_shape_projected_perimeter(&box_shape, Vec2 { x: 1.0, y: 0.0 });
        assert_eq!(proj, 2.0);
    }

    #[test]
    fn ray_cast_shape_respects_transform() {
        use crate::collision::{Circle, RayCastInput};
        use crate::math_functions::{Transform, Vec2, ROT_IDENTITY};

        // A unit circle at local origin, shifted to x = 3 by the transform.
        let shape = Shape {
            geometry: ShapeGeometry::Circle(Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 1.0,
            }),
            ..Default::default()
        };
        let transform = Transform {
            p: Vec2 { x: 3.0, y: 0.0 },
            q: ROT_IDENTITY,
        };

        let input = RayCastInput {
            origin: Vec2 { x: 0.0, y: 0.0 },
            translation: Vec2 { x: 8.0, y: 0.0 },
            max_fraction: 1.0,
        };
        let output = ray_cast_shape(&input, &shape, transform);
        assert!(output.hit);
        // Hit at x = 2 in the input frame.
        assert!((output.point.x - 2.0).abs() < 1e-5);
        assert!((output.fraction - 0.25).abs() < 1e-6);
        assert!((output.normal.x + 1.0).abs() < 1e-6);
    }

    #[test]
    fn proxy_lifecycle_updates_broad_phase() {
        use crate::broad_phase::BroadPhase;
        use crate::math_functions::{make_world_transform, TRANSFORM_IDENTITY};
        use crate::types::{BodyType, Capacity};

        let mut bp = BroadPhase::new(&Capacity::default());
        let mut shape = Shape {
            id: 5,
            geometry: ShapeGeometry::Circle(crate::collision::Circle {
                center: crate::math_functions::Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            }),
            aabb_margin: crate::constants::max_aabb_margin(),
            ..Default::default()
        };

        create_shape_proxy(
            &mut shape,
            &mut bp,
            BodyType::Dynamic,
            make_world_transform(TRANSFORM_IDENTITY),
            false,
        );
        assert!(shape.proxy_key != NULL_INDEX);
        // Fat AABB got both speculative and movement margins.
        assert!(shape.fat_aabb.upper_bound.x > shape.aabb.upper_bound.x);
        assert!(shape.aabb.upper_bound.x > 0.5);
        assert_eq!(bp.shape_index(shape.proxy_key), 5);

        destroy_shape_proxy(&mut shape, &mut bp);
        assert_eq!(shape.proxy_key, NULL_INDEX);
    }
}
