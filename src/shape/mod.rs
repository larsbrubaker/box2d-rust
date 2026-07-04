// Port of the shape module from box2d-cpp-reference/src/shape.h + shape.c.
//
// Split to satisfy the 800-line file limit:
// - dispatch.rs  — per-shape-type dispatch helpers (AABBs, mass, casts, mover)
// - lifecycle.rs — shape creation (margin, create_shape_internal, typed wrappers)
//
// This file holds the data model and filter predicates. The remaining
// shape.c (destruction, chains, world API accessors) lands in later slices.
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

mod api;
mod chain;
mod dispatch;
mod geometry_api;
mod lifecycle;

pub use api::*;
pub use chain::*;
pub use dispatch::*;
pub use geometry_api::*;
pub use lifecycle::*;

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
    fn create_shapes_on_body_updates_mass() {
        use crate::body::{create_body, get_body_full_id};
        use crate::collision::Circle;
        use crate::geometry::{compute_polygon_mass, make_box};
        use crate::math_functions::Vec2;
        use crate::solver_set::AWAKE_SET;
        use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
        use crate::world::World;

        let mut world = World::new(&default_world_def());

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let body = create_body(&mut world, &body_def);
        let body_index = get_body_full_id(&world, body);

        // A 1x1 half-extent box with density 1: mass = 4, matching geometry.
        let box_poly = make_box(1.0, 1.0);
        let shape_def = default_shape_def();
        let shape_id = create_polygon_shape(&mut world, body, &shape_def, &box_poly);
        assert!(shape_id.index1 >= 1);

        let expected = compute_polygon_mass(&box_poly, 1.0);
        {
            let b = &world.bodies[body_index as usize];
            assert_eq!(b.shape_count, 1);
            assert_eq!(b.mass, expected.mass);
            assert_eq!(b.inertia, expected.rotational_inertia);
        }
        {
            let sim = &world.solver_sets[AWAKE_SET as usize].body_sims[0];
            assert_eq!(sim.inv_mass, 1.0 / expected.mass);
            assert!(sim.max_extent > 1.4 && sim.max_extent < 1.5);
        }

        // The shape has a live broad-phase proxy in the dynamic tree.
        let raw_shape = (shape_id.index1 - 1) as usize;
        let proxy_key = world.shapes[raw_shape].proxy_key;
        assert!(proxy_key != NULL_INDEX);
        assert_eq!(world.broad_phase.shape_index(proxy_key), raw_shape as i32);

        // A second shape accumulates mass; deferring the update sets dirty.
        let circle = Circle {
            center: Vec2 { x: 2.0, y: 0.0 },
            radius: 0.5,
        };
        let mut lazy_def = default_shape_def();
        lazy_def.update_body_mass = false;
        let _s2 = create_circle_shape(&mut world, body, &lazy_def, &circle);
        {
            let b = &world.bodies[body_index as usize];
            assert_eq!(b.shape_count, 2);
            // Mass unchanged, flagged dirty.
            assert_eq!(b.mass, expected.mass);
            assert!(b.flags & crate::body::body_flags::DIRTY_MASS != 0);
        }
        crate::body::update_body_mass_data(&mut world, body_index);
        assert!(world.bodies[body_index as usize].mass > expected.mass);

        // Sensor shapes register in the sensor array.
        let mut sensor_def = default_shape_def();
        sensor_def.is_sensor = true;
        let s3 = create_circle_shape(&mut world, body, &sensor_def, &circle);
        let raw_s3 = (s3.index1 - 1) as usize;
        assert_eq!(world.shapes[raw_s3].sensor_index, 0);
        assert_eq!(world.sensors.len(), 1);
        assert_eq!(world.sensors[0].shape_id, raw_s3 as i32);

        world.validate_solver_sets();
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
