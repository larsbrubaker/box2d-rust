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
}
