// Shape and chain creation types and defaults from types.h / types.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS};
use crate::core::SECRET_COOKIE;
use crate::math_functions::Vec2;

/// Collision filtering data for shapes. (b2Filter)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Filter {
    /// The collision category bits. Normally you would just set one bit.
    pub category_bits: u64,
    /// The collision mask bits: the categories this shape accepts for collision.
    pub mask_bits: u64,
    /// Collision groups: never collide (negative) or always collide (positive).
    /// A group index of zero has no effect. Non-zero group filtering always
    /// wins against the mask bits.
    pub group_index: i32,
}

/// Initialize a filter with the default values. (b2DefaultFilter)
pub fn default_filter() -> Filter {
    Filter {
        category_bits: DEFAULT_CATEGORY_BITS,
        mask_bits: DEFAULT_MASK_BITS,
        group_index: 0,
    }
}

impl Default for Filter {
    fn default() -> Self {
        default_filter()
    }
}

/// The query filter is used to filter collisions between queries and shapes.
/// (b2QueryFilter)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryFilter {
    /// The collision category bits of this query.
    pub category_bits: u64,
    /// The shape categories this query accepts for collision.
    pub mask_bits: u64,
}

/// Initialize a query filter with the default values. (b2DefaultQueryFilter)
pub fn default_query_filter() -> QueryFilter {
    QueryFilter {
        category_bits: DEFAULT_CATEGORY_BITS,
        mask_bits: DEFAULT_MASK_BITS,
    }
}

impl Default for QueryFilter {
    fn default() -> Self {
        default_query_filter()
    }
}

/// Surface materials allow chain shapes to have per segment surface properties.
/// (b2SurfaceMaterial)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceMaterial {
    /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
    pub friction: f32,
    /// The coefficient of restitution (bounce), usually in the range [0,1].
    pub restitution: f32,
    /// The rolling resistance, usually in the range [0,1].
    pub rolling_resistance: f32,
    /// The tangent speed for conveyor belts.
    pub tangent_speed: f32,
    /// User material identifier, passed to friction/restitution callbacks and
    /// query results. Not used internally.
    pub user_material_id: u64,
    /// Custom debug draw color.
    pub custom_color: u32,
}

/// Initialize a surface material with the default values.
/// (b2DefaultSurfaceMaterial)
pub fn default_surface_material() -> SurfaceMaterial {
    SurfaceMaterial {
        friction: 0.6,
        restitution: 0.0,
        rolling_resistance: 0.0,
        tangent_speed: 0.0,
        user_material_id: 0,
        custom_color: 0,
    }
}

impl Default for SurfaceMaterial {
    fn default() -> Self {
        default_surface_material()
    }
}

/// Used to create a shape. Must be initialized using [`default_shape_def`].
/// (b2ShapeDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeDef {
    /// Application specific shape data.
    pub user_data: u64,
    /// The surface material for this shape.
    pub material: SurfaceMaterial,
    /// The density, usually in kg/m^2.
    pub density: f32,
    /// Collision filtering data.
    pub filter: Filter,
    /// Enable custom filtering. Only one of the two shapes needs to enable it.
    pub enable_custom_filtering: bool,
    /// A sensor shape generates overlap events but no collision response.
    pub is_sensor: bool,
    /// Enable sensor events for this shape. False by default, even for sensors.
    pub enable_sensor_events: bool,
    /// Enable contact events for this shape. False by default.
    pub enable_contact_events: bool,
    /// Enable hit events for this shape. False by default.
    pub enable_hit_events: bool,
    /// Enable pre-solve contact events for this shape.
    pub enable_pre_solve_events: bool,
    /// When shapes are created they scan the environment for collision next
    /// step. Ignored for dynamic and kinematic shapes.
    pub invoke_contact_creation: bool,
    /// Should the body update the mass properties when this shape is created.
    pub update_body_mass: bool,
    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// Initialize a shape definition with the default values. (b2DefaultShapeDef)
pub fn default_shape_def() -> ShapeDef {
    ShapeDef {
        user_data: 0,
        // C zero-inits the material then sets friction = 0.6, which is exactly
        // the default surface material.
        material: default_surface_material(),
        density: 1.0,
        filter: default_filter(),
        enable_custom_filtering: false,
        is_sensor: false,
        enable_sensor_events: false,
        enable_contact_events: false,
        enable_hit_events: false,
        enable_pre_solve_events: false,
        invoke_contact_creation: true,
        update_body_mass: true,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for ShapeDef {
    fn default() -> Self {
        default_shape_def()
    }
}

/// Used to create a chain of line segments. Must be initialized using
/// [`default_chain_def`]. (b2ChainDef)
///
/// The C struct uses `points`/`count` and `materials`/`materialCount` pointer +
/// length pairs; the Rust port carries owned `Vec`s whose lengths encode the
/// counts (the C notes these are cloned).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ChainDef {
    /// Application specific shape data.
    pub user_data: u64,
    /// An array of at least 4 points.
    pub points: Vec<Vec2>,
    /// Surface materials. Must have length 1 (one material for all segments) or
    /// `points.len()` (a unique material per segment).
    pub materials: Vec<SurfaceMaterial>,
    /// Contact filtering data.
    pub filter: Filter,
    /// Indicates a closed chain formed by connecting the first and last points.
    pub is_loop: bool,
    /// Enable sensors to detect this chain. False by default.
    pub enable_sensor_events: bool,
    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// Initialize a chain definition with the default values. (b2DefaultChainDef)
pub fn default_chain_def() -> ChainDef {
    ChainDef {
        user_data: 0,
        points: Vec::new(),
        materials: vec![default_surface_material()],
        filter: default_filter(),
        is_loop: false,
        enable_sensor_events: false,
        internal_value: SECRET_COOKIE,
    }
}
