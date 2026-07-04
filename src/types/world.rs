// World creation types and default from types.h / types.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
use crate::id::ShapeId;
use crate::math_functions::{Pos, Vec2, POS_ZERO, VEC2_ZERO};

/// Optional friction mixing callback. The default uses `sqrt(frictionA * frictionB)`.
/// (b2FrictionCallback)
///
/// Args: `(friction_a, user_material_id_a, friction_b, user_material_id_b)`.
pub type FrictionCallback = fn(f32, u64, f32, u64) -> f32;

/// Optional restitution mixing callback. The default uses
/// `max(restitutionA, restitutionB)`. (b2RestitutionCallback)
pub type RestitutionCallback = fn(f32, u64, f32, u64) -> f32;

/// Result from `b2World_RayCastClosest`. (b2RayResult)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayResult {
    pub shape_id: ShapeId,
    pub point: Pos,
    pub normal: Vec2,
    pub fraction: f32,
    pub node_visits: i32,
    pub leaf_visits: i32,
    pub hit: bool,
}

impl Default for RayResult {
    fn default() -> Self {
        RayResult {
            shape_id: ShapeId::default(),
            point: POS_ZERO,
            normal: VEC2_ZERO,
            fraction: 0.0,
            node_visits: 0,
            leaf_visits: 0,
            hit: false,
        }
    }
}

/// Optional world capacities that can be used to avoid run-time allocations.
/// (b2Capacity)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Capacity {
    /// Number of expected static shapes.
    pub static_shape_count: i32,
    /// Number of expected dynamic and kinematic shapes.
    pub dynamic_shape_count: i32,
    /// Number of expected static bodies.
    pub static_body_count: i32,
    /// Number of expected dynamic and kinematic bodies.
    pub dynamic_body_count: i32,
    /// Number of expected contacts.
    pub contact_count: i32,
}

/// Counters that give details of the simulation size. (b2Counters)
///
/// `byte_count`, `stack_used`, and `task_count` are always zero in this port:
/// there is no global allocation tracker, no arena stack allocator, and no
/// task system in the serial Rust implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Counters {
    pub byte_count: i64,
    pub body_count: i32,
    pub shape_count: i32,
    pub contact_count: i32,
    pub joint_count: i32,
    pub island_count: i32,
    pub stack_used: i32,
    pub static_tree_height: i32,
    pub tree_height: i32,
    pub task_count: i32,
    pub color_counts: [i32; crate::constants::GRAPH_COLOR_COUNT as usize],

    /// Number of contacts touched by the collide pass (graph contacts +
    /// awake-set non-touching).
    pub awake_contact_count: i32,

    /// Number of contacts recycled in the most recent step.
    pub recycled_contact_count: i32,
}

/// The explosion definition is used to configure options for explosions.
/// Explosions consider shape geometry when computing the impulse.
/// (b2ExplosionDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExplosionDef {
    /// Mask bits to filter shapes
    pub mask_bits: u64,

    /// The center of the explosion in world space
    pub position: Pos,

    /// The radius of the explosion
    pub radius: f32,

    /// The falloff distance beyond the radius. Impulse is reduced to zero at
    /// this distance.
    pub falloff: f32,

    /// Impulse per unit length. This applies an impulse according to the
    /// shape perimeter that is facing the explosion. Explosions only apply to
    /// circles, capsules, and polygons. This may be negative for implosions.
    pub impulse_per_length: f32,
}

/// Use this to initialize your explosion definition. (b2DefaultExplosionDef)
pub fn default_explosion_def() -> ExplosionDef {
    ExplosionDef {
        mask_bits: crate::dynamic_tree::DEFAULT_MASK_BITS,
        position: POS_ZERO,
        radius: 0.0,
        falloff: 0.0,
        impulse_per_length: 0.0,
    }
}

impl Default for ExplosionDef {
    fn default() -> Self {
        default_explosion_def()
    }
}

/// World definition used to create a simulation world. Must be initialized
/// using [`default_world_def`]. (b2WorldDef)
///
/// `PartialEq` is intentionally not derived: it holds optional function-pointer
/// callbacks, and function-pointer equality is not meaningful.
#[derive(Debug, Clone, Copy)]
pub struct WorldDef {
    /// Gravity vector. Box2D has no up-vector defined.
    pub gravity: Vec2,
    /// Restitution speed threshold, usually in m/s. Collisions above this speed
    /// have restitution applied (will bounce).
    pub restitution_threshold: f32,
    /// Threshold speed for hit events. Usually meters per second.
    pub hit_event_threshold: f32,
    /// Contact stiffness. Cycles per second.
    pub contact_hertz: f32,
    /// Contact bounciness. Non-dimensional.
    pub contact_damping_ratio: f32,
    /// Speed cap on overlap resolution, usually meters per second.
    pub contact_speed: f32,
    /// Maximum linear speed. Usually meters per second.
    pub maximum_linear_speed: f32,
    /// Optional mixing callback for friction.
    pub friction_callback: Option<FrictionCallback>,
    /// Optional mixing callback for restitution.
    pub restitution_callback: Option<RestitutionCallback>,
    /// Can bodies go to sleep to improve performance.
    pub enable_sleep: bool,
    /// Enable continuous collision.
    pub enable_continuous: bool,
    /// Contact softening when mass ratios are large. Experimental.
    pub enable_contact_softening: bool,
    /// Number of workers for multithreading, clamped to `[1, MAX_WORKERS]`.
    pub worker_count: i32,
    /// User data.
    pub user_data: u64,
    /// Optional initial capacities.
    pub capacity: Capacity,
    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// Initialize a world definition with the default values. (b2DefaultWorldDef)
pub fn default_world_def() -> WorldDef {
    let length_units = get_length_units_per_meter();
    WorldDef {
        gravity: Vec2 { x: 0.0, y: -10.0 },
        hit_event_threshold: 1.0 * length_units,
        restitution_threshold: 1.0 * length_units,
        contact_speed: 3.0 * length_units,
        contact_hertz: 30.0,
        contact_damping_ratio: 10.0,
        // 400 meters per second, faster than the speed of sound
        maximum_linear_speed: 400.0 * length_units,
        friction_callback: None,
        restitution_callback: None,
        enable_sleep: true,
        enable_continuous: true,
        enable_contact_softening: false,
        worker_count: 0,
        user_data: 0,
        capacity: Capacity::default(),
        internal_value: SECRET_COOKIE,
    }
}

impl Default for WorldDef {
    fn default() -> Self {
        default_world_def()
    }
}
