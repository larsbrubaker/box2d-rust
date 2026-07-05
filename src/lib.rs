//! Pure Rust port of [Box2D v3](https://github.com/erincatto/box2d), Erin Catto's 2D physics
//! engine. The port matches the C implementation's behavior exactly, including its
//! cross-platform deterministic math.
//!
//! Ported module by module from the pinned C reference in `box2d-cpp-reference/`.

pub mod aabb;
pub mod bitset;
pub mod body;
pub mod broad_phase;
pub mod collision;
pub mod constants;
pub mod constraint_graph;
pub mod contact;
pub mod contact_solver;
pub mod core;
pub mod distance;
pub mod distance_joint;
pub mod dynamic_tree;
pub mod events;
pub mod geometry;
pub mod hull;
pub mod id;
pub mod id_pool;
pub mod island;
pub mod joint;
pub mod manifold;
pub mod math_functions;
pub mod motor_joint;
pub mod mover;
pub mod prismatic_joint;
pub mod recording;
pub mod revolute_joint;
pub mod sensor;
pub mod shape;
pub mod solver;
pub mod solver_set;
pub mod table;
pub mod types;
pub mod weld_joint;
pub mod wheel_joint;
pub mod world;

#[cfg(test)]
mod aabb_tests;
#[cfg(test)]
mod bitset_tests;
#[cfg(test)]
mod body_api_tests;
#[cfg(test)]
mod determinism_tests;
#[cfg(test)]
mod distance_tests;
#[cfg(test)]
mod dynamic_tree_tests;
#[cfg(test)]
mod geometry_tests;
#[cfg(test)]
mod hull_tests;
#[cfg(test)]
mod id_tests;
#[cfg(test)]
mod large_world_tests;
#[cfg(test)]
mod manifold_tests;
#[cfg(test)]
mod math_functions_tests;
#[cfg(test)]
mod shape_api_tests;
#[cfg(test)]
mod snapshot_tests;
#[cfg(test)]
mod table_tests;
#[cfg(test)]
mod types_tests;
#[cfg(test)]
mod world_api_tests;

pub use collision::CastOutput;
pub use core::{get_version, is_double_precision, Version};
pub use id::{BodyId, ChainId, ContactId, JointId, ShapeId, WorldId};
pub use math_functions::{Aabb, CosSin, Mat22, Plane, Pos, Rot, Transform, Vec2, WorldTransform};

/// Library version, kept in sync with Cargo.toml.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
