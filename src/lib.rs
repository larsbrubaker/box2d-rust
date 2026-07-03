//! Pure Rust port of [Box2D v3](https://github.com/erincatto/box2d), Erin Catto's 2D physics
//! engine. The port matches the C implementation's behavior exactly, including its
//! cross-platform deterministic math.
//!
//! Ported module by module from the pinned C reference in `box2d-cpp-reference/`.

pub mod bitset;
pub mod constants;
pub mod core;
pub mod id;
pub mod math_functions;

#[cfg(test)]
mod bitset_tests;
#[cfg(test)]
mod id_tests;
#[cfg(test)]
mod math_functions_tests;

pub use core::{get_version, is_double_precision, Version};
pub use id::{BodyId, ChainId, ContactId, JointId, ShapeId, WorldId};
pub use math_functions::{Aabb, CosSin, Mat22, Plane, Pos, Rot, Transform, Vec2, WorldTransform};

/// Library version, kept in sync with Cargo.toml.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
