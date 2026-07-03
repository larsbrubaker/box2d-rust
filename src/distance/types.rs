// Distance-group types from include/box2d/collision.h.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::hull::MAX_POLYGON_VERTICES;
use crate::math_functions::{Rot, Transform, Vec2};

/// A distance proxy used by the GJK algorithm. It encapsulates any shape.
/// You can provide between 1 and [`MAX_POLYGON_VERTICES`] points and a radius.
/// (b2ShapeProxy)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeProxy {
    /// The point cloud
    pub points: [Vec2; MAX_POLYGON_VERTICES],
    /// The number of points. Must be greater than 0.
    pub count: i32,
    /// The external radius of the point cloud. May be zero.
    pub radius: f32,
}

impl Default for ShapeProxy {
    fn default() -> Self {
        ShapeProxy {
            points: [Vec2::default(); MAX_POLYGON_VERTICES],
            count: 0,
            radius: 0.0,
        }
    }
}

/// Result of computing the distance between two line segments.
/// (b2SegmentDistanceResult)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SegmentDistanceResult {
    /// The closest point on the first segment
    pub closest1: Vec2,
    /// The closest point on the second segment
    pub closest2: Vec2,
    /// The barycentric coordinate on the first segment
    pub fraction1: f32,
    /// The barycentric coordinate on the second segment
    pub fraction2: f32,
    /// The squared distance between the closest points
    pub distance_squared: f32,
}

/// Used to warm start the GJK simplex. If you call this function multiple times
/// with nearby transforms this might improve performance. Otherwise you can
/// zero initialize this. (b2SimplexCache)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimplexCache {
    /// The number of stored simplex points
    pub count: u16,
    /// The cached simplex indices on shape A
    pub index_a: [u8; 3],
    /// The cached simplex indices on shape B
    pub index_b: [u8; 3],
}

/// Input for [`shape_distance`](crate::distance::shape_distance).
/// (b2DistanceInput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceInput {
    /// The proxy for shape A
    pub proxy_a: ShapeProxy,
    /// The proxy for shape B
    pub proxy_b: ShapeProxy,
    /// Transform of shape B in shape A's frame, the relative pose B in A
    /// (`inv_mul_transforms(world_a, world_b)`). The query is origin
    /// independent and runs in frame A.
    pub transform: Transform,
    /// Should the proxy radius be considered?
    pub use_radii: bool,
}

/// Output for [`shape_distance`](crate::distance::shape_distance).
/// (b2DistanceOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DistanceOutput {
    /// Closest point on shape A, in shape A's frame
    pub point_a: Vec2,
    /// Closest point on shape B, in shape A's frame
    pub point_b: Vec2,
    /// A to B normal in shape A's frame. Invalid if distance is zero.
    pub normal: Vec2,
    /// The final distance, zero if overlapped
    pub distance: f32,
    /// Number of GJK iterations used
    pub iterations: i32,
    /// The number of simplexes stored in the simplex array
    pub simplex_count: i32,
}

/// Simplex vertex for debugging the GJK algorithm. (b2SimplexVertex)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SimplexVertex {
    /// support point in proxy A
    pub w_a: Vec2,
    /// support point in proxy B
    pub w_b: Vec2,
    /// w_b - w_a
    pub w: Vec2,
    /// barycentric coordinate for closest point
    pub a: f32,
    /// w_a index
    pub index_a: i32,
    /// w_b index
    pub index_b: i32,
}

/// Simplex from the GJK algorithm. (b2Simplex)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Simplex {
    /// vertices
    pub v1: SimplexVertex,
    pub v2: SimplexVertex,
    pub v3: SimplexVertex,
    /// number of valid vertices
    pub count: i32,
}

impl Simplex {
    /// The C code walks `b2SimplexVertex* vertices[] = {&v1, &v2, &v3}`; these
    /// accessors are the borrow-checked equivalent.
    pub(crate) fn vertex(&self, index: i32) -> &SimplexVertex {
        match index {
            0 => &self.v1,
            1 => &self.v2,
            _ => &self.v3,
        }
    }

    pub(crate) fn vertex_mut(&mut self, index: i32) -> &mut SimplexVertex {
        match index {
            0 => &mut self.v1,
            1 => &mut self.v2,
            _ => &mut self.v3,
        }
    }
}

/// Input parameters for [`shape_cast`](crate::distance::shape_cast).
/// (b2ShapeCastPairInput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeCastPairInput {
    /// The proxy for shape A
    pub proxy_a: ShapeProxy,
    /// The proxy for shape B
    pub proxy_b: ShapeProxy,
    /// Transform of shape B in shape A's frame, the relative pose B in A
    pub transform: Transform,
    /// The translation of shape B, in A's frame
    pub translation_b: Vec2,
    /// The fraction of the translation to consider, typically 1
    pub max_fraction: f32,
    /// Allows shapes with a radius to move slightly closer if already touching
    pub can_encroach: bool,
}

/// This describes the motion of a body/shape for TOI computation. Shapes are
/// defined with respect to the body origin, which may not coincide with the
/// center of mass. However, to support dynamics we must interpolate the center
/// of mass position. (b2Sweep)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sweep {
    /// Local center of mass position
    pub local_center: Vec2,
    /// Starting center of mass world position
    pub c1: Vec2,
    /// Ending center of mass world position
    pub c2: Vec2,
    /// Starting world rotation
    pub q1: Rot,
    /// Ending world rotation
    pub q2: Rot,
}

/// Time of impact input. (b2TOIInput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToiInput {
    /// The proxy for shape A
    pub proxy_a: ShapeProxy,
    /// The proxy for shape B
    pub proxy_b: ShapeProxy,
    /// The movement of shape A
    pub sweep_a: Sweep,
    /// The movement of shape B
    pub sweep_b: Sweep,
    /// Defines the sweep interval [0, max_fraction]
    pub max_fraction: f32,
}

/// Describes the TOI output. (b2TOIState)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ToiState {
    #[default]
    Unknown,
    Failed,
    Overlapped,
    Hit,
    Separated,
}

/// Time of impact output. (b2TOIOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ToiOutput {
    /// The type of result
    pub state: ToiState,
    /// The hit point
    pub point: Vec2,
    /// The hit normal
    pub normal: Vec2,
    /// The sweep time of the collision
    pub fraction: f32,
}
