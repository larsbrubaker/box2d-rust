// Port of box2d-cpp-reference/include/box2d/collision.h
//
// This module holds the collision data types, ported incrementally: each type
// lands together with the first module that consumes it.
//
// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

use crate::distance::ShapeProxy;
use crate::hull::MAX_POLYGON_VERTICES;
use crate::math_functions::{Plane, Vec2, VEC2_ZERO};

/// Low level ray cast input data. (b2RayCastInput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RayCastInput {
    /// Start point of the ray cast
    pub origin: Vec2,
    /// Translation of the ray cast
    pub translation: Vec2,
    /// The maximum fraction of the translation to consider, typically 1
    pub max_fraction: f32,
}

/// Low level shape cast input in generic form. This allows casting an
/// arbitrary point cloud wrapped with a radius. For example, a circle is a
/// single point with a non-zero radius. A capsule is two points with a
/// non-zero radius. A box is four points with a zero radius. (b2ShapeCastInput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ShapeCastInput {
    /// A generic shape
    pub proxy: ShapeProxy,
    /// The translation of the shape cast
    pub translation: Vec2,
    /// The maximum fraction of the translation to consider, typically 1
    pub max_fraction: f32,
    /// Allow shape cast to encroach when initially touching. This only works
    /// if the radius is greater than zero.
    pub can_encroach: bool,
}

/// Low level ray cast or shape cast output data. The hit point is in the local
/// or relative frame of the input. Returns a zero fraction and normal in the
/// case of initial overlap. (b2CastOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CastOutput {
    /// The surface normal at the hit point
    pub normal: Vec2,
    /// The surface hit point
    pub point: Vec2,
    /// The fraction of the input translation at collision
    pub fraction: f32,
    /// The number of iterations used
    pub iterations: i32,
    /// Did the cast hit?
    pub hit: bool,
}

/// Ray cast or shape cast output with the hit point lifted back to a world
/// position. In the C single-precision build this is an alias of b2CastOutput;
/// the double-precision build widens `point` to b2Pos, which is what the
/// unconditional `Pos` here does in both modes. (b2WorldCastOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WorldCastOutput {
    /// The surface normal at the hit point
    pub normal: Vec2,
    /// The surface hit point in world space
    pub point: crate::math_functions::Pos,
    /// The fraction of the input translation at collision
    pub fraction: f32,
    /// The number of iterations used
    pub iterations: i32,
    /// Did the cast hit?
    pub hit: bool,
}

/// This holds the mass data computed for a shape. (b2MassData)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MassData {
    /// The mass of the shape, usually in kilograms.
    pub mass: f32,
    /// The position of the shape's centroid relative to the shape's origin.
    pub center: Vec2,
    /// The rotational inertia of the shape about the shape center.
    pub rotational_inertia: f32,
}

/// A solid circle. (b2Circle)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Circle {
    /// The local center
    pub center: Vec2,
    /// The radius
    pub radius: f32,
}

/// A solid capsule can be viewed as two semicircles connected by a rectangle.
/// (b2Capsule)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Capsule {
    /// Local center of the first semicircle
    pub center1: Vec2,
    /// Local center of the second semicircle
    pub center2: Vec2,
    /// The radius of the semicircles
    pub radius: f32,
}

/// A solid convex polygon. It is assumed that the interior of the polygon is
/// to the left of each edge. Polygons have a maximum number of vertices equal
/// to [`MAX_POLYGON_VERTICES`]. In most cases you should not need many
/// vertices for a convex polygon. (b2Polygon)
///
/// @warning DO NOT fill this out manually, instead use a helper function like
/// `make_polygon` or `make_box`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Polygon {
    /// The polygon vertices
    pub vertices: [Vec2; MAX_POLYGON_VERTICES],
    /// The outward normal vectors of the polygon sides
    pub normals: [Vec2; MAX_POLYGON_VERTICES],
    /// The centroid of the polygon
    pub centroid: Vec2,
    /// The external radius for rounded polygons
    pub radius: f32,
    /// The number of polygon vertices
    pub count: i32,
}

impl Default for Polygon {
    fn default() -> Self {
        Polygon {
            vertices: [VEC2_ZERO; MAX_POLYGON_VERTICES],
            normals: [VEC2_ZERO; MAX_POLYGON_VERTICES],
            centroid: VEC2_ZERO,
            radius: 0.0,
            count: 0,
        }
    }
}

/// A line segment with two-sided collision. (b2Segment)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Segment {
    /// The first point
    pub point1: Vec2,
    /// The second point
    pub point2: Vec2,
}

/// A line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2. (b2ChainSegment)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChainSegment {
    /// The tail ghost vertex
    pub ghost1: Vec2,
    /// The line segment
    pub segment: Segment,
    /// The head ghost vertex
    pub ghost2: Vec2,
    /// The owning chain shape index (internal usage only)
    pub chain_id: i32,
}

/// Shape type. (types.h: b2ShapeType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeType {
    /// A circle with an offset
    Circle,
    /// A capsule is an extruded circle
    Capsule,
    /// A line segment
    Segment,
    /// A convex polygon
    Polygon,
    /// A line segment owned by a chain shape
    ChainSegment,
}

/// The number of shape types. (types.h: b2_shapeTypeCount)
pub const SHAPE_TYPE_COUNT: usize = 5;

/// The geometry payload of a shape. The C `b2Shape` stores a `b2ShapeType` tag
/// plus a union of the concrete geometries; in Rust that pairing is an enum.
/// The internal shape module embeds this.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShapeGeometry {
    Circle(Circle),
    Capsule(Capsule),
    Segment(Segment),
    Polygon(Polygon),
    ChainSegment(ChainSegment),
}

impl ShapeGeometry {
    /// The shape type tag for this geometry.
    pub fn shape_type(&self) -> ShapeType {
        match self {
            ShapeGeometry::Circle(_) => ShapeType::Circle,
            ShapeGeometry::Capsule(_) => ShapeType::Capsule,
            ShapeGeometry::Segment(_) => ShapeType::Segment,
            ShapeGeometry::Polygon(_) => ShapeType::Polygon,
            ShapeGeometry::ChainSegment(_) => ShapeType::ChainSegment,
        }
    }
}

/// A manifold point is a contact point belonging to a contact manifold. It
/// holds details related to the geometry and dynamics of the contact points.
/// Box2D uses speculative collision so some contact points may be separated.
/// (b2ManifoldPoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ManifoldPoint {
    /// Location of the contact point relative to body A's center of mass in
    /// world space.
    pub anchor_a: Vec2,
    /// Location of the contact point relative to body B's center of mass in
    /// world space.
    pub anchor_b: Vec2,
    /// The separation of the contact point, negative if penetrating
    pub separation: f32,
    /// Cached separation used for contact recycling
    pub base_separation: f32,
    /// The impulse along the manifold normal vector
    pub normal_impulse: f32,
    /// The friction impulse
    pub tangent_impulse: f32,
    /// The total normal impulse applied across sub-stepping and restitution
    pub total_normal_impulse: f32,
    /// Relative normal velocity pre-solve. Used for hit events.
    pub normal_velocity: f32,
    /// Uniquely identifies a contact point between two shapes
    pub id: u16,
    /// Did this contact point exist the previous step?
    pub persisted: bool,
}

/// A contact manifold describes the contact points between colliding shapes.
/// (b2Manifold)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Manifold {
    /// The unit normal vector in world space, points from shape A to body B
    pub normal: Vec2,
    /// Angular impulse applied for rolling resistance
    pub rolling_impulse: f32,
    /// The manifold points, up to two are possible in 2D
    pub points: [ManifoldPoint; 2],
    /// The number of contact points, will be 0, 1, or 2
    pub point_count: i32,
}

/// Contact manifold point in local coordinates (frame A).
/// (b2LocalManifoldPoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LocalManifoldPoint {
    /// Contact point in frame A
    pub point: Vec2,
    /// The separation of the contact point, negative if penetrating
    pub separation: f32,
    /// Uniquely identifies a contact point between two shapes
    pub id: u16,
}

/// Contact manifold in local coordinates (frame A). (b2LocalManifold)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LocalManifold {
    /// The unit normal vector in frame A, points from shape A to shape B
    pub normal: Vec2,
    /// The manifold points, up to two are possible in 2D
    pub points: [LocalManifoldPoint; 2],
    /// The number of contact points, will be 0, 1, or 2
    pub point_count: i32,
}

/// These are the collision planes returned from b2World_CollideMover.
/// The plane and point are relative to the query origin, matching the mover
/// capsule. (b2PlaneResult)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneResult {
    /// The collision plane between the mover and a convex shape
    pub plane: Plane,
    /// The collision point on the shape.
    pub point: Vec2,
    /// Did the collision register a hit? If not this plane should be ignored.
    pub hit: bool,
}

impl Default for PlaneResult {
    fn default() -> Self {
        PlaneResult {
            plane: Plane {
                normal: VEC2_ZERO,
                offset: 0.0,
            },
            point: VEC2_ZERO,
            hit: false,
        }
    }
}
