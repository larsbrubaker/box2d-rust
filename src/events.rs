// Port of the event types from include/box2d/types.h (events group).
//
// The C aggregate views (b2SensorEvents, b2ContactEvents, b2BodyEvents,
// b2JointEvents) are pointer+count pairs over world-owned arrays. The Rust
// world API returns slices; b2BodyEvents/b2JointEvents collapse to a single
// slice while SensorEvents/ContactEvents keep grouping structs because they
// bundle several arrays.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::Manifold;
use crate::id::{BodyId, ContactId, JointId, ShapeId};
use crate::math_functions::{Pos, Vec2, WorldTransform};

/// A begin touch event is generated when a shape starts to overlap a sensor
/// shape. (b2SensorBeginTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorBeginTouchEvent {
    /// The id of the sensor shape
    pub sensor_shape_id: ShapeId,
    /// The id of the shape that began touching the sensor shape
    pub visitor_shape_id: ShapeId,
}

/// An end touch event is generated when a shape stops overlapping a sensor
/// shape. Always confirm the shape id is valid before use — either shape may
/// have been destroyed. (b2SensorEndTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorEndTouchEvent {
    /// The id of the sensor shape. @warning may have been destroyed
    pub sensor_shape_id: ShapeId,
    /// The id of the shape that stopped touching. @warning may have been destroyed
    pub visitor_shape_id: ShapeId,
}

/// A begin touch event is generated when two shapes begin touching.
/// (b2ContactBeginTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactBeginTouchEvent {
    /// Id of the first shape
    pub shape_id_a: ShapeId,
    /// Id of the second shape
    pub shape_id_b: ShapeId,
    /// The transient contact id. May be destroyed automatically when the world
    /// is modified or simulated.
    pub contact_id: ContactId,
}

/// An end touch event is generated when two shapes stop touching.
/// (b2ContactEndTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactEndTouchEvent {
    /// Id of the first shape. @warning may have been destroyed
    pub shape_id_a: ShapeId,
    /// Id of the second shape. @warning may have been destroyed
    pub shape_id_b: ShapeId,
    /// Id of the contact. @warning may have been destroyed
    pub contact_id: ContactId,
}

/// A hit touch event is generated when two shapes collide faster than the hit
/// speed threshold. (b2ContactHitEvent)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactHitEvent {
    /// Id of the first shape
    pub shape_id_a: ShapeId,
    /// Id of the second shape
    pub shape_id_b: ShapeId,
    /// Id of the contact. @warning may have been destroyed
    pub contact_id: ContactId,
    /// Point where the shapes hit, a mid-point between the two surfaces.
    pub point: Pos,
    /// Normal vector pointing from shape A to shape B
    pub normal: Vec2,
    /// The speed the shapes are approaching. Always positive.
    pub approach_speed: f32,
}

/// Body move event, triggered when a body moves due to simulation. Not
/// reported for bodies moved by the user. (b2BodyMoveEvent)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyMoveEvent {
    pub user_data: u64,
    pub transform: WorldTransform,
    pub body_id: BodyId,
    pub fell_asleep: bool,
}

/// Joint event, reported for awake joints whose force and/or torque exceed the
/// threshold. (b2JointEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JointEvent {
    /// The joint id
    pub joint_id: JointId,
    /// The user data from the joint for convenience
    pub user_data: u64,
}

/// Sensor events are buffered in the world and are available as begin/end
/// events in the current time step. These are borrowed from world storage and
/// invalidated by the next call to `world_step`. (b2SensorEvents)
#[derive(Debug, Clone, Copy)]
pub struct SensorEvents<'a> {
    /// Events for shapes that began overlapping a sensor this step.
    pub begin_events: &'a [SensorBeginTouchEvent],
    /// Events for shapes that stopped overlapping a sensor. These are from the
    /// previous buffer so the user doesn't need to flush events mid-step.
    pub end_events: &'a [SensorEndTouchEvent],
}

/// Contact events are buffered in the world and are available as begin/end/hit
/// events in the current time step. These are borrowed from world storage and
/// invalidated by the next call to `world_step`. (b2ContactEvents)
#[derive(Debug, Clone, Copy)]
pub struct ContactEvents<'a> {
    /// Events for shapes that began touching this step.
    pub begin_events: &'a [ContactBeginTouchEvent],
    /// Events for shapes that stopped touching. These are from the previous
    /// buffer so the user doesn't need to flush events mid-step.
    pub end_events: &'a [ContactEndTouchEvent],
    /// Events for impacts above the hit-event threshold.
    pub hit_events: &'a [ContactHitEvent],
}

/// The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B. (b2ContactData)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactData {
    pub contact_id: ContactId,
    pub shape_id_a: ShapeId,
    pub shape_id_b: ShapeId,
    pub manifold: Manifold,
}
