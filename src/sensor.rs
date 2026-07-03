// Port of the sensor data model from box2d-cpp-reference/src/sensor.h.
// Logic from sensor.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::core::NULL_INDEX;

/// Used to track shapes that hit sensors using time of impact. (b2SensorHit)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorHit {
    pub sensor_id: i32,
    pub visitor_id: i32,
}

/// (b2Visitor)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Visitor {
    pub shape_id: i32,
    pub generation: u16,
}

/// Sensors are shapes that live in the broad-phase but never have contacts.
/// At the end of the time step all sensors are queried for overlap with any
/// other shapes. Sensors ignore body type and sleeping. Sensors generate
/// events when a new overlap appears or an overlap disappears. (b2Sensor)
#[derive(Debug, Clone, Default)]
pub struct Sensor {
    pub hits: Vec<Visitor>,
    pub overlaps1: Vec<Visitor>,
    pub overlaps2: Vec<Visitor>,
    pub shape_id: i32,
}

impl Sensor {
    pub fn new(shape_id: i32) -> Sensor {
        Sensor {
            hits: Vec::new(),
            overlaps1: Vec::new(),
            overlaps2: Vec::new(),
            shape_id,
        }
    }
}

/// (b2SensorTaskContext)
#[derive(Debug, Clone, Default)]
pub struct SensorTaskContext {
    pub event_bits: BitSet,
}

/// (b2SensorOverlaps, from shape.h)
#[derive(Debug, Clone, Default)]
pub struct SensorOverlaps {
    pub overlaps: Vec<i32>,
}

impl Default for SensorHit {
    fn default() -> Self {
        SensorHit {
            sensor_id: NULL_INDEX,
            visitor_id: NULL_INDEX,
        }
    }
}
