// Port of the sensor data model from box2d-cpp-reference/src/sensor.h plus
// sensor destruction from sensor.c. The overlap update logic lands in a later
// bring-up commit.
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

/// Destroy the sensor record for a sensor shape, emitting end-touch events for
/// its active overlaps. (b2DestroySensor — C takes the shape pointer; the Rust
/// port takes the shape id.)
pub fn destroy_sensor(world: &mut crate::world::World, sensor_shape_id: i32) {
    use crate::events::SensorEndTouchEvent;
    use crate::id::ShapeId;

    let sensor_index = world.shapes[sensor_shape_id as usize].sensor_index;
    let sensor_generation = world.shapes[sensor_shape_id as usize].generation;

    let overlaps2 = std::mem::take(&mut world.sensors[sensor_index as usize].overlaps2);
    for visitor in &overlaps2 {
        let event = SensorEndTouchEvent {
            sensor_shape_id: ShapeId {
                index1: sensor_shape_id + 1,
                world0: world.world_id,
                generation: sensor_generation,
            },
            visitor_shape_id: ShapeId {
                index1: visitor.shape_id + 1,
                world0: world.world_id,
                generation: visitor.generation,
            },
        };

        world.sensor_end_events[world.end_event_array_index as usize].push(event);
    }

    // Destroy sensor (the C b2Array_Destroy calls drop with the Sensor)
    let moved_index = world.sensors.len() as i32 - 1;
    world.sensors.swap_remove(sensor_index as usize);
    if moved_index != sensor_index {
        // Fixup moved sensor
        let moved_shape_id = world.sensors[sensor_index as usize].shape_id;
        world.shapes[moved_shape_id as usize].sensor_index = sensor_index;
    }
}
