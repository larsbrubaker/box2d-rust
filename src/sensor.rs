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

/// Update sensor overlaps and publish begin/end touch events.
/// (b2SensorTask + b2OverlapSensors — serial: one worker, one range)
pub fn overlap_sensors(world: &mut crate::world::World) {
    use crate::distance::{shape_distance, DistanceInput, SimplexCache};
    use crate::events::{SensorBeginTouchEvent, SensorEndTouchEvent};
    use crate::id::ShapeId;
    use crate::math_functions::inv_mul_world_transforms;
    use crate::shape::{make_shape_distance_proxy, should_shapes_collide};
    use crate::solver_set::DISABLED_SET;

    let sensor_count = world.sensors.len();
    if sensor_count == 0 {
        return;
    }

    world.sensor_task_contexts[0]
        .event_bits
        .set_bit_count_and_clear(sensor_count as u32);

    // (b2SensorTask over [0, sensorCount))
    for sensor_index in 0..sensor_count {
        // Swap overlap arrays
        {
            let sensor = &mut world.sensors[sensor_index];
            std::mem::swap(&mut sensor.overlaps1, &mut sensor.overlaps2);
            sensor.overlaps2.clear();

            // Append sensor hits, then clear them
            let hits = std::mem::take(&mut sensor.hits);
            sensor.overlaps2.extend_from_slice(&hits);
        }

        let sensor_shape_id = world.sensors[sensor_index].shape_id;
        let (sensor_body_id, sensor_enabled) = {
            let shape = &world.shapes[sensor_shape_id as usize];
            (shape.body_id, shape.enable_sensor_events)
        };

        if world.bodies[sensor_body_id as usize].set_index == DISABLED_SET || !sensor_enabled {
            if !world.sensors[sensor_index].overlaps1.is_empty() {
                // This sensor is dropping all overlaps because it has been
                // disabled.
                world.sensor_task_contexts[0]
                    .event_bits
                    .set_bit(sensor_index as u32);
            }
            continue;
        }

        let transform = crate::body::get_body_transform(world, sensor_body_id);

        debug_assert!(world.shapes[sensor_shape_id as usize].sensor_index == sensor_index as i32);
        let query_bounds = world.shapes[sensor_shape_id as usize].aabb;
        let mask_bits = world.shapes[sensor_shape_id as usize].filter.mask_bits;

        // Query all trees. The callback (b2SensorQueryCallback) collects
        // overlaps into a local list; the world stays borrowed shared.
        let mut new_overlaps: Vec<Visitor> = Vec::new();
        {
            let world_ref: &crate::world::World = world;
            let sensor_shape = &world_ref.shapes[sensor_shape_id as usize];

            let mut callback = |_proxy_id: i32, user_data: u64| -> bool {
                let shape_id = user_data as i32;

                if shape_id == sensor_shape_id {
                    return true;
                }

                let other_shape = &world_ref.shapes[shape_id as usize];

                // Are sensor events enabled on the other shape?
                if !other_shape.enable_sensor_events {
                    return true;
                }

                // Skip shapes on the same body
                if other_shape.body_id == sensor_shape.body_id {
                    return true;
                }

                // Check filter
                if !should_shapes_collide(sensor_shape.filter, other_shape.filter) {
                    return true;
                }

                // Custom user filter
                if sensor_shape.enable_custom_filtering || other_shape.enable_custom_filtering {
                    if let Some(custom_filter_fcn) = world_ref.custom_filter_fcn {
                        let id_a = ShapeId {
                            index1: sensor_shape_id + 1,
                            world0: world_ref.world_id,
                            generation: sensor_shape.generation,
                        };
                        let id_b = ShapeId {
                            index1: shape_id + 1,
                            world0: world_ref.world_id,
                            generation: other_shape.generation,
                        };
                        if !custom_filter_fcn(id_a, id_b, world_ref.custom_filter_context) {
                            return true;
                        }
                    }
                }

                // The relative pose is differenced in double so sensor overlap
                // stays exact far from the origin
                let other_transform =
                    crate::body::get_body_transform(world_ref, other_shape.body_id);

                let input = DistanceInput {
                    proxy_a: make_shape_distance_proxy(sensor_shape),
                    proxy_b: make_shape_distance_proxy(other_shape),
                    transform: inv_mul_world_transforms(transform, other_transform),
                    use_radii: true,
                };
                let mut cache = SimplexCache::default();
                let output = shape_distance(&input, &mut cache, None);

                let overlaps = output.distance < 10.0 * f32::EPSILON;
                if !overlaps {
                    return true;
                }

                // Record the overlap
                new_overlaps.push(Visitor {
                    shape_id,
                    generation: other_shape.generation,
                });

                true
            };

            for tree in &world_ref.broad_phase.trees {
                tree.query(query_bounds, mask_bits, &mut callback);
            }
        }

        let sensor = &mut world.sensors[sensor_index];
        sensor.overlaps2.extend_from_slice(&new_overlaps);

        // Sort the overlaps to enable finding begin and end events. (The C
        // comparator orders by shapeId only; duplicates are identical
        // Visitors, so a stable key sort matches.)
        sensor.overlaps2.sort_by_key(|visitor| visitor.shape_id);

        // Remove duplicates from overlaps2 (sorted). Duplicates are possible
        // due to the hit events appended earlier.
        sensor.overlaps2.dedup_by_key(|visitor| visitor.shape_id);

        let count1 = sensor.overlaps1.len();
        let count2 = sensor.overlaps2.len();
        if count1 != count2 {
            // something changed
            world.sensor_task_contexts[0]
                .event_bits
                .set_bit(sensor_index as u32);
        } else {
            for i in 0..count1 {
                let s1 = sensor.overlaps1[i];
                let s2 = sensor.overlaps2[i];

                if s1.shape_id != s2.shape_id || s1.generation != s2.generation {
                    // something changed
                    world.sensor_task_contexts[0]
                        .event_bits
                        .set_bit(sensor_index as u32);
                    break;
                }
            }
        }
    }

    // Iterate sensor bits and publish events (b2OverlapSensors tail)
    let world_id = world.world_id;
    let end_event_array_index = world.end_event_array_index as usize;

    let block_count = world.sensor_task_contexts[0].event_bits.block_count();
    for k in 0..block_count {
        let mut word = world.sensor_task_contexts[0].event_bits.block(k);
        while word != 0 {
            let ctz = word.trailing_zeros();
            let sensor_index = (64 * k + ctz) as usize;

            let sensor_shape_id = world.sensors[sensor_index].shape_id;
            let sensor_generation = world.shapes[sensor_shape_id as usize].generation;
            let sensor_id = ShapeId {
                index1: sensor_shape_id + 1,
                world0: world_id,
                generation: sensor_generation,
            };

            let count1 = world.sensors[sensor_index].overlaps1.len();
            let count2 = world.sensors[sensor_index].overlaps2.len();

            // overlaps1 can have overlaps that end
            // overlaps2 can have overlaps that begin
            let mut index1 = 0;
            let mut index2 = 0;
            while index1 < count1 && index2 < count2 {
                let r1 = world.sensors[sensor_index].overlaps1[index1];
                let r2 = world.sensors[sensor_index].overlaps2[index2];
                if r1.shape_id == r2.shape_id {
                    match r1.generation.cmp(&r2.generation) {
                        std::cmp::Ordering::Less => {
                            // end
                            let visitor_id = ShapeId {
                                index1: r1.shape_id + 1,
                                world0: world_id,
                                generation: r1.generation,
                            };
                            world.sensor_end_events[end_event_array_index].push(
                                SensorEndTouchEvent {
                                    sensor_shape_id: sensor_id,
                                    visitor_shape_id: visitor_id,
                                },
                            );
                            index1 += 1;
                        }
                        std::cmp::Ordering::Greater => {
                            // begin
                            let visitor_id = ShapeId {
                                index1: r2.shape_id + 1,
                                world0: world_id,
                                generation: r2.generation,
                            };
                            world.sensor_begin_events.push(SensorBeginTouchEvent {
                                sensor_shape_id: sensor_id,
                                visitor_shape_id: visitor_id,
                            });
                            index2 += 1;
                        }
                        std::cmp::Ordering::Equal => {
                            // persisted
                            index1 += 1;
                            index2 += 1;
                        }
                    }
                } else if r1.shape_id < r2.shape_id {
                    // end
                    let visitor_id = ShapeId {
                        index1: r1.shape_id + 1,
                        world0: world_id,
                        generation: r1.generation,
                    };
                    world.sensor_end_events[end_event_array_index].push(SensorEndTouchEvent {
                        sensor_shape_id: sensor_id,
                        visitor_shape_id: visitor_id,
                    });
                    index1 += 1;
                } else {
                    // begin
                    let visitor_id = ShapeId {
                        index1: r2.shape_id + 1,
                        world0: world_id,
                        generation: r2.generation,
                    };
                    world.sensor_begin_events.push(SensorBeginTouchEvent {
                        sensor_shape_id: sensor_id,
                        visitor_shape_id: visitor_id,
                    });
                    index2 += 1;
                }
            }

            while index1 < count1 {
                // end
                let r1 = world.sensors[sensor_index].overlaps1[index1];
                let visitor_id = ShapeId {
                    index1: r1.shape_id + 1,
                    world0: world_id,
                    generation: r1.generation,
                };
                world.sensor_end_events[end_event_array_index].push(SensorEndTouchEvent {
                    sensor_shape_id: sensor_id,
                    visitor_shape_id: visitor_id,
                });
                index1 += 1;
            }

            while index2 < count2 {
                // begin
                let r2 = world.sensors[sensor_index].overlaps2[index2];
                let visitor_id = ShapeId {
                    index1: r2.shape_id + 1,
                    world0: world_id,
                    generation: r2.generation,
                };
                world.sensor_begin_events.push(SensorBeginTouchEvent {
                    sensor_shape_id: sensor_id,
                    visitor_shape_id: visitor_id,
                });
                index2 += 1;
            }

            // Clear the smallest set bit
            word &= word - 1;
        }
    }
}
