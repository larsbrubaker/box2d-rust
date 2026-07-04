// Body public API from body.c, part 3: shape/joint/contact list accessors
// and connectivity validation (b2Body_*).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::get_body_full_id;
use crate::core::NULL_INDEX;
use crate::id::{BodyId, JointId, ShapeId};
use crate::solver_set::{AWAKE_SET, DISABLED_SET, STATIC_SET};
use crate::types::BodyType;
use crate::world::World;

/// (b2Body_GetShapeCount)
pub fn body_get_shape_count(world: &World, body_id: BodyId) -> i32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].shape_count
}

/// Fills at most `capacity` shape ids. (b2Body_GetShapes)
pub fn body_get_shapes(world: &World, body_id: BodyId, capacity: usize) -> Vec<ShapeId> {
    let body_index = get_body_full_id(world, body_id);
    let mut out = Vec::new();
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX && out.len() < capacity {
        let shape = &world.shapes[shape_id as usize];
        out.push(ShapeId {
            index1: shape_id + 1,
            world0: world.world_id,
            generation: shape.generation,
        });
        shape_id = shape.next_shape_id;
    }
    out
}

/// (b2Body_GetJointCount)
pub fn body_get_joint_count(world: &World, body_id: BodyId) -> i32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].joint_count
}

/// Fills at most `capacity` joint ids. (b2Body_GetJoints)
pub fn body_get_joints(world: &World, body_id: BodyId, capacity: usize) -> Vec<JointId> {
    let body_index = get_body_full_id(world, body_id);
    let mut out = Vec::new();
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX && out.len() < capacity {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        let joint = &world.joints[joint_id as usize];
        out.push(JointId {
            index1: joint_id + 1,
            world0: world.world_id,
            generation: joint.generation,
        });

        joint_key = joint.edges[edge_index as usize].next_key;
    }
    out
}

/// Conservative and fast. (b2Body_GetContactCapacity)
pub fn body_get_contact_capacity(world: &World, body_id: BodyId) -> i32 {
    debug_assert!(!world.locked);
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].contact_count
}

/// Touching contact data for a body, at most `capacity` entries.
/// (b2Body_GetContactData)
pub fn body_get_contact_data(
    world: &World,
    body_id: BodyId,
    capacity: usize,
) -> Vec<crate::events::ContactData> {
    debug_assert!(!world.locked);
    let body_index = get_body_full_id(world, body_id);

    let mut out = Vec::new();
    let mut contact_key = world.bodies[body_index as usize].head_contact_key;
    while contact_key != NULL_INDEX && out.len() < capacity {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        let contact = &world.contacts[contact_id as usize];

        // Is contact touching?
        if contact.flags & crate::contact::contact_flags::TOUCHING != 0 {
            let shape_a = &world.shapes[contact.shape_id_a as usize];
            let shape_b = &world.shapes[contact.shape_id_b as usize];

            let contact_sim = if contact.set_index == AWAKE_SET && contact.color_index != NULL_INDEX
            {
                &world.constraint_graph.colors[contact.color_index as usize].contact_sims
                    [contact.local_index as usize]
            } else {
                &world.solver_sets[contact.set_index as usize].contact_sims
                    [contact.local_index as usize]
            };

            out.push(crate::events::ContactData {
                contact_id: crate::id::ContactId {
                    index1: contact_id + 1,
                    world0: world.world_id,
                    padding: 0,
                    generation: contact.generation,
                },
                shape_id_a: ShapeId {
                    index1: shape_a.id + 1,
                    world0: world.world_id,
                    generation: shape_a.generation,
                },
                shape_id_b: ShapeId {
                    index1: shape_b.id + 1,
                    world0: world.world_id,
                    generation: shape_b.generation,
                },
                manifold: contact_sim.manifold,
            });
        }

        contact_key = contact.edges[edge_index as usize].next_key;
    }

    out
}

impl World {
    /// (b2ValidateConnectivity — C compiles the body under
    /// B2_ENABLE_VALIDATION; here the whole check runs in debug builds only)
    pub fn validate_connectivity(&self) {
        if !cfg!(debug_assertions) {
            return;
        }

        for body_index in 0..self.bodies.len() as i32 {
            let body = &self.bodies[body_index as usize];
            if body.id == NULL_INDEX {
                self.body_id_pool.validate_free_id(body_index);
                continue;
            }

            self.body_id_pool.validate_used_id(body_index);

            debug_assert!(body_index == body.id);

            let body_island_id = body.island_id;
            let body_set_index = body.set_index;

            let mut contact_key = body.head_contact_key;
            while contact_key != NULL_INDEX {
                let contact_id = contact_key >> 1;
                let edge_index = contact_key & 1;

                let contact = &self.contacts[contact_id as usize];

                let touching = contact.flags & crate::contact::contact_flags::TOUCHING != 0;
                if touching {
                    if body_set_index != STATIC_SET {
                        debug_assert!(contact.island_id == body_island_id);
                    }
                } else {
                    debug_assert!(contact.island_id == NULL_INDEX);
                }

                contact_key = contact.edges[edge_index as usize].next_key;
            }

            let mut joint_key = body.head_joint_key;
            while joint_key != NULL_INDEX {
                let joint_id = joint_key >> 1;
                let edge_index = joint_key & 1;

                let joint = &self.joints[joint_id as usize];

                let other_edge_index = edge_index ^ 1;
                let other_body =
                    &self.bodies[joint.edges[other_edge_index as usize].body_id as usize];

                if body_set_index == DISABLED_SET || other_body.set_index == DISABLED_SET {
                    debug_assert!(joint.island_id == NULL_INDEX);
                } else if body_set_index == STATIC_SET {
                    // Intentional nesting
                    if other_body.set_index == STATIC_SET {
                        debug_assert!(joint.island_id == NULL_INDEX);
                    }
                } else if body.type_ != BodyType::Dynamic && other_body.type_ != BodyType::Dynamic {
                    debug_assert!(joint.island_id == NULL_INDEX);
                } else {
                    debug_assert!(joint.island_id == body_island_id);
                }

                joint_key = joint.edges[edge_index as usize].next_key;
            }
        }
    }
}
