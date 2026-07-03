// Contact register table and contact creation/destruction from contact.c.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{contact_flags, Contact, ContactSim};
use crate::body::body_flags;
use crate::collision::{Manifold, ShapeType};
use crate::constants::GRAPH_COLOR_COUNT;
use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::events::ContactEndTouchEvent;
use crate::id::{ContactId, ShapeId};
use crate::solver_set::{AWAKE_SET, DISABLED_SET, STATIC_SET};
use crate::table::shape_pair_key;
use crate::world::World;

/// The pairs registered with primary order by b2InitializeContactRegisters.
fn is_primary_pair(type_a: ShapeType, type_b: ShapeType) -> bool {
    use ShapeType::*;
    matches!(
        (type_a, type_b),
        (Circle, Circle)
            | (Capsule, Circle)
            | (Capsule, Capsule)
            | (Polygon, Circle)
            | (Polygon, Capsule)
            | (Polygon, Polygon)
            | (Segment, Circle)
            | (Segment, Capsule)
            | (Segment, Polygon)
            | (ChainSegment, Circle)
            | (ChainSegment, Capsule)
            | (ChainSegment, Polygon)
    )
}

/// The C contact register table (s_registers) reduces to a match in Rust: for
/// a shape type pair, is there a manifold function, and is (a, b) the primary
/// (unflipped) order? (b2InitializeContactRegisters/b2AddType)
pub(super) fn contact_register(type_a: ShapeType, type_b: ShapeType) -> Option<bool> {
    if is_primary_pair(type_a, type_b) {
        return Some(true);
    }

    // The flipped table entry (b2AddType registers type2/type1 with
    // primary = false when the types differ).
    if type_a != type_b && is_primary_pair(type_b, type_a) {
        return Some(false);
    }

    None
}

/// (b2CanCollide)
pub fn can_collide(type_a: ShapeType, type_b: ShapeType) -> bool {
    contact_register(type_a, type_b).is_some()
}

/// WARNING: this should never fail to create a contact because the pair
/// already exists in the pairSet. (b2CreateContact — C takes shape pointers;
/// the Rust port takes shape ids.)
pub fn create_contact(world: &mut World, shape_id_a: i32, shape_id_b: i32) {
    let type_a = world.shapes[shape_id_a as usize].shape_type();
    let type_b = world.shapes[shape_id_b as usize].shape_type();

    let Some(primary) = contact_register(type_a, type_b) else {
        // For example, no segment vs segment collision
        return;
    };

    if !primary {
        // flip order
        create_contact(world, shape_id_b, shape_id_a);
        return;
    }

    let body_id_a = world.shapes[shape_id_a as usize].body_id;
    let body_id_b = world.shapes[shape_id_b as usize].body_id;

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;
    debug_assert!(set_a != DISABLED_SET && set_b != DISABLED_SET);
    debug_assert!(set_a != STATIC_SET || set_b != STATIC_SET);

    let set_index = if set_a == AWAKE_SET || set_b == AWAKE_SET {
        AWAKE_SET
    } else {
        // sleeping and non-touching contacts live in the disabled set
        // later if this set is found to be touching then the sleeping
        // islands will be linked and the contact moved to the merged island
        DISABLED_SET
    };

    // Create contact key and contact
    let contact_id = world.contact_id_pool.alloc_id();
    if contact_id == world.contacts.len() as i32 {
        world.contacts.push(Contact::default());
    }

    let local_index = world.solver_sets[set_index as usize].contact_sims.len() as i32;

    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.contact_id = contact_id;
        contact.generation = contact.generation.wrapping_add(1);
        contact.set_index = set_index;
        contact.color_index = NULL_INDEX;
        contact.local_index = local_index;
        contact.island_id = NULL_INDEX;
        contact.island_index = NULL_INDEX;
        contact.shape_id_a = shape_id_a;
        contact.shape_id_b = shape_id_b;
        contact.flags = 0;
    }

    // Both bodies must enable recycling
    if (world.bodies[body_id_a as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING) != 0
        && (world.bodies[body_id_b as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING) != 0
    {
        world.contacts[contact_id as usize].flags |= contact_flags::RECYCLE;
    }

    debug_assert!(
        world.shapes[shape_id_a as usize].sensor_index == NULL_INDEX
            && world.shapes[shape_id_b as usize].sensor_index == NULL_INDEX
    );

    if world.shapes[shape_id_a as usize].enable_contact_events
        || world.shapes[shape_id_b as usize].enable_contact_events
    {
        world.contacts[contact_id as usize].flags |= contact_flags::ENABLE_CONTACT_EVENTS;
    }

    // Connect to body A
    {
        let head_contact_key = world.bodies[body_id_a as usize].head_contact_key;
        {
            let contact = &mut world.contacts[contact_id as usize];
            contact.edges[0].body_id = body_id_a;
            contact.edges[0].prev_key = NULL_INDEX;
            contact.edges[0].next_key = head_contact_key;
        }

        let key_a = contact_id << 1;
        if head_contact_key != NULL_INDEX {
            let head_contact = &mut world.contacts[(head_contact_key >> 1) as usize];
            head_contact.edges[(head_contact_key & 1) as usize].prev_key = key_a;
        }
        let body_a = &mut world.bodies[body_id_a as usize];
        body_a.head_contact_key = key_a;
        body_a.contact_count += 1;
    }

    // Connect to body B
    {
        let head_contact_key = world.bodies[body_id_b as usize].head_contact_key;
        {
            let contact = &mut world.contacts[contact_id as usize];
            contact.edges[1].body_id = body_id_b;
            contact.edges[1].prev_key = NULL_INDEX;
            contact.edges[1].next_key = head_contact_key;
        }

        let key_b = (contact_id << 1) | 1;
        if head_contact_key != NULL_INDEX {
            let head_contact = &mut world.contacts[(head_contact_key >> 1) as usize];
            head_contact.edges[(head_contact_key & 1) as usize].prev_key = key_b;
        }
        let body_b = &mut world.bodies[body_id_b as usize];
        body_b.head_contact_key = key_b;
        body_b.contact_count += 1;
    }

    // Add to pair set for fast lookup.
    let pair_key = shape_pair_key(shape_id_a, shape_id_b);
    world.broad_phase.pair_set.add_key(pair_key);

    // Contacts are created as non-touching. Later if they are found to be
    // touching they will link islands and be moved into the constraint graph.
    let contact_flags_now = world.contacts[contact_id as usize].flags;
    let shape_a = &world.shapes[shape_id_a as usize];
    let shape_b = &world.shapes[shape_id_b as usize];

    let mut contact_sim = ContactSim {
        contact_id,
        // C: #if B2_ENABLE_VALIDATION — always present in the Rust port
        body_id_a,
        body_id_b,
        body_sim_index_a: NULL_INDEX,
        body_sim_index_b: NULL_INDEX,
        inv_mass_a: 0.0,
        inv_i_a: 0.0,
        inv_mass_b: 0.0,
        inv_i_b: 0.0,
        shape_id_a,
        shape_id_b,
        cache: SimplexCache::default(),
        manifold: Manifold::default(),
        // These get updated in the narrow phase, but these are needed for
        // first touch
        friction: (world.friction_callback.unwrap())(
            shape_a.material.friction,
            shape_a.material.user_material_id,
            shape_b.material.friction,
            shape_b.material.user_material_id,
        ),
        restitution: (world.restitution_callback.unwrap())(
            shape_a.material.restitution,
            shape_a.material.user_material_id,
            shape_b.material.restitution,
            shape_b.material.user_material_id,
        ),
        tangent_speed: 0.0,
        sim_flags: contact_flags_now,
        ..ContactSim::default()
    };

    if shape_a.enable_pre_solve_events || shape_b.enable_pre_solve_events {
        contact_sim.sim_flags |= contact_flags::SIM_ENABLE_PRE_SOLVE_EVENTS;
    }

    world.solver_sets[set_index as usize]
        .contact_sims
        .push(contact_sim);
}

/// A contact is destroyed when:
/// - broad-phase proxies stop overlapping
/// - a body is destroyed
/// - a body is disabled
/// - a body changes type from dynamic to kinematic or static
/// - a shape is destroyed
/// - contact filtering is modified
///
/// (b2DestroyContact — C takes a contact pointer; the Rust port takes the id.)
pub fn destroy_contact(world: &mut World, contact_id: i32, wake_bodies: bool) {
    let (shape_id_a, shape_id_b, edge_a, edge_b, flags, generation) = {
        let contact = &world.contacts[contact_id as usize];
        (
            contact.shape_id_a,
            contact.shape_id_b,
            contact.edges[0],
            contact.edges[1],
            contact.flags,
            contact.generation,
        )
    };

    // Remove pair from set
    let pair_key = shape_pair_key(shape_id_a, shape_id_b);
    world.broad_phase.pair_set.remove_key(pair_key);

    let body_id_a = edge_a.body_id;
    let body_id_b = edge_b.body_id;

    let touching = (flags & contact_flags::TOUCHING) != 0;

    // End touch event
    if touching && (flags & contact_flags::ENABLE_CONTACT_EVENTS) != 0 {
        let world_id = world.world_id;
        let shape_a = &world.shapes[shape_id_a as usize];
        let shape_b = &world.shapes[shape_id_b as usize];

        let event = ContactEndTouchEvent {
            shape_id_a: ShapeId {
                index1: shape_a.id + 1,
                world0: world_id,
                generation: shape_a.generation,
            },
            shape_id_b: ShapeId {
                index1: shape_b.id + 1,
                world0: world_id,
                generation: shape_b.generation,
            },
            contact_id: ContactId {
                index1: contact_id + 1,
                world0: world_id,
                padding: 0,
                generation,
            },
        };

        world.contact_end_events[world.end_event_array_index as usize].push(event);
    }

    // Remove from body A
    if edge_a.prev_key != NULL_INDEX {
        let prev_contact = &mut world.contacts[(edge_a.prev_key >> 1) as usize];
        prev_contact.edges[(edge_a.prev_key & 1) as usize].next_key = edge_a.next_key;
    }

    if edge_a.next_key != NULL_INDEX {
        let next_contact = &mut world.contacts[(edge_a.next_key >> 1) as usize];
        next_contact.edges[(edge_a.next_key & 1) as usize].prev_key = edge_a.prev_key;
    }

    let edge_key_a = contact_id << 1;
    {
        let body_a = &mut world.bodies[body_id_a as usize];
        if body_a.head_contact_key == edge_key_a {
            body_a.head_contact_key = edge_a.next_key;
        }
        body_a.contact_count -= 1;
    }

    // Remove from body B
    if edge_b.prev_key != NULL_INDEX {
        let prev_contact = &mut world.contacts[(edge_b.prev_key >> 1) as usize];
        prev_contact.edges[(edge_b.prev_key & 1) as usize].next_key = edge_b.next_key;
    }

    if edge_b.next_key != NULL_INDEX {
        let next_contact = &mut world.contacts[(edge_b.next_key >> 1) as usize];
        next_contact.edges[(edge_b.next_key & 1) as usize].prev_key = edge_b.prev_key;
    }

    let edge_key_b = (contact_id << 1) | 1;
    {
        let body_b = &mut world.bodies[body_id_b as usize];
        if body_b.head_contact_key == edge_key_b {
            body_b.head_contact_key = edge_b.next_key;
        }
        body_b.contact_count -= 1;
    }

    // Remove contact from the array that owns it
    if world.contacts[contact_id as usize].island_id != NULL_INDEX {
        crate::island::unlink_contact(world, contact_id);
    }

    let (color_index, local_index, set_index) = {
        let contact = &world.contacts[contact_id as usize];
        (contact.color_index, contact.local_index, contact.set_index)
    };
    if color_index != NULL_INDEX {
        // contact is an active constraint
        debug_assert!(set_index == AWAKE_SET);
        crate::constraint_graph::remove_contact_from_graph(
            world,
            body_id_a,
            body_id_b,
            color_index,
            local_index,
        );
    } else {
        // contact is non-touching or is sleeping
        debug_assert!(
            set_index != AWAKE_SET
                || (world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) == 0
        );
        let set = &mut world.solver_sets[set_index as usize];
        let moved_index = set.contact_sims.len() as i32 - 1;
        set.contact_sims.swap_remove(local_index as usize);
        if moved_index != local_index {
            let moved_contact_id =
                world.solver_sets[set_index as usize].contact_sims[local_index as usize].contact_id;
            world.contacts[moved_contact_id as usize].local_index = local_index;
        }
    }

    // Free contact and id (preserve generation)
    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.contact_id = NULL_INDEX;
        contact.set_index = NULL_INDEX;
        contact.color_index = NULL_INDEX;
        contact.local_index = NULL_INDEX;
    }
    world.contact_id_pool.free_id(contact_id);

    if wake_bodies && touching {
        crate::body::wake_body(world, body_id_a);
        crate::body::wake_body(world, body_id_b);
    }
}

/// Borrow a contact's sim data mutably: constraint graph color for awake
/// touching contacts, otherwise the owning solver set. (b2GetContactSim)
pub fn get_contact_sim(world: &mut World, contact_id: i32) -> &mut ContactSim {
    let (set_index, color_index, local_index) = {
        let contact = &world.contacts[contact_id as usize];
        (contact.set_index, contact.color_index, contact.local_index)
    };

    if set_index == AWAKE_SET && color_index != NULL_INDEX {
        // contact lives in constraint graph
        debug_assert!((0..GRAPH_COLOR_COUNT).contains(&color_index));
        &mut world.constraint_graph.colors[color_index as usize].contact_sims[local_index as usize]
    } else {
        &mut world.solver_sets[set_index as usize].contact_sims[local_index as usize]
    }
}
