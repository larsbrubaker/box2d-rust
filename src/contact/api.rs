// Contact public API from contact.c / physics_world.c (b2Contact_*).
// The C resolves the world from the id via the global registry; the Rust
// port takes `world` explicitly. b2Contact_GetWorld is not ported (there is
// no world registry).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_contact_full_id, get_contact_sim_ref};
use crate::core::NULL_INDEX;
use crate::events::ContactData;
use crate::id::{ContactId, ShapeId};
use crate::world::World;

/// Contact identifier validation. Provides validation for up to 2^32
/// allocations. (b2Contact_IsValid — the world-registry check collapses to
/// the index/generation check in the registry-less port)
pub fn contact_is_valid(world: &World, id: ContactId) -> bool {
    let contact_id = id.index1 - 1;
    if contact_id < 0 || world.contacts.len() as i32 <= contact_id {
        return false;
    }

    let contact = &world.contacts[contact_id as usize];
    if contact.contact_id == NULL_INDEX {
        // contact is free
        return false;
    }

    debug_assert!(contact.contact_id == contact_id);

    id.generation == contact.generation
}

/// Get the data for a contact. The manifold may have no points if the
/// contact is not touching. (b2Contact_GetData)
pub fn contact_get_data(world: &World, contact_id: ContactId) -> ContactData {
    let id = get_contact_full_id(world, contact_id);
    let contact = &world.contacts[id as usize];
    let contact_sim = get_contact_sim_ref(world, id);
    let shape_a = &world.shapes[contact.shape_id_a as usize];
    let shape_b = &world.shapes[contact.shape_id_b as usize];

    ContactData {
        contact_id,
        shape_id_a: ShapeId {
            index1: shape_a.id + 1,
            world0: contact_id.world0,
            generation: shape_a.generation,
        },
        shape_id_b: ShapeId {
            index1: shape_b.id + 1,
            world0: contact_id.world0,
            generation: shape_b.generation,
        },
        manifold: contact_sim.manifold,
    }
}
