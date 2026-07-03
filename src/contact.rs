// Port of the contact data model from box2d-cpp-reference/src/contact.h.
// Logic from contact.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::Manifold;
use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::math_functions::{Rot, Transform, ROT_IDENTITY, TRANSFORM_IDENTITY};

// enum b2ContactFlags
pub mod contact_flags {
    /// Set when the solid shapes are touching.
    pub const TOUCHING: u32 = 0x00000001;
    /// Contact has a hit event
    pub const HIT_EVENT: u32 = 0x00000002;
    /// This contact wants contact events
    pub const ENABLE_CONTACT_EVENTS: u32 = 0x00000004;
    pub const RECYCLE: u32 = 0x00000008;

    /// Set when the shapes are touching (sim flag)
    pub const SIM_TOUCHING: u32 = 0x00010000;
    /// This contact no longer has overlapping AABBs
    pub const SIM_DISJOINT: u32 = 0x00020000;
    /// This contact started touching
    pub const SIM_STARTED_TOUCHING: u32 = 0x00040000;
    /// This contact stopped touching
    pub const SIM_STOPPED_TOUCHING: u32 = 0x00080000;
    /// This contact has a hit event
    pub const SIM_ENABLE_HIT_EVENT: u32 = 0x00100000;
    /// This contact wants pre-solve events
    pub const SIM_ENABLE_PRE_SOLVE_EVENTS: u32 = 0x00200000;
    /// This contact has a cached relative transform
    pub const SIM_RELATIVE_TRANSFORM_VALID: u32 = 0x00400000;
}

/// A contact edge is used to connect bodies and contacts together in a contact
/// graph where each body is a node and each contact is an edge. A contact edge
/// belongs to a doubly linked list maintained in each attached body. Each
/// contact has two contact edges, one for each attached body. (b2ContactEdge)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactEdge {
    pub body_id: i32,
    pub prev_key: i32,
    pub next_key: i32,
}

impl Default for ContactEdge {
    fn default() -> Self {
        ContactEdge {
            body_id: NULL_INDEX,
            prev_key: NULL_INDEX,
            next_key: NULL_INDEX,
        }
    }
}

/// Cold contact data. Used as a persistent handle and for persistent island
/// connectivity. (b2Contact)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Contact {
    pub edges: [ContactEdge; 2],

    /// A contact only belongs to an island if touching, otherwise NULL_INDEX.
    pub island_id: i32,

    /// Index into the island's contacts array for O(1) swap-removal.
    /// NULL_INDEX when not in an island.
    pub island_index: i32,

    /// index of simulation set stored in World. NULL_INDEX when slot is free.
    pub set_index: i32,

    /// index into the constraint graph color array. NULL_INDEX for
    /// non-touching or sleeping contacts, and when the slot is free.
    pub color_index: i32,

    /// contact index within set or graph color. NULL_INDEX when slot is free.
    pub local_index: i32,

    pub shape_id_a: i32,
    pub shape_id_b: i32,
    pub contact_id: i32,

    /// contact_flags bits
    pub flags: u32,

    /// Monotonically advanced when a contact is allocated in this slot.
    /// Used to check for invalid ContactId.
    pub generation: u32,
}

impl Default for Contact {
    fn default() -> Self {
        Contact {
            edges: [ContactEdge::default(); 2],
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            set_index: NULL_INDEX,
            color_index: NULL_INDEX,
            local_index: NULL_INDEX,
            shape_id_a: NULL_INDEX,
            shape_id_b: NULL_INDEX,
            contact_id: NULL_INDEX,
            flags: 0,
            generation: 0,
        }
    }
}

/// Manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered), so a contact object may exist
/// that has no contact points. (b2ContactSim)
///
/// The C `#if B2_ENABLE_VALIDATION` bodyIdA/bodyIdB fields are always present
/// here; they only feed validation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactSim {
    pub contact_id: i32,

    /// Cache for contact recycling.
    pub cached_rotation_a: Rot,
    pub cached_rotation_b: Rot,
    pub cached_relative_pose: Transform,

    pub body_id_a: i32,
    pub body_id_b: i32,

    /// Transient body indices
    pub body_sim_index_a: i32,
    pub body_sim_index_b: i32,

    pub shape_id_a: i32,
    pub shape_id_b: i32,

    pub inv_mass_a: f32,
    pub inv_i_a: f32,

    pub inv_mass_b: f32,
    pub inv_i_b: f32,

    pub manifold: Manifold,

    /// Mixed friction and restitution
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub tangent_speed: f32,

    /// contact_flags bits (sim flags)
    pub sim_flags: u32,

    pub cache: SimplexCache,
}

impl Default for ContactSim {
    fn default() -> Self {
        ContactSim {
            contact_id: NULL_INDEX,
            cached_rotation_a: ROT_IDENTITY,
            cached_rotation_b: ROT_IDENTITY,
            cached_relative_pose: TRANSFORM_IDENTITY,
            body_id_a: NULL_INDEX,
            body_id_b: NULL_INDEX,
            body_sim_index_a: NULL_INDEX,
            body_sim_index_b: NULL_INDEX,
            shape_id_a: NULL_INDEX,
            shape_id_b: NULL_INDEX,
            inv_mass_a: 0.0,
            inv_i_a: 0.0,
            inv_mass_b: 0.0,
            inv_i_b: 0.0,
            manifold: Manifold::default(),
            friction: 0.0,
            restitution: 0.0,
            rolling_resistance: 0.0,
            tangent_speed: 0.0,
            sim_flags: 0,
            cache: SimplexCache::default(),
        }
    }
}
