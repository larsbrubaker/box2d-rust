// Port of contact.h (data model) and contact.c (contact lifecycle + narrow
// phase update). Split: lifecycle.rs holds the register table and contact
// creation/destruction; update.rs holds the narrow-phase manifold update.
//
// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in
// b2Island no matter the thread count. The order must reproduce from run to
// run. This is necessary because the Gauss-Seidel constraint solver is order
// dependent.
//
// Creation:
// - Contacts are created using results from b2UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move
//   array
// - The move array is ordered according to the shape creation order using a
//   bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are
//   immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake
//   bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state
//   changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all
//   contacts
// - As long as contacts are created in deterministic order, island link order
//   is deterministic.
// - This keeps the order of contacts in islands deterministic
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::Manifold;
use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::math_functions::{Rot, Transform, ROT_IDENTITY, TRANSFORM_IDENTITY};

mod api;
mod lifecycle;
mod update;

pub use api::*;
pub use lifecycle::*;
pub use update::*;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{create_body, get_body_full_id, get_body_transform};
    use crate::broad_phase::update_broad_phase_pairs;
    use crate::collision::ShapeType;
    use crate::geometry::make_box;
    use crate::math_functions::VEC2_ZERO;
    use crate::shape::create_polygon_shape;
    use crate::solver_set::AWAKE_SET;
    use crate::table::shape_pair_key;
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::World;

    #[test]
    fn contact_register_pairs() {
        use ShapeType::*;
        // All 12 registered pairs collide, in either order.
        for (a, b) in [
            (Circle, Circle),
            (Capsule, Circle),
            (Capsule, Capsule),
            (Polygon, Circle),
            (Polygon, Capsule),
            (Polygon, Polygon),
            (Segment, Circle),
            (Segment, Capsule),
            (Segment, Polygon),
            (ChainSegment, Circle),
            (ChainSegment, Capsule),
            (ChainSegment, Polygon),
        ] {
            assert!(can_collide(a, b), "{a:?} vs {b:?}");
            assert!(can_collide(b, a), "{b:?} vs {a:?}");
        }

        // No segment vs segment style collisions.
        assert!(!can_collide(Segment, Segment));
        assert!(!can_collide(Segment, ChainSegment));
        assert!(!can_collide(ChainSegment, ChainSegment));
    }

    // Slice-4 acceptance test: two dynamic bodies with overlapping box shapes.
    // update_broad_phase_pairs must create exactly one contact (dedup: both
    // proxies are in the move array) and update_contact must report touching
    // with a two point manifold.
    #[test]
    fn overlapping_boxes_create_touching_contact() {
        let mut world = World::new(&default_world_def());

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let body_a = create_body(&mut world, &body_def);
        let body_b = create_body(&mut world, &body_def);
        let body_index_a = get_body_full_id(&world, body_a);
        let body_index_b = get_body_full_id(&world, body_b);

        // Identical overlapping boxes at the origin.
        let box_poly = make_box(0.5, 0.5);
        let shape_def = default_shape_def();
        let sa = create_polygon_shape(&mut world, body_a, &shape_def, &box_poly);
        let sb = create_polygon_shape(&mut world, body_b, &shape_def, &box_poly);
        let shape_index_a = sa.index1 - 1;
        let shape_index_b = sb.index1 - 1;

        assert_eq!(world.broad_phase.move_array.len(), 2);

        // Broad-phase pair update creates exactly one non-touching contact in
        // the awake set and consumes the move buffer.
        update_broad_phase_pairs(&mut world);
        assert_eq!(world.contact_id_pool.id_count(), 1);
        assert_eq!(world.solver_sets[AWAKE_SET as usize].contact_sims.len(), 1);
        assert!(world.broad_phase.move_array.is_empty());
        assert!(world
            .broad_phase
            .pair_set
            .contains_key(shape_pair_key(shape_index_a, shape_index_b)));

        let contact_id = world.solver_sets[AWAKE_SET as usize].contact_sims[0].contact_id;
        {
            let contact = &world.contacts[contact_id as usize];
            assert_eq!(contact.set_index, AWAKE_SET);
            assert_eq!(contact.color_index, NULL_INDEX);
            assert_eq!(contact.flags & contact_flags::TOUCHING, 0);
            assert_eq!(contact.edges[0].body_id, body_index_a);
            assert_eq!(contact.edges[1].body_id, body_index_b);
        }
        // Contact edges are linked into both bodies.
        assert_eq!(world.bodies[body_index_a as usize].contact_count, 1);
        assert_eq!(world.bodies[body_index_b as usize].contact_count, 1);
        assert_eq!(
            world.bodies[body_index_a as usize].head_contact_key,
            contact_id << 1
        );
        assert_eq!(
            world.bodies[body_index_b as usize].head_contact_key,
            (contact_id << 1) | 1
        );

        // A second pair update is a no-op (move buffer empty, pair exists).
        update_broad_phase_pairs(&mut world);
        assert_eq!(world.contact_id_pool.id_count(), 1);

        // Narrow phase: the manifold has two points and the contact touches.
        let (shape_id_a, shape_id_b) = {
            let contact = &world.contacts[contact_id as usize];
            (contact.shape_id_a, contact.shape_id_b)
        };
        let ctx = ContactUpdateContext::new(&world);
        let transform_a = get_body_transform(&world, body_index_a);
        let transform_b = get_body_transform(&world, body_index_b);
        let shape_a = world.shapes[shape_id_a as usize].clone();
        let shape_b = world.shapes[shape_id_b as usize].clone();

        let mut contact_sim = world.solver_sets[AWAKE_SET as usize].contact_sims[0];
        let touching = update_contact(
            &ctx,
            &mut contact_sim,
            &shape_a,
            transform_a,
            VEC2_ZERO,
            &shape_b,
            transform_b,
            VEC2_ZERO,
        );
        assert!(touching);
        assert_eq!(contact_sim.manifold.point_count, 2);
        assert!(contact_sim.sim_flags & contact_flags::SIM_TOUCHING != 0);
        world.solver_sets[AWAKE_SET as usize].contact_sims[0] = contact_sim;

        // Destroying the contact unlinks the bodies and frees the pair.
        destroy_contact(&mut world, contact_id, false);
        assert_eq!(world.contact_id_pool.id_count(), 0);
        assert!(world.solver_sets[AWAKE_SET as usize]
            .contact_sims
            .is_empty());
        assert!(!world
            .broad_phase
            .pair_set
            .contains_key(shape_pair_key(shape_index_a, shape_index_b)));
        assert_eq!(world.bodies[body_index_a as usize].contact_count, 0);
        assert_eq!(world.bodies[body_index_b as usize].contact_count, 0);
        assert_eq!(
            world.bodies[body_index_a as usize].head_contact_key,
            NULL_INDEX
        );

        world.validate_solver_sets();
    }
}
