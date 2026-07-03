// Port of box2d-cpp-reference/src/broad_phase.h and the world-independent
// parts of broad_phase.c: the b2BroadPhase storage, proxy operations, move
// buffering, and overlap testing.
//
// b2UpdateBroadPhasePairs takes b2World* and drives contact creation through
// shape/contact/arena/parallel_for; it lands with the contact bring-up commit.
// The transient pair-query scratch (b2MoveResult*/b2MovePair* into arena
// memory) is owned by that phase too.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::dynamic_tree::DynamicTree;
use crate::math_functions::Aabb;
use crate::table::HashSet;
use crate::types::{BodyType, Capacity, BODY_TYPE_COUNT};

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30
// bits for the id.

/// (B2_PROXY_TYPE)
pub fn proxy_type(key: i32) -> BodyType {
    match key & 3 {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        _ => BodyType::Dynamic,
    }
}

/// (B2_PROXY_ID)
pub fn proxy_id(key: i32) -> i32 {
    key >> 2
}

/// (B2_PROXY_KEY)
pub fn proxy_key(id: i32, type_: BodyType) -> i32 {
    (id << 2) | (type_ as i32)
}

/// The broad-phase is used for computing pairs and performing volume queries
/// and ray casts. It does not persist pairs; it reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent
/// overlap. (b2BroadPhase)
#[derive(Debug)]
pub struct BroadPhase {
    pub trees: [DynamicTree; BODY_TYPE_COUNT],

    /// Per body-type bit sets indexed by proxyId, marking proxies moved this
    /// step. Paired with move_array which preserves deterministic insertion
    /// order for pair queries.
    pub moved_proxies: [BitSet; BODY_TYPE_COUNT],
    pub move_array: Vec<i32>,

    /// Tracks shape pairs that have a Contact.
    pub pair_set: HashSet,
}

impl BroadPhase {
    /// (b2CreateBroadPhase)
    pub fn new(capacity: &Capacity) -> BroadPhase {
        // The C code sizes the static tree by staticShapeCount and the
        // kinematic/dynamic trees by dynamicShapeCount, and the pair set by
        // contactCount (16 minimum inside the containers).
        BroadPhase {
            trees: [
                DynamicTree::new(capacity.static_shape_count),
                DynamicTree::new(capacity.dynamic_shape_count),
                DynamicTree::new(capacity.dynamic_shape_count),
            ],
            moved_proxies: [BitSet::new(16), BitSet::new(16), BitSet::new(16)],
            move_array: Vec::new(),
            pair_set: HashSet::new(crate::math_functions::max_int(16, capacity.contact_count)),
        }
    }

    /// (b2DestroyBroadPhase)
    pub fn destroy(&mut self) {
        *self = BroadPhase {
            trees: [
                DynamicTree::new(0),
                DynamicTree::new(0),
                DynamicTree::new(0),
            ],
            moved_proxies: Default::default(),
            move_array: Vec::new(),
            pair_set: HashSet::new(16),
        };
    }

    /// This triggers new contact pairs to be created. Must be called in
    /// deterministic order. (static inline b2BufferMove)
    pub fn buffer_move(&mut self, query_proxy: i32) {
        let proxy_type_ = proxy_type(query_proxy);
        let proxy_id_ = proxy_id(query_proxy);
        let set = &mut self.moved_proxies[proxy_type_ as usize];
        if !set.get_bit(proxy_id_ as u32) {
            set.set_bit_grow(proxy_id_ as u32);
            self.move_array.push(query_proxy);
        }
    }

    /// (b2BroadPhase_CreateProxy)
    pub fn create_proxy(
        &mut self,
        proxy_type_: BodyType,
        aabb: Aabb,
        category_bits: u64,
        shape_index: i32,
        force_pair_creation: bool,
    ) -> i32 {
        let proxy_id_ =
            self.trees[proxy_type_ as usize].create_proxy(aabb, category_bits, shape_index as u64);
        let proxy_key_ = proxy_key(proxy_id_, proxy_type_);
        if proxy_type_ != BodyType::Static || force_pair_creation {
            self.buffer_move(proxy_key_);
        }
        proxy_key_
    }

    /// (b2BroadPhase_DestroyProxy)
    pub fn destroy_proxy(&mut self, proxy_key: i32) {
        debug_assert!(!self.move_array.is_empty());

        // Purge from move buffer. Linear search.
        // This is done for clarity/simplicity to match the C implementation:
        // the moved bit alone is not enough because the moveArray preserves
        // deterministic order.
        if let Some(index) = self.move_array.iter().position(|&k| k == proxy_key) {
            // C: b2Array_RemoveSwap
            self.move_array.swap_remove(index);
        }

        let proxy_type_ = proxy_type(proxy_key);
        let proxy_id_ = proxy_id(proxy_key);

        self.moved_proxies[proxy_type_ as usize].clear_bit(proxy_id_ as u32);

        self.trees[proxy_type_ as usize].destroy_proxy(proxy_id_);
    }

    /// (b2BroadPhase_MoveProxy)
    pub fn move_proxy(&mut self, proxy_key: i32, aabb: Aabb) {
        let proxy_type_ = proxy_type(proxy_key);
        let proxy_id_ = proxy_id(proxy_key);

        self.trees[proxy_type_ as usize].move_proxy(proxy_id_, aabb);
        self.buffer_move(proxy_key);
    }

    /// (b2BroadPhase_EnlargeProxy)
    pub fn enlarge_proxy(&mut self, proxy_key: i32, aabb: Aabb) {
        debug_assert!(proxy_key != crate::core::NULL_INDEX);
        let proxy_type_ = proxy_type(proxy_key);
        let proxy_id_ = proxy_id(proxy_key);

        // Static bodies do not have enlarged proxies
        debug_assert!(proxy_type_ != BodyType::Static);

        self.trees[proxy_type_ as usize].enlarge_proxy(proxy_id_, aabb);
        self.buffer_move(proxy_key);
    }

    /// (b2BroadPhase_GetShapeIndex)
    pub fn shape_index(&self, proxy_key: i32) -> i32 {
        let proxy_type_ = proxy_type(proxy_key);
        let proxy_id_ = proxy_id(proxy_key);

        self.trees[proxy_type_ as usize].user_data(proxy_id_) as i32
    }

    /// (b2BroadPhase_TestOverlap)
    pub fn test_overlap(&self, proxy_key_a: i32, proxy_key_b: i32) -> bool {
        let type_a = proxy_type(proxy_key_a);
        let id_a = proxy_id(proxy_key_a);
        let type_b = proxy_type(proxy_key_b);
        let id_b = proxy_id(proxy_key_b);

        let aabb_a = self.trees[type_a as usize].aabb(id_a);
        let aabb_b = self.trees[type_b as usize].aabb(id_b);
        crate::math_functions::aabb_overlaps(aabb_a, aabb_b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_functions::Vec2;

    fn aabb(lx: f32, ly: f32, ux: f32, uy: f32) -> Aabb {
        Aabb {
            lower_bound: Vec2 { x: lx, y: ly },
            upper_bound: Vec2 { x: ux, y: uy },
        }
    }

    #[test]
    fn proxy_key_round_trip() {
        for (id, t) in [
            (0, BodyType::Static),
            (7, BodyType::Kinematic),
            (123456, BodyType::Dynamic),
        ] {
            let key = proxy_key(id, t);
            assert_eq!(proxy_id(key), id);
            assert_eq!(proxy_type(key), t);
        }
    }

    #[test]
    fn create_move_destroy_and_overlap() {
        let mut bp = BroadPhase::new(&Capacity::default());

        // Static proxies don't buffer a move unless forced.
        let k_static = bp.create_proxy(BodyType::Static, aabb(0.0, 0.0, 1.0, 1.0), 1, 10, false);
        assert!(bp.move_array.is_empty());

        let k_dyn = bp.create_proxy(BodyType::Dynamic, aabb(0.5, 0.5, 1.5, 1.5), 1, 11, false);
        assert_eq!(bp.move_array.len(), 1);

        // Buffering the same proxy twice only records it once.
        bp.buffer_move(k_dyn);
        assert_eq!(bp.move_array.len(), 1);

        assert!(bp.test_overlap(k_static, k_dyn));
        assert_eq!(bp.shape_index(k_static), 10);
        assert_eq!(bp.shape_index(k_dyn), 11);

        // Move the dynamic proxy away; overlap ends.
        bp.move_proxy(k_dyn, aabb(10.0, 10.0, 11.0, 11.0));
        assert!(!bp.test_overlap(k_static, k_dyn));

        // Destroy removes from the move buffer and the tree.
        bp.destroy_proxy(k_dyn);
        assert!(bp.move_array.is_empty());
        assert_eq!(bp.trees[BodyType::Dynamic as usize].proxy_count(), 0);
    }
}
