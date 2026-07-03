// Port of box2d-cpp-reference/src/broad_phase.h and broad_phase.c: the
// b2BroadPhase storage, proxy operations, move buffering, overlap testing,
// and the pair update that drives contact creation.
//
// The single-threaded port turns the C parallel find-pairs task into a serial
// loop and the arena b2MoveResult/b2MovePair scratch (a prepended linked list
// per moved proxy) into a Vec of shape-id pairs per moved proxy. The C list
// is iterated head-first, i.e. reverse discovery order, so contact creation
// iterates each Vec in reverse to preserve the exact contact creation order.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::core::NULL_INDEX;
use crate::dynamic_tree::{DynamicTree, DEFAULT_MASK_BITS};
use crate::id::ShapeId;
use crate::math_functions::Aabb;
use crate::table::{shape_pair_key, HashSet};
use crate::types::{BodyType, Capacity, BODY_TYPE_COUNT};
use crate::world::World;

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

    /// (static inline b2UnBufferMove)
    fn unbuffer_move(&mut self, proxy_key: i32) {
        let proxy_type_ = proxy_type(proxy_key);
        let proxy_id_ = proxy_id(proxy_key);
        let set = &mut self.moved_proxies[proxy_type_ as usize];

        if set.get_bit(proxy_id_ as u32) {
            set.clear_bit(proxy_id_ as u32);

            // Purge from move buffer. Linear search.
            if let Some(index) = self.move_array.iter().position(|&k| k == proxy_key) {
                // C: b2Array_RemoveSwap
                self.move_array.swap_remove(index);
            }
        }
    }

    /// (b2BroadPhase_DestroyProxy)
    pub fn destroy_proxy(&mut self, proxy_key: i32) {
        self.unbuffer_move(proxy_key);

        let proxy_type_ = proxy_type(proxy_key);
        let proxy_id_ = proxy_id(proxy_key);

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

    /// (b2ValidateBroadphase)
    pub fn validate(&self) {
        self.trees[BodyType::Dynamic as usize].validate();
        self.trees[BodyType::Kinematic as usize].validate();
    }

    /// (b2ValidateNoEnlarged — C compiles the body under B2_ENABLE_VALIDATION;
    /// here the check runs in debug builds only)
    pub fn validate_no_enlarged(&self) {
        if cfg!(debug_assertions) {
            for tree in &self.trees {
                tree.validate_no_enlarged();
            }
        }
    }

    /// (b2ValidateMovedProxies — C compiles the body under
    /// B2_ENABLE_VALIDATION; here the whole check runs in debug builds only)
    pub fn validate_moved_proxies(&self) {
        if cfg!(debug_assertions) {
            // Invariant: bit set in movedProxies[type] iff proxyKey is present
            // in moveArray.
            for &proxy_key_ in &self.move_array {
                let proxy_type_ = proxy_type(proxy_key_);
                let proxy_id_ = proxy_id(proxy_key_);
                debug_assert!(self.moved_proxies[proxy_type_ as usize].get_bit(proxy_id_ as u32));
            }

            let mut total_set_bits = 0;
            for i in 0..BODY_TYPE_COUNT {
                total_set_bits += self.moved_proxies[i].count_set_bits();
            }
            debug_assert!(total_set_bits == self.move_array.len() as i32);
        }
    }
}

/// Query one tree for new pairs against a moved proxy, appending them to
/// `pair_list` in discovery order. (b2PairQueryCallback — the C callback
/// context struct becomes closure captures)
fn query_tree_for_pairs(
    world: &World,
    tree_type: BodyType,
    query_proxy_key: i32,
    query_shape_index: i32,
    fat_aabb: Aabb,
    pair_list: &mut Vec<(i32, i32)>,
) {
    let bp = &world.broad_phase;
    let query_proxy_type = proxy_type(query_proxy_key);

    bp.trees[tree_type as usize].query(fat_aabb, DEFAULT_MASK_BITS, |tree_proxy_id, user_data| {
        let shape_id = user_data as i32;
        let proxy_key_ = proxy_key(tree_proxy_id, tree_type);

        // A proxy cannot form a pair with itself.
        if proxy_key_ == query_proxy_key {
            return true;
        }

        // De-duplication
        // It is important to prevent duplicate contacts from being created.
        // Ideally I can prevent duplicates early and in the worker. Most of
        // the time the movedProxies bit sets contain dynamic and kinematic
        // proxies, but sometimes static proxies are in there too
        // (b2ShapeDef::invokeContactCreation or a modified static shape), so
        // we always have to check.

        // Is this proxy also moving?
        if query_proxy_type == BodyType::Dynamic {
            if tree_type == BodyType::Dynamic && proxy_key_ < query_proxy_key {
                let moved = bp.moved_proxies[tree_type as usize].get_bit(tree_proxy_id as u32);
                if moved {
                    // Both proxies are moving. Avoid duplicate pairs.
                    return true;
                }
            }
        } else {
            debug_assert!(tree_type == BodyType::Dynamic);
            let moved = bp.moved_proxies[tree_type as usize].get_bit(tree_proxy_id as u32);
            if moved {
                // Both proxies are moving. Avoid duplicate pairs.
                return true;
            }
        }

        let pair_key = shape_pair_key(shape_id, query_shape_index);
        if bp.pair_set.contains_key(pair_key) {
            // contact exists
            return true;
        }

        let (shape_id_a, shape_id_b) = if proxy_key_ < query_proxy_key {
            (shape_id, query_shape_index)
        } else {
            (query_shape_index, shape_id)
        };

        let shape_a = &world.shapes[shape_id_a as usize];
        let shape_b = &world.shapes[shape_id_b as usize];

        let body_id_a = shape_a.body_id;
        let body_id_b = shape_b.body_id;

        // Are the shapes on the same body?
        if body_id_a == body_id_b {
            return true;
        }

        // Sensors are handled elsewhere
        if shape_a.sensor_index != NULL_INDEX || shape_b.sensor_index != NULL_INDEX {
            return true;
        }

        if !crate::shape::should_shapes_collide(shape_a.filter, shape_b.filter) {
            return true;
        }

        if !crate::contact::can_collide(shape_a.shape_type(), shape_b.shape_type()) {
            // For example, no segment vs segment collision
            return true;
        }

        // Does a joint override collision?
        if !crate::body::should_bodies_collide(world, body_id_a, body_id_b) {
            return true;
        }

        // Custom user filter
        if shape_a.enable_custom_filtering || shape_b.enable_custom_filtering {
            if let Some(custom_filter_fcn) = world.custom_filter_fcn {
                let id_a = ShapeId {
                    index1: shape_id_a + 1,
                    world0: world.world_id,
                    generation: shape_a.generation,
                };
                let id_b = ShapeId {
                    index1: shape_id_b + 1,
                    world0: world.world_id,
                    generation: shape_b.generation,
                };
                let should_collide = custom_filter_fcn(id_a, id_b, world.custom_filter_context);
                if !should_collide {
                    return true;
                }
            }
        }

        pair_list.push((shape_id_a, shape_id_b));

        // continue the query
        true
    });
}

/// Find new proxy pairs for everything in the move buffer and create their
/// contacts, in deterministic move-array order. Also rebuilds the dynamic and
/// kinematic trees and clears the move buffer. (b2UpdateBroadPhasePairs —
/// serial port of b2FindPairsTask/b2UpdateTreesTask)
pub fn update_broad_phase_pairs(world: &mut World) {
    world.broad_phase.validate_moved_proxies();

    let move_count = world.broad_phase.move_array.len();

    if move_count == 0 {
        return;
    }

    // Find pairs. (b2FindPairsTask over [0, moveCount) on one worker)
    let mut move_results: Vec<Vec<(i32, i32)>> = Vec::with_capacity(move_count);
    for i in 0..move_count {
        let mut pair_list: Vec<(i32, i32)> = Vec::new();

        let proxy_key_ = world.broad_phase.move_array[i];
        if proxy_key_ == NULL_INDEX {
            // proxy was destroyed after it moved
            move_results.push(pair_list);
            continue;
        }

        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);

        // We have to query the tree with the fat AABB so that
        // we don't fail to create a contact that may touch later.
        let base_tree = &world.broad_phase.trees[proxy_type_ as usize];
        let fat_aabb = base_tree.aabb(proxy_id_);
        let query_shape_index = base_tree.user_data(proxy_id_) as i32;

        // Query trees. Only dynamic proxies collide with kinematic and static
        // proxies. Using B2_DEFAULT_MASK_BITS so that b2Filter::groupIndex
        // works.
        if proxy_type_ == BodyType::Dynamic {
            query_tree_for_pairs(
                world,
                BodyType::Kinematic,
                proxy_key_,
                query_shape_index,
                fat_aabb,
                &mut pair_list,
            );
            query_tree_for_pairs(
                world,
                BodyType::Static,
                proxy_key_,
                query_shape_index,
                fat_aabb,
                &mut pair_list,
            );
        }

        // All proxies collide with dynamic proxies
        query_tree_for_pairs(
            world,
            BodyType::Dynamic,
            proxy_key_,
            query_shape_index,
            fat_aabb,
            &mut pair_list,
        );

        move_results.push(pair_list);
    }

    // Rebuild the collision tree for dynamic and kinematic bodies to keep
    // their query performance good. In C this runs as a task in parallel with
    // the narrow-phase; the serial fallback runs it here. (b2UpdateTreesTask)
    world.broad_phase.trees[BodyType::Dynamic as usize].rebuild(false);
    world.broad_phase.trees[BodyType::Kinematic as usize].rebuild(false);

    // Single-threaded work
    // - Create contacts in deterministic order
    // This is deterministic because the results follow the order of
    // b2BroadPhase::moveArray. Each C pair list is iterated head-first, which
    // is reverse discovery order, hence .rev().
    for pair_list in &move_results {
        for &(shape_id_a, shape_id_b) in pair_list.iter().rev() {
            crate::contact::create_contact(world, shape_id_a, shape_id_b);
        }
    }

    // Reset move buffer: clear only the bits that were set this step.
    // Invariant: bit set in movedProxies[type] iff proxyKey is present in
    // moveArray.
    for i in 0..world.broad_phase.move_array.len() {
        let proxy_key_ = world.broad_phase.move_array[i];
        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);
        world.broad_phase.moved_proxies[proxy_type_ as usize].clear_bit(proxy_id_ as u32);
    }
    world.broad_phase.move_array.clear();

    world.validate_solver_sets();
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
