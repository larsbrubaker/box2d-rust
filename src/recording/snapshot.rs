// World snapshot foundation from world_snapshot.c: the image header, the
// bounds-checked reader, and serialize/deserialize for the engine containers
// (id pools, bitsets, the pair hash set, and the broad-phase trees).
//
// Format note: the C snapshot memcpys internal structs and gates loading on a
// layout hash of their sizes — C refuses images from any other C build. The
// Rust port keeps that design (snapshots are build-specific) but writes
// fields explicitly in declaration order; the layout hash mixes Rust struct
// sizes, so Rust images are only accepted by a matching Rust build. The
// recording OP STREAM stays byte-compatible with C; the seed snapshot blob is
// inherently implementation-specific on both sides.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::write::{rec_w_i32, rec_w_u16, rec_w_u32, rec_w_u64};
use crate::bitset::BitSet;
use crate::dynamic_tree::{DynamicTree, TreeNode};
use crate::id_pool::IdPool;
use crate::math_functions::{Aabb, Vec2};
use crate::table::HashSet;

/// Snapshot image magic, 'BNS2'. (B2_SNAP_MAGIC)
pub const SNAP_MAGIC: u32 = 0x32534E42;

/// Bump this if any serialized data structure changes. (B2_SNAP_VERSION —
/// restarted at 1 for the Rust field-order format)
pub const SNAP_VERSION: u32 = 1;

/// Image was built with validation (debug assertions). (B2_SNAP_FLAG_VALIDATION)
pub const SNAP_FLAG_VALIDATION: u32 = 0x1;
/// Image was built with double-precision world positions.
/// (B2_SNAP_FLAG_DOUBLE_PRECISION)
pub const SNAP_FLAG_DOUBLE_PRECISION: u32 = 0x2;

/// Layout hash seeds from all serialized structs. Changing any struct size
/// updates the hash and refuses older images, same role as C's
/// b2ComputeLayoutHash over its memcpy'd structs.
pub fn compute_layout_hash() -> u32 {
    let mut h: u32 = 2166136261;
    let mut mix = |x: usize| {
        h ^= x as u32;
        h = h.wrapping_mul(16777619);
    };
    mix(std::mem::size_of::<crate::body::Body>());
    mix(std::mem::size_of::<crate::body::BodySim>());
    mix(std::mem::size_of::<crate::body::BodyState>());
    mix(std::mem::size_of::<crate::shape::Shape>());
    mix(std::mem::size_of::<crate::shape::ChainShape>());
    mix(std::mem::size_of::<crate::contact::Contact>());
    mix(std::mem::size_of::<crate::contact::ContactSim>());
    mix(std::mem::size_of::<crate::joint::Joint>());
    mix(std::mem::size_of::<crate::joint::JointSim>());
    mix(std::mem::size_of::<crate::island::Island>());
    mix(std::mem::size_of::<crate::island::IslandSim>());
    mix(std::mem::size_of::<TreeNode>());
    mix(std::mem::size_of::<crate::sensor::Sensor>());
    mix(crate::constants::GRAPH_COLOR_COUNT as usize);
    h
}

/// Snapshot image header. (b2SnapHeader)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SnapHeader {
    pub magic: u32,
    pub version: u32,
    pub layout_hash: u32,
    pub flags: u32,
}

impl SnapHeader {
    pub const SIZE: usize = 16;

    /// Header for an image produced by this build.
    pub fn current() -> SnapHeader {
        let mut flags = 0;
        if cfg!(debug_assertions) {
            flags |= SNAP_FLAG_VALIDATION;
        }
        if crate::core::is_double_precision() {
            flags |= SNAP_FLAG_DOUBLE_PRECISION;
        }
        SnapHeader {
            magic: SNAP_MAGIC,
            version: SNAP_VERSION,
            layout_hash: compute_layout_hash(),
            flags,
        }
    }

    pub fn write(&self, buf: &mut Vec<u8>) {
        rec_w_u32(buf, self.magic);
        rec_w_u32(buf, self.version);
        rec_w_u32(buf, self.layout_hash);
        rec_w_u32(buf, self.flags);
    }

    /// True when an image with this header can be restored by this build.
    /// The validation flag is diagnostic only, exactly like C.
    pub fn is_compatible(&self) -> bool {
        let current = SnapHeader::current();
        self.magic == SNAP_MAGIC
            && self.version == SNAP_VERSION
            && self.layout_hash == current.layout_hash
            && (self.flags & SNAP_FLAG_DOUBLE_PRECISION)
                == (current.flags & SNAP_FLAG_DOUBLE_PRECISION)
    }
}

/// Bounds-checked little-endian reader over a snapshot image. A short or
/// corrupt image trips `ok` instead of panicking, mirroring the C
/// b2SnapReader contract. (b2SnapReader)
#[derive(Debug)]
pub struct SnapReader<'a> {
    pub data: &'a [u8],
    pub cursor: usize,
    pub ok: bool,
}

impl<'a> SnapReader<'a> {
    pub fn new(data: &'a [u8]) -> SnapReader<'a> {
        SnapReader {
            data,
            cursor: 0,
            ok: true,
        }
    }

    fn take(&mut self, n: usize) -> &'a [u8] {
        if !self.ok || self.cursor + n > self.data.len() {
            self.ok = false;
            return &[];
        }
        let slice = &self.data[self.cursor..self.cursor + n];
        self.cursor += n;
        slice
    }

    pub fn r_u16(&mut self) -> u16 {
        let b = self.take(2);
        if b.len() < 2 {
            return 0;
        }
        u16::from_le_bytes([b[0], b[1]])
    }

    pub fn r_u32(&mut self) -> u32 {
        let b = self.take(4);
        if b.len() < 4 {
            return 0;
        }
        u32::from_le_bytes([b[0], b[1], b[2], b[3]])
    }

    pub fn r_i32(&mut self) -> i32 {
        self.r_u32() as i32
    }

    pub fn r_u64(&mut self) -> u64 {
        let b = self.take(8);
        if b.len() < 8 {
            return 0;
        }
        u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
    }

    pub fn r_f32(&mut self) -> f32 {
        f32::from_bits(self.r_u32())
    }

    pub fn r_f64(&mut self) -> f64 {
        f64::from_bits(self.r_u64())
    }

    pub fn r_bool(&mut self) -> bool {
        let b = self.take(1);
        !b.is_empty() && b[0] != 0
    }

    pub fn r_u8(&mut self) -> u8 {
        let b = self.take(1);
        if b.is_empty() {
            0
        } else {
            b[0]
        }
    }

    /// Guard an element count against the remaining stream so a corrupt
    /// count cannot force a huge allocation. (b2SnapCheckCount)
    pub fn check_count(&mut self, count: i32, min_stream_bytes: usize) -> bool {
        if count < 0
            || (count as usize) * min_stream_bytes
                > self.data.len() - self.cursor.min(self.data.len())
        {
            self.ok = false;
            return false;
        }
        true
    }

    pub fn r_header(&mut self) -> SnapHeader {
        SnapHeader {
            magic: self.r_u32(),
            version: self.r_u32(),
            layout_hash: self.r_u32(),
            flags: self.r_u32(),
        }
    }
}

// Container serialize/deserialize. (b2SerIdPool/b2DesIdPool, b2SerBitSet/
// b2DesBitSet, b2SerHashSet/b2DesHashSet, b2SerTree/b2DesTree)

pub(crate) fn ser_id_pool(buf: &mut Vec<u8>, pool: &IdPool) {
    rec_w_i32(buf, pool.free_array.len() as i32);
    for &id in pool.free_array.iter() {
        rec_w_i32(buf, id);
    }
    rec_w_i32(buf, pool.next_index);
}

pub(crate) fn des_id_pool(r: &mut SnapReader) -> IdPool {
    let count = r.r_i32();
    if !r.check_count(count, 4) {
        return IdPool::new();
    }
    let mut pool = IdPool::new();
    pool.free_array = (0..count).map(|_| r.r_i32()).collect();
    pool.next_index = r.r_i32();
    pool
}

pub(crate) fn ser_bitset(buf: &mut Vec<u8>, bs: &BitSet) {
    rec_w_u32(buf, bs.block_count);
    rec_w_i32(buf, bs.blocks.len() as i32);
    for &block in bs.blocks.iter() {
        rec_w_u64(buf, block);
    }
}

pub(crate) fn des_bitset(r: &mut SnapReader) -> BitSet {
    let block_count = r.r_u32();
    let capacity = r.r_i32();
    if !r.check_count(capacity, 8) {
        return BitSet::new(0);
    }
    let mut bs = BitSet::new(0);
    bs.block_count = block_count;
    bs.blocks = (0..capacity).map(|_| r.r_u64()).collect();
    bs
}

pub(crate) fn ser_hashset(buf: &mut Vec<u8>, hs: &HashSet) {
    rec_w_u32(buf, hs.count);
    rec_w_i32(buf, hs.items.len() as i32);
    for &item in hs.items.iter() {
        rec_w_u64(buf, item);
    }
}

pub(crate) fn des_hashset(r: &mut SnapReader) -> HashSet {
    let count = r.r_u32();
    let capacity = r.r_i32();
    if !r.check_count(capacity, 8) {
        return HashSet::new(16);
    }
    let mut hs = HashSet::new(16);
    hs.count = count;
    hs.items = (0..capacity).map(|_| r.r_u64()).collect();
    hs
}

fn ser_aabb(buf: &mut Vec<u8>, aabb: Aabb) {
    super::write::rec_w_f32(buf, aabb.lower_bound.x);
    super::write::rec_w_f32(buf, aabb.lower_bound.y);
    super::write::rec_w_f32(buf, aabb.upper_bound.x);
    super::write::rec_w_f32(buf, aabb.upper_bound.y);
}

fn des_aabb(r: &mut SnapReader) -> Aabb {
    Aabb {
        lower_bound: Vec2 {
            x: r.r_f32(),
            y: r.r_f32(),
        },
        upper_bound: Vec2 {
            x: r.r_f32(),
            y: r.r_f32(),
        },
    }
}

pub(crate) fn ser_tree(buf: &mut Vec<u8>, tree: &DynamicTree) {
    rec_w_i32(buf, tree.root);
    rec_w_i32(buf, tree.node_count);
    rec_w_i32(buf, tree.free_list);
    rec_w_i32(buf, tree.proxy_count);
    rec_w_i32(buf, tree.nodes.len() as i32);
    for node in tree.nodes.iter() {
        ser_aabb(buf, node.aabb);
        rec_w_u64(buf, node.category_bits);
        rec_w_i32(buf, node.child1);
        rec_w_i32(buf, node.child2);
        rec_w_u64(buf, node.user_data);
        rec_w_i32(buf, node.parent);
        rec_w_i32(buf, node.next);
        rec_w_u16(buf, node.height);
        rec_w_u16(buf, node.flags);
    }
    // Rebuild scratch (leaf_indices/leaf_centers/rebuild_capacity) is
    // per-step transient and rebuilt on demand; C serializes the node pool
    // and scalars the same way via memcpy.
}

pub(crate) fn des_tree(r: &mut SnapReader) -> DynamicTree {
    let mut tree = DynamicTree::new(0);
    tree.root = r.r_i32();
    tree.node_count = r.r_i32();
    tree.free_list = r.r_i32();
    tree.proxy_count = r.r_i32();
    let capacity = r.r_i32();
    // Each node is at least 48 bytes on the wire.
    if !r.check_count(capacity, 48) {
        return DynamicTree::new(0);
    }
    tree.nodes = (0..capacity)
        .map(|_| {
            let mut node = TreeNode::default_node();
            node.aabb = des_aabb(r);
            node.category_bits = r.r_u64();
            node.child1 = r.r_i32();
            node.child2 = r.r_i32();
            node.user_data = r.r_u64();
            node.parent = r.r_i32();
            node.next = r.r_i32();
            node.height = r.r_u16();
            node.flags = r.r_u16();
            node
        })
        .collect();
    tree
}

#[cfg(test)]
mod tests {
    use super::*;

    // Every container round trips exactly, and the header refuses foreign
    // images.
    #[test]
    fn container_round_trips() {
        // Id pool with a used range and free holes.
        let mut pool = IdPool::new();
        for _ in 0..6 {
            pool.alloc_id();
        }
        pool.free_id(2);
        pool.free_id(4);
        let mut buf = Vec::new();
        ser_id_pool(&mut buf, &pool);
        let mut r = SnapReader::new(&buf);
        let restored = des_id_pool(&mut r);
        assert!(r.ok);
        assert_eq!(restored.id_count(), pool.id_count());
        assert_eq!(restored.id_capacity(), pool.id_capacity());

        // Bit set.
        let mut bs = BitSet::new(200);
        bs.set_bit_count_and_clear(200);
        bs.set_bit(3);
        bs.set_bit(130);
        let mut buf = Vec::new();
        ser_bitset(&mut buf, &bs);
        let mut r = SnapReader::new(&buf);
        let restored = des_bitset(&mut r);
        assert!(r.ok);
        assert!(restored.get_bit(3) && restored.get_bit(130) && !restored.get_bit(64));

        // Pair hash set.
        let mut hs = HashSet::new(16);
        hs.add_key(crate::table::shape_pair_key(3, 9));
        hs.add_key(crate::table::shape_pair_key(1, 2));
        let mut buf = Vec::new();
        ser_hashset(&mut buf, &hs);
        let mut r = SnapReader::new(&buf);
        let restored = des_hashset(&mut r);
        assert!(r.ok);
        assert!(restored.contains_key(crate::table::shape_pair_key(3, 9)));
        assert!(!restored.contains_key(crate::table::shape_pair_key(3, 8)));

        // Dynamic tree with a few proxies.
        let mut tree = DynamicTree::new(4);
        for i in 0..5 {
            let x = i as f32;
            tree.create_proxy(
                Aabb {
                    lower_bound: Vec2 { x, y: 0.0 },
                    upper_bound: Vec2 { x: x + 1.0, y: 1.0 },
                },
                1,
                i as u64,
            );
        }
        let mut buf = Vec::new();
        ser_tree(&mut buf, &tree);
        let mut r = SnapReader::new(&buf);
        let restored = des_tree(&mut r);
        assert!(r.ok);
        assert_eq!(restored.proxy_count(), 5);
        assert_eq!(restored.height(), tree.height());
        restored.validate();

        // A truncated image trips ok instead of panicking.
        let mut short = SnapReader::new(&buf[..10]);
        let _ = des_tree(&mut short);
        assert!(!short.ok);
    }

    #[test]
    fn header_compatibility() {
        let header = SnapHeader::current();
        let mut buf = Vec::new();
        header.write(&mut buf);
        assert_eq!(buf.len(), SnapHeader::SIZE);

        let mut r = SnapReader::new(&buf);
        let read = r.r_header();
        assert!(r.ok);
        assert_eq!(read, header);
        assert!(read.is_compatible());

        // Wrong layout hash refuses to load.
        let foreign = SnapHeader {
            layout_hash: header.layout_hash ^ 1,
            ..header
        };
        assert!(!foreign.is_compatible());
    }
}
