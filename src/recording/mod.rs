// Recording subsystem foundation from recording.h + recording.c: the wire
// header, the append-only record buffer, record framing, and the world-state
// hash shared by recorder and replayer.
//
// Porting decisions:
// - b2RecBuffer becomes a plain Vec<u8>; the C countOnly sizing mode is an
//   allocation optimization with no wire effect and is not ported.
// - The recording mutex is dropped: the port is serial, so query records
//   can never interleave.
// - The C X-macro codegen over recording_ops.inl becomes explicit Rust in
//   write.rs (writers) and the replay dispatcher (later slice); opcodes and
//   field order are kept byte-identical.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#![allow(dead_code)] // bring-up: replay/snapshot slices land next; nothing calls this yet

use crate::math_functions::{Aabb, Pos};
use crate::world::World;

mod ops;
mod ops_body;
mod ops_joint;
mod ops_query;
mod ops_shape;
mod snapshot;
mod snapshot_joints;
mod snapshot_structs;
mod snapshot_world;
mod write;

pub use ops::*;
pub use ops_body::*;
pub use ops_joint::*;
pub use ops_query::*;
pub use ops_shape::*;
pub use snapshot::*;
pub use snapshot_world::*;
pub use write::*;

/// FNV-1a 64-bit initial value. (B2_SNAP_FNV_INIT)
pub const SNAP_FNV_INIT: u64 = 14695981039346656037;

/// FNV-1a 64-bit prime. (B2_SNAP_FNV_PRIME)
pub const SNAP_FNV_PRIME: u64 = 1099511628211;

/// Magic value 'B2RC' in little-endian. (B2_REC_MAGIC)
pub const REC_MAGIC: u32 = 0x43523242;

/// Recording format version. Any mismatch refuses to load. The minor tracks
/// op stream layout changes that keep the 32 byte header shape.
/// (B2_REC_VERSION_MAJOR/MINOR)
pub const REC_VERSION_MAJOR: u16 = 3;
pub const REC_VERSION_MINOR: u16 = 2;

/// Mix a world position at full width, or the determinism gates would
/// validate only to float precision and pass vacuously far from the origin.
/// (b2FnvMixPosition)
pub fn fnv_mix_position(mut hash: u64, p: Pos) -> u64 {
    // In the single-precision build Pos components are f32 and widen to u64
    // through u32 bits, matching the C #else branch.
    let (bx, by) = pos_bits(p);
    hash = (hash ^ bx).wrapping_mul(SNAP_FNV_PRIME);
    hash = (hash ^ by).wrapping_mul(SNAP_FNV_PRIME);
    hash
}

#[cfg(feature = "double-precision")]
fn pos_bits(p: Pos) -> (u64, u64) {
    (p.x.to_bits(), p.y.to_bits())
}

#[cfg(not(feature = "double-precision"))]
fn pos_bits(p: Pos) -> (u64, u64) {
    (p.x.to_bits() as u64, p.y.to_bits() as u64)
}

/// File header, fixed 32 bytes, little-endian. (b2RecHeader)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RecHeader {
    /// 'B2RC' = 0x43523242
    pub magic: u32,
    pub version_major: u16,
    pub version_minor: u16,
    /// The world length scale
    pub length_scale: f32,
    /// sizeof(void*) in C, gates POD-def memcpy; always 8 here
    pub pointer_width: u8,
    /// 0 on all supported targets
    pub big_endian: u8,
    /// 1 if built with validation, only for diagnostics on a layout mismatch
    pub validation_enabled: u8,
    /// bytes of snapshot blob after the header
    pub snapshot_size: u64,
}

impl RecHeader {
    pub const SIZE: usize = 32;

    /// Serialize in the exact C struct layout (little-endian, with the
    /// reserved fields zeroed).
    pub fn write(&self, buf: &mut Vec<u8>) {
        rec_w_u32(buf, self.magic);
        rec_w_u16(buf, self.version_major);
        rec_w_u16(buf, self.version_minor);
        rec_w_u32(buf, 0); // reserved2
        rec_w_f32(buf, self.length_scale);
        rec_w_u8(buf, 0); // reserved3
        rec_w_u8(buf, self.pointer_width);
        rec_w_u8(buf, self.big_endian);
        rec_w_u8(buf, self.validation_enabled);
        rec_w_u32(buf, 0); // reserved1
        rec_w_u64(buf, self.snapshot_size);
    }

    /// Parse a 32-byte header. Returns None if the data is too short.
    pub fn read(data: &[u8]) -> Option<RecHeader> {
        if data.len() < Self::SIZE {
            return None;
        }
        let u16_at = |o: usize| u16::from_le_bytes([data[o], data[o + 1]]);
        let u32_at =
            |o: usize| u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let u64_at = |o: usize| {
            u64::from_le_bytes([
                data[o],
                data[o + 1],
                data[o + 2],
                data[o + 3],
                data[o + 4],
                data[o + 5],
                data[o + 6],
                data[o + 7],
            ])
        };
        Some(RecHeader {
            magic: u32_at(0),
            version_major: u16_at(4),
            version_minor: u16_at(6),
            length_scale: f32::from_bits(u32_at(12)),
            pointer_width: data[17],
            big_endian: data[18],
            validation_enabled: data[19],
            snapshot_size: u64_at(24),
        })
    }
}

/// User-owned recording buffer. The world appends into it while recording;
/// the user saves and destroys it. (b2Recording — the mutex is dropped in
/// the serial port)
#[derive(Debug, Default)]
pub struct Recording {
    pub buffer: Vec<u8>,

    /// Offset of the 3-byte size field for u24 backpatch.
    record_start: usize,

    /// Union of world bounds over every recorded step, written out at stop so
    /// a replay can frame the whole motion. have_bounds gates the first union
    /// the same way world_get_bounds does.
    pub accumulated_bounds: Aabb,
    pub have_bounds: bool,
}

impl Recording {
    /// (b2CreateRecording — the capacity hint sizes the buffer up front)
    pub fn new(capacity_hint: usize) -> Recording {
        Recording {
            buffer: Vec::with_capacity(capacity_hint),
            record_start: 0,
            accumulated_bounds: Aabb::default(),
            have_bounds: false,
        }
    }

    /// Start a framed record: opcode byte plus a 3-byte payload-size slot
    /// backpatched by [`Recording::end_record`]. (b2RecBeginRecord)
    pub fn begin_record(&mut self, opcode: u8) {
        rec_w_u8(&mut self.buffer, opcode);
        self.record_start = self.buffer.len();
        self.buffer.extend_from_slice(&[0, 0, 0]);
    }

    /// Compute the final payload size and record it in the 24-bit space
    /// reserved right after the opcode. (b2RecEndRecord)
    pub fn end_record(&mut self) {
        let payload_size = self.buffer.len() - self.record_start - 3;
        debug_assert!(payload_size < (1 << 24));
        let p = &mut self.buffer[self.record_start..self.record_start + 3];
        p[0] = payload_size as u8;
        p[1] = (payload_size >> 8) as u8;
        p[2] = (payload_size >> 16) as u8;
    }

    /// Append a completed record (opcode + u24 size + payload) in one shot.
    /// (b2RecCommitRecord — lock-free in the serial port)
    pub fn commit_record(&mut self, opcode: u8, payload: &[u8]) {
        debug_assert!(payload.len() < (1 << 24));
        rec_w_u8(&mut self.buffer, opcode);
        let size = payload.len();
        self.buffer
            .extend_from_slice(&[size as u8, (size >> 8) as u8, (size >> 16) as u8]);
        self.buffer.extend_from_slice(payload);
    }

    /// Fold one step's world bounds into the running union the recorder
    /// writes out at stop. (b2RecAccumulateBounds)
    pub fn accumulate_bounds(&mut self, bounds: Aabb) {
        if self.have_bounds {
            self.accumulated_bounds =
                crate::math_functions::aabb_union(self.accumulated_bounds, bounds);
        } else {
            self.accumulated_bounds = bounds;
            self.have_bounds = true;
        }
    }
}

/// Reserve a u32 slot for backfill (query hit counts). Returns its offset.
/// (b2RecReserveU32)
pub fn rec_reserve_u32(buf: &mut Vec<u8>) -> usize {
    let offset = buf.len();
    buf.extend_from_slice(&[0, 0, 0, 0]);
    offset
}

/// Backfill a reserved u32 slot. (b2RecPatchU32)
pub fn rec_patch_u32(buf: &mut [u8], offset: usize, v: u32) {
    debug_assert!(offset + 4 <= buf.len());
    buf[offset..offset + 4].copy_from_slice(&v.to_le_bytes());
}

/// Deterministic hash over all body transforms and velocities. Called by
/// both recorder and replayer to verify simulation reproduces exactly.
/// (b2HashWorldState)
pub fn hash_world_state(world: &World) -> u64 {
    let mut hash = SNAP_FNV_INIT;
    let prime = SNAP_FNV_PRIME;

    let mix_f32 = |hash: &mut u64, f: f32| {
        *hash = (*hash ^ f.to_bits() as u64).wrapping_mul(prime);
    };

    for (i, body) in world.bodies.iter().enumerate() {
        if body.id != i as i32 {
            // Free or never-used slot
            continue;
        }

        let sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

        hash = fnv_mix_position(hash, sim.transform.p);
        mix_f32(&mut hash, sim.transform.q.c);
        mix_f32(&mut hash, sim.transform.q.s);

        if body.set_index == crate::solver_set::AWAKE_SET {
            let state = &world.solver_sets[crate::solver_set::AWAKE_SET as usize].body_states
                [body.local_index as usize];
            mix_f32(&mut hash, state.linear_velocity.x);
            mix_f32(&mut hash, state.linear_velocity.y);
            mix_f32(&mut hash, state.angular_velocity);
        }
    }

    hash
}
