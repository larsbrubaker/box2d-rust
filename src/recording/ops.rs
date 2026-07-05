// Recording op stream: opcode manifest from recording_ops.inl, the engine-
// emitted op writers, start/stop, and replay validation.
//
// Ownership differs from C by design: the C world holds a pointer to a
// user-owned b2Recording; the Rust world takes ownership for the duration of
// the session (world_start_recording moves the Recording in,
// world_stop_recording moves it back out).
//
// The API-mutation op writers (create body/shape/joint, setters, queries)
// land as their call-site hooks are added; the replay dispatcher skips
// unknown opcodes by their framed size, exactly like C's b2RecDispatchOne.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::snapshot::SnapReader;
use super::write::*;
use super::{RecHeader, Recording};
use crate::id::WorldId;
use crate::math_functions::Aabb;
use crate::world::World;

// Opcode manifest. (recording_ops.inl — ranges: 0x0x world config,
// 0x1x-0x3x body, 0x4x-0x6x shape, 0x7x chain, 0x80 step, 0x9x-0xD1 joints,
// 0xEx queries, 0xFx markers)
pub const OP_DESTROY_WORLD: u8 = 0x01;
pub const OP_WORLD_ENABLE_SLEEPING: u8 = 0x02;
pub const OP_WORLD_ENABLE_CONTINUOUS: u8 = 0x03;
pub const OP_WORLD_SET_RESTITUTION_THRESHOLD: u8 = 0x04;
pub const OP_WORLD_SET_HIT_EVENT_THRESHOLD: u8 = 0x05;
pub const OP_WORLD_SET_GRAVITY: u8 = 0x06;
pub const OP_WORLD_EXPLODE: u8 = 0x07;
pub const OP_WORLD_SET_CONTACT_TUNING: u8 = 0x08;
pub const OP_WORLD_SET_CONTACT_RECYCLE_DISTANCE: u8 = 0x09;
pub const OP_WORLD_SET_MAXIMUM_LINEAR_SPEED: u8 = 0x0A;
pub const OP_WORLD_ENABLE_WARM_STARTING: u8 = 0x0B;
pub const OP_WORLD_REBUILD_STATIC_TREE: u8 = 0x0C;
pub const OP_WORLD_ENABLE_SPECULATIVE: u8 = 0x0D;
pub const OP_STEP: u8 = 0x80;
pub const OP_STATE_HASH: u8 = 0xF1;
pub const OP_RECORDING_BOUNDS: u8 = 0xF2;

/// Run an op writer against the active recording session, if any. This is
/// the B2_REC macro: one branch when recording is off, the args built inside
/// the branch. (B2_REC)
pub(crate) fn record_op(world: &mut World, f: impl FnOnce(&mut Recording, WorldId)) {
    if let Some(mut rec) = world.recording.take() {
        let world_id = world_id_of(world);
        f(&mut rec, world_id);
        world.recording = Some(rec);
    }
}

/// A world-config op carrying a single bool. (WorldEnableSleeping and kin)
pub(crate) fn write_world_bool(rec: &mut Recording, opcode: u8, world_id: WorldId, flag: bool) {
    rec.begin_record(opcode);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_bool(&mut rec.buffer, flag);
    rec.end_record();
}

/// A world-config op carrying a single f32. (WorldSetRestitutionThreshold
/// and kin)
pub(crate) fn write_world_f32(rec: &mut Recording, opcode: u8, world_id: WorldId, value: f32) {
    rec.begin_record(opcode);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_f32(&mut rec.buffer, value);
    rec.end_record();
}

/// A world-config op with no payload beyond the world id.
/// (WorldRebuildStaticTree)
pub(crate) fn write_world_marker(rec: &mut Recording, opcode: u8, world_id: WorldId) {
    rec.begin_record(opcode);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec.end_record();
}

pub(crate) fn write_world_set_gravity(
    rec: &mut Recording,
    world_id: WorldId,
    gravity: crate::math_functions::Vec2,
) {
    rec.begin_record(OP_WORLD_SET_GRAVITY);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_vec2(&mut rec.buffer, gravity);
    rec.end_record();
}

pub(crate) fn write_world_set_contact_tuning(
    rec: &mut Recording,
    world_id: WorldId,
    hertz: f32,
    damping_ratio: f32,
    push_speed: f32,
) {
    rec.begin_record(OP_WORLD_SET_CONTACT_TUNING);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_f32(&mut rec.buffer, hertz);
    rec_w_f32(&mut rec.buffer, damping_ratio);
    rec_w_f32(&mut rec.buffer, push_speed);
    rec.end_record();
}

pub(crate) fn write_world_explode(
    rec: &mut Recording,
    world_id: WorldId,
    def: &crate::types::ExplosionDef,
) {
    rec.begin_record(OP_WORLD_EXPLODE);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_explosiondef(&mut rec.buffer, def);
    rec.end_record();
}

#[cfg(feature = "double-precision")]
pub(crate) fn read_position(r: &mut SnapReader) -> crate::math_functions::Pos {
    crate::math_functions::Pos {
        x: r.r_f64(),
        y: r.r_f64(),
    }
}

#[cfg(not(feature = "double-precision"))]
pub(crate) fn read_position(r: &mut SnapReader) -> crate::math_functions::Pos {
    crate::math_functions::Pos {
        x: r.r_f32(),
        y: r.r_f32(),
    }
}

fn world_id_of(world: &World) -> WorldId {
    WorldId {
        index1: world.world_id + 1,
        generation: world.generation,
    }
}

// Engine-emitted op writers. (codegen b2RecWrite_<Name>: begin, args, end)

pub(crate) fn write_step(rec: &mut Recording, world_id: WorldId, dt: f32, sub_step_count: i32) {
    rec.begin_record(OP_STEP);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_f32(&mut rec.buffer, dt);
    rec_w_i32(&mut rec.buffer, sub_step_count);
    rec.end_record();
}

pub(crate) fn write_state_hash(rec: &mut Recording, world_id: WorldId, hash: u64) {
    rec.begin_record(OP_STATE_HASH);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec_w_u64(&mut rec.buffer, hash);
    rec.end_record();
}

pub(crate) fn write_recording_bounds(rec: &mut Recording, bounds: Aabb) {
    rec.begin_record(OP_RECORDING_BOUNDS);
    rec_w_aabb(&mut rec.buffer, bounds);
    rec.end_record();
}

pub(crate) fn write_destroy_world(rec: &mut Recording, world_id: WorldId) {
    rec.begin_record(OP_DESTROY_WORLD);
    rec_w_worldid(&mut rec.buffer, world_id);
    rec.end_record();
}

/// Begin recording into the buffer: header, seed snapshot, seed bounds, and
/// the anchoring state hash. (b2StartRecordingIntoBuffer)
pub(crate) fn start_recording_into_buffer(world: &mut World, mut recording: Recording) {
    // Reset so a recording handle can be reused for a fresh session
    recording.buffer.clear();
    recording.have_bounds = false;

    // Serialize the live world into a blob that follows the header and seeds
    // replay.
    let mut blob = Vec::new();
    super::serialize_world(world, &mut blob);

    let header = RecHeader {
        magic: super::REC_MAGIC,
        version_major: super::REC_VERSION_MAJOR,
        version_minor: super::REC_VERSION_MINOR,
        length_scale: crate::core::get_length_units_per_meter(),
        pointer_width: std::mem::size_of::<usize>() as u8,
        big_endian: 0,
        validation_enabled: if cfg!(debug_assertions) { 1 } else { 0 },
        snapshot_size: blob.len() as u64,
    };
    header.write(&mut recording.buffer);
    recording.buffer.extend_from_slice(&blob);

    // Seed the bounds with the snapshot state so frame 0 is framed even if
    // nothing moves
    let (seed, have_bounds) = crate::world::compute_world_bounds(world);
    if have_bounds {
        recording.accumulate_bounds(seed);
    }

    // Anchor the recorded state hash so replay verifies the blob
    // deserialized to the same world.
    let world_id = world_id_of(world);
    let hash = super::hash_world_state(world);
    write_state_hash(&mut recording, world_id, hash);

    world.recording = Some(recording);
}

/// Stop recording: append the accumulated bounds and the DestroyWorld end
/// marker, and hand the buffer back. (b2StopRecordingInternal)
pub(crate) fn stop_recording_internal(world: &mut World) -> Option<Recording> {
    let mut rec = world.recording.take()?;

    // Stash the accumulated bounds so a viewer can frame the whole motion at
    // open time. Sits in the op stream ahead of the end marker.
    let bounds = if rec.have_bounds {
        rec.accumulated_bounds
    } else {
        Aabb::default()
    };
    write_recording_bounds(&mut rec, bounds);

    // Write DestroyWorld so the buffer is self-contained, an end marker the
    // viewer reads.
    let world_id = world_id_of(world);
    write_destroy_world(&mut rec, world_id);

    Some(rec)
}

/// Start recording this world's session into the given recording buffer.
/// No-op if a session is already active (the recording is returned unused).
/// (b2World_StartRecording)
pub fn world_start_recording(world: &mut World, recording: Recording) -> Option<Recording> {
    // Must be a step boundary, so refuse a locked world
    debug_assert!(!world.locked);
    if world.locked || world.recording.is_some() {
        return Some(recording);
    }

    start_recording_into_buffer(world, recording);
    None
}

/// Stop the active recording session and return the finished buffer.
/// (b2World_StopRecording)
pub fn world_stop_recording(world: &mut World) -> Option<Recording> {
    debug_assert!(!world.locked);
    if world.locked {
        return None;
    }

    stop_recording_internal(world)
}

/// Per-step recording emission, called by world_step while the world is
/// still locked so the buffer stays single-writer. (the recording block at
/// the end of b2World_Step)
pub(crate) fn record_step_end(world: &mut World) {
    let Some(mut rec) = world.recording.take() else {
        return;
    };

    // StateHash proves the simulation reproduced exactly on replay.
    let world_id = world_id_of(world);
    let hash = super::hash_world_state(world);
    write_state_hash(&mut rec, world_id, hash);

    // Grow the recorded bounds so a replay can frame the whole motion, not
    // just frame 0
    let (bounds, have_bounds) = crate::world::compute_world_bounds(world);
    if have_bounds {
        rec.accumulate_bounds(bounds);
    }

    world.recording = Some(rec);
}

/// Result of a replay pass. (b2RecPlayer diagnostics, condensed)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ReplayResult {
    pub steps: i32,
    pub hash_checks: i32,
    pub diverged: bool,
    pub ok: bool,
    /// Accumulated session bounds from the RecordingBounds record, for
    /// framing a viewer. (b2RecPlayer_GetInfo().bounds)
    pub bounds: Aabb,
    pub have_bounds: bool,
}

/// Persist a recording buffer. The library never opens files while
/// recording; this lets a host save a finished session.
/// (b2SaveRecordingToFile)
pub fn save_recording_to_file(recording: &Recording, path: &std::path::Path) -> bool {
    std::fs::write(path, &recording.buffer).is_ok()
}

/// Load a recording buffer saved by [`save_recording_to_file`].
/// (b2LoadRecordingFromFile)
pub fn load_recording_from_file(path: &std::path::Path) -> Option<Recording> {
    let buffer = std::fs::read(path).ok()?;
    let mut recording = Recording::new(0);
    recording.buffer = buffer;
    Some(recording)
}

/// Replay a recording buffer against a world restored from its seed
/// snapshot, verifying every recorded StateHash. Unknown opcodes are skipped
/// by their framed size, like C's b2RecDispatchOne, so a stream containing
/// not-yet-dispatched mutation ops still advances. Returns true only when
/// the whole stream reads cleanly and no hash diverges. (b2ValidateReplay)
pub fn validate_replay(data: &[u8]) -> bool {
    let result = replay_buffer(data);
    result.ok && !result.diverged
}

/// Dispatch a world-config opcode (0x01-0x0D). Shared by the linear
/// replay loop and the incremental player. Returns None when the opcode
/// is not in this family.
pub(crate) fn dispatch_world_op(opcode: u8, r: &mut SnapReader, world: &mut World) -> Option<bool> {
    match opcode {
        OP_WORLD_ENABLE_SLEEPING => {
            let _ = r.r_u32();
            let flag = r.r_bool();
            crate::world::world_enable_sleeping(world, flag);
        }
        OP_WORLD_ENABLE_CONTINUOUS => {
            let _ = r.r_u32();
            let flag = r.r_bool();
            crate::world::world_enable_continuous(world, flag);
        }
        OP_WORLD_SET_RESTITUTION_THRESHOLD => {
            let _ = r.r_u32();
            let value = r.r_f32();
            crate::world::world_set_restitution_threshold(world, value);
        }
        OP_WORLD_SET_HIT_EVENT_THRESHOLD => {
            let _ = r.r_u32();
            let value = r.r_f32();
            crate::world::world_set_hit_event_threshold(world, value);
        }
        OP_WORLD_SET_GRAVITY => {
            let _ = r.r_u32();
            let gravity = crate::math_functions::Vec2 {
                x: r.r_f32(),
                y: r.r_f32(),
            };
            crate::world::world_set_gravity(world, gravity);
        }
        OP_WORLD_EXPLODE => {
            let _ = r.r_u32();
            let mut def = crate::types::default_explosion_def();
            def.mask_bits = r.r_u64();
            def.position = read_position(r);
            def.radius = r.r_f32();
            def.falloff = r.r_f32();
            def.impulse_per_length = r.r_f32();
            crate::world::world_explode(world, &def);
        }
        OP_WORLD_SET_CONTACT_TUNING => {
            let _ = r.r_u32();
            let hertz = r.r_f32();
            let damping_ratio = r.r_f32();
            let push_speed = r.r_f32();
            crate::world::world_set_contact_tuning(world, hertz, damping_ratio, push_speed);
        }
        OP_WORLD_SET_CONTACT_RECYCLE_DISTANCE => {
            let _ = r.r_u32();
            let value = r.r_f32();
            crate::world::world_set_contact_recycle_distance(world, value);
        }
        OP_WORLD_SET_MAXIMUM_LINEAR_SPEED => {
            let _ = r.r_u32();
            let value = r.r_f32();
            crate::world::world_set_maximum_linear_speed(world, value);
        }
        OP_WORLD_ENABLE_WARM_STARTING => {
            let _ = r.r_u32();
            let flag = r.r_bool();
            crate::world::world_enable_warm_starting(world, flag);
        }
        OP_WORLD_REBUILD_STATIC_TREE => {
            let _ = r.r_u32();
            crate::world::world_rebuild_static_tree(world);
        }
        OP_WORLD_ENABLE_SPECULATIVE => {
            let _ = r.r_u32();
            let flag = r.r_bool();
            crate::world::world_enable_speculative(world, flag);
        }
        _ => return None,
    }
    Some(true)
}

/// (b2ReplayFile core loop, serial)
pub fn replay_buffer(data: &[u8]) -> ReplayResult {
    let mut result = ReplayResult::default();

    let Some(header) = RecHeader::read(data) else {
        return result;
    };
    if header.magic != super::REC_MAGIC
        || header.version_major != super::REC_VERSION_MAJOR
        || header.version_minor != super::REC_VERSION_MINOR
    {
        return result;
    }

    let snapshot_start = RecHeader::SIZE;
    let snapshot_end = snapshot_start + header.snapshot_size as usize;
    if snapshot_end > data.len() {
        return result;
    }

    let Some(mut world) = super::create_world_from_snapshot(&data[snapshot_start..snapshot_end])
    else {
        return result;
    };

    let mut r = SnapReader::new(&data[snapshot_end..]);
    while r.ok && r.cursor < r.data.len() {
        let opcode = r.r_u8();
        // u24 payload size
        let payload_size = r.r_u8() as usize | (r.r_u8() as usize) << 8 | (r.r_u8() as usize) << 16;
        let payload_start = r.cursor;
        if !r.ok || payload_start + payload_size > r.data.len() {
            return result;
        }

        match opcode {
            OP_STEP => {
                let _world_id = r.r_u32();
                let dt = r.r_f32();
                let sub_step_count = r.r_i32();
                crate::world::world_step(&mut world, dt, sub_step_count);
                result.steps += 1;
            }
            OP_STATE_HASH => {
                let _world_id = r.r_u32();
                let recorded = r.r_u64();
                let computed = super::hash_world_state(&world);
                result.hash_checks += 1;
                if recorded != computed {
                    // Non-fatal: reading continues so a viewer can show where
                    // divergence begins
                    result.diverged = true;
                }
            }
            OP_RECORDING_BOUNDS => {
                result.bounds = Aabb {
                    lower_bound: crate::math_functions::Vec2 {
                        x: r.r_f32(),
                        y: r.r_f32(),
                    },
                    upper_bound: crate::math_functions::Vec2 {
                        x: r.r_f32(),
                        y: r.r_f32(),
                    },
                };
                result.have_bounds = true;
            }
            OP_DESTROY_WORLD => {
                // End-of-session marker
                result.ok = true;
                return result;
            }
            _ => {
                let handled = dispatch_world_op(opcode, &mut r, &mut world)
                    .or_else(|| super::ops_body::dispatch_body_op(opcode, &mut r, &mut world, None))
                    .or_else(|| super::ops_shape::dispatch_shape_op(opcode, &mut r, &mut world))
                    .or_else(|| super::ops_joint::dispatch_joint_op(opcode, &mut r, &mut world))
                    .or_else(|| {
                        super::ops_query::dispatch_query_op(opcode, &mut r, &mut world, None)
                    });
                if let Some(ids_match) = handled {
                    if !ids_match {
                        // A create op returned a different id than recorded:
                        // the replay is not deterministic.
                        result.diverged = true;
                    }
                } else {
                    // Mutation ops gain dispatchers as their hooks land; skip
                    // by framed size
                }
            }
        }

        r.cursor = payload_start + payload_size;
    }

    result.ok = r.ok;
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::create_body;
    use crate::geometry::{make_box, make_square};
    use crate::math_functions::to_pos;
    use crate::math_functions::Vec2;
    use crate::shape::create_polygon_shape;
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::world_step;

    // Record a settling pile, then replay from the seed snapshot: every
    // recorded per-step StateHash must match the recomputed hash.
    #[test]
    fn record_and_validate_replay() {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let bd = default_body_def();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground, &sd, &make_box(20.0, 1.0));
        for i in 0..10 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: -2.0 + 0.45 * i as f32,
                y: 2.0 + 0.5 * i as f32,
            });
            let body = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body, &sd, &make_square(0.25));
        }

        // Settle a little before recording so the seed snapshot is nontrivial.
        for _ in 0..15 {
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        assert!(world_start_recording(&mut world, Recording::new(0)).is_none());
        // Double-start is refused and hands the buffer back.
        assert!(world_start_recording(&mut world, Recording::new(0)).is_some());

        for _ in 0..60 {
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        let recording = world_stop_recording(&mut world).expect("active session");
        assert!(world.recording.is_none());
        assert!(recording.have_bounds);
        assert!(recording.buffer.len() > RecHeader::SIZE);

        let result = replay_buffer(&recording.buffer);
        assert!(result.ok, "stream must parse to the end marker");
        assert!(!result.diverged, "replay hashes must match");
        assert_eq!(result.steps, 60);
        // Anchor hash + one per step
        assert_eq!(result.hash_checks, 61);
        assert!(validate_replay(&recording.buffer));

        // Corrupting a recorded hash diverges but still parses. The stream
        // tail is StateHash (16 bytes) + RecordingBounds (20) + DestroyWorld
        // (8); the final hash payload sits at len-36..len-28.
        let mut corrupt = recording.buffer.clone();
        let len = corrupt.len();
        corrupt[len - 30] ^= 0x01;
        let bad = replay_buffer(&corrupt);
        assert!(bad.diverged && bad.ok);
    }

    // World-config mutations recorded mid-stream replay through their
    // dispatchers: a gravity flip and an explosion change the trajectory, so
    // hashes only match if the ops re-execute at the right steps.
    #[test]
    fn config_ops_replay() {
        let mut world_def = default_world_def();
        world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
        let mut world = World::new(&world_def);

        let bd = default_body_def();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground, &sd, &make_box(15.0, 1.0));
        for i in 0..6 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: -2.0 + 0.8 * i as f32,
                y: 2.0,
            });
            let body = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body, &sd, &make_square(0.3));
        }

        assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

        for step in 0..90 {
            if step == 20 {
                let mut def = crate::types::default_explosion_def();
                def.position = to_pos(Vec2 { x: 0.0, y: 1.5 });
                def.radius = 2.0;
                def.falloff = 2.0;
                def.impulse_per_length = 4.0;
                crate::world::world_explode(&mut world, &def);
            }
            if step == 40 {
                crate::world::world_set_gravity(&mut world, Vec2 { x: 0.0, y: 3.0 });
                crate::world::world_enable_sleeping(&mut world, false);
            }
            if step == 60 {
                crate::world::world_set_gravity(&mut world, Vec2 { x: 0.0, y: -10.0 });
                crate::world::world_set_contact_tuning(&mut world, 20.0, 5.0, 2.0);
            }
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        let recording = world_stop_recording(&mut world).expect("active session");
        let result = replay_buffer(&recording.buffer);
        assert!(result.ok);
        assert!(!result.diverged, "config ops must re-execute on replay");
        assert_eq!(result.steps, 90);
    }
}
