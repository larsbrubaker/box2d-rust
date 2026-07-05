// Incremental recording player (b2RecPlayer in recording_replay.c): owns a
// private copy of the recording bytes and drives replay one step at a time,
// with an outliner body list, a keyframe ring for fast backward seeks, and a
// per-frame query stash for the viewer.
//
// The C player creates its replay world in the global registry and exposes a
// b2WorldId; the registry-less Rust port owns the World directly and exposes
// world()/world_mut() instead.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::ops::{
    dispatch_world_op, OP_DESTROY_WORLD, OP_RECORDING_BOUNDS, OP_STATE_HASH, OP_STEP,
};
use super::player_queries::{
    draw_stashed_queries, stash_query_hit, stash_query_info, QueryStash, RecQueryHit, RecQueryInfo,
};
use super::snapshot::SnapReader;
use super::RecHeader;
use crate::body::make_body_id;
use crate::core::{get_length_units_per_meter, set_length_units_per_meter};
use crate::debug_draw::DebugDraw;
use crate::id::BodyId;
use crate::math_functions::Aabb;
use crate::world::World;

// Keyframe ring tuning. A memory budget caps the snapshots kept; the spacing
// starts at the min and doubles when adding the next keyframe would exceed
// the budget, so memory stays bounded and seek cost grows only once a
// recording outgrows the budget.
const KEYFRAME_INTERVAL_DEFAULT: i32 = 16;
const KEYFRAME_BUDGET_DEFAULT: usize = 512 * 1024 * 1024;

/// Static metadata describing a recording, resolved once when the player
/// opens the file. (b2RecPlayerInfo)
#[derive(Debug, Clone, Copy, Default)]
pub struct RecPlayerInfo {
    /// Total recorded steps.
    pub frame_count: i32,
    /// dt of the recorded steps.
    pub time_step: f32,
    /// Recorded sub-steps.
    pub sub_step_count: i32,
    /// Length units per meter in effect when recorded.
    pub length_scale: f32,
    /// Accumulated world bounds over the recording, zero-extent if
    /// unavailable.
    pub bounds: Aabb,
}

/// A restore point captured during forward replay, so a backward seek can
/// re-simulate only the gap from the nearest keyframe instead of from frame
/// 0. The image is a full serialize_world blob. (b2RecKeyframe)
struct Keyframe {
    /// Serialized world at the end of this frame.
    image: Vec<u8>,
    /// Frame this restores to, a post-step boundary.
    frame: i32,
    /// Op-stream offset where the next frame resumes.
    cursor: usize,
    /// Outliner list as of this frame.
    body_ids: Vec<BodyId>,
    /// Divergence latches as of this frame, so a seek reports the linear
    /// path's state.
    diverge_frame: i32,
    diverged: bool,
}

impl Keyframe {
    fn bytes(&self) -> usize {
        self.image.len() + self.body_ids.len() * std::mem::size_of::<BodyId>()
    }
}

/// Incremental player. Owns a private copy of the recording bytes and drives
/// replay one step at a time. (b2RecPlayer)
pub struct RecPlayer {
    /// Recording bytes, a private copy owned here.
    data: Vec<u8>,
    /// First payload offset (past header and the seed snapshot blob, which
    /// doubles as the frame-0 restore image).
    header_end: usize,
    /// Length scale used in the recording.
    length_scale: f32,
    /// Global length scale before this player overrode it, restored on drop.
    previous_length_scale: f32,
    /// Steps dispatched so far.
    frame: i32,
    /// Total recorded steps, counted once at open.
    frame_count: i32,
    /// dt of the first recorded step.
    recorded_dt: f32,
    /// Sub-steps of the first recorded step.
    recorded_sub_step_count: i32,
    /// Accumulated world bounds, resolved by the open-time scan.
    bounds: Aabb,
    /// First step that diverged, -1 until then.
    diverge_frame: i32,
    /// A step_frame ran out of records without reaching a step.
    at_end: bool,

    // Reader state (b2RecReader, minus the borrow: transient SnapReader
    // windows are built per record instead)
    cursor: usize,
    ok: bool,
    diverged: bool,

    /// The replay world, restored from the seed snapshot.
    world: World,

    /// Per-frame query store, reset at the top of each step_frame.
    stash: QueryStash,

    /// Live bodies in creation order, tracked from create/destroy ops to
    /// drive the viewer outliner. Destroyed slots hold a null id so ordinals
    /// stay stable. Rebuilt deterministically on replay.
    body_ids: Vec<BodyId>,
    /// Frame-0 list so a restart or backward scrub rolls the outliner back.
    frame0_body_ids: Vec<BodyId>,

    // Keyframe ring for fast backward seeks. Captured in increasing-frame
    // order as the replay plays forward. The spacing doubles and the
    // off-grid keyframes are evicted once the memory budget is hit.
    keyframes: Vec<Keyframe>,
    /// Memory cap in bytes for the kept snapshots.
    keyframe_budget: usize,
    /// Running total of kept snapshot + body-list bytes.
    keyframe_bytes: usize,
    /// Finest spacing in frames.
    keyframe_min_interval: i32,
    /// Current spacing, a power-of-two multiple of the min, doubles on
    /// eviction.
    keyframe_interval: i32,
    /// Highest frame captured, guards against re-capture while back-stepping.
    last_keyframe_frame: i32,
}

impl RecPlayer {
    /// Open a recording for incremental playback. The player copies the
    /// bytes, so the source buffer can be dropped immediately after this
    /// call. Returns None if the recording is malformed.
    /// (b2RecPlayer_Create; workerCount is not a parameter of the serial
    /// port.)
    pub fn create(data: &[u8]) -> Option<RecPlayer> {
        if data.len() < RecHeader::SIZE {
            return None; // recording too small
        }

        // Validate the header before copying anything
        let header = RecHeader::read(data)?;
        if header.magic != super::REC_MAGIC
            || header.version_major != super::REC_VERSION_MAJOR
            || header.version_minor != super::REC_VERSION_MINOR
            || header.pointer_width != std::mem::size_of::<usize>() as u8
            || header.big_endian != 0
        {
            return None;
        }

        // Every recording is snapshot-seeded: the blob sits between the
        // header and the op stream
        if header.snapshot_size == 0 || header.snapshot_size > (data.len() - RecHeader::SIZE) as u64
        {
            return None;
        }
        let header_end = RecHeader::SIZE + header.snapshot_size as usize;

        // Override the global length scale with the recording's so replay
        // reproduces the same constants. This is global engine state, so the
        // previous value is captured and restored on drop.
        let previous_length_scale = get_length_units_per_meter();
        if header.length_scale > 0.0 {
            set_length_units_per_meter(header.length_scale);
        }

        // Deserialize the seed snapshot to stand up the replay world. The op
        // stream that follows is the hook log. The blob doubles as the
        // frame-0 restore image, owned by the copy held here.
        let Some(world) = super::create_world_from_snapshot(&data[RecHeader::SIZE..header_end])
        else {
            set_length_units_per_meter(previous_length_scale);
            return None; // snapshot deserialize failed
        };

        let mut player = RecPlayer {
            data: data.to_vec(),
            header_end,
            length_scale: header.length_scale,
            previous_length_scale,
            frame: 0,
            frame_count: 0,
            recorded_dt: 0.0,
            recorded_sub_step_count: 0,
            bounds: Aabb::default(),
            diverge_frame: -1,
            at_end: false,
            cursor: header_end,
            ok: true,
            diverged: false,
            world,
            stash: QueryStash::default(),
            body_ids: Vec::new(),
            frame0_body_ids: Vec::new(),
            keyframes: Vec::new(),
            keyframe_budget: KEYFRAME_BUDGET_DEFAULT,
            keyframe_bytes: 0,
            keyframe_min_interval: KEYFRAME_INTERVAL_DEFAULT,
            keyframe_interval: KEYFRAME_INTERVAL_DEFAULT,
            last_keyframe_frame: 0,
        };

        // Count steps and read the first step's tuning so a viewer can show
        // length and hz up front
        player.scan_file();

        // The seed snapshot holds the bodies present when recording began;
        // only post-snapshot creates reach the tracker, so seed the outliner
        // list directly from the restored world. (b2RecSeedBodyIds)
        player.seed_body_ids();
        player.frame0_body_ids = player.body_ids.clone();

        Some(player)
    }

    /// Count steps, read the first step's tuning, and pick up the bounds
    /// record, without touching the world. (b2RecScanFile)
    fn scan_file(&mut self) {
        let data = &self.data;
        let size = data.len();
        let mut cursor = self.header_end;
        let mut frame_count = 0;
        let mut got_step = false;

        while cursor + 4 <= size {
            let opcode = data[cursor];
            let payload_size = data[cursor + 1] as usize
                | (data[cursor + 2] as usize) << 8
                | (data[cursor + 3] as usize) << 16;
            let payload_start = cursor + 4;
            if payload_start + payload_size > size {
                break;
            }

            if opcode == OP_STEP {
                // Step: [u32 world][f32 dt][i32 subStepCount]
                frame_count += 1;
                if !got_step && payload_size >= 12 {
                    let mut r = SnapReader::new(&data[payload_start + 4..payload_start + 12]);
                    self.recorded_dt = r.r_f32();
                    self.recorded_sub_step_count = r.r_i32();
                    got_step = true;
                }
            } else if opcode == OP_RECORDING_BOUNDS && payload_size >= 16 {
                // RecordingBounds: [f32 lo.x][lo.y][hi.x][hi.y]
                let mut r = SnapReader::new(&data[payload_start..payload_start + 16]);
                self.bounds.lower_bound.x = r.r_f32();
                self.bounds.lower_bound.y = r.r_f32();
                self.bounds.upper_bound.x = r.r_f32();
                self.bounds.upper_bound.y = r.r_f32();
            }

            cursor = payload_start + payload_size;
        }

        self.frame_count = frame_count;
    }

    /// Populate the outliner list from the restored world; slot order is
    /// stable. (b2RecSeedBodyIds)
    fn seed_body_ids(&mut self) {
        self.body_ids.clear();
        for i in 0..self.world.bodies.len() {
            if self.world.bodies[i].id != i as i32 {
                continue; // free slot
            }
            self.body_ids.push(make_body_id(&self.world, i as i32));
        }
    }

    /// Dispatch the record at the cursor and advance past it. Returns the
    /// opcode, or None on framing failure. (b2RecDispatchOne)
    fn dispatch_one(&mut self) -> Option<u8> {
        if self.cursor + 4 > self.data.len() {
            return None;
        }
        let opcode = self.data[self.cursor];
        let payload_size = self.data[self.cursor + 1] as usize
            | (self.data[self.cursor + 2] as usize) << 8
            | (self.data[self.cursor + 3] as usize) << 16;
        let payload_start = self.cursor + 4;
        let payload_end = payload_start + payload_size;
        if payload_end > self.data.len() {
            self.ok = false;
            return None;
        }

        match opcode {
            OP_STEP => {
                let mut r = SnapReader::new(&self.data[payload_start..payload_end]);
                let _world_id = r.r_u32();
                let dt = r.r_f32();
                let sub_step_count = r.r_i32();
                if r.ok {
                    crate::world::world_step(&mut self.world, dt, sub_step_count);
                } else {
                    self.ok = false;
                }
            }
            OP_STATE_HASH => {
                let mut r = SnapReader::new(&self.data[payload_start..payload_end]);
                let _world_id = r.r_u32();
                let recorded = r.r_u64();
                if r.ok && recorded != super::hash_world_state(&self.world) {
                    // Non-fatal so a viewer can keep playing past the first
                    // divergent frame
                    self.diverged = true;
                }
            }
            OP_RECORDING_BOUNDS => {
                // Resolved by the open-time scan
            }
            OP_DESTROY_WORLD => {
                // The recorded session ended here. The player owns the
                // replay world's lifetime, so a viewer can keep drawing the
                // final step. This is always the last record.
            }
            _ => {
                // Split borrows: the reader window borrows `data`, the
                // dispatchers mutate `world` and the player hooks.
                let RecPlayer {
                    data,
                    world,
                    body_ids,
                    stash,
                    ..
                } = self;
                let mut r = SnapReader::new(&data[payload_start..payload_end]);
                let handled = dispatch_world_op(opcode, &mut r, world)
                    .or_else(|| {
                        super::ops_body::dispatch_body_op(opcode, &mut r, world, Some(body_ids))
                    })
                    .or_else(|| super::ops_shape::dispatch_shape_op(opcode, &mut r, world))
                    .or_else(|| super::ops_joint::dispatch_joint_op(opcode, &mut r, world))
                    .or_else(|| {
                        super::ops_query::dispatch_query_op(opcode, &mut r, world, Some(stash))
                    });
                let reader_ok = r.ok;
                match handled {
                    Some(true) | None => {}
                    Some(false) => {
                        if (0xE0..=0xE8).contains(&opcode) {
                            // A query hit failed to reproduce: divergence,
                            // non-fatal (b2RecReplayQueryCtx semantics)
                            self.diverged = true;
                        } else {
                            // A create returned a different id than
                            // recorded: structural drift, fatal since later
                            // ops would target the wrong objects
                            // (b2RecCheckId semantics)
                            self.ok = false;
                        }
                    }
                }
                if !reader_ok {
                    self.ok = false;
                }
            }
        }

        self.cursor = payload_end;
        Some(opcode)
    }

    /// Advance the replay by one recorded step. Returns true if a step
    /// executed, false once the end of the recording is reached.
    /// (b2RecPlayer_StepFrame)
    pub fn step_frame(&mut self) -> bool {
        if self.at_end {
            return false;
        }

        // Reset the per-frame query store before dispatching new records
        self.stash.clear();

        // Run this frame's Step, then consume the records that trail it
        // (StateHash, queries, any between-frame mutators) up to the next
        // Step. The queries and hash for a frame are recorded after its
        // Step, so grouping them with that Step keeps them paired with the
        // world state they were computed against. Stopping before the next
        // Step is what advances exactly one frame.
        let mut stepped = false;
        loop {
            // Peek the next opcode without consuming it. The next frame's
            // Step ends this frame.
            if self.cursor >= self.data.len() || !self.ok {
                self.at_end = true;
                return stepped;
            }
            if stepped && self.data[self.cursor] == OP_STEP {
                // Capture a keyframe at the interval. The guard skips frames
                // already covered, so re-stepping a gap during a backward
                // seek never re-captures.
                if self.frame > self.last_keyframe_frame && self.frame % self.keyframe_interval == 0
                {
                    self.capture_keyframe();
                }
                return true;
            }

            let Some(opcode) = self.dispatch_one() else {
                self.at_end = true;
                return stepped;
            };
            if opcode == OP_STEP {
                self.frame += 1;
                stepped = true;
            }
            // Latch the first frame that diverged for the timeline marker
            if self.diverge_frame < 0 && self.diverged {
                self.diverge_frame = self.frame;
            }
        }
    }

    /// Capture a restore point for the just-completed frame. The cursor
    /// already sits at the next frame's Step, so this records the exact
    /// resume position next to a full world image plus the outliner and
    /// divergence state forward stepping would otherwise have to rebuild.
    /// (b2RecCaptureKeyframe)
    fn capture_keyframe(&mut self) {
        let image = super::world_snapshot(&self.world);
        let body_bytes = self.body_ids.len() * std::mem::size_of::<BodyId>();
        let new_bytes = image.len() + body_bytes;

        // Make room under the budget: doubling the spacing drops the
        // off-grid keyframes, roughly halving the bytes, until the new
        // keyframe fits or only it remains. The budget is soft in the corner
        // where a single snapshot already exceeds it.
        while !self.keyframes.is_empty() && self.keyframe_bytes + new_bytes > self.keyframe_budget {
            self.keyframe_interval *= 2;
            let before = self.keyframes.len();
            let interval = self.keyframe_interval;
            self.keyframes.retain(|kf| kf.frame % interval == 0);
            self.keyframe_bytes = self.keyframes.iter().map(Keyframe::bytes).sum();
            if self.keyframes.len() == before {
                break;
            }
        }

        self.keyframes.push(Keyframe {
            image,
            frame: self.frame,
            cursor: self.cursor,
            body_ids: self.body_ids.clone(),
            diverge_frame: self.diverge_frame,
            diverged: self.diverged,
        });
        self.keyframe_bytes += new_bytes;
        self.last_keyframe_frame = self.frame;
    }

    /// Restore the world and player state from a keyframe, so a backward
    /// seek resumes from it instead of frame 0. Mirrors restart but targets
    /// a mid-stream image; world_restore is in place, so the replay world
    /// stays the same object. (b2RecPlayerRestoreKeyframe)
    fn restore_keyframe(&mut self, index: usize) {
        let kf = &self.keyframes[index];
        if !super::world_restore(&mut self.world, &kf.image) {
            self.ok = false;
            return;
        }
        let kf = &self.keyframes[index];
        self.cursor = kf.cursor;
        self.ok = true;
        self.diverged = kf.diverged;
        self.frame = kf.frame;
        self.diverge_frame = kf.diverge_frame;
        self.at_end = false;
        self.body_ids = kf.body_ids.clone();
    }

    /// Rewind the player to the first step, recreating the replay world from
    /// the frame-0 image in place. (b2RecPlayer_Restart)
    pub fn restart(&mut self) {
        if !super::world_restore(
            &mut self.world,
            &self.data[RecHeader::SIZE..self.header_end],
        ) {
            self.ok = false;
            return;
        }
        // Stepping resumes at the first Step, which sits right after the
        // header and snapshot blob
        self.cursor = self.header_end;
        self.ok = true;
        self.diverged = false;
        self.frame = 0;
        self.diverge_frame = -1;
        self.at_end = false;

        // Frame 0 is the pre-step snapshot, so it has no recorded queries.
        // Clear the per-frame store so the last stepped frame's queries do
        // not linger on a load or a backward scrub to the start.
        self.stash.clear();

        // Roll the outliner body list back to its frame-0 contents
        self.body_ids.clone_from(&self.frame0_body_ids);
    }

    /// Seek to a recorded step. Seeking backward restores the nearest
    /// keyframe and re-runs the gap. Clamps to the recording bounds.
    /// (b2RecPlayer_SeekFrame)
    pub fn seek_frame(&mut self, target_frame: i32) {
        let target_frame = target_frame.max(0);

        // Resume from the nearest keyframe strictly below the target when it
        // beats the current cursor. A backward seek must restore since the
        // cursor cannot rewind. A forward seek restores only when a keyframe
        // sits ahead of the cursor, capping a long forward fling at one
        // keyframe interval of replay instead of every intervening frame.
        // Strictly below so the step loop still runs the target frame and
        // regenerates its per-frame query store, body list, and divergence
        // latch exactly as a plain forward replay would.
        let mut best: Option<usize> = None;
        for (i, kf) in self.keyframes.iter().enumerate() {
            if kf.frame < target_frame && best.map_or(true, |b| kf.frame > self.keyframes[b].frame)
            {
                best = Some(i);
            }
        }

        if target_frame < self.frame {
            match best {
                Some(i) => self.restore_keyframe(i),
                None => self.restart(),
            }
        } else if let Some(i) = best {
            if self.keyframes[i].frame > self.frame {
                self.restore_keyframe(i);
            }
        }

        while self.frame < target_frame && self.step_frame() {}
    }

    /// The replay world. (b2RecPlayer_GetWorldId)
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Mutable access to the replay world, e.g. for a viewer's world_draw.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// The number of steps replayed so far. (b2RecPlayer_GetFrame)
    pub fn frame(&self) -> i32 {
        self.frame
    }

    /// Static metadata for the recording. (b2RecPlayer_GetInfo)
    pub fn info(&self) -> RecPlayerInfo {
        RecPlayerInfo {
            frame_count: self.frame_count,
            time_step: self.recorded_dt,
            sub_step_count: self.recorded_sub_step_count,
            length_scale: self.length_scale,
            bounds: self.bounds,
        }
    }

    /// True once the end of the recording has been reached.
    /// (b2RecPlayer_IsAtEnd)
    pub fn is_at_end(&self) -> bool {
        self.at_end
    }

    /// True if a recorded state hash or query failed to reproduce.
    /// (b2RecPlayer_HasDiverged)
    pub fn has_diverged(&self) -> bool {
        self.diverged
    }

    /// The first step at which replay diverged, or -1.
    /// (b2RecPlayer_GetDivergeFrame)
    pub fn diverge_frame(&self) -> i32 {
        self.diverge_frame
    }

    /// Tune the keyframe ring. A zero budget or a non-positive interval
    /// keeps that value. Clears the existing ring, so call restart afterward
    /// to repopulate it under the new policy. (b2RecPlayer_SetKeyframePolicy)
    pub fn set_keyframe_policy(&mut self, budget_bytes: usize, min_interval_frames: i32) {
        if budget_bytes > 0 {
            self.keyframe_budget = budget_bytes;
        }
        if min_interval_frames > 0 {
            self.keyframe_min_interval = min_interval_frames;
        }

        // Drop the ring so it repopulates under the new policy on the next
        // replay
        self.keyframes.clear();
        self.keyframe_bytes = 0;
        self.keyframe_interval = self.keyframe_min_interval;
        self.last_keyframe_frame = 0;
    }

    /// (b2RecPlayer_GetKeyframeBudget)
    pub fn keyframe_budget(&self) -> usize {
        self.keyframe_budget
    }

    /// (b2RecPlayer_GetKeyframeMinInterval)
    pub fn keyframe_min_interval(&self) -> i32 {
        self.keyframe_min_interval
    }

    /// The current keyframe spacing in frames; reflects the effective
    /// backward-seek granularity right now. (b2RecPlayer_GetKeyframeInterval)
    pub fn keyframe_interval(&self) -> i32 {
        self.keyframe_interval
    }

    /// The memory currently held by keyframe snapshots, in bytes.
    /// (b2RecPlayer_GetKeyframeBytes)
    pub fn keyframe_bytes(&self) -> usize {
        self.keyframe_bytes
    }

    /// Draw spatial queries recorded during the most recently replayed
    /// frame. Call after world_draw so queries are layered on top of the
    /// world. `query_index` < 0 draws all of them.
    /// (b2RecPlayer_DrawFrameQueries)
    pub fn draw_frame_queries(&self, draw: &mut dyn DebugDraw, query_index: i32) {
        draw_stashed_queries(&self.world, &self.stash, draw, query_index);
    }

    /// The number of spatial queries recorded for the most recently replayed
    /// frame. (b2RecPlayer_GetFrameQueryCount)
    pub fn frame_query_count(&self) -> i32 {
        self.stash.queries.len() as i32
    }

    /// A recorded query from the most recently replayed frame by index.
    /// (b2RecPlayer_GetFrameQuery)
    pub fn frame_query(&self, index: i32) -> RecQueryInfo {
        stash_query_info(&self.stash, index)
    }

    /// One result of a recorded query from the most recently replayed frame.
    /// (b2RecPlayer_GetFrameQueryHit)
    pub fn frame_query_hit(&self, query_index: i32, hit_index: i32) -> RecQueryHit {
        stash_query_hit(&self.stash, query_index, hit_index)
    }

    /// The number of body slots tracked for the outliner. This is the
    /// creation-order span and includes holes for destroyed bodies, so it
    /// only grows as the replay advances. (b2RecPlayer_GetBodyCount)
    pub fn body_count(&self) -> i32 {
        self.body_ids.len() as i32
    }

    /// A tracked body by creation ordinal. Returns the null id for a
    /// destroyed slot or an out-of-range index; validate with body_is_valid.
    /// (b2RecPlayer_GetBodyId)
    pub fn body_id(&self, index: i32) -> BodyId {
        if index < 0 || index as usize >= self.body_ids.len() {
            return BodyId::default();
        }
        self.body_ids[index as usize]
    }

    /// Internal-parity check used by validate tests: framing and ids read
    /// cleanly so far.
    pub fn is_ok(&self) -> bool {
        self.ok
    }
}

impl Drop for RecPlayer {
    fn drop(&mut self) {
        // Restore the global length scale. (b2RecPlayer_Destroy; the world
        // and buffers are owned values, dropped normally.)
        set_length_units_per_meter(self.previous_length_scale);
    }
}
