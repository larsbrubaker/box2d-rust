// Replay demo bindings: record a SimWorld session, play it back with
// the ported b2RecPlayer. Split from lib.rs.

use crate::interact::collect_world_draw;
use crate::sim::SimWorld;
use box2d_rust::body::{get_body_full_id, get_body_transform};
use box2d_rust::math_functions as m;
use wasm_bindgen::prelude::*;

// ---------------------------------------------------------------------------
// Replay demo: record a SimWorld session, then play it back with the ported
// b2RecPlayer (keyframe ring, timeline scrub, divergence checking).
// ---------------------------------------------------------------------------

#[wasm_bindgen]
impl SimWorld {
    /// Start recording this world into an op-stream buffer seeded with a
    /// world snapshot. Returns false if a session is already active.
    pub fn start_recording(&mut self) -> bool {
        box2d_rust::recording::world_start_recording(
            &mut self.world,
            box2d_rust::recording::Recording::new(0),
        )
        .is_none()
    }

    /// Stop recording and return the finished recording bytes (empty if no
    /// session was active).
    pub fn stop_recording(&mut self) -> Vec<u8> {
        box2d_rust::recording::world_stop_recording(&mut self.world)
            .map(|rec| rec.buffer)
            .unwrap_or_default()
    }
}

/// Incremental playback of a recorded session via the ported b2RecPlayer.
#[wasm_bindgen]
pub struct SimPlayer {
    player: box2d_rust::recording::RecPlayer,
    draw_polygons: Vec<f32>,
    draw_circles: Vec<f32>,
    draw_capsules: Vec<f32>,
    draw_lines: Vec<f32>,
}

#[wasm_bindgen]
impl SimPlayer {
    /// Open a recording. Returns undefined if the bytes are malformed.
    pub fn open(data: &[u8]) -> Option<SimPlayer> {
        box2d_rust::recording::RecPlayer::create(data).map(|player| SimPlayer {
            player,
            draw_polygons: Vec::new(),
            draw_circles: Vec::new(),
            draw_capsules: Vec::new(),
            draw_lines: Vec::new(),
        })
    }

    /// Advance one recorded step. False once the end is reached.
    pub fn step_frame(&mut self) -> bool {
        self.player.step_frame()
    }

    /// Seek to a recorded step; backward seeks restore the nearest keyframe
    /// and re-step only the gap.
    pub fn seek_frame(&mut self, frame: i32) {
        self.player.seek_frame(frame);
    }

    /// Restart at frame 0, keeping the keyframe ring. (b2RecPlayer_Restart)
    pub fn restart(&mut self) {
        self.player.restart();
    }

    pub fn is_at_end(&self) -> bool {
        self.player.is_at_end()
    }

    pub fn frame(&self) -> i32 {
        self.player.frame()
    }

    pub fn frame_count(&self) -> i32 {
        self.player.info().frame_count
    }

    pub fn has_diverged(&self) -> bool {
        self.player.has_diverged()
    }

    /// First frame where the replay hash diverged, or -1. (b2RecPlayer_GetDivergeFrame)
    pub fn diverge_frame(&self) -> i32 {
        self.player.diverge_frame()
    }

    /// Current keyframe spacing in frames (the backward-seek granularity).
    pub fn keyframe_interval(&self) -> i32 {
        self.player.keyframe_interval()
    }

    /// Memory held by keyframe snapshots, in kilobytes.
    pub fn keyframe_kilobytes(&self) -> f32 {
        self.player.keyframe_bytes() as f32 / 1024.0
    }

    /// Positions of the replayed bodies in creation (outliner) order:
    /// [x, y, angle] per body. Matches the recording SimWorld's positions()
    /// order because replay reproduces ids deterministically.
    pub fn positions(&self) -> Vec<f32> {
        let world = self.player.world();
        let count = self.player.body_count();
        let mut out = Vec::with_capacity(3 * count as usize);
        for ord in 0..count {
            let id = self.player.body_id(ord);
            if id.is_null() {
                // Destroyed slot: park it far offscreen, ordinals stay stable
                out.push(f32::NAN);
                out.push(f32::NAN);
                out.push(0.0);
                continue;
            }
            let transform = get_body_transform(world, get_body_full_id(world, id));
            out.push(transform.p.x as f32);
            out.push(transform.p.y as f32);
            out.push(m::rot_get_angle(transform.q));
        }
        out
    }

    pub fn awake_body_count(&self) -> i32 {
        self.player.world().solver_sets[box2d_rust::solver_set::AWAKE_SET as usize]
            .body_sims
            .len() as i32
    }

    pub fn contact_count(&self) -> i32 {
        self.player.world().contact_id_pool.id_count()
    }

    pub fn body_count(&self) -> i32 {
        self.player.body_count()
    }

    /// Run `b2World_Draw` on the replayed world into internal buffers.
    pub fn collect_draw(&mut self, lower_x: f32, lower_y: f32, upper_x: f32, upper_y: f32) {
        let collected = collect_world_draw(
            self.player.world_mut(),
            [lower_x, lower_y, upper_x, upper_y],
        );
        self.draw_polygons = collected.polygons;
        self.draw_circles = collected.circles;
        self.draw_capsules = collected.capsules;
        self.draw_lines = collected.lines;
    }

    pub fn draw_polygons(&self) -> Vec<f32> {
        self.draw_polygons.clone()
    }

    pub fn draw_circles(&self) -> Vec<f32> {
        self.draw_circles.clone()
    }

    pub fn draw_capsules(&self) -> Vec<f32> {
        self.draw_capsules.clone()
    }

    pub fn draw_lines(&self) -> Vec<f32> {
        self.draw_lines.clone()
    }
}
