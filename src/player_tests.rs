// Ports of the test_recording.c viewer subtests: the RecPlayer block of
// RecordingTest plus RecordingOutlinerTest, RecordingKeyframeTest,
// RecordingScrubTest, and RecordingQueryScrubTest.
//
// Not ported: RestepRaceTest re-steps at a high worker count hunting
// multithreaded races (the port is serial) and ReplayFileScrubDiag is a
// manual diagnostic over an external file. The C worker-count sweeps
// collapse to single serial runs.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_is_valid, create_body};
use crate::collision::{Circle, Segment};
use crate::debug_draw::{DebugDraw, HexColor};
use crate::geometry::make_box;
use crate::math_functions::{aabb_extents, to_pos, Aabb, Pos, Vec2, WorldTransform};
use crate::recording::{
    hash_world_state_deep, world_start_recording, world_stop_recording, RecPlayer, Recording,
};
use crate::shape::{create_circle_shape, create_polygon_shape, create_segment_shape};
use crate::types::{
    default_body_def, default_query_filter, default_shape_def, default_world_def, BodyType,
};
use crate::world::{
    world_cast_ray, world_get_counters, world_is_valid, world_overlap_aabb, world_step, World,
};

/// Counting draw so the headless draw path provably emits primitives.
#[derive(Default)]
struct CountDraw {
    lines: usize,
    points: usize,
    polygons: usize,
    capsules: usize,
}

impl DebugDraw for CountDraw {
    fn draw_polygon(&mut self, _t: WorldTransform, _v: &[Vec2], _c: HexColor) {
        self.polygons += 1;
    }
    fn draw_line(&mut self, _p1: Pos, _p2: Pos, _c: HexColor) {
        self.lines += 1;
    }
    fn draw_point(&mut self, _p: Pos, _s: f32, _c: HexColor) {
        self.points += 1;
    }
    fn draw_solid_capsule(&mut self, _p1: Pos, _p2: Pos, _r: f32, _c: HexColor) {
        self.capsules += 1;
    }
}

/// Deep hash of a replay world, the ground truth a keyframe seek must
/// reproduce. (ReplayDeepHash)
fn replay_deep_hash(player: &RecPlayer) -> u64 {
    hash_world_state_deep(player.world())
}

/// Record a 60-step session over a bracketed scene (ground circle bottom at
/// y = -20, segment spanning x in [-20, 20]) with queries issued
/// mid-simulation so the player has frame queries to draw.
fn record_bracketed_scene() -> Recording {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

    let mut ground_def = default_body_def();
    ground_def.position = to_pos(Vec2 { x: 0.0, y: -10.0 });
    let ground = create_body(&mut world, &ground_def);
    let sd = default_shape_def();
    create_circle_shape(
        &mut world,
        ground,
        &sd,
        &Circle {
            center: Vec2 { x: 0.0, y: 0.0 },
            radius: 10.0,
        },
    );
    create_segment_shape(
        &mut world,
        ground,
        &sd,
        &Segment {
            point1: Vec2 { x: -20.0, y: 0.0 },
            point2: Vec2 { x: 20.0, y: 0.0 },
        },
    );

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = to_pos(Vec2 { x: 0.0, y: 4.0 });
    let body = create_body(&mut world, &bd);
    let mut ball_def = default_shape_def();
    ball_def.density = 1.0;
    create_circle_shape(
        &mut world,
        body,
        &ball_def,
        &Circle {
            center: Vec2 { x: 0.0, y: 0.0 },
            radius: 0.5,
        },
    );

    let filter = default_query_filter();
    for i in 0..60 {
        world_step(&mut world, 1.0 / 60.0, 4);
        if i % 2 == 0 {
            world_overlap_aabb(
                &mut world,
                to_pos(Vec2 { x: 0.0, y: 2.0 }),
                Aabb {
                    lower_bound: Vec2 { x: -3.0, y: -3.0 },
                    upper_bound: Vec2 { x: 3.0, y: 3.0 },
                },
                filter,
                |_| true,
            );
            world_cast_ray(
                &mut world,
                to_pos(Vec2 { x: -20.0, y: 2.0 }),
                Vec2 { x: 40.0, y: 0.0 },
                filter,
                |_, _, _, fraction| fraction,
            );
        }
    }

    world_stop_recording(&mut world).expect("active session")
}

// The RecPlayer block of RecordingTest: per-frame stepping, restart, the
// getters, and the headless draw path beyond what validate_replay covers.
#[test]
fn recording_player_test() {
    let rec = record_bracketed_scene();
    let player = RecPlayer::create(&rec.buffer);
    let mut player = player.expect("player opens the recording");

    // Recorded bounds frame the whole session, so they must enclose the
    // static ground circle and segment that bracket the scene from x in
    // [-20, 20] down to the bottom of the circle
    let rec_bounds = player.info().bounds;
    let rec_extents = aabb_extents(rec_bounds);
    assert!(rec_extents.x > 0.0 && rec_extents.y > 0.0);
    assert!(rec_bounds.lower_bound.x <= -20.0 && rec_bounds.upper_bound.x >= 20.0);
    assert!(rec_bounds.lower_bound.y <= -20.0);
    assert_eq!(player.info().frame_count, 60);
    assert_eq!(player.info().time_step, 1.0 / 60.0);
    assert_eq!(player.info().sub_step_count, 4);

    // Exercise the draw path on every other frame
    let mut draw = CountDraw::default();
    let mut frames = 0;
    let mut drew_any = 0usize;
    while player.step_frame() {
        if frames % 2 == 0 {
            player.draw_frame_queries(&mut draw, -1);
            drew_any += player.frame_query_count() as usize;
        }
        frames += 1;
    }
    assert_eq!(frames, 60);
    assert_eq!(player.frame(), 60);
    assert!(player.is_at_end());
    assert!(!player.has_diverged());
    assert!(player.is_ok());

    // The queries recorded on even steps landed in the stash and drew
    assert!(drew_any > 0);
    assert!(draw.lines > 0, "ray casts draw lines");
    assert!(draw.polygons > 0, "overlap AABBs draw boxes");

    // The trailing DestroyWorld is an end marker; the world stays valid so a
    // viewer can keep drawing the final step rather than blanking at the end
    assert!(world_is_valid(player.world()));

    // Restart reproduces the same run without reloading the file
    player.restart();
    assert_eq!(player.frame(), 0);
    assert!(!player.is_at_end());

    let mut frames2 = 0;
    while player.step_frame() {
        frames2 += 1;
    }
    assert_eq!(frames2, 60);
    assert!(!player.has_diverged());
}

// Recording started mid-stream snapshots the live world as its seed. Those
// seed bodies are restored as a struct image on replay and never pass
// through the CreateBody hook, so the player must seed its outliner body
// list from the restored world. (RecordingOutlinerTest)
#[test]
fn recording_outliner_test() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    // Build a scene before recording so the bodies live in the seed
    // snapshot, not the op stream
    let ground_def = default_body_def();
    let ground = create_body(&mut world, &ground_def);
    let gsd = default_shape_def();
    create_circle_shape(
        &mut world,
        ground,
        &gsd,
        &Circle {
            center: Vec2 { x: 0.0, y: 0.0 },
            radius: 10.0,
        },
    );

    let dynamic_count = 3;
    for i in 0..dynamic_count {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec2 {
            x: i as f32,
            y: 4.0,
        });
        let body = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_circle_shape(
            &mut world,
            body,
            &sd,
            &Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            },
        );
    }
    let expected_bodies = 1 + dynamic_count;

    // Settle a step, then start recording with the scene already present
    // (non-empty seed)
    world_step(&mut world, 1.0 / 60.0, 4);

    assert!(world_start_recording(&mut world, Recording::new(0)).is_none());
    for _ in 0..10 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }
    let rec = world_stop_recording(&mut world).expect("active session");
    drop(world);

    assert!(!rec.buffer.is_empty());
    let mut player = RecPlayer::create(&rec.buffer).expect("player opens");

    // The outliner list must be populated from the seed snapshot before any
    // frame is stepped, and match the live body count of the restored world
    // (no destroys yet, so no nulled holes)
    let seed_count = player.body_count();
    assert_eq!(seed_count, expected_bodies);
    assert_eq!(seed_count, world_get_counters(player.world()).body_count);

    // Each seeded id is a valid handle into the replay world
    for ord in 0..seed_count {
        assert!(body_is_valid(player.world(), player.body_id(ord)));
    }

    while player.step_frame() {}

    // Restart rolls the outliner list back to its frame-0 seed contents
    player.restart();
    assert_eq!(player.body_count(), seed_count);
}

// A backward seek restores the nearest keyframe and re-steps the gap, so it
// must land on the exact state a linear forward replay would. Compares
// scattered backward and forward seeks against a forward-only deep-hash
// table. A positive budget tightens the keyframe policy on the player under
// test to force repeated budget eviction. (CheckKeyframeSeek)
fn check_keyframe_seek(rec_data: &[u8], budget_bytes: usize, min_interval: i32) {
    // Forward-only reference: a fresh player never seeks backward, so it
    // never restores a keyframe and gives the linear ground truth deep hash
    // at every frame
    let mut reference = RecPlayer::create(rec_data).expect("reference player opens");
    let frame_count = reference.info().frame_count;
    assert!(frame_count > 0);

    let mut ref_hash = vec![replay_deep_hash(&reference)];
    for _ in 1..=frame_count {
        assert!(reference.step_frame());
        ref_hash.push(replay_deep_hash(&reference));
    }
    assert!(!reference.has_diverged());
    drop(reference);

    // Player under test: play to the end so the keyframe ring is fully
    // populated (and an eviction has fired under a tight budget), then seek
    // around it
    let mut player = RecPlayer::create(rec_data).expect("player opens");
    if budget_bytes > 0 {
        player.set_keyframe_policy(budget_bytes, min_interval);
    }
    while player.step_frame() {}
    assert_eq!(player.frame(), frame_count);

    // Targets jump backward and forward: below the first keyframe (1, 5),
    // onto exact interval multiples (128, 256), and around the eviction
    // boundary near frame 272
    let targets = [
        frame_count,
        1,
        frame_count - 1,
        290,
        17,
        271,
        256,
        128,
        33,
        200,
        5,
        300,
        100,
        frame_count,
    ];
    for t in targets {
        let t = t.min(frame_count);
        player.seek_frame(t);
        assert_eq!(player.frame(), t);
        assert!(!player.has_diverged(), "diverged seeking to frame {t}");
        let got = replay_deep_hash(&player);
        assert_eq!(
            got, ref_hash[t as usize],
            "keyframe seek mismatch at frame {t}"
        );
    }
}

// Fast backward seeking via keyframes must be bit-identical to a linear
// replay. Records enough frames to fill the keyframe ring and trigger one
// eviction. (RecordingKeyframeTest)
#[test]
fn recording_keyframe_test() {
    let mut world_def = default_world_def();
    world_def.gravity = Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&world_def);

    // Ground
    let ground_def = default_body_def();
    let ground = create_body(&mut world, &ground_def);
    let gsd = default_shape_def();
    create_polygon_shape(&mut world, ground, &gsd, &make_box(20.0, 1.0));

    // A light stack of dynamic boxes so the world keeps evolving each step
    // (settling, then sleeping) without the cost of joints. The offset start
    // makes the stack topple a little, lengthening the dynamic phase that
    // discriminates a faithful restore.
    for i in 0..8 {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec2 {
            x: 0.05 * i as f32,
            y: 2.0 + 1.1 * i as f32,
        });
        let id = create_body(&mut world, &bd);
        let mut sd = default_shape_def();
        sd.density = 1.0;
        create_polygon_shape(&mut world, id, &sd, &make_box(0.5, 0.5));
    }

    assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

    // 320 steps fills the 16-deep ring (keyframes at 16..256) and fires the
    // eviction at 272
    for _ in 0..320 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("active session");
    drop(world);
    assert!(!rec.buffer.is_empty());

    // Measure one snapshot so the tight budget holds only a handful of
    // keyframes, forcing the interval-doubling eviction during the
    // 320-frame replay
    let probe = RecPlayer::create(&rec.buffer).expect("probe opens");
    let snap_size = crate::recording::world_snapshot(probe.world()).len();
    drop(probe);
    assert!(snap_size > 0);
    let tight_budget = 6 * snap_size;

    // Default policy (capture + restore, no eviction) and a tight budget
    // (forces eviction and the set_keyframe_policy path)
    check_keyframe_seek(&rec.buffer, 0, 0);
    check_keyframe_seek(&rec.buffer, tight_budget, 8);
}

// Build a settling pyramid: heavy stacking contact churn and warm starting,
// the regime where a replay-scrub divergence appears. A small drop gap keeps
// the early steps actively colliding. (BuildScrubPyramid)
fn build_scrub_pyramid(world: &mut World, base_count: i32) {
    let mut bd = default_body_def();
    bd.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
    let ground = create_body(world, &bd);
    let gsd = default_shape_def();
    create_polygon_shape(world, ground, &gsd, &make_box(40.0, 1.0));

    let h = 0.5f32;
    let pitch = 2.0 * h + 0.05;
    let box_poly = make_box(h, h);
    let mut sd = default_shape_def();
    sd.density = 1.0;

    for row in 0..base_count {
        let count = base_count - row;
        let y = h + row as f32 * pitch;
        let x_start = -0.5 * (count - 1) as f32 * pitch;
        for col in 0..count {
            let mut body = default_body_def();
            body.type_ = BodyType::Dynamic;
            body.position = to_pos(Vec2 {
                x: x_start + col as f32 * pitch,
                y,
            });
            let id = create_body(world, &body);
            create_polygon_shape(world, id, &sd, &box_poly);
        }
    }
}

// Issue many-hit spatial queries over the active region. Hit ORDER is the
// broad-phase tree traversal order, which a keyframe restore must reproduce
// or query re-verification flags a divergence. (IssuePileQueries)
fn issue_pile_queries(world: &mut World) {
    let filter = default_query_filter();

    let aabb = Aabb {
        lower_bound: Vec2 { x: -12.0, y: -2.0 },
        upper_bound: Vec2 { x: 12.0, y: 22.0 },
    };
    world_overlap_aabb(world, to_pos(Vec2 { x: 0.0, y: 0.0 }), aabb, filter, |_| {
        true
    });

    // Keep traversing so an all-hits ray reports every shape in pure
    // tree-traversal order
    world_cast_ray(
        world,
        to_pos(Vec2 { x: -12.0, y: 10.0 }),
        Vec2 { x: 24.0, y: 0.0 },
        filter,
        |_, _, _, _| 1.0,
    );
    world_cast_ray(
        world,
        to_pos(Vec2 { x: 0.0, y: 22.0 }),
        Vec2 { x: 0.0, y: -24.0 },
        filter,
        |_, _, _, _| 1.0,
    );
}

// Record step_count frames of a freshly built pyramid. When with_queries is
// set, many-hit queries are issued each step (recorded and re-verified on
// replay), exposing broad-phase traversal-order drift after a keyframe
// restore. (RecordSceneEx)
fn record_pyramid_scene(step_count: i32, with_queries: bool) -> Recording {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);
    build_scrub_pyramid(&mut world, 6);

    assert!(world_start_recording(&mut world, Recording::new(0)).is_none());
    for _ in 0..step_count {
        world_step(&mut world, 1.0 / 60.0, 4);
        if with_queries {
            issue_pile_queries(&mut world);
        }
    }
    world_stop_recording(&mut world).expect("active session")
}

// Drives the actual timeline-scrub path: seek to every frame, walking the
// scrubber from end to start, via the keyframe machinery and compare the
// replay deep hash against a linear forward reference. A tight budget forces
// keyframe eviction so seeks restore from distant keyframes and re-step long
// gaps. (CheckScrubAllFrames)
fn check_scrub_all_frames(rec_data: &[u8], budget_bytes: usize, min_interval: i32) {
    let mut reference = RecPlayer::create(rec_data).expect("reference opens");
    let frame_count = reference.info().frame_count;
    assert!(frame_count > 0);

    let mut ref_hash = vec![replay_deep_hash(&reference)];
    for _ in 1..=frame_count {
        assert!(reference.step_frame());
        ref_hash.push(replay_deep_hash(&reference));
    }
    assert!(!reference.has_diverged());
    drop(reference);

    let mut player = RecPlayer::create(rec_data).expect("player opens");
    if budget_bytes > 0 {
        player.set_keyframe_policy(budget_bytes, min_interval);
    }
    while player.step_frame() {}

    for t in (0..=frame_count).rev() {
        player.seek_frame(t);
        assert_eq!(player.frame(), t);
        let got = replay_deep_hash(&player);
        let diverged = player.has_diverged();
        // A deep-hash match with a divergence flag means an order-sensitive
        // query re-verification failed (broad-phase traversal order), not
        // the simulation state
        assert!(
            got == ref_hash[t as usize] && !diverged,
            "scrub mismatch at frame {t} (budget {budget_bytes}): {} divergence",
            if got == ref_hash[t as usize] {
                "query-order"
            } else {
                "state"
            }
        );
    }
}

// Scrub every frame of a recording at both a tight (eviction, long
// re-steps) and the default keyframe budget. (ScrubRecording)
fn scrub_recording(rec: &Recording) {
    assert!(!rec.buffer.is_empty());

    let probe = RecPlayer::create(&rec.buffer).expect("probe opens");
    let snap_size = crate::recording::world_snapshot(probe.world()).len();
    drop(probe);
    let tight_budget = 4 * snap_size;

    check_scrub_all_frames(&rec.buffer, tight_budget, 8);
    check_scrub_all_frames(&rec.buffer, 0, 0);
}

// Feature coverage for the timeline-scrub path: a backward seek restores a
// keyframe and re-steps the gap, landing on the exact linear-replay state at
// every frame, under the default and a tight (eviction) keyframe budget.
// (RecordingScrubTest)
#[test]
fn recording_scrub_test() {
    let rec = record_pyramid_scene(80, false);
    scrub_recording(&rec);
}

// Feature coverage for query re-verification on scrub: queries issued each
// step are recorded and replayed in order (broad-phase traversal order,
// which the deep state hash does not cover). (RecordingQueryScrubTest)
#[test]
fn recording_query_scrub_test() {
    let rec = record_pyramid_scene(80, true);
    scrub_recording(&rec);
}
