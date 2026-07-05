// Query op family (0xE0-0xE8): the per-call recorder that stands in for the
// C trampolines, and the replay dispatchers that re-run each query against
// the replay world, compare every hit bit-for-bit, and feed the recorded
// user returns back so control flow reproduces.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// See snapshot_structs.rs: assignment order IS the wire order.
#![allow(clippy::field_reassign_with_default)]

use super::ops::read_position;
use super::snapshot::SnapReader;
use super::snapshot_structs::r_vec2;
use super::write::*;
use crate::collision::{Capsule, PlaneResult, WorldCastOutput};
use crate::distance::ShapeProxy;
use crate::dynamic_tree::TreeStats;
use crate::id::ShapeId;
use crate::math_functions::{Aabb, Plane, Pos, Vec2};
use crate::types::{QueryFilter, RayResult};
use crate::world::World;

pub const OP_QUERY_OVERLAP_AABB: u8 = 0xE0;
pub const OP_QUERY_OVERLAP_SHAPE: u8 = 0xE1;
pub const OP_QUERY_CAST_RAY: u8 = 0xE2;
pub const OP_QUERY_CAST_SHAPE: u8 = 0xE3;
pub const OP_QUERY_COLLIDE_MOVER: u8 = 0xE4;
pub const OP_QUERY_CAST_RAY_CLOSEST: u8 = 0xE5;
pub const OP_QUERY_CAST_MOVER: u8 = 0xE6;
pub const OP_SHAPE_TEST_POINT: u8 = 0xE7;
pub const OP_SHAPE_RAY_CAST: u8 = 0xE8;

/// Per-call query recorder: builds the args + reserved hit-count payload in
/// a local buffer, tees each hit, and commits under the op framing. This is
/// the b2RecQueryWriter + trampoline pair, collapsed for the serial port.
/// (b2RecQueryBegin/b2RecQueryCommit)
pub(crate) struct QueryRecorder {
    pub buf: Vec<u8>,
    count_offset: usize,
    pub hits: u32,
    active: bool,
}

impl QueryRecorder {
    /// Inactive when the world has no recording session; all tee/commit
    /// calls become no-ops, keeping the query hot path single-branch like
    /// the C `world->recording != NULL` gate.
    pub fn begin(world: &World, args: impl FnOnce(&mut Vec<u8>)) -> QueryRecorder {
        if world.recording.is_none() {
            return QueryRecorder {
                buf: Vec::new(),
                count_offset: 0,
                hits: 0,
                active: false,
            };
        }

        let mut buf = Vec::new();
        // The world arg is informational; replay targets its own world.
        rec_w_u32(&mut buf, 1);
        args(&mut buf);
        let count_offset = super::rec_reserve_u32(&mut buf);
        QueryRecorder {
            buf,
            count_offset,
            hits: 0,
            active: true,
        }
    }

    pub fn active(&self) -> bool {
        self.active
    }

    /// Patch the hit count, append the optional stats tail, and commit the
    /// framed record.
    pub fn commit(mut self, world: &mut World, opcode: u8, stats: Option<TreeStats>) {
        if !self.active {
            return;
        }
        super::rec_patch_u32(&mut self.buf, self.count_offset, self.hits);
        if let Some(stats) = stats {
            rec_w_treestats(&mut self.buf, stats);
        }
        if let Some(rec) = world.recording.as_mut() {
            rec.commit_record(opcode, &self.buf);
        }
    }
}

/// Self-contained query records (CastRayClosest/CastMover/ShapeTestPoint/
/// ShapeRayCast): args + one result, no hit tail.
pub(crate) fn record_query_result(
    world: &mut World,
    opcode: u8,
    args: impl FnOnce(&mut Vec<u8>),
    result: impl FnOnce(&mut Vec<u8>),
) {
    if let Some(rec) = world.recording.as_mut() {
        let mut buf = Vec::new();
        if opcode != OP_SHAPE_TEST_POINT && opcode != OP_SHAPE_RAY_CAST {
            rec_w_u32(&mut buf, 1);
        }
        args(&mut buf);
        result(&mut buf);
        rec.commit_record(opcode, &buf);
    }
}

// Bitwise comparisons: truncating or epsilon compares would pass vacuously.
// (b2RecF32Differs/b2RecVec2Differs)

fn f32_differs(a: f32, b: f32) -> bool {
    a.to_bits() != b.to_bits()
}

fn vec2_differs(a: Vec2, b: Vec2) -> bool {
    f32_differs(a.x, b.x) || f32_differs(a.y, b.y)
}

fn pos_differs(a: Pos, b: Pos) -> bool {
    // Compare through the full-width delta, like C's b2SubPos against zero.
    vec2_differs(
        crate::math_functions::sub_pos(a, b),
        Vec2 { x: 0.0, y: 0.0 },
    )
}

fn id_differs(a: ShapeId, b: ShapeId) -> bool {
    a.index1 != b.index1 || a.generation != b.generation
}

// Recorded hit tails

struct OverlapHit {
    id: ShapeId,
    user_return: bool,
}

struct CastHit {
    id: ShapeId,
    point: Pos,
    normal: Vec2,
    fraction: f32,
    user_return: f32,
}

struct PlaneHit {
    id: ShapeId,
    plane: PlaneResult,
    user_return: bool,
}

fn r_filter(r: &mut SnapReader) -> QueryFilter {
    let mut filter = crate::types::default_query_filter();
    filter.category_bits = r.r_u64();
    filter.mask_bits = r.r_u64();
    filter
}

fn r_proxy(r: &mut SnapReader) -> ShapeProxy {
    let mut proxy = ShapeProxy::default();
    let count = r.r_i32().clamp(0, crate::hull::MAX_POLYGON_VERTICES as i32);
    proxy.count = count;
    for i in 0..count as usize {
        proxy.points[i] = r_vec2(r);
    }
    proxy.radius = r.r_f32();
    proxy
}

fn r_capsule(r: &mut SnapReader) -> Capsule {
    Capsule {
        center1: r_vec2(r),
        center2: r_vec2(r),
        radius: r.r_f32(),
    }
}

fn r_overlap_hits(r: &mut SnapReader) -> Vec<OverlapHit> {
    let n = r.r_u32() as i32;
    if !r.check_count(n, 9) {
        return Vec::new();
    }
    (0..n)
        .map(|_| OverlapHit {
            id: ShapeId::load(r.r_u64()),
            user_return: r.r_bool(),
        })
        .collect()
}

fn r_cast_hits(r: &mut SnapReader) -> Vec<CastHit> {
    let n = r.r_u32() as i32;
    if !r.check_count(n, 32) {
        return Vec::new();
    }
    (0..n)
        .map(|_| CastHit {
            id: ShapeId::load(r.r_u64()),
            point: read_position(r),
            normal: r_vec2(r),
            fraction: r.r_f32(),
            user_return: r.r_f32(),
        })
        .collect()
}

fn r_treestats(r: &mut SnapReader) -> TreeStats {
    TreeStats {
        node_visits: r.r_i32(),
        leaf_visits: r.r_i32(),
    }
}

/// Dispatch a query-family opcode: read the recorded hits, re-run the query
/// with a comparing trampoline that feeds the recorded user returns back,
/// and flag divergence on any mismatch. Returns None when the opcode is not
/// in this family; Some(matched) otherwise. When a player stash is supplied
/// (rdr->owner in C), the recorded args and hits are stored for per-frame
/// drawing and inspection.
pub(crate) fn dispatch_query_op(
    opcode: u8,
    r: &mut SnapReader,
    world: &mut World,
    mut stash: Option<&mut super::player_queries::QueryStash>,
) -> Option<bool> {
    use crate::shape::{shape_ray_cast, shape_test_point};
    use crate::world::*;

    match opcode {
        OP_QUERY_OVERLAP_AABB => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let aabb = Aabb {
                lower_bound: r_vec2(r),
                upper_bound: r_vec2(r),
            };
            let filter = r_filter(r);
            let hits = r_overlap_hits(r);
            let _stats = r_treestats(r);
            if !r.ok {
                return Some(false);
            }
            let mut cursor = 0usize;
            let mut matched = true;
            world_overlap_aabb(world, origin, aabb, filter, |id| {
                if cursor >= hits.len() {
                    matched = false;
                    return false;
                }
                let h = &hits[cursor];
                cursor += 1;
                if id_differs(id, h.id) {
                    matched = false;
                }
                h.user_return
            });
            if let Some(stash) = stash.as_deref_mut() {
                let pooled = overlap_stash_hits(&hits);
                let q = stash.begin(super::RecQueryType::OverlapAabb, &pooled);
                q.filter = filter;
                q.origin = origin;
                q.aabb = aabb;
            }
            Some(matched && cursor == hits.len())
        }
        OP_QUERY_OVERLAP_SHAPE => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let proxy = r_proxy(r);
            let filter = r_filter(r);
            let hits = r_overlap_hits(r);
            let _stats = r_treestats(r);
            if !r.ok {
                return Some(false);
            }
            let mut cursor = 0usize;
            let mut matched = true;
            world_overlap_shape(world, origin, &proxy, filter, |id| {
                if cursor >= hits.len() {
                    matched = false;
                    return false;
                }
                let h = &hits[cursor];
                cursor += 1;
                if id_differs(id, h.id) {
                    matched = false;
                }
                h.user_return
            });
            if let Some(stash) = stash.as_deref_mut() {
                let pooled = overlap_stash_hits(&hits);
                let q = stash.begin(super::RecQueryType::OverlapShape, &pooled);
                q.filter = filter;
                q.origin = origin;
                q.proxy = proxy;
            }
            Some(matched && cursor == hits.len())
        }
        OP_QUERY_CAST_RAY => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let translation = r_vec2(r);
            let filter = r_filter(r);
            let hits = r_cast_hits(r);
            let _stats = r_treestats(r);
            if !r.ok {
                return Some(false);
            }
            let mut cursor = 0usize;
            let mut matched = true;
            world_cast_ray(
                world,
                origin,
                translation,
                filter,
                |id, point, normal, fraction| {
                    if cursor >= hits.len() {
                        matched = false;
                        return 0.0;
                    }
                    let h = &hits[cursor];
                    cursor += 1;
                    if id_differs(id, h.id)
                        || pos_differs(point, h.point)
                        || vec2_differs(normal, h.normal)
                        || f32_differs(fraction, h.fraction)
                    {
                        matched = false;
                    }
                    h.user_return
                },
            );
            if let Some(stash) = stash.as_deref_mut() {
                let pooled = cast_stash_hits(&hits);
                let q = stash.begin(super::RecQueryType::CastRay, &pooled);
                q.filter = filter;
                q.origin = origin;
                q.translation = translation;
            }
            Some(matched && cursor == hits.len())
        }
        OP_QUERY_CAST_SHAPE => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let proxy = r_proxy(r);
            let translation = r_vec2(r);
            let filter = r_filter(r);
            let hits = r_cast_hits(r);
            let _stats = r_treestats(r);
            if !r.ok {
                return Some(false);
            }
            let mut cursor = 0usize;
            let mut matched = true;
            world_cast_shape(
                world,
                origin,
                &proxy,
                translation,
                filter,
                |id, point, normal, fraction| {
                    if cursor >= hits.len() {
                        matched = false;
                        return 0.0;
                    }
                    let h = &hits[cursor];
                    cursor += 1;
                    if id_differs(id, h.id)
                        || pos_differs(point, h.point)
                        || vec2_differs(normal, h.normal)
                        || f32_differs(fraction, h.fraction)
                    {
                        matched = false;
                    }
                    h.user_return
                },
            );
            if let Some(stash) = stash.as_deref_mut() {
                let pooled = cast_stash_hits(&hits);
                let q = stash.begin(super::RecQueryType::CastShape, &pooled);
                q.filter = filter;
                q.origin = origin;
                q.proxy = proxy;
                q.translation = translation;
            }
            Some(matched && cursor == hits.len())
        }
        OP_QUERY_COLLIDE_MOVER => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let mover = r_capsule(r);
            let filter = r_filter(r);
            // Plane hits: shapeid + plane result + bool return; no stats.
            let n = r.r_u32() as i32;
            if !r.check_count(n, 30) {
                return Some(false);
            }
            let hits: Vec<PlaneHit> = (0..n)
                .map(|_| PlaneHit {
                    id: ShapeId::load(r.r_u64()),
                    plane: PlaneResult {
                        plane: Plane {
                            normal: r_vec2(r),
                            offset: r.r_f32(),
                        },
                        point: r_vec2(r),
                        hit: r.r_bool(),
                    },
                    user_return: r.r_bool(),
                })
                .collect();
            if !r.ok {
                return Some(false);
            }
            let mut cursor = 0usize;
            let mut matched = true;
            world_collide_mover(world, origin, &mover, filter, |id, plane| {
                if cursor >= hits.len() {
                    matched = false;
                    return false;
                }
                let h = &hits[cursor];
                cursor += 1;
                if id_differs(id, h.id)
                    || vec2_differs(plane.plane.normal, h.plane.plane.normal)
                    || f32_differs(plane.plane.offset, h.plane.plane.offset)
                {
                    matched = false;
                }
                h.user_return
            });
            if let Some(stash) = stash.as_deref_mut() {
                let pooled: Vec<super::player_queries::RecordedHit> = hits
                    .iter()
                    .map(|h| super::player_queries::RecordedHit {
                        id: h.id,
                        plane: h.plane,
                        user_return_b: h.user_return,
                        ..Default::default()
                    })
                    .collect();
                let q = stash.begin(super::RecQueryType::CollideMover, &pooled);
                q.filter = filter;
                q.origin = origin;
                q.mover = mover;
            }
            Some(matched && cursor == hits.len())
        }
        OP_QUERY_CAST_RAY_CLOSEST => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let translation = r_vec2(r);
            let filter = r_filter(r);
            let rec = r_ray_result(r);
            if !r.ok {
                return Some(false);
            }
            let got = world_cast_ray_closest(world, origin, translation, filter);
            let matched = got.hit == rec.hit
                && (!got.hit
                    || (!id_differs(got.shape_id, rec.shape_id)
                        && !pos_differs(got.point, rec.point)
                        && !vec2_differs(got.normal, rec.normal)
                        && !f32_differs(got.fraction, rec.fraction)));
            if let Some(stash) = stash.as_deref_mut() {
                // Stash the closest result as a single pooled hit so the
                // shared draw loop renders its point
                let h = super::player_queries::RecordedHit {
                    id: rec.shape_id,
                    point: rec.point,
                    normal: rec.normal,
                    fraction: rec.fraction,
                    ..Default::default()
                };
                let pooled = if rec.hit { vec![h] } else { Vec::new() };
                let q = stash.begin(super::RecQueryType::CastRayClosest, &pooled);
                q.filter = filter;
                q.origin = origin;
                q.translation = translation;
            }
            Some(matched)
        }
        OP_QUERY_CAST_MOVER => {
            let _world = r.r_u32();
            let origin = read_position(r);
            let mover = r_capsule(r);
            let translation = r_vec2(r);
            let filter = r_filter(r);
            let rec = r.r_f32();
            if !r.ok {
                return Some(false);
            }
            let got = world_cast_mover(world, origin, &mover, translation, filter);
            if let Some(stash) = stash.as_deref_mut() {
                let q = stash.begin(super::RecQueryType::CastMover, &[]);
                q.filter = filter;
                q.origin = origin;
                q.mover = mover;
                q.translation = translation;
                q.cast_fraction = rec;
            }
            Some(!f32_differs(got, rec))
        }
        OP_SHAPE_TEST_POINT => {
            let shape = ShapeId::load(r.r_u64());
            let point = read_position(r);
            let rec = r.r_bool();
            if !r.ok {
                return Some(false);
            }
            let got = shape_test_point(world, shape, point);
            if let Some(stash) = stash.as_deref_mut() {
                let q = stash.begin(super::RecQueryType::ShapeTestPoint, &[]);
                q.shape = shape;
                q.origin = point;
                q.bool_result = rec;
            }
            Some(got == rec)
        }
        OP_SHAPE_RAY_CAST => {
            let shape = ShapeId::load(r.r_u64());
            let origin = read_position(r);
            let translation = r_vec2(r);
            let rec = r_world_cast_output(r);
            if !r.ok {
                return Some(false);
            }
            let got = shape_ray_cast(world, shape, origin, translation);
            let matched = got.hit == rec.hit
                && (!got.hit
                    || (!vec2_differs(got.normal, rec.normal)
                        && !pos_differs(got.point, rec.point)
                        && !f32_differs(got.fraction, rec.fraction)));
            if let Some(stash) = stash {
                let q = stash.begin(super::RecQueryType::ShapeRayCast, &[]);
                q.shape = shape;
                // The ray starts at the origin
                q.origin = origin;
                q.translation = translation;
                q.cast_out = rec;
            }
            Some(matched)
        }
        _ => None,
    }
}

// Convert the reader-local hit tails to the pooled player stash form.

fn overlap_stash_hits(hits: &[OverlapHit]) -> Vec<super::player_queries::RecordedHit> {
    hits.iter()
        .map(|h| super::player_queries::RecordedHit {
            id: h.id,
            user_return_b: h.user_return,
            ..Default::default()
        })
        .collect()
}

fn cast_stash_hits(hits: &[CastHit]) -> Vec<super::player_queries::RecordedHit> {
    hits.iter()
        .map(|h| super::player_queries::RecordedHit {
            id: h.id,
            point: h.point,
            normal: h.normal,
            fraction: h.fraction,
            user_return_f: h.user_return,
            ..Default::default()
        })
        .collect()
}

fn r_ray_result(r: &mut SnapReader) -> RayResult {
    let mut result = RayResult::default();
    result.shape_id = ShapeId::load(r.r_u64());
    result.point = read_position(r);
    result.normal = r_vec2(r);
    result.fraction = r.r_f32();
    result.node_visits = r.r_i32();
    result.leaf_visits = r.r_i32();
    result.hit = r.r_bool();
    result
}

fn r_world_cast_output(r: &mut SnapReader) -> WorldCastOutput {
    let mut out = WorldCastOutput::default();
    out.normal = r_vec2(r);
    out.point = read_position(r);
    out.fraction = r.r_f32();
    out.iterations = r.r_i32();
    out.hit = r.r_bool();
    out
}

#[cfg(test)]
mod tests {
    use crate::body::create_body;
    use crate::collision::Capsule;
    use crate::geometry::{make_box, make_square};
    use crate::math_functions::{to_pos, Aabb, Vec2};
    use crate::recording::{replay_buffer, world_start_recording, world_stop_recording, Recording};
    use crate::shape::{create_polygon_shape, shape_ray_cast, shape_test_point};
    use crate::types::{
        default_body_def, default_query_filter, default_shape_def, default_world_def, BodyType,
    };
    use crate::world::*;

    // Every query issued during a recording session is captured with its
    // hits and re-run on replay; the replayed hits must match bit-for-bit.
    #[test]
    fn query_ops_replay() {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let bd = default_body_def();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        let ground_shape = create_polygon_shape(&mut world, ground, &sd, &make_box(20.0, 1.0));

        for i in 0..5 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: -3.0 + 1.5 * i as f32,
                y: 2.5,
            });
            let body = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body, &sd, &make_square(0.4));
        }

        assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

        let filter = default_query_filter();
        for step in 0..40 {
            world_step(&mut world, 1.0 / 60.0, 4);

            // A batch of queries every few steps, mid-simulation.
            if step % 10 == 5 {
                world_overlap_aabb(
                    &mut world,
                    to_pos(Vec2 { x: 0.0, y: 1.5 }),
                    Aabb {
                        lower_bound: Vec2 { x: -3.0, y: -1.0 },
                        upper_bound: Vec2 { x: 3.0, y: 1.0 },
                    },
                    filter,
                    |_| true,
                );
                world_cast_ray(
                    &mut world,
                    to_pos(Vec2 { x: -8.0, y: 1.6 }),
                    Vec2 { x: 16.0, y: 0.0 },
                    filter,
                    |_, _, _, fraction| fraction,
                );
                world_cast_ray_closest(
                    &mut world,
                    to_pos(Vec2 { x: 8.0, y: 2.0 }),
                    Vec2 { x: -16.0, y: 0.0 },
                    filter,
                );
                let mover = Capsule {
                    center1: Vec2 { x: 0.0, y: 0.0 },
                    center2: Vec2 { x: 0.0, y: 0.4 },
                    radius: 0.3,
                };
                world_cast_mover(
                    &mut world,
                    to_pos(Vec2 { x: -6.0, y: 1.8 }),
                    &mover,
                    Vec2 { x: 12.0, y: 0.0 },
                    filter,
                );
                world_collide_mover(
                    &mut world,
                    to_pos(Vec2 { x: 0.0, y: 1.3 }),
                    &mover,
                    filter,
                    |_, _| true,
                );
                shape_test_point(&mut world, ground_shape, to_pos(Vec2 { x: 0.0, y: 0.5 }));
                shape_ray_cast(
                    &mut world,
                    ground_shape,
                    to_pos(Vec2 { x: 0.0, y: 5.0 }),
                    Vec2 { x: 0.0, y: -10.0 },
                );
            }
        }

        let recording = world_stop_recording(&mut world).expect("active session");
        let result = replay_buffer(&recording.buffer);
        assert!(result.ok, "stream parses");
        assert!(
            !result.diverged,
            "queries must re-run with identical hits on replay"
        );
        assert_eq!(result.steps, 40);
    }
}
