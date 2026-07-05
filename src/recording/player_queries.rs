// Player-side query store: the per-frame stash of recorded spatial queries
// and the debug drawing of them (b2RecDrawQuery / b2RecRecordedHit /
// b2RecPlayer_DrawFrameQueries and the public query inspection API).
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{Capsule, PlaneResult, WorldCastOutput};
use crate::debug_draw::{DebugDraw, HexColor};
use crate::distance::ShapeProxy;
use crate::id::ShapeId;
use crate::math_functions::{mul_sv, offset_pos, Aabb, Pos, Vec2, WorldTransform, ROT_IDENTITY};
use crate::shape::{shape_get_aabb, shape_is_valid};
use crate::types::QueryFilter;
use crate::world::World;

/// The kind of a recorded spatial query, matching the public query and cast
/// functions. (b2RecQueryType / b2RecQueryKind — the C internal and public
/// enums are value-identical, so the Rust port keeps one.)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RecQueryType {
    #[default]
    OverlapAabb,
    OverlapShape,
    CastRay,
    CastShape,
    CollideMover,
    CastRayClosest,
    CastMover,
    ShapeTestPoint,
    ShapeRayCast,
}

/// A single recorded callback hit, used both as reader scratch and as the
/// per-frame draw store. (b2RecRecordedHit)
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct RecordedHit {
    pub id: ShapeId,
    pub point: Pos,
    pub normal: Vec2,
    pub fraction: f32,
    pub plane: PlaneResult,
    #[allow(dead_code)] // parity with the C struct; the draw path reads only the fields above
    pub user_return_f: f32,
    #[allow(dead_code)]
    pub user_return_b: bool,
}

/// Per-frame draw record for one query call. (b2RecDrawQuery)
#[derive(Debug, Clone, Default)]
pub(crate) struct DrawQuery {
    pub kind: RecQueryType,
    pub filter: QueryFilter,
    pub aabb: Aabb,
    pub proxy: ShapeProxy,
    pub mover: Capsule,
    pub origin: Pos,
    pub translation: Vec2,
    pub bool_result: bool,
    #[allow(dead_code)] // parity with C; shown by viewers, not the draw path
    pub cast_fraction: f32,
    pub cast_out: WorldCastOutput,
    pub shape: ShapeId,
    pub hit_start: usize,
    pub hit_count: usize,
}

/// The per-frame query store, reset at the top of each StepFrame. Hits for
/// all queries pool into one array like C's frameHits, addressed by each
/// query's hit_start/hit_count window.
#[derive(Default)]
pub(crate) struct QueryStash {
    pub queries: Vec<DrawQuery>,
    pub hits: Vec<RecordedHit>,
}

impl QueryStash {
    pub fn clear(&mut self) {
        self.queries.clear();
        self.hits.clear();
    }

    /// Append a query with its pooled hits and return it for arg fill-in.
    /// (b2RecStashQueryBegin)
    pub fn begin(&mut self, kind: RecQueryType, hits: &[RecordedHit]) -> &mut DrawQuery {
        let hit_start = self.hits.len();
        self.hits.extend_from_slice(hits);
        self.queries.push(DrawQuery {
            kind,
            hit_start,
            hit_count: hits.len(),
            ..DrawQuery::default()
        });
        self.queries.last_mut().unwrap()
    }
}

/// A spatial query recorded during a replayed frame, exposed for inspection.
/// (b2RecQueryInfo)
#[derive(Debug, Clone, Copy, Default)]
pub struct RecQueryInfo {
    pub type_: RecQueryType,
    /// Zeroed for the shape local query types.
    pub filter: QueryFilter,
    /// Overlap AABB, relative to origin.
    pub aabb: Aabb,
    /// Query origin.
    pub origin: Pos,
    /// Ray and cast translation.
    pub translation: Vec2,
    /// Target shape for the shape local query types.
    pub shape: ShapeId,
    /// Number of recorded results.
    pub hit_count: i32,
}

/// One result of a recorded spatial query. (b2RecQueryHit)
#[derive(Debug, Clone, Copy, Default)]
pub struct RecQueryHit {
    pub shape: ShapeId,
    pub point: Pos,
    pub normal: Vec2,
    pub fraction: f32,
}

/// Highlight each reported overlap shape by its AABB. Skip any destroyed
/// since the query, per the shape_get_aabb contract that overlap results may
/// contain stale shapes. (b2RecDrawHitAABBs)
fn draw_hit_aabbs(world: &World, stash: &QueryStash, q: &DrawQuery, draw: &mut dyn DebugDraw) {
    for hit in &stash.hits[q.hit_start..q.hit_start + q.hit_count] {
        if !shape_is_valid(world, hit.id) {
            continue;
        }
        let b = shape_get_aabb(world, hit.id);
        let lower = b.lower_bound;
        let upper = b.upper_bound;
        let vs = [
            lower,
            Vec2 {
                x: upper.x,
                y: lower.y,
            },
            upper,
            Vec2 {
                x: lower.x,
                y: upper.y,
            },
        ];
        draw.draw_polygon(
            crate::math_functions::WORLD_TRANSFORM_IDENTITY,
            &vs,
            HexColor::MAGENTA,
        );
    }
}

/// Draw spatial queries recorded during the most recently replayed frame.
/// `query_index` < 0 draws all queries, otherwise just the one selected in
/// the viewer. The C NULL-callback skips map to the trait's default no-op
/// methods. (b2RecPlayer_DrawFrameQueries)
pub(crate) fn draw_stashed_queries(
    world: &World,
    stash: &QueryStash,
    draw: &mut dyn DebugDraw,
    query_index: i32,
) {
    for (qi, q) in stash.queries.iter().enumerate() {
        if query_index >= 0 && qi as i32 != query_index {
            continue;
        }

        match q.kind {
            RecQueryType::CastRay | RecQueryType::CastRayClosest => {
                // Ray origin to endpoint
                let origin = q.origin;
                let end = offset_pos(origin, q.translation);
                draw.draw_line(origin, end, HexColor::YELLOW);
                // Per-hit point + short normal
                for h in &stash.hits[q.hit_start..q.hit_start + q.hit_count] {
                    let point = h.point;
                    draw.draw_point(point, 4.0, HexColor::YELLOW);
                    let np = offset_pos(point, mul_sv(0.2, h.normal));
                    draw.draw_line(point, np, HexColor::LIGHT_YELLOW);
                }
            }
            RecQueryType::CastShape => {
                // Shape cast: draw per-hit points along the swept path
                for h in &stash.hits[q.hit_start..q.hit_start + q.hit_count] {
                    let point = h.point;
                    draw.draw_point(point, 4.0, HexColor::SKY_BLUE);
                    let np = offset_pos(point, mul_sv(0.2, h.normal));
                    draw.draw_line(point, np, HexColor::LIGHT_SKY_BLUE);
                }
            }
            RecQueryType::CastMover => {
                // The mover capsule is relative to the query origin
                let c1 = offset_pos(q.origin, q.mover.center1);
                let c2 = offset_pos(q.origin, q.mover.center2);
                draw.draw_solid_capsule(c1, c2, q.mover.radius, HexColor::LIGHT_SKY_BLUE);
            }
            RecQueryType::OverlapAabb => {
                // The query box is relative to the query origin
                let lower = q.aabb.lower_bound;
                let upper = q.aabb.upper_bound;
                let vs = [
                    lower,
                    Vec2 {
                        x: upper.x,
                        y: lower.y,
                    },
                    upper,
                    Vec2 {
                        x: lower.x,
                        y: upper.y,
                    },
                ];
                draw.draw_polygon(
                    WorldTransform {
                        p: q.origin,
                        q: ROT_IDENTITY,
                    },
                    &vs,
                    HexColor::LIME_GREEN,
                );
                draw_hit_aabbs(world, stash, q, draw);
            }
            RecQueryType::OverlapShape => {
                // The proxy points are relative to the query origin
                if q.proxy.count == 1 {
                    draw.draw_circle(
                        offset_pos(q.origin, q.proxy.points[0]),
                        q.proxy.radius,
                        HexColor::LIME_GREEN,
                    );
                } else if q.proxy.count >= 2 {
                    draw.draw_polygon(
                        WorldTransform {
                            p: q.origin,
                            q: ROT_IDENTITY,
                        },
                        &q.proxy.points[..q.proxy.count as usize],
                        HexColor::LIME_GREEN,
                    );
                }
                draw_hit_aabbs(world, stash, q, draw);
            }
            RecQueryType::CollideMover => {
                // The mover capsule and the collision planes are relative to
                // the query origin
                let c1 = offset_pos(q.origin, q.mover.center1);
                let c2 = offset_pos(q.origin, q.mover.center2);
                draw.draw_solid_capsule(c1, c2, q.mover.radius, HexColor::TAN);
                // Per-hit plane point and normal
                for h in &stash.hits[q.hit_start..q.hit_start + q.hit_count] {
                    if h.plane.hit {
                        let point = offset_pos(q.origin, h.plane.point);
                        let np = offset_pos(point, mul_sv(0.2, h.plane.plane.normal));
                        draw.draw_line(point, np, HexColor::ORANGE);
                    }
                }
            }
            RecQueryType::ShapeTestPoint => {
                let c = if q.bool_result {
                    HexColor::AQUA
                } else {
                    HexColor::RED
                };
                draw.draw_point(q.origin, 6.0, c);
            }
            RecQueryType::ShapeRayCast => {
                let origin = q.origin;
                let end = offset_pos(origin, q.translation);
                draw.draw_line(origin, end, HexColor::VIOLET);
                if q.cast_out.hit {
                    draw.draw_point(q.cast_out.point, 4.0, HexColor::VIOLET);
                }
            }
        }
    }
}

/// Public query inspection over the stash. (b2RecPlayer_GetFrameQuery)
pub(crate) fn stash_query_info(stash: &QueryStash, index: i32) -> RecQueryInfo {
    if index < 0 || index as usize >= stash.queries.len() {
        return RecQueryInfo::default();
    }
    let q = &stash.queries[index as usize];
    RecQueryInfo {
        type_: q.kind,
        filter: q.filter,
        aabb: q.aabb,
        origin: q.origin,
        translation: q.translation,
        shape: q.shape,
        hit_count: q.hit_count as i32,
    }
}

/// (b2RecPlayer_GetFrameQueryHit)
pub(crate) fn stash_query_hit(stash: &QueryStash, query_index: i32, hit_index: i32) -> RecQueryHit {
    if query_index < 0 || query_index as usize >= stash.queries.len() {
        return RecQueryHit::default();
    }
    let q = &stash.queries[query_index as usize];
    if hit_index < 0 || hit_index as usize >= q.hit_count {
        return RecQueryHit::default();
    }
    let h = &stash.hits[q.hit_start + hit_index as usize];
    RecQueryHit {
        shape: h.id,
        point: h.point,
        normal: h.normal,
        fraction: h.fraction,
    }
}
