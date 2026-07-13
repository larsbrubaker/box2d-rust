//! Benchmark Cast / Shape Distance query helpers (`sample_benchmark.cpp`).

use wasm_bindgen::prelude::*;

use box2d_rust::distance::{make_proxy, ShapeProxy};
use box2d_rust::id::ShapeId;
use box2d_rust::math_functions::{aabb_center, Aabb, Pos, Vec2, VEC2_ZERO};
use box2d_rust::shape::shape_get_aabb;
use box2d_rust::types::{default_query_filter, QueryFilter, RayResult};
use box2d_rust::world::{
    world_cast_ray_closest, world_cast_shape, world_overlap_aabb, world_rebuild_static_tree,
};

use super::SimWorld;

fn mask_filter(mask_bits: u32) -> QueryFilter {
    let mut f = default_query_filter();
    f.mask_bits = u64::from(mask_bits);
    f
}

#[wasm_bindgen]
impl SimWorld {
    /// (b2World_RebuildStaticTree) — Benchmark Cast "top down".
    pub fn rebuild_static_tree(&mut self) {
        world_rebuild_static_tree(&mut self.world);
    }

    /// Closest ray with mask. Returns
    /// `[hit, px, py, nx, ny, fraction, nodeVisits, leafVisits]`.
    pub fn cast_ray_closest_mask(
        &mut self,
        ox: f32,
        oy: f32,
        tx: f32,
        ty: f32,
        mask_bits: u32,
    ) -> Vec<f32> {
        let r: RayResult = world_cast_ray_closest(
            &mut self.world,
            Pos { x: ox, y: oy },
            Vec2 { x: tx, y: ty },
            mask_filter(mask_bits),
        );
        vec![
            if r.hit { 1.0 } else { 0.0 },
            r.point.x as f32,
            r.point.y as f32,
            r.normal.x,
            r.normal.y,
            r.fraction,
            r.node_visits as f32,
            r.leaf_visits as f32,
        ]
    }

    /// Circle shape cast (proxy radius, one point at origin). Closest-hit callback.
    /// Returns `[hit, px, py, fraction, nodeVisits, leafVisits]`.
    pub fn cast_circle_closest_mask(
        &mut self,
        ox: f32,
        oy: f32,
        radius: f32,
        tx: f32,
        ty: f32,
        mask_bits: u32,
    ) -> Vec<f32> {
        let proxy: ShapeProxy = make_proxy(&[VEC2_ZERO], radius);
        let mut hit = false;
        let mut point = Pos { x: 0.0, y: 0.0 };
        let mut fraction = 1.0f32;
        let stats = world_cast_shape(
            &mut self.world,
            Pos { x: ox, y: oy },
            &proxy,
            Vec2 { x: tx, y: ty },
            mask_filter(mask_bits),
            |_, p, _n, f| {
                hit = true;
                point = p;
                fraction = f;
                f
            },
        );
        vec![
            if hit { 1.0 } else { 0.0 },
            point.x as f32,
            point.y as f32,
            fraction,
            stats.node_visits as f32,
            stats.leaf_visits as f32,
        ]
    }

    /// Overlap AABB centered at origin with half-extents (hx, hy).
    /// Returns `[nodeVisits, leafVisits, count, cx0, cy0, …]` (centers, cap 32).
    pub fn overlap_aabb_centers_mask(
        &mut self,
        ox: f32,
        oy: f32,
        hx: f32,
        hy: f32,
        mask_bits: u32,
    ) -> Vec<f32> {
        let aabb = Aabb {
            lower_bound: Vec2 { x: -hx, y: -hy },
            upper_bound: Vec2 { x: hx, y: hy },
        };
        let mut centers: Vec<Vec2> = Vec::new();
        let world_ptr: *const box2d_rust::world::World = &self.world;
        let stats = world_overlap_aabb(
            &mut self.world,
            Pos { x: ox, y: oy },
            aabb,
            mask_filter(mask_bits),
            |shape_id: ShapeId| {
                if centers.len() < 32 {
                    // SAFETY: world lives for the duration of the query.
                    let aabb = unsafe { shape_get_aabb(&*world_ptr, shape_id) };
                    centers.push(aabb_center(aabb));
                }
                true
            },
        );
        let mut out = vec![
            stats.node_visits as f32,
            stats.leaf_visits as f32,
            centers.len() as f32,
        ];
        for c in centers {
            out.push(c.x);
            out.push(c.y);
        }
        out
    }
}
