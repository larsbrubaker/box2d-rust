//! SimWorld world-query bindings for Cast World / Overlap World
//! (`sample_collision.cpp`).

use wasm_bindgen::prelude::*;

use box2d_rust::distance::{make_proxy, ShapeProxy};
use box2d_rust::id::ShapeId;
use box2d_rust::math_functions::{Pos, Vec2};
use box2d_rust::shape::{shape_get_user_data, shape_set_user_data};
use box2d_rust::types::default_query_filter;
use box2d_rust::world::{
    world_cast_ray, world_cast_ray_closest, world_cast_shape, world_overlap_shape, World,
};

use super::SimWorld;

fn proxy_from_flat(pts: &[f32], radius: f32) -> ShapeProxy {
    let n = pts.len() / 2;
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        points.push(Vec2 {
            x: pts[i * 2],
            y: pts[i * 2 + 1],
        });
    }
    make_proxy(&points, radius)
}

fn shape_demo_index(shapes: &[ShapeId], shape_id: ShapeId) -> i32 {
    for (i, s) in shapes.iter().enumerate() {
        if s.index1 == shape_id.index1 && s.generation == shape_id.generation {
            return i as i32;
        }
    }
    shape_id.index1 - 1
}

/// Read user_data during a world query callback (world is already mutably borrowed).
/// SAFETY: `world` must point at the same live World passed to the query.
unsafe fn user_data_at(world: *const World, shape_id: ShapeId) -> u64 {
    shape_get_user_data(&*world, shape_id)
}

#[wasm_bindgen]
impl SimWorld {
    /// (b2Shape_SetUserData) — Cast/Overlap World ignore flag uses user_data==1.
    pub fn shape_set_user_data(&mut self, shape_index: usize, user_data: u32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let id = self.shapes[shape_index];
        shape_set_user_data(&mut self.world, id, user_data as u64);
    }

    pub fn shape_get_user_data(&self, shape_index: usize) -> u32 {
        if shape_index >= self.shapes.len() {
            return 0;
        }
        shape_get_user_data(&self.world, self.shapes[shape_index]) as u32
    }

    /// (b2World_CastRayClosest). Returns `[hit, fraction, px, py, nx, ny]`.
    pub fn cast_ray_closest(&mut self, ox: f32, oy: f32, tx: f32, ty: f32) -> Vec<f32> {
        let r = world_cast_ray_closest(
            &mut self.world,
            Pos { x: ox, y: oy },
            Vec2 { x: tx, y: ty },
            default_query_filter(),
        );
        vec![
            if r.hit { 1.0 } else { 0.0 },
            r.fraction,
            r.point.x as f32,
            r.point.y as f32,
            r.normal.x,
            r.normal.y,
        ]
    }

    /// (b2World_CastRay) collecting hits. Ignores shapes with user_data==1.
    /// `mode`: 0 any, 1 closest, 2 multiple, 3 sorted.
    /// Returns flat `[count, f,px,py,nx,ny,shapeIdx, …]`.
    pub fn cast_ray_hits(
        &mut self,
        ox: f32,
        oy: f32,
        tx: f32,
        ty: f32,
        mode: i32,
    ) -> Vec<f32> {
        let mut raw: Vec<(f32, f32, f32, f32, f32, ShapeId)> = Vec::new();
        let filter = default_query_filter();
        let world_ptr: *const World = &self.world;

        world_cast_ray(
            &mut self.world,
            Pos { x: ox, y: oy },
            Vec2 { x: tx, y: ty },
            filter,
            |shape_id, point, normal, fraction| {
                let ud = unsafe { user_data_at(world_ptr, shape_id) };
                if ud == 1 {
                    return -1.0;
                }
                match mode {
                    0 => {
                        raw.push((
                            fraction,
                            point.x as f32,
                            point.y as f32,
                            normal.x,
                            normal.y,
                            shape_id,
                        ));
                        0.0
                    }
                    1 => {
                        raw.clear();
                        raw.push((
                            fraction,
                            point.x as f32,
                            point.y as f32,
                            normal.x,
                            normal.y,
                            shape_id,
                        ));
                        fraction
                    }
                    _ => {
                        raw.push((
                            fraction,
                            point.x as f32,
                            point.y as f32,
                            normal.x,
                            normal.y,
                            shape_id,
                        ));
                        1.0
                    }
                }
            },
        );

        if mode == 3 {
            raw.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        }

        let mut out = vec![raw.len() as f32];
        for h in raw {
            let idx = shape_demo_index(&self.shapes, h.5);
            out.extend_from_slice(&[h.0, h.1, h.2, h.3, h.4, idx as f32]);
        }
        out
    }

    /// (b2World_CastShape). Same return layout as cast_ray_hits.
    pub fn cast_shape_hits(
        &mut self,
        ox: f32,
        oy: f32,
        pts: &[f32],
        radius: f32,
        tx: f32,
        ty: f32,
        mode: i32,
    ) -> Vec<f32> {
        let proxy = proxy_from_flat(pts, radius);
        let mut raw: Vec<(f32, f32, f32, f32, f32, ShapeId)> = Vec::new();
        let filter = default_query_filter();
        let world_ptr: *const World = &self.world;

        world_cast_shape(
            &mut self.world,
            Pos { x: ox, y: oy },
            &proxy,
            Vec2 { x: tx, y: ty },
            filter,
            |shape_id, point, normal, fraction| {
                let ud = unsafe { user_data_at(world_ptr, shape_id) };
                if ud == 1 {
                    return -1.0;
                }
                match mode {
                    0 => {
                        raw.push((
                            fraction,
                            point.x as f32,
                            point.y as f32,
                            normal.x,
                            normal.y,
                            shape_id,
                        ));
                        0.0
                    }
                    1 => {
                        raw.clear();
                        raw.push((
                            fraction,
                            point.x as f32,
                            point.y as f32,
                            normal.x,
                            normal.y,
                            shape_id,
                        ));
                        fraction
                    }
                    _ => {
                        raw.push((
                            fraction,
                            point.x as f32,
                            point.y as f32,
                            normal.x,
                            normal.y,
                            shape_id,
                        ));
                        1.0
                    }
                }
            },
        );

        if mode == 3 {
            raw.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        }

        let mut out = vec![raw.len() as f32];
        for h in raw {
            let idx = shape_demo_index(&self.shapes, h.5);
            out.extend_from_slice(&[h.0, h.1, h.2, h.3, h.4, idx as f32]);
        }
        out
    }

    /// (b2World_OverlapShape). Returns demo shape indices (cap 16).
    pub fn overlap_shape_hits(
        &mut self,
        ox: f32,
        oy: f32,
        pts: &[f32],
        radius: f32,
    ) -> Vec<i32> {
        let proxy = proxy_from_flat(pts, radius);
        let mut raw: Vec<ShapeId> = Vec::new();
        let world_ptr: *const World = &self.world;
        world_overlap_shape(
            &mut self.world,
            Pos { x: ox, y: oy },
            &proxy,
            default_query_filter(),
            |shape_id| {
                let ud = unsafe { user_data_at(world_ptr, shape_id) };
                if ud == 1 {
                    return true;
                }
                if raw.len() < 16 {
                    raw.push(shape_id);
                }
                true
            },
        );
        raw.into_iter()
            .map(|id| shape_demo_index(&self.shapes, id))
            .collect()
    }
}
