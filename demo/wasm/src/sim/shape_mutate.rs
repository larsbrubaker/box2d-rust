//! Shape filter / morph / wind setters. Split from `shape_ops.rs` for the
//! 800-line limit.

use super::SimWorld;
use box2d_rust::body::{body_apply_mass_from_shapes, body_enable_sleep, body_get_shapes};
use box2d_rust::collision::{Capsule, Circle, Segment};
use box2d_rust::geometry::make_box;
use box2d_rust::math_functions::Vec2;
use box2d_rust::shape::{
    shape_apply_wind, shape_set_capsule, shape_set_circle, shape_set_filter, shape_set_friction,
    shape_set_polygon, shape_set_restitution, shape_set_segment, shape_set_surface_material,
};
use box2d_rust::types::{default_surface_material, Filter, SurfaceMaterial};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl SimWorld {
    /// (b2Shape_SetFilter)
    pub fn shape_set_filter(&mut self, shape_index: usize, category_bits: u32, mask_bits: u32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let sid = self.shapes[shape_index];
        shape_set_filter(
            &mut self.world,
            sid,
            Filter {
                category_bits: u64::from(category_bits),
                mask_bits: u64::from(mask_bits),
                group_index: 0,
            },
        );
    }

    /// C ShapeUserData for mover planes (`sample_character.cpp:13-17`).
    pub fn shape_set_plane_user_data(
        &mut self,
        shape_index: usize,
        max_push: f32,
        clip_velocity: bool,
    ) {
        use box2d_rust::shape::shape_set_user_data;
        if shape_index >= self.shapes.len() {
            return;
        }
        let sid = self.shapes[shape_index];
        shape_set_user_data(
            &mut self.world,
            sid,
            super::mover::pack_plane_user_data(max_push, clip_velocity),
        );
    }

    /// (b2Shape_GetFilter) as [category, mask]
    pub fn shape_get_filter(&self, shape_index: usize) -> Vec<u32> {
        if shape_index >= self.shapes.len() {
            return vec![0, 0];
        }
        let sid = self.shapes[shape_index];
        let f = box2d_rust::shape::shape_get_filter(&self.world, sid);
        vec![f.category_bits as u32, f.mask_bits as u32]
    }

    /// (b2Shape_SetFriction)
    pub fn shape_set_friction(&mut self, shape_index: usize, friction: f32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_set_friction(&mut self.world, self.shapes[shape_index], friction);
    }

    /// (b2Shape_SetRestitution)
    pub fn shape_set_restitution(&mut self, shape_index: usize, restitution: f32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_set_restitution(&mut self.world, self.shapes[shape_index], restitution);
    }

    /// (b2Shape_SetSurfaceMaterial) friction/restitution/rolling/tangent.
    pub fn shape_set_surface(
        &mut self,
        shape_index: usize,
        friction: f32,
        restitution: f32,
        rolling: f32,
        tangent: f32,
    ) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let mut mat = default_surface_material();
        mat.friction = friction;
        mat.restitution = restitution;
        mat.rolling_resistance = rolling;
        mat.tangent_speed = tangent;
        shape_set_surface_material(&mut self.world, self.shapes[shape_index], mat);
    }

    /// Morph shape to circle (Modify Geometry).
    pub fn shape_set_circle(&mut self, shape_index: usize, cx: f32, cy: f32, radius: f32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_set_circle(
            &mut self.world,
            self.shapes[shape_index],
            &Circle {
                center: Vec2 { x: cx, y: cy },
                radius,
            },
        );
    }

    /// Morph shape to capsule.
    pub fn shape_set_capsule(
        &mut self,
        shape_index: usize,
        c1x: f32,
        c1y: f32,
        c2x: f32,
        c2y: f32,
        radius: f32,
    ) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_set_capsule(
            &mut self.world,
            self.shapes[shape_index],
            &Capsule {
                center1: Vec2 { x: c1x, y: c1y },
                center2: Vec2 { x: c2x, y: c2y },
                radius,
            },
        );
    }

    /// Morph shape to segment.
    pub fn shape_set_segment(&mut self, shape_index: usize, x1: f32, y1: f32, x2: f32, y2: f32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_set_segment(
            &mut self.world,
            self.shapes[shape_index],
            &Segment {
                point1: Vec2 { x: x1, y: y1 },
                point2: Vec2 { x: x2, y: y2 },
            },
        );
    }

    /// Morph shape to box polygon (hx, hy).
    pub fn shape_set_box(&mut self, shape_index: usize, hx: f32, hy: f32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let polygon = make_box(hx, hy);
        shape_set_polygon(&mut self.world, self.shapes[shape_index], &polygon);
    }

    /// (b2Body_ApplyMassFromShapes)
    pub fn body_apply_mass_from_shapes(&mut self, index: usize) {
        let body_id = self.body_id_at(index);
        body_apply_mass_from_shapes(&mut self.world, body_id);
    }

    /// (b2Body_EnableSleep)
    pub fn enable_body_sleep(&mut self, index: usize, enable: bool) {
        let body_id = self.body_id_at(index);
        body_enable_sleep(&mut self.world, body_id, enable);
    }

    /// Apply wind to all shapes on a body (Wind sample).
    pub fn apply_wind_to_body(
        &mut self,
        index: usize,
        wx: f32,
        wy: f32,
        drag: f32,
        lift: f32,
        wake: bool,
    ) {
        let body_id = self.body_id_at(index);
        let shapes = body_get_shapes(&self.world, body_id, 8);
        for sid in shapes {
            shape_apply_wind(
                &mut self.world,
                sid,
                Vec2 { x: wx, y: wy },
                drag,
                lift,
                wake,
            );
        }
    }

    /// Attach an open/loop chain to an existing body (Chain Link).
    pub fn attach_chain(&mut self, index: usize, points: &[f32], is_loop: bool) {
        use box2d_rust::shape::create_chain;
        use box2d_rust::types::default_chain_def;

        let body_id = self.body_id_at(index);
        let mut chain_def = default_chain_def();
        chain_def.is_loop = is_loop;
        chain_def.points = points
            .chunks_exact(2)
            .map(|p| Vec2 { x: p[0], y: p[1] })
            .collect();
        create_chain(&mut self.world, body_id, &chain_def);
    }

    /// Chain with per-point materials. `mats` is interleaved
    /// [friction, restitution, rolling, tangent] * N (N == point count or 1).
    /// Returns demo chain index for `chain_set_surface`.
    pub fn add_chain_mat(&mut self, points: &[f32], is_loop: bool, mats: &[f32]) -> usize {
        use box2d_rust::body::create_body;
        use box2d_rust::shape::create_chain;
        use box2d_rust::types::{default_body_def, default_chain_def};

        let body_def = default_body_def();
        let body_id = create_body(&mut self.world, &body_def);
        let mut chain_def = default_chain_def();
        chain_def.is_loop = is_loop;
        chain_def.points = points
            .chunks_exact(2)
            .map(|p| Vec2 { x: p[0], y: p[1] })
            .collect();
        let mat_count = mats.len() / 4;
        if mat_count > 0 {
            chain_def.materials = mats
                .chunks_exact(4)
                .map(|m| SurfaceMaterial {
                    friction: m[0],
                    restitution: m[1],
                    rolling_resistance: m[2],
                    tangent_speed: m[3],
                    user_material_id: 0,
                    custom_color: 0,
                })
                .collect();
        }
        let chain_id = create_chain(&mut self.world, body_id, &chain_def);
        self.track_body(body_id);
        self.track_chain(chain_id)
    }

    /// (b2Chain_SetSurfaceMaterial) — update material slot on a tracked chain.
    pub fn chain_set_surface(
        &mut self,
        chain_index: usize,
        friction: f32,
        restitution: f32,
        rolling: f32,
        tangent: f32,
        material_index: usize,
    ) {
        use box2d_rust::shape::chain_set_surface_material;
        use box2d_rust::types::default_surface_material;

        if chain_index >= self.chains.len() {
            return;
        }
        let mut mat = default_surface_material();
        mat.friction = friction;
        mat.restitution = restitution;
        mat.rolling_resistance = rolling;
        mat.tangent_speed = tangent;
        chain_set_surface_material(
            &mut self.world,
            self.chains[chain_index],
            mat,
            material_index,
        );
    }
}
