//! Shape attach helpers with event/filter flags for Events samples
//! (`sample_events.cpp`). Split from `event_ops.rs` for the 800-line limit.

use super::SimWorld;
use box2d_rust::collision::{Capsule, Circle, Segment};
use box2d_rust::geometry::{make_box, make_offset_box, make_rounded_box};
use box2d_rust::math_functions::{make_rot, Vec2};
use box2d_rust::shape::{
    create_capsule_shape, create_circle_shape, create_polygon_shape, create_segment_shape,
};
use box2d_rust::types::{default_shape_def, Filter};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl SimWorld {
    /// Attach box with event/filter flags. Returns demo shape index.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_box_ex(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        cx: f32,
        cy: f32,
        angle: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        is_sensor: bool,
        enable_sensor: bool,
        enable_contact: bool,
        enable_hit: bool,
        enable_presolve: bool,
        category: u32,
        mask: u32,
    ) -> usize {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.is_sensor = is_sensor;
        shape_def.enable_sensor_events = enable_sensor;
        shape_def.enable_contact_events = enable_contact;
        shape_def.enable_hit_events = enable_hit;
        shape_def.enable_pre_solve_events = enable_presolve;
        if category != 0 || mask != 0 {
            shape_def.filter = Filter {
                category_bits: u64::from(category),
                mask_bits: if mask == 0 {
                    box2d_rust::dynamic_tree::DEFAULT_MASK_BITS
                } else {
                    u64::from(mask)
                },
                group_index: 0,
            };
        }
        let polygon = if cx == 0.0 && cy == 0.0 && angle == 0.0 {
            make_box(hx, hy)
        } else {
            make_offset_box(hx, hy, Vec2 { x: cx, y: cy }, make_rot(angle))
        };
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach rounded box (Sensor Bookend solid/sensor). Returns shape index.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_rounded_box_ex(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        radius: f32,
        density: f32,
        is_sensor: bool,
        enable_sensor: bool,
    ) -> usize {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.is_sensor = is_sensor;
        shape_def.enable_sensor_events = enable_sensor;
        let polygon = make_rounded_box(hx, hy, radius);
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach circle with event flags. Returns shape index.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_circle_ex(
        &mut self,
        index: usize,
        cx: f32,
        cy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        rolling: f32,
        is_sensor: bool,
        enable_sensor: bool,
        enable_contact: bool,
        enable_hit: bool,
        category: u32,
        mask: u32,
    ) -> usize {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling;
        shape_def.is_sensor = is_sensor;
        shape_def.enable_sensor_events = enable_sensor;
        shape_def.enable_contact_events = enable_contact;
        shape_def.enable_hit_events = enable_hit;
        if category != 0 || mask != 0 {
            shape_def.filter = Filter {
                category_bits: u64::from(category),
                mask_bits: if mask == 0 {
                    box2d_rust::dynamic_tree::DEFAULT_MASK_BITS
                } else {
                    u64::from(mask)
                },
                group_index: 0,
            };
        }
        let circle = Circle {
            center: Vec2 { x: cx, y: cy },
            radius,
        };
        let sid = create_circle_shape(&mut self.world, body_id, &shape_def, &circle);
        self.track_shape(sid)
    }

    /// Attach capsule with event/filter flags. Returns shape index.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_capsule_ex(
        &mut self,
        index: usize,
        c1x: f32,
        c1y: f32,
        c2x: f32,
        c2y: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        is_sensor: bool,
        enable_sensor: bool,
        enable_contact: bool,
        category: u32,
        mask: u32,
    ) -> usize {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.is_sensor = is_sensor;
        shape_def.enable_sensor_events = enable_sensor;
        shape_def.enable_contact_events = enable_contact;
        if category != 0 || mask != 0 {
            shape_def.filter = Filter {
                category_bits: u64::from(category),
                mask_bits: if mask == 0 {
                    box2d_rust::dynamic_tree::DEFAULT_MASK_BITS
                } else {
                    u64::from(mask)
                },
                group_index: 0,
            };
        }
        let capsule = Capsule {
            center1: Vec2 { x: c1x, y: c1y },
            center2: Vec2 { x: c2x, y: c2y },
            radius,
        };
        let sid = create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
        self.track_shape(sid)
    }

    /// Attach segment with event flags. Returns shape index.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_segment_ex(
        &mut self,
        index: usize,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        is_sensor: bool,
        enable_sensor: bool,
        enable_contact: bool,
        enable_presolve: bool,
        category: u32,
        mask: u32,
    ) -> usize {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = is_sensor;
        shape_def.enable_sensor_events = enable_sensor;
        shape_def.enable_contact_events = enable_contact;
        shape_def.enable_pre_solve_events = enable_presolve;
        if category != 0 || mask != 0 {
            shape_def.filter = Filter {
                category_bits: u64::from(category),
                mask_bits: if mask == 0 {
                    box2d_rust::dynamic_tree::DEFAULT_MASK_BITS
                } else {
                    u64::from(mask)
                },
                group_index: 0,
            };
        }
        let segment = Segment {
            point1: Vec2 { x: x1, y: y1 },
            point2: Vec2 { x: x2, y: y2 },
        };
        let sid = create_segment_shape(&mut self.world, body_id, &shape_def, &segment);
        self.track_shape(sid)
    }

    /// Chain with filter + sensor-event flag (Foot Sensor / Persistent Contact).
    pub fn attach_chain_ex(
        &mut self,
        index: usize,
        points: &[f32],
        is_loop: bool,
        category: u32,
        mask: u32,
        enable_sensor: bool,
        friction: f32,
    ) {
        use box2d_rust::shape::create_chain;
        use box2d_rust::types::{default_chain_def, default_surface_material};

        let body_id = self.body_id_at(index);
        let mut chain_def = default_chain_def();
        chain_def.is_loop = is_loop;
        chain_def.enable_sensor_events = enable_sensor;
        chain_def.points = points
            .chunks_exact(2)
            .map(|p| Vec2 { x: p[0], y: p[1] })
            .collect();
        if category != 0 || mask != 0 {
            chain_def.filter = Filter {
                category_bits: u64::from(category),
                mask_bits: if mask == 0 {
                    box2d_rust::dynamic_tree::DEFAULT_MASK_BITS
                } else {
                    u64::from(mask)
                },
                group_index: 0,
            };
        }
        let mut mat = default_surface_material();
        mat.friction = friction;
        chain_def.materials = vec![mat];
        create_chain(&mut self.world, body_id, &chain_def);
    }
}
