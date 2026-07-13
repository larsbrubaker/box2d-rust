//! Shape material / filter / morph / wind APIs for Shapes samples.

use super::SimWorld;
use box2d_rust::body::{body_apply_mass_from_shapes, body_enable_sleep, body_get_shapes};
use box2d_rust::collision::{Capsule, ChainSegment, Circle, Segment};
use box2d_rust::geometry::{make_box, make_polygon};
use box2d_rust::hull::compute_hull;
use box2d_rust::id::ShapeId;
use box2d_rust::math_functions::Vec2;
use box2d_rust::shape::{
    create_chain_segment_shape, shape_apply_wind, shape_set_capsule, shape_set_chain_segment,
    shape_set_circle, shape_set_filter, shape_set_friction, shape_set_polygon, shape_set_restitution,
    shape_set_segment, shape_set_surface_material,
};
use box2d_rust::types::{default_shape_def, default_surface_material, Filter, SurfaceMaterial};
use box2d_rust::world::world_set_custom_filter_callback;
use std::cell::Cell;
use wasm_bindgen::prelude::*;

// CustomFilterFcn only receives ShapeIds + u64 context — stash a world pointer
// for the odd/even Custom Filter sample (set while that scene is active).
thread_local! {
    static FILTER_WORLD: Cell<*mut box2d_rust::world::World> = const { Cell::new(std::ptr::null_mut()) };
    /// Benchmark Sensor filter row (`sample_benchmark.cpp` Filter).
    static SENSOR_FILTER_ROW: Cell<i32> = const { Cell::new(0) };
    /// 0 = none, 1 = odd/even, 2 = sensor-row.
    static FILTER_MODE: Cell<u8> = const { Cell::new(0) };
}

fn odd_even_filter_live(shape_a: ShapeId, shape_b: ShapeId, _ctx: u64) -> bool {
    FILTER_WORLD.with(|cell| {
        let ptr = cell.get();
        if ptr.is_null() {
            return true;
        }
        // SAFETY: pointer set only while SimWorld lives and odd/even filter is on.
        let world = unsafe { &*ptr };
        let a = box2d_rust::shape::shape_get_user_data(world, shape_a) as i32;
        let b = box2d_rust::shape::shape_get_user_data(world, shape_b) as i32;
        if a == 0 || b == 0 {
            return true;
        }
        ((a & 1) + (b & 1)) != 1
    })
}

/// Pack `ShapeUserData { row, active }` into shape user_data bits.
pub(crate) fn pack_sensor_ud(row: i32, active: bool) -> u64 {
    ((row as u64) << 1) | u64::from(active)
}

fn unpack_sensor_ud(ud: u64) -> (i32, bool) {
    ((ud >> 1) as i32, (ud & 1) != 0)
}

/// Benchmark Sensor custom filter (`sample_benchmark.cpp:1984-2001`).
fn sensor_row_filter_live(shape_a: ShapeId, shape_b: ShapeId, _ctx: u64) -> bool {
    FILTER_WORLD.with(|cell| {
        let ptr = cell.get();
        if ptr.is_null() {
            return true;
        }
        // SAFETY: pointer set only while SimWorld lives and sensor filter is on.
        let world = unsafe { &*ptr };
        use box2d_rust::shape::{shape_get_user_data, shape_is_sensor};
        let ud = if shape_is_sensor(world, shape_a) {
            shape_get_user_data(world, shape_a)
        } else if shape_is_sensor(world, shape_b) {
            shape_get_user_data(world, shape_b)
        } else {
            return true;
        };
        let (row, active) = unpack_sensor_ud(ud);
        active || row != SENSOR_FILTER_ROW.get()
    })
}

impl SimWorld {
    pub(crate) fn track_shape(&mut self, shape_id: ShapeId) -> usize {
        self.shapes.push(shape_id);
        self.shapes.len() - 1
    }

    #[allow(dead_code)]
    pub(crate) fn shape_id_at(&self, index: usize) -> ShapeId {
        self.shapes[index]
    }

    pub(crate) fn refresh_filter_world_ptr(&mut self) {
        FILTER_WORLD.with(|cell| {
            cell.set(&mut self.world as *mut _);
        });
    }
}

#[wasm_bindgen]
impl SimWorld {
    /// Attach box with full surface material. Returns demo shape index.
    pub fn attach_box_mat(
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
        rolling: f32,
        tangent: f32,
    ) -> usize {
        use box2d_rust::geometry::make_offset_box;
        use box2d_rust::math_functions::make_rot;
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling;
        shape_def.material.tangent_speed = tangent;
        let polygon = if cx == 0.0 && cy == 0.0 && angle == 0.0 {
            make_box(hx, hy)
        } else {
            make_offset_box(hx, hy, Vec2 { x: cx, y: cy }, make_rot(angle))
        };
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach circle with full surface material. Returns demo shape index.
    pub fn attach_circle_mat(
        &mut self,
        index: usize,
        cx: f32,
        cy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        rolling: f32,
        tangent: f32,
    ) -> usize {
        use box2d_rust::shape::create_circle_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling;
        shape_def.material.tangent_speed = tangent;
        let circle = Circle {
            center: Vec2 { x: cx, y: cy },
            radius,
        };
        let sid = create_circle_shape(&mut self.world, body_id, &shape_def, &circle);
        self.track_shape(sid)
    }

    /// Attach capsule with full surface material. Returns demo shape index.
    pub fn attach_capsule_mat(
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
        rolling: f32,
        tangent: f32,
    ) -> usize {
        use box2d_rust::shape::create_capsule_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling;
        shape_def.material.tangent_speed = tangent;
        let capsule = Capsule {
            center1: Vec2 { x: c1x, y: c1y },
            center2: Vec2 { x: c2x, y: c2y },
            radius,
        };
        let sid = create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
        self.track_shape(sid)
    }

    /// Attach polygon with full surface material. Returns demo shape index.
    pub fn attach_polygon_mat(
        &mut self,
        index: usize,
        points: &[f32],
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        rolling: f32,
        tangent: f32,
    ) -> usize {
        use box2d_rust::shape::create_polygon_shape;

        let verts: Vec<Vec2> = points
            .chunks_exact(2)
            .map(|p| Vec2 { x: p[0], y: p[1] })
            .collect();
        let hull = compute_hull(&verts);
        let polygon = make_polygon(&hull, radius);
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling;
        shape_def.material.tangent_speed = tangent;
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach rounded box with material. Returns demo shape index.
    pub fn attach_rounded_box_mat(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        rolling: f32,
        tangent: f32,
    ) -> usize {
        use box2d_rust::geometry::make_rounded_box;
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling;
        shape_def.material.tangent_speed = tangent;
        let polygon = make_rounded_box(hx, hy, radius);
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach box with collision filter bits. Returns demo shape index.
    pub fn attach_box_filter(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        density: f32,
        category_bits: u32,
        mask_bits: u32,
    ) -> usize {
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.filter = Filter {
            category_bits: u64::from(category_bits),
            mask_bits: u64::from(mask_bits),
            group_index: 0,
        };
        let polygon = make_box(hx, hy);
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach segment with filter. Returns demo shape index.
    pub fn attach_segment_filter(
        &mut self,
        index: usize,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        category_bits: u32,
        mask_bits: u32,
    ) -> usize {
        use box2d_rust::shape::create_segment_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.filter = Filter {
            category_bits: u64::from(category_bits),
            mask_bits: u64::from(mask_bits),
            group_index: 0,
        };
        let segment = Segment {
            point1: Vec2 { x: x1, y: y1 },
            point2: Vec2 { x: x2, y: y2 },
        };
        let sid = create_segment_shape(&mut self.world, body_id, &shape_def, &segment);
        self.track_shape(sid)
    }

    /// Attach segment with surface friction (Friction sample ground).
    pub fn attach_segment_mat(
        &mut self,
        index: usize,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        friction: f32,
    ) -> usize {
        use box2d_rust::shape::create_segment_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.material.friction = friction;
        let segment = Segment {
            point1: Vec2 { x: x1, y: y1 },
            point2: Vec2 { x: x2, y: y2 },
        };
        let sid = create_segment_shape(&mut self.world, body_id, &shape_def, &segment);
        self.track_shape(sid)
    }

    /// Attach segment with invokeContactCreation (Recreate Static). Returns shape index.
    pub fn attach_segment_invoke(
        &mut self,
        index: usize,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
    ) -> usize {
        use box2d_rust::shape::create_segment_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.invoke_contact_creation = true;
        let segment = Segment {
            point1: Vec2 { x: x1, y: y1 },
            point2: Vec2 { x: x2, y: y2 },
        };
        let sid = create_segment_shape(&mut self.world, body_id, &shape_def, &segment);
        self.track_shape(sid)
    }

    /// Attach box with custom-filter flag + user_data (Custom Filter). Returns shape index.
    pub fn attach_box_custom(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        density: f32,
        user_data: u32,
    ) -> usize {
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.enable_custom_filtering = true;
        shape_def.user_data = u64::from(user_data);
        let polygon = make_box(hx, hy);
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Attach a chain-segment (ghost1, p1, p2, ghost2). Returns shape index.
    pub fn attach_chain_segment(
        &mut self,
        index: usize,
        g1x: f32,
        g1y: f32,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        g2x: f32,
        g2y: f32,
    ) -> usize {
        let body_id = self.body_id_at(index);
        let shape_def = default_shape_def();
        let cs = ChainSegment {
            ghost1: Vec2 { x: g1x, y: g1y },
            segment: Segment {
                point1: Vec2 { x: x1, y: y1 },
                point2: Vec2 { x: x2, y: y2 },
            },
            ghost2: Vec2 { x: g2x, y: g2y },
            chain_id: -1,
        };
        let sid = create_chain_segment_shape(&mut self.world, body_id, &shape_def, &cs);
        self.track_shape(sid)
    }

    /// Update a chain segment's geometry (Chain Segment Mutate).
    pub fn shape_set_chain_segment(
        &mut self,
        shape_index: usize,
        g1x: f32,
        g1y: f32,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        g2x: f32,
        g2y: f32,
    ) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let sid = self.shapes[shape_index];
        let cs = ChainSegment {
            ghost1: Vec2 { x: g1x, y: g1y },
            segment: Segment {
                point1: Vec2 { x: x1, y: y1 },
                point2: Vec2 { x: x2, y: y2 },
            },
            ghost2: Vec2 { x: g2x, y: g2y },
            chain_id: -1,
        };
        shape_set_chain_segment(&mut self.world, sid, &cs);
    }

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
    pub fn shape_set_segment(
        &mut self,
        shape_index: usize,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
    ) {
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
        create_chain(&mut self.world, body_id, &chain_def);
        self.track_body(body_id)
    }

    /// Install C Custom Filter odd/even rule (user_data indices).
    pub fn enable_odd_even_filter(&mut self, enabled: bool) {
        self.refresh_filter_world_ptr();
        if enabled {
            FILTER_MODE.set(1);
            world_set_custom_filter_callback(&mut self.world, Some(odd_even_filter_live), 0);
        } else {
            FILTER_MODE.set(0);
            world_set_custom_filter_callback(&mut self.world, None, 0);
            FILTER_WORLD.with(|cell| cell.set(std::ptr::null_mut()));
        }
    }

    /// Benchmark Sensor custom filter — packs row/active in shape user_data.
    /// (`sample_benchmark.cpp` FilterFcn).
    pub fn enable_sensor_row_filter(&mut self, enabled: bool, filter_row: i32) {
        self.refresh_filter_world_ptr();
        SENSOR_FILTER_ROW.set(filter_row);
        if enabled {
            FILTER_MODE.set(2);
            world_set_custom_filter_callback(&mut self.world, Some(sensor_row_filter_live), 0);
        } else {
            FILTER_MODE.set(0);
            world_set_custom_filter_callback(&mut self.world, None, 0);
            FILTER_WORLD.with(|cell| cell.set(std::ptr::null_mut()));
        }
    }

    /// Offset polygon sensor with user_data + optional custom filter + color.
    /// Returns demo shape index. (`BenchmarkSensor` ground sensors).
    #[allow(clippy::too_many_arguments)]
    pub fn attach_sensor_ud(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        cx: f32,
        cy: f32,
        angle: f32,
        radius: f32,
        is_sensor: bool,
        enable_sensor: bool,
        enable_custom_filter: bool,
        user_data: u32,
        custom_color: u32,
    ) -> usize {
        use box2d_rust::geometry::{make_offset_box, make_offset_rounded_box};
        use box2d_rust::math_functions::make_rot;
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = is_sensor;
        shape_def.enable_sensor_events = enable_sensor;
        shape_def.enable_custom_filtering = enable_custom_filter;
        shape_def.user_data = u64::from(user_data);
        shape_def.material.custom_color = custom_color;
        let polygon = if radius > 0.0 {
            make_offset_rounded_box(hx, hy, Vec2 { x: cx, y: cy }, make_rot(angle), radius)
        } else {
            make_offset_box(hx, hy, Vec2 { x: cx, y: cy }, make_rot(angle))
        };
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// (b2Shape_SetSurfaceMaterial.customColor) — Sensor visitor highlight.
    pub fn shape_set_custom_color(&mut self, shape_index: usize, custom_color: u32) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let mut mat = box2d_rust::shape::shape_get_surface_material(
            &self.world,
            self.shapes[shape_index],
        );
        mat.custom_color = custom_color;
        shape_set_surface_material(&mut self.world, self.shapes[shape_index], mat);
    }

    /// Pack row/active for Benchmark Sensor shape user_data.
    pub fn pack_sensor_user_data(&self, row: i32, active: bool) -> u32 {
        let _ = self;
        pack_sensor_ud(row, active) as u32
    }

    /// Static/dynamic box with category + custom color (Benchmark Cast grid).
    pub fn attach_box_category_color(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        category_bits: u32,
        custom_color: u32,
    ) -> usize {
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.filter = Filter {
            category_bits: u64::from(category_bits),
            mask_bits: box2d_rust::dynamic_tree::DEFAULT_MASK_BITS,
            group_index: 0,
        };
        shape_def.material.custom_color = custom_color;
        let polygon = make_box(hx, hy);
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Chain with a single custom color (Gear Lift ground).
    pub fn add_chain_color(
        &mut self,
        points: &[f32],
        is_loop: bool,
        friction: f32,
        custom_color: u32,
    ) -> usize {
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
        chain_def.materials = vec![SurfaceMaterial {
            friction,
            restitution: 0.0,
            rolling_resistance: 0.0,
            tangent_speed: 0.0,
            user_material_id: 0,
            custom_color,
        }];
        create_chain(&mut self.world, body_id, &chain_def);
        self.track_body(body_id)
    }

    /// Offset rounded box with color (Gear Lift teeth). Returns shape index.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_offset_rounded_box_color(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        cx: f32,
        cy: f32,
        angle: f32,
        radius: f32,
        density: f32,
        friction: f32,
        custom_color: u32,
    ) -> usize {
        use box2d_rust::geometry::make_offset_rounded_box;
        use box2d_rust::math_functions::make_rot;
        use box2d_rust::shape::create_polygon_shape;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.custom_color = custom_color;
        let polygon = make_offset_rounded_box(
            hx,
            hy,
            Vec2 { x: cx, y: cy },
            make_rot(angle),
            radius,
        );
        let sid = create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_shape(sid)
    }

    /// Body with sleep threshold (Scissor Lift dynamics).
    pub fn add_body_sleep_threshold(
        &mut self,
        x: f32,
        y: f32,
        angle: f32,
        body_type: i32,
        sleep_threshold: f32,
    ) -> usize {
        use box2d_rust::body::create_body;
        use box2d_rust::math_functions::{make_rot, to_pos};
        use box2d_rust::types::{default_body_def, BodyType};

        let mut body_def = default_body_def();
        body_def.type_ = match body_type {
            1 => BodyType::Kinematic,
            2 => BodyType::Dynamic,
            _ => BodyType::Static,
        };
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        body_def.sleep_threshold = sleep_threshold;
        let body_id = create_body(&mut self.world, &body_def);
        self.track_body(body_id)
    }

    /// Set weld/revolute local frame A angle (Explosion spinning welds).
    pub fn joint_set_frame_angle_a(&mut self, joint_index: usize, angle: f32) {
        use box2d_rust::joint::{joint_get_local_frame_a, joint_set_local_frame_a};
        use box2d_rust::math_functions::make_rot;

        if joint_index >= self.joints.len() {
            return;
        }
        let jid = self.joints[joint_index];
        let mut frame = joint_get_local_frame_a(&self.world, jid);
        frame.q = make_rot(angle);
        joint_set_local_frame_a(&mut self.world, jid, frame);
    }

    /// Body with gravity scale + sleep flag (Wind).
    pub fn add_body_ex(
        &mut self,
        x: f32,
        y: f32,
        angle: f32,
        body_type: i32,
        gravity_scale: f32,
        enable_sleep: bool,
    ) -> usize {
        use box2d_rust::body::create_body;
        use box2d_rust::math_functions::{make_rot, to_pos};
        use box2d_rust::types::{default_body_def, BodyType};

        let mut body_def = default_body_def();
        body_def.type_ = match body_type {
            1 => BodyType::Kinematic,
            2 => BodyType::Dynamic,
            _ => BodyType::Static,
        };
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        body_def.gravity_scale = gravity_scale;
        body_def.enable_sleep = enable_sleep;
        let body_id = create_body(&mut self.world, &body_def);
        self.track_body(body_id)
    }

    /// Dynamic/static body with CCD flags (Bounce House / Pinball / Drop Scene4).
    /// (b2BodyDef.isBullet + allowFastRotation + gravityScale + enableSleep)
    #[allow(clippy::too_many_arguments)]
    pub fn add_body_ccd(
        &mut self,
        x: f32,
        y: f32,
        angle: f32,
        body_type: i32,
        gravity_scale: f32,
        is_bullet: bool,
        allow_fast_rotation: bool,
        enable_sleep: bool,
    ) -> usize {
        use box2d_rust::body::create_body;
        use box2d_rust::math_functions::{make_rot, to_pos};
        use box2d_rust::types::{default_body_def, BodyType};

        let mut body_def = default_body_def();
        body_def.type_ = match body_type {
            1 => BodyType::Kinematic,
            2 => BodyType::Dynamic,
            _ => BodyType::Static,
        };
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        body_def.gravity_scale = gravity_scale;
        body_def.is_bullet = is_bullet;
        body_def.allow_fast_rotation = allow_fast_rotation;
        body_def.enable_sleep = enable_sleep;
        let body_id = create_body(&mut self.world, &body_def);
        self.track_body(body_id)
    }
}
