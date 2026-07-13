//! Event query / sensor / contact attach APIs for Events samples
//! (`sample_events.cpp`).

use super::SimWorld;
use box2d_rust::body::{
    body_apply_mass_from_shapes, body_get_shapes, body_get_transform, body_get_world_center,
    body_set_name, body_set_user_data, get_body_full_id, make_body_id,
};
use box2d_rust::collision::{Capsule, Circle, Segment, ShapeType};
use box2d_rust::core::NULL_INDEX;
use box2d_rust::geometry::{make_box, make_offset_box, make_rounded_box, transform_polygon};
use box2d_rust::id::ShapeId;
use box2d_rust::joint::{destroy_joint, joint_is_valid, joint_set_user_data};
use box2d_rust::math_functions::{
    inv_mul_world_transforms, make_rot, to_pos, transform_point, Vec2,
};
use box2d_rust::shape::{
    create_capsule_shape, create_circle_shape, create_polygon_shape, create_segment_shape,
    destroy_shape, shape_are_sensor_events_enabled, shape_enable_contact_events,
    shape_enable_hit_events, shape_enable_pre_solve_events, shape_enable_sensor_events,
    shape_get_aabb, shape_get_body, shape_get_capsule, shape_get_circle, shape_get_polygon,
    shape_get_sensor_data, shape_get_type, shape_is_valid,
};
use box2d_rust::solver_set::AWAKE_SET;
use box2d_rust::types::{default_query_filter, default_shape_def, Filter};
use box2d_rust::world::{
    world_cast_ray_closest, world_get_body_events, world_get_contact_events,
    world_get_joint_events, world_get_sensor_events, world_set_pre_solve_callback, PreSolveFcn,
};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

thread_local! {
    /// Platformer one-way platform: player shape index1 (0 = unset).
    static PLATFORMER_PLAYER_SHAPE: Cell<i32> = const { Cell::new(0) };
}

fn platformer_presolve(
    shape_a: ShapeId,
    shape_b: ShapeId,
    _point: box2d_rust::math_functions::Pos,
    normal: Vec2,
    _ctx: u64,
) -> bool {
    let player = PLATFORMER_PLAYER_SHAPE.with(|c| c.get());
    if player == 0 {
        return true;
    }
    let sign = if shape_a.index1 == player {
        -1.0
    } else if shape_b.index1 == player {
        1.0
    } else {
        return true;
    };
    // C Platform::PreSolve — allow contact only when normal points up relative to player.
    sign * normal.y > 0.95
}

impl SimWorld {
    fn demo_shape_index(&self, sid: ShapeId) -> i32 {
        self.shapes
            .iter()
            .position(|&s| s.index1 == sid.index1 && s.generation == sid.generation)
            .map(|i| i as i32)
            .unwrap_or(-1)
    }

    fn demo_body_index_from_id(&self, body_id: box2d_rust::id::BodyId) -> i32 {
        if body_id.index1 == 0 {
            return -1;
        }
        let full = get_body_full_id(&self.world, body_id);
        self.bodies
            .iter()
            .position(|&b| b == full)
            .map(|i| i as i32)
            .unwrap_or(-1)
    }

    fn contact_sim_manifold(&self, contact_index: i32) -> Option<&box2d_rust::collision::Manifold> {
        let contact = self.world.contacts.get(contact_index as usize)?;
        if contact.set_index == NULL_INDEX {
            return None;
        }
        if contact.set_index == AWAKE_SET && contact.color_index != NULL_INDEX {
            Some(
                &self.world.constraint_graph.colors[contact.color_index as usize].contact_sims
                    [contact.local_index as usize]
                    .manifold,
            )
        } else {
            Some(
                &self.world.solver_sets[contact.set_index as usize].contact_sims
                    [contact.local_index as usize]
                    .manifold,
            )
        }
    }
}

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

    pub fn shape_enable_sensor_events(&mut self, shape_index: usize, flag: bool) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_enable_sensor_events(&mut self.world, self.shapes[shape_index], flag);
    }

    pub fn shape_are_sensor_events_enabled(&self, shape_index: usize) -> bool {
        if shape_index >= self.shapes.len() {
            return false;
        }
        shape_are_sensor_events_enabled(&self.world, self.shapes[shape_index])
    }

    pub fn shape_is_valid(&self, shape_index: usize) -> bool {
        if shape_index >= self.shapes.len() {
            return false;
        }
        shape_is_valid(&self.world, self.shapes[shape_index])
    }

    pub fn shape_index1(&self, shape_index: usize) -> i32 {
        if shape_index >= self.shapes.len() {
            return 0;
        }
        self.shapes[shape_index].index1
    }

    pub fn destroy_shape(&mut self, shape_index: usize, update_body_mass: bool) {
        if shape_index >= self.shapes.len() {
            return;
        }
        let sid = self.shapes[shape_index];
        if !shape_is_valid(&self.world, sid) {
            return;
        }
        destroy_shape(&mut self.world, sid, update_body_mass);
    }

    pub fn apply_mass_from_shapes(&mut self, index: usize) {
        let body_id = self.body_id_at(index);
        body_apply_mass_from_shapes(&mut self.world, body_id);
    }

    pub fn body_set_user_data(&mut self, index: usize, data: u32) {
        let body_id = self.body_id_at(index);
        body_set_user_data(&mut self.world, body_id, u64::from(data));
    }

    pub fn body_set_name(&mut self, index: usize, name: &str) {
        let body_id = self.body_id_at(index);
        body_set_name(&mut self.world, body_id, name);
    }

    pub fn joint_set_user_data(&mut self, index: usize, data: u32) {
        if index >= self.joints.len() {
            return;
        }
        joint_set_user_data(&mut self.world, self.joints[index], u64::from(data));
    }

    /// Install Platformer one-way pre-solve using player shape demo index.
    pub fn enable_platformer_presolve(&mut self, player_shape_index: usize) {
        let index1 = if player_shape_index < self.shapes.len() {
            self.shapes[player_shape_index].index1
        } else {
            0
        };
        PLATFORMER_PLAYER_SHAPE.with(|c| c.set(index1));
        let fcn: PreSolveFcn = platformer_presolve;
        world_set_pre_solve_callback(&mut self.world, Some(fcn), 0);
    }

    pub fn clear_presolve(&mut self) {
        PLATFORMER_PLAYER_SHAPE.with(|c| c.set(0));
        world_set_pre_solve_callback(&mut self.world, None, 0);
    }

    /// Sensor begin events: [sensorShapeIdx, visitorShapeIdx]* (-1 if untracked).
    pub fn sensor_begin_events(&self) -> Vec<i32> {
        let ev = world_get_sensor_events(&self.world);
        let mut out = Vec::with_capacity(ev.begin_events.len() * 2);
        for e in ev.begin_events {
            out.push(self.demo_shape_index(e.sensor_shape_id));
            out.push(self.demo_shape_index(e.visitor_shape_id));
        }
        out
    }

    /// Sensor end events: [sensorShapeIdx, visitorShapeIdx]*.
    pub fn sensor_end_events(&self) -> Vec<i32> {
        let ev = world_get_sensor_events(&self.world);
        let mut out = Vec::with_capacity(ev.end_events.len() * 2);
        for e in ev.end_events {
            out.push(self.demo_shape_index(e.sensor_shape_id));
            out.push(self.demo_shape_index(e.visitor_shape_id));
        }
        out
    }

    /// Sensor begin with visitor body demo index: [sensorShapeIdx, visitorBodyIdx]*.
    pub fn sensor_begin_visitor_bodies(&self) -> Vec<i32> {
        let ev = world_get_sensor_events(&self.world);
        let mut out = Vec::with_capacity(ev.begin_events.len() * 2);
        for e in ev.begin_events {
            out.push(self.demo_shape_index(e.sensor_shape_id));
            let body = shape_get_body(&self.world, e.visitor_shape_id);
            out.push(self.demo_body_index_from_id(body));
        }
        out
    }

    /// Contact begin: [shapeAIdx, shapeBIdx, contactIndex1, generation]*.
    pub fn contact_begin_events(&self) -> Vec<i32> {
        let ev = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(ev.begin_events.len() * 4);
        for e in ev.begin_events {
            out.push(self.demo_shape_index(e.shape_id_a));
            out.push(self.demo_shape_index(e.shape_id_b));
            out.push(e.contact_id.index1);
            out.push(e.contact_id.generation as i32);
        }
        out
    }

    /// Contact end: [shapeAIdx, shapeBIdx, contactIndex1, generation]*.
    pub fn contact_end_events(&self) -> Vec<i32> {
        let ev = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(ev.end_events.len() * 4);
        for e in ev.end_events {
            out.push(self.demo_shape_index(e.shape_id_a));
            out.push(self.demo_shape_index(e.shape_id_b));
            out.push(e.contact_id.index1);
            out.push(e.contact_id.generation as i32);
        }
        out
    }

    /// Hit events extended: [x, y, speed, nx, ny, contactIndex1, generation]*.
    pub fn hit_events_ex(&self) -> Vec<f32> {
        let ev = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(ev.hit_events.len() * 7);
        for hit in ev.hit_events {
            out.push(hit.point.x as f32);
            out.push(hit.point.y as f32);
            out.push(hit.approach_speed);
            out.push(hit.normal.x);
            out.push(hit.normal.y);
            out.push(hit.contact_id.index1 as f32);
            out.push(hit.contact_id.generation as f32);
        }
        out
    }

    /// Body move events: [bodyIdx, fellAsleep, x, y, angle]*.
    pub fn body_move_events(&self) -> Vec<f32> {
        let ev = world_get_body_events(&self.world);
        let mut out = Vec::with_capacity(ev.len() * 5);
        for e in ev {
            let bi = self.demo_body_index_from_id(e.body_id);
            out.push(bi as f32);
            out.push(if e.fell_asleep { 1.0 } else { 0.0 });
            out.push(e.transform.p.x as f32);
            out.push(e.transform.p.y as f32);
            // angle from rot
            out.push(e.transform.q.s.atan2(e.transform.q.c));
        }
        out
    }

    /// Joint events: [jointDemoIdx, userData]* (-1 joint if destroyed already).
    pub fn joint_events(&self) -> Vec<i32> {
        let ev = world_get_joint_events(&self.world);
        let mut out = Vec::with_capacity(ev.len() * 2);
        for e in ev {
            let ji = self
                .joints
                .iter()
                .position(|&j| {
                    j.index1 == e.joint_id.index1 && j.generation == e.joint_id.generation
                })
                .map(|i| i as i32)
                .unwrap_or(-1);
            out.push(ji);
            out.push(e.user_data as i32);
        }
        out
    }

    /// Destroy joint by demo index if still valid (Joint Event sample).
    pub fn destroy_joint_if_valid(&mut self, index: usize) {
        if index >= self.joints.len() {
            return;
        }
        let jid = self.joints[index];
        if joint_is_valid(&self.world, jid) {
            destroy_joint(&mut self.world, jid, true);
        }
    }

    /// Contact validity (b2Contact_IsValid).
    pub fn contact_is_valid(&self, index1: i32, generation: u32) -> bool {
        if index1 <= 0 {
            return false;
        }
        let id = (index1 - 1) as usize;
        if id >= self.world.contacts.len() {
            return false;
        }
        let c = &self.world.contacts[id];
        c.set_index != NULL_INDEX && c.generation == generation
    }

    /// Manifold impulses for a contact: [nx, ny, pointCount, then per point:
    /// anchorAx, anchorAy, normalImpulse, totalNormalImpulse]*.
    pub fn contact_manifold(&self, index1: i32, generation: u32) -> Vec<f32> {
        if !self.contact_is_valid(index1, generation) {
            return Vec::new();
        }
        let contact_index = index1 - 1;
        let Some(manifold) = self.contact_sim_manifold(contact_index) else {
            return Vec::new();
        };
        let mut out = vec![
            manifold.normal.x,
            manifold.normal.y,
            manifold.point_count as f32,
        ];
        for i in 0..manifold.point_count as usize {
            let p = &manifold.points[i];
            out.push(p.anchor_a.x);
            out.push(p.anchor_a.y);
            out.push(p.normal_impulse);
            out.push(p.total_normal_impulse);
        }
        out
    }

    /// Persistent-contact draw helpers: world center of shape A's body + manifold.
    /// Returns [cx, cy, nx, ny, pointCount, then per-pt: px, py, totalImpulse]*.
    pub fn contact_draw_data(&self, index1: i32, generation: u32) -> Vec<f32> {
        if !self.contact_is_valid(index1, generation) {
            return Vec::new();
        }
        let contact_index = index1 - 1;
        let contact = &self.world.contacts[contact_index as usize];
        let shape_a = &self.world.shapes[contact.shape_id_a as usize];
        let body_id = make_body_id(&self.world, shape_a.body_id);
        let center = body_get_world_center(&self.world, body_id);
        let Some(manifold) = self.contact_sim_manifold(contact_index) else {
            return Vec::new();
        };
        let mut out = vec![
            center.x as f32,
            center.y as f32,
            manifold.normal.x,
            manifold.normal.y,
            manifold.point_count as f32,
        ];
        for i in 0..manifold.point_count as usize {
            let p = &manifold.points[i];
            out.push(center.x as f32 + p.anchor_a.x);
            out.push(center.y as f32 + p.anchor_a.y);
            out.push(p.total_normal_impulse);
        }
        out
    }

    /// Sensor visitor AABB centers for a sensor shape: [x, y]*.
    pub fn sensor_visitor_centers(&self, shape_index: usize) -> Vec<f32> {
        if shape_index >= self.shapes.len() {
            return Vec::new();
        }
        let sid = self.shapes[shape_index];
        if !shape_is_valid(&self.world, sid) {
            return Vec::new();
        }
        let visitors = shape_get_sensor_data(&self.world, sid, 64);
        let mut out = Vec::with_capacity(visitors.len() * 2);
        for v in visitors {
            if !shape_is_valid(&self.world, v) {
                continue;
            }
            let aabb = shape_get_aabb(&self.world, v);
            out.push(0.5 * (aabb.lower_bound.x + aabb.upper_bound.x) as f32);
            out.push(0.5 * (aabb.lower_bound.y + aabb.upper_bound.y) as f32);
        }
        out
    }

    /// Sensor overlap body names (Sensor Types readout): comma-separated.
    pub fn sensor_visitor_names(&self, shape_index: usize) -> String {
        if shape_index >= self.shapes.len() {
            return String::new();
        }
        let sid = self.shapes[shape_index];
        if !shape_is_valid(&self.world, sid) {
            return String::new();
        }
        let visitors = shape_get_sensor_data(&self.world, sid, 64);
        let mut names = Vec::new();
        for v in visitors {
            if !shape_is_valid(&self.world, v) {
                continue;
            }
            let body = shape_get_body(&self.world, v);
            let name = box2d_rust::body::body_get_name(&self.world, body);
            if !name.is_empty() {
                names.push(name.to_string());
            }
        }
        names.join(", ")
    }

    /// Closest ray cast: [hit(0/1), x, y, nx, ny, fraction].
    pub fn cast_ray_closest(&mut self, ox: f32, oy: f32, tx: f32, ty: f32) -> Vec<f32> {
        let origin = to_pos(Vec2 { x: ox, y: oy });
        let translation = Vec2 { x: tx, y: ty };
        let filter = default_query_filter();
        let r = world_cast_ray_closest(&mut self.world, origin, translation, filter);
        if r.hit {
            vec![
                1.0,
                r.point.x as f32,
                r.point.y as f32,
                r.normal.x,
                r.normal.y,
                r.fraction,
            ]
        } else {
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        }
    }

    /// Contact begin body pairs for Contact sample:
    /// [bodyA, bodyB, shapeA, shapeB]*.
    pub fn contact_begin_bodies(&self) -> Vec<i32> {
        let ev = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(ev.begin_events.len() * 4);
        for e in ev.begin_events {
            let ba = shape_get_body(&self.world, e.shape_id_a);
            let bb = shape_get_body(&self.world, e.shape_id_b);
            out.push(self.demo_body_index_from_id(ba));
            out.push(self.demo_body_index_from_id(bb));
            out.push(self.demo_shape_index(e.shape_id_a));
            out.push(self.demo_shape_index(e.shape_id_b));
        }
        out
    }

    /// Absorb all shapes from `src` onto `dest` with relative transform
    /// (Contact sample debris attach). Destroys `src`. New shapes enable
    /// contact events and are tracked.
    pub fn absorb_body_shapes(&mut self, dest: usize, src: usize) {
        if !self.is_body_alive(dest) || !self.is_body_alive(src) {
            return;
        }
        let dest_id = self.body_id_at(dest);
        let src_id = self.body_id_at(src);
        let player_xf = body_get_transform(&self.world, dest_id);
        let debris_xf = body_get_transform(&self.world, src_id);
        let relative = inv_mul_world_transforms(player_xf, debris_xf);

        let shapes = body_get_shapes(&self.world, src_id, 8);
        for sid in shapes {
            let mut shape_def = default_shape_def();
            shape_def.enable_contact_events = true;
            match shape_get_type(&self.world, sid) {
                ShapeType::Circle => {
                    let mut circle = shape_get_circle(&self.world, sid);
                    circle.center = transform_point(relative, circle.center);
                    let new_sid =
                        create_circle_shape(&mut self.world, dest_id, &shape_def, &circle);
                    self.track_shape(new_sid);
                }
                ShapeType::Capsule => {
                    let mut capsule = shape_get_capsule(&self.world, sid);
                    capsule.center1 = transform_point(relative, capsule.center1);
                    capsule.center2 = transform_point(relative, capsule.center2);
                    let new_sid =
                        create_capsule_shape(&mut self.world, dest_id, &shape_def, &capsule);
                    self.track_shape(new_sid);
                }
                ShapeType::Polygon => {
                    let original = shape_get_polygon(&self.world, sid);
                    let polygon = transform_polygon(relative, &original);
                    let new_sid =
                        create_polygon_shape(&mut self.world, dest_id, &shape_def, &polygon);
                    self.track_shape(new_sid);
                }
                _ => {}
            }
        }
        self.destroy_body(src);
    }

    /// Override mass while keeping inertia ratio (Circle Impulse).
    pub fn set_mass_data_scale(&mut self, index: usize, mass: f32) {
        use box2d_rust::body::{body_get_mass_data, body_set_mass_data};
        let body_id = self.body_id_at(index);
        let mut md = body_get_mass_data(&self.world, body_id);
        if md.mass > 0.0 {
            let ratio = mass / md.mass;
            md.mass = mass;
            md.rotational_inertia *= ratio;
            body_set_mass_data(&mut self.world, body_id, md);
        }
    }

    /// Body world center [x, y].
    pub fn body_world_center(&self, index: usize) -> Vec<f32> {
        let body_id = self.body_id_at(index);
        let c = body_get_world_center(&self.world, body_id);
        vec![c.x as f32, c.y as f32]
    }

    /// Shape body demo index (-1 if untracked).
    pub fn shape_body_index(&self, shape_index: usize) -> i32 {
        if shape_index >= self.shapes.len() {
            return -1;
        }
        let sid = self.shapes[shape_index];
        if !shape_is_valid(&self.world, sid) {
            return -1;
        }
        self.demo_body_index_from_id(shape_get_body(&self.world, sid))
    }

    pub fn shape_enable_contact_events(&mut self, shape_index: usize, flag: bool) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_enable_contact_events(&mut self.world, self.shapes[shape_index], flag);
    }

    pub fn shape_enable_hit_events(&mut self, shape_index: usize, flag: bool) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_enable_hit_events(&mut self.world, self.shapes[shape_index], flag);
    }

    pub fn shape_enable_presolve_events(&mut self, shape_index: usize, flag: bool) {
        if shape_index >= self.shapes.len() {
            return;
        }
        shape_enable_pre_solve_events(&mut self.world, self.shapes[shape_index], flag);
    }

    /// Sensor begin events that match a specific sensor shape demo index —
    /// returns visitor body user_data values (Sensor Funnel).
    pub fn sensor_begin_user_data_for(&self, sensor_shape: usize) -> Vec<u32> {
        if sensor_shape >= self.shapes.len() {
            return Vec::new();
        }
        let target = self.shapes[sensor_shape];
        let ev = world_get_sensor_events(&self.world);
        let mut out = Vec::new();
        for e in ev.begin_events {
            if e.sensor_shape_id.index1 == target.index1
                && e.sensor_shape_id.generation == target.generation
            {
                let body = shape_get_body(&self.world, e.visitor_shape_id);
                let ud = box2d_rust::body::body_get_user_data(&self.world, body);
                out.push(ud as u32);
            }
        }
        out
    }
}
