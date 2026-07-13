// SimWorld: the live physics world behind the simulation demo pages.
// Split from lib.rs; every value shown on the demo site is computed by
// the ported Rust code, never re-implemented in JavaScript.
//
// Extra construction/manipulation APIs for Phase 2 C samples live in
// sibling modules (joints / shapes / body_ops / world_ops) so this file
// stays under the 800-line limit.

mod benchmark_ops;
mod body_ops;
mod event_attach;
mod event_ops;
mod human;
mod human_create;
mod joint_ops;
mod joints;
mod mover;
mod query_ops;
mod shape_mutate;
mod shape_ops;
mod shapes;
#[cfg(test)]
mod tests;
mod world_ops;

use box2d_rust::collision::Capsule;
use box2d_rust::math_functions as m;
use box2d_rust::mover::CollisionPlane;
use wasm_bindgen::prelude::*;

use box2d_rust::body::{create_body, get_body_full_id, get_body_transform, make_body_id};
use box2d_rust::geometry::make_box;
use box2d_rust::id::{BodyId, ChainId, JointId, ShapeId};
use box2d_rust::joint::{
    create_distance_joint, create_revolute_joint, joint_get_body_a, joint_get_body_b,
    joint_get_local_frame_a, joint_get_local_frame_b,
};
use box2d_rust::math_functions::{inv_transform_world_point, transform_world_point};
use box2d_rust::shape::{create_circle_shape, create_polygon_shape};
use box2d_rust::types::{
    default_body_def, default_distance_joint_def, default_revolute_joint_def, default_shape_def,
    default_world_def, BodyType,
};
use box2d_rust::world::{
    world_enable_continuous, world_get_contact_events, world_get_sensor_events, world_step, World,
};

use crate::interact::{collect_world_draw, MouseGrab};
use mover::PogoShape;

/// A live physics world for the Bodies/Stacking demos. Every step runs the
/// ported b2World_Step pipeline: broad phase, narrow phase, graph-colored
/// soft-constraint solver, restitution, and sleeping.
#[wasm_bindgen]
pub struct SimWorld {
    pub(crate) world: World,
    /// Raw body indices in creation order; positions() reports in this order.
    pub(crate) bodies: Vec<i32>,
    /// Joint ids in creation order; joint_anchors() reports in this order.
    pub(crate) joints: Vec<JointId>,
    /// Shape ids from attach_*_mat / filter / chain-segment (Shapes samples).
    pub(crate) shapes: Vec<ShapeId>,
    /// Chain ids from `add_chain_mat` / `add_chain*` (Chain Shape surface updates).
    pub(crate) chains: Vec<ChainId>,
    /// Spawned ragdolls (`shared/human.c` CreateHuman).
    pub(crate) humans: Vec<human::Human>,
    /// Character mover state (not a body; driven by the mover queries).
    pub(crate) mover_position: m::Pos,
    pub(crate) mover_velocity: m::Vec2,
    pub(crate) mover_pogo_velocity: f32,
    pub(crate) mover_on_ground: bool,
    pub(crate) mover_jump_released: bool,
    pub(crate) mover_plane_count: i32,
    pub(crate) mover_total_iterations: i32,
    pub(crate) mover_planes: Vec<CollisionPlane>,
    pub(crate) mover_pogo_draw: Vec<f32>,
    pub(crate) mover_kick_draw: Vec<f32>,
    pub(crate) mover_pogo_shape: i32,
    pub(crate) mover_jump_speed: f32,
    pub(crate) mover_min_speed: f32,
    pub(crate) mover_max_speed: f32,
    pub(crate) mover_stop_speed: f32,
    pub(crate) mover_accelerate: f32,
    pub(crate) mover_friction: f32,
    pub(crate) mover_gravity: f32,
    pub(crate) mover_air_steer: f32,
    pub(crate) mover_pogo_hertz: f32,
    pub(crate) mover_pogo_damping: f32,
    /// C Sample mouse grab (kinematic body + motor joint).
    grab: MouseGrab,
    /// Last collected debug-draw buffers (see draw_* accessors).
    draw_polygons: Vec<f32>,
    draw_circles: Vec<f32>,
    draw_capsules: Vec<f32>,
    draw_lines: Vec<f32>,
    draw_points: Vec<f32>,
    draw_text: String,
}

impl SimWorld {
    pub(crate) fn track_body(&mut self, body_id: BodyId) -> usize {
        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    pub(crate) fn track_joint(&mut self, joint_id: JointId) -> usize {
        self.joints.push(joint_id);
        self.joints.len() - 1
    }

    pub(crate) fn track_chain(&mut self, chain_id: ChainId) -> usize {
        self.chains.push(chain_id);
        self.chains.len() - 1
    }

    pub(crate) fn body_id_at(&self, index: usize) -> BodyId {
        make_body_id(&self.world, self.bodies[index])
    }

    pub(crate) fn body_index_at(&self, index: usize) -> i32 {
        self.bodies[index]
    }
}

#[wasm_bindgen]
impl SimWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_y: f32) -> SimWorld {
        let mut world_def = default_world_def();
        world_def.gravity = m::Vec2 {
            x: 0.0,
            y: gravity_y,
        };
        SimWorld {
            world: World::new(&world_def),
            bodies: Vec::new(),
            joints: Vec::new(),
            shapes: Vec::new(),
            chains: Vec::new(),
            humans: Vec::new(),
            mover_position: m::POS_ZERO,
            mover_velocity: m::VEC2_ZERO,
            mover_pogo_velocity: 0.0,
            mover_on_ground: false,
            mover_jump_released: true,
            mover_plane_count: 0,
            mover_total_iterations: 0,
            mover_planes: Vec::new(),
            mover_pogo_draw: Vec::new(),
            mover_kick_draw: Vec::new(),
            // C defaults (`sample_character.cpp:595-606`)
            mover_pogo_shape: PogoShape::Segment as i32,
            mover_jump_speed: 10.0,
            mover_min_speed: 0.1,
            mover_max_speed: 6.0,
            mover_stop_speed: 3.0,
            mover_accelerate: 20.0,
            mover_friction: 8.0,
            mover_gravity: 30.0,
            mover_air_steer: 0.2,
            mover_pogo_hertz: 5.0,
            mover_pogo_damping: 0.8,
            grab: MouseGrab::default(),
            draw_polygons: Vec::new(),
            draw_circles: Vec::new(),
            draw_capsules: Vec::new(),
            draw_lines: Vec::new(),
            draw_points: Vec::new(),
            draw_text: String::new(),
        }
    }

    /// Static box (the ground or a wall). Returns the demo body index.
    pub fn add_static_box(&mut self, x: f32, y: f32, hx: f32, hy: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let shape_def = default_shape_def();
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Dynamic box. Returns the demo body index.
    pub fn add_box(&mut self, x: f32, y: f32, hx: f32, hy: f32, density: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Dynamic circle. Returns the demo body index.
    pub fn add_circle(&mut self, x: f32, y: f32, radius: f32, density: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        shape_def.material.restitution = 0.2;
        let circle = box2d_rust::collision::Circle {
            center: m::VEC2_ZERO,
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Advance the simulation. (b2World_Step)
    /// Drives the mouse-grab kinematic target before stepping when `dt > 0`.
    pub fn step(&mut self, dt: f32, sub_step_count: i32) {
        self.grab.pre_step(&mut self.world, dt);
        if dt > 0.0 {
            world_step(&mut self.world, dt, sub_step_count);
        }
    }

    /// Begin a mouse grab at world `(x, y)`. C spring: hertz 7.5, damping 1.0.
    /// Returns true if a dynamic body was grabbed.
    pub fn mouse_down(&mut self, x: f32, y: f32) -> bool {
        self.grab.begin(&mut self.world, x, y)
    }

    /// Update the grab target (world space).
    pub fn mouse_move(&mut self, x: f32, y: f32) {
        self.grab.move_to(x, y);
    }

    /// Release the mouse grab.
    pub fn mouse_up(&mut self) {
        self.grab.end(&mut self.world);
    }

    /// Whether a mouse joint is currently active.
    pub fn mouse_active(&self) -> bool {
        self.grab.is_active()
    }

    /// Override `m_mouseForceScale` (C Sample default 100).
    pub fn set_grab_force_scale(&mut self, scale: f32) {
        self.grab.force_scale = scale;
    }

    /// Run `b2World_Draw` into internal buffers. Bounds: lowerX, lowerY, upperX, upperY.
    /// Honors the global view-flag mask from `sim_set_debug_flags`.
    pub fn collect_draw(&mut self, lower_x: f32, lower_y: f32, upper_x: f32, upper_y: f32) {
        let collected = collect_world_draw(&mut self.world, [lower_x, lower_y, upper_x, upper_y]);
        let text = collected.text_json();
        self.draw_polygons = collected.polygons;
        self.draw_circles = collected.circles;
        self.draw_capsules = collected.capsules;
        self.draw_lines = collected.lines;
        self.draw_points = collected.points;
        self.draw_text = text;
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

    pub fn draw_points(&self) -> Vec<f32> {
        self.draw_points.clone()
    }

    pub fn draw_text(&self) -> String {
        self.draw_text.clone()
    }

    /// Interleaved [x, y, angle] for every demo body, in creation order.
    /// Destroyed slots (`destroy_body`) contribute zeros so indices stay stable.
    pub fn positions(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(3 * self.bodies.len());
        for &body_index in &self.bodies {
            if body_index == body_ops::DESTROYED_BODY_SLOT {
                out.extend_from_slice(&[0.0, 0.0, 0.0]);
                continue;
            }
            let transform = get_body_transform(&self.world, body_index);
            out.push(transform.p.x as f32);
            out.push(transform.p.y as f32);
            out.push(m::rot_get_angle(transform.q));
        }
        out
    }

    /// Number of awake bodies (sleeping islands leave this count).
    pub fn awake_body_count(&self) -> i32 {
        self.world.solver_sets[box2d_rust::solver_set::AWAKE_SET as usize]
            .body_sims
            .len() as i32
    }

    /// Live contact count.
    pub fn contact_count(&self) -> i32 {
        self.world.contact_id_pool.id_count()
    }

    /// Demo body slots in creation order (includes holes from `destroy_body`).
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Dynamic box with an initial rotation, for hinge chains. Returns the
    /// demo body index.
    pub fn add_box_rotated(
        &mut self,
        x: f32,
        y: f32,
        hx: f32,
        hy: f32,
        density: f32,
        angle: f32,
    ) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        body_def.rotation = m::make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Hinge two demo bodies at a world-space pivot with the falling-hinges
    /// tuning: limited, sprung, and motorized. (b2CreateRevoluteJoint)
    pub fn add_hinge_joint(&mut self, index_a: usize, index_b: usize, px: f32, py: f32) -> usize {
        let pivot = m::to_pos(m::Vec2 { x: px, y: py });
        let body_a = self.bodies[index_a];
        let body_b = self.bodies[index_b];
        let xf_a = get_body_transform(&self.world, body_a);
        let xf_b = get_body_transform(&self.world, body_b);

        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = make_body_id(&self.world, body_a);
        joint_def.base.body_id_b = make_body_id(&self.world, body_b);
        joint_def.base.local_frame_a.p = inv_transform_world_point(xf_a, pivot);
        joint_def.base.local_frame_b.p = inv_transform_world_point(xf_b, pivot);
        joint_def.enable_limit = true;
        joint_def.lower_angle = -0.1 * m::PI;
        joint_def.upper_angle = 0.2 * m::PI;
        joint_def.enable_spring = true;
        joint_def.hertz = 1.0;
        joint_def.damping_ratio = 1.0;
        joint_def.enable_motor = true;
        joint_def.max_motor_torque = 0.25;
        joint_def.base.constraint_hertz = 60.0;
        joint_def.base.constraint_damping_ratio = 0.0;

        let joint_id = create_revolute_joint(&mut self.world, &joint_def);
        self.joints.push(joint_id);
        self.joints.len() - 1
    }

    /// Connect two demo bodies with a rigid distance joint between world-space
    /// anchors. (b2CreateDistanceJoint)
    pub fn add_distance_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
    ) -> usize {
        let anchor_a = m::to_pos(m::Vec2 { x: ax, y: ay });
        let anchor_b = m::to_pos(m::Vec2 { x: bx, y: by });
        let body_a = self.bodies[index_a];
        let body_b = self.bodies[index_b];
        let xf_a = get_body_transform(&self.world, body_a);
        let xf_b = get_body_transform(&self.world, body_b);

        let mut joint_def = default_distance_joint_def();
        joint_def.base.body_id_a = make_body_id(&self.world, body_a);
        joint_def.base.body_id_b = make_body_id(&self.world, body_b);
        joint_def.base.local_frame_a.p = inv_transform_world_point(xf_a, anchor_a);
        joint_def.base.local_frame_b.p = inv_transform_world_point(xf_b, anchor_b);
        joint_def.length = m::length(m::sub_pos(anchor_b, anchor_a));

        let joint_id = create_distance_joint(&mut self.world, &joint_def);
        self.joints.push(joint_id);
        self.joints.len() - 1
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Interleaved world anchors [ax, ay, bx, by] for every joint, in
    /// creation order, for drawing noodles.
    pub fn joint_anchors(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(4 * self.joints.len());
        for &joint_id in &self.joints {
            let body_a = joint_get_body_a(&self.world, joint_id);
            let body_b = joint_get_body_b(&self.world, joint_id);
            let index_a = get_body_full_id(&self.world, body_a);
            let index_b = get_body_full_id(&self.world, body_b);
            let xf_a = get_body_transform(&self.world, index_a);
            let xf_b = get_body_transform(&self.world, index_b);
            let frame_a = joint_get_local_frame_a(&self.world, joint_id);
            let frame_b = joint_get_local_frame_b(&self.world, joint_id);
            let world_a = transform_world_point(xf_a, frame_a.p);
            let world_b = transform_world_point(xf_b, frame_b.p);
            out.push(world_a.x as f32);
            out.push(world_a.y as f32);
            out.push(world_b.x as f32);
            out.push(world_b.y as f32);
        }
        out
    }

    /// Dynamic circle with hit events enabled and lively restitution, for the
    /// Events demo.
    pub fn add_bouncy_ball(&mut self, x: f32, y: f32, radius: f32, restitution: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.material.friction = 0.3;
        shape_def.material.restitution = restitution;
        shape_def.enable_hit_events = true;
        let circle = box2d_rust::collision::Circle {
            center: m::VEC2_ZERO,
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Static sensor box that reports begin/end overlap events.
    pub fn add_sensor_box(&mut self, x: f32, y: f32, hx: f32, hy: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Enable sensor events on the most recently added body's shape so
    /// dynamic bodies can visit sensors.
    pub fn enable_sensor_visitor(&mut self, index: usize) {
        let body_index = self.bodies[index];
        let mut shape_id = self.world.bodies[body_index as usize].head_shape_id;
        while shape_id != box2d_rust::core::NULL_INDEX {
            self.world.shapes[shape_id as usize].enable_sensor_events = true;
            shape_id = self.world.shapes[shape_id as usize].next_shape_id;
        }
    }

    /// Contact/sensor event counts for the last step:
    /// [contactBegin, contactEnd, hit, sensorBegin, sensorEnd].
    pub fn event_counts(&self) -> Vec<u32> {
        let contact_events = world_get_contact_events(&self.world);
        let sensor_events = world_get_sensor_events(&self.world);
        vec![
            contact_events.begin_events.len() as u32,
            contact_events.end_events.len() as u32,
            contact_events.hit_events.len() as u32,
            sensor_events.begin_events.len() as u32,
            sensor_events.end_events.len() as u32,
        ]
    }

    /// Hit events from the last step as [x, y, approachSpeed]*.
    pub fn hit_events(&self) -> Vec<f32> {
        let contact_events = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(3 * contact_events.hit_events.len());
        for hit in contact_events.hit_events {
            out.push(hit.point.x as f32);
            out.push(hit.point.y as f32);
            out.push(hit.approach_speed);
        }
        out
    }

    /// Fast dynamic circle bullet for the Continuous demo. Continuous
    /// collision (TOI) keeps it from tunneling through thin walls.
    pub fn add_bullet(&mut self, x: f32, y: f32, radius: f32, vx: f32, vy: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        body_def.gravity_scale = 0.0;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        body_def.linear_velocity = m::Vec2 { x: vx, y: vy };
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.material.restitution = 0.1;
        let circle = box2d_rust::collision::Circle {
            center: m::VEC2_ZERO,
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Toggle continuous collision to demonstrate tunneling.
    /// (b2World_EnableContinuous)
    pub fn set_continuous(&mut self, flag: bool) {
        world_enable_continuous(&mut self.world, flag);
    }

    /// Serialize the full simulation state. (b2World_Snapshot)
    pub fn snapshot(&self) -> Vec<u8> {
        box2d_rust::recording::world_snapshot(&self.world)
    }

    /// Restore simulation state from a snapshot image. The demo body/joint
    /// tracking lists stay as-is: they only describe drawing, and restore is
    /// used on a world with the same scene. (b2World_Restore)
    pub fn restore(&mut self, image: &[u8]) -> bool {
        box2d_rust::recording::world_restore(&mut self.world, image)
    }

    /// FNV-1a hash over all body transforms and velocities, as a hex string
    /// for display. (b2HashWorldState)
    pub fn state_hash(&self) -> String {
        format!(
            "{:016X}",
            box2d_rust::recording::hash_world_state(&self.world)
        )
    }

    /// djb2 hash over demo-body transforms in the given order — matches
    /// `UpdateFallingHinges` (`shared/determinism.c`): `B2_HASH_INIT` then
    /// `b2Hash(hash, &transform, sizeof(b2WorldTransform))` per body.
    pub fn hash_body_transforms(&self, indices: &[u32]) -> u32 {
        use box2d_rust::core::{hash, HASH_INIT};
        let mut h = HASH_INIT;
        for &idx in indices {
            let body_index = self.bodies[idx as usize];
            if body_index == body_ops::DESTROYED_BODY_SLOT {
                continue;
            }
            let xf = get_body_transform(&self.world, body_index);
            // Same layout as determinism_tests::hash_transform / C sizeof(b2WorldTransform).
            let mut bytes = Vec::with_capacity(16);
            bytes.extend_from_slice(&xf.p.x.to_le_bytes());
            bytes.extend_from_slice(&xf.p.y.to_le_bytes());
            bytes.extend_from_slice(&xf.q.c.to_le_bytes());
            bytes.extend_from_slice(&xf.q.s.to_le_bytes());
            h = hash(h, &bytes);
        }
        h
    }

    /// Dynamic capsule (horizontal, half length `hl`), rotated by `angle`.
    /// Returns the demo body index.
    pub fn add_capsule(
        &mut self,
        x: f32,
        y: f32,
        hl: f32,
        radius: f32,
        density: f32,
        angle: f32,
    ) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        body_def.rotation = m::make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        let capsule = Capsule {
            center1: m::Vec2 { x: -hl, y: 0.0 },
            center2: m::Vec2 { x: hl, y: 0.0 },
            radius,
        };
        box2d_rust::shape::create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Static chain shape from interleaved world points [x, y]*. Chains are
    /// one-sided: wind right-to-left for a solid-side-up floor.
    /// (b2CreateChain) Returns the demo body index of the owning body.
    pub fn add_chain(&mut self, points: &[f32], is_loop: bool) -> usize {
        let body_def = default_body_def();
        let body_id = create_body(&mut self.world, &body_def);

        let mut chain_def = box2d_rust::types::default_chain_def();
        chain_def.is_loop = is_loop;
        chain_def.points = points
            .chunks_exact(2)
            .map(|p| m::Vec2 { x: p[0], y: p[1] })
            .collect();
        box2d_rust::shape::create_chain(&mut self.world, body_id, &chain_def);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Radial explosion. (b2World_Explode)
    pub fn explode(&mut self, x: f32, y: f32, radius: f32, falloff: f32, impulse_per_length: f32) {
        let mut def = box2d_rust::types::default_explosion_def();
        def.position = m::to_pos(m::Vec2 { x, y });
        def.radius = radius;
        def.falloff = falloff;
        def.impulse_per_length = impulse_per_length;
        box2d_rust::world::world_explode(&mut self.world, &def);
    }

    /// Change gravity at runtime. (b2World_SetGravity)
    pub fn set_gravity(&mut self, x: f32, y: f32) {
        box2d_rust::world::world_set_gravity(&mut self.world, m::Vec2 { x, y });
    }
}
