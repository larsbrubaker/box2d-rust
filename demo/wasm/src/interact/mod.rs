//! Shared mouse-grab helpers for demo worlds. The debug-draw collector
//! (`b2World_Draw` → canvas buffer adapter) lives in the [`draw`] submodule and
//! is re-exported here so `interact::collect_world_draw` keeps resolving.
//!
//! Mirrors the C samples' Sample::Mouse* grab (kinematic body + motor joint):
//!   linearHertz = 7.5, linearDampingRatio = 1.0,
//!   maxSpringForce = m_mouseForceScale * mass * |g| (default scale 100),
//!   maxVelocityTorque = 0.25 * lever * mg

use box2d_rust::body::{
    body_get_local_point, body_get_mass_data, body_get_type, body_is_valid, body_set_awake,
    body_set_target_transform, create_body, destroy_body,
};
use box2d_rust::id::{BodyId, JointId, ShapeId};
use box2d_rust::joint::{create_motor_joint, destroy_joint, joint_is_valid};
use box2d_rust::math_functions as m;
use box2d_rust::math_functions::{Aabb, Pos, Transform, Vec2, WorldTransform, ROT_IDENTITY};
use box2d_rust::shape::shape_get_body;
use box2d_rust::types::{
    default_body_def, default_motor_joint_def, default_query_filter, BodyType,
};
use box2d_rust::world::{world_get_gravity, world_overlap_aabb, World};

mod draw;
pub use draw::*;

/// Default `m_mouseForceScale` from Sample ctor (sample.cpp).
pub const DEFAULT_GRAB_FORCE_SCALE: f32 = 100.0;

#[derive(Clone, Copy)]
pub struct MouseGrab {
    pub mouse_body_id: BodyId,
    pub mouse_joint_id: JointId,
    pub mouse_point: Pos,
    pub force_scale: f32,
}

impl Default for MouseGrab {
    fn default() -> Self {
        Self {
            mouse_body_id: BodyId::default(),
            mouse_joint_id: JointId::default(),
            mouse_point: m::POS_ZERO,
            force_scale: DEFAULT_GRAB_FORCE_SCALE,
        }
    }
}

impl MouseGrab {
    #[allow(dead_code)]
    pub fn clear(&mut self) {
        *self = Self {
            force_scale: self.force_scale,
            ..Self::default()
        };
    }

    pub fn validate(&mut self, world: &mut World) {
        if self.mouse_joint_id.is_non_null() && !joint_is_valid(world, self.mouse_joint_id) {
            self.mouse_joint_id = JointId::default();
            if self.mouse_body_id.is_non_null() && body_is_valid(world, self.mouse_body_id) {
                destroy_body(world, self.mouse_body_id);
            }
            self.mouse_body_id = BodyId::default();
        }
    }

    pub fn pre_step(&mut self, world: &mut World, time_step: f32) {
        self.validate(world);
        if self.mouse_body_id.is_non_null()
            && body_is_valid(world, self.mouse_body_id)
            && time_step > 0.0
        {
            let target = WorldTransform {
                p: self.mouse_point,
                q: ROT_IDENTITY,
            };
            body_set_target_transform(world, self.mouse_body_id, target, time_step, true);
        }
    }

    /// Begin a grab at world point `(x, y)`. Returns true if a dynamic body was grabbed.
    pub fn begin(&mut self, world: &mut World, x: f32, y: f32) -> bool {
        self.end(world);

        let p = m::to_pos(Vec2 { x, y });
        self.mouse_point = p;

        // Tiny AABB around the click (sample.cpp QueryContext).
        let d = Vec2 { x: 0.001, y: 0.001 };
        let aabb = Aabb {
            lower_bound: Vec2 { x: -d.x, y: -d.y },
            upper_bound: d,
        };
        let filter = default_query_filter();
        // Collect shape ids first — the overlap callback cannot re-borrow `world`.
        let mut shape_ids: Vec<ShapeId> = Vec::new();
        world_overlap_aabb(world, p, aabb, filter, |shape_id: ShapeId| {
            shape_ids.push(shape_id);
            true
        });

        let mut hit_body = BodyId::default();
        for shape_id in shape_ids {
            let body_id = shape_get_body(world, shape_id);
            if body_get_type(world, body_id) == BodyType::Dynamic {
                hit_body = body_id;
                break;
            }
        }

        if hit_body.is_null() {
            return false;
        }

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = self.mouse_point;
        body_def.enable_sleep = false;
        self.mouse_body_id = create_body(world, &body_def);

        let mut joint_def = default_motor_joint_def();
        joint_def.base.body_id_a = self.mouse_body_id;
        joint_def.base.body_id_b = hit_body;
        joint_def.base.local_frame_b = Transform {
            p: body_get_local_point(world, hit_body, p),
            q: ROT_IDENTITY,
        };
        joint_def.linear_hertz = 7.5;
        joint_def.linear_damping_ratio = 1.0;

        let mass_data = body_get_mass_data(world, hit_body);
        let g = m::length(world_get_gravity(world));
        let mg = mass_data.mass * g;
        joint_def.max_spring_force = self.force_scale * mg;

        if mass_data.mass > 0.0 {
            let lever = (mass_data.rotational_inertia / mass_data.mass).sqrt();
            joint_def.max_velocity_torque = 0.25 * lever * mg;
        }

        self.mouse_joint_id = create_motor_joint(world, &joint_def);
        body_set_awake(world, hit_body, true);
        true
    }

    pub fn move_to(&mut self, x: f32, y: f32) {
        if self.mouse_joint_id.is_non_null() {
            self.mouse_point = m::to_pos(Vec2 { x, y });
        }
    }

    pub fn end(&mut self, world: &mut World) {
        if self.mouse_joint_id.is_non_null() && joint_is_valid(world, self.mouse_joint_id) {
            destroy_joint(world, self.mouse_joint_id, true);
        }
        if self.mouse_body_id.is_non_null() && body_is_valid(world, self.mouse_body_id) {
            destroy_body(world, self.mouse_body_id);
        }
        self.mouse_joint_id = JointId::default();
        self.mouse_body_id = BodyId::default();
    }

    pub fn is_active(&self) -> bool {
        self.mouse_joint_id.is_non_null()
    }
}
