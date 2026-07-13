//! Body manipulation APIs used by Bodies / Stacking / Joints samples.

use super::SimWorld;
use box2d_rust::body::{
    body_apply_force, body_apply_force_to_center, body_apply_linear_impulse,
    body_apply_linear_impulse_to_center, body_apply_torque, body_disable, body_enable,
    body_get_angular_velocity, body_get_linear_velocity, body_get_mass, body_get_type,
    body_is_enabled, body_set_angular_damping, body_set_angular_velocity, body_set_awake,
    body_set_bullet, body_set_gravity_scale, body_set_linear_damping, body_set_linear_velocity,
    body_set_motion_locks, body_set_target_transform, body_set_transform, body_set_type,
    body_wake_touching, destroy_body,
};
use box2d_rust::joint::joint_is_valid;
use box2d_rust::math_functions::{make_rot, to_pos, Vec2, WorldTransform};
use box2d_rust::types::{BodyType, MotionLocks};
use wasm_bindgen::prelude::*;

/// Demo-index sentinel after `destroy_body` — keeps later indices stable.
pub(crate) const DESTROYED_BODY_SLOT: i32 = -1;

fn parse_body_type(body_type: i32) -> BodyType {
    match body_type {
        1 => BodyType::Kinematic,
        2 => BodyType::Dynamic,
        _ => BodyType::Static,
    }
}

#[wasm_bindgen]
impl SimWorld {
    /// Destroy a tracked body and free its demo slot (index stays reserved).
    /// (b2DestroyBody) Used by Sleep / Vertical Stack mid-scene destroy.
    /// Stale joint demo entries are pruned; mouse grab is revalidated.
    pub fn destroy_body(&mut self, index: usize) {
        if index >= self.bodies.len() {
            return;
        }
        let raw = self.bodies[index];
        if raw == DESTROYED_BODY_SLOT {
            return;
        }
        let body_id = self.body_id_at(index);
        destroy_body(&mut self.world, body_id);
        self.bodies[index] = DESTROYED_BODY_SLOT;
        self.joints
            .retain(|&jid| joint_is_valid(&self.world, jid));
        self.grab.validate(&mut self.world);
    }

    /// Whether the demo index still refers to a live body.
    pub fn is_body_alive(&self, index: usize) -> bool {
        index < self.bodies.len() && self.bodies[index] != DESTROYED_BODY_SLOT
    }

    /// Set body origin transform. (b2Body_SetTransform)
    pub fn set_transform(&mut self, index: usize, x: f32, y: f32, angle: f32) {
        let body_id = self.body_id_at(index);
        body_set_transform(
            &mut self.world,
            body_id,
            to_pos(Vec2 { x, y }),
            make_rot(angle),
        );
    }

    /// Body type: 0=static, 1=kinematic, 2=dynamic. (b2Body_SetType)
    pub fn set_body_type(&mut self, index: usize, body_type: i32) {
        let body_id = self.body_id_at(index);
        body_set_type(&mut self.world, body_id, parse_body_type(body_type));
    }

    /// (b2Body_GetType) returns 0/1/2.
    pub fn get_body_type(&self, index: usize) -> i32 {
        let body_id = self.body_id_at(index);
        match body_get_type(&self.world, body_id) {
            BodyType::Static => 0,
            BodyType::Kinematic => 1,
            BodyType::Dynamic => 2,
        }
    }

    /// (b2Body_Enable)
    pub fn enable_body(&mut self, index: usize) {
        let body_id = self.body_id_at(index);
        body_enable(&mut self.world, body_id);
    }

    /// (b2Body_Disable)
    pub fn disable_body(&mut self, index: usize) {
        let body_id = self.body_id_at(index);
        body_disable(&mut self.world, body_id);
    }

    /// (b2Body_IsEnabled)
    pub fn is_body_enabled(&self, index: usize) -> bool {
        let body_id = self.body_id_at(index);
        body_is_enabled(&self.world, body_id)
    }

    /// (b2Body_SetAwake)
    pub fn set_awake(&mut self, index: usize, awake: bool) {
        let body_id = self.body_id_at(index);
        body_set_awake(&mut self.world, body_id, awake);
    }

    /// (b2Body_WakeTouching)
    pub fn wake_touching(&mut self, index: usize) {
        let body_id = self.body_id_at(index);
        body_wake_touching(&mut self.world, body_id);
    }

    /// (b2Body_SetLinearVelocity)
    pub fn set_linear_velocity(&mut self, index: usize, vx: f32, vy: f32) {
        let body_id = self.body_id_at(index);
        body_set_linear_velocity(&mut self.world, body_id, Vec2 { x: vx, y: vy });
    }

    /// (b2Body_GetLinearVelocity) as [vx, vy]
    pub fn get_linear_velocity(&self, index: usize) -> Vec<f32> {
        let v = body_get_linear_velocity(&self.world, self.body_id_at(index));
        vec![v.x, v.y]
    }

    /// (b2Body_SetAngularVelocity)
    pub fn set_angular_velocity(&mut self, index: usize, omega: f32) {
        let body_id = self.body_id_at(index);
        body_set_angular_velocity(&mut self.world, body_id, omega);
    }

    /// (b2Body_GetAngularVelocity)
    pub fn get_angular_velocity(&self, index: usize) -> f32 {
        body_get_angular_velocity(&self.world, self.body_id_at(index))
    }

    /// (b2Body_SetGravityScale)
    pub fn set_gravity_scale(&mut self, index: usize, scale: f32) {
        let body_id = self.body_id_at(index);
        body_set_gravity_scale(&mut self.world, body_id, scale);
    }

    /// (b2Body_GetMass)
    pub fn get_mass(&self, index: usize) -> f32 {
        body_get_mass(&self.world, self.body_id_at(index))
    }

    /// (b2Body_ApplyForce)
    pub fn apply_force(&mut self, index: usize, fx: f32, fy: f32, px: f32, py: f32, wake: bool) {
        let body_id = self.body_id_at(index);
        body_apply_force(
            &mut self.world,
            body_id,
            Vec2 { x: fx, y: fy },
            to_pos(Vec2 { x: px, y: py }),
            wake,
        );
    }

    /// (b2Body_ApplyForceToCenter)
    pub fn apply_force_to_center(&mut self, index: usize, fx: f32, fy: f32, wake: bool) {
        let body_id = self.body_id_at(index);
        body_apply_force_to_center(&mut self.world, body_id, Vec2 { x: fx, y: fy }, wake);
    }

    /// (b2Body_ApplyTorque)
    pub fn apply_torque(&mut self, index: usize, torque: f32, wake: bool) {
        let body_id = self.body_id_at(index);
        body_apply_torque(&mut self.world, body_id, torque, wake);
    }

    /// (b2Body_ApplyLinearImpulse)
    pub fn apply_linear_impulse(
        &mut self,
        index: usize,
        ix: f32,
        iy: f32,
        px: f32,
        py: f32,
        wake: bool,
    ) {
        let body_id = self.body_id_at(index);
        body_apply_linear_impulse(
            &mut self.world,
            body_id,
            Vec2 { x: ix, y: iy },
            to_pos(Vec2 { x: px, y: py }),
            wake,
        );
    }

    /// (b2Body_ApplyLinearImpulseToCenter)
    pub fn apply_linear_impulse_to_center(&mut self, index: usize, ix: f32, iy: f32, wake: bool) {
        let body_id = self.body_id_at(index);
        body_apply_linear_impulse_to_center(
            &mut self.world,
            body_id,
            Vec2 { x: ix, y: iy },
            wake,
        );
    }

    /// (b2Body_SetBullet)
    pub fn set_bullet(&mut self, index: usize, flag: bool) {
        if !self.is_body_alive(index) {
            return;
        }
        let body_id = self.body_id_at(index);
        body_set_bullet(&mut self.world, body_id, flag);
    }

    /// (b2Body_SetLinearDamping)
    pub fn set_linear_damping(&mut self, index: usize, damping: f32) {
        let body_id = self.body_id_at(index);
        body_set_linear_damping(&mut self.world, body_id, damping);
    }

    /// (b2Body_SetAngularDamping)
    pub fn set_angular_damping(&mut self, index: usize, damping: f32) {
        let body_id = self.body_id_at(index);
        body_set_angular_damping(&mut self.world, body_id, damping);
    }

    /// (b2Body_SetMotionLocks) — linearX/Y + angular as bools.
    pub fn set_motion_locks(
        &mut self,
        index: usize,
        linear_x: bool,
        linear_y: bool,
        angular: bool,
    ) {
        let body_id = self.body_id_at(index);
        body_set_motion_locks(
            &mut self.world,
            body_id,
            MotionLocks {
                linear_x,
                linear_y,
                angular_z: angular,
            },
        );
    }

    /// (b2Body_SetTargetTransform) — kinematic/dynamic target for Motor Joint.
    pub fn set_target_transform(
        &mut self,
        index: usize,
        x: f32,
        y: f32,
        angle: f32,
        time_step: f32,
        wake: bool,
    ) {
        let body_id = self.body_id_at(index);
        let target = WorldTransform {
            p: to_pos(Vec2 { x, y }),
            q: make_rot(angle),
        };
        body_set_target_transform(&mut self.world, body_id, target, time_step, wake);
    }
}
