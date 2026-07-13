//! Joint construction APIs beyond the existing hinge/distance helpers.
//! Mirrors b2Create*Joint patterns used by sample_joints.cpp / sample_bodies.cpp.

use super::SimWorld;
use box2d_rust::body::{body_get_local_point, get_body_transform};
use box2d_rust::joint::{
    create_filter_joint, create_motor_joint, create_prismatic_joint, create_revolute_joint,
    create_weld_joint, create_wheel_joint,
};
use box2d_rust::math_functions::{
    inv_transform_world_point, make_rot, make_rot_from_unit_vector, normalize, to_pos, Vec2,
};
use box2d_rust::types::{
    default_filter_joint_def, default_motor_joint_def, default_prismatic_joint_def,
    default_revolute_joint_def, default_weld_joint_def, default_wheel_joint_def,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl SimWorld {
    /// Revolute joint at a world pivot with explicit limit/motor/spring params.
    /// (b2CreateRevoluteJoint) Unlike `add_hinge_joint`, tuning is caller-driven.
    #[allow(clippy::too_many_arguments)]
    pub fn add_revolute_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        px: f32,
        py: f32,
        enable_limit: bool,
        lower_angle: f32,
        upper_angle: f32,
        enable_motor: bool,
        motor_speed: f32,
        max_motor_torque: f32,
        enable_spring: bool,
        hertz: f32,
        damping_ratio: f32,
        collide_connected: bool,
    ) -> usize {
        let pivot = to_pos(Vec2 { x: px, y: py });
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);
        let xf_a = get_body_transform(&self.world, self.body_index_at(index_a));
        let xf_b = get_body_transform(&self.world, self.body_index_at(index_b));

        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.local_frame_a.p = inv_transform_world_point(xf_a, pivot);
        joint_def.base.local_frame_b.p = inv_transform_world_point(xf_b, pivot);
        joint_def.base.collide_connected = collide_connected;
        joint_def.enable_limit = enable_limit;
        joint_def.lower_angle = lower_angle;
        joint_def.upper_angle = upper_angle;
        joint_def.enable_motor = enable_motor;
        joint_def.motor_speed = motor_speed;
        joint_def.max_motor_torque = max_motor_torque;
        joint_def.enable_spring = enable_spring;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;

        let joint_id = create_revolute_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Prismatic joint along world-space axis through a world pivot.
    /// Axis `(ax, ay)` is normalized. (b2CreatePrismaticJoint)
    #[allow(clippy::too_many_arguments)]
    pub fn add_prismatic_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        px: f32,
        py: f32,
        ax: f32,
        ay: f32,
        enable_limit: bool,
        lower: f32,
        upper: f32,
        enable_motor: bool,
        motor_speed: f32,
        max_motor_force: f32,
        enable_spring: bool,
        hertz: f32,
        damping_ratio: f32,
        collide_connected: bool,
    ) -> usize {
        let pivot = to_pos(Vec2 { x: px, y: py });
        let axis = normalize(Vec2 { x: ax, y: ay });
        let q = make_rot_from_unit_vector(axis);
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);

        let mut joint_def = default_prismatic_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.local_frame_a.p = body_get_local_point(&self.world, body_a, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&self.world, body_b, pivot);
        joint_def.base.local_frame_a.q = q;
        joint_def.base.local_frame_b.q = q;
        joint_def.base.collide_connected = collide_connected;
        joint_def.enable_limit = enable_limit;
        joint_def.lower_translation = lower;
        joint_def.upper_translation = upper;
        joint_def.enable_motor = enable_motor;
        joint_def.motor_speed = motor_speed;
        joint_def.max_motor_force = max_motor_force;
        joint_def.enable_spring = enable_spring;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;

        let joint_id = create_prismatic_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Wheel joint along world-space axis through a world pivot.
    /// (b2CreateWheelJoint)
    #[allow(clippy::too_many_arguments)]
    pub fn add_wheel_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        px: f32,
        py: f32,
        ax: f32,
        ay: f32,
        enable_limit: bool,
        lower: f32,
        upper: f32,
        enable_motor: bool,
        motor_speed: f32,
        max_motor_torque: f32,
        enable_spring: bool,
        hertz: f32,
        damping_ratio: f32,
        collide_connected: bool,
    ) -> usize {
        let pivot = to_pos(Vec2 { x: px, y: py });
        let axis = normalize(Vec2 { x: ax, y: ay });
        let q = make_rot_from_unit_vector(axis);
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);

        let mut joint_def = default_wheel_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.local_frame_a.p = body_get_local_point(&self.world, body_a, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&self.world, body_b, pivot);
        joint_def.base.local_frame_a.q = q;
        joint_def.base.collide_connected = collide_connected;
        joint_def.enable_limit = enable_limit;
        joint_def.lower_translation = lower;
        joint_def.upper_translation = upper;
        joint_def.enable_motor = enable_motor;
        joint_def.motor_speed = motor_speed;
        joint_def.max_motor_torque = max_motor_torque;
        joint_def.enable_spring = enable_spring;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;

        let joint_id = create_wheel_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Weld two bodies at a world pivot with optional soft stiffness.
    /// Hertz of 0 is maximum stiffness (C default). (b2CreateWeldJoint)
    pub fn add_weld_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        px: f32,
        py: f32,
        linear_hertz: f32,
        angular_hertz: f32,
        linear_damping_ratio: f32,
        angular_damping_ratio: f32,
        collide_connected: bool,
    ) -> usize {
        let pivot = to_pos(Vec2 { x: px, y: py });
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);

        let mut joint_def = default_weld_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.local_frame_a.p = body_get_local_point(&self.world, body_a, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&self.world, body_b, pivot);
        joint_def.base.collide_connected = collide_connected;
        joint_def.linear_hertz = linear_hertz;
        joint_def.angular_hertz = angular_hertz;
        joint_def.linear_damping_ratio = linear_damping_ratio;
        joint_def.angular_damping_ratio = angular_damping_ratio;

        let joint_id = create_weld_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Motor joint controlling relative motion between two bodies.
    /// Frames stay at body origins (common for Motor Joint sample).
    /// (b2CreateMotorJoint)
    pub fn add_motor_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        linear_hertz: f32,
        linear_damping_ratio: f32,
        max_spring_force: f32,
        angular_hertz: f32,
        angular_damping_ratio: f32,
        max_spring_torque: f32,
        max_velocity_force: f32,
        max_velocity_torque: f32,
        collide_connected: bool,
    ) -> usize {
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);

        let mut joint_def = default_motor_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.collide_connected = collide_connected;
        joint_def.linear_hertz = linear_hertz;
        joint_def.linear_damping_ratio = linear_damping_ratio;
        joint_def.max_spring_force = max_spring_force;
        joint_def.angular_hertz = angular_hertz;
        joint_def.angular_damping_ratio = angular_damping_ratio;
        joint_def.max_spring_torque = max_spring_torque;
        joint_def.max_velocity_force = max_velocity_force;
        joint_def.max_velocity_torque = max_velocity_torque;

        let joint_id = create_motor_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Disable collision between two specific bodies. (b2CreateFilterJoint)
    pub fn add_filter_joint(&mut self, index_a: usize, index_b: usize) -> usize {
        let mut joint_def = default_filter_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        let joint_id = create_filter_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Revolute with an explicit local-frame angle on body A (e.g. 90° hinge).
    pub fn add_revolute_joint_angled(
        &mut self,
        index_a: usize,
        index_b: usize,
        px: f32,
        py: f32,
        frame_angle_a: f32,
        enable_limit: bool,
        lower_angle: f32,
        upper_angle: f32,
        enable_motor: bool,
        max_motor_torque: f32,
    ) -> usize {
        let pivot = to_pos(Vec2 { x: px, y: py });
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);

        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.local_frame_a.p = body_get_local_point(&self.world, body_a, pivot);
        joint_def.base.local_frame_a.q = make_rot(frame_angle_a);
        joint_def.base.local_frame_b.p = body_get_local_point(&self.world, body_b, pivot);
        joint_def.enable_limit = enable_limit;
        joint_def.lower_angle = lower_angle;
        joint_def.upper_angle = upper_angle;
        joint_def.enable_motor = enable_motor;
        joint_def.max_motor_torque = max_motor_torque;

        let joint_id = create_revolute_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }
}
