//! Joint mutation / extended construction for sample_joints.cpp DrawControls.
//! Create helpers stay in `joints.rs`; this module owns setters, destroy, and
//! local-frame variants.

use super::SimWorld;
use box2d_rust::body::body_get_local_point;
use box2d_rust::distance_joint::{
    distance_joint_enable_limit, distance_joint_enable_motor, distance_joint_enable_spring,
    distance_joint_set_length, distance_joint_set_length_range, distance_joint_set_max_motor_force,
    distance_joint_set_motor_speed, distance_joint_set_spring_damping_ratio,
    distance_joint_set_spring_force_range, distance_joint_set_spring_hertz,
};
use box2d_rust::joint::{
    create_distance_joint, create_motor_joint, create_prismatic_joint, create_revolute_joint,
    create_weld_joint, create_wheel_joint, destroy_joint, joint_get_constraint_force,
    joint_get_constraint_torque, joint_get_linear_separation, joint_get_angular_separation,
    joint_set_collide_connected, joint_set_constraint_tuning, joint_set_force_threshold,
    joint_set_torque_threshold, joint_wake_bodies,
};
use box2d_rust::math_functions::{
    inv_mul_rot, length, make_rot, make_rot_from_unit_vector, normalize, sub_pos, to_pos, Vec2,
};
use box2d_rust::motor_joint::{
    motor_joint_set_max_spring_force, motor_joint_set_max_spring_torque,
};
use box2d_rust::prismatic_joint::{
    prismatic_joint_enable_limit, prismatic_joint_enable_motor, prismatic_joint_enable_spring,
    prismatic_joint_get_motor_force, prismatic_joint_set_limits, prismatic_joint_set_max_motor_force,
    prismatic_joint_set_motor_speed, prismatic_joint_set_spring_damping_ratio,
    prismatic_joint_set_spring_hertz, prismatic_joint_set_target_translation,
};
use box2d_rust::revolute_joint::{
    revolute_joint_enable_limit, revolute_joint_enable_motor, revolute_joint_enable_spring,
    revolute_joint_get_angle, revolute_joint_get_motor_torque, revolute_joint_set_limits,
    revolute_joint_set_max_motor_torque, revolute_joint_set_motor_speed,
    revolute_joint_set_spring_damping_ratio, revolute_joint_set_spring_hertz,
    revolute_joint_set_target_angle,
};
use box2d_rust::types::{
    default_distance_joint_def, default_motor_joint_def, default_prismatic_joint_def,
    default_revolute_joint_def, default_weld_joint_def, default_wheel_joint_def,
};
use box2d_rust::weld_joint::{
    weld_joint_set_angular_damping_ratio, weld_joint_set_angular_hertz,
    weld_joint_set_linear_damping_ratio, weld_joint_set_linear_hertz,
};
use box2d_rust::wheel_joint::{
    wheel_joint_enable_limit, wheel_joint_enable_motor, wheel_joint_enable_spring,
    wheel_joint_get_motor_torque, wheel_joint_set_limits, wheel_joint_set_max_motor_torque,
    wheel_joint_set_motor_speed, wheel_joint_set_spring_damping_ratio, wheel_joint_set_spring_hertz,
};
use box2d_rust::body::get_body_transform;
use wasm_bindgen::prelude::*;

fn joint_alive(sim: &SimWorld, index: usize) -> bool {
    index < sim.joints.len() && !sim.joints[index].is_null()
}

#[wasm_bindgen]
impl SimWorld {
    /// Distance joint with full spring/limit params (Distance Joint sample).
    #[allow(clippy::too_many_arguments)]
    pub fn add_distance_joint_ex(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
        length_override: f32,
        enable_spring: bool,
        hertz: f32,
        damping_ratio: f32,
        tension_force: f32,
        compression_force: f32,
        enable_limit: bool,
        min_length: f32,
        max_length: f32,
        collide_connected: bool,
    ) -> usize {
        let anchor_a = to_pos(Vec2 { x: ax, y: ay });
        let anchor_b = to_pos(Vec2 { x: bx, y: by });
        let body_a = self.body_id_at(index_a);
        let body_b = self.body_id_at(index_b);

        let mut joint_def = default_distance_joint_def();
        joint_def.base.body_id_a = body_a;
        joint_def.base.body_id_b = body_b;
        joint_def.base.local_frame_a.p = body_get_local_point(&self.world, body_a, anchor_a);
        joint_def.base.local_frame_b.p = body_get_local_point(&self.world, body_b, anchor_b);
        joint_def.base.collide_connected = collide_connected;
        joint_def.length = if length_override > 0.0 {
            length_override
        } else {
            length(sub_pos(anchor_b, anchor_a))
        };
        joint_def.enable_spring = enable_spring;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;
        joint_def.lower_spring_force = -tension_force;
        joint_def.upper_spring_force = compression_force;
        joint_def.enable_limit = enable_limit;
        joint_def.min_length = min_length;
        joint_def.max_length = max_length;

        let joint_id = create_distance_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Motor joint with explicit local-frame points (body-local).
    #[allow(clippy::too_many_arguments)]
    pub fn add_motor_joint_local(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
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
        let mut joint_def = default_motor_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        joint_def.base.local_frame_a.p = to_pos(Vec2 { x: ax, y: ay });
        joint_def.base.local_frame_b.p = to_pos(Vec2 { x: bx, y: by });
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

    /// Revolute using body-local anchors (Doohickey / local-frame samples).
    #[allow(clippy::too_many_arguments)]
    pub fn add_revolute_joint_local(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
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
        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        joint_def.base.local_frame_a.p = to_pos(Vec2 { x: ax, y: ay });
        joint_def.base.local_frame_b.p = to_pos(Vec2 { x: bx, y: by });
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

    /// Prismatic using body-local anchors + world axis.
    #[allow(clippy::too_many_arguments)]
    pub fn add_prismatic_joint_local(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
        world_ax: f32,
        world_ay: f32,
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
        let axis = normalize(Vec2 {
            x: world_ax,
            y: world_ay,
        });
        let q = make_rot_from_unit_vector(axis);
        let mut joint_def = default_prismatic_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        joint_def.base.local_frame_a.p = to_pos(Vec2 { x: ax, y: ay });
        joint_def.base.local_frame_b.p = to_pos(Vec2 { x: bx, y: by });
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

    /// Weld with body-local anchors and relative rotation (Soft Body donut).
    #[allow(clippy::too_many_arguments)]
    pub fn add_weld_joint_local(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
        angle_a: f32,
        angle_b: f32,
        linear_hertz: f32,
        angular_hertz: f32,
        linear_damping_ratio: f32,
        angular_damping_ratio: f32,
        collide_connected: bool,
    ) -> usize {
        let mut joint_def = default_weld_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        joint_def.base.local_frame_a.p = to_pos(Vec2 { x: ax, y: ay });
        joint_def.base.local_frame_b.p = to_pos(Vec2 { x: bx, y: by });
        // C: localFrameA.q = InvMulRot(qA, qB)
        joint_def.base.local_frame_a.q = inv_mul_rot(make_rot(angle_a), make_rot(angle_b));
        joint_def.base.collide_connected = collide_connected;
        joint_def.linear_hertz = linear_hertz;
        joint_def.angular_hertz = angular_hertz;
        joint_def.linear_damping_ratio = linear_damping_ratio;
        joint_def.angular_damping_ratio = angular_damping_ratio;
        let joint_id = create_weld_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// (b2DestroyJoint) — demo index slot retained.
    pub fn destroy_joint(&mut self, index: usize) {
        if !joint_alive(self, index) {
            return;
        }
        let joint_id = self.joints[index];
        destroy_joint(&mut self.world, joint_id, false);
        self.joints[index] = box2d_rust::id::JointId::default();
    }

    pub fn joint_wake_bodies(&mut self, index: usize) {
        if !joint_alive(self, index) {
            return;
        }
        joint_wake_bodies(&mut self.world, self.joints[index]);
    }

    pub fn joint_set_constraint_tuning(&mut self, index: usize, hertz: f32, damping_ratio: f32) {
        if !joint_alive(self, index) {
            return;
        }
        joint_set_constraint_tuning(&mut self.world, self.joints[index], hertz, damping_ratio);
    }

    pub fn joint_set_collide_connected(&mut self, index: usize, flag: bool) {
        if !joint_alive(self, index) {
            return;
        }
        joint_set_collide_connected(&mut self.world, self.joints[index], flag);
    }

    pub fn joint_set_force_threshold(&mut self, index: usize, threshold: f32) {
        if !joint_alive(self, index) {
            return;
        }
        joint_set_force_threshold(&mut self.world, self.joints[index], threshold);
    }

    pub fn joint_set_torque_threshold(&mut self, index: usize, threshold: f32) {
        if !joint_alive(self, index) {
            return;
        }
        joint_set_torque_threshold(&mut self.world, self.joints[index], threshold);
    }

    /// [fx, fy, torque]
    pub fn joint_constraint_ft(&self, index: usize) -> Vec<f32> {
        if !joint_alive(self, index) {
            return vec![0.0, 0.0, 0.0];
        }
        let f = joint_get_constraint_force(&self.world, self.joints[index]);
        let t = joint_get_constraint_torque(&self.world, self.joints[index]);
        vec![f.x, f.y, t]
    }

    pub fn joint_separations(&self, index: usize) -> Vec<f32> {
        if !joint_alive(self, index) {
            return vec![0.0, 0.0];
        }
        let lin = joint_get_linear_separation(&self.world, self.joints[index]);
        let ang = joint_get_angular_separation(&self.world, self.joints[index]);
        vec![lin, ang]
    }

    // --- distance setters ---
    pub fn distance_set_length(&mut self, index: usize, length: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_length(&mut self.world, self.joints[index], length);
    }
    pub fn distance_enable_spring(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_enable_spring(&mut self.world, self.joints[index], enable);
    }
    pub fn distance_set_spring_hertz(&mut self, index: usize, hertz: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_spring_hertz(&mut self.world, self.joints[index], hertz);
    }
    pub fn distance_set_spring_damping(&mut self, index: usize, damping: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_spring_damping_ratio(&mut self.world, self.joints[index], damping);
    }
    pub fn distance_set_spring_force_range(&mut self, index: usize, lower: f32, upper: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_spring_force_range(&mut self.world, self.joints[index], lower, upper);
    }
    pub fn distance_enable_limit(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_enable_limit(&mut self.world, self.joints[index], enable);
    }
    pub fn distance_set_length_range(&mut self, index: usize, min_l: f32, max_l: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_length_range(&mut self.world, self.joints[index], min_l, max_l);
    }
    pub fn distance_enable_motor(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_enable_motor(&mut self.world, self.joints[index], enable);
    }
    pub fn distance_set_motor_speed(&mut self, index: usize, speed: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_motor_speed(&mut self.world, self.joints[index], speed);
    }
    pub fn distance_set_max_motor_force(&mut self, index: usize, force: f32) {
        if !joint_alive(self, index) {
            return;
        }
        distance_joint_set_max_motor_force(&mut self.world, self.joints[index], force);
    }

    /// Distance joint with body-local anchors + motor (Scissor Lift).
    #[allow(clippy::too_many_arguments)]
    pub fn add_distance_joint_local_motor(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
        length_override: f32,
        enable_spring: bool,
        hertz: f32,
        damping_ratio: f32,
        enable_limit: bool,
        min_length: f32,
        max_length: f32,
        enable_motor: bool,
        motor_speed: f32,
        max_motor_force: f32,
        collide_connected: bool,
    ) -> usize {
        let mut joint_def = default_distance_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        joint_def.base.local_frame_a.p = to_pos(Vec2 { x: ax, y: ay });
        joint_def.base.local_frame_b.p = to_pos(Vec2 { x: bx, y: by });
        joint_def.base.collide_connected = collide_connected;
        if length_override > 0.0 {
            joint_def.length = length_override;
        } else {
            use box2d_rust::body::body_get_transform;
            use box2d_rust::math_functions::transform_world_point;
            let wa = transform_world_point(
                body_get_transform(&self.world, joint_def.base.body_id_a),
                to_pos(Vec2 { x: ax, y: ay }),
            );
            let wb = transform_world_point(
                body_get_transform(&self.world, joint_def.base.body_id_b),
                to_pos(Vec2 { x: bx, y: by }),
            );
            joint_def.length = length(sub_pos(wb, wa));
        }
        joint_def.enable_spring = enable_spring;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;
        joint_def.enable_limit = enable_limit;
        joint_def.min_length = min_length;
        joint_def.max_length = max_length;
        joint_def.enable_motor = enable_motor;
        joint_def.motor_speed = motor_speed;
        joint_def.max_motor_force = max_motor_force;
        let joint_id = create_distance_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    /// Wheel joint with body-local anchors (Scissor Lift). Axis = local +X of A.
    #[allow(clippy::too_many_arguments)]
    pub fn add_wheel_joint_local(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
        enable_spring: bool,
        hertz: f32,
        damping_ratio: f32,
        collide_connected: bool,
    ) -> usize {
        let mut joint_def = default_wheel_joint_def();
        joint_def.base.body_id_a = self.body_id_at(index_a);
        joint_def.base.body_id_b = self.body_id_at(index_b);
        joint_def.base.local_frame_a.p = to_pos(Vec2 { x: ax, y: ay });
        joint_def.base.local_frame_b.p = to_pos(Vec2 { x: bx, y: by });
        joint_def.base.collide_connected = collide_connected;
        joint_def.enable_spring = enable_spring;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;
        let joint_id = create_wheel_joint(&mut self.world, &joint_def);
        self.track_joint(joint_id)
    }

    // --- revolute setters ---
    pub fn revolute_enable_limit(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_enable_limit(&mut self.world, self.joints[index], enable);
    }
    pub fn revolute_enable_motor(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_enable_motor(&mut self.world, self.joints[index], enable);
    }
    pub fn revolute_enable_spring(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_enable_spring(&mut self.world, self.joints[index], enable);
    }
    pub fn revolute_set_motor_speed(&mut self, index: usize, speed: f32) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_set_motor_speed(&mut self.world, self.joints[index], speed);
    }
    pub fn revolute_set_max_motor_torque(&mut self, index: usize, torque: f32) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_set_max_motor_torque(&mut self.world, self.joints[index], torque);
    }
    pub fn revolute_set_spring_hertz(&mut self, index: usize, hertz: f32) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_set_spring_hertz(&mut self.world, self.joints[index], hertz);
    }
    pub fn revolute_set_spring_damping(&mut self, index: usize, damping: f32) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_set_spring_damping_ratio(&mut self.world, self.joints[index], damping);
    }
    pub fn revolute_set_target_angle(&mut self, index: usize, angle: f32) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_set_target_angle(&mut self.world, self.joints[index], angle);
    }
    pub fn revolute_set_limits(&mut self, index: usize, lower: f32, upper: f32) {
        if !joint_alive(self, index) {
            return;
        }
        revolute_joint_set_limits(&mut self.world, self.joints[index], lower, upper);
    }
    pub fn revolute_get_angle(&self, index: usize) -> f32 {
        if !joint_alive(self, index) {
            return 0.0;
        }
        revolute_joint_get_angle(&self.world, self.joints[index])
    }
    pub fn revolute_get_motor_torque(&self, index: usize) -> f32 {
        if !joint_alive(self, index) {
            return 0.0;
        }
        revolute_joint_get_motor_torque(&self.world, self.joints[index])
    }

    // --- prismatic ---
    pub fn prismatic_enable_limit(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_enable_limit(&mut self.world, self.joints[index], enable);
    }
    pub fn prismatic_enable_motor(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_enable_motor(&mut self.world, self.joints[index], enable);
    }
    pub fn prismatic_enable_spring(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_enable_spring(&mut self.world, self.joints[index], enable);
    }
    pub fn prismatic_set_motor_speed(&mut self, index: usize, speed: f32) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_set_motor_speed(&mut self.world, self.joints[index], speed);
    }
    pub fn prismatic_set_max_motor_force(&mut self, index: usize, force: f32) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_set_max_motor_force(&mut self.world, self.joints[index], force);
    }
    pub fn prismatic_set_spring_hertz(&mut self, index: usize, hertz: f32) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_set_spring_hertz(&mut self.world, self.joints[index], hertz);
    }
    pub fn prismatic_set_spring_damping(&mut self, index: usize, damping: f32) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_set_spring_damping_ratio(&mut self.world, self.joints[index], damping);
    }
    pub fn prismatic_set_target_translation(&mut self, index: usize, translation: f32) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_set_target_translation(&mut self.world, self.joints[index], translation);
    }
    pub fn prismatic_set_limits(&mut self, index: usize, lower: f32, upper: f32) {
        if !joint_alive(self, index) {
            return;
        }
        prismatic_joint_set_limits(&mut self.world, self.joints[index], lower, upper);
    }
    pub fn prismatic_get_motor_force(&self, index: usize) -> f32 {
        if !joint_alive(self, index) {
            return 0.0;
        }
        prismatic_joint_get_motor_force(&self.world, self.joints[index])
    }

    // --- wheel ---
    pub fn wheel_enable_limit(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_enable_limit(&mut self.world, self.joints[index], enable);
    }
    pub fn wheel_enable_motor(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_enable_motor(&mut self.world, self.joints[index], enable);
    }
    pub fn wheel_enable_spring(&mut self, index: usize, enable: bool) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_enable_spring(&mut self.world, self.joints[index], enable);
    }
    pub fn wheel_set_motor_speed(&mut self, index: usize, speed: f32) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_set_motor_speed(&mut self.world, self.joints[index], speed);
    }
    pub fn wheel_set_max_motor_torque(&mut self, index: usize, torque: f32) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_set_max_motor_torque(&mut self.world, self.joints[index], torque);
    }
    pub fn wheel_set_spring_hertz(&mut self, index: usize, hertz: f32) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_set_spring_hertz(&mut self.world, self.joints[index], hertz);
    }
    pub fn wheel_set_spring_damping(&mut self, index: usize, damping: f32) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_set_spring_damping_ratio(&mut self.world, self.joints[index], damping);
    }
    pub fn wheel_set_limits(&mut self, index: usize, lower: f32, upper: f32) {
        if !joint_alive(self, index) {
            return;
        }
        wheel_joint_set_limits(&mut self.world, self.joints[index], lower, upper);
    }
    pub fn wheel_get_motor_torque(&self, index: usize) -> f32 {
        if !joint_alive(self, index) {
            return 0.0;
        }
        wheel_joint_get_motor_torque(&self.world, self.joints[index])
    }

    // --- weld ---
    pub fn weld_set_linear_hertz(&mut self, index: usize, hertz: f32) {
        if !joint_alive(self, index) {
            return;
        }
        weld_joint_set_linear_hertz(&mut self.world, self.joints[index], hertz);
    }
    pub fn weld_set_angular_hertz(&mut self, index: usize, hertz: f32) {
        if !joint_alive(self, index) {
            return;
        }
        weld_joint_set_angular_hertz(&mut self.world, self.joints[index], hertz);
    }
    pub fn weld_set_linear_damping(&mut self, index: usize, damping: f32) {
        if !joint_alive(self, index) {
            return;
        }
        weld_joint_set_linear_damping_ratio(&mut self.world, self.joints[index], damping);
    }
    pub fn weld_set_angular_damping(&mut self, index: usize, damping: f32) {
        if !joint_alive(self, index) {
            return;
        }
        weld_joint_set_angular_damping_ratio(&mut self.world, self.joints[index], damping);
    }

    // --- motor ---
    pub fn motor_set_max_spring_force(&mut self, index: usize, force: f32) {
        if !joint_alive(self, index) {
            return;
        }
        motor_joint_set_max_spring_force(&mut self.world, self.joints[index], force);
    }
    pub fn motor_set_max_spring_torque(&mut self, index: usize, torque: f32) {
        if !joint_alive(self, index) {
            return;
        }
        motor_joint_set_max_spring_torque(&mut self.world, self.joints[index], torque);
    }

    /// World point from body-local (for overlay).
    pub fn body_world_point(&self, index: usize, lx: f32, ly: f32) -> Vec<f32> {
        let xf = get_body_transform(&self.world, self.body_index_at(index));
        let p = box2d_rust::math_functions::transform_world_point(xf, to_pos(Vec2 { x: lx, y: ly }));
        vec![p.x, p.y]
    }
}
