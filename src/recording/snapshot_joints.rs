// Joint serialize/deserialize for the world snapshot: the Joint bookkeeping
// struct, JointSim, and all six payload variants. Split from
// snapshot_structs.rs to stay under the file-length limit.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// See snapshot_structs.rs: assignment order IS the wire order.
#![allow(clippy::field_reassign_with_default)]

use super::snapshot::SnapReader;
use super::snapshot_structs::{r_mat22, r_softness, r_vec2, r_xf, w_mat22, w_softness};
use super::write::*;
use crate::joint::{Joint, JointEdge, JointPayload, JointSim, JointType};

// Joints

fn w_joint_edge(buf: &mut Vec<u8>, e: &JointEdge) {
    rec_w_i32(buf, e.body_id);
    rec_w_i32(buf, e.prev_key);
    rec_w_i32(buf, e.next_key);
}

fn r_joint_edge(r: &mut SnapReader) -> JointEdge {
    JointEdge {
        body_id: r.r_i32(),
        prev_key: r.r_i32(),
        next_key: r.r_i32(),
    }
}

fn r_joint_type(r: &mut SnapReader) -> JointType {
    match r.r_u8() {
        0 => JointType::Distance,
        1 => JointType::Filter,
        2 => JointType::Motor,
        3 => JointType::Prismatic,
        4 => JointType::Revolute,
        5 => JointType::Weld,
        _ => JointType::Wheel,
    }
}

pub(crate) fn ser_joint(buf: &mut Vec<u8>, j: &Joint) {
    // userData: not preserved
    rec_w_u64(buf, 0);
    rec_w_i32(buf, j.set_index);
    rec_w_i32(buf, j.color_index);
    rec_w_i32(buf, j.local_index);
    w_joint_edge(buf, &j.edges[0]);
    w_joint_edge(buf, &j.edges[1]);
    rec_w_i32(buf, j.joint_id);
    rec_w_i32(buf, j.island_id);
    rec_w_i32(buf, j.island_index);
    rec_w_f32(buf, j.draw_scale);
    rec_w_u8(buf, j.type_ as u8);
    rec_w_u16(buf, j.generation);
    rec_w_bool(buf, j.collide_connected);
}

pub(crate) fn des_joint(r: &mut SnapReader) -> Joint {
    let mut j = Joint::default();
    j.user_data = r.r_u64();
    j.set_index = r.r_i32();
    j.color_index = r.r_i32();
    j.local_index = r.r_i32();
    j.edges = [r_joint_edge(r), r_joint_edge(r)];
    j.joint_id = r.r_i32();
    j.island_id = r.r_i32();
    j.island_index = r.r_i32();
    j.draw_scale = r.r_f32();
    j.type_ = r_joint_type(r);
    j.generation = r.r_u16();
    j.collide_connected = r.r_bool();
    j
}

fn ser_payload(buf: &mut Vec<u8>, p: &JointPayload) {
    match p {
        JointPayload::Distance(d) => {
            rec_w_u8(buf, 0);
            rec_w_f32(buf, d.length);
            rec_w_f32(buf, d.hertz);
            rec_w_f32(buf, d.damping_ratio);
            rec_w_f32(buf, d.lower_spring_force);
            rec_w_f32(buf, d.upper_spring_force);
            rec_w_f32(buf, d.min_length);
            rec_w_f32(buf, d.max_length);
            rec_w_f32(buf, d.max_motor_force);
            rec_w_f32(buf, d.motor_speed);
            rec_w_f32(buf, d.impulse);
            rec_w_f32(buf, d.lower_impulse);
            rec_w_f32(buf, d.upper_impulse);
            rec_w_f32(buf, d.motor_impulse);
            rec_w_i32(buf, d.index_a);
            rec_w_i32(buf, d.index_b);
            rec_w_vec2(buf, d.anchor_a);
            rec_w_vec2(buf, d.anchor_b);
            rec_w_vec2(buf, d.delta_center);
            w_softness(buf, d.distance_softness);
            rec_w_f32(buf, d.axial_mass);
            rec_w_bool(buf, d.enable_spring);
            rec_w_bool(buf, d.enable_limit);
            rec_w_bool(buf, d.enable_motor);
        }
        JointPayload::Filter => rec_w_u8(buf, 1),
        JointPayload::Motor(m) => {
            rec_w_u8(buf, 2);
            rec_w_vec2(buf, m.linear_velocity);
            rec_w_f32(buf, m.max_velocity_force);
            rec_w_f32(buf, m.angular_velocity);
            rec_w_f32(buf, m.max_velocity_torque);
            rec_w_f32(buf, m.linear_hertz);
            rec_w_f32(buf, m.linear_damping_ratio);
            rec_w_f32(buf, m.max_spring_force);
            rec_w_f32(buf, m.angular_hertz);
            rec_w_f32(buf, m.angular_damping_ratio);
            rec_w_f32(buf, m.max_spring_torque);
            rec_w_vec2(buf, m.linear_velocity_impulse);
            rec_w_f32(buf, m.angular_velocity_impulse);
            rec_w_vec2(buf, m.linear_spring_impulse);
            rec_w_f32(buf, m.angular_spring_impulse);
            w_softness(buf, m.linear_spring);
            w_softness(buf, m.angular_spring);
            rec_w_i32(buf, m.index_a);
            rec_w_i32(buf, m.index_b);
            rec_w_xf(buf, m.frame_a);
            rec_w_xf(buf, m.frame_b);
            rec_w_vec2(buf, m.delta_center);
            w_mat22(buf, m.linear_mass);
            rec_w_f32(buf, m.angular_mass);
        }
        JointPayload::Prismatic(p) => {
            rec_w_u8(buf, 3);
            rec_w_vec2(buf, p.impulse);
            rec_w_f32(buf, p.spring_impulse);
            rec_w_f32(buf, p.motor_impulse);
            rec_w_f32(buf, p.lower_impulse);
            rec_w_f32(buf, p.upper_impulse);
            rec_w_f32(buf, p.hertz);
            rec_w_f32(buf, p.damping_ratio);
            rec_w_f32(buf, p.target_translation);
            rec_w_f32(buf, p.max_motor_force);
            rec_w_f32(buf, p.motor_speed);
            rec_w_f32(buf, p.lower_translation);
            rec_w_f32(buf, p.upper_translation);
            rec_w_i32(buf, p.index_a);
            rec_w_i32(buf, p.index_b);
            rec_w_xf(buf, p.frame_a);
            rec_w_xf(buf, p.frame_b);
            rec_w_vec2(buf, p.delta_center);
            w_softness(buf, p.spring_softness);
            rec_w_bool(buf, p.enable_spring);
            rec_w_bool(buf, p.enable_limit);
            rec_w_bool(buf, p.enable_motor);
        }
        JointPayload::Revolute(rv) => {
            rec_w_u8(buf, 4);
            rec_w_vec2(buf, rv.linear_impulse);
            rec_w_f32(buf, rv.spring_impulse);
            rec_w_f32(buf, rv.motor_impulse);
            rec_w_f32(buf, rv.lower_impulse);
            rec_w_f32(buf, rv.upper_impulse);
            rec_w_f32(buf, rv.hertz);
            rec_w_f32(buf, rv.damping_ratio);
            rec_w_f32(buf, rv.target_angle);
            rec_w_f32(buf, rv.max_motor_torque);
            rec_w_f32(buf, rv.motor_speed);
            rec_w_f32(buf, rv.lower_angle);
            rec_w_f32(buf, rv.upper_angle);
            rec_w_i32(buf, rv.index_a);
            rec_w_i32(buf, rv.index_b);
            rec_w_xf(buf, rv.frame_a);
            rec_w_xf(buf, rv.frame_b);
            rec_w_vec2(buf, rv.delta_center);
            rec_w_f32(buf, rv.axial_mass);
            w_softness(buf, rv.spring_softness);
            rec_w_bool(buf, rv.enable_spring);
            rec_w_bool(buf, rv.enable_motor);
            rec_w_bool(buf, rv.enable_limit);
        }
        JointPayload::Weld(w) => {
            rec_w_u8(buf, 5);
            rec_w_f32(buf, w.linear_hertz);
            rec_w_f32(buf, w.linear_damping_ratio);
            rec_w_f32(buf, w.angular_hertz);
            rec_w_f32(buf, w.angular_damping_ratio);
            w_softness(buf, w.linear_spring);
            w_softness(buf, w.angular_spring);
            rec_w_vec2(buf, w.linear_impulse);
            rec_w_f32(buf, w.angular_impulse);
            rec_w_i32(buf, w.index_a);
            rec_w_i32(buf, w.index_b);
            rec_w_xf(buf, w.frame_a);
            rec_w_xf(buf, w.frame_b);
            rec_w_vec2(buf, w.delta_center);
            rec_w_f32(buf, w.axial_mass);
        }
        JointPayload::Wheel(w) => {
            rec_w_u8(buf, 6);
            rec_w_f32(buf, w.perp_impulse);
            rec_w_f32(buf, w.motor_impulse);
            rec_w_f32(buf, w.spring_impulse);
            rec_w_f32(buf, w.lower_impulse);
            rec_w_f32(buf, w.upper_impulse);
            rec_w_f32(buf, w.max_motor_torque);
            rec_w_f32(buf, w.motor_speed);
            rec_w_f32(buf, w.lower_translation);
            rec_w_f32(buf, w.upper_translation);
            rec_w_f32(buf, w.hertz);
            rec_w_f32(buf, w.damping_ratio);
            rec_w_i32(buf, w.index_a);
            rec_w_i32(buf, w.index_b);
            rec_w_xf(buf, w.frame_a);
            rec_w_xf(buf, w.frame_b);
            rec_w_vec2(buf, w.delta_center);
            rec_w_f32(buf, w.perp_mass);
            rec_w_f32(buf, w.motor_mass);
            rec_w_f32(buf, w.axial_mass);
            w_softness(buf, w.spring_softness);
            rec_w_bool(buf, w.enable_spring);
            rec_w_bool(buf, w.enable_motor);
            rec_w_bool(buf, w.enable_limit);
        }
    }
}

fn des_payload(r: &mut SnapReader) -> JointPayload {
    use crate::joint::{
        DistanceJoint, MotorJoint, PrismaticJoint, RevoluteJoint, WeldJoint, WheelJoint,
    };
    match r.r_u8() {
        0 => {
            let mut d = DistanceJoint::default();
            d.length = r.r_f32();
            d.hertz = r.r_f32();
            d.damping_ratio = r.r_f32();
            d.lower_spring_force = r.r_f32();
            d.upper_spring_force = r.r_f32();
            d.min_length = r.r_f32();
            d.max_length = r.r_f32();
            d.max_motor_force = r.r_f32();
            d.motor_speed = r.r_f32();
            d.impulse = r.r_f32();
            d.lower_impulse = r.r_f32();
            d.upper_impulse = r.r_f32();
            d.motor_impulse = r.r_f32();
            d.index_a = r.r_i32();
            d.index_b = r.r_i32();
            d.anchor_a = r_vec2(r);
            d.anchor_b = r_vec2(r);
            d.delta_center = r_vec2(r);
            d.distance_softness = r_softness(r);
            d.axial_mass = r.r_f32();
            d.enable_spring = r.r_bool();
            d.enable_limit = r.r_bool();
            d.enable_motor = r.r_bool();
            JointPayload::Distance(d)
        }
        1 => JointPayload::Filter,
        2 => {
            let mut m = MotorJoint::default();
            m.linear_velocity = r_vec2(r);
            m.max_velocity_force = r.r_f32();
            m.angular_velocity = r.r_f32();
            m.max_velocity_torque = r.r_f32();
            m.linear_hertz = r.r_f32();
            m.linear_damping_ratio = r.r_f32();
            m.max_spring_force = r.r_f32();
            m.angular_hertz = r.r_f32();
            m.angular_damping_ratio = r.r_f32();
            m.max_spring_torque = r.r_f32();
            m.linear_velocity_impulse = r_vec2(r);
            m.angular_velocity_impulse = r.r_f32();
            m.linear_spring_impulse = r_vec2(r);
            m.angular_spring_impulse = r.r_f32();
            m.linear_spring = r_softness(r);
            m.angular_spring = r_softness(r);
            m.index_a = r.r_i32();
            m.index_b = r.r_i32();
            m.frame_a = r_xf(r);
            m.frame_b = r_xf(r);
            m.delta_center = r_vec2(r);
            m.linear_mass = r_mat22(r);
            m.angular_mass = r.r_f32();
            JointPayload::Motor(m)
        }
        3 => {
            let mut p = PrismaticJoint::default();
            p.impulse = r_vec2(r);
            p.spring_impulse = r.r_f32();
            p.motor_impulse = r.r_f32();
            p.lower_impulse = r.r_f32();
            p.upper_impulse = r.r_f32();
            p.hertz = r.r_f32();
            p.damping_ratio = r.r_f32();
            p.target_translation = r.r_f32();
            p.max_motor_force = r.r_f32();
            p.motor_speed = r.r_f32();
            p.lower_translation = r.r_f32();
            p.upper_translation = r.r_f32();
            p.index_a = r.r_i32();
            p.index_b = r.r_i32();
            p.frame_a = r_xf(r);
            p.frame_b = r_xf(r);
            p.delta_center = r_vec2(r);
            p.spring_softness = r_softness(r);
            p.enable_spring = r.r_bool();
            p.enable_limit = r.r_bool();
            p.enable_motor = r.r_bool();
            JointPayload::Prismatic(p)
        }
        4 => {
            let mut rv = RevoluteJoint::default();
            rv.linear_impulse = r_vec2(r);
            rv.spring_impulse = r.r_f32();
            rv.motor_impulse = r.r_f32();
            rv.lower_impulse = r.r_f32();
            rv.upper_impulse = r.r_f32();
            rv.hertz = r.r_f32();
            rv.damping_ratio = r.r_f32();
            rv.target_angle = r.r_f32();
            rv.max_motor_torque = r.r_f32();
            rv.motor_speed = r.r_f32();
            rv.lower_angle = r.r_f32();
            rv.upper_angle = r.r_f32();
            rv.index_a = r.r_i32();
            rv.index_b = r.r_i32();
            rv.frame_a = r_xf(r);
            rv.frame_b = r_xf(r);
            rv.delta_center = r_vec2(r);
            rv.axial_mass = r.r_f32();
            rv.spring_softness = r_softness(r);
            rv.enable_spring = r.r_bool();
            rv.enable_motor = r.r_bool();
            rv.enable_limit = r.r_bool();
            JointPayload::Revolute(rv)
        }
        5 => {
            let mut w = WeldJoint::default();
            w.linear_hertz = r.r_f32();
            w.linear_damping_ratio = r.r_f32();
            w.angular_hertz = r.r_f32();
            w.angular_damping_ratio = r.r_f32();
            w.linear_spring = r_softness(r);
            w.angular_spring = r_softness(r);
            w.linear_impulse = r_vec2(r);
            w.angular_impulse = r.r_f32();
            w.index_a = r.r_i32();
            w.index_b = r.r_i32();
            w.frame_a = r_xf(r);
            w.frame_b = r_xf(r);
            w.delta_center = r_vec2(r);
            w.axial_mass = r.r_f32();
            JointPayload::Weld(w)
        }
        _ => {
            let mut w = WheelJoint::default();
            w.perp_impulse = r.r_f32();
            w.motor_impulse = r.r_f32();
            w.spring_impulse = r.r_f32();
            w.lower_impulse = r.r_f32();
            w.upper_impulse = r.r_f32();
            w.max_motor_torque = r.r_f32();
            w.motor_speed = r.r_f32();
            w.lower_translation = r.r_f32();
            w.upper_translation = r.r_f32();
            w.hertz = r.r_f32();
            w.damping_ratio = r.r_f32();
            w.index_a = r.r_i32();
            w.index_b = r.r_i32();
            w.frame_a = r_xf(r);
            w.frame_b = r_xf(r);
            w.delta_center = r_vec2(r);
            w.perp_mass = r.r_f32();
            w.motor_mass = r.r_f32();
            w.axial_mass = r.r_f32();
            w.spring_softness = r_softness(r);
            w.enable_spring = r.r_bool();
            w.enable_motor = r.r_bool();
            w.enable_limit = r.r_bool();
            JointPayload::Wheel(w)
        }
    }
}

pub(crate) fn ser_joint_sim(buf: &mut Vec<u8>, j: &JointSim) {
    rec_w_i32(buf, j.joint_id);
    rec_w_i32(buf, j.body_id_a);
    rec_w_i32(buf, j.body_id_b);
    rec_w_xf(buf, j.local_frame_a);
    rec_w_xf(buf, j.local_frame_b);
    rec_w_f32(buf, j.inv_mass_a);
    rec_w_f32(buf, j.inv_mass_b);
    rec_w_f32(buf, j.inv_i_a);
    rec_w_f32(buf, j.inv_i_b);
    rec_w_f32(buf, j.constraint_hertz);
    rec_w_f32(buf, j.constraint_damping_ratio);
    w_softness(buf, j.constraint_softness);
    rec_w_f32(buf, j.force_threshold);
    rec_w_f32(buf, j.torque_threshold);
    ser_payload(buf, &j.payload);
}

pub(crate) fn des_joint_sim(r: &mut SnapReader) -> JointSim {
    let mut j = JointSim::default();
    j.joint_id = r.r_i32();
    j.body_id_a = r.r_i32();
    j.body_id_b = r.r_i32();
    j.local_frame_a = r_xf(r);
    j.local_frame_b = r_xf(r);
    j.inv_mass_a = r.r_f32();
    j.inv_mass_b = r.r_f32();
    j.inv_i_a = r.r_f32();
    j.inv_i_b = r.r_f32();
    j.constraint_hertz = r.r_f32();
    j.constraint_damping_ratio = r.r_f32();
    j.constraint_softness = r_softness(r);
    j.force_threshold = r.r_f32();
    j.torque_threshold = r.r_f32();
    j.payload = des_payload(r);
    j
}
