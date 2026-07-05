// Joint op family (0x90-0xD1): writers for the B2_REC hooks in the joint
// creates, the generic b2Joint_* setters, and every per-type setter, plus
// their replay dispatchers. Create ops append the returned id and replay
// asserts it, same as the other families.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::snapshot::SnapReader;
use super::snapshot_structs::{r_vec2, r_xf};
use super::write::*;
use super::Recording;
use crate::id::{BodyId, JointId};
use crate::math_functions::{Transform, Vec2};
use crate::types::JointDef;
use crate::world::World;

pub const OP_CREATE_DISTANCE_JOINT: u8 = 0x90;
pub const OP_CREATE_MOTOR_JOINT: u8 = 0x91;
pub const OP_CREATE_FILTER_JOINT: u8 = 0x92;
pub const OP_CREATE_PRISMATIC_JOINT: u8 = 0x93;
pub const OP_CREATE_REVOLUTE_JOINT: u8 = 0x94;
pub const OP_CREATE_WELD_JOINT: u8 = 0x95;
pub const OP_CREATE_WHEEL_JOINT: u8 = 0x96;
pub const OP_DESTROY_JOINT: u8 = 0x97;
pub const OP_JOINT_SET_LOCAL_FRAME_A: u8 = 0x98;
pub const OP_JOINT_SET_LOCAL_FRAME_B: u8 = 0x99;
pub const OP_JOINT_SET_COLLIDE_CONNECTED: u8 = 0x9A;
pub const OP_JOINT_WAKE_BODIES: u8 = 0x9B;
pub const OP_JOINT_SET_CONSTRAINT_TUNING: u8 = 0x9C;
pub const OP_JOINT_SET_FORCE_THRESHOLD: u8 = 0x9D;
pub const OP_JOINT_SET_TORQUE_THRESHOLD: u8 = 0x9E;

pub const OP_DISTANCE_SET_LENGTH: u8 = 0xA0;
pub const OP_DISTANCE_ENABLE_SPRING: u8 = 0xA1;
pub const OP_DISTANCE_SET_SPRING_FORCE_RANGE: u8 = 0xA2;
pub const OP_DISTANCE_SET_SPRING_HERTZ: u8 = 0xA3;
pub const OP_DISTANCE_SET_SPRING_DAMPING_RATIO: u8 = 0xA4;
pub const OP_DISTANCE_ENABLE_LIMIT: u8 = 0xA5;
pub const OP_DISTANCE_SET_LENGTH_RANGE: u8 = 0xA6;
pub const OP_DISTANCE_ENABLE_MOTOR: u8 = 0xA7;
pub const OP_DISTANCE_SET_MOTOR_SPEED: u8 = 0xA8;
pub const OP_DISTANCE_SET_MAX_MOTOR_FORCE: u8 = 0xA9;

pub const OP_MOTOR_SET_LINEAR_VELOCITY: u8 = 0xAA;
pub const OP_MOTOR_SET_ANGULAR_VELOCITY: u8 = 0xAB;
pub const OP_MOTOR_SET_MAX_VELOCITY_FORCE: u8 = 0xAC;
pub const OP_MOTOR_SET_MAX_VELOCITY_TORQUE: u8 = 0xAD;
pub const OP_MOTOR_SET_LINEAR_HERTZ: u8 = 0xAE;
pub const OP_MOTOR_SET_LINEAR_DAMPING_RATIO: u8 = 0xAF;
pub const OP_MOTOR_SET_ANGULAR_HERTZ: u8 = 0xB0;
pub const OP_MOTOR_SET_ANGULAR_DAMPING_RATIO: u8 = 0xB1;
pub const OP_MOTOR_SET_MAX_SPRING_FORCE: u8 = 0xB2;
pub const OP_MOTOR_SET_MAX_SPRING_TORQUE: u8 = 0xB3;

pub const OP_PRISMATIC_ENABLE_SPRING: u8 = 0xB4;
pub const OP_PRISMATIC_SET_SPRING_HERTZ: u8 = 0xB5;
pub const OP_PRISMATIC_SET_SPRING_DAMPING_RATIO: u8 = 0xB6;
pub const OP_PRISMATIC_SET_TARGET_TRANSLATION: u8 = 0xB7;
pub const OP_PRISMATIC_ENABLE_LIMIT: u8 = 0xB8;
pub const OP_PRISMATIC_SET_LIMITS: u8 = 0xB9;
pub const OP_PRISMATIC_ENABLE_MOTOR: u8 = 0xBA;
pub const OP_PRISMATIC_SET_MOTOR_SPEED: u8 = 0xBB;
pub const OP_PRISMATIC_SET_MAX_MOTOR_FORCE: u8 = 0xBC;

pub const OP_REVOLUTE_ENABLE_SPRING: u8 = 0xBD;
pub const OP_REVOLUTE_SET_SPRING_HERTZ: u8 = 0xBE;
pub const OP_REVOLUTE_SET_SPRING_DAMPING_RATIO: u8 = 0xBF;
pub const OP_REVOLUTE_SET_TARGET_ANGLE: u8 = 0xC0;
pub const OP_REVOLUTE_ENABLE_LIMIT: u8 = 0xC1;
pub const OP_REVOLUTE_SET_LIMITS: u8 = 0xC2;
pub const OP_REVOLUTE_ENABLE_MOTOR: u8 = 0xC3;
pub const OP_REVOLUTE_SET_MOTOR_SPEED: u8 = 0xC4;
pub const OP_REVOLUTE_SET_MAX_MOTOR_TORQUE: u8 = 0xC5;

pub const OP_WELD_SET_LINEAR_HERTZ: u8 = 0xC6;
pub const OP_WELD_SET_LINEAR_DAMPING_RATIO: u8 = 0xC7;
pub const OP_WELD_SET_ANGULAR_HERTZ: u8 = 0xC8;
pub const OP_WELD_SET_ANGULAR_DAMPING_RATIO: u8 = 0xC9;

pub const OP_WHEEL_ENABLE_SPRING: u8 = 0xCA;
pub const OP_WHEEL_SET_SPRING_HERTZ: u8 = 0xCB;
pub const OP_WHEEL_SET_SPRING_DAMPING_RATIO: u8 = 0xCC;
pub const OP_WHEEL_ENABLE_LIMIT: u8 = 0xCD;
pub const OP_WHEEL_SET_LIMITS: u8 = 0xCE;
pub const OP_WHEEL_ENABLE_MOTOR: u8 = 0xCF;
pub const OP_WHEEL_SET_MOTOR_SPEED: u8 = 0xD0;
pub const OP_WHEEL_SET_MAX_MOTOR_TORQUE: u8 = 0xD1;

// Writers

/// Joint create ops: informational world id + def payload + returned id.
/// (b2RecWriteRet_Create*Joint)
pub(crate) fn write_create_joint(
    rec: &mut Recording,
    opcode: u8,
    def_writer: impl FnOnce(&mut Vec<u8>),
    id: JointId,
) {
    rec.begin_record(opcode);
    rec_w_u32(&mut rec.buffer, 1);
    def_writer(&mut rec.buffer);
    rec_w_jointid(&mut rec.buffer, id);
    rec.end_record();
}

pub(crate) fn write_destroy_joint(rec: &mut Recording, joint: JointId, wake_attached: bool) {
    rec.begin_record(OP_DESTROY_JOINT);
    rec_w_jointid(&mut rec.buffer, joint);
    rec_w_bool(&mut rec.buffer, wake_attached);
    rec.end_record();
}

pub(crate) fn write_joint_marker(rec: &mut Recording, opcode: u8, joint: JointId) {
    rec.begin_record(opcode);
    rec_w_jointid(&mut rec.buffer, joint);
    rec.end_record();
}

pub(crate) fn write_joint_bool(rec: &mut Recording, opcode: u8, joint: JointId, flag: bool) {
    rec.begin_record(opcode);
    rec_w_jointid(&mut rec.buffer, joint);
    rec_w_bool(&mut rec.buffer, flag);
    rec.end_record();
}

pub(crate) fn write_joint_f32(rec: &mut Recording, opcode: u8, joint: JointId, value: f32) {
    rec.begin_record(opcode);
    rec_w_jointid(&mut rec.buffer, joint);
    rec_w_f32(&mut rec.buffer, value);
    rec.end_record();
}

pub(crate) fn write_joint_f32_pair(
    rec: &mut Recording,
    opcode: u8,
    joint: JointId,
    a: f32,
    b: f32,
) {
    rec.begin_record(opcode);
    rec_w_jointid(&mut rec.buffer, joint);
    rec_w_f32(&mut rec.buffer, a);
    rec_w_f32(&mut rec.buffer, b);
    rec.end_record();
}

pub(crate) fn write_joint_vec2(rec: &mut Recording, opcode: u8, joint: JointId, v: Vec2) {
    rec.begin_record(opcode);
    rec_w_jointid(&mut rec.buffer, joint);
    rec_w_vec2(&mut rec.buffer, v);
    rec.end_record();
}

pub(crate) fn write_joint_xf(rec: &mut Recording, opcode: u8, joint: JointId, frame: Transform) {
    rec.begin_record(opcode);
    rec_w_jointid(&mut rec.buffer, joint);
    rec_w_xf(&mut rec.buffer, frame);
    rec.end_record();
}

// Def readers (inverse of the rec_w_*jointdef writers)

fn r_joint_base(r: &mut SnapReader) -> JointDef {
    let mut base = crate::types::default_distance_joint_def().base;
    base.user_data = r.r_u64();
    base.body_id_a = BodyId::load(r.r_u64());
    base.body_id_b = BodyId::load(r.r_u64());
    base.local_frame_a = r_xf(r);
    base.local_frame_b = r_xf(r);
    base.force_threshold = r.r_f32();
    base.torque_threshold = r.r_f32();
    base.constraint_hertz = r.r_f32();
    base.constraint_damping_ratio = r.r_f32();
    base.draw_scale = r.r_f32();
    base.collide_connected = r.r_bool();
    base
}

fn ids_match(created: JointId, recorded: JointId) -> bool {
    created.index1 == recorded.index1 && created.generation == recorded.generation
}

/// Dispatch a joint-family opcode. Returns None when the opcode is not in
/// this family; Some(ids_match) otherwise.
pub(crate) fn dispatch_joint_op(opcode: u8, r: &mut SnapReader, world: &mut World) -> Option<bool> {
    use crate::joint::*;

    match opcode {
        OP_CREATE_DISTANCE_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_distance_joint_def();
            def.base = r_joint_base(r);
            def.length = r.r_f32();
            def.enable_spring = r.r_bool();
            def.lower_spring_force = r.r_f32();
            def.upper_spring_force = r.r_f32();
            def.hertz = r.r_f32();
            def.damping_ratio = r.r_f32();
            def.enable_limit = r.r_bool();
            def.min_length = r.r_f32();
            def.max_length = r.r_f32();
            def.enable_motor = r.r_bool();
            def.max_motor_force = r.r_f32();
            def.motor_speed = r.r_f32();
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_distance_joint(world, &def), recorded))
        }
        OP_CREATE_MOTOR_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_motor_joint_def();
            def.base = r_joint_base(r);
            def.linear_velocity = r_vec2(r);
            def.max_velocity_force = r.r_f32();
            def.angular_velocity = r.r_f32();
            def.max_velocity_torque = r.r_f32();
            def.linear_hertz = r.r_f32();
            def.linear_damping_ratio = r.r_f32();
            def.max_spring_force = r.r_f32();
            def.angular_hertz = r.r_f32();
            def.angular_damping_ratio = r.r_f32();
            def.max_spring_torque = r.r_f32();
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_motor_joint(world, &def), recorded))
        }
        OP_CREATE_FILTER_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_filter_joint_def();
            def.base = r_joint_base(r);
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_filter_joint(world, &def), recorded))
        }
        OP_CREATE_PRISMATIC_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_prismatic_joint_def();
            def.base = r_joint_base(r);
            def.enable_spring = r.r_bool();
            def.hertz = r.r_f32();
            def.damping_ratio = r.r_f32();
            def.target_translation = r.r_f32();
            def.enable_limit = r.r_bool();
            def.lower_translation = r.r_f32();
            def.upper_translation = r.r_f32();
            def.enable_motor = r.r_bool();
            def.max_motor_force = r.r_f32();
            def.motor_speed = r.r_f32();
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_prismatic_joint(world, &def), recorded))
        }
        OP_CREATE_REVOLUTE_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_revolute_joint_def();
            def.base = r_joint_base(r);
            def.target_angle = r.r_f32();
            def.enable_spring = r.r_bool();
            def.hertz = r.r_f32();
            def.damping_ratio = r.r_f32();
            def.enable_limit = r.r_bool();
            def.lower_angle = r.r_f32();
            def.upper_angle = r.r_f32();
            def.enable_motor = r.r_bool();
            def.max_motor_torque = r.r_f32();
            def.motor_speed = r.r_f32();
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_revolute_joint(world, &def), recorded))
        }
        OP_CREATE_WELD_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_weld_joint_def();
            def.base = r_joint_base(r);
            def.linear_hertz = r.r_f32();
            def.angular_hertz = r.r_f32();
            def.linear_damping_ratio = r.r_f32();
            def.angular_damping_ratio = r.r_f32();
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_weld_joint(world, &def), recorded))
        }
        OP_CREATE_WHEEL_JOINT => {
            let _world = r.r_u32();
            let mut def = crate::types::default_wheel_joint_def();
            def.base = r_joint_base(r);
            def.enable_spring = r.r_bool();
            def.hertz = r.r_f32();
            def.damping_ratio = r.r_f32();
            def.enable_limit = r.r_bool();
            def.lower_translation = r.r_f32();
            def.upper_translation = r.r_f32();
            def.enable_motor = r.r_bool();
            def.max_motor_torque = r.r_f32();
            def.motor_speed = r.r_f32();
            let recorded = JointId::load(r.r_u64());
            Some(ids_match(create_wheel_joint(world, &def), recorded))
        }
        OP_DESTROY_JOINT => {
            let joint = JointId::load(r.r_u64());
            let wake_attached = r.r_bool();
            destroy_joint(world, joint, wake_attached);
            Some(true)
        }
        OP_JOINT_SET_LOCAL_FRAME_A => {
            let joint = JointId::load(r.r_u64());
            let frame = r_xf(r);
            joint_set_local_frame_a(world, joint, frame);
            Some(true)
        }
        OP_JOINT_SET_LOCAL_FRAME_B => {
            let joint = JointId::load(r.r_u64());
            let frame = r_xf(r);
            joint_set_local_frame_b(world, joint, frame);
            Some(true)
        }
        OP_JOINT_SET_COLLIDE_CONNECTED => {
            let joint = JointId::load(r.r_u64());
            let should_collide = r.r_bool();
            joint_set_collide_connected(world, joint, should_collide);
            Some(true)
        }
        OP_JOINT_WAKE_BODIES => {
            let joint = JointId::load(r.r_u64());
            joint_wake_bodies(world, joint);
            Some(true)
        }
        OP_JOINT_SET_CONSTRAINT_TUNING => {
            let joint = JointId::load(r.r_u64());
            let hertz = r.r_f32();
            let damping_ratio = r.r_f32();
            joint_set_constraint_tuning(world, joint, hertz, damping_ratio);
            Some(true)
        }
        OP_JOINT_SET_FORCE_THRESHOLD => {
            let joint = JointId::load(r.r_u64());
            let threshold = r.r_f32();
            joint_set_force_threshold(world, joint, threshold);
            Some(true)
        }
        OP_JOINT_SET_TORQUE_THRESHOLD => {
            let joint = JointId::load(r.r_u64());
            let threshold = r.r_f32();
            joint_set_torque_threshold(world, joint, threshold);
            Some(true)
        }
        _ => dispatch_per_type_joint_op(opcode, r, world),
    }
}

/// Per-type setters, all `(joint, value)` shaped. Split out to keep the
/// match arms readable.
fn dispatch_per_type_joint_op(opcode: u8, r: &mut SnapReader, world: &mut World) -> Option<bool> {
    use crate::distance_joint::*;
    use crate::motor_joint::*;
    use crate::prismatic_joint::*;
    use crate::revolute_joint::*;
    use crate::weld_joint::*;
    use crate::wheel_joint::*;

    let joint = match opcode {
        0xA0..=0xD1 => JointId::load(r.r_u64()),
        _ => return None,
    };

    match opcode {
        OP_DISTANCE_SET_LENGTH => distance_joint_set_length(world, joint, r.r_f32()),
        OP_DISTANCE_ENABLE_SPRING => distance_joint_enable_spring(world, joint, r.r_bool()),
        OP_DISTANCE_SET_SPRING_FORCE_RANGE => {
            let lower = r.r_f32();
            let upper = r.r_f32();
            distance_joint_set_spring_force_range(world, joint, lower, upper);
        }
        OP_DISTANCE_SET_SPRING_HERTZ => distance_joint_set_spring_hertz(world, joint, r.r_f32()),
        OP_DISTANCE_SET_SPRING_DAMPING_RATIO => {
            distance_joint_set_spring_damping_ratio(world, joint, r.r_f32())
        }
        OP_DISTANCE_ENABLE_LIMIT => distance_joint_enable_limit(world, joint, r.r_bool()),
        OP_DISTANCE_SET_LENGTH_RANGE => {
            let min_length = r.r_f32();
            let max_length = r.r_f32();
            distance_joint_set_length_range(world, joint, min_length, max_length);
        }
        OP_DISTANCE_ENABLE_MOTOR => distance_joint_enable_motor(world, joint, r.r_bool()),
        OP_DISTANCE_SET_MOTOR_SPEED => distance_joint_set_motor_speed(world, joint, r.r_f32()),
        OP_DISTANCE_SET_MAX_MOTOR_FORCE => {
            distance_joint_set_max_motor_force(world, joint, r.r_f32())
        }

        OP_MOTOR_SET_LINEAR_VELOCITY => motor_joint_set_linear_velocity(world, joint, r_vec2(r)),
        OP_MOTOR_SET_ANGULAR_VELOCITY => motor_joint_set_angular_velocity(world, joint, r.r_f32()),
        OP_MOTOR_SET_MAX_VELOCITY_FORCE => {
            motor_joint_set_max_velocity_force(world, joint, r.r_f32())
        }
        OP_MOTOR_SET_MAX_VELOCITY_TORQUE => {
            motor_joint_set_max_velocity_torque(world, joint, r.r_f32())
        }
        OP_MOTOR_SET_LINEAR_HERTZ => motor_joint_set_linear_hertz(world, joint, r.r_f32()),
        OP_MOTOR_SET_LINEAR_DAMPING_RATIO => {
            motor_joint_set_linear_damping_ratio(world, joint, r.r_f32())
        }
        OP_MOTOR_SET_ANGULAR_HERTZ => motor_joint_set_angular_hertz(world, joint, r.r_f32()),
        OP_MOTOR_SET_ANGULAR_DAMPING_RATIO => {
            motor_joint_set_angular_damping_ratio(world, joint, r.r_f32())
        }
        OP_MOTOR_SET_MAX_SPRING_FORCE => motor_joint_set_max_spring_force(world, joint, r.r_f32()),
        OP_MOTOR_SET_MAX_SPRING_TORQUE => {
            motor_joint_set_max_spring_torque(world, joint, r.r_f32())
        }

        OP_PRISMATIC_ENABLE_SPRING => prismatic_joint_enable_spring(world, joint, r.r_bool()),
        OP_PRISMATIC_SET_SPRING_HERTZ => prismatic_joint_set_spring_hertz(world, joint, r.r_f32()),
        OP_PRISMATIC_SET_SPRING_DAMPING_RATIO => {
            prismatic_joint_set_spring_damping_ratio(world, joint, r.r_f32())
        }
        OP_PRISMATIC_SET_TARGET_TRANSLATION => {
            prismatic_joint_set_target_translation(world, joint, r.r_f32())
        }
        OP_PRISMATIC_ENABLE_LIMIT => prismatic_joint_enable_limit(world, joint, r.r_bool()),
        OP_PRISMATIC_SET_LIMITS => {
            let lower = r.r_f32();
            let upper = r.r_f32();
            prismatic_joint_set_limits(world, joint, lower, upper);
        }
        OP_PRISMATIC_ENABLE_MOTOR => prismatic_joint_enable_motor(world, joint, r.r_bool()),
        OP_PRISMATIC_SET_MOTOR_SPEED => prismatic_joint_set_motor_speed(world, joint, r.r_f32()),
        OP_PRISMATIC_SET_MAX_MOTOR_FORCE => {
            prismatic_joint_set_max_motor_force(world, joint, r.r_f32())
        }

        OP_REVOLUTE_ENABLE_SPRING => revolute_joint_enable_spring(world, joint, r.r_bool()),
        OP_REVOLUTE_SET_SPRING_HERTZ => revolute_joint_set_spring_hertz(world, joint, r.r_f32()),
        OP_REVOLUTE_SET_SPRING_DAMPING_RATIO => {
            revolute_joint_set_spring_damping_ratio(world, joint, r.r_f32())
        }
        OP_REVOLUTE_SET_TARGET_ANGLE => revolute_joint_set_target_angle(world, joint, r.r_f32()),
        OP_REVOLUTE_ENABLE_LIMIT => revolute_joint_enable_limit(world, joint, r.r_bool()),
        OP_REVOLUTE_SET_LIMITS => {
            let lower = r.r_f32();
            let upper = r.r_f32();
            revolute_joint_set_limits(world, joint, lower, upper);
        }
        OP_REVOLUTE_ENABLE_MOTOR => revolute_joint_enable_motor(world, joint, r.r_bool()),
        OP_REVOLUTE_SET_MOTOR_SPEED => revolute_joint_set_motor_speed(world, joint, r.r_f32()),
        OP_REVOLUTE_SET_MAX_MOTOR_TORQUE => {
            revolute_joint_set_max_motor_torque(world, joint, r.r_f32())
        }

        OP_WELD_SET_LINEAR_HERTZ => weld_joint_set_linear_hertz(world, joint, r.r_f32()),
        OP_WELD_SET_LINEAR_DAMPING_RATIO => {
            weld_joint_set_linear_damping_ratio(world, joint, r.r_f32())
        }
        OP_WELD_SET_ANGULAR_HERTZ => weld_joint_set_angular_hertz(world, joint, r.r_f32()),
        OP_WELD_SET_ANGULAR_DAMPING_RATIO => {
            weld_joint_set_angular_damping_ratio(world, joint, r.r_f32())
        }

        OP_WHEEL_ENABLE_SPRING => wheel_joint_enable_spring(world, joint, r.r_bool()),
        OP_WHEEL_SET_SPRING_HERTZ => wheel_joint_set_spring_hertz(world, joint, r.r_f32()),
        OP_WHEEL_SET_SPRING_DAMPING_RATIO => {
            wheel_joint_set_spring_damping_ratio(world, joint, r.r_f32())
        }
        OP_WHEEL_ENABLE_LIMIT => wheel_joint_enable_limit(world, joint, r.r_bool()),
        OP_WHEEL_SET_LIMITS => {
            let lower = r.r_f32();
            let upper = r.r_f32();
            wheel_joint_set_limits(world, joint, lower, upper);
        }
        OP_WHEEL_ENABLE_MOTOR => wheel_joint_enable_motor(world, joint, r.r_bool()),
        OP_WHEEL_SET_MOTOR_SPEED => wheel_joint_set_motor_speed(world, joint, r.r_f32()),
        OP_WHEEL_SET_MAX_MOTOR_TORQUE => wheel_joint_set_max_motor_torque(world, joint, r.r_f32()),

        _ => return None,
    }

    Some(true)
}

#[cfg(test)]
mod tests {
    use crate::body::create_body;
    use crate::distance_joint::*;
    use crate::geometry::{make_box, make_square};
    use crate::joint::*;
    use crate::math_functions::{to_pos, Vec2, PI};
    use crate::recording::{replay_buffer, world_start_recording, world_stop_recording, Recording};
    use crate::revolute_joint::*;
    use crate::shape::create_polygon_shape;
    use crate::types::{
        default_body_def, default_distance_joint_def, default_revolute_joint_def,
        default_shape_def, default_weld_joint_def, default_wheel_joint_def, default_world_def,
        BodyType,
    };
    use crate::world::{world_step, World};

    // The joint family round trips: joints created mid-stream (revolute,
    // distance, weld, wheel), per-type setters flipped live, frames moved,
    // and a destroy must all re-execute on replay.
    #[test]
    fn joint_ops_replay() {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let bd = default_body_def();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground, &sd, &make_box(20.0, 1.0));

        let mut bodies = Vec::new();
        for i in 0..5 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: -4.0 + 2.0 * i as f32,
                y: 3.0,
            });
            let body = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body, &sd, &make_square(0.3));
            bodies.push(body);
        }

        assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

        let mut revolute = None;
        let mut distance = None;
        for step in 0..80 {
            match step {
                8 => {
                    // Hinge with the falling-hinges tuning.
                    let mut rd = default_revolute_joint_def();
                    rd.enable_limit = true;
                    rd.lower_angle = -0.1 * PI;
                    rd.upper_angle = 0.2 * PI;
                    rd.enable_spring = true;
                    rd.hertz = 1.0;
                    rd.damping_ratio = 1.0;
                    rd.base.body_id_a = bodies[0];
                    rd.base.body_id_b = bodies[1];
                    rd.base.local_frame_a.p = Vec2 { x: 0.3, y: 0.0 };
                    rd.base.local_frame_b.p = Vec2 { x: -0.3, y: 0.0 };
                    revolute = Some(create_revolute_joint(&mut world, &rd));

                    let mut dd = default_distance_joint_def();
                    dd.length = 1.6;
                    dd.base.body_id_a = bodies[1];
                    dd.base.body_id_b = bodies[2];
                    distance = Some(create_distance_joint(&mut world, &dd));

                    let mut wd = default_weld_joint_def();
                    wd.linear_hertz = 4.0;
                    wd.base.body_id_a = bodies[2];
                    wd.base.body_id_b = bodies[3];
                    create_weld_joint(&mut world, &wd);

                    let mut hd = default_wheel_joint_def();
                    hd.enable_spring = true;
                    hd.hertz = 2.0;
                    hd.damping_ratio = 0.6;
                    hd.base.body_id_a = bodies[3];
                    hd.base.body_id_b = bodies[4];
                    create_wheel_joint(&mut world, &hd);
                }
                25 => {
                    let joint = revolute.unwrap();
                    revolute_joint_enable_motor(&mut world, joint, true);
                    revolute_joint_set_motor_speed(&mut world, joint, 2.0);
                    revolute_joint_set_max_motor_torque(&mut world, joint, 5.0);
                    revolute_joint_set_limits(&mut world, joint, -0.25 * PI, 0.25 * PI);
                    distance_joint_enable_spring(&mut world, distance.unwrap(), true);
                    distance_joint_set_spring_hertz(&mut world, distance.unwrap(), 3.0);
                }
                40 => {
                    let mut frame = joint_get_local_frame_a(&world, revolute.unwrap());
                    frame.p = Vec2 { x: 0.4, y: 0.1 };
                    joint_set_local_frame_a(&mut world, revolute.unwrap(), frame);
                    joint_set_collide_connected(&mut world, distance.unwrap(), true);
                    distance_joint_set_length(&mut world, distance.unwrap(), 2.2);
                }
                55 => {
                    destroy_joint(&mut world, distance.unwrap(), true);
                    joint_wake_bodies(&mut world, revolute.unwrap());
                }
                _ => {}
            }
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        let recording = world_stop_recording(&mut world).expect("active session");
        let result = replay_buffer(&recording.buffer);
        assert!(result.ok, "stream parses");
        assert!(!result.diverged, "joint ops must re-execute identically");
        assert_eq!(result.steps, 80);
    }
}
