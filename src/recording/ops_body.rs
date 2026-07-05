// Body op family (0x10-0x3C): writers for the B2_REC hooks in the b2Body_*
// API and their replay dispatchers. Created ids reproduce exactly on replay
// (the seed snapshot carries the id free lists), so the recorded id is
// asserted, never remapped — recording it verifies deterministic replay,
// same as C.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::ops::read_position;
use super::snapshot::SnapReader;
use super::snapshot_structs::{r_rot, r_string, r_vec2};
use super::write::*;
use super::Recording;
use crate::collision::MassData;
use crate::id::BodyId;
use crate::math_functions::{Pos, Rot, Vec2, WorldTransform};
use crate::types::{BodyDef, BodyType, MotionLocks};
use crate::world::World;

pub const OP_CREATE_BODY: u8 = 0x10;
pub const OP_DESTROY_BODY: u8 = 0x11;
pub const OP_BODY_SET_TRANSFORM: u8 = 0x20;
pub const OP_BODY_SET_LINEAR_VELOCITY: u8 = 0x21;
pub const OP_BODY_SET_TYPE: u8 = 0x22;
pub const OP_BODY_SET_NAME: u8 = 0x23;
pub const OP_BODY_SET_ANGULAR_VELOCITY: u8 = 0x24;
pub const OP_BODY_SET_TARGET_TRANSFORM: u8 = 0x25;
pub const OP_BODY_APPLY_FORCE: u8 = 0x26;
pub const OP_BODY_APPLY_FORCE_TO_CENTER: u8 = 0x27;
pub const OP_BODY_APPLY_TORQUE: u8 = 0x28;
pub const OP_BODY_CLEAR_FORCES: u8 = 0x29;
pub const OP_BODY_APPLY_LINEAR_IMPULSE: u8 = 0x2A;
pub const OP_BODY_APPLY_LINEAR_IMPULSE_TO_CENTER: u8 = 0x2B;
pub const OP_BODY_APPLY_ANGULAR_IMPULSE: u8 = 0x2C;
pub const OP_BODY_SET_MASS_DATA: u8 = 0x2D;
pub const OP_BODY_APPLY_MASS_FROM_SHAPES: u8 = 0x2E;
pub const OP_BODY_SET_LINEAR_DAMPING: u8 = 0x2F;
pub const OP_BODY_SET_ANGULAR_DAMPING: u8 = 0x30;
pub const OP_BODY_SET_GRAVITY_SCALE: u8 = 0x31;
pub const OP_BODY_SET_AWAKE: u8 = 0x32;
pub const OP_BODY_WAKE_TOUCHING: u8 = 0x33;
pub const OP_BODY_ENABLE_SLEEP: u8 = 0x34;
pub const OP_BODY_SET_SLEEP_THRESHOLD: u8 = 0x35;
pub const OP_BODY_DISABLE: u8 = 0x36;
pub const OP_BODY_ENABLE: u8 = 0x37;
pub const OP_BODY_SET_MOTION_LOCKS: u8 = 0x38;
pub const OP_BODY_SET_BULLET: u8 = 0x39;
pub const OP_BODY_ENABLE_CONTACT_RECYCLING: u8 = 0x3A;
pub const OP_BODY_ENABLE_CONTACT_EVENTS: u8 = 0x3B;
pub const OP_BODY_ENABLE_HIT_EVENTS: u8 = 0x3C;

// Writers

/// Create op with the returned id appended inside the same record so replay
/// can assert it matches. (B2_REC_CREATE / b2RecWriteRet_CreateBody)
pub(crate) fn write_create_body(rec: &mut Recording, def: &BodyDef, id: BodyId) {
    rec.begin_record(OP_CREATE_BODY);
    // The world arg is informational; replay targets its own world.
    rec_w_u32(&mut rec.buffer, 1);
    rec_w_bodydef(&mut rec.buffer, def);
    rec_w_bodyid(&mut rec.buffer, id);
    rec.end_record();
}

pub(crate) fn write_body_marker(rec: &mut Recording, opcode: u8, body: BodyId) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec.end_record();
}

pub(crate) fn write_body_bool(rec: &mut Recording, opcode: u8, body: BodyId, flag: bool) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_bool(&mut rec.buffer, flag);
    rec.end_record();
}

pub(crate) fn write_body_f32(rec: &mut Recording, opcode: u8, body: BodyId, value: f32) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_f32(&mut rec.buffer, value);
    rec.end_record();
}

pub(crate) fn write_body_vec2(rec: &mut Recording, opcode: u8, body: BodyId, v: Vec2) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_vec2(&mut rec.buffer, v);
    rec.end_record();
}

/// Force/impulse at a world point: VEC2 + POSITION + BOOL wake.
pub(crate) fn write_body_vec2_point_bool(
    rec: &mut Recording,
    opcode: u8,
    body: BodyId,
    v: Vec2,
    point: Pos,
    wake: bool,
) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_vec2(&mut rec.buffer, v);
    rec_w_position(&mut rec.buffer, point);
    rec_w_bool(&mut rec.buffer, wake);
    rec.end_record();
}

pub(crate) fn write_body_vec2_bool(
    rec: &mut Recording,
    opcode: u8,
    body: BodyId,
    v: Vec2,
    wake: bool,
) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_vec2(&mut rec.buffer, v);
    rec_w_bool(&mut rec.buffer, wake);
    rec.end_record();
}

pub(crate) fn write_body_f32_bool(
    rec: &mut Recording,
    opcode: u8,
    body: BodyId,
    value: f32,
    wake: bool,
) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_f32(&mut rec.buffer, value);
    rec_w_bool(&mut rec.buffer, wake);
    rec.end_record();
}

pub(crate) fn write_body_set_transform(
    rec: &mut Recording,
    body: BodyId,
    position: Pos,
    rotation: Rot,
) {
    rec.begin_record(OP_BODY_SET_TRANSFORM);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_position(&mut rec.buffer, position);
    rec_w_rot(&mut rec.buffer, rotation);
    rec.end_record();
}

pub(crate) fn write_body_set_type(rec: &mut Recording, body: BodyId, body_type: BodyType) {
    rec.begin_record(OP_BODY_SET_TYPE);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_i32(&mut rec.buffer, body_type as i32);
    rec.end_record();
}

pub(crate) fn write_body_set_name(rec: &mut Recording, body: BodyId, name: &str) {
    rec.begin_record(OP_BODY_SET_NAME);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_str(&mut rec.buffer, name);
    rec.end_record();
}

pub(crate) fn write_body_set_target_transform(
    rec: &mut Recording,
    body: BodyId,
    target: WorldTransform,
    time_step: f32,
    wake: bool,
) {
    rec.begin_record(OP_BODY_SET_TARGET_TRANSFORM);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_worldxf(&mut rec.buffer, target);
    rec_w_f32(&mut rec.buffer, time_step);
    rec_w_bool(&mut rec.buffer, wake);
    rec.end_record();
}

pub(crate) fn write_body_set_mass_data(rec: &mut Recording, body: BodyId, mass_data: MassData) {
    rec.begin_record(OP_BODY_SET_MASS_DATA);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_massdata(&mut rec.buffer, mass_data);
    rec.end_record();
}

pub(crate) fn write_body_set_motion_locks(rec: &mut Recording, body: BodyId, locks: MotionLocks) {
    rec.begin_record(OP_BODY_SET_MOTION_LOCKS);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_locks(&mut rec.buffer, locks);
    rec.end_record();
}

// Readers

fn r_body_id(r: &mut SnapReader) -> BodyId {
    BodyId::load(r.r_u64())
}

fn r_body_def(r: &mut SnapReader) -> BodyDef {
    // Readers start from the default def to get the cookie, then overwrite
    // fields, same as the C reader contract.
    let mut def = crate::types::default_body_def();
    def.type_ = match r.r_i32() {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        _ => BodyType::Dynamic,
    };
    def.position = read_position(r);
    def.rotation = r_rot(r);
    def.linear_velocity = r_vec2(r);
    def.angular_velocity = r.r_f32();
    def.linear_damping = r.r_f32();
    def.angular_damping = r.r_f32();
    def.gravity_scale = r.r_f32();
    def.sleep_threshold = r.r_f32();
    def.name = r_string(r);
    def.user_data = r.r_u64();
    def.motion_locks = MotionLocks {
        linear_x: r.r_bool(),
        linear_y: r.r_bool(),
        angular_z: r.r_bool(),
    };
    def.enable_sleep = r.r_bool();
    def.is_awake = r.r_bool();
    def.is_bullet = r.r_bool();
    def.is_enabled = r.r_bool();
    def.allow_fast_rotation = r.r_bool();
    def.enable_contact_recycling = r.r_bool();
    def
}

/// Dispatch a body-family opcode against the replay world. Returns None when
/// the opcode is not in this family; Some(ids_match) otherwise, where a false
/// means a create op returned a different id than recorded (a determinism
/// failure the caller reports as divergence).
pub(crate) fn dispatch_body_op(opcode: u8, r: &mut SnapReader, world: &mut World) -> Option<bool> {
    use crate::body::*;

    match opcode {
        OP_CREATE_BODY => {
            let _world = r.r_u32();
            let def = r_body_def(r);
            let recorded = r_body_id(r);
            let created = create_body(world, &def);
            Some(created.index1 == recorded.index1 && created.generation == recorded.generation)
        }
        OP_DESTROY_BODY => {
            let body = r_body_id(r);
            destroy_body(world, body);
            Some(true)
        }
        OP_BODY_SET_TRANSFORM => {
            let body = r_body_id(r);
            let position = read_position(r);
            let rotation = r_rot(r);
            body_set_transform(world, body, position, rotation);
            Some(true)
        }
        OP_BODY_SET_LINEAR_VELOCITY => {
            let body = r_body_id(r);
            let v = r_vec2(r);
            body_set_linear_velocity(world, body, v);
            Some(true)
        }
        OP_BODY_SET_TYPE => {
            let body = r_body_id(r);
            let body_type = match r.r_i32() {
                0 => BodyType::Static,
                1 => BodyType::Kinematic,
                _ => BodyType::Dynamic,
            };
            body_set_type(world, body, body_type);
            Some(true)
        }
        OP_BODY_SET_NAME => {
            let body = r_body_id(r);
            let name = r_string(r);
            body_set_name(world, body, &name);
            Some(true)
        }
        OP_BODY_SET_ANGULAR_VELOCITY => {
            let body = r_body_id(r);
            let w = r.r_f32();
            body_set_angular_velocity(world, body, w);
            Some(true)
        }
        OP_BODY_SET_TARGET_TRANSFORM => {
            let body = r_body_id(r);
            let target = WorldTransform {
                p: read_position(r),
                q: r_rot(r),
            };
            let time_step = r.r_f32();
            let wake = r.r_bool();
            body_set_target_transform(world, body, target, time_step, wake);
            Some(true)
        }
        OP_BODY_APPLY_FORCE => {
            let body = r_body_id(r);
            let force = r_vec2(r);
            let point = read_position(r);
            let wake = r.r_bool();
            body_apply_force(world, body, force, point, wake);
            Some(true)
        }
        OP_BODY_APPLY_FORCE_TO_CENTER => {
            let body = r_body_id(r);
            let force = r_vec2(r);
            let wake = r.r_bool();
            body_apply_force_to_center(world, body, force, wake);
            Some(true)
        }
        OP_BODY_APPLY_TORQUE => {
            let body = r_body_id(r);
            let torque = r.r_f32();
            let wake = r.r_bool();
            body_apply_torque(world, body, torque, wake);
            Some(true)
        }
        OP_BODY_CLEAR_FORCES => {
            let body = r_body_id(r);
            body_clear_forces(world, body);
            Some(true)
        }
        OP_BODY_APPLY_LINEAR_IMPULSE => {
            let body = r_body_id(r);
            let impulse = r_vec2(r);
            let point = read_position(r);
            let wake = r.r_bool();
            body_apply_linear_impulse(world, body, impulse, point, wake);
            Some(true)
        }
        OP_BODY_APPLY_LINEAR_IMPULSE_TO_CENTER => {
            let body = r_body_id(r);
            let impulse = r_vec2(r);
            let wake = r.r_bool();
            body_apply_linear_impulse_to_center(world, body, impulse, wake);
            Some(true)
        }
        OP_BODY_APPLY_ANGULAR_IMPULSE => {
            let body = r_body_id(r);
            let impulse = r.r_f32();
            let wake = r.r_bool();
            body_apply_angular_impulse(world, body, impulse, wake);
            Some(true)
        }
        OP_BODY_SET_MASS_DATA => {
            let body = r_body_id(r);
            let mass_data = MassData {
                mass: r.r_f32(),
                center: r_vec2(r),
                rotational_inertia: r.r_f32(),
            };
            body_set_mass_data(world, body, mass_data);
            Some(true)
        }
        OP_BODY_APPLY_MASS_FROM_SHAPES => {
            let body = r_body_id(r);
            body_apply_mass_from_shapes(world, body);
            Some(true)
        }
        OP_BODY_SET_LINEAR_DAMPING => {
            let body = r_body_id(r);
            let damping = r.r_f32();
            body_set_linear_damping(world, body, damping);
            Some(true)
        }
        OP_BODY_SET_ANGULAR_DAMPING => {
            let body = r_body_id(r);
            let damping = r.r_f32();
            body_set_angular_damping(world, body, damping);
            Some(true)
        }
        OP_BODY_SET_GRAVITY_SCALE => {
            let body = r_body_id(r);
            let scale = r.r_f32();
            body_set_gravity_scale(world, body, scale);
            Some(true)
        }
        OP_BODY_SET_AWAKE => {
            let body = r_body_id(r);
            let awake = r.r_bool();
            body_set_awake(world, body, awake);
            Some(true)
        }
        OP_BODY_WAKE_TOUCHING => {
            let body = r_body_id(r);
            body_wake_touching(world, body);
            Some(true)
        }
        OP_BODY_ENABLE_SLEEP => {
            let body = r_body_id(r);
            let flag = r.r_bool();
            body_enable_sleep(world, body, flag);
            Some(true)
        }
        OP_BODY_SET_SLEEP_THRESHOLD => {
            let body = r_body_id(r);
            let threshold = r.r_f32();
            body_set_sleep_threshold(world, body, threshold);
            Some(true)
        }
        OP_BODY_DISABLE => {
            let body = r_body_id(r);
            body_disable(world, body);
            Some(true)
        }
        OP_BODY_ENABLE => {
            let body = r_body_id(r);
            body_enable(world, body);
            Some(true)
        }
        OP_BODY_SET_MOTION_LOCKS => {
            let body = r_body_id(r);
            let locks = MotionLocks {
                linear_x: r.r_bool(),
                linear_y: r.r_bool(),
                angular_z: r.r_bool(),
            };
            body_set_motion_locks(world, body, locks);
            Some(true)
        }
        OP_BODY_SET_BULLET => {
            let body = r_body_id(r);
            let flag = r.r_bool();
            body_set_bullet(world, body, flag);
            Some(true)
        }
        OP_BODY_ENABLE_CONTACT_RECYCLING => {
            let body = r_body_id(r);
            let flag = r.r_bool();
            body_enable_contact_recycling(world, body, flag);
            Some(true)
        }
        OP_BODY_ENABLE_CONTACT_EVENTS => {
            let body = r_body_id(r);
            let flag = r.r_bool();
            body_enable_contact_events(world, body, flag);
            Some(true)
        }
        OP_BODY_ENABLE_HIT_EVENTS => {
            let body = r_body_id(r);
            let flag = r.r_bool();
            body_enable_hit_events(world, body, flag);
            Some(true)
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use crate::body::*;
    use crate::geometry::{make_box, make_square};
    use crate::math_functions::{make_rot, to_pos, Vec2};
    use crate::recording::{replay_buffer, world_start_recording, world_stop_recording, Recording};
    use crate::shape::create_polygon_shape;
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::{world_step, World};

    // The full body op family round trips: creates, impulses, teleports,
    // type changes, disable/enable, and destroys recorded mid-stream must
    // re-execute on replay or the per-step hashes diverge.
    #[test]
    fn body_ops_replay() {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let bd = default_body_def();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground, &sd, &make_box(20.0, 1.0));

        let mut first = Vec::new();
        for i in 0..4 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: -3.0 + 2.0 * i as f32,
                y: 2.0,
            });
            let body = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body, &sd, &make_square(0.4));
            first.push(body);
        }

        assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

        let mut created = Vec::new();
        for step in 0..80 {
            match step {
                10 => {
                    // Create mid-recording: one plain, one named bullet.
                    let mut bd = default_body_def();
                    bd.type_ = BodyType::Dynamic;
                    bd.position = to_pos(Vec2 { x: 0.0, y: 6.0 });
                    bd.linear_velocity = Vec2 { x: 0.5, y: 0.0 };
                    let a = create_body(&mut world, &bd);
                    create_polygon_shape(&mut world, a, &sd, &make_square(0.3));
                    bd.position = to_pos(Vec2 { x: 1.0, y: 7.0 });
                    bd.is_bullet = true;
                    bd.name = "replayed".to_string();
                    let b = create_body(&mut world, &bd);
                    create_polygon_shape(&mut world, b, &sd, &make_square(0.2));
                    created.push(a);
                    created.push(b);
                }
                20 => {
                    body_apply_linear_impulse_to_center(
                        &mut world,
                        first[0],
                        Vec2 { x: 2.0, y: 4.0 },
                        true,
                    );
                    body_apply_torque(&mut world, first[1], 15.0, true);
                    body_set_transform(
                        &mut world,
                        first[2],
                        to_pos(Vec2 { x: 5.0, y: 4.0 }),
                        make_rot(0.7),
                    );
                }
                35 => {
                    body_set_type(&mut world, first[3], BodyType::Static);
                    body_disable(&mut world, created[0]);
                    body_set_gravity_scale(&mut world, created[1], 0.25);
                }
                50 => {
                    body_set_type(&mut world, first[3], BodyType::Dynamic);
                    body_enable(&mut world, created[0]);
                    destroy_body(&mut world, first[0]);
                }
                _ => {}
            }
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        let recording = world_stop_recording(&mut world).expect("active session");
        let result = replay_buffer(&recording.buffer);
        assert!(result.ok, "stream parses");
        assert!(!result.diverged, "body ops must re-execute identically");
        assert_eq!(result.steps, 80);
    }
}
