// Wire write primitives from recording.c. Everything is little-endian and
// byte-identical to the C recording format: geometry PODs are written field
// by field in declaration order (the C memcpy layouts have no padding), and
// world positions widen to two doubles only in the double-precision build.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{
    Capsule, ChainSegment, Circle, MassData, Polygon, Segment, WorldCastOutput,
};
use crate::distance::ShapeProxy;
use crate::dynamic_tree::TreeStats;
use crate::hull::MAX_POLYGON_VERTICES;
use crate::id::{BodyId, ChainId, JointId, ShapeId, WorldId};
use crate::math_functions::{Aabb, Pos, Rot, Transform, Vec2, WorldTransform};
use crate::types::{
    BodyDef, ChainDef, DistanceJointDef, ExplosionDef, Filter, FilterJointDef, JointDef,
    MotionLocks, MotorJointDef, PrismaticJointDef, QueryFilter, RayResult, RevoluteJointDef,
    ShapeDef, SurfaceMaterial, WeldJointDef, WheelJointDef,
};

pub fn rec_w_u8(buf: &mut Vec<u8>, v: u8) {
    buf.push(v);
}

pub fn rec_w_u16(buf: &mut Vec<u8>, v: u16) {
    buf.extend_from_slice(&v.to_le_bytes());
}

pub fn rec_w_u32(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

pub fn rec_w_u64(buf: &mut Vec<u8>, v: u64) {
    buf.extend_from_slice(&v.to_le_bytes());
}

pub fn rec_w_i32(buf: &mut Vec<u8>, v: i32) {
    rec_w_u32(buf, v as u32);
}

pub fn rec_w_f32(buf: &mut Vec<u8>, v: f32) {
    rec_w_u32(buf, v.to_bits());
}

pub fn rec_w_f64(buf: &mut Vec<u8>, v: f64) {
    rec_w_u64(buf, v.to_bits());
}

pub fn rec_w_bool(buf: &mut Vec<u8>, v: bool) {
    rec_w_u8(buf, if v { 1 } else { 0 });
}

pub fn rec_w_vec2(buf: &mut Vec<u8>, v: Vec2) {
    rec_w_f32(buf, v.x);
    rec_w_f32(buf, v.y);
}

pub fn rec_w_rot(buf: &mut Vec<u8>, v: Rot) {
    rec_w_f32(buf, v.c);
    rec_w_f32(buf, v.s);
}

pub fn rec_w_xf(buf: &mut Vec<u8>, v: Transform) {
    rec_w_vec2(buf, v.p);
    rec_w_rot(buf, v.q);
}

/// A world position keeps full precision on the wire so a recording
/// reproduces the simulation far from the origin. In the float build this is
/// two floats, identical to VEC2. (b2RecW_POSITION)
#[cfg(feature = "double-precision")]
pub fn rec_w_position(buf: &mut Vec<u8>, v: Pos) {
    rec_w_f64(buf, v.x);
    rec_w_f64(buf, v.y);
}

#[cfg(not(feature = "double-precision"))]
pub fn rec_w_position(buf: &mut Vec<u8>, v: Pos) {
    rec_w_f32(buf, v.x);
    rec_w_f32(buf, v.y);
}

pub fn rec_w_worldxf(buf: &mut Vec<u8>, v: WorldTransform) {
    rec_w_position(buf, v.p);
    rec_w_rot(buf, v.q);
}

pub fn rec_w_worldid(buf: &mut Vec<u8>, v: WorldId) {
    rec_w_u32(buf, v.store());
}

pub fn rec_w_bodyid(buf: &mut Vec<u8>, v: BodyId) {
    rec_w_u64(buf, v.store());
}

pub fn rec_w_shapeid(buf: &mut Vec<u8>, v: ShapeId) {
    rec_w_u64(buf, v.store());
}

pub fn rec_w_chainid(buf: &mut Vec<u8>, v: ChainId) {
    rec_w_u64(buf, v.store());
}

pub fn rec_w_jointid(buf: &mut Vec<u8>, v: JointId) {
    rec_w_u64(buf, v.store());
}

// Geometry is pointer-free POD; C memcpys the structs, which have no padding,
// so field-order writes are byte-identical.

pub fn rec_w_circle(buf: &mut Vec<u8>, v: Circle) {
    rec_w_vec2(buf, v.center);
    rec_w_f32(buf, v.radius);
}

pub fn rec_w_capsule(buf: &mut Vec<u8>, v: Capsule) {
    rec_w_vec2(buf, v.center1);
    rec_w_vec2(buf, v.center2);
    rec_w_f32(buf, v.radius);
}

pub fn rec_w_segment(buf: &mut Vec<u8>, v: Segment) {
    rec_w_vec2(buf, v.point1);
    rec_w_vec2(buf, v.point2);
}

pub fn rec_w_polygon(buf: &mut Vec<u8>, v: &Polygon) {
    for vertex in v.vertices.iter() {
        rec_w_vec2(buf, *vertex);
    }
    for normal in v.normals.iter() {
        rec_w_vec2(buf, *normal);
    }
    rec_w_vec2(buf, v.centroid);
    rec_w_f32(buf, v.radius);
    rec_w_i32(buf, v.count);
}

pub fn rec_w_chainseg(buf: &mut Vec<u8>, v: ChainSegment) {
    rec_w_vec2(buf, v.ghost1);
    rec_w_segment(buf, v.segment);
    rec_w_vec2(buf, v.ghost2);
    rec_w_i32(buf, v.chain_id);
}

pub fn rec_w_filter(buf: &mut Vec<u8>, v: Filter) {
    rec_w_u64(buf, v.category_bits);
    rec_w_u64(buf, v.mask_bits);
    rec_w_i32(buf, v.group_index);
}

pub fn rec_w_material(buf: &mut Vec<u8>, v: SurfaceMaterial) {
    rec_w_f32(buf, v.friction);
    rec_w_f32(buf, v.restitution);
    rec_w_f32(buf, v.rolling_resistance);
    rec_w_f32(buf, v.tangent_speed);
    rec_w_u64(buf, v.user_material_id);
    rec_w_u32(buf, v.custom_color);
}

pub fn rec_w_massdata(buf: &mut Vec<u8>, v: MassData) {
    rec_w_f32(buf, v.mass);
    rec_w_vec2(buf, v.center);
    rec_w_f32(buf, v.rotational_inertia);
}

pub fn rec_w_locks(buf: &mut Vec<u8>, v: MotionLocks) {
    rec_w_bool(buf, v.linear_x);
    rec_w_bool(buf, v.linear_y);
    rec_w_bool(buf, v.angular_z);
}

/// Length-prefixed string; 0xFFFF marks NULL in C. Rust strings are never
/// null, so that sentinel only appears when reading C recordings.
/// (b2RecW_STR)
pub fn rec_w_str(buf: &mut Vec<u8>, s: &str) {
    let bytes = s.as_bytes();
    let len = bytes.len().min(65534);
    rec_w_u16(buf, len as u16);
    buf.extend_from_slice(&bytes[..len]);
}

pub fn rec_w_explosiondef(buf: &mut Vec<u8>, v: &ExplosionDef) {
    rec_w_u64(buf, v.mask_bits);
    rec_w_position(buf, v.position);
    rec_w_f32(buf, v.radius);
    rec_w_f32(buf, v.falloff);
    rec_w_f32(buf, v.impulse_per_length);
}

// Hand-written def helpers. userData and the validity cookie are not
// serialized; readers start from b2Default*Def() and overwrite fields.

pub fn rec_w_bodydef(buf: &mut Vec<u8>, v: &BodyDef) {
    rec_w_i32(buf, v.type_ as i32);
    rec_w_position(buf, v.position);
    rec_w_rot(buf, v.rotation);
    rec_w_vec2(buf, v.linear_velocity);
    rec_w_f32(buf, v.angular_velocity);
    rec_w_f32(buf, v.linear_damping);
    rec_w_f32(buf, v.angular_damping);
    rec_w_f32(buf, v.gravity_scale);
    rec_w_f32(buf, v.sleep_threshold);
    rec_w_str(buf, &v.name);
    // userData: not preserved
    rec_w_u64(buf, 0);
    rec_w_locks(buf, v.motion_locks);
    rec_w_bool(buf, v.enable_sleep);
    rec_w_bool(buf, v.is_awake);
    rec_w_bool(buf, v.is_bullet);
    rec_w_bool(buf, v.is_enabled);
    rec_w_bool(buf, v.allow_fast_rotation);
    rec_w_bool(buf, v.enable_contact_recycling);
}

pub fn rec_w_shapedef(buf: &mut Vec<u8>, v: &ShapeDef) {
    // userData: not preserved
    rec_w_u64(buf, 0);
    rec_w_material(buf, v.material);
    rec_w_f32(buf, v.density);
    rec_w_filter(buf, v.filter);
    rec_w_bool(buf, v.enable_custom_filtering);
    rec_w_bool(buf, v.is_sensor);
    rec_w_bool(buf, v.enable_sensor_events);
    rec_w_bool(buf, v.enable_contact_events);
    rec_w_bool(buf, v.enable_hit_events);
    rec_w_bool(buf, v.enable_pre_solve_events);
    rec_w_bool(buf, v.invoke_contact_creation);
    rec_w_bool(buf, v.update_body_mass);
}

/// Variable-length def: point and material arrays are length-prefixed and
/// inlined. (b2RecW_CHAINDEF)
pub fn rec_w_chaindef(buf: &mut Vec<u8>, v: &ChainDef) {
    // userData: not preserved
    rec_w_u64(buf, 0);
    rec_w_i32(buf, v.points.len() as i32);
    for point in v.points.iter() {
        rec_w_vec2(buf, *point);
    }
    rec_w_i32(buf, v.materials.len() as i32);
    for material in v.materials.iter() {
        rec_w_material(buf, *material);
    }
    rec_w_filter(buf, v.filter);
    rec_w_bool(buf, v.is_loop);
    rec_w_bool(buf, v.enable_sensor_events);
}

/// Joint defs share a base. Body ids are written as ids and remapped to the
/// replay world on read. (static b2RecW_JointBase)
fn rec_w_joint_base(buf: &mut Vec<u8>, base: &JointDef) {
    rec_w_u64(buf, 0); // userData
    rec_w_bodyid(buf, base.body_id_a);
    rec_w_bodyid(buf, base.body_id_b);
    rec_w_xf(buf, base.local_frame_a);
    rec_w_xf(buf, base.local_frame_b);
    rec_w_f32(buf, base.force_threshold);
    rec_w_f32(buf, base.torque_threshold);
    rec_w_f32(buf, base.constraint_hertz);
    rec_w_f32(buf, base.constraint_damping_ratio);
    rec_w_f32(buf, base.draw_scale);
    rec_w_bool(buf, base.collide_connected);
}

pub fn rec_w_distancejointdef(buf: &mut Vec<u8>, v: &DistanceJointDef) {
    rec_w_joint_base(buf, &v.base);
    rec_w_f32(buf, v.length);
    rec_w_bool(buf, v.enable_spring);
    rec_w_f32(buf, v.lower_spring_force);
    rec_w_f32(buf, v.upper_spring_force);
    rec_w_f32(buf, v.hertz);
    rec_w_f32(buf, v.damping_ratio);
    rec_w_bool(buf, v.enable_limit);
    rec_w_f32(buf, v.min_length);
    rec_w_f32(buf, v.max_length);
    rec_w_bool(buf, v.enable_motor);
    rec_w_f32(buf, v.max_motor_force);
    rec_w_f32(buf, v.motor_speed);
}

pub fn rec_w_motorjointdef(buf: &mut Vec<u8>, v: &MotorJointDef) {
    rec_w_joint_base(buf, &v.base);
    rec_w_vec2(buf, v.linear_velocity);
    rec_w_f32(buf, v.max_velocity_force);
    rec_w_f32(buf, v.angular_velocity);
    rec_w_f32(buf, v.max_velocity_torque);
    rec_w_f32(buf, v.linear_hertz);
    rec_w_f32(buf, v.linear_damping_ratio);
    rec_w_f32(buf, v.max_spring_force);
    rec_w_f32(buf, v.angular_hertz);
    rec_w_f32(buf, v.angular_damping_ratio);
    rec_w_f32(buf, v.max_spring_torque);
}

pub fn rec_w_filterjointdef(buf: &mut Vec<u8>, v: &FilterJointDef) {
    rec_w_joint_base(buf, &v.base);
}

pub fn rec_w_prismaticjointdef(buf: &mut Vec<u8>, v: &PrismaticJointDef) {
    rec_w_joint_base(buf, &v.base);
    rec_w_bool(buf, v.enable_spring);
    rec_w_f32(buf, v.hertz);
    rec_w_f32(buf, v.damping_ratio);
    rec_w_f32(buf, v.target_translation);
    rec_w_bool(buf, v.enable_limit);
    rec_w_f32(buf, v.lower_translation);
    rec_w_f32(buf, v.upper_translation);
    rec_w_bool(buf, v.enable_motor);
    rec_w_f32(buf, v.max_motor_force);
    rec_w_f32(buf, v.motor_speed);
}

pub fn rec_w_revolutejointdef(buf: &mut Vec<u8>, v: &RevoluteJointDef) {
    rec_w_joint_base(buf, &v.base);
    rec_w_f32(buf, v.target_angle);
    rec_w_bool(buf, v.enable_spring);
    rec_w_f32(buf, v.hertz);
    rec_w_f32(buf, v.damping_ratio);
    rec_w_bool(buf, v.enable_limit);
    rec_w_f32(buf, v.lower_angle);
    rec_w_f32(buf, v.upper_angle);
    rec_w_bool(buf, v.enable_motor);
    rec_w_f32(buf, v.max_motor_torque);
    rec_w_f32(buf, v.motor_speed);
}

pub fn rec_w_weldjointdef(buf: &mut Vec<u8>, v: &WeldJointDef) {
    rec_w_joint_base(buf, &v.base);
    rec_w_f32(buf, v.linear_hertz);
    rec_w_f32(buf, v.angular_hertz);
    rec_w_f32(buf, v.linear_damping_ratio);
    rec_w_f32(buf, v.angular_damping_ratio);
}

pub fn rec_w_wheeljointdef(buf: &mut Vec<u8>, v: &WheelJointDef) {
    rec_w_joint_base(buf, &v.base);
    rec_w_bool(buf, v.enable_spring);
    rec_w_f32(buf, v.hertz);
    rec_w_f32(buf, v.damping_ratio);
    rec_w_bool(buf, v.enable_limit);
    rec_w_f32(buf, v.lower_translation);
    rec_w_f32(buf, v.upper_translation);
    rec_w_bool(buf, v.enable_motor);
    rec_w_f32(buf, v.max_motor_torque);
    rec_w_f32(buf, v.motor_speed);
}

pub fn rec_w_aabb(buf: &mut Vec<u8>, v: Aabb) {
    rec_w_vec2(buf, v.lower_bound);
    rec_w_vec2(buf, v.upper_bound);
}

pub fn rec_w_queryfilter(buf: &mut Vec<u8>, v: QueryFilter) {
    rec_w_u64(buf, v.category_bits);
    rec_w_u64(buf, v.mask_bits);
}

pub fn rec_w_shapeproxy(buf: &mut Vec<u8>, v: &ShapeProxy) {
    let count = v.count.clamp(0, MAX_POLYGON_VERTICES as i32);
    rec_w_i32(buf, count);
    for point in v.points[..count as usize].iter() {
        rec_w_vec2(buf, *point);
    }
    rec_w_f32(buf, v.radius);
}

pub fn rec_w_worldcastoutput(buf: &mut Vec<u8>, v: WorldCastOutput) {
    rec_w_vec2(buf, v.normal);
    rec_w_position(buf, v.point);
    rec_w_f32(buf, v.fraction);
    rec_w_i32(buf, v.iterations);
    rec_w_bool(buf, v.hit);
}

pub fn rec_w_rayresult(buf: &mut Vec<u8>, v: &RayResult) {
    rec_w_shapeid(buf, v.shape_id);
    rec_w_position(buf, v.point);
    rec_w_vec2(buf, v.normal);
    rec_w_f32(buf, v.fraction);
    rec_w_i32(buf, v.node_visits);
    rec_w_i32(buf, v.leaf_visits);
    rec_w_bool(buf, v.hit);
}

pub fn rec_w_planeresult(buf: &mut Vec<u8>, v: &crate::collision::PlaneResult) {
    rec_w_vec2(buf, v.plane.normal);
    rec_w_f32(buf, v.plane.offset);
    rec_w_vec2(buf, v.point);
    rec_w_bool(buf, v.hit);
}

pub fn rec_w_treestats(buf: &mut Vec<u8>, v: TreeStats) {
    rec_w_i32(buf, v.node_visits);
    rec_w_i32(buf, v.leaf_visits);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recording::{rec_patch_u32, rec_reserve_u32, RecHeader, Recording};

    // Wire layout: primitives are little-endian and framing backpatches the
    // 24-bit payload size, matching the C format byte for byte.
    #[test]
    fn wire_primitives_and_framing() {
        let mut buf = Vec::new();
        rec_w_u16(&mut buf, 0x1234);
        rec_w_u32(&mut buf, 0xAABBCCDD);
        rec_w_f32(&mut buf, 1.0);
        rec_w_bool(&mut buf, true);
        assert_eq!(
            buf,
            [0x34, 0x12, 0xDD, 0xCC, 0xBB, 0xAA, 0x00, 0x00, 0x80, 0x3F, 0x01]
        );

        // Framing: opcode, u24 size, payload.
        let mut rec = Recording::new(64);
        rec.begin_record(0x80);
        rec_w_f32(&mut rec.buffer, 1.0 / 60.0);
        rec_w_i32(&mut rec.buffer, 4);
        rec.end_record();
        assert_eq!(rec.buffer[0], 0x80);
        assert_eq!(&rec.buffer[1..4], &[8, 0, 0]); // 8-byte payload
        assert_eq!(rec.buffer.len(), 12);

        // commit_record writes the same framing in one shot.
        let mut rec2 = Recording::new(64);
        rec2.commit_record(0x80, &rec.buffer[4..12]);
        assert_eq!(rec2.buffer, rec.buffer);

        // Reserve/patch round trip.
        let mut qbuf = Vec::new();
        let offset = rec_reserve_u32(&mut qbuf);
        rec_w_u8(&mut qbuf, 7);
        rec_patch_u32(&mut qbuf, offset, 3);
        assert_eq!(qbuf, [3, 0, 0, 0, 7]);
    }

    // The 32-byte header round trips through its exact C layout.
    #[test]
    fn header_round_trip() {
        let header = RecHeader {
            magic: crate::recording::REC_MAGIC,
            version_major: crate::recording::REC_VERSION_MAJOR,
            version_minor: crate::recording::REC_VERSION_MINOR,
            length_scale: 1.0,
            pointer_width: 8,
            big_endian: 0,
            validation_enabled: 1,
            snapshot_size: 1234,
        };
        let mut buf = Vec::new();
        header.write(&mut buf);
        assert_eq!(buf.len(), RecHeader::SIZE);
        assert_eq!(&buf[0..4], b"B2RC");
        assert_eq!(RecHeader::read(&buf), Some(header));
    }

    // The string writer length-prefixes and truncates like C.
    #[test]
    fn string_writer() {
        let mut buf = Vec::new();
        rec_w_str(&mut buf, "box");
        assert_eq!(buf, [3, 0, b'b', b'o', b'x']);

        let mut empty = Vec::new();
        rec_w_str(&mut empty, "");
        assert_eq!(empty, [0, 0]);
    }
}
