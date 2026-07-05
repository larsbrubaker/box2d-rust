// Per-struct serialize/deserialize for the world snapshot, standing in for
// the C memcpy of internal PODs in world_snapshot.c. Fields are written in
// declaration order; any struct change is caught by the layout hash.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::snapshot::SnapReader;
use super::write::*;
use crate::body::{Body, BodySim, BodyState};
use crate::collision::{Manifold, ManifoldPoint, ShapeGeometry};
use crate::contact::{Contact, ContactEdge, ContactSim};
use crate::distance::SimplexCache;
use crate::island::{ContactLink, Island, IslandSim, JointLink};
use crate::joint::{Joint, JointEdge, JointPayload, JointSim, JointType};
use crate::math_functions::{Mat22, Pos, Rot, Transform, Vec2, WorldTransform};
use crate::sensor::{Sensor, Visitor};
use crate::shape::Shape;
use crate::solver::Softness;
use crate::types::BodyType;

// Leaf helpers

fn r_vec2(r: &mut SnapReader) -> Vec2 {
    Vec2 {
        x: r.r_f32(),
        y: r.r_f32(),
    }
}

fn r_rot(r: &mut SnapReader) -> Rot {
    Rot {
        c: r.r_f32(),
        s: r.r_f32(),
    }
}

fn r_xf(r: &mut SnapReader) -> Transform {
    Transform {
        p: r_vec2(r),
        q: r_rot(r),
    }
}

#[cfg(feature = "double-precision")]
fn r_position(r: &mut SnapReader) -> Pos {
    Pos {
        x: r.r_f64(),
        y: r.r_f64(),
    }
}

#[cfg(not(feature = "double-precision"))]
fn r_position(r: &mut SnapReader) -> Pos {
    Pos {
        x: r.r_f32(),
        y: r.r_f32(),
    }
}

fn r_worldxf(r: &mut SnapReader) -> WorldTransform {
    WorldTransform {
        p: r_position(r),
        q: r_rot(r),
    }
}

fn w_softness(buf: &mut Vec<u8>, s: Softness) {
    rec_w_f32(buf, s.bias_rate);
    rec_w_f32(buf, s.mass_scale);
    rec_w_f32(buf, s.impulse_scale);
}

fn r_softness(r: &mut SnapReader) -> Softness {
    Softness {
        bias_rate: r.r_f32(),
        mass_scale: r.r_f32(),
        impulse_scale: r.r_f32(),
    }
}

fn w_mat22(buf: &mut Vec<u8>, m: Mat22) {
    rec_w_vec2(buf, m.cx);
    rec_w_vec2(buf, m.cy);
}

fn r_mat22(r: &mut SnapReader) -> Mat22 {
    Mat22 {
        cx: r_vec2(r),
        cy: r_vec2(r),
    }
}

fn r_body_type(r: &mut SnapReader) -> BodyType {
    match r.r_i32() {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        _ => BodyType::Dynamic,
    }
}

// Bodies

pub(crate) fn ser_body(buf: &mut Vec<u8>, b: &Body) {
    // userData: not preserved, same as the C recorder scrubbing to NULL
    rec_w_u64(buf, 0);
    rec_w_i32(buf, b.set_index);
    rec_w_i32(buf, b.local_index);
    rec_w_i32(buf, b.head_contact_key);
    rec_w_i32(buf, b.contact_count);
    rec_w_i32(buf, b.head_shape_id);
    rec_w_i32(buf, b.shape_count);
    rec_w_i32(buf, b.head_chain_id);
    rec_w_i32(buf, b.head_joint_key);
    rec_w_i32(buf, b.joint_count);
    rec_w_i32(buf, b.island_id);
    rec_w_i32(buf, b.island_index);
    rec_w_f32(buf, b.mass);
    rec_w_f32(buf, b.inertia);
    rec_w_f32(buf, b.sleep_threshold);
    rec_w_f32(buf, b.sleep_time);
    rec_w_i32(buf, b.body_move_index);
    rec_w_i32(buf, b.id);
    rec_w_u32(buf, b.flags);
    rec_w_i32(buf, b.type_ as i32);
    rec_w_u16(buf, b.generation);
    rec_w_str(buf, &b.name);
}

pub(crate) fn des_body(r: &mut SnapReader) -> Body {
    let mut b = Body::default();
    b.user_data = r.r_u64();
    b.set_index = r.r_i32();
    b.local_index = r.r_i32();
    b.head_contact_key = r.r_i32();
    b.contact_count = r.r_i32();
    b.head_shape_id = r.r_i32();
    b.shape_count = r.r_i32();
    b.head_chain_id = r.r_i32();
    b.head_joint_key = r.r_i32();
    b.joint_count = r.r_i32();
    b.island_id = r.r_i32();
    b.island_index = r.r_i32();
    b.mass = r.r_f32();
    b.inertia = r.r_f32();
    b.sleep_threshold = r.r_f32();
    b.sleep_time = r.r_f32();
    b.body_move_index = r.r_i32();
    b.id = r.r_i32();
    b.flags = r.r_u32();
    b.type_ = r_body_type(r);
    b.generation = r.r_u16();
    b.name = r_string(r);
    b
}

fn r_string(r: &mut SnapReader) -> String {
    let len = r.r_u16();
    if len == 0xFFFF {
        // C NULL marker
        return String::new();
    }
    let mut out = String::with_capacity(len as usize);
    for _ in 0..len {
        out.push(r.r_u8() as char);
    }
    out
}

pub(crate) fn ser_body_sim(buf: &mut Vec<u8>, s: &BodySim) {
    rec_w_worldxf(buf, s.transform);
    rec_w_position(buf, s.center);
    rec_w_rot(buf, s.rotation0);
    rec_w_position(buf, s.center0);
    rec_w_vec2(buf, s.local_center);
    rec_w_vec2(buf, s.force);
    rec_w_f32(buf, s.torque);
    rec_w_f32(buf, s.inv_mass);
    rec_w_f32(buf, s.inv_inertia);
    rec_w_f32(buf, s.min_extent);
    rec_w_f32(buf, s.max_extent);
    rec_w_f32(buf, s.linear_damping);
    rec_w_f32(buf, s.angular_damping);
    rec_w_f32(buf, s.gravity_scale);
    rec_w_i32(buf, s.body_id);
    rec_w_u32(buf, s.flags);
}

pub(crate) fn des_body_sim(r: &mut SnapReader) -> BodySim {
    let mut s = BodySim::default();
    s.transform = r_worldxf(r);
    s.center = r_position(r);
    s.rotation0 = r_rot(r);
    s.center0 = r_position(r);
    s.local_center = r_vec2(r);
    s.force = r_vec2(r);
    s.torque = r.r_f32();
    s.inv_mass = r.r_f32();
    s.inv_inertia = r.r_f32();
    s.min_extent = r.r_f32();
    s.max_extent = r.r_f32();
    s.linear_damping = r.r_f32();
    s.angular_damping = r.r_f32();
    s.gravity_scale = r.r_f32();
    s.body_id = r.r_i32();
    s.flags = r.r_u32();
    s
}

pub(crate) fn ser_body_state(buf: &mut Vec<u8>, s: &BodyState) {
    rec_w_vec2(buf, s.linear_velocity);
    rec_w_f32(buf, s.angular_velocity);
    rec_w_u32(buf, s.flags);
    rec_w_vec2(buf, s.delta_position);
    rec_w_rot(buf, s.delta_rotation);
}

pub(crate) fn des_body_state(r: &mut SnapReader) -> BodyState {
    let mut s = BodyState::default();
    s.linear_velocity = r_vec2(r);
    s.angular_velocity = r.r_f32();
    s.flags = r.r_u32();
    s.delta_position = r_vec2(r);
    s.delta_rotation = r_rot(r);
    s
}

// Shapes

fn ser_geometry(buf: &mut Vec<u8>, g: &ShapeGeometry) {
    match g {
        ShapeGeometry::Circle(c) => {
            rec_w_u8(buf, 0);
            rec_w_circle(buf, *c);
        }
        ShapeGeometry::Capsule(c) => {
            rec_w_u8(buf, 1);
            rec_w_capsule(buf, *c);
        }
        ShapeGeometry::Segment(s) => {
            rec_w_u8(buf, 2);
            rec_w_segment(buf, *s);
        }
        ShapeGeometry::Polygon(p) => {
            rec_w_u8(buf, 3);
            rec_w_polygon(buf, p);
        }
        ShapeGeometry::ChainSegment(c) => {
            rec_w_u8(buf, 4);
            rec_w_chainseg(buf, *c);
        }
    }
}

fn des_geometry(r: &mut SnapReader) -> ShapeGeometry {
    use crate::collision::{Capsule, ChainSegment, Circle, Polygon, Segment};
    match r.r_u8() {
        0 => ShapeGeometry::Circle(Circle {
            center: r_vec2(r),
            radius: r.r_f32(),
        }),
        1 => ShapeGeometry::Capsule(Capsule {
            center1: r_vec2(r),
            center2: r_vec2(r),
            radius: r.r_f32(),
        }),
        2 => ShapeGeometry::Segment(Segment {
            point1: r_vec2(r),
            point2: r_vec2(r),
        }),
        3 => {
            let mut p = Polygon::default();
            for v in p.vertices.iter_mut() {
                *v = r_vec2(r);
            }
            for n in p.normals.iter_mut() {
                *n = r_vec2(r);
            }
            p.centroid = r_vec2(r);
            p.radius = r.r_f32();
            p.count = r.r_i32();
            ShapeGeometry::Polygon(p)
        }
        _ => ShapeGeometry::ChainSegment(ChainSegment {
            ghost1: r_vec2(r),
            segment: Segment {
                point1: r_vec2(r),
                point2: r_vec2(r),
            },
            ghost2: r_vec2(r),
            chain_id: r.r_i32(),
        }),
    }
}

pub(crate) fn ser_shape(buf: &mut Vec<u8>, s: &Shape) {
    rec_w_i32(buf, s.id);
    rec_w_i32(buf, s.body_id);
    rec_w_i32(buf, s.prev_shape_id);
    rec_w_i32(buf, s.next_shape_id);
    rec_w_i32(buf, s.sensor_index);
    rec_w_material(buf, s.material);
    rec_w_f32(buf, s.density);
    rec_w_f32(buf, s.aabb_margin);
    rec_w_aabb(buf, s.aabb);
    rec_w_aabb(buf, s.fat_aabb);
    rec_w_vec2(buf, s.local_centroid);
    rec_w_i32(buf, s.proxy_key);
    rec_w_filter(buf, s.filter);
    // userData: not preserved
    rec_w_u64(buf, 0);
    ser_geometry(buf, &s.geometry);
    rec_w_u16(buf, s.generation);
    rec_w_bool(buf, s.enable_sensor_events);
    rec_w_bool(buf, s.enable_contact_events);
    rec_w_bool(buf, s.enable_custom_filtering);
    rec_w_bool(buf, s.enable_hit_events);
    rec_w_bool(buf, s.enable_pre_solve_events);
    rec_w_bool(buf, s.enlarged_aabb);
}

pub(crate) fn des_shape(r: &mut SnapReader) -> Shape {
    let mut s = Shape::default();
    s.id = r.r_i32();
    s.body_id = r.r_i32();
    s.prev_shape_id = r.r_i32();
    s.next_shape_id = r.r_i32();
    s.sensor_index = r.r_i32();
    s.material = r_material(r);
    s.density = r.r_f32();
    s.aabb_margin = r.r_f32();
    s.aabb = r_aabb(r);
    s.fat_aabb = r_aabb(r);
    s.local_centroid = r_vec2(r);
    s.proxy_key = r.r_i32();
    s.filter = r_filter(r);
    s.user_data = r.r_u64();
    s.geometry = des_geometry(r);
    s.generation = r.r_u16();
    s.enable_sensor_events = r.r_bool();
    s.enable_contact_events = r.r_bool();
    s.enable_custom_filtering = r.r_bool();
    s.enable_hit_events = r.r_bool();
    s.enable_pre_solve_events = r.r_bool();
    s.enlarged_aabb = r.r_bool();
    s
}

fn r_material(r: &mut SnapReader) -> crate::types::SurfaceMaterial {
    let mut m = crate::types::default_surface_material();
    m.friction = r.r_f32();
    m.restitution = r.r_f32();
    m.rolling_resistance = r.r_f32();
    m.tangent_speed = r.r_f32();
    m.user_material_id = r.r_u64();
    m.custom_color = r.r_u32();
    m
}

fn r_filter(r: &mut SnapReader) -> crate::types::Filter {
    let mut f = crate::types::default_filter();
    f.category_bits = r.r_u64();
    f.mask_bits = r.r_u64();
    f.group_index = r.r_i32();
    f
}

fn r_aabb(r: &mut SnapReader) -> crate::math_functions::Aabb {
    crate::math_functions::Aabb {
        lower_bound: r_vec2(r),
        upper_bound: r_vec2(r),
    }
}

// Contacts

fn w_contact_edge(buf: &mut Vec<u8>, e: &ContactEdge) {
    rec_w_i32(buf, e.body_id);
    rec_w_i32(buf, e.prev_key);
    rec_w_i32(buf, e.next_key);
}

fn r_contact_edge(r: &mut SnapReader) -> ContactEdge {
    ContactEdge {
        body_id: r.r_i32(),
        prev_key: r.r_i32(),
        next_key: r.r_i32(),
    }
}

pub(crate) fn ser_contact(buf: &mut Vec<u8>, c: &Contact) {
    w_contact_edge(buf, &c.edges[0]);
    w_contact_edge(buf, &c.edges[1]);
    rec_w_i32(buf, c.island_id);
    rec_w_i32(buf, c.island_index);
    rec_w_i32(buf, c.set_index);
    rec_w_i32(buf, c.color_index);
    rec_w_i32(buf, c.local_index);
    rec_w_i32(buf, c.shape_id_a);
    rec_w_i32(buf, c.shape_id_b);
    rec_w_i32(buf, c.contact_id);
    rec_w_u32(buf, c.flags);
    rec_w_u32(buf, c.generation);
}

pub(crate) fn des_contact(r: &mut SnapReader) -> Contact {
    let mut c = Contact::default();
    c.edges = [r_contact_edge(r), r_contact_edge(r)];
    c.island_id = r.r_i32();
    c.island_index = r.r_i32();
    c.set_index = r.r_i32();
    c.color_index = r.r_i32();
    c.local_index = r.r_i32();
    c.shape_id_a = r.r_i32();
    c.shape_id_b = r.r_i32();
    c.contact_id = r.r_i32();
    c.flags = r.r_u32();
    c.generation = r.r_u32();
    c
}

fn w_manifold(buf: &mut Vec<u8>, m: &Manifold) {
    rec_w_vec2(buf, m.normal);
    rec_w_f32(buf, m.rolling_impulse);
    for p in m.points.iter() {
        rec_w_vec2(buf, p.anchor_a);
        rec_w_vec2(buf, p.anchor_b);
        rec_w_f32(buf, p.separation);
        rec_w_f32(buf, p.base_separation);
        rec_w_f32(buf, p.normal_impulse);
        rec_w_f32(buf, p.tangent_impulse);
        rec_w_f32(buf, p.total_normal_impulse);
        rec_w_f32(buf, p.normal_velocity);
        rec_w_u16(buf, p.id);
        rec_w_bool(buf, p.persisted);
    }
    rec_w_i32(buf, m.point_count);
}

fn r_manifold(r: &mut SnapReader) -> Manifold {
    let mut m = Manifold::default();
    m.normal = r_vec2(r);
    m.rolling_impulse = r.r_f32();
    for p in m.points.iter_mut() {
        *p = ManifoldPoint {
            anchor_a: r_vec2(r),
            anchor_b: r_vec2(r),
            separation: r.r_f32(),
            base_separation: r.r_f32(),
            normal_impulse: r.r_f32(),
            tangent_impulse: r.r_f32(),
            total_normal_impulse: r.r_f32(),
            normal_velocity: r.r_f32(),
            id: r.r_u16(),
            persisted: r.r_bool(),
        };
    }
    m.point_count = r.r_i32();
    m
}

pub(crate) fn ser_contact_sim(buf: &mut Vec<u8>, c: &ContactSim) {
    rec_w_i32(buf, c.contact_id);
    rec_w_rot(buf, c.cached_rotation_a);
    rec_w_rot(buf, c.cached_rotation_b);
    rec_w_xf(buf, c.cached_relative_pose);
    rec_w_i32(buf, c.body_id_a);
    rec_w_i32(buf, c.body_id_b);
    rec_w_i32(buf, c.body_sim_index_a);
    rec_w_i32(buf, c.body_sim_index_b);
    rec_w_i32(buf, c.shape_id_a);
    rec_w_i32(buf, c.shape_id_b);
    rec_w_f32(buf, c.inv_mass_a);
    rec_w_f32(buf, c.inv_i_a);
    rec_w_f32(buf, c.inv_mass_b);
    rec_w_f32(buf, c.inv_i_b);
    w_manifold(buf, &c.manifold);
    rec_w_f32(buf, c.friction);
    rec_w_f32(buf, c.restitution);
    rec_w_f32(buf, c.rolling_resistance);
    rec_w_f32(buf, c.tangent_speed);
    rec_w_u32(buf, c.sim_flags);
    rec_w_u16(buf, c.cache.count);
    for i in 0..3 {
        rec_w_u8(buf, c.cache.index_a[i]);
    }
    for i in 0..3 {
        rec_w_u8(buf, c.cache.index_b[i]);
    }
}

pub(crate) fn des_contact_sim(r: &mut SnapReader) -> ContactSim {
    let mut c = ContactSim::default();
    c.contact_id = r.r_i32();
    c.cached_rotation_a = r_rot(r);
    c.cached_rotation_b = r_rot(r);
    c.cached_relative_pose = r_xf(r);
    c.body_id_a = r.r_i32();
    c.body_id_b = r.r_i32();
    c.body_sim_index_a = r.r_i32();
    c.body_sim_index_b = r.r_i32();
    c.shape_id_a = r.r_i32();
    c.shape_id_b = r.r_i32();
    c.inv_mass_a = r.r_f32();
    c.inv_i_a = r.r_f32();
    c.inv_mass_b = r.r_f32();
    c.inv_i_b = r.r_f32();
    c.manifold = r_manifold(r);
    c.friction = r.r_f32();
    c.restitution = r.r_f32();
    c.rolling_resistance = r.r_f32();
    c.tangent_speed = r.r_f32();
    c.sim_flags = r.r_u32();
    c.cache = SimplexCache {
        count: r.r_u16(),
        index_a: [r.r_u8(), r.r_u8(), r.r_u8()],
        index_b: [r.r_u8(), r.r_u8(), r.r_u8()],
    };
    c
}

// Islands and sensors

pub(crate) fn ser_island(buf: &mut Vec<u8>, island: &Island) {
    rec_w_i32(buf, island.set_index);
    rec_w_i32(buf, island.local_index);
    rec_w_i32(buf, island.island_id);
    rec_w_i32(buf, island.constraint_remove_count);
    rec_w_i32(buf, island.bodies.len() as i32);
    for &b in island.bodies.iter() {
        rec_w_i32(buf, b);
    }
    rec_w_i32(buf, island.contacts.len() as i32);
    for link in island.contacts.iter() {
        rec_w_i32(buf, link.contact_id);
        rec_w_i32(buf, link.body_id_a);
        rec_w_i32(buf, link.body_id_b);
    }
    rec_w_i32(buf, island.joints.len() as i32);
    for link in island.joints.iter() {
        rec_w_i32(buf, link.joint_id);
        rec_w_i32(buf, link.body_id_a);
        rec_w_i32(buf, link.body_id_b);
    }
}

pub(crate) fn des_island(r: &mut SnapReader) -> Island {
    let mut island = Island::default();
    island.set_index = r.r_i32();
    island.local_index = r.r_i32();
    island.island_id = r.r_i32();
    island.constraint_remove_count = r.r_i32();
    let n = r.r_i32();
    if !r.check_count(n, 4) {
        return island;
    }
    island.bodies = (0..n).map(|_| r.r_i32()).collect();
    let n = r.r_i32();
    if !r.check_count(n, 12) {
        return island;
    }
    island.contacts = (0..n)
        .map(|_| ContactLink {
            contact_id: r.r_i32(),
            body_id_a: r.r_i32(),
            body_id_b: r.r_i32(),
        })
        .collect();
    let n = r.r_i32();
    if !r.check_count(n, 12) {
        return island;
    }
    island.joints = (0..n)
        .map(|_| JointLink {
            joint_id: r.r_i32(),
            body_id_a: r.r_i32(),
            body_id_b: r.r_i32(),
        })
        .collect();
    island
}

pub(crate) fn ser_island_sim(buf: &mut Vec<u8>, s: &IslandSim) {
    rec_w_i32(buf, s.island_id);
}

pub(crate) fn des_island_sim(r: &mut SnapReader) -> IslandSim {
    IslandSim {
        island_id: r.r_i32(),
    }
}

fn w_visitors(buf: &mut Vec<u8>, visitors: &[Visitor]) {
    rec_w_i32(buf, visitors.len() as i32);
    for v in visitors.iter() {
        rec_w_i32(buf, v.shape_id);
        rec_w_u16(buf, v.generation);
    }
}

fn r_visitors(r: &mut SnapReader) -> Vec<Visitor> {
    let n = r.r_i32();
    if !r.check_count(n, 6) {
        return Vec::new();
    }
    (0..n)
        .map(|_| Visitor {
            shape_id: r.r_i32(),
            generation: r.r_u16(),
        })
        .collect()
}

pub(crate) fn ser_sensor(buf: &mut Vec<u8>, s: &Sensor) {
    rec_w_i32(buf, s.shape_id);
    w_visitors(buf, &s.hits);
    w_visitors(buf, &s.overlaps1);
    w_visitors(buf, &s.overlaps2);
}

pub(crate) fn des_sensor(r: &mut SnapReader) -> Sensor {
    let mut s = Sensor::default();
    s.shape_id = r.r_i32();
    s.hits = r_visitors(r);
    s.overlaps1 = r_visitors(r);
    s.overlaps2 = r_visitors(r);
    s
}

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
