// Shape and chain op family (0x40-0x72): writers for the B2_REC hooks in
// the b2Shape_*/b2Chain_* API and their replay dispatchers. Create ops
// append the returned id and replay asserts it, same as the body family.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::snapshot::SnapReader;
use super::snapshot_structs::{r_filter, r_material, r_vec2};
use super::write::*;
use super::Recording;
use crate::collision::{Capsule, ChainSegment, Circle, Polygon, Segment};
use crate::id::{BodyId, ChainId, ShapeId};
use crate::math_functions::Vec2;
use crate::types::{ChainDef, ShapeDef, SurfaceMaterial};
use crate::world::World;

pub const OP_CREATE_CIRCLE_SHAPE: u8 = 0x40;
pub const OP_CREATE_CAPSULE_SHAPE: u8 = 0x41;
pub const OP_CREATE_SEGMENT_SHAPE: u8 = 0x42;
pub const OP_CREATE_POLYGON_SHAPE: u8 = 0x43;
pub const OP_CREATE_CHAIN_SEGMENT_SHAPE: u8 = 0x44;
pub const OP_DESTROY_SHAPE: u8 = 0x45;
pub const OP_SHAPE_SET_DENSITY: u8 = 0x50;
pub const OP_SHAPE_SET_FRICTION: u8 = 0x51;
pub const OP_SHAPE_SET_RESTITUTION: u8 = 0x52;
pub const OP_SHAPE_SET_USER_MATERIAL: u8 = 0x53;
pub const OP_SHAPE_SET_SURFACE_MATERIAL: u8 = 0x54;
pub const OP_SHAPE_SET_FILTER: u8 = 0x55;
pub const OP_SHAPE_ENABLE_SENSOR_EVENTS: u8 = 0x56;
pub const OP_SHAPE_ENABLE_CONTACT_EVENTS: u8 = 0x57;
pub const OP_SHAPE_ENABLE_PRE_SOLVE_EVENTS: u8 = 0x58;
pub const OP_SHAPE_ENABLE_HIT_EVENTS: u8 = 0x59;
pub const OP_SHAPE_SET_CIRCLE: u8 = 0x5A;
pub const OP_SHAPE_SET_CAPSULE: u8 = 0x5B;
pub const OP_SHAPE_SET_SEGMENT: u8 = 0x5C;
pub const OP_SHAPE_SET_POLYGON: u8 = 0x5D;
pub const OP_SHAPE_SET_CHAIN_SEGMENT: u8 = 0x5E;
pub const OP_SHAPE_APPLY_WIND: u8 = 0x5F;
pub const OP_CREATE_CHAIN: u8 = 0x70;
pub const OP_DESTROY_CHAIN: u8 = 0x71;
pub const OP_CHAIN_SET_SURFACE_MATERIAL: u8 = 0x72;

// Writers

/// Shape create ops: body id + def + geometry payload + returned shape id.
/// (b2RecWriteRet_Create*Shape)
pub(crate) fn write_create_shape(
    rec: &mut Recording,
    opcode: u8,
    body: BodyId,
    def: &ShapeDef,
    geometry: impl FnOnce(&mut Vec<u8>),
    id: ShapeId,
) {
    rec.begin_record(opcode);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_shapedef(&mut rec.buffer, def);
    geometry(&mut rec.buffer);
    rec_w_shapeid(&mut rec.buffer, id);
    rec.end_record();
}

pub(crate) fn write_destroy_shape(rec: &mut Recording, shape: ShapeId, update_body_mass: bool) {
    rec.begin_record(OP_DESTROY_SHAPE);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_bool(&mut rec.buffer, update_body_mass);
    rec.end_record();
}

pub(crate) fn write_shape_bool(rec: &mut Recording, opcode: u8, shape: ShapeId, flag: bool) {
    rec.begin_record(opcode);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_bool(&mut rec.buffer, flag);
    rec.end_record();
}

pub(crate) fn write_shape_f32(rec: &mut Recording, opcode: u8, shape: ShapeId, value: f32) {
    rec.begin_record(opcode);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_f32(&mut rec.buffer, value);
    rec.end_record();
}

pub(crate) fn write_shape_set_density(
    rec: &mut Recording,
    shape: ShapeId,
    density: f32,
    update_body_mass: bool,
) {
    rec.begin_record(OP_SHAPE_SET_DENSITY);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_f32(&mut rec.buffer, density);
    rec_w_bool(&mut rec.buffer, update_body_mass);
    rec.end_record();
}

pub(crate) fn write_shape_set_user_material(rec: &mut Recording, shape: ShapeId, material: u64) {
    rec.begin_record(OP_SHAPE_SET_USER_MATERIAL);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_u64(&mut rec.buffer, material);
    rec.end_record();
}

pub(crate) fn write_shape_set_surface_material(
    rec: &mut Recording,
    shape: ShapeId,
    material: SurfaceMaterial,
) {
    rec.begin_record(OP_SHAPE_SET_SURFACE_MATERIAL);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_material(&mut rec.buffer, material);
    rec.end_record();
}

pub(crate) fn write_shape_set_filter(
    rec: &mut Recording,
    shape: ShapeId,
    filter: crate::types::Filter,
) {
    rec.begin_record(OP_SHAPE_SET_FILTER);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_filter(&mut rec.buffer, filter);
    rec.end_record();
}

/// Geometry replacement ops share the shape id + payload layout.
pub(crate) fn write_shape_geometry(
    rec: &mut Recording,
    opcode: u8,
    shape: ShapeId,
    geometry: impl FnOnce(&mut Vec<u8>),
) {
    rec.begin_record(opcode);
    rec_w_shapeid(&mut rec.buffer, shape);
    geometry(&mut rec.buffer);
    rec.end_record();
}

pub(crate) fn write_shape_apply_wind(
    rec: &mut Recording,
    shape: ShapeId,
    wind: Vec2,
    drag: f32,
    lift: f32,
    wake: bool,
) {
    rec.begin_record(OP_SHAPE_APPLY_WIND);
    rec_w_shapeid(&mut rec.buffer, shape);
    rec_w_vec2(&mut rec.buffer, wind);
    rec_w_f32(&mut rec.buffer, drag);
    rec_w_f32(&mut rec.buffer, lift);
    rec_w_bool(&mut rec.buffer, wake);
    rec.end_record();
}

pub(crate) fn write_create_chain(rec: &mut Recording, body: BodyId, def: &ChainDef, id: ChainId) {
    rec.begin_record(OP_CREATE_CHAIN);
    rec_w_bodyid(&mut rec.buffer, body);
    rec_w_chaindef(&mut rec.buffer, def);
    rec_w_chainid(&mut rec.buffer, id);
    rec.end_record();
}

pub(crate) fn write_destroy_chain(rec: &mut Recording, chain: ChainId) {
    rec.begin_record(OP_DESTROY_CHAIN);
    rec_w_chainid(&mut rec.buffer, chain);
    rec.end_record();
}

pub(crate) fn write_chain_set_surface_material(
    rec: &mut Recording,
    chain: ChainId,
    material: SurfaceMaterial,
    material_index: i32,
) {
    rec.begin_record(OP_CHAIN_SET_SURFACE_MATERIAL);
    rec_w_chainid(&mut rec.buffer, chain);
    rec_w_material(&mut rec.buffer, material);
    rec_w_i32(&mut rec.buffer, material_index);
    rec.end_record();
}

// Readers

fn r_shape_id(r: &mut SnapReader) -> ShapeId {
    ShapeId::load(r.r_u64())
}

fn r_shape_def(r: &mut SnapReader) -> ShapeDef {
    let mut def = crate::types::default_shape_def();
    def.user_data = r.r_u64();
    def.material = r_material(r);
    def.density = r.r_f32();
    def.filter = r_filter(r);
    def.enable_custom_filtering = r.r_bool();
    def.is_sensor = r.r_bool();
    def.enable_sensor_events = r.r_bool();
    def.enable_contact_events = r.r_bool();
    def.enable_hit_events = r.r_bool();
    def.enable_pre_solve_events = r.r_bool();
    def.invoke_contact_creation = r.r_bool();
    def.update_body_mass = r.r_bool();
    def
}

fn r_chain_def(r: &mut SnapReader) -> ChainDef {
    let mut def = crate::types::default_chain_def();
    def.user_data = r.r_u64();
    let n = r.r_i32();
    if !r.check_count(n, 8) {
        return def;
    }
    def.points = (0..n).map(|_| r_vec2(r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 28) {
        return def;
    }
    def.materials = (0..n).map(|_| r_material(r)).collect();
    def.filter = r_filter(r);
    def.is_loop = r.r_bool();
    def.enable_sensor_events = r.r_bool();
    def
}

fn r_circle(r: &mut SnapReader) -> Circle {
    Circle {
        center: r_vec2(r),
        radius: r.r_f32(),
    }
}

fn r_capsule(r: &mut SnapReader) -> Capsule {
    Capsule {
        center1: r_vec2(r),
        center2: r_vec2(r),
        radius: r.r_f32(),
    }
}

fn r_segment(r: &mut SnapReader) -> Segment {
    Segment {
        point1: r_vec2(r),
        point2: r_vec2(r),
    }
}

fn r_polygon(r: &mut SnapReader) -> Polygon {
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
    p
}

fn r_chain_segment(r: &mut SnapReader) -> ChainSegment {
    ChainSegment {
        ghost1: r_vec2(r),
        segment: r_segment(r),
        ghost2: r_vec2(r),
        chain_id: r.r_i32(),
    }
}

fn ids_match(created: ShapeId, recorded: ShapeId) -> bool {
    created.index1 == recorded.index1 && created.generation == recorded.generation
}

/// Dispatch a shape/chain-family opcode. Returns None when the opcode is not
/// in this family; Some(ids_match) otherwise.
pub(crate) fn dispatch_shape_op(opcode: u8, r: &mut SnapReader, world: &mut World) -> Option<bool> {
    use crate::shape::*;

    match opcode {
        OP_CREATE_CIRCLE_SHAPE => {
            let body = BodyId::load(r.r_u64());
            let def = r_shape_def(r);
            let circle = r_circle(r);
            let recorded = r_shape_id(r);
            let created = create_circle_shape(world, body, &def, &circle);
            Some(ids_match(created, recorded))
        }
        OP_CREATE_CAPSULE_SHAPE => {
            let body = BodyId::load(r.r_u64());
            let def = r_shape_def(r);
            let capsule = r_capsule(r);
            let recorded = r_shape_id(r);
            let created = create_capsule_shape(world, body, &def, &capsule);
            Some(ids_match(created, recorded))
        }
        OP_CREATE_SEGMENT_SHAPE => {
            let body = BodyId::load(r.r_u64());
            let def = r_shape_def(r);
            let segment = r_segment(r);
            let recorded = r_shape_id(r);
            let created = create_segment_shape(world, body, &def, &segment);
            Some(ids_match(created, recorded))
        }
        OP_CREATE_POLYGON_SHAPE => {
            let body = BodyId::load(r.r_u64());
            let def = r_shape_def(r);
            let polygon = r_polygon(r);
            let recorded = r_shape_id(r);
            let created = create_polygon_shape(world, body, &def, &polygon);
            Some(ids_match(created, recorded))
        }
        OP_CREATE_CHAIN_SEGMENT_SHAPE => {
            let body = BodyId::load(r.r_u64());
            let def = r_shape_def(r);
            let chain_segment = r_chain_segment(r);
            let recorded = r_shape_id(r);
            let created = create_chain_segment_shape(world, body, &def, &chain_segment);
            Some(ids_match(created, recorded))
        }
        OP_DESTROY_SHAPE => {
            let shape = r_shape_id(r);
            let update_body_mass = r.r_bool();
            destroy_shape(world, shape, update_body_mass);
            Some(true)
        }
        OP_SHAPE_SET_DENSITY => {
            let shape = r_shape_id(r);
            let density = r.r_f32();
            let update_body_mass = r.r_bool();
            shape_set_density(world, shape, density, update_body_mass);
            Some(true)
        }
        OP_SHAPE_SET_FRICTION => {
            let shape = r_shape_id(r);
            let friction = r.r_f32();
            shape_set_friction(world, shape, friction);
            Some(true)
        }
        OP_SHAPE_SET_RESTITUTION => {
            let shape = r_shape_id(r);
            let restitution = r.r_f32();
            shape_set_restitution(world, shape, restitution);
            Some(true)
        }
        OP_SHAPE_SET_USER_MATERIAL => {
            let shape = r_shape_id(r);
            let material = r.r_u64();
            shape_set_user_material(world, shape, material);
            Some(true)
        }
        OP_SHAPE_SET_SURFACE_MATERIAL => {
            let shape = r_shape_id(r);
            let material = r_material(r);
            shape_set_surface_material(world, shape, material);
            Some(true)
        }
        OP_SHAPE_SET_FILTER => {
            let shape = r_shape_id(r);
            let filter = r_filter(r);
            shape_set_filter(world, shape, filter);
            Some(true)
        }
        OP_SHAPE_ENABLE_SENSOR_EVENTS => {
            let shape = r_shape_id(r);
            let flag = r.r_bool();
            shape_enable_sensor_events(world, shape, flag);
            Some(true)
        }
        OP_SHAPE_ENABLE_CONTACT_EVENTS => {
            let shape = r_shape_id(r);
            let flag = r.r_bool();
            shape_enable_contact_events(world, shape, flag);
            Some(true)
        }
        OP_SHAPE_ENABLE_PRE_SOLVE_EVENTS => {
            let shape = r_shape_id(r);
            let flag = r.r_bool();
            shape_enable_pre_solve_events(world, shape, flag);
            Some(true)
        }
        OP_SHAPE_ENABLE_HIT_EVENTS => {
            let shape = r_shape_id(r);
            let flag = r.r_bool();
            shape_enable_hit_events(world, shape, flag);
            Some(true)
        }
        OP_SHAPE_SET_CIRCLE => {
            let shape = r_shape_id(r);
            let circle = r_circle(r);
            shape_set_circle(world, shape, &circle);
            Some(true)
        }
        OP_SHAPE_SET_CAPSULE => {
            let shape = r_shape_id(r);
            let capsule = r_capsule(r);
            shape_set_capsule(world, shape, &capsule);
            Some(true)
        }
        OP_SHAPE_SET_SEGMENT => {
            let shape = r_shape_id(r);
            let segment = r_segment(r);
            shape_set_segment(world, shape, &segment);
            Some(true)
        }
        OP_SHAPE_SET_POLYGON => {
            let shape = r_shape_id(r);
            let polygon = r_polygon(r);
            shape_set_polygon(world, shape, &polygon);
            Some(true)
        }
        OP_SHAPE_SET_CHAIN_SEGMENT => {
            let shape = r_shape_id(r);
            let chain_segment = r_chain_segment(r);
            shape_set_chain_segment(world, shape, &chain_segment);
            Some(true)
        }
        OP_SHAPE_APPLY_WIND => {
            let shape = r_shape_id(r);
            let wind = r_vec2(r);
            let drag = r.r_f32();
            let lift = r.r_f32();
            let wake = r.r_bool();
            shape_apply_wind(world, shape, wind, drag, lift, wake);
            Some(true)
        }
        OP_CREATE_CHAIN => {
            let body = BodyId::load(r.r_u64());
            let def = r_chain_def(r);
            let recorded = ChainId::load(r.r_u64());
            let created = create_chain(world, body, &def);
            Some(created.index1 == recorded.index1 && created.generation == recorded.generation)
        }
        OP_DESTROY_CHAIN => {
            let chain = ChainId::load(r.r_u64());
            destroy_chain(world, chain);
            Some(true)
        }
        OP_CHAIN_SET_SURFACE_MATERIAL => {
            let chain = ChainId::load(r.r_u64());
            let material = r_material(r);
            let material_index = r.r_i32();
            chain_set_surface_material(world, chain, material, material_index as usize);
            Some(true)
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use crate::body::create_body;
    use crate::collision::{Capsule, Circle};
    use crate::geometry::{make_box, make_square};
    use crate::math_functions::{to_pos, Vec2};
    use crate::recording::{replay_buffer, world_start_recording, world_stop_recording, Recording};
    use crate::shape::*;
    use crate::types::{
        default_body_def, default_chain_def, default_shape_def, default_surface_material,
        default_world_def, BodyType,
    };
    use crate::world::{world_step, World};

    // The shape/chain family round trips: every shape type created
    // mid-stream, material/filter/geometry mutations, chain create with a
    // material change, and destroys must all re-execute on replay.
    #[test]
    fn shape_ops_replay() {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let bd = default_body_def();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground, &sd, &make_box(20.0, 1.0));

        assert!(world_start_recording(&mut world, Recording::new(0)).is_none());

        let mut shapes = Vec::new();
        let mut chain = None;
        for step in 0..80 {
            match step {
                5 => {
                    // Every dynamic shape type created mid-stream.
                    let mut bd = default_body_def();
                    bd.type_ = BodyType::Dynamic;
                    for (i, x) in [-4.0f32, -2.0, 0.0, 2.0].iter().enumerate() {
                        bd.position = to_pos(Vec2 { x: *x, y: 4.0 });
                        let body = create_body(&mut world, &bd);
                        let shape = match i {
                            0 => create_circle_shape(
                                &mut world,
                                body,
                                &sd,
                                &Circle {
                                    center: Vec2 { x: 0.0, y: 0.0 },
                                    radius: 0.4,
                                },
                            ),
                            1 => create_capsule_shape(
                                &mut world,
                                body,
                                &sd,
                                &Capsule {
                                    center1: Vec2 { x: -0.3, y: 0.0 },
                                    center2: Vec2 { x: 0.3, y: 0.0 },
                                    radius: 0.25,
                                },
                            ),
                            2 => create_polygon_shape(&mut world, body, &sd, &make_square(0.35)),
                            _ => create_polygon_shape(&mut world, body, &sd, &make_box(0.5, 0.2)),
                        };
                        shapes.push(shape);
                    }
                    // Chain floor segment off to the side.
                    let chain_body = create_body(&mut world, &default_body_def());
                    let mut chain_def = default_chain_def();
                    chain_def.points = vec![
                        Vec2 { x: 12.0, y: 3.0 },
                        Vec2 { x: 10.0, y: 2.0 },
                        Vec2 { x: 6.0, y: 2.0 },
                        Vec2 { x: 4.0, y: 3.0 },
                    ];
                    chain = Some(create_chain(&mut world, chain_body, &chain_def));
                }
                20 => {
                    shape_set_friction(&mut world, shapes[0], 0.9);
                    shape_set_restitution(&mut world, shapes[1], 0.6);
                    let mut material = default_surface_material();
                    material.friction = 0.2;
                    material.restitution = 0.4;
                    shape_set_surface_material(&mut world, shapes[2], material);
                    chain_set_surface_material(&mut world, chain.unwrap(), material, 0);
                }
                35 => {
                    // Geometry replacement wakes and re-pairs.
                    shape_set_circle(
                        &mut world,
                        shapes[3],
                        &Circle {
                            center: Vec2 { x: 0.0, y: 0.0 },
                            radius: 0.3,
                        },
                    );
                    let mut filter = shape_get_filter(&world, shapes[0]);
                    filter.group_index = -2;
                    shape_set_filter(&mut world, shapes[0], filter);
                    shape_set_density(&mut world, shapes[1], 2.5, true);
                }
                55 => {
                    destroy_shape(&mut world, shapes[2], true);
                    destroy_chain(&mut world, chain.unwrap());
                }
                _ => {}
            }
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        let recording = world_stop_recording(&mut world).expect("active session");
        let result = replay_buffer(&recording.buffer);
        assert!(result.ok, "stream parses");
        assert!(!result.diverged, "shape ops must re-execute identically");
        assert_eq!(result.steps, 80);
    }
}
