// Full world snapshot from world_snapshot.c: b2SerializeWorld composed from
// the struct serializers, restore into an existing world, restore into a
// fresh world, and the deep world-state hash.
//
// Restore preserves the live world's host wiring (callbacks, user data,
// worker count) exactly like C: only simulation state rides the image.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// See snapshot_structs.rs: assignment order IS the wire order.
#![allow(clippy::field_reassign_with_default)]

use super::snapshot::{
    des_bitset, des_hashset, des_id_pool, des_tree, ser_bitset, ser_hashset, ser_id_pool, ser_tree,
    SnapHeader, SnapReader,
};
use super::snapshot_joints::{des_joint, des_joint_sim, ser_joint, ser_joint_sim};
use super::snapshot_structs::*;
use super::write::*;
use crate::constants::GRAPH_COLOR_COUNT;
use crate::constraint_graph::GraphColor;
use crate::math_functions::Vec2;
use crate::shape::ChainShape;
use crate::solver_set::SolverSet;
use crate::types::Capacity;
use crate::world::World;

fn ser_solver_set(buf: &mut Vec<u8>, set: &SolverSet) {
    rec_w_i32(buf, set.set_index);
    rec_w_i32(buf, set.body_sims.len() as i32);
    for sim in set.body_sims.iter() {
        ser_body_sim(buf, sim);
    }
    rec_w_i32(buf, set.body_states.len() as i32);
    for state in set.body_states.iter() {
        ser_body_state(buf, state);
    }
    rec_w_i32(buf, set.joint_sims.len() as i32);
    for sim in set.joint_sims.iter() {
        ser_joint_sim(buf, sim);
    }
    rec_w_i32(buf, set.contact_sims.len() as i32);
    for sim in set.contact_sims.iter() {
        ser_contact_sim(buf, sim);
    }
    rec_w_i32(buf, set.island_sims.len() as i32);
    for sim in set.island_sims.iter() {
        ser_island_sim(buf, sim);
    }
}

fn des_solver_set(r: &mut SnapReader) -> SolverSet {
    let mut set = SolverSet::default();
    set.set_index = r.r_i32();
    let n = r.r_i32();
    if !r.check_count(n, 40) {
        return set;
    }
    set.body_sims = (0..n).map(|_| des_body_sim(r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 20) {
        return set;
    }
    set.body_states = (0..n).map(|_| des_body_state(r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 60) {
        return set;
    }
    set.joint_sims = (0..n).map(|_| des_joint_sim(r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 80) {
        return set;
    }
    set.contact_sims = (0..n).map(|_| des_contact_sim(r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 4) {
        return set;
    }
    set.island_sims = (0..n).map(|_| des_island_sim(r)).collect();
    set
}

/// Graph color: bodySet + contactSims + jointSims (the overflow color has no
/// bodySet). (b2SerGraphColor/b2DesGraphColor)
fn ser_graph_color(buf: &mut Vec<u8>, color: &GraphColor, is_overflow: bool) {
    if !is_overflow {
        ser_bitset(buf, &color.body_set);
    }
    rec_w_i32(buf, color.contact_sims.len() as i32);
    for sim in color.contact_sims.iter() {
        ser_contact_sim(buf, sim);
    }
    rec_w_i32(buf, color.joint_sims.len() as i32);
    for sim in color.joint_sims.iter() {
        ser_joint_sim(buf, sim);
    }
}

fn des_graph_color(r: &mut SnapReader, color: &mut GraphColor, is_overflow: bool) {
    if !is_overflow {
        color.body_set = des_bitset(r);
    }
    let n = r.r_i32();
    if !r.check_count(n, 80) {
        return;
    }
    color.contact_sims = (0..n).map(|_| des_contact_sim(r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 60) {
        return;
    }
    color.joint_sims = (0..n).map(|_| des_joint_sim(r)).collect();
}

/// World scalar config: simulation settings only, never host or worker state
/// (callbacks, user data), so an in-place restore preserves the live world's
/// wiring. (b2SerWorldConfig)
fn ser_world_config(buf: &mut Vec<u8>, world: &World) {
    rec_w_vec2(buf, world.gravity);
    rec_w_f32(buf, world.hit_event_threshold);
    rec_w_f32(buf, world.restitution_threshold);
    rec_w_f32(buf, world.max_linear_speed);
    rec_w_f32(buf, world.contact_speed);
    rec_w_f32(buf, world.contact_hertz);
    rec_w_f32(buf, world.contact_damping_ratio);
    rec_w_f32(buf, world.contact_recycle_distance);
    rec_w_u64(buf, world.step_index);
    rec_w_i32(buf, world.split_island_id);
    // Step scaling cached for the force/torque reporting getters
    rec_w_f32(buf, world.inv_h);
    rec_w_f32(buf, world.inv_dt);
    // End-event double-buffer parity, so the first post-restore event query
    // reads the right half
    rec_w_i32(buf, world.end_event_array_index);
    rec_w_i32(buf, world.max_capacity.static_shape_count);
    rec_w_i32(buf, world.max_capacity.dynamic_shape_count);
    rec_w_i32(buf, world.max_capacity.static_body_count);
    rec_w_i32(buf, world.max_capacity.dynamic_body_count);
    rec_w_i32(buf, world.max_capacity.contact_count);
    let mut flags = 0u8;
    flags |= if world.enable_sleep { 0x01 } else { 0 };
    flags |= if world.enable_warm_starting { 0x02 } else { 0 };
    flags |= if world.enable_contact_softening {
        0x04
    } else {
        0
    };
    flags |= if world.enable_continuous { 0x08 } else { 0 };
    flags |= if world.enable_speculative { 0x10 } else { 0 };
    rec_w_u8(buf, flags);
}

fn des_world_config(r: &mut SnapReader, world: &mut World) {
    world.gravity = Vec2 {
        x: r.r_f32(),
        y: r.r_f32(),
    };
    world.hit_event_threshold = r.r_f32();
    world.restitution_threshold = r.r_f32();
    world.max_linear_speed = r.r_f32();
    world.contact_speed = r.r_f32();
    world.contact_hertz = r.r_f32();
    world.contact_damping_ratio = r.r_f32();
    world.contact_recycle_distance = r.r_f32();
    world.step_index = r.r_u64();
    world.split_island_id = r.r_i32();
    world.inv_h = r.r_f32();
    world.inv_dt = r.r_f32();
    world.end_event_array_index = r.r_i32();
    world.max_capacity = Capacity {
        static_shape_count: r.r_i32(),
        dynamic_shape_count: r.r_i32(),
        static_body_count: r.r_i32(),
        dynamic_body_count: r.r_i32(),
        contact_count: r.r_i32(),
    };
    let flags = r.r_u8();
    world.enable_sleep = flags & 0x01 != 0;
    world.enable_warm_starting = flags & 0x02 != 0;
    world.enable_contact_softening = flags & 0x04 != 0;
    world.enable_continuous = flags & 0x08 != 0;
    world.enable_speculative = flags & 0x10 != 0;
}

fn ser_chain(buf: &mut Vec<u8>, chain: &ChainShape) {
    rec_w_i32(buf, chain.id);
    rec_w_i32(buf, chain.body_id);
    rec_w_i32(buf, chain.next_chain_id);
    rec_w_u16(buf, chain.generation);
    rec_w_i32(buf, chain.shape_indices.len() as i32);
    for &index in chain.shape_indices.iter() {
        rec_w_i32(buf, index);
    }
    rec_w_i32(buf, chain.materials.len() as i32);
    for material in chain.materials.iter() {
        rec_w_material(buf, *material);
    }
}

fn des_chain(r: &mut SnapReader) -> ChainShape {
    let mut chain = ChainShape {
        id: crate::core::NULL_INDEX,
        body_id: crate::core::NULL_INDEX,
        next_chain_id: crate::core::NULL_INDEX,
        shape_indices: Vec::new(),
        materials: Vec::new(),
        generation: 0,
    };
    chain.id = r.r_i32();
    chain.body_id = r.r_i32();
    chain.next_chain_id = r.r_i32();
    chain.generation = r.r_u16();
    let n = r.r_i32();
    if !r.check_count(n, 4) {
        return chain;
    }
    chain.shape_indices = (0..n).map(|_| r.r_i32()).collect();
    let n = r.r_i32();
    if !r.check_count(n, 28) {
        return chain;
    }
    chain.materials = (0..n).map(|_| r_material(r)).collect();
    chain
}

/// Serialize the complete simulation state. (b2SerializeWorld)
pub fn serialize_world(world: &World, buf: &mut Vec<u8>) {
    SnapHeader::current().write(buf);

    ser_world_config(buf, world);

    // 7 id pools
    ser_id_pool(buf, &world.body_id_pool);
    ser_id_pool(buf, &world.shape_id_pool);
    ser_id_pool(buf, &world.chain_id_pool);
    ser_id_pool(buf, &world.contact_id_pool);
    ser_id_pool(buf, &world.joint_id_pool);
    ser_id_pool(buf, &world.island_id_pool);
    ser_id_pool(buf, &world.solver_set_id_pool);

    // Solver sets
    rec_w_i32(buf, world.solver_sets.len() as i32);
    for set in world.solver_sets.iter() {
        ser_solver_set(buf, set);
    }

    // Sparse arrays
    rec_w_i32(buf, world.bodies.len() as i32);
    for body in world.bodies.iter() {
        ser_body(buf, body);
    }
    rec_w_i32(buf, world.shapes.len() as i32);
    for shape in world.shapes.iter() {
        ser_shape(buf, shape);
    }
    rec_w_i32(buf, world.contacts.len() as i32);
    for contact in world.contacts.iter() {
        ser_contact(buf, contact);
    }
    rec_w_i32(buf, world.joints.len() as i32);
    for joint in world.joints.iter() {
        ser_joint(buf, joint);
    }

    // Chain shapes
    rec_w_i32(buf, world.chain_shapes.len() as i32);
    for chain in world.chain_shapes.iter() {
        ser_chain(buf, chain);
    }

    // Sensors
    rec_w_i32(buf, world.sensors.len() as i32);
    for sensor in world.sensors.iter() {
        ser_sensor(buf, sensor);
    }

    // Islands
    rec_w_i32(buf, world.islands.len() as i32);
    for island in world.islands.iter() {
        ser_island(buf, island);
    }

    // Broad phase
    for tree in world.broad_phase.trees.iter() {
        ser_tree(buf, tree);
    }
    for moved in world.broad_phase.moved_proxies.iter() {
        ser_bitset(buf, moved);
    }
    rec_w_i32(buf, world.broad_phase.move_array.len() as i32);
    for &proxy_key in world.broad_phase.move_array.iter() {
        rec_w_i32(buf, proxy_key);
    }
    ser_hashset(buf, &world.broad_phase.pair_set);

    // Constraint graph
    for (c, color) in world.constraint_graph.colors.iter().enumerate() {
        ser_graph_color(buf, color, c as i32 == GRAPH_COLOR_COUNT - 1);
    }
}

/// Restore simulation state from a snapshot image into an existing world,
/// preserving host wiring (callbacks, user data). Returns false and leaves
/// the world untouched on an incompatible or corrupt image.
/// (b2World_Restore / b2DeserializeIntoShell)
pub fn world_restore(world: &mut World, image: &[u8]) -> bool {
    debug_assert!(!world.locked);
    if world.locked {
        return false;
    }

    let mut r = SnapReader::new(image);
    let header = r.r_header();
    if !r.ok || !header.is_compatible() {
        return false;
    }

    // Deserialize into a scratch world first so a truncated image cannot
    // leave the live world half-overwritten.
    let mut fresh = World::new(&crate::types::default_world_def());
    des_world_config(&mut r, &mut fresh);

    fresh.body_id_pool = des_id_pool(&mut r);
    fresh.shape_id_pool = des_id_pool(&mut r);
    fresh.chain_id_pool = des_id_pool(&mut r);
    fresh.contact_id_pool = des_id_pool(&mut r);
    fresh.joint_id_pool = des_id_pool(&mut r);
    fresh.island_id_pool = des_id_pool(&mut r);
    fresh.solver_set_id_pool = des_id_pool(&mut r);

    let n = r.r_i32();
    if !r.check_count(n, 4) {
        return false;
    }
    fresh.solver_sets = (0..n).map(|_| des_solver_set(&mut r)).collect();

    let n = r.r_i32();
    if !r.check_count(n, 60) {
        return false;
    }
    fresh.bodies = (0..n).map(|_| des_body(&mut r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 80) {
        return false;
    }
    fresh.shapes = (0..n).map(|_| des_shape(&mut r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 60) {
        return false;
    }
    fresh.contacts = (0..n).map(|_| des_contact(&mut r)).collect();
    let n = r.r_i32();
    if !r.check_count(n, 40) {
        return false;
    }
    fresh.joints = (0..n).map(|_| des_joint(&mut r)).collect();

    let n = r.r_i32();
    if !r.check_count(n, 20) {
        return false;
    }
    fresh.chain_shapes = (0..n).map(|_| des_chain(&mut r)).collect();

    let n = r.r_i32();
    if !r.check_count(n, 16) {
        return false;
    }
    fresh.sensors = (0..n).map(|_| des_sensor(&mut r)).collect();

    let n = r.r_i32();
    if !r.check_count(n, 16) {
        return false;
    }
    fresh.islands = (0..n).map(|_| des_island(&mut r)).collect();

    for t in 0..fresh.broad_phase.trees.len() {
        fresh.broad_phase.trees[t] = des_tree(&mut r);
    }
    for t in 0..fresh.broad_phase.moved_proxies.len() {
        fresh.broad_phase.moved_proxies[t] = des_bitset(&mut r);
    }
    let n = r.r_i32();
    if !r.check_count(n, 4) {
        return false;
    }
    fresh.broad_phase.move_array = (0..n).map(|_| r.r_i32()).collect();
    fresh.broad_phase.pair_set = des_hashset(&mut r);

    for c in 0..GRAPH_COLOR_COUNT as usize {
        let is_overflow = c as i32 == GRAPH_COLOR_COUNT - 1;
        let mut color = std::mem::take(&mut fresh.constraint_graph.colors[c]);
        des_graph_color(&mut r, &mut color, is_overflow);
        fresh.constraint_graph.colors[c] = color;
    }

    if !r.ok {
        return false;
    }

    // Commit: move simulation state into the live world; host wiring
    // (callbacks, user data, world_id, generation) stays put. Event buffers
    // start empty, matching C's cleared shell.
    world.gravity = fresh.gravity;
    world.hit_event_threshold = fresh.hit_event_threshold;
    world.restitution_threshold = fresh.restitution_threshold;
    world.max_linear_speed = fresh.max_linear_speed;
    world.contact_speed = fresh.contact_speed;
    world.contact_hertz = fresh.contact_hertz;
    world.contact_damping_ratio = fresh.contact_damping_ratio;
    world.contact_recycle_distance = fresh.contact_recycle_distance;
    world.step_index = fresh.step_index;
    world.split_island_id = fresh.split_island_id;
    world.inv_h = fresh.inv_h;
    world.inv_dt = fresh.inv_dt;
    world.end_event_array_index = fresh.end_event_array_index;
    world.max_capacity = fresh.max_capacity;
    world.enable_sleep = fresh.enable_sleep;
    world.enable_warm_starting = fresh.enable_warm_starting;
    world.enable_contact_softening = fresh.enable_contact_softening;
    world.enable_continuous = fresh.enable_continuous;
    world.enable_speculative = fresh.enable_speculative;

    world.body_id_pool = fresh.body_id_pool;
    world.shape_id_pool = fresh.shape_id_pool;
    world.chain_id_pool = fresh.chain_id_pool;
    world.contact_id_pool = fresh.contact_id_pool;
    world.joint_id_pool = fresh.joint_id_pool;
    world.island_id_pool = fresh.island_id_pool;
    world.solver_set_id_pool = fresh.solver_set_id_pool;
    world.solver_sets = fresh.solver_sets;
    world.bodies = fresh.bodies;
    world.shapes = fresh.shapes;
    world.contacts = fresh.contacts;
    world.joints = fresh.joints;
    world.chain_shapes = fresh.chain_shapes;
    world.sensors = fresh.sensors;
    world.islands = fresh.islands;
    world.broad_phase = fresh.broad_phase;
    world.constraint_graph = fresh.constraint_graph;

    world.body_move_events.clear();
    world.sensor_begin_events.clear();
    world.contact_begin_events.clear();
    world.sensor_end_events[0].clear();
    world.sensor_end_events[1].clear();
    world.contact_end_events[0].clear();
    world.contact_end_events[1].clear();
    world.contact_hit_events.clear();
    world.joint_events.clear();

    world.validate_solver_sets();
    true
}

/// Serialize the world into a fresh image. (b2World_Snapshot — returns the
/// image instead of filling a caller buffer)
pub fn world_snapshot(world: &World) -> Vec<u8> {
    debug_assert!(!world.locked);
    let mut buf = Vec::new();
    serialize_world(world, &mut buf);
    buf
}

/// Create a new world from a snapshot image. Returns None on an incompatible
/// or corrupt image. (b2CreateWorldFromSnapshot)
pub fn create_world_from_snapshot(image: &[u8]) -> Option<World> {
    let mut world = World::new(&crate::types::default_world_def());
    if world_restore(&mut world, image) {
        Some(world)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::create_body;
    use crate::geometry::{make_box, make_square};
    use crate::math_functions::to_pos;
    use crate::recording::hash_world_state;
    use crate::shape::create_polygon_shape;
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::world_step;

    fn build_scene() -> World {
        let world_def = default_world_def();
        let mut world = World::new(&world_def);

        let body_def = default_body_def();
        let ground = create_body(&mut world, &body_def);
        let shape_def = default_shape_def();
        create_polygon_shape(&mut world, ground, &shape_def, &make_box(20.0, 1.0));

        for i in 0..12 {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = to_pos(Vec2 {
                x: -3.0 + 0.55 * i as f32,
                y: 2.0 + 0.3 * i as f32,
            });
            let body = create_body(&mut world, &body_def);
            create_polygon_shape(&mut world, body, &shape_def, &make_square(0.25));
        }
        world
    }

    // A snapshot taken mid-simulation restores into a fresh world that then
    // evolves bit-identically to the original — the core b2World_Snapshot/
    // Restore contract from test_snapshot.c.
    #[test]
    fn snapshot_round_trip_is_deterministic() {
        let mut world = build_scene();
        for _ in 0..30 {
            world_step(&mut world, 1.0 / 60.0, 4);
        }

        let image = world_snapshot(&world);
        assert!(image.len() > SnapHeader::SIZE);

        let mut restored = create_world_from_snapshot(&image).expect("image must restore");
        assert_eq!(hash_world_state(&restored), hash_world_state(&world));
        restored.validate_solver_sets();
        restored.validate_contacts();
        restored.validate_connectivity();

        // Both worlds must evolve identically, hash-checked every step while
        // the pile is still moving and after it sleeps.
        for step in 0..90 {
            world_step(&mut world, 1.0 / 60.0, 4);
            world_step(&mut restored, 1.0 / 60.0, 4);
            assert_eq!(
                hash_world_state(&restored),
                hash_world_state(&world),
                "diverged at step {step}"
            );
        }

        // Restore over a populated world also works (b2World_Restore).
        let mut other = build_scene();
        assert!(world_restore(&mut other, &image));

        // Corrupt and truncated images are refused without touching the world.
        assert!(create_world_from_snapshot(&image[..40]).is_none());
        let mut bad = image.clone();
        bad[4] ^= 0xFF; // version
        assert!(create_world_from_snapshot(&bad).is_none());
    }
}
