// Body creation and mass update from body.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{body_flags, create_island_for_body, make_body_id, Body, BodySim, BodyState};
use crate::constants::huge;
use crate::core::NULL_INDEX;
use crate::id::BodyId;
use crate::math_functions::{
    is_valid_float, is_valid_position, is_valid_rotation, is_valid_vec2,
    ROT_IDENTITY as ROT_IDENTITY_,
};
use crate::solver_set::{SolverSet, AWAKE_SET, DISABLED_SET, STATIC_SET};
use crate::types::BodyType;
use crate::world::World;

/// Create a rigid body given a definition. (b2CreateBody)
pub fn create_body(world: &mut World, def: &crate::types::BodyDef) -> BodyId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(is_valid_position(def.position));
    debug_assert!(is_valid_rotation(def.rotation));
    debug_assert!(is_valid_vec2(def.linear_velocity));
    debug_assert!(is_valid_float(def.angular_velocity));
    debug_assert!(is_valid_float(def.linear_damping) && def.linear_damping >= 0.0);
    debug_assert!(is_valid_float(def.angular_damping) && def.angular_damping >= 0.0);
    debug_assert!(is_valid_float(def.sleep_threshold) && def.sleep_threshold >= 0.0);
    debug_assert!(is_valid_float(def.gravity_scale));

    debug_assert!(!world.locked);
    if world.locked {
        return BodyId::default();
    }

    let is_awake = (def.is_awake || !def.enable_sleep) && def.is_enabled;

    // determine the solver set
    let set_id;
    if !def.is_enabled {
        // any body type can be disabled
        set_id = DISABLED_SET;
    } else if def.type_ == BodyType::Static {
        set_id = STATIC_SET;
    } else if is_awake {
        set_id = AWAKE_SET;
    } else {
        // new set for a sleeping body in its own island
        set_id = world.solver_set_id_pool.alloc_id();
        if set_id == world.solver_sets.len() as i32 {
            // Create a zero initialized solver set. All sub-arrays are also
            // zero initialized.
            world.solver_sets.push(SolverSet::default());
        } else {
            debug_assert!(world.solver_sets[set_id as usize].set_index == NULL_INDEX);
        }

        world.solver_sets[set_id as usize].set_index = set_id;
    }

    debug_assert!(0 <= set_id && set_id < world.solver_sets.len() as i32);

    let body_id = world.body_id_pool.alloc_id();

    let mut lock_flags = 0u32;
    lock_flags |= if def.motion_locks.linear_x {
        body_flags::LOCK_LINEAR_X
    } else {
        0
    };
    lock_flags |= if def.motion_locks.linear_y {
        body_flags::LOCK_LINEAR_Y
    } else {
        0
    };
    lock_flags |= if def.motion_locks.angular_z {
        body_flags::LOCK_ANGULAR_Z
    } else {
        0
    };

    let set = &mut world.solver_sets[set_id as usize];
    let mut body_sim = BodySim {
        transform: crate::math_functions::WorldTransform {
            p: def.position,
            q: def.rotation,
        },
        center: def.position,
        rotation0: def.rotation,
        center0: def.position,
        min_extent: huge(),
        max_extent: 0.0,
        linear_damping: def.linear_damping,
        angular_damping: def.angular_damping,
        gravity_scale: def.gravity_scale,
        body_id,
        flags: lock_flags,
        ..Default::default()
    };
    body_sim.flags |= if def.is_bullet {
        body_flags::IS_BULLET
    } else {
        0
    };
    body_sim.flags |= if def.allow_fast_rotation {
        body_flags::ALLOW_FAST_ROTATION
    } else {
        0
    };
    body_sim.flags |= if def.type_ == BodyType::Dynamic {
        body_flags::DYNAMIC_FLAG
    } else {
        0
    };
    body_sim.flags |= if def.enable_sleep {
        body_flags::ENABLE_SLEEP
    } else {
        0
    };
    body_sim.flags |= if def.enable_contact_recycling {
        body_flags::BODY_ENABLE_CONTACT_RECYCLING
    } else {
        0
    };
    let sim_flags = body_sim.flags;
    set.body_sims.push(body_sim);
    let local_index = set.body_sims.len() as i32 - 1;

    if set_id == AWAKE_SET {
        set.body_states.push(BodyState {
            linear_velocity: def.linear_velocity,
            angular_velocity: def.angular_velocity,
            delta_rotation: ROT_IDENTITY_,
            flags: sim_flags,
            ..Default::default()
        });
    }

    if body_id == world.bodies.len() as i32 {
        world.bodies.push(Body::default());
    } else {
        debug_assert!(world.bodies[body_id as usize].id == NULL_INDEX);
    }

    let body = &mut world.bodies[body_id as usize];

    // C: strncpy into char[B2_NAME_LENGTH + 1]; the Rust name is truncated to
    // the same limit.
    body.name = def
        .name
        .chars()
        .take(crate::constants::NAME_LENGTH as usize)
        .collect();

    body.user_data = def.user_data;
    body.set_index = set_id;
    body.local_index = local_index;
    body.generation += 1;
    body.head_shape_id = NULL_INDEX;
    body.shape_count = 0;
    body.head_chain_id = NULL_INDEX;
    body.head_contact_key = NULL_INDEX;
    body.contact_count = 0;
    body.head_joint_key = NULL_INDEX;
    body.joint_count = 0;
    body.island_id = NULL_INDEX;
    body.island_index = NULL_INDEX;
    body.body_move_index = NULL_INDEX;
    body.id = body_id;
    body.mass = 0.0;
    body.inertia = 0.0;
    body.sleep_threshold = def.sleep_threshold;
    body.sleep_time = 0.0;
    body.type_ = def.type_;
    body.flags = sim_flags;

    // dynamic and kinematic bodies that are enabled need an island
    if set_id >= AWAKE_SET {
        create_island_for_body(world, set_id, body_id);
    }

    world.validate_solver_sets();

    let id = make_body_id(world, body_id);

    // (B2_REC_CREATE(world, CreateBody, id, worldId, *def))
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_body(rec, def, id)
    });

    id
}

/// Update a body's mass, center of mass, rotational inertia, and extents from
/// its shapes. (b2UpdateBodyMassData)
pub fn update_body_mass_data(world: &mut World, body_id: i32) {
    use crate::collision::MassData;
    use crate::math_functions::{
        add, cross_sv, dot, max_float, min_float, mul_add, mul_sv, sub, sub_pos,
        transform_world_point, VEC2_ZERO,
    };
    use crate::shape::{compute_shape_extent, compute_shape_mass};

    let (set_index, local_index, body_type, head_shape_id, shape_count) = {
        let body = &mut world.bodies[body_id as usize];
        // Mass is no longer dirty
        body.flags &= !body_flags::DIRTY_MASS;
        // Compute mass data from shapes. Each shape has its own density.
        body.mass = 0.0;
        body.inertia = 0.0;
        (
            body.set_index,
            body.local_index,
            body.type_,
            body.head_shape_id,
            body.shape_count,
        )
    };

    {
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        sim.inv_mass = 0.0;
        sim.inv_inertia = 0.0;
        sim.local_center = VEC2_ZERO;
        sim.min_extent = huge();
        sim.max_extent = 0.0;
    }

    // Static and kinematic sims have zero mass.
    if body_type != BodyType::Dynamic {
        {
            let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
            sim.center = sim.transform.p;
            sim.center0 = sim.center;
        }

        // Need extents for kinematic bodies for sleeping to work correctly.
        if body_type == BodyType::Kinematic {
            let mut shape_id = head_shape_id;
            while shape_id != NULL_INDEX {
                let extent = {
                    let s = &world.shapes[shape_id as usize];
                    let e = compute_shape_extent(s, VEC2_ZERO);
                    shape_id = s.next_shape_id;
                    e
                };
                let sim =
                    &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
                sim.min_extent = min_float(sim.min_extent, extent.min_extent);
                sim.max_extent = max_float(sim.max_extent, extent.max_extent);
            }
        }

        return;
    }

    // C uses arena scratch (b2StackAlloc); a Vec is the Rust equivalent.
    let mut masses: Vec<MassData> = Vec::with_capacity(shape_count as usize);

    // Accumulate mass over all shapes.
    let mut local_center = VEC2_ZERO;
    let mut shape_id = head_shape_id;
    while shape_id != NULL_INDEX {
        let (mass_data, next) = {
            let s = &world.shapes[shape_id as usize];
            let next = s.next_shape_id;
            if s.density == 0.0 {
                (MassData::default(), next)
            } else {
                (compute_shape_mass(s), next)
            }
        };
        shape_id = next;

        if mass_data.mass != 0.0 {
            world.bodies[body_id as usize].mass += mass_data.mass;
            local_center = mul_add(local_center, mass_data.mass, mass_data.center);
        }
        masses.push(mass_data);
    }

    // Compute center of mass.
    let body_mass = world.bodies[body_id as usize].mass;
    if body_mass > 0.0 {
        let inv_mass = 1.0 / body_mass;
        world.solver_sets[set_index as usize].body_sims[local_index as usize].inv_mass = inv_mass;
        local_center = mul_sv(inv_mass, local_center);
    }

    // Second loop to accumulate the rotational inertia about the center of mass
    for mass_data in &masses {
        if mass_data.mass == 0.0 {
            continue;
        }

        // Shift to center of mass. This is safe because it can only increase.
        let offset = sub(local_center, mass_data.center);
        let inertia = mass_data.rotational_inertia + mass_data.mass * dot(offset, offset);
        world.bodies[body_id as usize].inertia += inertia;
    }

    debug_assert!(world.bodies[body_id as usize].inertia >= 0.0);

    let inertia = world.bodies[body_id as usize].inertia;
    if inertia > 0.0 {
        world.solver_sets[set_index as usize].body_sims[local_index as usize].inv_inertia =
            1.0 / inertia;
    } else {
        world.bodies[body_id as usize].inertia = 0.0;
        world.solver_sets[set_index as usize].body_sims[local_index as usize].inv_inertia = 0.0;
    }

    // Move center of mass.
    let old_center = {
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        let old = sim.center;
        sim.local_center = local_center;
        sim.center = transform_world_point(sim.transform, sim.local_center);
        sim.center0 = sim.center;
        old
    };

    // Update center of mass velocity
    if set_index == AWAKE_SET {
        let new_center =
            world.solver_sets[set_index as usize].body_sims[local_index as usize].center;
        let state = &mut world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
        let delta_linear = cross_sv(state.angular_velocity, sub_pos(new_center, old_center));
        state.linear_velocity = add(state.linear_velocity, delta_linear);
    }

    // Compute body extents relative to center of mass
    let mut shape_id = head_shape_id;
    while shape_id != NULL_INDEX {
        let extent = {
            let s = &world.shapes[shape_id as usize];
            let e = compute_shape_extent(s, local_center);
            shape_id = s.next_shape_id;
            e
        };
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        sim.min_extent = min_float(sim.min_extent, extent.min_extent);
        sim.max_extent = max_float(sim.max_extent, extent.max_extent);
    }
}

/// Destroy the attached contacts. (static b2DestroyBodyContacts)
pub(crate) fn destroy_body_contacts(world: &mut World, body_id: i32, wake_bodies: bool) {
    let mut edge_key = world.bodies[body_id as usize].head_contact_key;
    while edge_key != NULL_INDEX {
        let contact_id = edge_key >> 1;
        let edge_index = edge_key & 1;

        edge_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;
        crate::contact::destroy_contact(world, contact_id, wake_bodies);
    }

    world.validate_solver_sets();
}

/// Destroy a rigid body and everything attached to it: joints, contacts,
/// shapes, and chains. (b2DestroyBody)
pub fn destroy_body(world: &mut World, body_id: BodyId) {
    use super::{remove_body_from_island, remove_body_sim};
    use crate::body::get_body_full_id;
    use crate::solver_set::FIRST_SLEEPING_SET;

    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_marker(rec, crate::recording::OP_DESTROY_BODY, body_id)
    });

    let body_index = get_body_full_id(world, body_id);

    // Wake bodies attached to this body, even if this body is static.
    let wake_bodies = true;

    // Destroy the attached joints
    let mut edge_key = world.bodies[body_index as usize].head_joint_key;
    while edge_key != NULL_INDEX {
        let joint_id = edge_key >> 1;
        let edge_index = edge_key & 1;

        edge_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        // Careful because this modifies the list being traversed
        crate::joint::destroy_joint_internal(world, joint_id, wake_bodies);
    }

    // Destroy all contacts attached to this body.
    destroy_body_contacts(world, body_index, wake_bodies);

    // Destroy the attached shapes and their broad-phase proxies.
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        if world.shapes[shape_id as usize].sensor_index != NULL_INDEX {
            crate::sensor::destroy_sensor(world, shape_id);
        }

        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            crate::shape::destroy_shape_proxy(&mut shapes[shape_id as usize], broad_phase);
        }

        // Return shape to free list.
        world.shape_id_pool.free_id(shape_id);
        world.shapes[shape_id as usize].id = NULL_INDEX;

        shape_id = world.shapes[shape_id as usize].next_shape_id;
    }

    // Destroy the attached chains. The associated shapes have already been
    // destroyed above.
    let mut chain_id = world.bodies[body_index as usize].head_chain_id;
    while chain_id != NULL_INDEX {
        // Free the chain data. (b2FreeChainData)
        {
            let chain = &mut world.chain_shapes[chain_id as usize];
            chain.shape_indices = Vec::new();
            chain.materials = Vec::new();
        }

        // Return chain to free list.
        world.chain_id_pool.free_id(chain_id);
        world.chain_shapes[chain_id as usize].id = NULL_INDEX;

        chain_id = world.chain_shapes[chain_id as usize].next_chain_id;
    }

    remove_body_from_island(world, body_index);

    // Remove body sim from solver set that owns it
    let (set_index, local_index) = {
        let body = &world.bodies[body_index as usize];
        (body.set_index, body.local_index)
    };
    remove_body_sim(
        &mut world.solver_sets[set_index as usize].body_sims,
        &mut world.bodies,
        local_index,
    );

    // Remove body state from awake set
    if set_index == AWAKE_SET {
        world.solver_sets[set_index as usize]
            .body_states
            .swap_remove(local_index as usize);
    } else if set_index >= FIRST_SLEEPING_SET
        && world.solver_sets[set_index as usize].body_sims.is_empty()
    {
        // Remove solver set if it is empty
        crate::solver_set::destroy_solver_set(world, set_index);
    }

    // Free body and id (preserve body generation)
    let raw_id = world.bodies[body_index as usize].id;
    world.body_id_pool.free_id(raw_id);

    let body = &mut world.bodies[body_index as usize];
    body.set_index = NULL_INDEX;
    body.local_index = NULL_INDEX;
    body.id = NULL_INDEX;

    world.validate_solver_sets();
}
