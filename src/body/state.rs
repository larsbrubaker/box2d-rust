// Body public API from body.c, part 2: body type changes, sleep control,
// enable/disable, motion locks, per-body flags, and shape/joint/contact
// accessors (b2Body_*).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{
    body_flags, create_island_for_body, destroy_body_contacts, get_body_full_id,
    get_body_transform_quick, remove_body_from_island, sync_body_flags, update_body_mass_data,
    wake_body,
};
use crate::core::NULL_INDEX;
use crate::id::BodyId;
use crate::island::{link_joint, split_island, unlink_joint, validate_island};
use crate::shape::{create_shape_proxy, destroy_shape_proxy};
use crate::solver_set::{
    transfer_body, transfer_joint, try_sleep_island, AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET,
    STATIC_SET,
};
use crate::types::{BodyType, MotionLocks};
use crate::world::World;

/// This should follow similar steps as you would get destroying and
/// recreating the body, shapes, and joints. Contacts are difficult to
/// preserve because the broad-phase pairs change, so they are destroyed.
/// (b2Body_SetType — see the C comment block for the staged plan)
pub fn body_set_type(world: &mut World, body_id: BodyId, body_type: BodyType) {
    let body_index = get_body_full_id(world, body_id);

    let original_type = world.bodies[body_index as usize].type_;
    if original_type == body_type {
        return;
    }

    // Stage 1: skip disabled bodies
    if world.bodies[body_index as usize].set_index == DISABLED_SET {
        // Disabled bodies don't change solver sets or islands when they
        // change type.
        {
            let body = &mut world.bodies[body_index as usize];
            body.type_ = body_type;
            if body_type == BodyType::Dynamic {
                body.flags |= body_flags::DYNAMIC_FLAG;
            } else {
                body.flags &= !body_flags::DYNAMIC_FLAG;
            }
        }

        sync_body_flags(world, body_index);

        // Body type affects the mass properties
        update_body_mass_data(world, body_index);
        return;
    }

    // Stage 2: destroy all contacts but don't wake bodies (because we don't
    // need to)
    let wake_bodies = false;
    destroy_body_contacts(world, body_index, wake_bodies);

    // Stage 3: wake this body (does nothing if body is static), otherwise it
    // will also wake all bodies in the same sleeping solver set.
    wake_body(world, body_index);

    // Stage 4: move joints to temporary storage
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        // Joint may be disabled by other body
        if world.joints[joint_id as usize].set_index == DISABLED_SET {
            continue;
        }

        // Wake attached bodies. The wake_body call above does not wake bodies
        // attached to a static body. But it is necessary because the body may
        // have no joints.
        let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_id_b = world.joints[joint_id as usize].edges[1].body_id;
        wake_body(world, body_id_a);
        wake_body(world, body_id_b);

        // Remove joint from island
        unlink_joint(world, joint_id);

        // It is necessary to transfer all joints to the static set so they
        // can be added to the constraint graph below and acquire consistent
        // colors.
        let joint_source_set = world.joints[joint_id as usize].set_index;
        transfer_joint(world, STATIC_SET, joint_source_set, joint_id);
    }

    // Stage 5: change the body type and transfer body
    {
        let body = &mut world.bodies[body_index as usize];
        body.type_ = body_type;
        if body_type == BodyType::Dynamic {
            body.flags |= body_flags::DYNAMIC_FLAG;
        } else {
            body.flags &= !body_flags::DYNAMIC_FLAG;
        }
    }

    let source_set = world.bodies[body_index as usize].set_index;
    let target_set = if body_type == BodyType::Static {
        STATIC_SET
    } else {
        AWAKE_SET
    };

    // Transfer body
    transfer_body(world, target_set, source_set, body_index);

    // Stage 6: update island participation for the body
    if original_type == BodyType::Static {
        // Create island for body
        create_island_for_body(world, AWAKE_SET, body_index);
    } else if body_type == BodyType::Static {
        // Remove body from island.
        remove_body_from_island(world, body_index);
    }

    // Stage 7: Transfer joints to the target set
    joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        // Joint may be disabled by other body
        if world.joints[joint_id as usize].set_index == DISABLED_SET {
            continue;
        }

        // All joints were transferred to the static set in an earlier stage
        debug_assert!(world.joints[joint_id as usize].set_index == STATIC_SET);

        let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_id_b = world.joints[joint_id as usize].edges[1].body_id;
        debug_assert!(
            world.bodies[body_id_a as usize].set_index == STATIC_SET
                || world.bodies[body_id_a as usize].set_index == AWAKE_SET
        );
        debug_assert!(
            world.bodies[body_id_b as usize].set_index == STATIC_SET
                || world.bodies[body_id_b as usize].set_index == AWAKE_SET
        );

        if world.bodies[body_id_a as usize].type_ == BodyType::Dynamic
            || world.bodies[body_id_b as usize].type_ == BodyType::Dynamic
        {
            transfer_joint(world, AWAKE_SET, STATIC_SET, joint_id);
        }
    }

    // Recreate shape proxies in broadphase
    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            destroy_shape_proxy(&mut shapes[shape_id as usize], broad_phase);
            let force_pair_creation = true;
            create_shape_proxy(
                &mut shapes[shape_id as usize],
                broad_phase,
                body_type,
                transform,
                force_pair_creation,
            );
        }
        shape_id = next_shape_id;
    }

    // Relink all joints
    joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        let other_edge_index = edge_index ^ 1;
        let other_body_id =
            world.joints[joint_id as usize].edges[other_edge_index as usize].body_id;

        if world.bodies[other_body_id as usize].set_index == DISABLED_SET {
            continue;
        }

        if world.bodies[body_index as usize].type_ != BodyType::Dynamic
            && world.bodies[other_body_id as usize].type_ != BodyType::Dynamic
        {
            continue;
        }

        link_joint(world, joint_id);
    }

    sync_body_flags(world, body_index);

    // Body type affects the mass
    update_body_mass_data(world, body_index);

    world.validate_solver_sets();
    if world.bodies[body_index as usize].island_id != NULL_INDEX {
        validate_island(world, world.bodies[body_index as usize].island_id);
    }
}

/// (b2Body_IsAwake)
pub fn body_is_awake(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].set_index == AWAKE_SET
}

/// (b2Body_SetAwake)
pub fn body_set_awake(world: &mut World, body_id: BodyId, awake: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);

    let set_index = world.bodies[body_index as usize].set_index;
    if awake && set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    } else if !awake && set_index == AWAKE_SET {
        let island_id = world.bodies[body_index as usize].island_id;
        if world.islands[island_id as usize].constraint_remove_count > 0 {
            // Must split the island before sleeping. This is expensive.
            split_island(world, island_id);
        }

        try_sleep_island(world, island_id);
    }
}

/// (b2Body_WakeTouching)
pub fn body_wake_touching(world: &mut World, body_id: BodyId) {
    let body_index = get_body_full_id(world, body_id);

    let mut contact_key = world.bodies[body_index as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        let (shape_id_a, shape_id_b, next_key) = {
            let contact = &world.contacts[contact_id as usize];
            (
                contact.shape_id_a,
                contact.shape_id_b,
                contact.edges[edge_index as usize].next_key,
            )
        };

        let body_id_a = world.shapes[shape_id_a as usize].body_id;
        let body_id_b = world.shapes[shape_id_b as usize].body_id;

        if body_id_a == body_index {
            wake_body(world, body_id_b);
        } else {
            wake_body(world, body_id_a);
        }

        contact_key = next_key;
    }
}

/// (b2Body_IsEnabled)
pub fn body_is_enabled(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].set_index != DISABLED_SET
}

/// (b2Body_IsSleepEnabled)
pub fn body_is_sleep_enabled(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].flags & body_flags::ENABLE_SLEEP == body_flags::ENABLE_SLEEP
}

/// (b2Body_SetSleepThreshold)
pub fn body_set_sleep_threshold(world: &mut World, body_id: BodyId, sleep_threshold: f32) {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].sleep_threshold = sleep_threshold;
}

/// (b2Body_GetSleepThreshold)
pub fn body_get_sleep_threshold(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].sleep_threshold
}

/// (b2Body_EnableSleep)
pub fn body_enable_sleep(world: &mut World, body_id: BodyId, enable_sleep: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);

    let flag = world.bodies[body_index as usize].flags & body_flags::ENABLE_SLEEP
        == body_flags::ENABLE_SLEEP;
    if enable_sleep == flag {
        return;
    }

    {
        let body = &mut world.bodies[body_index as usize];
        body.flags = if enable_sleep {
            body.flags | body_flags::ENABLE_SLEEP
        } else {
            body.flags & !body_flags::ENABLE_SLEEP
        };
    }
    sync_body_flags(world, body_index);

    if !enable_sleep {
        wake_body(world, body_index);
    }
}

/// Disabling a body requires a lot of detailed bookkeeping, but it is a
/// valuable feature. The most challenging aspect is that joints may connect
/// to bodies that are not disabled. (b2Body_Disable)
pub fn body_disable(world: &mut World, body_id: BodyId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].set_index == DISABLED_SET {
        return;
    }

    // Destroy contacts and wake bodies touching this body. This avoid
    // floating bodies. This is necessary even for static bodies.
    let wake_bodies = true;
    destroy_body_contacts(world, body_index, wake_bodies);

    // Unlink joints and transfer them to the disabled set
    let body_set_index = world.bodies[body_index as usize].set_index;
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        // joint may already be disabled by other body
        if world.joints[joint_id as usize].set_index == DISABLED_SET {
            continue;
        }

        debug_assert!(
            world.joints[joint_id as usize].set_index == body_set_index
                || body_set_index == STATIC_SET
        );

        // Remove joint from island
        unlink_joint(world, joint_id);

        // Transfer joint to disabled set
        let joint_set = world.joints[joint_id as usize].set_index;
        transfer_joint(world, DISABLED_SET, joint_set, joint_id);
    }

    // Remove shapes from broad-phase
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            destroy_shape_proxy(&mut shapes[shape_id as usize], broad_phase);
        }
        shape_id = next_shape_id;
    }

    // Disabled bodies are not in an island. If the island becomes empty it
    // will be destroyed.
    remove_body_from_island(world, body_index);

    // Transfer body sim
    let set = world.bodies[body_index as usize].set_index;
    transfer_body(world, DISABLED_SET, set, body_index);

    world.validate_connectivity();
    world.validate_solver_sets();
}

/// (b2Body_Enable)
pub fn body_enable(world: &mut World, body_id: BodyId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].set_index != DISABLED_SET {
        return;
    }

    let body_type = world.bodies[body_index as usize].type_;
    let set_id = if body_type == BodyType::Static {
        STATIC_SET
    } else {
        AWAKE_SET
    };

    transfer_body(world, set_id, DISABLED_SET, body_index);

    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);

    // Add shapes to broad-phase
    let force_pair_creation = true;
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            create_shape_proxy(
                &mut shapes[shape_id as usize],
                broad_phase,
                body_type,
                transform,
                force_pair_creation,
            );
        }
        shape_id = next_shape_id;
    }

    if set_id != STATIC_SET {
        create_island_for_body(world, set_id, body_index);
    }

    // Transfer joints. If the other body is disabled, don't transfer. If the
    // other body is sleeping, wake it.
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        debug_assert!(world.joints[joint_id as usize].set_index == DISABLED_SET);
        debug_assert!(world.joints[joint_id as usize].island_id == NULL_INDEX);

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_id_b = world.joints[joint_id as usize].edges[1].body_id;
        let set_a = world.bodies[body_id_a as usize].set_index;
        let set_b = world.bodies[body_id_b as usize].set_index;

        if set_a == DISABLED_SET || set_b == DISABLED_SET {
            // one body is still disabled
            continue;
        }

        // Transfer joint first
        let joint_set_id = if set_a == STATIC_SET && set_b == STATIC_SET {
            STATIC_SET
        } else if set_a == STATIC_SET {
            set_b
        } else {
            set_a
        };

        transfer_joint(world, joint_set_id, DISABLED_SET, joint_id);

        // Now that the joint is in the correct set, I can link the joint in
        // the island.
        if joint_set_id != STATIC_SET {
            link_joint(world, joint_id);
        }
    }

    world.validate_solver_sets();
}

/// (b2Body_SetMotionLocks)
pub fn body_set_motion_locks(world: &mut World, body_id: BodyId, locks: MotionLocks) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let mut new_flags = 0;
    if locks.linear_x {
        new_flags |= body_flags::LOCK_LINEAR_X;
    }
    if locks.linear_y {
        new_flags |= body_flags::LOCK_LINEAR_Y;
    }
    if locks.angular_z {
        new_flags |= body_flags::LOCK_ANGULAR_Z;
    }

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].flags & body_flags::ALL_LOCKS != new_flags {
        {
            let body = &mut world.bodies[body_index as usize];
            body.flags &= !body_flags::ALL_LOCKS;
            body.flags |= new_flags;
        }

        sync_body_flags(world, body_index);

        let body = &world.bodies[body_index as usize];
        if body.set_index == AWAKE_SET {
            let state =
                &mut world.solver_sets[AWAKE_SET as usize].body_states[body.local_index as usize];
            if locks.linear_x {
                state.linear_velocity.x = 0.0;
            }
            if locks.linear_y {
                state.linear_velocity.y = 0.0;
            }
            if locks.angular_z {
                state.angular_velocity = 0.0;
            }
        }
    }
}

/// (b2Body_GetMotionLocks)
pub fn body_get_motion_locks(world: &World, body_id: BodyId) -> MotionLocks {
    let body_index = get_body_full_id(world, body_id);
    let flags = world.bodies[body_index as usize].flags;
    MotionLocks {
        linear_x: flags & body_flags::LOCK_LINEAR_X != 0,
        linear_y: flags & body_flags::LOCK_LINEAR_Y != 0,
        angular_z: flags & body_flags::LOCK_ANGULAR_Z != 0,
    }
}

/// (b2Body_SetBullet)
pub fn body_set_bullet(world: &mut World, body_id: BodyId, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let new_flag = if flag { body_flags::IS_BULLET } else { 0 };

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].flags & body_flags::IS_BULLET == new_flag {
        return;
    }

    {
        let body = &mut world.bodies[body_index as usize];
        body.flags &= !body_flags::IS_BULLET;
        body.flags |= new_flag;
    }

    sync_body_flags(world, body_index);
}

/// (b2Body_IsBullet)
pub fn body_is_bullet(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    let body_sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
    body_sim.flags & body_flags::IS_BULLET != 0
}

/// (b2Body_EnableContactRecycling)
pub fn body_enable_contact_recycling(world: &mut World, body_id: BodyId, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let new_flag = if flag {
        body_flags::BODY_ENABLE_CONTACT_RECYCLING
    } else {
        0
    };

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING
        == new_flag
    {
        return;
    }

    {
        let body = &mut world.bodies[body_index as usize];
        body.flags &= !body_flags::BODY_ENABLE_CONTACT_RECYCLING;
        body.flags |= new_flag;
    }

    sync_body_flags(world, body_index);
}

/// (b2Body_IsContactRecyclingEnabled)
pub fn body_is_contact_recycling_enabled(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING != 0
}

/// (b2Body_EnableContactEvents)
pub fn body_enable_contact_events(world: &mut World, body_id: BodyId, flag: bool) {
    let body_index = get_body_full_id(world, body_id);
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &mut world.shapes[shape_id as usize];
        shape.enable_contact_events = flag;
        shape_id = shape.next_shape_id;
    }
}

/// (b2Body_EnableHitEvents)
pub fn body_enable_hit_events(world: &mut World, body_id: BodyId, flag: bool) {
    let body_index = get_body_full_id(world, body_id);
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &mut world.shapes[shape_id as usize];
        shape.enable_hit_events = flag;
        shape_id = shape.next_shape_id;
    }
}
