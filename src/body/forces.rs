// Body forces and impulses from body.c (b2Body_ApplyForce family). Split
// from api.rs to stay under the file-length limit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::api::body_is_valid;
use super::plumbing::{get_body_full_id, limit_velocity, wake_body};
use crate::id::BodyId;
use crate::math_functions::{add, cross, mul_add, sub_pos, Pos, Vec2, VEC2_ZERO};
use crate::solver_set::{AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET};
use crate::types::BodyType;
use crate::world::World;

/// (b2Body_ApplyForce)
pub fn body_apply_force(world: &mut World, body_id: BodyId, force: Vec2, point: Pos, wake: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_vec2_point_bool(
            rec,
            crate::recording::OP_BODY_APPLY_FORCE,
            body_id,
            force,
            point,
            wake,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ != BodyType::Dynamic || body.set_index == DISABLED_SET {
            return;
        }
    }

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    }

    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        let local_index = body.local_index;
        let body_sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
        body_sim.force = add(body_sim.force, force);
        body_sim.torque += cross(sub_pos(point, body_sim.center), force);
    }
}

/// (b2Body_ApplyForceToCenter)
pub fn body_apply_force_to_center(world: &mut World, body_id: BodyId, force: Vec2, wake: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_vec2_bool(
            rec,
            crate::recording::OP_BODY_APPLY_FORCE_TO_CENTER,
            body_id,
            force,
            wake,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ != BodyType::Dynamic || body.set_index == DISABLED_SET {
            return;
        }
    }

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    }

    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        let local_index = body.local_index;
        let body_sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
        body_sim.force = add(body_sim.force, force);
    }
}

/// (b2Body_ApplyTorque)
pub fn body_apply_torque(world: &mut World, body_id: BodyId, torque: f32, wake: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_f32_bool(
            rec,
            crate::recording::OP_BODY_APPLY_TORQUE,
            body_id,
            torque,
            wake,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ != BodyType::Dynamic || body.set_index == DISABLED_SET {
            return;
        }
    }

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    }

    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        let local_index = body.local_index;
        let body_sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
        body_sim.torque += torque;
    }
}

/// (b2Body_ClearForces)
pub fn body_clear_forces(world: &mut World, body_id: BodyId) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_marker(rec, crate::recording::OP_BODY_CLEAR_FORCES, body_id)
    });
    let body_index = get_body_full_id(world, body_id);
    let (set_index, local_index) = {
        let body = &world.bodies[body_index as usize];
        (body.set_index, body.local_index)
    };
    let body_sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
    body_sim.force = VEC2_ZERO;
    body_sim.torque = 0.0;
}

/// (b2Body_ApplyLinearImpulse)
pub fn body_apply_linear_impulse(
    world: &mut World,
    body_id: BodyId,
    impulse: Vec2,
    point: Pos,
    wake: bool,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_vec2_point_bool(
            rec,
            crate::recording::OP_BODY_APPLY_LINEAR_IMPULSE,
            body_id,
            impulse,
            point,
            wake,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ != BodyType::Dynamic || body.set_index == DISABLED_SET {
            return;
        }
    }

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    }

    let max_linear_speed = world.max_linear_speed;
    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        let local_index = body.local_index as usize;
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        let body_sim = set.body_sims[local_index];
        let state = &mut set.body_states[local_index];
        state.linear_velocity = mul_add(state.linear_velocity, body_sim.inv_mass, impulse);
        state.angular_velocity +=
            body_sim.inv_inertia * cross(sub_pos(point, body_sim.center), impulse);

        limit_velocity(state, max_linear_speed);
    }
}

/// (b2Body_ApplyLinearImpulseToCenter)
pub fn body_apply_linear_impulse_to_center(
    world: &mut World,
    body_id: BodyId,
    impulse: Vec2,
    wake: bool,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_vec2_bool(
            rec,
            crate::recording::OP_BODY_APPLY_LINEAR_IMPULSE_TO_CENTER,
            body_id,
            impulse,
            wake,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ != BodyType::Dynamic || body.set_index == DISABLED_SET {
            return;
        }
    }

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    }

    let max_linear_speed = world.max_linear_speed;
    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        let local_index = body.local_index as usize;
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        let inv_mass = set.body_sims[local_index].inv_mass;
        let state = &mut set.body_states[local_index];
        state.linear_velocity = mul_add(state.linear_velocity, inv_mass, impulse);

        limit_velocity(state, max_linear_speed);
    }
}

/// (b2Body_ApplyAngularImpulse)
pub fn body_apply_angular_impulse(world: &mut World, body_id: BodyId, impulse: f32, wake: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_f32_bool(
            rec,
            crate::recording::OP_BODY_APPLY_ANGULAR_IMPULSE,
            body_id,
            impulse,
            wake,
        )
    });
    debug_assert!(body_is_valid(world, body_id));
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ != BodyType::Dynamic || body.set_index == DISABLED_SET {
            return;
        }
    }

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        // this will not invalidate body index
        wake_body(world, body_index);
    }

    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        let local_index = body.local_index as usize;
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        let inv_inertia = set.body_sims[local_index].inv_inertia;
        set.body_states[local_index].angular_velocity += inv_inertia * impulse;
    }
}
