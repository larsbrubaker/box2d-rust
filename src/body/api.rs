// Body public API from body.c, part 1: validity, transforms, velocities,
// forces/impulses, and mass properties (b2Body_*).
//
// The C resolves the world from the id via the global registry (b2GetWorld /
// b2GetWorldLocked); the Rust port takes `world` explicitly and the locked
// guard becomes a debug assert + early return. B2_REC recording is not
// ported.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_body_full_id, get_body_transform_quick, wake_body, BodySim, BodyState};
use crate::body::body_flags;
use crate::collision::MassData;
use crate::constants::speculative_distance;
use crate::core::NULL_INDEX;
use crate::geometry::compute_fat_shape_aabb;
use crate::id::BodyId;
use crate::math_functions::{
    aabb_contains, aabb_union, abs_float, add, cross_sv, inv_rotate_vector,
    inv_transform_world_point, is_valid_float, is_valid_position, is_valid_rotation, is_valid_vec2,
    length, length_squared, mul_sv, relative_angle, rotate_vector, round_down_float,
    round_up_float, sub, sub_pos, transform_world_point, Aabb, Pos, Rot, Vec2, WorldTransform,
    VEC2_ZERO,
};
use crate::solver_set::{AWAKE_SET, DISABLED_SET};
use crate::types::BodyType;
use crate::world::World;

/// Body identifier validation. Can be used to detect orphaned ids. Provides
/// validation for up to 64K allocations. (b2Body_IsValid — the world registry
/// checks collapse to the world argument)
pub fn body_is_valid(world: &World, id: BodyId) -> bool {
    if id.index1 < 1 || (world.bodies.len() as i32) < id.index1 {
        // invalid index
        return false;
    }

    let body = &world.bodies[(id.index1 - 1) as usize];
    if body.set_index == NULL_INDEX {
        // this was freed
        return false;
    }

    debug_assert!(body.local_index != NULL_INDEX);

    if body.generation != id.generation {
        // this id is orphaned
        return false;
    }

    true
}

/// (b2Body_GetPosition)
pub fn body_get_position(world: &World, body_id: BodyId) -> Pos {
    let body_index = get_body_full_id(world, body_id);
    get_body_transform_quick(world, &world.bodies[body_index as usize]).p
}

/// (b2Body_GetRotation)
pub fn body_get_rotation(world: &World, body_id: BodyId) -> Rot {
    let body_index = get_body_full_id(world, body_id);
    get_body_transform_quick(world, &world.bodies[body_index as usize]).q
}

/// (b2Body_GetTransform)
pub fn body_get_transform(world: &World, body_id: BodyId) -> WorldTransform {
    let body_index = get_body_full_id(world, body_id);
    get_body_transform_quick(world, &world.bodies[body_index as usize])
}

/// (b2Body_GetLocalPoint)
pub fn body_get_local_point(world: &World, body_id: BodyId, world_point: Pos) -> Vec2 {
    let transform = body_get_transform(world, body_id);
    inv_transform_world_point(transform, world_point)
}

/// (b2Body_GetWorldPoint)
pub fn body_get_world_point(world: &World, body_id: BodyId, local_point: Vec2) -> Pos {
    let transform = body_get_transform(world, body_id);
    transform_world_point(transform, local_point)
}

/// (b2Body_GetLocalVector)
pub fn body_get_local_vector(world: &World, body_id: BodyId, world_vector: Vec2) -> Vec2 {
    let transform = body_get_transform(world, body_id);
    inv_rotate_vector(transform.q, world_vector)
}

/// (b2Body_GetWorldVector)
pub fn body_get_world_vector(world: &World, body_id: BodyId, local_vector: Vec2) -> Vec2 {
    let transform = body_get_transform(world, body_id);
    rotate_vector(transform.q, local_vector)
}

/// (b2Body_SetTransform)
pub fn body_set_transform(world: &mut World, body_id: BodyId, position: Pos, rotation: Rot) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_set_transform(rec, body_id, position, rotation)
    });
    debug_assert!(is_valid_position(position));
    debug_assert!(is_valid_rotation(rotation));
    debug_assert!(body_is_valid(world, body_id));
    debug_assert!(!world.locked);

    let body_index = get_body_full_id(world, body_id);
    let (set_index, local_index) = {
        let body = &world.bodies[body_index as usize];
        (body.set_index, body.local_index)
    };

    let transform = {
        let body_sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        body_sim.transform.p = position;
        body_sim.transform.q = rotation;
        body_sim.center = transform_world_point(body_sim.transform, body_sim.local_center);

        body_sim.rotation0 = body_sim.transform.q;
        body_sim.center0 = body_sim.center;
        body_sim.transform
    };

    let speculative_distance_ = speculative_distance();

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let (next_shape_id, moved) = {
            let shape = &mut world.shapes[shape_id as usize];
            let aabb = compute_fat_shape_aabb(&shape.geometry, transform, speculative_distance_);
            shape.aabb = aabb;

            let mut moved = None;
            if !aabb_contains(shape.fat_aabb, aabb) {
                let margin = shape.aabb_margin;
                let fat_aabb = Aabb {
                    lower_bound: Vec2 {
                        x: aabb.lower_bound.x - margin,
                        y: aabb.lower_bound.y - margin,
                    },
                    upper_bound: Vec2 {
                        x: aabb.upper_bound.x + margin,
                        y: aabb.upper_bound.y + margin,
                    },
                };
                shape.fat_aabb = fat_aabb;

                // The body could be disabled
                if shape.proxy_key != NULL_INDEX {
                    moved = Some((shape.proxy_key, fat_aabb));
                }
            }
            (shape.next_shape_id, moved)
        };

        if let Some((proxy_key, fat_aabb)) = moved {
            world.broad_phase.move_proxy(proxy_key, fat_aabb);
        }

        shape_id = next_shape_id;
    }
}

/// (b2Body_GetLinearVelocity)
pub fn body_get_linear_velocity(world: &World, body_id: BodyId) -> Vec2 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        return world.solver_sets[AWAKE_SET as usize].body_states[body.local_index as usize]
            .linear_velocity;
    }
    VEC2_ZERO
}

/// (b2Body_GetAngularVelocity)
pub fn body_get_angular_velocity(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        return world.solver_sets[AWAKE_SET as usize].body_states[body.local_index as usize]
            .angular_velocity;
    }
    0.0
}

/// (b2Body_SetLinearVelocity)
pub fn body_set_linear_velocity(world: &mut World, body_id: BodyId, linear_velocity: Vec2) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_vec2(
            rec,
            crate::recording::OP_BODY_SET_LINEAR_VELOCITY,
            body_id,
            linear_velocity,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    if world.bodies[body_index as usize].type_ == BodyType::Static {
        return;
    }

    if length_squared(linear_velocity) > 0.0 {
        wake_body(world, body_index);
    }

    let body = &world.bodies[body_index as usize];
    if body.set_index != AWAKE_SET {
        return;
    }
    let local_index = body.local_index;
    world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].linear_velocity =
        linear_velocity;
}

/// (b2Body_SetAngularVelocity)
pub fn body_set_angular_velocity(world: &mut World, body_id: BodyId, angular_velocity: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_f32(
            rec,
            crate::recording::OP_BODY_SET_ANGULAR_VELOCITY,
            body_id,
            angular_velocity,
        )
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.type_ == BodyType::Static || body.flags & body_flags::LOCK_ANGULAR_Z != 0 {
            return;
        }
    }

    if angular_velocity != 0.0 {
        wake_body(world, body_index);
    }

    let body = &world.bodies[body_index as usize];
    if body.set_index != AWAKE_SET {
        return;
    }
    let local_index = body.local_index;
    world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].angular_velocity =
        angular_velocity;
}

/// (b2Body_SetTargetTransform)
pub fn body_set_target_transform(
    world: &mut World,
    body_id: BodyId,
    target: WorldTransform,
    time_step: f32,
    wake: bool,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_set_target_transform(rec, body_id, target, time_step, wake)
    });
    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.set_index == DISABLED_SET {
            return;
        }

        if body.type_ == BodyType::Static || time_step <= 0.0 {
            return;
        }

        if body.set_index != AWAKE_SET && !wake {
            return;
        }
    }

    let (sim_local_center, sim_center, sim_q, sim_max_extent) = {
        let body = &world.bodies[body_index as usize];
        let sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
        (
            sim.local_center,
            sim.center,
            sim.transform.q,
            sim.max_extent,
        )
    };

    // Compute linear velocity. The center difference is taken in world
    // precision then demoted
    let delta = sub_pos(transform_world_point(target, sim_local_center), sim_center);
    let inv_time_step = 1.0 / time_step;
    let linear_velocity = mul_sv(inv_time_step, delta);

    // Compute angular velocity
    let q1 = sim_q;
    let q2 = target.q;
    let delta_angle = relative_angle(q1, q2);
    let angular_velocity = inv_time_step * delta_angle;

    // Early out if the body is asleep already and the desired movement is
    // small
    if world.bodies[body_index as usize].set_index != AWAKE_SET {
        let max_velocity = length(linear_velocity) + abs_float(angular_velocity) * sim_max_extent;

        // Return if velocity would be sleepy
        if max_velocity < world.bodies[body_index as usize].sleep_threshold {
            return;
        }

        // Must wake for state to exist
        wake_body(world, body_index);
    }

    debug_assert!(world.bodies[body_index as usize].set_index == AWAKE_SET);

    let local_index = world.bodies[body_index as usize].local_index;
    let state = &mut world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
    state.linear_velocity = linear_velocity;
    state.angular_velocity = angular_velocity;
}

/// (b2Body_GetLocalPointVelocity)
pub fn body_get_local_point_velocity(world: &World, body_id: BodyId, local_point: Vec2) -> Vec2 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    if body.set_index != AWAKE_SET {
        return VEC2_ZERO;
    }

    let state: &BodyState =
        &world.solver_sets[AWAKE_SET as usize].body_states[body.local_index as usize];
    let body_sim: &BodySim =
        &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

    let r = rotate_vector(
        body_sim.transform.q,
        sub(local_point, body_sim.local_center),
    );
    add(state.linear_velocity, cross_sv(state.angular_velocity, r))
}

/// (b2Body_GetWorldPointVelocity)
pub fn body_get_world_point_velocity(world: &World, body_id: BodyId, world_point: Pos) -> Vec2 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    if body.set_index != AWAKE_SET {
        return VEC2_ZERO;
    }

    let state: &BodyState =
        &world.solver_sets[AWAKE_SET as usize].body_states[body.local_index as usize];
    let body_sim: &BodySim =
        &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

    let r = sub_pos(world_point, body_sim.center);
    add(state.linear_velocity, cross_sv(state.angular_velocity, r))
}

/// (b2Body_GetType)
pub fn body_get_type(world: &World, body_id: BodyId) -> BodyType {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].type_
}

/// (b2Body_ComputeAABB)
pub fn body_compute_aabb(world: &World, body_id: BodyId) -> Aabb {
    debug_assert!(!world.locked);

    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    if body.head_shape_id == NULL_INDEX {
        // No shapes, bracket the body origin so the box still contains the
        // true position far away
        let transform = get_body_transform_quick(world, body);
        return Aabb {
            lower_bound: Vec2 {
                x: round_down_float(transform.p.x as f64),
                y: round_down_float(transform.p.y as f64),
            },
            upper_bound: Vec2 {
                x: round_up_float(transform.p.x as f64),
                y: round_up_float(transform.p.y as f64),
            },
        };
    }

    let mut shape = &world.shapes[body.head_shape_id as usize];
    let mut aabb = shape.aabb;
    while shape.next_shape_id != NULL_INDEX {
        shape = &world.shapes[shape.next_shape_id as usize];
        aabb = aabb_union(aabb, shape.aabb);
    }

    aabb
}

/// (b2Body_GetMass)
pub fn body_get_mass(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].mass
}

/// (b2Body_GetRotationalInertia)
pub fn body_get_rotational_inertia(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].inertia
}

/// (b2Body_GetLocalCenter)
pub fn body_get_local_center(world: &World, body_id: BodyId) -> Vec2 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].local_center
}

/// (b2Body_GetWorldCenter)
pub fn body_get_world_center(world: &World, body_id: BodyId) -> Pos {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].center
}

/// (b2Body_SetMassData)
pub fn body_set_mass_data(world: &mut World, body_id: BodyId, mass_data: MassData) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_set_mass_data(rec, body_id, mass_data)
    });
    debug_assert!(is_valid_float(mass_data.mass) && mass_data.mass >= 0.0);
    debug_assert!(
        is_valid_float(mass_data.rotational_inertia) && mass_data.rotational_inertia >= 0.0
    );
    debug_assert!(is_valid_vec2(mass_data.center));
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    let (set_index, local_index) = {
        let body = &mut world.bodies[body_index as usize];
        body.mass = mass_data.mass;
        body.inertia = mass_data.rotational_inertia;
        (body.set_index, body.local_index)
    };

    let body_sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
    body_sim.local_center = mass_data.center;

    let center = transform_world_point(body_sim.transform, mass_data.center);
    body_sim.center = center;
    body_sim.center0 = center;

    body_sim.inv_mass = if mass_data.mass > 0.0 {
        1.0 / mass_data.mass
    } else {
        0.0
    };
    body_sim.inv_inertia = if mass_data.rotational_inertia > 0.0 {
        1.0 / mass_data.rotational_inertia
    } else {
        0.0
    };
}

/// (b2Body_GetMassData)
pub fn body_get_mass_data(world: &World, body_id: BodyId) -> MassData {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    let body_sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
    MassData {
        mass: body.mass,
        center: body_sim.local_center,
        rotational_inertia: body.inertia,
    }
}

/// (b2Body_ApplyMassFromShapes)
pub fn body_apply_mass_from_shapes(world: &mut World, body_id: BodyId) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_marker(
            rec,
            crate::recording::OP_BODY_APPLY_MASS_FROM_SHAPES,
            body_id,
        )
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    super::update_body_mass_data(world, body_index);
}

/// (b2Body_SetLinearDamping)
pub fn body_set_linear_damping(world: &mut World, body_id: BodyId, linear_damping: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_f32(
            rec,
            crate::recording::OP_BODY_SET_LINEAR_DAMPING,
            body_id,
            linear_damping,
        )
    });
    debug_assert!(is_valid_float(linear_damping) && linear_damping >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
        .linear_damping = linear_damping;
}

/// (b2Body_GetLinearDamping)
pub fn body_get_linear_damping(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].linear_damping
}

/// (b2Body_SetAngularDamping)
pub fn body_set_angular_damping(world: &mut World, body_id: BodyId, angular_damping: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_f32(
            rec,
            crate::recording::OP_BODY_SET_ANGULAR_DAMPING,
            body_id,
            angular_damping,
        )
    });
    debug_assert!(is_valid_float(angular_damping) && angular_damping >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
        .angular_damping = angular_damping;
}

/// (b2Body_GetAngularDamping)
pub fn body_get_angular_damping(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].angular_damping
}

/// (b2Body_SetGravityScale)
pub fn body_set_gravity_scale(world: &mut World, body_id: BodyId, gravity_scale: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_f32(
            rec,
            crate::recording::OP_BODY_SET_GRAVITY_SCALE,
            body_id,
            gravity_scale,
        )
    });
    debug_assert!(body_is_valid(world, body_id));
    debug_assert!(is_valid_float(gravity_scale));
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].gravity_scale =
        gravity_scale;
}

/// (b2Body_GetGravityScale)
pub fn body_get_gravity_scale(world: &World, body_id: BodyId) -> f32 {
    debug_assert!(body_is_valid(world, body_id));
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].gravity_scale
}

/// (b2Body_SetUserData)
pub fn body_set_user_data(world: &mut World, body_id: BodyId, user_data: u64) {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].user_data = user_data;
}

/// (b2Body_GetUserData)
pub fn body_get_user_data(world: &World, body_id: BodyId) -> u64 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].user_data
}

/// (b2Body_SetName — takes &str; truncation matches the C strncpy to
/// B2_NAME_LENGTH)
pub fn body_set_name(world: &mut World, body_id: BodyId, name: &str) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_body_set_name(rec, body_id, name)
    });
    let body_index = get_body_full_id(world, body_id);
    let body = &mut world.bodies[body_index as usize];
    // C: strncpy into char[B2_NAME_LENGTH + 1]; truncate to the same limit
    // (same as create_body).
    body.name = name
        .chars()
        .take(crate::constants::NAME_LENGTH as usize)
        .collect();
}

/// (b2Body_GetName)
pub fn body_get_name(world: &World, body_id: BodyId) -> &str {
    let body_index = get_body_full_id(world, body_id);
    &world.bodies[body_index as usize].name
}
