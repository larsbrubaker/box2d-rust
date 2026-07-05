// Joint public API from joint.c (b2Joint_*). The C resolves the world from
// the id via the global registry; the Rust port takes `world` explicitly.
// b2Joint_GetWorld is not ported (there is no world registry).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: exercised by the world API slice and tests.

use super::*;
use crate::body::{make_body_id, wake_body};
use crate::core::NULL_INDEX;
use crate::id::{BodyId, JointId};
use crate::math_functions::{
    abs_float, dot, is_valid_float, is_valid_transform, left_perp, length, relative_angle,
    rotate_vector, sub, to_relative_transform, transform_point, Transform, Vec2,
};
use crate::world::World;

/// Joint id validity. (b2Joint_IsValid — the world-registry check collapses
/// to the index/generation check in the registry-less port)
pub fn joint_is_valid(world: &World, id: JointId) -> bool {
    let joint_index = id.index1 - 1;
    if joint_index < 0 || world.joints.len() as i32 <= joint_index {
        return false;
    }

    let joint = &world.joints[joint_index as usize];
    if joint.joint_id == NULL_INDEX {
        // joint is free
        return false;
    }

    debug_assert!(joint.joint_id == joint_index);

    id.generation == joint.generation
}

/// (b2Joint_GetType)
pub fn joint_get_type(world: &World, joint_id: JointId) -> JointType {
    let joint_index = get_joint_full_id(world, joint_id);
    world.joints[joint_index as usize].type_
}

/// (b2Joint_GetBodyA)
pub fn joint_get_body_a(world: &World, joint_id: JointId) -> BodyId {
    let joint_index = get_joint_full_id(world, joint_id);
    make_body_id(world, world.joints[joint_index as usize].edges[0].body_id)
}

/// (b2Joint_GetBodyB)
pub fn joint_get_body_b(world: &World, joint_id: JointId) -> BodyId {
    let joint_index = get_joint_full_id(world, joint_id);
    make_body_id(world, world.joints[joint_index as usize].edges[1].body_id)
}

/// (b2Joint_SetLocalFrameA)
pub fn joint_set_local_frame_a(world: &mut World, joint_id: JointId, local_frame: Transform) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_xf(
            rec,
            crate::recording::OP_JOINT_SET_LOCAL_FRAME_A,
            joint_id,
            local_frame,
        )
    });
    debug_assert!(is_valid_transform(local_frame));

    let joint_index = get_joint_full_id(world, joint_id);
    let joint_sim = get_joint_sim(world, joint_index);
    joint_sim.local_frame_a = local_frame;
}

/// (b2Joint_GetLocalFrameA)
pub fn joint_get_local_frame_a(world: &World, joint_id: JointId) -> Transform {
    let joint_index = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, joint_index).local_frame_a
}

/// (b2Joint_SetLocalFrameB)
pub fn joint_set_local_frame_b(world: &mut World, joint_id: JointId, local_frame: Transform) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_xf(
            rec,
            crate::recording::OP_JOINT_SET_LOCAL_FRAME_B,
            joint_id,
            local_frame,
        )
    });
    debug_assert!(is_valid_transform(local_frame));

    let joint_index = get_joint_full_id(world, joint_id);
    let joint_sim = get_joint_sim(world, joint_index);
    joint_sim.local_frame_b = local_frame;
}

/// (b2Joint_GetLocalFrameB)
pub fn joint_get_local_frame_b(world: &World, joint_id: JointId) -> Transform {
    let joint_index = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, joint_index).local_frame_b
}

/// (b2Joint_SetCollideConnected)
pub fn joint_set_collide_connected(world: &mut World, joint_id: JointId, should_collide: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_JOINT_SET_COLLIDE_CONNECTED,
            joint_id,
            should_collide,
        )
    });
    let joint_index = get_joint_full_id(world, joint_id);
    if world.joints[joint_index as usize].collide_connected == should_collide {
        return;
    }

    world.joints[joint_index as usize].collide_connected = should_collide;

    let body_id_a = world.joints[joint_index as usize].edges[0].body_id;
    let body_id_b = world.joints[joint_index as usize].edges[1].body_id;

    if should_collide {
        // need to tell the broad-phase to look for new pairs for one of the
        // two bodies. Pick the one with the fewest shapes.
        let shape_count_a = world.bodies[body_id_a as usize].shape_count;
        let shape_count_b = world.bodies[body_id_b as usize].shape_count;

        let mut shape_id = if shape_count_a < shape_count_b {
            world.bodies[body_id_a as usize].head_shape_id
        } else {
            world.bodies[body_id_b as usize].head_shape_id
        };
        while shape_id != NULL_INDEX {
            let proxy_key = world.shapes[shape_id as usize].proxy_key;
            if proxy_key != NULL_INDEX {
                world.broad_phase.buffer_move(proxy_key);
            }

            shape_id = world.shapes[shape_id as usize].next_shape_id;
        }
    } else {
        destroy_contacts_between_bodies(world, body_id_a, body_id_b);
    }
}

/// (b2Joint_GetCollideConnected)
pub fn joint_get_collide_connected(world: &World, joint_id: JointId) -> bool {
    let joint_index = get_joint_full_id(world, joint_id);
    world.joints[joint_index as usize].collide_connected
}

/// (b2Joint_SetUserData)
pub fn joint_set_user_data(world: &mut World, joint_id: JointId, user_data: u64) {
    let joint_index = get_joint_full_id(world, joint_id);
    world.joints[joint_index as usize].user_data = user_data;
}

/// (b2Joint_GetUserData)
pub fn joint_get_user_data(world: &World, joint_id: JointId) -> u64 {
    let joint_index = get_joint_full_id(world, joint_id);
    world.joints[joint_index as usize].user_data
}

/// (b2Joint_WakeBodies)
pub fn joint_wake_bodies(world: &mut World, joint_id: JointId) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_marker(rec, crate::recording::OP_JOINT_WAKE_BODIES, joint_id)
    });
    let joint_index = get_joint_full_id(world, joint_id);
    let body_id_a = world.joints[joint_index as usize].edges[0].body_id;
    let body_id_b = world.joints[joint_index as usize].edges[1].body_id;

    wake_body(world, body_id_a);
    wake_body(world, body_id_b);
}

/// (b2Joint_GetConstraintForce)
pub fn joint_get_constraint_force(world: &World, joint_id: JointId) -> Vec2 {
    let joint_index = get_joint_full_id(world, joint_id);
    get_joint_constraint_force(world, joint_index)
}

/// (b2Joint_GetConstraintTorque)
pub fn joint_get_constraint_torque(world: &World, joint_id: JointId) -> f32 {
    let joint_index = get_joint_full_id(world, joint_id);
    get_joint_constraint_torque(world, joint_index)
}

/// (b2Joint_GetLinearSeparation)
pub fn joint_get_linear_separation(world: &World, joint_id: JointId) -> f32 {
    let joint_index = get_joint_full_id(world, joint_id);
    let base = get_joint_sim_ref(world, joint_index);
    let joint = &world.joints[joint_index as usize];

    // Relative to body A so the difference stays in float precision far from
    // the origin
    let wxf_a = crate::body::get_body_transform(world, joint.edges[0].body_id);
    let xf_a = to_relative_transform(wxf_a, wxf_a.p);
    let xf_b = to_relative_transform(
        crate::body::get_body_transform(world, joint.edges[1].body_id),
        wxf_a.p,
    );

    let p_a = transform_point(xf_a, base.local_frame_a.p);
    let p_b = transform_point(xf_b, base.local_frame_b.p);
    let dp = sub(p_b, p_a);

    match &base.payload {
        JointPayload::Distance(distance_joint) => {
            let length_ = length(dp);
            if distance_joint.enable_spring {
                if distance_joint.enable_limit {
                    if length_ < distance_joint.min_length {
                        return distance_joint.min_length - length_;
                    }

                    if length_ > distance_joint.max_length {
                        return length_ - distance_joint.max_length;
                    }

                    return 0.0;
                }

                return 0.0;
            }

            abs_float(length_ - distance_joint.length)
        }

        JointPayload::Motor(_) => 0.0,

        JointPayload::Filter => 0.0,

        JointPayload::Prismatic(prismatic_joint) => {
            let axis_a = rotate_vector(xf_a.q, Vec2 { x: 1.0, y: 0.0 });
            let perp_a = left_perp(axis_a);
            let perpendicular_separation = abs_float(dot(perp_a, dp));
            let mut limit_separation = 0.0;

            if prismatic_joint.enable_limit {
                let translation = dot(axis_a, dp);
                if translation < prismatic_joint.lower_translation {
                    limit_separation = prismatic_joint.lower_translation - translation;
                }

                if prismatic_joint.upper_translation < translation {
                    limit_separation = translation - prismatic_joint.upper_translation;
                }
            }

            (perpendicular_separation * perpendicular_separation
                + limit_separation * limit_separation)
                .sqrt()
        }

        JointPayload::Revolute(_) => length(dp),

        JointPayload::Weld(weld_joint) => {
            if weld_joint.linear_hertz == 0.0 {
                return length(dp);
            }

            0.0
        }

        JointPayload::Wheel(wheel_joint) => {
            let axis_a = rotate_vector(xf_a.q, Vec2 { x: 1.0, y: 0.0 });
            let perp_a = left_perp(axis_a);
            let perpendicular_separation = abs_float(dot(perp_a, dp));
            let mut limit_separation = 0.0;

            if wheel_joint.enable_limit {
                let translation = dot(axis_a, dp);
                if translation < wheel_joint.lower_translation {
                    limit_separation = wheel_joint.lower_translation - translation;
                }

                if wheel_joint.upper_translation < translation {
                    limit_separation = translation - wheel_joint.upper_translation;
                }
            }

            (perpendicular_separation * perpendicular_separation
                + limit_separation * limit_separation)
                .sqrt()
        }
    }
}

/// (b2Joint_GetAngularSeparation)
pub fn joint_get_angular_separation(world: &World, joint_id: JointId) -> f32 {
    let joint_index = get_joint_full_id(world, joint_id);
    let base = get_joint_sim_ref(world, joint_index);
    let joint = &world.joints[joint_index as usize];

    let q_a = crate::body::get_body_transform(world, joint.edges[0].body_id).q;
    let q_b = crate::body::get_body_transform(world, joint.edges[1].body_id).q;
    let relative_angle_ = relative_angle(q_a, q_b);

    match &base.payload {
        JointPayload::Distance(_) => 0.0,

        JointPayload::Motor(_) => 0.0,

        JointPayload::Filter => 0.0,

        JointPayload::Prismatic(_) => relative_angle_,

        JointPayload::Revolute(revolute_joint) => {
            if revolute_joint.enable_limit {
                let angle = relative_angle_;
                if angle < revolute_joint.lower_angle {
                    return revolute_joint.lower_angle - angle;
                }

                if revolute_joint.upper_angle < angle {
                    return angle - revolute_joint.upper_angle;
                }
            }

            0.0
        }

        JointPayload::Weld(weld_joint) => {
            if weld_joint.angular_hertz == 0.0 {
                return relative_angle_;
            }

            0.0
        }

        JointPayload::Wheel(_) => 0.0,
    }
}

/// (b2Joint_SetConstraintTuning)
pub fn joint_set_constraint_tuning(
    world: &mut World,
    joint_id: JointId,
    hertz: f32,
    damping_ratio: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32_pair(
            rec,
            crate::recording::OP_JOINT_SET_CONSTRAINT_TUNING,
            joint_id,
            hertz,
            damping_ratio,
        )
    });
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);

    let joint_index = get_joint_full_id(world, joint_id);
    let base = get_joint_sim(world, joint_index);
    base.constraint_hertz = hertz;
    base.constraint_damping_ratio = damping_ratio;
}

/// (b2Joint_GetConstraintTuning — C returns through out-pointers)
pub fn joint_get_constraint_tuning(world: &World, joint_id: JointId) -> (f32, f32) {
    let joint_index = get_joint_full_id(world, joint_id);
    let base = get_joint_sim_ref(world, joint_index);
    (base.constraint_hertz, base.constraint_damping_ratio)
}

/// (b2Joint_SetForceThreshold)
pub fn joint_set_force_threshold(world: &mut World, joint_id: JointId, threshold: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_JOINT_SET_FORCE_THRESHOLD,
            joint_id,
            threshold,
        )
    });
    debug_assert!(is_valid_float(threshold) && threshold >= 0.0);

    let joint_index = get_joint_full_id(world, joint_id);
    let base = get_joint_sim(world, joint_index);
    base.force_threshold = threshold;
}

/// (b2Joint_GetForceThreshold)
pub fn joint_get_force_threshold(world: &World, joint_id: JointId) -> f32 {
    let joint_index = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, joint_index).force_threshold
}

/// (b2Joint_SetTorqueThreshold)
pub fn joint_set_torque_threshold(world: &mut World, joint_id: JointId, threshold: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_JOINT_SET_TORQUE_THRESHOLD,
            joint_id,
            threshold,
        )
    });
    debug_assert!(is_valid_float(threshold) && threshold >= 0.0);

    let joint_index = get_joint_full_id(world, joint_id);
    let base = get_joint_sim(world, joint_index);
    base.torque_threshold = threshold;
}

/// (b2Joint_GetTorqueThreshold)
pub fn joint_get_torque_threshold(world: &World, joint_id: JointId) -> f32 {
    let joint_index = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, joint_index).torque_threshold
}
