// Port of motor_joint.c: public accessors, force/torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// Same conventions as distance_joint.rs: the world is passed explicitly,
// B2_REC recording is not ported, sim functions copy body states out and back
// writing only the fields the C guards with b2_dynamicFlag.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: prepare/warm-start/solve are called by the solver slice.
#![allow(dead_code)]

use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::joint::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::math_functions::{
    add, clamp_float, cross, cross_sv, get_inverse_22, inv_mul_rot, length_squared, max_float,
    mul_add, mul_mv, mul_rot, mul_sub, mul_sv, normalize, rot_get_angle, rotate_vector, sub,
    sub_pos, Vec2, MAT22_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b2MotorJoint_SetLinearVelocity)
pub fn motor_joint_set_linear_velocity(world: &mut World, joint_id: JointId, velocity: Vec2) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_vec2(
            rec,
            crate::recording::OP_MOTOR_SET_LINEAR_VELOCITY,
            joint_id,
            velocity,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().linear_velocity = velocity;
}

/// (b2MotorJoint_GetLinearVelocity)
pub fn motor_joint_get_linear_velocity(world: &World, joint_id: JointId) -> Vec2 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().linear_velocity
}

/// (b2MotorJoint_SetAngularVelocity)
pub fn motor_joint_set_angular_velocity(world: &mut World, joint_id: JointId, velocity: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_ANGULAR_VELOCITY,
            joint_id,
            velocity,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().angular_velocity = velocity;
}

/// (b2MotorJoint_GetAngularVelocity)
pub fn motor_joint_get_angular_velocity(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().angular_velocity
}

/// (b2MotorJoint_SetMaxVelocityTorque)
pub fn motor_joint_set_max_velocity_torque(world: &mut World, joint_id: JointId, max_torque: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_MAX_VELOCITY_TORQUE,
            joint_id,
            max_torque,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().max_velocity_torque = max_torque;
}

/// (b2MotorJoint_GetMaxVelocityTorque)
pub fn motor_joint_get_max_velocity_torque(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().max_velocity_torque
}

/// (b2MotorJoint_SetMaxVelocityForce)
pub fn motor_joint_set_max_velocity_force(world: &mut World, joint_id: JointId, max_force: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_MAX_VELOCITY_FORCE,
            joint_id,
            max_force,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().max_velocity_force = max_force;
}

/// (b2MotorJoint_GetMaxVelocityForce)
pub fn motor_joint_get_max_velocity_force(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().max_velocity_force
}

/// (b2MotorJoint_SetLinearHertz)
pub fn motor_joint_set_linear_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_LINEAR_HERTZ,
            joint_id,
            hertz,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().linear_hertz = hertz;
}

/// (b2MotorJoint_GetLinearHertz)
pub fn motor_joint_get_linear_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().linear_hertz
}

/// (b2MotorJoint_SetLinearDampingRatio)
pub fn motor_joint_set_linear_damping_ratio(world: &mut World, joint_id: JointId, damping: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_LINEAR_DAMPING_RATIO,
            joint_id,
            damping,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().linear_damping_ratio = damping;
}

/// (b2MotorJoint_GetLinearDampingRatio)
pub fn motor_joint_get_linear_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().linear_damping_ratio
}

/// (b2MotorJoint_SetAngularHertz)
pub fn motor_joint_set_angular_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_ANGULAR_HERTZ,
            joint_id,
            hertz,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().angular_hertz = hertz;
}

/// (b2MotorJoint_GetAngularHertz)
pub fn motor_joint_get_angular_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().angular_hertz
}

/// (b2MotorJoint_SetAngularDampingRatio)
pub fn motor_joint_set_angular_damping_ratio(world: &mut World, joint_id: JointId, damping: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_ANGULAR_DAMPING_RATIO,
            joint_id,
            damping,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().angular_damping_ratio = damping;
}

/// (b2MotorJoint_GetAngularDampingRatio)
pub fn motor_joint_get_angular_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().angular_damping_ratio
}

/// (b2MotorJoint_SetMaxSpringForce)
pub fn motor_joint_set_max_spring_force(world: &mut World, joint_id: JointId, max_force: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_MAX_SPRING_FORCE,
            joint_id,
            max_force,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().max_spring_force = max_float(0.0, max_force);
}

/// (b2MotorJoint_GetMaxSpringForce)
pub fn motor_joint_get_max_spring_force(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().max_spring_force
}

/// (b2MotorJoint_SetMaxSpringTorque)
pub fn motor_joint_set_max_spring_torque(world: &mut World, joint_id: JointId, max_torque: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_MOTOR_SET_MAX_SPRING_TORQUE,
            joint_id,
            max_torque,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Motor);
    joint.motor_mut().max_spring_torque = max_float(0.0, max_torque);
}

/// (b2MotorJoint_GetMaxSpringTorque)
pub fn motor_joint_get_max_spring_torque(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Motor);
    joint.motor().max_spring_torque
}

/// (b2GetMotorJointForce)
pub fn get_motor_joint_force(world: &World, base: &JointSim) -> Vec2 {
    let motor = base.motor();
    mul_sv(
        world.inv_h,
        add(motor.linear_velocity_impulse, motor.linear_spring_impulse),
    )
}

/// (b2GetMotorJointTorque)
pub fn get_motor_joint_torque(world: &World, base: &JointSim) -> f32 {
    let motor = base.motor();
    world.inv_h * (motor.angular_velocity_impulse + motor.angular_spring_impulse)
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/// (b2PrepareMotorJoint)
pub fn prepare_motor_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.joint_type() == JointType::Motor);

    // chase body id to the solver set where the body lives
    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_a = &world.bodies[id_a as usize];
    let body_b = &world.bodies[id_b as usize];

    debug_assert!(body_a.set_index == AWAKE_SET || body_b.set_index == AWAKE_SET);

    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

    let m_a = body_sim_a.inv_mass;
    let i_a = body_sim_a.inv_inertia;
    let m_b = body_sim_b.inv_mass;
    let i_b = body_sim_b.inv_inertia;

    base.inv_mass_a = m_a;
    base.inv_mass_b = m_b;
    base.inv_i_a = i_a;
    base.inv_i_b = i_b;

    let local_frame_a = base.local_frame_a;
    let local_frame_b = base.local_frame_b;

    let index_a = if body_a.set_index == AWAKE_SET {
        body_a.local_index
    } else {
        NULL_INDEX
    };
    let index_b = if body_b.set_index == AWAKE_SET {
        body_b.local_index
    } else {
        NULL_INDEX
    };

    let joint = base.motor_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;

    // Compute joint anchor frames with world space rotation, relative to
    // center of mass
    joint.frame_a.q = mul_rot(body_sim_a.transform.q, local_frame_a.q);
    joint.frame_a.p = rotate_vector(
        body_sim_a.transform.q,
        sub(local_frame_a.p, body_sim_a.local_center),
    );
    joint.frame_b.q = mul_rot(body_sim_b.transform.q, local_frame_b.q);
    joint.frame_b.p = rotate_vector(
        body_sim_b.transform.q,
        sub(local_frame_b.p, body_sim_b.local_center),
    );

    // Compute the initial center delta. Incremental position updates are
    // relative to this.
    joint.delta_center = sub_pos(body_sim_b.center, body_sim_a.center);

    let r_a = joint.frame_a.p;
    let r_b = joint.frame_b.p;

    joint.linear_spring = make_soft(joint.linear_hertz, joint.linear_damping_ratio, context.h);
    joint.angular_spring = make_soft(joint.angular_hertz, joint.angular_damping_ratio, context.h);

    let mut kl = MAT22_ZERO;
    kl.cx.x = m_a + m_b + r_a.y * r_a.y * i_a + r_b.y * r_b.y * i_b;
    kl.cx.y = -r_a.y * r_a.x * i_a - r_b.y * r_b.x * i_b;
    kl.cy.x = kl.cx.y;
    kl.cy.y = m_a + m_b + r_a.x * r_a.x * i_a + r_b.x * r_b.x * i_b;
    joint.linear_mass = get_inverse_22(kl);

    let ka = i_a + i_b;
    joint.angular_mass = if ka > 0.0 { 1.0 / ka } else { 0.0 };

    if !context.enable_warm_starting {
        joint.linear_velocity_impulse = Vec2 { x: 0.0, y: 0.0 };
        joint.angular_velocity_impulse = 0.0;
        joint.linear_spring_impulse = Vec2 { x: 0.0, y: 0.0 };
        joint.angular_spring_impulse = 0.0;
    }
}

/// (b2WarmStartMotorJoint)
pub fn warm_start_motor_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.joint_type() == JointType::Motor);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.motor_mut();

    // dummy state for static bodies
    let mut state_a = if joint.index_a == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_a as usize]
    };
    let mut state_b = if joint.index_b == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_b as usize]
    };

    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    let linear_impulse = add(joint.linear_velocity_impulse, joint.linear_spring_impulse);
    let angular_impulse = joint.angular_velocity_impulse + joint.angular_spring_impulse;

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, linear_impulse);
        state_a.angular_velocity -= i_a * (cross(r_a, linear_impulse) + angular_impulse);
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, linear_impulse);
        state_b.angular_velocity += i_b * (cross(r_b, linear_impulse) + angular_impulse);
        states[joint.index_b as usize] = state_b;
    }
}

/// (b2SolveMotorJoint)
pub fn solve_motor_joint(base: &mut JointSim, context: &StepContext, states: &mut [BodyState]) {
    debug_assert!(base.joint_type() == JointType::Motor);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.motor_mut();

    // dummy state for static bodies
    let mut state_a = if joint.index_a == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_a as usize]
    };
    let mut state_b = if joint.index_b == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_b as usize]
    };

    let mut v_a = state_a.linear_velocity;
    let mut w_a = state_a.angular_velocity;
    let mut v_b = state_b.linear_velocity;
    let mut w_b = state_b.angular_velocity;

    // angular spring
    if joint.max_spring_torque > 0.0 && joint.angular_hertz > 0.0 {
        let q_a = mul_rot(state_a.delta_rotation, joint.frame_a.q);
        let q_b = mul_rot(state_b.delta_rotation, joint.frame_b.q);
        let rel_q = inv_mul_rot(q_a, q_b);

        let c = rot_get_angle(rel_q);
        let bias = joint.angular_spring.bias_rate * c;
        let mass_scale = joint.angular_spring.mass_scale;
        let impulse_scale = joint.angular_spring.impulse_scale;

        let cdot = w_b - w_a;

        let max_impulse = context.h * joint.max_spring_torque;
        let old_impulse = joint.angular_spring_impulse;
        let mut impulse =
            -mass_scale * joint.angular_mass * (cdot + bias) - impulse_scale * old_impulse;
        joint.angular_spring_impulse =
            clamp_float(old_impulse + impulse, -max_impulse, max_impulse);
        impulse = joint.angular_spring_impulse - old_impulse;

        w_a -= i_a * impulse;
        w_b += i_b * impulse;
    }

    // angular velocity
    if joint.max_velocity_torque > 0.0 {
        let cdot = w_b - w_a - joint.angular_velocity;
        let mut impulse = -joint.angular_mass * cdot;

        let max_impulse = context.h * joint.max_velocity_torque;
        let old_impulse = joint.angular_velocity_impulse;
        joint.angular_velocity_impulse =
            clamp_float(old_impulse + impulse, -max_impulse, max_impulse);
        impulse = joint.angular_velocity_impulse - old_impulse;

        w_a -= i_a * impulse;
        w_b += i_b * impulse;
    }

    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    // linear spring
    if joint.max_spring_force > 0.0 && joint.linear_hertz > 0.0 {
        let dc_a = state_a.delta_position;
        let dc_b = state_b.delta_position;
        let c = add(add(sub(dc_b, dc_a), sub(r_b, r_a)), joint.delta_center);

        let bias = mul_sv(joint.linear_spring.bias_rate, c);
        let mass_scale = joint.linear_spring.mass_scale;
        let impulse_scale = joint.linear_spring.impulse_scale;

        let mut cdot = sub(add(v_b, cross_sv(w_b, r_b)), add(v_a, cross_sv(w_a, r_a)));
        cdot = add(cdot, bias);

        // Updating the effective mass here may be overkill
        let mut kl = MAT22_ZERO;
        kl.cx.x = m_a + m_b + r_a.y * r_a.y * i_a + r_b.y * r_b.y * i_b;
        kl.cx.y = -r_a.y * r_a.x * i_a - r_b.y * r_b.x * i_b;
        kl.cy.x = kl.cx.y;
        kl.cy.y = m_a + m_b + r_a.x * r_a.x * i_a + r_b.x * r_b.x * i_b;
        joint.linear_mass = get_inverse_22(kl);

        let b = mul_mv(joint.linear_mass, cdot);

        let old_impulse = joint.linear_spring_impulse;
        let mut impulse = Vec2 {
            x: -mass_scale * b.x - impulse_scale * old_impulse.x,
            y: -mass_scale * b.y - impulse_scale * old_impulse.y,
        };

        let max_impulse = context.h * joint.max_spring_force;
        joint.linear_spring_impulse = add(joint.linear_spring_impulse, impulse);

        if length_squared(joint.linear_spring_impulse) > max_impulse * max_impulse {
            joint.linear_spring_impulse = normalize(joint.linear_spring_impulse);
            joint.linear_spring_impulse.x *= max_impulse;
            joint.linear_spring_impulse.y *= max_impulse;
        }

        impulse = sub(joint.linear_spring_impulse, old_impulse);

        v_a = mul_sub(v_a, m_a, impulse);
        w_a -= i_a * cross(r_a, impulse);
        v_b = mul_add(v_b, m_b, impulse);
        w_b += i_b * cross(r_b, impulse);
    }

    // linear velocity
    if joint.max_velocity_force > 0.0 {
        let mut cdot = sub(add(v_b, cross_sv(w_b, r_b)), add(v_a, cross_sv(w_a, r_a)));
        cdot = sub(cdot, joint.linear_velocity);
        let b = mul_mv(joint.linear_mass, cdot);
        let mut impulse = Vec2 { x: -b.x, y: -b.y };

        let old_impulse = joint.linear_velocity_impulse;
        let max_impulse = context.h * joint.max_velocity_force;
        joint.linear_velocity_impulse = add(joint.linear_velocity_impulse, impulse);

        if length_squared(joint.linear_velocity_impulse) > max_impulse * max_impulse {
            joint.linear_velocity_impulse = normalize(joint.linear_velocity_impulse);
            joint.linear_velocity_impulse.x *= max_impulse;
            joint.linear_velocity_impulse.y *= max_impulse;
        }

        impulse = sub(joint.linear_velocity_impulse, old_impulse);

        v_a = mul_sub(v_a, m_a, impulse);
        w_a -= i_a * cross(r_a, impulse);
        v_b = mul_add(v_b, m_b, impulse);
        w_b += i_b * cross(r_b, impulse);
    }

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = v_a;
        state_a.angular_velocity = w_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = v_b;
        state_b.angular_velocity = w_b;
        states[joint.index_b as usize] = state_b;
    }
}
