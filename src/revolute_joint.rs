// Port of revolute_joint.c: public accessors, force/torque reporting, and
// the prepare/warm-start/solve simulation functions.
//
// Same conventions as distance_joint.rs: the world is passed explicitly,
// B2_REC recording is not ported, sim functions copy body states out and back
// writing only the fields the C guards with b2_dynamicFlag.
// b2DrawRevoluteJoint lands with the debug-draw phase.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: prepare/warm-start/solve are called by the solver slice.
#![allow(dead_code)]

use crate::body::{body_flags, get_body_transform, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::joint::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::math_functions::{
    add, clamp_float, cross, cross_sv, inv_mul_rot, max_float, min_float, mul_add, mul_rot,
    mul_sub, mul_sv, relative_angle, rot_get_angle, rotate_vector, solve_22, sub, sub_pos,
    unwind_angle, Vec2, MAT22_ZERO, PI, VEC2_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

// Point-to-point constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// J = [-E -skew(rA) E skew(rB) ]
//
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// Motor constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
// K = invIA + invIB

/// (b2RevoluteJoint_EnableSpring)
pub fn revolute_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_REVOLUTE_ENABLE_SPRING,
            joint_id,
            enable_spring,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    let revolute = joint.revolute_mut();
    if enable_spring != revolute.enable_spring {
        revolute.enable_spring = enable_spring;
        revolute.spring_impulse = 0.0;
    }
}

/// (b2RevoluteJoint_IsSpringEnabled)
pub fn revolute_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().enable_spring
}

/// (b2RevoluteJoint_SetSpringHertz)
pub fn revolute_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_REVOLUTE_SET_SPRING_HERTZ,
            joint_id,
            hertz,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    joint.revolute_mut().hertz = hertz;
}

/// (b2RevoluteJoint_GetSpringHertz)
pub fn revolute_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().hertz
}

/// (b2RevoluteJoint_SetSpringDampingRatio)
pub fn revolute_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_REVOLUTE_SET_SPRING_DAMPING_RATIO,
            joint_id,
            damping_ratio,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    joint.revolute_mut().damping_ratio = damping_ratio;
}

/// (b2RevoluteJoint_GetSpringDampingRatio)
pub fn revolute_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().damping_ratio
}

/// (b2RevoluteJoint_SetTargetAngle)
pub fn revolute_joint_set_target_angle(world: &mut World, joint_id: JointId, angle: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_REVOLUTE_SET_TARGET_ANGLE,
            joint_id,
            angle,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    joint.revolute_mut().target_angle = angle;
}

/// (b2RevoluteJoint_GetTargetAngle)
pub fn revolute_joint_get_target_angle(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().target_angle
}

/// (b2RevoluteJoint_GetAngle)
pub fn revolute_joint_get_angle(world: &World, joint_id: JointId) -> f32 {
    let joint_sim = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    let q_a = mul_rot(
        get_body_transform(world, joint_sim.body_id_a).q,
        joint_sim.local_frame_a.q,
    );
    let q_b = mul_rot(
        get_body_transform(world, joint_sim.body_id_b).q,
        joint_sim.local_frame_b.q,
    );

    relative_angle(q_a, q_b)
}

/// (b2RevoluteJoint_EnableLimit)
pub fn revolute_joint_enable_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_REVOLUTE_ENABLE_LIMIT,
            joint_id,
            enable_limit,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    let revolute = joint.revolute_mut();
    if enable_limit != revolute.enable_limit {
        revolute.enable_limit = enable_limit;
        revolute.lower_impulse = 0.0;
        revolute.upper_impulse = 0.0;
    }
}

/// (b2RevoluteJoint_IsLimitEnabled)
pub fn revolute_joint_is_limit_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().enable_limit
}

/// (b2RevoluteJoint_GetLowerLimit)
pub fn revolute_joint_get_lower_limit(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().lower_angle
}

/// (b2RevoluteJoint_GetUpperLimit)
pub fn revolute_joint_get_upper_limit(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().upper_angle
}

/// (b2RevoluteJoint_SetLimits)
pub fn revolute_joint_set_limits(world: &mut World, joint_id: JointId, lower: f32, upper: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32_pair(
            rec,
            crate::recording::OP_REVOLUTE_SET_LIMITS,
            joint_id,
            lower,
            upper,
        )
    });
    debug_assert!(lower <= upper);
    debug_assert!(lower >= -0.99 * PI);
    debug_assert!(upper <= 0.99 * PI);

    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    let revolute = joint.revolute_mut();
    if lower != revolute.lower_angle || upper != revolute.upper_angle {
        revolute.lower_angle = min_float(lower, upper);
        revolute.upper_angle = max_float(lower, upper);
        revolute.lower_impulse = 0.0;
        revolute.upper_impulse = 0.0;
    }
}

/// (b2RevoluteJoint_EnableMotor)
pub fn revolute_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_REVOLUTE_ENABLE_MOTOR,
            joint_id,
            enable_motor,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    let revolute = joint.revolute_mut();
    if enable_motor != revolute.enable_motor {
        revolute.enable_motor = enable_motor;
        revolute.motor_impulse = 0.0;
    }
}

/// (b2RevoluteJoint_IsMotorEnabled)
pub fn revolute_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().enable_motor
}

/// (b2RevoluteJoint_SetMotorSpeed)
pub fn revolute_joint_set_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_REVOLUTE_SET_MOTOR_SPEED,
            joint_id,
            motor_speed,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    joint.revolute_mut().motor_speed = motor_speed;
}

/// (b2RevoluteJoint_GetMotorSpeed)
pub fn revolute_joint_get_motor_speed(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().motor_speed
}

/// (b2RevoluteJoint_GetMotorTorque)
pub fn revolute_joint_get_motor_torque(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    world.inv_h * joint.revolute().motor_impulse
}

/// (b2RevoluteJoint_SetMaxMotorTorque)
pub fn revolute_joint_set_max_motor_torque(world: &mut World, joint_id: JointId, torque: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_REVOLUTE_SET_MAX_MOTOR_TORQUE,
            joint_id,
            torque,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute);
    joint.revolute_mut().max_motor_torque = torque;
}

/// (b2RevoluteJoint_GetMaxMotorTorque)
pub fn revolute_joint_get_max_motor_torque(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    joint.revolute().max_motor_torque
}

/// (b2GetRevoluteJointForce)
pub fn get_revolute_joint_force(world: &World, base: &JointSim) -> Vec2 {
    mul_sv(world.inv_h, base.revolute().linear_impulse)
}

/// (b2GetRevoluteJointTorque)
pub fn get_revolute_joint_torque(world: &World, base: &JointSim) -> f32 {
    let revolute = base.revolute();
    world.inv_h * (revolute.motor_impulse + revolute.lower_impulse - revolute.upper_impulse)
}

/// (b2PrepareRevoluteJoint)
pub fn prepare_revolute_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.joint_type() == JointType::Revolute);

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

    let joint = base.revolute_mut();

    joint.index_a = index_a;
    joint.index_b = index_b;

    // Compute joint anchor frames with world space rotation, relative to
    // center of mass. Avoid round-off here as much as possible.
    // b2Vec2 pf = (xf.p - c) + rot(xf.q, f.p)
    // pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
    // pf = rot(xf.q, f.p - lc)
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

    let k = i_a + i_b;
    joint.axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

    joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.linear_impulse = VEC2_ZERO;
        joint.spring_impulse = 0.0;
        joint.motor_impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
}

/// (b2WarmStartRevoluteJoint)
pub fn warm_start_revolute_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.joint_type() == JointType::Revolute);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.revolute_mut();

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

    let axial_impulse =
        joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, joint.linear_impulse);
        state_a.angular_velocity -= i_a * (cross(r_a, joint.linear_impulse) + axial_impulse);
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, joint.linear_impulse);
        state_b.angular_velocity += i_b * (cross(r_b, joint.linear_impulse) + axial_impulse);
        states[joint.index_b as usize] = state_b;
    }
}

/// (b2SolveRevoluteJoint)
pub fn solve_revolute_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.joint_type() == JointType::Revolute);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let constraint_softness = base.constraint_softness;

    let joint = base.revolute_mut();

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

    let q_a = mul_rot(state_a.delta_rotation, joint.frame_a.q);
    let q_b = mul_rot(state_b.delta_rotation, joint.frame_b.q);
    let rel_q = inv_mul_rot(q_a, q_b);

    let fixed_rotation = i_a + i_b == 0.0;

    // Solve spring.
    if joint.enable_spring && !fixed_rotation {
        let joint_angle = rot_get_angle(rel_q);
        let joint_angle_delta = unwind_angle(joint_angle - joint.target_angle);

        let c = joint_angle_delta;
        let bias = joint.spring_softness.bias_rate * c;
        let mass_scale = joint.spring_softness.mass_scale;
        let impulse_scale = joint.spring_softness.impulse_scale;

        let c_dot = w_b - w_a;
        let impulse =
            -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * joint.spring_impulse;
        joint.spring_impulse += impulse;

        w_a -= i_a * impulse;
        w_b += i_b * impulse;
    }

    // Solve motor constraint.
    if joint.enable_motor && !fixed_rotation {
        let c_dot = w_b - w_a - joint.motor_speed;
        let mut impulse = -joint.axial_mass * c_dot;
        let old_impulse = joint.motor_impulse;
        let max_impulse = context.h * joint.max_motor_torque;
        joint.motor_impulse = clamp_float(joint.motor_impulse + impulse, -max_impulse, max_impulse);
        impulse = joint.motor_impulse - old_impulse;

        w_a -= i_a * impulse;
        w_b += i_b * impulse;
    }

    if joint.enable_limit && !fixed_rotation {
        let joint_angle = rot_get_angle(rel_q);

        // Lower limit
        {
            let c = joint_angle - joint.lower_angle;
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;
            if c > 0.0 {
                // speculation
                bias = c * context.inv_h;
            } else if use_bias {
                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            let c_dot = w_b - w_a;
            let old_impulse = joint.lower_impulse;
            let mut impulse =
                -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * old_impulse;
            joint.lower_impulse = max_float(old_impulse + impulse, 0.0);
            impulse = joint.lower_impulse - old_impulse;

            w_a -= i_a * impulse;
            w_b += i_b * impulse;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is
        // satisfied. This also keeps the impulse positive when the limit is
        // active.
        {
            let c = joint.upper_angle - joint_angle;
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;
            if c > 0.0 {
                // speculation
                bias = c * context.inv_h;
            } else if use_bias {
                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            // sign flipped on Cdot
            let c_dot = w_a - w_b;
            let old_impulse = joint.upper_impulse;
            let mut impulse =
                -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * old_impulse;
            joint.upper_impulse = max_float(old_impulse + impulse, 0.0);
            impulse = joint.upper_impulse - old_impulse;

            // sign flipped on applied impulse
            w_a += i_a * impulse;
            w_b -= i_b * impulse;
        }
    }

    // Solve point-to-point constraint
    {
        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

        // current anchors
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let c_dot = sub(add(v_b, cross_sv(w_b, r_b)), add(v_a, cross_sv(w_a, r_a)));

        let mut bias = VEC2_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            let dc_a = state_a.delta_position;
            let dc_b = state_b.delta_position;

            let separation = add(add(sub(dc_b, dc_a), sub(r_b, r_a)), joint.delta_center);
            bias = mul_sv(constraint_softness.bias_rate, separation);
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        let mut k = MAT22_ZERO;
        k.cx.x = m_a + m_b + r_a.y * r_a.y * i_a + r_b.y * r_b.y * i_b;
        k.cy.x = -r_a.y * r_a.x * i_a - r_b.y * r_b.x * i_b;
        k.cx.y = k.cy.x;
        k.cy.y = m_a + m_b + r_a.x * r_a.x * i_a + r_b.x * r_b.x * i_b;
        let b = solve_22(k, add(c_dot, bias));

        let impulse = Vec2 {
            x: -mass_scale * b.x - impulse_scale * joint.linear_impulse.x,
            y: -mass_scale * b.y - impulse_scale * joint.linear_impulse.y,
        };
        joint.linear_impulse.x += impulse.x;
        joint.linear_impulse.y += impulse.y;

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
