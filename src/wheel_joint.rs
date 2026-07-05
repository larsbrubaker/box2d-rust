// Port of wheel_joint.c: public accessors, force/torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// Same conventions as distance_joint.rs. b2DrawWheelJoint lands with the
// debug-draw phase.
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
    add, clamp_float, cross, dot, left_perp, max_float, min_float, mul_add, mul_rot, mul_sub,
    mul_sv, rotate_vector, sub, sub_pos, Vec2,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b2WheelJoint_EnableSpring)
pub fn wheel_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_WHEEL_ENABLE_SPRING,
            joint_id,
            enable_spring,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    let wheel = joint.wheel_mut();
    if enable_spring != wheel.enable_spring {
        wheel.enable_spring = enable_spring;
        wheel.spring_impulse = 0.0;
    }
}

/// (b2WheelJoint_IsSpringEnabled)
pub fn wheel_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().enable_spring
}

/// (b2WheelJoint_SetSpringHertz)
pub fn wheel_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WHEEL_SET_SPRING_HERTZ,
            joint_id,
            hertz,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    joint.wheel_mut().hertz = hertz;
}

/// (b2WheelJoint_GetSpringHertz)
pub fn wheel_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().hertz
}

/// (b2WheelJoint_SetSpringDampingRatio)
pub fn wheel_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WHEEL_SET_SPRING_DAMPING_RATIO,
            joint_id,
            damping_ratio,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    joint.wheel_mut().damping_ratio = damping_ratio;
}

/// (b2WheelJoint_GetSpringDampingRatio)
pub fn wheel_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().damping_ratio
}

/// (b2WheelJoint_EnableLimit)
pub fn wheel_joint_enable_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_WHEEL_ENABLE_LIMIT,
            joint_id,
            enable_limit,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    let wheel = joint.wheel_mut();
    if wheel.enable_limit != enable_limit {
        wheel.lower_impulse = 0.0;
        wheel.upper_impulse = 0.0;
        wheel.enable_limit = enable_limit;
    }
}

/// (b2WheelJoint_IsLimitEnabled)
pub fn wheel_joint_is_limit_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().enable_limit
}

/// (b2WheelJoint_GetLowerLimit)
pub fn wheel_joint_get_lower_limit(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().lower_translation
}

/// (b2WheelJoint_GetUpperLimit)
pub fn wheel_joint_get_upper_limit(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().upper_translation
}

/// (b2WheelJoint_SetLimits)
pub fn wheel_joint_set_limits(world: &mut World, joint_id: JointId, lower: f32, upper: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32_pair(
            rec,
            crate::recording::OP_WHEEL_SET_LIMITS,
            joint_id,
            lower,
            upper,
        )
    });
    debug_assert!(lower <= upper);

    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    let wheel = joint.wheel_mut();
    if lower != wheel.lower_translation || upper != wheel.upper_translation {
        wheel.lower_translation = min_float(lower, upper);
        wheel.upper_translation = max_float(lower, upper);
        wheel.lower_impulse = 0.0;
        wheel.upper_impulse = 0.0;
    }
}

/// (b2WheelJoint_EnableMotor)
pub fn wheel_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_WHEEL_ENABLE_MOTOR,
            joint_id,
            enable_motor,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    let wheel = joint.wheel_mut();
    if wheel.enable_motor != enable_motor {
        wheel.motor_impulse = 0.0;
        wheel.enable_motor = enable_motor;
    }
}

/// (b2WheelJoint_IsMotorEnabled)
pub fn wheel_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().enable_motor
}

/// (b2WheelJoint_SetMotorSpeed)
pub fn wheel_joint_set_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WHEEL_SET_MOTOR_SPEED,
            joint_id,
            motor_speed,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    joint.wheel_mut().motor_speed = motor_speed;
}

/// (b2WheelJoint_GetMotorSpeed)
pub fn wheel_joint_get_motor_speed(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().motor_speed
}

/// (b2WheelJoint_GetMotorTorque)
pub fn wheel_joint_get_motor_torque(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    world.inv_h * joint.wheel().motor_impulse
}

/// (b2WheelJoint_SetMaxMotorTorque)
pub fn wheel_joint_set_max_motor_torque(world: &mut World, joint_id: JointId, torque: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WHEEL_SET_MAX_MOTOR_TORQUE,
            joint_id,
            torque,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel);
    joint.wheel_mut().max_motor_torque = torque;
}

/// (b2WheelJoint_GetMaxMotorTorque)
pub fn wheel_joint_get_max_motor_torque(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    joint.wheel().max_motor_torque
}

/// (b2GetWheelJointForce)
pub fn get_wheel_joint_force(world: &World, base: &JointSim) -> Vec2 {
    let q_a = get_body_transform(world, base.body_id_a).q;

    let local_axis_a = rotate_vector(base.local_frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    let axis_a = rotate_vector(q_a, local_axis_a);
    let perp_a = left_perp(axis_a);

    let joint = base.wheel();

    let perp_force = world.inv_h * joint.perp_impulse;
    let axial_force =
        world.inv_h * (joint.spring_impulse + joint.lower_impulse - joint.upper_impulse);

    add(mul_sv(perp_force, perp_a), mul_sv(axial_force, axis_a))
}

/// (b2GetWheelJointTorque)
pub fn get_wheel_joint_torque(world: &World, base: &JointSim) -> f32 {
    world.inv_h * base.wheel().motor_impulse
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]
//
// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]
//
// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

/// (b2PrepareWheelJoint)
pub fn prepare_wheel_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.joint_type() == JointType::Wheel);

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

    let joint = base.wheel_mut();

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

    let d = add(joint.delta_center, sub(r_b, r_a));
    let axis_a = rotate_vector(joint.frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    let perp_a = left_perp(axis_a);

    // perpendicular constraint (keep wheel on line)
    let s1 = cross(add(d, r_a), perp_a);
    let s2 = cross(r_b, perp_a);

    let kp = m_a + m_b + i_a * s1 * s1 + i_b * s2 * s2;
    joint.perp_mass = if kp > 0.0 { 1.0 / kp } else { 0.0 };

    // spring constraint
    let a1 = cross(add(d, r_a), axis_a);
    let a2 = cross(r_b, axis_a);

    let ka = m_a + m_b + i_a * a1 * a1 + i_b * a2 * a2;
    joint.axial_mass = if ka > 0.0 { 1.0 / ka } else { 0.0 };

    joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    let km = i_a + i_b;
    joint.motor_mass = if km > 0.0 { 1.0 / km } else { 0.0 };

    if !context.enable_warm_starting {
        joint.perp_impulse = 0.0;
        joint.spring_impulse = 0.0;
        joint.motor_impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
}

/// (b2WarmStartWheelJoint)
pub fn warm_start_wheel_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.joint_type() == JointType::Wheel);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.wheel_mut();

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

    let d = add(
        add(
            sub(state_b.delta_position, state_a.delta_position),
            joint.delta_center,
        ),
        sub(r_b, r_a),
    );
    let mut axis_a = rotate_vector(joint.frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    axis_a = rotate_vector(state_a.delta_rotation, axis_a);
    let perp_a = left_perp(axis_a);

    let a1 = cross(add(d, r_a), axis_a);
    let a2 = cross(r_b, axis_a);
    let s1 = cross(add(d, r_a), perp_a);
    let s2 = cross(r_b, perp_a);

    let axial_impulse = joint.spring_impulse + joint.lower_impulse - joint.upper_impulse;

    let p = add(
        mul_sv(axial_impulse, axis_a),
        mul_sv(joint.perp_impulse, perp_a),
    );
    let l_a = axial_impulse * a1 + joint.perp_impulse * s1 + joint.motor_impulse;
    let l_b = axial_impulse * a2 + joint.perp_impulse * s2 + joint.motor_impulse;

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, p);
        state_a.angular_velocity -= i_a * l_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, p);
        state_b.angular_velocity += i_b * l_b;
        states[joint.index_b as usize] = state_b;
    }
}

/// (b2SolveWheelJoint)
pub fn solve_wheel_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.joint_type() == JointType::Wheel);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let constraint_softness = base.constraint_softness;

    let joint = base.wheel_mut();

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

    let fixed_rotation = i_a + i_b == 0.0;

    // current anchors
    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    let d = add(
        add(
            sub(state_b.delta_position, state_a.delta_position),
            joint.delta_center,
        ),
        sub(r_b, r_a),
    );
    let mut axis_a = rotate_vector(joint.frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    axis_a = rotate_vector(state_a.delta_rotation, axis_a);
    let translation = dot(axis_a, d);

    let a1 = cross(add(d, r_a), axis_a);
    let a2 = cross(r_b, axis_a);

    // motor constraint
    if joint.enable_motor && !fixed_rotation {
        let c_dot = w_b - w_a - joint.motor_speed;
        let mut impulse = -joint.motor_mass * c_dot;
        let old_impulse = joint.motor_impulse;
        let max_impulse = context.h * joint.max_motor_torque;
        joint.motor_impulse = clamp_float(joint.motor_impulse + impulse, -max_impulse, max_impulse);
        impulse = joint.motor_impulse - old_impulse;

        w_a -= i_a * impulse;
        w_b += i_b * impulse;
    }

    // spring constraint
    if joint.enable_spring {
        // This is a real spring and should be applied even during relax
        let c = translation;
        let bias = joint.spring_softness.bias_rate * c;
        let mass_scale = joint.spring_softness.mass_scale;
        let impulse_scale = joint.spring_softness.impulse_scale;

        let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a;
        let impulse =
            -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * joint.spring_impulse;
        joint.spring_impulse += impulse;

        let p = mul_sv(impulse, axis_a);
        let l_a = impulse * a1;
        let l_b = impulse * a2;

        v_a = mul_sub(v_a, m_a, p);
        w_a -= i_a * l_a;
        v_b = mul_add(v_b, m_b, p);
        w_b += i_b * l_b;
    }

    if joint.enable_limit {
        // Lower limit
        {
            let c = translation - joint.lower_translation;
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

            let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a;
            let mut impulse = -mass_scale * joint.axial_mass * (c_dot + bias)
                - impulse_scale * joint.lower_impulse;
            let old_impulse = joint.lower_impulse;
            joint.lower_impulse = max_float(old_impulse + impulse, 0.0);
            impulse = joint.lower_impulse - old_impulse;

            let p = mul_sv(impulse, axis_a);
            let l_a = impulse * a1;
            let l_b = impulse * a2;

            v_a = mul_sub(v_a, m_a, p);
            w_a -= i_a * l_a;
            v_b = mul_add(v_b, m_b, p);
            w_b += i_b * l_b;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is
        // satisfied. This also keeps the impulse positive when the limit is
        // active.
        {
            // sign flipped
            let c = joint.upper_translation - translation;
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
            let c_dot = dot(axis_a, sub(v_a, v_b)) + a1 * w_a - a2 * w_b;
            let mut impulse = -mass_scale * joint.axial_mass * (c_dot + bias)
                - impulse_scale * joint.upper_impulse;
            let old_impulse = joint.upper_impulse;
            joint.upper_impulse = max_float(old_impulse + impulse, 0.0);
            impulse = joint.upper_impulse - old_impulse;

            let p = mul_sv(impulse, axis_a);
            let l_a = impulse * a1;
            let l_b = impulse * a2;

            // sign flipped on applied impulse
            v_a = mul_add(v_a, m_a, p);
            w_a += i_a * l_a;
            v_b = mul_sub(v_b, m_b, p);
            w_b -= i_b * l_b;
        }
    }

    // point to line constraint
    {
        let perp_a = left_perp(axis_a);

        let mut bias = 0.0;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            let c = dot(perp_a, d);
            bias = constraint_softness.bias_rate * c;
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        let s1 = cross(add(d, r_a), perp_a);
        let s2 = cross(r_b, perp_a);
        let c_dot = dot(perp_a, sub(v_b, v_a)) + s2 * w_b - s1 * w_a;

        let impulse =
            -mass_scale * joint.perp_mass * (c_dot + bias) - impulse_scale * joint.perp_impulse;
        joint.perp_impulse += impulse;

        let p = mul_sv(impulse, perp_a);
        let l_a = impulse * s1;
        let l_b = impulse * s2;

        v_a = mul_sub(v_a, m_a, p);
        w_a -= i_a * l_a;
        v_b = mul_add(v_b, m_b, p);
        w_b += i_b * l_b;
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
