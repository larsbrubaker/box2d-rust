// Simulation functions of prismatic_joint.c: prepare, warm start, solve.
// See mod.rs for the constraint derivation notes.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::{get_length_units_per_meter, NULL_INDEX};
use crate::joint::{JointSim, JointType};
use crate::math_functions::{
    add, clamp_float, cross, dot, inv_mul_rot, is_valid_float, is_valid_vec2, left_perp, max_float,
    min_float, mul_add, mul_rot, mul_sub, mul_sv, rot_get_angle, rotate_vector, solve_22, sub,
    sub_pos, Mat22, Vec2, VEC2_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b2PreparePrismaticJoint)
pub fn prepare_prismatic_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.joint_type() == JointType::Prismatic);

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

    let joint = base.prismatic_mut();
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

    joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.impulse = VEC2_ZERO;
        joint.spring_impulse = 0.0;
        joint.motor_impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
}

/// (b2WarmStartPrismaticJoint)
pub fn warm_start_prismatic_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.joint_type() == JointType::Prismatic);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.prismatic_mut();

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

    // impulse is applied at anchor point on body B
    let a1 = cross(add(r_a, d), axis_a);
    let a2 = cross(r_b, axis_a);
    let axial_impulse =
        joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;

    // perpendicular constraint
    let perp_a = left_perp(axis_a);
    let s1 = cross(add(r_a, d), perp_a);
    let s2 = cross(r_b, perp_a);
    let perp_impulse = joint.impulse.x;
    let angle_impulse = joint.impulse.y;

    let p = add(mul_sv(axial_impulse, axis_a), mul_sv(perp_impulse, perp_a));
    let l_a = axial_impulse * a1 + perp_impulse * s1 + angle_impulse;
    let l_b = axial_impulse * a2 + perp_impulse * s2 + angle_impulse;

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

/// (b2SolvePrismaticJoint)
pub fn solve_prismatic_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.joint_type() == JointType::Prismatic);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let softness = base.constraint_softness;

    let joint = base.prismatic_mut();

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

    // These scalars are for torques generated by axial forces
    let a1 = cross(add(r_a, d), axis_a);
    let a2 = cross(r_b, axis_a);

    let k = m_a + m_b + i_a * a1 * a1 + i_b * a2 * a2;
    let axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

    // spring constraint
    if joint.enable_spring {
        // This is a real spring and should be applied even during relax
        let c = translation - joint.target_translation;
        let bias = joint.spring_softness.bias_rate * c;
        let mass_scale = joint.spring_softness.mass_scale;
        let impulse_scale = joint.spring_softness.impulse_scale;

        let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a;
        let delta_impulse =
            -mass_scale * axial_mass * (c_dot + bias) - impulse_scale * joint.spring_impulse;
        joint.spring_impulse += delta_impulse;

        let p = mul_sv(delta_impulse, axis_a);
        let l_a = delta_impulse * a1;
        let l_b = delta_impulse * a2;

        v_a = mul_sub(v_a, m_a, p);
        w_a -= i_a * l_a;
        v_b = mul_add(v_b, m_b, p);
        w_b += i_b * l_b;
    }

    // Solve motor constraint
    if joint.enable_motor {
        let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a;
        let mut impulse = axial_mass * (joint.motor_speed - c_dot);
        let old_impulse = joint.motor_impulse;
        let max_impulse = context.h * joint.max_motor_force;
        joint.motor_impulse = clamp_float(joint.motor_impulse + impulse, -max_impulse, max_impulse);
        impulse = joint.motor_impulse - old_impulse;

        let p = mul_sv(impulse, axis_a);
        let l_a = impulse * a1;
        let l_b = impulse * a2;

        v_a = mul_sub(v_a, m_a, p);
        w_a -= i_a * l_a;
        v_b = mul_add(v_b, m_b, p);
        w_b += i_b * l_b;
    }

    if joint.enable_limit {
        // Clamp the speculative distance to a reasonable value
        let speculative_distance = 0.25 * (joint.upper_translation - joint.lower_translation);

        // Lower limit
        {
            let c = translation - joint.lower_translation;

            if c < speculative_distance {
                let mut bias = 0.0;
                let mut mass_scale = 1.0;
                let mut impulse_scale = 0.0;

                if c > 0.0 {
                    // speculation
                    let safe = get_length_units_per_meter();
                    bias = min_float(c, safe) * context.inv_h;
                } else if use_bias {
                    bias = softness.bias_rate * c;
                    mass_scale = softness.mass_scale;
                    impulse_scale = softness.impulse_scale;
                }

                let old_impulse = joint.lower_impulse;
                let c_dot = dot(axis_a, sub(v_b, v_a)) + a2 * w_b - a1 * w_a;
                let mut delta_impulse =
                    -axial_mass * mass_scale * (c_dot + bias) - impulse_scale * old_impulse;
                joint.lower_impulse = max_float(old_impulse + delta_impulse, 0.0);
                delta_impulse = joint.lower_impulse - old_impulse;

                let p = mul_sv(delta_impulse, axis_a);
                let l_a = delta_impulse * a1;
                let l_b = delta_impulse * a2;

                v_a = mul_sub(v_a, m_a, p);
                w_a -= i_a * l_a;
                v_b = mul_add(v_b, m_b, p);
                w_b += i_b * l_b;
            } else {
                joint.lower_impulse = 0.0;
            }
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is
        // satisfied. This also keeps the impulse positive when the limit is
        // active.
        {
            // sign flipped
            let c = joint.upper_translation - translation;

            if c < speculative_distance {
                let mut bias = 0.0;
                let mut mass_scale = 1.0;
                let mut impulse_scale = 0.0;

                if c > 0.0 {
                    // speculation
                    let safe = get_length_units_per_meter();
                    bias = min_float(c, safe) * context.inv_h;
                } else if use_bias {
                    bias = softness.bias_rate * c;
                    mass_scale = softness.mass_scale;
                    impulse_scale = softness.impulse_scale;
                }

                let old_impulse = joint.upper_impulse;

                // sign flipped
                let c_dot = dot(axis_a, sub(v_a, v_b)) + a1 * w_a - a2 * w_b;
                let mut delta_impulse =
                    -axial_mass * mass_scale * (c_dot + bias) - impulse_scale * old_impulse;
                joint.upper_impulse = max_float(old_impulse + delta_impulse, 0.0);
                delta_impulse = joint.upper_impulse - old_impulse;

                let p = mul_sv(delta_impulse, axis_a);
                let l_a = delta_impulse * a1;
                let l_b = delta_impulse * a2;

                // sign flipped
                v_a = mul_add(v_a, m_a, p);
                w_a += i_a * l_a;
                v_b = mul_sub(v_b, m_b, p);
                w_b -= i_b * l_b;
            } else {
                joint.upper_impulse = 0.0;
            }
        }
    }

    // Solve the prismatic constraint in block form
    {
        let perp_a = left_perp(axis_a);

        // These scalars are for torques generated by the perpendicular
        // constraint force
        let s1 = cross(add(d, r_a), perp_a);
        let s2 = cross(r_b, perp_a);

        let c_dot = Vec2 {
            x: dot(perp_a, sub(v_b, v_a)) + s2 * w_b - s1 * w_a,
            y: w_b - w_a,
        };

        let mut bias = VEC2_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            let c = Vec2 {
                x: dot(perp_a, d),
                y: rot_get_angle(rel_q),
            };

            bias = mul_sv(softness.bias_rate, c);
            mass_scale = softness.mass_scale;
            impulse_scale = softness.impulse_scale;
        }

        let k11 = m_a + m_b + i_a * s1 * s1 + i_b * s2 * s2;
        let k12 = i_a * s1 + i_b * s2;
        let mut k22 = i_a + i_b;
        if k22 == 0.0 {
            // For bodies with fixed rotation.
            k22 = 1.0;
        }

        let k = Mat22 {
            cx: Vec2 { x: k11, y: k12 },
            cy: Vec2 { x: k12, y: k22 },
        };

        let b = solve_22(k, add(c_dot, bias));
        let delta_impulse = Vec2 {
            x: -mass_scale * b.x - impulse_scale * joint.impulse.x,
            y: -mass_scale * b.y - impulse_scale * joint.impulse.y,
        };

        joint.impulse.x += delta_impulse.x;
        joint.impulse.y += delta_impulse.y;

        let p = mul_sv(delta_impulse.x, perp_a);
        let l_a = delta_impulse.x * s1 + delta_impulse.y;
        let l_b = delta_impulse.x * s2 + delta_impulse.y;

        v_a = mul_sub(v_a, m_a, p);
        w_a -= i_a * l_a;
        v_b = mul_add(v_b, m_b, p);
        w_b += i_b * l_b;
    }

    debug_assert!(is_valid_vec2(v_a));
    debug_assert!(is_valid_float(w_a));
    debug_assert!(is_valid_vec2(v_b));
    debug_assert!(is_valid_float(w_b));

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
