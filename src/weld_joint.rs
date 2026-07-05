// Port of weld_joint.c: public accessors, force/torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// Same conventions as distance_joint.rs. The C B2_WELD_BLOCK_SOLVE branch is
// compiled out upstream (block solve doesn't work correctly with mixed
// stiffness values) and is not ported.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: prepare/warm-start/solve are called by the solver slice.

use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::joint::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::math_functions::WorldTransform;
use crate::math_functions::{
    add, cross, cross_sv, inv_mul_rot, is_valid_float, is_valid_vec2, mul_add, mul_rot, mul_sub,
    mul_sv, rot_get_angle, rotate_vector, solve_22, sub, sub_pos, Vec2, MAT22_ZERO, VEC2_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b2WeldJoint_SetLinearHertz)
pub fn weld_joint_set_linear_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WELD_SET_LINEAR_HERTZ,
            joint_id,
            hertz,
        )
    });
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Weld);
    joint.weld_mut().linear_hertz = hertz;
}

/// (b2WeldJoint_GetLinearHertz)
pub fn weld_joint_get_linear_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Weld);
    joint.weld().linear_hertz
}

/// (b2WeldJoint_SetLinearDampingRatio)
pub fn weld_joint_set_linear_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WELD_SET_LINEAR_DAMPING_RATIO,
            joint_id,
            damping_ratio,
        )
    });
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Weld);
    joint.weld_mut().linear_damping_ratio = damping_ratio;
}

/// (b2WeldJoint_GetLinearDampingRatio)
pub fn weld_joint_get_linear_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Weld);
    joint.weld().linear_damping_ratio
}

/// (b2WeldJoint_SetAngularHertz)
pub fn weld_joint_set_angular_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WELD_SET_ANGULAR_HERTZ,
            joint_id,
            hertz,
        )
    });
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Weld);
    joint.weld_mut().angular_hertz = hertz;
}

/// (b2WeldJoint_GetAngularHertz)
pub fn weld_joint_get_angular_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Weld);
    joint.weld().angular_hertz
}

/// (b2WeldJoint_SetAngularDampingRatio)
pub fn weld_joint_set_angular_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_WELD_SET_ANGULAR_DAMPING_RATIO,
            joint_id,
            damping_ratio,
        )
    });
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Weld);
    joint.weld_mut().angular_damping_ratio = damping_ratio;
}

/// (b2WeldJoint_GetAngularDampingRatio)
pub fn weld_joint_get_angular_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Weld);
    joint.weld().angular_damping_ratio
}

/// (b2GetWeldJointForce)
pub fn get_weld_joint_force(world: &World, base: &JointSim) -> Vec2 {
    mul_sv(world.inv_h, base.weld().linear_impulse)
}

/// (b2GetWeldJointTorque)
pub fn get_weld_joint_torque(world: &World, base: &JointSim) -> f32 {
    world.inv_h * base.weld().angular_impulse
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-E -r1_skew E r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/// (b2PrepareWeldJoint)
pub fn prepare_weld_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.joint_type() == JointType::Weld);

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
    let constraint_softness = base.constraint_softness;

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

    let joint = base.weld_mut();
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

    let ka = i_a + i_b;
    joint.axial_mass = if ka > 0.0 { 1.0 / ka } else { 0.0 };

    if joint.linear_hertz == 0.0 {
        joint.linear_spring = constraint_softness;
    } else {
        joint.linear_spring = make_soft(joint.linear_hertz, joint.linear_damping_ratio, context.h);
    }

    if joint.angular_hertz == 0.0 {
        joint.angular_spring = constraint_softness;
    } else {
        joint.angular_spring =
            make_soft(joint.angular_hertz, joint.angular_damping_ratio, context.h);
    }

    if !context.enable_warm_starting {
        joint.linear_impulse = VEC2_ZERO;
        joint.angular_impulse = 0.0;
    }
}

/// (b2WarmStartWeldJoint)
pub fn warm_start_weld_joint(base: &mut JointSim, states: &mut [BodyState]) {
    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.weld_mut();

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

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, joint.linear_impulse);
        state_a.angular_velocity -=
            i_a * (cross(r_a, joint.linear_impulse) + joint.angular_impulse);
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, joint.linear_impulse);
        state_b.angular_velocity +=
            i_b * (cross(r_b, joint.linear_impulse) + joint.angular_impulse);
        states[joint.index_b as usize] = state_b;
    }
}

/// (b2SolveWeldJoint)
pub fn solve_weld_joint(
    base: &mut JointSim,
    _context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.joint_type() == JointType::Weld);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.weld_mut();

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

    // angular constraint
    {
        let q_a = mul_rot(state_a.delta_rotation, joint.frame_a.q);
        let q_b = mul_rot(state_b.delta_rotation, joint.frame_b.q);
        let rel_q = inv_mul_rot(q_a, q_b);
        let joint_angle = rot_get_angle(rel_q);

        let mut bias = 0.0;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias || joint.angular_hertz > 0.0 {
            let c = joint_angle;
            bias = joint.angular_spring.bias_rate * c;
            mass_scale = joint.angular_spring.mass_scale;
            impulse_scale = joint.angular_spring.impulse_scale;
        }

        let c_dot = w_b - w_a;
        let impulse =
            -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * joint.angular_impulse;
        joint.angular_impulse += impulse;

        w_a -= i_a * impulse;
        w_b += i_b * impulse;
    }

    // linear constraint
    {
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let mut bias = VEC2_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias || joint.linear_hertz > 0.0 {
            let dc_a = state_a.delta_position;
            let dc_b = state_b.delta_position;
            let c = add(add(sub(dc_b, dc_a), sub(r_b, r_a)), joint.delta_center);

            bias = mul_sv(joint.linear_spring.bias_rate, c);
            mass_scale = joint.linear_spring.mass_scale;
            impulse_scale = joint.linear_spring.impulse_scale;
        }

        let c_dot = sub(add(v_b, cross_sv(w_b, r_b)), add(v_a, cross_sv(w_a, r_a)));

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

        joint.linear_impulse = add(joint.linear_impulse, impulse);

        v_a = mul_sub(v_a, m_a, impulse);
        w_a -= i_a * cross(r_a, impulse);
        v_b = mul_add(v_b, m_b, impulse);
        w_b += i_b * cross(r_b, impulse);
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

/// (b2DrawWeldJoint)
pub fn draw_weld_joint(
    draw: &mut dyn crate::debug_draw::DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    draw_scale: f32,
) {
    use crate::debug_draw::HexColor;
    use crate::geometry::make_box;
    use crate::math_functions::offset_world_transform;

    debug_assert!(base.joint_type() == JointType::Weld);

    let frame_a = offset_world_transform(transform_a, base.local_frame_a);
    let frame_b = offset_world_transform(transform_b, base.local_frame_b);

    let box_ = make_box(0.25 * draw_scale, 0.125 * draw_scale);
    draw.draw_polygon(frame_a, &box_.vertices[..4], HexColor::DARK_ORANGE);
    draw.draw_polygon(frame_b, &box_.vertices[..4], HexColor::DARK_CYAN);
}
