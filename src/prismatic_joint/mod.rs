// Port of prismatic_joint.c: public accessors, force/torque reporting, and
// debug draw. The prepare/warm-start/solve simulation functions live in
// solve.rs.
//
// Same conventions as distance_joint.rs.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//

use crate::body::get_body_transform;
use crate::id::JointId;
use crate::joint::{
    get_joint_full_id, get_joint_sim_check_type, get_joint_sim_check_type_ref, get_joint_sim_ref,
    JointSim, JointType,
};
use crate::math_functions::WorldTransform;
use crate::math_functions::{
    add, cross_sv, dot, left_perp, max_float, min_float, mul_sv, rotate_vector, sub, sub_pos,
    to_relative_transform, transform_point, Vec2, VEC2_ZERO,
};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

mod solve;

pub use solve::*;

/// (b2PrismaticJoint_EnableSpring)
pub fn prismatic_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_PRISMATIC_ENABLE_SPRING,
            joint_id,
            enable_spring,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    let prismatic = joint.prismatic_mut();
    if enable_spring != prismatic.enable_spring {
        prismatic.enable_spring = enable_spring;
        prismatic.spring_impulse = 0.0;
    }
}

/// (b2PrismaticJoint_IsSpringEnabled)
pub fn prismatic_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().enable_spring
}

/// (b2PrismaticJoint_SetSpringHertz)
pub fn prismatic_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_PRISMATIC_SET_SPRING_HERTZ,
            joint_id,
            hertz,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    joint.prismatic_mut().hertz = hertz;
}

/// (b2PrismaticJoint_GetSpringHertz)
pub fn prismatic_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().hertz
}

/// (b2PrismaticJoint_SetSpringDampingRatio)
pub fn prismatic_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_PRISMATIC_SET_SPRING_DAMPING_RATIO,
            joint_id,
            damping_ratio,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    joint.prismatic_mut().damping_ratio = damping_ratio;
}

/// (b2PrismaticJoint_GetSpringDampingRatio)
pub fn prismatic_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().damping_ratio
}

/// (b2PrismaticJoint_SetTargetTranslation)
pub fn prismatic_joint_set_target_translation(
    world: &mut World,
    joint_id: JointId,
    translation: f32,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_PRISMATIC_SET_TARGET_TRANSLATION,
            joint_id,
            translation,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    joint.prismatic_mut().target_translation = translation;
}

/// (b2PrismaticJoint_GetTargetTranslation)
pub fn prismatic_joint_get_target_translation(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().target_translation
}

/// (b2PrismaticJoint_EnableLimit)
pub fn prismatic_joint_enable_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_PRISMATIC_ENABLE_LIMIT,
            joint_id,
            enable_limit,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    let prismatic = joint.prismatic_mut();
    if enable_limit != prismatic.enable_limit {
        prismatic.enable_limit = enable_limit;
        prismatic.lower_impulse = 0.0;
        prismatic.upper_impulse = 0.0;
    }
}

/// (b2PrismaticJoint_IsLimitEnabled)
pub fn prismatic_joint_is_limit_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().enable_limit
}

/// (b2PrismaticJoint_GetLowerLimit)
pub fn prismatic_joint_get_lower_limit(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().lower_translation
}

/// (b2PrismaticJoint_GetUpperLimit)
pub fn prismatic_joint_get_upper_limit(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().upper_translation
}

/// (b2PrismaticJoint_SetLimits)
pub fn prismatic_joint_set_limits(world: &mut World, joint_id: JointId, lower: f32, upper: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32_pair(
            rec,
            crate::recording::OP_PRISMATIC_SET_LIMITS,
            joint_id,
            lower,
            upper,
        )
    });
    debug_assert!(lower <= upper);

    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    let prismatic = joint.prismatic_mut();
    if lower != prismatic.lower_translation || upper != prismatic.upper_translation {
        prismatic.lower_translation = min_float(lower, upper);
        prismatic.upper_translation = max_float(lower, upper);
        prismatic.lower_impulse = 0.0;
        prismatic.upper_impulse = 0.0;
    }
}

/// (b2PrismaticJoint_EnableMotor)
pub fn prismatic_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_bool(
            rec,
            crate::recording::OP_PRISMATIC_ENABLE_MOTOR,
            joint_id,
            enable_motor,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    let prismatic = joint.prismatic_mut();
    if enable_motor != prismatic.enable_motor {
        prismatic.enable_motor = enable_motor;
        prismatic.motor_impulse = 0.0;
    }
}

/// (b2PrismaticJoint_IsMotorEnabled)
pub fn prismatic_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().enable_motor
}

/// (b2PrismaticJoint_SetMotorSpeed)
pub fn prismatic_joint_set_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_PRISMATIC_SET_MOTOR_SPEED,
            joint_id,
            motor_speed,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    joint.prismatic_mut().motor_speed = motor_speed;
}

/// (b2PrismaticJoint_GetMotorSpeed)
pub fn prismatic_joint_get_motor_speed(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().motor_speed
}

/// (b2PrismaticJoint_GetMotorForce)
pub fn prismatic_joint_get_motor_force(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    world.inv_h * base.prismatic().motor_impulse
}

/// (b2PrismaticJoint_SetMaxMotorForce)
pub fn prismatic_joint_set_max_motor_force(world: &mut World, joint_id: JointId, force: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_joint_f32(
            rec,
            crate::recording::OP_PRISMATIC_SET_MAX_MOTOR_FORCE,
            joint_id,
            force,
        )
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic);
    joint.prismatic_mut().max_motor_force = force;
}

/// (b2PrismaticJoint_GetMaxMotorForce)
pub fn prismatic_joint_get_max_motor_force(world: &World, joint_id: JointId) -> f32 {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    joint.prismatic().max_motor_force
}

/// (b2PrismaticJoint_GetTranslation)
pub fn prismatic_joint_get_translation(world: &World, joint_id: JointId) -> f32 {
    let joint_sim = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);

    // Relative to body A so the difference stays in float precision far from
    // the origin
    let wxf_a = get_body_transform(world, joint_sim.body_id_a);
    let transform_a = to_relative_transform(wxf_a, wxf_a.p);
    let transform_b =
        to_relative_transform(get_body_transform(world, joint_sim.body_id_b), wxf_a.p);

    let local_axis_a = rotate_vector(joint_sim.local_frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    let axis_a = rotate_vector(transform_a.q, local_axis_a);
    let p_a = transform_point(transform_a, joint_sim.local_frame_a.p);
    let p_b = transform_point(transform_b, joint_sim.local_frame_b.p);
    let d = sub(p_b, p_a);
    dot(d, axis_a)
}

/// (b2PrismaticJoint_GetSpeed)
pub fn prismatic_joint_get_speed(world: &World, joint_id: JointId) -> f32 {
    let joint_index = get_joint_full_id(world, joint_id);
    debug_assert!(world.joints[joint_index as usize].type_ == JointType::Prismatic);
    let base = get_joint_sim_ref(world, joint_index);
    debug_assert!(base.joint_type() == JointType::Prismatic);

    let body_a = &world.bodies[base.body_id_a as usize];
    let body_b = &world.bodies[base.body_id_b as usize];
    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];
    let body_state_a = if body_a.set_index == AWAKE_SET {
        Some(&world.solver_sets[AWAKE_SET as usize].body_states[body_a.local_index as usize])
    } else {
        None
    };
    let body_state_b = if body_b.set_index == AWAKE_SET {
        Some(&world.solver_sets[AWAKE_SET as usize].body_states[body_b.local_index as usize])
    } else {
        None
    };

    let q_a = body_sim_a.transform.q;
    let q_b = body_sim_b.transform.q;

    let local_axis_a = rotate_vector(base.local_frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    let axis_a = rotate_vector(q_a, local_axis_a);
    let r_a = rotate_vector(q_a, sub(base.local_frame_a.p, body_sim_a.local_center));
    let r_b = rotate_vector(q_b, sub(base.local_frame_b.p, body_sim_b.local_center));

    // Difference the centers in double so the speed stays exact far from the
    // origin
    let dc = sub_pos(body_sim_b.center, body_sim_a.center);
    let d = add(dc, sub(r_b, r_a));

    let v_a = body_state_a.map_or(VEC2_ZERO, |s| s.linear_velocity);
    let v_b = body_state_b.map_or(VEC2_ZERO, |s| s.linear_velocity);
    let w_a = body_state_a.map_or(0.0, |s| s.angular_velocity);
    let w_b = body_state_b.map_or(0.0, |s| s.angular_velocity);

    let v_rel = sub(add(v_b, cross_sv(w_b, r_b)), add(v_a, cross_sv(w_a, r_a)));
    dot(d, cross_sv(w_a, axis_a)) + dot(axis_a, v_rel)
}

/// (b2GetPrismaticJointForce)
pub fn get_prismatic_joint_force(world: &World, base: &JointSim) -> Vec2 {
    let q_a = get_body_transform(world, base.body_id_a).q;

    let joint = base.prismatic();

    let local_axis_a = rotate_vector(base.local_frame_a.q, Vec2 { x: 1.0, y: 0.0 });
    let axis_a = rotate_vector(q_a, local_axis_a);
    let perp_a = left_perp(axis_a);

    let inv_h = world.inv_h;
    let perp_force = inv_h * joint.impulse.x;
    let axial_force = inv_h * (joint.motor_impulse + joint.lower_impulse - joint.upper_impulse);

    add(mul_sv(perp_force, perp_a), mul_sv(axial_force, axis_a))
}

/// (b2GetPrismaticJointTorque)
pub fn get_prismatic_joint_torque(world: &World, base: &JointSim) -> f32 {
    world.inv_h * base.prismatic().impulse.y
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(perp, d)
// Cdot = dot(d, cross(wA, perp)) + dot(perp, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perp, vA) - dot(cross(rA + d, perp), wA) + dot(perp, vB) + dot(cross(rB, perp), vB)
// J = [-perp, -cross(rA + d, perp), perp, cross(rB, perp)]
//
// Angular constraint
// C = aB - aA + a_initial
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -sA a sB]
//     [0  -1  0  1]
// a = perp
// sA = cross(rA + d, a) = cross(pB - xA, a)
// sB = cross(rB, a) = cross(pB - xB, a)
//
// Motor/Limit linear constraint
// C = dot(axA, d)
// Cdot = -dot(axA, vA) - dot(cross(rA + d, axA), wA) + dot(axA, vB) + dot(cross(rB, axA), vB)
// J = [-axA -cross(rA + d, axA) axA cross(rB, ax1)]
//
// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0
//
// Block Solver
// We develop a block solver that includes the angular and linear constraints.
// This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

/// (b2DrawPrismaticJoint)
pub fn draw_prismatic_joint(
    draw: &mut dyn crate::debug_draw::DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    draw_scale: f32,
) {
    use crate::debug_draw::HexColor;
    use crate::math_functions::{
        left_perp, mul_sv, neg, offset_pos, offset_world_transform, rotate_vector, Vec2,
    };

    debug_assert!(base.joint_type() == JointType::Prismatic);

    let joint = base.prismatic();

    let frame_a = offset_world_transform(transform_a, base.local_frame_a);
    let frame_b = offset_world_transform(transform_b, base.local_frame_b);
    let axis_a = rotate_vector(frame_a.q, Vec2 { x: 1.0, y: 0.0 });

    draw.draw_line(frame_a.p, frame_b.p, HexColor::DIM_GRAY);

    if joint.enable_limit {
        let b = 0.25 * draw_scale;
        let lower = offset_pos(frame_a.p, mul_sv(joint.lower_translation, axis_a));
        let upper = offset_pos(frame_a.p, mul_sv(joint.upper_translation, axis_a));
        let perp = left_perp(axis_a);
        draw.draw_line(lower, upper, HexColor::GRAY);
        draw.draw_line(
            offset_pos(lower, mul_sv(-b, perp)),
            offset_pos(lower, mul_sv(b, perp)),
            HexColor::GREEN,
        );
        draw.draw_line(
            offset_pos(upper, mul_sv(-b, perp)),
            offset_pos(upper, mul_sv(b, perp)),
            HexColor::RED,
        );
    } else {
        draw.draw_line(
            offset_pos(frame_a.p, neg(axis_a)),
            offset_pos(frame_a.p, axis_a),
            HexColor::GRAY,
        );
    }

    if joint.enable_spring {
        let p = offset_pos(frame_a.p, mul_sv(joint.target_translation, axis_a));
        draw.draw_point(p, 8.0, HexColor::VIOLET);
    }

    draw.draw_point(frame_a.p, 5.0, HexColor::GRAY);
    draw.draw_point(frame_b.p, 5.0, HexColor::BLUE);
}
