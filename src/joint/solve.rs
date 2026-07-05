// Joint solve dispatch from joint.c: b2PrepareJoint/b2WarmStartJoint/
// b2SolveJoint and the joint reaction used for joint events.
//
// The C dispatch passes b2StepContext (which carries world and body-state
// pointers); the serial Rust port passes the world/states explicitly with the
// same split used by the per-type files. The overflow/graph-color task loops
// (b2PrepareJoints_Overflow, b2PrepareJointsTask, ...) belong to the solver
// slice, which iterates the colors serially.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: called by the solver slice.

use super::*;
use crate::body::BodyState;
use crate::distance_joint::{
    get_distance_joint_force, prepare_distance_joint, solve_distance_joint,
    warm_start_distance_joint,
};
use crate::math_functions::{abs_float, add, length, min_float, Vec2, VEC2_ZERO};
use crate::motor_joint::{
    get_motor_joint_force, get_motor_joint_torque, prepare_motor_joint, solve_motor_joint,
    warm_start_motor_joint,
};
use crate::prismatic_joint::{
    get_prismatic_joint_force, get_prismatic_joint_torque, prepare_prismatic_joint,
    solve_prismatic_joint, warm_start_prismatic_joint,
};
use crate::revolute_joint::{
    get_revolute_joint_force, get_revolute_joint_torque, prepare_revolute_joint,
    solve_revolute_joint, warm_start_revolute_joint,
};
use crate::solver::{make_soft, StepContext};
use crate::weld_joint::{
    get_weld_joint_force, get_weld_joint_torque, prepare_weld_joint, solve_weld_joint,
    warm_start_weld_joint,
};
use crate::wheel_joint::{
    get_wheel_joint_force, get_wheel_joint_torque, prepare_wheel_joint, solve_wheel_joint,
    warm_start_wheel_joint,
};
use crate::world::World;

/// (b2PrepareJoint)
pub fn prepare_joint(world: &World, joint: &mut JointSim, context: &StepContext) {
    // Clamp joint hertz based on the time step to reduce jitter.
    let hertz = min_float(joint.constraint_hertz, 0.25 * context.inv_h);
    joint.constraint_softness = make_soft(hertz, joint.constraint_damping_ratio, context.h);

    match joint.joint_type() {
        JointType::Distance => prepare_distance_joint(world, joint, context),
        JointType::Motor => prepare_motor_joint(world, joint, context),
        JointType::Filter => {}
        JointType::Prismatic => prepare_prismatic_joint(world, joint, context),
        JointType::Revolute => prepare_revolute_joint(world, joint, context),
        JointType::Weld => prepare_weld_joint(world, joint, context),
        JointType::Wheel => prepare_wheel_joint(world, joint, context),
    }
}

/// (b2WarmStartJoint)
pub fn warm_start_joint(joint: &mut JointSim, states: &mut [BodyState]) {
    match joint.joint_type() {
        JointType::Distance => warm_start_distance_joint(joint, states),
        JointType::Motor => warm_start_motor_joint(joint, states),
        JointType::Filter => {}
        JointType::Prismatic => warm_start_prismatic_joint(joint, states),
        JointType::Revolute => warm_start_revolute_joint(joint, states),
        JointType::Weld => warm_start_weld_joint(joint, states),
        JointType::Wheel => warm_start_wheel_joint(joint, states),
    }
}

/// (b2SolveJoint)
pub fn solve_joint(
    joint: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    match joint.joint_type() {
        JointType::Distance => solve_distance_joint(joint, context, states, use_bias),
        JointType::Motor => solve_motor_joint(joint, context, states),
        JointType::Filter => {}
        JointType::Prismatic => solve_prismatic_joint(joint, context, states, use_bias),
        JointType::Revolute => solve_revolute_joint(joint, context, states, use_bias),
        JointType::Weld => solve_weld_joint(joint, context, states, use_bias),
        JointType::Wheel => solve_wheel_joint(joint, context, states, use_bias),
    }
}

/// Reaction force/torque magnitudes for joint events. Returns
/// (force, torque). (b2GetJointReaction — C returns through out-pointers)
pub fn get_joint_reaction(sim: &JointSim, inv_time_step: f32) -> (f32, f32) {
    let mut linear_impulse = 0.0;
    let mut angular_impulse = 0.0;

    match &sim.payload {
        JointPayload::Distance(joint) => {
            linear_impulse = abs_float(
                joint.impulse + joint.lower_impulse - joint.upper_impulse + joint.motor_impulse,
            );
        }

        JointPayload::Motor(joint) => {
            linear_impulse = length(add(
                joint.linear_velocity_impulse,
                joint.linear_spring_impulse,
            ));
            angular_impulse =
                abs_float(joint.angular_velocity_impulse + joint.angular_spring_impulse);
        }

        JointPayload::Prismatic(joint) => {
            let perp_impulse = joint.impulse.x;
            let axial_impulse = joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;
            linear_impulse = (perp_impulse * perp_impulse + axial_impulse * axial_impulse).sqrt();
            angular_impulse = abs_float(joint.impulse.y);
        }

        JointPayload::Revolute(joint) => {
            linear_impulse = length(joint.linear_impulse);
            angular_impulse =
                abs_float(joint.motor_impulse + joint.lower_impulse - joint.upper_impulse);
        }

        JointPayload::Weld(joint) => {
            linear_impulse = length(joint.linear_impulse);
            angular_impulse = abs_float(joint.angular_impulse);
        }

        JointPayload::Wheel(joint) => {
            let perp_impulse = joint.perp_impulse;
            let axial_impulse = joint.spring_impulse + joint.lower_impulse - joint.upper_impulse;
            linear_impulse = (perp_impulse * perp_impulse + axial_impulse * axial_impulse).sqrt();
            angular_impulse = abs_float(joint.motor_impulse);
        }

        JointPayload::Filter => {}
    }

    (
        linear_impulse * inv_time_step,
        angular_impulse * inv_time_step,
    )
}

/// (static b2GetJointConstraintForce — takes the raw joint index)
pub(crate) fn get_joint_constraint_force(world: &World, joint_index: i32) -> Vec2 {
    let base = get_joint_sim_ref(world, joint_index);

    match world.joints[joint_index as usize].type_ {
        JointType::Distance => get_distance_joint_force(world, base),
        JointType::Motor => get_motor_joint_force(world, base),
        JointType::Filter => VEC2_ZERO,
        JointType::Prismatic => get_prismatic_joint_force(world, base),
        JointType::Revolute => get_revolute_joint_force(world, base),
        JointType::Weld => get_weld_joint_force(world, base),
        JointType::Wheel => get_wheel_joint_force(world, base),
    }
}

/// (static b2GetJointConstraintTorque)
pub(crate) fn get_joint_constraint_torque(world: &World, joint_index: i32) -> f32 {
    let base = get_joint_sim_ref(world, joint_index);

    match world.joints[joint_index as usize].type_ {
        JointType::Distance => 0.0,
        JointType::Motor => get_motor_joint_torque(world, base),
        JointType::Filter => 0.0,
        JointType::Prismatic => get_prismatic_joint_torque(world, base),
        JointType::Revolute => get_revolute_joint_torque(world, base),
        JointType::Weld => get_weld_joint_torque(world, base),
        JointType::Wheel => get_wheel_joint_torque(world, base),
    }
}
