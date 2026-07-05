// Debug draw dispatch for joints (b2DrawJoint in joint.c).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::get_body_transform_quick;
use crate::core::NULL_INDEX;
use crate::debug_draw::{DebugDraw, HexColor};
use crate::joint::solve::{get_joint_constraint_force, get_joint_constraint_torque};
use crate::joint::{get_joint_sim_ref, Joint, JointType};
use crate::math_functions::{lerp_position, max_float, mul_sv, offset_pos, transform_world_point};
use crate::solver_set::DISABLED_SET;
use crate::world::World;

/// (b2DrawJoint)
pub fn draw_joint(draw: &mut dyn DebugDraw, world: &World, joint: &Joint) {
    let body_a = &world.bodies[joint.edges[0].body_id as usize];
    let body_b = &world.bodies[joint.edges[1].body_id as usize];
    if body_a.set_index == DISABLED_SET || body_b.set_index == DISABLED_SET {
        return;
    }

    let joint_sim = get_joint_sim_ref(world, joint.joint_id);

    let xf_a = get_body_transform_quick(world, body_a);
    let xf_b = get_body_transform_quick(world, body_b);

    let p_a = transform_world_point(xf_a, joint_sim.local_frame_a.p);
    let p_b = transform_world_point(xf_b, joint_sim.local_frame_b.p);

    let scale = max_float(0.0001, draw.joint_scale() * joint.draw_scale);

    match joint.type_ {
        JointType::Distance => {
            crate::distance_joint::draw_distance_joint(draw, joint_sim, xf_a, xf_b);
        }

        JointType::Filter => {
            draw.draw_line(p_a, p_b, HexColor::GOLD);
        }

        JointType::Motor => {
            draw.draw_point(p_a, 8.0, HexColor::YELLOW_GREEN);
            draw.draw_point(p_b, 8.0, HexColor::PLUM);
            draw.draw_line(p_a, p_b, HexColor::LIGHT_GRAY);
        }

        JointType::Prismatic => {
            crate::prismatic_joint::draw_prismatic_joint(draw, joint_sim, xf_a, xf_b, scale);
        }

        JointType::Revolute => {
            crate::revolute_joint::draw_revolute_joint(draw, joint_sim, xf_a, xf_b, scale);
        }

        JointType::Weld => {
            crate::weld_joint::draw_weld_joint(draw, joint_sim, xf_a, xf_b, scale);
        }

        JointType::Wheel => {
            crate::wheel_joint::draw_wheel_joint(draw, joint_sim, xf_a, xf_b, scale);
        }
    }

    // The C switch has a default arm sketching three b2_colorDarkSeaGreen
    // lines; JointType is exhaustive in Rust so it is unreachable here.

    if draw.draw_graph_colors() {
        let color_index = joint.color_index;
        if color_index != NULL_INDEX {
            let p = lerp_position(p_a, p_b, 0.5);
            draw.draw_point(
                p,
                5.0,
                crate::constraint_graph::get_graph_color(color_index),
            );
        }
    }

    if draw.draw_joint_extras() {
        let joint_index = joint.joint_id;
        let force = get_joint_constraint_force(world, joint_index);
        let torque = get_joint_constraint_torque(world, joint_index);
        let p = lerp_position(p_a, p_b, 0.5);

        draw.draw_line(p, offset_pos(p, mul_sv(0.001, force)), HexColor::AZURE);

        let buffer = format!("f = [{}, {}], t = {}", force.x, force.y, torque);
        draw.draw_string(p, &buffer, HexColor::AZURE);
    }
}
