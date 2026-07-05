// Joint creation from joint.c: the shared b2CreateJoint machinery and the
// per-type b2Create*Joint constructors.
//
// The C b2CreateJoint returns a (b2Joint*, b2JointSim*) pair; the Rust port
// returns the raw joint index and callers re-fetch the sim through
// get_joint_sim, which also survives the solver-set merge that orphans the C
// pointer. The C memsets the sim union to zero; the Rust payload enum is
// reset with the per-type Default, whose transient fields (indices, frames,
// softness) are all overwritten by prepare before use.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::*;
use crate::body::get_body_full_id;
use crate::constants::linear_slop;
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::island::link_joint;
use crate::math_functions::{
    clamp_float, is_valid_float, is_valid_transform, max_float, max_int, PI,
};
use crate::solver_set::{
    merge_solver_sets, wake_solver_set, AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET, STATIC_SET,
};
use crate::types::{
    BodyType, DistanceJointDef, FilterJointDef, JointDef, MotorJointDef, PrismaticJointDef,
    RevoluteJointDef, WeldJointDef, WheelJointDef,
};
use crate::world::World;

/// An empty payload of the given type, standing in for the C memset of the
/// union plus the type tag assignment.
fn empty_payload(joint_type: JointType) -> JointPayload {
    match joint_type {
        JointType::Distance => JointPayload::Distance(DistanceJoint::default()),
        JointType::Filter => JointPayload::Filter,
        JointType::Motor => JointPayload::Motor(MotorJoint::default()),
        JointType::Prismatic => JointPayload::Prismatic(PrismaticJoint::default()),
        JointType::Revolute => JointPayload::Revolute(RevoluteJoint::default()),
        JointType::Weld => JointPayload::Weld(WeldJoint::default()),
        JointType::Wheel => JointPayload::Wheel(WheelJoint::default()),
    }
}

/// (static b2DestroyContactsBetweenBodies)
pub(crate) fn destroy_contacts_between_bodies(world: &mut World, body_id_a: i32, body_id_b: i32) {
    // use the smaller of the two contact lists
    let (mut contact_key, other_body_id) = {
        let body_a = &world.bodies[body_id_a as usize];
        let body_b = &world.bodies[body_id_b as usize];
        if body_a.contact_count < body_b.contact_count {
            (body_a.head_contact_key, body_b.id)
        } else {
            (body_b.head_contact_key, body_a.id)
        }
    };

    // no need to wake bodies when a joint removes collision between them
    let wake_bodies = false;

    // destroy the contacts
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        contact_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;

        let other_edge_index = edge_index ^ 1;
        if world.contacts[contact_id as usize].edges[other_edge_index as usize].body_id
            == other_body_id
        {
            // Careful, this removes the contact from the current doubly linked
            // list
            crate::contact::destroy_contact(world, contact_id, wake_bodies);
        }
    }

    world.validate_solver_sets();
}

/// Shared joint creation. Returns the raw joint index; the per-type
/// constructors fill the payload through get_joint_sim. (static b2CreateJoint)
pub(crate) fn create_joint(world: &mut World, def: &JointDef, joint_type: JointType) -> i32 {
    debug_assert!(is_valid_transform(def.local_frame_a));
    debug_assert!(is_valid_transform(def.local_frame_b));
    debug_assert!(def.body_id_a != def.body_id_b);

    let body_id_a = get_body_full_id(world, def.body_id_a);
    let body_id_b = get_body_full_id(world, def.body_id_b);
    let max_set_index = max_int(
        world.bodies[body_id_a as usize].set_index,
        world.bodies[body_id_b as usize].set_index,
    );

    // Create joint id and joint
    let joint_id = world.joint_id_pool.alloc_id();
    if joint_id == world.joints.len() as i32 {
        world.joints.push(Joint::default());
    }

    {
        let joint = &mut world.joints[joint_id as usize];
        joint.joint_id = joint_id;
        joint.user_data = def.user_data;
        joint.generation = joint.generation.wrapping_add(1);
        joint.set_index = NULL_INDEX;
        joint.color_index = NULL_INDEX;
        joint.local_index = NULL_INDEX;
        joint.island_id = NULL_INDEX;
        joint.island_index = NULL_INDEX;
        joint.draw_scale = def.draw_scale;
        joint.type_ = joint_type;
        joint.collide_connected = def.collide_connected;
    }

    // Doubly linked list on bodyA
    {
        let head_joint_key = world.bodies[body_id_a as usize].head_joint_key;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.edges[0].body_id = body_id_a;
            joint.edges[0].prev_key = NULL_INDEX;
            joint.edges[0].next_key = head_joint_key;
        }

        let key_a = joint_id << 1;
        if head_joint_key != NULL_INDEX {
            let head_joint = &mut world.joints[(head_joint_key >> 1) as usize];
            head_joint.edges[(head_joint_key & 1) as usize].prev_key = key_a;
        }
        let body_a = &mut world.bodies[body_id_a as usize];
        body_a.head_joint_key = key_a;
        body_a.joint_count += 1;
    }

    // Doubly linked list on bodyB
    {
        let head_joint_key = world.bodies[body_id_b as usize].head_joint_key;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.edges[1].body_id = body_id_b;
            joint.edges[1].prev_key = NULL_INDEX;
            joint.edges[1].next_key = head_joint_key;
        }

        let key_b = (joint_id << 1) | 1;
        if head_joint_key != NULL_INDEX {
            let head_joint = &mut world.joints[(head_joint_key >> 1) as usize];
            head_joint.edges[(head_joint_key & 1) as usize].prev_key = key_b;
        }
        let body_b = &mut world.bodies[body_id_b as usize];
        body_b.head_joint_key = key_b;
        body_b.joint_count += 1;
    }

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;
    let type_a = world.bodies[body_id_a as usize].type_;
    let type_b = world.bodies[body_id_b as usize].type_;

    if set_a == DISABLED_SET || set_b == DISABLED_SET {
        // if either body is disabled, create in disabled set
        let local_index = world.solver_sets[DISABLED_SET as usize].joint_sims.len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = DISABLED_SET;
            joint.local_index = local_index;
        }

        let joint_sim = JointSim {
            joint_id,
            body_id_a,
            body_id_b,
            payload: empty_payload(joint_type),
            ..JointSim::default()
        };
        world.solver_sets[DISABLED_SET as usize]
            .joint_sims
            .push(joint_sim);
    } else if type_a != BodyType::Dynamic && type_b != BodyType::Dynamic {
        // joint is not attached to a dynamic body
        let local_index = world.solver_sets[STATIC_SET as usize].joint_sims.len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = STATIC_SET;
            joint.local_index = local_index;
        }

        let joint_sim = JointSim {
            joint_id,
            body_id_a,
            body_id_b,
            payload: empty_payload(joint_type),
            ..JointSim::default()
        };
        world.solver_sets[STATIC_SET as usize]
            .joint_sims
            .push(joint_sim);
    } else if set_a == AWAKE_SET || set_b == AWAKE_SET {
        // if either body is sleeping, wake it
        if max_set_index >= FIRST_SLEEPING_SET {
            wake_solver_set(world, max_set_index);
        }

        world.joints[joint_id as usize].set_index = AWAKE_SET;

        let (color_index, local_index) =
            crate::constraint_graph::create_joint_in_graph(world, joint_id);
        {
            let joint_sim = &mut world.constraint_graph.colors[color_index as usize].joint_sims
                [local_index as usize];
            joint_sim.joint_id = joint_id;
            joint_sim.body_id_a = body_id_a;
            joint_sim.body_id_b = body_id_b;
            joint_sim.payload = empty_payload(joint_type);
        }
    } else {
        // joint connected between sleeping and/or static bodies
        debug_assert!(set_a >= FIRST_SLEEPING_SET || set_b >= FIRST_SLEEPING_SET);
        debug_assert!(set_a != STATIC_SET || set_b != STATIC_SET);

        // joint should go into the sleeping set (not static set)
        let set_index = max_set_index;

        let local_index = world.solver_sets[set_index as usize].joint_sims.len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = set_index;
            joint.local_index = local_index;
        }

        // These must be set to accommodate the merge below
        let joint_sim = JointSim {
            joint_id,
            body_id_a,
            body_id_b,
            payload: empty_payload(joint_type),
            ..JointSim::default()
        };
        world.solver_sets[set_index as usize]
            .joint_sims
            .push(joint_sim);

        if set_a != set_b && set_a >= FIRST_SLEEPING_SET && set_b >= FIRST_SLEEPING_SET {
            // merge sleeping sets. The C jointSim pointer is orphaned here;
            // the Rust port re-fetches through get_joint_sim below.
            merge_solver_sets(world, set_a, set_b);
            debug_assert!(
                world.bodies[body_id_a as usize].set_index
                    == world.bodies[body_id_b as usize].set_index
            );
        }
    }

    debug_assert!(is_valid_float(def.force_threshold) && def.force_threshold >= 0.0);
    debug_assert!(is_valid_float(def.torque_threshold) && def.torque_threshold >= 0.0);

    {
        let joint_sim = get_joint_sim(world, joint_id);
        joint_sim.local_frame_a = def.local_frame_a;
        joint_sim.local_frame_b = def.local_frame_b;
        joint_sim.constraint_hertz = def.constraint_hertz;
        joint_sim.constraint_damping_ratio = def.constraint_damping_ratio;
        joint_sim.constraint_softness = crate::solver::Softness {
            bias_rate: 0.0,
            mass_scale: 1.0,
            impulse_scale: 0.0,
        };
        joint_sim.force_threshold = def.force_threshold;
        joint_sim.torque_threshold = def.torque_threshold;

        debug_assert!(joint_sim.joint_id == joint_id);
        debug_assert!(joint_sim.body_id_a == body_id_a);
        debug_assert!(joint_sim.body_id_b == body_id_b);
    }

    if world.joints[joint_id as usize].set_index > DISABLED_SET {
        // Add edge to island graph
        link_joint(world, joint_id);
    }

    // If the joint prevents collisions, then destroy all contacts between
    // attached bodies
    if !def.collide_connected {
        destroy_contacts_between_bodies(world, body_id_a, body_id_b);
    }

    world.validate_solver_sets();

    joint_id
}

/// Build the public JointId for a raw joint index.
pub(crate) fn make_joint_id(world: &World, joint_id: i32) -> JointId {
    JointId {
        index1: joint_id + 1,
        world0: world.world_id,
        generation: world.joints[joint_id as usize].generation,
    }
}

/// (b2CreateDistanceJoint)
pub fn create_distance_joint(world: &mut World, def: &DistanceJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(is_valid_float(def.length) && def.length > 0.0);
    debug_assert!(def.lower_spring_force <= def.upper_spring_force);

    let joint_id = create_joint(world, &def.base, JointType::Distance);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.distance_mut();
    *joint = DistanceJoint::default();
    joint.length = max_float(def.length, linear_slop());
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.min_length = max_float(def.min_length, linear_slop());
    joint.max_length = max_float(def.min_length, def.max_length);
    joint.max_motor_force = def.max_motor_force;
    joint.motor_speed = def.motor_speed;
    joint.enable_spring = def.enable_spring;
    joint.lower_spring_force = def.lower_spring_force;
    joint.upper_spring_force = def.upper_spring_force;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;
    joint.impulse = 0.0;
    joint.lower_impulse = 0.0;
    joint.upper_impulse = 0.0;
    joint.motor_impulse = 0.0;

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_DISTANCE_JOINT,
            |buf| crate::recording::rec_w_distancejointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2CreateMotorJoint)
pub fn create_motor_joint(world: &mut World, def: &MotorJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);

    let joint_id = create_joint(world, &def.base, JointType::Motor);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.motor_mut();
    *joint = MotorJoint::default();
    joint.linear_velocity = def.linear_velocity;
    joint.max_velocity_force = def.max_velocity_force;
    joint.angular_velocity = def.angular_velocity;
    joint.max_velocity_torque = def.max_velocity_torque;
    joint.linear_hertz = def.linear_hertz;
    joint.linear_damping_ratio = def.linear_damping_ratio;
    joint.max_spring_force = def.max_spring_force;
    joint.angular_hertz = def.angular_hertz;
    joint.angular_damping_ratio = def.angular_damping_ratio;
    joint.max_spring_torque = def.max_spring_torque;

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_MOTOR_JOINT,
            |buf| crate::recording::rec_w_motorjointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2CreateFilterJoint)
pub fn create_filter_joint(world: &mut World, def: &FilterJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);

    let joint_id = create_joint(world, &def.base, JointType::Filter);

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_FILTER_JOINT,
            |buf| crate::recording::rec_w_filterjointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2CreatePrismaticJoint)
pub fn create_prismatic_joint(world: &mut World, def: &PrismaticJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(def.lower_translation <= def.upper_translation);

    let joint_id = create_joint(world, &def.base, JointType::Prismatic);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.prismatic_mut();
    *joint = PrismaticJoint::default();
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.target_translation = def.target_translation;
    joint.lower_translation = def.lower_translation;
    joint.upper_translation = def.upper_translation;
    joint.max_motor_force = def.max_motor_force;
    joint.motor_speed = def.motor_speed;
    joint.enable_spring = def.enable_spring;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_PRISMATIC_JOINT,
            |buf| crate::recording::rec_w_prismaticjointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2CreateRevoluteJoint)
pub fn create_revolute_joint(world: &mut World, def: &RevoluteJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(def.lower_angle <= def.upper_angle);
    debug_assert!(def.lower_angle >= -0.99 * PI);
    debug_assert!(def.upper_angle <= 0.99 * PI);

    let joint_id = create_joint(world, &def.base, JointType::Revolute);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.revolute_mut();
    *joint = RevoluteJoint::default();
    joint.target_angle = clamp_float(def.target_angle, -PI, PI);
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.lower_angle = def.lower_angle;
    joint.upper_angle = def.upper_angle;
    joint.max_motor_torque = def.max_motor_torque;
    joint.motor_speed = def.motor_speed;
    joint.enable_spring = def.enable_spring;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_REVOLUTE_JOINT,
            |buf| crate::recording::rec_w_revolutejointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2CreateWeldJoint)
pub fn create_weld_joint(world: &mut World, def: &WeldJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);

    let joint_id = create_joint(world, &def.base, JointType::Weld);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.weld_mut();
    *joint = WeldJoint::default();
    joint.linear_hertz = def.linear_hertz;
    joint.linear_damping_ratio = def.linear_damping_ratio;
    joint.angular_hertz = def.angular_hertz;
    joint.angular_damping_ratio = def.angular_damping_ratio;

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_WELD_JOINT,
            |buf| crate::recording::rec_w_weldjointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2CreateWheelJoint)
pub fn create_wheel_joint(world: &mut World, def: &WheelJointDef) -> JointId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(def.lower_translation <= def.upper_translation);

    let joint_id = create_joint(world, &def.base, JointType::Wheel);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.wheel_mut();
    *joint = WheelJoint::default();
    joint.lower_translation = def.lower_translation;
    joint.upper_translation = def.upper_translation;
    joint.max_motor_torque = def.max_motor_torque;
    joint.motor_speed = def.motor_speed;
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.enable_spring = def.enable_spring;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;

    let id = make_joint_id(world, joint_id);

    // (B2_REC_CREATE)
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_joint(
            rec,
            crate::recording::OP_CREATE_WHEEL_JOINT,
            |buf| crate::recording::rec_w_wheeljointdef(buf, def),
            id,
        )
    });

    id
}

/// (b2DestroyJoint)
pub fn destroy_joint(world: &mut World, joint_id: JointId, wake_attached: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_destroy_joint(rec, joint_id, wake_attached)
    });
    let joint_index = get_joint_full_id(world, joint_id);
    destroy_joint_internal(world, joint_index, wake_attached);
}
