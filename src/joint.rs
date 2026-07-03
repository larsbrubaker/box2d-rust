// Port of the joint data model from box2d-cpp-reference/src/joint.h.
// Logic from joint.c and the per-joint .c files lands in the joints bring-up
// commits.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::math_functions::{Mat22, Transform, Vec2, MAT22_ZERO, TRANSFORM_IDENTITY, VEC2_ZERO};
use crate::solver::Softness;

/// Joint type enumeration. (types.h: b2JointType)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JointType {
    #[default]
    Distance,
    Filter,
    Motor,
    Prismatic,
    Revolute,
    Weld,
    Wheel,
}

/// A joint edge connects bodies and joints together in a joint graph where
/// each body is a node and each joint is an edge. (b2JointEdge)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JointEdge {
    pub body_id: i32,
    pub prev_key: i32,
    pub next_key: i32,
}

impl Default for JointEdge {
    fn default() -> Self {
        JointEdge {
            body_id: NULL_INDEX,
            prev_key: NULL_INDEX,
            next_key: NULL_INDEX,
        }
    }
}

/// Map from JointId to joint data in the solver sets. (b2Joint)
#[derive(Debug, Clone)]
pub struct Joint {
    pub user_data: u64,

    /// index of simulation set stored in World. NULL_INDEX when slot is free.
    pub set_index: i32,

    /// index into the constraint graph color array, may be NULL_INDEX for
    /// sleeping/disabled joints. NULL_INDEX when slot is free.
    pub color_index: i32,

    /// joint index within set or graph color. NULL_INDEX when slot is free.
    pub local_index: i32,

    pub edges: [JointEdge; 2],

    pub joint_id: i32,
    pub island_id: i32,

    /// Index into the island's joints array for O(1) swap-removal.
    /// NULL_INDEX when not in an island.
    pub island_index: i32,

    pub draw_scale: f32,

    pub type_: JointType,

    /// Monotonically advanced when a joint is allocated in this slot.
    pub generation: u16,

    pub collide_connected: bool,
}

impl Default for Joint {
    fn default() -> Self {
        Joint {
            user_data: 0,
            set_index: NULL_INDEX,
            color_index: NULL_INDEX,
            local_index: NULL_INDEX,
            edges: [JointEdge::default(); 2],
            joint_id: NULL_INDEX,
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            draw_scale: 1.0,
            type_: JointType::Distance,
            generation: 0,
            collide_connected: false,
        }
    }
}

/// (b2DistanceJoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DistanceJoint {
    pub length: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub min_length: f32,
    pub max_length: f32,

    pub max_motor_force: f32,
    pub motor_speed: f32,

    pub impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub motor_impulse: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub anchor_a: Vec2,
    pub anchor_b: Vec2,
    pub delta_center: Vec2,
    pub distance_softness: Softness,
    pub axial_mass: f32,

    pub enable_spring: bool,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

/// (b2MotorJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorJoint {
    pub linear_velocity: Vec2,
    pub max_velocity_force: f32,
    pub angular_velocity: f32,
    pub max_velocity_torque: f32,
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub max_spring_force: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,
    pub max_spring_torque: f32,

    pub linear_velocity_impulse: Vec2,
    pub angular_velocity_impulse: f32,
    pub linear_spring_impulse: Vec2,
    pub angular_spring_impulse: f32,

    pub linear_spring: Softness,
    pub angular_spring: Softness,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec2,
    pub linear_mass: Mat22,
    pub angular_mass: f32,
}

impl Default for MotorJoint {
    fn default() -> Self {
        MotorJoint {
            linear_velocity: VEC2_ZERO,
            max_velocity_force: 0.0,
            angular_velocity: 0.0,
            max_velocity_torque: 0.0,
            linear_hertz: 0.0,
            linear_damping_ratio: 0.0,
            max_spring_force: 0.0,
            angular_hertz: 0.0,
            angular_damping_ratio: 0.0,
            max_spring_torque: 0.0,
            linear_velocity_impulse: VEC2_ZERO,
            angular_velocity_impulse: 0.0,
            linear_spring_impulse: VEC2_ZERO,
            angular_spring_impulse: 0.0,
            linear_spring: Softness::default(),
            angular_spring: Softness::default(),
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC2_ZERO,
            linear_mass: MAT22_ZERO,
            angular_mass: 0.0,
        }
    }
}

/// (b2PrismaticJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrismaticJoint {
    pub impulse: Vec2,
    pub spring_impulse: f32,
    pub motor_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_translation: f32,
    pub max_motor_force: f32,
    pub motor_speed: f32,
    pub lower_translation: f32,
    pub upper_translation: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec2,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

impl Default for PrismaticJoint {
    fn default() -> Self {
        PrismaticJoint {
            impulse: VEC2_ZERO,
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            target_translation: 0.0,
            max_motor_force: 0.0,
            motor_speed: 0.0,
            lower_translation: 0.0,
            upper_translation: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC2_ZERO,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_limit: false,
            enable_motor: false,
        }
    }
}

/// (b2RevoluteJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoluteJoint {
    pub linear_impulse: Vec2,
    pub spring_impulse: f32,
    pub motor_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_angle: f32,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
    pub lower_angle: f32,
    pub upper_angle: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec2,
    pub axial_mass: f32,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_motor: bool,
    pub enable_limit: bool,
}

impl Default for RevoluteJoint {
    fn default() -> Self {
        RevoluteJoint {
            linear_impulse: VEC2_ZERO,
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            target_angle: 0.0,
            max_motor_torque: 0.0,
            motor_speed: 0.0,
            lower_angle: 0.0,
            upper_angle: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC2_ZERO,
            axial_mass: 0.0,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_motor: false,
            enable_limit: false,
        }
    }
}

/// (b2WeldJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WeldJoint {
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,

    pub linear_spring: Softness,
    pub angular_spring: Softness,
    pub linear_impulse: Vec2,
    pub angular_impulse: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec2,
    pub axial_mass: f32,
}

impl Default for WeldJoint {
    fn default() -> Self {
        WeldJoint {
            linear_hertz: 0.0,
            linear_damping_ratio: 0.0,
            angular_hertz: 0.0,
            angular_damping_ratio: 0.0,
            linear_spring: Softness::default(),
            angular_spring: Softness::default(),
            linear_impulse: VEC2_ZERO,
            angular_impulse: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC2_ZERO,
            axial_mass: 0.0,
        }
    }
}

/// (b2WheelJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelJoint {
    pub perp_impulse: f32,
    pub motor_impulse: f32,
    pub spring_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
    pub lower_translation: f32,
    pub upper_translation: f32,
    pub hertz: f32,
    pub damping_ratio: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec2,
    pub perp_mass: f32,
    pub motor_mass: f32,
    pub axial_mass: f32,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_motor: bool,
    pub enable_limit: bool,
}

impl Default for WheelJoint {
    fn default() -> Self {
        WheelJoint {
            perp_impulse: 0.0,
            motor_impulse: 0.0,
            spring_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            max_motor_torque: 0.0,
            motor_speed: 0.0,
            lower_translation: 0.0,
            upper_translation: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC2_ZERO,
            perp_mass: 0.0,
            motor_mass: 0.0,
            axial_mass: 0.0,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_motor: false,
            enable_limit: false,
        }
    }
}

/// The per-type joint payload. The C `b2JointSim` stores a `b2JointType type`
/// tag plus a union; the Rust port stores this tagged enum. A filter joint has
/// no simulation data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointPayload {
    Distance(DistanceJoint),
    Filter,
    Motor(MotorJoint),
    Prismatic(PrismaticJoint),
    Revolute(RevoluteJoint),
    Weld(WeldJoint),
    Wheel(WheelJoint),
}

impl JointPayload {
    /// The joint type tag for this payload.
    pub fn joint_type(&self) -> JointType {
        match self {
            JointPayload::Distance(_) => JointType::Distance,
            JointPayload::Filter => JointType::Filter,
            JointPayload::Motor(_) => JointType::Motor,
            JointPayload::Prismatic(_) => JointType::Prismatic,
            JointPayload::Revolute(_) => JointType::Revolute,
            JointPayload::Weld(_) => JointType::Weld,
            JointPayload::Wheel(_) => JointType::Wheel,
        }
    }
}

/// The base joint simulation data. (b2JointSim)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointSim {
    pub joint_id: i32,

    pub body_id_a: i32,
    pub body_id_b: i32,

    pub local_frame_a: Transform,
    pub local_frame_b: Transform,

    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_i_a: f32,
    pub inv_i_b: f32,

    pub constraint_hertz: f32,
    pub constraint_damping_ratio: f32,

    pub constraint_softness: Softness,

    pub force_threshold: f32,
    pub torque_threshold: f32,

    /// The per-type data (C: type tag + union).
    pub payload: JointPayload,
}

impl JointSim {
    /// The joint type tag. (C: joint->type)
    pub fn joint_type(&self) -> JointType {
        self.payload.joint_type()
    }
}

impl Default for JointSim {
    fn default() -> Self {
        JointSim {
            joint_id: NULL_INDEX,
            body_id_a: NULL_INDEX,
            body_id_b: NULL_INDEX,
            local_frame_a: TRANSFORM_IDENTITY,
            local_frame_b: TRANSFORM_IDENTITY,
            inv_mass_a: 0.0,
            inv_mass_b: 0.0,
            inv_i_a: 0.0,
            inv_i_b: 0.0,
            constraint_hertz: 0.0,
            constraint_damping_ratio: 0.0,
            constraint_softness: Softness::default(),
            force_threshold: 0.0,
            torque_threshold: 0.0,
            payload: JointPayload::Distance(DistanceJoint::default()),
        }
    }
}

/// Destroy a joint: unlink it from both bodies' joint lists, the island
/// graph, and the solver set or constraint graph that owns its sim, then free
/// the id. (b2DestroyJointInternal — C takes the joint pointer; the Rust port
/// takes the id.)
pub fn destroy_joint_internal(world: &mut crate::world::World, joint_id: i32, wake_bodies: bool) {
    use crate::solver_set::{AWAKE_SET, DISABLED_SET};

    let (edge_a, edge_b) = {
        let joint = &world.joints[joint_id as usize];
        (joint.edges[0], joint.edges[1])
    };

    let id_a = edge_a.body_id;
    let id_b = edge_b.body_id;

    // Remove from body A
    if edge_a.prev_key != NULL_INDEX {
        let prev_joint = &mut world.joints[(edge_a.prev_key >> 1) as usize];
        prev_joint.edges[(edge_a.prev_key & 1) as usize].next_key = edge_a.next_key;
    }

    if edge_a.next_key != NULL_INDEX {
        let next_joint = &mut world.joints[(edge_a.next_key >> 1) as usize];
        next_joint.edges[(edge_a.next_key & 1) as usize].prev_key = edge_a.prev_key;
    }

    let edge_key_a = joint_id << 1;
    {
        let body_a = &mut world.bodies[id_a as usize];
        if body_a.head_joint_key == edge_key_a {
            body_a.head_joint_key = edge_a.next_key;
        }
        body_a.joint_count -= 1;
    }

    // Remove from body B
    if edge_b.prev_key != NULL_INDEX {
        let prev_joint = &mut world.joints[(edge_b.prev_key >> 1) as usize];
        prev_joint.edges[(edge_b.prev_key & 1) as usize].next_key = edge_b.next_key;
    }

    if edge_b.next_key != NULL_INDEX {
        let next_joint = &mut world.joints[(edge_b.next_key >> 1) as usize];
        next_joint.edges[(edge_b.next_key & 1) as usize].prev_key = edge_b.prev_key;
    }

    let edge_key_b = (joint_id << 1) | 1;
    {
        let body_b = &mut world.bodies[id_b as usize];
        if body_b.head_joint_key == edge_key_b {
            body_b.head_joint_key = edge_b.next_key;
        }
        body_b.joint_count -= 1;
    }

    if world.joints[joint_id as usize].island_id != NULL_INDEX {
        debug_assert!(world.joints[joint_id as usize].set_index > DISABLED_SET);
        crate::island::unlink_joint(world, joint_id);
    } else {
        debug_assert!(world.joints[joint_id as usize].set_index <= DISABLED_SET);
    }

    // Remove joint from solver set that owns it
    let (set_index, local_index, color_index) = {
        let joint = &world.joints[joint_id as usize];
        (joint.set_index, joint.local_index, joint.color_index)
    };

    if set_index == AWAKE_SET {
        crate::constraint_graph::remove_joint_from_graph(
            world,
            id_a,
            id_b,
            color_index,
            local_index,
        );
    } else {
        let set = &mut world.solver_sets[set_index as usize];
        let moved_index = set.joint_sims.len() as i32 - 1;
        set.joint_sims.swap_remove(local_index as usize);
        if moved_index != local_index {
            // Fix moved joint
            let moved_id =
                world.solver_sets[set_index as usize].joint_sims[local_index as usize].joint_id;
            let moved_joint = &mut world.joints[moved_id as usize];
            debug_assert!(moved_joint.local_index == moved_index);
            moved_joint.local_index = local_index;
        }
    }

    // Free joint and id (preserve joint generation)
    {
        let joint = &mut world.joints[joint_id as usize];
        joint.set_index = NULL_INDEX;
        joint.local_index = NULL_INDEX;
        joint.color_index = NULL_INDEX;
        joint.joint_id = NULL_INDEX;
    }
    world.joint_id_pool.free_id(joint_id);

    if wake_bodies {
        crate::body::wake_body(world, id_a);
        crate::body::wake_body(world, id_b);
    }

    world.validate_solver_sets();
}
