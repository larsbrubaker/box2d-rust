// Joint definition types from include/box2d/types.h and their b2Default*
// constructors from src/joint.c.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::huge;
use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
use crate::id::BodyId;
use crate::math_functions::{Transform, Vec2, ROT_IDENTITY, TRANSFORM_IDENTITY, VEC2_ZERO};

/// Base joint definition used by all joint types.
/// The local frames are measured from the body's origin rather than the center
/// of mass because:
/// 1. you might not know where the center of mass will be
/// 2. if you add/remove shapes from a body and recompute the mass, the joints
///    will be broken
///
/// (b2JointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointDef {
    /// User data
    pub user_data: u64,

    /// The first attached body
    pub body_id_a: BodyId,

    /// The second attached body
    pub body_id_b: BodyId,

    /// The first local joint frame
    pub local_frame_a: Transform,

    /// The second local joint frame
    pub local_frame_b: Transform,

    /// Force threshold for joint events
    pub force_threshold: f32,

    /// Torque threshold for joint events
    pub torque_threshold: f32,

    /// Constraint hertz (advanced feature)
    pub constraint_hertz: f32,

    /// Constraint damping ratio (advanced feature)
    pub constraint_damping_ratio: f32,

    /// Debug draw scale
    pub draw_scale: f32,

    /// Set this flag to true if the attached bodies should collide
    pub collide_connected: bool,
}

/// (static b2DefaultJointDef)
pub(crate) fn default_joint_def() -> JointDef {
    JointDef {
        user_data: 0,
        body_id_a: BodyId::default(),
        body_id_b: BodyId::default(),
        local_frame_a: Transform {
            q: ROT_IDENTITY,
            ..TRANSFORM_IDENTITY
        },
        local_frame_b: Transform {
            q: ROT_IDENTITY,
            ..TRANSFORM_IDENTITY
        },
        force_threshold: f32::MAX,
        torque_threshold: f32::MAX,
        constraint_hertz: 60.0,
        constraint_damping_ratio: 2.0,
        draw_scale: get_length_units_per_meter(),
        collide_connected: false,
    }
}

/// Distance joint definition.
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs. (b2DistanceJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// The rest length of this joint. Clamped to a stable minimum value.
    pub length: f32,

    /// Enable the distance constraint to behave like a spring. If false then
    /// the distance joint will be rigid, overriding the limit and motor.
    pub enable_spring: bool,

    /// The lower spring force controls how much tension it can sustain
    pub lower_spring_force: f32,

    /// The upper spring force controls how much compression it an sustain
    pub upper_spring_force: f32,

    /// The spring linear stiffness Hertz, cycles per second
    pub hertz: f32,

    /// The spring linear damping ratio, non-dimensional
    pub damping_ratio: f32,

    /// Enable/disable the joint limit
    pub enable_limit: bool,

    /// Minimum length for limit. Clamped to a stable minimum value.
    pub min_length: f32,

    /// Maximum length for limit. Must be greater than or equal to the minimum
    /// length.
    pub max_length: f32,

    /// Enable/disable the joint motor
    pub enable_motor: bool,

    /// The maximum motor force, usually in newtons
    pub max_motor_force: f32,

    /// The desired motor speed, usually in meters per second
    pub motor_speed: f32,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultDistanceJointDef)
pub fn default_distance_joint_def() -> DistanceJointDef {
    DistanceJointDef {
        base: default_joint_def(),
        length: 1.0,
        enable_spring: false,
        lower_spring_force: -f32::MAX,
        upper_spring_force: f32::MAX,
        hertz: 0.0,
        damping_ratio: 0.0,
        enable_limit: false,
        min_length: 0.0,
        max_length: huge(),
        enable_motor: false,
        max_motor_force: 0.0,
        motor_speed: 0.0,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for DistanceJointDef {
    fn default() -> Self {
        default_distance_joint_def()
    }
}

/// A motor joint is used to control the relative velocity and or transform
/// between two bodies. With a velocity of zero this acts like top-down
/// friction. (b2MotorJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// The desired linear velocity
    pub linear_velocity: Vec2,

    /// The maximum motor force in newtons
    pub max_velocity_force: f32,

    /// The desired angular velocity
    pub angular_velocity: f32,

    /// The maximum motor torque in newton-meters
    pub max_velocity_torque: f32,

    /// Linear spring hertz for position control
    pub linear_hertz: f32,

    /// Linear spring damping ratio
    pub linear_damping_ratio: f32,

    /// Maximum spring force in newtons
    pub max_spring_force: f32,

    /// Angular spring hertz for position control
    pub angular_hertz: f32,

    /// Angular spring damping ratio
    pub angular_damping_ratio: f32,

    /// Maximum spring torque in newton-meters
    pub max_spring_torque: f32,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultMotorJointDef)
pub fn default_motor_joint_def() -> MotorJointDef {
    MotorJointDef {
        base: default_joint_def(),
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
        internal_value: SECRET_COOKIE,
    }
}

impl Default for MotorJointDef {
    fn default() -> Self {
        default_motor_joint_def()
    }
}

/// A filter joint is used to disable collision between two specific bodies.
/// (b2FilterJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilterJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultFilterJointDef)
pub fn default_filter_joint_def() -> FilterJointDef {
    FilterJointDef {
        base: default_joint_def(),
        internal_value: SECRET_COOKIE,
    }
}

impl Default for FilterJointDef {
    fn default() -> Self {
        default_filter_joint_def()
    }
}

/// Prismatic joint definition.
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate
/// relative to body A. The joint translation is zero when the local frame
/// origins coincide in world space. (b2PrismaticJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrismaticJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// Enable a linear spring along the prismatic joint axis
    pub enable_spring: bool,

    /// The spring stiffness Hertz, cycles per second
    pub hertz: f32,

    /// The spring damping ratio, non-dimensional
    pub damping_ratio: f32,

    /// The target translation for the joint in meters. The spring-damper will
    /// drive to this translation.
    pub target_translation: f32,

    /// Enable/disable the joint limit
    pub enable_limit: bool,

    /// The lower translation limit
    pub lower_translation: f32,

    /// The upper translation limit
    pub upper_translation: f32,

    /// Enable/disable the joint motor
    pub enable_motor: bool,

    /// The maximum motor force, typically in newtons
    pub max_motor_force: f32,

    /// The desired motor speed, typically in meters per second
    pub motor_speed: f32,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultPrismaticJointDef)
pub fn default_prismatic_joint_def() -> PrismaticJointDef {
    PrismaticJointDef {
        base: default_joint_def(),
        enable_spring: false,
        hertz: 0.0,
        damping_ratio: 0.0,
        target_translation: 0.0,
        enable_limit: false,
        lower_translation: 0.0,
        upper_translation: 0.0,
        enable_motor: false,
        max_motor_force: 0.0,
        motor_speed: 0.0,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for PrismaticJointDef {
    fn default() -> Self {
        default_prismatic_joint_def()
    }
}

/// Revolute joint definition.
/// A point on body B is fixed to a point on body A. Allows relative rotation.
/// (b2RevoluteJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoluteJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// The target angle for the joint in radians. The spring-damper will
    /// drive to this angle.
    pub target_angle: f32,

    /// Enable a rotational spring on the revolute hinge axis
    pub enable_spring: bool,

    /// The spring stiffness Hertz, cycles per second
    pub hertz: f32,

    /// The spring damping ratio, non-dimensional
    pub damping_ratio: f32,

    /// A flag to enable joint limits
    pub enable_limit: bool,

    /// The lower angle for the joint limit in radians. Minimum of -0.99*pi
    /// radians.
    pub lower_angle: f32,

    /// The upper angle for the joint limit in radians. Maximum of 0.99*pi
    /// radians.
    pub upper_angle: f32,

    /// A flag to enable the joint motor
    pub enable_motor: bool,

    /// The maximum motor torque, typically in newton-meters
    pub max_motor_torque: f32,

    /// The desired motor speed in radians per second
    pub motor_speed: f32,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultRevoluteJointDef)
pub fn default_revolute_joint_def() -> RevoluteJointDef {
    RevoluteJointDef {
        base: default_joint_def(),
        target_angle: 0.0,
        enable_spring: false,
        hertz: 0.0,
        damping_ratio: 0.0,
        enable_limit: false,
        lower_angle: 0.0,
        upper_angle: 0.0,
        enable_motor: false,
        max_motor_torque: 0.0,
        motor_speed: 0.0,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for RevoluteJointDef {
    fn default() -> Self {
        default_revolute_joint_def()
    }
}

/// Weld joint definition.
/// Connects two bodies together rigidly. This constraint provides springs to
/// mimic soft-body simulation.
/// Note: The approximate solver in Box2D cannot hold many bodies together
/// rigidly. (b2WeldJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WeldJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// Linear stiffness expressed as Hertz (cycles per second). Use zero for
    /// maximum stiffness.
    pub linear_hertz: f32,

    /// Angular stiffness as Hertz (cycles per second). Use zero for maximum
    /// stiffness.
    pub angular_hertz: f32,

    /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
    pub linear_damping_ratio: f32,

    /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
    pub angular_damping_ratio: f32,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultWeldJointDef)
pub fn default_weld_joint_def() -> WeldJointDef {
    WeldJointDef {
        base: default_joint_def(),
        linear_hertz: 0.0,
        angular_hertz: 0.0,
        linear_damping_ratio: 0.0,
        angular_damping_ratio: 0.0,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for WeldJointDef {
    fn default() -> Self {
        default_weld_joint_def()
    }
}

/// Wheel joint definition.
/// Body B is a wheel that may rotate freely and slide along the local x-axis
/// in frame A. The joint translation is zero when the local frame origins
/// coincide in world space. (b2WheelJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelJointDef {
    /// Base joint definition
    pub base: JointDef,

    /// Enable a linear spring along the local axis
    pub enable_spring: bool,

    /// Spring stiffness in Hertz
    pub hertz: f32,

    /// Spring damping ratio, non-dimensional
    pub damping_ratio: f32,

    /// Enable/disable the joint linear limit
    pub enable_limit: bool,

    /// The lower translation limit
    pub lower_translation: f32,

    /// The upper translation limit
    pub upper_translation: f32,

    /// Enable/disable the joint rotational motor
    pub enable_motor: bool,

    /// The maximum motor torque, typically in newton-meters
    pub max_motor_torque: f32,

    /// The desired motor speed in radians per second
    pub motor_speed: f32,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (b2DefaultWheelJointDef)
pub fn default_wheel_joint_def() -> WheelJointDef {
    WheelJointDef {
        base: default_joint_def(),
        enable_spring: true,
        hertz: 1.0,
        damping_ratio: 0.7,
        enable_limit: false,
        lower_translation: 0.0,
        upper_translation: 0.0,
        enable_motor: false,
        max_motor_torque: 0.0,
        motor_speed: 0.0,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for WheelJointDef {
    fn default() -> Self {
        default_wheel_joint_def()
    }
}
