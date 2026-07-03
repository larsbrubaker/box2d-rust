// Port of the body data model from box2d-cpp-reference/src/body.h.
// Logic from body.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::distance::Sweep;
use crate::math_functions::{
    sub_pos, Pos, Rot, Vec2, WorldTransform, POS_ZERO, ROT_IDENTITY, VEC2_ZERO,
    WORLD_TRANSFORM_IDENTITY,
};
use crate::types::BodyType;

// enum b2BodyFlags
pub mod body_flags {
    /// This body has fixed translation along the x-axis
    pub const LOCK_LINEAR_X: u32 = 0x00000001;
    /// This body has fixed translation along the y-axis
    pub const LOCK_LINEAR_Y: u32 = 0x00000002;
    /// This body has fixed rotation
    pub const LOCK_ANGULAR_Z: u32 = 0x00000004;
    /// This flag is used for debug draw
    pub const IS_FAST: u32 = 0x00000008;
    /// This dynamic body does a final CCD pass against all body types, but not other bullets
    pub const IS_BULLET: u32 = 0x00000010;
    /// This body was speed capped in the current time step
    pub const IS_SPEED_CAPPED: u32 = 0x00000020;
    /// This body had a time of impact event in the current time step
    pub const HAD_TIME_OF_IMPACT: u32 = 0x00000040;
    /// This body has no limit on angular velocity
    pub const ALLOW_FAST_ROTATION: u32 = 0x00000080;
    /// This body needs to have its AABB increased
    pub const ENLARGE_BOUNDS: u32 = 0x00000100;
    /// This body is dynamic so the solver should write to it.
    pub const DYNAMIC_FLAG: u32 = 0x00000200;
    /// The user deferred mass computation but b2Body_ApplyMassFromShapes was
    /// not called before the world step.
    pub const DIRTY_MASS: u32 = 0x00000400;
    pub const ENABLE_SLEEP: u32 = 0x00000800;
    pub const BODY_ENABLE_CONTACT_RECYCLING: u32 = 0x00001000;

    /// All lock flags
    pub const ALL_LOCKS: u32 = LOCK_ANGULAR_Z | LOCK_LINEAR_X | LOCK_LINEAR_Y;
    /// If this flag is set then the body has fixed rotation
    pub const FIXED_ROTATION: u32 = LOCK_ANGULAR_Z;
    /// These flags are transient per time step. These may be different across
    /// Body, BodySim, and BodyState.
    pub const BODY_TRANSIENT_FLAGS: u32 = IS_FAST | IS_SPEED_CAPPED | HAD_TIME_OF_IMPACT;
}

/// Body organizational details that are not used in the solver. (b2Body)
#[derive(Debug, Clone)]
pub struct Body {
    pub user_data: u64,

    /// index of solver set stored in World. May be NULL_INDEX.
    pub set_index: i32,

    /// body sim and state index within set. May be NULL_INDEX.
    pub local_index: i32,

    /// [31 : contactId | 1 : edgeIndex]
    pub head_contact_key: i32,
    pub contact_count: i32,

    pub head_shape_id: i32,
    pub shape_count: i32,

    pub head_chain_id: i32,

    /// [31 : jointId | 1 : edgeIndex]
    pub head_joint_key: i32,
    pub joint_count: i32,

    /// All enabled dynamic and kinematic bodies are in an island.
    pub island_id: i32,

    /// Need this island index for faster union-find
    pub island_index: i32,

    pub mass: f32,

    /// Rotational inertia about the center of mass.
    pub inertia: f32,

    pub sleep_threshold: f32,
    pub sleep_time: f32,

    /// this is used to adjust the fellAsleep flag in the body move array
    pub body_move_index: i32,

    pub id: i32,

    /// body_flags bits
    pub flags: u32,

    pub type_: BodyType,

    /// Monotonically advanced when a body is allocated in this slot.
    /// Used to check for invalid BodyId.
    pub generation: u16,

    /// Body name for debugging (C: char[B2_NAME_LENGTH + 1]).
    pub name: String,
}

impl Default for Body {
    fn default() -> Self {
        Body {
            user_data: 0,
            set_index: NULL_INDEX,
            local_index: NULL_INDEX,
            head_contact_key: NULL_INDEX,
            contact_count: 0,
            head_shape_id: NULL_INDEX,
            shape_count: 0,
            head_chain_id: NULL_INDEX,
            head_joint_key: NULL_INDEX,
            joint_count: 0,
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            mass: 0.0,
            inertia: 0.0,
            sleep_threshold: 0.0,
            sleep_time: 0.0,
            body_move_index: NULL_INDEX,
            id: NULL_INDEX,
            flags: 0,
            type_: BodyType::Static,
            generation: 0,
            name: String::new(),
        }
    }
}

/// Body state, designed for fast conversion to and from SIMD via
/// scatter-gather. Only awake dynamic and kinematic bodies have a body state.
/// Used in the performance critical constraint solver. (b2BodyState, 32 bytes)
///
/// The solver operates on the body state. The body state array does not hold
/// static bodies; their delta rotation is identity via a dummy state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
    /// body_flags bits. Important flags: locking, dynamic.
    pub flags: u32,
    /// Using delta position reduces round-off error far from the origin
    pub delta_position: Vec2,
    /// Using delta rotation because the solver cannot access the full rotation
    /// on static bodies and must use zero delta rotation (c,s) = (1,0)
    pub delta_rotation: Rot,
}

/// Identity body state, notice the delta_rotation is {1, 0}.
/// (b2_identityBodyState)
pub const IDENTITY_BODY_STATE: BodyState = BodyState {
    linear_velocity: VEC2_ZERO,
    angular_velocity: 0.0,
    flags: 0,
    delta_position: VEC2_ZERO,
    delta_rotation: ROT_IDENTITY,
};

impl Default for BodyState {
    fn default() -> Self {
        IDENTITY_BODY_STATE
    }
}

/// Body simulation data used for integration of position and velocity.
/// Transform data used for collision and solver preparation. (b2BodySim)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodySim {
    /// transform for body origin, double translation in large world mode
    pub transform: WorldTransform,

    /// center of mass position in world space
    pub center: Pos,

    /// previous rotation and COM for TOI
    pub rotation0: Rot,
    pub center0: Pos,

    /// location of center of mass relative to the body origin
    pub local_center: Vec2,

    pub force: Vec2,
    pub torque: f32,

    /// inverse mass and inertia
    pub inv_mass: f32,
    pub inv_inertia: f32,

    pub min_extent: f32,
    pub max_extent: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,

    /// Index of Body
    pub body_id: i32,

    /// body_flags bits
    pub flags: u32,
}

impl Default for BodySim {
    fn default() -> Self {
        BodySim {
            transform: WORLD_TRANSFORM_IDENTITY,
            center: POS_ZERO,
            rotation0: ROT_IDENTITY,
            center0: POS_ZERO,
            local_center: VEC2_ZERO,
            force: VEC2_ZERO,
            torque: 0.0,
            inv_mass: 0.0,
            inv_inertia: 0.0,
            min_extent: 0.0,
            max_extent: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 0.0,
            body_id: NULL_INDEX,
            flags: 0,
        }
    }
}

/// Build a sweep relative to a base position so continuous collision keeps
/// float precision far from the origin. The base cancels out of the relative
/// motion the TOI actually solves. (b2MakeRelativeSweep)
pub fn make_relative_sweep(body_sim: &BodySim, base: Pos) -> Sweep {
    Sweep {
        c1: sub_pos(body_sim.center0, base),
        c2: sub_pos(body_sim.center, base),
        q1: body_sim.rotation0,
        q2: body_sim.transform.q,
        local_center: body_sim.local_center,
    }
}
