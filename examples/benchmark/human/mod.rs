// Port of box2d-cpp-reference/shared/human.c + human.h.
//
// Only CreateHuman / DestroyHuman are needed by the Rain benchmark scene, so
// the mutation helpers (Human_SetScale, Human_ApplyRandomAngularImpulse, ...)
// are not ported. Several Human/Bone fields exist for struct parity with the C
// helper but are not read back by the benchmark; the module allows that dead
// code rather than dropping fields that keep the port readable against the C.
//
// The large CreateHuman body lives in the `create` submodule to keep each file
// within the project's per-file line limit.
#![allow(dead_code)]

mod create;
pub use create::create_human;

use box2d_rust::body::{body_get_local_point, destroy_body};
use box2d_rust::id::{BodyId, JointId};
use box2d_rust::joint::{create_revolute_joint, destroy_joint};
use box2d_rust::math_functions::{Pos, Rot};
use box2d_rust::types::default_revolute_joint_def;
use box2d_rust::world::World;

// (BoneId)
pub const BONE_HIP: usize = 0;
pub const BONE_TORSO: usize = 1;
pub const BONE_HEAD: usize = 2;
pub const BONE_UPPER_LEFT_LEG: usize = 3;
pub const BONE_LOWER_LEFT_LEG: usize = 4;
pub const BONE_UPPER_RIGHT_LEG: usize = 5;
pub const BONE_LOWER_RIGHT_LEG: usize = 6;
pub const BONE_UPPER_LEFT_ARM: usize = 7;
pub const BONE_LOWER_LEFT_ARM: usize = 8;
pub const BONE_UPPER_RIGHT_ARM: usize = 9;
pub const BONE_LOWER_RIGHT_ARM: usize = 10;
pub const BONE_COUNT: usize = 11;

// (Bone)
#[derive(Clone, Copy)]
pub struct Bone {
    pub body_id: BodyId,
    pub joint_id: JointId,
    pub friction_scale: f32,
    pub parent_index: i32,
}

impl Default for Bone {
    fn default() -> Self {
        Bone {
            body_id: BodyId::default(),
            joint_id: JointId::default(),
            friction_scale: 1.0,
            parent_index: -1,
        }
    }
}

// (Human)
#[derive(Clone)]
pub struct Human {
    pub bones: [Bone; BONE_COUNT],
    pub friction_torque: f32,
    pub original_scale: f32,
    pub scale: f32,
    pub is_spawned: bool,
}

impl Default for Human {
    fn default() -> Self {
        Human {
            bones: [Bone::default(); BONE_COUNT],
            friction_torque: 0.0,
            original_scale: 0.0,
            scale: 0.0,
            is_spawned: false,
        }
    }
}

/// Shared per-bone revolute joint construction. All bones use the same frame
/// derivation (local points on both bodies at `pivot`); the two lower arms also
/// rotate local frame A by a quarter turn.
#[allow(clippy::too_many_arguments)]
pub(crate) fn attach_revolute_joint(
    world: &mut World,
    parent_body: BodyId,
    child_body: BodyId,
    pivot: Pos,
    lower_angle: f32,
    upper_angle: f32,
    enable_limit: bool,
    enable_motor: bool,
    max_motor_torque: f32,
    enable_spring: bool,
    hertz: f32,
    damping_ratio: f32,
    draw_scale: f32,
    frame_a_rotation: Option<Rot>,
) -> JointId {
    let mut joint_def = default_revolute_joint_def();
    joint_def.base.body_id_a = parent_body;
    joint_def.base.body_id_b = child_body;
    joint_def.base.local_frame_a.p = body_get_local_point(world, parent_body, pivot);
    if let Some(q) = frame_a_rotation {
        joint_def.base.local_frame_a.q = q;
    }
    joint_def.base.local_frame_b.p = body_get_local_point(world, child_body, pivot);
    joint_def.enable_limit = enable_limit;
    joint_def.lower_angle = lower_angle;
    joint_def.upper_angle = upper_angle;
    joint_def.enable_motor = enable_motor;
    joint_def.max_motor_torque = max_motor_torque;
    joint_def.enable_spring = enable_spring;
    joint_def.hertz = hertz;
    joint_def.damping_ratio = damping_ratio;
    joint_def.base.draw_scale = draw_scale;
    create_revolute_joint(world, &joint_def)
}

/// (DestroyHuman)
pub fn destroy_human(world: &mut World, human: &mut Human) {
    debug_assert!(human.is_spawned);

    for i in 0..BONE_COUNT {
        if human.bones[i].joint_id.is_null() {
            continue;
        }

        destroy_joint(world, human.bones[i].joint_id, false);
        human.bones[i].joint_id = JointId::default();
    }

    for i in 0..BONE_COUNT {
        if human.bones[i].body_id.is_null() {
            continue;
        }

        destroy_body(world, human.bones[i].body_id);
        human.bones[i].body_id = BodyId::default();
    }

    human.is_spawned = false;
}
