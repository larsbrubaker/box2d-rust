//! C-exact `CreateHuman` and helpers (`shared/human.c` / `human.h`).
//! Sample ragdolls, Rain, Bounce Humans, Scale Ragdoll, etc. all go through this.

use super::body_ops::DESTROYED_BODY_SLOT;
use super::SimWorld;
use box2d_rust::body::{
    body_apply_angular_impulse, body_apply_mass_from_shapes, body_get_local_point,
    body_get_position, body_get_shapes, body_get_transform, body_set_linear_velocity,
    body_set_transform, create_body, destroy_body, get_body_full_id,
};
use box2d_rust::collision::{Capsule, ShapeType};
use box2d_rust::debug_draw::HexColor;
use box2d_rust::geometry::make_polygon;
use box2d_rust::hull::compute_hull;
use box2d_rust::id::{BodyId, JointId};
use box2d_rust::joint::{
    create_revolute_joint, destroy_joint, joint_get_local_frame_a, joint_get_local_frame_b,
    joint_get_type, joint_set_local_frame_a, joint_set_local_frame_b, JointType,
};
use box2d_rust::math_functions::{
    make_rot, mul_sv, offset_pos, sub_pos, to_pos, Pos, Vec2, PI,
};
use box2d_rust::revolute_joint::{
    revolute_joint_enable_motor, revolute_joint_enable_spring, revolute_joint_set_max_motor_torque,
    revolute_joint_set_spring_damping_ratio, revolute_joint_set_spring_hertz,
};
use box2d_rust::shape::{
    create_capsule_shape, create_polygon_shape, shape_enable_sensor_events, shape_get_capsule,
    shape_get_polygon, shape_get_type, shape_set_capsule, shape_set_polygon,
};
use box2d_rust::types::{default_body_def, default_revolute_joint_def, default_shape_def, BodyType};
use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

/// C `BoneId` (`human.h:8-22`)
const BONE_HIP: usize = 0;
const BONE_TORSO: usize = 1;
#[allow(dead_code)]
const BONE_HEAD: usize = 2;
const BONE_UPPER_LEFT_LEG: usize = 3;
const BONE_LOWER_LEFT_LEG: usize = 4;
const BONE_UPPER_RIGHT_LEG: usize = 5;
const BONE_LOWER_RIGHT_LEG: usize = 6;
const BONE_UPPER_LEFT_ARM: usize = 7;
const BONE_LOWER_LEFT_ARM: usize = 8;
const BONE_UPPER_RIGHT_ARM: usize = 9;
const BONE_LOWER_RIGHT_ARM: usize = 10;
const BONE_COUNT: usize = 11;

/// C `RAND_SEED` / `g_randomSeed` (`shared/utils.h`)
const RAND_LIMIT: u32 = 32767;
static G_RANDOM_SEED: AtomicU32 = AtomicU32::new(12345);

fn random_int() -> i32 {
    let mut x = G_RANDOM_SEED.load(Ordering::Relaxed);
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    G_RANDOM_SEED.store(x, Ordering::Relaxed);
    (x % (RAND_LIMIT + 1)) as i32
}

fn random_float_range(lo: f32, hi: f32) -> f32 {
    let r = (random_int() as u32 & RAND_LIMIT) as f32 / RAND_LIMIT as f32;
    (hi - lo) * r + lo
}

#[derive(Clone, Copy)]
struct Bone {
    body_id: BodyId,
    joint_id: JointId,
    /// Demo body slot (for `destroy_body` index stability).
    body_index: usize,
    /// Demo joint slot; unused when `joint_id` is null (hip).
    joint_index: usize,
    friction_scale: f32,
    parent_index: i32,
}

impl Default for Bone {
    fn default() -> Self {
        Bone {
            body_id: BodyId::default(),
            joint_id: JointId::default(),
            body_index: 0,
            joint_index: 0,
            friction_scale: 1.0,
            parent_index: -1,
        }
    }
}

/// C `Human` (`human.h:33-40`)
pub(crate) struct Human {
    bones: [Bone; BONE_COUNT],
    friction_torque: f32,
    original_scale: f32,
    scale: f32,
    is_spawned: bool,
}

impl Default for Human {
    fn default() -> Self {
        Human {
            bones: [Bone::default(); BONE_COUNT],
            friction_torque: 0.0,
            original_scale: 1.0,
            scale: 1.0,
            is_spawned: false,
        }
    }
}

fn shirt_color() -> u32 {
    HexColor::MEDIUM_TURQUOISE.0
}
fn pant_color() -> u32 {
    HexColor::DODGER_BLUE.0
}
fn skin_colors() -> [u32; 4] {
    [
        HexColor::NAVAJO_WHITE.0,
        HexColor::LIGHT_YELLOW.0,
        HexColor::PERU.0,
        HexColor::TAN.0,
    ]
}

impl SimWorld {
    fn human_alloc_slot(&mut self) -> usize {
        if let Some(i) = self.humans.iter().position(|h| !h.is_spawned) {
            self.humans[i] = Human::default();
            i
        } else {
            self.humans.push(Human::default());
            self.humans.len() - 1
        }
    }

    fn track_human_body(&mut self, body_id: BodyId) -> usize {
        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    fn track_human_joint(&mut self, joint_id: JointId) -> usize {
        self.joints.push(joint_id);
        self.joints.len() - 1
    }

    /// Create a revolute bone joint matching `human.c` defaults.
    fn human_create_revolute(
        &mut self,
        parent_body: BodyId,
        bone_body: BodyId,
        pivot: Pos,
        lower: f32,
        upper: f32,
        max_torque: f32,
        hertz: f32,
        damping_ratio: f32,
        frame_a_angle: f32,
        draw_size: f32,
    ) -> (JointId, usize) {
        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = parent_body;
        joint_def.base.body_id_b = bone_body;
        joint_def.base.local_frame_a.p = body_get_local_point(&self.world, parent_body, pivot);
        if frame_a_angle != 0.0 {
            joint_def.base.local_frame_a.q = make_rot(frame_a_angle);
        }
        joint_def.base.local_frame_b.p = body_get_local_point(&self.world, bone_body, pivot);
        joint_def.enable_limit = true;
        joint_def.lower_angle = lower;
        joint_def.upper_angle = upper;
        joint_def.enable_motor = true;
        joint_def.max_motor_torque = max_torque;
        joint_def.enable_spring = hertz > 0.0;
        joint_def.hertz = hertz;
        joint_def.damping_ratio = damping_ratio;
        joint_def.base.draw_scale = draw_size;
        let joint_id = create_revolute_joint(&mut self.world, &joint_def);
        let joint_index = self.track_human_joint(joint_id);
        (joint_id, joint_index)
    }
}

#[wasm_bindgen]
impl SimWorld {
    /// C `CreateHuman` (`shared/human.c:13-510`). Returns a demo human index.
    /// `user_data` is written onto every bone body (`bodyDef.userData`); samples
    /// often pass a slot index (Sensor Funnel) or `0` for `nullptr`.
    pub fn create_human(
        &mut self,
        x: f32,
        y: f32,
        scale: f32,
        friction_torque: f32,
        hertz: f32,
        damping_ratio: f32,
        group_index: i32,
        colorize: bool,
        user_data: u32,
    ) -> usize {
        let human_index = self.human_alloc_slot();
        let position = to_pos(Vec2 { x, y });

        // Initialize bones (human.c:18-28)
        {
            let human = &mut self.humans[human_index];
            debug_assert!(!human.is_spawned);
            for bone in human.bones.iter_mut() {
                *bone = Bone::default();
            }
            human.original_scale = scale;
            human.scale = scale;
            human.friction_torque = friction_torque;
        }

        let s = scale;
        let max_torque = friction_torque * s;
        let draw_size = 0.05f32;
        let shirt = shirt_color();
        let pant = pant_color();
        let skin = skin_colors()[(group_index.rem_euclid(4)) as usize];

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.sleep_threshold = 0.1;
        body_def.user_data = u64::from(user_data);

        let mut shape_def = default_shape_def();
        shape_def.material.friction = 0.2;
        shape_def.filter.group_index = -group_index;
        shape_def.filter.category_bits = 2;
        shape_def.filter.mask_bits = 1 | 2;

        let mut foot_shape_def = shape_def;
        foot_shape_def.material.friction = 0.05;
        foot_shape_def.filter.category_bits = 2;
        foot_shape_def.filter.mask_bits = 1;
        if colorize {
            foot_shape_def.material.custom_color = HexColor::SADDLE_BROWN.0;
        }

        // Foot hull (shared by both lower legs) — human.c:208-216
        let foot_points = [
            Vec2 {
                x: -0.03 * s,
                y: -0.185 * s,
            },
            Vec2 {
                x: 0.11 * s,
                y: -0.185 * s,
            },
            Vec2 {
                x: 0.11 * s,
                y: -0.16 * s,
            },
            Vec2 {
                x: -0.03 * s,
                y: -0.14 * s,
            },
        ];
        let foot_hull = compute_hull(&foot_points);
        let foot_polygon = make_polygon(&foot_hull, 0.015 * s);

        // --- hip ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.95 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "hip".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = pant;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.02 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.02 * s,
                },
                radius: 0.095 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let bone = &mut self.humans[human_index].bones[BONE_HIP];
            bone.parent_index = -1;
            bone.body_id = body_id;
            bone.body_index = body_index;
        }

        // --- torso ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 1.2 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "torso".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = shirt;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.135 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.135 * s,
                },
                radius: 0.09 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.0 * s });
            let parent = self.humans[human_index].bones[BONE_HIP].body_id;
            let friction_scale = 0.5f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.25 * PI,
                0.0,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_TORSO];
            bone.parent_index = BONE_HIP as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- head ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 1.475 * s });
            body_def.linear_damping = 0.1;
            body_def.name = "head".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = skin;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.038 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.039 * s,
                },
                radius: 0.075 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.4 * s });
            let parent = self.humans[human_index].bones[BONE_TORSO].body_id;
            let friction_scale = 0.25f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.3 * PI,
                0.1 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_HEAD];
            bone.parent_index = BONE_TORSO as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- upper left leg ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.775 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "upper_left_leg".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = pant;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.125 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.06 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 0.9 * s });
            let parent = self.humans[human_index].bones[BONE_HIP].body_id;
            let friction_scale = 1.0f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.05 * PI,
                0.4 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_UPPER_LEFT_LEG];
            bone.parent_index = BONE_HIP as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- lower left leg ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.475 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "lower_left_leg".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = pant;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.155 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.045 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            create_polygon_shape(&mut self.world, body_id, &foot_shape_def, &foot_polygon);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 0.625 * s });
            let parent = self.humans[human_index].bones[BONE_UPPER_LEFT_LEG].body_id;
            let friction_scale = 0.5f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.5 * PI,
                -0.02 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_LOWER_LEFT_LEG];
            bone.parent_index = BONE_UPPER_LEFT_LEG as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- upper right leg ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.775 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "upper_right_leg".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = pant;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.125 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.06 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 0.9 * s });
            let parent = self.humans[human_index].bones[BONE_HIP].body_id;
            let friction_scale = 1.0f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.05 * PI,
                0.4 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_UPPER_RIGHT_LEG];
            bone.parent_index = BONE_HIP as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- lower right leg ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.475 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "lower_right_leg".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = pant;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.155 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.045 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            create_polygon_shape(&mut self.world, body_id, &foot_shape_def, &foot_polygon);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 0.625 * s });
            let parent = self.humans[human_index].bones[BONE_UPPER_RIGHT_LEG].body_id;
            let friction_scale = 0.5f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.5 * PI,
                -0.02 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_LOWER_RIGHT_LEG];
            bone.parent_index = BONE_UPPER_RIGHT_LEG as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- upper left arm ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 1.225 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "upper_left_arm".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = shirt;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.125 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.035 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.35 * s });
            let parent = self.humans[human_index].bones[BONE_TORSO].body_id;
            let friction_scale = 0.5f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.1 * PI,
                0.8 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_UPPER_LEFT_ARM];
            bone.parent_index = BONE_TORSO as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- lower left arm ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.975 * s });
            body_def.linear_damping = 0.1;
            body_def.name = "lower_left_arm".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = skin;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.125 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.03 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.1 * s });
            let parent = self.humans[human_index].bones[BONE_UPPER_LEFT_ARM].body_id;
            let friction_scale = 0.1f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.2 * PI,
                0.3 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.25 * PI,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_LOWER_LEFT_ARM];
            bone.parent_index = BONE_UPPER_LEFT_ARM as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- upper right arm ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 1.225 * s });
            body_def.linear_damping = 0.0;
            body_def.name = "upper_right_arm".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = shirt;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.125 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.035 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.35 * s });
            let parent = self.humans[human_index].bones[BONE_TORSO].body_id;
            let friction_scale = 0.5f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.1 * PI,
                0.8 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.0,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_UPPER_RIGHT_ARM];
            bone.parent_index = BONE_TORSO as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        // --- lower right arm ---
        {
            body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 0.975 * s });
            body_def.linear_damping = 0.1;
            body_def.name = "lower_right_arm".to_string();
            let body_id = create_body(&mut self.world, &body_def);
            let body_index = self.track_human_body(body_id);
            if colorize {
                shape_def.material.custom_color = skin;
            }
            let capsule = Capsule {
                center1: Vec2 {
                    x: 0.0,
                    y: -0.125 * s,
                },
                center2: Vec2 {
                    x: 0.0,
                    y: 0.125 * s,
                },
                radius: 0.03 * s,
            };
            create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
            let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.1 * s });
            let parent = self.humans[human_index].bones[BONE_UPPER_RIGHT_ARM].body_id;
            let friction_scale = 0.1f32;
            let (joint_id, joint_index) = self.human_create_revolute(
                parent,
                body_id,
                pivot,
                -0.2 * PI,
                0.3 * PI,
                friction_scale * max_torque,
                hertz,
                damping_ratio,
                0.25 * PI,
                draw_size,
            );
            let bone = &mut self.humans[human_index].bones[BONE_LOWER_RIGHT_ARM];
            bone.parent_index = BONE_UPPER_RIGHT_ARM as i32;
            bone.friction_scale = friction_scale;
            bone.body_id = body_id;
            bone.body_index = body_index;
            bone.joint_id = joint_id;
            bone.joint_index = joint_index;
        }

        self.humans[human_index].is_spawned = true;
        human_index
    }

    /// C `DestroyHuman` (`human.c:512-539`).
    pub fn destroy_human(&mut self, index: usize) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        // Joints first
        for i in 0..BONE_COUNT {
            let joint_id = self.humans[index].bones[i].joint_id;
            let joint_index = self.humans[index].bones[i].joint_index;
            if joint_id.is_null() {
                continue;
            }
            destroy_joint(&mut self.world, joint_id, false);
            if joint_index < self.joints.len() {
                self.joints[joint_index] = JointId::default();
            }
            self.humans[index].bones[i].joint_id = JointId::default();
        }
        // Bodies
        for i in 0..BONE_COUNT {
            let body_id = self.humans[index].bones[i].body_id;
            let body_index = self.humans[index].bones[i].body_index;
            if body_id.is_null() {
                continue;
            }
            destroy_body(&mut self.world, body_id);
            if body_index < self.bodies.len() {
                self.bodies[body_index] = DESTROYED_BODY_SLOT;
            }
            self.humans[index].bones[i].body_id = BodyId::default();
        }
        self.humans[index].is_spawned = false;
        self.grab.validate(&mut self.world);
    }

    /// C `Human_SetVelocity` (`human.c:541-554`).
    pub fn human_set_velocity(&mut self, index: usize, vx: f32, vy: f32) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        let v = Vec2 { x: vx, y: vy };
        for i in 0..BONE_COUNT {
            let body_id = self.humans[index].bones[i].body_id;
            if body_id.is_null() {
                continue;
            }
            body_set_linear_velocity(&mut self.world, body_id, v);
        }
    }

    /// C `Human_ApplyRandomAngularImpulse` (`human.c:556-561`).
    pub fn human_apply_random_angular_impulse(&mut self, index: usize, magnitude: f32) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        let impulse = random_float_range(-magnitude, magnitude);
        let torso = self.humans[index].bones[BONE_TORSO].body_id;
        body_apply_angular_impulse(&mut self.world, torso, impulse, true);
    }

    /// C `Human_SetJointFrictionTorque` (`human.c:563-582`).
    pub fn human_set_joint_friction_torque(&mut self, index: usize, torque: f32) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        let scale = self.humans[index].scale;
        if torque == 0.0 {
            for i in 1..BONE_COUNT {
                let jid = self.humans[index].bones[i].joint_id;
                revolute_joint_enable_motor(&mut self.world, jid, false);
            }
        } else {
            for i in 1..BONE_COUNT {
                let jid = self.humans[index].bones[i].joint_id;
                let fs = self.humans[index].bones[i].friction_scale;
                revolute_joint_enable_motor(&mut self.world, jid, true);
                revolute_joint_set_max_motor_torque(&mut self.world, jid, scale * fs * torque);
            }
        }
    }

    /// C `Human_SetJointSpringHertz` (`human.c:584-602`).
    pub fn human_set_joint_spring_hertz(&mut self, index: usize, hertz: f32) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        if hertz == 0.0 {
            for i in 1..BONE_COUNT {
                let jid = self.humans[index].bones[i].joint_id;
                revolute_joint_enable_spring(&mut self.world, jid, false);
            }
        } else {
            for i in 1..BONE_COUNT {
                let jid = self.humans[index].bones[i].joint_id;
                revolute_joint_enable_spring(&mut self.world, jid, true);
                revolute_joint_set_spring_hertz(&mut self.world, jid, hertz);
            }
        }
    }

    /// C `Human_SetJointDampingRatio` (`human.c:604-611`).
    pub fn human_set_joint_damping_ratio(&mut self, index: usize, damping_ratio: f32) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        for i in 1..BONE_COUNT {
            let jid = self.humans[index].bones[i].joint_id;
            revolute_joint_set_spring_damping_ratio(&mut self.world, jid, damping_ratio);
        }
    }

    /// C `Human_EnableSensorEvents` (`human.c:613-624`).
    pub fn human_enable_sensor_events(&mut self, index: usize, enable: bool) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        let body_id = self.humans[index].bones[BONE_TORSO].body_id;
        let shapes = body_get_shapes(&self.world, body_id, 1);
        if shapes.len() == 1 {
            shape_enable_sensor_events(&mut self.world, shapes[0], enable);
        }
    }

    /// C `Human_SetScale` (`human.c:626-698`).
    pub fn human_set_scale(&mut self, index: usize, scale: f32) {
        if index >= self.humans.len() || !self.humans[index].is_spawned {
            return;
        }
        debug_assert!(0.01 < scale && scale < 100.0);
        let old_scale = self.humans[index].scale;
        debug_assert!(0.0 < old_scale);
        let ratio = scale / old_scale;
        let original_ratio = scale / self.humans[index].original_scale;
        let friction_torque =
            (original_ratio * original_ratio * original_ratio) * self.humans[index].friction_torque;
        let origin = body_get_position(&self.world, self.humans[index].bones[0].body_id);

        for bone_index in 0..BONE_COUNT {
            let bone = self.humans[index].bones[bone_index];
            if bone_index > 0 {
                let world_transform = body_get_transform(&self.world, bone.body_id);
                let d = mul_sv(ratio, sub_pos(world_transform.p, origin));
                body_set_transform(
                    &mut self.world,
                    bone.body_id,
                    offset_pos(origin, d),
                    world_transform.q,
                );

                let mut local_frame_a = joint_get_local_frame_a(&self.world, bone.joint_id);
                let mut local_frame_b = joint_get_local_frame_b(&self.world, bone.joint_id);
                local_frame_a.p = mul_sv(ratio, local_frame_a.p);
                local_frame_b.p = mul_sv(ratio, local_frame_b.p);
                joint_set_local_frame_a(&mut self.world, bone.joint_id, local_frame_a);
                joint_set_local_frame_b(&mut self.world, bone.joint_id, local_frame_b);

                if joint_get_type(&self.world, bone.joint_id) == JointType::Revolute {
                    revolute_joint_set_max_motor_torque(
                        &mut self.world,
                        bone.joint_id,
                        bone.friction_scale * friction_torque,
                    );
                }
            }

            let shape_ids = body_get_shapes(&self.world, bone.body_id, 2);
            for &shape_id in &shape_ids {
                match shape_get_type(&self.world, shape_id) {
                    ShapeType::Capsule => {
                        let mut capsule = shape_get_capsule(&self.world, shape_id);
                        capsule.center1 = mul_sv(ratio, capsule.center1);
                        capsule.center2 = mul_sv(ratio, capsule.center2);
                        capsule.radius *= ratio;
                        shape_set_capsule(&mut self.world, shape_id, &capsule);
                    }
                    ShapeType::Polygon => {
                        let mut polygon = shape_get_polygon(&self.world, shape_id);
                        for point_index in 0..polygon.count as usize {
                            polygon.vertices[point_index] =
                                mul_sv(ratio, polygon.vertices[point_index]);
                        }
                        polygon.centroid = mul_sv(ratio, polygon.centroid);
                        polygon.radius *= ratio;
                        shape_set_polygon(&mut self.world, shape_id, &polygon);
                    }
                    _ => {}
                }
            }
            body_apply_mass_from_shapes(&mut self.world, bone.body_id);
        }

        self.humans[index].scale = scale;
    }

    /// Whether a demo human index is currently spawned.
    pub fn human_is_spawned(&self, index: usize) -> bool {
        index < self.humans.len() && self.humans[index].is_spawned
    }
}
