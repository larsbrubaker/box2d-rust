//! C-exact `CreateHuman` (`shared/human.c`). Split from `human.rs` for the
//! 800-line limit.

use super::human::{
    pant_color, shirt_color, skin_colors, Bone, BONE_HEAD, BONE_HIP, BONE_LOWER_LEFT_ARM,
    BONE_LOWER_LEFT_LEG, BONE_LOWER_RIGHT_ARM, BONE_LOWER_RIGHT_LEG, BONE_TORSO,
    BONE_UPPER_LEFT_ARM, BONE_UPPER_LEFT_LEG, BONE_UPPER_RIGHT_ARM, BONE_UPPER_RIGHT_LEG,
};
use super::SimWorld;
use box2d_rust::body::create_body;
use box2d_rust::collision::Capsule;
use box2d_rust::debug_draw::HexColor;
use box2d_rust::geometry::make_polygon;
use box2d_rust::hull::compute_hull;
use box2d_rust::math_functions::{offset_pos, to_pos, Vec2, PI};
use box2d_rust::shape::{create_capsule_shape, create_polygon_shape};
use box2d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.95 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 1.475 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.775 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.475 * s,
                },
            );
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
            let pivot = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.625 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.775 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.475 * s,
                },
            );
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
            let pivot = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.625 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 1.225 * s,
                },
            );
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
            let pivot = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 1.35 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.975 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 1.225 * s,
                },
            );
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
            let pivot = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 1.35 * s,
                },
            );
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
            body_def.position = offset_pos(
                position,
                Vec2 {
                    x: 0.0,
                    y: 0.975 * s,
                },
            );
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
}
