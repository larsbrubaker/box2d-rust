// CreateHuman, split out from the human module so each file stays within the
// project's per-file line limit. This is the direct port of the C CreateHuman
// body; see `super` for the Human/Bone types and the joint helper.

use box2d_rust::body::create_body;
use box2d_rust::collision::Capsule;
use box2d_rust::debug_draw::HexColor;
use box2d_rust::geometry::make_polygon;
use box2d_rust::hull::compute_hull;
use box2d_rust::id::{BodyId, JointId};
use box2d_rust::math_functions::{make_rot, offset_pos, Pos, Vec2, PI};
use box2d_rust::shape::{create_capsule_shape, create_polygon_shape};
use box2d_rust::types::{default_body_def, default_shape_def, BodyType};
use box2d_rust::world::World;

use super::{
    attach_revolute_joint, Human, BONE_HEAD, BONE_HIP, BONE_LOWER_LEFT_ARM, BONE_LOWER_LEFT_LEG,
    BONE_LOWER_RIGHT_ARM, BONE_LOWER_RIGHT_LEG, BONE_TORSO, BONE_UPPER_LEFT_ARM,
    BONE_UPPER_LEFT_LEG, BONE_UPPER_RIGHT_ARM, BONE_UPPER_RIGHT_LEG,
};

/// (CreateHuman)
#[allow(clippy::too_many_arguments)]
pub fn create_human(
    human: &mut Human,
    world: &mut World,
    position: Pos,
    scale: f32,
    friction_torque: f32,
    hertz: f32,
    damping_ratio: f32,
    group_index: i32,
    user_data: u64,
    colorize: bool,
) {
    debug_assert!(!human.is_spawned);

    for bone in human.bones.iter_mut() {
        bone.body_id = BodyId::default();
        bone.joint_id = JointId::default();
        bone.friction_scale = 1.0;
        bone.parent_index = -1;
    }

    human.original_scale = scale;
    human.scale = scale;
    human.friction_torque = friction_torque;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.sleep_threshold = 0.1;
    body_def.user_data = user_data;

    let mut shape_def = default_shape_def();
    shape_def.material.friction = 0.2;
    shape_def.filter.group_index = -group_index;
    shape_def.filter.category_bits = 2;
    shape_def.filter.mask_bits = 1 | 2;

    let mut foot_shape_def = shape_def;
    foot_shape_def.material.friction = 0.05;
    // feet don't collide with ragdolls
    foot_shape_def.filter.category_bits = 2;
    foot_shape_def.filter.mask_bits = 1;

    if colorize {
        foot_shape_def.material.custom_color = HexColor::SADDLE_BROWN.0;
    }

    let s = scale;
    let max_torque = friction_torque * s;
    let enable_motor = true;
    let enable_limit = true;
    let draw_size = 0.05;

    let shirt_color = HexColor::MEDIUM_TURQUOISE;
    let pant_color = HexColor::DODGER_BLUE;

    let skin_colors = [
        HexColor::NAVAJO_WHITE,
        HexColor::LIGHT_YELLOW,
        HexColor::PERU,
        HexColor::TAN,
    ];
    let skin_color = skin_colors[(group_index % 4) as usize];

    // hip
    {
        human.bones[BONE_HIP].parent_index = -1;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.95 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "hip".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_HIP].body_id = body_id;

        if colorize {
            shape_def.material.custom_color = pant_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);
    }

    // torso
    {
        human.bones[BONE_TORSO].parent_index = BONE_HIP as i32;

        body_def.position = offset_pos(position, Vec2 { x: 0.0, y: 1.2 * s });
        body_def.linear_damping = 0.0;
        body_def.name = "torso".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_TORSO].body_id = body_id;
        human.bones[BONE_TORSO].friction_scale = 0.5;
        body_def.type_ = BodyType::Dynamic;

        if colorize {
            shape_def.material.custom_color = shirt_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.0 * s });
        let parent = human.bones[BONE_HIP].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.25 * PI,
            0.0,
            enable_limit,
            enable_motor,
            human.bones[BONE_TORSO].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_TORSO].joint_id = joint_id;
    }

    // head
    {
        human.bones[BONE_HEAD].parent_index = BONE_TORSO as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0 * s,
                y: 1.475 * s,
            },
        );
        body_def.linear_damping = 0.1;
        body_def.name = "head".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_HEAD].body_id = body_id;
        human.bones[BONE_HEAD].friction_scale = 0.25;

        if colorize {
            shape_def.material.custom_color = skin_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.4 * s });
        let parent = human.bones[BONE_TORSO].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.3 * PI,
            0.1 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_HEAD].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_HEAD].joint_id = joint_id;
    }

    // upper left leg
    {
        human.bones[BONE_UPPER_LEFT_LEG].parent_index = BONE_HIP as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.775 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "upper_left_leg".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_UPPER_LEFT_LEG].body_id = body_id;
        human.bones[BONE_UPPER_LEFT_LEG].friction_scale = 1.0;

        if colorize {
            shape_def.material.custom_color = pant_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(position, Vec2 { x: 0.0, y: 0.9 * s });
        let parent = human.bones[BONE_HIP].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.05 * PI,
            0.4 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_UPPER_LEFT_LEG].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_UPPER_LEFT_LEG].joint_id = joint_id;
    }

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

    // lower left leg
    {
        human.bones[BONE_LOWER_LEFT_LEG].parent_index = BONE_UPPER_LEFT_LEG as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.475 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "lower_left_leg".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_LOWER_LEFT_LEG].body_id = body_id;
        human.bones[BONE_LOWER_LEFT_LEG].friction_scale = 0.5;

        if colorize {
            shape_def.material.custom_color = pant_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        create_polygon_shape(world, body_id, &foot_shape_def, &foot_polygon);

        let pivot = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.625 * s,
            },
        );
        let parent = human.bones[BONE_UPPER_LEFT_LEG].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.5 * PI,
            -0.02 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_LOWER_LEFT_LEG].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_LOWER_LEFT_LEG].joint_id = joint_id;
    }

    // upper right leg
    {
        human.bones[BONE_UPPER_RIGHT_LEG].parent_index = BONE_HIP as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.775 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "upper_right_leg".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_UPPER_RIGHT_LEG].body_id = body_id;
        human.bones[BONE_UPPER_RIGHT_LEG].friction_scale = 1.0;

        if colorize {
            shape_def.material.custom_color = pant_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(position, Vec2 { x: 0.0, y: 0.9 * s });
        let parent = human.bones[BONE_HIP].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.05 * PI,
            0.4 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_UPPER_RIGHT_LEG].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_UPPER_RIGHT_LEG].joint_id = joint_id;
    }

    // lower right leg
    {
        human.bones[BONE_LOWER_RIGHT_LEG].parent_index = BONE_UPPER_RIGHT_LEG as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.475 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "lower_right_leg".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_LOWER_RIGHT_LEG].body_id = body_id;
        human.bones[BONE_LOWER_RIGHT_LEG].friction_scale = 0.5;

        if colorize {
            shape_def.material.custom_color = pant_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        create_polygon_shape(world, body_id, &foot_shape_def, &foot_polygon);

        let pivot = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.625 * s,
            },
        );
        let parent = human.bones[BONE_UPPER_RIGHT_LEG].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.5 * PI,
            -0.02 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_LOWER_RIGHT_LEG].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_LOWER_RIGHT_LEG].joint_id = joint_id;
    }

    // upper left arm
    {
        human.bones[BONE_UPPER_LEFT_ARM].parent_index = BONE_TORSO as i32;
        human.bones[BONE_UPPER_LEFT_ARM].friction_scale = 0.5;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 1.225 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "upper_left_arm".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_UPPER_LEFT_ARM].body_id = body_id;

        if colorize {
            shape_def.material.custom_color = shirt_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 1.35 * s,
            },
        );
        let parent = human.bones[BONE_TORSO].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.1 * PI,
            0.8 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_UPPER_LEFT_ARM].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_UPPER_LEFT_ARM].joint_id = joint_id;
    }

    // lower left arm
    {
        human.bones[BONE_LOWER_LEFT_ARM].parent_index = BONE_UPPER_LEFT_ARM as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.975 * s,
            },
        );
        body_def.linear_damping = 0.1;
        body_def.name = "lower_left_arm".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_LOWER_LEFT_ARM].body_id = body_id;
        human.bones[BONE_LOWER_LEFT_ARM].friction_scale = 0.1;

        if colorize {
            shape_def.material.custom_color = skin_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.1 * s });
        let parent = human.bones[BONE_UPPER_LEFT_ARM].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.2 * PI,
            0.3 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_LOWER_LEFT_ARM].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            Some(make_rot(0.25 * PI)),
        );
        human.bones[BONE_LOWER_LEFT_ARM].joint_id = joint_id;
    }

    // upper right arm
    {
        human.bones[BONE_UPPER_RIGHT_ARM].parent_index = BONE_TORSO as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 1.225 * s,
            },
        );
        body_def.linear_damping = 0.0;
        body_def.name = "upper_right_arm".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_UPPER_RIGHT_ARM].body_id = body_id;
        human.bones[BONE_UPPER_RIGHT_ARM].friction_scale = 0.5;

        if colorize {
            shape_def.material.custom_color = shirt_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 1.35 * s,
            },
        );
        let parent = human.bones[BONE_TORSO].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.1 * PI,
            0.8 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_UPPER_RIGHT_ARM].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            None,
        );
        human.bones[BONE_UPPER_RIGHT_ARM].joint_id = joint_id;
    }

    // lower right arm
    {
        human.bones[BONE_LOWER_RIGHT_ARM].parent_index = BONE_UPPER_RIGHT_ARM as i32;

        body_def.position = offset_pos(
            position,
            Vec2 {
                x: 0.0,
                y: 0.975 * s,
            },
        );
        body_def.linear_damping = 0.1;
        body_def.name = "lower_right_arm".to_string();

        let body_id = create_body(world, &body_def);
        human.bones[BONE_LOWER_RIGHT_ARM].body_id = body_id;
        human.bones[BONE_LOWER_RIGHT_ARM].friction_scale = 0.1;

        if colorize {
            shape_def.material.custom_color = skin_color.0;
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
        create_capsule_shape(world, body_id, &shape_def, &capsule);

        let pivot = offset_pos(position, Vec2 { x: 0.0, y: 1.1 * s });
        let parent = human.bones[BONE_UPPER_RIGHT_ARM].body_id;
        let joint_id = attach_revolute_joint(
            world,
            parent,
            body_id,
            pivot,
            -0.2 * PI,
            0.3 * PI,
            enable_limit,
            enable_motor,
            human.bones[BONE_LOWER_RIGHT_ARM].friction_scale * max_torque,
            hertz > 0.0,
            hertz,
            damping_ratio,
            draw_size,
            Some(make_rot(0.25 * PI)),
        );
        human.bones[BONE_LOWER_RIGHT_ARM].joint_id = joint_id;
    }

    human.is_spawned = true;
}
