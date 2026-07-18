// Machine-style benchmark scenes: spinner, smash, tumbler, washer, and
// junkyard. Ported from benchmarks.c. The C globals g_spinnerData and
// g_junkyardData become thread-local state here.

use std::cell::Cell;

use box2d_rust::body::{body_get_local_point, body_set_target_transform, create_body};
use box2d_rust::collision::{Capsule, Circle};
use box2d_rust::geometry::{
    make_box, make_offset_box, make_polygon, make_rounded_box, make_square,
};
use box2d_rust::hull::compute_hull;
use box2d_rust::id::{BodyId, JointId};
use box2d_rust::joint::create_revolute_joint;
use box2d_rust::math_functions::{
    compute_cos_sin, inv_rotate_vector, make_rot, mul_sv, rotate_vector, to_pos, Rot, Vec2,
    WorldTransform, PI, ROT_IDENTITY, VEC2_ZERO,
};
use box2d_rust::revolute_joint::revolute_joint_get_angle;
use box2d_rust::shape::{
    create_capsule_shape, create_chain, create_circle_shape, create_polygon_shape,
};
use box2d_rust::types::{
    default_body_def, default_chain_def, default_revolute_joint_def, default_shape_def, BodyType,
    SurfaceMaterial,
};
use box2d_rust::world::{world_set_gravity, World};

use super::BENCHMARK_DEBUG;

thread_local! {
    static SPINNER_ID: Cell<JointId> = Cell::new(JointId::default());
    static JUNKYARD_PUSHER_ID: Cell<BodyId> = Cell::new(BodyId::default());
}

const SPINNER_POINT_COUNT: usize = 360;

// (CreateSpinner)
pub fn create_spinner(world: &mut World) {
    let ground_id;
    {
        let body_def = default_body_def();
        ground_id = create_body(world, &body_def);

        let q = make_rot(-2.0 * PI / SPINNER_POINT_COUNT as f32);
        let mut p = Vec2 { x: 40.0, y: 0.0 };
        let mut points: Vec<Vec2> = Vec::with_capacity(SPINNER_POINT_COUNT);
        for _ in 0..SPINNER_POINT_COUNT {
            points.push(Vec2 {
                x: p.x,
                y: p.y + 32.0,
            });
            p = rotate_vector(q, p);
        }

        // C: b2SurfaceMaterial material = { 0 }; material.friction = 0.1f;
        let material = SurfaceMaterial {
            friction: 0.1,
            restitution: 0.0,
            rolling_resistance: 0.0,
            tangent_speed: 0.0,
            user_material_id: 0,
            custom_color: 0,
        };

        let mut chain_def = default_chain_def();
        chain_def.points = points;
        chain_def.is_loop = true;
        chain_def.materials = vec![material];

        create_chain(world, ground_id, &chain_def);
    }

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = to_pos(Vec2 { x: 0.0, y: 12.0 });
        body_def.enable_sleep = false;

        let spinner_id = create_body(world, &body_def);

        let box_shape = make_rounded_box(0.4, 20.0, 0.2);
        let mut shape_def = default_shape_def();
        shape_def.material.friction = 0.0;
        create_polygon_shape(world, spinner_id, &shape_def, &box_shape);

        let motor_speed = 5.0;
        let max_motor_torque = f32::MAX;
        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = ground_id;
        joint_def.base.body_id_b = spinner_id;
        joint_def.base.local_frame_a.p = body_get_local_point(world, ground_id, body_def.position);
        joint_def.enable_motor = true;
        joint_def.motor_speed = motor_speed;
        joint_def.max_motor_torque = max_motor_torque;

        let joint_id = create_revolute_joint(world, &joint_def);
        SPINNER_ID.with(|id| id.set(joint_id));
    }

    let capsule = Capsule {
        center1: Vec2 { x: -0.25, y: 0.0 },
        center2: Vec2 { x: 0.25, y: 0.0 },
        radius: 0.25,
    };
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.35,
    };
    let square = make_square(0.35);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let mut shape_def = default_shape_def();
    shape_def.material.friction = 0.1;
    shape_def.material.restitution = 0.1;
    shape_def.density = 0.25;

    let body_count: i32 = if BENCHMARK_DEBUG { 499 } else { 2 * 3038 };

    let mut x = -23.0;
    let mut y = 2.0;
    for i in 0..body_count {
        body_def.position = to_pos(Vec2 { x, y });
        let body_id = create_body(world, &body_def);

        let remainder = i % 3;
        if remainder == 0 {
            create_capsule_shape(world, body_id, &shape_def, &capsule);
        } else if remainder == 1 {
            create_circle_shape(world, body_id, &shape_def, &circle);
        } else if remainder == 2 {
            create_polygon_shape(world, body_id, &shape_def, &square);
        }

        x += 0.5;

        if x >= 23.0 {
            x = -23.0;
            y += 0.5;
        }
    }
}

// (StepSpinner)
pub fn step_spinner(world: &mut World, _step_count: i32) -> f32 {
    let joint_id = SPINNER_ID.with(|id| id.get());
    revolute_joint_get_angle(world, joint_id)
}

// (CreateSmash)
pub fn create_smash(world: &mut World) {
    world_set_gravity(world, VEC2_ZERO);

    {
        let box_shape = make_box(4.0, 4.0);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = to_pos(Vec2 { x: -20.0, y: 0.0 });
        body_def.linear_velocity = Vec2 { x: 40.0, y: 0.0 };
        let body_id = create_body(world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 8.0;
        create_polygon_shape(world, body_id, &shape_def, &box_shape);
    }

    let d = 0.4;
    let box_shape = make_square(0.5 * d);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.is_awake = false;

    let shape_def = default_shape_def();

    let columns: i32 = if BENCHMARK_DEBUG { 20 } else { 120 };
    let rows: i32 = if BENCHMARK_DEBUG { 10 } else { 80 };

    for i in 0..columns {
        for j in 0..rows {
            body_def.position = to_pos(Vec2 {
                x: i as f32 * d + 30.0,
                y: (j as f32 - rows as f32 / 2.0) * d,
            });
            let body_id = create_body(world, &body_def);
            create_polygon_shape(world, body_id, &shape_def, &box_shape);
        }
    }
}

// (CreateTumbler)
pub fn create_tumbler(world: &mut World) {
    let ground_id;
    {
        let body_def = default_body_def();
        ground_id = create_body(world, &body_def);
    }

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = to_pos(Vec2 { x: 0.0, y: 10.0 });
        let body_id = create_body(world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 50.0;

        let polygon = make_offset_box(0.5, 10.0, Vec2 { x: 10.0, y: 0.0 }, ROT_IDENTITY);
        create_polygon_shape(world, body_id, &shape_def, &polygon);
        let polygon = make_offset_box(0.5, 10.0, Vec2 { x: -10.0, y: 0.0 }, ROT_IDENTITY);
        create_polygon_shape(world, body_id, &shape_def, &polygon);
        let polygon = make_offset_box(10.0, 0.5, Vec2 { x: 0.0, y: 10.0 }, ROT_IDENTITY);
        create_polygon_shape(world, body_id, &shape_def, &polygon);
        let polygon = make_offset_box(10.0, 0.5, Vec2 { x: 0.0, y: -10.0 }, ROT_IDENTITY);
        create_polygon_shape(world, body_id, &shape_def, &polygon);

        let motor_speed = 25.0;

        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = ground_id;
        joint_def.base.body_id_b = body_id;
        joint_def.base.local_frame_a.p = Vec2 { x: 0.0, y: 10.0 };
        joint_def.base.local_frame_b.p = Vec2 { x: 0.0, y: 0.0 };
        joint_def.motor_speed = (PI / 180.0) * motor_speed;
        joint_def.max_motor_torque = 1e8;
        joint_def.enable_motor = true;

        create_revolute_joint(world, &joint_def);
    }

    let grid_count: i32 = if BENCHMARK_DEBUG { 20 } else { 45 };

    let polygon = make_box(0.125, 0.125);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();

    let mut y = -0.2 * grid_count as f32 + 10.0;
    for _ in 0..grid_count {
        let mut x = -0.2 * grid_count as f32;

        for _ in 0..grid_count {
            body_def.position = to_pos(Vec2 { x, y });
            let body_id = create_body(world, &body_def);

            create_polygon_shape(world, body_id, &shape_def, &polygon);

            x += 0.4;
        }

        y += 0.4;
    }
}

// (CreateWasher)
pub fn create_washer(world: &mut World) {
    let kinematic = true;

    {
        let body_def = default_body_def();
        // groundId is only used to anchor the (disabled) revolute joint branch.
        let _ground_id = create_body(world, &body_def);
    }

    {
        let motor_speed = 25.0;

        let mut body_def = default_body_def();
        body_def.position = to_pos(Vec2 { x: 0.0, y: 10.0 });

        if kinematic {
            body_def.type_ = BodyType::Kinematic;
            body_def.angular_velocity = (PI / 180.0) * motor_speed;
            body_def.linear_velocity = Vec2 {
                x: 0.001,
                y: -0.002,
            };
        } else {
            body_def.type_ = BodyType::Dynamic;
        }

        let body_id = create_body(world, &body_def);

        let shape_def = default_shape_def();

        let r0 = 14.0;
        let r1 = 16.0;
        let r2 = 18.0;

        let angle = PI / 18.0;
        let q = Rot {
            c: angle.cos(),
            s: angle.sin(),
        };
        let qo = Rot {
            c: (0.1 * angle).cos(),
            s: (0.1 * angle).sin(),
        };
        let mut u1 = Vec2 { x: 1.0, y: 0.0 };
        for i in 0..36 {
            let u2 = if i == 35 {
                Vec2 { x: 1.0, y: 0.0 }
            } else {
                rotate_vector(q, u1)
            };

            {
                let a1 = inv_rotate_vector(qo, u1);
                let a2 = rotate_vector(qo, u2);

                let p1 = mul_sv(r1, a1);
                let p2 = mul_sv(r2, a1);
                let p3 = mul_sv(r1, a2);
                let p4 = mul_sv(r2, a2);

                let points = [p1, p2, p3, p4];
                let hull = compute_hull(&points);

                let polygon = make_polygon(&hull, 0.0);
                create_polygon_shape(world, body_id, &shape_def, &polygon);
            }

            if i % 9 == 0 {
                let p1 = mul_sv(r0, u1);
                let p2 = mul_sv(r1, u1);
                let p3 = mul_sv(r0, u2);
                let p4 = mul_sv(r1, u2);

                let points = [p1, p2, p3, p4];
                let hull = compute_hull(&points);

                let polygon = make_polygon(&hull, 0.0);
                create_polygon_shape(world, body_id, &shape_def, &polygon);
            }

            u1 = u2;
        }
    }

    let grid_count: i32 = if BENCHMARK_DEBUG { 20 } else { 90 };
    let a = 0.1;

    let polygon = make_square(a);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let mut shape_def = default_shape_def();
    shape_def.enable_hit_events = true;

    let mut y = -1.1 * a * grid_count as f32 + 10.0;
    for _ in 0..grid_count {
        let mut x = -1.1 * a * grid_count as f32;

        for _ in 0..grid_count {
            body_def.position = to_pos(Vec2 { x, y });
            let body_id = create_body(world, &body_def);

            create_polygon_shape(world, body_id, &shape_def, &polygon);

            x += 2.1 * a;
        }

        y += 2.1 * a;
    }
}

// (CreateJunkyard)
pub fn create_junkyard(world: &mut World) {
    {
        let grid_size = 1.0;

        let body_def = default_body_def();
        let ground_id = create_body(world, &body_def);

        let shape_def = default_shape_def();

        let y = 0.0;
        let mut x = -80.0 * grid_size;
        for _ in 0..161 {
            let box_shape = make_offset_box(
                0.55 * grid_size,
                0.5 * grid_size,
                Vec2 { x, y },
                ROT_IDENTITY,
            );
            create_polygon_shape(world, ground_id, &shape_def, &box_shape);
            x += grid_size;
        }

        let mut y = grid_size;
        let x = -80.0 * grid_size;
        for _ in 0..50 {
            let box_shape = make_offset_box(
                0.5 * grid_size,
                0.55 * grid_size,
                Vec2 { x, y },
                ROT_IDENTITY,
            );
            create_polygon_shape(world, ground_id, &shape_def, &box_shape);
            y += grid_size;
        }

        let mut y = grid_size;
        let x = 80.0 * grid_size;
        for _ in 0..50 {
            let box_shape = make_offset_box(
                0.5 * grid_size,
                0.55 * grid_size,
                Vec2 { x, y },
                ROT_IDENTITY,
            );
            create_polygon_shape(world, ground_id, &shape_def, &box_shape);
            y += grid_size;
        }
    }

    let column_count = 200;
    let row_count: i32 = if BENCHMARK_DEBUG { 2 } else { 40 };

    let radius = 0.25;
    let polygon;
    {
        // Fibonacci sphere algorithm
        let phi = PI * (5.0f32.sqrt() - 1.0);
        let mut points = [Vec2 { x: 0.0, y: 0.0 }; 5];

        for (i, point) in points.iter_mut().enumerate() {
            let theta = phi * i as f32;
            let cs = compute_cos_sin(theta);
            point.x = radius * cs.cosine;
            point.y = radius * cs.sine;
        }

        let hull = compute_hull(&points);
        polygon = make_polygon(&hull, 0.0);
    }

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();

    let mut side = -0.1;
    let y_start = 15.0;

    for i in 0..column_count {
        let x = 1.5 * (2.0 * i as f32 - column_count as f32) * radius;

        for j in 0..row_count {
            let y = 4.0 * j as f32 * radius + y_start;

            body_def.position = to_pos(Vec2 { x: x + side, y });
            side = -side;

            let body_id = create_body(world, &body_def);
            create_polygon_shape(world, body_id, &shape_def, &polygon);
        }
    }

    body_def.type_ = BodyType::Kinematic;
    body_def.position = to_pos(VEC2_ZERO);
    let pusher_id = create_body(world, &body_def);
    JUNKYARD_PUSHER_ID.with(|id| id.set(pusher_id));
    let box_shape = make_offset_box(2.0, 4.0, Vec2 { x: 0.0, y: 4.0 }, ROT_IDENTITY);
    create_polygon_shape(world, pusher_id, &shape_def, &box_shape);
}

// (StepJunkyard)
pub fn step_junkyard(world: &mut World, step_count: i32) -> f32 {
    let time_step = 1.0 / 60.0;
    let time = time_step * step_count as f32;
    let cs = compute_cos_sin(0.2 * time);
    let target = WorldTransform {
        p: to_pos(Vec2 {
            x: 60.0 * cs.sine,
            y: 0.0,
        }),
        q: ROT_IDENTITY,
    };
    let pusher_id = JUNKYARD_PUSHER_ID.with(|id| id.get());
    body_set_target_transform(world, pusher_id, target, time_step, true);
    0.0
}
