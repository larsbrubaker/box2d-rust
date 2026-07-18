// Pyramid-family benchmark scenes: joint grid, large pyramid, many pyramids,
// and the compound-shape barrel. Ported from benchmarks.c.

use box2d_rust::body::create_body;
use box2d_rust::collision::{Circle, Segment};
use box2d_rust::geometry::{make_box, make_offset_box, make_polygon, make_square};
use box2d_rust::hull::compute_hull;
use box2d_rust::id::BodyId;
use box2d_rust::joint::create_revolute_joint;
use box2d_rust::math_functions::{to_pos, Vec2, ROT_IDENTITY};
use box2d_rust::shape::{create_circle_shape, create_polygon_shape, create_segment_shape};
use box2d_rust::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, BodyType,
};
use box2d_rust::world::{world_enable_sleeping, World};

use super::BENCHMARK_DEBUG;

// (CreateJointGrid)
pub fn create_joint_grid(world: &mut World) {
    world_enable_sleeping(world, false);

    let n: i32 = if BENCHMARK_DEBUG { 20 } else { 100 };

    let mut bodies: Vec<BodyId> = Vec::with_capacity((n * n) as usize);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.filter.category_bits = 2;
    shape_def.filter.mask_bits = (!2u32) as u64;

    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.4,
    };

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.draw_scale = 0.4;

    let mut body_def = default_body_def();

    for k in 0..n {
        for i in 0..n {
            let fk = k as f32;
            let fi = i as f32;

            if k >= n / 2 - 3 && k <= n / 2 + 3 && i == 0 {
                body_def.type_ = BodyType::Static;
            } else {
                body_def.type_ = BodyType::Dynamic;
            }

            body_def.position = to_pos(Vec2 { x: fk, y: -fi });

            let body = create_body(world, &body_def);
            create_circle_shape(world, body, &shape_def, &circle);

            let index = bodies.len();

            if i > 0 {
                joint_def.base.body_id_a = bodies[index - 1];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a.p = Vec2 { x: 0.0, y: -0.5 };
                joint_def.base.local_frame_b.p = Vec2 { x: 0.0, y: 0.5 };
                create_revolute_joint(world, &joint_def);
            }

            if k > 0 {
                joint_def.base.body_id_a = bodies[index - n as usize];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a.p = Vec2 { x: 0.5, y: 0.0 };
                joint_def.base.local_frame_b.p = Vec2 { x: -0.5, y: 0.0 };
                create_revolute_joint(world, &joint_def);
            }

            bodies.push(body);
        }
    }
}

// (CreateLargePyramid)
pub fn create_large_pyramid(world: &mut World) {
    world_enable_sleeping(world, false);

    let base_count: i32 = if BENCHMARK_DEBUG { 20 } else { 100 };

    {
        let mut body_def = default_body_def();
        body_def.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
        let ground_id = create_body(world, &body_def);

        let box_shape = make_box(100.0, 1.0);
        let shape_def = default_shape_def();
        create_polygon_shape(world, ground_id, &shape_def, &box_shape);
    }

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    let a = 0.5;
    let box_shape = make_square(a);

    let shift = 1.0 * a;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift;

        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - a * base_count as f32;

            body_def.position = to_pos(Vec2 { x, y });

            let body_id = create_body(world, &body_def);
            create_polygon_shape(world, body_id, &shape_def, &box_shape);
        }
    }
}

// (static CreateSmallPyramid)
fn create_small_pyramid(
    world: &mut World,
    base_count: i32,
    extent: f32,
    center_x: f32,
    base_y: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    let shape_def = default_shape_def();

    let box_shape = make_square(extent);

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent + base_y;

        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;
            body_def.position = to_pos(Vec2 { x, y });

            let body_id = create_body(world, &body_def);
            create_polygon_shape(world, body_id, &shape_def, &box_shape);
        }
    }
}

// (CreateManyPyramids)
pub fn create_many_pyramids(world: &mut World) {
    world_enable_sleeping(world, false);

    let base_count = 10;
    let extent = 0.5;
    let row_count: i32 = if BENCHMARK_DEBUG { 5 } else { 20 };
    let column_count: i32 = if BENCHMARK_DEBUG { 5 } else { 20 };

    let body_def = default_body_def();
    let ground_id = create_body(world, &body_def);

    let ground_delta_y = 2.0 * extent * (base_count as f32 + 1.0);
    let ground_width = 2.0 * extent * column_count as f32 * (base_count as f32 + 1.0);
    let shape_def = default_shape_def();

    let mut ground_y = 0.0;

    for _ in 0..row_count {
        let segment = Segment {
            point1: Vec2 {
                x: -0.5 * ground_width,
                y: ground_y,
            },
            point2: Vec2 {
                x: 0.5 * ground_width,
                y: ground_y,
            },
        };
        create_segment_shape(world, ground_id, &shape_def, &segment);
        ground_y += ground_delta_y;
    }

    let base_width = 2.0 * extent * base_count as f32;
    let mut base_y = 0.0;

    for _ in 0..row_count {
        for j in 0..column_count {
            let center_x =
                -0.5 * ground_width + j as f32 * (base_width + 2.0 * extent) + 2.0 * extent;
            create_small_pyramid(world, base_count, extent, center_x, base_y);
        }

        base_y += ground_delta_y;
    }
}

// (CreateCompounds)
// Lifted from samples/sample_benchmark.cpp BenchmarkBarrel (e_compoundShape
// branch). Each dynamic body is a compound of two triangular polygon shapes.
pub fn create_compounds(world: &mut World) {
    {
        let grid_size = 1.0;

        let body_def = default_body_def();
        let ground_id = create_body(world, &body_def);

        let shape_def = default_shape_def();

        let y = 0.0;
        let mut x = -40.0 * grid_size;
        for _ in 0..81 {
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
        let x = -40.0 * grid_size;
        for _ in 0..100 {
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
        let x = 40.0 * grid_size;
        for _ in 0..100 {
            let box_shape = make_offset_box(
                0.5 * grid_size,
                0.55 * grid_size,
                Vec2 { x, y },
                ROT_IDENTITY,
            );
            create_polygon_shape(world, ground_id, &shape_def, &box_shape);
            y += grid_size;
        }

        let segment = Segment {
            point1: Vec2 {
                x: -800.0,
                y: -80.0,
            },
            point2: Vec2 { x: 800.0, y: -80.0 },
        };
        create_segment_shape(world, ground_id, &shape_def, &segment);
    }

    let column_count: i32 = if BENCHMARK_DEBUG { 10 } else { 20 };
    let row_count: i32 = if BENCHMARK_DEBUG { 40 } else { 150 };

    let left_points = [
        Vec2 { x: -1.0, y: 0.0 },
        Vec2 { x: 0.5, y: 1.0 },
        Vec2 { x: 0.0, y: 2.0 },
    ];
    let left_hull = compute_hull(&left_points);
    let left = make_polygon(&left_hull, 0.0);

    let right_points = [
        Vec2 { x: 1.0, y: 0.0 },
        Vec2 { x: -0.5, y: 1.0 },
        Vec2 { x: 0.0, y: 2.0 },
    ];
    let right_hull = compute_hull(&right_points);
    let right = make_polygon(&right_hull, 0.0);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.material.friction = 0.5;

    // Match the sample exactly: centery is computed before shift is reset for
    // the compound branch.
    let shift = 2.0;
    let extray = 0.25;
    let mut side = 0.25;
    let centerx = shift * column_count as f32 / 2.0 - 1.0;
    let centery = 1.15 / 2.0;
    let y_start = 5.0;

    for i in 0..column_count {
        let x = i as f32 * shift - centerx;

        for j in 0..row_count {
            let y = j as f32 * (shift + extray) + centery + y_start;

            body_def.position = to_pos(Vec2 { x: x + side, y });
            side = -side;

            let body_id = create_body(world, &body_def);
            create_polygon_shape(world, body_id, &shape_def, &left);
            create_polygon_shape(world, body_id, &shape_def, &right);
        }
    }
}
