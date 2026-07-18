// The Rain benchmark scene: rows of ragdoll "humans" that spawn and recycle
// over time. Ported from benchmarks.c. The C global g_rainData becomes
// thread-local state here.

use std::cell::RefCell;

use box2d_rust::body::create_body;
use box2d_rust::geometry::make_offset_box;
use box2d_rust::math_functions::{offset_pos, to_pos, Vec2, ROT_IDENTITY};
use box2d_rust::shape::create_polygon_shape;
use box2d_rust::types::{default_body_def, default_shape_def};
use box2d_rust::world::World;

use crate::human::{create_human, destroy_human, Human};

use super::BENCHMARK_DEBUG;

// (RainConstants)
const RAIN_ROW_COUNT: i32 = if BENCHMARK_DEBUG { 3 } else { 5 };
const RAIN_COLUMN_COUNT: i32 = if BENCHMARK_DEBUG { 10 } else { 40 };
const RAIN_GROUP_SIZE: i32 = if BENCHMARK_DEBUG { 2 } else { 5 };

// (RainData) — groups[row * column] each holding RAIN_GROUP_SIZE humans.
struct RainData {
    groups: Vec<Vec<Human>>,
    grid_size: f32,
    grid_count: i32,
    column_count: i32,
    column_index: i32,
}

impl RainData {
    fn new() -> Self {
        RainData {
            groups: Vec::new(),
            grid_size: 0.0,
            grid_count: 0,
            column_count: 0,
            column_index: 0,
        }
    }

    // memset( &g_rainData, 0, sizeof( g_rainData ) )
    fn reset(&mut self) {
        let group_count = (RAIN_ROW_COUNT * RAIN_COLUMN_COUNT) as usize;
        self.groups = (0..group_count)
            .map(|_| vec![Human::default(); RAIN_GROUP_SIZE as usize])
            .collect();
        self.grid_size = 0.0;
        self.grid_count = 0;
        self.column_count = 0;
        self.column_index = 0;
    }
}

thread_local! {
    static RAIN_DATA: RefCell<RainData> = RefCell::new(RainData::new());
}

// (CreateRain)
pub fn create_rain(world: &mut World) {
    RAIN_DATA.with(|rd| rd.borrow_mut().reset());

    let grid_size = 0.5;
    let grid_count = if BENCHMARK_DEBUG { 200 } else { 500 };
    RAIN_DATA.with(|rd| {
        let mut rd = rd.borrow_mut();
        rd.grid_size = grid_size;
        rd.grid_count = grid_count;
    });

    {
        let body_def = default_body_def();
        let ground_id = create_body(world, &body_def);

        let shape_def = default_shape_def();
        let mut y = 0.0;
        let width = grid_size;
        let height = grid_size;

        for _ in 0..RAIN_ROW_COUNT {
            let mut x = -0.5 * grid_count as f32 * grid_size;
            for _ in 0..=grid_count {
                let box_shape =
                    make_offset_box(0.5 * width, 0.5 * height, Vec2 { x, y }, ROT_IDENTITY);
                create_polygon_shape(world, ground_id, &shape_def, &box_shape);

                x += grid_size;
            }

            y += 45.0;
        }
    }

    RAIN_DATA.with(|rd| {
        let mut rd = rd.borrow_mut();
        rd.column_count = 0;
        rd.column_index = 0;
    });
}

// (CreateGroup)
fn create_group(world: &mut World, row_index: i32, column_index: i32) {
    debug_assert!(row_index < RAIN_ROW_COUNT && column_index < RAIN_COLUMN_COUNT);

    RAIN_DATA.with(|rd| {
        let mut rd = rd.borrow_mut();

        let group_index = (row_index * RAIN_COLUMN_COUNT + column_index) as usize;

        let span = rd.grid_count as f32 * rd.grid_size;
        let group_distance = 1.0 * span / RAIN_COLUMN_COUNT as f32;

        let mut position = to_pos(Vec2 {
            x: -0.5 * span + group_distance * (column_index as f32 + 0.5),
            y: 40.0 + 45.0 * row_index as f32,
        });

        let scale = 1.0;
        let joint_friction = 0.05;
        let joint_hertz = 5.0;
        let joint_damping = 0.5;

        for i in 0..RAIN_GROUP_SIZE {
            create_human(
                &mut rd.groups[group_index][i as usize],
                world,
                position,
                scale,
                joint_friction,
                joint_hertz,
                joint_damping,
                i + 1,
                0,
                false,
            );
            position = offset_pos(position, Vec2 { x: 0.5, y: 0.0 });
        }
    });
}

// (DestroyGroup)
fn destroy_group(world: &mut World, row_index: i32, column_index: i32) {
    debug_assert!(row_index < RAIN_ROW_COUNT && column_index < RAIN_COLUMN_COUNT);

    RAIN_DATA.with(|rd| {
        let mut rd = rd.borrow_mut();
        let group_index = (row_index * RAIN_COLUMN_COUNT + column_index) as usize;

        for i in 0..RAIN_GROUP_SIZE {
            destroy_human(world, &mut rd.groups[group_index][i as usize]);
        }
    });
}

// (StepRain)
pub fn step_rain(world: &mut World, step_count: i32) -> f32 {
    let delay: i32 = if BENCHMARK_DEBUG { 0x1F } else { 0x7 };

    if (step_count & delay) == 0 {
        let column_count = RAIN_DATA.with(|rd| rd.borrow().column_count);

        if column_count < RAIN_COLUMN_COUNT {
            for i in 0..RAIN_ROW_COUNT {
                create_group(world, i, column_count);
            }

            RAIN_DATA.with(|rd| rd.borrow_mut().column_count += 1);
        } else {
            let column_index = RAIN_DATA.with(|rd| rd.borrow().column_index);

            for i in 0..RAIN_ROW_COUNT {
                destroy_group(world, i, column_index);
                create_group(world, i, column_index);
            }

            RAIN_DATA.with(|rd| {
                let mut rd = rd.borrow_mut();
                rd.column_index = (rd.column_index + 1) % RAIN_COLUMN_COUNT;
            });
        }
    }

    0.0
}
