// Repro of the demo scenes: Bodies (mixed shower into container) and
// Stacking (pyramid + heavy ball).
use box2d_rust::body::{create_body, get_body_full_id, get_body_transform};
use box2d_rust::geometry::make_box;
use box2d_rust::math_functions as m;
use box2d_rust::shape::{create_circle_shape, create_polygon_shape};
use box2d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box2d_rust::world::{world_step, World};

fn add_static_box(world: &mut World, x: f32, y: f32, hx: f32, hy: f32) -> i32 {
    let mut body_def = default_body_def();
    body_def.position = m::to_pos(m::Vec2 { x, y });
    let id = create_body(world, &body_def);
    let def = default_shape_def();
    let poly = make_box(hx, hy);
    create_polygon_shape(world, id, &def, &poly);
    get_body_full_id(world, id)
}

fn add_box(world: &mut World, x: f32, y: f32, hx: f32, hy: f32) -> i32 {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = m::to_pos(m::Vec2 { x, y });
    let id = create_body(world, &body_def);
    let mut def = default_shape_def();
    def.density = 1.0;
    def.material.friction = 0.3;
    let poly = make_box(hx, hy);
    create_polygon_shape(world, id, &def, &poly);
    get_body_full_id(world, id)
}

fn add_circle(world: &mut World, x: f32, y: f32, r: f32, density: f32) -> i32 {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = m::to_pos(m::Vec2 { x, y });
    let id = create_body(world, &body_def);
    let mut def = default_shape_def();
    def.density = density;
    def.material.friction = 0.3;
    def.material.restitution = 0.2;
    let circle = box2d_rust::collision::Circle {
        center: m::VEC2_ZERO,
        radius: r,
    };
    create_circle_shape(world, id, &def, &circle);
    get_body_full_id(world, id)
}

fn main() {
    // === Bodies demo scene ===
    let mut wd = default_world_def();
    wd.gravity = m::Vec2 { x: 0.0, y: -10.0 };
    let mut world = World::new(&wd);

    add_static_box(&mut world, 0.0, -0.5, 13.0, 0.5);
    add_static_box(&mut world, -12.2, 2.0, 0.3, 2.0);
    add_static_box(&mut world, 12.2, 2.0, 0.3, 2.0);

    let mut tracked = Vec::new();
    for i in 0..24usize {
        let x = -6.0 + (i % 8) as f32 * 1.7 + 0.13 * (i % 3) as f32;
        let y = 5.0 + (i / 8) as f32 * 1.6;
        if i % 2 == 0 {
            let hx = 0.25 + 0.2 * ((i * 7) % 3) as f32 * 0.5;
            tracked.push(add_box(&mut world, x, y, hx, hx));
        } else {
            let r = 0.22 + 0.16 * ((i * 5) % 3) as f32 * 0.5;
            tracked.push(add_circle(&mut world, x, y, r, 1.0));
        }
    }

    for step in 0..600 {
        world_step(&mut world, 1.0 / 60.0, 4);
        if step % 100 == 0 {
            let t = get_body_transform(&world, tracked[0]);
            println!(
                "bodies step {step}: body0 = ({:.3}, {:.3}), contacts = {}",
                t.p.x,
                t.p.y,
                world.contact_id_pool.id_count()
            );
        }
    }
    println!("BODIES SCENE OK");

    // === Stacking demo scene ===
    let mut world = World::new(&wd);
    add_static_box(&mut world, 0.0, -0.5, 11.0, 0.5);
    let h = 0.4f32;
    let base = 9i32;
    for row in 0..base {
        let count = base - row;
        let y = h + row as f32 * 2.0 * h;
        for i in 0..count {
            let x = (i as f32 - (count - 1) as f32 / 2.0) * 2.05 * h;
            add_box(&mut world, x, y, h, h);
        }
    }
    for _ in 0..600 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }
    println!(
        "stacking settled: awake = {}",
        world.solver_sets[box2d_rust::solver_set::AWAKE_SET as usize]
            .body_sims
            .len()
    );
    // Drop the heavy ball
    add_circle(&mut world, 0.3, 9.0, 0.5, 4.0);
    for _ in 0..600 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }
    println!("STACKING SCENE OK");
}
