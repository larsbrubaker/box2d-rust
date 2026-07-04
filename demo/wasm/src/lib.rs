// WASM bindings for the browser demos. Every value shown on the demo site is computed
// by the ported Rust code, never re-implemented in JavaScript.

use box2d_rust::math_functions as m;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn version() -> String {
    box2d_rust::VERSION.to_string()
}

/// Deterministic cosine/sine from the ported `b2ComputeCosSin`. Returns [cos, sin].
#[wasm_bindgen]
pub fn compute_cos_sin(radians: f32) -> Vec<f32> {
    let cs = m::compute_cos_sin(radians);
    vec![cs.cosine, cs.sine]
}

/// Deterministic arctangent from the ported `b2Atan2`.
#[wasm_bindgen]
pub fn atan2(y: f32, x: f32) -> f32 {
    m::atan2(y, x)
}

/// Build a regular polygon rotated by `angle`, centered at (cx, cy).
/// Returns interleaved [x0, y0, x1, y1, ...] computed with the ported Rot/Transform math.
#[wasm_bindgen]
pub fn polygon_points(sides: u32, radius: f32, angle: f32, cx: f32, cy: f32) -> Vec<f32> {
    let q = m::make_rot(angle);
    let t = m::Transform {
        p: m::Vec2 { x: cx, y: cy },
        q,
    };

    let mut out = Vec::with_capacity(2 * sides as usize);
    for i in 0..sides {
        let vertex_angle = 2.0 * m::PI * i as f32 / sides as f32;
        let cs = m::compute_cos_sin(vertex_angle);
        let local = m::Vec2 {
            x: radius * cs.cosine,
            y: radius * cs.sine,
        };
        let world = m::transform_point(t, local);
        out.push(world.x);
        out.push(world.y);
    }
    out
}

// ---------------------------------------------------------------------------
// Geometry demo: a small scene queried with the ported geometry + distance
// modules. World units are meters; the page scales to pixels.
// ---------------------------------------------------------------------------

use box2d_rust::collision::{Capsule, Circle, RayCastInput, Segment};
use box2d_rust::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use box2d_rust::geometry::{
    make_polygon, ray_cast_capsule, ray_cast_circle, ray_cast_polygon, ray_cast_segment,
};
use box2d_rust::hull::compute_hull;
use box2d_rust::math_functions::{Vec2, TRANSFORM_IDENTITY};

fn scene_polygon() -> box2d_rust::collision::Polygon {
    // A convex pentagon via the ported quickhull.
    let pts = [
        Vec2 { x: -1.2, y: -0.8 },
        Vec2 { x: 0.0, y: -1.2 },
        Vec2 { x: 1.2, y: -0.5 },
        Vec2 { x: 0.8, y: 0.9 },
        Vec2 { x: -0.8, y: 1.0 },
    ];
    let hull = compute_hull(&pts);
    make_polygon(&hull, 0.0)
}

fn scene_circle() -> Circle {
    Circle {
        center: Vec2 { x: 3.2, y: 0.6 },
        radius: 0.8,
    }
}

fn scene_capsule() -> Capsule {
    Capsule {
        center1: Vec2 { x: -3.6, y: -0.6 },
        center2: Vec2 { x: -2.4, y: 0.9 },
        radius: 0.5,
    }
}

fn scene_segment() -> Segment {
    Segment {
        point1: Vec2 { x: -1.6, y: 2.0 },
        point2: Vec2 { x: 1.6, y: 2.4 },
    }
}

/// The demo scene outline geometry, one shape per call:
/// 0 = polygon vertices [x,y]*, 1 = circle [cx, cy, r],
/// 2 = capsule [c1x, c1y, c2x, c2y, r], 3 = segment [p1x, p1y, p2x, p2y].
#[wasm_bindgen]
pub fn scene_shape(index: u32) -> Vec<f32> {
    match index {
        0 => {
            let p = scene_polygon();
            let mut out = Vec::new();
            for i in 0..p.count as usize {
                out.push(p.vertices[i].x);
                out.push(p.vertices[i].y);
            }
            out
        }
        1 => {
            let c = scene_circle();
            vec![c.center.x, c.center.y, c.radius]
        }
        2 => {
            let c = scene_capsule();
            vec![c.center1.x, c.center1.y, c.center2.x, c.center2.y, c.radius]
        }
        _ => {
            let s = scene_segment();
            vec![s.point1.x, s.point1.y, s.point2.x, s.point2.y]
        }
    }
}

/// Cast a ray against every scene shape with the ported local-space ray casts.
/// Returns [hit, fraction, px, py, nx, ny] per shape (4 shapes, 24 floats).
#[wasm_bindgen]
pub fn ray_cast_scene(ox: f32, oy: f32, tx: f32, ty: f32) -> Vec<f32> {
    let input = RayCastInput {
        origin: Vec2 { x: ox, y: oy },
        translation: Vec2 { x: tx, y: ty },
        max_fraction: 1.0,
    };

    let outputs = [
        ray_cast_polygon(&scene_polygon(), &input),
        ray_cast_circle(&scene_circle(), &input),
        ray_cast_capsule(&scene_capsule(), &input),
        ray_cast_segment(&scene_segment(), &input, false),
    ];

    let mut out = Vec::with_capacity(24);
    for o in outputs {
        out.push(if o.hit { 1.0 } else { 0.0 });
        out.push(o.fraction);
        out.push(o.point.x);
        out.push(o.point.y);
        out.push(o.normal.x);
        out.push(o.normal.y);
    }
    out
}

/// Contact manifold between a fixed unit box at the origin and a moving shape,
/// using the ported b2Collide* functions. `kind`: 0 = box, 1 = circle,
/// 2 = capsule. The moving shape sits at (bx, by) rotated by `angle`.
/// Returns [nx, ny, pointCount, p0x, p0y, sep0, p1x, p1y, sep1].
#[wasm_bindgen]
pub fn collide_with_box(kind: u32, bx: f32, by: f32, angle: f32) -> Vec<f32> {
    use box2d_rust::collision::Capsule;
    use box2d_rust::geometry::make_box;
    use box2d_rust::manifold::{
        collide_polygon_and_capsule, collide_polygon_and_circle, collide_polygons,
    };
    use box2d_rust::math_functions::{make_rot, Transform};

    let box_a = make_box(1.0, 1.0);
    let xf = Transform {
        p: Vec2 { x: bx, y: by },
        q: make_rot(angle),
    };

    let m = match kind {
        1 => {
            let circle = box2d_rust::collision::Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.6,
            };
            collide_polygon_and_circle(&box_a, &circle, xf)
        }
        2 => {
            let capsule = Capsule {
                center1: Vec2 { x: -0.6, y: 0.0 },
                center2: Vec2 { x: 0.6, y: 0.0 },
                radius: 0.35,
            };
            collide_polygon_and_capsule(&box_a, &capsule, xf)
        }
        _ => {
            let box_b = make_box(0.7, 0.7);
            collide_polygons(&box_a, &box_b, xf)
        }
    };

    let mut out = vec![m.normal.x, m.normal.y, m.point_count as f32];
    for i in 0..2 {
        out.push(m.points[i].point.x);
        out.push(m.points[i].point.y);
        out.push(m.points[i].separation);
    }
    out
}

/// GJK closest points between the scene polygon and a probe triangle centered
/// at (bx, by), using the ported b2ShapeDistance.
/// Returns [pax, pay, pbx, pby, distance, iterations].
#[wasm_bindgen]
pub fn closest_points(bx: f32, by: f32) -> Vec<f32> {
    let p = scene_polygon();
    let probe = [
        Vec2 {
            x: bx - 0.4,
            y: by - 0.3,
        },
        Vec2 {
            x: bx + 0.4,
            y: by - 0.3,
        },
        Vec2 { x: bx, y: by + 0.4 },
    ];

    let input = DistanceInput {
        proxy_a: make_proxy(&p.vertices[..p.count as usize], 0.0),
        proxy_b: make_proxy(&probe, 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    vec![
        output.point_a.x,
        output.point_a.y,
        output.point_b.x,
        output.point_b.y,
        output.distance,
        output.iterations as f32,
    ]
}

use box2d_rust::body::{create_body, get_body_full_id, get_body_transform};
use box2d_rust::geometry::make_box;
use box2d_rust::shape::{create_circle_shape, create_polygon_shape};
use box2d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box2d_rust::world::{world_step, World};

/// A live physics world for the Bodies/Stacking demos. Every step runs the
/// ported b2World_Step pipeline: broad phase, narrow phase, graph-colored
/// soft-constraint solver, restitution, and sleeping.
#[wasm_bindgen]
pub struct SimWorld {
    world: World,
    /// Raw body indices in creation order; positions() reports in this order.
    bodies: Vec<i32>,
}

#[wasm_bindgen]
impl SimWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_y: f32) -> SimWorld {
        let mut world_def = default_world_def();
        world_def.gravity = m::Vec2 {
            x: 0.0,
            y: gravity_y,
        };
        SimWorld {
            world: World::new(&world_def),
            bodies: Vec::new(),
        }
    }

    /// Static box (the ground or a wall). Returns the demo body index.
    pub fn add_static_box(&mut self, x: f32, y: f32, hx: f32, hy: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let shape_def = default_shape_def();
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Dynamic box. Returns the demo body index.
    pub fn add_box(&mut self, x: f32, y: f32, hx: f32, hy: f32, density: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Dynamic circle. Returns the demo body index.
    pub fn add_circle(&mut self, x: f32, y: f32, radius: f32, density: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        shape_def.material.restitution = 0.2;
        let circle = box2d_rust::collision::Circle {
            center: m::VEC2_ZERO,
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Advance the simulation. (b2World_Step)
    pub fn step(&mut self, dt: f32, sub_step_count: i32) {
        world_step(&mut self.world, dt, sub_step_count);
    }

    /// Interleaved [x, y, angle] for every demo body, in creation order.
    pub fn positions(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(3 * self.bodies.len());
        for &body_index in &self.bodies {
            let transform = get_body_transform(&self.world, body_index);
            out.push(transform.p.x as f32);
            out.push(transform.p.y as f32);
            out.push(m::rot_get_angle(transform.q));
        }
        out
    }

    /// Number of awake bodies (sleeping islands leave this count).
    pub fn awake_body_count(&self) -> i32 {
        self.world.solver_sets[box2d_rust::solver_set::AWAKE_SET as usize]
            .body_sims
            .len() as i32
    }

    /// Live contact count.
    pub fn contact_count(&self) -> i32 {
        self.world.contact_id_pool.id_count()
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
}
