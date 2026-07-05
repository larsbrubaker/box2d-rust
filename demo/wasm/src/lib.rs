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

use box2d_rust::body::{create_body, get_body_full_id, get_body_transform, make_body_id};
use box2d_rust::geometry::make_box;
use box2d_rust::id::JointId;
use box2d_rust::joint::{
    create_distance_joint, create_revolute_joint, joint_get_body_a, joint_get_body_b,
    joint_get_local_frame_a, joint_get_local_frame_b,
};
use box2d_rust::math_functions::{inv_transform_world_point, transform_world_point};
use box2d_rust::shape::{create_circle_shape, create_polygon_shape};
use box2d_rust::types::{
    default_body_def, default_distance_joint_def, default_revolute_joint_def, default_shape_def,
    default_world_def, BodyType,
};
use box2d_rust::world::{
    world_cast_mover, world_collide_mover, world_enable_continuous, world_get_contact_events,
    world_get_sensor_events, world_step, World,
};

/// A live physics world for the Bodies/Stacking demos. Every step runs the
/// ported b2World_Step pipeline: broad phase, narrow phase, graph-colored
/// soft-constraint solver, restitution, and sleeping.
#[wasm_bindgen]
pub struct SimWorld {
    world: World,
    /// Raw body indices in creation order; positions() reports in this order.
    bodies: Vec<i32>,
    /// Joint ids in creation order; joint_anchors() reports in this order.
    joints: Vec<JointId>,
    /// Character mover state (not a body; driven by the mover queries).
    mover_position: m::Pos,
    mover_velocity: m::Vec2,
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
            joints: Vec::new(),
            mover_position: m::POS_ZERO,
            mover_velocity: m::VEC2_ZERO,
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

    /// Dynamic box with an initial rotation, for hinge chains. Returns the
    /// demo body index.
    pub fn add_box_rotated(
        &mut self,
        x: f32,
        y: f32,
        hx: f32,
        hy: f32,
        density: f32,
        angle: f32,
    ) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        body_def.rotation = m::make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Hinge two demo bodies at a world-space pivot with the falling-hinges
    /// tuning: limited, sprung, and motorized. (b2CreateRevoluteJoint)
    pub fn add_hinge_joint(&mut self, index_a: usize, index_b: usize, px: f32, py: f32) -> usize {
        let pivot = m::to_pos(m::Vec2 { x: px, y: py });
        let body_a = self.bodies[index_a];
        let body_b = self.bodies[index_b];
        let xf_a = get_body_transform(&self.world, body_a);
        let xf_b = get_body_transform(&self.world, body_b);

        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = make_body_id(&self.world, body_a);
        joint_def.base.body_id_b = make_body_id(&self.world, body_b);
        joint_def.base.local_frame_a.p = inv_transform_world_point(xf_a, pivot);
        joint_def.base.local_frame_b.p = inv_transform_world_point(xf_b, pivot);
        joint_def.enable_limit = true;
        joint_def.lower_angle = -0.1 * m::PI;
        joint_def.upper_angle = 0.2 * m::PI;
        joint_def.enable_spring = true;
        joint_def.hertz = 1.0;
        joint_def.damping_ratio = 1.0;
        joint_def.enable_motor = true;
        joint_def.max_motor_torque = 0.25;
        joint_def.base.constraint_hertz = 60.0;
        joint_def.base.constraint_damping_ratio = 0.0;

        let joint_id = create_revolute_joint(&mut self.world, &joint_def);
        self.joints.push(joint_id);
        self.joints.len() - 1
    }

    /// Connect two demo bodies with a rigid distance joint between world-space
    /// anchors. (b2CreateDistanceJoint)
    pub fn add_distance_joint(
        &mut self,
        index_a: usize,
        index_b: usize,
        ax: f32,
        ay: f32,
        bx: f32,
        by: f32,
    ) -> usize {
        let anchor_a = m::to_pos(m::Vec2 { x: ax, y: ay });
        let anchor_b = m::to_pos(m::Vec2 { x: bx, y: by });
        let body_a = self.bodies[index_a];
        let body_b = self.bodies[index_b];
        let xf_a = get_body_transform(&self.world, body_a);
        let xf_b = get_body_transform(&self.world, body_b);

        let mut joint_def = default_distance_joint_def();
        joint_def.base.body_id_a = make_body_id(&self.world, body_a);
        joint_def.base.body_id_b = make_body_id(&self.world, body_b);
        joint_def.base.local_frame_a.p = inv_transform_world_point(xf_a, anchor_a);
        joint_def.base.local_frame_b.p = inv_transform_world_point(xf_b, anchor_b);
        joint_def.length = m::length(m::sub_pos(anchor_b, anchor_a));

        let joint_id = create_distance_joint(&mut self.world, &joint_def);
        self.joints.push(joint_id);
        self.joints.len() - 1
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Interleaved world anchors [ax, ay, bx, by] for every joint, in
    /// creation order, for drawing noodles.
    pub fn joint_anchors(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(4 * self.joints.len());
        for &joint_id in &self.joints {
            let body_a = joint_get_body_a(&self.world, joint_id);
            let body_b = joint_get_body_b(&self.world, joint_id);
            let index_a = get_body_full_id(&self.world, body_a);
            let index_b = get_body_full_id(&self.world, body_b);
            let xf_a = get_body_transform(&self.world, index_a);
            let xf_b = get_body_transform(&self.world, index_b);
            let frame_a = joint_get_local_frame_a(&self.world, joint_id);
            let frame_b = joint_get_local_frame_b(&self.world, joint_id);
            let world_a = transform_world_point(xf_a, frame_a.p);
            let world_b = transform_world_point(xf_b, frame_b.p);
            out.push(world_a.x as f32);
            out.push(world_a.y as f32);
            out.push(world_b.x as f32);
            out.push(world_b.y as f32);
        }
        out
    }

    /// Dynamic circle with hit events enabled and lively restitution, for the
    /// Events demo.
    pub fn add_bouncy_ball(&mut self, x: f32, y: f32, radius: f32, restitution: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.material.friction = 0.3;
        shape_def.material.restitution = restitution;
        shape_def.enable_hit_events = true;
        let circle = box2d_rust::collision::Circle {
            center: m::VEC2_ZERO,
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Static sensor box that reports begin/end overlap events.
    pub fn add_sensor_box(&mut self, x: f32, y: f32, hx: f32, hy: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.position = m::to_pos(m::Vec2 { x, y });
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Enable sensor events on the most recently added body's shape so
    /// dynamic bodies can visit sensors.
    pub fn enable_sensor_visitor(&mut self, index: usize) {
        let body_index = self.bodies[index];
        let mut shape_id = self.world.bodies[body_index as usize].head_shape_id;
        while shape_id != box2d_rust::core::NULL_INDEX {
            self.world.shapes[shape_id as usize].enable_sensor_events = true;
            shape_id = self.world.shapes[shape_id as usize].next_shape_id;
        }
    }

    /// Contact/sensor event counts for the last step:
    /// [contactBegin, contactEnd, hit, sensorBegin, sensorEnd].
    pub fn event_counts(&self) -> Vec<u32> {
        let contact_events = world_get_contact_events(&self.world);
        let sensor_events = world_get_sensor_events(&self.world);
        vec![
            contact_events.begin_events.len() as u32,
            contact_events.end_events.len() as u32,
            contact_events.hit_events.len() as u32,
            sensor_events.begin_events.len() as u32,
            sensor_events.end_events.len() as u32,
        ]
    }

    /// Hit events from the last step as [x, y, approachSpeed]*.
    pub fn hit_events(&self) -> Vec<f32> {
        let contact_events = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(3 * contact_events.hit_events.len());
        for hit in contact_events.hit_events {
            out.push(hit.point.x as f32);
            out.push(hit.point.y as f32);
            out.push(hit.approach_speed);
        }
        out
    }

    /// Fast dynamic circle bullet for the Continuous demo. Continuous
    /// collision (TOI) keeps it from tunneling through thin walls.
    pub fn add_bullet(&mut self, x: f32, y: f32, radius: f32, vx: f32, vy: f32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        body_def.gravity_scale = 0.0;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        body_def.linear_velocity = m::Vec2 { x: vx, y: vy };
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.material.restitution = 0.1;
        let circle = box2d_rust::collision::Circle {
            center: m::VEC2_ZERO,
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Toggle continuous collision to demonstrate tunneling.
    /// (b2World_EnableContinuous)
    pub fn set_continuous(&mut self, flag: bool) {
        world_enable_continuous(&mut self.world, flag);
    }

    /// Serialize the full simulation state. (b2World_Snapshot)
    pub fn snapshot(&self) -> Vec<u8> {
        box2d_rust::recording::world_snapshot(&self.world)
    }

    /// Restore simulation state from a snapshot image. The demo body/joint
    /// tracking lists stay as-is: they only describe drawing, and restore is
    /// used on a world with the same scene. (b2World_Restore)
    pub fn restore(&mut self, image: &[u8]) -> bool {
        box2d_rust::recording::world_restore(&mut self.world, image)
    }

    /// FNV-1a hash over all body transforms and velocities, as a hex string
    /// for display. (b2HashWorldState)
    pub fn state_hash(&self) -> String {
        format!(
            "{:016X}",
            box2d_rust::recording::hash_world_state(&self.world)
        )
    }

    /// Dynamic capsule (horizontal, half length `hl`), rotated by `angle`.
    /// Returns the demo body index.
    pub fn add_capsule(
        &mut self,
        x: f32,
        y: f32,
        hl: f32,
        radius: f32,
        density: f32,
        angle: f32,
    ) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = m::to_pos(m::Vec2 { x, y });
        body_def.rotation = m::make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        let capsule = Capsule {
            center1: m::Vec2 { x: -hl, y: 0.0 },
            center2: m::Vec2 { x: hl, y: 0.0 },
            radius,
        };
        box2d_rust::shape::create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Static chain shape from interleaved world points [x, y]*. Chains are
    /// one-sided: wind right-to-left for a solid-side-up floor.
    /// (b2CreateChain) Returns the demo body index of the owning body.
    pub fn add_chain(&mut self, points: &[f32], is_loop: bool) -> usize {
        let body_def = default_body_def();
        let body_id = create_body(&mut self.world, &body_def);

        let mut chain_def = box2d_rust::types::default_chain_def();
        chain_def.is_loop = is_loop;
        chain_def.points = points
            .chunks_exact(2)
            .map(|p| m::Vec2 { x: p[0], y: p[1] })
            .collect();
        box2d_rust::shape::create_chain(&mut self.world, body_id, &chain_def);

        self.bodies.push(get_body_full_id(&self.world, body_id));
        self.bodies.len() - 1
    }

    /// Radial explosion. (b2World_Explode)
    pub fn explode(&mut self, x: f32, y: f32, radius: f32, falloff: f32, impulse_per_length: f32) {
        let mut def = box2d_rust::types::default_explosion_def();
        def.position = m::to_pos(m::Vec2 { x, y });
        def.radius = radius;
        def.falloff = falloff;
        def.impulse_per_length = impulse_per_length;
        box2d_rust::world::world_explode(&mut self.world, &def);
    }

    /// Change gravity at runtime. (b2World_SetGravity)
    pub fn set_gravity(&mut self, x: f32, y: f32) {
        box2d_rust::world::world_set_gravity(&mut self.world, m::Vec2 { x, y });
    }

    /// Place the capsule character mover.
    pub fn mover_spawn(&mut self, x: f32, y: f32) {
        self.mover_position = m::to_pos(m::Vec2 { x, y });
        self.mover_velocity = m::VEC2_ZERO;
    }

    /// One character-controller step built entirely from the ported mover
    /// queries: gather planes (b2World_CollideMover), resolve them
    /// (b2SolvePlanes), sweep the move (b2World_CastMover), and clip the
    /// velocity (b2ClipVector). Returns [x, y, grounded, planeCount].
    pub fn mover_update(&mut self, dt: f32, move_x: f32, jump: bool) -> Vec<f32> {
        use box2d_rust::mover::{clip_vector, solve_planes, CollisionPlane};

        let capsule = mover_capsule();
        let filter = box2d_rust::types::default_query_filter();

        // Gather contact planes at the current position.
        let mut planes: Vec<CollisionPlane> = Vec::new();
        world_collide_mover(
            &mut self.world,
            self.mover_position,
            &capsule,
            filter,
            |_, hit| {
                if planes.len() < 8 {
                    planes.push(CollisionPlane {
                        plane: hit.plane,
                        push_limit: f32::MAX,
                        push: 0.0,
                        clip_velocity: true,
                    });
                }
                true
            },
        );
        let grounded = planes.iter().any(|p| p.plane.normal.y > 0.7);

        // Input: horizontal approach, jump only from the ground, gravity.
        let v = &mut self.mover_velocity;
        v.x += (6.0 * move_x - v.x) * (10.0 * dt).min(1.0);
        if jump && grounded {
            v.y = 8.0;
        }
        v.y -= 10.0 * dt;

        // Resolve the desired motion against the planes, then sweep it so a
        // fast fall cannot skip a thin ledge.
        let target = m::mul_sv(dt, *v);
        let result = solve_planes(target, &mut planes);
        let fraction = world_cast_mover(
            &mut self.world,
            self.mover_position,
            &capsule,
            result.translation,
            filter,
        );
        self.mover_position =
            m::offset_pos(self.mover_position, m::mul_sv(fraction, result.translation));
        self.mover_velocity = clip_vector(self.mover_velocity, &planes);

        vec![
            self.mover_position.x as f32,
            self.mover_position.y as f32,
            if grounded { 1.0 } else { 0.0 },
            planes.len() as f32,
        ]
    }
}

/// The character capsule, in mover-local space.
fn mover_capsule() -> Capsule {
    Capsule {
        center1: m::Vec2 { x: 0.0, y: -0.25 },
        center2: m::Vec2 { x: 0.0, y: 0.25 },
        radius: 0.3,
    }
}

// ---------------------------------------------------------------------------
// Replay demo: record a SimWorld session, then play it back with the ported
// b2RecPlayer (keyframe ring, timeline scrub, divergence checking).
// ---------------------------------------------------------------------------

#[wasm_bindgen]
impl SimWorld {
    /// Start recording this world into an op-stream buffer seeded with a
    /// world snapshot. Returns false if a session is already active.
    pub fn start_recording(&mut self) -> bool {
        box2d_rust::recording::world_start_recording(
            &mut self.world,
            box2d_rust::recording::Recording::new(0),
        )
        .is_none()
    }

    /// Stop recording and return the finished recording bytes (empty if no
    /// session was active).
    pub fn stop_recording(&mut self) -> Vec<u8> {
        box2d_rust::recording::world_stop_recording(&mut self.world)
            .map(|rec| rec.buffer)
            .unwrap_or_default()
    }
}

/// Incremental playback of a recorded session via the ported b2RecPlayer.
#[wasm_bindgen]
pub struct SimPlayer {
    player: box2d_rust::recording::RecPlayer,
}

#[wasm_bindgen]
impl SimPlayer {
    /// Open a recording. Returns undefined if the bytes are malformed.
    pub fn open(data: &[u8]) -> Option<SimPlayer> {
        box2d_rust::recording::RecPlayer::create(data).map(|player| SimPlayer { player })
    }

    /// Advance one recorded step. False once the end is reached.
    pub fn step_frame(&mut self) -> bool {
        self.player.step_frame()
    }

    /// Seek to a recorded step; backward seeks restore the nearest keyframe
    /// and re-step only the gap.
    pub fn seek_frame(&mut self, frame: i32) {
        self.player.seek_frame(frame);
    }

    pub fn frame(&self) -> i32 {
        self.player.frame()
    }

    pub fn frame_count(&self) -> i32 {
        self.player.info().frame_count
    }

    pub fn has_diverged(&self) -> bool {
        self.player.has_diverged()
    }

    /// Current keyframe spacing in frames (the backward-seek granularity).
    pub fn keyframe_interval(&self) -> i32 {
        self.player.keyframe_interval()
    }

    /// Memory held by keyframe snapshots, in kilobytes.
    pub fn keyframe_kilobytes(&self) -> f32 {
        self.player.keyframe_bytes() as f32 / 1024.0
    }

    /// Positions of the replayed bodies in creation (outliner) order:
    /// [x, y, angle] per body. Matches the recording SimWorld's positions()
    /// order because replay reproduces ids deterministically.
    pub fn positions(&self) -> Vec<f32> {
        let world = self.player.world();
        let count = self.player.body_count();
        let mut out = Vec::with_capacity(3 * count as usize);
        for ord in 0..count {
            let id = self.player.body_id(ord);
            if id.is_null() {
                // Destroyed slot: park it far offscreen, ordinals stay stable
                out.push(f32::NAN);
                out.push(f32::NAN);
                out.push(0.0);
                continue;
            }
            let transform = get_body_transform(world, get_body_full_id(world, id));
            out.push(transform.p.x as f32);
            out.push(transform.p.y as f32);
            out.push(m::rot_get_angle(transform.q));
        }
        out
    }

    pub fn awake_body_count(&self) -> i32 {
        self.player.world().solver_sets[box2d_rust::solver_set::AWAKE_SET as usize]
            .body_sims
            .len() as i32
    }

    pub fn contact_count(&self) -> i32 {
        self.player.world().contact_id_pool.id_count()
    }
}
