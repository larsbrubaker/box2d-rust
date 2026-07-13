//! Free wasm bindings for Collision samples (`sample_collision.cpp`).
//! Geometry / GJK / manifold / TOI queries — no world required.

use wasm_bindgen::prelude::*;

use box2d_rust::collision::{Capsule, ChainSegment, Circle, RayCastInput, Segment};
use box2d_rust::distance::{
    get_sweep_transform, make_proxy, shape_cast, shape_distance, time_of_impact, DistanceInput,
    ShapeCastPairInput, ShapeProxy, SimplexCache, Sweep, ToiInput, ToiState,
};
use box2d_rust::geometry::{
    make_box, make_offset_box, make_polygon, make_square, ray_cast_capsule, ray_cast_circle,
    ray_cast_polygon, ray_cast_segment,
};
use box2d_rust::hull::compute_hull;
use box2d_rust::manifold::{
    collide_capsule_and_circle, collide_capsules, collide_chain_segment_and_circle,
    collide_chain_segment_and_polygon, collide_circles, collide_polygon_and_capsule,
    collide_polygon_and_circle, collide_polygons, collide_segment_and_capsule,
    collide_segment_and_circle, collide_segment_and_polygon,
};
use box2d_rust::math_functions::{
    inv_mul_transforms, inv_mul_world_transforms, inv_rotate_vector, inv_transform_world_point,
    make_rot, rotate_vector, transform_world_point, Pos, Rot, Transform, Vec2, WorldTransform, PI,
    ROT_IDENTITY, WORLD_TRANSFORM_IDENTITY,
};

fn pack_manifold(m: &box2d_rust::collision::LocalManifold) -> Vec<f32> {
    let mut out = vec![m.normal.x, m.normal.y, m.point_count as f32];
    for i in 0..2 {
        out.push(m.points[i].point.x);
        out.push(m.points[i].point.y);
        out.push(m.points[i].separation);
        out.push(m.points[i].id as f32);
    }
    out
}

fn proxy_from_flat(pts: &[f32], radius: f32) -> ShapeProxy {
    let n = pts.len() / 2;
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        points.push(Vec2 {
            x: pts[i * 2],
            y: pts[i * 2 + 1],
        });
    }
    make_proxy(&points, radius)
}

fn xf(px: f32, py: f32, angle: f32) -> Transform {
    Transform {
        p: Vec2 { x: px, y: py },
        q: make_rot(angle),
    }
}

fn wxf(px: f32, py: f32, angle: f32) -> WorldTransform {
    WorldTransform {
        p: Pos { x: px, y: py },
        q: make_rot(angle),
    }
}

/// Shape Distance (`sample_collision.cpp` ShapeDistance / `b2ShapeDistance`).
/// Returns `[pax, pay, pbx, pby, distance, iterations, nx, ny]`.
#[wasm_bindgen]
pub fn collision_shape_distance(
    pts_a: &[f32],
    radius_a: f32,
    pts_b: &[f32],
    radius_b: f32,
    tx: f32,
    ty: f32,
    angle: f32,
    use_radii: bool,
) -> Vec<f32> {
    let input = DistanceInput {
        proxy_a: proxy_from_flat(pts_a, radius_a),
        proxy_b: proxy_from_flat(pts_b, radius_b),
        transform: xf(tx, ty, angle),
        use_radii,
    };
    let mut cache = SimplexCache::default();
    let out = shape_distance(&input, &mut cache, None);
    vec![
        out.point_a.x,
        out.point_a.y,
        out.point_b.x,
        out.point_b.y,
        out.distance,
        out.iterations as f32,
        out.normal.x,
        out.normal.y,
    ]
}

/// Shape Cast (`b2ShapeCast`). Returns `[hit, fraction, px, py, nx, ny, iterations]`.
#[wasm_bindgen]
pub fn collision_shape_cast(
    pts_a: &[f32],
    radius_a: f32,
    pts_b: &[f32],
    radius_b: f32,
    tx: f32,
    ty: f32,
    angle: f32,
    tdx: f32,
    tdy: f32,
    max_fraction: f32,
    can_encroach: bool,
) -> Vec<f32> {
    let input = ShapeCastPairInput {
        proxy_a: proxy_from_flat(pts_a, radius_a),
        proxy_b: proxy_from_flat(pts_b, radius_b),
        transform: xf(tx, ty, angle),
        translation_b: Vec2 { x: tdx, y: tdy },
        max_fraction,
        can_encroach,
    };
    let out = shape_cast(&input);
    vec![
        if out.hit { 1.0 } else { 0.0 },
        out.fraction,
        out.point.x,
        out.point.y,
        out.normal.x,
        out.normal.y,
        out.iterations as f32,
    ]
}

/// Time of Impact sample — hardcoded sweeps from `sample_collision.cpp:3595-3656`.
/// Returns `[state, fraction, dist, …transforms]` (state 0..4; Hit=3).
#[wasm_bindgen]
pub fn collision_time_of_impact() -> Vec<f32> {
    let vertices_a = [
        Vec2 {
            x: -16.25,
            y: 44.75,
        },
        Vec2 {
            x: -15.75,
            y: 44.75,
        },
        Vec2 {
            x: -15.75,
            y: 45.25,
        },
        Vec2 {
            x: -16.25,
            y: 45.25,
        },
    ];
    let vertices_b = [Vec2 { x: 0.0, y: -0.125 }, Vec2 { x: 0.0, y: 0.125 }];

    let sweep_a = Sweep {
        local_center: Vec2 { x: 0.0, y: 0.0 },
        c1: Vec2 { x: 0.0, y: 0.0 },
        c2: Vec2 { x: 0.0, y: 0.0 },
        q1: ROT_IDENTITY,
        q2: ROT_IDENTITY,
    };
    let sweep_b = Sweep {
        local_center: Vec2 { x: 0.0, y: 0.0 },
        c1: Vec2 {
            x: -15.8332710,
            y: 45.3520279,
        },
        c2: Vec2 {
            x: -15.8324337,
            y: 45.3413048,
        },
        q1: Rot {
            c: -0.540891349,
            s: 0.841092527,
        },
        q2: Rot {
            c: -0.457797021,
            s: 0.889056742,
        },
    };

    let input = ToiInput {
        proxy_a: make_proxy(&vertices_a, 0.0),
        proxy_b: make_proxy(&vertices_b, 0.0299999993),
        sweep_a,
        sweep_b,
        max_fraction: 1.0,
    };
    let output = time_of_impact(&input);
    let state = match output.state {
        ToiState::Unknown => 0.0,
        ToiState::Failed => 1.0,
        ToiState::Overlapped => 2.0,
        ToiState::Hit => 3.0,
        ToiState::Separated => 4.0,
    };

    let mut dist = 0.0;
    if matches!(output.state, ToiState::Hit) {
        let ta = get_sweep_transform(&sweep_a, output.fraction);
        let tb = get_sweep_transform(&sweep_b, output.fraction);
        let d_in = DistanceInput {
            proxy_a: input.proxy_a,
            proxy_b: input.proxy_b,
            transform: inv_mul_transforms(ta, tb),
            use_radii: false,
        };
        let mut cache = SimplexCache::default();
        dist = shape_distance(&d_in, &mut cache, None).distance;
    }

    let t0a = get_sweep_transform(&sweep_a, 0.0);
    let t0b = get_sweep_transform(&sweep_b, 0.0);
    let thb = get_sweep_transform(&sweep_b, output.fraction);
    let t1b = get_sweep_transform(&sweep_b, 1.0);

    vec![
        state,
        output.fraction,
        dist,
        t0a.q.c,
        t0a.q.s,
        t0a.p.x,
        t0a.p.y,
        t0b.q.c,
        t0b.q.s,
        t0b.p.x,
        t0b.p.y,
        thb.q.c,
        thb.q.s,
        thb.p.x,
        thb.p.y,
        t1b.q.c,
        t1b.q.s,
        t1b.p.x,
        t1b.p.y,
    ]
}

/// Ray Cast sample — 5 shapes at C offsets. Returns 5×`[hit,frac,px,py,nx,ny]`.
#[wasm_bindgen]
pub fn collision_ray_cast_shapes(
    ox: f32,
    oy: f32,
    angle: f32,
    rsx: f32,
    rsy: f32,
    rex: f32,
    rey: f32,
) -> Vec<f32> {
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 2.0,
    };
    let capsule = Capsule {
        center1: Vec2 { x: -1.0, y: 1.0 },
        center2: Vec2 { x: 1.0, y: -1.0 },
        radius: 1.5,
    };
    let box_ = make_box(2.0, 2.0);
    let tri_pts = [
        Vec2 { x: -2.0, y: 0.0 },
        Vec2 { x: 2.0, y: 0.0 },
        Vec2 { x: 2.0, y: 3.0 },
    ];
    let triangle = make_polygon(&compute_hull(&tri_pts), 0.0);
    let segment = Segment {
        point1: Vec2 { x: -3.0, y: 0.0 },
        point2: Vec2 { x: 3.0, y: 0.0 },
    };

    let offsets = [
        Vec2 { x: -20.0, y: 20.0 },
        Vec2 { x: -10.0, y: 20.0 },
        Vec2 { x: 0.0, y: 20.0 },
        Vec2 { x: 10.0, y: 20.0 },
        Vec2 { x: 20.0, y: 20.0 },
    ];

    let mut out = Vec::with_capacity(30);
    for (i, off) in offsets.iter().enumerate() {
        let transform = wxf(ox + off.x, oy + off.y, angle);
        let start = inv_transform_world_point(transform, Pos { x: rsx, y: rsy });
        let translation = inv_rotate_vector(
            transform.q,
            Vec2 {
                x: rex - rsx,
                y: rey - rsy,
            },
        );
        let input = RayCastInput {
            origin: Vec2 {
                x: start.x as f32,
                y: start.y as f32,
            },
            translation,
            max_fraction: 1.0,
        };
        let local = match i {
            0 => ray_cast_circle(&circle, &input),
            1 => ray_cast_capsule(&capsule, &input),
            2 => ray_cast_polygon(&box_, &input),
            3 => ray_cast_polygon(&triangle, &input),
            _ => ray_cast_segment(&segment, &input, false),
        };
        let (px, py, nx, ny) = if local.hit {
            let p = transform_world_point(
                transform,
                Pos {
                    x: local.point.x,
                    y: local.point.y,
                },
            );
            let n = rotate_vector(transform.q, local.normal);
            (p.x as f32, p.y as f32, n.x, n.y)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };
        out.push(if local.hit { 1.0 } else { 0.0 });
        out.push(local.fraction);
        out.push(px);
        out.push(py);
        out.push(nx);
        out.push(ny);
    }
    out
}

/// Manifold sample pair by kind. Returns packed manifold.
#[wasm_bindgen]
pub fn collision_manifold_pair(kind: u32, bx: f32, by: f32, angle: f32, round: f32) -> Vec<f32> {
    let relative = xf(bx, by, angle);
    let m = match kind {
        0 => {
            let c1 = Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            };
            let c2 = Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 1.0,
            };
            collide_circles(&c1, &c2, relative)
        }
        1 => {
            let cap = Capsule {
                center1: Vec2 { x: -0.5, y: 0.0 },
                center2: Vec2 { x: 0.5, y: 0.0 },
                radius: 0.25,
            };
            let c = Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            };
            collide_capsule_and_circle(&cap, &c, relative)
        }
        2 => {
            let seg = Segment {
                point1: Vec2 { x: -1.0, y: 0.0 },
                point2: Vec2 { x: 1.0, y: 0.0 },
            };
            let c = Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            };
            collide_segment_and_circle(&seg, &c, relative)
        }
        3 => {
            let mut box_ = make_square(0.5);
            box_.radius = round;
            let c = Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            };
            collide_polygon_and_circle(&box_, &c, relative)
        }
        4 => {
            let a = Capsule {
                center1: Vec2 { x: -0.5, y: 0.0 },
                center2: Vec2 { x: 0.5, y: 0.0 },
                radius: 0.25,
            };
            let b = Capsule {
                center1: Vec2 { x: 0.25, y: 0.0 },
                center2: Vec2 { x: 1.0, y: 0.0 },
                radius: 0.1,
            };
            collide_capsules(&a, &b, relative)
        }
        5 => {
            let box_ = make_offset_box(0.25, 1.0, Vec2 { x: 1.0, y: -1.0 }, make_rot(0.25 * PI));
            let cap = Capsule {
                center1: Vec2 { x: -0.4, y: 0.0 },
                center2: Vec2 { x: -0.1, y: 0.0 },
                radius: 0.1,
            };
            collide_polygon_and_capsule(&box_, &cap, relative)
        }
        6 => {
            let seg = Segment {
                point1: Vec2 { x: -1.0, y: 0.0 },
                point2: Vec2 { x: 1.0, y: 0.0 },
            };
            let cap = Capsule {
                center1: Vec2 { x: -0.5, y: 0.0 },
                center2: Vec2 { x: 0.5, y: 0.0 },
                radius: 0.25,
            };
            collide_segment_and_capsule(&seg, &cap, relative)
        }
        7 => {
            let mut a = make_box(0.5, 0.5);
            a.radius = round;
            let mut b = make_box(0.5, 0.5);
            b.radius = round;
            collide_polygons(&a, &b, relative)
        }
        8 => {
            let seg = Segment {
                point1: Vec2 { x: -1.5, y: 0.0 },
                point2: Vec2 { x: 1.5, y: 0.0 },
            };
            let mut box_ = make_box(0.5, 0.5);
            box_.radius = round;
            collide_segment_and_polygon(&seg, &box_, relative)
        }
        9 => {
            let pts = [
                Vec2 { x: -0.1, y: -0.5 },
                Vec2 { x: 0.1, y: -0.5 },
                Vec2 { x: 0.0, y: 0.5 },
            ];
            let mut wedge = make_polygon(&compute_hull(&pts), 0.0);
            wedge.radius = round;
            let mut box_ = make_box(0.5, 0.5);
            box_.radius = round;
            collide_polygons(&wedge, &box_, relative)
        }
        _ => box2d_rust::collision::LocalManifold::default(),
    };
    pack_manifold(&m)
}

/// Smooth Manifold — collide circle/box vs one chain segment.
#[wasm_bindgen]
pub fn collision_smooth_manifold(
    shape_type: u32,
    bx: f32,
    by: f32,
    angle: f32,
    round: f32,
    g1x: f32,
    g1y: f32,
    p1x: f32,
    p1y: f32,
    p2x: f32,
    p2y: f32,
    g2x: f32,
    g2y: f32,
) -> Vec<f32> {
    let segment = ChainSegment {
        ghost1: Vec2 { x: g1x, y: g1y },
        segment: Segment {
            point1: Vec2 { x: p1x, y: p1y },
            point2: Vec2 { x: p2x, y: p2y },
        },
        ghost2: Vec2 { x: g2x, y: g2y },
        chain_id: -1,
    };
    let relative = inv_mul_world_transforms(WORLD_TRANSFORM_IDENTITY, wxf(bx, by, angle));
    let m = if shape_type == 0 {
        let c = Circle {
            center: Vec2 { x: 0.0, y: 0.0 },
            radius: 0.5 + round,
        };
        collide_chain_segment_and_circle(&segment, &c, relative)
    } else {
        let mut box_ = make_box(0.5, 0.5);
        box_.radius = round;
        let mut cache = SimplexCache::default();
        collide_chain_segment_and_polygon(&segment, &box_, relative, &mut cache)
    };
    pack_manifold(&m)
}
