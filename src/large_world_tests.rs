// Port of test_large_world.c: large-world (double-precision) acceptance.
// A pyramid, a bullet, a ray cast, and the origin-taking queries must behave
// identically at the origin and at 1e7 m from it. In the single-precision
// build only the origin runs execute, same as C.
//
// LargeWorldRecordingTest is not ported: the recording/replay subsystem does
// not exist in the Rust port yet.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_sim_location, create_body, get_body_full_id};
use crate::collision::{Capsule, Circle};
use crate::distance::make_proxy;
use crate::geometry::make_box;
use crate::id::BodyId;
use crate::math_functions::{offset_pos, sub_pos, Pos, Vec2, POS_ZERO};
// The 1e7 base only exists in the double-precision branches.
#[cfg(feature = "double-precision")]
use crate::math_functions::to_pos;
use crate::shape::{
    create_circle_shape, create_polygon_shape, shape_ray_cast, shape_test_point,
};
use crate::types::{default_body_def, default_query_filter, default_shape_def, default_world_def};
use crate::types::BodyType;
use crate::world::{
    world_cast_mover, world_cast_ray_closest, world_cast_shape, world_collide_mover,
    world_get_awake_body_count, world_get_body_events, world_overlap_shape, world_step, World,
};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

/// Read a body center of mass relative to a base position. The public getters
/// demote to float (~1 m resolution at 1e7), so the precision check reaches
/// into the body sim, same as C. (static BodyRelativeCenter)
fn body_relative_center(world: &World, body_id: BodyId, base: Pos) -> Vec2 {
    let body_index = get_body_full_id(world, body_id);
    let (set_index, local_index) = body_sim_location(world, body_index);
    let center = world.solver_sets[set_index as usize].body_sims[local_index as usize].center;
    sub_pos(center, base)
}

const PYRAMID_BODY_COUNT: usize = 9;

struct PyramidResult {
    sleep_step: i32,
    // Only compared in the double-precision branch of the test.
    #[cfg_attr(not(feature = "double-precision"), allow(dead_code))]
    rel: [Vec2; PYRAMID_BODY_COUNT],
}

/// Build a stepped pyramid of unit boxes on a base position, settle it, and
/// report the sleep frame and the settled centers relative to the base.
/// Integer offsets keep every float bodyDef.position exact even at 1e7.
/// (static RunPyramid)
fn run_pyramid(base: Pos) -> PyramidResult {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    {
        let mut body_def = default_body_def();
        body_def.position = base;
        let ground_id = create_body(&mut world, &body_def);

        // Ground top surface at baseY + 0.5
        let ground_box = make_box(10.0, 0.5);
        let shape_def = default_shape_def();
        create_polygon_shape(&mut world, ground_id, &shape_def, &ground_box);
    }

    // Each box is 1 m, centers on integer offsets. Row 0 rests on the ground,
    // rows stack directly above so the configuration is stable and sleeps
    // quickly.
    const OFFSETS: [Vec2; PYRAMID_BODY_COUNT] = [
        Vec2 { x: -2.0, y: 1.0 },
        Vec2 { x: -1.0, y: 1.0 },
        Vec2 { x: 0.0, y: 1.0 },
        Vec2 { x: 1.0, y: 1.0 },
        Vec2 { x: 2.0, y: 1.0 },
        Vec2 { x: -1.0, y: 2.0 },
        Vec2 { x: 0.0, y: 2.0 },
        Vec2 { x: 1.0, y: 2.0 },
        Vec2 { x: 0.0, y: 3.0 },
    ];

    let box_poly = make_box(0.5, 0.5);
    let shape_def = default_shape_def();
    let mut body_ids: Vec<BodyId> = Vec::with_capacity(PYRAMID_BODY_COUNT);
    for offset in OFFSETS.iter() {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = offset_pos(base, *offset);
        let body_id = create_body(&mut world, &body_def);
        create_polygon_shape(&mut world, body_id, &shape_def, &box_poly);
        body_ids.push(body_id);
    }

    let mut sleep_step = -1;

    for step in 0..250 {
        world_step(&mut world, 1.0 / 60.0, 4);

        if sleep_step < 0
            && world_get_body_events(&world).is_empty()
            && world_get_awake_body_count(&world) == 0
        {
            sleep_step = step;
        }
    }

    let mut rel = [Vec2 { x: 0.0, y: 0.0 }; PYRAMID_BODY_COUNT];
    for (i, body_id) in body_ids.iter().enumerate() {
        rel[i] = body_relative_center(&world, *body_id, base);
    }

    PyramidResult { sleep_step, rel }
}

// Far from the origin the contact boundary is differenced in double, so a
// settling pyramid must follow the same relative trajectory as one at the
// origin and sleep on the same frame. (LargeWorldPyramidTest)
#[test]
fn large_world_pyramid() {
    let origin = run_pyramid(POS_ZERO);
    assert!(origin.sleep_step > 0);

    #[cfg(feature = "double-precision")]
    {
        let large = run_pyramid(to_pos(Vec2 { x: 1.0e7, y: 0.0 }));

        assert_eq!(large.sleep_step, origin.sleep_step);
        for i in 0..PYRAMID_BODY_COUNT {
            ensure_small(large.rel[i].x - origin.rel[i].x, 1e-3);
            ensure_small(large.rel[i].y - origin.rel[i].y, 1e-3);
        }
    }
}

/// Fire a bullet at a thin wall and report where it ends up relative to the
/// base. With continuous collision the bullet stops at the near face.
/// (static RunBullet)
fn run_bullet(base: Pos) -> f32 {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    // Thin tall wall centered on the base
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = base;
    let wall_id = create_body(&mut world, &body_def);
    let wall = make_box(0.05, 5.0);
    let shape_def = default_shape_def();
    create_polygon_shape(&mut world, wall_id, &shape_def, &wall);

    // Bullet fired at the wall from the far side. At 200 m/s it crosses
    // ~3.3 m per step, so without continuous collision it passes the 0.1 m
    // wall in the first step.
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.is_bullet = true;
    body_def.gravity_scale = 0.0;
    body_def.position = offset_pos(base, Vec2 { x: 10.0, y: 0.0 });
    body_def.linear_velocity = Vec2 { x: -200.0, y: 0.0 };
    let bullet_id = create_body(&mut world, &body_def);
    let circle = Circle {
        center: Vec2 { x: 0.0, y: 0.0 },
        radius: 0.1,
    };
    let shape_def = default_shape_def();
    create_circle_shape(&mut world, bullet_id, &shape_def, &circle);

    for _ in 0..30 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    body_relative_center(&world, bullet_id, base).x
}

// The swept query box is rounded back to world float, which at 1e7 has ~1 m
// resolution, the most likely place to drop the hit. The re-centered TOI must
// still catch the wall with no tunneling. (LargeWorldBulletTest)
#[test]
fn large_world_bullet() {
    // At the origin the bullet must stop at the near face in both precision
    // modes. Wall face at 0.05 plus the 0.1 bullet radius puts the rest
    // position near 0.15.
    let rel_x = run_bullet(POS_ZERO);
    assert!(rel_x > 0.0 && rel_x < 0.5);

    #[cfg(feature = "double-precision")]
    {
        let rel_x = run_bullet(to_pos(Vec2 { x: 1.0e7, y: 0.0 }));
        assert!(rel_x > 0.0 && rel_x < 0.5);
    }
}

/// Cast a ray at a unit box on the base and report the hit point relative to
/// the base. (static RunRayCast)
fn run_ray_cast(base: Pos) -> Vec2 {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.position = base;
    let body_id = create_body(&mut world, &body_def);
    let box_poly = make_box(0.5, 0.5);
    let shape_def = default_shape_def();
    create_polygon_shape(&mut world, body_id, &shape_def, &box_poly);

    // Ray from 5 m left of the box, traveling 10 m right. Hits the left face
    // at base + {-0.5, 0}.
    let origin = offset_pos(base, Vec2 { x: -5.0, y: 0.0 });
    let translation = Vec2 { x: 10.0, y: 0.0 };
    let result = world_cast_ray_closest(&world, origin, translation, default_query_filter());

    // A miss leaves the point at the origin, which the caller's position
    // check rejects
    sub_pos(result.point, base)
}

// A float ray cast at 1e7 would resolve the hit only to the ~1 m coordinate
// ULP. The double origin plus per-shape re-centering keeps the analytic hit
// point accurate far from the origin. (LargeWorldRayCastTest)
#[test]
fn large_world_ray_cast() {
    let rel = run_ray_cast(POS_ZERO);
    ensure_small(rel.x + 0.5, 1e-4);
    ensure_small(rel.y, 1e-4);

    #[cfg(feature = "double-precision")]
    {
        let rel = run_ray_cast(to_pos(Vec2 { x: 1.0e7, y: 0.0 }));
        ensure_small(rel.x + 0.5, 1e-4);
        ensure_small(rel.y, 1e-4);
    }
}

#[derive(Debug, Clone, Copy)]
struct OriginQueryData {
    overlap_count: i32,
    cast_point: Pos,
    cast_fraction: f32,
    mover_fraction: f32,
    plane_count: i32,
    inside_point: bool,
    shape_ray_point: Pos,
    shape_ray_hit: bool,
}

/// Issue every origin-taking query against a unit box on the base, with all
/// geometry relative to the base. (static RunOriginQueries)
fn run_origin_queries(base: Pos) -> OriginQueryData {
    let world_def = default_world_def();
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.position = base;
    let body_id = create_body(&mut world, &body_def);
    let box_poly = make_box(0.5, 0.5);
    let shape_def = default_shape_def();
    let shape_id = create_polygon_shape(&mut world, body_id, &shape_def, &box_poly);

    let filter = default_query_filter();
    let mut data = OriginQueryData {
        overlap_count: 0,
        cast_point: POS_ZERO,
        cast_fraction: 1.0,
        mover_fraction: 1.0,
        plane_count: 0,
        inside_point: false,
        shape_ray_point: POS_ZERO,
        shape_ray_hit: false,
    };

    // Overlap a small circle centered on the box
    let center = Vec2 { x: 0.0, y: 0.0 };
    let overlap_proxy = make_proxy(&[center], 0.1);
    world_overlap_shape(&world, base, &overlap_proxy, filter, |_| {
        data.overlap_count += 1;
        true
    });

    // Cast a small circle at the left face. Center stops at -0.6, hit point
    // on the face at -0.5.
    let start = Vec2 { x: -5.0, y: 0.0 };
    let cast_proxy = make_proxy(&[start], 0.1);
    world_cast_shape(
        &world,
        base,
        &cast_proxy,
        Vec2 { x: 10.0, y: 0.0 },
        filter,
        |_, point, _, fraction| {
            data.cast_point = point;
            data.cast_fraction = fraction;
            fraction
        },
    );

    // Mover cast at the box
    let mover = Capsule {
        center1: Vec2 { x: -5.0, y: -0.2 },
        center2: Vec2 { x: -5.0, y: 0.2 },
        radius: 0.3,
    };
    data.mover_fraction = world_cast_mover(&world, base, &mover, Vec2 { x: 10.0, y: 0.0 }, filter);

    // Mover overlapping the box gathers planes
    let touching = Capsule {
        center1: Vec2 { x: -0.9, y: -0.2 },
        center2: Vec2 { x: -0.9, y: 0.2 },
        radius: 0.5,
    };
    world_collide_mover(&world, base, &touching, filter, |_, _| {
        data.plane_count += 1;
        true
    });

    // Shape level queries at the base
    data.inside_point = shape_test_point(&world, shape_id, base);

    let ray_output = shape_ray_cast(
        &world,
        shape_id,
        offset_pos(base, Vec2 { x: -5.0, y: 0.0 }),
        Vec2 { x: 10.0, y: 0.0 },
    );
    data.shape_ray_hit = ray_output.hit;
    data.shape_ray_point = ray_output.point;

    data
}

// The results must match an origin-zero run, which is what makes the origin
// plumbing (tree lift, per-shape re-centering, output compose) non-vacuous
// far from the origin. (LargeWorldOriginQueryTest)
#[test]
fn large_world_origin_query() {
    let origin = run_origin_queries(POS_ZERO);
    assert_eq!(origin.overlap_count, 1);
    assert!(origin.cast_fraction < 1.0);
    assert!(origin.mover_fraction < 1.0);
    assert!(origin.plane_count >= 1);
    assert!(origin.inside_point);
    assert!(origin.shape_ray_hit);

    let cast_rel = sub_pos(origin.cast_point, POS_ZERO);
    ensure_small(cast_rel.x + 0.5, 1e-3);
    let ray_rel = sub_pos(origin.shape_ray_point, POS_ZERO);
    ensure_small(ray_rel.x + 0.5, 1e-3);

    #[cfg(feature = "double-precision")]
    {
        // The same relative queries far from the origin must reproduce the
        // origin run. A float query at 1e7 could not resolve the faces below
        // the coordinate ULP.
        let base = to_pos(Vec2 { x: 1.0e7, y: 0.0 });
        let large = run_origin_queries(base);
        assert_eq!(large.overlap_count, origin.overlap_count);
        assert_eq!(large.plane_count, origin.plane_count);
        assert_eq!(large.inside_point, origin.inside_point);
        assert_eq!(large.shape_ray_hit, origin.shape_ray_hit);
        ensure_small(large.cast_fraction - origin.cast_fraction, 1e-4);
        ensure_small(large.mover_fraction - origin.mover_fraction, 1e-4);

        let cast_rel_large = sub_pos(large.cast_point, base);
        ensure_small(cast_rel_large.x - cast_rel.x, 1e-3);
        ensure_small(cast_rel_large.y - cast_rel.y, 1e-3);

        let ray_rel_large = sub_pos(large.shape_ray_point, base);
        ensure_small(ray_rel_large.x - ray_rel.x, 1e-3);
        ensure_small(ray_rel_large.y - ray_rel.y, 1e-3);
    }
}
