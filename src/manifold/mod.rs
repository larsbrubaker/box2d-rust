// Port of box2d-cpp-reference/src/manifold.c — contact manifold generation.
//
// Split to satisfy the 800-line file limit:
// - circles.rs — circle-vs-{circle, capsule, polygon, segment}
// - capsules.rs — capsule-vs-capsule and its segment/polygon wrappers
// - polygons.rs — SAT polygon clipper and polygon-vs-polygon
// - chain.rs   — one-sided chain segment collisions (Gauss map ghost logic)
//
// The manifold types (LocalManifold and friends) live in collision.rs.
// The dead `#else` branch inside b2CollidePolygons (`#if 1`) is not ported.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

mod capsules;
mod chain;
mod circles;
mod polygons;

pub use capsules::{collide_capsules, collide_polygon_and_capsule, collide_segment_and_capsule};
pub use chain::{
    collide_chain_segment_and_capsule, collide_chain_segment_and_circle,
    collide_chain_segment_and_polygon,
};
pub use circles::{
    collide_capsule_and_circle, collide_circles, collide_polygon_and_circle,
    collide_segment_and_circle,
};
pub use polygons::{collide_polygons, collide_segment_and_polygon};

use crate::collision::Polygon;
use crate::math_functions::{length_squared, lerp, neg, normalize, right_perp, sub, Vec2};

/// C: B2_MAKE_ID(A, B) — pack two feature indices into a contact id.
pub(crate) fn make_id(a: i32, b: i32) -> u16 {
    ((a as u8 as u16) << 8) | (b as u8 as u16)
}

/// Build a 2-vertex polygon representing a capsule. (static b2MakeCapsule)
pub(crate) fn make_capsule_polygon(p1: Vec2, p2: Vec2, radius: f32) -> Polygon {
    let mut shape = Polygon {
        centroid: lerp(p1, p2, 0.5),
        count: 2,
        radius,
        ..Default::default()
    };
    shape.vertices[0] = p1;
    shape.vertices[1] = p2;

    let d = sub(p2, p1);
    debug_assert!(length_squared(d) > f32::EPSILON);
    let axis = normalize(d);
    let normal = right_perp(axis);

    shape.normals[0] = normal;
    shape.normals[1] = neg(normal);

    shape
}
