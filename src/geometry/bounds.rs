// Shape AABB computation from geometry.c.
//
// AABBs are built in double and narrowed to float once. In large world mode
// the narrowing rounds outward so the box always contains the shape, and the
// inflation (speculative margin) folds into the double step, otherwise it
// vanishes into a float ULP far from the origin. In float mode the rounding
// helpers are plain casts.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// In single precision the Pos coordinates are f32, so the widening `as f64`
// promotions below mirror the C double math in both modes.
#![allow(clippy::unnecessary_cast)]

use crate::collision::{Capsule, Circle, Polygon, Segment, ShapeGeometry};
use crate::math_functions::{
    round_down_float, round_up_float, transform_world_point, Aabb, Vec2, WorldTransform,
};

fn compute_circle_fat_aabb(shape: &Circle, xf: WorldTransform, extra: f32) -> Aabb {
    let c = transform_world_point(xf, shape.center);
    let r = shape.radius as f64 + extra as f64;
    Aabb {
        lower_bound: Vec2 {
            x: round_down_float(c.x as f64 - r),
            y: round_down_float(c.y as f64 - r),
        },
        upper_bound: Vec2 {
            x: round_up_float(c.x as f64 + r),
            y: round_up_float(c.y as f64 + r),
        },
    }
}

fn compute_capsule_fat_aabb(shape: &Capsule, xf: WorldTransform, extra: f32) -> Aabb {
    let v1 = transform_world_point(xf, shape.center1);
    let v2 = transform_world_point(xf, shape.center2);
    let r = shape.radius as f64 + extra as f64;
    let (v1x, v1y) = (v1.x as f64, v1.y as f64);
    let (v2x, v2y) = (v2.x as f64, v2.y as f64);
    Aabb {
        lower_bound: Vec2 {
            x: round_down_float(if v1x < v2x { v1x } else { v2x } - r),
            y: round_down_float(if v1y < v2y { v1y } else { v2y } - r),
        },
        upper_bound: Vec2 {
            x: round_up_float(if v1x > v2x { v1x } else { v2x } + r),
            y: round_up_float(if v1y > v2y { v1y } else { v2y } + r),
        },
    }
}

fn compute_polygon_fat_aabb(shape: &Polygon, xf: WorldTransform, extra: f32) -> Aabb {
    debug_assert!(shape.count > 0);
    let v = transform_world_point(xf, shape.vertices[0]);
    let (mut lx, mut ly, mut ux, mut uy) = (v.x as f64, v.y as f64, v.x as f64, v.y as f64);

    for i in 1..shape.count as usize {
        let v = transform_world_point(xf, shape.vertices[i]);
        let (vx, vy) = (v.x as f64, v.y as f64);
        lx = if vx < lx { vx } else { lx };
        ly = if vy < ly { vy } else { ly };
        ux = if vx > ux { vx } else { ux };
        uy = if vy > uy { vy } else { uy };
    }

    let r = shape.radius as f64 + extra as f64;
    Aabb {
        lower_bound: Vec2 {
            x: round_down_float(lx - r),
            y: round_down_float(ly - r),
        },
        upper_bound: Vec2 {
            x: round_up_float(ux + r),
            y: round_up_float(uy + r),
        },
    }
}

fn compute_segment_fat_aabb(shape: &Segment, xf: WorldTransform, extra: f32) -> Aabb {
    let v1 = transform_world_point(xf, shape.point1);
    let v2 = transform_world_point(xf, shape.point2);
    let e = extra as f64;
    let (v1x, v1y) = (v1.x as f64, v1.y as f64);
    let (v2x, v2y) = (v2.x as f64, v2.y as f64);
    Aabb {
        lower_bound: Vec2 {
            x: round_down_float(if v1x < v2x { v1x } else { v2x } - e),
            y: round_down_float(if v1y < v2y { v1y } else { v2y } - e),
        },
        upper_bound: Vec2 {
            x: round_up_float(if v1x > v2x { v1x } else { v2x } + e),
            y: round_up_float(if v1y > v2y { v1y } else { v2y } + e),
        },
    }
}

/// Compute the bounding box of a transformed circle. (b2ComputeCircleAABB)
pub fn compute_circle_aabb(shape: &Circle, xf: WorldTransform) -> Aabb {
    compute_circle_fat_aabb(shape, xf, 0.0)
}

/// Compute the bounding box of a transformed capsule. (b2ComputeCapsuleAABB)
pub fn compute_capsule_aabb(shape: &Capsule, xf: WorldTransform) -> Aabb {
    compute_capsule_fat_aabb(shape, xf, 0.0)
}

/// Compute the bounding box of a transformed polygon. (b2ComputePolygonAABB)
pub fn compute_polygon_aabb(shape: &Polygon, xf: WorldTransform) -> Aabb {
    compute_polygon_fat_aabb(shape, xf, 0.0)
}

/// Compute the bounding box of a transformed line segment. (b2ComputeSegmentAABB)
pub fn compute_segment_aabb(shape: &Segment, xf: WorldTransform) -> Aabb {
    compute_segment_fat_aabb(shape, xf, 0.0)
}

/// Compute a fattened bounding box of a transformed shape. (b2ComputeFatShapeAABB)
pub fn compute_fat_shape_aabb(geometry: &ShapeGeometry, xf: WorldTransform, extra: f32) -> Aabb {
    match geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_fat_aabb(capsule, xf, extra),
        ShapeGeometry::Circle(circle) => compute_circle_fat_aabb(circle, xf, extra),
        ShapeGeometry::Polygon(polygon) => compute_polygon_fat_aabb(polygon, xf, extra),
        ShapeGeometry::Segment(segment) => compute_segment_fat_aabb(segment, xf, extra),
        ShapeGeometry::ChainSegment(chain_segment) => {
            compute_segment_fat_aabb(&chain_segment.segment, xf, extra)
        }
    }
}
