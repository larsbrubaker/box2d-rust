// Shape cast wrappers and mover collision from geometry.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{
    Capsule, CastOutput, Circle, PlaneResult, Polygon, Segment, ShapeCastInput,
};
use crate::distance::{
    make_proxy, shape_cast, shape_distance, DistanceInput, ShapeCastPairInput, SimplexCache,
};
use crate::math_functions::{Plane, TRANSFORM_IDENTITY};

/// Shape cast versus a circle. (b2ShapeCastCircle)
pub fn shape_cast_circle(shape: &Circle, input: &ShapeCastInput) -> CastOutput {
    let pair_input = ShapeCastPairInput {
        proxy_a: make_proxy(&[shape.center], shape.radius),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };

    shape_cast(&pair_input)
}

/// Shape cast versus a capsule. (b2ShapeCastCapsule)
pub fn shape_cast_capsule(shape: &Capsule, input: &ShapeCastInput) -> CastOutput {
    let pair_input = ShapeCastPairInput {
        proxy_a: make_proxy(&[shape.center1, shape.center2], shape.radius),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };

    shape_cast(&pair_input)
}

/// Shape cast versus a line segment. (b2ShapeCastSegment)
pub fn shape_cast_segment(shape: &Segment, input: &ShapeCastInput) -> CastOutput {
    let pair_input = ShapeCastPairInput {
        proxy_a: make_proxy(&[shape.point1, shape.point2], 0.0),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };

    shape_cast(&pair_input)
}

/// Shape cast versus a convex polygon. (b2ShapeCastPolygon)
pub fn shape_cast_polygon(shape: &Polygon, input: &ShapeCastInput) -> CastOutput {
    let pair_input = ShapeCastPairInput {
        proxy_a: make_proxy(&shape.vertices[..shape.count as usize], shape.radius),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };

    shape_cast(&pair_input)
}

/// Collide a mover capsule against a circle. (b2CollideMoverAndCircle)
pub fn collide_mover_and_circle(mover: &Capsule, shape: &Circle) -> PlaneResult {
    let distance_input = DistanceInput {
        proxy_a: make_proxy(&[shape.center], 0.0),
        proxy_b: make_proxy(&[mover.center1, mover.center2], mover.radius),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let total_radius = mover.radius + shape.radius;

    let mut cache = SimplexCache::default();
    let distance_output = shape_distance(&distance_input, &mut cache, None);

    if distance_output.distance <= total_radius {
        let plane = Plane {
            normal: distance_output.normal,
            offset: total_radius - distance_output.distance,
        };
        return PlaneResult {
            plane,
            point: distance_output.point_a,
            hit: true,
        };
    }

    PlaneResult::default()
}

/// Collide a mover capsule against a capsule. (b2CollideMoverAndCapsule)
pub fn collide_mover_and_capsule(mover: &Capsule, shape: &Capsule) -> PlaneResult {
    let distance_input = DistanceInput {
        proxy_a: make_proxy(&[shape.center1, shape.center2], 0.0),
        proxy_b: make_proxy(&[mover.center1, mover.center2], mover.radius),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let total_radius = mover.radius + shape.radius;

    let mut cache = SimplexCache::default();
    let distance_output = shape_distance(&distance_input, &mut cache, None);

    if distance_output.distance <= total_radius {
        let plane = Plane {
            normal: distance_output.normal,
            offset: total_radius - distance_output.distance,
        };
        return PlaneResult {
            plane,
            point: distance_output.point_a,
            hit: true,
        };
    }

    PlaneResult::default()
}

/// Collide a mover capsule against a polygon. (b2CollideMoverAndPolygon)
pub fn collide_mover_and_polygon(mover: &Capsule, shape: &Polygon) -> PlaneResult {
    let distance_input = DistanceInput {
        proxy_a: make_proxy(&shape.vertices[..shape.count as usize], shape.radius),
        proxy_b: make_proxy(&[mover.center1, mover.center2], mover.radius),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let total_radius = mover.radius + shape.radius;

    let mut cache = SimplexCache::default();
    let distance_output = shape_distance(&distance_input, &mut cache, None);

    if distance_output.distance <= total_radius {
        let plane = Plane {
            normal: distance_output.normal,
            offset: total_radius - distance_output.distance,
        };
        return PlaneResult {
            plane,
            point: distance_output.point_a,
            hit: true,
        };
    }

    PlaneResult::default()
}

/// Collide a mover capsule against a segment. (b2CollideMoverAndSegment)
pub fn collide_mover_and_segment(mover: &Capsule, shape: &Segment) -> PlaneResult {
    let distance_input = DistanceInput {
        proxy_a: make_proxy(&[shape.point1, shape.point2], 0.0),
        proxy_b: make_proxy(&[mover.center1, mover.center2], mover.radius),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let total_radius = mover.radius;

    let mut cache = SimplexCache::default();
    let distance_output = shape_distance(&distance_input, &mut cache, None);

    if distance_output.distance <= total_radius {
        let plane = Plane {
            normal: distance_output.normal,
            offset: total_radius - distance_output.distance,
        };
        return PlaneResult {
            plane,
            point: distance_output.point_a,
            hit: true,
        };
    }

    PlaneResult::default()
}
