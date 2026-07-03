// Port of box2d-cpp-reference/src/geometry.c.
//
// Split to satisfy the 800-line file limit:
// - shapes.rs     — polygon constructors and transforms
// - mass.rs       — mass properties for circle, capsule, polygon
// - bounds.rs     — shape AABB computation (fat and tight)
// - point_ray.rs  — point-in-shape tests and ray casts
// - shape_cast.rs — shape cast wrappers and mover collision
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

mod bounds;
mod mass;
mod point_ray;
mod shape_cast;
mod shapes;

pub use bounds::{
    compute_capsule_aabb, compute_circle_aabb, compute_fat_shape_aabb, compute_polygon_aabb,
    compute_segment_aabb,
};
pub use mass::{compute_capsule_mass, compute_circle_mass, compute_polygon_mass};
pub use point_ray::{
    point_in_capsule, point_in_circle, point_in_polygon, ray_cast_capsule, ray_cast_circle,
    ray_cast_polygon, ray_cast_segment,
};
pub use shape_cast::{
    collide_mover_and_capsule, collide_mover_and_circle, collide_mover_and_polygon,
    collide_mover_and_segment, shape_cast_capsule, shape_cast_circle, shape_cast_polygon,
    shape_cast_segment,
};
pub use shapes::{
    make_box, make_offset_box, make_offset_polygon, make_offset_rounded_box,
    make_offset_rounded_polygon, make_polygon, make_rounded_box, make_square, transform_polygon,
};

use crate::collision::RayCastInput;
use crate::constants::huge;
use crate::math_functions::{is_valid_float, is_valid_vec2};

/// Validate ray cast input data (NaN, etc). (b2IsValidRay)
pub fn is_valid_ray(input: &RayCastInput) -> bool {
    is_valid_vec2(input.origin)
        && is_valid_vec2(input.translation)
        && is_valid_float(input.max_fraction)
        && 0.0 <= input.max_fraction
        && input.max_fraction < huge()
}
