// Geometry access for the b2Shape_* API: typed getters, the b2Shape_Set*
// geometry replacements (with proxy reset), parent chain lookup, and wind.
// Split from api.rs to stay under the file-length limit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::api::reset_proxy;
use super::{compute_shape_margin, get_shape_index};
use crate::body::wake_body;
use crate::collision::{Capsule, ChainSegment, Circle, Polygon, Segment, ShapeGeometry, ShapeType};
use crate::core::NULL_INDEX;
use crate::id::{ChainId, ShapeId};
use crate::math_functions::{
    abs_float, add, cross, cross_sv, distance_squared, dot, get_length_and_normalize, left_perp,
    lerp, mul_add, mul_sub, mul_sv, neg, normalize, right_perp, rotate_vector, sub, Vec2,
    VEC2_ZERO,
};
use crate::solver_set::{AWAKE_SET, FIRST_SLEEPING_SET};
use crate::types::BodyType;
use crate::world::World;

/// Get a copy of the shape's circle. Asserts the type is correct.
/// (b2Shape_GetCircle)
pub fn shape_get_circle(world: &World, shape_id: ShapeId) -> Circle {
    let shape_index = get_shape_index(world, shape_id);
    match &world.shapes[shape_index as usize].geometry {
        ShapeGeometry::Circle(circle) => *circle,
        _ => unreachable!("shape is not a circle"),
    }
}

/// Get a copy of the shape's line segment. Asserts the type is correct.
/// (b2Shape_GetSegment)
pub fn shape_get_segment(world: &World, shape_id: ShapeId) -> Segment {
    let shape_index = get_shape_index(world, shape_id);
    match &world.shapes[shape_index as usize].geometry {
        ShapeGeometry::Segment(segment) => *segment,
        _ => unreachable!("shape is not a segment"),
    }
}

/// Get a copy of the shape's chain segment. These come from chain shapes.
/// Asserts the type is correct. (b2Shape_GetChainSegment)
pub fn shape_get_chain_segment(world: &World, shape_id: ShapeId) -> ChainSegment {
    let shape_index = get_shape_index(world, shape_id);
    match &world.shapes[shape_index as usize].geometry {
        ShapeGeometry::ChainSegment(chain_segment) => *chain_segment,
        _ => unreachable!("shape is not a chain segment"),
    }
}

/// Get a copy of the shape's capsule. Asserts the type is correct.
/// (b2Shape_GetCapsule)
pub fn shape_get_capsule(world: &World, shape_id: ShapeId) -> Capsule {
    let shape_index = get_shape_index(world, shape_id);
    match &world.shapes[shape_index as usize].geometry {
        ShapeGeometry::Capsule(capsule) => *capsule,
        _ => unreachable!("shape is not a capsule"),
    }
}

/// Get a copy of the shape's convex polygon. Asserts the type is correct.
/// (b2Shape_GetPolygon)
pub fn shape_get_polygon(world: &World, shape_id: ShapeId) -> Polygon {
    let shape_index = get_shape_index(world, shape_id);
    match &world.shapes[shape_index as usize].geometry {
        ShapeGeometry::Polygon(polygon) => *polygon,
        _ => unreachable!("shape is not a polygon"),
    }
}

/// Change the geometry after a shape's geometry field is replaced: recompute
/// the margin and refresh the proxy, waking attached bodies. Shared tail of
/// the b2Shape_Set* functions.
fn set_geometry(world: &mut World, shape_index: i32, geometry: ShapeGeometry) {
    {
        let shape = &mut world.shapes[shape_index as usize];
        shape.geometry = geometry;
        shape.aabb_margin = compute_shape_margin(shape);
    }

    // need to wake bodies so they can react to the shape change
    let wake_bodies = true;
    let destroy_proxy = true;
    reset_proxy(world, shape_index, wake_bodies, destroy_proxy);
}

/// Allows you to change a shape to be a circle or update the current circle.
/// (b2Shape_SetCircle)
pub fn shape_set_circle(world: &mut World, shape_id: ShapeId, circle: &Circle) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    set_geometry(world, shape_index, ShapeGeometry::Circle(*circle));
}

/// Allows you to change a shape to be a capsule or update the current capsule.
/// (b2Shape_SetCapsule)
pub fn shape_set_capsule(world: &mut World, shape_id: ShapeId, capsule: &Capsule) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let length_sqr = distance_squared(capsule.center1, capsule.center2);
    if length_sqr <= crate::constants::linear_slop() * crate::constants::linear_slop() {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    set_geometry(world, shape_index, ShapeGeometry::Capsule(*capsule));
}

/// Allows you to change a shape to be a segment or update the current segment.
/// (b2Shape_SetSegment)
pub fn shape_set_segment(world: &mut World, shape_id: ShapeId, segment: &Segment) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    set_geometry(world, shape_index, ShapeGeometry::Segment(*segment));
}

/// Allows you to change a shape to be a polygon or update the current polygon.
/// (b2Shape_SetPolygon)
pub fn shape_set_polygon(world: &mut World, shape_id: ShapeId, polygon: &Polygon) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    set_geometry(world, shape_index, ShapeGeometry::Polygon(*polygon));
}

/// Allows you to change a shape to be a stand-alone chain segment or update
/// the current one. Cannot be used on a segment owned by a chain shape.
/// (b2Shape_SetChainSegment)
pub fn shape_set_chain_segment(world: &mut World, shape_id: ShapeId, chain_segment: &ChainSegment) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);

    // Cannot modify a chain segment that has a parent chain shape
    if let ShapeGeometry::ChainSegment(existing) = &world.shapes[shape_index as usize].geometry {
        if existing.chain_id != NULL_INDEX {
            debug_assert!(false, "cannot modify a chain segment owned by a chain");
            return;
        }
    }

    let length_sqr = distance_squared(chain_segment.segment.point1, chain_segment.segment.point2);
    if length_sqr <= crate::constants::linear_slop() * crate::constants::linear_slop() {
        return;
    }

    let mut local = *chain_segment;
    local.chain_id = NULL_INDEX;
    set_geometry(world, shape_index, ShapeGeometry::ChainSegment(local));
}

/// Get the parent chain id if the shape type is a chain segment, otherwise
/// returns the null chain id. (b2Shape_GetParentChain)
pub fn shape_get_parent_chain(world: &World, shape_id: ShapeId) -> ChainId {
    let shape_index = get_shape_index(world, shape_id);
    if let ShapeGeometry::ChainSegment(chain_segment) = &world.shapes[shape_index as usize].geometry
    {
        let chain_id = chain_segment.chain_id;
        if chain_id != NULL_INDEX {
            let chain = &world.chain_shapes[chain_id as usize];
            return ChainId {
                index1: chain_id + 1,
                world0: world.world_id,
                generation: chain.generation,
            };
        }
    }

    ChainId::default()
}

/// Apply wind to a shape, applying drag and lift forces to the owning body.
/// (b2Shape_ApplyWind)
///
/// force = 0.5 * air_density * velocity^2 * area
/// <https://en.wikipedia.org/wiki/Density_of_air>
/// <https://en.wikipedia.org/wiki/Lift_(force)>
pub fn shape_apply_wind(
    world: &mut World,
    shape_id: ShapeId,
    wind: Vec2,
    drag: f32,
    lift: f32,
    wake: bool,
) {
    let shape_index = get_shape_index(world, shape_id);

    let shape_type = world.shapes[shape_index as usize].shape_type();
    if shape_type != ShapeType::Circle
        && shape_type != ShapeType::Capsule
        && shape_type != ShapeType::Polygon
    {
        return;
    }

    let body_id = world.shapes[shape_index as usize].body_id;

    if world.bodies[body_id as usize].type_ != BodyType::Dynamic {
        return;
    }

    if world.bodies[body_id as usize].set_index >= FIRST_SLEEPING_SET && !wake {
        return;
    }

    if world.bodies[body_id as usize].set_index != AWAKE_SET {
        // Must wake for state to exist
        wake_body(world, body_id);
    }

    debug_assert!(world.bodies[body_id as usize].set_index == AWAKE_SET);

    let local_index = world.bodies[body_id as usize].local_index;
    let (transform, local_center) = {
        let sim = &world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
        (sim.transform, sim.local_center)
    };
    let (linear_velocity, angular_velocity) = {
        let state = &world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
        (state.linear_velocity, state.angular_velocity)
    };
    let shape = &world.shapes[shape_index as usize];

    let length_units = crate::core::get_length_units_per_meter();
    let volume_units = length_units * length_units * length_units;

    // In 2D I'm assuming unit depth
    let air_density = 1.2250 / volume_units;

    let mut force = VEC2_ZERO;
    let mut torque = 0.0f32;

    match &shape.geometry {
        ShapeGeometry::Circle(circle) => {
            let radius = circle.radius;
            let centroid = shape.local_centroid;
            let lever = rotate_vector(transform.q, sub(centroid, local_center));
            let shape_velocity = add(linear_velocity, cross_sv(angular_velocity, lever));
            let relative_velocity = mul_sub(wind, drag, shape_velocity);
            let mut speed = 0.0;
            let direction = get_length_and_normalize(&mut speed, relative_velocity);
            let projected_area = 2.0 * radius;
            force = mul_sv(
                0.5 * air_density * projected_area * speed * speed,
                direction,
            );
            torque = cross(lever, force);
        }

        ShapeGeometry::Capsule(capsule) => {
            let centroid = shape.local_centroid;
            let lever = rotate_vector(transform.q, sub(centroid, local_center));
            let shape_velocity = add(linear_velocity, cross_sv(angular_velocity, lever));
            let relative_velocity = mul_sub(wind, drag, shape_velocity);
            let mut speed = 0.0;
            let direction = get_length_and_normalize(&mut speed, relative_velocity);

            let mut d = sub(capsule.center2, capsule.center1);
            d = rotate_vector(transform.q, d);

            let radius = capsule.radius;
            let projected_area = 2.0 * radius + abs_float(cross(d, direction));

            // Normal that opposes the wind
            let mut normal = left_perp(normalize(d));
            if dot(normal, direction) > 0.0 {
                normal = neg(normal);
            }

            // portion of wind that is perpendicular to surface
            let lift_direction = cross_sv(cross(normal, direction), direction);

            let force_magnitude = 0.5 * air_density * projected_area * speed * speed;
            force = mul_sv(force_magnitude, mul_add(direction, lift, lift_direction));

            let edge_lever = mul_add(lever, radius, normal);
            torque = cross(edge_lever, force);
        }

        ShapeGeometry::Polygon(polygon) => {
            let centroid = shape.local_centroid;
            let lever = rotate_vector(transform.q, sub(centroid, local_center));
            let shape_velocity = add(linear_velocity, cross_sv(angular_velocity, lever));
            let relative_velocity = mul_sub(wind, drag, shape_velocity);
            let mut speed = 0.0;
            let direction = get_length_and_normalize(&mut speed, relative_velocity);

            // polygon radius is ignored for simplicity
            let count = polygon.count as usize;
            let vertices = &polygon.vertices;

            let mut v1 = vertices[count - 1];
            for vertex in vertices[..count].iter() {
                let v2 = *vertex;
                let mut d = sub(v2, v1);
                let edge_center = lerp(v1, v2, 0.5);
                v1 = v2;

                d = rotate_vector(transform.q, d);

                let projected_area = cross(d, direction);
                if projected_area <= 0.0 {
                    // back facing
                    continue;
                }

                let normal = right_perp(normalize(d));

                // portion of wind that is perpendicular to surface
                let lift_direction = cross_sv(cross(normal, direction), direction);

                let force_magnitude = 0.5 * air_density * projected_area * speed * speed;
                let f = mul_sv(force_magnitude, mul_add(direction, lift, lift_direction));

                let edge_lever = rotate_vector(transform.q, sub(edge_center, local_center));

                force = add(force, f);
                torque += cross(edge_lever, f);
            }
        }

        _ => {}
    }

    let sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
    sim.force = add(sim.force, force);
    sim.torque += torque;
}
