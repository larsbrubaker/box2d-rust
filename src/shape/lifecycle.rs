// Shape creation from shape.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{create_shape_proxy, get_shape_centroid, Shape};
use crate::collision::ShapeGeometry;
use crate::core::NULL_INDEX;
use crate::math_functions::{max_float, min_float, Aabb, WorldTransform};

/// AABB margin for the broad phase fat AABB, limited by shape size.
/// (static b2ComputeShapeMargin)
pub(crate) fn compute_shape_margin(shape: &Shape) -> f32 {
    use crate::constants::{max_aabb_margin, AABB_MARGIN_FRACTION};
    use crate::math_functions::distance;

    let margin = match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            0.5 * distance(capsule.center2, capsule.center1) + capsule.radius
        }
        ShapeGeometry::Circle(circle) => circle.radius,
        ShapeGeometry::Polygon(poly) => {
            let mut max_extent_sqr = 0.0f32;
            for i in 0..poly.count as usize {
                let distance_sqr =
                    crate::math_functions::distance_squared(poly.vertices[i], poly.centroid);
                max_extent_sqr = max_float(max_extent_sqr, distance_sqr);
            }
            max_extent_sqr.sqrt()
        }
        ShapeGeometry::Segment(segment) => 0.5 * distance(segment.point1, segment.point2),
        ShapeGeometry::ChainSegment(chain_segment) => {
            0.5 * distance(chain_segment.segment.point1, chain_segment.segment.point2)
        }
    };

    min_float(max_aabb_margin(), AABB_MARGIN_FRACTION * margin)
}

/// Create a shape on a body. Returns the raw shape id.
/// (static b2CreateShapeInternal — the C void* geometry + type tag is the
/// ShapeGeometry enum)
pub(crate) fn create_shape_internal(
    world: &mut crate::world::World,
    body_id: i32,
    transform: WorldTransform,
    def: &crate::types::ShapeDef,
    geometry: ShapeGeometry,
) -> i32 {
    let shape_id = world.shape_id_pool.alloc_id();

    if shape_id == world.shapes.len() as i32 {
        world.shapes.push(Shape::default());
    } else {
        debug_assert!(world.shapes[shape_id as usize].id == NULL_INDEX);
    }

    let (body_raw_id, body_set_index, body_type, head_shape_id) = {
        let body = &world.bodies[body_id as usize];
        (body.id, body.set_index, body.type_, body.head_shape_id)
    };

    {
        let shape = &mut world.shapes[shape_id as usize];
        shape.geometry = geometry;

        shape.id = shape_id;
        shape.body_id = body_raw_id;
        shape.density = def.density;
        shape.material = def.material;
        shape.filter = def.filter;
        shape.user_data = def.user_data;
        shape.enlarged_aabb = false;
        shape.enable_sensor_events = def.enable_sensor_events;
        shape.enable_contact_events = def.enable_contact_events;
        shape.enable_custom_filtering = def.enable_custom_filtering;
        shape.enable_hit_events = def.enable_hit_events;
        shape.enable_pre_solve_events = def.enable_pre_solve_events;
        shape.proxy_key = NULL_INDEX;
        shape.local_centroid = get_shape_centroid(shape);
        shape.aabb_margin = compute_shape_margin(shape);
        shape.aabb = Aabb::default();
        shape.fat_aabb = Aabb::default();
        shape.generation += 1;
    }

    if body_set_index != crate::solver_set::DISABLED_SET {
        // Split borrows: the shape and the broad phase are different World fields.
        let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
        create_shape_proxy(
            &mut shapes[shape_id as usize],
            broad_phase,
            body_type,
            transform,
            def.invoke_contact_creation || def.is_sensor,
        );
    }

    // Add to shape doubly linked list
    if head_shape_id != NULL_INDEX {
        world.shapes[head_shape_id as usize].prev_shape_id = shape_id;
    }

    world.shapes[shape_id as usize].prev_shape_id = NULL_INDEX;
    world.shapes[shape_id as usize].next_shape_id = head_shape_id;
    world.bodies[body_id as usize].head_shape_id = shape_id;
    world.bodies[body_id as usize].shape_count += 1;

    if def.is_sensor {
        world.shapes[shape_id as usize].sensor_index = world.sensors.len() as i32;
        world.sensors.push(crate::sensor::Sensor::new(shape_id));
    } else {
        world.shapes[shape_id as usize].sensor_index = NULL_INDEX;
    }

    world.validate_solver_sets();

    shape_id
}

/// (static b2CreateShape)
fn create_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    geometry: ShapeGeometry,
) -> crate::id::ShapeId {
    use crate::body::body_flags::DIRTY_MASS;
    use crate::body::{
        get_body_full_id, get_body_transform, sync_body_flags, update_body_mass_data,
    };
    use crate::math_functions::is_valid_float;

    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(is_valid_float(def.density) && def.density >= 0.0);
    debug_assert!(is_valid_float(def.material.friction) && def.material.friction >= 0.0);
    debug_assert!(is_valid_float(def.material.restitution) && def.material.restitution >= 0.0);
    debug_assert!(
        is_valid_float(def.material.rolling_resistance) && def.material.rolling_resistance >= 0.0
    );
    debug_assert!(is_valid_float(def.material.tangent_speed));

    let body_index = get_body_full_id(world, body_id);
    let transform = get_body_transform(world, body_index);

    let shape_id = create_shape_internal(world, body_index, transform, def, geometry);

    if def.update_body_mass {
        update_body_mass_data(world, body_index);
    } else if world.bodies[body_index as usize].flags & DIRTY_MASS == 0 {
        world.bodies[body_index as usize].flags |= DIRTY_MASS;
        sync_body_flags(world, body_index);
    }

    world.validate_solver_sets();

    crate::id::ShapeId {
        index1: shape_id + 1,
        world0: body_id.world0,
        generation: world.shapes[shape_id as usize].generation,
    }
}

/// (b2CreateCircleShape)
pub fn create_circle_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    circle: &crate::collision::Circle,
) -> crate::id::ShapeId {
    create_shape(world, body_id, def, ShapeGeometry::Circle(*circle))
}

/// (b2CreateCapsuleShape)
pub fn create_capsule_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    capsule: &crate::collision::Capsule,
) -> crate::id::ShapeId {
    use crate::constants::linear_slop;
    use crate::math_functions::distance_squared;

    let length_sqr = distance_squared(capsule.center1, capsule.center2);
    if length_sqr <= linear_slop() * linear_slop() {
        return crate::id::ShapeId::default();
    }

    create_shape(world, body_id, def, ShapeGeometry::Capsule(*capsule))
}

/// (b2CreatePolygonShape)
pub fn create_polygon_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    polygon: &crate::collision::Polygon,
) -> crate::id::ShapeId {
    debug_assert!(crate::math_functions::is_valid_float(polygon.radius) && polygon.radius >= 0.0);

    create_shape(world, body_id, def, ShapeGeometry::Polygon(*polygon))
}

/// (b2CreateSegmentShape)
pub fn create_segment_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    segment: &crate::collision::Segment,
) -> crate::id::ShapeId {
    use crate::constants::linear_slop;
    use crate::math_functions::distance_squared;

    let length_sqr = distance_squared(segment.point1, segment.point2);
    if length_sqr <= linear_slop() * linear_slop() {
        debug_assert!(false);
        return crate::id::ShapeId::default();
    }

    create_shape(world, body_id, def, ShapeGeometry::Segment(*segment))
}

/// (b2CreateChainSegmentShape)
pub fn create_chain_segment_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    chain_segment: &crate::collision::ChainSegment,
) -> crate::id::ShapeId {
    use crate::constants::linear_slop;
    use crate::math_functions::distance_squared;

    let length_sqr = distance_squared(chain_segment.segment.point1, chain_segment.segment.point2);
    if length_sqr <= linear_slop() * linear_slop() {
        debug_assert!(false);
        return crate::id::ShapeId::default();
    }

    // No parent chain shape
    let mut local = *chain_segment;
    local.chain_id = NULL_INDEX;

    create_shape(world, body_id, def, ShapeGeometry::ChainSegment(local))
}
