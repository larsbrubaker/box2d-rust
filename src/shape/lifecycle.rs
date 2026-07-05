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
    let id = create_shape(world, body_id, def, ShapeGeometry::Circle(*circle));
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_shape(
            rec,
            crate::recording::OP_CREATE_CIRCLE_SHAPE,
            body_id,
            def,
            |buf| crate::recording::rec_w_circle(buf, *circle),
            id,
        )
    });
    id
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

    let id = create_shape(world, body_id, def, ShapeGeometry::Capsule(*capsule));
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_shape(
            rec,
            crate::recording::OP_CREATE_CAPSULE_SHAPE,
            body_id,
            def,
            |buf| crate::recording::rec_w_capsule(buf, *capsule),
            id,
        )
    });
    id
}

/// (b2CreatePolygonShape)
pub fn create_polygon_shape(
    world: &mut crate::world::World,
    body_id: crate::id::BodyId,
    def: &crate::types::ShapeDef,
    polygon: &crate::collision::Polygon,
) -> crate::id::ShapeId {
    debug_assert!(crate::math_functions::is_valid_float(polygon.radius) && polygon.radius >= 0.0);

    let id = create_shape(world, body_id, def, ShapeGeometry::Polygon(*polygon));
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_shape(
            rec,
            crate::recording::OP_CREATE_POLYGON_SHAPE,
            body_id,
            def,
            |buf| crate::recording::rec_w_polygon(buf, polygon),
            id,
        )
    });
    id
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

    let id = create_shape(world, body_id, def, ShapeGeometry::Segment(*segment));
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_shape(
            rec,
            crate::recording::OP_CREATE_SEGMENT_SHAPE,
            body_id,
            def,
            |buf| crate::recording::rec_w_segment(buf, *segment),
            id,
        )
    });
    id
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

    let id = create_shape(world, body_id, def, ShapeGeometry::ChainSegment(local));
    // C records `local` (parent chain id nulled), not the caller value.
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_shape(
            rec,
            crate::recording::OP_CREATE_CHAIN_SEGMENT_SHAPE,
            body_id,
            def,
            |buf| crate::recording::rec_w_chainseg(buf, local),
            id,
        )
    });
    id
}

/// Validate a ShapeId and return the raw shape index. (b2GetShape — C returns
/// a pointer; Rust returns the index into `world.shapes`)
pub fn get_shape_index(world: &crate::world::World, shape_id: crate::id::ShapeId) -> i32 {
    let index = shape_id.index1 - 1;
    debug_assert!((index as usize) < world.shapes.len());
    let shape = &world.shapes[index as usize];
    debug_assert!(shape.id == index && shape.generation == shape_id.generation);
    index
}

/// (static b2DestroyShapeInternal — C takes shape/body pointers; the Rust
/// port takes ids)
pub(crate) fn destroy_shape_internal(
    world: &mut crate::world::World,
    shape_id: i32,
    body_id: i32,
    wake_bodies: bool,
) {
    // Remove the shape from the body's doubly linked list.
    let (prev_shape_id, next_shape_id) = {
        let shape = &world.shapes[shape_id as usize];
        (shape.prev_shape_id, shape.next_shape_id)
    };

    if prev_shape_id != NULL_INDEX {
        world.shapes[prev_shape_id as usize].next_shape_id = next_shape_id;
    }

    if next_shape_id != NULL_INDEX {
        world.shapes[next_shape_id as usize].prev_shape_id = prev_shape_id;
    }

    {
        let body = &mut world.bodies[body_id as usize];
        if shape_id == body.head_shape_id {
            body.head_shape_id = next_shape_id;
        }
        body.shape_count -= 1;
    }

    // Remove from broad-phase.
    {
        let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
        super::destroy_shape_proxy(&mut shapes[shape_id as usize], broad_phase);
    }

    // Destroy any contacts associated with the shape.
    let mut contact_key = world.bodies[body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        contact_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;

        let (contact_shape_a, contact_shape_b) = {
            let contact = &world.contacts[contact_id as usize];
            (contact.shape_id_a, contact.shape_id_b)
        };
        if contact_shape_a == shape_id || contact_shape_b == shape_id {
            crate::contact::destroy_contact(world, contact_id, wake_bodies);
        }
    }

    if world.shapes[shape_id as usize].sensor_index != NULL_INDEX {
        // The C code inlines a copy of b2DestroySensor here (end-touch events
        // for active overlaps, swap-remove with sensorIndex fixup).
        crate::sensor::destroy_sensor(world, shape_id);
    }

    // Return shape to free list.
    world.shape_id_pool.free_id(shape_id);
    world.shapes[shape_id as usize].id = NULL_INDEX;

    world.validate_solver_sets();
}

/// Destroy a shape by id, optionally updating the body mass.
/// (b2DestroyShape)
pub fn destroy_shape(
    world: &mut crate::world::World,
    shape_id: crate::id::ShapeId,
    update_body_mass: bool,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_destroy_shape(rec, shape_id, update_body_mass)
    });
    let shape_index = get_shape_index(world, shape_id);

    // Cannot destroy a chain segment that has a parent chain shape
    if let ShapeGeometry::ChainSegment(chain_segment) = &world.shapes[shape_index as usize].geometry
    {
        if chain_segment.chain_id != NULL_INDEX {
            debug_assert!(false, "cannot destroy a chain segment owned by a chain");
            return;
        }
    }

    // need to wake bodies because this might be a static body
    let wake_bodies = true;
    let body_id = world.shapes[shape_index as usize].body_id;
    destroy_shape_internal(world, shape_index, body_id, wake_bodies);

    if update_body_mass {
        crate::body::update_body_mass_data(world, body_id);
    }
}
