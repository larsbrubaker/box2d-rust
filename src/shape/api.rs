// The b2Shape_* public API from shape.c: accessors, geometry get/set with
// proxy reset, event flags, contact/sensor introspection, queries, and wind.
//
// b2Shape_GetWorld is omitted: there is no world registry in the Rust port.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{
    compute_shape_mass, get_shape_index, make_shape_distance_proxy, ray_cast_shape,
    update_shape_aabbs,
};
use crate::body::{
    get_body_transform, get_body_transform_quick, make_body_id, update_body_mass_data,
};
use crate::collision::{MassData, RayCastInput, ShapeGeometry, ShapeType, WorldCastOutput};
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::events::ContactData;
use crate::geometry::{point_in_capsule, point_in_circle, point_in_polygon};
use crate::id::{BodyId, ContactId, ShapeId};
use crate::math_functions::{
    inv_mul_world_transforms, inv_transform_world_point, is_valid_float, is_valid_position,
    is_valid_vec2, offset_pos, to_relative_transform, transform_world_point, Aabb, Pos, Vec2,
    ROT_IDENTITY, VEC2_ZERO,
};
use crate::solver_set::AWAKE_SET;
use crate::types::{Filter, SurfaceMaterial};
use crate::world::World;

/// Shape id validity. (b2Shape_IsValid — the world-registry check collapses
/// to the index/generation check in the registry-less port)
pub fn shape_is_valid(world: &World, id: ShapeId) -> bool {
    let shape_id = id.index1 - 1;
    if shape_id < 0 || world.shapes.len() as i32 <= shape_id {
        return false;
    }

    let shape = &world.shapes[shape_id as usize];
    if shape.id == NULL_INDEX {
        // shape is free
        return false;
    }

    debug_assert!(shape.id == shape_id);

    id.generation == shape.generation
}

/// Get the id of the body that a shape is attached to. (b2Shape_GetBody)
pub fn shape_get_body(world: &World, shape_id: ShapeId) -> BodyId {
    let shape_index = get_shape_index(world, shape_id);
    make_body_id(world, world.shapes[shape_index as usize].body_id)
}

/// Set the user data for a shape. (b2Shape_SetUserData)
pub fn shape_set_user_data(world: &mut World, shape_id: ShapeId, user_data: u64) {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].user_data = user_data;
}

/// Get the user data for a shape. (b2Shape_GetUserData)
pub fn shape_get_user_data(world: &World, shape_id: ShapeId) -> u64 {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].user_data
}

/// Returns true if the shape is a sensor. (b2Shape_IsSensor)
pub fn shape_is_sensor(world: &World, shape_id: ShapeId) -> bool {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].sensor_index != NULL_INDEX
}

/// Test a point for overlap with a shape. (b2Shape_TestPoint)
pub fn shape_test_point(world: &mut World, shape_id: ShapeId, point: Pos) -> bool {
    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];

    let transform = get_body_transform(world, shape.body_id);
    let local_point = inv_transform_world_point(transform, point);

    let result = match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => point_in_capsule(capsule, local_point),
        ShapeGeometry::Circle(circle) => point_in_circle(circle, local_point),
        ShapeGeometry::Polygon(polygon) => point_in_polygon(polygon, local_point),
        _ => false,
    };

    crate::recording::record_query_result(
        world,
        crate::recording::OP_SHAPE_TEST_POINT,
        |buf| {
            crate::recording::rec_w_shapeid(buf, shape_id);
            crate::recording::rec_w_position(buf, point);
        },
        |buf| crate::recording::rec_w_bool(buf, result),
    );

    result
}

/// Ray cast a shape directly. (b2Shape_RayCast)
pub fn shape_ray_cast(
    world: &mut World,
    shape_id: ShapeId,
    origin: Pos,
    translation: Vec2,
) -> WorldCastOutput {
    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec2(translation));

    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];

    // Re-center on the origin so the cast runs in float precision
    let transform = to_relative_transform(get_body_transform(world, shape.body_id), origin);

    // The ray starts at the origin, so its origin in the re-centered frame is zero
    let input = RayCastInput {
        origin: VEC2_ZERO,
        translation,
        max_fraction: 1.0,
    };

    // Lift the re-centered float result back to a world position
    let local = ray_cast_shape(&input, shape, transform);
    let output = WorldCastOutput {
        normal: local.normal,
        point: offset_pos(origin, local.point),
        fraction: local.fraction,
        iterations: local.iterations,
        hit: local.hit,
    };

    crate::recording::record_query_result(
        world,
        crate::recording::OP_SHAPE_RAY_CAST,
        |buf| {
            crate::recording::rec_w_shapeid(buf, shape_id);
            crate::recording::rec_w_position(buf, origin);
            crate::recording::rec_w_vec2(buf, translation);
        },
        |buf| crate::recording::rec_w_worldcastoutput(buf, output),
    );

    output
}

/// Set the mass density of a shape, usually in kg/m^2. (b2Shape_SetDensity)
pub fn shape_set_density(
    world: &mut World,
    shape_id: ShapeId,
    density: f32,
    update_body_mass: bool,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_set_density(rec, shape_id, density, update_body_mass)
    });
    debug_assert!(is_valid_float(density) && density >= 0.0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    let shape = &mut world.shapes[shape_index as usize];
    if density == shape.density {
        // early return to avoid expensive function
        return;
    }

    shape.density = density;

    if update_body_mass {
        let body_id = shape.body_id;
        update_body_mass_data(world, body_id);
    }
}

/// Get the density of a shape, usually in kg/m^2. (b2Shape_GetDensity)
pub fn shape_get_density(world: &World, shape_id: ShapeId) -> f32 {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].density
}

/// Set the friction on a shape. (b2Shape_SetFriction)
pub fn shape_set_friction(world: &mut World, shape_id: ShapeId, friction: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_f32(
            rec,
            crate::recording::OP_SHAPE_SET_FRICTION,
            shape_id,
            friction,
        )
    });
    debug_assert!(is_valid_float(friction) && friction >= 0.0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material.friction = friction;
}

/// Get the friction of a shape. (b2Shape_GetFriction)
pub fn shape_get_friction(world: &World, shape_id: ShapeId) -> f32 {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material.friction
}

/// Set the shape restitution. (b2Shape_SetRestitution)
pub fn shape_set_restitution(world: &mut World, shape_id: ShapeId, restitution: f32) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_f32(
            rec,
            crate::recording::OP_SHAPE_SET_RESTITUTION,
            shape_id,
            restitution,
        )
    });
    debug_assert!(is_valid_float(restitution) && restitution >= 0.0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material.restitution = restitution;
}

/// Get the shape restitution. (b2Shape_GetRestitution)
pub fn shape_get_restitution(world: &World, shape_id: ShapeId) -> f32 {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material.restitution
}

/// Set the shape's user material identifier. (b2Shape_SetUserMaterial)
pub fn shape_set_user_material(world: &mut World, shape_id: ShapeId, material: u64) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_set_user_material(rec, shape_id, material)
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material.user_material_id = material;
}

/// Get the shape's user material identifier. (b2Shape_GetUserMaterial)
pub fn shape_get_user_material(world: &World, shape_id: ShapeId) -> u64 {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material.user_material_id
}

/// Get the shape's surface material. (b2Shape_GetSurfaceMaterial)
pub fn shape_get_surface_material(world: &World, shape_id: ShapeId) -> SurfaceMaterial {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material
}

/// Set the shape's surface material. (b2Shape_SetSurfaceMaterial)
pub fn shape_set_surface_material(
    world: &mut World,
    shape_id: ShapeId,
    surface_material: SurfaceMaterial,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_set_surface_material(rec, shape_id, surface_material)
    });
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].material = surface_material;
}

/// Get the shape filter. (b2Shape_GetFilter)
pub fn shape_get_filter(world: &World, shape_id: ShapeId) -> Filter {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].filter
}

/// Destroy this shape's contacts and refresh its broad-phase presence after a
/// filter or geometry change. (static b2ResetProxy)
pub(crate) fn reset_proxy(
    world: &mut World,
    shape_index: i32,
    wake_bodies: bool,
    destroy_proxy: bool,
) {
    let body_id = world.shapes[shape_index as usize].body_id;

    // destroy all contacts associated with this shape
    let mut contact_key = world.bodies[body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        contact_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;

        let (contact_shape_a, contact_shape_b) = {
            let contact = &world.contacts[contact_id as usize];
            (contact.shape_id_a, contact.shape_id_b)
        };
        if contact_shape_a == shape_index || contact_shape_b == shape_index {
            crate::contact::destroy_contact(world, contact_id, wake_bodies);
        }
    }

    let transform = get_body_transform_quick(world, &world.bodies[body_id as usize]);
    let proxy_key = world.shapes[shape_index as usize].proxy_key;
    if proxy_key != NULL_INDEX {
        let proxy_type = crate::broad_phase::proxy_type(proxy_key);
        update_shape_aabbs(
            &mut world.shapes[shape_index as usize],
            transform,
            proxy_type,
        );

        if destroy_proxy {
            world.broad_phase.destroy_proxy(proxy_key);

            let (fat_aabb, category_bits) = {
                let shape = &world.shapes[shape_index as usize];
                (shape.fat_aabb, shape.filter.category_bits)
            };
            let force_pair_creation = true;
            world.shapes[shape_index as usize].proxy_key = world.broad_phase.create_proxy(
                proxy_type,
                fat_aabb,
                category_bits,
                shape_index,
                force_pair_creation,
            );
        } else {
            let fat_aabb = world.shapes[shape_index as usize].fat_aabb;
            world.broad_phase.move_proxy(proxy_key, fat_aabb);
        }
    } else {
        let proxy_type = world.bodies[body_id as usize].type_;
        update_shape_aabbs(
            &mut world.shapes[shape_index as usize],
            transform,
            proxy_type,
        );
    }

    world.validate_solver_sets();
}

/// Set the current filter. This is almost as expensive as recreating the
/// shape. Sensor overlaps are not updated until the next time step.
/// (b2Shape_SetFilter)
pub fn shape_set_filter(world: &mut World, shape_id: ShapeId, filter: Filter) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_set_filter(rec, shape_id, filter)
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    {
        let shape = &world.shapes[shape_index as usize];
        if filter.mask_bits == shape.filter.mask_bits
            && filter.category_bits == shape.filter.category_bits
            && filter.group_index == shape.filter.group_index
        {
            return;
        }
    }

    // If the category bits change, I need to destroy the proxy because it
    // affects the tree sorting.
    let destroy_proxy =
        filter.category_bits != world.shapes[shape_index as usize].filter.category_bits;

    world.shapes[shape_index as usize].filter = filter;

    // need to wake bodies because a filter change may destroy contacts
    let wake_bodies = true;
    reset_proxy(world, shape_index, wake_bodies, destroy_proxy);

    // note: this does not immediately update sensor overlaps. Instead sensor
    // overlaps are updated the next time step
}

/// Enable sensor events for this shape. (b2Shape_EnableSensorEvents)
pub fn shape_enable_sensor_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_bool(
            rec,
            crate::recording::OP_SHAPE_ENABLE_SENSOR_EVENTS,
            shape_id,
            flag,
        )
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_sensor_events = flag;
}

/// Returns true if sensor events are enabled. (b2Shape_AreSensorEventsEnabled)
pub fn shape_are_sensor_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_sensor_events
}

/// Enable contact events for this shape. (b2Shape_EnableContactEvents)
pub fn shape_enable_contact_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_bool(
            rec,
            crate::recording::OP_SHAPE_ENABLE_CONTACT_EVENTS,
            shape_id,
            flag,
        )
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_contact_events = flag;
}

/// Returns true if contact events are enabled.
/// (b2Shape_AreContactEventsEnabled)
pub fn shape_are_contact_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_contact_events
}

/// Enable pre-solve contact events for this shape. Only applies to dynamic
/// bodies. These are expensive and must be carefully handled due to
/// multithreading. (b2Shape_EnablePreSolveEvents)
pub fn shape_enable_pre_solve_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_bool(
            rec,
            crate::recording::OP_SHAPE_ENABLE_PRE_SOLVE_EVENTS,
            shape_id,
            flag,
        )
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_pre_solve_events = flag;
}

/// Returns true if pre-solve events are enabled.
/// (b2Shape_ArePreSolveEventsEnabled)
pub fn shape_are_pre_solve_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_pre_solve_events
}

/// Enable contact hit events for this shape. (b2Shape_EnableHitEvents)
pub fn shape_enable_hit_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_shape_bool(
            rec,
            crate::recording::OP_SHAPE_ENABLE_HIT_EVENTS,
            shape_id,
            flag,
        )
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_hit_events = flag;
}

/// Returns true if hit events are enabled. (b2Shape_AreHitEventsEnabled)
pub fn shape_are_hit_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].enable_hit_events
}

/// Get the type of a shape. (b2Shape_GetType)
pub fn shape_get_type(world: &World, shape_id: ShapeId) -> ShapeType {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].shape_type()
}

/// Get the maximum capacity required for retrieving all the touching contacts
/// on a shape. (b2Shape_GetContactCapacity)
pub fn shape_get_contact_capacity(world: &World, shape_id: ShapeId) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index != NULL_INDEX {
        return 0;
    }

    // Conservative and fast
    world.bodies[shape.body_id as usize].contact_count
}

/// Get the touching contact data for a shape. (b2Shape_GetContactData —
/// returns a Vec instead of filling a caller array)
pub fn shape_get_contact_data(
    world: &World,
    shape_id: ShapeId,
    capacity: usize,
) -> Vec<ContactData> {
    debug_assert!(!world.locked);
    if world.locked {
        return Vec::new();
    }

    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index != NULL_INDEX {
        return Vec::new();
    }

    let mut out = Vec::new();
    let mut contact_key = world.bodies[shape.body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX && out.len() < capacity {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        let contact = &world.contacts[contact_id as usize];
        contact_key = contact.edges[edge_index as usize].next_key;

        // Does contact involve this shape and is it touching?
        if (contact.shape_id_a == shape_index || contact.shape_id_b == shape_index)
            && (contact.flags & crate::contact::contact_flags::TOUCHING) != 0
        {
            let shape_a = &world.shapes[contact.shape_id_a as usize];
            let shape_b = &world.shapes[contact.shape_id_b as usize];

            let contact_sim = if contact.set_index == AWAKE_SET && contact.color_index != NULL_INDEX
            {
                &world.constraint_graph.colors[contact.color_index as usize].contact_sims
                    [contact.local_index as usize]
            } else {
                &world.solver_sets[contact.set_index as usize].contact_sims
                    [contact.local_index as usize]
            };

            out.push(ContactData {
                contact_id: ContactId {
                    index1: contact_id + 1,
                    world0: world.world_id,
                    padding: 0,
                    generation: contact.generation,
                },
                shape_id_a: ShapeId {
                    index1: shape_a.id + 1,
                    world0: world.world_id,
                    generation: shape_a.generation,
                },
                shape_id_b: ShapeId {
                    index1: shape_b.id + 1,
                    world0: world.world_id,
                    generation: shape_b.generation,
                },
                manifold: contact_sim.manifold,
            });
        }
    }

    out
}

/// Get the maximum capacity required for retrieving all the overlapped shapes
/// on a sensor shape. Returns 0 if the shape is not a sensor.
/// (b2Shape_GetSensorCapacity)
pub fn shape_get_sensor_capacity(world: &World, shape_id: ShapeId) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index == NULL_INDEX {
        return 0;
    }

    world.sensors[shape.sensor_index as usize].overlaps2.len() as i32
}

/// Get the overlapped shapes for a sensor shape. Overlaps may contain
/// destroyed shapes, so use [`shape_is_valid`] to confirm each overlap.
/// (b2Shape_GetSensorData — returns a Vec instead of filling a caller array)
pub fn shape_get_sensor_data(world: &World, shape_id: ShapeId, capacity: usize) -> Vec<ShapeId> {
    debug_assert!(!world.locked);
    if world.locked {
        return Vec::new();
    }

    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index == NULL_INDEX {
        return Vec::new();
    }

    let sensor = &world.sensors[shape.sensor_index as usize];
    let count = sensor.overlaps2.len().min(capacity);
    sensor.overlaps2[..count]
        .iter()
        .map(|visitor| ShapeId {
            index1: visitor.shape_id + 1,
            world0: world.world_id,
            generation: visitor.generation,
        })
        .collect()
}

/// Get the current world AABB. This is the axis-aligned bounding box in world
/// coordinates. (b2Shape_GetAABB)
pub fn shape_get_aabb(world: &World, shape_id: ShapeId) -> Aabb {
    let shape_index = get_shape_index(world, shape_id);
    world.shapes[shape_index as usize].aabb
}

/// Compute the mass data for a shape. (b2Shape_ComputeMassData)
pub fn shape_compute_mass_data(world: &World, shape_id: ShapeId) -> MassData {
    let shape_index = get_shape_index(world, shape_id);
    compute_shape_mass(&world.shapes[shape_index as usize])
}

/// Get the closest point on a shape to a target point. Target and result are
/// in world space. (b2Shape_GetClosestPoint)
pub fn shape_get_closest_point(world: &World, shape_id: ShapeId, target: Pos) -> Pos {
    let shape_index = get_shape_index(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    let body = &world.bodies[shape.body_id as usize];
    let transform = get_body_transform_quick(world, body);

    // The target rides in as the frame of proxy B, so the relative pose is
    // differenced in double and the result stays exact far from the origin
    let zero = VEC2_ZERO;
    let target_transform = crate::math_functions::WorldTransform {
        p: target,
        q: ROT_IDENTITY,
    };

    let input = DistanceInput {
        proxy_a: make_shape_distance_proxy(shape),
        proxy_b: make_proxy(&[zero], 0.0),
        transform: inv_mul_world_transforms(transform, target_transform),
        use_radii: true,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    transform_world_point(transform, output.point_a)
}
