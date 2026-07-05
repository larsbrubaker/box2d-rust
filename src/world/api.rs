// The b2World_* public API from physics_world.c: validity, bounds, events,
// enable flags, tuning setters, counters, callbacks, explosions, and the
// static-tree rebuild. World queries (overlap/cast) live in query.rs.
//
// Omitted relative to C, all consequences of the registry-less, serial port:
// - b2CreateWorld/b2DestroyWorld map to World::new and drop.
// - b2World_SetWorkerCount / GetWorkerCount: the solver is serial; the world
//   always has exactly one worker context.
// - b2World_StartRecording / StopRecording: the recording subsystem is not
//   ported yet.
// - b2World_Draw / b2World_DumpMemoryStats: debug drawing and allocation
//   tracking are deferred.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{default_friction_callback, default_restitution_callback, World};
use super::{CustomFilterFcn, PreSolveFcn, Profile};
use crate::aabb::offset_aabb;
use crate::body::{get_body_transform_quick, wake_body};
use crate::constants::GRAPH_COLOR_COUNT;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::events::{BodyMoveEvent, ContactEvents, JointEvent, SensorEvents};
use crate::math_functions::{
    aabb_union, clamp_float, cross, inv_transform_world_point, is_valid_float, is_valid_position,
    left_perp, length_squared, max_int, mul_add, mul_sv, normalize, rotate_vector, sub, Aabb, Pos,
    Vec2, TRANSFORM_IDENTITY,
};
use crate::shape::{get_shape_centroid, get_shape_projected_perimeter, make_shape_distance_proxy};
use crate::solver_set::{AWAKE_SET, FIRST_SLEEPING_SET};
use crate::types::{
    BodyType, Capacity, Counters, ExplosionDef, FrictionCallback, RestitutionCallback,
};

/// World id validity. (b2World_IsValid)
///
/// C validates the id against the global world registry; the registry-less
/// Rust port owns the `World`, so a reachable world is valid unless it has
/// been torn down.
pub fn world_is_valid(world: &World) -> bool {
    world.in_use
}

/// Compute the bounds of all shapes in the world. Returns the bounds and
/// whether the world had any proxies at all. (b2ComputeWorldBounds)
pub fn compute_world_bounds(world: &World) -> (Aabb, bool) {
    let mut world_bounds = Aabb::default();
    let mut have_bounds = false;

    for tree in world.broad_phase.trees.iter() {
        if tree.proxy_count() == 0 {
            continue;
        }

        let tree_bounds = tree.root_bounds();
        world_bounds = if have_bounds {
            aabb_union(world_bounds, tree_bounds)
        } else {
            tree_bounds
        };
        have_bounds = true;
    }

    (world_bounds, have_bounds)
}

/// Get the bounds of all shapes in the world. Returns a zero AABB for an
/// empty world. (b2World_GetBounds)
pub fn world_get_bounds(world: &World) -> Aabb {
    debug_assert!(!world.locked);
    if world.locked {
        return Aabb::default();
    }

    let (bounds, _) = compute_world_bounds(world);
    bounds
}

/// Get the body events for the current time step. The event data is transient.
/// Do not store a reference to this data. (b2World_GetBodyEvents)
pub fn world_get_body_events(world: &World) -> &[BodyMoveEvent] {
    debug_assert!(!world.locked);
    if world.locked {
        return &[];
    }

    &world.body_move_events
}

/// Get sensor events for the current time step. The event data is transient.
/// Do not store a reference to this data. (b2World_GetSensorEvents)
pub fn world_get_sensor_events(world: &World) -> SensorEvents<'_> {
    debug_assert!(!world.locked);
    if world.locked {
        return SensorEvents {
            begin_events: &[],
            end_events: &[],
        };
    }

    // Careful to use previous buffer
    let end_event_array_index = 1 - world.end_event_array_index;

    SensorEvents {
        begin_events: &world.sensor_begin_events,
        end_events: &world.sensor_end_events[end_event_array_index as usize],
    }
}

/// Get contact events for this current time step. The event data is transient.
/// Do not store a reference to this data. (b2World_GetContactEvents)
pub fn world_get_contact_events(world: &World) -> ContactEvents<'_> {
    debug_assert!(!world.locked);
    if world.locked {
        return ContactEvents {
            begin_events: &[],
            end_events: &[],
            hit_events: &[],
        };
    }

    // Careful to use previous buffer
    let end_event_array_index = 1 - world.end_event_array_index;

    ContactEvents {
        begin_events: &world.contact_begin_events,
        end_events: &world.contact_end_events[end_event_array_index as usize],
        hit_events: &world.contact_hit_events,
    }
}

/// Get the joint events for the current time step. The event data is
/// transient. Do not store a reference to this data. (b2World_GetJointEvents)
pub fn world_get_joint_events(world: &World) -> &[JointEvent] {
    debug_assert!(!world.locked);
    if world.locked {
        return &[];
    }

    &world.joint_events
}

/// Enable/disable sleep. If your application does not need sleeping, you can
/// gain some performance by disabling sleep completely at the world level.
/// (b2World_EnableSleeping)
pub fn world_enable_sleeping(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    if flag == world.enable_sleep {
        return;
    }

    world.enable_sleep = flag;

    if !flag {
        let set_count = world.solver_sets.len();
        for i in (FIRST_SLEEPING_SET as usize)..set_count {
            if !world.solver_sets[i].body_sims.is_empty() {
                crate::solver_set::wake_solver_set(world, i as i32);
            }
        }
    }
}

/// Is body sleeping enabled? (b2World_IsSleepingEnabled)
pub fn world_is_sleeping_enabled(world: &World) -> bool {
    world.enable_sleep
}

/// Enable/disable constraint warm starting. Advanced feature for testing.
/// Disabling warm starting greatly reduces stability and provides no
/// performance gain. (b2World_EnableWarmStarting)
pub fn world_enable_warm_starting(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.enable_warm_starting = flag;
}

/// Is constraint warm starting enabled? (b2World_IsWarmStartingEnabled)
pub fn world_is_warm_starting_enabled(world: &World) -> bool {
    world.enable_warm_starting
}

/// Get the number of awake bodies. (b2World_GetAwakeBodyCount)
pub fn world_get_awake_body_count(world: &World) -> i32 {
    world.solver_sets[AWAKE_SET as usize].body_sims.len() as i32
}

/// Enable/disable continuous collision between dynamic and static bodies.
/// Generally you should keep continuous collision enabled to prevent fast
/// moving objects from going through static objects. The performance gain
/// from disabling continuous collision is minor. (b2World_EnableContinuous)
pub fn world_enable_continuous(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.enable_continuous = flag;
}

/// Is continuous collision enabled? (b2World_IsContinuousEnabled)
pub fn world_is_continuous_enabled(world: &World) -> bool {
    world.enable_continuous
}

/// Adjust the restitution threshold. It is recommended not to make this value
/// very small because it will prevent bodies from sleeping. Usually in meters
/// per second. (b2World_SetRestitutionThreshold)
pub fn world_set_restitution_threshold(world: &mut World, value: f32) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.restitution_threshold = clamp_float(value, 0.0, f32::MAX);
}

/// Get the the restitution speed threshold. Usually in meters per second.
/// (b2World_GetRestitutionThreshold)
pub fn world_get_restitution_threshold(world: &World) -> f32 {
    world.restitution_threshold
}

/// Adjust the hit event threshold. This controls the collision speed needed
/// to generate a b2ContactHitEvent. Usually in meters per second.
/// (b2World_SetHitEventThreshold)
pub fn world_set_hit_event_threshold(world: &mut World, value: f32) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.hit_event_threshold = clamp_float(value, 0.0, f32::MAX);
}

/// Get the the hit event speed threshold. Usually in meters per second.
/// (b2World_GetHitEventThreshold)
pub fn world_get_hit_event_threshold(world: &World) -> f32 {
    world.hit_event_threshold
}

/// Adjust contact tuning parameters: hertz, damping ratio, and push speed.
/// Advanced feature for testing. (b2World_SetContactTuning)
pub fn world_set_contact_tuning(
    world: &mut World,
    hertz: f32,
    damping_ratio: f32,
    push_speed: f32,
) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.contact_hertz = clamp_float(hertz, 0.0, f32::MAX);
    world.contact_damping_ratio = clamp_float(damping_ratio, 0.0, f32::MAX);
    world.contact_speed = clamp_float(push_speed, 0.0, f32::MAX);
}

/// Set the contact recycle distance. Contacts this close to a recently
/// destroyed contact reuse the old impulses for warm starting.
/// (b2World_SetContactRecycleDistance)
pub fn world_set_contact_recycle_distance(world: &mut World, recycle_distance: f32) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.contact_recycle_distance = clamp_float(recycle_distance, 0.0, f32::MAX);
}

/// Get the contact recycle distance. (b2World_GetContactRecycleDistance)
pub fn world_get_contact_recycle_distance(world: &World) -> f32 {
    world.contact_recycle_distance
}

/// Set the maximum linear speed. Usually in m/s.
/// (b2World_SetMaximumLinearSpeed)
pub fn world_set_maximum_linear_speed(world: &mut World, maximum_linear_speed: f32) {
    debug_assert!(is_valid_float(maximum_linear_speed) && maximum_linear_speed > 0.0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.max_linear_speed = maximum_linear_speed;
}

/// Get the maximum linear speed. Usually in m/s.
/// (b2World_GetMaximumLinearSpeed)
pub fn world_get_maximum_linear_speed(world: &World) -> f32 {
    world.max_linear_speed
}

/// Get the current world performance profile. (b2World_GetProfile)
pub fn world_get_profile(world: &World) -> Profile {
    world.profile
}

/// Get world counters and sizes. (b2World_GetCounters)
pub fn world_get_counters(world: &World) -> Counters {
    let mut s = Counters {
        body_count: world.body_id_pool.id_count(),
        shape_count: world.shape_id_pool.id_count(),
        contact_count: world.contact_id_pool.id_count(),
        joint_count: world.joint_id_pool.id_count(),
        island_count: world.island_id_pool.id_count(),
        ..Default::default()
    };

    let static_tree = &world.broad_phase.trees[BodyType::Static as usize];
    s.static_tree_height = static_tree.height();

    let dynamic_tree = &world.broad_phase.trees[BodyType::Dynamic as usize];
    let kinematic_tree = &world.broad_phase.trees[BodyType::Kinematic as usize];
    s.tree_height = max_int(dynamic_tree.height(), kinematic_tree.height());

    // stack_used, byte_count, and task_count stay zero: no arena allocator,
    // no global allocation tracking, no task system in the serial port.

    for i in 0..world.worker_count as usize {
        s.recycled_contact_count += world.task_contexts[i].recycled_contact_count;
    }

    for i in 0..GRAPH_COLOR_COUNT as usize {
        let color = &world.constraint_graph.colors[i];
        s.color_counts[i] = (color.contact_sims.len() + color.joint_sims.len()) as i32;
        s.awake_contact_count += color.contact_sims.len() as i32;
    }
    s.awake_contact_count += world.solver_sets[AWAKE_SET as usize].contact_sims.len() as i32;

    s
}

/// Get the maximum capacity the world has reached. (b2World_GetMaxCapacity)
pub fn world_get_max_capacity(world: &World) -> Capacity {
    world.max_capacity
}

/// Set the user data pointer. (b2World_SetUserData)
pub fn world_set_user_data(world: &mut World, user_data: u64) {
    world.user_data = user_data;
}

/// Get the user data pointer. (b2World_GetUserData)
pub fn world_get_user_data(world: &World) -> u64 {
    world.user_data
}

/// Register the friction callback. This is optional. Passing `None` restores
/// the default mixing rule. (b2World_SetFrictionCallback)
pub fn world_set_friction_callback(world: &mut World, callback: Option<FrictionCallback>) {
    if world.locked {
        return;
    }

    world.friction_callback = Some(callback.unwrap_or(default_friction_callback));
}

/// Register the restitution callback. This is optional. Passing `None`
/// restores the default mixing rule. (b2World_SetRestitutionCallback)
pub fn world_set_restitution_callback(world: &mut World, callback: Option<RestitutionCallback>) {
    if world.locked {
        return;
    }

    world.restitution_callback = Some(callback.unwrap_or(default_restitution_callback));
}

/// Register the custom filter callback. This is optional.
/// (b2World_SetCustomFilterCallback)
pub fn world_set_custom_filter_callback(
    world: &mut World,
    fcn: Option<CustomFilterFcn>,
    context: u64,
) {
    world.custom_filter_fcn = fcn;
    world.custom_filter_context = context;
}

/// Register the pre-solve callback. This is optional.
/// (b2World_SetPreSolveCallback)
pub fn world_set_pre_solve_callback(world: &mut World, fcn: Option<PreSolveFcn>, context: u64) {
    world.pre_solve_fcn = fcn;
    world.pre_solve_context = context;
}

/// Set the gravity vector for the entire world. Box2D has no concept of an up
/// direction and this is left as a decision for the application.
/// (b2World_SetGravity)
pub fn world_set_gravity(world: &mut World, gravity: Vec2) {
    world.gravity = gravity;
}

/// Get the gravity vector. (b2World_GetGravity)
pub fn world_get_gravity(world: &World) -> Vec2 {
    world.gravity
}

/// Apply a radial explosion. Explosions are modeled as a force, not as a
/// collision event. (b2World_Explode + static ExplosionCallback)
pub fn world_explode(world: &mut World, explosion_def: &ExplosionDef) {
    let mask_bits = explosion_def.mask_bits;
    let position = explosion_def.position;
    let radius = explosion_def.radius;
    let falloff = explosion_def.falloff;
    let impulse_per_length = explosion_def.impulse_per_length;

    debug_assert!(is_valid_position(position));
    debug_assert!(is_valid_float(radius) && radius >= 0.0);
    debug_assert!(is_valid_float(falloff) && falloff >= 0.0);
    debug_assert!(is_valid_float(impulse_per_length));

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    // The broad-phase tree is float, so translate a local query box out to
    // world with outward rounding
    let extent = radius + falloff;
    let local_box = Aabb {
        lower_bound: Vec2 {
            x: -extent,
            y: -extent,
        },
        upper_bound: Vec2 {
            x: extent,
            y: extent,
        },
    };
    let aabb = offset_aabb(local_box, position);

    // C applies impulses inside the tree traversal (ExplosionCallback), but
    // waking a body needs &mut World while the tree is borrowed. Collect the
    // candidate shape ids in traversal order first, then apply; waking a body
    // moves its sim between solver sets without touching any transform or
    // broad-phase proxy, so the two-phase form is behaviorally identical.
    let mut shape_ids: Vec<i32> = Vec::new();
    world.broad_phase.trees[BodyType::Dynamic as usize].query(aabb, mask_bits, |_, user_data| {
        shape_ids.push(user_data as i32);
        true
    });

    for shape_id in shape_ids {
        explosion_callback(
            world,
            shape_id,
            position,
            radius,
            falloff,
            impulse_per_length,
        );
    }
}

/// Apply the explosion impulse to one candidate shape.
/// (static ExplosionCallback)
fn explosion_callback(
    world: &mut World,
    shape_id: i32,
    position: Pos,
    radius: f32,
    falloff: f32,
    impulse_per_length: f32,
) {
    let shape = &world.shapes[shape_id as usize];
    let body_id = shape.body_id;
    let body = &world.bodies[body_id as usize];
    debug_assert!(body.type_ == BodyType::Dynamic);

    let xf = get_body_transform_quick(world, body);

    // Re-center the explosion into the shape local frame so distance and
    // direction stay precise far from the origin. Everything below runs in
    // that near-origin frame.
    let local_position = inv_transform_world_point(xf, position);

    let input = DistanceInput {
        proxy_a: make_shape_distance_proxy(shape),
        proxy_b: make_proxy(&[local_position], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: true,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    if output.distance > radius + falloff {
        return;
    }

    // All shape-derived values are computed before waking the body; C computes
    // them after b2WakeBody, but waking does not modify the shape or transform.
    let mut closest_point = output.point_a;
    if output.distance == 0.0 {
        closest_point = get_shape_centroid(shape);
    }

    let mut direction = sub(closest_point, local_position);
    if length_squared(direction) > 100.0 * f32::EPSILON * f32::EPSILON {
        direction = normalize(direction);
    } else {
        direction = Vec2 { x: 1.0, y: 0.0 };
    }

    let local_line = left_perp(direction);
    let perimeter = get_shape_projected_perimeter(shape, local_line);
    let mut scale = 1.0;
    if output.distance > radius && falloff > 0.0 {
        scale = clamp_float((radius + falloff - output.distance) / falloff, 0.0, 1.0);
    }

    let magnitude = impulse_per_length * perimeter * scale;
    let impulse = mul_sv(magnitude, rotate_vector(xf.q, direction));

    wake_body(world, body_id);

    let body = &world.bodies[body_id as usize];
    if body.set_index != AWAKE_SET {
        return;
    }

    let local_index = body.local_index;
    let set = &mut world.solver_sets[AWAKE_SET as usize];
    let (inv_mass, inv_inertia, local_center) = {
        let body_sim = &set.body_sims[local_index as usize];
        (
            body_sim.inv_mass,
            body_sim.inv_inertia,
            body_sim.local_center,
        )
    };
    let state = &mut set.body_states[local_index as usize];
    state.linear_velocity = mul_add(state.linear_velocity, inv_mass, impulse);

    // Lever arm from the center of mass to the closest point, rotated to world
    let r = rotate_vector(xf.q, sub(closest_point, local_center));
    state.angular_velocity += inv_inertia * cross(r, impulse);
}

/// This is for internal testing. (b2World_RebuildStaticTree)
pub fn world_rebuild_static_tree(world: &mut World) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.broad_phase.trees[BodyType::Static as usize].rebuild(true);
}

/// This is for internal testing. (b2World_EnableSpeculative)
pub fn world_enable_speculative(world: &mut World, flag: bool) {
    world.enable_speculative = flag;
}
