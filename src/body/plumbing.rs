// Body storage plumbing from body.c: accessors, island glue, sim removal.
//
// Borrow strategy: C returns interior pointers (b2GetBodySim). Rust callers
// pass ids; accessors either return copies of small data (transforms) or
// borrow through &/&mut World for the duration of one access.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{body_flags, Body, BodySim, BodyState};
use crate::core::NULL_INDEX;
use crate::id::BodyId;
use crate::island::{create_island, destroy_island, validate_island};
use crate::math_functions::WorldTransform;
use crate::math_functions::{length_squared, mul_sv};
use crate::solver_set::{AWAKE_SET, DISABLED_SET};
use crate::world::World;

/// (static b2LimitVelocity)
// bring-up: called by the solver slice.
#[allow(dead_code)]
pub(crate) fn limit_velocity(state: &mut BodyState, max_linear_speed: f32) {
    let v2 = length_squared(state.linear_velocity);
    if v2 > max_linear_speed * max_linear_speed {
        state.linear_velocity = mul_sv(max_linear_speed / v2.sqrt(), state.linear_velocity);
    }
}

/// Remove a body sim from a set with swap-removal, fixing the moved body's
/// local index. (b2RemoveBodySim)
pub fn remove_body_sim(body_sims: &mut Vec<BodySim>, bodies: &mut [Body], local_index: i32) {
    debug_assert!(0 <= local_index && (local_index as usize) < body_sims.len());
    let last_index = body_sims.len() - 1;
    body_sims.swap_remove(local_index as usize);
    if (local_index as usize) < body_sims.len() {
        let moved_body = &mut bodies[body_sims[local_index as usize].body_id as usize];
        debug_assert!(moved_body.local_index == last_index as i32);
        moved_body.local_index = local_index;
    }
}

/// Get a validated body index from an id. (b2GetBodyFullId — C returns a
/// pointer; Rust returns the raw index into `world.bodies`)
pub fn get_body_full_id(world: &World, body_id: BodyId) -> i32 {
    debug_assert!(body_id.index1 >= 1);
    let index = body_id.index1 - 1;
    debug_assert!((index as usize) < world.bodies.len());
    debug_assert!(world.bodies[index as usize].generation == body_id.generation);
    // id index starts at one so that zero can represent null
    index
}

/// (b2GetBodyTransformQuick)
pub fn get_body_transform_quick(world: &World, body: &Body) -> WorldTransform {
    let set = &world.solver_sets[body.set_index as usize];
    set.body_sims[body.local_index as usize].transform
}

/// (b2GetBodyTransform)
pub fn get_body_transform(world: &World, body_id: i32) -> WorldTransform {
    let body = &world.bodies[body_id as usize];
    get_body_transform_quick(world, body)
}

/// Create a BodyId from a raw id. (b2MakeBodyId)
pub fn make_body_id(world: &World, body_id: i32) -> BodyId {
    let body = &world.bodies[body_id as usize];
    BodyId {
        index1: body_id + 1,
        world0: world.world_id,
        generation: body.generation,
    }
}

/// Location of a body's sim data: (set_index, local_index). Use to borrow the
/// BodySim through `world.solver_sets`. (b2GetBodySim resolves to a pointer in
/// C; the index pair is the borrow-safe equivalent.)
pub fn body_sim_location(world: &World, body_id: i32) -> (i32, i32) {
    let body = &world.bodies[body_id as usize];
    (body.set_index, body.local_index)
}

/// Borrow a body's sim data mutably. (b2GetBodySim)
pub fn get_body_sim<'a>(world: &'a mut World, body: &Body) -> &'a mut BodySim {
    let set = &mut world.solver_sets[body.set_index as usize];
    &mut set.body_sims[body.local_index as usize]
}

/// Borrow a body's state if it is in the awake set. (b2GetBodyState)
pub fn get_body_state<'a>(world: &'a mut World, body: &Body) -> Option<&'a mut BodyState> {
    if body.set_index == AWAKE_SET {
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        return Some(&mut set.body_states[body.local_index as usize]);
    }

    None
}

/// (b2SyncBodyFlags)
pub fn sync_body_flags(world: &mut World, body_id: i32) {
    let body = &world.bodies[body_id as usize];
    // Never sync transient flags
    let flags = body.flags & !body_flags::BODY_TRANSIENT_FLAGS;
    let (set_index, local_index) = (body.set_index, body.local_index);

    let set = &mut world.solver_sets[set_index as usize];
    set.body_sims[local_index as usize].flags = flags;

    if set_index == AWAKE_SET {
        set.body_states[local_index as usize].flags = flags;
    }
}

/// (static b2CreateIslandForBody)
pub(crate) fn create_island_for_body(world: &mut World, set_index: i32, body_id: i32) {
    debug_assert!(world.bodies[body_id as usize].island_id == NULL_INDEX);
    debug_assert!(set_index != DISABLED_SET);

    let island_id = create_island(world, set_index);
    world.islands[island_id as usize].bodies.push(body_id);
    let body = &mut world.bodies[body_id as usize];
    body.island_id = island_id;
    body.island_index = 0;

    validate_island(world, island_id);
}

/// (static b2RemoveBodyFromIsland)
// bring-up: called by destroy_body when the joint/contact slices land.
#[allow(dead_code)]
pub(crate) fn remove_body_from_island(world: &mut World, body_id: i32) {
    let (island_id, island_index) = {
        let body = &world.bodies[body_id as usize];
        (body.island_id, body.island_index)
    };
    if island_id == NULL_INDEX {
        debug_assert!(island_index == NULL_INDEX);
        return;
    }

    {
        let local_index = island_index;
        let last = world.islands[island_id as usize].bodies.len() - 1;
        let moved_body_id = world.islands[island_id as usize].bodies[last];
        world.islands[island_id as usize].bodies[local_index as usize] = moved_body_id;
        debug_assert!(world.bodies[moved_body_id as usize].island_index == last as i32);
        world.bodies[moved_body_id as usize].island_index = local_index;
        world.islands[island_id as usize].bodies.pop();
    }

    if world.islands[island_id as usize].bodies.is_empty() {
        // Destroy empty island
        debug_assert!(world.islands[island_id as usize].contacts.is_empty());
        debug_assert!(world.islands[island_id as usize].joints.is_empty());

        // Free the island
        destroy_island(world, island_id);
    } else {
        validate_island(world, island_id);
    }

    let body = &mut world.bodies[body_id as usize];
    body.island_id = NULL_INDEX;
    body.island_index = NULL_INDEX;
}
