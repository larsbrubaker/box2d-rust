// Port of the body data model from box2d-cpp-reference/src/body.h.
// Logic from body.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::distance::Sweep;
use crate::math_functions::{
    sub_pos, Pos, Rot, Vec2, WorldTransform, POS_ZERO, ROT_IDENTITY, VEC2_ZERO,
    WORLD_TRANSFORM_IDENTITY,
};
use crate::types::BodyType;

// enum b2BodyFlags
pub mod body_flags {
    /// This body has fixed translation along the x-axis
    pub const LOCK_LINEAR_X: u32 = 0x00000001;
    /// This body has fixed translation along the y-axis
    pub const LOCK_LINEAR_Y: u32 = 0x00000002;
    /// This body has fixed rotation
    pub const LOCK_ANGULAR_Z: u32 = 0x00000004;
    /// This flag is used for debug draw
    pub const IS_FAST: u32 = 0x00000008;
    /// This dynamic body does a final CCD pass against all body types, but not other bullets
    pub const IS_BULLET: u32 = 0x00000010;
    /// This body was speed capped in the current time step
    pub const IS_SPEED_CAPPED: u32 = 0x00000020;
    /// This body had a time of impact event in the current time step
    pub const HAD_TIME_OF_IMPACT: u32 = 0x00000040;
    /// This body has no limit on angular velocity
    pub const ALLOW_FAST_ROTATION: u32 = 0x00000080;
    /// This body needs to have its AABB increased
    pub const ENLARGE_BOUNDS: u32 = 0x00000100;
    /// This body is dynamic so the solver should write to it.
    pub const DYNAMIC_FLAG: u32 = 0x00000200;
    /// The user deferred mass computation but b2Body_ApplyMassFromShapes was
    /// not called before the world step.
    pub const DIRTY_MASS: u32 = 0x00000400;
    pub const ENABLE_SLEEP: u32 = 0x00000800;
    pub const BODY_ENABLE_CONTACT_RECYCLING: u32 = 0x00001000;

    /// All lock flags
    pub const ALL_LOCKS: u32 = LOCK_ANGULAR_Z | LOCK_LINEAR_X | LOCK_LINEAR_Y;
    /// If this flag is set then the body has fixed rotation
    pub const FIXED_ROTATION: u32 = LOCK_ANGULAR_Z;
    /// These flags are transient per time step. These may be different across
    /// Body, BodySim, and BodyState.
    pub const BODY_TRANSIENT_FLAGS: u32 = IS_FAST | IS_SPEED_CAPPED | HAD_TIME_OF_IMPACT;
}

/// Body organizational details that are not used in the solver. (b2Body)
#[derive(Debug, Clone)]
pub struct Body {
    pub user_data: u64,

    /// index of solver set stored in World. May be NULL_INDEX.
    pub set_index: i32,

    /// body sim and state index within set. May be NULL_INDEX.
    pub local_index: i32,

    /// [31 : contactId | 1 : edgeIndex]
    pub head_contact_key: i32,
    pub contact_count: i32,

    pub head_shape_id: i32,
    pub shape_count: i32,

    pub head_chain_id: i32,

    /// [31 : jointId | 1 : edgeIndex]
    pub head_joint_key: i32,
    pub joint_count: i32,

    /// All enabled dynamic and kinematic bodies are in an island.
    pub island_id: i32,

    /// Need this island index for faster union-find
    pub island_index: i32,

    pub mass: f32,

    /// Rotational inertia about the center of mass.
    pub inertia: f32,

    pub sleep_threshold: f32,
    pub sleep_time: f32,

    /// this is used to adjust the fellAsleep flag in the body move array
    pub body_move_index: i32,

    pub id: i32,

    /// body_flags bits
    pub flags: u32,

    pub type_: BodyType,

    /// Monotonically advanced when a body is allocated in this slot.
    /// Used to check for invalid BodyId.
    pub generation: u16,

    /// Body name for debugging (C: char[B2_NAME_LENGTH + 1]).
    pub name: String,
}

impl Default for Body {
    fn default() -> Self {
        Body {
            user_data: 0,
            set_index: NULL_INDEX,
            local_index: NULL_INDEX,
            head_contact_key: NULL_INDEX,
            contact_count: 0,
            head_shape_id: NULL_INDEX,
            shape_count: 0,
            head_chain_id: NULL_INDEX,
            head_joint_key: NULL_INDEX,
            joint_count: 0,
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            mass: 0.0,
            inertia: 0.0,
            sleep_threshold: 0.0,
            sleep_time: 0.0,
            body_move_index: NULL_INDEX,
            id: NULL_INDEX,
            flags: 0,
            type_: BodyType::Static,
            generation: 0,
            name: String::new(),
        }
    }
}

/// Body state, designed for fast conversion to and from SIMD via
/// scatter-gather. Only awake dynamic and kinematic bodies have a body state.
/// Used in the performance critical constraint solver. (b2BodyState, 32 bytes)
///
/// The solver operates on the body state. The body state array does not hold
/// static bodies; their delta rotation is identity via a dummy state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
    /// body_flags bits. Important flags: locking, dynamic.
    pub flags: u32,
    /// Using delta position reduces round-off error far from the origin
    pub delta_position: Vec2,
    /// Using delta rotation because the solver cannot access the full rotation
    /// on static bodies and must use zero delta rotation (c,s) = (1,0)
    pub delta_rotation: Rot,
}

/// Identity body state, notice the delta_rotation is {1, 0}.
/// (b2_identityBodyState)
pub const IDENTITY_BODY_STATE: BodyState = BodyState {
    linear_velocity: VEC2_ZERO,
    angular_velocity: 0.0,
    flags: 0,
    delta_position: VEC2_ZERO,
    delta_rotation: ROT_IDENTITY,
};

impl Default for BodyState {
    fn default() -> Self {
        IDENTITY_BODY_STATE
    }
}

/// Body simulation data used for integration of position and velocity.
/// Transform data used for collision and solver preparation. (b2BodySim)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodySim {
    /// transform for body origin, double translation in large world mode
    pub transform: WorldTransform,

    /// center of mass position in world space
    pub center: Pos,

    /// previous rotation and COM for TOI
    pub rotation0: Rot,
    pub center0: Pos,

    /// location of center of mass relative to the body origin
    pub local_center: Vec2,

    pub force: Vec2,
    pub torque: f32,

    /// inverse mass and inertia
    pub inv_mass: f32,
    pub inv_inertia: f32,

    pub min_extent: f32,
    pub max_extent: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,

    /// Index of Body
    pub body_id: i32,

    /// body_flags bits
    pub flags: u32,
}

impl Default for BodySim {
    fn default() -> Self {
        BodySim {
            transform: WORLD_TRANSFORM_IDENTITY,
            center: POS_ZERO,
            rotation0: ROT_IDENTITY,
            center0: POS_ZERO,
            local_center: VEC2_ZERO,
            force: VEC2_ZERO,
            torque: 0.0,
            inv_mass: 0.0,
            inv_inertia: 0.0,
            min_extent: 0.0,
            max_extent: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 0.0,
            body_id: NULL_INDEX,
            flags: 0,
        }
    }
}

/// Build a sweep relative to a base position so continuous collision keeps
/// float precision far from the origin. The base cancels out of the relative
/// motion the TOI actually solves. (b2MakeRelativeSweep)
pub fn make_relative_sweep(body_sim: &BodySim, base: Pos) -> Sweep {
    Sweep {
        c1: sub_pos(body_sim.center0, base),
        c2: sub_pos(body_sim.center, base),
        q1: body_sim.rotation0,
        q2: body_sim.transform.q,
        local_center: body_sim.local_center,
    }
}

// ---------------------------------------------------------------------------
// Body storage plumbing and creation from body.c. Destruction and the public
// b2Body_* accessors that need joints/contacts/sensors land with those slices.
//
// Borrow strategy: C returns interior pointers (b2GetBodySim). Rust callers
// pass ids; accessors either return copies of small data (transforms) or
// borrow through &/&mut World for the duration of one access.
// ---------------------------------------------------------------------------

use crate::constants::huge;
use crate::id::BodyId;
use crate::island::{create_island, destroy_island, validate_island};
use crate::math_functions::{
    is_valid_float, is_valid_position, is_valid_rotation, is_valid_vec2, length_squared, mul_sv,
    ROT_IDENTITY as ROT_IDENTITY_,
};
use crate::solver_set::SolverSet;
use crate::solver_set::{AWAKE_SET, DISABLED_SET, STATIC_SET};
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

/// Create a rigid body given a definition. (b2CreateBody)
pub fn create_body(world: &mut World, def: &crate::types::BodyDef) -> BodyId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(is_valid_position(def.position));
    debug_assert!(is_valid_rotation(def.rotation));
    debug_assert!(is_valid_vec2(def.linear_velocity));
    debug_assert!(is_valid_float(def.angular_velocity));
    debug_assert!(is_valid_float(def.linear_damping) && def.linear_damping >= 0.0);
    debug_assert!(is_valid_float(def.angular_damping) && def.angular_damping >= 0.0);
    debug_assert!(is_valid_float(def.sleep_threshold) && def.sleep_threshold >= 0.0);
    debug_assert!(is_valid_float(def.gravity_scale));

    debug_assert!(!world.locked);
    if world.locked {
        return BodyId::default();
    }

    let is_awake = (def.is_awake || !def.enable_sleep) && def.is_enabled;

    // determine the solver set
    let set_id;
    if !def.is_enabled {
        // any body type can be disabled
        set_id = DISABLED_SET;
    } else if def.type_ == BodyType::Static {
        set_id = STATIC_SET;
    } else if is_awake {
        set_id = AWAKE_SET;
    } else {
        // new set for a sleeping body in its own island
        set_id = world.solver_set_id_pool.alloc_id();
        if set_id == world.solver_sets.len() as i32 {
            // Create a zero initialized solver set. All sub-arrays are also
            // zero initialized.
            world.solver_sets.push(SolverSet::default());
        } else {
            debug_assert!(world.solver_sets[set_id as usize].set_index == NULL_INDEX);
        }

        world.solver_sets[set_id as usize].set_index = set_id;
    }

    debug_assert!(0 <= set_id && set_id < world.solver_sets.len() as i32);

    let body_id = world.body_id_pool.alloc_id();

    let mut lock_flags = 0u32;
    lock_flags |= if def.motion_locks.linear_x {
        body_flags::LOCK_LINEAR_X
    } else {
        0
    };
    lock_flags |= if def.motion_locks.linear_y {
        body_flags::LOCK_LINEAR_Y
    } else {
        0
    };
    lock_flags |= if def.motion_locks.angular_z {
        body_flags::LOCK_ANGULAR_Z
    } else {
        0
    };

    let set = &mut world.solver_sets[set_id as usize];
    let mut body_sim = BodySim {
        transform: crate::math_functions::WorldTransform {
            p: def.position,
            q: def.rotation,
        },
        center: def.position,
        rotation0: def.rotation,
        center0: def.position,
        min_extent: huge(),
        max_extent: 0.0,
        linear_damping: def.linear_damping,
        angular_damping: def.angular_damping,
        gravity_scale: def.gravity_scale,
        body_id,
        flags: lock_flags,
        ..Default::default()
    };
    body_sim.flags |= if def.is_bullet {
        body_flags::IS_BULLET
    } else {
        0
    };
    body_sim.flags |= if def.allow_fast_rotation {
        body_flags::ALLOW_FAST_ROTATION
    } else {
        0
    };
    body_sim.flags |= if def.type_ == BodyType::Dynamic {
        body_flags::DYNAMIC_FLAG
    } else {
        0
    };
    body_sim.flags |= if def.enable_sleep {
        body_flags::ENABLE_SLEEP
    } else {
        0
    };
    body_sim.flags |= if def.enable_contact_recycling {
        body_flags::BODY_ENABLE_CONTACT_RECYCLING
    } else {
        0
    };
    let sim_flags = body_sim.flags;
    set.body_sims.push(body_sim);
    let local_index = set.body_sims.len() as i32 - 1;

    if set_id == AWAKE_SET {
        set.body_states.push(BodyState {
            linear_velocity: def.linear_velocity,
            angular_velocity: def.angular_velocity,
            delta_rotation: ROT_IDENTITY_,
            flags: sim_flags,
            ..Default::default()
        });
    }

    if body_id == world.bodies.len() as i32 {
        world.bodies.push(Body::default());
    } else {
        debug_assert!(world.bodies[body_id as usize].id == NULL_INDEX);
    }

    let body = &mut world.bodies[body_id as usize];

    // C: strncpy into char[B2_NAME_LENGTH + 1]; the Rust name is truncated to
    // the same limit.
    body.name = def
        .name
        .chars()
        .take(crate::constants::NAME_LENGTH as usize)
        .collect();

    body.user_data = def.user_data;
    body.set_index = set_id;
    body.local_index = local_index;
    body.generation += 1;
    body.head_shape_id = NULL_INDEX;
    body.shape_count = 0;
    body.head_chain_id = NULL_INDEX;
    body.head_contact_key = NULL_INDEX;
    body.contact_count = 0;
    body.head_joint_key = NULL_INDEX;
    body.joint_count = 0;
    body.island_id = NULL_INDEX;
    body.island_index = NULL_INDEX;
    body.body_move_index = NULL_INDEX;
    body.id = body_id;
    body.mass = 0.0;
    body.inertia = 0.0;
    body.sleep_threshold = def.sleep_threshold;
    body.sleep_time = 0.0;
    body.type_ = def.type_;
    body.flags = sim_flags;

    // dynamic and kinematic bodies that are enabled need an island
    if set_id >= AWAKE_SET {
        create_island_for_body(world, set_id, body_id);
    }

    world.validate_solver_sets();

    make_body_id(world, body_id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_functions::{make_rot, to_pos, Vec2};
    use crate::types::{default_body_def, default_world_def};

    #[test]
    fn create_static_dynamic_and_sleeping_bodies() {
        let mut world = World::new(&default_world_def());

        // Static body goes in the static set with no island.
        let s = create_body(&mut world, &default_body_def());
        let s_index = get_body_full_id(&world, s);
        assert_eq!(world.bodies[s_index as usize].set_index, STATIC_SET);
        assert_eq!(world.bodies[s_index as usize].island_id, NULL_INDEX);
        assert!(world.solver_sets[STATIC_SET as usize]
            .body_states
            .is_empty());

        // Dynamic awake body: awake set, body state, island.
        let mut def = default_body_def();
        def.type_ = BodyType::Dynamic;
        def.position = to_pos(Vec2 { x: 2.0, y: 3.0 });
        def.rotation = make_rot(0.5);
        def.linear_velocity = Vec2 { x: 1.0, y: -1.0 };
        def.name = "crate".into();
        let d = create_body(&mut world, &def);
        let d_index = get_body_full_id(&world, d);
        {
            let body = &world.bodies[d_index as usize];
            assert_eq!(body.set_index, AWAKE_SET);
            assert_eq!(body.type_, BodyType::Dynamic);
            assert!(body.island_id != NULL_INDEX);
            assert_eq!(body.name, "crate");
            assert!(body.flags & body_flags::DYNAMIC_FLAG != 0);
        }
        let xf = get_body_transform(&world, d_index);
        assert_eq!(crate::math_functions::to_vec2(xf.p).x, 2.0);
        let awake = &world.solver_sets[AWAKE_SET as usize];
        assert_eq!(awake.body_sims.len(), 1);
        assert_eq!(awake.body_states.len(), 1);
        assert_eq!(awake.body_states[0].linear_velocity.x, 1.0);
        assert_eq!(awake.island_sims.len(), 1);

        // Sleeping dynamic body gets its own new sleeping set + island.
        let mut sleep_def = default_body_def();
        sleep_def.type_ = BodyType::Dynamic;
        sleep_def.is_awake = false;
        let z = create_body(&mut world, &sleep_def);
        let z_index = get_body_full_id(&world, z);
        let z_set = world.bodies[z_index as usize].set_index;
        assert!(z_set >= crate::solver_set::FIRST_SLEEPING_SET);
        assert_eq!(world.solver_sets[z_set as usize].body_sims.len(), 1);
        assert!(world.solver_sets[z_set as usize].body_states.is_empty());
        assert!(world.bodies[z_index as usize].island_id != NULL_INDEX);

        // Disabled body: disabled set, no island.
        let mut disabled_def = default_body_def();
        disabled_def.type_ = BodyType::Dynamic;
        disabled_def.is_enabled = false;
        let x = create_body(&mut world, &disabled_def);
        let x_index = get_body_full_id(&world, x);
        assert_eq!(world.bodies[x_index as usize].set_index, DISABLED_SET);
        assert_eq!(world.bodies[x_index as usize].island_id, NULL_INDEX);

        // Ids are 1-based with generations.
        assert_eq!(s.index1, 1);
        assert_eq!(d.index1, 2);
        assert!(d.generation >= 1);

        world.validate_solver_sets();
    }

    #[test]
    fn island_lifecycle_via_bodies() {
        let mut world = World::new(&default_world_def());

        let mut def = default_body_def();
        def.type_ = BodyType::Dynamic;
        let a = create_body(&mut world, &def);
        let b = create_body(&mut world, &def);
        let a_index = get_body_full_id(&world, a);
        let b_index = get_body_full_id(&world, b);

        assert_eq!(world.island_id_pool.id_count(), 2);

        // Remove body A from its island; the island becomes empty and dies.
        let a_island = world.bodies[a_index as usize].island_id;
        remove_body_from_island(&mut world, a_index);
        assert_eq!(world.bodies[a_index as usize].island_id, NULL_INDEX);
        assert_eq!(world.islands[a_island as usize].set_index, NULL_INDEX);
        assert_eq!(world.island_id_pool.id_count(), 1);

        // B's island is intact.
        crate::island::validate_island(&world, world.bodies[b_index as usize].island_id);

        // Name truncation to NAME_LENGTH.
        let mut long_name = default_body_def();
        long_name.name = "abcdefghijklmnop".into();
        let c = create_body(&mut world, &long_name);
        let c_index = get_body_full_id(&world, c);
        assert_eq!(
            world.bodies[c_index as usize].name.len(),
            crate::constants::NAME_LENGTH as usize
        );
    }
}
