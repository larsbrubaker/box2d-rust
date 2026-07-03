// Port of the creation/definition types from box2d-cpp-reference/include/box2d/types.h
// and the b2Default* constructors from src/types.c.
//
// Split to satisfy the 800-line file limit:
// - world.rs — WorldDef, Capacity, RayResult, mixing-callback aliases
// - body.rs  — BodyType, MotionLocks, BodyDef
// - shape.rs — Filter, QueryFilter, SurfaceMaterial, ShapeDef, ChainDef
//
// Porting decisions applied throughout:
// - C `void* userData` becomes `u64` (Box2D only stores and returns it, never
//   dereferences it; this matches the dynamic tree's user_data and stays safe).
// - Optional mixing callbacks become `Option<fn(...)>`.
// - The multithreading task-system fields (b2EnqueueTaskCallback,
//   b2FinishTaskCallback, userTaskContext) are deferred to the scheduler phase;
//   the port runs single-threaded, which preserves determinism. `worker_count`
//   is kept.
// - `internalValue` (the B2_SECRET_COOKIE validity cookie) is kept as
//   `internal_value` for fidelity with the world-creation validation.
//
// The event, profile, counters, joint-def, hex-color, and debug-draw types from
// types.h are ported in their respective later phases, not here.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

mod body;
mod shape;
mod world;

pub use body::{default_body_def, BodyDef, BodyType, MotionLocks, BODY_TYPE_COUNT};
pub use shape::{
    default_chain_def, default_filter, default_query_filter, default_shape_def,
    default_surface_material, ChainDef, Filter, QueryFilter, ShapeDef, SurfaceMaterial,
};
pub use world::{
    default_world_def, Capacity, FrictionCallback, RayResult, RestitutionCallback, WorldDef,
};

// types.h: B2_DEFAULT_CATEGORY_BITS / B2_DEFAULT_MASK_BITS. These are the same
// constants used by the dynamic tree; re-exported here from their canonical home.
pub use crate::dynamic_tree::{DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS};
