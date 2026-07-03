// Port of the solver set data model from box2d-cpp-reference/src/solver_set.h.
// Logic from solver_set.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{BodySim, BodyState};
use crate::contact::ContactSim;
use crate::island::IslandSim;
use crate::joint::JointSim;

// The solver set type by index (enum b2SolverSetType)

/// Static set for static bodies and joints between static bodies.
pub const STATIC_SET: i32 = 0;
/// Disabled set for disabled bodies and their joints.
pub const DISABLED_SET: i32 = 1;
/// Awake set for awake bodies and awake non-touching contacts. Awake touching
/// contacts and awake joints live in the constraint graph.
pub const AWAKE_SET: i32 = 2;
/// The index of the first sleeping set. Each island that goes to sleep is put
/// into a sleeping set holding all bodies, contacts, and joints from that
/// island, making it very efficient to wake a single island.
pub const FIRST_SLEEPING_SET: i32 = 3;

/// This holds solver set data. The following sets are used:
/// - static set for all static bodies and joints between static bodies
/// - active set for all active bodies with body states (no contacts or joints)
/// - disabled set for disabled bodies and their joints
/// - all further sets are sleeping island sets along with contacts and joints
///
/// The purpose of solver sets is to achieve high memory locality.
/// <https://www.youtube.com/watch?v=nZNd5FjSquk> (b2SolverSet)
#[derive(Debug, Clone, Default)]
pub struct SolverSet {
    /// Body array. Empty for unused set.
    pub body_sims: Vec<BodySim>,

    /// Body state only exists for active set
    pub body_states: Vec<BodyState>,

    /// This holds sleeping/disabled joints. Empty for static/active set.
    pub joint_sims: Vec<JointSim>,

    /// This holds all contacts for sleeping sets.
    /// This holds non-touching contacts for the awake set.
    pub contact_sims: Vec<ContactSim>,

    /// The awake set has an array of islands. Sleeping sets normally have a
    /// single island; joints created between sleeping sets cause sets to merge,
    /// leaving them with multiple islands that merge naturally when woken.
    /// The static and disabled sets have no islands.
    pub island_sims: Vec<IslandSim>,

    /// Aligns with World::solver_set_id_pool. Used to create a stable id for
    /// body/contact/joint/islands.
    pub set_index: i32,
}
