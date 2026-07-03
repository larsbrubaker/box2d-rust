// Port of the constraint graph data model from
// box2d-cpp-reference/src/constraint_graph.h. Logic from constraint_graph.c
// lands in the solver bring-up commit.
//
// The C b2GraphColor carries a transient union of raw pointers into arena
// scratch memory (wideConstraints/overflowConstraints), rebuilt every step by
// the solver. The Rust solver phase owns that scratch as Vecs local to the
// step; the persistent graph state here is the bitset and the sim arrays.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::constants::GRAPH_COLOR_COUNT;
use crate::contact::ContactSim;
use crate::joint::JointSim;

/// This holds constraints that cannot fit the graph color limit. This happens
/// when a single dynamic body is touching many other bodies. (B2_OVERFLOW_INDEX)
pub const OVERFLOW_INDEX: i32 = GRAPH_COLOR_COUNT - 1;

/// This keeps constraints involving two dynamic bodies at a lower solver
/// priority than constraints involving a dynamic and static bodies. This
/// reduces tunneling due to push through. (B2_DYNAMIC_COLOR_COUNT)
pub const DYNAMIC_COLOR_COUNT: i32 = GRAPH_COLOR_COUNT - 4;

/// (b2GraphColor)
#[derive(Debug, Clone, Default)]
pub struct GraphColor {
    /// This bitset is indexed by bodyId so it is over-sized to encompass
    /// static bodies; the bits are never traversed or counted. Unused on the
    /// overflow color.
    pub body_set: BitSet,

    /// cache friendly arrays
    pub contact_sims: Vec<ContactSim>,
    pub joint_sims: Vec<JointSim>,
}

/// (b2ConstraintGraph)
#[derive(Debug, Clone, Default)]
pub struct ConstraintGraph {
    /// including overflow at the end
    pub colors: Vec<GraphColor>,
}
