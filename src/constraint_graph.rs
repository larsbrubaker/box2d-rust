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

use crate::math_functions::max_int;
use crate::solver_set::AWAKE_SET;
use crate::types::{BodyType, Capacity};
use crate::world::World;

impl ConstraintGraph {
    /// (b2CreateGraph)
    pub fn new(capacity: &Capacity) -> ConstraintGraph {
        const _: () = assert!(GRAPH_COLOR_COUNT >= 2, "must have at least two colors");
        const _: () = assert!(DYNAMIC_COLOR_COUNT >= 2, "need more dynamic colors");

        let body_capacity = max_int(capacity.static_body_count + capacity.dynamic_body_count, 16);

        let mut colors = Vec::with_capacity(GRAPH_COLOR_COUNT as usize);
        // Initialize graph color bit set. No bitset for overflow color.
        for i in 0..GRAPH_COLOR_COUNT {
            let mut color = GraphColor::default();
            if i < OVERFLOW_INDEX {
                color.body_set = BitSet::new(body_capacity as u32);
                color.body_set.set_bit_count_and_clear(body_capacity as u32);
                color.contact_sims.reserve(16);
            }
            colors.push(color);
        }

        ConstraintGraph { colors }
    }
}

/// Contacts are always created as non-touching. They get moved into the
/// constraint graph once they are found to be touching. (b2AddContactToGraph)
pub fn add_contact_to_graph(world: &mut World, contact_sim: ContactSim, contact_id: i32) {
    use crate::contact::contact_flags::{SIM_TOUCHING, TOUCHING};

    debug_assert!(contact_sim.manifold.point_count > 0);
    debug_assert!(contact_sim.sim_flags & SIM_TOUCHING != 0);
    debug_assert!(world.contacts[contact_id as usize].flags & TOUCHING != 0);

    let mut color_index = OVERFLOW_INDEX;

    let (body_id_a, body_id_b) = {
        let contact = &world.contacts[contact_id as usize];
        (contact.edges[0].body_id, contact.edges[1].body_id)
    };
    let type_a = world.bodies[body_id_a as usize].type_;
    let type_b = world.bodies[body_id_b as usize].type_;
    debug_assert!(type_a == BodyType::Dynamic || type_b == BodyType::Dynamic);

    // (B2_FORCE_OVERFLOW == 0 path)
    if type_a == BodyType::Dynamic && type_b == BodyType::Dynamic {
        // Dynamic constraint colors cannot encroach on colors reserved for
        // static constraints
        for i in 0..DYNAMIC_COLOR_COUNT {
            let color = &mut world.constraint_graph.colors[i as usize];
            if color.body_set.get_bit(body_id_a as u32) || color.body_set.get_bit(body_id_b as u32)
            {
                continue;
            }

            color.body_set.set_bit_grow(body_id_a as u32);
            color.body_set.set_bit_grow(body_id_b as u32);
            color_index = i;
            break;
        }
    } else if type_a == BodyType::Dynamic {
        // Static constraint colors build from the end to get higher priority
        // than dyn-dyn constraints
        let mut i = OVERFLOW_INDEX - 1;
        while i >= 1 {
            let color = &mut world.constraint_graph.colors[i as usize];
            if !color.body_set.get_bit(body_id_a as u32) {
                color.body_set.set_bit_grow(body_id_a as u32);
                color_index = i;
                break;
            }
            i -= 1;
        }
    } else if type_b == BodyType::Dynamic {
        let mut i = OVERFLOW_INDEX - 1;
        while i >= 1 {
            let color = &mut world.constraint_graph.colors[i as usize];
            if !color.body_set.get_bit(body_id_b as u32) {
                color.body_set.set_bit_grow(body_id_b as u32);
                color_index = i;
                break;
            }
            i -= 1;
        }
    }

    let local_index = world.constraint_graph.colors[color_index as usize]
        .contact_sims
        .len() as i32;
    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.color_index = color_index;
        contact.local_index = local_index;
    }

    let mut new_contact = contact_sim;

    if type_a == BodyType::Static {
        new_contact.body_sim_index_a = crate::core::NULL_INDEX;
        new_contact.inv_mass_a = 0.0;
        new_contact.inv_i_a = 0.0;
    } else {
        debug_assert!(world.bodies[body_id_a as usize].set_index == AWAKE_SET);
        let local = world.bodies[body_id_a as usize].local_index;
        new_contact.body_sim_index_a = local;

        let body_sim_a = &world.solver_sets[AWAKE_SET as usize].body_sims[local as usize];
        new_contact.inv_mass_a = body_sim_a.inv_mass;
        new_contact.inv_i_a = body_sim_a.inv_inertia;
    }

    if type_b == BodyType::Static {
        new_contact.body_sim_index_b = crate::core::NULL_INDEX;
        new_contact.inv_mass_b = 0.0;
        new_contact.inv_i_b = 0.0;
    } else {
        debug_assert!(world.bodies[body_id_b as usize].set_index == AWAKE_SET);
        let local = world.bodies[body_id_b as usize].local_index;
        new_contact.body_sim_index_b = local;

        let body_sim_b = &world.solver_sets[AWAKE_SET as usize].body_sims[local as usize];
        new_contact.inv_mass_b = body_sim_b.inv_mass;
        new_contact.inv_i_b = body_sim_b.inv_inertia;
    }

    world.constraint_graph.colors[color_index as usize]
        .contact_sims
        .push(new_contact);
}

/// (b2RemoveContactFromGraph)
pub fn remove_contact_from_graph(
    world: &mut World,
    body_id_a: i32,
    body_id_b: i32,
    color_index: i32,
    local_index: i32,
) {
    debug_assert!((0..GRAPH_COLOR_COUNT).contains(&color_index));
    let color = &mut world.constraint_graph.colors[color_index as usize];

    if color_index != OVERFLOW_INDEX {
        // This might clear a bit for a kinematic or static body, but this has
        // no effect
        color.body_set.clear_bit(body_id_a as u32);
        color.body_set.clear_bit(body_id_b as u32);
    }

    let moved_index = color.contact_sims.len() as i32 - 1;
    color.contact_sims.swap_remove(local_index as usize);
    if moved_index != local_index {
        // Fix moved contact
        let moved_id = color.contact_sims[local_index as usize].contact_id;
        let moved_contact = &mut world.contacts[moved_id as usize];
        debug_assert!(moved_contact.set_index == AWAKE_SET);
        debug_assert!(moved_contact.color_index == color_index);
        debug_assert!(moved_contact.local_index == moved_index);
        moved_contact.local_index = local_index;
    }
}

/// Notice that a joint cannot share the same color as a contact between the
/// same two bodies, so contacts and joints can be solved in parallel within
/// each color. (static b2AssignJointColor)
fn assign_joint_color(
    graph: &mut ConstraintGraph,
    body_id_a: i32,
    body_id_b: i32,
    type_a: BodyType,
    type_b: BodyType,
) -> i32 {
    debug_assert!(type_a == BodyType::Dynamic || type_b == BodyType::Dynamic);

    if type_a == BodyType::Dynamic && type_b == BodyType::Dynamic {
        for i in 0..DYNAMIC_COLOR_COUNT {
            let color = &mut graph.colors[i as usize];
            if color.body_set.get_bit(body_id_a as u32) || color.body_set.get_bit(body_id_b as u32)
            {
                continue;
            }

            color.body_set.set_bit_grow(body_id_a as u32);
            color.body_set.set_bit_grow(body_id_b as u32);
            return i;
        }
    } else if type_a == BodyType::Dynamic {
        let mut i = OVERFLOW_INDEX - 1;
        while i >= 1 {
            let color = &mut graph.colors[i as usize];
            if !color.body_set.get_bit(body_id_a as u32) {
                color.body_set.set_bit_grow(body_id_a as u32);
                return i;
            }
            i -= 1;
        }
    } else if type_b == BodyType::Dynamic {
        let mut i = OVERFLOW_INDEX - 1;
        while i >= 1 {
            let color = &mut graph.colors[i as usize];
            if !color.body_set.get_bit(body_id_b as u32) {
                color.body_set.set_bit_grow(body_id_b as u32);
                return i;
            }
            i -= 1;
        }
    }

    OVERFLOW_INDEX
}

/// Assign a color and slot in the graph for a joint. Returns
/// (color_index, local_index) rather than the C interior pointer.
/// (b2CreateJointInGraph)
pub fn create_joint_in_graph(world: &mut World, joint_id: i32) -> (i32, i32) {
    let (body_id_a, body_id_b) = {
        let joint = &world.joints[joint_id as usize];
        (joint.edges[0].body_id, joint.edges[1].body_id)
    };
    let type_a = world.bodies[body_id_a as usize].type_;
    let type_b = world.bodies[body_id_b as usize].type_;

    let color_index = assign_joint_color(
        &mut world.constraint_graph,
        body_id_a,
        body_id_b,
        type_a,
        type_b,
    );

    world.constraint_graph.colors[color_index as usize]
        .joint_sims
        .push(JointSim::default());
    let local_index = world.constraint_graph.colors[color_index as usize]
        .joint_sims
        .len() as i32
        - 1;

    let joint = &mut world.joints[joint_id as usize];
    joint.color_index = color_index;
    joint.local_index = local_index;
    (color_index, local_index)
}

/// (b2AddJointToGraph)
pub fn add_joint_to_graph(world: &mut World, joint_sim: JointSim, joint_id: i32) {
    let (color_index, local_index) = create_joint_in_graph(world, joint_id);
    world.constraint_graph.colors[color_index as usize].joint_sims[local_index as usize] =
        joint_sim;
}

/// (b2RemoveJointFromGraph)
pub fn remove_joint_from_graph(
    world: &mut World,
    body_id_a: i32,
    body_id_b: i32,
    color_index: i32,
    local_index: i32,
) {
    debug_assert!((0..GRAPH_COLOR_COUNT).contains(&color_index));
    let color = &mut world.constraint_graph.colors[color_index as usize];

    if color_index != OVERFLOW_INDEX {
        // May clear static bodies, no effect
        color.body_set.clear_bit(body_id_a as u32);
        color.body_set.clear_bit(body_id_b as u32);
    }

    let moved_index = color.joint_sims.len() as i32 - 1;
    color.joint_sims.swap_remove(local_index as usize);
    if moved_index != local_index {
        // Fix moved joint
        let moved_id = color.joint_sims[local_index as usize].joint_id;
        let moved_joint = &mut world.joints[moved_id as usize];
        debug_assert!(moved_joint.set_index == AWAKE_SET);
        debug_assert!(moved_joint.color_index == color_index);
        debug_assert!(moved_joint.local_index == moved_index);
        moved_joint.local_index = local_index;
    }
}

/// Visualization colors for the constraint graph slots. The last index
/// (GRAPH_COLOR_COUNT - 1) is the overflow color. (b2_graphColors)
const GRAPH_COLORS: [crate::debug_draw::HexColor; GRAPH_COLOR_COUNT as usize] = {
    use crate::debug_draw::HexColor;
    [
        HexColor::RED,
        HexColor::ORANGE,
        HexColor::YELLOW,
        HexColor::LIME_GREEN,
        HexColor::SPRING_GREEN,
        HexColor::AQUA,
        HexColor::DODGER_BLUE,
        HexColor::BLUE_VIOLET,
        HexColor::MAGENTA,
        HexColor::DEEP_PINK,
        HexColor::CRIMSON,
        HexColor::CORAL,
        HexColor::GOLD,
        HexColor::GREEN_YELLOW,
        HexColor::MEDIUM_SEA_GREEN,
        HexColor::TURQUOISE,
        HexColor::DEEP_SKY_BLUE,
        HexColor::CORNFLOWER_BLUE,
        HexColor::MEDIUM_SLATE_BLUE,
        HexColor::MEDIUM_ORCHID,
        HexColor::HOT_PINK,
        HexColor::TOMATO,
        HexColor::KHAKI,
        HexColor::SILVER,
    ]
};

/// Get the visualization color assigned to a constraint graph color slot.
/// (b2GetGraphColor)
pub fn get_graph_color(index: i32) -> crate::debug_draw::HexColor {
    debug_assert!((0..GRAPH_COLOR_COUNT).contains(&index));
    GRAPH_COLORS[index as usize]
}
