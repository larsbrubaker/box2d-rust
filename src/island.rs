// Port of the island data model from box2d-cpp-reference/src/island.h.
// Logic from island.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;

/// Cached contact data stored in the island for fast contiguous iteration.
/// Avoids touching Contact during union-find in island splitting.
/// (b2ContactLink)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactLink {
    pub contact_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
}

/// Cached joint data stored in the island for fast contiguous iteration.
/// (b2JointLink)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JointLink {
    pub joint_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
}

/// Persistent island for awake bodies, joints, and contacts. Contacts are
/// touching. Contacts and joints may connect to static bodies, but static
/// bodies are not in the island. (b2Island)
///
/// <https://en.wikipedia.org/wiki/Component_(graph_theory)>
/// <https://en.wikipedia.org/wiki/Dynamic_connectivity>
#[derive(Debug, Clone)]
pub struct Island {
    /// index of solver set stored in World. May be NULL_INDEX.
    pub set_index: i32,

    /// island index within set. May be NULL_INDEX.
    pub local_index: i32,

    pub island_id: i32,

    /// How many contacts have been removed from this island. Used to determine
    /// if an island is a candidate for splitting.
    pub constraint_remove_count: i32,

    pub bodies: Vec<i32>,

    /// Contacts and joints that belong to this island. May connect to static
    /// bodies not in the island. Each link carries the two body ids so island
    /// splitting's union-find never needs to touch Contact/Joint.
    pub contacts: Vec<ContactLink>,
    pub joints: Vec<JointLink>,
}

impl Default for Island {
    fn default() -> Self {
        Island {
            set_index: NULL_INDEX,
            local_index: NULL_INDEX,
            island_id: NULL_INDEX,
            constraint_remove_count: 0,
            bodies: Vec::new(),
            contacts: Vec::new(),
            joints: Vec::new(),
        }
    }
}

/// Used to move islands across solver sets. (b2IslandSim)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IslandSim {
    pub island_id: i32,
}

impl Default for IslandSim {
    fn default() -> Self {
        IslandSim {
            island_id: NULL_INDEX,
        }
    }
}

// ---------------------------------------------------------------------------
// Island lifecycle from island.c. The link/unlink/merge/split logic lands with
// the contact and joint slices.
// ---------------------------------------------------------------------------

use crate::solver_set::{AWAKE_SET, FIRST_SLEEPING_SET};
use crate::world::World;

/// Create an empty island in the given set. Returns the island id.
/// (b2CreateIsland — C returns a pointer; Rust returns the id)
pub fn create_island(world: &mut World, set_index: i32) -> i32 {
    debug_assert!(set_index == AWAKE_SET || set_index >= FIRST_SLEEPING_SET);

    let island_id = world.island_id_pool.alloc_id();

    if island_id == world.islands.len() as i32 {
        world.islands.push(Island::default());
    } else {
        debug_assert!(world.islands[island_id as usize].set_index == NULL_INDEX);
    }

    let set = &mut world.solver_sets[set_index as usize];

    let local_index = set.island_sims.len() as i32;
    set.island_sims.push(IslandSim { island_id });

    let island = &mut world.islands[island_id as usize];
    island.set_index = set_index;
    island.local_index = local_index;
    island.island_id = island_id;
    island.bodies = Vec::new();
    island.contacts = Vec::new();
    island.joints = Vec::new();
    island.constraint_remove_count = 0;

    island_id
}

/// (b2DestroyIsland)
pub fn destroy_island(world: &mut World, island_id: i32) {
    if world.split_island_id == island_id {
        world.split_island_id = NULL_INDEX;
    }

    // assume island is empty
    let (set_index, local_index) = {
        let island = &world.islands[island_id as usize];
        (island.set_index, island.local_index)
    };
    let set = &mut world.solver_sets[set_index as usize];
    {
        let last_index = set.island_sims.len() - 1;
        debug_assert!(0 <= local_index && local_index as usize <= last_index);
        let move_island_id = set.island_sims[last_index].island_id;
        set.island_sims.swap_remove(local_index as usize);
        world.islands[move_island_id as usize].local_index = local_index;
    }

    // Free island and id (preserve island revision)
    let island = &mut world.islands[island_id as usize];
    island.bodies = Vec::new();
    island.contacts = Vec::new();
    island.joints = Vec::new();
    island.constraint_remove_count = 0;
    island.local_index = NULL_INDEX;
    island.island_id = NULL_INDEX;
    island.set_index = NULL_INDEX;

    world.island_id_pool.free_id(island_id);
}

/// Merge two islands, keeping the bigger one. Returns the surviving island id.
/// (static b2MergeIslands)
pub(crate) fn merge_islands(world: &mut World, island_id_a: i32, island_id_b: i32) -> i32 {
    if island_id_a == island_id_b {
        return island_id_a;
    }

    if island_id_a == NULL_INDEX {
        debug_assert!(island_id_b != NULL_INDEX);
        return island_id_b;
    }

    if island_id_b == NULL_INDEX {
        debug_assert!(island_id_a != NULL_INDEX);
        return island_id_a;
    }

    // Keep the biggest island to reduce cache misses
    let (big_island_id, small_island_id) = {
        let count_a = world.islands[island_id_a as usize].bodies.len();
        let count_b = world.islands[island_id_b as usize].bodies.len();
        if count_a >= count_b {
            (island_id_a, island_id_b)
        } else {
            (island_id_b, island_id_a)
        }
    };

    // Move bodies from smaller island to larger island
    let small_bodies = std::mem::take(&mut world.islands[small_island_id as usize].bodies);
    for &body_id in &small_bodies {
        let body = &mut world.bodies[body_id as usize];
        debug_assert!(body.island_id == small_island_id);
        body.island_id = big_island_id;
        body.island_index = world.islands[big_island_id as usize].bodies.len() as i32;
        world.islands[big_island_id as usize].bodies.push(body_id);
    }

    // Migrate contacts from smaller island to larger island
    let small_contacts = std::mem::take(&mut world.islands[small_island_id as usize].contacts);
    for link in &small_contacts {
        let contact = &mut world.contacts[link.contact_id as usize];
        contact.island_id = big_island_id;
        contact.island_index = world.islands[big_island_id as usize].contacts.len() as i32;
        world.islands[big_island_id as usize].contacts.push(*link);
    }

    // Migrate joints from smaller island to larger island
    let small_joints = std::mem::take(&mut world.islands[small_island_id as usize].joints);
    for link in &small_joints {
        let joint = &mut world.joints[link.joint_id as usize];
        joint.island_id = big_island_id;
        joint.island_index = world.islands[big_island_id as usize].joints.len() as i32;
        world.islands[big_island_id as usize].joints.push(*link);
    }

    // Track removed constraints
    let small_removed = world.islands[small_island_id as usize].constraint_remove_count;
    world.islands[big_island_id as usize].constraint_remove_count += small_removed;

    destroy_island(world, small_island_id);

    validate_island(world, big_island_id);

    big_island_id
}

/// (static b2AddContactToIsland)
fn add_contact_to_island(world: &mut World, island_id: i32, contact_id: i32) {
    let (edge_body_a, edge_body_b) = {
        let contact = &world.contacts[contact_id as usize];
        debug_assert!(contact.island_id == NULL_INDEX);
        debug_assert!(contact.island_index == NULL_INDEX);
        (contact.edges[0].body_id, contact.edges[1].body_id)
    };

    let island = &mut world.islands[island_id as usize];

    let island_index = island.contacts.len() as i32;
    island.contacts.push(ContactLink {
        contact_id,
        body_id_a: edge_body_a,
        body_id_b: edge_body_b,
    });

    let contact = &mut world.contacts[contact_id as usize];
    contact.island_id = island_id;
    contact.island_index = island_index;

    validate_island(world, island_id);
}

/// Link a contact into an island when it starts having contact points.
/// (b2LinkContact)
pub fn link_contact(world: &mut World, contact_id: i32) {
    use crate::contact::contact_flags::TOUCHING;
    use crate::solver_set::DISABLED_SET;

    let (body_id_a, body_id_b) = {
        let contact = &world.contacts[contact_id as usize];
        debug_assert!(contact.flags & TOUCHING != 0);
        (contact.edges[0].body_id, contact.edges[1].body_id)
    };

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;

    debug_assert!(set_a != DISABLED_SET && set_b != DISABLED_SET);
    debug_assert!(set_a != crate::solver_set::STATIC_SET || set_b != crate::solver_set::STATIC_SET);

    // Wake bodyB if bodyA is awake and bodyB is sleeping
    if set_a == AWAKE_SET && set_b >= FIRST_SLEEPING_SET {
        crate::solver_set::wake_solver_set(world, set_b);
    }

    // Wake bodyA if bodyB is awake and bodyA is sleeping
    let set_a2 = world.bodies[body_id_a as usize].set_index;
    let set_b2 = world.bodies[body_id_b as usize].set_index;
    if set_b2 == AWAKE_SET && set_a2 >= FIRST_SLEEPING_SET {
        crate::solver_set::wake_solver_set(world, set_a2);
    }

    let island_id_a = world.bodies[body_id_a as usize].island_id;
    let island_id_b = world.bodies[body_id_b as usize].island_id;

    // Static bodies have null island indices.
    debug_assert!(
        world.bodies[body_id_a as usize].set_index != crate::solver_set::STATIC_SET
            || island_id_a == NULL_INDEX
    );
    debug_assert!(
        world.bodies[body_id_b as usize].set_index != crate::solver_set::STATIC_SET
            || island_id_b == NULL_INDEX
    );
    debug_assert!(island_id_a != NULL_INDEX || island_id_b != NULL_INDEX);

    // Merge islands. This will destroy one of the islands.
    let final_island_id = merge_islands(world, island_id_a, island_id_b);

    // Add contact to the island that survived
    add_contact_to_island(world, final_island_id, contact_id);
}

/// Unlink a contact from the island graph when it stops having contact points
/// or is destroyed. (b2UnlinkContact)
pub fn unlink_contact(world: &mut World, contact_id: i32) {
    let (island_id, remove_index) = {
        let contact = &world.contacts[contact_id as usize];
        debug_assert!(contact.island_id != NULL_INDEX);
        (contact.island_id, contact.island_index)
    };

    // remove from island
    {
        let island = &mut world.islands[island_id as usize];
        debug_assert!(0 <= remove_index && (remove_index as usize) < island.contacts.len());
        debug_assert!(island.contacts[remove_index as usize].contact_id == contact_id);

        let moved_index = island.contacts.len() as i32 - 1;
        island.contacts.swap_remove(remove_index as usize);
        if moved_index != remove_index {
            // Fix islandIndex on the contact that was swapped into removeIndex
            let moved_contact_id = island.contacts[remove_index as usize].contact_id;
            let moved_contact = &mut world.contacts[moved_contact_id as usize];
            debug_assert!(moved_contact.island_index == moved_index);
            moved_contact.island_index = remove_index;
        }
    }

    let contact = &mut world.contacts[contact_id as usize];
    contact.island_id = NULL_INDEX;
    contact.island_index = NULL_INDEX;
    world.islands[island_id as usize].constraint_remove_count += 1;

    validate_island(world, island_id);
}

/// (static b2AddJointToIsland)
fn add_joint_to_island(world: &mut World, island_id: i32, joint_id: i32) {
    let (edge_body_a, edge_body_b) = {
        let joint = &world.joints[joint_id as usize];
        debug_assert!(joint.island_id == NULL_INDEX);
        debug_assert!(joint.island_index == NULL_INDEX);
        (joint.edges[0].body_id, joint.edges[1].body_id)
    };

    let island = &mut world.islands[island_id as usize];

    let island_index = island.joints.len() as i32;
    island.joints.push(JointLink {
        joint_id,
        body_id_a: edge_body_a,
        body_id_b: edge_body_b,
    });

    let joint = &mut world.joints[joint_id as usize];
    joint.island_id = island_id;
    joint.island_index = island_index;

    validate_island(world, island_id);
}

/// Link a joint into the island graph when it is created. (b2LinkJoint)
pub fn link_joint(world: &mut World, joint_id: i32) {
    use crate::types::BodyType;

    let (body_id_a, body_id_b) = {
        let joint = &world.joints[joint_id as usize];
        (joint.edges[0].body_id, joint.edges[1].body_id)
    };

    debug_assert!(
        world.bodies[body_id_a as usize].type_ == BodyType::Dynamic
            || world.bodies[body_id_b as usize].type_ == BodyType::Dynamic
    );

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;

    if set_a == AWAKE_SET && set_b >= FIRST_SLEEPING_SET {
        crate::solver_set::wake_solver_set(world, set_b);
    } else if set_b == AWAKE_SET && set_a >= FIRST_SLEEPING_SET {
        crate::solver_set::wake_solver_set(world, set_a);
    }

    let island_id_a = world.bodies[body_id_a as usize].island_id;
    let island_id_b = world.bodies[body_id_b as usize].island_id;

    debug_assert!(island_id_a != NULL_INDEX || island_id_b != NULL_INDEX);

    // Merge islands. This will destroy one of the islands.
    let final_island_id = merge_islands(world, island_id_a, island_id_b);

    // Add joint to the island that survived
    add_joint_to_island(world, final_island_id, joint_id);
}

/// Unlink a joint from the island graph when it is destroyed. (b2UnlinkJoint)
pub fn unlink_joint(world: &mut World, joint_id: i32) {
    let (island_id, remove_index) = {
        let joint = &world.joints[joint_id as usize];
        if joint.island_id == NULL_INDEX {
            return;
        }
        (joint.island_id, joint.island_index)
    };

    // remove from island
    {
        let island = &mut world.islands[island_id as usize];
        debug_assert!(0 <= remove_index && (remove_index as usize) < island.joints.len());
        debug_assert!(island.joints[remove_index as usize].joint_id == joint_id);

        let moved_index = island.joints.len() as i32 - 1;
        island.joints.swap_remove(remove_index as usize);
        if moved_index != remove_index {
            // Fix islandIndex on the joint that was swapped into removeIndex
            let moved_joint_id = island.joints[remove_index as usize].joint_id;
            let moved_joint = &mut world.joints[moved_joint_id as usize];
            debug_assert!(moved_joint.island_index == moved_index);
            moved_joint.island_index = remove_index;
        }
    }

    let joint = &mut world.joints[joint_id as usize];
    joint.island_id = NULL_INDEX;
    joint.island_index = NULL_INDEX;
    world.islands[island_id as usize].constraint_remove_count += 1;

    validate_island(world, island_id);
}

/// Validate island connectivity and bookkeeping. (b2ValidateIsland)
///
/// The C version is compiled in only with B2_ENABLE_VALIDATION; here it always
/// runs when called and asserts in debug builds.
pub fn validate_island(world: &World, island_id: i32) {
    if island_id == NULL_INDEX {
        return;
    }

    let island = &world.islands[island_id as usize];
    debug_assert!(island.island_id == island_id);
    debug_assert!(island.set_index != NULL_INDEX);

    {
        debug_assert!(!island.bodies.is_empty());
        debug_assert!(island.bodies.len() as i32 <= world.body_id_pool.id_count());

        for (i, &body_id) in island.bodies.iter().enumerate() {
            let body = &world.bodies[body_id as usize];
            debug_assert!(body.island_id == island_id);
            debug_assert!(body.island_index == i as i32);
            debug_assert!(body.set_index == island.set_index);
            let _ = (body, i);
        }
    }

    if !island.contacts.is_empty() {
        debug_assert!(island.contacts.len() as i32 <= world.contact_id_pool.id_count());

        for (i, link) in island.contacts.iter().enumerate() {
            let contact = &world.contacts[link.contact_id as usize];
            debug_assert!(contact.set_index == island.set_index);
            debug_assert!(contact.island_id == island_id);
            debug_assert!(contact.island_index == i as i32);
            let _ = (contact, i);
        }
    }

    if !island.joints.is_empty() {
        debug_assert!(island.joints.len() as i32 <= world.joint_id_pool.id_count());

        for (i, link) in island.joints.iter().enumerate() {
            let joint = &world.joints[link.joint_id as usize];
            debug_assert!(joint.set_index == island.set_index);
            debug_assert!(joint.island_id == island_id);
            debug_assert!(joint.island_index == i as i32);
            let _ = (joint, i);
        }
    }
}

/// Find parent of a node. Uses path halving to speed up further queries.
/// (static b2IslandFindParent)
fn island_find_parent(parents: &mut [i32], mut node: i32) -> i32 {
    // Walk the chain of parents to find the node that is its own parent (the
    // root)
    while parents[node as usize] != node {
        let grand_parent = parents[parents[node as usize] as usize];
        parents[node as usize] = grand_parent;
        node = grand_parent;
    }

    node
}

/// Connect the components containing node1 and node2. Uses rank to keep the
/// tree balanced. Tracks per-component contact and joint counts.
/// (static b2IslandUnion)
fn island_union(
    parents: &mut [i32],
    ranks: &mut [i32],
    node1: i32,
    node2: i32,
    contact_counts: &mut [i32],
    joint_counts: &mut [i32],
) {
    let root1 = island_find_parent(parents, node1);
    let root2 = island_find_parent(parents, node2);
    if root1 != root2 {
        if ranks[root1 as usize] < ranks[root2 as usize] {
            parents[root1 as usize] = root2;
            contact_counts[root2 as usize] += contact_counts[root1 as usize];
            joint_counts[root2 as usize] += joint_counts[root1 as usize];
        } else if ranks[root1 as usize] > ranks[root2 as usize] {
            parents[root2 as usize] = root1;
            contact_counts[root1 as usize] += contact_counts[root2 as usize];
            joint_counts[root1 as usize] += joint_counts[root2 as usize];
        } else {
            parents[root2 as usize] = root1;
            ranks[root1 as usize] += 1;
            contact_counts[root1 as usize] += contact_counts[root2 as usize];
            joint_counts[root1 as usize] += joint_counts[root2 as usize];
        }
    }
}

/// Split an island because some contacts and/or joints have been removed.
/// This uses union-find and touches a lot of memory, so it can be slow.
/// Note: contacts/joints connected to static bodies must belong to an island
/// but don't affect island connectivity.
/// Note: static bodies are never in an island.
/// (b2SplitIsland / b2SplitIslandTask — the C runs this as a task in
/// parallel with the constraint solve; the serial port calls it inline)
pub fn split_island(world: &mut World, base_id: i32) {
    debug_assert!(world.islands[base_id as usize].constraint_remove_count > 0);
    debug_assert!(world.islands[base_id as usize].set_index == AWAKE_SET);

    validate_island(world, base_id);

    // Detach the base island's arrays. (The C caches raw pointers because
    // b2CreateIsland may reallocate the island array; taking the Vecs is the
    // owned equivalent and also protects them from b2DestroyIsland.)
    let base_body_ids = std::mem::take(&mut world.islands[base_id as usize].bodies);
    let base_contacts = std::mem::take(&mut world.islands[base_id as usize].contacts);
    let base_joints = std::mem::take(&mut world.islands[base_id as usize].joints);

    let base_body_count = base_body_ids.len();

    // Arena scratch in C; plain Vecs here.
    let mut parents: Vec<i32> = (0..base_body_count as i32).collect();
    let mut contact_counts: Vec<i32> = vec![0; base_body_count];
    let mut joint_counts: Vec<i32> = vec![0; base_body_count];
    let mut ranks: Vec<i32> = vec![0; base_body_count];

    // Union over contacts, tracking per-component contact counts
    for link in &base_contacts {
        let island_index_a = world.bodies[link.body_id_a as usize].island_index;
        let island_index_b = world.bodies[link.body_id_b as usize].island_index;

        // Only connect non-static bodies
        if island_index_a != NULL_INDEX && island_index_b != NULL_INDEX {
            island_union(
                &mut parents,
                &mut ranks,
                island_index_a,
                island_index_b,
                &mut contact_counts,
                &mut joint_counts,
            );
            let root = island_find_parent(&mut parents, island_index_a);
            contact_counts[root as usize] += 1;
        } else {
            let island_index = if island_index_a != NULL_INDEX {
                island_index_a
            } else {
                island_index_b
            };
            let root = island_find_parent(&mut parents, island_index);
            contact_counts[root as usize] += 1;
        }
    }

    // Union over joints, tracking per-component joint counts
    for link in &base_joints {
        let island_index_a = world.bodies[link.body_id_a as usize].island_index;
        let island_index_b = world.bodies[link.body_id_b as usize].island_index;

        // Only connect non-static bodies
        if island_index_a != NULL_INDEX && island_index_b != NULL_INDEX {
            island_union(
                &mut parents,
                &mut ranks,
                island_index_a,
                island_index_b,
                &mut contact_counts,
                &mut joint_counts,
            );
            let root = island_find_parent(&mut parents, island_index_a);
            joint_counts[root as usize] += 1;
        } else {
            let island_index = if island_index_a != NULL_INDEX {
                island_index_a
            } else {
                island_index_b
            };
            let root = island_find_parent(&mut parents, island_index);
            joint_counts[root as usize] += 1;
        }
    }

    // Flatten all parent indices and count connected components.
    let mut component_count = 0usize;
    #[allow(clippy::needless_range_loop)]
    for i in 0..base_body_count {
        let root = island_find_parent(&mut parents, i as i32);
        parents[i] = root;
        if root == i as i32 {
            component_count += 1;
        }
    }

    // Early return — island is still fully connected, no split needed.
    if component_count == 1 {
        let island = &mut world.islands[base_id as usize];
        island.constraint_remove_count = 0;
        island.bodies = base_body_ids;
        island.contacts = base_contacts;
        island.joints = base_joints;
        return;
    }

    // Map from body index to new island index. Only set for root bodies.
    let mut root_map: Vec<i32> = vec![NULL_INDEX; base_body_count];

    let mut component_body_counts: Vec<i32> = vec![0; component_count];
    let mut component_contact_counts: Vec<i32> = vec![0; component_count];
    let mut component_joint_counts: Vec<i32> = vec![0; component_count];
    let mut island_count = 0usize;

    // Find the root body for each body and create islands as needed.
    // Extract per-component counts from the root nodes' accumulated counts.
    #[allow(clippy::needless_range_loop)]
    for i in 0..base_body_count {
        let root_index = parents[i] as usize;
        if root_map[root_index] == NULL_INDEX {
            root_map[root_index] = island_count as i32;
            component_body_counts[island_count] = 0;
            component_contact_counts[island_count] = contact_counts[root_index];
            component_joint_counts[island_count] = joint_counts[root_index];
            island_count += 1;
        }

        component_body_counts[root_map[root_index] as usize] += 1;
    }

    debug_assert!(island_count == component_count);

    // Map from new island index to island id
    let mut island_ids: Vec<i32> = Vec::with_capacity(island_count);

    // Create new islands and reserve body/contact/joint arrays
    for i in 0..island_count {
        let new_island_id = create_island(world, AWAKE_SET);
        island_ids.push(new_island_id);

        // Reserve arrays to avoid wasteful growth
        let new_island = &mut world.islands[new_island_id as usize];
        new_island.bodies.reserve(component_body_counts[i] as usize);
        new_island
            .contacts
            .reserve(component_contact_counts[i] as usize);
        new_island
            .joints
            .reserve(component_joint_counts[i] as usize);
    }

    // Assign bodies to new islands
    for (i, &body_id) in base_body_ids.iter().enumerate() {
        let root = island_find_parent(&mut parents, i as i32);
        let new_island_id = island_ids[root_map[root as usize] as usize];

        let island_index = world.islands[new_island_id as usize].bodies.len() as i32;
        let body = &mut world.bodies[body_id as usize];
        body.island_id = new_island_id;
        body.island_index = island_index;

        world.islands[new_island_id as usize].bodies.push(body_id);
    }

    // Assign contacts to the island of their bodies
    for link in &base_contacts {
        // Static bodies don't have an island id.
        let island_id_a = world.bodies[link.body_id_a as usize].island_id;
        let target_island_id = if island_id_a != NULL_INDEX {
            island_id_a
        } else {
            world.bodies[link.body_id_b as usize].island_id
        };

        let island_index = world.islands[target_island_id as usize].contacts.len() as i32;
        let contact = &mut world.contacts[link.contact_id as usize];
        contact.island_id = target_island_id;
        contact.island_index = island_index;

        world.islands[target_island_id as usize]
            .contacts
            .push(*link);
    }

    // Assign joints to the island of their bodies
    for link in &base_joints {
        // Static bodies don't have an island id.
        let island_id_a = world.bodies[link.body_id_a as usize].island_id;
        let target_island_id = if island_id_a != NULL_INDEX {
            island_id_a
        } else {
            world.bodies[link.body_id_b as usize].island_id
        };

        let island_index = world.islands[target_island_id as usize].joints.len() as i32;
        let joint = &mut world.joints[link.joint_id as usize];
        joint.island_id = target_island_id;
        joint.island_index = island_index;

        world.islands[target_island_id as usize].joints.push(*link);
    }

    // Destroy the base island
    destroy_island(world, base_id);
}
