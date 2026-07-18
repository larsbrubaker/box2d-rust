// The narrow phase from physics_world.c: b2CollideTask/b2Collide (narrow phase
// + contact state transitions), plus b2ValidateContacts.
//
// The serial port gathers contact locations instead of the C's arena array of
// ContactSim pointers, and processes them in the same order: graph colors
// 0..GRAPH_COLOR_COUNT (overflow included) then the awake set's non-touching
// contacts.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::World;
use crate::body::BodySim;
use crate::constants::{speculative_distance, GRAPH_COLOR_COUNT};
use crate::contact::{contact_flags, update_contact, Contact, ContactSim, ContactUpdateContext};
use crate::core::NULL_INDEX;
use crate::events::{ContactBeginTouchEvent, ContactEndTouchEvent};
use crate::id::{ContactId, ShapeId};
use crate::math_functions::{
    aabb_overlaps, abs_float, distance, dot, inv_mul_rot, inv_mul_world_transforms, invert_rot,
    max_float, min_float, mul_rot, rotate_vector, sub, sub_pos, Rot,
};
use crate::solver::StepContext;
use crate::solver_set::{SolverSet, AWAKE_SET, FIRST_SLEEPING_SET};
use crate::types::BodyType;

/// (static inline b2RelativeCos)
fn relative_cos(a: Rot, b: Rot) -> f32 {
    a.c * b.c + a.s * b.s
}

/// Location of a contact sim during the narrow phase: a graph color index or
/// NULL_INDEX for the awake set's non-touching array, plus the local index.
/// (The C gathers b2ContactSim pointers; pointers are unstable in Rust.)
#[derive(Clone, Copy)]
struct ContactSimLocation {
    color_index: i32,
    local_index: i32,
}

/// Borrow a body sim in place from the split solver sets. The solver sets are
/// split so the awake set's non-touching contact sims can be borrowed mutably
/// while its body sims (and those of the static/sleeping sets) are read through
/// shared references, mirroring the C which holds raw pointers to the contact
/// sim and both body sims at once (physics_world.c b2CollideTask).
fn located_body_sim<'a>(
    head_sets: &'a [SolverSet],
    awake_body_sims: &'a [BodySim],
    sleeping_sets: &'a [SolverSet],
    set_index: i32,
    local_index: i32,
) -> &'a BodySim {
    let local = local_index as usize;
    if set_index == AWAKE_SET {
        &awake_body_sims[local]
    } else if set_index < AWAKE_SET {
        &head_sets[set_index as usize].body_sims[local]
    } else {
        &sleeping_sets[(set_index - FIRST_SLEEPING_SET) as usize].body_sims[local]
    }
}

/// Update one contact in the narrow phase. (the b2CollideTask loop body)
fn collide_contact(
    world: &mut World,
    location: ContactSimLocation,
    update_context: &ContactUpdateContext,
) {
    let recycle_distance = world.contact_recycle_distance;
    let speculative_distance_ = speculative_distance();
    let recycle_distance_non_touching = min_float(recycle_distance, speculative_distance_);

    // Split the solver sets so the awake set's non-touching contact sims can be
    // mutated in place while the body sims (which may live in the same set) are
    // read through shared references. This mirrors the C, which holds raw
    // pointers to the contact sim and both body sims at once, avoiding the
    // per-contact ContactSim/BodySim copies of the earlier port.
    let (head_sets, tail_sets) = world.solver_sets.split_at_mut(AWAKE_SET as usize);
    let (awake_slice, sleeping_sets) = tail_sets.split_at_mut(1);
    let SolverSet {
        contact_sims: awake_contact_sims,
        body_sims: awake_body_sims,
        ..
    } = &mut awake_slice[0];

    // Update the contact sim in place: touching contacts live in the constraint
    // graph, non-touching contacts in the awake set.
    let contact_sim: &mut ContactSim = if location.color_index != NULL_INDEX {
        &mut world.constraint_graph.colors[location.color_index as usize].contact_sims
            [location.local_index as usize]
    } else {
        &mut awake_contact_sims[location.local_index as usize]
    };

    let contact_id = contact_sim.contact_id;

    // Do proxies still overlap?
    let overlap = {
        let shape_a = &world.shapes[contact_sim.shape_id_a as usize];
        let shape_b = &world.shapes[contact_sim.shape_id_b as usize];
        aabb_overlaps(shape_a.fat_aabb, shape_b.fat_aabb)
    };

    if !overlap {
        contact_sim.sim_flags |= contact_flags::SIM_DISJOINT;
        contact_sim.sim_flags &= !contact_flags::SIM_TOUCHING;
        world.task_contexts[0]
            .contact_state_bit_set
            .set_bit(contact_id as u32);
        return;
    }

    let was_touching = contact_sim.sim_flags & contact_flags::SIM_TOUCHING != 0;

    // Update contact respecting shape/body order (A,B)
    let (shape_body_id_a, shape_body_id_b) = {
        let shape_a = &world.shapes[contact_sim.shape_id_a as usize];
        let shape_b = &world.shapes[contact_sim.shape_id_b as usize];
        (shape_a.body_id, shape_b.body_id)
    };

    let body_a = &world.bodies[shape_body_id_a as usize];
    let body_b = &world.bodies[shape_body_id_b as usize];
    let body_sim_a = located_body_sim(
        &*head_sets,
        &*awake_body_sims,
        &*sleeping_sets,
        body_a.set_index,
        body_a.local_index,
    );
    let body_sim_b = located_body_sim(
        &*head_sets,
        &*awake_body_sims,
        &*sleeping_sets,
        body_b.set_index,
        body_b.local_index,
    );
    let transform_a = body_sim_a.transform;
    let transform_b = body_sim_b.transform;

    // These may not be skipped by relative transform check below
    contact_sim.body_sim_index_a = if body_a.set_index == AWAKE_SET {
        body_a.local_index
    } else {
        NULL_INDEX
    };
    contact_sim.inv_mass_a = body_sim_a.inv_mass;
    contact_sim.inv_i_a = body_sim_a.inv_inertia;

    contact_sim.body_sim_index_b = if body_b.set_index == AWAKE_SET {
        body_b.local_index
    } else {
        NULL_INDEX
    };
    contact_sim.inv_mass_b = body_sim_b.inv_mass;
    contact_sim.inv_i_b = body_sim_b.inv_inertia;

    let type_a = body_a.type_;
    let type_b = body_b.type_;

    // Contact recycling optimization. Please cite this code if you use this
    // optimization. This is inspired by persistent contact manifolds used in
    // some physics engines, such as PhysX. However, this allows larger
    // relative motion and has fewer tuning parameters (just one).
    if recycle_distance > 0.0
        && contact_sim.sim_flags & contact_flags::SIM_RELATIVE_TRANSFORM_VALID != 0
        && contact_sim.sim_flags & contact_flags::RECYCLE != 0
    {
        let cached_q_a = contact_sim.cached_rotation_a;
        let cached_q_b = contact_sim.cached_rotation_b;
        let xfc = contact_sim.cached_relative_pose;
        let xf = inv_mul_world_transforms(transform_a, transform_b);

        let cos_a = relative_cos(transform_a.q, cached_q_a);
        let cos_b = relative_cos(transform_b.q, cached_q_b);
        let min_cos = min_float(cos_a, cos_b);

        let max_extent_a = if type_a == BodyType::Static {
            0.0
        } else {
            body_sim_a.max_extent
        };
        let max_extent_b = if type_b == BodyType::Static {
            0.0
        } else {
            body_sim_b.max_extent
        };
        let max_extent = max_float(max_extent_a, max_extent_b);
        let distance_ = distance(xf.p, xfc.p);
        let qr = inv_mul_rot(xf.q, xfc.q);

        // This metric is used for fast bodies and sleeping. It comes from
        // conservative advancement. Note that qr.s == sin(theta) ~= theta for
        // small angles. Need a tighter tolerance for non-touching shapes so
        // that contacts are not missed.
        let tolerance = if was_touching {
            recycle_distance
        } else {
            recycle_distance_non_touching
        };

        if min_cos > crate::constants::CONTACT_RECYCLE_COS_ANGLE
            && distance_ + max_extent * abs_float(qr.s) < tolerance
        {
            let dq_a = mul_rot(transform_a.q, invert_rot(cached_q_a));
            let dq_b = mul_rot(transform_b.q, invert_rot(cached_q_b));
            let normal = contact_sim.manifold.normal;

            // Minimize round-off
            let dc = sub_pos(body_sim_b.center, body_sim_a.center);

            for i in 0..contact_sim.manifold.point_count as usize {
                // Keep anchors but update separation, same as sub-stepping.
                // This eliminates jitter.
                let mp = &mut contact_sim.manifold.points[i];
                let r_a = rotate_vector(dq_a, mp.anchor_a);
                let r_b = rotate_vector(dq_b, mp.anchor_b);
                let dp = crate::math_functions::add(dc, sub(r_b, r_a));
                mp.separation = mp.base_separation + dot(dp, normal);
                mp.persisted = true;
            }

            world.task_contexts[0].recycled_contact_count += 1;

            // Contact is recycled. This also skips updating other aspects of
            // the contact such as material parameters.
            return;
        }
    }

    // Caching for contact recycling.
    contact_sim.cached_rotation_a = transform_a.q;
    contact_sim.cached_rotation_b = transform_b.q;
    contact_sim.cached_relative_pose = inv_mul_world_transforms(transform_a, transform_b);
    contact_sim.sim_flags |= contact_flags::SIM_RELATIVE_TRANSFORM_VALID;

    let center_offset_a = rotate_vector(transform_a.q, body_sim_a.local_center);
    let center_offset_b = rotate_vector(transform_b.q, body_sim_b.local_center);

    // This updates solid contacts
    let touching = {
        let shape_a = &world.shapes[contact_sim.shape_id_a as usize];
        let shape_b = &world.shapes[contact_sim.shape_id_b as usize];
        update_contact(
            update_context,
            contact_sim,
            shape_a,
            transform_a,
            center_offset_a,
            shape_b,
            transform_b,
            center_offset_b,
        )
    };

    // State changes that affect island connectivity. Also affects contact
    // events.
    if touching && !was_touching {
        contact_sim.sim_flags |= contact_flags::SIM_STARTED_TOUCHING;
        world.task_contexts[0]
            .contact_state_bit_set
            .set_bit(contact_id as u32);
    } else if !touching && was_touching {
        contact_sim.sim_flags |= contact_flags::SIM_STOPPED_TOUCHING;
        world.task_contexts[0]
            .contact_state_bit_set
            .set_bit(contact_id as u32);
    }

    for i in 0..contact_sim.manifold.point_count as usize {
        let mp = &mut contact_sim.manifold.points[i];
        mp.base_separation = mp.separation;
    }
}

/// (static b2AddNonTouchingContact)
fn add_non_touching_contact(world: &mut World, contact_id: i32, contact_sim: ContactSim) {
    debug_assert!(world.contacts[contact_id as usize].set_index == AWAKE_SET);
    let local_index = world.solver_sets[AWAKE_SET as usize].contact_sims.len() as i32;
    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.color_index = NULL_INDEX;
        contact.local_index = local_index;
    }
    world.solver_sets[AWAKE_SET as usize]
        .contact_sims
        .push(contact_sim);
}

/// (static b2RemoveNonTouchingContact)
fn remove_non_touching_contact(world: &mut World, set_index: i32, local_index: i32) {
    let set = &mut world.solver_sets[set_index as usize];
    let moved_index = set.contact_sims.len() as i32 - 1;
    set.contact_sims.swap_remove(local_index as usize);
    if moved_index != local_index {
        let moved_contact_id =
            world.solver_sets[set_index as usize].contact_sims[local_index as usize].contact_id;
        let moved_contact = &mut world.contacts[moved_contact_id as usize];
        debug_assert!(moved_contact.set_index == set_index);
        debug_assert!(moved_contact.local_index == moved_index);
        debug_assert!(moved_contact.color_index == NULL_INDEX);
        moved_contact.local_index = local_index;
    }
}

/// Narrow-phase collision. (b2Collide)
pub(super) fn collide(world: &mut World, _context: &StepContext) {
    // gather contacts into a single array for easier iteration; the order is
    // graph colors then the awake set, matching the C gather. Size the buffer
    // up front from the same counts the loops use, mirroring the C's single
    // b2StackAlloc( contactCount * ... ) (physics_world.c b2Collide).
    let mut contact_count = world.solver_sets[AWAKE_SET as usize].contact_sims.len();
    for color_index in 0..GRAPH_COLOR_COUNT {
        contact_count += world.constraint_graph.colors[color_index as usize]
            .contact_sims
            .len();
    }
    let mut locations: Vec<ContactSimLocation> = Vec::with_capacity(contact_count);
    for color_index in 0..GRAPH_COLOR_COUNT {
        let count = world.constraint_graph.colors[color_index as usize]
            .contact_sims
            .len();
        for local_index in 0..count as i32 {
            locations.push(ContactSimLocation {
                color_index,
                local_index,
            });
        }
    }
    {
        let count = world.solver_sets[AWAKE_SET as usize].contact_sims.len();
        for local_index in 0..count as i32 {
            locations.push(ContactSimLocation {
                color_index: NULL_INDEX,
                local_index,
            });
        }
    }

    if locations.is_empty() {
        return;
    }

    // Contact bit set on ids because contact locations are unstable as they
    // move between touching and not touching.
    let contact_id_capacity = world.contact_id_pool.id_capacity();
    {
        let task_context = &mut world.task_contexts[0];
        task_context
            .contact_state_bit_set
            .set_bit_count_and_clear(contact_id_capacity as u32);
        task_context.recycled_contact_count = 0;
    }

    // (b2CollideTask over the whole range on one worker)
    let update_context = ContactUpdateContext::new(world);
    for &location in &locations {
        collide_contact(world, location, &update_context);
    }

    // Serially update contact state. Process contact state changes, iterating
    // over set bits.
    let end_event_array_index = world.end_event_array_index as usize;
    let world_id = world.world_id;

    let block_count = world.task_contexts[0].contact_state_bit_set.block_count();
    for k in 0..block_count {
        let mut bits = world.task_contexts[0].contact_state_bit_set.block(k);
        while bits != 0 {
            let ctz = bits.trailing_zeros();
            let contact_id = (64 * k + ctz) as i32;

            let (color_index, local_index, flags, generation, shape_id_a, shape_id_b) = {
                let contact: &Contact = &world.contacts[contact_id as usize];
                debug_assert!(contact.set_index == AWAKE_SET);
                (
                    contact.color_index,
                    contact.local_index,
                    contact.flags,
                    contact.generation,
                    contact.shape_id_a,
                    contact.shape_id_b,
                )
            };

            let sim_flags = if color_index != NULL_INDEX {
                // contact lives in constraint graph
                debug_assert!((0..GRAPH_COLOR_COUNT).contains(&color_index));
                world.constraint_graph.colors[color_index as usize].contact_sims
                    [local_index as usize]
                    .sim_flags
            } else {
                world.solver_sets[AWAKE_SET as usize].contact_sims[local_index as usize].sim_flags
            };

            let (shape_id_a_full, shape_id_b_full) = {
                let shape_a = &world.shapes[shape_id_a as usize];
                let shape_b = &world.shapes[shape_id_b as usize];
                (
                    ShapeId {
                        index1: shape_a.id + 1,
                        world0: world_id,
                        generation: shape_a.generation,
                    },
                    ShapeId {
                        index1: shape_b.id + 1,
                        world0: world_id,
                        generation: shape_b.generation,
                    },
                )
            };
            let contact_full_id = ContactId {
                index1: contact_id + 1,
                world0: world_id,
                padding: 0,
                generation,
            };

            if sim_flags & contact_flags::SIM_DISJOINT != 0 {
                // Bounding boxes no longer overlap
                crate::contact::destroy_contact(world, contact_id, false);
            } else if sim_flags & contact_flags::SIM_STARTED_TOUCHING != 0 {
                debug_assert!(world.contacts[contact_id as usize].island_id == NULL_INDEX);

                if flags & contact_flags::ENABLE_CONTACT_EVENTS != 0 {
                    world.contact_begin_events.push(ContactBeginTouchEvent {
                        shape_id_a: shape_id_a_full,
                        shape_id_b: shape_id_b_full,
                        contact_id: contact_full_id,
                    });
                }

                debug_assert!(color_index == NULL_INDEX);
                debug_assert!(
                    world.solver_sets[AWAKE_SET as usize].contact_sims[local_index as usize]
                        .manifold
                        .point_count
                        > 0
                );

                // Link first because this wakes colliding bodies and ensures
                // the body sims are in the correct place.
                world.contacts[contact_id as usize].flags |= contact_flags::TOUCHING;
                crate::island::link_contact(world, contact_id);

                // Make sure these didn't change
                debug_assert!(world.contacts[contact_id as usize].color_index == NULL_INDEX);
                debug_assert!(world.contacts[contact_id as usize].local_index == local_index);

                // Refresh the contact sim after the awake set may have grown
                let mut contact_sim =
                    world.solver_sets[AWAKE_SET as usize].contact_sims[local_index as usize];
                contact_sim.sim_flags &= !contact_flags::SIM_STARTED_TOUCHING;

                // Add first for memcpy
                crate::constraint_graph::add_contact_to_graph(world, contact_sim, contact_id);

                // This destroys the contact sim in the awake set
                remove_non_touching_contact(world, AWAKE_SET, local_index);
            } else if sim_flags & contact_flags::SIM_STOPPED_TOUCHING != 0 {
                debug_assert!(color_index != NULL_INDEX);
                let contact_sim = {
                    let sim = &mut world.constraint_graph.colors[color_index as usize].contact_sims
                        [local_index as usize];
                    sim.sim_flags &= !contact_flags::SIM_STOPPED_TOUCHING;
                    *sim
                };
                world.contacts[contact_id as usize].flags &= !contact_flags::TOUCHING;

                if flags & contact_flags::ENABLE_CONTACT_EVENTS != 0 {
                    world.contact_end_events[end_event_array_index].push(ContactEndTouchEvent {
                        shape_id_a: shape_id_a_full,
                        shape_id_b: shape_id_b_full,
                        contact_id: contact_full_id,
                    });
                }

                debug_assert!(contact_sim.manifold.point_count == 0);

                crate::island::unlink_contact(world, contact_id);
                let body_id_a = world.contacts[contact_id as usize].edges[0].body_id;
                let body_id_b = world.contacts[contact_id as usize].edges[1].body_id;

                // Add first for memcpy
                add_non_touching_contact(world, contact_id, contact_sim);
                crate::constraint_graph::remove_contact_from_graph(
                    world,
                    body_id_a,
                    body_id_b,
                    color_index,
                    local_index,
                );
            }

            // Clear the smallest set bit
            bits &= bits - 1;
        }
    }

    world.validate_solver_sets();
    world.validate_contacts();
}

impl World {
    /// (b2ValidateContacts — C compiles the body under B2_ENABLE_VALIDATION;
    /// here the whole check runs in debug builds only)
    pub fn validate_contacts(&self) {
        if !cfg!(debug_assertions) {
            return;
        }

        let contact_count = self.contacts.len() as i32;
        debug_assert!(contact_count == self.contact_id_pool.id_capacity());
        let mut allocated_contact_count = 0;

        for contact_index in 0..contact_count {
            let contact = &self.contacts[contact_index as usize];
            if contact.contact_id == NULL_INDEX {
                continue;
            }

            debug_assert!(contact.contact_id == contact_index);

            allocated_contact_count += 1;

            let touching = contact.flags & contact_flags::TOUCHING != 0;

            let set_id = contact.set_index;

            if set_id == AWAKE_SET {
                if touching {
                    debug_assert!(
                        0 <= contact.color_index && contact.color_index < GRAPH_COLOR_COUNT
                    );
                } else {
                    debug_assert!(contact.color_index == NULL_INDEX);
                }
            } else if set_id >= FIRST_SLEEPING_SET {
                // Only touching contacts allowed in a sleeping set
                debug_assert!(touching);
            } else {
                // Sleeping and non-touching contacts belong in the disabled set
                debug_assert!(!touching && set_id == crate::solver_set::DISABLED_SET);
            }

            let contact_sim = if set_id == AWAKE_SET && contact.color_index != NULL_INDEX {
                &self.constraint_graph.colors[contact.color_index as usize].contact_sims
                    [contact.local_index as usize]
            } else {
                &self.solver_sets[set_id as usize].contact_sims[contact.local_index as usize]
            };
            debug_assert!(contact_sim.contact_id == contact_index);
            debug_assert!(contact_sim.body_id_a == contact.edges[0].body_id);
            debug_assert!(contact_sim.body_id_b == contact.edges[1].body_id);

            let sim_touching = contact_sim.sim_flags & contact_flags::SIM_TOUCHING != 0;
            debug_assert!(touching == sim_touching);

            debug_assert!(
                0 <= contact_sim.manifold.point_count && contact_sim.manifold.point_count <= 2
            );
        }

        let contact_id_count = self.contact_id_pool.id_count();
        debug_assert!(allocated_contact_count == contact_id_count);
        let _ = allocated_contact_count;
    }
}
