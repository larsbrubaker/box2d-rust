// Narrow-phase contact update from contact.c: manifold dispatch and
// b2UpdateContact.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{contact_flags, ContactSim};
use crate::collision::LocalManifold;
use crate::collision::Manifold;
use crate::constants::linear_slop;
use crate::distance::SimplexCache;
use crate::id::ShapeId;
use crate::manifold::{
    collide_capsule_and_circle, collide_capsules, collide_chain_segment_and_capsule,
    collide_chain_segment_and_circle, collide_chain_segment_and_polygon, collide_circles,
    collide_polygon_and_capsule, collide_polygon_and_circle, collide_polygons,
    collide_segment_and_capsule, collide_segment_and_circle, collide_segment_and_polygon,
};
use crate::math_functions::{
    add, inv_mul_world_transforms, max_float, offset_pos, rotate_vector, sub, sub_pos, Transform,
    Vec2, WorldTransform,
};
use crate::shape::Shape;
use crate::types::{FrictionCallback, RestitutionCallback};
use crate::world::{PreSolveFcn, World};

/// Manifold function dispatch over the primary shape pair. The caller must
/// pass shapes in primary order (create_contact guarantees this).
/// (b2ManifoldFcn dispatch through s_registers)
fn compute_manifold(
    shape_a: &Shape,
    shape_b: &Shape,
    xf: Transform,
    cache: &mut SimplexCache,
) -> LocalManifold {
    use crate::collision::ShapeGeometry;
    match (&shape_a.geometry, &shape_b.geometry) {
        (ShapeGeometry::Circle(a), ShapeGeometry::Circle(b)) => collide_circles(a, b, xf),
        (ShapeGeometry::Capsule(a), ShapeGeometry::Circle(b)) => {
            collide_capsule_and_circle(a, b, xf)
        }
        (ShapeGeometry::Capsule(a), ShapeGeometry::Capsule(b)) => collide_capsules(a, b, xf),
        (ShapeGeometry::Polygon(a), ShapeGeometry::Circle(b)) => {
            collide_polygon_and_circle(a, b, xf)
        }
        (ShapeGeometry::Polygon(a), ShapeGeometry::Capsule(b)) => {
            collide_polygon_and_capsule(a, b, xf)
        }
        (ShapeGeometry::Polygon(a), ShapeGeometry::Polygon(b)) => collide_polygons(a, b, xf),
        (ShapeGeometry::Segment(a), ShapeGeometry::Circle(b)) => {
            collide_segment_and_circle(a, b, xf)
        }
        (ShapeGeometry::Segment(a), ShapeGeometry::Capsule(b)) => {
            collide_segment_and_capsule(a, b, xf)
        }
        (ShapeGeometry::Segment(a), ShapeGeometry::Polygon(b)) => {
            collide_segment_and_polygon(a, b, xf)
        }
        (ShapeGeometry::ChainSegment(a), ShapeGeometry::Circle(b)) => {
            collide_chain_segment_and_circle(a, b, xf)
        }
        (ShapeGeometry::ChainSegment(a), ShapeGeometry::Capsule(b)) => {
            collide_chain_segment_and_capsule(a, b, xf, cache)
        }
        (ShapeGeometry::ChainSegment(a), ShapeGeometry::Polygon(b)) => {
            collide_chain_segment_and_polygon(a, b, xf, cache)
        }
        // Contacts are only created for registered primary pairs, so this is
        // unreachable (the C equivalent would call a null function pointer).
        _ => unreachable!("no manifold function for this shape pair"),
    }
}

/// The world data update_contact needs, copied out so the caller can hold
/// `&mut ContactSim` and `&Shape` borrows at the same time (the C version
/// reads these through b2World*).
#[derive(Clone, Copy)]
pub struct ContactUpdateContext {
    pub friction_callback: FrictionCallback,
    pub restitution_callback: RestitutionCallback,
    pub pre_solve_fcn: Option<PreSolveFcn>,
    pub pre_solve_context: u64,
    pub world_id: u16,
    pub enable_speculative: bool,
}

impl ContactUpdateContext {
    pub fn new(world: &World) -> ContactUpdateContext {
        ContactUpdateContext {
            friction_callback: world.friction_callback.unwrap(),
            restitution_callback: world.restitution_callback.unwrap(),
            pre_solve_fcn: world.pre_solve_fcn,
            pre_solve_context: world.pre_solve_context,
            world_id: world.world_id,
            enable_speculative: world.enable_speculative,
        }
    }
}

/// Update the contact manifold and touching status.
/// Note: do not assume the shape AABBs are overlapping or are valid.
/// (b2UpdateContact)
#[allow(clippy::too_many_arguments)]
pub fn update_contact(
    ctx: &ContactUpdateContext,
    contact_sim: &mut ContactSim,
    shape_a: &Shape,
    transform_a: WorldTransform,
    center_offset_a: Vec2,
    shape_b: &Shape,
    transform_b: WorldTransform,
    center_offset_b: Vec2,
) -> bool {
    // Save old manifold
    let mut old_manifold = contact_sim.manifold;

    // Compute the manifold in frame A, then marshal it to world anchors
    // relative to each shape origin. The relative pose differences the two
    // world positions before the narrow phase runs in frame A, so precision is
    // retained far from the origin.
    // anchorB = worldPoint - pB = rot(qA, localAnchorA) + pA - pB = anchorA + (pA - pB)
    let relative_transform = inv_mul_world_transforms(transform_a, transform_b);
    let local = compute_manifold(shape_a, shape_b, relative_transform, &mut contact_sim.cache);

    contact_sim.manifold = Manifold::default();
    contact_sim.manifold.normal = rotate_vector(transform_a.q, local.normal);
    contact_sim.manifold.point_count = local.point_count;

    let origin_delta = sub_pos(transform_a.p, transform_b.p);
    for i in 0..local.point_count as usize {
        let mp = &mut contact_sim.manifold.points[i];
        mp.anchor_a = rotate_vector(transform_a.q, local.points[i].point);
        mp.anchor_b = add(mp.anchor_a, origin_delta);
        mp.separation = local.points[i].separation;
        mp.id = local.points[i].id;
    }

    // Keep these updated in case the values on the shapes are modified
    contact_sim.friction = (ctx.friction_callback)(
        shape_a.material.friction,
        shape_a.material.user_material_id,
        shape_b.material.friction,
        shape_b.material.user_material_id,
    );
    contact_sim.restitution = (ctx.restitution_callback)(
        shape_a.material.restitution,
        shape_a.material.user_material_id,
        shape_b.material.restitution,
        shape_b.material.user_material_id,
    );

    if shape_a.material.rolling_resistance > 0.0 || shape_b.material.rolling_resistance > 0.0 {
        let radius_a = shape_a.radius();
        let radius_b = shape_b.radius();
        let max_radius = max_float(radius_a, radius_b);
        contact_sim.rolling_resistance = max_float(
            shape_a.material.rolling_resistance,
            shape_b.material.rolling_resistance,
        ) * max_radius;
    } else {
        contact_sim.rolling_resistance = 0.0;
    }

    contact_sim.tangent_speed = shape_a.material.tangent_speed + shape_b.material.tangent_speed;

    let mut point_count = contact_sim.manifold.point_count;
    let mut touching = point_count > 0;

    if let (true, Some(pre_solve_fcn), true) = (
        touching,
        ctx.pre_solve_fcn,
        (contact_sim.sim_flags & contact_flags::SIM_ENABLE_PRE_SOLVE_EVENTS) != 0,
    ) {
        let shape_id_a = ShapeId {
            index1: shape_a.id + 1,
            world0: ctx.world_id,
            generation: shape_a.generation,
        };
        let shape_id_b = ShapeId {
            index1: shape_b.id + 1,
            world0: ctx.world_id,
            generation: shape_b.generation,
        };

        let manifold = &contact_sim.manifold;
        let mut best_separation = manifold.points[0].separation;
        let mut best_point = offset_pos(transform_a.p, manifold.points[0].anchor_a);

        // Get deepest point
        for i in 1..manifold.point_count as usize {
            let separation = manifold.points[i].separation;
            if separation < best_separation {
                best_separation = separation;
                best_point = offset_pos(transform_a.p, manifold.points[i].anchor_a);
            }
        }

        // this call assumes thread safety
        touching = pre_solve_fcn(
            shape_id_a,
            shape_id_b,
            best_point,
            manifold.normal,
            ctx.pre_solve_context,
        );
        if !touching {
            // disable contact
            point_count = 0;
            contact_sim.manifold.point_count = 0;
        }
    }

    // This flag is for testing
    if !ctx.enable_speculative && point_count == 2 {
        if contact_sim.manifold.points[0].separation > 1.5 * linear_slop() {
            contact_sim.manifold.points[0] = contact_sim.manifold.points[1];
            contact_sim.manifold.point_count = 1;
        } else if contact_sim.manifold.points[1].separation > 1.5 * linear_slop() {
            contact_sim.manifold.point_count = 1;
        }

        point_count = contact_sim.manifold.point_count;
    }

    if touching && (shape_a.enable_hit_events || shape_b.enable_hit_events) {
        contact_sim.sim_flags |= contact_flags::SIM_ENABLE_HIT_EVENT;
    } else {
        contact_sim.sim_flags &= !contact_flags::SIM_ENABLE_HIT_EVENT;
    }

    if point_count > 0 {
        contact_sim.manifold.rolling_impulse = old_manifold.rolling_impulse;
    }

    // Match old contact ids to new contact ids and copy the
    // stored impulses to warm start the solver.
    let mut unmatched_count = 0;
    for i in 0..point_count as usize {
        let mp2 = &mut contact_sim.manifold.points[i];

        // shift anchors to be center of mass relative
        mp2.anchor_a = sub(mp2.anchor_a, center_offset_a);
        mp2.anchor_b = sub(mp2.anchor_b, center_offset_b);

        mp2.normal_impulse = 0.0;
        mp2.tangent_impulse = 0.0;
        mp2.total_normal_impulse = 0.0;
        mp2.normal_velocity = 0.0;
        mp2.persisted = false;

        let id2 = mp2.id;

        for j in 0..old_manifold.point_count as usize {
            let mp1 = &mut old_manifold.points[j];

            if mp1.id == id2 {
                mp2.normal_impulse = mp1.normal_impulse;
                mp2.tangent_impulse = mp1.tangent_impulse;
                mp2.persisted = true;

                // clear old impulse
                mp1.normal_impulse = 0.0;
                mp1.tangent_impulse = 0.0;
                break;
            }
        }

        unmatched_count += if mp2.persisted { 0 } else { 1 };
    }

    // The C `#if 0` block distributing unmatched impulses is not ported.
    let _ = unmatched_count;

    if touching {
        contact_sim.sim_flags |= contact_flags::SIM_TOUCHING;
    } else {
        contact_sim.sim_flags &= !contact_flags::SIM_TOUCHING;
    }

    touching
}
