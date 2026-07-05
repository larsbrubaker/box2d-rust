// Port of contact_solver.h/.c: the scalar contact constraint kernels.
//
// The C file has scalar "overflow" kernels plus SIMD "wide" kernels used for
// graph-color contacts (with a per-lane scalar emulation when no SIMD target
// is enabled). The wide kernels compute exactly the same per-contact float
// sequence as the scalar kernels because bodies within a color are disjoint,
// so this serial port routes every color through the scalar kernels; the
// solver iterates colors in order (overflow last, matching the C stage
// layout).
//
// The C stores per-color constraint arrays in arena scratch pointed to by
// b2GraphColor; the Rust solve pass owns them as local Vecs and passes
// parallel slices in.
//
// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translate and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: called by the solver slice.

use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::contact::ContactSim;
use crate::core::NULL_INDEX;
use crate::math_functions::{
    add, clamp_float, cross, cross_sv, dot, max_float, mul_add, mul_sub, mul_sv, right_perp,
    rotate_vector, sub, Vec2, VEC2_ZERO,
};
use crate::solver::{Softness, StepContext};

/// (b2ContactConstraintPoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactConstraintPoint {
    pub anchor_a: Vec2,
    pub anchor_b: Vec2,
    pub base_separation: f32,
    pub relative_velocity: f32,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub total_normal_impulse: f32,
    pub normal_mass: f32,
    pub tangent_mass: f32,
}

/// (b2ContactConstraint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactConstraint {
    /// base-1, 0 for null
    pub index_a: i32,
    pub index_b: i32,
    pub points: [ContactConstraintPoint; 2],
    pub normal: Vec2,
    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_i_a: f32,
    pub inv_i_b: f32,
    pub friction: f32,
    pub restitution: f32,
    pub tangent_speed: f32,
    pub rolling_resistance: f32,
    pub rolling_mass: f32,
    pub rolling_impulse: f32,
    pub softness: Softness,
    pub point_count: i32,
}

/// Build the constraints for one color's touching contacts. The constraints
/// slice is parallel to the contacts slice.
/// (b2PrepareContacts_Overflow / per-lane b2PrepareContactsTask)
pub fn prepare_contacts(
    constraints: &mut [ContactConstraint],
    contacts: &[ContactSim],
    states: &[BodyState],
    context: &StepContext,
) {
    debug_assert!(constraints.len() == contacts.len());

    // Stiffer for static contacts to avoid bodies getting pushed through the
    // ground
    let contact_softness = context.contact_softness;
    let static_softness = context.static_softness;

    let warm_start_scale = if context.enable_warm_starting {
        1.0
    } else {
        0.0
    };

    for (constraint, contact_sim) in constraints.iter_mut().zip(contacts.iter()) {
        let manifold = &contact_sim.manifold;
        let point_count = manifold.point_count;

        debug_assert!(0 < point_count && point_count <= 2);

        let index_a = contact_sim.body_sim_index_a;
        let index_b = contact_sim.body_sim_index_b;

        // 0 is null
        constraint.index_a = index_a + 1;
        constraint.index_b = index_b + 1;
        constraint.normal = manifold.normal;
        constraint.friction = contact_sim.friction;
        constraint.restitution = contact_sim.restitution;
        constraint.rolling_resistance = contact_sim.rolling_resistance;
        constraint.rolling_impulse = warm_start_scale * manifold.rolling_impulse;
        constraint.tangent_speed = contact_sim.tangent_speed;
        constraint.point_count = point_count;

        let mut v_a = VEC2_ZERO;
        let mut w_a = 0.0;
        let m_a = contact_sim.inv_mass_a;
        let i_a = contact_sim.inv_i_a;
        if index_a != NULL_INDEX {
            let state_a = &states[index_a as usize];
            v_a = state_a.linear_velocity;
            w_a = state_a.angular_velocity;
        }

        let mut v_b = VEC2_ZERO;
        let mut w_b = 0.0;
        let m_b = contact_sim.inv_mass_b;
        let i_b = contact_sim.inv_i_b;
        if index_b != NULL_INDEX {
            let state_b = &states[index_b as usize];
            v_b = state_b.linear_velocity;
            w_b = state_b.angular_velocity;
        }

        if index_a == NULL_INDEX || index_b == NULL_INDEX {
            constraint.softness = static_softness;
        } else {
            constraint.softness = contact_softness;
        }

        // copy mass into constraint to avoid cache misses during sub-stepping
        constraint.inv_mass_a = m_a;
        constraint.inv_i_a = i_a;
        constraint.inv_mass_b = m_b;
        constraint.inv_i_b = i_b;

        {
            let k = i_a + i_b;
            constraint.rolling_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        }

        let normal = constraint.normal;
        let tangent = right_perp(constraint.normal);

        for j in 0..point_count as usize {
            let mp = &manifold.points[j];
            let cp = &mut constraint.points[j];

            cp.normal_impulse = warm_start_scale * mp.normal_impulse;
            cp.tangent_impulse = warm_start_scale * mp.tangent_impulse;
            cp.total_normal_impulse = 0.0;

            let r_a = mp.anchor_a;
            let r_b = mp.anchor_b;

            cp.anchor_a = r_a;
            cp.anchor_b = r_b;
            cp.base_separation = mp.separation - dot(sub(r_b, r_a), normal);

            let rn_a = cross(r_a, normal);
            let rn_b = cross(r_b, normal);
            let k_normal = m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;
            cp.normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            let rt_a = cross(r_a, tangent);
            let rt_b = cross(r_b, tangent);
            let k_tangent = m_a + m_b + i_a * rt_a * rt_a + i_b * rt_b * rt_b;
            cp.tangent_mass = if k_tangent > 0.0 {
                1.0 / k_tangent
            } else {
                0.0
            };

            // Save relative velocity for restitution
            let vr_a = add(v_a, cross_sv(w_a, r_a));
            let vr_b = add(v_b, cross_sv(w_b, r_b));
            cp.relative_velocity = dot(normal, sub(vr_b, vr_a));
        }
    }
}

/// (b2WarmStartContacts_Overflow / per-lane b2WarmStartContactsTask)
pub fn warm_start_contacts(constraints: &mut [ContactConstraint], states: &mut [BodyState]) {
    for constraint in constraints.iter_mut() {
        let index_a = constraint.index_a - 1;
        let index_b = constraint.index_b - 1;

        // This is a dummy state to represent a static body because static
        // bodies don't have a solver body.
        let mut state_a = if index_a == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_a as usize]
        };
        let mut state_b = if index_b == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_b as usize]
        };

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        let m_a = constraint.inv_mass_a;
        let i_a = constraint.inv_i_a;
        let m_b = constraint.inv_mass_b;
        let i_b = constraint.inv_i_b;

        let normal = constraint.normal;
        let tangent = right_perp(constraint.normal);
        let point_count = constraint.point_count;

        for j in 0..point_count as usize {
            let cp = &mut constraint.points[j];

            // fixed anchors
            let r_a = cp.anchor_a;
            let r_b = cp.anchor_b;

            let p = add(
                mul_sv(cp.normal_impulse, normal),
                mul_sv(cp.tangent_impulse, tangent),
            );

            cp.total_normal_impulse += cp.normal_impulse;

            w_a -= i_a * cross(r_a, p);
            v_a = mul_add(v_a, -m_a, p);
            w_b += i_b * cross(r_b, p);
            v_b = mul_add(v_b, m_b, p);
        }

        w_a -= i_a * constraint.rolling_impulse;
        w_b += i_b * constraint.rolling_impulse;

        if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
            state_a.linear_velocity = v_a;
            state_a.angular_velocity = w_a;
            states[index_a as usize] = state_a;
        }

        if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
            state_b.linear_velocity = v_b;
            state_b.angular_velocity = w_b;
            states[index_b as usize] = state_b;
        }
    }
}

/// (b2SolveContacts_Overflow / per-lane b2SolveContactsTask)
pub fn solve_contacts(
    constraints: &mut [ContactConstraint],
    states: &mut [BodyState],
    context: &StepContext,
    use_bias: bool,
) {
    let inv_h = context.inv_h;
    let contact_speed = context.contact_speed;

    for constraint in constraints.iter_mut() {
        let m_a = constraint.inv_mass_a;
        let i_a = constraint.inv_i_a;
        let m_b = constraint.inv_mass_b;
        let i_b = constraint.inv_i_b;

        let index_a = constraint.index_a - 1;
        let index_b = constraint.index_b - 1;

        // This is a dummy body to represent a static body since static bodies
        // don't have a solver body.
        let mut state_a = if index_a == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_a as usize]
        };
        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let dq_a = state_a.delta_rotation;

        let mut state_b = if index_b == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_b as usize]
        };
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;
        let dq_b = state_b.delta_rotation;

        let dp = sub(state_b.delta_position, state_a.delta_position);

        let normal = constraint.normal;
        let tangent = right_perp(normal);
        let friction = constraint.friction;
        let softness = constraint.softness;

        let point_count = constraint.point_count;
        let mut total_normal_impulse = 0.0;

        // Non-penetration
        for j in 0..point_count as usize {
            let cp = &mut constraint.points[j];

            // fixed anchor points
            let r_a = cp.anchor_a;
            let r_b = cp.anchor_b;

            // compute current separation
            // this is subject to round-off error if the anchor is far from the
            // body center of mass
            let ds = add(dp, sub(rotate_vector(dq_b, r_b), rotate_vector(dq_a, r_a)));
            let s = cp.base_separation + dot(ds, normal);

            let mut velocity_bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;
            if s > 0.0 {
                // speculative bias
                velocity_bias = s * inv_h;
            } else if use_bias {
                velocity_bias =
                    max_float(softness.mass_scale * softness.bias_rate * s, -contact_speed);
                mass_scale = softness.mass_scale;
                impulse_scale = softness.impulse_scale;
            }

            // relative normal velocity at contact
            let vr_a = add(v_a, cross_sv(w_a, r_a));
            let vr_b = add(v_b, cross_sv(w_b, r_b));
            let vn = dot(sub(vr_b, vr_a), normal);

            // incremental normal impulse
            let mut impulse = -cp.normal_mass * (mass_scale * vn + velocity_bias)
                - impulse_scale * cp.normal_impulse;

            // clamp the accumulated impulse
            let new_impulse = max_float(cp.normal_impulse + impulse, 0.0);
            impulse = new_impulse - cp.normal_impulse;
            cp.normal_impulse = new_impulse;
            cp.total_normal_impulse += impulse;

            total_normal_impulse += new_impulse;

            // apply normal impulse
            let p = mul_sv(impulse, normal);
            v_a = mul_sub(v_a, m_a, p);
            w_a -= i_a * cross(r_a, p);

            v_b = mul_add(v_b, m_b, p);
            w_b += i_b * cross(r_b, p);
        }

        if !use_bias {
            // Friction
            for j in 0..point_count as usize {
                let cp = &mut constraint.points[j];

                // fixed anchor points
                let r_a = cp.anchor_a;
                let r_b = cp.anchor_b;

                // relative tangent velocity at contact
                let vr_b = add(v_b, cross_sv(w_b, r_b));
                let vr_a = add(v_a, cross_sv(w_a, r_a));

                // vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
                //    = dot(vrB - vrA, tangent) - (sA + sB)
                let vt = dot(sub(vr_b, vr_a), tangent) - constraint.tangent_speed;

                // incremental tangent impulse
                let mut impulse = cp.tangent_mass * (-vt);

                // clamp the accumulated force
                let max_friction = friction * cp.normal_impulse;
                let new_impulse =
                    clamp_float(cp.tangent_impulse + impulse, -max_friction, max_friction);
                impulse = new_impulse - cp.tangent_impulse;
                cp.tangent_impulse = new_impulse;

                // apply tangent impulse
                let p = mul_sv(impulse, tangent);
                v_a = mul_sub(v_a, m_a, p);
                w_a -= i_a * cross(r_a, p);
                v_b = mul_add(v_b, m_b, p);
                w_b += i_b * cross(r_b, p);
            }

            // Rolling resistance
            {
                let mut delta_lambda = -constraint.rolling_mass * (w_b - w_a);
                let lambda = constraint.rolling_impulse;
                let max_lambda = constraint.rolling_resistance * total_normal_impulse;
                constraint.rolling_impulse =
                    clamp_float(lambda + delta_lambda, -max_lambda, max_lambda);
                delta_lambda = constraint.rolling_impulse - lambda;

                w_a -= i_a * delta_lambda;
                w_b += i_b * delta_lambda;
            }
        }

        if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
            state_a.linear_velocity = v_a;
            state_a.angular_velocity = w_a;
            states[index_a as usize] = state_a;
        }

        if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
            state_b.linear_velocity = v_b;
            state_b.angular_velocity = w_b;
            states[index_b as usize] = state_b;
        }
    }
}

/// (b2ApplyRestitution_Overflow / per-lane b2ApplyRestitutionTask)
pub fn apply_restitution(
    constraints: &mut [ContactConstraint],
    states: &mut [BodyState],
    context: &StepContext,
) {
    let threshold = context.restitution_threshold;

    for constraint in constraints.iter_mut() {
        let restitution = constraint.restitution;
        if restitution == 0.0 {
            continue;
        }

        let m_a = constraint.inv_mass_a;
        let i_a = constraint.inv_i_a;
        let m_b = constraint.inv_mass_b;
        let i_b = constraint.inv_i_b;

        let index_a = constraint.index_a - 1;
        let index_b = constraint.index_b - 1;

        // dummy state to represent a static body
        let mut state_a = if index_a == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_a as usize]
        };
        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;

        let mut state_b = if index_b == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_b as usize]
        };
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        let normal = constraint.normal;
        let point_count = constraint.point_count;

        // it is possible to get more accurate restitution by iterating
        // this only makes a difference if there are two contact points
        for j in 0..point_count as usize {
            let cp = &mut constraint.points[j];

            // if the normal impulse is zero then there was no collision
            // this skips speculative contact points that didn't generate an
            // impulse. The max normal impulse is used in case there was a
            // collision that moved away within the sub-step process
            if cp.relative_velocity > -threshold || cp.total_normal_impulse == 0.0 {
                continue;
            }

            // fixed anchor points
            let r_a = cp.anchor_a;
            let r_b = cp.anchor_b;

            // relative normal velocity at contact
            let vr_b = add(v_b, cross_sv(w_b, r_b));
            let vr_a = add(v_a, cross_sv(w_a, r_a));
            let vn = dot(sub(vr_b, vr_a), normal);

            // compute normal impulse
            let mut impulse = -cp.normal_mass * (vn + restitution * cp.relative_velocity);

            // clamp the accumulated impulse
            let new_impulse = max_float(cp.normal_impulse + impulse, 0.0);
            impulse = new_impulse - cp.normal_impulse;
            cp.normal_impulse = new_impulse;
            cp.total_normal_impulse += impulse;

            // apply contact impulse
            let p = mul_sv(impulse, normal);
            v_a = mul_sub(v_a, m_a, p);
            w_a -= i_a * cross(r_a, p);
            v_b = mul_add(v_b, m_b, p);
            w_b += i_b * cross(r_b, p);
        }

        if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
            state_a.linear_velocity = v_a;
            state_a.angular_velocity = w_a;
            states[index_a as usize] = state_a;
        }

        if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
            state_b.linear_velocity = v_b;
            state_b.angular_velocity = w_b;
            states[index_b as usize] = state_b;
        }
    }
}

/// (b2StoreImpulses_Overflow / per-lane b2StoreImpulsesTask)
pub fn store_impulses(constraints: &[ContactConstraint], contacts: &mut [ContactSim]) {
    for (constraint, contact) in constraints.iter().zip(contacts.iter_mut()) {
        let manifold = &mut contact.manifold;
        let point_count = manifold.point_count;

        for j in 0..point_count as usize {
            manifold.points[j].normal_impulse = constraint.points[j].normal_impulse;
            manifold.points[j].tangent_impulse = constraint.points[j].tangent_impulse;
            manifold.points[j].total_normal_impulse = constraint.points[j].total_normal_impulse;
            manifold.points[j].normal_velocity = constraint.points[j].relative_velocity;
        }

        manifold.rolling_impulse = constraint.rolling_impulse;
    }
}
