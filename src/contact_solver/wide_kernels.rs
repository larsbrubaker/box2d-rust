// Wide (SIMD) contact constraint kernels, ported from the b2*ContactsTask
// functions in contact_solver.c. These run the graph-color contacts four lanes
// at a time; the overflow color keeps the scalar kernels in
// contact_solver.rs. Bodies within a graph color are disjoint, so gather /
// scatter over lane indices is race-free and the per-lane arithmetic is
// bit-identical to the scalar path (see wide.rs for the op-composition rules).
//
// A color's constraints are stored in a `Vec<ContactConstraintWide>` sized in
// blocks of SIMD_WIDTH (ceil(count / 4)). The Vec is zero-initialized so the
// dead tail lanes of the last block carry null indices (stored 0) and zero
// data; gather returns identity for them and scatter skips them, so they have
// no effect. This mirrors the C, which zeroes remainder lanes in solver setup
// and never writes them in b2PrepareContactsTask.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: called by the solver slice for graph colors.
//
// The prepare/store lane writes index a block by (i / 4, i % 4); the range
// loops mirror the C's lane fill and are clearer than an iterator here.
#![allow(clippy::needless_range_loop)]

use super::wide::{
    cross_w, dot_w, gather_bodies, rotate_vector_w, scatter_bodies, FloatW, Vec2W, SIMD_WIDTH,
};
use crate::body::BodyState;
use crate::contact::ContactSim;
use crate::core::NULL_INDEX;
use crate::math_functions::max_float;
use crate::math_functions::min_float;
use crate::math_functions::{cross, dot, right_perp, sub, Vec2};
use crate::solver::{make_soft, StepContext};

/// Struct-of-lane-arrays contact constraint for four contacts. (b2ContactConstraintWide)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactConstraintWide {
    /// base-1 body sim indices, 0 for null
    pub index_a: [i32; SIMD_WIDTH],
    pub index_b: [i32; SIMD_WIDTH],

    pub inv_mass_a: FloatW,
    pub inv_mass_b: FloatW,
    pub inv_i_a: FloatW,
    pub inv_i_b: FloatW,
    pub normal: Vec2W,
    pub friction: FloatW,
    pub tangent_speed: FloatW,
    pub rolling_resistance: FloatW,
    pub rolling_mass: FloatW,
    pub rolling_impulse: FloatW,
    pub bias_rate: FloatW,
    pub mass_scale: FloatW,
    pub impulse_scale: FloatW,
    pub anchor_a1: Vec2W,
    pub anchor_b1: Vec2W,
    pub normal_mass1: FloatW,
    pub tangent_mass1: FloatW,
    pub base_separation1: FloatW,
    pub normal_impulse1: FloatW,
    pub total_normal_impulse1: FloatW,
    pub tangent_impulse1: FloatW,
    pub anchor_a2: Vec2W,
    pub anchor_b2: Vec2W,
    pub base_separation2: FloatW,
    pub normal_impulse2: FloatW,
    pub total_normal_impulse2: FloatW,
    pub tangent_impulse2: FloatW,
    pub normal_mass2: FloatW,
    pub tangent_mass2: FloatW,
    pub restitution: FloatW,
    pub relative_velocity1: FloatW,
    pub relative_velocity2: FloatW,
}

/// Number of wide blocks needed for `contact_count` contacts.
#[inline]
pub fn wide_block_count(contact_count: usize) -> usize {
    contact_count.div_ceil(SIMD_WIDTH)
}

/// Build the wide constraints for one color's touching contacts. `wide` is
/// sized `wide_block_count(contacts.len())` and zero-initialized; only live
/// lanes are written. (b2PrepareContactsTask, per color)
#[allow(clippy::too_many_arguments)]
pub fn prepare_contacts_wide(
    wide: &mut [ContactConstraintWide],
    contacts: &[ContactSim],
    states: &[BodyState],
    context: &StepContext,
    enable_softening: bool,
    contact_hertz: f32,
    contact_damping_ratio: f32,
) {
    debug_assert!(wide.len() == wide_block_count(contacts.len()));

    // Stiffer for static contacts to avoid bodies getting pushed through the
    // ground
    let contact_softness = context.contact_softness;
    let static_softness = context.static_softness;

    let warm_start_scale = if context.enable_warm_starting {
        1.0
    } else {
        0.0
    };

    for (i, contact_sim) in contacts.iter().enumerate() {
        let block = i / SIMD_WIDTH;
        let lane = i % SIMD_WIDTH;
        let constraint = &mut wide[block];

        let manifold = &contact_sim.manifold;
        let point_count = manifold.point_count;
        debug_assert!(0 < point_count && point_count <= 2);

        let index_a = contact_sim.body_sim_index_a;
        let index_b = contact_sim.body_sim_index_b;

        // 0 for null
        constraint.index_a[lane] = index_a + 1;
        constraint.index_b[lane] = index_b + 1;

        let mut v_a = Vec2 { x: 0.0, y: 0.0 };
        let mut w_a = 0.0;
        let m_a = contact_sim.inv_mass_a;
        let i_a = contact_sim.inv_i_a;
        if index_a != NULL_INDEX {
            let state_a = &states[index_a as usize];
            v_a = state_a.linear_velocity;
            w_a = state_a.angular_velocity;
        }

        let mut v_b = Vec2 { x: 0.0, y: 0.0 };
        let mut w_b = 0.0;
        let m_b = contact_sim.inv_mass_b;
        let i_b = contact_sim.inv_i_b;
        if index_b != NULL_INDEX {
            let state_b = &states[index_b as usize];
            v_b = state_b.linear_velocity;
            w_b = state_b.angular_velocity;
        }

        constraint.inv_mass_a.0[lane] = m_a;
        constraint.inv_mass_b.0[lane] = m_b;
        constraint.inv_i_a.0[lane] = i_a;
        constraint.inv_i_b.0[lane] = i_b;

        {
            let k = i_a + i_b;
            constraint.rolling_mass.0[lane] = if k > 0.0 { 1.0 / k } else { 0.0 };
        }

        // Soft contact behavior. The overflow scalar path uses static/contact
        // softness only; the graph-color wide path additionally supports the
        // experimental per-contact softening feature, matching the C.
        let soft = if index_a == NULL_INDEX || index_b == NULL_INDEX {
            static_softness
        } else if enable_softening {
            let contact_hertz = min_float(contact_hertz, 0.125 * context.inv_h);
            let ratio = if m_a < m_b {
                max_float(0.5, m_a / m_b)
            } else if m_b < m_a {
                max_float(0.5, m_b / m_a)
            } else {
                1.0
            };
            make_soft(
                ratio * contact_hertz,
                ratio * contact_damping_ratio,
                context.h,
            )
        } else {
            contact_softness
        };

        let normal = manifold.normal;
        constraint.normal.x.0[lane] = normal.x;
        constraint.normal.y.0[lane] = normal.y;

        constraint.friction.0[lane] = contact_sim.friction;
        constraint.tangent_speed.0[lane] = contact_sim.tangent_speed;
        constraint.restitution.0[lane] = contact_sim.restitution;
        constraint.rolling_resistance.0[lane] = contact_sim.rolling_resistance;
        constraint.rolling_impulse.0[lane] = warm_start_scale * manifold.rolling_impulse;

        constraint.bias_rate.0[lane] = soft.bias_rate;
        constraint.mass_scale.0[lane] = soft.mass_scale;
        constraint.impulse_scale.0[lane] = soft.impulse_scale;

        let tangent = right_perp(normal);

        // point 1
        {
            let mp = &manifold.points[0];
            let r_a = mp.anchor_a;
            let r_b = mp.anchor_b;

            constraint.anchor_a1.x.0[lane] = r_a.x;
            constraint.anchor_a1.y.0[lane] = r_a.y;
            constraint.anchor_b1.x.0[lane] = r_b.x;
            constraint.anchor_b1.y.0[lane] = r_b.y;

            constraint.base_separation1.0[lane] = mp.separation - dot(sub(r_b, r_a), normal);

            constraint.normal_impulse1.0[lane] = warm_start_scale * mp.normal_impulse;
            constraint.tangent_impulse1.0[lane] = warm_start_scale * mp.tangent_impulse;
            constraint.total_normal_impulse1.0[lane] = 0.0;

            let rn_a = cross(r_a, normal);
            let rn_b = cross(r_b, normal);
            let k_normal = m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;
            constraint.normal_mass1.0[lane] = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            let rt_a = cross(r_a, tangent);
            let rt_b = cross(r_b, tangent);
            let k_tangent = m_a + m_b + i_a * rt_a * rt_a + i_b * rt_b * rt_b;
            constraint.tangent_mass1.0[lane] = if k_tangent > 0.0 {
                1.0 / k_tangent
            } else {
                0.0
            };

            let vr_a = crate::math_functions::add(v_a, crate::math_functions::cross_sv(w_a, r_a));
            let vr_b = crate::math_functions::add(v_b, crate::math_functions::cross_sv(w_b, r_b));
            constraint.relative_velocity1.0[lane] = dot(normal, sub(vr_b, vr_a));
        }

        if point_count == 2 {
            let mp = &manifold.points[1];
            let r_a = mp.anchor_a;
            let r_b = mp.anchor_b;

            constraint.anchor_a2.x.0[lane] = r_a.x;
            constraint.anchor_a2.y.0[lane] = r_a.y;
            constraint.anchor_b2.x.0[lane] = r_b.x;
            constraint.anchor_b2.y.0[lane] = r_b.y;

            constraint.base_separation2.0[lane] = mp.separation - dot(sub(r_b, r_a), normal);

            constraint.normal_impulse2.0[lane] = warm_start_scale * mp.normal_impulse;
            constraint.tangent_impulse2.0[lane] = warm_start_scale * mp.tangent_impulse;
            constraint.total_normal_impulse2.0[lane] = 0.0;

            let rn_a = cross(r_a, normal);
            let rn_b = cross(r_b, normal);
            let k_normal = m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;
            constraint.normal_mass2.0[lane] = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            let rt_a = cross(r_a, tangent);
            let rt_b = cross(r_b, tangent);
            let k_tangent = m_a + m_b + i_a * rt_a * rt_a + i_b * rt_b * rt_b;
            constraint.tangent_mass2.0[lane] = if k_tangent > 0.0 {
                1.0 / k_tangent
            } else {
                0.0
            };

            let vr_a = crate::math_functions::add(v_a, crate::math_functions::cross_sv(w_a, r_a));
            let vr_b = crate::math_functions::add(v_b, crate::math_functions::cross_sv(w_b, r_b));
            constraint.relative_velocity2.0[lane] = dot(normal, sub(vr_b, vr_a));
        } else {
            // dummy data that has no effect
            constraint.base_separation2.0[lane] = 0.0;
            constraint.normal_impulse2.0[lane] = 0.0;
            constraint.tangent_impulse2.0[lane] = 0.0;
            constraint.total_normal_impulse2.0[lane] = 0.0;
            constraint.anchor_a2.x.0[lane] = 0.0;
            constraint.anchor_a2.y.0[lane] = 0.0;
            constraint.anchor_b2.x.0[lane] = 0.0;
            constraint.anchor_b2.y.0[lane] = 0.0;
            constraint.normal_mass2.0[lane] = 0.0;
            constraint.tangent_mass2.0[lane] = 0.0;
            constraint.relative_velocity2.0[lane] = 0.0;
        }
    }
}

/// (b2WarmStartContactsTask, per color)
pub fn warm_start_contacts_wide(wide: &mut [ContactConstraintWide], states: &mut [BodyState]) {
    for c in wide.iter_mut() {
        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let tangent_x = c.normal.y;
        let tangent_y = FloatW::zero().sub(c.normal.x);

        // point 1
        {
            let r_a = c.anchor_a1;
            let r_b = c.anchor_b1;

            let p = Vec2W {
                x: c.normal_impulse1
                    .mul(c.normal.x)
                    .add(c.tangent_impulse1.mul(tangent_x)),
                y: c.normal_impulse1
                    .mul(c.normal.y)
                    .add(c.tangent_impulse1.mul(tangent_y)),
            };
            b_a.w = b_a.w.mul_sub(c.inv_i_a, cross_w(r_a, p));
            b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, p.x);
            b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, p.y);
            b_b.w = b_b.w.mul_add(c.inv_i_b, cross_w(r_b, p));
            b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, p.x);
            b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, p.y);

            c.total_normal_impulse1 = c.total_normal_impulse1.add(c.normal_impulse1);
        }

        // point 2
        {
            let r_a = c.anchor_a2;
            let r_b = c.anchor_b2;

            let p = Vec2W {
                x: c.normal_impulse2
                    .mul(c.normal.x)
                    .add(c.tangent_impulse2.mul(tangent_x)),
                y: c.normal_impulse2
                    .mul(c.normal.y)
                    .add(c.tangent_impulse2.mul(tangent_y)),
            };
            b_a.w = b_a.w.mul_sub(c.inv_i_a, cross_w(r_a, p));
            b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, p.x);
            b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, p.y);
            b_b.w = b_b.w.mul_add(c.inv_i_b, cross_w(r_b, p));
            b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, p.x);
            b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, p.y);

            c.total_normal_impulse2 = c.total_normal_impulse2.add(c.normal_impulse2);
        }

        b_a.w = b_a.w.mul_sub(c.inv_i_a, c.rolling_impulse);
        b_b.w = b_b.w.mul_add(c.inv_i_b, c.rolling_impulse);

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// (b2SolveContactsTask, per color)
pub fn solve_contacts_wide(
    wide: &mut [ContactConstraintWide],
    states: &mut [BodyState],
    context: &StepContext,
    use_bias: bool,
) {
    let inv_h = FloatW::splat(context.inv_h);
    let contact_speed = FloatW::splat(-context.contact_speed);
    let one_w = FloatW::splat(1.0);
    let zero = FloatW::zero();

    for c in wide.iter_mut() {
        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let (bias_rate, mass_scale, impulse_scale) = if use_bias {
            (c.mass_scale.mul(c.bias_rate), c.mass_scale, c.impulse_scale)
        } else {
            (zero, one_w, zero)
        };

        let mut total_normal_impulse = zero;

        let dp = Vec2W {
            x: b_b.dp.x.sub(b_a.dp.x),
            y: b_b.dp.y.sub(b_a.dp.y),
        };

        // point 1 non-penetration constraint
        {
            let r_a = c.anchor_a1;
            let r_b = c.anchor_b1;

            let rs_a = rotate_vector_w(b_a.dq, r_a);
            let rs_b = rotate_vector_w(b_b.dq, r_b);

            let ds = Vec2W {
                x: dp.x.add(rs_b.x.sub(rs_a.x)),
                y: dp.y.add(rs_b.y.sub(rs_a.y)),
            };
            let s = dot_w(c.normal, ds).add(c.base_separation1);

            let mask = s.greater_than(zero);
            let spec_bias = s.mul(inv_h);
            let soft_bias = bias_rate.mul(s).max(contact_speed);
            let bias = FloatW::blend(soft_bias, spec_bias, mask);

            let point_mass_scale = FloatW::blend(mass_scale, one_w, mask);
            let point_impulse_scale = FloatW::blend(impulse_scale, zero, mask);

            let dvx = b_b
                .v
                .x
                .sub(b_b.w.mul(r_b.y))
                .sub(b_a.v.x.sub(b_a.w.mul(r_a.y)));
            let dvy = b_b
                .v
                .y
                .add(b_b.w.mul(r_b.x))
                .sub(b_a.v.y.add(b_a.w.mul(r_a.x)));
            let vn = dvx.mul(c.normal.x).add(dvy.mul(c.normal.y));

            let neg_impulse = c
                .normal_mass1
                .mul(point_mass_scale.mul(vn).add(bias))
                .add(point_impulse_scale.mul(c.normal_impulse1));

            let new_impulse = c.normal_impulse1.sub(neg_impulse).max(zero);
            let impulse = new_impulse.sub(c.normal_impulse1);
            c.normal_impulse1 = new_impulse;
            c.total_normal_impulse1 = c.total_normal_impulse1.add(impulse);

            total_normal_impulse = total_normal_impulse.add(new_impulse);

            let px = impulse.mul(c.normal.x);
            let py = impulse.mul(c.normal.y);

            b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, px);
            b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, py);
            b_a.w = b_a.w.mul_sub(c.inv_i_a, r_a.x.mul(py).sub(r_a.y.mul(px)));

            b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, px);
            b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, py);
            b_b.w = b_b.w.mul_add(c.inv_i_b, r_b.x.mul(py).sub(r_b.y.mul(px)));
        }

        // point 2 non-penetration constraint
        {
            let rs_a = rotate_vector_w(b_a.dq, c.anchor_a2);
            let rs_b = rotate_vector_w(b_b.dq, c.anchor_b2);

            let ds = Vec2W {
                x: dp.x.add(rs_b.x.sub(rs_a.x)),
                y: dp.y.add(rs_b.y.sub(rs_a.y)),
            };
            let s = dot_w(c.normal, ds).add(c.base_separation2);

            let mask = s.greater_than(zero);
            let spec_bias = s.mul(inv_h);
            let soft_bias = bias_rate.mul(s).max(contact_speed);
            let bias = FloatW::blend(soft_bias, spec_bias, mask);

            let point_mass_scale = FloatW::blend(mass_scale, one_w, mask);
            let point_impulse_scale = FloatW::blend(impulse_scale, zero, mask);

            let r_a = c.anchor_a2;
            let r_b = c.anchor_b2;

            let dvx = b_b
                .v
                .x
                .sub(b_b.w.mul(r_b.y))
                .sub(b_a.v.x.sub(b_a.w.mul(r_a.y)));
            let dvy = b_b
                .v
                .y
                .add(b_b.w.mul(r_b.x))
                .sub(b_a.v.y.add(b_a.w.mul(r_a.x)));
            let vn = dvx.mul(c.normal.x).add(dvy.mul(c.normal.y));

            let neg_impulse = c
                .normal_mass2
                .mul(point_mass_scale.mul(vn).add(bias))
                .add(point_impulse_scale.mul(c.normal_impulse2));

            let new_impulse = c.normal_impulse2.sub(neg_impulse).max(zero);
            let impulse = new_impulse.sub(c.normal_impulse2);
            c.normal_impulse2 = new_impulse;
            c.total_normal_impulse2 = c.total_normal_impulse2.add(impulse);

            total_normal_impulse = total_normal_impulse.add(new_impulse);

            let px = impulse.mul(c.normal.x);
            let py = impulse.mul(c.normal.y);

            b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, px);
            b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, py);
            b_a.w = b_a.w.mul_sub(c.inv_i_a, r_a.x.mul(py).sub(r_a.y.mul(px)));

            b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, px);
            b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, py);
            b_b.w = b_b.w.mul_add(c.inv_i_b, r_b.x.mul(py).sub(r_b.y.mul(px)));
        }

        if !use_bias {
            // Rolling resistance
            if !c.rolling_resistance.all_zero() {
                let delta_lambda = c.rolling_mass.mul(b_a.w.sub(b_b.w));
                let lambda = c.rolling_impulse;
                let max_lambda = c.rolling_resistance.mul(total_normal_impulse);
                c.rolling_impulse = lambda.add(delta_lambda).sym_clamp(max_lambda);
                let delta_lambda = c.rolling_impulse.sub(lambda);

                b_a.w = b_a.w.mul_sub(c.inv_i_a, delta_lambda);
                b_b.w = b_b.w.mul_add(c.inv_i_b, delta_lambda);
            }

            let tangent_x = c.normal.y;
            let tangent_y = zero.sub(c.normal.x);

            // point 1 friction constraint
            {
                let r_a = c.anchor_a1;
                let r_b = c.anchor_b1;

                let dvx = b_b
                    .v
                    .x
                    .sub(b_b.w.mul(r_b.y))
                    .sub(b_a.v.x.sub(b_a.w.mul(r_a.y)));
                let dvy = b_b
                    .v
                    .y
                    .add(b_b.w.mul(r_b.x))
                    .sub(b_a.v.y.add(b_a.w.mul(r_a.x)));
                let vt = dvx.mul(tangent_x).add(dvy.mul(tangent_y));
                let vt = vt.sub(c.tangent_speed);

                let neg_impulse = c.tangent_mass1.mul(vt);

                let max_friction = c.friction.mul(c.normal_impulse1);
                let new_impulse = c.tangent_impulse1.sub(neg_impulse);
                let new_impulse = zero.sub(max_friction).max(new_impulse.min(max_friction));
                let impulse = new_impulse.sub(c.tangent_impulse1);
                c.tangent_impulse1 = new_impulse;

                let px = impulse.mul(tangent_x);
                let py = impulse.mul(tangent_y);

                b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, px);
                b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, py);
                b_a.w = b_a.w.mul_sub(c.inv_i_a, r_a.x.mul(py).sub(r_a.y.mul(px)));

                b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, px);
                b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, py);
                b_b.w = b_b.w.mul_add(c.inv_i_b, r_b.x.mul(py).sub(r_b.y.mul(px)));
            }

            // point 2 friction constraint
            {
                let r_a = c.anchor_a2;
                let r_b = c.anchor_b2;

                let dvx = b_b
                    .v
                    .x
                    .sub(b_b.w.mul(r_b.y))
                    .sub(b_a.v.x.sub(b_a.w.mul(r_a.y)));
                let dvy = b_b
                    .v
                    .y
                    .add(b_b.w.mul(r_b.x))
                    .sub(b_a.v.y.add(b_a.w.mul(r_a.x)));
                let vt = dvx.mul(tangent_x).add(dvy.mul(tangent_y));
                let vt = vt.sub(c.tangent_speed);

                let neg_impulse = c.tangent_mass2.mul(vt);

                let max_friction = c.friction.mul(c.normal_impulse2);
                let new_impulse = c.tangent_impulse2.sub(neg_impulse);
                let new_impulse = zero.sub(max_friction).max(new_impulse.min(max_friction));
                let impulse = new_impulse.sub(c.tangent_impulse2);
                c.tangent_impulse2 = new_impulse;

                let px = impulse.mul(tangent_x);
                let py = impulse.mul(tangent_y);

                b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, px);
                b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, py);
                b_a.w = b_a.w.mul_sub(c.inv_i_a, r_a.x.mul(py).sub(r_a.y.mul(px)));

                b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, px);
                b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, py);
                b_b.w = b_b.w.mul_add(c.inv_i_b, r_b.x.mul(py).sub(r_b.y.mul(px)));
            }
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// (b2ApplyRestitutionTask, per color)
pub fn apply_restitution_wide(
    wide: &mut [ContactConstraintWide],
    states: &mut [BodyState],
    context: &StepContext,
) {
    let threshold = FloatW::splat(context.restitution_threshold);
    let zero = FloatW::zero();

    for c in wide.iter_mut() {
        if c.restitution.all_zero() {
            // No lanes have restitution. Common case.
            continue;
        }

        // lanes with no restitution are masked out below
        let restitution_mask = c.restitution.equals(zero);

        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        // point 1 non-penetration constraint
        {
            let mask1 = c.relative_velocity1.add(threshold).greater_than(zero);
            let mask2 = c.total_normal_impulse1.equals(zero);
            let mask = mask1.or(mask2).or(restitution_mask);
            let mass = FloatW::blend(c.normal_mass1, zero, mask);

            let r_a = c.anchor_a1;
            let r_b = c.anchor_b1;

            let dvx = b_b
                .v
                .x
                .sub(b_b.w.mul(r_b.y))
                .sub(b_a.v.x.sub(b_a.w.mul(r_a.y)));
            let dvy = b_b
                .v
                .y
                .add(b_b.w.mul(r_b.x))
                .sub(b_a.v.y.add(b_a.w.mul(r_a.x)));
            let vn = dvx.mul(c.normal.x).add(dvy.mul(c.normal.y));

            let neg_impulse = mass.mul(vn.add(c.restitution.mul(c.relative_velocity1)));

            let new_impulse = c.normal_impulse1.sub(neg_impulse).max(zero);
            let delta_impulse = new_impulse.sub(c.normal_impulse1);
            c.normal_impulse1 = new_impulse;
            c.total_normal_impulse1 = c.total_normal_impulse1.add(delta_impulse);

            let px = delta_impulse.mul(c.normal.x);
            let py = delta_impulse.mul(c.normal.y);

            b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, px);
            b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, py);
            b_a.w = b_a.w.mul_sub(c.inv_i_a, r_a.x.mul(py).sub(r_a.y.mul(px)));

            b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, px);
            b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, py);
            b_b.w = b_b.w.mul_add(c.inv_i_b, r_b.x.mul(py).sub(r_b.y.mul(px)));
        }

        // point 2 non-penetration constraint
        {
            let mask1 = c.relative_velocity2.add(threshold).greater_than(zero);
            let mask2 = c.total_normal_impulse2.equals(zero);
            let mask = mask1.or(mask2).or(restitution_mask);
            let mass = FloatW::blend(c.normal_mass2, zero, mask);

            let r_a = c.anchor_a2;
            let r_b = c.anchor_b2;

            let dvx = b_b
                .v
                .x
                .sub(b_b.w.mul(r_b.y))
                .sub(b_a.v.x.sub(b_a.w.mul(r_a.y)));
            let dvy = b_b
                .v
                .y
                .add(b_b.w.mul(r_b.x))
                .sub(b_a.v.y.add(b_a.w.mul(r_a.x)));
            let vn = dvx.mul(c.normal.x).add(dvy.mul(c.normal.y));

            let neg_impulse = mass.mul(vn.add(c.restitution.mul(c.relative_velocity2)));

            let new_impulse = c.normal_impulse2.sub(neg_impulse).max(zero);
            let delta_impulse = new_impulse.sub(c.normal_impulse2);
            c.normal_impulse2 = new_impulse;
            c.total_normal_impulse2 = c.total_normal_impulse2.add(delta_impulse);

            let px = delta_impulse.mul(c.normal.x);
            let py = delta_impulse.mul(c.normal.y);

            b_a.v.x = b_a.v.x.mul_sub(c.inv_mass_a, px);
            b_a.v.y = b_a.v.y.mul_sub(c.inv_mass_a, py);
            b_a.w = b_a.w.mul_sub(c.inv_i_a, r_a.x.mul(py).sub(r_a.y.mul(px)));

            b_b.v.x = b_b.v.x.mul_add(c.inv_mass_b, px);
            b_b.v.y = b_b.v.y.mul_add(c.inv_mass_b, py);
            b_b.w = b_b.w.mul_add(c.inv_i_b, r_b.x.mul(py).sub(r_b.y.mul(px)));
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// Write the accumulated impulses back to the color's contact manifolds. Hit
/// events are flagged by the caller from the updated manifolds, matching the
/// serial store path. (b2StoreImpulsesTask, write-back portion)
pub fn store_impulses_wide(wide: &[ContactConstraintWide], contacts: &mut [ContactSim]) {
    for (i, contact) in contacts.iter_mut().enumerate() {
        let block = i / SIMD_WIDTH;
        let lane = i % SIMD_WIDTH;
        let c = &wide[block];

        let manifold = &mut contact.manifold;
        manifold.rolling_impulse = c.rolling_impulse.0[lane];

        manifold.points[0].normal_impulse = c.normal_impulse1.0[lane];
        manifold.points[0].tangent_impulse = c.tangent_impulse1.0[lane];
        manifold.points[0].total_normal_impulse = c.total_normal_impulse1.0[lane];
        manifold.points[0].normal_velocity = c.relative_velocity1.0[lane];

        manifold.points[1].normal_impulse = c.normal_impulse2.0[lane];
        manifold.points[1].tangent_impulse = c.tangent_impulse2.0[lane];
        manifold.points[1].total_normal_impulse = c.total_normal_impulse2.0[lane];
        manifold.points[1].normal_velocity = c.relative_velocity2.0[lane];
    }
}
