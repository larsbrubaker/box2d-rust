// Solver module. This starts with the b2Softness type from solver.h, which
// the joint sim structs embed; the rest of solver.h/solver.c (step context,
// constraint types, the solver stages) lands in the solver bring-up commit.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

/// Soft constraint coefficients. (b2Softness)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Softness {
    pub bias_rate: f32,
    pub mass_scale: f32,
    pub impulse_scale: f32,
}
