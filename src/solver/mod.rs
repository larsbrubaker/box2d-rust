// Solver module: b2Softness/b2MakeSoft and the step context from solver.h.
// The solver stages land in the solver bring-up commit.
//
// The C b2StepContext carries multithreading scratch (solver stages, sync
// blocks, atomics, per-color wide-constraint arrays and interior pointers to
// world data). The single-threaded port keeps only the step parameters here;
// solver scratch is owned locally by the solve pass and world data is passed
// as separate arguments to avoid aliasing &mut World.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::math_functions::PI;

/// Soft constraint coefficients. (b2Softness)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Softness {
    pub bias_rate: f32,
    pub mass_scale: f32,
    pub impulse_scale: f32,
}

/// (static inline b2MakeSoft)
pub fn make_soft(hertz: f32, zeta: f32, h: f32) -> Softness {
    if hertz == 0.0 {
        return Softness {
            bias_rate: 0.0,
            mass_scale: 0.0,
            impulse_scale: 0.0,
        };
    }

    let omega = 2.0 * PI * hertz;
    let a1 = 2.0 * zeta + h * omega;
    let a2 = h * omega * a1;
    let a3 = 1.0 / (1.0 + a2);

    // bias = w / (2 * z + hw)
    // massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
    // impulseScale = 1 / (1 + hw * (2 * z + hw))
    //
    // If z == 0
    // bias = 1/h
    // massScale = hw^2 / (1 + hw^2)
    // impulseScale = 1 / (1 + hw^2)
    //
    // w -> inf
    // bias = 1/h
    // massScale = 1
    // impulseScale = 0
    //
    // In all cases:
    // massScale + impulseScale == 1
    Softness {
        bias_rate: omega / a1,
        mass_scale: a2 * a3,
        impulse_scale: a3,
    }
}

/// Context for a time step. Recreated each time step. (b2StepContext — step
/// parameters only; see the module header for what the serial port drops)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StepContext {
    /// time step
    pub dt: f32,

    /// inverse time step (0 if dt == 0).
    pub inv_dt: f32,

    /// sub-step
    pub h: f32,
    pub inv_h: f32,

    pub sub_step_count: i32,

    pub contact_softness: Softness,
    pub static_softness: Softness,

    pub restitution_threshold: f32,
    pub max_linear_velocity: f32,

    /// Copied from World::contact_speed (the C reads it through
    /// context->world).
    pub contact_speed: f32,

    pub enable_warm_starting: bool,
}

mod continuous;
mod integrate;
mod solve;

pub use solve::*;
