// Port of box2d-cpp-reference/include/box2d/collision.h
//
// This module holds the collision data types. It is being ported incrementally:
// each type lands together with the first module that consumes it. Right now
// that is `b2CastOutput`, used by the AABB ray cast.
//
// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

use crate::math_functions::Vec2;

/// Low level ray cast or shape cast output data. (b2CastOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CastOutput {
    /// The surface normal at the hit point
    pub normal: Vec2,
    /// The surface hit point
    pub point: Vec2,
    /// The fraction of the input translation at collision
    pub fraction: f32,
    /// The number of iterations used
    pub iterations: i32,
    /// Did the cast hit?
    pub hit: bool,
}
