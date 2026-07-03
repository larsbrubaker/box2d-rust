// Port of box2d-cpp-reference/include/box2d/constants.h
//
// The C header defines these as macros. Constants that do not depend on runtime
// state are `pub const`. Those defined in terms of `b2GetLengthUnitsPerMeter()`
// re-read that global on every use, so they are ported as functions to preserve
// that behavior exactly.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::get_length_units_per_meter;
use crate::math_functions::PI;

/// Maximum parallel workers. Used for some fixed size arrays. (B2_MAX_WORKERS)
pub const MAX_WORKERS: i32 = 32;

/// Maximum number of tasks queued per world step. (B2_MAX_TASKS)
pub const MAX_TASKS: i32 = 256;

/// Maximum number of colors in the constraint graph. (B2_GRAPH_COLOR_COUNT)
pub const GRAPH_COLOR_COUNT: i32 = 24;

/// Maximum number of simultaneous worlds that can be allocated. (B2_MAX_WORLDS)
pub const MAX_WORLDS: i32 = 128;

/// Maximum length of the body name. (B2_NAME_LENGTH)
pub const NAME_LENGTH: i32 = 10;

/// The maximum rotation of a body per time step. Used to prevent numerical
/// problems. (B2_MAX_ROTATION)
///
/// @warning increasing this to 0.5 * pi or greater will break continuous collision.
pub const MAX_ROTATION: f32 = 0.25 * PI;

/// The default contact recycling world angle threshold. 0.98 ~= 11.5 degrees.
/// (B2_CONTACT_RECYCLE_COS_ANGLE)
pub const CONTACT_RECYCLE_COS_ANGLE: f32 = 0.98;

/// For small objects the margin is limited to this fraction times the maximum
/// extent. (B2_AABB_MARGIN_FRACTION)
pub const AABB_MARGIN_FRACTION: f32 = 0.125;

/// The time that a body must be still before it will go to sleep, in seconds.
/// (B2_TIME_TO_SLEEP)
pub const TIME_TO_SLEEP: f32 = 0.5;

/// Used to detect bad values. Positions greater than about 16km will have
/// precision problems in single precision, so 100km as a limit should be fine.
/// In large world mode the broad-phase starts to have excessive padding at
/// 10,000km. (B2_HUGE)
#[cfg(feature = "double-precision")]
pub fn huge() -> f32 {
    1.0e9 * get_length_units_per_meter()
}

/// See [`huge`].
#[cfg(not(feature = "double-precision"))]
pub fn huge() -> f32 {
    1.0e5 * get_length_units_per_meter()
}

/// A small length used as a collision and constraint tolerance. Usually chosen
/// to be numerically significant but visually insignificant, normally 0.5cm.
/// (B2_LINEAR_SLOP)
///
/// @warning modifying this can have a significant impact on stability.
pub fn linear_slop() -> f32 {
    0.005 * get_length_units_per_meter()
}

/// Box2D uses limited speculative collision, normally 2cm. This reduces jitter.
/// (B2_SPECULATIVE_DISTANCE)
///
/// @warning modifying this can have a significant impact on performance and stability.
pub fn speculative_distance() -> f32 {
    4.0 * linear_slop()
}

/// The default contact recycling distance. (B2_CONTACT_RECYCLE_DISTANCE)
pub fn contact_recycle_distance() -> f32 {
    10.0 * linear_slop()
}

/// Used to fatten AABBs in the dynamic tree so proxies can move a small amount
/// without triggering a tree adjustment, normally 5cm. (B2_MAX_AABB_MARGIN)
///
/// @warning modifying this can have a significant impact on performance.
pub fn max_aabb_margin() -> f32 {
    0.05 * get_length_units_per_meter()
}
