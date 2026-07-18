// Port of box2d-cpp-reference/shared/utils.c + utils.h: the XorShift32 random
// number generator used for cross-platform determinism.
//
// GetNumberOfCores is intentionally not ported: the Rust benchmark harness is
// serial and always reports a single worker.
//
// None of the ported benchmark scenes currently call the RNG (RandomPolygon and
// friends are used by other shared helpers/samples), so the whole module is
// allowed dead code. It is kept as a faithful companion to the C shared library.
#![allow(dead_code)]

use std::cell::Cell;

use box2d_rust::collision::Polygon;
use box2d_rust::geometry::{make_polygon, make_square};
use box2d_rust::hull::compute_hull;
use box2d_rust::math_functions::{make_rot, Rot, Vec2, PI};

pub const RAND_LIMIT: i32 = 32767;
pub const RAND_SEED: u32 = 12345;

thread_local! {
    /// Global seed for the simple random number generator. (g_randomSeed)
    static G_RANDOM_SEED: Cell<u32> = const { Cell::new(RAND_SEED) };
}

/// Reset the RNG seed. (assign g_randomSeed = RAND_SEED)
pub fn reset_random_seed() {
    G_RANDOM_SEED.with(|seed| seed.set(RAND_SEED));
}

/// Simple random number generator. Using this instead of rand() for
/// cross-platform determinism. (RandomInt)
pub fn random_int() -> i32 {
    G_RANDOM_SEED.with(|seed| {
        // XorShift32 algorithm
        let mut x = seed.get();
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        seed.set(x);

        // Map the 32-bit value to the range 0 to RAND_LIMIT
        (x % (RAND_LIMIT as u32 + 1)) as i32
    })
}

/// Random integer in range [lo, hi]. (RandomIntRange)
pub fn random_int_range(lo: i32, hi: i32) -> i32 {
    lo + random_int() % (hi - lo + 1)
}

/// Random number in range [-1,1]. (RandomFloat)
pub fn random_float() -> f32 {
    let mut r = (random_int() & RAND_LIMIT) as f32;
    r /= RAND_LIMIT as f32;
    r = 2.0 * r - 1.0;
    r
}

/// Random floating point number in range [lo, hi]. (RandomFloatRange)
pub fn random_float_range(lo: f32, hi: f32) -> f32 {
    let mut r = (random_int() & RAND_LIMIT) as f32;
    r /= RAND_LIMIT as f32;
    r = (hi - lo) * r + lo;
    r
}

/// Random vector with coordinates in range [lo, hi]. (RandomVec2)
pub fn random_vec2(lo: f32, hi: f32) -> Vec2 {
    Vec2 {
        x: random_float_range(lo, hi),
        y: random_float_range(lo, hi),
    }
}

/// Random rotation with angle in range [-pi, pi]. (RandomRot)
pub fn random_rot() -> Rot {
    let angle = random_float_range(-PI, PI);
    make_rot(angle)
}

/// (RandomPolygon)
pub fn random_polygon(extent: f32) -> Polygon {
    let count = 3 + random_int() % 6;
    let mut points = Vec::with_capacity(count as usize);
    for _ in 0..count {
        points.push(random_vec2(-extent, extent));
    }

    let hull = compute_hull(&points);
    if hull.count > 0 {
        return make_polygon(&hull, 0.0);
    }

    make_square(extent)
}
