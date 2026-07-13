//! Geometry category — C `sample_geometry.cpp` Convex Hull.
//!
//! Point generation uses the shared samples XorShift (`utils.h` RAND_SEED /
//! RandomFloat). Hull computation and validation go through the ported
//! `b2ComputeHull` / `b2ValidateHull`. Invented Geometry Queries / Labs
//! manifolds helpers are retired (C Manifold scenes live under Collision).

use std::cell::RefCell;

use wasm_bindgen::prelude::*;

use box2d_rust::hull::{compute_hull, validate_hull, Hull, MAX_POLYGON_VERTICES};
use box2d_rust::math_functions::{clamp, make_rot, rotate_vector, Vec2, PI};

const RAND_LIMIT: u32 = 32767;
const RAND_SEED: u32 = 12345;

thread_local! {
    static RNG_SEED: RefCell<u32> = RefCell::new(RAND_SEED);
    static SAMPLE: RefCell<ConvexHullSample> = RefCell::new(ConvexHullSample::new());
}

/// C XorShift32 RandomInt (shared/utils.h).
fn random_int() -> i32 {
    RNG_SEED.with(|cell| {
        let mut x = *cell.borrow();
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *cell.borrow_mut() = x;
        (x % (RAND_LIMIT + 1)) as i32
    })
}

/// C RandomFloat — range [-1, 1] (shared/utils.h:49).
fn random_float() -> f32 {
    let mut r = (random_int() as u32 & RAND_LIMIT) as f32;
    r /= RAND_LIMIT as f32;
    2.0 * r - 1.0
}

struct ConvexHullSample {
    points: [Vec2; MAX_POLYGON_VERTICES],
    count: i32,
    generation: i32,
    auto_mode: bool,
    bulk: bool,
}

impl ConvexHullSample {
    fn new() -> Self {
        let mut s = Self {
            points: [Vec2 { x: 0.0, y: 0.0 }; MAX_POLYGON_VERTICES],
            count: 0,
            generation: 0,
            auto_mode: false,
            bulk: false,
        };
        s.generate();
        s
    }

    /// ConvexHull::Generate (sample_geometry.cpp:35).
    fn generate(&mut self) {
        let angle = PI * random_float();
        let r = make_rot(angle);
        let lower_bound = Vec2 { x: -4.0, y: -4.0 };
        let upper_bound = Vec2 { x: 4.0, y: 4.0 };

        for i in 0..MAX_POLYGON_VERTICES {
            let x = 10.0 * random_float();
            let y = 10.0 * random_float();
            // Clamp onto a square to help create collinearities.
            let v = clamp(Vec2 { x, y }, lower_bound, upper_bound);
            self.points[i] = rotate_vector(r, v);
        }
        self.count = MAX_POLYGON_VERTICES as i32;
        self.generation += 1;
    }

    fn compute(&self) -> (Hull, bool) {
        let hull = compute_hull(&self.points[..self.count as usize]);
        if hull.count == 0 {
            return (hull, false);
        }
        let valid = validate_hull(&hull);
        (hull, valid)
    }

    /// ConvexHull::Step bulk / auto / single path (sample_geometry.cpp:112).
    fn step(&mut self, advance: bool) -> (Hull, bool) {
        if self.bulk {
            // Defect hunting: up to 10_000 generate+validate iterations.
            for _ in 0..10_000 {
                self.generate();
                let (hull, valid) = self.compute();
                if hull.count == 0 {
                    continue;
                }
                if !valid || !self.bulk {
                    self.bulk = false;
                    return (hull, valid);
                }
            }
            return self.compute();
        }

        if self.auto_mode && advance {
            self.generate();
        }

        let (hull, valid) = self.compute();
        if hull.count > 0 && !valid {
            self.auto_mode = false;
        }
        (hull, valid)
    }
}

fn pack(state: &ConvexHullSample, hull: &Hull, valid: bool) -> Vec<f32> {
    // [generation, pointCount, valid, hullCount, auto, bulk,
    //  points (2*pointCount), hullPoints (2*hullCount)]
    let mut out = Vec::with_capacity(6 + 2 * MAX_POLYGON_VERTICES * 2);
    out.push(state.generation as f32);
    out.push(state.count as f32);
    out.push(if valid { 1.0 } else { 0.0 });
    out.push(hull.count as f32);
    out.push(if state.auto_mode { 1.0 } else { 0.0 });
    out.push(if state.bulk { 1.0 } else { 0.0 });
    for i in 0..state.count as usize {
        out.push(state.points[i].x);
        out.push(state.points[i].y);
    }
    for i in 0..hull.count as usize {
        out.push(hull.points[i].x);
        out.push(hull.points[i].y);
    }
    out
}

/// Reset RNG + sample state (page init / Restart).
#[wasm_bindgen]
pub fn geometry_hull_reset() -> Vec<f32> {
    RNG_SEED.with(|s| *s.borrow_mut() = RAND_SEED);
    SAMPLE.with(|cell| {
        *cell.borrow_mut() = ConvexHullSample::new();
        let state = cell.borrow();
        let (hull, valid) = state.compute();
        pack(&state, &hull, valid)
    })
}

/// Keyboard: A auto, B bulk, G generate (sample_geometry.cpp:91).
/// `key` is uppercase ASCII (`b'A'`, `b'B'`, `b'G'`).
#[wasm_bindgen]
pub fn geometry_hull_key(key: u32) {
    SAMPLE.with(|cell| {
        let mut state = cell.borrow_mut();
        match key {
            0x41 => state.auto_mode = !state.auto_mode, // A
            0x42 => state.bulk = !state.bulk,           // B
            0x47 => state.generate(),                   // G
            _ => {}
        }
    });
}

/// One Step() of ConvexHull. `advance` is true when the sample is not paused
/// (drives auto regenerate). Returns the packed draw/HUD buffer.
#[wasm_bindgen]
pub fn geometry_hull_step(advance: bool) -> Vec<f32> {
    SAMPLE.with(|cell| {
        let mut state = cell.borrow_mut();
        let (hull, valid) = state.step(advance);
        pack(&state, &hull, valid)
    })
}
