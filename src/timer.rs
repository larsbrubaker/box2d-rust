//! Port of the timing helpers from box2d-cpp-reference/src/timer.c
//! (`b2GetTicks`, `b2GetMilliseconds`, `b2GetMillisecondsAndReset`).
//!
//! Mutex / semaphore / thread wrappers from that file are not ported — Rust's
//! standard library covers them. Only the profiling clock remains.
//!
//! Uses `std::time::Instant` for relative durations. On wasm32 Instant is backed
//! by `performance.now()` in current rustc, which is enough for demo profiling
//! (Capacity's 20 ms threshold) as well as native.
//!
//! SPDX-FileCopyrightText: 2023 Erin Catto
//! SPDX-License-Identifier: MIT

use std::time::Instant;

/// Opaque tick capture. (return value of `b2GetTicks`)
#[derive(Clone, Copy)]
pub struct Ticks {
    start: Instant,
}

/// Capture the current time. (`b2GetTicks`)
#[inline]
pub fn get_ticks() -> Ticks {
    Ticks {
        start: Instant::now(),
    }
}

/// Milliseconds elapsed since `ticks`. (`b2GetMilliseconds`)
#[inline]
pub fn get_milliseconds(ticks: Ticks) -> f32 {
    // Match C: cast the double ms value down to float.
    (ticks.start.elapsed().as_secs_f64() * 1000.0) as f32
}

/// Milliseconds elapsed since `ticks`, then reset `ticks` to now.
/// (`b2GetMillisecondsAndReset`)
#[inline]
pub fn get_milliseconds_and_reset(ticks: &mut Ticks) -> f32 {
    let now = Instant::now();
    let ms = (now.duration_since(ticks.start).as_secs_f64() * 1000.0) as f32;
    ticks.start = now;
    ms
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn milliseconds_are_non_negative() {
        let ticks = get_ticks();
        let ms = get_milliseconds(ticks);
        assert!(ms >= 0.0);
    }

    #[test]
    fn reset_advances_the_mark() {
        let mut ticks = get_ticks();
        let start = Instant::now();
        while start.elapsed().as_nanos() < 50_000 {
            core::hint::spin_loop();
        }
        let first = get_milliseconds_and_reset(&mut ticks);
        assert!(first >= 0.0);
        let second = get_milliseconds(ticks);
        assert!(second >= 0.0);
        assert!(second <= first + 1.0);
    }
}
