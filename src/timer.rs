//! Port of the timing helpers from box2d-cpp-reference/src/timer.c
//! (`b2GetTicks`, `b2GetMilliseconds`, `b2GetMillisecondsAndReset`).
//!
//! Mutex / semaphore / thread wrappers from that file are not ported — Rust's
//! standard library covers them. Only the profiling clock remains.
//!
//! A `Ticks` is an opaque capture of a monotonic nanosecond counter. The
//! counter comes from one of three sources, which is where this diverges from
//! the C original (C picks a platform clock with `#ifdef`s; we also have to
//! deal with WASM, where "the platform" is whatever host embeds the module):
//!
//! 1. Native targets: `std::time::Instant` elapsed since a process-wide epoch.
//! 2. `wasm32-unknown-unknown` with the default `web-time` feature: the same,
//!    using `web_time::Instant`, which is backed by `performance.now()`. That
//!    provides real wall-clock values needed for demo profiling (Capacity's
//!    20 ms threshold) without trapping every world step.
//! 3. `wasm32-unknown-unknown` with `default-features = false`: a null clock
//!    that always returns 0, so `Profile` timings read as zero. This mirrors
//!    timer.c's own "Fallbacks for unknown platforms" branch, where
//!    `b2GetTicks` returns 0 and `b2GetMilliseconds` returns `0.0f`. The
//!    configuration exists because `web-time` pulls in `wasm-bindgen`, whose
//!    `__wbindgen_placeholder__` imports do not exist on non-browser WASM
//!    hosts such as Wasmtime, making the module impossible to instantiate
//!    there. `std::time::Instant::now()` panics on that target, so it cannot
//!    serve as the fallback. Such hosts can supply their own clock with
//!    [`set_clock`].
//!
//! None of this affects simulation results; the clock only fills `Profile`.
//!
//! SPDX-FileCopyrightText: 2023 Erin Catto
//! SPDX-License-Identifier: MIT

use core::sync::atomic::{AtomicPtr, Ordering};

/// A monotonic clock returning nanoseconds since an arbitrary fixed epoch.
/// (see [`set_clock`])
pub type ClockFn = fn() -> u64;

/// Installed clock override, or null for the default clock. Stored as a raw
/// pointer so reads are lock-free: the clock is sampled ~19 times per world
/// step.
static CLOCK: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// Install a custom profiling clock, or restore the default with `None`.
///
/// The function must return monotonically non-decreasing nanoseconds since
/// some arbitrary, fixed epoch. This is mainly useful on non-browser WASM
/// hosts, which have no clock of their own that this crate could reach;
/// without one, all `Profile` timings are zero.
///
/// Install the clock once at startup, before the first world step. Swapping
/// clocks mid-step mixes epochs: a `Ticks` captured from the old clock is then
/// differenced against the new one, so that one `Profile` sample saturates to
/// zero or reports a nonsense-large duration. Later steps are fine.
pub fn set_clock(clock: Option<ClockFn>) {
    let raw = match clock {
        Some(f) => f as *mut (),
        None => core::ptr::null_mut(),
    };
    CLOCK.store(raw, Ordering::Release);
}

/// Read the active clock.
#[inline]
fn now_nanos() -> u64 {
    let raw = CLOCK.load(Ordering::Acquire);
    if raw.is_null() {
        default_now_nanos()
    } else {
        // SAFETY: `raw` is non-null here, so it can only be a value stored by
        // `set_clock`, which stores exactly `f as *mut ()` for some
        // `f: ClockFn`. Casting that pointer back to the same function pointer
        // type is the inverse of the cast that produced it. `transmute` checks
        // that the two types have the same size at compile time, so a target
        // where they differ is a build error rather than undefined behavior.
        let clock: ClockFn = unsafe { core::mem::transmute::<*mut (), ClockFn>(raw) };
        clock()
    }
}

#[cfg(not(all(target_arch = "wasm32", target_os = "unknown")))]
#[inline]
fn default_now_nanos() -> u64 {
    use std::sync::OnceLock;
    use std::time::Instant;

    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let d = EPOCH.get_or_init(Instant::now).elapsed();
    d.as_secs() * 1_000_000_000 + d.subsec_nanos() as u64
}

#[cfg(all(target_arch = "wasm32", target_os = "unknown", feature = "web-time"))]
#[inline]
fn default_now_nanos() -> u64 {
    use std::sync::OnceLock;
    use web_time::Instant;

    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let d = EPOCH.get_or_init(Instant::now).elapsed();
    d.as_secs() * 1_000_000_000 + d.subsec_nanos() as u64
}

// Null clock: no clock is reachable from a bare wasm32-unknown-unknown module.
#[cfg(all(
    target_arch = "wasm32",
    target_os = "unknown",
    not(feature = "web-time")
))]
#[inline]
fn default_now_nanos() -> u64 {
    0
}

/// Opaque tick capture. (return value of `b2GetTicks`)
#[derive(Clone, Copy)]
pub struct Ticks {
    start: u64,
}

/// Capture the current time. (`b2GetTicks`)
#[inline]
pub fn get_ticks() -> Ticks {
    Ticks { start: now_nanos() }
}

/// Milliseconds elapsed since `ticks`. (`b2GetMilliseconds`)
#[inline]
pub fn get_milliseconds(ticks: Ticks) -> f32 {
    // Match C: cast the double ms value down to float. `saturating_sub` guards
    // against a user clock that steps backwards.
    (now_nanos().saturating_sub(ticks.start) as f64 / 1.0e6) as f32
}

/// Milliseconds elapsed since `ticks`, then reset `ticks` to now.
/// (`b2GetMillisecondsAndReset`)
#[inline]
pub fn get_milliseconds_and_reset(ticks: &mut Ticks) -> f32 {
    let now = now_nanos();
    // Match C: cast the double ms value down to float.
    let ms = (now.saturating_sub(ticks.start) as f64 / 1.0e6) as f32;
    ticks.start = now;
    ms
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::sync::atomic::AtomicU64;
    use std::sync::Mutex;
    use std::time::Instant;

    /// The installed clock is process-global while cargo runs unit tests on
    /// many threads, so every test that touches it holds this lock.
    static CLOCK_TEST_LOCK: Mutex<()> = Mutex::new(());

    static FAKE_NOW: AtomicU64 = AtomicU64::new(0);

    thread_local! {
        /// Only the thread that installed the fake clock sees fake time; every
        /// other thread keeps a real monotonic clock, so concurrent tests that
        /// measure real elapsed time are unaffected.
        static USE_FAKE: Cell<bool> = const { Cell::new(false) };
    }

    fn fake_clock() -> u64 {
        if USE_FAKE.with(|f| f.get()) {
            FAKE_NOW.load(Ordering::Relaxed)
        } else {
            default_now_nanos()
        }
    }

    fn set_fake(nanos: u64) {
        FAKE_NOW.store(nanos, Ordering::Relaxed);
    }

    #[test]
    fn milliseconds_are_non_negative() {
        let _guard = CLOCK_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let ticks = get_ticks();
        let ms = get_milliseconds(ticks);
        assert!(ms >= 0.0);
    }

    #[test]
    fn reset_advances_the_mark() {
        let _guard = CLOCK_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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

    #[test]
    fn custom_clock_drives_reported_times() {
        let _guard = CLOCK_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        set_fake(1_000_000_000);
        USE_FAKE.with(|f| f.set(true));
        set_clock(Some(fake_clock));

        let mut ticks = get_ticks();

        set_fake(1_002_500_000);
        assert_eq!(get_milliseconds(ticks), 2.5);

        set_fake(1_009_500_000);
        assert_eq!(get_milliseconds_and_reset(&mut ticks), 9.5);

        set_fake(1_010_000_000);
        assert_eq!(get_milliseconds(ticks), 0.5);

        // A clock that steps backwards saturates to zero instead of wrapping.
        set_fake(0);
        assert_eq!(get_milliseconds(ticks), 0.0);

        set_clock(None);
        USE_FAKE.with(|f| f.set(false));

        // Default clock is back.
        let ticks = get_ticks();
        assert!(get_milliseconds(ticks) >= 0.0);
    }
}
