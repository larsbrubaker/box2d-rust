// Port of box2d-cpp-reference/shared/benchmarks.c: the ten scene "Create"
// functions and the three "Step" functions used by the benchmark harness.
//
// Split into submodules to stay within the project's per-file line limit:
//   - pyramids: joint grid, pyramids, and the compound-shape barrel
//   - machines: spinner, smash, tumbler, washer, junkyard
//   - rain:     the ragdoll rain scene (uses the shared `human` helper)
//
// BENCHMARK_DEBUG mirrors the C `#ifdef NDEBUG` switch: it is 0 (false) in a
// release build (NDEBUG defined, asserts off) and 1 (true) in a debug build.
// Rust's `debug_assertions` cfg is exactly the inverse of NDEBUG, so the scene
// sizes match the C library for both build profiles.
//
// The C globals g_rainData / g_spinnerData / g_junkyardData become thread-local
// state; the harness runs serially so a single thread owns them, and each
// Create function overwrites the state exactly like the C globals.

mod machines;
mod pyramids;
mod rain;

pub use machines::*;
pub use pyramids::*;
pub use rain::*;

/// (BENCHMARK_DEBUG)
pub(crate) const BENCHMARK_DEBUG: bool = cfg!(debug_assertions);
