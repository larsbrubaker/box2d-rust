# box2d-rust

A pure Rust port of [Box2D v3](https://github.com/erincatto/box2d), Erin Catto's 2D physics
engine — exact behavioral match, including cross-platform deterministic math.

[![crates.io](https://img.shields.io/crates/v/box2d-rust.svg)](https://crates.io/crates/box2d-rust)
[![docs.rs](https://docs.rs/box2d-rust/badge.svg)](https://docs.rs/box2d-rust)
[![License](https://img.shields.io/badge/License-MIT-lightblue.svg)](LICENSE)
[![Live Demo](https://img.shields.io/badge/Live_Demo-Interactive-blue)](https://larsbrubaker.github.io/box2d-rust/)

## Interactive Demo

**[Try it in your browser — no installation required](https://larsbrubaker.github.io/box2d-rust/)**

[![box2d-rust demo](readme_hero.jpg)](https://larsbrubaker.github.io/box2d-rust/)

Live WebAssembly demos running the ported engine — all 13 upstream sample categories:
bodies, shapes, stacking, joints, events, continuous collision, character movers, world
queries and explosions, determinism (live snapshot/restore with bit-identical state
hashes), overlap-recovery robustness, and benchmarks.

> Part of the [rust-apps](https://github.com/larsbrubaker/rust-apps) suite — a collection of
> Rust graphics and geometry libraries by Lars Brubaker.

## Status: Complete

Every portable module of the Box2D v3.1 C source is ported, together with the C test suite
(129 tests, green in both precision modes). The pinned reference source lives in the
`box2d-cpp-reference/` submodule.

| Area | Ported | Tests |
|---|---|---|
| Foundation: math_functions, core/constants, id, bitset, id_pool, table, types | ✅ | ✅ (test_math/id/bitset/table.c) |
| Collision: aabb, distance (GJK/TOI), hull, geometry, manifold, dynamic_tree | ✅ | ✅ (test_collision/distance/shape/dynamic_tree.c) |
| Broad phase: proxy ops, move buffer, pair update → contact creation | ✅ | ✅ |
| Dynamics: body/shape/contact lifecycles, constraint graph, solver sets, islands | ✅ | ✅ |
| Joints: distance, motor, filter, prismatic, revolute, weld, wheel | ✅ | ✅ |
| Solver: contact solver + serial step pipeline, sensors, sleeping, continuous | ✅ | ✅ (test_world.c) |
| World API: queries, casts, character movers, explosions, all setters | ✅ | ✅ |
| Determinism: hand-rolled trig, bit-exact FallingHinges vs the C build | ✅ | ✅ (test_determinism.c) |
| Snapshots: `world_snapshot` / `world_restore`, deep state hash | ✅ | ✅ (test_snapshot.c) |
| Recording: full op-stream record/replay of every API mutation and query | ✅ | ✅ (test_recording.c) |
| Replay player: incremental playback, keyframe ring, timeline scrub, outliner | ✅ | ✅ (test_recording.c viewer subtests) |
| Debug draw: `world_draw` with the complete `DebugDraw` trait + color palette | ✅ | ✅ |
| Large world mode (`double-precision` feature = `BOX2D_DOUBLE_PRECISION`) | ✅ | ✅ (test_large_world.c) |

Not ported (by design): threading/task system (the port is serial), the global world
registry (worlds are owned values), and the C arena allocator (Rust `Vec`s).

## Performance

The port is measured against the C reference using the C repo's own `benchmark` app (10
scenes) and a line-for-line Rust port of it (`examples/benchmark`, run with
`cargo run --release --example benchmark`). Both run **single-threaded** — the Rust port is
serial by design, so C runs with `-w=1` (its serial fallback, no scheduler). Both use the
same scenes, the same constants, `dt = 1/60`, 4 sub-steps, the warm-up step excluded, and a
minimum of 4 runs kept.

Methodology: Intel Core i7-7660U (2C/4T mobile, 2017) · 8 GB RAM · Windows 10 · rustc 1.91.0
(release) vs MSVC 19.x /O2 (VS 2022 Build Tools) · C reference @ submodule pin `56edae7` ·
measured 2026-07-18. Results are indicative; the ratios are the stable quantity, absolute
times vary with hardware.

Total ms for the scene's full step count (min of 4 runs):

| Scene | Steps | C (ms) | Rust (ms) | Rust / C |
|---|---|---|---|---|
| compounds | 500 | 3685 | 6701 | 1.82× |
| joint_grid | 500 | 5774 | 6842 | 1.18× |
| junkyard | 800 | 8425 | 17799 | 2.11× |
| large_pyramid | 500 | 2842 | 6235 | 2.19× |
| many_pyramids | 200 | 5974 | 11489 | 1.92× |
| rain | 1000 | 17145 | 28143 | 1.64× |
| smash | 300 | 3395 | 7345 | 2.16× |
| spinner | 500 | 9992 | 27259 | 2.73× |
| tumbler | 750 | 3004 | 4663 | 1.55× |
| washer | 500 | 10523 | 19497 | 1.85× |

Geometric mean ≈ **1.9× slower than C** today (range 1.18–2.73×).

Attribution (from `b2Profile` per-phase dumps, `-s` flag, mean ms/step):

- Joint solver is near parity: joint_grid `constraints` 9.4 ms vs C 8.2 ms (1.15×).
- Contact solver is ~2.7×: large_pyramid `constraints` 12.6 ms vs C 4.7 ms.
- Narrow-phase `collide` is the biggest outlier on capsule-heavy scenes: spinner 31.7 ms vs
  C 6.9 ms (~4.6×) — this explains spinner's 2.73× overall.
- Broad phase `pairs`/`refit` run ~1.5–2×.

### Performance roadmap

Ordered by expected win:

1. **Capsule/segment manifold path (narrow phase)** — ~4.6× on spinner; profile and optimize
   `collide` for capsule vs capsule/chain.
2. **Contact solver inner loops** — ~2.7×; investigate bounds-check elimination in the
   Vec-indexed constraint arrays, memory layout, and whether MSVC is auto-vectorizing the C
   soft-constraint loops that rustc isn't.
3. **Dynamic tree refit + pair traversal** (~1.5–2×).
4. **Re-measure after each change** with `cargo run --release --example benchmark` vs the C
   app (commands documented in `examples/benchmark` and this section).

Multithreading (a work-stealing solver like C's built-in scheduler) is a separate, larger
lever — the C version gains ~Nx with workers; the port is serial today by design.

## Porting principles

- **Exact behavioral match** — same algorithms, same `f32` arithmetic, same edge cases as the C
  source. Floating-point operations are never reordered or "improved".
- **Determinism preserved** — Box2D's hand-rolled `b2Atan2` and `b2ComputeCosSin` (built for
  cross-platform determinism) are ported bit-for-bit, never replaced with std functions.
- **Tests ported too** — every module lands together with its portion of the C test suite.
- **No stubs** — no `todo!()`, no placeholders; modules are ported whole, in dependency order.
- **Large world mode** — the `double-precision` cargo feature mirrors `BOX2D_DOUBLE_PRECISION`.

The approach follows
[HOW_WE_PORTED_CLIPPER2.md](https://github.com/larsbrubaker/clipper2-rust/blob/main/HOW_WE_PORTED_CLIPPER2.md)
from our Clipper2 port.

## Development

```bash
# Clone with the C reference submodule
git clone --recurse-submodules https://github.com/larsbrubaker/box2d-rust.git

# Run tests (both precision modes)
cargo test
cargo test --features double-precision

# Full pre-commit gauntlet: file lengths, tests, fmt, clippy, build
./scripts/pre-commit-check.ps1   # or .sh
```

### Demo site

The demo site (`demo/`) mirrors the upstream `samples` app in the browser via WebAssembly.

The quickest way to see it — builds the wasm and serves the demo at
`http://localhost:3000`, opening your browser:

```
run_demo.cmd      # Windows (double-click or run from a terminal)
./run_demo.sh     # Linux / macOS
```

Or drive the steps yourself:

```bash
cd demo
bun install
bun run build:wasm   # wasm-pack build (once, and after Rust changes)
bun run dev          # dev server at http://localhost:3000, rebuilds wasm on Rust edits
```

Deployed automatically to GitHub Pages on push to `main`.

## Acknowledgments

- [Erin Catto](https://github.com/erincatto) — Box2D author
- [MatterHackers](https://www.matterhackers.com/) — sponsoring the port
- Ported with [Claude Code](https://claude.com/claude-code)
