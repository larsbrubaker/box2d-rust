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

Live WebAssembly demos running the ported engine: deterministic math, geometry queries,
contact manifolds, falling bodies, and stacking — with full `b2World_Step` simulation,
graph-colored constraint solving, and island sleeping.

> Part of the [rust-apps](https://github.com/larsbrubaker/rust-apps) suite — a collection of
> Rust graphics and geometry libraries by Lars Brubaker.

## Status: In progress

This is an exact behavioral port of the Box2D v3.1 C source, done module by module with the C
test suite ported alongside. The pinned reference source lives in the `box2d-cpp-reference/`
submodule.

| Module | Ported | Tests |
|---|---|---|
| math_functions | ✅ | ✅ (test_math.c) |
| core / constants | ✅ | ✅ |
| id (handles) | ✅ | ✅ (test_id.c) |
| bitset | ✅ | ✅ (test_bitset.c) |
| id_pool (index allocator) | ✅ | ✅ |
| types (world/body/shape/chain defs + defaults) | ✅ | ✅ |
| table (open-addressing hash set) | ✅ | ✅ (test_table.c) |
| aabb (perimeter, enlarge, offset, ray cast) | ✅ | ✅ (test_collision.c AABB subtests) |
| distance (GJK, shape cast, TOI, segment distance) | ✅ | ✅ (test_distance.c) |
| hull (quickhull convex hull) | ✅ | ✅ |
| geometry (shapes, mass, AABB, point tests, ray/shape casts) | ✅ | ✅ (test_shape.c) |
| manifold (contact generation, all shape pairs) | ✅ | ✅ (test_collision.c manifold subtests) |
| dynamic_tree (AABB tree: insert, rotate, query, ray/box cast, rebuild) | ✅ | ✅ (test_dynamic_tree.c) |
| broad_phase (proxy ops, move buffer, overlap, pair update → contact creation) | ✅ | ✅ |
| dynamics core data model (body/shape/contact/joint/island/solver_set/graph/world) | 🟡 | — |
| body lifecycle (create/destroy), shape lifecycle (create/destroy), mass data | 🟡 | ✅ |
| contact (create/destroy/narrow-phase update), constraint_graph, solver_set, island linking | 🟡 | ✅ |
| joints (distance, motor, filter, prismatic, revolute, weld, wheel + joint core) | ✅ | ✅ |
| contact_solver + solver (serial step pipeline) | ✅ | ✅ |
| sensor overlap, world step (b2World_Step, HelloWorld passing) | ✅ | ✅ (test_world.c: HelloWorld, EmptyWorld) |
| world public API (queries, casts, explosions, setters) | — | — |

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
