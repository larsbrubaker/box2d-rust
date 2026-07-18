# TODO — remaining gaps vs C reference

State snapshot (2026-07-14): `main` @ `e77b58b` (v1.2.0), clean tree,
submodule pinned at `56edae7`. Sample tracker: **Exact 118 · Partial 21 ·
Missing 0** (per-category table in `demo/task-samples.md`).

This list is what remains after the July 2026 gap audit (C headers vs `src/`,
C test suite vs Rust tests, C samples vs demo registry) **and** the follow-up
work that landed in v1.2.0. Everything from the original audit's "real work"
list is done: `contact_is_valid`/`contact_get_data` (`f6b1b70`),
`LargeWorldAABBTest` (`76d5025`, `src/manifold_tests.rs:92`), timer module +
filled `b2Profile` timings (`b072ccb`, `src/timer.rs`), Benchmark Capacity
Exact, the six Joints Partials → Exact (`abafa81`), Replay Viewer inspector /
query index / keyframe popup → Exact (`aba0a86`..`0a1f5d4`), and all C View
flags in the panel (`5b802a9`).

## 1. Tracker doc corrections (5 min, docs only)

`demo/task-samples.md` lags its own table after the Replay Exact upgrade:

- [x] Line ~43 headline says "Exact **117** · Partial **22**" — table says
  118 / 21. Same stale count in the Phase 2 paragraph.
- [x] "Audit follow-ups" section: the **Contact / AABB (lib)** bullet is not
  checked off, but both items shipped (`f6b1b70`, `76d5025`). Mark done.
- [x] "Decisions (Phase 0)" still says Replay is "now `partial` route-only" —
  it is Exact since `0a1f5d4`.

## 2. The 21 remaining Partials — perf-gated count scaling

All 21 are functionally complete ports running the C `m_isDebug` (reduced)
counts so wasm stays real-time; the only divergence is magnitude, disclosed
per-row in `demo/src/registry.ts:106-127,135,267`:

- 19 Benchmark samples (all but Sensor and Capacity)
- Collision / Dynamic Tree (100×100 grid vs release 1000×1000)
- World / Tiles (`cycleCount` 10 vs release 600)

These may stay Partial by design. If we want to close them, options:

- [x] Profile current wasm perf per sample; bump any counts that hold 60 fps
  to release values (some likely can: Sleep, CreateDestroy, Kinematic).
  Measured 2026-07-18 (desktop Chrome, release wasm): at C release counts
  Sleep = 33.4 ms/step (5050 awake), CreateDestroy = 17.3 ms/step +
  create/destroy overhead, Kinematic = 26.4 ms/step — all exceed the 16.7 ms
  60 fps budget, so no counts were bumped. Partials stay Partial by design.
- [ ] Or add a user-facing "release counts" toggle (default DEBUG) so the
  Exact scene is *reachable* even where it isn't real-time — would need a
  status-legend decision in the tracker first.
- Minor extra divergences noted in registry comments while touching these:
  Washer "hit events approx", Spinner "chain friction default".

## 3. Library non-goals (do NOT re-audit; intentional omissions)

Documented single-threaded / registry-less design decisions, listed here so
future gap sweeps don't re-flag them:

- Threading: `parallel_for.c`, `scheduler.c`, `test_thread.c`,
  `b2World_SetWorkerCount/GetWorkerCount`, `enqueueTask`/`finishTask`/
  `userTaskContext` in `b2WorldDef` (see `src/types/mod.rs`,
  `src/world/api.rs` headers).
- `b2World_DumpMemoryStats` — no allocation tracking (arena allocator
  replaced by native `Vec`/ownership, `src/core.rs` header).
- `arena_allocator.c`, `test_container.c` (`b2Array` → std `Vec`).
- `_GetWorld` accessors + world registry (`TestWorldRecycle`,
  `TestSetWorkerCount`) — world is passed explicitly.
- `b2SetAllocator`, `b2GetByteCount`, `b2SetAssertFcn`, `b2SetLogFcn`,
  `b2Yield` — allocator/diagnostic shims.
- Replay worker-count slider — N/A on serial wasm (disclosed in registry).

## 4. Housekeeping

- [x] Local branches `demo/phase-3-harness-parity`, `demo/phase-3-housekeeping`,
  `demo/phase-3-joints-replay-exact` all point at merged history
  (`2cf7fe4` or behind) — delete once confirmed fully merged into `main`.
  (verified gone 2026-07-18 — only `main` remains)

## 5. Performance roadmap (vs C, measured 2026-07-18)

Serial Rust vs serial C (`-w=1`), 10 benchmark scenes; geometric mean ≈ **1.9×
slower than C** (range 1.18–2.73×). Full per-scene table and methodology are in
the README `## Performance` section. Ordered by expected win:

- [ ] Capsule/segment manifold path (narrow phase) — ~4.6× on spinner; profile
  and optimize `collide` for capsule vs capsule/chain.
- [ ] Contact solver inner loops — ~2.7×; investigate bounds-check elimination
  in the Vec-indexed constraint arrays, memory layout, and whether MSVC is
  auto-vectorizing the C soft-constraint loops that rustc isn't.
- [ ] Dynamic tree refit + pair traversal (~1.5–2×).
- [ ] Re-measure after each change with `cargo run --release --example benchmark`
  vs the C app.
