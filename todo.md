# TODO — remaining gaps vs C reference

State snapshot (2026-07-18): `main` @ `e77b58b` (v1.2.0), clean tree,
submodule pinned at `56edae7`. Sample tracker: **Exact 139 · Partial 0 ·
Missing 0** — every C sample is a faithful port after the counts toggle
(`demo/release-counts-toggle`) and the Washer/Spinner divergence fixes
(`demo/washer-spinner-exact`). Per-category table in `demo/task-samples.md`.

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

## 2. Count-gated samples + last 2 divergences — RESOLVED (all Exact)

**Counts toggle shipped on `demo/release-counts-toggle`.** The count-gated ports
run the C `m_isDebug` (reduced) counts by default so wasm stays real-time, and now
expose the exact C release (NDEBUG) scene via a user-facing **COUNTS** toggle
(`demo/src/demos/counts.ts` → `pickCount(debug, release)`; default DEBUG). Both
counts are the C source's own gated values, so matching either is Exact. **19 of
the former 21 Partials flipped to Exact** (all 21 formerly disclosed in
`demo/src/registry.ts`): 17 Benchmark + Collision/Dynamic Tree
(100×100 / 1000×1000) + World/Tiles (`cycleCount` 10 / 600).

**Last 2 divergences closed on `demo/washer-spinner-exact`** → all 139 Exact:

- [x] Benchmark / Washer — `shapeDef.enableHitEvents = true` on the grid bodies
  (`benchmarks.c:680`). Demo calls `enable_body_hit_events(b, true)`
  (`b2Body_EnableHitEvents`) on each grid body after attaching its box.
- [x] Benchmark / Spinner — `chainDef` surface-material friction 0.1 on the ground
  loop (`benchmarks.c:375`). Demo uses `attach_chain_ex(..., 0.1)` (restitution/
  rolling/tangent 0, matching C's zeroed `b2SurfaceMaterial`).

- [x] Profile current wasm perf per sample; bump any counts that hold 60 fps
  to release values (some likely can: Sleep, CreateDestroy, Kinematic).
  Measured 2026-07-18 (desktop Chrome, release wasm): at C release counts
  Sleep = 33.4 ms/step (5050 awake), CreateDestroy = 17.3 ms/step +
  create/destroy overhead, Kinematic = 26.4 ms/step — all exceed the 16.7 ms
  60 fps budget, so no counts were bumped. The toggle makes the release scene
  *reachable* (disclosed "may run below real-time") without changing the default.
- [x] Add a user-facing "release counts" toggle (default DEBUG) so the Exact
  scene is *reachable* even where it isn't real-time — shipped on
  `demo/release-counts-toggle`; status-legend note added to the tracker.

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
