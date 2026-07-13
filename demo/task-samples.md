# Sample conformance tracker (Phase 0+)

**Port Erin Catto's Box2D Samples App 1:1.** Do **not** invent new demos.
Every interactive scene must map to a `RegisterSample(category, name, …)` entry
in `box2d-cpp-reference/samples/` (or `RegisterReplay` for Replay / Viewer).
Same scene construction, same per-sample controls, same camera defaults, closest
feasible match to the C app's rendering and UI shell.

Single source of truth: [`demo/src/registry.ts`](src/registry.ts).
CI drift guard: [`demo/tests/registry.test.ts`](tests/registry.test.ts)
(`bun test` runs in `.github/workflows/ci.yml` on PRs and in
`deploy-demo.yml` before the Pages build).
Runtime twin: `assertRouteScenes(route, SCENES)` (wired per page as categories
are ported).

**Merge note:** parallel Phase 2 category branches will collide on
`registry.ts`, `PAGES` in `registry.test.ts`, and this tracker — the
coordinator owns those merges; keep category edits scoped and additive.

## Pin

Confirmed: **`56edae7`** (`56edae79f2949d86142b03450d5d60f63bcf5a6f`) —
Box2D v3.1.1+, submodule `box2d-cpp-reference/`.
Message: *More doubles clean up and testing (#1070)*.

Inventory at this pin:

- **138** `RegisterSample` / `RegisterSampleWithCapacity` entries across **14**
  categories
- **+1** `RegisterReplay("Replay", "Viewer", …)` (not `RegisterSample` — same
  special case box3d handled)
- **139** registry rows, **15** categories total
- No `RegisterSample` calls are `#if 0`'d at this pin

## Status legend

| Status | Meaning |
|---|---|
| Exact (`live`) | Route+scene exists; matches C scene/values/controls/camera with no undisclosed divergence |
| Partial | Route exists; disclosed divergence from C |
| Missing (`planned`) | No faithful route yet |

Phase 0 assignment: **Exact 0 · Partial 0 · Missing 139.** Current demo pages
are invented composites / capability showcases — none are claimed as live or
partial until a scene is ported 1:1.

## Per-category progress

| Category (C file) | Total | Exact | Partial | Missing | Notes |
|---|---|---|---|---|---|
| Benchmark (`sample_benchmark.cpp`) | 21 | 0 | 0 | 21 | Includes `RegisterSampleWithCapacity` "Many Pyramids" |
| Bodies (`sample_bodies.cpp`) | 9 | 0 | 0 | 9 | Current `#/bodies` is an invented shower composite |
| Character (`sample_character.cpp`) | 1 | 0 | 0 | 1 | C has only "Mover" |
| Collision (`sample_collision.cpp`) | 9 | 0 | 0 | 9 | Manifold / queries live under Collision in C; current `#/manifolds` + `#/geometry` are invented |
| Continuous (`sample_continuous.cpp`) | 15 | 0 | 0 | 15 | Current `#/continuous` is an invented bullet/wall composite |
| Determinism (`sample_determinism.cpp`) | 2 | 0 | 0 | 2 | Falling Hinges, SnapShot — current `#/determinism` is invented |
| Events (`sample_events.cpp`) | 12 | 0 | 0 | 12 | Current `#/events` is invented |
| Geometry (`sample_geometry.cpp`) | 1 | 0 | 0 | 1 | Convex Hull only |
| Issues (`sample_issues.cpp`) | 6 | 0 | 0 | 6 | No current demo route |
| Joints (`sample_joints.cpp`) | 22 | 0 | 0 | 22 | Current `#/joints` is invented hinge-chain composite |
| Replay (`sample_replay.cpp`) | 1 | 0 | 0 | 1 | Via `RegisterReplay`, not `RegisterSample`. Current `#/replay` is invented |
| Robustness (`sample_robustness.cpp`) | 7 | 0 | 0 | 7 | Current `#/robustness` borrows OverlapRecovery ideas but is a composite — stays Missing |
| Shapes (`sample_shapes.cpp`) | 19 | 0 | 0 | 19 | Current `#/shapes` is invented |
| Stacking (`sample_stacking.cpp`) | 10 | 0 | 0 | 10 | Current `#/stacking` is invented pyramid composite |
| World (`sample_world.cpp`) | 4 | 0 | 0 | 4 | Current `#/world` is invented |
| **Total** | **139** | **0** | **0** | **139** | |

## Invented demos to retire from Samples nav

These routes exist today but are **not** C `RegisterSample` entries. Keep them
reachable as about/lab pages if useful, but remove them from the Samples tree
once registry-driven nav lands:

| Route | Why |
|---|---|
| `#/math` | No C sample — deterministic math showcase. Retire from Samples nav; may remain as a non-sample about page |
| `#/roadmap` | Meta progress page, not a C sample |
| `#/manifolds` | Invented collision/manifold playground; C's Manifold / Smooth Manifold live under **Collision** |
| Category composites (`#/bodies`, `#/stacking`, `#/joints`, `#/events`, `#/continuous`, `#/shapes`, `#/world`, `#/determinism`, `#/robustness`, `#/benchmark`, `#/character`, `#/geometry`, `#/replay`) | Capability demos, not 1:1 ports — replace scene-by-scene as categories are ported |

## Phases

### Phase 0 — Registry + parity contract (this doc) — DONE when landed

- [x] `demo/src/registry.ts` — full C inventory, pin recorded, helpers + `assertRouteScenes`
- [x] `demo/tests/registry.test.ts` — internal consistency + empty `PAGES` bidirectional scaffold
- [x] This tracker

### Phase 1 — Harness parity

Structural blockers before faithful sample ports:

- Shared sample harness (camera / view defaults matching C `SetView`, pause/reset,
  debug draw flags, info panel, grab/select if applicable)
- WASM bindings for per-sample construction APIs still missing or incomplete
  (shape/joint helpers, sensors, continuous toggles, world draw dump, etc.)
- Registry-driven Samples menu + deep links (`#/<route>/<slug>`) replacing the
  flat invented nav
- Wire `assertRouteScenes` + export `SCENES` on the first multi-scene page; add
  that route to `PAGES` in the parity test

### Phase 2 — Category ports

Port one C category at a time (Bodies / Stacking recommended first — small and
core). For each sample: flip `planned` → `live`/`partial`, set `route`+`scene`,
implement the scene, keep the parity test green. Update the table above.

### Phase 3 — Shell polish + retire invented pages

C-faithful menu/info/debug draw; remove or demote remaining invented composites
from Samples nav; Math stays optional non-sample.

## How the parity test tightens

Today (`PAGES = {}`):

- Pin hash, inventory counts, unique names/slugs, sort order, planned-only
  contract
- "Every multi-scene registry route is covered" — vacuously true until the first
  `route`+`scene` lands

When a category is ported:

1. Registry rows gain `route` / `scene` and `live`|`partial`
2. Page exports `SCENES` and calls `assertRouteScenes(route, SCENES)`
3. Add `{ route: { scenes: SCENES } }` to `PAGES` in `registry.test.ts`
4. Bidirectional drift fails CI if either side moves alone

## Decisions (Phase 0)

- **Replay**: included via `RegisterReplay` as category `Replay` / name `Viewer`,
  status `planned` (same special-case treatment as box3d)
- **Math**: excluded from the registry; tracked above as invent-to-retire
- **Many Pyramids**: included (`RegisterSampleWithCapacity`)
- **Conservative status**: nothing marked live/partial despite partial thematic
  overlap (e.g. robustness ↔ OverlapRecovery)
