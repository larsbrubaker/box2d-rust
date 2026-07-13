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
| Benchmark (`sample_benchmark.cpp`) | 21 | 0 | 17 | 4 | DEBUG/wasm counts disclosed. Exact 0. Partial: Barrel (no Human), Barrel 2.4, Compounds, Tumbler, Washer, Many Tumblers, Large/Many Pyramid(s), CreateDestroy, Sleep, Joint Grid, Smash, Large Compounds, Kinematic, Spinner, Capacity (wall-clock vs profile), Junkyard. Missing: Cast, Rain (CreateHuman), Shape Distance, Sensor |
| Bodies (`sample_bodies.cpp`) | 9 | 5 | 4 | 0 | Exact: Body Type, Bad, Pivot, Set Velocity, Wake Touching. Partial: Weeble (no mass-data/friction callbacks), Sleep (no sensor events / sleepThreshold / enableSleep), Kinematic (SetTransform snap), Mixed Locks (motionLocks unbound). Invented shower retired from Labs. |
| Character (`sample_character.cpp`) | 1 | 1 | 0 | 0 | Exact: Mover (C SolveMove + scene) |
| Collision (`sample_collision.cpp`) | 9 | 8 | 1 | 0 | Exact: Shape Distance, Ray Cast, Cast World, Overlap World, Manifold, Smooth Manifold, Shape Cast, Time of Impact. Partial: Dynamic Tree (C debug 100A-100 grid, not release 1000A-1000). Invented `#/manifolds` retired from Labs. |
| Continuous (`sample_continuous.cpp`) | 15 | 13 | 1 | 1 | Exact: Bounce House, Chain Drop/Slide, Segment Slide, Skinny Box, Ghost Bumps, Speculative Fallback/Sliver/Ghost, Pixel Imperfect, Restitution Threshold, Pinball, Wedge. Partial: Drop (Scene3 ragdoll needs CreateHuman). Missing: Bounce Humans (CreateHuman). Invented bullet/wall composite replaced. |
| Determinism (`sample_determinism.cpp`) | 2 | 2 | 0 | 0 | Exact: Falling Hinges, SnapShot on `#/determinism` |
| Events (`sample_events.cpp`) | 12 | 10 | 2 | 0 | Exact: Sensor Bookend, Foot Sensor, Contact, Platformer, Body Move, Sensor Types, Joint, Persistent Contact, Projectile Event, Circle Impulse. Partial: Sensor Funnel (no CreateHuman — donut stand-in), Sensor Hits (prismatic motor reverse via body-x approx). Invented `#/events` composite replaced. |
| Geometry (`sample_geometry.cpp`) | 1 | 1 | 0 | 0 | Exact: Convex Hull on `#/geometry` (invented Geometry Queries retired) |
| Issues (`sample_issues.cpp`) | 6 | 6 | 0 | 0 | All 6 RegisterSample scenes live on `#/issues` |
| Joints (`sample_joints.cpp`) | 22 | 11 | 7 | 4 | Exact: Distance, Motor, Filter, Revolute, Prismatic, Wheel, Bridge, Cantilever, Motion Locks, Soft Body, Doohickey. Partial: Top Down Friction, Ball & Chain, Breakable, Separation, User Constraint, Driving, Door. Missing: Ragdoll, Scissor Lift, Gear Lift, Scale Ragdoll. Invented composite retired. |
| Replay (`sample_replay.cpp`) | 1 | 0 | 1 | 0 | Via `RegisterReplay`. Partial Viewer on `#/replay` (route-only): transport/scrub/draw live; no inspector/query index/keyframe popup |
| Robustness (`sample_robustness.cpp`) | 7 | 7 | 0 | 0 | Exact: HighMassRatio1/2/3, Overlap Recovery, Tiny Pyramid, Cart, Multiple Prismatic. Invented composite replaced by multi-scene `#/robustness`. |
| Shapes (`sample_shapes.cpp`) | 19 | 16 | 3 | 0 | Exact: Chain Segment, Filter, Custom Filter, Restitution, Friction, Rolling Resistance, Conveyor Belt, Tangent Speed, Modify Geometry, Chain Link, Rounded, Ellipse, Offset, Explosion, Recreate Static, Box Restitution. Partial: Chain Shape (no chain_SetSurfaceMaterial), Compound Shapes (approx Body AABBs), Wind (revolute local frames approx). Invented shapes composite replaced. |
| Stacking (`sample_stacking.cpp`) | 10 | 10 | 0 | 0 | All 10 RegisterSample scenes live on `#/stacking` |
| World (`sample_world.cpp`) | 4 | 0 | 0 | 4 | Current `#/world` is invented |
| **Total** | **139** | **90** | **36** | **13** | Bodies 5/4 + Stacking 10 + Joints 11/7/4 + Shapes 16/3 + Continuous 13/1/1 + Events 10/2 + Benchmark 0/17/4 + Robustness 7 + Collision 8/1 + Issues 6 + Determinism 2 + Replay 0/1 + Geometry 1 + Character 1 |

## Invented demos to retire from Samples nav

These routes exist today but are **not** C `RegisterSample` entries. Phase 1
moved them under **Labs** / **About** in the sidebar; the Samples tree is
registry-only. Math is under About (not Samples).

| Route | Why |
|---|---|
| `#/math` | No C sample — deterministic math showcase. **Retired from Samples**; About page |
| `#/roadmap` | Meta progress page, not a C sample |
| `#/manifolds` | **Retired from Labs** — C Manifold / Smooth Manifold live under `#/collision` |
| Category composites (`#/world`) | Capability demos / harness previews — replace scene-by-scene as categories are ported. `#/bodies`, `#/stacking`, `#/joints`, `#/shapes`, `#/continuous`, `#/events`, `#/benchmark`, `#/robustness`, `#/collision`, `#/issues`, `#/determinism`, `#/replay`, `#/geometry`, `#/character` now host C samples. |

## Phases

### Phase 0 — Registry + parity contract (this doc) — DONE when landed

- [x] `demo/src/registry.ts` — full C inventory, pin recorded, helpers + `assertRouteScenes`
- [x] `demo/tests/registry.test.ts` — internal consistency + empty `PAGES` bidirectional scaffold
- [x] This tracker

### Phase 1 — Harness parity — DONE on `demo/phase-1-harness`

Structural blockers before faithful sample ports:

- [x] Shared sample harness (`demo/src/demos/sample-shell.ts`): pause / single-step /
  restart (P/O/R), Hertz + sub-steps, C camera (`center` + `zoom` half-height)
- [x] Mouse grab via motor joint with C spring values (hertz 7.5, damping 1.0,
  force scale 100) — `demo/wasm/src/interact/` (box3d layout)
- [x] Incremental engine-driven debug draw (`b2World_Draw` → canvas adapter);
  solids + lines landed. **Deferred:** contacts, mass, bounds, text, chain
  normals, graph colors, islands, joint extras, view-flag menu bar
- [x] Registry-driven Samples tree (`#sample-tree`) + deep links
  (`#/<route>/<slug>`); Math retired to About; invented composites under Labs
- [x] `assertRouteScenes` scaffolding — `stacking.ts` exports empty `SCENES` and
  calls it; `PAGES` stays empty until the first live/partial multi-scene row
- [x] `bun test` wired in CI (`ci.yml` + `deploy-demo.yml`); live/partial ↔
  scene/PAGES contract with Replay route-only exception

**Bindings added on this branch (coordinator: reconcile vs `demo/bindings-sample-apis`):**
`SimWorld.mouse_down/move/up/active`, `set_grab_force_scale`, `collect_draw`,
`draw_polygons/circles/capsules/lines`. Shared surface: `demo/wasm/src/interact/`.

### Binding gaps vs Bodies / Stacking / Shapes / Joints (Phase 1)

Inventory of C APIs those four categories call vs current `SimWorld` wasm surface
(before `demo/bindings-sample-apis`). Prefer Rust/wasm additions; keep TS churn low.

**Already present (usable for some scenes):** static/dynamic box, circle, capsule,
chain, hinge (hardcoded falling-hinges tuning), distance joint, bullet + continuous
toggle, sensor box / events, explode, set gravity, snapshot/restore, mover queries.

| Gap | Needed by | Notes |
|---|---|---|
| Prismatic / wheel / weld / motor / filter joints | Joints (+ Bodies slider) | **Done** on `demo/bindings-sample-apis` |
| Flexible revolute (limits/motor/spring params) | Joints, Shapes | **Done** (`add_revolute_joint` / angled) |
| Segment shape | Bodies, Stacking, Shapes, Joints | **Done** (`add_segment` / `attach_segment`) |
| Offset / multi-shape polygons on one body | Shapes (tables/ships) | **Done** (`add_body` + `attach_*`) |
| Polygon from hull points | Shapes | **Done** (`add_polygon`) |
| Body transform / type / enable-disable | Bodies, Shapes | **Done** |
| Apply force / impulse / set velocities | Bodies, Stacking, Joints | **Done** |
| Sleep / warm starting / speculative toggles | Harness + World samples | **Done** (continuous was already present) |
| Contact tuning | Stacking, Joints | **Done** |
| Mouse grab (motor joint + kinematic proxy) | Harness (Sample.cpp) | **Owned by Phase 1 harness `interact/`** — not on this branch |
| Debug draw dump (`world_draw` → buffers) | Harness | **Owned by Phase 1 harness `interact/`** (`collect_draw`) |
| Shape filter / material / morph APIs | Shapes | Still missing — lower priority for first ports |
| Joint runtime setters / constraint readouts | Joints GUI | Still missing — follow once create surface exists |
| Custom friction/restitution/filter callbacks | Bodies, Shapes | Still missing — WASM callback bridging later |

### Phase 2 — Category ports

Port one C category at a time (Bodies / Stacking recommended first — small and
core). For each sample: flip `planned` → `live`/`partial`, set `route`+`scene`,
implement the scene, keep the parity test green. Update the table above.
When the first multi-scene page gains live rows: fill `SCENES`, add the route to
`PAGES` in `registry.test.ts`.

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
