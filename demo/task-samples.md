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

**Merge note:** parallel Phase 2 category branches collided on `registry.ts`,
`PAGES` in `registry.test.ts`, and this tracker — coordinator owned those
merges; category edits stayed scoped and additive.

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

**Current totals:** Exact **110** · Partial **29** · Missing **0**.
Phase 0 baseline was Exact 0 · Partial 0 · Missing 139 (all planned, empty
`PAGES`).

## Per-category progress

| Category (C file) | Total | Exact | Partial | Missing | Notes |
|---|---|---|---|---|---|
| Benchmark (`sample_benchmark.cpp`) | 21 | 1 | 20 | 0 | DEBUG/wasm counts disclosed. Exact: Sensor. Partial: Barrel (Human via CreateHuman), Barrel 2.4, Compounds, Tumbler, Washer, Many Tumblers, Large/Many Pyramid(s), CreateDestroy, Sleep, Joint Grid, Smash, Large Compounds, Kinematic, Cast (DEBUG grid/queries), Spinner, Rain (DEBUG CreateRain), Shape Distance (DEBUG count), Capacity (wall-clock vs profile), Junkyard. |
| Bodies (`sample_bodies.cpp`) | 9 | 9 | 0 | 0 | All Exact (Phase 3): Weeble mix callbacks + SetMassData; Sleep sensors/thresholds; Kinematic SetTargetTransform; Mixed Locks motionLocks. |
| Character (`sample_character.cpp`) | 1 | 1 | 0 | 0 | Exact: Mover (C SolveMove + scene) |
| Collision (`sample_collision.cpp`) | 9 | 8 | 1 | 0 | Exact: Shape Distance, Ray Cast, Cast World, Overlap World, Manifold, Smooth Manifold, Shape Cast, Time of Impact. Partial: Dynamic Tree (C debug 100A-100 grid, not release 1000A-1000). Invented `#/manifolds` fully retired (route + wasm helper gone). |
| Continuous (`sample_continuous.cpp`) | 15 | 15 | 0 | 0 | Exact: Bounce House, Bounce Humans (CreateHuman), Chain Drop/Slide, Segment Slide, Skinny Box, Ghost Bumps, Speculative Fallback/Sliver/Ghost, Pixel Imperfect, Restitution Threshold, Drop (Scenes 1–4 incl. ragdoll), Pinball, Wedge. Invented bullet/wall composite replaced. |
| Determinism (`sample_determinism.cpp`) | 2 | 2 | 0 | 0 | Exact: Falling Hinges, SnapShot on `#/determinism` |
| Events (`sample_events.cpp`) | 12 | 12 | 0 | 0 | All Exact (Phase 3): Sensor Hits uses prismatic GetTranslation. |
| Geometry (`sample_geometry.cpp`) | 1 | 1 | 0 | 0 | Exact: Convex Hull on `#/geometry` (invented Geometry Queries retired) |
| Issues (`sample_issues.cpp`) | 6 | 6 | 0 | 0 | All 6 RegisterSample scenes live on `#/issues` |
| Joints (`sample_joints.cpp`) | 22 | 16 | 6 | 0 | Exact + Ball & Chain (category/mask). Partial: Top Down Friction, Breakable, Separation, User Constraint, Driving, Door. |
| Replay (`sample_replay.cpp`) | 1 | 0 | 1 | 0 | Via `RegisterReplay`. Partial Viewer on `#/replay` (route-only): transport/scrub/draw live; no inspector/query index/keyframe popup |
| Robustness (`sample_robustness.cpp`) | 7 | 7 | 0 | 0 | Exact: HighMassRatio1/2/3, Overlap Recovery, Tiny Pyramid, Cart, Multiple Prismatic. Invented composite replaced by multi-scene `#/robustness`. |
| Shapes (`sample_shapes.cpp`) | 19 | 19 | 0 | 0 | All Exact (Phase 3): Chain Shape surface material; Compound ComputeAABB; Wind revolute local frames. |
| Stacking (`sample_stacking.cpp`) | 10 | 10 | 0 | 0 | All 10 RegisterSample scenes live on `#/stacking` |
| World (`sample_world.cpp`) | 4 | 3 | 1 | 0 | Exact: Far Pyramid, Far Ragdolls (CreateHuman), Far Gate. Partial: Tiles (DEBUG cycleCount=10; CreateHuman Exact). Invented `#/world` composite replaced. |
| **Total** | **139** | **110** | **29** | **0** | Bodies 9 + Stacking 10 + Joints 16/6 + Shapes 19 + Continuous 15 + Events 12 + Benchmark 1/20 + Robustness 7 + Collision 8/1 + Issues 6 + Determinism 2 + Replay 0/1 + Geometry 1 + Character 1 + World 3/1 |

## Non-sample About pages

These routes are **not** C `RegisterSample` entries. They live under **About**
in the sidebar; the Samples tree is registry-only. Phase 3 batch A removed the
duplicate Labs links that pointed at categories the Samples tree already owns,
and fully retired `#/manifolds`.

| Route | Why |
|---|---|
| `#/math` | No C sample — deterministic math showcase. About page |
| `#/roadmap` | Meta progress page, not a C sample |
| ~~`#/manifolds`~~ | **Deleted** — C Manifold / Smooth Manifold live under `#/collision` |
| Former category composites / Labs dupes | All retired — `#/bodies`, `#/stacking`, `#/joints`, `#/shapes`, `#/continuous`, `#/events`, `#/benchmark`, `#/robustness`, `#/collision`, `#/issues`, `#/determinism`, `#/replay`, `#/geometry`, `#/character`, `#/world` are Samples-tree hosts only |

## Phases

### Phase 0 — Registry + parity contract — DONE

- [x] `demo/src/registry.ts` — full C inventory, pin recorded, helpers + `assertRouteScenes`
- [x] `demo/tests/registry.test.ts` — internal consistency + `PAGES` bidirectional scaffold
  (started empty; filled as categories landed in Phase 2)
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
  (`#/<route>/<slug>`); Math retired to About; invented composites demoted then
  later replaced by C ports
- [x] `assertRouteScenes` scaffolding — first live category filled `SCENES` /
  `PAGES`; contract now covers all multi-scene hosts
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

### Phase 2 — Category ports — DONE

All 15 categories have registry-backed routes. Remaining planned gaps closed in
Phase 3 batch C (Benchmark Cast / Shape Distance / Sensor; Joints Scissor /
Gear Lift). Phase 3 Partial upgrades brought totals to Exact **110** / Partial
**29** / Missing **0**.

### Phase 3 — Shell polish + retire invented pages — IN PROGRESS

**Missing-complete:** planned inventory is **0**; the Missing list is closed
(CreateHuman batch + batch C Cast / Shape Distance / Sensor / Scissor / Gear Lift).
**Partial upgrades (this branch):** Bodies Weeble/Sleep/Kinematic/Mixed Locks,
Shapes Chain/Compound/Wind, Events Sensor Hits, Joints Ball & Chain → Exact.
Left as Partial: DEBUG-count Benchmarks / Dynamic Tree / World Tiles, incomplete
Joints galleries (Top Down / Breakable / Separation / User Constraint / Driving /
Door), Replay inspector polish.

- [x] **Batch A:** Remove Labs sidebar duplicates; fully retire `#/manifolds`
  (route, `demos/manifolds.ts`, `collide_with_box` wasm helper); keep About
  `#/math` / `#/roadmap`; refresh this tracker
- [ ] C-faithful menu/info/debug draw polish (Partial / shell — not Missing)
- [x] Close remaining planned gaps (CreateHuman batch + Cast/Sensor/lifts) — Missing-complete
- [x] Partial→Exact binding-completeable upgrades (Bodies/Shapes/Sensor Hits/Ball & Chain)

## How the parity test tightens

`PAGES` lists every multi-scene host. Bidirectional checks:

1. Registry rows gain `route` / `scene` and `live`|`partial`
2. Page exports `SCENES` and calls `assertRouteScenes(route, SCENES)`
3. Add `{ route: { scenes: SCENES } }` to `PAGES` in `registry.test.ts`
4. Bidirectional drift fails CI if either side moves alone

Also enforced: pin hash, inventory counts, unique names/slugs, sort order,
live/partial ↔ scene/PAGES contract (Replay route-only exception).

## Decisions (Phase 0)

- **Replay**: included via `RegisterReplay` as category `Replay` / name `Viewer`
  (same special-case treatment as box3d); now `partial` route-only
- **Math**: excluded from the registry; About page only
- **Many Pyramids**: included (`RegisterSampleWithCapacity`)
- **Conservative status**: live/partial only with disclosed divergences — no
  silent thematic overlap claims
