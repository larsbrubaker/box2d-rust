// Sample registry — one entry per C `RegisterSample( category, name, … )` in
// box2d-cpp-reference/samples/sample_*.cpp, plus the Replay viewer registered via
// `RegisterReplay` (not `RegisterSample` — see sample_replay.cpp / sample.cpp).
// This is the single source of truth for the category→sample tree, the Samples
// menu, prev/next ordering, and (once Phase 1+ lands) registry-derived navigation.
// Statuses are honest, per this convention:
//   `live`    — a route+scene exists and matches the C sample's scene, values,
//               controls, and camera with no undisclosed divergence.
//   `partial` — a route exists but has a *disclosed* divergence from C.
//   `planned` — no route yet (or the current demo page is an invented composite
//               that does not map 1:1 to this RegisterSample entry).
//
// Enumerated from the pinned submodule (Box2D v3.1.1+, 56edae7 / 56edae79f2949d86142b03450d5d60f63bcf5a6f).
// 138 active `RegisterSample` / `RegisterSampleWithCapacity` entries across
// 14 categories, plus 1 `RegisterReplay` entry (Replay / Viewer) → 139 total,
// 15 categories. No `RegisterSample` calls are `#if 0`'d at this pin.
//
// Phase 0 note: every entry is `planned` with no `route`/`scene`. Current
// demo pages under `demo/src/demos/` are invented composites (or about pages)
// and must not be claimed as live/partial until a scene is ported 1:1. The
// invented `Math` demo has no C counterpart and is intentionally absent here.
//
// ---------------------------------------------------------------------------
// Single-registration pattern (registry ↔ multi-scene page link)
// ---------------------------------------------------------------------------
// This registry is the ONE source of truth for the category→sample tree AND for
// the scene key each sample maps to inside its hosting page. A multi-scene page
// never keeps a second, private list that has to be edited in lockstep with this
// file. Instead it validates its internal scene table against the registry at
// page-init with `assertRouteScenes(route, [...its scene keys])`, which
// `console.error`s the moment the two drift apart — no silent default fallback
// (see `scenesFor` / `assertRouteScenes` below). The CI twin is
// `tests/registry.test.ts`.
//
// To add a sample a category agent does exactly TWO things:
//   1. Add ONE `RegisterSample`-mirroring entry here (name, status, route?,
//      scene?), placed in its category `cat(...)` block — flip `planned` →
//      `live`/`partial` and set `route` + `scene`.
//   2. Implement that `scene` in the page named by `route` (its reset/camera/
//      controls) and include the scene key in the array passed to
//      `assertRouteScenes` (and export `SCENES` for the parity test).
// Nothing else needs to stay in sync: the tree, Samples menu, prev/next order,
// deep links, and home grid all derive from this array.

export type SampleStatus = "live" | "partial" | "planned";

export interface SampleEntry {
  /** C category string (first RegisterSample / RegisterReplay arg). */
  category: string;
  /** C sample name (second RegisterSample / RegisterReplay arg). */
  name: string;
  /** URL-safe slug derived from the name; unique within a category. */
  slug: string;
  /** Hash route of the hosting demo page, when one exists. */
  route?: string;
  /** Scene key within a multi-scene page (the page's `mode`/`scene` value). */
  scene?: string;
  /** Honest port status. */
  status: SampleStatus;
  /** C source file the sample lives in (for "view C source" links). */
  cSource: string;
}

/** name → lowercase-hyphen slug. "Ball & Chain" → "ball-chain". */
export function slugify(name: string): string {
  return name
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");
}

// [name, status, route?, scene?]
type Spec = [string, SampleStatus, string?, string?];

function cat(category: string, cSource: string, specs: Spec[]): SampleEntry[] {
  return specs.map(([name, status, route, scene]) => ({
    category,
    name,
    slug: slugify(name),
    status,
    route,
    scene,
    cSource,
  }));
}

/**
 * The full inventory. Grouped by category for readability; `SAMPLES_SORTED`
 * applies the C sort order (Category then Name via strcmp).
 */
export const SAMPLES: SampleEntry[] = [
  ...cat("Bodies", "sample_bodies.cpp", [
    // live: joints/types/enable match C. partial rows disclose binding gaps.
    ["Body Type", "live", "bodies", "body-type"],
    ["Weeble", "partial", "bodies", "weeble"], // no friction/restitution callbacks / SetMassData
    ["Sleep", "partial", "bodies", "sleep"], // no sensor events / sleepThreshold / enableSleep
    ["Bad", "live", "bodies", "bad"],
    ["Pivot", "live", "bodies", "pivot"],
    ["Kinematic", "partial", "bodies", "kinematic"], // SetTargetTransform → SetTransform snap
    ["Mixed Locks", "partial", "bodies", "mixed-locks"], // motionLocks unbound
    ["Set Velocity", "live", "bodies", "set-velocity"],
    ["Wake Touching", "live", "bodies", "wake-touching"],
  ]),
  ...cat("Benchmark", "sample_benchmark.cpp", [
    // All live ports use C DEBUG / wasm-scaled counts (disclosed) → partial.
    ["Barrel", "partial", "benchmark", "barrel"], // DEBUG rows/cols; Human shape deferred
    ["Barrel 2.4", "partial", "benchmark", "barrel-2-4"], // DEBUG numj=5
    ["Compounds", "partial", "benchmark", "compounds"], // DEBUG 10×40
    ["Tumbler", "partial", "benchmark", "tumbler"], // DEBUG grid 20
    ["Washer", "partial", "benchmark", "washer"], // DEBUG grid 20; hit events approx
    ["Many Tumblers", "partial", "benchmark", "many-tumblers"], // DEBUG 2×2×8
    ["Large Pyramid", "partial", "benchmark", "large-pyramid"], // DEBUG base 20
    ["Many Pyramids", "partial", "benchmark", "many-pyramids"], // DEBUG 5×5
    ["CreateDestroy", "partial", "benchmark", "create-destroy"], // DEBUG base 40
    ["Sleep", "partial", "benchmark", "sleep"], // DEBUG base 40
    ["Joint Grid", "partial", "benchmark", "joint-grid"], // DEBUG N=20
    ["Smash", "partial", "benchmark", "smash"], // DEBUG 20×10
    ["Large Compounds", "partial", "benchmark", "large-compounds"], // DEBUG ground/span
    ["Kinematic", "partial", "benchmark", "kinematic"], // DEBUG span 20
    ["Cast", "planned"], // needs world ray/circle cast + overlap query APIs
    ["Spinner", "partial", "benchmark", "spinner"], // DEBUG 499; chain friction default
    ["Rain", "planned"], // needs CreateHuman
    ["Shape Distance", "planned"], // needs b2ShapeDistance bind
    ["Sensor", "planned"], // needs custom filter + sensor-event shape flags
    ["Capacity", "partial", "benchmark", "capacity"], // wall-clock vs b2Profile.step
    ["Junkyard", "partial", "benchmark", "junkyard"], // DEBUG rowCount 2
  ]),
  ...cat("Character", "sample_character.cpp", [
    ["Mover", "planned"],
  ]),
  ...cat("Collision", "sample_collision.cpp", [
    ["Shape Distance", "planned"],
    ["Dynamic Tree", "planned"],
    ["Ray Cast", "planned"],
    ["Cast World", "planned"],
    ["Overlap World", "planned"],
    ["Manifold", "planned"],
    ["Smooth Manifold", "planned"],
    ["Shape Cast", "planned"],
    ["Time of Impact", "planned"],
  ]),
  ...cat("Continuous", "sample_continuous.cpp", [
    ["Bounce House", "live", "continuous", "bounce-house"],
    ["Bounce Humans", "planned"], // needs CreateHuman (shared/human.c)
    ["Chain Drop", "live", "continuous", "chain-drop"],
    ["Chain Slide", "live", "continuous", "chain-slide"],
    ["Segment Slide", "live", "continuous", "segment-slide"],
    ["Skinny Box", "live", "continuous", "skinny-box"],
    ["Ghost Bumps", "live", "continuous", "ghost-bumps"],
    ["Speculative Fallback", "live", "continuous", "speculative-fallback"],
    ["Speculative Sliver", "live", "continuous", "speculative-sliver"],
    ["Speculative Ghost", "live", "continuous", "speculative-ghost"],
    ["Pixel Imperfect", "live", "continuous", "pixel-imperfect"],
    ["Restitution Threshold", "live", "continuous", "restitution-threshold"],
    // Partial: Scene3 ragdoll needs CreateHuman; Scenes 1/2/4 + C/V/S keys match
    ["Drop", "partial", "continuous", "drop"],
    ["Pinball", "live", "continuous", "pinball"],
    ["Wedge", "live", "continuous", "wedge"],
  ]),
  ...cat("Determinism", "sample_determinism.cpp", [
    ["Falling Hinges", "planned"],
    ["SnapShot", "planned"],
  ]),
  ...cat("Events", "sample_events.cpp", [
    // Partial: donut path only — CreateHuman not bound
    ["Sensor Funnel", "partial", "events", "sensor-funnel"],
    ["Sensor Bookend", "live", "events", "sensor-bookend"],
    ["Foot Sensor", "live", "events", "foot-sensor"],
    ["Contact", "live", "events", "contact"],
    ["Platformer", "live", "events", "platformer"],
    ["Body Move", "live", "events", "body-move"],
    ["Sensor Types", "live", "events", "sensor-types"],
    ["Joint", "live", "events", "joint"],
    ["Persistent Contact", "live", "events", "persistent-contact"],
    // Partial: prismatic motor reverse uses body-x approx (no GetTranslation bind)
    ["Sensor Hits", "partial", "events", "sensor-hits"],
    ["Projectile Event", "live", "events", "projectile-event"],
    ["Circle Impulse", "live", "events", "circle-impulse"],
  ]),
  ...cat("Geometry", "sample_geometry.cpp", [
    ["Convex Hull", "planned"],
  ]),
  ...cat("Issues", "sample_issues.cpp", [
    ["Bad Steiner", "planned"],
    ["Disable", "planned"],
    ["Crash01", "planned"],
    ["StaticVsBulletBug", "planned"],
    ["Unstable Prismatic Joints", "planned"],
    ["Unstable Windmill", "planned"],
  ]),
  ...cat("Joints", "sample_joints.cpp", [
    ["Distance Joint", "live", "joints", "distance-joint"],
    ["Motor Joint", "live", "joints", "motor-joint"],
    // Partial: remainder==3 RandomPolygon branch uses square (hull attach not wired for mid-body)
    ["Top Down Friction", "partial", "joints", "top-down-friction"],
    ["Filter Joint", "live", "joints", "filter-joint"],
    ["Revolute", "live", "joints", "revolute"],
    ["Prismatic", "live", "joints", "prismatic"],
    ["Wheel", "live", "joints", "wheel"],
    ["Bridge", "live", "joints", "bridge"],
    // Partial: category/mask filter bits → groupIndex -1 approximation for link-link
    ["Ball & Chain", "partial", "joints", "ball-chain"],
    ["Cantilever", "live", "joints", "cantilever"],
    ["Motion Locks", "live", "joints", "motion-locks"],
    // Partial: force-threshold break on 2 joints; C six-joint gallery incomplete
    ["Breakable", "partial", "joints", "breakable"],
    // Partial: HUD separations only — C Separation construction not fully mirrored
    ["Separation", "partial", "joints", "separation"],
    // Partial: simplified impulse constraint toward (0,5); C has richer dual-impulse setup
    ["User Constraint", "partial", "joints", "user-constraint"],
    // Partial: Car + bumps; missing teeter/bridge/truck/chase-cam extras
    ["Driving", "partial", "joints", "driving"],
    ["Ragdoll", "planned"], // needs CreateHuman (shared/human.c)
    ["Soft Body", "live", "joints", "soft-body"],
    ["Doohickey", "live", "joints", "doohickey"],
    ["Scissor Lift", "planned"], // large; deferred this session
    ["Gear Lift", "planned"], // large; deferred this session
    // Partial: single hinged panel; C two-joint/latch extras missing
    ["Door", "partial", "joints", "door"],
    ["Scale Ragdoll", "planned"], // needs Human_SetScale
  ]),
  ...cat("Robustness", "sample_robustness.cpp", [
    ["HighMassRatio1", "live", "robustness", "high-mass-ratio1"],
    ["HighMassRatio2", "live", "robustness", "high-mass-ratio2"],
    ["HighMassRatio3", "live", "robustness", "high-mass-ratio3"],
    ["Overlap Recovery", "live", "robustness", "overlap-recovery"],
    ["Tiny Pyramid", "live", "robustness", "tiny-pyramid"],
    ["Cart", "live", "robustness", "cart"],
    ["Multiple Prismatic", "live", "robustness", "multiple-prismatic"],
  ]),
  ...cat("Shapes", "sample_shapes.cpp", [
    // partial: friction slider updates dynamic shape only (no chain_SetSurfaceMaterial bind)
    ["Chain Shape", "partial", "shapes", "chain-shape"],
    ["Chain Segment", "live", "shapes", "chain-segment"],
    // partial: Body AABBs overlay approximates extents (no b2Body_ComputeAABB)
    ["Compound Shapes", "partial", "shapes", "compound-shapes"],
    ["Filter", "live", "shapes", "filter"],
    ["Custom Filter", "live", "shapes", "custom-filter"],
    ["Restitution", "live", "shapes", "restitution"],
    ["Friction", "live", "shapes", "friction"],
    ["Rolling Resistance", "live", "shapes", "rolling-resistance"],
    ["Conveyor Belt", "live", "shapes", "conveyor-belt"],
    ["Tangent Speed", "live", "shapes", "tangent-speed"],
    ["Modify Geometry", "live", "shapes", "modify-geometry"],
    ["Chain Link", "live", "shapes", "chain-link"],
    ["Rounded", "live", "shapes", "rounded"],
    ["Ellipse", "live", "shapes", "ellipse"],
    ["Offset", "live", "shapes", "offset"],
    ["Explosion", "live", "shapes", "explosion"],
    ["Recreate Static", "live", "shapes", "recreate-static"],
    ["Box Restitution", "live", "shapes", "box-restitution"],
    // partial: hanging chain revolute local frames approximated via world pivot
    ["Wind", "partial", "shapes", "wind"],
  ]),
  ...cat("Stacking", "sample_stacking.cpp", [
    ["Single Box", "live", "stacking", "single-box"],
    ["Tilted Stack", "live", "stacking", "tilted-stack"],
    ["Vertical Stack", "live", "stacking", "vertical-stack"],
    ["Circle Stack", "live", "stacking", "circle-stack"],
    ["Capsule Stack", "live", "stacking", "capsule-stack"],
    ["Cliff", "live", "stacking", "cliff"],
    ["Arch", "live", "stacking", "arch"],
    ["Double Domino", "live", "stacking", "double-domino"],
    ["Confined", "live", "stacking", "confined"],
    ["Card House", "live", "stacking", "card-house"],
  ]),
  ...cat("World", "sample_world.cpp", [
    ["Tiles", "planned"],
    ["Far Pyramid", "planned"],
    ["Far Ragdolls", "planned"],
    ["Far Gate", "planned"],
  ]),
  // Replay is registered via RegisterReplay (sample_replay.cpp), not RegisterSample.
  // Kept as its own category so the Samples tree matches the C Replay menu entry.
  ...cat("Replay", "sample_replay.cpp", [
    ["Viewer", "planned"],
  ]),
];

/** ASCII/code-unit comparison — matches C `strcmp`, NOT locale-aware collation. */
function strcmp(a: string, b: string): number {
  return a < b ? -1 : a > b ? 1 : 0;
}

/** C main.cpp / sample.cpp CompareSamples: Category then Name (strcmp). */
export function compareSamples(a: SampleEntry, b: SampleEntry): number {
  return strcmp(a.category, b.category) || strcmp(a.name, b.name);
}

/** The inventory in the exact order the C Samples menu presents it. */
export const SAMPLES_SORTED: SampleEntry[] = [...SAMPLES].sort(compareSamples);

/** Categories in sorted (strcmp) order. */
export function categoryOrder(): string[] {
  const seen: string[] = [];
  for (const s of SAMPLES_SORTED) {
    if (!seen.includes(s.category)) seen.push(s.category);
  }
  return seen;
}

/** Sorted samples grouped by category, preserving sort order. */
export function samplesByCategory(): Map<string, SampleEntry[]> {
  const map = new Map<string, SampleEntry[]>();
  for (const s of SAMPLES_SORTED) {
    let list = map.get(s.category);
    if (!list) {
      list = [];
      map.set(s.category, list);
    }
    list.push(s);
  }
  return map;
}

export interface CategoryStats {
  live: number;
  partial: number;
  planned: number;
  total: number;
}

export function categoryStats(category: string): CategoryStats {
  const stats: CategoryStats = { live: 0, partial: 0, planned: 0, total: 0 };
  for (const s of SAMPLES) {
    if (s.category !== category) continue;
    stats.total += 1;
    stats[s.status] += 1;
  }
  return stats;
}

/** Aggregate live/partial/planned/total across the whole inventory. */
export function totalStats(): CategoryStats {
  const stats: CategoryStats = { live: 0, partial: 0, planned: 0, total: 0 };
  for (const s of SAMPLES) {
    stats.total += 1;
    stats[s.status] += 1;
  }
  return stats;
}

/** Entries with a working route (live + partial), in sorted order. */
export const NAVIGABLE_SAMPLES: SampleEntry[] = SAMPLES_SORTED.filter((s) => s.route);

/** Entries whose port is bit-exact with C (status === "live"), in sorted order. */
export const LIVE_SAMPLES: SampleEntry[] = SAMPLES_SORTED.filter((s) => s.status === "live");

/** Resolve a `#/<route>/<slug>` deep link to its entry. */
export function findByRouteSlug(route: string, slug: string): SampleEntry | undefined {
  return SAMPLES_SORTED.find((s) => s.route === route && s.slug === slug);
}

/** First routable entry whose page uses `route` (fallback when only a route is given). */
export function firstEntryForRoute(route: string): SampleEntry | undefined {
  return SAMPLES_SORTED.find((s) => s.route === route);
}

/**
 * Resolve a route + C sample name to its entry. Multi-scene pages call this when
 * the in-page selector switches scenes; the label is the registry `name`.
 */
export function findByRouteName(route: string, name: string): SampleEntry | undefined {
  return SAMPLES_SORTED.find((s) => s.route === route && s.name === name);
}

/** Canonical deep-link hash for an entry (`#/<route>/<slug>`). */
export function entryHref(entry: SampleEntry): string {
  if (!entry.route) return "#/";
  if (!entry.slug) return `#/${entry.route}`;
  return `#/${entry.route}/${entry.slug}`;
}

/**
 * The neighbor of `entry` in the C-sorted navigable order (`NAVIGABLE_SAMPLES`),
 * clamped at the ends. `dir` = -1 previous, +1 next.
 */
export function neighborOf(entry: SampleEntry | undefined, dir: -1 | 1): SampleEntry | null {
  const list = NAVIGABLE_SAMPLES;
  if (list.length === 0) return null;
  let idx = entry ? list.findIndex((s) => s.route === entry.route && s.slug === entry.slug) : -1;
  if (idx === -1) idx = dir === 1 ? -1 : list.length;
  const next = Math.min(list.length - 1, Math.max(0, idx + dir));
  return list[next] ?? null;
}

/**
 * The pinned box2d-cpp-reference submodule commit (full hash), matching
 * `git -C box2d-cpp-reference rev-parse HEAD`. Used to build stable "C source"
 * links into the exact upstream sources this port mirrors.
 */
export const CPP_REFERENCE_COMMIT = "56edae79f2949d86142b03450d5d60f63bcf5a6f";

/** Upstream GitHub URL for the C sample file an entry was ported from, at the pin. */
export function cSourceUrl(entry: SampleEntry): string {
  return `https://github.com/erincatto/box2d/blob/${CPP_REFERENCE_COMMIT}/samples/${entry.cSource}`;
}

/**
 * Registry entries hosted by a multi-scene page `route`, in registry (C sort)
 * order — every navigable (live/partial with a route) entry for that route.
 * Entries whose `scene` is undefined (single-scene route-only hosts such as
 * Replay) are still returned so callers can tell a route apart from an empty
 * one; {@link assertRouteScenes} filters to entries that declare a `scene`.
 */
function scenesFor(route: string): SampleEntry[] {
  return NAVIGABLE_SAMPLES.filter((s) => s.route === route);
}

/**
 * Dev self-check for a multi-scene page: assert the page implements exactly the
 * scenes the registry declares for `route`. Logs `console.error` on any drift.
 * `extra` whitelists internal scenes intentionally not backed by a RegisterSample.
 *
 * Phase 0: no page calls this yet (all registry entries are planned). Category
 * ports wire it when they export `SCENES` and flip entries to live/partial.
 */
export function assertRouteScenes(
  route: string,
  implemented: readonly string[],
  extra: readonly string[] = [],
): string[] {
  const registryScenes = scenesFor(route)
    .map((e) => e.scene)
    .filter((s): s is string => s != null);
  const impl = new Set(implemented);
  const allowed = new Set([...registryScenes, ...extra]);
  for (const s of registryScenes) {
    if (!impl.has(s)) {
      console.error(
        `[registry] route "${route}": registry declares scene "${s}" but the page does not implement it`,
      );
    }
  }
  for (const s of implemented) {
    if (!allowed.has(s)) {
      console.error(
        `[registry] route "${route}": page implements scene "${s}" with no matching registry entry`,
      );
    }
  }
  return registryScenes;
}
