// Build-time guard that the sample registry (the single source of truth for the
// category→scene tree) and each multi-scene page's implemented scene table agree
// exactly, in both directions. This is the CI-enforced version of the page-init
// `assertRouteScenes` self-check (which only `console.error`s at runtime): here a
// drift fails `bun test`, so a RegisterSample row added without its page scene —
// or a scene key renamed on only one side — cannot reach main.
//
// Phase 2 filled `PAGES` for every multi-scene host. As further samples land:
//   1. Flip entries to live/partial and set route + scene in registry.ts
//   2. Export `SCENES` from the page and call `assertRouteScenes`
//   3. Keep that route in `PAGES` below
// The bidirectional checks then tighten automatically. This file also enforces
// registry internal consistency, the C pin metadata, and the live/partial ↔
// scene/PAGES contract (see ROUTE_ONLY_EXCEPTIONS below).

import { test, expect } from "bun:test";
import {
  CPP_REFERENCE_COMMIT,
  SAMPLES,
  SAMPLES_SORTED,
  categoryOrder,
  categoryStats,
  cSourceUrl,
  compareSamples,
  slugify,
  totalStats,
} from "../src/registry.ts";

/**
 * Each multi-scene route's page SCENES, plus any `extra` scene keys the page
 * implements that are intentionally not backed by a RegisterSample entry (mirrors
 * the `extra` arg to `assertRouteScenes`).
 *
 * Every multi-scene C category host is listed. Do not add invented composite
 * demos here; only registry-backed live/partial scene tables.
 */
import { SCENES as benchmarkScenes } from "../src/demos/benchmark.ts";
import { SCENES as bodiesScenes } from "../src/demos/bodies.ts";
import { SCENES as characterScenes } from "../src/demos/character.ts";
import { SCENES as collisionScenes } from "../src/demos/collision.ts";
import { SCENES as continuousScenes } from "../src/demos/continuous.ts";
import { SCENES as determinismScenes } from "../src/demos/determinism.ts";
import { SCENES as eventsScenes } from "../src/demos/events.ts";
import { SCENES as geometryScenes } from "../src/demos/geometry.ts";
import { SCENES as issuesScenes } from "../src/demos/issues.ts";
import { SCENES as jointsScenes } from "../src/demos/joints.ts";
import { SCENES as robustnessScenes } from "../src/demos/robustness.ts";
import { SCENES as shapesScenes } from "../src/demos/shapes.ts";
import { SCENES as stackingScenes } from "../src/demos/stacking.ts";
import { SCENES as worldScenes } from "../src/demos/world.ts";

const PAGES: Record<string, { scenes: readonly string[]; extra?: readonly string[] }> = {
  benchmark: { scenes: benchmarkScenes },
  bodies: { scenes: bodiesScenes },
  character: { scenes: characterScenes },
  collision: { scenes: collisionScenes },
  continuous: { scenes: continuousScenes },
  determinism: { scenes: determinismScenes },
  events: { scenes: eventsScenes },
  geometry: { scenes: geometryScenes },
  issues: { scenes: issuesScenes },
  joints: { scenes: jointsScenes },
  robustness: { scenes: robustnessScenes },
  shapes: { scenes: shapesScenes },
  stacking: { scenes: stackingScenes },
  world: { scenes: worldScenes },
};

/**
 * Route-only exception policy (mirrors box3d Height Field / Replay Viewer):
 * a live/partial entry may omit `scene` only when it is a single-scene host —
 * one registry row owns the whole page, so there is no multi-scene table for
 * `PAGES` / `assertRouteScenes` to guard. Multi-sample categories and any route
 * that hosts more than one live/partial sample MUST declare `scene` keys and
 * appear in `PAGES`. Add a route here only with a one-line reason.
 */
const ROUTE_ONLY_EXCEPTIONS: ReadonlySet<string> = new Set([
  // Replay / Viewer — RegisterReplay single-scene page (no scene selector).
  "replay",
]);

/** Registry-declared scene keys for a route (entries that own a working scene). */
function registryScenes(route: string): string[] {
  return SAMPLES.filter((s) => s.route === route && s.scene != null).map((s) => s.scene!);
}

for (const [route, { scenes, extra = [] }] of Object.entries(PAGES)) {
  test(`registry <-> ${route} page scenes match exactly`, () => {
    const registry = new Set(registryScenes(route));
    const implemented = new Set(scenes);
    const allowed = new Set<string>([...registry, ...extra]);

    // Direction 1: every registry scene is implemented by the page.
    const missing = [...registry].filter((s) => !implemented.has(s)).sort();
    expect(missing).toEqual([]);

    // Direction 2: every page scene is backed by the registry (or a known extra).
    const unexpected = [...implemented].filter((s) => !allowed.has(s)).sort();
    expect(unexpected).toEqual([]);
  });
}

test("every multi-scene registry route is covered by this test", () => {
  const routesWithScenes = new Set(
    SAMPLES.filter((s) => s.scene != null && s.route).map((s) => s.route!),
  );
  const covered = new Set(Object.keys(PAGES));
  const uncovered = [...routesWithScenes].filter((r) => !covered.has(r)).sort();
  expect(uncovered).toEqual([]);
});

test("live/partial entries require route; multi-sample hosts require scene + PAGES", () => {
  // Prevent the loophole where a sample is live/partial with only `route` and no
  // `scene`, leaving PAGES empty so bidirectional SCENES checks never fire.
  const livePartial = SAMPLES.filter((s) => s.status === "live" || s.status === "partial");

  for (const s of livePartial) {
    expect(s.route).toBeTruthy();
  }

  // Count live/partial samples per category and per route.
  const byCategory = new Map<string, typeof livePartial>();
  const byRoute = new Map<string, typeof livePartial>();
  for (const s of livePartial) {
    const catList = byCategory.get(s.category) ?? [];
    catList.push(s);
    byCategory.set(s.category, catList);
    if (s.route) {
      const routeList = byRoute.get(s.route) ?? [];
      routeList.push(s);
      byRoute.set(s.route, routeList);
    }
  }

  for (const s of livePartial) {
    const multiCategory = (byCategory.get(s.category)?.length ?? 0) > 1;
    const multiRoute = (byRoute.get(s.route!)?.length ?? 0) > 1;
    const needsScene = multiCategory || multiRoute;

    if (needsScene) {
      expect(s.scene).toBeTruthy();
    } else if (!s.scene) {
      // Single-sample, single-scene host: must be an explicit exception.
      expect(ROUTE_ONLY_EXCEPTIONS.has(s.route!)).toBe(true);
    }
  }

  // Every live/partial route that declares scenes must be listed in PAGES.
  const routesNeedingPages = new Set(
    livePartial.filter((s) => s.scene != null && s.route).map((s) => s.route!),
  );
  const covered = new Set(Object.keys(PAGES));
  const missingPages = [...routesNeedingPages].filter((r) => !covered.has(r)).sort();
  expect(missingPages).toEqual([]);
});

test("pinned C submodule commit is recorded", () => {
  expect(CPP_REFERENCE_COMMIT).toMatch(/^[0-9a-f]{40}$/);
  expect(CPP_REFERENCE_COMMIT.startsWith("56edae7")).toBe(true);
});

test("inventory size matches the C pin (138 RegisterSample + 1 RegisterReplay)", () => {
  expect(SAMPLES.length).toBe(139);
  expect(categoryOrder().length).toBe(15);
  const stats = totalStats();
  expect(stats.total).toBe(139);
  // Bodies (9) + Stacking (10) + Joints (16+6) + Shapes (19) + Continuous (15) + Events (12) + Benchmark (1+20) + Robustness (7) + Collision (8+1) + Issues (6) + Determinism (2) + Replay (0+1) + Geometry (1) + Character (1) + World (3+1)
  expect(stats.live).toBe(110);
  expect(stats.partial).toBe(29);
  expect(stats.planned).toBe(0);
});

test("category totals match the C pin inventory", () => {
  const expected: Record<string, number> = {
    Benchmark: 21,
    Bodies: 9,
    Character: 1,
    Collision: 9,
    Continuous: 15,
    Determinism: 2,
    Events: 12,
    Geometry: 1,
    Issues: 6,
    Joints: 22,
    Replay: 1,
    Robustness: 7,
    Shapes: 19,
    Stacking: 10,
    World: 4,
  };
  const expectedOrder = Object.keys(expected).sort((a, b) => (a < b ? -1 : a > b ? 1 : 0));
  expect(categoryOrder()).toEqual(expectedOrder);
  for (const [cat, total] of Object.entries(expected)) {
    expect(categoryStats(cat).total).toBe(total);
    if (cat === "Bodies") {
      expect(categoryStats(cat).live).toBe(9);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Stacking") {
      expect(categoryStats(cat).live).toBe(10);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Joints") {
      expect(categoryStats(cat).live).toBe(16);
      expect(categoryStats(cat).partial).toBe(6);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Shapes") {
      expect(categoryStats(cat).live).toBe(19);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Continuous") {
      expect(categoryStats(cat).live).toBe(15);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Events") {
      expect(categoryStats(cat).live).toBe(12);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Benchmark") {
      expect(categoryStats(cat).live).toBe(1);
      expect(categoryStats(cat).partial).toBe(20);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Robustness") {
      expect(categoryStats(cat).live).toBe(7);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Collision") {
      expect(categoryStats(cat).live).toBe(8);
      expect(categoryStats(cat).partial).toBe(1);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Issues") {
      expect(categoryStats(cat).live).toBe(6);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Character") {
      expect(categoryStats(cat).live).toBe(1);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Determinism") {
      expect(categoryStats(cat).live).toBe(2);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Replay") {
      expect(categoryStats(cat).live).toBe(0);
      expect(categoryStats(cat).partial).toBe(1);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "Geometry") {
      expect(categoryStats(cat).live).toBe(1);
      expect(categoryStats(cat).partial).toBe(0);
      expect(categoryStats(cat).planned).toBe(0);
    } else if (cat === "World") {
      expect(categoryStats(cat).live).toBe(3);
      expect(categoryStats(cat).partial).toBe(1);
      expect(categoryStats(cat).planned).toBe(0);
    } else {
      expect(categoryStats(cat).planned).toBe(total);
    }
  }
});

test("every (category, name) pair is unique", () => {
  const keys = SAMPLES.map((s) => `${s.category}\0${s.name}`);
  expect(new Set(keys).size).toBe(keys.length);
});

test("slugs are unique within each category", () => {
  for (const cat of categoryOrder()) {
    const slugs = SAMPLES.filter((s) => s.category === cat).map((s) => s.slug);
    expect(new Set(slugs).size).toBe(slugs.length);
  }
});

test("slugify matches each entry.slug", () => {
  for (const s of SAMPLES) {
    expect(s.slug).toBe(slugify(s.name));
  }
});

test("SAMPLES_SORTED is Category then Name via strcmp", () => {
  for (let i = 1; i < SAMPLES_SORTED.length; i++) {
    expect(compareSamples(SAMPLES_SORTED[i - 1]!, SAMPLES_SORTED[i]!)).toBeLessThanOrEqual(0);
  }
});

test("live/partial entries must declare a route; planned must not claim scenes", () => {
  for (const s of SAMPLES) {
    if (s.status === "live" || s.status === "partial") {
      expect(s.route).toBeTruthy();
    }
    if (s.status === "planned") {
      // Phase 0/1 contract: planned rows have no hosting route/scene yet.
      expect(s.route).toBeUndefined();
      expect(s.scene).toBeUndefined();
    }
  }
});

test("cSourceUrl points at the pinned erincatto/box2d samples file", () => {
  const entry = SAMPLES[0]!;
  expect(cSourceUrl(entry)).toBe(
    `https://github.com/erincatto/box2d/blob/${CPP_REFERENCE_COMMIT}/samples/${entry.cSource}`,
  );
});
