// Build-time guard that the sample registry (the single source of truth for the
// category→scene tree) and each multi-scene page's implemented scene table agree
// exactly, in both directions. This is the CI-enforced version of the page-init
// `assertRouteScenes` self-check (which only `console.error`s at runtime): here a
// drift fails `bun test`, so a RegisterSample row added without its page scene —
// or a scene key renamed on only one side — cannot reach main.
//
// Phase 0: every registry entry is `planned` with no `route`/`scene`, and no demo
// page yet exports a C-faithful `SCENES` table. `PAGES` is therefore empty. As
// categories are ported:
//   1. Flip entries to live/partial and set route + scene in registry.ts
//   2. Export `SCENES` from the page and call `assertRouteScenes`
//   3. Add that route to `PAGES` below
// The bidirectional checks then tighten automatically. Until then, this file
// still enforces registry internal consistency and the C pin metadata.

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
 * Phase 0: empty — no page has a registry-backed scene yet. Do not add invented
 * composite demos here; only add a route when its SCENES map 1:1 to live/partial
 * registry rows.
 */
const PAGES: Record<string, { scenes: readonly string[]; extra?: readonly string[] }> = {
  // Example (Phase 1+):
  // stacking: { scenes: stackingScenes },
};

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

test("pinned C submodule commit is recorded", () => {
  expect(CPP_REFERENCE_COMMIT).toMatch(/^[0-9a-f]{40}$/);
  expect(CPP_REFERENCE_COMMIT.startsWith("56edae7")).toBe(true);
});

test("inventory size matches the C pin (138 RegisterSample + 1 RegisterReplay)", () => {
  expect(SAMPLES.length).toBe(139);
  expect(categoryOrder().length).toBe(15);
  const stats = totalStats();
  expect(stats.total).toBe(139);
  expect(stats.live).toBe(0);
  expect(stats.partial).toBe(0);
  expect(stats.planned).toBe(139);
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
    expect(categoryStats(cat).planned).toBe(total);
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
      // Phase 0 contract: planned rows have no hosting route/scene yet.
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
