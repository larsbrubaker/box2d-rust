// Main entry — SPA shell with registry-driven Samples tree (box3d layout),
// hash router with per-scene deep links (`#/<route>/<slug>`), and lab pages
// for invented composites. Math lives under About, not Samples.

import { loadWasm } from "./wasm.ts";
import {
  samplesByCategory,
  categoryStats,
  totalStats,
  findByRouteSlug,
  firstEntryForRoute,
  entryHref,
  type SampleEntry,
  type CategoryStats,
} from "./registry.ts";

type DemoInit = (
  container: HTMLElement,
  initialScene?: string,
) => (() => void) | void;

const demoModules: Record<string, () => Promise<{ init: DemoInit }>> = {
  bodies: () => import("./demos/bodies.ts"),
  stacking: () => import("./demos/stacking.ts"),
  joints: () => import("./demos/joints.ts"),
  events: () => import("./demos/events.ts"),
  continuous: () => import("./demos/continuous.ts"),
  character: () => import("./demos/character.ts"),
  shapes: () => import("./demos/shapes.ts"),
  world: () => import("./demos/world.ts"),
  determinism: () => import("./demos/determinism.ts"),
  robustness: () => import("./demos/robustness.ts"),
  benchmark: () => import("./demos/benchmark.ts"),
  replay: () => import("./demos/replay.ts"),
  geometry: () => import("./demos/geometry.ts"),
  manifolds: () => import("./demos/manifolds.ts"),
  math: () => import("./demos/math.ts"),
  roadmap: () => import("./demos/roadmap.ts"),
};

let currentCleanup: (() => void) | null = null;

const menuToggle = document.getElementById("menu-toggle")!;
const sidebar = document.getElementById("sidebar")!;
const sidebarOverlay = document.getElementById("sidebar-overlay")!;

function openSidebar() {
  sidebar.classList.add("open");
  menuToggle.classList.add("open");
  sidebarOverlay.classList.add("visible");
}

function closeSidebar() {
  sidebar.classList.remove("open");
  menuToggle.classList.remove("open");
  sidebarOverlay.classList.remove("visible");
}

menuToggle.addEventListener("click", () => {
  if (sidebar.classList.contains("open")) closeSidebar();
  else openSidebar();
});
sidebarOverlay.addEventListener("click", closeSidebar);

// ---------------------------------------------------------------------------
// Hash routing. `#/` → home. `#/<route>` → page default.
// `#/<route>/<slug>` → deep link (Phase 2 scene select).
// ---------------------------------------------------------------------------

interface ParsedRoute {
  route: string;
  slug?: string;
}

function parseHash(): ParsedRoute {
  const raw = window.location.hash.replace(/^#\/?/, "");
  if (raw === "") return { route: "home" };
  const parts = raw.split("/").filter(Boolean);
  return { route: parts[0] ?? "home", slug: parts[1] };
}

function currentEntry(parsed: ParsedRoute): SampleEntry | undefined {
  if (parsed.route === "home") return undefined;
  if (parsed.slug) return findByRouteSlug(parsed.route, parsed.slug);
  return firstEntryForRoute(parsed.route);
}

/** Canonical hash for the current location (`#/` when empty). */
function currentHash(): string {
  const h = window.location.hash;
  return h === "" || h === "#" ? "#/" : h;
}

function isHomeHash(hash: string = currentHash()): boolean {
  return hash === "#/" || hash === "#" || hash === "";
}

// ---------------------------------------------------------------------------
// Local settings — category expand/collapse + last-visited sample fallback.
// URL hash is the primary demo identity; localStorage fills gaps only.
// ---------------------------------------------------------------------------

const STORAGE_EXPANDED = "box2d-demo.expandedCategories";
const STORAGE_LAST_HASH = "box2d-demo.lastHash";

function loadExpandedCategories(): Set<string> {
  try {
    const raw = localStorage.getItem(STORAGE_EXPANDED);
    if (!raw) return new Set();
    const parsed = JSON.parse(raw) as unknown;
    if (!Array.isArray(parsed)) return new Set();
    return new Set(parsed.filter((x): x is string => typeof x === "string"));
  } catch {
    return new Set();
  }
}

function saveExpandedCategories(): void {
  try {
    localStorage.setItem(STORAGE_EXPANDED, JSON.stringify([...expanded]));
  } catch {
    // Quota / private mode — ignore.
  }
}

function saveLastHash(hash: string = currentHash()): void {
  if (isHomeHash(hash)) return;
  try {
    localStorage.setItem(STORAGE_LAST_HASH, hash);
  } catch {
    // ignore
  }
}

function loadLastHash(): string | null {
  try {
    const h = localStorage.getItem(STORAGE_LAST_HASH);
    if (!h || isHomeHash(h)) return null;
    return h;
  } catch {
    return null;
  }
}

/**
 * Empty hash (first visit / bare `/`) restores the last sample. Explicit `#/`
 * (Home link) stays on home so refresh there does not bounce back.
 */
function restoreLastHashIfBare(): void {
  if (window.location.hash !== "") return;
  const last = loadLastHash();
  if (!last) return;
  history.replaceState(null, "", last);
}

// ---------------------------------------------------------------------------
// Sidebar tree — collapsible categories in C sort order (box3d pattern).
// ---------------------------------------------------------------------------

const treeRoot = document.getElementById("sample-tree")!;
const expanded = loadExpandedCategories();

let g_byCat: Map<string, SampleEntry[]> | null = null;
function byCatMemo(): Map<string, SampleEntry[]> {
  return (g_byCat ??= samplesByCategory());
}
const g_catStats = new Map<string, CategoryStats>();
function catStatsMemo(category: string): CategoryStats {
  let stats = g_catStats.get(category);
  if (!stats) {
    stats = categoryStats(category);
    g_catStats.set(category, stats);
  }
  return stats;
}

function statusTag(status: SampleEntry["status"]): string {
  if (status === "live") return `<span class="s-tag s-live">LIVE</span>`;
  if (status === "partial") return `<span class="s-tag s-partial">PARTIAL</span>`;
  return `<span class="s-tag s-planned">PLANNED</span>`;
}

function escapeHtml(s: string): string {
  return s.replace(/[&<>"']/g, (c) =>
    c === "&" ? "&amp;" : c === "<" ? "&lt;" : c === ">" ? "&gt;" : c === '"' ? "&quot;" : "&#39;",
  );
}

const itemKey = (route: string | undefined, slug: string): string => `${route ?? ""}\0${slug}`;
const treeItemEls = new Map<string, HTMLElement>();
const treeCatEls = new Map<string, HTMLElement>();
let treeHomeEl: HTMLAnchorElement | null = null;
let activeItemEl: HTMLElement | null = null;
let treeBuilt = false;

function buildTree(): void {
  const byCat = byCatMemo();
  const frag = document.createDocumentFragment();

  const home = document.createElement("a");
  home.href = "#/";
  home.className = "tree-home";
  home.textContent = "Home";
  treeHomeEl = home;
  frag.appendChild(home);

  const samplesLabel = document.createElement("div");
  samplesLabel.className = "lab-section";
  samplesLabel.textContent = "Samples";
  frag.appendChild(samplesLabel);

  for (const [category, entries] of byCat) {
    const stats = catStatsMemo(category);
    const countClass = stats.live + stats.partial > 0 ? "has-live" : "";
    const catDiv = document.createElement("div");
    catDiv.className = "tree-cat";
    catDiv.dataset.cat = category;
    catDiv.innerHTML =
      `<button class="tree-cat-head" data-cat="${category}">` +
      `<span class="tree-chevron">▾</span>` +
      `<span class="tree-cat-name">${escapeHtml(category)}</span>` +
      `<span class="tree-cat-count ${countClass}">${stats.live + stats.partial}/${stats.total}</span>` +
      `</button>`;
    const body = document.createElement("div");
    body.className = "tree-cat-body";
    for (const e of entries) {
      const item = document.createElement(e.route ? "a" : "span");
      item.innerHTML = `<span class="tree-item-name">${escapeHtml(e.name)}</span>${statusTag(e.status)}`;
      if (e.route) {
        (item as HTMLAnchorElement).href = entryHref(e);
        item.className = `tree-item ${e.status}`;
        treeItemEls.set(itemKey(e.route, e.slug), item);
      } else {
        item.className = "tree-item planned";
        item.title = "Not ported yet";
      }
      body.appendChild(item);
    }
    catDiv.appendChild(body);
    treeCatEls.set(category, catDiv);
    frag.appendChild(catDiv);
  }

  treeRoot.innerHTML = "";
  treeRoot.appendChild(frag);
  treeBuilt = true;
}

function renderTree(active?: SampleEntry): void {
  if (!treeBuilt) buildTree();

  const atHome = location.hash === "" || location.hash === "#/";
  treeHomeEl?.classList.toggle("active", atHome);

  if (activeItemEl) activeItemEl.classList.remove("active");
  activeItemEl = null;
  if (active?.route) {
    const el = treeItemEls.get(itemKey(active.route, active.slug));
    if (el) {
      el.classList.add("active");
      activeItemEl = el;
    }
  }

  for (const [category, el] of treeCatEls) {
    el.classList.toggle("open", expanded.has(category));
  }
}

treeRoot.addEventListener("click", (e) => {
  const head = (e.target as HTMLElement).closest(".tree-cat-head") as HTMLElement | null;
  if (head) {
    const cat = head.dataset.cat!;
    if (expanded.has(cat)) expanded.delete(cat);
    else expanded.add(cat);
    head.parentElement!.classList.toggle("open", expanded.has(cat));
    saveExpandedCategories();
    return;
  }
  if ((e.target as HTMLElement).closest(".tree-item, .tree-home")) closeSidebar();
});

document.querySelectorAll(".lab-link").forEach((link) => {
  link.addEventListener("click", closeSidebar);
});

function updateLabNav(route: string) {
  document.querySelectorAll(".lab-link").forEach((el) => {
    const r = (el as HTMLElement).dataset.route;
    el.classList.toggle("active", r === route);
  });
}

// ---------------------------------------------------------------------------
// Home
// ---------------------------------------------------------------------------

function renderHome(container: HTMLElement) {
  const total = totalStats();
  container.innerHTML = `
    <div class="home-page">
      <div class="github-badge">
        <a href="https://github.com/larsbrubaker/box2d-rust" target="_blank" class="github-badge-link">
          <svg height="20" viewBox="0 0 16 16" width="20" fill="currentColor"><path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0016 8c0-4.42-3.58-8-8-8z"/></svg>
          <span>larsbrubaker/box2d-rust</span>
        </a>
      </div>
      <div class="hero">
        <h1>Box2D <span>for Rust</span></h1>
        <p>
          A pure Rust port of Erin Catto's Box2D v3 physics engine with exact behavioral
          matching. The Samples tree mirrors the C app inventory
          (${total.total} entries) — ports flip to LIVE as each category lands.
        </p>
      </div>
      <div class="stats-row" style="margin: 24px 0">
        <div class="stat">
          <div class="stat-value" id="stat-version">…</div>
          <div class="stat-label">Port version</div>
        </div>
        <div class="stat">
          <div class="stat-value">${total.live}</div>
          <div class="stat-label">Exact samples</div>
        </div>
        <div class="stat">
          <div class="stat-value">${total.planned}</div>
          <div class="stat-label">Still planned</div>
        </div>
        <div class="stat">
          <div class="stat-value">WASM</div>
          <div class="stat-label">In-browser</div>
        </div>
      </div>
      <div class="about-section">
        <h2>About This Project</h2>
        <p>
          Module-by-module Rust port of
          <a href="https://github.com/erincatto/box2d" target="_blank">Box2D v3</a> by Erin Catto.
          Lab pages under the sidebar exercise the engine today; Phase 2 replaces them with
          1:1 C sample ports driven by the registry.
        </p>
        <p style="margin-top: 12px">
          Ported by <strong>Lars Brubaker</strong>, sponsored by
          <a href="https://www.matterhackers.com" target="_blank">MatterHackers</a>.
        </p>
      </div>
    </div>
  `;

  loadWasm()
    .then((wasm) => {
      const el = document.getElementById("stat-version");
      if (el) el.textContent = `v${wasm.version()}`;
    })
    .catch(() => {});
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

/** Keep tree highlight + last-hash in sync when demos `replaceState` the hash. */
function syncNavFromHash(): void {
  const parsed = parseHash();
  const active = currentEntry(parsed);
  if (active && !expanded.has(active.category)) {
    expanded.add(active.category);
    saveExpandedCategories();
  }
  renderTree(active);
  updateLabNav(parsed.route);
  saveLastHash();
}

async function navigate(): Promise<void> {
  const parsed = parseHash();
  const container = document.getElementById("main-content")!;

  if (currentCleanup) {
    currentCleanup();
    currentCleanup = null;
  }

  const active = currentEntry(parsed);
  if (active && !expanded.has(active.category)) {
    expanded.add(active.category);
    saveExpandedCategories();
  }
  renderTree(active);
  updateLabNav(parsed.route);
  saveLastHash();

  // Prefer deep links with a slug when the registry has one.
  if (active?.route && active.slug && !parsed.slug) {
    const href = entryHref(active);
    if (currentHash() !== href) {
      history.replaceState(null, "", href);
    }
  }

  if (parsed.route === "home") {
    renderHome(container);
    return;
  }

  const loader = demoModules[parsed.route];
  if (!loader) {
    container.innerHTML = `<div class="home-page"><h2>Page not found</h2><p>Unknown route: ${escapeHtml(
      parsed.route,
    )}</p></div>`;
    return;
  }

  container.innerHTML = `<div class="home-page" style="display:flex;align-items:center;justify-content:center;height:80vh;"><p style="color:var(--text-muted)">Loading demo...</p></div>`;

  try {
    await loadWasm();
    const mod = await loader();
    container.innerHTML = "";
    const scene = active?.scene ?? parsed.slug;
    const cleanup = mod.init(container, scene);
    if (cleanup) currentCleanup = cleanup;
  } catch (e) {
    console.error("Failed to load demo:", e);
    container.innerHTML = `<div class="home-page"><h2>Error loading demo</h2><pre style="color:var(--clip-stroke)">${escapeHtml(
      String(e),
    )}</pre></div>`;
  }
}

window.addEventListener("hashchange", () => {
  void navigate();
});

// Scene dropdowns update the hash via replaceState (no hashchange). Mirror the
// active tree item + last-visited hash without remounting the demo.
const origReplaceState = history.replaceState.bind(history);
history.replaceState = ((data: unknown, unused: string, url?: string | URL | null) => {
  const before = currentHash();
  origReplaceState(data, unused, url);
  const after = currentHash();
  if (before !== after) syncNavFromHash();
}) as typeof history.replaceState;

restoreLastHashIfBare();
void navigate();
