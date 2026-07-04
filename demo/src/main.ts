// Main entry point - SPA router and WASM initialization (matches the
// clipper2-rust demo shell).

import { loadWasm } from "./wasm.ts";

// Demo page modules (lazy loaded)
type DemoInit = (container: HTMLElement) => (() => void) | void;
const demoModules: Record<string, () => Promise<{ init: DemoInit }>> = {
  bodies: () => import("./demos/bodies.ts"),
  stacking: () => import("./demos/stacking.ts"),
  joints: () => import("./demos/joints.ts"),
  events: () => import("./demos/events.ts"),
  continuous: () => import("./demos/continuous.ts"),
  character: () => import("./demos/character.ts"),
  geometry: () => import("./demos/geometry.ts"),
  manifolds: () => import("./demos/manifolds.ts"),
  math: () => import("./demos/math.ts"),
  roadmap: () => import("./demos/roadmap.ts"),
};

let currentCleanup: (() => void) | null = null;

// Mobile sidebar toggle
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
  if (sidebar.classList.contains("open")) {
    closeSidebar();
  } else {
    openSidebar();
  }
});

sidebarOverlay.addEventListener("click", closeSidebar);

// Close sidebar when a nav link is clicked (mobile UX)
document.querySelectorAll(".nav-link").forEach((link) => {
  link.addEventListener("click", closeSidebar);
});

function getRoute(): string {
  const hash = window.location.hash.slice(2) || "";
  return hash || "home";
}

function updateNav(route: string) {
  document.querySelectorAll(".nav-link").forEach((el) => {
    const r = (el as HTMLElement).dataset.route;
    el.classList.toggle("active", r === route);
  });
}

function renderHome(container: HTMLElement) {
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
          matching, including its cross-platform deterministic math. Explore live simulations
          and collision demos &mdash; all running in your browser via WebAssembly compiled from
          the Rust port.
        </p>
      </div>
      <div class="feature-grid">
        <a href="#/bodies" class="feature-card">
          <span class="card-icon">&#9660;</span>
          <h3>Falling Bodies</h3>
          <p>A full simulation driven by the ported b2World_Step: boxes and balls shower into a container. Click to drop more.</p>
        </a>
        <a href="#/stacking" class="feature-card">
          <span class="card-icon">&#9650;</span>
          <h3>Stacking</h3>
          <p>A box pyramid settles until its island falls asleep. Drop a heavy ball to wake everything back up.</p>
        </a>
        <a href="#/joints" class="feature-card">
          <span class="card-icon">&#9903;</span>
          <h3>Joints</h3>
          <p>Falling hinge chains with limits, springs, and motors &mdash; the exact scene the determinism test hashes bit-for-bit against C.</p>
        </a>
        <a href="#/events" class="feature-card">
          <span class="card-icon">&#9889;</span>
          <h3>Events</h3>
          <p>Contact begin/end, impact hit events, and sensor overlaps streamed from the engine's double-buffered event arrays.</p>
        </a>
        <a href="#/continuous" class="feature-card">
          <span class="card-icon">&#10148;</span>
          <h3>Continuous</h3>
          <p>Bullets vs a thin wall. Toggle continuous collision and watch the same shot tunnel straight through.</p>
        </a>
        <a href="#/geometry" class="feature-card">
          <span class="card-icon">&#10140;</span>
          <h3>Geometry Queries</h3>
          <p>Ray casts against polygons, circles, capsules, and segments, plus GJK closest points, tracking your cursor.</p>
        </a>
        <a href="#/manifolds" class="feature-card">
          <span class="card-icon">&#9649;</span>
          <h3>Contact Manifolds</h3>
          <p>Contact points and normals from the narrow phase as you drag a shape against a fixed box.</p>
        </a>
        <a href="#/math" class="feature-card">
          <span class="card-icon">&#9881;</span>
          <h3>Deterministic Math</h3>
          <p>Box2D's hand-rolled cosine/sine and atan2 &mdash; the foundation of cross-platform reproducibility.</p>
        </a>
        <a href="#/roadmap" class="feature-card">
          <span class="card-icon">&#9776;</span>
          <h3>Demo Roadmap</h3>
          <p>One demo per category of the upstream samples app, flipping live as each engine module is ported.</p>
        </a>
      </div>

      <div class="about-section">
        <h2>About This Project</h2>
        <p>
          This is a module-by-module Rust port of
          <a href="https://github.com/erincatto/box2d" target="_blank">Box2D v3</a> by Erin Catto,
          with the C test suite ported alongside each module. The full simulation pipeline is
          running: broad-phase pairs, narrow-phase manifolds, graph-colored soft-constraint
          solving with sub-stepping, restitution, joints, and island sleeping.
        </p>
        <p style="margin-top: 12px">
          Ported by <strong>Lars Brubaker</strong>, sponsored by
          <a href="https://www.matterhackers.com" target="_blank">MatterHackers</a>.
          Available on <a href="https://crates.io/crates/box2d-rust" target="_blank">crates.io</a>.
        </p>
        <div class="stats-row">
          <div class="stat">
            <div class="stat-value" id="stat-version">0.1.0</div>
            <div class="stat-label">On crates.io</div>
          </div>
          <div class="stat">
            <div class="stat-value">107</div>
            <div class="stat-label">Tests Passing</div>
          </div>
          <div class="stat">
            <div class="stat-value">f32 + f64</div>
            <div class="stat-label">Precision Modes</div>
          </div>
          <div class="stat">
            <div class="stat-value">60 Hz</div>
            <div class="stat-label">4 Sub-steps</div>
          </div>
        </div>
        <p style="margin-top: 16px; color: var(--text-secondary); font-size: 0.95rem;">
          <strong>Determinism is a feature</strong> &mdash; Box2D hand-rolls its trig functions for
          cross-platform reproducibility. This port keeps them bit-for-bit, never substituting
          the standard library.
        </p>
      </div>
    </div>
  `;

  // Live version badge from the wasm module.
  loadWasm()
    .then((wasm) => {
      const el = document.getElementById("stat-version");
      if (el) el.textContent = `v${wasm.version()}`;
    })
    .catch(() => {});
}

async function navigate(route: string) {
  const container = document.getElementById("main-content")!;

  // Cleanup previous demo
  if (currentCleanup) {
    currentCleanup();
    currentCleanup = null;
  }

  updateNav(route);

  if (route === "home") {
    renderHome(container);
    return;
  }

  const loader = demoModules[route];
  if (!loader) {
    container.innerHTML = `<div class="home-page"><h2>Page not found</h2><p>Unknown route: ${route}</p></div>`;
    return;
  }

  container.innerHTML = `<div class="home-page" style="display:flex;align-items:center;justify-content:center;height:80vh;"><p style="color:var(--text-muted)">Loading demo...</p></div>`;

  try {
    await loadWasm();
    const mod = await loader();
    container.innerHTML = "";
    const cleanup = mod.init(container);
    if (cleanup) currentCleanup = cleanup;
  } catch (e) {
    console.error("Failed to load demo:", e);
    container.innerHTML = `<div class="home-page"><h2>Error loading demo</h2><pre style="color:var(--clip-stroke)">${e}</pre></div>`;
  }
}

// Route on hash change
window.addEventListener("hashchange", () => navigate(getRoute()));

// Initial load
navigate(getRoute());
