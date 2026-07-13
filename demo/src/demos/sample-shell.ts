// Shared sample harness — pause / single-step / restart, hertz / sub-steps,
// and a C-faithful 2D camera (center + zoom half-height, matching samples/draw.c).
// Category ports opt into this instead of inventing per-page transport controls.

import {
  createButton,
  createCheckbox,
  createCollapsingSection,
  createSeparator,
  createSlider,
} from "../controls.ts";
import {
  cSourceUrl,
  entryHref,
  findByRouteName,
  firstEntryForRoute,
  neighborOf,
  type SampleEntry,
} from "../registry.ts";
import { defaultViewFlags, maskFromFlags, PANEL_FLAG_DEFS } from "../view-flags.ts";
import { getWasm } from "../wasm.ts";

/** C Camera defaults from GetDefaultCamera / ResetView (draw.c). */
export const DEFAULT_CAMERA = {
  centerX: 0,
  centerY: 20,
  zoom: 1,
} as const;

/** Arrow-key pan step in world units (main.cpp KeyCallback). */
export const CAMERA_PAN_STEP = 0.5;
/** Scroll zoom factor (main.cpp ScrollCallback). */
export const CAMERA_SCROLL_ZOOM = 1.1;
/** Held Z/X zoom rates and clamps (main.cpp main loop). */
export const CAMERA_ZOOM_OUT_RATE = 1.005;
export const CAMERA_ZOOM_IN_RATE = 0.995;
export const CAMERA_ZOOM_MAX = 100.0;
export const CAMERA_ZOOM_MIN = 0.5;

/** Solver defaults matching the C samples app (60 Hz, 4 sub-steps). */
export const DEFAULT_HERTZ = 60;
export const DEFAULT_SUB_STEPS = 4;

export interface SampleCamera {
  centerX: number;
  centerY: number;
  /** Half-height of the view in world units (C `camera.zoom`). */
  zoom: number;
}

export function makeCamera(
  centerX = DEFAULT_CAMERA.centerX,
  centerY = DEFAULT_CAMERA.centerY,
  zoom = DEFAULT_CAMERA.zoom,
): SampleCamera {
  return { centerX, centerY, zoom };
}

/** C ResetView (draw.c) — HOME / View → Reset Camera. */
export function resetCameraView(camera: SampleCamera): void {
  camera.centerX = DEFAULT_CAMERA.centerX;
  camera.centerY = DEFAULT_CAMERA.centerY;
  camera.zoom = DEFAULT_CAMERA.zoom;
}

/** Pixel → world, matching ConvertScreenToWorld (draw.c). */
export function screenToWorld(
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  px: number,
  py: number,
): { x: number; y: number } {
  const w = canvas.width;
  const h = canvas.height;
  const u = px / w;
  const v = (h - py) / h;
  const ratio = w / Math.max(1, h);
  const ex = camera.zoom * ratio;
  const ey = camera.zoom;
  return {
    x: camera.centerX + ex * (2 * u - 1),
    y: camera.centerY + ey * (2 * v - 1),
  };
}

/**
 * Cursor-anchored zoom matching ScrollCallback (main.cpp): change zoom then
 * pan so the world point under the cursor stays fixed. Scroll itself does not
 * clamp; held Z/X do (CAMERA_ZOOM_MIN/MAX).
 */
export function zoomCameraAtScreen(
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  px: number,
  py: number,
  zoomFactor: number,
): void {
  const before = screenToWorld(camera, canvas, px, py);
  camera.zoom *= zoomFactor;
  const after = screenToWorld(camera, canvas, px, py);
  camera.centerX -= after.x - before.x;
  camera.centerY -= after.y - before.y;
}

/**
 * Interactive camera matching C samples main.cpp: right-drag pan, scroll zoom,
 * arrow-key pan, HOME reset, held Z (zoom out) / X (zoom in).
 */
export function bindCameraControls(
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
): () => void {
  let rightDown = false;
  let clickWorld = { x: 0, y: 0 };
  let zoomOutHeld = false;
  let zoomInHeld = false;
  let rafId = 0;

  const canvasPoint = (e: PointerEvent | WheelEvent) => {
    const rect = canvas.getBoundingClientRect();
    return {
      px: ((e.clientX - rect.left) / Math.max(1, rect.width)) * canvas.width,
      py: ((e.clientY - rect.top) / Math.max(1, rect.height)) * canvas.height,
    };
  };

  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 2) return;
    const { px, py } = canvasPoint(e);
    clickWorld = screenToWorld(camera, canvas, px, py);
    rightDown = true;
    canvas.setPointerCapture(e.pointerId);
    e.preventDefault();
  };

  const onPointerMove = (e: PointerEvent) => {
    if (!rightDown) return;
    const { px, py } = canvasPoint(e);
    const pw = screenToWorld(camera, canvas, px, py);
    camera.centerX -= pw.x - clickWorld.x;
    camera.centerY -= pw.y - clickWorld.y;
    clickWorld = screenToWorld(camera, canvas, px, py);
  };

  const onPointerUp = (e: PointerEvent) => {
    if (e.button === 2) rightDown = false;
  };

  const onContextMenu = (e: Event) => e.preventDefault();

  const onWheel = (e: WheelEvent) => {
    e.preventDefault();
    const { px, py } = canvasPoint(e);
    const factor = e.deltaY < 0 ? 1 / CAMERA_SCROLL_ZOOM : CAMERA_SCROLL_ZOOM;
    zoomCameraAtScreen(camera, canvas, px, py, factor);
  };

  const onKeyDown = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement | null)?.tagName;
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
    switch (e.key) {
      case "ArrowLeft":
        camera.centerX -= CAMERA_PAN_STEP;
        e.preventDefault();
        break;
      case "ArrowRight":
        camera.centerX += CAMERA_PAN_STEP;
        e.preventDefault();
        break;
      case "ArrowDown":
        camera.centerY -= CAMERA_PAN_STEP;
        e.preventDefault();
        break;
      case "ArrowUp":
        camera.centerY += CAMERA_PAN_STEP;
        e.preventDefault();
        break;
      case "Home":
        resetCameraView(camera);
        e.preventDefault();
        break;
      case "z":
      case "Z":
        zoomOutHeld = true;
        e.preventDefault();
        break;
      case "x":
      case "X":
        zoomInHeld = true;
        e.preventDefault();
        break;
      default:
        break;
    }
  };

  const onKeyUp = (e: KeyboardEvent) => {
    if (e.key === "z" || e.key === "Z") zoomOutHeld = false;
    else if (e.key === "x" || e.key === "X") zoomInHeld = false;
  };

  const tickHeldZoom = () => {
    if (zoomOutHeld) {
      camera.zoom = Math.min(CAMERA_ZOOM_OUT_RATE * camera.zoom, CAMERA_ZOOM_MAX);
    } else if (zoomInHeld) {
      camera.zoom = Math.max(CAMERA_ZOOM_IN_RATE * camera.zoom, CAMERA_ZOOM_MIN);
    }
    rafId = requestAnimationFrame(tickHeldZoom);
  };
  rafId = requestAnimationFrame(tickHeldZoom);

  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerUp);
  canvas.addEventListener("contextmenu", onContextMenu);
  canvas.addEventListener("wheel", onWheel, { passive: false });
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);

  return () => {
    cancelAnimationFrame(rafId);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    canvas.removeEventListener("contextmenu", onContextMenu);
    canvas.removeEventListener("wheel", onWheel);
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
  };
}

/** World → pixel, matching ConvertWorldToScreen / ConvertViewToScreen. */
export function worldToScreen(
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  x: number,
  y: number,
): { x: number; y: number } {
  const w = canvas.width;
  const h = canvas.height;
  const ratio = w / Math.max(1, h);
  const ex = camera.zoom * ratio;
  const ey = camera.zoom;
  const u = (x - camera.centerX + ex) / (2 * ex);
  const v = (y - camera.centerY + ey) / (2 * ey);
  return { x: u * w, y: (1 - v) * h };
}

/** Pixels per world unit at the current zoom (vertical). */
export function pixelsPerMeter(camera: SampleCamera, canvas: HTMLCanvasElement): number {
  return canvas.height / (2 * Math.max(1e-6, camera.zoom));
}

/** View AABB in world space (C GetViewBounds). */
export function viewBounds(
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
): { lowerX: number; lowerY: number; upperX: number; upperY: number } {
  const bl = screenToWorld(camera, canvas, 0, canvas.height);
  const tr = screenToWorld(camera, canvas, canvas.width, 0);
  return { lowerX: bl.x, lowerY: bl.y, upperX: tr.x, upperY: tr.y };
}

export interface SampleTransport {
  paused: boolean;
  singleStep: boolean;
  hertz: number;
  subSteps: number;
  /** Effective dt for this frame (0 when paused without a single-step). */
  consumeStepDt(): number;
  /** Attach Space/P/O/R keyboard shortcuts; returns a disposer. */
  bindKeys(target?: Window): () => void;
  /** Append Pause / Step / Restart + Hertz / Sub-steps controls. */
  mountControls(parent: HTMLElement, onRestart: () => void): void;
}

/**
 * Shared pause / single-step / restart transport matching Sample::Step
 * (sample.cpp): when paused, dt is 0 unless singleStep was armed.
 */
export function createSampleTransport(opts?: {
  hertz?: number;
  subSteps?: number;
}): SampleTransport {
  const state = {
    paused: false,
    singleStep: false,
    hertz: opts?.hertz ?? DEFAULT_HERTZ,
    subSteps: opts?.subSteps ?? DEFAULT_SUB_STEPS,
  };

  const transport: SampleTransport = {
    get paused() {
      return state.paused;
    },
    set paused(v: boolean) {
      state.paused = v;
    },
    get singleStep() {
      return state.singleStep;
    },
    set singleStep(v: boolean) {
      state.singleStep = v;
    },
    get hertz() {
      return state.hertz;
    },
    set hertz(v: number) {
      state.hertz = v;
    },
    get subSteps() {
      return state.subSteps;
    },
    set subSteps(v: number) {
      state.subSteps = v;
    },
    consumeStepDt() {
      let dt = state.hertz > 0 ? 1 / state.hertz : 0;
      if (state.paused) {
        if (state.singleStep) {
          state.singleStep = false;
        } else {
          dt = 0;
        }
      }
      return dt;
    },
    bindKeys(target: Window = window) {
      const onKey = (e: KeyboardEvent) => {
        const tag = (e.target as HTMLElement | null)?.tagName;
        if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
        const k = e.key.toLowerCase();
        // C main.cpp uses SPACE for pause; P is a web-friendly alias.
        if (k === " " || k === "space" || k === "p") {
          state.paused = !state.paused;
          e.preventDefault();
        } else if (k === "o") {
          state.paused = true;
          state.singleStep = true;
          e.preventDefault();
        } else if (k === "r") {
          // Restart is wired by the page via a custom event so mountControls
          // and bindKeys stay decoupled from the scene builder.
          target.dispatchEvent(new CustomEvent("sample-restart"));
          e.preventDefault();
        }
      };
      target.addEventListener("keydown", onKey);
      return () => target.removeEventListener("keydown", onKey);
    },
    mountControls(parent: HTMLElement, onRestart: () => void) {
      const pauseBtn = createButton("Pause (Space)", () => {
        state.paused = !state.paused;
        pauseBtn.classList.toggle("active", state.paused);
        pauseBtn.textContent = state.paused ? "Resume (Space)" : "Pause (Space)";
      });
      const stepBtn = createButton("Step (O)", () => {
        state.paused = true;
        state.singleStep = true;
        pauseBtn.classList.add("active");
        pauseBtn.textContent = "Resume (Space)";
      });
      const restartBtn = createButton("Restart (R)", onRestart);
      parent.appendChild(pauseBtn);
      parent.appendChild(stepBtn);
      parent.appendChild(restartBtn);
      parent.appendChild(createSeparator());
      parent.appendChild(
        createSlider("Hertz", 5, 240, state.hertz, 1, (v) => {
          state.hertz = v;
        }),
      );
      parent.appendChild(
        createSlider("Sub-steps", 1, 8, state.subSteps, 1, (v) => {
          state.subSteps = v;
        }),
      );
      parent.appendChild(
        createCheckbox("Paused", state.paused, (v) => {
          state.paused = v;
          pauseBtn.classList.toggle("active", v);
          pauseBtn.textContent = v ? "Resume (Space)" : "Pause (Space)";
        }),
      );

      const onRestartEvent = () => onRestart();
      window.addEventListener("sample-restart", onRestartEvent);
      // Caller is responsible for removing this listener on cleanup via the
      // transport's bindKeys disposer + their own restart wiring; stash it.
      (transport as SampleTransport & { _restartListener?: () => void })._restartListener =
        () => window.removeEventListener("sample-restart", onRestartEvent);
    },
  };

  return transport;
}

/** Dispose the restart CustomEvent listener attached by mountControls. */
export function disposeTransport(transport: SampleTransport) {
  const extra = transport as SampleTransport & { _restartListener?: () => void };
  extra._restartListener?.();
  extra._restartListener = undefined;
}

function escapeHtml(s: string): string {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

export interface SampleWorldToggles {
  set_sleeping?(flag: boolean): void;
  set_warm_starting?(flag: boolean): void;
  set_continuous?(flag: boolean): void;
}

export interface SampleChrome {
  afterHead: HTMLElement;
  setSampleName(name: string): void;
  tick(opts: {
    frameMs: number;
    stepCount: number;
    paused: boolean;
    camera: SampleCamera;
    bodyCount?: number;
    awakeCount?: number;
    contactCount?: number;
  }): void;
  dispose(): void;
}

/**
 * C-faithful Info-panel chrome: title ◀/▶, category, C source, pause badge,
 * stats, Solver, Debug-draw flag toggles, keyboard legend. Adopts preexisting
 * control children into `afterHead`.
 *
 * When `canvas` + `camera` are provided, also binds interactive camera controls
 * (main.cpp) plus TAB hide-UI and M diagnostics.
 */
export function mountSampleChrome(opts: {
  controls: HTMLElement;
  route: string;
  category: string;
  sampleName: string;
  transport: SampleTransport;
  onRestart: () => void;
  getWorld?: () => SampleWorldToggles | null | undefined;
  canvas?: HTMLCanvasElement;
  camera?: SampleCamera;
}): SampleChrome {
  const { controls, route, category, transport, onRestart } = opts;
  const preexisting = Array.from(controls.childNodes);
  controls.replaceChildren();
  controls.classList.add("samples-info-panel");

  const viewFlags = defaultViewFlags();
  const pushFlags = () => {
    try {
      getWasm().sim_set_debug_flags?.(maskFromFlags(viewFlags));
    } catch {
      /* ignore */
    }
  };
  pushFlags();

  const infoHead = document.createElement("div");
  infoHead.className = "samples-info-head";
  infoHead.innerHTML = `
    <div class="sample-title-row">
      <button class="sample-nav-btn sample-prev" type="button" title="Previous sample ([)" aria-label="Previous sample">◀</button>
      <div class="sample-name">${escapeHtml(opts.sampleName)}</div>
      <button class="sample-nav-btn sample-next" type="button" title="Next sample (])" aria-label="Next sample">▶</button>
    </div>
    <div class="sample-category">${escapeHtml(category)}</div>
    <a class="sample-csource" target="_blank" rel="noopener" hidden>C source ↗</a>
    <div class="sample-paused" hidden>PAUSED <span class="sample-paused-hint">(SPACE)</span></div>
    <div class="sample-sep"></div>
    <div class="sample-stats">
      <div class="sample-stat frame-ms">0.0 ms</div>
      <div class="sample-stat step-count">step 0</div>
      <div class="sample-stat body-count"></div>
      <div class="sample-stat awake-count"></div>
      <div class="sample-stat contact-count"></div>
    </div>
    <div class="sample-sep"></div>
    <div class="sample-camera">
      <div class="sample-stat cam-center">center (0.0, 0.0)</div>
      <div class="sample-stat cam-zoom">zoom 0.0</div>
    </div>
    <div class="sample-sep"></div>
  `;
  controls.appendChild(infoHead);

  const afterHead = document.createElement("div");
  afterHead.className = "samples-after-head";
  for (const n of preexisting) afterHead.appendChild(n);
  controls.appendChild(afterHead);

  const nameEl = infoHead.querySelector(".sample-name") as HTMLElement;
  const pauseBadge = infoHead.querySelector(".sample-paused") as HTMLElement;
  const frameMsEl = infoHead.querySelector(".frame-ms") as HTMLElement;
  const stepCountEl = infoHead.querySelector(".step-count") as HTMLElement;
  const bodyCountEl = infoHead.querySelector(".body-count") as HTMLElement;
  const awakeCountEl = infoHead.querySelector(".awake-count") as HTMLElement;
  const contactCountEl = infoHead.querySelector(".contact-count") as HTMLElement;
  const camCenterEl = infoHead.querySelector(".cam-center") as HTMLElement;
  const camZoomEl = infoHead.querySelector(".cam-zoom") as HTMLElement;
  const prevBtn = infoHead.querySelector(".sample-prev") as HTMLButtonElement;
  const nextBtn = infoHead.querySelector(".sample-next") as HTMLButtonElement;
  const cSourceLink = infoHead.querySelector(".sample-csource") as HTMLAnchorElement;

  let activeEntry: SampleEntry | undefined =
    findByRouteName(route, opts.sampleName) ?? firstEntryForRoute(route);

  const refreshSampleMeta = () => {
    if (activeEntry) {
      cSourceLink.href = cSourceUrl(activeEntry);
      cSourceLink.textContent = `C source: ${activeEntry.cSource} ↗`;
      cSourceLink.hidden = false;
    } else cSourceLink.hidden = true;
    const prev = neighborOf(activeEntry, -1);
    const next = neighborOf(activeEntry, 1);
    prevBtn.disabled = prev == null || prev === activeEntry;
    nextBtn.disabled = next == null || next === activeEntry;
  };
  refreshSampleMeta();

  const gotoNeighbor = (dir: -1 | 1) => {
    const entry = neighborOf(activeEntry, dir);
    if (!entry || entry === activeEntry) return;
    window.location.hash = entryHref(entry);
  };
  prevBtn.addEventListener("click", () => gotoNeighbor(-1));
  nextBtn.addEventListener("click", () => gotoNeighbor(1));
  const onNavKey = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement | null)?.tagName;
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
    if (e.key === "[") {
      gotoNeighbor(-1);
      e.preventDefault();
    } else if (e.key === "]") {
      gotoNeighbor(1);
      e.preventDefault();
    }
  };
  window.addEventListener("keydown", onNavKey);

  const transportRow = document.createElement("div");
  transportRow.className = "control-row samples-transport";
  const pauseBtn = createButton("Pause (Space)", () => {
    transport.paused = !transport.paused;
    pauseBtn.classList.toggle("active", transport.paused);
    pauseBtn.textContent = transport.paused ? "Resume (Space)" : "Pause (Space)";
  });
  const stepBtn = createButton("Step (O)", () => {
    transport.paused = true;
    transport.singleStep = true;
    pauseBtn.classList.add("active");
    pauseBtn.textContent = "Resume (Space)";
  });
  transportRow.append(pauseBtn, stepBtn);
  controls.appendChild(transportRow);

  const solver = createCollapsingSection("Solver", true);
  controls.appendChild(solver.root);
  solver.body.appendChild(
    createSlider("Hertz", 5, 240, transport.hertz, 1, (v) => {
      transport.hertz = v;
    }),
  );
  solver.body.appendChild(
    createSlider("Sub-steps", 1, 8, transport.subSteps, 1, (v) => {
      transport.subSteps = v;
    }),
  );
  solver.body.appendChild(
    createCheckbox("Sleep", true, (v) => opts.getWorld?.()?.set_sleeping?.(v)),
  );
  solver.body.appendChild(
    createCheckbox("Warm Starting", true, (v) => opts.getWorld?.()?.set_warm_starting?.(v)),
  );
  solver.body.appendChild(
    createCheckbox("Continuous", true, (v) => opts.getWorld?.()?.set_continuous?.(v)),
  );
  const restartBtn = createButton("Restart (R)", onRestart);
  restartBtn.classList.add("control-btn-block");
  solver.body.appendChild(restartBtn);

  const onRestartEvent = () => onRestart();
  window.addEventListener("sample-restart", onRestartEvent);
  (transport as SampleTransport & { _restartListener?: () => void })._restartListener = () =>
    window.removeEventListener("sample-restart", onRestartEvent);

  const dbg = createCollapsingSection("Debug draw", false);
  controls.appendChild(dbg.root);
  for (const f of PANEL_FLAG_DEFS) {
    dbg.body.appendChild(
      createCheckbox(f.label, viewFlags[f.viewKey] ?? false, (on) => {
        viewFlags[f.viewKey] = on;
        pushFlags();
      }),
    );
  }
  let jointScale = 1;
  let forceScale = 1;
  const pushScales = () => {
    try {
      getWasm().sim_set_draw_scales?.(jointScale, forceScale);
    } catch {
      /* ignore */
    }
  };
  dbg.body.appendChild(
    createSlider("Joint scale", 0.1, 5, 1, 0.1, (v) => {
      jointScale = v;
      pushScales();
    }),
  );
  dbg.body.appendChild(
    createSlider("Force scale", 0.1, 5, 1, 0.1, (v) => {
      forceScale = v;
      pushScales();
    }),
  );

  const keys = createCollapsingSection("Keyboard", false);
  controls.appendChild(keys.root);
  keys.body.innerHTML = `
    <table class="key-legend">
      <tr><td>Space / P</td><td>Pause / resume</td></tr>
      <tr><td>O</td><td>Single step</td></tr>
      <tr><td>R</td><td>Restart sample</td></tr>
      <tr><td>Tab</td><td>Hide / show UI</td></tr>
      <tr><td>M</td><td>Diagnostics</td></tr>
      <tr><td>Home</td><td>Reset camera</td></tr>
      <tr><td>Arrows</td><td>Pan camera</td></tr>
      <tr><td>Z / X</td><td>Zoom out / in</td></tr>
      <tr><td>RMB drag</td><td>Pan camera</td></tr>
      <tr><td>Scroll</td><td>Zoom at cursor</td></tr>
      <tr><td>[ / ]</td><td>Prev / next sample</td></tr>
      <tr><td>LMB drag</td><td>Grab body</td></tr>
    </table>
  `;

  // --- TAB hide UI + M diagnostics (main.cpp KeyCallback) -------------------
  const page = controls.closest(".demo-page") as HTMLElement | null;
  const canvasArea = page?.querySelector(".demo-canvas-area") as HTMLElement | null;
  const metrics = document.createElement("div");
  metrics.className = "sample-metrics";
  metrics.hidden = true;
  metrics.innerHTML = `
    <div class="sample-metrics-title">Diagnostics <span class="sample-metrics-hint">(M)</span></div>
    <div class="sample-metrics-grid">
      <div class="sample-stat m-frame">0.0 ms</div>
      <div class="sample-stat m-step">step 0</div>
      <div class="sample-stat m-bodies"></div>
      <div class="sample-stat m-awake"></div>
      <div class="sample-stat m-contacts"></div>
      <div class="sample-stat m-cam">center (0.0, 0.0) · zoom 0.00</div>
      <div class="sample-stat m-paused"></div>
    </div>
  `;
  (canvasArea ?? page ?? controls).appendChild(metrics);
  const mFrame = metrics.querySelector(".m-frame") as HTMLElement;
  const mStep = metrics.querySelector(".m-step") as HTMLElement;
  const mBodies = metrics.querySelector(".m-bodies") as HTMLElement;
  const mAwake = metrics.querySelector(".m-awake") as HTMLElement;
  const mContacts = metrics.querySelector(".m-contacts") as HTMLElement;
  const mCam = metrics.querySelector(".m-cam") as HTMLElement;
  const mPaused = metrics.querySelector(".m-paused") as HTMLElement;

  let showUI = true;
  let showMetrics = false;
  let lastTick = {
    frameMs: 0,
    stepCount: 0,
    bodyCount: undefined as number | undefined,
    awakeCount: undefined as number | undefined,
    contactCount: undefined as number | undefined,
  };

  const applyUiVisibility = () => {
    controls.style.display = showUI ? "" : "none";
    const hint = page?.querySelector(".canvas-hint") as HTMLElement | null;
    if (hint) hint.style.display = showUI ? "" : "none";
    const sidebar = document.getElementById("sidebar");
    if (sidebar) sidebar.style.display = showUI ? "" : "none";
    const main = document.getElementById("main-content");
    if (main) main.style.marginLeft = showUI ? "" : "0";
    metrics.hidden = !showMetrics;
  };

  const onChromeKey = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement | null)?.tagName;
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
    if (e.key === "Tab") {
      showUI = !showUI;
      applyUiVisibility();
      e.preventDefault();
    } else if (e.key === "m" || e.key === "M") {
      showMetrics = !showMetrics;
      applyUiVisibility();
      e.preventDefault();
    }
  };
  window.addEventListener("keydown", onChromeKey);

  const unbindCamera =
    opts.canvas && opts.camera ? bindCameraControls(opts.camera, opts.canvas) : () => {};

  let rafId = 0;
  let lastRaf = performance.now();
  const refreshHud = () => {
    const now = performance.now();
    const dtMs = now - lastRaf;
    lastRaf = now;
    if (lastTick.frameMs <= 0) lastTick.frameMs = dtMs;
    const cam = opts.camera;
    if (cam) {
      camCenterEl.textContent = `center (${cam.centerX.toFixed(1)}, ${cam.centerY.toFixed(1)})`;
      camZoomEl.textContent = `zoom ${cam.zoom.toFixed(2)}`;
      mCam.textContent = `center (${cam.centerX.toFixed(1)}, ${cam.centerY.toFixed(1)}) · zoom ${cam.zoom.toFixed(2)}`;
    }
    pauseBadge.hidden = !transport.paused;
    pauseBtn.classList.toggle("active", transport.paused);
    pauseBtn.textContent = transport.paused ? "Resume (Space)" : "Pause (Space)";
    mPaused.textContent = transport.paused ? "PAUSED" : "";
    mPaused.hidden = !transport.paused;
    mFrame.textContent = `${lastTick.frameMs.toFixed(1)} ms`;
    mStep.textContent = `step ${lastTick.stepCount}`;
    mBodies.textContent = lastTick.bodyCount != null ? `bodies ${lastTick.bodyCount}` : "";
    mBodies.hidden = lastTick.bodyCount == null;
    mAwake.textContent = lastTick.awakeCount != null ? `awake ${lastTick.awakeCount}` : "";
    mAwake.hidden = lastTick.awakeCount == null;
    mContacts.textContent =
      lastTick.contactCount != null ? `contacts ${lastTick.contactCount}` : "";
    mContacts.hidden = lastTick.contactCount == null;
    rafId = requestAnimationFrame(refreshHud);
  };
  rafId = requestAnimationFrame(refreshHud);

  return {
    afterHead,
    setSampleName(name: string) {
      nameEl.textContent = name;
      activeEntry = findByRouteName(route, name) ?? firstEntryForRoute(route);
      refreshSampleMeta();
    },
    tick({ frameMs, stepCount, paused, camera, bodyCount, awakeCount, contactCount }) {
      lastTick = { frameMs, stepCount, bodyCount, awakeCount, contactCount };
      pauseBadge.hidden = !paused;
      pauseBtn.classList.toggle("active", paused);
      pauseBtn.textContent = paused ? "Resume (Space)" : "Pause (Space)";
      frameMsEl.textContent = `${frameMs.toFixed(1)} ms`;
      stepCountEl.textContent = `step ${stepCount}`;
      bodyCountEl.textContent = bodyCount != null ? `bodies ${bodyCount}` : "";
      bodyCountEl.hidden = bodyCount == null;
      awakeCountEl.textContent = awakeCount != null ? `awake ${awakeCount}` : "";
      awakeCountEl.hidden = awakeCount == null;
      contactCountEl.textContent = contactCount != null ? `contacts ${contactCount}` : "";
      contactCountEl.hidden = contactCount == null;
      camCenterEl.textContent = `center (${camera.centerX.toFixed(1)}, ${camera.centerY.toFixed(1)})`;
      camZoomEl.textContent = `zoom ${camera.zoom.toFixed(2)}`;
    },
    dispose() {
      cancelAnimationFrame(rafId);
      unbindCamera();
      window.removeEventListener("keydown", onNavKey);
      window.removeEventListener("keydown", onChromeKey);
      window.removeEventListener("sample-restart", onRestartEvent);
      metrics.remove();
      showUI = true;
      showMetrics = false;
      applyUiVisibility();
    },
  };
}
