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
  /** Attach P/O/R keyboard shortcuts; returns a disposer. */
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
        if (k === "p") {
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
      const pauseBtn = createButton("Pause (P)", () => {
        state.paused = !state.paused;
        pauseBtn.classList.toggle("active", state.paused);
        pauseBtn.textContent = state.paused ? "Resume (P)" : "Pause (P)";
      });
      const stepBtn = createButton("Step (O)", () => {
        state.paused = true;
        state.singleStep = true;
        pauseBtn.classList.add("active");
        pauseBtn.textContent = "Resume (P)";
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
          pauseBtn.textContent = v ? "Resume (P)" : "Pause (P)";
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
 */
export function mountSampleChrome(opts: {
  controls: HTMLElement;
  route: string;
  category: string;
  sampleName: string;
  transport: SampleTransport;
  onRestart: () => void;
  getWorld?: () => SampleWorldToggles | null | undefined;
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
    <div class="sample-paused" hidden>PAUSED <span class="sample-paused-hint">(P)</span></div>
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
  const pauseBtn = createButton("Pause (P)", () => {
    transport.paused = !transport.paused;
    pauseBtn.classList.toggle("active", transport.paused);
    pauseBtn.textContent = transport.paused ? "Resume (P)" : "Pause (P)";
  });
  const stepBtn = createButton("Step (O)", () => {
    transport.paused = true;
    transport.singleStep = true;
    pauseBtn.classList.add("active");
    pauseBtn.textContent = "Resume (P)";
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
      <tr><td>P</td><td>Pause / resume</td></tr>
      <tr><td>O</td><td>Single step</td></tr>
      <tr><td>R</td><td>Restart sample</td></tr>
      <tr><td>A</td><td>Flippers (Pinball)</td></tr>
      <tr><td>[ / ]</td><td>Prev / next sample</td></tr>
      <tr><td>Drag</td><td>Grab body</td></tr>
    </table>
  `;

  return {
    afterHead,
    setSampleName(name: string) {
      nameEl.textContent = name;
      activeEntry = findByRouteName(route, name) ?? firstEntryForRoute(route);
      refreshSampleMeta();
    },
    tick({ frameMs, stepCount, paused, camera, bodyCount, awakeCount, contactCount }) {
      pauseBadge.hidden = !paused;
      pauseBtn.classList.toggle("active", paused);
      pauseBtn.textContent = paused ? "Resume (P)" : "Pause (P)";
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
      window.removeEventListener("keydown", onNavKey);
      window.removeEventListener("sample-restart", onRestartEvent);
    },
  };
}
