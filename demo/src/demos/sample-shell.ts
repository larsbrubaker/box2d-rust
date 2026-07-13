// Shared sample harness — pause / single-step / restart, hertz / sub-steps,
// and a C-faithful 2D camera (center + zoom half-height, matching samples/draw.c).
// Category ports opt into this instead of inventing per-page transport controls.

import {
  createButton,
  createCheckbox,
  createSeparator,
  createSlider,
} from "../controls.ts";

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
