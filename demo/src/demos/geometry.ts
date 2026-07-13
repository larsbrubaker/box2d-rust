// Geometry — Convex Hull from sample_geometry.cpp (the only RegisterSample in
// that file). Point generation, b2ComputeHull, and b2ValidateHull run in wasm;
// this page orbits the C camera and draws hull / input points.

import {
  createButton,
  createButtonGroup,
  createInfoBox,
  createReadout,
  createSeparator,
  updateReadout,
} from "../controls.ts";
import { assertRouteScenes } from "../registry.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, fitCanvas, runSimLoop } from "./sim-common.ts";
import {
  createSampleTransport,
  mountSampleChrome,
  disposeTransport,
  makeCamera,
  worldToScreen,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — must match slugify(C name) / registry.ts. */
export const SCENES = ["convex-hull"] as const;
export type Scene = (typeof SCENES)[number];

assertRouteScenes("geometry", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "convex-hull": "Convex Hull",
};

/** C camera (sample_geometry.cpp:25-26). */
const CAMERA = { cx: 0.5, cy: 0.0, zoom: 25.0 * 0.3 };

// C debug palette (types.h)
const C_GRAY = "#808080";
const C_BLUE = "#0000FF";
const C_GREEN = "#008000";
const C_WHITE = "#FFFFFF";

interface HullFrame {
  generation: number;
  pointCount: number;
  valid: boolean;
  hullCount: number;
  auto: boolean;
  bulk: boolean;
  points: Float32Array;
  hull: Float32Array;
}

function parseFrame(data: Float32Array): HullFrame {
  const generation = data[0]!;
  const pointCount = data[1]! | 0;
  const valid = data[2]! === 1.0;
  const hullCount = data[3]! | 0;
  const auto = data[4]! === 1.0;
  const bulk = data[5]! === 1.0;
  let off = 6;
  const points = data.subarray(off, off + pointCount * 2);
  off += pointCount * 2;
  const hull = data.subarray(off, off + hullCount * 2);
  return { generation, pointCount, valid, hullCount, auto, bulk, points, hull };
}

function applyCamera(camera: SampleCamera) {
  camera.centerX = CAMERA.cx;
  camera.centerY = CAMERA.cy;
  camera.zoom = CAMERA.zoom;
}

function drawPoint(
  ctx: CanvasRenderingContext2D,
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  x: number,
  y: number,
  radiusPx: number,
  color: string,
) {
  const p = worldToScreen(camera, canvas, x, y);
  ctx.beginPath();
  ctx.arc(p.x, p.y, radiusPx, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function drawHullPolygon(
  ctx: CanvasRenderingContext2D,
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  hull: Float32Array,
  count: number,
) {
  if (count < 2) return;
  ctx.beginPath();
  for (let i = 0; i < count; i++) {
    const p = worldToScreen(camera, canvas, hull[2 * i]!, hull[2 * i + 1]!);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  }
  ctx.closePath();
  ctx.strokeStyle = C_GRAY;
  ctx.lineWidth = 2;
  ctx.stroke();
}

export function init(container: HTMLElement, _initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Geometry",
    "C <code>sample_geometry.cpp</code> — Convex Hull (b2ComputeHull / b2ValidateHull).",
    "G generate · A auto · B bulk · P pause · O step · R restart",
    { category: "Geometry", samplesShell: true }
  );

  const camera: SampleCamera = makeCamera();
  applyCamera(camera);
  const transport = createSampleTransport();

  let frame = parseFrame(wasm.geometry_hull_reset());

  controls.appendChild(
    createInfoBox(
      "Blue dots are the random input vertices; green dots are the hull. " +
        "Gray outline is <code>b2ComputeHull</code>. Bulk mode stress-tests validation.",
    ),
  );
  controls.appendChild(createSeparator());

  // Single C sample — still expose the scene label for deep-link consistency.
  controls.appendChild(
    createButtonGroup(
      SCENES.map((s) => ({ label: SCENE_LABEL[s], value: s })),
      "convex-hull",
      () => {
        /* only one scene */
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "geometry",
    category: "Geometry",
    sampleName: "Convex Hull",
    transport,
    onRestart: () => {
      frame = parseFrame(wasm.geometry_hull_reset());
    },
  });
  controls.appendChild(createSeparator());

  const actions = document.createElement("div");
  actions.className = "scene-controls";
  actions.appendChild(
    createButton("Generate (G)", () => {
      wasm.geometry_hull_key(0x47);
      frame = parseFrame(wasm.geometry_hull_step(false));
    }),
  );
  actions.appendChild(
    createButton("Auto (A)", () => {
      wasm.geometry_hull_key(0x41);
      frame = parseFrame(wasm.geometry_hull_step(false));
    }),
  );
  actions.appendChild(
    createButton("Bulk (B)", () => {
      wasm.geometry_hull_key(0x42);
      frame = parseFrame(wasm.geometry_hull_step(true));
    }),
  );
  controls.appendChild(actions);
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const onKey = (e: KeyboardEvent) => {
    if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
    const k = e.key.toUpperCase();
    if (k === "G") {
      wasm.geometry_hull_key(0x47);
      frame = parseFrame(wasm.geometry_hull_step(false));
    } else if (k === "A") {
      wasm.geometry_hull_key(0x41);
      frame = parseFrame(wasm.geometry_hull_step(false));
    } else if (k === "B") {
      wasm.geometry_hull_key(0x42);
      // Bulk runs inside step (C Step when m_bulk).
      frame = parseFrame(wasm.geometry_hull_step(true));
    }
  };
  window.addEventListener("keydown", onKey);

  const unbindKeys = transport.bindKeys();
  const ctx = canvas.getContext("2d")!;

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    const advance = dt > 0;
    // Always step so HUD/flags stay current; auto only advances when not paused.
    frame = parseFrame(wasm.geometry_hull_step(advance));

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#1a1d23";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    drawHullPolygon(ctx, camera, canvas, frame.hull, frame.hullCount);

    for (let i = 0; i < frame.pointCount; i++) {
      const x = frame.points[2 * i]!;
      const y = frame.points[2 * i + 1]!;
      drawPoint(ctx, camera, canvas, x, y, 5, C_BLUE);
      const label = worldToScreen(camera, canvas, x + 0.1, y + 0.1);
      ctx.fillStyle = C_WHITE;
      ctx.font = "12px monospace";
      ctx.fillText(String(i), label.x, label.y);
    }
    for (let i = 0; i < frame.hullCount; i++) {
      drawPoint(ctx, camera, canvas, frame.hull[2 * i]!, frame.hull[2 * i + 1]!, 6, C_GREEN);
    }

    // C DrawScreenTextLine overlays
    ctx.fillStyle = C_WHITE;
    ctx.font = "13px monospace";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    let ty = 10;
    ctx.fillText("Options: generate(g), auto(a), bulk(b)", 10, ty);
    ty += 18;
    if (!frame.valid) {
      ctx.fillText(`generation = ${frame.generation | 0}, FAILED`, 10, ty);
    } else {
      ctx.fillText(
        `generation = ${frame.generation | 0}, count = ${frame.hullCount}`,
        10,
        ty,
      );
    }

    updateReadout(readout, [
      { label: "Sample", value: SCENE_LABEL["convex-hull"] },
      { label: "Generation", value: String(frame.generation | 0) },
      { label: "Hull count", value: frame.valid ? String(frame.hullCount) : "FAILED" },
      { label: "Auto", value: frame.auto ? "on" : "off" },
      { label: "Bulk", value: frame.bulk ? "on" : "off" },
    ]);
  }, readout);

  return () => {
    stop();
    unbindKeys();
    window.removeEventListener("keydown", onKey);
    chrome.dispose();
    disposeTransport(transport);
  };
}
