// Shared helpers for the simulation demo pages.

import type { SimWorld } from "../wasm.ts";

export type SimShape =
  | { kind: "box"; hx: number; hy: number; color: string }
  | { kind: "circle"; r: number; color: string }
  | { kind: "capsule"; hl: number; r: number; color: string }
  | { kind: "chain"; points: number[]; loop: boolean; color: string };

export const COLORS = {
  ground: "#5a6170",
  box: "#2563eb",
  ball: "#15803d",
  heavy: "#dc2626",
};

/// Sync the canvas backing store to its CSS layout size.
export function fitCanvas(canvas: HTMLCanvasElement) {
  const w = Math.max(1, Math.round(canvas.clientWidth));
  const h = Math.max(1, Math.round(canvas.clientHeight));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
}

/// Draw every tracked body. Positions come from the ported engine; shapes are
/// tracked JS-side as parallel descriptors.
export function drawSimBodies(
  canvas: HTMLCanvasElement,
  scale: number,
  originY: number,
  shapes: SimShape[],
  positions: Float32Array,
) {
  const ctx = canvas.getContext("2d")!;
  const toPx = (x: number, y: number): [number, number] => [
    canvas.width / 2 + x * scale,
    canvas.height - originY - y * scale,
  ];

  for (let i = 0; i < shapes.length; i++) {
    const shape = shapes[i];
    const x = positions[3 * i];
    const y = positions[3 * i + 1];
    const angle = positions[3 * i + 2];
    const [px, py] = toPx(x, y);

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-angle);
    ctx.lineWidth = 2;
    ctx.strokeStyle = shape.color;
    ctx.fillStyle = shape.color + "1a"; // 10% alpha
    ctx.beginPath();
    if (shape.kind === "box") {
      ctx.rect(-shape.hx * scale, -shape.hy * scale, 2 * shape.hx * scale, 2 * shape.hy * scale);
    } else if (shape.kind === "circle") {
      ctx.arc(0, 0, shape.r * scale, 0, 2 * Math.PI);
      // radius line so rotation is visible
      ctx.moveTo(0, 0);
      ctx.lineTo(shape.r * scale, 0);
    } else if (shape.kind === "capsule") {
      const hl = shape.hl * scale;
      const r = shape.r * scale;
      ctx.arc(-hl, 0, r, Math.PI / 2, -Math.PI / 2);
      ctx.lineTo(hl, -r);
      ctx.arc(hl, 0, r, -Math.PI / 2, Math.PI / 2);
      ctx.closePath();
    } else {
      // Chain points are world coordinates on a static body at the origin;
      // the canvas transform above is already at the body position (0, 0
      // offset), so undo the translate and draw in canvas space.
      ctx.restore();
      ctx.save();
      ctx.lineWidth = 2;
      ctx.strokeStyle = shape.color;
      ctx.beginPath();
      for (let p = 0; p + 1 < shape.points.length; p += 2) {
        const [cx, cy] = toPx(shape.points[p], shape.points[p + 1]);
        if (p === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      }
      if (shape.loop) ctx.closePath();
      ctx.stroke();
      ctx.restore();
      continue;
    }
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}

/// Run a simulation render loop with error reporting and a cleanup handle.
/// The first frame renders synchronously (hidden tabs suspend rAF).
export function runSimLoop(
  frame: () => void,
  readout: HTMLElement,
): () => void {
  let rafId = 0;
  let stopped = false;

  function tick() {
    if (stopped) return;
    try {
      frame();
    } catch (e) {
      readout.textContent = `Simulation error: ${e}`;
      console.error(e);
      return;
    }
    rafId = requestAnimationFrame(tick);
  }

  tick();

  return () => {
    stopped = true;
    cancelAnimationFrame(rafId);
  };
}

/// Standard demo page scaffolding. Sample pages pass `samplesShell: true`
/// for the dark C Samples App Info panel.
export type DemoPageOpts = {
  category?: string;
  samplesShell?: boolean;
  hideHeader?: boolean;
};

export function demoPage(
  container: HTMLElement,
  title: string,
  description: string,
  hint: string,
  opts: DemoPageOpts = {},
): { canvas: HTMLCanvasElement; controls: HTMLElement; readout: HTMLElement; page: HTMLElement } {
  const samplesShell = opts.samplesShell === true;
  const hideHeader = opts.hideHeader ?? samplesShell;
  const shellClass = samplesShell ? " samples-shell" : "";
  container.innerHTML = `
    <div class="demo-page${shellClass}">
      ${
        hideHeader
          ? ""
          : `<div class="demo-header">
        <h2>${title}</h2>
        <p>${description}</p>
      </div>`
      }
      <div class="demo-body">
        <div class="demo-canvas-area">
          <canvas id="demo-canvas"></canvas>
          <div class="canvas-hint">${hint}</div>
        </div>
        <div class="demo-controls" id="controls"></div>
      </div>
    </div>
  `;
  return {
    canvas: document.getElementById("demo-canvas") as HTMLCanvasElement,
    controls: document.getElementById("controls")!,
    readout: document.createElement("div"),
    page: container.querySelector(".demo-page") as HTMLElement,
  };
}

/// Free a SimWorld, ignoring double-free after navigation races.
export function freeSim(sim: SimWorld | null) {
  try {
    sim?.free();
  } catch {
    // already freed
  }
}
