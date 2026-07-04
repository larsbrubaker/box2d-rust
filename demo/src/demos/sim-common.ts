// Shared helpers for the simulation demo pages.

import type { SimWorld } from "../wasm.ts";

export type SimShape =
  | { kind: "box"; hx: number; hy: number; color: string }
  | { kind: "circle"; r: number; color: string };

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
    } else {
      ctx.arc(0, 0, shape.r * scale, 0, 2 * Math.PI);
      // radius line so rotation is visible
      ctx.moveTo(0, 0);
      ctx.lineTo(shape.r * scale, 0);
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

/// Standard demo page scaffolding (matches the clipper2-rust layout).
export function demoPage(
  container: HTMLElement,
  title: string,
  description: string,
  hint: string,
): { canvas: HTMLCanvasElement; controls: HTMLElement; readout: HTMLElement } {
  container.innerHTML = `
    <div class="demo-page">
      <div class="demo-header">
        <h2>${title}</h2>
        <p>${description}</p>
      </div>
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
