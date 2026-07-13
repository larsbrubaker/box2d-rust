// Shared helpers for the simulation demo pages.

import type { SimWorld } from "../wasm.ts";

/// Sync the canvas backing store to its CSS layout size.
export function fitCanvas(canvas: HTMLCanvasElement) {
  const w = Math.max(1, Math.round(canvas.clientWidth));
  const h = Math.max(1, Math.round(canvas.clientHeight));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
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
