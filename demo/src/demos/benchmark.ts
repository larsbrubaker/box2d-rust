// Benchmark — a large pyramid stress scene with live step timing. Everything
// below the readout is the ported engine; the page only draws and times.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const ORIGIN_Y = 36;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Benchmark",
    "A large box pyramid solved every frame, with wall-clock step timing. Watch the cost drop " +
      "to near zero once the island falls asleep, then wake it with a ball.",
    "Click the canvas to drop a heavy ball",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let rows = 30;
  let scale = 16;
  let stepTimes: number[] = [];
  let maxStepMs = 0;

  function dropBall(x: number) {
    sim.add_circle(Math.max(-14, Math.min(14, x)), rows * 0.5 + 3.0, 0.8, 5.0);
    shapes.push({ kind: "circle", r: 0.8, color: COLORS.heavy });
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];
    stepTimes = [];
    maxStepMs = 0;

    const halfWidth = rows * 0.55 + 4.0;
    sim.add_static_box(0.0, -0.5, halfWidth, 0.5);
    shapes.push({ kind: "box", hx: halfWidth, hy: 0.5, color: COLORS.ground });

    // Big pyramid, upstream benchmark style: rows of touching boxes.
    const h = 0.25;
    for (let row = 0; row < rows; row++) {
      const count = rows - row;
      const y = h + row * 2 * h;
      for (let i = 0; i < count; i++) {
        const x = (i - (count - 1) / 2) * 2.05 * h;
        sim.add_box(x, y, h, h, 1.0);
        shapes.push({ kind: "box", hx: h, hy: h, color: COLORS.box });
      }
    }

    // Zoom out for bigger pyramids.
    scale = Math.max(10, Math.min(24, Math.round(560 / (rows * 0.6 + 8))));
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    dropBall((px - canvas.width / 2) / scale);
  });

  controls.appendChild(
    createInfoBox(
      "The solver is the ported graph-colored soft-constraint pipeline with 4 sub-steps at " +
        "60 Hz, running serially in WebAssembly. A 30-row pyramid is 465 bodies and ~1,300 " +
        "contacts while settling; once asleep it costs almost nothing until something wakes it.",
    ),
  );
  controls.appendChild(
    createSlider("Pyramid rows", 10, 50, rows, 5, (v) => {
      rows = v;
      buildScene();
    }),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createButton("Drop heavy ball", () => dropBall(0.0)));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);

    const t0 = performance.now();
    sim.step(1 / 60, 4);
    const stepMs = performance.now() - t0;
    stepTimes.push(stepMs);
    if (stepTimes.length > 60) stepTimes.shift();
    maxStepMs = Math.max(maxStepMs, stepMs);
    const avgMs = stepTimes.reduce((a, b) => a + b, 0) / stepTimes.length;

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, scale, ORIGIN_Y, shapes, sim.positions());

    const awake = sim.awake_body_count();
    updateReadout(readout, [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Step (avg)", value: `${avgMs.toFixed(2)} ms` },
      { label: "Step (max)", value: `${maxStepMs.toFixed(2)} ms` },
      { label: "Awake", value: awake === 0 ? "asleep" : String(awake) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
