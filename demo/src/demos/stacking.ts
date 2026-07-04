// Stacking — a box pyramid solved by the ported engine, with island sleeping.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 42;
const ORIGIN_Y = 40;
const MAX_BODIES = 160;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "A box pyramid solved by the ported engine. Watch the island fall asleep once it settles " +
      "(the readout shows the awake body count reaching zero), then wake it back up.",
    "Click the canvas to drop a heavy ball",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let rows = 9;

  function dropBall(x: number) {
    if (sim.body_count() > MAX_BODIES) return;
    sim.add_circle(Math.max(-10, Math.min(10, x)), 9.0, 0.5, 4.0);
    shapes.push({ kind: "circle", r: 0.5, color: COLORS.heavy });
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];

    sim.add_static_box(0.0, -0.5, 11.0, 0.5);
    shapes.push({ kind: "box", hx: 11.0, hy: 0.5, color: COLORS.ground });

    // Pyramid of boxes, like the upstream Stacking sample.
    const h = 0.4;
    for (let row = 0; row < rows; row++) {
      const count = rows - row;
      const y = h + row * 2 * h;
      for (let i = 0; i < count; i++) {
        const x = (i - (count - 1) / 2) * 2.05 * h;
        sim.add_box(x, y, h, h, 1.0);
        shapes.push({ kind: "box", hx: h, hy: h, color: COLORS.box });
      }
    }
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    dropBall((px - canvas.width / 2) / SCALE);
  });

  controls.appendChild(
    createInfoBox(
      "Once every body in the island is slow enough for long enough, the whole island moves " +
        "to a sleeping solver set and costs nothing to simulate. Any new contact wakes it.",
    ),
  );
  controls.appendChild(
    createSlider("Pyramid rows", 3, 14, rows, 1, (v) => {
      rows = v;
      buildScene();
    }),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createButton("Drop heavy ball", () => dropBall(0.3)));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

    const awake = sim.awake_body_count();
    updateReadout(readout, [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Awake", value: String(awake) },
      { label: "Island", value: awake === 0 ? "asleep" : "awake" },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
