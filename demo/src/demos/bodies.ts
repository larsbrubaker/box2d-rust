// Falling Bodies — a full simulation driven by the ported b2World_Step.

import { createButton, createInfoBox, createReadout, createSeparator, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 36;
const ORIGIN_Y = 40; // px from canvas bottom to world y=0
const MAX_BODIES = 140;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Falling Bodies",
    "A full simulation driven by the ported b2World_Step: broad-phase pairs, narrow-phase " +
      "manifolds, graph-colored soft-constraint solving with sub-stepping, restitution, and " +
      "island sleeping.",
    "Click the canvas to drop more bodies",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let spawned = 0;

  function spawn(x: number, y: number, index: number) {
    if (index % 2 === 0) {
      const hx = 0.25 + 0.2 * ((index * 7) % 3) * 0.5;
      sim.add_box(x, y, hx, hx, 1.0);
      shapes.push({ kind: "box", hx, hy: hx, color: COLORS.box });
    } else {
      const r = 0.22 + 0.16 * ((index * 5) % 3) * 0.5;
      sim.add_circle(x, y, r, 1.0);
      shapes.push({ kind: "circle", r, color: COLORS.ball });
    }
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];
    spawned = 0;

    // Ground and two containment walls.
    sim.add_static_box(0.0, -0.5, 13.0, 0.5);
    shapes.push({ kind: "box", hx: 13.0, hy: 0.5, color: COLORS.ground });
    sim.add_static_box(-12.2, 2.0, 0.3, 2.0);
    shapes.push({ kind: "box", hx: 0.3, hy: 2.0, color: COLORS.ground });
    sim.add_static_box(12.2, 2.0, 0.3, 2.0);
    shapes.push({ kind: "box", hx: 0.3, hy: 2.0, color: COLORS.ground });

    // Initial shower of bodies.
    for (let i = 0; i < 24; i++) {
      const x = -6.0 + (i % 8) * 1.7 + 0.13 * (i % 3);
      const y = 5.0 + Math.floor(i / 8) * 1.6;
      spawn(x, y, spawned++);
    }
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    if (sim.body_count() > MAX_BODIES) return;
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const x = (px - canvas.width / 2) / SCALE;
    spawn(Math.max(-11, Math.min(11, x)), 10.5, spawned++);
  });

  controls.appendChild(
    createInfoBox(
      "Every step runs the real physics pipeline in WebAssembly compiled from the Rust port. " +
        "Boxes and balls mix dynamic-vs-dynamic and dynamic-vs-static contacts across the " +
        "constraint graph colors.",
    ),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(
    createButton("Drop 8 more", () => {
      if (sim.body_count() > MAX_BODIES) return;
      for (let i = 0; i < 8; i++) {
        spawn(-5.6 + i * 1.6, 10.5 + 0.8 * (i % 2), spawned++);
      }
    }),
  );
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    // Fixed timestep like the C samples app.
    sim.step(1 / 60, 4);

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

    updateReadout(readout, [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
