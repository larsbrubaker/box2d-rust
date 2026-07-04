// Shapes — every shape type living together: a chain-shape terrain with
// boxes, circles, and capsules raining onto it.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 40;
const ORIGIN_Y = 60;
const MAX_BODIES = 150;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Shapes",
    "Chain-shape terrain (one-sided segments with ghost vertices for smooth sliding) under a " +
      "rain of boxes, circles, and capsules — every shape type the engine supports, colliding.",
    "Click the canvas to drop a random shape",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let rainCount = 24;

  function dropShape(x: number, kind?: number) {
    if (sim.body_count() > MAX_BODIES) return;
    const cx = Math.max(-10, Math.min(10, x));
    const y = 8.5 + 2 * Math.random();
    const angle = Math.PI * (2 * Math.random() - 1);
    const pick = kind ?? Math.floor(Math.random() * 3);
    if (pick === 0) {
      const h = 0.25 + 0.2 * Math.random();
      sim.add_box_rotated(cx, y, h, h, 1.0, angle);
      shapes.push({ kind: "box", hx: h, hy: h, color: COLORS.box });
    } else if (pick === 1) {
      const r = 0.2 + 0.15 * Math.random();
      sim.add_circle(cx, y, r, 1.0);
      shapes.push({ kind: "circle", r, color: COLORS.ball });
    } else {
      const hl = 0.3 + 0.2 * Math.random();
      const r = 0.15 + 0.1 * Math.random();
      sim.add_capsule(cx, y, hl, r, 1.0, angle);
      shapes.push({ kind: "capsule", hl, r, color: "#d97706" });
    }
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];

    // Rolling terrain as one open chain. Chains are one-sided (wind
    // right-to-left for solid-side-up), and an open chain's first and last
    // points are ghost vertices only — so the containment walls must sit
    // inside dedicated ghost endpoints or bodies roll off the open ends.
    const points: number[] = [];
    const n = 40;
    points.push(12.0, 10.0); // right ghost
    points.push(11.4, 9.0); // right wall (solid)
    for (let i = 0; i <= n; i++) {
      const x = 11.0 - (22.0 * i) / n;
      const y = 1.1 + 1.0 * Math.cos((x / 11.0) * Math.PI) + 0.55 * Math.sin(0.9 * x);
      points.push(x, y);
    }
    points.push(-11.4, 9.0); // left wall (solid)
    points.push(-12.0, 10.0); // left ghost
    sim.add_chain(points, false);
    // Draw only the solid segments (skip the ghost endpoints).
    shapes.push({ kind: "chain", points: points.slice(2, -2), loop: false, color: COLORS.ground });

    for (let i = 0; i < rainCount; i++) {
      dropShape(-9 + (18 * i) / Math.max(1, rainCount - 1), i % 3);
    }
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    dropShape((px - canvas.width / 2) / SCALE);
  });

  controls.appendChild(
    createInfoBox(
      "The terrain is a single <b>b2CreateChain</b> call: each solid segment carries ghost " +
        "vertices from its neighbors so shapes slide across the joins without snagging. " +
        "Capsules use their own dedicated collision routines against every other type.",
    ),
  );
  controls.appendChild(
    createSlider("Shape rain", 6, 60, rainCount, 6, (v) => {
      rainCount = v;
      buildScene();
    }),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createButton("Drop box", () => dropShape(6 * (Math.random() * 2 - 1), 0)));
  controls.appendChild(createButton("Drop circle", () => dropShape(6 * (Math.random() * 2 - 1), 1)));
  controls.appendChild(createButton("Drop capsule", () => dropShape(6 * (Math.random() * 2 - 1), 2)));
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
      { label: "Awake", value: awake === 0 ? "asleep" : String(awake) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
