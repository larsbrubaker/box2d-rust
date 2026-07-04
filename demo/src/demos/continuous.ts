// Continuous — bullets vs a thin wall. Toggle continuous collision to watch
// the same shot tunnel straight through.

import { createButton, createCheckbox, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 42;
const ORIGIN_Y = 40;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Continuous",
    "A 5 cm wall against 100+ m/s bullets. Continuous collision sweeps each bullet's " +
      "trajectory (time of impact) so it stops at the near face; turn it off and the same " +
      "shot crosses the whole wall inside one step.",
    "Click the canvas to fire a bullet at the wall",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let continuous = true;
  let speed = 150;
  let fired = 0;
  let tunneled = 0;
  let bulletIndices: number[] = [];

  function fire(y: number) {
    const index = sim.add_bullet(-10.5, Math.max(0.5, Math.min(9, y)), 0.12, speed, 0);
    bulletIndices.push(index);
    shapes.push({ kind: "circle", r: 0.12, color: continuous ? COLORS.ball : COLORS.heavy });
    fired++;
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    sim.set_continuous(continuous);
    shapes = [];
    bulletIndices = [];
    fired = 0;
    tunneled = 0;

    sim.add_static_box(0.0, -0.5, 11.5, 0.5);
    shapes.push({ kind: "box", hx: 11.5, hy: 0.5, color: COLORS.ground });

    // The thin wall: 10 cm wide, 5 m tall. At 150 m/s a bullet moves ~2.5 m
    // per 60 Hz step, 25x the wall thickness.
    sim.add_static_box(0.0, 5.0, 0.05, 5.0);
    shapes.push({ kind: "box", hx: 0.05, hy: 5.0, color: COLORS.box });
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    fire((canvas.height - ORIGIN_Y - py) / SCALE);
  });

  controls.appendChild(
    createInfoBox(
      "Fast bodies flagged as bullets get a time-of-impact pass after the solver: their swept " +
        "trajectory is re-tested against static geometry and the transform is pulled back to " +
        "the first hit. This is the ported b2SolveContinuous pipeline.",
    ),
  );
  controls.appendChild(
    createCheckbox("Continuous collision", continuous, (v) => {
      continuous = v;
      sim.set_continuous(v);
    }),
  );
  controls.appendChild(
    createSlider("Bullet speed (m/s)", 50, 300, speed, 10, (v) => {
      speed = v;
    }),
  );
  controls.appendChild(createButton("Fire bullet", () => fire(4 + 3 * Math.random())));
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    // A bullet that ends up right of the wall tunneled through it.
    const positions = sim.positions();
    tunneled = 0;
    for (const index of bulletIndices) {
      if (positions[3 * index] > 0.2) tunneled++;
    }

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, positions);

    updateReadout(readout, [
      { label: "Continuous", value: continuous ? "on" : "off" },
      { label: "Bullets fired", value: String(fired) },
      { label: "Tunneled", value: String(tunneled) },
      { label: "Contacts", value: String(sim.contact_count()) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
