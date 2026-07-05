// Robustness — deep overlap recovery and extreme size ratios. The solver's
// push-out speed cap resolves impossible starting states without explosions.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 40;
const ORIGIN_Y = 40;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Robustness",
    "Left: the upstream OverlapRecovery scene — every column spawns its boxes fully " +
      "coincident, and the capped push-out separates them smoothly. Right: a large slab " +
      "resting on tiny boxes stays stable.",
    "Click the canvas to spawn an overlapped cluster",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let baseCount = 4;
  let peakSpeed = 0;

  function spawnCluster(x: number, count: number) {
    // Coincident bodies, exactly like a column of the upstream sample.
    for (let i = 0; i < count; i++) {
      sim.add_box(x, 2.0, 0.5, 0.5, 1.0);
      shapes.push({ kind: "box", hx: 0.5, hy: 0.5, color: COLORS.heavy });
    }
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];
    peakSpeed = 0;

    sim.add_static_box(0.0, -0.5, 11.0, 0.5);
    shapes.push({ kind: "box", hx: 11.0, hy: 0.5, color: COLORS.ground });
    sim.add_static_box(-11.0, 4.0, 0.5, 4.5);
    shapes.push({ kind: "box", hx: 0.5, hy: 4.5, color: COLORS.ground });
    sim.add_static_box(11.0, 4.0, 0.5, 4.5);
    shapes.push({ kind: "box", hx: 0.5, hy: 4.5, color: COLORS.ground });

    // Overlap recovery, the exact upstream OverlapRecovery scene: a pyramid
    // where each column spawns its bodies fully coincident, with columns
    // packed to 75% spacing. (samples/sample_robustness.cpp)
    const extent = 0.5;
    const overlap = 0.25;
    const fraction = 1.0 - overlap;
    const y = extent + 2.0;
    for (let i = 0; i < baseCount; i++) {
      const x = -5.0 + fraction * extent * (i - baseCount);
      for (let j = i; j < baseCount; j++) {
        sim.add_box(x, y, extent, extent, 1.0);
        shapes.push({ kind: "box", hx: extent, hy: extent, color: COLORS.heavy });
      }
    }

    // Extreme mass/size ratio on the right: a big slab resting on tiny boxes,
    // with tiny boxes on top of it.
    sim.add_box(5.0, 0.15, 0.15, 0.15, 1.0);
    shapes.push({ kind: "box", hx: 0.15, hy: 0.15, color: COLORS.ball });
    sim.add_box(6.6, 0.15, 0.15, 0.15, 1.0);
    shapes.push({ kind: "box", hx: 0.15, hy: 0.15, color: COLORS.ball });
    sim.add_box(5.8, 0.65, 2.5, 0.3, 1.0);
    shapes.push({ kind: "box", hx: 2.5, hy: 0.3, color: COLORS.box });
    for (let i = 0; i < 4; i++) {
      sim.add_box(4.6 + 0.8 * i, 1.15, 0.15, 0.15, 1.0);
      shapes.push({ kind: "box", hx: 0.15, hy: 0.15, color: COLORS.ball });
    }
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    spawnCluster(Math.max(-9, Math.min(9, (px - canvas.width / 2) / SCALE)), 4);
  });

  controls.appendChild(
    createInfoBox(
      "Overlap resolution speed is capped per contact (<b>contactSpeed</b>, default 3 m/s) so " +
        "penetration bleeds off over several steps instead of firing bodies apart with one " +
        "giant impulse. Peak height stays bounded and the pile still falls asleep.",
    ),
  );
  controls.appendChild(
    createSlider("Pyramid base", 2, 8, baseCount, 1, (v) => {
      baseCount = v;
      buildScene();
    }),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    const positions = sim.positions();

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, positions);

    // Any body launched above the walls means the recovery exploded.
    let maxY = 0;
    for (let i = 0; i < shapes.length; i++) {
      maxY = Math.max(maxY, positions[3 * i + 1]);
    }
    peakSpeed = Math.max(peakSpeed, maxY);

    const awake = sim.awake_body_count();
    updateReadout(readout, [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Peak height", value: `${peakSpeed.toFixed(1)} m` },
      { label: "Exploded", value: peakSpeed > 15.0 ? "YES" : "no" },
      { label: "Awake", value: awake === 0 ? "asleep" : String(awake) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
