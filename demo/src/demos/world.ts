// World — explosions and runtime gravity, straight through b2World_Explode
// and b2World_SetGravity.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 40;
const ORIGIN_Y = 40;

interface Blast {
  x: number;
  y: number;
  radius: number;
  ttl: number;
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "World",
    "Click anywhere to detonate a radial explosion: the impulse scales with each shape's " +
      "perimeter facing the blast and falls off past the radius. Gravity is adjustable live.",
    "Click the canvas to explode",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let gravityY = -10;
  let radius = 2.0;
  let impulse = 6.0;
  let blasts: Blast[] = [];
  let explosionCount = 0;

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(gravityY);
    shapes = [];
    blasts = [];
    explosionCount = 0;

    // Container: floor + walls + ceiling so bodies stay in view even with
    // gravity flipped.
    const walls: Array<[number, number, number, number]> = [
      [0.0, -0.5, 11.0, 0.5],
      [0.0, 11.0, 11.0, 0.5],
      [-11.0, 5.25, 0.5, 6.25],
      [11.0, 5.25, 0.5, 6.25],
    ];
    for (const [x, y, hx, hy] of walls) {
      sim.add_static_box(x, y, hx, hy);
      shapes.push({ kind: "box", hx, hy, color: COLORS.ground });
    }

    // A grid of light boxes and a few heavier circles to toss around.
    for (let row = 0; row < 5; row++) {
      for (let col = 0; col < 12; col++) {
        const x = -8.25 + 1.5 * col;
        const y = 0.4 + 0.8 * row;
        sim.add_box(x, y, 0.35, 0.35, 1.0);
        shapes.push({ kind: "box", hx: 0.35, hy: 0.35, color: COLORS.box });
      }
    }
    for (let i = 0; i < 4; i++) {
      sim.add_circle(-6 + 4 * i, 4.5, 0.5, 3.0);
      shapes.push({ kind: "circle", r: 0.5, color: COLORS.heavy });
    }
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const wx = (px - canvas.width / 2) / SCALE;
    const wy = (canvas.height - ORIGIN_Y - py) / SCALE;
    sim.explode(wx, wy, radius, radius, impulse);
    blasts.push({ x: wx, y: wy, radius, ttl: 20 });
    explosionCount++;
  });

  controls.appendChild(
    createInfoBox(
      "<b>b2World_Explode</b> queries the dynamic broad-phase tree around the blast, computes " +
        "each shape's projected perimeter facing the explosion, and applies a matching impulse " +
        "at the closest point &mdash; so bigger faces catch more blast, and off-center hits spin.",
    ),
  );
  controls.appendChild(
    createSlider("Gravity", -20, 20, gravityY, 1, (v) => {
      gravityY = v;
      sim.set_gravity(0, v);
    }),
  );
  controls.appendChild(
    createSlider("Blast radius", 0.5, 5, radius, 0.5, (v) => {
      radius = v;
    }),
  );
  controls.appendChild(
    createSlider("Impulse per length", 1, 20, impulse, 1, (v) => {
      impulse = v;
    }),
  );
  controls.appendChild(createButton("Explode center", () => {
    sim.explode(0, 3, radius, radius, impulse);
    blasts.push({ x: 0, y: 3, radius, ttl: 20 });
    explosionCount++;
  }));
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

    // Expanding blast rings.
    blasts = blasts.filter((b) => b.ttl-- > 0);
    for (const b of blasts) {
      const t = 1 - b.ttl / 20;
      ctx.beginPath();
      ctx.arc(
        canvas.width / 2 + b.x * SCALE,
        canvas.height - ORIGIN_Y - b.y * SCALE,
        (0.3 + t * b.radius) * SCALE,
        0,
        2 * Math.PI,
      );
      ctx.strokeStyle = `rgba(220, 38, 38, ${(1 - t).toFixed(2)})`;
      ctx.lineWidth = 3;
      ctx.stroke();
    }

    const awake = sim.awake_body_count();
    updateReadout(readout, [
      { label: "Explosions", value: String(explosionCount) },
      { label: "Gravity", value: gravityY.toFixed(0) },
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Awake", value: awake === 0 ? "asleep" : String(awake) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
