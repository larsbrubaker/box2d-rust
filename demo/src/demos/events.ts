// Events — contact begin/end, hit events with impact flashes, and a sensor
// zone, all straight from the ported double-buffered world event arrays.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 42;
const ORIGIN_Y = 40;
const MAX_BODIES = 120;

interface Flash {
  x: number;
  y: number;
  speed: number;
  ttl: number;
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Events",
    "Bouncy balls generate contact begin/end events, hit events above the impact speed " +
      "threshold (drawn as red flashes), and sensor begin/end events in the amber zone.",
    "Click the canvas to drop a bouncy ball",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let restitution = 0.7;
  let flashes: Flash[] = [];
  let totals = { begin: 0, end: 0, hit: 0, sensorBegin: 0, sensorEnd: 0 };

  function dropBall(x: number) {
    if (sim.body_count() > MAX_BODIES) return;
    const index = sim.add_bouncy_ball(Math.max(-9, Math.min(9, x)), 9.5, 0.35, restitution);
    sim.enable_sensor_visitor(index);
    shapes.push({ kind: "circle", r: 0.35, color: COLORS.ball });
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];
    flashes = [];
    totals = { begin: 0, end: 0, hit: 0, sensorBegin: 0, sensorEnd: 0 };

    sim.add_static_box(0.0, -0.5, 10.5, 0.5);
    shapes.push({ kind: "box", hx: 10.5, hy: 0.5, color: COLORS.ground });

    // Angled shelves so the balls scatter (thin static boxes).
    sim.add_static_box(-5.0, 4.0, 2.4, 0.15);
    shapes.push({ kind: "box", hx: 2.4, hy: 0.15, color: COLORS.ground });
    sim.add_static_box(5.0, 5.5, 2.4, 0.15);
    shapes.push({ kind: "box", hx: 2.4, hy: 0.15, color: COLORS.ground });

    // Sensor zone in the middle: counts entries and exits without colliding.
    sim.add_sensor_box(0.0, 1.6, 1.8, 1.6);
    shapes.push({ kind: "box", hx: 1.8, hy: 1.6, color: "#d97706" });

    for (let i = 0; i < 6; i++) {
      dropBall(-6 + 2.4 * i);
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
      "Begin events come from the current step; end events are double-buffered from the " +
        "previous step so nothing is missed between reads &mdash; the same contract as the C API. " +
        "Hit events fire when the approach speed exceeds the world threshold.",
    ),
  );
  controls.appendChild(
    createSlider("Restitution", 0.1, 0.95, restitution, 0.05, (v) => {
      restitution = v;
    }),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createButton("Drop ball", () => dropBall(6 * (Math.random() * 2 - 1))));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    const counts = sim.event_counts();
    totals.begin += counts[0];
    totals.end += counts[1];
    totals.hit += counts[2];
    totals.sensorBegin += counts[3];
    totals.sensorEnd += counts[4];

    const hits = sim.hit_events();
    for (let i = 0; i + 2 < hits.length; i += 3) {
      flashes.push({ x: hits[i], y: hits[i + 1], speed: hits[i + 2], ttl: 24 });
    }

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

    // Impact flashes fade over ~0.4 s, radius scaled by approach speed.
    flashes = flashes.filter((f) => f.ttl-- > 0);
    for (const f of flashes) {
      const alpha = f.ttl / 24;
      const radius = 6 + Math.min(18, 1.6 * f.speed);
      ctx.beginPath();
      ctx.arc(
        canvas.width / 2 + f.x * SCALE,
        canvas.height - ORIGIN_Y - f.y * SCALE,
        radius * (1.4 - 0.4 * alpha),
        0,
        2 * Math.PI,
      );
      ctx.strokeStyle = `rgba(220, 38, 38, ${alpha.toFixed(2)})`;
      ctx.lineWidth = 3;
      ctx.stroke();
    }

    updateReadout(readout, [
      { label: "Contact begin", value: String(totals.begin) },
      { label: "Contact end", value: String(totals.end) },
      { label: "Hit events", value: String(totals.hit) },
      { label: "Sensor begin / end", value: `${totals.sensorBegin} / ${totals.sensorEnd}` },
      { label: "Bodies", value: String(sim.body_count()) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
