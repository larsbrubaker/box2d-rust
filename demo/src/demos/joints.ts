// Joints — falling hinges (revolute) and pendulums (distance), the same
// revolute tuning the determinism acceptance test uses.

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 46;
const ORIGIN_Y = 40;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Joints",
    "Revolute hinges with limits, springs, and motors — the exact scene the determinism " +
      "acceptance test runs — plus rigid distance-joint pendulums. Joints draw as amber lines.",
    "Click the canvas to drop a ball on the hinges",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let rows = 12;

  function dropBall(x: number) {
    sim.add_circle(Math.max(-9, Math.min(9, x)), 9.5, 0.4, 3.0);
    shapes.push({ kind: "circle", r: 0.4, color: COLORS.heavy });
  }

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];

    sim.add_static_box(0.0, -0.5, 10.5, 0.5);
    shapes.push({ kind: "box", hx: 10.5, hy: 0.5, color: COLORS.ground });

    // Falling hinges: columns of tilted boxes, hinged in pairs. As they fall
    // the sprung, motorized hinges fold and the stacks settle.
    // (shared/determinism.c CreateFallingHinges)
    const h = 0.25;
    const columnCount = 3;
    const offset = 0.4 * h;
    const dx = 10.0 * h;
    const xBase = -0.5 * dx * (columnCount - 1) - 4.0;

    for (let j = 0; j < columnCount; j++) {
      const x = xBase + j * dx;
      let prev = -1;
      for (let i = 0; i < rows; i++) {
        const angle = (i & 1) === 0 ? -0.1 : 0.1;
        const index = sim.add_box_rotated(x + offset * i, h + 2 * h * i, h, h, 1.0, angle);
        shapes.push({ kind: "box", hx: h, hy: h, color: COLORS.box });

        if ((i & 1) === 0) {
          prev = index;
        } else {
          // Hinge at the shared corner between the pair.
          sim.add_hinge_joint(prev, index, x + offset * i - h, 2 * h * i);
          prev = -1;
        }
      }
    }

    // Distance-joint pendulums: balls hanging from static anchors at
    // staggered lengths.
    for (let k = 0; k < 3; k++) {
      const ax = 4.0 + 1.6 * k;
      const anchorY = 8.0;
      const length = 2.5 + 0.9 * k;
      const anchor = sim.add_static_box(ax, anchorY, 0.12, 0.12);
      shapes.push({ kind: "box", hx: 0.12, hy: 0.12, color: COLORS.ground });

      // Start displaced to the side so the pendulums swing.
      const ball = sim.add_circle(ax - length, anchorY, 0.35, 1.0);
      shapes.push({ kind: "circle", r: 0.35, color: COLORS.ball });
      sim.add_distance_joint(anchor, ball, ax, anchorY, ax - length, anchorY);
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
      "Each hinge is limited to [-18&deg;, 36&deg;], sprung at 1 Hz, and driven by a weak motor " +
        "&mdash; the same tuning whose settled positions hash bit-for-bit against the C engine. " +
        "The pendulums use rigid distance joints.",
    ),
  );
  controls.appendChild(
    createSlider("Boxes per column", 4, 20, rows, 2, (v) => {
      rows = v;
      buildScene();
    }),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createButton("Drop ball", () => dropBall(-4)));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

    // Joint noodles: world anchors straight from the ported joint frames.
    const anchors = sim.joint_anchors();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#d97706";
    for (let i = 0; i + 3 < anchors.length; i += 4) {
      const [ax, ay] = [canvas.width / 2 + anchors[i] * SCALE, canvas.height - ORIGIN_Y - anchors[i + 1] * SCALE];
      const [bx, by] = [canvas.width / 2 + anchors[i + 2] * SCALE, canvas.height - ORIGIN_Y - anchors[i + 3] * SCALE];
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(ax, ay, 3, 0, 2 * Math.PI);
      ctx.arc(bx, by, 3, 0, 2 * Math.PI);
      ctx.stroke();
    }

    const awake = sim.awake_body_count();
    updateReadout(readout, [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Joints", value: String(sim.joint_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Awake", value: awake === 0 ? "asleep" : String(awake) },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(sim);
  };
}
