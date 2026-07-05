// Determinism — two worlds, one cloned from a live snapshot, stepped in
// lockstep. The FNV state hash over every transform and velocity must match
// on every single frame, forever.

import { createButton, createInfoBox, createReadout, createSeparator, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 24;
const ORIGIN_Y = 40;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Determinism",
    "The left world runs live; the right world was cloned from a snapshot of it. Both step " +
      "independently, yet their state hashes stay bit-identical frame after frame — the same " +
      "property the FallingHinges acceptance test verifies against the C engine.",
    "Click either half to drop a ball into BOTH worlds",
  );

  let simA: SimWorld = null as unknown as SimWorld;
  let simB: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let stepCount = 0;
  let matchedSteps = 0;
  let everDiverged = false;

  function buildScene() {
    freeSim(simA);
    freeSim(simB);
    shapes = [];
    stepCount = 0;
    matchedSteps = 0;
    everDiverged = false;

    simA = new wasm.SimWorld(-10.0);
    simA.add_static_box(0.0, -0.5, 9.0, 0.5);
    shapes.push({ kind: "box", hx: 9.0, hy: 0.5, color: COLORS.ground });

    // Hinged pairs and a small pyramid: enough interacting constraints that
    // any divergence would compound within a few steps.
    const h = 0.3;
    for (let row = 0; row < 6; row++) {
      const count = 6 - row;
      for (let i = 0; i < count; i++) {
        const x = (i - (count - 1) / 2) * 2.1 * h - 4.0;
        simA.add_box(x, h + row * 2 * h, h, h, 1.0);
        shapes.push({ kind: "box", hx: h, hy: h, color: COLORS.box });
      }
    }
    for (let j = 0; j < 3; j++) {
      const x = 2.0 + 1.8 * j;
      const a = simA.add_box_rotated(x, 0.3, 0.25, 0.25, 1.0, -0.1);
      shapes.push({ kind: "box", hx: 0.25, hy: 0.25, color: COLORS.ball });
      const b = simA.add_box_rotated(x + 0.1, 0.9, 0.25, 0.25, 1.0, 0.1);
      shapes.push({ kind: "box", hx: 0.25, hy: 0.25, color: COLORS.ball });
      simA.add_hinge_joint(a, b, x - 0.25, 0.6);
    }

    // Let it start moving, then clone world B from a mid-flight snapshot.
    for (let i = 0; i < 30; i++) {
      simA.step(1 / 60, 4);
    }
    simB = new wasm.SimWorld(-10.0);
    // Mirror the scene so the body tracking lists match, then overwrite the
    // simulation state with the snapshot.
    simB.add_static_box(0.0, -0.5, 9.0, 0.5);
    for (let row = 0; row < 6; row++) {
      const count = 6 - row;
      for (let i = 0; i < count; i++) {
        const x = (i - (count - 1) / 2) * 2.1 * h - 4.0;
        simB.add_box(x, h + row * 2 * h, h, h, 1.0);
      }
    }
    for (let j = 0; j < 3; j++) {
      const x = 2.0 + 1.8 * j;
      const a = simB.add_box_rotated(x, 0.3, 0.25, 0.25, 1.0, -0.1);
      const b = simB.add_box_rotated(x + 0.1, 0.9, 0.25, 0.25, 1.0, 0.1);
      simB.add_hinge_joint(a, b, x - 0.25, 0.6);
    }
    const image = simA.snapshot();
    if (!simB.restore(image)) {
      throw new Error("snapshot restore failed");
    }
  }

  function dropBall(x: number) {
    // The same mutation applied to both worlds keeps them in lockstep.
    const cx = Math.max(-8, Math.min(8, x));
    simA.add_circle(cx, 9.0, 0.35, 2.0);
    simB.add_circle(cx, 9.0, 0.35, 2.0);
    shapes.push({ kind: "circle", r: 0.35, color: COLORS.heavy });
  }

  buildScene();

  canvas.addEventListener("click", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    // Both halves map to the same world coordinates.
    const half = canvas.width / 2;
    const local = px < half ? px : px - half;
    dropBall((local - half / 2) / SCALE);
  });

  controls.appendChild(
    createInfoBox(
      "Box2D hand-rolls its trig for cross-platform reproducibility, and this port keeps every " +
        "float operation in the same order as C. The snapshot clone (via <b>b2World_Snapshot / " +
        "b2World_Restore</b>) carries the complete solver state — warm-start impulses included — " +
        "so the twin never drifts.",
    ),
  );
  controls.appendChild(createButton("Reset", () => buildScene()));
  controls.appendChild(createButton("Drop ball in both", () => dropBall(4 * (Math.random() * 2 - 1))));
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    simA.step(1 / 60, 4);
    simB.step(1 / 60, 4);
    stepCount++;

    const hashA = simA.state_hash();
    const hashB = simB.state_hash();
    const matches = hashA === hashB;
    if (matches) {
      matchedSteps++;
    } else {
      everDiverged = true;
    }

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw world A on the left half, world B on the right half.
    const half = Math.floor(canvas.width / 2);
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, half, canvas.height);
    ctx.clip();
    ctx.translate(-half / 2, 0);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, simA.positions());
    ctx.restore();

    ctx.save();
    ctx.beginPath();
    ctx.rect(half, 0, canvas.width - half, canvas.height);
    ctx.clip();
    ctx.translate(half / 2, 0);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, simB.positions());
    ctx.restore();

    // Divider and labels.
    ctx.strokeStyle = "#cbd5e1";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(half + 0.5, 0);
    ctx.lineTo(half + 0.5, canvas.height);
    ctx.stroke();
    ctx.fillStyle = "#64748b";
    ctx.font = "12px Inter, sans-serif";
    ctx.fillText("live world", 10, 18);
    ctx.fillText("snapshot clone", half + 10, 18);

    updateReadout(readout, [
      { label: "Step", value: String(stepCount) },
      { label: "Hash A", value: hashA.slice(0, 8) + "…" },
      { label: "Hash B", value: hashB.slice(0, 8) + "…" },
      {
        label: "Bit-identical",
        value: everDiverged ? "DIVERGED" : `yes (${matchedSteps}/${stepCount})`,
      },
    ]);
  }, readout);

  return () => {
    stop();
    freeSim(simA);
    freeSim(simB);
  };
}
