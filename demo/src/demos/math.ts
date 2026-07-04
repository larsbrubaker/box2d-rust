// Deterministic Math — Box2D's hand-rolled trig running as wasm.

import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, fitCanvas, runSimLoop } from "./sim-common.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Deterministic Math",
    "The polygons are rotated with the ported b2MakeRot / b2TransformPoint, using Box2D's " +
      "hand-rolled deterministic cosine/sine — the foundation of cross-platform " +
      "reproducibility.",
    "Rotation driven by b2ComputeCosSin",
  );

  controls.appendChild(
    createInfoBox(
      "Box2D never calls the platform's trig functions during simulation: results must be " +
        "bit-identical on every OS, CPU, and compiler. This port keeps those approximations " +
        "bit-for-bit.",
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const ctx = canvas.getContext("2d")!;
  const start = performance.now();

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const t = (performance.now() - start) / 1000;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // A row of regular polygons (3..8 sides), each rotated by the ported math.
    const count = 6;
    for (let i = 0; i < count; i++) {
      const sides = 3 + i;
      const cx = ((i + 0.5) / count) * canvas.width;
      const cy = canvas.height / 2;
      const radius = Math.min(64, canvas.width / 14);
      const angle = t * (0.3 + 0.15 * i) * (i % 2 === 0 ? 1 : -1);

      const pts = wasm.polygon_points(sides, radius, angle, cx, cy);
      ctx.beginPath();
      ctx.moveTo(pts[0], pts[1]);
      for (let k = 1; k < sides; k++) {
        ctx.lineTo(pts[2 * k], pts[2 * k + 1]);
      }
      ctx.closePath();
      ctx.fillStyle = "rgba(37, 99, 235, 0.10)";
      ctx.fill();
      ctx.strokeStyle = "#2563eb";
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    const cs = wasm.compute_cos_sin(t);
    updateReadout(readout, [
      { label: "t", value: t.toFixed(2) },
      { label: "b2ComputeCosSin", value: `(${cs[0].toFixed(6)}, ${cs[1].toFixed(6)})` },
      { label: "b2Atan2(sin, cos)", value: wasm.atan2(cs[1], cs[0]).toFixed(6) },
    ]);
  }, readout);

  return stop;
}
