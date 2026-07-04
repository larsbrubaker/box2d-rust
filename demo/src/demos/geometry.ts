// Geometry Queries — ray casts and GJK closest points tracking the cursor.

import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, fitCanvas, runSimLoop } from "./sim-common.ts";

const GEO_SCALE = 80;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Geometry Queries",
    "A ray from the left edge tracks the cursor and is cast against every shape with the " +
      "ported b2RayCastPolygon / b2RayCastCircle / b2RayCastCapsule / b2RayCastSegment — the " +
      "pentagon comes from b2ComputeHull. The probe triangle reports GJK closest points from " +
      "b2ShapeDistance.",
    "Move your mouse over the canvas",
  );

  controls.appendChild(
    createInfoBox(
      "Red dots are ray hits with their surface normals. The dashed green line is the " +
        "closest-point witness between the probe triangle and the nearest shape.",
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  // Static scene geometry, fetched once from the Rust port.
  const polygon = wasm.scene_shape(0);
  const circle = wasm.scene_shape(1);
  const capsule = wasm.scene_shape(2);
  const segment = wasm.scene_shape(3);

  const worldToCanvas = (x: number, y: number): [number, number] => [
    canvas.width / 2 + x * GEO_SCALE,
    canvas.height / 2 - y * GEO_SCALE,
  ];
  const canvasToWorld = (px: number, py: number): [number, number] => [
    (px - canvas.width / 2) / GEO_SCALE,
    (canvas.height / 2 - py) / GEO_SCALE,
  ];

  let target: [number, number] = [1.0, -0.5];
  canvas.addEventListener("mousemove", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    target = canvasToWorld(px, py);
  });

  const ACCENT = "#2563eb";
  const SHAPE = "#5a6170";
  const HIT = "#dc2626";
  const GOOD = "#15803d";

  const ctx = canvas.getContext("2d")!;

  function moveTo(x: number, y: number) {
    const [px, py] = worldToCanvas(x, y);
    ctx.moveTo(px, py);
  }
  function lineTo(x: number, y: number) {
    const [px, py] = worldToCanvas(x, y);
    ctx.lineTo(px, py);
  }
  function dot(x: number, y: number, color: string, r = 4) {
    const [px, py] = worldToCanvas(x, y);
    ctx.beginPath();
    ctx.arc(px, py, r, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
  }

  function drawScene() {
    ctx.lineWidth = 2;
    ctx.strokeStyle = SHAPE;
    ctx.fillStyle = "rgba(90, 97, 112, 0.08)";

    // Polygon
    ctx.beginPath();
    moveTo(polygon[0], polygon[1]);
    for (let i = 1; i < polygon.length / 2; i++) {
      lineTo(polygon[2 * i], polygon[2 * i + 1]);
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Circle
    {
      const [px, py] = worldToCanvas(circle[0], circle[1]);
      ctx.beginPath();
      ctx.arc(px, py, circle[2] * GEO_SCALE, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }

    // Capsule: two circles + connecting lines
    {
      const [c1x, c1y] = worldToCanvas(capsule[0], capsule[1]);
      const [c2x, c2y] = worldToCanvas(capsule[2], capsule[3]);
      const r = capsule[4] * GEO_SCALE;
      const angle = Math.atan2(c2y - c1y, c2x - c1x);
      ctx.beginPath();
      ctx.arc(c1x, c1y, r, angle + Math.PI / 2, angle - Math.PI / 2);
      ctx.arc(c2x, c2y, r, angle - Math.PI / 2, angle + Math.PI / 2);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }

    // Segment
    ctx.beginPath();
    moveTo(segment[0], segment[1]);
    lineTo(segment[2], segment[3]);
    ctx.stroke();
  }

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawScene();

    // Ray from a fixed origin on the left toward the cursor, extended 12 m.
    const origin: [number, number] = [-5.2, 0.0];
    const dx = target[0] - origin[0];
    const dy = target[1] - origin[1];
    const len = Math.hypot(dx, dy) || 1;
    const tx = (dx / len) * 12;
    const ty = (dy / len) * 12;

    const results = wasm.ray_cast_scene(origin[0], origin[1], tx, ty);

    // Find nearest hit to clip the drawn ray.
    let nearest = 1.0;
    let hitCount = 0;
    for (let i = 0; i < 4; i++) {
      if (results[6 * i] === 1.0) {
        hitCount++;
        nearest = Math.min(nearest, results[6 * i + 1]);
      }
    }

    ctx.strokeStyle = ACCENT;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    moveTo(origin[0], origin[1]);
    lineTo(origin[0] + tx * nearest, origin[1] + ty * nearest);
    ctx.stroke();
    dot(origin[0], origin[1], ACCENT, 5);

    // Hit points and normals.
    for (let i = 0; i < 4; i++) {
      if (results[6 * i] !== 1.0) continue;
      const hx = results[6 * i + 2];
      const hy = results[6 * i + 3];
      const nx = results[6 * i + 4];
      const ny = results[6 * i + 5];
      dot(hx, hy, HIT);
      ctx.strokeStyle = HIT;
      ctx.beginPath();
      moveTo(hx, hy);
      lineTo(hx + nx * 0.5, hy + ny * 0.5);
      ctx.stroke();
    }

    // GJK probe triangle at the cursor with closest-point witness line.
    const cp = wasm.closest_points(target[0], target[1]);
    ctx.strokeStyle = GOOD;
    ctx.fillStyle = "rgba(21, 128, 61, 0.10)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    moveTo(target[0] - 0.4, target[1] - 0.3);
    lineTo(target[0] + 0.4, target[1] - 0.3);
    lineTo(target[0], target[1] + 0.4);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    if (cp[4] > 0) {
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      moveTo(cp[0], cp[1]);
      lineTo(cp[2], cp[3]);
      ctx.stroke();
      ctx.setLineDash([]);
      dot(cp[0], cp[1], GOOD);
      dot(cp[2], cp[3], GOOD);
    }

    updateReadout(readout, [
      { label: "Ray hits", value: `${hitCount}/4` },
      { label: "Nearest fraction", value: nearest.toFixed(4) },
      { label: "b2ShapeDistance", value: `${cp[4].toFixed(4)} m` },
      { label: "GJK iterations", value: String(cp[5]) },
    ]);
  }, readout);

  return stop;
}
