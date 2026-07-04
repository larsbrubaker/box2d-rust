// Contact Manifolds — narrow-phase contact points against a fixed box.

import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, fitCanvas, runSimLoop } from "./sim-common.ts";

const SCALE = 90;
const KIND_NAMES = ["box", "circle", "capsule"];

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Contact Manifolds",
    "Contact points and the manifold normal come from the ported b2CollidePolygons / " +
      "b2CollidePolygonAndCircle / b2CollidePolygonAndCapsule. Red points are penetrating " +
      "(negative separation); green are speculative contacts.",
    "Move to drag the shape · click to cycle it",
  );

  let target: [number, number] = [2.2, 0.6];
  let kind = 0; // 0 box, 1 circle, 2 capsule

  controls.appendChild(
    createInfoBox(
      "The moving shape rotates slowly so you can watch the manifold ids persist as contact " +
        "points slide along the fixed box.",
    ),
  );
  const kindGroup = createButtonGroup(
    [
      { label: "Box", value: "0" },
      { label: "Circle", value: "1" },
      { label: "Capsule", value: "2" },
    ],
    "0",
    (v) => {
      kind = parseInt(v, 10);
    },
  );
  controls.appendChild(kindGroup);
  const readout = createReadout();
  controls.appendChild(readout);

  canvas.addEventListener("mousemove", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    target = [(px - canvas.width / 2) / SCALE, (canvas.height / 2 - py) / SCALE];
  });
  canvas.addEventListener("click", () => {
    kind = (kind + 1) % 3;
    const buttons = kindGroup.querySelectorAll("button");
    buttons.forEach((b, i) => b.classList.toggle("active", i === kind));
  });

  const ctx = canvas.getContext("2d")!;
  const start = performance.now();

  const toPx = (x: number, y: number): [number, number] => [
    canvas.width / 2 + x * SCALE,
    canvas.height / 2 - y * SCALE,
  ];

  function drawShapeB(angle: number) {
    const [cx, cy] = target;
    ctx.save();
    const [px, py] = toPx(cx, cy);
    ctx.translate(px, py);
    ctx.rotate(-angle);
    ctx.strokeStyle = "#15803d";
    ctx.fillStyle = "rgba(21, 128, 61, 0.10)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    if (kind === 0) {
      const s = 0.7 * SCALE;
      ctx.rect(-s, -s, 2 * s, 2 * s);
    } else if (kind === 1) {
      ctx.arc(0, 0, 0.6 * SCALE, 0, 2 * Math.PI);
    } else {
      const h = 0.6 * SCALE;
      const r = 0.35 * SCALE;
      ctx.arc(-h, 0, r, Math.PI / 2, -Math.PI / 2);
      ctx.arc(h, 0, r, -Math.PI / 2, Math.PI / 2);
      ctx.closePath();
    }
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const t = (performance.now() - start) / 1000;
    const angle = 0.3 * t;

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Fixed unit box at origin
    ctx.strokeStyle = "#5a6170";
    ctx.fillStyle = "rgba(90, 97, 112, 0.08)";
    ctx.lineWidth = 2;
    const [bx, by] = toPx(-1.0, 1.0);
    ctx.beginPath();
    ctx.rect(bx, by, 2 * SCALE, 2 * SCALE);
    ctx.fill();
    ctx.stroke();

    drawShapeB(angle);

    const m = wasm.collide_with_box(kind, target[0], target[1], angle);
    const pointCount = m[2];

    for (let i = 0; i < pointCount; i++) {
      const px = m[3 + 3 * i];
      const py = m[4 + 3 * i];
      const sep = m[5 + 3 * i];

      const [cx, cy] = toPx(px, py);
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, 2 * Math.PI);
      ctx.fillStyle = sep < 0 ? "#dc2626" : "#15803d";
      ctx.fill();

      // Normal arrow from the contact point
      const [nx2, ny2] = toPx(px + m[0] * 0.5, py + m[1] * 0.5);
      ctx.strokeStyle = "#2563eb";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(nx2, ny2);
      ctx.stroke();
    }

    const entries = [
      { label: "Shape", value: KIND_NAMES[kind] },
      { label: "Points", value: String(pointCount) },
    ];
    if (pointCount > 0) {
      entries.push({ label: "Normal", value: `(${m[0].toFixed(3)}, ${m[1].toFixed(3)})` });
      entries.push({
        label: "Separations",
        value: Array.from({ length: pointCount }, (_, i) => m[5 + 3 * i].toFixed(4)).join(", "),
      });
    }
    updateReadout(readout, entries);
  }, readout);

  return stop;
}
