import { loadWasm } from "./wasm.ts";

// One card per category of the upstream samples app (box2d-cpp-reference/samples).
// Cards flip from "planned" to live as engine modules are ported.
const CATEGORIES: Array<{ name: string; blurb: string; live?: string }> = [
  { name: "Bodies", blurb: "Body types, sleeping, user data", live: "bodies-canvas" },
  { name: "Shapes", blurb: "Circles, capsules, polygons, chains" },
  { name: "Geometry", blurb: "Hulls, rays, and shape queries", live: "geometry-canvas" },
  { name: "Collision", blurb: "Manifolds, distance, casting", live: "manifold-canvas" },
  { name: "Stacking", blurb: "Pyramids, towers, and piles", live: "stacking-canvas" },
  { name: "Joints", blurb: "Revolute, prismatic, wheel, weld…" },
  { name: "Continuous", blurb: "Fast bodies without tunneling" },
  { name: "Events", blurb: "Contacts, sensors, hit events" },
  { name: "Character", blurb: "Movers and platforming" },
  { name: "World", blurb: "Gravity, explosions, large worlds" },
  { name: "Determinism", blurb: "Cross-platform reproducibility" },
  { name: "Robustness", blurb: "Degenerate input, overlap recovery" },
  { name: "Benchmark", blurb: "Performance stress scenes" },
];

function buildGrid() {
  const grid = document.getElementById("demo-grid")!;
  for (const cat of CATEGORIES) {
    const card = document.createElement("div");
    if (cat.live) {
      card.className = "card done";
      card.innerHTML =
        `<h3><span>${cat.name}</span><span class="status">live</span></h3>` +
        `<p>${cat.blurb}</p>`;
      card.style.cursor = "pointer";
      card.addEventListener("click", () => {
        document.getElementById(cat.live!)?.scrollIntoView({ behavior: "smooth", block: "center" });
      });
    } else {
      card.className = "card soon";
      card.innerHTML =
        `<h3><span>${cat.name}</span><span class="status">planned</span></h3>` +
        `<p>${cat.blurb}</p>`;
    }
    grid.appendChild(card);
  }
}

// World (meters) <-> canvas (pixels) mapping for the geometry demo.
const GEO_SCALE = 80;
function worldToCanvas(canvas: HTMLCanvasElement, x: number, y: number): [number, number] {
  return [canvas.width / 2 + x * GEO_SCALE, canvas.height / 2 - y * GEO_SCALE];
}
function canvasToWorld(canvas: HTMLCanvasElement, px: number, py: number): [number, number] {
  return [(px - canvas.width / 2) / GEO_SCALE, (canvas.height / 2 - py) / GEO_SCALE];
}

async function runGeometryDemo() {
  const wasm = await loadWasm();

  const canvas = document.getElementById("geometry-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const readout = document.getElementById("geometry-readout")!;

  // Static scene geometry, fetched once from the Rust port.
  const polygon = wasm.scene_shape(0);
  const circle = wasm.scene_shape(1);
  const capsule = wasm.scene_shape(2);
  const segment = wasm.scene_shape(3);

  // Cursor state in world coordinates.
  let target: [number, number] = [1.0, -0.5];
  canvas.addEventListener("mousemove", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    target = canvasToWorld(canvas, px, py);
  });

  const ACCENT = "#2563eb";
  const SHAPE = "#5a6170";
  const HIT = "#dc2626";
  const GOOD = "#15803d";

  function moveTo(x: number, y: number) {
    const [px, py] = worldToCanvas(canvas, x, y);
    ctx.moveTo(px, py);
  }
  function lineTo(x: number, y: number) {
    const [px, py] = worldToCanvas(canvas, x, y);
    ctx.lineTo(px, py);
  }
  function dot(x: number, y: number, color: string, r = 4) {
    const [px, py] = worldToCanvas(canvas, x, y);
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
      const [px, py] = worldToCanvas(canvas, circle[0], circle[1]);
      ctx.beginPath();
      ctx.arc(px, py, circle[2] * GEO_SCALE, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }

    // Capsule: two circles + connecting lines
    {
      const [c1x, c1y] = worldToCanvas(canvas, capsule[0], capsule[1]);
      const [c2x, c2y] = worldToCanvas(canvas, capsule[2], capsule[3]);
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

  function frame() {
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

    readout.textContent =
      `ray hits: ${hitCount}/4   nearest fraction: ${nearest.toFixed(4)}   ` +
      `b2ShapeDistance: ${cp[4].toFixed(4)} m in ${cp[5]} iterations`;

    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}

async function runManifoldDemo() {
  const wasm = await loadWasm();

  const canvas = document.getElementById("manifold-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const readout = document.getElementById("manifold-readout")!;

  const SCALE = 90;
  const toPx = (x: number, y: number): [number, number] => [
    canvas.width / 2 + x * SCALE,
    canvas.height / 2 - y * SCALE,
  ];

  let target: [number, number] = [2.2, 0.6];
  let kind = 0; // 0 box, 1 circle, 2 capsule
  const KIND_NAMES = ["box", "circle", "capsule"];

  canvas.addEventListener("mousemove", (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    target = [(px - canvas.width / 2) / SCALE, (canvas.height / 2 - py) / SCALE];
  });
  canvas.addEventListener("click", () => {
    kind = (kind + 1) % 3;
  });

  const start = performance.now();

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

  function frame() {
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

    readout.textContent =
      `shape: ${KIND_NAMES[kind]} (click to cycle)   points: ${pointCount}` +
      (pointCount > 0
        ? `   normal: (${m[0].toFixed(3)}, ${m[1].toFixed(3)})   separations: ` +
          Array.from({ length: pointCount }, (_, i) => m[5 + 3 * i].toFixed(4)).join(", ")
        : "");

    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}

async function runMathDemo() {
  const wasm = await loadWasm();

  document.getElementById("version-badge")!.textContent = `v${wasm.version()} · wasm`;

  const canvas = document.getElementById("math-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const readout = document.getElementById("math-readout")!;

  const start = performance.now();

  function frame() {
    const t = (performance.now() - start) / 1000;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // A row of regular polygons (3..8 sides), each rotated by the ported math.
    const count = 6;
    for (let i = 0; i < count; i++) {
      const sides = 3 + i;
      const cx = ((i + 0.5) / count) * canvas.width;
      const cy = canvas.height / 2;
      const radius = 52;
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
    readout.textContent =
      `b2ComputeCosSin(${t.toFixed(2)}) = (${cs[0].toFixed(6)}, ${cs[1].toFixed(6)})   ` +
      `b2Atan2(sin, cos) = ${wasm.atan2(cs[1], cs[0]).toFixed(6)}`;

    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}

// Shared renderer for the simulation demos. Bodies are tracked JS-side as
// parallel shape descriptors; positions come from the ported engine.
type SimShape =
  | { kind: "box"; hx: number; hy: number; color: string }
  | { kind: "circle"; r: number; color: string };

function drawSimBodies(
  canvas: HTMLCanvasElement,
  scale: number,
  originY: number,
  shapes: SimShape[],
  positions: Float32Array,
) {
  const ctx = canvas.getContext("2d")!;
  const toPx = (x: number, y: number): [number, number] => [
    canvas.width / 2 + x * scale,
    canvas.height - originY - y * scale,
  ];

  for (let i = 0; i < shapes.length; i++) {
    const shape = shapes[i];
    const x = positions[3 * i];
    const y = positions[3 * i + 1];
    const angle = positions[3 * i + 2];
    const [px, py] = toPx(x, y);

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-angle);
    ctx.lineWidth = 2;
    ctx.strokeStyle = shape.color;
    ctx.fillStyle = shape.color + "1a"; // 10% alpha
    ctx.beginPath();
    if (shape.kind === "box") {
      ctx.rect(-shape.hx * scale, -shape.hy * scale, 2 * shape.hx * scale, 2 * shape.hy * scale);
    } else {
      ctx.arc(0, 0, shape.r * scale, 0, 2 * Math.PI);
      // radius line so rotation is visible
      ctx.moveTo(0, 0);
      ctx.lineTo(shape.r * scale, 0);
    }
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}

async function runBodiesDemo() {
  const wasm = await loadWasm();

  const canvas = document.getElementById("bodies-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const readout = document.getElementById("bodies-readout")!;

  const SCALE = 36;
  const ORIGIN_Y = 40; // px from canvas bottom to world y=0

  const sim = new wasm.SimWorld(-10.0);
  const shapes: SimShape[] = [];

  const GROUND = "#5a6170";
  const BOX = "#2563eb";
  const BALL = "#15803d";

  // Ground and two containment walls.
  sim.add_static_box(0.0, -0.5, 13.0, 0.5);
  shapes.push({ kind: "box", hx: 13.0, hy: 0.5, color: GROUND });
  sim.add_static_box(-12.2, 2.0, 0.3, 2.0);
  shapes.push({ kind: "box", hx: 0.3, hy: 2.0, color: GROUND });
  sim.add_static_box(12.2, 2.0, 0.3, 2.0);
  shapes.push({ kind: "box", hx: 0.3, hy: 2.0, color: GROUND });

  function spawn(x: number, y: number, index: number) {
    if (index % 2 === 0) {
      const hx = 0.25 + 0.2 * ((index * 7) % 3) * 0.5;
      sim.add_box(x, y, hx, hx, 1.0);
      shapes.push({ kind: "box", hx, hy: hx, color: BOX });
    } else {
      const r = 0.22 + 0.16 * ((index * 5) % 3) * 0.5;
      sim.add_circle(x, y, r, 1.0);
      shapes.push({ kind: "circle", r, color: BALL });
    }
  }

  // Initial shower of bodies.
  let spawned = 0;
  for (let i = 0; i < 24; i++) {
    const x = -6.0 + (i % 8) * 1.7 + 0.13 * (i % 3);
    const y = 5.0 + Math.floor(i / 8) * 1.6;
    spawn(x, y, spawned++);
  }

  canvas.addEventListener("click", (e) => {
    if (sim.body_count() > 140) return;
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const x = (px - canvas.width / 2) / SCALE;
    spawn(Math.max(-11, Math.min(11, x)), 10.5, spawned++);
  });

  function frame() {
    try {
      // Fixed timestep like the C samples app.
      sim.step(1 / 60, 4);

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

      readout.textContent =
        `bodies: ${sim.body_count()}   contacts: ${sim.contact_count()}   ` +
        `awake: ${sim.awake_body_count()}   (click to drop more)`;
    } catch (e) {
      readout.textContent = `Simulation error: ${e}`;
      console.error(e);
      return;
    }

    requestAnimationFrame(frame);
  }

  // Render immediately so the scene is visible even before the first
  // animation frame fires (hidden/throttled tabs suspend rAF).
  frame();
}

async function runStackingDemo() {
  const wasm = await loadWasm();

  const canvas = document.getElementById("stacking-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const readout = document.getElementById("stacking-readout")!;

  const SCALE = 42;
  const ORIGIN_Y = 40;

  const sim = new wasm.SimWorld(-10.0);
  const shapes: SimShape[] = [];

  sim.add_static_box(0.0, -0.5, 11.0, 0.5);
  shapes.push({ kind: "box", hx: 11.0, hy: 0.5, color: "#5a6170" });

  // Pyramid of boxes (base 9), like the upstream Stacking sample.
  const H = 0.4;
  const BASE = 9;
  for (let row = 0; row < BASE; row++) {
    const count = BASE - row;
    const y = H + row * 2 * H;
    for (let i = 0; i < count; i++) {
      const x = (i - (count - 1) / 2) * 2.05 * H;
      sim.add_box(x, y, H, H, 1.0);
      shapes.push({ kind: "box", hx: H, hy: H, color: "#2563eb" });
    }
  }

  canvas.addEventListener("click", (e) => {
    if (sim.body_count() > 120) return;
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const x = (px - canvas.width / 2) / SCALE;
    sim.add_circle(Math.max(-10, Math.min(10, x)), 9.0, 0.5, 4.0);
    shapes.push({ kind: "circle", r: 0.5, color: "#dc2626" });
  });

  function frame() {
    try {
      sim.step(1 / 60, 4);

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

      const awake = sim.awake_body_count();
      readout.textContent =
        `bodies: ${sim.body_count()}   contacts: ${sim.contact_count()}   awake: ${awake}` +
        (awake === 0 ? "   — island asleep, click to wake it" : "   (click to drop a heavy ball)");
    } catch (e) {
      readout.textContent = `Simulation error: ${e}`;
      console.error(e);
      return;
    }

    requestAnimationFrame(frame);
  }

  frame();
}

buildGrid();
runMathDemo().catch((e) => {
  document.getElementById("math-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
runGeometryDemo().catch((e) => {
  document.getElementById("geometry-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
runManifoldDemo().catch((e) => {
  document.getElementById("manifold-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
runBodiesDemo().catch((e) => {
  document.getElementById("bodies-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
runStackingDemo().catch((e) => {
  document.getElementById("stacking-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
