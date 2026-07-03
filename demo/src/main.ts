import { loadWasm } from "./wasm.ts";

// One card per category of the upstream samples app (box2d-cpp-reference/samples).
// Cards flip from "planned" to live as engine modules are ported.
const CATEGORIES: Array<{ name: string; blurb: string; live?: string }> = [
  { name: "Bodies", blurb: "Body types, sleeping, user data" },
  { name: "Shapes", blurb: "Circles, capsules, polygons, chains" },
  { name: "Geometry", blurb: "Hulls, rays, and shape queries", live: "geometry-canvas" },
  { name: "Collision", blurb: "Manifolds, distance, casting" },
  { name: "Stacking", blurb: "Pyramids, towers, and piles" },
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

buildGrid();
runMathDemo().catch((e) => {
  document.getElementById("math-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
runGeometryDemo().catch((e) => {
  document.getElementById("geometry-readout")!.textContent = `Failed to load WASM: ${e}`;
  console.error(e);
});
