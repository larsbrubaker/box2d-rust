import { loadWasm } from "./wasm.ts";

// One card per category of the upstream samples app (box2d-cpp-reference/samples).
// Cards flip from "planned" to live pages as engine modules are ported.
const CATEGORIES: Array<{ name: string; blurb: string }> = [
  { name: "Bodies", blurb: "Body types, sleeping, user data" },
  { name: "Shapes", blurb: "Circles, capsules, polygons, chains" },
  { name: "Geometry", blurb: "Hulls, rays, and shape queries" },
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
    card.className = "card soon";
    card.innerHTML = `<h3>${cat.name}</h3><p>${cat.blurb}</p>`;
    grid.appendChild(card);
  }
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
      ctx.strokeStyle = "#4fb3ff";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = "rgba(79, 179, 255, 0.08)";
      ctx.fill();
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
