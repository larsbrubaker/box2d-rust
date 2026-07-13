// Collision — RegisterSample ports from sample_collision.cpp.
// C citations use sample_collision.cpp line numbers at the pinned submodule.

import {
  createButton,
  createCheckbox,
  createDropdown,
  createInfoBox,
  createReadout,
  createSeparator,
  createSlider,
  updateReadout,
} from "../controls.ts";
import { assertRouteScenes } from "../registry.ts";
import { getWasm, type Box2dWasm, type SimWorld } from "../wasm.ts";
import { paintDebugDraw } from "./debug-draw.ts";
import { demoPage, fitCanvas, freeSim, runSimLoop } from "./sim-common.ts";
import {
  createSampleTransport,
  disposeTransport,
  makeCamera,
  screenToWorld,
  viewBounds,
  worldToScreen,
  type SampleCamera,
} from "./sample-shell.ts";

export const SCENES = [
  "shape-distance",
  "dynamic-tree",
  "ray-cast",
  "cast-world",
  "overlap-world",
  "manifold",
  "smooth-manifold",
  "shape-cast",
  "time-of-impact",
] as const;

export type Scene = (typeof SCENES)[number];
assertRouteScenes("collision", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "shape-distance": "Shape Distance",
  "dynamic-tree": "Dynamic Tree",
  "ray-cast": "Ray Cast",
  "cast-world": "Cast World",
  "overlap-world": "Overlap World",
  manifold: "Manifold",
  "smooth-manifold": "Smooth Manifold",
  "shape-cast": "Shape Cast",
  "time-of-impact": "Time of Impact",
};

/** C camera.center / camera.zoom (half-height). */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "shape-distance": { cx: 0.0, cy: 0.0, zoom: 3.0 }, // :32-33
  "dynamic-tree": { cx: 500.0, cy: 500.0, zoom: 25.0 * 21.0 }, // :473-474
  "ray-cast": { cx: 0.0, cy: 20.0, zoom: 17.5 }, // :877-878
  "cast-world": { cx: 2.0, cy: 14.0, zoom: 25.0 * 0.75 }, // :1388-1389
  "overlap-world": { cx: 0.0, cy: 10.0, zoom: 25.0 * 0.7 }, // :1901-1902
  manifold: { cx: 1.8, cy: 0.0, zoom: 25.0 * 0.45 }, // :2230-2231
  "smooth-manifold": { cx: 2.0, cy: 20.0, zoom: 21.0 }, // :2896-2897
  "shape-cast": { cx: 0.0, cy: 0.25, zoom: 3.0 }, // :3189-3190
  "time-of-impact": { cx: -16.0, cy: 45.0, zoom: 5.0 }, // :3581-3582
};

const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const PI = Math.PI;

// Smooth Manifold chain path (sample_collision.cpp:2918-2954)
const SMOOTH_POINTS: number[] = [
  -20.58325, 14.54175, -21.90625, 15.8645, -24.552, 17.1875, -27.198, 11.89575, -29.84375, 15.8645,
  -29.84375, 21.15625, -25.875, 23.802, -20.58325, 25.125, -25.875, 29.09375, -20.58325, 31.7395,
  -11.0089998, 23.2290001, -8.67700005, 21.15625, -6.03125, 21.15625, -7.35424995, 29.09375,
  -3.38549995, 29.09375, 1.90625, 30.41675, 5.875, 17.1875, 11.16675, 25.125, 9.84375, 29.09375,
  13.8125, 31.7395, 21.75, 30.41675, 28.3644981, 26.448, 25.71875, 18.5105, 24.3957481, 13.21875,
  17.78125, 11.89575, 15.1355, 7.92700005, 5.875, 9.25, 1.90625, 11.89575, -3.25, 11.89575, -3.25,
  9.9375, -4.70825005, 9.25, -8.67700005, 9.25, -11.323, 11.89575, -13.96875, 11.89575, -15.29175,
  14.54175, -19.2605, 14.54175,
];

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  paintOverlay?: (ctx: CanvasRenderingContext2D, camera: SampleCamera, canvas: HTMLCanvasElement) => void;
  readoutExtra?: () => { label: string; value: string }[];
  onPointerDown?: (wx: number, wy: number, mods: { shift: boolean; ctrl: boolean }) => void;
  onPointerMove?: (wx: number, wy: number) => void;
  onPointerUp?: () => void;
  dispose?: () => void;
  /** Non-sim scenes (distance/manifold/tree) skip world step. */
  skipSim?: boolean;
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

function makeRng(seed = 12345) {
  let s = seed >>> 0;
  const next = () => {
    let x = s;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    s = x >>> 0;
    return s & 0x7fff;
  };
  const floatRange = (lo: number, hi: number) => {
    const r = (next() & 0x7fff) / 0x7fff;
    return (1 - r) * lo + r * hi;
  };
  return { next, floatRange };
}

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}

function drawDot(
  ctx: CanvasRenderingContext2D,
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  x: number,
  y: number,
  color: string,
  r = 4,
) {
  const p = worldToScreen(camera, canvas, x, y);
  ctx.beginPath();
  ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function drawSeg(
  ctx: CanvasRenderingContext2D,
  camera: SampleCamera,
  canvas: HTMLCanvasElement,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  color: string,
  width = 1.5,
) {
  const a = worldToScreen(camera, canvas, x0, y0);
  const b = worldToScreen(camera, canvas, x1, y1);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
}

function boxVerts(hx: number, hy: number): number[] {
  return [-hx, -hy, hx, -hy, hx, hy, -hx, hy];
}

// ---------------------------------------------------------------------------
// Shape Distance — sample_collision.cpp:16-439
// ---------------------------------------------------------------------------

function buildShapeDistance(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  let typeA = 3; // box
  let typeB = 3;
  let radiusA = 0.0;
  let radiusB = 0.0;
  let tx = 0.0;
  let ty = 0.0;
  let angle = 0.0;
  let dragging = false;
  let rotating = false;
  let startX = 0;
  let startY = 0;
  let baseX = 0;
  let baseY = 0;
  let baseAngle = 0;
  let lastDist = 0;
  let lastIters = 0;

  const square = boxVerts(0.5, 0.5);
  const segment = [-0.5, 0.0, 0.5, 0.0];
  const triangle = [-0.5, 0.0, 0.5, 0.0, 0.0, 1.0];
  const point = [0.0, 0.0];

  function proxy(type: number): number[] {
    if (type === 0) return point;
    if (type === 1) return segment;
    if (type === 2) return triangle;
    return square;
  }

  controls.appendChild(
    createDropdown(
      "shape A",
      [
        { value: "0", text: "point" },
        { value: "1", text: "segment" },
        { value: "2", text: "triangle" },
        { value: "3", text: "box" },
      ],
      String(typeA),
      (v) => {
        typeA = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createSlider("radius A", 0, 0.5, radiusA, 0.01, (v) => {
      radiusA = v;
    }),
  );
  controls.appendChild(
    createDropdown(
      "shape B",
      [
        { value: "0", text: "point" },
        { value: "1", text: "segment" },
        { value: "2", text: "triangle" },
        { value: "3", text: "box" },
      ],
      String(typeB),
      (v) => {
        typeB = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createSlider("radius B", 0, 0.5, radiusB, 0.01, (v) => {
      radiusB = v;
    }),
  );
  controls.appendChild(
    createSlider("x offset", -2, 2, tx, 0.01, (v) => {
      tx = v;
    }),
  );
  controls.appendChild(
    createSlider("y offset", -2, 2, ty, 0.01, (v) => {
      ty = v;
    }),
  );
  controls.appendChild(
    createSlider("angle", -PI, PI, angle, 0.01, (v) => {
      angle = v;
    }),
  );
  controls.appendChild(
    createInfoBox("Drag to translate B · Shift+drag to rotate · values from b2ShapeDistance"),
  );

  return {
    skipSim: true,
    onPointerDown: (wx, wy, mods) => {
      startX = wx;
      startY = wy;
      if (mods.shift) {
        rotating = true;
        baseAngle = angle;
      } else {
        dragging = true;
        baseX = tx;
        baseY = ty;
      }
    },
    onPointerMove: (wx, wy) => {
      const dx = wx - startX;
      const dy = wy - startY;
      if (dragging) {
        tx = baseX + 0.5 * dx;
        ty = baseY + 0.5 * dy;
      } else if (rotating) {
        angle = clamp(baseAngle + 1.0 * dx, -PI, PI);
      }
    },
    onPointerUp: () => {
      dragging = false;
      rotating = false;
    },
    paintOverlay: (ctx, camera, canvas) => {
      const useRadii = radiusA > 0 || radiusB > 0;
      const out = wasm.collision_shape_distance(
        proxy(typeA),
        radiusA,
        proxy(typeB),
        radiusB,
        tx,
        ty,
        angle,
        useRadii,
      );
      lastDist = out[4]!;
      lastIters = out[5]!;
      // Draw A at identity (cyan), B at transform (bisque) — simplified outlines
      const drawProxy = (pts: number[], ox: number, oy: number, oa: number, color: string) => {
        const c = Math.cos(oa);
        const s = Math.sin(oa);
        const n = pts.length / 2;
        if (n === 1) {
          drawDot(ctx, camera, canvas, ox + pts[0]!, oy + pts[1]!, color, 5);
          return;
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
          const lx = pts[i * 2]!;
          const ly = pts[i * 2 + 1]!;
          const wx = ox + c * lx - s * ly;
          const wy = oy + s * lx + c * ly;
          const p = worldToScreen(camera, canvas, wx, wy);
          if (i === 0) ctx.moveTo(p.x, p.y);
          else ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.stroke();
      };
      drawProxy(proxy(typeA), 0, 0, 0, "#00ffff");
      drawProxy(proxy(typeB), tx, ty, angle, "#ffe4c4");
      drawSeg(ctx, camera, canvas, out[0]!, out[1]!, out[2]!, out[3]!, "#696969");
      drawDot(ctx, camera, canvas, out[0]!, out[1]!, "#ffffff", 5);
      drawDot(ctx, camera, canvas, out[2]!, out[3]!, "#ffffff", 5);
      if (lastDist > 0) {
        drawSeg(
          ctx,
          camera,
          canvas,
          out[0]!,
          out[1]!,
          out[0]! + 0.5 * out[6]!,
          out[1]! + 0.5 * out[7]!,
          "#9400d3",
        );
      }
    },
    readoutExtra: () => [
      { label: "distance", value: lastDist.toFixed(4) },
      { label: "iterations", value: String(lastIters) },
    ],
  };
}

// ---------------------------------------------------------------------------
// Dynamic Tree — sample_collision.cpp:465-867 (debug 100×100 grid → partial)
// ---------------------------------------------------------------------------

function buildDynamicTree(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  const tree = new wasm.TreeDemo();
  let queryDrag = false;
  let rayDrag = false;
  let qx0 = 0;
  let qy0 = 0;
  let qx1 = 0;
  let qy1 = 0;
  let rsx = 0;
  let rsy = 0;
  let rex = 0;
  let rey = 0;
  let updateType = 0;

  controls.appendChild(
    createInfoBox(
      "Partial: uses C <code>m_isDebug</code> grid (100×100), not release 1000×1000. " +
        "Drag AABB query · Shift+drag ray cast.",
    ),
  );
  controls.appendChild(
    createSlider("rows", 0, 200, tree.row_count(), 1, (v) => {
      tree.set_rows(v | 0);
      tree.build_tree();
    }),
  );
  controls.appendChild(
    createSlider("columns", 0, 200, tree.column_count(), 1, (v) => {
      tree.set_columns(v | 0);
      tree.build_tree();
    }),
  );
  controls.appendChild(
    createSlider("fill", 0, 1, 0.25, 0.01, (v) => {
      tree.set_fill(v);
      tree.build_tree();
    }),
  );
  controls.appendChild(
    createDropdown(
      "update",
      [
        { value: "0", text: "Incremental" },
        { value: "1", text: "Partial Rebuild" },
        { value: "2", text: "Full Rebuild" },
      ],
      "0",
      (v) => {
        updateType = parseInt(v, 10);
        tree.set_update_type(updateType);
      },
    ),
  );
  controls.appendChild(createButton("Rebuild", () => tree.build_tree()));

  return {
    skipSim: true,
    beforeStep: () => tree.step(),
    onPointerDown: (wx, wy, mods) => {
      if (mods.shift) {
        rayDrag = true;
        rsx = rex = wx;
        rsy = rey = wy;
      } else {
        queryDrag = true;
        qx0 = qx1 = wx;
        qy0 = qy1 = wy;
      }
    },
    onPointerMove: (wx, wy) => {
      if (queryDrag) {
        qx1 = wx;
        qy1 = wy;
      }
      if (rayDrag) {
        rex = wx;
        rey = wy;
      }
    },
    onPointerUp: () => {
      if (queryDrag) tree.query_aabb(qx0, qy0, qx1, qy1);
      if (rayDrag) tree.ray_cast(rsx, rsy, rex, rey);
      queryDrag = false;
      rayDrag = false;
    },
    paintOverlay: (ctx, camera, canvas) => {
      const boxes = tree.leaf_boxes();
      const flags = tree.highlight_flags();
      for (let i = 0; i < boxes.length; i += 4) {
        const qi = (i / 4) * 2;
        const queried = flags[qi] === 1;
        const rayed = flags[qi + 1] === 1;
        ctx.strokeStyle = rayed ? "#ff4500" : queried ? "#00ff7f" : "#555555";
        ctx.lineWidth = rayed || queried ? 1.5 : 0.5;
        const a = worldToScreen(camera, canvas, boxes[i]!, boxes[i + 1]!);
        const b = worldToScreen(camera, canvas, boxes[i + 2]!, boxes[i + 3]!);
        ctx.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y);
      }
      if (queryDrag) {
        drawSeg(ctx, camera, canvas, qx0, qy0, qx1, qy0, "#00ff7f");
        drawSeg(ctx, camera, canvas, qx1, qy0, qx1, qy1, "#00ff7f");
        drawSeg(ctx, camera, canvas, qx1, qy1, qx0, qy1, "#00ff7f");
        drawSeg(ctx, camera, canvas, qx0, qy1, qx0, qy0, "#00ff7f");
      }
      if (rayDrag) drawSeg(ctx, camera, canvas, rsx, rsy, rex, rey, "#ffffff");
    },
    readoutExtra: () => [
      { label: "proxies", value: String(tree.proxy_count()) },
      { label: "height", value: String(tree.tree_height()) },
      { label: "area ratio", value: tree.area_ratio().toFixed(3) },
    ],
    dispose: () => tree.free(),
  };
}

// ---------------------------------------------------------------------------
// Ray Cast — sample_collision.cpp:869-1188
// ---------------------------------------------------------------------------

function buildRayCast(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  let ox = 0;
  let oy = 0;
  let angle = 0;
  let rayStart = { x: 0.0, y: 30.0 };
  let rayEnd = { x: 0.0, y: 0.0 };
  let rayDrag = false;
  let translating = false;
  let rotating = false;
  let start = { x: 0, y: 0 };
  let base = { x: 0, y: 0, a: 0 };
  let showFraction = false;
  let hits: Float32Array = new Float32Array(30);

  controls.appendChild(
    createSlider("x offset", -2, 2, ox, 0.01, (v) => {
      ox = v;
    }),
  );
  controls.appendChild(
    createSlider("y offset", -2, 2, oy, 0.01, (v) => {
      oy = v;
    }),
  );
  controls.appendChild(
    createSlider("angle", -PI, PI, angle, 0.01, (v) => {
      angle = v;
    }),
  );
  controls.appendChild(
    createCheckbox("show fraction", showFraction, (v) => {
      showFraction = v;
    }),
  );
  controls.appendChild(createButton("Reset", () => {
    ox = oy = angle = 0;
  }));
  controls.appendChild(
    createInfoBox("Click drag: ray · Shift: translate shapes · Ctrl: rotate"),
  );

  return {
    skipSim: true,
    onPointerDown: (wx, wy, mods) => {
      start = { x: wx, y: wy };
      if (mods.shift) {
        translating = true;
        base = { x: ox, y: oy, a: angle };
      } else if (mods.ctrl) {
        rotating = true;
        base = { x: ox, y: oy, a: angle };
      } else {
        rayDrag = true;
        rayStart = { x: wx, y: wy };
        rayEnd = { x: wx, y: wy };
      }
    },
    onPointerMove: (wx, wy) => {
      const dx = wx - start.x;
      const dy = wy - start.y;
      if (rayDrag) rayEnd = { x: wx, y: wy };
      else if (translating) {
        ox = base.x + 0.5 * dx;
        oy = base.y + 0.5 * dy;
      } else if (rotating) {
        angle = clamp(base.a + 0.5 * dx, -PI, PI);
      }
    },
    onPointerUp: () => {
      rayDrag = translating = rotating = false;
    },
    paintOverlay: (ctx, camera, canvas) => {
      hits = wasm.collision_ray_cast_shapes(ox, oy, angle, rayStart.x, rayStart.y, rayEnd.x, rayEnd.y);
      // Draw shapes at C offsets
      const offsets = [
        [-20, 20],
        [-10, 20],
        [0, 20],
        [10, 20],
        [20, 20],
      ];
      for (const [i, off] of offsets.entries()) {
        const cx = ox + off[0]!;
        const cy = oy + off[1]!;
        drawDot(ctx, camera, canvas, cx, cy, "#ffff00", 3);
        const h = hits.subarray(i * 6, i * 6 + 6);
        if (h[0]! > 0.5) {
          drawSeg(ctx, camera, canvas, rayStart.x, rayStart.y, h[2]!, h[3]!, "#ffffff");
          drawDot(ctx, camera, canvas, h[2]!, h[3]!, "#ffffff", 5);
          drawSeg(ctx, camera, canvas, h[2]!, h[3]!, h[2]! + h[4]!, h[3]! + h[5]!, "#9400d3");
          if (showFraction) {
            const p = worldToScreen(camera, canvas, h[2]! + 0.05, h[3]! - 0.02);
            ctx.fillStyle = "#fff";
            ctx.font = "12px sans-serif";
            ctx.fillText(h[1]!.toFixed(2), p.x, p.y);
          }
        } else {
          drawSeg(ctx, camera, canvas, rayStart.x, rayStart.y, rayEnd.x, rayEnd.y, "#ffffff");
        }
      }
      drawDot(ctx, camera, canvas, rayStart.x, rayStart.y, "#00ff00", 5);
      drawDot(ctx, camera, canvas, rayEnd.x, rayEnd.y, "#ff0000", 5);
    },
  };
}

// ---------------------------------------------------------------------------
// Cast World — sample_collision.cpp:1359-1856
// ---------------------------------------------------------------------------

function buildCastWorld(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  const MAX = 64;
  const IGNORE = 7;
  const bodies: number[] = [];
  const shapeIdx: number[] = [];
  let bodyIndex = 0;
  let mode = 1; // closest
  let castType = 0; // ray
  let castRadius = 0.5;
  let simple = false;
  let rayStart = { x: -20.0, y: 10.0 };
  let rayEnd = { x: 20.0, y: 10.0 };
  let dragging = false;
  let rotating = false;
  let angle = 0;
  let baseAngle = 0;
  let angleAnchor = { x: 0, y: 0 };
  const rng = makeRng();

  // Ground :1392-1399
  {
    const g = sim.add_body(0, 0, 0, BODY_STATIC);
    sim.attach_segment_mat(g, -40, 0, 40, 0, 0.6);
  }

  function create(index: number) {
    if (bodies[bodyIndex] != null && sim.is_body_alive(bodies[bodyIndex]!)) {
      sim.destroy_body(bodies[bodyIndex]!);
    }
    const x = rng.floatRange(-20, 20);
    const y = rng.floatRange(0, 20);
    const a = rng.floatRange(-PI, PI);
    const mod = bodyIndex % 3;
    const type = mod === 0 ? BODY_STATIC : mod === 1 ? BODY_KINEMATIC : BODY_DYNAMIC;
    const b =
      type === BODY_DYNAMIC
        ? sim.add_body_ex(x, y, a, type, 0.0, true)
        : sim.add_body(x, y, a, type);
    bodies[bodyIndex] = b;
    let sid = -1;
    if (index === 0) {
      const pts =
        (bodyIndex & 1) === 0
          ? [-0.1, 0.0, 0.1, 0.0, 0.0, 1.5]
          : (() => {
              const w = 1.0;
              const bb = w / (2.0 + Math.SQRT2);
              const s = Math.SQRT2 * bb;
              return [
                0.5 * s, 0, 0.5 * w, bb, 0.5 * w, bb + s, 0.5 * s, w, -0.5 * s, w, -0.5 * w,
                bb + s, -0.5 * w, bb, -0.5 * s, 0,
              ];
            })();
      sid = sim.attach_polygon_mat(b, pts, index === 0 && (bodyIndex & 1) === 0 ? 0.5 : 0, 1, 0.6, 0, 0, 0);
    } else if (index === 1) sid = sim.attach_box_mat(b, 0.5, 0.5, 0, 0, 0, 1, 0.6, 0, 0, 0);
    else if (index === 2) sid = sim.attach_circle_mat(b, 0, 0, 0.5, 1, 0.6, 0, 0, 0);
    else if (index === 3) sid = sim.attach_capsule_mat(b, -0.5, 0, 0.5, 0, 0.25, 1, 0.6, 0, 0, 0);
    else if (index === 4) sid = sim.attach_segment_mat(b, -1, 0, 1, 0, 0.6);
    else {
      sim.attach_chain(b, [1, 0, -1, 0, -1, -1, 1, -1], true);
      sid = -1;
    }
    shapeIdx[bodyIndex] = sid;
    if (sid >= 0 && bodyIndex === IGNORE) sim.shape_set_user_data(sid, 1);
    bodyIndex = (bodyIndex + 1) % MAX;
  }

  function createN(index: number, count: number) {
    for (let i = 0; i < count; i++) create(index);
  }

  controls.appendChild(
    createCheckbox("Simple", simple, (v) => {
      simple = v;
    }),
  );
  controls.appendChild(
    createDropdown(
      "Mode",
      [
        { value: "0", text: "Any" },
        { value: "1", text: "Closest" },
        { value: "2", text: "Multiple" },
        { value: "3", text: "Sorted" },
      ],
      "1",
      (v) => {
        mode = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createDropdown(
      "Cast",
      [
        { value: "0", text: "Ray" },
        { value: "1", text: "Circle" },
        { value: "2", text: "Capsule" },
        { value: "3", text: "Polygon" },
      ],
      "0",
      (v) => {
        castType = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createSlider("cast radius", 0, 2, castRadius, 0.05, (v) => {
      castRadius = v;
    }),
  );
  for (const [label, idx] of [
    ["1× polygon", 0],
    ["1× box", 1],
    ["1× circle", 2],
    ["1× capsule", 3],
    ["1× segment", 4],
    ["1× chain", 5],
  ] as const) {
    controls.appendChild(createButton(label, () => create(idx)));
  }
  controls.appendChild(createButton("10× boxes", () => createN(1, 10)));
  controls.appendChild(
    createButton("Destroy body", () => {
      for (let i = 0; i < MAX; i++) {
        if (bodies[i] != null && sim.is_body_alive(bodies[i]!)) {
          sim.destroy_body(bodies[i]!);
          bodies[i] = -1;
          return;
        }
      }
    }),
  );

  let hitBuf: Float32Array = new Float32Array([0]);

  return {
    onPointerDown: (wx, wy, mods) => {
      if (mods.shift) {
        rotating = true;
        angleAnchor = { x: wx, y: wy };
        baseAngle = angle;
      } else {
        dragging = true;
        rayStart = { x: wx, y: wy };
        rayEnd = { x: wx, y: wy };
      }
    },
    onPointerMove: (wx, wy) => {
      if (dragging) rayEnd = { x: wx, y: wy };
      if (rotating) angle = baseAngle + 1.0 * (wx - angleAnchor.x);
    },
    onPointerUp: () => {
      dragging = rotating = false;
    },
    afterStep: () => {
      const tx = rayEnd.x - rayStart.x;
      const ty = rayEnd.y - rayStart.y;
      if (simple && castType === 0) {
        const r = sim.cast_ray_closest(rayStart.x, rayStart.y, tx, ty);
        // Events layout: [hit,x,y,nx,ny,fraction]
        hitBuf =
          r[0]! > 0.5
            ? new Float32Array([1, r[5]!, r[1]!, r[2]!, r[3]!, r[4]!, -1])
            : new Float32Array([0]);
      } else if (castType === 0) {
        hitBuf = sim.cast_ray_hits(rayStart.x, rayStart.y, tx, ty, mode);
      } else {
        let pts: number[];
        let radius = castRadius;
        if (castType === 1) pts = [0, 0];
        else if (castType === 2) pts = [-0.5, 0, 0.5, 0];
        else pts = boxVerts(0.5, 0.5);
        // Rotate proxy by angle around cast start (C rotates cast shape)
        const c = Math.cos(angle);
        const s = Math.sin(angle);
        const rot: number[] = [];
        for (let i = 0; i < pts.length; i += 2) {
          rot.push(c * pts[i]! - s * pts[i + 1]!);
          rot.push(s * pts[i]! + c * pts[i + 1]!);
        }
        hitBuf = sim.cast_shape_hits(rayStart.x, rayStart.y, rot, radius, tx, ty, mode);
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      drawSeg(ctx, camera, canvas, rayStart.x, rayStart.y, rayEnd.x, rayEnd.y, "#ffffff");
      drawDot(ctx, camera, canvas, rayStart.x, rayStart.y, "#00ff00", 5);
      const count = hitBuf[0] | 0;
      for (let i = 0; i < count; i++) {
        const o = 1 + i * 6;
        const px = hitBuf[o + 1]!;
        const py = hitBuf[o + 2]!;
        const nx = hitBuf[o + 3]!;
        const ny = hitBuf[o + 4]!;
        drawDot(ctx, camera, canvas, px, py, "#ffffff", 5);
        drawSeg(ctx, camera, canvas, px, py, px + nx, py + ny, "#9400d3");
      }
    },
    readoutExtra: () => [{ label: "hits", value: String(hitBuf[0] | 0) }],
  };
}

// ---------------------------------------------------------------------------
// Overlap World — sample_collision.cpp:1858-2218
// ---------------------------------------------------------------------------

function buildOverlapWorld(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  const MAX = 64;
  const IGNORE = 7;
  const bodies: number[] = new Array(MAX).fill(-1);
  const shapes: number[] = new Array(MAX).fill(-1);
  let bodyIndex = 0;
  let shapeType = 0;
  let position = { x: 0.0, y: 10.0 };
  let angle = 0.0;
  let dragging = false;
  let rotating = false;
  let start = { x: 0, y: 0 };
  let baseAngle = 0;
  const rng = makeRng();

  function create(index: number) {
    if (bodies[bodyIndex]! >= 0 && sim.is_body_alive(bodies[bodyIndex]!)) {
      sim.destroy_body(bodies[bodyIndex]!);
    }
    const x = rng.floatRange(-20, 20);
    const y = rng.floatRange(0, 20);
    const a = rng.floatRange(-PI, PI);
    const b = sim.add_body(x, y, a, BODY_STATIC);
    bodies[bodyIndex] = b;
    let sid = -1;
    if (index < 4) {
      const polys: number[][] = [
        [-0.5, 0, 0.5, 0, 0, 1.5],
        [-0.1, 0, 0.1, 0, 0, 1.5],
        (() => {
          const w = 1.0;
          const bb = w / (2.0 + Math.SQRT2);
          const s = Math.SQRT2 * bb;
          return [
            0.5 * s, 0, 0.5 * w, bb, 0.5 * w, bb + s, 0.5 * s, w, -0.5 * s, w, -0.5 * w, bb + s,
            -0.5 * w, bb, -0.5 * s, 0,
          ];
        })(),
        boxVerts(0.5, 0.5),
      ];
      sid = sim.attach_polygon_mat(b, polys[index]!, 0, 1, 0.6, 0, 0, 0);
    } else if (index === 4) sid = sim.attach_circle_mat(b, 0, 0, 0.5, 1, 0.6, 0, 0, 0);
    else if (index === 5) sid = sim.attach_capsule_mat(b, -0.5, 0, 0.5, 0, 0.25, 1, 0.6, 0, 0, 0);
    else sid = sim.attach_segment_mat(b, -1, 0, 1, 0, 0.6);
    shapes[bodyIndex] = sid;
    if (sid >= 0 && bodyIndex === IGNORE) sim.shape_set_user_data(sid, 1);
    bodyIndex = (bodyIndex + 1) % MAX;
  }

  for (let i = 0; i < 10; i++) create(0);

  controls.appendChild(
    createDropdown(
      "overlap shape",
      [
        { value: "0", text: "circle" },
        { value: "1", text: "capsule" },
        { value: "2", text: "box" },
      ],
      "0",
      (v) => {
        shapeType = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(createButton("1× triangle", () => create(0)));
  controls.appendChild(createButton("10× boxes", () => {
    for (let i = 0; i < 10; i++) create(3);
  }));
  controls.appendChild(createButton("1× circle", () => create(4)));
  controls.appendChild(
    createInfoBox("Drag probe · Shift+drag rotate · overlapping shapes are destroyed (C doom)"),
  );

  return {
    onPointerDown: (wx, wy, mods) => {
      start = { x: wx, y: wy };
      if (mods.shift) {
        rotating = true;
        baseAngle = angle;
      } else {
        dragging = true;
        position = { x: wx, y: wy };
      }
    },
    onPointerMove: (wx, wy) => {
      if (dragging) position = { x: wx, y: wy };
      if (rotating) angle = baseAngle + 1.0 * (wx - start.x);
    },
    onPointerUp: () => {
      dragging = rotating = false;
    },
    afterStep: () => {
      let pts: number[];
      let radius = 0.5;
      if (shapeType === 0) pts = [0, 0];
      else if (shapeType === 1) {
        pts = [-0.5, 0, 0.5, 0];
        radius = 0.25;
      } else pts = boxVerts(0.5, 0.5);
      const c = Math.cos(angle);
      const s = Math.sin(angle);
      const rot: number[] = [];
      for (let i = 0; i < pts.length; i += 2) {
        rot.push(c * pts[i]! - s * pts[i + 1]!);
        rot.push(s * pts[i]! + c * pts[i + 1]!);
      }
      const doomed = sim.overlap_shape_hits(position.x, position.y, rot, radius);
      // Destroy bodies owning doomed shapes (C destroys shape's body)
      for (const si of doomed) {
        for (let i = 0; i < MAX; i++) {
          if (shapes[i] === si && bodies[i]! >= 0 && sim.is_body_alive(bodies[i]!)) {
            sim.destroy_body(bodies[i]!);
            bodies[i] = -1;
            shapes[i] = -1;
            break;
          }
        }
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      drawDot(ctx, camera, canvas, position.x, position.y, "#00ffff", 6);
      if (shapeType === 0) {
        // circle outline approx via points
        for (let i = 0; i < 16; i++) {
          const a0 = (i / 16) * 2 * PI;
          const a1 = ((i + 1) / 16) * 2 * PI;
          drawSeg(
            ctx,
            camera,
            canvas,
            position.x + 0.5 * Math.cos(a0),
            position.y + 0.5 * Math.sin(a0),
            position.x + 0.5 * Math.cos(a1),
            position.y + 0.5 * Math.sin(a1),
            "#00ffff",
          );
        }
      }
    },
  };
}

// ---------------------------------------------------------------------------
// Manifold — sample_collision.cpp:2221-2880
// ---------------------------------------------------------------------------

function buildManifold(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  let tx = 0.17;
  let ty = 1.12;
  let angle = 0;
  let round = 0.1;
  let dragging = false;
  let rotating = false;
  let start = { x: 0, y: 0 };
  let base = { x: 0, y: 0, a: 0 };
  let showCount = false;
  let showIds = false;
  let showSeparation = false;

  controls.appendChild(
    createSlider("x offset", -2, 2, tx, 0.01, (v) => {
      tx = v;
    }),
  );
  controls.appendChild(
    createSlider("y offset", -2, 2, ty, 0.01, (v) => {
      ty = v;
    }),
  );
  controls.appendChild(
    createSlider("angle", -PI, PI, angle, 0.01, (v) => {
      angle = v;
    }),
  );
  controls.appendChild(
    createSlider("round", 0, 0.4, round, 0.05, (v) => {
      round = v;
    }),
  );
  controls.appendChild(createCheckbox("show count", showCount, (v) => (showCount = v)));
  controls.appendChild(createCheckbox("show ids", showIds, (v) => (showIds = v)));
  controls.appendChild(
    createCheckbox("show separation", showSeparation, (v) => (showSeparation = v)),
  );
  controls.appendChild(createButton("Reset", () => {
    tx = ty = angle = 0;
  }));

  return {
    skipSim: true,
    onPointerDown: (wx, wy, mods) => {
      start = { x: wx, y: wy };
      if (mods.shift) {
        rotating = true;
        base = { x: tx, y: ty, a: angle };
      } else {
        dragging = true;
        base = { x: tx, y: ty, a: angle };
      }
    },
    onPointerMove: (wx, wy) => {
      const dx = wx - start.x;
      const dy = wy - start.y;
      if (dragging) {
        tx = base.x + 0.5 * dx;
        ty = base.y + 0.5 * dy;
      } else if (rotating) {
        angle = clamp(base.a + 1.0 * dx, -PI, PI);
      }
    },
    onPointerUp: () => {
      dragging = rotating = false;
    },
    paintOverlay: (ctx, camera, canvas) => {
      // sample_collision.cpp:2395-2396 grid
      let offsetX = -10.0;
      const offsetY = -5.0;
      const increment = 4.0;
      for (let kind = 0; kind < 10; kind++) {
        const m = wasm.collision_manifold_pair(kind, tx, ty, angle, round);
        const ox = offsetX;
        const oy = offsetY;
        drawDot(ctx, camera, canvas, ox, oy, "#7fffd4", 3);
        drawDot(ctx, camera, canvas, ox + tx, oy + ty, "#eee8aa", 3);
        const nx = m[0]!;
        const ny = m[1]!;
        const pc = m[2]! | 0;
        if (showCount) {
          const p = worldToScreen(camera, canvas, ox + tx * 0.5, oy + ty * 0.5);
          ctx.fillStyle = "#fff";
          ctx.font = "12px sans-serif";
          ctx.fillText(String(pc), p.x, p.y);
        }
        for (let i = 0; i < pc; i++) {
          const px = ox + m[3 + i * 4]!;
          const py = oy + m[4 + i * 4]!;
          const sep = m[5 + i * 4]!;
          const id = m[6 + i * 4]! | 0;
          drawDot(ctx, camera, canvas, px, py, "#0000ff", 5);
          drawSeg(ctx, camera, canvas, px, py, px + 0.5 * nx, py + 0.5 * ny, "#ee82ee");
          if (showIds || showSeparation) {
            const p = worldToScreen(camera, canvas, px + 0.05, py + 0.03);
            ctx.fillStyle = "#fff";
            ctx.font = "10px sans-serif";
            ctx.fillText(
              `${showIds ? "0x" + (id & 0xffff).toString(16) : ""} ${showSeparation ? sep.toFixed(3) : ""}`,
              p.x,
              p.y,
            );
          }
        }
        offsetX += increment;
      }
    },
  };
}

// ---------------------------------------------------------------------------
// Smooth Manifold — sample_collision.cpp:2882-3171
// ---------------------------------------------------------------------------

function buildSmoothManifold(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  let shapeType = 1; // box
  let tx = 0.0;
  let ty = 20.0;
  let angle = 0.0;
  let round = 0.0;
  let dragging = false;
  let rotating = false;
  let start = { x: 0, y: 0 };
  let base = { x: 0, y: 0, a: 0 };
  const count = SMOOTH_POINTS.length / 2;

  controls.appendChild(
    createDropdown(
      "shape",
      [
        { value: "0", text: "circle" },
        { value: "1", text: "box" },
      ],
      "1",
      (v) => {
        shapeType = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createSlider("round", 0, 0.4, round, 0.05, (v) => {
      round = v;
    }),
  );

  return {
    skipSim: true,
    onPointerDown: (wx, wy, mods) => {
      start = { x: wx, y: wy };
      if (mods.shift) {
        rotating = true;
        base = { x: tx, y: ty, a: angle };
      } else {
        dragging = true;
        base = { x: tx, y: ty, a: angle };
      }
    },
    onPointerMove: (wx, wy) => {
      const dx = wx - start.x;
      const dy = wy - start.y;
      if (dragging) {
        tx = base.x + dx;
        ty = base.y + dy;
      } else if (rotating) {
        angle = clamp(base.a + dx, -PI, PI);
      }
    },
    onPointerUp: () => {
      dragging = rotating = false;
    },
    paintOverlay: (ctx, camera, canvas) => {
      for (let i = 0; i < count; i++) {
        const i0 = i > 0 ? i - 1 : count - 1;
        const i1 = i;
        const i2 = i1 < count - 1 ? i1 + 1 : 0;
        const i3 = i2 < count - 1 ? i2 + 1 : 0;
        const g1x = SMOOTH_POINTS[i0 * 2]!;
        const g1y = SMOOTH_POINTS[i0 * 2 + 1]!;
        const p1x = SMOOTH_POINTS[i1 * 2]!;
        const p1y = SMOOTH_POINTS[i1 * 2 + 1]!;
        const p2x = SMOOTH_POINTS[i2 * 2]!;
        const p2y = SMOOTH_POINTS[i2 * 2 + 1]!;
        const g2x = SMOOTH_POINTS[i3 * 2]!;
        const g2y = SMOOTH_POINTS[i3 * 2 + 1]!;
        drawSeg(ctx, camera, canvas, p1x, p1y, p2x, p2y, "#7fffd4");
        const m = wasm.collision_smooth_manifold(
          shapeType,
          tx,
          ty,
          angle,
          round,
          g1x,
          g1y,
          p1x,
          p1y,
          p2x,
          p2y,
          g2x,
          g2y,
        );
        const pc = m[2]! | 0;
        for (let k = 0; k < pc; k++) {
          const px = m[3 + k * 4]!;
          const py = m[4 + k * 4]!;
          drawDot(ctx, camera, canvas, px, py, "#0000ff", 4);
          drawSeg(ctx, camera, canvas, px, py, px + 0.5 * m[0]!, py + 0.5 * m[1]!, "#ee82ee");
        }
      }
      drawDot(ctx, camera, canvas, tx, ty, "#eee8aa", 6);
    },
  };
}

// ---------------------------------------------------------------------------
// Shape Cast — sample_collision.cpp:3173-3570
// ---------------------------------------------------------------------------

function buildShapeCast(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  let typeA = 3; // box
  let typeB = 0; // point
  let radiusA = 0.0;
  let radiusB = 0.2;
  let tx = -0.6;
  let ty = 0.0;
  let angle = 0.0;
  let translation = { x: 2.0, y: 0.0 };
  let dragging = false;
  let sweeping = false;
  let rotating = false;
  let start = { x: 0, y: 0 };
  let base = { x: 0, y: 0, a: 0 };
  let encroach = false;
  let hit: Float32Array = new Float32Array(7);

  const box = boxVerts(8.984375, 0.5); // :3227
  const segment = [0, 0, 0.5, 0];
  const triangle = [-0.5, 0, 0.5, 0, 0, 1];
  const point = [0, 0];

  function proxy(t: number): number[] {
    if (t === 0) return point;
    if (t === 1) return segment;
    if (t === 2) return triangle;
    return box;
  }

  controls.appendChild(
    createDropdown(
      "shape A",
      [
        { value: "0", text: "point" },
        { value: "1", text: "segment" },
        { value: "2", text: "triangle" },
        { value: "3", text: "box" },
      ],
      "3",
      (v) => {
        typeA = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createDropdown(
      "shape B",
      [
        { value: "0", text: "point" },
        { value: "1", text: "segment" },
        { value: "2", text: "triangle" },
        { value: "3", text: "box" },
      ],
      "0",
      (v) => {
        typeB = parseInt(v, 10);
      },
    ),
  );
  controls.appendChild(
    createSlider("radius A", 0, 0.5, radiusA, 0.01, (v) => {
      radiusA = v;
    }),
  );
  controls.appendChild(
    createSlider("radius B", 0, 0.5, radiusB, 0.01, (v) => {
      radiusB = v;
    }),
  );
  controls.appendChild(createCheckbox("encroach", encroach, (v) => (encroach = v)));
  controls.appendChild(
    createInfoBox("Drag B · Shift rotate · Ctrl sweep translation endpoint"),
  );

  return {
    skipSim: true,
    onPointerDown: (wx, wy, mods) => {
      start = { x: wx, y: wy };
      if (mods.ctrl) {
        sweeping = true;
      } else if (mods.shift) {
        rotating = true;
        base = { x: tx, y: ty, a: angle };
      } else {
        dragging = true;
        base = { x: tx, y: ty, a: angle };
      }
    },
    onPointerMove: (wx, wy) => {
      if (dragging) {
        tx = base.x + (wx - start.x);
        ty = base.y + (wy - start.y);
      } else if (rotating) {
        angle = clamp(base.a + (wx - start.x), -PI, PI);
      } else if (sweeping) {
        translation = { x: wx - tx, y: wy - ty };
      }
    },
    onPointerUp: () => {
      dragging = rotating = sweeping = false;
    },
    paintOverlay: (ctx, camera, canvas) => {
      hit = wasm.collision_shape_cast(
        proxy(typeA),
        radiusA,
        proxy(typeB),
        radiusB,
        tx,
        ty,
        angle,
        translation.x,
        translation.y,
        1.0,
        encroach,
      );
      drawDot(ctx, camera, canvas, 0, 0, "#00ffff", 3);
      drawDot(ctx, camera, canvas, tx, ty, "#ffe4c4", 4);
      drawSeg(ctx, camera, canvas, tx, ty, tx + translation.x, ty + translation.y, "#888888");
      if (hit[0]! > 0.5) {
        drawDot(ctx, camera, canvas, hit[2]!, hit[3]!, "#ffffff", 5);
        drawSeg(
          ctx,
          camera,
          canvas,
          hit[2]!,
          hit[3]!,
          hit[2]! + hit[4]!,
          hit[3]! + hit[5]!,
          "#9400d3",
        );
      }
    },
    readoutExtra: () => [
      { label: "hit", value: hit[0]! > 0.5 ? "yes" : "no" },
      { label: "fraction", value: hit[1]!.toFixed(4) },
      { label: "iterations", value: String(hit[6]! | 0) },
    ],
  };
}

// ---------------------------------------------------------------------------
// Time of Impact — sample_collision.cpp:3572-3659
// ---------------------------------------------------------------------------

function buildTimeOfImpact(wasm: Box2dWasm, controls: HTMLElement): SceneRuntime {
  let toi = 0;
  let dist = 0;
  let state = 0;
  const vertsA = [-16.25, 44.75, -15.75, 44.75, -15.75, 45.25, -16.25, 45.25];
  const vertsB = [0.0, -0.125, 0.0, 0.125];
  const radiusB = 0.0299999993;

  controls.appendChild(
    createInfoBox("Hardcoded sweeps from C TimeOfImpact sample — b2TimeOfImpact output."),
  );

  return {
    skipSim: true,
    paintOverlay: (ctx, camera, canvas) => {
      const out = wasm.collision_time_of_impact();
      state = out[0]! | 0;
      toi = out[1]!;
      dist = out[2]!;
      // transforms: t0a[3..6], t0b[7..10], thb[11..14], t1b[15..18]
      const drawPoly = (verts: number[], qc: number, qs: number, px: number, py: number, color: string) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i < verts.length; i += 2) {
          const lx = verts[i]!;
          const ly = verts[i + 1]!;
          const wx = px + qc * lx - qs * ly;
          const wy = py + qs * lx + qc * ly;
          const p = worldToScreen(camera, canvas, wx, wy);
          if (i === 0) ctx.moveTo(p.x, p.y);
          else ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.stroke();
      };
      const drawCap = (verts: number[], qc: number, qs: number, px: number, py: number, color: string) => {
        const x0 = px + qc * verts[0]! - qs * verts[1]!;
        const y0 = py + qs * verts[0]! + qc * verts[1]!;
        const x1 = px + qc * verts[2]! - qs * verts[3]!;
        const y1 = py + qs * verts[2]! + qc * verts[3]!;
        drawSeg(ctx, camera, canvas, x0, y0, x1, y1, color, 2);
        drawDot(ctx, camera, canvas, x0, y0, color, 3);
        drawDot(ctx, camera, canvas, x1, y1, color, 3);
      };
      drawPoly(vertsA, out[3]!, out[4]!, out[5]!, out[6]!, "#808080");
      drawCap(vertsB, out[7]!, out[8]!, out[9]!, out[10]!, "#00ff00");
      drawPoly(vertsB, out[11]!, out[12]!, out[13]!, out[14]!, "#ffa500");
      drawCap(vertsB, out[15]!, out[16]!, out[17]!, out[18]!, "#ff0000");
      void radiusB;
    },
    readoutExtra: () => [
      { label: "toi", value: toi.toFixed(6) },
      { label: "state", value: String(state) },
      { label: "distance", value: dist.toFixed(6) },
    ],
  };
}

// ---------------------------------------------------------------------------
// Dispatch + page
// ---------------------------------------------------------------------------

function buildScene(
  scene: Scene,
  sim: SimWorld,
  wasm: Box2dWasm,
  controls: HTMLElement,
): SceneRuntime {
  controls.replaceChildren();
  switch (scene) {
    case "shape-distance":
      return buildShapeDistance(wasm, controls);
    case "dynamic-tree":
      return buildDynamicTree(wasm, controls);
    case "ray-cast":
      return buildRayCast(wasm, controls);
    case "cast-world":
      return buildCastWorld(sim, controls);
    case "overlap-world":
      return buildOverlapWorld(sim, controls);
    case "manifold":
      return buildManifold(wasm, controls);
    case "smooth-manifold":
      return buildSmoothManifold(wasm, controls);
    case "shape-cast":
      return buildShapeCast(wasm, controls);
    case "time-of-impact":
      return buildTimeOfImpact(wasm, controls);
  }
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Collision",
    "C <code>sample_collision.cpp</code> RegisterSample ports — GJK distance, dynamic tree, " +
      "ray/shape cast, overlap, manifolds, and time of impact. Invented <code>#/manifolds</code> retired.",
    "Drag / Shift / Ctrl per sample · P pause · O step · R restart",
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "shape-distance";

  const camera: SampleCamera = makeCamera();
  applyCamera(camera, scene);
  const transport = createSampleTransport();
  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = {};

  const sceneControls = document.createElement("div");
  sceneControls.className = "scene-controls";

  function rebuild() {
    runtime.dispose?.();
    freeSim(sim);
    sim = new wasm.SimWorld(0.0);
    applyCamera(camera, scene);
    runtime = buildScene(scene, sim, wasm, sceneControls);
  }

  rebuild();

  let grabbing = false;
  const mods = { shift: false, ctrl: false };
  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    fitCanvas(canvas);
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    mods.shift = e.shiftKey;
    mods.ctrl = e.ctrlKey || e.metaKey;
    if (runtime.onPointerDown) {
      runtime.onPointerDown(w.x, w.y, mods);
      canvas.setPointerCapture(e.pointerId);
      return;
    }
    if (!runtime.skipSim) {
      grabbing = sim.mouse_down(w.x, w.y);
      if (grabbing) canvas.setPointerCapture(e.pointerId);
    }
  };
  const onPointerMove = (e: PointerEvent) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    if (runtime.onPointerMove) {
      runtime.onPointerMove(w.x, w.y);
      return;
    }
    if (grabbing || (!runtime.skipSim && sim.mouse_active())) sim.mouse_move(w.x, w.y);
  };
  const onPointerUp = () => {
    runtime.onPointerUp?.();
    if (grabbing || (!runtime.skipSim && sim.mouse_active())) sim.mouse_up();
    grabbing = false;
  };
  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerUp);

  controls.appendChild(
    createDropdown(
      "Sample",
      SCENES.map((s) => ({ value: s, text: SCENE_LABEL[s] })),
      scene,
      (v) => {
        scene = v as Scene;
        history.replaceState(null, "", `#/collision/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  transport.mountControls(controls, () => rebuild());
  controls.appendChild(createSeparator());
  controls.appendChild(sceneControls);
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const unbindKeys = transport.bindKeys();

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    runtime.beforeStep?.(dt);
    if (!runtime.skipSim) {
      sim.step(dt, transport.subSteps);
    }
    runtime.afterStep?.(dt);

    if (!runtime.skipSim) {
      const b = viewBounds(camera, canvas);
      sim.collect_draw(b.lowerX, b.lowerY, b.upperX, b.upperY);
      paintDebugDraw(canvas, camera, {
        polygons: sim.draw_polygons(),
        circles: sim.draw_circles(),
        capsules: sim.draw_capsules(),
        lines: sim.draw_lines(),
      });
    } else {
      const ctx = canvas.getContext("2d")!;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    }
    const ctx = canvas.getContext("2d");
    if (ctx && runtime.paintOverlay) runtime.paintOverlay(ctx, camera, canvas);

    updateReadout(readout, [
      { label: "Sample", value: SCENE_LABEL[scene] },
      ...(runtime.skipSim
        ? []
        : [
            { label: "Bodies", value: String(sim.body_count()) },
            { label: "Awake", value: String(sim.awake_body_count()) },
          ]),
      { label: "Hz", value: String(transport.hertz) },
      { label: "Paused", value: transport.paused ? "yes" : "no" },
      ...(runtime.readoutExtra?.() ?? []),
    ]);
  });

  return () => {
    stop();
    unbindKeys();
    runtime.dispose?.();
    disposeTransport(transport);
    freeSim(sim);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
  };
}
