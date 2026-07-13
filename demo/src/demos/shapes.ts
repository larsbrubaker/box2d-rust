// Shapes — RegisterSample ports from sample_shapes.cpp.
// C citations use sample_shapes.cpp line numbers at the pinned submodule.

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
import { getWasm, type SimWorld } from "../wasm.ts";
import { paintSampleDraw } from "./debug-draw.ts";
import { demoPage, fitCanvas, freeSim, runSimLoop } from "./sim-common.ts";
import {
  createSampleTransport,
  mountSampleChrome,
  disposeTransport,
  makeCamera,
  screenToWorld,
  worldToScreen,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — must match slugify(C name) / registry.ts. */
export const SCENES = [
  "chain-shape",
  "chain-segment",
  "compound-shapes",
  "filter",
  "custom-filter",
  "restitution",
  "friction",
  "rolling-resistance",
  "conveyor-belt",
  "tangent-speed",
  "modify-geometry",
  "chain-link",
  "rounded",
  "ellipse",
  "offset",
  "explosion",
  "recreate-static",
  "box-restitution",
  "wind",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("shapes", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "chain-shape": "Chain Shape",
  "chain-segment": "Chain Segment",
  "compound-shapes": "Compound Shapes",
  filter: "Filter",
  "custom-filter": "Custom Filter",
  restitution: "Restitution",
  friction: "Friction",
  "rolling-resistance": "Rolling Resistance",
  "conveyor-belt": "Conveyor Belt",
  "tangent-speed": "Tangent Speed",
  "modify-geometry": "Modify Geometry",
  "chain-link": "Chain Link",
  rounded: "Rounded",
  ellipse: "Ellipse",
  offset: "Offset",
  explosion: "Explosion",
  "recreate-static": "Recreate Static",
  "box-restitution": "Box Restitution",
  wind: "Wind",
};

/** C camera.center / camera.zoom (half-height). */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "chain-shape": { cx: 0.0, cy: 0.0, zoom: 25.0 * 1.75 }, // :34-35
  "chain-segment": { cx: 0.0, cy: 0.0, zoom: 25.0 * 1.0 }, // :240-241
  "compound-shapes": { cx: 0.0, cy: 6.0, zoom: 25.0 * 0.5 }, // :417-418
  filter: { cx: 0.0, cy: 5.0, zoom: 25.0 * 0.5 }, // :639-640
  "custom-filter": { cx: 0.0, cy: 5.0, zoom: 10.0 }, // :841-842
  restitution: { cx: 4.0, cy: 17.0, zoom: 27.5 }, // :937-938
  friction: { cx: 0.0, cy: 14.0, zoom: 25.0 * 0.6 }, // :1048-1049
  "rolling-resistance": { cx: 5.0, cy: 20.0, zoom: 27.5 }, // :1115-1116
  "conveyor-belt": { cx: 2.0, cy: 7.5, zoom: 12.0 }, // :1206-1207
  "tangent-speed": { cx: 60.0, cy: -15.0, zoom: 38.0 }, // :1265-1266
  "modify-geometry": { cx: 0.0, cy: 5.0, zoom: 25.0 * 0.25 }, // :1402-1403
  "chain-link": { cx: 0.0, cy: 5.0, zoom: 25.0 * 0.5 }, // :1558-1559
  rounded: { cx: 2.0, cy: 8.0, zoom: 25.0 * 0.55 }, // :1638-1639
  ellipse: { cx: 2.0, cy: 8.0, zoom: 25.0 * 0.55 }, // :1708-1709
  offset: { cx: 2.0, cy: 8.0, zoom: 25.0 * 0.55 }, // :1773-1774
  explosion: { cx: 0.0, cy: 0.0, zoom: 14.0 }, // :1832-1833
  "recreate-static": { cx: 0.0, cy: 2.5, zoom: 3.5 }, // :1944-1945
  "box-restitution": { cx: 0.0, cy: 5.0, zoom: 10.0 }, // :2001-2002
  wind: { cx: 0.0, cy: 1.0, zoom: 2.0 }, // :2075-2076
};

const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const FRIC = 0.6;
const ALL_BITS = 0xffffffff;
const GROUND = 0x00000001;
const TEAM1 = 0x00000002;
const TEAM2 = 0x00000004;
const TEAM3 = 0x00000008;

// sample_shapes.cpp:74-80 Chain Shape loop points
const CHAIN_SHAPE_POINTS = [
  -56.885498, 12.8985004, -56.885498, 16.2057495, 56.885498, 16.2057495, 56.885498, -16.2057514,
  51.5935059, -16.2057514, 43.6559982, -10.9139996, 35.7184982, -10.9139996, 27.7809982, -10.9139996,
  21.1664963, -14.2212505, 11.9059982, -16.2057514, 0, -16.2057514, -10.5835037, -14.8827496,
  -17.1980019, -13.5597477, -21.1665001, -12.2370014, -25.1355019, -9.5909977, -31.75, -3.63799858,
  -38.3644981, 6.2840004, -42.3334999, 9.59125137, -47.625, 11.5755005, -56.885498, 12.8985004,
];

// Tangent Speed ParsePath output (sample.cpp:1148 Y-flip, capacity 20)
const TANGENT_PATH = [
  113.29168, -37.091666, 104.825004, -37.091666, 97.4166716, -37.091666, 90.5375096, -37.091666,
  84.7166756, -36.5625, 79.4250076, -34.975, 74.1333416, -32.329166, 69.3708396, -28.095834,
  66.7250076, -28.095834, 66.7250076, -37.091666, 4.283342, -37.091666, 4.283342, -0.05, 0.05,
  -0.05, 0.05, -41.325, 113.29168, -41.324998,
];

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  paintOverlay?: (ctx: CanvasRenderingContext2D, camera: SampleCamera, canvas: HTMLCanvasElement) => void;
  readoutExtra?: () => { label: string; value: string }[];
  dispose?: () => void;
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

function posOf(sim: SimWorld, index: number) {
  const p = sim.positions();
  return { x: p[index * 3]!, y: p[index * 3 + 1]!, angle: p[index * 3 + 2]! };
}

/** C XorShift RandomInt (utils.h) seeded at RAND_SEED 12345. */
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

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

function buildChainShape(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_shapes.cpp:19-221 — Exact: chain_SetSurfaceMaterial on friction slider.
  let shapeType: "circle" | "capsule" | "box" = "circle";
  let friction = 0.2;
  let restitution = 0.0;
  let bodyId = -1;
  let shapeId = -1;
  let chainId = -1;

  function createScene() {
    const mats = [friction, restitution, 0, 0];
    chainId = sim.add_chain_mat(CHAIN_SHAPE_POINTS, true, mats);
  }

  function launch() {
    if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.destroy_body(bodyId);
    bodyId = sim.add_body(-55.0, 13.5, 0.0, BODY_DYNAMIC);
    if (shapeType === "circle") {
      shapeId = sim.attach_circle_mat(bodyId, 0, 0, 0.5, 1.0, friction, restitution, 0, 0);
    } else if (shapeType === "capsule") {
      shapeId = sim.attach_capsule_mat(bodyId, -0.5, 0, 0.5, 0, 0.25, 1.0, friction, restitution, 0, 0);
    } else {
      shapeId = sim.attach_box_mat(bodyId, 0.5, 0.5, 0, 0, 0, 1.0, friction, restitution, 0, 0);
    }
  }

  createScene();
  launch();

  controls.appendChild(
    createDropdown(
      "Shape",
      [
        { value: "circle", text: "Circle" },
        { value: "capsule", text: "Capsule" },
        { value: "box", text: "Box" },
      ],
      shapeType,
      (v) => {
        shapeType = v as typeof shapeType;
        launch();
      },
    ),
  );
  controls.appendChild(
    createSlider("Friction", 0, 1, friction, 0.01, (v) => {
      friction = v;
      if (shapeId >= 0) sim.shape_set_surface(shapeId, friction, restitution, 0, 0);
      if (chainId >= 0) sim.chain_set_surface(chainId, friction, restitution, 0, 0, 0);
    }),
  );
  controls.appendChild(
    createSlider("Restitution", 0, 2, restitution, 0.1, (v) => {
      restitution = v;
      if (shapeId >= 0) sim.shape_set_surface(shapeId, friction, restitution, 0, 0);
    }),
  );
  controls.appendChild(createButton("Launch", () => launch()));

  return {
    paintOverlay: (ctx, camera, canvas) => {
      // C DrawLine axes at origin (:204-205)
      const o = worldToScreen(camera, canvas, 0, 0);
      const x = worldToScreen(camera, canvas, 0.5, 0);
      const y = worldToScreen(camera, canvas, 0, 0.5);
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#ff0000";
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(x.x, x.y);
      ctx.stroke();
      ctx.strokeStyle = "#00ff00";
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(y.x, y.y);
      ctx.stroke();
    },
  };
}

function buildChainSegment(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_shapes.cpp:225-402
  const SEGMENT_COUNT = 32;
  const POINT_COUNT = SEGMENT_COUNT + 3;
  const points: { x: number; y: number }[] = [];
  const segmentShapes: number[] = [];
  let shapeType: "circle" | "capsule" | "box" = "circle";
  let bodyId = -1;
  let mutateIndex = 0;

  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  for (let i = 0; i < POINT_COUNT; ++i) {
    const x = 25.0 - (50.0 * i) / (POINT_COUNT - 1);
    const y = 1.5 * Math.sin(0.18 * x);
    points.push({ x, y });
  }
  for (let i = 0; i < SEGMENT_COUNT; ++i) {
    const g1 = points[i]!;
    const p1 = points[i + 1]!;
    const p2 = points[i + 2]!;
    const g2 = points[i + 3]!;
    segmentShapes.push(
      sim.attach_chain_segment(ground, g1.x, g1.y, p1.x, p1.y, p2.x, p2.y, g2.x, g2.y),
    );
  }

  function launch() {
    if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.destroy_body(bodyId);
    bodyId = sim.add_body(-18.0, 5.0, 0.0, BODY_DYNAMIC);
    if (shapeType === "circle") sim.attach_circle(bodyId, 0, 0, 0.25, 1.0, FRIC, 0);
    else if (shapeType === "capsule") sim.attach_capsule(bodyId, -0.5, 0, 0.5, 0, 0.25, 1.0, FRIC, 0);
    else sim.attach_box(bodyId, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, 0);
  }

  function mutate() {
    let index = mutateIndex + 1;
    mutateIndex += 1;
    if (mutateIndex === SEGMENT_COUNT) mutateIndex = 0;
    points[index]!.y += 0.25;

    const setAt = (seg: number, base: number) => {
      const g1 = points[base]!;
      const p1 = points[base + 1]!;
      const p2 = points[base + 2]!;
      const g2 = points[base + 3]!;
      sim.shape_set_chain_segment(segmentShapes[seg]!, g1.x, g1.y, p1.x, p1.y, p2.x, p2.y, g2.x, g2.y);
    };
    setAt(index - 1, index - 1);
    if (index - 1 > 0) setAt(index - 2, index - 2);
    if (index + 1 < POINT_COUNT - 2) setAt(index, index);
  }

  launch();
  controls.appendChild(
    createDropdown(
      "Shape",
      [
        { value: "circle", text: "Circle" },
        { value: "capsule", text: "Capsule" },
        { value: "box", text: "Box" },
      ],
      shapeType,
      (v) => {
        shapeType = v as typeof shapeType;
        launch();
      },
    ),
  );
  controls.appendChild(createButton("Launch", () => launch()));
  controls.appendChild(createButton("Mutate", () => mutate()));
  return {};
}

function buildCompoundShapes(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_shapes.cpp:409-617
  sim.add_segment(50.0, 0.0, -50.0, 0.0);

  const table1 = sim.add_body(-15.0, 1.0, 0.0, BODY_DYNAMIC);
  sim.attach_box(table1, 3.0, 0.5, 0.0, 3.5, 0.0, 1.0, FRIC, 0);
  sim.attach_box(table1, 0.5, 1.5, -2.5, 1.5, 0.0, 1.0, FRIC, 0);
  sim.attach_box(table1, 0.5, 1.5, 2.5, 1.5, 0.0, 1.0, FRIC, 0);

  const table2 = sim.add_body(-5.0, 1.0, 0.0, BODY_DYNAMIC);
  sim.attach_box(table2, 3.0, 0.5, 0.0, 3.5, 0.0, 1.0, FRIC, 0);
  sim.attach_box(table2, 0.5, 2.0, -2.5, 2.0, 0.0, 1.0, FRIC, 0);
  sim.attach_box(table2, 0.5, 2.0, 2.5, 2.0, 0.0, 1.0, FRIC, 0);

  const ship1 = sim.add_body(5.0, 1.0, 0.0, BODY_DYNAMIC);
  sim.attach_polygon(ship1, [-2.0, 0.0, 0.0, 4.0 / 3.0, 0.0, 4.0], 0.0, 1.0, FRIC, 0);
  sim.attach_polygon(ship1, [2.0, 0.0, 0.0, 4.0 / 3.0, 0.0, 4.0], 0.0, 1.0, FRIC, 0);

  const ship2 = sim.add_body(15.0, 1.0, 0.0, BODY_DYNAMIC);
  sim.attach_polygon(ship2, [-2.0, 0.0, 1.0, 2.0, 0.0, 4.0], 0.0, 1.0, FRIC, 0);
  sim.attach_polygon(ship2, [2.0, 0.0, -1.0, 2.0, 0.0, 4.0], 0.0, 1.0, FRIC, 0);

  let drawAabb = false;
  controls.appendChild(
    createButton("Intrude", () => {
      const t1 = posOf(sim, table1);
      const o1 = sim.add_body(t1.x, t1.y, t1.angle, BODY_DYNAMIC);
      sim.attach_box(o1, 4.0, 0.1, 0.0, 3.0, 0.0, 1.0, FRIC, 0);
      const t2 = posOf(sim, table2);
      const o2 = sim.add_body(t2.x, t2.y, t2.angle, BODY_DYNAMIC);
      sim.attach_box(o2, 4.0, 0.1, 0.0, 3.0, 0.0, 1.0, FRIC, 0);
      const s1 = posOf(sim, ship1);
      const c1 = sim.add_body(s1.x, s1.y, s1.angle, BODY_DYNAMIC);
      sim.attach_circle(c1, 0.0, 2.0, 0.5, 1.0, FRIC, 0);
      const s2 = posOf(sim, ship2);
      const c2 = sim.add_body(s2.x, s2.y, s2.angle, BODY_DYNAMIC);
      sim.attach_circle(c2, 0.0, 2.0, 0.5, 1.0, FRIC, 0);
    }),
  );
  controls.appendChild(
    createCheckbox("Body AABBs", drawAabb, (v) => {
      drawAabb = v;
    }),
  );

  return {
    paintOverlay: (ctx, camera, canvas) => {
      if (!drawAabb) return;
      ctx.strokeStyle = "#ffff00";
      ctx.lineWidth = 1;
      for (const id of [table1, table2, ship1, ship2]) {
        const aabb = sim.body_compute_aabb(id);
        const lo = worldToScreen(camera, canvas, aabb[0]!, aabb[1]!);
        const hi = worldToScreen(camera, canvas, aabb[2]!, aabb[3]!);
        ctx.strokeRect(lo.x, hi.y, hi.x - lo.x, lo.y - hi.y);
      }
    },
  };
}

function buildFilter(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_shapes.cpp:621-823
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment_filter(ground, -20, 0, 20, 0, GROUND, ALL_BITS);

  const p1 = sim.add_body(0, 2, 0, BODY_DYNAMIC);
  const p2 = sim.add_body(0, 5, 0, BODY_DYNAMIC);
  const p3 = sim.add_body(0, 8, 0, BODY_DYNAMIC);
  const s1 = sim.attach_box_filter(p1, 2, 1, 1.0, TEAM1, GROUND | TEAM2 | TEAM3);
  const s2 = sim.attach_box_filter(p2, 2, 1, 1.0, TEAM2, GROUND | TEAM1 | TEAM3);
  const s3 = sim.attach_box_filter(p3, 2, 1, 1.0, TEAM3, GROUND | TEAM1 | TEAM2);

  const toggle = (shape: number, bit: number, on: boolean) => {
    const f = sim.shape_get_filter(shape);
    let mask = f[1]!;
    if (on) mask |= bit;
    else mask &= ~bit;
    sim.shape_set_filter(shape, f[0]!, mask >>> 0);
  };

  controls.appendChild(createInfoBox("Player 1 collides with"));
  controls.appendChild(createCheckbox("Team 2##1", true, (v) => toggle(s1, TEAM2, v)));
  controls.appendChild(createCheckbox("Team 3##1", true, (v) => toggle(s1, TEAM3, v)));
  controls.appendChild(createSeparator());
  controls.appendChild(createInfoBox("Player 2 collides with"));
  controls.appendChild(createCheckbox("Team 1##2", true, (v) => toggle(s2, TEAM1, v)));
  controls.appendChild(createCheckbox("Team 3##2", true, (v) => toggle(s2, TEAM3, v)));
  controls.appendChild(createSeparator());
  controls.appendChild(createInfoBox("Player 3 collides with"));
  controls.appendChild(createCheckbox("Team 1##3", true, (v) => toggle(s3, TEAM1, v)));
  controls.appendChild(createCheckbox("Team 2##3", true, (v) => toggle(s3, TEAM2, v)));

  return {
    paintOverlay: (ctx, camera, canvas) => {
      ctx.fillStyle = "#ffffff";
      ctx.font = "12px sans-serif";
      for (const [id, label] of [
        [p1, "player 1"],
        [p2, "player 2"],
        [p3, "player 3"],
      ] as const) {
        const p = posOf(sim, id);
        const s = worldToScreen(camera, canvas, p.x - 0.5, p.y);
        ctx.fillText(label, s.x, s.y);
      }
    },
  };
}

function buildCustomFilter(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_shapes.cpp:828-918
  sim.enable_odd_even_filter(true);
  sim.add_segment(-40, 0, 40, 0);
  const COUNT = 10;
  const bodies: number[] = [];
  let x = -COUNT;
  for (let i = 0; i < COUNT; ++i) {
    const b = sim.add_body(x, 5.0, 0, BODY_DYNAMIC);
    sim.attach_box_custom(b, 1.0, 1.0, 1.0, i + 1);
    bodies.push(b);
    x += 2.0;
  }
  controls.appendChild(
    createInfoBox("Custom filter disables collision between odd and even shapes"),
  );
  return {
    dispose: () => sim.enable_odd_even_filter(false),
    paintOverlay: (ctx, camera, canvas) => {
      ctx.fillStyle = "#ffffff";
      ctx.font = "12px sans-serif";
      for (let i = 0; i < COUNT; ++i) {
        const p = posOf(sim, bodies[i]!);
        const s = worldToScreen(camera, canvas, p.x, p.y);
        ctx.fillText(String(i), s.x, s.y);
      }
    },
  };
}

function buildRestitutionFixed(
  sim: SimWorld,
  controls: HTMLElement,
  shapeType: "circle" | "box",
  setShape: (t: "circle" | "box") => void,
  rebuild: () => void,
): SceneRuntime {
  const count = 40;
  const h = 1.0 * count;
  sim.add_segment(-h, 0, h, 0);
  let restitution = 0.0;
  const dr = 1.0 / (count - 1);
  let x = -1.0 * (count - 1);
  for (let i = 0; i < count; ++i) {
    const b = sim.add_body(x, 40.0, 0, BODY_DYNAMIC);
    if (shapeType === "circle") sim.attach_circle(b, 0, 0, 0.5, 1.0, FRIC, restitution);
    else sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, restitution);
    restitution += dr;
    x += 2.0;
  }
  controls.appendChild(
    createDropdown(
      "Shape",
      [
        { value: "circle", text: "Circle" },
        { value: "box", text: "Box" },
      ],
      shapeType,
      (v) => {
        setShape(v as "circle" | "box");
        rebuild();
      },
    ),
  );
  controls.appendChild(createButton("Reset", () => rebuild()));
  return {};
}

function buildFrictionScene(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1052-1096
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment_mat(ground, -40, 0, 40, 0, 0.2);
  sim.attach_box_mat(ground, 13, 0.25, -4, 22, -0.25, 0, 0.2, 0, 0, 0);
  sim.attach_box_mat(ground, 0.25, 1, 10.5, 19, 0, 0, 0.2, 0, 0, 0);
  sim.attach_box_mat(ground, 13, 0.25, 4, 14, 0.25, 0, 0.2, 0, 0, 0);
  sim.attach_box_mat(ground, 0.25, 1, -10.5, 11, 0, 0, 0.2, 0, 0, 0);
  sim.attach_box_mat(ground, 13, 0.25, -4, 6, -0.25, 0, 0.2, 0, 0, 0);

  const frictions = [0.75, 0.5, 0.35, 0.1, 0.0];
  for (let i = 0; i < 5; ++i) {
    const b = sim.add_body(-15.0 + 4.0 * i, 28.0, 0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 25.0, frictions[i]!, 0);
  }
  return {};
}

function buildRollingResistance(
  sim: SimWorld,
  controls: HTMLElement,
  lift: number,
  setLift: (v: number) => void,
  rebuild: () => void,
): SceneRuntime {
  // sample_shapes.cpp:1107-1194
  const resistScale = 0.02;
  for (let i = 0; i < 20; ++i) {
    const ground = sim.add_body(0, 0, 0, BODY_STATIC);
    sim.attach_segment(ground, -40, 2.0 * i, 40, 2.0 * i + lift);
    const body = sim.add_body(-39.5, 2.0 * i + 0.75, 0, BODY_DYNAMIC);
    sim.set_angular_velocity(body, -10.0);
    sim.set_linear_velocity(body, 5.0, 0.0);
    sim.attach_circle_mat(body, 0, 0, 0.5, 1.0, FRIC, 0, resistScale * i, 0);
  }
  controls.appendChild(
    createInfoBox("Keys 1/2/3 set lift 0 / +5 / -5 (or use buttons)"),
  );
  controls.appendChild(createButton("Lift 0", () => { setLift(0); rebuild(); }));
  controls.appendChild(createButton("Lift +5", () => { setLift(5); rebuild(); }));
  controls.appendChild(createButton("Lift -5", () => { setLift(-5); rebuild(); }));
  return {
    paintOverlay: (ctx, camera, canvas) => {
      ctx.fillStyle = "#ffffff";
      ctx.font = "11px sans-serif";
      for (let i = 0; i < 20; ++i) {
        const s = worldToScreen(camera, canvas, -41.5, 2.0 * i + 1.0);
        ctx.fillText((resistScale * i).toFixed(2), s.x, s.y);
      }
    },
  };
}

function buildConveyorBelt(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1198-1253
  sim.add_segment(-20, 0, 20, 0);
  const platform = sim.add_body(-5.0, 5.0, 0, BODY_STATIC);
  sim.attach_rounded_box_mat(platform, 10.0, 0.25, 0.25, 0.0, 0.8, 0.0, 0.0, 2.0);
  for (let i = 0; i < 5; ++i) {
    const b = sim.add_body(-10.0 + 2.0 * i, 7.0, 0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, 0);
  }
  return {};
}

function buildTangentSpeed(
  sim: SimWorld,
  controls: HTMLElement,
  friction: number,
  rolling: number,
  setFric: (v: number) => void,
  setRoll: (v: number) => void,
  rebuild: () => void,
): SceneRuntime {
  // sample_shapes.cpp:1257-1388
  const n = TANGENT_PATH.length / 2;
  const mats: number[] = [];
  const tangents = [-10, -20, -30, -40, -50, -60, -70];
  for (let i = 0; i < n; ++i) {
    mats.push(0.6, 0, 0, i < tangents.length ? tangents[i]! : 0);
  }
  sim.add_chain_mat(TANGENT_PATH, true, mats);

  const bodies: number[] = [];
  let stepCount = 0;
  const totalCount = 200;

  function dropBall() {
    const b = sim.add_body(110.0, -30.0, 0, BODY_DYNAMIC);
    sim.attach_circle_mat(b, 0, 0, 0.5, 1.0, friction, 0, rolling, 0);
    bodies.push(b);
  }

  controls.appendChild(
    createSlider("Friction", 0, 2, friction, 0.01, (v) => {
      setFric(v);
      rebuild();
    }),
  );
  controls.appendChild(
    createSlider("Rolling Resistance", 0, 1, rolling, 0.01, (v) => {
      setRoll(v);
      rebuild();
    }),
  );

  return {
    afterStep: () => {
      stepCount += 1;
      if (stepCount % 25 === 0 && bodies.length < totalCount) dropBall();
    },
  };
}

function buildModifyGeometry(
  sim: SimWorld,
  controls: HTMLElement,
): SceneRuntime {
  // sample_shapes.cpp:1394-1545
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 10, 1, 0, -1, 0, 0, FRIC, 0);
  const falling = sim.add_body(0, 4, 0, BODY_DYNAMIC);
  sim.attach_box(falling, 1, 1, 0, 0, 0, 1.0, FRIC, 0);

  let shapeType: "circle" | "capsule" | "segment" | "polygon" = "circle";
  let scale = 1.0;
  const kin = sim.add_body(0, 1, 0, BODY_KINEMATIC);
  let shapeId = sim.attach_circle_mat(kin, 0, 0, 0.5, 0, FRIC, 0, 0, 0);

  function updateShape() {
    if (shapeType === "circle") sim.shape_set_circle(shapeId, 0, 0, 0.5 * scale);
    else if (shapeType === "capsule")
      sim.shape_set_capsule(shapeId, -0.5 * scale, 0, 0, 0.5 * scale, 0.5 * scale);
    else if (shapeType === "segment")
      sim.shape_set_segment(shapeId, -0.5 * scale, 0, 0.75 * scale, 0);
    else sim.shape_set_box(shapeId, 0.5 * scale, 0.75 * scale);
    sim.body_apply_mass_from_shapes(kin);
  }

  for (const t of ["circle", "capsule", "segment", "polygon"] as const) {
    controls.appendChild(
      createButton(t[0]!.toUpperCase() + t.slice(1), () => {
        shapeType = t;
        updateShape();
      }),
    );
  }
  controls.appendChild(
    createSlider("Scale", 0.1, 10, scale, 0.01, (v) => {
      scale = v;
      updateShape();
    }),
  );
  controls.appendChild(
    createButton("Static", () => sim.set_body_type(kin, BODY_STATIC)),
  );
  controls.appendChild(
    createButton("Kinematic", () => sim.set_body_type(kin, BODY_KINEMATIC)),
  );
  controls.appendChild(
    createButton("Dynamic", () => sim.set_body_type(kin, BODY_DYNAMIC)),
  );
  return {};
}

function buildChainLink(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1550-1626
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  const points1 = [40, 1, 0, 0, -40, 0, -40, -1, 0, -1, 40, -1];
  const points2 = [-40, -1, 0, -1, 40, -1, 40, 0, 0, 0, -40, 0];
  sim.attach_chain(ground, points1, false);
  sim.attach_chain(ground, points2, false);
  const c = sim.add_body(-5, 2, 0, BODY_DYNAMIC);
  sim.attach_circle(c, 0, 0, 0.5, 1.0, FRIC, 0);
  const cap = sim.add_body(0, 2, 0, BODY_DYNAMIC);
  sim.attach_capsule(cap, -0.5, 0, 0.5, 0, 0.25, 1.0, FRIC, 0);
  const box = sim.add_body(5, 2, 0, BODY_DYNAMIC);
  sim.attach_box(box, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, 0);
  return {
    readoutExtra: () => [{ label: "Note", value: "two linked open chains" }],
  };
}

function buildRounded(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1630-1696
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 20, 1, 0, -1, 0, 0, FRIC, 0);
  sim.attach_box(ground, 1, 5, 19, 5, 0, 0, FRIC, 0);
  sim.attach_box(ground, 1, 5, -19, 5, 0, 0, FRIC, 0);

  const rng = makeRng(12345);
  let y = 2.0;
  for (let i = 0; i < 10; ++i) {
    let x = -5.0;
    for (let j = 0; j < 10; ++j) {
      const count = 3 + (rng.next() % 6);
      const pts: number[] = [];
      for (let k = 0; k < count; ++k) {
        pts.push(rng.floatRange(-0.5, 0.5), rng.floatRange(-0.5, 0.5));
      }
      const radius = rng.floatRange(0.05, 0.25);
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_polygon_mat(b, pts, radius, 1.0, FRIC, 0, 0.3, 0);
      x += 1.0;
    }
    y += 1.0;
  }
  return {};
}

function buildEllipse(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1700-1761
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 20, 1, 0, -1, 0, 0, FRIC, 0);
  sim.attach_box(ground, 1, 5, 19, 5, 0, 0, FRIC, 0);
  sim.attach_box(ground, 1, 5, -19, 5, 0, 0, FRIC, 0);
  const points = [0, -0.25, 0, 0.25, 0.05, 0.075, -0.05, 0.075, 0.05, -0.075, -0.05, -0.075];
  let y = 2.0;
  for (let i = 0; i < 10; ++i) {
    let x = -5.0;
    for (let j = 0; j < 10; ++j) {
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_polygon_mat(b, points, 0.2, 1.0, FRIC, 0, 0.2, 0);
      x += 1.0;
    }
    y += 1.0;
  }
  return {};
}

function buildOffset(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1765-1818
  const ground = sim.add_body(-1.0, 1.0, 0, BODY_STATIC);
  sim.attach_box(ground, 1, 1, 10, -2, 0.5 * Math.PI, 0, FRIC, 0);
  const cap = sim.add_body(13.5, -0.75, 0, BODY_DYNAMIC);
  sim.attach_capsule(cap, -5, 1, -4, 1, 0.25, 1.0, FRIC, 0);
  const box = sim.add_body(0, 0, 0, BODY_DYNAMIC);
  sim.attach_box(box, 0.75, 0.5, 9, 2, 0.5 * Math.PI, 1.0, FRIC, 0);
  return {
    paintOverlay: (ctx, camera, canvas) => {
      // DrawTransform identity (:1812)
      const o = worldToScreen(camera, canvas, 0, 0);
      const x = worldToScreen(camera, canvas, 1, 0);
      const y = worldToScreen(camera, canvas, 0, 1);
      ctx.strokeStyle = "#ff0000";
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(x.x, x.y);
      ctx.stroke();
      ctx.strokeStyle = "#00ff00";
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(y.x, y.y);
      ctx.stroke();
    },
  };
}

function buildExplosion(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_shapes.cpp:1825-1931
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let referenceAngle = 0.0;
  let radius = 7.0;
  let falloff = 3.0;
  let impulse = 10.0;
  const joints: number[] = [];
  const r = 8.0;
  for (let angle = 0; angle < 360; angle += 30) {
    const rad = (angle * Math.PI) / 180;
    const px = r * Math.cos(rad);
    const py = r * Math.sin(rad);
    const body = sim.add_body_ex(px, py, 0, BODY_DYNAMIC, 0.0, true);
    sim.attach_box(body, 1.0, 0.1, 0, 0, 0, 1.0, FRIC, 0);
    const j = sim.add_weld_joint(ground, body, px, py, 0.5, 0.5, 0.7, 0.7, false);
    joints.push(j);
  }
  controls.appendChild(
    createButton("Explode", () => sim.explode(0, 0, radius, falloff, impulse)),
  );
  controls.appendChild(createSlider("radius", 0, 20, radius, 0.1, (v) => { radius = v; }));
  controls.appendChild(createSlider("falloff", 0, 20, falloff, 0.1, (v) => { falloff = v; }));
  controls.appendChild(createSlider("impulse", -20, 20, impulse, 0.1, (v) => { impulse = v; }));

  return {
    afterStep: (dt) => {
      if (dt <= 0) return;
      const hertz = 1 / dt;
      referenceAngle += hertz > 0 ? ((60 * Math.PI) / 180) / hertz : 0;
      // unwind roughly
      while (referenceAngle > Math.PI) referenceAngle -= 2 * Math.PI;
      while (referenceAngle < -Math.PI) referenceAngle += 2 * Math.PI;
      for (const j of joints) sim.joint_set_frame_angle_a(j, referenceAngle);
    },
    paintOverlay: (ctx, camera, canvas) => {
      const drawCircle = (rad: number, color: string) => {
        const c = worldToScreen(camera, canvas, 0, 0);
        const edge = worldToScreen(camera, canvas, rad, 0);
        const rpx = Math.hypot(edge.x - c.x, edge.y - c.y);
        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.arc(c.x, c.y, rpx, 0, Math.PI * 2);
        ctx.stroke();
      };
      drawCircle(radius + falloff, "#1b4f72");
      drawCircle(radius, "#f4d03f");
    },
    readoutExtra: () => [{ label: "ref angle", value: referenceAngle.toFixed(3) }],
  };
}

function buildRecreateStatic(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1936-1989
  const box = sim.add_body(0, 1, 0, BODY_DYNAMIC);
  sim.attach_box(box, 1, 1, 0, 0, 0, 1.0, FRIC, 0);
  let groundId = -1;
  return {
    beforeStep: () => {
      if (groundId >= 0 && sim.is_body_alive(groundId)) sim.destroy_body(groundId);
      groundId = sim.add_body(0, 0, 0, BODY_STATIC);
      sim.attach_segment_invoke(groundId, -10, 0, 10, 0);
    },
  };
}

function buildBoxRestitution(sim: SimWorld): SceneRuntime {
  // sample_shapes.cpp:1993-2055
  const count = 10;
  const h = 2.0 * count;
  sim.add_segment(-h, 0, h, 0);
  let restitution = 0.0;
  const dr = 1.0 / (count - 1);
  let x = -1.0 * (count - 1);
  for (let i = 0; i < count; ++i) {
    let b = sim.add_body(x, 1.0, 0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, restitution);
    b = sim.add_body(x, 4.0, 0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, restitution);
    restitution += dr;
    x += 2.0;
  }
  return {};
}

function buildWind(
  sim: SimWorld,
  controls: HTMLElement,
  shapeType: "circle" | "capsule" | "box",
  count: number,
  setType: (t: "circle" | "capsule" | "box") => void,
  setCount: (n: number) => void,
  rebuild: () => void,
): SceneRuntime {
  // sample_shapes.cpp:2059-2220
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let windX = 6.0;
  let windY = 0.0;
  let drag = 1.0;
  let lift = 0.75;
  let noiseX = 0;
  let noiseY = 0;
  const radius = 0.1;
  const bodies: number[] = [];

  let prev = ground;
  for (let i = 0; i < count; ++i) {
    const b = sim.add_body_ex(0, 2.0 - 2.0 * radius * i, 0, BODY_DYNAMIC, 0.5, false);
    if (shapeType === "circle") sim.attach_circle(b, 0, 0, radius, 20.0, FRIC, 0);
    else if (shapeType === "capsule")
      sim.attach_capsule(b, 0, -radius, 0, radius, 0.25 * radius, 20.0, FRIC, 0);
    else sim.attach_box(b, 0.25 * radius, 1.25 * radius, 0, 0, 0, 20.0, FRIC, 0);

    // Revolute spring joint (:2110-2149) — local frames Exact via add_revolute_joint_local
    const ax = 0;
    const ay = i === 0 ? 2.0 + radius : -radius;
    const bx = 0;
    const by = radius;
    sim.add_revolute_joint_local(
      prev, b, ax, ay, bx, by,
      false, 0, 0, false, 0, 0, true, 0.1, 0.0, false,
    );
    bodies.push(b);
    prev = b;
  }

  controls.appendChild(
    createDropdown(
      "Shape",
      [
        { value: "circle", text: "Circle" },
        { value: "capsule", text: "Capsule" },
        { value: "box", text: "Box" },
      ],
      shapeType,
      (v) => {
        setType(v as typeof shapeType);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSlider("Wind X", -20, 20, windX, 0.1, (v) => { windX = v; }));
  controls.appendChild(createSlider("Wind Y", -20, 20, windY, 0.1, (v) => { windY = v; }));
  controls.appendChild(createSlider("Drag", 0, 1, drag, 0.01, (v) => { drag = v; }));
  controls.appendChild(createSlider("Lift", 0, 4, lift, 0.01, (v) => { lift = v; }));
  controls.appendChild(
    createSlider("Count", 1, 60, count, 1, (v) => {
      setCount(v);
      rebuild();
    }),
  );

  const rng = makeRng(99991);
  return {
    afterStep: (dt) => {
      if (dt <= 0) return;
      const speed = Math.hypot(windX, windY);
      const dx = speed > 0 ? windX / speed : 0;
      const dy = speed > 0 ? windY / speed : 0;
      const wx = speed * (dx + noiseX);
      const wy = speed * (dy + noiseY);
      for (const b of bodies) sim.apply_wind_to_body(b, wx, wy, drag, lift, true);
      const rx = rng.floatRange(-0.3, 0.3);
      const ry = rng.floatRange(-0.3, 0.3);
      noiseX = noiseX + (rx - noiseX) * 0.05;
      noiseY = noiseY + (ry - noiseY) * 0.05;
    },
    paintOverlay: (ctx, camera, canvas) => {
      const speed = Math.hypot(windX, windY);
      const dx = speed > 0 ? windX / speed : 0;
      const dy = speed > 0 ? windY / speed : 0;
      const wx = 0.2 * speed * (dx + noiseX);
      const wy = 0.2 * speed * (dy + noiseY);
      const o = worldToScreen(camera, canvas, 0, 0);
      const t = worldToScreen(camera, canvas, wx, wy);
      ctx.strokeStyle = "#ff00ff";
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(t.x, t.y);
      ctx.stroke();
    },
  };
}

// ---------------------------------------------------------------------------
// Page state for scenes that need rebuild params
// ---------------------------------------------------------------------------

type PageState = {
  restitutionShape: "circle" | "box";
  rollingLift: number;
  tangentFric: number;
  tangentRoll: number;
  windShape: "circle" | "capsule" | "box";
  windCount: number;
};

function freshState(): PageState {
  return {
    restitutionShape: "circle",
    rollingLift: 0,
    tangentFric: 0.6,
    tangentRoll: 0.3,
    windShape: "capsule",
    windCount: 10,
  };
}

function buildScene(
  scene: Scene,
  sim: SimWorld,
  controls: HTMLElement,
  state: PageState,
  rebuild: () => void,
): SceneRuntime {
  controls.innerHTML = "";
  switch (scene) {
    case "chain-shape":
      return buildChainShape(sim, controls);
    case "chain-segment":
      return buildChainSegment(sim, controls);
    case "compound-shapes":
      return buildCompoundShapes(sim, controls);
    case "filter":
      return buildFilter(sim, controls);
    case "custom-filter":
      return buildCustomFilter(sim, controls);
    case "restitution":
      return buildRestitutionFixed(
        sim,
        controls,
        state.restitutionShape,
        (t) => {
          state.restitutionShape = t;
        },
        rebuild,
      );
    case "friction":
      return buildFrictionScene(sim);
    case "rolling-resistance":
      return buildRollingResistance(
        sim,
        controls,
        state.rollingLift,
        (v) => {
          state.rollingLift = v;
        },
        rebuild,
      );
    case "conveyor-belt":
      return buildConveyorBelt(sim);
    case "tangent-speed":
      return buildTangentSpeed(
        sim,
        controls,
        state.tangentFric,
        state.tangentRoll,
        (v) => {
          state.tangentFric = v;
        },
        (v) => {
          state.tangentRoll = v;
        },
        rebuild,
      );
    case "modify-geometry":
      return buildModifyGeometry(sim, controls);
    case "chain-link":
      return buildChainLink(sim);
    case "rounded":
      return buildRounded(sim);
    case "ellipse":
      return buildEllipse(sim);
    case "offset":
      return buildOffset(sim);
    case "explosion":
      return buildExplosion(sim, controls);
    case "recreate-static":
      return buildRecreateStatic(sim);
    case "box-restitution":
      return buildBoxRestitution(sim);
    case "wind":
      return buildWind(
        sim,
        controls,
        state.windShape,
        state.windCount,
        (t) => {
          state.windShape = t;
        },
        (n) => {
          state.windCount = n;
        },
        rebuild,
      );
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Shapes",
    "C <code>sample_shapes.cpp</code> RegisterSample ports — chains, filters, " +
      "materials, conveyor, wind, and geometry morph.",
    "Drag to grab · P pause · O step · R restart",
    { category: "Shapes", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "chain-shape";

  const camera: SampleCamera = makeCamera();
  applyCamera(camera, scene);
  const transport = createSampleTransport();
  const state = freshState();
  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = {};

  const sceneControls = document.createElement("div");
  sceneControls.className = "scene-controls";

  function rebuild() {
    runtime.dispose?.();
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera, scene);
    runtime = buildScene(scene, sim, sceneControls, state, rebuild);
  }

  rebuild();

  let grabbing = false;
  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    fitCanvas(canvas);
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    grabbing = sim.mouse_down(w.x, w.y);
    if (grabbing) canvas.setPointerCapture(e.pointerId);
  };
  const onPointerMove = (e: PointerEvent) => {
    if (!grabbing && !sim.mouse_active()) return;
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    sim.mouse_move(w.x, w.y);
  };
  const onPointerUp = () => {
    if (grabbing || sim.mouse_active()) sim.mouse_up();
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
        history.replaceState(null, "", `#/shapes/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    route: "shapes",
    category: "Shapes",
    sampleName: SCENE_LABEL[scene],
    transport,
    onRestart: () => rebuild(),
    getWorld: () => sim,
  });
  controls.appendChild(createSeparator());
  chrome.afterHead.appendChild(sceneControls);
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const unbindKeys = transport.bindKeys();

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    runtime.beforeStep?.(dt);
    sim.step(dt, transport.subSteps);
    runtime.afterStep?.(dt);

    paintSampleDraw(canvas, camera, sim);
    const ctx = canvas.getContext("2d");
    if (ctx && runtime.paintOverlay) runtime.paintOverlay(ctx, camera, canvas);

    updateReadout(readout, [
      { label: "Sample", value: SCENE_LABEL[scene] },
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
      { label: "Hz", value: String(transport.hertz) },
      { label: "Paused", value: transport.paused ? "yes" : "no" },
      ...(runtime.readoutExtra?.() ?? []),
    ]);
  }, readout);

  return () => {
    stop();
    unbindKeys();
    chrome.dispose();
    disposeTransport(transport);
    runtime.dispose?.();
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    freeSim(sim);
  };
}
