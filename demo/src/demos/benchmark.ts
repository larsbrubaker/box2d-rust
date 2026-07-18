// Benchmark — RegisterSample ports from sample_benchmark.cpp / shared/benchmarks.c.
// C citations use line numbers at the pinned submodule (56edae7).
// Body counts follow C DEBUG / m_isDebug (wasm-safe); disclosed per scene → Partial.
// Missing: Cast (world query APIs), Shape Distance / Sensor (distance / custom filter).

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
import { demoPage, fitCanvas, freeSim } from "./sim-common.ts";
import {
  createSampleTransport,
  mountSampleChrome,
  runSampleLoop,
  disposeTransport,
  makeCamera,
  screenToWorld,

  worldToScreen,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — all Benchmark RegisterSample scenes. */
export const SCENES = [
  "barrel",
  "barrel-2-4",
  "compounds",
  "tumbler",
  "washer",
  "many-tumblers",
  "large-pyramid",
  "many-pyramids",
  "create-destroy",
  "sleep",
  "joint-grid",
  "smash",
  "large-compounds",
  "kinematic",
  "cast",
  "spinner",
  "rain",
  "shape-distance",
  "sensor",
  "capacity",
  "junkyard",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("benchmark", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  barrel: "Barrel",
  "barrel-2-4": "Barrel 2.4",
  compounds: "Compounds",
  tumbler: "Tumbler",
  washer: "Washer",
  "many-tumblers": "Many Tumblers",
  "large-pyramid": "Large Pyramid",
  "many-pyramids": "Many Pyramids",
  "create-destroy": "CreateDestroy",
  sleep: "Sleep",
  "joint-grid": "Joint Grid",
  smash: "Smash",
  "large-compounds": "Large Compounds",
  kinematic: "Kinematic",
  cast: "Cast",
  spinner: "Spinner",
  rain: "Rain",
  "shape-distance": "Shape Distance",
  sensor: "Sensor",
  capacity: "Capacity",
  junkyard: "Junkyard",
};

/** Disclosed wasm/DEBUG count notes (C release is larger). */
const SCENE_NOTE: Record<Scene, string> = {
  barrel:
    "DEBUG rows/cols (40×10 compound default; Human 5×10). C sample_benchmark.cpp Barrel.",
  "barrel-2-4": "DEBUG numj=5 (C release 5×26). sample_benchmark.cpp Barrel 2.4.",
  compounds: "DEBUG 10×40 compounds (C release 20×150). CreateCompounds / benchmarks.c.",
  tumbler: "DEBUG gridCount 20 (C release 45). CreateTumbler.",
  washer: "DEBUG gridCount 20 (C release 90). CreateWasher; hit count via hit_events.",
  "many-tumblers": "DEBUG 2×2 tumblers × 8 bodies (C release 19×19 × 50). sample_benchmark.cpp.",
  "large-pyramid": "DEBUG baseCount 20 (C release 100). CreateLargePyramid; sleep off.",
  "many-pyramids": "DEBUG 5×5 pyramids (C release 20×20). CreateManyPyramids; sleep off.",
  "create-destroy": "DEBUG baseCount 40, iterations 1 (C release 100 / 10).",
  sleep: "DEBUG baseCount 40 (C release 100). Filter-joint wake/sleep timing.",
  "joint-grid": "DEBUG N=20 (C release 100). CreateJointGrid; sleep off.",
  smash: "DEBUG 20×10 (C release 120×80). CreateSmash; zero gravity.",
  "large-compounds": "DEBUG ground 100, span/count 5 (C release 200 / 20 / 5).",
  kinematic: "DEBUG span 20 (C release 100). One kinematic compound spinner.",
  cast: "DEBUG 100×100 grid / 100 queries (C release 1000×1000 / 10000). sample_benchmark.cpp Cast.",
  spinner: "DEBUG 499 fill bodies (C release 6076). CreateSpinner; chain friction default (C 0.1).",
  rain: "DEBUG gridCount=200, 3×10×2 humans (C release 500 / 5×40×5). CreateRain / StepRain.",
  "shape-distance":
    "DEBUG count 100 (C release 10000). Free b2ShapeDistance via collision_shape_distance.",
  sensor:
    "Exact: 40×40 sensors + custom filter row + active kill strip. sample_benchmark.cpp Sensor.",
  capacity:
    "Exact: spawns 200 boxes every 32 steps until b2Profile.step >20ms for 60 frames.",
  junkyard: "DEBUG rowCount 2 (C release 40). CreateJunkyard + StepJunkyard pusher.",
};

/** C camera.center / camera.zoom (half-height). */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  barrel: { cx: 8.0, cy: 53.0, zoom: 25.0 * 2.35 },
  "barrel-2-4": { cx: 8.0, cy: 53.0, zoom: 25.0 * 2.35 },
  compounds: { cx: 0.0, cy: 50.0, zoom: 25.0 * 2.2 },
  tumbler: { cx: 1.5, cy: 10.0, zoom: 15.0 },
  washer: { cx: 1.5, cy: 10.0, zoom: 20.0 },
  "many-tumblers": { cx: 1.0, cy: -5.5, zoom: 25.0 * 3.4 },
  "large-pyramid": { cx: 0.0, cy: 50.0, zoom: 25.0 * 2.2 },
  "many-pyramids": { cx: 23.0, cy: 72.5, zoom: 165.0 },
  "create-destroy": { cx: 0.0, cy: 50.0, zoom: 25.0 * 2.2 },
  sleep: { cx: 0.0, cy: 50.0, zoom: 25.0 * 2.2 },
  "joint-grid": { cx: 60.0, cy: -57.0, zoom: 25.0 * 2.5 },
  smash: { cx: 60.0, cy: 6.0, zoom: 25.0 * 1.6 },
  "large-compounds": { cx: 18.0, cy: 115.0, zoom: 25.0 * 5.5 },
  kinematic: { cx: 0.0, cy: 0.0, zoom: 150.0 },
  cast: { cx: 500.0, cy: 500.0, zoom: 25.0 * 21.0 },
  spinner: { cx: 0.0, cy: 32.0, zoom: 42.0 },
  rain: { cx: 0.0, cy: 110.0, zoom: 125.0 },
  "shape-distance": { cx: 0.0, cy: 0.0, zoom: 3.0 },
  sensor: { cx: 0.0, cy: 105.0, zoom: 125.0 },
  capacity: { cx: 0.0, cy: 150.0, zoom: 200.0 },
  junkyard: { cx: 8.0, cy: 25.0, zoom: 60.0 },
};

const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const PI = Math.PI;

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  readoutExtra?: () => { label: string; value: string }[];
  paintOverlay?: (
    ctx: CanvasRenderingContext2D,
    camera: SampleCamera,
    canvas: HTMLCanvasElement,
  ) => void;
  dispose?: () => void;
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

/** C XorShift RandomFloatRange (utils.h) seeded at RAND_SEED. */
function makeRng(seed = 42) {
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
  /** C RandomFloat — [-1, 1]. */
  const float = () => floatRange(-1, 1);
  const intRange = (lo: number, hi: number) => {
    const r = (next() & 0x7fff) / 0x7fff;
    return lo + Math.floor(r * (hi - lo + 1));
  };
  return { next, floatRange, float, intRange };
}

function addBarrelGround(sim: SimWorld, wallExtent: number) {
  // sample_benchmark.cpp Barrel / CreateCompounds / CreateJunkyard ground pattern
  const gridSize = 1.0;
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  let y = 0.0;
  let x = -wallExtent * gridSize;
  for (let i = 0; i < wallExtent * 2 + 1; ++i) {
    sim.attach_box(g, 0.55 * gridSize, 0.5 * gridSize, x, y, 0, 0, 0.6, 0);
    x += gridSize;
  }
  y = gridSize;
  x = -wallExtent * gridSize;
  const wallH = wallExtent === 80 ? 50 : 100;
  for (let i = 0; i < wallH; ++i) {
    sim.attach_box(g, 0.5 * gridSize, 0.55 * gridSize, x, y, 0, 0, 0.6, 0);
    y += gridSize;
  }
  y = gridSize;
  x = wallExtent * gridSize;
  for (let i = 0; i < wallH; ++i) {
    sim.attach_box(g, 0.5 * gridSize, 0.55 * gridSize, x, y, 0, 0, 0.6, 0);
    y += gridSize;
  }
  if (wallExtent === 40) {
    sim.attach_segment(g, -800, -80, 800, -80);
  }
  return g;
}

const LEFT_TRI = new Float32Array([-1.0, 0.0, 0.5, 1.0, 0.0, 2.0]);
const RIGHT_TRI = new Float32Array([1.0, 0.0, -0.5, 1.0, 0.0, 2.0]);

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

function buildBarrel(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:49-321 — DEBUG counts; Human via CreateHuman
  type ShapeKind = "circle" | "capsule" | "mix" | "compound" | "human";
  let shapeType: ShapeKind = "compound";
  const bodies: number[] = [];
  const humans: number[] = [];
  addBarrelGround(sim, 40);

  function createScene() {
    for (const id of bodies) {
      if (sim.is_body_alive(id)) sim.destroy_body(id);
    }
    bodies.length = 0;
    for (const h of humans) {
      if (sim.human_is_spawned(h)) sim.destroy_human(h);
    }
    humans.length = 0;
    const rng = makeRng(42);

    let columnCount = 10;
    let rowCount = 40;
    if (shapeType === "compound") columnCount = 10;
    if (shapeType === "human") {
      rowCount = 5;
      columnCount = 10;
    }

    let shift = 1.15;
    let extray = 0.5;
    let side = -0.1;
    let centerx = (shift * columnCount) / 2.0;
    const centery = shift / 2.0;
    const yStart = shapeType === "human" ? 2.0 : 100.0;

    if (shapeType === "compound") {
      extray = 0.25;
      side = 0.25;
      shift = 2.0;
      centerx = (shift * columnCount) / 2.0 - 1.0;
    } else if (shapeType === "human") {
      extray = 0.5;
      side = 0.55;
      shift = 2.5;
      centerx = (shift * columnCount) / 2.0;
    }

    let index = 0;
    for (let i = 0; i < columnCount; ++i) {
      const x = i * shift - centerx;
      for (let j = 0; j < rowCount; ++j) {
        const y = j * (shift + extray) + centery + yStart;
        const bx = x + side;
        side = -side;
        if (shapeType === "human") {
          // :289-296 CreateHuman scale=3.5
          humans.push(
            sim.create_human(bx, y, 3.5, 0.05, 5.0, 0.5, index + 1, false, 0),
          );
        } else {
          const body = sim.add_body(bx, y, 0, BODY_DYNAMIC);
          bodies.push(body);
          if (shapeType === "circle") {
            const rad = rng.floatRange(0.25, 0.75);
            sim.attach_circle_mat(body, 0, 0, rad, 1.0, 0.5, 0, 0.2, 0);
          } else if (shapeType === "capsule") {
            const rad = rng.floatRange(0.25, 0.5);
            const length = rng.floatRange(0.25, 1.0);
            sim.attach_capsule_mat(body, 0, -0.5 * length, 0, 0.5 * length, rad, 1.0, 0.5, 0, 0.2, 0);
          } else if (shapeType === "mix") {
            sim.set_angular_damping(body, 0.3);
            const mod = bodies.length % 3;
            if (mod === 1) {
              const rad = rng.floatRange(0.25, 0.75);
              sim.attach_circle(body, 0, 0, rad, 1.0, 0.5, 0);
            } else if (mod === 2) {
              const rad = rng.floatRange(0.25, 0.5);
              const length = rng.floatRange(0.25, 1.0);
              sim.attach_capsule(body, 0, -0.5 * length, 0, 0.5 * length, rad, 1.0, 0.5, 0);
            } else {
              const width = rng.floatRange(0.1, 0.5);
              const height = rng.floatRange(0.5, 0.75);
              const value = rng.floatRange(-1.0, 1.0);
              const radius = 0.25 * Math.max(0.0, value);
              sim.attach_rounded_box(body, width, height, radius, 1.0, 0.5, 0);
            }
          } else {
            sim.attach_polygon(body, LEFT_TRI, 0, 1.0, 0.5, 0);
            sim.attach_polygon(body, RIGHT_TRI, 0, 1.0, 0.5, 0);
          }
        }
        index += 1;
      }
    }
  }

  createScene();

  controls.appendChild(
    createDropdown(
      "Shape",
      [
        { value: "circle", text: "Circle" },
        { value: "capsule", text: "Capsule" },
        { value: "mix", text: "Mix" },
        { value: "compound", text: "Compound" },
        { value: "human", text: "Human" },
      ],
      shapeType,
      (v) => {
        shapeType = v as ShapeKind;
        createScene();
      },
    ),
  );
  controls.appendChild(createButton("Reset Scene", () => createScene()));

  return {};
}

function buildBarrel24(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:342-417 — DEBUG numj=5
  const groundSize = 25.0;
  {
    const g = sim.add_body(0, 0, 0, BODY_STATIC);
    sim.attach_box(g, groundSize, 1.2, 0, 0, 0, 0, 0.6, 0);
  }
  {
    const g = sim.add_body(groundSize, 2.0 * groundSize, 0.5 * PI, BODY_STATIC);
    sim.attach_box(g, 2.0 * groundSize, 1.2, 0, 0, 0, 0, 0.6, 0);
  }
  {
    const g = sim.add_body(-groundSize, 2.0 * groundSize, 0.5 * PI, BODY_STATIC);
    sim.attach_box(g, 2.0 * groundSize, 1.2, 0, 0, 0, 0, 0.6, 0);
  }

  const num = 26;
  const rad = 0.5;
  const shift = rad * 2.0;
  const centerx = (shift * num) / 2.0;
  const centery = shift / 2.0;
  const numj = 5; // DEBUG

  for (let i = 0; i < num; ++i) {
    const x = i * shift - centerx;
    for (let j = 0; j < numj; ++j) {
      const y = j * shift + centery + 2.0;
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 1.0, 0.5, 0);
    }
  }
  return {};
}

function buildCompounds(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateCompounds — DEBUG 10×40
  sim.set_sleeping(false);
  addBarrelGround(sim, 40);

  const columnCount = 10;
  const rowCount = 40;
  const shift = 2.0;
  const extray = 0.25;
  let side = 0.25;
  const centerx = (shift * columnCount) / 2.0 - 1.0;
  const centery = 1.15 / 2.0;
  const yStart = 5.0;

  for (let i = 0; i < columnCount; ++i) {
    const x = i * shift - centerx;
    for (let j = 0; j < rowCount; ++j) {
      const y = j * (shift + extray) + centery + yStart;
      const body = sim.add_body(x + side, y, 0, BODY_DYNAMIC);
      side = -side;
      sim.attach_polygon(body, LEFT_TRI, 0, 1.0, 0.5, 0);
      sim.attach_polygon(body, RIGHT_TRI, 0, 1.0, 0.5, 0);
    }
  }
  return {};
}

function buildTumbler(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateTumbler — DEBUG gridCount 20
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  const drum = sim.add_body(0, 10, 0, BODY_DYNAMIC);
  sim.attach_box(drum, 0.5, 10.0, 10.0, 0, 0, 50.0, 0.6, 0);
  sim.attach_box(drum, 0.5, 10.0, -10.0, 0, 0, 50.0, 0.6, 0);
  sim.attach_box(drum, 10.0, 0.5, 0, 10.0, 0, 50.0, 0.6, 0);
  sim.attach_box(drum, 10.0, 0.5, 0, -10.0, 0, 50.0, 0.6, 0);

  const motorSpeed = (PI / 180.0) * 25.0;
  sim.add_revolute_joint_local(
    ground,
    drum,
    0,
    10,
    0,
    0,
    false,
    0,
    0,
    true,
    motorSpeed,
    1e8,
    false,
    0,
    0,
    false,
  );

  const gridCount = 20;
  let y = -0.2 * gridCount + 10.0;
  for (let i = 0; i < gridCount; ++i) {
    let x = -0.2 * gridCount;
    for (let j = 0; j < gridCount; ++j) {
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_box(b, 0.125, 0.125, 0, 0, 0, 1.0, 0.6, 0);
      x += 0.4;
    }
    y += 0.4;
  }
  return {};
}

function buildWasher(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateWasher — kinematic drum, DEBUG grid 20
  sim.add_body(0, 0, 0, BODY_STATIC);
  const motorSpeed = (PI / 180.0) * 25.0;
  const drum = sim.add_body(0, 10, 0, BODY_KINEMATIC);
  sim.set_angular_velocity(drum, motorSpeed);
  sim.set_linear_velocity(drum, 0.001, -0.002);

  const r0 = 14.0;
  const r1 = 16.0;
  const r2 = 18.0;
  const angle = PI / 18.0;
  const qo = 0.1 * angle;
  let u1x = 1.0;
  let u1y = 0.0;
  const cosA = Math.cos(angle);
  const sinA = Math.sin(angle);
  const cosQo = Math.cos(qo);
  const sinQo = Math.sin(qo);

  for (let i = 0; i < 36; ++i) {
    let u2x: number;
    let u2y: number;
    if (i === 35) {
      u2x = 1.0;
      u2y = 0.0;
    } else {
      u2x = cosA * u1x - sinA * u1y;
      u2y = sinA * u1x + cosA * u1y;
    }
    {
      const a1x = cosQo * u1x + sinQo * u1y;
      const a1y = -sinQo * u1x + cosQo * u1y;
      const a2x = cosQo * u2x - sinQo * u2y;
      const a2y = sinQo * u2x + cosQo * u2y;
      const pts = new Float32Array([
        r1 * a1x,
        r1 * a1y,
        r2 * a1x,
        r2 * a1y,
        r1 * a2x,
        r1 * a2y,
        r2 * a2x,
        r2 * a2y,
      ]);
      sim.attach_polygon(drum, pts, 0, 0, 0.6, 0);
    }
    if (i % 9 === 0) {
      const pts = new Float32Array([
        r0 * u1x,
        r0 * u1y,
        r1 * u1x,
        r1 * u1y,
        r0 * u2x,
        r0 * u2y,
        r1 * u2x,
        r1 * u2y,
      ]);
      sim.attach_polygon(drum, pts, 0, 0, 0.6, 0);
    }
    u1x = u2x;
    u1y = u2y;
  }

  const gridCount = 20;
  const a = 0.1;
  let y = -1.1 * a * gridCount + 10.0;
  for (let i = 0; i < gridCount; ++i) {
    let x = -1.1 * a * gridCount;
    for (let j = 0; j < gridCount; ++j) {
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_box(b, a, a, 0, 0, 0, 1.0, 0.6, 0);
      // C enableHitEvents on shapes — partial (hit_events may under-count without shape flag).
      x += 2.1 * a;
    }
    y += 2.1 * a;
  }

  let lastHits = 0;
  return {
    afterStep: () => {
      const ev = sim.hit_events();
      lastHits = Math.floor(ev.length / 3);
    },
    readoutExtra: () => [{ label: "hits", value: String(lastHits) }],
  };
}

function buildManyTumblers(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:501-683 — DEBUG 2×2
  let rowCount = 2;
  let columnCount = 2;
  let angularSpeed = 25.0;
  let tumblerIds: number[] = [];
  let positions: { x: number; y: number }[] = [];
  let bodyIds: number[] = [];
  let bodyIndex = 0;
  let stepCount = 0;
  sim.add_body(0, 0, 0, BODY_STATIC);

  function createTumbler(x: number, y: number): number {
    const body = sim.add_body(x, y, 0, BODY_KINEMATIC);
    sim.set_angular_velocity(body, (PI / 180.0) * angularSpeed);
    sim.attach_box(body, 0.25, 2.0, 2.0, 0, 0, 50.0, 0.6, 0);
    sim.attach_box(body, 0.25, 2.0, -2.0, 0, 0, 50.0, 0.6, 0);
    sim.attach_box(body, 2.0, 0.25, 0, 2.0, 0, 50.0, 0.6, 0);
    sim.attach_box(body, 2.0, 0.25, 0, -2.0, 0, 50.0, 0.6, 0);
    return body;
  }

  function createScene() {
    for (const id of bodyIds) {
      if (id >= 0 && sim.is_body_alive(id)) sim.destroy_body(id);
    }
    for (const id of tumblerIds) {
      if (sim.is_body_alive(id)) sim.destroy_body(id);
    }
    tumblerIds = [];
    positions = [];
    bodyIds = [];
    bodyIndex = 0;
    stepCount = 0;

    let x = -4.0 * rowCount;
    for (let i = 0; i < rowCount; ++i) {
      let y = -4.0 * columnCount;
      for (let j = 0; j < columnCount; ++j) {
        positions.push({ x, y });
        tumblerIds.push(createTumbler(x, y));
        y += 8.0;
      }
      x += 8.0;
    }

    const bodiesPerTumbler = 8;
    const bodyCount = bodiesPerTumbler * tumblerIds.length;
    bodyIds = new Array(bodyCount).fill(-1);
  }

  createScene();

  controls.appendChild(
    createSlider("Row Count", 1, 8, rowCount, 1, (v) => {
      rowCount = v;
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Column Count", 1, 8, columnCount, 1, (v) => {
      columnCount = v;
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Speed", 0, 100, angularSpeed, 1, (v) => {
      angularSpeed = v;
      for (const id of tumblerIds) {
        if (sim.is_body_alive(id)) {
          sim.set_angular_velocity(id, (PI / 180.0) * angularSpeed);
          sim.set_awake(id, true);
        }
      }
    }),
  );

  return {
    afterStep: () => {
      stepCount += 1;
      if (bodyIndex < bodyIds.length && (stepCount & 0x7) === 0) {
        for (let i = 0; i < tumblerIds.length; ++i) {
          if (bodyIndex >= bodyIds.length) break;
          const p = positions[i]!;
          const b = sim.add_body(p.x, p.y, 0, BODY_DYNAMIC);
          sim.attach_capsule(b, -0.1, 0, 0.1, 0, 0.075, 1.0, 0.6, 0);
          bodyIds[bodyIndex] = b;
          bodyIndex += 1;
        }
      }
    },
  };
}

function buildLargePyramid(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateLargePyramid — DEBUG baseCount 20
  sim.set_sleeping(false);
  const g = sim.add_body(0, -1, 0, BODY_STATIC);
  sim.attach_box(g, 100, 1, 0, 0, 0, 0, 0.6, 0);

  const baseCount = 20;
  const a = 0.5;
  const shift = 1.0 * a;
  for (let i = 0; i < baseCount; ++i) {
    const y = (2.0 * i + 1.0) * shift;
    for (let j = i; j < baseCount; ++j) {
      const x = (i + 1.0) * shift + 2.0 * (j - i) * shift - a * baseCount;
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_box(b, a, a, 0, 0, 0, 1.0, 0.6, 0);
    }
  }
  return {};
}

function buildManyPyramids(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateManyPyramids — DEBUG 5×5
  sim.set_sleeping(false);
  const baseCount = 10;
  const extent = 0.5;
  const rowCount = 5;
  const columnCount = 5;

  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  const groundDeltaY = 2.0 * extent * (baseCount + 1.0);
  const groundWidth = 2.0 * extent * columnCount * (baseCount + 1.0);
  let groundY = 0.0;
  for (let i = 0; i < rowCount; ++i) {
    sim.attach_segment(ground, -0.5 * groundWidth, groundY, 0.5 * groundWidth, groundY);
    groundY += groundDeltaY;
  }

  const baseWidth = 2.0 * extent * baseCount;
  let baseY = 0.0;
  for (let i = 0; i < rowCount; ++i) {
    for (let j = 0; j < columnCount; ++j) {
      const centerX = -0.5 * groundWidth + j * (baseWidth + 2.0 * extent) + 2.0 * extent;
      for (let r = 0; r < baseCount; ++r) {
        const y = (2.0 * r + 1.0) * extent + baseY;
        for (let c = r; c < baseCount; ++c) {
          const x = (r + 1.0) * extent + 2.0 * (c - r) * extent + centerX - 0.5;
          const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
          sim.attach_box(b, extent, extent, 0, 0, 0, 1.0, 0.6, 0);
        }
      }
    }
    baseY += groundDeltaY;
  }
  return {};
}

function buildCreateDestroy(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:741-868 — DEBUG base 40, iterations 1
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(g, 100, 1, 0, 0, 0, 0, 0.6, 0);

  const baseCount = 40;
  const iterations = 1;
  let bodies: number[] = [];
  let createMs = 0;
  let destroyMs = 0;
  let bodyCount = 0;

  function createScene() {
    const t0 = performance.now();
    for (const id of bodies) {
      if (sim.is_body_alive(id)) sim.destroy_body(id);
    }
    destroyMs += performance.now() - t0;

    const t1 = performance.now();
    bodies = [];
    const count = baseCount;
    const rad = 0.5;
    const shift = rad * 2.0;
    const centerx = (shift * count) / 2.0;
    const centery = shift / 2.0 + 1.0;
    const h = 0.5;

    for (let i = 0; i < count; ++i) {
      const y = i * shift + centery;
      for (let j = i; j < count; ++j) {
        const x = 0.5 * i * shift + (j - i) * shift - centerx;
        const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
        sim.attach_box(b, h, h, 0, 0, 0, 1.0, 0.5, 0);
        bodies.push(b);
      }
    }
    createMs += performance.now() - t1;
    bodyCount = bodies.length;
    sim.step(1 / 60, 4);
  }

  return {
    beforeStep: () => {
      createMs = 0;
      destroyMs = 0;
      for (let i = 0; i < iterations; ++i) createScene();
    },
    readoutExtra: () => [
      { label: "create ms", value: createMs.toFixed(2) },
      { label: "destroy ms", value: destroyMs.toFixed(2) },
      {
        label: "body µs",
        value:
          bodyCount > 0
            ? `${((1000 * createMs) / iterations / bodyCount).toFixed(1)} / ${((1000 * destroyMs) / iterations / bodyCount).toFixed(1)}`
            : "—",
      },
    ],
  };
}

function buildSleep(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:872-985 — DEBUG base 40
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(g, 100, 1, 0, 0, 0, 0, 0.6, 0);

  const baseCount = 40;
  const bodies: number[] = [];
  const rad = 0.5;
  const shift = rad * 2.0;
  const centerx = (shift * baseCount) / 2.0;
  const centery = shift / 2.0 + 1.0;
  const h = 0.5;

  for (let i = 0; i < baseCount; ++i) {
    const y = i * shift + centery;
    for (let j = i; j < baseCount; ++j) {
      const x = 0.5 * i * shift + (j - i) * shift - centerx;
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.attach_box(b, h, h, 0, 0, 0, 1.0, 0.5, 0);
      bodies.push(b);
    }
  }

  let stepCount = 0;
  let wakeTotal = 0;
  let sleepTotal = 0;

  return {
    afterStep: () => {
      stepCount += 1;
      if (stepCount > 20 && bodies.length >= 2) {
        const joint = sim.add_filter_joint(bodies[0]!, bodies[1]!);
        const t0 = performance.now();
        sim.destroy_joint(joint);
        wakeTotal += performance.now() - t0;
        const t1 = performance.now();
        sim.set_awake(bodies[0]!, false);
        sleepTotal += performance.now() - t1;
      }
    },
    readoutExtra: () => {
      const n = Math.max(1, stepCount - 20);
      return [
        { label: "wake ave ms", value: (wakeTotal / n).toFixed(3) },
        { label: "sleep ave ms", value: (sleepTotal / n).toFixed(3) },
      ];
    },
  };
}

function buildJointGrid(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateJointGrid — DEBUG N=20
  sim.set_sleeping(false);
  const N = 20;
  const bodies: number[] = [];
  let index = 0;

  for (let k = 0; k < N; ++k) {
    for (let i = 0; i < N; ++i) {
      const isStatic = k >= N / 2 - 3 && k <= N / 2 + 3 && i === 0;
      const body = sim.add_body(k, -i, 0, isStatic ? BODY_STATIC : BODY_DYNAMIC);
      const sh = sim.attach_circle_mat(body, 0, 0, 0.4, 1.0, 0.6, 0, 0, 0);
      sim.shape_set_filter(sh, 2, ~2 >>> 0);

      if (i > 0) {
        sim.add_revolute_joint_local(
          bodies[index - 1]!,
          body,
          0,
          -0.5,
          0,
          0.5,
          false,
          0,
          0,
          false,
          0,
          0,
          false,
          0,
          0,
          false,
        );
      }
      if (k > 0) {
        sim.add_revolute_joint_local(
          bodies[index - N]!,
          body,
          0.5,
          0,
          -0.5,
          0,
          false,
          0,
          0,
          false,
          0,
          0,
          false,
          0,
          0,
          false,
        );
      }
      bodies.push(body);
      index += 1;
    }
  }
  return {};
}

function buildSmash(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateSmash — DEBUG 20×10
  sim.set_gravity(0, 0);
  const heavy = sim.add_body(-20, 0, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(heavy, 40, 0);
  sim.attach_box(heavy, 4, 4, 0, 0, 0, 8.0, 0.6, 0);

  const d = 0.4;
  const columns = 20;
  const rows = 10;
  for (let i = 0; i < columns; ++i) {
    for (let j = 0; j < rows; ++j) {
      const x = i * d + 30.0;
      const y = (j - rows / 2.0) * d;
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.set_awake(b, false);
      sim.attach_box(b, 0.5 * d, 0.5 * d, 0, 0, 0, 1.0, 0.6, 0);
    }
  }
  return {};
}

function buildLargeCompounds(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:1036-1131 — DEBUG 100 / span 5
  const grid = 1.0;
  const height = 100;
  const width = 100;
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  for (let i = 0; i < height; ++i) {
    const y = grid * i;
    for (let j = i; j < width; ++j) {
      sim.attach_box(ground, 0.5 * grid, 0.5 * grid, grid * j, y, 0, 0, 0.6, 0);
      sim.attach_box(ground, 0.5 * grid, 0.5 * grid, -grid * j, y, 0, 0, 0.6, 0);
    }
  }

  const span = 5;
  const count = 5;
  for (let m = 0; m < count; ++m) {
    const ybody = (100.0 + m * span) * grid;
    for (let n = 0; n < count; ++n) {
      const xbody = -0.5 * grid * count * span + n * span * grid;
      const body = sim.add_body(xbody, ybody, 0, BODY_DYNAMIC);
      for (let i = 0; i < span; ++i) {
        for (let j = 0; j < span; ++j) {
          sim.attach_box(body, 0.5 * grid, 0.5 * grid, j * grid, i * grid, 0, 1.0, 0.6, 0);
        }
      }
      sim.body_apply_mass_from_shapes(body);
    }
  }
  return {};
}

function buildKinematic(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:1136-1187 — DEBUG span 20
  const grid = 1.0;
  const span = 20;
  const body = sim.add_body(0, 0, 0, BODY_KINEMATIC);
  sim.set_angular_velocity(body, 1.0);
  for (let i = -span; i < span; ++i) {
    const y = i * grid;
    for (let j = -span; j < span; ++j) {
      const x = j * grid;
      // density 0 while attaching; ApplyMassFromShapes after (C updateBodyMass=false)
      const sh = sim.attach_box_mat(body, 0.5 * grid, 0.5 * grid, x, y, 0, 0, 0.6, 0, 0, 0);
      sim.shape_set_filter(sh, 1, 2);
    }
  }
  sim.body_apply_mass_from_shapes(body);
  return {};
}

function buildSpinner(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateSpinner — DEBUG 499 fill bodies
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  const pointCount = 360;
  const points: number[] = [];
  let px = 40.0;
  let py = 0.0;
  const dq = (-2.0 * PI) / pointCount;
  const cosQ = Math.cos(dq);
  const sinQ = Math.sin(dq);
  for (let i = 0; i < pointCount; ++i) {
    points.push(px, py + 32.0);
    const nx = cosQ * px - sinQ * py;
    const ny = sinQ * px + cosQ * py;
    px = nx;
    py = ny;
  }
  // C: chain on ground with friction 0.1. attach_chain has no per-material; disclose.
  sim.attach_chain(ground, new Float32Array(points), true);

  const spinner = sim.add_body(0, 12, 0, BODY_DYNAMIC);
  sim.enable_body_sleep(spinner, false);
  sim.attach_rounded_box(spinner, 0.4, 20.0, 0.2, 1.0, 0.0, 0);
  const joint = sim.add_revolute_joint_local(
    ground,
    spinner,
    0,
    12,
    0,
    0,
    false,
    0,
    0,
    true,
    5.0,
    Number.MAX_VALUE,
    false,
    0,
    0,
    false,
  );

  const bodyCount = 499;
  let x = -23.0;
  let y = 2.0;
  for (let i = 0; i < bodyCount; ++i) {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    const rem = i % 3;
    if (rem === 0) {
      sim.attach_capsule(b, -0.25, 0, 0.25, 0, 0.25, 0.25, 0.1, 0.1);
    } else if (rem === 1) {
      sim.attach_circle(b, 0, 0, 0.35, 0.25, 0.1, 0.1);
    } else {
      sim.attach_box(b, 0.35, 0.35, 0, 0, 0, 0.25, 0.1, 0.1);
    }
    x += 0.5;
    if (x >= 23.0) {
      x = -23.0;
      y += 0.5;
    }
  }

  return {
    readoutExtra: () => [
      { label: "spinner angle", value: sim.revolute_get_angle(joint).toFixed(2) },
    ],
  };
}

/** Capacity gates on `b2Profile.step` from the engine (not wall-clock). */
type CapacityRuntime = SceneRuntime;

function buildRain(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // shared/benchmarks.c CreateRain / StepRain — DEBUG constants
  const RAIN_ROW_COUNT = 3;
  const RAIN_COLUMN_COUNT = 10;
  const RAIN_GROUP_SIZE = 2;
  const gridSize = 0.5;
  const gridCount = 200; // BENCHMARK_DEBUG
  const delay = 0x1f; // DEBUG delay mask

  // CreateRain ground shelves
  {
    const g = sim.add_body(0, 0, 0, BODY_STATIC);
    let y = 0.0;
    const width = gridSize;
    const height = gridSize;
    for (let i = 0; i < RAIN_ROW_COUNT; ++i) {
      let x = -0.5 * gridCount * gridSize;
      for (let j = 0; j <= gridCount; ++j) {
        sim.attach_box(g, 0.5 * width, 0.5 * height, x, y, 0, 0, 0.6, 0);
        x += gridSize;
      }
      y += 45.0;
    }
  }

  // groupIndex → human demo indices (RAIN_GROUP_SIZE each)
  const groups: number[][] = Array.from(
    { length: RAIN_ROW_COUNT * RAIN_COLUMN_COUNT },
    () => [],
  );
  let columnCount = 0;
  let columnIndex = 0;
  let stepCount = 0;

  function createGroup(rowIndex: number, colIndex: number) {
    const groupIndex = rowIndex * RAIN_COLUMN_COUNT + colIndex;
    const span = gridCount * gridSize;
    const groupDistance = (1.0 * span) / RAIN_COLUMN_COUNT;
    let px = -0.5 * span + groupDistance * (colIndex + 0.5);
    const py = 40.0 + 45.0 * rowIndex;
    const slot: number[] = [];
    for (let i = 0; i < RAIN_GROUP_SIZE; ++i) {
      slot.push(sim.create_human(px, py, 1.0, 0.05, 5.0, 0.5, i + 1, false, 0));
      px += 0.5;
    }
    groups[groupIndex] = slot;
  }

  function destroyGroup(rowIndex: number, colIndex: number) {
    const groupIndex = rowIndex * RAIN_COLUMN_COUNT + colIndex;
    for (const h of groups[groupIndex]!) {
      if (sim.human_is_spawned(h)) sim.destroy_human(h);
    }
    groups[groupIndex] = [];
  }

  controls.appendChild(
    createInfoBox(
      "PARTIAL: DEBUG Rain (gridCount=200, 3×10×2 humans). " +
        "<code>CreateRain</code>/<code>StepRain</code> via <code>CreateHuman</code>.",
    ),
  );

  return {
    beforeStep: () => {
      // StepRain runs before world step when not paused (sample_benchmark.cpp:1646-1648)
      if ((stepCount & delay) === 0) {
        if (columnCount < RAIN_COLUMN_COUNT) {
          for (let i = 0; i < RAIN_ROW_COUNT; ++i) createGroup(i, columnCount);
          columnCount += 1;
        } else {
          for (let i = 0; i < RAIN_ROW_COUNT; ++i) {
            destroyGroup(i, columnIndex);
            createGroup(i, columnIndex);
          }
          columnIndex = (columnIndex + 1) % RAIN_COLUMN_COUNT;
        }
      }
      stepCount += 1;
    },
    readoutExtra: () => [
      { label: "columns", value: String(columnCount) },
      { label: "cycle", value: String(columnIndex) },
    ],
  };
}

function buildCapacityFull(sim: SimWorld, _controls: HTMLElement): CapacityRuntime {
  const g = sim.add_body(0, -5, 0, BODY_STATIC);
  sim.attach_box(g, 800, 5, 0, 0, 0, 0, 0.6, 0);

  let stepCount = 0;
  let reachCount = 0;
  let done = false;
  let lastStepMs = 0;

  const rt: CapacityRuntime = {
    afterStep: () => {
      stepCount += 1;
      if (done) return;
      // C: b2World_GetProfile(m_worldId).step vs 20 ms for 60 consecutive frames
      lastStepMs = sim.get_profile_step();
      if (lastStepMs > 20) {
        reachCount += 1;
        if (reachCount > 60) done = true;
      } else {
        reachCount = 0;
      }
      if (done) return;
      if ((stepCount & 0x1f) !== 0x1f) return;
      let x = -200.0;
      let y = 200.0;
      for (let i = 0; i < 200; ++i) {
        y += 0.5;
        const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
        sim.attach_box(b, 0.5, 0.5, 0, 0, 0, 1.0, 0.6, 0);
        x += 2.0;
      }
    },
    readoutExtra: () => [
      { label: "profile step ms", value: lastStepMs.toFixed(2) },
      { label: "done", value: done ? "yes" : "no" },
    ],
  };
  return rt;
}

function buildJunkyard(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // benchmarks.c CreateJunkyard + StepJunkyard — DEBUG rowCount 2
  addBarrelGround(sim, 80);

  const columnCount = 200;
  const rowCount = 2;
  const radius = 0.25;
  const phi = PI * (Math.sqrt(5.0) - 1.0);
  const pts: number[] = [];
  for (let i = 0; i < 5; ++i) {
    const theta = phi * i;
    pts.push(radius * Math.cos(theta), radius * Math.sin(theta));
  }
  const hull = new Float32Array(pts);

  let side = -0.1;
  const yStart = 15.0;
  for (let i = 0; i < columnCount; ++i) {
    const x = 1.5 * (2.0 * i - columnCount) * radius;
    for (let j = 0; j < rowCount; ++j) {
      const y = 4.0 * j * radius + yStart;
      const b = sim.add_body(x + side, y, 0, BODY_DYNAMIC);
      side = -side;
      sim.attach_polygon(b, hull, 0, 1.0, 0.6, 0);
    }
  }

  const pusher = sim.add_body(0, 0, 0, BODY_KINEMATIC);
  sim.attach_box(pusher, 2.0, 4.0, 0, 4.0, 0, 0, 0.6, 0);
  let stepCount = 0;

  return {
    beforeStep: (dt) => {
      if (dt <= 0) return;
      stepCount += 1;
      const time = (1 / 60) * stepCount;
      const sine = Math.sin(0.2 * time);
      sim.set_target_transform(pusher, 60.0 * sine, 0.0, 0.0, 1 / 60, true);
    },
  };
}

// ---------------------------------------------------------------------------
// Cast / Shape Distance / Sensor — previously Missing
// ---------------------------------------------------------------------------

const COLOR_BOX2D_BLUE = 0x30aebf;
const COLOR_BOX2D_YELLOW = 0xffee8c;
const COLOR_BOX2D_GREEN = 0x8cc924;
const COLOR_FUCHSIA = 0xff00ff;
const COLOR_LIME = 0x00ff00;

function buildCast(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:1199-1581 — DEBUG grid/sample counts
  type QueryKind = "ray" | "circle" | "overlap";
  let queryType: QueryKind = "circle";
  let rowCount = 100;
  let columnCount = 100;
  let fill = 0.1;
  let grid = 1.0;
  let ratio = 5.0;
  let topDown = false;
  let radius = 0.1;
  let drawIndex = 0;
  let minTime = 1e6;
  let buildTime = 0;
  let lastHit = 0;
  let lastNode = 0;
  let lastLeaf = 0;
  let lastMs = 0;
  const sampleCount = 100;
  const origins: { x: number; y: number }[] = [];
  const translations: { x: number; y: number }[] = [];
  const gridBodies: number[] = [];

  let drawHit = false;
  let drawPx = 0;
  let drawPy = 0;
  let drawFrac = 1;
  const overlapPts: { x: number; y: number }[] = [];

  function precomputeRays() {
    const rng = makeRng(1234);
    origins.length = 0;
    translations.length = 0;
    const extent = rowCount * grid;
    for (let i = 0; i < sampleCount; ++i) {
      const sx = rng.floatRange(0, extent);
      const sy = rng.floatRange(0, extent);
      const ex = rng.floatRange(0, extent);
      const ey = rng.floatRange(0, extent);
      origins.push({ x: sx, y: sy });
      translations.push({ x: ex - sx, y: ey - sy });
    }
    drawIndex = drawIndex % sampleCount;
  }

  function buildGrid() {
    for (const id of gridBodies) {
      if (sim.is_body_alive(id)) sim.destroy_body(id);
    }
    gridBodies.length = 0;
    const rng = makeRng(1234);
    const t0 = performance.now();
    let y = 0;
    for (let i = 0; i < rowCount; ++i) {
      let x = 0;
      for (let j = 0; j < columnCount; ++j) {
        if (rng.floatRange(0, 1) <= fill) {
          const b = sim.add_body(x, y, 0, BODY_STATIC);
          const r = rng.floatRange(1, ratio);
          const half = rng.floatRange(0.05, 0.25);
          const hx = rng.float() > 0 ? r * half : half;
          const hy = rng.float() > 0 ? half : r * half;
          const cat = rng.intRange(0, 2);
          const color =
            cat === 0 ? COLOR_BOX2D_BLUE : cat === 1 ? COLOR_BOX2D_YELLOW : COLOR_BOX2D_GREEN;
          sim.attach_box_category_color(b, hx, hy, 1 << cat, color);
          gridBodies.push(b);
        }
        x += grid;
      }
      y += grid;
    }
    if (topDown) sim.rebuild_static_tree();
    buildTime = performance.now() - t0;
    minTime = 1e6;
  }

  precomputeRays();
  buildGrid();

  controls.appendChild(
    createDropdown(
      "Query",
      [
        { value: "ray", text: "Ray" },
        { value: "circle", text: "Circle" },
        { value: "overlap", text: "Overlap" },
      ],
      queryType,
      (v) => {
        queryType = v as QueryKind;
        radius = queryType === "overlap" ? 5.0 : 0.1;
        minTime = 1e6;
      },
    ),
  );
  controls.appendChild(
    createSlider("rows", 0, 100, rowCount, 1, (v) => {
      rowCount = v;
      precomputeRays();
      buildGrid();
    }),
  );
  controls.appendChild(
    createSlider("columns", 0, 100, columnCount, 1, (v) => {
      columnCount = v;
      precomputeRays();
      buildGrid();
    }),
  );
  controls.appendChild(
    createSlider("fill", 0, 1, fill, 0.01, (v) => {
      fill = v;
      buildGrid();
    }),
  );
  controls.appendChild(
    createSlider("grid", 0.5, 2, grid, 0.01, (v) => {
      grid = v;
      precomputeRays();
      buildGrid();
    }),
  );
  controls.appendChild(
    createSlider("ratio", 1, 10, ratio, 0.01, (v) => {
      ratio = v;
      buildGrid();
    }),
  );
  controls.appendChild(
    createCheckbox("top down", topDown, (v) => {
      topDown = v;
      buildGrid();
    }),
  );
  controls.appendChild(
    createButton("Draw Next", () => {
      drawIndex = (drawIndex + 1) % sampleCount;
    }),
  );

  return {
    afterStep: (dt) => {
      if (dt <= 0) return;
      let hitCount = 0;
      let nodeVisits = 0;
      let leafVisits = 0;
      drawHit = false;
      overlapPts.length = 0;
      const t0 = performance.now();
      if (queryType === "ray") {
        for (let i = 0; i < sampleCount; ++i) {
          const o = origins[i]!;
          const t = translations[i]!;
          const r = sim.cast_ray_closest_mask(o.x, o.y, t.x, t.y, 1);
          nodeVisits += r[6]!;
          leafVisits += r[7]!;
          hitCount += r[0]! > 0 ? 1 : 0;
          if (i === drawIndex) {
            drawHit = r[0]! > 0;
            drawPx = r[1]!;
            drawPy = r[2]!;
          }
        }
      } else if (queryType === "circle") {
        for (let i = 0; i < sampleCount; ++i) {
          const o = origins[i]!;
          const t = translations[i]!;
          const r = sim.cast_circle_closest_mask(o.x, o.y, radius, t.x, t.y, 1);
          nodeVisits += r[4]!;
          leafVisits += r[5]!;
          hitCount += r[0]! > 0 ? 1 : 0;
          if (i === drawIndex) {
            drawHit = r[0]! > 0;
            drawPx = r[1]!;
            drawPy = r[2]!;
            drawFrac = r[3]!;
          }
        }
      } else {
        for (let i = 0; i < sampleCount; ++i) {
          const o = origins[i]!;
          const r = sim.overlap_aabb_centers_mask(o.x, o.y, radius, radius, 1);
          nodeVisits += r[0]!;
          leafVisits += r[1]!;
          const count = r[2]!;
          hitCount += count;
          if (i === drawIndex) {
            for (let k = 0; k < count; ++k) {
              overlapPts.push({ x: r[3 + k * 2]!, y: r[4 + k * 2]! });
            }
          }
        }
      }
      lastMs = performance.now() - t0;
      minTime = Math.min(minTime, lastMs);
      lastHit = hitCount;
      lastNode = nodeVisits;
      lastLeaf = leafVisits;
    },
    readoutExtra: () => [
      { label: "build ms", value: buildTime.toFixed(2) },
      { label: "hits / nodes / leaves", value: `${lastHit} / ${lastNode} / ${lastLeaf}` },
      { label: "total ms", value: lastMs.toFixed(3) },
      { label: "min ms", value: minTime.toFixed(3) },
      {
        label: "ave us",
        value: ((1000 * minTime) / sampleCount).toFixed(2),
      },
    ],
    paintOverlay: (ctx, camera, canvas) => {
      const o = origins[drawIndex];
      const t = translations[drawIndex];
      if (!o || !t) return;
      if (queryType === "overlap") {
        const lowerX = Math.floor(o.x - radius);
        const lowerY = Math.floor(o.y - radius);
        const upperX = Math.ceil(o.x + radius);
        const upperY = Math.ceil(o.y + radius);
        const a = worldToScreen(camera, canvas, lowerX, lowerY);
        const b = worldToScreen(camera, canvas, upperX, upperY);
        ctx.strokeStyle = "#fff";
        ctx.strokeRect(a.x, b.y, b.x - a.x, a.y - b.y);
        for (const p of overlapPts) {
          const s = worldToScreen(camera, canvas, p.x, p.y);
          ctx.fillStyle = "#ff69b4";
          ctx.beginPath();
          ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
          ctx.fill();
        }
        return;
      }
      const p1 = worldToScreen(camera, canvas, o.x, o.y);
      const p2 = worldToScreen(camera, canvas, o.x + t.x, o.y + t.y);
      ctx.strokeStyle = "#fff";
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      ctx.fillStyle = "#0f0";
      ctx.beginPath();
      ctx.arc(p1.x, p1.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#f00";
      ctx.beginPath();
      ctx.arc(p2.x, p2.y, 4, 0, Math.PI * 2);
      ctx.fill();
      if (drawHit) {
        if (queryType === "circle") {
          const cx = o.x + drawFrac * t.x;
          const cy = o.y + drawFrac * t.y;
          const c = worldToScreen(camera, canvas, cx, cy);
          const ppm = Math.abs(
            worldToScreen(camera, canvas, cx + radius, cy).x - c.x,
          );
          ctx.strokeStyle = "#fff";
          ctx.beginPath();
          ctx.arc(c.x, c.y, ppm, 0, Math.PI * 2);
          ctx.stroke();
        }
        const h = worldToScreen(camera, canvas, drawPx, drawPy);
        ctx.fillStyle = "#fff";
        ctx.beginPath();
        ctx.arc(h.x, h.y, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    },
  };
}

function octagonVerts(radius: number): number[] {
  const pts: number[] = [];
  const q = (2 * PI) / 8;
  let x = radius;
  let y = 0;
  pts.push(x, y);
  for (let i = 1; i < 8; ++i) {
    const nx = Math.cos(q) * x - Math.sin(q) * y;
    const ny = Math.sin(q) * x + Math.cos(q) * y;
    x = nx;
    y = ny;
    pts.push(x, y);
  }
  return pts;
}

function invMulWorld(
  ax: number,
  ay: number,
  aa: number,
  bx: number,
  by: number,
  ba: number,
): { tx: number; ty: number; angle: number } {
  const c = Math.cos(aa);
  const s = Math.sin(aa);
  const dx = bx - ax;
  const dy = by - ay;
  return {
    tx: c * dx + s * dy,
    ty: -s * dx + c * dy,
    angle: ba - aa,
  };
}

function buildShapeDistance(sim: SimWorld, controls: HTMLElement, wasm: ReturnType<typeof getWasm>): SceneRuntime {
  // sample_benchmark.cpp:1667-1798 — DEBUG count 100
  const count = 100;
  const vertsA = octagonVerts(0.5);
  const vertsB = octagonVerts(0.5);
  const radiusA = 0.0;
  const radiusB = 0.1;
  const xfA: { x: number; y: number; a: number }[] = [];
  const xfB: { x: number; y: number; a: number }[] = [];
  const outputs: {
    pax: number;
    pay: number;
    pbx: number;
    pby: number;
    dist: number;
    iter: number;
    nx: number;
    ny: number;
  }[] = [];
  let drawIndex = 0;
  let minMs = Number.POSITIVE_INFINITY;
  let lastAveIter = 0;

  const rng = makeRng(42);
  for (let i = 0; i < count; ++i) {
    xfA.push({
      x: rng.floatRange(-0.1, 0.1),
      y: rng.floatRange(-0.1, 0.1),
      a: rng.floatRange(-PI, PI),
    });
    xfB.push({
      x: rng.floatRange(0.25, 2.0),
      y: rng.floatRange(0.25, 2.0),
      a: rng.floatRange(-PI, PI),
    });
    outputs.push({ pax: 0, pay: 0, pbx: 0, pby: 0, dist: 0, iter: 0, nx: 0, ny: 0 });
  }

  // Visual stand-ins for the draw-index pair (empty world otherwise).
  const bodyA = sim.add_body(0, 0, 0, BODY_KINEMATIC);
  sim.attach_polygon(bodyA, vertsA, radiusA, 0, 0.3, 0);
  const bodyB = sim.add_body(1, 0, 0, BODY_KINEMATIC);
  sim.attach_polygon(bodyB, vertsB, radiusB, 0, 0.3, 0);

  controls.appendChild(
    createSlider("draw index", 0, count - 1, drawIndex, 1, (v) => {
      drawIndex = v;
    }),
  );

  return {
    afterStep: (dt) => {
      if (dt <= 0) return;
      let totalIter = 0;
      const t0 = performance.now();
      for (let i = 0; i < count; ++i) {
        const a = xfA[i]!;
        const b = xfB[i]!;
        const rel = invMulWorld(a.x, a.y, a.a, b.x, b.y, b.a);
        const out = wasm.collision_shape_distance(
          vertsA,
          radiusA,
          vertsB,
          radiusB,
          rel.tx,
          rel.ty,
          rel.angle,
          true,
        );
        outputs[i] = {
          pax: out[0]!,
          pay: out[1]!,
          pbx: out[2]!,
          pby: out[3]!,
          dist: out[4]!,
          iter: out[5]!,
          nx: out[6]!,
          ny: out[7]!,
        };
        totalIter += out[5]!;
      }
      const ms = performance.now() - t0;
      minMs = Math.min(minMs, ms);
      lastAveIter = totalIter / count;
      const da = xfA[drawIndex]!;
      const db = xfB[drawIndex]!;
      sim.set_transform(bodyA, da.x, da.y, da.a);
      sim.set_transform(bodyB, db.x, db.y, db.a);
    },
    readoutExtra: () => {
      const o = outputs[drawIndex]!;
      return [
        { label: "count", value: String(count) },
        {
          label: "min ms / ave us",
          value: `${minMs.toFixed(3)} / ${((1000 * minMs) / count).toFixed(2)}`,
        },
        { label: "ave iterations", value: lastAveIter.toFixed(2) },
        { label: "distance", value: o.dist.toFixed(4) },
      ];
    },
    paintOverlay: (ctx, camera, canvas) => {
      const a = xfA[drawIndex]!;
      const o = outputs[drawIndex]!;
      const c = Math.cos(a.a);
      const s = Math.sin(a.a);
      const wxA = a.x + c * o.pax - s * o.pay;
      const wyA = a.y + s * o.pax + c * o.pay;
      const wxB = a.x + c * o.pbx - s * o.pby;
      const wyB = a.y + s * o.pbx + c * o.pby;
      const nx = c * o.nx - s * o.ny;
      const ny = s * o.nx + c * o.ny;
      const pA = worldToScreen(camera, canvas, wxA, wyA);
      const pB = worldToScreen(camera, canvas, wxB, wyB);
      const pN = worldToScreen(camera, canvas, wxA + 0.5 * nx, wyA + 0.5 * ny);
      ctx.strokeStyle = "#696969";
      ctx.beginPath();
      ctx.moveTo(pA.x, pA.y);
      ctx.lineTo(pB.x, pB.y);
      ctx.stroke();
      ctx.strokeStyle = "#ff0";
      ctx.beginPath();
      ctx.moveTo(pA.x, pA.y);
      ctx.lineTo(pN.x, pN.y);
      ctx.stroke();
      ctx.fillStyle = "#fff";
      for (const p of [pA, pB]) {
        ctx.beginPath();
        ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
        ctx.fill();
      }
    },
  };
}

function buildSensor(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_benchmark.cpp:1808-2023 — Exact 40×40
  const columnCount = 40;
  const rowCount = 40;
  const filterRow = rowCount >> 1;
  sim.enable_sensor_row_filter(true, filterRow);

  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  const activeUd = sim.pack_sensor_user_data(0, true);
  {
    const gridSize = 3.0;
    let x = -40.0 * gridSize;
    for (let i = 0; i < 81; ++i) {
      sim.attach_sensor_ud(
        ground,
        0.5 * gridSize,
        0.5 * gridSize,
        x,
        0,
        0,
        0,
        true,
        true,
        false,
        activeUd,
        0,
      );
      x += gridSize;
    }
  }

  const rng = makeRng(42);
  const shift = 5.0;
  const xCenter = 0.5 * shift * columnCount;
  const yStart = 10.0;
  for (let j = 0; j < rowCount; ++j) {
    const ud = sim.pack_sensor_user_data(j, false);
    const custom = j === filterRow;
    const color = custom ? COLOR_FUCHSIA : 0;
    const y = j * shift + yStart;
    for (let i = 0; i < columnCount; ++i) {
      const x = i * shift - xCenter;
      sim.attach_sensor_ud(ground, 0.5, 0.5, x, y, 0, 0.1, true, true, custom, ud, color);
    }
  }

  let maxBegin = 0;
  let maxEnd = 0;
  let lastStep = -1;
  let stepCount = 0;
  void controls;

  function createRow(y: number) {
    for (let i = 0; i < columnCount; ++i) {
      const yOffset = rng.floatRange(-1, 1);
      const b = sim.add_body_ex(shift * i - xCenter, y + yOffset, 0, BODY_DYNAMIC, 0, true);
      sim.set_linear_velocity(b, 0, -5);
      sim.attach_circle_ex(b, 0, 0, 0.5, 1, 0.3, 0, 0, false, true, false, false, 0, 0);
    }
  }

  return {
    afterStep: (dt) => {
      if (dt <= 0) return;
      stepCount += 1;
      if (stepCount === lastStep) return;

      const begin = sim.sensor_begin_events();
      const end = sim.sensor_end_events();
      const beginCount = begin.length / 2;
      const endCount = end.length / 2;
      const zombies = new Set<number>();

      for (let i = 0; i < begin.length; i += 2) {
        const sensorShape = begin[i]!;
        const visitorShape = begin[i + 1]!;
        const ud = sim.shape_get_user_data(sensorShape);
        const active = (ud & 1) !== 0;
        if (active) {
          const body = sim.shape_body_index(visitorShape);
          if (body >= 0) zombies.add(body);
        } else {
          sim.shape_set_custom_color(visitorShape, COLOR_LIME);
        }
      }
      for (let i = 0; i < end.length; i += 2) {
        const visitorShape = end[i + 1]!;
        if (!sim.shape_is_valid(visitorShape)) continue;
        sim.shape_set_custom_color(visitorShape, 0);
      }
      for (const body of zombies) {
        if (sim.is_body_alive(body)) sim.destroy_body(body);
      }
      if ((stepCount & 0x1f) === 0) {
        createRow(10.0 + rowCount * 5.0);
      }
      lastStep = stepCount;
      maxBegin = Math.max(maxBegin, beginCount);
      maxEnd = Math.max(maxEnd, endCount);
    },
    readoutExtra: () => [
      { label: "max begin", value: String(maxBegin) },
      { label: "max end", value: String(maxEnd) },
    ],
    dispose: () => sim.enable_sensor_row_filter(false, 0),
  };
}

// ---------------------------------------------------------------------------
// Dispatch + page
// ---------------------------------------------------------------------------

function buildScene(
  scene: Scene,
  sim: SimWorld,
  controls: HTMLElement,
  wasm: ReturnType<typeof getWasm>,
): SceneRuntime {
  controls.replaceChildren();
  controls.appendChild(createInfoBox(SCENE_NOTE[scene]));
  switch (scene) {
    case "barrel":
      return buildBarrel(sim, controls);
    case "barrel-2-4":
      return buildBarrel24(sim, controls);
    case "compounds":
      return buildCompounds(sim, controls);
    case "tumbler":
      return buildTumbler(sim, controls);
    case "washer":
      return buildWasher(sim, controls);
    case "many-tumblers":
      return buildManyTumblers(sim, controls);
    case "large-pyramid":
      return buildLargePyramid(sim, controls);
    case "many-pyramids":
      return buildManyPyramids(sim, controls);
    case "create-destroy":
      return buildCreateDestroy(sim, controls);
    case "sleep":
      return buildSleep(sim, controls);
    case "joint-grid":
      return buildJointGrid(sim, controls);
    case "smash":
      return buildSmash(sim, controls);
    case "large-compounds":
      return buildLargeCompounds(sim, controls);
    case "kinematic":
      return buildKinematic(sim, controls);
    case "cast":
      return buildCast(sim, controls);
    case "spinner":
      return buildSpinner(sim, controls);
    case "rain":
      return buildRain(sim, controls);
    case "shape-distance":
      return buildShapeDistance(sim, controls, wasm);
    case "sensor":
      return buildSensor(sim, controls);
    case "capacity":
      return buildCapacityFull(sim, controls);
    case "junkyard":
      return buildJunkyard(sim, controls);
  }
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Benchmark",
    "C <code>sample_benchmark.cpp</code> / <code>shared/benchmarks.c</code> ports. " +
      "DEBUG/wasm body counts disclosed where noted.",
    "Drag to grab · P pause · O step · R restart",
    { category: "Benchmark", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "large-pyramid";

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
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera, scene);
    runtime = buildScene(scene, sim, sceneControls, wasm);
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
        history.replaceState(null, "", `#/benchmark/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "benchmark",
    category: "Benchmark",
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

  const stop = runSampleLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    runtime.beforeStep?.(dt);
    const t0 = performance.now();
    sim.step(dt, transport.subSteps);
    const stepMs = performance.now() - t0;
    runtime.afterStep?.(dt);

    paintSampleDraw(canvas, camera, sim);
    const ctx = canvas.getContext("2d");
    if (ctx) runtime.paintOverlay?.(ctx, camera, canvas);

    updateReadout(readout, [
      { label: "Sample", value: SCENE_LABEL[scene] },
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
      { label: "step ms", value: stepMs.toFixed(2) },
      { label: "Hz", value: String(transport.hertz) },
      { label: "Paused", value: transport.paused ? "yes" : "no" },
      ...(runtime.readoutExtra?.() ?? []),
    ]);
  }, { chrome, transport, camera, readout, getWorld: () => sim });

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
