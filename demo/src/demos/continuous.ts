// Continuous — RegisterSample ports from sample_continuous.cpp.
// C citations use sample_continuous.cpp line numbers at the pinned submodule.

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

/** Registry scene keys — Bounce Humans uses CreateHuman. */
export const SCENES = [
  "bounce-house",
  "bounce-humans",
  "chain-drop",
  "chain-slide",
  "segment-slide",
  "skinny-box",
  "ghost-bumps",
  "speculative-fallback",
  "speculative-sliver",
  "speculative-ghost",
  "pixel-imperfect",
  "restitution-threshold",
  "drop",
  "pinball",
  "wedge",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("continuous", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "bounce-house": "Bounce House",
  "bounce-humans": "Bounce Humans",
  "chain-drop": "Chain Drop",
  "chain-slide": "Chain Slide",
  "segment-slide": "Segment Slide",
  "skinny-box": "Skinny Box",
  "ghost-bumps": "Ghost Bumps",
  "speculative-fallback": "Speculative Fallback",
  "speculative-sliver": "Speculative Sliver",
  "speculative-ghost": "Speculative Ghost",
  "pixel-imperfect": "Pixel Imperfect",
  "restitution-threshold": "Restitution Threshold",
  drop: "Drop",
  pinball: "Pinball",
  wedge: "Wedge",
};

/** C camera.center / camera.zoom (half-height). */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "bounce-house": { cx: 0.0, cy: 0.0, zoom: 25.0 * 0.45 }, // :40-41
  "bounce-humans": { cx: 0.0, cy: 0.0, zoom: 12.0 }, // :198-199
  "chain-drop": { cx: 0.0, cy: 0.0, zoom: 25.0 * 0.35 }, // :283-284
  "chain-slide": { cx: 0.0, cy: 10.0, zoom: 15.0 }, // :374-375
  "segment-slide": { cx: 0.0, cy: 10.0, zoom: 15.0 }, // :459-460
  "skinny-box": { cx: 1.0, cy: 5.0, zoom: 25.0 * 0.25 }, // :514-515
  "ghost-bumps": { cx: 1.5, cy: 16.0, zoom: 25.0 * 0.8 }, // :645-646
  "speculative-fallback": { cx: 1.0, cy: 5.0, zoom: 25.0 * 0.25 }, // :929-930
  "speculative-sliver": { cx: 0.0, cy: 1.75, zoom: 2.5 }, // :978-979
  "speculative-ghost": { cx: 0.0, cy: 1.75, zoom: 2.0 }, // :1023-1024
  "pixel-imperfect": { cx: 7.0, cy: 5.0, zoom: 6.0 }, // :1072-1073
  "restitution-threshold": { cx: 7.0, cy: 5.0, zoom: 6.0 }, // :1138-1139
  drop: { cx: 0.0, cy: 1.5, zoom: 3.0 }, // :1208-1209
  pinball: { cx: 0.0, cy: 9.0, zoom: 25.0 * 0.5 }, // :1548-1549
  wedge: { cx: 0.0, cy: 5.5, zoom: 6.0 }, // :1722-1723
};

const BODY_STATIC = 0;
const BODY_DYNAMIC = 2;
const FRIC = 0.6;
const PI = Math.PI;

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  paintOverlay?: (ctx: CanvasRenderingContext2D, camera: SampleCamera, canvas: HTMLCanvasElement) => void;
  readoutExtra?: () => { label: string; value: string }[];
  dispose?: () => void;
  onKeyDown?: (e: KeyboardEvent) => void;
  onKeyUp?: (e: KeyboardEvent) => void;
}

/** C Pinball motor speeds: A pressed → ±20, else ∓10 (sample_continuous.cpp:1688-1697). */
export function pinballMotorSpeeds(aPressed: boolean): { left: number; right: number } {
  return aPressed ? { left: 20, right: -20 } : { left: -10, right: 10 };
}

/** True when the event is physical A (C GLFW_KEY_A), including Caps Lock / empty key. */
export function isPinballFlipperKey(e: Pick<KeyboardEvent, "code" | "key">): boolean {
  if (e.code === "KeyA") return true;
  return typeof e.key === "string" && e.key.toLowerCase() === "a";
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

/** C XorShift RandomFloatRange (utils.h) seeded at RAND_SEED 12345. */
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

function addRoomSegments(sim: SimWorld, friction: number, _restitution: number) {
  // sample_continuous.cpp Bounce House / Bounce Humans walls
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment_mat(g, -10, -10, 10, -10, friction);
  sim.attach_segment_mat(g, 10, -10, 10, 10, friction);
  sim.attach_segment_mat(g, 10, 10, -10, 10, friction);
  sim.attach_segment_mat(g, -10, 10, -10, -10, friction);
  return g;
}

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

function buildBounceHouse(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:18-188
  let shapeType: "circle" | "capsule" | "box" = "circle";
  let enableHit = true;
  let bodyId = -1;
  const hits: { x: number; y: number; speed: number; step: number }[] = [
    { x: 0, y: 0, speed: 0, step: 0 },
    { x: 0, y: 0, speed: 0, step: 0 },
    { x: 0, y: 0, speed: 0, step: 0 },
    { x: 0, y: 0, speed: 0, step: 0 },
  ];
  let stepCount = 0;

  addRoomSegments(sim, FRIC, 0);

  function launch() {
    if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.destroy_body(bodyId);
    // :84-94 bullet, gravity 0, allowFastRotation for circle
    bodyId = sim.add_body_ccd(0, 0, 0, BODY_DYNAMIC, 0, true, shapeType === "circle", true);
    sim.set_linear_velocity(bodyId, 10, 20);
    if (shapeType === "circle") {
      sim.attach_circle_hit(bodyId, 0, 0, 0.5, 1.0, 0, 1.0, 0);
    } else if (shapeType === "capsule") {
      sim.attach_capsule_mat(bodyId, -0.5, 0, 0.5, 0, 0.25, 1.0, 0, 1.0, 0, 0);
      sim.enable_body_hit_events(bodyId, enableHit);
    } else {
      const h = 0.1;
      sim.attach_box_mat(bodyId, 20 * h, h, 0, 0, 0, 1.0, 0, 1.0, 0, 0);
      sim.enable_body_hit_events(bodyId, enableHit);
    }
    if (shapeType === "circle") sim.enable_body_hit_events(bodyId, enableHit);
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
  controls.appendChild(
    createCheckbox("hit events", enableHit, (v) => {
      enableHit = v;
      if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.enable_body_hit_events(bodyId, enableHit);
    }),
  );

  return {
    afterStep: () => {
      stepCount += 1;
      const ev = sim.hit_events();
      for (let i = 0; i + 2 < ev.length; i += 3) {
        let slot = hits[0]!;
        for (let j = 1; j < 4; j++) {
          if (hits[j]!.step < slot.step) slot = hits[j]!;
        }
        slot.x = ev[i]!;
        slot.y = ev[i + 1]!;
        slot.speed = ev[i + 2]!;
        slot.step = stepCount;
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      for (const e of hits) {
        if (e.step > 0 && stepCount <= e.step + 30) {
          const p = worldToScreen(camera, canvas, e.x, e.y);
          ctx.beginPath();
          ctx.arc(p.x, p.y, 4, 0, 2 * PI);
          ctx.fillStyle = "#ff4500";
          ctx.fill();
          ctx.fillStyle = "#fff";
          ctx.font = "12px sans-serif";
          ctx.fillText(e.speed.toFixed(1), p.x + 6, p.y - 4);
        }
      }
    },
  };
}

function buildBounceHumans(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:192-273 Bounce Humans
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  const wallRest = 1.3;
  const wallFric = 0.1;
  for (const [x1, y1, x2, y2] of [
    [-10, -10, 10, -10],
    [10, -10, 10, 10],
    [10, 10, -10, 10],
    [-10, 10, -10, -10],
  ] as const) {
    const sh = sim.attach_segment_mat(g, x1, y1, x2, y2, wallFric);
    sim.shape_set_restitution(sh, wallRest);
  }
  // :228-230 center circle restitution 2
  sim.attach_circle_mat(g, 0, 0, 2.0, 0, wallFric, 2.0, 0, 0);

  let humanCount = 0;
  let countDown = 0;
  let time = 0;
  let gravX = 0;
  let gravY = -10;

  controls.appendChild(
    createInfoBox(
      "Exact: up to 5 <code>CreateHuman</code> ragdolls; gravity rotates with time. " +
        "C <code>sample_continuous.cpp</code> Bounce Humans.",
    ),
  );

  return {
    beforeStep: (dt) => {
      if (humanCount < 5 && countDown <= 0) {
        // :237-242
        sim.create_human(0, 5, 1.0, 0.0, 1.0, 0.1, 1, true, 0);
        countDown = 2.0;
        humanCount += 1;
      }
      const cs1 = Math.sin(0.5 * time);
      const cs2 = Math.cos(time);
      const gravity = 10.0;
      gravX = gravity * cs1;
      gravY = gravity * cs2;
      sim.set_gravity(gravX, gravY);
      time += dt;
      countDown -= dt;
    },
    paintOverlay: (ctx, camera, canvas) => {
      // :254 DrawLine origin → 3*(sin, cos) gravity indicator
      const cs1 = Math.sin(0.5 * time);
      const cs2 = Math.cos(time);
      const a = worldToScreen(camera, canvas, 0, 0);
      const b = worldToScreen(camera, canvas, 3 * cs1, 3 * cs2);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    },
    readoutExtra: () => [
      { label: "humans", value: String(humanCount) },
      { label: "g.x", value: gravX.toFixed(2) },
      { label: "g.y", value: gravY.toFixed(2) },
    ],
  };
}

function buildChainDrop(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:275-362
  let bodyId = -1;
  let yOffset = -0.1;
  let speed = -42.0;

  const ground = sim.add_body(0, -6, 0, BODY_STATIC);
  sim.attach_chain(ground, [-10, -2, 10, -2, 10, 1, -10, 1], true);

  function launch() {
    if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.destroy_body(bodyId);
    bodyId = sim.add_body(0, 10 + yOffset, 0.5 * PI, BODY_DYNAMIC);
    sim.set_linear_velocity(bodyId, 0, speed);
    sim.set_motion_locks(bodyId, false, false, true);
    sim.attach_circle(bodyId, 0, 0, 0.5, 1.0, FRIC, 0);
  }

  launch();

  controls.appendChild(
    createSlider("Speed", -100, 0, speed, 1, (v) => {
      speed = v;
    }),
  );
  controls.appendChild(
    createSlider("Y Offset", -1, 1, yOffset, 0.1, (v) => {
      yOffset = v;
    }),
  );
  controls.appendChild(createButton("Launch", () => launch()));

  return {};
}

function buildChainSlide(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:366-447
  const count = 80;
  const points: number[] = [];
  const w = 2.0;
  const h = 1.0;
  let x = 20.0;
  let y = 0.0;
  for (let i = 0; i < 20; i++) {
    points.push(x, y);
    x -= w;
  }
  for (let i = 20; i < 40; i++) {
    points.push(x, y);
    y += h;
  }
  for (let i = 40; i < 60; i++) {
    points.push(x, y);
    x += w;
  }
  for (let i = 60; i < 80; i++) {
    points.push(x, y);
    y -= h;
  }
  void count;
  sim.add_chain(points, true);

  const ball = sim.add_body(-19.5, 0.5, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(ball, 100, 0);
  sim.attach_circle(ball, 0, 0, 0.5, 1.0, 0, 0);
  return {};
}

function buildSegmentSlide(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:451-502
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(g, -40, 0, 40, 0);
  sim.attach_segment(g, 40, 0, 40, 10);

  const ball = sim.add_body(-20, 0.7, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(ball, 100, 0);
  sim.attach_circle(ball, 0, 0, 0.5, 1.0, FRIC, 0);
  return {};
}

function buildSkinnyBox(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:506-625
  const rng = makeRng();
  let capsule = false;
  let autoTest = false;
  let bodyId = -1;
  let bulletId = -1;
  let stepCount = 0;

  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment_mat(ground, -10, 0, 10, 0, 0.9);
  sim.attach_box(ground, 0.1, 1.0, 0, 1.0, 0, 0, 0.9, 0);

  function launch() {
    if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.destroy_body(bodyId);
    if (bulletId >= 0 && sim.is_body_alive(bulletId)) sim.destroy_body(bulletId);
    bulletId = -1;

    const omega = rng.floatRange(-50, 50);
    bodyId = sim.add_body(0, 8, 0, BODY_DYNAMIC);
    sim.set_angular_velocity(bodyId, omega);
    sim.set_linear_velocity(bodyId, 0, -100);
    if (capsule) {
      sim.attach_capsule(bodyId, 0, -1, 0, 1, 0.1, 1.0, 0.9, 0);
    } else {
      sim.attach_box(bodyId, 2.0, 0.05, 0, 0, 0, 1.0, 0.9, 0);
    }
  }

  launch();

  controls.appendChild(
    createCheckbox("Capsule", capsule, (v) => {
      capsule = v;
    }),
  );
  controls.appendChild(createButton("Launch", () => launch()));
  controls.appendChild(
    createCheckbox("Auto Test", autoTest, (v) => {
      autoTest = v;
    }),
  );

  return {
    afterStep: () => {
      stepCount += 1;
      if (autoTest && stepCount % 60 === 0) launch();
    },
  };
}

function ghostChainPoints(): number[] {
  // sample_continuous.cpp:674-702
  const m = 1.0 / Math.sqrt(2);
  const mm = 2.0 * (Math.sqrt(2) - 1);
  const hx = 4.0;
  const hy = 0.25;
  const pts: { x: number; y: number }[] = [];
  pts[0] = { x: -3 * hx, y: hy };
  const add = (i: number, dx: number, dy: number) => {
    pts[i] = { x: pts[i - 1]!.x + dx, y: pts[i - 1]!.y + dy };
  };
  add(1, -2 * hx * m, 2 * hx * m);
  add(2, -2 * hx * m, 2 * hx * m);
  add(3, -2 * hx * m, 2 * hx * m);
  add(4, -2 * hy * m, -2 * hy * m);
  add(5, 2 * hx * m, -2 * hx * m);
  add(6, 2 * hx * m, -2 * hx * m);
  add(7, 2 * hx * m + 2 * hy * (1 - m), -2 * hx * m - 2 * hy * (1 - m));
  add(8, 2 * hx + hy * mm, 0);
  add(9, 2 * hx, 0);
  add(10, 2 * hx + hy * mm, 0);
  add(11, 2 * hx * m + 2 * hy * (1 - m), 2 * hx * m + 2 * hy * (1 - m));
  add(12, 2 * hx * m, 2 * hx * m);
  add(13, 2 * hx * m, 2 * hx * m);
  add(14, -2 * hy * m, 2 * hy * m);
  add(15, -2 * hx * m, -2 * hx * m);
  add(16, -2 * hx * m, -2 * hx * m);
  add(17, -2 * hx * m, -2 * hx * m);
  add(18, -2 * hx, 0);
  add(19, -2 * hx, 0);
  const out: number[] = [];
  for (const p of pts) out.push(p.x, p.y);
  return out;
}

function buildGhostBumps(sim: SimWorld, controls: HTMLElement, rebuild: () => void, state: GhostState): SceneRuntime {
  // sample_continuous.cpp:629-915
  let bodyId = -1;
  let shapeId = -1;

  function createScene() {
    const ground = sim.add_body(0, 0, 0, BODY_STATIC);
    if (state.useChain) {
      sim.add_chain_mat(ghostChainPoints(), true, [state.friction, 0, 0, 0]);
    } else {
      const hx = 4.0;
      const hy = 0.25;
      const m = 1.0 / Math.sqrt(2);
      const bevel = state.bevel;
      let local: number[];
      if (bevel > 0) {
        const hb = bevel;
        local = [
          hx + hb,
          hy - 0.05,
          hx,
          hy,
          -hx,
          hy,
          -hx - hb,
          hy - 0.05,
          -hx - hb,
          -hy + 0.05,
          -hx,
          -hy,
          hx,
          -hy,
          hx + hb,
          -hy + 0.05,
        ];
      } else {
        local = [hx, hy, -hx, hy, -hx, -hy, hx, -hy];
      }

      const place = (px: number, py: number, angle: number) => {
        const c = Math.cos(angle);
        const s = Math.sin(angle);
        const pts: number[] = [];
        for (let i = 0; i < local.length; i += 2) {
          const lx = local[i]!;
          const ly = local[i + 1]!;
          pts.push(px + c * lx - s * ly, py + s * lx + c * ly);
        }
        sim.attach_polygon(ground, pts, 0, 0, state.friction, 0);
      };

      let x = -3 * hx - m * hx - m * hy;
      let y = hy + m * hx - m * hy;
      const qNeg = -0.25 * PI;
      for (let i = 0; i < 3; i++) {
        place(x, y, qNeg);
        x -= 2 * m * hx;
        y += 2 * m * hx;
      }
      x = -2 * hx;
      y = 0;
      for (let i = 0; i < 3; i++) {
        place(x, y, 0);
        x += 2 * hx;
      }
      x = 3 * hx + m * hx + m * hy;
      y = hy + m * hx - m * hy;
      const qPos = 0.25 * PI;
      for (let i = 0; i < 3; i++) {
        place(x, y, qPos);
        x += 2 * m * hx;
        y += 2 * m * hx;
      }
    }
  }

  function launch() {
    if (bodyId >= 0 && sim.is_body_alive(bodyId)) sim.destroy_body(bodyId);
    bodyId = sim.add_body(-28, 18, 0, BODY_DYNAMIC);
    if (state.shapeType === "circle") {
      shapeId = sim.attach_circle_mat(bodyId, 0, 0, 0.5, 1.0, state.friction, 0, 0, 0);
    } else if (state.shapeType === "capsule") {
      shapeId = sim.attach_capsule_mat(bodyId, -0.5, 0, 0.5, 0, 0.25, 1.0, state.friction, 0, 0, 0);
    } else {
      const h = 0.5 - state.round;
      shapeId = sim.attach_rounded_box_mat(bodyId, h, 2 * h, state.round, 1.0, state.friction, 0, 0, 0);
    }
  }

  createScene();
  launch();

  controls.appendChild(
    createCheckbox("Chain", state.useChain, (v) => {
      state.useChain = v;
      rebuild();
    }),
  );
  if (!state.useChain) {
    controls.appendChild(
      createSlider("Bevel", 0, 1, state.bevel, 0.01, (v) => {
        state.bevel = v;
        rebuild();
      }),
    );
  }
  controls.appendChild(
    createDropdown(
      "Shape",
      [
        { value: "circle", text: "Circle" },
        { value: "capsule", text: "Capsule" },
        { value: "box", text: "Box" },
      ],
      state.shapeType,
      (v) => {
        state.shapeType = v as GhostState["shapeType"];
        launch();
      },
    ),
  );
  if (state.shapeType === "box") {
    controls.appendChild(
      createSlider("Round", 0, 0.4, state.round, 0.1, (v) => {
        state.round = v;
      }),
    );
  }
  controls.appendChild(
    createSlider("Friction", 0, 1, state.friction, 0.1, (v) => {
      state.friction = v;
      if (shapeId >= 0) sim.shape_set_friction(shapeId, v);
      rebuild();
    }),
  );
  controls.appendChild(createButton("Launch", () => launch()));

  return {};
}

type GhostState = {
  useChain: boolean;
  bevel: number;
  shapeType: "circle" | "capsule" | "box";
  round: number;
  friction: number;
};

function buildSpeculativeFallback(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:921-966
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(g, -10, 0, 10, 0);
  sim.attach_polygon(g, [-2, 4, 2, 4, 2, 4.1, -0.5, 4.2, -2, 4.2], 0, 0, FRIC, 0);

  const offset = 8.0;
  const body = sim.add_body(offset, 12, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(body, 0, -100);
  sim.attach_box(body, 2.0, 0.05, -offset, 0, PI, 1.0, FRIC, 0);
  return {};
}

function buildSpeculativeSliver(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:970-1010
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(g, -10, 0, 10, 0);

  const body = sim.add_body(0, 12, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(body, 0, -100);
  sim.attach_polygon(body, [-2, 0, -1, 0, 2, 0.5], 0, 1.0, FRIC, 0);
  return {};
}

function buildSpeculativeGhost(sim: SimWorld, _controls: HTMLElement, hertz: number): SceneRuntime {
  // sample_continuous.cpp:1014-1059
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(g, -10, 0, 10, 0);
  sim.attach_box(g, 1.0, 0.1, 0, 0.9, 0, 0, FRIC, 0);

  const body = sim.add_body_ex(0.015, 2.515, 0, BODY_DYNAMIC, 0, true);
  const s = 0.1 * 1.25 * hertz;
  sim.set_linear_velocity(body, s, -s);
  sim.attach_box(body, 0.25, 0.25, 0, 0, 0, 1.0, FRIC, 0);
  return {};
}

function buildPixelImperfect(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:1063-1126
  const ppm = 30;
  const block = sim.add_body(175 / ppm, 150 / ppm, 0, BODY_STATIC);
  sim.attach_box(block, 20 / ppm, 10 / ppm, 0, 0, 0, 0, 0, 0);

  const ball = sim.add_body_ex(200 / ppm, 275 / ppm, 0, BODY_DYNAMIC, 0, true);
  sim.attach_rounded_box(ball, 4 / ppm, 4 / ppm, 0.9 / ppm, 1.0, 0, 0);
  sim.set_linear_velocity(ball, 0, -5);
  sim.set_motion_locks(ball, false, false, true);

  return {
    readoutExtra: () => {
      const p = sim.positions();
      const v = sim.get_linear_velocity(ball);
      return [
        { label: "p.x", value: p[ball * 3]!.toFixed(9) },
        { label: "v.y", value: v[1]!.toFixed(9) },
      ];
    },
  };
}

function buildRestitutionThreshold(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:1130-1196
  const ppm = 30;
  sim.set_restitution_threshold(0.1);

  const block = sim.add_body(205 / ppm, 120 / ppm, (70 * 3.14) / 180, BODY_STATIC);
  sim.attach_box(block, 50 / ppm, 5 / ppm, 0, 0, 0, 0, 0, 0);

  const ball = sim.add_body(200 / ppm, 250 / ppm, 0, BODY_DYNAMIC);
  sim.attach_circle(ball, 0, 0, 5 / ppm, 1.0, 0, 1.0);
  sim.set_linear_velocity(ball, 0, -2.9);
  sim.set_motion_locks(ball, false, false, true);

  return {
    readoutExtra: () => {
      const p = sim.positions();
      const v = sim.get_linear_velocity(ball);
      return [
        { label: "p.x", value: p[ball * 3]!.toFixed(9) },
        { label: "v.y", value: v[1]!.toFixed(9) },
        { label: "rest thresh", value: sim.get_restitution_threshold().toFixed(2) },
      ];
    },
  };
}

type DropState = {
  continuous: boolean;
  speculative: boolean;
  frameSkip: number;
  subScene: 1 | 2 | 3 | 4;
};

function buildDrop(sim: SimWorld, controls: HTMLElement, state: DropState, rebuild: () => void): SceneRuntime {
  // sample_continuous.cpp:1200-1534 — Scenes 1–4 including CreateHuman ragdoll Scene3
  sim.set_continuous(state.continuous);
  sim.set_speculative(state.speculative);
  sim.set_sleeping(false);

  let frameCount = 0;

  function ground1() {
    const g = sim.add_body(0, 0, 0, BODY_STATIC);
    const w = 0.25;
    const count = 40;
    sim.attach_segment(g, -0.5 * count * w, 0, 0.5 * count * w, 0);
  }
  function ground2() {
    const g = sim.add_body(0, 0, 0, BODY_STATIC);
    const w = 0.25;
    const count = 40;
    let x = -0.5 * count * w;
    const h = 0.05;
    for (let j = 0; j <= count; j++) {
      sim.attach_box(g, 0.5 * w, h, x, 0, 0, 0, FRIC, 0);
      x += w;
    }
  }
  function ground3() {
    const g = sim.add_body(0, 0, 0, BODY_STATIC);
    const w = 0.25;
    const count = 40;
    sim.attach_segment(g, -0.5 * count * w, 0, 0.5 * count * w, 0);
    sim.attach_segment(g, 3, 0, 3, 8);
  }

  if (state.subScene === 1) {
    ground2();
    const b = sim.add_body(0, 4, 0, BODY_DYNAMIC);
    sim.set_linear_velocity(b, 0, -100);
    sim.attach_circle(b, 0, 0, 0.125, 1.0, FRIC, 0);
  } else if (state.subScene === 2) {
    ground1();
    const b = sim.add_body(0, 4, 0.5 * PI, BODY_DYNAMIC);
    sim.set_angular_velocity(b, -0.5);
    sim.attach_box(b, 0.75, 0.01, 0, 0, 0, 1.0, FRIC, 0);
  } else if (state.subScene === 3) {
    // :1375-1388 ragdoll
    ground2();
    sim.create_human(0, 40, 1.0, 0.03, 1.0, 0.5, 1, true, 0);
  } else {
    ground3();
    const a = 0.25;
    const offset = 0.01;
    for (let i = 0; i < 5; i++) {
      const shift = i % 2 === 0 ? -offset : offset;
      const b = sim.add_body(2.5 + shift, a + 2 * a * i, 0, BODY_DYNAMIC);
      sim.attach_box(b, a, a, 0, 0, 0, 1.0, FRIC, 0);
    }
    const bullet = sim.add_body_ccd(-7.7, 1.9, 0, BODY_DYNAMIC, 1, true, false, true);
    sim.set_linear_velocity(bullet, 200, 0);
    sim.attach_circle(bullet, 0, 0, 0.125, 4.0, FRIC, 0);
  }

  controls.appendChild(
    createInfoBox(
      "Keys: <strong>1</strong> ball · <strong>2</strong> ruler · <strong>3</strong> ragdoll · " +
        "<strong>4</strong> stack+bullet · <strong>C</strong> continuous · <strong>V</strong> speculative · " +
        "<strong>S</strong> slow. Exact Scene3 via <code>CreateHuman</code>.",
    ),
  );
  controls.appendChild(
    createDropdown(
      "Scene",
      [
        { value: "1", text: "1 Ball" },
        { value: "2", text: "2 Ruler" },
        { value: "3", text: "3 Ragdoll" },
        { value: "4", text: "4 Stack+Bullet" },
      ],
      String(state.subScene),
      (v) => {
        state.subScene = Number(v) as 1 | 2 | 3 | 4;
        rebuild();
      },
    ),
  );
  controls.appendChild(
    createCheckbox("Continuous", state.continuous, (v) => {
      state.continuous = v;
      sim.set_continuous(v);
    }),
  );
  controls.appendChild(
    createCheckbox("Speculative", state.speculative, (v) => {
      state.speculative = v;
      sim.set_speculative(v);
    }),
  );

  return {
    beforeStep: () => {
      sim.set_continuous(state.continuous);
    },
    afterStep: () => {
      frameCount += 1;
    },
    onKeyDown: (e) => {
      const k = e.key.toLowerCase();
      if (k === "1") {
        state.subScene = 1;
        rebuild();
      } else if (k === "2") {
        state.subScene = 2;
        rebuild();
      } else if (k === "3") {
        state.subScene = 3;
        rebuild();
      } else if (k === "4") {
        state.subScene = 4;
        rebuild();
      } else if (k === "c") {
        state.continuous = !state.continuous;
        rebuild();
      } else if (k === "v") {
        state.speculative = !state.speculative;
        rebuild();
      } else if (k === "s") {
        state.frameSkip = state.frameSkip > 0 ? 0 : 60;
      }
    },
    readoutExtra: () => [
      { label: "Continuous", value: state.continuous && state.speculative ? "ON" : "OFF" },
      { label: "Slow", value: state.frameSkip > 0 ? "yes" : "no" },
      { label: "Frame", value: String(frameCount) },
    ],
  };
}

function buildPinball(sim: SimWorld, controls: HTMLElement, keysDown: Set<string>): SceneRuntime {
  // sample_continuous.cpp:1538-1708
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_chain(ground, [-8, 6, -8, 20, 8, 20, 8, 6, 0, -2], true);

  const left = sim.add_body_ex(-2, 0, 0, BODY_DYNAMIC, 1, false);
  const right = sim.add_body_ex(2, 0, 0, BODY_DYNAMIC, 1, false);
  sim.attach_box(left, 1.75, 0.2, 0, 0, 0, 1.0, FRIC, 0);
  sim.attach_box(right, 1.75, 0.2, 0, 0, 0, 1.0, FRIC, 0);

  const deg = PI / 180;
  const leftJ = sim.add_revolute_joint_local(
    ground,
    left,
    -2,
    0,
    0,
    0,
    true,
    -30 * deg,
    5 * deg,
    true,
    0,
    1000,
    false,
    0,
    0,
    false,
  );
  const rightJ = sim.add_revolute_joint_local(
    ground,
    right,
    2,
    0,
    0,
    0,
    true,
    -5 * deg,
    30 * deg,
    true,
    0,
    1000,
    false,
    0,
    0,
    false,
  );

  // Spinners
  for (const [sx, sy] of [
    [-4, 17],
    [4, 8],
  ] as const) {
    const sp = sim.add_body(sx, sy, 0, BODY_DYNAMIC);
    sim.attach_box(sp, 1.5, 0.125, 0, 0, 0, 1.0, FRIC, 0);
    sim.attach_box(sp, 0.125, 1.5, 0, 0, 0, 1.0, FRIC, 0);
    sim.add_revolute_joint_local(ground, sp, sx, sy, 0, 0, false, 0, 0, true, 0, 0.1, false, 0, 0, false);
  }

  // Bumpers
  for (const [bx, by] of [
    [-4, 8],
    [4, 17],
  ] as const) {
    const b = sim.add_body(bx, by, 0, BODY_STATIC);
    sim.attach_circle(b, 0, 0, 1.0, 0, FRIC, 1.5);
  }

  const ball = sim.add_body_ccd(1, 15, 0, BODY_DYNAMIC, 1, true, false, true);
  sim.attach_circle(ball, 0, 0, 0.2, 1.0, FRIC, 0);

  controls.appendChild(createInfoBox("Flipper: hold <strong>A</strong>"));

  // C Pinball::Step polls glfwGetKey(A) every frame after Sample::Step
  // (sample_continuous.cpp:1688-1697). Poll page-level key state so a rebuild
  // (R) while A is held still drives flippers — edge-triggered scene state
  // would reset to "released" until the next keydown.
  return {
    beforeStep: () => {
      const speeds = pinballMotorSpeeds(keysDown.has("KeyA"));
      sim.revolute_set_motor_speed(leftJ, speeds.left);
      sim.revolute_set_motor_speed(rightJ, speeds.right);
    },
    readoutExtra: () => [
      {
        label: "Flippers",
        value: keysDown.has("KeyA") ? "A held (+20/-20)" : "rest (-10/+10)",
      },
    ],
  };
}

function buildWedge(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_continuous.cpp:1712-1756
  const g = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(g, -4, 8, 0, 0);
  sim.attach_segment(g, 0, 0, 0, 8);

  const ball = sim.add_body(-0.45, 10.75, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(ball, 0, -200);
  sim.attach_circle(ball, 0, 0, 0.3, 1.0, 0.2, 0);
  return {};
}

type PageState = {
  ghost: GhostState;
  drop: DropState;
};

function freshState(): PageState {
  return {
    ghost: { useChain: true, bevel: 0, shapeType: "circle", round: 0, friction: 0.2 },
    drop: { continuous: true, speculative: true, frameSkip: 0, subScene: 1 },
  };
}

function buildScene(
  scene: Scene,
  sim: SimWorld,
  controls: HTMLElement,
  state: PageState,
  rebuild: () => void,
  hertz: number,
  keysDown: Set<string>,
): SceneRuntime {
  controls.replaceChildren();
  switch (scene) {
    case "bounce-house":
      return buildBounceHouse(sim, controls);
    case "bounce-humans":
      return buildBounceHumans(sim, controls);
    case "chain-drop":
      return buildChainDrop(sim, controls);
    case "chain-slide":
      return buildChainSlide(sim, controls);
    case "segment-slide":
      return buildSegmentSlide(sim, controls);
    case "skinny-box":
      return buildSkinnyBox(sim, controls);
    case "ghost-bumps":
      return buildGhostBumps(sim, controls, rebuild, state.ghost);
    case "speculative-fallback":
      return buildSpeculativeFallback(sim, controls);
    case "speculative-sliver":
      return buildSpeculativeSliver(sim, controls);
    case "speculative-ghost":
      return buildSpeculativeGhost(sim, controls, hertz);
    case "pixel-imperfect":
      return buildPixelImperfect(sim, controls);
    case "restitution-threshold":
      return buildRestitutionThreshold(sim, controls);
    case "drop":
      return buildDrop(sim, controls, state.drop, rebuild);
    case "pinball":
      return buildPinball(sim, controls, keysDown);
    case "wedge":
      return buildWedge(sim, controls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Continuous",
    "C <code>sample_continuous.cpp</code> RegisterSample ports — CCD, speculative " +
      "collision, ghost bumps, Bounce Humans, pinball, and restitution threshold.",
    "Drag to grab · P pause · O step · R restart · A flippers (Pinball)",
    { category: "Continuous", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "bounce-house";

  const camera: SampleCamera = makeCamera();
  applyCamera(camera, scene);
  const transport = createSampleTransport();
  const state = freshState();
  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = {};
  let dropGate = 0;

  const sceneControls = document.createElement("div");
  sceneControls.className = "scene-controls";

  // Page-level key set survives scene rebuild (C polls hardware each Step).
  const keysDown = new Set<string>();
  canvas.tabIndex = 0;

  function rebuild() {
    runtime.dispose?.();
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera, scene);
    runtime = buildScene(scene, sim, sceneControls, state, rebuild, transport.hertz, keysDown);
    dropGate = 0;
  }

  rebuild();

  let grabbing = false;
  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    canvas.focus();
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

  const onKeyDown = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement | null)?.tagName;
    if (tag !== "INPUT" && tag !== "TEXTAREA") {
      if (isPinballFlipperKey(e)) keysDown.add("KeyA");
      keysDown.add(e.code);
    }
    runtime.onKeyDown?.(e);
  };
  const onKeyUp = (e: KeyboardEvent) => {
    if (isPinballFlipperKey(e)) keysDown.delete("KeyA");
    keysDown.delete(e.code);
    runtime.onKeyUp?.(e);
  };
  const onWindowBlur = () => keysDown.clear();
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  window.addEventListener("blur", onWindowBlur);

  controls.appendChild(
    createDropdown(
      "Sample",
      SCENES.map((s) => ({ value: s, text: SCENE_LABEL[s] })),
      scene,
      (v) => {
        scene = v as Scene;
        history.replaceState(null, "", `#/continuous/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "continuous",
    category: "Continuous",
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
    // Drop frame-skip (C m_frameSkip): advance physics only every N frames.
    let doStep = true;
    if (scene === "drop" && state.drop.frameSkip > 0) {
      dropGate += 1;
      doStep = dropGate % state.drop.frameSkip === 0;
    }
    const dt = doStep ? transport.consumeStepDt() : 0;
    if (doStep) {
      runtime.beforeStep?.(dt);
      sim.step(dt, transport.subSteps);
      runtime.afterStep?.(dt);
    }

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
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    window.removeEventListener("blur", onWindowBlur);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    freeSim(sim);
  };
}
