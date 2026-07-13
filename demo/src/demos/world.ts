// World — RegisterSample ports from sample_world.cpp.
// C citations use sample_world.cpp line numbers at the pinned submodule.

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
import { paintDebugDraw } from "./debug-draw.ts";
import { demoPage, fitCanvas, freeSim, runSimLoop } from "./sim-common.ts";
import {
  createSampleTransport,
  DEFAULT_SUB_STEPS,
  disposeTransport,
  makeCamera,
  screenToWorld,
  viewBounds,
  worldToScreen,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — all four C World samples. */
export const SCENES = ["tiles", "far-pyramid", "far-ragdolls", "far-gate"] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("world", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  tiles: "Tiles",
  "far-pyramid": "Far Pyramid",
  "far-ragdolls": "Far Ragdolls",
  "far-gate": "Far Gate",
};

/** C camera.center / camera.zoom (half-height). Far scenes use absolute origins. */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  // TileWorld sets viewPosition then camera.center = viewPosition; zoom 25 (:31-36)
  tiles: { cx: 0, cy: 15, zoom: 25.0 },
  // FarPyramid :261-262 — origin 10e6 + (0,12), zoom 17
  "far-pyramid": { cx: 10.0e6, cy: 12.0, zoom: 17.0 },
  // FarRagdolls :327-328 — origin 10e6 + (0,6), zoom 10
  "far-ragdolls": { cx: 10.0e6, cy: 6.0, zoom: 10.0 },
  // FarGate :410-411 — origin 1e6 + (0,6), zoom 7
  "far-gate": { cx: 1.0e6, cy: 6.0, zoom: 7.0 },
};

const BODY_STATIC = 0;
const BODY_DYNAMIC = 2;
const FRIC = 0.6;
const PI = Math.PI;

/** Browser DEBUG scale — C m_isDebug ? 10 : 600 (:25). */
const TILES_CYCLE_COUNT = 10;

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

function clearControls(el: HTMLElement) {
  el.replaceChildren();
}

/** C XorShift32 (utils.h RAND_SEED=12345). */
function makeXorShift(seed = 12345) {
  let s = seed >>> 0;
  const RAND_LIMIT = 32767;
  const next = () => {
    let x = s;
    x ^= (x << 13) >>> 0;
    x ^= x >>> 17;
    x ^= (x << 5) >>> 0;
    s = x >>> 0;
    return (s % (RAND_LIMIT + 1)) | 0;
  };
  const floatRange = (lo: number, hi: number) => {
    const r = (next() & RAND_LIMIT) / RAND_LIMIT;
    return (hi - lo) * r + lo;
  };
  const intRange = (lo: number, hi: number) => lo + (next() % (hi - lo + 1));
  return { next, floatRange, intRange };
}

/** C Sample::ParsePath (sample.cpp:1047) — Y-flipped scaled points. */
function parsePath(
  svgPath: string,
  offsetX: number,
  offsetY: number,
  capacity: number,
  scale: number,
): number[] {
  const points: number[] = [];
  let currentX = 0;
  let currentY = 0;
  let command = "";
  let ptr = 0;
  const s = svgPath;

  const skipSpaces = () => {
    while (ptr < s.length && /\s/.test(s[ptr]!)) ptr++;
  };
  const skipToken = () => {
    while (ptr < s.length && !/\s/.test(s[ptr]!)) ptr++;
  };
  const readFloat = (): number => {
    skipSpaces();
    const start = ptr;
    skipToken();
    return parseFloat(s.slice(start, ptr));
  };
  const readPair = (): [number, number] => {
    skipSpaces();
    const start = ptr;
    skipToken();
    const token = s.slice(start, ptr);
    const parts = token.split(",");
    return [parseFloat(parts[0]!), parseFloat(parts[1]!)];
  };

  skipSpaces();
  while (ptr < s.length) {
    const ch = s[ptr]!;
    if (!/[0-9.\-]/.test(ch)) {
      command = ch;
      if ("MLHVmlhv".includes(command)) {
        ptr += 2; // command + space (C :1064)
      }
      if (command === "z" || command === "Z") break;
      skipSpaces();
    }

    switch (command) {
      case "M":
      case "L": {
        const [x, y] = readPair();
        currentX = x;
        currentY = y;
        break;
      }
      case "H":
        currentX = readFloat();
        break;
      case "V":
        currentY = readFloat();
        break;
      case "m":
      case "l": {
        const [x, y] = readPair();
        currentX += x;
        currentY += y;
        break;
      }
      case "h":
        currentX += readFloat();
        break;
      case "v":
        currentY += readFloat();
        break;
      default:
        skipToken();
        break;
    }

    points.push(scale * (currentX + offsetX), -scale * (currentY + offsetY));
    if (points.length / 2 >= capacity) break;

    skipSpaces();
  }
  return points;
}

/** C RandomPolygon (utils.c:18) via attach_polygon_mat. */
function attachRandomPolygon(
  sim: SimWorld,
  body: number,
  extent: number,
  radius: number,
  density: number,
  friction: number,
  rolling: number,
  rng: ReturnType<typeof makeXorShift>,
) {
  const count = 3 + (rng.next() % 6);
  const pts: number[] = [];
  for (let i = 0; i < count; i++) {
    pts.push(rng.floatRange(-extent, extent), rng.floatRange(-extent, extent));
  }
  sim.attach_polygon_mat(body, pts, radius, density, friction, 0, rolling, 0);
}

/** C Car::Spawn (car.cpp:21) — scale/hertz/damping/torque. */
function spawnCar(
  sim: SimWorld,
  px: number,
  py: number,
  scale: number,
  hertz: number,
  damping: number,
  torque: number,
): { chassis: number; rearAxle: number; frontAxle: number } {
  const verts = [-1.5, -0.5, 1.5, -0.5, 1.5, 0, 0, 0.9, -1.15, 0.9, -1.5, 0.2].map(
    (v) => v * 0.85 * scale,
  );
  const chassis = sim.add_polygon(px, py + 1 * scale, 0, verts, 0.15 * scale, 1 / scale);
  const rear = sim.add_body(px + -1 * scale, py + 0.35 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(rear, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  const front = sim.add_body(px + 1 * scale, py + 0.4 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(front, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  // axis (0,1) → localFrameA.q = 0.5π (matches car.cpp :81)
  const rearAxle = sim.add_wheel_joint(
    chassis,
    rear,
    px + -1 * scale,
    py + 0.35 * scale,
    0,
    1,
    true,
    -0.25 * scale,
    0.25 * scale,
    true,
    0,
    torque,
    true,
    hertz,
    damping,
    false,
  );
  const frontAxle = sim.add_wheel_joint(
    chassis,
    front,
    px + 1 * scale,
    py + 0.4 * scale,
    0,
    1,
    true,
    -0.25 * scale,
    0.25 * scale,
    true,
    0,
    torque,
    true,
    hertz,
    damping,
    false,
  );
  return { chassis, rearAxle, frontAxle };
}

/** C Donut::Create (donut.cpp) — Soft Body / Events path. */
function spawnDonut(sim: SimWorld, px: number, py: number, scale: number) {
  const sides = 7;
  const radius = 1.0 * scale;
  const delta = (2 * PI) / sides;
  const length = (2 * PI * radius) / sides;
  const bodies: number[] = [];
  let angle = 0;
  for (let i = 0; i < sides; i++) {
    const b = sim.add_body(radius * Math.cos(angle) + px, radius * Math.sin(angle) + py, angle, BODY_DYNAMIC);
    sim.attach_capsule_filtered(b, 0, -0.5 * length, 0, 0.5 * length, 0.25 * scale, 1, 0.3, 0, 0);
    bodies.push(b);
    angle += delta;
  }
  let prev = bodies[sides - 1]!;
  for (let i = 0; i < sides; i++) {
    const b = bodies[i]!;
    const p = sim.positions();
    const angA = p[prev * 3 + 2]!;
    const angB = p[b * 3 + 2]!;
    sim.add_weld_joint_local(prev, b, 0, 0.5 * length, 0, -0.5 * length, angA, angB, 0, 5, 0, 0, false);
    prev = b;
  }
}

/** f32 nextafter(x, +inf) − x for float grid readout (FarGate :713-714). */
function floatGridStep(x: number): number {
  const f32 = new Float32Array([x]);
  const u32 = new Uint32Array(f32.buffer);
  u32[0]! += 1;
  return f32[0]! - x;
}

// ---------------------------------------------------------------------------
// Scene builders — cite sample_world.cpp
// ---------------------------------------------------------------------------

function buildTiles(sim: SimWorld, controls: HTMLElement, camera: SampleCamera): SceneRuntime {
  // :17-241 — PARTIAL: DEBUG cycleCount=10 (C release 600); CreateHuman cycles Exact
  const period = 40.0;
  const omega = (2.0 * PI) / period;
  const cycleCount = TILES_CYCLE_COUNT;
  const gridSize = 1.0;
  const gridCount = Math.floor((cycleCount * period) / gridSize);
  const xStart = -0.5 * (cycleCount * period);

  camera.centerX = xStart;
  camera.centerY = 15.0;
  camera.zoom = 25.0;

  {
    let xBody = xStart;
    let xShape = xStart;
    let groundId = -1;
    const height = 4.0;
    for (let i = 0; i < gridCount; i++) {
      if (i % 10 === 0) {
        groundId = sim.add_body(xBody, 0, 0, BODY_STATIC);
        xShape = 0.0;
      }
      let y = 0.0;
      const ycount = Math.round(height * Math.cos(omega * xBody)) + 12;
      for (let j = 0; j < ycount; j++) {
        // :73-75 MakeOffsetBox 0.4*grid, radius 0.1; invokeContactCreation=false
        sim.attach_offset_rounded_box(
          groundId,
          0.4 * gridSize,
          0.4 * gridSize,
          xShape,
          y,
          0,
          0.1,
          0,
          FRIC,
          0,
          false,
        );
        y += gridSize;
      }
      xBody += gridSize;
      xShape += gridSize;
    }
  }

  let humanIndex = 0;
  for (let cycleIndex = 0; cycleIndex < cycleCount; cycleIndex++) {
    const xbase = (0.5 + cycleIndex) * period + xStart;
    const remainder = cycleIndex % 3;
    if (remainder === 0) {
      // :93-110 box columns
      let x = xbase - 3.0;
      for (let i = 0; i < 10; i++) {
        let y = 10.0;
        for (let j = 0; j < 5; j++) {
          sim.add_box(x, y, 0.3, 0.2, 1.0);
          y += 0.5;
        }
        x += 0.6;
      }
    } else if (remainder === 1) {
      // :114-121 CreateHuman scale=1.5, friction=0.05, hertz=0, damping=0
      let x = xbase - 2.0;
      for (let i = 0; i < 5; i++) {
        sim.create_human(x, 10.0, 1.5, 0.05, 0.0, 0.0, humanIndex + 1, false, 0);
        humanIndex += 1;
        x += 1.0;
      }
    } else {
      // :125-132 Donut
      let x = xbase - 4.0;
      for (let i = 0; i < 5; i++) {
        spawnDonut(sim, x, 12.0, 0.75);
        x += 2.0;
      }
    }
  }

  // :136 Car::Spawn scale=10, hertz=2, damping=0.7, torque=2000
  const car = spawnCar(sim, xStart + 20.0, 40.0, 10.0, 2.0, 0.7, 2000.0);

  let viewX = xStart;
  let speed = 0.0;
  let explode = true;
  let followCar = false;
  let cycleIndex = 0;
  let stepCount = 0;
  const span = 0.5 * (period * cycleCount);
  let explosionX = (0.5 + cycleIndex) * period + xStart;
  const explosionY = 7.0;
  const radius = 2.0;

  const keys = new Set<string>();
  const onKey = (e: KeyboardEvent) => {
    if (e.type === "keydown") keys.add(e.key.toLowerCase());
    else keys.delete(e.key.toLowerCase());
  };
  window.addEventListener("keydown", onKey);
  window.addEventListener("keyup", onKey);

  controls.appendChild(
    createSlider("speed", -400, 400, speed, 1, (v) => {
      speed = v;
    }),
  );
  controls.appendChild(
    createButton("stop", () => {
      speed = 0;
    }),
  );
  controls.appendChild(
    createCheckbox("explode", explode, (en) => {
      explode = en;
    }),
  );
  controls.appendChild(
    createCheckbox("follow car", followCar, (en) => {
      followCar = en;
    }),
  );
  controls.appendChild(
    createInfoBox(
      "PARTIAL: DEBUG <code>cycleCount=10</code> (C release 600). Human cycles use " +
        "<code>CreateHuman</code>. ASD drives the car.",
    ),
  );

  return {
    beforeStep(dt) {
      viewX += dt * speed;
      viewX = Math.max(-span, Math.min(span, viewX));
      if (speed !== 0) {
        camera.centerX = viewX;
        camera.centerY = 15.0;
      }
      if (followCar) {
        const p = sim.positions();
        camera.centerX = p[car.chassis * 3]!;
      }

      if ((stepCount & 0x1) === 0x1 && explode) {
        explosionX = (0.5 + cycleIndex) * period - span;
        sim.explode(explosionX, explosionY, radius, 0.1, 1.0);
        cycleIndex = (cycleIndex + 1) % cycleCount;
      }

      // :206-218 ASD throttle
      if (keys.has("a")) {
        sim.wheel_set_motor_speed(car.rearAxle, 20);
        sim.wheel_set_motor_speed(car.frontAxle, 20);
      } else if (keys.has("s")) {
        sim.wheel_set_motor_speed(car.rearAxle, 0);
        sim.wheel_set_motor_speed(car.frontAxle, 0);
      } else if (keys.has("d")) {
        sim.wheel_set_motor_speed(car.rearAxle, -5);
        sim.wheel_set_motor_speed(car.frontAxle, -5);
      }
    },
    afterStep() {
      stepCount++;
    },
    paintOverlay(ctx, cam, canvas) {
      if (!explode) return;
      const c = worldToScreen(cam, canvas, explosionX, explosionY);
      const ppm = canvas.height / (2 * Math.max(1e-6, cam.zoom));
      ctx.beginPath();
      ctx.arc(c.x, c.y, radius * ppm, 0, 2 * PI);
      ctx.strokeStyle = "rgba(240,255,255,0.8)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    },
    readoutExtra: () => [
      {
        label: "World size",
        value: `${((gridSize * gridCount) / 1000.0).toFixed(3)} km`,
      },
    ],
    dispose() {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("keyup", onKey);
    },
  };
}

function buildFarPyramid(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :249-313 Exact
  const originX = 10.0e6;
  const originY = 0.0;
  const h = 0.25;
  const ground = sim.add_body(originX, originY, 0, BODY_STATIC);
  sim.attach_segment(ground, -40.0, 0.0, 40.0, 0.0);

  const baseCount = 50;
  for (let i = 0; i < baseCount; i++) {
    const y = (2.0 * i + 1.0) * h;
    for (let j = i; j < baseCount; j++) {
      const x = (i + 1.0) * h + 2.0 * (j - i) * h - h * baseCount;
      sim.add_box(originX + x, originY + y, h, h, 1.0);
    }
  }

  return {
    readoutExtra: () => [
      { label: "Precision", value: "Single precision" },
      { label: "View", value: `${(0.001 * originX).toFixed(0)} km from origin` },
    ],
  };
}

function buildFarRagdolls(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :317-388 Exact CreateHuman pile at 1e7 m (5×5)
  const originX = 10.0e6;
  const originY = 0.0;
  const ground = sim.add_body(originX, originY, 0, BODY_STATIC);
  const w = 6.0;
  const h = 12.0;
  sim.attach_segment(ground, -w, 0.0, w, 0.0);
  sim.attach_segment(ground, -w, 0.0, -w, h);
  sim.attach_segment(ground, w, 0.0, w, h);

  const scale = 1.0;
  const columnCount = 5;
  const rowCount = 5;
  const rng = makeXorShift(12345);
  let humanCount = 0;
  for (let i = 0; i < rowCount; i++) {
    for (let j = 0; j < columnCount; j++) {
      const x =
        2.4 * scale * (j - 0.5 * (columnCount - 1)) + rng.floatRange(-0.3, 0.3);
      const y = 2.0 + 2.2 * scale * i;
      // :354 CreateHuman(…, scale, 0.05, 1.0, 0.5, index+1, nullptr, false)
      sim.create_human(originX + x, originY + y, scale, 0.05, 1.0, 0.5, humanCount + 1, false, 0);
      humanCount++;
    }
  }

  controls.appendChild(
    createInfoBox(
      "Exact: 5×5 <code>CreateHuman</code> ragdolls piled at 10 000 km. " +
        "C <code>sample_world.cpp</code> Far Ragdolls.",
    ),
  );

  return {
    readoutExtra: () => [
      { label: "Precision", value: "Single precision" },
      {
        label: "Pile",
        value: `${humanCount} ragdolls @ ${(0.001 * originX).toFixed(0)} km`,
      },
    ],
  };
}

function buildFarGate(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :397-739 Exact Gear Lift mechanism at 1e6 m
  const originX = 1.0e6;
  const originY = 0.0;

  const ground = sim.add_body(originX, originY, 0, BODY_STATIC);
  const path =
    "m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l " +
    "-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v " +
    "-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l " +
    "-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166";
  const chainPts = parsePath(path, -120.0, -200.0, 64, 0.2);
  sim.attach_chain(ground, chainPts, true);

  const gearRadius = 1.0;
  const toothHalfWidth = 0.09;
  const toothHalfHeight = 0.06;
  const toothRadius = 0.03;
  const linkHalfLength = 0.07;
  const linkRadius = 0.05;
  const doorHalfHeight = 1.5;

  const gearPosition1X = originX - 4.25;
  const gearPosition1Y = originY + 9.75;
  const gearPosition2X = gearPosition1X + 2.0;
  const gearPosition2Y = gearPosition1Y + 1.0;
  const linkAttachX = gearPosition2X + gearRadius + 2.0 * toothHalfWidth + toothRadius;
  const linkAttachY = gearPosition2Y;
  const doorX = linkAttachX;
  const doorY = linkAttachY - 2.0 * 40 * linkHalfLength - doorHalfHeight;

  let motorTorque = 80.0;
  let motorSpeed = -0.3;
  let enableMotor = true;

  // Driver gear :460-504
  const driver = sim.add_body(gearPosition1X, gearPosition1Y, 0, BODY_DYNAMIC);
  sim.attach_circle(driver, 0, 0, gearRadius, 1, 0.1, 0);
  {
    const deltaAngle = (2.0 * PI) / 16;
    let rotation = 0;
    for (let i = 0; i < 16; i++) {
      const cx = Math.cos(rotation) * (gearRadius + toothHalfHeight);
      const cy = Math.sin(rotation) * (gearRadius + toothHalfHeight);
      sim.attach_offset_rounded_box(
        driver,
        toothHalfWidth,
        toothHalfHeight,
        cx,
        cy,
        rotation,
        toothRadius,
        1,
        0.1,
        0,
        true,
      );
      rotation += deltaAngle;
    }
  }
  const driverJoint = sim.add_revolute_joint(
    ground,
    driver,
    gearPosition1X,
    gearPosition1Y,
    false,
    0,
    0,
    enableMotor,
    motorSpeed,
    motorTorque,
    false,
    0,
    0,
    false,
  );

  // Follower gear :506-551 — localFrameA.q = 0.25π
  const follower = sim.add_body(gearPosition2X, gearPosition2Y, 0, BODY_DYNAMIC);
  sim.attach_circle(follower, 0, 0, gearRadius, 1, 0.1, 0);
  {
    const deltaAngle = (2.0 * PI) / 16;
    let rotation = 0;
    for (let i = 0; i < 16; i++) {
      const cx = Math.cos(rotation) * (gearRadius + toothHalfWidth);
      const cy = Math.sin(rotation) * (gearRadius + toothHalfWidth);
      sim.attach_offset_rounded_box(
        follower,
        toothHalfWidth,
        toothHalfHeight,
        cx,
        cy,
        rotation,
        toothRadius,
        1,
        0.1,
        0,
        true,
      );
      rotation += deltaAngle;
    }
  }
  sim.add_revolute_joint_angled(
    ground,
    follower,
    gearPosition2X,
    gearPosition2Y,
    0.25 * PI,
    true,
    -0.3 * PI,
    0.8 * PI,
    true,
    0.5,
  );

  // Link chain :553-591
  let prevBody = follower;
  let positionY = linkAttachY - linkHalfLength;
  let lastLink = follower;
  for (let i = 0; i < 40; i++) {
    const body = sim.add_body(linkAttachX, positionY, 0, BODY_DYNAMIC);
    sim.attach_capsule(body, 0, -linkHalfLength, 0, linkHalfLength, linkRadius, 2.0, FRIC, 0);
    const pivotY = positionY + linkHalfLength;
    const p = sim.positions();
    // local pivots via world pivot on both bodies
    sim.add_revolute_joint(
      prevBody,
      body,
      linkAttachX,
      pivotY,
      false,
      0,
      0,
      true,
      0,
      0.05,
      false,
      0,
      0,
      false,
    );
    void p;
    positionY -= 2.0 * linkHalfLength;
    prevBody = body;
    lastLink = body;
  }

  // Door :593-632
  const door = sim.add_body(doorX, doorY, 0, BODY_DYNAMIC);
  sim.attach_box(door, 0.15, doorHalfHeight, 0, 0, 0, 1, 0.1, 0);
  {
    const pivotY = doorY + doorHalfHeight;
    sim.add_revolute_joint(
      lastLink,
      door,
      doorX,
      pivotY,
      false,
      0,
      0,
      true,
      0,
      0.05,
      false,
      0,
      0,
      false,
    );
  }
  // Prismatic along +Y at doorPosition (:619-630)
  {
    const localAx = doorX - originX;
    const localAy = doorY - originY;
    sim.add_prismatic_joint_local(
      ground,
      door,
      localAx,
      localAy,
      0,
      0,
      0,
      1,
      false,
      0,
      0,
      true,
      0,
      0.2,
      false,
      0,
      0,
      true,
    );
  }

  // Debris pile :634-665
  const rng = makeXorShift(12345);
  let y = 4.25;
  for (let i = 0; i < 20; i++) {
    let x = -3.15;
    for (let j = 0; j < 10; j++) {
      const body = sim.add_body(originX + x, originY + y, 0, BODY_DYNAMIC);
      const rad = rng.floatRange(0.01, 0.02);
      attachRandomPolygon(sim, body, 0.1, rad, 1, FRIC, 0.3, rng);
      x += 0.2;
    }
    y += 0.2;
  }

  const keys = new Set<string>();
  const onKey = (e: KeyboardEvent) => {
    if (e.type === "keydown") keys.add(e.key.toLowerCase());
    else keys.delete(e.key.toLowerCase());
  };
  window.addEventListener("keydown", onKey);
  window.addEventListener("keyup", onKey);

  controls.appendChild(
    createCheckbox("Motor", enableMotor, (en) => {
      enableMotor = en;
      sim.revolute_enable_motor(driverJoint, en);
      sim.joint_wake_bodies(driverJoint);
    }),
  );
  controls.appendChild(
    createSlider("Max Torque", 0, 100, motorTorque, 1, (v) => {
      motorTorque = v;
      sim.revolute_set_max_motor_torque(driverJoint, v);
      sim.joint_wake_bodies(driverJoint);
    }),
  );
  controls.appendChild(
    createSlider("Speed", -0.3, 0.3, motorSpeed, 0.01, (v) => {
      motorSpeed = v;
      sim.revolute_set_motor_speed(driverJoint, v);
      sim.joint_wake_bodies(driverJoint);
    }),
  );

  const gridStep = floatGridStep(originX);

  return {
    beforeStep() {
      if (keys.has("a")) {
        motorSpeed = Math.max(-0.3, motorSpeed - 0.01);
        sim.revolute_set_motor_speed(driverJoint, motorSpeed);
        sim.joint_wake_bodies(driverJoint);
      }
      if (keys.has("d")) {
        motorSpeed = Math.min(0.3, motorSpeed + 0.01);
        sim.revolute_set_motor_speed(driverJoint, motorSpeed);
        sim.joint_wake_bodies(driverJoint);
      }
    },
    readoutExtra: () => [
      { label: "Precision", value: "Single precision" },
      { label: "Offset", value: `${(originX / 1000).toFixed(0)} km` },
      {
        label: "Float grid",
        value: `${gridStep} m (teeth ~0.12 m)`,
      },
      { label: "Motor speed", value: motorSpeed.toFixed(2) },
    ],
    dispose() {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("keyup", onKey);
    },
  };
}

function buildScene(
  scene: Scene,
  sim: SimWorld,
  controls: HTMLElement,
  camera: SampleCamera,
): SceneRuntime {
  clearControls(controls);
  switch (scene) {
    case "tiles":
      return buildTiles(sim, controls, camera);
    case "far-pyramid":
      return buildFarPyramid(sim, controls);
    case "far-ragdolls":
      return buildFarRagdolls(sim, controls);
    case "far-gate":
      return buildFarGate(sim, controls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "World",
    "C <code>sample_world.cpp</code> RegisterSample ports — large tiled terrain, " +
      "far-origin pyramid / ragdolls / gear-lift gate.",
    "Drag to grab · P pause · O step · R restart · ASD where noted",
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "tiles";

  const camera: SampleCamera = makeCamera();
  applyCamera(camera, scene);
  const transport = createSampleTransport({ subSteps: DEFAULT_SUB_STEPS });
  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = {};

  const sceneControls = document.createElement("div");
  sceneControls.className = "scene-controls";

  function rebuild() {
    runtime.dispose?.();
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera, scene);
    runtime = buildScene(scene, sim, sceneControls, camera);
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
        history.replaceState(null, "", `#/world/${scene}`);
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
    sim.step(dt, transport.subSteps);
    runtime.afterStep?.(dt);

    const b = viewBounds(camera, canvas);
    sim.collect_draw(b.lowerX, b.lowerY, b.upperX, b.upperY);
    paintDebugDraw(canvas, camera, {
      polygons: sim.draw_polygons(),
      circles: sim.draw_circles(),
      capsules: sim.draw_capsules(),
      lines: sim.draw_lines(),
    });
    const ctx = canvas.getContext("2d");
    if (ctx && runtime.paintOverlay) runtime.paintOverlay(ctx, camera, canvas);

    updateReadout(readout, [
      { label: "Sample", value: SCENE_LABEL[scene] },
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
      { label: "Hz", value: String(transport.hertz) },
      { label: "Sub", value: String(transport.subSteps) },
      { label: "Paused", value: transport.paused ? "yes" : "no" },
      ...(runtime.readoutExtra?.() ?? []),
    ]);
  }, readout);

  return () => {
    stop();
    unbindKeys();
    disposeTransport(transport);
    runtime.dispose?.();
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    freeSim(sim);
  };
}
