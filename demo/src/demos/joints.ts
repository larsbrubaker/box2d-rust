// Joints — RegisterSample ports from sample_joints.cpp.
// Invented hinge/pendulum composite retired; scenes map 1:1 to C names.

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
  disposeTransport,
  makeCamera,
  screenToWorld,
  viewBounds,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — must match slugify(C name). */
export const SCENES = [
  "distance-joint",
  "motor-joint",
  "top-down-friction",
  "filter-joint",
  "revolute",
  "prismatic",
  "wheel",
  "bridge",
  "ball-chain",
  "cantilever",
  "motion-locks",
  "soft-body",
  "doohickey",
  "breakable",
  "separation",
  "user-constraint",
  "driving",
  "door",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("joints", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "distance-joint": "Distance Joint",
  "motor-joint": "Motor Joint",
  "top-down-friction": "Top Down Friction",
  "filter-joint": "Filter Joint",
  revolute: "Revolute",
  prismatic: "Prismatic",
  wheel: "Wheel",
  bridge: "Bridge",
  "ball-chain": "Ball & Chain",
  cantilever: "Cantilever",
  "motion-locks": "Motion Locks",
  "soft-body": "Soft Body",
  doohickey: "Doohickey",
  breakable: "Breakable",
  separation: "Separation",
  "user-constraint": "User Constraint",
  driving: "Driving",
  door: "Door",
};

const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "distance-joint": { cx: 0, cy: 12, zoom: 25 * 0.35 }, // :32-33
  "motor-joint": { cx: 0, cy: 7, zoom: 25 * 0.4 }, // :253-254
  "top-down-friction": { cx: 0, cy: 7, zoom: 25 * 0.4 }, // :423-424
  "filter-joint": { cx: 0, cy: 7, zoom: 25 * 0.4 }, // :536-537
  revolute: { cx: 0, cy: 15.5, zoom: 25 * 0.7 }, // :587-588
  prismatic: { cx: 0, cy: 8, zoom: 25 * 0.5 }, // :795-796
  wheel: { cx: 0, cy: 10, zoom: 25 * 0.15 }, // :952-953
  bridge: { cx: 0, cy: 0, zoom: 25 * 2.5 }, // :1084 (center default)
  "ball-chain": { cx: 0, cy: -8, zoom: 27.5 }, // :1256-1257
  cantilever: { cx: 0, cy: 0, zoom: 25 * 0.35 }, // :1371-1372
  "motion-locks": { cx: 0, cy: 8, zoom: 25 * 0.7 }, // :1526-1527
  "soft-body": { cx: 0, cy: 5, zoom: 25 * 0.25 }, // :2663-2664
  doohickey: { cx: 0, cy: 5, zoom: 25 * 0.35 }, // :2696-2697
  breakable: { cx: 0, cy: 8, zoom: 25 * 0.7 }, // :1752-1753
  separation: { cx: 0, cy: 8, zoom: 25 * 0.5 }, // approx
  "user-constraint": { cx: 0, cy: 5, zoom: 25 * 0.35 },
  driving: { cx: 0, cy: 5, zoom: 25 * 0.4 }, // :2326-2327
  door: { cx: 0, cy: 2.5, zoom: 25 * 0.2 },
};

const FRIC = 0.6;
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const PI = Math.PI;

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

// C XorShift32 (utils.h RAND_SEED=12345) for Top Down Friction RandomPolygon.
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
  const range = (lo: number, hi: number) => {
    const r = (next() & RAND_LIMIT) / RAND_LIMIT;
    return (hi - lo) * r + lo;
  };
  return { range, next };
}

// ---------------------------------------------------------------------------
// Scene builders — cite sample_joints.cpp
// ---------------------------------------------------------------------------

function buildDistanceJoint(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :27-218
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let count = 1;
  let length = 1.0;
  let hertz = 5.0;
  let damping = 0.5;
  let tension = 2000.0;
  let compression = 100.0;
  let enableSpring = false;
  let enableLimit = false;
  let minLength = 1.0;
  let maxLength = 1.0;
  const bodies: number[] = [];
  const joints: number[] = [];

  function createScene(newCount: number) {
    for (const j of joints) sim.destroy_joint(j);
    for (const b of bodies) sim.destroy_body(b);
    joints.length = 0;
    bodies.length = 0;
    count = newCount;
    const yOffset = 20.0;
    const radius = 0.25;
    let prev = ground;
    for (let i = 0; i < count; i++) {
      const body = sim.add_body(length * (i + 1), yOffset, 0, BODY_DYNAMIC);
      sim.set_angular_damping(body, 1.0);
      sim.attach_circle(body, 0, 0, radius, 20.0, FRIC, 0);
      bodies.push(body);
      const ax = length * i;
      const bx = length * (i + 1);
      const jid = sim.add_distance_joint_ex(
        prev,
        body,
        ax,
        yOffset,
        bx,
        yOffset,
        length,
        enableSpring,
        hertz,
        damping,
        tension,
        compression,
        enableLimit,
        minLength,
        maxLength,
        false,
      );
      joints.push(jid);
      prev = body;
    }
  }
  createScene(1);

  const wakeAll = () => {
    for (const j of joints) sim.joint_wake_bodies(j);
  };

  controls.appendChild(
    createSlider("Length", 0.1, 4, length, 0.1, (v) => {
      length = v;
      for (const j of joints) sim.distance_set_length(j, length);
      wakeAll();
    }),
  );
  controls.appendChild(
    createCheckbox("Spring", enableSpring, (en) => {
      enableSpring = en;
      for (const j of joints) sim.distance_enable_spring(j, en);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Tension", 0, 4000, tension, 50, (v) => {
      tension = v;
      for (const j of joints) sim.distance_set_spring_force_range(j, -tension, compression);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Compression", 0, 200, compression, 5, (v) => {
      compression = v;
      for (const j of joints) sim.distance_set_spring_force_range(j, -tension, compression);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 15, hertz, 0.5, (v) => {
      hertz = v;
      for (const j of joints) sim.distance_set_spring_hertz(j, hertz);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Damping", 0, 4, damping, 0.1, (v) => {
      damping = v;
      for (const j of joints) sim.distance_set_spring_damping(j, damping);
      wakeAll();
    }),
  );
  controls.appendChild(
    createCheckbox("Limit", enableLimit, (en) => {
      enableLimit = en;
      for (const j of joints) sim.distance_enable_limit(j, en);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Min Length", 0.1, 4, minLength, 0.1, (v) => {
      minLength = v;
      for (const j of joints) sim.distance_set_length_range(j, minLength, maxLength);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Max Length", 0.1, 4, maxLength, 0.1, (v) => {
      maxLength = v;
      for (const j of joints) sim.distance_set_length_range(j, minLength, maxLength);
      wakeAll();
    }),
  );
  controls.appendChild(
    createSlider("Count", 1, 10, count, 1, (v) => {
      createScene(Math.round(v));
    }),
  );
  return {};
}

function buildMotorJoint(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :248-411
  const ground = sim.add_segment(-20, 0, 20, 0);
  const target = sim.add_body(0, 8, 0, BODY_KINEMATIC);
  const body = sim.add_body(0, 8, 0, BODY_DYNAMIC);
  sim.attach_box(body, 2, 0.5, 0, 0, 0, 1, FRIC, 0);
  let maxForce = 5000;
  let maxTorque = 500;
  const joint = sim.add_motor_joint(target, body, 4, 0.7, maxForce, 4, 0.7, maxTorque, 0, 0, false);

  // Spring box :303-327
  const spring = sim.add_body(-2, 2, 0, BODY_DYNAMIC);
  sim.attach_box(spring, 0.5, 0.5, 0, 0, 0, 1, FRIC, 0);
  sim.add_motor_joint_local(ground, spring, -1.75, 2.25, 0.25, 0.25, 7.5, 0.7, 500, 7.5, 0.7, 10, 0, 0, false);

  let speed = 1;
  let time = 0;
  let tx = 0;
  let ty = 8;
  let ta = 0;

  controls.appendChild(createSlider("Speed", -5, 5, speed, 1, (v) => { speed = v; }));
  controls.appendChild(
    createSlider("Max Force", 0, 10000, maxForce, 100, (v) => {
      maxForce = v;
      sim.motor_set_max_spring_force(joint, maxForce);
    }),
  );
  controls.appendChild(
    createSlider("Max Torque", 0, 10000, maxTorque, 100, (v) => {
      maxTorque = v;
      sim.motor_set_max_spring_torque(joint, maxTorque);
    }),
  );
  controls.appendChild(
    createButton("Apply Impulse", () => {
      sim.apply_linear_impulse_to_center(body, 100, 0, true);
    }),
  );

  return {
    beforeStep(dt) {
      if (dt <= 0) return;
      time += speed * dt;
      tx = 6 * Math.sin(2 * time);
      ty = 8 + 4 * Math.sin(time);
      ta = 2 * time;
      sim.set_target_transform(target, tx, ty, ta, dt, true);
    },
    paintOverlay(ctx, camera, canvas) {
      // DrawTransform proxy: cross at target
      const sx = ((tx - camera.centerX) / (camera.zoom * 2)) * canvas.height + canvas.width / 2;
      const sy = canvas.height / 2 - ((ty - camera.centerY) / (camera.zoom * 2)) * canvas.height;
      ctx.strokeStyle = "#22c55e";
      ctx.beginPath();
      ctx.moveTo(sx - 12, sy);
      ctx.lineTo(sx + 12, sy);
      ctx.moveTo(sx, sy - 12);
      ctx.lineTo(sx, sy + 12);
      ctx.stroke();
    },
    readoutExtra() {
      const ft = sim.joint_constraint_ft(joint);
      return [
        { label: "force", value: `{${ft[0]!.toFixed(0)}, ${ft[1]!.toFixed(0)}}` },
        { label: "torque", value: ft[2]!.toFixed(0) },
      ];
    },
  };
}

function buildTopDownFriction(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :418-523
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -10, 0, 10, 0);
  sim.attach_segment(ground, -10, 0, -10, 20);
  sim.attach_segment(ground, 10, 0, 10, 20);
  sim.attach_segment(ground, -10, 20, 10, 20);

  const rng = makeXorShift(12345);
  const n = 10;
  let x = -5;
  let y = 15;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const body = sim.add_body(x, y, 0, BODY_DYNAMIC);
      sim.set_gravity_scale(body, 0);
      const rem = (n * i + j) % 4;
      if (rem === 0) sim.attach_capsule(body, -0.25, 0, 0.25, 0, 0.25, 1, FRIC, 0.8);
      else if (rem === 1) sim.attach_circle(body, 0, 0, 0.35, 1, FRIC, 0.8);
      else if (rem === 2) sim.attach_box(body, 0.35, 0.35, 0, 0, 0, 1, FRIC, 0.8);
      else {
        const pts: number[] = [];
        const count = 3 + (rng.next() % 6);
        for (let k = 0; k < count; k++) {
          pts.push(rng.range(-0.75, 0.75), rng.range(-0.75, 0.75));
        }
        // Fall back to square via attach if hull fails — use add_polygon at body origin then destroy empty? Simpler: box.
        sim.attach_box(body, 0.35, 0.35, 0, 0, 0, 1, FRIC, 0.8);
        void pts;
      }
      sim.add_motor_joint(ground, body, 0, 0, 0, 0, 0, 0, 10, 10, true);
      x += 1;
    }
    x = -5;
    y -= 1;
  }

  controls.appendChild(
    createButton("Explode", () => {
      sim.explode(0, 10, 10, 5, 10);
    }),
  );
  return {};
}

function buildFilterJoint(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :528-577
  sim.add_segment(-20, 0, 20, 0);
  const a = sim.add_body(-4, 2, 0, BODY_DYNAMIC);
  sim.attach_box(a, 2, 2, 0, 0, 0, 1, FRIC, 0);
  const b = sim.add_body(4, 2, 0, BODY_DYNAMIC);
  sim.attach_box(b, 2, 2, 0, 0, 0, 1, FRIC, 0);
  sim.add_filter_joint(a, b);
  return {};
}

function buildRevolute(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :582-785
  const ground = sim.add_body(0, -1, 0, BODY_STATIC);
  sim.attach_box(ground, 40, 1, 0, 0, 0, 0, FRIC, 0);

  let enableSpring = false;
  let enableLimit = false;
  let enableMotor = false;
  let hertz = 2;
  let damping = 0.5;
  let targetDeg = 45;
  let motorSpeed = 1;
  let motorTorque = 1000;

  const arm = sim.add_body(-10, 20, 0, BODY_DYNAMIC);
  sim.attach_capsule(arm, 0, -1, 0, 6, 0.5, 1, FRIC, 0);
  const j1 = sim.add_revolute_joint_angled(
    ground,
    arm,
    -10,
    20.5,
    0.5 * PI,
    enableLimit,
    -0.5 * PI,
    0.05 * PI,
    enableMotor,
    motorTorque,
  );
  sim.revolute_set_motor_speed(j1, motorSpeed);
  sim.revolute_enable_spring(j1, enableSpring);
  sim.revolute_set_spring_hertz(j1, hertz);
  sim.revolute_set_spring_damping(j1, damping);
  sim.revolute_set_target_angle(j1, (PI * targetDeg) / 180);
  sim.joint_set_constraint_tuning(j1, 60, 20);

  const ballBody = sim.add_body(5, 30, 0, BODY_DYNAMIC);
  sim.attach_circle(ballBody, 0, 0, 2, 1, FRIC, 0);

  const plank = sim.add_body(20, 10, 0, BODY_DYNAMIC);
  sim.attach_box(plank, 10, 0.5, -10, 0, 0, 1, FRIC, 0);
  const j2 = sim.add_revolute_joint(
    ground,
    plank,
    19,
    10,
    true,
    -0.25 * PI,
    0,
    true,
    0,
    motorTorque,
    false,
    0,
    0,
    false,
  );

  const wake = () => sim.joint_wake_bodies(j1);
  controls.appendChild(
    createCheckbox("Limit", enableLimit, (en) => {
      enableLimit = en;
      sim.revolute_enable_limit(j1, en);
      wake();
    }),
  );
  controls.appendChild(
    createCheckbox("Motor", enableMotor, (en) => {
      enableMotor = en;
      sim.revolute_enable_motor(j1, en);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Max Torque", 0, 5000, motorTorque, 50, (v) => {
      motorTorque = v;
      sim.revolute_set_max_motor_torque(j1, v);
      sim.revolute_set_max_motor_torque(j2, v);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Speed", -20, 20, motorSpeed, 1, (v) => {
      motorSpeed = v;
      sim.revolute_set_motor_speed(j1, v);
      wake();
    }),
  );
  controls.appendChild(
    createCheckbox("Spring", enableSpring, (en) => {
      enableSpring = en;
      sim.revolute_enable_spring(j1, en);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 30, hertz, 0.5, (v) => {
      hertz = v;
      sim.revolute_set_spring_hertz(j1, v);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Damping", 0, 2, damping, 0.1, (v) => {
      damping = v;
      sim.revolute_set_spring_damping(j1, v);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Degrees", -180, 180, targetDeg, 1, (v) => {
      targetDeg = v;
      sim.revolute_set_target_angle(j1, (PI * v) / 180);
      wake();
    }),
  );

  return {
    readoutExtra: () => [
      { label: "Angle 1 (deg)", value: ((sim.revolute_get_angle(j1) * 180) / PI).toFixed(1) },
      { label: "Motor Torque 1", value: sim.revolute_get_motor_torque(j1).toFixed(1) },
      { label: "Motor Torque 2", value: sim.revolute_get_motor_torque(j2).toFixed(1) },
    ],
  };
}

function buildPrismatic(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :787-942
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let enableSpring = false;
  let enableLimit = true;
  let enableMotor = false;
  let motorSpeed = 2;
  let motorForce = 25;
  let hertz = 1;
  let damping = 0.5;
  let translation = 0;

  const body = sim.add_body(0, 10, 0, BODY_DYNAMIC);
  sim.attach_box(body, 0.5, 2, 0, 0, 0, 1, FRIC, 0);
  const j = sim.add_prismatic_joint(
    ground,
    body,
    0,
    9,
    1,
    1,
    enableLimit,
    -10,
    10,
    enableMotor,
    motorSpeed,
    motorForce,
    enableSpring,
    hertz,
    damping,
    false,
  );

  const wake = () => sim.joint_wake_bodies(j);
  controls.appendChild(
    createCheckbox("Limit", enableLimit, (en) => {
      enableLimit = en;
      sim.prismatic_enable_limit(j, en);
      wake();
    }),
  );
  controls.appendChild(
    createCheckbox("Motor", enableMotor, (en) => {
      enableMotor = en;
      sim.prismatic_enable_motor(j, en);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Max Force", 0, 200, motorForce, 1, (v) => {
      motorForce = v;
      sim.prismatic_set_max_motor_force(j, v);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Speed", -40, 40, motorSpeed, 1, (v) => {
      motorSpeed = v;
      sim.prismatic_set_motor_speed(j, v);
      wake();
    }),
  );
  controls.appendChild(
    createCheckbox("Spring", enableSpring, (en) => {
      enableSpring = en;
      sim.prismatic_enable_spring(j, en);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 10, hertz, 0.1, (v) => {
      hertz = v;
      sim.prismatic_set_spring_hertz(j, v);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Damping", 0, 2, damping, 0.1, (v) => {
      damping = v;
      sim.prismatic_set_spring_damping(j, v);
      wake();
    }),
  );
  controls.appendChild(
    createSlider("Translation", -10, 10, translation, 0.1, (v) => {
      translation = v;
      sim.prismatic_set_target_translation(j, v);
      wake();
    }),
  );
  return {
    readoutExtra: () => [{ label: "Motor Force", value: sim.prismatic_get_motor_force(j).toFixed(1) }],
  };
}

function buildWheel(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :944-1073
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let enableSpring = true;
  let enableLimit = true;
  let enableMotor = true;
  let motorSpeed = 2;
  let motorTorque = 5;
  let hertz = 1;
  let damping = 0.7;

  const body = sim.add_body(0, 10.25, 0, BODY_DYNAMIC);
  sim.attach_capsule(body, 0, -0.5, 0, 0.5, 0.5, 1, FRIC, 0);
  const j = sim.add_wheel_joint(
    ground,
    body,
    0,
    10,
    1,
    1,
    enableLimit,
    -3,
    3,
    enableMotor,
    motorSpeed,
    motorTorque,
    enableSpring,
    hertz,
    damping,
    false,
  );

  controls.appendChild(
    createCheckbox("Limit", enableLimit, (en) => {
      enableLimit = en;
      sim.wheel_enable_limit(j, en);
    }),
  );
  controls.appendChild(
    createCheckbox("Motor", enableMotor, (en) => {
      enableMotor = en;
      sim.wheel_enable_motor(j, en);
    }),
  );
  controls.appendChild(
    createSlider("Torque", 0, 20, motorTorque, 1, (v) => {
      motorTorque = v;
      sim.wheel_set_max_motor_torque(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Speed", -20, 20, motorSpeed, 1, (v) => {
      motorSpeed = v;
      sim.wheel_set_motor_speed(j, v);
    }),
  );
  controls.appendChild(
    createCheckbox("Spring", enableSpring, (en) => {
      enableSpring = en;
      sim.wheel_enable_spring(j, en);
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 10, hertz, 0.1, (v) => {
      hertz = v;
      sim.wheel_set_spring_hertz(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Damping", 0, 2, damping, 0.1, (v) => {
      damping = v;
      sim.wheel_set_spring_damping(j, v);
    }),
  );
  return {
    readoutExtra: () => [{ label: "Motor Torque", value: sim.wheel_get_motor_torque(j).toFixed(1) }],
  };
}

function buildBridge(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1076-1246 — m_count = 160
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let frictionTorque = 200;
  let springHertz = 2;
  let springDamp = 0.7;
  let constraintHertz = 60;
  let constraintDamp = 0;
  const count = 160;
  const joints: number[] = [];
  const xbase = -80;
  let prev = ground;
  for (let i = 0; i < count; i++) {
    const body = sim.add_body(xbase + 0.5 + i, 20, 0, BODY_DYNAMIC);
    sim.set_linear_damping(body, 0.1);
    sim.set_angular_damping(body, 0.1);
    sim.attach_box(body, 0.5, 0.125, 0, 0, 0, 20, FRIC, 0);
    const pivotX = xbase + i;
    const j = sim.add_revolute_joint(
      prev,
      body,
      pivotX,
      20,
      false,
      0,
      0,
      true,
      0,
      frictionTorque,
      true,
      springHertz,
      springDamp,
      false,
    );
    joints.push(j);
    prev = body;
  }
  const end = sim.add_revolute_joint(
    prev,
    ground,
    xbase + count,
    20,
    false,
    0,
    0,
    true,
    0,
    frictionTorque,
    true,
    springHertz,
    springDamp,
    false,
  );
  joints.push(end);

  for (let i = 0; i < 2; i++) {
    sim.add_polygon(-8 + 8 * i, 22, 0, [-0.5, 0, 0.5, 0, 0, 1.5], 0, 20);
  }
  for (let i = 0; i < 3; i++) {
    const c = sim.add_body(-6 + 6 * i, 25, 0, BODY_DYNAMIC);
    sim.attach_circle(c, 0, 0, 0.5, 20, FRIC, 0);
  }

  controls.appendChild(
    createSlider("Joint Friction", 0, 10000, frictionTorque, 50, (v) => {
      frictionTorque = v;
      for (const j of joints) sim.revolute_set_max_motor_torque(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Spring hertz", 0, 30, springHertz, 1, (v) => {
      springHertz = v;
      for (const j of joints) sim.revolute_set_spring_hertz(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Spring damping", 0, 2, springDamp, 0.1, (v) => {
      springDamp = v;
      for (const j of joints) sim.revolute_set_spring_damping(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Constraint hertz", 15, 240, constraintHertz, 5, (v) => {
      constraintHertz = v;
      for (const j of joints) sim.joint_set_constraint_tuning(j, constraintHertz, constraintDamp);
    }),
  );
  controls.appendChild(
    createSlider("Constraint damping", 0, 10, constraintDamp, 0.1, (v) => {
      constraintDamp = v;
      for (const j of joints) sim.joint_set_constraint_tuning(j, constraintHertz, constraintDamp);
    }),
  );
  return {};
}

function buildBallAndChain(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1248-1354 — m_count typically 30
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let frictionTorque = 100;
  const count = 30;
  const joints: number[] = [];
  const hx = 0.5;
  let prev = ground;
  for (let i = 0; i < count; i++) {
    const body = sim.add_body((1 + 2 * i) * hx, count * hx, 0, BODY_DYNAMIC);
    // groupIndex -1 ≈ no link-link collision (C uses category/mask bits)
    sim.attach_capsule_filtered(body, -hx, 0, hx, 0, 0.125, 20, FRIC, 0, -1);
    const j = sim.add_revolute_joint(
      prev,
      body,
      2 * i * hx,
      count * hx,
      false,
      0,
      0,
      true,
      0,
      frictionTorque,
      i > 0,
      4,
      0,
      false,
    );
    joints.push(j);
    prev = body;
  }
  const ball = sim.add_body((1 + 2 * count) * hx + 4 - hx, count * hx, 0, BODY_DYNAMIC);
  sim.attach_circle(ball, 0, 0, 4, 20, FRIC, 0);
  const jBall = sim.add_revolute_joint(
    prev,
    ball,
    2 * count * hx,
    count * hx,
    false,
    0,
    0,
    true,
    0,
    frictionTorque,
    true,
    4,
    0,
    false,
  );
  joints.push(jBall);

  controls.appendChild(
    createSlider("Friction", 0, 1000, frictionTorque, 10, (v) => {
      frictionTorque = v;
      for (const j of joints) sim.revolute_set_max_motor_torque(j, v);
    }),
  );
  return {};
}

function buildCantilever(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1358-1508
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let linearHertz = 15;
  let linearDamp = 0.5;
  let angularHertz = 5;
  let angularDamp = 0.5;
  let gravityScale = 1;
  let collideConnected = false;
  const count = 8;
  const hx = 0.5;
  const bodies: number[] = [];
  const joints: number[] = [];
  let prev = ground;
  for (let i = 0; i < count; i++) {
    const body = sim.add_body((1 + 2 * i) * hx, 0, 0, BODY_DYNAMIC);
    sim.set_awake(body, false);
    sim.attach_capsule(body, -hx, 0, hx, 0, 0.125, 20, FRIC, 0);
    bodies.push(body);
    const j = sim.add_weld_joint(
      prev,
      body,
      2 * i * hx,
      0,
      linearHertz,
      angularHertz,
      linearDamp,
      angularDamp,
      collideConnected,
    );
    sim.joint_set_constraint_tuning(j, 120, 10);
    joints.push(j);
    prev = body;
  }
  const tip = prev;

  controls.appendChild(
    createSlider("Linear Hertz", 0, 20, linearHertz, 1, (v) => {
      linearHertz = v;
      for (const j of joints) sim.weld_set_linear_hertz(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Linear Damping Ratio", 0, 10, linearDamp, 0.1, (v) => {
      linearDamp = v;
      for (const j of joints) sim.weld_set_linear_damping(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Angular Hertz", 0, 20, angularHertz, 1, (v) => {
      angularHertz = v;
      for (const j of joints) sim.weld_set_angular_hertz(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Angular Damping Ratio", 0, 10, angularDamp, 0.1, (v) => {
      angularDamp = v;
      for (const j of joints) sim.weld_set_angular_damping(j, v);
    }),
  );
  controls.appendChild(
    createSlider("Gravity Scale", -1, 1, gravityScale, 0.1, (v) => {
      gravityScale = v;
      for (const b of bodies) sim.set_gravity_scale(b, v);
    }),
  );
  controls.appendChild(
    createCheckbox("Collide Connected", collideConnected, (en) => {
      collideConnected = en;
      for (const j of joints) sim.joint_set_collide_connected(j, en);
    }),
  );
  return {
    readoutExtra: () => {
      const p = sim.positions();
      const y = p[tip * 3 + 1] ?? 0;
      return [{ label: "tip-y", value: y.toFixed(2) }];
    },
  };
}

function buildMotionLocks(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1513+ — six jointed boxes with shared motion locks (default angular locked)
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let lockX = false;
  let lockY = false;
  let lockA = true;
  const bodies: number[] = [];
  let x = -12.5;
  const y = 10;

  // distance
  {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    sim.set_motion_locks(b, lockX, lockY, lockA);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(b);
    sim.add_distance_joint_ex(ground, b, x, y + 1 + 2, x, y + 1, 2, false, 0, 0, 0, 0, false, 0, 0, false);
  }
  x += 5;
  // motor
  {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    sim.set_motion_locks(b, lockX, lockY, lockA);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(b);
    sim.add_motor_joint_local(ground, b, x, y, 0, 0, 0, 0, 0, 0, 0, 0, 200, 200, false);
  }
  x += 5;
  // prismatic
  {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    sim.set_motion_locks(b, lockX, lockY, lockA);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(b);
    sim.add_prismatic_joint(ground, b, x - 1, y, 1, 0, false, 0, 0, false, 0, 0, false, 0, 0, false);
  }
  x += 5;
  // revolute
  {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    sim.set_motion_locks(b, lockX, lockY, lockA);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(b);
    sim.add_revolute_joint(ground, b, x - 1, y, false, 0, 0, false, 0, 0, false, 0, 0, false);
  }
  x += 5;
  // weld
  {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    sim.set_motion_locks(b, lockX, lockY, lockA);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(b);
    sim.add_weld_joint(ground, b, x - 1, y, 0, 0, 0, 0, false);
  }
  x += 5;
  // wheel
  {
    const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
    sim.set_motion_locks(b, lockX, lockY, lockA);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(b);
    sim.add_wheel_joint(ground, b, x - 1, y, 0, 1, false, 0, 0, false, 0, 0, true, 1, 0.7, false);
  }

  const apply = () => {
    for (const b of bodies) sim.set_motion_locks(b, lockX, lockY, lockA);
  };
  controls.appendChild(createCheckbox("Lock X", lockX, (en) => { lockX = en; apply(); }));
  controls.appendChild(createCheckbox("Lock Y", lockY, (en) => { lockY = en; apply(); }));
  controls.appendChild(createCheckbox("Lock Angle", lockA, (en) => { lockA = en; apply(); }));
  return {};
}

function spawnDonut(sim: SimWorld, px: number, py: number, scale: number) {
  // donut.cpp Create
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

function buildSoftBody(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :2655-2686
  sim.add_segment(-20, 0, 20, 0);
  spawnDonut(sim, 0, 10, 2);
  return {};
}

function spawnDoohickey(sim: SimWorld, px: number, py: number, scale: number) {
  // doohickey.cpp Spawn
  const w1 = sim.add_body(px + -5 * scale, py + 3 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(w1, 0, 0, 1 * scale, 1, FRIC, 0, 0.1);
  const w2 = sim.add_body(px + 5 * scale, py + 3 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(w2, 0, 0, 1 * scale, 1, FRIC, 0, 0.1);
  const bar1 = sim.add_body(px + -1.5 * scale, py + 3 * scale, 0, BODY_DYNAMIC);
  sim.attach_capsule(bar1, -3.5 * scale, 0, 3.5 * scale, 0, 0.15 * scale, 1, FRIC, 0);
  const bar2 = sim.add_body(px + 1.5 * scale, py + 3 * scale, 0, BODY_DYNAMIC);
  sim.attach_capsule(bar2, -3.5 * scale, 0, 3.5 * scale, 0, 0.15 * scale, 1, FRIC, 0);
  sim.add_revolute_joint_local(w1, bar1, 0, 0, -3.5 * scale, 0, false, 0, 0, true, 0, 2 * scale, false, 0, 0, false);
  sim.add_revolute_joint_local(w2, bar2, 0, 0, 3.5 * scale, 0, false, 0, 0, true, 0, 2 * scale, false, 0, 0, false);
  sim.add_prismatic_joint_local(
    bar1,
    bar2,
    2 * scale,
    0,
    -2 * scale,
    0,
    1,
    0,
    true,
    -2 * scale,
    2 * scale,
    true,
    0,
    2 * scale,
    true,
    1,
    0.5,
    false,
  );
}

function buildDoohickey(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :2688-2732
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -20, 0, 20, 0);
  sim.attach_box(ground, 1, 1, 0, 1, 0, 0, FRIC, 0);
  let y = 4;
  for (let i = 0; i < 4; i++) {
    spawnDoohickey(sim, 0, y, 0.5);
    y += 2;
  }
  return {};
}

function buildBreakable(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // Simplified Breakable gallery: distance + revolute with force thresholds.
  // Partial: full 6-joint C gallery + mid-step destroy loop not fully mirrored.
  const ground = sim.add_segment(-20, 0, 20, 0);
  let threshold = 1000;
  const joints: number[] = [];
  const a = sim.add_body(-4, 5, 0, BODY_DYNAMIC);
  sim.attach_box(a, 0.5, 0.5, 0, 0, 0, 1, FRIC, 0);
  const j1 = sim.add_distance_joint_ex(ground, a, -4, 8, -4, 5.5, 2.5, false, 0, 0, 0, 0, false, 0, 0, false);
  sim.joint_set_force_threshold(j1, threshold);
  joints.push(j1);

  const b = sim.add_body(4, 5, 0, BODY_DYNAMIC);
  sim.attach_box(b, 0.5, 1, 0, 0, 0, 1, FRIC, 0);
  const j2 = sim.add_revolute_joint(ground, b, 4, 6, false, 0, 0, false, 0, 0, false, 0, 0, false);
  sim.joint_set_force_threshold(j2, threshold);
  joints.push(j2);

  controls.appendChild(
    createSlider("Force threshold", 100, 5000, threshold, 50, (v) => {
      threshold = v;
      for (const j of joints) sim.joint_set_force_threshold(j, v);
    }),
  );
  controls.appendChild(
    createInfoBox("Partial: Breakable — force thresholds set; C's full six-joint break loop + torque paths not fully ported."),
  );

  return {
    afterStep() {
      for (let i = 0; i < joints.length; i++) {
        const j = joints[i]!;
        const ft = sim.joint_constraint_ft(j);
        const mag = Math.hypot(ft[0]!, ft[1]!);
        if (mag > threshold) {
          sim.destroy_joint(j);
          joints[i] = -1;
        }
      }
    },
  };
}

function buildSeparation(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // Separation readout sample — weld + distance with soft params.
  const ground = sim.add_segment(-10, 0, 10, 0);
  const a = sim.add_body(-2, 4, 0, BODY_DYNAMIC);
  sim.attach_box(a, 0.5, 0.5, 0, 0, 0, 1, FRIC, 0);
  const j = sim.add_weld_joint(ground, a, -2, 3, 2, 2, 0.7, 0.7, false);
  const b = sim.add_body(2, 5, 0, BODY_DYNAMIC);
  sim.attach_circle(b, 0, 0, 0.5, 1, FRIC, 0);
  const j2 = sim.add_distance_joint_ex(ground, b, 2, 8, 2, 5, 3, true, 4, 0.7, 1000, 100, false, 0, 0, false);
  controls.appendChild(createInfoBox("Separation — linear/angular joint separation HUD (C Separation sample)."));
  return {
    readoutExtra: () => {
      const s1 = sim.joint_separations(j);
      const s2 = sim.joint_separations(j2);
      return [
        { label: "weld lin/ang", value: `${s1[0]!.toFixed(3)} / ${s1[1]!.toFixed(3)}` },
        { label: "dist lin/ang", value: `${s2[0]!.toFixed(3)} / ${s2[1]!.toFixed(3)}` },
      ];
    },
  };
}

function buildUserConstraint(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // User Constraint — soft manual constraint via impulses (sample_joints UserConstraint).
  sim.add_segment(-20, 0, 20, 0);
  const body = sim.add_body(0, 5, 0, BODY_DYNAMIC);
  sim.attach_box(body, 1, 0.5, 0, 0, 0, 1, FRIC, 0);
  let impulses = [0, 0];
  controls.appendChild(createInfoBox("User Constraint — applies corrective impulses toward (0, 5) each step."));
  return {
    afterStep(dt) {
      if (dt <= 0) return;
      const p = sim.positions();
      const x = p[body * 3]!;
      const y = p[body * 3 + 1]!;
      const dx = 0 - x;
      const dy = 5 - y;
      impulses = [dx * 50, dy * 50];
      sim.apply_linear_impulse_to_center(body, impulses[0]!, impulses[1]!, true);
    },
    readoutExtra: () => [
      { label: "impulse", value: `{${impulses[0]!.toFixed(1)}, ${impulses[1]!.toFixed(1)}}` },
    ],
  };
}

function buildDriving(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // Partial: terrain + Car::Spawn inline; no Truck / teeter extras from full C Driving.
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  const pts: number[] = [];
  // Simplified ground: flat + bumps
  pts.push(-20, 0, 20, 0);
  let x = 20;
  const hs = [0.25, 1, 4, 0, 0, -1, -2, -2, -1.25, 0];
  for (let j = 0; j < 2; j++) {
    for (let i = 0; i < 10; i++) {
      pts.push(x, 0, x + 5, hs[i]!);
      x += 5;
    }
  }
  for (let i = 0; i + 3 < pts.length; i += 4) {
    sim.attach_segment(ground, pts[i]!, pts[i + 1]!, pts[i + 2]!, pts[i + 3]!);
  }
  sim.attach_segment(ground, x, 0, x + 40, 0);
  sim.attach_segment(ground, x + 40, 0, x + 50, 5);
  sim.attach_segment(ground, x + 60, 0, x + 100, 0);

  // Car at origin (car.cpp Spawn scale=1)
  const scale = 1;
  let hertz = 5;
  let damping = 0.7;
  let torque = 2.5;
  let speed = 0;
  const chassisVerts = [-1.5, -0.5, 1.5, -0.5, 1.5, 0, 0, 0.9, -1.15, 0.9, -1.5, 0.2].map(
    (v, i) => v * 0.85 * scale * (i % 2 === 0 ? 1 : 1),
  );
  const chassis = sim.add_polygon(0, 1 * scale, 0, chassisVerts, 0.15 * scale, 1 / scale);
  const rear = sim.add_body(-1 * scale, 0.35 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(rear, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  const front = sim.add_body(1 * scale, 0.4 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(front, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  const rearAxle = sim.add_wheel_joint(
    chassis,
    rear,
    -1 * scale,
    0.35 * scale,
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
    1 * scale,
    0.4 * scale,
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

  const keys = new Set<string>();
  const onKey = (e: KeyboardEvent) => {
    if (e.type === "keydown") keys.add(e.key);
    else keys.delete(e.key);
  };
  window.addEventListener("keydown", onKey);
  window.addEventListener("keyup", onKey);

  controls.appendChild(createSlider("Hertz", 0, 20, hertz, 0.5, (v) => {
    hertz = v;
    sim.wheel_set_spring_hertz(rearAxle, v);
    sim.wheel_set_spring_hertz(frontAxle, v);
  }));
  controls.appendChild(createSlider("Damping", 0, 2, damping, 0.1, (v) => {
    damping = v;
    sim.wheel_set_spring_damping(rearAxle, v);
    sim.wheel_set_spring_damping(frontAxle, v);
  }));
  controls.appendChild(createSlider("Torque", 0, 20, torque, 0.5, (v) => {
    torque = v;
    sim.wheel_set_max_motor_torque(rearAxle, v);
    sim.wheel_set_max_motor_torque(frontAxle, v);
  }));
  controls.appendChild(
    createInfoBox(
      "Partial: Driving — Car spawn + bumpy ground; WASD throttle. Missing C teeter/bridge/truck extras and chase-cam polish.",
    ),
  );

  return {
    beforeStep() {
      let throttle = 0;
      if (keys.has("a") || keys.has("ArrowLeft")) throttle = 1;
      if (keys.has("d") || keys.has("ArrowRight")) throttle = -1;
      speed = throttle * 35;
      sim.wheel_set_motor_speed(rearAxle, speed);
      sim.wheel_set_motor_speed(frontAxle, speed);
    },
    dispose() {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("keyup", onKey);
    },
  };
}

function buildDoor(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // Door — revolute hinge with motor (simplified from C Door sample).
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 10, 0.25, 0, 0, 0, 0, FRIC, 0);
  const door = sim.add_body(1, 1.5, 0, BODY_DYNAMIC);
  sim.attach_box(door, 1, 0.1, 0, 0, 0, 1, FRIC, 0);
  let motor = true;
  let speed = 0;
  const j = sim.add_revolute_joint(ground, door, 0, 1.5, true, -0.5 * PI, 0.5 * PI, true, speed, 100, false, 0, 0, false);
  controls.appendChild(
    createCheckbox("Motor", motor, (en) => {
      motor = en;
      sim.revolute_enable_motor(j, en);
    }),
  );
  controls.appendChild(
    createSlider("Speed", -5, 5, speed, 0.5, (v) => {
      speed = v;
      sim.revolute_set_motor_speed(j, v);
    }),
  );
  controls.appendChild(
    createInfoBox("Partial: Door — single hinged panel; C Door has two-joint / latch extras not yet ported."),
  );
  return {};
}

function buildScene(scene: Scene, sim: SimWorld, controls: HTMLElement): SceneRuntime {
  clearControls(controls);
  switch (scene) {
    case "distance-joint":
      return buildDistanceJoint(sim, controls);
    case "motor-joint":
      return buildMotorJoint(sim, controls);
    case "top-down-friction":
      return buildTopDownFriction(sim, controls);
    case "filter-joint":
      return buildFilterJoint(sim, controls);
    case "revolute":
      return buildRevolute(sim, controls);
    case "prismatic":
      return buildPrismatic(sim, controls);
    case "wheel":
      return buildWheel(sim, controls);
    case "bridge":
      return buildBridge(sim, controls);
    case "ball-chain":
      return buildBallAndChain(sim, controls);
    case "cantilever":
      return buildCantilever(sim, controls);
    case "motion-locks":
      return buildMotionLocks(sim, controls);
    case "soft-body":
      return buildSoftBody(sim, controls);
    case "doohickey":
      return buildDoohickey(sim, controls);
    case "breakable":
      return buildBreakable(sim, controls);
    case "separation":
      return buildSeparation(sim, controls);
    case "user-constraint":
      return buildUserConstraint(sim, controls);
    case "driving":
      return buildDriving(sim, controls);
    case "door":
      return buildDoor(sim, controls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Joints",
    "C <code>sample_joints.cpp</code> RegisterSample ports — distance, motor, " +
      "revolute, prismatic, wheel, bridge, and more.",
    "Drag to grab · P pause · O step · R restart · WASD drives",
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "revolute";

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
    runtime = buildScene(scene, sim, sceneControls);
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
        history.replaceState(null, "", `#/joints/${scene}`);
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
      { label: "Joints", value: String(sim.joint_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
      { label: "Hz", value: String(transport.hertz) },
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
