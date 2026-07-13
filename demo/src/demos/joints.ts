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
  "ragdoll",
  "scissor-lift",
  "gear-lift",
  "door",
  "scale-ragdoll",
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
  ragdoll: "Ragdoll",
  "scissor-lift": "Scissor Lift",
  "gear-lift": "Gear Lift",
  door: "Door",
  "scale-ragdoll": "Scale Ragdoll",
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
  separation: { cx: 0, cy: 8, zoom: 25 }, // :1984-1985
  "user-constraint": { cx: 3, cy: -1, zoom: 25 * 0.15 }, // :2205-2206
  driving: { cx: 0, cy: 5, zoom: 25 * 0.4 }, // :2326-2327
  ragdoll: { cx: 0, cy: 12, zoom: 16.0 }, // :2578-2579 (else branch zoom 16)
  "scissor-lift": { cx: 0, cy: 9, zoom: 25 * 0.4 }, // :2742-2743
  "gear-lift": { cx: 0, cy: 6, zoom: 7.0 }, // :2960-2961
  door: { cx: 0, cy: 0, zoom: 4 }, // :3285-3286
  "scale-ragdoll": { cx: 0, cy: 4.5, zoom: 6.0 }, // :3408-3409
};

const FRIC = 0.6;
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const PI = Math.PI;

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  updateCamera?: (camera: SampleCamera) => void;
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
  // sample_joints.cpp:415-523 TopDownFriction — RandomPolygon via attach_polygon
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
      const body = sim.add_body_ex(x, y, 0, BODY_DYNAMIC, 0, true);
      const rem = (n * i + j) % 4;
      if (rem === 0) sim.attach_capsule(body, -0.25, 0, 0.25, 0, 0.25, 1, FRIC, 0.8);
      else if (rem === 1) sim.attach_circle(body, 0, 0, 0.35, 1, FRIC, 0.8);
      else if (rem === 2) sim.attach_box(body, 0.35, 0.35, 0, 0, 0, 1, FRIC, 0.8);
      else {
        // C RandomPolygon(0.75) then poly.radius = 0.1 (:485-487 / utils.c:18)
        const pts: number[] = [];
        const count = 3 + (rng.next() % 6);
        for (let k = 0; k < count; k++) {
          pts.push(rng.range(-0.75, 0.75), rng.range(-0.75, 0.75));
        }
        sim.attach_polygon(body, pts, 0.1, 1, FRIC, 0.8);
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
  // :1248-1354 — Exact: category/mask 0x1↔0x2 (not groupIndex approx).
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  let frictionTorque = 100;
  const count = 30;
  const joints: number[] = [];
  const hx = 0.5;
  let prev = ground;
  for (let i = 0; i < count; i++) {
    const body = sim.add_body((1 + 2 * i) * hx, count * hx, 0, BODY_DYNAMIC);
    sim.attach_capsule_ex(body, -hx, 0, hx, 0, 0.125, 20, FRIC, 0, false, false, false, 0x1, 0x2);
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
  sim.attach_circle_ex(ball, 0, 0, 4, 20, FRIC, 0, 0, false, false, false, false, 0x2, 0x1);
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
  // sample_joints.cpp:1739-1968 BreakableJoint — 6-joint gallery + force break
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -40, 0, 40, 0);

  let breakForce = 1000;
  let gravityY = sim.get_gravity()[1] ?? -10;
  const joints: (number | null)[] = [];
  const labelPos: { x: number; y: number }[] = [];

  let positionX = -12.5;
  const positionY = 10;

  // distance (:1776-1795)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    const length = 2;
    const pivot1x = positionX;
    const pivot1y = positionY + 1 + length;
    joints.push(
      sim.add_distance_joint_ex(
        ground, body, pivot1x, pivot1y, positionX, positionY + 1, length,
        false, 0, 0, 0, 0, false, 0, 0, true,
      ),
    );
    labelPos.push({ x: pivot1x, y: pivot1y });
  }
  positionX += 5;

  // motor (:1800-1816)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    joints.push(
      sim.add_motor_joint_local(ground, body, positionX, positionY, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 20, true),
    );
    labelPos.push({ x: positionX, y: positionY });
  }
  positionX += 5;

  // prismatic (:1821-1837)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(
      sim.add_prismatic_joint(ground, body, pivotX, pivotY, 1, 0, false, 0, 0, false, 0, 0, false, 0, 0, true),
    );
    labelPos.push({ x: pivotX, y: pivotY });
  }
  positionX += 5;

  // revolute (:1842-1858)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(
      sim.add_revolute_joint(ground, body, pivotX, pivotY, false, 0, 0, false, 0, 0, false, 0, 0, true),
    );
    labelPos.push({ x: pivotX, y: pivotY });
  }
  positionX += 5;

  // weld (:1863-1879)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(sim.add_weld_joint(ground, body, pivotX, pivotY, 0, 0, 0, 0, true));
    labelPos.push({ x: pivotX, y: pivotY });
  }
  positionX += 5;

  // wheel (:1884-1908)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(
      sim.add_wheel_joint(ground, body, pivotX, pivotY, 0, 1, true, -1, 1, true, 1, 10, true, 1, 0.7, true),
    );
    labelPos.push({ x: pivotX, y: pivotY });
  }

  const forces: { x: number; y: number }[] = joints.map(() => ({ x: 0, y: 0 }));

  controls.appendChild(
    createSlider("break force", 0, 10000, breakForce, 1, (v) => {
      breakForce = v;
    }),
  );
  controls.appendChild(
    createSlider("gravity", -50, 50, gravityY, 0.1, (v) => {
      gravityY = v;
      sim.set_gravity(0, v);
    }),
  );

  return {
    // C Step checks forces then Sample::Step (:1933-1956)
    beforeStep() {
      const threshSq = breakForce * breakForce;
      for (let i = 0; i < joints.length; i++) {
        const j = joints[i];
        if (j == null) continue;
        const ft = sim.joint_constraint_ft(j);
        const fx = ft[0]!;
        const fy = ft[1]!;
        if (fx * fx + fy * fy > threshSq) {
          sim.destroy_joint(j);
          joints[i] = null;
        } else {
          forces[i] = { x: fx, y: fy };
        }
      }
    },
    paintOverlay(ctx, camera, canvas) {
      ctx.fillStyle = "#ffffff";
      ctx.font = "12px monospace";
      for (let i = 0; i < joints.length; i++) {
        if (joints[i] == null) continue;
        const pos = worldToScreen(camera, canvas, labelPos[i]!.x, labelPos[i]!.y);
        const f = forces[i]!;
        ctx.fillText(`(${f.x.toFixed(1)}, ${f.y.toFixed(1)})`, pos.x, pos.y);
      }
    },
  };
}

function buildSeparation(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_joints.cpp:1971-2194 JointSeparation — 5 joints + separation overlays
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -40, 0, 40, 0);

  const bodies: number[] = [];
  const joints: number[] = [];
  const labelPos: { x: number; y: number }[] = [];

  let positionX = -20;
  const positionY = 10;

  // distance (:2003-2022)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(body);
    const length = 2;
    const pivot1x = positionX;
    const pivot1y = positionY + 1 + length;
    joints.push(
      sim.add_distance_joint_ex(
        ground, body, pivot1x, pivot1y, positionX, positionY + 1, length,
        false, 0, 0, 0, 0, false, 0, 0, true,
      ),
    );
    labelPos.push({ x: pivot1x, y: pivot1y });
  }
  positionX += 10;

  // prismatic (:2027-2043)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(body);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(
      sim.add_prismatic_joint(ground, body, pivotX, pivotY, 1, 0, false, 0, 0, false, 0, 0, false, 0, 0, true),
    );
    labelPos.push({ x: pivotX, y: pivotY });
  }
  positionX += 10;

  // revolute (:2048-2064)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(body);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(
      sim.add_revolute_joint(ground, body, pivotX, pivotY, false, 0, 0, false, 0, 0, false, 0, 0, true),
    );
    labelPos.push({ x: pivotX, y: pivotY });
  }
  positionX += 10;

  // weld (:2069-2085)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(body);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(sim.add_weld_joint(ground, body, pivotX, pivotY, 0, 0, 0, 0, true));
    labelPos.push({ x: pivotX, y: pivotY });
  }
  positionX += 10;

  // wheel (:2090-2114)
  {
    const body = sim.add_body_ex(positionX, positionY, 0, BODY_DYNAMIC, 1, false);
    sim.attach_box(body, 1, 1, 0, 0, 0, 1, FRIC, 0);
    bodies.push(body);
    const pivotX = positionX - 1;
    const pivotY = positionY;
    joints.push(
      sim.add_wheel_joint(ground, body, pivotX, pivotY, 0, 1, true, -1, 1, true, 1, 10, true, 1, 0.7, true),
    );
    labelPos.push({ x: pivotX, y: pivotY });
  }

  let impulse = 500;
  let jointHertz = 60;
  let jointDamping = 2;
  let gravityY = sim.get_gravity()[1] ?? -10;

  const applyTuning = () => {
    for (const j of joints) sim.joint_set_constraint_tuning(j, jointHertz, jointDamping);
  };

  controls.appendChild(
    createSlider("gravity", -500, 500, gravityY, 1, (v) => {
      gravityY = v;
      sim.set_gravity(0, v);
    }),
  );
  controls.appendChild(createSlider("magnitude", 0, 1000, impulse, 1, (v) => { impulse = v; }));
  controls.appendChild(
    createSlider("hertz", 15, 120, jointHertz, 1, (v) => {
      jointHertz = v;
      applyTuning();
    }),
  );
  controls.appendChild(
    createSlider("damping", 0, 10, jointDamping, 0.1, (v) => {
      jointDamping = v;
      applyTuning();
    }),
  );
  controls.appendChild(
    createButton("impulse", () => {
      for (const b of bodies) {
        const wp = sim.body_world_point(b, 1, 1);
        sim.apply_linear_impulse(b, impulse, -impulse, wp[0]!, wp[1]!, true);
      }
    }),
  );

  return {
    paintOverlay(ctx, camera, canvas) {
      ctx.fillStyle = "#ffffff";
      ctx.font = "12px monospace";
      for (let i = 0; i < joints.length; i++) {
        const sep = sim.joint_separations(joints[i]!);
        const linear = sep[0]!;
        const angularDeg = (180 * sep[1]!) / PI;
        const pos = worldToScreen(camera, canvas, labelPos[i]!.x, labelPos[i]!.y);
        ctx.fillText(`${linear.toFixed(2)} m, ${angularDeg.toFixed(1)} deg`, pos.x, pos.y);
      }
    },
  };
}

function buildUserConstraint(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_joints.cpp:2197-2315 UserConstraint — dual-anchor soft tether solver
  const body = sim.add_body_ex(0, 0, 0, BODY_DYNAMIC, 1, true);
  sim.set_angular_damping(body, 0.5);
  sim.set_linear_damping(body, 0.2);
  sim.attach_box(body, 1, 0.5, 0, 0, 0, 20, FRIC, 0);

  const localAnchors = [
    { x: 1, y: -0.5 },
    { x: 1, y: 0.5 },
  ];
  const impulses = [0, 0];
  let lastInvTimeStep = 60;
  const lineColors = ["#E0FFFF", "#EE82EE"]; // light cyan / violet when taut
  const drawLines: { ax: number; ay: number; bx: number; by: number; color: string }[] = [
    { ax: 3, ay: 0, bx: 0, by: 0, color: lineColors[0]! },
    { ax: 3, ay: 0, bx: 0, by: 0, color: lineColors[0]! },
  ];

  const softHertz = 3;
  const zeta = 0.7;
  const maxForce = 1000;

  controls.appendChild(
    createInfoBox("User Constraint — soft dual-anchor tethers to (3,0); forces shown in readout."),
  );

  return {
    // C applies after Sample::Step when not paused (:2226-2303)
    afterStep(dt) {
      if (dt <= 0) return;
      const timeStep = dt;
      lastInvTimeStep = 1 / timeStep;
      const omega = 2 * PI * softHertz;
      const sigma = 2 * zeta + timeStep * omega;
      const s = timeStep * omega * sigma;
      const impulseCoefficient = 1 / (1 + s);
      const massCoefficient = s * impulseCoefficient;
      const biasCoefficient = omega / sigma;

      const mass = sim.get_mass(body);
      const invMass = mass < 0.0001 ? 0 : 1 / mass;
      const inertiaTensor = sim.get_rotational_inertia(body);
      const invI = inertiaTensor < 0.0001 ? 0 : 1 / inertiaTensor;

      let vx = sim.get_linear_velocity(body)[0]!;
      let vy = sim.get_linear_velocity(body)[1]!;
      let omegaB = sim.get_angular_velocity(body);
      const pos = sim.positions();
      const pBx = pos[body * 3]!;
      const pBy = pos[body * 3 + 1]!;

      for (let i = 0; i < 2; i++) {
        const anchorA = { x: 3, y: 0 };
        const anchorB = sim.body_world_point(body, localAnchors[i]!.x, localAnchors[i]!.y);
        const bx = anchorB[0]!;
        const by = anchorB[1]!;
        const dx = bx - anchorA.x;
        const dy = by - anchorA.y;
        const slackLength = 1;
        const length = Math.sqrt(dx * dx + dy * dy);
        const C = length - slackLength;
        if (C < 0 || length < 0.001) {
          drawLines[i] = { ax: anchorA.x, ay: anchorA.y, bx, by, color: lineColors[0]! };
          impulses[i] = 0;
          continue;
        }
        drawLines[i] = { ax: anchorA.x, ay: anchorA.y, bx, by, color: lineColors[1]! };
        const axisX = dx / length;
        const axisY = dy / length;
        const rBx = bx - pBx;
        const rBy = by - pBy;
        const Jb = rBx * axisY - rBy * axisX;
        const K = invMass + Jb * invI * Jb;
        const invK = K < 0.0001 ? 0 : 1 / K;
        const Cdot = vx * axisX + vy * axisY + Jb * omegaB;
        const impulse = -massCoefficient * invK * (Cdot + biasCoefficient * C);
        const appliedImpulse = Math.max(-maxForce * timeStep, Math.min(0, impulse));
        vx += invMass * appliedImpulse * axisX;
        vy += invMass * appliedImpulse * axisY;
        omegaB += appliedImpulse * invI * Jb;
        impulses[i] = appliedImpulse;
      }

      sim.set_linear_velocity(body, vx, vy);
      sim.set_angular_velocity(body, omegaB);
      void impulseCoefficient;
    },
    paintOverlay(ctx, camera, canvas) {
      const o = worldToScreen(camera, canvas, 0, 0);
      const ox = worldToScreen(camera, canvas, 1, 0);
      const oy = worldToScreen(camera, canvas, 0, 1);
      ctx.strokeStyle = "#22c55e";
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(ox.x, ox.y);
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(oy.x, oy.y);
      ctx.stroke();
      for (const line of drawLines) {
        const a = worldToScreen(camera, canvas, line.ax, line.ay);
        const b = worldToScreen(camera, canvas, line.bx, line.by);
        ctx.strokeStyle = line.color;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
    },
    readoutExtra: () => {
      const f0 = impulses[0]! * lastInvTimeStep;
      const f1 = impulses[1]! * lastInvTimeStep;
      return [{ label: "forces", value: `${f0.toPrecision(6)}, ${f1.toPrecision(6)}` }];
    },
  };
}

function buildDriving(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_joints.cpp:2318-2569 Driving — chain terrain, teeter, bridge, boxes, Car
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);

  // Chain loop filled reverse (:2336-2367)
  const points: number[] = new Array(25 * 2);
  let count = 24;
  points[count * 2] = -20;
  points[count * 2 + 1] = -20;
  count--;
  points[count * 2] = -20;
  points[count * 2 + 1] = 0;
  count--;
  points[count * 2] = 20;
  points[count * 2 + 1] = 0;
  count--;

  const hs = [0.25, 1, 4, 0, 0, -1, -2, -2, -1.25, 0];
  let x = 20;
  const dx = 5;
  for (let j = 0; j < 2; j++) {
    for (let i = 0; i < 10; i++) {
      points[count * 2] = x + dx;
      points[count * 2 + 1] = hs[i]!;
      count--;
      x += dx;
    }
  }
  points[count * 2] = x + 40;
  points[count * 2 + 1] = 0;
  count--;
  points[count * 2] = x + 40;
  points[count * 2 + 1] = -20;
  count--;
  sim.attach_chain(ground, points, true);

  // flat after bridge / jump / corner (:2369-2387)
  x += 80;
  sim.attach_segment(ground, x, 0, x + 40, 0);
  x += 40;
  sim.attach_segment(ground, x, 0, x + 10, 5);
  x += 20;
  sim.attach_segment(ground, x, 0, x + 40, 0);
  x += 40;
  sim.attach_segment(ground, x, 0, x, 20);

  // Teeter (:2390-2412)
  {
    const teeter = sim.add_body(140, 1, 0, BODY_DYNAMIC);
    sim.set_angular_velocity(teeter, 1);
    sim.attach_box(teeter, 10, 0.25, 0, 0, 0, 1, FRIC, 0);
    sim.add_revolute_joint(
      ground, teeter, 140, 1, true, (-8 * PI) / 180, (8 * PI) / 180,
      false, 0, 0, false, 0, 0, false,
    );
  }

  // Bridge N=20 (:2414-2449)
  {
    const N = 20;
    let prev = ground;
    for (let i = 0; i < N; i++) {
      const body = sim.add_body(161 + 2 * i, -0.125, 0, BODY_DYNAMIC);
      sim.attach_capsule(body, -1, 0, 1, 0, 0.125, 1, FRIC, 0);
      sim.add_revolute_joint(prev, body, 160 + 2 * i, -0.125, false, 0, 0, false, 0, 0, false, 0, 0, false);
      prev = body;
    }
    sim.add_revolute_joint(prev, ground, 160 + 2 * N, -0.125, false, 0, 0, true, 0, 50, false, 0, 0, false);
  }

  // Boxes (:2451-2483)
  for (let i = 0; i < 5; i++) {
    const box = sim.add_body(230, 0.5 + i, 0, BODY_DYNAMIC);
    sim.attach_box(box, 0.5, 0.5, 0, 0, 0, 0.25, 0.25, 0.25);
  }

  // Car::Spawn (:2485-2493 / car.cpp:21)
  const scale = 1;
  let hertz = 5;
  let damping = 0.7;
  let torque = 5;
  let speed = 35;
  let throttle = 0;

  const verts = [-1.5, -0.5, 1.5, -0.5, 1.5, 0, 0, 0.9, -1.15, 0.9, -1.5, 0.2].map(
    (v) => v * 0.85 * scale,
  );
  const chassis = sim.add_body(0, 1 * scale, 0, BODY_DYNAMIC);
  sim.attach_polygon(chassis, verts, 0.15 * scale, 1 / scale, 0.2, 0);
  const rear = sim.add_body_ccd(-1 * scale, 0.35 * scale, 0, BODY_DYNAMIC, 1, false, true, true);
  sim.attach_circle_rolling(rear, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  const front = sim.add_body_ccd(1 * scale, 0.4 * scale, 0, BODY_DYNAMIC, 1, false, true, true);
  sim.attach_circle_rolling(front, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  const rearAxle = sim.add_wheel_joint(
    chassis, rear, -1 * scale, 0.35 * scale, 0, 1, true, -0.25 * scale, 0.25 * scale,
    true, 0, torque, true, hertz, damping, false,
  );
  const frontAxle = sim.add_wheel_joint(
    chassis, front, 1 * scale, 0.4 * scale, 0, 1, true, -0.25 * scale, 0.25 * scale,
    true, 0, torque, true, hertz, damping, false,
  );

  const keys = new Set<string>();
  const onKey = (e: KeyboardEvent) => {
    if (e.type === "keydown") keys.add(e.key.toLowerCase());
    else keys.delete(e.key.toLowerCase());
  };
  window.addEventListener("keydown", onKey);
  window.addEventListener("keyup", onKey);

  controls.appendChild(
    createSlider("Spring Hertz", 0, 20, hertz, 1, (v) => {
      hertz = v;
      sim.wheel_set_spring_hertz(rearAxle, v);
      sim.wheel_set_spring_hertz(frontAxle, v);
    }),
  );
  controls.appendChild(
    createSlider("Damping Ratio", 0, 10, damping, 0.1, (v) => {
      damping = v;
      sim.wheel_set_spring_damping(rearAxle, v);
      sim.wheel_set_spring_damping(frontAxle, v);
    }),
  );
  controls.appendChild(
    createSlider("Speed", 0, 50, speed, 1, (v) => {
      speed = v;
      sim.wheel_set_motor_speed(rearAxle, throttle * speed);
      sim.wheel_set_motor_speed(frontAxle, throttle * speed);
    }),
  );
  controls.appendChild(
    createSlider("Torque", 0, 10, torque, 0.1, (v) => {
      torque = v;
      sim.wheel_set_max_motor_torque(rearAxle, v);
      sim.wheel_set_max_motor_torque(frontAxle, v);
    }),
  );
  controls.appendChild(createInfoBox("Keys: left = a, brake = s, right = d"));

  return {
    beforeStep() {
      if (keys.has("a")) {
        throttle = 1;
        sim.wheel_set_motor_speed(rearAxle, speed);
        sim.wheel_set_motor_speed(frontAxle, speed);
      }
      if (keys.has("s")) {
        throttle = 0;
        sim.wheel_set_motor_speed(rearAxle, 0);
        sim.wheel_set_motor_speed(frontAxle, 0);
      }
      if (keys.has("d")) {
        throttle = -1;
        sim.wheel_set_motor_speed(rearAxle, -speed);
        sim.wheel_set_motor_speed(frontAxle, -speed);
      }
    },
    updateCamera(camera) {
      const pos = sim.positions();
      camera.centerX = pos[chassis * 3]!;
    },
    readoutExtra: () => {
      const v = sim.get_linear_velocity(chassis);
      const kph = v[0]! * 3.6;
      return [
        { label: "Keys", value: "a / s / d" },
        { label: "speed in kph", value: kph.toPrecision(2) },
      ];
    },
    dispose() {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("keyup", onKey);
    },
  };
}

function buildDoor(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_joints.cpp:3277-3398 Door — single revolute spring hinge + impulse
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);

  let enableLimit = true;
  let impulse = 50000;
  let translationError = 0;
  let jointHertz = 240;
  let jointDamping = 1;

  const door = sim.add_body_ex(0, 1.5, 0, BODY_DYNAMIC, 0, true);
  sim.attach_box(door, 0.1, 1.5, 0, 0, 0, 1000, FRIC, 0);

  // local frames: A (0,0), B (0,-1.5) via world pivot (0,0)
  const j = sim.add_revolute_joint(
    ground, door, 0, 0, true, -0.5 * PI, 0.5 * PI, false, 0, 0, true, 1, 0.5, false,
  );
  sim.revolute_set_target_angle(j, 0);
  sim.joint_set_constraint_tuning(j, jointHertz, jointDamping);
  if (!enableLimit) sim.revolute_enable_limit(j, false);

  controls.appendChild(
    createButton("impulse", () => {
      const wp = sim.body_world_point(door, 0, 1.5);
      sim.apply_linear_impulse(door, impulse, 0, wp[0]!, wp[1]!, true);
      translationError = 0;
    }),
  );
  controls.appendChild(
    createSlider("magnitude", 1000, 100000, impulse, 1000, (v) => {
      impulse = v;
    }),
  );
  controls.appendChild(
    createSlider("hertz", 15, 480, jointHertz, 1, (v) => {
      jointHertz = v;
      sim.joint_set_constraint_tuning(j, jointHertz, jointDamping);
    }),
  );
  controls.appendChild(
    createSlider("damping", 0, 10, jointDamping, 0.1, (v) => {
      jointDamping = v;
      sim.joint_set_constraint_tuning(j, jointHertz, jointDamping);
    }),
  );
  controls.appendChild(
    createCheckbox("limit", enableLimit, (en) => {
      enableLimit = en;
      sim.revolute_enable_limit(j, en);
    }),
  );

  return {
    afterStep() {
      const sep = sim.joint_separations(j);
      translationError = Math.max(translationError, sep[0]!);
    },
    paintOverlay(ctx, camera, canvas) {
      const wp = sim.body_world_point(door, 0, 1.5);
      const s = worldToScreen(camera, canvas, wp[0]!, wp[1]!);
      ctx.fillStyle = "#bdb76b"; // dark khaki
      ctx.beginPath();
      ctx.arc(s.x, s.y, 5, 0, 2 * PI);
      ctx.fill();
    },
    readoutExtra: () => [{ label: "translation error", value: String(translationError) }],
  };
}

function buildRagdoll(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_joints.cpp:2568-2650 Ragdoll — CreateHuman + contact tuning
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -20, 0, 20, 0);

  let jointFrictionTorque = 0.03;
  let jointHertz = 5.0;
  let jointDampingRatio = 0.5;
  let human = -1;

  function spawn() {
    if (human >= 0 && sim.human_is_spawned(human)) sim.destroy_human(human);
    // :2607-2608 CreateHuman(…, {0,25}, 1, friction, hertz, damping, 1, nullptr, false)
    human = sim.create_human(0, 25, 1.0, jointFrictionTorque, jointHertz, jointDampingRatio, 1, false, 0);
  }
  spawn();
  sim.set_contact_tuning(240.0, 0.0, 2.0); // :2602

  controls.appendChild(
    createSlider("Friction", 0, 1, jointFrictionTorque, 0.01, (v) => {
      jointFrictionTorque = v;
      if (sim.human_is_spawned(human)) sim.human_set_joint_friction_torque(human, v);
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 10, jointHertz, 0.1, (v) => {
      jointHertz = v;
      if (sim.human_is_spawned(human)) sim.human_set_joint_spring_hertz(human, v);
    }),
  );
  controls.appendChild(
    createSlider("Damping", 0, 4, jointDampingRatio, 0.1, (v) => {
      jointDampingRatio = v;
      if (sim.human_is_spawned(human)) sim.human_set_joint_damping_ratio(human, v);
    }),
  );
  controls.appendChild(
    createButton("Respawn", () => {
      spawn();
    }),
  );
  controls.appendChild(
    createInfoBox(
      "Exact: <code>CreateHuman</code> ragdoll (<code>shared/human.c</code>). " +
        "C <code>sample_joints.cpp</code> Ragdoll.",
    ),
  );
  return {};
}

function buildScaleRagdoll(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_joints.cpp:3400-3459 Scale Ragdoll — Human_SetScale
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 20, 1, 0, -1, 0, 0, FRIC, 0);

  let scale = 1.0;
  let human = -1;

  function spawn() {
    if (human >= 0 && sim.human_is_spawned(human)) sim.destroy_human(human);
    // :3430-3435
    human = sim.create_human(0, 5, scale, 0.03, 1.0, 0.5, 1, false, 0);
    sim.human_apply_random_angular_impulse(human, 0.1);
  }
  spawn();

  controls.appendChild(
    createSlider("Scale", 0.1, 10, scale, 0.01, (v) => {
      scale = v;
      if (sim.human_is_spawned(human)) sim.human_set_scale(human, v);
    }),
  );
  controls.appendChild(
    createInfoBox(
      "Exact: <code>CreateHuman</code> + <code>Human_SetScale</code> " +
        "(<code>shared/human.c</code>). C <code>sample_joints.cpp</code> Scale Ragdoll.",
    ),
  );
  return {};
}

/** C Sample::ParsePath — Y-flipped (sample.cpp:1047). */
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
      if ("MLHVmlhv".includes(command)) ptr += 2;
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

function spawnCarAt(
  sim: SimWorld,
  px: number,
  py: number,
  scale: number,
  hertz: number,
  damping: number,
  torque: number,
) {
  const verts = [-1.5, -0.5, 1.5, -0.5, 1.5, 0, 0, 0.9, -1.15, 0.9, -1.5, 0.2].map(
    (v) => v * 0.85 * scale,
  );
  const chassis = sim.add_polygon(px, py + 1 * scale, 0, verts, 0.15 * scale, 1 / scale);
  const rear = sim.add_body(px + -1 * scale, py + 0.35 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(rear, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  const front = sim.add_body(px + 1 * scale, py + 0.4 * scale, 0, BODY_DYNAMIC);
  sim.attach_circle_rolling(front, 0, 0, 0.4 * scale, 2 / scale, 1.5, 0, 0.1);
  sim.add_wheel_joint(
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
  sim.add_wheel_joint(
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
}

function tune(sim: SimWorld, jid: number, hertz: number, damping: number) {
  sim.joint_set_constraint_tuning(jid, hertz, damping);
}

function buildScissorLift(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // Exact: sample_joints.cpp:2734-2948
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -20, 0, 20, 0);

  const constraintDamping = 20.0;
  const constraintHertz = 240.0;
  let baseId1 = ground;
  let baseId2 = ground;
  let baseAnchor1 = { x: -2.5, y: 0.2 };
  let baseAnchor2 = { x: 2.5, y: 0.2 };
  let y = 0.5;
  let linkId1 = -1;
  const N = 3;

  for (let i = 0; i < N; ++i) {
    const body1 = sim.add_body_sleep_threshold(0, y, 0.15, BODY_DYNAMIC, 0.01);
    sim.attach_capsule(body1, -2.5, 0, 2.5, 0, 0.15, 1, 0.3, 0);
    const body2 = sim.add_body_sleep_threshold(0, y, -0.15, BODY_DYNAMIC, 0.01);
    sim.attach_capsule(body2, -2.5, 0, 2.5, 0, 0.15, 1, 0.3, 0);
    if (i === 1) linkId1 = body2;

    const left = sim.add_revolute_joint_local(
      baseId1,
      body1,
      baseAnchor1.x,
      baseAnchor1.y,
      -2.5,
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
      i === 0,
    );
    tune(sim, left, constraintHertz, constraintDamping);

    if (i === 0) {
      const wh = sim.add_wheel_joint_local(
        baseId2,
        body2,
        baseAnchor2.x,
        baseAnchor2.y,
        2.5,
        0,
        false,
        1,
        0.7,
        true,
      );
      tune(sim, wh, constraintHertz, constraintDamping);
    } else {
      const right = sim.add_revolute_joint_local(
        baseId2,
        body2,
        baseAnchor2.x,
        baseAnchor2.y,
        2.5,
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
      tune(sim, right, constraintHertz, constraintDamping);
    }

    const mid = sim.add_revolute_joint_local(
      body1,
      body2,
      0,
      0,
      0,
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
    tune(sim, mid, constraintHertz, constraintDamping);

    baseId1 = body2;
    baseId2 = body1;
    baseAnchor1 = { x: -2.5, y: 0 };
    baseAnchor2 = { x: 2.5, y: 0 };
    y += 1.0;
  }

  const platform = sim.add_body_sleep_threshold(0, y, 0, BODY_DYNAMIC, 0.01);
  sim.attach_box(platform, 3.0, 0.2, 0, 0, 0, 1, 0.3, 0);

  const platL = sim.add_revolute_joint_local(
    platform,
    baseId1,
    -2.5,
    -0.4,
    baseAnchor1.x,
    baseAnchor1.y,
    false,
    0,
    0,
    false,
    0,
    0,
    false,
    0,
    0,
    true,
  );
  tune(sim, platL, constraintHertz, constraintDamping);
  const platR = sim.add_wheel_joint_local(
    platform,
    baseId2,
    2.5,
    -0.4,
    baseAnchor2.x,
    baseAnchor2.y,
    false,
    1,
    0.7,
    true,
  );
  tune(sim, platR, constraintHertz, constraintDamping);

  let enableMotor = false;
  let motorSpeed = 0.25;
  let motorForce = 2000;
  const lift = sim.add_distance_joint_local_motor(
    ground,
    linkId1,
    -2.5,
    0.2,
    0.5,
    0,
    0,
    true,
    0,
    0,
    true,
    0.2,
    5.5,
    enableMotor,
    motorSpeed,
    motorForce,
    false,
  );

  spawnCarAt(sim, 0, y + 2.0, 1.0, 3.0, 0.7, 0.0);

  controls.appendChild(
    createCheckbox("Motor", enableMotor, (v) => {
      enableMotor = v;
      sim.distance_enable_motor(lift, v);
      sim.joint_wake_bodies(lift);
    }),
  );
  controls.appendChild(
    createSlider("Max Force", 0, 3000, motorForce, 1, (v) => {
      motorForce = v;
      sim.distance_set_max_motor_force(lift, v);
      sim.joint_wake_bodies(lift);
    }),
  );
  controls.appendChild(
    createSlider("Speed", -0.3, 0.3, motorSpeed, 0.01, (v) => {
      motorSpeed = v;
      sim.distance_set_motor_speed(lift, v);
      sim.joint_wake_bodies(lift);
    }),
  );
  controls.appendChild(
    createInfoBox(
      "Exact: scissor + distance-motor lift + Car::Spawn. Prefer 8 sub-steps (C). sample_joints.cpp Scissor Lift.",
    ),
  );
  return {};
}

const GEAR_PATH =
  "m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l " +
  "-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v " +
  "-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l " +
  "-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166";

function buildGearLift(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // Exact: sample_joints.cpp:2952-3272
  const pts = parsePath(GEAR_PATH, -120, -200, 64, 0.2);
  const ground = sim.add_chain_color(pts, true, 0.6, 0x8fbc8f);

  const gearRadius = 1.0;
  const toothHalfWidth = 0.09;
  const toothHalfHeight = 0.06;
  const toothRadius = 0.03;
  const linkHalfLength = 0.07;
  const linkRadius = 0.05;
  const linkCount = 40;
  const doorHalfHeight = 1.5;
  const gearPosition1 = { x: -4.25, y: 9.75 };
  const gearPosition2 = { x: gearPosition1.x + 2.0, y: gearPosition1.y + 1.0 };
  const linkAttach = {
    x: gearPosition2.x + gearRadius + 2.0 * toothHalfWidth + toothRadius,
    y: gearPosition2.y,
  };
  const doorPosition = {
    x: linkAttach.x,
    y: linkAttach.y - 2.0 * linkCount * linkHalfLength - doorHalfHeight,
  };

  const COLOR_SADDLE = 0x8b4513;
  const COLOR_GRAY = 0x808080;
  const COLOR_STEEL = 0xb0c4de;
  const COLOR_CYAN = 0x008b8b;

  const driver = sim.add_body(gearPosition1.x, gearPosition1.y, 0, BODY_DYNAMIC);
  const hub1 = sim.attach_circle_ex(
    driver,
    0,
    0,
    gearRadius,
    1,
    0.1,
    0,
    0,
    false,
    false,
    false,
    false,
    0,
    0,
  );
  sim.shape_set_custom_color(hub1, COLOR_SADDLE);
  {
    const dq = (2 * PI) / 16;
    let rotation = 0;
    for (let i = 0; i < 16; ++i) {
      const cx = Math.cos(rotation) * (gearRadius + toothHalfHeight);
      const cy = Math.sin(rotation) * (gearRadius + toothHalfHeight);
      sim.attach_offset_rounded_box_color(
        driver,
        toothHalfWidth,
        toothHalfHeight,
        cx,
        cy,
        rotation,
        toothRadius,
        1,
        0.1,
        COLOR_GRAY,
      );
      rotation += dq;
    }
  }
  let enableMotor = true;
  let motorTorque = 80.0;
  let motorSpeed = 0.0;
  const driverJoint = sim.add_revolute_joint(
    ground,
    driver,
    gearPosition1.x,
    gearPosition1.y,
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

  const follower = sim.add_body(gearPosition2.x, gearPosition2.y, 0, BODY_DYNAMIC);
  const hub2 = sim.attach_circle_ex(
    follower,
    0,
    0,
    gearRadius,
    1,
    0.1,
    0,
    0,
    false,
    false,
    false,
    false,
    0,
    0,
  );
  sim.shape_set_custom_color(hub2, COLOR_SADDLE);
  {
    const dq = (2 * PI) / 16;
    let rotation = 0;
    for (let i = 0; i < 16; ++i) {
      const cx = Math.cos(rotation) * (gearRadius + toothHalfWidth);
      const cy = Math.sin(rotation) * (gearRadius + toothHalfWidth);
      sim.attach_offset_rounded_box_color(
        follower,
        toothHalfWidth,
        toothHalfHeight,
        cx,
        cy,
        rotation,
        toothRadius,
        1,
        0.1,
        COLOR_GRAY,
      );
      rotation += dq;
    }
  }
  sim.add_revolute_joint_angled(
    ground,
    follower,
    gearPosition2.x,
    gearPosition2.y,
    0.25 * PI,
    true,
    -0.3 * PI,
    0.8 * PI,
    true,
    0.5,
  );

  let prev = follower;
  let position = { x: linkAttach.x, y: linkAttach.y - linkHalfLength };
  let lastLink = follower;
  for (let i = 0; i < 40; ++i) {
    const body = sim.add_body(position.x, position.y, 0, BODY_DYNAMIC);
    const cap = sim.attach_capsule_ex(
      body,
      0,
      -linkHalfLength,
      0,
      linkHalfLength,
      linkRadius,
      2,
      0.3,
      0,
      false,
      false,
      false,
      0,
      0,
    );
    sim.shape_set_custom_color(cap, COLOR_STEEL);
    const pivotY = position.y + linkHalfLength;
    sim.add_revolute_joint(
      prev,
      body,
      position.x,
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
    position = { x: position.x, y: position.y - 2.0 * linkHalfLength };
    prev = body;
    lastLink = body;
  }

  const door = sim.add_body(doorPosition.x, doorPosition.y, 0, BODY_DYNAMIC);
  const doorShape = sim.attach_box_ex(
    door,
    0.15,
    doorHalfHeight,
    0,
    0,
    0,
    1,
    0.1,
    0,
    false,
    false,
    false,
    false,
    false,
    0,
    0,
  );
  sim.shape_set_custom_color(doorShape, COLOR_CYAN);
  {
    const pivotY = doorPosition.y + doorHalfHeight;
    sim.add_revolute_joint(
      lastLink,
      door,
      doorPosition.x,
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
  sim.add_prismatic_joint_local(
    ground,
    door,
    doorPosition.x,
    doorPosition.y,
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

  const rng = makeXorShift(12345);
  const colors = [0x808080, 0xdcdcdc, 0xd3d3d3, 0x778899, 0xa9a9a9];
  let yy = 4.25;
  for (let i = 0; i < 20; ++i) {
    let xx = -3.15;
    for (let j = 0; j < 10; ++j) {
      const body = sim.add_body(xx, yy, 0, BODY_DYNAMIC);
      const count = 3 + (rng.next() % 6);
      const poly: number[] = [];
      for (let k = 0; k < count; ++k) {
        poly.push(rng.range(-0.1, 0.1), rng.range(-0.1, 0.1));
      }
      const radius = rng.range(0.01, 0.02);
      const colorIdx = Math.min(4, Math.floor(rng.range(0, 5)));
      const sid = sim.attach_polygon_mat(body, poly, radius, 1, 0.3, 0, 0.3, 0);
      sim.shape_set_custom_color(sid, colors[colorIdx]!);
      xx += 0.2;
    }
    yy += 0.2;
  }

  const keys = new Set<string>();
  const onKey = (e: KeyboardEvent) => {
    if (e.type === "keydown") keys.add(e.key.toLowerCase());
    else keys.delete(e.key.toLowerCase());
  };
  window.addEventListener("keydown", onKey);
  window.addEventListener("keyup", onKey);

  controls.appendChild(
    createCheckbox("Motor", enableMotor, (v) => {
      enableMotor = v;
      sim.revolute_enable_motor(driverJoint, v);
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
  controls.appendChild(
    createInfoBox("Exact: ParsePath ground, meshed gears, door prismatic. A/D adjusts motor speed."),
  );

  return {
    beforeStep: () => {
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
    dispose: () => {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("keyup", onKey);
    },
  };
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
    case "ragdoll":
      return buildRagdoll(sim, controls);
    case "scissor-lift":
      return buildScissorLift(sim, controls);
    case "gear-lift":
      return buildGearLift(sim, controls);
    case "door":
      return buildDoor(sim, controls);
    case "scale-ragdoll":
      return buildScaleRagdoll(sim, controls);
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
    { category: "Joints", samplesShell: true }
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
    if (scene === "scissor-lift") transport.subSteps = 8;
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
        if (scene === "scissor-lift") transport.subSteps = 8;
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    route: "joints",
    category: "Joints",
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

    runtime.updateCamera?.(camera);
    paintSampleDraw(canvas, camera, sim);
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
