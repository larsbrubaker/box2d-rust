// Bodies — nine RegisterSample ports from sample_bodies.cpp.
// C citations use sample_bodies.cpp line numbers at the pinned submodule.

import {
  createButton,
  createButtonGroup,
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
  "body-type",
  "weeble",
  "sleep",
  "bad",
  "pivot",
  "kinematic",
  "mixed-locks",
  "set-velocity",
  "wake-touching",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("bodies", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "body-type": "Body Type",
  weeble: "Weeble",
  sleep: "Sleep",
  bad: "Bad",
  pivot: "Pivot",
  kinematic: "Kinematic",
  "mixed-locks": "Mixed Locks",
  "set-velocity": "Set Velocity",
  "wake-touching": "Wake Touching",
};

/** C camera.center / camera.zoom from each sample ctor (half-height zoom). */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  // sample_bodies.cpp:19-20
  "body-type": { cx: 0.8, cy: 6.4, zoom: 25.0 * 0.4 },
  // :313-314
  weeble: { cx: 2.3, cy: 10.0, zoom: 25.0 * 0.5 },
  // :423-424
  sleep: { cx: 3.0, cy: 50.0, zoom: 25.0 * 2.2 },
  // :668-669
  bad: { cx: 2.3, cy: 10.0, zoom: 25.0 * 0.5 },
  // :746-747
  pivot: { cx: 0.8, cy: 6.4, zoom: 25.0 * 0.4 },
  // :817-818
  kinematic: { cx: 0.0, cy: 0.0, zoom: 4.0 },
  // :888-889
  "mixed-locks": { cx: 0.0, cy: 2.5, zoom: 3.5 },
  // :997-998
  "set-velocity": { cx: 0.0, cy: 2.5, zoom: 3.5 },
  // :1051-1052
  "wake-touching": { cx: 0.0, cy: 4.0, zoom: 8.0 },
};

/** C default shape material friction (types/shape.rs / b2DefaultSurfaceMaterial). */
const FRIC = 0.6;

const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

interface SceneRuntime {
  /** Per-frame work after transport dt is known, before sim.step. */
  beforeStep?: (dt: number) => void;
  /** After sim.step — forces, text overlays, kinematic drive follow-up. */
  afterStep?: (dt: number) => void;
  /** Extra canvas draw (C DrawCircle / DrawLine / DrawPoint). */
  paintOverlay?: (ctx: CanvasRenderingContext2D, camera: SampleCamera, canvas: HTMLCanvasElement) => void;
  /** HUD rows beyond the shared counters. */
  readoutExtra?: () => { label: string; value: string }[];
  /** Tear down scene-only DOM listeners. */
  dispose?: () => void;
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

function posOf(sim: SimWorld, index: number): { x: number; y: number; angle: number } {
  const p = sim.positions();
  return { x: p[index * 3]!, y: p[index * 3 + 1]!, angle: p[index * 3 + 2]! };
}

// ---------------------------------------------------------------------------
// Scene builders — literal C values
// ---------------------------------------------------------------------------

function buildBodyType(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:26-199
  const ground = sim.add_segment(-20.0, 0.0, 20.0, 0.0);

  const attachment = sim.add_body(-2.0, 3.0, 0.0, BODY_DYNAMIC);
  sim.attach_box(attachment, 0.5, 2.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  let type = BODY_STATIC; // m_type = b2_staticBody (:23)
  let isEnabled = true; // :24

  const secondAttachment = sim.add_body(3.0, 3.0, 0.0, type);
  sim.attach_box(secondAttachment, 0.5, 2.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  // Platform: offset box hx=0.5 hy=4 at local (4,0) rotated 0.5π (:75)
  const platform = sim.add_body(-4.0, 5.0, 0.0, type);
  sim.attach_box(platform, 0.5, 4.0, 4.0, 0.0, 0.5 * Math.PI, 2.0, FRIC, 0.0);

  // Revolute motors at pivots (-2,5) and (3,5); maxMotorTorque=50 (:81-98)
  sim.add_revolute_joint(
    attachment, platform, -2.0, 5.0,
    false, 0, 0, true, 0.0, 50.0, false, 0, 0, false,
  );
  sim.add_revolute_joint(
    secondAttachment, platform, 3.0, 5.0,
    false, 0, 0, true, 0.0, 50.0, false, 0, 0, false,
  );

  // Prismatic ground↔platform at (0,5), axis +X, limits ±10, motor force 1000 (:100-113)
  sim.add_prismatic_joint(
    ground, platform, 0.0, 5.0, 1.0, 0.0,
    true, -10.0, 10.0, true, 0.0, 1000.0, false, 0, 0, false,
  );

  const speed = 3.0; // m_speed (:115)

  // Payload crate1 always dynamic (:119-132)
  const crate1 = sim.add_body(-3.0, 8.0, 0.0, BODY_DYNAMIC);
  sim.attach_box(crate1, 0.75, 0.75, 0.0, 0.0, 0.0, 2.0, FRIC, 0.0);

  const secondPayload = sim.add_body(2.0, 8.0, 0.0, type);
  sim.attach_box(secondPayload, 0.75, 0.75, 0.0, 0.0, 0.0, 2.0, FRIC, 0.0);

  // Touching debris capsule (:152-166)
  const touching = sim.add_body(8.0, 0.2, 0.0, type);
  sim.attach_capsule(touching, 0.0, 0.0, 1.0, 0.0, 0.25, 2.0, FRIC, 0.0);

  // Static debris (:169-181)
  const staticDebris = sim.add_body(8.5, 0.2, 0.0, BODY_STATIC);
  sim.attach_capsule(staticDebris, 0.0, 0.0, 1.0, 0.0, 0.5, 0.0, FRIC, 0.0);

  // Floater circle, gravityScale 0 (:184-199)
  const floating = sim.add_body(-8.0, 12.0, 0.0, type);
  sim.attach_circle(floating, 0.0, 0.5, 0.25, 2.0, FRIC, 0.0);
  sim.set_gravity_scale(floating, 0.0);

  const typed = [platform, secondAttachment, secondPayload, touching, floating];

  const typeRow = createButtonGroup(
    [
      { label: "Static", value: "0" },
      { label: "Kinematic", value: "1" },
      { label: "Dynamic", value: "2" },
    ],
    "0",
    (v) => {
      type = Number(v);
      for (const id of typed) sim.set_body_type(id, type);
      if (type === BODY_KINEMATIC) {
        // :217-227
        sim.set_linear_velocity(platform, -speed, 0.0);
        sim.set_angular_velocity(platform, 0.0);
        sim.set_linear_velocity(secondAttachment, 0.0, 0.0);
        sim.set_angular_velocity(secondAttachment, 0.0);
      }
    },
  );
  controls.appendChild(typeRow);
  controls.appendChild(
    createCheckbox("Enable", isEnabled, (en) => {
      // :240-253 — only attachment, secondPayload, floating
      isEnabled = en;
      if (en) {
        sim.enable_body(attachment);
        sim.enable_body(secondPayload);
        sim.enable_body(floating);
      } else {
        sim.disable_body(attachment);
        sim.disable_body(secondPayload);
        sim.disable_body(floating);
      }
    }),
  );

  return {
    beforeStep() {
      // Drive kinematic platform (:261-272)
      if (type !== BODY_KINEMATIC) return;
      const p = posOf(sim, platform);
      const v = sim.get_linear_velocity(platform);
      if ((p.x < -14.0 && v[0]! < 0.0) || (p.x > 6.0 && v[0]! > 0.0)) {
        sim.set_linear_velocity(platform, -v[0]!, v[1]!);
      }
    },
  };
}

function buildWeeble(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:305-411 — Exact: mix callbacks + SetMassData COM offset.
  sim.enable_weeble_mix_callbacks(true);
  sim.add_segment(-20.0, 0.0, 20.0, 0.0);

  const weeble = sim.add_body(0.0, 3.0, 0.25 * Math.PI, BODY_DYNAMIC);
  sim.attach_capsule(weeble, 0.0, -1.0, 0.0, 1.0, 1.0, 1.0, FRIC, 0.0);

  // Parallel-axis COM offset (:343-352)
  const mass = sim.get_mass(weeble);
  let inertia = sim.get_rotational_inertia(weeble);
  const offset = 1.5;
  inertia += mass * offset * offset;
  sim.set_mass_data(weeble, mass, 0.0, -offset, inertia);

  let explosionMagnitude = 8.0; // :357
  const explosionPosition = { x: 0.0, y: 0.0 }; // :355
  const explosionRadius = 2.0; // :356

  controls.appendChild(
    createButton("Teleport", () => {
      // :364 — 0.95 * π
      sim.set_transform(weeble, 0.0, 5.0, 0.95 * Math.PI);
    }),
  );
  controls.appendChild(
    createButton("Explode", () => {
      // :369-374 falloff 0.1
      sim.explode(
        explosionPosition.x,
        explosionPosition.y,
        explosionRadius,
        0.1,
        explosionMagnitude,
      );
    }),
  );
  controls.appendChild(
    createSlider("Magnitude", -100, 100, explosionMagnitude, 0.1, (v) => {
      explosionMagnitude = v;
    }),
  );

  return {
    dispose() {
      sim.enable_weeble_mix_callbacks(false);
    },
    paintOverlay(ctx, camera, canvas) {
      // DrawCircle explosion marker (:388) — azure
      const c = worldToScreen(camera, canvas, explosionPosition.x, explosionPosition.y);
      const ppm = canvas.height / (2 * Math.max(1e-6, camera.zoom));
      ctx.beginPath();
      ctx.arc(c.x, c.y, explosionRadius * ppm, 0, Math.PI * 2);
      ctx.strokeStyle = "rgba(240,255,255,0.8)";
      ctx.lineWidth = 2;
      ctx.stroke();

      // Local/world point velocity lines (:391-399)
      const wp = sim.body_world_point(weeble, 0.0, 2.0);
      const v1 = sim.get_local_point_velocity(weeble, 0.0, 2.0);
      const v2 = sim.get_world_point_velocity(weeble, wp[0]!, wp[1]!);
      const a = worldToScreen(camera, canvas, wp[0]!, wp[1]!);
      const b1 = worldToScreen(camera, canvas, wp[0]! + v1[0]!, wp[1]! + v1[1]!);
      const b2 = worldToScreen(
        camera,
        canvas,
        wp[0]! + 0.05 + v2[0]!,
        wp[1]! + v2[1]!,
      );
      const a2 = worldToScreen(camera, canvas, wp[0]! + 0.05, wp[1]!);
      ctx.lineWidth = 2;
      ctx.strokeStyle = "rgb(255,0,0)";
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b1.x, b1.y);
      ctx.stroke();
      ctx.strokeStyle = "rgb(0,255,0)";
      ctx.beginPath();
      ctx.moveTo(a2.x, a2.y);
      ctx.lineTo(b2.x, b2.y);
      ctx.stroke();
    },
  };
}

function buildSleep(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:415-656 — Exact: enableSleep / sensors / sleepThreshold / invokeContact.
  const ground = sim.add_body(0.0, 0.0, 0.0, BODY_STATIC);
  const groundShape = sim.attach_segment_ex(
    ground, -40.0, 0.0, 40.0, 0.0, false, true, false, false, 0, 0,
  );

  const sensorIds: number[] = [];
  const sensorTouching = [false, false];

  // Sleeping capsules with sensors (:439-457)
  for (let i = 0; i < 2; ++i) {
    const body = sim.add_body_ex(-4.0, 3.0 + 2.0 * i, 0.0, BODY_DYNAMIC, 1.0, true);
    sim.set_awake(body, false);
    sim.attach_capsule(body, 0.0, 1.0, 1.0, 1.0, 0.75, 1.0, FRIC, 0.0);
    const sensor = sim.attach_capsule_ex(
      body, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, FRIC, 0.0, true, true, false, 0, 0,
    );
    sensorIds.push(sensor);
  }

  // Sleeping body but sleep is disabled (:460-471)
  {
    const body = sim.add_body_ex(0.0, 3.0, 0.0, BODY_DYNAMIC, 1.0, false);
    sim.set_awake(body, false);
    sim.attach_circle(body, 1.0, 1.0, 1.0, 1.0, FRIC, 0.0);
  }

  // Awake, sleep disabled (:474-485)
  {
    const body = sim.add_body_ex(5.0, 3.0, 0.0, BODY_DYNAMIC, 1.0, false);
    sim.attach_box(body, 1.0, 1.0, 0.0, 1.0, 0.25 * Math.PI, 1.0, FRIC, 0.0);
  }

  // Sleeping square (:488-499)
  {
    const body = sim.add_body_ex(5.0, 1.0, 0.0, BODY_DYNAMIC, 1.0, true);
    sim.set_awake(body, false);
    sim.attach_box(body, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
  }

  // Pendulum capsule (:502-521)
  const pendulum = sim.add_body_sleep_threshold(0.0, 100.0, 0.0, BODY_DYNAMIC, 0.05);
  sim.set_angular_damping(pendulum, 0.5);
  sim.attach_capsule(pendulum, 0.0, 0.0, 90.0, 0.0, 0.25, 1.0, FRIC, 0.0);
  sim.add_revolute_joint(
    ground, pendulum, 0.0, 100.0,
    false, 0, 0, false, 0, 0, false, 0, 0, false,
  );

  // Sleeping box for contact-destroyed wake (:524-535)
  {
    const body = sim.add_body_ex(-10.0, 1.0, 0.0, BODY_DYNAMIC, 1.0, true);
    sim.set_awake(body, false);
    sim.attach_box(body, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
  }

  let invoker: number | null = null;

  controls.appendChild(
    createSlider("sleep velocity", 0, 1, sim.get_sleep_threshold(pendulum), 0.01, (v) => {
      sim.set_sleep_threshold(pendulum, v);
      sim.set_awake(pendulum, true);
    }),
  );
  controls.appendChild(
    createSlider("angular damping", 0, 2, sim.get_angular_damping(pendulum), 0.01, (v) => {
      sim.set_angular_damping(pendulum, v);
    }),
  );

  const toggleBtn = createButton("Create", () => {
    if (invoker === null) {
      // :544-551 — offset box 2×0.1 at 0.25π with invokeContactCreation
      invoker = sim.add_body(-10.5, 3.0, 0.0, BODY_STATIC);
      sim.attach_offset_rounded_box(
        invoker, 2.0, 0.1, 0.0, 0.0, 0.25 * Math.PI, 0.0, 0.0, FRIC, 0.0, true,
      );
      toggleBtn.textContent = "Destroy";
    } else {
      sim.destroy_body(invoker);
      invoker = null;
      toggleBtn.textContent = "Create";
    }
  });
  controls.appendChild(toggleBtn);

  return {
    afterStep() {
      const begins = sim.sensor_begin_events();
      for (let i = 0; i + 1 < begins.length; i += 2) {
        const sensor = begins[i]!;
        const visitor = begins[i + 1]!;
        if (visitor !== groundShape) continue;
        if (sensor === sensorIds[0]) sensorTouching[0] = true;
        else if (sensor === sensorIds[1]) sensorTouching[1] = true;
      }
      const ends = sim.sensor_end_events();
      for (let i = 0; i + 1 < ends.length; i += 2) {
        const sensor = ends[i]!;
        const visitor = ends[i + 1]!;
        if (visitor !== groundShape) continue;
        if (sensor === sensorIds[0]) sensorTouching[0] = false;
        else if (sensor === sensorIds[1]) sensorTouching[1] = false;
      }
    },
    readoutExtra: () => [
      { label: "sensor touch 0", value: String(sensorTouching[0]) },
      { label: "sensor touch 1", value: String(sensorTouching[1]) },
    ],
  };
}

function buildBad(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:660-733
  sim.add_segment(-20.0, 0.0, 20.0, 0.0);

  // density 0 intentionally (:696)
  const bad = sim.add_body(0.0, 3.0, 0.25 * Math.PI, BODY_DYNAMIC);
  sim.attach_capsule(bad, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, FRIC, 0.0);
  sim.set_angular_velocity(bad, 0.5); // :687

  const good = sim.add_body(2.0, 3.0, 0.25 * Math.PI, BODY_DYNAMIC);
  sim.attach_capsule(good, 0.0, -1.0, 0.0, 1.0, 1.0, 1.0, FRIC, 0.0);

  return {
    afterStep() {
      // :724
      sim.apply_force_to_center(bad, 0.0, 10.0, true);
    },
    readoutExtra: () => [
      {
        label: "note",
        value: "bad body = dynamic with zero mass (invalid)",
      },
    ],
  };
}

function buildPivot(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:738-804
  sim.add_segment(-20.0, 0.0, 20.0, 0.0);

  const lever = 3.0; // :772
  const vx = 5.0; // :762
  const body = sim.add_body(0.0, 3.0, 0.0, BODY_DYNAMIC);
  sim.set_linear_velocity(body, vx, 0.0);
  // omega = cross(v, r) / dot(r, r); r = (0, -lever) → cross = vx*(-lever) - 0 = -vx*lever
  // C: b2Cross(v, r) / b2Dot(r, r) with r={0,-lever} → (5*−3 − 0*0)/(0+9) = -15/9
  const rx = 0.0;
  const ry = -lever;
  const omega = (vx * ry - 0.0 * rx) / (rx * rx + ry * ry);
  sim.set_angular_velocity(body, omega);
  sim.attach_box(body, 0.1, lever, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  return {
    readoutExtra: () => {
      // :789-794 pivot velocity at tip
      const v = sim.get_linear_velocity(body);
      const w = sim.get_angular_velocity(body);
      const angle = posOf(sim, body).angle;
      const cos = Math.cos(angle);
      const sin = Math.sin(angle);
      // world vector of local (0, -lever)
      const wrx = -sin * ry; // rotate (0, ry)
      const wry = cos * ry;
      // vp = v + crossSV(omega, r) = v + (-omega*ry_world_y?); 2D: (−ω ry, ω rx) wait
      // b2CrossSV(s, v) = (−s*vy, s*vx)
      const vpx = v[0]! + -w * wry;
      const vpy = v[1]! + w * wrx;
      return [{ label: "pivot velocity", value: `(${vpx.toFixed(3)}, ${vpy.toFixed(3)})` }];
    },
  };
}

function buildKinematic(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:809-875 — Exact: SetTargetTransform.
  const amplitude = 2.0; // :821
  const body = sim.add_body(2.0 * amplitude, 0.0, 0.0, BODY_KINEMATIC);
  sim.attach_box(body, 0.1, 1.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  let time = 0.0;
  let target = { x: 2.0 * amplitude, y: 0.0, angle: 0.0 };

  return {
    beforeStep(dt) {
      if (dt <= 0) return;
      // :848-859
      const pointX = 2.0 * amplitude * Math.cos(time);
      const pointY = amplitude * Math.sin(2.0 * time);
      const rotation = 2.0 * time;
      target = { x: pointX, y: pointY, angle: rotation };
      sim.set_target_transform(body, pointX, pointY, rotation, dt, true);
      time += dt;
    },
    paintOverlay(ctx, camera, canvas) {
      // DrawLine + DrawPoint plum (:854-856)
      const axisX = -Math.sin(target.angle); // rotate (0,1)
      const axisY = Math.cos(target.angle);
      const a = worldToScreen(
        camera,
        canvas,
        target.x - 0.5 * axisX,
        target.y - 0.5 * axisY,
      );
      const b = worldToScreen(
        camera,
        canvas,
        target.x + 0.5 * axisX,
        target.y + 0.5 * axisY,
      );
      const p = worldToScreen(camera, canvas, target.x, target.y);
      ctx.strokeStyle = "rgb(221,160,221)";
      ctx.fillStyle = "rgb(221,160,221)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
      ctx.fill();
    },
  };
}

function buildMixedLocks(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:880-985 — Exact: motionLocks via set_motion_locks.
  sim.add_segment(-40.0, 0.0, 40.0, 0.0);

  // static at (2,1) (:905-911)
  {
    const b = sim.add_body(2.0, 1.0, 0.0, BODY_STATIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
  }
  // free (1,1) (:914-921)
  {
    const b = sim.add_body(1.0, 1.0, 0.0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
  }
  // free (1,3) (:924-931)
  {
    const b = sim.add_body(1.0, 3.0, 0.0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
  }
  // angular z (-1,1) (:934-942)
  {
    const b = sim.add_body(-1.0, 1.0, 0.0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
    sim.set_motion_locks(b, false, false, true);
  }
  // linear x (-2,2) (:945-953)
  {
    const b = sim.add_body(-2.0, 2.0, 0.0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
    sim.set_motion_locks(b, true, false, false);
  }
  // lin y ang z (-1,2.5) (:956-964)
  {
    const b = sim.add_body(-1.0, 2.5, 0.0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
    sim.set_motion_locks(b, false, true, true);
  }
  // full (0,1) (:968-977)
  {
    const b = sim.add_body(0.0, 1.0, 0.0, BODY_DYNAMIC);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
    sim.set_motion_locks(b, true, true, true);
  }

  return {};
}

function buildSetVelocity(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:989-1039
  const ground = sim.add_body(0.0, -0.25, 0.0, BODY_STATIC);
  sim.attach_box(ground, 20.0, 0.25, 0.0, 0.0, 0.0, 0.0, FRIC, 0.0);

  const body = sim.add_body(0.0, 0.5, 0.0, BODY_DYNAMIC);
  sim.attach_box(body, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  return {
    afterStep() {
      // :1027 — set every step after Sample::Step; we apply after step like C's order
      // C calls SetLinearVelocity after Sample::Step, so velocity is for next frame.
      sim.set_linear_velocity(body, 0.0, -20.0);
    },
    readoutExtra: () => {
      const p = posOf(sim, body);
      return [{ label: "(x, y)", value: `(${p.x.toFixed(2)}, ${p.y.toFixed(2)})` }];
    },
  };
}

function buildWakeTouching(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_bodies.cpp:1043-1101
  const ground = sim.add_segment(-20.0, 0.0, 20.0, 0.0);

  const count = 10; // m_count (:1098)
  let x = -1.0 * (count - 1); // :1072
  for (let i = 0; i < count; ++i) {
    const body = sim.add_body(x, 4.0, 0.0, BODY_DYNAMIC);
    sim.attach_box(body, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);
    x += 2.0;
  }

  controls.appendChild(
    createButton("Wake Touching", () => {
      sim.wake_touching(ground);
    }),
  );

  return {};
}

function buildScene(
  scene: Scene,
  sim: SimWorld,
  sceneControls: HTMLElement,
): SceneRuntime {
  sceneControls.innerHTML = "";
  switch (scene) {
    case "body-type":
      return buildBodyType(sim, sceneControls);
    case "weeble":
      return buildWeeble(sim, sceneControls);
    case "sleep":
      return buildSleep(sim, sceneControls);
    case "bad":
      return buildBad(sim, sceneControls);
    case "pivot":
      return buildPivot(sim, sceneControls);
    case "kinematic":
      return buildKinematic(sim, sceneControls);
    case "mixed-locks":
      return buildMixedLocks(sim, sceneControls);
    case "set-velocity":
      return buildSetVelocity(sim, sceneControls);
    case "wake-touching":
      return buildWakeTouching(sim, sceneControls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Bodies",
    "C <code>sample_bodies.cpp</code> RegisterSample ports — body types, sleep, " +
      "kinematic targets, velocity, motion locks.",
    "Drag to grab · P pause · O step · R restart",
    { category: "Bodies", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "body-type";

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

  // Mouse grab
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
        history.replaceState(null, "", `#/bodies/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "bodies",
    category: "Bodies",
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
