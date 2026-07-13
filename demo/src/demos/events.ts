// Events — RegisterSample ports from sample_events.cpp.
// Invented bounce/sensor composite retired; scenes map 1:1 to C names.

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
  "sensor-funnel",
  "sensor-bookend",
  "foot-sensor",
  "contact",
  "platformer",
  "body-move",
  "sensor-types",
  "joint",
  "persistent-contact",
  "sensor-hits",
  "projectile-event",
  "circle-impulse",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("events", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "sensor-funnel": "Sensor Funnel",
  "sensor-bookend": "Sensor Bookend",
  "foot-sensor": "Foot Sensor",
  contact: "Contact",
  platformer: "Platformer",
  "body-move": "Body Move",
  "sensor-types": "Sensor Types",
  joint: "Joint",
  "persistent-contact": "Persistent Contact",
  "sensor-hits": "Sensor Hits",
  "projectile-event": "Projectile Event",
  "circle-impulse": "Circle Impulse",
};

const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "sensor-funnel": { cx: 0, cy: 0, zoom: 25 * 1.333 }, // :33-34
  "sensor-bookend": { cx: 0, cy: 6, zoom: 7.5 }, // :343-344
  "foot-sensor": { cx: 0, cy: 6, zoom: 7.5 }, // :681-682
  contact: { cx: 0, cy: 0, zoom: 25 * 1.75 }, // :817-818
  platformer: { cx: 0.5, cy: 7.5, zoom: 25 * 0.4 }, // :1231-1232
  "body-move": { cx: 2, cy: 8, zoom: 25 * 0.55 }, // :1465-1466
  "sensor-types": { cx: 0, cy: 3, zoom: 4.5 }, // :1653-1654
  joint: { cx: 0, cy: 8, zoom: 25 * 0.7 }, // :1845-1846
  "persistent-contact": { cx: 0, cy: 6, zoom: 7.5 }, // :2069-2070
  "sensor-hits": { cx: 0, cy: 5, zoom: 7.5 }, // :2173-2174
  "projectile-event": { cx: -7, cy: 9, zoom: 14 }, // :2400-2401
  "circle-impulse": { cx: 0, cy: 2.7, zoom: 3.4 }, // :2572-2573
};

const FRIC = 0.6;
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const PI = Math.PI;

const GROUND = 0x00000001;
const PLAYER = 0x00000002;
const FOOT = 0x00000004;
const SENSOR = 0x00000002;
const DEFAULT_BITS = 0x00000004;

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  paintOverlay?: (ctx: CanvasRenderingContext2D, camera: SampleCamera, canvas: HTMLCanvasElement) => void;
  readoutExtra?: () => { label: string; value: string }[];
  dispose?: () => void;
  onPointerDown?: (wx: number, wy: number, mods: { ctrl: boolean }) => void;
  onPointerMove?: (wx: number, wy: number) => void;
  onPointerUp?: () => void;
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

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

function spawnDonut(sim: SimWorld, px: number, py: number, scale: number, userData: number) {
  // donut.cpp Create — 7 welded capsules
  const sides = 7;
  const radius = 1.0 * scale;
  const delta = (2 * PI) / sides;
  const length = (2 * PI * radius) / sides;
  const bodies: number[] = [];
  let angle = 0;
  for (let i = 0; i < sides; i++) {
    const b = sim.add_body(radius * Math.cos(angle) + px, radius * Math.sin(angle) + py, angle, BODY_DYNAMIC);
    sim.attach_capsule_filtered(b, 0, -0.5 * length, 0, 0.5 * length, 0.25 * scale, 1, 0.3, 0, 0);
    sim.enable_sensor_visitor(b);
    sim.body_set_user_data(b, userData);
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
  return bodies;
}

// ---------------------------------------------------------------------------
// Scene builders — cite sample_events.cpp
// ---------------------------------------------------------------------------

function buildSensorFunnel(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :18-331 Exact: donut + CreateHuman paths; bottom sensor destroys via begin events
  const FUNNEL = [
    -16.8672504, 31.088623, 16.8672485, 31.088623, 16.8672485, 17.1978741, 8.26824951, 11.906374,
    16.8672485, 11.906374, 16.8672485, -0.661376953, 8.26824951, -5.953125, 16.8672485, -5.953125,
    16.8672485, -13.229126, 3.63799858, -23.151123, 3.63799858, -31.088623, -3.63800049, -31.088623,
    -3.63800049, -23.151123, -16.8672504, -13.229126, -16.8672504, -5.953125, -8.26825142, -5.953125,
    -16.8672504, -0.661376953, -16.8672504, 11.906374, -8.26825142, 11.906374, -16.8672504, 17.1978741,
  ];
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_chain_ex(ground, FUNNEL, true, 0, 0, false, 0.2);
  const sensorShape = sim.attach_box_ex(ground, 4, 1, 0, -30.5, 0, 0, FRIC, 0, true, true, false, false, false, 0, 0);

  let sign = 1;
  let y = 14;
  for (let i = 0; i < 3; i++) {
    const body = sim.add_body(0, y, 0, BODY_DYNAMIC);
    sim.attach_box(body, 6, 0.5, 0, 0, 0, 1, 0.1, 1);
    sim.add_revolute_joint(ground, body, 0, y, false, 0, 0, true, 2 * sign, 200, false, 0, 0, false);
    y -= 14;
    sign = -sign;
  }

  const COUNT = 32;
  type Slot = { kind: "donut"; bodies: number[] } | { kind: "human"; id: number };
  const spawned: (Slot | null)[] = Array.from({ length: COUNT }, () => null);
  let wait = 0.5;
  let side = -15;
  let type: "donut" | "human" = "human"; // C default m_type = e_human

  const destroySlot = (index: number) => {
    const slot = spawned[index];
    if (!slot) return;
    if (slot.kind === "donut") {
      for (const b of slot.bodies) if (sim.is_body_alive(b)) sim.destroy_body(b);
    } else if (sim.human_is_spawned(slot.id)) {
      sim.destroy_human(slot.id);
    }
    spawned[index] = null;
  };

  const clear = () => {
    for (let i = 0; i < COUNT; i++) destroySlot(i);
  };

  const createElement = () => {
    const index = spawned.findIndex((s) => s == null);
    if (index < 0) return;
    if (type === "donut") {
      spawned[index] = { kind: "donut", bodies: spawnDonut(sim, side, 29.5, 1, index + 1) };
    } else {
      // :187-194 CreateHuman scale=2, friction=0.05, hertz=6, damping=0.5, colorize
      const h = sim.create_human(side, 29.5, 2.0, 0.05, 6.0, 0.5, index + 1, true, index + 1);
      sim.human_enable_sensor_events(h, true);
      spawned[index] = { kind: "human", id: h };
    }
    side = -side;
  };

  createElement();

  controls.appendChild(
    createInfoBox(
      "Exact: donut / human (<code>CreateHuman</code>) toggle. Bottom sensor destroys " +
        "visitors via begin events. C <code>sample_events.cpp</code> Sensor Funnel.",
    ),
  );
  controls.appendChild(
    createDropdown(
      "Type",
      [
        { value: "donut", text: "donut" },
        { value: "human", text: "human" },
      ],
      type,
      (v) => {
        clear();
        type = v as "donut" | "human";
      },
    ),
  );

  return {
    afterStep: (dt) => {
      const hits = sim.sensor_begin_user_data_for(sensorShape);
      const deferred = new Set<number>();
      for (let i = 0; i < hits.length; i++) {
        const ud = hits[i]!;
        if (ud >= 1 && ud <= COUNT) deferred.add(ud - 1);
      }
      for (const index of deferred) destroySlot(index);
      if (dt > 0) {
        wait -= dt;
        if (wait < 0) {
          createElement();
          wait += 0.5;
        }
      }
    },
    dispose: () => clear(),
  };
}

function buildSensorBookend(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :335-660
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment(ground, -10, 0, 10, 0);
  sim.attach_segment(ground, -10, 0, -10, 10);
  sim.attach_segment(ground, 10, 0, 10, 10);

  let sensor1 = -1;
  let sensorShape1 = -1;
  let sensor2 = -1;
  let sensorShape2 = -1;
  let visitor = -1;
  let visitorShape = -1;
  let visiting1 = false;
  let visiting2 = false;
  let overlapCount = 0;

  const createSensor1 = () => {
    sensor1 = sim.add_body(-2, 1, 0, BODY_STATIC);
    sensorShape1 = sim.attach_box_ex(sensor1, 1, 1, 0, 0, 0, 0, FRIC, 0, true, true, false, false, false, 0, 0);
  };
  const createSensor2 = () => {
    sensor2 = sim.add_body(2, 1, 0, BODY_DYNAMIC);
    sensorShape2 = sim.attach_rounded_box_ex(sensor2, 0.5, 0.5, 0.5, 1, true, true);
    sim.attach_box_ex(sensor2, 0.5, 0.5, 0, 0, 0, 1, FRIC, 0, false, false, false, false, false, 0, 0);
  };
  const createVisitor = () => {
    visitor = sim.add_body(-4, 1, 0, BODY_DYNAMIC);
    visitorShape = sim.attach_circle_ex(visitor, 0, 0, 0.5, 1, FRIC, 0, 0, false, true, false, false, 0, 0);
  };
  createSensor1();
  createSensor2();
  createVisitor();

  const btn = (label: string, fn: () => void) => controls.appendChild(createButton(label, fn));
  btn("destroy visitor", () => {
    if (visitor >= 0 && sim.is_body_alive(visitor)) {
      sim.destroy_body(visitor);
      visitor = -1;
    }
  });
  btn("create visitor", () => {
    if (visitor < 0 || !sim.is_body_alive(visitor)) createVisitor();
  });
  controls.appendChild(
    createCheckbox("visitor events", true, (en) => {
      if (visitorShape >= 0) sim.shape_enable_sensor_events(visitorShape, en);
    }),
  );
  controls.appendChild(
    createCheckbox("enable visitor body", true, (en) => {
      if (visitor < 0 || !sim.is_body_alive(visitor)) return;
      if (en) sim.enable_body(visitor);
      else sim.disable_body(visitor);
    }),
  );
  controls.appendChild(createSeparator());
  btn("destroy sensor1", () => {
    if (sensor1 >= 0 && sim.is_body_alive(sensor1)) {
      sim.destroy_body(sensor1);
      sensor1 = -1;
    }
  });
  btn("create sensor1", () => {
    if (sensor1 < 0 || !sim.is_body_alive(sensor1)) createSensor1();
  });
  controls.appendChild(
    createCheckbox("sensor 1 events", true, (en) => {
      if (sensorShape1 >= 0) sim.shape_enable_sensor_events(sensorShape1, en);
    }),
  );

  return {
    afterStep: () => {
      const begins = sim.sensor_begin_events();
      for (let i = 0; i + 1 < begins.length; i += 2) {
        const s = begins[i]!;
        const v = begins[i + 1]!;
        if (s === sensorShape1) {
          if (v === visitorShape) visiting1 = true;
          else if (v === sensorShape2) overlapCount += 1;
        } else if (s === sensorShape2) {
          if (v === visitorShape) visiting2 = true;
          else if (v === sensorShape1) overlapCount += 1;
        }
      }
      const ends = sim.sensor_end_events();
      for (let i = 0; i + 1 < ends.length; i += 2) {
        const s = ends[i]!;
        const v = ends[i + 1]!;
        if (s === sensorShape1) {
          if (v === visitorShape) visiting1 = false;
          else if (v === sensorShape2) overlapCount -= 1;
        } else if (s === sensorShape2) {
          if (v === visitorShape) visiting2 = false;
          else if (v === sensorShape1) overlapCount -= 1;
        }
      }
      if (visitorShape >= 0 && !sim.shape_is_valid(visitorShape)) visitorShape = -1;
      if (sensorShape1 >= 0 && !sim.shape_is_valid(sensorShape1)) sensorShape1 = -1;
      if (sensorShape2 >= 0 && !sim.shape_is_valid(sensorShape2)) sensorShape2 = -1;
    },
    readoutExtra: () => [
      { label: "visiting 1", value: String(visiting1) },
      { label: "visiting 2", value: String(visiting2) },
      { label: "sensors overlap", value: String(overlapCount) },
    ],
  };
}

function buildFootSensor(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :664-795
  const points: number[] = [];
  let x = 10;
  for (let i = 0; i < 20; i++) {
    points.push(x, 0);
    x -= 1;
  }
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_chain_ex(ground, points, false, GROUND, FOOT | PLAYER, true, FRIC);

  const player = sim.add_body(0, 1, 0, BODY_DYNAMIC);
  sim.set_motion_locks(player, false, false, true);
  sim.attach_capsule_ex(player, 0, -0.5, 0, 0.5, 0.5, 1, 0.3, 0, false, false, false, PLAYER, GROUND);
  const foot = sim.attach_box_ex(player, 0.5, 0.25, 0, -1, 0, 1, FRIC, 0, true, true, false, false, false, FOOT, GROUND);

  let overlap = 0;
  let keys = { a: false, d: false };
  const onKey = (e: KeyboardEvent, down: boolean) => {
    if (e.code === "KeyA") keys.a = down;
    if (e.code === "KeyD") keys.d = down;
  };
  const kd = (e: KeyboardEvent) => onKey(e, true);
  const ku = (e: KeyboardEvent) => onKey(e, false);
  window.addEventListener("keydown", kd);
  window.addEventListener("keyup", ku);

  controls.appendChild(createInfoBox("WASD: A/D move. Foot sensor overlap count + visitor AABB centers."));

  return {
    beforeStep: () => {
      if (keys.a) sim.apply_force_to_center(player, -50, 0, true);
      if (keys.d) sim.apply_force_to_center(player, 50, 0, true);
    },
    afterStep: () => {
      const begins = sim.sensor_begin_events();
      for (let i = 0; i + 1 < begins.length; i += 2) {
        if (begins[i] === foot) overlap += 1;
      }
      const ends = sim.sensor_end_events();
      for (let i = 0; i + 1 < ends.length; i += 2) {
        if (ends[i] === foot) overlap -= 1;
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      const centers = sim.sensor_visitor_centers(foot);
      ctx.fillStyle = "#fff";
      for (let i = 0; i + 1 < centers.length; i += 2) {
        const p = worldToScreen(camera, canvas, centers[i]!, centers[i + 1]!);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 5, 0, 2 * PI);
        ctx.fill();
      }
    },
    readoutExtra: () => [{ label: "count", value: String(overlap) }],
    dispose: () => {
      window.removeEventListener("keydown", kd);
      window.removeEventListener("keyup", ku);
    },
  };
}

function buildContact(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :804-1217
  const wall = [40, -40, -40, -40, -40, 40, 40, 40];
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_chain_ex(ground, wall, true, 0, 0, false, FRIC);

  const player = sim.add_body(0, 0, 0, BODY_DYNAMIC);
  sim.set_gravity_scale(player, 0);
  sim.set_linear_damping(player, 0.5);
  sim.set_angular_damping(player, 0.5);
  sim.set_bullet(player, true);
  const core = sim.attach_circle_ex(player, 0, 0, 1, 1, FRIC, 0, 0, false, false, true, false, 0, 0);

  const COUNT = 20;
  const debris: number[] = Array(COUNT).fill(-1);
  const rng = makeXorShift(12345);
  let wait = 0.5;
  let force = 200;
  let keys = { a: false, d: false, w: false, s: false };

  const spawnDebris = () => {
    const index = debris.findIndex((d) => d < 0 || !sim.is_body_alive(d));
    if (index < 0) return;
    const b = sim.add_body(rng.range(-38, 38), rng.range(-38, 38), rng.range(-PI, PI), BODY_DYNAMIC);
    sim.set_gravity_scale(b, 0);
    sim.set_linear_velocity(b, rng.range(-5, 5), rng.range(-5, 5));
    sim.set_angular_velocity(b, rng.range(-1, 1));
    sim.body_set_user_data(b, index + 1);
    const kind = (index + 1) % 3;
    if (kind === 0) sim.attach_circle_ex(b, 0, 0, 0.5, 1, FRIC, 0.8, 0, false, false, false, false, 0, 0);
    else if (kind === 1) sim.attach_capsule_ex(b, 0, -0.25, 0, 0.25, 0.25, 1, FRIC, 0.8, false, false, false, 0, 0);
    else sim.attach_box_ex(b, 0.4, 0.6, 0, 0, 0, 1, FRIC, 0.8, false, false, false, false, false, 0, 0);
    debris[index] = b;
  };

  const onKey = (e: KeyboardEvent, down: boolean) => {
    if (e.code === "KeyA") keys.a = down;
    if (e.code === "KeyD") keys.d = down;
    if (e.code === "KeyW") keys.w = down;
    if (e.code === "KeyS") keys.s = down;
  };
  const kd = (e: KeyboardEvent) => onKey(e, true);
  const ku = (e: KeyboardEvent) => onKey(e, false);
  window.addEventListener("keydown", kd);
  window.addEventListener("keyup", ku);

  controls.appendChild(createInfoBox("WASD move. Debris sticks to the player on contact begin."));
  controls.appendChild(createSlider("force", 100, 500, force, 1, (v) => { force = v; }));

  return {
    beforeStep: () => {
      const p = sim.positions();
      const px = p[player * 3]!;
      const py = p[player * 3 + 1]!;
      if (keys.a) sim.apply_force(player, -force, 0, px, py, true);
      if (keys.d) sim.apply_force(player, force, 0, px, py, true);
      if (keys.w) sim.apply_force(player, 0, force, px, py, true);
      if (keys.s) sim.apply_force(player, 0, -force, px, py, true);
    },
    afterStep: (dt) => {
      const begins = sim.contact_begin_bodies();
      const toAttach: number[] = [];
      const toDestroy: number[] = [];
      for (let i = 0; i + 3 < begins.length; i += 4) {
        const ba = begins[i]!;
        const bb = begins[i + 1]!;
        const sa = begins[i + 2]!;
        const sb = begins[i + 3]!;
        if (ba === player) {
          const other = bb;
          // user data: debris index+1 stored on body — approximate via slot lookup
          const di = debris.indexOf(other);
          if (di >= 0) toAttach.push(di);
          else if (sa !== core && sa >= 0) toDestroy.push(sa);
        } else if (bb === player) {
          const other = ba;
          const di = debris.indexOf(other);
          if (di >= 0) toAttach.push(di);
          else if (sb !== core && sb >= 0) toDestroy.push(sb);
        }
      }
      for (const di of toAttach) {
        const d = debris[di]!;
        if (d >= 0 && sim.is_body_alive(d)) {
          sim.absorb_body_shapes(player, d);
          debris[di] = -1;
        }
      }
      for (const s of toDestroy) {
        if (sim.shape_is_valid(s)) sim.destroy_shape(s, false);
      }
      if (toDestroy.length > 0) sim.apply_mass_from_shapes(player);
      if (dt > 0) {
        wait -= dt;
        if (wait < 0) {
          spawnDebris();
          wait += 0.5;
        }
      }
    },
    dispose: () => {
      window.removeEventListener("keydown", kd);
      window.removeEventListener("keyup", ku);
    },
  };
}

function buildPlatformer(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1223-1447
  sim.add_segment(-20, 0, 20, 0);
  const staticPlat = sim.add_body(-6, 6, 0, BODY_STATIC);
  sim.attach_box_ex(staticPlat, 2, 0.5, 0, 0, 0, 0, FRIC, 0, false, false, false, false, true, 0, 0);
  const moving = sim.add_body(0, 6, 0, BODY_KINEMATIC);
  sim.set_linear_velocity(moving, 2, 0);
  sim.attach_box_ex(moving, 3, 0.5, 0, 0, 0, 0, FRIC, 0, false, false, false, false, true, 0, 0);

  const player = sim.add_body(0, 1, 0, BODY_DYNAMIC);
  sim.set_motion_locks(player, false, false, true);
  sim.set_linear_damping(player, 0.5);
  const playerShape = sim.attach_capsule_ex(player, 0, 0, 0, 1, 0.5, 1, 0.1, 0, false, false, false, 0, 0);
  sim.enable_platformer_presolve(playerShape);

  let force = 25;
  let impulse = 25;
  let jumpDelay = 0.25;
  let keys = { a: false, d: false, space: false };
  const onKey = (e: KeyboardEvent, down: boolean) => {
    if (e.code === "KeyA") keys.a = down;
    if (e.code === "KeyD") keys.d = down;
    if (e.code === "Space") keys.space = down;
  };
  const kd = (e: KeyboardEvent) => onKey(e, true);
  const ku = (e: KeyboardEvent) => onKey(e, false);
  window.addEventListener("keydown", kd);
  window.addEventListener("keyup", ku);

  controls.appendChild(createInfoBox("A/D move · Space jump. One-way platforms via pre-solve."));
  controls.appendChild(createSlider("force", 0, 50, force, 1, (v) => { force = v; }));
  controls.appendChild(createSlider("impulse", 0, 50, impulse, 1, (v) => { impulse = v; }));

  return {
    beforeStep: (dt) => {
      const p = sim.positions();
      const mx = p[moving * 3]!;
      if (mx < -10) sim.set_linear_velocity(moving, 2, 0);
      else if (mx > 10) sim.set_linear_velocity(moving, -2, 0);

      if (keys.a) sim.apply_force_to_center(player, -force, 0, true);
      if (keys.d) sim.apply_force_to_center(player, force, 0, true);
      if (jumpDelay > 0) jumpDelay -= dt;
      if (keys.space && jumpDelay <= 0) {
        sim.apply_linear_impulse_to_center(player, 0, impulse, true);
        jumpDelay = 0.25;
      }
    },
    dispose: () => {
      sim.clear_presolve();
      window.removeEventListener("keydown", kd);
      window.removeEventListener("keyup", ku);
    },
  };
}

function buildBodyMove(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1452-1632
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box_ex(ground, 12, 0.1, -10, -0.1, -0.15 * PI, 0, 0.1, 0, false, false, false, false, false, 0, 0);
  sim.attach_box_ex(ground, 12, 0.1, 10, -0.1, 0.15 * PI, 0, 0.1, 0, false, false, false, false, false, 0, 0);
  sim.attach_box_ex(ground, 0.1, 10, 19.9, 10, 0, 0, 0.1, 0.8, false, false, false, false, false, 0, 0);
  sim.attach_box_ex(ground, 0.1, 10, -19.9, 10, 0, 0, 0.1, 0.8, false, false, false, false, false, 0, 0);
  sim.attach_box_ex(ground, 20, 0.1, 0, 20.1, 0, 0, 0.1, 0.8, false, false, false, false, false, 0, 0);

  const COUNT = 50;
  const bodies: number[] = [];
  const sleeping: boolean[] = [];
  let sleepCount = 0;
  let stepBit = 0;
  let magnitude = 10;
  const explosion = { x: 0, y: -5, r: 10 };
  const rng = makeXorShift(12345);

  const createBodies = () => {
    let x = -5;
    const y = 10;
    for (let i = 0; i < 10 && bodies.length < COUNT; i++) {
      const b = sim.add_body(x, y, 0, BODY_DYNAMIC);
      if (bodies.length % 12 === 0) sim.set_bullet(b, true);
      sim.body_set_user_data(b, bodies.length + 1);
      const rem = bodies.length % 4;
      if (rem === 0) sim.attach_capsule(b, -0.25, 0, 0.25, 0, 0.25, 1, FRIC, 0);
      else if (rem === 1) sim.attach_circle(b, 0, 0, 0.35, 1, FRIC, 0);
      else if (rem === 2) sim.attach_box(b, 0.35, 0.35, 0, 0, 0, 1, FRIC, 0);
      else {
        const pts: number[] = [];
        for (let k = 0; k < 6; k++) {
          const a = (k / 6) * 2 * PI + rng.range(-0.2, 0.2);
          const r = rng.range(0.2, 0.75);
          pts.push(r * Math.cos(a), r * Math.sin(a));
        }
        sim.attach_polygon(b, pts, 0.1, 1, FRIC, 0);
      }
      bodies.push(b);
      sleeping.push(false);
      x += 1;
    }
  };

  controls.appendChild(
    createButton("Explode", () => sim.explode(explosion.x, explosion.y, explosion.r, 0.1, magnitude)),
  );
  controls.appendChild(createSlider("Magnitude", -20, 20, magnitude, 0.1, (v) => { magnitude = v; }));

  const transforms: { x: number; y: number; a: number }[] = [];

  return {
    beforeStep: () => {
      stepBit = (stepBit + 1) & 15;
      if (stepBit === 15 && bodies.length < COUNT) createBodies();
    },
    afterStep: () => {
      transforms.length = 0;
      const ev = sim.body_move_events();
      for (let i = 0; i + 4 < ev.length; i += 5) {
        const bi = ev[i]!;
        if (bi < 0) continue;
        const fell = ev[i + 1]! > 0.5;
        transforms.push({ x: ev[i + 2]!, y: ev[i + 3]!, a: ev[i + 4]! });
        const slot = bodies.indexOf(bi);
        if (slot < 0) continue;
        if (fell) {
          if (!sleeping[slot]) {
            sleeping[slot] = true;
            sleepCount += 1;
          }
        } else if (sleeping[slot]) {
          sleeping[slot] = false;
          sleepCount -= 1;
        }
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      const c = worldToScreen(camera, canvas, explosion.x, explosion.y);
      const r = worldToScreen(camera, canvas, explosion.x + explosion.r, explosion.y);
      ctx.strokeStyle = "#f0ffff";
      ctx.beginPath();
      ctx.arc(c.x, c.y, Math.abs(r.x - c.x), 0, 2 * PI);
      ctx.stroke();
      ctx.strokeStyle = "#0f0";
      for (const t of transforms) {
        const p = worldToScreen(camera, canvas, t.x, t.y);
        const len = 20;
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + Math.cos(t.a) * len, p.y - Math.sin(t.a) * len);
        ctx.stroke();
      }
    },
    readoutExtra: () => [{ label: "sleep count", value: String(sleepCount) }],
  };
}

function buildSensorTypes(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :1636-1827
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.body_set_name(ground, "ground");
  sim.attach_segment_ex(ground, -6, 0, 6, 0, false, true, false, false, GROUND, DEFAULT_BITS);
  sim.attach_segment_ex(ground, -6, 0, -6, 4, false, true, false, false, GROUND, DEFAULT_BITS);
  sim.attach_segment_ex(ground, 6, 0, 6, 4, false, true, false, false, GROUND, DEFAULT_BITS);

  const staticS = sim.add_body(-3, 0.8, 0, BODY_STATIC);
  sim.body_set_name(staticS, "static sensor");
  const staticShape = sim.attach_box_ex(staticS, 1, 1, 0, 0, 0, 0, FRIC, 0, true, true, false, false, false, SENSOR, 0xffffffff);

  const kin = sim.add_body(0, 0, 0, BODY_KINEMATIC);
  sim.body_set_name(kin, "kinematic sensor");
  sim.set_linear_velocity(kin, 0, 1);
  const kinShape = sim.attach_box_ex(kin, 1, 1, 0, 0, 0, 0, FRIC, 0, true, true, false, false, false, SENSOR, 0xffffffff);

  const dyn = sim.add_body(3, 1, 0, BODY_DYNAMIC);
  sim.body_set_name(dyn, "dynamic sensor");
  const dynShape = sim.attach_box_ex(dyn, 1, 1, 0, 0, 0, 1, FRIC, 0, true, true, false, false, false, SENSOR, 0xffffffff);
  sim.attach_box_ex(dyn, 0.8, 0.8, 0, 0, 0, 1, FRIC, 0, false, false, false, false, false, DEFAULT_BITS, 0xffffffff);

  const ball = sim.add_body(-5, 1, 0, BODY_DYNAMIC);
  sim.body_set_name(ball, "ball_01");
  sim.attach_circle_ex(ball, 0, 0, 0.5, 1, FRIC, 0, 0, false, true, false, false, DEFAULT_BITS, GROUND | DEFAULT_BITS | SENSOR);

  let rayHit: { x: number; y: number } | null = null;
  let names = { static: "", kinematic: "", dynamic: "" };

  controls.appendChild(createInfoBox("Static / kinematic / dynamic sensors report visitor body names."));

  return {
    beforeStep: () => {
      const p = sim.positions();
      const y = p[kin * 3 + 1]!;
      if (y < 0) sim.set_linear_velocity(kin, 0, 1);
      else if (y > 3) sim.set_linear_velocity(kin, 0, -1);
    },
    afterStep: () => {
      names.static = sim.sensor_visitor_names(staticShape);
      names.kinematic = sim.sensor_visitor_names(kinShape);
      names.dynamic = sim.sensor_visitor_names(dynShape);
      const r = sim.cast_ray_closest(5, 1, -10, 0);
      rayHit = r[0]! > 0.5 ? { x: r[1]!, y: r[2]! } : null;
    },
    paintOverlay: (ctx, camera, canvas) => {
      const a = worldToScreen(camera, canvas, 5, 1);
      const b = worldToScreen(camera, canvas, -5, 1);
      ctx.strokeStyle = "#696969";
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      if (rayHit) {
        const p = worldToScreen(camera, canvas, rayHit.x, rayHit.y);
        ctx.fillStyle = "#0ff";
        ctx.beginPath();
        ctx.arc(p.x, p.y, 5, 0, 2 * PI);
        ctx.fill();
      }
    },
    readoutExtra: () => [
      { label: "static", value: names.static || "—" },
      { label: "kinematic", value: names.kinematic || "—" },
      { label: "dynamic", value: names.dynamic || "—" },
    ],
  };
}

function buildJoint(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :1832-2057
  const ground = sim.add_segment(-40, 0, 40, 0);
  const forceTh = 20000;
  const torqueTh = 10000;
  let x = -12.5;
  const y = 10;
  const joints: number[] = [];

  const makeBox = (px: number) => {
    const b = sim.add_body(px, y, 0, BODY_DYNAMIC);
    sim.enable_body_sleep(b, false);
    sim.attach_box(b, 1, 1, 0, 0, 0, 1, FRIC, 0);
    return b;
  };

  // distance
  {
    const b = makeBox(x);
    const j = sim.add_distance_joint_ex(ground, b, x, y + 1 + 2, x, y + 1, 2, false, 0, 0, 0, 0, false, 0, 0, true);
    sim.joint_set_force_threshold(j, forceTh);
    sim.joint_set_torque_threshold(j, torqueTh);
    sim.joint_set_user_data(j, 0);
    joints.push(j);
  }
  x += 5;
  // motor
  {
    const b = makeBox(x);
    const j = sim.add_motor_joint_local(ground, b, x, y, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 20, true);
    sim.joint_set_force_threshold(j, forceTh);
    sim.joint_set_torque_threshold(j, torqueTh);
    sim.joint_set_user_data(j, 1);
    joints.push(j);
  }
  x += 5;
  // prismatic
  {
    const b = makeBox(x);
    const j = sim.add_prismatic_joint_local(ground, b, x - 1, y, -1, 0, 1, 0, false, 0, 0, false, 0, 0, false, 0, 0, true);
    sim.joint_set_force_threshold(j, forceTh);
    sim.joint_set_torque_threshold(j, torqueTh);
    sim.joint_set_user_data(j, 2);
    joints.push(j);
  }
  x += 5;
  // revolute
  {
    const b = makeBox(x);
    const j = sim.add_revolute_joint_local(ground, b, x - 1, y, -1, 0, false, 0, 0, false, 0, 0, false, 0, 0, true);
    sim.joint_set_force_threshold(j, forceTh);
    sim.joint_set_torque_threshold(j, torqueTh);
    sim.joint_set_user_data(j, 3);
    joints.push(j);
  }
  x += 5;
  // weld
  {
    const b = makeBox(x);
    const j = sim.add_weld_joint_local(ground, b, x - 1, y, -1, 0, 0, 0, 0, 2, 0, 0.5, true);
    sim.joint_set_force_threshold(j, forceTh);
    sim.joint_set_torque_threshold(j, torqueTh);
    sim.joint_set_user_data(j, 4);
    joints.push(j);
  }
  x += 5;
  // wheel
  {
    const b = makeBox(x);
    const j = sim.add_wheel_joint(ground, b, x - 1, y, 0, 1, true, -1, 1, true, 1, 10, true, 1, 0.7, true);
    sim.joint_set_force_threshold(j, forceTh);
    sim.joint_set_torque_threshold(j, torqueTh);
    sim.joint_set_user_data(j, 5);
    joints.push(j);
  }

  return {
    afterStep: () => {
      const ev = sim.joint_events();
      for (let i = 0; i + 1 < ev.length; i += 2) {
        const ud = ev[i + 1]!;
        if (ud >= 0 && ud < joints.length) sim.destroy_joint_if_valid(joints[ud]!);
      }
    },
  };
}

function buildPersistentContact(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :2061-2160
  const points: number[] = [];
  let x = 10;
  for (let i = 0; i < 20; i++) {
    points.push(x, 0);
    x -= 1;
  }
  points.push(-9, 10, 10, 10);
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_chain_ex(ground, points, true, 0, 0, false, FRIC);

  const ball = sim.add_body(-8, 1, 0, BODY_DYNAMIC);
  sim.set_linear_velocity(ball, 2, 0);
  sim.attach_circle_ex(ball, 0, 0, 0.5, 1, FRIC, 0, 0, false, false, true, false, 0, 0);

  let contact: { index1: number; gen: number } | null = null;
  let drawPts: { x: number; y: number; impulse: number; nx: number; ny: number }[] = [];

  return {
    afterStep: () => {
      const begins = sim.contact_begin_events();
      if (begins.length >= 4 && !contact) {
        contact = { index1: begins[2]!, gen: begins[3]! };
      }
      const ends = sim.contact_end_events();
      for (let i = 0; i + 3 < ends.length; i += 4) {
        if (contact && ends[i + 2] === contact.index1 && ends[i + 3] === contact.gen) {
          contact = null;
          break;
        }
      }
      drawPts = [];
      if (contact && sim.contact_is_valid(contact.index1, contact.gen)) {
        const d = sim.contact_draw_data(contact.index1, contact.gen);
        const nx = d[2]!;
        const ny = d[3]!;
        const n = d[4]!;
        for (let i = 0; i < n; i++) {
          const o = 5 + i * 3;
          drawPts.push({ x: d[o]!, y: d[o + 1]!, impulse: d[o + 2]!, nx, ny });
        }
      } else {
        contact = null;
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      ctx.strokeStyle = "#dc143c";
      ctx.fillStyle = "#dc143c";
      for (const p of drawPts) {
        const a = worldToScreen(camera, canvas, p.x, p.y);
        const b = worldToScreen(camera, canvas, p.x + p.impulse * p.nx, p.y + p.impulse * p.ny);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(a.x, a.y, 3, 0, 2 * PI);
        ctx.fill();
      }
    },
  };
}

function buildSensorHits(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :2164-2387
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.body_set_name(ground, "ground");
  sim.attach_segment(ground, -10, 0, 10, 0);
  sim.attach_segment(ground, 10, 0, 10, 10);

  sim.attach_segment_ex(sim.add_body(-4, 1, 0, BODY_STATIC), 0, 0, 0, 10, true, true, false, false, 0, 0);

  const kin = sim.add_body(0, 1, 0, BODY_KINEMATIC);
  sim.set_linear_velocity(kin, 0.5, 0);
  sim.attach_segment_ex(kin, 0, 0, 0, 10, true, true, false, false, 0, 0);

  const dyn = sim.add_body(4, 1, 0, BODY_DYNAMIC);
  sim.attach_capsule_ex(dyn, 0, 1, 0, 9, 0.1, 1, FRIC, 0, true, true, false, 0, 0);
  const joint = sim.add_prismatic_joint(ground, dyn, 4, 7, 1, 0, false, 0, 0, true, 0.5, 1000, false, 0, 0, false);

  let bullet = true;
  let projectile = -1;
  let beginCount = 0;
  let endCount = 0;
  const transforms: { x: number; y: number; a: number }[] = [];
  const rng = makeXorShift(12345);

  const launch = () => {
    if (projectile >= 0 && sim.is_body_alive(projectile)) sim.destroy_body(projectile);
    transforms.length = 0;
    beginCount = 0;
    endCount = 0;
    projectile = sim.add_body(-26.7, 6, 0, BODY_DYNAMIC);
    sim.set_linear_velocity(projectile, rng.range(200, 300), 0);
    sim.set_bullet(projectile, bullet);
    sim.attach_circle_ex(projectile, 0, 0, 0.25, 1, 0.8, 0, 0.01, false, true, false, false, 0, 0);
  };
  launch();

  controls.appendChild(createCheckbox("Bullet", bullet, (en) => { bullet = en; }));
  controls.appendChild(createButton("Launch", launch));

  return {
    beforeStep: () => {
      const p = sim.positions();
      const kx = p[kin * 3]!;
      if (kx > 1) sim.set_linear_velocity(kin, -0.5, 0);
      else if (kx < -1) sim.set_linear_velocity(kin, 0.5, 0);
      // Prismatic motor reverse via GetTranslation (:C Sensor Hits)
      const translation = sim.prismatic_get_translation(joint);
      if (translation > 1) sim.prismatic_set_motor_speed(joint, -0.5);
      else if (translation < -1) sim.prismatic_set_motor_speed(joint, 0.5);
    },
    afterStep: () => {
      const begins = sim.sensor_begin_events();
      beginCount += begins.length / 2;
      endCount += sim.sensor_end_events().length / 2;
      for (let i = 0; i + 1 < begins.length; i += 2) {
        const s = begins[i]!;
        if (s >= 0 && sim.shape_is_valid(s) && transforms.length < 20) {
          const bi = sim.shape_body_index(s);
          if (bi >= 0) {
            const p = sim.positions();
            transforms.push({ x: p[bi * 3]!, y: p[bi * 3 + 1]!, a: p[bi * 3 + 2]! });
          }
        }
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      ctx.strokeStyle = "#0f0";
      for (const t of transforms) {
        const p = worldToScreen(camera, canvas, t.x, t.y);
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + Math.cos(t.a) * 20, p.y - Math.sin(t.a) * 20);
        ctx.stroke();
      }
    },
    readoutExtra: () => [
      { label: "begin touch count", value: String(beginCount) },
      { label: "end touch count", value: String(endCount) },
    ],
  };
}

function buildProjectileEvent(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :2392-2553
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_segment_ex(ground, 10, 0, 10, 20, false, true, false, false, 0, 0);
  sim.attach_segment_ex(ground, -30, 0, 30, 0, false, true, false, false, 0, 0);

  const offset = 0.01;
  for (let i = 0; i < 8; i++) {
    const shift = i % 2 === 0 ? -offset : offset;
    const b = sim.add_body(8 + shift, 0.5 + i, 0, BODY_DYNAMIC);
    sim.attach_rounded_box_ex(b, 0.45, 0.45, 0.05, 1, false, true);
  }

  let projectile = -1;
  let projShape = -1;
  let dragging = false;
  let p1 = { x: 0, y: 0 };
  let p2 = { x: 0, y: 0 };

  const fire = () => {
    if (projectile >= 0 && sim.is_body_alive(projectile)) sim.destroy_body(projectile);
    projectile = sim.add_body(p1.x, p1.y, 0, BODY_DYNAMIC);
    sim.set_linear_velocity(projectile, 4 * (p2.x - p1.x), 4 * (p2.y - p1.y));
    sim.set_bullet(projectile, true);
    projShape = sim.attach_circle_ex(projectile, 0, 0, 0.25, 1, FRIC, 0, 0, false, false, true, false, 0, 0);
  };

  controls.appendChild(createInfoBox("Ctrl + Left Mouse drag to aim and shoot a projectile."));

  return {
    afterStep: () => {
      if (projectile < 0 || !sim.is_body_alive(projectile)) return;
      const begins = sim.contact_begin_events();
      for (let i = 0; i + 3 < begins.length; i += 4) {
        if (begins[i] === projShape || begins[i + 1] === projShape) {
          const index1 = begins[i + 2]!;
          const gen = begins[i + 3]!;
          if (sim.contact_is_valid(index1, gen)) {
            const d = sim.contact_draw_data(index1, gen);
            if (d[4]! > 0) {
              sim.explode(d[5]!, d[6]!, 1, 0, 20);
              sim.destroy_body(projectile);
              projectile = -1;
              projShape = -1;
            }
          }
          break;
        }
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      if (!dragging) return;
      const a = worldToScreen(camera, canvas, p1.x, p1.y);
      const b = worldToScreen(camera, canvas, p2.x, p2.y);
      ctx.strokeStyle = "#fff";
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      ctx.fillStyle = "#0f0";
      ctx.beginPath();
      ctx.arc(a.x, a.y, 4, 0, 2 * PI);
      ctx.fill();
      ctx.fillStyle = "#f00";
      ctx.beginPath();
      ctx.arc(b.x, b.y, 4, 0, 2 * PI);
      ctx.fill();
    },
    onPointerDown: (wx, wy, mods) => {
      if (mods.ctrl) {
        dragging = true;
        p1 = { x: wx, y: wy };
        p2 = { x: wx, y: wy };
      }
    },
    onPointerMove: (wx, wy) => {
      if (dragging) p2 = { x: wx, y: wy };
    },
    onPointerUp: () => {
      if (dragging) {
        dragging = false;
        fire();
      }
    },
  };
}

function buildCircleImpulse(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :2557-2694
  sim.add_segment(-10, 0, 10, 0);
  let useGravity = false;
  let useRestitution = false;
  const mass = 1;
  const restitution = 0.25;
  let body = -1;
  const events: { speed: number; impulse: number; total: number }[] = [];
  const hits: { x: number; y: number }[] = [];

  const spawn = () => {
    if (body >= 0 && sim.is_body_alive(body)) sim.destroy_body(body);
    events.length = 0;
    hits.length = 0;
    body = sim.add_body(0, 5.5, 0, BODY_DYNAMIC);
    sim.set_gravity_scale(body, useGravity ? 1 : 0);
    sim.set_linear_velocity(body, 0, -25);
    sim.attach_circle_ex(body, 0, 0, 0.25, 1, 0, useRestitution ? restitution : 0, 0, false, false, false, true, 0, 0);
    sim.set_mass_data_scale(body, mass);
  };
  spawn();

  controls.appendChild(createCheckbox("gravity", useGravity, (en) => { useGravity = en; spawn(); }));
  controls.appendChild(createCheckbox("restitution", useRestitution, (en) => { useRestitution = en; spawn(); }));

  return {
    afterStep: () => {
      const h = sim.hit_events_ex();
      for (let i = 0; i + 6 < h.length; i += 7) {
        hits.push({ x: h[i]!, y: h[i + 1]! });
        const index1 = h[i + 5]!;
        const gen = h[i + 6]!;
        let impulse = 0;
        let total = 0;
        if (sim.contact_is_valid(index1, gen)) {
          const m = sim.contact_manifold(index1, gen);
          if (m[2]! > 0) {
            impulse = m[5]!;
            total = m[6]!;
          }
        }
        events.push({ speed: h[i + 2]!, impulse, total });
      }
    },
    paintOverlay: (ctx, camera, canvas) => {
      ctx.fillStyle = "#fff";
      for (const h of hits) {
        const p = worldToScreen(camera, canvas, h.x, h.y);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 5, 0, 2 * PI);
        ctx.fill();
      }
    },
    readoutExtra: () => {
      const rows = [
        {
          label: "setup",
          value: `mass=${mass}, gravity=${useGravity ? 10 : 0}, restitution=${useRestitution ? restitution : 0}`,
        },
      ];
      for (const e of events) {
        rows.push({
          label: "hit",
          value: `speed=${e.speed.toFixed(3)}, momentum=${(mass * e.speed).toFixed(3)}, impulse=${e.impulse.toFixed(3)}, total=${e.total.toFixed(3)}`,
        });
      }
      return rows;
    },
  };
}

function buildScene(scene: Scene, sim: SimWorld, controls: HTMLElement): SceneRuntime {
  controls.replaceChildren();
  switch (scene) {
    case "sensor-funnel":
      return buildSensorFunnel(sim, controls);
    case "sensor-bookend":
      return buildSensorBookend(sim, controls);
    case "foot-sensor":
      return buildFootSensor(sim, controls);
    case "contact":
      return buildContact(sim, controls);
    case "platformer":
      return buildPlatformer(sim, controls);
    case "body-move":
      return buildBodyMove(sim, controls);
    case "sensor-types":
      return buildSensorTypes(sim, controls);
    case "joint":
      return buildJoint(sim, controls);
    case "persistent-contact":
      return buildPersistentContact(sim, controls);
    case "sensor-hits":
      return buildSensorHits(sim, controls);
    case "projectile-event":
      return buildProjectileEvent(sim, controls);
    case "circle-impulse":
      return buildCircleImpulse(sim, controls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Events",
    "C <code>sample_events.cpp</code> RegisterSample ports — sensors, contacts, " +
      "body move, joint break, and hit impulses.",
    "Drag to grab · Ctrl+drag aims projectile · P pause · O step · R restart",
    { category: "Events", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "sensor-bookend";

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
    if (e.ctrlKey && runtime.onPointerDown) {
      runtime.onPointerDown(w.x, w.y, { ctrl: true });
      canvas.setPointerCapture(e.pointerId);
      return;
    }
    grabbing = sim.mouse_down(w.x, w.y);
    if (grabbing) canvas.setPointerCapture(e.pointerId);
  };
  const onPointerMove = (e: PointerEvent) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    if (runtime.onPointerMove && e.ctrlKey) {
      runtime.onPointerMove(w.x, w.y);
      return;
    }
    if (!grabbing && !sim.mouse_active()) return;
    sim.mouse_move(w.x, w.y);
  };
  const onPointerUp = () => {
    runtime.onPointerUp?.();
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
        history.replaceState(null, "", `#/events/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "events",
    category: "Events",
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
