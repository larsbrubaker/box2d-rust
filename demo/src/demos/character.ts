// Character — RegisterSample ports from sample_character.cpp.
// C has a single sample: Mover. Citations use sample_character.cpp line numbers.

import {
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
  worldToScreen,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — C RegisterSample("Character", "Mover"). */
export const SCENES = ["mover"] as const;
export type Scene = (typeof SCENES)[number];

assertRouteScenes("character", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  mover: "Mover",
};

/** C camera.center / camera.zoom (`sample_character.cpp:64-65`). */
const CAMERA0 = { cx: 20.0, cy: 9.0, zoom: 10.0 };

const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const ALL_BITS = 0xffffffff;
/** sample_character.cpp:19-26 */
const MOVER_BIT = 0x0002;
const DYNAMIC_BIT = 0x0004;
const DEBRIS_BIT = 0x0008;

const ELEVATOR_BASE = { x: 112.0, y: 10.0 };
const ELEVATOR_AMPLITUDE = 4.0;
const PI = Math.PI;

/** Capsule centers (±0.5) + radius 0.3 — draw only (`:73`). */
const CAPSULE = { c1y: -0.5, c2y: 0.5, r: 0.3 };

/**
 * C Sample::ParsePath (`sample.cpp:1047-1168`) — SVG path → interleaved points
 * with Y flip: `points = { scale*(x+ox), -scale*(y+oy) }`.
 */
function parsePath(
  svgPath: string,
  offsetX: number,
  offsetY: number,
  scale: number,
  capacity: number,
): number[] {
  const points: number[] = [];
  let currentX = 0;
  let currentY = 0;
  let command = svgPath[0] ?? "M";
  let i = 0;
  const n = svgPath.length;

  const isCommandChar = (c: string) => "MLHVmlhvz".includes(c);
  const skipSpaces = () => {
    while (i < n && /\s/.test(svgPath[i]!)) i++;
  };
  const readNumber = (): number => {
    skipSpaces();
    const start = i;
    if (svgPath[i] === "-" || svgPath[i] === "+") i++;
    while (i < n && /[0-9.eE]/.test(svgPath[i]!)) i++;
    const v = Number(svgPath.slice(start, i));
    if (svgPath[i] === ",") i++;
    skipSpaces();
    return v;
  };

  while (i < n && points.length / 2 < capacity) {
    skipSpaces();
    if (i >= n) break;
    const ch = svgPath[i]!;
    if (isCommandChar(ch)) {
      command = ch;
      if ("MLHVmlhv".includes(command)) {
        i += 1;
        if (svgPath[i] === " ") i += 1;
      }
      if (command === "z") break;
      continue;
    }

    switch (command) {
      case "M":
      case "L":
        currentX = readNumber();
        currentY = readNumber();
        break;
      case "H":
        currentX = readNumber();
        break;
      case "V":
        currentY = readNumber();
        break;
      case "m":
      case "l":
        currentX += readNumber();
        currentY += readNumber();
        break;
      case "h":
        currentX += readNumber();
        break;
      case "v":
        currentY += readNumber();
        break;
      default:
        return points;
    }

    points.push(scale * (currentX + offsetX), -scale * (currentY + offsetY));
  }
  return points;
}

// sample_character.cpp:81-86 / :109-116
const PATH1 =
  "M 2.6458333,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 " +
  "-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 H 142.875 v -5.29167 h -5.29166 v 5.29167 H 119.0625 v " +
  "-2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 H 84.666668 v -2.64583 h -5.291666 v " +
  "-2.64584 h -5.291667 v -2.64583 H 68.791668 V 174.625 h -5.291666 v -2.64584 H 52.916669 L 39.6875,177.27083 H " +
  "34.395833 L 23.8125,185.20833 H 15.875 L 5.2916669,187.85416 V 153.45833 H 2.6458333 v 47.625";

const PATH2 =
  "M 2.6458333,201.08333 H 293.68751 l 0,-23.8125 h -23.8125 l 21.16667,21.16667 h -23.8125 l -39.68751,-13.22917 " +
  "-26.45833,7.9375 -23.8125,2.64583 h -13.22917 l -0.0575,2.64584 h -5.29166 v -2.64583 l -7.86855,-1e-5 " +
  "-0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l -2.64584,2.64583 -2.58891,-2.64584 h -13.28609 v " +
  "-2.64583 h -2.64583 v -2.64584 l -5.29167,1e-5 v -2.64583 h -2.64583 v -2.64583 l -5.29167,-1e-5 v -2.64583 h " +
  "-2.64583 v -2.64584 h -5.291667 v -2.64583 H 92.60417 V 174.625 h -5.291667 v -2.64584 l -34.395835,1e-5 " +
  "-7.9375,-2.64584 -7.9375,-2.64583 -5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H " +
  "2.6458334 l -10e-8,47.625";

const GROUND1 = parsePath(PATH1, -50.0, -200.0, 0.2, 64);
const GROUND2 = parsePath(PATH2, 0.0, -200.0, 0.2, 64);

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  paintOverlay?: (
    ctx: CanvasRenderingContext2D,
    camera: SampleCamera,
    canvas: HTMLCanvasElement,
  ) => void;
  readoutExtra?: () => { label: string; value: string }[];
  dispose?: () => void;
  onKeyDown?: (e: KeyboardEvent) => void;
  onKeyUp?: (e: KeyboardEvent) => void;
  /** Manual camera input (right-drag / wheel) — disengage the follow-cam. */
  onUserCamera?: () => void;
}

function applyCamera(camera: SampleCamera) {
  camera.centerX = CAMERA0.cx;
  camera.centerY = CAMERA0.cy;
  camera.zoom = CAMERA0.zoom;
}

function buildMover(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_character.cpp:59-230
  const ground1 = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_chain(ground1, GROUND1, true);

  const ground2 = sim.add_body(98, 0, 0, BODY_STATIC);
  sim.attach_chain(ground2, GROUND2, true);

  // Bridge (:133-174)
  const xBase = 48.7;
  const yBase = 9.2;
  const bridgeCount = 50;
  let prev = ground1;
  for (let i = 0; i < bridgeCount; i++) {
    const body = sim.add_body(xBase + 0.5 + 1.0 * i, yBase, 0, BODY_DYNAMIC);
    sim.set_angular_damping(body, 0.2);
    sim.attach_box(body, 0.5, 0.125, 0, 0, 0, 1.0, 0.6, 0.0);
    sim.add_revolute_joint(
      prev,
      body,
      xBase + 1.0 * i,
      yBase,
      false,
      0,
      0,
      true,
      0,
      10.0,
      true,
      3.0,
      0.8,
      false,
    );
    prev = body;
  }
  sim.add_revolute_joint(
    prev,
    ground2,
    xBase + 1.0 * bridgeCount,
    yBase,
    false,
    0,
    0,
    true,
    0,
    10.0,
    true,
    3.0,
    0.8,
    false,
  );

  // Friendly soft capsule (:176-188)
  const friendly = sim.add_body(32.0, 4.5, 0, BODY_STATIC);
  const friendlyShape = sim.attach_capsule_mat(
    friendly,
    0,
    -0.5,
    0,
    0.5,
    0.3,
    0,
    0.6,
    0,
    0,
    0,
  );
  sim.shape_set_filter(friendlyShape, MOVER_BIT, ALL_BITS);
  sim.shape_set_plane_user_data(friendlyShape, 0.025, false);

  // Debris ball (:190-203)
  const ball = sim.add_body(7.0, 7.0, 0, BODY_DYNAMIC);
  const ballShape = sim.attach_circle_mat(ball, 0, 0, 0.3, 1.0, 0.6, 0.7, 0.2, 0);
  sim.shape_set_filter(ballShape, DEBRIS_BIT, ALL_BITS);

  // Elevator (:205-221)
  const elevator = sim.add_body(
    ELEVATOR_BASE.x,
    ELEVATOR_BASE.y - ELEVATOR_AMPLITUDE,
    0,
    BODY_KINEMATIC,
  );
  const elevShape = sim.attach_box_filter(elevator, 2.0, 0.1, 0, DYNAMIC_BIT, ALL_BITS);
  sim.shape_set_plane_user_data(elevShape, 0.1, true);

  // Mover spawn (:71)
  sim.mover_spawn(2.0, 8.0);
  sim.mover_set_params(10, 0.1, 6, 3, 20, 8, 30, 0.2, 5, 0.8);
  sim.mover_set_pogo_shape(2);

  let jumpSpeed = 10;
  let minSpeed = 0.1;
  let maxSpeed = 6;
  let stopSpeed = 3;
  let accelerate = 20;
  let friction = 8;
  let gravity = 30;
  let airSteer = 0.2;
  let pogoHertz = 5;
  let pogoDamping = 0.8;
  let pogoShape = 2;
  let lockCamera = true;
  let time = 0;
  let moverX = 2;
  let moverY = 8;
  let moverVx = 0;
  let moverVy = 0;
  let grounded = false;
  let planeCount = 0;
  let iterations = 0;
  const keys = new Set<string>();

  const applyParams = () => {
    sim.mover_set_params(
      jumpSpeed,
      minSpeed,
      maxSpeed,
      stopSpeed,
      accelerate,
      friction,
      gravity,
      airSteer,
      pogoHertz,
      pogoDamping,
    );
    sim.mover_set_pogo_shape(pogoShape);
  };

  controls.replaceChildren();
  controls.appendChild(
    createInfoBox(
      "C <code>Mover</code>: Quake friction + pogo spring + CollideMover / SolvePlanes / CastMover. " +
        "A/D throttle, W jump, K kick debris.",
    ),
  );
  controls.appendChild(
    createSlider("Jump Speed", 0, 40, jumpSpeed, 1, (v) => {
      jumpSpeed = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Min Speed", 0, 1, minSpeed, 0.01, (v) => {
      minSpeed = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Max Speed", 0, 20, maxSpeed, 1, (v) => {
      maxSpeed = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Stop Speed", 0, 10, stopSpeed, 0.1, (v) => {
      stopSpeed = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Accelerate", 0, 100, accelerate, 1, (v) => {
      accelerate = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Friction", 0, 10, friction, 0.1, (v) => {
      friction = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Gravity", 0, 100, gravity, 0.1, (v) => {
      gravity = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Air Steer", 0, 1, airSteer, 0.01, (v) => {
      airSteer = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Pogo Hertz", 0, 30, pogoHertz, 1, (v) => {
      pogoHertz = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createSlider("Pogo Damping", 0, 4, pogoDamping, 0.1, (v) => {
      pogoDamping = v;
      applyParams();
    }),
  );
  controls.appendChild(
    createDropdown(
      "Pogo Shape",
      [
        { value: "0", text: "Point" },
        { value: "1", text: "Circle" },
        { value: "2", text: "Segment" },
      ],
      String(pogoShape),
      (v) => {
        pogoShape = Number(v);
        applyParams();
      },
    ),
  );
  const lockCameraCheckbox = createCheckbox("Lock Camera", lockCamera, (v) => {
    lockCamera = v;
  });
  controls.appendChild(lockCameraCheckbox);
  controls.appendChild(createSeparator());

  return {
    beforeStep: (dt) => {
      if (dt > 0) {
        const y = ELEVATOR_AMPLITUDE * Math.cos(1.0 * time + PI) + ELEVATOR_BASE.y;
        sim.set_target_transform(elevator, ELEVATOR_BASE.x, y, 0, dt, true);
        time += dt;
      }
    },
    afterStep: (dt) => {
      let throttle = 0;
      // C sample_character.cpp:531-541 uses only A/D/W; the arrow keys are
      // reserved for camera panning (sample-shell bindCameraControls), so no
      // arrow aliases here — otherwise arrows would drive character and camera.
      if (keys.has("KeyA")) throttle -= 1;
      if (keys.has("KeyD")) throttle += 1;
      const jumpHeld = keys.has("KeyW");
      const state = sim.mover_update(dt, throttle, jumpHeld);
      moverX = state[0]!;
      moverY = state[1]!;
      moverVx = state[2]!;
      moverVy = state[3]!;
      grounded = state[4]! > 0.5;
      planeCount = state[5]!;
      iterations = state[6]!;
    },
    paintOverlay: (ctx, camera, canvas) => {
      const ppm = canvas.height / (2 * Math.max(1e-6, camera.zoom));

      const planes = sim.mover_planes();
      for (let i = 0; i + 2 < planes.length; i += 3) {
        const nx = planes[i]!;
        const ny = planes[i + 1]!;
        const offset = planes[i + 2]!;
        const p1x = moverX + (offset - CAPSULE.r) * nx;
        const p1y = moverY + (offset - CAPSULE.r) * ny;
        const a = worldToScreen(camera, canvas, p1x, p1y);
        const b = worldToScreen(camera, canvas, p1x + 0.1 * nx, p1y + 0.1 * ny);
        ctx.fillStyle = "#eab308";
        ctx.beginPath();
        ctx.arc(a.x, a.y, 3, 0, PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#eab308";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }

      const p1 = worldToScreen(camera, canvas, moverX, moverY + CAPSULE.c1y);
      const p2 = worldToScreen(camera, canvas, moverX, moverY + CAPSULE.c2y);
      const r = CAPSULE.r * ppm;
      ctx.beginPath();
      ctx.arc(p1.x, p1.y, r, Math.PI, 0);
      ctx.lineTo(p2.x + r, p2.y);
      ctx.arc(p2.x, p2.y, r, 0, Math.PI);
      ctx.closePath();
      ctx.fillStyle = grounded ? "#f973162a" : "#7fffd42a";
      ctx.strokeStyle = grounded ? "#f97316" : "#7fffd4";
      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();

      const origin = worldToScreen(camera, canvas, moverX, moverY);
      const velEnd = worldToScreen(camera, canvas, moverX + moverVx, moverY + moverVy);
      ctx.strokeStyle = "#a855f7";
      ctx.beginPath();
      ctx.moveTo(origin.x, origin.y);
      ctx.lineTo(velEnd.x, velEnd.y);
      ctx.stroke();

      const pogo = sim.mover_pogo_draw();
      if (pogo.length >= 7) {
        const a = worldToScreen(camera, canvas, pogo[0]!, pogo[1]!);
        const b = worldToScreen(camera, canvas, pogo[2]!, pogo[3]!);
        const hit = pogo[4]! > 0.5;
        const shape = pogo[5]!;
        const aux = pogo[6]!;
        ctx.strokeStyle = "#9ca3af";
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.fillStyle = hit ? "#dda0dd" : "#9ca3af";
        ctx.strokeStyle = hit ? "#dda0dd" : "#9ca3af";
        if (shape === 0) {
          ctx.beginPath();
          ctx.arc(b.x, b.y, 5, 0, PI * 2);
          ctx.fill();
        } else if (shape === 1) {
          ctx.beginPath();
          ctx.arc(b.x, b.y, aux * ppm, 0, PI * 2);
          ctx.stroke();
        } else {
          const half = aux * ppm;
          ctx.beginPath();
          ctx.moveTo(b.x - half, b.y);
          ctx.lineTo(b.x + half, b.y);
          ctx.stroke();
        }
      }

      const kick = sim.mover_kick_draw();
      if (kick.length >= 3) {
        const c = worldToScreen(camera, canvas, kick[0]!, kick[1]!);
        ctx.strokeStyle = "#daa520";
        ctx.beginPath();
        ctx.arc(c.x, c.y, kick[2]! * ppm, 0, PI * 2);
        ctx.stroke();
        sim.mover_clear_kick_draw();
      }

      if (lockCamera) camera.centerX = moverX;
    },
    readoutExtra: () => [
      { label: "position", value: `${moverX.toFixed(2)} ${moverY.toFixed(2)}` },
      { label: "velocity", value: `${moverVx.toFixed(2)} ${moverVy.toFixed(2)}` },
      { label: "iterations", value: String(iterations | 0) },
      { label: "planes", value: String(planeCount | 0) },
      { label: "grounded", value: grounded ? "yes" : "no" },
    ],
    onKeyDown: (e) => {
      keys.add(e.code);
      if (e.code === "KeyK") {
        sim.mover_kick();
        e.preventDefault();
      }
      if (["Space", "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(e.code)) {
        e.preventDefault();
      }
    },
    onKeyUp: (e) => {
      keys.delete(e.code);
    },
    onUserCamera: () => {
      // Manual pan / zoom disengages the follow-cam so the next painted frame
      // doesn't snap centerX back to the mover (see paintOverlay). Re-checking
      // "Lock Camera" resumes following.
      if (!lockCamera) return;
      lockCamera = false;
      const input = lockCameraCheckbox.querySelector("input");
      if (input) input.checked = false;
    },
    dispose: () => {
      keys.clear();
    },
  };
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Character",
    "Erin Catto’s <code>sample_character.cpp</code> Mover: capsule character controller over " +
      "chain terrain, a spring bridge, soft mover capsule, debris ball, and kinematic elevator.",
    "A/D move · W jump · K kick · P pause · O step · R restart",
    { category: "Character", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "mover";

  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = {};
  const camera = makeCamera(CAMERA0.cx, CAMERA0.cy, CAMERA0.zoom);
  const transport = createSampleTransport();
  const sceneControls = document.createElement("div");
  sceneControls.className = "scene-controls";
  const readout = createReadout();

  function rebuild() {
    runtime.dispose?.();
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera);
    runtime = buildMover(sim, sceneControls);
  }

  rebuild();

  controls.appendChild(
    createDropdown(
      "Sample",
      SCENES.map((s) => ({ value: s, text: SCENE_LABEL[s] })),
      scene,
      (v) => {
        scene = v as Scene;
        rebuild();
      },
    ),
  );
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "character",
    category: "Character",
    sampleName: SCENE_LABEL[scene],
    transport,
    onRestart: rebuild,
    getWorld: () => sim,
    onUserCamera: () => runtime.onUserCamera?.(),
  });
  chrome.afterHead.appendChild(sceneControls);
  controls.appendChild(readout);

  const onKeyDown = (e: KeyboardEvent) => runtime.onKeyDown?.(e);
  const onKeyUp = (e: KeyboardEvent) => runtime.onKeyUp?.(e);
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  const unbindTransport = transport.bindKeys(window);

  const stop = runSampleLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    runtime.beforeStep?.(dt);
    if (dt > 0) sim.step(dt, transport.subSteps);
    runtime.afterStep?.(dt);

    paintSampleDraw(canvas, camera, sim);
    const ctx = canvas.getContext("2d")!;
    runtime.paintOverlay?.(ctx, camera, canvas);

    updateReadout(readout, [
      { label: "Scene", value: SCENE_LABEL[scene] },
      { label: "Hz", value: String(transport.hertz) },
      ...(runtime.readoutExtra?.() ?? []),
    ]);
  }, { chrome, transport, camera, readout, getWorld: () => sim });

  return () => {
    stop();
    unbindTransport();
    chrome.dispose();
    disposeTransport(transport);
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    runtime.dispose?.();
    freeSim(sim);
  };
}
