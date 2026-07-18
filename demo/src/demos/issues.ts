// Issues — RegisterSample ports from sample_issues.cpp.
// Regression scenes for known solver / geometry edge cases.

import {
  createButtonGroup,
  createCheckbox,
  createDropdown,
  createReadout,
  createSeparator,
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
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — must match slugify(C name). */
export const SCENES = [
  "bad-steiner",
  "disable",
  "crash01",
  "staticvsbulletbug",
  "unstable-prismatic-joints",
  "unstable-windmill",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("issues", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "bad-steiner": "Bad Steiner",
  disable: "Disable",
  crash01: "Crash01",
  staticvsbulletbug: "StaticVsBulletBug",
  "unstable-prismatic-joints": "Unstable Prismatic Joints",
  "unstable-windmill": "Unstable Windmill",
};

const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "bad-steiner": { cx: 0.0, cy: 1.75, zoom: 2.5 }, // :15-16
  disable: { cx: 0.8, cy: 6.4, zoom: 25.0 * 0.4 }, // :64-65
  crash01: { cx: 0.8, cy: 6.4, zoom: 25.0 * 0.4 }, // :143-144
  staticvsbulletbug: { cx: 48.8525391, cy: 68.1518555, zoom: 100.0 * 0.5 }, // :274-275
  "unstable-prismatic-joints": { cx: 0.0, cy: 1.75, zoom: 32.0 }, // :338-339
  "unstable-windmill": { cx: 0.0, cy: 1.75, zoom: 32.0 }, // :433-434
};

const FRIC = 0.6;
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;
const PI = Math.PI;

interface SceneRuntime {
  beforeStep?: (dt: number) => void;
  afterStep?: (dt: number) => void;
  dispose?: () => void;
  readoutExtra?: () => { label: string; value: string }[];
}

function applyCamera(camera: SampleCamera, scene: Scene) {
  const c = CAMERAS[scene];
  camera.centerX = c.cx;
  camera.centerY = c.cy;
  camera.zoom = c.zoom;
}

// ---------------------------------------------------------------------------
// Scene builders — cite sample_issues.cpp
// ---------------------------------------------------------------------------

function buildBadSteiner(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :7-54
  sim.add_segment(-100.0, 0.0, 100.0, 0.0);

  const body = sim.add_body(-48.0, 62.0, 0.0, BODY_DYNAMIC);
  sim.attach_polygon(
    body,
    [
      48.7599983, -60.5699997,
      48.7400017, -60.5400009,
      48.6800003, -60.5600014,
    ],
    0.0,
    1.0,
    FRIC,
    0.0,
  );
  return {};
}

function buildDisable(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :56-133
  let isEnabled = true;

  const attachment = sim.add_body(-2.0, 3.0, 0.0, BODY_DYNAMIC);
  sim.attach_box(attachment, 0.5, 2.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  // Platform is static (:85-87); offset box hx=0.5 hy=4 at (4,0) rot 0.5π
  const platform = sim.add_body(-4.0, 5.0, 0.0, BODY_STATIC);
  sim.attach_box(platform, 0.5, 4.0, 4.0, 0.0, 0.5 * PI, 1.0, FRIC, 0.0);

  sim.add_revolute_joint(
    attachment,
    platform,
    -2.0,
    5.0,
    false,
    0,
    0,
    true,
    0.0,
    50.0,
    false,
    0,
    0,
    false,
  );

  controls.appendChild(
    createCheckbox("Enable", isEnabled, (en) => {
      isEnabled = en;
      if (en) sim.enable_body(attachment);
      else sim.disable_body(attachment);
    }),
  );
  return {};
}

function buildCrash01(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // :135-264
  let type = BODY_DYNAMIC;
  let isEnabled = true;

  const ground = sim.add_segment(-20.0, 0.0, 20.0, 0.0);

  const attachment = sim.add_body(-2.0, 3.0, 0.0, BODY_DYNAMIC);
  sim.attach_box(attachment, 0.5, 2.0, 0.0, 0.0, 0.0, 1.0, FRIC, 0.0);

  const platform = sim.add_body(-4.0, 5.0, 0.0, type);
  sim.attach_box(platform, 0.5, 4.0, 4.0, 0.0, 0.5 * PI, 2.0, FRIC, 0.0);

  sim.add_revolute_joint(
    attachment,
    platform,
    -2.0,
    5.0,
    false,
    0,
    0,
    true,
    0.0,
    50.0,
    false,
    0,
    0,
    false,
  );

  sim.add_prismatic_joint(
    ground,
    platform,
    0.0,
    5.0,
    1.0,
    0.0,
    true,
    -10.0,
    10.0,
    true,
    0.0,
    1000.0,
    false,
    0,
    0,
    false,
  );

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Static", value: "0" },
        { label: "Kinematic", value: "1" },
        { label: "Dynamic", value: "2" },
      ],
      "2",
      (v) => {
        type = Number(v);
        sim.set_body_type(platform, type);
        if (type === BODY_KINEMATIC) {
          sim.set_linear_velocity(platform, -0.1, 0.0);
        }
      },
    ),
  );
  controls.appendChild(
    createCheckbox("Enable", isEnabled, (en) => {
      isEnabled = en;
      if (en) sim.enable_body(attachment);
      else sim.disable_body(attachment);
    }),
  );
  return {};
}

function buildStaticVsBulletBug(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :266-326 — create as dynamic, attach hull poly, then SetType(static)
  const wall = sim.add_body(0.0, 0.0, 0.0, BODY_DYNAMIC);
  sim.attach_polygon_mat(
    wall,
    [
      48.8525391, 68.1518555,
      49.1821289, 68.1152344,
      68.8476562, 68.1152344,
      68.8476562, 70.2392578,
      48.8525391, 70.2392578,
    ],
    0.0,
    1.0,
    0.5,
    0.1,
    0.0,
    0.0,
  );
  sim.set_body_type(wall, BODY_STATIC);

  const ball = sim.add_body(58.924305, 77.5401459, 0.0, BODY_DYNAMIC);
  sim.set_motion_locks(ball, false, false, true);
  sim.set_linear_velocity(ball, 104.868881, -281.073883);
  sim.set_bullet(ball, true);
  sim.attach_circle_mat(ball, 0.0, 0.0, 0.3, 3.0, 0.2, 0.9, 0.0, 0.0);
  return {};
}

function buildUnstablePrismaticJoints(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :328-423 — light center mass between two spring prismatics
  sim.add_segment(-100.0, 0.0, 100.0, 0.0);

  const center = sim.add_body(0.0, 3.0, 0.0, BODY_DYNAMIC);
  sim.attach_circle(center, 0.0, 0.0, 0.5, 1.0, FRIC, 0.0);

  const left = sim.add_body(-3.5, 3.0, 0.0, BODY_DYNAMIC);
  sim.attach_circle(left, 0.0, 0.0, 2.0, 1.0, FRIC, 0.0);
  const jLeft = sim.add_prismatic_joint_local(
    center,
    left,
    0,
    0,
    0,
    0,
    1,
    0,
    false,
    0,
    0,
    false,
    0,
    0,
    true,
    10.0,
    2.0,
    false,
  );
  sim.prismatic_set_target_translation(jLeft, -3.0);

  const right = sim.add_body(3.5, 3.0, 0.0, BODY_DYNAMIC);
  sim.attach_circle(right, 0.0, 0.0, 2.0, 1.0, FRIC, 0.0);
  const jRight = sim.add_prismatic_joint_local(
    center,
    right,
    0,
    0,
    0,
    0,
    1,
    0,
    false,
    0,
    0,
    false,
    0,
    0,
    true,
    10.0,
    2.0,
    false,
  );
  sim.prismatic_set_target_translation(jRight, 3.0);

  return {};
}

function buildUnstableWindmill(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // :425-507 — welded rotors; constraintHertz 30 for stability knob
  sim.add_segment(-100.0, -10.0, 100.0, -10.0);

  const makeRotorBody = (x: number, y: number) => {
    const b = sim.add_body(x, y, 0.0, BODY_DYNAMIC);
    sim.set_gravity_scale(b, 0.0);
    return b;
  };

  const center = makeRotorBody(10.0, 10.0);
  sim.attach_circle(center, 0.0, 0.0, 5.0, 1.0, 0.1, 0.0);

  const weld = (body: number, ax: number, ay: number, bx: number, by: number) => {
    const j = sim.add_weld_joint_local(
      center,
      body,
      ax,
      ay,
      bx,
      by,
      0,
      0,
      0,
      0,
      0,
      0,
      false,
    );
    sim.joint_set_constraint_tuning(j, 30.0, 2.0);
  };

  const south = makeRotorBody(10.0, 0.0);
  sim.attach_box(south, 4.0, 5.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0);
  weld(south, 0, -5, 0, 5);

  const east = makeRotorBody(20.0, 10.0);
  sim.attach_box(east, 5.0, 4.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0);
  weld(east, 5, 0, -5, 0);

  const north = makeRotorBody(10.0, 20.0);
  sim.attach_box(north, 4.0, 5.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0);
  weld(north, 0, 5, 0, -5);

  const west = makeRotorBody(0.0, 10.0);
  sim.attach_box(west, 5.0, 4.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0);
  weld(west, -5, 0, 5, 0);

  return {};
}

function buildScene(scene: Scene, sim: SimWorld, controls: HTMLElement): SceneRuntime {
  controls.replaceChildren();
  switch (scene) {
    case "bad-steiner":
      return buildBadSteiner(sim, controls);
    case "disable":
      return buildDisable(sim, controls);
    case "crash01":
      return buildCrash01(sim, controls);
    case "staticvsbulletbug":
      return buildStaticVsBulletBug(sim, controls);
    case "unstable-prismatic-joints":
      return buildUnstablePrismaticJoints(sim, controls);
    case "unstable-windmill":
      return buildUnstableWindmill(sim, controls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Issues",
    "C <code>sample_issues.cpp</code> RegisterSample ports — Steiner hull, " +
      "enable/disable, bullet-vs-static, and stiff joint stress cases.",
    "Drag to grab · P pause · O step · R restart",
    { category: "Issues", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "bad-steiner";

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
        history.replaceState(null, "", `#/issues/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "issues",
    category: "Issues",
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
    sim.step(dt, transport.subSteps);
    runtime.afterStep?.(dt);

    paintSampleDraw(canvas, camera, sim);

    updateReadout(readout, [
      { label: "Sample", value: SCENE_LABEL[scene] },
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Joints", value: String(sim.joint_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
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
