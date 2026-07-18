// Robustness — RegisterSample ports from sample_robustness.cpp.
// C citations use sample_robustness.cpp line numbers at the pinned submodule.

import {
  createButton,
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
  DEFAULT_SUB_STEPS,
  disposeTransport,
  makeCamera,
  screenToWorld,
  type SampleCamera,
} from "./sample-shell.ts";

/** Registry scene keys — all seven C Robustness samples. */
export const SCENES = [
  "high-mass-ratio1",
  "high-mass-ratio2",
  "high-mass-ratio3",
  "overlap-recovery",
  "tiny-pyramid",
  "cart",
  "multiple-prismatic",
] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("robustness", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "high-mass-ratio1": "HighMassRatio1",
  "high-mass-ratio2": "HighMassRatio2",
  "high-mass-ratio3": "HighMassRatio3",
  "overlap-recovery": "Overlap Recovery",
  "tiny-pyramid": "Tiny Pyramid",
  cart: "Cart",
  "multiple-prismatic": "Multiple Prismatic",
};

/** C camera.center / camera.zoom (half-height). */
const CAMERAS: Record<Scene, { cx: number; cy: number; zoom: number }> = {
  "high-mass-ratio1": { cx: 3.0, cy: 14.0, zoom: 25.0 }, // :21-22
  "high-mass-ratio2": { cx: 0.0, cy: 16.5, zoom: 25.0 }, // :84-85
  "high-mass-ratio3": { cx: 0.0, cy: 16.5, zoom: 25.0 }, // :142-143
  "overlap-recovery": { cx: 0.0, cy: 2.5, zoom: 3.75 }, // :201-202
  "tiny-pyramid": { cx: 0.0, cy: 0.8, zoom: 1.0 }, // :322-323
  cart: { cx: 0.0, cy: 1.0, zoom: 1.5 }, // :388-389
  "multiple-prismatic": { cx: 0.0, cy: 8.0, zoom: 25.0 * 0.5 }, // :554-555
};

const BODY_STATIC = 0;
const BODY_DYNAMIC = 2;
const FRIC = 0.6;
const DEFAULT_GRAB_FORCE = 100;
const MULTI_PRISMATIC_GRAB_FORCE = 100000;

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

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

function buildHighMassRatio1(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:13-71 — Pyramid with heavy box on top (×3)
  const extent = 1.0;
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 50.0, 1.0, 0.0, -1.0, 0.0, 0.0, FRIC, 0);

  for (let j = 0; j < 3; ++j) {
    let count = 10;
    const offset = -20.0 * extent + 2.0 * (count + 1.0) * extent * j;
    let y = extent;
    while (count > 0) {
      for (let i = 0; i < count; ++i) {
        const coeff = i - 0.5 * count;
        const yy = count === 1 ? y + 2.0 : y;
        const body = sim.add_body(2.0 * coeff * extent + offset, yy, 0, BODY_DYNAMIC);
        const density = count === 1 ? (j + 1.0) * 100.0 : 1.0;
        sim.attach_box(body, extent, extent, 0, 0, 0, density, FRIC, 0);
      }
      --count;
      y += 2.0 * extent;
    }
  }
  return {};
}

function buildHighMassRatio2(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:76-129 — Big box on small boxes
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 50.0, 1.0, 0.0, -1.0, 0.0, 0.0, FRIC, 0);

  const extent = 1.0;
  const left = sim.add_body(-9.0 * extent, 0.5 * extent, 0, BODY_DYNAMIC);
  sim.attach_box(left, 0.5 * extent, 0.5 * extent, 0, 0, 0, 1.0, FRIC, 0);
  const right = sim.add_body(9.0 * extent, 0.5 * extent, 0, BODY_DYNAMIC);
  sim.attach_box(right, 0.5 * extent, 0.5 * extent, 0, 0, 0, 1.0, FRIC, 0);
  const big = sim.add_body(0.0, (10.0 + 16.0) * extent, 0, BODY_DYNAMIC);
  sim.attach_box(big, 10.0 * extent, 10.0 * extent, 0, 0, 0, 1.0, FRIC, 0);
  return {};
}

function buildHighMassRatio3(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:134-189 — Big box on small triangles
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 50.0, 1.0, 0.0, -1.0, 0.0, 0.0, FRIC, 0);

  const extent = 1.0;
  // C points: {-0.5e,0}, {0.5e,0}, {0,1e}
  const tri = [-0.5 * extent, 0.0, 0.5 * extent, 0.0, 0.0, 1.0 * extent];
  const left = sim.add_body(-9.0 * extent, 0.5 * extent, 0, BODY_DYNAMIC);
  sim.attach_polygon(left, tri, 0.0, 1.0, FRIC, 0);
  const right = sim.add_body(9.0 * extent, 0.5 * extent, 0, BODY_DYNAMIC);
  sim.attach_polygon(right, tri, 0.0, 1.0, FRIC, 0);
  const big = sim.add_body(0.0, (10.0 + 4.0) * extent, 0, BODY_DYNAMIC);
  sim.attach_box(big, 10.0 * extent, 10.0 * extent, 0, 0, 0, 1.0, FRIC, 0);
  return {};
}

function buildOverlapRecovery(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:193-307
  // Ground segment stays across CreateScene; dynamics are rebuilt in-place.
  let extent = 0.5;
  let baseCount = 4;
  let overlap = 0.25;
  let speed = 3.0;
  let hertz = 30.0;
  let dampingRatio = 10.0;
  const bodyIds: number[] = [];

  sim.add_segment(-40.0, 0.0, 40.0, 0.0);

  function createScene() {
    for (const id of bodyIds) {
      if (sim.is_body_alive(id)) sim.destroy_body(id);
    }
    bodyIds.length = 0;

    sim.set_contact_tuning(hertz, dampingRatio, speed);

    const boxHx = extent;
    const fraction = 1.0 - overlap;
    let y = extent;
    for (let i = 0; i < baseCount; ++i) {
      let x = fraction * extent * (i - baseCount);
      for (let j = i; j < baseCount; ++j) {
        const body = sim.add_body(x, y, 0, BODY_DYNAMIC);
        sim.attach_box(body, boxHx, boxHx, 0, 0, 0, 1.0, FRIC, 0);
        bodyIds.push(body);
        x += 2.0 * fraction * extent;
      }
      y += 2.0 * fraction * extent;
    }
  }

  createScene();

  // C DrawControls (:271-287)
  controls.appendChild(
    createSlider("Extent", 0.1, 1.0, extent, 0.1, (v) => {
      extent = v;
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Base Count", 1, 10, baseCount, 1, (v) => {
      baseCount = Math.round(v);
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Overlap", 0, 1, overlap, 0.01, (v) => {
      overlap = v;
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Speed", 0, 10, speed, 0.1, (v) => {
      speed = v;
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 240, hertz, 1, (v) => {
      hertz = v;
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Damping Ratio", 0, 20, dampingRatio, 0.1, (v) => {
      dampingRatio = v;
      createScene();
    }),
  );
  controls.appendChild(createButton("Reset Scene", () => createScene()));

  return {};
}

function buildTinyPyramid(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:314-373 — 5 cm squares
  const ground = sim.add_body(0, 0, 0, BODY_STATIC);
  sim.attach_box(ground, 5.0, 1.0, 0.0, -1.0, 0.0, 0.0, FRIC, 0);

  const extent = 0.025;
  const baseCount = 30;
  for (let i = 0; i < baseCount; ++i) {
    const y = (2.0 * i + 1.0) * extent;
    for (let j = i; j < baseCount; ++j) {
      const x = (i + 1.0) * extent + 2.0 * (j - i) * extent - baseCount * extent;
      const body = sim.add_body(x, y, 0, BODY_DYNAMIC);
      // b2MakeSquare(extent) → half-extent = extent
      sim.attach_box(body, extent, extent, 0, 0, 0, 1.0, FRIC, 0);
    }
  }

  return {
    // C DrawScreenTextLine("%.1fcm squares", 200 * extent) → 5.0cm
    readoutExtra: () => [{ label: "Squares", value: `${(200.0 * extent).toFixed(1)} cm` }],
  };
}

function buildCart(sim: SimWorld, controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:380-541 — high gravity / mass ratio cart
  const groundBody = sim.add_body(0.0, -1.0, 0, BODY_STATIC);
  sim.attach_box(groundBody, 20.0, 1.0, 0, 0, 0, 0.0, FRIC, 0);

  sim.set_gravity(0, -22.0);

  let contactHertz = 240.0;
  let contactDampingRatio = 10.0;
  let contactSpeed = 0.5;
  let constraintHertz = 240.0;
  let constraintDampingRatio = 0.0;

  sim.set_contact_tuning(contactHertz, contactDampingRatio, contactSpeed);

  let chassisId = -1;
  let wheelId1 = -1;
  let wheelId2 = -1;
  let jointId1 = -1;
  let jointId2 = -1;

  function createScene() {
    if (chassisId >= 0 && sim.is_body_alive(chassisId)) sim.destroy_body(chassisId);
    if (wheelId1 >= 0 && sim.is_body_alive(wheelId1)) sim.destroy_body(wheelId1);
    if (wheelId2 >= 0 && sim.is_body_alive(wheelId2)) sim.destroy_body(wheelId2);

    const yBase = 2.0;
    chassisId = sim.add_body(0.0, yBase, 0, BODY_DYNAMIC);
    // density 1000 chassis box offset (0, 0.25)
    sim.attach_box_mat(chassisId, 1.0, 0.25, 0.0, 0.25, 0.0, 1000.0, FRIC, 0, 0, 0);

    // wheels: density 50, rollingResistance 0.02
    wheelId1 = sim.add_body(-0.9, yBase - 0.15, 0, BODY_DYNAMIC);
    sim.attach_circle_mat(wheelId1, 0, 0, 0.1, 50.0, FRIC, 0, 0.02, 0);
    wheelId2 = sim.add_body(0.9, yBase - 0.15, 0, BODY_DYNAMIC);
    sim.attach_circle_mat(wheelId2, 0, 0, 0.1, 50.0, FRIC, 0, 0.02, 0);

    // C revolute: base.constraintHertz=120 then SetConstraintTuning to live values
    jointId1 = sim.add_revolute_joint_local(
      chassisId,
      wheelId1,
      -0.9,
      -0.15,
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
    sim.joint_set_constraint_tuning(jointId1, constraintHertz, constraintDampingRatio);

    jointId2 = sim.add_revolute_joint_local(
      chassisId,
      wheelId2,
      0.9,
      -0.15,
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
    sim.joint_set_constraint_tuning(jointId2, constraintHertz, constraintDampingRatio);
  }

  createScene();

  // C DrawControls (:486-518) — Contact block rebuilds scene; Joint block retunes + rebuilds
  controls.appendChild(createInfoBox("<strong>Contact</strong>"));
  controls.appendChild(
    createSlider("Contact Hertz", 0, 240, contactHertz, 1, (v) => {
      contactHertz = v;
      sim.set_contact_tuning(contactHertz, contactDampingRatio, contactSpeed);
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Contact Damping", 0, 100, contactDampingRatio, 1, (v) => {
      contactDampingRatio = v;
      sim.set_contact_tuning(contactHertz, contactDampingRatio, contactSpeed);
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Push Speed", 0, 5, contactSpeed, 0.1, (v) => {
      contactSpeed = v;
      sim.set_contact_tuning(contactHertz, contactDampingRatio, contactSpeed);
      createScene();
    }),
  );
  controls.appendChild(createSeparator());
  controls.appendChild(createInfoBox("<strong>Joint</strong>"));
  controls.appendChild(
    createSlider("Joint Hertz", 0, 240, constraintHertz, 1, (v) => {
      constraintHertz = v;
      if (jointId1 >= 0) sim.joint_set_constraint_tuning(jointId1, constraintHertz, constraintDampingRatio);
      if (jointId2 >= 0) sim.joint_set_constraint_tuning(jointId2, constraintHertz, constraintDampingRatio);
      createScene();
    }),
  );
  controls.appendChild(
    createSlider("Joint Damping", 0, 20, constraintDampingRatio, 1, (v) => {
      constraintDampingRatio = v;
      if (jointId1 >= 0) sim.joint_set_constraint_tuning(jointId1, constraintHertz, constraintDampingRatio);
      if (jointId2 >= 0) sim.joint_set_constraint_tuning(jointId2, constraintHertz, constraintDampingRatio);
      createScene();
    }),
  );
  controls.appendChild(createSeparator());
  controls.appendChild(
    createButton("Reset Scene", () => {
      if (jointId1 >= 0) sim.joint_set_constraint_tuning(jointId1, constraintHertz, constraintDampingRatio);
      if (jointId2 >= 0) sim.joint_set_constraint_tuning(jointId2, constraintHertz, constraintDampingRatio);
      createScene();
    }),
  );

  return {};
}

function buildMultiplePrismatic(sim: SimWorld, _controls: HTMLElement): SceneRuntime {
  // sample_robustness.cpp:546-600
  const groundId = sim.add_body(0, 0, 0, BODY_STATIC);

  let bodyIdA = groundId;
  let localAx = 0.0;
  let localAy = 0.0;

  for (let i = 0; i < 6; ++i) {
    const body = sim.add_body(0.0, 0.6 + 1.2 * i, 0, BODY_DYNAMIC);
    sim.attach_box(body, 0.5, 0.5, 0, 0, 0, 1.0, FRIC, 0);

    // C: localFrameB.p = {0,-0.6}; axis = identity (+X); limit ±6; constraintHertz 240
    const j = sim.add_prismatic_joint_local(
      bodyIdA,
      body,
      localAx,
      localAy,
      0.0,
      -0.6,
      1.0,
      0.0,
      true,
      -6.0,
      6.0,
      false,
      0,
      0,
      false,
      0,
      0,
      false,
    );
    // default damping 2.0; C only overrides Hertz to 240
    sim.joint_set_constraint_tuning(j, 240.0, 2.0);

    bodyIdA = body;
    localAx = 0.0;
    localAy = 0.6;
  }

  sim.set_grab_force_scale(MULTI_PRISMATIC_GRAB_FORCE);
  return {
    dispose: () => sim.set_grab_force_scale(DEFAULT_GRAB_FORCE),
  };
}

function buildScene(scene: Scene, sim: SimWorld, controls: HTMLElement): SceneRuntime {
  controls.replaceChildren();
  switch (scene) {
    case "high-mass-ratio1":
      return buildHighMassRatio1(sim, controls);
    case "high-mass-ratio2":
      return buildHighMassRatio2(sim, controls);
    case "high-mass-ratio3":
      return buildHighMassRatio3(sim, controls);
    case "overlap-recovery":
      return buildOverlapRecovery(sim, controls);
    case "tiny-pyramid":
      return buildTinyPyramid(sim, controls);
    case "cart":
      return buildCart(sim, controls);
    case "multiple-prismatic":
      return buildMultiplePrismatic(sim, controls);
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Robustness",
    "C <code>sample_robustness.cpp</code> RegisterSample ports — high mass ratios, " +
      "overlap recovery, tiny stacking, tuned cart, and distorted prismatics.",
    "Drag to grab · P pause · O step · R restart",
    { category: "Robustness", samplesShell: true }
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "high-mass-ratio1";

  const camera: SampleCamera = makeCamera();
  applyCamera(camera, scene);
  const transport = createSampleTransport({
    subSteps: scene === "cart" ? 12 : DEFAULT_SUB_STEPS,
  });
  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = {};

  const sceneControls = document.createElement("div");
  sceneControls.className = "scene-controls";

  function rebuild() {
    runtime.dispose?.();
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera, scene);
    if (scene === "cart") {
      // C Cart ctor sets subStepCount = 12 when not restarting; keep user value after.
      if (transport.subSteps === DEFAULT_SUB_STEPS) transport.subSteps = 12;
    }
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
        history.replaceState(null, "", `#/robustness/${scene}`);
        if (scene === "cart") transport.subSteps = 12;
        else if (transport.subSteps === 12) transport.subSteps = DEFAULT_SUB_STEPS;
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "robustness",
    category: "Robustness",
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
