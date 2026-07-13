// Determinism — RegisterSample ports from sample_determinism.cpp + shared/determinism.c.
// Falling Hinges and SnapShot share CreateFallingHinges / UpdateFallingHinges.

import {
  createDropdown,
  createInfoBox,
  createReadout,
  createSeparator,
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
export const SCENES = ["falling-hinges", "snapshot"] as const;

export type Scene = (typeof SCENES)[number];

assertRouteScenes("determinism", SCENES);

const SCENE_LABEL: Record<Scene, string> = {
  "falling-hinges": "Falling Hinges",
  snapshot: "SnapShot",
};

/** C camera.center / camera.zoom (sample_determinism.cpp :28-29, :104-105). */
const CAMERA = { cx: 0.0, cy: 7.5, zoom: 10.0 };

const BODY_DYNAMIC = 2;
const PI = Math.PI;
/** CrossPlatformTest float pin (test_determinism.c / determinism_tests.rs). */
const EXPECTED_SLEEP_STEP = 294;
const EXPECTED_HASH = 0x006f0f5e;

interface FallingHingeData {
  bodyIds: number[];
  stepCount: number;
  sleepStep: number;
  hash: number;
}

interface SceneRuntime {
  data: FallingHingeData;
  done: boolean;
  /** Sample::m_stepCount — SnapShot snapshot/restore gates. */
  sampleStep: number;
  image: Uint8Array | null;
  afterStep?: () => void;
  readoutExtra?: () => { label: string; value: string }[];
}

function applyCamera(camera: SampleCamera) {
  camera.centerX = CAMERA.cx;
  camera.centerY = CAMERA.cy;
  camera.zoom = CAMERA.zoom;
}

/**
 * shared/determinism.c CreateFallingHinges — ground + 4×20 hinged boxes.
 * Uses default shape friction (0.6) and revolute local frames (-h,h)/(-h,-h).
 */
function createFallingHinges(sim: SimWorld): FallingHingeData {
  // :14-21 ground
  sim.add_static_box(0.0, -1.0, 40.0, 1.0);

  const columnCount = 4;
  const rowCount = 20;
  const bodyIds: number[] = [];

  const h = 0.25;
  // C MakeSquare(h) after a dead MakeRoundedBox store — :29-32
  const offset = 0.4 * h;
  const dx = 10.0 * h;
  const xBase = -0.5 * dx * (columnCount - 1.0);

  for (let j = 0; j < columnCount; ++j) {
    const x = xBase + j * dx;
    let prevBodyId = -1;

    for (let i = 0; i < rowCount; ++i) {
      // :72 — deterministic sin/cos via MakeRot
      const angle = (i & 1) === 0 ? -0.1 : 0.1;
      const bodyId = sim.add_body(x + offset * i, h + 2.0 * h * i, angle, BODY_DYNAMIC);
      // default_shape_def: density 1, friction 0.6
      sim.attach_box(bodyId, h, h, 0, 0, 0, 1.0, 0.6, 0);

      if ((i & 1) === 0) {
        prevBodyId = bodyId;
      } else {
        // :37-50 revolute: limit, spring, motor; local frames (-h,h)/(-h,-h)
        // constraint_hertz 60 / damping 0 come from default_revolute_joint_def
        sim.add_revolute_joint_local(
          prevBodyId,
          bodyId,
          -h,
          h,
          -h,
          -h,
          true,
          -0.1 * PI,
          0.2 * PI,
          true,
          0.0,
          0.25,
          true,
          1.0,
          1.0,
          false,
        );
        prevBodyId = -1;
      }

      bodyIds.push(bodyId);
    }
  }

  return {
    bodyIds,
    stepCount: 0,
    sleepStep: -1,
    hash: 0,
  };
}

/** shared/determinism.c UpdateFallingHinges — hash when all asleep. */
function updateFallingHinges(sim: SimWorld, data: FallingHingeData): boolean {
  if (data.hash === 0) {
    const moves = sim.body_move_events();
    if (moves.length === 0) {
      // C asserts awakeCount == 0 here
      if (sim.awake_body_count() === 0) {
        data.hash = sim.hash_body_transforms(new Uint32Array(data.bodyIds));
        data.sleepStep = data.stepCount;
      }
    }
  }
  data.stepCount += 1;
  return data.hash !== 0;
}

function buildFallingHinges(sim: SimWorld): SceneRuntime {
  const data = createFallingHinges(sim);
  const rt: SceneRuntime = {
    data,
    done: false,
    sampleStep: 0,
    image: null,
    afterStep() {
      if (!rt.done) {
        rt.done = updateFallingHinges(sim, data);
      }
    },
    readoutExtra() {
      if (rt.done) {
        const match =
          data.sleepStep === EXPECTED_SLEEP_STEP && data.hash === EXPECTED_HASH;
        return [
          { label: "sleep step", value: String(data.sleepStep) },
          { label: "hash", value: `0x${data.hash.toString(16).toUpperCase().padStart(8, "0")}` },
          { label: "CrossPlatform", value: match ? "match" : "diverge" },
        ];
      }
      return [
        { label: "hinge step", value: String(data.stepCount) },
        { label: "awake", value: String(sim.awake_body_count()) },
      ];
    },
  };
  return rt;
}

function buildSnapShot(sim: SimWorld): SceneRuntime {
  const data = createFallingHinges(sim);
  const rt: SceneRuntime = {
    data,
    done: false,
    sampleStep: 0,
    image: null,
    afterStep() {
      // sample_determinism.cpp SnapShot::Step — gates on Sample::m_stepCount
      if (rt.sampleStep === 50) {
        rt.image = sim.snapshot();
      } else if (rt.sampleStep === 150 && rt.image != null) {
        sim.restore(rt.image);
      }
      if (!rt.done) {
        rt.done = updateFallingHinges(sim, data);
      }
    },
    readoutExtra() {
      const rows: { label: string; value: string }[] = [
        { label: "sample step", value: String(rt.sampleStep) },
        {
          label: "snapshot",
          value:
            rt.sampleStep < 50
              ? "pending @50"
              : rt.sampleStep < 150
                ? `held ${rt.image?.byteLength ?? 0} B`
                : "restored @150",
        },
      ];
      if (rt.done) {
        rows.push(
          { label: "sleep step", value: String(data.sleepStep) },
          {
            label: "hash",
            value: `0x${data.hash.toString(16).toUpperCase().padStart(8, "0")}`,
          },
        );
      } else {
        rows.push({ label: "hinge step", value: String(data.stepCount) });
      }
      return rows;
    },
  };
  return rt;
}

function buildScene(scene: Scene, sim: SimWorld): SceneRuntime {
  return scene === "snapshot" ? buildSnapShot(sim) : buildFallingHinges(sim);
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Determinism",
    "C <code>sample_determinism.cpp</code> — Falling Hinges soak (CrossPlatformTest) " +
      "and SnapShot mid-run restore of the same scene.",
    "P pause · O step · R restart · drag to grab",
  );

  let scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "falling-hinges";

  const camera: SampleCamera = makeCamera();
  applyCamera(camera);
  const transport = createSampleTransport();
  let sim: SimWorld = null as unknown as SimWorld;
  let runtime: SceneRuntime = null as unknown as SceneRuntime;

  function rebuild() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    applyCamera(camera);
    runtime = buildScene(scene, sim);
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
        history.replaceState(null, "", `#/determinism/${scene}`);
        rebuild();
      },
    ),
  );
  controls.appendChild(createSeparator());
  transport.mountControls(controls, () => rebuild());
  controls.appendChild(createSeparator());
  controls.appendChild(
    createInfoBox(
      scene === "snapshot"
        ? "SnapShot captures <code>b2World_Snapshot</code> at sample step 50 and " +
            "<code>b2World_Restore</code> at 150, then continues until sleep " +
            "(<code>sample_determinism.cpp</code>)."
        : "Falling Hinges is the CrossPlatformTest visual: 80 hinged boxes settle; " +
            "sleep step + transform hash must match the unit-test pin " +
            "(<code>shared/determinism.c</code>).",
    ),
  );
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const unbindKeys = transport.bindKeys();

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    sim.step(dt, transport.subSteps);
    if (dt > 0) {
      runtime.sampleStep += 1;
      runtime.afterStep?.();
    }

    const b = viewBounds(camera, canvas);
    sim.collect_draw(b.lowerX, b.lowerY, b.upperX, b.upperY);
    paintDebugDraw(canvas, camera, {
      polygons: sim.draw_polygons(),
      circles: sim.draw_circles(),
      capsules: sim.draw_capsules(),
      lines: sim.draw_lines(),
    });

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
    disposeTransport(transport);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    freeSim(sim);
  };
}
