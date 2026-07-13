// Stacking — 1:1 ports of RegisterSample entries from sample_stacking.cpp.
// Shared Phase 1 harness: pause/step/restart, Hertz/sub-steps, C camera,
// mouse grab, engine debug draw.

import {
  createButton,
  createDropdown,
  createInfoBox,
  createReadout,
  createSeparator,
  createSlider,
  updateReadout,
} from "../controls.ts";
import { assertRouteScenes, entryHref, findByRouteName } from "../registry.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { paintSampleDraw } from "./debug-draw.ts";
import { demoPage, fitCanvas, freeSim, runSimLoop } from "./sim-common.ts";
import {
  createSampleTransport,
  mountSampleChrome,
  disposeTransport,
  makeCamera,
  screenToWorld,
  type SampleCamera,
} from "./sample-shell.ts";

/** Scene keys matching registry `scene` for route `stacking`. */
export const SCENES = [
  "single-box",
  "tilted-stack",
  "vertical-stack",
  "circle-stack",
  "capsule-stack",
  "cliff",
  "arch",
  "double-domino",
  "confined",
  "card-house",
] as const;

export type StackingScene = (typeof SCENES)[number];

assertRouteScenes("stacking", SCENES);

const SCENE_LABELS: Record<StackingScene, string> = {
  "single-box": "Single Box",
  "tilted-stack": "Tilted Stack",
  "vertical-stack": "Vertical Stack",
  "circle-stack": "Circle Stack",
  "capsule-stack": "Capsule Stack",
  cliff: "Cliff",
  arch: "Arch",
  "double-domino": "Double Domino",
  confined: "Confined",
  "card-house": "Card House",
};

function cameraFor(scene: StackingScene): SampleCamera {
  // C sample_stacking.cpp camera.center / camera.zoom (half-height).
  switch (scene) {
    case "single-box":
      return makeCamera(0.0, 2.5, 3.5); // :23-24
    case "tilted-stack":
      return makeCamera(7.5, 7.5, 20.0); // :75-76
    case "vertical-stack":
      return makeCamera(-7.0, 9.0, 14.0); // :164-165
    case "circle-stack":
      return makeCamera(0.0, 5.0, 6.0); // :424-425
    case "capsule-stack":
      return makeCamera(0.0, 5.0, 6.0); // :523-524
    case "cliff":
      return makeCamera(0.0, 5.0, 25.0 * 0.5); // :579-580
    case "arch":
      return makeCamera(0.0, 8.0, 25.0 * 0.35); // :721-722
    case "double-domino":
      return makeCamera(0.0, 4.0, 25.0 * 0.25); // :819-820
    case "confined":
      return makeCamera(0.0, 10.0, 25.0 * 0.5); // :872-873
    case "card-house":
      return makeCamera(0.75, 0.9, 25.0 * 0.05); // :945-946
  }
}

type SceneState = {
  /** Vertical Stack controls (C VerticalStack). */
  vsShape: "circle" | "box";
  vsRows: number;
  vsColumns: number;
  vsBulletCount: number;
  vsBulletType: "circle" | "box";
  vsBodies: (number | null)[];
  vsBullets: (number | null)[];
  /** Cliff flip (C Cliff::m_flip). */
  cliffFlip: boolean;
  /** Circle Stack accumulated hit pairs (C CircleStack::m_events). */
  circleHits: { a: number; b: number }[];
  /** Single Box body index for HUD. */
  singleBoxId: number;
};

function freshState(): SceneState {
  return {
    vsShape: "box",
    vsRows: 12,
    vsColumns: 1,
    vsBulletCount: 1,
    vsBulletType: "circle",
    vsBodies: [],
    vsBullets: [],
    cliffFlip: false,
    circleHits: [],
    singleBoxId: -1,
  };
}

// ---------------------------------------------------------------------------
// Scene builders — literal values from sample_stacking.cpp
// ---------------------------------------------------------------------------

function buildSingleBox(sim: SimWorld, state: SceneState) {
  // SingleBox :27-44
  const extent = 1.0;
  const groundWidth = 66.0 * extent;
  const ground = sim.add_body(0.0, 0.0, 0.0, 0);
  sim.attach_segment(ground, -0.5 * 2.0 * groundWidth, 0.0, 0.5 * 2.0 * groundWidth, 0.0);

  const box = sim.add_body(0.0, 1.0, 0.0, 2);
  sim.attach_box(box, extent, extent, 0.0, 0.0, 0.0, 1.0, 0.6, 0.0);
  sim.set_linear_velocity(box, 5.0, 0.0);
  state.singleBoxId = box;
}

function buildTiltedStack(sim: SimWorld) {
  // TiltedStack :79-122
  const ground = sim.add_body(0.0, -1.0, 0.0, 0);
  sim.attach_box(ground, 1000.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0);

  const columns = 10;
  const rows = 10;
  const offset = 0.2;
  const dx = 5.0;
  const xroot = -0.5 * dx * (columns - 1.0);

  for (let j = 0; j < columns; ++j) {
    const x = xroot + j * dx;
    for (let i = 0; i < rows; ++i) {
      const body = sim.add_body(x + offset * i, 0.5 + 1.0 * i, 0.0, 2);
      // b2MakeRoundedBox(0.45, 0.45, 0.05); density 1; friction 0.3
      sim.attach_rounded_box(body, 0.45, 0.45, 0.05, 1.0, 0.3, 0.0);
    }
  }
}

const VS_MAX_ROWS = 80;
const VS_MAX_COLUMNS = 10;
const VS_MAX_BULLETS = 8;

function createVerticalStacks(sim: SimWorld, state: SceneState) {
  // VerticalStack::CreateStacks :201-262
  for (const id of state.vsBodies) {
    if (id != null && sim.is_body_alive(id)) sim.destroy_body(id);
  }
  state.vsBodies = [];

  const offset = state.vsShape === "circle" ? 0.0 : 0.01;
  const dx = -3.0;
  const xroot = 8.0;

  for (let j = 0; j < state.vsColumns; ++j) {
    const x = xroot + j * dx;
    for (let i = 0; i < state.vsRows; ++i) {
      const shift = i % 2 === 0 ? -offset : offset;
      const body = sim.add_body(x + shift, 0.5 + 1.0 * i, 0.0, 2);
      if (state.vsShape === "circle") {
        sim.attach_circle(body, 0.0, 0.0, 0.5, 1.0, 0.3, 0.0);
      } else {
        sim.attach_rounded_box(body, 0.45, 0.45, 0.05, 1.0, 0.3, 0.0);
      }
      state.vsBodies.push(body);
    }
  }
}

function destroyVerticalBullets(sim: SimWorld, state: SceneState) {
  for (const id of state.vsBullets) {
    if (id != null && sim.is_body_alive(id)) sim.destroy_body(id);
  }
  state.vsBullets = [];
}

function fireVerticalBullets(sim: SimWorld, state: SceneState) {
  // VerticalStack::FireBullets :297-326
  while (state.vsBullets.length < state.vsBulletCount) state.vsBullets.push(null);
  for (let i = 0; i < state.vsBulletCount; ++i) {
    const existing = state.vsBullets[i];
    if (existing != null && sim.is_body_alive(existing)) continue;
    const speed = 200.0 + Math.random() * 100.0; // RandomFloatRange(200, 300)
    const body = sim.add_body(-26.7 - i, 6.0, 0.0, 2);
    sim.set_linear_velocity(body, speed, 0.0);
    sim.set_bullet(body, true);
    if (state.vsBulletType === "box") {
      sim.attach_box(body, 0.25, 0.25, 0.0, 0.0, 0.0, 4.0, 0.6, 0.0);
    } else {
      sim.attach_circle(body, 0.0, 0.0, 0.25, 4.0, 0.6, 0.0);
    }
    state.vsBullets[i] = body;
  }
}

function destroyVerticalBody(sim: SimWorld, state: SceneState) {
  // VerticalStack::DestroyBody :265-280 — first live body per column
  for (let j = 0; j < state.vsColumns; ++j) {
    for (let i = 0; i < state.vsRows; ++i) {
      const n = j * state.vsRows + i;
      const id = state.vsBodies[n];
      if (id != null && sim.is_body_alive(id)) {
        sim.destroy_body(id);
        state.vsBodies[n] = null;
        break;
      }
    }
  }
}

function buildVerticalStackGround(sim: SimWorld) {
  // VerticalStack ground :168-179
  const ground = sim.add_body(0.0, 0.0, 0.0, 0);
  sim.attach_segment(ground, 10.0, 0.0, 10.0, 20.0);
  sim.attach_segment(ground, -30.0, 0.0, 30.0, 0.0);
}

function buildCircleStack(sim: SimWorld, state: SceneState) {
  // CircleStack :428-471
  state.circleHits = [];
  let shapeIndex = 0;
  const ground = sim.add_body(0.0, 0.0, 0.0, 0);
  // ground shape userData = 0
  sim.attach_segment(ground, -10.0, 0.0, 10.0, 0.0);
  // C sets userData on segment; we only need hit indices on the circles.
  shapeIndex += 1;

  sim.set_gravity(0.0, -20.0);
  sim.set_contact_tuning(0.25 * 360.0, 10.0, 3.0); // :443

  let y = 0.75;
  for (let i = 0; i < 4; ++i) {
    const body = sim.add_body(0.0, y, 0.0, 2);
    const density = 1.0 + 4.0 * i;
    sim.attach_circle_hit(body, 0.0, 0.0, 0.5, density, 0.0, 0.8, shapeIndex);
    shapeIndex += 1;
    y += 1.25;
  }
}

function buildCapsuleStack(sim: SimWorld) {
  // CapsuleStack :527-560
  const ground = sim.add_body(0.0, -1.0, 0.0, 0);
  sim.attach_box(ground, 10.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0);

  const a = 0.25;
  // capsule = { {-4a,0}, {4a,0}, a } → hl = 4a
  let y = 2.0 * a;
  for (let i = 0; i < 20; ++i) {
    sim.add_capsule(0.0, y, 4.0 * a, a, 1.0, 0.0);
    y += 3.0 * a;
  }
}

function createCliffBodies(sim: SimWorld, state: SceneState, existing: (number | null)[]) {
  // Cliff::CreateBodies :612-688
  for (const id of existing) {
    if (id != null && sim.is_body_alive(id)) sim.destroy_body(id);
  }
  const ids: (number | null)[] = new Array(9).fill(null);
  const sign = state.cliffFlip ? -1.0 : 1.0;

  {
    const friction = 0.01;
    const vx = 2.0 * sign;
    const offset = state.cliffFlip ? -4.0 : 0.0;

    let b = sim.add_body(-9.0 + offset, 4.25, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_capsule(b, -0.25, 0.0, 0.25, 0.0, 0.25, 1.0, friction, 0.0);
    ids[0] = b;

    b = sim.add_body(2.0 + offset, 4.75, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_capsule(b, -0.25, 0.0, 0.25, 0.0, 0.25, 1.0, friction, 0.0);
    ids[1] = b;

    b = sim.add_body(13.0 + offset, 4.75, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_capsule(b, -0.25, 0.0, 0.25, 0.0, 0.25, 1.0, friction, 0.0);
    ids[2] = b;
  }

  {
    const friction = 0.01;
    const vx = 2.5 * sign;

    let b = sim.add_body(-11.0, 4.5, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, friction, 0.0);
    ids[3] = b;

    b = sim.add_body(0.0, 5.0, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, friction, 0.0);
    ids[4] = b;

    b = sim.add_body(11.0, 5.0, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_box(b, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0, friction, 0.0);
    ids[5] = b;
  }

  {
    const friction = 0.2;
    const vx = 1.5 * sign;
    const offset = state.cliffFlip ? 4.0 : 0.0;

    let b = sim.add_body(-13.0 + offset, 4.5, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_circle(b, 0.0, 0.0, 0.5, 1.0, friction, 0.0);
    ids[6] = b;

    b = sim.add_body(-2.0 + offset, 5.0, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_circle(b, 0.0, 0.0, 0.5, 1.0, friction, 0.0);
    ids[7] = b;

    b = sim.add_body(9.0 + offset, 5.0, 0.0, 2);
    sim.set_linear_velocity(b, vx, 0.0);
    sim.attach_circle(b, 0.0, 0.0, 0.5, 1.0, friction, 0.0);
    ids[8] = b;
  }

  return ids;
}

function buildCliff(sim: SimWorld, state: SceneState) {
  // Cliff ground :583-600
  const ground = sim.add_body(0.0, 0.0, 0.0, 0);
  sim.attach_box(ground, 100.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.6, 0.0);
  sim.attach_segment(ground, -14.0, 4.0, -8.0, 4.0);
  sim.attach_box(ground, 3.0, 0.5, 0.0, 4.0, 0.0, 0.0, 0.6, 0.0);
  sim.attach_capsule(ground, 8.5, 4.0, 13.5, 4.0, 0.5, 0.0, 0.6, 0.0);

  state.vsBodies = createCliffBodies(sim, state, []);
}

function buildArch(sim: SimWorld) {
  // Arch :725-800
  const ps1 = [
    [16.0, 0.0],
    [14.93803712795643, 5.133601056842984],
    [13.79871746027416, 10.24928069555078],
    [12.56252963284711, 15.34107019122473],
    [11.20040987372525, 20.39856541571217],
    [9.66521217819836, 25.40369899225096],
    [7.87179930638133, 30.3179337000085],
    [5.635199558196225, 35.03820717801641],
    [2.405937953536585, 39.09554102558315],
  ];
  const ps2 = [
    [24.0, 0.0],
    [22.33619528222415, 6.02299846205841],
    [20.54936888969905, 12.00964361211476],
    [18.60854610798073, 17.9470321677465],
    [16.46769273811807, 23.81367936585418],
    [14.05325025774858, 29.57079353071012],
    [11.23551045834022, 35.13775818285372],
    [7.752568160730571, 40.30450679009583],
    [3.016931552701656, 44.28891593799322],
  ];
  const scale = 0.25;
  for (let i = 0; i < 9; ++i) {
    ps1[i]![0]! *= scale;
    ps1[i]![1]! *= scale;
    ps2[i]![0]! *= scale;
    ps2[i]![1]! *= scale;
  }

  const friction = 0.6;
  const ground = sim.add_body(0.0, 0.0, 0.0, 0);
  sim.attach_segment(ground, -100.0, 0.0, 100.0, 0.0);

  for (let i = 0; i < 8; ++i) {
    const body = sim.add_body(0.0, 0.0, 0.0, 2);
    const pts = [
      ps1[i]![0]!,
      ps1[i]![1]!,
      ps2[i]![0]!,
      ps2[i]![1]!,
      ps2[i + 1]![0]!,
      ps2[i + 1]![1]!,
      ps1[i + 1]![0]!,
      ps1[i + 1]![1]!,
    ];
    sim.attach_polygon(body, pts, 0.0, 1.0, friction, 0.0);
  }

  for (let i = 0; i < 8; ++i) {
    const body = sim.add_body(0.0, 0.0, 0.0, 2);
    const pts = [
      -ps2[i]![0]!,
      ps2[i]![1]!,
      -ps1[i]![0]!,
      ps1[i]![1]!,
      -ps1[i + 1]![0]!,
      ps1[i + 1]![1]!,
      -ps2[i + 1]![0]!,
      ps2[i + 1]![1]!,
    ];
    sim.attach_polygon(body, pts, 0.0, 1.0, friction, 0.0);
  }

  {
    const body = sim.add_body(0.0, 0.0, 0.0, 2);
    const pts = [
      ps1[8]![0]!,
      ps1[8]![1]!,
      ps2[8]![0]!,
      ps2[8]![1]!,
      -ps2[8]![0]!,
      ps2[8]![1]!,
      -ps1[8]![0]!,
      ps1[8]![1]!,
    ];
    sim.attach_polygon(body, pts, 0.0, 1.0, friction, 0.0);
  }

  for (let i = 0; i < 4; ++i) {
    const y = 0.5 + ps2[8]![1]! + 1.0 * i;
    const body = sim.add_body(0.0, y, 0.0, 2);
    sim.attach_box(body, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0, friction, 0.0);
  }
}

function buildDoubleDomino(sim: SimWorld) {
  // DoubleDomino :823-853
  const ground = sim.add_body(0.0, -1.0, 0.0, 0);
  sim.attach_box(ground, 100.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0);

  const count = 15;
  let x = -0.5 * count;
  for (let i = 0; i < count; ++i) {
    const body = sim.add_body(x, 0.5, 0.0, 2);
    sim.attach_box(body, 0.125, 0.5, 0.0, 0.0, 0.0, 1.0, 0.6, 0.0);
    if (i === 0) {
      sim.apply_linear_impulse(body, 0.2, 0.0, x, 1.0, true);
    }
    x += 1.0;
  }
}

function buildConfined(sim: SimWorld) {
  // Confined :876-919
  const ground = sim.add_body(0.0, 0.0, 0.0, 0);
  sim.attach_capsule(ground, -10.5, 0.0, 10.5, 0.0, 0.5, 0.0, 0.6, 0.0);
  sim.attach_capsule(ground, -10.5, 0.0, -10.5, 20.5, 0.5, 0.0, 0.6, 0.0);
  sim.attach_capsule(ground, 10.5, 0.0, 10.5, 20.5, 0.5, 0.0, 0.6, 0.0);
  sim.attach_capsule(ground, -10.5, 20.5, 10.5, 20.5, 0.5, 0.0, 0.6, 0.0);

  const gridCount = 25;
  const maxCount = gridCount * gridCount;
  let count = 0;
  let column = 0;
  while (count < maxCount) {
    let row = 0;
    for (let i = 0; i < gridCount; ++i) {
      const x = -8.75 + column * 18.0 / gridCount;
      const y = 1.5 + row * 18.0 / gridCount;
      const body = sim.add_body(x, y, 0.0, 2);
      sim.set_gravity_scale(body, 0.0);
      sim.attach_circle(body, 0.0, 0.0, 0.5, 1.0, 0.6, 0.0);
      count += 1;
      row += 1;
    }
    column += 1;
  }
}

function buildCardHouse(sim: SimWorld) {
  // CardHouse :949-1002
  const ground = sim.add_body(0.0, -2.0, 0.0, 0);
  sim.attach_box(ground, 40.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.7, 0.0);

  const cardHeight = 0.2;
  const cardThickness = 0.001;
  const angle0 = (25.0 * Math.PI) / 180.0;
  const angle1 = (-25.0 * Math.PI) / 180.0;
  const angle2 = 0.5 * Math.PI;

  let Nb = 5;
  let z0 = 0.0;
  let y = cardHeight - 0.02;
  while (Nb) {
    let z = z0;
    for (let i = 0; i < Nb; i++) {
      if (i !== Nb - 1) {
        const body = sim.add_body(z + 0.25, y + cardHeight - 0.015, angle2, 2);
        sim.attach_box(body, cardThickness, cardHeight, 0.0, 0.0, 0.0, 1.0, 0.7, 0.0);
      }

      {
        const body = sim.add_body(z, y, angle1, 2);
        sim.attach_box(body, cardThickness, cardHeight, 0.0, 0.0, 0.0, 1.0, 0.7, 0.0);
      }

      z += 0.175;

      {
        const body = sim.add_body(z, y, angle0, 2);
        sim.attach_box(body, cardThickness, cardHeight, 0.0, 0.0, 0.0, 1.0, 0.7, 0.0);
      }

      z += 0.175;
    }
    y += cardHeight * 2.0 - 0.03;
    z0 += 0.175;
    Nb--;
  }
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "C sample_stacking.cpp roster — Single Box through Card House. " +
      "Pause/step/restart, Hertz/sub-steps, mouse grab, and engine debug draw.",
    "Drag to grab · P pause · O step · R restart",
    { category: "Stacking", samplesShell: true }
  );

  let mode: StackingScene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as StackingScene)
      : "single-box";

  const camera = cameraFor(mode);
  const transport = createSampleTransport();
  let sim: SimWorld = null as unknown as SimWorld;
  let state = freshState();
  /** Cliff body ids stored in state.vsBodies when mode === cliff. */
  let cliffBodies: (number | null)[] = [];

  const sceneControls = document.createElement("div");
  sceneControls.className = "stacking-scene-controls";

  function applyCamera() {
    const c = cameraFor(mode);
    camera.centerX = c.centerX;
    camera.centerY = c.centerY;
    camera.zoom = c.zoom;
  }

  function rebuildSceneControls() {
    sceneControls.innerHTML = "";
    if (mode === "vertical-stack") {
      sceneControls.appendChild(
        createDropdown(
          "Shape",
          [
            { value: "box", text: "Box" },
            { value: "circle", text: "Circle" },
          ],
          state.vsShape,
          (v) => {
            state.vsShape = v as "circle" | "box";
            destroyVerticalBullets(sim, state);
            createVerticalStacks(sim, state);
          },
        ),
      );
      sceneControls.appendChild(
        createSlider("Rows", 1, VS_MAX_ROWS, state.vsRows, 1, (v) => {
          state.vsRows = v;
          destroyVerticalBullets(sim, state);
          createVerticalStacks(sim, state);
        }),
      );
      sceneControls.appendChild(
        createSlider("Columns", 1, VS_MAX_COLUMNS, state.vsColumns, 1, (v) => {
          state.vsColumns = v;
          destroyVerticalBullets(sim, state);
          createVerticalStacks(sim, state);
        }),
      );
      sceneControls.appendChild(
        createSlider("Bullets", 1, VS_MAX_BULLETS, state.vsBulletCount, 1, (v) => {
          state.vsBulletCount = v;
        }),
      );
      sceneControls.appendChild(
        createDropdown(
          "Bullet Shape",
          [
            { value: "circle", text: "Circle" },
            { value: "box", text: "Box" },
          ],
          state.vsBulletType,
          (v) => {
            state.vsBulletType = v as "circle" | "box";
          },
        ),
      );
      sceneControls.appendChild(
        createButton("Fire Bullets", () => {
          destroyVerticalBullets(sim, state);
          fireVerticalBullets(sim, state);
        }),
      );
      sceneControls.appendChild(
        createButton("Destroy Body", () => destroyVerticalBody(sim, state)),
      );
      sceneControls.appendChild(
        createButton("Reset Stack", () => {
          destroyVerticalBullets(sim, state);
          createVerticalStacks(sim, state);
        }),
      );
    } else if (mode === "cliff") {
      sceneControls.appendChild(
        createButton("Flip", () => {
          state.cliffFlip = !state.cliffFlip;
          cliffBodies = createCliffBodies(sim, state, cliffBodies);
        }),
      );
    }
  }

  function buildScene() {
    freeSim(sim);
    // Default gravity -10 matches C samples; Circle Stack overrides.
    sim = new wasm.SimWorld(-10.0);
    state = freshState();
    cliffBodies = [];
    applyCamera();

    switch (mode) {
      case "single-box":
        buildSingleBox(sim, state);
        break;
      case "tilted-stack":
        buildTiltedStack(sim);
        break;
      case "vertical-stack":
        buildVerticalStackGround(sim);
        createVerticalStacks(sim, state);
        break;
      case "circle-stack":
        buildCircleStack(sim, state);
        break;
      case "capsule-stack":
        buildCapsuleStack(sim);
        break;
      case "cliff":
        buildCliff(sim, state);
        cliffBodies = state.vsBodies;
        break;
      case "arch":
        buildArch(sim);
        break;
      case "double-domino":
        buildDoubleDomino(sim);
        break;
      case "confined":
        buildConfined(sim);
        break;
      case "card-house":
        buildCardHouse(sim);
        break;
    }
    rebuildSceneControls();
  }

  function setMode(next: StackingScene, pushHash: boolean) {
    mode = next;
    buildScene();
    if (pushHash) {
      const entry = findByRouteName("stacking", SCENE_LABELS[mode]);
      if (entry) {
        const href = entryHref(entry);
        if (window.location.hash !== href) {
          history.replaceState(null, "", href);
        }
      }
    }
  }

  buildScene();

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

  const onKey = (e: KeyboardEvent) => {
    if (e.code === "KeyB" && mode === "vertical-stack" && !e.repeat) {
      fireVerticalBullets(sim, state);
    }
  };
  window.addEventListener("keydown", onKey);

  controls.appendChild(
    createInfoBox(
      "Exact C <code>sample_stacking.cpp</code> values. Scene selector matches the " +
        "registry; Vertical Stack supports Fire/Destroy/Reset (B fires). " +
        "Circle Stack accumulates hit-event shape user-data pairs.",
    ),
  );
  controls.appendChild(
    createDropdown(
      "Sample",
      SCENES.map((s) => ({ value: s, text: SCENE_LABELS[s] })),
      mode,
      (v) => setMode(v as StackingScene, true),
    ),
  );
  const chrome = mountSampleChrome({
    controls,
    canvas,
    camera,
    route: "stacking",
    category: "Stacking",
    sampleName: SCENE_LABELS[mode],
    transport,
    onRestart: () => buildScene(),
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
    sim.step(dt, transport.subSteps);

    if (mode === "circle-stack" && dt > 0) {
      const pairs = sim.hit_event_user_pairs();
      for (let i = 0; i + 1 < pairs.length; i += 2) {
        state.circleHits.push({ a: pairs[i]!, b: pairs[i + 1]! });
      }
    }

    paintSampleDraw(canvas, camera, sim);

    const rows: { label: string; value: string }[] = [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Awake", value: String(sim.awake_body_count()) },
      { label: "Hz", value: String(transport.hertz) },
      { label: "Paused", value: transport.paused ? "yes" : "no" },
      { label: "Grab", value: sim.mouse_active() ? "yes" : "no" },
    ];

    if (mode === "single-box" && state.singleBoxId >= 0 && sim.is_body_alive(state.singleBoxId)) {
      const pos = sim.positions();
      const i = state.singleBoxId * 3;
      rows.push({
        label: "(x, y)",
        value: `(${pos[i]!.toPrecision(2)}, ${pos[i + 1]!.toPrecision(2)})`,
      });
    }
    if (mode === "circle-stack") {
      const n = Math.min(state.circleHits.length, 12);
      for (let i = state.circleHits.length - n; i < state.circleHits.length; i++) {
        const ev = state.circleHits[i]!;
        rows.push({ label: "hit", value: `${ev.a}, ${ev.b}` });
      }
    }

    updateReadout(readout, rows);
  }, readout);

  return () => {
    stop();
    unbindKeys();
    chrome.dispose();
    disposeTransport(transport);
    window.removeEventListener("keydown", onKey);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    freeSim(sim);
  };
}
