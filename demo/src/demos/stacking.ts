// Stacking — invented capability demo that exercises the Phase 1 harness
// (pause/step/restart, hertz/sub-steps, C camera, mouse grab, engine debug draw).
// Not a 1:1 RegisterSample port; Phase 2 will replace scenes with C samples.

import {
  createButton,
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

/**
 * Scene keys this page will host once Phase 2 ports Stacking samples.
 * Validated against the registry; empty until the first live/partial row lands.
 */
export const SCENES = [] as const;
assertRouteScenes("stacking", SCENES);

const MAX_BODIES = 160;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "Harness preview: pause/step/restart, Hertz/sub-steps, mouse grab (C spring), " +
      "and engine-driven debug draw. Phase 2 will replace this with 1:1 Stacking samples.",
    "Drag to grab · P pause · O step · R restart",
  );

  // C Single Box camera is tighter; use a mid stacking view for the pyramid.
  const camera: SampleCamera = makeCamera(0, 5, 6);
  const transport = createSampleTransport();
  let sim: SimWorld = null as unknown as SimWorld;
  let rows = 9;
  let useEngineDraw = true;

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    sim.add_static_box(0.0, -0.5, 11.0, 0.5);
    const h = 0.4;
    for (let row = 0; row < rows; row++) {
      const count = rows - row;
      const y = h + row * 2 * h;
      for (let i = 0; i < count; i++) {
        const x = (i - (count - 1) / 2) * 2.05 * h;
        sim.add_box(x, y, h, h, 1.0);
      }
    }
  }

  function dropBall(x: number) {
    if (sim.body_count() > MAX_BODIES) return;
    sim.add_circle(Math.max(-10, Math.min(10, x)), 9.0, 0.5, 4.0);
  }

  buildScene();

  // Mouse grab (left drag) — C Sample::Mouse* spring values via wasm.
  let grabbing = false;
  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    fitCanvas(canvas);
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    if (e.shiftKey) {
      dropBall(w.x);
      return;
    }
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
    createInfoBox(
      "Shared Phase 1 harness. <em>Drag</em> grabs with the C motor-joint spring " +
        "(hertz 7.5). <em>Shift+click</em> drops a heavy ball. Engine draw uses " +
        "<code>b2World_Draw</code> → canvas adapter.",
    ),
  );
  transport.mountControls(controls, () => buildScene());
  controls.appendChild(createSeparator());
  controls.appendChild(
    createSlider("Pyramid rows", 3, 14, rows, 1, (v) => {
      rows = v;
      buildScene();
    }),
  );
  controls.appendChild(createButton("Drop heavy ball", () => dropBall(0.3)));
  const drawToggle = createButton("Engine draw: ON", () => {
    useEngineDraw = !useEngineDraw;
    drawToggle.textContent = useEngineDraw ? "Engine draw: ON" : "Engine draw: OFF";
    drawToggle.classList.toggle("active", useEngineDraw);
  }, true);
  controls.appendChild(drawToggle);

  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const unbindKeys = transport.bindKeys();

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    const dt = transport.consumeStepDt();
    sim.step(dt, transport.subSteps);

    if (useEngineDraw) {
      const b = viewBounds(camera, canvas);
      sim.collect_draw(b.lowerX, b.lowerY, b.upperX, b.upperY);
      paintDebugDraw(canvas, camera, {
        polygons: sim.draw_polygons(),
        circles: sim.draw_circles(),
        capsules: sim.draw_capsules(),
        lines: sim.draw_lines(),
      });
    } else {
      // Fallback: clear only (positions path kept for invented demos elsewhere).
      const ctx = canvas.getContext("2d")!;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    }

    const awake = sim.awake_body_count();
    updateReadout(readout, [
      { label: "Bodies", value: String(sim.body_count()) },
      { label: "Contacts", value: String(sim.contact_count()) },
      { label: "Awake", value: String(awake) },
      { label: "Hz", value: String(transport.hertz) },
      { label: "Paused", value: transport.paused ? "yes" : "no" },
      { label: "Grab", value: sim.mouse_active() ? "yes" : "no" },
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
