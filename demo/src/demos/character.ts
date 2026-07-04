// Character — a capsule mover driven by the ported character-controller
// queries: b2World_CollideMover -> b2SolvePlanes -> b2World_CastMover ->
// b2ClipVector. The mover is not a rigid body; it is pure query-driven.

import { createButton, createInfoBox, createReadout, createSeparator, updateReadout } from "../controls.ts";
import { getWasm, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 42;
const ORIGIN_Y = 60;
const SPAWN: [number, number] = [-8.0, 2.0];

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Character",
    "A capsule mover walking stairs and slopes without being a rigid body: collision planes " +
      "are gathered, solved, and swept every frame by the ported mover queries.",
    "Move with A/D or arrow keys, jump with Space or W",
  );

  let sim: SimWorld = null as unknown as SimWorld;
  let shapes: SimShape[] = [];
  let grounded = false;
  let planeCount = 0;
  let moverX = SPAWN[0];
  let moverY = SPAWN[1];

  const keys = new Set<string>();

  function buildScene() {
    freeSim(sim);
    sim = new wasm.SimWorld(-10.0);
    shapes = [];

    const addBox = (x: number, y: number, hx: number, hy: number, color = COLORS.ground) => {
      sim.add_static_box(x, y, hx, hy);
      shapes.push({ kind: "box", hx, hy, color });
    };

    // Floor with a pit, stairs up, a slope of small steps, and platforms.
    addBox(-7.5, -0.25, 3.5, 0.25);
    addBox(1.0, -0.25, 5.0, 0.25);
    // Pit walls between the floor slabs (x in [-4, -2]).
    addBox(-3.0, -1.75, 1.0, 0.25);
    addBox(-4.25, -1.0, 0.25, 1.0);
    addBox(-1.75, -1.0, 0.25, 1.0);

    // Stairs: shallow 0.15 m risers the capsule's rounded bottom can walk
    // up (a riser over ~half the capsule radius reads as a wall).
    const riser = 0.15;
    const stepCount = 8;
    for (let i = 0; i < stepCount; i++) {
      const top = riser * (i + 1);
      addBox(2.4 + 0.6 * i, top / 2, 0.3, top / 2, COLORS.box);
    }

    // Upper deck and floating platforms.
    addBox(8.4, riser * stepCount - 0.125, 1.8, 0.125);
    addBox(-6.5, 2.0, 1.2, 0.15, COLORS.box);
    addBox(-3.0, 3.2, 1.2, 0.15, COLORS.box);

    sim.mover_spawn(SPAWN[0], SPAWN[1]);
    grounded = false;
    planeCount = 0;
  }

  buildScene();

  // Track physical key codes (layout independent). Space would otherwise
  // re-activate a focused button (e.g. Reset) and arrows would scroll, so
  // both are suppressed while this page is up.
  const onKeyDown = (e: KeyboardEvent) => {
    keys.add(e.code);
    if (["Space", "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(e.code)) {
      e.preventDefault();
    }
  };
  const onKeyUp = (e: KeyboardEvent) => keys.delete(e.code);
  const onBlur = () => keys.clear();
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  window.addEventListener("blur", onBlur);

  controls.appendChild(
    createInfoBox(
      "Each frame: <b>CollideMover</b> gathers contact planes around the capsule, " +
        "<b>SolvePlanes</b> finds a translation satisfying them, <b>CastMover</b> sweeps the " +
        "move so nothing is skipped, and <b>ClipVector</b> removes velocity into the planes.",
    ),
  );
  controls.appendChild(
    createButton("Reset", () => {
      buildScene();
      // Drop focus so Space jumps instead of re-clicking this button.
      (document.activeElement as HTMLElement | null)?.blur();
    }),
  );
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    sim.step(1 / 60, 4);

    let moveX = 0;
    if (keys.has("KeyA") || keys.has("ArrowLeft")) moveX -= 1;
    if (keys.has("KeyD") || keys.has("ArrowRight")) moveX += 1;
    const jump = keys.has("Space") || keys.has("KeyW") || keys.has("ArrowUp");

    const state = sim.mover_update(1 / 60, moveX, jump);
    moverX = state[0];
    moverY = state[1];
    grounded = state[2] > 0.5;
    planeCount = state[3];

    // Fell into the void: respawn.
    if (moverY < -6) sim.mover_spawn(SPAWN[0], SPAWN[1]);

    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, sim.positions());

    // Draw the capsule mover (radius 0.3, half-height 0.25).
    const px = canvas.width / 2 + moverX * SCALE;
    const py = canvas.height - ORIGIN_Y - moverY * SCALE;
    const r = 0.3 * SCALE;
    const h = 0.25 * SCALE;
    ctx.beginPath();
    ctx.arc(px, py - h, r, Math.PI, 0);
    ctx.lineTo(px + r, py + h);
    ctx.arc(px, py + h, r, 0, Math.PI);
    ctx.closePath();
    ctx.fillStyle = grounded ? "#15803d2a" : "#dc26262a";
    ctx.strokeStyle = grounded ? COLORS.ball : COLORS.heavy;
    ctx.lineWidth = 2.5;
    ctx.fill();
    ctx.stroke();

    updateReadout(readout, [
      { label: "Position", value: `${moverX.toFixed(2)}, ${moverY.toFixed(2)}` },
      { label: "Grounded", value: grounded ? "yes" : "no" },
      { label: "Contact planes", value: String(planeCount) },
    ]);
  }, readout);

  return () => {
    stop();
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    window.removeEventListener("blur", onBlur);
    freeSim(sim);
  };
}
