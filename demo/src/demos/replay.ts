// Replay — record a session with the engine's op-stream recorder, then play
// it back through the ported b2RecPlayer: timeline scrubbing restores the
// nearest keyframe and re-steps only the gap, landing bit-identical to the
// original run (the diverged flag would trip otherwise).

import { createButton, createInfoBox, createReadout, createSeparator, createSlider, updateReadout } from "../controls.ts";
import { getWasm, type SimPlayer, type SimWorld } from "../wasm.ts";
import { COLORS, demoPage, drawSimBodies, fitCanvas, freeSim, runSimLoop, type SimShape } from "./sim-common.ts";

const SCALE = 40;
const ORIGIN_Y = 60;
const RECORD_FRAMES = 240;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Replay",
    "A pyramid session recorded once through the op-stream recorder, played back with the " +
      "recording player. Drag the timeline: backward seeks restore a keyframe snapshot and " +
      "re-step the gap, reproducing the original run bit for bit.",
    "Drag the timeline to scrub • click the canvas to toggle play",
  );

  let shapes: SimShape[] = [];
  let player: SimPlayer | null = null;
  let playing = true;
  let scrubbing = false;

  // Record the session once: the scene is built after start_recording so
  // every create lands in the op stream, and a mid-run explosion makes the
  // timeline worth scrubbing.
  function recordSession(): Uint8Array {
    const sim: SimWorld = new wasm.SimWorld(-10.0);
    shapes = [];

    if (!sim.start_recording()) {
      throw new Error("recording session failed to start");
    }

    sim.add_static_box(0.0, -1.0, 12.0, 1.0);
    shapes.push({ kind: "box", hx: 12.0, hy: 1.0, color: COLORS.ground });

    // The scrub-test pyramid: heavy stacking contact churn
    const h = 0.5;
    const pitch = 2.0 * h + 0.05;
    const baseCount = 7;
    for (let row = 0; row < baseCount; row++) {
      const count = baseCount - row;
      const y = h + row * pitch;
      const xStart = -0.5 * (count - 1) * pitch;
      for (let col = 0; col < count; col++) {
        sim.add_box(xStart + col * pitch, y, h, h, 1.0);
        shapes.push({ kind: "box", hx: h, hy: h, color: COLORS.box });
      }
    }

    for (let i = 0; i < RECORD_FRAMES; i++) {
      sim.step(1 / 60, 4);
      if (i === 90) {
        sim.explode(-2.0, 1.0, 4.0, 2.0, 6.0);
      }
      if (i === 170) {
        sim.explode(2.5, 1.0, 4.0, 2.0, 8.0);
      }
    }

    const bytes = sim.stop_recording();
    freeSim(sim);
    return bytes;
  }

  const recording = recordSession();
  player = wasm.SimPlayer.open(recording) ?? null;

  const timeline = createSlider("Frame", 0, RECORD_FRAMES, 0, 1, (v) => {
    if (!player) return;
    scrubbing = true;
    playing = false;
    player.seek_frame(v);
    scrubbing = false;
  });

  canvas.addEventListener("click", () => {
    playing = !playing;
  });

  controls.appendChild(
    createInfoBox(
      "The recording seeds a world snapshot, then logs every API call and step. " +
        "The player re-runs the engine against that log, checking a state hash every " +
        "frame — <b>Bit-identical</b> stays yes even after keyframe-restored seeks.",
    ),
  );
  controls.appendChild(timeline);
  controls.appendChild(
    createButton("Play / Pause", () => {
      playing = !playing;
    }),
  );
  controls.appendChild(
    createButton("Restart", () => {
      player?.seek_frame(0);
      playing = true;
    }),
  );
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  const setSlider = (v: number) => {
    const input = timeline.querySelector("input");
    if (input && !scrubbing) input.value = String(v);
  };

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    if (!player) return;

    if (playing && !scrubbing) {
      if (!player.step_frame()) {
        // Loop the playback from frame 0
        player.seek_frame(0);
      }
      setSlider(player.frame());
    }

    const positions = player.positions();
    const ctx = canvas.getContext("2d")!;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawSimBodies(canvas, SCALE, ORIGIN_Y, shapes, positions);

    updateReadout(readout, [
      { label: "Frame", value: `${player.frame()} / ${player.frame_count()}` },
      { label: "Bodies", value: String(shapes.length) },
      { label: "Contacts", value: String(player.contact_count()) },
      { label: "Bit-identical", value: player.has_diverged() ? "NO" : "yes" },
      { label: "Keyframe every", value: `${player.keyframe_interval()} frames` },
      { label: "Keyframe memory", value: `${player.keyframe_kilobytes().toFixed(0)} KB` },
    ]);
  }, readout);

  return () => {
    stop();
    player?.free();
    player = null;
  };
}
