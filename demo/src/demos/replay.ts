// Replay — RegisterReplay("Replay", "Viewer") port of sample_replay.cpp.
// Route-only single-scene host (no SCENES / PAGES entry), matching box3d.
//
// Shipped: load a self-recorded Falling Hinges session (or a user .b2rec via
// file picker), transport (play/pause/step/seek/loop/speed), debug-draw of the
// replayed world, divergence + keyframe readout.
// Disclosed gaps vs C ReplayViewer: no ImGui inspector outliner / selection
// detail, no frame-query overlay / search index, no keyframe-policy Load popup
// (defaults used), no worker-count slider.

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
import { getWasm, type SimPlayer, type SimWorld } from "../wasm.ts";
import { paintDebugDraw } from "./debug-draw.ts";
import { demoPage, fitCanvas, freeSim, runSimLoop } from "./sim-common.ts";
import {
  makeCamera,
  viewBounds,
  type SampleCamera,
} from "./sample-shell.ts";

/** Route-only page — empty SCENES (no multi-scene assertRouteScenes). */
export const SCENES: readonly string[] = [];

const BODY_DYNAMIC = 2;
const PI = Math.PI;
const SPEEDS = [
  { label: "0.25x", value: 0.25 },
  { label: "0.5x", value: 0.5 },
  { label: "1x", value: 1 },
  { label: "2x", value: 2 },
  { label: "4x", value: 4 },
];

/** C camera defaults for ReplayViewer (:135-136). */
const CAMERA = { cx: 0.0, cy: 7.5, zoom: 10.0 };

/**
 * Record a Falling Hinges session (same scene as sample_determinism.cpp) so the
 * Viewer always has a C-faithful recording without shipping a binary asset.
 * Mirrors CreateFallingHinges + Step until sleep (shared/determinism.c).
 */
function recordFallingHinges(wasm: ReturnType<typeof getWasm>): Uint8Array {
  const sim: SimWorld = new wasm.SimWorld(-10.0);
  if (!sim.start_recording()) {
    freeSim(sim);
    throw new Error("recording failed to start");
  }

  sim.add_static_box(0.0, -1.0, 40.0, 1.0);

  const columnCount = 4;
  const rowCount = 20;
  const bodyIds: number[] = [];
  const h = 0.25;
  const offset = 0.4 * h;
  const dx = 10.0 * h;
  const xBase = -0.5 * dx * (columnCount - 1.0);

  for (let j = 0; j < columnCount; ++j) {
    const x = xBase + j * dx;
    let prevBodyId = -1;
    for (let i = 0; i < rowCount; ++i) {
      const angle = (i & 1) === 0 ? -0.1 : 0.1;
      const bodyId = sim.add_body(x + offset * i, h + 2.0 * h * i, angle, BODY_DYNAMIC);
      sim.attach_box(bodyId, h, h, 0, 0, 0, 1.0, 0.6, 0);
      if ((i & 1) === 0) {
        prevBodyId = bodyId;
      } else {
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

  // Step until sleep (or a hard cap). Falling Hinges issues queries each step
  // in C for the Replay viewer; we keep cast_ray_closest as the bound equivalent.
  let hash = 0;
  let stepCount = 0;
  const stepLimit = 500;
  for (let n = 0; n < stepLimit; n++) {
    sim.step(1 / 60, 4);
    sim.cast_ray_closest(0.0, 12.0, 0.0, -14.0);
    if (hash === 0 && sim.body_move_events().length === 0 && sim.awake_body_count() === 0) {
      hash = sim.hash_body_transforms(new Uint32Array(bodyIds));
    }
    stepCount += 1;
    if (hash !== 0) break;
  }
  void stepCount;

  const bytes = sim.stop_recording();
  freeSim(sim);
  return bytes;
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Replay",
    "C <code>sample_replay.cpp</code> Replay Viewer — play a .b2rec recording with " +
      "timeline scrubbing. Default session is a recorded Falling Hinges soak.",
    "Scrub timeline · Play/Pause · Loop · load a local .b2rec",
  );

  const camera: SampleCamera = makeCamera(CAMERA.cx, CAMERA.cy, CAMERA.zoom);

  let player: SimPlayer | null = null;
  let status = "recording Falling Hinges…";
  let playing = false; // C opens paused (:142)
  let loop = false;
  let speed = 1.0;
  let frameAccumulator = 0;
  let scrubbing = false;
  let sourceLabel = "Falling Hinges (generated)";

  function closePlayer() {
    player?.free?.();
    // SimPlayer may not expose free via wasm-bindgen Drop; drop by nulling.
    player = null;
  }

  function openBytes(data: Uint8Array, label: string) {
    closePlayer();
    sourceLabel = label;
    const opened = wasm.SimPlayer.open(data);
    if (!opened) {
      status = "failed to open recording";
      player = null;
      return;
    }
    player = opened;
    status = "loaded";
    playing = false;
    frameAccumulator = 0;
    timeline.querySelector("input")!.max = String(player.frame_count());
    timeline.querySelector("input")!.value = "0";
  }

  // --- Transport ---
  const timeline = createSlider("Frame", 0, 1, 0, 1, (v) => {
    if (!player) return;
    scrubbing = true;
    playing = false;
    player.seek_frame(v);
    frameAccumulator = 0;
    scrubbing = false;
  });

  controls.appendChild(
    createInfoBox(
      "Partial vs C: transport + scrub + debug draw + divergence are live. " +
        "Not ported: inspector outline/detail, query overlay/search index, " +
        "keyframe-budget Load popup, worker-count slider.",
    ),
  );
  controls.appendChild(createSeparator());

  const transportRow = document.createElement("div");
  transportRow.style.display = "flex";
  transportRow.style.flexWrap = "wrap";
  transportRow.style.gap = "6px";
  transportRow.appendChild(
    createButton("|<", () => {
      player?.seek_frame(0);
      frameAccumulator = 0;
    }),
  );
  transportRow.appendChild(
    createButton("<", () => {
      if (!player) return;
      player.seek_frame(player.frame() - 1);
      playing = false;
      frameAccumulator = 0;
    }),
  );
  const playBtn = createButton("Play", () => {
    playing = !playing;
    playBtn.textContent = playing ? "Pause" : "Play";
  });
  transportRow.appendChild(playBtn);
  transportRow.appendChild(
    createButton(">", () => {
      if (!player) return;
      player.seek_frame(player.frame() + 1);
      playing = false;
      frameAccumulator = 0;
    }),
  );
  transportRow.appendChild(
    createButton(">|", () => {
      if (!player) return;
      player.seek_frame(player.frame_count());
      frameAccumulator = 0;
    }),
  );
  controls.appendChild(transportRow);

  controls.appendChild(
    createDropdown(
      "Speed",
      SPEEDS.map((s) => ({ value: String(s.value), text: s.label })),
      "1",
      (v) => {
        speed = Number(v);
      },
    ),
  );
  controls.appendChild(
    createCheckbox("Loop", false, (v) => {
      loop = v;
    }),
  );
  controls.appendChild(timeline);
  controls.appendChild(createSeparator());

  const fileInput = document.createElement("input");
  fileInput.type = "file";
  fileInput.accept = ".b2rec,application/octet-stream";
  fileInput.style.display = "none";
  fileInput.addEventListener("change", async () => {
    const file = fileInput.files?.[0];
    if (!file) return;
    const buf = new Uint8Array(await file.arrayBuffer());
    openBytes(buf, file.name);
  });
  controls.appendChild(fileInput);
  controls.appendChild(
    createButton("Open .b2rec…", () => fileInput.click()),
  );
  controls.appendChild(
    createButton("Reload Falling Hinges", () => {
      status = "recording Falling Hinges…";
      const bytes = recordFallingHinges(wasm);
      openBytes(bytes, "Falling Hinges (generated)");
    }),
  );
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  // Generate default recording synchronously on first paint path.
  try {
    const bytes = recordFallingHinges(wasm);
    openBytes(bytes, "Falling Hinges (generated)");
  } catch (e) {
    status = e instanceof Error ? e.message : "record failed";
  }

  const stop = runSimLoop(() => {
    fitCanvas(canvas);
    if (!player) {
      updateReadout(readout, [
        { label: "Status", value: status },
        { label: "Source", value: sourceLabel },
      ]);
      return;
    }

    if (playing && !scrubbing) {
      frameAccumulator += speed;
      while (frameAccumulator >= 1.0) {
        frameAccumulator -= 1.0;
        if (player.is_at_end()) {
          if (loop) {
            player.restart();
          } else {
            frameAccumulator = 0;
            playing = false;
            playBtn.textContent = "Play";
            break;
          }
        }
        if (!player.step_frame()) {
          if (loop) {
            player.restart();
          } else {
            playing = false;
            playBtn.textContent = "Play";
            frameAccumulator = 0;
            break;
          }
        }
      }
      const input = timeline.querySelector("input");
      if (input && !scrubbing) input.value = String(player.frame());
    }

    const b = viewBounds(camera, canvas);
    player.collect_draw(b.lowerX, b.lowerY, b.upperX, b.upperY);
    paintDebugDraw(canvas, camera, {
      polygons: player.draw_polygons(),
      circles: player.draw_circles(),
      capsules: player.draw_capsules(),
      lines: player.draw_lines(),
    });

    const diverge = player.diverge_frame();
    updateReadout(readout, [
      { label: "Status", value: status },
      { label: "Source", value: sourceLabel },
      {
        label: "Frame",
        value: `${player.frame()} / ${player.frame_count()}${player.is_at_end() ? " (end)" : ""}`,
      },
      { label: "Bodies", value: String(player.body_count()) },
      { label: "Contacts", value: String(player.contact_count()) },
      { label: "Bit-identical", value: player.has_diverged() ? "NO" : "yes" },
      {
        label: "Diverged at",
        value: diverge >= 0 ? String(diverge) : "—",
      },
      {
        label: "Keyframe",
        value: `every ${player.keyframe_interval()} · ${player.keyframe_kilobytes().toFixed(0)} KB`,
      },
    ]);
  }, readout);

  return () => {
    stop();
    closePlayer();
  };
}
