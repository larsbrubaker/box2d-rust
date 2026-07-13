// Replay — RegisterReplay("Replay", "Viewer") port of sample_replay.cpp.
// Route-only single-scene host (no SCENES / PAGES entry), matching box3d.
//
// Exact (serial wasm): transport, scrub, debug draw, divergence, inspector
// outliner + detail + click-pick, per-frame query index/overlay, keyframe-
// policy Load popup. Workers slider disclosed N/A (single-threaded wasm).

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
import { paintSampleDraw } from "./debug-draw.ts";
import { demoPage, fitCanvas, freeSim, runSimLoop } from "./sim-common.ts";
import {
  bindCameraControls,
  makeCamera,
  screenToWorld,
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

const SEL_NONE = 0;
const SEL_BODY = 1;
const SEL_SHAPE = 2;
const SEL_JOINT = 3;
const SEL_QUERY = 4;

/** C camera defaults for ReplayViewer (:135-136). */
const CAMERA = { cx: 0.0, cy: 7.5, zoom: 10.0 };

/** Persisted like SampleContext replayKeyframeBudgetMB / MinInterval. */
let persistedBudgetMB = 512;
let persistedMinInterval = 16;

type OutlineBody = {
  ord: number;
  label: string;
  shapes: { slot: number; label: string }[];
  joints: { slot: number; label: string }[];
};
type OutlineQuery = { index: number; label: string; hits: number };
type Outline = { bodies: OutlineBody[]; queries: OutlineQuery[] };

/**
 * Record a Falling Hinges session (same scene as sample_determinism.cpp) so the
 * Viewer always has a C-faithful recording without shipping a binary asset.
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

  let hash = 0;
  const stepLimit = 500;
  for (let n = 0; n < stepLimit; n++) {
    sim.step(1 / 60, 4);
    sim.cast_ray_closest(0.0, 12.0, 0.0, -14.0);
    if (hash === 0 && sim.body_move_events().length === 0 && sim.awake_body_count() === 0) {
      hash = sim.hash_body_transforms(new Uint32Array(bodyIds));
    }
    if (hash !== 0) break;
  }

  const bytes = sim.stop_recording();
  freeSim(sim);
  return bytes;
}

function el(tag: string, className?: string, text?: string): HTMLElement {
  const n = document.createElement(tag);
  if (className) n.className = className;
  if (text != null) n.textContent = text;
  return n;
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls, page } = demoPage(
    container,
    "Replay",
    "C <code>sample_replay.cpp</code> Replay Viewer — play a .b2rec recording with " +
      "timeline scrubbing. Default session is a recorded Falling Hinges soak.",
    "Click shape to inspect · Scrub timeline · Play/Pause · Loop · Open .b2rec",
    { category: "Replay", samplesShell: true },
  );

  const camera: SampleCamera = makeCamera(CAMERA.cx, CAMERA.cy, CAMERA.zoom);
  const canvasArea = page.querySelector(".demo-canvas-area") as HTMLElement;

  // --- Inspector (left panel, C DrawInspectorPanel) ---
  const inspector = el("div", "replay-inspector");
  const outlineTitle = el("div", "replay-inspector-head", "Outline");
  const treeEl = el("div", "replay-outline-tree");
  const detailTitle = el("div", "replay-inspector-head", "Detail");
  const detailEl = el("pre", "replay-detail");
  detailEl.textContent = "Click a node, or a shape in the view.";
  inspector.append(outlineTitle, treeEl, el("hr", "replay-inspector-sep"), detailTitle, detailEl);
  canvasArea.appendChild(inspector);

  // --- Load popup (C DrawLoadPopup) ---
  const popup = el("div", "replay-load-popup hidden");
  const popupInner = el("div", "replay-load-popup-inner");
  const popupTitle = el("div", "replay-load-title", "Load Replay");
  const popupFile = el("div", "replay-load-file");
  const popupForm = el("div", "replay-load-form");
  const budgetSlider = createSlider("Memory budget (MB)", 128, 4096, persistedBudgetMB, 64, (v) => {
    popupBudgetMB = v;
  });
  const intervalSlider = createSlider("Min sample interval", 8, 60, persistedMinInterval, 1, (v) => {
    popupMinInterval = v;
  });
  popupForm.append(budgetSlider, intervalSlider);
  const popupError = el("div", "replay-load-error");
  const popupProgress = el("div", "replay-load-progress hidden");
  const progressLabel = el("div", "", "Generating keyframes");
  const progressBar = document.createElement("progress");
  progressBar.max = 1;
  progressBar.value = 0;
  const progressOverlay = el("div", "replay-load-progress-text", "0 / 0");
  popupProgress.append(progressLabel, progressBar, progressOverlay);
  const popupActions = el("div", "replay-load-actions");
  const loadBtn = createButton("Load", () => confirmLoad());
  const cancelBtn = createButton("Cancel", () => hidePopup(true));
  popupActions.append(loadBtn, cancelBtn);
  popupInner.append(
    popupTitle,
    popupFile,
    popupForm,
    popupError,
    popupProgress,
    popupActions,
  );
  popup.appendChild(popupInner);
  canvasArea.appendChild(popup);

  let player: SimPlayer | null = null;
  let status = "recording Falling Hinges…";
  let playing = false;
  let loop = false;
  let speed = 1.0;
  let frameAccumulator = 0;
  let scrubbing = false;
  let sourceLabel = "Falling Hinges (generated)";
  let pendingBytes: Uint8Array | null = null;
  let pendingLabel = "";
  let popupBudgetMB = persistedBudgetMB;
  let popupMinInterval = persistedMinInterval;
  let generating = false;
  let outlineDirty = true;
  let expandedBodies = new Set<number>();
  let queriesExpanded = true;
  let revealSelection = false;

  function closePlayer() {
    player?.free?.();
    player = null;
    expandedBodies.clear();
    outlineDirty = true;
  }

  function hidePopup(cancel: boolean) {
    if (cancel && generating) {
      // Keep player if mid-generate was abandoned — close without commit.
      closePlayer();
      status = "cancelled";
    }
    generating = false;
    popup.classList.add("hidden");
    popupForm.classList.remove("hidden");
    popupActions.classList.remove("hidden");
    popupProgress.classList.add("hidden");
    pendingBytes = null;
  }

  function showLoadPopup(data: Uint8Array, label: string) {
    pendingBytes = data;
    pendingLabel = label;
    sourceLabel = label;
    popupBudgetMB = persistedBudgetMB;
    popupMinInterval = persistedMinInterval;
    budgetSlider.querySelector("input")!.value = String(popupBudgetMB);
    budgetSlider.querySelector(".slider-value")!.textContent = String(popupBudgetMB);
    intervalSlider.querySelector("input")!.value = String(popupMinInterval);
    intervalSlider.querySelector(".slider-value")!.textContent = String(popupMinInterval);
    popupFile.innerHTML = `<span class="muted">File:</span> ${label}`;
    popupError.textContent = "";
    popupForm.classList.remove("hidden");
    popupActions.classList.remove("hidden");
    popupProgress.classList.add("hidden");
    generating = false;
    popup.classList.remove("hidden");
  }

  function confirmLoad() {
    if (!pendingBytes) return;
    closePlayer();
    persistedBudgetMB = popupBudgetMB;
    persistedMinInterval = popupMinInterval;
    const opened = wasm.SimPlayer.open(pendingBytes);
    if (!opened) {
      status = "failed to open recording";
      popupError.textContent = status;
      player = null;
      return;
    }
    player = opened;
    player.set_keyframe_policy(persistedBudgetMB, persistedMinInterval);
    status = "loaded";
    playing = false;
    playBtn.textContent = "Play";
    frameAccumulator = 0;
    timeline.querySelector("input")!.max = String(player.frame_count());
    timeline.querySelector("input")!.value = "0";
    outlineDirty = true;

    if (player.frame_count() > 0) {
      generating = true;
      popupForm.classList.add("hidden");
      popupActions.classList.add("hidden");
      popupProgress.classList.remove("hidden");
      progressBar.max = player.frame_count();
      progressBar.value = 0;
      progressOverlay.textContent = `0 / ${player.frame_count()}`;
    } else {
      hidePopup(false);
    }
  }

  function advanceKeyframeGeneration() {
    if (!player || !generating) return;
    const t0 = performance.now();
    while (!player.is_at_end() && performance.now() - t0 < 12) {
      player.step_frame();
    }
    const frame = player.frame();
    const total = player.frame_count();
    progressBar.value = frame;
    progressOverlay.textContent = `${frame} / ${total}`;
    if (player.is_at_end()) {
      player.restart();
      generating = false;
      playing = false;
      playBtn.textContent = "Play";
      frameAccumulator = 0;
      timeline.querySelector("input")!.value = "0";
      outlineDirty = true;
      hidePopup(false);
    }
  }

  function select(kind: number, bodyOrd: number, slot: number, query: number) {
    if (!player) return;
    player.set_selection(kind, bodyOrd, slot, query);
    if (kind === SEL_SHAPE || kind === SEL_JOINT) {
      expandedBodies.add(bodyOrd);
      revealSelection = true;
    } else if (kind === SEL_BODY) {
      revealSelection = true;
    } else if (kind === SEL_QUERY) {
      queriesExpanded = true;
      revealSelection = true;
    }
    outlineDirty = true;
  }

  function renderOutline() {
    if (!player) {
      treeEl.textContent = "";
      return;
    }
    let data: Outline;
    try {
      data = JSON.parse(player.outline_json()) as Outline;
    } catch {
      treeEl.textContent = "(outline parse error)";
      return;
    }
    const sel = Array.from(player.selection());
    const selKind = sel[0] ?? SEL_NONE;
    const selBody = sel[1] ?? -1;
    const selSlot = sel[2] ?? -1;
    const selQuery = sel[3] ?? -1;

    treeEl.replaceChildren();
    for (const body of data.bodies) {
      const open = expandedBodies.has(body.ord);
      const bodyRow = el("div", "replay-tree-row");
      if (selKind === SEL_BODY && selBody === body.ord) bodyRow.classList.add("selected");
      const twist = el("button", "replay-tree-twist", open ? "▾" : "▸");
      twist.addEventListener("click", (e) => {
        e.stopPropagation();
        if (expandedBodies.has(body.ord)) expandedBodies.delete(body.ord);
        else expandedBodies.add(body.ord);
        outlineDirty = true;
      });
      const bodyLabel = el("span", "replay-tree-label", body.label);
      bodyRow.append(twist, bodyLabel);
      bodyRow.addEventListener("click", () => select(SEL_BODY, body.ord, -1, -1));
      if (revealSelection && selKind === SEL_BODY && selBody === body.ord) {
        bodyRow.scrollIntoView({ block: "nearest" });
      }
      treeEl.appendChild(bodyRow);

      if (!open) continue;
      for (const s of body.shapes) {
        const row = el("div", "replay-tree-row nested");
        if (selKind === SEL_SHAPE && selBody === body.ord && selSlot === s.slot) {
          row.classList.add("selected");
          if (revealSelection) row.scrollIntoView({ block: "nearest" });
        }
        row.appendChild(el("span", "replay-tree-label", s.label));
        row.addEventListener("click", () => select(SEL_SHAPE, body.ord, s.slot, -1));
        treeEl.appendChild(row);
      }
      for (const j of body.joints) {
        const row = el("div", "replay-tree-row nested");
        if (selKind === SEL_JOINT && selBody === body.ord && selSlot === j.slot) {
          row.classList.add("selected");
        }
        row.appendChild(el("span", "replay-tree-label", j.label));
        row.addEventListener("click", () => select(SEL_JOINT, body.ord, j.slot, -1));
        treeEl.appendChild(row);
      }
    }

    const qHead = el("div", "replay-tree-row");
    const qTwist = el("button", "replay-tree-twist", queriesExpanded ? "▾" : "▸");
    qTwist.addEventListener("click", (e) => {
      e.stopPropagation();
      queriesExpanded = !queriesExpanded;
      outlineDirty = true;
    });
    qHead.append(
      qTwist,
      el("span", "replay-tree-label", `Queries (${data.queries.length})`),
    );
    treeEl.appendChild(qHead);
    if (queriesExpanded) {
      for (const q of data.queries) {
        const row = el("div", "replay-tree-row nested");
        if (selKind === SEL_QUERY && selQuery === q.index) {
          row.classList.add("selected");
          if (revealSelection) row.scrollIntoView({ block: "nearest" });
        }
        row.appendChild(el("span", "replay-tree-label", q.label));
        row.addEventListener("click", () => select(SEL_QUERY, -1, -1, q.index));
        treeEl.appendChild(row);
      }
    }
    revealSelection = false;
  }

  // --- Transport ---
  const timeline = createSlider("Frame", 0, 1, 0, 1, (v) => {
    if (!player || generating) return;
    scrubbing = true;
    playing = false;
    playBtn.textContent = "Play";
    player.seek_frame(v);
    frameAccumulator = 0;
    scrubbing = false;
    outlineDirty = true;
  });

  controls.appendChild(
    createInfoBox(
      "Exact vs C (serial wasm): inspector outliner/detail, click-pick, frame " +
        "query index/overlay, and keyframe Load popup are live. " +
        "Workers: N/A — single-threaded wasm (C Workers slider not applicable).",
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
      outlineDirty = true;
    }),
  );
  transportRow.appendChild(
    createButton("<", () => {
      if (!player) return;
      player.seek_frame(player.frame() - 1);
      playing = false;
      playBtn.textContent = "Play";
      frameAccumulator = 0;
      outlineDirty = true;
    }),
  );
  const playBtn = createButton("Play", () => {
    if (generating) return;
    playing = !playing;
    playBtn.textContent = playing ? "Pause" : "Play";
  });
  transportRow.appendChild(playBtn);
  transportRow.appendChild(
    createButton(">", () => {
      if (!player) return;
      player.seek_frame(player.frame() + 1);
      playing = false;
      playBtn.textContent = "Play";
      frameAccumulator = 0;
      outlineDirty = true;
    }),
  );
  transportRow.appendChild(
    createButton(">|", () => {
      if (!player) return;
      player.seek_frame(player.frame_count());
      frameAccumulator = 0;
      outlineDirty = true;
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
  controls.appendChild(
    createInfoBox("Workers: N/A (serial wasm — recorded worker count only)"),
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
    showLoadPopup(buf, file.name);
    fileInput.value = "";
  });
  controls.appendChild(fileInput);
  controls.appendChild(createButton("Open .b2rec…", () => fileInput.click()));
  controls.appendChild(
    createButton("Reload Falling Hinges", () => {
      status = "recording Falling Hinges…";
      try {
        const bytes = recordFallingHinges(wasm);
        showLoadPopup(bytes, "Falling Hinges (generated)");
      } catch (e) {
        status = e instanceof Error ? e.message : "record failed";
      }
    }),
  );
  controls.appendChild(createSeparator());
  const readout = createReadout();
  controls.appendChild(readout);

  // Default recording → Load popup (C fresh open path).
  try {
    const bytes = recordFallingHinges(wasm);
    showLoadPopup(bytes, "Falling Hinges (generated)");
  } catch (e) {
    status = e instanceof Error ? e.message : "record failed";
  }

  const unbindCamera = bindCameraControls(camera, canvas);

  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0 || !player || generating) return;
    fitCanvas(canvas);
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const w = screenToWorld(camera, canvas, px, py);
    const hit = Array.from(player.pick_at(w.x, w.y));
    const kind = hit[0] ?? SEL_NONE;
    if (kind === SEL_NONE) {
      select(SEL_NONE, -1, -1, -1);
    } else {
      select(kind, hit[1] ?? -1, hit[2] ?? -1, -1);
    }
  };
  canvas.addEventListener("pointerdown", onPointerDown);

  const stop = runSimLoop(() => {
    fitCanvas(canvas);

    if (generating) {
      advanceKeyframeGeneration();
      return;
    }

    if (!player) {
      updateReadout(readout, [
        { label: "Status", value: status },
        { label: "Source", value: sourceLabel },
      ]);
      detailEl.textContent = status;
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
        outlineDirty = true;
      }
      const input = timeline.querySelector("input");
      if (input && !scrubbing) input.value = String(player.frame());
    }

    paintSampleDraw(canvas, camera, player);

    if (outlineDirty) {
      renderOutline();
      detailEl.textContent = player.detail_text();
      outlineDirty = false;
    }

    const diverge = player.diverge_frame();
    const qn = player.frame_query_count();
    updateReadout(readout, [
      { label: "Status", value: status },
      { label: "Source", value: sourceLabel },
      {
        label: "Frame",
        value: `${player.frame()} / ${player.frame_count()}${player.is_at_end() ? " (end)" : ""}`,
      },
      { label: "Bodies", value: String(player.body_count()) },
      { label: "Contacts", value: String(player.contact_count()) },
      { label: "Queries", value: String(qn) },
      { label: "Bit-identical", value: player.has_diverged() ? "NO" : "yes" },
      {
        label: "Diverged at",
        value: diverge >= 0 ? String(diverge) : "—",
      },
      {
        label: "Keyframe",
        value:
          `spacing ${player.keyframe_interval()} · ` +
          `${(player.keyframe_kilobytes() / 1024).toFixed(1)} MB ` +
          `(budget ${player.keyframe_budget_mb()} MB, min ${player.keyframe_min_interval()})`,
      },
      { label: "Workers", value: "N/A (serial wasm)" },
    ]);
  }, readout);

  return () => {
    stop();
    unbindCamera();
    canvas.removeEventListener("pointerdown", onPointerDown);
    closePlayer();
  };
}
