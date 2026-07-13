// Canvas adapter for engine-driven debug draw (`b2World_Draw`).
// Phase 3: solids + lines + points + world-space text under view flags.

import type { SampleCamera } from "./sample-shell.ts";
import { viewBounds, worldToScreen } from "./sample-shell.ts";

function hexToCss(color: number, alpha = 1): string {
  const r = (color >> 16) & 0xff;
  const g = (color >> 8) & 0xff;
  const b = color & 0xff;
  return `rgba(${r},${g},${b},${alpha})`;
}

export interface DebugDrawBuffers {
  polygons: Float32Array;
  circles: Float32Array;
  capsules: Float32Array;
  lines: Float32Array;
  points?: Float32Array;
  textJson?: string;
}

export interface DrawSource {
  collect_draw(lowerX: number, lowerY: number, upperX: number, upperY: number): void;
  draw_polygons(): Float32Array;
  draw_circles(): Float32Array;
  draw_capsules(): Float32Array;
  draw_lines(): Float32Array;
  draw_points?: () => Float32Array;
  draw_text?: () => string;
}

export function paintSampleDraw(
  canvas: HTMLCanvasElement,
  camera: SampleCamera,
  source: DrawSource,
) {
  const b = viewBounds(camera, canvas);
  source.collect_draw(b.lowerX, b.lowerY, b.upperX, b.upperY);
  paintDebugDraw(canvas, camera, {
    polygons: source.draw_polygons(),
    circles: source.draw_circles(),
    capsules: source.draw_capsules(),
    lines: source.draw_lines(),
    points: source.draw_points?.(),
    textJson: source.draw_text?.(),
  });
}

export function paintDebugDraw(
  canvas: HTMLCanvasElement,
  camera: SampleCamera,
  buffers: DebugDrawBuffers,
) {
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const polys = buffers.polygons;
  let i = 0;
  while (i + 1 < polys.length) {
    const count = polys[i++]!;
    if (count < 2 || i + count * 2 >= polys.length) break;
    const color = polys[i + count * 2]!;
    ctx.beginPath();
    for (let v = 0; v < count; v++) {
      const p = worldToScreen(camera, canvas, polys[i + 2 * v]!, polys[i + 2 * v + 1]!);
      if (v === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    ctx.fillStyle = hexToCss(color, 0.35);
    ctx.strokeStyle = hexToCss(color, 1);
    ctx.lineWidth = 1.5;
    ctx.fill();
    ctx.stroke();
    i += count * 2 + 1;
  }

  const circles = buffers.circles;
  for (let c = 0; c + 4 < circles.length; c += 5) {
    const center = worldToScreen(camera, canvas, circles[c]!, circles[c + 1]!);
    const scale = canvas.height / (2 * Math.max(1e-6, camera.zoom));
    const r = circles[c + 2]! * scale;
    const angle = circles[c + 3]!;
    const color = circles[c + 4]!;
    ctx.beginPath();
    ctx.arc(center.x, center.y, r, 0, Math.PI * 2);
    ctx.fillStyle = hexToCss(color, 0.35);
    ctx.strokeStyle = hexToCss(color, 1);
    ctx.lineWidth = 1.5;
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(center.x, center.y);
    ctx.lineTo(center.x + Math.cos(-angle) * r, center.y + Math.sin(-angle) * r);
    ctx.stroke();
  }

  const capsules = buffers.capsules;
  for (let c = 0; c + 5 < capsules.length; c += 6) {
    const a = worldToScreen(camera, canvas, capsules[c]!, capsules[c + 1]!);
    const b = worldToScreen(camera, canvas, capsules[c + 2]!, capsules[c + 3]!);
    const scale = canvas.height / (2 * Math.max(1e-6, camera.zoom));
    const r = capsules[c + 4]! * scale;
    const color = capsules[c + 5]!;
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len = Math.hypot(dx, dy) || 1;
    const nx = (-dy / len) * r;
    const ny = (dx / len) * r;
    ctx.beginPath();
    ctx.moveTo(a.x + nx, a.y + ny);
    ctx.lineTo(b.x + nx, b.y + ny);
    ctx.arc(b.x, b.y, r, Math.atan2(ny, nx), Math.atan2(-ny, -nx));
    ctx.lineTo(a.x - nx, a.y - ny);
    ctx.arc(a.x, a.y, r, Math.atan2(-ny, -nx), Math.atan2(ny, nx));
    ctx.closePath();
    ctx.fillStyle = hexToCss(color, 0.35);
    ctx.strokeStyle = hexToCss(color, 1);
    ctx.lineWidth = 1.5;
    ctx.fill();
    ctx.stroke();
  }

  const lines = buffers.lines;
  for (let c = 0; c + 4 < lines.length; c += 5) {
    const a = worldToScreen(camera, canvas, lines[c]!, lines[c + 1]!);
    const b = worldToScreen(camera, canvas, lines[c + 2]!, lines[c + 3]!);
    const color = lines[c + 4]!;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.strokeStyle = hexToCss(color, 1);
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  const points = buffers.points;
  if (points) {
    const ppm = canvas.height / (2 * Math.max(1e-6, camera.zoom));
    for (let c = 0; c + 3 < points.length; c += 4) {
      const p = worldToScreen(camera, canvas, points[c]!, points[c + 1]!);
      const r = Math.max(2, (points[c + 2]! * ppm) / 40);
      const color = points[c + 3]!;
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle = hexToCss(color, 0.95);
      ctx.fill();
    }
  }

  if (buffers.textJson) {
    let labels: { x: number; y: number; color: number | string; text: string }[] = [];
    try {
      const raw = JSON.parse(buffers.textJson);
      if (Array.isArray(raw)) labels = raw;
    } catch {
      labels = [];
    }
    ctx.save();
    ctx.font = "12px ui-sans-serif, system-ui, sans-serif";
    ctx.textBaseline = "middle";
    for (const lab of labels) {
      if (!lab?.text) continue;
      const p = worldToScreen(camera, canvas, Number(lab.x) || 0, Number(lab.y) || 0);
      const color =
        typeof lab.color === "number" ? hexToCss(lab.color, 1) : String(lab.color || "#fff");
      ctx.fillStyle = "rgba(0,0,0,0.45)";
      const w = ctx.measureText(lab.text).width;
      ctx.fillRect(p.x - 2, p.y - 8, w + 4, 16);
      ctx.fillStyle = color;
      ctx.fillText(lab.text, p.x, p.y);
    }
    ctx.restore();
  }
}
