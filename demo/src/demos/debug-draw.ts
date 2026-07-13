// Canvas adapter for engine-driven debug draw (`b2World_Draw`).
// Consumes interleaved float buffers produced by SimWorld.draw_* collectors.
//
// Phase 1 scope (incremental): solid polygons, solid circles, solid capsules,
// and lines. Gaps (joints extras, contacts, mass, bounds, text, chain normals,
// graph colors, islands) are tracked in demo/task-samples.md.

import type { SampleCamera } from "./sample-shell.ts";
import { worldToScreen } from "./sample-shell.ts";

function hexToCss(color: number, alpha = 1): string {
  const r = (color >> 16) & 0xff;
  const g = (color >> 8) & 0xff;
  const b = color & 0xff;
  return `rgba(${r},${g},${b},${alpha})`;
}

export interface DebugDrawBuffers {
  /** [x0,y0, x1,y1, ..., color] per polygon; length prefix per poly. */
  polygons: Float32Array;
  /** [cx, cy, radius, angle, color]* */
  circles: Float32Array;
  /** [x1,y1, x2,y2, radius, color]* */
  capsules: Float32Array;
  /** [x1,y1, x2,y2, color]* */
  lines: Float32Array;
}

/** Paint collected debug-draw primitives onto a 2D canvas. */
export function paintDebugDraw(
  canvas: HTMLCanvasElement,
  camera: SampleCamera,
  buffers: DebugDrawBuffers,
) {
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Polygons: packed as [count, x0,y0,..., xn,yn, color] repeating.
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
    // Radius tick so rotation is visible (C solid circle draws an axis).
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
}
