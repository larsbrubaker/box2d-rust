// WASM module loader. The pkg is imported at runtime (not bundled) so the
// box2d_wasm_bg.wasm URL resolves relative to public/pkg/ in both the dev
// server and the static GitHub Pages deployment.

export interface SimWorld {
  add_static_box(x: number, y: number, hx: number, hy: number): number;
  add_box(x: number, y: number, hx: number, hy: number, density: number): number;
  add_box_rotated(x: number, y: number, hx: number, hy: number, density: number, angle: number): number;
  add_circle(x: number, y: number, radius: number, density: number): number;
  add_hinge_joint(indexA: number, indexB: number, px: number, py: number): number;
  add_distance_joint(indexA: number, indexB: number, ax: number, ay: number, bx: number, by: number): number;
  joint_count(): number;
  joint_anchors(): Float32Array;
  add_bouncy_ball(x: number, y: number, radius: number, restitution: number): number;
  add_sensor_box(x: number, y: number, hx: number, hy: number): number;
  enable_sensor_visitor(index: number): void;
  event_counts(): Uint32Array;
  hit_events(): Float32Array;
  add_bullet(x: number, y: number, radius: number, vx: number, vy: number): number;
  set_continuous(flag: boolean): void;
  snapshot(): Uint8Array;
  restore(image: Uint8Array): boolean;
  state_hash(): string;
  start_recording(): boolean;
  stop_recording(): Uint8Array;
  mover_spawn(x: number, y: number): void;
  mover_update(dt: number, moveX: number, jump: boolean): Float32Array;
  add_capsule(x: number, y: number, hl: number, radius: number, density: number, angle: number): number;
  add_chain(points: Float32Array | number[], isLoop: boolean): number;
  explode(x: number, y: number, radius: number, falloff: number, impulsePerLength: number): void;
  set_gravity(x: number, y: number): void;
  step(dt: number, subStepCount: number): void;
  positions(): Float32Array;
  awake_body_count(): number;
  contact_count(): number;
  body_count(): number;
  /** Mouse grab (C Sample motor joint). Returns true if a dynamic body was grabbed. */
  mouse_down(x: number, y: number): boolean;
  mouse_move(x: number, y: number): void;
  mouse_up(): void;
  mouse_active(): boolean;
  set_grab_force_scale(scale: number): void;
  /** Run b2World_Draw into internal buffers for the given view AABB. */
  collect_draw(lowerX: number, lowerY: number, upperX: number, upperY: number): void;
  draw_polygons(): Float32Array;
  draw_circles(): Float32Array;
  draw_capsules(): Float32Array;
  draw_lines(): Float32Array;
  free(): void;
}

export interface SimPlayer {
  step_frame(): boolean;
  seek_frame(frame: number): void;
  frame(): number;
  frame_count(): number;
  has_diverged(): boolean;
  keyframe_interval(): number;
  keyframe_kilobytes(): number;
  positions(): Float32Array;
  awake_body_count(): number;
  contact_count(): number;
  free(): void;
}

export interface Box2dWasm {
  version(): string;
  compute_cos_sin(radians: number): Float32Array;
  atan2(y: number, x: number): number;
  polygon_points(sides: number, radius: number, angle: number, cx: number, cy: number): Float32Array;
  scene_shape(index: number): Float32Array;
  ray_cast_scene(ox: number, oy: number, tx: number, ty: number): Float32Array;
  closest_points(bx: number, by: number): Float32Array;
  collide_with_box(kind: number, bx: number, by: number, angle: number): Float32Array;
  SimWorld: new (gravityY: number) => SimWorld;
  SimPlayer: { open(data: Uint8Array): SimPlayer | undefined };
}

let wasmModule: Box2dWasm | null = null;

export async function loadWasm(): Promise<Box2dWasm> {
  if (!wasmModule) {
    // Resolve relative to page URL (works in both dev server and static deployment)
    const wasmUrl = new URL("./public/pkg/box2d_wasm.js", window.location.href).href;
    const mod = await import(wasmUrl);
    await mod.default();
    wasmModule = mod as Box2dWasm;
  }
  return wasmModule;
}

/// Synchronous access for demo pages; the router awaits loadWasm() before
/// initializing any page.
export function getWasm(): Box2dWasm {
  if (!wasmModule) throw new Error("WASM not loaded yet");
  return wasmModule;
}
