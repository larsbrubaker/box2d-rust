// WASM module loader. The pkg is imported at runtime (not bundled) so the
// box2d_wasm_bg.wasm URL resolves relative to public/pkg/ in both the dev
// server and the static GitHub Pages deployment.

export interface SimWorld {
  add_static_box(x: number, y: number, hx: number, hy: number): number;
  add_box(x: number, y: number, hx: number, hy: number, density: number): number;
  add_circle(x: number, y: number, radius: number, density: number): number;
  step(dt: number, subStepCount: number): void;
  positions(): Float32Array;
  awake_body_count(): number;
  contact_count(): number;
  body_count(): number;
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
