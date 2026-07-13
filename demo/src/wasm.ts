// WASM module loader. The pkg is imported at runtime (not bundled) so the
// box2d_wasm_bg.wasm URL resolves relative to public/pkg/ in both the dev
// server and the static GitHub Pages deployment.

export interface SimWorld {
  // --- construction (legacy + sample) ---
  add_static_box(x: number, y: number, hx: number, hy: number): number;
  add_box(x: number, y: number, hx: number, hy: number, density: number): number;
  add_box_rotated(
    x: number,
    y: number,
    hx: number,
    hy: number,
    density: number,
    angle: number,
  ): number;
  add_circle(x: number, y: number, radius: number, density: number): number;
  add_capsule(
    x: number,
    y: number,
    hl: number,
    radius: number,
    density: number,
    angle: number,
  ): number;
  add_chain(points: Float32Array | number[], isLoop: boolean): number;
  add_segment(x1: number, y1: number, x2: number, y2: number): number;
  add_static_capsule(
    x: number,
    y: number,
    c1x: number,
    c1y: number,
    c2x: number,
    c2y: number,
    radius: number,
    angle: number,
  ): number;
  /** Empty body: 0=static, 1=kinematic, 2=dynamic. Attach shapes with attach_*. */
  add_body(x: number, y: number, angle: number, bodyType: number): number;
  attach_box(
    index: number,
    hx: number,
    hy: number,
    cx: number,
    cy: number,
    angle: number,
    density: number,
    friction: number,
    restitution: number,
  ): void;
  attach_circle(
    index: number,
    cx: number,
    cy: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
  ): void;
  attach_capsule(
    index: number,
    c1x: number,
    c1y: number,
    c2x: number,
    c2y: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
  ): void;
  attach_segment(index: number, x1: number, y1: number, x2: number, y2: number): void;
  add_polygon(
    x: number,
    y: number,
    angle: number,
    points: Float32Array | number[],
    radius: number,
    density: number,
  ): number;
  add_kinematic_box(
    x: number,
    y: number,
    hx: number,
    hy: number,
    angle: number,
    vx: number,
    vy: number,
    omega: number,
  ): number;

  // --- joints ---
  add_hinge_joint(indexA: number, indexB: number, px: number, py: number): number;
  add_distance_joint(
    indexA: number,
    indexB: number,
    ax: number,
    ay: number,
    bx: number,
    by: number,
  ): number;
  add_revolute_joint(
    indexA: number,
    indexB: number,
    px: number,
    py: number,
    enableLimit: boolean,
    lowerAngle: number,
    upperAngle: number,
    enableMotor: boolean,
    motorSpeed: number,
    maxMotorTorque: number,
    enableSpring: boolean,
    hertz: number,
    dampingRatio: number,
    collideConnected: boolean,
  ): number;
  add_prismatic_joint(
    indexA: number,
    indexB: number,
    px: number,
    py: number,
    ax: number,
    ay: number,
    enableLimit: boolean,
    lower: number,
    upper: number,
    enableMotor: boolean,
    motorSpeed: number,
    maxMotorForce: number,
    enableSpring: boolean,
    hertz: number,
    dampingRatio: number,
    collideConnected: boolean,
  ): number;
  add_wheel_joint(
    indexA: number,
    indexB: number,
    px: number,
    py: number,
    ax: number,
    ay: number,
    enableLimit: boolean,
    lower: number,
    upper: number,
    enableMotor: boolean,
    motorSpeed: number,
    maxMotorTorque: number,
    enableSpring: boolean,
    hertz: number,
    dampingRatio: number,
    collideConnected: boolean,
  ): number;
  add_weld_joint(
    indexA: number,
    indexB: number,
    px: number,
    py: number,
    linearHertz: number,
    angularHertz: number,
    linearDampingRatio: number,
    angularDampingRatio: number,
    collideConnected: boolean,
  ): number;
  add_motor_joint(
    indexA: number,
    indexB: number,
    linearHertz: number,
    linearDampingRatio: number,
    maxSpringForce: number,
    angularHertz: number,
    angularDampingRatio: number,
    maxSpringTorque: number,
    maxVelocityForce: number,
    maxVelocityTorque: number,
    collideConnected: boolean,
  ): number;
  add_filter_joint(indexA: number, indexB: number): number;
  add_revolute_joint_angled(
    indexA: number,
    indexB: number,
    px: number,
    py: number,
    frameAngleA: number,
    enableLimit: boolean,
    lowerAngle: number,
    upperAngle: number,
    enableMotor: boolean,
    maxMotorTorque: number,
  ): number;
  joint_count(): number;
  joint_anchors(): Float32Array;

  // --- body ops ---
  destroy_body(index: number): void;
  set_transform(index: number, x: number, y: number, angle: number): void;
  set_body_type(index: number, bodyType: number): void;
  get_body_type(index: number): number;
  enable_body(index: number): void;
  disable_body(index: number): void;
  is_body_enabled(index: number): boolean;
  set_awake(index: number, awake: boolean): void;
  wake_touching(index: number): void;
  set_linear_velocity(index: number, vx: number, vy: number): void;
  get_linear_velocity(index: number): Float32Array;
  set_angular_velocity(index: number, omega: number): void;
  get_angular_velocity(index: number): number;
  set_gravity_scale(index: number, scale: number): void;
  get_mass(index: number): number;
  apply_force(index: number, fx: number, fy: number, px: number, py: number, wake: boolean): void;
  apply_force_to_center(index: number, fx: number, fy: number, wake: boolean): void;
  apply_torque(index: number, torque: number, wake: boolean): void;
  apply_linear_impulse(
    index: number,
    ix: number,
    iy: number,
    px: number,
    py: number,
    wake: boolean,
  ): void;
  apply_linear_impulse_to_center(index: number, ix: number, iy: number, wake: boolean): void;

  // --- world ops / misc demos ---
  add_bouncy_ball(x: number, y: number, radius: number, restitution: number): number;
  add_sensor_box(x: number, y: number, hx: number, hy: number): number;
  enable_sensor_visitor(index: number): void;
  event_counts(): Uint32Array;
  hit_events(): Float32Array;
  add_bullet(x: number, y: number, radius: number, vx: number, vy: number): number;
  /** Prefer this over set_continuous_collision (same C API). */
  set_continuous(flag: boolean): void;
  set_sleeping(flag: boolean): void;
  set_warm_starting(flag: boolean): void;
  is_warm_starting_enabled(): boolean;
  set_speculative(flag: boolean): void;
  set_contact_tuning(hertz: number, dampingRatio: number, pushVelocity: number): void;
  get_gravity(): Float32Array;
  snapshot(): Uint8Array;
  restore(image: Uint8Array): boolean;
  state_hash(): string;
  start_recording(): boolean;
  stop_recording(): Uint8Array;
  mover_spawn(x: number, y: number): void;
  mover_update(dt: number, moveX: number, jump: boolean): Float32Array;
  explode(x: number, y: number, radius: number, falloff: number, impulsePerLength: number): void;
  set_gravity(x: number, y: number): void;
  step(dt: number, subStepCount: number): void;
  positions(): Float32Array;
  awake_body_count(): number;
  contact_count(): number;
  body_count(): number;

  // --- mouse grab (C Sample motor joint) ---
  mouse_down(x: number, y: number): boolean;
  mouse_move(x: number, y: number): void;
  mouse_up(): void;
  mouse_active(): boolean;
  set_grab_force_scale(scale: number): void;

  // --- debug draw dump ---
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
