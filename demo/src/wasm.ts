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
  attach_capsule_filtered(
    index: number,
    c1x: number,
    c1y: number,
    c2x: number,
    c2y: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    groupIndex: number,
  ): void;
  attach_circle_rolling(
    index: number,
    cx: number,
    cy: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    rollingResistance: number,
  ): void;
  attach_segment(index: number, x1: number, y1: number, x2: number, y2: number): void;
  attach_rounded_box(
    index: number,
    hx: number,
    hy: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
  ): void;
  attach_circle_hit(
    index: number,
    cx: number,
    cy: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    userData: number,
  ): void;
  attach_polygon(
    index: number,
    points: Float32Array | number[],
    radius: number,
    density: number,
    friction: number,
    restitution: number,
  ): void;
  /** Full surface material (rolling + tangent). Returns demo shape index. */
  attach_box_mat(
    index: number,
    hx: number,
    hy: number,
    cx: number,
    cy: number,
    angle: number,
    density: number,
    friction: number,
    restitution: number,
    rolling: number,
    tangent: number,
  ): number;
  attach_circle_mat(
    index: number,
    cx: number,
    cy: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    rolling: number,
    tangent: number,
  ): number;
  attach_capsule_mat(
    index: number,
    c1x: number,
    c1y: number,
    c2x: number,
    c2y: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    rolling: number,
    tangent: number,
  ): number;
  attach_polygon_mat(
    index: number,
    points: Float32Array | number[],
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    rolling: number,
    tangent: number,
  ): number;
  attach_rounded_box_mat(
    index: number,
    hx: number,
    hy: number,
    radius: number,
    density: number,
    friction: number,
    restitution: number,
    rolling: number,
    tangent: number,
  ): number;
  attach_box_filter(
    index: number,
    hx: number,
    hy: number,
    density: number,
    categoryBits: number,
    maskBits: number,
  ): number;
  attach_segment_filter(
    index: number,
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    categoryBits: number,
    maskBits: number,
  ): number;
  attach_segment_mat(
    index: number,
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    friction: number,
  ): number;
  attach_segment_invoke(index: number, x1: number, y1: number, x2: number, y2: number): number;
  attach_box_custom(index: number, hx: number, hy: number, density: number, userData: number): number;
  attach_chain_segment(
    index: number,
    g1x: number,
    g1y: number,
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    g2x: number,
    g2y: number,
  ): number;
  shape_set_chain_segment(
    shapeIndex: number,
    g1x: number,
    g1y: number,
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    g2x: number,
    g2y: number,
  ): void;
  shape_set_filter(shapeIndex: number, categoryBits: number, maskBits: number): void;
  shape_get_filter(shapeIndex: number): Uint32Array;
  shape_set_friction(shapeIndex: number, friction: number): void;
  shape_set_restitution(shapeIndex: number, restitution: number): void;
  shape_set_surface(
    shapeIndex: number,
    friction: number,
    restitution: number,
    rolling: number,
    tangent: number,
  ): void;
  shape_set_circle(shapeIndex: number, cx: number, cy: number, radius: number): void;
  shape_set_capsule(
    shapeIndex: number,
    c1x: number,
    c1y: number,
    c2x: number,
    c2y: number,
    radius: number,
  ): void;
  shape_set_segment(shapeIndex: number, x1: number, y1: number, x2: number, y2: number): void;
  shape_set_box(shapeIndex: number, hx: number, hy: number): void;
  body_apply_mass_from_shapes(index: number): void;
  enable_body_sleep(index: number, enable: boolean): void;
  apply_wind_to_body(
    index: number,
    wx: number,
    wy: number,
    drag: number,
    lift: number,
    wake: boolean,
  ): void;
  /** Chain with per-point materials: mats = [fric,rest,rolling,tangent]*N. */
  add_chain_mat(points: Float32Array | number[], isLoop: boolean, mats: Float32Array | number[]): number;
  attach_chain(index: number, points: Float32Array | number[], isLoop: boolean): void;
  enable_odd_even_filter(enabled: boolean): void;
  joint_set_frame_angle_a(jointIndex: number, angle: number): void;
  add_body_ex(
    x: number,
    y: number,
    angle: number,
    bodyType: number,
    gravityScale: number,
    enableSleep: boolean,
  ): number;
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
  add_distance_joint_ex(
    indexA: number,
    indexB: number,
    ax: number,
    ay: number,
    bx: number,
    by: number,
    lengthOverride: number,
    enableSpring: boolean,
    hertz: number,
    dampingRatio: number,
    tensionForce: number,
    compressionForce: number,
    enableLimit: boolean,
    minLength: number,
    maxLength: number,
    collideConnected: boolean,
  ): number;
  add_motor_joint_local(
    indexA: number,
    indexB: number,
    ax: number,
    ay: number,
    bx: number,
    by: number,
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
  add_revolute_joint_local(
    indexA: number,
    indexB: number,
    ax: number,
    ay: number,
    bx: number,
    by: number,
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
  add_prismatic_joint_local(
    indexA: number,
    indexB: number,
    ax: number,
    ay: number,
    bx: number,
    by: number,
    worldAx: number,
    worldAy: number,
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
  add_weld_joint_local(
    indexA: number,
    indexB: number,
    ax: number,
    ay: number,
    bx: number,
    by: number,
    angleA: number,
    angleB: number,
    linearHertz: number,
    angularHertz: number,
    linearDampingRatio: number,
    angularDampingRatio: number,
    collideConnected: boolean,
  ): number;
  destroy_joint(index: number): void;
  joint_wake_bodies(index: number): void;
  joint_set_constraint_tuning(index: number, hertz: number, dampingRatio: number): void;
  joint_set_collide_connected(index: number, flag: boolean): void;
  joint_set_force_threshold(index: number, threshold: number): void;
  joint_set_torque_threshold(index: number, threshold: number): void;
  joint_constraint_ft(index: number): Float32Array;
  joint_separations(index: number): Float32Array;
  distance_set_length(index: number, length: number): void;
  distance_enable_spring(index: number, enable: boolean): void;
  distance_set_spring_hertz(index: number, hertz: number): void;
  distance_set_spring_damping(index: number, damping: number): void;
  distance_set_spring_force_range(index: number, lower: number, upper: number): void;
  distance_enable_limit(index: number, enable: boolean): void;
  distance_set_length_range(index: number, minL: number, maxL: number): void;
  revolute_enable_limit(index: number, enable: boolean): void;
  revolute_enable_motor(index: number, enable: boolean): void;
  revolute_enable_spring(index: number, enable: boolean): void;
  revolute_set_motor_speed(index: number, speed: number): void;
  revolute_set_max_motor_torque(index: number, torque: number): void;
  revolute_set_spring_hertz(index: number, hertz: number): void;
  revolute_set_spring_damping(index: number, damping: number): void;
  revolute_set_target_angle(index: number, angle: number): void;
  revolute_set_limits(index: number, lower: number, upper: number): void;
  revolute_get_angle(index: number): number;
  revolute_get_motor_torque(index: number): number;
  prismatic_enable_limit(index: number, enable: boolean): void;
  prismatic_enable_motor(index: number, enable: boolean): void;
  prismatic_enable_spring(index: number, enable: boolean): void;
  prismatic_set_motor_speed(index: number, speed: number): void;
  prismatic_set_max_motor_force(index: number, force: number): void;
  prismatic_set_spring_hertz(index: number, hertz: number): void;
  prismatic_set_spring_damping(index: number, damping: number): void;
  prismatic_set_target_translation(index: number, translation: number): void;
  prismatic_set_limits(index: number, lower: number, upper: number): void;
  prismatic_get_motor_force(index: number): number;
  wheel_enable_limit(index: number, enable: boolean): void;
  wheel_enable_motor(index: number, enable: boolean): void;
  wheel_enable_spring(index: number, enable: boolean): void;
  wheel_set_motor_speed(index: number, speed: number): void;
  wheel_set_max_motor_torque(index: number, torque: number): void;
  wheel_set_spring_hertz(index: number, hertz: number): void;
  wheel_set_spring_damping(index: number, damping: number): void;
  wheel_set_limits(index: number, lower: number, upper: number): void;
  wheel_get_motor_torque(index: number): number;
  weld_set_linear_hertz(index: number, hertz: number): void;
  weld_set_angular_hertz(index: number, hertz: number): void;
  weld_set_linear_damping(index: number, damping: number): void;
  weld_set_angular_damping(index: number, damping: number): void;
  motor_set_max_spring_force(index: number, force: number): void;
  motor_set_max_spring_torque(index: number, torque: number): void;
  body_world_point(index: number, lx: number, ly: number): Float32Array;
  joint_count(): number;
  joint_anchors(): Float32Array;

  // --- body ops ---
  set_bullet(index: number, flag: boolean): void;
  is_body_alive(index: number): boolean;
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
  set_linear_damping(index: number, damping: number): void;
  set_angular_damping(index: number, damping: number): void;
  set_motion_locks(index: number, linearX: boolean, linearY: boolean, angular: boolean): void;
  set_target_transform(
    index: number,
    x: number,
    y: number,
    angle: number,
    timeStep: number,
    wake: boolean,
  ): void;
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
  hit_event_user_pairs(): Int32Array;
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
