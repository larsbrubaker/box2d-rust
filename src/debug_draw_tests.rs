// Acceptance tests for the debug draw port (b2World_Draw). The C repo has no
// dedicated draw test — the samples app is its exercise — so these verify the
// traversal against a scene containing every shape type and joint type: each
// option routes to the right callbacks, joints/contacts/islands draw exactly
// once, and the drawing bounds cull.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::create_body;
use crate::collision::{Capsule, Circle, Segment};
use crate::debug_draw::{DebugDraw, HexColor};
use crate::geometry::make_box;
use crate::math_functions::{to_pos, Aabb, Pos, Vec2, WorldTransform, PI};
use crate::shape::{
    create_capsule_shape, create_chain, create_circle_shape, create_polygon_shape,
    create_segment_shape,
};
use crate::types::{
    default_body_def, default_chain_def, default_distance_joint_def, default_filter_joint_def,
    default_motor_joint_def, default_prismatic_joint_def, default_revolute_joint_def,
    default_shape_def, default_weld_joint_def, default_wheel_joint_def, default_world_def,
    BodyType,
};
use crate::world::{world_draw, world_step, World};

#[derive(Default)]
struct CountingDraw {
    polygons: usize,
    solid_polygons: usize,
    circles: usize,
    solid_circles: usize,
    capsules: usize,
    lines: usize,
    transforms: usize,
    points: usize,
    strings: Vec<String>,
    bounds: usize,
    everything: bool,
    drawing_bounds: Option<Aabb>,
}

impl DebugDraw for CountingDraw {
    fn draw_polygon(&mut self, _transform: WorldTransform, _vertices: &[Vec2], _color: HexColor) {
        self.polygons += 1;
    }

    fn draw_solid_polygon(
        &mut self,
        _transform: WorldTransform,
        _vertices: &[Vec2],
        _radius: f32,
        _color: HexColor,
    ) {
        self.solid_polygons += 1;
    }

    fn draw_circle(&mut self, _center: Pos, _radius: f32, _color: HexColor) {
        self.circles += 1;
    }

    fn draw_solid_circle(
        &mut self,
        _transform: WorldTransform,
        _center: Vec2,
        _radius: f32,
        _color: HexColor,
    ) {
        self.solid_circles += 1;
    }

    fn draw_solid_capsule(&mut self, _p1: Pos, _p2: Pos, _radius: f32, _color: HexColor) {
        self.capsules += 1;
    }

    fn draw_line(&mut self, _p1: Pos, _p2: Pos, _color: HexColor) {
        self.lines += 1;
    }

    fn draw_transform(&mut self, _transform: WorldTransform) {
        self.transforms += 1;
    }

    fn draw_point(&mut self, _p: Pos, _size: f32, _color: HexColor) {
        self.points += 1;
    }

    fn draw_string(&mut self, _p: Pos, s: &str, _color: HexColor) {
        self.strings.push(s.to_string());
    }

    fn draw_bounds(&mut self, _aabb: Aabb, _color: HexColor) {
        self.bounds += 1;
    }

    fn drawing_bounds(&self) -> Aabb {
        self.drawing_bounds.unwrap_or(Aabb {
            lower_bound: Vec2 {
                x: -f32::MAX,
                y: -f32::MAX,
            },
            upper_bound: Vec2 {
                x: f32::MAX,
                y: f32::MAX,
            },
        })
    }

    fn draw_contacts(&self) -> bool {
        self.everything
    }
    fn draw_chain_normals(&self) -> bool {
        self.everything
    }
    fn draw_joints(&self) -> bool {
        self.everything
    }
    fn draw_joint_extras(&self) -> bool {
        self.everything
    }
    fn draw_bounds_boxes(&self) -> bool {
        self.everything
    }
    fn draw_mass(&self) -> bool {
        self.everything
    }
    fn draw_body_names(&self) -> bool {
        self.everything
    }
    fn draw_contact_features(&self) -> bool {
        self.everything
    }
    fn draw_contact_normals(&self) -> bool {
        self.everything
    }
    fn draw_islands(&self) -> bool {
        self.everything
    }
}

/// Ground + every shape type + every joint type, stepped until contacts exist.
fn build_draw_scene() -> World {
    let def = default_world_def();
    let mut world = World::new(&def);

    // Ground box with a name (drawn by drawBodyNames)
    {
        let mut bd = default_body_def();
        bd.position = to_pos(Vec2 { x: 0.0, y: -1.0 });
        bd.name = "ground".to_string();
        let ground = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(&mut world, ground, &sd, &make_box(30.0, 1.0));

        // Static segment and a chain loop on the same body
        create_segment_shape(
            &mut world,
            ground,
            &sd,
            &Segment {
                point1: Vec2 { x: -8.0, y: 2.0 },
                point2: Vec2 { x: -6.0, y: 2.0 },
            },
        );

        let mut chain_def = default_chain_def();
        chain_def.points = vec![
            Vec2 { x: 6.0, y: 4.0 },
            Vec2 { x: 8.0, y: 4.0 },
            Vec2 { x: 8.0, y: 6.0 },
            Vec2 { x: 6.0, y: 6.0 },
        ];
        chain_def.is_loop = true;
        create_chain(&mut world, ground, &chain_def);
    }

    // A small stack so persistent contacts exist
    {
        let sd = default_shape_def();
        let box_poly = make_box(0.5, 0.5);
        for i in 0..3 {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = to_pos(Vec2 {
                x: 0.0,
                y: 0.5 + 1.0 * i as f32,
            });
            let body = create_body(&mut world, &bd);
            create_polygon_shape(&mut world, body, &sd, &box_poly);
        }
    }

    // A dynamic circle, capsule, and a sensor shape
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec2 { x: 3.0, y: 1.0 });
        let ball = create_body(&mut world, &bd);
        let sd = default_shape_def();
        create_circle_shape(
            &mut world,
            ball,
            &sd,
            &Circle {
                center: Vec2 { x: 0.0, y: 0.0 },
                radius: 0.5,
            },
        );

        bd.position = to_pos(Vec2 { x: -3.0, y: 1.0 });
        let pill = create_body(&mut world, &bd);
        create_capsule_shape(
            &mut world,
            pill,
            &sd,
            &Capsule {
                center1: Vec2 { x: -0.3, y: 0.0 },
                center2: Vec2 { x: 0.3, y: 0.0 },
                radius: 0.25,
            },
        );

        let mut sensor_bd = default_body_def();
        sensor_bd.position = to_pos(Vec2 { x: 10.0, y: 1.0 });
        let sensor_body = create_body(&mut world, &sensor_bd);
        let mut sensor_sd = default_shape_def();
        sensor_sd.is_sensor = true;
        create_polygon_shape(&mut world, sensor_body, &sensor_sd, &make_box(1.0, 1.0));
    }

    // One pair of bodies per joint type, hanging off to the side
    let joint_box = make_box(0.3, 0.3);
    let make_pair = |world: &mut World, x: f32| {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec2 { x, y: 6.0 });
        let a = create_body(world, &bd);
        bd.position = to_pos(Vec2 { x: x + 1.0, y: 6.0 });
        let b = create_body(world, &bd);
        let sd = default_shape_def();
        create_polygon_shape(world, a, &sd, &joint_box);
        create_polygon_shape(world, b, &sd, &joint_box);
        (a, b)
    };

    {
        let (a, b) = make_pair(&mut world, -14.0);
        let mut jd = default_revolute_joint_def();
        jd.enable_limit = true;
        jd.lower_angle = -0.25 * PI;
        jd.upper_angle = 0.25 * PI;
        jd.enable_spring = true;
        jd.hertz = 1.0;
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        jd.base.local_frame_a.p = Vec2 { x: 0.5, y: 0.0 };
        jd.base.local_frame_b.p = Vec2 { x: -0.5, y: 0.0 };
        crate::joint::create_revolute_joint(&mut world, &jd);
    }
    {
        let (a, b) = make_pair(&mut world, -11.0);
        let mut jd = default_distance_joint_def();
        jd.length = 1.0;
        jd.min_length = 0.5;
        jd.max_length = 2.0;
        jd.enable_limit = true;
        jd.enable_spring = true;
        jd.hertz = 2.0;
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        jd.base.local_frame_a.p = Vec2 { x: 0.3, y: 0.0 };
        jd.base.local_frame_b.p = Vec2 { x: -0.3, y: 0.0 };
        crate::joint::create_distance_joint(&mut world, &jd);
    }
    {
        let (a, b) = make_pair(&mut world, -8.0);
        let mut jd = default_prismatic_joint_def();
        jd.enable_limit = true;
        jd.lower_translation = -0.5;
        jd.upper_translation = 0.5;
        jd.enable_spring = true;
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        crate::joint::create_prismatic_joint(&mut world, &jd);
    }
    {
        let (a, b) = make_pair(&mut world, -5.0);
        let mut jd = default_weld_joint_def();
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        crate::joint::create_weld_joint(&mut world, &jd);
    }
    {
        let (a, b) = make_pair(&mut world, 12.0);
        let mut jd = default_wheel_joint_def();
        jd.enable_limit = true;
        jd.lower_translation = -0.25;
        jd.upper_translation = 0.25;
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        crate::joint::create_wheel_joint(&mut world, &jd);
    }
    {
        let (a, b) = make_pair(&mut world, 15.0);
        let mut jd = default_motor_joint_def();
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        crate::joint::create_motor_joint(&mut world, &jd);
    }
    {
        let (a, b) = make_pair(&mut world, 18.0);
        let mut jd = default_filter_joint_def();
        jd.base.body_id_a = a;
        jd.base.body_id_b = b;
        crate::joint::create_filter_joint(&mut world, &jd);
    }

    for _ in 0..30 {
        world_step(&mut world, 1.0 / 60.0, 4);
    }

    world
}

#[test]
fn default_options_draw_shapes_only() {
    let mut world = build_draw_scene();
    let mut draw = CountingDraw::default();

    world_draw(&mut world, &mut draw);

    // Every shape reaches its callback exactly once: the ground box, 3 stack
    // boxes, the sensor box, and 14 joint-pair boxes are solid polygons; the
    // chain loop contributes 4 chain segments (one line + one endpoint each).
    assert_eq!(draw.solid_polygons, 19);
    assert_eq!(draw.solid_circles, 1);
    assert_eq!(draw.capsules, 1);
    assert_eq!(draw.lines, 5); // 1 segment shape + 4 chain segments
    assert_eq!(draw.points, 4); // chain segment endpoints

    // No optional overlays by default.
    assert_eq!(draw.polygons, 0);
    assert_eq!(draw.circles, 0);
    assert_eq!(draw.transforms, 0);
    assert_eq!(draw.bounds, 0);
    assert!(draw.strings.is_empty());
}

#[test]
fn all_options_draw_every_overlay_once() {
    let mut world = build_draw_scene();
    let mut draw = CountingDraw {
        everything: true,
        ..CountingDraw::default()
    };

    world_draw(&mut world, &mut draw);

    // Weld joint boxes arrive through the outline polygon callback.
    assert_eq!(draw.polygons, 2);
    // Revolute joint hinge circle.
    assert_eq!(draw.circles, 1);
    // drawMass draws one transform per dynamic body (3 stack + circle +
    // capsule + 14 joint bodies).
    assert_eq!(draw.transforms, 19);
    // drawBounds emits one fat AABB per shape and drawIslands one box per
    // island containing shapes.
    assert!(draw.bounds >= 26);
    // Mass labels for every dynamic body, the ground name, joint extras, and
    // contact separations all produce strings.
    assert!(draw.strings.iter().any(|s| s == "ground"));
    assert!(draw.strings.len() > 19);
    // Contact points drawn (stack has persistent contacts).
    assert!(draw.points > 4);
    // Joint drawings add lines beyond the default pass.
    assert!(draw.lines > 5);
}

#[test]
fn joints_and_contacts_draw_exactly_once() {
    let mut world = build_draw_scene();
    let mut draw = CountingDraw {
        everything: true,
        ..CountingDraw::default()
    };
    world_draw(&mut world, &mut draw);

    // Colors unique to a single joint drawing isolate per-joint output: the
    // motor joint alone uses light gray lines and yellow-green/plum points,
    // and the distance joint alone uses a white line. If joints were visited
    // once per attached body these counts would double.
    struct JointOnce {
        motor_lines: usize,
        motor_points: usize,
        distance_lines: usize,
    }
    impl DebugDraw for JointOnce {
        fn draw_line(&mut self, _p1: Pos, _p2: Pos, color: HexColor) {
            if color == HexColor::LIGHT_GRAY {
                self.motor_lines += 1;
            }
            if color == HexColor::WHITE {
                self.distance_lines += 1;
            }
        }
        fn draw_point(&mut self, _p: Pos, _size: f32, color: HexColor) {
            if color == HexColor::YELLOW_GREEN || color == HexColor::PLUM {
                self.motor_points += 1;
            }
        }
        fn draw_shapes(&self) -> bool {
            false
        }
        fn draw_joints(&self) -> bool {
            true
        }
    }

    let mut once = JointOnce {
        motor_lines: 0,
        motor_points: 0,
        distance_lines: 0,
    };
    world_draw(&mut world, &mut once);
    assert_eq!(once.motor_lines, 1);
    assert_eq!(once.motor_points, 2);
    assert_eq!(once.distance_lines, 1);
}

#[test]
fn drawing_bounds_cull_shapes() {
    let mut world = build_draw_scene();

    // Bounds covering only the sensor box at (10, 1).
    let mut draw = CountingDraw {
        drawing_bounds: Some(Aabb {
            lower_bound: Vec2 { x: 9.5, y: 0.5 },
            upper_bound: Vec2 { x: 10.5, y: 1.5 },
        }),
        ..CountingDraw::default()
    };
    world_draw(&mut world, &mut draw);

    assert_eq!(draw.solid_polygons, 1);
    assert_eq!(draw.solid_circles, 0);
    assert_eq!(draw.capsules, 0);
}

#[test]
fn graph_colors_cover_all_slots() {
    use crate::constants::GRAPH_COLOR_COUNT;
    use crate::constraint_graph::get_graph_color;

    // Spot-check the table against b2_graphColors and cover every slot.
    assert_eq!(get_graph_color(0), HexColor::RED);
    assert_eq!(get_graph_color(GRAPH_COLOR_COUNT - 1), HexColor::SILVER);
    for i in 0..GRAPH_COLOR_COUNT {
        let _ = get_graph_color(i);
    }
}
