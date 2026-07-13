//! Incremental `b2World_Draw` collector for the demo canvas adapter.
//! Collects solids, lines, points, and text under a global view-flag mask
//! matching the C samples View menu (`sample.cpp` / `b2DebugDraw`).

use box2d_rust::debug_draw::{DebugDraw, HexColor};
use box2d_rust::math_functions as m;
use box2d_rust::math_functions::{transform_world_point, Aabb, Pos, Vec2, WorldTransform};
use box2d_rust::world::{world_draw, World};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

pub const MENU_SHAPES: u32 = 1 << 0;
pub const MENU_CHAIN_NORMALS: u32 = 1 << 1;
pub const MENU_JOINTS: u32 = 1 << 2;
pub const MENU_JOINT_EXTRAS: u32 = 1 << 3;
pub const MENU_BOUNDS: u32 = 1 << 4;
pub const MENU_MASS: u32 = 1 << 5;
pub const MENU_BODY_NAMES: u32 = 1 << 6;
pub const MENU_GRAPH_COLORS: u32 = 1 << 7;
pub const MENU_ISLANDS: u32 = 1 << 8;
pub const MENU_CONTACTS: u32 = 1 << 9;
pub const MENU_CONTACT_NORMALS: u32 = 1 << 10;
pub const MENU_CONTACT_FEATURES: u32 = 1 << 11;
pub const MENU_CONTACT_FORCES: u32 = 1 << 12;
pub const MENU_FRICTION_FORCES: u32 = 1 << 13;
pub const MENU_ANCHOR_A: u32 = 1 << 14;

/// Default mask: shapes only (`b2DefaultDebugDraw`).
pub const DEFAULT_DRAW_FLAGS: u32 = MENU_SHAPES;

thread_local! {
    static DEBUG_FLAGS: Cell<u32> = const { Cell::new(DEFAULT_DRAW_FLAGS) };
    static JOINT_SCALE: Cell<f32> = const { Cell::new(1.0) };
    static FORCE_SCALE: Cell<f32> = const { Cell::new(1.0) };
}

pub fn set_debug_flags(mask: u32) {
    DEBUG_FLAGS.with(|c| c.set(mask));
}
pub fn debug_flags() -> u32 {
    DEBUG_FLAGS.with(|c| c.get())
}
pub fn set_draw_scales(joint_scale: f32, force_scale: f32) {
    JOINT_SCALE.with(|c| c.set(joint_scale));
    FORCE_SCALE.with(|c| c.set(force_scale));
}
pub fn joint_scale() -> f32 {
    JOINT_SCALE.with(|c| c.get())
}
pub fn force_scale() -> f32 {
    FORCE_SCALE.with(|c| c.get())
}

#[wasm_bindgen]
pub fn sim_set_debug_flags(mask: u32) {
    set_debug_flags(mask);
}
#[wasm_bindgen]
pub fn sim_get_debug_flags() -> u32 {
    debug_flags()
}
#[wasm_bindgen]
pub fn sim_set_draw_scales(joint: f32, force: f32) {
    set_draw_scales(joint, force);
}

struct TextEntry {
    x: f32,
    y: f32,
    color: u32,
    text: String,
}

pub struct CollectingDraw {
    pub polygons: Vec<f32>,
    pub circles: Vec<f32>,
    pub capsules: Vec<f32>,
    pub lines: Vec<f32>,
    pub points: Vec<f32>,
    strings: Vec<TextEntry>,
    bounds: Aabb,
    flags: u32,
    joint_scale: f32,
    force_scale: f32,
}

impl CollectingDraw {
    pub fn new(bounds: Aabb) -> Self {
        Self {
            polygons: Vec::new(),
            circles: Vec::new(),
            capsules: Vec::new(),
            lines: Vec::new(),
            points: Vec::new(),
            strings: Vec::new(),
            bounds,
            flags: debug_flags(),
            joint_scale: joint_scale(),
            force_scale: force_scale(),
        }
    }

    fn flag(&self, bit: u32) -> bool {
        self.flags & bit != 0
    }

    pub fn text_json(&self) -> String {
        if self.strings.is_empty() {
            return "[]".to_string();
        }
        let mut out = String::from("[");
        for (i, e) in self.strings.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "{{\"x\":{},\"y\":{},\"color\":{},\"text\":\"{}\"}}",
                e.x,
                e.y,
                e.color,
                escape_json(&e.text)
            ));
        }
        out.push(']');
        out
    }
}

fn escape_json(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c.is_control() => {}
            c => out.push(c),
        }
    }
    out
}

impl DebugDraw for CollectingDraw {
    fn drawing_bounds(&self) -> Aabb {
        self.bounds
    }
    fn force_scale(&self) -> f32 {
        self.force_scale
    }
    fn joint_scale(&self) -> f32 {
        self.joint_scale
    }
    fn draw_shapes(&self) -> bool {
        self.flag(MENU_SHAPES)
    }
    fn draw_chain_normals(&self) -> bool {
        self.flag(MENU_CHAIN_NORMALS)
    }
    fn draw_joints(&self) -> bool {
        self.flag(MENU_JOINTS)
    }
    fn draw_joint_extras(&self) -> bool {
        self.flag(MENU_JOINT_EXTRAS)
    }
    fn draw_bounds_boxes(&self) -> bool {
        self.flag(MENU_BOUNDS)
    }
    fn draw_mass(&self) -> bool {
        self.flag(MENU_MASS)
    }
    fn draw_body_names(&self) -> bool {
        self.flag(MENU_BODY_NAMES)
    }
    fn draw_graph_colors(&self) -> bool {
        self.flag(MENU_GRAPH_COLORS)
    }
    fn draw_islands(&self) -> bool {
        self.flag(MENU_ISLANDS)
    }
    fn draw_contacts(&self) -> bool {
        self.flag(MENU_CONTACTS)
            || self.flag(MENU_CONTACT_NORMALS)
            || self.flag(MENU_CONTACT_FEATURES)
            || self.flag(MENU_CONTACT_FORCES)
            || self.flag(MENU_FRICTION_FORCES)
    }
    fn draw_contact_normals(&self) -> bool {
        self.flag(MENU_CONTACT_NORMALS)
    }
    fn draw_contact_features(&self) -> bool {
        self.flag(MENU_CONTACT_FEATURES)
    }
    fn draw_contact_forces(&self) -> bool {
        self.flag(MENU_CONTACT_FORCES)
    }
    fn draw_friction_forces(&self) -> bool {
        self.flag(MENU_FRICTION_FORCES)
    }
    fn draw_anchor_a(&self) -> bool {
        self.flag(MENU_ANCHOR_A)
    }

    fn draw_solid_polygon(
        &mut self,
        transform: WorldTransform,
        vertices: &[Vec2],
        _radius: f32,
        color: HexColor,
    ) {
        if vertices.is_empty() {
            return;
        }
        self.polygons.push(vertices.len() as f32);
        for &v in vertices {
            let p = transform_world_point(transform, v);
            self.polygons.push(p.x as f32);
            self.polygons.push(p.y as f32);
        }
        self.polygons.push(color.0 as f32);
    }

    fn draw_polygon(&mut self, transform: WorldTransform, vertices: &[Vec2], color: HexColor) {
        self.draw_solid_polygon(transform, vertices, 0.0, color);
    }

    fn draw_solid_circle(
        &mut self,
        transform: WorldTransform,
        center: Vec2,
        radius: f32,
        color: HexColor,
    ) {
        let p = transform_world_point(transform, center);
        let angle = m::rot_get_angle(transform.q);
        self.circles.push(p.x as f32);
        self.circles.push(p.y as f32);
        self.circles.push(radius);
        self.circles.push(angle);
        self.circles.push(color.0 as f32);
    }

    fn draw_circle(&mut self, center: Pos, radius: f32, color: HexColor) {
        self.circles.push(center.x as f32);
        self.circles.push(center.y as f32);
        self.circles.push(radius);
        self.circles.push(0.0);
        self.circles.push(color.0 as f32);
    }

    fn draw_solid_capsule(&mut self, p1: Pos, p2: Pos, radius: f32, color: HexColor) {
        self.capsules.push(p1.x as f32);
        self.capsules.push(p1.y as f32);
        self.capsules.push(p2.x as f32);
        self.capsules.push(p2.y as f32);
        self.capsules.push(radius);
        self.capsules.push(color.0 as f32);
    }

    fn draw_line(&mut self, p1: Pos, p2: Pos, color: HexColor) {
        self.lines.push(p1.x as f32);
        self.lines.push(p1.y as f32);
        self.lines.push(p2.x as f32);
        self.lines.push(p2.y as f32);
        self.lines.push(color.0 as f32);
    }

    fn draw_point(&mut self, p: Pos, size: f32, color: HexColor) {
        self.points.push(p.x as f32);
        self.points.push(p.y as f32);
        self.points.push(size);
        self.points.push(color.0 as f32);
    }

    fn draw_bounds(&mut self, aabb: Aabb, color: HexColor) {
        let l = aabb.lower_bound;
        let u = aabb.upper_bound;
        let corners = [
            Pos {
                x: l.x as _,
                y: l.y as _,
            },
            Pos {
                x: u.x as _,
                y: l.y as _,
            },
            Pos {
                x: u.x as _,
                y: u.y as _,
            },
            Pos {
                x: l.x as _,
                y: u.y as _,
            },
        ];
        for i in 0..4 {
            self.draw_line(corners[i], corners[(i + 1) % 4], color);
        }
    }

    fn draw_transform(&mut self, transform: WorldTransform) {
        let scale = 0.4f32;
        let origin = transform.p;
        let x_axis = transform_world_point(transform, Vec2 { x: scale, y: 0.0 });
        let y_axis = transform_world_point(transform, Vec2 { x: 0.0, y: scale });
        self.draw_line(origin, x_axis, HexColor::RED);
        self.draw_line(origin, y_axis, HexColor::GREEN);
    }

    fn draw_string(&mut self, p: Pos, s: &str, color: HexColor) {
        if s.is_empty() {
            return;
        }
        self.strings.push(TextEntry {
            x: p.x as f32,
            y: p.y as f32,
            color: color.0,
            text: s.to_string(),
        });
    }
}

pub fn collect_world_draw(world: &mut World, bounds: [f32; 4]) -> CollectingDraw {
    let aabb = Aabb {
        lower_bound: Vec2 {
            x: bounds[0],
            y: bounds[1],
        },
        upper_bound: Vec2 {
            x: bounds[2],
            y: bounds[3],
        },
    };
    let mut draw = CollectingDraw::new(aabb);
    world_draw(world, &mut draw);
    draw
}
