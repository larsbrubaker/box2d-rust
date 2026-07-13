//! Incremental `b2World_Draw` collector for the demo canvas adapter.
//! Collects solid polygons, solid circles, solid capsules, and lines.
//! See demo/task-samples.md for deferred debug-draw features.

use box2d_rust::debug_draw::{DebugDraw, HexColor};
use box2d_rust::math_functions as m;
use box2d_rust::math_functions::{transform_world_point, Aabb, Pos, Vec2, WorldTransform};
use box2d_rust::world::{world_draw, World};

/// Collecting DebugDraw that packs primitives into interleaved float buffers.
pub struct CollectingDraw {
    pub polygons: Vec<f32>,
    pub circles: Vec<f32>,
    pub capsules: Vec<f32>,
    pub lines: Vec<f32>,
    bounds: Aabb,
    draw_joints: bool,
}

impl CollectingDraw {
    pub fn new(bounds: Aabb) -> Self {
        Self {
            polygons: Vec::new(),
            circles: Vec::new(),
            capsules: Vec::new(),
            lines: Vec::new(),
            bounds,
            draw_joints: true,
        }
    }

    #[allow(dead_code)]
    pub fn clear(&mut self) {
        self.polygons.clear();
        self.circles.clear();
        self.capsules.clear();
        self.lines.clear();
    }
}

impl DebugDraw for CollectingDraw {
    fn drawing_bounds(&self) -> Aabb {
        self.bounds
    }

    fn draw_shapes(&self) -> bool {
        true
    }

    fn draw_joints(&self) -> bool {
        self.draw_joints
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
        // Outline-only polygons: reuse the solid path with zero fill opacity on JS.
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
}

/// Run `world_draw` into a fresh collector for the given view bounds.
/// Bounds are `[lowerX, lowerY, upperX, upperY]`.
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
