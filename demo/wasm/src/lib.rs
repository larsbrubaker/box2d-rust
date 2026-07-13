// WASM bindings for the browser demos. Every value shown on the demo site is computed
// by the ported Rust code, never re-implemented in JavaScript.

use box2d_rust::math_functions as m;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn version() -> String {
    box2d_rust::VERSION.to_string()
}

/// Deterministic cosine/sine from the ported `b2ComputeCosSin`. Returns [cos, sin].
#[wasm_bindgen]
pub fn compute_cos_sin(radians: f32) -> Vec<f32> {
    let cs = m::compute_cos_sin(radians);
    vec![cs.cosine, cs.sine]
}

/// Deterministic arctangent from the ported `b2Atan2`.
#[wasm_bindgen]
pub fn atan2(y: f32, x: f32) -> f32 {
    m::atan2(y, x)
}

/// Build a regular polygon rotated by `angle`, centered at (cx, cy).
/// Returns interleaved [x0, y0, x1, y1, ...] computed with the ported Rot/Transform math.
#[wasm_bindgen]
pub fn polygon_points(sides: u32, radius: f32, angle: f32, cx: f32, cy: f32) -> Vec<f32> {
    let q = m::make_rot(angle);
    let t = m::Transform {
        p: m::Vec2 { x: cx, y: cy },
        q,
    };

    let mut out = Vec::with_capacity(2 * sides as usize);
    for i in 0..sides {
        let vertex_angle = 2.0 * m::PI * i as f32 / sides as f32;
        let cs = m::compute_cos_sin(vertex_angle);
        let local = m::Vec2 {
            x: radius * cs.cosine,
            y: radius * cs.sine,
        };
        let world = m::transform_point(t, local);
        out.push(world.x);
        out.push(world.y);
    }
    out
}

// ---------------------------------------------------------------------------
// Geometry category: Convex Hull (sample_geometry.cpp).

mod collision_queries;
mod dynamic_tree_demo;
mod geometry_demo;
mod interact;
mod replay;
mod sim;

pub use dynamic_tree_demo::TreeDemo;
pub use sim::SimWorld;
