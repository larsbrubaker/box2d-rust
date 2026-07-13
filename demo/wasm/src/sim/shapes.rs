//! Shape / body construction variants needed by Bodies, Stacking, and Shapes samples.

use super::SimWorld;
use box2d_rust::body::create_body;
use box2d_rust::collision::{Capsule, Circle, Segment};
use box2d_rust::geometry::{make_box, make_offset_box, make_polygon, make_rounded_box};
use box2d_rust::hull::compute_hull;
use box2d_rust::math_functions::{make_rot, to_pos, Vec2};
use box2d_rust::shape::{
    create_capsule_shape, create_circle_shape, create_polygon_shape, create_segment_shape,
    shape_get_user_data,
};
use box2d_rust::types::{default_body_def, default_shape_def, BodyType};
use box2d_rust::world::world_get_contact_events;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl SimWorld {
    /// Static segment on its own body. (b2CreateSegmentShape)
    pub fn add_segment(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) -> usize {
        let body_id = create_body(&mut self.world, &default_body_def());
        let shape_def = default_shape_def();
        let segment = Segment {
            point1: Vec2 { x: x1, y: y1 },
            point2: Vec2 { x: x2, y: y2 },
        };
        create_segment_shape(&mut self.world, body_id, &shape_def, &segment);
        self.track_body(body_id)
    }

    /// Static capsule. (b2CreateCapsuleShape on a static body)
    pub fn add_static_capsule(
        &mut self,
        x: f32,
        y: f32,
        c1x: f32,
        c1y: f32,
        c2x: f32,
        c2y: f32,
        radius: f32,
        angle: f32,
    ) -> usize {
        let mut body_def = default_body_def();
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);

        let shape_def = default_shape_def();
        let capsule = Capsule {
            center1: Vec2 { x: c1x, y: c1y },
            center2: Vec2 { x: c2x, y: c2y },
            radius,
        };
        create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
        self.track_body(body_id)
    }

    /// Empty body of the given type (0=static, 1=kinematic, 2=dynamic) for
    /// multi-shape construction. Attach shapes with `attach_*`.
    pub fn add_body(&mut self, x: f32, y: f32, angle: f32, body_type: i32) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = match body_type {
            1 => BodyType::Kinematic,
            2 => BodyType::Dynamic,
            _ => BodyType::Static,
        };
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);
        self.track_body(body_id)
    }

    /// Attach an axis-aligned box (optionally offset/rotated in body space).
    pub fn attach_box(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        cx: f32,
        cy: f32,
        angle: f32,
        density: f32,
        friction: f32,
        restitution: f32,
    ) {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        let polygon = if cx == 0.0 && cy == 0.0 && angle == 0.0 {
            make_box(hx, hy)
        } else {
            make_offset_box(hx, hy, Vec2 { x: cx, y: cy }, make_rot(angle))
        };
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
    }

    /// Attach a circle centered in body space.
    pub fn attach_circle(
        &mut self,
        index: usize,
        cx: f32,
        cy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
    ) {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        let circle = Circle {
            center: Vec2 { x: cx, y: cy },
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);
    }

    /// Attach a circle with hit events and shape user-data (Circle Stack).
    pub fn attach_circle_hit(
        &mut self,
        index: usize,
        cx: f32,
        cy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        user_data: u32,
    ) {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.enable_hit_events = true;
        shape_def.user_data = u64::from(user_data);
        let circle = Circle {
            center: Vec2 { x: cx, y: cy },
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);
    }

    /// Attach a capsule in body-local space.
    pub fn attach_capsule(
        &mut self,
        index: usize,
        c1x: f32,
        c1y: f32,
        c2x: f32,
        c2y: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
    ) {
        self.attach_capsule_filtered(
            index, c1x, c1y, c2x, c2y, radius, density, friction, restitution, 0,
        );
    }

    /// Capsule with `filter.groupIndex` (negative = never collide within group).
    #[allow(clippy::too_many_arguments)]
    pub fn attach_capsule_filtered(
        &mut self,
        index: usize,
        c1x: f32,
        c1y: f32,
        c2x: f32,
        c2y: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        group_index: i32,
    ) {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.filter.group_index = group_index;
        let capsule = Capsule {
            center1: Vec2 { x: c1x, y: c1y },
            center2: Vec2 { x: c2x, y: c2y },
            radius,
        };
        create_capsule_shape(&mut self.world, body_id, &shape_def, &capsule);
    }

    /// Circle with rolling resistance (Doohickey wheels).
    #[allow(clippy::too_many_arguments)]
    pub fn attach_circle_rolling(
        &mut self,
        index: usize,
        cx: f32,
        cy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        rolling_resistance: f32,
    ) {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.material.rolling_resistance = rolling_resistance;
        let circle = Circle {
            center: Vec2 { x: cx, y: cy },
            radius,
        };
        create_circle_shape(&mut self.world, body_id, &shape_def, &circle);
    }

    /// Attach a segment in body-local space.
    pub fn attach_segment(&mut self, index: usize, x1: f32, y1: f32, x2: f32, y2: f32) {
        let body_id = self.body_id_at(index);
        let shape_def = default_shape_def();
        let segment = Segment {
            point1: Vec2 { x: x1, y: y1 },
            point2: Vec2 { x: x2, y: y2 },
        };
        create_segment_shape(&mut self.world, body_id, &shape_def, &segment);
    }

    /// Attach a convex polygon from interleaved body-local points [x,y]*.
    pub fn attach_polygon(
        &mut self,
        index: usize,
        points: &[f32],
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
    ) {
        let verts: Vec<Vec2> = points
            .chunks_exact(2)
            .map(|p| Vec2 { x: p[0], y: p[1] })
            .collect();
        let hull = compute_hull(&verts);
        let polygon = make_polygon(&hull, radius);

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
    }

    /// Attach a rounded box (`b2MakeRoundedBox`) centered on the body.
    pub fn attach_rounded_box(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
    ) {
        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        let polygon = make_rounded_box(hx, hy, radius);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
    }

    /// Attach an offset rounded box (`b2MakeOffsetRoundedBox`) in body space.
    /// `invoke_contact_creation` mirrors `b2ShapeDef.invokeContactCreation`
    /// (Tiles ground sets this false).
    #[allow(clippy::too_many_arguments)]
    pub fn attach_offset_rounded_box(
        &mut self,
        index: usize,
        hx: f32,
        hy: f32,
        cx: f32,
        cy: f32,
        angle: f32,
        radius: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        invoke_contact_creation: bool,
    ) {
        use box2d_rust::geometry::make_offset_rounded_box;

        let body_id = self.body_id_at(index);
        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = restitution;
        shape_def.invoke_contact_creation = invoke_contact_creation;
        let polygon = make_offset_rounded_box(
            hx,
            hy,
            Vec2 { x: cx, y: cy },
            make_rot(angle),
            radius,
        );
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
    }

    /// Dynamic convex polygon from interleaved world-relative points [x,y]*.
    /// Hull is computed via `b2ComputeHull`; radius is skin radius.
    pub fn add_polygon(
        &mut self,
        x: f32,
        y: f32,
        angle: f32,
        points: &[f32],
        radius: f32,
        density: f32,
    ) -> usize {
        let verts: Vec<Vec2> = points
            .chunks_exact(2)
            .map(|p| Vec2 { x: p[0], y: p[1] })
            .collect();
        let hull = compute_hull(&verts);
        let polygon = make_polygon(&hull, radius);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        let body_id = create_body(&mut self.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = density;
        shape_def.material.friction = 0.3;
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_body(body_id)
    }

    /// Hit events from the last step as shape user-data pairs `[userA, userB]*`
    /// (Circle Stack text readout).
    pub fn hit_event_user_pairs(&self) -> Vec<i32> {
        let contact_events = world_get_contact_events(&self.world);
        let mut out = Vec::with_capacity(2 * contact_events.hit_events.len());
        for hit in contact_events.hit_events {
            out.push(shape_get_user_data(&self.world, hit.shape_id_a) as i32);
            out.push(shape_get_user_data(&self.world, hit.shape_id_b) as i32);
        }
        out
    }

    /// Kinematic box (moving platforms). Returns the demo body index.
    pub fn add_kinematic_box(
        &mut self,
        x: f32,
        y: f32,
        hx: f32,
        hy: f32,
        angle: f32,
        vx: f32,
        vy: f32,
        omega: f32,
    ) -> usize {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = to_pos(Vec2 { x, y });
        body_def.rotation = make_rot(angle);
        body_def.linear_velocity = Vec2 { x: vx, y: vy };
        body_def.angular_velocity = omega;
        let body_id = create_body(&mut self.world, &body_def);

        let shape_def = default_shape_def();
        let polygon = make_box(hx, hy);
        create_polygon_shape(&mut self.world, body_id, &shape_def, &polygon);
        self.track_body(body_id)
    }
}
