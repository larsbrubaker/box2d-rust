//! Replay inspector: outliner JSON, detail text, pick, selection highlight.
//! Mirrors sample_replay.cpp DrawOutlineTree / DrawDetail / MouseDown / DrawSelectionHighlight.

use crate::interact::CollectingDraw;
use super::names::{body_type_name, joint_type_name, query_type_name, shape_type_name};
use box2d_rust::body::{
    body_compute_aabb, body_get_angular_velocity, body_get_contact_capacity, body_get_contact_data,
    body_get_gravity_scale, body_get_joint_count, body_get_joints, body_get_linear_velocity,
    body_get_mass, body_get_name, body_get_position, body_get_rotational_inertia,
    body_get_shape_count, body_get_shapes, body_get_type, body_get_world_center, body_is_awake,
    body_is_bullet, body_is_enabled, body_is_valid, get_body_full_id, get_body_transform,
};
use box2d_rust::debug_draw::{DebugDraw, HexColor};
use box2d_rust::distance_joint::distance_joint_get_current_length;
use box2d_rust::id::{BodyId, JointId, ShapeId};
use box2d_rust::joint::{
    joint_get_body_a, joint_get_body_b, joint_get_collide_connected, joint_get_constraint_force,
    joint_get_constraint_torque, joint_get_type, joint_is_valid, JointType,
};
use box2d_rust::math_functions as m;
use box2d_rust::math_functions::{Aabb, Pos, Vec2, WorldTransform};
use box2d_rust::prismatic_joint::prismatic_joint_get_translation;
use box2d_rust::recording::{RecPlayer, RecQueryType};
use box2d_rust::revolute_joint::revolute_joint_get_angle;
use box2d_rust::shape::{
    shape_get_aabb, shape_get_body, shape_get_density, shape_get_filter, shape_get_friction,
    shape_get_restitution, shape_get_surface_material, shape_get_type, shape_is_sensor,
    shape_is_valid, shape_test_point,
};
use box2d_rust::types::default_query_filter;
use box2d_rust::world::{world_get_counters, world_get_gravity, world_overlap_aabb, World};

/// Selection kind matching C ReplayViewer::SelKind.
pub const SEL_NONE: i32 = 0;
pub const SEL_BODY: i32 = 1;
pub const SEL_SHAPE: i32 = 2;
pub const SEL_JOINT: i32 = 3;
pub const SEL_QUERY: i32 = 4;

pub struct Selection {
    pub kind: i32,
    pub body_ordinal: i32,
    pub slot: i32,
    pub query: i32,
}

impl Default for Selection {
    fn default() -> Self {
        Self {
            kind: SEL_NONE,
            body_ordinal: -1,
            slot: -1,
            query: -1,
        }
    }
}

fn escape_json(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if c.is_control() => {}
            c => out.push(c),
        }
    }
    out
}

fn selected_body(player: &RecPlayer, sel: &Selection) -> BodyId {
    if sel.body_ordinal < 0 {
        return BodyId::default();
    }
    player.body_id(sel.body_ordinal)
}

fn selected_shape(player: &RecPlayer, sel: &Selection) -> ShapeId {
    let body = selected_body(player, sel);
    let world = player.world();
    if sel.kind != SEL_SHAPE || !body_is_valid(world, body) {
        return ShapeId::default();
    }
    let shapes = body_get_shapes(world, body, 32);
    if sel.slot < 0 || sel.slot as usize >= shapes.len() {
        return ShapeId::default();
    }
    shapes[sel.slot as usize]
}

fn selected_joint(player: &RecPlayer, sel: &Selection) -> JointId {
    let body = selected_body(player, sel);
    let world = player.world();
    if sel.kind != SEL_JOINT || !body_is_valid(world, body) {
        return JointId::default();
    }
    let joints = body_get_joints(world, body, 16);
    if sel.slot < 0 || sel.slot as usize >= joints.len() {
        return JointId::default();
    }
    joints[sel.slot as usize]
}

fn find_body_ordinal(player: &RecPlayer, body: BodyId) -> i32 {
    let count = player.body_count();
    for i in 0..count {
        let id = player.body_id(i);
        if id.index1 == body.index1 && id.generation == body.generation {
            return i;
        }
    }
    -1
}

/// Pick a shape at a world point. Returns [kind, body_ord, slot]; miss clears.
pub fn pick_at(player: &mut RecPlayer, x: f32, y: f32) -> Selection {
    let p = m::to_pos(Vec2 { x, y });
    let d = Vec2 { x: 0.001, y: 0.001 };
    let aabb = Aabb {
        lower_bound: Vec2 { x: -d.x, y: -d.y },
        upper_bound: d,
    };
    let filter = default_query_filter();
    let mut shape_ids: Vec<ShapeId> = Vec::new();
    world_overlap_aabb(player.world_mut(), p, aabb, filter, |shape_id| {
        shape_ids.push(shape_id);
        true
    });

    let mut hit = ShapeId::default();
    for sid in shape_ids {
        if shape_test_point(player.world_mut(), sid, p) {
            hit = sid;
            break;
        }
    }

    if hit.is_null() {
        return Selection::default();
    }

    let body = shape_get_body(player.world(), hit);
    let ordinal = find_body_ordinal(player, body);
    if ordinal < 0 {
        return Selection::default();
    }
    let shapes = body_get_shapes(player.world(), body, 32);
    let mut slot = -1;
    for (i, s) in shapes.iter().enumerate() {
        if s.index1 == hit.index1 && s.generation == hit.generation {
            slot = i as i32;
            break;
        }
    }
    Selection {
        kind: SEL_SHAPE,
        body_ordinal: ordinal,
        slot,
        query: -1,
    }
}

/// Outliner tree as JSON for the left inspector panel.
pub fn outline_json(player: &RecPlayer) -> String {
    let world = player.world();
    let mut out = String::from("{\"bodies\":[");
    let mut first_body = true;
    let count = player.body_count();
    for ord in 0..count {
        let body = player.body_id(ord);
        if body.is_null() || !body_is_valid(world, body) {
            continue;
        }
        if !first_body {
            out.push(',');
        }
        first_body = false;
        let name = body_get_name(world, body);
        let label = if name.is_empty() {
            format!("Body {}  {}", ord, body_type_name(body_get_type(world, body)))
        } else {
            format!("Body {}  {}", ord, name)
        };
        out.push_str(&format!(
            "{{\"ord\":{},\"label\":\"{}\",\"shapes\":[",
            ord,
            escape_json(&label)
        ));
        let shapes = body_get_shapes(world, body, 32);
        for (s, sid) in shapes.iter().enumerate() {
            if s > 0 {
                out.push(',');
            }
            let sl = format!("Shape {}  {}", s, shape_type_name(shape_get_type(world, *sid)));
            out.push_str(&format!(
                "{{\"slot\":{},\"label\":\"{}\"}}",
                s,
                escape_json(&sl)
            ));
        }
        out.push_str("],\"joints\":[");
        let joints = body_get_joints(world, body, 16);
        for (j, jid) in joints.iter().enumerate() {
            if j > 0 {
                out.push(',');
            }
            let jl = format!("{} joint", joint_type_name(joint_get_type(world, *jid)));
            out.push_str(&format!(
                "{{\"slot\":{},\"label\":\"{}\"}}",
                j,
                escape_json(&jl)
            ));
        }
        out.push_str("]}");
    }
    out.push_str("],\"queries\":[");
    let qn = player.frame_query_count();
    for i in 0..qn {
        if i > 0 {
            out.push(',');
        }
        let q = player.frame_query(i);
        let ql = format!("{}  ({})", query_type_name(q.type_), q.hit_count);
        out.push_str(&format!(
            "{{\"index\":{},\"label\":\"{}\",\"hits\":{}}}",
            i,
            escape_json(&ql),
            q.hit_count
        ));
    }
    out.push_str("]}");
    out
}

fn body_xf(world: &World, body: BodyId) -> WorldTransform {
    get_body_transform(world, get_body_full_id(world, body))
}

fn append_body_detail(world: &World, body: BodyId, out: &mut String) {
    let name = body_get_name(world, body);
    let xf = body_xf(world, body);
    let v = body_get_linear_velocity(world, body);
    out.push_str(&format!("id      {}\n", body.index1));
    out.push_str(&format!(
        "name    {}\n",
        if name.is_empty() { "(none)" } else { name }
    ));
    out.push_str(&format!(
        "type    {}\n",
        body_type_name(body_get_type(world, body))
    ));
    out.push_str(&format!("pos     ({:.3}, {:.3})\n", xf.p.x, xf.p.y));
    out.push_str(&format!(
        "angle   {:.1} deg\n",
        m::rot_get_angle(xf.q) * 57.2957795
    ));
    out.push_str(&format!("vel     ({:.3}, {:.3})\n", v.x, v.y));
    out.push_str(&format!(
        "omega   {:.3} rad/s\n",
        body_get_angular_velocity(world, body)
    ));
    out.push_str(&format!("mass    {:.4} kg\n", body_get_mass(world, body)));
    out.push_str(&format!(
        "inertia {:.4}\n",
        body_get_rotational_inertia(world, body)
    ));
    out.push_str(&format!(
        "awake   {}\n",
        if body_is_awake(world, body) {
            "yes"
        } else {
            "no"
        }
    ));
    out.push_str(&format!(
        "enabled {}\n",
        if body_is_enabled(world, body) {
            "yes"
        } else {
            "no"
        }
    ));
    out.push_str(&format!(
        "bullet  {}\n",
        if body_is_bullet(world, body) {
            "yes"
        } else {
            "no"
        }
    ));
    out.push_str(&format!(
        "gravity scale {:.2}\n",
        body_get_gravity_scale(world, body)
    ));
    out.push_str(&format!(
        "shapes {}  joints {}\n",
        body_get_shape_count(world, body),
        body_get_joint_count(world, body)
    ));
}

fn append_shape_detail(world: &World, shape: ShapeId, out: &mut String) {
    out.push_str("--- Shape ---\n");
    out.push_str(&format!("id      {}\n", shape.index1));
    out.push_str(&format!(
        "type     {}\n",
        shape_type_name(shape_get_type(world, shape))
    ));
    let f = shape_get_filter(world, shape);
    out.push_str(&format!("category 0x{:016x}\n", f.category_bits));
    out.push_str(&format!("mask     0x{:016x}\n", f.mask_bits));
    out.push_str(&format!("group    {}\n", f.group_index));
    out.push_str(&format!("density  {:.3}\n", shape_get_density(world, shape)));
    out.push_str(&format!("friction {:.3}\n", shape_get_friction(world, shape)));
    out.push_str(&format!(
        "restitution {:.3}\n",
        shape_get_restitution(world, shape)
    ));
    out.push_str(&format!(
        "sensor   {}\n",
        if shape_is_sensor(world, shape) {
            "yes"
        } else {
            "no"
        }
    ));
    let mat = shape_get_surface_material(world, shape);
    out.push_str(&format!("custom color 0x{:06x}\n", mat.custom_color));
    let aabb = shape_get_aabb(world, shape);
    out.push_str(&format!(
        "aabb ({:.2}, {:.2})-({:.2}, {:.2})\n",
        aabb.lower_bound.x, aabb.lower_bound.y, aabb.upper_bound.x, aabb.upper_bound.y
    ));
}

fn append_joint_detail(world: &World, joint: JointId, out: &mut String) {
    out.push_str("--- Joint ---\n");
    let type_ = joint_get_type(world, joint);
    out.push_str(&format!("type     {}\n", joint_type_name(type_)));
    out.push_str(&format!(
        "body A   {}\n",
        joint_get_body_a(world, joint).index1
    ));
    out.push_str(&format!(
        "body B   {}\n",
        joint_get_body_b(world, joint).index1
    ));
    out.push_str(&format!(
        "collide  {}\n",
        if joint_get_collide_connected(world, joint) {
            "yes"
        } else {
            "no"
        }
    ));
    let force = joint_get_constraint_force(world, joint);
    out.push_str(&format!("force    {:.3}\n", m::length(force)));
    out.push_str(&format!(
        "torque   {:.3}\n",
        joint_get_constraint_torque(world, joint)
    ));
    match type_ {
        JointType::Revolute => {
            out.push_str(&format!(
                "angle    {:.1} deg\n",
                revolute_joint_get_angle(world, joint) * 57.2957795
            ));
        }
        JointType::Prismatic => {
            out.push_str(&format!(
                "translation {:.3}\n",
                prismatic_joint_get_translation(world, joint)
            ));
        }
        JointType::Distance => {
            out.push_str(&format!(
                "length   {:.3}\n",
                distance_joint_get_current_length(world, joint)
            ));
        }
        _ => {}
    }
}

fn append_contact_detail(world: &World, body: BodyId, out: &mut String) {
    let mut capacity = body_get_contact_capacity(world, body);
    if capacity > 64 {
        capacity = 64;
    }
    let contacts = body_get_contact_data(world, body, capacity as usize);
    out.push_str(&format!("--- Contacts ({}) ---\n", contacts.len()));
    for c in &contacts {
        let mfold = &c.manifold;
        out.push_str(&format!(
            "shapes {} / {}\n",
            c.shape_id_a.index1, c.shape_id_b.index1
        ));
        out.push_str(&format!(
            "normal ({:.2}, {:.2})\n",
            mfold.normal.x, mfold.normal.y
        ));
        out.push_str(&format!("points {}\n", mfold.point_count));
        for j in 0..mfold.point_count as usize {
            let mp = &mfold.points[j];
            out.push_str(&format!(
                "  sep {:.3}  Pn {:.2}\n",
                mp.separation, mp.normal_impulse
            ));
        }
        out.push('\n');
    }
}

fn append_query_detail(player: &RecPlayer, sel: &Selection, out: &mut String) {
    let count = player.frame_query_count();
    if sel.query < 0 || sel.query >= count {
        out.push_str("Query not present at this frame.\n");
        return;
    }
    let q = player.frame_query(sel.query);
    out.push_str("--- Query ---\n");
    out.push_str(&format!("type     {}\n", query_type_name(q.type_)));
    let shape_local =
        q.type_ == RecQueryType::ShapeTestPoint || q.type_ == RecQueryType::ShapeRayCast;
    if !shape_local {
        out.push_str(&format!("category 0x{:016x}\n", q.filter.category_bits));
        out.push_str(&format!("mask     0x{:016x}\n", q.filter.mask_bits));
    } else {
        out.push_str(&format!("shape    {}\n", q.shape.index1));
    }
    out.push_str(&format!("hits     {}\n", q.hit_count));
    if q.hit_count > 0 {
        let mut line = String::from("hit shapes: ");
        for h in 0..q.hit_count {
            let hit = player.frame_query_hit(sel.query, h);
            line.push_str(&format!("{} ", hit.shape.index1));
        }
        out.push_str(&line);
        out.push('\n');
    }
}

/// Detail pane text for the current selection.
pub fn detail_text(player: &RecPlayer, sel: &Selection) -> String {
    let world = player.world();
    let mut out = String::new();
    if sel.kind == SEL_NONE {
        out.push_str("Click a node, or a shape in the view.\n");
        let g = world_get_gravity(world);
        let c = world_get_counters(world);
        out.push_str(&format!("gravity ({:.2}, {:.2})\n", g.x, g.y));
        out.push_str(&format!("bodies {}  shapes {}\n", c.body_count, c.shape_count));
        out.push_str(&format!(
            "contacts {}  joints {}\n",
            c.contact_count, c.joint_count
        ));
        return out;
    }
    if sel.kind == SEL_QUERY {
        append_query_detail(player, sel, &mut out);
        return out;
    }
    let body = selected_body(player, sel);
    if !body_is_valid(world, body) {
        out.push_str("Not present at this frame.\n");
        return out;
    }
    out.push_str("--- Body ---\n");
    append_body_detail(world, body, &mut out);
    if sel.kind == SEL_SHAPE {
        let shape = selected_shape(player, sel);
        if shape_is_valid(world, shape) {
            append_shape_detail(world, shape, &mut out);
        }
    } else if sel.kind == SEL_JOINT {
        let joint = selected_joint(player, sel);
        if joint_is_valid(world, joint) {
            append_joint_detail(world, joint, &mut out);
        }
    }
    append_contact_detail(world, body, &mut out);
    out
}

fn draw_body_contacts(world: &World, body: BodyId, draw: &mut CollectingDraw) {
    let mut capacity = body_get_contact_capacity(world, body);
    if capacity > 64 {
        capacity = 64;
    }
    let contacts = body_get_contact_data(world, body, capacity as usize);
    for c in &contacts {
        let pos_a = body_get_position(world, shape_get_body(world, c.shape_id_a));
        let mfold = &c.manifold;
        for j in 0..mfold.point_count as usize {
            let point = m::offset_pos(pos_a, mfold.points[j].anchor_a);
            draw.draw_point(point, 6.0, HexColor::ORANGE);
            let end = Pos {
                x: point.x + 0.3 * mfold.normal.x,
                y: point.y + 0.3 * mfold.normal.y,
            };
            draw.draw_line(point, end, HexColor::ORANGE);
        }
    }
}

fn draw_xf(draw: &mut CollectingDraw, xf: WorldTransform, scale: f32) {
    let origin = xf.p;
    let x_axis = m::transform_world_point(xf, Vec2 { x: scale, y: 0.0 });
    let y_axis = m::transform_world_point(xf, Vec2 { x: 0.0, y: scale });
    draw.draw_line(origin, x_axis, HexColor::RED);
    draw.draw_line(origin, y_axis, HexColor::GREEN);
}

/// Selection highlight + optional single-query overlay into the draw buffers.
pub fn draw_selection(
    player: &RecPlayer,
    sel: &Selection,
    draw: &mut CollectingDraw,
) {
    if sel.kind == SEL_QUERY {
        player.draw_frame_queries(draw, sel.query);
        return;
    }
    let world = player.world();
    if sel.kind == SEL_SHAPE {
        let shape = selected_shape(player, sel);
        if !shape_is_valid(world, shape) {
            return;
        }
        let body = shape_get_body(world, shape);
        draw.draw_bounds(shape_get_aabb(world, shape), HexColor::YELLOW);
        draw_xf(draw, body_xf(world, body), 0.5);
        draw.draw_point(body_get_world_center(world, body), 8.0, HexColor::YELLOW);
        draw_body_contacts(world, body, draw);
    } else if sel.kind == SEL_BODY {
        let body = selected_body(player, sel);
        if !body_is_valid(world, body) {
            return;
        }
        draw.draw_bounds(body_compute_aabb(world, body), HexColor::YELLOW);
        draw_xf(draw, body_xf(world, body), 0.5);
        draw.draw_point(body_get_world_center(world, body), 8.0, HexColor::YELLOW);
        draw_body_contacts(world, body, draw);
    } else if sel.kind == SEL_JOINT {
        let joint = selected_joint(player, sel);
        if !joint_is_valid(world, joint) {
            return;
        }
        let a = joint_get_body_a(world, joint);
        let b = joint_get_body_b(world, joint);
        if body_is_valid(world, a) {
            draw.draw_point(body_get_world_center(world, a), 8.0, HexColor::MAGENTA);
        }
        if body_is_valid(world, b) {
            draw.draw_point(body_get_world_center(world, b), 8.0, HexColor::MAGENTA);
        }
    }
}
