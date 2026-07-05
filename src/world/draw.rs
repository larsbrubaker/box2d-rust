// Debug draw traversal for the world (b2World_Draw in physics_world.c).
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::body_flags;
use crate::collision::ShapeGeometry;
use crate::constants::linear_slop;
use crate::contact::{Contact, ContactSim};
use crate::core::NULL_INDEX;
use crate::debug_draw::{DebugDraw, HexColor};
use crate::joint::draw_joint;
use crate::math_functions::{
    aabb_union, is_valid_aabb, lerp_position, mul_sv, normalize, offset_pos, right_perp, sub_pos,
    transform_world_point, Aabb, Vec2, WorldTransform,
};
use crate::shape::Shape;
use crate::solver_set::{AWAKE_SET, DISABLED_SET};
use crate::types::{BodyType, BODY_TYPE_COUNT};
use crate::world::World;

/// (b2DrawShape)
fn draw_shape(
    draw: &mut dyn DebugDraw,
    shape: &Shape,
    transform: WorldTransform,
    color: HexColor,
    draw_chain_normals: bool,
) {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let p1 = transform_world_point(transform, capsule.center1);
            let p2 = transform_world_point(transform, capsule.center2);
            draw.draw_solid_capsule(p1, p2, capsule.radius, color);
        }

        ShapeGeometry::Circle(circle) => {
            draw.draw_solid_circle(transform, circle.center, circle.radius, color);
        }

        ShapeGeometry::Polygon(poly) => {
            draw.draw_solid_polygon(
                transform,
                &poly.vertices[..poly.count as usize],
                poly.radius,
                color,
            );
        }

        ShapeGeometry::Segment(segment) => {
            let p1 = transform_world_point(transform, segment.point1);
            let p2 = transform_world_point(transform, segment.point2);
            draw.draw_line(p1, p2, color);
        }

        ShapeGeometry::ChainSegment(chain_segment) => {
            let segment = &chain_segment.segment;
            let p1 = transform_world_point(transform, segment.point1);
            let p2 = transform_world_point(transform, segment.point2);
            draw.draw_line(p1, p2, color);
            draw.draw_point(p2, 4.0, color);

            if draw_chain_normals {
                let c = lerp_position(p1, p2, 0.5);
                let e = normalize(sub_pos(p2, p1));
                let n = right_perp(e);
                let length = 0.2 * crate::core::get_length_units_per_meter();
                draw.draw_line(c, offset_pos(c, mul_sv(length, n)), HexColor::PALE_GREEN);
            }
        }
    }
}

/// Immutable counterpart of get_contact_sim for the draw traversal.
/// (b2GetContactSim)
fn get_contact_sim_ref<'a>(world: &'a World, contact: &Contact) -> &'a ContactSim {
    if contact.set_index == AWAKE_SET && contact.color_index != NULL_INDEX {
        // contact lives in constraint graph
        &world.constraint_graph.colors[contact.color_index as usize].contact_sims
            [contact.local_index as usize]
    } else {
        &world.solver_sets[contact.set_index as usize].contact_sims[contact.local_index as usize]
    }
}

/// The per-shape body of the C DrawQueryCallback: colors the shape by body
/// state and marks its body for the second pass.
fn draw_query_callback(world: &mut World, draw: &mut dyn DebugDraw, shape_id: i32) {
    let shape = &world.shapes[shape_id as usize];
    debug_assert!(shape.id == shape_id);
    let body = &world.bodies[shape.body_id as usize];
    let body_sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

    if draw.draw_shapes() {
        let color = if shape.material.custom_color != 0 {
            HexColor(shape.material.custom_color)
        } else if body.type_ == BodyType::Dynamic && body.mass == 0.0 {
            // Bad body
            HexColor::RED
        } else if body.set_index == DISABLED_SET {
            HexColor::SLATE_GRAY
        } else if shape.sensor_index != NULL_INDEX {
            HexColor::WHEAT
        } else if body.flags & body_flags::HAD_TIME_OF_IMPACT != 0 {
            HexColor::LIME
        } else if (body_sim.flags & body_flags::IS_BULLET != 0) && body.set_index == AWAKE_SET {
            HexColor::TURQUOISE
        } else if body.flags & body_flags::IS_SPEED_CAPPED != 0 {
            HexColor::YELLOW
        } else if body_sim.flags & body_flags::IS_FAST != 0 {
            HexColor::SALMON
        } else if body.type_ == BodyType::Static {
            HexColor::PALE_GREEN
        } else if body.type_ == BodyType::Kinematic {
            HexColor::ROYAL_BLUE
        } else if body.set_index == AWAKE_SET {
            HexColor::PINK
        } else {
            HexColor::GRAY
        };

        let draw_chain_normals = draw.draw_chain_normals();
        draw_shape(draw, shape, body_sim.transform, color, draw_chain_normals);
    }

    if draw.draw_bounds_boxes() {
        draw.draw_bounds(shape.fat_aabb, HexColor::GOLD);
    }

    let body_id = shape.body_id;
    world.debug_body_set.set_bit(body_id as u32);
}

/// Draw joints attached to this body that have not been drawn yet.
fn draw_body_joints(world: &mut World, draw: &mut dyn DebugDraw, body_id: i32) {
    let mut joint_key = world.bodies[body_id as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        // avoid double draw
        if !world.debug_joint_set.get_bit(joint_id as u32) {
            let joint = &world.joints[joint_id as usize];
            draw_joint(draw, world, joint);
            world.debug_joint_set.set_bit(joint_id as u32);
        }

        let joint = &world.joints[joint_id as usize];
        joint_key = joint.edges[edge_index as usize].next_key;
    }
}

/// Draw contact points on this dynamic body that have not been drawn yet.
fn draw_body_contacts(world: &mut World, draw: &mut dyn DebugDraw, body_id: i32) {
    const K_AXIS_SCALE: f32 = 0.3;
    let speculative_color = HexColor::GAINSBORO;
    let add_color = HexColor::GREEN;
    let persist_color = HexColor::BLUE;
    let normal_color = HexColor::DIM_GRAY;
    let impulse_color = HexColor::MAGENTA;
    let friction_color = HexColor::YELLOW;

    let slop = linear_slop();

    let mut contact_key = world.bodies[body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;
        let contact = &world.contacts[contact_id as usize];
        contact_key = contact.edges[edge_index as usize].next_key;

        // avoid double draw
        if !world.debug_contact_set.get_bit(contact_id as u32) {
            let contact_sim = get_contact_sim_ref(world, contact);
            let body_a = &world.bodies[contact.edges[0].body_id as usize];
            let body_sim_a = &world.solver_sets[body_a.set_index as usize].body_sims
                [body_a.local_index as usize];
            let body_b = &world.bodies[contact.edges[1].body_id as usize];
            let body_sim_b = &world.solver_sets[body_b.set_index as usize].body_sims
                [body_b.local_index as usize];
            let point_count = contact_sim.manifold.point_count;
            let normal = contact_sim.manifold.normal;

            for j in 0..point_count as usize {
                let mp = &contact_sim.manifold.points[j];

                let p = if draw.draw_anchor_a() {
                    offset_pos(body_sim_a.center, mp.anchor_a)
                } else {
                    offset_pos(body_sim_b.center, mp.anchor_b)
                };

                if draw.draw_graph_colors() && contact.color_index != NULL_INDEX {
                    // graph color
                    let point_size =
                        if contact.color_index == crate::constraint_graph::OVERFLOW_INDEX {
                            7.5
                        } else {
                            5.0
                        };
                    draw.draw_point(
                        p,
                        point_size,
                        crate::constraint_graph::get_graph_color(contact.color_index),
                    );
                } else if mp.separation > slop {
                    // Speculative
                    draw.draw_point(p, 5.0, speculative_color);
                } else if !mp.persisted {
                    // Add
                    draw.draw_point(p, 10.0, add_color);
                } else {
                    // Persist
                    draw.draw_point(p, 5.0, persist_color);
                }

                if draw.draw_contact_normals() {
                    let p1 = p;
                    let p2 = offset_pos(p1, mul_sv(K_AXIS_SCALE, normal));
                    draw.draw_line(p1, p2, normal_color);

                    let buffer = format!(" {:.2}", mp.separation);
                    draw.draw_string(p1, &buffer, HexColor::WHITE);
                } else if draw.draw_contact_forces() {
                    // todo validate
                    // multiply by one-half due to relax iteration
                    let force = 0.5 * mp.total_normal_impulse * world.inv_dt;
                    let p1 = p;
                    let p2 = offset_pos(p1, mul_sv(draw.force_scale() * force, normal));
                    draw.draw_line(p1, p2, impulse_color);
                    let buffer = format!("{:.1}", force);
                    draw.draw_string(p1, &buffer, HexColor::WHITE);
                }

                if draw.draw_contact_features() {
                    let buffer = format!("{}", mp.id);
                    draw.draw_string(p, &buffer, HexColor::ORANGE);
                }

                if draw.draw_friction_forces() {
                    let force = 0.5 * mp.tangent_impulse * world.inv_h;
                    let tangent = right_perp(normal);
                    let p1 = p;
                    let p2 = offset_pos(p1, mul_sv(draw.force_scale() * force, tangent));
                    draw.draw_line(p1, p2, friction_color);
                    let buffer = format!("{:.1}", force);
                    draw.draw_string(p1, &buffer, HexColor::WHITE);
                }
            }

            world.debug_contact_set.set_bit(contact_id as u32);
        }
    }
}

/// Draw the island containing this body as a bounding box, once per island.
fn draw_body_island(world: &mut World, draw: &mut dyn DebugDraw, body_id: i32) {
    let island_id = world.bodies[body_id as usize].island_id;
    if island_id != NULL_INDEX && !world.debug_island_set.get_bit(island_id as u32) {
        let island = &world.islands[island_id as usize];
        if island.set_index == NULL_INDEX {
            // C `continue`s here without clearing the body bit, which would
            // never terminate; a valid islandId never has a null set index.
            return;
        }

        let mut shape_count = 0;
        let mut aabb = Aabb {
            lower_bound: Vec2 {
                x: f32::MAX,
                y: f32::MAX,
            },
            upper_bound: Vec2 {
                x: -f32::MAX,
                y: -f32::MAX,
            },
        };

        for body_index in 0..island.bodies.len() {
            let island_body_id = island.bodies[body_index];
            let island_body = &world.bodies[island_body_id as usize];
            let mut shape_id = island_body.head_shape_id;
            while shape_id != NULL_INDEX {
                let shape = &world.shapes[shape_id as usize];
                aabb = aabb_union(aabb, shape.fat_aabb);
                shape_count += 1;
                shape_id = shape.next_shape_id;
            }
        }

        if shape_count > 0 {
            draw.draw_bounds(aabb, HexColor::ORANGE_RED);
        }

        world.debug_island_set.set_bit(island_id as u32);
    }
}

// todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
// solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
/// (b2World_Draw)
pub fn world_draw(world: &mut World, draw: &mut dyn DebugDraw) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    debug_assert!(is_valid_aabb(draw.drawing_bounds()));

    let body_capacity = world.body_id_pool.id_capacity();
    world
        .debug_body_set
        .set_bit_count_and_clear(body_capacity as u32);

    let joint_capacity = world.joint_id_pool.id_capacity();
    world
        .debug_joint_set
        .set_bit_count_and_clear(joint_capacity as u32);

    let contact_capacity = world.contact_id_pool.id_capacity();
    world
        .debug_contact_set
        .set_bit_count_and_clear(contact_capacity as u32);

    let island_capacity = world.island_id_pool.id_capacity();
    world
        .debug_island_set
        .set_bit_count_and_clear(island_capacity as u32);

    for i in 0..BODY_TYPE_COUNT {
        // The C DrawQueryCallback runs during traversal; the shape ids are
        // collected first here so the callback body can borrow the world
        // mutably, preserving the per-tree visit order.
        let mut shape_ids = Vec::new();
        world.broad_phase.trees[i].query_all(draw.drawing_bounds(), |_, user_data| {
            shape_ids.push(user_data as i32);
            true
        });

        for shape_id in shape_ids {
            draw_query_callback(world, draw, shape_id);
        }
    }

    let word_count = world.debug_body_set.block_count as usize;
    // The body-bit iteration only appends draw output, never new body bits, so
    // a copy of the words matches C iterating the live set.
    let words: Vec<u64> = world.debug_body_set.blocks[..word_count].to_vec();
    for (k, word) in words.into_iter().enumerate() {
        let mut word = word;
        while word != 0 {
            let ctz = word.trailing_zeros();
            let body_id = (64 * k as u32 + ctz) as i32;

            if draw.draw_body_names() && !world.bodies[body_id as usize].name.is_empty() {
                let offset = Vec2 { x: 0.1, y: 0.1 };
                let body = &world.bodies[body_id as usize];
                let body_sim = &world.solver_sets[body.set_index as usize].body_sims
                    [body.local_index as usize];

                let transform = WorldTransform {
                    p: body_sim.center,
                    q: body_sim.transform.q,
                };
                let p = transform_world_point(transform, offset);
                let name = world.bodies[body_id as usize].name.clone();
                draw.draw_string(p, &name, HexColor::BLUE_VIOLET);
            }

            if draw.draw_mass() && world.bodies[body_id as usize].type_ == BodyType::Dynamic {
                let offset = Vec2 { x: 0.1, y: 0.1 };
                let body = &world.bodies[body_id as usize];
                let body_sim = &world.solver_sets[body.set_index as usize].body_sims
                    [body.local_index as usize];

                let transform = WorldTransform {
                    p: body_sim.center,
                    q: body_sim.transform.q,
                };
                draw.draw_line(body_sim.center0, body_sim.center, HexColor::WHITE_SMOKE);
                draw.draw_transform(transform);

                let p = transform_world_point(transform, offset);
                let buffer = format!("  {:.2}", world.bodies[body_id as usize].mass);
                draw.draw_string(p, &buffer, HexColor::WHITE);
            }

            if draw.draw_joints() {
                draw_body_joints(world, draw, body_id);
            }

            if draw.draw_contacts() && world.bodies[body_id as usize].type_ == BodyType::Dynamic {
                draw_body_contacts(world, draw, body_id);
            }

            if draw.draw_islands() {
                draw_body_island(world, draw, body_id);
            }

            // Clear the smallest set bit
            word &= word - 1;
        }
    }
}
