//! C-exact Character Mover (`sample_character.cpp` Mover::SolveMove / Kick).
//! Capsule is query-driven (not a rigid body): CollideMover → SolvePlanes →
//! CastMover → ClipVector, with Quake-style friction and a pogo spring.

use super::SimWorld;
use box2d_rust::body::{
    body_apply_force, body_apply_linear_impulse_to_center, body_get_type, body_get_world_center,
};
use box2d_rust::collision::Capsule;
use box2d_rust::distance::make_proxy;
use box2d_rust::math_functions::{
    add, dot, length, length_squared, mul_sv, normalize, offset_pos, spring_damper, sub_pos, Pos,
    Vec2, VEC2_ZERO,
};
use box2d_rust::mover::{clip_vector, solve_planes, CollisionPlane};
use box2d_rust::shape::{shape_get_body, shape_get_user_data};
use box2d_rust::types::{BodyType, QueryFilter};
use box2d_rust::world::{
    world_cast_mover, world_cast_shape, world_collide_mover, world_overlap_shape,
};
use wasm_bindgen::prelude::*;

/// sample_character.cpp CollisionBits
const STATIC_BIT: u64 = 0x0001;
const MOVER_BIT: u64 = 0x0002;
const DYNAMIC_BIT: u64 = 0x0004;
const DEBRIS_BIT: u64 = 0x0008;

/// C `m_planeCapacity` (:591)
const PLANE_CAPACITY: usize = 8;

/// Pack ShapeUserData into shape.user_data (`maxPush` + `clipVelocity`).
/// Bit 1 marks “present”; bit 0 is clipVelocity; high 32 bits are maxPush.
pub(crate) fn pack_plane_user_data(max_push: f32, clip_velocity: bool) -> u64 {
    let bits = u64::from(max_push.to_bits());
    (bits << 32) | if clip_velocity { 1 } else { 0 } | 0x2
}

fn unpack_plane_user_data(data: u64) -> (f32, bool) {
    if data & 0x2 == 0 {
        (f32::MAX, true)
    } else {
        (f32::from_bits((data >> 32) as u32), data & 1 != 0)
    }
}

/// C capsule: centers (0,±0.5), radius 0.3 (`sample_character.cpp:73`).
fn mover_capsule() -> Capsule {
    Capsule {
        center1: Vec2 { x: 0.0, y: -0.5 },
        center2: Vec2 { x: 0.0, y: 0.5 },
        radius: 0.3,
    }
}

/// PogoShape enum (`sample_character.cpp:29-34`)
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub(crate) enum PogoShape {
    Point = 0,
    Circle = 1,
    Segment = 2,
}

impl SimWorld {
    pub(crate) fn mover_reset_state(&mut self) {
        self.mover_pogo_velocity = 0.0;
        self.mover_on_ground = false;
        self.mover_jump_released = true;
        self.mover_plane_count = 0;
        self.mover_total_iterations = 0;
        self.mover_planes.clear();
        self.mover_pogo_draw.clear();
    }
}

#[wasm_bindgen]
impl SimWorld {
    /// Place the capsule character mover (`sample_character.cpp:71`).
    pub fn mover_spawn(&mut self, x: f32, y: f32) {
        self.mover_position = Pos { x, y };
        self.mover_velocity = VEC2_ZERO;
        self.mover_reset_state();
    }

    /// Tunables matching Mover ImGui defaults (`sample_character.cpp:595-604`).
    #[allow(clippy::too_many_arguments)]
    pub fn mover_set_params(
        &mut self,
        jump_speed: f32,
        min_speed: f32,
        max_speed: f32,
        stop_speed: f32,
        accelerate: f32,
        friction: f32,
        gravity: f32,
        air_steer: f32,
        pogo_hertz: f32,
        pogo_damping: f32,
    ) {
        self.mover_jump_speed = jump_speed;
        self.mover_min_speed = min_speed;
        self.mover_max_speed = max_speed;
        self.mover_stop_speed = stop_speed;
        self.mover_accelerate = accelerate;
        self.mover_friction = friction;
        self.mover_gravity = gravity;
        self.mover_air_steer = air_steer;
        self.mover_pogo_hertz = pogo_hertz;
        self.mover_pogo_damping = pogo_damping;
    }

    /// 0=Point, 1=Circle, 2=Segment (`sample_character.cpp:606`).
    pub fn mover_set_pogo_shape(&mut self, shape: i32) {
        self.mover_pogo_shape = match shape {
            0 => PogoShape::Point as i32,
            1 => PogoShape::Circle as i32,
            _ => PogoShape::Segment as i32,
        };
    }

    /// Kick debris in a circle under the mover (`sample_character.cpp:481-491`).
    pub fn mover_kick(&mut self) {
        let capsule = mover_capsule();
        let origin = Pos {
            x: self.mover_position.x,
            y: self.mover_position.y + capsule.center1.y - 3.0 * capsule.radius,
        };
        let radius = 0.5f32;
        let proxy = make_proxy(&[VEC2_ZERO], radius);
        let filter = QueryFilter {
            category_bits: MOVER_BIT,
            mask_bits: DEBRIS_BIT,
        };
        let mut hit_shapes = Vec::new();
        world_overlap_shape(&mut self.world, origin, &proxy, filter, |shape_id| {
            hit_shapes.push(shape_id);
            true
        });
        let mover_pos = self.mover_position;
        for shape_id in hit_shapes {
            let body_id = shape_get_body(&self.world, shape_id);
            if body_get_type(&self.world, body_id) != BodyType::Dynamic {
                continue;
            }
            let center = body_get_world_center(&self.world, body_id);
            let direction = normalize(sub_pos(center, mover_pos));
            let impulse = Vec2 {
                x: 2.0 * direction.x,
                y: 2.0,
            };
            body_apply_linear_impulse_to_center(&mut self.world, body_id, impulse, true);
        }
        self.mover_kick_draw = vec![origin.x, origin.y, radius];
    }

    /// Clear one-shot kick draw after the frame paints it.
    pub fn mover_clear_kick_draw(&mut self) {
        self.mover_kick_draw.clear();
    }

    pub fn mover_kick_draw(&self) -> Vec<f32> {
        self.mover_kick_draw.clone()
    }

    /// Collision planes for debug draw: [nx, ny, offset] * count.
    pub fn mover_planes(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.mover_planes.len() * 3);
        for p in &self.mover_planes {
            out.push(p.plane.normal.x);
            out.push(p.plane.normal.y);
            out.push(p.plane.offset);
        }
        out
    }

    /// Pogo debug: [ox, oy, hx, hy, hit (0/1), shape (0/1/2), r or seg half].
    pub fn mover_pogo_draw(&self) -> Vec<f32> {
        self.mover_pogo_draw.clone()
    }

    /// One C SolveMove step. `jump_held` is W-key state this frame.
    /// Returns [x, y, vx, vy, grounded, planeCount, iterations].
    pub fn mover_update(&mut self, dt: f32, throttle: f32, jump_held: bool) -> Vec<f32> {
        if dt <= 0.0 {
            return vec![
                self.mover_position.x,
                self.mover_position.y,
                self.mover_velocity.x,
                self.mover_velocity.y,
                if self.mover_on_ground { 1.0 } else { 0.0 },
                self.mover_plane_count as f32,
                self.mover_total_iterations as f32,
            ];
        }

        // Jump edge (`sample_character.cpp:541-553`)
        if jump_held {
            if self.mover_on_ground && self.mover_jump_released {
                self.mover_velocity.y = self.mover_jump_speed;
                self.mover_on_ground = false;
                self.mover_jump_released = false;
            }
        } else {
            self.mover_jump_released = true;
        }

        self.solve_move(dt, throttle);

        vec![
            self.mover_position.x,
            self.mover_position.y,
            self.mover_velocity.x,
            self.mover_velocity.y,
            if self.mover_on_ground { 1.0 } else { 0.0 },
            self.mover_plane_count as f32,
            self.mover_total_iterations as f32,
        ]
    }
}

impl SimWorld {
    /// `Mover::SolveMove` (`sample_character.cpp:233-409`).
    fn solve_move(&mut self, time_step: f32, throttle: f32) {
        let capsule = mover_capsule();

        // Friction (:235-251)
        let speed = length(self.mover_velocity);
        if speed < self.mover_min_speed {
            self.mover_velocity = VEC2_ZERO;
        } else if self.mover_on_ground {
            let control = if speed < self.mover_stop_speed {
                self.mover_stop_speed
            } else {
                speed
            };
            let drop = control * self.mover_friction * time_step;
            let new_speed = (speed - drop).max(0.0);
            self.mover_velocity = mul_sv(new_speed / speed, self.mover_velocity);
        }

        let desired_velocity = Vec2 {
            x: self.mover_max_speed * throttle,
            y: 0.0,
        };
        let desired_speed_raw = length(desired_velocity);
        let desired_direction = if desired_speed_raw > 0.0 {
            normalize(desired_velocity)
        } else {
            VEC2_ZERO
        };
        let desired_speed = desired_speed_raw.min(self.mover_max_speed);

        if self.mover_on_ground {
            self.mover_velocity.y = 0.0;
        }

        // Accelerate (:267-280)
        let current_speed = dot(self.mover_velocity, desired_direction);
        let add_speed = desired_speed - current_speed;
        if add_speed > 0.0 {
            let steer = if self.mover_on_ground {
                1.0
            } else {
                self.mover_air_steer
            };
            let mut accel_speed = steer * self.mover_accelerate * self.mover_max_speed * time_step;
            if accel_speed > add_speed {
                accel_speed = add_speed;
            }
            self.mover_velocity = add(self.mover_velocity, mul_sv(accel_speed, desired_direction));
        }

        self.mover_velocity.y -= self.mover_gravity * time_step;

        // Pogo cast (:284-371)
        let pogo_rest_length = 3.0 * capsule.radius;
        let ray_length = pogo_rest_length + capsule.radius;
        let circle_r = 0.5 * capsule.radius;
        let segment_offset = Vec2 {
            x: 0.75 * capsule.radius,
            y: 0.0,
        };

        let pogo_filter = QueryFilter {
            category_bits: MOVER_BIT,
            mask_bits: STATIC_BIT | DYNAMIC_BIT,
        };

        let (proxy, translation) = match self.mover_pogo_shape {
            x if x == PogoShape::Point as i32 => (
                make_proxy(&[VEC2_ZERO], 0.0),
                Vec2 {
                    x: 0.0,
                    y: -ray_length,
                },
            ),
            x if x == PogoShape::Circle as i32 => (
                make_proxy(&[VEC2_ZERO], circle_r),
                Vec2 {
                    x: 0.0,
                    y: -ray_length + circle_r,
                },
            ),
            _ => (
                make_proxy(
                    &[
                        Vec2 {
                            x: -segment_offset.x,
                            y: -segment_offset.y,
                        },
                        Vec2 {
                            x: segment_offset.x,
                            y: segment_offset.y,
                        },
                    ],
                    0.0,
                ),
                Vec2 {
                    x: 0.0,
                    y: -ray_length,
                },
            ),
        };

        let origin = offset_pos(self.mover_position, capsule.center1);
        let mut cast_hit = false;
        let mut cast_fraction = 1.0f32;
        let mut cast_point = Pos { x: 0.0, y: 0.0 };
        let mut cast_shape = None;

        world_cast_shape(
            &mut self.world,
            origin,
            &proxy,
            translation,
            pogo_filter,
            |shape_id, point, _normal, fraction| {
                cast_hit = true;
                cast_fraction = fraction;
                cast_point = point;
                cast_shape = Some(shape_id);
                fraction
            },
        );

        if !self.mover_on_ground {
            self.mover_on_ground = cast_hit && self.mover_velocity.y <= 0.01;
        } else {
            self.mover_on_ground = cast_hit;
        }

        self.mover_pogo_draw.clear();
        if !cast_hit {
            self.mover_pogo_velocity = 0.0;
            let delta = translation;
            self.mover_pogo_draw.extend_from_slice(&[
                origin.x,
                origin.y,
                origin.x + delta.x,
                origin.y + delta.y,
                0.0,
                self.mover_pogo_shape as f32,
                if self.mover_pogo_shape == PogoShape::Circle as i32 {
                    circle_r
                } else {
                    segment_offset.x
                },
            ]);
        } else {
            let pogo_current_length = cast_fraction * ray_length;
            let offset = pogo_current_length - pogo_rest_length;
            self.mover_pogo_velocity = spring_damper(
                self.mover_pogo_hertz,
                self.mover_pogo_damping,
                offset,
                self.mover_pogo_velocity,
                time_step,
            );
            let delta = mul_sv(cast_fraction, translation);
            self.mover_pogo_draw.extend_from_slice(&[
                origin.x,
                origin.y,
                origin.x + delta.x,
                origin.y + delta.y,
                1.0,
                self.mover_pogo_shape as f32,
                if self.mover_pogo_shape == PogoShape::Circle as i32 {
                    circle_r
                } else {
                    segment_offset.x
                },
            ]);
            if let Some(shape_id) = cast_shape {
                let body_id = shape_get_body(&self.world, shape_id);
                body_apply_force(
                    &mut self.world,
                    body_id,
                    Vec2 { x: 0.0, y: -50.0 },
                    cast_point,
                    true,
                );
            }
        }

        let target = offset_pos(
            self.mover_position,
            add(
                mul_sv(time_step, self.mover_velocity),
                mul_sv(
                    time_step * self.mover_pogo_velocity,
                    Vec2 { x: 0.0, y: 1.0 },
                ),
            ),
        );

        let collide_filter = QueryFilter {
            category_bits: MOVER_BIT,
            mask_bits: STATIC_BIT | DYNAMIC_BIT | MOVER_BIT,
        };
        let cast_filter = QueryFilter {
            category_bits: MOVER_BIT,
            mask_bits: STATIC_BIT | DYNAMIC_BIT,
        };

        self.mover_total_iterations = 0;
        let tolerance = 0.01f32;

        for _iteration in 0..5 {
            let mut hits: Vec<(box2d_rust::id::ShapeId, box2d_rust::math_functions::Plane)> =
                Vec::with_capacity(PLANE_CAPACITY);
            world_collide_mover(
                &mut self.world,
                self.mover_position,
                &capsule,
                collide_filter,
                |shape_id, plane_result| {
                    if plane_result.hit {
                        hits.push((shape_id, plane_result.plane));
                    }
                    true
                },
            );

            let mut planes: Vec<CollisionPlane> = Vec::with_capacity(PLANE_CAPACITY);
            for (shape_id, plane) in hits {
                if planes.len() >= PLANE_CAPACITY {
                    break;
                }
                let (max_push, clip_velocity) =
                    unpack_plane_user_data(shape_get_user_data(&self.world, shape_id));
                planes.push(CollisionPlane {
                    plane,
                    push_limit: max_push,
                    push: 0.0,
                    clip_velocity,
                });
            }

            let delta_target = sub_pos(target, self.mover_position);
            let result = solve_planes(delta_target, &mut planes);
            self.mover_total_iterations += result.iteration_count;

            let fraction = world_cast_mover(
                &mut self.world,
                self.mover_position,
                &capsule,
                result.translation,
                cast_filter,
            );
            let delta = mul_sv(fraction, result.translation);
            self.mover_position = offset_pos(self.mover_position, delta);
            self.mover_planes = planes;
            self.mover_plane_count = self.mover_planes.len() as i32;

            if length_squared(delta) < tolerance * tolerance {
                break;
            }
        }

        self.mover_velocity = clip_vector(self.mover_velocity, &self.mover_planes);
    }
}
