// Port of mover.c: the plane solver for character movers. Collision planes
// gathered by `world_collide_mover` are fed to `solve_planes` to find a
// translation that satisfies them, then `clip_vector` removes velocity into
// the touched planes.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::linear_slop;
use crate::math_functions::{
    abs_float, clamp_float, dot, min_float, mul_add, mul_sub, plane_separation, Plane, Vec2,
};

/// A collision plane that can be fed to [`solve_planes`]. Normally assembled
/// by the user from the plane results of `world_collide_mover`.
/// (b2CollisionPlane)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionPlane {
    /// The collision plane between the mover and some shape
    pub plane: Plane,

    /// Setting this to f32::MAX makes the plane as rigid as possible. Lower
    /// values can make the plane collision soft. Usually in meters.
    pub push_limit: f32,

    /// The push on the mover determined by [`solve_planes`]. Usually in
    /// meters.
    pub push: f32,

    /// Indicates if [`clip_vector`] should clip against this plane. Should be
    /// false for soft collision.
    pub clip_velocity: bool,
}

/// Result returned by [`solve_planes`]. (b2PlaneSolverResult)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PlaneSolverResult {
    /// The translation of the mover
    pub translation: Vec2,

    /// The number of iterations used by the plane solver. For diagnostics.
    pub iteration_count: i32,
}

/// Solves the position of a mover that satisfies the given collision planes.
///
/// `target_delta` is the desired movement from the position used to generate
/// the collision planes. (b2SolvePlanes)
pub fn solve_planes(target_delta: Vec2, planes: &mut [CollisionPlane]) -> PlaneSolverResult {
    for plane in planes.iter_mut() {
        plane.push = 0.0;
    }

    let mut delta = target_delta;
    let tolerance = linear_slop();

    let mut iteration = 0;
    while iteration < 20 {
        let mut total_push = 0.0;
        for plane in planes.iter_mut() {
            // Add slop to prevent jitter
            let separation = plane_separation(plane.plane, delta) + linear_slop();

            let push = -separation;

            // Clamp accumulated push
            let accumulated_push = plane.push;
            plane.push = clamp_float(plane.push + push, 0.0, plane.push_limit);
            let push = plane.push - accumulated_push;
            delta = mul_add(delta, push, plane.plane.normal);

            // Track maximum push for convergence
            total_push += abs_float(push);
        }

        if total_push < tolerance {
            break;
        }

        iteration += 1;
    }

    PlaneSolverResult {
        translation: delta,
        iteration_count: iteration,
    }
}

/// Clips the velocity against the given planes so the mover doesn't keep
/// pushing into what it already touched. (b2ClipVector)
pub fn clip_vector(vector: Vec2, planes: &[CollisionPlane]) -> Vec2 {
    let mut v = vector;

    for plane in planes.iter() {
        if plane.push == 0.0 || !plane.clip_velocity {
            continue;
        }

        v = mul_sub(
            v,
            min_float(0.0, dot(v, plane.plane.normal)),
            plane.plane.normal,
        );
    }

    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_functions::normalize;

    // A mover pushed toward a floor plane must slide along it, and clipping
    // must remove the velocity component into the plane.
    #[test]
    fn solve_and_clip_against_floor() {
        // Floor with normal +y, mover 0.05 m above it (separation offset).
        let floor = Plane {
            normal: Vec2 { x: 0.0, y: 1.0 },
            offset: -0.05,
        };
        let mut planes = [CollisionPlane {
            plane: floor,
            push_limit: f32::MAX,
            push: 0.0,
            clip_velocity: true,
        }];

        // Try to move diagonally down into the floor.
        let target = Vec2 { x: 1.0, y: -1.0 };
        let result = solve_planes(target, &mut planes);

        // Horizontal motion survives; vertical penetration is pushed out to
        // roughly the plane surface (within slop).
        assert!((result.translation.x - 1.0).abs() < 1e-6);
        assert!(result.translation.y > -0.1);
        assert!(planes[0].push > 0.0);

        // Velocity into the plane is removed, tangential velocity kept.
        let velocity = Vec2 { x: 2.0, y: -3.0 };
        let clipped = clip_vector(velocity, &planes);
        assert!((clipped.x - 2.0).abs() < 1e-6);
        assert!(clipped.y.abs() < 1e-6);

        // A soft plane (clip_velocity = false) leaves velocity alone.
        planes[0].clip_velocity = false;
        let unclipped = clip_vector(velocity, &planes);
        assert_eq!(unclipped, velocity);
    }

    // The iterative solver converges for a wedge of two planes.
    #[test]
    fn solve_planes_wedge() {
        let mut planes = [
            CollisionPlane {
                plane: Plane {
                    normal: normalize(Vec2 { x: 1.0, y: 1.0 }),
                    offset: 0.0,
                },
                push_limit: f32::MAX,
                push: 0.0,
                clip_velocity: true,
            },
            CollisionPlane {
                plane: Plane {
                    normal: normalize(Vec2 { x: -1.0, y: 1.0 }),
                    offset: 0.0,
                },
                push_limit: f32::MAX,
                push: 0.0,
                clip_velocity: true,
            },
        ];

        // Push straight down into the wedge: the solved translation must not
        // penetrate either plane by more than the slop tolerance.
        let result = solve_planes(Vec2 { x: 0.0, y: -1.0 }, &mut planes);
        for plane in planes.iter() {
            assert!(plane_separation(plane.plane, result.translation) > -0.02);
        }
        assert!(result.iteration_count < 20);
    }
}
