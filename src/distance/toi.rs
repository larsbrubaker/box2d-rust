// Time of impact (b2TimeOfImpact) via the local separating axis method, from
// distance.c. The B2_SNOOP_TOI_COUNTERS profiling globals are not ported.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::get_sweep_transform;
use super::gjk::{find_support, shape_distance};
use super::types::{DistanceInput, ShapeProxy, SimplexCache, Sweep, ToiInput, ToiOutput, ToiState};
use crate::constants::linear_slop;
use crate::hull::MAX_POLYGON_VERTICES;
use crate::math_functions::{
    abs_float, cross_vs, dot, inv_rotate_vector, is_normalized_rot, lerp, max_float, mul_add, neg,
    normalize, rotate_vector, sub, transform_point, Vec2, VEC2_ZERO,
};

enum SeparationType {
    Points,
    FaceA,
    FaceB,
}

struct SeparationFunction<'a> {
    proxy_a: &'a ShapeProxy,
    proxy_b: &'a ShapeProxy,
    sweep_a: Sweep,
    sweep_b: Sweep,
    local_point: Vec2,
    axis: Vec2,
    kind: SeparationType,
}

fn make_separation_function<'a>(
    cache: &SimplexCache,
    proxy_a: &'a ShapeProxy,
    sweep_a: &Sweep,
    proxy_b: &'a ShapeProxy,
    sweep_b: &Sweep,
    t1: f32,
) -> SeparationFunction<'a> {
    let count = cache.count;
    debug_assert!(0 < count && count < 3);

    let xf_a = get_sweep_transform(sweep_a, t1);
    let xf_b = get_sweep_transform(sweep_b, t1);

    if count == 1 {
        let local_point_a = proxy_a.points[cache.index_a[0] as usize];
        let local_point_b = proxy_b.points[cache.index_b[0] as usize];
        let point_a = transform_point(xf_a, local_point_a);
        let point_b = transform_point(xf_b, local_point_b);
        return SeparationFunction {
            proxy_a,
            proxy_b,
            sweep_a: *sweep_a,
            sweep_b: *sweep_b,
            axis: normalize(sub(point_b, point_a)),
            local_point: VEC2_ZERO,
            kind: SeparationType::Points,
        };
    }

    if cache.index_a[0] == cache.index_a[1] {
        // Two points on B and one on A.
        let local_point_b1 = proxy_b.points[cache.index_b[0] as usize];
        let local_point_b2 = proxy_b.points[cache.index_b[1] as usize];

        let mut axis = normalize(cross_vs(sub(local_point_b2, local_point_b1), 1.0));
        let normal = rotate_vector(xf_b.q, axis);

        let local_point = Vec2 {
            x: 0.5 * (local_point_b1.x + local_point_b2.x),
            y: 0.5 * (local_point_b1.y + local_point_b2.y),
        };
        let point_b = transform_point(xf_b, local_point);

        let local_point_a = proxy_a.points[cache.index_a[0] as usize];
        let point_a = transform_point(xf_a, local_point_a);

        let s = dot(sub(point_a, point_b), normal);
        if s < 0.0 {
            axis = neg(axis);
        }
        return SeparationFunction {
            proxy_a,
            proxy_b,
            sweep_a: *sweep_a,
            sweep_b: *sweep_b,
            axis,
            local_point,
            kind: SeparationType::FaceB,
        };
    }

    // Two points on A and one or two points on B.
    let local_point_a1 = proxy_a.points[cache.index_a[0] as usize];
    let local_point_a2 = proxy_a.points[cache.index_a[1] as usize];

    let mut axis = normalize(cross_vs(sub(local_point_a2, local_point_a1), 1.0));
    let normal = rotate_vector(xf_a.q, axis);

    let local_point = Vec2 {
        x: 0.5 * (local_point_a1.x + local_point_a2.x),
        y: 0.5 * (local_point_a1.y + local_point_a2.y),
    };
    let point_a = transform_point(xf_a, local_point);

    let local_point_b = proxy_b.points[cache.index_b[0] as usize];
    let point_b = transform_point(xf_b, local_point_b);

    let s = dot(sub(point_b, point_a), normal);
    if s < 0.0 {
        axis = neg(axis);
    }
    SeparationFunction {
        proxy_a,
        proxy_b,
        sweep_a: *sweep_a,
        sweep_b: *sweep_b,
        axis,
        local_point,
        kind: SeparationType::FaceA,
    }
}

/// Returns (separation, index_a, index_b). (b2FindMinSeparation)
fn find_min_separation(f: &SeparationFunction, t: f32) -> (f32, i32, i32) {
    let xf_a = get_sweep_transform(&f.sweep_a, t);
    let xf_b = get_sweep_transform(&f.sweep_b, t);

    match f.kind {
        SeparationType::Points => {
            let axis_a = inv_rotate_vector(xf_a.q, f.axis);
            let axis_b = inv_rotate_vector(xf_b.q, neg(f.axis));

            let index_a = find_support(f.proxy_a, axis_a);
            let index_b = find_support(f.proxy_b, axis_b);

            let local_point_a = f.proxy_a.points[index_a as usize];
            let local_point_b = f.proxy_b.points[index_b as usize];

            let point_a = transform_point(xf_a, local_point_a);
            let point_b = transform_point(xf_b, local_point_b);

            let separation = dot(sub(point_b, point_a), f.axis);
            (separation, index_a, index_b)
        }

        SeparationType::FaceA => {
            let normal = rotate_vector(xf_a.q, f.axis);
            let point_a = transform_point(xf_a, f.local_point);

            let axis_b = inv_rotate_vector(xf_b.q, neg(normal));

            let index_a = -1;
            let index_b = find_support(f.proxy_b, axis_b);

            let local_point_b = f.proxy_b.points[index_b as usize];
            let point_b = transform_point(xf_b, local_point_b);

            let separation = dot(sub(point_b, point_a), normal);
            (separation, index_a, index_b)
        }

        SeparationType::FaceB => {
            let normal = rotate_vector(xf_b.q, f.axis);
            let point_b = transform_point(xf_b, f.local_point);

            let axis_a = inv_rotate_vector(xf_a.q, neg(normal));

            let index_b = -1;
            let index_a = find_support(f.proxy_a, axis_a);

            let local_point_a = f.proxy_a.points[index_a as usize];
            let point_a = transform_point(xf_a, local_point_a);

            let separation = dot(sub(point_a, point_b), normal);
            (separation, index_a, index_b)
        }
    }
}

/// (b2EvaluateSeparation)
fn evaluate_separation(f: &SeparationFunction, index_a: i32, index_b: i32, t: f32) -> f32 {
    let xf_a = get_sweep_transform(&f.sweep_a, t);
    let xf_b = get_sweep_transform(&f.sweep_b, t);

    match f.kind {
        SeparationType::Points => {
            let local_point_a = f.proxy_a.points[index_a as usize];
            let local_point_b = f.proxy_b.points[index_b as usize];

            let point_a = transform_point(xf_a, local_point_a);
            let point_b = transform_point(xf_b, local_point_b);

            dot(sub(point_b, point_a), f.axis)
        }

        SeparationType::FaceA => {
            let normal = rotate_vector(xf_a.q, f.axis);
            let point_a = transform_point(xf_a, f.local_point);

            let local_point_b = f.proxy_b.points[index_b as usize];
            let point_b = transform_point(xf_b, local_point_b);

            dot(sub(point_b, point_a), normal)
        }

        SeparationType::FaceB => {
            let normal = rotate_vector(xf_b.q, f.axis);
            let point_b = transform_point(xf_b, f.local_point);

            let local_point_a = f.proxy_a.points[index_a as usize];
            let point_a = transform_point(xf_a, local_point_a);

            dot(sub(point_a, point_b), normal)
        }
    }
}

/// Compute the upper bound on time before two shapes penetrate. Time is
/// represented as a fraction between [0, max_fraction]. This uses a swept
/// separating axis and may miss some intermediate, non-tunneling collisions.
/// If you change the time interval, you should call this function again.
/// (b2TimeOfImpact)
///
/// CCD via the local separating axis method. This seeks progression by
/// computing the largest time at which separation is maintained.
pub fn time_of_impact(input: &ToiInput) -> ToiOutput {
    let mut output = ToiOutput {
        state: ToiState::Unknown,
        fraction: input.max_fraction,
        ..Default::default()
    };

    let sweep_a = input.sweep_a;
    let sweep_b = input.sweep_b;
    debug_assert!(is_normalized_rot(sweep_a.q1) && is_normalized_rot(sweep_a.q2));
    debug_assert!(is_normalized_rot(sweep_b.q1) && is_normalized_rot(sweep_b.q2));

    let proxy_a = &input.proxy_a;
    let proxy_b = &input.proxy_b;

    let t_max = input.max_fraction;

    // Setup target distance and tolerance
    let slop = linear_slop();
    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = max_float(slop, total_radius - slop);
    let tolerance = 0.25 * slop;
    debug_assert!(target > tolerance);

    let mut t1 = 0.0f32;
    let k_max_iterations = 20;
    let mut distance_iterations = 0;

    // Prepare input for distance query.
    let mut cache = SimplexCache::default();
    let mut distance_input = DistanceInput {
        proxy_a: input.proxy_a,
        proxy_b: input.proxy_b,
        transform: crate::math_functions::TRANSFORM_IDENTITY,
        use_radii: false,
    };

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    loop {
        // Get the distance between shapes. We can also use the results to get
        // a separating axis.
        let xf_a = get_sweep_transform(&sweep_a, t1);
        let xf_b = get_sweep_transform(&sweep_b, t1);
        distance_input.transform = crate::math_functions::inv_mul_transforms(xf_a, xf_b);
        let distance_output = shape_distance(&distance_input, &mut cache, None);

        // The distance query runs in frame A, project the witness data back to world
        let world_normal = rotate_vector(xf_a.q, distance_output.normal);
        let world_point_a = transform_point(xf_a, distance_output.point_a);
        let world_point_b = transform_point(xf_a, distance_output.point_b);

        distance_iterations += 1;

        // If the shapes are overlapped, we give up on continuous collision.
        if distance_output.distance <= 0.0 {
            // Failure!
            output.state = ToiState::Overlapped;
            output.fraction = 0.0;
            break;
        }

        if distance_output.distance <= target + tolerance {
            // Success!
            output.state = ToiState::Hit;
            // Averaged hit point
            let p_a = mul_add(world_point_a, proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5);
            output.normal = world_normal;
            output.fraction = t1;
            break;
        }

        // Initialize the separating axis.
        let fcn = make_separation_function(&cache, proxy_a, &sweep_a, proxy_b, &sweep_b, t1);

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of
        // vertices.
        let mut done = false;
        let mut t2 = t_max;
        let mut push_back_iterations = 0;
        loop {
            // Find the deepest point at t2. Store the witness point indices.
            let (mut s2, index_a, index_b) = find_min_separation(&fcn, t2);

            // Is the final configuration separated?
            if s2 > target + tolerance {
                // Victory!
                output.state = ToiState::Separated;
                output.fraction = t_max;
                done = true;
                break;
            }

            // Has the separation reached tolerance?
            if s2 > target - tolerance {
                // Advance the sweeps
                t1 = t2;
                break;
            }

            // Compute the initial separation of the witness points.
            let mut s1 = evaluate_separation(&fcn, index_a, index_b, t1);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if s1 < target - tolerance {
                output.state = ToiState::Failed;
                output.fraction = t1;
                done = true;
                break;
            }

            // Check for touching
            if s1 <= target + tolerance {
                // Success! t1 should hold the TOI (could be 0.0).
                output.state = ToiState::Hit;
                // Averaged hit point
                let p_a = mul_add(world_point_a, proxy_a.radius, world_normal);
                let p_b = mul_add(world_point_b, -proxy_b.radius, world_normal);
                output.point = lerp(p_a, p_b, 0.5);
                output.normal = world_normal;
                output.fraction = t1;
                done = true;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            let mut root_iteration_count = 0;
            let (mut a1, mut a2) = (t1, t2);
            loop {
                // Use a mix of false position and bisection.
                let t = if root_iteration_count & 1 == 1 {
                    // False position to improve convergence.
                    a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                } else {
                    // Bisection to guarantee progress.
                    0.5 * (a1 + a2)
                };

                root_iteration_count += 1;

                let s = evaluate_separation(&fcn, index_a, index_b, t);

                // Has the separation reached tolerance?
                if abs_float(s - target) < tolerance {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }

                // Ensure we continue to bracket the root.
                if s > target {
                    a1 = t;
                    s1 = s;
                } else {
                    a2 = t;
                    s2 = s;
                }

                if root_iteration_count == 50 {
                    break;
                }
            }

            push_back_iterations += 1;

            if push_back_iterations == MAX_POLYGON_VERTICES as i32 {
                break;
            }
        }

        if done {
            break;
        }

        if distance_iterations == k_max_iterations {
            // Root finder got stuck. Semi-victory.
            output.state = ToiState::Failed;
            // Averaged hit point
            let p_a = mul_add(world_point_a, proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5);
            output.normal = world_normal;
            output.fraction = t1;
            break;
        }
    }

    output
}
