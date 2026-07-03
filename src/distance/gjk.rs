// GJK distance (b2ShapeDistance) and the simplex machinery from distance.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::types::{DistanceInput, DistanceOutput, ShapeProxy, Simplex, SimplexCache};
use crate::math_functions::{
    add, cross, cross_sv, distance, dot, is_normalized, max_float, mul_add, mul_sub, neg,
    normalize, sub, transform_point, Vec2, VEC2_ZERO,
};

pub(crate) fn weight2(a1: f32, w1: Vec2, a2: f32, w2: Vec2) -> Vec2 {
    Vec2 {
        x: a1 * w1.x + a2 * w2.x,
        y: a1 * w1.y + a2 * w2.y,
    }
}

fn weight3(a1: f32, w1: Vec2, a2: f32, w2: Vec2, a3: f32, w3: Vec2) -> Vec2 {
    Vec2 {
        x: a1 * w1.x + a2 * w2.x + a3 * w3.x,
        y: a1 * w1.y + a2 * w2.y + a3 * w3.y,
    }
}

pub(crate) fn find_support(proxy: &ShapeProxy, direction: Vec2) -> i32 {
    let points = &proxy.points;
    let count = proxy.count;

    let mut best_index = 0;
    let mut best_value = dot(points[0], direction);
    for i in 1..count {
        let value = dot(points[i as usize], direction);
        if value > best_value {
            best_index = i;
            best_value = value;
        }
    }

    best_index
}

fn make_simplex_from_cache(
    cache: &SimplexCache,
    proxy_a: &ShapeProxy,
    proxy_b: &ShapeProxy,
) -> Simplex {
    debug_assert!(cache.count <= 3);
    let mut s = Simplex {
        count: cache.count as i32,
        ..Default::default()
    };

    // Copy data from cache.
    for i in 0..s.count {
        let v = s.vertex_mut(i);
        v.index_a = cache.index_a[i as usize] as i32;
        v.index_b = cache.index_b[i as usize] as i32;
        v.w_a = proxy_a.points[v.index_a as usize];
        v.w_b = proxy_b.points[v.index_b as usize];
        v.w = sub(v.w_a, v.w_b);

        // invalid
        v.a = -1.0;
    }

    // If the cache is empty or invalid ...
    if s.count == 0 {
        let v = s.vertex_mut(0);
        v.index_a = 0;
        v.index_b = 0;
        v.w_a = proxy_a.points[0];
        v.w_b = proxy_b.points[0];
        v.w = sub(v.w_a, v.w_b);
        v.a = 1.0;
        s.count = 1;
    }

    s
}

fn make_simplex_cache(simplex: &Simplex) -> SimplexCache {
    let mut cache = SimplexCache {
        count: simplex.count as u16,
        ..Default::default()
    };
    for i in 0..simplex.count {
        cache.index_a[i as usize] = simplex.vertex(i).index_a as u8;
        cache.index_b[i as usize] = simplex.vertex(i).index_b as u8;
    }

    cache
}

pub(crate) fn compute_witness_points(s: &Simplex) -> (Vec2, Vec2) {
    match s.count {
        1 => (s.v1.w_a, s.v1.w_b),

        2 => (
            weight2(s.v1.a, s.v1.w_a, s.v2.a, s.v2.w_a),
            weight2(s.v1.a, s.v1.w_b, s.v2.a, s.v2.w_b),
        ),

        3 => {
            let a = weight3(s.v1.a, s.v1.w_a, s.v2.a, s.v2.w_a, s.v3.a, s.v3.w_a);
            // C: todo why are these not equal?
            // b = weight3(s.v1.a, s.v1.w_b, s.v2.a, s.v2.w_b, s.v3.a, s.v3.w_b);
            (a, a)
        }

        _ => {
            debug_assert!(false);
            (VEC2_ZERO, VEC2_ZERO)
        }
    }
}

// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
//
// returns a vector that points towards the origin
pub(crate) fn solve_simplex2(s: &mut Simplex) -> Vec2 {
    let w1 = s.v1.w;
    let w2 = s.v2.w;
    let e12 = sub(w2, w1);

    // w1 region
    let d12_2 = -dot(w1, e12);
    if d12_2 <= 0.0 {
        // a2 <= 0, so we clamp it to 0
        s.v1.a = 1.0;
        s.count = 1;
        return neg(w1);
    }

    // w2 region
    let d12_1 = dot(w2, e12);
    if d12_1 <= 0.0 {
        // a1 <= 0, so we clamp it to 0
        s.v2.a = 1.0;
        s.count = 1;
        s.v1 = s.v2;
        return neg(w2);
    }

    // Must be in e12 region.
    let inv_d12 = 1.0 / (d12_1 + d12_2);
    s.v1.a = d12_1 * inv_d12;
    s.v2.a = d12_2 * inv_d12;
    s.count = 2;
    cross_sv(cross(add(w1, w2), e12), e12)
}

pub(crate) fn solve_simplex3(s: &mut Simplex) -> Vec2 {
    let w1 = s.v1.w;
    let w2 = s.v2.w;
    let w3 = s.v3.w;

    // Edge12
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    let e12 = sub(w2, w1);
    let w1e12 = dot(w1, e12);
    let w2e12 = dot(w2, e12);
    let d12_1 = w2e12;
    let d12_2 = -w1e12;

    // Edge13
    // [1      1     ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    let e13 = sub(w3, w1);
    let w1e13 = dot(w1, e13);
    let w3e13 = dot(w3, e13);
    let d13_1 = w3e13;
    let d13_2 = -w1e13;

    // Edge23
    // [1      1     ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    let e23 = sub(w3, w2);
    let w2e23 = dot(w2, e23);
    let w3e23 = dot(w3, e23);
    let d23_1 = w3e23;
    let d23_2 = -w2e23;

    // Triangle123
    let n123 = cross(e12, e13);

    let d123_1 = n123 * cross(w2, w3);
    let d123_2 = n123 * cross(w3, w1);
    let d123_3 = n123 * cross(w1, w2);

    // w1 region
    if d12_2 <= 0.0 && d13_2 <= 0.0 {
        s.v1.a = 1.0;
        s.count = 1;
        return neg(w1);
    }

    // e12
    if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
        let inv_d12 = 1.0 / (d12_1 + d12_2);
        s.v1.a = d12_1 * inv_d12;
        s.v2.a = d12_2 * inv_d12;
        s.count = 2;
        return cross_sv(cross(add(w1, w2), e12), e12);
    }

    // e13
    if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
        let inv_d13 = 1.0 / (d13_1 + d13_2);
        s.v1.a = d13_1 * inv_d13;
        s.v3.a = d13_2 * inv_d13;
        s.count = 2;
        s.v2 = s.v3;
        return cross_sv(cross(add(w1, w3), e13), e13);
    }

    // w2 region
    if d12_1 <= 0.0 && d23_2 <= 0.0 {
        s.v2.a = 1.0;
        s.count = 1;
        s.v1 = s.v2;
        return neg(w2);
    }

    // w3 region
    if d13_1 <= 0.0 && d23_1 <= 0.0 {
        s.v3.a = 1.0;
        s.count = 1;
        s.v1 = s.v3;
        return neg(w3);
    }

    // e23
    if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
        let inv_d23 = 1.0 / (d23_1 + d23_2);
        s.v2.a = d23_1 * inv_d23;
        s.v3.a = d23_2 * inv_d23;
        s.count = 2;
        s.v1 = s.v3;
        return cross_sv(cross(add(w2, w3), e23), e23);
    }

    // Must be in triangle123
    let inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
    s.v1.a = d123_1 * inv_d123;
    s.v2.a = d123_2 * inv_d123;
    s.v3.a = d123_3 * inv_d123;
    s.count = 3;

    // No search direction
    VEC2_ZERO
}

/// Compute the closest points between two shapes represented as point clouds.
/// The cache is input/output: on the first call set `SimplexCache::count` to
/// zero. (b2ShapeDistance)
///
/// The underlying GJK algorithm may be debugged by passing in a simplex buffer;
/// pass `None` normally. The C version only records intermediate simplexes in
/// debug builds; this port records them whenever a buffer is provided.
///
/// Uses GJK for computing the distance between convex shapes.
/// <https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf>
pub fn shape_distance(
    input: &DistanceInput,
    cache: &mut SimplexCache,
    mut simplexes: Option<&mut [Simplex]>,
) -> DistanceOutput {
    debug_assert!(input.proxy_a.count > 0 && input.proxy_b.count > 0);
    debug_assert!(input.proxy_a.radius >= 0.0);
    debug_assert!(input.proxy_b.radius >= 0.0);

    let mut output = DistanceOutput::default();

    let proxy_a = &input.proxy_a;

    // Get proxyB in frame A to avoid further transforms in the main loop.
    // This is still a performance gain at 8 points.
    let mut local_proxy_b = ShapeProxy {
        count: input.proxy_b.count,
        radius: input.proxy_b.radius,
        ..Default::default()
    };
    for i in 0..local_proxy_b.count as usize {
        local_proxy_b.points[i] = transform_point(input.transform, input.proxy_b.points[i]);
    }

    // Initialize the simplex.
    let mut simplex = make_simplex_from_cache(cache, proxy_a, &local_proxy_b);

    let mut simplex_index = 0;
    if let Some(buffer) = simplexes.as_mut() {
        if simplex_index < buffer.len() {
            buffer[simplex_index] = simplex;
            simplex_index += 1;
        }
    }

    let mut non_unit_normal = VEC2_ZERO;

    // These store the vertices of the last simplex so that we can check for
    // duplicates and prevent cycling.
    let mut save_a = [0i32; 3];
    let mut save_b = [0i32; 3];

    // Main iteration loop. All computations are done in frame A.
    let max_iterations = 20;
    let mut iteration = 0;
    while iteration < max_iterations {
        // Copy simplex so we can identify duplicates.
        let save_count = simplex.count;
        for i in 0..save_count {
            save_a[i as usize] = simplex.vertex(i).index_a;
            save_b[i as usize] = simplex.vertex(i).index_b;
        }

        let d = match simplex.count {
            1 => neg(simplex.v1.w),
            2 => solve_simplex2(&mut simplex),
            3 => solve_simplex3(&mut simplex),
            _ => {
                debug_assert!(false);
                VEC2_ZERO
            }
        };

        // If we have 3 points, then the origin is in the corresponding triangle.
        if simplex.count == 3 {
            // Overlap
            let (local_point_a, local_point_b) = compute_witness_points(&simplex);
            output.point_a = local_point_a;
            output.point_b = local_point_b;
            return output;
        }

        if let Some(buffer) = simplexes.as_mut() {
            if simplex_index < buffer.len() {
                buffer[simplex_index] = simplex;
                simplex_index += 1;
            }
        }

        // Ensure the search direction is numerically fit.
        if dot(d, d) < f32::EPSILON * f32::EPSILON {
            // This is unlikely but could lead to bad cycling.
            // The branch predictor seems to make this check have low cost.

            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.

            // Must return overlap due to invalid normal.
            let (local_point_a, local_point_b) = compute_witness_points(&simplex);
            output.point_a = local_point_a;
            output.point_b = local_point_b;
            return output;
        }

        // Save the normal
        non_unit_normal = d;

        // Compute a tentative new simplex vertex using support points.
        // support = support(a, d) - support(b, -d)
        let index_a = find_support(proxy_a, d);
        let index_b = find_support(&local_proxy_b, neg(d));
        let vertex = simplex.vertex_mut(simplex.count);
        vertex.index_a = index_a;
        vertex.w_a = proxy_a.points[index_a as usize];
        vertex.index_b = index_b;
        vertex.w_b = local_proxy_b.points[index_b as usize];
        vertex.w = sub(vertex.w_a, vertex.w_b);

        // Iteration count is equated to the number of support point calls.
        iteration += 1;

        // Check for duplicate support points. This is the main termination criteria.
        let mut duplicate = false;
        for i in 0..save_count {
            if index_a == save_a[i as usize] && index_b == save_b[i as usize] {
                duplicate = true;
                break;
            }
        }

        // If we found a duplicate support point we must exit to avoid cycling.
        if duplicate {
            break;
        }

        // New vertex is valid and needed.
        simplex.count += 1;
    }

    if let Some(buffer) = simplexes.as_mut() {
        if simplex_index < buffer.len() {
            buffer[simplex_index] = simplex;
            simplex_index += 1;
        }
    }

    // Prepare output in frame A
    let normal = normalize(non_unit_normal);
    debug_assert!(is_normalized(normal));

    let (local_point_a, local_point_b) = compute_witness_points(&simplex);
    output.normal = normal;
    output.distance = distance(local_point_a, local_point_b);
    output.point_a = local_point_a;
    output.point_b = local_point_b;
    output.iterations = iteration;
    output.simplex_count = simplex_index as i32;

    // Cache the simplex
    *cache = make_simplex_cache(&simplex);

    // Apply radii if requested
    if input.use_radii {
        let radius_a = input.proxy_a.radius;
        let radius_b = input.proxy_b.radius;
        output.distance = max_float(0.0, output.distance - radius_a - radius_b);

        // Keep closest points on perimeter even if overlapped, this way the
        // points move smoothly.
        output.point_a = mul_add(output.point_a, radius_a, normal);
        output.point_b = mul_sub(output.point_b, radius_b, normal);
    }

    output
}
