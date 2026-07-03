// SAT polygon clipper and polygon-vs-polygon manifold from manifold.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{make_capsule_polygon, make_id};
use crate::collision::{LocalManifold, Polygon, Segment};
use crate::constants::{linear_slop, speculative_distance};
use crate::distance::segment_distance;
use crate::math_functions::{
    add, cross_sv, dot, lerp, min_float, mul_add, neg, rotate_vector, sub, transform_point,
    Transform, VEC2_ZERO,
};

/// Polygon clipper used to compute contact points when there are potentially
/// two contact points. (static b2ClipPolygons)
fn clip_polygons(
    poly_a: &Polygon,
    poly_b: &Polygon,
    edge_a: i32,
    edge_b: i32,
    flip: bool,
) -> LocalManifold {
    let mut manifold = LocalManifold::default();

    // reference polygon
    let poly1: &Polygon;
    let (i11, i12): (usize, usize);

    // incident polygon
    let poly2: &Polygon;
    let (i21, i22): (usize, usize);

    if flip {
        poly1 = poly_b;
        poly2 = poly_a;
        i11 = edge_b as usize;
        i12 = if edge_b + 1 < poly_b.count {
            (edge_b + 1) as usize
        } else {
            0
        };
        i21 = edge_a as usize;
        i22 = if edge_a + 1 < poly_a.count {
            (edge_a + 1) as usize
        } else {
            0
        };
    } else {
        poly1 = poly_a;
        poly2 = poly_b;
        i11 = edge_a as usize;
        i12 = if edge_a + 1 < poly_a.count {
            (edge_a + 1) as usize
        } else {
            0
        };
        i21 = edge_b as usize;
        i22 = if edge_b + 1 < poly_b.count {
            (edge_b + 1) as usize
        } else {
            0
        };
    }

    let normal = poly1.normals[i11];

    // Reference edge vertices
    let v11 = poly1.vertices[i11];
    let v12 = poly1.vertices[i12];

    // Incident edge vertices
    let v21 = poly2.vertices[i21];
    let v22 = poly2.vertices[i22];

    let tangent = cross_sv(1.0, normal);

    let lower1 = 0.0;
    let upper1 = dot(sub(v12, v11), tangent);

    // Incident edge points opposite of tangent due to CCW winding
    let upper2 = dot(sub(v21, v11), tangent);
    let lower2 = dot(sub(v22, v11), tangent);

    // Are the segments disjoint?
    if upper2 < lower1 || upper1 < lower2 {
        return manifold;
    }

    let mut v_lower = if lower2 < lower1 && upper2 - lower2 > f32::EPSILON {
        lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2))
    } else {
        v22
    };

    let mut v_upper = if upper2 > upper1 && upper2 - lower2 > f32::EPSILON {
        lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2))
    } else {
        v21
    };

    let separation_lower = dot(sub(v_lower, v11), normal);
    let separation_upper = dot(sub(v_upper, v11), normal);

    let r1 = poly1.radius;
    let r2 = poly2.radius;

    // Put contact points at midpoint, accounting for radii
    v_lower = mul_add(v_lower, 0.5 * (r1 - r2 - separation_lower), normal);
    v_upper = mul_add(v_upper, 0.5 * (r1 - r2 - separation_upper), normal);

    let radius = r1 + r2;

    if !flip {
        manifold.normal = normal;

        {
            let cp = &mut manifold.points[0];
            cp.point = v_lower;
            cp.separation = separation_lower - radius;
            cp.id = make_id(i11 as i32, i22 as i32);
            manifold.point_count += 1;
        }
        {
            let cp = &mut manifold.points[1];
            cp.point = v_upper;
            cp.separation = separation_upper - radius;
            cp.id = make_id(i12 as i32, i21 as i32);
            manifold.point_count += 1;
        }
    } else {
        manifold.normal = neg(normal);

        {
            let cp = &mut manifold.points[0];
            cp.point = v_upper;
            cp.separation = separation_upper - radius;
            cp.id = make_id(i21 as i32, i12 as i32);
            manifold.point_count += 1;
        }
        {
            let cp = &mut manifold.points[1];
            cp.point = v_lower;
            cp.separation = separation_lower - radius;
            cp.id = make_id(i22 as i32, i11 as i32);
            manifold.point_count += 1;
        }
    }

    manifold
}

/// Find the max separation between poly1 and poly2 using edge normals from
/// poly1. Returns (max separation, edge index). (static b2FindMaxSeparation)
fn find_max_separation(poly1: &Polygon, poly2: &Polygon) -> (f32, i32) {
    let count1 = poly1.count as usize;
    let count2 = poly2.count as usize;
    let n1s = &poly1.normals;
    let v1s = &poly1.vertices;
    let v2s = &poly2.vertices;

    let mut best_index = 0i32;
    let mut max_separation = -f32::MAX;
    for i in 0..count1 {
        // Get poly1 normal in frame2.
        let n = n1s[i];
        let v1 = v1s[i];

        // Find the deepest point for normal i.
        let mut si = f32::MAX;
        for v2 in v2s.iter().take(count2) {
            let sij = dot(n, sub(*v2, v1));
            if sij < si {
                si = sij;
            }
        }

        if si > max_separation {
            max_separation = si;
            best_index = i as i32;
        }
    }

    (max_separation, best_index)
}

/// Compute the contact manifold between two polygons. (b2CollidePolygons)
///
/// Due to speculation, every polygon is rounded.
/// Algorithm:
///
/// compute edge separation using the separating axis test (SAT)
/// if (separation > speculation_distance)
///   return
/// find reference and incident edge
/// if separation >= 0.1f * B2_LINEAR_SLOP
///   compute closest points between reference and incident edge
///   if vertices are closest
///      single vertex-vertex contact
///   else
///      clip edges
///   end
/// else
///   clip edges
/// end
pub fn collide_polygons(polygon_a: &Polygon, polygon_b: &Polygon, xf: Transform) -> LocalManifold {
    let origin = polygon_a.vertices[0];
    let slop = linear_slop();
    let speculative = speculative_distance();

    // Shift to the origin in frame A for round-off, a pure translation in A's frame
    let xfs = Transform {
        p: sub(xf.p, origin),
        q: xf.q,
    };

    let mut local_poly_a = Polygon {
        count: polygon_a.count,
        radius: polygon_a.radius,
        ..Default::default()
    };
    local_poly_a.vertices[0] = VEC2_ZERO;
    local_poly_a.normals[0] = polygon_a.normals[0];
    for i in 1..local_poly_a.count as usize {
        local_poly_a.vertices[i] = sub(polygon_a.vertices[i], origin);
        local_poly_a.normals[i] = polygon_a.normals[i];
    }

    // Put polyB in polyA's frame to reduce round-off error
    let mut local_poly_b = Polygon {
        count: polygon_b.count,
        radius: polygon_b.radius,
        ..Default::default()
    };
    for i in 0..local_poly_b.count as usize {
        local_poly_b.vertices[i] = transform_point(xfs, polygon_b.vertices[i]);
        local_poly_b.normals[i] = rotate_vector(xfs.q, polygon_b.normals[i]);
    }

    let (separation_a, mut edge_a) = find_max_separation(&local_poly_a, &local_poly_b);
    let (separation_b, mut edge_b) = find_max_separation(&local_poly_b, &local_poly_a);

    let radius = local_poly_a.radius + local_poly_b.radius;

    if separation_a > speculative + radius || separation_b > speculative + radius {
        return LocalManifold::default();
    }

    // Find incident edge
    let flip;
    if separation_a >= separation_b {
        flip = false;

        let search_direction = local_poly_a.normals[edge_a as usize];

        // Find the incident edge on polyB
        let count = local_poly_b.count as usize;
        let normals = &local_poly_b.normals;
        edge_b = 0;
        let mut min_dot = f32::MAX;
        for (i, normal) in normals.iter().enumerate().take(count) {
            let dot_ = dot(search_direction, *normal);
            if dot_ < min_dot {
                min_dot = dot_;
                edge_b = i as i32;
            }
        }
    } else {
        flip = true;

        let search_direction = local_poly_b.normals[edge_b as usize];

        // Find the incident edge on polyA
        let count = local_poly_a.count as usize;
        let normals = &local_poly_a.normals;
        edge_a = 0;
        let mut min_dot = f32::MAX;
        for (i, normal) in normals.iter().enumerate().take(count) {
            let dot_ = dot(search_direction, *normal);
            if dot_ < min_dot {
                min_dot = dot_;
                edge_a = i as i32;
            }
        }
    }

    let mut manifold = LocalManifold::default();

    // Using slop here to ensure vertex-vertex normal vectors can be safely normalized
    if separation_a > 0.1 * slop || separation_b > 0.1 * slop {
        // Edges are disjoint. Find closest points between reference edge and incident edge
        // Reference edge on polygon A
        let i11 = edge_a as usize;
        let i12 = if edge_a + 1 < local_poly_a.count {
            (edge_a + 1) as usize
        } else {
            0
        };
        let i21 = edge_b as usize;
        let i22 = if edge_b + 1 < local_poly_b.count {
            (edge_b + 1) as usize
        } else {
            0
        };

        let v11 = local_poly_a.vertices[i11];
        let v12 = local_poly_a.vertices[i12];
        let v21 = local_poly_b.vertices[i21];
        let v22 = local_poly_b.vertices[i22];

        let result = segment_distance(v11, v12, v21, v22);
        debug_assert!(result.distance_squared > 0.0);
        let distance = result.distance_squared.sqrt();
        let separation = distance - radius;

        if distance - radius > speculative {
            // This can happen in the vertex-vertex case
            return manifold;
        }

        // Attempt to clip edges
        manifold = clip_polygons(&local_poly_a, &local_poly_b, edge_a, edge_b, flip);

        let mut min_separation = f32::MAX;
        for i in 0..manifold.point_count as usize {
            min_separation = min_float(min_separation, manifold.points[i].separation);
        }

        // Does vertex-vertex have substantially larger separation?
        if separation + 0.1 * slop < min_separation {
            // The four vertex-vertex feature pairs, matching the C branches.
            let feature: Option<(
                crate::math_functions::Vec2,
                crate::math_functions::Vec2,
                usize,
                usize,
            )> = if result.fraction1 == 0.0 && result.fraction2 == 0.0 {
                Some((v11, v21, i11, i21))
            } else if result.fraction1 == 0.0 && result.fraction2 == 1.0 {
                Some((v11, v22, i11, i22))
            } else if result.fraction1 == 1.0 && result.fraction2 == 0.0 {
                Some((v12, v21, i12, i21))
            } else if result.fraction1 == 1.0 && result.fraction2 == 1.0 {
                Some((v12, v22, i12, i22))
            } else {
                None
            };

            if let Some((va, vb, ia, ib)) = feature {
                let mut normal = sub(vb, va);
                let inv_distance = 1.0 / distance;
                normal.x *= inv_distance;
                normal.y *= inv_distance;

                let c1 = mul_add(va, local_poly_a.radius, normal);
                let c2 = mul_add(vb, -local_poly_b.radius, normal);

                manifold.normal = normal;
                manifold.points[0].point = lerp(c1, c2, 0.5);
                manifold.points[0].separation = distance - radius;
                manifold.points[0].id = make_id(ia as i32, ib as i32);
                manifold.point_count = 1;
            }
        }
    } else {
        // Polygons overlap
        manifold = clip_polygons(&local_poly_a, &local_poly_b, edge_a, edge_b, flip);
    }

    // Undo the origin shift so points are in frame A
    for i in 0..manifold.point_count as usize {
        manifold.points[i].point = add(manifold.points[i].point, origin);
    }

    manifold
}

/// Compute the contact manifold between a segment and a polygon.
/// (b2CollideSegmentAndPolygon)
pub fn collide_segment_and_polygon(
    segment_a: &Segment,
    polygon_b: &Polygon,
    xf: Transform,
) -> LocalManifold {
    let polygon_a = make_capsule_polygon(segment_a.point1, segment_a.point2, 0.0);
    collide_polygons(&polygon_a, polygon_b, xf)
}
