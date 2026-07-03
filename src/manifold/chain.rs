// One-sided chain segment manifolds from manifold.c: chain segment vs circle,
// capsule, and polygon, using the Gauss map to avoid ghost collisions.
// See https://box2d.org/posts/2020/06/ghost-collisions/
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{make_capsule_polygon, make_id};
use crate::collision::{Capsule, ChainSegment, Circle, LocalManifold, Polygon};
use crate::constants::{linear_slop, speculative_distance};
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::hull::MAX_POLYGON_VERTICES;
use crate::math_functions::{
    cross, dot, get_length_and_normalize, left_perp, lerp, min_float, mul_add, mul_sv, neg,
    normalize, right_perp, rotate_vector, sub, transform_point, Transform, Vec2,
    TRANSFORM_IDENTITY, VEC2_ZERO,
};

/// Compute the contact manifold between a chain segment and a circle.
/// (b2CollideChainSegmentAndCircle)
pub fn collide_chain_segment_and_circle(
    segment_a: &ChainSegment,
    circle_b: &Circle,
    xf: Transform,
) -> LocalManifold {
    let mut manifold = LocalManifold::default();

    // Compute circle in frame of segment
    let p_b = transform_point(xf, circle_b.center);

    let p1 = segment_a.segment.point1;
    let p2 = segment_a.segment.point2;
    let e = sub(p2, p1);

    // Normal points to the right
    let offset = dot(right_perp(e), sub(p_b, p1));
    if offset < 0.0 {
        // collision is one-sided
        return manifold;
    }

    // Barycentric coordinates
    let u = dot(e, sub(p2, p_b));
    let v = dot(e, sub(p_b, p1));

    let p_a;

    if v <= 0.0 {
        // Behind point1?
        // Is pB in the Voronoi region of the previous edge?
        let prev_edge = sub(p1, segment_a.ghost1);
        let u_prev = dot(prev_edge, sub(p_b, p1));
        if u_prev <= 0.0 {
            return manifold;
        }

        p_a = p1;
    } else if u <= 0.0 {
        // Ahead of point2?
        let next_edge = sub(segment_a.ghost2, p2);
        let v_next = dot(next_edge, sub(p_b, p2));

        // Is pB in the Voronoi region of the next edge?
        if v_next > 0.0 {
            return manifold;
        }

        p_a = p2;
    } else {
        let ee = dot(e, e);
        let pa = Vec2 {
            x: u * p1.x + v * p2.x,
            y: u * p1.y + v * p2.y,
        };
        p_a = if ee > 0.0 { mul_sv(1.0 / ee, pa) } else { p1 };
    }

    let mut distance = 0.0;
    let normal = get_length_and_normalize(&mut distance, sub(p_b, p_a));

    let radius = circle_b.radius;
    let separation = distance - radius;
    if separation > speculative_distance() {
        return manifold;
    }

    let c_a = p_a;
    let c_b = mul_add(p_b, -radius, normal);

    manifold.normal = normal;

    let mp = &mut manifold.points[0];
    mp.point = lerp(c_a, c_b, 0.5);
    mp.separation = separation;
    mp.id = 0;
    manifold.point_count = 1;
    manifold
}

/// Compute the contact manifold between a chain segment and a capsule.
/// (b2CollideChainSegmentAndCapsule)
pub fn collide_chain_segment_and_capsule(
    segment_a: &ChainSegment,
    capsule_b: &Capsule,
    xf: Transform,
    cache: &mut SimplexCache,
) -> LocalManifold {
    let poly_b = make_capsule_polygon(capsule_b.center1, capsule_b.center2, capsule_b.radius);
    collide_chain_segment_and_polygon(segment_a, &poly_b, xf, cache)
}

/// (static b2ClipSegments)
#[allow(clippy::too_many_arguments)]
fn clip_segments(
    a1: Vec2,
    a2: Vec2,
    b1: Vec2,
    b2: Vec2,
    normal: Vec2,
    ra: f32,
    rb: f32,
    id1: u16,
    id2: u16,
) -> LocalManifold {
    let mut manifold = LocalManifold::default();

    let tangent = left_perp(normal);

    // Barycentric coordinates of each point relative to a1 along tangent
    let lower1 = 0.0;
    let upper1 = dot(sub(a2, a1), tangent);

    // Incident edge points opposite of tangent due to CCW winding
    let upper2 = dot(sub(b1, a1), tangent);
    let lower2 = dot(sub(b2, a1), tangent);

    // Do segments overlap?
    if upper2 < lower1 || upper1 < lower2 {
        return manifold;
    }

    let mut v_lower = if lower2 < lower1 && upper2 - lower2 > f32::EPSILON {
        lerp(b2, b1, (lower1 - lower2) / (upper2 - lower2))
    } else {
        b2
    };

    let mut v_upper = if upper2 > upper1 && upper2 - lower2 > f32::EPSILON {
        lerp(b2, b1, (upper1 - lower2) / (upper2 - lower2))
    } else {
        b1
    };

    let separation_lower = dot(sub(v_lower, a1), normal);
    let separation_upper = dot(sub(v_upper, a1), normal);

    // Put contact points at midpoint, accounting for radii
    v_lower = mul_add(v_lower, 0.5 * (ra - rb - separation_lower), normal);
    v_upper = mul_add(v_upper, 0.5 * (ra - rb - separation_upper), normal);

    let radius = ra + rb;

    manifold.normal = normal;
    {
        let cp = &mut manifold.points[0];
        cp.point = v_lower;
        cp.separation = separation_lower - radius;
        cp.id = id1;
    }
    {
        let cp = &mut manifold.points[1];
        cp.point = v_upper;
        cp.separation = separation_upper - radius;
        cp.id = id2;
    }

    manifold.point_count = 2;

    manifold
}

/// (enum b2NormalType)
#[derive(PartialEq, Eq, Clone, Copy)]
enum NormalType {
    /// The normal points in a direction that is non-smooth relative to a
    /// convex vertex and should be skipped.
    Skip,
    /// The normal points in a direction that is smooth relative to a convex
    /// vertex and should be used for collision.
    Admit,
    /// The normal is in a region of a concave vertex and should be snapped to
    /// the segment normal.
    Snap,
}

/// (struct b2ChainSegmentParams)
struct ChainSegmentParams {
    edge1: Vec2,
    normal0: Vec2,
    normal2: Vec2,
    convex1: bool,
    convex2: bool,
}

/// Evaluate Gauss map. (static b2ClassifyNormal)
fn classify_normal(params: &ChainSegmentParams, normal: Vec2) -> NormalType {
    let sin_tol = 0.01;

    if dot(normal, params.edge1) <= 0.0 {
        // Normal points towards the segment tail
        if params.convex1 {
            if cross(normal, params.normal0) > sin_tol {
                return NormalType::Skip;
            }

            NormalType::Admit
        } else {
            NormalType::Snap
        }
    } else {
        // Normal points towards segment head
        if params.convex2 {
            if cross(params.normal2, normal) > sin_tol {
                return NormalType::Skip;
            }

            NormalType::Admit
        } else {
            NormalType::Snap
        }
    }
}

/// Compute the contact manifold between a chain segment and a polygon.
/// (b2CollideChainSegmentAndPolygon)
pub fn collide_chain_segment_and_polygon(
    segment_a: &ChainSegment,
    polygon_b: &Polygon,
    xf: Transform,
    cache: &mut SimplexCache,
) -> LocalManifold {
    let mut manifold = LocalManifold::default();

    let centroid_b = transform_point(xf, polygon_b.centroid);
    let radius_b = polygon_b.radius;

    let p1 = segment_a.segment.point1;
    let p2 = segment_a.segment.point2;

    let edge1 = normalize(sub(p2, p1));

    let convex_tol = 0.01;
    let edge0 = normalize(sub(p1, segment_a.ghost1));
    let edge2 = normalize(sub(segment_a.ghost2, p2));

    let smooth_params = ChainSegmentParams {
        edge1,
        normal0: right_perp(edge0),
        convex1: cross(edge0, edge1) >= convex_tol,
        normal2: right_perp(edge2),
        convex2: cross(edge1, edge2) >= convex_tol,
    };

    // Normal points to the right
    let normal1 = right_perp(edge1);
    let behind1 = dot(normal1, sub(centroid_b, p1)) < 0.0;
    let mut behind0 = true;
    let mut behind2 = true;
    if smooth_params.convex1 {
        behind0 = dot(smooth_params.normal0, sub(centroid_b, p1)) < 0.0;
    }

    if smooth_params.convex2 {
        behind2 = dot(smooth_params.normal2, sub(centroid_b, p2)) < 0.0;
    }

    if behind1 && behind0 && behind2 {
        // one-sided collision
        return manifold;
    }

    // Get polygonB in frameA
    let count = polygon_b.count as usize;
    let mut vertices = [VEC2_ZERO; MAX_POLYGON_VERTICES];
    let mut normals = [VEC2_ZERO; MAX_POLYGON_VERTICES];
    for i in 0..count {
        vertices[i] = transform_point(xf, polygon_b.vertices[i]);
        normals[i] = rotate_vector(xf.q, polygon_b.normals[i]);
    }

    // Distance doesn't work correctly with partial polygons
    let input = DistanceInput {
        proxy_a: make_proxy(&[segment_a.segment.point1, segment_a.segment.point2], 0.0),
        proxy_b: make_proxy(&vertices[..count], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let output = shape_distance(&input, cache, None);

    if output.distance > radius_b + speculative_distance() {
        return manifold;
    }

    // Snap concave normals for partial polygon
    let n0 = if smooth_params.convex1 {
        smooth_params.normal0
    } else {
        normal1
    };
    let n2 = if smooth_params.convex2 {
        smooth_params.normal2
    } else {
        normal1
    };

    // Index of incident vertex on polygon
    let mut incident_index: i32 = -1;
    let mut incident_normal: i32 = -1;

    if !behind1 && output.distance > 0.1 * linear_slop() {
        // The closest features may be two vertices or an edge and a vertex even
        // when there should be face contact

        if cache.count == 1 {
            // vertex-vertex collision
            let p_a = output.point_a;
            let p_b = output.point_b;

            let normal = normalize(sub(p_b, p_a));

            match classify_normal(&smooth_params, normal) {
                NormalType::Skip => {
                    return manifold;
                }
                NormalType::Admit => {
                    manifold.normal = normal;
                    let cp = &mut manifold.points[0];
                    cp.point = p_a;
                    cp.separation = output.distance - radius_b;
                    cp.id = make_id(cache.index_a[0] as i32, cache.index_b[0] as i32);
                    manifold.point_count = 1;
                    return manifold;
                }
                NormalType::Snap => {
                    // fall through
                    incident_index = cache.index_b[0] as i32;
                }
            }
        } else {
            // vertex-edge collision
            debug_assert!(cache.count == 2);

            let ia1 = cache.index_a[0] as i32;
            let ia2 = cache.index_a[1] as i32;
            let mut ib1 = cache.index_b[0] as usize;
            let mut ib2 = cache.index_b[1] as usize;

            if ia1 == ia2 {
                // 1 point on A, expect 2 points on B
                debug_assert!(ib1 != ib2);

                // Find polygon normal most aligned with vector between closest
                // points. This effectively sorts ib1 and ib2
                let mut normal_b = sub(output.point_a, output.point_b);
                let dot1 = dot(normal_b, normals[ib1]);
                let dot2 = dot(normal_b, normals[ib2]);
                let ib = if dot1 > dot2 { ib1 } else { ib2 };

                // Use accurate normal
                normal_b = normals[ib];

                match classify_normal(&smooth_params, neg(normal_b)) {
                    NormalType::Skip => {
                        return manifold;
                    }
                    NormalType::Admit => {
                        // Get polygon edge associated with normal
                        ib1 = ib;
                        ib2 = if ib < count - 1 { ib + 1 } else { 0 };

                        let b1 = vertices[ib1];
                        let b2 = vertices[ib2];

                        // Find incident segment vertex
                        let dot1 = dot(normal_b, sub(p1, b1));
                        let dot2 = dot(normal_b, sub(p2, b1));

                        if dot1 < dot2 {
                            if dot(n0, normal_b) < dot(normal1, normal_b) {
                                // Neighbor is incident
                                return manifold;
                            }
                        } else if dot(n2, normal_b) < dot(normal1, normal_b) {
                            // Neighbor is incident
                            return manifold;
                        }

                        manifold = clip_segments(
                            b1,
                            b2,
                            p1,
                            p2,
                            normal_b,
                            radius_b,
                            0.0,
                            make_id(ib1 as i32, 1),
                            make_id(ib2 as i32, 0),
                        );

                        debug_assert!(manifold.point_count == 0 || manifold.point_count == 2);
                        if manifold.point_count == 2 {
                            manifold.normal = neg(normal_b);
                        }
                        return manifold;
                    }
                    NormalType::Snap => {
                        // fall through
                        incident_normal = ib as i32;
                    }
                }
            } else {
                // Get index of incident polygonB vertex
                let dot1 = dot(normal1, sub(vertices[ib1], p1));
                let dot2 = dot(normal1, sub(vertices[ib2], p2));
                incident_index = if dot1 < dot2 { ib1 as i32 } else { ib2 as i32 };
            }
        }
    } else {
        // SAT edge normal
        let mut edge_separation = f32::MAX;

        for (i, vertex) in vertices.iter().enumerate().take(count) {
            let s = dot(normal1, sub(*vertex, p1));
            if s < edge_separation {
                edge_separation = s;
                incident_index = i as i32;
            }
        }

        // Check convex neighbor for edge separation
        if smooth_params.convex1 {
            let mut s0 = f32::MAX;

            for vertex in vertices.iter().take(count) {
                let s = dot(smooth_params.normal0, sub(*vertex, p1));
                if s < s0 {
                    s0 = s;
                }
            }

            if s0 > edge_separation {
                edge_separation = s0;

                // Indicate neighbor owns edge separation
                incident_index = -1;
            }
        }

        // Check convex neighbor for edge separation
        if smooth_params.convex2 {
            let mut s2 = f32::MAX;

            for vertex in vertices.iter().take(count) {
                let s = dot(smooth_params.normal2, sub(*vertex, p2));
                if s < s2 {
                    s2 = s;
                }
            }

            if s2 > edge_separation {
                edge_separation = s2;

                // Indicate neighbor owns edge separation
                incident_index = -1;
            }
        }

        // SAT polygon normals
        let mut polygon_separation = -f32::MAX;
        let mut reference_index: i32 = -1;

        for i in 0..count {
            let n = normals[i];

            if classify_normal(&smooth_params, neg(n)) != NormalType::Admit {
                continue;
            }

            let p = vertices[i];
            let s = min_float(dot(n, sub(p2, p)), dot(n, sub(p1, p)));

            if s > polygon_separation {
                polygon_separation = s;
                reference_index = i as i32;
            }
        }

        if polygon_separation > edge_separation {
            let ia1 = reference_index as usize;
            let ia2 = if ia1 < count - 1 { ia1 + 1 } else { 0 };
            let a1 = vertices[ia1];
            let a2 = vertices[ia2];

            let n = normals[ia1];

            let dot1 = dot(n, sub(p1, a1));
            let dot2 = dot(n, sub(p2, a1));

            if dot1 < dot2 {
                if dot(n0, n) < dot(normal1, n) {
                    // Neighbor is incident
                    return manifold;
                }
            } else if dot(n2, n) < dot(normal1, n) {
                // Neighbor is incident
                return manifold;
            }

            manifold = clip_segments(
                a1,
                a2,
                p1,
                p2,
                normals[ia1],
                radius_b,
                0.0,
                make_id(ia1 as i32, 1),
                make_id(ia2 as i32, 0),
            );

            debug_assert!(manifold.point_count == 0 || manifold.point_count == 2);
            if manifold.point_count == 2 {
                manifold.normal = neg(normals[ia1]);
            }

            return manifold;
        }

        if incident_index == -1 {
            // neighboring segment is the separating axis
            return manifold;
        }

        // fall through segment normal axis
    }

    debug_assert!(incident_normal != -1 || incident_index != -1);

    // Segment normal

    // Find incident polygon normal: normal adjacent to deepest vertex that is
    // most anti-parallel to segment normal
    let (b1, b2, ib1, ib2);

    if incident_normal != -1 {
        ib1 = incident_normal as usize;
        ib2 = if ib1 < count - 1 { ib1 + 1 } else { 0 };
        b1 = vertices[ib1];
        b2 = vertices[ib2];
    } else {
        let i2 = incident_index as usize;
        let i1 = if i2 > 0 { i2 - 1 } else { count - 1 };
        let d1 = dot(normal1, normals[i1]);
        let d2 = dot(normal1, normals[i2]);
        if d1 < d2 {
            ib1 = i1;
            ib2 = i2;
        } else {
            ib1 = i2;
            ib2 = if i2 < count - 1 { i2 + 1 } else { 0 };
        }
        b1 = vertices[ib1];
        b2 = vertices[ib2];
    }

    manifold = clip_segments(
        p1,
        p2,
        b1,
        b2,
        normal1,
        0.0,
        radius_b,
        make_id(0, ib2 as i32),
        make_id(1, ib1 as i32),
    );

    debug_assert!(manifold.point_count == 0 || manifold.point_count == 2);

    manifold
}
