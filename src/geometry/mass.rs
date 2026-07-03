// Mass properties for circle, capsule, and polygon from geometry.c.
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::collision::{Capsule, Circle, MassData, Polygon};
use crate::hull::MAX_POLYGON_VERTICES;
use crate::math_functions::{
    add, cross, dot, length, mul_add, normalize, sub, Vec2, PI, VEC2_ZERO,
};

/// Compute mass properties of a circle. (b2ComputeCircleMass)
pub fn compute_circle_mass(shape: &Circle, density: f32) -> MassData {
    let rr = shape.radius * shape.radius;

    let mass = density * PI * rr;
    MassData {
        mass,
        center: shape.center,
        // inertia about the center of mass
        rotational_inertia: mass * 0.5 * rr,
    }
}

/// Compute mass properties of a capsule. (b2ComputeCapsuleMass)
pub fn compute_capsule_mass(shape: &Capsule, density: f32) -> MassData {
    let radius = shape.radius;
    let rr = radius * radius;
    let p1 = shape.center1;
    let p2 = shape.center2;
    let length = length(sub(p2, p1));
    let ll = length * length;

    let circle_mass = density * (PI * radius * radius);
    let box_mass = density * (2.0 * radius * length);

    let mut mass_data = MassData {
        mass: circle_mass + box_mass,
        center: Vec2 {
            x: 0.5 * (p1.x + p2.x),
            y: 0.5 * (p1.y + p2.y),
        },
        rotational_inertia: 0.0,
    };

    // two offset half circles, both halves add up to full circle and each half is offset by half length
    // semi-circle centroid = 4 r / 3 pi
    // Need to apply parallel-axis theorem twice:
    // 1. shift semi-circle centroid to origin
    // 2. shift semi-circle to box end
    // m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
    // See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
    // I verified this formula by computing the convex hull of a 128 vertex capsule

    // half circle centroid
    let lc = 4.0 * radius / (3.0 * PI);

    // half length of rectangular portion of capsule
    let h = 0.5 * length;

    let circle_inertia = circle_mass * (0.5 * rr + h * h + 2.0 * h * lc);
    let box_inertia = box_mass * (4.0 * rr + ll) / 12.0;
    mass_data.rotational_inertia = circle_inertia + box_inertia;

    mass_data
}

/// Compute mass properties of a polygon. (b2ComputePolygonMass)
pub fn compute_polygon_mass(shape: &Polygon, density: f32) -> MassData {
    // Polygon mass, centroid, and inertia.
    // Let rho be the polygon density in mass per unit area.
    // Then:
    // mass = rho * int(dA)
    // centroid.x = (1/mass) * rho * int(x * dA)
    // centroid.y = (1/mass) * rho * int(y * dA)
    // I = rho * int((x*x + y*y) * dA)
    //
    // We can compute these integrals by summing all the integrals
    // for each triangle of the polygon. To evaluate the integral
    // for a single triangle, we make a change of variables to
    // the (u,v) coordinates of the triangle:
    // x = x0 + e1x * u + e2x * v
    // y = y0 + e1y * u + e2y * v
    // where 0 <= u && 0 <= v && u + v <= 1.
    //
    // We integrate u from [0,1-v] and then v from [0,1].
    // We also need to use the Jacobian of the transformation:
    // D = cross(e1, e2)
    //
    // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
    //
    // The rest of the derivation is handled by computer algebra.

    debug_assert!(shape.count > 0);

    if shape.count == 1 {
        let circle = Circle {
            center: shape.vertices[0],
            radius: shape.radius,
        };
        return compute_circle_mass(&circle, density);
    }

    if shape.count == 2 {
        let capsule = Capsule {
            center1: shape.vertices[0],
            center2: shape.vertices[1],
            radius: shape.radius,
        };
        return compute_capsule_mass(&capsule, density);
    }

    let mut vertices = [VEC2_ZERO; MAX_POLYGON_VERTICES];
    let count = shape.count as usize;
    let radius = shape.radius;

    if radius > 0.0 {
        // Approximate mass of rounded polygons by pushing out the vertices.
        let sqrt2 = 1.412f32;
        for (i, vertex) in vertices.iter_mut().enumerate().take(count) {
            let j = if i == 0 { count - 1 } else { i - 1 };
            let n1 = shape.normals[j];
            let n2 = shape.normals[i];

            let mid = normalize(add(n1, n2));
            *vertex = mul_add(shape.vertices[i], sqrt2 * radius, mid);
        }
    } else {
        vertices[..count].copy_from_slice(&shape.vertices[..count]);
    }

    let mut center = VEC2_ZERO;
    let mut area = 0.0f32;
    let mut rotational_inertia = 0.0f32;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    let r = vertices[0];

    let inv3 = 1.0 / 3.0;

    for i in 1..count - 1 {
        // Triangle edges
        let e1 = sub(vertices[i], r);
        let e2 = sub(vertices[i + 1], r);

        let d = cross(e1, e2);

        let triangle_area = 0.5 * d;
        area += triangle_area;

        // Area weighted centroid, r at origin
        center = mul_add(center, triangle_area * inv3, add(e1, e2));

        let (ex1, ey1) = (e1.x, e1.y);
        let (ex2, ey2) = (e2.x, e2.y);

        let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
        let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

        rotational_inertia += (0.25 * inv3 * d) * (intx2 + inty2);
    }

    let mut mass_data = MassData {
        // Total mass
        mass: density * area,
        ..Default::default()
    };

    // Center of mass, shift back from origin at r
    debug_assert!(area > f32::EPSILON);
    let inv_area = 1.0 / area;
    center.x *= inv_area;
    center.y *= inv_area;
    mass_data.center = add(r, center);

    // Inertia tensor relative to the local origin (point s).
    mass_data.rotational_inertia = density * rotational_inertia;

    // Shift inertia to center of mass
    mass_data.rotational_inertia -= mass_data.mass * dot(center, center);

    // If this goes negative we are hosed
    debug_assert!(mass_data.rotational_inertia >= 0.0);

    mass_data
}
