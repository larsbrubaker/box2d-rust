// Vec2 arithmetic, length/normalize, and closely related helpers.
// Part of the math_functions module.

use super::*;

/// Compute the rotation between two unit vectors
pub fn compute_rotation_between_unit_vectors(v1: Vec2, v2: Vec2) -> Rot {
    debug_assert!(abs_float(1.0 - length(v1)) < 100.0 * f32::EPSILON);
    debug_assert!(abs_float(1.0 - length(v2)) < 100.0 * f32::EPSILON);

    let rot = Rot {
        c: dot(v1, v2),
        s: cross(v1, v2),
    };
    normalize_rot(rot)
}

/// Vector dot product
pub fn dot(a: Vec2, b: Vec2) -> f32 {
    a.x * b.x + a.y * b.y
}

/// Vector cross product. In 2D this yields a scalar.
pub fn cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
pub fn cross_vs(v: Vec2, s: f32) -> Vec2 {
    Vec2 {
        x: s * v.y,
        y: -s * v.x,
    }
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
pub fn cross_sv(s: f32, v: Vec2) -> Vec2 {
    Vec2 {
        x: -s * v.y,
        y: s * v.x,
    }
}

/// Get a left pointing perpendicular vector. Equivalent to cross_sv(1.0, v)
pub fn left_perp(v: Vec2) -> Vec2 {
    Vec2 { x: -v.y, y: v.x }
}

/// Get a right pointing perpendicular vector. Equivalent to cross_vs(v, 1.0)
pub fn right_perp(v: Vec2) -> Vec2 {
    Vec2 { x: v.y, y: -v.x }
}

/// Vector addition
pub fn add(a: Vec2, b: Vec2) -> Vec2 {
    Vec2 {
        x: a.x + b.x,
        y: a.y + b.y,
    }
}

/// Vector subtraction
pub fn sub(a: Vec2, b: Vec2) -> Vec2 {
    Vec2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

/// Vector negation
pub fn neg(a: Vec2) -> Vec2 {
    Vec2 { x: -a.x, y: -a.y }
}

/// Vector linear interpolation
/// <https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/>
pub fn lerp(a: Vec2, b: Vec2, t: f32) -> Vec2 {
    Vec2 {
        x: (1.0 - t) * a.x + t * b.x,
        y: (1.0 - t) * a.y + t * b.y,
    }
}

/// Component-wise multiplication
pub fn mul(a: Vec2, b: Vec2) -> Vec2 {
    Vec2 {
        x: a.x * b.x,
        y: a.y * b.y,
    }
}

/// Multiply a scalar and vector
pub fn mul_sv(s: f32, v: Vec2) -> Vec2 {
    Vec2 {
        x: s * v.x,
        y: s * v.y,
    }
}

/// a + s * b
pub fn mul_add(a: Vec2, s: f32, b: Vec2) -> Vec2 {
    Vec2 {
        x: a.x + s * b.x,
        y: a.y + s * b.y,
    }
}

/// a - s * b
pub fn mul_sub(a: Vec2, s: f32, b: Vec2) -> Vec2 {
    Vec2 {
        x: a.x - s * b.x,
        y: a.y - s * b.y,
    }
}

/// Component-wise absolute vector
pub fn abs(a: Vec2) -> Vec2 {
    Vec2 {
        x: abs_float(a.x),
        y: abs_float(a.y),
    }
}

/// Component-wise minimum vector
pub fn min(a: Vec2, b: Vec2) -> Vec2 {
    Vec2 {
        x: min_float(a.x, b.x),
        y: min_float(a.y, b.y),
    }
}

/// Component-wise maximum vector
pub fn max(a: Vec2, b: Vec2) -> Vec2 {
    Vec2 {
        x: max_float(a.x, b.x),
        y: max_float(a.y, b.y),
    }
}

/// Component-wise clamp vector v into the range [a, b]
pub fn clamp(v: Vec2, a: Vec2, b: Vec2) -> Vec2 {
    Vec2 {
        x: clamp_float(v.x, a.x, b.x),
        y: clamp_float(v.y, a.y, b.y),
    }
}

/// Get the length of this vector (the norm)
pub fn length(v: Vec2) -> f32 {
    (v.x * v.x + v.y * v.y).sqrt()
}

/// Get the distance between two points
pub fn distance(a: Vec2, b: Vec2) -> f32 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    (dx * dx + dy * dy).sqrt()
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
pub fn normalize(v: Vec2) -> Vec2 {
    let length = (v.x * v.x + v.y * v.y).sqrt();
    if length < f32::EPSILON {
        return Vec2 { x: 0.0, y: 0.0 };
    }

    let inv_length = 1.0 / length;
    Vec2 {
        x: inv_length * v.x,
        y: inv_length * v.y,
    }
}

/// Determines if the provided vector is normalized (norm(a) == 1).
pub fn is_normalized(a: Vec2) -> bool {
    let aa = dot(a, a);
    abs_float(1.0 - aa) < 100.0 * f32::EPSILON
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
/// outputs the length.
pub fn get_length_and_normalize(length: &mut f32, v: Vec2) -> Vec2 {
    *length = (v.x * v.x + v.y * v.y).sqrt();
    if *length < f32::EPSILON {
        return Vec2 { x: 0.0, y: 0.0 };
    }

    let inv_length = 1.0 / *length;
    Vec2 {
        x: inv_length * v.x,
        y: inv_length * v.y,
    }
}

/// Normalize rotation
pub fn normalize_rot(q: Rot) -> Rot {
    let mag = (q.s * q.s + q.c * q.c).sqrt();
    let inv_mag = if mag > 0.0 { 1.0 / mag } else { 0.0 };
    Rot {
        c: q.c * inv_mag,
        s: q.s * inv_mag,
    }
}

/// Integrate rotation from angular velocity
/// * `q1` - initial rotation
/// * `delta_angle` - the angular displacement in radians
pub fn integrate_rotation(q1: Rot, delta_angle: f32) -> Rot {
    // dc/dt = -omega * sin(t)
    // ds/dt = omega * cos(t)
    // c2 = c1 - omega * h * s1
    // s2 = s1 + omega * h * c1
    let q2 = Rot {
        c: q1.c - delta_angle * q1.s,
        s: q1.s + delta_angle * q1.c,
    };
    let mag = (q2.s * q2.s + q2.c * q2.c).sqrt();
    let inv_mag = if mag > 0.0 { 1.0 / mag } else { 0.0 };
    Rot {
        c: q2.c * inv_mag,
        s: q2.s * inv_mag,
    }
}

/// Get the length squared of this vector
pub fn length_squared(v: Vec2) -> f32 {
    v.x * v.x + v.y * v.y
}

/// Get the distance squared between points
pub fn distance_squared(a: Vec2, b: Vec2) -> f32 {
    let c = Vec2 {
        x: b.x - a.x,
        y: b.y - a.y,
    };
    c.x * c.x + c.y * c.y
}
