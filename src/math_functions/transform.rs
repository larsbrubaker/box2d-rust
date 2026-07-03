// Rigid transforms, world-position (large-world) operations, and 2x2 matrices.
// Part of the math_functions module.

use super::*;

/// Transform a point (e.g. local space to world space)
pub fn transform_point(t: Transform, p: Vec2) -> Vec2 {
    let x = (t.q.c * p.x - t.q.s * p.y) + t.p.x;
    let y = (t.q.s * p.x + t.q.c * p.y) + t.p.y;

    Vec2 { x, y }
}

/// Inverse transform a point (e.g. world space to local space)
pub fn inv_transform_point(t: Transform, p: Vec2) -> Vec2 {
    let vx = p.x - t.p.x;
    let vy = p.y - t.p.y;
    Vec2 {
        x: t.q.c * vx + t.q.s * vy,
        y: -t.q.s * vx + t.q.c * vy,
    }
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame.
/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
pub fn mul_transforms(a: Transform, b: Transform) -> Transform {
    Transform {
        q: mul_rot(a.q, b.q),
        p: add(rotate_vector(a.q, b.p), a.p),
    }
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
pub fn inv_mul_transforms(a: Transform, b: Transform) -> Transform {
    Transform {
        q: inv_mul_rot(a.q, b.q),
        p: inv_rotate_vector(a.q, sub(b.p, a.p)),
    }
}

/// Convert a vector to a world position. no-op in single precision.
pub fn to_pos(v: Vec2) -> Pos {
    Pos {
        x: v.x as _,
        y: v.y as _,
    }
}

/// Lossy conversion of a world position to a float vector. no-op in single precision.
pub fn to_vec2(p: Pos) -> Vec2 {
    Vec2 {
        x: p.x as f32,
        y: p.y as f32,
    }
}

/// Narrow a world coordinate to float, rounding toward negative infinity. Use with
/// [`round_up_float`] to build a conservative float box that always contains double bounds,
/// where plain rounding far from the origin could clip. nextafterf is an exact IEEE
/// operation, so this is cross-platform deterministic. With large world mode off this is
/// a plain conversion.
#[cfg(feature = "double-precision")]
pub fn round_down_float(x: f64) -> f32 {
    let f = x as f32;
    if f as f64 > x {
        next_after_f32(f, f32::MIN)
    } else {
        f
    }
}

/// Narrow a world coordinate to float, rounding toward positive infinity.
#[cfg(feature = "double-precision")]
pub fn round_up_float(x: f64) -> f32 {
    let f = x as f32;
    if (f as f64) < x {
        next_after_f32(f, f32::MAX)
    } else {
        f
    }
}

#[cfg(not(feature = "double-precision"))]
pub fn round_down_float(x: f64) -> f32 {
    x as f32
}

#[cfg(not(feature = "double-precision"))]
pub fn round_up_float(x: f64) -> f32 {
    x as f32
}

/// C nextafterf: the next representable f32 after `from` in the direction of `to`.
#[cfg(feature = "double-precision")]
fn next_after_f32(from: f32, to: f32) -> f32 {
    if from.is_nan() || to.is_nan() {
        return f32::NAN;
    }
    if from == to {
        return to;
    }
    if from == 0.0 {
        return if to > 0.0 {
            f32::from_bits(1)
        } else {
            -f32::from_bits(1)
        };
    }
    let bits = from.to_bits();
    let next = if (from < to) == (from > 0.0) {
        bits + 1
    } else {
        bits - 1
    };
    f32::from_bits(next)
}

/// a - b, demoted to float. The primary precision boundary operation.
pub fn sub_pos(a: Pos, b: Pos) -> Vec2 {
    Vec2 {
        x: (a.x - b.x) as f32,
        y: (a.y - b.y) as f32,
    }
}

/// p + d
pub fn offset_pos(p: Pos, d: Vec2) -> Pos {
    Pos {
        x: p.x + d.x as PosScalar,
        y: p.y + d.y as PosScalar,
    }
}

/// World position interpolation for sweeps and sampling.
pub fn lerp_position(a: Pos, b: Pos, t: f32) -> Pos {
    Pos {
        x: (1.0 - t) as PosScalar * a.x + t as PosScalar * b.x,
        y: (1.0 - t) as PosScalar * a.y + t as PosScalar * b.y,
    }
}

#[cfg(feature = "double-precision")]
type PosScalar = f64;
#[cfg(not(feature = "double-precision"))]
type PosScalar = f32;

/// Transform a local point to a world position. Rotation in float, translation in double.
pub fn transform_world_point(t: WorldTransform, p: Vec2) -> Pos {
    let rx = t.q.c * p.x - t.q.s * p.y;
    let ry = t.q.s * p.x + t.q.c * p.y;
    Pos {
        x: t.p.x + rx as PosScalar,
        y: t.p.y + ry as PosScalar,
    }
}

/// Transform a world position to a local point. One double subtraction, then float.
pub fn inv_transform_world_point(t: WorldTransform, p: Pos) -> Vec2 {
    let vx = (p.x - t.p.x) as f32;
    let vy = (p.y - t.p.y) as f32;
    Vec2 {
        x: t.q.c * vx + t.q.s * vy,
        y: -t.q.s * vx + t.q.c * vy,
    }
}

/// Relative transform of frame B in frame A.
pub fn inv_mul_world_transforms(a: WorldTransform, b: WorldTransform) -> Transform {
    let d = Vec2 {
        x: (b.p.x - a.p.x) as f32,
        y: (b.p.y - a.p.y) as f32,
    };
    Transform {
        q: inv_mul_rot(a.q, b.q),
        p: inv_rotate_vector(a.q, d),
    }
}

/// Convert a local transform B into world space using world transform A.
pub fn offset_world_transform(a: WorldTransform, b: Transform) -> WorldTransform {
    WorldTransform {
        q: mul_rot(a.q, b.q),
        p: offset_pos(a.p, rotate_vector(a.q, b.p)),
    }
}

/// Shift a world transform into the frame of a base position.
pub fn to_relative_transform(t: WorldTransform, base: Pos) -> Transform {
    Transform {
        q: t.q,
        p: Vec2 {
            x: (t.p.x - base.x) as f32,
            y: (t.p.y - base.y) as f32,
        },
    }
}

/// Promote a float transform to a world transform. Lossless.
pub fn make_world_transform(t: Transform) -> WorldTransform {
    WorldTransform {
        p: to_pos(t.p),
        q: t.q,
    }
}

/// Multiply a 2-by-2 matrix times a 2D vector
pub fn mul_mv(a: Mat22, v: Vec2) -> Vec2 {
    Vec2 {
        x: a.cx.x * v.x + a.cy.x * v.y,
        y: a.cx.y * v.x + a.cy.y * v.y,
    }
}

/// Get the inverse of a 2-by-2 matrix
pub fn get_inverse_22(a: Mat22) -> Mat22 {
    let (m11, m12, m21, m22) = (a.cx.x, a.cy.x, a.cx.y, a.cy.y);
    let mut det = m11 * m22 - m12 * m21;
    if det != 0.0 {
        det = 1.0 / det;
    }

    Mat22 {
        cx: Vec2 {
            x: det * m22,
            y: -det * m21,
        },
        cy: Vec2 {
            x: -det * m12,
            y: det * m11,
        },
    }
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
pub fn solve_22(a: Mat22, b: Vec2) -> Vec2 {
    let (a11, a12, a21, a22) = (a.cx.x, a.cy.x, a.cx.y, a.cy.y);
    let mut det = a11 * a22 - a12 * a21;
    if det != 0.0 {
        det = 1.0 / det;
    }
    Vec2 {
        x: det * (a22 * b.x - a12 * b.y),
        y: det * (a11 * b.y - a21 * b.x),
    }
}
