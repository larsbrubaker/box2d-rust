// Rotation (b2Rot) operations, angle helpers, and vector rotation.
// Part of the math_functions module.

use super::*;

/// Make a rotation using an angle in radians
pub fn make_rot(radians: f32) -> Rot {
    let cs = compute_cos_sin(radians);
    Rot {
        c: cs.cosine,
        s: cs.sine,
    }
}

/// Make a rotation using a unit vector
pub fn make_rot_from_unit_vector(unit_vector: Vec2) -> Rot {
    debug_assert!(is_normalized(unit_vector));
    Rot {
        c: unit_vector.x,
        s: unit_vector.y,
    }
}

/// Is this rotation normalized?
pub fn is_normalized_rot(q: Rot) -> bool {
    // larger tolerance due to failure on mingw 32-bit
    let qq = q.s * q.s + q.c * q.c;
    1.0 - 0.0006 < qq && qq < 1.0 + 0.0006
}

/// Get the inverse of a rotation
pub fn invert_rot(a: Rot) -> Rot {
    Rot { c: a.c, s: -a.s }
}

/// Normalized linear interpolation
/// <https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/>
/// <https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/>
pub fn nlerp(q1: Rot, q2: Rot, t: f32) -> Rot {
    let omt = 1.0 - t;
    let q = Rot {
        c: omt * q1.c + t * q2.c,
        s: omt * q1.s + t * q2.s,
    };

    let mag = (q.s * q.s + q.c * q.c).sqrt();
    let inv_mag = if mag > 0.0 { 1.0 / mag } else { 0.0 };
    Rot {
        c: q.c * inv_mag,
        s: q.s * inv_mag,
    }
}

/// Compute the angular velocity necessary to rotate between two rotations over a given time
/// * `q1` - initial rotation
/// * `q2` - final rotation
/// * `inv_h` - inverse time step
pub fn compute_angular_velocity(q1: Rot, q2: Rot, inv_h: f32) -> f32 {
    // ds/dt = omega * cos(t)
    // dc/dt = -omega * sin(t)
    // s2 = s1 + omega * h * c1
    // c2 = c1 - omega * h * s1

    // omega * h * s1 = c1 - c2
    // omega * h * c1 = s2 - s1
    // omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
    // omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
    // omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
    inv_h * (q2.s * q1.c - q2.c * q1.s)
}

/// Get the angle in radians in the range [-pi, pi]
pub fn rot_get_angle(q: Rot) -> f32 {
    atan2(q.s, q.c)
}

/// Get the x-axis
pub fn rot_get_x_axis(q: Rot) -> Vec2 {
    Vec2 { x: q.c, y: q.s }
}

/// Get the y-axis
pub fn rot_get_y_axis(q: Rot) -> Vec2 {
    Vec2 { x: -q.s, y: q.c }
}

/// Multiply two rotations: q * r
pub fn mul_rot(q: Rot, r: Rot) -> Rot {
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s(q + r) = qs * rc + qc * rs
    // c(q + r) = qc * rc - qs * rs
    Rot {
        s: q.s * r.c + q.c * r.s,
        c: q.c * r.c - q.s * r.s,
    }
}

/// Transpose multiply two rotations: inv(a) * b
/// This rotates a vector local in frame b into a vector local in frame a
pub fn inv_mul_rot(a: Rot, b: Rot) -> Rot {
    // [ ac as] * [bc -bs] = [ac*bc+qs*bs -ac*bs+as*bc]
    // [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
    // s(a - b) = ac * bs - as * bc
    // c(a - b) = ac * bc + as * bs
    Rot {
        s: a.c * b.s - a.s * b.c,
        c: a.c * b.c + a.s * b.s,
    }
}

/// Relative angle between a and b
pub fn relative_angle(a: Rot, b: Rot) -> f32 {
    // sin(b - a) = bs * ac - bc * as
    // cos(b - a) = bc * ac + bs * as
    let s = a.c * b.s - a.s * b.c;
    let c = a.c * b.c + a.s * b.s;
    atan2(s, c)
}

/// Convert any angle into the range [-pi, pi]
pub fn unwind_angle(radians: f32) -> f32 {
    // Assuming this is deterministic
    remainder_f32(radians, 2.0 * PI)
}

/// C remainderf: IEEE 754 remainder, result in [-|y|/2, |y|/2].
/// Rust has no stable std equivalent (`f32::rem_euclid` differs), so implement it directly.
fn remainder_f32(x: f32, y: f32) -> f32 {
    if y == 0.0 || x.is_infinite() || x.is_nan() || y.is_nan() {
        return f32::NAN;
    }

    // Round-half-to-even quotient, computed in f64 to avoid double-rounding error
    // for the magnitudes used here (angle unwinding).
    let q = (x as f64 / y as f64).round_ties_even();
    (x as f64 - q * y as f64) as f32
}

/// Rotate a vector
pub fn rotate_vector(q: Rot, v: Vec2) -> Vec2 {
    Vec2 {
        x: q.c * v.x - q.s * v.y,
        y: q.s * v.x + q.c * v.y,
    }
}

/// Inverse rotate a vector
pub fn inv_rotate_vector(q: Rot, v: Vec2) -> Vec2 {
    Vec2 {
        x: q.c * v.x + q.s * v.y,
        y: -q.s * v.x + q.c * v.y,
    }
}
