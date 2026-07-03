// Port of box2d-cpp-reference/include/box2d/math_functions.h and src/math_functions.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// Float literals are written with the exact digits of the C source, and Pos math casts
// through PosScalar so the same expression compiles in both precision modes (the cast is
// a no-op in single precision).
#![allow(clippy::excessive_precision)]
#![allow(clippy::unnecessary_cast)]

/// 2D vector
/// This can be used to represent a point or free vector
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CosSin {
    pub cosine: f32,
    pub sine: f32,
}

/// 2D rotation
/// This is similar to using a complex number for rotation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rot {
    pub c: f32,
    pub s: f32,
}

/// A 2D rigid transform
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub p: Vec2,
    pub q: Rot,
}

/// A world position. Double precision in large world mode so coordinates stay accurate far
/// from the origin.
#[cfg(feature = "double-precision")]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pos {
    pub x: f64,
    pub y: f64,
}

/// A world transform with double precision translation and float rotation. Rotation is frame
/// local and never needs the extra range, the same split as Jolt's DMat44.
#[cfg(feature = "double-precision")]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldTransform {
    pub p: Pos,
    pub q: Rot,
}

#[cfg(not(feature = "double-precision"))]
pub type Pos = Vec2;

#[cfg(not(feature = "double-precision"))]
pub type WorldTransform = Transform;

/// A 2-by-2 Matrix stored as columns
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat22 {
    pub cx: Vec2,
    pub cy: Vec2,
}

/// Axis-aligned bounding box
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub lower_bound: Vec2,
    pub upper_bound: Vec2,
}

/// separation = dot(normal, point) - offset
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub normal: Vec2,
    pub offset: f32,
}

/// <https://en.wikipedia.org/wiki/Pi>
/// The C `B2_PI` literal (3.14159265359f) rounds to exactly this f32 value.
pub const PI: f32 = core::f32::consts::PI;

pub const VEC2_ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };
pub const ROT_IDENTITY: Rot = Rot { c: 1.0, s: 0.0 };
pub const TRANSFORM_IDENTITY: Transform = Transform {
    p: Vec2 { x: 0.0, y: 0.0 },
    q: Rot { c: 1.0, s: 0.0 },
};
pub const MAT22_ZERO: Mat22 = Mat22 {
    cx: Vec2 { x: 0.0, y: 0.0 },
    cy: Vec2 { x: 0.0, y: 0.0 },
};

#[cfg(feature = "double-precision")]
pub const POS_ZERO: Pos = Pos { x: 0.0, y: 0.0 };
#[cfg(not(feature = "double-precision"))]
pub const POS_ZERO: Pos = VEC2_ZERO;

#[cfg(feature = "double-precision")]
pub const WORLD_TRANSFORM_IDENTITY: WorldTransform = WorldTransform {
    p: Pos { x: 0.0, y: 0.0 },
    q: Rot { c: 1.0, s: 0.0 },
};
#[cfg(not(feature = "double-precision"))]
pub const WORLD_TRANSFORM_IDENTITY: WorldTransform = TRANSFORM_IDENTITY;

impl Vec2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }
}

impl Rot {
    pub const fn new(c: f32, s: f32) -> Self {
        Rot { c, s }
    }
}

impl Transform {
    pub const fn new(p: Vec2, q: Rot) -> Self {
        Transform { p, q }
    }
}

/// Is this a valid number? Not NaN or infinity.
pub fn is_valid_float(a: f32) -> bool {
    if a.is_nan() {
        return false;
    }

    if a.is_infinite() {
        return false;
    }

    true
}

/// Is this a valid vector? Not NaN or infinity.
pub fn is_valid_vec2(v: Vec2) -> bool {
    if v.x.is_nan() || v.y.is_nan() {
        return false;
    }

    if v.x.is_infinite() || v.y.is_infinite() {
        return false;
    }

    true
}

/// Is this a valid rotation? Not NaN or infinity. Is normalized.
pub fn is_valid_rotation(q: Rot) -> bool {
    if q.s.is_nan() || q.c.is_nan() {
        return false;
    }

    if q.s.is_infinite() || q.c.is_infinite() {
        return false;
    }

    is_normalized_rot(q)
}

/// Is this a valid transform? Not NaN or infinity. Rotation is normalized.
pub fn is_valid_transform(t: Transform) -> bool {
    if !is_valid_vec2(t.p) {
        return false;
    }

    is_valid_rotation(t.q)
}

/// Is this a valid bounding box? Not NaN or infinity. Upper bound greater than or equal to lower bound.
pub fn is_valid_aabb(aabb: Aabb) -> bool {
    let d = sub(aabb.upper_bound, aabb.lower_bound);
    let mut valid = d.x >= 0.0 && d.y >= 0.0;
    valid = valid && is_valid_vec2(aabb.lower_bound) && is_valid_vec2(aabb.upper_bound);
    valid
}

/// Is this a valid plane? Normal is a unit vector. Not NaN or infinity.
pub fn is_valid_plane(a: Plane) -> bool {
    is_valid_vec2(a.normal) && is_normalized(a.normal) && is_valid_float(a.offset)
}

/// Is this a valid world position? Not NaN or infinity.
pub fn is_valid_position(p: Pos) -> bool {
    if p.x.is_nan() || p.y.is_nan() {
        return false;
    }

    if p.x.is_infinite() || p.y.is_infinite() {
        return false;
    }

    true
}

/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
pub fn is_valid_world_transform(t: WorldTransform) -> bool {
    if !is_valid_position(t.p) {
        return false;
    }

    is_valid_rotation(t.q)
}

/// @return the minimum of two integers
pub fn min_int(a: i32, b: i32) -> i32 {
    if a < b {
        a
    } else {
        b
    }
}

/// @return the maximum of two integers
pub fn max_int(a: i32, b: i32) -> i32 {
    if a > b {
        a
    } else {
        b
    }
}

/// @return the absolute value of an integer
pub fn abs_int(a: i32) -> i32 {
    if a < 0 {
        -a
    } else {
        a
    }
}

/// @return an integer clamped between a lower and upper bound
pub fn clamp_int(a: i32, lower: i32, upper: i32) -> i32 {
    if a < lower {
        lower
    } else if a > upper {
        upper
    } else {
        a
    }
}

/// <https://en.wikipedia.org/wiki/Floor_and_ceiling_functions>
pub fn ceiling_int(numerator: i32, denominator: i32) -> i32 {
    debug_assert!(denominator > 0 && numerator >= 0);
    (numerator + denominator - 1) / denominator
}

/// @return the minimum of two floats
/// Matches the C ternary exactly, including NaN propagation (`a < b` is false for NaN).
pub fn min_float(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

/// @return the maximum of two floats
pub fn max_float(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

/// @return the absolute value of a float
pub fn abs_float(a: f32) -> f32 {
    if a < 0.0 {
        -a
    } else {
        a
    }
}

/// @return a float clamped between a lower and upper bound
pub fn clamp_float(a: f32, lower: f32, upper: f32) -> f32 {
    if a < lower {
        lower
    } else if a > upper {
        upper
    } else {
        a
    }
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
/// Accurate to around 0.0023 degrees
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
pub fn atan2(y: f32, x: f32) -> f32 {
    // Added check for (0,0) to match atan2f and avoid NaN
    if x == 0.0 && y == 0.0 {
        return 0.0;
    }

    let ax = abs_float(x);
    let ay = abs_float(y);
    let mx = max_float(ay, ax);
    let mn = min_float(ay, ax);
    let a = mn / mx;

    // Minimax polynomial approximation to atan(a) on [0,1]
    let s = a * a;
    let c = s * a;
    let q = s * s;
    let mut r = 0.024840285 * q + 0.18681418;
    let t = -0.094097948 * q - 0.33213072;
    r = r * s + t;
    r = r * c + a;

    // Map to full circle
    if ay > ax {
        r = 1.57079637 - r;
    }

    if x < 0.0 {
        r = 3.14159274 - r;
    }

    if y < 0.0 {
        r = -r;
    }

    r
}

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
pub fn compute_cos_sin(radians: f32) -> CosSin {
    let x = unwind_angle(radians);
    let pi2 = PI * PI;

    // cosine needs angle in [-pi/2, pi/2]
    let c: f32 = if x < -0.5 * PI {
        let y = x + PI;
        let y2 = y * y;
        -(pi2 - 4.0 * y2) / (pi2 + y2)
    } else if x > 0.5 * PI {
        let y = x - PI;
        let y2 = y * y;
        -(pi2 - 4.0 * y2) / (pi2 + y2)
    } else {
        let y2 = x * x;
        (pi2 - 4.0 * y2) / (pi2 + y2)
    };

    // sine needs angle in [0, pi]
    let s: f32 = if x < 0.0 {
        let y = x + PI;
        -16.0 * y * (PI - y) / (5.0 * pi2 - 4.0 * y * (PI - y))
    } else {
        16.0 * x * (PI - x) / (5.0 * pi2 - 4.0 * x * (PI - x))
    };

    let mag = (s * s + c * c).sqrt();
    let inv_mag = if mag > 0.0 { 1.0 / mag } else { 0.0 };
    CosSin {
        cosine: c * inv_mag,
        sine: s * inv_mag,
    }
}

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

/// Does a fully contain b
pub fn aabb_contains(a: Aabb, b: Aabb) -> bool {
    let mut s = true;
    s = s && a.lower_bound.x <= b.lower_bound.x;
    s = s && a.lower_bound.y <= b.lower_bound.y;
    s = s && b.upper_bound.x <= a.upper_bound.x;
    s = s && b.upper_bound.y <= a.upper_bound.y;
    s
}

/// Get the center of the AABB.
pub fn aabb_center(a: Aabb) -> Vec2 {
    Vec2 {
        x: 0.5 * (a.lower_bound.x + a.upper_bound.x),
        y: 0.5 * (a.lower_bound.y + a.upper_bound.y),
    }
}

/// Get the extents of the AABB (half-widths).
pub fn aabb_extents(a: Aabb) -> Vec2 {
    Vec2 {
        x: 0.5 * (a.upper_bound.x - a.lower_bound.x),
        y: 0.5 * (a.upper_bound.y - a.lower_bound.y),
    }
}

/// Union of two AABBs
pub fn aabb_union(a: Aabb, b: Aabb) -> Aabb {
    Aabb {
        lower_bound: Vec2 {
            x: min_float(a.lower_bound.x, b.lower_bound.x),
            y: min_float(a.lower_bound.y, b.lower_bound.y),
        },
        upper_bound: Vec2 {
            x: max_float(a.upper_bound.x, b.upper_bound.x),
            y: max_float(a.upper_bound.y, b.upper_bound.y),
        },
    }
}

/// Do a and b overlap
pub fn aabb_overlaps(a: Aabb, b: Aabb) -> bool {
    !(b.lower_bound.x > a.upper_bound.x
        || b.lower_bound.y > a.upper_bound.y
        || a.lower_bound.x > b.upper_bound.x
        || a.lower_bound.y > b.upper_bound.y)
}

/// Compute the bounding box of an array of points
pub fn make_aabb(points: &[Vec2], radius: f32) -> Aabb {
    debug_assert!(!points.is_empty());
    let mut a = Aabb {
        lower_bound: points[0],
        upper_bound: points[0],
    };
    for point in &points[1..] {
        a.lower_bound = min(a.lower_bound, *point);
        a.upper_bound = max(a.upper_bound, *point);
    }

    let r = Vec2 {
        x: radius,
        y: radius,
    };
    a.lower_bound = sub(a.lower_bound, r);
    a.upper_bound = add(a.upper_bound, r);

    a
}

/// Signed separation of a point from a plane
pub fn plane_separation(plane: Plane, point: Vec2) -> f32 {
    dot(plane.normal, point) - plane.offset
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += time_step * new_velocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
pub fn spring_damper(
    hertz: f32,
    damping_ratio: f32,
    position: f32,
    velocity: f32,
    time_step: f32,
) -> f32 {
    let omega = 2.0 * PI * hertz;
    let omega_h = omega * time_step;
    (velocity - omega * omega_h * position)
        / (1.0 + 2.0 * damping_ratio * omega_h + omega_h * omega_h)
}

// Operator overloads mirroring the C++ operators in math_functions.h

impl core::ops::AddAssign for Vec2 {
    fn add_assign(&mut self, b: Vec2) {
        self.x += b.x;
        self.y += b.y;
    }
}

impl core::ops::SubAssign for Vec2 {
    fn sub_assign(&mut self, b: Vec2) {
        self.x -= b.x;
        self.y -= b.y;
    }
}

impl core::ops::MulAssign<f32> for Vec2 {
    fn mul_assign(&mut self, b: f32) {
        self.x *= b;
        self.y *= b;
    }
}

impl core::ops::Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2 {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl core::ops::Add for Vec2 {
    type Output = Vec2;
    fn add(self, b: Vec2) -> Vec2 {
        Vec2 {
            x: self.x + b.x,
            y: self.y + b.y,
        }
    }
}

impl core::ops::Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, b: Vec2) -> Vec2 {
        Vec2 {
            x: self.x - b.x,
            y: self.y - b.y,
        }
    }
}

impl core::ops::Mul<Vec2> for f32 {
    type Output = Vec2;
    fn mul(self, b: Vec2) -> Vec2 {
        Vec2 {
            x: self * b.x,
            y: self * b.y,
        }
    }
}

impl core::ops::Mul<f32> for Vec2 {
    type Output = Vec2;
    fn mul(self, b: f32) -> Vec2 {
        Vec2 {
            x: self.x * b,
            y: self.y * b,
        }
    }
}
