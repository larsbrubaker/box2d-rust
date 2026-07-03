// Scalar helpers: int/float min/max/abs/clamp, deterministic atan2 and cos/sin.
// Part of the math_functions module.

use super::*;

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
