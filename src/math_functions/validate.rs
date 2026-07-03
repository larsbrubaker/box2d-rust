// Validity checks (is_valid_*). Part of the math_functions module.

use super::*;

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
