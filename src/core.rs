// Port of the surviving behavior from box2d-cpp-reference/src/core.c, core.h,
// include/box2d/base.h, and src/ctz.h.
//
// core.c is largely an allocator / threading / timing shim (b2Alloc, b2Free,
// aligned allocation, mutex/semaphore/thread wrappers). Rust covers all of that
// natively through Vec, ownership, and std::thread, so those pieces are not
// ported — they carry no algorithmic behavior. What remains here is the pieces
// that do carry behavior: the runtime length-unit scale, the version/precision
// query functions, the deterministic djb2 hash, and the bit-twiddling helpers.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use core::sync::atomic::{AtomicU32, Ordering};

/// Used to indicate an unset or invalid index value. (base.h: B2_NULL_INDEX)
pub const NULL_INDEX: i32 = -1;

/// Use to validate definitions. (core.h: B2_SECRET_COOKIE)
pub const SECRET_COOKIE: i32 = 1152023;

/// Initial value for the djb2 hash. (base.h: B2_HASH_INIT)
pub const HASH_INIT: u32 = 5381;

// The length-unit scale is a single global that the user sets once at startup.
// C stores it as a plain `static float`; we store the bit pattern in an atomic
// so the global is sound under Rust's threading rules. The observable value is
// identical. 0x3F80_0000 is the bit pattern of 1.0f32.
static LENGTH_UNITS_PER_METER_BITS: AtomicU32 = AtomicU32::new(0x3F80_0000);

/// Box2D bases all length units on meters. Set this to use different units for
/// all length values passed to and returned from Box2D. Must be set at
/// application startup, before any other Box2D calls.
///
/// See the extended documentation on `b2SetLengthUnitsPerMeter` in
/// math_functions.h.
pub fn set_length_units_per_meter(length_units: f32) {
    debug_assert!(crate::math_functions::is_valid_float(length_units) && length_units > 0.0);
    LENGTH_UNITS_PER_METER_BITS.store(length_units.to_bits(), Ordering::Relaxed);
}

/// Get the current length units per meter.
pub fn get_length_units_per_meter() -> f32 {
    f32::from_bits(LENGTH_UNITS_PER_METER_BITS.load(Ordering::Relaxed))
}

/// Version numbering scheme. See <https://semver.org/>
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    /// Significant changes
    pub major: i32,
    /// Incremental changes
    pub minor: i32,
    /// Bug fixes
    pub revision: i32,
}

/// Get the current version of Box2D.
pub fn get_version() -> Version {
    Version {
        major: 3,
        minor: 2,
        revision: 0,
    }
}

/// @return true if the library was built with the `double-precision` feature
/// (large world mode), mirroring `BOX2D_DOUBLE_PRECISION`.
pub fn is_double_precision() -> bool {
    cfg!(feature = "double-precision")
}

/// Simple djb2 hash function for determinism testing. (timer.c)
pub fn hash(hash: u32, data: &[u8]) -> u32 {
    let mut result = hash;
    for &byte in data {
        // C: result = (result << 5) + result + data[i], all in wrapping uint32.
        result = (result << 5).wrapping_add(result).wrapping_add(byte as u32);
    }
    result
}

// ---------------------------------------------------------------------------
// Bit helpers (ctz.h). The C versions are thin wrappers over compiler
// intrinsics (__builtin_ctz / _BitScanForward / __popcnt). The count-leading
// and count-trailing intrinsics are undefined for a zero argument in C; every
// caller guarantees a nonzero argument, and Rust's intrinsics are well defined
// (returning the bit width) even for zero, so the ported callers behave
// identically.
// ---------------------------------------------------------------------------

/// Count trailing zeros of a 32-bit block. (ctz.h: b2CTZ32)
pub fn ctz32(block: u32) -> u32 {
    block.trailing_zeros()
}

/// Count leading zeros of a 32-bit value. (ctz.h: b2CLZ32)
pub fn clz32(value: u32) -> u32 {
    value.leading_zeros()
}

/// Count trailing zeros of a 64-bit block. (ctz.h: b2CTZ64)
pub fn ctz64(block: u64) -> u32 {
    block.trailing_zeros()
}

/// Population count of a 64-bit block. (ctz.h: b2PopCount64)
pub fn pop_count64(block: u64) -> i32 {
    block.count_ones() as i32
}

/// (ctz.h: b2IsPowerOf2)
pub fn is_power_of2(x: i32) -> bool {
    (x & (x - 1)) == 0
}

/// (ctz.h: b2BoundingPowerOf2)
pub fn bounding_power_of2(x: i32) -> i32 {
    if x <= 1 {
        return 1;
    }

    32 - clz32((x as u32) - 1) as i32
}

/// (ctz.h: b2RoundUpPowerOf2)
pub fn round_up_power_of2(x: i32) -> i32 {
    if x <= 1 {
        return 1;
    }

    1 << (32 - clz32((x as u32) - 1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn djb2_hash_matches_reference() {
        // djb2("box2d") computed from the reference formula starting at HASH_INIT.
        let mut expected: u32 = HASH_INIT;
        for &b in b"box2d" {
            expected = (expected << 5)
                .wrapping_add(expected)
                .wrapping_add(b as u32);
        }
        assert_eq!(hash(HASH_INIT, b"box2d"), expected);
        // Empty input returns the seed unchanged.
        assert_eq!(hash(HASH_INIT, b""), HASH_INIT);
    }

    #[test]
    fn bit_helpers() {
        assert_eq!(ctz32(0b1000), 3);
        assert_eq!(clz32(1), 31);
        assert_eq!(ctz64(1u64 << 40), 40);
        assert_eq!(pop_count64(0xFFFF_FFFF_FFFF_FFFF), 64);
        assert!(is_power_of2(8));
        assert!(!is_power_of2(6));
        assert_eq!(round_up_power_of2(5), 8);
        assert_eq!(round_up_power_of2(1), 1);
        assert_eq!(bounding_power_of2(5), 3);
    }

    #[test]
    fn version_and_precision() {
        let v = get_version();
        assert_eq!((v.major, v.minor, v.revision), (3, 2, 0));
        assert_eq!(is_double_precision(), cfg!(feature = "double-precision"));
    }

    // Runs the whole length-unit lifecycle in one test so it never races another
    // test observing the shared global (default -> scaled -> reset).
    #[test]
    fn length_units_scale_constants() {
        use crate::constants;

        assert_eq!(get_length_units_per_meter(), 1.0);
        assert_eq!(constants::linear_slop(), 0.005);

        set_length_units_per_meter(100.0);
        assert_eq!(get_length_units_per_meter(), 100.0);
        assert_eq!(constants::linear_slop(), 0.5);
        assert_eq!(constants::speculative_distance(), 4.0 * 0.5);
        assert_eq!(constants::max_aabb_margin(), 0.05 * 100.0);

        set_length_units_per_meter(1.0);
        assert_eq!(constants::linear_slop(), 0.005);
    }
}
