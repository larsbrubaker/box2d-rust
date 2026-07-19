// Wide (SIMD) primitives for the contact solver, ported from the b2FloatW
// abstraction in contact_solver.c.
//
// Box2D's default x64 build uses B2_SIMD_SSE2 with B2_SIMD_WIDTH == 4, so the
// wide contact solver processes four graph-color contacts per block. This port
// stays in safe Rust: `FloatW` is a newtype over `[f32; 4]` and every operation
// is a fixed-size lane loop, which the fat-LTO release profile autovectorizes.
//
// Bit-exactness requirement: the lane arithmetic is composed op-for-op exactly
// as the C intrinsics compose it. In particular b2MulAddW on SSE2 is
// `_mm_add_ps(a, _mm_mul_ps(b, c))` — a real multiply followed by a real add,
// NOT a fused multiply-add — so `mul_add(a, b, c)` here is `a + b * c` computed
// as two separate rounded operations. min/max/sym_clamp mirror the SSE2 `>`/`<`
// comparison semantics, which coincide with the scalar `max_float`/`min_float`/
// `clamp_float` used by the overflow path. This is why the wide path produces
// results bit-identical to the scalar path for disjoint bodies within a color.
//
// b2FloatW is four `float` lanes regardless of BOX2D_DOUBLE_PRECISION: in
// Box2D `b2Vec2` and `b2BodyState` stay f32 in large-world mode (only b2Pos /
// transform translation becomes f64), so the wide solver is identical in both
// precision configurations and `FloatW` is always `[f32; 4]`.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// bring-up: called by the wide contact kernels.
//
// The fixed-size lane loops index several `[f32; 4]` arrays in lockstep, which
// is the point of the SIMD layout; enumerate() would obscure the C-mirroring
// intent. The add/sub/mul methods deliberately mirror b2AddW/b2SubW/b2MulW
// rather than the std::ops traits so the kernels read like the C source.
#![allow(clippy::needless_range_loop)]
#![allow(clippy::should_implement_trait)]

use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::math_functions::Vec2;

/// Number of SIMD lanes. Matches the C B2_SIMD_WIDTH for the SSE2/NEON x64/ARM
/// default (BOX2D_AVX2 with width 8 is not the default and is not ported).
pub const SIMD_WIDTH: usize = 4;

/// Wide float holding four lanes. (b2FloatW)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FloatW(pub [f32; SIMD_WIDTH]);

/// Comparison mask, one boolean per lane. The C code represents masks as
/// all-ones / all-zero float bit patterns and blends with bitwise and/or; a
/// per-lane boolean with a `select` blend is behaviorally identical.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MaskW(pub [bool; SIMD_WIDTH]);

impl MaskW {
    /// (b2OrW on masks)
    #[inline]
    pub fn or(self, other: MaskW) -> MaskW {
        let mut r = [false; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] || other.0[i];
        }
        MaskW(r)
    }
}

impl FloatW {
    /// (b2ZeroW)
    #[inline]
    pub fn zero() -> FloatW {
        FloatW([0.0; SIMD_WIDTH])
    }

    /// (b2SplatW)
    #[inline]
    pub fn splat(scalar: f32) -> FloatW {
        FloatW([scalar; SIMD_WIDTH])
    }

    /// (b2AddW)
    #[inline]
    pub fn add(self, b: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] + b.0[i];
        }
        FloatW(r)
    }

    /// (b2SubW)
    #[inline]
    pub fn sub(self, b: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] - b.0[i];
        }
        FloatW(r)
    }

    /// (b2MulW)
    #[inline]
    pub fn mul(self, b: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] * b.0[i];
        }
        FloatW(r)
    }

    /// (b2MulAddW) self + b * c, two separate rounded ops as on SSE2.
    #[inline]
    pub fn mul_add(self, b: FloatW, c: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] + b.0[i] * c.0[i];
        }
        FloatW(r)
    }

    /// (b2MulSubW) self - b * c, two separate rounded ops as on SSE2.
    #[inline]
    pub fn mul_sub(self, b: FloatW, c: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] - b.0[i] * c.0[i];
        }
        FloatW(r)
    }

    /// (b2MinW) matches SSE2 `_mm_min_ps` (a < b ? a : b) and `min_float`.
    #[inline]
    pub fn min(self, b: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = if self.0[i] < b.0[i] {
                self.0[i]
            } else {
                b.0[i]
            };
        }
        FloatW(r)
    }

    /// (b2MaxW) matches SSE2 `_mm_max_ps` (a > b ? a : b) and `max_float`.
    #[inline]
    pub fn max(self, b: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = if self.0[i] > b.0[i] {
                self.0[i]
            } else {
                b.0[i]
            };
        }
        FloatW(r)
    }

    /// (b2SymClampW) clamp(self, -b, b) as max(-b, min(self, b)).
    #[inline]
    pub fn sym_clamp(self, b: FloatW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            let nb = -b.0[i];
            let m = if self.0[i] < b.0[i] {
                self.0[i]
            } else {
                b.0[i]
            };
            r[i] = if nb > m { nb } else { m };
        }
        FloatW(r)
    }

    /// (b2GreaterThanW) per-lane self > b.
    #[inline]
    pub fn greater_than(self, b: FloatW) -> MaskW {
        let mut r = [false; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] > b.0[i];
        }
        MaskW(r)
    }

    /// (b2EqualsW) per-lane self == b.
    #[inline]
    pub fn equals(self, b: FloatW) -> MaskW {
        let mut r = [false; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = self.0[i] == b.0[i];
        }
        MaskW(r)
    }

    /// (b2AllZeroW) true if every lane is exactly zero.
    #[inline]
    pub fn all_zero(self) -> bool {
        self.0[0] == 0.0 && self.0[1] == 0.0 && self.0[2] == 0.0 && self.0[3] == 0.0
    }

    /// (b2BlendW) component-wise returns mask ? b : a.
    #[inline]
    pub fn blend(a: FloatW, b: FloatW, mask: MaskW) -> FloatW {
        let mut r = [0.0; SIMD_WIDTH];
        for i in 0..SIMD_WIDTH {
            r[i] = if mask.0[i] { b.0[i] } else { a.0[i] };
        }
        FloatW(r)
    }
}

/// Wide vec2. (b2Vec2W)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2W {
    pub x: FloatW,
    pub y: FloatW,
}

/// Wide rotation. (b2RotW)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RotW {
    pub c: FloatW,
    pub s: FloatW,
}

/// (b2DotW)
#[inline]
pub fn dot_w(a: Vec2W, b: Vec2W) -> FloatW {
    a.x.mul(b.x).add(a.y.mul(b.y))
}

/// (b2CrossW)
#[inline]
pub fn cross_w(a: Vec2W, b: Vec2W) -> FloatW {
    a.x.mul(b.y).sub(a.y.mul(b.x))
}

/// (b2RotateVectorW)
#[inline]
pub fn rotate_vector_w(q: RotW, v: Vec2W) -> Vec2W {
    Vec2W {
        x: q.c.mul(v.x).sub(q.s.mul(v.y)),
        y: q.s.mul(v.x).add(q.c.mul(v.y)),
    }
}

/// Wide version of b2BodyState. (b2BodyStateW)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BodyStateW {
    pub v: Vec2W,
    pub w: FloatW,
    pub flags: FloatW,
    pub dp: Vec2W,
    pub dq: RotW,
}

/// Load four body states through lane indices. A stored index of 0 means null
/// (the C encodes body sim index + 1), which yields the identity body state.
/// This mirrors the scalar-fallback `b2GatherBodies`; the SSE2/NEON transpose
/// variants compute the same logical result. (b2GatherBodies)
#[inline]
pub fn gather_bodies(states: &[BodyState], indices: &[i32; SIMD_WIDTH]) -> BodyStateW {
    let mut out = BodyStateW::default();
    for lane in 0..SIMD_WIDTH {
        // zero means null
        let i = indices[lane] - 1;
        let s = if i == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[i as usize]
        };
        out.v.x.0[lane] = s.linear_velocity.x;
        out.v.y.0[lane] = s.linear_velocity.y;
        out.w.0[lane] = s.angular_velocity;
        out.flags.0[lane] = s.flags as f32;
        out.dp.x.0[lane] = s.delta_position.x;
        out.dp.y.0[lane] = s.delta_position.y;
        out.dq.c.0[lane] = s.delta_rotation.c;
        out.dq.s.0[lane] = s.delta_rotation.s;
    }
    out
}

/// Write the four lanes' velocities back to the solver bodies. Only dynamic
/// bodies are written (checked against the live state flags, not the gathered
/// lane), matching the C which avoids sharing a dummy body across workers.
/// (b2ScatterBodies)
#[inline]
pub fn scatter_bodies(states: &mut [BodyState], indices: &[i32; SIMD_WIDTH], body: &BodyStateW) {
    for lane in 0..SIMD_WIDTH {
        // zero means null
        let i = indices[lane] - 1;
        if i == NULL_INDEX {
            continue;
        }
        let state = &mut states[i as usize];
        if state.flags & body_flags::DYNAMIC_FLAG != 0 {
            state.linear_velocity = Vec2 {
                x: body.v.x.0[lane],
                y: body.v.y.0[lane],
            };
            state.angular_velocity = body.w.0[lane];
        }
    }
}
