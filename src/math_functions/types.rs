// Math types, constants, and operator overloads.
// Part of the math_functions module (see mod.rs).

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
#[derive(Debug, Clone, Copy, PartialEq, Default)]
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
