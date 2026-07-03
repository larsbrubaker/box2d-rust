// Port of box2d-cpp-reference/include/box2d/math_functions.h and src/math_functions.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
//
// Split into focused submodules to stay under the project file-length limit. The
// submodules form one flat namespace: everything is re-exported here, so callers
// use `crate::math_functions::<name>` exactly as before.

// Float literals are written with the exact digits of the C source, and Pos math casts
// through PosScalar so the same expression compiles in both precision modes (the cast is
// a no-op in single precision). These allows cascade into the submodules below.
#![allow(clippy::excessive_precision)]
#![allow(clippy::unnecessary_cast)]

mod query;
mod rotation;
mod scalar;
mod transform;
mod types;
mod validate;
mod vector;

pub use query::*;
pub use rotation::*;
pub use scalar::*;
pub use transform::*;
pub use types::*;
pub use validate::*;
pub use vector::*;
