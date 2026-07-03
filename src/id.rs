// Port of box2d-cpp-reference/include/box2d/id.h
//
// These ids are opaque handles to internal Box2D objects, passed by value. All
// ids are null when zero-initialized. The store/load helpers pack and unpack a
// handle into a plain integer; the bit layout is reproduced exactly so handles
// round-trip identically to the C library.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

/// World id references a world instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WorldId {
    pub index1: u16,
    pub generation: u16,
}

/// Body id references a body instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BodyId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Shape id references a shape instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ShapeId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Chain id references a chain instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChainId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Joint id references a joint instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct JointId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Contact id references a contact instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ContactId {
    pub index1: i32,
    pub world0: u16,
    pub padding: i16,
    pub generation: u32,
}

impl WorldId {
    /// Store a world id into a u32. (b2StoreWorldId)
    pub fn store(self) -> u32 {
        ((self.index1 as u32) << 16) | (self.generation as u32)
    }

    /// Load a u32 into a world id. (b2LoadWorldId)
    pub fn load(x: u32) -> WorldId {
        WorldId {
            index1: (x >> 16) as u16,
            generation: x as u16,
        }
    }

    /// True if this id is null (index1 == 0). (B2_IS_NULL)
    pub fn is_null(self) -> bool {
        self.index1 == 0
    }

    /// True if this id is non-null. (B2_IS_NON_NULL)
    pub fn is_non_null(self) -> bool {
        self.index1 != 0
    }
}

// Body, shape, chain, and joint ids share the same 64-bit layout, so a macro
// generates their identical store/load/null helpers.
macro_rules! impl_u64_id {
    ($ty:ident) => {
        impl $ty {
            /// Store this id into a u64.
            pub fn store(self) -> u64 {
                ((self.index1 as u64) << 32)
                    | ((self.world0 as u64) << 16)
                    | (self.generation as u64)
            }

            /// Load a u64 into this id type.
            pub fn load(x: u64) -> $ty {
                $ty {
                    index1: (x >> 32) as i32,
                    world0: (x >> 16) as u16,
                    generation: x as u16,
                }
            }

            /// True if this id is null (index1 == 0). (B2_IS_NULL)
            pub fn is_null(self) -> bool {
                self.index1 == 0
            }

            /// True if this id is non-null. (B2_IS_NON_NULL)
            pub fn is_non_null(self) -> bool {
                self.index1 != 0
            }
        }
    };
}

impl_u64_id!(BodyId);
impl_u64_id!(ShapeId);
impl_u64_id!(ChainId);
impl_u64_id!(JointId);

impl ContactId {
    /// Store a contact id into three u32 values. (b2StoreContactId)
    pub fn store(self) -> [u32; 3] {
        [self.index1 as u32, self.world0 as u32, self.generation]
    }

    /// Load three u32 values into a contact id. (b2LoadContactId)
    pub fn load(values: [u32; 3]) -> ContactId {
        ContactId {
            index1: values[0] as i32,
            world0: values[1] as u16,
            padding: 0,
            generation: values[2],
        }
    }

    /// True if this id is null (index1 == 0). (B2_IS_NULL)
    pub fn is_null(self) -> bool {
        self.index1 == 0
    }

    /// True if this id is non-null. (B2_IS_NON_NULL)
    pub fn is_non_null(self) -> bool {
        self.index1 != 0
    }
}
