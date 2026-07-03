// Port of box2d-cpp-reference/test/test_id.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::id::*;

#[test]
fn id_round_trips() {
    let a: u32 = 0x0123_4567;

    {
        let id = WorldId::load(a);
        let b = id.store();
        assert_eq!(b, a);
    }

    let x: u64 = 0x0123_4567_89AB_CDEF;

    {
        let id = BodyId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }

    {
        let id = ShapeId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }

    {
        let id = ChainId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }

    {
        let id = JointId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }
}

// Not in test_id.c, but locks in the null semantics and the contact-id layout.
#[test]
fn null_and_contact_round_trip() {
    assert!(WorldId::default().is_null());
    assert!(BodyId::default().is_null());
    assert!(!BodyId {
        index1: 1,
        ..Default::default()
    }
    .is_null());

    let values = [0x0123_4567u32, 0x0000_89AB, 0xCDEF_0011];
    let id = ContactId::load(values);
    assert_eq!(id.store(), values);
    assert_eq!(id.padding, 0);
}
