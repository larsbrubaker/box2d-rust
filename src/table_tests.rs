// Port of box2d-cpp-reference/test/test_table.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{bounding_power_of2, round_up_power_of2};
use crate::table::{shape_pair_key, HashSet};

const SET_SPAN: i32 = 317;
const ITEM_COUNT: usize = ((SET_SPAN * SET_SPAN - SET_SPAN) / 2) as usize;

#[test]
fn helper_power_of_two() {
    let power = bounding_power_of2(3008);
    assert_eq!(power, 12);

    let next_power_of2 = round_up_power_of2(3008);
    assert_eq!(next_power_of2, 1 << power);
}

#[test]
fn basic_create_and_destroy() {
    let mut set = HashSet::new(16);
    assert_eq!(set.count(), 0);
    assert_eq!(set.capacity(), 16);

    set.destroy();
    assert_eq!(set.count(), 0);
    assert_eq!(set.capacity(), 0);
}

#[test]
fn capacity_rounds_to_power_of_two() {
    assert_eq!(HashSet::new(1).capacity(), 16); // minimum capacity
    assert_eq!(HashSet::new(15).capacity(), 16); // rounds up to 16
    assert_eq!(HashSet::new(32).capacity(), 32); // stays at 32
    assert_eq!(HashSet::new(33).capacity(), 64); // rounds up to 64
}

#[test]
fn add_remove() {
    let mut set = HashSet::new(16);

    assert!(!set.add_key(42)); // new
    assert_eq!(set.count(), 1);

    assert!(!set.add_key(123)); // new
    assert_eq!(set.count(), 2);

    assert!(set.add_key(42)); // already exists
    assert_eq!(set.count(), 2); // count unchanged

    assert!(set.contains_key(42));
    assert!(set.contains_key(123));
    assert!(!set.contains_key(999));

    assert!(set.remove_key(42));
    assert_eq!(set.count(), 1);
    assert!(!set.contains_key(42));
    assert!(set.contains_key(123));

    assert!(!set.remove_key(999)); // non-existent
    assert_eq!(set.count(), 1);

    assert!(!set.remove_key(42)); // already removed
    assert_eq!(set.count(), 1);
}

#[test]
fn clear() {
    let mut set = HashSet::new(16);

    set.add_key(10);
    set.add_key(20);
    set.add_key(30);
    assert_eq!(set.count(), 3);

    set.clear();
    assert_eq!(set.count(), 0);
    assert!(!set.contains_key(10));
    assert!(!set.contains_key(20));
    assert!(!set.contains_key(30));

    set.add_key(40);
    assert_eq!(set.count(), 1);
    assert!(set.contains_key(40));
}

#[test]
fn growth_preserves_keys() {
    let mut set = HashSet::new(16);
    let initial_capacity = set.capacity();

    // Load factor is 0.5, so with capacity 16 growth happens at count 8.
    for i in 0..8u64 {
        set.add_key(i + 1);
    }

    assert!(set.capacity() >= initial_capacity);
    assert_eq!(set.count(), 8);

    for i in 1..=8u64 {
        assert!(set.contains_key(i));
    }
}

#[test]
fn edge_case_keys() {
    let mut set = HashSet::new(16);

    // Max value minus 1 (0 is the sentinel).
    let large_key = 0xFFFF_FFFF_FFFF_FFFF - 1;
    set.add_key(large_key);
    assert!(set.contains_key(large_key));
    assert_eq!(set.count(), 1);

    let key1 = 0x0123_4567_89AB_CDEF;
    let key2 = 0x0987_6543_21FE_DCBA;
    set.add_key(key1);
    set.add_key(key2);
    assert!(set.contains_key(key1));
    assert!(set.contains_key(key2));

    // A clustering pattern.
    for i in 0x1000..0x1010u64 {
        set.add_key(i);
    }
    for i in 0x1000..0x1010u64 {
        assert!(set.contains_key(i));
    }
}

#[test]
fn removal_reorganization() {
    let mut set = HashSet::new(16);

    let keys = [100u64, 116, 132, 148, 164];
    for &k in &keys {
        set.add_key(k);
    }
    for &k in &keys {
        assert!(set.contains_key(k));
    }

    // Remove from the middle; the others must remain (tests backward shifting).
    set.remove_key(keys[2]);
    assert!(!set.contains_key(keys[2]));
    for (i, &k) in keys.iter().enumerate() {
        if i != 2 {
            assert!(set.contains_key(k));
        }
    }
}

#[test]
fn stress_add_verify_remove() {
    const TEST_SIZE: usize = 1000;
    let mut set = HashSet::new(32);

    let keys: Vec<u64> = (0..TEST_SIZE).map(|i| (i * 7 + 13) as u64).collect();

    for &k in &keys {
        assert!(!set.add_key(k));
    }
    assert_eq!(set.count() as usize, TEST_SIZE);

    for &k in &keys {
        assert!(set.contains_key(k));
    }

    let mut removed_count = 0;
    for i in (0..TEST_SIZE).step_by(2) {
        assert!(set.remove_key(keys[i]));
        removed_count += 1;
    }
    assert_eq!(set.count() as usize, TEST_SIZE - removed_count);

    for (i, &k) in keys.iter().enumerate() {
        assert_eq!(set.contains_key(k), i % 2 == 1);
    }
}

#[test]
fn shape_pair_key_is_symmetric() {
    let mut set = HashSet::new(16);

    let key1 = shape_pair_key(5, 10);
    let key2 = shape_pair_key(10, 5); // same as key1
    assert_eq!(key1, key2);

    set.add_key(key1);
    assert!(set.contains_key(key1));
    assert!(set.contains_key(key2));

    let key3 = shape_pair_key(1, 2);
    let key4 = shape_pair_key(2, 3);
    assert_ne!(key3, key4);

    set.add_key(key3);
    set.add_key(key4);
    assert_eq!(set.count(), 3);
}

#[test]
fn bytes_tracks_capacity() {
    let mut set = HashSet::new(32);

    let expected = 32 * core::mem::size_of::<u64>() as i32;
    assert_eq!(set.bytes(), expected);

    set.add_key(100);
    set.add_key(200);
    assert_eq!(set.bytes(), expected);
}

// The large fill/remove/search cycle from test_table.c (HashSetTest), minus the
// timing prints. Exercises every shape pair over a 317-wide span.
#[test]
fn large_fill_remove_search() {
    let n = SET_SPAN;
    let item_count = ITEM_COUNT;
    let mut removed = vec![false; item_count];

    let mut set = HashSet::new(16);

    // Fill set with every (i, j) pair, i < j.
    for i in 0..n {
        for j in (i + 1)..n {
            let key = shape_pair_key(i, j);
            assert!(!set.add_key(key));
        }
    }
    assert_eq!(set.count() as usize, item_count);

    // Remove the j == i + 1 pairs.
    let mut k = 0;
    let mut remove_count = 0;
    for i in 0..n {
        for j in (i + 1)..n {
            if j == i + 1 {
                let key = shape_pair_key(i, j);
                let size1 = set.count();
                assert!(set.remove_key(key));
                assert_eq!(set.count(), size1 - 1);
                removed[k] = true;
                remove_count += 1;
            } else {
                removed[k] = false;
            }
            k += 1;
        }
    }
    assert_eq!(set.count() as usize, item_count - remove_count);

    // Every remaining key is found; removed ones may or may not be.
    let mut k = 0;
    for i in 0..n {
        for j in (i + 1)..n {
            let key = shape_pair_key(j, i);
            let found = set.contains_key(key);
            assert!(found || removed[k]);
            k += 1;
        }
    }

    // Remove everything.
    for i in 0..n {
        for j in (i + 1)..n {
            set.remove_key(shape_pair_key(i, j));
        }
    }
    assert_eq!(set.count(), 0);
}
