// Port of box2d-cpp-reference/src/table.h and src/table.c
//
// An open-addressing hash set of u64 keys (a key of 0 is the empty sentinel).
// Linear probing for lookup, backward-shift deletion to keep probe chains tight.
// The C `b2SetItem` is just a `uint64_t key`, so the backing store is a
// `Vec<u64>` whose length is always the (power-of-two) capacity.
//
// b2CountSetBits, defined in this C file, lives in bitset.rs next to the type it
// operates on.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::round_up_power_of2;

/// Build a symmetric key from a pair of shape indices. (B2_SHAPE_PAIR_KEY)
///
/// The smaller index goes in the high 32 bits so `(a, b)` and `(b, a)` map to
/// the same key.
pub fn shape_pair_key(k1: i32, k2: i32) -> u64 {
    if k1 < k2 {
        (k1 as u64) << 32 | (k2 as u64)
    } else {
        (k2 as u64) << 32 | (k1 as u64)
    }
}

/// An open-addressing hash set of non-zero u64 keys. (b2HashSet)
#[derive(Debug, Clone, Default)]
pub struct HashSet {
    /// Backing slots; `items.len()` is the capacity. A slot of 0 is empty.
    pub(crate) items: Vec<u64>,
    pub(crate) count: u32,
}

// Murmur hash finalizer. A good mixer matters here because keys are built from
// pairs of increasing integers, where a simple XOR hash collides heavily.
fn key_hash(key: u64) -> u64 {
    let mut h = key;
    h ^= h >> 33;
    h = h.wrapping_mul(0xff51afd7ed558ccd);
    h ^= h >> 33;
    h = h.wrapping_mul(0xc4ceb9fe1a85ec53);
    h ^= h >> 33;
    h
}

impl HashSet {
    /// Create a set with at least `capacity` slots, rounded up to a power of two
    /// (minimum 16). (b2CreateSet)
    pub fn new(capacity: i32) -> HashSet {
        // Capacity must be a power of 2
        let capacity = if capacity > 16 {
            round_up_power_of2(capacity)
        } else {
            16
        };

        HashSet {
            items: vec![0; capacity as usize],
            count: 0,
        }
    }

    /// Release the storage. (b2DestroySet)
    pub fn destroy(&mut self) {
        self.items = Vec::new();
        self.count = 0;
    }

    /// Remove all keys, keeping capacity. (b2ClearSet)
    pub fn clear(&mut self) {
        self.count = 0;
        self.items.iter_mut().for_each(|slot| *slot = 0);
    }

    /// Number of keys in the set. (b2GetSetCount)
    pub fn count(&self) -> i32 {
        self.count as i32
    }

    /// Number of allocated slots. (b2GetSetCapacity)
    pub fn capacity(&self) -> i32 {
        self.items.len() as i32
    }

    /// Byte size of the backing storage. (b2GetHashSetBytes)
    pub fn bytes(&self) -> i32 {
        self.items.len() as i32 * core::mem::size_of::<u64>() as i32
    }

    fn cap(&self) -> u32 {
        self.items.len() as u32
    }

    // Find the slot holding `key`, or the first empty slot on its probe chain.
    fn find_slot(&self, key: u64, hash: u64) -> usize {
        let capacity = self.cap();
        let mut index = (hash as u32) & (capacity - 1);
        while self.items[index as usize] != 0 && self.items[index as usize] != key {
            index = (index + 1) & (capacity - 1);
        }
        index as usize
    }

    fn add_key_have_capacity(&mut self, key: u64, hash: u64) {
        let index = self.find_slot(key, hash);
        debug_assert!(self.items[index] == 0);
        self.items[index] = key;
        self.count += 1;
    }

    fn grow_table(&mut self) {
        let old_items = core::mem::take(&mut self.items);

        self.count = 0;
        // Capacity must be a power of 2
        self.items = vec![0; 2 * old_items.len()];

        // Transfer items into new array
        for &key in &old_items {
            if key == 0 {
                // this item was empty
                continue;
            }

            let hash = key_hash(key);
            self.add_key_have_capacity(key, hash);
        }
    }

    /// True if `key` is present. (b2ContainsKey)
    pub fn contains_key(&self, key: u64) -> bool {
        // key of zero is a sentinel
        debug_assert!(key != 0);
        let hash = key_hash(key);
        let index = self.find_slot(key, hash);
        self.items[index] == key
    }

    /// Add `key`. Returns true if it was already present. (b2AddKey)
    pub fn add_key(&mut self, key: u64) -> bool {
        // key of zero is a sentinel
        debug_assert!(key != 0);

        let hash = key_hash(key);
        debug_assert!(hash != 0);

        let index = self.find_slot(key, hash);
        if self.items[index] != 0 {
            // Already in set
            debug_assert!(self.items[index] == key);
            return true;
        }

        if 2 * self.count >= self.cap() {
            self.grow_table();
        }

        self.add_key_have_capacity(key, hash);
        false
    }

    /// Remove `key`. Returns true if it was found. (b2RemoveKey)
    // See https://en.wikipedia.org/wiki/Open_addressing
    pub fn remove_key(&mut self, key: u64) -> bool {
        let hash = key_hash(key);
        let mut i = self.find_slot(key, hash);
        if self.items[i] == 0 {
            // Not in set
            return false;
        }

        // Mark item i as unoccupied
        self.items[i] = 0;

        debug_assert!(self.count > 0);
        self.count -= 1;

        // Attempt to fill item i
        let mask = self.items.len() - 1;
        let mut j = i;
        loop {
            j = (j + 1) & mask;
            if self.items[j] == 0 {
                break;
            }

            // k is the first slot for the hash of j
            let hash_j = key_hash(self.items[j]);
            let k = (hash_j as usize) & mask;

            // determine if k lies cyclically in (i,j]
            // i <= j: | i..k..j |
            // i > j: |.k..j  i....| or |....j     i..k.|
            if i <= j {
                if i < k && k <= j {
                    continue;
                }
            } else if i < k || k <= j {
                continue;
            }

            // Move j into i
            self.items[i] = self.items[j];

            // Mark item j as unoccupied
            self.items[j] = 0;

            i = j;
        }

        true
    }
}
