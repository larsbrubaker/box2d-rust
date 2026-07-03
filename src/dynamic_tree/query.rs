// AABB query, ray cast, and box cast traversals from dynamic_tree.c.
//
// The C callbacks take a `void* context`; the Rust versions take closures,
// which capture their context directly.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{BoxCastInput, DynamicTree, TreeStats, TREE_STACK_SIZE};
use crate::collision::RayCastInput;
use crate::core::NULL_INDEX;
use crate::math_functions::{
    aabb_center, aabb_extents, aabb_overlaps, abs, abs_float, add, cross_sv, distance_squared, dot,
    max, min, mul_add, mul_sv, normalize, sub, Aabb,
};

impl DynamicTree {
    /// Query an AABB for overlapping proxies. The callback is called for each
    /// proxy that overlaps the supplied AABB and passes the mask-bits filter;
    /// return false from the callback to stop. (b2DynamicTree_Query)
    pub fn query(
        &self,
        aabb: Aabb,
        mask_bits: u64,
        mut callback: impl FnMut(i32, u64) -> bool,
    ) -> TreeStats {
        let mut result = TreeStats::default();

        if self.node_count == 0 {
            return result;
        }

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];

            let node = &self.nodes[node_id as usize];
            result.node_visits += 1;

            if aabb_overlaps(node.aabb, aabb) && (node.category_bits & mask_bits) != 0 {
                if node.is_leaf() {
                    // callback to user code with proxy id
                    let proceed = callback(node_id, node.user_data);
                    result.leaf_visits += 1;

                    if !proceed {
                        return result;
                    }
                } else if stack_count < TREE_STACK_SIZE - 1 {
                    stack[stack_count] = node.child1;
                    stack_count += 1;
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                } else {
                    debug_assert!(stack_count < TREE_STACK_SIZE - 1);
                }
            }
        }

        result
    }

    /// Query an AABB for overlapping proxies with no filtering.
    /// (b2DynamicTree_QueryAll)
    pub fn query_all(&self, aabb: Aabb, mut callback: impl FnMut(i32, u64) -> bool) -> TreeStats {
        let mut result = TreeStats::default();

        if self.node_count == 0 {
            return result;
        }

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];

            let node = &self.nodes[node_id as usize];
            result.node_visits += 1;

            if aabb_overlaps(node.aabb, aabb) {
                if node.is_leaf() {
                    // callback to user code with proxy id
                    let proceed = callback(node_id, node.user_data);
                    result.leaf_visits += 1;

                    if !proceed {
                        return result;
                    }
                } else if stack_count < TREE_STACK_SIZE - 1 {
                    stack[stack_count] = node.child1;
                    stack_count += 1;
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                } else {
                    debug_assert!(stack_count < TREE_STACK_SIZE - 1);
                }
            }
        }

        result
    }

    /// Ray cast against the proxies in the tree. The callback performs an
    /// exact ray cast when the proxy contains a shape, and returns the new
    /// ray fraction:
    /// - return 0 to terminate the ray cast
    /// - return a value less than the input max_fraction to clip the ray
    /// - return the input max_fraction to continue without clipping
    ///
    /// (b2DynamicTree_RayCast)
    pub fn ray_cast(
        &self,
        input: &RayCastInput,
        mask_bits: u64,
        mut callback: impl FnMut(&RayCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        let mut result = TreeStats::default();

        if self.node_count == 0 {
            return result;
        }

        let p1 = input.origin;
        let d = input.translation;

        let r = normalize(d);

        // v is perpendicular to the segment.
        let v = cross_sv(1.0, r);
        let abs_v = abs(v);

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)

        let mut max_fraction = input.max_fraction;

        let mut p2 = mul_add(p1, max_fraction, d);

        // Build a bounding box for the segment.
        let mut segment_aabb = Aabb {
            lower_bound: min(p1, p2),
            upper_bound: max(p1, p2),
        };

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        let mut sub_input = *input;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];
            if node_id == NULL_INDEX {
                debug_assert!(false);
                continue;
            }

            let node = &self.nodes[node_id as usize];
            result.node_visits += 1;

            let node_aabb = node.aabb;

            if (node.category_bits & mask_bits) == 0 || !aabb_overlaps(node_aabb, segment_aabb) {
                continue;
            }

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            // radius extension is added to the node in this case
            let c = aabb_center(node_aabb);
            let h = aabb_extents(node_aabb);
            let term1 = abs_float(dot(v, sub(p1, c)));
            let term2 = dot(abs_v, h);
            if term2 < term1 {
                continue;
            }

            if node.is_leaf() {
                sub_input.max_fraction = max_fraction;

                let value = callback(&sub_input, node_id, node.user_data);
                result.leaf_visits += 1;

                // The user may return -1 to indicate this shape should be skipped

                if value == 0.0 {
                    // The client has terminated the ray cast.
                    return result;
                }

                if 0.0 < value && value <= max_fraction {
                    // Update segment bounding box.
                    max_fraction = value;
                    p2 = mul_add(p1, max_fraction, d);
                    segment_aabb.lower_bound = min(p1, p2);
                    segment_aabb.upper_bound = max(p1, p2);
                }
            } else if stack_count < TREE_STACK_SIZE - 1 {
                let c1 = aabb_center(self.nodes[node.child1 as usize].aabb);
                let c2 = aabb_center(self.nodes[node.child2 as usize].aabb);
                if distance_squared(c1, p1) < distance_squared(c2, p1) {
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                    stack[stack_count] = node.child1;
                    stack_count += 1;
                } else {
                    stack[stack_count] = node.child1;
                    stack_count += 1;
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                }
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE - 1);
            }
        }

        result
    }

    /// Cast a swept AABB through the tree. The callback returns the new cast
    /// fraction, with the same semantics as [`DynamicTree::ray_cast`].
    /// (b2DynamicTree_BoxCast)
    pub fn box_cast(
        &self,
        input: &BoxCastInput,
        mask_bits: u64,
        mut callback: impl FnMut(&BoxCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        let mut stats = TreeStats::default();

        if self.node_count == 0 {
            return stats;
        }

        // The caller folds the shape radius into the box
        let origin_aabb = input.box_;

        let p1 = aabb_center(origin_aabb);
        let extension = aabb_extents(origin_aabb);

        // v is perpendicular to the segment.
        let r = input.translation;
        let v = cross_sv(1.0, r);
        let abs_v = abs(v);

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)

        let mut max_fraction = input.max_fraction;

        // Build total box for the cast
        let mut t = mul_sv(max_fraction, input.translation);
        let mut total_aabb = Aabb {
            lower_bound: min(origin_aabb.lower_bound, add(origin_aabb.lower_bound, t)),
            upper_bound: max(origin_aabb.upper_bound, add(origin_aabb.upper_bound, t)),
        };

        let mut sub_input = *input;

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];
            if node_id == NULL_INDEX {
                debug_assert!(false);
                continue;
            }

            let node = &self.nodes[node_id as usize];
            stats.node_visits += 1;

            if (node.category_bits & mask_bits) == 0 || !aabb_overlaps(node.aabb, total_aabb) {
                continue;
            }

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            // radius extension is added to the node in this case
            let c = aabb_center(node.aabb);
            let h = add(aabb_extents(node.aabb), extension);
            let term1 = abs_float(dot(v, sub(p1, c)));
            let term2 = dot(abs_v, h);
            if term2 < term1 {
                continue;
            }

            if node.is_leaf() {
                sub_input.max_fraction = max_fraction;

                let value = callback(&sub_input, node_id, node.user_data);
                stats.leaf_visits += 1;

                if value == 0.0 {
                    // The client has terminated the ray cast.
                    return stats;
                }

                if 0.0 < value && value < max_fraction {
                    // Update segment bounding box.
                    max_fraction = value;
                    t = mul_sv(max_fraction, input.translation);
                    total_aabb.lower_bound =
                        min(origin_aabb.lower_bound, add(origin_aabb.lower_bound, t));
                    total_aabb.upper_bound =
                        max(origin_aabb.upper_bound, add(origin_aabb.upper_bound, t));
                }
            } else if stack_count < TREE_STACK_SIZE - 1 {
                let c1 = aabb_center(self.nodes[node.child1 as usize].aabb);
                let c2 = aabb_center(self.nodes[node.child2 as usize].aabb);
                if distance_squared(c1, p1) < distance_squared(c2, p1) {
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                    stack[stack_count] = node.child1;
                    stack_count += 1;
                } else {
                    stack[stack_count] = node.child1;
                    stack_count += 1;
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                }
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE - 1);
            }
        }

        stats
    }
}
