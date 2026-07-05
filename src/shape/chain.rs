// Chain shapes from shape.c: b2CreateChain, b2DestroyChain, and the
// b2Chain_* accessors. A chain is a sequence of chain-segment shapes with
// ghost vertices for smooth sliding, owned by one body.
//
// b2Chain_GetWorld is omitted: there is no world registry in the Rust port.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use super::{create_shape_internal, destroy_shape_internal, ChainShape};
use crate::body::get_body_full_id;
use crate::collision::{ChainSegment, Segment, ShapeGeometry};
use crate::core::NULL_INDEX;
use crate::id::{BodyId, ChainId, ShapeId};
use crate::math_functions::is_valid_float;
use crate::types::{default_shape_def, ChainDef, SurfaceMaterial};
use crate::world::World;

/// Chain id validity. (b2Chain_IsValid — the world-registry check collapses
/// to the index/generation check in the registry-less port)
pub fn chain_is_valid(world: &World, id: ChainId) -> bool {
    let chain_id = id.index1 - 1;
    if chain_id < 0 || world.chain_shapes.len() as i32 <= chain_id {
        return false;
    }

    let chain = &world.chain_shapes[chain_id as usize];
    if chain.id == NULL_INDEX {
        // chain is free
        return false;
    }

    debug_assert!(chain.id == chain_id);

    id.generation == chain.generation
}

/// Validate a ChainId and return the raw chain index. (static b2GetChainShape
/// — C returns a pointer; Rust returns the index into `world.chain_shapes`)
pub fn get_chain_index(world: &World, chain_id: ChainId) -> i32 {
    let index = chain_id.index1 - 1;
    debug_assert!((index as usize) < world.chain_shapes.len());
    let chain = &world.chain_shapes[index as usize];
    debug_assert!(chain.id == index && chain.generation == chain_id.generation);
    index
}

/// Create a chain shape: a sequence of chain segments attached to a body.
/// The def must have at least 4 points. (b2CreateChain)
pub fn create_chain(world: &mut World, body_id: BodyId, def: &ChainDef) -> ChainId {
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(def.points.len() >= 4);
    debug_assert!(def.materials.len() == 1 || def.materials.len() == def.points.len());

    debug_assert!(!world.locked);
    if world.locked {
        return ChainId::default();
    }

    let body_index = get_body_full_id(world, body_id);
    let transform =
        crate::body::get_body_transform_quick(world, &world.bodies[body_index as usize]);

    let chain_id = world.chain_id_pool.alloc_id();

    if chain_id == world.chain_shapes.len() as i32 {
        world.chain_shapes.push(ChainShape {
            id: NULL_INDEX,
            body_id: NULL_INDEX,
            next_chain_id: NULL_INDEX,
            shape_indices: Vec::new(),
            materials: Vec::new(),
            generation: 0,
        });
    } else {
        debug_assert!(world.chain_shapes[chain_id as usize].id == NULL_INDEX);
    }

    let material_count = def.materials.len();
    for material in def.materials.iter() {
        debug_assert!(is_valid_float(material.friction) && material.friction >= 0.0);
        debug_assert!(is_valid_float(material.restitution) && material.restitution >= 0.0);
        debug_assert!(
            is_valid_float(material.rolling_resistance) && material.rolling_resistance >= 0.0
        );
        debug_assert!(is_valid_float(material.tangent_speed));
    }

    let head_chain_id = world.bodies[body_index as usize].head_chain_id;
    {
        let chain_shape = &mut world.chain_shapes[chain_id as usize];
        chain_shape.id = chain_id;
        chain_shape.body_id = world.bodies[body_index as usize].id;
        chain_shape.next_chain_id = head_chain_id;
        chain_shape.generation += 1;
        chain_shape.materials = def.materials.clone();
    }

    world.bodies[body_index as usize].head_chain_id = chain_id;

    let mut shape_def = default_shape_def();
    shape_def.user_data = def.user_data;
    shape_def.filter = def.filter;
    shape_def.enable_sensor_events = def.enable_sensor_events;
    shape_def.enable_contact_events = false;
    shape_def.enable_hit_events = false;

    let points = &def.points;
    let n = points.len();

    // Materials are indexed by the leading point of the solid segment (or 0
    // when the chain has a single shared material).
    let material_for = |index: usize| -> SurfaceMaterial {
        if material_count == 1 {
            def.materials[0]
        } else {
            def.materials[index]
        }
    };

    let make_segment = |g1: usize, p1: usize, p2: usize, g2: usize| ChainSegment {
        ghost1: points[g1],
        segment: Segment {
            point1: points[p1],
            point2: points[p2],
        },
        ghost2: points[g2],
        chain_id,
    };

    let mut shape_indices: Vec<i32>;

    if def.is_loop {
        shape_indices = Vec::with_capacity(n);

        let mut prev_index = n - 1;
        for i in 0..(n - 2) {
            let chain_segment = make_segment(prev_index, i, i + 1, i + 2);
            prev_index = i;

            shape_def.material = material_for(i);
            let shape_id = create_shape_internal(
                world,
                body_index,
                transform,
                &shape_def,
                ShapeGeometry::ChainSegment(chain_segment),
            );
            shape_indices.push(shape_id);
        }

        {
            let chain_segment = make_segment(n - 3, n - 2, n - 1, 0);
            shape_def.material = material_for(n - 2);
            let shape_id = create_shape_internal(
                world,
                body_index,
                transform,
                &shape_def,
                ShapeGeometry::ChainSegment(chain_segment),
            );
            shape_indices.push(shape_id);
        }

        {
            let chain_segment = make_segment(n - 2, n - 1, 0, 1);
            shape_def.material = material_for(n - 1);
            let shape_id = create_shape_internal(
                world,
                body_index,
                transform,
                &shape_def,
                ShapeGeometry::ChainSegment(chain_segment),
            );
            shape_indices.push(shape_id);
        }
    } else {
        shape_indices = Vec::with_capacity(n - 3);

        for i in 0..(n - 3) {
            let chain_segment = make_segment(i, i + 1, i + 2, i + 3);

            // Material is associated with leading point of solid segment
            shape_def.material = material_for(i + 1);
            let shape_id = create_shape_internal(
                world,
                body_index,
                transform,
                &shape_def,
                ShapeGeometry::ChainSegment(chain_segment),
            );
            shape_indices.push(shape_id);
        }
    }

    world.chain_shapes[chain_id as usize].shape_indices = shape_indices;

    let id = ChainId {
        index1: chain_id + 1,
        world0: world.world_id,
        generation: world.chain_shapes[chain_id as usize].generation,
    };

    // (B2_REC_CREATE(world, CreateChain, id, bodyId, *def))
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_create_chain(rec, body_id, def, id)
    });

    id
}

/// Destroy a chain shape and all its segments. (b2DestroyChain +
/// b2FreeChainData)
pub fn destroy_chain(world: &mut World, chain_id: ChainId) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_destroy_chain(rec, chain_id)
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let chain_index = get_chain_index(world, chain_id);
    let body_id = world.chain_shapes[chain_index as usize].body_id;

    // Remove the chain from the body's singly linked list.
    let mut found = false;
    if world.bodies[body_id as usize].head_chain_id == chain_index {
        world.bodies[body_id as usize].head_chain_id =
            world.chain_shapes[chain_index as usize].next_chain_id;
        found = true;
    } else {
        let mut prev_chain_id = world.bodies[body_id as usize].head_chain_id;
        while prev_chain_id != NULL_INDEX {
            let next = world.chain_shapes[prev_chain_id as usize].next_chain_id;
            if next == chain_index {
                world.chain_shapes[prev_chain_id as usize].next_chain_id =
                    world.chain_shapes[chain_index as usize].next_chain_id;
                found = true;
                break;
            }
            prev_chain_id = next;
        }
    }

    debug_assert!(found);
    if !found {
        return;
    }

    let shape_indices = std::mem::take(&mut world.chain_shapes[chain_index as usize].shape_indices);
    for shape_id in shape_indices {
        let wake_bodies = true;
        destroy_shape_internal(world, shape_id, body_id, wake_bodies);
    }

    // (b2FreeChainData)
    world.chain_shapes[chain_index as usize].materials = Vec::new();

    // Return chain to free list.
    world.chain_id_pool.free_id(chain_index);
    world.chain_shapes[chain_index as usize].id = NULL_INDEX;

    world.validate_solver_sets();
}

/// Get the number of segments on this chain. (b2Chain_GetSegmentCount)
pub fn chain_get_segment_count(world: &World, chain_id: ChainId) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let chain_index = get_chain_index(world, chain_id);
    world.chain_shapes[chain_index as usize].shape_indices.len() as i32
}

/// Get the segment shape ids for a chain, up to `capacity`.
/// (b2Chain_GetSegments — returns a Vec instead of filling a caller array)
pub fn chain_get_segments(world: &World, chain_id: ChainId, capacity: usize) -> Vec<ShapeId> {
    debug_assert!(!world.locked);
    if world.locked {
        return Vec::new();
    }

    let chain_index = get_chain_index(world, chain_id);
    let chain = &world.chain_shapes[chain_index as usize];

    let count = chain.shape_indices.len().min(capacity);
    chain.shape_indices[..count]
        .iter()
        .map(|&shape_id| ShapeId {
            index1: shape_id + 1,
            world0: world.world_id,
            generation: world.shapes[shape_id as usize].generation,
        })
        .collect()
}

/// Get the number of chain surface materials: 1 (shared) or the segment
/// count. (b2Chain_GetSurfaceMaterialCount)
pub fn chain_get_surface_material_count(world: &World, chain_id: ChainId) -> i32 {
    let chain_index = get_chain_index(world, chain_id);
    world.chain_shapes[chain_index as usize].materials.len() as i32
}

/// Set a chain surface material, propagating it to the affected segment
/// shape(s). (b2Chain_SetSurfaceMaterial)
pub fn chain_set_surface_material(
    world: &mut World,
    chain_id: ChainId,
    material: SurfaceMaterial,
    material_index: usize,
) {
    crate::recording::record_op(world, |rec, _| {
        crate::recording::write_chain_set_surface_material(
            rec,
            chain_id,
            material,
            material_index as i32,
        )
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let chain_index = get_chain_index(world, chain_id);
    debug_assert!(material_index < world.chain_shapes[chain_index as usize].materials.len());
    world.chain_shapes[chain_index as usize].materials[material_index] = material;

    let (material_count, count) = {
        let chain = &world.chain_shapes[chain_index as usize];
        (chain.materials.len(), chain.shape_indices.len())
    };
    debug_assert!(material_count == 1 || material_count == count);

    if material_count == 1 {
        for i in 0..count {
            let shape_id = world.chain_shapes[chain_index as usize].shape_indices[i];
            world.shapes[shape_id as usize].material = material;
        }
    } else {
        let shape_id = world.chain_shapes[chain_index as usize].shape_indices[material_index];
        world.shapes[shape_id as usize].material = material;
    }
}

/// Get a chain surface material by segment index. (b2Chain_GetSurfaceMaterial)
pub fn chain_get_surface_material(
    world: &World,
    chain_id: ChainId,
    segment_index: usize,
) -> SurfaceMaterial {
    let chain_index = get_chain_index(world, chain_id);
    let chain = &world.chain_shapes[chain_index as usize];
    // C asserts against the segment count and indexes the material array
    // directly; with a single shared material any index above 0 would read
    // past the C allocation, so a shared material is returned for all
    // segments here.
    debug_assert!(segment_index < chain.shape_indices.len());
    if chain.materials.len() == 1 {
        chain.materials[0]
    } else {
        chain.materials[segment_index]
    }
}
