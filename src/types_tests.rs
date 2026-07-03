// Tests for the definition defaults. Box2D has no standalone test_types.c
// (test_world.c exercises these), so these lock in the b2Default* values.
//
// SPDX-License-Identifier: MIT

use crate::core::SECRET_COOKIE;
use crate::types::*;

#[test]
fn world_def_defaults() {
    let def = default_world_def();
    assert_eq!(def.gravity.x, 0.0);
    assert_eq!(def.gravity.y, -10.0);
    assert_eq!(def.contact_hertz, 30.0);
    assert_eq!(def.contact_damping_ratio, 10.0);
    assert!(def.enable_sleep);
    assert!(def.enable_continuous);
    assert!(!def.enable_contact_softening);
    assert_eq!(def.internal_value, SECRET_COOKIE);
    assert!(def.friction_callback.is_none());
    assert!(def.restitution_callback.is_none());

    // Default delegates to the free constructor (WorldDef holds fn-pointer
    // callbacks so it has no PartialEq; compare representative fields).
    let d2 = WorldDef::default();
    assert_eq!(d2.gravity, def.gravity);
    assert_eq!(d2.contact_hertz, def.contact_hertz);
    assert_eq!(d2.internal_value, def.internal_value);
}

#[test]
fn body_def_defaults() {
    let def = default_body_def();
    assert_eq!(def.type_, BodyType::Static);
    assert_eq!(def.rotation, crate::math_functions::ROT_IDENTITY);
    assert_eq!(def.gravity_scale, 1.0);
    assert!(def.enable_sleep);
    assert!(def.is_awake);
    assert!(def.is_enabled);
    assert!(def.enable_contact_recycling);
    assert!(!def.is_bullet);
    assert!(def.name.is_empty());
    assert_eq!(def.internal_value, SECRET_COOKIE);

    assert_eq!(BodyType::default(), BodyType::Static);
    assert_eq!(BODY_TYPE_COUNT, 3);
    assert_eq!(BodyType::Dynamic as i32, 2);
}

#[test]
fn shape_and_filter_defaults() {
    let def = default_shape_def();
    assert_eq!(def.material.friction, 0.6);
    assert_eq!(def.material.restitution, 0.0);
    assert_eq!(def.density, 1.0);
    assert!(def.update_body_mass);
    assert!(def.invoke_contact_creation);
    assert!(!def.is_sensor);
    assert_eq!(def.internal_value, SECRET_COOKIE);

    let filter = default_filter();
    assert_eq!(filter.category_bits, DEFAULT_CATEGORY_BITS);
    assert_eq!(filter.mask_bits, DEFAULT_MASK_BITS);
    assert_eq!(filter.group_index, 0);
    assert_eq!(def.filter, filter);

    let qf = default_query_filter();
    assert_eq!(qf.category_bits, DEFAULT_CATEGORY_BITS);
    assert_eq!(qf.mask_bits, DEFAULT_MASK_BITS);

    assert_eq!(default_surface_material().friction, 0.6);
}

#[test]
fn chain_def_defaults() {
    let def = default_chain_def();
    assert_eq!(def.materials.len(), 1);
    assert_eq!(def.materials[0].friction, 0.6);
    assert!(def.points.is_empty());
    assert!(!def.is_loop);
    assert_eq!(def.filter, default_filter());
    assert_eq!(def.internal_value, SECRET_COOKIE);
}
