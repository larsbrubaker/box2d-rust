//! World setting toggles used by C samples / Sample.cpp Step.
//! Mouse grab and debug-draw live on the harness `interact/` surface — do not
//! reimplement them here.

use super::SimWorld;
use box2d_rust::world::{
    world_enable_continuous, world_enable_sleeping, world_enable_speculative,
    world_enable_warm_starting, world_get_gravity, world_get_profile,
    world_get_restitution_threshold, world_is_warm_starting_enabled, world_set_contact_tuning,
    world_set_restitution_threshold,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl SimWorld {
    /// (b2World_EnableSleeping)
    pub fn set_sleeping(&mut self, flag: bool) {
        world_enable_sleeping(&mut self.world, flag);
    }

    /// (b2World_EnableWarmStarting)
    pub fn set_warm_starting(&mut self, flag: bool) {
        world_enable_warm_starting(&mut self.world, flag);
    }

    /// (b2World_IsWarmStartingEnabled)
    pub fn is_warm_starting_enabled(&self) -> bool {
        world_is_warm_starting_enabled(&self.world)
    }

    /// (b2World_EnableContinuous) — also exposed as `set_continuous`.
    pub fn set_continuous_collision(&mut self, flag: bool) {
        world_enable_continuous(&mut self.world, flag);
    }

    /// (b2World_EnableSpeculative)
    pub fn set_speculative(&mut self, flag: bool) {
        world_enable_speculative(&mut self.world, flag);
    }

    /// (b2World_SetContactTuning)
    pub fn set_contact_tuning(&mut self, hertz: f32, damping_ratio: f32, push_velocity: f32) {
        world_set_contact_tuning(&mut self.world, hertz, damping_ratio, push_velocity);
    }

    /// (b2World_SetRestitutionThreshold) — Continuous Restitution Threshold sample.
    pub fn set_restitution_threshold(&mut self, value: f32) {
        world_set_restitution_threshold(&mut self.world, value);
    }

    /// (b2World_GetRestitutionThreshold)
    pub fn get_restitution_threshold(&self) -> f32 {
        world_get_restitution_threshold(&self.world)
    }

    /// (b2World_GetGravity) as [gx, gy]
    pub fn get_gravity(&self) -> Vec<f32> {
        let g = world_get_gravity(&self.world);
        vec![g.x, g.y]
    }

    /// `b2Profile.step` from the last `world_step`, in milliseconds.
    /// (`b2World_GetProfile`)
    pub fn get_profile_step(&self) -> f32 {
        world_get_profile(&self.world).step
    }
}
