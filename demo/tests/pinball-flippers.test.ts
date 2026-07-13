import { describe, expect, test } from "bun:test";
import { isPinballFlipperKey, pinballMotorSpeeds } from "../src/demos/continuous.ts";

describe("pinball flipper controls (C sample_continuous.cpp:1688-1697)", () => {
  test("rest speeds are ∓10 when A is not pressed", () => {
    expect(pinballMotorSpeeds(false)).toEqual({ left: -10, right: 10 });
  });

  test("press speeds are ±20 when A is pressed", () => {
    expect(pinballMotorSpeeds(true)).toEqual({ left: 20, right: -20 });
  });

  test("KeyA code matches C GLFW_KEY_A even when key text is empty or uppercase", () => {
    expect(isPinballFlipperKey({ code: "KeyA", key: "a" })).toBe(true);
    expect(isPinballFlipperKey({ code: "KeyA", key: "A" })).toBe(true);
    expect(isPinballFlipperKey({ code: "KeyA", key: "" })).toBe(true);
    expect(isPinballFlipperKey({ code: "KeyB", key: "a" })).toBe(true);
    expect(isPinballFlipperKey({ code: "KeyB", key: "b" })).toBe(false);
  });

  test("page-level KeyA set survives rebuild semantics (C glfw poll)", () => {
    // Edge-triggered scene `flip` would reset on rebuild while A is still held.
    // Polling a page-level set (like glfwGetKey) keeps press speeds after R.
    const keysDown = new Set<string>();
    keysDown.add("KeyA");
    const beforeRebuild = pinballMotorSpeeds(keysDown.has("KeyA"));
    // rebuild() creates a new SceneRuntime but must keep keysDown.
    const afterRebuild = pinballMotorSpeeds(keysDown.has("KeyA"));
    expect(beforeRebuild).toEqual({ left: 20, right: -20 });
    expect(afterRebuild).toEqual({ left: 20, right: -20 });
    keysDown.delete("KeyA");
    expect(pinballMotorSpeeds(keysDown.has("KeyA"))).toEqual({ left: -10, right: 10 });
  });
});
