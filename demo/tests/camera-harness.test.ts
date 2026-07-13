// Harness parity: interactive camera math + transport pause keys match C
// samples/main.cpp + draw.c ResetView / ScrollCallback.

import { describe, expect, test } from "bun:test";
import {
  CAMERA_PAN_STEP,
  CAMERA_SCROLL_ZOOM,
  CAMERA_ZOOM_MAX,
  CAMERA_ZOOM_MIN,
  createSampleTransport,
  DEFAULT_CAMERA,
  makeCamera,
  resetCameraView,
  screenToWorld,
  zoomCameraAtScreen,
} from "../src/demos/sample-shell.ts";

function fakeCanvas(width: number, height: number): HTMLCanvasElement {
  return { width, height } as HTMLCanvasElement;
}

describe("camera harness (C main.cpp / draw.c)", () => {
  test("ResetView restores GetDefaultCamera center and zoom", () => {
    const cam = makeCamera(3, 4, 12);
    resetCameraView(cam);
    expect(cam.centerX).toBe(DEFAULT_CAMERA.centerX);
    expect(cam.centerY).toBe(DEFAULT_CAMERA.centerY);
    expect(cam.zoom).toBe(DEFAULT_CAMERA.zoom);
  });

  test("arrow pan step is 0.5 world units", () => {
    expect(CAMERA_PAN_STEP).toBe(0.5);
  });

  test("scroll zoom keeps the world point under the cursor fixed", () => {
    const canvas = fakeCanvas(800, 600);
    const cam = makeCamera(0, 10, 5);
    const px = 200;
    const py = 150;
    const before = screenToWorld(cam, canvas, px, py);
    zoomCameraAtScreen(cam, canvas, px, py, 1 / CAMERA_SCROLL_ZOOM);
    const after = screenToWorld(cam, canvas, px, py);
    expect(after.x).toBeCloseTo(before.x, 5);
    expect(after.y).toBeCloseTo(before.y, 5);
    expect(cam.zoom).toBeCloseTo(5 / CAMERA_SCROLL_ZOOM, 5);
  });

  test("scroll zoom multiplies by 1.1 without clamping (C ScrollCallback)", () => {
    const canvas = fakeCanvas(400, 300);
    const cam = makeCamera(0, 0, 90);
    zoomCameraAtScreen(cam, canvas, 200, 150, CAMERA_SCROLL_ZOOM);
    expect(cam.zoom).toBeCloseTo(90 * CAMERA_SCROLL_ZOOM, 5);
    // Held Z/X clamps are separate constants matching main.cpp.
    expect(CAMERA_ZOOM_MAX).toBe(100);
    expect(CAMERA_ZOOM_MIN).toBe(0.5);
  });
});

describe("transport pause keys (C SPACE, web P alias)", () => {
  test("SPACE and P both toggle pause; O single-steps", () => {
    const transport = createSampleTransport();
    expect(transport.paused).toBe(false);

    const target = {
      addEventListener(_type: string, fn: (e: KeyboardEvent) => void) {
        (target as { _onKey?: (e: KeyboardEvent) => void })._onKey = fn;
      },
      removeEventListener() {},
      dispatchEvent() {
        return true;
      },
    } as unknown as Window;

    const unbind = transport.bindKeys(target);
    const onKey = (target as unknown as { _onKey: (e: KeyboardEvent) => void })._onKey;

    const fire = (key: string) => {
      const e = {
        key,
        target: { tagName: "BODY" },
        preventDefault() {},
      } as unknown as KeyboardEvent;
      onKey(e);
    };

    fire(" ");
    expect(transport.paused).toBe(true);
    fire("p");
    expect(transport.paused).toBe(false);
    fire("o");
    expect(transport.paused).toBe(true);
    expect(transport.singleStep).toBe(true);
    expect(transport.consumeStepDt()).toBeCloseTo(1 / 60, 6);
    expect(transport.singleStep).toBe(false);
    expect(transport.consumeStepDt()).toBe(0);

    unbind();
  });
});
