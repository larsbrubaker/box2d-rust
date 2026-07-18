// User-facing DEBUG / RELEASE body-count switch for the Partial benchmark samples.
//
// The C samples pick their body counts by NDEBUG: shared/benchmarks.c uses
// `BENCHMARK_DEBUG ? debug : release` and sample_benchmark.cpp uses `m_isDebug`.
// The wasm demo defaults to the DEBUG counts so the Partial scenes stay
// real-time in the browser; flipping to RELEASE reproduces the larger scene the
// C release (NDEBUG) build runs. Default stays "debug".

export type CountsMode = "debug" | "release";

const STORAGE_COUNTS_MODE = "box2d.countsMode";

/** Current counts mode from localStorage (default "debug"). */
export function getCountsMode(): CountsMode {
  try {
    return localStorage.getItem(STORAGE_COUNTS_MODE) === "release" ? "release" : "debug";
  } catch {
    // Quota / private mode — fall back to the default.
    return "debug";
  }
}

/** Persist the counts mode; the caller triggers a scene rebuild afterwards. */
export function setCountsMode(mode: CountsMode): void {
  try {
    localStorage.setItem(STORAGE_COUNTS_MODE, mode);
  } catch {
    // Quota / private mode — ignore.
  }
}

/** Pick the count for the current mode (C `BENCHMARK_DEBUG ? debugVal : releaseVal`). */
export function pickCount(debugVal: number, releaseVal: number): number {
  return getCountsMode() === "release" ? releaseVal : debugVal;
}
