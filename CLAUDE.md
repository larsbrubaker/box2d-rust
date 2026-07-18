# Claude Code Guidelines

## Philosophy

**Quality through iterations** - Start with correct implementations, then improve. Code that doesn't matter can be quick and dirty. But code that matters *really* matters—treat it with respect and improve it meticulously. In a porting project, every function matters.

**Circumstances alter cases** - Use judgment. There are no rigid rules—context determines the right approach. However, this project has strong defaults because porting demands precision.

**No stubs, no shortcuts** - Every function must be complete and production-ready. No `todo!()`, no `unimplemented!()`, no `panic!("not implemented")`, no partial implementations. If dependencies aren't ready, stop and implement them first.

## Test-First Bug Fixing (Critical Practice)

**This is the single most important practice for agent performance and reliability.**

When a bug is reported, always follow this workflow:

1. **Write a reproducing test first** - Create a test that fails, demonstrating the bug
2. **Fix the bug** - Make the minimal change needed to address the issue
3. **Verify via passing test** - The previously failing test should now pass

**Do not skip the reproducing test.** Even if the fix seems obvious, the test validates your understanding and prevents regressions.

## Testing

- Tests MUST test actual production code, not copies - Never duplicate production logic in tests. Import and call the real code.
- Tests should run as fast as possible—fast tests get run more often
- Write tests for regressions and complex logic
- All tests must pass before merging
- Tests must verify **exact behavioral match** with the C implementation
- Port the C test suite (`box2d-cpp-reference/test/`) module by module alongside the code it tests
- When test failures occur, use the fix-test-failures agent (`.claude/agents/fix-test-failures.md`) — it treats all failures as real bugs and resolves them through instrumentation and root cause analysis, never by weakening tests

**Running tests:**
```bash
# Run all tests
cargo test

# Run tests for a specific module
cargo test --lib math_functions_tests

# Run a specific test
cargo test test_name -- --exact

# Run with output
cargo test -- --nocapture

# Run with the double-precision (large world) feature, mirroring BOX2D_DOUBLE_PRECISION
cargo test --features double-precision
```

## Code Quality

**Names** - Follow Rust conventions (`snake_case` for functions/variables, `PascalCase` for types, `SCREAMING_SNAKE_CASE` for constants). Keep names mappable to the C original: `b2ClampFloat` → `clamp_float`, `b2Vec2` → `Vec2`, `b2InvMulTransforms` → `inv_mul_transforms`. A reader diffing Rust against C should never have to guess which function corresponds to which.

**Comments** - Explain *why*, not *what*. Preserve the C source's explanatory comments (they encode Erin Catto's reasoning). Comments explaining *why* the Rust approach differs from C are especially valuable.

**Refactoring** - Improve code when it serves a purpose, not for aesthetics. A tripped file-length test means split the file into real modules — never compact lines or bump the limit.

## C to Rust Porting Rules

This project is a strict port of the Box2D v3 C library to Rust. These rules ensure fidelity:

### The C Reference

The exact source being ported is the git submodule at `box2d-cpp-reference/` (pinned, currently v3.1.1+ at `56edae7`). **Always read the pinned submodule, not the upstream website** — upstream moves. Layout:

- `src/*.c` + `src/*.h` — internal implementation (the bulk of the port)
- `include/box2d/*.h` — public API and inline functions (port these with the module that owns them)
- `test/test_*.c` — the C test suite (port alongside each module)
- `samples/` — the interactive samples app (becomes the wasm demo site)

### Exact Behavioral Matching
- Rust implementation must match C behavior exactly: same algorithms, same arithmetic, same edge cases
- **Float precision is behavior.** Box2D does all math in `f32`. Never promote to `f64` for "extra accuracy", never reorder floating-point operations, never replace `x * x + y * y` with `hypot`. `(a + b) + c ≠ a + (b + c)` in floats.
- **Determinism is a feature.** Box2D hand-rolls `b2Atan2` and `b2ComputeCosSin` specifically for cross-platform determinism. Never replace them with `f32::atan2` / `f32::sin_cos` — port the approximations bit-for-bit.
- `BOX2D_DOUBLE_PRECISION` (large world mode) maps to the `double-precision` cargo feature; both configurations must build and pass tests.
- C `B2_ASSERT` / `B2_VALIDATE` map to `debug_assert!`.
- Box2D's arena allocators, free lists, and index-based object pools should be ported keying by `Vec` index, never by pointer/reference juggling.

### Dependency-Ordered Implementation
Before implementing any function:
1. Read the corresponding C source to identify all functions called by the target function
2. Verify all dependencies are already implemented and tested in the Rust codebase
3. If any dependency is incomplete, implement dependencies first

Port phase-by-phase in complete, testable modules (this worked for clipper2-rust; function-by-function tracking did not). The collision layer (math_functions → core/constants → aabb/hull/geometry → distance/manifold → dynamic_tree → id_pool/types) was ported this way, one green module per commit.

### The Dynamics Core (body/shape/contact/joint/island/solver_set/constraint_graph/solver/world)
These C files are mutually recursive — every function takes `b2World*` and the Sim/State structs reference each other — so the module-per-commit rule cannot apply. For this unit the rules are:

1. **Data model first.** Land the complete internal data model (all structs/enums for the whole unit, including `World`) in one commit. Structs carry no logic yet; that is not a "stub" — functions are where the stub rule applies.
2. **Logic in complete C-file slices.** Port one C file's functions per commit, each function complete (no `todo!()`, no placeholder bodies). A ported function may call a function from a C file not yet ported **only if** that callee is ported in the same commit or the call is behind the not-yet-reachable public API. Every commit must compile (`cargo build`) and keep all existing tests green.
3. **Unreachable is acceptable, incomplete is not.** During bring-up, complete functions that nothing calls yet are expected (`#[allow(dead_code)]` at the module level with a `// bring-up:` note, removed when the world API lands).
4. **World tests gate completion.** `test_world.c` and `test_determinism.c` are the acceptance tests for the whole unit. The unit is not "done" until they pass; the demo site gets the Bodies/Stacking samples only after that.

### Forbidden Patterns
- `todo!()` or `unimplemented!()` macros
- `panic!()` for missing functionality
- Stub functions or placeholder implementations
- Implementing without dependencies ready (outside the dynamics-core bring-up rules above)
- Marking functions complete prematurely
- "Close enough" or "good enough for now" implementations
- Guessing at divergences — when Rust and C disagree, instrument both and diff the traces

## Shell

This project uses **PowerShell** on Windows. Heredocs (`<<'EOF'`) don't work — use PowerShell string variables with backtick-n (`` `n ``) for newlines instead.

## Demo Site

The wasm demo site (`demo/`) mirrors the C `samples/` app: every sample category (bodies, shapes, joints, stacking, events, continuous, geometry, collision, character, determinism, robustness, benchmark) eventually gets an interactive browser demo. Build with `bun run build` in `demo/`, develop with `bun run dev`. Deployed to GitHub Pages by `.github/workflows/deploy-demo.yml` on push to main.

## Orchestration pattern

The main session (Fable 5) acts as planner and orchestrator only — it should not write or edit code directly. All implementation is delegated to the **implementer** subagent (`.claude/agents/implementer.md`), one scoped step at a time. All post-change review is delegated to the **reviewer** subagent (`.claude/agents/reviewer.md`). The main session handles only planning, architecture decisions, and synthesizing subagent results.
