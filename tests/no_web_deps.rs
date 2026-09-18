//! Guards GitHub issue #1: the library must be usable on non-browser WASM
//! hosts (Wasmtime, WASI runtimes, ...). Those hosts have no
//! `__wbindgen_placeholder__` module, so any `wasm-bindgen` import in the
//! dependency graph makes the module fail to instantiate:
//!
//! ```text
//! unknown import: `__wbindgen_placeholder__::__wbindgen_describe` has not been defined
//! ```
//!
//! With `default-features = false` the `wasm32-unknown-unknown` dependency
//! graph must therefore be free of `web-time` (and its `wasm-bindgen`
//! dependency). With default features it must still contain `web-time`, so
//! browser users keep a real `performance.now()` based profiling clock.

use std::process::Command;

fn cargo_tree(extra: &[&str]) -> String {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    let mut args = vec![
        "tree",
        "--locked",
        "-p",
        "box2d-rust",
        "--target",
        "wasm32-unknown-unknown",
        "-e",
        "normal",
    ];
    args.extend_from_slice(extra);

    let output = Command::new(&cargo)
        .args(&args)
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .output()
        .unwrap_or_else(|e| panic!("failed to run `{cargo} {}`: {e}", args.join(" ")));

    assert!(
        output.status.success(),
        "`cargo {}` failed:\n{}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr)
    );

    String::from_utf8(output.stdout).expect("cargo tree emitted non-utf8 output")
}

#[test]
fn wasm_tree_without_default_features_has_no_wasm_bindgen() {
    let tree = cargo_tree(&["--no-default-features"]);

    // Guard against a vacuous pass: the tree must actually be the box2d-rust one.
    assert!(
        tree.contains("box2d-rust"),
        "cargo tree did not report the box2d-rust root:\n{tree}"
    );
    assert!(
        !tree.contains("wasm-bindgen"),
        "wasm32-unknown-unknown dependency tree with --no-default-features must not \
         contain wasm-bindgen (breaks non-browser WASM hosts):\n{tree}"
    );
    assert!(
        !tree.contains("web-time"),
        "wasm32-unknown-unknown dependency tree with --no-default-features must not \
         contain web-time (it pulls in wasm-bindgen):\n{tree}"
    );
}

#[test]
fn wasm_tree_with_default_features_keeps_web_time() {
    let tree = cargo_tree(&[]);

    assert!(
        tree.contains("box2d-rust"),
        "cargo tree did not report the box2d-rust root:\n{tree}"
    );
    assert!(
        tree.contains("web-time"),
        "default features must keep the browser clock for wasm32-unknown-unknown:\n{tree}"
    );
}
