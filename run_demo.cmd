@echo off
REM Build the WASM demo and serve it at http://localhost:3000.
REM Double-click this file, or run it from a terminal. Press Ctrl+C to stop.
setlocal

cd /d "%~dp0demo"

where bun >nul 2>nul || (
  echo [run_demo] Bun is required but was not found on PATH.
  echo           Install it from https://bun.sh and try again.
  pause
  exit /b 1
)

where wasm-pack >nul 2>nul || (
  echo [run_demo] wasm-pack is required but was not found on PATH.
  echo           Install it with: cargo install wasm-pack
  pause
  exit /b 1
)

if not exist node_modules (
  echo [run_demo] Installing demo dependencies...
  call bun install || (echo [run_demo] bun install failed. & pause & exit /b 1)
)

echo [run_demo] Building WASM from the Rust port...
call bun run build:wasm || (echo [run_demo] WASM build failed. & pause & exit /b 1)

REM Open the browser shortly after the server has had time to start.
start "" cmd /c "timeout /t 2 >nul & start "" http://localhost:3000"

echo [run_demo] Serving at http://localhost:3000  (Ctrl+C to stop)
call bun run dev
