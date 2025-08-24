#!/bin/bash

# AVBD Demo 3D - Linux Build Script (enhanced)
# Features:
#  * Optional dependency installation ( --deps )
#  * Configurable build type ( --type Release|Debug|RelWithDebInfo )
#  * Optional clean rebuild ( --clean )
#  * Parallel build using all cores ( override with --jobs N )
#  * Safe defaults (no automatic apt install unless requested)
#  * Usage help ( --help )

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== AVBD Demo 3D - Linux Build Script ==="

if [[ "${OSTYPE:-}" != linux-gnu* ]]; then
    echo "Error: This script is designed for Linux systems" >&2
    exit 1
fi

# Defaults
BUILD_TYPE="Release"
DO_DEPS=0
CLEAN=0
JOBS="$(command -v nproc >/dev/null 2>&1 && nproc || echo 4)"
VERBOSE=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --deps              Install (apt) build dependencies
  --type <T>          CMake build type (Release, Debug, RelWithDebInfo) [default: Release]
  --clean             Remove existing build directory before configuring
  --jobs <N>          Parallel build jobs (default: detected cores = $JOBS)
  --verbose           Verbose CMake build output
  --no-parallel       Force single-threaded build
  --help              Show this help

Examples:
  $0 --deps --clean --type RelWithDebInfo
  $0 --type Debug --jobs 8
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --deps) DO_DEPS=1; shift ;;
        --type) BUILD_TYPE="$2"; shift 2 ;;
        --clean) CLEAN=1; shift ;;
        --jobs) JOBS="$2"; shift 2 ;;
        --verbose) VERBOSE=1; shift ;;
        --no-parallel) JOBS=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

echo "Config: type=$BUILD_TYPE clean=$CLEAN deps=$DO_DEPS jobs=$JOBS verbose=$VERBOSE"

echo "Initializing git submodules (recursive)..."
git submodule update --init --recursive

if (( DO_DEPS )); then
    echo "Installing dependencies (apt)..."
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libx11-dev \
        libxext-dev \
        libasound2-dev
else
    echo "(Skipping dependency installation; use --deps to enable)"
fi

if (( CLEAN )) && [[ -d build ]]; then
    echo "Removing existing build directory (clean rebuild)..."
    rm -rf build
fi

if [[ ! -d build ]]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

# Configure (idempotent)
if [[ ! -f CMakeCache.txt ]] || grep -qi "CMAKE_BUILD_TYPE" CMakeCache.txt | false; then
    echo "Configuring with CMake (type=$BUILD_TYPE)..."
    cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
else
    echo "(CMake cache exists; skipping configure — use --clean to force)"
fi

echo "Building (parallel jobs: $JOBS)..."
BUILD_CMD=(cmake --build . --config "$BUILD_TYPE")
if (( VERBOSE )); then BUILD_CMD+=(--verbose); fi
if [[ "$JOBS" != "1" ]]; then BUILD_CMD+=(-- -j"$JOBS"); fi
"${BUILD_CMD[@]}"

echo "=== Build Complete ==="
if [[ -f avbd_demo3d ]]; then
    chmod +x avbd_demo3d || true
    echo "Executable: $(pwd)/avbd_demo3d"
else
    echo "Warning: Executable not found (build may have failed earlier)." >&2
fi

echo "Run with: ./build/avbd_demo3d [--headless ...]"