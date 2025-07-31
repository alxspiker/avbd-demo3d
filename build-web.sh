#!/bin/bash

# AVBD Demo 3D - Web Build Script (Emscripten)
# This script automates the complete build process for Web deployment

set -e  # Exit on any error

echo "=== AVBD Demo 3D - Web Build Script ==="

# Initialize submodules
echo "Initializing git submodules..."
git submodule update --init --recursive

# Check if Emscripten is installed and activated
if ! command -v emcc &> /dev/null; then
    echo "Error: Emscripten is not installed or not activated"
    echo "Please install Emscripten SDK from: https://emscripten.org/docs/getting_started/downloads.html"
    echo "And activate it with: source path/to/emsdk/emsdk_env.sh"
    exit 1
fi

# Install Ninja if not present (Linux/macOS)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if ! command -v ninja &> /dev/null; then
        echo "Installing Ninja build system..."
        sudo apt-get update && sudo apt-get install -y ninja-build
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v ninja &> /dev/null; then
        echo "Installing Ninja build system..."
        brew install ninja
    fi
fi

# Create build directory for web
echo "Creating web build directory..."
rm -rf build-web
mkdir build-web
cd build-web

# Configure and build
echo "Configuring with emcmake..."
emcmake cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s EXPORT_NAME=avbd_demo3d" ..

echo "Building with Ninja..."
ninja --verbose

echo "=== Build Complete ==="
echo "Web files generated in: $(pwd)/"
echo "Files created:"
ls -la avbd_demo3d.*

echo ""
echo "To run the web version:"
echo "1. Start a local web server in the build-web directory"
echo "2. For example: python3 -m http.server 8000"
echo "3. Open http://localhost:8000/avbd_demo3d.html in your browser"