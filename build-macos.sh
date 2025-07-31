#!/bin/bash

# AVBD Demo 3D - macOS Build Script
# This script automates the complete build process for macOS

set -e  # Exit on any error

echo "=== AVBD Demo 3D - macOS Build Script ==="

# Check if running on macOS
if [[ "$OSTYPE" != "darwin"* ]]; then
    echo "Error: This script is designed for macOS systems"
    exit 1
fi

# Initialize submodules
echo "Initializing git submodules..."
git submodule update --init --recursive

# Install dependencies
echo "Installing dependencies..."
# Check if Homebrew is installed
if ! command -v brew &> /dev/null; then
    echo "Error: Homebrew is required but not installed"
    echo "Install Homebrew from: https://brew.sh"
    exit 1
fi

brew install sdl2 pkg-config

# Create build directory
echo "Creating build directory..."
rm -rf build
mkdir build
cd build

# Configure and build
echo "Configuring with CMake..."
cmake .. -DCMAKE_CXX_FLAGS="-F/Library/Frameworks -framework OpenGL"

echo "Building..."
cmake --build . --config Release --verbose

echo "=== Build Complete ==="
echo "Executable: $(pwd)/avbd_demo3d"

# Make executable runnable
chmod +x avbd_demo3d

echo "Run with: ./build/avbd_demo3d"