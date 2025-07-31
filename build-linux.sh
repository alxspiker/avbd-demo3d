#!/bin/bash

# AVBD Demo 3D - Linux Build Script
# This script automates the complete build process for Linux

set -e  # Exit on any error

echo "=== AVBD Demo 3D - Linux Build Script ==="

# Check if running on Linux
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "Error: This script is designed for Linux systems"
    exit 1
fi

# Initialize submodules
echo "Initializing git submodules..."
git submodule update --init --recursive

# Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libx11-dev \
    libxext-dev \
    libasound2-dev

# Create build directory
echo "Creating build directory..."
rm -rf build
mkdir build
cd build

# Configure and build
echo "Configuring with CMake..."
cmake ..

echo "Building..."
cmake --build . --config Release --verbose

echo "=== Build Complete ==="
echo "Executable: $(pwd)/avbd_demo3d"

# Make executable runnable
chmod +x avbd_demo3d

echo "Run with: ./build/avbd_demo3d"