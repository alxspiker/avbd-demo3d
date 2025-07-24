# AVBD Demo 3D

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Language](https://img.shields.io/badge/language-C%2B%2B17-orange.svg)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS%20%7C%20Web-brightgreen.svg)

---

This project is a 3D rigid body physics engine built from scratch in C++. It implements the **Augmented Vertex Block Descendent (AVBD)** algorithm, bringing the concepts from the original 2D demo into a fully 3D environment.

<p align="center">
  <a href="https://www.youtube.com/watch?v=soUKf1JE-40">
    <img src="https://img.youtube.com/vi/soUKf1JE-40/0.jpg" alt="AVBD Demo 3D in Action" width="720">
    <br>
    <em><strong>Watch the engine in action on YouTube!</strong></em>
  </a>
</p>

## About The Project

This repository contains a simple, clean, and easy-to-understand implementation of AVBD in 3D. It is not intended to be a hyper-optimized physics library, but rather an educational tool for developers to see how the technique can be translated from 2D to 3D.

The architecture and algorithms are a direct 3D conversion of the principles found in the original **[avbd-demo2d](https://github.com/savant117/avbd-demo2d)** by Chris Giles.

For more technical details on the AVBD technique itself, including the original paper, see the University of Utah project page: **[graphics.cs.utah.edu/research/projects/avbd/](https://graphics.cs.utah.edu/research/projects/avbd/)**

### Core Features

*   **3D AVBD Solver:** A full 6-DOF implementation of the iterative, position-based AVBD algorithm, including warm-starting, penalty updates, and stabilization parameters (`alpha`, `beta`, `gamma`).
*   **3D Collision:** Robust Oriented Bounding Box (OBB) collision detection using the Separating Axis Theorem (SAT) and a Box2D-style clipping manifold for stable stacking.
*   **Modern C++:** Built with C++17 and a clean, organized structure.
*   **Cross-Platform:** Uses CMake, SDL2, and Dear ImGui to build and run on Windows, Linux, macOS, and the Web (via Emscripten).
*   **Complete Constraint Suite:** A faithful 3D conversion of all original constraints:
    *   `Manifold` for collision, friction, and stacking.
    *   `Joint` for creating weld/fixed connections.
    *   `Spring` for distance constraints.
    *   `IgnoreCollision` for complex object setups.

## Building

The project uses CMake and depends on SDL2 and OpenGL/GLU.

### Prerequisites

*   A C++17 compatible compiler (GCC, Clang, MSVC).
*   CMake (3.13+).
*   Git.
*   **Linux:** `sudo apt-get install libsdl2-dev libglu1-mesa-dev`
*   **Windows (Web Build):** Ninja is recommended (`winget install Ninja-build.Ninja`).

### 1. Clone the Repository

The project uses submodules for dependencies, so clone it recursively:

```bash
git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
```

### 2. Build for Desktop (Windows, Linux, macOS)

```bash
# Create a build directory
mkdir build
cd build

# Configure the project
cmake ..

# Compile
cmake --build . --config Release
```

The final executable (`avbd_demo3d` or `Release/avbd_demo3d.exe`) will be inside the `build` directory.

### 3. Build for Web (Emscripten)

First, ensure you have the Emscripten SDK installed and activated.

```bash
# Create a separate build directory for the web version
mkdir build-web
cd build-web

# Configure the project with emcmake
emcmake cmake ..

# Compile with Ninja or Make
ninja
# or 'make' if you don't have ninja
```

This will generate `avbd_demo3d.html`, `.js`, and `.wasm` files. To run it, serve the files with a local web server and open the HTML file in your browser.

## Controls

*   **Orbit Camera:** Middle Mouse Button + Drag
*   **Pan Camera:** Shift + Middle Mouse Button + Drag
*   **Zoom Camera:** Mouse Wheel
*   **Create Box:** Right Mouse Click
*   **Drag Box:** *(Functionality requires 3D raycasting and is not yet implemented)*

## Acknowledgments

*   **Chris Giles** for the original 2D implementation and the clear demonstration of the AVBD algorithm.
*   **Erin Catto** for the robust collision algorithms in Box2D that were adapted for this project.
*   **Dear ImGui** and **SDL2** for the excellent libraries that make this demo possible.
