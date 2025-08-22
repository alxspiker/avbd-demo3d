# AVBD Demo 3D

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Language](https://img.shields.io/badge/language-C%2B%2B17-orange.svg)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS%20%7C%20Web-brightgreen.svg)

> **⚠️ ARCHIVED PROJECT**  
> This project has been archived and is no longer actively maintained.  
> **Please use [@IsaacLagoy/AVBD3D](https://github.com/IsaacLagoy/AVBD3D) instead, which is a better implementation.**

**A comprehensive 3D conversion of the acclaimed AVBD 2D physics engine**

Test the latest version here: [**Live Web Demo**](https://github.com/alxspiker/avbd-demo3d/releases/tag/prerelease)

<p align="center">
  <a href="https://www.youtube.com/watch?v=lBtpCrjKPBQ">
    <img src="https://img.youtube.com/vi/lBtpCrjKPBQ/0.jpg" alt="AVBD Demo 3D in Action" width="720">
    <br>
    <em><strong>Watch the 3D physics engine in action!</strong></em>
  </a>
</p>

---

## About This Project

This project is a **direct 3D conversion** of Chris Giles' excellent [**avbd-demo2d**](https://github.com/savant117/avbd-demo2d) implementation. It faithfully translates the 2D Augmented Vertex Block Descent (AVBD) physics algorithm into a full 3D rigid body physics engine, maintaining the educational clarity and algorithmic purity of the original while extending it to six degrees of freedom.

The conversion preserves the core AVBD solver architecture while implementing the substantial changes required for 3D physics simulation, making this an ideal educational resource for understanding both the AVBD technique and the challenges of 2D-to-3D physics engine development.

### The AVBD Algorithm

AVBD is a constraint-based physics solver that uses iterative penalty methods with warm-starting to achieve stable, realistic physics simulation. For complete technical details, see the [**University of Utah project page**](https://graphics.cs.utah.edu/research/projects/avbd/).

## Key 2D to 3D Conversion Changes

This section highlights the major architectural and algorithmic changes made to extend the 2D engine to 3D:

### 🧮 Mathematics Framework

- **2D**: `float2` vectors, `float2x2` rotation matrices, scalar angles
- **3D**: `vec3` vectors, `mat3` rotation matrices, `quat` quaternions for orientation
- **Added**: Full 3D linear algebra including cross products, quaternion operations, and 6-DOF transformations

### 🏗️ Rigid Body Representation

- **2D**: Position (`float2`), rotation (`float`), 3 degrees of freedom
- **3D**: Position (`vec3`), orientation (`quat`), size (`vec3`), **6 degrees of freedom**
- **Added**: 3D inertia tensors, quaternion-based rotation, oriented bounding boxes

### 🎯 Collision Detection

- **2D**: Circle-circle, box-box using SAT in 2D space
- **3D**: **Oriented Bounding Box (OBB) collision** using 3D Separating Axis Theorem
- **Added**: 15 potential separating axes (6 face normals + 9 edge cross-products)
- **Enhanced**: 3D contact manifold generation with proper contact point clipping

### ⚙️ Constraint Solver

- **2D**: 3-DOF constraints (2 linear + 1 angular)
- **3D**: **6-DOF constraints** (3 linear + 3 angular)
- **Expanded**: All constraint types (manifold, joint, spring) support full 6-DOF
- **Enhanced**: Proper 3D Jacobian matrices and constraint equations

### 🎮 Rendering & Interaction

- **2D**: SDL2 with 2D graphics primitives
- **3D**: **OpenGL 3D rendering** with perspective projection
- **Added**: 3D camera system (orbit, pan, zoom), ImGui-based 3D scene controls
- **Cross-platform**: Windows, Linux, macOS, and Web (Emscripten) support

## Features

- **📚 Educational Focus**: Clean, readable code that clearly demonstrates 3D AVBD implementation
- **🔧 Complete Constraint System**: All constraint types from the 2D version, extended to 6-DOF
  - **Manifold**: 3D collision detection and response with friction
  - **Joint**: Fixed/weld constraints for connecting bodies
  - **Spring**: Distance constraints with proper 3D force application
  - **IgnoreCollision**: Selective collision filtering
- **🎯 Robust Collision**: SAT-based OBB detection with stable contact manifold generation
- **⚡ Cross-Platform**: CMake build system with automated build scripts
- **🌐 Web Ready**: Full Emscripten support for browser deployment

## Quick Start

### Automated Build Scripts

We provide build scripts that handle all dependencies and configuration:

#### Linux
```bash
git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
./build-linux.sh
```

#### Windows
```cmd
git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
build-windows.bat
```

#### macOS
```bash
git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
./build-macos.sh
```

#### Web (Emscripten)
```bash
git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
./build-web.sh
```

### Manual Build (Alternative)

If you prefer manual building or need to customize the process:

<details>
<summary>Manual Build Instructions</summary>

#### Prerequisites
- C++17 compatible compiler (GCC, Clang, MSVC)
- CMake 3.13+
- Git
- **Platform-specific dependencies:**
  - **Linux**: `sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev libx11-dev libxext-dev libasound2-dev`
  - **macOS**: `brew install sdl2 pkg-config`
  - **Windows**: vcpkg for OpenGL dependencies

#### Build Steps
1. **Clone with submodules:**
   ```bash
   git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
   cd avbd-demo3d
   ```

2. **Configure and build:**
   ```bash
   mkdir build && cd build
   cmake ..
   cmake --build . --config Release
   ```

3. **For Web (Emscripten):**
   ```bash
   mkdir build-web && cd build-web
   emcmake cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
   ninja
   ```

</details>

## Controls

- **🖱️ Orbit Camera**: Middle Mouse + Drag
- **📐 Pan Camera**: Shift + Middle Mouse + Drag  
- **🔍 Zoom**: Mouse Wheel
- **📦 Create Box**: Right Mouse Click
- **⚙️ Physics Controls**: Use ImGui panel for solver parameters
- **Drag Box**: Left Mouse Button

### Command-Line Options

You can run the simulation in headless mode or configure the initial state with the following command-line arguments:

| Option | Alias | Description | Example |
| :--- | :--- | :--- | :--- |
| `--nogfx` | `--headless` | Run the simulation without a graphical interface. | `avbd_demo3d --nogfx` |
| `--scene` | `-s` | Load the specified scene. | `avbd_demo3d --scene Stack` |
| `--steps` | `-n` | Run for a specified number of steps in headless mode. | `avbd_demo3d --nogfx --steps 100` |

### 📂 Project Structure

The codebase maintains the same clean organization as the original 2D version:

```
source/
├── main.cpp          # Application entry point and 3D rendering
├── maths.h           # 3D math library (vec3, quat, mat3)
├── solver.h/.cpp     # Core AVBD solver with 6-DOF constraints
├── rigid.cpp         # 3D rigid body representation
├── collision.cpp     # OBB collision detection using SAT
├── manifold.cpp      # 3D contact manifold generation
├── joint.h/.cpp      # 6-DOF joint constraints
├── spring.h/.cpp     # 3D spring/distance constraints
└── scenes.h          # Demo scene configurations
```

## Educational Value

This project serves as an excellent resource for:

- **🎓 Learning AVBD**: See how the algorithm translates from 2D to 3D
- **🔧 Physics Engine Development**: Understand the challenges of 3D constraint solving
- **📚 3D Mathematics**: Study practical applications of quaternions, rotation matrices, and 3D geometry
- **🏗️ Engine Architecture**: Explore clean, modular physics engine design patterns

## Acknowledgments

- **[Chris Giles](https://github.com/savant117)** - Original 2D AVBD implementation that made this 3D conversion possible
- **[University of Utah Graphics Lab](https://graphics.cs.utah.edu/research/projects/avbd/)** - AVBD algorithm research and development  
- **Erin Catto** - Box2D collision algorithms adapted for 3D OBB detection
- **Dear ImGui & SDL2** - Essential libraries for cross-platform development

---

*This 3D conversion maintains the educational spirit of the original 2D demo while showcasing the fascinating challenges of extending 2D physics algorithms to three dimensions.*
