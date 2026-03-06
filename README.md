# AVBD Demo 3D

3D rigid-body AVBD-style sandbox in C++17 with SDL2 + OpenGL + ImGui.

This repository is a practical 3D adaptation of the AVBD demo style used in
`savant117/avbd-demo2d`, focused on box-on-box contact behavior, solver
stability experiments, and stress testing.

## Project Lineage

- Started as a 3D conversion of `savant117/avbd-demo2d`
- Now actively aligned with the 3D reference repository:
  `https://github.com/savant117/avbd-demo3d`
- Recent updates in this repo were made to better reflect that 3D reference,
  while keeping project-specific behavior and tuning goals

## Alignment vs Upstream 3D

Recent areas brought closer to `savant117/avbd-demo3d`:

- SAT OBB collision flow with face/edge axis selection
- Contact manifold generation and warmstart feature matching behavior
- AVBD-style iterative solve structure and broadphase manifold creation logic
- OBB picking/raycast support used for interaction

Intentional differences in this repository:

- Row-based force API (`getRowCount`, `computeConstraint`,
  `computeDerivatives`) instead of upstream `updatePrimal/updateDual`
- Custom solver behavior for stability tuning (6x6 body solve, damping,
  diagnostics, manifold penalty cap)
- Contact manifold capped at 4 contacts here (upstream uses up to 8)
- Focused scene set plus stress diagnostics (`TwoBlockDrop`, `Stress1000`)
  instead of the full upstream demo scene catalog (rope/bridge/breakable/etc.)
- Headless/CLI simulation mode in `main.cpp` (`--nogfx`, `--scene`, `--steps`)
- Extra build automation scripts for Windows/Linux/macOS/Web
- Simpler desktop-first rendering/input path (no touch/mobile gesture path or
  projected shadow pipeline from upstream main loop)

## Current Status

Implemented and active in the current code:

- 3D rigid boxes with quaternion orientation and full 6-DOF state
- SAT-based OBB collision detection (face and edge axes)
- Contact manifold generation with up to 4 contacts
- Normal + two tangent rows per contact with friction cone clamping
- Iterative AVBD-style solve with warmstarting and penalty ramping
- Solver diagnostics in GUI and headless modes
- Scene presets including a 1000-block stress scene

Present in code but not fully wired into gameplay scenes:

- `Joint`, `Spring`, and `IgnoreCollision` force types
- `Rod (WIP)` and `Soft Body (WIP)` scenes are placeholders

## Scene List

Scene names are case-sensitive when passed through `--scene`:

- `Empty`
- `Ground`
- `Stack`
- `Pyramid`
- `Wall`
- `TwoBlockDrop`
- `Stress1000`
- `Rod (WIP)`
- `Soft Body (WIP)`

## Build

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
```

Platform scripts:

- Windows: `build-windows.bat`
- Linux: `./build-linux.sh`
- macOS: `./build-macos.sh`
- Web (Emscripten): `./build-web.sh`

Manual desktop build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --verbose
```

Incremental rebuild from the build directory:

```bash
cd build
cmake --build . --config Release --verbose
```

## Run

GUI:

- Windows: `build\\Release\\avbd_demo3d.exe`
- Linux/macOS: `./build/avbd_demo3d`

Headless examples:

```bash
./build/avbd_demo3d --nogfx --scene Stack --steps 300
./build/avbd_demo3d --headless --scene Stress1000 --steps 600
```

Command-line options:

- `--nogfx` or `--headless`: disable graphics and run simulation only
- `--scene <name>` or `-s <name>`: choose initial scene
- `--steps <n>` or `-n <n>`: number of steps in headless mode (default `300`)

Notes:

- In headless mode, diagnostics are forced on every step.
- Headless output is intentionally verbose and prints body state each step.

## Controls (GUI)

- Middle mouse drag: orbit camera
- Shift + middle mouse drag: pan camera
- Mouse wheel: zoom camera
- Right click: spawn a box near the camera target
- ImGui panel: change scene and solver parameters

## Stress Testing

Use `Stress1000` for high-load behavior:

- Spawns `10 x 10 x 10 = 1000` dynamic cubes
- Applies scene-specific solver tuning (`iterations=20`, `beta=30000`,
  `gamma=0.995`) to reduce immediate blow-ups during mass contact events

## Project Layout

```text
source/
  main.cpp         App entry point, UI, camera, CLI, loop
  solver.h/.cpp    Core solver, constraints, diagnostics
  rigid.cpp        Rigid body representation and rendering
  collision.cpp    SAT OBB collision + contact generation
  manifold.cpp     Contact constraint rows and friction handling
  joint.h/.cpp     6-DOF weld-like joint (available, limited scene usage)
  spring.h/.cpp    Distance constraint spring (available, limited scene usage)
  scenes.h         Scene definitions and stress test presets
  maths.h          vec3/quat/mat3 math utilities
```

## Known Limitations

- Collision shapes are OBB boxes only.
- Broadphase is pairwise `O(n^2)`.
- Very dense impact clusters can still destabilize and require tuning.
- WIP scenes are scaffolded but not feature-complete.

## Credits

- Chris Giles (`savant117`) for the original AVBD demo lineage
- University of Utah Graphics Lab for AVBD research context
- SDL2 and Dear ImGui for platform/window/UI support

## License

MIT. See [LICENSE](LICENSE).
