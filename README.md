# AVBD Demo 3D — experimental physics core

A **headless, dependency-free C++17** implementation of 3D rigid-box contacts based on the ideas of Augmented Vertex Block Descent (AVBD). This project replaces the earlier SDL/OpenGL/ImGui demo with a smaller physics library, reproducible tests, and a 96-box sky-drop capture scene.

**Status:** experimental proof of concept, not a finished game physics engine, not an exact reproduction of the full SIGGRAPH 2025 AVBD algorithm, and not GPU-accelerated.

## Build and test — no GitHub Actions required

Requires CMake 3.16+ and a C++17 compiler (MSVC, GCC, or Clang).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

On Windows, executables are commonly in `build/Release/`. On Linux/macOS, they are commonly in `build/`.

Programs:

- `avbd3d_tests`: 14 automated baseline physics tests.
- `avbd3d_demo [steps]`: a five-box stack simulation (default 360 steps).
- `avbd3d_stress [side] [steps]`: `side³` stacked boxes (default 10³ boxes, 120 steps).
- `avbd3d_sky_capture`: 96 boxes dropping onto a floor, writes JSON frames to standard output.
- `avbd3d_limitations`: demonstrates known high-speed tunnelling. **This failure is expected**, not part of the passing test suite.

To capture the sky-drop scene, run from the repo root (adapt binary path on Windows):

```sh
./build/avbd3d_sky_capture > sky_trace.json
```

Optional MP4 rendering (requires Python 3, Pillow and an `ffmpeg` executable in PATH):

```sh
python -m pip install Pillow
python tools/render_sky.py sky_trace.json sky_drop.mp4
```

The 360 captured frames represent **6 seconds of simulated time** at 60 captured frames/second (the solver steps at 120 Hz). The renderer outputs **12 seconds of video at 30 fps**, i.e. half-speed playback so collisions are easier to inspect.

## Implemented

- Full 3D position and quaternion orientation with mass and 3D box inertia.
- Fixed-step inertial prediction and per-body 6×6 primal solver with augmented contact dual variables.
- OBB SAT collision detection (15 axes), face clipping and edge–edge closest points.
- Persistent manifolds up to four contacts per pair with normal and two tangential friction directions.
- Sweep-and-prune broadphase (single-threaded); headless simulation, deterministic scenarios and tests.

## Known limitations — do not treat these as solved

- **No continuous collision detection:** fast small objects can pass through thin walls.
- **No restitution:** collisions mostly stop instead of bouncing.
- No general joints, springs, motors, meshes, convex hulls, sleeping, GPU, or multithreading.
- The exact full AVBD 3D rotational geometric Hessian is not implemented; a positive-definite Gauss–Newton approximation is used.
- Contact manifolds, friction and high-energy impacts still need more validation.
- 1,000 stacked dynamic boxes took approximately **53 ms/step** on the original test machine (24 solver iterations + 20 post-stabilization); too slow for a 60 Hz game under that workload.
- There is **no graphical/editor application** in this revision. The `avbd3d_sky_capture` executable emits recorded simulation data, not a live viewer.

See [docs/VALIDATION.md](docs/VALIDATION.md) for test scope and limitations.

## References and credits

- [AVBD SIGGRAPH 2025 research](https://graphics.cs.utah.edu/research/projects/avbd/) — Chris Giles, Elie Diaz, Cem Yuksel.
- [Original educational AVBD 2D demo](https://github.com/savant117/avbd-demo2d) — Chris Giles.
- [Educational AVBD 3D reference](https://github.com/savant117/avbd-demo3d).

Independent implementation inspired by these works; not a verbatim port. MIT licensed; see [LICENSE](LICENSE).

## Repository history

This commit deliberately replaces the former graphical demo and its SDL/ImGui submodules. Previous versions remain accessible in Git history. No GitHub Actions are configured in this personal repository. Build locally or run the CMake commands in an organization-owned CI project if desired.
