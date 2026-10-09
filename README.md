# AVBD Demo 3D — experimental physics core

An independent, **headless C++17** experimental 3D physics engine based on the ideas of [Augmented Vertex Block Descent (AVBD)](https://graphics.cs.utah.edu/research/projects/avbd/) and Chris Giles' educational [2D AVBD demo](https://github.com/savant117/avbd-demo2d). The old SDL/OpenGL prototype was replaced with a testable physics library.

**Status:** early-stage engine, **not a complete implementation of the SIGGRAPH AVBD paper** and not comparable yet to the research project's GPU throughput. Do not mistake the rendered videos for correctness proofs. A reproducible, measured core is the priority.

## Manual build — no GitHub Actions needed

Requires a C++17 compiler and CMake 3.16+:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

Windows often puts executables in `build/Release/`; Linux/macOS usually put them directly in `build/`.

### Executables

| Program | Purpose |
| --- | --- |
| `avbd3d_tests` | 14 core physics regressions |
| `avbd3d_advancement` | 18 additional checks for restitution, shapes, substeps, joints, sleeping, scene-specific problems |
| `avbd3d_demo` | Small stack simulation |
| `avbd3d_stress` | Configurable box stress scene |
| `avbd3d_benchmark` | Resting-pile speed measurement with sleeping enabled/disabled |
| `avbd3d_stages 1..5` | Emit actual 3D simulation frame data for each development stage as JSON |
| `avbd3d_sky_capture` | Original 96-box sky-drop data |
| `avbd3d_limitations` | Intentionally reproduces a *known* failure if adaptive substepping is disabled |

Capture/render the stage videos (Pillow and ffmpeg required for *rendering only*, not for the physics engine):

```sh
./build/avbd3d_stages 4 > stage4.json
python -m pip install Pillow
python tools/render_stages.py stage4.json stage4.mp4
```

### Development stages

1. **Elastic contacts:** 84 boxes, restitution, impact-induced rotation, persistent frictional contact.
2. **Fast projectiles:** 22 spheres strike a 0.09-unit-thick wall using optional adaptive discrete substeps. Regressions check every simulation tick that no projectile crosses the wall in this scenario. **This is not swept/continuous CCD.**
3. **Sphere demolition:** A 132-brick wall struck by two moving spheres, testing mixed sphere–box and box–box contacts.
4. **Attached wrecking ball:** 13 distance links hold a ball while it strikes a stack. All links are unbreakable **in this demonstration**. Fracture is still supported and separately unit-tested.
5. **Sleeping and waking:** An initially resting body group sleeps and wakes when struck by a moving sphere.

All stage recordings use 360 recorded frames produced by advancing the C++ solver four steps per frame, corresponding to about 12 seconds of simulation at the default 120 Hz solver rate. The MP4 renderer outputs 12 seconds at 24 fps from this saved trace; frame rendering never invents physics trajectories.

## Implemented so far

- Rigid box and sphere bodies with full 3D translation, quaternions, world inertia and angular response.
- AVBD-inspired per-body six-degree-of-freedom iterative primal solves, dual variables, penalty ramping and post-stabilization.
- 15-axis OBB SAT, clipped face contacts, box edges, sphere–sphere and sphere–box contacts.
- Contact manifold persistence, normal and paired tangential rows, approximate Coulomb disc projection, friction.
- Restitution impulses for newly closing contacts (a deliberately **hybrid extension**, not part of the 2D AVBD reference).
- Sweep-and-prune broadphase; configurable adaptive **discrete** substepping to reduce tunnelling.
- Distance joints, configurable fracture force and collision filtering between adjacent linked bodies.
- Optional resting-body sleeping and impact wakeup.
- Automated physics tests, stress scenes, measurements and offline video capture.

## Important limitations and observed behaviour

- **Exact swept continuous collision detection is still missing.** Very fast/thin contacts can tunnel if a motion exceeds the substep budget; adaptive substeps are not a universal guarantee.
- **Joint constraints are limited:** no motor/hinge/ball-socket-specific joints, ragdoll or articulation islands yet.
- No collision meshes, convex hulls, capsules, continuous shape casts, materials API, game-engine integration, multithreading or GPU implementation.
- Full rigorous AVBD rotational geometric Hessians have not been derived and validated against the paper; the current six-DOF solve uses a positive-definite approximation.
- Contact/friction energy, extreme mass ratios, high-speed and persistent pile behaviour require more extensive validation. A leaning box in a pile can legitimately be supported by adjacent boxes; isolated tipped boxes are tested to settle flat. We do **not** snap orientations to be axis-aligned.
- Sleeping can speed up *already resting* piles but does not speed up sustained chaotic collisions. Demonstration benchmarks depend strongly on hardware and workload.
- No live GUI/editor; captured JSON and rendered MP4s are diagnostic tools.

See [docs/VALIDATION.md](docs/VALIDATION.md) for test specifics, honest failure modes and benchmark scope. No workflow is installed or required on the owner's personal GitHub repository.

## Credits and license

Educational lineage: Chris Giles and `savant117/avbd-demo2d`; AVBD research by Chris Giles, Elie Diaz and Cem Yuksel. The engine is an independent experimental implementation rather than the official research code. MIT license; see [LICENSE](LICENSE).
