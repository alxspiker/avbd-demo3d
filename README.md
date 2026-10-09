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
| `avbd3d_ccd_parallel_tests` | 19 additional CCD/parallel assertions |
| `avbd3d_parallel_benchmark [side] [layers] [steps]` | Serial vs colored CPU speed in the same pile scene |
| `avbd3d_milestones ccd\|parallel` | Reproduce new CCD comparison or 406-body parallel trace |
| `avbd3d_rotational_ccd_tests` | Rotating-contact assertions, including CCD-on/off impulse comparisons |
| `avbd3d_rotational_capture` | Stage 8 side-by-side, measured sphere/bar motion capture |

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
6. **Linear-translation CCD:** Event-driven swept sphere–sphere, sphere–OBB and OBB–OBB TOI, with a side-by-side discrete comparison. Not rotational CCD.
7. **Parallel CPU solver:** Graph-colored independent rigid-body updates using OpenMP when available; 405 blocks and one sphere in the stage video.
8. **Rotational CCD:** Eight controlled fast-spinning-bar contacts, side by side with CCD disabled. CCD produces a measured sphere impulse where the discrete path misses.

Stages 1–5 use 360 recorded frames produced by advancing the C++ solver four steps per frame, corresponding to about 12 seconds of simulation at the default 120 Hz solver rate. The MP4 renderer outputs 12 seconds at 24 fps from this saved trace; frame rendering never invents physics trajectories.

### New CCD / parallel videos

```sh
./build/avbd3d_milestones ccd > ccd_stage.json
./build/avbd3d_milestones parallel > parallel_stage.json
python tools/render_milestones.py ccd_stage.json ccd.mp4
python tools/render_milestones.py parallel_stage.json parallel.mp4
./build/avbd3d_parallel_benchmark 8 4 20
```

The first video compares CCD disabled and enabled in separate recorded C++ worlds, with scripted launches and measured position trails. The second is 406 moving bodies using four CPU threads (OpenMP must be available for actual parallelism). The engine remains experimental.

## Implemented so far

- Rigid box and sphere bodies with full 3D translation, quaternions, world inertia and angular response.
- AVBD-inspired per-body six-degree-of-freedom iterative primal solves, dual variables, penalty ramping and post-stabilization.
- 15-axis OBB SAT, clipped face contacts, box edges, sphere–sphere and sphere–box contacts.
- Contact manifold persistence, normal and paired tangential rows, approximate Coulomb disc projection, friction.
- Restitution impulses for newly closing contacts (a deliberately **hybrid extension**, not part of the 2D AVBD reference).
- Sweep-and-prune broadphase; optional adaptive **discrete** substepping, event-driven translation CCD for spheres and oriented boxes, and experimental rotational conservative advancement.
- Distance joints, configurable fracture force and collision filtering between adjacent linked bodies.
- Optional resting-body sleeping and impact wakeup.
- Graph-colored body solver with optional OpenMP parallel execution and serial fallback.
- Automated physics tests, stress scenes, measurements and offline video capture.

## Important limitations and observed behaviour

- **Robust general rotational/accelerating-body CCD remains missing.** Rotating boxes now use approximate constant-angular-velocity conservative advancement. This is not an exact continuous six-degree-of-freedom trajectory. Exhausting rotational-search iterations or the global TOI budget is reported; the discrete fallback still risks tunnelling.
- **Joint constraints are limited:** no motor/hinge/ball-socket-specific joints, ragdoll or articulation islands yet.
- No collision meshes, convex hulls, capsules, general rotational shape casts, materials API, game-engine integration, or GPU implementation.
- Full rigorous AVBD rotational geometric Hessians have not been derived and validated against the paper; the current six-DOF solve uses a positive-definite approximation.
- Contact/friction energy, extreme mass ratios, high-speed and persistent pile behaviour require more extensive validation. A leaning box in a pile can legitimately be supported by adjacent boxes; isolated tipped boxes are tested to settle flat. We do **not** snap orientations to be axis-aligned.
- Sleeping can speed up *already resting* piles but does not speed up sustained chaotic collisions. Demonstration benchmarks depend strongly on hardware and workload.
- No live GUI/editor; captured JSON and rendered MP4s are diagnostic tools.

See [docs/VALIDATION.md](docs/VALIDATION.md) and [docs/CCD_AND_PARALLEL.md](docs/CCD_AND_PARALLEL.md) for test specifics, honest failure modes and benchmark scope. No workflow is installed or required on the owner's personal GitHub repository.

## Credits and license

Educational lineage: Chris Giles and `savant117/avbd-demo2d`; AVBD research by Chris Giles, Elie Diaz and Cem Yuksel. The engine is an independent experimental implementation rather than the official research code. MIT license; see [LICENSE](LICENSE).

### Stage 8 — Rotating-body CCD (reviewed comparison)

`avbd3d_rotational_capture` records **eight separate rod-versus-sphere experiments** twice, with identical initial states and only CCD enabled/disabled. All eight produced an impact impulse and sphere displacement with CCD enabled, while the discrete run missed the first impact. Both positive and negative spins are tested. This verifies these specific circumstances, not arbitrary rotating bodies.

```sh
./build/avbd3d_rotational_capture > rotational_trace.json
python tools/render_rotational.py rotational_trace.json stage8-reviewed.mp4
```

The 12-second video is a **6× slowed diagnostic replay**: every actual 120 Hz physics pose is held for six rendered 24-fps frames. There are no scripted contact impulses or fabricated motion; the initial scene is reset between the eight controlled experiments. The measured sphere speed, impulse count, and TOI events appear in each panel. The final display frame's contact count is **not** used as a proxy for impacts occurring earlier in the tick.

The renderer produces an MP4 that is played separately; there is no hosted interactive graphics application. See `docs/CCD_AND_PARALLEL.md` for assumptions and limitations.
