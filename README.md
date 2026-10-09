# AVBD Demo 3D — experimental physics core

## Stage 11 — CPU motion layout and flat free-flight grid (October 2026)

**New on `main`:** `Settings::enableDataOrientedPredictor` integrates a reusable six-channel structure-of-arrays translation predictor into **actual `World::step()` calls**, preserving the external `Body` API, quaternion dynamics, CCD, joints and full AVBD contact solving. Because `Body&` can be externally mutated, it gathers/scatters each step. That memory transfer has a real cost: this bridge is **not** a full SoA constraint solver.

`Settings::enableFlatFreeFlightCertificate` adds a reusable contiguous 3D occupancy table to the Stage 10 **zero-contact** isolation proof. Either optimization is opt-in. In three repeat local million-body sparse runs (4 steps each), the baseline averaged **464 ms/step**, flat-grid **295 ms/step**, and flat-grid plus SoA predictor **343 ms/step**. The packed predictor did not beat the flat-only path. A separate full-solver 324-body contact scene achieved identical terminal states and contact counts with and without the packed predictor; timings vary by hardware.

- [Stage 11 architecture, validation, benchmark limitations](docs/STAGE11_DATA_LAYOUT.md)
- [Stage 11 Kaggle CPU comparison + optional CUDA notebook](notebooks/avbd_stage11_kaggle.ipynb)
- `avbd3d_layout_benchmark aos|soa 120` measures the real contact-rich solver.
- `avbd3d_million_benchmark 1000000 4 certified|flat|soa_flat` measures real sparse World steps.
- `avbd3d_layout_capture` and `tools/render_layout.py` generate a reviewed replay of 324 bodies with real contacts.

**Not achieved:** 1 million densely interacting objects at real-time speed, a full SoA AVBD constraint/contact solver, and GPU-based collision solving. Stage 10's T4 CUDA kernel was **translation only**.


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

| `avbd3d_freeflight_tests` | Stage 10 certified no-contact parity, collision fallback and configuration tests |
| `avbd3d_million_benchmark [n] [steps] [certified|bvh]` | Actual engine 10K/100K/1M sparse CPU workload |
| `avbd3d_soa_reference [n] [steps]` | Isolated contiguous-array translation microbenchmark, not full physics |
| `avbd3d_freeflight_capture` | Capture a real 10K-body / 1,250-visible-sample falling-field video |

### Development stages

1. **Elastic contacts:** 84 boxes, restitution, impact-induced rotation, persistent frictional contact.
2. **Fast projectiles:** 22 spheres strike a 0.09-unit-thick wall using optional adaptive discrete substeps. Regressions check every simulation tick that no projectile crosses the wall in this scenario. **This is not swept/continuous CCD.**
3. **Sphere demolition:** A 132-brick wall struck by two moving spheres, testing mixed sphere–box and box–box contacts.
4. **Attached wrecking ball:** 13 distance links hold a ball while it strikes a stack. All links are unbreakable **in this demonstration**. Fracture is still supported and separately unit-tested.
5. **Sleeping and waking:** An initially resting body group sleeps and wakes when struck by a moving sphere.
6. **Linear-translation CCD:** Event-driven swept sphere–sphere, sphere–OBB and OBB–OBB TOI, with a side-by-side discrete comparison. Not rotational CCD.
7. **Parallel CPU solver:** Graph-colored independent rigid-body updates using OpenMP when available; 405 blocks and one sphere in the stage video.
8. **Rotational CCD:** Eight controlled fast-spinning-bar contacts, side by side with CCD disabled. CCD produces a measured sphere impulse where the discrete path misses.
9. **CPU scaling:** 3D BVH broadphase, parallel contact detection, independent collision-island scheduling.
10. **Million-body CPU capacity / GPU probe:** Optional certified zero-contact optimization can integrate one million isolated bodies at ~0.56 seconds/step in this container. CPU/GPU SoA translation kernels and a Kaggle notebook are provided; these are **not full AVBD physics GPU implementations**.

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

### Stage 9 — 3D BVH broadphase, parallel narrowphase and collision-island scheduling

Stage 9 adds three independent **opt-in** switches so old and new paths can be compared directly:

- `enableSpatialBroadphase`: deterministic, median-split 3D AABB tree for both discrete and swept CCD candidate gathering. This improves sparse layouts where a one-axis sweep has many false overlaps. A very large static ground plane is stored in one leaf, not duplicated across spatial cells.
- `enableParallelNarrowphase`: compute pair contacts in independent slots with OpenMP, then merge/manifold-match in stable sorted pair order on one thread. The path remains usable without OpenMP (serial fallback).
- `enableIslandSolver`: when `enableParallelSolver` is also on, schedule disconnected groups of active dynamic bodies independently. Enabled distance joints connect islands; static floors do **not** join otherwise unrelated groups. If only one island exists, the Stage 7 graph-colour solver is retained.

All switches default to **false** for backwards compatibility. Example:

```cpp
world.settings.enableSpatialBroadphase = true;
world.settings.enableParallelNarrowphase = true;
world.settings.enableParallelSolver = true;
world.settings.enableIslandSolver = true;
world.settings.parallelThreads = 4;
```

Rebuild and reproduce the Stage 9 regression, 1k/5k/10k benchmarks and **actual recorded** four-tower simulation:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DAVBD_ENABLE_OPENMP=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/avbd3d_scaling_benchmark 1000 sparse 5
./build/avbd3d_scaling_benchmark 5000 islands 5
./build/avbd3d_scaling_benchmark 10000 islands 5
./build/avbd3d_scaling_capture > stage9.json
python tools/render_scaling.py stage9.json stage9.mp4
```

`stage9.mp4` uses 240 recorded frames at 24 fps (10 seconds), with two actual 120 Hz physics steps per display frame (5× slow playback). It has 324 moving bodies, **not** 10,000. The visible shapes are actual C++ rigid-body poses; contact counts and island counts come from the simulation.

See [docs/STAGE9_SCALING.md](docs/STAGE9_SCALING.md) for benchmark details, limitations and reproducibility. The MP4 and JSON output are not stored in GitHub; run the capture locally to recreate them. No GitHub Actions are required.

### Stage 10 — million-body CPU capacity and reproducible GPU experiment

**See [Stage 10 technical report](docs/STAGE10_CPU_GPU.md)** and the runnable [Kaggle notebook](notebooks/avbd_stage10_kaggle.ipynb). On the test machine the C++ engine advanced **one million isolated bodies** in about 556 ms/step using the new optional certified free-flight path; **zero contacts** were present. The separate one-million-body SoA translation kernel measured ~2.57 ms/step, but omits all collision and angular solve work. No real GPU run has been completed here.

```
./build/avbd3d_million_benchmark 1000000 3 certified
./build/avbd3d_soa_reference 1000000 100
./build/avbd3d_freeflight_capture > stage10-trace.json
python tools/render_freeflight.py stage10-trace.json stage10-sparse-physics.mp4
```

No GitHub Actions, paid APIs, or hidden GPU fallbacks. The original solver is still the default; all experimental optimizations are opt-in.
