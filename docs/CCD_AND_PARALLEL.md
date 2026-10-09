# Development milestones 6 and 7 — October 9, 2026

## Milestone 6: translation CCD

The optional `Settings::enableCCD` uses event-driven **time-of-impact (TOI)** splitting rather than merely raising a fixed discrete substep count. It considers swept broadphase intervals and advances the AVBD solver to a collision boundary before consuming the remaining timestep.

Three implemented narrowphase sweep families:

- **Sphere–sphere:** quadratic first contact time between linearly moving centers.
- **Sphere–oriented box:** piecewise exact quadratic first intersection of a moving point with the Minkowski-expanded AABB in the box's local frame. This captures corners and edges, not only an expanded slab.
- **Oriented box–oriented box:** swept separating-axis intervals over three axes from each box plus nine cross-product axes. Valid for fixed orientations over the sweep.

These calculations are exact for their **constant-relative-velocity, fixed-orientation geometric model**, up to numeric tolerances. They are **NOT full six-degree-of-freedom rigid-body CCD**: box rotation, angular TOI, continuous gravitational acceleration trajectories, deformables, and dynamic triangle mesh sweeps remain unsolved. `ccdUnsupportedRotation` counts candidate sweeps skipped because a box has nonzero angular velocity. `ccdUnresolved` explicitly reports an exhausted event budget (`maxCCDSteps`, default 48); a budget exhaustion falls back to a discrete remainder and has no guarantee against tunnelling. CCD is opt-in and remains experimental.

The renderer's stage 6 compares 16 sequentially scripted projectile launches in two separately simulated worlds, identical except CCD off/on. The 0.08-unit wall is thinner than a single 1/30 s timestep's projectile movement. At every recorded tick: **0 wrong-side/penetrating samples with CCD**, versus **1,230 samples** with discrete detection. The CCD path recorded eight TOI split events (two symmetric impacts can share one time) and zero exhausted budgets. This is a scene-specific result, not a universal proof.

Regression tests cover 1,800-unit/s sphere-to-thin-box, 1,600-unit/s moving box-to-wall, two 1,500-unit/s opposing spheres, a rotated stationary thin box, grazing misses, a 100,000-unit/s non-rotating sphere, event-budget reporting, and spinning-body limitation reporting.

## Milestone 7: graph-colored CPU parallelism

The AVBD per-body primal step is now optionally executed in **greedy contact-graph color wavefronts**. Connected dynamic bodies (collision contacts or enabled distance joints) receive distinct colors, while simultaneously processed bodies cannot mutate one another. Each color is a synchronization boundary. Contact/joint dual updates, collision search, restitution, and broadphase remain serial. On OpenMP-capable builds, large color waves run over `Settings::parallelThreads` (default 0 = runtime default). Without OpenMP, the same colored ordering runs on one thread.

Performance in this container with Release compilation and no sleeping, **specific resting-pile** benchmark:

| Dynamic boxes | Serial | Colored, 4 threads | Relative speedup |
| --- | ---: | ---: | ---: |
| 256 (8×8×4) | ~9.21 ms/step | ~4.33 ms/step | ~2.1× |
| 500 (10×10×5) | ~14.48 ms/step | ~12.03 ms/step | ~1.2× |

Results depend on CPU load, scheduler and contact graph. These benchmarks run the whole simulation timestep, not just the parallel primal update; coloring overhead is included. The 406-moving-body staged recording reached seven colors and 2,545 contact points. A dense 144-box pile stayed under 0.002 units of peak reported witness penetration in a 240-tick test. One isolated-contact parity test matched the serial path within 1e-6. Dense pile trajectories may differ due to valid solve-order changes.

This is **not a GPU or million-body result**. The scene contact representation, broadphase, maps, and repeated global loops still require redesign for large-scale throughput. The authors' actual GPU research demonstration uses different hardware, implementation, iterations, and scenarios; direct speed comparisons are invalid.

## Build, measure and reproduce

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DAVBD_ENABLE_OPENMP=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/avbd3d_parallel_benchmark 8 4 20
./build/avbd3d_milestones ccd > ccd_stage.json
./build/avbd3d_milestones parallel > parallel_stage.json
python -m pip install Pillow
python tools/render_milestones.py ccd_stage.json ccd.mp4
python tools/render_milestones.py parallel_stage.json parallel.mp4
```

Running the stage capture programs creates *recordings*, not a live GUI. Rendering requires `ffmpeg` in PATH. No GitHub Actions are required, configured or run on the owner's personal repository.

## Verification and debt

- 14 baseline tests, 18 previous advancement assertions, 19 new CCD/parallel assertions: all passed on the Release build (three CTest suites).
- AddressSanitizer/UndefinedBehaviorSanitizer instrumented optimized build: all three suites passed. The initial unoptimized Debug sanitizer run hit a time limit rather than a test failure.
- Event times are finite-resolution; high angular velocities, repeated TOI events, zero-time contacts, extreme mass ratios, joint chains and multiple interacting CCD contacts need much stronger verification.
- Next priorities: rotational screw-motion CCD with conservative advancement, collision islands, task-based parallel broadphase/narrowphase, layout/scratch-memory consolidation and a GPU backend, followed by independent scale/accuracy benchmarking. No milestone should be marked complete based solely on a rendered MP4.
