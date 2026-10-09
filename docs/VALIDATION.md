# Physics validation and limitations — October 9, 2026

Tests were run with local CMake Release and a separate sanitizer build. `ctest --output-on-failure` reported both test suites passed, in both configurations. The core test executable has **14 checks**; the advancement suite currently has **18 checks**.

## Specific user-reported concerns

### Blocks apparently balanced on corners

An isolated 1 × 1 × 1 box was dropped onto a plane from six distinct initial rotations (0.15, 0.35, 0.55, 0.75, 0.95 and 1.15 radians) with nonzero friction. In each 1,000-step run, the final height was within 0.02 of the 0.5-unit expected face-down resting height, and the most-upward-facing body axis was aligned to within 0.01 of vertical. All six passed.

In the 84-body crowded scene, some cubes remain tilted because their neighbours form mutual supports. That may be physically legitimate; this test does **not** prove all pile contacts are correct. No artificial face snapping is applied. The revised Stage 1 video covers the complete 12 simulated seconds, rather than a short early interval.

### Spheres apparently passing through the wall

The wall is 0.09 units thick and 22 fast spheres (radius 0.28 units, 30 units/s) approach from both sides. Every sphere's signed distance from the wall was checked at **every physics step** for 1,436 steps, not just the video sample frames. The closest recorded center was approximately **0.3284 units** from the center plane. The expected nonpenetrating center distance is 0.045 + 0.28 = **0.325** units. No sphere crossed in the tested scenario. The video is a software depth-sorted 3D projection, not a pixel-accurate renderer, so misleading overlaps are possible.

**Do not extrapolate this to general continuous collision detection.** The optional adaptive substep budget can be exceeded in other high-speed cases. The `avbd3d_limitations` executable intentionally reproduces a tunnel case with adaptive substepping disabled.

### Pendulum losing its wrecking ball

The original demonstration had a deliberate break threshold of 200 force units at the final joint. This generated a fracture event prematurely for the desired wrecking-ball demonstration. The revised Stage 4 makes all 13 links unbreakable, records all links active through the 360-frame trace, and the moving ball displaces at least 17 blocks in the wall by the end. Peak link-length deviation in the recorded trajectory is under 0.005 units (relative to initial link lengths). Breakable-joint behaviour is still separately verified by a regression test.

## Regression coverage

- **14 baseline checks:** gravitational freefall, free quaternion rotation, single resting box, three/eight-box stacks, tilted box, head-on momentum, ground friction, off-centre torque, rotated box contact, frictionless sliding, overlap recovery, high-frequency fixed timestep, 64-body stability.
- **18 advancement checks:** bounce detection and rebound, opposing impact velocities, adaptive substep engagement, fast projectile block, sphere-floor contact, sphere-sphere blocking/bounce, sphere-box block, joint stretch bound and finite state, intentional fracture, resting sleep and impact wake, 22 fast spheres remaining outside the thin wall at *every tick*, valid wall clearance, and six isolated tilted-box resting cases.

These deterministic finite-duration scenarios cannot certify arbitrary contact, material, constraint or performance behaviour. Visual demonstration is additional evidence, not a replacement for objective physics tests.

## Benchmarks

- Earlier 1,000-box stacked baseline: approximately **53 ms/step** (specific previous CPU/workload, not engine performance guarantee).
- Local resting-pile benchmark for **144 dynamic boxes**: approximately **6.43 ms/step without sleeping** vs **1.21 ms/step with sleeping**, after warm-up, on this testing runtime. Differences in body count, hardware, settings, and contact state prohibit direct comparisons with AVBD research's GPU results.
- No million-body real-time claim. The solver is primarily CPU, single-threaded and sequential per-body; substantial structural redesign is required to reach GPU-scale simulation.

## Remaining engineering priorities

1. Validate full AVBD 3D rotational objective, manifold geometry, and geometric stiffness against the paper's equations and numerical derivatives.
2. Add genuinely swept CCD with robust contact time-of-impact solving, not just fixed substeps.
3. Test friction cones, restitution energy, near-singular inertia, extreme mass ratios and dynamic joints via quantitative invariants.
4. Introduce collision island grouping and graph colouring, sleeping islands, broadphase profiling, batched memory layout, SIMD and parallel/GPU execution after correctness.
5. Build consistent cross-platform integration interfaces; expand shape and joint APIs and add independent visual validation.

## Build policy

The personal GitHub repository intentionally contains no `.github/workflows` or compiled binaries. Builds/tests are local or can be run manually; organization-owned CI can be added later when requested.

## Milestones 6/7 update

New optional translation CCD and colored multithreaded solver tested in 19 assertions; see [CCD_AND_PARALLEL.md](CCD_AND_PARALLEL.md) for boundaries, measurements and reproduction. This supersedes the earlier statements on this page that no swept CCD or CPU parallelism existed. Full rotating rigid CCD and GPU scaling are still missing.
