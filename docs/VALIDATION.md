# Physics validation and limits

## Baseline verification (Linux / GCC / Release, October 9, 2026)

Executed `ctest` and `avbd3d_tests`: **14 pass, 0 fail**.

Covered: free fall, quaternion rotation, resting contact, three- and eight-box stacks, tilted impacts, head-on momentum symmetry, ground friction braking, off-center contact torque, rotated OBB contact, frictionless sliding, deep initial overlap recovery, 240 Hz stepping, and 64-body stress stability. Tests have finite-run, finite-tolerance expectations and do not prove correct behaviour for arbitrary configurations.

The isolated `avbd3d_limitations` program reproduces **high-speed tunnelling**: a small 0.3-unit box moving at 400 units/s passes a thin 0.1-unit wall in one 1/120-second step without generating contacts. This is an explicitly documented non-blocking failure.

Benchmarks depend on hardware and build settings. Previous measurement: 1,000 stacked boxes, 120 steps, 53.4 ms/step on the original test environment. No claim of real-time performance at this scale.

## Required before a game engine

1. Verify complete AVBD 6-DOF equations including rotational geometric stiffness and force/correction signs against the paper and reference.
2. Add CCD and restitution, tests for moving targets, high-speed and extreme aspect ratios, static/dynamic friction convergence, deterministic replay.
3. Expand primitives/constraints and independently verify momentum/energy where applicable.
4. Optimize broadphase, solver, cache, sleeping/islands, parallelization, and eventually GPU after correctness is established.

## Builds

This source-only repository intentionally contains no `.github/workflows` files. Manual build commands are in the root README. Do not represent platform builds as verified until run on those platforms.
