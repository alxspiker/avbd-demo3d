# Stage 10 — one-million-body CPU capacity and CUDA SoA feasibility

This Stage 10 experiment builds on the Stage 9 3D BVH and parallel solver. **No full GPU physics backend has been implemented.** The Kaggle GPU experiment is a real CUDA kernel, but it measures only collision-free **translation**. No GPU-AVBD contacts, rotations, constraints, CCD, or dual updates are claimed.

## Why the target should be split

The [original AVBD research project](https://graphics.cs.utah.edu/research/projects/avbd/) describes its large-scale simulations as **parallel GPU** work. Do not describe its million-object example as a measured CPU demonstration. The correct CPU capacity target is separate from interactive performance and from dense connected contacts.

## Certified no-contact CPU frame

New opt-in `Settings::enableCertifiedFreeFlight` (default false) checks the *predicted end-of-step* padded AABB of each box/sphere. It assigns every box to each 3D occupancy cell its bounds touch. A cell occupied twice is a conservative potential-overlap witness, so the normal AVBD collision and constraint pipeline runs instead. A successful certificate proves there are no candidate contacts for the *discrete end-of-step positions*, allowing the engine to skip the unnecessary 6×6 per-body AVBD iterations while still updating all body positions, quaternions and BDF1 velocities. Conditions:

- No contact-cache manifolds and no joints, no CCD, no adaptive substeps, no sleeping; these always use the normal engine pipeline.
- Occupancy size must be small (at most 2 cells per spatial axis per body), else certification rejects; this protects against huge boxes and giant static floors.
- Finite grid coordinates within safe integer limits and finite positive `freeFlightCellSize`.
- The certificate is a **sufficient condition**, not a complete collision detector. Rejection is expected for close but not touching objects. Success does **not** certify absence of intermediate swept collisions; use CCD when tunnelling matters.
- Stat fields `certifiedFreeFlight` and `freeFlightBodies` expose whether the optimization actually ran. A failure is never silently counted as a success. The default engine is unchanged.

The new `tests/freeflight.cpp` tests eight-step legacy equivalence for moving and rotating boxes, contact fallback for overlapping spheres and floors, and guards for CCD, sleeping and joints.

## Local CPU results (October 9, 2026)

Release C++17, container with 5 visible logical CPUs, no GPU; 7 primal + 4 post iterations; 3 full steps per test, sparse 3D grid of actual boxed `Body` instances (zero contacts), no sleeping; timings include the entire per-frame certificate / broadphase and state integration but **exclude initial world construction**.

| Actual bodies | Stage 10 certificate / full body integration | Baseline Stage 9 BVH / full constraint solve | Contacts |
|---:|---:|---:|---:|
| 10,000 | 2.17 ms/step | 52.26 ms/step | 0 |
| 100,000 | 31.13 ms/step | 552.80 ms/step | 0 |
| **1,000,000** | **556.03 ms/step** | Not attempted | **0** |

First unshifted-grid measurement took 5.08 s/step at 1M due to grid-alignment duplication; shifting grid origin by half a cell reduced occupancy count and improved to the 556 ms measurement, with tests re-run. This is a significant illustration of spatial hashing sensitivity. A 1M `Body` benchmark peaked near **292,128 KiB** of resident memory. Timings are short-run measurements, **not 120 Hz or 60 Hz**. The free-flight million-body test achieves roughly 1.8 full physics steps/s here.

The 1M test is a real `World::step` call with one million bodies—not a NumPy or rendering-only fake. It is also much easier than a dense interacting pile. There is **no claim that a million bodies in contact can be solved quickly**.

```
./build/avbd3d_million_benchmark 10000 3 certified
./build/avbd3d_million_benchmark 10000 3 bvh
./build/avbd3d_million_benchmark 100000 3 certified
./build/avbd3d_million_benchmark 100000 3 bvh
./build/avbd3d_million_benchmark 1000000 3 certified
```

## Data-oriented and GPU translation prototypes

`avbd3d_soa_reference` uses six contiguous `std::vector<double>` arrays, not the current `Body` array of structs. On the same CPU, a **1M-body, 100-step benchmark measured ~2.57 ms/step** for *translation-only* state updates, with an independent analytic-displacement check. There is no broadphase, contact detection, rotation, or AVBD matrix solve; this microbenchmark **cannot be substituted for a full-engine timing**.

`tools/gpu_freeflight_probe.py` is a CUDA **float32** SoA kernel built with CuPy `RawKernel` (NVRTC), using the NVIDIA device and CUDA events. It verifies sampled positions against an independent analytic formula and reports the GPU model, kernel milliseconds/step, and measured FP32 error. It excludes PCIe transfers from its timed region. Precision and included work differ from the CPU C++ physics engine, so **do not claim a GPU speedup over AVBD from this number**.

Kaggle: import `notebooks/avbd_stage10_kaggle.ipynb`, enable Internet for cloning the GitHub repo, and select a CUDA-enabled GPU accelerator (e.g. T4/P100). The notebook builds, runs all six suites, then runs 10K/100K/1M CPU samples and the GPU probe if an NVIDIA GPU plus compatible CuPy are present. If unavailable, the GPU section explicitly prints **SKIPPED**. GPU benchmarks have **not been run here** because this execution environment does not expose a GPU.

## Visualization

`avbd3d_freeflight_capture` advances **10,000 actual box bodies for 720 solver steps**, samples 1,250 specific body poses at 120 instants, and asserts the separation certificate at every substep. `tools/render_freeflight.py` creates a six-second 20-fps MP4 from these exact captured positions, with solid box faces and labels that distinguish actual bodies from samples shown. The video illustrates a sparse falling field and **does not claim to show a million bodies**.

```
./build/avbd3d_freeflight_capture > stage10-trace.json
python tools/render_freeflight.py stage10-trace.json stage10-sparse-physics.mp4
```

## Next milestone

Actual million-body *dense* CPU physics needs the expensive per-body allocation and contact caching pipeline redesigned: persistent islands, stable compact contact pools, SIMD work scheduling, incremental broadphase, and careful pressure tests at 10K/50K/100K truly interacting bodies before claiming 1M dense interactions. A full compute backend needs device-resident positions, broadphase, contact generation, constraint manifold state, color dispatch and solver iterations, not just the translation kernel.
