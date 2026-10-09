# Stage 9 — measured CPU scaling (October 9, 2026)

Stage 9 continues from the reviewed Stage 8 commit `917027349ce7a863c72f89aca9024bb1426804ea`. The milestone targets spatial candidate search, parallel contact generation, and connected-component/island scheduling. It is **not** a GPU engine or a million-body simulation.

## Architecture

1. **Deterministic 3D BVH**: per-step rebuild of a median-split tree over padded rigid-body AABBs. Traverse the two sides recursively; emit each overlapping leaf pair once, then sort lexicographically. It is an alternative to the existing one-axis sweep-and-prune, not a replacement for the SAT/sphere narrowphase. Swept rotating-box bounds encompass full rotation-radius spheres as before. Huge static floors occupy one leaf and do not cause unbounded uniform-grid expansion.
2. **Parallel narrowphase**: sort pairs, skip fixed/fixed and directly joint-linked bodies, compute contacts into per-pair slots using static OpenMP scheduling, and deterministically merge warmstarting, the contact cache, and contact impact data serially. Contact generation is embarrassingly parallel; contact **solution** is not.
3. **Dynamic-body collision islands**: union connected dynamic awake bodies on contact manifolds and enabled distance joints. Static floors do not create connectivity. For multiple islands, solve bodies in deterministic per-island order with independent threads; for one large island, use Stage 7 graph coloring. Each iteration still synchronizes before updating contact and joint multipliers. The island graph rebuild is currently per step.
4. **Preserved legacy modes**: all new flags are disabled by default; falling bodies, CCD and contact solver behavior are unchanged without opting in. Without OpenMP, parallel paths execute sequentially.

## Evidence and validation

The five CTest suites pass in Release, including the new `scaling` suite (29,537 individual checks in its detailed run). The new suite compares the legacy and BVH world positions, velocities, quaternions, narrowphase calls and contact counts across connected 3D stacks, sparse 2,000-body fields, and a fast CCD sphere hitting a thin wall. It also compares serial and parallel narrowphase across the same deterministic scene, compares detached island solving to the serial reference, and verifies that enabled distance joints connect bodies while a common static floor does not.

The optimized/partially optimized AddressSanitizer+UndefinedBehaviorSanitizer build **passed the base physics, advancement, scaling, and rotational CCD test executables individually**. The full sanitizer CTest run timed out inside the slower `ccd_parallel` stress suite; that suite remains Release-tested, not sanitizer-verified for this milestone. Do not represent the full sanitizer test run as passed.

## Benchmarks

Commands:

```sh
./build/avbd3d_scaling_benchmark 1000 sparse 5
./build/avbd3d_scaling_benchmark 5000 sparse 5
./build/avbd3d_scaling_benchmark 10000 sparse 5
./build/avbd3d_scaling_benchmark 1000 islands 5
./build/avbd3d_scaling_benchmark 5000 islands 5
./build/avbd3d_scaling_benchmark 10000 islands 5
```

Wall-clock milliseconds per **full simulation step** for this container, Release, five steps each, 7 primal + 4 post iterations, OpenMP with four threads for the last column. No sleeping. These are small, short-run measurements, not stable machine-independent frame rates. CPU scheduling and background load can shift them. The serial baseline uses the Stage 8 one-axis sweep and serial solver; Stage 9 uses the 3D BVH plus parallel narrowphase and island solving.

| Layout | Bodies | Stage 8 SAP/serial (ms) | BVH/serial (ms) | Stage 9 BVH/4 threads (ms) | Contact points per step |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sparse, coincident X-projections | 1,000 | 5.77 | 3.95 | 1.77 | 0 |
| Sparse, coincident X-projections | 5,000 | 51.71 | 22.53 | 12.46 | 0 |
| Sparse, coincident X-projections | 10,000 | 174.85 | 49.90 | 22.22 | 0 |
| 4-body separated piles | 1,000 | 14.18 | 14.48 | 7.20 | 3,000 |
| 4-body separated piles | 5,000 | 61.64 | 68.09 | 35.15 | 15,000 |
| 4-body separated piles | 10,000 | 140.09 | 141.81 | 82.31 | 30,000 |

**Critical distinction:** the strongest speedup (~7.9× on the 10,000-body sparse layout) occurs in a layout particularly unfavorable to one-axis sweep-and-prune: almost every X projection overlaps, but bodies are far apart in 3D. It has **zero physical contacts**. It cannot demonstrate dense solver scaling. The 10,000-body pile layout is **2,500 separate four-body islands**, not one interconnected mountain of 10,000 boxes. A single densely connected 10,000-body pile is neither benchmarked nor demonstrated here. The parallel dense-island result is about 1.7× faster than serial, not real-time at 120 Hz.

The BVH adds overhead on some dense layouts (e.g., 5,000 island bodies: 68.09 ms versus 61.64 ms serial) because partitioning doesn't reduce the real contact workload. This is precisely why the legacy option remains available.

## Recorded visual demonstration

`avbd3d_scaling_capture` records four disconnected 80-box towers and four incoming heavy spheres (324 moving bodies) under gravity, with a common static floor. Every displayed pose comes from the C++ physics engine; no scripted trajectories or fabricated impulses. All collision-island and narrowphase data are read from the simulation. It reports a maximum of 1,816 simultaneous contact points across the sampled run, and 240 frames show 3.98 seconds of simulated time at 5× slow playback.

```sh
./build/avbd3d_scaling_capture > stage9-trace.json
python tools/render_scaling.py stage9-trace.json stage9.mp4
```

Only source and rendering scripts are committed. The raw trace and generated MP4 are local artifacts; they can be regenerated without network access using the above commands and Pillow/ffmpeg. The video **does not** claim to show 10,000 moving objects.

## Remaining limitations

- Rebuilding a BVH every timestep costs allocations and sorting; no incremental refit/dynamic tree, caching, or job-system implementation yet.
- Collision-island scheduling is per-iteration with global dual-update synchronization, not a full independent-island solve over entire steps.
- Contacts live in `std::map` and per-contact vectors; allocator and cache throughput remain a scaling bottleneck.
- CCD is conservative/finite-iteration, not a universal no-tunneling proof; extreme angular motion and manifold approximations remain debt.
- Overlapping dense body clouds can still create quadratic genuine candidates; no broadphase cures that geometry.
- GPU compute pipeline, memory layout, work queues and massive dense-scene scaling are future engineering work.
- This 5-step benchmark is not a statistically rigorous repeatable benchmark suite and should be independently reproduced on target hardware.

**Next:** a data-oriented contact pipeline and a CPU island scheduler with persistent work queues, followed by GPU-compute feasibility experiments and larger *connected* pile benchmarks.
