# Stage 11 — Data-oriented motion integration and flat isolation hash

Stage 11 is an incremental, verifiable CPU-engine milestone, **not** a GPU solver, a 1-million-contact demonstration, or a full structure-of-arrays rewrite of the AVBD solver.

## In-engine implementation

- `Settings::enableDataOrientedPredictor`: reusable per-`World` vectors of `px/py/pz/vx/vy/vz`, plus a moving mask. On **real `World::step()` calls**, gather from authoritative `Body` objects, apply the inertial position predictor across contiguous arrays, then scatter positions back to `Body`. Orientation is integrated by the existing quaternion update. Both contact-rich discrete stepping and certified free flight use the same opt-in predictor. The CCD path also reaches it when entering discrete substeps.
- `Body& World::body(id)` is still authoritative and mutable. The gather happens **every step** to avoid stale data if callers retain body references. The contact solver, friction, joints, restitution, manifold warm-starting, and public API are unchanged. It is a bridge for future data-oriented ownership, **not** a full SoA physics engine. AoS↔SoA packing can dominate small or sparse cases.
- `Settings::enableFlatFreeFlightCertificate`: optional contiguous open-addressed, epoch-tagged cell occupancy array. It preserves the Stage 10 sufficient (not necessary) conservative separation certificate, but reuses slots rather than allocating an unordered-set node for every grid cell on each step. Full 3D cell coordinates are compared on hash collisions. A full table refuses to certify, falling back to the existing collision solver. The legacy `unordered_set` certificate remains selectable for comparison.
- Both switches are **off by default**. They do not silently enable unrelated fallback modes. The flat certificate applies only when `enableCertifiedFreeFlight=true` and Stage 10's normal restrictions (no CCD, joints, sleeping or adaptive stepping; no previous manifolds) are met.

## Tests and semantics

- `ctest`: seven Release suites, including new `data_layout` test for multi-step freeflight, rotations, retained mutable `Body&` state, changed timestep, dynamic body additions, contact-rich stacks and impulses, static/dynamic joint constraints, rotational CCD, overlapping spheres, oversized floor and flat hash comparisons.
- Focused ASan+UBSan checks: `data_layout`, `certified_freeflight`, `rotational_ccd` passed locally with optimization (not a claim that the *entire* sanitizer suite completed).
- The recorded 324-body Stage 11 replay has real rigid-body poses from the engine, not interpolated or hand-animated motion. Max reported contacts: 1,816 during the recorded 5-second playback (238 advanced physics steps), max islands 324, impulses 9,154. The sample MP4 shows 5s at 24fps for 2s of simulation slowed 2.5× (each video frame advances two 1/120s physics steps). The renderer reports the correct display ratio in its overlay if available; check any labels against timing.

## Local performance measurements (not Kaggle hardware)

The sparse case contains *real engine rigid bodies, zero contacts*, and the full engine `World::step` plus certificate. Each row summarizes the average of 3 separate processes, 4 timed steps each; the measurements are wall-clock milliseconds per step and are sensitive to hardware load.

| Bodies | Stage 10 certificate (`certified`) | Flat grid (`flat`) | Flat grid + SoA predictor (`soa_flat`) |
| ---: | ---: | ---: | ---: |
| 10,000 | 1.84 ms | 1.41 ms | 1.60 ms |
| 100,000 | 27.54 ms | 23.30 ms | 26.66 ms |
| 1,000,000 | 464.46 ms | 295.46 ms | 343.10 ms |

**At 1,000,000 isolated bodies, the new flat certificate alone was about 1.57× faster than the baseline**; the combined packed motion path was about 1.35× faster. Importantly, the SoA gather/scatter **reduced** performance compared to flat certificate alone. This is a useful architectural finding, not evidence that a full SoA engine is slower in principle.

A separate 324-body *contact-rich* 120-step scene was measured four times in alternating order. Mean results: AoS 6.69 ms/step; SoA predictor 6.24 ms/step. The final position of body 1 was exactly `0.560631193478` in both, and the maximum number of contact points was 1,984. The small difference could vary on other hardware. This is a full AVBD contact solver benchmark, unlike the standalone translation-only kernel from Stage 10.

**None of these tests demonstrate a million interacting bodies in real time.** The CUDA translation-only experiment from Stage 10 likewise does not provide an accelerated GPU collision solver. GPU work remains future work.

## Local reproduction

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DAVBD_ENABLE_OPENMP=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
for n in 10000 100000 1000000; do
  for mode in certified flat soa_flat; do
    ./build/avbd3d_million_benchmark "$n" 4 "$mode"
  done
done
./build/avbd3d_layout_benchmark aos 120
./build/avbd3d_layout_benchmark soa 120
./build/avbd3d_layout_capture > stage11-trace.json
python tools/render_layout.py stage11-trace.json stage11.mp4
```

The Kaggle notebook `notebooks/avbd_stage11_kaggle.ipynb` clones the latest `main`, verifies the test suites and runs paired CPU comparisons. Optionally, it reruns the **Stage 10** CUDA translation probe on a compatible GPU, but labels that result as an isolated kernel rather than AVBD GPU physics.

## Next improvements

1. Move collision prediction bounds to contiguous arrays so the narrowphase and spatial index read them directly, reducing AoS↔SoA copying.
2. Replace per-body map/manifold and contact-memory allocation patterns with compact persistent contact pools, keeping deterministic ordering.
3. Benchmark contact-rich scenes at 1,000, 5,000, and 10,000 bodies on Kaggle CPU, then design comparable CUDA collision candidate generation and compare exact candidate sets to the CPU reference.

### Million-body video in Kaggle

The Stage 11 Kaggle notebook now produces `stage11-million-bodies.mp4` from
`avbd3d_million_video_capture` and `tools/render_million_video.py`. This is a
**real CPU simulation with one million `World::addBox` bodies**. Each of its
48 recorded frames is computed from `World::body` positions following genuine
`World::step` calls (141 solver steps after the initial frame). All bodies are
included in a three-channel fixed-camera density projection: collocated screen
pixels aggregate their population rather than skipping bodies. A per-frame
uint16 raster checksum must equal the declared body count.

This intentionally records **only gravity/free-flight without any contacts**;
it proves a sparse-million simulation, *not* a real-time million-contact engine.
The recorder refuses to produce a successful trace if the conservative
zero-contact spatial certificate fails, a camera frame crops even one body,
or any projected state is non-finite. The renderer independently checks the
per-frame projected population and certification stats. MP4 playback at 12 FPS
is visual playback and does not claim realtime simulation performance.

Running manually after a Release build:

```bash
./build/avbd3d_million_video_capture 1000000 48 3 capture
python tools/render_million_video.py capture stage11-million-bodies.mp4 --fps 12
```

The raw intermediate `density_####.bin` rasters store uint16 counts in
row-major 1280×720×3 depth-band order; `physics_stats.jsonl` records the
measured simulated time, body count, visible count, contacts and certificate.
The notebook deletes the large intermediate rasters *after* encoding and
preserves the stats, MP4 and preview images.
