# AVBD Demo 3D

An experimental, headless C++17 rigid-body physics engine inspired by [Augmented Vertex Block Descent (AVBD)](https://graphics.cs.utah.edu/research/projects/avbd/). It includes a physics library, regression tests, benchmark scenes, and tools for rendering recorded simulations.

## Build and run

Requires CMake 3.16+ and a C++17 compiler. OpenMP is optional for CPU parallelism.

```sh
git clone https://github.com/alxspiker/avbd-demo3d.git
cd avbd-demo3d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
./build/avbd3d_demo
```

The demo prints statistics for a small box stack. On Windows with a multi-configuration generator, run `build/Release/avbd3d_demo.exe`.

## Features

- Boxes and spheres with 3D translation, quaternion rotation, and angular response.
- Persistent contacts, friction, restitution, and an AVBD-inspired six-degree-of-freedom solver.
- Distance joints with fracture thresholds, sleeping, and impact wakeup.
- Adaptive substeps, swept translation CCD, and experimental rotational CCD.
- BVH broadphase, parallel contact detection, and graph-colored or collision-island CPU solving.
- Sparse free-flight experiments and dense-contact allocation benchmarks.

The public library API is in [src/avbd3d.h](src/avbd3d.h).

## Render a simulation

Capture programs record poses from the C++ solver. Python tools turn those traces into MP4s; there is no interactive viewer.

For example, capture the wrecking-ball scene:

```sh
./build/avbd3d_stages 4 > stage4.json
python -m pip install Pillow
python tools/render_stages.py stage4.json stage4.mp4
```

Rendering also requires `ffmpeg` on your PATH. This renderer currently loads DejaVu Sans from `/usr/share/fonts/truetype/dejavu/`.

## Status and limits

This is an independent experimental implementation, not the official AVBD research code or a complete implementation of the paper. The rotational solve uses an approximate Hessian, and rotational CCD is approximate. Supported shapes are boxes and spheres; there are no mesh collisions, articulation systems, or full GPU physics solver.

Million-body results cover isolated bodies with zero contacts. They do not establish real-time simulation of a million interacting objects. Benchmark workloads, measurements, and remaining failure cases are documented below.

The latest contact-generation work removes temporary narrowphase allocations. The measured reduction in heap allocation calls did not establish a runtime speedup.

## Documentation

| Topic | Guide |
| --- | --- |
| Physics tests and known limitations | [Validation](docs/VALIDATION.md) |
| Translation/rotational CCD and CPU parallel solving | [CCD and parallelism](docs/CCD_AND_PARALLEL.md) |
| BVH broadphase and collision islands | [CPU scaling](docs/STAGE9_SCALING.md) |
| Million-body sparse CPU runs and GPU translation probe | [CPU/GPU experiments](docs/STAGE10_CPU_GPU.md) |
| Packed translation predictor and flat occupancy grid | [Data layout](docs/STAGE11_DATA_LAYOUT.md) |
| Persistent contact storage | [Contact storage](docs/STAGE12_PERSISTENT_CONTACTS.md) |
| Bounded narrowphase scratch and allocation measurements | [Contact generation](docs/STAGE13_CONTACT_GENERATION.md) |

Kaggle notebooks: [sparse CPU/GPU experiment](notebooks/avbd_stage10_kaggle.ipynb), [data-layout comparison](notebooks/avbd_stage11_kaggle.ipynb), and [contact-generation benchmark](notebooks/avbd_stage13_kaggle.ipynb).

## Credits and license

Based on ideas from AVBD research by Chris Giles, Elie Diaz, and Cem Yuksel, and Chris Giles' educational [2D demo](https://github.com/savant117/avbd-demo2d).

[MIT license](LICENSE).
