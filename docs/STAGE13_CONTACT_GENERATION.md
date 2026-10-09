# Stage 13 — bounded, allocation-free narrowphase scratch

The prior Stage 12 optimization removed repeated **ordered-map node** allocations for persistent contact pairs, but every box–box narrowphase still allocated temporary contact vectors, face clip polygons, and sometimes a temporary witness-point vector. It also allocated a tiny `std::vector<bool>` for contact matching. Stage 13 removes those allocations from the contact-generation and matching hot path **without a runtime feature switch**.

## Implementation

- `ContactBatch`: exactly four inline `Contact` entries plus a count. `collide`, `appendContact`, `faceContacts` and `edgeContacts` use this bounded per-pair result, which can be assigned to a slot of the parallel narrowphase output array without nested contact-vector allocation.
- `ClipPoly`: a fixed-capacity 12-vertex polygon scratch buffer; a quadrilateral clipped against four face planes has at most eight geometric vertices. Face witnesses also use a fixed array and preserve the prior deepest-first sort order.
- Warmstart matches use a four-element stack `std::array<bool,4>`.
- Persistent `Manifold` retains its compact `std::vector<Contact>`: for a continuing pair, `assign()` reuses its existing capacity when sufficient. New contact pairs may allocate a manifold node and contact capacity; a persistent pair whose contact count grows beyond its capacity may allocate too. Other parts of `World::step` still allocate.

We explicitly rejected changing each `Manifold` to an inline four-element `ContactBatch`: it increased node footprint and made the original dense-contact benchmark slower, despite having no per-manifold heap storage. The selected design keeps expensive in-place solver state compact and only puts short-lived narrowphase geometry on the stack.

## Measurements and interpretation

Same C++17 CPU engine, real 243-moving-box pile against static ground; four-thread Release contact benchmark: 972 contacts and 243 manifolds per step, no change in contact counts. To count total heap allocations inside `World::step`, a serial version of that scene ran 30 warm-up and 50 measured steps using global `operator new` instrumentation:

| Version | Allocations per entire `World::step` | 50-step total |
| --- | ---: | ---: |
| Stage 12 (`6e8a301`) | 5,624 | 281,200 |
| Stage 13 | 764 | 38,200 |

**86.4% fewer heap allocation calls** in this exact workload. Counts are for the *whole step*, not solely narrowphase allocations; remaining allocations come from other solver and spatial data structures. Other machines or scenes can have different counts.

Four-round interleaved, complete `World::step` timings (150 timed frames each, OMP_NUM_THREADS=4) were noisy: Stage 12 median **2.454 ms/step**, Stage 13 median **2.425 ms/step**. Do **not** interpret this as an established runtime speedup; medians are effectively comparable within noise. There is no GPU solver or one-million-contact demonstration.

## Correctness and safety

- All **10 Release `ctest` suites** pass, including saved Stage 12 rigid-body contact fixture and the new rotated-box/sphere geometry test.
- A seeded 80-moving-body rotated contact scene executed 120 steps with precisely the **same** aggregate 9,067 contact points, 5,178 manifolds, and numerical trajectory checksums as an independently compiled Stage 12 binary. This exercises fixed polygon clipping beyond the axis-aligned pile.
- `contact_allocations` verifies the real 243-manifold case stays below 1,500 heap allocations per step after warm-up. It is a regression indicator, not proof all allocations are gone.
- Stage 13 `contact_allocations` and `contact_geometry` also passed focused ASan/UBSan executions. The additional longer sanitized persistent-manifold fixture was started but did not complete within the local execution time budget; Release version passed.

Reproduce locally:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DAVBD_ENABLE_OPENMP=ON
cmake --build build --parallel 4
OMP_NUM_THREADS=4 ctest --test-dir build --output-on-failure -j2
OMP_NUM_THREADS=4 ./build/avbd3d_contact_benchmark 5 150
./build/avbd3d_contact_allocations
./build/avbd3d_contact_geometry
```

For a fair before/after total-allocation comparison, build the *same* `tests/contact_allocations.cpp` against the Stage 12 library in addition to the Stage 13 library, because Stage 12 did not contain this instrumentation by default.

## Next bottleneck

`World::step` still constructs per-frame broadphase vectors, solver incidence lists, and constraint-island data. Reduce allocations in those **after** profiling and preserving the legacy contact trajectory fixtures. Neither the 243-contact pile nor the million-body zero-contact case alone establishes million-body dense-contact scalability.
