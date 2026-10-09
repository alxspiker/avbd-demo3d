# Stage 12: single persistent manifold-node implementation

Stage 12's experimental `enablePersistentManifoldStorage` switch has been
removed. The engine **always** reuses the ordered-map node for a contact pair
that persists between physics steps, using C++17 `std::map::extract` and
`insert`. Pairs that disappear release their map nodes; newly contacting pairs
allocate one. Deterministic pair ordering and the existing solver are unchanged.

We deliberately **do not claim** that contact-vector allocations are now reused.
The old vector is cleared and swapped with freshly generated contacts to avoid
copying potentially four large contact structs. This avoids reallocating the
ordered-map nodes for persistent pairs, but new contact generation still uses
its own vectors.

## Why this implementation, rather than an optional path?

We measured three concrete implementations on a local Release build with
OpenMP and a sustained pile (243 moving boxes plus static floor; ~972 contacts
per simulation step, 30-step warm-up, 100 timed steps, 4 solver threads):

1. Rebuild all map nodes every step (legacy baseline).
2. Reuse map nodes, move newly generated contacts (selected, now unconditional).
3. Reuse map nodes and copy new contacts into the previous vector capacity.
4. Reuse map nodes and maintain persistent per-pair narrowphase scratch vectors.

A matched, five-round interleaved comparison measured median **2.311 ms**
per step for legacy rebuild-all, **2.551 ms** for the previous optional
node-reuse branch, and **2.225 ms** for the single-path implementation.
All measured 972 contacts and 243 manifolds per step. The apparent
**3.7% median improvement** over rebuild-all is modest relative to the
run-to-run noise, so it is not a generally established speedup.

The two attempts to reuse contact-vector capacity were consistently **slower**
in the preliminary local runs; the scratch-vector version was also more complex.
We selected node reuse rather than adding memory overhead with an uncertain
benefit. This is a decision based on these specific workloads, **not** a
universal speedup claim. Noise in the measured single-digit-percent differences
between node reuse and the baseline means that comparisons should use repeated
runs on the same machine.

Run the benchmark after configuring/building the repository:

```sh
./build/avbd3d_contact_benchmark 5 100
```

For an apples-to-apples comparison, build and run the same benchmark from the
parent commit with `enablePersistentManifoldStorage` disabled and then from the
new commit. Do not infer million-body contact performance from these tests.

## Regression protection

`ctest` includes `persistent_contacts`, which checks a 196-moving-box pile
against **saved output from the pre-Stage-12 solver** at 30, 90 and 179 steps.
It also compares serial versus four-thread narrowphase, and tests expiration
and recreation of sphere-sphere contacts. The full suite includes separate
CCD, contact, sleeping, free-flight, and spatial-indexing tests.

The Stage 10/11 million-body zero-contact certification remains a separate
capability, not a million-body dense-contact demonstration.
