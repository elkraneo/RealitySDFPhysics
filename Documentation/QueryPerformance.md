# Query wrapper evaluation

Local evaluation on 2026-11-05, native arm64 macOS 27.2 with Xcode 27.2, Swift release build.
This is a controlled library microbenchmark, not a phone frame-rate or game performance result.

## Workload and comparison

`SDFQueryPerformanceTests` builds worlds of one and 57 static analytic boxes. Boxes have alternating
collision groups. The ray crosses every box. The overlap query either intersects one box or misses
all boxes. Empty-mask queries exercise the explicitly empty filter. No survey fixture or game
metadata is needed.

The benchmark includes a frozen copy of the 0.3.1 Swift wrappers. Legacy and modified wrappers run
in the same process against the **same current native core**, so this isolates wrapper changes;
it does not compare different native collision algorithms. Each value below is the median of five
batches after 20 warmup calls. Rays and clear overlaps use 3,000 calls per batch; one-hit overlaps
use 100; empty-mask overlaps use 10,000. A checksum comparison ensures both paths return the same
number of results. Timing is observational, never a pass/fail threshold.

Run:

```sh
SDF_QUERY_BENCHMARK=1 swift test -c release \
  --scratch-path /Volumes/Faulty/Phoebe-scratch/build-sdf-query-perf \
  --filter SDFQueryPerformanceTests
```

The benchmark is disabled by default. Two functional query tests run normally.

## Observed medians, microseconds per query

The table uses the second post-change run (`/private/tmp/sdf-query-after-repeat.log`). A preceding
post-change run showed the same main result (57-body nearest ray 71.37 to 5.70 microseconds).
Concurrent development/build activity and machine state can affect these measurements.

| Bodies | Query | Legacy wrapper | Modified wrapper |
| ---: | --- | ---: | ---: |
| 1 | nearest ray | 0.448 | 0.284 |
| 1 | all-hit ray | 0.425 | 0.288 |
| 1 | clear overlap | 0.296 | 0.228 |
| 1 | one-hit overlap | 74.952 | 74.853 |
| 1 | empty-mask overlap | 0.239 | 0.044 |
| 57 | nearest ray | 72.051 | 5.734 |
| 57 | all-hit ray | 70.762 | 68.259 |
| 57 | clear overlap | 3.289 | 3.204 |
| 57 | one-hit overlap | 76.675 | 76.760 |
| 57 | empty-mask overlap | 0.720 | 0.043 |

The nearest-ray improvement is substantial in this workload. The one-hit overlap cost is
essentially unchanged: the native narrow phase dominates, so these wrapper changes do not solve
expensive sampled casts or establish that an interactive drag is fast.

## Changes and preserved semantics

- Native POD result scratch uses `withUnsafeTemporaryAllocation`, avoiding a zero-initialized
  Swift scratch array. Returned result arrays still allocate as needed; this is not an allocation-
  free API, and no Instruments allocation-count claim is made. Allocation strategy is the Swift
  runtime's choice; large buffers may use heap storage.
- `.nearest` and `.any` choose the nearest mapped hit without constructing and sorting every
  Swift hit. `.any` intentionally retains the existing nearest-hit semantics. Equal-distance hits
  retain the first native result as before. `.all` still returns sorted hits.
- Empty masks and empty worlds return immediately.
- No query-result cache or persistent shared scratch buffer was introduced. Body additions,
  removals, transforms and filters are synchronized by the existing simulation update path;
  queries continue to inspect that current native world.
- Query masks select a body's group; its physical collision mask does not hide it from queries.
  The functional tests cover that distinction, masks selecting either group, unmatched/empty
  masks, hit ordering, positions/normals/separation, and empty/growing/shrinking worlds.

The existing app trace `placement-before-broadphase.trace` did not capture a useful drag stack.
It is not evidence of a before/after drag improvement. No phone profiling or continuous-collision
accuracy result is inferred from this microbenchmark. Additional native overlap/cast work must be
measured separately with its own correctness fixtures.
