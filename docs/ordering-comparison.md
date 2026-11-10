# Pickup ordering comparison

This is an offline graph-planning comparison. It does not execute a robot,
measure physical travel or prove a globally optimal continuous trajectory.
Each row was produced by the installed CLI with `--suite --exact --repeat 5`.

Source baseline: `0e4d2d68d618486cc28cd4e8ea98afa7178b55b7` plus the
comparison candidate. Comparison module SHA-256:
`2b3cc99bc17f3a0350e41d7a7d3647027d434b3c0aa25146f8f2fc87b64a5bc8`.
Graph SHA-256: `9c3795e81eb91ff9a9ab015ebae41ce886bb96dd1ab07cec295b15131bccf5f3`.

Environment: Linux x86_64, rootless Podman, local ROS Humble image
`c5df849eb3d9`, Python 3.10.12. The installed module was
compared byte-for-byte with the measured candidate. Five timed repeats per
strategy follow one warmup. Times are medians of complete graph-plan
construction in microseconds; graph loading, geometry validation and exact
enumeration are excluded. The same inputs are used for each strategy;
NN+2-opt includes NN initialization. These single-session timings vary with
host load and are illustrative, not real-time guarantees.

| Case | Original requests | Exact graph optimum (m) | FIFO m / μs | NN m / μs | NN+2-opt m / μs |
|---|---|---:|---:|---:|---:|
| empty | (none) | 0.0 | 0 / 5.360 | 0 / 6.873 | 0 / 12.594 |
| single | r1c2 | 12.0 | 12 / 35.346 | 12 / 51.187 | 12 / 94.357 |
| repeated_and_depot | r2c2, r0c0, r2c2 | 16.0 | 32 / 96.151 | 16 / 145.715 | 16 / 386.037 |
| comparison | r1c2, r2c0, r2c4 | 24.0 | 32 / 95.019 | 28 / 195.969 | 24 / 701.100 |
| six_requests | r3c4, r1c1, r2c0, r3c2, r1c4, r0c2 | 32.0 | 52 / 137.188 | 32 / 516.863 | 32 / 2049.971 |

The exact reference enumerates original request-index permutations using
precomputed Dijkstra distances, including depot departure and return. Duplicate
destinations remain distinct requests. Equal optima may have different request
orders. Enumeration is intentionally bounded to eight requests; the graph's
geometry is validated against the installed SDF world before running the CLI.
A zero-length optimum has an undefined percentage gap, encoded as JSON null.

The five chosen fixtures include an empty case, a single request, repeated
locations including a depot request, a three-request comparison and a
six-request case. They are small fixed examples, not a representative workload.
The FIFO/NN/NN+2-opt example distances are 32/28/24 m; the exact reference is
24 m for this graph. No general claim that either heuristic always beats FIFO
follows from this set. Graph lengths differ from actual robot travel distance.

## Reproduction

In a sourced ROS Humble workspace containing the candidate:

```bash
colcon build --packages-up-to warehouse_planning
source install/setup.bash
cd /tmp
ros2 run warehouse_planning compare_ordering --suite --exact --repeat 5 --json
```

`--pickups NODE ...` selects a custom request list, and `--graph` changes
the input graph while retaining validation against the installed world.
Unknown or unreachable requests produce an error; the tool does not send
velocity commands. JSON includes original request IDs, complete route nodes,
per-leg lengths, timing samples and exact-reference gaps.

The final isolated Humble build succeeded for `warehouse_sim` and
`warehouse_planning`; the planning test suite passed 458 tests, zero errors,
failures or skips. The final installed CLI was run from `/tmp` for the fixed
suite and a separate comparison example; invalid node and nine-request exact
reference were rejected. A separate eight-request installed CLI run
enumerated 40,320 permutations and found a 20 m graph optimum. A logically
disconnected graph rejected an unreachable request. No Gazebo motion run was
performed for this planning-only change. Raw benchmark JSON and logs remain local in the
temporary evidence directory; they are not presented as new motion evidence.
