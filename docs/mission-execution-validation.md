# FIFO mission execution validation — 2026-10-13

This historical raw-wheel FIFO execution slice passed its prescribed gates on
2026-10-13. It is not a fresh measurement of the current source. No physical gate
failed, and no model calibration, controller gains or target offsets changed.
The physical limit remains **0.15 m**. These are simulation measurements, not
hardware or mechanical-loading results.

## Configuration and contract

Pre-implementation baseline: `4df71f2f74d9a69c807f71bbabb9f64b88cdc686`.
The measured implementation was subsequently committed unchanged as
`726e62051383da691468b80f4ae3d0c8d8b0052d`; the two validation scripts were
kept separate. The JSON `source_sha256` records the historical implementation,
model, tests and final harnesses, not current-HEAD hashes. Host: Fedora
Linux/amd64, rootless Podman; owned container
`var-e9t-mission-execution-20261013`, image
`localhost/var-e9t-warehouse:humble` (`c5df849eb3d9`). Ubuntu 22.04, ROS 2 Humble,
Python 3.10.12, Gazebo Fortress 6.18.0 and the existing DART physics model.
Other project containers were not operated.

In that configuration, the node loads the installed graph and world through
`ament_index_python`, checks all graph geometry with the existing 0.45 m inflation, verifies the
SDF spawn agrees with the depot, and calls `plan_fifo_mission` before creating
its one command publisher. `RouteProgress` follows each leg; `GoalController`
retains odometry validation, bounded commands and the existing startup guard.
The `run_controller` shutdown path is reused. Missing or invalid requests,
geometry errors and unreachable legs have no partial-motion fallback.

`pickup_nodes` is required and ultimately read-only `STRING_ARRAY`. A measured
Humble parser behavior represents CLI `[]` as a present `NOT_SET` override;
Python list inference instead produces `BYTE_ARRAY`. The node inspects presence
in Humble's startup override mapping solely to distinguish empty from missing,
then uses public parameter APIs to normalize/type/lock it before any executor
or velocity publisher exists. This small compatibility dependency is Humble
specific. Installed empty and missing-parameter behavior was tested.

Each pickup has one continuous simulated-service second with zero commands,
within 0.05 m odometry tolerance and advancing clock/feedback timestamps.
Every observed tolerance departure invalidates the wait, even between ticks.
Repeated timestamps or frozen simulation time cannot complete service. Each
tick completes at most one request; zero-length/repeated legs remain separate.
Intermediate arrivals never latch global `REACHED`. Empty execution checks the
fresh depot start without moving. Final `REACHED` and feedback `FAULT` latch.

The physical wheel centers remain 0.38 m apart; the existing offline-calibrated
DiffDrive effective rolling track remains 0.363 m. Its role and rate-dependent
limits are documented in [odometry diagnostics](odometry-diagnostics.md).
The installed model matched source SHA-256
`c837988dae1755abfe96012df9774bedc0d264fc1c1157ea85d8d0cbaa7edc75`.
Navigation receives raw wheel odometry on `/warehouse/odom` and the static
SDF spawn transform only. No heading estimator or IMU feedback remap is used.
Gazebo world poses remain a separate transport observation stream.

## Tests and measurement design

The full three-package Humble build succeeded. The final colcon run passed
**342 tests: 183 control and 159 planning; 0 errors, failures or skips**.
There are 49 new mission tests. Existing goal/route callback and ideal-drive
checks, the old `route_example` CLI and the FIFO planner/CLI suite also ran.
Coverage includes leg transitions, final/FAULT latches, empty/repeated requests,
passing later pickups, input snapshots, continuous service timing, tolerance
reset between ticks, repeated/frozen timestamps, missing/invalid parameters,
unreachable legs, invalid starts and service-time distance overflow.

The first test run exposed a ROS float-field test fixture error and the actual
CLI-empty-array `NOT_SET` behavior. Both were fixed before the repeated mission
batch. Review additionally hardened service-time overflow to latch `FAULT`;
the full 342-test run verified that guard. Valid-input navigation was unchanged.
The validation container disables Python byte compilation; setuptools reported
only the corresponding warnings. No dependency was added.

Every measurement case starts a fresh installed world and directional bridges.
Concurrent batches use separate ROS domains 84–87 and Ignition partitions;
the final unit/subprocess run uses a separate domain 90. The observer monitors
at most one velocity publisher throughout mission runs. The harness reuses the route
harness's process ownership, pose reader and physical-rest/clearance checks.

Gates were fixed before running: exact per-leg waypoints and numbered request
order; separate service stops; at most 360 simulation seconds per mission;
600 s wall timeout including startup and final-rest observation; at most
0.05 m odometry and 0.15 m physical error at service stops and depot; zero
sampled obstacle intersections; zero final command and at least five seconds
of physical rest after one braking second. Physical rest bounds are 0.001 m,
0.001 m/s and 0.001 rad. Static clearance uses the existing 0.3202 m circumscribed
robot radius and consecutive physical-pose segments against inflated obstacles.

Source-stamped world poses had positive gaps below 0.1 s (observed maximum
0.019 s). Service error is the maximum over the entire service interval,
including time-aligned arrival/completion samples (nearest within 0.06 s).
Both feedback and ROS-clock service intervals must be at least 1 s. Command
observations have an asynchronous observer clock; alignment is bounded by one
50 ms timer period. Offline checks of raw commands additionally prove a
continuous zero-command streak of at least 1 s covering each service, using the
same `zero_streak` function as the final harness. Repeated colocated requests
share a longer zero-command streak but have separate logged waits/completions.

## Three fresh runs per mission

A: `r2c2 -> r1c4 -> depot`, planned legs **8 + 6 + 10 = 24 m**.
B: `r2c2 -> r1c0 -> r1c0 -> depot`, planned legs **8 + 6 + 0 + 2 = 16 m**.
The initial traversal of `r1c0` in B does not serve requests 2 or 3 early.
Errors below are metres; service columns are maxima over all requests.

| Mission / run | Execution (sim s) | Max service odom error | Max service physical error | Final depot odom error | Final depot physical error | Final rest (s) |
|---|---:|---:|---:|---:|---:|---:|
| A / 1 | 194.879 | 0.049982 | 0.065917 | 0.048595 | 0.105024 | 5.014 |
| A / 2 | 195.378 | 0.049440 | 0.054446 | 0.047646 | 0.100768 | 5.005 |
| A / 3 | 194.784 | 0.049528 | 0.054861 | 0.048351 | 0.098950 | 5.016 |
| B / 1 | 139.865 | 0.049009 | 0.079852 | 0.049034 | 0.084581 | 5.006 |
| B / 2 | 139.859 | 0.049708 | 0.088262 | 0.048066 | 0.093695 | 5.014 |
| B / 3 | 139.812 | 0.049050 | 0.088149 | 0.047564 | 0.097900 | 5.008 |

All six runs passed every gate. Service feedback intervals were 1.00–1.05 s,
ROS-clock intervals 1.000–1.045 s. The shortest measured zero-command streak
was 1.041 s. Mission wall execution/rest observation was 149.605–230.663 s,
below the 600 s limit. All sampled collision counts and measured final-rest
translation, speed and yaw changes were zero. Final commands were zero.
An earlier exploratory A run recorded 0.099603 m depot physical error;
it is preserved separately and excluded from these three-run statistics.

## Fault, shutdown and controller regressions

Five mission cases passed: explicit empty, odometry loss in travel, odometry
loss during service, terminal Ctrl+C via PTY, and SIGTERM directly to the node
PID. The command bridge stayed alive during both feedback-loss tests. Fresh
feedback was restored; `FAULT` persisted, no request service completed and no
motion resumed throughout the final-rest window. Empty execution travelled
0 m. Every case ended with zero command and at least five seconds of physical
rest. SIGINT/SIGTERM cases exited normally.

Ten existing controller cases passed: `single_goal`, `r3c2`, `r2c2`, `odom_loss`,
`ctrl_c`, `sigterm`, `invalid_goal`, `unreachable`, `invalid_start`, `moved_start`.
Their physical final errors were respectively 0.048450, 0.025233 and 0.033745 m
for the three motion-success cases. Invalid plans created no command publisher;
invalid starts produced no controller movement. The shared measurement
extraction therefore retains the old harness's checked behavior.

## Evidence and reproduction

[Compact measured results](mission-execution-results.json) preserve all six
runs, per-request timestamps/errors/zero-command proofs and fault/regression
summaries. Raw physical/odometry/command JSONL, controller/bridge/Gazebo logs,
original result JSON and supplementary command proofs remain in ignored
`log/mission-execution-validation-20261013/` (`final-A`, `final-B`,
`mission-faults`, `controller-regression`, and separate `baseline-A`).
Original build/install caches were created outside public source under
`/tmp/warehouse-mission-execution-validation-20261013/`. That temporary cache
is no longer present at split review. Build/test logs and the final pytest XML
reports remain in the ignored evidence directory alongside the raw measurements.
The measurement container was removed after the original validation.

To reproduce the historical configuration, first export its implementation into
an independent directory. Do not reset the current checkout. From this candidate's
repository root (the two scripts must match the historical JSON hashes):

```bash
repo="$PWD"
repro=$(mktemp -d /tmp/warehouse-fifo-repro.XXXXXX)
git archive 726e62051383da691468b80f4ae3d0c8d8b0052d | tar -x -C "$repro"
cp "$repo/scripts/validate_route_following.py" \
   "$repo/scripts/validate_mission_execution.py" "$repro/scripts/"
# Check all eight historical hashes before building.
python3 - "$repo/docs/mission-execution-results.json" "$repro" <<'PYHASH'
import hashlib, json, pathlib, sys
manifest = json.loads(pathlib.Path(sys.argv[1]).read_text())
root = pathlib.Path(sys.argv[2])
for name, expected in manifest['source_sha256'].items():
    if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
        raise SystemExit('historical source mismatch: ' + name)
PYHASH
```

Start a new owned rootless container with the local image (historical ID
`c5df849eb3d9`). Use a new name, ROS domain and Ignition partition if these
example values are already in use. Keep its evidence on the host:

```bash
evidence=$(mktemp -d /tmp/warehouse-fifo-evidence.XXXXXX)
podman run --rm -it --name warehouse-fifo-reproduction \
  --network none --userns keep-id --security-opt label=disable \
  -e ROS_DOMAIN_ID=91 -e IGN_PARTITION=fifo-reproduction-unique \
  -e ROS_LOG_DIR=/evidence/ros-logs -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$repro:/source:ro" -v "$evidence:/evidence:rw" \
  localhost/var-e9t-warehouse:humble bash
```

Inside that container, build the exported historical source. Do not launch the
IMU estimator or remap controller feedback:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /evidence/fifo-build
cd /evidence/fifo-build
colcon build --base-paths /source/src --packages-up-to warehouse_control
source install/setup.bash
colcon test --packages-select warehouse_control warehouse_planning \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
cd /tmp
python3 /source/scripts/validate_mission_execution.py \
  --case A --case B --repeat 3 --wall-timeout 600 \
  --output /evidence/new-mission-evidence
exit
```

Use a new output directory for each invocation and specify each case at most
once. Both harnesses protect an existing top-level `results.json`, but do not
protect partial case logs or reject duplicate case selections. For the legacy
route harness, supply a finite positive `--wall-timeout`; unlike the mission
CLI, its parser does not validate this bound. The mounted evidence survives
container removal. The image is a local prerequisite, not supplied by this
patch. These commands rerun the protocol; they do not promise bit-identical
simulator observations. To check the current source instead, export/build the
current HEAD plus this candidate in another directory and label its results
with that source version. That run uses the current model and cannot replace
this historical record.

Fault/empty cases are available through `--case`; omitting it runs all seven
case types. Existing controller regressions use `validate_route_following.py`.

## Historical source provenance — 2026-10-14

The implementation and its 49 mission tests are now committed in
[`726e620`](https://github.com/PeterxVarga/var_e9t_warehouse/commit/726e62051383da691468b80f4ae3d0c8d8b0052d).
The original validation split was prepared on that commit. The JSON's
`baseline` remains
`4df71f2f74d9a69c807f71bbabb9f64b88cdc686`, the original pre-implementation
measurement baseline; `implementation_commit` identifies the subsequent split.
Its measurement date and numerical observations have not been changed.

The original split notes reported a separate build and 342 passing tests on
2026-10-14. No separate XML for that date was located in the retained evidence;
this review uses the verified 2026-10-13 XML, not that additional run, as test
evidence. The two harnesses match the recorded historical hashes. The current
HEAD implementation/model no longer match those historical hashes. No historical
Gazebo measurement was relabeled as a current-source run.

## Current-source checks — 2026-10-25

The candidate was checked separately against HEAD
`feae5887b4b105d3eee5678f8924ae306261868c`. Its validation scripts still match
the historical script hashes; the runtime implementation, tests and model
have since changed. Historical measurement values and the 342-test count above
remain unchanged. The [IMU strategy campaign](../README.md#controller-strategy-validation--2026-10-20)
is separate evidence with explicit estimated-heading feedback; none of the
checks below uses that estimator or remapping.

- All three packages built in an isolated rootless Humble container. Installed
  source, model, graph, world, bridge configuration and launch files matched
  the exported current candidate (21 files).
- **282 targeted tests passed**, with zero failures, errors or skips (42.68 s):
  `test_mission_controller.py`, `test_mission_execution.py`, `test_route.py`,
  `test_controller.py`, `test_mission.py` and `test_mission_cli.py`. This was
  a targeted run, not a new full 342- or 656-test result.
- Three fresh mission harness probes passed: `empty`, `odom_loss_service` and
  `sigterm`. Two fresh shared route harness probes passed: `single_goal` and
  `invalid_goal`. They checked integration, sampled clearance and final rest;
  they are not a repeat of the six historical A/B accuracy measurements.
- Python syntax and real ROS/package imports, CLI help/rejections, existing-result
  protection, and synthetic orchestration/failure-reporting probes passed.
  Historical JSON, eight source hashes, test XML, all six mission tables,
  service errors/zero-command proofs and the five mission/ten controller
  summaries were audited against the retained evidence.

The first current-source attempts failed because the image's default ROS log
path was unwritable. Their logs were retained; the successful attempts used
`ROS_LOG_DIR=/evidence/ros-logs`. No controller or harness source was changed to
resolve that environment issue.

The historical pose reader continued collecting during clearance calculation
and cleanup. Its final raw stream and later `physical_samples` count therefore
extend beyond the rest-window snapshot. Reconstructing the first physical sample
at or after five seconds from the first post-braking sample independently
matches all six recorded rest durations, exactly in integer nanoseconds. For
A/1 the rest window is 196.427–201.441 s (5.014 s); the sample-count timestamp
is 201.697 s and cleanup ends at 202.240 s. These are different observation
boundaries, not different acceptance thresholds. The empty mission's raw
success schema has gates and services but no fault/recovery flags: its compact
false flags were checked against its REACHED/depot log, zero commands, empty
service list and absence of a fault event, rather than filled from defaults.

## Limits

The historical checks cover these two FIFO missions and the recorded headless
Fortress/DART dynamics. A residual physical/odometry difference accumulates
through turns; the observed depot error reaches about 0.105 m. Passing these
routes does not establish the same accuracy for arbitrary longer missions,
other speeds, loads, physics models or hardware. No further calibration was
justified or applied. Historical failures before the effective-track
calibration remain documented separately in the earlier route/odometry reports.

Continuous tolerance means continuous over observed valid feedback samples;
it cannot prove the unobserved physical trajectory. Clearance is sampled
static geometry, not continuous collision safety. There is no mechanical
loading, SLAM/localization correction, optimized ordering, replanning, dynamic
obstacle avoidance or independent drive watchdog. The reused process shutdown
requires a working ROS/command bridge; SIGKILL, crashes or bridge loss can leave
Gazebo holding its last command. No such protection was added in this slice.
