# Route-following validation — 2026-10-10

This is the historical **pre-calibration** record. The later
[repeated diagnosis and calibration](odometry-diagnostics.md) resolves the
physical-accuracy failure for the tested configuration and validates a
reserved route. Original measurements below are preserved unchanged.

At this pre-calibration stage, physical route acceptance was **not complete**.
Both routes reached every planned waypoint in odometry and satisfied timing,
sampled clearance and stopping gates, but both failed the independent 0.15 m
physical goal-error gate.

## Environment and source state

- Base commit: `8286d3076aeea5a4c2f9fd0f4cd6bbf8ae76949c`.
- The working tree was clean before applying the route-following implementation.
  Forward patch checking succeeded; reverse checking failed because the new
  route files were absent. The implementation was applied once.
- No applicable `AGENTS.md` was present. No commit or push was performed.
- Rootless Podman image: `localhost/var-e9t-warehouse:humble`, image ID
  `9d36a750d194`.
- Dedicated temporary container: `var-e9t-route-validation-20261010`;
  ROS domain 77 and Ignition partition `var-e9t-route-validation-20261010`.
  All owned simulation/controller processes were stopped after measurement;
  the temporary container was then removed.
- Ubuntu 22.04, ROS 2 Humble, Python 3.10.12, pytest 6.2.5,
  Gazebo Fortress 6.18.0, `ros-humble-ros-gz-bridge`
  `0.244.26-1jammy.20260907.225444`.
- Robot model, world, production bridge, graph and controller defaults were
  unchanged during the default validation runs.

## Build and package tests

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-up-to warehouse_control
source install/setup.bash
colcon test --packages-select warehouse_control warehouse_planning \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

All three packages built successfully. Final tests: **242 passed, 0 failures,
0 errors, 0 skipped** (134 control, 108 planning).

The first ROS run exposed 15 failures in the new test fixture. It mocked every
`Node.create_publisher()` call, including the parameter-event publisher created
by Humble during node construction, and tried to read `ParameterEvent.linear`.
The fixture now intercepts only `geometry_msgs/Twist` on `/warehouse/cmd_vel`;
other publishers retain the real ROS implementation. No production controller
change was required for this failure.

The initial terminal Ctrl+C integration trial also exposed a Humble shutdown
failure: a raised `KeyboardInterrupt` inside the pybind11 odometry take surfaced
as `RuntimeError: Unable to convert call argument to Python object`. Cleanup
sent zero, but the controller exited with status 1. Signal handlers now set a
stop flag rather than raise asynchronously. The shared runner uses
`rclpy.spin_once(..., timeout_sec=0.1)` and sends the final zero while ROS is
still alive. The regression suite covers SIGINT, SIGTERM, direct
`KeyboardInterrupt`, cleanup order and restoration of both signal handlers.

## Physical measurement method

The opt-in [validation harness](../scripts/validate_route_following.py) starts
a new headless Gazebo server for every case, using the installed world/model.
It starts directional command, odometry and clock bridges in separate
processes so odometry can be interrupted while command delivery stays alive.
The mappings match the production bridge configuration. Controller subprocesses
run the installed `ros2 run warehouse_control ...` entrypoints with
`use_sim_time:=true` and working directory `/tmp`.

An independent `ign topic --echo --json-output` subscription reads
`/world/warehouse/dynamic_pose/info`. The harness selects the named
`warehouse_robot` model pose and uses the top-level **Gazebo timestamp**;
wheel odometry is not used to reconstruct these physical positions.
All consecutive physical sample segments are tested against the seven
static shelf/wall rectangles expanded by a **0.3202 m robot radius**.
No intersections were found by this conservative sampled geometry test.

The route timer runs from controller process launch until observed `REACHED`.
There is a 180 s simulation deadline and a separate 240 s wall-time deadline
per case. After a one-second braking allowance, each case checks at least
five simulation seconds of physical rest. Rest thresholds are displacement
below 0.001 m, sampled translation speed below 0.001 m/s and yaw change below
0.001 rad. These are test thresholds, not a formal stopping guarantee.

## Default route results

| Measurement | Outer route `r3c2` | Inner route `r2c2` |
|---|---:|---:|
| Planned graph length | 10.000 m | 8.000 m |
| Sampled physical distance | 9.907 m | 7.908 m |
| Sampled odom distance | 9.908 m | 7.909 m |
| Time to observed `REACHED` | 81.600 s | 66.328 s |
| Final odom goal error (limit 0.05 m) | 0.048983 m — pass | 0.047594 m — pass |
| Final physical goal error (limit 0.15 m) | 0.464332 m — **fail** | 0.323104 m — **fail** |
| Physical samples | 5,169  | 4,272  |
| Largest physical sample gap | 0.019 s | 0.018 s |
| Sampled segment/obstacle intersections | 0 — pass | 0 — pass |
| Rest after braking | 5.000 s — pass | 5.016 s — pass |
| Ordered waypoint arrivals | 6/6 — pass | 5/5 — pass |
| Maximum rest translation / speed / yaw change | 0 / 0 / 0 | 0 / 0 / 0 |
| Final command | zero | zero |

Outer waypoint order: `r0c0 → r1c0 → r2c0 → r3c0 → r3c1 → r3c2`.
Inner waypoint order: `r0c0 → r1c0 → r2c0 → r2c1 → r2c2`.

Outer final world position: `(-0.464189, 2.988461)` m, target `(0, 3)` m.
Inner final world position: `(-0.323030, 0.993114)` m, target `(0, 1)` m.
`REACHED` means the final **odom** tolerance was satisfied; it does not establish
world-frame arrival.

## Fault, shutdown and regression checks

All eight remaining default cases passed. Each included independent physical
pose sampling, zero-command checks, sampled obstacle checks and at least five
seconds of physical rest after braking.

| Case | Observed result |
|---|---|
| Odometry transport loss | Only the odom bridge was stopped. Command bridge stayed alive; `FAULT` latched and zero stopped the robot. Restarting the odom bridge delivered fresh feedback without motion or rearming. |
| Actual terminal Ctrl+C | A controlling PTY received the Ctrl+C byte while the robot moved. The installed controller exited normally, DDS delivered zero and the robot stopped. |
| Direct node SIGTERM | SIGTERM targeted the installed controller child PID, rather than the `ros2 run` wrapper. Normal exit, observed zero and physical stop. |
| Invalid goal | Unknown `goal_node` exited nonzero, created no command publisher and produced no physical motion. |
| Unreachable goal | A temporary installed-share overlay isolated the depot in a copied graph. Nonzero exit before command publication; no physical motion. Production graph was unchanged. |
| Invalid odom start | Fresh, remapped feedback started at `(0.06, 0)` m. `FAULT` latched with only zero commands; no physical motion. |
| Real moved start | A sole temporary publisher moved the robot 0.114450 m, sent zero and was destroyed before controller launch. The follower rejected the displaced odometry, latched `FAULT` and produced 0 m additional physical motion. |
| Existing single-goal controller | Fresh default `(1, 0)` odom goal reached, with matching physical accuracy and sustained rest. |

The single-goal regression reached in 9.393 simulated seconds, with 0.047589 m odom error and 0.047589 m physical error.
Its rest interval was 5.003 s.

A separate outer-route diagnostic used `max_angular_speed:=0.2`, ROS domain 78 and an independent Ignition partition. It reached every waypoint in 89.036 s but the physical error was **0.551831 m**. Reducing this limit did not resolve the disagreement, so controller defaults were retained.

The moved-start check used domain 79 and its own Ignition partition. Simultaneous diagnostics used independent worlds and command namespaces; each world had at most one command publisher.

## Reproduction and artifacts

To reproduce these historical failures with the current source, first copy the
installed model resource directory to a fresh
`/workspace/log/route-validation/baseline-models/` directory. In its copied
`warehouse_robot/model.sdf`, set only DiffDrive's `wheel_separation` to `0.38`;
keep the physical wheel centers and all other model parameters unchanged.
The [diagnostic report](odometry-diagnostics.md#reproduction-and-artifacts)
describes this baseline override and the separate calibrated runs.

Run only in a dedicated container with isolated ROS/transport namespaces
and no other velocity publisher:

```bash
export ROS_DOMAIN_ID=77 IGN_PARTITION=var-e9t-route-reproduction
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
cd /tmp
python3 /workspace/scripts/validate_route_following.py \
  --model-dir /workspace/log/route-validation/baseline-models \
  --output /workspace/log/route-validation/reproduction
```

Without `--model-dir`, the current installed model uses the calibrated `0.363`
effective track and does not reproduce the pre-calibration configuration.

The full default harness exits nonzero when either physical-accuracy gate
fails. Use `--case r3c2` or another named case to run a subset. Every case
produces `controller.log`, bridge/server logs, raw `physical.jsonl`,
`odom.jsonl`, `commands.jsonl`, and `result.json`; the output root also contains
`results.json`. Logs are kept under the ignored workspace `log/` directory.
The nine-case reference output is `log/route-validation/final-corrected/`;
the extra real moved-start case is in `log/route-validation/moved-start/`.
The slower-turn diagnostic is in `log/route-validation/slow-turn-diagnostic/`.
The current full harness includes all ten default cases.
[Machine-readable measurements](route-following-validation-results.json)
include all reference outcomes and source SHA-256 hashes. No raw pose/log
artifacts or transfer notes are added to the public source tree.

## Limits observed before calibration

Before calibration, wheel odometry and physical yaw differed after turning.
In a separate initial
outer-route run, at simulation time 35.004 s the independently measured yaw
was 1.640269 rad and the nearest odom sample was 1.570681 rad, with a 0.004 s
stamp difference. The accumulated physical lateral displacement was about
0.281 m even though transformed odometry stayed near the nominal aisle line.
This is evidence of odometry/physical disagreement; the test does not isolate
a specific contact-model or wheel-calibration cause.

The known spawn transform aligns frame origins but cannot correct subsequent
wheel-odometry drift. Physical route acceptance at this stage required an
independently validated odometry/model calibration or localization solution.
The later [calibration report](odometry-diagnostics.md) records that validation
and the remaining residual errors. A waypoint
controller that only sees wheel odometry cannot independently certify world
arrival. No goal-specific coordinate offset or relaxed acceptance limit was
introduced.

The checks use one static world and a simplified contact model. Sampled
segments are not a continuous dynamic collision guarantee. There is no
independent drive watchdog, online localization, dynamic-obstacle handling
or replanning. SIGKILL, a crash or command-bridge loss can leave DiffDrive
holding the previous command. Headless simulation results are not hardware
validation.
