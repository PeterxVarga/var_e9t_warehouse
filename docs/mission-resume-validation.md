# Mission resume service

`/warehouse/resume_mission` uses `warehouse_interfaces/srv/ResumeMission`:
`string start_node` in the request, `bool success` and `string message` in the
response. For example, after pausing and observing the robot at `r1c0`:

```bash
ros2 service call /warehouse/resume_mission warehouse_interfaces/srv/ResumeMission \
  '{start_node: r1c0}'
```

Resume is available only for a started, paused mission. The explicit node must
exist in the startup geometry-checked graph and the fresh odometry position must
be within the configured waypoint position tolerance of that node under the
startup static world-to-odom transform. Yaw is unrestricted. This interface
is not localization and does not support resuming from the middle of an edge.
`FAULT` and `REACHED` remain latched. Odometry or time faults continue to latch
during pause under the existing controller rules.

## Observed rest and synchronization

After each new pause, the controller accumulates a sliding window of odometry
samples while commanding zero. It requires at least 0.5 seconds on both the
odometry stamp and monotonic receipt clocks, with each adjacent stamp and
receipt gap strictly positive and at most 0.2 seconds. The retained window
spans at most 2 seconds on either clock; if a clock advances too slowly to
cover the minimum in that bound, resume is rejected. The most recent sample
must be at most `min(0.2, odom_timeout)` seconds old by both receipt and ROS
clock, with a nonzero, non-regressing ROS clock. Every linear and angular twist
component must be finite and at most 0.01 in absolute value (m/s or rad/s);
all positions must be within 0.002 m of the first sample, and wrapped yaw
within 0.002 rad. A duplicate, invalid or missing stream cannot fill the
window. A sampling gap resets it; moving samples must age out before another
attempt can succeed. Frame, pose, quaternion and timestamp validation remain
active. Repeated pause requests do not restart an otherwise valid window.

These are finite, sampled odometry observations and thresholds, not a real
hardware stopping guarantee, continuous-motion proof, independent drive
watchdog or localization guarantee. Odometry and actual pose may disagree.
The ROS message does not distinguish measured zero twist from fields left at
their default zero values; the odometry producer must supply meaningful data.
An operator must select the node based on appropriate external knowledge.

Odometry, control timer and service callbacks use the node's default mutually
exclusive callback group. The service verifies the window and plans from a
snapshot, then checks the snapshot and both clocks again immediately before
committing the complete candidate. During that callback other callbacks cannot
overwrite it. If planning or validation fails, the old plan and checkpoint stay
in place and the paused controller publishes zero. If the last odometry sample
was already consumed by a paused control tick, the core requires a strictly
newer sample and clock; retry after further feedback. A successful response
only accepts the new plan. The following control cycle still requires newer
feedback before producing a nonzero command.

Completed original request IDs are retained and excluded from the replacement
plan. Partial service time is discarded; the interrupted original request is
served again for the full one-second simulated interval. Repeated node names
remain distinct requests. This is simulated service, not proof of loading.

## Reproduction

Build all four packages with ROS 2 Humble, source the resulting install and run
`colcon test --base-paths /source/src --packages-select warehouse_planning
warehouse_control --return-code-on-test-failure`. For an isolated Gazebo
Fortress trial, run `python3 /source/scripts/validate_mission_resume.py --output
<new-directory>` with separate ROS domain and Ignition partition, and the
installed world and model paths available. The script records independent
sampled world poses, ROS odometry, commands and controller logs. It pauses the
second of two identical requests, rejects an unknown node, observes rest,
resumes at `r1c0`, verifies original request completion order and full service
and measures the depot arrival. A second fresh world stops the odometry bridge
and checks rejection and the existing latched timeout. World poses are only
verification observations and never navigation input.

## Candidate measurements — 2026-11-02

The candidate was built with the existing rootless Humble/Fortress image
`c5df849eb3d9`. Four packages built and the planning/control regression passed
930 tests (439 planning and 491 control), with no failures or skips. The first
exploratory regression, run concurrently with Gazebo, had one process-test
startup timeout; it and the later successful isolated runs are retained.

In a fresh world the request sequence was `r1c0`, `r1c0`, `r0c0`. The first
request completed at feedback time 19.800 s. The second arrived at 19.850 s;
pause was requested at observer simulation/wall times 19.885/22.836 s. The
unknown-node request returned `unknown graph node: invalid-node` with no
observed nonzero command. After 0.989 s of sampled world-pose rest (60 samples,
20.892–21.881 s), maximum displacement, speed and yaw change were zero at
recorded precision; maximum sample gap was 0.017 s. Resume from `r1c0` was
accepted at observer simulation/wall times 22.203/25.209 s after retries for a
strictly newer feedback sample. Request 2 started again at feedback/clock
22.250/22.271 s and completed at 23.250/23.271 s, exactly 1.000 s later on
both clocks. Request 3 completed at feedback time 43.900 s and the final depot
was reached at 43.950 s. Completion IDs were 1, 2, 3, each once.

In a second fresh world, stopping the odometry bridge produced the explicit
`odometry or ROS clock is stale or invalid` rejection; the existing
odometry timeout then latched `FAULT`. No nonzero command or completion of
the interrupted request was observed after the rejected resume. Its rest
window contained 59 world-pose samples over 20.784–21.770 s, with zero
displacement, speed and yaw change at recorded precision and a maximum
sample gap of 0.017 s.

Nearest independent world-pose samples give physical position errors of
0.04955 m, 0.04955 m and 0.05193 m at the three completions, and 0.05193 m at
the final depot. These are time-aligned sampled measurements, not guaranteed
continuous errors or real-world positioning accuracy. Observer timestamps and
controller feedback timestamps have different sources.

Raw controller logs, odometry, command and world-pose streams, test logs and
both successful and exploratory trial results are preserved in ignored
`log/mission-resume-validation-20261102-codex/`. The results describe this
simulation and do not prove a physical stopping guarantee.
