# Mission pause service validation

`/warehouse/pause_mission` is a `std_srvs/srv/Trigger` service on the mission
controller. A successful response acknowledges acceptance of the pause request;
it does not certify physical rest. A started mission is paused immediately, its
checkpoint retains each unfinished original request, and zero velocity is
published. Repeated requests return success without changing the checkpoint.
Requests before startup or after `REACHED` or `FAULT` fail. There is no ROS
resume interface in this change. Odometry and time faults remain latched during
pause, and feedback recovery cannot resume motion.

## Predefined Gazebo gates

Before the targeted trials, fix these criteria: one fresh headless Fortress
world per case; request pause during forward travel and during the first pickup's
simulated service; accepted response within 2 wall seconds; first observed zero
command within 2 wall seconds of the request; no observed nonzero command after
the observed zero; braking completes within 1 simulation second; at least 5
simulation seconds of subsequent sampled rest, with displacement and speed each
below 0.001 m and 0.001 m/s and yaw change below 0.001 rad; world-pose sample
gaps strictly positive and below 0.1 simulation second. During the service case,
no completion for the interrupted request may appear. Capture request, response
and command observations using the observer's simulation and monotonic wall
clocks. Measure sampled world-pose path length from the request to the first
post-braking sample. The world-pose stream and prior harness are independent
verification only and never feed navigation.

These sampled checks cannot establish continuous physical rest, collision
freedom, a drive watchdog, or stopping on real hardware. Observer callback and
transport delays limit timestamp alignment. Raw successes and failures are kept
outside the public candidate; historical measurements are not new results.

## Current candidate results — 2026-10-31

The source was built in a newly owned rootless Podman container using local
`localhost/var-e9t-warehouse:humble` image ID `c5df849eb3d9` (ROS 2 Humble,
Gazebo Fortress), with separate ROS domains and Ignition partitions. All three
packages built. The final planning/control regression passed 909 tests: 439
planning and 470 control, with no errors, failures or skips. The 13 new ROS
pause tests cover actual service calls, repeated pauses, partial service,
original request indices, terminal and pre-start rejection, zero commands,
paused feedback/time failures and latched faults after feedback recovery.
Existing goal, route, mission and estimator regressions are included.

From `/tmp`, the installed `mission_controller` entrypoint advertised the
service as `std_srvs/srv/Trigger`; an actual request before odometry returned
`success=False` with `Pause unavailable: mission has not started.` The first
colcon test invocation accidentally discovered zero packages, so it is not a
test result; the subsequent run explicitly supplied the source base path.
Both logs were retained.

Each row below is a new fresh-world trial of this candidate, not a historical
measurement. Simulation times are Gazebo clock seconds. Wall times are monotonic
seconds relative to the start of the corresponding trial, not simulation time.
All timestamps are observer-side receipt or call times, not exact controller or
physical event times.

| Case | Request sim / wall (s) | Response sim / wall (s) | First observed zero sim / wall (s) | Path to sampled rest (m) | Path over 1 s braking window (m) | Rest (sim s) |
| --- | --- | --- | --- | ---: | ---: | ---: |
| Travel | 4.540 / 7.032079 | 4.542 / 7.034511 | 4.541 / 7.033916 | 0.008106 | 0.008108 | 5.011 |
| Service | 18.728 / 21.103640 | 18.731 / 21.106517 | 18.729 / 21.104520 | 0.000178 | 0.000178 | 5.012 |

Sampled rest began at simulation times 4.641 s and 18.748 s, respectively.
The initial world-pose samples used for these paths were at 4.539 s and
18.731 s, respectively, so in the service case the first 3 ms after the request
are not observed in the path. The sampled rest onset is the beginning of the
observed suffix satisfying the rest limits; it is not the exact physical stop
time. The one-second path ends at the first sample after request + 1 s. All
rest displacement, speed and yaw maxima were zero at the recorded precision;
maximum world-pose gap was 0.018 s in each case. No nonzero command was observed
after the first observed zero (121 and 122 command samples, respectively).
The service trial logged arrival followed by pause and no service completion.
The zero observed during service may also be a timer zero; the callback tests
separately assert that the service callback itself publishes zero immediately.
No sampled static obstacle intersection occurred; this is not proof of
continuous collision freedom.

Raw observations, successful and exploratory runs, build/test logs and XML are
in ignored `log/mission-pause-validation-20261031-NFYrum/`. The observer's
clock callbacks, command callbacks, and world-pose stream are asynchronous. The
0.06 s alignment bound and 0.1 s maximum sample-gap gate do not prove what
happened between samples. The service response confirms acceptance only; no
independent drive watchdog, continuous collision guarantee or real hardware
stopping behavior is established.

## Reproduction

Use an isolated source export with this candidate applied, an existing local
Humble/Fortress image and a new writable evidence directory. The prior harness
is used only as an observer and process owner. For example, inside a newly owned
container with the source mounted read-only at `/source` and evidence writable
at `/evidence`:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /evidence/build
cd /evidence/build
colcon build --base-paths /source/src --packages-up-to warehouse_control
source install/setup.bash
colcon test --base-paths /source/src --packages-select warehouse_control warehouse_planning \
  --return-code-on-test-failure
colcon test-result --verbose
cd /tmp
python3 /source/scripts/validate_mission_pause.py --output /evidence/new-pause-trials
```

Set unique `ROS_DOMAIN_ID` and `IGN_PARTITION` values, and set `ROS_LOG_DIR` to
a writable evidence directory. The output directory must be new. Different
simulation and wall timing is expected on another run.
