# IMU heading odometry integration — 2026-10-19

This is a standalone, opt-in simulation feedback integration. It builds and
tests against commit `8ac5d264249c8d6b12cac3eba0348b7bb88d349b` without the
pending mission-controller strategy or old mission-validation changes.
A separate validation copy adds only the four existing strategy source/test
files for the targeted fresh NN run. Those files are not part of this candidate.
**The complete three-strategy physical acceptance campaign remains outstanding.**

## Interfaces and observation model

The existing base link contains a 100 Hz Fortress IMU with a `(0,0,0.3)` rad
mount rotation and zero translation. The CPU IMU system is installed in the
world. The simulation launch reads the bridge configuration and exposes
`/warehouse/imu`: `ignition.msgs.IMU` to `sensor_msgs/msg/Imu`, frame
`warehouse_robot/base_link/imu_heading`. The orientation reference is CUSTOM,
identity RPY relative to world; no noise is configured. This ideal simulated
absolute attitude does not establish real-IMU yaw observability or robustness
to bias, drift or magnetic disturbances.

Physical wheel separation remains **0.38 m**, radius **0.08 m**, and calibrated
DiffDrive effective track **0.363 m**. Existing mass, inertia, collisions and
calibration comments are preserved. Parsed model/world equivalence, after
removing only the new sensor/plugin, was checked. See the existing
[calibration explanation](odometry-diagnostics.md) for the effective track.

`/warehouse/odom` remains raw and is never overwritten. The separate
`/warehouse/imu_odom` output reintegrates signed wheel displacement chords
under midpoint IMU heading. The mathematical core is unchanged from the
feasibility candidate: full quaternion mount removal, static spawn-yaw
conversion, short-arc ±pi interpolation and no extrapolation. IMU endpoints
must bracket each wheel interval; maximum IMU and wheel gaps are 0.1 s.
Raw accumulated XY is not copied to the estimated XY. The installed sensor
mount is read from SDF and its world-reference assumptions are checked.

Dynamic Gazebo world-pose transport is used only by an independent observer,
never by navigation. Static spawn is the only world/odom origin transform.
No TF publisher, relocalization, target-dependent correction or replanning is
introduced. An explicitly remapped controller is the only velocity publisher.

## Initial readiness versus runtime faults

The estimator starts in `WAITING_FOR_SENSORS`. Missing input, an absent clock
or old queued startup samples do not trigger the old one-second runtime
watchdog. No feedback is emitted until the clock is positive and both channels
have at least two valid, fresh, advancing source samples with matching IMU
brackets. Sources must be within 0.15 simulation seconds of the clock and no
older than 0.5 wall seconds. Frames, finite values, unit quaternion and source
stamp fields are checked. Invalid input and backwards time fail closed;
repeated stamps never establish readiness or refresh freshness.

A read-only DOUBLE `startup_timeout` defaults to **10 wall seconds from node
construction**, independent of simulation time. The installed launch exposes
that argument and sets `use_sim_time=true`. Expiry latches `FAULT` with an
explicit startup-timeout log and no feedback. The node remains alive for
inspection. A controller with no feedback remains at zero. Correcting the
missing input after expiry does not activate this estimator; use a fresh
simulation and restart both nodes.

Once `ACTIVE`, stale/missing/invalid inputs, source gaps, backwards time or
queue overflow latch estimator `FAULT`; feedback stops rather than receiving
new timestamps on old data. The existing controller then stops on its own
feedback timeout and latches `FAULT`. No automatic recovery or restart is
implemented. Controller crash, SIGKILL and command-bridge loss remain outside
this protection; there is no independent drive watchdog.

## Output meaning

Output is `nav_msgs/msg/Odometry`, frame `odom`, child
`warehouse_robot/base_link`. Its source timestamp is copied from wheel input.
Only the planar pose is a new estimate. Z/roll/pitch are planar placeholders.
Twist and its covariance are exact copies of raw wheel input in the body
frame, not IMU-derived or fused velocity. Estimated **pose covariance is
unavailable**: the zero-filled ROS array is a structural placeholder, not a
claim of perfect certainty and not suitable as covariance input to a filter.
No arbitrary uncertainty numbers are invented. Consumers must explicitly
accept this limited pose-only estimation contract.

For the complete build/start/remap commands see the README's
[IMU heading odometry section](../README.md#imu-heading-odometry-simulation).
The standalone mission example uses FIFO with requests explicitly in the
chosen order; the default controller/feedback selection is unchanged.

## Actual checks

Owned rootless container: `var-e9t-imu-integration-20261019`, image
`localhost/var-e9t-warehouse:humble` (`c5df849eb3d9`), ROS Humble / Ubuntu 22.04,
Fortress 6.18.0 and bridge 0.244.26. Other project containers were not operated.

- Standalone candidate: all three packages built; **559 tests passed**
  (325 planning, 234 control), zero failures/errors/skips. This is the 508-test
  committed baseline plus 20 arithmetic and 31 boundary/readiness cases.
- Separate strategy-overlay copy: all three packages built; **627 tests passed**
  (325 planning, 302 control), zero failures/errors/skips. All installed estimator
  sources matched the standalone candidate byte for byte.
- Six installed launch/controller checks from `/tmp` used synthetic source
  messages: IMU first, wheel first, controller late, missing IMU, missing wheel,
  missing clock. Each passed. First-channel-only waiting exceeded two wall
  seconds without premature FAULT. Missing-input cases used a 3.0 s timeout;
  FAULT was observed 3.76–3.81 s after launching, including process startup.
  They emitted no estimator feedback and only zero commands, including after
  the missing sensor returned. Pure boundary tests check the exact timeout
  boundary, read-only/type rules, stale backlog, repeated stamps and latches.
- Build/test outer deadlines: 180/240 wall seconds. Installed readiness checks
  had a 100 s outer deadline. The default 10 s value and synthetic short-timeout
  paths were checked separately.

## Fresh targeted NN mission

The original request list was `['r1c2','r2c0','r2c4']`, strategy `nn`, executed
original IDs **2,1,3**, with **28 m planned graph length**. The installed
simulation launch exercised the actual four-channel bridge configuration;
the installed estimator launch used its readiness policy without a manual
IMU warmup. Entrypoints ran from `/tmp`. The pending strategy integration was
included only in the separate validation copy. One fresh run was requested
for this integration step; the earlier prototype's repeats remain historical.

| Original request / node | Max estimated target error m | Max physical target error m |
|---|---:|---:|
| 2 / r2c0 | 0.049926 | 0.050414 |
| 1 / r1c2 | 0.049999 | 0.050771 |
| 3 / r2c4 | 0.049909 | 0.050237 |
| Depot (arrival through rest) | 0.049714 | 0.049121 |

Execution took **234.512 simulation seconds**
(244.397 measured wall seconds). Planned graph distance is not
physical travel distance or runtime. Exact waypoint/original-request order,
separate >=1 s continuous-tolerance zero-command services, <=0.05 m estimated
and <=0.15 m physical target errors, 360 s simulation / 600 s wall deadlines,
zero sampled obstacle intersections and final zero all passed. Final physical
rest lasted **5.014 s**, with zero measured displacement,
speed and yaw change. Sampled obstacle checking uses the unchanged 0.3202 m
footprint approximation; it is not continuous collision proof.

Common-source-time interpolation gave maximum raw-wheel/world position
mismatch **0.160266 m**, compared with
**0.001103 m** for the new estimate.
Maximum yaw disagreements were **0.02973531**
and **0.00006940 rad**, respectively.
Physical samples are bracketed/finite/strictly increasing, with gaps <=0.1 s;
XY is linearly interpolated and yaw follows the short arc. The historical
0.186843 m uncorrected failure and the frozen feasibility archive were not
edited or reclassified.

## Live sensor-loss checks

Each channel was interrupted separately in a fresh simulation during travel
and service, with the command bridge still alive. All four passed:

| Probe | Loss to controller FAULT, sim s | Final rest s | Result |
|---|---:|---:|---|
| loss-wheel-travel | 0.996 | 5.006 | PASS |
| loss-wheel-service | 0.996 | 5.005 | PASS |
| loss-imu-travel | 0.982 | 5.004 | PASS |
| loss-imu-service | 0.999 | 5.016 | PASS |

Estimator output stopped and stayed latched after source recovery. To also
check the controller's independent latch with returning **valid feedback**,
the harness injected advancing raw-wheel feedback only after controller
FAULT; the estimator itself remained latched. These deliberate injected
samples are recorded separately and are not presented as estimator output.
No world pose was injected. Neither movement nor service completion resumed.
All cases verified zero command and >=5 s physical rest. The two prototype
startup failures remain preserved in the untouched earlier archive; no new
startup failure occurred in this integration validation.

## Evidence and limits

[Compact measured results](imu-odometry-integration-results.json) include
source-result SHA-256 values, source-aligned comparisons and explicit scope.
Raw JSONL, process logs, commands, build/test XML and exact harness scripts are
kept outside the public candidate in the local integration artifact directory
`/tmp/warehouse-imu-integration-20261019-gsprp2qr/`. The final artifact archive
contains the candidate patch, manifests and these measurements. Reproduction
uses a new output directory, ROS domain and Ignition partition; do not overwrite
prior evidence. Existing pending harness files were only copied, never edited.

This establishes targeted simulation integration evidence, **not full system
acceptance**. Ideal absolute attitude, remaining wheel-distance/slip errors,
unknown covariance, limited repeat coverage, sampled obstacles and the missing
drive watchdog remain limitations. The complete fresh FIFO/NN/NN+2-opt campaign
and realistic noise/bias/hardware validation are separate work.
