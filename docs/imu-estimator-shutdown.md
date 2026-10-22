# IMU estimator shutdown validation — 2026-10-22

Candidate base: `a265bf54ba5d2535dd23ae5b355b24f7007afba5`.
The pending mission-validation files remain outside this candidate.

## Defect and change

The earlier strategy campaign recorded `KeyboardInterrupt` in
`HeadingNode.destroy_node()` and estimator exit status -2 during cleanup.
Fresh direct installed-process reproduction, without launch forwarding:
10 single SIGINT trials exited zero; 10 trials with two SIGINTs 1 ms apart
all exited -2 with cleanup tracebacks. This isolates a repeated-interrupt
cleanup defect; it does not establish a single-signal failure.

`main()` now installs flag-only SIGINT/SIGTERM handlers before ROS initialization,
disables rclpy's default signal handlers, and uses a 0.1 s bounded `spin_once()`.
The handlers remain installed through node destruction and context shutdown,
then are restored even if cleanup fails. Context cleanup is attempted even when
node destruction raises. Known configuration errors still return 1; unexpected
runtime errors propagate. No general exception suppression was added.

The estimator publishes no final synthetic feedback or motion command.
The controller's existing feedback-loss watchdog remains responsible for stopping.
All module code outside imports and `main()` is AST-identical to the base.
Estimation, calibration, sensor readiness, freshness limits and latches are unchanged.

## Validation

A dedicated rootless, network-disabled Humble container used image
`c5df849eb3d9fa3e184d860803c8dbba7dce9c51cc7bfb3c3c4362bc6d9657af`.
Both baseline and candidate were built into separate, non-symlink installs.
All three packages built successfully. Full `colcon test` and `test-result`:
**656 passed, zero errors, failures or skips** (325 planning, 331 control;
the simulation package has no registered tests).

The 29 new cases comprise 14 lifecycle/error tests and 15 installed-process tests:
12 SIGINT/SIGTERM cases across WAITING_FOR_SENSORS, ACTIVE and FAULT, each with
single and repeated signals; two estimator/controller integration cases; and
one invalid-startup case. All 12 shutdown cases exited zero within 0.265 s
(3 s deadline), without traceback, forced cleanup or a remaining process/group.
FAULT was induced by the startup watchdog. Four ACTIVE cases checked advancing
source stamps, planar pose, orientation, raw twist and covariance field contracts;
the same analytic checks also passed on the installed baseline (2/2 single-signal cases).

Both integration cases first observed nonzero mission-controller commands, then
stopped only the estimator while sensor inputs continued. With `odom_timeout=0.3`,
the controller logged feedback-loss FAULT within 0.117 s after estimator exit,
continued publishing zeros, and stayed latched through fresh feedback injection.
Neither service completion nor mission completion occurred. Every owned subprocess
was reaped; no ROS node remained after the suite. The dedicated container was
stopped normally and removed after validation.

Evidence: `log/imu-estimator-shutdown-20261022/`, including `reproduction.json`,
`build.log`, `test.log`, `results.log`, per-process logs/exit records under
`full-test-tmp/`, and `final-verification.json`. Three preliminary harness failures
(context/executor binding, startup heading, completed-process tracking) are retained
separately; the final full suite passed after those harness corrections.

## Limits

These are real installed ROS processes with synthetic sensor inputs and a wall
clock. Physical braking, launch forwarding, hardware timing and signals before
entry into `main()` are not newly validated. Shutdown waits for an in-flight
initialization/callback/destructor to return; 0.1 s is the idle spin bound, not a
universal process-exit guarantee. The ten-mission campaign was not repeated:
there is no motion/algorithm change or observed regression. Earlier campaign
measurements and their historical cleanup limitation remain preserved.
