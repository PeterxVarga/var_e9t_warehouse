# Paused mission replanning with logical closures

The demonstrated flow is mission → pause → provide all logical closures →
plan a detour → resume → finish remaining original requests → return to depot.
Closures are manually supplied planning data. No sensor detects an obstacle,
no physical obstacle is inserted into Gazebo, and no hardware safety guarantee
is made. Resume requires an explicit graph node; intermediate edge positions
are unsupported.

## Interfaces and transaction

`/warehouse/resume_mission_with_closures` uses
`warehouse_interfaces/srv/ResumeMissionWithClosures`:

```text
string start_node
warehouse_interfaces/ClosedEdge[] closed_edges
---
bool success
string message
```

Each `ClosedEdge` contains `string node_a` and `string node_b`. It names one
undirected edge of the original startup graph. Either endpoint order is valid;
repeated edges, including reversed duplicates, unknown endpoints, self-loops
and nonexistent edges are rejected. `closed_edges` is the **complete** desired
closure list, not a delta. An empty list explicitly reopens all edges.

The existing `/warehouse/resume_mission` (`ResumeMission`, unchanged request
and response types) retains the last accepted closures. Both services call the
same internal resume implementation. Every request filters the original,
geometry-checked graph with `graph_without_edges`, then uses
`MissionProgress.resume_from_node`. Replanning preserves remaining checkpoint
order without rerunning NN/2-opt. Original request IDs remain distinct even at
repeated target nodes; completed requests are excluded. An interrupted service
starts a new full one-second simulated wait.

All existing [resume checks](mission-resume-validation.md) remain active:
started PAUSED state, continuous sampled rest on both clocks, finite velocity,
fresh odometry, advancing timestamps and the start-node coordinate check using
the static spawn transform. The complete candidate is validated before the
precommit snapshot/freshness check. Plan, accepted closures and visualization
change only after successful execution-plan replacement. A failed request
leaves the old plan, routes, checkpoint and accepted closures unchanged,
publishes zero while PAUSED, and publishes no rejected candidate as accepted.
Normal odometry timeout/invalid-feedback faults still latch FAULT during pause.
Accepted closures are in-memory state for this controller instance, not durable
configuration. A successful service response accepts a plan; it does not prove
that execution has completed.

## Build and run

Use the existing `localhost/var-e9t-warehouse:humble` image in a dedicated
rootless Podman container. For example, from the repository root:

```bash
podman run --rm -it --name warehouse-closures-demo \
  --network private --ipc private \
  -v "$PWD:/workspace:Z" \
  -e ROS_DOMAIN_ID=173 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=warehouse_closures_demo \
  localhost/var-e9t-warehouse:humble bash
```

Use a domain/partition unused by other sessions. In that container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-up-to warehouse_control
source install/setup.bash
ros2 launch warehouse_sim simulation.launch.py headless:=true
```

In another shell in the **same container**, source both setup files, then:

```bash
ros2 run warehouse_control mission_controller --ros-args \
  -p use_sim_time:=true -p 'pickup_nodes:=["r1c0","r1c0","r0c0"]'
```

Run one velocity-command publisher. Pause at a graph node, then allow braking
and at least 0.5 seconds of continuous fresh, stationary odometry. These manual
examples assume the robot was paused at `r1c0`; if the robot is between nodes,
resume is rejected rather than snapping its pose onto a node:

```bash
ros2 service call /warehouse/pause_mission std_srvs/srv/Trigger '{}'
ros2 service call /warehouse/resume_mission_with_closures \
  warehouse_interfaces/srv/ResumeMissionWithClosures \
  '{start_node: r1c0, closed_edges: [{node_a: r0c0, node_b: r1c0}]}'
```

With that edge closed, the return path goes along row 1 to column 4, down to
row 0, then back to the depot along row 0. A later pause can resume while
retaining the same closures, or explicitly replace them with an empty list:

```bash
ros2 service call /warehouse/resume_mission warehouse_interfaces/srv/ResumeMission \
  '{start_node: r1c0}'
# Alternative after a pause: reopen every edge.
ros2 service call /warehouse/resume_mission_with_closures \
  warehouse_interfaces/srv/ResumeMissionWithClosures \
  '{start_node: r1c0, closed_edges: []}'
```

Each call requires a new pause if the preceding one succeeded. A call can race
the paused timer for the latest feedback sample; a `strictly newer` rejection
may be retried after another odometry sample. Other errors require correcting
the stated condition. Do not bypass the freshness/rest checks.

## RViz display and frames

`/warehouse/mission_plan_markers` is `visualization_msgs/msg/MarkerArray`,
reliable, transient local, depth 1. A late subscriber receives the complete
latest snapshot. Set the RViz MarkerArray subscription to **Reliable / Transient
Local**. A ready-made configuration is installed:

```bash
rviz2 -d "$(ros2 pkg prefix warehouse_control)/share/warehouse_control/rviz/mission_plans.rviz"
```

RViz **Fixed Frame must be `odom`**. Graph coordinates are world coordinates,
converted with the same static `PlanarTransform` used for execution. With the
current spawn `(-4, -3, yaw=0)`, world `(-4,-3)` becomes odom `(0,0)`, and
world `(-4,-1)` becomes odom `(0,2)`. No complete TF tree or dynamic localization
is supplied, and selecting `world` as Fixed Frame without additional TF would
be incorrect for these messages. Small marker Z offsets separate the lines;
they do not represent measured heights. Labels also have display-only planar
offsets for readability; spheres and line endpoints retain the graph XY.
Without any TF publisher RViz may show a global `No tf data` warning even
though these same-frame markers render correctly. This display does not add
the missing robot TF tree.

| Display | Meaning |
|---|---|
| Blue line | Full active plan immediately before the latest resume/pause |
| Green line | Full currently accepted execution plan |
| Red segments | Accepted logical closures |
| Yellow sphere and label | Explicit start of the accepted plan, not current robot position |
| Pink spheres and request labels | Remaining original request IDs, including duplicates |
| White depot label and legend | Return depot; all lines are plans, not travelled paths |

The initial plan is also accepted. Before the first resume the blue and green
plans may overlap. Lines retain their complete snapshots, including traversed
prefixes; remaining-request labels disappear on service completion. They are
not a live position trace. Every publication begins with DELETEALL on this
dedicated topic, then adds the whole current snapshot. Replaced closures and
old labels therefore cannot survive as stale markers. Do not mix unrelated
marker publishers into this topic. Rejected closures are not drawn as accepted;
the last accepted display stays intact. Stationary one-node plans use a start
marker without an invalid one-point line.

## Acceptance and reproduction

From a sourced build, without an active simulation or controller:

```bash
colcon test --packages-select warehouse_planning warehouse_control \
  --return-code-on-test-failure
colcon test-result --verbose
# This script owns two fresh servers/bridges/controllers. Do not also launch them.
python3 scripts/validate_mission_closures.py --output /tmp/new-closure-evidence
```

The output directory must not exist. Run only in a dedicated container with
isolated ROS/Gazebo settings. The script preserves raw world poses, odometry,
commands, marker messages, controller/bridge/Gazebo logs and result JSON.
Navigation consumes odometry only; world poses are independent observations.

Acceptance requires:

- Valid closures produce a reachable remaining plan and depot return without
  using closed edges. Empty closures reopen edges. The compatibility service
  retains accepted closures.
- Unknown/duplicate/nonexistent edges, unreachable targets, invalid nodes,
  wrong position/state, insufficient/moving/stale feedback and failed
  precommit reject without partial plan/checkpoint/closure replacement.
  PAUSED rejections continue commanding zero.
- Completed requests stay completed; repeated targets keep their original
  IDs and separate service waits.
- Marker coordinates match execution's static transform; before/accepted
  lines and closures match their snapshots. Rejected candidates are absent,
  DELETEALL replaces old displays and late subscription works.
- A fresh detour trial records the actual observed waypoint order, remaining
  completions, depot arrival and at least five seconds of sampled physical
  rest after braking. A separate unreachable trial remains paused with zero
  commands, no extra completion and unchanged accepted markers.

Fresh measurements and actual RViz capability checks are recorded in
[the validation report](mission-closures-validation.md). Earlier resume,
controller and accuracy measurements remain historical, separate evidence.
The unresolved raw-wheel physical-accuracy limitations are unchanged. The
earlier local diagnosis (`docs/mission-controller-accuracy-diagnosis.md`)
remains separate pending documentation outside this commit. Committed
[odometry diagnostics](odometry-diagnostics.md) describe the calibration and
its limits.
Sampled rest/clearance observations do not establish continuous collision
safety, localization accuracy or drive-watchdog behavior.
