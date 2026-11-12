# Integrated warehouse simulation demonstration

This scenario starts the existing Gazebo Fortress warehouse, ROS/Gazebo bridges,
IMU/wheel heading odometry, mission controller and optionally RViz from one ROS
launch. It **does not** add obstacle sensing, a physical blockage, hardware
localization, or a TF broadcaster. Closures are manually entered **logical**
graph edges. The simulated IMU reports ideal world-referenced orientation;
wheel distance errors and unknown pose uncertainty remain.

## Prerequisites and isolated container

Use ROS 2 Humble / Gazebo Fortress with the existing local
`localhost/var-e9t-warehouse:humble` image. With rootless Podman, start in the
repository root. Use an unused name/domain/partition (the example belongs only
to this demo); do not use another project's container.

```bash
podman run --rm -it --name warehouse-demo-interactive \
  --network private --ipc private \
  -e ROS_DOMAIN_ID=171 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=warehouse_demo_interactive \
  -v "$PWD:/workspace:Z" localhost/var-e9t-warehouse:humble bash
```

Inside the container, from `/workspace`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to warehouse_control
source install/setup.bash
ros2 launch warehouse_control warehouse_demo.launch.py headless:=true rviz:=false
```

The default ordered requests are `r1c0`, `r1c0` and `r0c0` (FIFO). The duplicate
at `r1c0` gives time to pause the **second** simulated pickup service. The
launch's `strategy` accepts `fifo`, `nn` and `nn-2opt`; `pickup_nodes` accepts a
JSON string array, for example:

```bash
ros2 launch warehouse_control warehouse_demo.launch.py \
  headless:=true rviz:=false strategy:=fifo \
  'pickup_nodes:=["r1c0","r1c0","r0c0"]'
```

The launch starts the simulation's own four directional bridges, one estimator,
one controller and (if requested) one RViz instance. The controller subscribes
to `/warehouse/imu_odom`, not raw wheel odometry. Wait for estimator `ACTIVE` and
controller feedback before interpreting the mission as running. Default
`startup_timeout:=30.0` is a *wall-time* estimator deadline; if it faults,
restart in a fresh simulation. Only one controller may publish
`/warehouse/cmd_vel`.

## Pause, manually close an aisle and resume

Open a **second terminal in the same container** using
`podman exec -it warehouse-demo-interactive bash`, then source:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 service list | grep /warehouse/
ros2 topic info /warehouse/cmd_vel -v
ros2 topic echo /warehouse/mission_plan_markers --once \
  --qos-reliability reliable --qos-durability transient_local
```

Watch the first terminal. After the line `Pickup request 1 r1c0 simulated
service completed`, wait for `Pickup request 2 r1c0 arrived; simulated service
started`. **Immediately** call pause from the second terminal, before the
one-second second pickup completes:

```bash
ros2 service call /warehouse/pause_mission std_srvs/srv/Trigger '{}'
```

The response must have `success: true` and `Pause request accepted.` An accepted
pause is not itself proof of physical rest. The controller commands zero; allow
braking and at least 0.5 **wall and simulation** seconds of *fresh continuous*
stationary odometry. Verify it remains paused and the robot has not moved from
`r1c0` (world `-4,-1`, approximately odom `0,2`). The resume service rejects
an incorrect node or a point between graph nodes; do not change the node merely
to make the service succeed.

Manually declare that the graph edge `r0c0`–`r1c0` is closed:

```bash
ros2 service call /warehouse/resume_mission_with_closures \
  warehouse_interfaces/srv/ResumeMissionWithClosures \
  '{start_node: r1c0, closed_edges: [{node_a: r0c0, node_b: r1c0}]}'
```

The `closed_edges` array replaces the complete logical closure set. Successful
response: `Resumed from r1c0; 1 closed edge(s).` The expected detour is:

```text
r1c0 -> r1c1 -> r1c2 -> r1c3 -> r1c4
     -> r0c4 -> r0c3 -> r0c2 -> r0c1 -> r0c0
```

The controller finishes original pickup requests **2 and 3**, then reaches
depot `r0c0`, enters `REACHED`, and publishes a final zero velocity command.
Check the controller log for `Accepted resume plan:`, `Pickup request 2`,
`Pickup request 3`, `Final depot r0c0` and `REACHED:`. The detour must **not**
contain edge `r0c0`–`r1c0`. The blue/green lines are plans, not measured paths;
red markers are manually accepted closures, not sensor detections.

If pause arrives after the second service completed, or after the robot left
`r1c0`, **restart a fresh simulation**; do not claim that the example succeeded.
A `strictly newer` response may happen when the service races a paused control
tick: retry only after another fresh odometry sample. For any other rejection,
read the actual error and correct the conditions, not the robot position.

## Optional desktop visualization

For Fedora KDE/Xwayland, **stop the headless container first**. In a desktop
terminal at the repository root, verify `DISPLAY` and `XAUTHORITY` are set,
then launch a dedicated GUI-capable container (do not attach to another
project's ROS domain or Ignition partition):

```bash
podman run --rm -it --name warehouse-demo-gui \
  --network private --ipc private --security-opt label=disable \
  --memory 4g --cpus 4 \
  -v "$PWD:/workspace" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${XAUTHORITY:?Set XAUTHORITY}:/tmp/warehouse.xauth:ro" \
  -e DISPLAY -e XAUTHORITY=/tmp/warehouse.xauth \
  -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e ROS_DOMAIN_ID=171 -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=warehouse_demo_gui \
  localhost/var-e9t-warehouse:humble bash
```

Build and source the four packages as above, then run:

```bash
ros2 launch warehouse_control warehouse_demo.launch.py headless:=false rviz:=true
```

Use `podman exec -it warehouse-demo-gui bash` for the second shell instead of
`warehouse-demo-interactive`. The 4 GiB / 4 CPU limits and software rendering
were exercised on Fedora; actual responsiveness depends on host load. Gazebo
shows the physical robot and shelves, whereas RViz shows the **logical plans**
in `odom`, not a live robot model. RViz uses `mission_plans.rviz` with
Reliable/Transient Local MarkerArray QoS. A `No tf data` global warning is
expected without the missing full TF tree, even when same-frame markers draw.
The red closure is only a manual planning decision, never a Gazebo obstacle.
A screenshot proves only the displayed scene at capture time, not physical
trajectory accuracy or a complete successful mission.

## Reproducible acceptance and shutdown

Run the existing-source-based validator **without another launch running**, in
an isolated container with ROS packages built and sourced. Its own launch owns
each fresh simulation and its IMU-backed controller. Choose a previously
nonexistent output directory:

```bash
python3 scripts/validate_warehouse_demo.py --output /tmp/warehouse-demo-check-01
```

It exercises an accepted detour and a rejected unreachable closure set in
separate fresh worlds. Do not run colcon/ROS service tests or a second
controller concurrently in the same ROS domain: their service calls can reach
the demonstration controller and invalidate its observations.

Gates include both services, pause rest, IMU feedback, exact replan edge and
waypoint checks, original completion IDs, depot return,
independent Gazebo world pose and final five-second sampled rest, single command
publisher, and launch termination. The unreachable case must remain PAUSED
without accepting a new plan or moving after rejection. Review
`results.json`, per-case `controller.log`, and physical/odometry/command data.
The validator may take minutes of simulation time; outcomes are not guaranteed.

Stop the interactive launch with **Ctrl+C in the first terminal**. Wait for
Gazebo, bridges, estimator and controller to exit. The DiffDrive plugin holds
its last velocity command unless explicitly stopped; the controller's normal
stop path must therefore be verified rather than inferred from a GUI close.
Do not stop an unrelated process or modify any other running container.

This is an isolated simulated demonstration, not real-hardware or continuous
collision-safety acceptance. The IMU orientation is ideal, pose covariance is
unavailable, world-pose observations do not feed navigation, the logical closure
never adds a Gazebo obstacle, and no full dynamic TF is published.
