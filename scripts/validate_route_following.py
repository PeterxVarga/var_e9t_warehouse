#!/usr/bin/env python3
"""Opt-in installed Humble/Fortress checks; run only in an isolated container.

Every case owns a fresh Gazebo server and directional bridges. World poses
come directly from Ignition transport, with Gazebo simulation timestamps.
Artifacts include raw poses, ROS feedback/commands, logs and measured gates.
This is sampled static geometry validation, not a continuous safety proof.
"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import re
import signal
import subprocess
import termios
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

from warehouse_control.route import prepare_route
from warehouse_planning.roadmap import load_obstacles, segment_intersects

CASES = ('r3c2', 'r2c2', 'odom_loss', 'ctrl_c', 'sigterm',
         'invalid_goal', 'unreachable', 'invalid_start', 'moved_start', 'single_goal')
RADIUS = 0.3202
ROUTE_CASES = ('r3c2', 'r2c2', 'r3c4')


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Trial:
    def __init__(self, case, output, wall_timeout, controller_parameters=(), model_dir=None):
        self.case, self.wall_timeout = case, wall_timeout
        self.controller_parameters = controller_parameters
        self.directory = output / case
        self.directory.mkdir(parents=True, exist_ok=True)
        self.processes, self.handles = [], []
        self.physical, self.odom, self.commands = [], [], []
        self.sim_time = 0.0
        self.begin = time.monotonic()
        self.deadline = self.begin + wall_timeout
        self.share = Path(get_package_share_directory('warehouse_sim'))
        self.graph = (Path(get_package_share_directory('warehouse_planning'))
                      / 'config/warehouse_graph.json')
        self.world = self.share / 'worlds/warehouse.sdf'
        self.env = dict(os.environ, IGN_GAZEBO_RESOURCE_PATH=str(model_dir or self.share / 'models'))
        self.node = rclpy.create_node('route_validation_observer')
        self.node.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.node.create_subscription(Odometry, '/warehouse/odom', self.on_odom,
                                      qos_profile_sensor_data)
        self.node.create_subscription(Twist, '/warehouse/cmd_vel', self.on_command, 100)
        self.bad_odom = self.node.create_publisher(Odometry, '/validation/bad_odom', 1)
        self.master = None
        self.result = {}

    def on_clock(self, msg):
        self.sim_time = stamp_seconds(msg.clock)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((stamp_seconds(msg.header.stamp), p.x, p.y,
                          yaw(msg.pose.pose.orientation),
                          msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    def on_command(self, msg):
        self.commands.append((self.sim_time, time.monotonic() - self.begin,
                              msg.linear.x, msg.angular.z))

    def start(self, name, args, env=None, terminal=False, pipe=False):
        handle = open(self.directory / (name + '.log'), 'w')
        self.handles.append(handle)
        options = dict(cwd='/tmp', env=env or self.env, stderr=handle,
                       stdout=subprocess.PIPE if pipe else handle,
                       text=True, start_new_session=True)
        if terminal:
            self.master, slave = pty.openpty()

            def controlling_terminal():
                fcntl.ioctl(slave, termios.TIOCSCTTY, 0)

            options.update(stdin=slave, preexec_fn=controlling_terminal)
        proc = subprocess.Popen(args, **options)
        if terminal:
            os.close(slave)
        self.processes.append(proc)
        return proc

    def stop(self, proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)

    def spin(self):
        if time.monotonic() > self.deadline:
            raise TimeoutError(f'{self.wall_timeout}s wall-time deadline exceeded')
        rclpy.spin_once(self.node, timeout_sec=0.02)
        if self.case == 'invalid_start' and self.sim_time:
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = 'odom', 'warehouse_robot/base_link'
            msg.header.stamp.sec = int(self.sim_time)
            msg.header.stamp.nanosec = int((self.sim_time % 1) * 1e9)
            msg.pose.pose.position.x = 0.06
            msg.pose.pose.orientation.w = 1.0
            self.bad_odom.publish(msg)

    def until(self, condition, timeout=20):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                raise TimeoutError('condition did not arrive: inspect case logs')
            self.spin()

    def text(self):
        return (self.directory / 'controller.log').read_text()

    def poses(self, proc):
        with open(self.directory / 'physical.jsonl', 'w') as output:
            for line in proc.stdout:
                if not line.strip():
                    continue
                try:
                    data = json.loads(line)
                    stamp = data['header']['stamp']
                    t = int(stamp.get('sec', 0)) + int(stamp.get('nsec', 0)) * 1e-9
                    pose = next(p for p in data['pose'] if p.get('name') == 'warehouse_robot')
                    p, q = pose['position'], pose['orientation']
                    x, y = p.get('x', 0.0), p.get('y', 0.0)
                    angle = math.atan2(2 * (q.get('w', 0) * q.get('z', 0)
                                           + q.get('x', 0) * q.get('y', 0)),
                                       1 - 2 * (q.get('y', 0)**2 + q.get('z', 0)**2))
                    sample = (t, x, y, angle)
                    self.physical.append(sample)
                    output.write(json.dumps(sample) + '\n')
                except (ValueError, KeyError, StopIteration) as exc:
                    output.write(json.dumps({'parse_error': str(exc), 'raw': line}) + '\n')

    def bridge(self, name, ros_type, gz_type, direction):
        return self.start(name, ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                f'/warehouse/{name}@{ros_type}{direction}{gz_type}'])

    def start_simulation(self):
        """Start an owned world and observe it without providing navigation data."""
        self.start('gazebo', ['ign', 'gazebo', '-r', '-s', '-v', '3', str(self.world)])
        command_bridge = self.bridge('cmd_vel', 'geometry_msgs/msg/Twist', 'ignition.msgs.Twist', ']')
        odom_bridge = self.bridge('odom', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry', '[')
        self.start('clock', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                            '/world/warehouse/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                            '--ros-args', '-r', '/world/warehouse/clock:=/clock'])
        self.until(lambda: self.odom and self.sim_time > 0)
        pose_process = self.start('pose_transport', ['ign', 'topic', '-t',
                                 '/world/warehouse/dynamic_pose/info', '-e', '--json-output'], pipe=True)
        reader = threading.Thread(target=self.poses, args=(pose_process,), daemon=True)
        reader.start()
        self.until(lambda: len(self.physical) >= 10)
        assert distance(self.physical[-1][1:3], (-4, -3)) < 0.01
        return command_bridge, odom_bridge

    def execute(self):
        command_bridge, odom_bridge = self.start_simulation()
        if self.case == 'moved_start':
            # Set up a real invalid restart: move, brake, remove this publisher,
            # then launch the follower. Never overlap command publishers.
            manual = self.node.create_publisher(Twist, '/warehouse/cmd_vel', 1)
            self.until(lambda: self.node.count_subscribers('/warehouse/cmd_vel') >= 2)
            command = Twist()
            command.linear.x = 0.15
            while distance(self.odom[-1][1:3], (0, 0)) < 0.1:
                manual.publish(command)
                self.spin()
            command.linear.x = 0.0
            braking_end = self.sim_time + 1
            while self.sim_time < braking_end:
                manual.publish(command)
                self.spin()
            self.node.destroy_publisher(manual)
            self.until(lambda: self.node.count_publishers('/warehouse/cmd_vel') == 0)
            self.commands.clear()
        controller_start_pose = self.physical[-1][1:3]
        goal = self.case if self.case in ROUTE_CASES else 'r3c2'
        route, _, planned_length = prepare_route(self.graph, self.world, goal)
        env = dict(self.env)
        if self.case == 'unreachable':
            overlay = self.directory / 'disconnected_overlay'
            marker = overlay / 'share/ament_index/resource_index/packages/warehouse_planning'
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.write_text('')
            graph = overlay / 'share/warehouse_planning/config/warehouse_graph.json'
            graph.parent.mkdir(parents=True, exist_ok=True)
            data = json.loads(self.graph.read_text())
            data['edges'] = [edge for edge in data['edges'] if data['depot'] not in edge]
            graph.write_text(json.dumps(data))
            env['AMENT_PREFIX_PATH'] = str(overlay) + ':' + env['AMENT_PREFIX_PATH']
        entry = 'goal_controller' if self.case == 'single_goal' else 'route_controller'
        args = ['ros2', 'run', 'warehouse_control', entry, '--ros-args', '-p', 'use_sim_time:=true']
        if entry == 'route_controller':
            args += ['-p', 'goal_node:=' + ('missing' if self.case == 'invalid_goal' else goal)]
        if self.case == 'invalid_start':
            args += ['-r', '/warehouse/odom:=/validation/bad_odom']
        for parameter in self.controller_parameters:
            args += ['-p', parameter]
        start_sim, start_wall = self.sim_time, time.monotonic()
        controller = self.start('controller', args, env=env, terminal=self.case == 'ctrl_c')
        event, signal_pid = None, None
        if self.case in ('invalid_goal', 'unreachable'):
            self.until(lambda: controller.poll() is not None)
            assert controller.returncode != 0
            assert 'Controller failed:' in self.text()
            expected = 'unknown' if self.case == 'invalid_goal' else 'no graph route'
            assert expected in self.text(), self.text()
            assert not self.commands
            terminal_sim = self.sim_time
        elif self.case in ('invalid_start', 'moved_start'):
            self.until(lambda: 'FAULT:' in self.text())
            terminal_sim = self.sim_time
        elif self.case in ('odom_loss', 'ctrl_c', 'sigterm'):
            self.until(lambda: self.commands and self.commands[-1][2] > 0.05
                       and distance(self.physical[-1][1:3], (-4, -3)) > 0.1)
            event = self.sim_time
            assert self.node.count_publishers('/warehouse/cmd_vel') == 1
            if self.case == 'odom_loss':
                self.stop(odom_bridge)
                self.until(lambda: 'FAULT:' in self.text())
                terminal_sim = self.sim_time
                assert command_bridge.poll() is None
                self.bridge('odom', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry', '[')
                last_odom = self.odom[-1][0]
                self.until(lambda: self.odom[-1][0] > last_odom + 0.2)
            else:
                if self.case == 'ctrl_c':
                    os.write(self.master, b'\x03')  # Real terminal Ctrl+C.
                else:
                    children = Path(f'/proc/{controller.pid}/task/{controller.pid}/children').read_text().split()
                    assert len(children) == 1, children
                    signal_pid = int(children[0])
                    os.kill(signal_pid, signal.SIGTERM)  # Node PID, not the ros2 wrapper.
                self.until(lambda: controller.poll() is not None)
                assert controller.returncode == 0
                terminal_sim = self.sim_time
        else:
            while 'REACHED:' not in self.text():
                self.spin()
                assert 'FAULT:' not in self.text(), self.text()
                assert controller.poll() is None, self.text()
                assert self.sim_time - start_sim <= 180, '180s simulation deadline exceeded'
            terminal_sim = self.sim_time
            assert self.node.count_publishers('/warehouse/cmd_vel') == 1
        # Allow one simulation second for braking, then prove five seconds of rest.
        rest_start = terminal_sim + 1
        self.until(lambda: self.physical[-1][0] >= rest_start, timeout=15)
        first_rest = next(p[0] for p in self.physical if p[0] >= rest_start)
        self.until(lambda: self.physical[-1][0] >= first_rest + 5, timeout=30)
        rest = [p for p in self.physical if p[0] >= rest_start]
        assert len(rest) >= 100
        rest_displacement = max(distance(p[1:3], rest[0][1:3]) for p in rest)
        rest_yaw = max(abs(math.atan2(math.sin(p[3]-rest[0][3]),
                                     math.cos(p[3]-rest[0][3]))) for p in rest)
        rest_speed = max(distance(a[1:3], b[1:3])/(b[0]-a[0])
                         for a, b in zip(rest, rest[1:]) if b[0] > a[0])
        rest_duration_ns = round(rest[-1][0] * 1e9) - round(rest[0][0] * 1e9)
        gaps = [b[0]-a[0] for a, b in zip(self.physical, self.physical[1:])]
        assert gaps and min(gaps) > 0 and max(gaps) < 0.1, 'missing or stale world poses'
        obstacles = load_obstacles(self.world, radius=RADIUS)
        collisions = [(a[0], b[0], obstacle.name)
                      for a, b in zip(self.physical, self.physical[1:])
                      for obstacle in obstacles
                      if segment_intersects(a[1:3], b[1:3], obstacle)]
        assert not collisions, collisions[:10]
        assert rest_displacement < 0.001 and rest_speed < 0.001 and rest_yaw < 0.001
        assert ((self.commands and self.commands[-1][2:] == (0.0, 0.0))
                or self.case in ('invalid_goal', 'unreachable'))
        # After braking, no nonzero command may appear, including after feedback recovery.
        assert all(c[2:] == (0.0, 0.0) for c in self.commands if c[0] >= rest_start)
        result = dict(case=self.case, passed=True, start_sim=start_sim,
                      terminal_sim=terminal_sim, execution_sim_seconds=terminal_sim-start_sim,
                      wall_seconds=time.monotonic()-start_wall, physical_samples=len(self.physical),
                      max_physical_gap_seconds=max(gaps), sampled_collision_count=len(collisions),
                      rest_seconds=rest_duration_ns * 1e-9, rest_displacement_m=rest_displacement,
                      rest_speed_m_s=rest_speed, rest_yaw_rad=rest_yaw,
                      final_physical_pose=self.physical[-1][1:],
                      final_odom_pose=self.odom[-1][1:4], final_command=self.commands[-1] if self.commands else None,
                      signal_sim=event, signal_node_pid=signal_pid,
                      controller_parameters=list(self.controller_parameters))
        self.result = result
        result['physical_distance_m'] = sum(distance(a[1:3], b[1:3])
                                           for a, b in zip(self.physical, self.physical[1:]))
        result['odom_distance_m'] = sum(distance(a[1:3], b[1:3])
                                       for a, b in zip(self.odom, self.odom[1:]))
        if self.case in (*ROUTE_CASES, 'single_goal'):
            odom_goal = (1, 0) if self.case == 'single_goal' else route.points[-1]
            world_goal = (-3, -3) if self.case == 'single_goal' else (odom_goal[0]-4, odom_goal[1]-3)
            result.update(odom_error_m=distance(self.odom[-1][1:3], odom_goal),
                          physical_error_m=distance(self.physical[-1][1:3], world_goal),
                          planned_length_m=1 if self.case == 'single_goal' else planned_length)
            result['gates'] = dict(odom_accuracy=result['odom_error_m'] <= 0.05,
                                   physical_accuracy=result['physical_error_m'] <= 0.15,
                                   execution_time=result['execution_sim_seconds'] <= 180,
                                   sampled_clearance=not collisions,
                                   stopped=rest_displacement < 0.001 and rest_speed < 0.001
                                   and rest_yaw < 0.001 and rest_duration_ns >= 5000000000)
            if self.case != 'single_goal':
                reached = re.findall(r'Waypoint \d+/\d+ (\w+) reached', self.text())
                result['waypoints'] = reached
                result['gates']['waypoint_order'] = reached == list(route.nodes)
            result['passed'] = all(result['gates'].values())
        else:
            result['fault_latched'] = 'FAULT:' in self.text()
            if self.case in ('invalid_start', 'moved_start', 'invalid_goal', 'unreachable'):
                assert all(c[2:] == (0.0, 0.0) for c in self.commands)
                controller_poses = [p for p in self.physical if p[0] >= start_sim]
                assert max(distance(p[1:3], controller_start_pose) for p in controller_poses) < 0.001
                result['controller_start_world_xy'] = controller_start_pose
                result['controller_motion_m'] = max(distance(p[1:3], controller_start_pose)
                                                     for p in controller_poses)
        return result

    def close(self):
        for proc in reversed(self.processes):
            self.stop(proc)
        if self.master is not None:
            os.close(self.master)
        for handle in self.handles:
            handle.close()
        for name, values in (('odom', self.odom), ('commands', self.commands)):
            with open(self.directory / (name + '.jsonl'), 'w') as output:
                for value in values:
                    output.write(json.dumps(value) + '\n')
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=(*CASES, 'r3c4'), action='append')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-timeout', type=float, default=240)
    parser.add_argument('--controller-parameter', action='append', default=[],
                        help='ROS parameter override, e.g. max_angular_speed:=0.2')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--model-dir', type=Path,
                        help='Diagnostic model resource directory; installed world is retained')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    if (args.output / 'results.json').exists():
        parser.error('choose a new output directory to preserve previous measurements')
    results = []
    rclpy.init()
    try:
        cases = [(repeat, case) for repeat in range(1, args.repeat + 1)
                 for case in args.case or CASES]
        for repeat, case in cases:
            print(f'START {case} repeat {repeat}', flush=True)
            output = args.output if args.repeat == 1 else args.output / f'repeat-{repeat}'
            trial = Trial(case, output, args.wall_timeout, args.controller_parameter, args.model_dir)
            try:
                result = trial.execute()
            except Exception as exc:
                result = dict(trial.result, case=case, passed=False, error=repr(exc))
                import traceback
                traceback.print_exc()
            finally:
                trial.close()
            result['repeat'] = repeat
            results.append(result)
            (trial.directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()
    return int(any(not result['passed'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
