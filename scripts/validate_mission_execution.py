#!/usr/bin/env python3
"""Opt-in FIFO execution gates in fresh, isolated Humble/Fortress worlds.

Reuse the route harness's process ownership, bridges, independent world-pose
reader, sampled obstacle method and physical-rest gates. Never feed these
measurements to navigation. Raw evidence belongs outside public source.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import time

import rclpy

from warehouse_control.mission import prepare_mission
from validate_route_following import Trial, distance


CASES = ('A', 'B', 'empty', 'odom_loss_travel', 'odom_loss_service', 'ctrl_c', 'sigterm')
PICKUPS = dict(A=('r2c2', 'r1c4'), B=('r2c2', 'r1c0', 'r1c0'), empty=())
TIMING = r'; feedback_stamp=([\d.]+) s; clock_stamp=([\d.]+) s'


def nearest(samples, stamp):
    sample = min(samples, key=lambda p: abs(p[0] - stamp))
    assert abs(sample[0] - stamp) <= 0.06, 'no time-aligned observation'
    return sample


def zero_streak(commands, start, end):
    """Bound asynchronous observer-clock alignment by one 50 ms timer period."""
    interior = [i for i, c in enumerate(commands) if start + 0.05 <= c[0] <= end]
    if not interior or any(commands[i][2:] != (0.0, 0.0) for i in interior):
        return False, 0.0
    left, right = interior[0], interior[-1]
    while left > 0 and commands[left - 1][2:] == (0.0, 0.0):
        left -= 1
    while right + 1 < len(commands) and commands[right + 1][2:] == (0.0, 0.0):
        right += 1
    duration = commands[right][0] - commands[left][0]
    covered = commands[left][0] <= start + 0.05 and commands[right][0] >= end - 0.05
    return covered and round(duration * 1e9) >= 1000000000, duration


class MissionTrial(Trial):
    def spin(self):
        super().spin()
        publishers = self.node.count_publishers('/warehouse/cmd_vel')
        self.max_publishers = max(getattr(self, 'max_publishers', 0), publishers)
        assert publishers <= 1, 'more than one velocity-command publisher'

    def execute(self):
        command_bridge, odom_bridge = self.start_simulation()
        pickups = PICKUPS.get(self.case, ('r1c0',))
        mission, transform = prepare_mission(self.graph, self.world, pickups)
        args = ['ros2', 'run', 'warehouse_control', 'mission_controller', '--ros-args',
                '-p', 'use_sim_time:=true', '-p', 'pickup_nodes:=' + json.dumps(pickups)]
        start_sim, start_wall = self.sim_time, time.monotonic()
        controller = self.start('controller', args, terminal=self.case == 'ctrl_c')
        event, signal_pid = None, None
        if self.case in ('A', 'B', 'empty'):
            while 'REACHED:' not in self.text():
                self.spin()
                assert 'FAULT:' not in self.text(), self.text()
                assert controller.poll() is None, self.text()
                assert self.sim_time - start_sim <= 360, '360s simulation deadline exceeded'
            terminal_sim = self.sim_time
        else:
            if self.case == 'odom_loss_service':
                self.until(lambda: 'simulated service started' in self.text(), timeout=60)
            else:
                self.until(lambda: self.commands and self.commands[-1][2] > 0.05
                           and distance(self.physical[-1][1:3], (-4, -3)) > 0.1)
            assert self.node.count_publishers('/warehouse/cmd_vel') == 1
            event = self.sim_time
            if self.case.startswith('odom_loss'):
                self.stop(odom_bridge)
                self.until(lambda: 'FAULT:' in self.text())
                terminal_sim = self.sim_time
                assert command_bridge.poll() is None
                last_odom = self.odom[-1][0]
                self.bridge('odom', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry', '[')
                self.until(lambda: self.odom[-1][0] > last_odom + 0.2)
                assert 'simulated service completed' not in self.text()
                assert 'REACHED:' not in self.text()
            else:
                if self.case == 'ctrl_c':
                    os.write(self.master, b'\x03')
                else:
                    children = Path(f'/proc/{controller.pid}/task/{controller.pid}/children').read_text().split()
                    assert len(children) == 1, children
                    signal_pid = int(children[0])
                    os.kill(signal_pid, signal.SIGTERM)
                self.until(lambda: controller.poll() is not None)
                assert controller.returncode == 0
                terminal_sim = self.sim_time
        if controller.poll() is None:
            assert self.node.count_publishers('/warehouse/cmd_vel') == 1
        result = dict(case=self.case, start_sim=start_sim, terminal_sim=terminal_sim,
                      execution_sim_seconds=terminal_sim - start_sim,
                      wall_timeout_seconds=self.wall_timeout,
                      planned_length_m=mission.plan.total_distance_m,
                      signal_sim=event, signal_node_pid=signal_pid)
        self.result = result
        result.update(self.verify_stop(terminal_sim))
        result['max_velocity_publishers'] = self.max_publishers
        result['wall_seconds'] = time.monotonic() - start_wall
        result['physical_distance_m'] = sum(distance(a[1:3], b[1:3])
                                           for a, b in zip(self.physical, self.physical[1:]))
        if self.case not in ('A', 'B', 'empty'):
            result['fault_latched'] = 'FAULT:' in self.text()
            result['feedback_recovered_without_restart'] = self.case.startswith('odom_loss')
            result['passed'] = True
            return result
        text = self.text()
        waypoints = [(int(leg), node) for leg, node in re.findall(
            r'Mission waypoint leg (\d+) \d+/\d+ (\w+) reached', text)]
        expected = [(i, node) for i, leg in enumerate(mission.plan.legs, 1) for node in leg.nodes]
        arrivals = re.findall(r'Pickup request (\d+) (\w+) arrived; simulated service started' + TIMING, text)
        completed = re.findall(r'Pickup request (\d+) (\w+) simulated service completed' + TIMING, text)
        request_order = [(int(i), node) for i, node, _, _ in arrivals]
        completion_order = [(int(i), node) for i, node, _, _ in completed]
        expected_requests = list(enumerate(pickups, 1))
        services = []
        for arrival, completion in zip(arrivals, completed):
            i, node, feedback_start, clock_start = arrival
            _, _, feedback_end, clock_end = completion
            fs, fe, cs, ce = map(float, (feedback_start, feedback_end, clock_start, clock_end))
            restarts = re.findall(r'Pickup request ' + i + ' ' + node + r' service_restart' + TIMING, text)
            restarts = [(float(f), float(c)) for f, c in restarts if cs <= float(c) <= ce]
            if restarts:
                fs, cs = restarts[-1]
            world_target = transform.odom_to_world(mission.points[int(i) - 1][-1])
            odom_target = mission.points[int(i) - 1][-1]
            odom = [p for p in self.odom if fs <= p[0] <= fe]
            physical = [p for p in self.physical if fs <= p[0] <= fe]
            assert odom and physical
            # Include the independently aligned arrival and completion samples.
            physical += [nearest(self.physical, fs), nearest(self.physical, fe)]
            odom += [nearest(self.odom, fs), nearest(self.odom, fe)]
            commands = [c for c in self.commands if cs + 0.05 <= c[0] <= ce]
            zero_commands, zero_seconds = zero_streak(self.commands, cs, ce)
            services.append(dict(request=int(i), node=node, feedback_start=fs, feedback_end=fe,
                                 clock_start=cs, clock_end=ce, feedback_seconds=fe-fs,
                                 clock_seconds=ce-cs, command_samples=len(commands),
                                 zero_commands=zero_commands, zero_streak_seconds=zero_seconds,
                                 max_odom_error_m=max(distance(p[1:3], odom_target) for p in odom),
                                 max_physical_error_m=max(distance(p[1:3], world_target) for p in physical)))
        depot = re.findall(r'Final depot (\w+) reached in odom' + TIMING, text)
        assert len(depot) == 1
        node, feedback, clock = depot[0]
        result.update(waypoints=waypoints, expected_waypoints=expected,
                      request_order=request_order, completion_order=completion_order,
                      services=services, depot_feedback_stamp=float(feedback),
                      depot_clock_stamp=float(clock),
                      odom_error_m=distance(self.odom[-1][1:3], (0, 0)),
                      physical_error_m=distance(self.physical[-1][1:3], transform.odom_to_world((0, 0))))
        result['gates'] = dict(
            waypoint_order=waypoints == expected,
            request_order=request_order == completion_order == expected_requests,
            separate_service=len(services) == len(pickups) and all(
                round(s['feedback_seconds'] * 1e9) >= 1000000000
                and round(s['clock_seconds'] * 1e9) >= 1000000000
                and s['zero_commands'] for s in services),
            odom_accuracy=result['odom_error_m'] <= 0.05
            and all(s['max_odom_error_m'] <= 0.05 for s in services),
            physical_accuracy=result['physical_error_m'] <= 0.15
            and all(s['max_physical_error_m'] <= 0.15 for s in services),
            execution_time=result['execution_sim_seconds'] <= 360,
            sampled_clearance=result['sampled_collision_count'] == 0,
            stopped=result['stopped'],
            empty_stationary=bool(pickups) or all(c[2:] == (0.0, 0.0) for c in self.commands))
        result['passed'] = all(result['gates'].values())
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, action='append')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-timeout', type=float, default=600)
    parser.add_argument('--repeat', type=int, default=1)
    args = parser.parse_args()
    if args.repeat < 1 or not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0:
        parser.error('repeat and finite wall-timeout must be positive')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'results.json').exists():
        parser.error('choose a new output directory to preserve previous measurements')
    results = []
    rclpy.init()
    try:
        for repeat in range(1, args.repeat + 1):
            for case in args.case or CASES:
                print(f'START {case} repeat {repeat}', flush=True)
                output = args.output / f'repeat-{repeat}'
                trial = MissionTrial(case, output, args.wall_timeout)
                try:
                    result = trial.execute()
                except Exception as exc:
                    import traceback
                    traceback.print_exc()
                    result = dict(trial.result, case=case, passed=False, error=repr(exc))
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
