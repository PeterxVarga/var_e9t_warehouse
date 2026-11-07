#!/usr/bin/env python3
"""Fresh isolated Gazebo detour and unreachable-closure evidence, with ROS markers.

Use a new output directory. World poses only observe execution; they never
provide controller feedback or choose the resume node. The trial pauses the
known second r1c0 service and explicitly supplies that graph node.
"""

import argparse
import json
import math
import re
import time
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from warehouse_interfaces.msg import ClosedEdge
from warehouse_interfaces.srv import ResumeMissionWithClosures

from warehouse_control.mission import prepare_mission
from warehouse_planning.mission import plan_remaining_mission
from warehouse_planning.roadmap import graph_without_edges
from validate_mission_execution import nearest, zero_streak
from validate_mission_resume import ResumeTrial
from validate_route_following import distance


class ClosuresTrial(ResumeTrial):
    def __init__(self, *args):
        super().__init__(*args)
        self.markers = []
        self.marker_stream = open(self.directory / 'markers.jsonl', 'w')
        self.node.create_subscription(MarkerArray, '/warehouse/mission_plan_markers',
                                      self.on_markers, QoSProfile(
                                          depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_markers(self, msg):
        self.markers.append(msg)
        self.marker_stream.write(json.dumps(message_to_ordereddict(msg)) + '\n')
        self.marker_stream.flush()

    def close(self):
        super().close()
        self.marker_stream.close()

    def execute(self):
        self.start_simulation()
        mission, transform = prepare_mission(
            self.graph, self.world, ('r1c0', 'r1c0', 'r0c0'))
        self.start('controller', [
            'ros2', 'run', 'warehouse_control', 'mission_controller', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'pickup_nodes:=["r1c0","r1c0","r0c0"]'])
        pause = self.node.create_client(Trigger, '/warehouse/pause_mission')
        resume = self.node.create_client(ResumeMissionWithClosures,
                                         '/warehouse/resume_mission_with_closures')
        self.until(lambda: pause.service_is_ready() and resume.service_is_ready(), timeout=30)
        self.until(lambda: 'Pickup request 2 r1c0 arrived' in self.text(), timeout=90)
        assert 'Pickup request 1 r1c0 simulated service completed' in self.text()
        result = dict(case=self.case, pause_sim=self.sim_time, responses=[])
        self.result = result
        assert self.call(pause, Trigger.Request()).success
        self.until(lambda: self.sim_time >= result['pause_sim'] + 2, timeout=10)
        rest = [p for p in self.physical if result['pause_sim'] + 1 <= p[0] <= self.sim_time]
        assert len(rest) >= 10
        gaps = [b[0] - a[0] for a, b in zip(rest, rest[1:])]
        assert all(0 < gap < 0.1 for gap in gaps)
        displacement = max(distance(p[1:3], rest[0][1:3]) for p in rest)
        speed = max(distance(a[1:3], b[1:3]) / (b[0] - a[0])
                    for a, b in zip(rest, rest[1:]))
        angle = max(abs(math.atan2(math.sin(p[3] - rest[0][3]),
                                   math.cos(p[3] - rest[0][3]))) for p in rest)
        assert displacement < 0.001 and speed < 0.001 and angle < 0.001
        result['pause_rest'] = dict(samples=len(rest), duration_s=rest[-1][0] - rest[0][0],
                                    max_gap_s=max(gaps), displacement_m=displacement,
                                    speed_m_s=speed, yaw_rad=angle)
        self.until(lambda: len(self.markers) >= 3)
        old_marker = message_to_ordereddict(self.markers[-1])
        marker_count = len(self.markers)
        command_count = len(self.commands)
        edges = [('r0c0', 'r1c0')]
        if self.case == 'unreachable':
            edges.append(('r0c0', 'r0c1'))
        req = ResumeMissionWithClosures.Request(
            start_node='r1c0', closed_edges=[ClosedEdge(node_a=a, node_b=b) for a, b in edges])
        result['request'] = message_to_ordereddict(req)
        end = time.monotonic() + 5
        while True:
            response = self.call(resume, req)
            result['responses'].append(dict(success=response.success, message=response.message,
                                             observer_sim=self.sim_time))
            if response.success or 'unreachable' in response.message:
                break
            # Only the documented timer/sample race may be retried.
            assert 'strictly newer' in response.message, response.message
            assert time.monotonic() < end, result['responses']
            self.spin()
        if self.case == 'unreachable':
            assert not response.success and 'unreachable' in response.message
            result['stop'] = self.verify_stop(self.sim_time)
            assert len(self.markers) == marker_count
            assert message_to_ordereddict(self.markers[-1]) == old_marker
            assert all(c[2:] == (0.0, 0.0) for c in self.commands[command_count:])
            assert 'Accepted resume plan:' not in self.text()
            assert 'Pickup request 2 r1c0 simulated service completed' not in self.text()
            assert 'FAULT:' not in self.text() and 'REACHED:' not in self.text()
            result.update(passed=True, observed_state='PAUSED (log and continued zero commands)',
                          completed_request_ids=[1], depot_returned=False,
                          accepted_markers_unchanged=True)
            return result

        assert response.success, response.message
        plan = plan_remaining_mission(graph_without_edges(mission.validated_graph, edges),
                                      'r1c0', mission.original_requests, (1, 2))
        result['accepted_plan_nodes'] = plan.nodes
        result['accepted_plan_legs'] = [leg.nodes for leg in plan.legs]
        result['accepted_plan_distance_m'] = plan.total_distance_m
        self.until(lambda: len(self.markers) > marker_count)
        accepted_marker = self.markers[marker_count]
        assert accepted_marker.markers[0].action == Marker.DELETEALL
        assert all(m.header.frame_id == 'odom' for m in accepted_marker.markers)
        for namespace, nodes in [('accepted_plan', plan.nodes),
                                 ('plan_before_resume', mission.plan.nodes),
                                 ('accepted_closures', tuple(n for e in edges for n in e))]:
            line = next(m for m in accepted_marker.markers if m.ns == namespace)
            assert [(p.x, p.y) for p in line.points] == [
                transform.world_to_odom(mission.validated_graph.nodes[n]) for n in nodes]
        labels = [m.text for m in accepted_marker.markers if m.ns == 'remaining_request_labels']
        assert labels == ['Request 2: r1c0', 'Request 3: r0c0']
        self.until(lambda: 'REACHED:' in self.text(), timeout=300)
        assert 'FAULT:' not in self.text(), self.text()
        result['stop'] = self.verify_stop(self.sim_time)
        timing = r'; feedback_stamp=([\d.]+) s; clock_stamp=([\d.]+) s'
        completions = re.findall(
            r'Pickup request (\d+) (\w+) simulated service completed' + timing, self.text())
        assert [(int(i), n) for i, n, _, _ in completions] == [
            (1, 'r1c0'), (2, 'r1c0'), (3, 'r0c0')]
        arrivals = re.findall(
            r'Pickup request (\d+) (\w+) arrived; simulated service started' + timing,
            self.text())
        assert len([a for a in arrivals if a[0] == '2']) == 2
        waits = []
        for i, node, stamp, clock in completions:
            arrival = next(a for a in reversed(arrivals) if a[0] == i)
            seconds = float(stamp) - float(arrival[2])
            clock_seconds = float(clock) - float(arrival[3])
            assert round(seconds * 1e9) >= 1000000000
            assert round(clock_seconds * 1e9) >= 1000000000
            zero, zero_seconds = zero_streak(self.commands, float(arrival[3]), float(clock))
            assert zero, (i, arrival, stamp)
            physical = nearest(self.physical, float(stamp))
            waits.append(dict(request_id=int(i), node=node, feedback_seconds=seconds,
                              clock_seconds=clock_seconds, zero_command_seconds=zero_seconds,
                              physical_error_m=distance(
                                  physical[1:3], mission.validated_graph.nodes[node])))
        after = self.text().split('Accepted resume plan:', 1)[1]
        waypoints = re.findall(r'Mission waypoint leg (\d+) \d+/\d+ (\w+) reached in odom' + timing, after)
        assert [(int(leg), n) for leg, n, _, _ in waypoints] == [
            (i + 1, n) for i, leg in enumerate(plan.legs) for n in leg.nodes]
        observations = []
        for leg, node, stamp, _ in waypoints:
            physical = nearest(self.physical, float(stamp))
            odom = nearest(self.odom, float(stamp))
            target = mission.validated_graph.nodes[node]
            observations.append(dict(leg=int(leg), node=node, feedback=float(stamp),
                                     physical_world_xy=physical[1:3], odom_xy=odom[1:3],
                                     physical_error_m=distance(physical[1:3], target)))
        depot = re.search(r'Final depot (\w+) reached in odom' + timing, self.text())
        assert depot and depot.group(1) == 'r0c0'
        depot_pose = nearest(self.physical, float(depot.group(2)))
        assert not any(m.ns == 'remaining_request_labels' for m in self.markers[-1].markers)
        assert all(array.markers[0].action == Marker.DELETEALL for array in self.markers)
        result.update(passed=True, completed_request_ids=[int(c[0]) for c in completions],
                      completions=completions, service_waits=waits, observed_waypoints=observations,
                      depot_returned=True, depot_feedback=float(depot.group(2)),
                      depot_physical_error_m=distance(depot_pose[1:3], (-4, -3)),
                      marker_coordinates_and_replacements=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=('detour', 'unreachable'), action='append')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    rclpy.init()
    try:
        for case in args.case or ('detour', 'unreachable'):
            print('START', case, flush=True)
            trial = ClosuresTrial(case, args.output, 420)
            try:
                result = trial.execute()
            except Exception as exc:
                import traceback
                traceback.print_exc()
                result = dict(trial.result, case=case, passed=False, error=repr(exc))
            finally:
                trial.close()
            results.append(result)
            (trial.directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()
    return int(any(not result['passed'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
