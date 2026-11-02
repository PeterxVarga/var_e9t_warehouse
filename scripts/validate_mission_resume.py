#!/usr/bin/env python3
"""Fresh Gazebo resume and feedback-loss trials; world poses are observations only."""

import argparse
import json
import math
import re
import time
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger
from warehouse_interfaces.srv import ResumeMission

from validate_route_following import Trial, distance
from validate_mission_execution import nearest


class ResumeTrial(Trial):
    def call(self, client, request):
        future = client.call_async(request)
        self.until(future.done, timeout=3)
        response = future.result()
        assert response is not None
        return response

    def execute(self):
        _, odom_bridge = self.start_simulation()
        self.start('controller', [
            'ros2', 'run', 'warehouse_control', 'mission_controller', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'pickup_nodes:=["r1c0","r1c0","r0c0"]'])
        pause = self.node.create_client(Trigger, '/warehouse/pause_mission')
        resume = self.node.create_client(ResumeMission, '/warehouse/resume_mission')
        self.until(lambda: pause.service_is_ready() and resume.service_is_ready(), timeout=30)
        self.until(lambda: 'Pickup request 2 r1c0 arrived; simulated service started' in self.text(), timeout=90)
        assert 'Pickup request 1 r1c0 simulated service completed' in self.text()
        assert 'Pickup request 2 r1c0 simulated service completed' not in self.text()
        pause_sim, pause_wall = self.sim_time, time.monotonic() - self.begin
        assert self.call(pause, Trigger.Request()).success
        # Allow braking, then collect at least one second of independently sampled rest.
        self.until(lambda: self.sim_time >= pause_sim + 2.0, timeout=10)
        samples = [p for p in self.physical if pause_sim + 1 <= p[0] <= self.sim_time]
        assert len(samples) >= 10
        gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        assert all(0 < gap < 0.1 for gap in gaps)
        rest_displacement = max(distance(p[1:3], samples[0][1:3]) for p in samples)
        rest_speed = max(distance(a[1:3], b[1:3]) / (b[0] - a[0])
                         for a, b in zip(samples, samples[1:]) if b[0] > a[0])
        rest_yaw = max(abs(math.atan2(math.sin(p[3] - samples[0][3]),
                                      math.cos(p[3] - samples[0][3]))) for p in samples)
        assert rest_displacement < 0.001 and rest_speed < 0.001 and rest_yaw < 0.001
        before = len(self.commands)
        invalid = ResumeMission.Request()
        invalid.start_node = 'invalid-node'
        response = self.call(resume, invalid)
        assert not response.success and 'unknown graph node' in response.message
        self.until(lambda: self.sim_time >= pause_sim + 2.3)
        assert all(c[2:] == (0.0, 0.0) for c in self.commands[before:])
        result = dict(case=self.case, pause_sim=pause_sim, pause_wall=pause_wall,
                      rest_start=samples[0][0], rest_end=samples[-1][0],
                      rest_samples=len(samples), rest_max_displacement_m=rest_displacement,
                      rest_max_speed_m_s=rest_speed, rest_max_yaw_rad=rest_yaw,
                      rest_max_sample_gap_s=max(gaps), invalid_response=response.message)
        self.result = result
        req = ResumeMission.Request()
        req.start_node = 'r1c0'
        if self.case == 'stale':
            self.stop(odom_bridge)
            # Wait for the strict 0.2 s observation freshness gate, but not the
            # existing 1 s controller fault timer. Do not depend on sim clock.
            start = time.monotonic()
            while time.monotonic() - start < 0.4:
                self.spin()
            before = len(self.commands)
            response = self.call(resume, req)
            assert not response.success and 'stale' in response.message
            self.until(lambda: 'FAULT:' in self.text(), timeout=3)
            self.until(lambda: self.sim_time >= pause_sim + 3.5, timeout=5)
            assert all(c[2:] == (0.0, 0.0) for c in self.commands[before:])
            assert 'Pickup request 2 r1c0 simulated service completed' not in self.text()
            result.update(stale_response=response.message, fault_latched=True, passed=True)
            return result
        # A service call can race the paused timer for the most recent sample;
        # retry rejected calls with continuing feedback instead of using old data.
        attempts = []
        end = time.monotonic() + 5
        while time.monotonic() < end:
            response = self.call(resume, req)
            attempts.append(response.message)
            if response.success:
                break
            self.spin()
        assert response.success, attempts
        result.update(resume_sim=self.sim_time, resume_wall=time.monotonic() - self.begin,
                      resume_attempts=attempts)
        self.until(lambda: 'REACHED:' in self.text(), timeout=120)
        assert 'FAULT:' not in self.text(), self.text()
        timing = r'; feedback_stamp=([\d.]+) s; clock_stamp=([\d.]+) s'
        arrivals = re.findall(r'Pickup request (\d+) (\w+) arrived; simulated service started' + timing, self.text())
        completions = re.findall(r'Pickup request (\d+) (\w+) simulated service completed' + timing, self.text())
        assert [(int(i), n) for i, n, _, _ in completions] == [(1, 'r1c0'), (2, 'r1c0'), (3, 'r0c0')]
        second = [a for a in arrivals if a[0] == '2']
        assert len(second) == 2
        second_complete = next(c for c in completions if c[0] == '2')
        assert float(second_complete[2]) - float(second[-1][2]) >= 1
        assert float(second_complete[3]) - float(second[-1][3]) >= 1
        errors = []
        for i, node, stamp, _ in completions:
            target = (-4, -1) if node == 'r1c0' else (-4, -3)
            pose = nearest(self.physical, float(stamp))
            errors.append(dict(request=int(i), node=node, feedback=float(stamp),
                               physical_error_m=distance(pose[1:3], target)))
        depot = re.search(r'Final depot (\w+) reached in odom' + timing, self.text())
        assert depot and depot.group(1) == 'r0c0'
        depot_pose = nearest(self.physical, float(depot.group(2)))
        result.update(arrivals=arrivals, completions=completions, completion_errors=errors,
                      interrupted_service_seconds=float(second_complete[2])-float(second[-1][2]),
                      depot_feedback=float(depot.group(2)),
                      depot_physical_error_m=distance(depot_pose[1:3], (-4, -3)), passed=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    rclpy.init()
    try:
        for case in ('resume', 'stale'):
            print('START', case, flush=True)
            trial = ResumeTrial(case, args.output, 240)
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
