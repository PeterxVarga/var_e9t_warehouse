#!/usr/bin/env python3
"""Measure accepted mission pauses in isolated installed Gazebo worlds.

World poses are observations only; they are never navigation input. This script
uses sampled physical motion and cannot prove continuous rest or hardware safety.
"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger

from validate_route_following import Trial, distance


class PauseTrial(Trial):
    def execute(self):
        self.start_simulation()
        self.start('controller', [
            'ros2', 'run', 'warehouse_control', 'mission_controller', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'pickup_nodes:=["r1c0"]'])
        client = self.node.create_client(Trigger, '/warehouse/pause_mission')
        self.until(lambda: client.service_is_ready(), timeout=30)
        if self.case == 'travel':
            self.until(lambda: self.commands and self.commands[-1][2] > 0.05
                       and distance(self.physical[-1][1:3], (-4, -3)) > 0.1, timeout=60)
        else:
            self.until(lambda: 'Pickup request 1 r1c0 arrived; simulated service started'
                       in self.text(), timeout=90)
            assert 'simulated service completed' not in self.text()
        assert self.node.count_publishers('/warehouse/cmd_vel') == 1
        command_index = len(self.commands)
        request_sim, request_wall = self.sim_time, time.monotonic() - self.begin
        future = client.call_async(Trigger.Request())
        self.until(lambda: future.done(), timeout=2)
        response_sim, response_wall = self.sim_time, time.monotonic() - self.begin
        response = future.result()
        assert response is not None and response.success, str(response)
        self.until(lambda: any(c[2:] == (0.0, 0.0) for c in self.commands[command_index:]),
                   timeout=2)
        zero_index = next(i for i in range(command_index, len(self.commands))
                          if self.commands[i][2:] == (0.0, 0.0))
        zero = self.commands[zero_index]
        assert response_wall - request_wall <= 2 and zero[1] - request_wall <= 2
        result = dict(case=self.case, request_sim=request_sim, request_wall=request_wall,
                      response_sim=response_sim, response_wall=response_wall,
                      response_message=response.message, zero_sim=zero[0], zero_wall=zero[1],
                      response_latency_wall=response_wall-request_wall,
                      zero_latency_wall=zero[1]-request_wall)
        self.result = result
        # Fixed one-simulation-second braking allowance plus five seconds rest.
        result.update(self.verify_stop(request_sim))
        assert all(c[2:] == (0.0, 0.0) for c in self.commands[zero_index:])
        assert 'FAULT:' not in self.text() and 'REACHED:' not in self.text()
        if self.case == 'service':
            assert 'Pickup request 1 r1c0 simulated service completed' not in self.text()
        before = min(self.physical, key=lambda p: abs(p[0] - request_sim))
        assert abs(before[0] - request_sim) <= 0.06
        end = next(p for p in self.physical if p[0] >= request_sim + 1)
        braking = [p for p in self.physical if before[0] <= p[0] <= end[0]]
        # Find the earliest sampled suffix that stays within the rest limits.
        # This is an observation bound, not the exact instant of physical rest.
        observed = [p for p in self.physical if before[0] <= p[0]]
        final = observed[-1]
        settled = len(observed) - 1
        for i in range(len(observed) - 2, -1, -1):
            a, b = observed[i:i + 2]
            angle = abs(math.atan2(math.sin(a[3] - final[3]), math.cos(a[3] - final[3])))
            if (distance(a[1:3], final[1:3]) >= 0.001 or angle >= 0.001
                    or distance(a[1:3], b[1:3]) / (b[0] - a[0]) >= 0.001):
                break
            settled = i
        assert observed[settled][0] <= end[0]
        to_settle = observed[:settled + 1]
        result.update(braking_start_sample_sim=before[0], braking_end_sample_sim=end[0],
                      braking_path_m=sum(distance(a[1:3], b[1:3])
                                         for a, b in zip(braking, braking[1:])),
                      settled_sample_sim=observed[settled][0],
                      path_to_settled_sample_m=sum(distance(a[1:3], b[1:3])
                                                   for a, b in zip(to_settle, to_settle[1:])),
                      post_zero_command_samples=len(self.commands)-zero_index,
                      passed=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'results.json').exists():
        parser.error('choose a new output directory')
    results = []
    rclpy.init()
    try:
        for case in ('travel', 'service'):
            print(f'START {case}', flush=True)
            trial = PauseTrial(case, args.output, 180)
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
