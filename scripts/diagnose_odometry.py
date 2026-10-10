#!/usr/bin/env python3
"""Repeated open-loop primitives in fresh isolated Gazebo worlds.

Commands stop from wheel odometry thresholds only. World pose is recorded and
analyzed after commands finish; it never steers or calibrates a running robot.
"""

import argparse
import json
import math
from pathlib import Path
import traceback

import rclpy
from geometry_msgs.msg import Twist

from analyze_odometry import geometry_audit, phase_metrics, wrap
from validate_route_following import Trial, distance

CASES = {f'turn_{direction}_{rate}': [('turn', 0.0, sign * rate, math.pi / 2)]
         for direction, sign in (('positive', 1), ('negative', -1))
         for rate in (0.2, 0.4, 0.6)}
CASES.update(straight_015=[('straight', 0.15, 0.0, 2.0)],
             straight_030=[('straight', 0.30, 0.0, 2.0)],
             turn_then_straight=[('turn', 0.0, 0.6, math.pi / 2),
                                 ('straight', 0.15, 0.0, 2.0)])


class MotionTrial(Trial):
    def execute(self):
        self.start_simulation()
        model = Path(self.env['IGN_GAZEBO_RESOURCE_PATH']) / 'warehouse_robot/model.sdf'
        publisher = self.node.create_publisher(Twist, '/warehouse/cmd_vel', 1)
        phases = []
        try:
            self.until(lambda: self.node.count_subscribers('/warehouse/cmd_vel') >= 2)
            assert self.node.count_publishers('/warehouse/cmd_vel') == 1
            for kind, linear, angular, target in CASES[self.case]:
                start = self.odom[-1]
                command = Twist()
                command.linear.x, command.angular.z = linear, angular
                while True:
                    current = self.odom[-1]
                    progress = (abs(wrap(current[3] - start[3])) if kind == 'turn'
                                else distance(current[1:3], start[1:3]))
                    if progress >= target:
                        break
                    publisher.publish(command)
                    self.spin()
                publisher.publish(Twist())
                brake_end = self.sim_time + 1
                while self.sim_time < brake_end:
                    publisher.publish(Twist())
                    self.spin()
                assert abs(self.odom[-1][4]) < 0.001 and abs(self.odom[-1][5]) < 0.001
                phases.append(dict(kind=kind, linear_command=linear, angular_command=angular,
                                   odom_target=target, start_stamp=start[0], end_stamp=self.odom[-1][0]))
            rest_start = self.physical[-1][0]
            while self.physical[-1][0] < rest_start + 5:
                publisher.publish(Twist())
                self.spin()
            rest = [p for p in self.physical if p[0] >= rest_start]
            rest_motion = max(distance(p[1:3], rest[0][1:3]) for p in rest)
            rest_yaw = max(abs(wrap(p[3] - rest[0][3])) for p in rest)
            assert rest_motion < 0.001 and rest_yaw < 0.001
            gaps = [b[0] - a[0] for a, b in zip(self.physical, self.physical[1:])]
            assert gaps and min(gaps) > 0 and max(gaps) < 0.1
            for phase in phases:
                phase.update(phase_metrics(self.physical, self.odom,
                                           phase['start_stamp'], phase['end_stamp']))
            return dict(case=self.case, passed=True, phases=phases,
                        geometry=geometry_audit(model), physical_samples=len(self.physical),
                        max_physical_gap_seconds=max(gaps), rest_seconds=rest[-1][0]-rest[0][0],
                        rest_motion_m=rest_motion, rest_yaw_rad=rest_yaw)
        finally:
            publisher.publish(Twist())
            self.node.destroy_publisher(publisher)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--case', choices=tuple(CASES), action='append')
    parser.add_argument('--repeat', default=3, type=int)
    parser.add_argument('--model-dir', type=Path)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.repeat < 1 or (args.output / 'results.json').exists():
        parser.error('positive repetitions and a new output directory are required')
    results = []
    rclpy.init()
    try:
        for repeat in range(1, args.repeat + 1):
            for case in args.case or CASES:
                print(f'START {case} repeat {repeat}', flush=True)
                trial = MotionTrial(case, args.output / f'repeat-{repeat}', 90, model_dir=args.model_dir)
                try:
                    result = trial.execute()
                except Exception as exc:
                    traceback.print_exc()
                    result = dict(case=case, passed=False, error=repr(exc))
                finally:
                    trial.close()
                result['repeat'] = repeat
                results.append(result)
                (trial.directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
                (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
                print(json.dumps({'case': case, 'repeat': repeat, 'passed': result['passed'],
                                  'phases': result.get('phases'), 'error': result.get('error')}), flush=True)
    finally:
        rclpy.shutdown()
    return int(any(not result['passed'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
