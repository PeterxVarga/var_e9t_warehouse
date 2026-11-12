#!/usr/bin/env python3
"""Exercise the unified IMU-backed demo with the existing physical closure gates.

Only run inside a dedicated isolated ROS/Gazebo container. Physical world poses
are independent observations, never navigation or closure input.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from validate_mission_closures import ClosuresTrial
from validate_route_following import distance


class DemoTrial(ClosuresTrial):
    def __init__(self, *args):
        super().__init__(*args)
        self.demo_process = None
        self.imu_samples = 0
        self.shutdown = {'checked': False}
        self.node.create_subscription(
            Odometry, '/warehouse/imu_odom', self.on_imu, qos_profile_sensor_data)

    def on_imu(self, _message):
        self.imu_samples += 1

    def start(self, name, args, **kwargs):
        # The parent closure validator starts a separate mission controller;
        # here that process is already owned by the unified demo launch.
        if name == 'controller' and self.demo_process is not None:
            return self.demo_process
        return super().start(name, args, **kwargs)

    def start_simulation(self):
        self.demo_process = super().start('controller', [
            'ros2', 'launch', 'warehouse_control', 'warehouse_demo.launch.py',
            'headless:=true', 'rviz:=false',
        ])
        self.until(lambda: self.sim_time > 0 and len(self.odom) >= 5
                   and self.imu_samples >= 5, timeout=60)
        self.until(lambda: self.node.count_publishers('/warehouse/cmd_vel') == 1
                   and self.node.count_publishers('/warehouse/imu_odom') == 1
                   and self.node.count_subscribers('/warehouse/imu_odom') >= 2,
                   timeout=15)
        pose_process = super().start('pose_transport', [
            'ign', 'topic', '-t', '/world/warehouse/dynamic_pose/info',
            '-e', '--json-output',
        ], pipe=True)
        threading.Thread(target=self.poses, args=(pose_process,), daemon=True).start()
        self.until(lambda: len(self.physical) >= 10, timeout=30)
        assert distance(self.physical[-1][1:3], (-4, -3)) < 0.01
        return None, None

    def execute(self):
        result = super().execute()
        assert self.imu_samples > 0
        assert self.node.count_publishers('/warehouse/cmd_vel') == 1
        result['integrated_launch'] = True
        result['imu_feedback_samples'] = self.imu_samples
        result['single_velocity_publisher'] = True
        return result

    def close(self):
        if self.demo_process is not None:
            if self.demo_process.poll() is None:
                os.killpg(self.demo_process.pid, signal.SIGINT)
                try:
                    self.demo_process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    self.shutdown['error'] = 'Launch did not exit within 15 s of SIGINT'
                except Exception as exc:
                    self.shutdown['error'] = repr(exc)
            self.shutdown.update(checked=True, exit_code=self.demo_process.poll(),
                                 exited=self.demo_process.poll() is not None)
        super().close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--case', choices=('detour', 'unreachable'), action='append')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    rclpy.init()
    try:
        for case in args.case or ('detour', 'unreachable'):
            print('START', case, flush=True)
            trial = DemoTrial(case, args.output, 420)
            try:
                result = trial.execute()
            except Exception as exc:
                import traceback
                traceback.print_exc()
                result = dict(trial.result, case=case, passed=False, error=repr(exc))
            finally:
                trial.close()
            result['launch_shutdown'] = trial.shutdown
            result['passed'] = bool(
                result.get('passed') and trial.shutdown.get('exit_code') == 0
                and 'error' not in trial.shutdown)
            results.append(result)
            (trial.directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()
    return int(any(not result['passed'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
