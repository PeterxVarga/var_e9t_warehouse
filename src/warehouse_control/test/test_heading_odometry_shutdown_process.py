"""Installed Humble subprocess shutdown and feedback-loss integration checks."""
import json
from math import cos, sin
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


class Probe:
    def __init__(self, directory):
        self.directory, self.processes = directory, []
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('shutdown_probe_' + uuid.uuid4().hex[:8], context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        base = '/' + self.node.get_name()
        self.topics = {k: base + '/' + k for k in ('wheel', 'imu', 'feedback', 'command')}
        self.wheel = self.node.create_publisher(Odometry, self.topics['wheel'], 10)
        self.imu = self.node.create_publisher(Imu, self.topics['imu'], 10)
        self.recovery = self.node.create_publisher(Odometry, self.topics['feedback'], 10)
        self.outputs, self.commands, self.inputs = [], [], {}
        self.node.create_subscription(Odometry, self.topics['feedback'], self.outputs.append, 10)
        self.node.create_subscription(
            Twist, self.topics['command'], lambda m: self.commands.append(
                (time.monotonic(), m.linear.x, m.angular.z)), qos_profile_sensor_data)
        self.count = 0
        self.heading = .2

    def start(self, executable, *args):
        prefix = Path(get_package_prefix('warehouse_control'))
        path = self.directory / (executable + '.log')
        with path.open('w') as log:
            p = subprocess.Popen(
                [str(prefix / 'lib/warehouse_control' / executable), '--ros-args', *args],
                cwd='/tmp', stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((p, path))
        return p, path

    def estimator(self, *args):
        return self.start('heading_odometry',
                          '-r', '/warehouse/odom:=' + self.topics['wheel'],
                          '-r', '/warehouse/imu:=' + self.topics['imu'],
                          '-r', '/warehouse/imu_odom:=' + self.topics['feedback'], *args)

    def sample(self):
        t = self.node.get_clock().now().nanoseconds
        wheel, imu = Odometry(), Imu()
        for m in (wheel, imu):
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(t, 1_000_000_000)
        wheel.header.frame_id, wheel.child_frame_id = 'odom', 'warehouse_robot/base_link'
        wheel.pose.pose.orientation.w = 1.
        wheel.pose.pose.position.x = self.count * .0002
        wheel.twist.twist.linear.x, wheel.twist.twist.angular.z = .12, .03
        wheel.twist.covariance[0] = .04
        imu.header.frame_id = 'warehouse_robot/base_link/imu_heading'
        angle = (.3 + self.heading) / 2  # Installed sensor mount plus body heading.
        imu.orientation.z, imu.orientation.w = sin(angle), cos(angle)
        self.inputs[t] = wheel
        self.count += 1
        return wheel, imu

    def pump(self, seconds=.02, sensors=False, recovery=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if sensors or recovery:
                wheel, imu = self.sample()
                if sensors:
                    self.imu.publish(imu)
                    self.wheel.publish(wheel)
                if recovery:
                    self.recovery.publish(wheel)
            self.executor.spin_once(timeout_sec=.01)
            time.sleep(.01)

    def until(self, predicate, sensors=False, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, 'probe deadline exceeded'
            assert all(p.poll() is None for p, _ in self.processes), [
                path.read_text() for _, path in self.processes]
            self.pump(sensors=sensors)

    def stop(self, process, path, signum, repeated=False):
        start = time.monotonic()
        process.send_signal(signum)
        if repeated:
            time.sleep(.001)
            if process.poll() is None:
                process.send_signal(signum)
        try:
            code = process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
            pytest.fail('shutdown timed out; forced process-group cleanup: ' + path.read_text())
        assert code == 0, path.read_text()
        elapsed = time.monotonic() - start
        assert elapsed < 3
        assert 'Traceback' not in path.read_text()
        with pytest.raises(ProcessLookupError):
            os.kill(process.pid, 0)
        with pytest.raises(ProcessLookupError):
            os.killpg(process.pid, 0)
        (self.directory / (path.stem + '-exit.json')).write_text(json.dumps(
            dict(pid=process.pid, code=process.returncode, seconds=elapsed,
                 signal=signum, repeated=repeated, process_group_gone=True)) + '\n')
        self.processes.remove((process, path))

    def close(self):
        try:
            for p, path in list(self.processes):
                if p.poll() is None:
                    self.stop(p, path, signal.SIGTERM)
        finally:
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.try_shutdown()


@pytest.fixture
def probe(tmp_path):
    p = Probe(tmp_path)
    try:
        yield p
    finally:
        p.close()


@pytest.mark.parametrize('state', ['WAITING_FOR_SENSORS', 'ACTIVE', 'FAULT'])
@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('repeated', [False, True])
def test_installed_shutdown(probe, state, signum, repeated):
    p, log = probe.estimator('-p', 'startup_timeout:=0.3' if state == 'FAULT'
                             else 'startup_timeout:=10.0')
    probe.until(lambda: state + ':' in log.read_text() if state != 'FAULT'
                else 'ESTIMATOR_FAULT:' in log.read_text(), sensors=state == 'ACTIVE')
    if state == 'ACTIVE':
        probe.until(lambda: len(probe.outputs) >= 12, sensors=True)
        assert 'ESTIMATOR_FAULT:' not in log.read_text()
        first = probe.outputs[0]
        stamp = lambda m: m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        origin = probe.inputs[stamp(first)].pose.pose.position.x
        stamps = [stamp(m) for m in probe.outputs]
        assert stamps == sorted(set(stamps))
        for out in probe.outputs:
            source = probe.inputs[stamp(out)]
            dx = source.pose.pose.position.x - origin
            assert out.pose.pose.position.x == pytest.approx(dx * cos(.2), abs=1e-10)
            assert out.pose.pose.position.y == pytest.approx(dx * sin(.2), abs=1e-10)
            assert out.pose.pose.orientation.z == pytest.approx(sin(.1), abs=1e-10)
            assert out.pose.pose.orientation.w == pytest.approx(cos(.1), abs=1e-10)
            assert out.header == source.header and out.child_frame_id == source.child_frame_id
            assert out.twist == source.twist and list(out.pose.covariance) == [0.] * 36
    else:
        assert not probe.outputs
    probe.stop(p, log, signum, repeated)


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_estimator_exit_stops_and_latches_mission_controller(probe, signum):
    probe.heading = 0.  # The mission requires the fresh depot heading at startup.
    p, log = probe.estimator()
    probe.until(lambda: len(probe.outputs) >= 3, sensors=True)
    controller, controller_log = probe.start(
        'mission_controller', '-p', 'pickup_nodes:=[r0c1]', '-p', 'odom_timeout:=0.3',
        '-r', '/warehouse/odom:=' + probe.topics['feedback'],
        '-r', '/warehouse/cmd_vel:=' + probe.topics['command'])
    probe.until(lambda: any(v != 0 or w != 0 for _, v, w in probe.commands), sensors=True)
    probe.stop(p, log, signum)
    stopped_at = time.monotonic()
    probe.until(lambda: 'FAULT: odometry stopped advancing' in controller_log.read_text(),
                sensors=True, timeout=2)
    fault_seconds = time.monotonic() - stopped_at
    assert fault_seconds < 2
    probe.pump(.2, sensors=True)  # Drain in-flight transport before checking silence.
    output_count = len(probe.outputs)
    probe.commands.clear()
    probe.pump(.5, sensors=True)
    assert len(probe.outputs) == output_count
    assert len(probe.commands) >= 3
    assert all(v == w == 0 for _, v, w in probe.commands)
    stopped_commands = len(probe.commands)
    # Fresh feedback is restored directly; it must not rearm the controller.
    probe.commands.clear()
    probe.pump(.6, sensors=True, recovery=True)
    assert len(probe.outputs) > output_count
    assert len(probe.commands) >= 3
    assert all(v == w == 0 for _, v, w in probe.commands)
    text = controller_log.read_text()
    assert 'REACHED:' not in text and 'simulated service completed' not in text
    (probe.directory / 'feedback-loss.json').write_text(json.dumps(dict(
        fault_seconds=fault_seconds, stopped_commands=stopped_commands,
        recovered_commands=len(probe.commands), all_commands_zero=True,
        recovered_feedback_samples=len(probe.outputs)-output_count)) + '\n')
    probe.stop(controller, controller_log, signal.SIGTERM)


def test_installed_invalid_startup_stays_nonzero(probe):
    p, log = probe.estimator('-p', 'startup_timeout:=-1.0')
    assert p.wait(timeout=5) == 1
    assert 'Heading odometry failed: startup_timeout' in log.read_text()
