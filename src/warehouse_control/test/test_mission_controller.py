"""Humble parameter typing, callback fault latching and installed entrypoint."""

from pathlib import Path
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix
from rclpy.parameter import Parameter

from warehouse_control.mission_controller import MissionController
import warehouse_control.mission_controller as mission_module
from test_controller import harness, odom


@pytest.fixture
def configured(harness, monkeypatch):
    declare = MissionController.declare_parameter
    def create(requests=('r2c2', 'r1c4')):
        def override(node, name, default=None, *args, **kwargs):
            if name == 'pickup_nodes':
                default = list(requests) if isinstance(requests, tuple) else requests
            return declare(node, name, default, *args, **kwargs)
        monkeypatch.setattr(MissionController, 'declare_parameter', override)
        return harness.create(MissionController)
    return create


@pytest.mark.parametrize('requests', [(), ('r2c2', 'r1c4')])
def test_read_only_typed_parameter_and_no_other_targets(configured, requests):
    node = configured(requests)
    descriptor = node.describe_parameter('pickup_nodes')
    assert descriptor.read_only and not descriptor.dynamic_typing
    assert node.get_parameter('pickup_nodes').type_ == Parameter.Type.STRING_ARRAY
    assert tuple(node.get_parameter('pickup_nodes').value) == requests
    assert not node.set_parameters([Parameter('pickup_nodes', value=['r1c0'])])[0].successful
    assert not node.has_parameter('goal_node') and not node.has_parameter('target_x')


@pytest.mark.parametrize('requests', [None, 'r2c2', 1, [1], [False], ('missing',)])
def test_invalid_or_missing_parameter_before_publisher(configured, harness, requests):
    with pytest.raises(ValueError, match='pickup_nodes|unknown pickup'):
        configured(requests)
    assert harness.commands == []


def test_invalid_preparation_creates_no_publisher(configured, harness, monkeypatch):
    def invalid(*args):
        raise ValueError('mission leg 2 is unreachable')
    monkeypatch.setattr(mission_module, 'prepare_mission', invalid)
    with pytest.raises(ValueError, match='unreachable'):
        configured()
    assert harness.commands == []


def test_empty_does_not_move_and_completion_latches(configured, harness):
    node = configured(())
    node.tick()
    assert node.state == 'WAITING_FOR_ODOM'
    node.on_odom(odom())
    node.tick()
    assert node.state == node.mission.state == 'REACHED'
    harness.clock.ros = 1
    node.on_odom(odom(1, x=10.0))
    node.tick()
    assert node.state == 'REACHED' and all(c == (0, 0) for c in harness.commands)


def test_invalid_start_latches(configured, harness):
    node = configured()
    node.on_odom(odom(x=0.06))
    node.tick()
    assert node.state == node.mission.state == 'FAULT'
    harness.clock.ros += 50000000
    node.on_odom(odom(harness.clock.ros))
    node.tick()
    assert node.state == 'FAULT' and all(c == (0, 0) for c in harness.commands)


def test_tolerance_departure_between_ticks_resets_service(configured, harness):
    node = configured(('r1c0',))
    node.on_odom(odom())
    node.tick()
    harness.clock.ros += 50000000
    node.on_odom(odom(harness.clock.ros, y=2.0))
    node.tick()
    assert node.state == 'SERVICING'
    harness.clock.ros += 900000000
    node.on_odom(odom(harness.clock.ros, y=1.94))
    assert node.mission.service_start is None
    harness.clock.ros += 100000000
    node.on_odom(odom(harness.clock.ros, y=2.0))
    node.tick()
    assert node.mission.completed_requests == 0
    assert node.mission.service_start == (harness.clock.ros, harness.clock.ros)


@pytest.mark.parametrize('phase', ['travel', 'service'])
@pytest.mark.parametrize('failure', ['missing', 'repeated', 'frame', 'clock', 'overflow'])
def test_feedback_faults_during_travel_and_service(configured, harness, phase, failure):
    node = configured(('r1c0',))
    node.on_odom(odom())
    node.tick()
    if phase == 'service':
        harness.clock.ros += 50000000
        node.on_odom(odom(harness.clock.ros, y=2.0))
        node.tick()
        assert node.state == 'SERVICING'
    if failure in ('missing', 'repeated'):
        harness.clock.wall += 1.01
        if failure == 'repeated':
            node.on_odom(odom(node.stamp, y=2.0 if phase == 'service' else 0.0))
    elif failure in ('frame', 'overflow'):
        msg = odom(node.stamp + 50000000)
        harness.clock.ros = node.stamp + 50000000
        if failure == 'frame':
            msg.header.frame_id = 'world'
        else:
            msg.pose.pose.position.x = msg.pose.pose.position.y = 1.79e308
        node.on_odom(msg)
    else:
        harness.clock.ros -= 1
    node.tick()
    assert node.state == node.mission.state == 'FAULT'
    count = node.mission.completed_requests
    harness.clock.ros += 2000000000
    harness.clock.wall += 0.1
    node.on_odom(odom(harness.clock.ros, y=2.0))
    node.tick()
    assert node.state == 'FAULT' and node.mission.completed_requests == count
    assert harness.commands[-1] == (0, 0)


@pytest.mark.parametrize('argument,message', [
    (None, 'pickup_nodes is required'),
    ('pickup_nodes:=r2c2', 'STRING_ARRAY'),
    ('pickup_nodes:=[1]', 'STRING_ARRAY'),
    ("pickup_nodes:=['missing']", 'unknown pickup'),
])
def test_installed_parameter_errors(tmp_path, argument, message):
    args = ['ros2', 'run', 'warehouse_control', 'mission_controller', '--ros-args']
    if argument:
        args += ['-p', argument]
    result = subprocess.run(args, cwd=tmp_path, text=True, capture_output=True, timeout=15)
    assert result.returncode == 1 and message in result.stderr
    assert 'WAITING_FOR_ODOM' not in result.stderr + result.stdout


def test_installed_explicit_empty_humble_parameter(tmp_path):
    executable = Path(get_package_prefix('warehouse_control')) / 'lib/warehouse_control/mission_controller'
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'pickup_nodes:=[]'],
                               cwd=tmp_path, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        # No simulator: startup must accept [], wait for feedback and stay alive.
        time.sleep(1)
        assert process.poll() is None
        process.send_signal(signal.SIGTERM)
        out, err = process.communicate(timeout=10)
        assert process.returncode == 0, (out, err)
        assert 'FIFO pickup requests: (); planned length=0 m' in out + err
        assert 'WAITING_FOR_ODOM' in out + err
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
