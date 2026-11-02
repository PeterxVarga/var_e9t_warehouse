"""Typed ROS resume service and paused feedback synchronization regressions."""

from math import nan

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from warehouse_interfaces.srv import ResumeMission

from test_controller import harness, odom
from test_mission_controller import configured
from test_mission_pause_service import request as pause, start
import warehouse_control.mission as mission_module


def resume(node, name='r0c0'):
    req = ResumeMission.Request()
    req.start_node = name
    return node.on_resume(req, ResumeMission.Response())


def sample(node, harness, x=0.0, y=0.0, velocity=0.0, tick=True):
    harness.clock.ros += 50000000
    harness.clock.wall += 0.05
    msg = odom(harness.clock.ros, x, y)
    msg.twist.twist.linear.x = velocity
    node.on_odom(msg)
    if tick:
        node.tick()


def observe(node, harness, x=0.0, y=0.0, velocity=0.0):
    for i in range(13):
        sample(node, harness, x, y, velocity, tick=i != 12)
    harness.clock.ros += 1000000


def test_typed_service_real_call(configured, harness):
    node = configured(('r1c0',))
    start(node, harness)
    assert pause(node).success
    observe(node, harness)
    client_node = rclpy.create_node('resume_test_client')
    client = client_node.create_client(ResumeMission, '/warehouse/resume_mission')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    try:
        assert client.wait_for_service(timeout_sec=2)
        req = ResumeMission.Request()
        req.start_node = 'r0c0'
        future = client.call_async(req)
        executor.spin_until_future_complete(future, timeout_sec=2)
        assert future.result().success, future.result().message
        assert node.state == node.mission.state == 'TRACKING'
        assert harness.commands[-1] == (0, 0)
    finally:
        executor.remove_node(node)
        executor.remove_node(client_node)
        executor.shutdown()
        client_node.destroy_node()


@pytest.mark.parametrize('case', ['missing', 'wrong_position', 'moving', 'nan_velocity',
                                  'short', 'lost', 'stale_ros', 'duplicate', 'gap', 'pose_drift'])
def test_rejections_keep_plan_and_stop(configured, harness, case):
    node = configured(('r1c0',))
    start(node, harness)
    assert pause(node).success
    plan, routes, checkpoint = node.mission.plan, node.mission._routes, node.mission._checkpoint
    if case == 'short':
        sample(node, harness, tick=False)
    else:
        observe(node, harness, velocity=0.02 if case == 'moving' else nan if case == 'nan_velocity' else 0.0)
    if case == 'lost':
        harness.clock.wall += 0.21
    elif case == 'stale_ros':
        harness.clock.ros += 210000000
    elif case == 'duplicate':
        node.on_odom(odom(node.stamp))
    elif case == 'pose_drift':
        sample(node, harness, x=0.01, tick=False)
        sample(node, harness, tick=False)
    elif case == 'gap':
        harness.clock.ros += 250000000
        harness.clock.wall += 0.25
        sample(node, harness, tick=False)
    response = resume(node, 'missing' if case == 'missing' else 'r1c0' if case == 'wrong_position' else 'r0c0')
    assert not response.success and response.message
    if case == 'missing':
        assert 'unknown graph node' in response.message
    assert node.state == node.mission.state == 'PAUSED'
    assert node.mission.plan is plan and node.mission._routes is routes
    assert node.mission._checkpoint is checkpoint and harness.commands[-1] == (0, 0)


@pytest.mark.parametrize('state', ['before', 'tracking', 'reached', 'fault'])
def test_nonpaused_rejected(configured, harness, state):
    node = configured(()) if state == 'reached' else configured()
    if state == 'tracking':
        start(node, harness)
    elif state == 'reached':
        node.on_odom(odom())
        node.tick()
    elif state == 'fault':
        node.on_odom(odom(x=0.06))
        node.tick()
    assert not resume(node).success
    assert node.state != 'PAUSED'


@pytest.mark.parametrize('failure', ['planning', 'planning_timeout', 'clock_reversal', 'replaced'])
def test_plan_commit_failure_is_atomic(configured, harness, monkeypatch, failure):
    node = configured()
    start(node, harness)
    assert pause(node).success
    observe(node, harness)
    plan, routes, checkpoint = node.mission.plan, node.mission._routes, node.mission._checkpoint
    original = mission_module.plan_remaining_mission
    def altered(*args):
        result = original(*args)
        if failure == 'planning':
            raise ValueError('planning failed')
        if failure == 'planning_timeout':
            harness.clock.wall += 0.21
        elif failure == 'clock_reversal':
            harness.clock.ros -= 10000000
        else:
            node.pose = (9.0, 9.0, 0.0)
        return result
    monkeypatch.setattr(mission_module, 'plan_remaining_mission', altered)
    result = resume(node)
    assert not result.success and result.message
    assert node.state == node.mission.state == 'PAUSED'
    assert (node.mission.plan, node.mission._routes, node.mission._checkpoint) == (plan, routes, checkpoint)
    assert harness.commands[-1] == (0, 0)


def test_duplicate_requests_keep_ids_and_interrupted_service_restarts(configured, harness):
    node = configured(('r1c0', 'r1c0', 'r1c0'))
    start(node, harness)
    sample(node, harness, y=2.0)
    for _ in range(21):
        sample(node, harness, y=2.0)
    assert node.mission.completed_requests == 1
    sample(node, harness, y=2.0)
    assert node.state == 'SERVICING'
    for _ in range(8):
        sample(node, harness, y=2.0)
    assert pause(node).success
    assert node.mission._checkpoint.completed_order == (0,)
    assert node.mission._checkpoint.remaining_order == (1, 2)
    observe(node, harness, y=2.0)
    assert resume(node, 'r1c0').success
    sample(node, harness, y=2.0)
    assert node.mission.request_index(0) == 1
    assert node.mission.completed_requests == 1
    assert node.state == 'SERVICING'
    for _ in range(19):
        sample(node, harness, y=2.0)
    assert node.mission.completed_requests == 1
    for _ in range(3):
        sample(node, harness, y=2.0)
    assert node.mission.completed_requests == 2
    sample(node, harness, y=2.0)
    assert node.mission.request_index(1) == 2
    for _ in range(21):
        sample(node, harness, y=2.0)
    assert node.mission.completed_requests == 3


def test_slow_sim_clock_does_not_create_unbounded_window(configured, harness):
    node = configured()
    start(node, harness)
    assert pause(node).success
    for _ in range(60):
        harness.clock.ros += 1000000
        harness.clock.wall += 0.05
        node.on_odom(odom(harness.clock.ros))
    assert node.rest_samples[-1][1] - node.rest_samples[0][1] <= 2.0
    assert not resume(node).success
    assert node.state == node.mission.state == 'PAUSED'
    assert harness.commands[-1] == (0, 0)
