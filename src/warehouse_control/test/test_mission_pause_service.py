"""ROS pause service, checkpoint and paused feedback fault regressions."""

from math import nan

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from test_controller import harness, odom
from test_mission_controller import configured


def request(node):
    return node.on_pause(Trigger.Request(), Trigger.Response())


def start(node, harness):
    node.on_odom(odom())
    node.tick()
    harness.clock.ros += 50000000
    node.on_odom(odom(harness.clock.ros))
    node.tick()
    assert node.mission.started and node.state == 'TRACKING'


def test_real_service_travel_and_repeated_request(configured, harness):
    node = configured(('r1c0', 'r1c0'))
    start(node, harness)
    assert harness.commands[-1] != (0, 0)
    client_node = rclpy.create_node('pause_test_client')
    client = client_node.create_client(Trigger, '/warehouse/pause_mission')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    try:
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=2)
        assert future.result().success and future.result().message == 'Pause request accepted.'
        assert node.state == node.mission.state == 'PAUSED'
        assert harness.commands[-1] == (0, 0)
        checkpoint = node.mission.pause()
        assert checkpoint.original_requests == ('r1c0', 'r1c0')
        assert checkpoint.completed_order == () and checkpoint.remaining_order == (0, 1)
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=2)
        assert future.result().success and node.mission.pause() is checkpoint
        for _ in range(5):
            harness.clock.ros += 50000000
            node.on_odom(odom(harness.clock.ros))
            node.tick()
            assert node.state == 'PAUSED' and harness.commands[-1] == (0, 0)
    finally:
        executor.remove_node(node)
        executor.remove_node(client_node)
        executor.shutdown()
        client_node.destroy_node()


def test_pause_during_partial_service_preserves_request_id(configured, harness):
    node = configured(('r1c0', 'r1c0'))
    start(node, harness)
    harness.clock.ros += 50000000
    node.on_odom(odom(harness.clock.ros, y=2.0))
    node.tick()
    assert node.state == 'SERVICING'
    harness.clock.ros += 500000000
    node.on_odom(odom(harness.clock.ros, y=2.0))
    node.tick()
    assert request(node).success
    checkpoint = node.mission.pause()
    assert checkpoint.completed_order == () and checkpoint.remaining_order == (0, 1)
    assert node.mission.service_start is None and node.mission.completed_requests == 0
    for _ in range(25):
        harness.clock.ros += 50000000
        node.on_odom(odom(harness.clock.ros, y=2.0))
        node.tick()
    assert node.mission.completed_requests == 0 and node.mission.pause() is checkpoint
    assert node.state == 'PAUSED' and harness.commands[-1] == (0, 0)


def test_checkpoint_preserves_completed_and_remaining_original_ids(configured, harness):
    node = configured(('r1c0', 'r1c0'))
    start(node, harness)
    harness.clock.ros += 50000000
    node.on_odom(odom(harness.clock.ros, y=2.0))
    node.tick()
    for _ in range(20):
        harness.clock.ros += 50000000
        node.on_odom(odom(harness.clock.ros, y=2.0))
        node.tick()
    assert node.mission.completed_requests == 1
    assert request(node).success
    checkpoint = node.mission.pause()
    assert checkpoint.original_requests == ('r1c0', 'r1c0')
    assert checkpoint.completed_order == (0,) and checkpoint.remaining_order == (1,)


@pytest.mark.parametrize('state', ['before', 'reached', 'fault'])
def test_rejected_states(configured, harness, state):
    node = configured(()) if state == 'reached' else configured()
    if state == 'reached':
        node.on_odom(odom())
        node.tick()
    elif state == 'fault':
        node.on_odom(odom(x=0.06))
        node.tick()
    previous = node.state
    response = request(node)
    assert not response.success and response.message
    assert node.state == previous
    node.tick()
    assert harness.commands[-1] == (0, 0)


@pytest.mark.parametrize('failure', ['missing', 'repeated', 'stale', 'frame', 'nan',
                                     'backwards_odom', 'backwards_clock'])
def test_paused_feedback_fault_latches_after_recovery(configured, harness, failure):
    node = configured()
    start(node, harness)
    assert request(node).success
    checkpoint = node.mission.pause()
    if failure in ('missing', 'repeated'):
        harness.clock.wall += 1.01
        if failure == 'repeated':
            node.on_odom(odom(node.stamp))
    elif failure == 'backwards_clock':
        harness.clock.ros -= 1
    else:
        harness.clock.ros += 50000000
        msg = odom(harness.clock.ros)
        if failure == 'stale':
            harness.clock.ros += 1100000000
        elif failure == 'frame':
            msg.header.frame_id = 'world'
        elif failure == 'nan':
            msg.pose.pose.position.x = nan
        else:
            msg = odom(node.stamp - 1)
        node.on_odom(msg)
    node.tick()
    assert node.state == node.mission.state == 'FAULT'
    assert not request(node).success and harness.commands[-1] == (0, 0)
    harness.clock.ros += 2000000000
    node.on_odom(odom(harness.clock.ros))
    node.tick()
    assert node.state == node.mission.state == 'FAULT'
    assert node.mission._checkpoint is checkpoint and harness.commands[-1] == (0, 0)
