"""Closure replacement, atomic rejection, request identity and marker contracts."""

from math import pi
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from warehouse_interfaces.msg import ClosedEdge
from warehouse_interfaces.srv import ResumeMissionWithClosures

from test_controller import harness
from test_mission_controller import configured
from test_mission_pause_service import request as pause, start
from test_mission_resume_service import observe, resume, sample
import warehouse_control.mission as mission_module
from warehouse_control.mission_markers import mission_markers
from warehouse_control.route import PlanarTransform


EDGE = ('r0c0', 'r1c0')


def request(node, edges=(EDGE,), start_node='r0c0'):
    req = ResumeMissionWithClosures.Request(start_node=start_node)
    req.closed_edges = [ClosedEdge(node_a=a, node_b=b) for a, b in edges]
    return node.on_resume_with_closures(req, ResumeMissionWithClosures.Response())


def paused(configured, harness):
    node = configured(('r1c0', 'r1c0'))
    start(node, harness)
    assert pause(node).success
    observe(node, harness)
    return node


def snapshot(node):
    m = node.mission
    return (m.plan, m._routes, m.points, m._checkpoint, m._active_order,
            m.completed_requests, m.leg_index, node.accepted_closures, node.before_resume_plan)


def capture(node):
    markers = []
    node.marker_publisher = SimpleNamespace(publish=markers.append)
    return markers


def by_ns(array, namespace):
    return [m for m in array.markers if m.ns == namespace]


def test_detour_retain_then_explicit_reopen(configured, harness):
    node = paused(configured, harness)
    markers = capture(node)
    assert request(node, [tuple(reversed(EDGE))]).success
    assert node.accepted_closures == (EDGE,)
    detour = node.mission.plan
    assert detour.legs[0].nodes == ('r0c0', 'r0c1', 'r0c2', 'r0c3', 'r0c4',
                                   'r1c4', 'r1c3', 'r1c2', 'r1c1', 'r1c0')
    assert EDGE not in tuple(zip(detour.nodes, detour.nodes[1:]))
    assert node.mission.validated_graph.edges  # Startup graph was not replaced.
    assert pause(node).success
    observe(node, harness)
    assert resume(node).success
    assert node.accepted_closures == (EDGE,) and node.mission.plan == detour
    assert pause(node).success
    observe(node, harness)
    assert request(node, []).success
    assert node.accepted_closures == ()
    assert node.mission.plan.legs[0].nodes == ('r0c0', 'r1c0')
    assert markers[-1].markers[0].action == Marker.DELETEALL
    assert not by_ns(markers[-1], 'accepted_closures')
    assert node.before_resume_plan == detour


def test_nonempty_list_replaces_instead_of_accumulating(configured, harness):
    node = paused(configured, harness)
    assert request(node).success
    assert pause(node).success
    observe(node, harness)
    markers = capture(node)
    replacement = ('r0c1', 'r0c2')
    assert request(node, [replacement]).success
    assert node.accepted_closures == (replacement,)
    assert node.mission.plan.legs[0].nodes == ('r0c0', 'r1c0')
    line = by_ns(markers[-1], 'accepted_closures')[0]
    assert [(p.x, p.y) for p in line.points] == [(2.0, 0.0), (4.0, 0.0)]
    assert markers[-1].markers[0].action == Marker.DELETEALL


@pytest.mark.parametrize('edges,start_node,error', [
    ([('missing', 'r0c0')], 'r0c0', 'unknown blocked edge endpoint'),
    ([('r0c0', 'r2c0')], 'r0c0', 'does not exist'),
    ([('r0c0', 'r0c0')], 'r0c0', 'self-loop'),
    ([EDGE, tuple(reversed(EDGE))], 'r0c0', 'duplicate'),
    ([EDGE, ('r0c0', 'r0c1')], 'r0c0', 'unreachable'),
    ([EDGE], 'missing', 'unknown graph node'),
    ([EDGE], 'r1c0', 'does not match'),
])
def test_invalid_replacement_preserves_previous_acceptance(configured, harness,
                                                          edges, start_node, error):
    node = paused(configured, harness)
    # Seed a real accepted snapshot; failure must retain nonempty closures too.
    assert request(node, [('r0c1', 'r0c2')]).success
    assert pause(node).success
    observe(node, harness)
    before = snapshot(node)
    markers = capture(node)
    response = request(node, edges, start_node)
    assert not response.success and error in response.message
    assert snapshot(node) == before
    assert node.mission.plan is before[0] and node.mission._checkpoint is before[3]
    assert node.state == node.mission.state == 'PAUSED'
    assert harness.commands[-1] == (0, 0) and not markers
    sample(node, harness)
    assert node.state == 'PAUSED' and harness.commands[-1] == (0, 0)


@pytest.mark.parametrize('failure', ['moving', 'stale_wall', 'stale_ros', 'short', 'gap',
                                     'planning', 'precommit'])
def test_observation_and_commit_failure(configured, harness, monkeypatch, failure):
    node = paused(configured, harness)
    before = snapshot(node)
    markers = capture(node)
    if failure == 'moving':
        observe(node, harness, velocity=0.02)
    elif failure == 'stale_wall':
        harness.clock.wall += 0.21
    elif failure == 'stale_ros':
        harness.clock.ros += 210000000
    elif failure == 'short':
        node.rest_samples.clear()
        sample(node, harness, tick=False)
    elif failure == 'gap':
        harness.clock.wall += 0.25
        sample(node, harness, tick=False)
    else:
        original = mission_module.plan_remaining_mission
        def altered(*args):
            result = original(*args)
            if failure == 'planning':
                raise ValueError('injected planning failure')
            harness.clock.wall += 0.21
            return result
        monkeypatch.setattr(mission_module, 'plan_remaining_mission', altered)
    response = request(node)
    assert not response.success and response.message
    assert snapshot(node) == before and not markers
    assert node.state == node.mission.state == 'PAUSED'
    assert harness.commands[-1] == (0, 0)


@pytest.mark.parametrize('state', ['before', 'tracking', 'reached', 'fault'])
def test_state_gate(configured, harness, state):
    node = configured(()) if state == 'reached' else configured()
    if state in ('tracking', 'reached', 'fault'):
        start(node, harness) if state == 'tracking' else sample(node, harness)
    if state == 'fault':
        node.transition('FAULT', 'test fault')
    before = snapshot(node)
    assert not request(node).success
    assert snapshot(node) == before


def test_completed_duplicates_and_marker_replacement(configured, harness):
    node = configured(('r1c0', 'r1c0', 'r1c0'))
    start(node, harness)
    sample(node, harness, y=2.0)
    for _ in range(21):
        sample(node, harness, y=2.0)
    assert node.mission.completed_requests == 1
    assert pause(node).success
    observe(node, harness, y=2.0)
    markers = capture(node)
    assert request(node, [EDGE], 'r1c0').success
    assert node.mission.plan.remaining_order == (1, 2)
    labels = by_ns(markers[-1], 'remaining_request_labels')
    assert [m.text for m in labels] == ['Request 2: r1c0', 'Request 3: r1c0']
    assert labels[0].pose.position.z != labels[1].pose.position.z
    for _ in range(46):
        sample(node, harness, y=2.0)
    assert node.mission.completed_requests == 3
    assert not by_ns(markers[-1], 'remaining_request_labels')
    for x, y in node.mission.route.points:
        sample(node, harness, x=float(x), y=float(y))
    assert node.state == 'REACHED' and harness.commands[-1] == (0, 0)
    assert all(m.markers[0].action == Marker.DELETEALL for m in markers)


def test_marker_coordinates_and_rotated_static_transform(configured):
    node = configured(('r1c0',))
    for transform in (node.transform, PlanarTransform(-4.0, -3.0, pi / 2)):
        array = mission_markers(node.mission.validated_graph, transform,
                                node.mission.plan, node.mission.plan, [EDGE],
                                node.mission.original_requests, (0,),
                                rclpy.time.Time(nanoseconds=123).to_msg())
        assert array.markers[0].action == Marker.DELETEALL
        assert all(m.header.frame_id == 'odom' and m.header.stamp.nanosec == 123
                   for m in array.markers)
        for ns, nodes in [('accepted_closures', EDGE),
                          ('accepted_plan', node.mission.plan.nodes),
                          ('plan_before_resume', node.mission.plan.nodes)]:
            points = by_ns(array, ns)[0].points
            assert [(p.x, p.y) for p in points] == [
                transform.world_to_odom(node.mission.validated_graph.nodes[n]) for n in nodes]
        assert 'NOT TRAVELLED PATHS' in by_ns(array, 'legend')[0].text


def test_stationary_depot_plan_has_no_invalid_single_point_line(configured):
    node = configured(())
    array = mission_markers(node.mission.validated_graph, node.transform,
                            node.mission.plan, node.mission.plan, [], (), (),
                            rclpy.time.Time().to_msg())
    assert not by_ns(array, 'accepted_plan')
    assert not by_ns(array, 'plan_before_resume')
    assert not by_ns(array, 'remaining_goals')
    assert by_ns(array, 'plan_start') and array.markers[0].action == Marker.DELETEALL


def test_real_typed_call_and_late_marker_subscriber(configured, harness):
    node = paused(configured, harness)
    client_node = rclpy.create_node('closures_test_client')
    client = client_node.create_client(ResumeMissionWithClosures,
                                       '/warehouse/resume_mission_with_closures')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    arrays = []
    try:
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(ResumeMissionWithClosures.Request(
            start_node='r0c0', closed_edges=[ClosedEdge(node_a=EDGE[0], node_b=EDGE[1])]))
        executor.spin_until_future_complete(future, timeout_sec=2)
        assert future.result().success, future.result().message
        assert node.state == 'TRACKING'
        client_node.create_subscription(MarkerArray, '/warehouse/mission_plan_markers',
                                        arrays.append, QoSProfile(
                                            depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for _ in range(30):
            executor.spin_once(timeout_sec=0.05)
            if arrays:
                break
        assert len(arrays) == 1 and by_ns(arrays[0], 'accepted_closures')
        assert len(by_ns(arrays[0], 'accepted_plan')[0].points) == len(node.mission.plan.nodes)
    finally:
        executor.remove_node(node)
        executor.remove_node(client_node)
        executor.shutdown()
        client_node.destroy_node()
