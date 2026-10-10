"""ROS-node callback regressions with captured commands, without a simulator."""

from math import nan
import signal
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from warehouse_control.control import goal_command
from warehouse_control.goal_controller import GoalController, run_controller
from warehouse_control.route_controller import RouteController
import warehouse_control.goal_controller as controller_module
import warehouse_control.route_controller as route_module


@pytest.fixture
def harness(monkeypatch):
    commands, nodes = [], []
    clock = SimpleNamespace(ros=1000000000, wall=10.0)
    monkeypatch.setattr(controller_module.time, 'monotonic', lambda: clock.wall)
    # No command is sent to a live robot, including the constructor's zero.
    create_publisher = GoalController.create_publisher

    def capture_commands(node, msg_type, topic, *args, **kwargs):
        if msg_type is Twist and topic == '/warehouse/cmd_vel':
            return SimpleNamespace(publish=lambda msg:
                                   commands.append((msg.linear.x, msg.angular.z)))
        return create_publisher(node, msg_type, topic, *args, **kwargs)

    monkeypatch.setattr(GoalController, 'create_publisher', capture_commands)
    rclpy.init(args=[])

    def create(factory=RouteController):
        node = factory()
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=clock.ros))
        nodes.append(node)
        return node

    yield SimpleNamespace(create=create, commands=commands, clock=clock)
    for node in nodes:
        node.timer.cancel()
        node.destroy_node()
    rclpy.shutdown()


def odom(stamp=1000000000, x=0.0, y=0.0):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'warehouse_robot/base_link'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    return msg


def start(harness):
    node = harness.create()
    node.on_odom(odom())
    node.tick()  # Validate start and finish the depot waypoint without motion.
    assert node.route.index == 1 and harness.commands[-1] == (0, 0)
    node.tick()
    assert harness.commands[-1][1] > 0  # First leg faces +y; rotate first.
    return node


def assert_fault_stays_stopped(node, harness):
    assert node.state == 'FAULT' and harness.commands[-1] == (0, 0)
    index = node.route.index
    harness.clock.wall += 0.05
    harness.clock.ros += 50000000
    node.on_odom(odom(harness.clock.ros, 0.0, 2.0))
    node.tick()
    assert node.state == 'FAULT' and node.route.index == index
    assert harness.commands[-1] == (0, 0)


def test_single_goal_hook_preserves_commands(harness):
    node = harness.create(GoalController)
    node.on_odom(odom())
    node.tick()
    expected = goal_command(0, 0, 0, 1, 0, node.config)
    assert node.get_name() == 'goal_controller'
    assert harness.commands[-1] == expected[:2] and node.state == 'TRACKING'


def test_absent_feedback_is_stationary(harness):
    node = harness.create()
    node.tick()
    assert node.state == 'WAITING_FOR_ODOM' and node.route.index == 0
    assert all(command == (0, 0) for command in harness.commands)
    assert node.describe_parameter('goal_node').read_only
    assert not node.has_parameter('target_x')


@pytest.mark.parametrize('xy', [(0.06, 0.0), (0.0, 0.06)])
def test_moved_start_faults_without_motion(harness, xy):
    node = harness.create()
    node.on_odom(odom(x=xy[0], y=xy[1]))
    node.tick()
    assert all(command == (0, 0) for command in harness.commands)
    assert_fault_stays_stopped(node, harness)


@pytest.mark.parametrize('failure', [
    'frame', 'child_frame', 'quaternion', 'nonfinite', 'backwards_stamp',
    'stale_stamp', 'repeated_stamp', 'absent_stamp', 'backwards_clock',
])
def test_route_feedback_faults_latch(harness, failure):
    node = start(harness)
    msg = odom(stamp=1050000000)
    harness.clock.ros = 1050000000
    if failure == 'frame':
        msg.header.frame_id = 'world'
    elif failure == 'child_frame':
        msg.child_frame_id = 'other'
    elif failure == 'quaternion':
        msg.pose.pose.orientation.w = 0.0
    elif failure == 'nonfinite':
        msg.pose.pose.position.x = nan
    elif failure == 'backwards_stamp':
        msg = odom(stamp=999999999)
    elif failure == 'stale_stamp':
        harness.clock.ros = 2100000001
    elif failure in ('repeated_stamp', 'absent_stamp'):
        harness.clock.wall += 1.01
        msg = odom() if failure == 'repeated_stamp' else None
    elif failure == 'backwards_clock':
        harness.clock.ros = 999999999
    if msg is not None:
        node.on_odom(msg)
    node.tick()
    assert_fault_stays_stopped(node, harness)


def test_route_completion_only_at_final_waypoint(harness):
    node = start(harness)
    for index, (x, y) in enumerate(node.route.points[1:], 1):
        harness.clock.ros += 50000000
        harness.clock.wall += 0.05
        node.on_odom(odom(harness.clock.ros, float(x), float(y)))
        node.tick()
        assert harness.commands[-1] == (0, 0)
        assert (node.state == 'REACHED') == (index == len(node.route.points) - 1)
    harness.clock.ros = 1  # A completed route stays completed after clock reset.
    node.on_odom(odom(stamp=1, x=10.0))
    node.tick()
    assert node.state == 'REACHED' and harness.commands[-1] == (0, 0)


def test_invalid_plan_creates_no_publisher(harness, monkeypatch):
    def invalid_plan(*args):
        raise ValueError('no graph route')
    monkeypatch.setattr(route_module, 'prepare_route', invalid_plan)
    with pytest.raises(ValueError, match='no graph route'):
        harness.create()
    assert harness.commands == []


@pytest.mark.parametrize('kind', ['keyboard', 'sigterm', 'sigint'])
def test_runner_cancels_and_stops_on_interrupt(monkeypatch, kind):
    events = []
    node = SimpleNamespace(
        timer=SimpleNamespace(cancel=lambda: events.append('cancel')),
        publish_stop=lambda: events.append('stop'),
        destroy_node=lambda: events.append('destroy'))
    monkeypatch.setattr(rclpy, 'init', lambda **k: events.append('init'))
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(controller_module.time, 'sleep', lambda t: None)
    previous_term = signal.getsignal(signal.SIGTERM)
    previous_int = signal.getsignal(signal.SIGINT)
    def interrupt(_node, timeout_sec):
        assert timeout_sec == 0.1
        if kind in ('sigterm', 'sigint'):
            signal.raise_signal(signal.SIGTERM if kind == 'sigterm' else signal.SIGINT)
        else:
            raise KeyboardInterrupt
    monkeypatch.setattr(rclpy, 'spin_once', interrupt)
    assert run_controller(lambda: node) == 0
    assert events == ['init', 'cancel', 'stop', 'destroy', 'shutdown']
    assert signal.getsignal(signal.SIGTERM) == previous_term
    assert signal.getsignal(signal.SIGINT) == previous_int


def test_runner_configuration_failure_exits_nonzero(monkeypatch, capsys):
    events = []
    monkeypatch.setattr(rclpy, 'init', lambda **k: None)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: events.append('shutdown'))
    def invalid():
        raise ValueError('no graph route')
    assert run_controller(invalid) == 1
    assert events == ['shutdown'] and 'no graph route' in capsys.readouterr().err
