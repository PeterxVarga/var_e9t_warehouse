"""Readiness, timestamp pairing and fail-closed ROS boundary tests."""
from math import cos, nan, sin
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.exceptions import InvalidParameterTypeException
from sensor_msgs.msg import Imu

from warehouse_control.heading_odometry_node import HeadingNode
import warehouse_control.heading_odometry_node as module


@pytest.fixture
def fixture(monkeypatch):
    state = SimpleNamespace(wall=10., ros=1_000_000_000)
    output = []
    monkeypatch.setattr(module.time, 'monotonic', lambda: state.wall)
    create = HeadingNode.create_publisher

    def capture(node, kind, *args, **kwargs):
        if kind is Odometry:
            return SimpleNamespace(publish=output.append)
        return create(node, kind, *args, **kwargs)

    monkeypatch.setattr(HeadingNode, 'create_publisher', capture)
    rclpy.init(args=[])
    node = HeadingNode()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=state.ros))
    yield node, state, output
    node.destroy_node()
    rclpy.shutdown()


def imu(t=1_000_000_000):
    m = Imu()
    m.header.frame_id = 'warehouse_robot/base_link/imu_heading'
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(t, 1_000_000_000)
    m.orientation.z, m.orientation.w = sin(.15), cos(.15)
    return m


def wheel(t=1_000_000_000, x=0.):
    m = Odometry()
    m.header.frame_id, m.child_frame_id = 'odom', 'warehouse_robot/base_link'
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(t, 1_000_000_000)
    m.pose.pose.position.x, m.pose.pose.orientation.w = x, 1.
    m.twist.twist.linear.x, m.twist.twist.angular.z = .12, .03
    m.twist.covariance[0] = .04
    return m


def activate(n, c, output):
    n.imu(imu(c.ros))
    n.wheel(wheel(c.ros))
    assert n.state == 'WAITING_FOR_SENSORS' and not output
    c.ros += 50_000_000
    n.imu(imu(c.ros))
    n.wheel(wheel(c.ros))
    assert n.state == 'ACTIVE' and len(output) == 2


def test_paired_source_stamps_pose_and_raw_twist_contract(fixture):
    n, c, out = fixture
    activate(n, c, out)
    c.ros += 50_000_000
    m = wheel(c.ros, .01)
    n.wheel(m)
    assert len(out) == 2
    n.imu(imu(c.ros))
    assert len(out) == 3
    assert out[-1].header.stamp.nanosec == 100_000_000
    assert out[-1].pose.pose.position.x == pytest.approx(.01)
    assert out[-1].pose.pose.orientation.w == pytest.approx(1.)
    assert list(out[-1].pose.covariance) == [0.] * 36
    assert out[-1].twist == m.twist
    m.twist.twist.linear.x = 99.
    assert out[-1].twist.twist.linear.x == .12
    n.watchdog()
    assert len(out) == 3


@pytest.mark.parametrize('first', ['wheel', 'imu'])
def test_either_channel_can_start_late_without_premature_fault(fixture, first):
    n, c, out = fixture
    for _ in range(45):
        getattr(n, first)(wheel(c.ros) if first == 'wheel' else imu(c.ros))
        n.watchdog()
        assert n.state == 'WAITING_FOR_SENSORS' and not out
        c.ros += 50_000_000
        c.wall += .05
    for _ in range(4):
        n.imu(imu(c.ros))
        n.wheel(wheel(c.ros))
        c.ros += 50_000_000
    assert n.state == 'ACTIVE' and out


def test_clock_may_start_after_both_sensors(fixture):
    n, c, out = fixture
    c.ros = 0
    for t in [1_000_000_000, 1_050_000_000]:
        n.imu(imu(t))
        n.wheel(wheel(t))
    c.wall += 2.
    n.watchdog()
    assert n.state == 'WAITING_FOR_SENSORS' and not out
    c.ros = 1_100_000_000
    activate(n, c, out)


@pytest.mark.parametrize('channel', ['none', 'wheel', 'imu', 'no_clock'])
def test_startup_deadline_is_wall_based_and_latched(fixture, channel):
    n, c, out = fixture
    if channel == 'wheel':
        n.wheel(wheel())
    elif channel == 'imu':
        n.imu(imu())
    elif channel == 'no_clock':
        c.ros = 0
        n.imu(imu())
        n.wheel(wheel())
    c.wall += 9.99
    n.watchdog()
    assert not n.fault and not out
    c.wall += .02
    n.watchdog()
    assert n.fault and not out
    c.ros = 1_100_000_000
    n.imu(imu(c.ros))
    n.wheel(wheel(c.ros))
    assert n.fault and not out


def test_stale_startup_backlog_does_not_become_fresh_feedback(fixture):
    n, c, out = fixture
    n.imu(imu(100_000_000))
    n.wheel(wheel(100_000_000))
    assert not n.receipts and not out and not n.fault
    activate(n, c, out)


@pytest.mark.parametrize('channel', ['wheel', 'imu'])
def test_loss_and_return_latch_without_stale_output(fixture, channel):
    n, c, out = fixture
    activate(n, c, out)
    c.wall += .51
    c.ros += 50_000_000
    if channel == 'wheel':
        n.imu(imu(c.ros))
    else:
        n.wheel(wheel(c.ros))
    n.watchdog()
    assert n.fault
    count = len(out)
    c.ros += 50_000_000
    n.imu(imu(c.ros))
    n.wheel(wheel(c.ros))
    n.watchdog()
    assert len(out) == count and n.fault


@pytest.mark.parametrize('channel,kind', [(c, k) for c in ['wheel', 'imu'] for k in
                                         ['frame', 'quaternion', 'nonfinite', 'backwards', 'future', 'gap']])
def test_bad_operating_input_stops_output(fixture, channel, kind):
    n, c, out = fixture
    activate(n, c, out)
    c.ros += 50_000_000
    m = wheel(c.ros) if channel == 'wheel' else imu(c.ros)
    if kind == 'frame':
        m.header.frame_id = 'world'
    elif kind == 'quaternion':
        q = m.pose.pose.orientation if channel == 'wheel' else m.orientation
        q.x = q.y = q.z = q.w = 0.
    elif kind == 'nonfinite':
        if channel == 'wheel':
            m.pose.pose.position.x = nan
        else:
            m.angular_velocity.z = nan
    elif kind == 'backwards':
        m.header.stamp.sec = 0
    elif kind == 'future':
        m.header.stamp.sec = 10
    else:
        c.ros += 100_000_000
        m.header.stamp.nanosec += 100_000_000
    getattr(n, channel)(m)
    assert n.fault and len(out) == 2


def test_duplicate_samples_neither_establish_readiness_nor_refresh_active_input(fixture):
    n, c, out = fixture
    for _ in range(3):
        n.imu(imu(c.ros))
        n.wheel(wheel(c.ros))
    assert not out
    c.ros += 50_000_000
    n.imu(imu(c.ros))
    n.wheel(wheel(c.ros))
    assert n.state == 'ACTIVE'
    count = len(out)
    c.wall += .51
    n.imu(imu(c.ros))
    n.wheel(wheel(c.ros))
    n.watchdog()
    assert n.fault and len(out) == count


def test_stopped_and_reversed_clock_faults_after_activation(fixture):
    n, c, out = fixture
    activate(n, c, out)
    n.watchdog()
    c.ros -= 1
    n.watchdog()
    assert n.fault


def test_readonly_startup_deadline(fixture):
    n, _, _ = fixture
    assert n.get_parameter('startup_timeout').value == 10.
    result = n.set_parameters([Parameter('startup_timeout', value=20.)])[0]
    assert not result.successful


@pytest.mark.parametrize('value', ['0.0', '-1.0', '.nan', 'true', '"bad"'])
def test_bad_startup_deadline(value):
    rclpy.init(args=['--ros-args', '-p', 'startup_timeout:=' + value])
    try:
        with pytest.raises((ValueError, InvalidParameterTypeException)):
            HeadingNode()
    finally:
        rclpy.shutdown()
