"""Targeted feasibility arithmetic; no simulation or world-pose feedback."""
from math import cos,pi,sin,sqrt,nan,inf
import pytest
from warehouse_control.heading_odometry import HeadingOdometry,heading,multiply,quaternion,wrap


def qyaw(x):return (0.,0.,sin(x/2),cos(x/2))


def samples(headings):
    c=HeadingOdometry()
    for t,h in enumerate(headings):c.add_imu(t*50000000,h)
    return c


def test_reintegrates_distance_instead_of_replacing_raw_yaw():
    c=samples([0]*5)
    assert c.update(0,0,0,0)==(0,0,0,0)
    c.update(50000000,0,0,.1)
    a=c.update(100000000,.1*cos(.1),.1*sin(.1),.1)
    b=c.update(150000000,.2*cos(.1),.2*sin(.1),.1)
    assert a[1:3]==pytest.approx((.1,0))
    assert b[1:3]==pytest.approx((.2,0))
    assert b[2] != pytest.approx(.2*sin(.1))


def test_reverse_and_turn_only():
    c=samples([0,0,0])
    c.update(0,0,0,0)
    assert c.update(50000000,-.1,0,0)[1:3]==pytest.approx((-.1,0))
    assert c.update(100000000,-.1,0,.2)[1:3]==pytest.approx((-.1,0))


def test_wrap_interpolation_and_integration_use_short_arc():
    c=samples([pi-.01,-pi+.01])
    assert abs(c.at(25000000))==pytest.approx(pi)
    c.update(0,0,0,0)
    out=c.update(50000000,.1,0,0)
    assert out[1:3]==pytest.approx((-.1,0),abs=1e-12)


def test_sensor_mount_and_nonzero_reference():
    mount=multiply((sin(.2),0,0,cos(.2)),qyaw(.3))
    body=qyaw(1.2)
    sensor=multiply(body,mount)
    assert heading(sensor,mount,.7)==pytest.approx(.5)
    assert heading(tuple(-x for x in sensor),mount,.7)==pytest.approx(.5)


@pytest.mark.parametrize('q',[(0,0,0,0),(0,0,0,2),(nan,0,0,1),(inf,0,0,1)])
def test_bad_quaternion(q):
    with pytest.raises(ValueError):quaternion(q)


@pytest.mark.parametrize('t',[-1,True,1.0,None])
def test_bad_time(t):
    with pytest.raises(ValueError):HeadingOdometry().add_imu(t,0)


def test_interpolation_requires_brackets_and_small_gap():
    c=HeadingOdometry(max_imu_gap_ns=30000000);c.add_imu(0,0);c.add_imu(40000000,.1)
    with pytest.raises(ValueError,match='gap'):c.at(20000000)
    with pytest.raises(LookupError):c.at(50000000)


def test_waiting_for_imu_does_not_advance_position():
    c=HeadingOdometry();c.add_imu(0,0);c.update(0,0,0,0)
    with pytest.raises(LookupError):c.update(50000000,.1,0,0)
    assert c.xy==(0,0) and c.previous[0]==0
    c.add_imu(20000000,0);c.add_imu(40000000,0);c.add_imu(60000000,0)
    assert c.update(50000000,.1,0,0)[1:3]==pytest.approx((.1,0))


@pytest.mark.parametrize('t',[0,200000000])
def test_repeated_wheel_and_large_gap_rejected(t):
    c=HeadingOdometry();c.add_imu(0,0);c.add_imu(t or 10000000,0);c.update(0,0,0,0)
    with pytest.raises(ValueError):c.update(t,.1,0,0)


def test_straight_and_curved_midpoint_geometry():
    c=samples([0,pi/2]);c.update(0,0,0,0)
    assert c.update(50000000,1,1,pi/2)[1:3]==pytest.approx((1,1))


@pytest.mark.parametrize('pose',[(nan,0,0),(0,inf,0),(0,0,nan)])
def test_nonfinite_pose_rejected(pose):
    c=samples([0,0]);c.update(0,0,0,0)
    with pytest.raises(ValueError):c.update(50000000,*pose)
