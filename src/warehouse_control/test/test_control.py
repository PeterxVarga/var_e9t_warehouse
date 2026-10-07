"""Checks for steering decisions, limits and invalid inputs."""

from dataclasses import replace
from math import inf, nan, pi

import pytest

from warehouse_control.control import ControlConfig, goal_command


CONFIG = ControlConfig()


def command(target_x, target_y, yaw=0.0, config=CONFIG):
    return goal_command(0.0, 0.0, yaw, target_x, target_y, config)


@pytest.mark.parametrize('distance', [0.0, 0.025, 0.05])
def test_arrival_includes_tolerance_boundary(distance):
    assert command(distance, 0.0) == (0.0, 0.0, True)


def test_forward_motion_and_slowdown():
    far = command(2.0, 0.0)
    near = command(0.1, 0.0)
    assert far == (CONFIG.max_linear_speed, 0.0, False)
    assert 0.0 < near[0] < far[0]
    assert not command(0.050001, 0.0)[2]


@pytest.mark.parametrize('x,y,sign', [(0.0, 1.0, 1), (0.0, -1.0, -1), (-1.0, 0.1, 1)])
def test_large_heading_error_rotates_without_forward_motion(x, y, sign):
    linear, angular, reached = command(x, y)
    assert linear == 0.0
    assert angular * sign > 0.0
    assert abs(angular) <= CONFIG.max_angular_speed
    assert not reached


def test_angle_wrap_uses_short_turn():
    linear, angular, reached = command(-1.0, -0.001, yaw=pi - 0.01)
    assert linear > 0.0
    assert 0.0 < angular < 0.1
    assert not reached


def test_small_heading_error_corrects_while_advancing():
    linear, angular, _ = command(1.0, 0.1)
    assert 0.0 < linear <= CONFIG.max_linear_speed
    assert 0.0 < angular <= CONFIG.max_angular_speed


@pytest.mark.parametrize('bad', [nan, inf, -inf])
@pytest.mark.parametrize('index', range(5))
def test_nonfinite_pose_or_target_is_rejected(bad, index):
    values = [0.0, 0.0, 0.0, 1.0, 0.0]
    values[index] = bad
    with pytest.raises(ValueError):
        goal_command(*values, CONFIG)


@pytest.mark.parametrize('field', [
    'goal_tolerance', 'max_linear_speed', 'max_angular_speed',
    'linear_gain', 'angular_gain', 'turn_threshold',
])
@pytest.mark.parametrize('bad', [0.0, -1.0, nan, inf])
def test_invalid_configuration_is_rejected(field, bad):
    with pytest.raises(ValueError):
        replace(CONFIG, **{field: bad})


def test_turn_threshold_must_allow_in_place_rotation():
    with pytest.raises(ValueError):
        replace(CONFIG, turn_threshold=pi)


def test_overflowing_distance_is_rejected():
    with pytest.raises(ValueError):
        goal_command(-1e308, 0.0, 0.0, 1e308, 0.0, CONFIG)
