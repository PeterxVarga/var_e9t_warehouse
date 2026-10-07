"""Planar proportional goal tracking, independent of ROS message transport."""

from dataclasses import dataclass, fields
from math import atan2, cos, hypot, isfinite, pi, sin


@dataclass(frozen=True)
class ControlConfig:
    """Positive gains, speed limits and position tolerance in SI units."""

    goal_tolerance: float = 0.05
    max_linear_speed: float = 0.15
    max_angular_speed: float = 0.60
    linear_gain: float = 0.5
    angular_gain: float = 1.5
    turn_threshold: float = 0.35

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f'{field.name} must be finite and positive')
        if self.turn_threshold >= pi:
            raise ValueError('turn_threshold must be smaller than pi')


def goal_command(x, y, yaw, target_x, target_y, config):
    """Return (forward m/s, yaw rate rad/s, reached) for an odom-frame goal."""
    if not all(isfinite(v) for v in (x, y, yaw, target_x, target_y)):
        raise ValueError('pose and target must be finite')
    dx, dy = target_x - x, target_y - y
    distance = hypot(dx, dy)
    if not isfinite(distance):
        raise ValueError('goal distance must be finite')
    if distance <= config.goal_tolerance:
        return 0.0, 0.0, True
    direction = atan2(dy, dx) - yaw
    error = atan2(sin(direction), cos(direction))
    angular = max(-config.max_angular_speed,
                  min(config.max_angular_speed, config.angular_gain * error))
    linear = 0.0
    if abs(error) <= config.turn_threshold:
        linear = min(config.max_linear_speed, config.linear_gain * distance)
    return linear, angular, False
