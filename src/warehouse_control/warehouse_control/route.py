"""Validated startup frame transform and ordered waypoint tracking."""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin
import xml.etree.ElementTree as ET

from warehouse_control.control import goal_command
from warehouse_planning.roadmap import (
    load_graph, load_obstacles, shortest_path, validate_geometry,
)


def _finite(values, count, label):
    try:
        values = tuple(values)
        valid = (len(values) == count and all(type(v) in (float, int) for v in values)
                 and all(isfinite(v) for v in values))
    except (TypeError, OverflowError):
        valid = False
    if not valid:
        raise ValueError(f'{label} must contain {count} finite numbers')
    return values


@dataclass(frozen=True)
class PlanarTransform:
    """Known world pose of the odom origin; metres and radians, not localization."""

    x: float
    y: float
    yaw: float

    def __post_init__(self):
        _finite((self.x, self.y, self.yaw), 3, 'transform')

    def world_to_odom(self, point):
        x, y = _finite(point, 2, 'world point')
        dx, dy = x - self.x, y - self.y
        c, s = cos(self.yaw), sin(self.yaw)
        return _finite((c * dx + s * dy, -s * dx + c * dy), 2, 'odom point')

    def odom_to_world(self, point):
        x, y = _finite(point, 2, 'odom point')
        c, s = cos(self.yaw), sin(self.yaw)
        return _finite((self.x + c * x - s * y, self.y + s * x + c * y),
                       2, 'world point')


def load_spawn(sdf_path):
    """Read the one warehouse robot include in the implicit world frame.

    The current obstacle validator additionally rejects rotated SDF poses;
    the reusable planar transform itself supports yaw rotations.
    """
    root = ET.parse(sdf_path).getroot()
    worlds = root.findall('world')
    if (root.tag != 'sdf' or root.get('version') != '1.8' or len(root) != 1
            or len(worlds) != 1 or worlds[0].get('name') != 'warehouse'):
        raise ValueError('expected exactly one warehouse world')
    includes = worlds[0].findall('include')
    if len(includes) != 1:
        raise ValueError('expected exactly one warehouse_robot include')
    include = includes[0]
    if (include.attrib or any(c.tag not in {'uri', 'name', 'pose'} for c in include)
            or len(include.findall('name')) != 1 or len(include.findall('uri')) != 1
            or include.findtext('name') != 'warehouse_robot'
            or include.findtext('uri') != 'model://warehouse_robot'):
        raise ValueError('unsupported robot include')
    poses = include.findall('pose')
    if len(poses) != 1 or poses[0].attrib:
        raise ValueError('expected one robot pose in the implicit world frame')
    try:
        values = tuple(float(part) for part in (poses[0].text or '').split())
    except ValueError:
        raise ValueError('invalid robot spawn pose') from None
    x, y, z, roll, pitch, yaw = _finite(values, 6, 'robot spawn pose')
    if z != 0 or roll != 0 or pitch != 0:
        raise ValueError('robot spawn must be planar at floor height')
    return PlanarTransform(x, y, yaw)


def validate_start(pose):
    """Require fresh-simulation wheel odometry within 0.05 m and 0.05 rad."""
    x, y, yaw = _finite(pose, 3, 'initial odometry')
    if hypot(x, y) > 0.05 or abs(atan2(sin(yaw), cos(yaw))) > 0.05:
        raise ValueError('initial odometry is not at the depot; start a fresh simulation')


def prepare_route(graph_path, sdf_path, goal_id):
    """Check the actual world, plan from its depot and return odom waypoints."""
    graph = load_graph(graph_path)
    validate_geometry(graph, load_obstacles(sdf_path))
    transform = load_spawn(sdf_path)
    if hypot(graph.nodes[graph.depot][0] - transform.x,
             graph.nodes[graph.depot][1] - transform.y) > 1e-9:
        raise ValueError('graph depot does not match the SDF robot spawn')
    result = shortest_path(graph, graph.depot, goal_id)
    if result is None:
        raise ValueError(f'no graph route from {graph.depot} to {goal_id}')
    nodes, cost = result
    points = tuple(transform.world_to_odom(graph.nodes[node]) for node in nodes)
    return RouteProgress(nodes, points), transform, cost


class RouteProgress:
    """Copy waypoint data; advance at most once per call, stopping between legs."""

    def __init__(self, nodes, points):
        self._nodes = tuple(nodes)
        self._points = tuple(_finite(p, 2, 'waypoint') for p in points)
        if (not self._nodes or len(self._nodes) != len(self._points)
                or any(not isinstance(n, str) or not n.strip() for n in self._nodes)):
            raise ValueError('route requires one identifier per waypoint')
        self.index = 0
        self.reached = False

    @property
    def nodes(self):
        return self._nodes

    @property
    def points(self):
        return self._points

    def command(self, pose, config):
        if self.reached:
            return 0.0, 0.0, True
        linear, angular, arrived = goal_command(
            *pose, *self.points[self.index], config)
        if arrived:
            if self.index == len(self.points) - 1:
                self.reached = True
            else:
                self.index += 1
            return 0.0, 0.0, self.reached
        return linear, angular, False
