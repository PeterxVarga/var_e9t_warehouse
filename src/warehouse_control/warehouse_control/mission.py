"""ROS-free preparation and ordered execution of validated FIFO missions."""

from dataclasses import dataclass
from math import hypot

from warehouse_control.control import goal_command
from warehouse_control.route import RouteProgress, _finite, load_spawn, validate_start
from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.roadmap import load_graph, load_obstacles, validate_geometry


SERVICE_NS = 1000000000


@dataclass(frozen=True)
class MissionEvent:
    """A progress observation, not evidence of mechanical loading."""

    kind: str
    leg: int
    waypoint: int
    node: str
    feedback_stamp: int
    clock_stamp: int


def prepare_mission(graph_path, sdf_path, pickup_nodes):
    """Validate the entire installed world/graph and plan before any motion."""
    graph = load_graph(graph_path)
    validate_geometry(graph, load_obstacles(sdf_path))
    transform = load_spawn(sdf_path)
    if hypot(graph.nodes[graph.depot][0] - transform.x,
             graph.nodes[graph.depot][1] - transform.y) > 1e-9:
        raise ValueError('graph depot does not match the SDF robot spawn')
    plan = plan_fifo_mission(graph, pickup_nodes)
    points = tuple(tuple(transform.world_to_odom(graph.nodes[node]) for node in leg.nodes)
                   for leg in plan.legs)
    return MissionProgress(plan, points), transform


class MissionProgress:
    """One leg at a time; service requires continuous tolerance and fresh time.

    The caller must also validate odometry frames, freshness and quaternion.
    Each call completes at most one request. Both terminal states latch.
    Plan and point data are immutable snapshots; world poses are never inputs.
    """

    def __init__(self, plan, leg_points):
        self.plan = plan
        self._points = tuple(tuple(_finite(p, 2, 'waypoint') for p in leg)
                             for leg in leg_points)
        if len(self._points) != len(plan.legs):
            raise ValueError('mission requires points for every planned leg')
        # Validate all legs now, including legs that will be followed later.
        self._routes = tuple(RouteProgress(leg.nodes, points)
                             for leg, points in zip(plan.legs, self._points))
        self.leg_index = 0
        self.completed_requests = 0
        self.state = 'TRACKING'
        self.started = False
        self.events = ()
        self.last_stamp = self.last_clock = self.service_start = None
        self.last_command = (0.0, 0.0, False)

    @property
    def route(self):
        return self._routes[self.leg_index] if self._routes else None

    @property
    def points(self):
        return self._points

    def fail(self):
        if self.state != 'REACHED':
            self.state = 'FAULT'
            self.service_start = None
            self.last_command = (0.0, 0.0, False)

    def observe_service_pose(self, pose, config):
        """Invalidate a wait on every observed tolerance departure, between ticks."""
        if self.state == 'SERVICING':
            pose = _finite(pose, 3, 'odometry pose')
            if not goal_command(*pose, *self.route.points[-1], config)[2]:
                self.service_start = None

    def _event(self, kind, stamp, now):
        route = self.route
        self.events += (MissionEvent(kind, self.leg_index,
                                     route.index if route else 0,
                                     route.nodes[route.index] if route else self.plan.nodes[0],
                                     stamp, now),)

    def command(self, pose, config, feedback_stamp, clock_stamp):
        self.events = ()
        if self.state in ('REACHED', 'FAULT'):
            return 0.0, 0.0, self.state == 'REACHED'
        try:
            command = self._command(pose, config, feedback_stamp, clock_stamp)
        except ValueError:
            self.fail()
            raise
        self.last_command = command
        return command

    def _command(self, pose, config, stamp, now):
        pose = _finite(pose, 3, 'odometry pose')
        if any(type(t) is not int or t < 0 for t in (stamp, now)):
            raise ValueError('mission timestamps must be nonnegative integer nanoseconds')
        if ((self.last_stamp is not None and stamp < self.last_stamp)
                or (self.last_clock is not None and now < self.last_clock)):
            raise ValueError('mission time moved backwards')
        clock_advanced = self.last_clock is None or now > self.last_clock
        self.last_clock = now
        if stamp == self.last_stamp or not clock_advanced:
            # Repeated samples cannot advance waypoints or service. Preserve
            # the travel command between feedback messages; paused time stops.
            return self.last_command if clock_advanced else (0.0, 0.0, False)
        self.last_stamp = stamp
        if not self.started:
            validate_start(pose)
            self.started = True
        if not self.plan.legs:
            self.state = 'REACHED'
            self._event('depot', stamp, now)
            return 0.0, 0.0, True
        route = self.route
        if self.state == 'SERVICING':
            linear, angular, arrived = goal_command(*pose, *route.points[-1], config)
            if not arrived:
                if self.service_start is not None:
                    self._event('service_reset', stamp, now)
                self.service_start = None
                return linear, angular, False
            if self.service_start is None:
                self.service_start = (stamp, now)
                self._event('service_restart', stamp, now)
            if min(stamp - self.service_start[0], now - self.service_start[1]) >= SERVICE_NS:
                self._event('serviced', stamp, now)
                self.completed_requests += 1
                self.leg_index += 1
                self.service_start = None
                self.state = 'TRACKING'
            return 0.0, 0.0, False
        index = route.index
        command = route.command(pose, config)
        if route.index != index or command[2]:
            # RouteProgress advances before returning; log the point just met.
            self.events += (MissionEvent('waypoint', self.leg_index, index,
                                         route.nodes[index], stamp, now),)
        if not command[2]:
            return command
        if self.leg_index < len(self.plan.pickup_nodes):
            self.state = 'SERVICING'
            self.service_start = (stamp, now)
            self._event('arrival', stamp, now)
            return 0.0, 0.0, False
        self.state = 'REACHED'
        self._event('depot', stamp, now)
        return 0.0, 0.0, True
