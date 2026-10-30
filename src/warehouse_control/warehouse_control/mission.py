"""ROS-free preparation and execution of validated, ordered pickup missions."""

from collections.abc import Sequence
from dataclasses import dataclass
from math import hypot

from warehouse_control.control import ControlConfig, goal_command
from warehouse_control.route import (
    PlanarTransform, RouteProgress, _finite, load_spawn, validate_start,
)
from warehouse_planning.mission import plan_fifo_mission, plan_remaining_mission
from warehouse_planning.ordering import nearest_neighbor_order, two_opt_order
from warehouse_planning.roadmap import load_graph, load_obstacles, validate_geometry


SERVICE_NS = 1000000000


@dataclass(frozen=True)
class MissionCheckpoint:
    """Request bookkeeping at pause time, not a physical robot location.

    Orders contain original zero-based indices, preserving separate requests
    at the same node. Only completed service waits belong to completed_order.
    Supply original_requests and remaining_order to plan_remaining_mission
    with a separately chosen start node; this neither replans nor resumes.
    """

    original_requests: tuple[str, ...]
    completed_order: tuple[int, ...]
    remaining_order: tuple[int, ...]


@dataclass(frozen=True)
class MissionEvent:
    """A progress observation, not evidence of mechanical loading."""

    kind: str
    leg: int
    waypoint: int
    node: str
    feedback_stamp: int
    clock_stamp: int
    request_index: int | None = None


def prepare_mission(graph_path, sdf_path, pickup_nodes, strategy='fifo'):
    """Validate the entire installed world/graph and plan before any motion."""
    graph = load_graph(graph_path)
    validate_geometry(graph, load_obstacles(sdf_path))
    transform = load_spawn(sdf_path)
    if hypot(graph.nodes[graph.depot][0] - transform.x,
             graph.nodes[graph.depot][1] - transform.y) > 1e-9:
        raise ValueError('graph depot does not match the SDF robot spawn')
    if not isinstance(strategy, str) or strategy not in ('fifo', 'nn', 'nn-2opt'):
        raise ValueError('strategy must be one of fifo, nn, nn-2opt')
    if (not isinstance(pickup_nodes, Sequence)
            or isinstance(pickup_nodes, (str, bytes, bytearray))):
        raise ValueError('pickup_nodes must be an ordered sequence of node identifiers')
    requests = tuple(pickup_nodes)
    order = tuple(range(len(requests)))
    if strategy != 'fifo':
        order = nearest_neighbor_order(graph, requests)
        if strategy == 'nn-2opt':
            order = two_opt_order(graph, requests, order)
    plan = plan_fifo_mission(graph, [requests[i] for i in order])
    points = tuple(tuple(transform.world_to_odom(graph.nodes[node]) for node in leg.nodes)
                   for leg in plan.legs)
    return MissionProgress(plan, points, original_requests=requests,
                           request_order=order, strategy=strategy), transform


class MissionProgress:
    """One leg at a time; service requires continuous tolerance and fresh time.

    The caller must also validate odometry frames, freshness and quaternion.
    Each call completes at most one request. Both terminal states latch.
    Plan and point data are immutable snapshots; world poses are never inputs.
    leg_index and event leg/waypoint indices refer to the active plan; request
    indices, request_order and completed_requests refer to the original mission.
    """

    def __init__(self, plan, leg_points, *, original_requests=None,
                 request_order=None, strategy='fifo'):
        self.plan = plan
        self._original_requests = tuple(plan.pickup_nodes if original_requests is None
                                        else original_requests)
        self._request_order = tuple(range(len(plan.pickup_nodes)) if request_order is None
                                    else request_order)
        count = len(self._original_requests)
        if (len(self._request_order) != count
                or any(type(i) is not int for i in self._request_order)
                or set(self._request_order) != set(range(count))):
            raise ValueError('request_order must be a permutation of original request indices')
        if tuple(self._original_requests[i] for i in self._request_order) != plan.pickup_nodes:
            raise ValueError('request_order must map original requests to the planned pickups')
        if not isinstance(strategy, str) or strategy not in ('fifo', 'nn', 'nn-2opt'):
            raise ValueError('strategy must be one of fifo, nn, nn-2opt')
        self._strategy = strategy
        self._active_order = self._request_order
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
        self._checkpoint = None

    @property
    def original_requests(self):
        return self._original_requests

    @property
    def request_order(self):
        return self._request_order

    @property
    def strategy(self):
        return self._strategy

    def request_index(self, leg):
        """Map an active-plan leg to its original request; return has no request."""
        return self._active_order[leg] if leg < len(self._active_order) else None

    @property
    def route(self):
        return self._routes[self.leg_index] if self._routes else None

    @property
    def points(self):
        return self._points

    def pause(self) -> MissionCheckpoint:
        """Latch PAUSED and zero the stored command without publishing ROS.

        Only a started TRACKING/SERVICING mission can pause. Other states
        raise RuntimeError unchanged, except PAUSED returns the same frozen
        checkpoint. Partial service time is discarded; arrival alone never
        completes a request. The caller must forward command() results.
        This does not verify physical rest; resume_from_node requires a
        separately supplied graph node and matching fresh odometry.
        """
        if self.state == 'PAUSED':
            return self._checkpoint
        if not self.started or self.state not in ('TRACKING', 'SERVICING'):
            raise RuntimeError('pause requires a started TRACKING or SERVICING mission')
        self._checkpoint = MissionCheckpoint(
            self._original_requests,
            self._request_order[:self.completed_requests],
            self._request_order[self.completed_requests:])
        self.state = 'PAUSED'
        self.service_start = None
        self.last_command = (0.0, 0.0, False)
        return self._checkpoint

    def resume_from_node(self, graph, start_node, transform, pose, config,
                         feedback_stamp, clock_stamp):
        """Atomically replan a started PAUSED mission from an explicit node.

        Supply a geometry-checked Graph and static PlanarTransform from world
        to odom. The finite odom pose must match start_node within config's
        position tolerance; yaw is unrestricted. Both integer nanosecond
        timestamps must be nonnegative and strictly newer than processed time.
        These checks prove neither transport freshness nor physical rest;
        stopping, frame correctness and those checks remain the caller's job.

        plan_remaining_mission retains checkpoint order and original request
        identities without optimization. Build and validate every candidate
        leg before replacing execution state. Invalid input raises ValueError;
        invalid state raises RuntimeError. Failures leave PAUSED unchanged.
        Success returns None, resets active leg_index to zero and clears events
        and partial service. Global completed_requests and request_order remain.
        No command is issued: fresh command() calls execute the replacement
        plan, including an empty plan's stationary depot completion.
        """
        if not self.started or self.state != 'PAUSED':
            raise RuntimeError('resume requires a started PAUSED mission')
        pose = _finite(pose, 3, 'odometry pose')
        if any(type(t) is not int or t < 0 for t in (feedback_stamp, clock_stamp)):
            raise ValueError('mission timestamps must be nonnegative integer nanoseconds')
        if ((self.last_stamp is not None and feedback_stamp <= self.last_stamp)
                or (self.last_clock is not None and clock_stamp <= self.last_clock)):
            raise ValueError('resume requires strictly newer feedback and clock timestamps')
        if not isinstance(transform, PlanarTransform):
            raise ValueError('transform must be a static PlanarTransform')
        if not isinstance(config, ControlConfig):
            raise ValueError('config must be a ControlConfig')
        checkpoint = self._checkpoint
        plan = plan_remaining_mission(graph, start_node, checkpoint.original_requests,
                                      checkpoint.remaining_order)
        start_point = transform.world_to_odom(graph.nodes[start_node])
        if not goal_command(*pose, *start_point, config)[2]:
            raise ValueError('odometry position does not match start_node')
        points = tuple(tuple(transform.world_to_odom(graph.nodes[node])
                             for node in leg.nodes) for leg in plan.legs)
        routes = tuple(RouteProgress(leg.nodes, leg_points)
                       for leg, leg_points in zip(plan.legs, points))

        self.plan = plan
        self._points = points
        self._routes = routes
        self._active_order = plan.remaining_order
        self.leg_index = 0
        self.service_start = None
        self.events = ()
        self.last_stamp, self.last_clock = feedback_stamp, clock_stamp
        self.last_command = (0.0, 0.0, False)
        self._checkpoint = None
        self.state = 'TRACKING'

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
                                     stamp, now, self.request_index(self.leg_index)),)

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
        if self.state == 'PAUSED':
            # Keep pose/time validation active, but never advance execution.
            return 0.0, 0.0, False
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
                                         route.nodes[index], stamp, now,
                                         self.request_index(self.leg_index)),)
        if not command[2]:
            return command
        if self.leg_index < len(self._active_order):
            self.state = 'SERVICING'
            self.service_start = (stamp, now)
            self._event('arrival', stamp, now)
            return 0.0, 0.0, False
        self.state = 'REACHED'
        self._event('depot', stamp, now)
        return 0.0, 0.0, True
