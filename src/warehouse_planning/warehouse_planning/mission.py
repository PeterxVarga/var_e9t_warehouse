"""Ordered pickup plans on a validated graph, with a final depot return."""

from collections.abc import Sequence
from dataclasses import dataclass
from math import isfinite

from warehouse_planning.roadmap import Graph, shortest_path


@dataclass(frozen=True)
class MissionLeg:
    """One graph route, including both endpoints; distance is in metres."""

    start: str
    goal: str
    nodes: tuple[str, ...]
    distance_m: float


@dataclass(frozen=True)
class MissionPlan:
    """Immutable request order, leg boundaries and concatenated graph route.

    Leg i serves pickup_nodes[i]; the last leg returns to the depot. An empty
    mission has no legs. Passing through a later pickup is only graph travel,
    not early fulfillment of that request. Repeated requests retain separate
    legs even when their endpoints coincide.
    """

    pickup_nodes: tuple[str, ...]
    legs: tuple[MissionLeg, ...]
    nodes: tuple[str, ...]
    total_distance_m: float


@dataclass(frozen=True)
class RemainingMissionPlan:
    """Immutable remaining route with original request identities preserved.

    Leg i serves original_requests[remaining_order[i]]; the final leg returns
    to the depot. With no remaining requests, only a return leg is needed,
    unless already at the depot, when there are no legs. Zero-length service
    legs remain separate. The first route node is the explicit start node.
    """

    original_requests: tuple[str, ...]
    remaining_order: tuple[int, ...]
    legs: tuple[MissionLeg, ...]
    nodes: tuple[str, ...]
    total_distance_m: float


def plan_fifo_mission(graph: Graph, pickup_nodes: Sequence[str]) -> MissionPlan:
    """Plan depot -> ordered pickups -> depot using shortest_path per leg.

    Supply a Graph from graph_from_data/load_graph and an ordered sequence of
    node identifiers, such as a list or tuple (not a string). Inputs are not
    modified. Invalid requests, unreachable legs or non-finite accumulated
    distance raise ValueError without returning a partial plan.

    This preserves FIFO order; it does not optimize the order, command a robot
    or perform physical pickups. Geometry validation remains the caller's job.
    """
    requests = _validated_requests(graph, pickup_nodes)
    goals = (*requests, graph.depot) if requests else ()
    legs, nodes, total = _plan_legs(graph, graph.depot, goals)
    return MissionPlan(requests, legs, nodes, total)


def plan_remaining_mission(graph: Graph, start_node: str,
                           original_requests: Sequence[str],
                           remaining_order: Sequence[int]) -> RemainingMissionPlan:
    """Plan start -> remaining requests in supplied order -> depot.

    Supply a validated Graph (possibly from graph_without_edges), an existing
    start node, the full original request sequence, and unique original
    zero-based request indices in the service order to retain. Sequences must
    not be strings; indices must be integers, excluding bool. All input is
    validated before route search, including requests omitted from the order.
    Invalid input, unreachable legs or non-finite length raise ValueError
    without a partial plan or direct-route fallback. Inputs are unchanged.

    The caller decides which requests remain: neither the current node nor
    passing through a later goal implies service. No ordering optimization,
    odometry matching, intermediate-position handling, robot stopping or live
    controller-plan replacement is performed. Check geometry separately.
    """
    requests = _validated_requests(graph, original_requests)
    if not isinstance(start_node, str) or start_node not in graph.nodes:
        raise ValueError('start_node must be an existing graph node identifier')
    if (not isinstance(remaining_order, Sequence)
            or isinstance(remaining_order, (str, bytes, bytearray))):
        raise ValueError('remaining_order must be an ordered sequence of request indices')
    order = tuple(remaining_order)
    if (any(type(i) is not int or i < 0 or i >= len(requests) for i in order)
            or len(set(order)) != len(order)):
        raise ValueError('remaining_order must contain unique valid request indices (integers, not bool)')
    goals = (*(requests[i] for i in order), graph.depot) if order or start_node != graph.depot else ()
    legs, nodes, total = _plan_legs(graph, start_node, goals)
    return RemainingMissionPlan(requests, order, legs, nodes, total)


def _validated_requests(graph, pickup_nodes):
    """Snapshot every request, retaining the FIFO validation contract."""
    if not isinstance(graph, Graph):
        raise ValueError('graph must be a validated Graph from graph_from_data or load_graph')
    if not isinstance(graph.depot, str) or graph.depot not in graph.nodes:
        raise ValueError('graph must contain its depot node')
    if (not isinstance(pickup_nodes, Sequence)
            or isinstance(pickup_nodes, (str, bytes, bytearray))):
        raise ValueError('pickup_nodes must be an ordered sequence of node identifiers')
    requests = tuple(pickup_nodes)
    for index, node in enumerate(requests, 1):
        if not isinstance(node, str) or not node.strip():
            raise ValueError(f'pickup request {index} must be a non-empty node identifier')
        if node not in graph.nodes:
            raise ValueError(f'unknown pickup node in request {index}: {node}')
    return requests


def _plan_legs(graph, start, goals):
    """Build explicit service/return boundaries with the shared pathfinder."""
    legs, nodes, total = [], [start], 0.0
    for index, goal in enumerate(goals, 1):
        try:
            result = shortest_path(graph, start, goal)
        except ValueError as exc:
            raise ValueError(f'mission leg {index} ({start} -> {goal}): {exc}') from exc
        if result is None:
            raise ValueError(f'mission leg {index} ({start} -> {goal}) is unreachable')
        path, distance = result
        if type(distance) not in (int, float) or not isfinite(distance) or distance < 0:
            raise ValueError(f'mission leg {index} ({start} -> {goal}) distance must be finite and nonnegative')
        total += distance
        if not isfinite(total):
            raise ValueError(f'total mission distance is not finite at leg {index} ({start} -> {goal})')
        legs.append(MissionLeg(start, goal, tuple(path), distance))
        nodes.extend(path[1:])
        start = goal
    return tuple(legs), tuple(nodes), total
