"""FIFO pickup plans on a validated graph, with a final depot return."""

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


def plan_fifo_mission(graph: Graph, pickup_nodes: Sequence[str]) -> MissionPlan:
    """Plan depot -> ordered pickups -> depot using shortest_path per leg.

    Supply a Graph from graph_from_data/load_graph and an ordered sequence of
    node identifiers, such as a list or tuple (not a string). Inputs are not
    modified. Invalid requests, unreachable legs or non-finite accumulated
    distance raise ValueError without returning a partial plan.

    This preserves FIFO order; it does not optimize the order, command a robot
    or perform physical pickups. Geometry validation remains the caller's job.
    """
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

    legs, nodes, total = [], [graph.depot], 0.0
    start = graph.depot
    for index, goal in enumerate((*requests, graph.depot) if requests else (), 1):
        try:
            result = shortest_path(graph, start, goal)
        except ValueError as exc:
            raise ValueError(f'mission leg {index} ({start} -> {goal}): {exc}') from exc
        if result is None:
            raise ValueError(f'mission leg {index} ({start} -> {goal}) is unreachable')
        path, distance = result
        total += distance
        if not isfinite(total):
            raise ValueError(f'total mission distance is not finite at leg {index} ({start} -> {goal})')
        legs.append(MissionLeg(start, goal, tuple(path), distance))
        nodes.extend(path[1:])
        start = goal
    return MissionPlan(requests, tuple(legs), tuple(nodes), total)
