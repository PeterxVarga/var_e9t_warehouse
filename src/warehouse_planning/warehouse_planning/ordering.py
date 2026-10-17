"""Deterministic graph-distance ordering of distinct pickup requests."""

from collections.abc import Sequence
from math import fsum, isfinite

from warehouse_planning.roadmap import Graph, shortest_path


def nearest_neighbor_order(graph: Graph, pickup_nodes: Sequence[str]) -> tuple[int, ...]:
    """Return original zero-based request indices in greedy service order.

    Start at the depot and choose the remaining request with the smallest
    Dijkstra graph distance from the current node. Exact distance ties retain
    input order. Duplicate nodes and depot requests remain separate requests.
    The returned tuple is a permutation of range(len(pickup_nodes)); neither
    the graph nor the request sequence is modified.

    Supply a Graph from graph_from_data/load_graph and an ordered sequence of
    node identifiers. Invalid input, unreachable requests or a non-finite
    Dijkstra distance raise ValueError without returning a partial order.
    Geometry validation remains the caller's responsibility.

    This heuristic does not guarantee an optimal tour or improvement over
    FIFO, and does not consider the depot return when choosing each pickup.
    Use plan_fifo_mission with the reordered requests to construct and validate
    the complete route, including its return leg and finite total distance.
    """
    requests = _validated_requests(graph, pickup_nodes)

    remaining = list(range(len(requests)))
    order = []
    current = graph.depot
    while remaining:
        candidates = []
        for index in remaining:
            goal = requests[index]
            try:
                result = shortest_path(graph, current, goal)
            except ValueError as exc:
                raise ValueError(f'pickup request {index + 1} ({current} -> {goal}): {exc}') from exc
            if result is None:
                raise ValueError(f'pickup request {index + 1} ({current} -> {goal}) is unreachable')
            candidates.append((result[1], index))
        _, chosen = min(candidates)
        order.append(chosen)
        remaining.remove(chosen)
        current = requests[chosen]
    return tuple(order)


def _validated_requests(graph, pickup_nodes):
    """Share the existing nearest-neighbor input contract without mutation."""
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


def _tour_distance(graph, requests, order):
    """Evaluate every leg, including both depot connections, before comparison."""
    if not order:
        return 0.0
    nodes = (graph.depot, *(requests[i] for i in order), graph.depot)
    distances = []
    for start, goal in zip(nodes, nodes[1:]):
        # Graph is undirected. Fix the Dijkstra direction so a reversed leg
        # uses the same floating-point distance as its forward counterpart.
        a, b = sorted((start, goal))
        try:
            result = shortest_path(graph, a, b)
        except ValueError as exc:
            raise ValueError(f'tour leg ({start} -> {goal}): {exc}') from exc
        if result is None:
            raise ValueError(f'tour leg ({start} -> {goal}) is unreachable')
        distance = result[1]
        try:
            valid = type(distance) in (int, float) and isfinite(distance) and distance >= 0
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'tour leg ({start} -> {goal}) distance must be finite and nonnegative')
        distances.append(distance)
    try:
        total = fsum(distances)
    except OverflowError:
        raise ValueError('total tour distance is not finite') from None
    if not isfinite(total):
        raise ValueError('total tour distance is not finite')
    return total


def two_opt_order(graph: Graph, pickup_nodes: Sequence[str],
                  initial_order=None) -> tuple[int, ...]:
    """Improve a complete depot tour by deterministic first-improvement 2-opt.

    Return original request indices, starting with FIFO unless initial_order
    supplies a permutation of every index (integers, excluding bool). Inputs
    are snapshots; repeated nodes and depot requests retain distinct indices.
    The graph/request contract is the same as nearest_neighbor_order.

    Scan inclusive reversals (left, right) with left ascending from zero and
    right ascending from left + 1. Evaluate the entire depot-return tour for
    every candidate; accept only a strictly shorter finite length and restart
    the scan. Lengths use Dijkstra in canonical endpoint order on the undirected
    graph and math.fsum, avoiding summation-order changes on equal tours.

    The result is a reversal-local optimum, not necessarily a global optimum.
    It cannot worsen its supplied starting tour; an NN start need not beat
    FIFO. Invalid input, unreachable legs or non-finite distances raise
    ValueError without a partial result. Validate obstacle geometry separately.
    """
    requests = _validated_requests(graph, pickup_nodes)
    count = len(requests)
    if initial_order is None:
        order = tuple(range(count))
    else:
        if (not isinstance(initial_order, Sequence)
                or isinstance(initial_order, (str, bytes, bytearray))):
            raise ValueError('initial_order must be an ordered sequence of request indices')
        order = tuple(initial_order)
        if (len(order) != count or any(type(i) is not int for i in order)
                or set(order) != set(range(count))):
            raise ValueError('initial_order must contain every request index exactly once (integers, not bool)')
    current_distance = _tour_distance(graph, requests, order)
    while True:
        for left in range(count - 1):
            for right in range(left + 1, count):
                candidate = order[:left] + order[left:right + 1][::-1] + order[right + 1:]
                distance = _tour_distance(graph, requests, candidate)
                if distance < current_distance:
                    order, current_distance = candidate, distance
                    break
            else:
                continue
            break
        else:
            return order
