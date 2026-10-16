"""Deterministic nearest-neighbor ordering of distinct pickup requests."""

from collections.abc import Sequence

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
