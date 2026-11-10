"""Compare depot-return pickup orderings on the same checked graph."""

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time
import xml.etree.ElementTree as ET

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.ordering import nearest_neighbor_order, two_opt_order
from warehouse_planning.roadmap import load_graph, load_obstacles, shortest_path, validate_geometry

EXACT_LIMIT = 8
CASES = (
    ("empty", ()),
    ("single", ("r1c2",)),
    ("repeated_and_depot", ("r2c2", "r0c0", "r2c2")),
    ("comparison", ("r1c2", "r2c0", "r2c4")),
    ("six_requests", ("r3c4", "r1c1", "r2c0", "r3c2", "r1c4", "r0c2")),
)


def _strategy(graph, requests, name):
    if name == "fifo":
        order = tuple(range(len(requests)))
    elif name == "nn":
        order = nearest_neighbor_order(graph, requests)
    elif name == "nn-2opt":
        initial = nearest_neighbor_order(graph, requests)
        order = two_opt_order(graph, requests, initial)
    else:
        raise ValueError(f"unknown strategy: {name}")
    return order, plan_fifo_mission(graph, [requests[i] for i in order])


def exact_reference(graph, requests):
    """Enumerate request-index permutations, using graph shortest distances.

    The optimum is for the supplied graph and includes the depot return.
    Duplicate destinations remain separate original requests. The limit bounds
    factorial work; no physical path or execution-time optimum is asserted.
    """
    if not isinstance(requests, (tuple, list)):
        raise ValueError("requests must be a list or tuple")
    requests = tuple(requests)
    if len(requests) > EXACT_LIMIT:
        raise ValueError(f"exact reference supports at most {EXACT_LIMIT} requests")
    # Validate all nodes and connectivity with the public planner first.
    plan_fifo_mission(graph, requests)
    nodes = tuple(sorted(set((graph.depot, *requests))))
    distances = {}
    for a in nodes:
        for b in nodes:
            result = shortest_path(graph, a, b)
            if result is None:
                raise ValueError(f"unreachable reference leg: {a} -> {b}")
            distances[a, b] = result[1]
    best = None
    for order in itertools.permutations(range(len(requests))):
        tour = (graph.depot, *(requests[i] for i in order), graph.depot)
        length = math.fsum(distances[a, b] for a, b in zip(tour, tour[1:]))
        if not math.isfinite(length):
            raise ValueError("reference tour length is not finite")
        if best is None or length < best[0]:
            best = (length, order)
    return {"distance_m": best[0], "request_ids": [i + 1 for i in best[1]],
            "permutations_evaluated": math.factorial(len(requests))}


def compare(graph, requests, repeat=5, exact=False, clock_ns=time.perf_counter_ns):
    """Return full plan and wall-time statistics for each strategy.

    One untimed warmup per strategy precedes repeat timed full plan
    constructions. NN+2-opt includes NN initialization. No robot is controlled.
    """
    if type(repeat) is not int or not 1 <= repeat <= 100:
        raise ValueError("repeat must be an integer between 1 and 100")
    if isinstance(requests, (str, bytes)) or not isinstance(requests, (tuple, list)):
        raise ValueError("requests must be a list or tuple")
    requests = tuple(requests)
    if exact and len(requests) > EXACT_LIMIT:
        raise ValueError(f"exact reference supports at most {EXACT_LIMIT} requests")
    results = []
    for name in ("fifo", "nn", "nn-2opt"):
        _strategy(graph, requests, name)
        times = []
        for _ in range(repeat):
            start = clock_ns()
            order, plan = _strategy(graph, requests, name)
            elapsed = clock_ns() - start
            if elapsed < 0:
                raise ValueError("measurement clock went backwards")
            times.append(elapsed)
        results.append({
            "strategy": name, "request_ids": [i + 1 for i in order],
            "pickup_nodes": list(plan.pickup_nodes), "route_nodes": list(plan.nodes),
            "leg_distances_m": [leg.distance_m for leg in plan.legs],
            "distance_m": plan.total_distance_m,
            "planning_time_ns_samples": times,
            "planning_time_ns_median": statistics.median(times),
        })
    reference = exact_reference(graph, requests) if exact else None
    if reference is not None:
        minimum = reference["distance_m"]
        for item in results:
            delta = item["distance_m"] - minimum
            if delta < 0 and not math.isclose(item["distance_m"], minimum, rel_tol=1e-12, abs_tol=1e-9):
                raise ValueError("strategy result shorter than exhaustive reference")
            if math.isclose(item["distance_m"], minimum, rel_tol=1e-12, abs_tol=1e-9):
                delta = 0.0
            item["distance_gap_m"] = delta
            percent = 100 * (delta / minimum) if minimum else None
            if percent is not None and not math.isfinite(percent):
                raise ValueError("distance gap percentage is not finite")
            item["distance_gap_percent"] = percent
    return {"requests": list(requests), "repeat": repeat, "reference": reference,
            "strategies": results}


def _checked_graph(path):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

    try:
        graph_path = path or (Path(get_package_share_directory("warehouse_planning"))
                              / "config" / "warehouse_graph.json")
        graph_path = Path(graph_path)
        sdf_path = Path(get_package_share_directory("warehouse_sim")) / "worlds" / "warehouse.sdf"
    except PackageNotFoundError as error:
        raise ValueError(f"installed warehouse package missing: {error}") from error
    graph = load_graph(graph_path)
    validate_geometry(graph, load_obstacles(sdf_path))
    return graph, hashlib.sha256(graph_path.read_bytes()).hexdigest()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pickups", nargs="*", metavar="NODE")
    group.add_argument("--suite", action="store_true", help="Run the fixed documented cases")
    parser.add_argument("--repeat", type=int, default=5, help="Timed runs per strategy (1-100)")
    parser.add_argument("--exact", action="store_true", help="Enumerate request orderings (max 8)")
    parser.add_argument("--graph", type=Path, help="Override installed graph; validate against installed world")
    parser.add_argument("--json", action="store_true", help="Output one machine-readable JSON document")
    options = parser.parse_args(args)
    try:
        graph, graph_hash = _checked_graph(options.graph)
        cases = CASES if options.suite else (("custom", tuple(options.pickups)),)
        comparisons = [dict(name=name, **compare(graph, requests, options.repeat, options.exact))
                       for name, requests in cases]
        report = {"schema_version": 1, "frame": graph.frame, "depot": graph.depot,
                  "graph_sha256": graph_hash, "python_version": platform.python_version(),
                  "timing": "perf_counter_ns; median of complete plan construction after one warmup",
                  "cases": comparisons}
    except (ValueError, OSError, ET.ParseError, OverflowError) as error:
        print(f"Comparison failed: {error}", file=sys.stderr)
        return 1
    if options.json:
        print(json.dumps(report, indent=2, allow_nan=False))
    else:
        print(f"Graph frame: {report['frame']}; depot: {report['depot']}; SHA-256: {graph_hash}")
        print("Planning time: median ns including full plan; not robot travel time")
        for case in comparisons:
            print(f"\nCase {case['name']}: {case['requests']}")
            if case["reference"] is not None:
                print(f"  Exact reference: {case['reference']['distance_m']:g} m; "
                      f"request IDs {case['reference']['request_ids']}; "
                      f"permutations {case['reference']['permutations_evaluated']}")
            for item in case["strategies"]:
                gap = (f"; gap {item['distance_gap_m']:g} m"
                       if case["reference"] is not None else "")
                print(f"  {item['strategy']}: {item['distance_m']:g} m; "
                      f"IDs {item['request_ids']}; median {item['planning_time_ns_median']:g} ns{gap}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
