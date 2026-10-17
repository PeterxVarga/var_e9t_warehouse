"""Complete depot tours, deterministic reversals and independent local checks."""

from dataclasses import replace
from itertools import combinations, permutations
from math import fsum, inf, nan, sqrt

import pytest

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.ordering import nearest_neighbor_order, two_opt_order
import warehouse_planning.ordering as ordering_module
from warehouse_planning.roadmap import graph_from_data


def graph(points, edges=None):
    if edges is None:
        edges = [[a[0], b[0]] for a, b in combinations(points, 2)]
    return graph_from_data(dict(schema_version=1, frame='world', depot=points[0][0],
                                nodes=[dict(id=n, x=x, y=y) for n, x, y in points],
                                edges=edges))


@pytest.fixture
def line():
    return graph([('depot', 0, 0), ('a', 1, 0), ('b', -2, 0), ('c', 5, 0)],
                 [['depot', 'a'], ['depot', 'b'], ['a', 'c']])


def grid():
    points = [(f'r{y}c{x}', x, y) for y in range(5) for x in range(5)]
    edges = []
    for name, x, y in points:
        if x < 4:
            edges.append([name, f'r{y}c{x + 1}'])
        if y < 4:
            edges.append([name, f'r{y + 1}c{x}'])
    return graph(points, edges)


def independent_distances(g):
    """Floyd-Warshall oracle, independent of the ordering and Dijkstra code."""
    distances = {(a, b): 0 if a == b else inf for a in g.nodes for b in g.nodes}
    for a, neighbors in g.adjacency.items():
        for b, weight in neighbors:
            distances[a, b] = weight
    for via in g.nodes:
        for a in g.nodes:
            for b in g.nodes:
                distances[a, b] = min(distances[a, b], distances[a, via] + distances[via, b])
    return distances


def tour_cost(g, requests, order, distances):
    nodes = [g.depot, *(requests[i] for i in order), g.depot]
    return fsum(distances[a, b] for a, b in zip(nodes, nodes[1:]))


def assert_local_optimum(g, requests, order, distances):
    cost = tour_cost(g, requests, order, distances)
    # Exhaust every contiguous reversal, rather than following the search's
    # control flow or relying on its private cost helper.
    for left, right in combinations(range(len(order)), 2):
        candidate = (*order[:left], *reversed(order[left:right + 1]), *order[right + 1:])
        assert tour_cost(g, requests, candidate, distances) >= cost


def test_nn_counterexample_improves_16_to_14_and_fifo_stays_optimal(line):
    requests = ('b', 'a', 'c')
    initial = nearest_neighbor_order(line, requests)
    assert initial == (1, 0, 2)
    result = two_opt_order(line, requests, initial)
    assert result == (0, 1, 2)
    assert plan_fifo_mission(line, [requests[i] for i in initial]).total_distance_m == 16
    assert plan_fifo_mission(line, [requests[i] for i in result]).total_distance_m == 14
    assert two_opt_order(line, requests) == two_opt_order(line, requests, (0, 1, 2)) == result


def test_depot_departure_edge_enables_first_improvement():
    g = graph([('depot', 0, 0), ('a', 3, 0), ('b', 1, 0), ('c', 2, 0)])
    requests = ('a', 'b', 'c')
    distances = independent_distances(g)
    assert distances['a', 'b'] + distances['b', 'c'] == 3
    assert distances['b', 'a'] + distances['a', 'c'] == 3
    result = two_opt_order(g, requests)
    assert result == (1, 0, 2)
    assert tour_cost(g, requests, (0, 1, 2), distances) == 8
    assert tour_cost(g, requests, result, distances) == 6


def test_depot_return_edge_enables_suffix_improvement():
    g = graph([('depot', 0, 0), ('a', 5, sqrt(24)), ('b', 4, 0), ('c', 6, 0)])
    requests = ('a', 'b', 'c')
    distances = independent_distances(g)
    assert distances['a', 'b'] == distances['a', 'c'] == 5
    result = two_opt_order(g, requests)
    assert result == (0, 2, 1)
    assert tour_cost(g, requests, (0, 1, 2), distances) == 20
    assert tour_cost(g, requests, result, distances) == 18


def test_fixed_first_improvement_and_restart_order():
    g = grid()
    requests = ('r2c0', 'r2c2', 'r1c0', 'r0c3', 'r4c4')
    # Hand-checked accepted moves: (1,2), then restart to (0,1), then (3,4).
    # Best-improvement selects a different equal-length final permutation.
    assert two_opt_order(g, requests) == (2, 0, 1, 4, 3)


def test_nn_local_optimum_need_not_beat_fifo():
    g = grid()
    requests = ('r1c0', 'r4c2', 'r3c2', 'r1c3', 'r1c4', 'r0c3')
    initial = nearest_neighbor_order(g, requests)
    assert initial == (0, 3, 4, 5, 2, 1)
    result = two_opt_order(g, requests, initial)
    assert result == initial
    distances = independent_distances(g)
    assert tour_cost(g, requests, result, distances) == 18
    assert tour_cost(g, requests, tuple(range(6)), distances) == 16
    assert_local_optimum(g, requests, result, distances)


@pytest.mark.parametrize('requests,initial,expected', [
    ([], None, ()), ((), (), ()), ([], [], ()),
    (['a'], None, (0,)), (['depot'], [0], (0,)),
    (['a', 'b'], None, (0, 1)), (['a', 'b'], [1, 0], (1, 0)),
    (['depot', 'depot'], [1, 0], (1, 0)),
    (['a', 'a'], [1, 0], (1, 0)),
])
def test_empty_small_and_equal_cost_orders_do_not_rearrange(line, requests, initial, expected):
    assert two_opt_order(line, requests, initial) == expected


def test_equal_cost_star_preserves_explicit_identity_order():
    g = graph([('depot', 0, 0), ('a', 1, 0), ('b', -1, 0), ('c', 0, 1)],
              [['depot', 'a'], ['depot', 'b'], ['depot', 'c']])
    assert two_opt_order(g, ['a', 'b', 'c'], (2, 0, 1)) == (2, 0, 1)


def test_repeated_and_depot_requests_keep_separate_indices(line):
    requests = ['b', 'a', 'a', 'depot', 'depot', 'c']
    initial = nearest_neighbor_order(line, requests)
    result = two_opt_order(line, requests, initial)
    assert sorted(result) == list(range(len(requests)))
    plan = plan_fifo_mission(line, [requests[i] for i in result])
    assert len(plan.legs) == len(requests) + 1
    assert plan.pickup_nodes.count('a') == 2 and plan.pickup_nodes.count('depot') == 2
    distances = independent_distances(line)
    assert tour_cost(line, requests, result, distances) <= tour_cost(line, requests, initial, distances)
    assert_local_optimum(line, requests, result, distances)


@pytest.mark.parametrize('initial', [
    True, 1, 1.0, '012', b'012', bytearray(b'012'), {0, 1, 2}, {0: 0}, iter([0, 1, 2]),
    [], [0], [0, 1], [0, 1, 2, 3], [0, 0, 2], [0, 1, 3], [-1, 1, 2],
    [True, 1, 2], [0, False, 2], [0, 1, 2.0], [0, 1, '2'], [0, 1, None], [0, 1, []],
])
def test_invalid_permutation(line, initial):
    with pytest.raises(ValueError, match='initial_order'):
        two_opt_order(line, ['a', 'b', 'c'], initial)


@pytest.mark.parametrize('requests', [None, 'a', b'a', bytearray(b'a'), 1, True, {'a'}, {'a': 1}, iter(['a'])])
def test_invalid_request_container(line, requests):
    with pytest.raises(ValueError, match='ordered sequence'):
        two_opt_order(line, requests)


@pytest.mark.parametrize('node', [None, True, 1, [], {}, '', ' ', 'missing'])
def test_invalid_or_unknown_request(line, node):
    with pytest.raises(ValueError, match='request 2'):
        two_opt_order(line, ['a', node])


@pytest.mark.parametrize('g', [None, {}, 'graph', object()])
def test_invalid_graph_even_for_empty_requests(g):
    with pytest.raises(ValueError, match='validated Graph'):
        two_opt_order(g, [])


def test_invalid_depot_even_for_empty_requests(line):
    with pytest.raises(ValueError, match='depot node'):
        two_opt_order(replace(line, depot='missing'), [])


@pytest.mark.parametrize('initial', [None, (1, 0)])
def test_unreachable_request_has_no_partial_order(initial):
    g = graph([('depot', 0, 0), ('a', 1, 0), ('island', 3, 0)], [['depot', 'a']])
    requests = ['a', 'island']
    with pytest.raises(ValueError, match='unreachable'):
        two_opt_order(g, requests, initial)
    assert requests == ['a', 'island']


def test_nonfinite_dijkstra_route_rejected():
    g = graph([('depot', -1e308, 0), ('middle', 0, 0), ('far', 1e308, 0)],
              [['depot', 'middle'], ['middle', 'far']])
    with pytest.raises(ValueError, match='not finite'):
        two_opt_order(g, ['far'])


def test_finite_legs_with_overflowing_complete_tour_rejected():
    g = graph([('depot', 0, 0), ('far', 1e308, 0)])
    with pytest.raises(ValueError, match='total tour distance is not finite'):
        two_opt_order(g, ['far'])


def test_overflowing_candidate_aborts_without_returning_a_partial_improvement():
    g = graph([('depot', 0, 0), ('a', -4e307, 0), ('b', -2e307, 0),
               ('c', 2e307, 0), ('d', 4e307, 0)])
    requests = ['a', 'b', 'c', 'd']
    assert plan_fifo_mission(g, requests).total_distance_m == pytest.approx(1.6e308)
    with pytest.raises(ValueError, match='total tour distance is not finite'):
        two_opt_order(g, requests)
    assert requests == ['a', 'b', 'c', 'd']


@pytest.mark.parametrize('distance', [nan, inf, -inf, -1.0, True, None, '1', 10**400])
def test_invalid_distance_from_solver_rejected(line, monkeypatch, distance):
    monkeypatch.setattr(ordering_module, 'shortest_path', lambda *args: ([], distance))
    with pytest.raises(ValueError, match='finite and nonnegative'):
        two_opt_order(line, ['a'])


def test_inputs_and_result_snapshot(line):
    requests, initial = ['b', 'a', 'c'], [1, 0, 2]
    before = (dict(line.nodes), line.edges, dict(line.adjacency), requests[:], initial[:])
    result = two_opt_order(line, requests, initial)
    assert (dict(line.nodes), line.edges, dict(line.adjacency), requests, initial) == before
    requests.clear()
    initial.clear()
    assert result == (0, 1, 2)
    with pytest.raises(TypeError):
        result[0] = 1


@pytest.mark.parametrize('initial', list(permutations(range(4))))
def test_independent_local_optimum_nonworsening_and_idempotence(initial):
    g = grid()
    requests = ('r0c3', 'r3c0', 'r3c3', 'r1c1')
    distances = independent_distances(g)
    result = two_opt_order(g, requests, initial)
    assert sorted(result) == list(range(4))
    assert tour_cost(g, requests, result, distances) <= tour_cost(g, requests, initial, distances)
    assert_local_optimum(g, requests, result, distances)
    assert two_opt_order(g, requests, result) == result
    assert two_opt_order(g, requests, list(initial)) == result


def test_graph_record_order_does_not_change_result():
    g = grid()
    points = [(n, *xy) for n, xy in g.nodes.items()]
    reordered = graph([points[0], *reversed(points[1:])],
                      [list(reversed(edge)) for edge in reversed(g.edges)])
    requests = ('r2c0', 'r2c2', 'r1c0', 'r0c3', 'r4c4')
    assert two_opt_order(g, requests) == two_opt_order(reordered, requests) == (2, 0, 1, 4, 3)
