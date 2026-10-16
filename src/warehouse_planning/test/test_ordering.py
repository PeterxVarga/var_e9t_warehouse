"""Graph-distance ordering, request identity and a greedy counterexample."""

from dataclasses import replace
from pathlib import Path

import pytest

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.ordering import nearest_neighbor_order
from warehouse_planning.roadmap import graph_from_data, load_graph, load_obstacles, validate_geometry


PACKAGE = Path(__file__).resolve().parents[1]


def make_graph(points, edges):
    return graph_from_data(dict(
        schema_version=1, frame='world', depot=points[0][0],
        nodes=[dict(id=node, x=x, y=y) for node, x, y in points], edges=edges))


@pytest.fixture
def line():
    return make_graph([('depot', 0, 0), ('near', 1, 0), ('far', 3, 0)],
                      [['depot', 'near'], ['near', 'far']])


def test_each_choice_uses_the_current_node_not_the_depot():
    graph = make_graph([('depot', 0, 0), ('first', 2, 0), ('next', 3, 0), ('other', -2.5, 0)],
                       [['depot', 'first'], ['first', 'next'], ['depot', 'other']])
    assert nearest_neighbor_order(graph, ['other', 'next', 'first']) == (2, 1, 0)


def test_graph_distance_not_euclidean_proximity():
    graph = make_graph([('depot', 0, 0), ('near_xy', 1, 0), ('detour', 0, 5), ('near_graph', 0, 2)],
                       [['depot', 'detour'], ['detour', 'near_xy'], ['depot', 'near_graph']])
    assert nearest_neighbor_order(graph, ['near_xy', 'near_graph']) == (1, 0)


def test_graph_distance_not_edge_count():
    graph = make_graph([('depot', 0, 0), ('middle', 0.5, 0), ('near', 1, 0), ('far', 0, 2)],
                       [['depot', 'middle'], ['middle', 'near'], ['depot', 'far']])
    assert nearest_neighbor_order(graph, ['far', 'near']) == (1, 0)


@pytest.mark.parametrize('requests', [('z', 'a'), ('a', 'z')])
def test_exact_ties_follow_request_order_not_node_name(requests):
    points = [('depot', 0, 0), ('a', -1, 0), ('z', 1, 0)]
    edges = [['depot', 'z'], ['depot', 'a']]
    for candidate_points, candidate_edges in [(points, edges),
                                             ([points[0], *reversed(points[1:])],
                                              [list(reversed(e)) for e in reversed(edges)])]:
        assert nearest_neighbor_order(make_graph(candidate_points, candidate_edges), requests) == (0, 1)


def test_duplicates_and_depot_requests_keep_original_identity(line):
    requests = ['far', 'near', 'near', 'depot', 'depot']
    order = nearest_neighbor_order(line, requests)
    assert order == (3, 4, 1, 2, 0)
    assert sorted(order) == list(range(len(requests)))
    plan = plan_fifo_mission(line, [requests[i] for i in order])
    assert [leg.distance_m for leg in plan.legs] == [0, 0, 1, 0, 2, 3]
    assert len(plan.legs) == len(requests) + 1


@pytest.mark.parametrize('requests,expected', [([], ()), ((), ()), (['far'], (0,))])
def test_empty_and_single_request(line, requests, expected):
    assert nearest_neighbor_order(line, requests) == expected


def test_nearest_neighbor_can_be_worse_than_fifo():
    graph = make_graph([('depot', 0, 0), ('a', 1, 0), ('b', -2, 0), ('c', 5, 0)],
                       [['depot', 'a'], ['depot', 'b'], ['a', 'c']])
    requests = ['b', 'a', 'c']
    order = nearest_neighbor_order(graph, requests)
    assert order == (1, 0, 2)
    fifo = plan_fifo_mission(graph, requests)
    greedy = plan_fifo_mission(graph, [requests[i] for i in order])
    assert fifo.total_distance_m == 14
    assert greedy.total_distance_m == 16
    assert greedy.legs[-1].goal == graph.depot


def test_nearest_neighbor_can_shorten_a_backtracking_fifo_tour():
    graph = make_graph([('depot', 0, 0), ('near', 1, 0), ('far', 3, 0), ('last', 4, 0)],
                       [['depot', 'near'], ['near', 'far'], ['far', 'last']])
    requests = ['far', 'near', 'last']
    order = nearest_neighbor_order(graph, requests)
    assert order == (1, 0, 2)
    assert plan_fifo_mission(graph, requests).total_distance_m == 12
    assert plan_fifo_mission(graph, [requests[i] for i in order]).total_distance_m == 8


@pytest.mark.parametrize('requests', [None, 'near', b'near', bytearray(b'near'),
                                     1, True, {'near'}, {'near': 1}, iter(['near'])])
def test_invalid_request_container(line, requests):
    with pytest.raises(ValueError, match='ordered sequence'):
        nearest_neighbor_order(line, requests)


@pytest.mark.parametrize('node', [None, True, 1, [], {}, '', '  ', 'missing'])
def test_invalid_or_unknown_request(line, node):
    with pytest.raises(ValueError, match='request 2'):
        nearest_neighbor_order(line, ['near', node])


@pytest.mark.parametrize('graph', [None, {}, 'graph', object()])
def test_invalid_graph_even_for_empty_requests(graph):
    with pytest.raises(ValueError, match='validated Graph'):
        nearest_neighbor_order(graph, [])


def test_invalid_depot_even_for_empty_requests(line):
    with pytest.raises(ValueError, match='depot node'):
        nearest_neighbor_order(replace(line, depot='missing'), [])


def test_disconnected_request_rejected_without_input_changes():
    graph = make_graph([('depot', 0, 0), ('near', 1, 0), ('island', 3, 0)],
                       [['depot', 'near']])
    requests = ['near', 'island']
    with pytest.raises(ValueError, match=r'request 2 \(depot -> island\) is unreachable'):
        nearest_neighbor_order(graph, requests)
    assert requests == ['near', 'island']


def test_non_finite_graph_distance_has_request_context():
    graph = make_graph([('depot', -1e308, 0), ('middle', 0, 0), ('far', 1e308, 0)],
                       [['depot', 'middle'], ['middle', 'far']])
    with pytest.raises(ValueError, match=r'request 1 \(depot -> far\): .*not finite'):
        nearest_neighbor_order(graph, ['far'])


def test_inputs_unchanged_and_result_is_a_snapshot(line):
    requests = ['far', 'near', 'near']
    snapshot = (dict(line.nodes), line.edges, dict(line.adjacency), list(requests))
    order = nearest_neighbor_order(line, requests)
    assert (dict(line.nodes), line.edges, dict(line.adjacency), requests) == snapshot
    assert order == nearest_neighbor_order(line, tuple(requests)) == (1, 2, 0)
    requests.clear()
    assert order == (1, 2, 0)
    with pytest.raises(TypeError):
        order[0] = 0


def test_warehouse_example_and_depot_return():
    graph = load_graph(PACKAGE / 'config/warehouse_graph.json')
    validate_geometry(graph, load_obstacles(PACKAGE.parent / 'warehouse_sim/worlds/warehouse.sdf'))
    requests = ['r1c4', 'r2c2', 'r1c0']
    order = nearest_neighbor_order(graph, requests)
    assert order == (2, 1, 0)
    fifo = plan_fifo_mission(graph, requests)
    greedy = plan_fifo_mission(graph, [requests[i] for i in order])
    assert [leg.distance_m for leg in fifo.legs] == [10, 6, 6, 2]
    assert [leg.distance_m for leg in greedy.legs] == [2, 6, 6, 10]
    assert greedy.total_distance_m == fifo.total_distance_m == 24
