"""Hand-derived remaining missions: identity, closures and explicit node starts."""

from dataclasses import FrozenInstanceError, replace
from math import isfinite
from pathlib import Path

import pytest

from warehouse_planning.mission import (
    MissionLeg, MissionPlan, RemainingMissionPlan, plan_fifo_mission,
    plan_remaining_mission,
)
from warehouse_planning.roadmap import graph_from_data, graph_without_edges, load_graph


def make_graph(points, edges):
    return graph_from_data(dict(
        schema_version=1, frame='world', depot=points[0][0],
        nodes=[dict(id=node, x=x, y=y) for node, x, y in points], edges=edges))


@pytest.fixture
def line():
    return make_graph([('depot', 0, 0), ('near', 1, 0), ('far', 3, 0),
                       ('beyond', 4, 0), ('island', 0, 8)],
                      [['depot', 'near'], ['near', 'far'], ['far', 'beyond']])


def test_non_depot_start_skips_completed_and_retains_non_contiguous_order(line):
    # Request 0 was served. Reaching near does not restore it, and passing far
    # on the way to beyond does not serve request 1 before request 3.
    plan = plan_remaining_mission(line, 'near', ['near', 'far', 'depot', 'beyond'], [3, 1])
    assert isinstance(plan, RemainingMissionPlan)
    assert plan.original_requests == ('near', 'far', 'depot', 'beyond')
    assert plan.remaining_order == (3, 1)
    assert plan.legs == (
        MissionLeg('near', 'beyond', ('near', 'far', 'beyond'), 3),
        MissionLeg('beyond', 'far', ('beyond', 'far'), 1),
        MissionLeg('far', 'depot', ('far', 'near', 'depot'), 3),
    )
    assert plan.nodes == ('near', 'far', 'beyond', 'far', 'near', 'depot')
    assert plan.total_distance_m == 7 and isfinite(plan.total_distance_m)


def test_duplicate_current_and_depot_requests_keep_every_service_leg(line):
    plan = plan_remaining_mission(line, 'near', ['near', 'near', 'depot', 'depot'], [1, 0, 2, 3])
    assert plan.remaining_order == (1, 0, 2, 3)
    assert plan.legs == (
        MissionLeg('near', 'near', ('near',), 0),
        MissionLeg('near', 'near', ('near',), 0),
        MissionLeg('near', 'depot', ('near', 'depot'), 1),
        MissionLeg('depot', 'depot', ('depot',), 0),
        MissionLeg('depot', 'depot', ('depot',), 0),
    )
    assert plan.nodes == ('near', 'depot') and plan.total_distance_m == 1


@pytest.mark.parametrize('requests', [[], ['near', 'island']])
@pytest.mark.parametrize('order', [[], ()])
@pytest.mark.parametrize('start,nodes,distance', [
    ('depot', ('depot',), 0), ('far', ('far', 'near', 'depot'), 3),
])
def test_empty_remaining_is_only_return_if_needed(line, requests, order, start, nodes, distance):
    # Unserved/completed state comes only from order; even an omitted island
    # need not be reachable. Every original identifier must still be valid.
    plan = plan_remaining_mission(line, start, requests, order)
    assert plan.original_requests == tuple(requests) and plan.remaining_order == ()
    assert plan.nodes == nodes and plan.total_distance_m == distance
    assert plan.legs == (() if start == 'depot' else (
        MissionLeg('far', 'depot', nodes, 3),))


def test_closed_edge_requires_weighted_detour_without_mutation():
    graph = make_graph([('depot', 0, 0), ('a', 3, 0), ('b', 3, 4), ('c', 0, 4)],
                       [['depot', 'a'], ['a', 'b'], ['b', 'c'], ['c', 'depot']])
    blocked = [['a', 'b']]
    closed = graph_without_edges(graph, blocked)
    plan = plan_remaining_mission(closed, 'a', ['a', 'b'], [1])
    assert plan.legs == (
        MissionLeg('a', 'b', ('a', 'depot', 'c', 'b'), 10),
        MissionLeg('b', 'depot', ('b', 'c', 'depot'), 7),
    )
    assert plan.nodes == ('a', 'depot', 'c', 'b', 'c', 'depot')
    assert plan.total_distance_m == 17
    assert plan_remaining_mission(graph, 'a', ['a', 'b'], [1]).total_distance_m == 11
    assert ('a', 'b') in graph.edges and ('a', 'b') not in closed.edges
    assert blocked == [['a', 'b']]


@pytest.mark.parametrize('order,leg,start', [([1], 1, 'near'), ([0, 1], 2, 'far')])
def test_unreachable_request_raises_without_fallback(line, order, leg, start):
    with pytest.raises(ValueError, match=rf'mission leg {leg} \({start} -> island\) is unreachable'):
        plan_remaining_mission(line, 'near', ['far', 'island'], order)


@pytest.mark.parametrize('order,leg,start', [([], 1, 'near'), ([0], 2, 'far')])
def test_unreachable_depot_is_checked_separately(line, order, leg, start):
    closed = graph_without_edges(line, [('depot', 'near')])
    with pytest.raises(ValueError, match=rf'mission leg {leg} \({start} -> depot\) is unreachable'):
        plan_remaining_mission(closed, 'near', ['far'], order)


@pytest.mark.parametrize('value', [None, 'near', b'near', bytearray(b'near'),
                                  1, True, {'near'}, {'near': 1}, iter(['near'])])
def test_invalid_original_request_container(line, value):
    with pytest.raises(ValueError, match='ordered sequence'):
        plan_remaining_mission(line, 'depot', value, [])


@pytest.mark.parametrize('value', [None, '', '0', b'0', bytearray(b'0'),
                                  1, True, {0}, {0: 1}, iter([0])])
def test_invalid_remaining_order_container(line, value):
    with pytest.raises(ValueError, match='remaining_order.*ordered sequence'):
        plan_remaining_mission(line, 'depot', ['near'], value)


@pytest.mark.parametrize('order', [[True], [False], [-1], [2], [1.0], ['1'], [None],
                                  [[]], [{}], [0, 0], [0, 1, 0], [0, False], [10**100]])
def test_invalid_original_indices(line, order):
    with pytest.raises(ValueError, match='unique valid request indices'):
        plan_remaining_mission(line, 'depot', ['near', 'near'], order)


@pytest.mark.parametrize('node', [None, True, 1, [], {}, '', '  ', 'missing'])
def test_all_original_requests_validated_before_search_even_if_omitted(line, node, monkeypatch):
    def forbidden_search(*args):
        pytest.fail('route search began before all inputs were validated')

    monkeypatch.setattr('warehouse_planning.mission.shortest_path', forbidden_search)
    with pytest.raises(ValueError, match='request 2'):
        plan_remaining_mission(line, 'depot', ['near', node], [0])


def test_entire_order_validated_before_search(line, monkeypatch):
    def forbidden_search(*args):
        pytest.fail('route search began before the complete order was validated')

    monkeypatch.setattr('warehouse_planning.mission.shortest_path', forbidden_search)
    with pytest.raises(ValueError, match='unique valid request indices'):
        plan_remaining_mission(line, 'depot', ['near', 'far'], [0, 2])


@pytest.mark.parametrize('start', [None, True, 1, [], {}, '', '  ', 'missing'])
def test_invalid_start_even_without_remaining_requests(line, start):
    with pytest.raises(ValueError, match='start_node'):
        plan_remaining_mission(line, start, [], [])


@pytest.mark.parametrize('graph', [None, {}, 'graph', object()])
def test_invalid_graph(graph):
    with pytest.raises(ValueError, match='validated Graph'):
        plan_remaining_mission(graph, 'depot', [], [])


def test_invalid_depot_even_without_remaining_requests(line):
    with pytest.raises(ValueError, match='depot node'):
        plan_remaining_mission(replace(line, depot='missing'), 'near', [], [])


def test_non_finite_total_rejected():
    graph = make_graph([('depot', 0, 0), ('far', 1e308, 0)], [['depot', 'far']])
    with pytest.raises(ValueError, match='total mission distance is not finite at leg 2'):
        plan_remaining_mission(graph, 'depot', ['far'], [0])


def test_non_finite_route_rejected_with_leg_context():
    graph = make_graph([('depot', -1e308, 0), ('middle', 0, 0), ('far', 1e308, 0)],
                       [['depot', 'middle'], ['middle', 'far']])
    with pytest.raises(ValueError, match=r'mission leg 1 \(depot -> far\): .*not finite'):
        plan_remaining_mission(graph, 'depot', ['far'], [0])


def test_snapshots_are_deeply_immutable_and_inputs_preserved_on_success_and_failure(line):
    requests, order = ['far', 'near', 'island'], [0, 1]
    before = (dict(line.nodes), line.edges, dict(line.adjacency), list(requests), list(order))
    plan = plan_remaining_mission(line, 'near', requests, order)
    with pytest.raises(ValueError, match='unreachable'):
        plan_remaining_mission(line, 'near', requests, [0, 2])
    assert (dict(line.nodes), line.edges, dict(line.adjacency), requests, order) == before
    requests[:], order[:] = ['depot'], []
    assert plan.original_requests == ('far', 'near', 'island') and plan.remaining_order == (0, 1)
    for field in ('original_requests', 'remaining_order', 'legs', 'nodes', 'total_distance_m'):
        with pytest.raises(FrozenInstanceError):
            setattr(plan, field, None)
    for field in ('start', 'goal', 'nodes', 'distance_m'):
        with pytest.raises(FrozenInstanceError):
            setattr(plan.legs[0], field, None)
    for value in (plan.original_requests, plan.remaining_order, plan.legs, plan.nodes,
                  plan.legs[0].nodes):
        with pytest.raises(TypeError):
            value[0] = None
    assert isinstance(hash(plan), int)


def test_equal_cost_routes_are_deterministic_independent_of_graph_input_order():
    points = [('depot', 0, 0), ('a', 1, 0), ('b', 0, 1), ('goal', 1, 1)]
    edges = [['depot', 'b'], ['b', 'goal'], ['depot', 'a'], ['a', 'goal']]
    first = make_graph(points, edges)
    second = graph_from_data(dict(
        schema_version=1, frame='world', depot='depot',
        nodes=[dict(id=n, x=x, y=y) for n, x, y in reversed(points)],
        edges=[list(reversed(edge)) for edge in reversed(edges)]))
    for graph in (first, second, first):
        plan = plan_remaining_mission(graph, 'depot', ('goal',), (0,))
        assert plan.nodes == ('depot', 'a', 'goal', 'a', 'depot')
        assert plan.total_distance_m == 4


def test_fifo_regression_preserves_type_boundaries_and_empty_behavior(line):
    expected = MissionPlan(('near', 'near', 'depot'), (
        MissionLeg('depot', 'near', ('depot', 'near'), 1),
        MissionLeg('near', 'near', ('near',), 0),
        MissionLeg('near', 'depot', ('near', 'depot'), 1),
        MissionLeg('depot', 'depot', ('depot',), 0),
    ), ('depot', 'near', 'depot'), 2)
    assert plan_fifo_mission(line, ['near', 'near', 'depot']) == expected
    assert plan_fifo_mission(line, []) == MissionPlan((), (), ('depot',), 0)
    with pytest.raises(ValueError, match=r'^unknown pickup node in request 2: missing$'):
        plan_fifo_mission(line, ['near', 'missing'])


def test_real_warehouse_remainder_with_closure():
    graph = load_graph(Path(__file__).resolve().parents[1] / 'config/warehouse_graph.json')
    closed = graph_without_edges(graph, [('r0c0', 'r0c1')])
    plan = plan_remaining_mission(closed, 'r1c0', ['r1c0', 'r0c2', 'r1c2'], [2, 1])
    # Edges are 2 m; the closure forces the last return up the left column.
    assert plan.original_requests == ('r1c0', 'r0c2', 'r1c2')
    assert plan.remaining_order == (2, 1)
    assert plan.legs == (
        MissionLeg('r1c0', 'r1c2', ('r1c0', 'r1c1', 'r1c2'), 4),
        MissionLeg('r1c2', 'r0c2', ('r1c2', 'r1c3', 'r1c4', 'r0c4', 'r0c3', 'r0c2'), 10),
        MissionLeg('r0c2', 'r0c0', ('r0c2', 'r0c3', 'r0c4', 'r1c4', 'r1c3',
                                  'r1c2', 'r1c1', 'r1c0', 'r0c0'), 16),
    )
    assert plan.nodes == ('r1c0', 'r1c1', 'r1c2', 'r1c3', 'r1c4', 'r0c4', 'r0c3',
                          'r0c2', 'r0c3', 'r0c4', 'r1c4', 'r1c3', 'r1c2', 'r1c1',
                          'r1c0', 'r0c0')
    assert plan.total_distance_m == 30
