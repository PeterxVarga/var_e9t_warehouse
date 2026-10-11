"""Hand-computed FIFO routes, request boundaries and immutable plan snapshots."""

from dataclasses import FrozenInstanceError, replace
from math import isfinite
from pathlib import Path

import pytest

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.roadmap import graph_from_data, load_graph


GRAPH = Path(__file__).resolve().parents[1] / 'config/warehouse_graph.json'


def make_graph(points, edges):
    return graph_from_data(dict(
        schema_version=1, frame='world', depot=points[0][0],
        nodes=[dict(id=node, x=x, y=y) for node, x, y in points], edges=edges))


@pytest.fixture
def line():
    return make_graph([('depot', 0, 0), ('near', 1, 0), ('far', 3, 0)],
                      [['depot', 'near'], ['near', 'far']])


@pytest.mark.parametrize('requests,expected', [
    (['a', 'b'], [('depot', 'a', ('depot', 'a'), 3),
                  ('a', 'b', ('a', 'b'), 4), ('b', 'depot', ('b', 'depot'), 5)]),
    (('b', 'a'), [('depot', 'b', ('depot', 'b'), 5),
                  ('b', 'a', ('b', 'a'), 4), ('a', 'depot', ('a', 'depot'), 3)]),
])
def test_hand_computed_order_distances_and_boundaries(requests, expected):
    graph = make_graph([('depot', 0, 0), ('a', 0, 3), ('b', 4, 3)],
                       [['depot', 'a'], ['a', 'b'], ['depot', 'b']])
    plan = plan_fifo_mission(graph, requests)
    assert plan.pickup_nodes == tuple(requests)
    assert [(leg.start, leg.goal, leg.nodes, leg.distance_m) for leg in plan.legs] == expected
    assert plan.nodes == ('depot', *requests, 'depot')
    assert plan.total_distance_m == 12 and isfinite(plan.total_distance_m)


def test_each_leg_uses_weighted_shortest_path():
    graph = make_graph([('depot', 0, 0), ('detour', 0, 10), ('a', 1, 0),
                        ('b', 2, 0), ('goal', 3, 0)],
                       [['depot', 'detour'], ['detour', 'goal'],
                        ['depot', 'a'], ['a', 'b'], ['b', 'goal']])
    plan = plan_fifo_mission(graph, ['goal'])
    assert plan.legs[0].nodes == ('depot', 'a', 'b', 'goal')
    assert plan.legs[1].nodes == ('goal', 'b', 'a', 'depot')
    assert [leg.distance_m for leg in plan.legs] == [3, 3]
    assert plan.nodes == ('depot', 'a', 'b', 'goal', 'b', 'a', 'depot')
    assert plan.total_distance_m == 6


def test_passing_later_pickup_does_not_complete_it_early(line):
    plan = plan_fifo_mission(line, ['far', 'near'])
    assert plan.pickup_nodes == ('far', 'near')
    assert [leg.goal for leg in plan.legs] == ['far', 'near', 'depot']
    assert plan.legs[0].nodes == ('depot', 'near', 'far')
    assert plan.legs[1].nodes == ('far', 'near')
    assert plan.nodes == ('depot', 'near', 'far', 'near', 'depot')
    assert [leg.distance_m for leg in plan.legs] == [3, 2, 1]
    assert plan.total_distance_m == 6


@pytest.mark.parametrize('requests', [[], ()])
def test_empty_mission(line, requests):
    plan = plan_fifo_mission(line, requests)
    assert plan.pickup_nodes == () and plan.legs == ()
    assert plan.nodes == ('depot',) and plan.total_distance_m == 0


def test_repeated_requests_and_depot_pickups_keep_zero_length_legs(line):
    requests = ['depot', 'near', 'near', 'depot']
    plan = plan_fifo_mission(line, requests)
    assert plan.pickup_nodes == tuple(requests)
    assert [(leg.start, leg.goal, leg.nodes, leg.distance_m) for leg in plan.legs] == [
        ('depot', 'depot', ('depot',), 0),
        ('depot', 'near', ('depot', 'near'), 1),
        ('near', 'near', ('near',), 0),
        ('near', 'depot', ('near', 'depot'), 1),
        ('depot', 'depot', ('depot',), 0),
    ]
    assert plan.nodes == ('depot', 'near', 'depot')
    assert plan.total_distance_m == 2


@pytest.mark.parametrize('requests', [None, 'near', b'near', bytearray(b'near'),
                                     1, True, {'near'}, {'near': 1}, iter(['near'])])
def test_invalid_request_container(line, requests):
    with pytest.raises(ValueError, match='ordered sequence'):
        plan_fifo_mission(line, requests)


@pytest.mark.parametrize('node', [None, True, 1, [], {}, '', '  ', 'missing'])
def test_invalid_or_unknown_pickup(line, node):
    requests = ['near', node]
    with pytest.raises(ValueError, match='request 2'):
        plan_fifo_mission(line, requests)
    assert requests[0] == 'near' and requests[1] is node


@pytest.mark.parametrize('graph', [None, {}, 'graph', object()])
def test_invalid_graph(graph):
    with pytest.raises(ValueError, match='validated Graph'):
        plan_fifo_mission(graph, [])


def test_missing_depot_is_invalid_even_for_empty_mission(line):
    with pytest.raises(ValueError, match='depot node'):
        plan_fifo_mission(replace(line, depot='missing'), [])


@pytest.mark.parametrize('requests,leg', [(['island'], 1), (['near', 'island'], 2)])
def test_unreachable_leg_fails_without_partial_plan(requests, leg):
    graph = make_graph([('depot', 0, 0), ('near', 1, 0), ('island', 3, 0)],
                       [['depot', 'near']])
    snapshot = (dict(graph.nodes), graph.edges, dict(graph.adjacency), list(requests))
    with pytest.raises(ValueError, match=rf'mission leg {leg} \(.* -> island\) is unreachable'):
        plan_fifo_mission(graph, requests)
    assert (dict(graph.nodes), graph.edges, dict(graph.adjacency), requests) == snapshot


def test_total_distance_overflow_is_rejected():
    graph = make_graph([('depot', 0, 0), ('far', 1e308, 0)], [['depot', 'far']])
    with pytest.raises(ValueError, match='total mission distance is not finite at leg 2'):
        plan_fifo_mission(graph, ['far'])


def test_leg_distance_overflow_has_leg_context():
    graph = make_graph([('depot', -1e308, 0), ('middle', 0, 0), ('far', 1e308, 0)],
                       [['depot', 'middle'], ['middle', 'far']])
    with pytest.raises(ValueError, match=r'mission leg 1 \(depot -> far\): .*not finite'):
        plan_fifo_mission(graph, ['far'])


def test_inputs_unchanged_and_result_deeply_immutable(line):
    requests = ['far', 'near']
    before = (dict(line.nodes), line.edges, dict(line.adjacency), list(requests))
    plan = plan_fifo_mission(line, requests)
    assert (dict(line.nodes), line.edges, dict(line.adjacency), requests) == before
    requests[:] = ['depot']
    assert plan.pickup_nodes == ('far', 'near')
    for field in ('pickup_nodes', 'legs', 'nodes', 'total_distance_m'):
        with pytest.raises(FrozenInstanceError):
            setattr(plan, field, None)
    for field in ('start', 'goal', 'nodes', 'distance_m'):
        with pytest.raises(FrozenInstanceError):
            setattr(plan.legs[0], field, None)
    for value in (plan.pickup_nodes, plan.legs, plan.nodes, plan.legs[0].nodes):
        with pytest.raises(TypeError):
            value[0] = None
    assert isinstance(hash(plan), int)


def test_real_warehouse_fifo_with_depot_return():
    graph = load_graph(GRAPH)
    plan = plan_fifo_mission(graph, ['r2c2', 'r1c4'])
    assert plan.pickup_nodes == ('r2c2', 'r1c4')
    assert [(leg.start, leg.goal) for leg in plan.legs] == [
        ('r0c0', 'r2c2'), ('r2c2', 'r1c4'), ('r1c4', 'r0c0')]
    assert [leg.distance_m for leg in plan.legs] == [8, 6, 10]
    assert plan.total_distance_m == 24 and isfinite(plan.total_distance_m)
    assert plan.nodes == (
        'r0c0', 'r1c0', 'r2c0', 'r2c1', 'r2c2', 'r2c3', 'r2c4',
        'r1c4', 'r0c4', 'r0c3', 'r0c2', 'r0c1', 'r0c0')
    assert plan.nodes == (plan.legs[0].nodes + plan.legs[1].nodes[1:]
                          + plan.legs[2].nodes[1:])
