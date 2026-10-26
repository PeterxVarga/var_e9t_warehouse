"""Hand-computed logical closures, immutable snapshots and FIFO integration."""

from copy import deepcopy
from dataclasses import FrozenInstanceError
from itertools import permutations, product
from pathlib import Path

import pytest

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.roadmap import (
    graph_from_data, graph_without_edges, load_graph, shortest_path,
)


GRAPH = Path(__file__).resolve().parents[1] / 'config/warehouse_graph.json'


@pytest.fixture
def graph():
    # A 3-4-5 triangle with a 3 m leaf: a--b--c--d, plus a--c.
    return graph_from_data(dict(
        schema_version=1, frame='world', depot='a',
        nodes=[dict(id=node, x=x, y=y) for node, x, y in
               [('a', 0, 0), ('b', 3, 0), ('c', 3, 4), ('d', 6, 4)]],
        edges=[['c', 'd'], ['b', 'c'], ['a', 'c'], ['a', 'b']]))


@pytest.mark.parametrize('blocked', [[], ()])
def test_no_closures_preserve_graph_and_hand_computed_routes(graph, blocked):
    result = graph_without_edges(graph, blocked)
    assert result is not graph and result == graph
    # Shortest distances read directly from the triangle and its leaf.
    expected = {
        ('a', 'b'): (['a', 'b'], 3),
        ('a', 'c'): (['a', 'c'], 5),
        ('a', 'd'): (['a', 'c', 'd'], 8),
        ('b', 'c'): (['b', 'c'], 4),
        ('b', 'd'): (['b', 'c', 'd'], 7),
        ('c', 'd'): (['c', 'd'], 3),
    }
    for (start, goal), (nodes, cost) in expected.items():
        assert shortest_path(result, start, goal) == (nodes, cost)
        assert shortest_path(result, goal, start) == (list(reversed(nodes)), cost)
    for node in graph.nodes:
        assert shortest_path(result, node, node) == ([node], 0)


def test_detour_closes_both_directions_and_keeps_costs_and_order(graph):
    result = graph_without_edges(graph, [('c', 'a')])
    assert shortest_path(result, 'a', 'c') == (['a', 'b', 'c'], 7)
    assert shortest_path(result, 'c', 'a') == (['c', 'b', 'a'], 7)
    assert shortest_path(result, 'a', 'd') == (['a', 'b', 'c', 'd'], 10)
    assert result.edges == (('a', 'b'), ('b', 'c'), ('c', 'd'))
    assert dict(result.adjacency) == {
        'a': (('b', 3),), 'b': (('a', 3), ('c', 4)),
        'c': (('b', 4), ('d', 3)), 'd': (('c', 3),),
    }
    for node, neighbors in result.adjacency.items():
        for neighbor, cost in neighbors:
            assert (neighbor, cost) in graph.adjacency[node]
    assert result.frame == graph.frame == 'world'
    assert result.depot == graph.depot == 'a'
    assert dict(result.nodes) == dict(graph.nodes)


def test_multiple_closures_allow_isolated_nodes_and_disconnected_graph(graph):
    result = graph_without_edges(graph, [['a', 'b'], ['c', 'a']])
    assert result.edges == (('b', 'c'), ('c', 'd'))
    assert result.adjacency['a'] == ()
    assert set(result.nodes) == {'a', 'b', 'c', 'd'}
    assert shortest_path(result, 'a', 'd') is None
    assert shortest_path(result, 'd', 'a') is None
    assert shortest_path(result, 'a', 'a') == (['a'], 0)
    assert shortest_path(result, 'b', 'd') == (['b', 'c', 'd'], 7)
    empty = graph_without_edges(graph, graph.edges)
    assert empty.edges == ()
    assert dict(empty.adjacency) == {node: () for node in ('a', 'b', 'c', 'd')}
    assert shortest_path(empty, 'b', 'd') is None


def test_closure_order_and_pair_direction_do_not_change_result(graph):
    # Remove the diagonal and leaf; the surviving a--b--c chain costs 7 m.
    for order in permutations((('a', 'c'), ('c', 'd'))):
        for reverse in product((False, True), repeat=2):
            blocked = tuple(pair[::-1] if flip else pair
                            for pair, flip in zip(order, reverse))
            result = graph_without_edges(graph, blocked)
            assert result.edges == (('a', 'b'), ('b', 'c'))
            assert dict(result.adjacency) == {
                'a': (('b', 3),), 'b': (('a', 3), ('c', 4)),
                'c': (('b', 4),), 'd': (),
            }
            assert shortest_path(result, 'a', 'c') == (['a', 'b', 'c'], 7)


@pytest.mark.parametrize('blocked', [
    None, 'ac', b'ac', 1, True, {'a', 'c'}, {'a': 'c'}, iter([('a', 'c')]),
])
def test_invalid_closure_container(graph, blocked):
    with pytest.raises(ValueError, match='list or tuple'):
        graph_without_edges(graph, blocked)


@pytest.mark.parametrize('pair', [None, 'ac', {'a', 'c'}, {'a': 'c'},
                                 [], ['a'], ['a', 'b', 'c']])
def test_invalid_pair_shape(graph, pair):
    with pytest.raises(ValueError, match='pair of node identifiers'):
        graph_without_edges(graph, [pair])


@pytest.mark.parametrize('node', [None, True, 1, [], {}, '', '  ', 'missing'])
def test_invalid_or_unknown_endpoint(graph, node):
    for pair in ([node, 'a'], ['a', node]):
        with pytest.raises(ValueError):
            graph_without_edges(graph, [pair])


@pytest.mark.parametrize('blocked,message', [
    ([['a', 'a']], 'self-loop'),
    ([['a', 'd']], 'does not exist'),
    ([['a', 'c'], ['a', 'c']], 'duplicate undirected'),
    ([['a', 'c'], ('c', 'a')], 'duplicate undirected'),
])
def test_invalid_edges_leave_inputs_unchanged(graph, blocked, message):
    before = deepcopy(blocked)
    snapshot = (graph.frame, graph.depot, dict(graph.nodes), graph.edges,
                dict(graph.adjacency))
    with pytest.raises(ValueError, match=message):
        graph_without_edges(graph, blocked)
    assert blocked == before
    assert (graph.frame, graph.depot, dict(graph.nodes), graph.edges,
            dict(graph.adjacency)) == snapshot


@pytest.mark.parametrize('value', [None, {}, 'graph', object()])
def test_non_graph_input(value):
    with pytest.raises(ValueError, match='validated Graph'):
        graph_without_edges(value, [])


def test_inputs_unchanged_and_result_deeply_immutable(graph):
    blocked = [['a', 'c'], ['c', 'd']]
    before = deepcopy(blocked)
    snapshot = (dict(graph.nodes), graph.edges, dict(graph.adjacency))
    result = graph_without_edges(graph, blocked)
    assert blocked == before
    assert (dict(graph.nodes), graph.edges, dict(graph.adjacency)) == snapshot
    assert shortest_path(graph, 'a', 'd') == (['a', 'c', 'd'], 8)
    blocked[0][:] = ['a', 'b']
    blocked.clear()
    assert result.edges == (('a', 'b'), ('b', 'c'))
    for field in ('frame', 'depot', 'nodes', 'edges', 'adjacency'):
        with pytest.raises(FrozenInstanceError):
            setattr(result, field, None)
    for mapping in (result.nodes, result.adjacency):
        with pytest.raises(TypeError):
            mapping['a'] = ()
    for value in (result.edges, result.edges[0], result.nodes['a'],
                  result.adjacency['a'], result.adjacency['a'][0]):
        with pytest.raises(TypeError):
            value[0] = None


def test_fifo_mission_detour_and_unreachable_later_request(graph):
    closed = graph_without_edges(graph, [('a', 'c')])
    plan = plan_fifo_mission(closed, ['c', 'd'])
    assert plan.pickup_nodes == ('c', 'd')
    assert [(leg.start, leg.goal, leg.nodes, leg.distance_m) for leg in plan.legs] == [
        ('a', 'c', ('a', 'b', 'c'), 7),
        ('c', 'd', ('c', 'd'), 3),
        ('d', 'a', ('d', 'c', 'b', 'a'), 10),
    ]
    assert plan.nodes == ('a', 'b', 'c', 'd', 'c', 'b', 'a')
    assert plan.total_distance_m == 20
    isolated = graph_without_edges(closed, [('c', 'd')])
    with pytest.raises(ValueError, match=r'mission leg 2 \(c -> d\) is unreachable'):
        plan_fifo_mission(isolated, ['c', 'd'])


def test_real_warehouse_detour_and_isolated_depot():
    graph = load_graph(GRAPH)
    assert shortest_path(graph, 'r0c0', 'r0c2') == (['r0c0', 'r0c1', 'r0c2'], 4)
    closed = graph_without_edges(graph, [('r0c0', 'r0c1')])
    # Only outer columns link rows: up, across row 1, down, back along row 0.
    detour = ['r0c0', 'r1c0', 'r1c1', 'r1c2', 'r1c3', 'r1c4',
              'r0c4', 'r0c3', 'r0c2']
    assert shortest_path(closed, 'r0c0', 'r0c2') == (detour, 16)
    assert shortest_path(closed, 'r0c2', 'r0c0') == (list(reversed(detour)), 16)
    plan = plan_fifo_mission(closed, ['r0c2'])
    assert [leg.nodes for leg in plan.legs] == [tuple(detour), tuple(reversed(detour))]
    assert [leg.distance_m for leg in plan.legs] == [16, 16]
    assert plan.total_distance_m == 32
    isolated = graph_without_edges(graph, [('r0c0', 'r0c1'), ('r0c0', 'r1c0')])
    assert isolated.adjacency['r0c0'] == ()
    assert shortest_path(isolated, 'r0c0', 'r0c2') is None
    assert shortest_path(isolated, 'r0c2', 'r0c0') is None
    assert shortest_path(isolated, 'r0c0', 'r0c0') == (['r0c0'], 0)
    with pytest.raises(ValueError, match=r'mission leg 1 \(r0c0 -> r0c2\) is unreachable'):
        plan_fifo_mission(isolated, ['r0c2'])
