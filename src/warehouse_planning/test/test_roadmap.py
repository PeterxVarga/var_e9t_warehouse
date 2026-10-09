"""Independent path reference, schema checks and real warehouse geometry."""

from copy import deepcopy
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

from warehouse_planning.roadmap import (
    Bounds, graph_from_data, load_graph, load_obstacles, segment_intersects,
    shortest_path, validate_geometry,
)


PACKAGE = Path(__file__).resolve().parents[1]
SDF = PACKAGE.parent / 'warehouse_sim' / 'worlds' / 'warehouse.sdf'
GRAPH = PACKAGE / 'config' / 'warehouse_graph.json'


def data(points=None, edges=None):
    points = points or [('a', 0, 0), ('b', 1, 0), ('c', 2, 0)]
    return dict(schema_version=1, frame='world', depot=points[0][0],
                nodes=[dict(id=i, x=x, y=y) for i, x, y in points],
                edges=[['a', 'b'], ['b', 'c']] if edges is None else edges)


def test_small_paths_and_input_unchanged():
    source = data()
    before = deepcopy(source)
    graph = graph_from_data(source)
    snapshot = (dict(graph.nodes), graph.edges, dict(graph.adjacency))
    assert shortest_path(graph, 'a', 'c') == (['a', 'b', 'c'], 2)
    assert shortest_path(graph, 'c', 'a') == (['c', 'b', 'a'], 2)
    assert shortest_path(graph, 'b', 'b') == (['b'], 0)
    assert source == before
    assert (dict(graph.nodes), graph.edges, dict(graph.adjacency)) == snapshot
    source['nodes'][0]['x'] = 50
    assert graph.nodes['a'] == (0, 0)
    with pytest.raises(TypeError):
        graph.nodes['a'] = (5, 5)


def test_weighted_path_is_not_fewest_edges():
    graph = graph_from_data(data(
        [('a', 0, 0), ('b', 0, 10), ('c', 1, 0), ('d', 2, 0), ('g', 3, 0)],
        [['a', 'b'], ['b', 'g'], ['a', 'c'], ['c', 'd'], ['d', 'g']]))
    assert shortest_path(graph, 'a', 'g') == (['a', 'c', 'd', 'g'], 3)


def test_unreachable_and_unknown():
    graph = graph_from_data(data(edges=[['a', 'b']]))
    assert shortest_path(graph, 'a', 'c') is None
    for a, b in [('missing', 'a'), ('a', 'missing'), ([], 'a')]:
        with pytest.raises(ValueError, match='unknown endpoint'):
            shortest_path(graph, a, b)


def test_deterministic_ties_independent_of_input_order():
    source = data([('s', 0, 0), ('a', 1, 1), ('b', 1, -1), ('g', 2, 0)],
                  [['s', 'b'], ['b', 'g'], ['s', 'a'], ['a', 'g']])
    rng = random.Random(7)
    for _ in range(12):
        rng.shuffle(source['nodes'])
        rng.shuffle(source['edges'])
        for edge in source['edges']:
            edge.reverse()
        path, cost = shortest_path(graph_from_data(source), 's', 'g')
        assert path == ['s', 'a', 'g']
        assert cost == pytest.approx(2 * math.sqrt(2))


@pytest.mark.parametrize('field,value', [
    ('schema_version', 2), ('schema_version', True), ('schema_version', 1.0),
    ('frame', 'odom'), ('frame', None), ('depot', 'unknown'), ('depot', []),
    ('nodes', []), ('nodes', {}), ('nodes', None), ('edges', {}), ('edges', None),
])
def test_bad_top_level_fields(field, value):
    source = data()
    source[field] = value
    with pytest.raises(ValueError):
        graph_from_data(source)


@pytest.mark.parametrize('value', [None, [], 1, True, {}, {'extra': 1}])
def test_bad_root(value):
    with pytest.raises(ValueError):
        graph_from_data(value)


@pytest.mark.parametrize('coordinate', [True, False, None, '1', [], {},
                                      math.nan, math.inf, -math.inf, 10**400])
@pytest.mark.parametrize('axis', ['x', 'y'])
def test_bad_coordinates(coordinate, axis):
    source = data()
    source['nodes'][0][axis] = coordinate
    with pytest.raises(ValueError):
        graph_from_data(source)


@pytest.mark.parametrize('identifier', ['', '  ', None, True, 1, [], {}])
def test_bad_identifiers(identifier):
    source = data()
    source['nodes'][0]['id'] = identifier
    with pytest.raises(ValueError):
        graph_from_data(source)


@pytest.mark.parametrize('record', [None, [], {}, {'id': 'a', 'x': 0},
                                  {'id': 'a', 'x': 0, 'y': 0, 'z': 0}])
def test_bad_node_records(record):
    source = data()
    source['nodes'][0] = record
    with pytest.raises(ValueError):
        graph_from_data(source)


def test_duplicate_nodes():
    source = data()
    source['nodes'].append(deepcopy(source['nodes'][0]))
    with pytest.raises(ValueError, match='duplicate node'):
        graph_from_data(source)


@pytest.mark.parametrize('edges', [
    [None], [['a']], [['a', 'b', 'c']], ['ab'], [['a', []]],
    [['a', 'unknown']], [['a', 'a']], [['a', 'b'], ['b', 'a']],
])
def test_bad_edges(edges):
    with pytest.raises(ValueError):
        graph_from_data(data(edges=edges))


@pytest.mark.parametrize('points', [
    [('a', 0, 0), ('b', 0, 0)],
    [('a', -1e308, 0), ('b', 1e308, 0)],
])
def test_invalid_edge_length(points):
    with pytest.raises(ValueError, match='edge length'):
        graph_from_data(data(points, [['a', 'b']]))


@pytest.mark.parametrize('text', ['{', '{"frame":"world","frame":"odom"}',
                                '{"nodes":[{"id":"a","id":"b"}]}'])
def test_invalid_json(tmp_path, text):
    path = tmp_path / 'graph.json'
    path.write_text(text)
    with pytest.raises(ValueError):
        load_graph(path)


def floyd_warshall(source):
    """Test-only reference: compute weights directly from the source coordinates."""
    nodes = {n['id']: (n['x'], n['y']) for n in source['nodes']}
    distances = {(a, b): 0.0 if a == b else math.inf for a in nodes for b in nodes}
    for a, b in source['edges']:
        ax, ay = nodes[a]
        bx, by = nodes[b]
        distances[a, b] = distances[b, a] = math.sqrt((ax-bx)**2 + (ay-by)**2)
    for k in nodes:
        for a in nodes:
            for b in nodes:
                distances[a, b] = min(distances[a, b], distances[a, k] + distances[k, b])
    return distances


@pytest.mark.parametrize('kind', ['warehouse', 'weighted', 'disconnected', 'random'])
def test_all_pairs_against_floyd_warshall(kind):
    if kind == 'warehouse':
        source = json.loads(GRAPH.read_text())
    elif kind == 'weighted':
        source = data([('a', 0, 0), ('b', 0, 10), ('c', 1, 0), ('d', 2, 0)],
                      [['a', 'b'], ['b', 'd'], ['a', 'c'], ['c', 'd']])
    elif kind == 'disconnected':
        source = data(edges=[['a', 'b']])
    else:
        rng = random.Random(42)
        points = [(str(i), rng.uniform(-5, 5), rng.uniform(-5, 5)) for i in range(9)]
        edges = [[str(i), str(j)] for i in range(9) for j in range(i+1, 9)
                 if rng.random() < 0.4]
        source = data(points, edges)
    graph = graph_from_data(source)
    for (a, b), expected in floyd_warshall(source).items():
        result = shortest_path(graph, a, b)
        if math.isinf(expected):
            assert result is None
        else:
            path, cost = result
            assert cost == pytest.approx(expected)
            assert path[0] == a and path[-1] == b
            assert len(path) == len(set(path))
            lengths = dict(((u, v), w) for u, neighbors in graph.adjacency.items()
                           for v, w in neighbors)
            assert sum(lengths[u, v] for u, v in zip(path, path[1:])) == pytest.approx(cost)


def test_real_warehouse():
    graph = load_graph(GRAPH)
    assert graph.frame == 'world' and graph.depot == 'r0c0'
    assert graph.nodes['r0c0'] == (-4, -3)
    assert len(graph.nodes) == 20 and len(graph.edges) == 22
    assert dict(graph.nodes) == {f'r{r}c{c}': (x, y)
                                 for r, y in enumerate((-3, -1, 1, 3))
                                 for c, x in enumerate((-4, -2, 0, 2, 4))}
    expected_edges = {tuple(sorted((f'r{r}c{c}', f'r{r}c{c+1}')))
                      for r in range(4) for c in range(4)}
    expected_edges |= {tuple(sorted((f'r{r}c{c}', f'r{r+1}c{c}')))
                       for r in range(3) for c in (0, 4)}
    assert set(graph.edges) == expected_edges
    assert shortest_path(graph, 'r0c0', 'r3c2')[1] == 10
    obstacles = load_obstacles(SDF)
    assert len(obstacles) == 7
    validate_geometry(graph, obstacles)
    assert math.dist(graph.nodes['r0c0'], graph.nodes['r3c2']) == pytest.approx(math.sqrt(52))
    assert any(segment_intersects(graph.nodes['r0c0'], graph.nodes['r3c2'], o)
               for o in obstacles)


def test_crossing_and_touching_rejected():
    obstacles = load_obstacles(SDF)
    crossing = graph_from_data(data([('a', -4, -3), ('b', 0, 3)], [['a', 'b']]))
    with pytest.raises(ValueError, match='edge'):
        validate_geometry(crossing, obstacles)
    shelf = next(o for o in obstacles if o.name == 'shelf_row_1')
    touching = graph_from_data(data([('a', -4, shelf.ymin), ('b', 4, shelf.ymin)], [['a', 'b']]))
    with pytest.raises(ValueError, match='edge'):
        validate_geometry(touching, obstacles)
    node_touch = graph_from_data(data([('a', shelf.xmin, shelf.ymin)], []))
    with pytest.raises(ValueError, match='node'):
        validate_geometry(node_touch, obstacles)


@pytest.mark.parametrize('a,b,expected', [
    ((-1, 0), (2, 0), True), ((-1, -1), (0, 0), True),
    ((0.5, 0.5), (0.5, 0.5), True), ((-1, 2), (2, 2), False),
    ((-1, 0.5), (-1, 0.5), False), ((2, 2), (3, 3), False),
])
def test_segment_slabs(a, b, expected):
    box = Bounds('test', 0, 1, 0, 1)
    assert segment_intersects(a, b, box) is expected
    assert segment_intersects(b, a, box) is expected


def changed_sdf(tmp_path, change):
    worlds = tmp_path / 'worlds'
    worlds.mkdir()
    shutil.copytree(SDF.parent.parent / 'models', tmp_path / 'models')
    tree = ET.parse(SDF)
    change(tree.getroot().find('world'))
    path = worlds / 'warehouse.sdf'
    tree.write(path)
    return path


@pytest.mark.parametrize('change', [
    lambda w: w.remove(w.find("model[@name='shelf_row_1']")),
    lambda w: w.append(deepcopy(w.find("model[@name='north_wall']"))),
    lambda w: w.find("model[@name='north_wall']").set('name', 'unexpected'),
    lambda w: w.find("model[@name='north_wall']/static").__setattr__('text', 'false'),
    lambda w: w.find("model[@name='north_wall']/pose").__setattr__('text', '0 5 0 0 0 0.1'),
    lambda w: w.find("model[@name='north_wall']/pose").set('relative_to', 'other'),
    lambda w: w.find("model[@name='north_wall']/pose").__setattr__('text', '0 nan 0 0 0 0'),
    lambda w: w.find("model[@name='north_wall']/link/collision/geometry/box").__setattr__('tag', 'sphere'),
    lambda w: w.find("model[@name='north_wall']/link/collision/geometry/box/size").__setattr__('text', '0 1 1'),
    lambda w: w.find("model[@name='north_wall']/link/collision/geometry/box/size").__setattr__('text', '1 1 inf'),
    lambda w: w.find("model[@name='north_wall']/link").append(ET.Element('frame')),
    lambda w: w.find("model[@name='north_wall']/link").append(ET.Element('model')),
    lambda w: w.find("model[@name='north_wall']/link").remove(
        w.find("model[@name='north_wall']/link/collision")),
    lambda w: ET.SubElement(w.find("model[@name='north_wall']/link"), 'pose').__setattr__(
        'text', '0 0 0 0.1 0 0'),
    lambda w: ET.SubElement(w.find("model[@name='north_wall']/link/collision"), 'pose').set(
        'relative_to', 'body'),
    lambda w: w.find("model[@name='north_wall']").set('placement_frame', 'body'),
    lambda w: w.find("model[@name='depot_marker']/link").append(
        deepcopy(w.find("model[@name='north_wall']/link/collision"))),
    lambda w: w.find('include/uri').__setattr__('text', 'model://other'),
    lambda w: w.append(deepcopy(w.find('include'))),
    lambda w: w.append(ET.Element('frame')),
    lambda w: w.find("model[@name='floor']/link/visual").append(ET.Element('collision')),
])
def test_unsupported_or_missing_geometry(tmp_path, change):
    with pytest.raises(ValueError):
        load_obstacles(changed_sdf(tmp_path, change))


def test_model_link_collision_translations(tmp_path):
    def change(w):
        link = w.find("model[@name='shelf_row_1']/link")
        ET.SubElement(link, 'pose').text = '0.1 0.2 0 0 0 0'
        ET.SubElement(link.find('collision'), 'pose').text = '0.3 0.4 0 0 0 0'
    shelf = next(o for o in load_obstacles(changed_sdf(tmp_path, change))
                 if o.name == 'shelf_row_1')
    assert shelf.xmin == pytest.approx(-3.05)
    assert shelf.ymin == pytest.approx(-2.25)


def test_static_robot_is_not_exempt(tmp_path):
    path = changed_sdf(tmp_path, lambda w: None)
    robot_path = tmp_path / 'models' / 'warehouse_robot' / 'model.sdf'
    tree = ET.parse(robot_path)
    ET.SubElement(tree.getroot().find('model'), 'static').text = 'true'
    tree.write(robot_path)
    with pytest.raises(ValueError, match='dynamic'):
        load_obstacles(path)


@pytest.mark.parametrize('radius', [True, -1, math.nan, math.inf])
def test_invalid_radius(radius):
    with pytest.raises(ValueError):
        load_obstacles(SDF, radius)


def test_installed_cli_from_tmp(tmp_path):
    """Only ROS installation checks need the installed ament index and ros2."""
    pytest.importorskip('ament_index_python')
    command = ['ros2', 'run', 'warehouse_planning', 'route_example']
    result = subprocess.run(command + ['--start', 'r0c0', '--goal', 'r3c2'],
                            cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'Frame: world' in result.stdout
    assert 'Total route length: 10 m' in result.stdout
    assert '(-4, -3)' in result.stdout and '(0, 3)' in result.stdout
    disconnected = json.loads(GRAPH.read_text())
    disconnected['edges'] = []
    graph_path = tmp_path / 'disconnected.json'
    graph_path.write_text(json.dumps(disconnected))
    result = subprocess.run(command + ['--graph', str(graph_path)], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode != 0 and 'No graph route' in result.stderr
    graph_path.write_text('{"schema_version": 2}')
    result = subprocess.run(command + ['--graph', str(graph_path)], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode != 0 and 'Route planning failed' in result.stderr
    result = subprocess.run(command + ['--start', 'missing'], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode != 0 and 'unknown endpoint' in result.stderr
