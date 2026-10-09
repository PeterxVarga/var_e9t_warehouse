"""Undirected Euclidean roadmaps, Dijkstra search and conservative SDF checks.

Geometry support is deliberately limited to implicit parent reference frames
and axis-aligned boxes with translation-only model/link/collision poses.
"""

from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
from types import MappingProxyType
from typing import Mapping
import xml.etree.ElementTree as ET


def _number(value, label):
    if type(value) not in (int, float):
        raise ValueError(f'{label} must be a finite number (not bool)')
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ValueError(f'{label} must be finite') from None
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def _identifier(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('node identifiers must be non-empty strings')
    return value


@dataclass(frozen=True)
class Graph:
    """Validated graph; node and adjacency mappings are read-only snapshots.

    Construct through graph_from_data or load_graph. Edges are canonical pairs,
    and adjacency entries contain (neighbor identifier, length in metres).
    """

    frame: str
    depot: str
    nodes: Mapping[str, tuple[float, float]]
    edges: tuple[tuple[str, str], ...]
    adjacency: Mapping[str, tuple[tuple[str, float], ...]]


def graph_from_data(data):
    """Validate schema version 1 and copy an undirected world-frame graph."""
    if not isinstance(data, dict) or set(data) != {
            'schema_version', 'frame', 'depot', 'nodes', 'edges'}:
        raise ValueError('graph must contain schema_version, frame, depot, nodes and edges')
    if type(data['schema_version']) is not int or data['schema_version'] != 1:
        raise ValueError('unsupported schema_version: expected integer 1')
    if data['frame'] != 'world':
        raise ValueError('unsupported frame: expected world')
    if not isinstance(data['nodes'], list) or not data['nodes']:
        raise ValueError('nodes must be a non-empty list')
    if not isinstance(data['edges'], list):
        raise ValueError('edges must be a list')
    nodes = {}
    for record in data['nodes']:
        if not isinstance(record, dict) or set(record) != {'id', 'x', 'y'}:
            raise ValueError('each node must contain exactly id, x and y')
        identifier = _identifier(record['id'])
        if identifier in nodes:
            raise ValueError(f'duplicate node identifier: {identifier}')
        nodes[identifier] = (_number(record['x'], f'{identifier}.x'),
                             _number(record['y'], f'{identifier}.y'))
    depot = _identifier(data['depot'])
    if depot not in nodes:
        raise ValueError(f'unknown depot: {depot}')
    edges = set()
    adjacency = {identifier: [] for identifier in nodes}
    for pair in data['edges']:
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError('each edge must be a pair of node identifiers')
        a, b = map(_identifier, pair)
        if a not in nodes or b not in nodes:
            raise ValueError(f'unknown edge endpoint: {a}, {b}')
        if a == b:
            raise ValueError(f'self-loop: {a}')
        edge = tuple(sorted((a, b)))
        if edge in edges:
            raise ValueError(f'duplicate undirected edge: {a}, {b}')
        length = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
        if not math.isfinite(length) or length <= 0:
            raise ValueError(f'edge length must be finite and positive: {a}, {b}')
        edges.add(edge)
        adjacency[a].append((b, length))
        adjacency[b].append((a, length))
    return Graph('world', depot, MappingProxyType(nodes), tuple(sorted(edges)),
                 MappingProxyType({k: tuple(sorted(v)) for k, v in adjacency.items()}))


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def load_graph(path):
    """Read JSON, rejecting duplicate object keys as well as invalid records."""
    with Path(path).open(encoding='utf-8') as stream:
        return graph_from_data(json.load(stream, object_pairs_hook=_unique_object))


def shortest_path(graph, start_id, goal_id):
    """Return (ordered node list, metres), or None for an unreachable goal.

    Dijkstra is optimal only on this graph. Sorted neighbors, identifier heap
    ordering and retaining the first equal-cost predecessor make ties stable
    regardless of JSON node/edge ordering. The input graph is never modified.
    """
    for identifier in (start_id, goal_id):
        if not isinstance(identifier, str) or identifier not in graph.nodes:
            raise ValueError(f'unknown endpoint: {identifier}')
    distances = {start_id: 0.0}
    previous = {}
    queue = [(0.0, start_id)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != distances[current]:
            continue
        if current == goal_id:
            path = [current]
            while current != start_id:
                current = previous[current]
                path.append(current)
            return list(reversed(path)), cost
        for neighbor, weight in graph.adjacency[current]:
            candidate = cost + weight
            if not math.isfinite(candidate):
                raise ValueError('accumulated route length is not finite')
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))
    return None


@dataclass(frozen=True)
class Bounds:
    """Closed XY rectangle in metres, already inflated by the chosen radius."""

    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


def _vector(text, count, label):
    try:
        values = tuple(float(part) for part in (text or '').split())
    except ValueError:
        raise ValueError(f'invalid {label}') from None
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError(f'{label} must contain {count} finite values')
    return values


def _pose(element):
    poses = element.findall('pose')
    if len(poses) > 1:
        raise ValueError('duplicate SDF pose')
    if not poses:
        return (0.0, 0.0, 0.0)
    pose = poses[0]
    if pose.attrib:
        raise ValueError('unsupported pose attributes/reference frame')
    values = _vector(pose.text, 6, 'pose')
    if any(value != 0 for value in values[3:]):
        raise ValueError('unsupported rotation: only translation-only poses are supported')
    return values[:3]


def _children(element, allowed):
    if any(child.tag not in allowed for child in element):
        raise ValueError(f'unsupported SDF structure in {element.tag}')
    if set(element.attrib) - {'name'}:
        raise ValueError(f'unsupported SDF attributes in {element.tag}')


def load_obstacles(sdf_path, radius=0.45):
    """Read the current warehouse's seven static obstacles, failing closed.

    Floor geometry is validated but excluded. Only the named visual-only depot
    marker and the local dynamic warehouse_robot include are exempted. Nested
    models, frame references, rotations and non-box collisions are unsupported.
    """
    radius = _number(radius, 'radius')
    if radius < 0:
        raise ValueError('radius must be non-negative')
    root = ET.parse(sdf_path).getroot()
    worlds = root.findall('world')
    if (root.tag != 'sdf' or root.get('version') != '1.8' or len(root) != 1
            or len(worlds) != 1 or worlds[0].get('name') != 'warehouse'):
        raise ValueError('expected exactly one warehouse world')
    world = worlds[0]
    _children(world, {'gravity', 'physics', 'plugin', 'light', 'model', 'include'})
    expected = {'shelf_row_1', 'shelf_row_2', 'shelf_row_3',
                'north_wall', 'south_wall', 'east_wall', 'west_wall'}
    seen = set()
    bounds = []
    total_collisions = 0
    for model in world.findall('model'):
        name = model.get('name')
        if name in seen or name not in expected | {'floor', 'depot_marker'}:
            raise ValueError(f'unexpected or duplicate model: {name}')
        seen.add(name)
        _children(model, {'static', 'pose', 'link'})
        if len(model.findall('static')) != 1 or model.findtext('static') not in ('true', '1'):
            raise ValueError(f'expected static model: {name}')
        model_pose = _pose(model)
        collision_count = 0
        for link in model.findall('link'):
            _children(link, {'pose', 'collision', 'visual'})
            link_pose = _pose(link)
            for collision in link.findall('collision'):
                collision_count += 1
                total_collisions += 1
                _children(collision, {'pose', 'geometry'})
                collision_pose = _pose(collision)
                geometries = collision.findall('geometry')
                if len(geometries) != 1 or len(geometries[0]) != 1:
                    raise ValueError(f'unsupported collision geometry: {name}')
                box = geometries[0].find('box')
                if box is None or len(box) != 1 or box.find('size') is None:
                    raise ValueError(f'only box collision geometry is supported: {name}')
                if geometries[0].attrib or box.attrib or box[0].attrib:
                    raise ValueError(f'unsupported geometry attributes: {name}')
                size = _vector(box.findtext('size'), 3, 'box size')
                if any(v <= 0 for v in size):
                    raise ValueError(f'box size must be positive: {name}')
                center = tuple(sum(values) for values in zip(
                    model_pose, link_pose, collision_pose))
                xmin, xmax = center[0] - size[0]/2 - radius, center[0] + size[0]/2 + radius
                ymin, ymax = center[1] - size[1]/2 - radius, center[1] + size[1]/2 + radius
                if not all(math.isfinite(v) for v in (*center, xmin, xmax, ymin, ymax)):
                    raise ValueError(f'non-finite transformed bounds: {name}')
                if name in expected:
                    bounds.append(Bounds(name, xmin, xmax, ymin, ymax))
        required_count = 0 if name == 'depot_marker' else 1
        if collision_count != required_count:
            raise ValueError(f'expected {required_count} collisions in {name}')
    if seen != expected | {'floor', 'depot_marker'}:
        raise ValueError(f'missing expected models: {sorted((expected | {"floor", "depot_marker"}) - seen)}')
    if len(list(world.iter('collision'))) != total_collisions:
        raise ValueError('unexpected collision outside supported model links')
    includes = world.findall('include')
    if len(includes) != 1:
        raise ValueError('expected only the dynamic warehouse_robot include')
    include = includes[0]
    _children(include, {'uri', 'name', 'pose'})
    if len(include.findall('uri')) != 1 or len(include.findall('name')) != 1:
        raise ValueError('expected one included model URI and name')
    if include.findtext('uri') != 'model://warehouse_robot' or include.findtext('name') != 'warehouse_robot':
        raise ValueError('unexpected included model')
    _pose(include)
    robot_path = Path(sdf_path).parent.parent / 'models' / 'warehouse_robot' / 'model.sdf'
    robot_root = ET.parse(robot_path).getroot()
    robots = robot_root.findall('model')
    if (robot_root.tag != 'sdf' or len(robot_root) != 1
            or len(robots) != 1 or robots[0].get('name') != 'warehouse_robot'
            or any(s.text not in ('false', '0') for s in robots[0].iter('static'))
            or robots[0].find('.//model') is not None
            or robots[0].find('.//include') is not None):
        raise ValueError('warehouse_robot must be a local dynamic model')
    return tuple(bounds)


def segment_intersects(a, b, bounds):
    """Closed segment/rectangle slab test: boundary contact counts as collision."""
    low, high = 0.0, 1.0
    for start, end, minimum, maximum in (
            (a[0], b[0], bounds.xmin, bounds.xmax),
            (a[1], b[1], bounds.ymin, bounds.ymax)):
        delta = end - start
        if delta == 0:
            if start < minimum or start > maximum:
                return False
            continue
        enter, leave = sorted(((minimum - start)/delta, (maximum - start)/delta))
        low, high = max(low, enter), min(high, leave)
        if low > high:
            return False
    return True


def validate_geometry(graph, obstacles):
    """Reject any node or entire edge segment touching an inflated obstacle."""
    for identifier, point in graph.nodes.items():
        for obstacle in obstacles:
            if segment_intersects(point, point, obstacle):
                raise ValueError(f'node {identifier} intersects {obstacle.name}')
    for a, b in graph.edges:
        for obstacle in obstacles:
            if segment_intersects(graph.nodes[a], graph.nodes[b], obstacle):
                raise ValueError(f'edge {a}--{b} intersects {obstacle.name}')
