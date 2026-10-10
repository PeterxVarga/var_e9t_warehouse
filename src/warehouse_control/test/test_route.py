"""Startup contracts, ordered progress and an independent ideal-drive check."""

from copy import deepcopy
import json
from math import cos, hypot, inf, nan, pi, sin
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest

from warehouse_control.control import ControlConfig
from warehouse_control.route import (
    PlanarTransform, RouteProgress, load_spawn, prepare_route, validate_start,
)
from warehouse_planning.roadmap import load_obstacles, segment_intersects


PACKAGES = Path(__file__).resolve().parents[2]
GRAPH = PACKAGES / 'warehouse_planning' / 'config' / 'warehouse_graph.json'
SDF = PACKAGES / 'warehouse_sim' / 'worlds' / 'warehouse.sdf'
CONFIG = ControlConfig()


@pytest.mark.parametrize('world,odom', [
    ((-4, -3), (0, 0)), ((-4, 3), (0, 6)), ((0, 3), (4, 6)),
])
def test_translation(world, odom):
    transform = PlanarTransform(-4, -3, 0)
    assert transform.world_to_odom(world) == odom
    assert transform.odom_to_world(odom) == world


def test_rotated_transform_against_hand_computed_points():
    transform = PlanarTransform(2, 3, pi / 2)
    assert transform.world_to_odom((2, 5)) == pytest.approx((2, 0), abs=1e-12)
    assert transform.odom_to_world((1, 2)) == pytest.approx((0, 4))
    point = (-1.25, 6.75)
    assert transform.odom_to_world(transform.world_to_odom(point)) == pytest.approx(point)


@pytest.mark.parametrize('bad', [nan, inf, -inf, True, '1', 10**400])
@pytest.mark.parametrize('index', range(3))
def test_invalid_transform(bad, index):
    values = [0, 0, 0]
    values[index] = bad
    with pytest.raises(ValueError):
        PlanarTransform(*values)


@pytest.mark.parametrize('point', [(nan, 0), (0, inf), (1,), (0, 0, 0), None])
def test_invalid_points(point):
    transform = PlanarTransform(0, 0, 0)
    for convert in (transform.world_to_odom, transform.odom_to_world):
        with pytest.raises(ValueError):
            convert(point)


def test_transform_overflow():
    with pytest.raises(ValueError):
        PlanarTransform(-1e308, 0, 0).world_to_odom((1e308, 0))


def changed_world(tmp_path, change):
    world_dir = tmp_path / 'worlds'
    world_dir.mkdir()
    shutil.copytree(SDF.parent.parent / 'models', tmp_path / 'models')
    tree = ET.parse(SDF)
    change(tree.getroot().find('world'))
    path = world_dir / 'warehouse.sdf'
    tree.write(path)
    return path


def test_actual_spawn():
    assert load_spawn(SDF) == PlanarTransform(-4, -3, 0)


@pytest.mark.parametrize('change', [
    lambda w: w.remove(w.find('include')),
    lambda w: w.append(deepcopy(w.find('include'))),
    lambda w: w.find('include').remove(w.find('include/pose')),
    lambda w: w.find('include').append(deepcopy(w.find('include/pose'))),
    lambda w: w.find('include/pose').set('relative_to', 'floor'),
    lambda w: w.find('include/pose').__setattr__('text', '0 0 0 0 0'),
    lambda w: w.find('include/pose').__setattr__('text', '0 nan 0 0 0 0'),
    lambda w: w.find('include/pose').__setattr__('text', '0 0 0.1 0 0 0'),
    lambda w: w.find('include/pose').__setattr__('text', '0 0 0 0.1 0 0'),
    lambda w: w.find('include/uri').__setattr__('text', 'model://other'),
])
def test_invalid_spawn(tmp_path, change):
    with pytest.raises(ValueError):
        load_spawn(changed_world(tmp_path, change))


def test_plan_actual_routes_without_mutating_input():
    before = GRAPH.read_bytes(), SDF.read_bytes()
    for goal, ids, points, length in [
        ('r3c2', ('r0c0', 'r1c0', 'r2c0', 'r3c0', 'r3c1', 'r3c2'),
         ((0, 0), (0, 2), (0, 4), (0, 6), (2, 6), (4, 6)), 10),
        ('r2c2', ('r0c0', 'r1c0', 'r2c0', 'r2c1', 'r2c2'),
         ((0, 0), (0, 2), (0, 4), (2, 4), (4, 4)), 8),
    ]:
        route, transform, cost = prepare_route(GRAPH, SDF, goal)
        assert route.nodes == ids and route.points == points and cost == length
        assert transform == PlanarTransform(-4, -3, 0)
    assert (GRAPH.read_bytes(), SDF.read_bytes()) == before


def test_depot_spawn_mismatch(tmp_path):
    path = changed_world(tmp_path, lambda w: w.find('include/pose').__setattr__(
        'text', '-4.1 -3 0 0 0 0'))
    with pytest.raises(ValueError, match='depot does not match'):
        prepare_route(GRAPH, path, 'r3c2')


def test_invalid_and_unreachable_targets(tmp_path):
    with pytest.raises(ValueError, match='unknown endpoint'):
        prepare_route(GRAPH, SDF, 'missing')
    source = json.loads(GRAPH.read_text())
    source['edges'] = []
    disconnected = tmp_path / 'graph.json'
    disconnected.write_text(json.dumps(source))
    with pytest.raises(ValueError, match='no graph route'):
        prepare_route(disconnected, SDF, 'r3c2')


def test_preparation_rejects_shelf_crossing_edge(tmp_path):
    source = json.loads(GRAPH.read_text())
    source['edges'].append(['r0c0', 'r3c2'])
    graph = tmp_path / 'crossing.json'
    graph.write_text(json.dumps(source))
    with pytest.raises(ValueError, match='intersects'):
        prepare_route(graph, SDF, 'r3c2')


def test_preparation_rejects_missing_obstacle(tmp_path):
    world = changed_world(tmp_path, lambda w:
                          w.remove(w.find("model[@name='shelf_row_1']")))
    with pytest.raises(ValueError, match='missing expected models'):
        prepare_route(GRAPH, world, 'r3c2')


@pytest.mark.parametrize('pose', [(0, 0, 0), (0.05, 0, 0.05), (0, 0, 2*pi)])
def test_valid_start(pose):
    validate_start(pose)


@pytest.mark.parametrize('pose', [
    (0.050001, 0, 0), (0.04, 0.04, 0), (0, 0, 0.050001),
    (0, 0, -0.050001), (nan, 0, 0), (0, 0, inf),
])
def test_invalid_start(pose):
    with pytest.raises(ValueError):
        validate_start(pose)


def test_ordered_progress_stop_between_legs_and_final_latch():
    route = RouteProgress(['a', 'b', 'c'], [(0, 0), (1, 0), (1, 1)])
    assert route.command((0, 0, 0), CONFIG) == (0, 0, False)
    assert route.index == 1
    assert route.command((0, 0, 0), CONFIG)[0] > 0
    assert route.index == 1
    # Being at a later waypoint cannot skip the current one.
    assert not route.command((1, 1, 0), CONFIG)[2]
    assert route.index == 1
    assert route.command((1, 0, 0), CONFIG) == (0, 0, False)
    assert route.index == 2
    linear, angular, reached = route.command((1, 0, 0), CONFIG)
    assert linear == 0 and angular > 0 and not reached
    assert route.command((1, 1, pi/2), CONFIG) == (0, 0, True)
    assert route.command((10, 10, 0), CONFIG) == (0, 0, True)
    assert route.index == 2


def test_reverse_edge_turns_before_advancing():
    route = RouteProgress(['a', 'b', 'a'], [(0, 0), (1, 0), (0, 0)])
    route.command((0, 0, 0), CONFIG)
    route.command((1, 0, 0), CONFIG)
    linear, angular, reached = route.command((1, 0, 0), CONFIG)
    assert linear == 0 and abs(angular) == CONFIG.max_angular_speed and not reached


def test_repeated_arrival_advances_at_most_one_point():
    route = RouteProgress(['a', 'b', 'c'], [(0, 0)] * 3)
    for index in (1, 2):
        assert route.command((0, 0, 0), CONFIG) == (0, 0, False)
        assert route.index == index
    assert route.command((0, 0, 0), CONFIG) == (0, 0, True)


def test_single_depot_and_input_snapshot():
    route, _, cost = prepare_route(GRAPH, SDF, 'r0c0')
    assert cost == 0 and route.command((0, 0, 0), CONFIG) == (0, 0, True)
    nodes, points = ['a', 'b'], [[0, 0], [1, 0]]
    route = RouteProgress(nodes, points)
    nodes[0], points[0][0] = 'changed', 99
    assert route.nodes == ('a', 'b') and route.points == ((0, 0), (1, 0))


@pytest.mark.parametrize('nodes,points', [
    ([], []), (['a'], []), (['a', 'b'], [(0, 0)]),
    ([''], [(0, 0)]), (['a'], [(nan, 0)]), (['a'], [(0, inf)]),
])
def test_invalid_route(nodes, points):
    with pytest.raises(ValueError):
        RouteProgress(nodes, points)


@pytest.mark.parametrize('goal,target', [('r3c2', (4, 6)), ('r2c2', (4, 4))])
def test_ideal_drive_tracks_real_route_without_crossing_shelves(goal, target):
    """Exact unicycle arcs with perfect feedback; NOT Gazebo/contact validation."""
    route, _, _ = prepare_route(GRAPH, SDF, goal)
    obstacles = load_obstacles(SDF, radius=hypot(0.25, 0.20))
    x = y = yaw = 0.0
    dt = 0.05
    indices = [0]
    for step in range(3600):
        v, w, reached = route.command((x, y, yaw), CONFIG)
        assert 0 <= v <= CONFIG.max_linear_speed and abs(w) <= CONFIG.max_angular_speed
        previous = (-4 + x, -3 + y)
        if abs(w) > 1e-12:
            x += v/w * (sin(yaw + w*dt) - sin(yaw))
            y += v/w * (cos(yaw) - cos(yaw + w*dt))
        else:
            x += v * cos(yaw) * dt
            y += v * sin(yaw) * dt
        yaw += w * dt
        current = (-4 + x, -3 + y)
        assert not any(segment_intersects(previous, current, box) for box in obstacles)
        if route.index != indices[-1]:
            indices.append(route.index)
        if reached:
            break
    else:
        pytest.fail('ideal drive did not arrive within 180 simulated seconds')
    assert hypot(x - target[0], y - target[1]) <= 0.05
    assert indices == list(range(len(route.nodes)))
    for _ in range(100):
        assert route.command((x, y, yaw), CONFIG) == (0, 0, True)
