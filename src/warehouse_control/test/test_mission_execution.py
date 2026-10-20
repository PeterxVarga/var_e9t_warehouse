"""Hand-computed mission transitions and startup geometry contracts."""

import json

import pytest

from warehouse_control.control import ControlConfig
from warehouse_control.mission import MissionProgress, prepare_mission
from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.roadmap import load_graph
from test_route import GRAPH, SDF, changed_world


CONFIG = ControlConfig()


@pytest.fixture
def line(tmp_path):
    data = dict(schema_version=1, frame='world', depot='depot',
                nodes=[dict(id=n, x=x, y=0) for n, x in
                       [('depot', 0), ('near', 1), ('far', 2)]],
                edges=[['depot', 'near'], ['near', 'far']])
    path = tmp_path / 'line.json'
    path.write_text(json.dumps(data))
    return load_graph(path)


def progress(graph, requests):
    plan = plan_fifo_mission(graph, requests)
    return MissionProgress(plan, [[graph.nodes[n] for n in leg.nodes] for leg in plan.legs])


def step(mission, xy, t):
    return mission.command((*xy, 0), CONFIG, t, t)


def arrive(mission, start=1000000000):
    t = start
    for point in mission.route.points:
        assert step(mission, point, t) == (0, 0, False)
        t += 50000000
    assert mission.state == 'SERVICING'
    return t - 50000000


def serve(mission, t):
    point = mission.route.points[-1]
    for offset in range(1, 21):
        assert step(mission, point, t + offset * 50000000) == (0, 0, False)
        assert mission.state == ('TRACKING' if offset == 20 else 'SERVICING')
    return t + 1000000000


def test_full_fifo_legs_and_latched_return(line):
    mission = progress(line, ['far', 'near'])
    observed, requests = [], []
    t = 1000000000
    for leg in range(3):
        route = mission.route
        for point in route.points:
            command = step(mission, point, t)
            observed.extend((event.leg, event.node) for event in mission.events
                            if event.kind == 'waypoint')
            requests.extend(event.node for event in mission.events if event.kind == 'arrival')
            if leg < 2:
                assert not command[2]
            t += 50000000
        if leg < 2:
            t = serve(mission, t - 50000000) + 50000000
    assert observed == [(0, 'depot'), (0, 'near'), (0, 'far'),
                        (1, 'far'), (1, 'near'), (2, 'near'), (2, 'depot')]
    assert requests == ['far', 'near']
    assert mission.completed_requests == 2 and mission.state == 'REACHED'
    assert mission.plan.total_distance_m == 4
    assert step(mission, (99, 99), 0) == (0, 0, True)
    assert mission.events == ()
    mission.fail()
    assert mission.state == 'REACHED'


def test_passing_later_request_does_not_serve_it(line):
    mission = progress(line, ['far', 'near'])
    step(mission, (0, 0), 1000000000)
    step(mission, (1, 0), 1050000000)
    assert mission.state == 'TRACKING' and mission.completed_requests == 0
    assert [event.kind for event in mission.events] == ['waypoint']
    assert mission.leg_index == 0 and mission.route.index == 2


def test_empty_is_stationary_and_checks_start(line):
    mission = progress(line, [])
    assert step(mission, (0, 0), 1000000000) == (0, 0, True)
    assert mission.route is None and mission.plan.nodes == ('depot',)
    assert mission.plan.total_distance_m == 0
    assert [event.kind for event in mission.events] == ['depot']
    bad = progress(line, [])
    with pytest.raises(ValueError, match='fresh simulation'):
        step(bad, (0.06, 0), 1000000000)
    assert bad.state == 'FAULT'


def test_repeated_requests_have_separate_service_and_ticks(line):
    mission = progress(line, ['near', 'near'])
    t = serve(mission, arrive(mission))
    assert mission.completed_requests == 1 and mission.leg_index == 1
    assert mission.plan.legs[1].distance_m == 0
    step(mission, (1, 0), t + 50000000)
    assert mission.state == 'SERVICING' and mission.completed_requests == 1
    assert [e.kind for e in mission.events] == ['waypoint', 'arrival']
    serve(mission, t + 50000000)
    assert mission.completed_requests == 2 and mission.leg_index == 2


def test_service_restarts_after_leaving_tolerance(line):
    mission = progress(line, ['near'])
    t = arrive(mission)
    step(mission, (1, 0), t + 900000000)
    assert mission.completed_requests == 0
    assert step(mission, (0.94, 0), t + 950000000)[0] > 0
    assert mission.service_start is None
    step(mission, (1, 0), t + 1000000000)
    assert [e.kind for e in mission.events] == ['service_restart']
    step(mission, (1, 0), t + 1950000000)
    assert mission.completed_requests == 0
    step(mission, (1, 0), t + 2000000000)
    assert mission.completed_requests == 1


def test_repeated_feedback_and_paused_clock_cannot_complete_service(line):
    mission = progress(line, ['near'])
    t = arrive(mission)
    for offset in range(1, 41):
        assert mission.command((1, 0, 0), CONFIG, t, t + offset * 50000000) == (0, 0, False)
    assert mission.completed_requests == 0
    paused = progress(line, ['near'])
    t = arrive(paused)
    for offset in range(1, 41):
        assert paused.command((1, 0, 0), CONFIG, t + offset * 50000000, t) == (0, 0, False)
    assert paused.completed_requests == 0


@pytest.mark.parametrize('bad_time', [-1, 1.0, True, None])
def test_invalid_time_fault_latches(line, bad_time):
    mission = progress(line, ['near'])
    with pytest.raises(ValueError, match='timestamps'):
        mission.command((0, 0, 0), CONFIG, bad_time, 1000000000)
    assert mission.state == 'FAULT'
    assert step(mission, (0, 0), 2000000000) == (0, 0, False)


@pytest.mark.parametrize('clock_backwards', [False, True])
def test_backward_time_and_explicit_fault_do_not_rearm(line, clock_backwards):
    mission = progress(line, ['near'])
    t = arrive(mission)
    with pytest.raises(ValueError, match='backwards'):
        mission.command((1, 0, 0), CONFIG, t if clock_backwards else t - 1,
                        t - 1 if clock_backwards else t)
    mission.fail()
    assert step(mission, (0, 0), t + 2000000000) == (0, 0, False)
    assert mission.completed_requests == 0 and mission.state == 'FAULT'


def test_snapshot_inputs_and_plan_unchanged(line):
    requests = ['far', 'near']
    before = dict(line.nodes), dict(line.adjacency), requests[:]
    plan = plan_fifo_mission(line, requests)
    points = [[list(line.nodes[n]) for n in leg.nodes] for leg in plan.legs]
    mission = MissionProgress(plan, points)
    step(mission, (0, 0), 1000000000)
    assert (line.nodes, line.adjacency, requests) == before
    requests.clear()
    points[0][0][0] = 99
    assert mission.plan.pickup_nodes == ('far', 'near') and mission.points[0][0] == (0, 0)
    assert mission.plan == plan


@pytest.mark.parametrize('requests,lengths,total', [
    (['r2c2', 'r1c4'], [8, 6, 10], 24),
    (['r2c2', 'r1c0', 'r1c0'], [8, 6, 0, 2], 16),
    ([], [], 0),
])
def test_actual_preparation(requests, lengths, total):
    before = GRAPH.read_bytes(), SDF.read_bytes(), requests[:]
    mission, transform = prepare_mission(GRAPH, SDF, requests)
    assert [leg.distance_m for leg in mission.plan.legs] == lengths
    assert mission.plan.total_distance_m == total
    assert transform.odom_to_world((0, 0)) == (-4, -3)
    assert (GRAPH.read_bytes(), SDF.read_bytes(), requests) == before


@pytest.mark.parametrize('requests', [None, 'r2c2', [True], ['missing']])
def test_invalid_requests(requests):
    with pytest.raises(ValueError):
        prepare_mission(GRAPH, SDF, requests)


def test_unreachable_later_leg_and_full_geometry_even_when_empty(tmp_path):
    data = json.loads(GRAPH.read_text())
    data['edges'] = [e for e in data['edges'] if 'r1c4' not in e]
    graph = tmp_path / 'graph.json'
    graph.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='leg 2.*unreachable'):
        prepare_mission(graph, SDF, ['r2c2', 'r1c4'])
    data['edges'].append(['r0c0', 'r3c2'])
    graph.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='intersects'):
        prepare_mission(graph, SDF, [])


def test_depot_spawn_mismatch(tmp_path):
    world = changed_world(tmp_path, lambda w: w.find('include/pose').__setattr__(
        'text', '-4.1 -3 0 0 0 0'))
    with pytest.raises(ValueError, match='depot does not match'):
        prepare_mission(GRAPH, world, ['r2c2'])


@pytest.mark.parametrize('strategy,order,lengths,total', [
    ('fifo', (0, 1, 2), (6, 6, 8, 12), 32),
    ('nn', (1, 0, 2), (4, 6, 6, 12), 28),
    ('nn-2opt', (1, 2, 0), (4, 8, 6, 6), 24),
])
def test_prepared_strategies_and_complete_identity_aware_execution(strategy, order, lengths, total):
    requests = ['r1c2', 'r2c0', 'r2c4']
    mission, _ = prepare_mission(GRAPH, SDF, requests, strategy)
    assert mission.strategy == strategy
    assert mission.original_requests == tuple(requests)
    assert mission.request_order == order
    assert mission.plan.pickup_nodes == tuple(requests[i] for i in order)
    assert tuple(leg.distance_m for leg in mission.plan.legs) == lengths
    assert mission.plan.total_distance_m == total
    requests.clear()
    with pytest.raises(AttributeError):
        mission.request_order = ()
    with pytest.raises(TypeError):
        mission.request_order[0] = 1
    t, events = 1000000000, []
    for leg in range(4):
        for point in mission.route.points:
            step(mission, point, t)
            events.extend(mission.events)
            t += 50000000
        if leg < 3:
            for _ in range(20):
                step(mission, point, t)
                events.extend(mission.events)
                t += 50000000
            assert mission.completed_requests == leg + 1 and mission.state != 'REACHED'
    assert mission.state == 'REACHED'
    assert [e.request_index for e in events if e.kind == 'arrival'] == list(order)
    assert [e.request_index for e in events if e.kind == 'serviced'] == list(order)
    assert all(e.request_index is None for e in events if e.leg == 3)
    assert events[-1].kind == 'depot' and events[-1].request_index is None
    assert step(mission, (99, 99), 0) == (0, 0, True)


@pytest.mark.parametrize('strategy', ['fifo', 'nn', 'nn-2opt'])
def test_strategy_empty_and_duplicate_depot_identities(strategy):
    empty, _ = prepare_mission(GRAPH, SDF, [], strategy)
    assert empty.request_order == empty.original_requests == ()
    assert step(empty, (0, 0), 1000000000) == (0, 0, True)
    assert empty.events[0].request_index is None
    requests = ['r1c4', 'r0c0', 'r2c2', 'r2c2', 'r0c0']
    mission, _ = prepare_mission(GRAPH, SDF, requests, strategy)
    assert mission.original_requests == tuple(requests)
    assert mission.request_order == ((0, 1, 2, 3, 4) if strategy == 'fifo' else (1, 4, 2, 3, 0))
    assert len(mission.plan.legs) == 6
    assert mission.request_index(5) is None


@pytest.mark.parametrize('strategy', [None, True, 1, [], '', 'NN', 'unknown'])
def test_invalid_strategy(strategy):
    with pytest.raises(ValueError, match='strategy'):
        prepare_mission(GRAPH, SDF, [], strategy)


@pytest.mark.parametrize('strategy', ['fifo', 'nn', 'nn-2opt'])
def test_geometry_spawn_and_unreachable_validation_for_all_strategies(tmp_path, strategy):
    data = json.loads(GRAPH.read_text())
    data['edges'].append(['r0c0', 'r3c2'])
    graph = tmp_path / 'graph.json'
    graph.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='intersects'):
        prepare_mission(graph, SDF, [], strategy)
    world = changed_world(tmp_path, lambda w: w.find('include/pose').__setattr__(
        'text', '-4.1 -3 0 0 0 0'))
    with pytest.raises(ValueError, match='depot does not match'):
        prepare_mission(GRAPH, world, [], strategy)
    data['edges'] = [edge for edge in json.loads(GRAPH.read_text())['edges'] if 'r1c4' not in edge]
    graph.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='unreachable'):
        prepare_mission(graph, SDF, ['r2c2', 'r1c4'], strategy)
    with pytest.raises(ValueError, match='unknown pickup'):
        prepare_mission(GRAPH, SDF, ['missing'], strategy)


@pytest.mark.parametrize('order', [(0, 0), (0,), (0, 2), (False, 1), (0, 1.0)])
def test_invalid_request_identity_permutation(line, order):
    plan = plan_fifo_mission(line, ['near', 'far'])
    with pytest.raises(ValueError, match='permutation'):
        MissionProgress(plan, [tuple(line.nodes[n] for n in leg.nodes) for leg in plan.legs],
                        request_order=order)


def test_mapping_snapshot_and_plan_alignment(line):
    plan = plan_fifo_mission(line, ['near', 'far'])
    requests, order = ['far', 'near'], [1, 0]
    points = [tuple(line.nodes[n] for n in leg.nodes) for leg in plan.legs]
    mission = MissionProgress(plan, points, original_requests=requests, request_order=order, strategy='nn')
    requests.clear()
    order.clear()
    assert mission.original_requests == ('far', 'near') and mission.request_order == (1, 0)
    with pytest.raises(ValueError, match='map original'):
        MissionProgress(plan, points, original_requests=['far', 'near'])


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
def test_identity_mapping_does_not_change_commands_for_the_same_ordered_plan(strategy):
    optimized, _ = prepare_mission(GRAPH, SDF, ['r1c2', 'r2c0', 'r2c4'], strategy)
    fifo, _ = prepare_mission(GRAPH, SDF, optimized.plan.pickup_nodes)
    assert optimized.plan == fifo.plan and optimized.points == fifo.points
    t, moving_comparisons = 1000000000, 0
    for leg, points in enumerate(optimized.points):
        for index, point in enumerate(points):
            if index and point != points[index - 1]:
                previous = points[index - 1]
                between = tuple((a + b) / 2 for a, b in zip(previous, point))
                command = step(optimized, between, t)
                assert command == step(fifo, between, t)
                assert command[:2] != (0, 0) and not command[2]
                moving_comparisons += 1
                t += 50000000
            assert step(optimized, point, t) == step(fifo, point, t)
            assert optimized.state == fifo.state
            t += 50000000
        if leg < 3:
            for _ in range(20):
                assert step(optimized, point, t) == step(fifo, point, t)
                assert optimized.state == fifo.state
                t += 50000000
    assert moving_comparisons > 0 and optimized.state == fifo.state == 'REACHED'
