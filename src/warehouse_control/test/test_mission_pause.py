"""Pause transitions with hand-written original request identities."""

from dataclasses import FrozenInstanceError
from math import inf, nan

import pytest

from warehouse_control.control import ControlConfig
from warehouse_control.mission import MissionCheckpoint, MissionProgress, prepare_mission
from warehouse_planning.mission import plan_fifo_mission, plan_remaining_mission
from warehouse_planning.roadmap import graph_from_data, load_graph
from test_route import GRAPH, SDF


CONFIG = ControlConfig()
SECOND = 1000000000
TICK = 50000000
ZERO = (0.0, 0.0, False)


@pytest.fixture
def line():
    return graph_from_data(dict(
        schema_version=1, frame='world', depot='depot',
        nodes=[dict(id=n, x=x, y=0) for n, x in
               [('depot', 0), ('near', 1), ('far', 2)]],
        edges=[['depot', 'near'], ['near', 'far']]))


def progress(graph, requests):
    plan = plan_fifo_mission(graph, requests)
    points = [[graph.nodes[n] for n in leg.nodes] for leg in plan.legs]
    return MissionProgress(plan, points)


def step(mission, point, t):
    return mission.command((*point, 0), CONFIG, t, t)


def arrive(mission, t=SECOND):
    for point in mission.route.points:
        assert step(mission, point, t) == ZERO
        t += TICK
    assert mission.state == 'SERVICING'
    return t - TICK


def serve(mission, t):
    assert step(mission, mission.route.points[-1], t + SECOND) == ZERO
    assert mission.state == 'TRACKING'
    assert [e.kind for e in mission.events] == ['serviced']
    return t + SECOND


def execution_state(mission):
    return (mission.leg_index, mission.completed_requests, mission.service_start,
            tuple((r.index, r.reached) for r in mission._routes))


def assert_checkpoint(checkpoint, requests, completed, remaining):
    assert isinstance(checkpoint, MissionCheckpoint)
    assert checkpoint.original_requests == requests
    assert checkpoint.completed_order == completed
    assert checkpoint.remaining_order == remaining
    assert all(type(value) is tuple for value in
               (checkpoint.original_requests, checkpoint.completed_order,
                checkpoint.remaining_order))
    indices = checkpoint.completed_order + checkpoint.remaining_order
    assert sorted(indices) == list(range(len(requests)))
    assert len(set(indices)) == len(requests)


def assert_stays_paused(mission, t):
    before = execution_state(mission)
    # Include every target, the depot, departures and enough time for service.
    points = [point for leg in mission.points for point in leg] + [(99, 99)]
    for offset, point in enumerate(points, 1):
        assert step(mission, point, t + offset * 2 * SECOND) == ZERO
        assert mission.last_command == ZERO
        assert mission.state == 'PAUSED' and mission.events == ()
        assert execution_state(mission) == before


def test_pause_during_travel_zeros_stored_and_future_commands(line):
    mission = progress(line, ['far', 'near'])
    step(mission, (0, 0), SECOND)
    assert mission.command((0.2, 0.2, 0), CONFIG, SECOND + TICK,
                           SECOND + TICK)[:2] != (0, 0)
    before = execution_state(mission)
    checkpoint = mission.pause()
    assert mission.last_command == ZERO
    assert execution_state(mission) == before
    assert_checkpoint(checkpoint, ('far', 'near'), (), (0, 1))
    assert_stays_paused(mission, SECOND + TICK)


@pytest.mark.parametrize('partial', [0, SECOND // 2, SECOND - 1])
def test_arrival_and_partial_service_remain_pending(line, partial):
    mission = progress(line, ['near', 'far'])
    t = arrive(mission)
    if partial:
        step(mission, (1, 0), t + partial)
    assert mission.service_start is not None
    checkpoint = mission.pause()
    assert mission.service_start is None
    assert_checkpoint(checkpoint, ('near', 'far'), (), (0, 1))
    assert_stays_paused(mission, t + partial)
    mission.observe_service_pose((1, 0, 0), CONFIG)
    assert mission.service_start is None and mission.state == 'PAUSED'


def test_pause_immediately_after_completed_service(line):
    mission = progress(line, ['near', 'far'])
    t = serve(mission, arrive(mission))
    assert mission.leg_index == 1 and mission.completed_requests == 1
    checkpoint = mission.pause()
    assert_checkpoint(checkpoint, ('near', 'far'), (0,), (1,))
    assert_stays_paused(mission, t)


def test_pause_on_return_has_no_pending_requests_but_never_reaches_depot(line):
    mission = progress(line, ['far'])
    t = serve(mission, arrive(mission))
    step(mission, (2, 0), t + TICK)
    assert step(mission, (1.5, 0), t + 2 * TICK)[:2] != (0, 0)
    assert mission.route.nodes == ('far', 'near', 'depot')
    checkpoint = mission.pause()
    assert_checkpoint(checkpoint, ('far',), (0,), ())
    assert_stays_paused(mission, t + 2 * TICK)
    remaining = plan_remaining_mission(line, 'near', checkpoint.original_requests,
                                       checkpoint.remaining_order)
    assert remaining.remaining_order == ()
    assert tuple(leg.goal for leg in remaining.legs) == ('depot',)
    assert remaining.total_distance_m == 1


@pytest.mark.parametrize('phase', ['travel', 'service', 'after_service', 'return'])
def test_repeated_pause_is_same_checkpoint_and_does_not_change_progress(line, phase):
    mission = progress(line, ['near', 'far'] if phase != 'return' else ['near'])
    if phase == 'travel':
        t = SECOND
        step(mission, (0, 0), t)
    else:
        t = arrive(mission)
        if phase != 'service':
            t = serve(mission, t)
    checkpoint = mission.pause()
    before = mission.__dict__.copy()
    assert mission.pause() is checkpoint
    assert mission.__dict__ == before
    assert_stays_paused(mission, t)
    after = mission.__dict__.copy()
    assert mission.pause() is checkpoint
    assert mission.__dict__ == after


@pytest.mark.parametrize('requests', [[], ['near']])
def test_pause_before_start_is_rejected_without_changes(line, requests):
    mission = progress(line, requests)
    before = mission.__dict__.copy()
    with pytest.raises(RuntimeError, match='started TRACKING or SERVICING'):
        mission.pause()
    assert mission.__dict__ == before
    step(mission, (0, 0), SECOND)
    assert mission.started and mission.state == ('TRACKING' if requests else 'REACHED')


@pytest.mark.parametrize('phase', ['unstarted_fault', 'started_fault', 'empty_reached', 'reached'])
def test_pause_cannot_replace_terminal_latches(line, phase):
    mission = progress(line, [] if phase == 'empty_reached' else ['near'])
    if phase == 'reached':
        t = serve(mission, arrive(mission))
        for point in mission.route.points:
            t += TICK
            step(mission, point, t)
    elif phase != 'unstarted_fault':
        step(mission, (0, 0), SECOND)
    if phase.endswith('fault'):
        mission.fail()
    terminal = mission.state
    before = mission.__dict__.copy()
    with pytest.raises(RuntimeError):
        mission.pause()
    assert mission.__dict__ == before
    mission.fail()
    assert mission.state == terminal
    assert mission.command(None, CONFIG, -1, -1) == (0, 0, terminal == 'REACHED')


def test_explicit_fault_from_pause_latches_and_checkpoint_survives(line):
    mission = progress(line, ['near'])
    t = arrive(mission)
    checkpoint = mission.pause()
    mission.fail()
    assert mission.state == 'FAULT' and mission.last_command == ZERO
    assert mission.service_start is None
    with pytest.raises(RuntimeError):
        mission.pause()
    mission.fail()
    assert step(mission, (1, 0), t + 10 * SECOND) == ZERO
    assert mission.state == 'FAULT' and mission.completed_requests == 0
    assert_checkpoint(checkpoint, ('near',), (), (0,))


@pytest.mark.parametrize('pose', [None, (0, 0), (nan, 0, 0), (0, inf, 0),
                                  (0, 0, nan), (True, 0, 0), ('0', 0, 0),
                                  (10**400, 0, 0)])
def test_pose_validation_stays_active_even_on_repeated_feedback(line, pose):
    mission = progress(line, ['near'])
    step(mission, (0, 0), SECOND)
    checkpoint = mission.pause()
    with pytest.raises(ValueError, match='odometry pose'):
        mission.command(pose, CONFIG, SECOND, SECOND + TICK)
    assert mission.state == 'FAULT' and mission.last_command == ZERO
    assert step(mission, (1, 0), 10 * SECOND) == ZERO
    assert mission.state == 'FAULT'
    with pytest.raises(RuntimeError):
        mission.pause()
    assert_checkpoint(checkpoint, ('near',), (), (0,))


@pytest.mark.parametrize('which', ['feedback', 'clock'])
@pytest.mark.parametrize('bad', [-1, True, 1.0, None, SECOND - 1])
def test_invalid_and_backward_times_fault_while_paused(line, which, bad):
    mission = progress(line, ['near'])
    step(mission, (0, 0), SECOND)
    mission.pause()
    stamp, now = (bad, SECOND) if which == 'feedback' else (SECOND, bad)
    with pytest.raises(ValueError, match='timestamps|backwards'):
        mission.command((0, 0, 0), CONFIG, stamp, now)
    assert mission.state == 'FAULT' and mission.last_command == ZERO
    assert step(mission, (1, 0), 10 * SECOND) == ZERO
    assert mission.state == 'FAULT' and mission.completed_requests == 0


@pytest.mark.parametrize('stamp,now', [(SECOND, 2 * SECOND),
                                      (2 * SECOND, SECOND), (SECOND, SECOND)])
def test_repeated_timestamps_and_fresh_recovery_never_resume(line, stamp, now):
    mission = progress(line, ['near'])
    step(mission, (0, 0), SECOND)
    checkpoint = mission.pause()
    before = execution_state(mission)
    assert mission.command((1, 0, 0), CONFIG, stamp, now) == ZERO
    assert mission.last_command == ZERO and mission.state == 'PAUSED'
    assert execution_state(mission) == before
    assert_stays_paused(mission, 3 * SECOND)
    assert mission.pause() is checkpoint


def test_backward_time_after_fresh_paused_samples_is_still_detected(line):
    mission = progress(line, ['near'])
    step(mission, (0, 0), SECOND)
    mission.pause()
    step(mission, (1, 0), 2 * SECOND)
    assert mission.last_stamp == mission.last_clock == 2 * SECOND
    with pytest.raises(ValueError, match='backwards'):
        step(mission, (1, 0), 2 * SECOND - 1)
    assert mission.state == 'FAULT'


# These are manually specified original indices, never derived from request_order.
@pytest.mark.parametrize('strategy,count,completed,remaining', [
    ('fifo', 0, (), (0, 1, 2)),
    ('fifo', 1, (0,), (1, 2)),
    ('fifo', 2, (0, 1), (2,)),
    ('fifo', 3, (0, 1, 2), ()),
    ('nn', 0, (), (1, 0, 2)),
    ('nn', 1, (1,), (0, 2)),
    ('nn', 2, (1, 0), (2,)),
    ('nn', 3, (1, 0, 2), ()),
    ('nn-2opt', 0, (), (1, 2, 0)),
    ('nn-2opt', 1, (1,), (2, 0)),
    ('nn-2opt', 2, (1, 2), (0,)),
    ('nn-2opt', 3, (1, 2, 0), ()),
])
def test_original_indices_for_all_strategies(strategy, count, completed, remaining):
    requests = ['r1c2', 'r2c0', 'r2c4']
    mission, _ = prepare_mission(GRAPH, SDF, requests, strategy)
    t, serviced = SECOND, []
    for _ in range(count):
        t = serve(mission, arrive(mission, t)) + TICK
        serviced.extend(e.request_index for e in mission.events if e.kind == 'serviced')
    # Arrive at the next goal without serving it; it must still be pending.
    if count < 3:
        t = arrive(mission, t)
    checkpoint = mission.pause()
    assert tuple(serviced) == completed
    assert_checkpoint(checkpoint, ('r1c2', 'r2c0', 'r2c4'), completed, remaining)
    requests.clear()
    assert_stays_paused(mission, t)


@pytest.mark.parametrize('strategy,count,completed,remaining', [
    ('fifo', 2, (0, 1), (2, 3, 4)),
    ('fifo', 3, (0, 1, 2), (3, 4)),
    ('nn', 1, (1,), (4, 2, 3, 0)),
    ('nn', 3, (1, 4, 2), (3, 0)),
    ('nn-2opt', 1, (1,), (4, 2, 3, 0)),
    ('nn-2opt', 3, (1, 4, 2), (3, 0)),
])
def test_duplicate_and_depot_requests_keep_distinct_identities(strategy, count,
                                                              completed, remaining):
    requests = ('r1c4', 'r0c0', 'r2c2', 'r2c2', 'r0c0')
    mission, _ = prepare_mission(GRAPH, SDF, requests, strategy)
    t = SECOND
    for _ in range(count):
        t = serve(mission, arrive(mission, t)) + TICK
    t = arrive(mission, t)
    checkpoint = mission.pause()
    assert_checkpoint(checkpoint, requests, completed, remaining)
    assert_stays_paused(mission, t)


def test_checkpoint_is_frozen_and_detached_from_mutable_inputs(line):
    requests, order = ['far', 'near'], [1, 0]
    plan = plan_fifo_mission(line, ['near', 'far'])
    points = [[list(line.nodes[n]) for n in leg.nodes] for leg in plan.legs]
    mission = MissionProgress(plan, points, original_requests=requests, request_order=order)
    t = serve(mission, arrive(mission))
    checkpoint = mission.pause()
    before_hash = hash(checkpoint)
    requests.clear()
    order.clear()
    points[0][0][0] = 99
    for field in ('original_requests', 'completed_order', 'remaining_order'):
        with pytest.raises(FrozenInstanceError):
            setattr(checkpoint, field, ())
        with pytest.raises(TypeError):
            getattr(checkpoint, field)[0] = 99
    assert_checkpoint(checkpoint, ('far', 'near'), (1,), (0,))
    assert_stays_paused(mission, t)
    mission.fail()
    assert hash(checkpoint) == before_hash
    assert_checkpoint(checkpoint, ('far', 'near'), (1,), (0,))


def test_checkpoint_connects_to_remaining_planner_without_reordering():
    mission, _ = prepare_mission(GRAPH, SDF, ['r1c2', 'r2c0', 'r2c4'], 'nn-2opt')
    serve(mission, arrive(mission))
    checkpoint = mission.pause()
    assert_checkpoint(checkpoint, ('r1c2', 'r2c0', 'r2c4'), (1,), (2, 0))
    # Explicit graph node chosen by the caller, never inferred by pause().
    plan = plan_remaining_mission(load_graph(GRAPH), 'r2c0',
                                  checkpoint.original_requests, checkpoint.remaining_order)
    assert plan.original_requests == ('r1c2', 'r2c0', 'r2c4')
    assert plan.remaining_order == (2, 0)
    assert tuple(leg.goal for leg in plan.legs) == ('r2c4', 'r1c2', 'r0c0')
    assert tuple(leg.distance_m for leg in plan.legs) == (8, 6, 6)
    assert plan.total_distance_m == 20
    assert mission.state == 'PAUSED' and mission.completed_requests == 1


def test_checkpoint_at_unserved_duplicate_retains_zero_length_service_leg(line):
    mission = progress(line, ['near', 'near', 'depot'])
    t = serve(mission, arrive(mission))
    arrive(mission, t + TICK)
    checkpoint = mission.pause()
    assert_checkpoint(checkpoint, ('near', 'near', 'depot'), (0,), (1, 2))
    plan = plan_remaining_mission(line, 'near', checkpoint.original_requests,
                                  checkpoint.remaining_order)
    assert plan.remaining_order == (1, 2)
    assert tuple(leg.goal for leg in plan.legs) == ('near', 'depot', 'depot')
    assert tuple(leg.distance_m for leg in plan.legs) == (0, 1, 0)
