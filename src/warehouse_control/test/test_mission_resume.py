"""Atomic node-based resumption, original identities and fresh execution time."""

from math import inf, nan, pi

import pytest

import warehouse_control.mission as mission_module
from warehouse_control.control import ControlConfig
from warehouse_control.mission import MissionProgress, prepare_mission
from warehouse_control.route import PlanarTransform
from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.roadmap import graph_from_data, graph_without_edges, load_graph
from test_route import GRAPH, SDF


CONFIG = ControlConfig()
TRANSFORM = PlanarTransform(0, 0, 0)
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
    return MissionProgress(plan, [[graph.nodes[n] for n in leg.nodes] for leg in plan.legs])


class Driver:
    def __init__(self, mission):
        self.mission = mission
        self.time = SECOND

    def step(self, point, elapsed=TICK):
        self.time += elapsed
        return self.mission.command((*point, 0), CONFIG, self.time, self.time)

    def arrive(self):
        for point in self.mission.route.points[self.mission.route.index:]:
            assert self.step(point) == ZERO
        assert self.mission.state == 'SERVICING'

    def serve(self):
        self.arrive()
        assert self.step(self.mission.route.points[-1], SECOND) == ZERO
        assert self.mission.state == 'TRACKING'
        return self.mission.events[-1]

    def resume(self, graph, node, transform=TRANSFORM):
        self.time += TICK
        point = transform.world_to_odom(graph.nodes[node])
        assert self.mission.resume_from_node(
            graph, node, transform, (*point, pi), CONFIG, self.time, self.time) is None
        assert self.mission.last_command == ZERO
        assert self.mission.events == ()


def paused(graph, requests=('far', 'near')):
    mission = progress(graph, requests)
    driver = Driver(mission)
    driver.step((0, 0))
    mission.pause()
    return mission, driver


def snapshot(mission):
    # Include object identity and the mutable RouteProgress internals, not just
    # bookkeeping values: a failed candidate must not advance an old route.
    return (mission.__dict__.copy(),
            tuple((id(r), r.__dict__.copy()) for r in mission._routes))


@pytest.mark.parametrize('strategy,order', [
    ('fifo', (0, 1, 2)), ('nn', (1, 0, 2)), ('nn-2opt', (1, 2, 0)),
])
def test_all_strategies_keep_original_order_across_multiple_resumes(strategy, order):
    requests = ('r1c2', 'r2c0', 'r2c4')
    mission, transform = prepare_mission(GRAPH, SDF, requests, strategy)
    graph, driver = load_graph(GRAPH), Driver(mission)
    completed = []
    for count, request in enumerate(order):
        # Complete one request, replace the plan, then interrupt the next
        # travel before doing any further service. Never optimize again.
        event = driver.serve()
        completed.append(event.request_index)
        assert event.leg == 0
        checkpoint = mission.pause()
        assert checkpoint.completed_order == tuple(completed)
        assert checkpoint.remaining_order == order[count + 1:]
        driver.resume(graph, requests[request], transform)
        assert mission.leg_index == 0
        assert mission.completed_requests == count + 1
        assert mission.request_order == order
        assert mission.strategy == strategy
        assert mission.plan.original_requests == requests
        assert mission.plan.remaining_order == order[count + 1:]
        assert tuple(leg.goal for leg in mission.plan.legs) == (
            *(requests[i] for i in order[count + 1:]), graph.depot)
        again = mission.pause()
        assert again == checkpoint
        assert mission.pause() is again
        driver.resume(graph, requests[request], transform)
    assert tuple(completed) == order
    assert mission.request_index(0) is None  # Only the depot return remains.
    for point in mission.route.points:
        result = driver.step(point)
    assert result == (0.0, 0.0, True)
    assert mission.events[-1].kind == 'depot'
    assert mission.events[-1].request_index is None
    assert mission.completed_requests == 3
    assert mission.state == 'REACHED'


def test_duplicate_and_depot_requests_remain_distinct_after_partial_service(line):
    mission = progress(line, ['near', 'near', 'depot', 'near'])
    driver = Driver(mission)
    events = [driver.serve()]
    driver.arrive()
    driver.step((1, 0), SECOND // 2)
    checkpoint = mission.pause()
    assert checkpoint.completed_order == (0,)
    assert checkpoint.remaining_order == (1, 2, 3)
    driver.resume(line, 'near')
    assert mission.plan.legs[0].nodes == ('near',)
    assert mission.request_index(0) == 1
    for node in ('near', 'depot', 'near'):
        events.append(driver.serve())
        checkpoint = mission.pause()
        count = len(events)
        assert checkpoint.completed_order == tuple(range(count))
        assert checkpoint.remaining_order == tuple(range(count, 4))
        driver.resume(line, node)
    assert [e.request_index for e in events] == [0, 1, 2, 3]
    assert [e.leg for e in events] == [0, 0, 0, 0]
    assert mission.completed_requests == 4


def test_multiple_services_in_one_replacement_use_active_leg_and_global_count(line):
    mission = progress(line, ['far', 'near', 'near', 'depot'])
    driver = Driver(mission)
    driver.serve()
    mission.pause()
    driver.resume(line, 'far')
    events = [driver.serve(), driver.serve()]
    assert [(e.leg, e.request_index) for e in events] == [(0, 1), (1, 2)]
    assert mission.leg_index == 2 and mission.completed_requests == 3
    checkpoint = mission.pause()
    assert checkpoint.completed_order == (0, 1, 2)
    assert checkpoint.remaining_order == (3,)
    driver.resume(line, 'near')
    event = driver.serve()
    assert (event.leg, event.request_index) == (0, 3)
    assert mission.completed_requests == 4


def test_interrupted_service_restarts_full_wait_and_resume_does_not_arrive(line):
    mission = progress(line, ['near'])
    driver = Driver(mission)
    driver.arrive()
    driver.step((1, 0), SECOND - 1)
    mission.pause()
    driver.resume(line, 'near')
    assert mission.state == 'TRACKING' and mission.service_start is None
    # Even a long pause or large time jump cannot count toward the new wait.
    assert driver.step((1, 0), 10 * SECOND) == ZERO
    assert mission.events[-1].kind == 'arrival'
    assert mission.state == 'SERVICING'
    assert driver.step((1, 0), SECOND - 1) == ZERO
    assert mission.completed_requests == 0
    assert driver.step((1, 0), 1) == ZERO
    assert mission.completed_requests == 1
    assert mission.events[-1].kind == 'serviced'


def test_interrupted_travel_uses_explicit_node_and_never_replays_speed(line):
    mission = progress(line, ['far'])
    driver = Driver(mission)
    driver.step((0, 0))
    assert driver.step((0.2, 0))[:2] != (0, 0)
    mission.pause()
    driver.resume(line, 'near')
    assert mission.plan.nodes == ('near', 'far', 'near', 'depot')
    assert mission.route.index == 0 and not mission.route.reached
    assert mission.command((1, 0, 0), CONFIG, driver.time, driver.time) == ZERO
    assert mission.route.index == 0
    assert driver.step((1, 0)) == ZERO
    assert mission.route.index == 1
    assert driver.step((1, 0))[:2] != (0, 0)


def test_rotated_translated_frame_and_arbitrary_yaw(line):
    mission, driver = paused(line)
    transform = PlanarTransform(10, 20, pi / 2)
    # world (1, 0) maps to odom (-20, 9), not (1, 0).
    mission.resume_from_node(line, 'near', transform, (-20, 9, -2.7), CONFIG,
                             driver.time + TICK, driver.time + TICK)
    assert mission.points[0][0] == pytest.approx((-20, 9))
    assert mission.points[0][1] == pytest.approx((-20, 8))
    assert mission.started and mission.state == 'TRACKING'


@pytest.mark.parametrize('offset,accepted', [(0.05, True), (0.050001, False)])
def test_existing_position_tolerance_is_inclusive(line, offset, accepted):
    mission, driver = paused(line)
    # Use the depot to avoid subtraction rounding around the 0.05 boundary.
    args = (line, 'depot', TRANSFORM, (offset, 0, pi), CONFIG,
            driver.time + TICK, driver.time + TICK)
    before = snapshot(mission)
    if accepted:
        mission.resume_from_node(*args)
        assert mission.state == 'TRACKING'
    else:
        with pytest.raises(ValueError, match='does not match'):
            mission.resume_from_node(*args)
        assert snapshot(mission) == before


def test_configured_position_tolerance_is_used(line):
    mission, driver = paused(line)
    mission.resume_from_node(line, 'near', TRANSFORM, (1.1, 0, 2),
                             ControlConfig(goal_tolerance=0.2),
                             driver.time + TICK, driver.time + TICK)
    assert mission.last_command == ZERO


def test_closed_edge_detour_is_planned_for_all_legs_before_execution():
    graph = graph_from_data(dict(
        schema_version=1, frame='world', depot='d',
        nodes=[dict(id=n, x=x, y=y) for n, x, y in
               [('d', 0, 0), ('a', 1, 0), ('b', 2, 0), ('c', 1, 1)]],
        edges=[['d', 'a'], ['a', 'b'], ['a', 'c'], ['c', 'b']]))
    mission, driver = paused(graph, ('b', 'a'))
    closed = graph_without_edges(graph, [('a', 'b')])
    driver.resume(closed, 'a')
    assert tuple(leg.nodes for leg in mission.plan.legs) == (
        ('a', 'c', 'b'), ('b', 'c', 'a'), ('a', 'd'))
    assert len(mission._routes) == 3
    assert mission.plan.remaining_order == (0, 1)
    assert mission.last_command == ZERO


@pytest.mark.parametrize('requests', [('far',), ('near', 'far'), ()])
def test_unreachable_late_leg_or_return_leaves_everything_unchanged(line, requests):
    mission, driver = paused(line, requests or ('near',))
    if not requests:
        driver.resume(line, 'depot')
        driver.serve()
        mission.pause()
    closed = graph_without_edges(line, [('depot', 'near')] if not requests
                                 else [('near', 'far')])
    before = snapshot(mission)
    with pytest.raises(ValueError, match='unreachable'):
        driver.resume(closed, 'near')
    assert snapshot(mission) == before
    assert mission.state == 'PAUSED' and mission.last_command == ZERO
    assert driver.step((1, 0), 2 * SECOND) == ZERO


@pytest.mark.parametrize('node,legs', [('near', (('near', 'depot'),)), ('depot', ())])
def test_no_remaining_requests_return_or_finish_without_motion(line, node, legs):
    mission = progress(line, ['near'])
    driver = Driver(mission)
    driver.serve()
    checkpoint = mission.pause()
    driver.resume(line, node)
    assert checkpoint.remaining_order == ()
    assert tuple(leg.nodes for leg in mission.plan.legs) == legs
    assert mission.completed_requests == 1
    assert mission.last_command == ZERO
    if legs:
        assert driver.step((1, 0)) == ZERO
        assert driver.step((1, 0))[:2] != (0, 0)
        assert driver.step((0, 0)) == (0.0, 0.0, True)
    else:
        assert mission.route is None and mission.points == ()
        assert driver.step((0, 0)) == (0.0, 0.0, True)
    assert mission.state == 'REACHED'
    assert mission.events[-1].kind == 'depot'
    assert mission.events[-1].request_index is None


@pytest.mark.parametrize('state,started', [
    ('TRACKING', False), ('PAUSED', False), ('TRACKING', True),
    ('SERVICING', True), ('REACHED', True), ('FAULT', True),
])
def test_resume_requires_started_paused_and_preserves_terminal_latches(line, state, started):
    mission = progress(line, ['near'])
    driver = Driver(mission)
    if started:
        driver.step((0, 0))
    if state == 'SERVICING':
        driver.arrive()
    elif state == 'FAULT':
        mission.pause()
        mission.fail()
    elif state == 'REACHED':
        driver.serve()
        for point in mission.route.points:
            driver.step(point)
    elif state == 'PAUSED':
        mission.state = 'PAUSED'  # An unstarted object is not a resumable mission.
    assert mission.state == state and mission.started == started
    before = snapshot(mission)
    with pytest.raises(RuntimeError, match='started PAUSED'):
        driver.resume(line, 'near')
    assert snapshot(mission) == before
    if state in ('FAULT', 'REACHED'):
        assert mission.command(None, None, None, None) == (0.0, 0.0, state == 'REACHED')
        assert mission.state == state


@pytest.mark.parametrize('field,value', [
    ('graph', None), ('node', 'missing'), ('node', None), ('node', True),
    ('node', ['near']), ('pose', None), ('pose', (1, 0)),
    ('pose', (1, 0, nan)), ('pose', (1, inf, 0)), ('pose', (True, 0, 0)),
    ('pose', (1, 0, '0')), ('pose', (10**400, 0, 0)),
    ('pose', (0, 0, 0)), ('pose', (1.5, 0, 0)),
    ('transform', None), ('transform', (0, 0, 0)),
    ('config', None), ('config', 0.05),
])
def test_bad_graph_node_pose_transform_or_mid_edge_is_atomic(line, field, value):
    mission, driver = paused(line)
    args = dict(graph=line, start_node='near', transform=TRANSFORM, pose=(1, 0, 0),
                config=CONFIG, feedback_stamp=driver.time + TICK,
                clock_stamp=driver.time + TICK)
    args[{'node': 'start_node'}.get(field, field)] = value
    before = snapshot(mission)
    with pytest.raises(ValueError):
        mission.resume_from_node(**args)
    assert snapshot(mission) == before
    assert mission.pause() is before[0]['_checkpoint']
    assert mission.last_command == ZERO
    driver.resume(line, 'near')  # A rejected candidate does not latch FAULT.


def test_finite_transform_overflow_rejects_without_replacement(line):
    mission, driver = paused(line)
    before = snapshot(mission)
    with pytest.raises(ValueError, match='finite'):
        mission.resume_from_node(line, 'near', PlanarTransform(1.7e308, 1.7e308, pi / 4),
                                 (1, 0, 0), CONFIG, driver.time + TICK, driver.time + TICK)
    assert snapshot(mission) == before


@pytest.mark.parametrize('repeated', ['feedback', 'clock'])
def test_one_repeated_timestamp_after_resume_cannot_arrive_or_complete_service(line, repeated):
    mission, driver = paused(line)
    driver.resume(line, 'near')
    stamp, now = driver.time, driver.time
    times = (stamp, now + TICK) if repeated == 'feedback' else (stamp + TICK, now)
    assert mission.command((1, 0, 0), CONFIG, *times) == ZERO
    assert mission.route.index == 0 and mission.state == 'TRACKING'
    driver.time += TICK
    driver.arrive()
    stamp, now = driver.time, driver.time
    times = (stamp, now + 2 * SECOND) if repeated == 'feedback' else (stamp + 2 * SECOND, now)
    assert mission.command((2, 0, 0), CONFIG, *times) == ZERO
    assert mission.state == 'SERVICING' and mission.completed_requests == 0
    assert mission.events == ()


@pytest.mark.parametrize('value', [None, True, 1.0, '2', -1, nan, inf, 'same', 'older'])
@pytest.mark.parametrize('field', ['feedback_stamp', 'clock_stamp'])
def test_invalid_or_nonfresh_timestamps_preserve_paused_state(line, field, value):
    mission, driver = paused(line)
    driver.step((1, 0), SECOND)  # Paused processing also sets the lower bound.
    args = dict(graph=line, start_node='near', transform=TRANSFORM, pose=(1, 0, 0),
                config=CONFIG, feedback_stamp=driver.time + TICK,
                clock_stamp=driver.time + TICK)
    args[field] = driver.time if value == 'same' else driver.time - 1 if value == 'older' else value
    before = snapshot(mission)
    with pytest.raises(ValueError):
        mission.resume_from_node(**args)
    assert snapshot(mission) == before
    assert mission.last_command == ZERO


@pytest.mark.parametrize('field', ['feedback_stamp', 'clock_stamp'])
def test_monotonicity_fault_after_resume_stays_latched(line, field):
    mission, driver = paused(line)
    driver.resume(line, 'near')
    times = dict(feedback_stamp=driver.time + TICK, clock_stamp=driver.time + TICK)
    times[field] = driver.time - 1
    with pytest.raises(ValueError, match='backwards'):
        mission.command((1, 0, 0), CONFIG, **times)
    assert mission.state == 'FAULT' and mission.last_command == ZERO
    with pytest.raises(RuntimeError):
        driver.resume(line, 'near')
    assert driver.step((1, 0)) == ZERO


@pytest.mark.parametrize('failure', ['transform', 'route'])
def test_late_candidate_validation_failure_cannot_replace_active_objects(line, monkeypatch, failure):
    mission, driver = paused(line)
    before = snapshot(mission)
    calls = []
    original = (PlanarTransform.world_to_odom if failure == 'transform'
                else mission_module.RouteProgress)

    def checked(*args):
        calls.append(args)
        if len(calls) == (5 if failure == 'transform' else 3):
            raise ValueError('late candidate validation failed')
        return original(*args)

    with monkeypatch.context() as patch:
        if failure == 'transform':
            patch.setattr(PlanarTransform, 'world_to_odom', checked)
        else:
            patch.setattr(mission_module, 'RouteProgress', checked)
        with pytest.raises(ValueError, match='late candidate'):
            driver.resume(line, 'near')
    assert snapshot(mission) == before
    assert mission.state == 'PAUSED' and mission.last_command == ZERO
    driver.resume(line, 'near')
