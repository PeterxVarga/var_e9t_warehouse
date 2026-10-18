"""Exercise installed mission CLI strategies from temporary working directories."""

import json
from pathlib import Path
import subprocess

import pytest

pytest.importorskip('ament_index_python')
from ament_index_python.packages import get_package_prefix, get_package_share_directory


@pytest.fixture
def cli(tmp_path):
    executable = (Path(get_package_prefix('warehouse_planning'))
                  / 'lib' / 'warehouse_planning' / 'mission_example')

    def run(*args):
        result = subprocess.run(['ros2', 'run', 'warehouse_planning', 'mission_example', *args],
                                cwd=tmp_path, capture_output=True, text=True, timeout=15)
        if result.returncode:
            # Humble's ros2run writes its own exit summary to stdout. Also
            # exercise the installed entrypoint to verify the CLI's streams.
            direct = subprocess.run([str(executable), *args], cwd=tmp_path,
                                    capture_output=True, text=True, timeout=15)
            assert direct.returncode == result.returncode
            assert direct.stdout == ''
            assert direct.stderr == result.stderr
        return result
    return run


@pytest.fixture
def graph_data():
    path = (Path(get_package_share_directory('warehouse_planning'))
            / 'config' / 'warehouse_graph.json')
    return json.loads(path.read_text())


def write_graph(tmp_path, data):
    path = tmp_path / 'graph.json'
    path.write_text(json.dumps(data))
    return str(path)


def assert_failure(result, code, message):
    assert result.returncode == code, (result.stdout, result.stderr)
    assert result.stdout == f'[ros2run]: Process exited with failure {code}\n'
    assert message in result.stderr, result.stderr


def test_installed_warehouse_mission(cli):
    result = cli('--pickups', 'r2c2', 'r1c4')
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines() == [
        'Frame: world',
        'Pickup requests: r2c2 -> r1c4',
        'Pickup 1: r0c0 -> r2c2; distance: 8 m',
        'Pickup 2: r2c2 -> r1c4; distance: 6 m',
        'Return: r1c4 -> r0c0; distance: 10 m',
        'Nodes: r0c0 -> r1c0 -> r2c0 -> r2c1 -> r2c2 -> r2c3 -> r2c4'
        ' -> r1c4 -> r0c4 -> r0c3 -> r0c2 -> r0c1 -> r0c0',
        'Total planned distance: 24 m',
    ]


def test_explicit_empty_mission(cli):
    result = cli('--pickups')
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines() == [
        'Frame: world', 'Pickup requests: (none)',
        'Nodes: r0c0', 'Total planned distance: 0 m',
    ]


def test_repeated_requests_remain_numbered_in_fifo_order(cli):
    result = cli('--pickups', 'r2c2', 'r2c2', 'r1c4')
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines()[:6] == [
        'Frame: world', 'Pickup requests: r2c2 -> r2c2 -> r1c4',
        'Pickup 1: r0c0 -> r2c2; distance: 8 m',
        'Pickup 2: r2c2 -> r2c2; distance: 0 m',
        'Pickup 3: r2c2 -> r1c4; distance: 6 m',
        'Return: r1c4 -> r0c0; distance: 10 m',
    ]
    assert 'r2c2 -> r2c2' not in result.stdout.splitlines()[-2]
    assert result.stdout.splitlines()[-1] == 'Total planned distance: 24 m'


def test_override_graph_and_pass_through_later_pickup(cli, tmp_path):
    data = dict(schema_version=1, frame='world', depot='depot',
                nodes=[dict(id='depot', x=-4, y=-3), dict(id='near', x=-2, y=-3),
                       dict(id='far', x=0, y=-3)],
                edges=[['depot', 'near'], ['near', 'far']])
    result = cli('--graph', write_graph(tmp_path, data), '--pickups', 'far', 'near')
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines() == [
        'Frame: world', 'Pickup requests: far -> near',
        'Pickup 1: depot -> far; distance: 4 m',
        'Pickup 2: far -> near; distance: 2 m',
        'Return: near -> depot; distance: 2 m',
        'Nodes: depot -> near -> far -> near -> depot',
        'Total planned distance: 8 m',
    ]


def test_unknown_pickup_has_no_partial_plan(cli):
    assert_failure(cli('--pickups', 'r2c2', 'missing'), 1,
                   'Mission planning failed: unknown pickup node in request 2: missing')


def test_later_unreachable_pickup_has_no_partial_plan(cli, tmp_path, graph_data):
    graph_data['edges'] = [edge for edge in graph_data['edges'] if 'r1c4' not in edge]
    result = cli('--pickups', 'r2c2', 'r1c4', '--graph', write_graph(tmp_path, graph_data))
    assert_failure(result, 1, 'mission leg 2 (r2c2 -> r1c4) is unreachable')
    assert result.stderr.startswith('Mission planning failed:')


@pytest.mark.parametrize('contents', ['{', '{"schema_version": 2}'])
def test_invalid_graph_file(cli, tmp_path, contents):
    path = tmp_path / 'invalid.json'
    path.write_text(contents)
    assert_failure(cli('--graph', str(path), '--pickups'), 1, 'Mission planning failed:')


def test_missing_graph_file(cli, tmp_path):
    assert_failure(cli('--pickups', '--graph', str(tmp_path / 'missing.json')), 1,
                   'Mission planning failed:')


@pytest.mark.parametrize('pickups', [[], ['r2c2', 'r1c4']])
def test_entire_graph_checked_even_for_empty_mission(cli, tmp_path, graph_data, pickups):
    graph_data['edges'].append(['r0c0', 'r3c2'])
    result = cli('--graph', write_graph(tmp_path, graph_data), '--pickups', *pickups)
    assert_failure(result, 1, 'edge r0c0--r3c2 intersects')
    assert result.stderr.startswith('Mission planning failed:')


def test_help_does_not_plan(cli):
    result = cli('--help')
    assert result.returncode == 0 and result.stderr == ''
    assert '--pickups' in result.stdout and '--graph' in result.stdout
    assert '--strategy {fifo,nn,nn-2opt}' in result.stdout
    assert 'Total planned distance:' not in result.stdout


@pytest.mark.parametrize('args,message', [
    ([], 'required: --pickups'),
    (['--pickups', '--graph'], 'expected one argument'),
    (['--pickups', '--unknown'], 'unrecognized arguments'),
    (['--pickup', 'r2c2'], 'required: --pickups'),
])
def test_argparse_errors(cli, args, message):
    result = cli(*args)
    assert_failure(result, 2, message)
    assert 'usage:' in result.stderr


@pytest.mark.parametrize('strategy,order,distances,total', [
    ('fifo', (0, 1, 2), (6, 6, 8, 12), 32),
    ('nn', (1, 0, 2), (4, 6, 6, 12), 28),
    ('nn-2opt', (1, 2, 0), (4, 8, 6, 6), 24),
])
def test_strategy_comparison_includes_depot_return(cli, strategy, order, distances, total):
    requests = ('r1c2', 'r2c0', 'r2c4')
    result = cli('--strategy', strategy, '--pickups', *requests)
    assert result.returncode == 0 and result.stderr == ''
    lines = result.stdout.splitlines()
    assert lines[0] == 'Frame: world'
    if strategy == 'fifo':
        assert not any('Strategy:' in line or 'Execution order:' in line for line in lines)
        offset = 2
    else:
        assert lines[1:4] == [
            f'Strategy: {strategy}',
            'Original pickup requests: 1: r1c2 -> 2: r2c0 -> 3: r2c4',
            'Execution order: ' + ' -> '.join(f'{i + 1}: {requests[i]}' for i in order),
        ]
        offset = 5
    assert lines[offset - 1] == 'Pickup requests: ' + ' -> '.join(requests[i] for i in order)
    start = 'r0c0'
    for position, request in enumerate(order):
        label = f'Pickup {position + 1}'
        if strategy != 'fifo':
            label += f' (request {request + 1})'
        assert lines[offset + position] == (
            f'{label}: {start} -> {requests[request]}; distance: {distances[position]} m')
        start = requests[request]
    assert lines[offset + 3] == f'Return: {start} -> r0c0; distance: {distances[-1]} m'
    assert lines[-2].startswith('Nodes: r0c0 -> ') and lines[-2].endswith(' -> r0c0')
    assert lines[-1] == f'Total planned distance: {total} m'


@pytest.mark.parametrize('requests', [
    (), ('r2c2', 'r1c4'), ('r1c2', 'r2c0', 'r2c4'),
    ('r1c4', 'r0c0', 'r2c2', 'r2c2', 'r0c0'),
])
def test_default_and_explicit_fifo_have_identical_output(cli, requests):
    default = cli('--pickups', *requests)
    explicit = cli('--strategy', 'fifo', '--pickups', *requests)
    assert default.returncode == explicit.returncode == 0
    assert default.stderr == explicit.stderr == ''
    assert default.stdout == explicit.stdout


def test_fifo_preserves_repeated_and_depot_request_positions(cli):
    result = cli('--strategy', 'fifo', '--pickups',
                 'r1c4', 'r0c0', 'r2c2', 'r2c2', 'r0c0')
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines()[:8] == [
        'Frame: world', 'Pickup requests: r1c4 -> r0c0 -> r2c2 -> r2c2 -> r0c0',
        'Pickup 1: r0c0 -> r1c4; distance: 10 m',
        'Pickup 2: r1c4 -> r0c0; distance: 10 m',
        'Pickup 3: r0c0 -> r2c2; distance: 8 m',
        'Pickup 4: r2c2 -> r2c2; distance: 0 m',
        'Pickup 5: r2c2 -> r0c0; distance: 8 m',
        'Return: r0c0 -> r0c0; distance: 0 m',
    ]
    assert result.stdout.splitlines()[-1] == 'Total planned distance: 36 m'


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
def test_optimized_empty_mission_has_no_invented_legs(cli, strategy):
    result = cli('--strategy', strategy, '--pickups')
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines() == [
        'Frame: world', f'Strategy: {strategy}',
        'Original pickup requests: (none)', 'Execution order: (none)',
        'Pickup requests: (none)', 'Nodes: r0c0', 'Total planned distance: 0 m',
    ]


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
def test_optimized_repeated_and_depot_requests_keep_original_ids(cli, strategy):
    result = cli('--pickups', 'r1c4', 'r0c0', 'r2c2', 'r2c2', 'r0c0',
                 '--strategy', strategy)
    assert result.returncode == 0 and result.stderr == ''
    assert result.stdout.splitlines()[:11] == [
        'Frame: world', f'Strategy: {strategy}',
        'Original pickup requests: 1: r1c4 -> 2: r0c0 -> 3: r2c2 -> 4: r2c2 -> 5: r0c0',
        'Execution order: 2: r0c0 -> 5: r0c0 -> 3: r2c2 -> 4: r2c2 -> 1: r1c4',
        'Pickup requests: r0c0 -> r0c0 -> r2c2 -> r2c2 -> r1c4',
        'Pickup 1 (request 2): r0c0 -> r0c0; distance: 0 m',
        'Pickup 2 (request 5): r0c0 -> r0c0; distance: 0 m',
        'Pickup 3 (request 3): r0c0 -> r2c2; distance: 8 m',
        'Pickup 4 (request 4): r2c2 -> r2c2; distance: 0 m',
        'Pickup 5 (request 1): r2c2 -> r1c4; distance: 6 m',
        'Return: r1c4 -> r0c0; distance: 10 m',
    ]
    assert result.stdout.splitlines()[-1] == 'Total planned distance: 24 m'


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
def test_optimized_unknown_pickup_has_no_partial_plan(cli, strategy):
    assert_failure(cli('--strategy', strategy, '--pickups', 'r2c2', 'missing'), 1,
                   'Mission planning failed: unknown pickup node in request 2: missing')


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
def test_optimized_unreachable_pickup_has_no_partial_plan(cli, tmp_path, graph_data, strategy):
    graph_data['edges'] = [edge for edge in graph_data['edges'] if 'r1c4' not in edge]
    result = cli('--strategy', strategy, '--pickups', 'r2c2', 'r1c4',
                 '--graph', write_graph(tmp_path, graph_data))
    assert_failure(result, 1, 'is unreachable')
    assert result.stderr.startswith('Mission planning failed:')


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
@pytest.mark.parametrize('contents', [None, '{', '{"schema_version": 2}'])
def test_optimized_graph_loading_errors(cli, tmp_path, strategy, contents):
    path = tmp_path / 'invalid.json'
    if contents is not None:
        path.write_text(contents)
    assert_failure(cli('--strategy', strategy, '--graph', str(path), '--pickups'), 1,
                   'Mission planning failed:')


@pytest.mark.parametrize('strategy', ['nn', 'nn-2opt'])
@pytest.mark.parametrize('pickups', [[], ['r2c2', 'missing']])
def test_optimized_geometry_checked_before_ordering_even_when_empty(
        cli, tmp_path, graph_data, strategy, pickups):
    graph_data['edges'].append(['r0c0', 'r3c2'])
    result = cli('--strategy', strategy, '--graph', write_graph(tmp_path, graph_data),
                 '--pickups', *pickups)
    assert_failure(result, 1, 'edge r0c0--r3c2 intersects')
    assert result.stderr.startswith('Mission planning failed:')


@pytest.mark.parametrize('args,message', [
    (['--pickups', '--strategy', 'bad'], 'invalid choice'),
    (['--pickups', '--strategy', 'NN'], 'invalid choice'),
    (['--pickups', '--strategy'], 'expected one argument'),
    (['--strategy', 'nn'], 'required: --pickups'),
])
def test_strategy_argument_errors_preserve_stream_contract(cli, args, message):
    result = cli(*args)
    assert_failure(result, 2, message)
    assert 'usage:' in result.stderr
