"""Exercise the installed FIFO mission CLI from temporary working directories."""

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
