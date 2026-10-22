"""Signal coverage across the estimator lifecycle; real failures stay visible."""
import signal
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
import warehouse_control.heading_odometry_node as module


@pytest.fixture
def lifecycle(monkeypatch):
    events = []
    originals = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    node = SimpleNamespace(destroy_node=lambda: events.append('destroy'))
    monkeypatch.setattr(module, 'HeadingNode', lambda: node)
    monkeypatch.setattr(rclpy, 'init', lambda **kw: events.append(('init', kw)))
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: events.append('shutdown'))
    yield events, node
    assert all(signal.getsignal(s) == h for s, h in originals.items())


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('phase', ['init', 'construct', 'spin'])
def test_signal_requests_bounded_exit_and_survives_repeated_cleanup_signals(
        lifecycle, monkeypatch, signum, phase):
    events, node = lifecycle

    def interrupt():
        signal.raise_signal(signum)
        events.append('signal returned')

    def initialize(**kwargs):
        assert kwargs == dict(args=[], signal_handler_options=SignalHandlerOptions.NO)
        events.append('init')
        if phase == 'init':
            interrupt()

    def construct():
        events.append('construct')
        if phase == 'construct':
            interrupt()
        return node

    def spin(actual, timeout_sec):
        assert actual is node and timeout_sec == .1
        events.append('spin')
        interrupt()

    def destroy():
        events.append('destroy')
        signal.raise_signal(signal.SIGINT)
        signal.raise_signal(signal.SIGTERM)

    def shutdown():
        events.append('shutdown')
        signal.raise_signal(signal.SIGINT)
        signal.raise_signal(signal.SIGTERM)

    monkeypatch.setattr(rclpy, 'init', initialize)
    monkeypatch.setattr(module, 'HeadingNode', construct)
    monkeypatch.setattr(rclpy, 'spin_once', spin)
    monkeypatch.setattr(rclpy, 'try_shutdown', shutdown)
    node.destroy_node = destroy
    assert module.main([]) == 0
    prefix = ['init'] + ([] if phase == 'init' else ['construct'])
    if phase == 'spin':
        prefix += ['spin']
    assert events == prefix + ['signal returned'] + (
        [] if phase == 'init' else ['destroy']) + ['shutdown']


@pytest.mark.parametrize('error', [KeyboardInterrupt, ExternalShutdownException])
def test_executor_interrupt_still_cleans_up(lifecycle, monkeypatch, error):
    events, _ = lifecycle

    def spin(*args, **kwargs):
        raise error()

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    assert module.main([]) == 0
    assert events[-2:] == ['destroy', 'shutdown']


@pytest.mark.parametrize('phase', ['init', 'construct', 'spin', 'destroy', 'shutdown'])
def test_runtime_errors_propagate_and_handlers_are_restored(lifecycle, monkeypatch, phase):
    events, node = lifecycle

    def fail(*args, **kwargs):
        raise RuntimeError(f'{phase} failed')

    def interrupt(*args, **kwargs):
        signal.raise_signal(signal.SIGTERM)

    monkeypatch.setattr(rclpy, 'spin_once', interrupt)
    if phase == 'construct':
        monkeypatch.setattr(module, 'HeadingNode', fail)
    elif phase == 'destroy':
        node.destroy_node = fail
    else:
        monkeypatch.setattr(rclpy, dict(init='init', spin='spin_once',
                                      shutdown='try_shutdown')[phase], fail)
    with pytest.raises(RuntimeError, match=f'{phase} failed'):
        module.main([])
    if phase != 'shutdown':
        assert events[-1] == 'shutdown'


def test_configuration_error_remains_nonzero(lifecycle, monkeypatch, capsys):
    def invalid():
        raise ValueError('invalid estimator configuration')

    monkeypatch.setattr(module, 'HeadingNode', invalid)
    assert module.main([]) == 1
    assert 'Heading odometry failed: invalid estimator configuration' in capsys.readouterr().err
