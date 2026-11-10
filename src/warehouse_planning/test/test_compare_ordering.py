"""Comparison of complete depot tours and bounded exact reference."""

import itertools
import json
from pathlib import Path

import pytest

from warehouse_planning import compare_ordering as comparison
from warehouse_planning.roadmap import graph_from_data, load_graph


@pytest.fixture
def square():
    return graph_from_data({
        "schema_version": 1, "frame": "world", "depot": "a",
        "nodes": [{"id": "a", "x": 0, "y": 0},
                  {"id": "b", "x": 2, "y": 0},
                  {"id": "c", "x": 2, "y": 2},
                  {"id": "d", "x": 0, "y": 2}],
        "edges": [["a", "b"], ["b", "c"], ["c", "d"], ["d", "a"]],
    })


def test_square_known_optimum_and_depot_return(square):
    report = comparison.compare(square, ["b", "d", "c"], repeat=2, exact=True)
    rows = {row["strategy"]: row for row in report["strategies"]}
    assert [rows[name]["distance_m"] for name in ("fifo", "nn", "nn-2opt")] == [12, 8, 8]
    assert report["reference"]["distance_m"] == 8
    assert report["reference"]["permutations_evaluated"] == 6
    assert rows["fifo"]["distance_gap_percent"] == 50
    for row in rows.values():
        assert row["route_nodes"][0] == row["route_nodes"][-1] == "a"
        assert sorted(row["request_ids"]) == [1, 2, 3]
        assert len(row["planning_time_ns_samples"]) == 2


def test_warehouse_historical_fixture(square):
    path = Path(__file__).resolve().parents[1] / "config" / "warehouse_graph.json"
    graph = load_graph(path)
    report = comparison.compare(graph, ["r1c2", "r2c0", "r2c4"], repeat=1, exact=True)
    assert [row["distance_m"] for row in report["strategies"]] == [32, 28, 24]
    assert report["reference"]["distance_m"] == 24


def test_empty_zero_reference_has_no_percentage(square):
    report = comparison.compare(square, [], repeat=1, exact=True)
    assert report["reference"] == {"distance_m": 0.0, "request_ids": [], "permutations_evaluated": 1}
    assert all(row["distance_m"] == 0 and row["distance_gap_percent"] is None
               for row in report["strategies"])


def test_duplicates_are_separate_and_inputs_unchanged(square):
    requests = ["c", "a", "c"]
    snapshot = tuple(requests)
    report = comparison.compare(square, requests, repeat=1, exact=True)
    assert tuple(requests) == snapshot
    assert report["reference"]["permutations_evaluated"] == 6
    assert all(sorted(row["request_ids"]) == [1, 2, 3] for row in report["strategies"])
    assert all(len(row["leg_distances_m"]) == 4 for row in report["strategies"])


@pytest.mark.parametrize("requests", [["b"], ["b", "d"], ["b", "d", "c"],
                                      ["c", "a", "c", "b"]])
def test_exact_bounded_by_all_strategies_and_two_opt_by_nn(square, requests):
    report = comparison.compare(square, requests, repeat=1, exact=True)
    rows = {r["strategy"]: r for r in report["strategies"]}
    assert all(row["distance_m"] >= report["reference"]["distance_m"]
               for row in rows.values())
    assert rows["nn-2opt"]["distance_m"] <= rows["nn"]["distance_m"]


@pytest.mark.parametrize("repeat", [0, -1, 101, True, 1.5])
def test_invalid_repeat(square, repeat):
    with pytest.raises(ValueError, match="repeat"):
        comparison.compare(square, ["b"], repeat=repeat)


def test_exact_limit_and_unknown_node(square):
    with pytest.raises(ValueError, match="at most"):
        comparison.compare(square, ["b"] * 9, repeat=1, exact=True)
    with pytest.raises(ValueError, match="unknown"):
        comparison.compare(square, ["missing"], repeat=1)


def test_unreachable_goal_does_not_create_partial_result():
    graph = graph_from_data({
        "schema_version": 1, "frame": "world", "depot": "a",
        "nodes": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 1, "y": 0}],
        "edges": [],
    })
    with pytest.raises(ValueError, match="unreachable"):
        comparison.compare(graph, ["b"], repeat=1, exact=True)


def test_clock_statistics_include_entire_plan(square):
    ticks = iter(itertools.count(100, 7))
    result = comparison.compare(square, ["b"], repeat=2, clock_ns=lambda: next(ticks))
    assert all(row["planning_time_ns_samples"] == [7, 7] for row in result["strategies"])
    assert all(row["planning_time_ns_median"] == 7 for row in result["strategies"])


def test_clock_reversal_rejected(square):
    ticks = iter([10, 9])
    with pytest.raises(ValueError, match="clock"):
        comparison.compare(square, ["b"], repeat=1, clock_ns=lambda: next(ticks))


def test_json_cli_and_error_without_partial_stdout(square, monkeypatch, capsys):
    monkeypatch.setattr(comparison, "_checked_graph", lambda _: (square, "testhash"))
    assert comparison.main(["--pickups", "b", "d", "c", "--repeat", "1", "--exact", "--json"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["graph_sha256"] == "testhash"
    assert report["cases"][0]["reference"]["distance_m"] == 8
    assert comparison.main(["--pickups", "missing", "--json"]) == 1
    captured = capsys.readouterr()
    assert captured.out == "" and "unknown" in captured.err


def test_exact_reference_rejects_string_even_if_valid_node_id(square):
    with pytest.raises(ValueError, match="list or tuple"):
        comparison.exact_reference(square, "b")
