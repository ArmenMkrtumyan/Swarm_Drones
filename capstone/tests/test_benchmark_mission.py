"""Smoke tests for capstone.missions.benchmark_mission.

Uses the canonical PID baseline log in mission_logs/baseline_pid/. That
batch directory holds only good runs; the 2 aborted runs from earlier
attempts were deleted. Future RL eval batches go in their own subdir
(e.g. mission_logs/rl_eval_<date>/) so logs from different controllers
never get mixed in the same benchmark.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from capstone.missions.benchmark_mission import (
    aggregate_per_wp,
    analyze_run,
    collect_logs_for_mission,
    latlon_to_ne,
    main,
    percentile,
    perp_dist_to_line,
)
from capstone.missions.dsl import HomePosition, load as load_mission


REPO = Path(__file__).resolve().parents[3]
SQUARE_YAML = REPO / "Swarm_Drones" / "capstone" / "missions" / "cases" / "square_20m.yaml"
MISSION_LOGS = REPO / "mission_logs" / "baseline_pid"
GOOD_LOG = MISSION_LOGS / "mission_20260504_223554_square_20m.jsonl"
HOME = HomePosition(lat=40.192, lon=44.50446)


@pytest.fixture
def mission():
    if not SQUARE_YAML.exists():
        pytest.skip(f"missing {SQUARE_YAML}")
    return load_mission(SQUARE_YAML)


@pytest.fixture
def mission_logs():
    paths = collect_logs_for_mission(MISSION_LOGS, "square_20m")
    if not paths:
        pytest.skip(f"no square_20m logs in {MISSION_LOGS}")
    return paths


@pytest.fixture
def good_log():
    if not GOOD_LOG.exists():
        pytest.skip(f"missing known-good log {GOOD_LOG}")
    return GOOD_LOG


# -----------------------------------------------------------------------------
# Geometry helpers
# -----------------------------------------------------------------------------
def test_latlon_to_ne_round_trip():
    """Round-tripping through latlon_to_ne must agree with the DSL's compile."""
    n, e = latlon_to_ne(HOME.lat, HOME.lon, HOME.lat, HOME.lon)
    assert abs(n) < 1e-6 and abs(e) < 1e-6
    # 20 m north of home -> small lat delta back to ~20.
    dlat = 20.0 / 111_111.0
    n, e = latlon_to_ne(HOME.lat, HOME.lon, HOME.lat + dlat, HOME.lon)
    assert abs(n - 20.0) < 0.01
    assert abs(e) < 0.01


def test_perp_dist_to_line_axis_aligned():
    # Line along east (a=(0,0), b=(0,20)). Point at (1.5, 10) -> 1.5 m perp.
    d = perp_dist_to_line(1.5, 10, 0, 0, 0, 20)
    assert abs(d - 1.5) < 1e-9


def test_perp_dist_to_line_zero_length():
    # Degenerate line -> falls back to point-to-point distance.
    d = perp_dist_to_line(3, 4, 0, 0, 0, 0)
    assert abs(d - 5.0) < 1e-9


def test_percentile_basic():
    xs = list(range(101))   # 0..100
    assert percentile(xs, 0) == 0
    assert percentile(xs, 50) == 50
    assert percentile(xs, 95) == 95
    assert percentile(xs, 100) == 100
    assert percentile([], 50) is None


# -----------------------------------------------------------------------------
# analyze_run on the real square_20m logs
# -----------------------------------------------------------------------------
def test_analyze_run_does_not_raise(mission, mission_logs):
    """Every log -- even an aborted one -- analyzes without raising."""
    for path in mission_logs:
        run = analyze_run(path, mission, HOME)
        assert run.n_waypoints == 4
        assert len(run.waypoint_metrics) == 4
        assert 0.0 <= run.completion <= 1.0


def test_good_run_full_completion(mission, good_log):
    """The 223554 log is the known-good run (5/5 mission items reached + RTL)."""
    run = analyze_run(good_log, mission, HOME)
    assert run.completion == 1.0
    assert all(wp.reached for wp in run.waypoint_metrics)


def test_good_run_overshoot_2m_per_corner(mission, good_log):
    """Manual analysis of the good log: every corner overshot by 1.7-2.7 m
    after passing within 1-4 cm. The analyzer must reproduce both numbers."""
    run = analyze_run(good_log, mission, HOME)
    for wp in run.waypoint_metrics:
        assert wp.closest_approach_m is not None
        assert wp.overshoot_m is not None
        # Closest approach was always cm-level (drone passed right through
        # the WP at speed before overshooting).
        assert wp.closest_approach_m < 0.10, (
            f"WP{wp.seq}: closest {wp.closest_approach_m:.3f} m unexpectedly far")
        # Overshoot was 1.7-2.7 m on every corner (manual analysis).
        assert 1.5 < wp.overshoot_m < 3.0, (
            f"WP{wp.seq}: overshoot {wp.overshoot_m:.3f} m outside expected range")


def test_good_run_settle_lag_exceeds_hold(mission, good_log):
    """Manual analysis of the good log: 22-30 s settle lag at every WP
    because the drone re-entered the accept radius repeatedly after each
    overshoot. Has to exceed whatever hold_s the YAML currently specifies."""
    run = analyze_run(good_log, mission, HOME)
    hold_s = mission.waypoints[0].hold_s
    for wp in run.waypoint_metrics:
        assert wp.settle_lag_s is not None
        assert wp.settle_lag_s > hold_s, (
            f"WP{wp.seq}: settle_lag {wp.settle_lag_s:.2f} s should exceed "
            f"hold_s {hold_s}")
        assert wp.settle_lag_s < 60.0, (
            f"WP{wp.seq}: settle_lag {wp.settle_lag_s:.2f} s implausibly large")


def test_aggregate_per_wp_shape(mission, mission_logs):
    runs = [analyze_run(p, mission, HOME) for p in mission_logs]
    rows = aggregate_per_wp(runs, 4)
    assert len(rows) == 4
    for i, r in enumerate(rows):
        assert r["wp_idx"] == i
        assert r["seq"] == i + 1
        assert r["n_runs"] == len(runs)
        assert 0 <= r["n_reached"] <= len(runs)
        for k in ("closest_approach_m", "overshoot_m", "settle_lag_s",
                  "xtrack_p95_m", "leg_duration_s"):
            assert f"{k}_mean" in r
            assert f"{k}_std" in r


# -----------------------------------------------------------------------------
# End-to-end main()
# -----------------------------------------------------------------------------
def test_main_writes_artifacts(mission, mission_logs, tmp_path):
    """main() with --logs (analyze-only) writes all the expected files."""
    rc = main([
        str(SQUARE_YAML),
        "--logs", str(MISSION_LOGS),
        "--out", str(tmp_path),
        "--no-plots",
    ])
    assert rc == 0
    assert (tmp_path / "per_run_per_wp.csv").exists()
    assert (tmp_path / "per_run_summary.csv").exists()
    assert (tmp_path / "per_wp_summary.csv").exists()
    assert (tmp_path / "mission_summary.json").exists()
    assert not (tmp_path / "summary.txt").exists(), \
        "summary.txt should NOT be generated -- redundant with the CSVs"

    with (tmp_path / "per_run_per_wp.csv").open() as f:
        rows = list(csv.DictReader(f))
    # n_runs * 4 WPs rows.
    assert len(rows) == 4 * len(mission_logs)

    summary_data = json.loads((tmp_path / "mission_summary.json").read_text())
    assert len(summary_data) == len(mission_logs)
    for run in summary_data:
        assert run["n_waypoints"] == 4
        assert 0.0 <= run["completion"] <= 1.0
    # At least one run must have full completion (the known-good log).
    assert any(run["completion"] == 1.0 for run in summary_data)


def test_main_errors_when_no_logs_match(tmp_path):
    """Asking for a mission name that has no logs returns exit code 2."""
    # Mission YAML is fine; --logs points at a tmp dir with no matching logs.
    rc = main([
        str(SQUARE_YAML),
        "--logs", str(tmp_path),
        "--out", str(tmp_path / "out"),
        "--no-plots",
    ])
    assert rc == 2
