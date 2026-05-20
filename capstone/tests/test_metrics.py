"""Regression tests for capstone.control.metrics.

Locks in the hover-quality numbers measured against the existing flight logs.
If the bridge log schema changes, these tests will surface it.
"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

from capstone.common.logging import load
from capstone.control.metrics import compute_metrics, find_hover_window


REPO = Path(__file__).resolve().parents[2]   # Swarm_Drones/ (capstone/tests/<file>.py is 2 levels deep)
LOGS = REPO / "logs" / "flight_logs"
GOOD_LOG = LOGS / "flight_20260504_105431.jsonl"
BAD_LOG = LOGS / "flight_20260502_204830.jsonl"


@pytest.fixture
def good_log():
    if not GOOD_LOG.exists():
        pytest.skip(f"missing {GOOD_LOG}")
    return load(GOOD_LOG)


@pytest.fixture
def bad_log():
    if not BAD_LOG.exists():
        pytest.skip(f"missing {BAD_LOG}")
    return load(BAD_LOG)


def test_good_log_calm_metrics_match_known_quality(good_log):
    """Lock in the metric values for the historical good_log calm hover.

    Gates were re-baselined 2026-05-05 from a 3-run calm reference batch
    (CALM1/CALM2/CALM3). good_log is an *older* calm hover that drifts
    slightly more on the north axis (5.5 cm vs ~3.8 cm on the new runs),
    so it just barely fails the new 5 cm pos_rms_north_m gate while
    passing everything else.
    """
    m = compute_metrics(good_log, "calm")
    assert m.window_t1 - m.window_t0 > 5.0
    assert 2.0 < m.alt_mean_m < 4.0
    assert m.alt_std_m < 0.12          # passes new gate
    assert m.pos_rms_east_m < 0.05      # passes new gate
    assert m.roll_rms_rad < 0.003       # passes new gate
    assert m.pitch_rms_rad < 0.003      # passes new gate
    assert m.gyro_rms < 0.008           # passes new gate
    assert m.crashed == 0
    # Just-barely-fails north gate (0.055 vs 0.05 gate) — older hover with
    # slightly more drift than the 3-run calm reference.
    assert m.pos_rms_north_m > 0.05
    assert m.pos_rms_north_m < 0.06
    assert not m.passed
    assert any("pos_rms_north_m" in f for f in m.failures), m.failures


def test_bad_log_fails_calm_gate(bad_log):
    m = compute_metrics(bad_log, "calm")
    assert not m.passed
    # Pre-tuning hover from 2026-05-02. Failures here are large -- alt_std,
    # position drift, tilt, gyro_rms all exceed calm bounds. The new window
    # finder surfaces a much longer hover window than the old |vz|<0.3 filter
    # gave us, but the metrics computed over it still fail every gate.
    assert m.failures, "expected explicit gate failures"
    assert any("pos_max" in f or "alt_std" in f or "gyro_rms" in f or "roll_max" in f
               for f in m.failures), m.failures


def test_window_finder_brackets_hover_at_target_altitude():
    """Window opens 2 s after drone reaches target alt (settling trim) and
    closes when it last sits within 50 cm of target.

    Synthetic flight: takeoff at t=0-2 s, arrived-at-hover at t=2 s, then
    8 seconds of hover with a brief disturbance excursion at t=6 s, then
    descends at t=10.
    """
    states: list[dict] = []
    states.append({"t": 0.0, "pos_ned": [0, 0,  0.00], "vel_ned": [0, 0, 0]})
    states.append({"t": 1.0, "pos_ned": [0, 0, -1.00], "vel_ned": [0, 0, -0.5]})
    # Hover at -2 m NED (= 2 m altitude) from t=2 onward.
    for i in range(2, 11):
        alt_ned = -2.0
        if i == 6:                     # disturbance push at t=6
            alt_ned = -1.85
        states.append({"t": float(i), "pos_ned": [0, 0, alt_ned],
                       "vel_ned": [0, 0, 0]})
    states.append({"t": 11.0, "pos_ned": [0, 0, -1.00], "vel_ned": [0, 0, 0.5]})
    states.append({"t": 12.0, "pos_ned": [0, 0,  0.00], "vel_ned": [0, 0, 0.1]})

    t0, t1, target = find_hover_window(states)
    # Drone arrives at hover at t=2; SETTLING_TRIM_S=0 so the window opens
    # immediately at arrival and runs for WINDOW_DURATION_S (10 s) OR until
    # the drone descends below AIRBORNE_THRESHOLD_M (0.3 m), whichever is
    # earlier. This synthetic flight descends past 0.3 m at t=12 (alt=0
    # there), well before the 10 s cap (which would close at t=12 anyway).
    # Disturbance excursion at t=6 is inside the window.
    assert t0 == pytest.approx(2.0)
    assert t1 == pytest.approx(12.0)
    assert t0 <= 6.0 <= t1


def test_window_finder_includes_disturbance_excursions():
    """Window should NOT cut out samples where the drone is being pushed --
    the disturbance response is exactly what we want to grade."""
    states: list[dict] = []
    states.append({"t": 0.0, "pos_ned": [0, 0,  0.00], "vel_ned": [0, 0, 0]})
    states.append({"t": 1.0, "pos_ned": [0, 0, -1.00], "vel_ned": [0, 0, -0.5]})
    for i in range(2, 11):
        alt_ned = -2.0
        if i == 7:                     # strong updraft push at t=7 (post-trim)
            alt_ned = -2.40
        states.append({"t": float(i), "pos_ned": [0, 0, alt_ned],
                       "vel_ned": [0, 0, -1.0 if i == 7 else 0.0]})
    states.append({"t": 11.0, "pos_ned": [0, 0,  0.00], "vel_ned": [0, 0, 0.5]})
    t0, t1, target = find_hover_window(states)
    # t=7 is post-settling-trim and pre-descent. vz=-1.0 (would have failed
    # the old |vz|<0.3 filter). MUST be inside the graded window.
    assert t0 <= 7.0 <= t1


def test_window_finder_returns_none_if_never_airborne():
    states = [
        {"t": float(i), "pos_ned": [0, 0, 0], "vel_ned": [0, 0, 0]}
        for i in range(5)
    ]
    t0, t1, target = find_hover_window(states)
    assert t0 is None and t1 is None
    assert target is None


def test_crash_detected_on_nan():
    from capstone.common.logging import FlightLog
    log = FlightLog(path=Path("synthetic"))
    log.states = [
        {"t": 0.0,  "pos_ned": [0, 0,  0.0], "vel_ned": [0, 0, 0],
         "gyro_frd": [0, 0, 0], "rpy": [0, 0, 0], "home_locked": True},
        {"t": 1.0,  "pos_ned": [0, 0, math.nan], "vel_ned": [0, 0, 0],
         "gyro_frd": [0, 0, 0], "rpy": [0, 0, 0], "home_locked": True},
    ]
    m = compute_metrics(log, "calm")
    assert m.crashed == 1
    assert "NaN" in (m.crash_reason or "")
