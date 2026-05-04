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


REPO = Path(__file__).resolve().parents[3]   # capstone/ at Swarm_Drones/capstone/, repo root is 3 up
LOGS = REPO / "flight_logs"
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


def test_good_log_calm_metrics_capture_known_north_drift(good_log):
    """Today's calm hover holds altitude/jitter tight but drifts ~8 cm north.

    Every profile uses the calm gates now, and the per-axis 5 cm position
    bound flags this as a FAIL -- which is correct: the drone really did
    drift 8 cm from home. The other calm gates (altitude std, gyro RMS,
    jitter) all still pass and are checked here to lock in the underlying
    flight quality.
    """
    m = compute_metrics(good_log, "calm")
    assert m.window_t1 - m.window_t0 > 5.0
    assert 2.0 < m.alt_mean_m < 4.0
    assert m.alt_std_m < 0.10
    assert m.xy_std_m < 0.05      # jitter stays tight
    assert m.gyro_rms < 0.005
    assert m.crashed == 0
    # Today's calm hover drifts ~7 cm RMS north (steady drift toward 0.09 m
    # max). RMS is the gated metric -- it captures the typical drift, not
    # just the worst sample. The result is a calm FAIL, which is correct.
    assert not m.passed
    assert any("pos_rms_north_m" in f for f in m.failures), m.failures
    assert m.pos_rms_north_m > 0.05


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
    # Drone arrives at hover at t=2; with SETTLING_TRIM_S=2.0 the graded
    # window opens at t=4 and closes at the last sample within 50 cm of
    # target (last airborne hover sample, t=10). The disturbance excursion
    # at t=6 falls inside [4, 10].
    assert t0 == pytest.approx(4.0)
    assert t1 == pytest.approx(10.0)
    assert 4.0 <= 6.0 <= t1


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
