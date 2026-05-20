"""Smoke tests for capstone.common.logview pretty-printer."""
from __future__ import annotations

from pathlib import Path

import pytest

from capstone.common.logging import load
from capstone.common.logview import (
    render,
    render_calibration,
    render_disturbance,
    render_header,
    render_hover_window,
    render_snapshots,
    render_timeline,
)


REPO = Path(__file__).resolve().parents[2]   # Swarm_Drones/ (capstone/tests/<file>.py is 2 levels deep)
GOOD_LOG = REPO / "logs" / "flight_logs" / "flight_20260504_105431.jsonl"
WIND_LOG = REPO / "logs" / "flight_logs" / "flight_20260504_123507_WIND5.jsonl"


@pytest.fixture
def good_log():
    if not GOOD_LOG.exists():
        pytest.skip(f"missing {GOOD_LOG}")
    return load(GOOD_LOG)


@pytest.fixture
def wind_log():
    if not WIND_LOG.exists():
        pytest.skip(f"missing {WIND_LOG}")
    return load(WIND_LOG)


def test_header_includes_filename(good_log):
    out = render_header(good_log)
    assert good_log.path.name in out
    assert "duration" in out
    assert "SITL packets" in out


def test_calibration_renders(good_log):
    out = render_calibration(good_log)
    assert "mass" in out
    assert "K_thrust" in out
    assert "K_torque" in out


def test_disturbance_section_empty_when_absent(good_log):
    """Pre-disturbance log has no capstone_disturbance_active event."""
    out = render_disturbance(good_log)
    assert out == ""


def test_disturbance_section_present_when_active(wind_log):
    out = render_disturbance(wind_log)
    assert "DISTURBANCE" in out
    assert "wind5" in out


def test_timeline_lists_events(good_log):
    out = render_timeline(good_log)
    assert "TIMELINE" in out
    assert "bridge_setup_started" in out
    assert "motor_model_calibrated" in out


def test_hover_window_renders_metrics(good_log):
    out = render_hover_window(good_log)
    assert "HOVER WINDOW" in out
    assert "altitude" in out
    assert "gyro RMS" in out


def test_snapshots_respect_interval(good_log):
    out = render_snapshots(good_log, interval_s=10.0)
    rows = [r for r in out.splitlines() if r.strip().startswith(tuple("0123456789"))]
    assert len(rows) > 1


def test_full_render_runs(good_log):
    """Renders and includes a VERDICT (PASS or FAIL is fine -- the calm log
    currently fails the per-axis 5 cm position gate)."""
    out = render(good_log, interval_s=5.0, gate_profile="calm")
    assert "VERDICT" in out
    assert ("PASS" in out) or ("FAIL" in out)
    # ASCII-only on Windows console.
    assert "+/-" in out