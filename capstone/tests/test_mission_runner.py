"""Smoke tests for capstone.missions.runner -- exercises the parsing + compile
+ logging glue without actually opening a MAVLink connection."""
from __future__ import annotations

from pathlib import Path

import pytest

from capstone.missions.runner import MissionLogger, latlon_to_int


def test_latlon_to_int_round_trip():
    """MAVLink uses int1e7 for lat/lon. Conversion preserves precision."""
    assert latlon_to_int(40.192) == 401_920_000
    assert latlon_to_int(-44.50446) == -445_044_600
    # Round-trip back to float is within 1e-7 deg (~1 cm at the equator).
    for deg in (40.192, 44.50446, -33.0, 0.0):
        back = latlon_to_int(deg) / 1e7
        assert abs(back - deg) < 1e-7


def test_mission_logger_writes_jsonl(tmp_path: Path):
    log = MissionLogger(tmp_path, "smoke_test")
    log.event("hello", value=42)
    log.event("done")
    log.close()
    lines = [ln for ln in log.path.read_text().splitlines() if ln.strip()]
    # script_started + 2 events + script_ended = 4
    assert len(lines) == 4
    import json
    first = json.loads(lines[0])
    assert first["src"] == "script"
    assert first["event"] == "script_started"
    assert first["mission"] == "smoke_test"
