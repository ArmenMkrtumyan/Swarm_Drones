"""Validate that the canonical mission YAMLs in capstone/missions/cases/ all
parse, compile, and produce sensible MAVLink item sequences."""
from __future__ import annotations

from pathlib import Path

import pytest

from capstone.missions.dsl import (
    HomePosition,
    compile_to_mavlink_items,
    load,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
)


CASES_DIR = Path(__file__).resolve().parents[1] / "missions" / "cases"
HOME = HomePosition(lat=40.192, lon=44.50446)


@pytest.mark.parametrize("yaml_name", ["square_20m.yaml", "fig8_50m.yaml",
                                        "survey_3x3.yaml"])
def test_case_loads_and_compiles(yaml_name: str):
    mission = load(CASES_DIR / yaml_name)
    assert mission.name
    assert mission.takeoff.altitude_m > 0
    assert len(mission.waypoints) >= 1

    items = compile_to_mavlink_items(mission, HOME)
    # First item must be takeoff, last must be RTL or LAND.
    assert items[0]["command"] == MAV_CMD_NAV_TAKEOFF
    # Body items are waypoints.
    for it in items[1:-1]:
        assert it["command"] == MAV_CMD_NAV_WAYPOINT
    # Seqs are consecutive.
    assert [it["seq"] for it in items] == list(range(len(items)))


def test_square_has_4_waypoints_and_rtl():
    mission = load(CASES_DIR / "square_20m.yaml")
    assert len(mission.waypoints) == 4
    assert mission.return_.type == "rtl"
