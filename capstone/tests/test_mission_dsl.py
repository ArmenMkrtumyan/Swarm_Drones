"""Unit tests for capstone.missions.dsl -- pure parsing and compilation,
no MAVLink stack required."""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from capstone.missions.dsl import (
    HomePosition,
    Mission,
    Waypoint,
    compile_to_mavlink_items,
    load,
    parse,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
)


HOME = HomePosition(lat=40.192, lon=44.50446)


def _square_yaml() -> dict:
    return {
        "name": "square_20m",
        "description": "20 m square at 3 m altitude",
        "takeoff": {"altitude_m": 3.0},
        "defaults": {"altitude_m": 3.0, "hold_s": 2.0, "accept_radius_m": 1.0},
        "waypoints": [
            {"north_m": 20, "east_m":  0},
            {"north_m": 20, "east_m": 20},
            {"north_m":  0, "east_m": 20},
            {"north_m":  0, "east_m":  0},
        ],
        "return": {"type": "rtl"},
    }


def test_parse_minimal_mission():
    mission = parse(_square_yaml())
    assert mission.name == "square_20m"
    assert mission.takeoff.altitude_m == 3.0
    assert len(mission.waypoints) == 4
    assert mission.waypoints[0].north_m == 20
    assert mission.waypoints[0].altitude_m == 3.0   # from defaults
    assert mission.waypoints[0].hold_s == 2.0
    assert mission.return_.type == "rtl"


def test_load_from_file(tmp_path: Path):
    p = tmp_path / "m.yaml"
    p.write_text(yaml.safe_dump(_square_yaml()))
    mission = load(p)
    assert mission.name == "square_20m"


def test_waypoint_must_specify_position_exactly_once():
    """(lat,lon) XOR (north_m,east_m) -- both or neither is an error."""
    with pytest.raises(ValueError, match="exactly one"):
        Waypoint(altitude_m=3.0)
    with pytest.raises(ValueError, match="exactly one"):
        Waypoint(altitude_m=3.0, lat=1.0, lon=2.0, north_m=10, east_m=10)
    # Either alone is fine.
    Waypoint(altitude_m=3.0, lat=1.0, lon=2.0)
    Waypoint(altitude_m=3.0, north_m=10, east_m=10)


def test_offset_resolves_close_to_home():
    """A 0,0 offset resolves exactly at home; small offsets give the right
    cardinal direction."""
    wp = Waypoint(altitude_m=3.0, north_m=0, east_m=0)
    assert wp.resolve(HOME) == pytest.approx((HOME.lat, HOME.lon), abs=1e-9)

    # 100 m north should bump lat by ~100/111111 = 0.0009 deg
    wp_n = Waypoint(altitude_m=3.0, north_m=100, east_m=0)
    lat_n, lon_n = wp_n.resolve(HOME)
    assert lat_n - HOME.lat == pytest.approx(100 / 111_111, abs=1e-6)
    assert lon_n == pytest.approx(HOME.lon, abs=1e-9)

    # 100 m east at 40 N -> lon shift by 100 / (111111 * cos(40))
    import math
    expected_dlon = 100 / (111_111 * math.cos(math.radians(HOME.lat)))
    wp_e = Waypoint(altitude_m=3.0, north_m=0, east_m=100)
    lat_e, lon_e = wp_e.resolve(HOME)
    assert lat_e == pytest.approx(HOME.lat, abs=1e-9)
    assert lon_e - HOME.lon == pytest.approx(expected_dlon, abs=1e-6)


def test_compile_produces_takeoff_waypoints_rtl():
    mission = parse(_square_yaml())
    items = compile_to_mavlink_items(mission, HOME)
    # 1 takeoff + 4 waypoints + 1 RTL = 6 items
    assert len(items) == 6
    assert items[0]["command"] == MAV_CMD_NAV_TAKEOFF
    assert items[0]["z"] == 3.0
    for it in items[1:5]:
        assert it["command"] == MAV_CMD_NAV_WAYPOINT
        assert it["frame"] == MAV_FRAME_GLOBAL_RELATIVE_ALT
        assert it["param2"] == 1.0  # accept_radius_m
        assert it["param1"] == 2.0  # hold_s
    assert items[-1]["command"] == MAV_CMD_NAV_RETURN_TO_LAUNCH


def test_compile_with_land_endpoint():
    spec = _square_yaml()
    spec["return"] = {"type": "land"}
    items = compile_to_mavlink_items(parse(spec), HOME)
    assert items[-1]["command"] == MAV_CMD_NAV_LAND
    assert items[-1]["z"] == 0.0


def test_compile_seq_numbers_are_consecutive():
    mission = parse(_square_yaml())
    items = compile_to_mavlink_items(mission, HOME)
    seqs = [it["seq"] for it in items]
    assert seqs == list(range(len(items)))
    # Only the first item has current=1.
    assert items[0]["current"] == 1
    assert all(it["current"] == 0 for it in items[1:])


def test_invalid_return_type_rejected():
    spec = _square_yaml()
    spec["return"] = {"type": "teleport"}
    with pytest.raises(ValueError, match="return.type"):
        parse(spec)


def test_missing_required_keys_rejected():
    with pytest.raises(ValueError, match="missing 'name'"):
        parse({"takeoff": {"altitude_m": 3}, "waypoints": [{"north_m": 0, "east_m": 0}]})
    with pytest.raises(ValueError, match="missing 'takeoff'"):
        parse({"name": "x", "waypoints": [{"north_m": 0, "east_m": 0}]})
    with pytest.raises(ValueError, match="at least one waypoint"):
        parse({"name": "x", "takeoff": {"altitude_m": 3}, "waypoints": []})


def test_per_waypoint_overrides_defaults():
    spec = _square_yaml()
    spec["waypoints"][2] = {"north_m": 0, "east_m": 20,
                            "altitude_m": 5.0, "hold_s": 10.0,
                            "accept_radius_m": 0.5}
    mission = parse(spec)
    items = compile_to_mavlink_items(mission, HOME)
    third_wp = items[3]   # 0=takeoff, 1=wp0, 2=wp1, 3=wp2
    assert third_wp["z"] == 5.0
    assert third_wp["param1"] == 10.0
    assert third_wp["param2"] == 0.5
