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
    MAV_CMD_CONDITION_YAW,
    MAV_CMD_NAV_DELAY,
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
        "defaults": {"altitude_m": 3.0, "post_yaw_settle_s": 2.0, "accept_radius_m": 1.0},
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
    assert mission.waypoints[0].post_yaw_settle_s == 2.0
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
    """With post_yaw_settle_s > 0 each WP compiles to 3 items
    (NAV_WAYPOINT + CONDITION_YAW + NAV_DELAY). 1 takeoff + 4*3 + 1 RTL = 14."""
    mission = parse(_square_yaml())
    items = compile_to_mavlink_items(mission, HOME)
    assert len(items) == 14
    assert items[0]["command"] == MAV_CMD_NAV_TAKEOFF
    assert items[0]["z"] == 3.0
    # Per-WP triple: NAV_WAYPOINT(p1=pre_yaw_stop_s, p2=accept_radius),
    # CONDITION_YAW, NAV_DELAY(p1=post_yaw_settle_s).
    for wp_idx in range(4):
        base = 1 + wp_idx * 3
        nav, yaw, delay = items[base], items[base + 1], items[base + 2]
        assert nav["command"] == MAV_CMD_NAV_WAYPOINT
        assert nav["frame"] == MAV_FRAME_GLOBAL_RELATIVE_ALT
        assert nav["param1"] == 0.0  # pre_yaw_stop_s default in _square_yaml
        assert nav["param2"] == 1.0  # accept_radius_m
        assert yaw["command"] == MAV_CMD_CONDITION_YAW
        assert 0.0 <= yaw["param1"] < 360.0  # absolute bearing degrees
        assert delay["command"] == MAV_CMD_NAV_DELAY
        assert delay["param1"] == 2.0  # post_yaw_settle_s
    assert items[-1]["command"] == MAV_CMD_NAV_RETURN_TO_LAUNCH


def test_pre_yaw_stop_s_flows_to_nav_waypoint_p1():
    """pre_yaw_stop_s should appear as NAV_WAYPOINT.param1 in the compiled
    output (and ONLY when post_yaw_settle_s > 0 — fly-through ignores it)."""
    spec = _square_yaml()
    spec["defaults"]["pre_yaw_stop_s"] = 1.5
    items = compile_to_mavlink_items(parse(spec), HOME)
    for wp_idx in range(4):
        nav = items[1 + wp_idx * 3]
        assert nav["command"] == MAV_CMD_NAV_WAYPOINT
        assert nav["param1"] == 1.5

    # Fly-through (post=0) ignores pre_yaw_stop_s.
    spec_ft = _square_yaml()
    spec_ft["defaults"]["pre_yaw_stop_s"] = 1.5
    spec_ft["defaults"]["post_yaw_settle_s"] = 0.0
    items_ft = compile_to_mavlink_items(parse(spec_ft), HOME)
    for wp_idx in range(4):
        nav = items_ft[1 + wp_idx]   # no triples in fly-through
        assert nav["command"] == MAV_CMD_NAV_WAYPOINT
        assert nav["param1"] == 0.0  # ignored


def test_compile_flythrough_skips_yaw_and_delay():
    """post_yaw_settle_s == 0 emits only NAV_WAYPOINT — no CONDITION_YAW or NAV_DELAY."""
    spec = _square_yaml()
    spec["defaults"]["post_yaw_settle_s"] = 0.0
    items = compile_to_mavlink_items(parse(spec), HOME)
    # 1 takeoff + 4 waypoints + 1 RTL = 6
    assert len(items) == 6
    assert items[0]["command"] == MAV_CMD_NAV_TAKEOFF
    for it in items[1:5]:
        assert it["command"] == MAV_CMD_NAV_WAYPOINT
        assert it["param1"] == 0.0
    assert items[-1]["command"] == MAV_CMD_NAV_RETURN_TO_LAUNCH
    assert not any(it["command"] == MAV_CMD_CONDITION_YAW for it in items)
    assert not any(it["command"] == MAV_CMD_NAV_DELAY for it in items)


def test_compile_last_waypoint_yaws_toward_home():
    """The last WP's CONDITION_YAW target should be the bearing to home
    (so the drone is pointed at the launchpad before RTL begins)."""
    mission = parse(_square_yaml())
    items = compile_to_mavlink_items(mission, HOME)
    # Last WP triple starts at index 1 + 3*3 = 10. NAV_WAYPOINT at 10,
    # CONDITION_YAW at 11, NAV_DELAY at 12.
    last_yaw = items[11]
    assert last_yaw["command"] == MAV_CMD_CONDITION_YAW
    # In the square_20m, WP3 is at offset (0, 0) = home itself — bearing is
    # undefined (atan2(0,0) = 0). Just check the field is a valid bearing.
    assert 0.0 <= last_yaw["param1"] < 360.0
    # And confirm with a non-degenerate case: shift WP3 a bit off-home so the
    # bearing has a meaningful target.
    spec = _square_yaml()
    spec["waypoints"][3] = {"north_m": 10, "east_m": 0}  # 10 m north of home
    items2 = compile_to_mavlink_items(parse(spec), HOME)
    last_yaw2 = items2[11]
    # WP3 is 10 m north of home; bearing from WP3 to home is due south = 180 deg.
    assert abs(last_yaw2["param1"] - 180.0) < 0.5, last_yaw2["param1"]


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
                            "altitude_m": 5.0, "post_yaw_settle_s": 10.0,
                            "accept_radius_m": 0.5}
    mission = parse(spec)
    items = compile_to_mavlink_items(mission, HOME)
    # Item layout (each WP = 3 items): takeoff + (wp0 nav,yaw,delay) +
    # (wp1 nav,yaw,delay) + (wp2 nav,yaw,delay) + ...
    # wp2's NAV_WAYPOINT lives at index 1 + 2*3 = 7; its NAV_DELAY at 9.
    third_wp_nav = items[7]
    third_wp_delay = items[9]
    assert third_wp_nav["command"] == MAV_CMD_NAV_WAYPOINT
    assert third_wp_nav["z"] == 5.0
    assert third_wp_nav["param2"] == 0.5      # accept_radius override
    assert third_wp_delay["command"] == MAV_CMD_NAV_DELAY
    assert third_wp_delay["param1"] == 10.0   # post_yaw_settle_s override
