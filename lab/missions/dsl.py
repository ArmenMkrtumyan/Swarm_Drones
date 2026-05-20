"""YAML mission spec -> validated dataclass + MAVLink mission items.

A mission YAML looks like:

    name: square_20m
    description: 20 m square at 3 m altitude
    takeoff:
      altitude_m: 3.0

    # Default settings applied to each waypoint (override per-waypoint).
    defaults:
      altitude_m: 3.0
      pre_yaw_stop_s: 2.0        # brake and stop at WP for this long before yaw
      post_yaw_settle_s: 2.0     # post-yaw stationary settle. 0 => fly through.
      accept_radius_m: 1.0

    waypoints:
      - {north_m:  20, east_m:   0}
      - {north_m:  20, east_m:  20}
      - {north_m:   0, east_m:  20}
      - {north_m:   0, east_m:   0}

    return:
      type: rtl     # 'rtl' or 'land'

Each waypoint can use **either** (north_m, east_m) offsets from home OR
(lat, lon) absolute coordinates. Offsets are converted to lat/lon at compile
time using a local-flat-earth approximation around `home_lat`.

Compile path:
    mission = load(path)                                    # validate + dataclass
    items = compile_to_mavlink_items(mission, home)         # list of MAV mission items
    # items can be uploaded via pymavlink mission protocol.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


# MAVLink command numbers (from common.xml). We avoid importing pymavlink here
# so this module is pure-Python and testable without a MAVLink stack.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_DELAY = 93
MAV_CMD_CONDITION_YAW = 115

MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

# Meters per degree at the equator. Latitude is constant; longitude scales
# with cos(lat). Good enough for missions of a few hundred metres.
_M_PER_DEG_LAT = 111_111.0


# -----------------------------------------------------------------------------
# Dataclasses
# -----------------------------------------------------------------------------
@dataclass
class HomePosition:
    lat: float
    lon: float


@dataclass
class TakeoffSpec:
    altitude_m: float


@dataclass
class WaypointDefaults:
    altitude_m: float | None = None
    pre_yaw_stop_s: float = 0.0
    post_yaw_settle_s: float = 0.0
    accept_radius_m: float = 1.0


@dataclass
class Waypoint:
    """A single waypoint. Specifies position EITHER as (lat, lon) OR as
    (north_m, east_m) offsets from home, but not both.

    Stop-and-yaw behavior is controlled by two fields:
      pre_yaw_stop_s  -- time to brake and hold at the WP BEFORE the yaw.
                         Implemented via NAV_WAYPOINT.p1 (loiter time).
                         Without this, the drone arrives at the WP with
                         residual transit velocity and CONDITION_YAW runs
                         while it's still drifting, producing an off-course
                         path on the next leg.
      post_yaw_settle_s -- additional stationary settle AFTER the yaw, via
                         NAV_DELAY. Lets pos_control damp residual
                         oscillation before the next leg begins.

    Compile behavior:
      post_yaw_settle_s == 0  => fly-through. Single NAV_WAYPOINT(p1=0).
                                 pre_yaw_stop_s is IGNORED in this mode
                                 (no stop, no yaw, no settle).
      post_yaw_settle_s  > 0  => stop-yaw-settle triple:
                                 NAV_WAYPOINT(p1=pre_yaw_stop_s)
                                 + CONDITION_YAW(toward next WP or home)
                                 + NAV_DELAY(post_yaw_settle_s)
    """
    altitude_m: float
    pre_yaw_stop_s: float = 0.0
    post_yaw_settle_s: float = 0.0
    accept_radius_m: float = 1.0
    lat: float | None = None
    lon: float | None = None
    north_m: float | None = None
    east_m: float | None = None

    def __post_init__(self) -> None:
        has_latlon = self.lat is not None and self.lon is not None
        has_offset = self.north_m is not None and self.east_m is not None
        if has_latlon == has_offset:
            raise ValueError(
                "waypoint must specify exactly one of (lat,lon) or (north_m,east_m); "
                f"got lat={self.lat} lon={self.lon} N={self.north_m} E={self.east_m}"
            )

    def resolve(self, home: HomePosition) -> tuple[float, float]:
        """Return absolute (lat, lon) for this waypoint."""
        if self.lat is not None and self.lon is not None:
            return self.lat, self.lon
        # Local-flat-earth around home_lat.
        cos_lat = math.cos(math.radians(home.lat))
        if abs(cos_lat) < 1e-9:
            raise ValueError("invalid home latitude: cos(lat) ~= 0")
        dlat = (self.north_m or 0.0) / _M_PER_DEG_LAT
        dlon = (self.east_m or 0.0) / (_M_PER_DEG_LAT * cos_lat)
        return home.lat + dlat, home.lon + dlon


@dataclass
class ReturnSpec:
    type: str = "rtl"  # 'rtl' or 'land'

    def __post_init__(self) -> None:
        if self.type not in ("rtl", "land"):
            raise ValueError(f"return.type must be 'rtl' or 'land', got {self.type!r}")


@dataclass
class Mission:
    name: str
    description: str
    takeoff: TakeoffSpec
    waypoints: list[Waypoint]
    return_: ReturnSpec  # `return` is a Python keyword
    defaults: WaypointDefaults = field(default_factory=WaypointDefaults)


# -----------------------------------------------------------------------------
# YAML loader
# -----------------------------------------------------------------------------
def _apply_defaults(wp_data: dict, defaults: WaypointDefaults) -> dict:
    """Fill in missing waypoint fields from the mission's defaults block."""
    out = dict(wp_data)
    out.setdefault("altitude_m", defaults.altitude_m)
    out.setdefault("pre_yaw_stop_s", defaults.pre_yaw_stop_s)
    out.setdefault("post_yaw_settle_s", defaults.post_yaw_settle_s)
    out.setdefault("accept_radius_m", defaults.accept_radius_m)
    if out["altitude_m"] is None:
        raise ValueError("waypoint missing altitude_m and no defaults.altitude_m set")
    return out


def _bearing_deg(curr_lat: float, curr_lon: float,
                 next_lat: float, next_lon: float,
                 ref_lat: float) -> float:
    """Compass bearing from (curr_lat,curr_lon) to (next_lat,next_lon) in
    degrees, North=0, East=90, in [0, 360). Flat-earth approximation around
    ref_lat — fine for the few-hundred-meter missions this DSL targets."""
    cos_lat = math.cos(math.radians(ref_lat))
    delta_n = (next_lat - curr_lat) * _M_PER_DEG_LAT
    delta_e = (next_lon - curr_lon) * _M_PER_DEG_LAT * cos_lat
    return math.degrees(math.atan2(delta_e, delta_n)) % 360.0


def parse(data: dict) -> Mission:
    """Validate a parsed YAML dict and return a Mission. Use load() for files."""
    if "name" not in data:
        raise ValueError("mission missing 'name'")
    if "takeoff" not in data:
        raise ValueError("mission missing 'takeoff' block")
    if "waypoints" not in data or not data["waypoints"]:
        raise ValueError("mission needs at least one waypoint")

    defaults = WaypointDefaults(**(data.get("defaults") or {}))
    takeoff = TakeoffSpec(**data["takeoff"])
    waypoints = [Waypoint(**_apply_defaults(wp, defaults))
                 for wp in data["waypoints"]]
    return_ = ReturnSpec(**(data.get("return") or {"type": "rtl"}))

    return Mission(
        name=data["name"],
        description=data.get("description", ""),
        takeoff=takeoff,
        waypoints=waypoints,
        return_=return_,
        defaults=defaults,
    )


def load(path: str | Path) -> Mission:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{p}: top-level YAML must be a mapping")
    return parse(data)


# -----------------------------------------------------------------------------
# MAVLink compilation
# -----------------------------------------------------------------------------
def _item(seq: int, command: int, frame: int,
          p1: float = 0.0, p2: float = 0.0, p3: float = 0.0, p4: float = 0.0,
          x: float = 0.0, y: float = 0.0, z: float = 0.0,
          autocontinue: int = 1) -> dict:
    """Construct a generic mission item dict matching MAVLink MISSION_ITEM_INT
    semantics. lat/lon (x, y) are stored in degrees here; the runner converts
    to int1e7 before sending."""
    return {
        "seq": seq,
        "command": command,
        "frame": frame,
        "param1": float(p1), "param2": float(p2),
        "param3": float(p3), "param4": float(p4),
        "x": float(x), "y": float(y), "z": float(z),
        "current": 1 if seq == 0 else 0,
        "autocontinue": autocontinue,
    }


def compile_to_mavlink_items(mission: Mission, home: HomePosition) -> list[dict]:
    """Return a sequence of mission items: [TAKEOFF, ...waypoints (with optional
    yaw+settle), RTL/LAND].

    For each waypoint with post_yaw_settle_s > 0, three items are emitted:
        NAV_WAYPOINT(p1=pre_yaw_stop_s)  — fly to WP, brake, hold for the
                                            pre-yaw stop duration. The hold
                                            forces ArduCopter to decelerate
                                            to zero before advancing.
        CONDITION_YAW(target=bearing)    — yaw in place toward next WP
                                            (or home for the last WP)
        NAV_DELAY(p1=post_yaw_settle_s)  — additional settle after yaw

    This decouples yaw from transit, so the drone arrives at a WP, stops,
    finishes its yaw while stationary, settles, and only then begins moving
    toward the next leg — instead of yawing and translating concurrently
    (which is what WP_YAW_BEHAVIOR=1 produces by default).

    Waypoints with post_yaw_settle_s == 0 emit only NAV_WAYPOINT(p1=0) and
    preserve fly-through semantics (useful for figure-8 / survey missions);
    pre_yaw_stop_s is ignored in that mode.
    """
    items: list[dict] = []

    # 0: takeoff (lat/lon ignored by ArduCopter on takeoff item).
    items.append(_item(
        seq=0,
        command=MAV_CMD_NAV_TAKEOFF,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        x=home.lat, y=home.lon, z=mission.takeoff.altitude_m,
    ))

    n_wp = len(mission.waypoints)
    for wp_idx, wp in enumerate(mission.waypoints):
        lat, lon = wp.resolve(home)

        # NAV_WAYPOINT.p1 = pre_yaw_stop_s only when stop-yaw-settle is active.
        # When post_yaw_settle_s == 0 (fly-through), p1 is 0 so the drone
        # passes through without stopping.
        nav_p1 = wp.pre_yaw_stop_s if wp.post_yaw_settle_s > 0.0 else 0.0
        items.append(_item(
            seq=len(items),
            command=MAV_CMD_NAV_WAYPOINT,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            p1=nav_p1,
            p2=wp.accept_radius_m,
            x=lat, y=lon, z=wp.altitude_m,
        ))

        if wp.post_yaw_settle_s <= 0.0:
            continue

        # Target heading: bearing to next WP, or home for the final WP.
        if wp_idx + 1 < n_wp:
            next_lat, next_lon = mission.waypoints[wp_idx + 1].resolve(home)
        else:
            next_lat, next_lon = home.lat, home.lon
        target_yaw_deg = _bearing_deg(lat, lon, next_lat, next_lon, home.lat)

        items.append(_item(
            seq=len(items),
            command=MAV_CMD_CONDITION_YAW,
            frame=MAV_FRAME_MISSION,
            p1=target_yaw_deg,
            p2=0.0,   # angular speed: 0 => autopilot default
            p3=0.0,   # direction: 0 => shortest path
            p4=0.0,   # absolute angle (not relative)
        ))
        items.append(_item(
            seq=len(items),
            command=MAV_CMD_NAV_DELAY,
            frame=MAV_FRAME_MISSION,
            p1=wp.post_yaw_settle_s,
            p2=-1.0, p3=-1.0, p4=-1.0,   # time-of-day fields unused
        ))

    # Final: RTL or LAND.
    if mission.return_.type == "rtl":
        items.append(_item(
            seq=len(items),
            command=MAV_CMD_NAV_RETURN_TO_LAUNCH,
            frame=MAV_FRAME_MISSION,
        ))
    else:  # 'land'
        items.append(_item(
            seq=len(items),
            command=MAV_CMD_NAV_LAND,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            x=home.lat, y=home.lon, z=0.0,
        ))

    return items
