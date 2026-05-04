"""YAML mission spec -> validated dataclass + MAVLink mission items.

A mission YAML looks like:

    name: square_20m
    description: 20 m square at 3 m altitude
    takeoff:
      altitude_m: 3.0

    # Default settings applied to each waypoint (override per-waypoint).
    defaults:
      altitude_m: 3.0
      hold_s: 2.0
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
    hold_s: float = 0.0
    accept_radius_m: float = 1.0


@dataclass
class Waypoint:
    """A single waypoint. Specifies position EITHER as (lat, lon) OR as
    (north_m, east_m) offsets from home, but not both.
    """
    altitude_m: float
    hold_s: float = 0.0
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
    out.setdefault("hold_s", defaults.hold_s)
    out.setdefault("accept_radius_m", defaults.accept_radius_m)
    if out["altitude_m"] is None:
        raise ValueError("waypoint missing altitude_m and no defaults.altitude_m set")
    return out


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
    """Return a sequence of mission items: [TAKEOFF, ...waypoints, RTL/LAND]."""
    items: list[dict] = []

    # 0: takeoff (lat/lon ignored by ArduCopter on takeoff item).
    items.append(_item(
        seq=0,
        command=MAV_CMD_NAV_TAKEOFF,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        x=home.lat, y=home.lon, z=mission.takeoff.altitude_m,
    ))

    # 1..N: waypoints.
    for i, wp in enumerate(mission.waypoints, start=1):
        lat, lon = wp.resolve(home)
        items.append(_item(
            seq=i,
            command=MAV_CMD_NAV_WAYPOINT,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            p1=wp.hold_s,
            p2=wp.accept_radius_m,
            x=lat, y=lon, z=wp.altitude_m,
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
