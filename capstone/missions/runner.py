"""Run a YAML-defined mission against ArduCopter SITL via MAVLink.

Wraps the proven upload-and-run sequence from
`Swarm_Drones/my_drone_simulation/upload_and_run_mission.py` so each mission
case file can be flown end-to-end without editing global constants:

    python -m capstone.missions.runner capstone/missions/cases/square_20m.yaml

The MAVLink dance ArduCopter 4.8-dev requires (cannot arm in AUTO; takeoff in
GUIDED; jump mission past takeoff; switch to AUTO mid-flight) is reproduced
here. A timestamped JSONL log is written to `mission_logs/`.

Default home (40.192°N, 44.50446°E -- the AUA reference point used in
mission_creator.py) is configurable. Mission waypoints are interpreted as
offsets from this home, so the actual SIM_OPOS_LAT/LNG don't have to match
exactly -- the drone always RTLs to where it took off.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import time
from pathlib import Path

from capstone.missions.dsl import (
    HomePosition,
    Mission,
    compile_to_mavlink_items,
    load,
)


DEFAULT_HOME_LAT = 40.192
DEFAULT_HOME_LON = 44.50446
DEFAULT_MASTER = "udpin:localhost:14551"
DEFAULT_MONITOR_TIMEOUT_S = 300.0


# -----------------------------------------------------------------------------
# JSONL logging (mirrors the format from upload_and_run_mission.py)
# -----------------------------------------------------------------------------
class MissionLogger:
    def __init__(self, log_dir: Path, mission_name: str):
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"mission_{stamp}_{mission_name}.jsonl"
        self.fh = self.path.open("w", encoding="utf-8", buffering=1)
        self.t0 = time.time()
        self._write({"src": "script", "event": "script_started",
                     "mission": mission_name})
        print(f"Mission log: {self.path}")

    def _write(self, entry: dict) -> None:
        t = round(time.time() - self.t0, 4)
        rec = {"t": t, **entry}
        self.fh.write(json.dumps(rec, separators=(",", ":"), default=str))
        self.fh.write("\n")

    def event(self, event: str, **payload) -> None:
        self._write({"src": "script", "event": event, **payload})

    def mavlink(self, msg) -> None:
        if msg is None:
            return
        try:
            fields = msg.to_dict()
        except Exception:
            fields = {"raw": str(msg), "type": msg.get_type()}
        self._write({"src": "mavlink", **fields})

    def close(self) -> None:
        try:
            self._write({"src": "script", "event": "script_ended"})
            self.fh.close()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# MAVLink helpers (lifted from upload_and_run_mission.py and pruned)
# -----------------------------------------------------------------------------
def latlon_to_int(deg: float) -> int:
    return int(round(deg * 1e7))


def normalize_param_id(param_id):
    if isinstance(param_id, bytes):
        return param_id.decode("utf-8", errors="ignore").rstrip("\x00")
    if isinstance(param_id, str):
        return param_id.rstrip("\x00")
    return str(param_id).rstrip("\x00")


def request_streams(master, mavutil) -> None:
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
    except Exception as e:
        print(f"Stream request warning: {e!r}")


def drain_messages(master, logger: MissionLogger, duration: float = 3.0,
                   verbose: bool = True) -> None:
    end = time.time() + duration
    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue
        logger.mavlink(msg)
        if not verbose:
            continue
        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
        elif mtype == "MISSION_ITEM_REACHED":
            print(f"MISSION_ITEM_REACHED: seq={msg.seq}")
        elif mtype == "MISSION_CURRENT":
            print(f"MISSION_CURRENT: seq={msg.seq}")
        elif mtype == "GLOBAL_POSITION_INT":
            pass  # too chatty


def wait_command_ack(master, command, timeout: float = 5.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return msg
    return None


def wait_mission_ack(master, timeout: float = 10.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=0.5)
        if msg:
            return msg
    return None


def is_armed(master, mavutil) -> bool:
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
    if hb is None:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_armed(master, mavutil, timeout: float = 5.0) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        if is_armed(master, mavutil):
            return True
    return False


def set_mode(master, mavutil, mode_name: str) -> None:
    mode_map = master.mode_mapping()
    if mode_map is None or mode_name not in mode_map:
        raise RuntimeError(f"Mode {mode_name} not available")
    mode_id = mode_map[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"Requested mode: {mode_name}")
    time.sleep(1.5)


def arm(master, mavutil, force: bool = False) -> None:
    param2 = 21196 if force else 0
    print(f"Sending {'FORCE ' if force else ''}ARM command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, param2, 0, 0, 0, 0, 0,
    )
    ack = wait_command_ack(
        master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5.0)
    print(f"Arm ACK: {ack}")


def send_position_target(master, mavutil, x: float, y: float, z: float,
                          yaw: float = 0.0) -> None:
    type_mask = 0b0000111111111000
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0.0,
    )


def takeoff_guided_and_wait(master, mavutil, alt: float,
                              logger: MissionLogger,
                              climb_timeout: float = 45.0) -> bool:
    """Fire NAV_TAKEOFF and PASSIVELY wait for the drone to reach target alt.

    Critical: do NOT stream SET_POSITION_TARGET_LOCAL_NED during the climb.

    ArduCopter's GUIDED mode runs a dedicated takeoff submode (SubMode::TakeOff)
    after MAV_CMD_NAV_TAKEOFF. That submode owns the altitude target and ramps
    throttle past the 90% breakout that calls set_land_complete(false). If we
    send SET_POSITION_TARGET_LOCAL_NED while takeoff is still running,
    set_pos_NED_m calls pos_control_start() which unconditionally overwrites
    guided_mode = SubMode::Pos -- evicting takeoff before it finishes ramping.
    pos_control_run then sees ap.land_complete still true and routes through
    make_safe_ground_handling, clamping motors at MOT_SPIN_ARM (~PWM 1100).
    The drone never lifts. (See ArduCopter takeoff.cpp:18-48, mode_guided.cpp:367.)

    GUID_TIMEOUT only fires while GUIDED has no active controller target; during
    takeoff the takeoff submode IS the target, so no stream is needed. We let
    NAV_TAKEOFF run undisturbed, and after this function returns the caller
    immediately switches to AUTO -- no position-hold streaming required.
    """
    print(f"GUIDED takeoff to {alt:.1f}m (passive -- not streaming targets)...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, float(alt),
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=3.0)
    print(f"Takeoff ACK: {ack}")

    start = time.time()
    last_report = 0.0
    while time.time() - start < climb_timeout:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        logger.mavlink(msg)
        mtype = msg.get_type()
        now = time.time()
        if mtype == "GLOBAL_POSITION_INT":
            alt_now = msg.relative_alt / 1000.0
            vz = msg.vz / 100.0
            if alt_now >= 0.95 * alt:
                print(f"Reached {alt_now:.2f}m (target {alt}m). Climb done.")
                return True
            if now - last_report >= 1.0:
                print(f"[climb] t={now-start:5.1f}s rel_alt={alt_now:.2f}m vz={vz:+.2f}")
                last_report = now
        elif mtype == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
    print(f"Climb timeout after {climb_timeout:.0f}s.")
    return False


# EKF readiness flags (from EKF_STATUS_REPORT). The arm gate is POS_HORIZ_ABS.
_EKF_POS_HORIZ_ABS = 16
_EKF_GPS_GLITCHING = 32768


def _request_message_interval(master, mavutil, message_id: int, hz: float) -> None:
    interval_us = int(1_000_000 / hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0,
    )


def wait_for_ekf_ready(master, mavutil, logger: MissionLogger,
                        timeout: float = 30.0, poll_hz: float = 2.0) -> bool:
    """Poll EKF_STATUS_REPORT.flags until POS_HORIZ_ABS is set (and GPS not
    glitching). This is the actual gate ArduCopter checks before letting
    GUIDED arm with "Need Position Estimate". The earlier STATUSTEXT-scraping
    approach missed announcements that fired before we started listening.
    """
    _request_message_interval(master, mavutil, 193, poll_hz)  # EKF_STATUS_REPORT
    _request_message_interval(master, mavutil, 24, 1.0)        # GPS_RAW_INT

    print(f"Waiting up to {timeout:.0f}s for EKF3 POS_HORIZ_ABS...")
    end = time.time() + timeout
    last_flags = None
    while time.time() < end:
        msg = master.recv_match(
            type=["EKF_STATUS_REPORT", "GPS_RAW_INT", "STATUSTEXT"],
            blocking=True, timeout=0.5,
        )
        if msg is None:
            continue
        logger.mavlink(msg)
        if msg.get_type() == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
            continue
        if msg.get_type() != "EKF_STATUS_REPORT":
            continue
        flags = int(msg.flags)
        if flags != last_flags:
            print(f"  [ekf] flags=0x{flags:04x}")
            last_flags = flags
        if (flags & _EKF_POS_HORIZ_ABS) and not (flags & _EKF_GPS_GLITCHING):
            print("  [ekf] POS_HORIZ_ABS set -- ready to arm")
            return True
    print(f"  [ekf] timed out; last flags=0x{last_flags or 0:04x}")
    return False


# -----------------------------------------------------------------------------
# Mission upload (using the items dict from dsl.compile_to_mavlink_items)
# -----------------------------------------------------------------------------
def send_mission_clear_all(master, mavutil) -> None:
    try:
        master.mav.mission_clear_all_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_clear_all_send(
            master.target_system, master.target_component,
        )
    print("Sent MISSION_CLEAR_ALL")


def send_mission_count(master, mavutil, count: int) -> None:
    try:
        master.mav.mission_count_send(
            master.target_system, master.target_component, count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_count_send(
            master.target_system, master.target_component, count,
        )
    print(f"Sent MISSION_COUNT: {count}")


def send_mission_item_int(master, mavutil, item: dict) -> None:
    """Send one item. dsl emits lat/lon as float degrees; MAVLink wants int1e7."""
    x_int = (latlon_to_int(item["x"])
             if item["frame"] != mavutil.mavlink.MAV_FRAME_MISSION
             else int(item["x"]))
    y_int = (latlon_to_int(item["y"])
             if item["frame"] != mavutil.mavlink.MAV_FRAME_MISSION
             else int(item["y"]))
    args = (
        master.target_system, master.target_component,
        item["seq"], item["frame"], item["command"],
        item["current"], item["autocontinue"],
        item["param1"], item["param2"], item["param3"], item["param4"],
        x_int, y_int, item["z"],
    )
    try:
        master.mav.mission_item_int_send(*args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    except TypeError:
        master.mav.mission_item_int_send(*args)


def upload_mission(master, mavutil, items: list[dict]) -> None:
    send_mission_clear_all(master, mavutil)
    ack = wait_mission_ack(master, timeout=5.0)
    if ack is None or int(ack.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"MISSION_CLEAR_ALL failed: {ack}")

    send_mission_count(master, mavutil, len(items))

    sent = set()
    while True:
        msg = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True, timeout=10.0,
        )
        if msg is None:
            raise RuntimeError("Timed out waiting for mission protocol response")
        mtype = msg.get_type()
        if mtype in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            if seq < 0 or seq >= len(items):
                raise RuntimeError(f"Invalid requested seq {seq}")
            send_mission_item_int(master, mavutil, items[seq])
            sent.add(seq)
            print(f"Sent mission item {seq}")
        elif mtype == "MISSION_ACK":
            if len(sent) != len(items):
                raise RuntimeError(
                    f"Got MISSION_ACK before all items requested. sent={sorted(sent)}")
            if int(msg.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Mission upload failed: {msg}")
            print("Mission upload complete.")
            break


def set_current_mission_item(master, mavutil, seq: int = 0) -> None:
    print(f"Setting current mission item = {seq}")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0,
        float(seq), 0, 0, 0, 0, 0, 0,
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                            timeout=3.0)
    if ack is None:
        master.mav.mission_set_current_send(
            master.target_system, master.target_component, seq)
    time.sleep(0.5)


def start_mission(master, mavutil) -> None:
    print("Sending MAV_CMD_MISSION_START...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START, 0,
        0, 0, 0, 0, 0, 0, 0,
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_MISSION_START, timeout=5.0)
    print(f"Mission start ACK: {ack}")


# -----------------------------------------------------------------------------
# Driver
# -----------------------------------------------------------------------------
def run_mission(
    mission: Mission,
    home: HomePosition,
    *,
    master_url: str = DEFAULT_MASTER,
    log_dir: Path | None = None,
    monitor_timeout_s: float = DEFAULT_MONITOR_TIMEOUT_S,
) -> Path:
    """Run the mission end-to-end. Returns the path of the JSONL log written."""
    from pymavlink import mavutil  # imported lazily so unit tests don't need it

    # capstone/ lives at Swarm_Drones/capstone/, repo root (where mission_logs/
    # lives) is 3 levels up from capstone/missions/runner.py.
    log_dir = log_dir or Path(__file__).resolve().parents[3] / "mission_logs"
    logger = MissionLogger(log_dir, mission.name)

    try:
        items = compile_to_mavlink_items(mission, home)
        logger.event("compiled_mission",
                     n_items=len(items),
                     home_lat=home.lat, home_lon=home.lon,
                     takeoff_alt=mission.takeoff.altitude_m,
                     n_waypoints=len(mission.waypoints),
                     return_type=mission.return_.type)

        print(f"Connecting to SITL at {master_url}...")
        master = mavutil.mavlink_connection(master_url)
        master.wait_heartbeat()
        print(f"Heartbeat from system={master.target_system} "
              f"component={master.target_component}")

        request_streams(master, mavutil)
        drain_messages(master, logger, duration=5.0)

        print(f"Mission to upload ({len(items)} items):")
        for it in items:
            print(f"  seq={it['seq']:2d} cmd={it['command']:3d} "
                  f"frame={it['frame']} z={it['z']:.2f} "
                  f"x={it['x']:.6f} y={it['y']:.6f}")

        upload_mission(master, mavutil, items)
        drain_messages(master, logger, duration=3.0)

        # GUIDED -> wait for EKF -> arm -> takeoff (passive) -> AUTO mid-flight.
        # AUTO can't arm in 4.8-dev ("Auto mode not armable"); takeoff completes
        # under GUIDED's NAV_TAKEOFF submode, then we set mission current=1 and
        # switch to AUTO to fly the waypoints.
        set_mode(master, mavutil, "GUIDED")
        drain_messages(master, logger, duration=1.0)

        if not wait_for_ekf_ready(master, mavutil, logger, timeout=30.0):
            raise RuntimeError("EKF never reported POS_HORIZ_ABS -- check GPS / origin.")

        arm(master, mavutil, force=False)
        drain_messages(master, logger, duration=5.0)

        if not wait_armed(master, mavutil, timeout=5.0):
            raise RuntimeError("Vehicle did not arm. Check pre-arm checks / params.")

        if not takeoff_guided_and_wait(
                master, mavutil, alt=mission.takeoff.altitude_m, logger=logger,
                climb_timeout=25.0):
            raise RuntimeError("Takeoff did not reach target altitude.")

        # Skip past the takeoff item and switch to AUTO for waypoints + RTL.
        set_current_mission_item(master, mavutil, seq=1)
        drain_messages(master, logger, duration=1.0)

        set_mode(master, mavutil, "AUTO")
        drain_messages(master, logger, duration=2.0)

        start_mission(master, mavutil)
        logger.event("mission_started")
        drain_messages(master, logger, duration=monitor_timeout_s, verbose=True)

        logger.event("monitor_done")
        print("Mission monitoring window finished.")
        return logger.path
    finally:
        logger.close()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("mission", type=Path, help="Path to a mission YAML file")
    p.add_argument("--home-lat", type=float, default=DEFAULT_HOME_LAT,
                    help=f"reference home latitude (default {DEFAULT_HOME_LAT})")
    p.add_argument("--home-lon", type=float, default=DEFAULT_HOME_LON,
                    help=f"reference home longitude (default {DEFAULT_HOME_LON})")
    p.add_argument("--master", default=DEFAULT_MASTER,
                    help=f"MAVLink connection string (default {DEFAULT_MASTER})")
    p.add_argument("--monitor", type=float, default=DEFAULT_MONITOR_TIMEOUT_S,
                    help="how long to watch after mission start (seconds)")
    args = p.parse_args(argv)

    mission = load(args.mission)
    home = HomePosition(lat=args.home_lat, lon=args.home_lon)

    print(f"Mission: {mission.name}")
    print(f"  takeoff alt: {mission.takeoff.altitude_m} m")
    print(f"  waypoints:   {len(mission.waypoints)}")
    print(f"  return:      {mission.return_.type}")
    print(f"  home:        ({home.lat}, {home.lon})")
    print()

    log_path = run_mission(
        mission, home,
        master_url=args.master,
        monitor_timeout_s=args.monitor,
    )
    print(f"\nDone. Log: {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
