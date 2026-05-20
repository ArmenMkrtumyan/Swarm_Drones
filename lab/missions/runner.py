"""Run a YAML-defined mission against ArduCopter SITL via MAVLink.

Wraps the proven MAVLink upload-and-run sequence so each mission case file
can be flown end-to-end without editing global constants:

    python -m lab.missions.runner lab/missions/cases/square_20m.yaml

The MAVLink dance ArduCopter 4.8-dev requires (cannot arm in AUTO; takeoff in
GUIDED; jump mission past takeoff; switch to AUTO mid-flight) is reproduced
here. A timestamped JSONL log is written to `logs/mission_logs/`.

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

import math

from lab.missions.dsl import (
    HomePosition,
    Mission,
    compile_to_mavlink_items,
    load,
)


_M_PER_DEG_LAT = 111_111.0


_WIND_LABEL_TO_DIR = {
    # Maps the suffix in `mission_worst_case_<suffix>` → (north_unit, east_unit)
    # under the bridge's NED world frame (X=N, Y=E).
    "px": (+1.0,  0.0),
    "nx": (-1.0,  0.0),
    "py":  (0.0, +1.0),
    "ny":  (0.0, -1.0),
}


def _extract_bridge_overlay(bridge_log: Path) -> dict:
    """Read a bridge flight_*.jsonl and pull out:
      - wind_label:  'px'/'nx'/'py'/'ny' or None
      - drop_t:      bridge sim_time when mass_drop_event fired (s) or None
      - drop_pos_ne: (north_m, east_m) of drone at drop time, or None
      - payload_kg:  the drop payload mass (kg) or None
    """
    wind_label: str | None = None
    drop_t: float | None = None
    payload_kg: float | None = None
    state_samples: list[tuple[float, float, float]] = []  # (t, n, e)

    with bridge_log.open("r", encoding="utf-8") as fh:
        for line in fh:
            try:
                d = json.loads(line)
            except Exception:
                continue
            ev = d.get("event")
            if wind_label is None and ev == "capstone_disturbance_active":
                profile = str(d.get("profile", ""))
                if len(profile) >= 2 and profile[-3] == "_" and profile[-2:] in _WIND_LABEL_TO_DIR:
                    wind_label = profile[-2:]
            if drop_t is None and ev == "mass_drop_event":
                drop_t = float(d.get("t", 0.0))
                payload_kg = float(d.get("payload_kg", 0.0))
            if d.get("src") == "isaac->sitl" and "pos_ned" in d:
                pn = d["pos_ned"]
                state_samples.append((float(d["t"]), float(pn[0]), float(pn[1])))

    drop_pos_ne = None
    if drop_t is not None and state_samples:
        closest = min(state_samples, key=lambda s: abs(s[0] - drop_t))
        drop_pos_ne = (closest[1], closest[2])

    return {
        "wind_label": wind_label,
        "drop_t": drop_t,
        "drop_pos_ne": drop_pos_ne,
        "payload_kg": payload_kg,
    }


def _find_matching_bridge_log(mission_log: Path,
                              window_s: float = 600.0) -> Path | None:
    """Heuristic: find the bridge flight_*.jsonl that pairs with this mission
    log. Searches the SAME directory for any `flight_*.jsonl` whose mtime is
    within ``window_s`` of the mission log's mtime, returns the closest. The
    user-convention is to move the bridge's log into the mission log dir
    alongside the mission JSONL after each run — when they do, pairing works.
    Returns None if no candidate is found."""
    try:
        mtime = mission_log.stat().st_mtime
    except OSError:
        return None
    best = None
    best_dt = window_s
    for cand in mission_log.parent.glob("flight_*.jsonl"):
        try:
            dt = abs(cand.stat().st_mtime - mtime)
        except OSError:
            continue
        if dt < best_dt:
            best_dt = dt
            best = cand
    return best


def plot_run_xy(log_path: Path, mission: Mission, home: HomePosition,
                out_dir: Path, *, bridge_log: Path | None = None) -> Path | None:
    """Generate a 2D XY trajectory PNG from a single mission JSONL log.

    Parses GLOBAL_POSITION_INT entries, converts to local NED relative to
    home, and plots the trajectory together with WP markers + the planned
    legs. Output: ``<out_dir>/<log_stem>.png``. Returns the PNG path on
    success, None if matplotlib is unavailable or the log has no positions.

    If `bridge_log` is provided (the matching `flight_*.jsonl`), the plot is
    annotated with the run's wind direction (corner arrow) and the
    mass-drop position (orange diamond). Bridge overlay is best-effort —
    missing log or missing events are silently skipped.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")  # headless
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[plot] matplotlib unavailable ({e!r}); skipping XY plot")
        return None

    cos_lat = math.cos(math.radians(home.lat))
    norths: list[float] = []
    easts: list[float] = []
    n_dropped = 0
    with log_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get("src") != "mavlink":
                continue
            if d.get("mavpackettype") != "GLOBAL_POSITION_INT":
                continue
            lat_i = int(d["lat"])
            lon_i = int(d["lon"])
            # ArduPilot occasionally emits GLOBAL_POSITION_INT with lat=0,
            # lon=0 — typically post-disarm or during EKF-init transients.
            # These decode to a point ~4500 km from home and blow up the
            # auto-axis of the XY plot. Drop them.
            if lat_i == 0 and lon_i == 0:
                n_dropped += 1
                continue
            lat = lat_i / 1e7
            lon = lon_i / 1e7
            n = (lat - home.lat) * _M_PER_DEG_LAT
            e = (lon - home.lon) * _M_PER_DEG_LAT * cos_lat
            # Defensive: drop any sample > 10 km from home — way beyond any
            # mission we'd run, almost certainly a glitch.
            if abs(n) > 10_000.0 or abs(e) > 10_000.0:
                n_dropped += 1
                continue
            norths.append(n)
            easts.append(e)
    if n_dropped:
        print(f"[plot] dropped {n_dropped} invalid GLOBAL_POSITION_INT samples "
              f"(lat=0/lon=0 or > 10 km from home)")

    if not norths:
        print(f"[plot] no GLOBAL_POSITION_INT entries in {log_path.name}; skipping")
        return None

    # Resolve planned WPs in (north, east).
    wp_n: list[float] = []
    wp_e: list[float] = []
    for wp in mission.waypoints:
        if wp.lat is not None and wp.lon is not None:
            n = (wp.lat - home.lat) * _M_PER_DEG_LAT
            e = (wp.lon - home.lon) * _M_PER_DEG_LAT * cos_lat
        else:
            n = float(wp.north_m or 0.0)
            e = float(wp.east_m or 0.0)
        wp_n.append(n)
        wp_e.append(e)

    out_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    # Plot east on x, north on y so up = north (standard map orientation).
    ax.plot(easts, norths, "-", color="#1f77b4", lw=1.4, label="trajectory")
    ax.plot([0] + wp_e, [0] + wp_n, "--", color="#888888", lw=0.8, alpha=0.7,
            label="planned path")
    ax.scatter(easts[0], norths[0], marker="o", color="green", s=70,
               zorder=5, label="start")
    ax.scatter(easts[-1], norths[-1], marker="s", color="red", s=70,
               zorder=5, label="end")
    ax.scatter([0], [0], marker="*", color="black", s=120, zorder=5,
               label="home")
    for i, (n, e) in enumerate(zip(wp_n, wp_e), start=1):
        ax.scatter(e, n, marker="x", color="black", s=80, zorder=4)
        ax.annotate(f"WP{i}", (e, n), xytext=(6, 6), textcoords="offset points",
                    fontsize=10)

    # Optional bridge-log overlay: wind direction arrow + drop point marker.
    title_suffix = ""
    if bridge_log is not None and bridge_log.exists():
        ov = _extract_bridge_overlay(bridge_log)
        if ov["drop_pos_ne"] is not None:
            drop_n, drop_e = ov["drop_pos_ne"]
            ax.scatter([drop_e], [drop_n], marker="D", color="#ff7f0e",
                       s=110, edgecolor="black", linewidth=1.0, zorder=6,
                       label=f"mass drop ({ov['payload_kg']:.2f} kg)")
            ax.annotate(f"drop t={ov['drop_t']:.0f}s",
                        (drop_e, drop_n), xytext=(8, -12),
                        textcoords="offset points", fontsize=9,
                        color="#ff7f0e")
        if ov["wind_label"] is not None:
            title_suffix = f"  wind: {ov['wind_label']}"
            # Wind arrow in upper-left corner (axes fraction coords).
            dn, de = _WIND_LABEL_TO_DIR[ov["wind_label"]]
            # arrow length in axes-fraction units
            L = 0.10
            x0, y0 = 0.10, 0.90  # tail
            ax.annotate(
                "", xy=(x0 + L * de, y0 + L * dn), xytext=(x0, y0),
                xycoords="axes fraction",
                arrowprops=dict(arrowstyle="->", color="#2ca02c", lw=2),
            )
            ax.text(x0, y0 + 0.04,
                    f"wind {ov['wind_label']} (~5 m/s peak)",
                    transform=ax.transAxes, color="#2ca02c", fontsize=9)

    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"{mission.name} — {log_path.stem}{title_suffix}")
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()

    out_png = out_dir / f"{log_path.stem}.png"
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return out_png


DEFAULT_HOME_LAT = 40.192
DEFAULT_HOME_LON = 44.50446
DEFAULT_MASTER = "udpin:localhost:14551"
DEFAULT_MONITOR_TIMEOUT_S = 900.0  # 15 min wall ~= 6 min sim at Isaac 40% realtime


# Auto-route logs by mission name. Keys match the `name:` field of each mission
# YAML in lab/missions/cases/. Values are subdirectories under
# `logs/mission_logs/`. For RL-deployed runs, override via `--log-subdir rl_*`.
DEFAULT_LOG_SUBDIRS: dict[str, str] = {
    "square_100m": "baseline_square100",
    "square_20m":  "baseline_square20",
    "aua_short":   "baseline_pid_aua",
    "fig8_50m":    "baseline_fig8_50m",
    "survey_3x3":  "baseline_survey_3x3",
}


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
                   verbose: bool = True, *, exit_on_disarm: bool = False) -> None:
    """Read MAVLink messages for up to `duration` seconds, logging each.

    When `exit_on_disarm=True`, returns early as soon as the vehicle reports
    `armed -> disarmed` via HEARTBEAT (must have observed an armed heartbeat
    first, so we don't trip on the disarmed steady-state before takeoff).
    The post-mission monitor uses this so the runner exits cleanly after
    LAND auto-disarm — otherwise it would keep logging
    `MISSION_CURRENT: seq=0` heartbeats until the timeout.
    """
    from pymavlink import mavutil
    end = time.time() + duration
    saw_armed = False
    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue
        logger.mavlink(msg)
        mtype = msg.get_type()

        if exit_on_disarm and mtype == "HEARTBEAT":
            armed_now = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed_now:
                saw_armed = True
            elif saw_armed:
                logger.event("monitor_exit_on_disarm")
                if verbose:
                    print("[monitor] vehicle disarmed — mission complete, exiting monitor.")
                return

        if not verbose:
            continue
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


def wait_disarmed(master, mavutil, timeout: float = 30.0) -> bool:
    """Wait until the vehicle reports disarmed via heartbeat. Used between
    consecutive runs in a batch — the previous run's mission may still be
    executing or descending when the next run kicks off."""
    end = time.time() + timeout
    while time.time() < end:
        if not is_armed(master, mavutil):
            return True
        time.sleep(0.5)
    return False


def cleanup_for_next_run(master, mavutil, logger, timeout: float = 60.0) -> None:
    """Force the vehicle into a state where MISSION_CLEAR_ALL will be accepted.

    ArduCopter rejects mission clear while AUTO is actively executing or while
    the vehicle is airborne+armed. Switch to GUIDED first (stops AUTO mode
    state machine), then if still armed wait for natural disarm (the previous
    RTL/LAND should bring it down). If after `timeout` it's still armed,
    force-disarm so the next run can proceed.
    """
    if is_armed(master, mavutil):
        print("[cleanup] vehicle still armed -- switching to GUIDED to stop AUTO")
        try:
            set_mode(master, mavutil, "GUIDED")
        except RuntimeError as e:
            print(f"[cleanup] set_mode GUIDED failed: {e!r}")
        if logger is not None:
            drain_messages(master, logger, duration=1.0)
        if not wait_disarmed(master, mavutil, timeout=timeout):
            print("[cleanup] still armed after wait -- force-disarming")
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 21196, 0, 0, 0, 0, 0,
            )
            time.sleep(1.0)
    else:
        # Disarmed but possibly still in AUTO from a prior crash/abort; flip
        # to GUIDED so the upload's clear-all isn't rejected on mode grounds.
        try:
            set_mode(master, mavutil, "GUIDED")
        except RuntimeError:
            pass


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

    # lab/ lives at Swarm_Drones/lab/; logs/ lives at the repo root
    # (Swarm_Drones/), which is 2 levels up from lab/missions/runner.py.
    log_dir = log_dir or Path(__file__).resolve().parents[2] / "logs" / "mission_logs"
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

        # Clean state before uploading: if the previous run is still flying
        # or armed, MISSION_CLEAR_ALL gets rejected. Stop AUTO and wait for
        # disarm (force-disarm if it overruns). 60 s wall covers Isaac's
        # 40 % realtime ratio with margin for a long descent.
        cleanup_for_next_run(master, mavutil, logger, timeout=60.0)
        drain_messages(master, logger, duration=2.0)

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

        # Climb timeout: Isaac+SITL runs at ~40 % of wall-clock realtime, plus
        # ~10-15 s of pre-climb motor spin-up before the drone visibly lifts.
        # The original 30 s floor false-failed on a 30 m takeoff (drone reached
        # only 5 m at t=29 s wall while still accelerating). Set to 30 min
        # wall, which is "effectively unlimited" for any sane mission while
        # still bailing if SITL/EKF hangs entirely.
        climb_timeout = 1800.0
        if not takeoff_guided_and_wait(
                master, mavutil, alt=mission.takeoff.altitude_m, logger=logger,
                climb_timeout=climb_timeout):
            raise RuntimeError("Takeoff did not reach target altitude.")

        # Skip past the takeoff item and switch to AUTO for waypoints + RTL.
        set_current_mission_item(master, mavutil, seq=1)
        drain_messages(master, logger, duration=1.0)

        set_mode(master, mavutil, "AUTO")
        drain_messages(master, logger, duration=2.0)

        start_mission(master, mavutil)
        logger.event("mission_started")
        drain_messages(master, logger, duration=monitor_timeout_s, verbose=True,
                       exit_on_disarm=True)

        logger.event("monitor_done")
        print("Mission monitoring window finished.")
        return logger.path
    finally:
        logger.close()
        # Auto-generate the 2D XY trajectory PNG for this run. Shared report
        # folder per log dir so all runs in a batch end up side-by-side.
        # Best-effort overlay of the matching bridge flight_*.jsonl (wind +
        # drop): looks for the most-recently-modified flight_*.jsonl in the
        # SAME directory whose mtime is within 10 minutes of the mission log
        # (user convention: bridge log gets moved alongside the mission log).
        try:
            report_dir = logger.path.parent / "report_flight"
            bridge_log = _find_matching_bridge_log(logger.path)
            out_png = plot_run_xy(logger.path, mission, home, report_dir,
                                   bridge_log=bridge_log)
            if out_png is not None:
                print(f"XY plot: {out_png}"
                      + (f"  (with bridge overlay: {bridge_log.name})"
                         if bridge_log else "  (no bridge log found — no overlay)"))
        except Exception as e:
            print(f"[plot] auto-plot failed: {e!r}")


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
    p.add_argument("--log-subdir", default=None,
                    help="Subdirectory under logs/mission_logs/ for this run's JSONL. "
                         "If omitted, auto-routes by mission name "
                         "(see DEFAULT_LOG_SUBDIRS). Pass e.g. 'rl_square100' "
                         "for RL-deployed runs of the same mission.")
    args = p.parse_args(argv)

    mission = load(args.mission)
    home = HomePosition(lat=args.home_lat, lon=args.home_lon)

    print(f"Mission: {mission.name}")
    print(f"  takeoff alt: {mission.takeoff.altitude_m} m")
    print(f"  waypoints:   {len(mission.waypoints)}")
    print(f"  return:      {mission.return_.type}")
    print(f"  home:        ({home.lat}, {home.lon})")

    # Resolve log subdirectory: CLI override > auto-routing by mission name >
    # empty (writes to logs/mission_logs/ root).
    mission_logs_root = Path(__file__).resolve().parents[2] / "logs" / "mission_logs"
    subdir = args.log_subdir or DEFAULT_LOG_SUBDIRS.get(mission.name, "")
    log_dir = mission_logs_root / subdir if subdir else mission_logs_root
    print(f"  log dir:     {log_dir}")
    print()

    log_path = run_mission(
        mission, home,
        master_url=args.master,
        log_dir=log_dir,
        monitor_timeout_s=args.monitor,
    )
    print(f"\nDone. Log: {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
