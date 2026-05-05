"""Pretty-print a bridge JSONL flight log for human eyes.

Usage:
    python -m capstone.common.logview LOG.jsonl
    python -m capstone.common.logview LOG.jsonl --interval 2
    python -m capstone.common.logview LOG.jsonl --gate wind5
    python -m capstone.common.logview LOG.jsonl --full

Sections:
  HEADER           file name, duration, sample counts
  CALIBRATION      values from motor_model_calibrated event
  DISTURBANCE      values from capstone_disturbance_active event
  TIMELINE         all bridge lifecycle events with absolute t
  HOVER WINDOW     metrics over the time the drone is at target altitude
  SNAPSHOTS        sampled state every --interval seconds
  VERDICT          if --gate was given, pass/fail vs that profile

The JSONL file remains the source of truth. This tool only reads.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

from capstone.common.logging import FlightLog, load
from capstone.control.metrics import compute_metrics


# -----------------------------------------------------------------------------
# Formatting helpers
# -----------------------------------------------------------------------------
def _hr(width: int = 70, char: str = "=") -> str:
    return char * width


def _fmt_seconds(t: float) -> str:
    return f"{t:8.3f} s"


def _fmt_vec3(v, width: int = 7, prec: int = 3) -> str:
    return "[" + ", ".join(f"{float(x):+{width}.{prec}f}" for x in v) + "]"


def _kv(rows: list[tuple[str, str]], indent: str = "    ", key_w: int = 16) -> str:
    return "\n".join(f"{indent}{k:<{key_w}}{v}" for k, v in rows)


def _maybe(d: dict, key: str, default: str = "-") -> str:
    return str(d.get(key, default))


# -----------------------------------------------------------------------------
# Section renderers
# -----------------------------------------------------------------------------
def render_header(log: FlightLog) -> str:
    name = log.path.name
    duration = log.duration_s
    n_states = len(log.states)
    n_packets = len(log.sitl_packets)
    state_hz = (n_states / duration) if duration > 0 else 0.0
    pkt_hz = (n_packets / duration) if duration > 0 else 0.0

    kinds = Counter(e.get("event", "?") for e in log.events)
    events_summary = ", ".join(f"{n}x {k}" for k, n in sorted(kinds.items())) or "(none)"

    out = [
        _hr(),
        f"  {name}",
        _hr(),
        _kv([
            ("duration",       f"{duration:.2f} s"),
            ("state samples",  f"{n_states} @ ~{state_hz:.1f} Hz"),
            ("SITL packets",   f"{n_packets} @ ~{pkt_hz:.1f} Hz"),
            ("events",         f"{len(log.events)}  ({events_summary})"),
        ]),
    ]
    return "\n".join(out)


def render_calibration(log: FlightLog) -> str:
    e = log.find_event("motor_model_calibrated")
    if not e:
        return "\n  CALIBRATION\n    (not present in log)"
    com = e.get("com_local_base_m", [0, 0, 0])
    return "\n".join([
        "",
        "  CALIBRATION",
        _kv([
            ("mass",           f"{e['total_mass_kg']:.4f} kg"),
            ("omega_hover",    f"{e['omega_hover_rad_s']:.1f} rad/s"),
            ("K_thrust",       f"{e['K_thrust']:.3e}  N/(rad/s)^2"),
            ("K_torque",       f"{e['K_torque']:.3e}  Nm/(rad/s)^2"),
            ("CoM (mm)",       _fmt_vec3([x * 1000 for x in com], width=7, prec=2)),
            ("physics_hz",     f"{e['physics_hz']:.1f}"),
        ]),
    ])


def render_legend(log: FlightLog) -> str:
    """Pretty-print the per-key documentation written by the bridge.

    The bridge emits one `log_schema` event per (stream, field) pair at
    log-start, so the JSONL stays line-by-line readable. We regroup them
    here. Older logs without any log_schema events get a short fallback.
    """
    schema_evs = [e for e in log.events if e.get("event") == "log_schema"]
    if not schema_evs:
        return "\n".join([
            "",
            "  LEGEND",
            "    (log_schema events missing -- pre-2026-05-05 log; key meanings:",
            "     pos_ned = position NED, vel_ned = velocity NED, rpy = roll/pitch/yaw,",
            "     gyro_frd / accel_frd = body FRD frame.)",
        ])
    # Group by stream while preserving insertion order within each stream.
    groups: dict[str, list[tuple[str, str]]] = {}
    for e in schema_evs:
        groups.setdefault(e.get("stream", "?"), []).append(
            (str(e.get("field", "?")), str(e.get("desc", "")))
        )
    src_labels = {"state": "isaac->sitl", "sitl_packet": "sitl->isaac", "events": "bridge"}
    lines = ["", "  LEGEND"]
    for stream, rows in groups.items():
        lines.append(f"    [{src_labels.get(stream, stream)}]")
        for field, desc in rows:
            lines.append(f"      {field:<16}{desc}")
    return "\n".join(lines)


def render_disturbance(log: FlightLog) -> str:
    e = log.find_event("capstone_disturbance_active")
    if not e:
        return ""
    return "\n".join([
        "",
        "  DISTURBANCE",
        _kv([
            ("profile",         _maybe(e, "profile")),
            ("mass_multiplier", f"{e.get('mass_multiplier', 1.0):.3f}"),
            ("seed",            _maybe(e, "seed")),
        ]),
    ])


def render_timeline(log: FlightLog) -> str:
    lines = ["", "  TIMELINE"]
    if not log.events:
        lines.append("    (no events)")
        return "\n".join(lines)
    # `log_schema` rows are pure metadata (rendered by render_legend); skip
    # them here so the TIMELINE stays focused on actual lifecycle events.
    schema_count = sum(1 for e in log.events if e.get("event") == "log_schema")
    if schema_count:
        lines.append(f"    ({schema_count} log_schema rows omitted -- see LEGEND)")
    for e in log.events:
        if e.get("event") == "log_schema":
            continue
        t = float(e.get("t", 0.0))
        name = e.get("event", "?")
        # Pull a couple of useful fields next to the name when present.
        extras = []
        for key in ("profile", "addr"):
            if key in e:
                extras.append(f"{key}={e[key]}")
        suffix = (" (" + ", ".join(extras) + ")") if extras else ""
        lines.append(f"  {_fmt_seconds(t)}  {name}{suffix}")
    return "\n".join(lines)


def render_hover_window(log: FlightLog) -> str:
    # Use the metrics module so this view stays in lockstep with the gates.
    m = compute_metrics(log, "calm", log_path=str(log.path))
    lines = ["", "  HOVER WINDOW"]
    if m.window_t0 is None:
        lines.append("    (no airborne hover window found)")
        return "\n".join(lines)
    dur = (m.window_t1 - m.window_t0) if m.window_t1 is not None else 0.0
    lines.append(_kv([
        ("range",       f"{m.window_t0:.2f} - {m.window_t1:.2f} s  ({dur:.2f} s, "
                        f"{m.n_window_samples} samples)"),
        ("altitude",    f"{m.alt_mean_m:.3f} +/- {m.alt_std_m:.4f} m"),
        ("xy std",      f"{m.xy_std_m:.4f} m  (max excursion {m.xy_excursion_max_m:.3f} m)"),
        ("gyro RMS",    f"{m.gyro_rms:.4f} rad/s  (peak {m.gyro_peak:.4f})"),
        ("roll std",    f"{m.roll_std:.4f} rad"),
        ("pitch std",   f"{m.pitch_std:.4f} rad"),
        ("crashed",     str(m.crashed) + (f" ({m.crash_reason})" if m.crash_reason else "")),
    ]))
    return "\n".join(lines)


def render_snapshots(log: FlightLog, interval_s: float) -> str:
    lines = ["", f"  STATE SNAPSHOTS  (every {interval_s:.1f} s)"]
    if not log.states:
        lines.append("    (no state samples)")
        return "\n".join(lines)
    t0 = float(log.states[0]["t"])
    t_end = float(log.states[-1]["t"])
    have_wind = any("wind_world" in s for s in log.states)

    header = (f"    {'t':>8}  {'alt':>6}  {'vz':>6}  {'vxy':>5}  "
              f"{'roll':>7}  {'pitch':>7}  {'yaw':>7}  {'|gyro|':>7}")
    if have_wind:
        header += f"  {'|wind|':>7}"
    lines.append(header)

    next_t = t0
    for s in log.states:
        t = float(s["t"])
        if t < next_t:
            continue
        alt = -float(s["pos_ned"][2])
        vz = -float(s["vel_ned"][2])
        vxy = math.hypot(float(s["vel_ned"][0]), float(s["vel_ned"][1]))
        roll, pitch, yaw = (float(x) for x in s["rpy"])
        gyro_mag = math.sqrt(sum(float(g) ** 2 for g in s["gyro_frd"]))
        row = (f"    {t:8.2f}  {alt:6.2f}  {vz:+6.2f}  {vxy:5.2f}  "
               f"{roll:+7.3f}  {pitch:+7.3f}  {yaw:+7.3f}  {gyro_mag:7.4f}")
        if have_wind:
            wmag = 0.0
            if "wind_world" in s:
                wmag = math.sqrt(sum(float(x) ** 2 for x in s["wind_world"]))
            row += f"  {wmag:7.2f}"
        lines.append(row)
        next_t += interval_s
        if next_t > t_end:
            break
    return "\n".join(lines)


def render_verdict(log: FlightLog, profile: str) -> str:
    m = compute_metrics(log, profile, log_path=str(log.path))
    verdict = "PASS" if m.passed else "FAIL"
    out = ["", f"  VERDICT  vs profile {profile!r}: {verdict}"]
    if m.failures:
        out.extend(f"    - {f}" for f in m.failures)
    return "\n".join(out)


# -----------------------------------------------------------------------------
# Driver
# -----------------------------------------------------------------------------
def render(log: FlightLog, *, interval_s: float, gate_profile: str | None) -> str:
    parts = [
        render_header(log),
        render_legend(log),
        render_calibration(log),
        render_disturbance(log),
        render_timeline(log),
        render_hover_window(log),
        render_snapshots(log, interval_s),
    ]
    if gate_profile:
        parts.append(render_verdict(log, gate_profile))
    parts.append(_hr())
    return "\n".join(parts)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("log", type=Path, help="JSONL flight log")
    p.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="seconds between state snapshots (default 5; --full -> 1)",
    )
    p.add_argument(
        "--full",
        action="store_true",
        help="dense output: 1 s state snapshots and full timeline",
    )
    p.add_argument(
        "--gate",
        default=None,
        help="grade against this profile and add a VERDICT section "
             "(calm | mass_drop_300g | wind5 | wind_up3 | wind_down3 | "
             "imu_noise | worst_case)",
    )
    args = p.parse_args(argv)

    interval_s = 1.0 if args.full else float(args.interval)

    log = load(args.log)
    print(render(log, interval_s=interval_s, gate_profile=args.gate))
    return 0


if __name__ == "__main__":
    sys.exit(main())
