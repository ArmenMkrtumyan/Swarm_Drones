"""Hover benchmark: lock in current ArduCopter PID performance per profile.

Walks a folder of bridge JSONL flight logs, detects which disturbance profile
each log used, computes the same metrics that the Stage-1 gates use, and writes
a benchmark package:

  benchmark_hover_report/
    baseline.csv            -- one row per log, summary stats
    baseline.json           -- full metric records for machine consumption
    plots/<log>_series.png  -- per-log time series (alt / xy / attitude / gyro)
    comparison.png          -- cross-profile bar chart (only if 2+ profiles)
                              For curated calm-vs-worst_case-vs-RL comparison,
                              use `capstone.control.compare_batches` instead.

Without this benchmark locked, "RL beat ArduCopter" is unmeasurable. Sister
tool for Stage-2 missions: capstone.missions.benchmark_mission.

Usage:
    python -m capstone.control.benchmark_hover flight_logs/
    python -m capstone.control.benchmark_hover flight_logs/ --out benchmark_hover_report
    python -m capstone.control.benchmark_hover flight_logs/ --no-plots       # CSV/JSON only
    python -m capstone.control.benchmark_hover flight_logs/ --require-event  # official:
                                                                              # only logs from
                                                                              # the new bridge
                                                                              # with a chosen
                                                                              # profile
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

from capstone.common.logging import FlightLog, load
from capstone.control.metrics import (
    PROFILE_GATES,
    HoverMetrics,
    compute_metrics,
)
from capstone.control import plot_style


# -----------------------------------------------------------------------------
# Profile detection: prefer the capstone_disturbance_active event, fall back
# to filename hint when the event is missing (typical for older logs).
# -----------------------------------------------------------------------------
FILENAME_HINTS = {
    "WIND_UP3":      "wind_up3",
    "WIND_DOWN3":    "wind_down3",
    "WIND5":         "wind5",
    "MASS_DROP_300": "mass_drop_300g",
    "MASSDROP300":   "mass_drop_300g",
    "DROP300":       "mass_drop_300g",
    "IMU_NOISE":     "imu_noise",
    "WORST":         "worst_case",
    "CALM":          "calm",
}


def detect_profile(log: FlightLog) -> str:
    ev = log.find_event("capstone_disturbance_active")
    if ev and ev.get("profile") in PROFILE_GATES:
        return ev["profile"]
    name = log.path.name.upper()
    for hint, profile in FILENAME_HINTS.items():
        if hint in name:
            return profile
    return "calm"


# -----------------------------------------------------------------------------
# Action-window detection: from first to last sample with altitude above this
# threshold. Used to crop the zoomed-in time-series plot so the eye lands on
# the part of the flight where the drone is doing something.
# -----------------------------------------------------------------------------
ACTION_ALT_THRESHOLD_M = 0.2
ACTION_PAD_S = 1.0


def find_action_window(states: list[dict]) -> tuple[float | None, float | None]:
    """Return (t_action_start, t_landed) padded by ACTION_PAD_S on each side,
    or (None, None) if the drone never went above ACTION_ALT_THRESHOLD_M."""
    if not states:
        return None, None
    above = [s for s in states if -float(s["pos_ned"][2]) > ACTION_ALT_THRESHOLD_M]
    if not above:
        return None, None
    t_first = float(above[0]["t"])
    t_last = float(above[-1]["t"])
    t_log_start = float(states[0]["t"])
    t_log_end = float(states[-1]["t"])
    return (
        max(t_log_start, t_first - ACTION_PAD_S),
        min(t_log_end, t_last + ACTION_PAD_S),
    )


# -----------------------------------------------------------------------------
# Per-log series extraction (used by both CSV/JSON metrics and plots).
# -----------------------------------------------------------------------------
def extract_series(log: FlightLog) -> dict[str, list[float]]:
    """Return parallel lists of t, alt, xy_radius, roll, pitch, yaw, gyro_mag,
    wind_mag (or None if no wind_world fields)."""
    ts: list[float] = []
    alts: list[float] = []
    xs: list[float] = []
    ys: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    yaws: list[float] = []
    gyros: list[float] = []
    gyros_truth: list[float] = []
    have_truth = any("gyro_frd_truth" in s for s in log.states)
    have_wind = any("wind_world" in s for s in log.states)
    winds: list[float] = []
    for s in log.states:
        ts.append(float(s["t"]))
        pos = s["pos_ned"]
        alts.append(-float(pos[2]))
        xs.append(float(pos[0]))
        ys.append(float(pos[1]))
        r, p, y = (float(x) for x in s["rpy"])
        rolls.append(r)
        pitches.append(p)
        yaws.append(y)
        gyros.append(math.sqrt(sum(float(g) ** 2 for g in s["gyro_frd"])))
        # Truth gyro: present only on noise-injecting profiles. Fall back to
        # the post-noise reading so calm/wind/mass logs render a single line
        # (truth == post-noise) without a separate codepath.
        gyro_t_src = s.get("gyro_frd_truth", s["gyro_frd"])
        gyros_truth.append(math.sqrt(sum(float(g) ** 2 for g in gyro_t_src)))
        if have_wind:
            w = s.get("wind_world", [0.0, 0.0, 0.0])
            winds.append(math.sqrt(sum(float(x) ** 2 for x in w)))
    return {
        "t": ts,
        "alt": alts,
        "x": xs,
        "y": ys,
        "roll": rolls,
        "pitch": pitches,
        "yaw": yaws,
        "gyro_mag": gyros,
        "gyro_mag_truth": gyros_truth,
        "have_gyro_truth": have_truth,
        "wind_mag": winds if have_wind else [],
    }


# -----------------------------------------------------------------------------
# Plot generation (matplotlib).
# -----------------------------------------------------------------------------
def describe_profile(profile_name: str, total_mass_kg: float | None) -> str:
    """One-line human-readable summary of the disturbance config that produced
    this log. Reads parameters straight from the disturbance module so the
    description stays in lockstep with the harness."""
    from capstone.control.disturbance import make as make_profile

    try:
        p = make_profile(profile_name, seed=0)
    except KeyError:
        return f"profile={profile_name} (unknown)"

    parts = []

    if p.wind.sigma_mps > 0:
        peak_estimate = p.wind.sigma_mps * 3.0  # OU 3-sigma envelope
        parts.append(f"wind ~{peak_estimate:.0f} m/s peaks")
    else:
        parts.append("wind: calm")

    if p.mass_drop is not None:
        payload_g = p.mass_drop.payload_kg * 1000.0
        drop_t = p.mass_drop.drop_after_hover_s
        if total_mass_kg is not None:
            loaded = total_mass_kg + p.mass_drop.payload_kg
            parts.append(
                f"mass {total_mass_kg:.3f} kg + {payload_g:.0f} g payload "
                f"(loaded {loaded:.3f} kg), drops after {drop_t:.0f} s hover"
            )
        else:
            parts.append(
                f"mass nominal + {payload_g:.0f} g payload, "
                f"drops after {drop_t:.0f} s hover"
            )
    elif total_mass_kg is not None:
        m = total_mass_kg * p.mass_multiplier
        pct = (p.mass_multiplier - 1.0) * 100
        if abs(pct) < 0.5:
            parts.append(f"mass {m:.3f} kg (nominal)")
        else:
            parts.append(f"mass {m:.3f} kg ({pct:+.0f}%)")
    else:
        parts.append("mass nominal" if abs(p.mass_multiplier - 1.0) < 1e-6
                     else f"mass {(p.mass_multiplier - 1.0)*100:+.0f}%")

    if p.imu.gyro_white_sigma > 0 or p.imu.accel_white_sigma > 0:
        parts.append(
            f"IMU noise: gyro {p.imu.gyro_white_sigma:g} rad/s + "
            f"accel {p.imu.accel_white_sigma:g} m/s^2 + bias walk"
        )
    else:
        parts.append("IMU clean")

    return "    ".join(parts)


def format_gate_annotation(
    metric_name: str,
    profile: str,
    actual_value: float | None,
    label: str | None = None,
) -> tuple[str, str] | None:
    """Pure helper: build the (text, edge_color) for a per-panel gate box.

    Returns None if there's nothing to annotate (no actual value yet). Edge
    color encodes status: green = gated and passing, red = gated and failing,
    grey = not gated for this profile (metric still reported).
    """
    if actual_value is None:
        return None
    pretty = label or metric_name
    gates = PROFILE_GATES.get(profile, {})
    if metric_name in gates:
        cmp, threshold = gates[metric_name]
        ok = (
            (cmp == "<" and actual_value < threshold)
            or (cmp == "<=" and actual_value <= threshold)
            or (cmp == "==" and actual_value == threshold)
        )
        status = "PASS" if ok else "FAIL"
        edge = plot_style.COLORS["gate_pass_edge"] if ok else plot_style.COLORS["gate_fail_edge"]
        text = f"{pretty} = {actual_value:.4g}   gate: {cmp} {threshold}  [{status}]"
        return text, edge
    return (
        f"{pretty} = {actual_value:.4g}   (no gate for this profile)",
        plot_style.COLORS["gate_none_edge"],
    )


def _annotate_gate(ax, metric_name: str, profile: str,
                   actual_value: float | None, label: str | None = None) -> None:
    """Single-metric box: upper-right of `ax`."""
    out = format_gate_annotation(metric_name, profile, actual_value, label)
    if out is None:
        return
    text, edge = out
    ax.text(
        0.99, 0.97, text,
        transform=ax.transAxes, ha="right", va="top",
        fontsize=7.5, color="#222",
        bbox=dict(boxstyle="round,pad=0.25",
                  facecolor="white", edgecolor=edge, alpha=0.9),
    )


def _annotate_gate_pair(ax, specs: list[tuple[str, str, float | None, str]]) -> None:
    """Stack two metric boxes vertically in the upper-right.

    Each spec is (metric_name, profile, actual_value, label). Used for the
    xy and roll/pitch panels which each have a per-axis gate pair.
    """
    formatted = [format_gate_annotation(m, p, v, lbl) for m, p, v, lbl in specs]
    formatted = [f for f in formatted if f is not None]
    if not formatted:
        return
    # Stack from top down: y=0.97, then y=0.80 (each box ~0.13 of axes height
    # for a 4-panel figure of this size; eyeballed from the sample plots).
    for i, (text, edge) in enumerate(formatted):
        y = 0.97 - i * 0.18
        ax.text(
            0.99, y, text,
            transform=ax.transAxes, ha="right", va="top",
            fontsize=7.5, color="#222",
            bbox=dict(boxstyle="round,pad=0.25",
                      facecolor="white", edgecolor=edge, alpha=0.9),
        )


def plot_log(
    log: FlightLog,
    metrics: HoverMetrics,
    out_path: Path,
    *,
    t_range: tuple[float, float] | None = None,
) -> None:
    """Render the per-log time-series plot.

    If `t_range` is given, the x-axis is clipped to that window (a "zoomed"
    view); the underlying data is unchanged so the plot lines still show the
    full trace, just cropped. Title gets a "(zoom)" suffix in that case.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    s = extract_series(log)
    if not s["t"]:
        return

    n_rows = 4
    fig, axes = plt.subplots(n_rows, 1, figsize=(10, 2.2 * n_rows), sharex=True)

    win_t0, win_t1 = metrics.window_t0, metrics.window_t1

    def shade_window(ax):
        if win_t0 is not None and win_t1 is not None:
            # No legend label -- the subtitle explains the shading.
            ax.axvspan(win_t0, win_t1, color=plot_style.COLORS["hover_window"], alpha=0.4)

    # mass_drop event marker (vertical line + small label) — only present in
    # mass_drop_* logs. Drawn in every panel so the eye can correlate the drop
    # to altitude / xy / attitude / gyro response together.
    drop_ev = log.find_event("mass_drop_event")
    drop_t = float(drop_ev["t"]) if drop_ev else None

    def mark_drop(ax):
        if drop_t is not None:
            ax.axvline(
                drop_t,
                color=plot_style.COLORS["mass_drop_event"],
                linewidth=plot_style.LINE_WIDTHS["marker"],
                linestyle="--",
                alpha=0.8,
            )

    # 1) Altitude
    ax = axes[0]
    ax.plot(s["t"], s["alt"],
            color=plot_style.COLORS["altitude"],
            linewidth=plot_style.LINE_WIDTHS["altitude"])
    shade_window(ax)
    mark_drop(ax)
    if drop_t is not None:
        ax.annotate(
            "payload dropped",
            xy=(drop_t, 0), xycoords=("data", "axes fraction"),
            xytext=(4, 4), textcoords="offset points",
            fontsize=7.5, color=plot_style.COLORS["mass_drop_event"],
        )
    ax.set_ylabel(plot_style.LABELS["altitude_y"])
    ax.grid(True, alpha=0.3)
    _annotate_gate(ax, "alt_std_m", metrics.profile, metrics.alt_std_m,
                   label="alt_std")

    # 2) Horizontal position relative to home -- separate N and E so you can
    # see direction, not just magnitude. NED frame is anchored at home-lock,
    # so "north" = drone's initial-forward direction (positive = drone moved
    # forward from launch), "east" = drone's initial-right direction.
    ax = axes[1]
    ax.plot(s["t"], s["x"],
            color=plot_style.COLORS["north"],
            linewidth=plot_style.LINE_WIDTHS["position"],
            label=plot_style.LABELS["north_line"])
    ax.plot(s["t"], s["y"],
            color=plot_style.COLORS["east"],
            linewidth=plot_style.LINE_WIDTHS["position"],
            label=plot_style.LABELS["east_line"])
    shade_window(ax)
    mark_drop(ax)
    ax.set_ylabel(plot_style.LABELS["position_y"])
    # Legend goes upper-LEFT so it doesn't collide with the gate boxes.
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    # Gated metric is RMS (typical position error). Max is shown as a
    # smaller "(also: ...)" suffix so the worst transient stays visible
    # without driving the verdict.
    _annotate_gate_pair(ax, [
        ("pos_rms_north_m", metrics.profile, metrics.pos_rms_north_m,
         f"N rms (max {metrics.pos_max_north_m:.3f})"
         if metrics.pos_max_north_m is not None else "N rms"),
        ("pos_rms_east_m",  metrics.profile, metrics.pos_rms_east_m,
         f"E rms (max {metrics.pos_max_east_m:.3f})"
         if metrics.pos_max_east_m is not None else "E rms"),
    ])

    # 3) Attitude (roll/pitch). Yaw not shown -- it doesn't drive position hold
    # and including it would compress roll/pitch off the visible scale.
    ax = axes[2]
    ax.plot(s["t"], s["roll"],
            color=plot_style.COLORS["roll"],
            linewidth=plot_style.LINE_WIDTHS["attitude"],
            label=plot_style.LABELS["roll_line"])
    ax.plot(s["t"], s["pitch"],
            color=plot_style.COLORS["pitch"],
            linewidth=plot_style.LINE_WIDTHS["attitude"],
            label=plot_style.LABELS["pitch_line"])
    shade_window(ax)
    mark_drop(ax)
    ax.set_ylabel(plot_style.LABELS["attitude_y"])
    # Legend on left so the gate boxes can stack on the right.
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    _annotate_gate_pair(ax, [
        ("roll_rms_rad",  metrics.profile, metrics.roll_rms_rad,
         f"roll rms (max {metrics.roll_max_rad:.3f})"
         if metrics.roll_max_rad is not None else "roll rms"),
        ("pitch_rms_rad", metrics.profile, metrics.pitch_rms_rad,
         f"pitch rms (max {metrics.pitch_max_rad:.3f})"
         if metrics.pitch_max_rad is not None else "pitch rms"),
    ])

    # 4) Angular speed magnitude: |gyro| = rotation speed regardless of axis.
    # Solid line = post-noise (what controller saw, the gated value).
    # Dotted line = truth (real attitude motion, only differs when imu_noise
    # is being injected). Same color so they read as the same signal.
    ax = axes[3]
    ax.plot(s["t"], s["gyro_mag"],
            color=plot_style.COLORS["gyro_mag"],
            linewidth=plot_style.LINE_WIDTHS["gyro"],
            label="post-noise (sent to SITL)" if s.get("have_gyro_truth") else None)
    if s.get("have_gyro_truth"):
        ax.plot(s["t"], s["gyro_mag_truth"],
                color=plot_style.COLORS["gyro_mag"],
                linewidth=plot_style.LINE_WIDTHS["gyro"],
                linestyle=":",
                label="truth (pre-noise)")
        ax.legend(loc="upper left", fontsize=8)
    shade_window(ax)
    mark_drop(ax)
    ax.set_ylabel(plot_style.LABELS["gyro_y"])
    ax.grid(True, alpha=0.3)
    if s.get("have_gyro_truth") and metrics.gyro_rms_truth is not None:
        _annotate_gate_pair(ax, [
            ("gyro_rms", metrics.profile, metrics.gyro_rms, "gyro_rms"),
            ("gyro_rms_truth", metrics.profile, metrics.gyro_rms_truth,
             "gyro_rms (truth)"),
        ])
    else:
        _annotate_gate(ax, "gyro_rms", metrics.profile, metrics.gyro_rms,
                       label="gyro_rms")

    axes[-1].set_xlabel(plot_style.LABELS["time_x"])
    if t_range is not None:
        for ax in axes:
            ax.set_xlim(t_range)

    verdict = "PASS" if metrics.passed else "FAIL"

    # Top-line title: identity + verdict. The zoom range used to be appended
    # here but was redundant -- the parent folder (plots/zoom vs plots/full)
    # already says which view this is, and the x-axis itself shows the range.
    fig.suptitle(
        f"{log.path.name}   profile={metrics.profile}   verdict={verdict}",
        fontsize=11, y=0.985,
    )

    # Subtitle: actual disturbance values + shading legend (replaces the
    # legacy |wind| panel).
    cal = log.find_event("motor_model_calibrated")
    base_mass = float(cal["total_mass_kg"]) if cal else None
    config_str = describe_profile(metrics.profile, base_mass)
    fig.text(
        0.5, 0.955,
        config_str + "    |    shaded region = hover window",
        ha="center", fontsize=9, color="#444",
    )

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    plot_style.savefig_dual(fig, out_path)
    plt.close(fig)


def plot_comparison(rows: list[dict], out_path: Path) -> None:
    """Bar chart of key metrics across all profiles in the baseline."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not rows:
        return

    # Aggregate by profile (mean across logs of that profile, if multiple).
    profiles_in_order = [
        p for p in ("calm", "mass_drop_300g",
                    "wind5", "wind_up3", "wind_down3",
                    "imu_noise", "worst_case")
        if any(r["profile"] == p for r in rows)
    ]
    # Show the metrics that actually drive the gates. Drop the deprecated
    # xy_std / xy_excursion_max from the comparison since they're no longer
    # gated.
    # Show the metrics that actually drive the gates (RMS-based for position
    # and attitude; max kept in CSV for diagnostic).
    metric_names = [
        "alt_std_m",
        "pos_rms_north_m", "pos_rms_east_m",
        "roll_rms_rad", "pitch_rms_rad",
        "gyro_rms",
    ]
    titles = [
        "alt std (m)",
        "north RMS |pos| (m)", "east RMS |pos| (m)",
        "roll RMS |angle| (rad)", "pitch RMS |angle| (rad)",
        "gyro RMS (rad/s)",
    ]

    display_labels = [plot_style.display_name(p) for p in profiles_in_order]
    bar_colors = [
        plot_style.COLORS["bar_calm"] if p == "calm"
        else plot_style.COLORS["bar_disturbed"]
        for p in profiles_in_order
    ]

    def _profile_mean(profile: str, metric: str) -> float | None:
        vals: list[float] = []
        for r in rows:
            if r["profile"] != profile:
                continue
            v = r.get(metric, None)
            if v is None or v == "":
                continue
            try:
                vals.append(float(v))
            except (TypeError, ValueError):
                continue
        return (sum(vals) / len(vals)) if vals else None

    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    import numpy as np  # local import — only needed for paired-bar layout
    for ax, mname, title in zip(axes.ravel(), metric_names, titles):
        means = [_profile_mean(p, mname) or 0.0 for p in profiles_in_order]
        # gyro_rms gets a paired truth bar ON TOP OF the post-noise bar so the
        # IMU-noise contribution to the metric is visually separable. For
        # profiles without IMU noise, the two values are identical and the
        # truth bar is hidden by the main bar — that's intentional, not noise.
        if mname == "gyro_rms":
            truth_means = [
                _profile_mean(p, "gyro_rms_truth") for p in profiles_in_order
            ]
            x = np.arange(len(profiles_in_order))
            w = 0.38
            bars = ax.bar(x - w/2, means, w, color=bar_colors,
                          label="post-noise (sent to SITL)")
            truth_present = any(t is not None for t in truth_means)
            if truth_present:
                ax.bar(x + w/2, [t or 0.0 for t in truth_means], w,
                       color=bar_colors, alpha=0.45, hatch="//",
                       edgecolor="white", label="truth (pre-noise)")
                ax.legend(loc="upper left", fontsize=7)
                for xi, tval in zip(x, truth_means):
                    if tval is None:
                        continue
                    ax.text(xi + w/2, tval, f"{tval:.4g}",
                            ha="center", va="bottom", fontsize=7)
            ax.set_xticks(x)
            ax.set_xticklabels(display_labels)
        else:
            bars = ax.bar(display_labels, means, color=bar_colors)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)
        # Rotate x-tick labels so longer profile names (wind_down3, worst_case)
        # don't overlap their neighbors.
        ax.tick_params(axis="x", labelrotation=30)
        for tick in ax.get_xticklabels():
            tick.set_horizontalalignment("right")
        for bar, val in zip(bars, means):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f"{val:.4g}", ha="center", va="bottom", fontsize=8)
    fig.suptitle("Hover benchmark: hardcoded-PID quality vs disturbance profile",
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    plot_style.savefig_dual(fig, out_path)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Driver
# -----------------------------------------------------------------------------
CSV_FIELDS = [
    "log_path", "profile", "duration_s", "n_state_samples",
    "window_t0", "window_t1", "window_duration_s",
    "alt_mean_m", "alt_std_m",
    "pos_rms_north_m", "pos_rms_east_m",
    "pos_max_north_m", "pos_max_east_m",
    "xy_std_m", "xy_excursion_max_m",
    "roll_rms_rad", "pitch_rms_rad",
    "roll_max_rad", "pitch_max_rad",
    "gyro_rms", "gyro_peak",
    "gyro_rms_truth", "gyro_peak_truth",
    "roll_std", "pitch_std",
    "crashed", "passed", "failures",
]


def metrics_to_csv_row(m: HoverMetrics) -> dict:
    duration = (
        (m.window_t1 - m.window_t0)
        if (m.window_t0 is not None and m.window_t1 is not None)
        else None
    )
    return {
        "log_path": Path(m.log_path).name,
        "profile": m.profile,
        "duration_s": round(m.duration_s, 2),
        "n_state_samples": m.n_state_samples,
        "window_t0": round(m.window_t0, 2) if m.window_t0 is not None else "",
        "window_t1": round(m.window_t1, 2) if m.window_t1 is not None else "",
        "window_duration_s": round(duration, 2) if duration is not None else "",
        "alt_mean_m": round(m.alt_mean_m, 4) if m.alt_mean_m is not None else "",
        "alt_std_m": round(m.alt_std_m, 4) if m.alt_std_m is not None else "",
        "pos_rms_north_m": round(m.pos_rms_north_m, 4)
            if m.pos_rms_north_m is not None else "",
        "pos_rms_east_m": round(m.pos_rms_east_m, 4)
            if m.pos_rms_east_m is not None else "",
        "pos_max_north_m": round(m.pos_max_north_m, 4)
            if m.pos_max_north_m is not None else "",
        "pos_max_east_m": round(m.pos_max_east_m, 4)
            if m.pos_max_east_m is not None else "",
        "xy_std_m": round(m.xy_std_m, 4) if m.xy_std_m is not None else "",
        "xy_excursion_max_m": round(m.xy_excursion_max_m, 4)
            if m.xy_excursion_max_m is not None else "",
        "roll_rms_rad": round(m.roll_rms_rad, 4)
            if m.roll_rms_rad is not None else "",
        "pitch_rms_rad": round(m.pitch_rms_rad, 4)
            if m.pitch_rms_rad is not None else "",
        "roll_max_rad": round(m.roll_max_rad, 4)
            if m.roll_max_rad is not None else "",
        "pitch_max_rad": round(m.pitch_max_rad, 4)
            if m.pitch_max_rad is not None else "",
        "gyro_rms": round(m.gyro_rms, 4) if m.gyro_rms is not None else "",
        "gyro_peak": round(m.gyro_peak, 4) if m.gyro_peak is not None else "",
        "gyro_rms_truth": round(m.gyro_rms_truth, 4)
            if m.gyro_rms_truth is not None else "",
        "gyro_peak_truth": round(m.gyro_peak_truth, 4)
            if m.gyro_peak_truth is not None else "",
        "roll_std": round(m.roll_std, 4) if m.roll_std is not None else "",
        "pitch_std": round(m.pitch_std, 4) if m.pitch_std is not None else "",
        "crashed": m.crashed,
        "passed": int(m.passed),
        "failures": "; ".join(m.failures or []),
    }


def collect_logs(folder: Path) -> list[Path]:
    return sorted(folder.glob("*.jsonl"))


# NOTE: README.md is hand-maintained by the user. The hover benchmark used to
# auto-generate it from PROFILE_GATES and disturbance configs, but that wiped
# the user's edits on every run. If you need an up-to-date gates table to
# paste, run `python -c "from capstone.control.metrics import PROFILE_GATES;
# import json; print(json.dumps(PROFILE_GATES, indent=2))"`.


def render_summary(rows: list[dict]) -> str:
    if not rows:
        return "no logs"
    lines = []
    width = {f: max(len(f), max(len(str(r[f])) for r in rows)) for f in CSV_FIELDS}
    show = ["profile", "log_path", "window_duration_s", "alt_std_m",
            "xy_std_m", "xy_excursion_max_m", "gyro_rms", "passed"]
    header = "  ".join(f"{f:<{width[f]}}" for f in show)
    lines.append(header)
    lines.append("-" * len(header))
    for r in sorted(rows, key=lambda x: (x["profile"], x["log_path"])):
        lines.append("  ".join(f"{str(r[f]):<{width[f]}}" for f in show))
    n_pass = sum(r["passed"] for r in rows)
    lines.append("")
    lines.append(f"{n_pass}/{len(rows)} logs pass their profile gates.")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("folder", type=Path, help="directory with *.jsonl flight logs")
    p.add_argument("--out", type=Path, default=Path("benchmark_hover_report"),
                   help="output directory (default: benchmark_hover_report)")
    p.add_argument("--no-plots", action="store_true",
                   help="skip matplotlib plots; CSV/JSON only")
    p.add_argument("--min-samples", type=int, default=200,
                   help="skip logs with fewer state samples (default: 200)")
    p.add_argument("--require-event", action="store_true",
                   help="only include logs containing a capstone_disturbance_active "
                        "event (i.e. produced by the new bridge with a deliberately "
                        "chosen profile). Excludes pre-edit and aborted logs.")
    args = p.parse_args(argv)

    if not args.folder.is_dir():
        print(f"error: {args.folder} is not a directory", file=sys.stderr)
        return 2

    args.out.mkdir(parents=True, exist_ok=True)
    # PNG and SVG live in parallel sibling trees (png/ and svg/) under the
    # report root, so the user can edit colors/labels in the SVGs without
    # the PNGs getting in the way. Per-log plots are further split into
    # full/ (entire flight) and zoom/ (takeoff-through-landing crop).
    png_root = args.out / "png"
    svg_root = args.out / "svg"
    plots_full_dir = png_root / "plots" / "full"
    plots_zoom_dir = png_root / "plots" / "zoom"
    # Wipe stale plots from prior runs (PNG side AND SVG side AND the legacy
    # flat layout). Only *.png and *.svg are deleted; nothing else.
    legacy_dirs = [args.out / "plots", args.out / "plots" / "full", args.out / "plots" / "zoom"]
    for d in (plots_full_dir, plots_zoom_dir,
              svg_root / "plots" / "full", svg_root / "plots" / "zoom",
              png_root, svg_root,
              *legacy_dirs):
        if d.exists():
            for ext in ("*.png", "*.svg"):
                for stale in d.glob(ext):
                    try:
                        stale.unlink()
                    except OSError as _e:
                        print(f"  warn: could not delete stale plot {stale.name}: {_e!r}")
    plots_full_dir.mkdir(parents=True, exist_ok=True)
    plots_zoom_dir.mkdir(parents=True, exist_ok=True)

    csv_rows: list[dict] = []
    json_records: list[dict] = []

    for log_path in collect_logs(args.folder):
        try:
            log = load(log_path)
        except Exception as e:
            print(f"  skip {log_path.name}: load error {e!r}")
            continue
        if len(log.states) < args.min_samples:
            print(f"  skip {log_path.name}: only {len(log.states)} state samples")
            continue
        if args.require_event and log.find_event("capstone_disturbance_active") is None:
            print(f"  skip {log_path.name}: no capstone_disturbance_active event "
                  f"(--require-event)")
            continue
        profile = detect_profile(log)
        m = compute_metrics(log, profile, log_path=str(log_path))
        csv_rows.append(metrics_to_csv_row(m))
        json_records.append(m.to_dict())
        verdict = "PASS" if m.passed else "FAIL"
        print(f"  {log_path.name}   profile={profile:<10s}  {verdict}")

        if not args.no_plots:
            plot_log(log, m, plots_full_dir / f"{log_path.stem}_series.png")
            zoom_range = find_action_window(log.states)
            if zoom_range[0] is not None and zoom_range[1] is not None:
                plot_log(
                    log, m,
                    plots_zoom_dir / f"{log_path.stem}_series.png",
                    t_range=zoom_range,
                )

    # Write CSV
    csv_path = args.out / "baseline.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for row in csv_rows:
            w.writerow(row)

    # Write JSON
    json_path = args.out / "baseline.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(json_records, f, indent=2, default=str)

    # Print summary table to stdout for the human running the command.
    # We deliberately do NOT write summary.txt — baseline.csv has the same
    # data in machine-readable form; duplicating it as text was redundant.
    print()
    print(render_summary(csv_rows))
    # README.md is intentionally NOT generated -- it's hand-maintained so the
    # user's edits don't get overwritten on every regeneration.

    if not args.no_plots and csv_rows:
        # Cross-profile bar chart only makes sense if 2+ profiles are present;
        # for single-profile batch dirs (e.g. batch_worst_case/) we'd just get
        # one bar per panel, which is useless. The top-level calm-vs-worst_case
        # comparison is generated separately by capstone.control.compare_batches.
        n_profiles = len({r["profile"] for r in csv_rows})
        if n_profiles >= 2:
            plot_comparison(csv_rows, png_root / "comparison.png")
        print(f"\nplots:")
        print(f"  PNG: {png_root}/")
        print(f"  SVG: {svg_root}/  (mirror — edit colors/labels here)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
