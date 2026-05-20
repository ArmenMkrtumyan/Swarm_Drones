"""Mission benchmark: lock in current ArduCopter mission performance.

Either flies a YAML mission --runs times against a live SITL and analyzes
the resulting JSONL logs, or reanalyzes existing logs in --logs DIR.
Sister tool of lab.control.benchmark_hover; same role, different stage.

Per-waypoint metrics (the headline numbers RL is graded against):
    closest_approach_m  -- min distance to WP center during the leg
    overshoot_m         -- max distance from WP after first arrival within
                           accept_radius_m (proxy for braking quality)
    settle_lag_s        -- t(MISSION_ITEM_REACHED) - t(first arrival),
                           i.e. how long it takes ArduCopter to settle
                           inside the accept radius and hold for post_yaw_settle_s
    xtrack_p95_m        -- 95th-percentile perpendicular distance from the
                           prev->this straight-line leg (path tracking)
    leg_duration_s      -- t(MISSION_ITEM_REACHED) - t(prev MISSION_ITEM_REACHED)

Mission-level metrics:
    completion          -- fraction of waypoints that fired MISSION_ITEM_REACHED
    mission_duration_s  -- mission_started -> last MISSION_ITEM_REACHED
    rtl_offset_m        -- distance from first to last GLOBAL_POSITION_INT,
                           proxy for RTL accuracy

Outputs (under reports/benchmark_mission_report/<mission_name>/):
    per_run_per_wp.csv   one row per (run, waypoint)
    per_run_summary.csv  one row per run
    per_wp_summary.csv   one row per waypoint, mean +/- std across runs
    mission_summary.json full machine-readable record
    (no summary.txt — same data lives in the per-run CSVs)
    plots/<run>_xy.png   per-run XY trajectory
    xy_overlay.png       all runs overlaid
    overshoot.png        per-WP overshoot bar (mean +/- std)
    settle_lag.png       per-WP settle-lag bar

Usage:
    # Fly + analyze 10 runs
    python -m lab.missions.benchmark_mission \\
        lab/missions/cases/square_20m.yaml --runs 10

    # Just reanalyze existing logs
    python -m lab.missions.benchmark_mission \\
        lab/missions/cases/square_20m.yaml --logs logs/mission_logs/

    # Single run + plots
    python -m lab.missions.benchmark_mission \\
        lab/missions/cases/square_20m.yaml --runs 1
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

from lab.missions.dsl import (
    HomePosition,
    Mission,
    Waypoint,
    load as load_mission,
)
from lab.control import plot_style


# Local-flat-earth approximation: same constant the DSL uses for compiling
# (north_m, east_m) waypoints to lat/lon. Inverting it here keeps the round-trip
# self-consistent.
_M_PER_DEG_LAT = 111_111.0
_DEFAULT_HOME_LAT = 40.192
_DEFAULT_HOME_LON = 44.50446


# -----------------------------------------------------------------------------
# Result dataclasses
# -----------------------------------------------------------------------------
@dataclass
class WaypointMetrics:
    seq: int
    wp_idx: int
    target_north_m: float
    target_east_m: float
    target_alt_m: float
    accept_radius_m: float
    leg_t_start: float | None
    leg_t_end: float | None
    leg_duration_s: float | None
    closest_approach_m: float | None
    overshoot_m: float | None
    settle_lag_s: float | None
    xtrack_p95_m: float | None
    reached: bool


@dataclass
class MissionRunMetrics:
    log_path: str
    mission_name: str
    n_waypoints: int
    waypoint_metrics: list[WaypointMetrics]
    mission_started_t: float | None
    last_reached_t: float | None
    mission_duration_s: float | None
    completion: float
    rtl_offset_m: float | None

    def to_dict(self) -> dict:
        return asdict(self)


# -----------------------------------------------------------------------------
# Mission JSONL loader. logs/mission_logs/*.jsonl mixes:
#   src=script  (lifecycle: script_started, mission_started, monitor_done, ...)
#   src=mavlink (every MAVLink message the runner observed)
# lab.common.logging silently ignores both, so we use a focused loader.
# -----------------------------------------------------------------------------
def _iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                # Tolerate a partial trailing line if the runner crashed.
                continue


def load_mission_log(path: Path) -> dict:
    """Pull positions, MISSION_ITEM_REACHED firings, and script events."""
    positions: list[dict] = []
    mission_reached: list[dict] = []
    events: list[dict] = []
    home_origin: tuple[float, float] | None = None
    first_global_pos: tuple[float, float, float] | None = None
    last_global_pos: tuple[float, float, float] | None = None

    for entry in _iter_jsonl(path):
        src = entry.get("src")
        t = float(entry.get("t", 0.0))

        if src == "script":
            events.append(entry)
            continue
        if src != "mavlink":
            continue

        mtype = entry.get("mavpackettype")
        if mtype == "GLOBAL_POSITION_INT":
            lat = entry["lat"] / 1e7
            lon = entry["lon"] / 1e7
            relative_alt = entry["relative_alt"] / 1000.0
            positions.append({
                "t": t,
                "lat": lat,
                "lon": lon,
                "relative_alt": relative_alt,
            })
            if first_global_pos is None:
                first_global_pos = (lat, lon, relative_alt)
            last_global_pos = (lat, lon, relative_alt)
        elif mtype == "GPS_GLOBAL_ORIGIN":
            home_origin = (entry["latitude"] / 1e7, entry["longitude"] / 1e7)
        elif mtype == "MISSION_ITEM_REACHED":
            mission_reached.append({"t": t, "seq": int(entry["seq"])})

    return {
        "positions": positions,
        "mission_reached": mission_reached,
        "events": events,
        "home_origin": home_origin,
        "first_global_pos": first_global_pos,
        "last_global_pos": last_global_pos,
    }


# -----------------------------------------------------------------------------
# Geometry helpers
# -----------------------------------------------------------------------------
def latlon_to_ne(home_lat: float, home_lon: float,
                 lat: float, lon: float) -> tuple[float, float]:
    cos_lat = math.cos(math.radians(home_lat))
    north = (lat - home_lat) * _M_PER_DEG_LAT
    east = (lon - home_lon) * _M_PER_DEG_LAT * cos_lat
    return north, east


def waypoint_target_ne(wp: Waypoint, home: HomePosition) -> tuple[float, float]:
    if wp.lat is not None and wp.lon is not None:
        return latlon_to_ne(home.lat, home.lon, wp.lat, wp.lon)
    return (wp.north_m or 0.0, wp.east_m or 0.0)


def perp_dist_to_line(px: float, py: float,
                      ax: float, ay: float,
                      bx: float, by: float) -> float:
    """Perpendicular distance from (px,py) to the infinite line through a,b."""
    abx, aby = bx - ax, by - ay
    ab_len = math.hypot(abx, aby)
    if ab_len < 1e-9:
        return math.hypot(px - ax, py - ay)
    cross = abs(abx * (py - ay) - aby * (px - ax))
    return cross / ab_len


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    if pct <= 0:
        return s[0]
    if pct >= 100:
        return s[-1]
    k = (pct / 100.0) * (len(s) - 1)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return s[int(k)]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


# -----------------------------------------------------------------------------
# Per-run analysis
# -----------------------------------------------------------------------------
def analyze_run(log_path: Path, mission: Mission, home: HomePosition) -> MissionRunMetrics:
    """Compute per-waypoint and mission-level metrics for one mission JSONL log.

    Mission item layout (from dsl.compile_to_mavlink_items):
        seq=0                  NAV_TAKEOFF                (skipped via set_current=1)
        per waypoint:          NAV_WAYPOINT
                               CONDITION_YAW    (only if post_yaw_settle_s > 0)
                               NAV_DELAY        (only if post_yaw_settle_s > 0)
        seq=last               RTL or LAND

    We don't assume seqs 1..N are the waypoints — instead we compile the
    items and read the actual NAV_WAYPOINT seqs. That keeps the analyzer
    correct whether each WP has a post-yaw-settle triple or fly-through.
    """
    from lab.missions.dsl import compile_to_mavlink_items, MAV_CMD_NAV_WAYPOINT

    raw = load_mission_log(log_path)
    positions = raw["positions"]
    mission_reached = raw["mission_reached"]

    # Convert all positions to (north, east) once.
    ne_track: list[tuple[float, float, float]] = [
        (p["t"],) + latlon_to_ne(home.lat, home.lon, p["lat"], p["lon"])
        for p in positions
    ]

    reached_by_seq: dict[int, float] = {}
    for r in mission_reached:
        # If a seq fires more than once (rare), keep the first -- that's the
        # canonical "we passed it".
        reached_by_seq.setdefault(r["seq"], r["t"])

    mission_started_t: float | None = None
    for ev in raw["events"]:
        if ev.get("event") == "mission_started":
            mission_started_t = float(ev["t"])
            break

    n_wp = len(mission.waypoints)

    # Resolve MAVLink seqs of each waypoint's NAV_WAYPOINT item from the
    # compiled mission. wp_seqs[i] is the seq for waypoint i (0-indexed).
    items = compile_to_mavlink_items(mission, home)
    wp_seqs = [it["seq"] for it in items if it["command"] == MAV_CMD_NAV_WAYPOINT]
    if len(wp_seqs) != n_wp:
        raise RuntimeError(
            f"compiled NAV_WAYPOINT count {len(wp_seqs)} != mission.waypoints {n_wp}; "
            "DSL and analyzer disagree on item layout"
        )

    wp_metrics: list[WaypointMetrics] = []
    for i, wp in enumerate(mission.waypoints):
        seq = wp_seqs[i]
        n_m, e_m = waypoint_target_ne(wp, home)
        accept = wp.accept_radius_m
        leg_t_end = reached_by_seq.get(seq)

        # Leg start = previous WP's NAV_WAYPOINT MISSION_ITEM_REACHED. For
        # WP1, takeoff (seq=0) usually doesn't fire reached because the
        # runner uses set_current=1 to skip past it, so we fall back to
        # mission_started_t.
        if i > 0 and wp_seqs[i - 1] in reached_by_seq:
            leg_t_start = reached_by_seq[wp_seqs[i - 1]]
        else:
            leg_t_start = mission_started_t

        leg_duration = (
            leg_t_end - leg_t_start
            if leg_t_start is not None and leg_t_end is not None
            else None
        )

        # Slice the position track to this leg's window. If leg_t_end is None
        # (WP never reached), use the tail of the track to still report a
        # closest_approach.
        leg_t_start_eff = leg_t_start if leg_t_start is not None else (
            ne_track[0][0] if ne_track else None
        )
        leg_t_end_eff = leg_t_end if leg_t_end is not None else (
            ne_track[-1][0] if ne_track else None
        )
        leg_track = []
        if leg_t_start_eff is not None and leg_t_end_eff is not None:
            leg_track = [
                (t, n, e) for (t, n, e) in ne_track
                if leg_t_start_eff <= t <= leg_t_end_eff
            ]

        closest = (
            min(math.hypot(n - n_m, e - e_m) for (_, n, e) in leg_track)
            if leg_track else None
        )

        first_arrival_t: float | None = None
        for t, n, e in leg_track:
            if math.hypot(n - n_m, e - e_m) <= accept:
                first_arrival_t = t
                break

        overshoot: float | None = None
        settle_lag: float | None = None
        if first_arrival_t is not None and leg_t_end is not None:
            tail = [
                math.hypot(n - n_m, e - e_m)
                for (t, n, e) in leg_track if t >= first_arrival_t
            ]
            overshoot = max(tail) if tail else None
            settle_lag = leg_t_end - first_arrival_t

        if i == 0:
            prev_n, prev_e = 0.0, 0.0
        else:
            prev_n, prev_e = waypoint_target_ne(mission.waypoints[i - 1], home)
        xtrack_dists = [
            perp_dist_to_line(n, e, prev_n, prev_e, n_m, e_m)
            for (_, n, e) in leg_track
        ]
        xtrack_p95 = percentile(xtrack_dists, 95.0) if xtrack_dists else None

        wp_metrics.append(WaypointMetrics(
            seq=seq,
            wp_idx=i,
            target_north_m=n_m,
            target_east_m=e_m,
            target_alt_m=wp.altitude_m,
            accept_radius_m=accept,
            leg_t_start=leg_t_start,
            leg_t_end=leg_t_end,
            leg_duration_s=leg_duration,
            closest_approach_m=closest,
            overshoot_m=overshoot,
            settle_lag_s=settle_lag,
            xtrack_p95_m=xtrack_p95,
            reached=leg_t_end is not None,
        ))

    completion = sum(1 for m in wp_metrics if m.reached) / max(1, n_wp)
    last_reached_t = max(
        (m.leg_t_end for m in wp_metrics if m.leg_t_end is not None),
        default=None,
    )
    mission_duration = (
        last_reached_t - mission_started_t
        if last_reached_t is not None and mission_started_t is not None
        else None
    )

    rtl_offset = None
    fp = raw["first_global_pos"]
    lp = raw["last_global_pos"]
    if fp is not None and lp is not None:
        n_first, e_first = latlon_to_ne(home.lat, home.lon, fp[0], fp[1])
        n_last, e_last = latlon_to_ne(home.lat, home.lon, lp[0], lp[1])
        rtl_offset = math.hypot(n_last - n_first, e_last - e_first)

    return MissionRunMetrics(
        log_path=str(log_path),
        mission_name=mission.name,
        n_waypoints=n_wp,
        waypoint_metrics=wp_metrics,
        mission_started_t=mission_started_t,
        last_reached_t=last_reached_t,
        mission_duration_s=mission_duration,
        completion=completion,
        rtl_offset_m=rtl_offset,
    )


# -----------------------------------------------------------------------------
# Aggregation
# -----------------------------------------------------------------------------
def aggregate_per_wp(runs: list[MissionRunMetrics], n_wp: int) -> list[dict]:
    rows = []
    for i in range(n_wp):
        bucket = {
            "closest_approach_m": [],
            "overshoot_m": [],
            "settle_lag_s": [],
            "xtrack_p95_m": [],
            "leg_duration_s": [],
        }
        n_reached = 0
        for run in runs:
            wp = run.waypoint_metrics[i]
            if wp.reached:
                n_reached += 1
            for k in bucket:
                v = getattr(wp, k)
                if v is not None:
                    bucket[k].append(v)
        row: dict = {
            "wp_idx": i,
            "seq": i + 1,
            "n_runs": len(runs),
            "n_reached": n_reached,
        }
        for k, xs in bucket.items():
            if xs:
                row[f"{k}_mean"] = round(statistics.fmean(xs), 4)
                row[f"{k}_std"] = round(
                    statistics.pstdev(xs) if len(xs) > 1 else 0.0, 4)
            else:
                row[f"{k}_mean"] = ""
                row[f"{k}_std"] = ""
        rows.append(row)
    return rows


# -----------------------------------------------------------------------------
# CSV writers
# -----------------------------------------------------------------------------
PER_RUN_PER_WP_FIELDS = [
    "log_path", "wp_idx", "seq",
    "target_north_m", "target_east_m", "target_alt_m", "accept_radius_m",
    "reached", "leg_duration_s",
    "closest_approach_m", "overshoot_m", "settle_lag_s", "xtrack_p95_m",
]

PER_RUN_FIELDS = [
    "log_path", "mission_name", "n_waypoints",
    "completion", "mission_duration_s", "rtl_offset_m",
]


def _round_or_blank(v: float | None, digits: int = 3) -> str | float:
    if v is None:
        return ""
    return round(v, digits)


def write_per_run_per_wp_csv(runs: list[MissionRunMetrics], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=PER_RUN_PER_WP_FIELDS)
        w.writeheader()
        for run in runs:
            for wp in run.waypoint_metrics:
                w.writerow({
                    "log_path": Path(run.log_path).name,
                    "wp_idx": wp.wp_idx,
                    "seq": wp.seq,
                    "target_north_m": round(wp.target_north_m, 3),
                    "target_east_m": round(wp.target_east_m, 3),
                    "target_alt_m": wp.target_alt_m,
                    "accept_radius_m": wp.accept_radius_m,
                    "reached": int(wp.reached),
                    "leg_duration_s": _round_or_blank(wp.leg_duration_s),
                    "closest_approach_m": _round_or_blank(wp.closest_approach_m),
                    "overshoot_m": _round_or_blank(wp.overshoot_m),
                    "settle_lag_s": _round_or_blank(wp.settle_lag_s),
                    "xtrack_p95_m": _round_or_blank(wp.xtrack_p95_m),
                })


def write_per_run_csv(runs: list[MissionRunMetrics], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=PER_RUN_FIELDS)
        w.writeheader()
        for run in runs:
            w.writerow({
                "log_path": Path(run.log_path).name,
                "mission_name": run.mission_name,
                "n_waypoints": run.n_waypoints,
                "completion": round(run.completion, 3),
                "mission_duration_s": _round_or_blank(run.mission_duration_s),
                "rtl_offset_m": _round_or_blank(run.rtl_offset_m),
            })


def write_per_wp_summary_csv(rows: list[dict], path: Path) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)


# -----------------------------------------------------------------------------
# Plots
# -----------------------------------------------------------------------------
def plot_xy_trajectory(
    runs: list[MissionRunMetrics],
    mission: Mission,
    home: HomePosition,
    out_path: Path,
    *,
    single_run: MissionRunMetrics | None = None,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    targets = [(0.0, 0.0)] + [
        waypoint_target_ne(wp, home) for wp in mission.waypoints
    ]
    path_n = [pt[0] for pt in targets] + [0.0]
    path_e = [pt[1] for pt in targets] + [0.0]
    ax.plot(path_e, path_n, color="#888", linestyle="--", linewidth=1.0,
            label="planned path")
    for i, (n, e) in enumerate(targets[1:], start=1):
        ax.plot(e, n, "x", color="#222", markersize=10, markeredgewidth=2)
        ax.annotate(f"WP{i}", xy=(e, n), xytext=(5, 5),
                    textcoords="offset points", fontsize=10)
        wp = mission.waypoints[i - 1]
        circle = plt.Circle((e, n), wp.accept_radius_m, color="#777",
                            fill=False, linestyle=":", linewidth=0.7)
        ax.add_artist(circle)
    ax.plot(0, 0, "s", color="#222", markersize=8)
    ax.annotate("home", xy=(0, 0), xytext=(5, -10),
                textcoords="offset points", fontsize=9)

    target_runs = [single_run] if single_run else runs
    cmap = matplotlib.colormaps.get_cmap("tab10")
    for idx, run in enumerate(target_runs):
        raw = load_mission_log(Path(run.log_path))
        ne = [
            latlon_to_ne(home.lat, home.lon, p["lat"], p["lon"])
            for p in raw["positions"]
        ]
        if not ne:
            continue
        ns = [pt[0] for pt in ne]
        es = [pt[1] for pt in ne]
        color = "#1f77b4" if single_run else cmap(idx % 10)
        ax.plot(es, ns, color=color, linewidth=0.8, alpha=0.7,
                label=Path(run.log_path).stem if not single_run else None)

    ax.set_xlabel("east (m, right of home)")
    ax.set_ylabel("north (m, forward of home)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    n_runs = len(target_runs)
    ax.set_title(f"{mission.name}: {n_runs} run{'s' if n_runs != 1 else ''}")

    fig.tight_layout()
    plot_style.savefig_dual(fig, out_path)
    plt.close(fig)


def plot_per_wp_bar(
    rows: list[dict],
    metric_mean: str,
    metric_std: str,
    title: str,
    ylabel: str,
    out_path: Path,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    means = []
    stds = []
    labels = []
    for r in rows:
        labels.append(f"WP{r['seq']}")
        m = r.get(metric_mean, "")
        s = r.get(metric_std, "")
        means.append(float(m) if m != "" else 0.0)
        stds.append(float(s) if s != "" else 0.0)

    fig, ax = plt.subplots(figsize=(7, 4))
    x = list(range(len(labels)))
    ax.bar(x, means, yerr=stds, capsize=5, color="#fdc086")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)
    for xi, mi in zip(x, means):
        ax.text(xi, mi, f"{mi:.2f}", ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    plot_style.savefig_dual(fig, out_path)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Summary text
# -----------------------------------------------------------------------------
def _fmt(v, w: int = 7, p: int = 2) -> str:
    if v is None or v == "":
        return f"{'-':<{w}}"
    if isinstance(v, str):
        try:
            v = float(v)
        except ValueError:
            return f"{v:<{w}}"
    return f"{v:<{w}.{p}f}"


def render_summary(runs: list[MissionRunMetrics],
                   per_wp_rows: list[dict]) -> str:
    if not runs:
        return "no runs"
    lines = []
    lines.append(f"Mission:    {runs[0].mission_name}")
    lines.append(f"Runs:       {len(runs)}")
    completions = [r.completion for r in runs]
    lines.append(
        f"Completion: mean {statistics.fmean(completions):.2f}  "
        f"min {min(completions):.2f}  max {max(completions):.2f}")
    durations = [r.mission_duration_s for r in runs
                 if r.mission_duration_s is not None]
    if durations:
        lines.append(
            f"Duration:   mean {statistics.fmean(durations):.1f} s  "
            f"min {min(durations):.1f}  max {max(durations):.1f}")
    rtls = [r.rtl_offset_m for r in runs if r.rtl_offset_m is not None]
    if rtls:
        lines.append(
            f"RTL offset: mean {statistics.fmean(rtls):.2f} m  "
            f"min {min(rtls):.2f}  max {max(rtls):.2f}")
    lines.append("")
    lines.append("Per-waypoint (mean +/- std across runs):")
    header = ("  WP   reached  closest_m  overshoot_m       "
              "settle_lag_s     xtrack_p95_m   leg_dur_s")
    lines.append(header)
    lines.append("-" * len(header))
    for r in per_wp_rows:
        lines.append(
            f"  WP{r['seq']}  {r['n_reached']}/{r['n_runs']}     "
            f"{_fmt(r['closest_approach_m_mean'], 8, 3)} "
            f"{_fmt(r['overshoot_m_mean'], 6, 2)}+/-{_fmt(r['overshoot_m_std'], 5, 2)}  "
            f"{_fmt(r['settle_lag_s_mean'], 6, 2)}+/-{_fmt(r['settle_lag_s_std'], 5, 2)}  "
            f"{_fmt(r['xtrack_p95_m_mean'], 6, 2)}+/-{_fmt(r['xtrack_p95_m_std'], 5, 2)}  "
            f"{_fmt(r['leg_duration_s_mean'], 6, 2)}"
        )
    return "\n".join(lines)


# -----------------------------------------------------------------------------
# Driver
# -----------------------------------------------------------------------------
def collect_logs_for_mission(folder: Path, mission_name: str) -> list[Path]:
    """Mission logs are written by runner.MissionLogger as
    mission_<stamp>_<missionname>.jsonl."""
    return sorted(folder.glob(f"mission_*_{mission_name}.jsonl"))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("mission", type=Path, help="Mission YAML file")
    p.add_argument("--runs", type=int, default=0,
                   help="Fly the mission this many times before analyzing. "
                        "If 0, analyze pre-existing logs in --logs.")
    p.add_argument("--logs", type=Path, default=None,
                   help="Directory of mission_*.jsonl files. Used as the runner's "
                        "output dir when --runs > 0, and as the input dir in "
                        "analyze-only mode. Default: <repo>/logs/mission_logs/baseline_pid")
    p.add_argument("--out", type=Path, default=None,
                   help="Output directory "
                        "(default: <repo>/reports/benchmark_mission_report/<mission_name>/)")
    p.add_argument("--home-lat", type=float, default=_DEFAULT_HOME_LAT)
    p.add_argument("--home-lon", type=float, default=_DEFAULT_HOME_LON)
    p.add_argument("--master", default="udpin:localhost:14551",
                   help="MAVLink connection string (only when --runs > 0)")
    p.add_argument("--monitor", type=float, default=300.0,
                   help="how long to watch each run (s, only when --runs > 0)")
    p.add_argument("--no-plots", action="store_true",
                   help="Skip matplotlib plots; CSV/JSON only")
    args = p.parse_args(argv)

    mission = load_mission(args.mission)
    home = HomePosition(lat=args.home_lat, lon=args.home_lon)

    repo_root = Path(__file__).resolve().parents[2]   # Swarm_Drones/
    logs_dir = args.logs or (repo_root / "logs" / "mission_logs" / "baseline_pid")
    out_dir = args.out or (repo_root / "reports" / "benchmark_mission_report" / mission.name)
    out_dir.mkdir(parents=True, exist_ok=True)

    new_log_paths: list[Path] = []
    if args.runs > 0:
        # Lazy import: lets analyze-only mode work without pymavlink installed.
        from lab.missions.runner import run_mission
        for i in range(args.runs):
            print(f"\n=== Run {i + 1}/{args.runs} ===")
            log_path = run_mission(
                mission, home,
                master_url=args.master,
                log_dir=logs_dir,
                monitor_timeout_s=args.monitor,
            )
            new_log_paths.append(log_path)

    log_paths = new_log_paths or collect_logs_for_mission(logs_dir, mission.name)
    if not log_paths:
        print(f"error: no mission logs found for '{mission.name}' in {logs_dir}",
              file=sys.stderr)
        return 2

    print(f"\nAnalyzing {len(log_paths)} run(s)...")
    runs: list[MissionRunMetrics] = []
    for path in log_paths:
        try:
            run = analyze_run(path, mission, home)
            runs.append(run)
            duration = (f"{run.mission_duration_s:.1f}s"
                        if run.mission_duration_s is not None else "-")
            print(f"  {path.name}   completion={run.completion:.2f}   "
                  f"duration={duration}")
        except Exception as e:
            print(f"  skip {path.name}: {e!r}")

    if not runs:
        print("error: no runs analyzed", file=sys.stderr)
        return 2

    per_wp_rows = aggregate_per_wp(runs, len(mission.waypoints))

    write_per_run_per_wp_csv(runs, out_dir / "per_run_per_wp.csv")
    write_per_run_csv(runs, out_dir / "per_run_summary.csv")
    write_per_wp_summary_csv(per_wp_rows, out_dir / "per_wp_summary.csv")
    (out_dir / "mission_summary.json").write_text(
        json.dumps([r.to_dict() for r in runs], indent=2, default=str),
        encoding="utf-8",
    )

    # Print summary table to stdout for the human running the command.
    # We deliberately do NOT write summary.txt — the per-run CSVs have the
    # same data in machine-readable form.
    print()
    print(render_summary(runs, per_wp_rows))

    if not args.no_plots:
        # PNG and SVG live in parallel sibling trees (png/ and svg/) so the
        # user can edit colors/labels in the SVGs without the PNGs in the way.
        png_root = out_dir / "png"
        svg_root = out_dir / "svg"
        plots_dir = png_root / "plots"
        plots_dir.mkdir(parents=True, exist_ok=True)
        for run in runs:
            plot_xy_trajectory(
                runs, mission, home,
                plots_dir / f"{Path(run.log_path).stem}_xy.png",
                single_run=run,
            )
        plot_xy_trajectory(runs, mission, home, png_root / "xy_overlay.png")
        plot_per_wp_bar(
            per_wp_rows, "overshoot_m_mean", "overshoot_m_std",
            f"{mission.name}: post-arrival overshoot per waypoint",
            "overshoot (m)", png_root / "overshoot.png",
        )
        plot_per_wp_bar(
            per_wp_rows, "settle_lag_s_mean", "settle_lag_s_std",
            f"{mission.name}: settle lag per waypoint",
            "settle lag (s)", png_root / "settle_lag.png",
        )
        print(f"\nplots:")
        print(f"  PNG: {png_root}/")
        print(f"  SVG: {svg_root}/  (mirror — edit colors/labels here)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
