"""Stage-1 hover-stability metrics + gate evaluator.

Reads a bridge JSONL flight log and computes hover-quality numbers over the
airborne portion. Compares against profile-specific gates (calm / mass /
wind / imu) and returns a pass/fail report.

Designed to run without numpy when called from the standalone PowerShell
harness; uses the stdlib `statistics` module. If numpy is installed the math
is identical.

Usage:
    python -m capstone.control.metrics LOG.jsonl --profile calm
    # exit code: 0 pass, 1 fail
    # stdout: pretty table; --json for machine-readable
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

from capstone.common.logging import FlightLog, load


# -----------------------------------------------------------------------------
# Gates per profile. Numbers come from the approved plan (Stage 1).
# -----------------------------------------------------------------------------
# Each gate maps metric_name -> (cmp, threshold). cmp is '<' or '<='.
# Every profile uses the SAME set of gates -- the calm-quality bar:
#   - alt_std        < 0.10 m
#   - pos_max N / E  < 0.05 m  (within 5 cm of home in each direction)
#   - roll_max       < 0.05 rad (~3 deg)
#   - pitch_max      < 0.05 rad
#   - gyro_rms       < 0.005 rad/s
#   - crashed        == 0
#
# Why uniform gates: the baseline's job is to show *how far short hardcoded
# PIDs fall from calm-quality hover under each disturbance*. By holding every
# profile to the same strict bar, the failure modes are visible per-profile
# (calm fails on north drift, wind5 fails on tilt + drift, imu_noise fails on
# everything, etc) and RL's target becomes unambiguous: drive every profile
# to PASS the calm gate. If RL can do that, the result is publishable.
#
# Per-profile gates can come back later (e.g. relaxing wind5 to allow ~30 cm
# drift) if the calm bar turns out to be physically impossible for some
# profile -- but right now we don't know that, and we want RL to surprise us.
CALM_GATES: dict[str, tuple[str, float]] = {
    # Gates re-baselined 2026-05-05 from a 3-run calm reference batch
    # (CALM1/CALM2/CALM3, copied into reports/benchmark_hover_report/calm_baseline_runs/).
    # Each threshold is set above what the 3 runs actually measure so calm
    # passes reliably. ANY profile that meets these is at "calm-quality hover"
    # — that's the bar RL has to clear under disturbance.
    "alt_std_m":       ("<", 0.08),    # calm 3-run max: 0.073
    "pos_rms_north_m": ("<", 0.06),    # v2 calm 10-run max: 0.0525 (post-USD-fix re-derive 2026-05-07)
    "pos_rms_east_m":  ("<", 0.06),    # v2 calm 10-run max: 0.0534 (post-USD-fix re-derive 2026-05-07)
    "roll_rms_rad":    ("<", 0.003),   # calm 3-run max: 0.0022
    "pitch_rms_rad":   ("<", 0.004),   # v2 calm 10-run max: 0.00317 (post-USD-fix re-derive 2026-05-07)
    "gyro_rms":        ("<", 0.009),   # v2 calm 10-run max: 0.00817 (post-USD-fix re-derive 2026-05-07)
    "crashed":         ("==", 0),
}

PROFILE_GATES: dict[str, dict[str, tuple[str, float]]] = {
    name: dict(CALM_GATES)
    for name in (
        "calm",
        "mass_drop_300g",
        "wind5", "wind_up3", "wind_down3",
        "imu_noise",
        "worst_case",
    )
}

# Window selection knobs.
#
# Approach:
#   1. Find the airborne portion (samples with altitude > AIRBORNE_THRESHOLD_M)
#      so we ignore the long pre-flight idle period.
#   2. The drone's *target altitude* is the median of those airborne samples
#      -- whatever the takeoff/mission script told it to hover at. We don't
#      need to read it from a log event; it's the most-visited altitude.
#   3. Hover window starts the moment altitude first reaches target -
#      HOVER_ENTRY_M (drone has arrived, within 2 cm). NO settling trim --
#      a clean RL controller might arrive without overshoot and we want
#      those samples in the score, not silently dropped.
#   4. Hover window ends at the last sample where altitude is still within
#      HOVER_EXIT_M of target (drone has not yet clearly descended). The
#      50 cm slack tolerates disturbance excursions -- e.g. updraft / downdraft
#      pushing the drone temporarily off altitude is part of the test, not a
#      reason to clip the window. There is NO upper duration cap; the
#      window runs as long as the takeoff script holds the drone in hover.
#
# Critically: NO "is the drone steady" filter. The previous version required
# |vz| < 0.3 m/s, which excluded the very samples where the controller was
# fighting back against a disturbance -- defeating the whole point of having
# disturbance profiles. The window now spans "drone reached hover altitude"
# through "drone began clear descent", and disturbance response is included
# in the metrics computed over that window.
#
# If the actual hover lasts less than MIN_USABLE_WINDOW_S, the run is
# marked unscoreable (short hover) and fails the gate explicitly.
AIRBORNE_THRESHOLD_M = 0.3        # alt > this counts as "off the ground"
HOVER_ENTRY_M = 0.05              # within 5 cm of target = "arrived at hover"
SETTLING_TRIM_S = 0.0             # no trim — RL with clean low-overshoot
                                  # arrivals gets credit for early samples.
# Fixed-duration hover window in SIM-SECONDS. Cross-run comparison is now
# apples-to-apples — every run gets graded on the first WINDOW_DURATION_S
# seconds after arrival, regardless of how long the actual hover lasted or
# how it ended.
#
# Why 10 and not 30 (matching arm_hover.HOLD_SECONDS): arm_hover holds for
# 30 WALL-clock seconds via time.time(), but Isaac+SITL run at ~40 % of
# realtime on this rig, so 30 wall-seconds yields ~12 sim-seconds of actual
# hover. The metric uses sim-time (state samples are timestamped in sim-t).
# 10 sim-s comfortably fits inside what every clean hover delivers.
#
# If Isaac throughput improves (or arm_hover is rewired to hold by sim-t
# instead of wall-t), this can be raised. For now, 10 s == "the actual
# hover phase the drone flies".
WINDOW_DURATION_S = 10.0
MIN_USABLE_WINDOW_S = 5.0
STATE_TIMESTEP_S = 0.02           # bridge logs at 50 Hz -> 0.02 s per sample
CRASH_DROP_PER_SEC_M = 5.0        # alt drops faster than this -> crash

# Legacy alias kept for backward compatibility with any external callers
# that imported the old constant.
TAKEOFF_ALT_THRESHOLD_M = AIRBORNE_THRESHOLD_M


# -----------------------------------------------------------------------------
# Data class for the report
# -----------------------------------------------------------------------------
@dataclass
class HoverMetrics:
    log_path: str
    profile: str
    duration_s: float
    n_state_samples: int
    home_lock_t: float | None
    airborne_t0: float | None
    window_t0: float | None
    window_t1: float | None
    n_window_samples: int
    target_alt_m: float | None = None  # implicit hover altitude (median of airborne samples)
    alt_mean_m: float | None = None
    alt_std_m: float | None = None
    xy_drift_m: float | None = None
    xy_std_m: float | None = None
    xy_excursion_max_m: float | None = None
    xy_recovery_s: float | None = None
    # Per-axis RMS deviation from home: sqrt(mean(x^2)). Captures typical
    # position error during hover; brief transients get diluted by the
    # steady-state samples that dominate the window. *This is the gated
    # metric.* It's also the standard tracking-error metric in the quadrotor
    # control / RL literature (Kaufmann, Hwangbo, RLDroneSim's reward, etc.).
    pos_rms_north_m: float | None = None
    pos_rms_east_m: float | None = None
    # Per-axis MAX absolute deviation. Diagnostic only -- shown alongside RMS
    # in the plot annotation but no longer gated, because a brief recovery
    # from a gust shouldn't fail an otherwise-good hover.
    pos_max_north_m: float | None = None
    pos_max_east_m: float | None = None
    gyro_rms: float | None = None
    gyro_peak: float | None = None
    # Truth-channel gyro (pre-IMU-noise). Equal to gyro_rms / gyro_peak on
    # profiles that don't inject IMU noise (calm, wind*, mass_drop_*). On
    # imu_noise / worst_case these isolate the actual attitude motion from
    # the synthetic sensor noise so cross-profile comparison isn't biased
    # by how aggressive the noise model is.
    gyro_rms_truth: float | None = None
    gyro_peak_truth: float | None = None
    roll_std: float | None = None
    pitch_std: float | None = None
    # Per-axis RMS tilt over the hover window (radians) -- gated.
    roll_rms_rad: float | None = None
    pitch_rms_rad: float | None = None
    # Per-axis MAX tilt -- diagnostic, no longer gated.
    roll_max_rad: float | None = None
    pitch_max_rad: float | None = None
    crashed: int = 0
    crash_reason: str | None = None
    gates: dict[str, tuple[str, float]] | None = None
    failures: list[str] | None = None

    @property
    def passed(self) -> bool:
        return not self.failures

    def to_dict(self) -> dict:
        d = asdict(self)
        d["passed"] = self.passed
        return d


# -----------------------------------------------------------------------------
# Window finding
# -----------------------------------------------------------------------------
def estimate_target_altitude(states: list[dict]) -> float | None:
    """Return the drone's implicit hover altitude: median of airborne samples.

    "Airborne" = altitude above AIRBORNE_THRESHOLD_M. The median is robust to
    transient excursions (e.g. a brief overshoot during takeoff) and matches
    whatever altitude the takeoff/mission script told the drone to hold.
    Returns None if the drone never got airborne.
    """
    if not states:
        return None
    airborne_alts = [
        -float(s["pos_ned"][2]) for s in states
        if -float(s["pos_ned"][2]) > AIRBORNE_THRESHOLD_M
    ]
    if not airborne_alts:
        return None
    return statistics.median(airborne_alts)


def find_hover_window(
    states: list[dict],
) -> tuple[float | None, float | None, float | None]:
    """Return (t0, t1, target_alt) for the hover window.

    Logic:
      - target_alt = median of airborne samples
      - t0 = first sample within HOVER_ENTRY_M (5 cm) of target altitude
      - t1 = t0 + WINDOW_DURATION_S, OR earlier if the drone descends below
             AIRBORNE_THRESHOLD_M (0.3 m) before then.

    Every run is graded on the SAME fixed duration after arrival, so
    cross-run comparison is fair regardless of how long the actual hover
    lasted (some runs may overshoot the script's intended duration; some
    may crash early). WINDOW_DURATION_S should match the takeoff script's
    intended hover hold time.

    Returns (None, None, None) if the drone never reached hover altitude.
    Returns (None, None, target) if the usable window is shorter than
    MIN_USABLE_WINDOW_S.
    """
    if not states:
        return None, None, None

    target = estimate_target_altitude(states)
    if target is None:
        return None, None, None

    entry_alt = target - HOVER_ENTRY_M

    t_arrived: float | None = None
    for s in states:
        alt = -float(s["pos_ned"][2])
        if alt >= entry_alt:
            t_arrived = float(s["t"])
            break
    if t_arrived is None:
        return None, None, target

    # Walk forward looking for the first descent below AIRBORNE_THRESHOLD_M
    # after arrival; that caps the window if it happens before
    # t_arrived + WINDOW_DURATION_S.
    t_descended = float(states[-1]["t"])
    for s in states:
        t = float(s["t"])
        if t <= t_arrived:
            continue
        alt = -float(s["pos_ned"][2])
        if alt < AIRBORNE_THRESHOLD_M:
            t_descended = t
            break

    t0 = t_arrived + SETTLING_TRIM_S  # SETTLING_TRIM_S = 0 by default
    t1 = min(t0 + WINDOW_DURATION_S, t_descended)
    if t1 - t0 < MIN_USABLE_WINDOW_S:
        return None, None, target
    return t0, t1, target


def detect_crash(states: list[dict]) -> tuple[bool, str | None]:
    """Return (crashed, reason). Looks for NaN/inf and runaway altitude drops."""
    if not states:
        return False, None
    last_alt = None
    last_t = None
    for s in states:
        try:
            pos = [float(x) for x in s["pos_ned"]]
            vel = [float(x) for x in s["vel_ned"]]
        except (TypeError, ValueError, KeyError):
            return True, "malformed state"
        for v in pos + vel:
            if math.isnan(v) or math.isinf(v):
                return True, f"NaN/inf in state at t={s['t']:.2f}"
        alt = -pos[2]
        t = float(s["t"])
        if last_alt is not None and last_t is not None:
            dt = t - last_t
            if dt > 0:
                drop = (last_alt - alt) / dt
                if drop > CRASH_DROP_PER_SEC_M:
                    return True, (
                        f"altitude drop {drop:.1f} m/s at t={t:.2f} "
                        f"(threshold {CRASH_DROP_PER_SEC_M} m/s)"
                    )
        last_alt, last_t = alt, t
    return False, None


# -----------------------------------------------------------------------------
# Stat helpers (stdlib only)
# -----------------------------------------------------------------------------
def _std(xs: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    return statistics.pstdev(xs)


def _rms(xs: list[float]) -> float:
    if not xs:
        return 0.0
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def _norm(v: list[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def compute_metrics(
    log: FlightLog, profile: str, *, log_path: str | None = None
) -> HoverMetrics:
    states = log.states
    duration = log.duration_s
    home_t = log.home_lock_time()

    crashed, crash_reason = detect_crash(states)

    t0, t1, target_alt = find_hover_window(states)

    metrics = HoverMetrics(
        log_path=log_path or str(log.path),
        profile=profile,
        duration_s=duration,
        n_state_samples=len(states),
        home_lock_t=home_t,
        airborne_t0=t0,
        window_t0=t0,
        window_t1=t1,
        n_window_samples=0,
        target_alt_m=target_alt,
        crashed=int(crashed),
        crash_reason=crash_reason,
    )

    if t0 is None or t1 is None:
        metrics.failures = [
            f"no hover window found (drone never reached >{AIRBORNE_THRESHOLD_M} m"
            " above home)"
        ]
        metrics.gates = PROFILE_GATES.get(profile, {})
        return metrics

    if (t1 - t0) < MIN_USABLE_WINDOW_S:
        metrics.failures = [
            f"hover window too short: {t1 - t0:.2f} s < {MIN_USABLE_WINDOW_S} s"
        ]
        metrics.gates = PROFILE_GATES.get(profile, {})
        return metrics

    win = [s for s in states if t0 <= s["t"] <= t1]
    metrics.n_window_samples = len(win)

    alts = [-float(s["pos_ned"][2]) for s in win]
    xs = [float(s["pos_ned"][0]) for s in win]
    ys = [float(s["pos_ned"][1]) for s in win]
    rolls = [float(s["rpy"][0]) for s in win]
    pitches = [float(s["rpy"][1]) for s in win]

    # Gyro magnitude per sample, then RMS and peak. The bridge optionally
    # logs `gyro_frd_truth` (pre-IMU-noise) on profiles that inject noise;
    # when absent, truth == post-noise (calm logs, older logs).
    gyro_mags = [_norm([float(g) for g in s["gyro_frd"]]) for s in win]
    gyro_mags_truth = [
        _norm([float(g) for g in s.get("gyro_frd_truth", s["gyro_frd"])])
        for s in win
    ]

    # XY drift: peak distance from window-mean center.
    if xs and ys:
        x_mean = sum(xs) / len(xs)
        y_mean = sum(ys) / len(ys)
        xy_radii = [math.hypot(x - x_mean, y - y_mean) for x, y in zip(xs, ys)]
        metrics.xy_excursion_max_m = max(xy_radii) if xy_radii else 0.0
        metrics.xy_drift_m = max(xy_radii) if xy_radii else 0.0
        metrics.xy_std_m = math.hypot(_std(xs), _std(ys))
    metrics.alt_mean_m = sum(alts) / len(alts) if alts else 0.0
    metrics.alt_std_m = _std(alts)
    # Position: RMS (gated, typical-error) and MAX (diagnostic, worst-transient).
    metrics.pos_rms_north_m = _rms(xs)
    metrics.pos_rms_east_m = _rms(ys)
    metrics.pos_max_north_m = max(abs(x) for x in xs) if xs else 0.0
    metrics.pos_max_east_m = max(abs(y) for y in ys) if ys else 0.0
    # Attitude: RMS (gated) and MAX (diagnostic).
    metrics.roll_std = _std(rolls)
    metrics.pitch_std = _std(pitches)
    metrics.roll_rms_rad = _rms(rolls)
    metrics.pitch_rms_rad = _rms(pitches)
    metrics.roll_max_rad = max(abs(r) for r in rolls) if rolls else 0.0
    metrics.pitch_max_rad = max(abs(p) for p in pitches) if pitches else 0.0
    metrics.gyro_rms = _rms(gyro_mags)
    metrics.gyro_peak = max(gyro_mags) if gyro_mags else 0.0
    metrics.gyro_rms_truth = _rms(gyro_mags_truth)
    metrics.gyro_peak_truth = max(gyro_mags_truth) if gyro_mags_truth else 0.0

    # xy_recovery_s: time to return inside ±0.3 m of window-mean after the
    # max excursion sample. Only meaningful for wind profile, but cheap to
    # compute always.
    if xs and ys:
        peak_idx = max(range(len(win)), key=lambda i: math.hypot(xs[i] - x_mean, ys[i] - y_mean))
        recovery_t = None
        for i in range(peak_idx, len(win)):
            if math.hypot(xs[i] - x_mean, ys[i] - y_mean) < 0.3:
                recovery_t = win[i]["t"] - win[peak_idx]["t"]
                break
        metrics.xy_recovery_s = recovery_t

    metrics.gates = PROFILE_GATES.get(profile, {})
    metrics.failures = _check_gates(metrics)
    return metrics


def _check_gates(m: HoverMetrics) -> list[str]:
    gates = m.gates or {}
    failures: list[str] = []
    for metric_name, (cmp, threshold) in gates.items():
        actual = getattr(m, metric_name, None)
        if actual is None:
            failures.append(f"{metric_name}: missing")
            continue
        ok = (
            (cmp == "<" and actual < threshold)
            or (cmp == "<=" and actual <= threshold)
            or (cmp == "==" and actual == threshold)
        )
        if not ok:
            failures.append(f"{metric_name}={actual:.4g} fails {cmp} {threshold}")
    return failures


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def _format_table(m: HoverMetrics) -> str:
    rows = [
        ("log", str(Path(m.log_path).name)),
        ("profile", m.profile),
        ("duration_s", f"{m.duration_s:.2f}"),
        ("n_states", str(m.n_state_samples)),
        ("home_lock_t", f"{m.home_lock_t:.2f}" if m.home_lock_t is not None else "-"),
        ("window_s", f"{m.window_t0:.2f}-{m.window_t1:.2f} ({m.n_window_samples} samples)"
            if m.window_t0 is not None else "-"),
    ]
    if m.alt_mean_m is not None:
        rows += [
            ("alt_mean_m", f"{m.alt_mean_m:.3f}"),
            ("alt_std_m", f"{m.alt_std_m:.4f}"),
            ("xy_std_m", f"{m.xy_std_m:.4f}"),
            ("xy_excursion_max_m", f"{m.xy_excursion_max_m:.3f}"),
            ("xy_recovery_s", f"{m.xy_recovery_s:.2f}" if m.xy_recovery_s is not None else "-"),
            ("gyro_rms", f"{m.gyro_rms:.4f}"),
            ("gyro_rms_truth", f"{m.gyro_rms_truth:.4f}"
                if m.gyro_rms_truth is not None else "-"),
            ("gyro_peak", f"{m.gyro_peak:.4f}"),
            ("gyro_peak_truth", f"{m.gyro_peak_truth:.4f}"
                if m.gyro_peak_truth is not None else "-"),
            ("roll_std", f"{m.roll_std:.4f}"),
            ("pitch_std", f"{m.pitch_std:.4f}"),
        ]
    rows += [("crashed", str(m.crashed))]
    if m.crash_reason:
        rows.append(("crash_reason", m.crash_reason))
    width = max(len(k) for k, _ in rows)
    lines = [f"{k:<{width}}  {v}" for k, v in rows]
    verdict = "PASS" if m.passed else "FAIL"
    lines.append("-" * (width + 2 + 8))
    lines.append(f"verdict: {verdict}")
    if m.failures:
        for fail in m.failures:
            lines.append(f"  - {fail}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("log", type=Path, help="JSONL flight log")
    p.add_argument(
        "--profile",
        default="calm",
        choices=sorted(PROFILE_GATES.keys()),
        help="disturbance profile to grade against",
    )
    p.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = p.parse_args(argv)

    log = load(args.log)
    metrics = compute_metrics(log, args.profile, log_path=str(args.log))

    if args.json:
        print(json.dumps(metrics.to_dict(), indent=2, default=str))
    else:
        print(_format_table(metrics))

    return 0 if metrics.passed else 1


if __name__ == "__main__":
    sys.exit(main())
