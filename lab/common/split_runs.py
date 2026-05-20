"""Split a multi-run bridge log into one JSONL per takeoff-land cycle.

The bridge writes ONE jsonl per Isaac session. Auto-reset on disarm lets a
single session contain N batch runs. This tool slices that file into N
single-run files so `benchmark_hover` can grade each run independently.

Run boundaries are the `disturbance_reset_on_disarm` events emitted by the
bridge. Each output file gets the setup preamble (bridge_setup_started,
log_schema rows, motor_model_calibrated, capstone_disturbance_active,
home_locked, first_sitl_packet) copied at the top, then the state and PWM
samples that fall inside that run's time window.

Usage:
    python -m lab.common.split_runs logs/flight_logs/flight_20260505_170533.jsonl
    python -m lab.common.split_runs flight.jsonl --out runs_dir/
    python -m lab.common.split_runs flight.jsonl --dry-run
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lab.common.logging import iter_jsonl


# Setup events that must appear at the top of every sub-log so analyzers can
# find calibration / disturbance config / home anchor regardless of which run
# they're looking at.
SETUP_EVENTS = {
    "bridge_setup_started",
    "log_schema",
    "motor_model_calibrated",
    "capstone_disturbance_active",
    "home_locked",
    "first_sitl_packet",
}
# The reset event marks a run boundary. Excluded from any sub-log because a
# single-run log shouldn't contain a "this is where the previous run ended"
# marker -- it would confuse analyzers that count drops/resets.
SPLIT_EVENT = "disturbance_reset_on_disarm"


def find_run_windows(entries: list[dict]) -> list[tuple[float, float]]:
    """Return [(t_start, t_end), ...] -- one (start, end) per run.

    Run 1 starts at t=0 and ends at the first disturbance_reset_on_disarm.
    Run 2 starts at that reset's t and ends at the next reset.
    The final run ends at the last entry's t (run may still be in progress).
    """
    reset_ts = [
        float(e["t"]) for e in entries
        if e.get("src") == "bridge" and e.get("event") == SPLIT_EVENT
    ]
    if not entries:
        return []
    last_t = float(entries[-1]["t"])
    boundaries = [0.0] + reset_ts + [last_t]
    windows = []
    for i in range(len(boundaries) - 1):
        windows.append((boundaries[i], boundaries[i + 1]))
    return windows


def collect_setup_preamble(entries: list[dict]) -> list[dict]:
    """Return ordered list of setup events to copy into every sub-log."""
    return [
        e for e in entries
        if e.get("src") == "bridge" and e.get("event") in SETUP_EVENTS
    ]


def slice_run(
    entries: list[dict],
    t0: float,
    t1: float,
    preamble: list[dict],
) -> list[dict]:
    """Build a single-run log: preamble + entries with t0 < t <= t1.

    Strict `<` on t0 so a reset event itself doesn't bleed into the next
    run. The reset event is excluded entirely (see SPLIT_EVENT comment).
    Setup events are excluded from the body since they're already in the
    preamble.
    """
    out = list(preamble)
    for e in entries:
        t = float(e.get("t", 0.0))
        # First run: include t==0 events not in setup. Later runs: exclude
        # the reset event itself (its t == t0 for runs 2..N).
        if t0 == 0.0:
            in_window = t <= t1
        else:
            in_window = t0 < t <= t1
        if not in_window:
            continue
        if e.get("src") == "bridge":
            ev_name = e.get("event")
            if ev_name in SETUP_EVENTS or ev_name == SPLIT_EVENT:
                continue
        out.append(e)
    return out


def write_jsonl(entries: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for e in entries:
            f.write(json.dumps(e, separators=(",", ":")))
            f.write("\n")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("log", type=Path, help="multi-run JSONL flight log")
    p.add_argument(
        "--out", type=Path, default=None,
        help="output directory (default: same dir as input)",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="report run boundaries + planned filenames; write nothing",
    )
    args = p.parse_args(argv)

    if not args.log.is_file():
        print(f"error: {args.log} is not a file", file=sys.stderr)
        return 2

    entries = list(iter_jsonl(args.log))
    windows = find_run_windows(entries)
    preamble = collect_setup_preamble(entries)

    if not windows:
        print("error: empty log, nothing to split", file=sys.stderr)
        return 2

    print(f"Input: {args.log.name}")
    print(f"  entries: {len(entries)}  preamble events: {len(preamble)}  runs: {len(windows)}")

    out_dir = args.out or args.log.parent
    stem = args.log.stem
    width = max(2, len(str(len(windows))))

    for i, (t0, t1) in enumerate(windows, start=1):
        run_label = f"run{i:0{width}d}"
        run_path = out_dir / f"{stem}_{run_label}.jsonl"
        sub = slice_run(entries, t0, t1, preamble)
        n_states = sum(1 for e in sub if e.get("src") == "isaac->sitl")
        n_pkts = sum(1 for e in sub if e.get("src") == "sitl->isaac")
        print(f"  {run_label}  t=[{t0:7.2f}, {t1:7.2f}]  "
              f"states={n_states}  pwm={n_pkts}  -> {run_path.name}")
        if not args.dry_run:
            write_jsonl(sub, run_path)

    if args.dry_run:
        print("(dry-run: nothing written)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
