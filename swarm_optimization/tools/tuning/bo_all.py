"""
Run BO hyperparameter search across every controller in `bo_search.CONTROLLERS`.

Sequentially runs `tools/tuning/bo_search.run_one(...)` for each controller, writes
each result JSON to `outputs/bo/<controller>.json`, and finally writes a
combined `outputs/bo/summary.json` with each controller's best score / params.

Optuna trials are independent per controller, so running sequentially in one
process is fine (and easier to monitor than spawning subprocesses).

Usage:
    python3 tools/tuning/bo_all.py --n-trials 30
    python3 tools/tuning/bo_all.py --n-trials 30 --only stc,pso,boustrophedon
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from tools.tuning.bo_search import CONTROLLERS, run_one


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-trials", type=int, default=30,
                        help="Optuna trials per controller (default 30 ≈ 13 min/controller).")
    parser.add_argument("--only", type=str, default=None,
                        help="Comma-separated controller names to run (default: all).")
    parser.add_argument("--seed", type=int, default=42,
                        help="TPE sampler seed for reproducibility.")
    parser.add_argument("--out-dir", type=str, default="outputs/bo",
                        help="Directory to write per-controller JSON + summary.")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.only:
        names = [n.strip() for n in args.only.split(",") if n.strip()]
        unknown = set(names) - set(CONTROLLERS)
        if unknown:
            raise SystemExit(f"Unknown controllers: {unknown}. "
                             f"Available: {sorted(CONTROLLERS)}")
    else:
        names = list(CONTROLLERS.keys())

    print(f"[bo_all] controllers ({len(names)}): {names}")
    print(f"[bo_all] n_trials per controller: {args.n_trials}")
    print(f"[bo_all] writing JSONs to {out_dir}/")
    print()

    summary = {}
    total_start = time.time()
    for i, name in enumerate(names):
        print(f"[bo_all] [{i+1}/{len(names)}] {name} — starting "
              f"({args.n_trials} trials)")
        t0 = time.time()
        result = run_one(
            name,
            n_trials=args.n_trials,
            output_path=out_dir / f"{name}.json",
            seed=args.seed,
        )
        elapsed = time.time() - t0
        summary[name] = {
            "best_value": result["best_value"],
            "best_params": result["best_params"],
            "wall_seconds": elapsed,
        }
        print(f"[bo_all] [{i+1}/{len(names)}] {name} — best={result['best_value']:+.4f} "
              f"in {elapsed:.1f}s")
        # Save partial summary after each so a crash doesn't lose progress.
        with open(out_dir / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)
        print()

    total_elapsed = time.time() - total_start
    print(f"[bo_all] ALL DONE in {total_elapsed:.1f}s ({total_elapsed/60:.1f} min)")
    print()
    print("[bo_all] best scores (sorted):")
    ranked = sorted(summary.items(), key=lambda kv: -kv[1]["best_value"])
    for name, info in ranked:
        print(f"  {name:15s}  {info['best_value']:+.4f}   "
              f"(wall: {info['wall_seconds']:6.1f}s)")


if __name__ == "__main__":
    main()
