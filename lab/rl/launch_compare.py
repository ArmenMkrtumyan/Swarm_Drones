"""Parallel launcher for the compare_v0 experiment.

Spawns one subprocess per (algo, seed) pair, capturing stdout to per-run log
files. Caps concurrency; prints a live progress line. Returns nonzero exit if
any run failed.

Usage:

    lab/rl/.rl_venv/Scripts/python.exe -m lab.rl.launch_compare \\
        --config lab/rl/cfg/compare_v0.yaml \\
        --max-parallel 6
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml


def main() -> int:
    ap = argparse.ArgumentParser(description="Parallel runner for compare_v0.")
    ap.add_argument("--config", required=True)
    ap.add_argument(
        "--max-parallel", type=int, default=6,
        help="Max concurrent subprocesses (default 6 — fits comfortably on a 24-core / 32GB+ VRAM box).",
    )
    ap.add_argument("--total-timesteps", type=int, default=None)
    ap.add_argument("--algos", default=None,
                    help="Comma-separated algo subset, e.g. 'DDPG,TD3'.")
    ap.add_argument("--seeds", default=None,
                    help="Comma-separated seed subset, e.g. '0,1,2'.")
    args = ap.parse_args()

    raw = yaml.safe_load(Path(args.config).read_text())
    algos = [a.strip() for a in (args.algos.split(",") if args.algos else raw["algos"])]
    seeds = [int(s) for s in (args.seeds.split(",") if args.seeds else raw["seeds"])]
    # Interleave by seed so the first batch of concurrent runs covers all algos
    # — keeps GPU/CPU load balanced (DDPG/TD3 are GPU-bound, PPO is CPU-bound).
    pairs = [(a, s) for s in seeds for a in algos]

    log_dir = Path(raw["log_root"]) / "_subprocess_logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    py = sys.executable
    cfg_arg = ["--config", args.config]
    if args.total_timesteps is not None:
        cfg_arg += ["--total-timesteps", str(args.total_timesteps)]

    print(f"[launch] {len(pairs)} runs, max_parallel={args.max_parallel}")
    print(f"[launch] python={py}")
    print(f"[launch] subprocess logs -> {log_dir}")

    # State per run.
    pending: list[tuple[str, int]] = list(pairs)
    running: dict[subprocess.Popen, tuple[str, int, float, Path]] = {}
    done: list[tuple[str, int, int, float]] = []   # (algo, seed, returncode, elapsed_s)
    started = time.time()

    while pending or running:
        # Fill up to max_parallel.
        while pending and len(running) < args.max_parallel:
            algo, seed = pending.pop(0)
            log_path = log_dir / f"{algo}_seed{seed}.log"
            cmd = [py, "-m", "lab.rl.train_compare",
                   *cfg_arg, "--only", f"{algo}:{seed}"]
            f = open(log_path, "w", encoding="utf-8")
            # PYTHONUNBUFFERED so the log streams as it runs.
            env = dict(os.environ)
            env.setdefault("PYTHONUNBUFFERED", "1")
            p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env)
            running[p] = (algo, seed, time.time(), log_path)
            print(f"[launch] +start {algo}_seed{seed} (pid={p.pid})  log={log_path}")

        # Reap finished.
        finished = []
        for p, (algo, seed, t0, log_path) in running.items():
            rc = p.poll()
            if rc is not None:
                el = time.time() - t0
                finished.append(p)
                done.append((algo, seed, rc, el))
                tag = "OK " if rc == 0 else f"FAIL({rc})"
                print(f"[launch] -done  {algo}_seed{seed}  {tag}  elapsed={el:6.1f}s "
                      f"(pending={len(pending)}, running={len(running)-1})")
        for p in finished:
            running.pop(p)

        if running and not finished:
            time.sleep(2.0)

    total = time.time() - started
    failed = [d for d in done if d[2] != 0]
    print(f"\n[launch] all done in {total:.1f}s  ({len(done)} runs, "
          f"{len(failed)} failed)")
    for algo, seed, rc, el in failed:
        print(f"[launch]   FAILED  {algo}_seed{seed}  rc={rc}  elapsed={el:.1f}s")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
