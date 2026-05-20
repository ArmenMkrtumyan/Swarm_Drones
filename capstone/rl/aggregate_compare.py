"""Aggregate compare_v0 TensorBoard runs into figures + table + REPORT.md.

Reads every per-run TB event file under ``log_root``, extracts the
``rollout/ep_rew_mean`` curve, aligns curves across seeds (via 1k-step
binning), and writes:

  <log_root>/_figures/learning_curves.png       mean ± std, three algos.
  <log_root>/_figures/learning_curves.svg       editable vector.
  <log_root>/_eval_results.json                 final-eval table per (algo, seed).
  <log_root>/_eval_summary.csv                  same, flat CSV.
  <log_root>/REPORT.md                          markdown writeup.

Usage:

    capstone/rl/.rl_venv/Scripts/python.exe -m capstone.rl.aggregate_compare \\
        --config capstone/rl/cfg/compare_v0.yaml
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml


log = logging.getLogger("capstone.rl.aggregate")


ALGO_COLORS = {
    "DDPG": "#d62728",   # red
    "TD3":  "#1f77b4",   # blue
    "PPO":  "#2ca02c",   # green
    "SAC":  "#ff7f0e",   # orange
}


def _scan_runs(log_root: Path, algos: list[str], seeds: list[int]):
    """Return {(algo, seed): tb_run_dir} from any matching subfolder.

    SB3 names runs ``<tb_log_name>_<n>`` where n increments on collision. We
    pick the largest n (most recent) for each pair.
    """
    found: dict[tuple[str, int], Path] = {}
    for algo in algos:
        for seed in seeds:
            stem = f"{algo}_seed{seed}"
            candidates = sorted(
                [p for p in log_root.iterdir()
                 if p.is_dir() and p.name.startswith(stem)],
                key=lambda p: p.name,
            )
            if candidates:
                found[(algo, seed)] = candidates[-1]
    return found


def _load_scalar(run_dir: Path, tag: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (steps, values) for `tag` from the TB events in `run_dir`."""
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    ea = EventAccumulator(str(run_dir),
                          size_guidance={"scalars": 100000, "tensors": 0})
    ea.Reload()
    if tag not in ea.Tags().get("scalars", []):
        return np.array([]), np.array([])
    events = ea.Scalars(tag)
    steps = np.array([e.step for e in events], dtype=np.int64)
    vals = np.array([e.value for e in events], dtype=np.float64)
    return steps, vals


def _bin_curve(steps: np.ndarray, vals: np.ndarray,
               bin_size: int, max_step: int) -> tuple[np.ndarray, np.ndarray]:
    """Bin (steps, vals) into uniform bins of width `bin_size` up to `max_step`.

    Returns (bin_centers, mean-per-bin). NaN for empty bins.
    """
    if len(steps) == 0:
        n_bins = max_step // bin_size
        return np.arange(n_bins) * bin_size + bin_size / 2, np.full(n_bins, np.nan)
    n_bins = max_step // bin_size
    centers = np.arange(n_bins) * bin_size + bin_size / 2
    out = np.full(n_bins, np.nan)
    for i in range(n_bins):
        mask = (steps >= i * bin_size) & (steps < (i + 1) * bin_size)
        if mask.any():
            out[i] = float(np.mean(vals[mask]))
    # Forward-fill NaNs (for sparse PPO curves at start before first rollout).
    last = np.nan
    for i in range(n_bins):
        if math.isnan(out[i]):
            out[i] = last
        else:
            last = out[i]
    return centers, out


def _evaluate_policy(algo: str, ckpt_path: Path, raw_cfg: dict,
                     n_episodes: int, seed: int) -> dict:
    """Load checkpoint, run n_episodes on a fresh env, return mean / std return."""
    import math as _math
    from capstone.rl.envs.hover_pretrain_v0 import (
        DRRange, GainSpec, PretrainConfig,
    )
    from capstone.rl.envs.sb3_vec_adapter import HoverPretrainSB3VecEnv

    e = raw_cfg["env"]
    dr = raw_cfg["dr"]
    cfg = PretrainConfig(
        num_envs=1,                                # single env for eval
        device=str(e["device"]),
        hover_alt_m=float(e["hover_alt_m"]),
        max_episode_steps=int(e["max_episode_steps"]),
        inner_steps_per_action=int(e["inner_steps_per_action"]),
        physics_dt=float(e["physics_dt"]),
        crash_alt_m=float(e["crash_alt_m"]),
        crash_pos_xy_m=float(e["crash_pos_xy_m"]),
        crash_attitude_rad=_math.radians(float(e["crash_attitude_deg"])),
        survival_bonus=float(e["survival_bonus"]),
        crash_penalty=float(e["crash_penalty"]),
        reward_scale=float(e["reward_scale"]),
        base_step_reward=float(e["base_step_reward"]),
        w_alt_err=float(e["w_alt_err"]),
        w_pos_err=float(e["w_pos_err"]),
        w_vel=float(e["w_vel"]),
        w_attitude=float(e["w_attitude"]),
        w_gyro=float(e["w_gyro"]),
        w_action_smooth=float(e["w_action_smooth"]),
        dr_mass=DRRange(**dr["mass"]),
        dr_motor_tau=DRRange(**dr["motor_tau"]),
        dr_K_thrust_jitter=DRRange(**dr["K_thrust_jitter"]),
        dr_K_drag=DRRange(**dr["K_drag"]),
        dr_gyro_bias=DRRange(**dr["gyro_bias"]),
        dr_gyro_noise_sigma=DRRange(**dr["gyro_noise_sigma"]),
        dr_init_attitude_rad=DRRange(
            lo=_math.radians(float(dr["init_attitude_deg"]["lo"])),
            hi=_math.radians(float(dr["init_attitude_deg"]["hi"])),
        ),
        dr_init_alt_offset=DRRange(**dr["init_alt_offset"]),
        dr_init_xy_offset=DRRange(**dr["init_xy_offset"]),
        dr_initial_gain_mistune_pct=DRRange(**dr["initial_gain_mistune_pct"]),
        gains=[GainSpec(**g) for g in raw_cfg["gains"]],
    )

    vec_env = HoverPretrainSB3VecEnv(cfg)
    vec_env.seed(seed)

    if algo == "DDPG":
        from stable_baselines3 import DDPG as Cls
    elif algo == "TD3":
        from stable_baselines3 import TD3 as Cls
    elif algo == "PPO":
        from stable_baselines3 import PPO as Cls
    elif algo == "SAC":
        from stable_baselines3 import SAC as Cls
    else:
        raise ValueError(algo)
    model = Cls.load(str(ckpt_path), env=vec_env, device="cpu")

    returns: list[float] = []
    obs = vec_env.reset()
    cur_return = 0.0
    eps_done = 0
    while eps_done < n_episodes:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, _ = vec_env.step(action)
        cur_return += float(reward[0])
        if bool(done[0]):
            returns.append(cur_return)
            cur_return = 0.0
            eps_done += 1
    vec_env.close()
    arr = np.array(returns)
    return {
        "n_episodes": int(n_episodes),
        "mean": float(arr.mean()),
        "std": float(arr.std(ddof=0)),
        "min": float(arr.min()),
        "max": float(arr.max()),
        "all_returns": [float(x) for x in arr],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Aggregate compare_v0 results.")
    ap.add_argument("--config", required=True)
    ap.add_argument("--bin-size", type=int, default=2000,
                    help="Bin width (env steps) for learning-curve smoothing.")
    ap.add_argument("--no-eval", action="store_true",
                    help="Skip the final-policy eval pass.")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    raw = yaml.safe_load(Path(args.config).read_text())
    log_root = Path(raw["log_root"])
    fig_dir = log_root / "_figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    algos: list[str] = list(raw["algos"])
    seeds: list[int] = list(raw["seeds"])
    total_steps = int(raw["shared"]["total_timesteps"])

    # -------- Reward curves --------
    runs = _scan_runs(log_root, algos, seeds)
    log.info("found %d run dirs out of %d expected",
             len(runs), len(algos) * len(seeds))
    for k, p in runs.items():
        log.info("  %s -> %s", k, p)

    curves: dict[str, list[np.ndarray]] = defaultdict(list)
    for (algo, seed), run_dir in runs.items():
        steps, vals = _load_scalar(run_dir, "rollout/ep_rew_mean")
        if len(steps) == 0:
            log.warning("no rollout/ep_rew_mean for %s_seed%d", algo, seed)
            continue
        centers, binned = _bin_curve(steps, vals, args.bin_size, total_steps)
        curves[algo].append(binned)
    bin_centers = np.arange(total_steps // args.bin_size) * args.bin_size + args.bin_size / 2

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5))
    for algo in algos:
        if not curves.get(algo):
            continue
        stack = np.vstack(curves[algo])               # (n_seeds, n_bins)
        mean = np.nanmean(stack, axis=0)
        std = np.nanstd(stack, axis=0)
        ax.plot(bin_centers, mean, label=f"{algo} (n={len(curves[algo])})",
                color=ALGO_COLORS.get(algo, None), linewidth=2)
        ax.fill_between(bin_centers, mean - std, mean + std,
                        alpha=0.20, color=ALGO_COLORS.get(algo, None))
    ax.set_xlabel("Environment steps")
    ax.set_ylabel("Mean episode return")
    algos_str = " vs ".join(algos)
    ax.set_title(f"HoverPretrain — {algos_str}  (mean ± std across seeds)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(fig_dir / "learning_curves.png", dpi=160)
    fig.savefig(fig_dir / "learning_curves.svg")
    log.info("wrote %s.{png,svg}", fig_dir / "learning_curves")
    plt.close(fig)

    # -------- Final-policy eval --------
    # Resume-friendly: load any existing _eval_results.json so previously
    # completed runs don't have to be re-evaluated. Save after every run so a
    # mid-pass kill doesn't lose work.
    eval_results: dict[str, dict[int, dict]] = {a: {} for a in algos}
    eval_path = log_root / "_eval_results.json"
    if eval_path.exists():
        prev = json.loads(eval_path.read_text())
        for algo in algos:
            for sk, v in (prev.get(algo, {}) or {}).items():
                eval_results[algo][int(sk)] = v
        log.info("resumed eval_results from %s (%d algos, %d total)",
                 eval_path, len(prev),
                 sum(len(d) for d in eval_results.values()))

    if not args.no_eval:
        ckpt_root = Path(raw["checkpoint_root"])
        n_eval = int(raw.get("eval_episodes", 10))
        for algo in algos:
            for seed in seeds:
                if seed in eval_results[algo]:
                    log.info("skip %s_seed%d (already evaluated)", algo, seed)
                    continue
                ckpt = ckpt_root / f"{algo}_seed{seed}.zip"
                if not ckpt.exists():
                    log.warning("missing checkpoint %s", ckpt)
                    continue
                log.info("evaluating %s_seed%d (%d episodes)...", algo, seed, n_eval)
                res = _evaluate_policy(algo, ckpt, raw, n_eval, seed=seed + 1000)
                eval_results[algo][seed] = res
                log.info("  %s_seed%d  mean=%.2f  std=%.2f",
                         algo, seed, res["mean"], res["std"])
                # Persist after every run so a kill doesn't lose work.
                eval_path.write_text(json.dumps(eval_results, indent=2))

        with open(log_root / "_eval_summary.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["algo", "seed", "mean_return", "std_return", "min", "max"])
            for algo in algos:
                for seed in seeds:
                    r = eval_results[algo].get(seed)
                    if r is None:
                        continue
                    w.writerow([algo, seed, f"{r['mean']:.3f}",
                                f"{r['std']:.3f}", f"{r['min']:.3f}",
                                f"{r['max']:.3f}"])

    # -------- Markdown report --------
    md = _build_report(raw, runs, curves, eval_results, total_steps, args.bin_size,
                       fig_path="_figures/learning_curves.png")
    (log_root / "REPORT.md").write_text(md, encoding="utf-8")
    log.info("wrote %s", log_root / "REPORT.md")


def _build_report(raw, runs, curves, eval_results, total_steps, bin_size, fig_path):
    algos = list(raw["algos"])
    seeds = list(raw["seeds"])
    shared = raw["shared"]

    lines: list[str] = []
    algos_title = " vs ".join(algos)
    lines.append(f"# {algos_title} on HoverPretrain\n")
    lines.append(f"Comparison of the algorithms {', '.join(algos)} on the "
                 "HoverPretrain environment from the F450 capstone. All algorithms are "
                 "trained on the same env, with the same total timesteps, the same "
                 "network, and the same learning rate. On-policy algorithms have their "
                 "own rollout / GAE / clip hyperparameters; off-policy algorithms share "
                 "buffer / target-net / exploration knobs.\n\n")

    lines.append("## Environment\n")
    lines.append(f"- Action space: {len(raw['gains'])} attitude-loop PID gains, each in [-1, 1] mapped to a per-step delta.\n")
    lines.append(f"- Observation: 19-dim (8 normalised gains + altitude error + xy position + 3-vec velocity + roll + pitch + 3-vec gyro).\n")
    lines.append(f"- Episode: {raw['env']['max_episode_steps']} env steps × {raw['env']['inner_steps_per_action']} physics ticks at dt={raw['env']['physics_dt']}s = {raw['env']['max_episode_steps']*raw['env']['inner_steps_per_action']*raw['env']['physics_dt']:.1f}s of simulated hover.\n")
    lines.append("- Per-env domain randomisation over mass, motor τ, K_thrust, K_drag, gyro bias / noise, initial attitude / altitude / xy / gain mistune.\n")
    lines.append(f"- Reward shaping: w_alt={raw['env']['w_alt_err']}, w_attitude={raw['env']['w_attitude']}, w_gyro={raw['env']['w_gyro']}, w_vel={raw['env']['w_vel']}, w_action_smooth={raw['env']['w_action_smooth']}; survival bonus {raw['env']['survival_bonus']}, crash penalty {raw['env']['crash_penalty']}.\n\n")

    lines.append("## Experimental setup\n")
    lines.append(f"- **Total timesteps**: {total_steps:,} per (algo, seed).\n")
    lines.append(f"- **Seeds**: {seeds}.\n")
    lines.append(f"- **Algorithms**: {', '.join(algos)} from Stable-Baselines3.\n")
    lines.append(f"- **Network**: shared MLP `{shared['policy_kwargs']['net_arch']}` for actor and critic.\n")
    lines.append(f"- **Learning rate**: {shared['learning_rate']}, gamma {shared['gamma']}.\n")
    lines.append(f"- **Off-policy (DDPG, TD3)**: replay buffer {shared['buffer_size']:,}, learning_starts {shared['learning_starts']:,}, batch {shared['batch_size']}, tau {shared['tau']}, gradient_steps {shared['gradient_steps']}/env-step, NormalActionNoise σ={shared['action_noise_sigma']}.\n")
    td3 = raw['td3']
    lines.append(f"- **TD3-specific**: target_policy_noise {td3['target_policy_noise']}, target_noise_clip {td3['target_noise_clip']}, policy_delay {td3['policy_delay']}.\n")
    ppo = raw['ppo']
    lines.append(f"- **PPO-specific**: n_steps {ppo['n_steps']}, batch {ppo['batch_size']}, n_epochs {ppo['n_epochs']}, gae_lambda {ppo['gae_lambda']}, clip_range {ppo['clip_range']}, ent_coef {ppo['ent_coef']}, vf_coef {ppo['vf_coef']}.\n")
    lines.append(f"- **Logging**: TensorBoard scalars at `{raw['log_root']}/{{ALGO}}_seed{{N}}_*/`.\n\n")

    lines.append("## Learning curves\n")
    lines.append(f"![learning curves]({fig_path})\n\n")
    lines.append(f"Curves are mean ± std across seeds, smoothed with a {bin_size}-step bin.\n")
    lines.append("`rollout/ep_rew_mean` is a sliding 256-episode mean of episode returns, recomputed inside `EpisodeRewardCallback` because our torch-batched VecEnv isn't Monitor-wrapped.\n\n")

    if eval_results and any(d for d in eval_results.values()):
        lines.append("## Final-policy evaluation\n")
        lines.append("Each trained policy is evaluated **deterministic** on a fresh env (different seed) for ")
        lines.append(f"{raw.get('eval_episodes', 10)} episodes. Reported as mean ± std return.\n\n")
        lines.append("| Algorithm | Seed | Mean return | Std | Min | Max |\n")
        lines.append("|---|---|---:|---:|---:|---:|\n")
        for algo in algos:
            for seed in seeds:
                r = eval_results.get(algo, {}).get(seed)
                if r is None:
                    continue
                lines.append(f"| {algo} | {seed} | {r['mean']:.2f} | {r['std']:.2f} | {r['min']:.2f} | {r['max']:.2f} |\n")
        lines.append("\n### Per-algorithm summary (across seeds)\n")
        lines.append("| Algorithm | Mean of seed-means | Std across seeds | Best seed | Worst seed |\n")
        lines.append("|---|---:|---:|---:|---:|\n")
        for algo in algos:
            seed_means = [r["mean"] for r in eval_results.get(algo, {}).values()]
            if not seed_means:
                continue
            arr = np.array(seed_means)
            lines.append(f"| {algo} | {arr.mean():.2f} | {arr.std(ddof=0):.2f} | "
                         f"{arr.max():.2f} | {arr.min():.2f} |\n")
        lines.append("\n")

    lines.append("## Reproducing\n")
    lines.append("```bash\n")
    lines.append("# train (parallel; 15 runs across 5 seeds × 3 algos)\n")
    lines.append("capstone/rl/.rl_venv/Scripts/python.exe -m capstone.rl.launch_compare \\\n")
    lines.append("    --config capstone/rl/cfg/compare_v0.yaml --max-parallel 6\n\n")
    lines.append("# aggregate\n")
    lines.append("capstone/rl/.rl_venv/Scripts/python.exe -m capstone.rl.aggregate_compare \\\n")
    lines.append("    --config capstone/rl/cfg/compare_v0.yaml\n\n")
    lines.append("# inspect curves\n")
    lines.append("capstone/rl/.rl_venv/Scripts/python.exe -m tensorboard.main --logdir logs/tb_logs/compare_v0\n")
    lines.append("```\n")

    return "".join(lines)


if __name__ == "__main__":
    main()
