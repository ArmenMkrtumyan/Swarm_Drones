# RL hover benchmark — run instructions

Goal: deploy `PPO_seed2.zip` (best Phase-1 checkpoint, deterministic-eval mean
126.7 ± 1.0) against real ArduCopter SITL + Isaac bridge, for 10 calm runs and
10 worst_case runs. Then post-process with the same `benchmark_hover.py` you
use for the baseline so the RL batches are directly comparable to
`reports/benchmark_hover_report/calm_baseline_runs/` and `batch_worst_case/`.

The adapter only **tunes PIDs** during hover — it does NOT take off, change
modes, or disarm. Your existing flight pipeline still runs the takeoff and
hover. You start the adapter once the drone is stable in the air; you stop it
before landing.

## One-time setup

### 1. Add port 14552 to your SITL launch line

Your current launch (per memory) only opens 14551:

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=...sitl/params.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console \
  --out=udp:127.0.0.1:14551
```

Add a second `--out=` line so the RL adapter has its own endpoint:

```bash
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552
```

(That's the only change — everything else stays.)

### 2. Verify checkpoint exists

```powershell
Get-Item Swarm_Drones/logs/checkpoints/compare_v0/PPO_seed2.zip
```

Should show ~1.7 MB.

## Per-run procedure (one of 10)

For each run:

**Terminal A — SITL (WSL)**

```bash
cd /mnt/c/Users/user1811/Desktop/armen-capstone/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=/mnt/c/Users/user1811/Desktop/armen-capstone/Swarm_Drones/sitl/params.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552
```

**Terminal B — Isaac Sim**

Launch Isaac the way you normally do for the calm/worst_case baseline batches.
Pick the disturbance profile in the bridge GUI / config:

- For runs that go in `rl_calm_run/`: profile **`calm`**.
- For runs that go in `rl_worst_case/`: profile **`worst_case`**.

The bridge writes its flight JSONL exactly like the baseline runs.

**Important — adapter does NOT control flight.** It only sends `PARAM_SET` to
nudge ATC PIDs. Takeoff, hover hold, land are still owned by your existing
`arm_hover.py` (or whatever you launch). The adapter self-aligns: it waits
for stable hover (alt_err < 0.3 m, |vel| < 0.3 m/s) before the policy loop
starts, and auto-exits when altitude drops > 1.5 m below target (drone is
landing).

**Do NOT change `HOLD_SECONDS` — keep it at 30 (your baseline value).** The
RL benchmark must match the baseline flight duration so the comparison is
apples-to-apples. Both baselines and RL get 30 wall-sec hover (≈12 sim-sec)
and the same 10-sim-sec automatic scoring window. Only difference between
the two batches: default ATC PIDs vs RL-tuned ATC PIDs.

If RL turns out to need more sim-time to settle, that's a *finding* — not
something to paper over by extending the hover. Run a separate
"longer-hold" experiment after the apples-to-apples set if you want to know
whether more time would have helped.

Set `--duration 30` on the adapter (or higher — it auto-exits on descent
either way). If you want it to run as long as flight allows, use `--duration
120` and rely on the auto-exit.

**Per run — three-terminal flow**

Terminal C (RL adapter) starts in parallel with Terminal B (Isaac+bridge) /
Terminal A (`arm_hover.py`). The adapter waits for hover to be detected, so
order doesn't matter — you can launch the adapter even before takeoff.

```powershell
# Terminal C — RL adapter (calm run #1)
cd C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones
capstone\rl\.rl_venv\Scripts\python.exe -m lab.rl.rl_hover_adapter `
    --algo PPO `
    --checkpoint logs/checkpoints/compare_v0/PPO_seed2.zip `
    --duration 120 `
    --sidecar reports\benchmark_hover_report\rl_calm_run\rl_actions_RL1.jsonl
```

(`--duration 120` is just a generous upper bound — the adapter auto-exits when
the drone starts to descend, which `arm_hover.py` triggers at ~30 wall-sec.)

What happens, in order:

1. Adapter connects to MAVLink at `udpin:localhost:14552`, restores baseline
   ATC gains, then **waits** for stable hover.
2. You launch `arm_hover.py` (Terminal A or wherever you usually do). Drone
   takes off and climbs to 3 m.
3. Adapter sees `alt_err < 0.3 m, |vel| < 0.3 m/s` for ~1 s, locks `home_xy`,
   and starts the policy loop at 2 Hz.
4. Policy runs while `arm_hover` holds altitude (~30 wall-sec).
5. When `arm_hover` finishes its hold and switches to LAND, the drone descends.
   Adapter auto-exits the moment alt < 1.5 m → no manual stop needed.
6. Bridge writes its flight JSONL like normal. Move it into
   `reports/benchmark_hover_report/rl_calm_run/`.

**End-of-run**

`arm_hover.py` lands and disarms on its own. Save / move the bridge JSONL into
`reports/benchmark_hover_report/rl_calm_run/` (or `rl_worst_case/`). Increment the
sidecar filename `rl_actions_RL2.jsonl`, etc. for the next run.

Restart SITL + Isaac for each run, exactly as you do for the baseline batches.

## After all 20 runs

Post-process exactly like the baseline:

```powershell
cd C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones

# RL calm batch — compute the same metrics as calm_baseline_runs
capstone\rl\.rl_venv\Scripts\python.exe -m lab.control.benchmark_hover `
    reports\benchmark_hover_report\rl_calm_run `
    --out reports\benchmark_hover_report\rl_calm_run\report

# RL worst_case batch
capstone\rl\.rl_venv\Scripts\python.exe -m lab.control.benchmark_hover `
    reports\benchmark_hover_report\rl_worst_case `
    --out reports\benchmark_hover_report\rl_worst_case\report
```

This produces `report/baseline.csv`, `report/baseline.json`, and per-log time
series PNGs — same structure as the existing baseline batches.

Then compare cross-batch:

```powershell
capstone\rl\.rl_venv\Scripts\python.exe -m lab.control.compare_batches `
    --batches reports\benchmark_hover_report\calm_baseline_runs reports\benchmark_hover_report\rl_calm_run `
    --out reports\benchmark_hover_report\compare_calm.png

capstone\rl\.rl_venv\Scripts\python.exe -m lab.control.compare_batches `
    --batches reports\benchmark_hover_report\batch_worst_case reports\benchmark_hover_report\rl_worst_case `
    --out reports\benchmark_hover_report\compare_worst.png
```

## What to expect

- **Calm**: PPO_seed2 has near-perfect hover in Phase-1 (det-eval std=1.0).
  Expect calm metrics within ~10-20 % of baseline. Likely passes calm gates.
- **Worst_case**: this is where the sim-to-real gap bites. Phase-1 trained
  against an *approximate* ArduPilot mirror; the real ArduPilot's notch
  filters, anti-windup, and rate-loop quirks aren't in our mirror. The policy
  may over- or under-correct gains under aggressive disturbances.

If RL beats baseline → ship.
If RL is competitive → calm_baseline already passes 10/10 gate; the RL run
just shows the policy is non-destructive.
If RL is significantly worse on worst_case → run Phase-2 fine-tune to close
the gap. See `PHASE2_RESET_DESIGN.md` for the fine-tune harness design.

## Sanity checks if the adapter misbehaves

| Symptom | Likely cause |
|---|---|
| `no telemetry within 10 s` | Port 14552 isn't open in SITL. Add the `--out=...:14552` line. |
| Heartbeat received but `local_pos` never arrives | `SR1_POSITION` is 0 in your param file. Run `param show SR1_POSITION` in MAVProxy. Should be ≥ 5 Hz. |
| Adapter starts, gains never change | The drone's state is too far from `hover_alt_m=3.0` — policy outputs ~0 deltas because obs is out of distribution. Confirm with the sidecar JSONL. |
| Drone tips after adapter starts | Warm-start sim-to-real gap. Stop the adapter, let baseline gains hold. The policy needs Phase-2 fine-tuning. |
| `loop overran budget by 0.xxx s` warnings | Telemetry stalls. Won't break results, but reduce step interval if persistent. |

## Files in play

- `Swarm_Drones/lab/rl/rl_hover_adapter.py` — the adapter (~200 LOC).
- `Swarm_Drones/logs/checkpoints/compare_v0/PPO_seed2.zip` — the policy.
- `reports/benchmark_hover_report/rl_calm_run/` — 10 bridge JSONLs + 10 sidecars + `report/`.
- `reports/benchmark_hover_report/rl_worst_case/` — same shape, 10 of each.
- `Swarm_Drones/lab/control/benchmark_hover.py` — existing post-processor.
- `Swarm_Drones/lab/control/compare_batches.py` — existing cross-batch plot.
