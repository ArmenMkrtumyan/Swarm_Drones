# Stage 3 — Plan C: hybrid GPU pre-train + SITL fine-tune

Living design doc for the Stage-3 RL hover task. Updated as the implementation
progresses; if a section here disagrees with the code, the code is wrong (or
this doc is stale — fix one of them).

---

## 1. Goal

Train a supervisory PID-gain-tuning policy for ArduPilot ATC + PSC that:

1. **Uses the GPU productively** during training (RTX 5090 actually does work,
   not 1% utilization while real-time SITL ticks).
2. **Transfers cleanly to ArduPilot** so the same firmware that trained the
   policy is what runs on the real F450 — no re-tuning when we move to
   hardware.
3. **Beats RLDroneSim** (Ghazaryan et al., AUA, IEEE RA-L 2026) on the same
   hover task by being dramatically more sample-efficient: train the bulk of
   the policy in minutes on torch, not days on real-time SITL.
4. Provides a published-research-quality contribution: a hybrid training
   architecture that bridges GPU-vectorized motor-level training and real-time
   autopilot-supervisory training. No existing reference does this cleanly.

Stages 4 (RL missions) and 5 (RL swarm) keep ArduPilot in the loop; Plan C's
architecture transfers to those stages directly.

---

## 1.5 Alternatives we rejected (and why)

Documented here so a future-us, a reviewer, or a teammate doesn't repeat the
analysis or quietly assume one of these is "obviously" better.

### Plan A — single SITL only (RLDroneSim's path)

What it is: one ArduPilot SITL + one Isaac Sim instance, real-time, SB3
TD3/PPO via Gym wrapper. Exactly what the AUA paper publishes.

| Resource | Cost / value |
|---|---|
| Setup time | ~done already (we built it pre-Plan C) |
| Throughput | ~1 step/sec wall clock |
| 50 k steps | ~14 h wall clock |
| GPU utilization | <1% (RTX 5090 idle) |
| Sim-to-real fidelity | High (real ArduPilot + real Isaac physics) |
| Novelty | None — duplicates AUA paper |

**Why rejected**: matches RLDroneSim too closely. User goal is to *beat*
that paper. Also wastes the 5090.

### Plan B — N parallel SITL instances + SB3 SubprocVecEnv

What it would be: spawn N SITL processes (`sim_vehicle.py -n N`), N Isaac
Sim instances, N MAVLink ports, wrap each in a Gym env, batch through
`SubprocVecEnv`. Reference template:
[JacopoPan/aerial-autonomy-stack](https://github.com/JacopoPan/aerial-autonomy-stack)
(reports 10× FTRT per-instance with PX4+Gazebo headless, Linux-Docker).

Per-instance overhead (rough, on Windows):

| Component | RAM | VRAM | CPU | Disk |
|---|---|---|---|---|
| ArduPilot SITL | ~80 MB | 0 | 1 core (single-thread) | tens of MB |
| Isaac Sim instance | ~2 GB | ~3 GB | ~1.5 cores | ~6 GB |
| MAVProxy + map + console | ~150 MB | 0 | <0.1 core | small |
| Bridge Python process | ~200 MB | 0 | <0.1 core | small |
| **Per instance total** | **~2.4 GB** | **~3 GB** | **~3 cores** | (cached) |

For N = 4 (conservative target): 10 GB RAM + 12 GB VRAM + 12 cores. The 5090
has 32 GB VRAM, your typical workstation has 32-64 GB RAM. **Hardware-feasible
in principle.** The reasons we still rejected it:

1. **Isaac Sim is not friendly to N concurrent app instances on Windows.**
   USD context locking, GPU compositor contention, and OmniHydra renderers
   all assume single-process exclusivity. Multi-instance works on
   headless-Linux-with-Docker but is an engineering project on Windows.
2. **Reaching 4× throughput still leaves us at ~4 steps/sec.** That's a
   25 k-step run in ~2 h — better than Plan A (14 h) but nowhere near what
   Phase 1 of Plan C (~10 k env-steps/sec) gets you on the same hardware.
3. **No novelty.** Plan B is ~"RLDroneSim, but we engineered the parallel
   wrapper they didn't bother to publish." Capstone-quality but not
   paper-quality.
4. **Plan B and Plan C share most of the SITL infrastructure.** Plan B is a
   subset of Plan C's Phase 2. So we can always *fall back* to Plan B if
   Plan C's Phase 1 transfer doesn't work — no architecture lock-in.
5. **MAVLink coexistence on Windows: untested at N=4.** The `--out=` fan-out
   pattern works for a couple of clients (we already use 14551 + 14552); at
   N=4 we'd need 4× SITL each binding their own MAVLink, plus 4× FDM
   sockets to Isaac. Likely fine, but more debugging surface.

### Plan D — drop ArduPilot entirely, full motor-level RL on IsaacLab

What it would be: fork IsaacLab's stock quadcopter PPO env, retune to
F450, train end-to-end thrust-control policy at 100 k+ env-steps/sec on the
5090. OmniDrones / Aerial Gym already publish this approach.

**Why rejected**:

1. Walks back the Flavor-A decision in `project_rl_flavor_a_decision.md` —
   the PID-supervisory architecture is what makes Stages 4-5 (missions,
   swarm) feasible without re-implementing WP/RTL/failsafe/geofence.
2. End-to-end thrust RL is a crowded field (NTNU / ETH / UZH all ship to
   real Crazyflies + F450s). We'd compete against well-resourced labs on
   their home turf.
3. Requires retuning ArduPilot ATC after deployment, since the trained
   policy replaces ATC rather than modulating it. That's two sim-to-real
   gaps stacked, not one.
4. **Stages 4-5 need ArduPilot's mature mission stack.** Plan D leaves
   nothing useful for them.

### Why Plan C beats both

Plan C inherits the strengths of each: GPU-vectorized training (Plan D's
strength) + autopilot-supervisory architecture (Plan A/B's strength) +
straight-line transfer to the existing SITL+Isaac eval harness (free with
Plan A/B). The 250-line torch ArduPilot mirror is the cost; the published
research gap is the upside (no public reference does this hybrid cleanly).

If Phase 1 → Phase 2 transfer fails for any reason, **we fall back to Plan
B**: keep the same Phase 2 harness, abandon Phase 1, accept SITL-real-time
training. The Plan C investment is not destroyed — just the Phase 1 part
becomes an unused appendix until we figure out the gap.

---

## 2. Architecture at a glance

| Phase | Where it runs | What's in the loop | Steps/sec | What it learns |
|---|---|---|---|---|
| **Phase 1 — Pre-train** | GPU (torch tensors) | F450 motor + body dynamics + approximate cascaded PID | ~10⁵ | Generic gain-tuning rules, robust to controller variation |
| **Phase 2 — Fine-tune** | Isaac Sim + ArduPilot SITL + bridge | Real ArduPilot ATC + PSC, real EKF, MAVLink | ~1 | The last-mile adjustments specific to ArduPilot's exact behavior |

The same policy weights move between phases — same observation space, same
action space.

---

## 2.5 Where Isaac Sim and IsaacLab fit in (and don't)

This question keeps coming up — easy to get confused since the project has
both vendored. Reality check:

### Isaac Sim — the application

**Used in Phase 2 only**, and only as the high-fidelity physics + rendering
backend. The pipeline is unchanged from your existing Stage-1 / Stage-2
hover/mission benchmarks:

```
ArduPilot SITL ↔ JSON FDM ↔ Isaac Sim (PhysX + USD + cameras)
                       ↑
                   (RL Gym env reads MAVLink telemetry,
                    writes PARAM_SET deltas — does NOT
                    touch Isaac directly)
```

Isaac Sim's role:
- F450 articulation physics (motor links, body, cameras, IMU sensors).
- Stereo camera rendering for any future vision tasks.
- The AUA scene + terrain so missions and obstacle interaction look real.
- Visualization while training (you can watch the drone learn hover live).

**Not used in Phase 1.** Phase 1's torch dynamics is intentionally a
*simpler, faster* substitute. We're trading Isaac PhysX's accuracy for raw
throughput, then closing the gap with Phase 2 fine-tuning. Adding Isaac to
Phase 1 would defeat its purpose — we'd be back to single-instance throughput.

### IsaacLab — the framework

**Not used in Plan C at all.** IsaacLab is NVIDIA's vectorized-RL framework
on top of Isaac Sim. It's designed for "10000 envs running on GPU memory,
all stepped in parallel." Two reasons it's not the right fit here:

1. **Phase 2's bottleneck is real-time SITL, not physics.** IsaacLab
   parallelizing Isaac PhysX doesn't help when ArduPilot SITL is what's
   pacing the loop.
2. **Phase 1's bottleneck is *our* dynamics throughput, not Isaac's.**
   Plain torch with our 250-line dynamics is faster than IsaacLab's
   articulation system because we discarded the parts that don't matter for
   hover (sensor fusion math, contact dynamics, render passes).

When IsaacLab *will* matter:
- **Stage 4 (RL missions)**: a single drone navigating obstacles benefits
  from Isaac PhysX collision handling. IsaacLab's vectorization could
  parallelize *evaluation* (e.g., test the policy on 100 mission variants
  simultaneously). Training likely still goes through SITL real-time.
- **Stage 5 (RL swarm)**: many drones in one Isaac Sim scene; IsaacLab's
  multi-agent infrastructure (MAPPO + CTDE) is mature. ArduPilot SITL
  parallelism scales worse than GPU vectorization here, so the Stage-5
  training architecture might tilt toward IsaacLab + per-drone-policy +
  approximate dynamics, with SITL kept for the eval harness only.

Decision recap for now: **Phase 1 = pure torch, Phase 2 = real Isaac Sim +
real ArduPilot. IsaacLab stays vendored for Stage 5.**

---

## 3. The action / observation contract (frozen across both phases)

The whole architecture rests on this contract being identical in Phase 1 and
Phase 2. If we change it in either phase, weight transfer breaks.

**Action (8 dims, `Box(-1, +1)`)** — gain deltas, scaled to per-gain
`delta_per_step` (defined in `cfg/hover_v0.yaml`):

| idx | Gain | Baseline | Delta per step |
|---|---|---|---|
| 0 | `ATC_ANG_RLL_P` | 4.5 | 0.20 |
| 1 | `ATC_ANG_PIT_P` | 4.5 | 0.20 |
| 2 | `ATC_RAT_RLL_P` | 0.135 | 0.010 |
| 3 | `ATC_RAT_PIT_P` | 0.135 | 0.010 |
| 4 | `ATC_RAT_RLL_I` | 0.135 | 0.010 |
| 5 | `ATC_RAT_PIT_I` | 0.135 | 0.010 |
| 6 | `ATC_RAT_RLL_D` | 0.0036 | 0.0010 |
| 7 | `ATC_RAT_PIT_D` | 0.0036 | 0.0010 |

Yaw and PSC gains are intentionally fixed in v0. They become extension points
for v1+.

**Observation (19 dims, `Box(-inf, +inf)`)**:

```
[0..7]   8 normalized gains in [0, 1]:    (gain[i] − lo[i]) / (hi[i] − lo[i])
[8]      altitude error          (target_alt - current_alt) m
[9..10]  position error NE       m
[11..13] velocity NED            m/s
[14..15] roll, pitch             rad
[16..18] body angular rates      rad/s
```

Frame conventions match the bridge: NED for position/velocity, body FRD for
gyros (consistent with what ArduPilot reports over MAVLink).

---

## 4. Phase 1 in detail — pure torch, no Isaac, no SITL

### 4.1 What runs per env step

```text
state_t = (pos_w, vel_w, attitude_quat, ang_vel_b, motor_omega)   ← tensor on GPU, batched [B, ...]
                                ↓
[PSC alt]    thrust_demand  = K_p_pos·alt_err + K_d_pos·vel_z              (Phase-1 hover only — full PSC for missions later)
[ATC angle]  rate_setpoint  = K_p_ang·attitude_err                          (P-only outer)
[ATC rate]   torque_demand  = K_p_rat·rate_err + K_i_rat·∫err + K_d_rat·d/dt (P + I + D + clamp)
[Mixer]      pwm[4]         = mix_F450_X(thrust, roll_τ, pitch_τ, yaw_τ)    (X-config geometry from bridge)
[Motor]      omega          = first_order_lag(omega, pwm_to_omega(pwm), τ_motor, dt)
             thrust_per     = K_T · omega²
             yaw_torque_per = K_Q · omega² · spin_dir
[Aero]       drag           = -K_drag · v · |v|
             ground_effect  = factor(z_AGL)
[Body]       F_b            = sum_thrust + drag + R_bw·gravity_w
             τ_b            = motor_torques_b + cross-product terms
             state_{t+1}    = integrate(state_t, F_b, τ_b, dt)
                                ↓
reward, done = compute(state_{t+1}, action)
```

All operations are tensor ops over the leading batch dim. No Python `for` loops
over envs. ~220 lines of torch.

### 4.2 What we re-implement (the ~250 lines)

| Component | Lines | Source | Notes |
|---|---|---|---|
| Altitude PID | 30 | textbook + ArduCopter ALT-HOLD math | hover only in v0 |
| Angle PID (ATC outer) | 30 | textbook | P-only matches ArduCopter ATC outer-loop default |
| Rate PID (ATC inner) | 50 | textbook + integrator clamp | the gain-tuning target |
| F450 X-mixer | 20 | bridge `MOTOR_SPIN_DIR`, `MOTOR_POS_REL_BASE` | already exists, re-vectorize |
| Motor model + lag | 30 | bridge `Nvidia_SITL_connecter.py` lines ~270-400 | port to torch |
| Body dynamics | 30 | textbook quadrotor (Mahony / Hwangbo) | quaternion attitude integration |
| Aero (drag + ground effect) | 30 | bridge `K_DRAG`, `GROUND_EFFECT_*` | port to torch |
| **Total** | **~220** | | |

### 4.3 What we DON'T re-implement

The other ~95% of ArduPilot. We deliberately skip:

- **EKF** — Phase 1 uses ground-truth state; no Kalman filter needed.
- **Sensor models with realistic noise** — Phase 1 adds Gaussian-noise DR
  (cheap), not full bias-walk + scale-factor + temperature.
- **Notch / low-pass filters on the gyro / D-term** — single time constant.
- **Mode logic, failsafes, mission state machines** — irrelevant for hover.
- **Battery, voltage sag, motor temperature** — DR covers a thrust scaling
  factor instead.
- **MAVLink, params, scheduler, Lua, logging** — not in the loop.
- **IMU pre-processing, gyro bias estimation** — DR covers a bias term.

These are exactly the parts that make ArduPilot a *real autopilot* but don't
change what optimal gains look like. They're handled by Phase 2 instead.

### 4.4 Domain randomization (CRITICAL — bridges the sim-to-sim gap)

Per env, on `reset()`, sample:

| Parameter | Range | Why |
|---|---|---|
| Total mass | 1.1 - 1.6 kg (nominal 1.365) | F450 build variance + payload |
| Motor time constant τ | 0.03 - 0.08 s (nominal 0.05) | A2212 ESC variance |
| K_thrust | ±15% of nominal | prop variance |
| K_drag | 0.020 - 0.040 (nominal 0.028) | airframe variance |
| Gyro bias | ±0.03 rad/s constant per env | IMU calibration drift |
| Gyro noise σ | 0 - 0.05 rad/s | environment-dependent |
| Init attitude | ±5° roll/pitch perturbation | can't always start level |
| Init position | ±0.5 m XY, ±0.3 m alt | EKF position error after takeoff |
| External wind | 0 - 5 m/s OU process | matches your existing wind disturbance |
| Initial gain mistuning | ±20% from baseline per gain | so the policy learns to recover from bad starts, not just maintain good ones |

DR is what makes Phase 1 → Phase 2 transfer work. The policy never sees the
*exact* parameters of any deployment, so it learns a robust strategy rather
than overfitting to torch-mirror quirks.

---

## 5. Phase 2 in detail — what we already have

What we built before this doc:

- `envs/hover_pid_tuner_v0.py` — Gymnasium env, MAVLink to SITL on UDP 14552,
  same 19-dim obs + 8-dim action.
- `cfg/hover_v0.yaml` — config (gains, weights, hyperparams).
- `train_hover.py` — SB3 launcher.
- `.rl_venv/` — Python 3.10 venv with SB3 2.8 + gymnasium + pymavlink + torch.

What changes for Phase 2:

- `train_hover.py` learns to **load Phase 1 weights** as initialization
  (`--resume <phase1_final.zip>`).
- Smaller `total_timesteps` (~10-50k) since most learning happened in Phase 1.
- Possibly lower `learning_rate` and smaller `action_noise.sigma` for stable
  fine-tuning.

Phase 2's role in the overall pipeline: the policy was trained on an
approximate ArduPilot-ish cascaded PID. Real ArduPilot has notch filters,
specific anti-windup logic, and timing details our mirror doesn't. Phase 2
fine-tunes the policy on *actual ArduPilot* until its behavior on real SITL
matches what was learned in torch. Most of the policy doesn't change; the
last layer adapts at the margins.

---

## 6. Why weight transfer works

The action and observation spec is **identical** in both phases. The policy's
input is "what state am I in, what gains am I currently using" and its output
is "how should the gains change." That mapping is a property of *the
controller-tuning task*, not of which controller is running.

A policy that learned "if gyro is too high after a step in attitude target,
reduce rate-P by 0.005" learns a generic PID-tuning heuristic. The same
heuristic works in Phase 1's torch PID and Phase 2's ArduPilot PID, because
both controllers respond to "rate-P too high → oscillation" the same way.

The differences (notch filters, exact integrator math, sample-rate effects)
mean the *magnitude* and *timing* of the optimal correction differs slightly
between Phase 1 and Phase 2. Phase 2 fine-tuning closes that gap.

---

## 7. Throughput estimates (back-of-envelope)

Phase 1 on RTX 5090, 10000 envs, 200 Hz physics, 4 Hz policy:

```
env-step throughput  ≈ 10000 envs × 4 Hz policy = 40000 policy steps/sec
training run         ≈ 1M policy steps in 25 sec on raw env throughput
                       (real wall time will be SB3 update bound, not env bound)
estimated wall time  ≈ 1-3 hours for first reasonable hover policy
```

Phase 2 on real-time SITL:

```
step-per-second       ≈ 2 (with step_interval_s = 0.5)
50k fine-tune steps   ≈ 7 hours wall clock
```

vs. RLDroneSim baseline (50k steps, all on SITL real-time): ~14-50 hours
depending on episode length and reset cost. Plan C is 2-7× faster total wall
clock and uses the GPU instead of leaving it idle.

---

## 8. Risk register

| Risk | Mitigation |
|---|---|
| Phase 1 mirror behaves too differently from ArduPilot → bad transfer | DR over a wide range; tune mirror's filter constants by simulating step responses against ArduPilot SITL recorded responses |
| Phase 1 policy learns to exploit mirror bugs (unrealistic actions look optimal) | Conservative action bounds (`delta_per_step` in YAML); reward includes smoothness penalty; sanity-check policy on SITL at 10k-step intervals |
| Phase 2 fine-tuning unlearns Phase 1 entirely | Lower learning rate in Phase 2; resume from saved replay buffer if SB3 supports it; track reward at fine-tune start vs end |
| 50k Phase 1 envs × DR → memory/time blow up | Start at 1k envs, scale up only after profiling |
| Domain randomization too wide → Phase 1 never converges | Curriculum: train without DR first, add it gradually |
| Sim-to-sim gap is bigger than fine-tuning can close in 50k steps | Iterate on Phase 1 mirror — closer-to-ArduPilot filtering, better motor model, etc. Standard sim-to-sim research cycle. |

---

## 9. Implementation roadmap (ordered, each step independently testable)

### Milestone 1: Phase-1 dynamics  ✓ DONE 2026-05-06

- [x] **1.1** `dynamics/motor_model.py` — torch port of bridge motor + aero math.
      Vectorized over batch dim. Hover thrust calibration verified: 4×PWM_HOVER
      → total thrust = m·g to float precision.
- [x] **1.2** `dynamics/body.py` — torch rigid-body integration with
      quaternion attitude. Free-fall and hover behaviors match analytic
      expectations.
- [x] **1.3** Smoke tests pass: free-fall close to (kinematics + drag),
      pre-spun hover holds altitude to floating-point precision, differential
      thrust produces correct-sign roll torque.

### Milestone 2: Phase-1 cascaded PID mirror  ✓ DONE 2026-05-06

- [x] **2.1** `dynamics/atc_psc_mirror.py` — altitude P/D, attitude P,
      rate P/I/D with integrator clamp, F450 X-config mixer. Param names
      mirror ArduPilot's MAVLink names so the policy's gain semantics
      transfer 1:1 to Phase 2.
- [x] **2.2** Integration tests pass: hover from ground at level start
      converges to within 1.6 cm of target in 5 s (PSC I-term still winding
      up); recovers to 0° attitude from +5° and +15° roll perturbations,
      drone drifts horizontally during recovery (expected — no PSC horizontal
      in v0).
- [ ] **2.3** Step-response calibration vs ArduPilot SITL recorded log —
      DEFERRED to v1 cycle after first training run shows transfer quality.

### Milestone 3: Phase-1 vectorized env  ✓ DONE 2026-05-06

- [x] **3.1** `envs/hover_pretrain_v0.py` — torch-vectorized env class.
      Same 19-dim obs + 8-dim action spec as `hover_pid_tuner_v0.py`.
- [x] **3.2** DR sampled per env on `reset()` and on auto-reset
      (mass, motor τ, K_drag, K_thrust jitter, gyro bias/noise, init
      attitude/altitude/xy, initial gain mistuning).
- [x] **3.3** Reward function identical to Phase 2 (same weights from
      `hover_v0.yaml`).
- [x] **3.4** Smoke: 64 envs × 20 steps × 200 inner physics each in 4.3 s
      wall on CPU torch (~50k physics ticks/sec). Will scale ~10-50× on
      CUDA torch.

### Milestone 4: Phase-1 trainer  ✓ DONE 2026-05-06

- [x] **4.1** `train_pretrain.py` — SB3 PPO launcher (PPO chosen for
      vectorized envs; on-policy is well-suited when the env is fast).
      Saves to `logs/checkpoints/pretrain_v0/*.zip`.
- [x] **4.2** `envs/sb3_vec_adapter.py` — adapter from
      `HoverPretrainVecEnv` to SB3's `VecEnv` API.
- [x] **4.3** Smoke run: 5000 timesteps × 64 envs in 21 s wall, model saved,
      TensorBoard log written. Reward progression visible in `logs/tb_logs/pretrain_v0`.
- [ ] **4.4** Full run: 1M timesteps × 1024 envs. **Pending GPU torch
      upgrade** — see "Outstanding work" below.

### Milestone 5: weight transfer + Phase-2 fine-tune

- [ ] **5.1** Resume Phase 2 from a Phase 1 checkpoint:
      `train_hover.py --resume logs/checkpoints/pretrain_v0/final.zip`. Verify the
      policy survives takeoff in SITL on first try (no random catastrophic
      action).
- [ ] **5.2** Reward-at-resume sanity check: log the first 100 steps' reward
      in Phase 2 — it should be in the same range as Phase 1's converged
      reward, not the random-action floor.
- [ ] **5.3** Fine-tune for 10k SITL steps. Compare hover quality (alt rms,
      pos rms, attitude rms) before vs after fine-tune.

### Milestone 6: evaluation

- [ ] **6.1** Eval harness: take a saved policy, run N hover episodes through
      Phase 2 only (real ArduPilot, real Isaac), report metrics matching
      `benchmark_hover` gates so we can compare against the v2 PID baseline.
- [ ] **6.2** Compare: pre-fix PID baseline (already collected) vs post-fix
      PID baseline (5 calm + 10 worst_case being collected now) vs
      Phase-1-only policy vs Phase-1+Phase-2 policy. The story for the
      capstone writeup.

---

## 10. File layout (target)

```
Swarm_Drones/capstone/
├── dynamics/
│   ├── __init__.py
│   ├── motor_model.py          # M1 — torch motor + aero
│   ├── body.py                 # M1 — rigid-body integration
│   └── atc_psc_mirror.py       # M2 — cascaded PID mirror
└── rl/
    ├── PLAN_C.md               # this file
    ├── README.md               # setup + run commands
    ├── requirements.txt        # pip freeze
    ├── .rl_venv/               # gitignored
    ├── cfg/
    │   ├── hover_v0.yaml       # Phase 2 (existing)
    │   └── pretrain_v0.yaml    # M4 — Phase 1 hyperparams + DR ranges
    ├── envs/
    │   ├── __init__.py
    │   ├── hover_pid_tuner_v0.py   # Phase 2 (existing)
    │   └── hover_pretrain_v0.py    # M3 — vectorized torch env
    ├── config_loader.py
    ├── train_hover.py          # Phase 2 launcher (existing)
    └── train_pretrain.py       # M4 — Phase 1 launcher
```

---

## 11. Outstanding work

After Milestones 1-4 done (2026-05-06), the remaining items are user-driven
or resource-driven:

1. **GPU utilization is lower than originally estimated.** Discovered after
   installing the CUDA torch build (`torch 2.11.0+cu128`, RTX 5090 SM_120
   detected, 34 GB VRAM available, GPU matmul confirmed working):

   At our current per-env tensor sizes (3-element body state, 4-element
   omega, 8-element gain vector), each tensor op is dominated by CUDA
   kernel-launch overhead (~10-30 µs per launch on Windows), not compute.
   With 200 inner physics steps × ~15 tensor ops per step = ~3000 launches
   per env step. CUDA: 110 k physics ticks/s. CPU torch: 350 k ticks/s.
   **CPU torch is ~3× faster than CUDA for this exact workload.** SB3
   prints the same warning for the PPO MlpPolicy ("GPU is poor for small
   networks, prefer CPU").

   So `device: "cpu"` for the env is the configured default. Throughput
   benchmark on CPU at the chosen `num_envs = 4096`:

   | Batch | env-steps/s | physics ticks/s | wall clock for 1 M timesteps |
   |---|---|---|---|
   | 256   | 1.2 k | 0.24 M | ~14 min |
   | 1024  | 2.6 k | 0.52 M | ~6 min |
   | 4096  | 5.6 k | 1.12 M | **~3 min** |
   | 8192  | 7.3 k | 1.46 M | ~2.5 min |

   Diminishing returns past B=8192 (Python-side overhead floor). For the
   first full training run we use B=4096 which is the knee of the curve.

   **Total wall clock for the planned 1 M-step Phase 1 run: under 10
   minutes**. Faster than the originally estimated "1-3 hours on GPU"
   precisely because we're bottlenecked on CPU tensor dispatch, not
   actual compute, and CPU dispatch is cheap.

   Future GPU-utilization options if we ever need more throughput
   (none of these are blocking the current capstone work):
   - `torch.compile` on the physics inner loop to fuse kernel launches.
   - Rewrite the inner loop as a single torch op (e.g., via
     `torch.func.scan`) so 200 ticks runs in one CUDA dispatch.
   - Increase the policy network size (small MLPs don't benefit from
     GPU; CNN-class networks do).
   - Move to IsaacLab for Stage 4-5 where vectorized PhysX shines.

2. **Step-response calibration (M2.3)** — capture an ArduPilot SITL log of
   a 10° roll attitude step (e.g., from your existing flight logs at
   `logs/flight_logs/`), simulate the same step with our mirror at default
   gains, plot rise time + overshoot side-by-side, tune the mirror's
   filter τ if rise/overshoot differ by >20%. This bounds the sim-to-sim
   gap before we commit to a long pre-train run.

3. **Full Phase-1 training run** — once GPU is wired and M2.3 is done,
   1M timesteps × 1024 envs. Expected wall time ~1-3 h on the 5090.
   Watch reward, episode length, action-norm in TensorBoard.

4. **Phase-2 fine-tune (M5)** — load `pretrain_v0_final.zip` into
   `train_hover.py --resume`. Verify policy survives takeoff in SITL on
   first try. Fine-tune for ~10-50k SITL steps. Compare hover metrics.

5. **Evaluation harness (M6)** — wrap `benchmark_hover` so it can score a
   trained policy alongside the pre-fix and post-fix PID baselines. The
   capstone-writeup story.

## 12. Decisions log (append-only, dated)

- **2026-05-06** — Plan C chosen over Plan A (single-SITL only) and Plan B
  (parallel SITL only). Rationale: GPU utilization + published-research
  novelty + faster total wall-clock + same final policy quality. User
  rejected Plan D (drop ArduPilot, full motor-level RL) because Stages 4-5
  (missions, swarm) need ArduPilot's mature WP/RTL/failsafe behavior.
- **2026-05-06** — Action set frozen at 8 attitude-loop gains
  (`ATC_ANG_*_P`, `ATC_RAT_*_{P,I,D}` for roll+pitch). Yaw and PSC are v1+
  extensions.
- **2026-05-06** — Phase 1 will use pure torch, NOT IsaacLab. Reason: SITL
  bottlenecks Phase 2 anyway, IsaacLab adds setup cost without speeding up
  the bottleneck. IsaacLab remains on the table for Stages 4-5 (swarm).
