---
title: "Weekly Report"
subtitle: "Physics, controllers and benchmark"
author: "Armen Mkrtumyan"
date: "2026-05-03"
---

# What was accomplished this week

1. **Physics of the 2D simulator.** 
2. **3 Drone Swarm controllers implemented** — Potential Fields,
   Consensus-Based Coordination, and Multi-Agent RL.
3. **Comparison study.** 144 runs across 3 hand-drawn maps × 4 swarm sizes
   × 4 policies × 3 seeds

# Questions / Challenges

The most difficult and time-consuming part was ensuring the physics engine is correctly modeled. I would say this is the most important piece also, since if the physics is not correct, no optimization algorithm would work properly on it.

# Goals for next week

Do a full e2e run with what Raffi and Elen did, to understand which algorithm will perform better for the given case. 

# 1. Physics fix - what our drones have

Each drone is a **Hawk's Work F450** 

| Component | Spec | Mass |
|---|---|---|
| Frame | F450, 450 mm wheelbase | 280 g |
| Motors (4×) | A2212 920 KV brushless | 208 g |
| ESCs (4×) | 20 A brushless | 71.6 g |
| Propellers (4×) | 9450 self-tightening | 40 g |
| Battery | 11.1 V 3S LiPo, 4200 mAh, 25 C | 330 g |
| Flight controller | Pixhawk 2.4.8 (PX4 autopilot) | 15.8 g |
| Companion computer | NVIDIA Jetson Nano | 178 g |
| Camera | e-con STEEReoCAM Nano | 159 g |
| **Total** | | **≈ 1.3 kg** |


# 2. How did I do the tests?

## 2.1. Test maps

![Hand-drawn evaluation maps](../outputs/images/eval_maps.png)

## 2.2. Swarm sizes

Tested four drone counts: **n_drones in {2, 5, 10, 20}**. Each drone is
the same Hawk's Work F450 build (1.3 kg, 11.1 V 3S 4200 mAh LiPo,
STEEReoCAM Nano forward-facing wedge sensor).

## 2.3. Algorithms implemented

| Controller | Type | State | Training |
|---|---|---|---|
| **Random** | baseline | stateless | none |
| **PF** | classical (potential fields) | stateless | none |
| **Consensus** | classical (decentralized) | stateless | none |
| **MARL** | reinforcement learning | stateful (NN weights) | 150K PPO steps × 4 parallel envs |

**Potential Fields.** Khatib-style superposition of three forces:

```
F_total = attract_to_nearest_uncovered  +  repulse_from_drones  +  repulse_from_walls
```

**Consensus.** 

Each drone polls neighbors; the communication group implicitly partitions
uncovered cells by Euclidean distance, and each drone heads to the
**nearest uncovered cell in its own Voronoi region**. Repulsion identical
to PF.

**MARL.** 

Independent PPO with shared parameters via Stable-Baselines3.
The joint policy network sees per-drone local state (own pos/vel/heading,
battery fraction, 5×5 local coverage mask, global coverage fraction)
flattened across the swarm; emits joint 2D acceleration.

# 2.4. Benchmarks

3 maps × 4 drone counts × 4 policies × 3 seeds = **144 runs**

## Final coverage (mean of 3 seeds)

![Final coverage vs swarm size](../outputs/images/sweep_coverage_vs_drones.png)

From here we can already observe that **n = 10 is the sweet spot**. For a 165 × 165 m world (24,025 m² navigable interior in the open case), 10 drones is enough.

### `open_33` — open arena

| n_drones | Random | PF | Consensus | MARL |
|---|---|---|---|---|
| 2  | **94.8 %** | 55.4 % | 53.5 % | 61.6 % |
| 5  | **99.8 %** | 94.7 % | 87.4 % | 97.0 % |
| 10 | 100.0 % | **100.0 %** | **100.0 %** | 99.0 % |
| 20 | 100.0 % | 100.0 % | 100.0 % | 100.0 % |

### `partial_33` — scattered obstacles

| n_drones | Random | PF | Consensus | MARL |
|---|---|---|---|---|
| 2  | 82.4 % | 83.6 % | **91.1 %** | 50.1 % |
| 5  | 98.2 % | 92.5 % | **100.0 %** | 92.2 % |
| 10 | **100.0 %** | **100.0 %** | 99.9 % | 97.9 % |
| 20 | 100.0 % | 100.0 % | 100.0 % | 100.0 % |

### `closed_33` — maze-like

| n_drones | Random | PF | Consensus | MARL |
|---|---|---|---|---|
| 2  | **73.7 %** | 64.0 % | 65.1 % | 51.7 % |
| 5  | 94.6 % | 86.2 % | **96.0 %** | 79.8 % |
| 10 | **99.3 %** | 98.4 % | 97.6 % | 95.2 % |
| 20 | 99.9 % | **100.0 %** | 99.7 % | 99.7 % |

## Time to 80 % coverage

**Consensus is the fastest** to 80 % at n ≥ 5 across all three maps. PF
catches up at n = 20. MARL is consistently the slowest because its
trained policy emits less aggressive forward motion than the analytic
potential gradient.

![Time to 80 % vs swarm size](../outputs/images/sweep_time_to_80_vs_drones.png)

## Re-coverage waste (wasted entries per % coverage)

**PF wins on efficiency, especially at large swarms**. This is because in PF each drone heads to its nearest *uncovered* cell, so once a cell is covered the target moves forward and the drone rarely backtracks.

![Re-coverage waste](../outputs/images/sweep_efficiency_vs_drones.png)

## Coverage curves over time — full grid

**MARL — a reinforcement learning method — never wins.** I was surprised by this. It could be either the training time I gave, or the parameters. But overall, I expected better results from it.

![Coverage curves grid](../outputs/images/sweep_curves_grid.png)