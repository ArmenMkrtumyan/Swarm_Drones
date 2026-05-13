# Objective functions — what each algorithm *actually* optimizes

There are two distinct objectives in this project. The **evaluation composite** in `score.py` is what we use to *rank* runs after they finish. Each algorithm's **internal objective** is what drives its target / action selection during a run. They are not the same — only MARL trains against the composite directly.

This doc lists each objective verbatim from code, with math notation alongside the source pointer, for presentation use.

---

## 0. The evaluation composite (used to score *every* algorithm after the fact)

**Source:** `score.py:44-87`

$$
\mathrm{score} \;=\; w_{\text{cov}} \cdot \mathrm{cov} \;-\; w_{\text{ov}} \cdot \frac{\mathrm{overlap}_{m^2}}{A_{\text{free}}} \;-\; w_{\text{wast}} \cdot \frac{n_{\text{wasted}}}{n_{\text{wasted}} + n_{\text{unique}}} \;-\; w_{\text{en}} \cdot \frac{E_{\text{used}}}{E_{\text{budget}}}
$$

With **default weights** `ScoreWeights(coverage=1.0, overlap=0.30, wasted=0.30, energy=0.20)`. Range ≈ `[−0.8, +1.0]`.

```python
return (
    w.coverage * coverage_fraction
    - w.overlap * overlap_frac
    - w.wasted * wasted_frac
    - w.energy * energy_frac
)
```

**Key point:** *no controller sees this during a run.* It's the **outer** ranking function used by `tools/bo_search.py` to tune hyperparameters and by `outputs/leaderboards/` to compare algorithms.

---

## Notation used below

- $p_i$ = position of drone $i$ (cells)
- $v_i$ = velocity of drone $i$ (cells/s)
- $\theta_i$ = heading of drone $i$ (rad)
- $\mathcal{U}_t$ = set of uncovered free cells at step $t$, $|\mathcal{U}_t| = M$
- $c$ = a candidate target cell (cell center)
- $\hat{u}_{a \to b} = (b - a) / \|b - a\|$ = unit vector from $a$ to $b$
- $t_i$ = drone $i$'s **current target**
- All algorithms emit `actions[i, :] = (a_x, a_y, \alpha_{\mathrm{yaw}})`; env clips translation at `max_accel` and yaw at `max_yaw_accel`

---

## 0.5 Important: separate the **algorithm** from the **flight controller**

Most algorithms in this project follow a clean two-layer split:

> **Algorithm = pick a target cell $t_i$ each step.**
>
> **Flight controller = "fly to target $t_i$ with the same PD-style motion law everyone reuses".**

The flight controller is the same code across 11 of the 13 controllers:

$$
a_i \;=\; \underbrace{K_a \cdot \hat{u}_{p_i \to t_i}}_{\text{attract toward target}} \;-\; \underbrace{K_d \cdot v_i}_{\text{velocity damping}} \;+\; \underbrace{\sum_{j \ne i} \frac{K_{dr}(p_i - p_j)}{\max(\|p_i - p_j\|^2,\, \varepsilon)} \mathbb{1}[\|p_i - p_j\| < r_{dr}]}_{\text{repel from other drones}} \;+\; \underbrace{\sum_w \frac{K_{wr}(p_i - p_w)}{\max(\|p_i - p_w\|^2,\, \varepsilon)} \mathbb{1}[\|p_i - p_w\| < r_{wr}]}_{\text{repel from walls}}
$$

After the sum, the env clips to `max_accel`. The same code path emits this in `controllers/{boustrophedon, spiral, voronoi_partition, grid_decomposition, stc, ga, sa, aco, potential_fields, consensus}.py` — the only thing that changes between them is **how $t_i$ is computed**.

> **Therefore, when reading each algorithm's section below, focus on the "target rule." The motion-law block above applies to all of them.**

**Two exceptions** that don't use this template:

- **PSO and GWO** have algorithm-specific acceleration formulas (PSO: inertia + cognitive + social pulls; GWO: average of three leader pulls). For these, the motion law *is* the algorithm — they're called out below.
- **MARL** has no explicit target. A neural network maps observation → acceleration directly, bypassing the entire target-then-fly stack.

---

# Track 1 — Classical (geometric / deterministic plans)

## 1.1 Boustrophedon — `controllers/boustrophedon.py`

*Uses the shared flight controller (§0.5). Algorithm is only the target rule.*

**Plan generation** (`_compute_plan`, lines 143-220):
- Vertical strips: drone $i$ owns $x \in [1 + i \cdot w_s + 0.5,\, 1 + (i+1) \cdot w_s - 0.5]$ where $w_s = (W-1)/n$
- Waypoint list alternates $(x_{\text{left}}, y_k)$ and $(x_{\text{right}}, y_k)$ for $y_k = 1.5 + k \cdot \ell$, lane spacing $\ell$
- Wall-snap: if $(x_{\text{wp}}, y_{\text{wp}})$ lands on a wall cell, replace with the nearest free cell within `wall_snap_radius`

**Target each step** (`__call__`, lines 234-280):
$$
t_i = \mathrm{wps}_i[k] \quad \text{where } k = \min\{j : \mathrm{wps}_i[j] \text{ not reached/covered/stuck}\}
$$

## 1.2 Spiral — `controllers/spiral.py`

*Uses the shared flight controller (§0.5). Same target-rule template as Boustrophedon; only the plan differs.*

**Plan generation** (lines 120-154): per-drone Archimedean spiral from start $(x_0, y_0)$:
$$
(x(\theta), y(\theta)) = (x_0 + r \cos \theta,\, y_0 + r \sin \theta), \quad r = \frac{\mathrm{pitch} \cdot \theta}{2\pi}
$$
Sampled every $\Delta\theta = $ `angle_step_deg` until $r > $ `max_radius`. Same wall-snap as Boustrophedon.

**Target each step:** same waypoint-follower logic as Boustrophedon (lines 200-260).

## 1.3 VoronoiPartition — `controllers/voronoi_partition.py`

*Uses the shared flight controller (§0.5). Algorithm is only the target rule.*

**Plan generation** (`_compute_partition`, lines 100-114): a **one-time** Voronoi partition over the free cells, frozen at $t=0$:
$$
\mathrm{owner}(y, x) = \arg\min_j \left\| p_j(0) - (x + 0.5,\, y + 0.5) \right\|
$$
Drone $i$'s region: $R_i = \{(y, x) : \mathrm{owner}(y, x) = i\}$.

**Target each step** (lines 171-196):
$$
t_i = \arg\min_{c \in R_i \cap \mathcal{U}_t \setminus \mathcal{B}_i} \|p_i - c\|
$$
where $\mathcal{B}_i$ is drone $i$'s stuck-detector blacklist. Fallback to global $\arg\min_{c \in \mathcal{U}_t}$ if $R_i$ is fully covered.

```python
# voronoi_partition.py:181-182
d2 = ((my_pos - drone.pos) ** 2).sum(axis=1)
target = my_pos[int(d2.argmin())]
```

## 1.4 GridDecomposition — `controllers/grid_decomposition.py`

*Uses the shared flight controller (§0.5). Algorithm is only the target rule.*

**Plan generation** (lines 112-155): blocks $b = (y_0, y_0 + B,\, x_0, x_0 + B)$, centroid $c_b = $ mean of free cells in $b$. Drone $i$ owns:
$$
\mathcal{B}_i^{\text{blocks}} = \left\{b : \arg\min_j \|p_j(0) - c_b\| = i\right\}
$$
sorted ascending by $\|c_b - p_i(0)\|$.

- **Target each step** (lines 199-238): walk through own block list, picking nearest uncov in current block; advance when block fully covered.
$$
t_i = \arg\min_{c \in b_k \cap \mathcal{U}_t \setminus \mathcal{B}_i} \|p_i - c\|
$$

## 1.5 STC (Spanning Tree Coverage) — `controllers/stc.py`

*Uses the shared flight controller (§0.5). Algorithm is only the target rule.*

**Plan generation** (`_compute_plans`, lines 159-188): Voronoi partition as above, then a **4-connected BFS walk** from each drone's start:
$$
W_i = \mathrm{BFS}_4(R_i,\; \mathrm{start} = \lfloor p_i(0) \rfloor)
$$

- **Target each step:** next walk index that's neither covered nor blacklisted:
$$
t_i = (x_k + 0.5,\, y_k + 0.5) \quad \text{where } (y_k, x_k) = W_i[k_{\min}]
$$

---

# Track 2 — Metaheuristic (population-based stochastic search)

## 2.1 PSO — `controllers/pso.py:154-188`

⚠️ *Exception: PSO does **NOT** use the shared flight controller. Its acceleration formula is the algorithm.*

**Target choice each step:**
$$
\mathrm{pbest}_i = \arg\min_{c \in \mathcal{U}_t} \|p_i - c\|, \qquad \mathrm{gbest} = \arg\min_{c \in \mathcal{U}_t} \|\bar{p} - c\|
$$
where $\bar{p} = \frac{1}{n} \sum_j p_j$ is the swarm centroid.

- **Update rule (acceleration command)**:
$$
a_i = \underbrace{w \cdot v_i}_{\text{inertia (keep momentum)}} \;+\; \underbrace{c_1\, r_1 \odot (\mathrm{pbest}_i - p_i)}_{\substack{\text{cognitive} \\ \text{(pull to own best)}}} \;+\; \underbrace{c_2\, r_2 \odot (\mathrm{gbest} - p_i)}_{\substack{\text{social} \\ \text{(pull to swarm best)}}} \;-\; \underbrace{K_d \cdot v_i}_{\substack{\text{damping} \\ \text{(brake)}}}
$$
with $w = $ `inertia`, $c_1 = $ `cognitive`, $c_2 = $ `social`, $r_1, r_2 \sim \mathcal{U}(0,1)^2$, plus wall-repel sum.

```python
# pso.py:181-187
a = (
    cfg.inertia * v
    + cfg.cognitive * r1 * (pbest_pos[i] - pos)
    + cfg.social * r2 * (gbest_pos - pos)
    - cfg.attract_damp_gain * v
)
```

## 2.2 GA — `controllers/ga.py`

*Uses the shared flight controller (§0.5). Algorithm is the fitness function + target-evolution operators below.*

#### The GA metaphor (unusual application)

Standard GA: a population of *abstract solutions* evolves over generations toward a fixed optimum. Our GA: the **swarm itself is the population**. Each drone is one "chromosome," and that chromosome is just *one number* — the drone's current target cell. The swarm evolves targets while physically chasing them.

| Standard GA | Our adaptation |
|---|---|
| Population of chromosomes | The $n$ drones |
| Chromosome | A target cell $t_i$ (2D coordinate) |
| Fitness | Count of uncov cells near $t_i$ (see below) |
| Selection | Elite preservation — top fittest drones keep their targets |
| Mutation | Random jump to a nearby uncov cell |
| Crossover | Copy an elite's target with Gaussian perturbation |
| Generation | One `__call__` of the controller (gated on drone arrival) |

#### Fitness function (`_fitness`, lines 142-146)

$$
\underbrace{\mathrm{fitness}(t)}_{\text{score for target } t} \;=\; \underbrace{\Big|\,\{\, c \in \mathcal{U}_t \,:\, \|c - t\| < r_{\text{fit}}\,\}\,\Big|}_{\text{count of uncov cells within } r_{\text{fit}} \text{ of } t}
$$

Reading the right-hand side piece by piece:

- $t$ — the candidate target cell we're scoring (any 2D point on the map)
- $\mathcal{U}_t$ — the set of currently uncovered free cells
- $c \in \mathcal{U}_t$ — each $c$ is an uncov cell; we iterate over all of them
- $\|c - t\| < r_{\text{fit}}$ — the filter: keep only uncov cells whose Euclidean distance to $t$ is less than `fitness_radius`
- $\{\,\ldots\,\}$ — the **set** of $c$'s satisfying the filter
- $|\,\ldots\,|$ — the **cardinality**: how many elements are in the set, i.e. the count

**Plain English:** the fitness of a target $t$ is "**how many uncov cells are sitting in a circle of radius `fitness_radius` centered at $t$**." Higher fitness = target sits in a *cluster* of uncov work. Visiting a high-fitness cell means the drone covers many cells with one trip (sensor sweep + nearby motion).

**Worked numeric example.** Target $t = (5, 5)$, `fitness_radius` = 4. Six uncov candidates:

| Uncov cell $c$ | $\|c - t\|$ | Inside circle? |
|---|---|---|
| (4, 4) | 1.41 | ✓ |
| (7, 5) | 2.00 | ✓ |
| (5, 8) | 3.00 | ✓ |
| (3, 7) | 2.83 | ✓ |
| (9, 9) | 5.66 | ✗ |
| (1, 1) | 5.66 | ✗ |

`fitness((5, 5))` = **4** — four uncov cells inside the circle of radius 4.

A target like (8, 8) would have a different score against the same map — possibly higher if there's a denser cluster of uncov work over there. The fittest cells are *in the middle of the most work*.

```python
# ga.py:142-146
def _fitness(self, target, uncov_pos):
    d2 = ((uncov_pos - target) ** 2).sum(axis=1)
    return float((d2 < self.cfg.fitness_radius ** 2).sum())
```

#### Generation step (lines 226-264, gated on per-drone arrival)

Each step, the GA runs three operators in sequence on the swarm's targets. Operators are only applied to drones that have **arrived** at their previous target (target-commitment from §3.3 of the README) — drones still in flight keep their target untouched.

**1. Elite selection** ("the fittest survive").
$$
\mathcal{E} \;=\; \{\,\text{top}\;\lceil \mathrm{elite\_fraction} \cdot n \rceil \text{ drones, sorted by fitness descending}\,\}
$$

  - Compute `fitness(t_i)` for every drone $i$
  - Sort drones by fitness, descending
  - Top $\lceil 0.40 \cdot n \rceil$ drones are **elite** — they keep their current targets unchanged
  - The remaining $n - |\mathcal{E}|$ drones will be mutated or crossed-over

For $n = 5$: elite count = $\lceil 0.40 \cdot 5 \rceil = 2$. The 2 fittest drones keep their targets; the other 3 are eligible for evolution.

**2. Mutation** (probability $p_{\text{mut}} = 0.35$, applied independently per non-elite drone).

  - Roll a uniform $u \sim \mathcal{U}(0, 1)$. If $u < p_{\text{mut}}$, mutate.
  - **Pick a random uncov cell** within `mutation_jump_radius` of the drone's *current position*.
  - Replace the drone's target with that random cell. Skip crossover for this drone.

$$
t_i \;\leftarrow\; \mathrm{Uniform}\,\Big(\,\{\, c \in \mathcal{U}_t \,:\, \|c - p_i\| < r_{\text{mut}}\,\}\,\Big)
$$

Mutation is the **exploration** operator — it injects randomness so the swarm doesn't all converge on the same area. Without mutation, all non-elites would just copy elites and the swarm collapses to one spot.

**3. Crossover** (probability $p_{\text{cx}} = 0.15$, only if mutation didn't fire).

  - Pick a **random elite** $e \in \mathcal{E}$ uniformly.
  - Perturb the elite's target by Gaussian noise: $t' = t_e + \mathcal{N}\!\big(0,\, (\sigma_{\text{cx}}/2)^2\big)$
  - **Snap** $t'$ to the nearest uncov cell (so the perturbed target lands on a real uncov cell, not in a wall or covered area).
  - Replace the drone's target with the snapped cell.

$$
t_i \;\leftarrow\; \arg\min_{c \in \mathcal{U}_t}\, \Big\|\, t_e + \mathcal{N}(0,\, (\sigma_{\text{cx}}/2)^2)\, -\, c\, \Big\|, \quad e \sim \mathrm{Uniform}(\mathcal{E})
$$

Crossover is the **exploitation** operator — it copies known-good targets (from elites) with a small random offset, hoping the offset lands on an equally-good or better nearby cell. This is how successful patterns propagate through the swarm.

**Drones that get neither operator** (mutation rolled false AND crossover rolled false) keep their current target. With $p_{\text{mut}} = 0.35$ and $p_{\text{cx}} = 0.15$, that's a $0.65 \cdot 0.85 = 0.55$ chance of no change per non-elite per generation.

#### Step-by-step example for n = 5 drones

Suppose at some generation:

| Drone | Position $p_i$ | Target $t_i$ | fitness($t_i$) |
|---|---|---|---|
| 1 | (10, 10) | (12, 10) | 8 |
| 2 | (15, 15) | (17, 15) | 5 |
| 3 | (3, 4) | (4, 3) | 3 |
| 4 | (8, 12) | (10, 14) | 7 |
| 5 | (20, 5) | (22, 6) | 4 |

Sort by fitness descending: D1 (8), D4 (7), D2 (5), D5 (4), D3 (3).

Elite = top 2 = {D1, D4}. Keep their targets.

Non-elite = {D2, D3, D5}. For each, roll mutation, then crossover if mutation didn't fire:

- **D2**: roll $u_m = 0.92 > 0.35$ → no mutation. Roll $u_x = 0.08 < 0.15$ → crossover. Pick random elite, say D1. Perturb (12, 10) by $\mathcal{N}(0, \sigma_{\text{cx}}/2)$, get e.g. (12.7, 9.4). Snap to nearest uncov → maybe (13, 9). New target: (13, 9).
- **D3**: roll $u_m = 0.20 < 0.35$ → mutate. Pick random uncov within `mutation_jump_radius` of (3, 4), e.g. (5, 6). New target: (5, 6).
- **D5**: roll $u_m = 0.55$, $u_x = 0.30$ → both fail. Keep target (22, 6).

The targets evolve toward where the work is densest (via crossover from elites) while keeping some random exploration (via mutation).

#### Why this works for coverage

The fitness function says "**good targets are surrounded by lots of work.**" High-fitness cells are interior to dense uncov clusters — going there gives the drone a productive 5-second flight covering many cells, not a wasted trip to an isolated cell with empty neighbors.

The three operators implement the standard GA exploration–exploitation balance:
- **Elite preservation** (40 % keep their good targets) → don't lose progress
- **Crossover** (15 % copy elites with noise) → propagate successful patterns
- **Mutation** (35 % random jump) → explore new regions

The target-commitment gate (mutation/crossover only fire when a drone has arrived) is our project-specific addition — without it, the operators would reassign targets while drones are still in flight, and the drone would never reach any cell.

## 2.3 ACO — `controllers/aco.py`

*Uses the shared flight controller (§0.5). Algorithm is the pheromone update + target-sampling rule below.*

**Pheromone field** $\tau(y, x)$ updated every step (lines 134-135):
$$
\tau_{t+1}(y, x) \;=\; \underbrace{(1 - \rho) \cdot \tau_t(y, x)}_{\substack{\text{evaporation} \\ \text{(fade old memory)}}} \;+\; \underbrace{Q \cdot \mathbb{1}[(y, x) \in \mathcal{U}_t]}_{\substack{\text{deposition} \\ \text{(add Q if cell is uncov)}}}
$$
Uncov cells accumulate $\tau$ over time; once covered, only evaporation acts and $\tau$ decays toward 0.

- **Heuristic** (lines 188):
$$
\underbrace{\eta(c)}_{\substack{\text{proximity bias} \\ \text{(higher = closer)}}} \;=\; \frac{1}{\max(\|p_i - c\|,\, 0.5)}
$$
Inverse distance from drone $i$ to candidate cell $c$; the `max(d, 0.5)` floors the denominator so a drone sitting on the cell doesn't divide by zero.

- **Target sampling probability** (lines 185-200, on target commitment):

$$
\underbrace{P(t_i = c)}_{\text{prob. of picking } c} \;\propto\; \underbrace{\tau(c)^\alpha}_{\text{pheromone bias}} \;\cdot\; \underbrace{\eta(c)^\beta}_{\text{distance bias}}
$$

restricted to $c$ within `target_search_radius` of $p_i$.

  - $P(t_i = c)$ — probability that drone $i$ picks cell $c$ as its next target. **Stochastic**: ACO rolls dice; it doesn't argmax.
  - $\tau(c)^\alpha$ — pheromone score; higher means the cell has been uncov for longer.
  - $\eta(c)^\beta$ — distance score; higher means the cell is closer to drone $i$.
  - The proportional sign $\propto$ means the right-hand side is the **score** of each cell, not the actual probability. To get probabilities you divide by the sum of all scores:

$$
P(c) \;=\; \frac{\tau(c)^\alpha \, \eta(c)^\beta}{\sum\limits_{c' \in \text{candidates}}\, \tau(c')^\alpha \, \eta(c')^\beta}
$$

#### How the drone actually picks a target — step by step

1. **Collect candidates.** The drone gathers every uncovered cell within `target_search_radius = 6.3` cells of itself. Call this list $\{c_1, c_2, \ldots, c_K\}$.
2. **Score each candidate** using the formula above. For each $c_k$, compute
$$
s_k \;=\; \tau(c_k)^\alpha \cdot \eta(c_k)^\beta
$$
   Both factors are positive, so all scores are positive.
3. **Normalize** the scores into probabilities:
$$
P(c_k) \;=\; \frac{s_k}{\sum_{j=1}^{K} s_j}
$$
   These now sum to 1 and form a proper probability distribution over candidates.
4. **Sample one candidate** by drawing $u \sim \mathcal{U}(0, 1)$ and picking the $c_k$ whose cumulative probability first exceeds $u$. (NumPy's `np.random.choice(K, p=probs)` does this in one line.)
5. **Drone $i$'s new target** is the sampled $c_k$.

Each drone does this independently every time it needs a new target — same pheromone field $\tau$, but different positions $p_i$ mean different $\eta$, so the distributions and the samples differ across drones.

#### Worked numeric example

Drone at $p_i = (5, 5)$. Three candidate uncov cells:

| Cell | Distance | $\eta = 1/d$ | $\tau$ | $s = \tau^{0.93} \cdot \eta^{3.62}$ |
|---|---|---|---|---|
| $c_1 = (6, 5)$ | 1.0 | 1.000 | 5 | $5^{0.93} \cdot 1^{3.62} \approx 4.65$ |
| $c_2 = (8, 6)$ | 3.16 | 0.316 | 20 | $20^{0.93} \cdot 0.316^{3.62} \approx 16.5 \cdot 0.0156 \approx 0.258$ |
| $c_3 = (10, 10)$ | 7.07 | 0.141 | 50 | $50^{0.93} \cdot 0.141^{3.62} \approx 38.7 \cdot 0.0008 \approx 0.029$ |

Sum of scores: $4.65 + 0.258 + 0.029 \approx 4.94$.

Normalize:

| Cell | Score $s$ | Probability $P = s / 4.94$ |
|---|---|---|
| $c_1$ | 4.65 | **94.1 %** |
| $c_2$ | 0.258 | 5.2 % |
| $c_3$ | 0.029 | 0.6 % |

Now sample: draw $u \sim \mathcal{U}(0, 1)$. If $u < 0.941$, pick $c_1$ (the near cell). Else if $u < 0.993$, pick $c_2$. Otherwise pick $c_3$.

So **94 % of the time the drone picks the close cell**, even though $c_3$ has 10× more pheromone. The reason: $\beta = 3.62$ means distance dominates pheromone *by a power of about 4*, and $c_3$'s distance penalty (raised to the 3.62 power) crushes its pheromone advantage.

The 0.6 % probability of jumping to $c_3$ is the **exploration tail** — ACO's promise that occasionally the drone will commit to a far-away long-neglected cell. Over thousands of steps across multiple drones, that small probability adds up to "the colony eventually visits even far/awkward cells."

#### Why this isn't `argmin` — the key difference

A `argmin` rule would pick $c_1$ **every time** (because it's closest). ACO picks $c_1$ **most of the time but not always**. That's the deliberate stochasticity:

- **Determinism trap** — `argmin` always greedy. If two drones are equidistant to a cell, both head there, collide, one wastes the trip.
- **Stochastic spread** — ACO has a small chance of picking $c_2$ instead, so two equidistant drones often pick *different* nearby cells just by random luck. The swarm spreads without explicit coordination.

That's why ACO is filed under "metaheuristic" rather than "classical": the algorithm's distinctive feature isn't the formula, it's the **dice-rolling**.

```python
# aco.py:189-200
scores = (np.maximum(cand_tau, 1e-9) ** cfg.pheromone_weight) \
       * (eta ** cfg.heuristic_weight)
probs = scores / scores.sum()
idx = int(self._rng.choice(len(cand_pos), p=probs))
target = cand_pos[idx]
```

## 2.4 SA — `controllers/sa.py`

*Uses the shared flight controller (§0.5). Algorithm is the Metropolis update on each drone's persistent target.*

#### The SA metaphor: cooling a metal

Real annealing: heat a metal, slowly cool it. At high temperature atoms move freely and can escape local "stuck" configurations; as you cool, atoms settle into the lowest-energy crystalline structure. Kirkpatrick, Gelatt, Vecchi (1983) used this as a metaphor: a temperature-controlled stochastic search that's willing to accept worse moves early (exploration) and only better moves late (greedy hill-climbing).

| Standard SA | Our coverage version |
|---|---|
| One candidate solution | One target cell **per drone**, each drone runs SA independently |
| Energy $E$ | $E = -\mathrm{fitness}(t)$ — we *maximize* fitness, so "energy" is what *decreases* when fitness goes up |
| Proposal | Gaussian perturbation, then snap to nearest uncov cell |
| Acceptance | Metropolis rule: always accept better; sometimes accept worse |
| Cooling | $T$ shrinks each step (geometric decay) |

#### Fitness function (same form as GA, lines 114-118)

$$
\underbrace{\mathrm{fitness}(t)}_{\text{score for target } t} \;=\; \underbrace{\Big|\,\{\, c \in \mathcal{U}_t \,:\, \|c - t\| < r_{\text{fit}}\,\}\,\Big|}_{\text{count of uncov cells within } r_{\text{fit}} \text{ of } t}
$$

  - $t$ — the candidate target cell being scored
  - $\mathcal{U}_t$ — set of currently uncovered cells
  - $r_{\text{fit}}$ — `fitness_radius` (default 4 cells); the circle radius around $t$ that defines "neighbors"
  - $|\cdot|$ — cardinality (count) of the set inside the braces

**Plain English:** fitness(t) is how many uncov cells sit in a circle of radius `fitness_radius` centered at $t$. High fitness = target is in the middle of a dense cluster of uncov work. Going there gives the drone a productive flight covering many cells with one trip.

#### Annealing schedule (line 196)

$$
\underbrace{T_{k+1}}_{\text{next step's temperature}} \;=\; \underbrace{\max\big(T_{\min},\, T_k \cdot \mathrm{cooling\_rate}\big)}_{\substack{\text{geometric decay,} \\ \text{floored at } T_{\min}}}
$$

  - $T_k$ — temperature at step $k$ (starts at `T_initial`)
  - `cooling_rate` — multiplicative decay (BO-tuned: 0.9616, so $T$ halves every $\ln 2 / \ln(1/0.9616) \approx 17.7$ steps)
  - $T_{\min}$ — floor that prevents $T$ from underflowing to numerical zero (default $10^{-3}$)
  - $\max(\ldots)$ — clamps $T$ from below so the Metropolis $\exp(-\Delta E / T)$ formula doesn't divide by zero

**Plain English:** temperature decays geometrically each step. With BO-tuned config: start at $T_0 = 3.31$, halves every ~18 steps, reaches the $10^{-3}$ floor around step 220.

| Step | $T$ value | Mode |
|---|---|---|
| 0 | 3.31 | hot — accepts most worse moves (exploration) |
| 50 | 1.50 | warm |
| 100 | 0.66 | cool |
| 200 | 0.13 | cold |
| 500+ | $10^{-3}$ floor | frozen — pure greedy hill-climbing |

#### Metropolis update (lines 198-217, gated on arrival)

Three substeps each time a drone has arrived at its target:

**(a) Propose a perturbation:**
$$
\underbrace{t'}_{\text{proposed new target}} \;=\; \underbrace{t_i}_{\text{current target}} \;+\; \underbrace{\boldsymbol{\xi}}_{\substack{\text{Gaussian noise} \\ \boldsymbol{\xi} \sim \mathcal{N}(0,\, \sigma_{\text{perturb}}^2 I)}}, \qquad t' \;\leftarrow\; \arg\min_{c \in \mathcal{U}_t}\, \|t' - c\|
$$

  - Sample a 2D Gaussian offset with standard deviation $\sigma_{\text{perturb}}$ (BO-tuned: 1.23 cells, so most proposals are within $\pm 2.5$ cells, 2σ).
  - **Snap** the proposed location to the nearest uncov cell. This guarantees the proposal is a valid target (not a wall, not already covered), even if the Gaussian sample landed somewhere bogus.

**(b) Compute the energy change:**
$$
\underbrace{\Delta E}_{\text{energy difference}} \;=\; \underbrace{\mathrm{fitness}(t_i)}_{\text{old fitness}} \;-\; \underbrace{\mathrm{fitness}(t')}_{\text{new fitness}}
$$

  - **Sign convention:** we *maximize* fitness, so a **better move (higher fitness)** gives $\Delta E < 0$, and a **worse move (lower fitness)** gives $\Delta E > 0$. This matches the physics convention where energy *decreases* in the desired direction. Hence "$\Delta E = -\Delta f$."

**(c) Apply the Metropolis acceptance rule:**
$$
\text{accept } t' \;\;\Longleftrightarrow\;\; \begin{cases} \text{always} & \text{if } \Delta E \le 0 \quad \text{(better or equal — climb the hill)} \\[6pt] \underbrace{u < \exp(-\Delta E / T)}_{\substack{\text{coin flip with bias} \\ \exp(-\Delta E / T)}} & \text{if } \Delta E > 0, \; u \sim \mathcal{U}(0, 1) \quad \text{(worse — sometimes step downhill)} \end{cases}
$$

  - **Better moves are always accepted** — there's no rejection of an improvement.
  - **Worse moves get a probabilistic acceptance**: $\exp(-\Delta E / T)$ is the probability of taking the move anyway. This is the **only stochastic decision in SA**.
  - **Effect of $T$:** higher $T$ → larger $\exp(-\Delta E / T)$ → more likely to accept worse moves → more exploration. As $T \to 0$, only better moves accepted → pure hill-climbing.
  - **Effect of $\Delta E$:** larger $\Delta E$ (much-worse move) → smaller $\exp(-\Delta E / T)$ → less likely to accept. SA is more willing to take *small* downhill steps than *big* ones.

```python
# sa.py:213-217
if delta <= 0:
    self._targets[i] = new_target               # better/equal — always accept
else:
    if self._rng.uniform() < math.exp(-delta / max(self._T, 1e-9)):
        self._targets[i] = new_target           # worse — sometimes accept
```

#### Worked numeric example — the acceptance probability over time

Suppose drone $i$ has $\mathrm{fitness}(t_i) = 5$ and proposes $t'$ with $\mathrm{fitness}(t') = 3$. So $\Delta E = +2$ (move is worse by 2 fitness points).

| Step | $T$ | $\exp(-2 / T)$ | Accept worse move? |
|---|---|---|---|
| 0 | 3.31 | $\exp(-0.60) = 0.547$ | **55 %** of the time |
| 50 | 1.50 | $\exp(-1.33) = 0.264$ | 26 % |
| 100 | 0.66 | $\exp(-3.03) = 0.0484$ | 5 % |
| 200 | 0.13 | $\exp(-15.4) = 2 \cdot 10^{-7}$ | essentially never |
| 500 | $T_{\min} = 10^{-3}$ | $\exp(-2000) \approx 0$ | never |

**Reading this table:** early in the run, the drone has a coin-flip chance of taking a step that makes fitness *worse*. Mid-run, that probability shrinks. Late in the run, only improvements are accepted.

If the proposed move were *better* ($\Delta E < 0$), it would be accepted at every temperature — no coin flip needed.

#### So are we minimizing?

**Yes — implicitly maximizing fitness through a temperature-controlled random walk.** Same pattern as GA: explicit objective, no explicit $\arg\max$. The optimization happens emergently across many Metropolis steps.

Formally: SA at fixed temperature $T$ samples from the **Boltzmann distribution** $P(t) \propto \exp(\mathrm{fitness}(t) / T)$. As $T \to 0$, this distribution concentrates on $\arg\max_t \mathrm{fitness}(t)$. So infinitely-slow cooling guarantees finding the global max (Geman & Geman 1984). In practice we cool fast enough to be useful but not infinitely slow, so we end up near a *good* local maximum.

#### What BO tuning revealed

| Knob | Default | BO-tuned | What changed |
|---|---|---|---|
| `T_initial` | 0.5 | **3.31** | 6.6× hotter start — much more willing to accept worse moves early |
| `cooling_rate` | 0.999 | **0.9616** | 25× faster cooling — commits to greedy mode sooner |
| `perturb_radius` | 3.0 | **1.23** | 2.4× smaller jumps — proposals stay close to current target |

The story: **start exploring boldly, cool fast, but make small jumps**. That's "wide exploration up front, then settle into a local optimum" — different from the textbook "very slow cooling for global optimality." It's tuned for our specific objective, where the fitness landscape isn't very rugged and time is bounded by the battery.

Result: SA scored **+0.6453 BO composite / 98.0 % mean coverage** — the overall winner across all 14 algorithms. Its single-particle simplicity + smart cooling beat both population-based methods (GA, PSO) and the carefully-tuned partition methods (Voronoi, STC).

## 2.5 GWO — `controllers/gwo.py:125-167`

⚠️ *Exception: GWO does **NOT** use the shared flight controller. Its acceleration formula is the algorithm.*

#### Slide-ready summary (screenshot-friendly)

**GWO (Grey Wolf Optimizer)**

- Three-leader social hierarchy (α, β, δ) guides the pack
- Each drone moves toward **consensus** of the 3 leader targets
- Random coefficients $A, C$ balance exploration vs. exploitation
- Exploration parameter $a$ decays linearly over time

$$
\underbrace{X_L}_{\substack{\text{candidate position} \\ \text{from leader } L}}
\;\;=\;\;
x_L \;-\; A_L \cdot \big|\, C_L \cdot x_L - p_i\, \big|
$$

$$
\underbrace{a_i}_{\text{drone acceleration}}
\;\;=\;\;
\frac{1}{3} \!\!\! \sum_{L \in \{\alpha,\, \beta,\, \delta\}} X_L \;\;-\;\; p_i
$$

---

#### The GWO metaphor: grey wolves hunting prey

Mirjalili, Mirjalili & Lewis (2014) modeled the pack hunting behavior of grey wolves. Wolves have a strict social hierarchy:

| Rank | Wolf | Role |
|---|---|---|
| 1st | **α (alpha)** | Pack leader. Knows roughly where the prey is. Best candidate solution so far. |
| 2nd | **β (beta)** | Lieutenant. Second-best estimate. |
| 3rd | **δ (delta)** | Scout/sentinel. Third-best estimate. |
| 4th+ | **ω (omega)** | Followers. Don't know where the prey is — *they update their position toward where α, β, δ collectively suggest.* |

The algorithm: each non-leader wolf computes three "candidate next-positions" (one influenced by each leader) and moves to the average. Over generations, the pack converges on the prey.

| Standard GWO | Our coverage version |
|---|---|
| Wolf | Drone |
| Prey | Uncov cells (the algorithm's "food") |
| Leader's known position | The leader drone's *nearest uncov cell* (i.e., where it's about to hunt) |
| Pack converges on prey | Drones converge on uncov regions |
| Position update $x_{k+1} = X_{\text{new}}$ | **Acceleration command** $a_i = X_{\text{new}} - p_i$ — used to drive the env's physics integrator |

#### Step 1 — Ranking & leader election (lines 130-143)

Each step, every drone $i$ knows its "fitness" = how close it is to the nearest uncov cell. Sort the swarm by that:

$$
\underbrace{r_0, r_1, r_2, \ldots, r_{n-1}}_{\text{drone indices, ascending by } \|p_i - \mathrm{nearestuncov}_i\|}
$$

The drone closest to its nearest uncov cell is $r_0$ (the **fittest**). The top three drones become the leaders, and their *targets* (nearest uncov cell of that drone) become the leader positions:

$$
\underbrace{x_\alpha = \mathrm{nearestuncov}_{r_0}}_{\text{best drone's target}}, \quad
\underbrace{x_\beta = \mathrm{nearestuncov}_{r_1}}_{\text{2nd-best target}}, \quad
\underbrace{x_\delta = \mathrm{nearestuncov}_{r_2}}_{\text{3rd-best target}}
$$

  - $r_0, r_1, r_2$ — drone indices (which drone is ranked 1st, 2nd, 3rd)
  - $\mathrm{nearestuncov}_j$ — drone $j$'s nearest uncovered cell
  - $x_\alpha, x_\beta, x_\delta$ — the three leader positions, expressed as 2D coordinates in cell space

**Important nuance:** the leader positions are NOT the leader drones' physical positions — they're the *targets* the leaders are heading toward. So leaders effectively share "where I'm aiming" with the rest of the pack.

#### Step 2 — The control parameter $a$ (lines 145-149)

GWO uses a single scalar $a$ that decays linearly over the run:

$$
\underbrace{a(k)}_{\text{control parameter at step } k} \;=\; \underbrace{a_{\text{initial}}}_{\text{starting value}} \;-\; \underbrace{\min(1,\, k / \mathrm{decay\_steps})}_{\substack{\text{progress fraction,} \\ \text{capped at 1}}} \cdot \underbrace{(a_{\text{initial}} - a_{\text{final}})}_{\text{range of decay}}
$$

  - $k$ — current step count
  - `a_initial` — value at $k = 0$ (BO-tuned: 2.88)
  - `a_final` — value once `decay_steps` have passed (BO-tuned: 0.68)
  - `decay_steps` — how many steps to take to finish decaying (BO-tuned: 206)

**Plain English:** $a$ shrinks linearly from `a_initial` down to `a_final` over the first `decay_steps` steps, then stays flat. This parameter controls **exploration vs. exploitation** — high $a$ means wider random search; low $a$ means tight pursuit of the leaders (see Step 3).

#### Step 3 — Per-leader pull (lines 159-167)

For each leader $L \in \{\alpha, \beta, \delta\}$, draw two random 2-vectors $r_1, r_2 \sim \mathcal{U}(0, 1)^2$ and compute three intermediate quantities:

$$
\underbrace{A_L}_{\substack{\text{attack-or-explore} \\ \text{coefficient, } \in [-a, +a]^2}} \;=\; 2\,a\,r_1 - a
$$

$$
\underbrace{C_L}_{\substack{\text{leader-position swing,} \\ \in [0, 2]^2}} \;=\; 2\,r_2
$$

$$
\underbrace{X_L}_{\substack{\text{candidate position} \\ \text{influenced by leader L}}} \;=\; x_L \;-\; A_L \odot \big|\, C_L \odot x_L - p_i\, \big|
$$

where $\odot$ is the elementwise (Hadamard) product and $|\cdot|$ is the component-wise absolute value.

**What each piece does:**

  - **$A_L = 2 a r_1 - a$.** With $r_1 \in [0, 1]^2$, the term $2 a r_1$ ranges over $[0, 2a]^2$; subtracting $a$ shifts it to $[-a, +a]^2$. So each component of $A_L$ is a uniform random number in $[-a, +a]$.
    - **If $|A_L| < 1$**: wolf moves *toward* the leader → **exploitation** (attack the prey).
    - **If $|A_L| > 1$**: wolf moves *away* from the leader → **exploration** (search elsewhere).
    - At $a = 2$ (high), about half of $A_L$ values exceed 1 → 50/50 split of exploration vs. attack.
    - At $a = 0$ (low), $A_L$ is always 0 → wolf moves to $X_L = x_L$ directly → pure attack.

  - **$C_L = 2 r_2$.** Each component is uniform in $[0, 2]$. This "swings" the leader's position randomly:
    - $C \approx 1$ → wolf computes distance to the leader as-is.
    - $C \approx 2$ → wolf overshoots, computing distance to "twice the leader's position."
    - $C \approx 0$ → wolf computes distance to the origin (ignoring leader).
    - This adds noise to the pursuit — wolves don't all converge to identical points.

  - **$X_L = x_L - A_L \odot |C_L \odot x_L - p_i|$.** The candidate new position.
    - $C_L \odot x_L - p_i$ is the (randomly-scaled) offset from drone $i$ to leader $L$.
    - The absolute value $|\cdot|$ converts it into a per-axis distance.
    - $A_L \odot |\cdot|$ is a *signed* per-axis step (with sign from $A_L$'s sign).
    - Subtracting from $x_L$ gives the new position. If $A_L > 0$, the step shrinks toward $x_L$; if $A_L < 0$, it moves past $x_L$ in the opposite direction.

#### Step 4 — Average the three pulls and compute acceleration (lines 168-171)

$$
\underbrace{X_{\text{new}}}_{\text{consensus next-position}} \;=\; \frac{X_\alpha + X_\beta + X_\delta}{3}
$$

  - The drone trusts each leader equally — averages the three candidates with equal weight.
  - More sophisticated GWO variants weight α more than β more than δ; our implementation uses uniform 1/3 weighting (matches Mirjalili 2014's basic formulation).

Then the **acceleration command** (since we drive a physical drone, not a teleporting search agent):

$$
\underbrace{a_i}_{\text{commanded acceleration}} \;=\; \underbrace{X_{\text{new}} - p_i}_{\text{displacement to consensus position}} \;-\; \underbrace{K_d \cdot v_i}_{\substack{\text{velocity damping} \\ \text{(our addition)}}} \;+\; \underbrace{\sum_w F_{iw}^{\text{wall}}}_{\text{wall repel sum}}
$$

  - $X_{\text{new}} - p_i$ — vector from drone to the consensus next-position. Used as an *acceleration* because the env integrates it (the original GWO teleports with $p_i \leftarrow X_{\text{new}}$; we can't, so we use the displacement as a force).
  - $K_d v_i$ — damping. Same role as in PSO; brakes against velocity to prevent orbit overshoot.
  - Wall-repel sum — same 1/r² form as the shared flight controller; not algorithm-specific.

```python
# gwo.py:160-171
for leader in (alpha, beta, delta):
    r1 = self._rng.uniform(0.0, 1.0, size=2)
    r2 = self._rng.uniform(0.0, 1.0, size=2)
    A = 2 * a * r1 - a                         # ∈ [-a, +a]^2
    C = 2 * r2                                  # ∈ [0, 2]^2
    X = leader - A * np.abs(C * leader - pos)
    X_pulls.append(X)
X_new = sum(X_pulls) / 3.0
accel = X_new - pos - cfg.attract_damp_gain * drone.vel
```

#### Worked numeric example

Suppose at step 50, $a = 1.5$ (mid-run), drone $i$ at $p_i = (5, 5)$, and the three leader targets are $x_\alpha = (10, 8)$, $x_\beta = (12, 6)$, $x_\delta = (4, 12)$.

Computing $X_\alpha$ (the rest are analogous):

- Sample $r_1 = (0.7, 0.3)$, so $A_\alpha = 2 \cdot 1.5 \cdot (0.7, 0.3) - 1.5 = (0.6, -0.6)$
- Sample $r_2 = (0.4, 0.9)$, so $C_\alpha = (0.8, 1.8)$
- $C_\alpha \odot x_\alpha = (0.8 \cdot 10,\, 1.8 \cdot 8) = (8, 14.4)$
- $C_\alpha \odot x_\alpha - p_i = (8 - 5,\, 14.4 - 5) = (3, 9.4)$
- $|C_\alpha \odot x_\alpha - p_i| = (3, 9.4)$
- $A_\alpha \odot |\cdot| = (0.6 \cdot 3,\, -0.6 \cdot 9.4) = (1.8, -5.64)$
- $X_\alpha = x_\alpha - A_\alpha \odot |\cdot| = (10 - 1.8,\, 8 - (-5.64)) = (8.2, 13.64)$

Doing the same for β and δ with fresh random samples gives, say, $X_\beta = (11.2, 7.1)$ and $X_\delta = (5.3, 10.8)$.

Average: $X_{\text{new}} = (8.2 + 11.2 + 5.3,\, 13.64 + 7.1 + 10.8) / 3 \approx (8.23, 10.51)$.

Displacement: $X_{\text{new}} - p_i = (3.23, 5.51)$. The drone accelerates up-right toward the consensus point (which is roughly the centroid of the three leaders' positions, but biased and noised by the random samples).

#### So are we minimizing?

**No clear objective being optimized.** GWO doesn't have an explicit fitness function in the same way GA/SA do — there's no $f(t)$ that the algorithm tries to maximize. Instead:

- Ranking by distance-to-nearest-uncov implicitly says "drones closer to work are more credible leaders."
- The update rule pulls each drone toward a *consensus average* of leader positions.
- Over many steps, the swarm clusters around regions with uncov work, because the leaders keep "knowing" where to go.

It's closer to **dynamical-system flocking** than optimization: there's no formal objective, but the system's emergent behavior is "wolves drift toward prey." The randomness in $A_L, C_L$ prevents collapse to a single point.

In our coverage taxonomy:

| Algorithm | Has explicit objective? | Optimization mechanism |
|---|---|---|
| GA | Yes (count of uncov in $B(t, r_{\text{fit}})$) | Population evolution |
| SA | Yes (same as GA) | Metropolis acceptance |
| ACO | No (heuristic + pheromone bias) | Stochastic sampling |
| **GWO** | **No** (distance-to-uncov is a *ranking* proxy, not a maximized quantity) | **Leader-follower dynamics** |

GWO is the most "metaheuristic-y" of the four — its mechanism is the *metaphor itself*, not a specific computable objective.

#### What BO tuning revealed

| Knob | Default | BO-tuned | What changed |
|---|---|---|---|
| `a_initial` | 5.0 | **2.88** | Lower starting exploration — wolves attack sooner |
| `a_final` | 0.5 | **0.68** | Slightly higher final — keeps a bit of noise even at convergence |
| `decay_steps` | 3000 | **206** | 15× faster decay — pack stops exploring almost immediately |
| `attract_damp_gain` | 0.2 | **0.0506** | 4× less damping — relies on the natural step size of $X_{\text{new}} - p_i$ |

The narrative: **BO killed almost all of GWO's exploration character.** With `decay_steps = 206`, $a$ reaches its final value after ~21 seconds — for a run that lasts up to 734 seconds, that means 97% of the run is in "low-exploration" mode. The pack essentially picks three leaders and crowds toward them with mild noise.

Result: GWO scored **+0.2475 BO composite / 80.1 % mean coverage** — the worst of all 14 algorithms. The leader-pull formulation has a fundamental ceiling on this coverage objective: when three drones each pull the rest toward their nearest cells, the swarm collapses onto those three cells instead of spreading. BO couldn't fix the structural problem; it could only minimize the damage by suppressing exploration noise and damping.

---

# Track 3 — Control / Learning

## 3.1 Potential Fields (PF) — `controllers/potential_fields.py:188-264`

*Uses the shared flight controller (§0.5). **The whole algorithm is one line:***

$$
t_i = \arg\min_{c \in \mathcal{U}_t} \|p_i - c\|
$$

Globally nearest uncov, **no partition**. The attract + damp + repel formula isn't algorithm-specific — it's the same flight controller every nearest-uncov method shares. PF is the cleanest example of "the algorithm is just the target rule, the rest is shared infrastructure."

```python
# potential_fields.py:198-203
d_uncov = uncov_pos - drone.pos
dist_sq = (d_uncov ** 2).sum(axis=1)
nearest = int(dist_sq.argmin())
target = drone.pos + d_uncov[nearest]   # the nearest uncov cell
```

## 3.2 Consensus — `controllers/consensus.py:217-260`

*Uses the shared flight controller (§0.5). Algorithm is the visibility-restricted Voronoi attractor rule.*

**Visibility-restricted neighbor set**: $\mathcal{N}_i = \{j : \|p_i - p_j\| \le \mathrm{commrange}\}$

- **Local Voronoi over group** $\{i\} \cup \mathcal{N}_i$:
$$
\mathrm{owner}_i(c) = \arg\min_{j \in \{i\} \cup \mathcal{N}_i} \|p_j - c\|
$$
Owned cells: $\mathcal{M}_i = \{c \in \mathcal{U}_t : \mathrm{owner}_i(c) = i\}$.

- **Target** (lines 233-246), two variants by `target_strategy`:
$$
t_i = \begin{cases}
\frac{1}{|\mathcal{M}_i|} \sum_{c \in \mathcal{M}_i} c & \text{(centroid — Lloyd-flow)} \\[4pt]
\arg\min_{c \in \mathcal{M}_i} \|p_i - c\| & \text{(nearest — default)}
\end{cases}
$$
Fallback to global nearest uncov if $\mathcal{M}_i = \emptyset$.

- **Motion law:** same as PF.

## 3.3 MARL — `controllers/marl_env.py:186-208` (training reward) + `controllers/marl.py` (inference)

**The only algorithm whose internal objective includes the full energy-aware terms.** PPO trains a decentralized policy $\pi_\theta(a \mid o)$ to maximize the expected discounted return $\mathbb{E}[\sum_t \gamma^t r_t]$.

**Per-step reward** (lines 186-208, summed across all drones):
$$
r_t \;=\; \underbrace{\alpha_{\text{cov}} \cdot \Delta\mathrm{covered}_t}_{\text{coverage gain}} \;-\; \underbrace{0.01}_{\text{time penalty}} \;-\; \underbrace{\alpha_{\text{ov}} \cdot \Delta\mathrm{overlap}_t}_{\text{(optional)}} \;-\; \underbrace{\alpha_{\text{wast}} \cdot \Delta\mathrm{wasted}_t}_{\text{(optional)}} \;-\; \underbrace{\alpha_{\text{eng}} \cdot \Delta E_t}_{\text{(optional)}} \;+\; \underbrace{\alpha_{\text{done}} \mathbb{1}[\mathrm{done}_t]}_{\text{completion bonus}} \;-\; \underbrace{\alpha_{\text{dep}} \mathbb{1}[\mathrm{depleted}_t]}_{\text{depletion penalty}}
$$

```python
# marl_env.py:187-208
covered_now = int(env.covered.sum())
delta_cells = covered_now - self._covered_count_prev
reward = float(delta_cells) - 0.01  # coverage gain - time penalty

if self.overlap_penalty_per_m2 != 0.0:
    d_overlap = max(0.0, overlap_now - self._overlap_m2_prev)
    reward -= self.overlap_penalty_per_m2 * d_overlap
if self.wasted_visit_penalty != 0.0:
    d_wasted = max(0, wasted_now - self._wasted_visits_prev)
    reward -= self.wasted_visit_penalty * float(d_wasted)
if self.energy_penalty_per_kj != 0.0:
    d_energy_j = max(0.0, self._battery_total_prev - battery_now)
    reward -= self.energy_penalty_per_kj * (d_energy_j / 1000.0)
```

**PPO clipped surrogate objective** (Schulman et al. 2017):
$$
L^{\text{CLIP}}(\theta) = \mathbb{E}_t\!\left[\min\left( \rho_t(\theta) \hat{A}_t,\; \mathrm{clip}(\rho_t(\theta),\, 1 - \epsilon,\, 1 + \epsilon) \hat{A}_t \right)\right]
$$
with $\rho_t(\theta) = \pi_\theta(a_t \mid o_t) / \pi_{\theta_{\text{old}}}(a_t \mid o_t)$ and $\hat{A}_t$ the generalized-advantage estimate.

---

# Summary table — what each algorithm actually optimizes

| Algorithm | Target rule | Has explicit fitness? | Energy in objective? |
|---|---|---|---|
| Boustrophedon | next non-stuck waypoint in lawnmower plan | no | no |
| Spiral | next non-stuck waypoint in spiral plan | no | no |
| VoronoiPartition | $\arg\min_{c \in R_i \cap \mathcal{U}} \|p_i - c\|$ | no (distance proxy) | no |
| GridDecomposition | $\arg\min_{c \in b_k \cap \mathcal{U}} \|p_i - c\|$ | no (distance proxy) | no |
| STC | next uncov in BFS walk | no | no |
| PSO | weighted pull toward $\mathrm{pbest}_i, \mathrm{gbest}$ | no (distance proxy via target) | no |
| **GA** | persistent target evolved by mutation/crossover | **yes — `count(uncov ∈ B(t, r_fit))`** | no |
| **SA** | persistent target evolved by Metropolis perturb | **yes — same as GA** | no |
| **ACO** | sample $P(c) \propto \tau(c)^\alpha \eta(c)^\beta$ | implicit (pheromone field) | no |
| GWO | weighted pull toward α/β/δ leaders' targets | no (distance proxy) | no |
| PF | $\arg\min_{c \in \mathcal{U}} \|p_i - c\|$ | no (distance proxy) | no |
| Consensus | centroid or nearest in local Voronoi region | no (distance proxy) | no |
| **MARL** | $\pi_\theta(a \mid o)$ — learned policy | **yes — full PPO reward** | **yes — `energy_penalty_per_kj`** |

**Only 4 of 13 algorithms have an explicit fitness function** (GA, SA, ACO via pheromones, MARL).

**Only 1 algorithm (MARL) has energy in its internal objective.** Every other controller relies on the environment's physics (battery drains automatically with motion) to indirectly penalize energy — the controller itself doesn't *see* energy when choosing actions.

This is the central design tension the report can highlight: do simple geometric/distance-based proxies match a policy that has the full multi-criteria objective in its training loop? The benchmark answers this empirically: **mostly yes** — SA, GA, ACO, Voronoi, STC all beat MARL on the composite score under default reward shaping. The implicit energy penalty (less motion = less battery drain = more reach) appears to be sufficient for the partition-based methods.
