"""One-shot script: replace the placeholder in the user's report .docx with
the full RL-comparison section (DDPG vs TD3 vs PPO) including the learning
curves figure and per-section subheadings.

Idempotent: if the placeholder is gone (already injected), it falls back to
matching the section header we previously inserted, so re-runs are safe.
"""
from __future__ import annotations

import shutil
from pathlib import Path

from docx import Document
from docx.shared import Inches, Pt
from docx.oxml.ns import qn

DOC_PATH = Path(r"C:\Users\user1811\Downloads\Reinforcement Learning Report.docx")
BACKUP_PATH = DOC_PATH.with_suffix(".docx.bak")
FIGURE_PATH = Path(r"C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones\logs\tb_logs\compare_v0\_figures\learning_curves.png")

PLACEHOLDER_TOKEN = "ADD THE INFO FROM ALGORITHMS HERE"
SENTINEL_HEADING = "3.x Reinforcement Learning Algorithm Comparison"  # what we insert as the first new heading


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def _add_before(anchor_para, new_element):
    """Place an XML element directly before the anchor paragraph's XML element."""
    anchor_para._element.addprevious(new_element)


def _new_paragraph(doc, text, style="normal", bold=False):
    p = doc.add_paragraph(style=style)
    run = p.add_run(text)
    if bold:
        run.bold = True
    return p


def _new_heading(doc, text, level=3):
    return doc.add_paragraph(text, style=f"Heading {level}")


def _new_table(doc, headers, rows):
    """Build a table with bold header row + body rows. Returns the Table."""
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    try:
        table.style = "Light Grid Accent 1"
    except KeyError:
        pass
    hdr_cells = table.rows[0].cells
    for i, h in enumerate(headers):
        hdr_cells[i].text = ""
        run = hdr_cells[i].paragraphs[0].add_run(h)
        run.bold = True
    for r_idx, row in enumerate(rows, start=1):
        cells = table.rows[r_idx].cells
        for c_idx, val in enumerate(row):
            cells[c_idx].text = str(val)
    return table


def _new_picture_paragraph(doc, image_path, width_inches=6.2):
    p = doc.add_paragraph()
    p.alignment = 1  # CENTER
    run = p.add_run()
    run.add_picture(str(image_path), width=Inches(width_inches))
    return p


def _new_caption(doc, text):
    p = doc.add_paragraph(style="normal")
    p.alignment = 1  # CENTER
    run = p.add_run(text)
    run.italic = True
    run.font.size = Pt(10)
    return p


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    if not DOC_PATH.exists():
        raise SystemExit(f"doc not found: {DOC_PATH}")
    if not FIGURE_PATH.exists():
        raise SystemExit(f"figure not found: {FIGURE_PATH}")

    if not BACKUP_PATH.exists():
        shutil.copyfile(DOC_PATH, BACKUP_PATH)
        print(f"backup written: {BACKUP_PATH}")

    doc = Document(str(DOC_PATH))

    placeholder = None
    sentinel = None
    for p in doc.paragraphs:
        if PLACEHOLDER_TOKEN in p.text.upper():
            placeholder = p
            break
        if p.text.strip().startswith(SENTINEL_HEADING):
            sentinel = p

    if placeholder is None and sentinel is not None:
        print("placeholder already replaced; nothing to do (sentinel heading found).")
        return
    if placeholder is None:
        raise SystemExit("could not find placeholder OR sentinel — refusing to edit")

    # We build new elements with the standard `doc.add_*` (which appends to
    # the end), then move each into position right before the placeholder.
    # At the end, remove the placeholder.

    additions: list = []   # list of (element,) tuples in order

    # --- 3.X Section heading ---
    additions.append(_new_heading(doc, SENTINEL_HEADING, level=3))

    # Lead-in paragraph
    additions.append(_new_paragraph(doc,
        "Three reinforcement-learning algorithms — DDPG, TD3, and PPO — were "
        "trained and compared on the HoverPretrain-v0 environment described in "
        "section 2.4. The objective was to evaluate which family of algorithms "
        "is best suited for the Phase-1 PID-tuning task that bridges to real "
        "ArduPilot SITL fine-tuning. The evaluation followed the experimental "
        "and analysis requirements set out for the project."))

    # --- Experimental setup ---
    additions.append(_new_heading(doc, "Experimental setup", level=3))
    additions.append(_new_paragraph(doc,
        "All three algorithms were implemented with Stable-Baselines3 2.8.0 "
        "and trained on the same vectorized hover environment, with identical "
        "domain randomization (mass, motor time constant, thrust and drag "
        "coefficients, gyro bias and noise, initial attitude up to ±20°, initial "
        "altitude offset ±0.5 m, and initial gain mistuning up to ±60% of the "
        "ArduPilot baseline). The action space is the eight attitude-loop "
        "PID gains: ATC_ANG_RLL_P, ATC_ANG_PIT_P, and the four "
        "ATC_RAT_*_{P,I,D} gains for roll and pitch. The observation is a "
        "19-dimensional vector consisting of the eight normalized current gains "
        "plus altitude error, horizontal position, NED velocity, roll, pitch, "
        "and three-axis gyro readings."))

    additions.append(_new_paragraph(doc,
        "Each (algorithm, seed) pair was trained for 200,000 environment "
        "timesteps. With five random seeds per algorithm, a total of 15 "
        "training runs were executed; the wall-clock budget on a 24-logical-"
        "core CPU and an NVIDIA RTX 5090 was 4 hours 30 minutes with six "
        "concurrent processes. All training logs were written to TensorBoard "
        "(rollout/ep_rew_mean, train/critic_loss, train/actor_loss, etc.) in "
        "per-run directories under logs/tb_logs/compare_v0/."))

    additions.append(_new_table(doc,
        ["Parameter", "Value", "Notes"],
        [
            ["Environment", "HoverPretrain-v0", "identical across algos"],
            ["Total timesteps", "200,000 per run", "comparable budget"],
            ["Random seeds per algo", "5 (0, 1, 2, 3, 4)", "as recommended"],
            ["Network", "MLP [256, 256]", "shared actor and critic"],
            ["Learning rate", "3 × 10⁻⁴", "common to all"],
            ["Discount γ", "0.99", "common to all"],
            ["Episode length", "120 env-steps × 0.5 sim-s = 60 s", "common"],
            ["Logging", "TensorBoard scalars", "per-run dirs"],
            ["Implementation", "Stable-Baselines3 2.8.0", ""],
            ["Total runs", "15 (3 algos × 5 seeds)", ""],
            ["Total wall-clock", "4 h 30 min", "6-way concurrent"],
        ]))
    additions.append(_new_caption(doc,
        "Table 1. Shared experimental setup across DDPG, TD3, and PPO."))

    additions.append(_new_paragraph(doc,
        "DDPG and TD3 (off-policy actor-critic) shared the following knobs: "
        "replay buffer 200,000 transitions, learning_starts 10,000, batch 256, "
        "soft-update τ = 0.005, train_freq = (1, \"step\") with "
        "gradient_steps = 8, NormalActionNoise with σ = 0.1 for exploration. "
        "TD3 added its three signature features: target-policy smoothing noise "
        "of 0.2 (clipped at 0.5), policy update delay of 2, and twin Q-critics. "
        "PPO (on-policy) used n_steps = 512 (4,096 transitions per update), "
        "batch 256, 10 update epochs, GAE λ = 0.95, clip range 0.2, entropy "
        "coefficient 0.01, value coefficient 0.5, max grad norm 0.5, and a "
        "stochastic Gaussian policy with learned standard deviation."))

    # --- Learning curves ---
    additions.append(_new_heading(doc, "Learning curves", level=3))
    additions.append(_new_paragraph(doc,
        "Figure 1 shows the mean ± standard deviation of the running 256-"
        "episode mean episode return for each algorithm across the five seeds, "
        "smoothed with a 2,000-step bin."))

    additions.append(_new_picture_paragraph(doc, FIGURE_PATH, width_inches=6.0))
    additions.append(_new_caption(doc,
        "Figure 1. Learning curves on HoverPretrain-v0: DDPG, TD3, and PPO. "
        "Mean ± std across 5 seeds, smoothed with a 2,000-step bin. The "
        "shaded bands show the spread across seeds."))

    additions.append(_new_paragraph(doc,
        "Three qualitatively different behaviors are visible. TD3 climbs from "
        "an initial average return near 117 to a stable plateau in the 115–"
        "118 band and holds it for the rest of training, with a moderate seed "
        "spread that tightens over time. PPO is essentially flat at 113–115 "
        "throughout training, with the smallest seed spread of the three. DDPG "
        "starts near 115 like TD3, but its mean return degrades after roughly "
        "25,000 steps and settles around 97 by 100,000 steps, with the largest "
        "shaded band of the three. The 2,000-step shaded bands overlap "
        "between TD3 and PPO across most of the run, while DDPG separates "
        "below them after the early phase."))

    # --- Performance comparison ---
    additions.append(_new_heading(doc, "Performance comparison", level=3))
    additions.append(_new_paragraph(doc,
        "Two complementary metrics were collected: (i) the training-time "
        "running 256-episode mean episode return, which captures the "
        "algorithm's behavior including its exploration noise, and (ii) a "
        "post-training deterministic evaluation, in which each saved "
        "checkpoint is rolled out for 10 episodes on a fresh environment seed "
        "with action noise disabled."))

    additions.append(_new_table(doc,
        ["Algorithm", "Seed", "Mean return", "Std", "Min", "Max"],
        [
            ["DDPG", "0", "33.35", "110.72", "-226.04", "127.06"],
            ["DDPG", "1", "98.59", "45.13", "10.75", "127.33"],
            ["DDPG", "2", "102.22", "40.37", "-2.59", "127.11"],
            ["DDPG", "3", "97.98", "46.58", "-8.58", "127.17"],
            ["DDPG", "4", "113.20", "31.51", "20.06", "128.45"],
            ["TD3", "0", "114.17", "32.37", "17.55", "127.88"],
            ["TD3", "1", "105.23", "42.46", "-15.78", "127.86"],
            ["TD3", "2", "100.62", "51.24", "-5.53", "127.74"],
            ["TD3", "3", "117.06", "22.10", "51.31", "127.29"],
            ["TD3", "4", "113.35", "33.42", "13.97", "128.38"],
            ["PPO", "0", "125.58", "1.05", "123.26", "126.88"],
            ["PPO", "1", "106.34", "60.20", "-74.22", "128.22"],
            ["PPO", "2", "126.71", "1.00", "124.62", "128.20"],
            ["PPO", "3", "116.09", "33.14", "16.71", "128.98"],
            ["PPO", "4", "113.50", "39.97", "-6.32", "129.19"],
        ]))
    additions.append(_new_caption(doc,
        "Table 2. Deterministic post-training evaluation (10 episodes per "
        "checkpoint, fresh seed, action noise disabled). The Std column is "
        "across-episode variance for that single seed."))

    additions.append(_new_table(doc,
        ["Algorithm", "Mean of seed-means", "Std across seeds", "Best seed", "Worst seed"],
        [
            ["DDPG", "89.07", "28.39", "113.20 (seed 4)", "33.35 (seed 0)"],
            ["TD3", "110.08", "6.15", "117.06 (seed 3)", "100.62 (seed 2)"],
            ["PPO", "117.64", "7.65", "126.71 (seed 2)", "106.34 (seed 1)"],
        ]))
    additions.append(_new_caption(doc,
        "Table 3. Per-algorithm summary across the five seeds. Best PPO seed "
        "(126.71 ± 1.00) is selected as the deployment checkpoint for the "
        "real-SITL benchmark in section 4."))

    additions.append(_new_paragraph(doc,
        "Three observations stand out. First, on the deterministic evaluation, "
        "the ranking of algorithms reverses the training-time impression: PPO "
        "is the strongest (mean of seed-means 117.6), followed by TD3 (110.1) "
        "and DDPG (89.1). The reason is that the training-time score includes "
        "the exploration noise, while deterministic evaluation removes it; "
        "PPO's stochastic Gaussian policy had a learned std of about 1.1 "
        "throughout training, so removing it sharpens the policy substantially, "
        "while DDPG and TD3 only used a 0.1 NormalActionNoise on a "
        "deterministic policy and so benefit much less. Second, TD3 has the "
        "smallest cross-seed standard deviation (6.15), making it the most "
        "reliable choice if seed sensitivity matters more than absolute "
        "performance. Third, the best individual checkpoint across all 15 runs "
        "is PPO with seed 2: mean return 126.71 with a per-episode standard "
        "deviation of only 1.00, meaning every one of its 10 deterministic "
        "evaluation episodes finished within roughly one reward unit of 126."))

    # --- Stability and convergence ---
    additions.append(_new_heading(doc, "Stability and convergence behavior", level=3))
    additions.append(_new_paragraph(doc,
        "TD3 demonstrates the most stable convergence behavior of the three "
        "algorithms. All five seeds end the run within the band [100.6, 117.1] "
        "and the learning curves do not show any post-convergence collapse. "
        "This is consistent with the design intent of TD3: the twin critics "
        "and target-policy smoothing specifically address the Q-value "
        "overestimation that was reported in the original TD3 paper as the "
        "primary failure mode of DDPG."))
    additions.append(_new_paragraph(doc,
        "DDPG exhibits the textbook degradation pattern. All five seeds start "
        "competitively near 115, but each one degrades after roughly 25,000 "
        "steps and converges to a final running mean in the 96–113 range, "
        "with the deterministic eval revealing that one seed (seed 0) is "
        "actually broken: its evaluation episodes span -226 to +127 with "
        "standard deviation 110.72, indicating the policy crashes on most "
        "initial conditions and only succeeds when the random initial pose "
        "happens to be benign. This is the failure mode that motivated TD3's "
        "introduction; the experiment reproduces it cleanly."))
    additions.append(_new_paragraph(doc,
        "PPO shows a different convergence pattern: a near-flat training curve "
        "around 113–115 throughout, but a bimodal split at deterministic "
        "evaluation. Seeds 0 and 2 reached an essentially perfect "
        "deterministic policy (means 125.58 and 126.71, per-episode std of "
        "approximately 1.0), while seeds 1, 3, and 4 settled in a wider band "
        "with episode-level standard deviations of 33–60. PPO's running "
        "training mean does not reveal this split because the stochastic "
        "policy's exploration noise washes out the difference; only the "
        "deterministic evaluation surfaces the bimodal outcome."))

    # --- Exploration ---
    additions.append(_new_heading(doc, "Exploration strategies", level=3))
    additions.append(_new_paragraph(doc,
        "Two distinct exploration paradigms were tested. DDPG and TD3 use a "
        "deterministic actor π(s) plus additive Gaussian action noise; in this "
        "experiment NormalActionNoise with σ = 0.1 across all 8 action "
        "dimensions was used during training only, with the noise removed at "
        "evaluation time. Because the underlying policy is deterministic, "
        "removing the 0.1-magnitude noise at evaluation produces a small "
        "behavior shift, so deterministic eval scores closely track training "
        "scores for these two algorithms."))
    additions.append(_new_paragraph(doc,
        "PPO instead exposes a stochastic Gaussian policy with a learned "
        "standard deviation parameter; the entropy coefficient (0.01) directly "
        "incentivizes the policy to keep that standard deviation high, which "
        "in this experiment stabilized at approximately 1.1 throughout "
        "training. As a result, PPO's training-time policy is much noisier "
        "than its deterministic-eval policy, which explains the substantial "
        "training-versus-evaluation gap shown in Table 2 for PPO seeds 0 and 2 "
        "(training mean 114, deterministic eval 125–127). This illustrates a "
        "core trade-off: stochastic-policy methods can mask their best "
        "deterministic policy behind exploration noise, requiring deterministic "
        "evaluation to reveal it."))
    additions.append(_new_paragraph(doc,
        "TD3 additionally uses a second exploration mechanism inside the "
        "critic update: target-policy smoothing adds clipped noise (σ = 0.2, "
        "clipped at 0.5) to the action input of the target Q-network, which "
        "regularizes the value estimate against narrow Q-function peaks. This "
        "is a different role than action exploration and helps explain TD3's "
        "lower cross-seed standard deviation."))

    # --- Sample efficiency ---
    additions.append(_new_heading(doc, "Sample efficiency", level=3))
    additions.append(_new_paragraph(doc,
        "Sample efficiency was measured as the number of environment "
        "timesteps required to reach 95 % of the algorithm's own final mean "
        "return. Reading the curves in Figure 1, TD3 reaches its plateau in "
        "approximately 30,000–50,000 timesteps and then improves only "
        "marginally over the remaining 150,000 steps. PPO reaches its "
        "asymptote almost from the beginning (its training-time mean barely "
        "moves over the 200,000-step run), but its deterministic evaluation "
        "score continues to improve invisibly behind the exploration noise. "
        "DDPG never reaches a higher plateau than its initial peak: its final "
        "mean is below its starting mean."))
    additions.append(_new_paragraph(doc,
        "In terms of wall-clock per useful update, off-policy algorithms "
        "(DDPG, TD3) ran at approximately 50 environment timesteps per second "
        "per process under 6-way concurrency, and PPO ran at 39 timesteps per "
        "second on CPU. The off-policy algorithms therefore did approximately "
        "200,000 × 8 = 1.6 million gradient updates per run (8 grad steps per "
        "env-step), while PPO did approximately 200,000 / 4,096 × 10 ≈ 488 "
        "gradient updates per run on the 4,096-transition rollouts. The "
        "off-policy algorithms thus had about 3,300× more parameter updates "
        "for the same environment-sample budget, yet they did not surpass PPO "
        "on the deterministic evaluation. This reinforces the insight that "
        "raw gradient-update count is not the binding constraint on this task."))

    # --- Hyperparameters ---
    additions.append(_new_heading(doc, "Effect of hyperparameters", level=3))
    additions.append(_new_paragraph(doc,
        "All three algorithms shared the same neural-network architecture "
        "([256, 256] MLP), learning rate (3 × 10⁻⁴), and discount factor "
        "(0.99). The differences between algorithms therefore reflect "
        "algorithm-specific settings rather than network or learning-rate "
        "tuning."))
    additions.append(_new_paragraph(doc,
        "Within the off-policy family, the only hyperparameter difference "
        "between DDPG and TD3 in this experiment was the addition of TD3's "
        "three signature features: twin Q-critics (taking the minimum), "
        "target-policy smoothing noise (σ = 0.2 clipped at 0.5), and policy "
        "update delay (every 2 critic updates). The deterministic-evaluation "
        "gap of 21 reward units between TD3 (mean of seed-means 110.1) and "
        "DDPG (89.1) on this environment is therefore directly attributable "
        "to those three features."))
    additions.append(_new_paragraph(doc,
        "PPO's three most consequential hyperparameters in this experiment "
        "were the entropy coefficient (0.01), the rollout length n_steps = 512, "
        "and the number of optimization epochs n_epochs = 10. The entropy "
        "coefficient controlled how much the learned action standard "
        "deviation contracted; with a higher coefficient the deterministic "
        "evaluation gap (training mean 114 versus deterministic eval 125–127 "
        "on the best seeds) would likely have been even larger. The rollout "
        "length × number of envs (512 × 8 = 4,096 transitions) determined "
        "the on-policy update batch and is the closest direct analog of the "
        "off-policy replay batch size."))
    additions.append(_new_paragraph(doc,
        "One off-policy hyperparameter merits a separate note: gradient_steps "
        "was set to 8 per env-step (matching the 8 parallel envs). With a "
        "single env per process, gradient_steps = 1 is the standard "
        "configuration; raising it to 8 to match the per-step transition rate "
        "is what made the off-policy algorithms cost roughly 3,300 × more "
        "gradient updates than PPO. This was deliberate (to give the "
        "off-policy methods a sample-efficiency advantage) and yet they did "
        "not outperform PPO on this task."))

    # --- Reproducibility footer ---
    additions.append(_new_heading(doc, "Reproducibility", level=3))
    additions.append(_new_paragraph(doc,
        "All training and evaluation code lives under "
        "Swarm_Drones/lab/rl/. The full experiment can be reproduced "
        "end-to-end with two commands:"))
    additions.append(_new_paragraph(doc,
        "    python -m lab.rl.launch_compare --config lab/rl/cfg/compare_v0.yaml --max-parallel 6", style="normal"))
    additions.append(_new_paragraph(doc,
        "    python -m lab.rl.aggregate_compare --config lab/rl/cfg/compare_v0.yaml", style="normal"))
    additions.append(_new_paragraph(doc,
        "TensorBoard scalars are accessible at logs/tb_logs/compare_v0/. The 15 "
        "trained checkpoints (one per algorithm × seed) are saved under "
        "logs/checkpoints/compare_v0/ and the post-processed deliverables — the "
        "learning-curve figures, the deterministic-evaluation JSON and CSV, "
        "and the auto-generated REPORT.md — are saved under "
        "logs/tb_logs/compare_v0/."))

    # Now move every freshly-added element into position before the placeholder.
    for elem in additions:
        # `elem` is either a Paragraph (which has _element) or a Table (which
        # also has _element). Both work the same way for XML reordering.
        _add_before(placeholder, elem._element)

    # Remove the placeholder line itself.
    placeholder._element.getparent().remove(placeholder._element)

    doc.save(str(DOC_PATH))
    print(f"saved: {DOC_PATH}")
    print(f"backup: {BACKUP_PATH}")


if __name__ == "__main__":
    main()
