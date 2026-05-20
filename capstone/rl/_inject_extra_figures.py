"""Insert the five additional figures into the report at relevant subsections.

Figures are placed RIGHT AFTER the last paragraph of each named subsection,
so they sit inside the section that discusses them.

Idempotent: each figure is only inserted if a unique sentinel caption isn't
already present.
"""
from __future__ import annotations

import shutil
from pathlib import Path

from docx import Document
from docx.shared import Inches, Pt
from docx.oxml.ns import qn

DOC_PATH = Path(r"C:\Users\user1811\Downloads\Reinforcement Learning Report.docx")
BACKUP_PATH = DOC_PATH.with_suffix(".docx.bak2")
FIG_DIR = Path(r"C:\Users\user1811\Desktop\armen-capstone\Swarm_Drones\logs\tb_logs\compare_v0\_figures")


# Each figure: (subsection-heading text we anchor to, image filename, width inches, caption text)
FIGURES = [
    # Performance comparison gets THREE figures.
    ("Performance comparison", "eval_box.png", 6.0,
     "Figure 2. Distribution of all 50 deterministic-evaluation episodes per algorithm "
     "(10 episodes × 5 seeds). The dashed line marks the ideal hover return (~125). "
     "DDPG shows the widest spread with episodes as low as -226; TD3 has a tight "
     "central band with a few low outliers; PPO is the most concentrated near the ideal."),
    ("Performance comparison", "eval_bars.png", 5.0,
     "Figure 3. Cross-seed mean ± standard deviation of the per-seed mean returns. "
     "PPO leads in mean (117.6); TD3 has the smallest cross-seed variance (±6.1); "
     "DDPG is both worst and most variable across seeds (±28.4)."),
    ("Performance comparison", "eval_seeds.png", 6.0,
     "Figure 4. Per-seed mean returns (each marker = one seed; error bars = "
     "per-seed standard deviation across 10 evaluation episodes). DDPG seed 0 "
     "is the visible outlier (mean 33.4, std 110.7), reflecting a policy that "
     "crashes on most random initial poses."),
    # Stability and convergence — TB critic loss
    ("Stability and convergence behavior", "tb_critic_loss.png", 6.5,
     "Figure 5. TensorBoard-derived critic-loss curves. Mean ± std across 5 seeds, "
     "log y-axis. DDPG's critic loss climbs about two orders of magnitude during "
     "training (Q-value overestimation), while TD3's stays bounded at ~10. This "
     "is the textbook failure-mode TD3 was designed to fix; the experiment "
     "reproduces it cleanly."),
    # Exploration — PPO action std + entropy
    ("Exploration strategies", "tb_ppo_exploration.png", 6.5,
     "Figure 6. TensorBoard-derived PPO exploration trajectory. Left: the learned "
     "action standard deviation rises from 1.00 to ~1.20 over training (entropy "
     "bonus dominates the policy gradient); right: entropy loss becomes more "
     "negative (entropy increases). This is the opposite of the usual PPO "
     "expectation that std contracts during convergence — and explains why "
     "PPO's deterministic-eval policy is much sharper than its training-mean "
     "policy: removing a learned σ ≈ 1.2 sharpens behavior dramatically."),
]


def _new_picture_paragraph(doc, image_path: Path, width_inches: float):
    p = doc.add_paragraph()
    p.alignment = 1
    run = p.add_run()
    run.add_picture(str(image_path), width=Inches(width_inches))
    return p


def _new_caption(doc, text: str):
    p = doc.add_paragraph(style="normal")
    p.alignment = 1
    run = p.add_run(text)
    run.italic = True
    run.font.size = Pt(10)
    return p


def _last_paragraph_of_subsection(doc, subsection_text: str):
    """Find the last paragraph that belongs to the named Heading-3 subsection.
    "Belongs" means: starting from the heading, walk forward until the next
    Heading-1/2/3 appears or end of doc; the previous paragraph is the anchor.

    Returns the paragraph element to insert AFTER (we'll use addnext)."""
    paragraphs = doc.paragraphs
    start_idx = None
    for i, p in enumerate(paragraphs):
        if p.style.name == "Heading 3" and p.text.strip().startswith(subsection_text):
            start_idx = i
            break
    if start_idx is None:
        return None
    end_idx = len(paragraphs) - 1
    for j in range(start_idx + 1, len(paragraphs)):
        s = paragraphs[j].style.name
        if s.startswith("Heading"):
            end_idx = j - 1
            break
    return paragraphs[end_idx]


def main():
    if not DOC_PATH.exists():
        raise SystemExit(f"doc not found: {DOC_PATH}")
    if not BACKUP_PATH.exists():
        shutil.copyfile(DOC_PATH, BACKUP_PATH)
        print(f"backup written: {BACKUP_PATH}")

    doc = Document(str(DOC_PATH))

    # Idempotency check — if any of our new captions are already present,
    # do nothing.
    full_text = "\n".join(p.text for p in doc.paragraphs)
    if "Figure 2." in full_text and "Figure 5." in full_text:
        print("figures already injected; nothing to do.")
        return

    # We insert each figure RIGHT AFTER the last paragraph of its subsection.
    # We process in reverse order PER SUBSECTION so insertions don't shift the
    # paragraph index of later anchors. But because different figures may go
    # into different subsections, we group them by subsection first.
    by_subsection: dict[str, list] = {}
    for sub, fn, w, cap in FIGURES:
        by_subsection.setdefault(sub, []).append((fn, w, cap))

    inserted = 0
    for subsection, items in by_subsection.items():
        anchor = _last_paragraph_of_subsection(doc, subsection)
        if anchor is None:
            print(f"WARN: subsection '{subsection}' not found; skipping its figures")
            continue
        # Build all elements for this subsection (figure + caption pairs),
        # then insert them in order RIGHT AFTER the anchor. To preserve order
        # we insert in reverse using addnext — but the easier path is:
        # insert a marker, build all elements, then chain-insert.

        new_elements = []
        for fn, w, cap in items:
            img_path = FIG_DIR / fn
            if not img_path.exists():
                print(f"WARN: figure not found: {img_path}")
                continue
            new_elements.append(_new_picture_paragraph(doc, img_path, w)._element)
            new_elements.append(_new_caption(doc, cap)._element)

        # Insert all elements after the anchor, in order.
        # addnext puts the new element immediately after the target — so to
        # preserve order we walk the list in REVERSE and addnext each one.
        for el in reversed(new_elements):
            anchor._element.addnext(el)
        inserted += len(items)
        print(f"inserted {len(items)} figure(s) into '{subsection}'")

    doc.save(str(DOC_PATH))
    print(f"saved: {DOC_PATH}  ({inserted} figures)")


if __name__ == "__main__":
    main()
