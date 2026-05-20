"""
Apply BO-found best configs to each controller's *Config dataclass defaults.

Reads `outputs/bo/summary.json` (or `outputs/bo/<controller>.json`) and
edits `controllers/<name>.py` in place, replacing the default value of each
BO-tuned field with the winner. Prints a diff-like preview before applying
unless `--apply` is set.

The previous default value is preserved as a `# (was 1.5)` end-of-line
comment so the prior grid-search-tuned baseline isn't lost.

Field replacement is regex-based — assumes the standard dataclass form
`    field_name: <type> = <value>` (one per line, 4-space indent inside the
@dataclass class body). This matches every existing *Config dataclass in
this project.

Usage:
    python3 tools/tuning/bo_apply.py                   # preview only (no write)
    python3 tools/tuning/bo_apply.py --apply           # apply all controllers
    python3 tools/tuning/bo_apply.py --apply --only stc,pso
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))


# Maps controller name → (config-source filename, config class name in source)
CONFIG_LOCATIONS = {
    "boustrophedon": ("controllers/boustrophedon.py",     "BoustrophedonConfig"),
    "spiral":        ("controllers/spiral.py",            "SpiralConfig"),
    "voronoi":       ("controllers/voronoi_partition.py", "VoronoiPartitionConfig"),
    "grid_decomp":   ("controllers/grid_decomposition.py","GridDecompositionConfig"),
    "stc":           ("controllers/stc.py",               "STCConfig"),
    "pso":           ("controllers/pso.py",               "PSOConfig"),
    "ga":            ("controllers/ga.py",                "GAConfig"),
    "sa":            ("controllers/sa.py",                "SAConfig"),
    "aco":           ("controllers/aco.py",               "ACOConfig"),
    "gwo":           ("controllers/gwo.py",               "GWOConfig"),
    "pf":            ("controllers/potential_fields.py",  "PFConfig"),
    "consensus":     ("controllers/consensus.py",         "ConsensusConfig"),
}


def _format_value(v) -> str:
    """Render a python literal the way a dataclass default should look."""
    if isinstance(v, bool):
        return "True" if v else "False"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        # 4 sig figs is plenty for these tuning knobs — matches how the
        # original grid-search comments showed them.
        return f"{v:.4f}".rstrip("0").rstrip(".") or "0"
    return repr(v)


def patch_one(controller: str, params: dict, *, apply: bool) -> tuple[str, list[str]]:
    """Patch one controller's Config dataclass. Returns (status, edits).
    `status` is 'patched' / 'preview' / 'no-changes'. `edits` is a list of
    human-readable lines describing each field replacement."""
    src_path, cls_name = CONFIG_LOCATIONS[controller]
    src = Path(src_path)
    if not src.exists():
        return "missing-source", [f"{src_path} not found"]
    content = src.read_text()

    # Narrow to the body of the target dataclass.
    cls_re = re.compile(
        rf"(@dataclass\s*\nclass\s+{cls_name}\b.*?:.*?\n)(?P<body>.*?)(?=\n(?:@dataclass|class\s+\w+|def\s+\w+)|\Z)",
        re.DOTALL,
    )
    m = cls_re.search(content)
    if not m:
        return "missing-class", [f"could not locate @dataclass {cls_name} in {src_path}"]

    body = m.group("body")
    new_body = body
    edits = []
    for field_name, new_value in params.items():
        new_val_str = _format_value(new_value)
        # Match exactly one `    field_name: <type> = <value>` line; capture
        # trailing comment so we can append `# (was X)`.
        field_re = re.compile(
            rf"^(    {re.escape(field_name)}:\s*\w+\s*=\s*)([0-9.eE+-]+|True|False|'[^']*'|\"[^\"]*\")(\s*(?:#.*)?)$",
            re.MULTILINE,
        )
        m2 = field_re.search(new_body)
        if not m2:
            edits.append(f"  ⚠ {field_name}: not found in {cls_name} body (skipped)")
            continue
        old_val_str = m2.group(2)
        if old_val_str == new_val_str:
            edits.append(f"  · {field_name}: {old_val_str} (unchanged)")
            continue
        trailing = m2.group(3).strip()
        # Preserve / extend the trailing comment with a note that it was retuned.
        if trailing:
            if "(was " in trailing:
                comment = trailing  # already has history; don't pile up
            else:
                comment = f"{trailing}  (BO; was {old_val_str})"
        else:
            comment = f"# BO-tuned (was {old_val_str})"
        new_line = f"{m2.group(1)}{new_val_str}   {comment}"
        new_body = field_re.sub(new_line, new_body, count=1)
        edits.append(f"  ✓ {field_name}: {old_val_str} → {new_val_str}")

    if new_body == body:
        return "no-changes", edits

    new_content = content[:m.start("body")] + new_body + content[m.end("body"):]
    if apply:
        src.write_text(new_content)
        return "patched", edits
    return "preview", edits


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=str, default="outputs/bo/summary.json")
    parser.add_argument("--apply", action="store_true",
                        help="actually write the patched files (default: preview)")
    parser.add_argument("--only", type=str, default=None,
                        help="comma-separated controller names to apply (default: all)")
    args = parser.parse_args()

    summary = json.loads(Path(args.summary).read_text())
    targets = list(summary)
    if args.only:
        wanted = [n.strip() for n in args.only.split(",") if n.strip()]
        unknown = set(wanted) - set(targets)
        if unknown:
            raise SystemExit(f"unknown controllers in --only: {unknown} "
                             f"(summary has: {sorted(targets)})")
        targets = wanted

    print(f"[bo_apply] mode: {'APPLY' if args.apply else 'PREVIEW'}")
    print(f"[bo_apply] summary: {args.summary}")
    print(f"[bo_apply] controllers: {targets}")
    print()

    for name in targets:
        info = summary[name]
        params = info["best_params"]
        print(f"=== {name}  (best score {info['best_value']:+.4f}) ===")
        status, edits = patch_one(name, params, apply=args.apply)
        for line in edits:
            print(line)
        print(f"  → {status}")
        print()


if __name__ == "__main__":
    main()
