#!/usr/bin/env python3
# Copyright 2026 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Offline separation report for relocalization candidate diagnostics.

Joins each run's ``candidates.csv`` against the ground truth recorded in the
same directory's ``cell.json`` and prints, per registration stage, how each
evidence field is distributed over CORRECT and WRONG candidates.

Ground truth classifies rows for THIS REPORT ONLY. It never participates in the
online score, so a threshold picked here is still selected from evidence the
node can observe at run time.

Usage:
    scripts/analyze_reloc_candidates.py log/gazebo_reloc_matrix/2026*_dev*/
"""

import argparse
import csv
import json
import math
import pathlib
import sys

# Fields whose LOW values indicate a better candidate.
LOWER_IS_BETTER = {
    "error",
    "condition_number",
    "score",
    "motion_residual",
    "prior_deviation",
}
FIELDS = (
    "overlap",
    "error",
    "inliers",
    "min_info_eigenvalue",
    "condition_number",
    "motion_residual",
    "prior_deviation",
    "score",
)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quantiles(values):
    if not values:
        return None
    ordered = sorted(values)
    n = len(ordered)
    return {
        "min": ordered[0],
        "p10": ordered[max(0, int(0.10 * (n - 1)))],
        "med": ordered[n // 2],
        "p90": ordered[min(n - 1, int(0.90 * (n - 1)))],
        "max": ordered[-1],
        "n": n,
    }


def fmt(value):
    if value is None:
        return "-"
    if abs(value) >= 1e4 or (value != 0.0 and abs(value) < 1e-3):
        return f"{value:.3g}"
    return f"{value:.4g}"


def load_run(run_dir):
    """Return (rows, truth) for one run directory, or None when unusable."""
    csv_path = run_dir / "candidates.csv"
    cell_path = run_dir / "cell.json"
    if not csv_path.is_file() or not cell_path.is_file():
        return None
    cell = json.loads(cell_path.read_text())
    truth = cell.get("truth_map_to_odom") or {}
    if truth.get("x") is None or truth.get("y") is None:
        return None
    with csv_path.open() as handle:
        rows = list(csv.DictReader(handle))
    return rows, truth


def classify(row, truth, xy_tol, yaw_tol):
    """True when this candidate's resolved pose is the real one."""
    try:
        err_xy = math.hypot(
            float(row["result_x"]) - float(truth["x"]),
            float(row["result_y"]) - float(truth["y"]),
        )
        err_yaw = abs(wrap(float(row["result_yaw"]) - float(truth.get("yaw") or 0.0)))
    except (KeyError, TypeError, ValueError):
        return None
    return err_xy <= xy_tol and err_yaw <= yaw_tol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "runs", nargs="+", help="run directories holding candidates.csv + cell.json"
    )
    parser.add_argument(
        "--xy-tol", type=float, default=0.30, help="correct-solution xy tolerance"
    )
    parser.add_argument(
        "--yaw-tol", type=float, default=0.20, help="correct-solution yaw tolerance"
    )
    args = parser.parse_args()

    pooled = {}
    used_runs = []
    for raw in args.runs:
        run_dir = pathlib.Path(raw)
        loaded = load_run(run_dir)
        if loaded is None:
            continue
        rows, truth = loaded
        used_runs.append(run_dir.name)
        for row in rows:
            verdict = classify(row, truth, args.xy_tol, args.yaw_tol)
            if verdict is None:
                continue
            bucket = pooled.setdefault(
                row.get("stage", "?"), {"correct": [], "wrong": []}
            )
            bucket["correct" if verdict else "wrong"].append(row)

    if not pooled:
        print(
            "no candidate diagnostics found; enable gicp_candidate_log_path",
            file=sys.stderr,
        )
        return 1

    print("# Candidate evidence separation")
    print()
    print(f"- runs pooled: {len(used_runs)}")
    for name in used_runs:
        print(f"  - {name}")
    print(
        f"- correct definition: xy <= {args.xy_tol} m and |yaw| <= {args.yaw_tol} rad vs cell truth"
    )
    print()

    for stage in sorted(pooled):
        correct = pooled[stage]["correct"]
        wrong = pooled[stage]["wrong"]
        print(f"## stage `{stage}` (correct={len(correct)} wrong={len(wrong)})")
        print()
        print(
            "|field|correct min/p10/med/p90/max|wrong min/med/p90/max|separating threshold|"
        )
        print("|---|---|---|---|")
        for field in FIELDS:

            def series(rows):
                out = []
                for row in rows:
                    try:
                        value = float(row[field])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if math.isfinite(value):
                        out.append(value)
                return out

            c = quantiles(series(correct))
            w = quantiles(series(wrong))
            if c is None or w is None:
                continue
            # A threshold is only reported when it splits the two populations
            # cleanly: no wrong candidate may sit on the correct side of it.
            if field in LOWER_IS_BETTER:
                gap = w["min"] - c["max"]
                threshold = (
                    f"<= {fmt((c['max'] + w['min']) / 2.0)}" if gap > 0 else "none"
                )
            else:
                gap = c["min"] - w["max"]
                threshold = (
                    f">= {fmt((c['min'] + w['max']) / 2.0)}" if gap > 0 else "none"
                )
            print(
                f"|{field}|{fmt(c['min'])}/{fmt(c['p10'])}/{fmt(c['med'])}/{fmt(c['p90'])}/"
                f"{fmt(c['max'])}|{fmt(w['min'])}/{fmt(w['med'])}/{fmt(w['p90'])}/{fmt(w['max'])}"
                f"|{threshold}|"
            )
        print()

    fine = pooled.get("multi_guess_fine")
    if fine and fine["correct"] and fine["wrong"]:
        print("## acceptance implication")
        print()
        print(
            "A field whose separating threshold is `none` cannot be a hard gate on this "
            "sample; it may only contribute to the combined score."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
