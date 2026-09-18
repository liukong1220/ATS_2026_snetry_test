#!/usr/bin/env python3
"""Aggregate relocalization matrix cell reports into a per-cell gate table."""

from __future__ import annotations

import json
import sys
from collections import OrderedDict
from pathlib import Path

RECOVER_RATE_GATE = 0.9
# A 9/10 claim needs at least 9 scored repetitions behind it.
MIN_SCORED = 9


def fmt(value, digits=3):
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "usage: summarize_gazebo_reloc_matrix.py <summary.jsonl> [more.jsonl ...]",
            file=sys.stderr,
        )
        return 2
    # Several batches of the same matrix aggregate into one verdict; a cell's
    # repetitions may be split across invocations.
    records = [
        json.loads(line)
        for path in sys.argv[1:]
        for line in Path(path).read_text().splitlines()
        if line.strip()
    ]
    cells: "OrderedDict[str, list[dict]]" = OrderedDict()
    for record in records:
        cells.setdefault(record["cell_id"], []).append(record)

    print("# Relocalization acceptance matrix")
    print()
    print(
        "|cell|entry|offset|runs|recovered|fault-injected|status-live|"
        "wrong-accept|nonfinite-accept|"
        "best_xy min/med/max [m]|first_recover med [s]|hold med [s]|"
        "obs acc med|status gap max [s]|gate|"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")

    all_pass = True
    total_wrong = 0
    total_lost = 0
    for cell, runs in cells.items():
        # A repetition whose bringup never reached the probe carries no
        # relocalization information: it is neither a recovery nor a failure.
        # It is excluded from the rates and reported as an infrastructure loss,
        # and a cell still needs MIN_SCORED scored repetitions to claim a rate.
        lost = [r for r in runs if r.get("parse_error")]
        total_lost += len(lost)
        runs = [r for r in runs if not r.get("parse_error")]
        n = len(runs)
        if n == 0:
            all_pass = False
            print(f"|{cell}|-|-|0|-|-|-|-|-|-|-|-|-|-|FAIL (no scored run)|")
            continue
        recovered = sum(1 for r in runs if r.get("pass_recover"))
        wrong = sum(int(r.get("wrong_accept_count") or 0) for r in runs)
        nonfinite = sum(int(r.get("nonfinite_error_accepted") or 0) for r in runs)
        total_wrong += wrong
        best = sorted(r["best_xy_err_m"] for r in runs if r.get("best_xy_err_m") is not None)
        firsts = sorted(r["first_recover_s"] for r in runs if r.get("first_recover_s") is not None)
        holds = sorted(r["hold_best_s"] for r in runs if r.get("hold_best_s") is not None)
        obs = sorted(r["obs_accepted"] for r in runs if r.get("obs_accepted") is not None)
        gaps = [r["status_max_gap_s"] for r in runs if r.get("status_max_gap_s") is not None]
        entry = runs[0].get("entry", "-")
        offset = runs[0].get("offset") or {}
        offset_text = (
            f"({fmt(offset.get('x'), 2)},{fmt(offset.get('y'), 2)},{fmt(offset.get('yaw'), 2)})"
        )
        # Every cell gate counts: a "recovery" without a proven injected fault, a
        # dead status stream, or a non-finite acceptance is not a pass.
        injected = sum(1 for r in runs if r.get("pass_fault_injected"))
        live = sum(1 for r in runs if r.get("pass_status_live"))
        gate = (
            n >= MIN_SCORED
            and recovered >= RECOVER_RATE_GATE * n
            and wrong == 0
            and nonfinite == 0
            and injected == n
            and live == n
        )
        all_pass = all_pass and gate
        best_text = f"{fmt(best[0])}/{fmt(best[len(best) // 2])}/{fmt(best[-1])}" if best else "-"
        print(
            f"|{cell}|{entry}|{offset_text}|{n}{f'+{len(lost)}L' if lost else ''}|"
            f"{recovered}/{n}|{injected}/{n}|{live}/{n}|"
            f"{wrong}|{nonfinite}|"
            f"{best_text}|{fmt(firsts[len(firsts) // 2], 1) if firsts else '-'}|"
            f"{fmt(holds[len(holds) // 2], 1) if holds else '-'}|"
            f"{obs[len(obs) // 2] if obs else '-'}|"
            f"{fmt(max(gaps), 2) if gaps else '-'}|{'PASS' if gate else 'FAIL'}|"
        )

    print()
    print(f"- runs total: {len(records)}")
    print(f"- wrong acceptances total: {total_wrong} (gate: 0)")
    print(f"- recovery-rate gate per cell: >= {RECOVER_RATE_GATE:.0%} of >= {MIN_SCORED} scored")
    print(f"- repetitions lost to infrastructure (excluded from rates): {total_lost}")
    retries = sum(max(0, int(r.get("infra_attempts") or 1) - 1) for r in records)
    print(f"- infrastructure retries (bringup never reached the probe): {retries}")
    starved = sum(1 for r in records if r.get("sim_starved"))
    print(f"- repetitions whose scan pipeline starved after retry: {starved}")
    print(f"- matrix verdict: {'PASS' if all_pass else 'FAIL'}")

    failures = [
        r
        for r in records
        if not r.get("parse_error")
        and (not r.get("pass_recover") or int(r.get("wrong_accept_count") or 0) > 0)
    ]
    lost = [r for r in records if r.get("parse_error")]
    if lost:
        print()
        print("## Repetitions lost to infrastructure (no relocalization evidence)")
        for r in lost:
            print(
                f"- `{r['cell_id']}` rep{r.get('rep')} domain{r.get('domain')}: "
                f"attempts={r.get('infra_attempts')} reason={r.get('parse_error')!r} "
                f"out={r.get('out')}"
            )
    if failures:
        print()
        print("## Failing repetitions")
        for r in failures:
            print(
                f"- `{r['cell_id']}` rep{r.get('rep')} domain{r.get('domain')}: "
                f"best_xy={fmt(r.get('best_xy_err_m'))} hold={fmt(r.get('hold_best_s'), 1)} "
                f"obs_accepted={r.get('obs_accepted')} wrong={r.get('wrong_accept_count')} "
                f"status_last={r.get('status_last_message')!r} out={r.get('out')}"
            )
    return 0 if all_pass else 4


if __name__ == "__main__":
    raise SystemExit(main())
