#!/usr/bin/env python3
"""Compare numeric fields in two JSON experiment summaries."""

from __future__ import annotations

import argparse
import fnmatch
import json
import math
from pathlib import Path
import sys


DEFAULT_LOWER_TERMS = (
    "cost",
    "deadline_miss",
    "drift",
    "error",
    "jitter",
    "latency",
    "loss",
    "max",
    "mean",
    "memory",
    "p50",
    "p90",
    "p95",
    "p99",
    "rmse",
    "stdev",
    "time",
)
DEFAULT_HIGHER_TERMS = (
    "accuracy",
    "availability",
    "fps",
    "precision",
    "recall",
    "success",
    "throughput",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", help="Baseline JSON summary")
    parser.add_argument("candidate", help="Candidate JSON summary")
    parser.add_argument("--prefix", help="Only compare flattened keys under this prefix")
    parser.add_argument(
        "--lower-is-better",
        action="append",
        default=[],
        metavar="GLOB",
        help="Classify matching flattened keys as lower-is-better; repeat as needed",
    )
    parser.add_argument(
        "--higher-is-better",
        action="append",
        default=[],
        metavar="GLOB",
        help="Classify matching flattened keys as higher-is-better; repeat as needed",
    )
    parser.add_argument(
        "--regression-threshold-pct",
        type=float,
        default=5.0,
        help="Relative degradation required to label a classified metric regression",
    )
    parser.add_argument("--no-infer-direction", action="store_true")
    parser.add_argument("--fail-on-regression", action="store_true")
    parser.add_argument("--format", choices=("json", "markdown"), default="markdown")
    parser.add_argument("--output")
    return parser.parse_args()


def load_json(path_value: str) -> tuple[Path, object]:
    path = Path(path_value).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"JSON file does not exist: {path}")
    try:
        return path, json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {path}: {error}") from error


def flatten_numbers(value, prefix: str = "") -> dict[str, float]:
    result: dict[str, float] = {}
    if isinstance(value, bool):
        return result
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        result[prefix or "value"] = float(value)
    elif isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            result.update(flatten_numbers(child, child_prefix))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            child_prefix = f"{prefix}[{index}]" if prefix else f"[{index}]"
            result.update(flatten_numbers(child, child_prefix))
    return result


def matches(key: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatch(key, pattern) for pattern in patterns)


def infer_direction(key: str) -> str | None:
    lowered = key.lower()
    leaf = lowered.rsplit(".", 1)[-1]
    if any(term in leaf or term in lowered for term in DEFAULT_HIGHER_TERMS):
        return "higher"
    if any(term in leaf or term in lowered for term in DEFAULT_LOWER_TERMS):
        return "lower"
    return None


def classify_direction(key: str, args: argparse.Namespace) -> tuple[str | None, str]:
    lower_match = matches(key, args.lower_is_better)
    higher_match = matches(key, args.higher_is_better)
    if lower_match and higher_match:
        return None, "conflicting-user-rules"
    if lower_match:
        return "lower", "user-rule"
    if higher_match:
        return "higher", "user-rule"
    if args.no_infer_direction:
        return None, "unclassified"
    direction = infer_direction(key)
    return direction, "name-heuristic" if direction else "unclassified"


def compare(args: argparse.Namespace) -> dict:
    baseline_path, baseline_data = load_json(args.baseline)
    candidate_path, candidate_data = load_json(args.candidate)
    baseline = flatten_numbers(baseline_data)
    candidate = flatten_numbers(candidate_data)
    keys = sorted(set(baseline) & set(candidate))
    if args.prefix:
        keys = [key for key in keys if key == args.prefix or key.startswith(args.prefix + ".")]
    if not keys:
        raise ValueError("no common numeric fields matched the requested prefix")
    if args.regression_threshold_pct < 0:
        raise ValueError("--regression-threshold-pct must be non-negative")

    comparisons = []
    regressions = 0
    improvements = 0
    for key in keys:
        before = baseline[key]
        after = candidate[key]
        delta = after - before
        relative_pct = None if before == 0.0 else 100.0 * delta / abs(before)
        direction, direction_source = classify_direction(key, args)
        outcome = "unclassified"
        degradation_pct = None
        if direction and relative_pct is not None:
            signed_improvement = -relative_pct if direction == "lower" else relative_pct
            degradation_pct = -signed_improvement
            if degradation_pct > args.regression_threshold_pct:
                outcome = "regression"
                regressions += 1
            elif signed_improvement > args.regression_threshold_pct:
                outcome = "improvement"
                improvements += 1
            else:
                outcome = "within-threshold"
        comparisons.append(
            {
                "key": key,
                "baseline": before,
                "candidate": after,
                "delta": delta,
                "relative_change_pct": relative_pct,
                "direction": direction,
                "direction_source": direction_source,
                "outcome": outcome,
                "degradation_pct": degradation_pct,
            }
        )
    return {
        "schema": "robot-experiment-comparison/v1",
        "baseline": str(baseline_path),
        "candidate": str(candidate_path),
        "prefix": args.prefix,
        "regression_threshold_pct": args.regression_threshold_pct,
        "direction_warning": "name-heuristic directions require domain review",
        "summary": {
            "compared": len(comparisons),
            "regressions": regressions,
            "improvements": improvements,
            "unclassified": sum(item["outcome"] == "unclassified" for item in comparisons),
        },
        "comparisons": comparisons,
    }


def display_number(value) -> str:
    if value is None:
        return "-"
    return f"{value:.6g}"


def render_markdown(data: dict) -> str:
    summary = data["summary"]
    lines = [
        "# Experiment Comparison",
        "",
        f"Baseline: `{data['baseline']}`",
        "",
        f"Candidate: `{data['candidate']}`",
        "",
        f"Compared {summary['compared']} numeric fields: {summary['regressions']} regressions, "
        f"{summary['improvements']} improvements, {summary['unclassified']} unclassified.",
        "",
        "| Metric | Baseline | Candidate | Change | Direction | Result |",
        "|---|---:|---:|---:|---|---|",
    ]
    ordered = sorted(
        data["comparisons"],
        key=lambda item: (
            {"regression": 0, "improvement": 1, "within-threshold": 2, "unclassified": 3}.get(
                item["outcome"], 4
            ),
            item["key"],
        ),
    )
    for item in ordered[:200]:
        change = item["relative_change_pct"]
        change_text = f"{change:+.3f}%" if change is not None else "n/a"
        direction = item["direction"] or "-"
        if item["direction_source"] == "name-heuristic" and item["direction"]:
            direction += " (inferred)"
        lines.append(
            f"| `{item['key']}` | {display_number(item['baseline'])} | "
            f"{display_number(item['candidate'])} | {change_text} | {direction} | "
            f"{item['outcome']} |"
        )
    if len(ordered) > 200:
        lines.append(f"\n{len(ordered) - 200} additional comparisons omitted from Markdown.")
    lines.extend(
        [
            "",
            "> Direction inferred from metric names is advisory. Confirm domain semantics, run-to-run",
            "> variance, sample counts, confidence intervals, and experiment comparability before deciding.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    try:
        data = compare(args)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    output = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
    if args.format == "markdown":
        output = render_markdown(data)
    if args.output:
        Path(args.output).expanduser().write_text(output, encoding="utf-8")
    else:
        sys.stdout.write(output)
    if args.fail_on_regression and data["summary"]["regressions"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
