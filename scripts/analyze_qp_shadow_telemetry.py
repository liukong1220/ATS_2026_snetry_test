#!/usr/bin/env python3
"""Summarize raw fixed-ring MPC telemetry without claiming comparability that data cannot prove."""

import argparse
import collections
import json
import math
import sys
from pathlib import Path
from typing import Any


def finite_values(samples: list[Any]) -> list[float]:
    return sorted(float(value) for value in samples if isinstance(value, (int, float)) and math.isfinite(value))


def distribution(samples: list[Any]) -> dict[str, Any]:
    values = finite_values(samples)
    if not values:
        return {"count": 0, "p50_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}

    def percentile(quantile: float) -> float:
        return values[min(len(values) - 1, int(quantile * (len(values) - 1)))]

    return {
        "count": len(values),
        "p50_ms": percentile(0.50),
        "p95_ms": percentile(0.95),
        "p99_ms": percentile(0.99),
        "max_ms": values[-1],
    }


def load_run(specification: str) -> tuple[str, dict[str, Any], dict[str, Any]]:
    name, separator, directory = specification.partition("=")
    if not separator or not name or not directory:
        raise ValueError("--run must use NAME=DIRECTORY")
    root = Path(directory)
    telemetry = json.loads((root / "raw_telemetry.json").read_text())
    manifest = json.loads((root / "manifest.json").read_text())
    if telemetry.get("schema_version") != 2:
        raise ValueError(f"{name}: unsupported telemetry schema")
    if not isinstance(telemetry.get("samples"), list):
        raise ValueError(f"{name}: missing samples")
    return name, telemetry, manifest


def summarize(telemetry: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    samples = telemetry["samples"]
    stage_names = sorted({key for sample in samples for key in sample.get("timing_ms", {})})
    timing = {
        name: distribution([sample.get("timing_ms", {}).get(name) for sample in samples])
        for name in stage_names
    }
    attempted = [sample for sample in samples if sample.get("qp_shadow_attempted")]
    reported = {
        "osqp_reported_update_ms": distribution([sample.get("osqp_reported_update_ms") for sample in attempted]),
        "osqp_reported_solve_ms": distribution([sample.get("osqp_reported_solve_ms") for sample in attempted]),
        "osqp_wall_update_ms": distribution([sample.get("osqp_wall_update_ms") for sample in attempted]),
        "osqp_wall_solve_ms": distribution([sample.get("osqp_wall_solve_ms") for sample in attempted]),
    }
    metrics = [sample.get("qp_problem_metrics") for sample in samples]
    metric_names = sorted({key for metric in metrics if isinstance(metric, dict) for key in metric})
    status = collections.Counter(sample.get("status", "missing") for sample in samples if sample.get("qp_shadow_attempted"))
    rejection = collections.Counter(sample.get("rejection_reason", "missing") for sample in samples if sample.get("qp_shadow_attempted"))
    return {
        "metadata": telemetry.get("metadata", {}),
        "manifest": manifest,
        "sample_count": len(samples),
        "qp_shadow_sample_count": len(attempted),
        "timing_ms": timing,
        "osqp_time_ms": reported,
        "deadline_counters": telemetry.get("deadline_counters", {}),
        "status_counts": dict(status),
        "candidate_rejection_counts": dict(rejection),
        "candidate_feasible_count": sum(bool(sample.get("candidate_feasible")) for sample in attempted),
        "warm_start_used_count": sum(bool(sample.get("warm_start_used")) for sample in attempted),
        "same_snapshot_identity_count": sum(bool(sample.get("same_snapshot_identity")) for sample in attempted),
        "qp_problem_metrics": {
            name: distribution([metric.get(name) for metric in metrics if isinstance(metric, dict)])
            for name in metric_names
        },
        "cpu": "unverified_no_trusted_profiler",
        "allocation": "unverified_no_trusted_allocator_profiler",
    }


def comparability(runs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    required = {"A_ilqr_warn", "B_qp_shadow_warn", "C_qp_shadow_info"}
    if set(runs) != required:
        return {"verdict": "not_evaluated", "reason": "requires A_ilqr_warn, B_qp_shadow_warn, C_qp_shadow_info"}
    baseline = runs["A_ilqr_warn"]
    static_keys = ["revisions", "scenario"]
    static_equal = all(
        all(run["manifest"].get(key) == baseline["manifest"].get(key) for key in static_keys)
        for name, run in runs.items() if name != "A_ilqr_warn"
    )
    runtime_keys = ["control_rate_hz", "control_period_ms", "qp_time_limit_ms", "use_sim_time", "params_file"]
    runtime_equal = all(
        all(
            run["metadata"].get(key) == baseline["metadata"].get(key)
            if key != "params_file" else run["manifest"]["runtime"].get(key) == baseline["manifest"]["runtime"].get(key)
            for key in runtime_keys
        )
        for name, run in runs.items() if name != "A_ilqr_warn"
    )
    digest_sequences = {
        name: [sample.get("snapshot_identity_digest") for sample in run.get("raw_samples", [])]
        for name, run in runs.items()
    }
    dynamic_identity_equal = len({tuple(value) for value in digest_sequences.values()}) == 1
    durations = [run["manifest"]["runtime"].get("duration_ms_at_dump") for run in runs.values()]
    finite_durations = [float(value) for value in durations if isinstance(value, (int, float)) and math.isfinite(value)]
    runtime_duration_equal = len(finite_durations) == len(runs) and (
        max(finite_durations) - min(finite_durations) <= 0.05 * min(finite_durations)
    )
    reasons: list[str] = []
    if not static_equal:
        reasons.append("revision_or_scenario_differs")
    if not runtime_equal:
        reasons.append("effective_runtime_parameter_differs")
    if not runtime_duration_equal:
        reasons.append("runtime_duration_differs")
    if not dynamic_identity_equal:
        reasons.append("per_cycle_snapshot_identity_differs_or_is_unavailable")
    return {
        "static_scenario_comparable": static_equal and runtime_equal and runtime_duration_equal,
        "runtime_duration_comparable": runtime_duration_equal,
        "dynamic_input_identity_comparable": dynamic_identity_equal,
        "verdict": "comparable" if not reasons else "not_comparable",
        "reasons": reasons,
        "shadow_increment_cost_conclusion": "withheld" if reasons else "eligible_for_review_not_automatically_accepted",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True, help="NAME=DIRECTORY")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    summarized: dict[str, dict[str, Any]] = {}
    for specification in args.run:
        name, telemetry, manifest = load_run(specification)
        summary = summarize(telemetry, manifest)
        summary["raw_samples"] = telemetry["samples"]
        summarized[name] = summary
    comparison = comparability(summarized)
    for summary in summarized.values():
        summary.pop("raw_samples", None)
    payload = {"schema_version": 1, "runs": summarized, "comparison": comparison}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n")
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
