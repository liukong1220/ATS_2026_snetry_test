#!/usr/bin/env python3
"""Summarize raw fixed-ring MPC telemetry without claiming comparability that data cannot prove."""

import argparse
import collections
import json
import math
import sys
from pathlib import Path
from typing import Any


SAMPLING_IDENTITY_FIELDS = (
    "manager_incarnation",
    "goal_id",
    "localization_epoch",
    "map_generation",
    "map_publication_sequence",
    "reference_stamp_ns",
    "reference_deadline_ns",
    "reference_frame",
)

EFFECTIVE_PARAMETER_FIELDS = (
    "control_rate_hz",
    "control_period_ms",
    "qp_max_iterations",
    "qp_time_limit_ms",
    "qp_max_primal_residual",
    "qp_max_dual_residual",
    "qp_max_tracking_slack",
    "qp_max_hard_constraint_violation",
    "use_sim_time",
)


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
    if telemetry.get("schema_version") not in (2, 3, 4):
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
        "qp_complete_phase_ms": distribution([sample.get("qp_complete_phase_ms") for sample in attempted]),
    }
    metrics = [sample.get("qp_problem_metrics") for sample in samples]
    metric_names = sorted({key for metric in metrics if isinstance(metric, dict) for key in metric})
    status = collections.Counter(sample.get("status", "missing") for sample in samples if sample.get("qp_shadow_attempted"))
    rejection = collections.Counter(sample.get("rejection_reason", "missing") for sample in samples if sample.get("qp_shadow_attempted"))
    sampling_window = telemetry.get("sampling_window")
    requested = sampling_window.get("requested_cycle_count") if isinstance(sampling_window, dict) else None
    collected = sampling_window.get("collected_cycle_count") if isinstance(sampling_window, dict) else None
    status_name = sampling_window.get("status") if isinstance(sampling_window, dict) else "missing"
    identity_sequences = {
        field: [sample.get(field) for sample in samples]
        for field in SAMPLING_IDENTITY_FIELDS
    }
    identity_stable = all(
        len(set(values)) == 1 and values and values[0] not in (None, "")
        for values in identity_sequences.values()
    )
    lease_and_reference_healthy = bool(samples) and all(
        sample.get("execution_lease_valid") is True and sample.get("reference_fresh") is True
        for sample in samples
    )
    command_sequences = [sample.get("command_sequence") for sample in samples]
    command_sequence_monotonic = bool(command_sequences) and all(
        isinstance(sequence, int) and sequence > 0 for sequence in command_sequences
    ) and all(
        current >= previous
        for previous, current in zip(command_sequences, command_sequences[1:])
    )
    window_complete = (
        isinstance(requested, int)
        and requested > 0
        and requested == len(samples)
        and collected == len(samples)
        and status_name == "complete"
    )
    snapshot_digests_valid = bool(samples) and all(
        isinstance(sample.get("snapshot_identity_digest"), int)
        and sample["snapshot_identity_digest"] != 0
        for sample in samples
    )
    shadow_snapshot_identity_consistent = all(
        sample.get("same_snapshot_identity") is True for sample in attempted
    )
    max_iteration_diagnosis = {
        "classification": (
            "all_qp_attempts_max_iterations"
            if attempted and set(status) == {"max_iterations"}
            else "mixed_or_no_qp_attempts"
        ),
        "attempted_count": len(attempted),
        "iterations": sorted({sample.get("iterations") for sample in attempted}),
        "matrix_scale_evidence": {
            name: distribution([metric.get(name) for metric in metrics if isinstance(metric, dict)])
            for name in metric_names
        },
        "scaling_or_preconditioning": (
            "not_implemented; fixed-window paired evidence and independent conditioning analysis are required"
        ),
    }
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
        "sampling_window": {
            "raw": sampling_window,
            "manifest_matches_raw": manifest.get("sampling_window") == sampling_window,
            "complete": window_complete,
            "identity_stable_within_run": identity_stable,
            "lease_and_reference_healthy": lease_and_reference_healthy,
            "command_sequence_monotonic": command_sequence_monotonic,
            "snapshot_digests_valid": snapshot_digests_valid,
            "shadow_snapshot_identity_consistent": shadow_snapshot_identity_consistent,
            "identity": {
                field: values[0] if len(set(values)) == 1 and values else None
                for field, values in identity_sequences.items()
            },
        },
        "qp_problem_metrics": {
            name: distribution([metric.get(name) for metric in metrics if isinstance(metric, dict)])
            for name in metric_names
        },
        "cpu": "unverified_no_trusted_profiler",
        "allocation": "unverified_no_trusted_allocator_profiler",
        "max_iterations_diagnosis": max_iteration_diagnosis,
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
    runtime_keys = ["params_file"]
    runtime_equal = all(
        all(
            run["metadata"].get(key) == baseline["metadata"].get(key)
            if key != "params_file" else run["manifest"]["runtime"].get(key) == baseline["manifest"]["runtime"].get(key)
            for key in runtime_keys
        )
        for name, run in runs.items() if name != "A_ilqr_warn"
    )
    effective_parameters_equal = all(
        all(
            run["manifest"].get("effective_parameters", {}).get(key) ==
            baseline["manifest"].get("effective_parameters", {}).get(key)
            for key in EFFECTIVE_PARAMETER_FIELDS
        )
        for name, run in runs.items() if name != "A_ilqr_warn"
    )
    sampling_windows_valid = all(
        run["sampling_window"]["complete"]
        and run["sampling_window"]["manifest_matches_raw"]
        and run["sampling_window"]["identity_stable_within_run"]
        and run["sampling_window"]["lease_and_reference_healthy"]
        and run["sampling_window"]["command_sequence_monotonic"]
        and run["sampling_window"]["snapshot_digests_valid"]
        and run["sampling_window"]["shadow_snapshot_identity_consistent"]
        for run in runs.values()
    )
    requested_cycle_counts = {
        run["sampling_window"]["raw"].get("requested_cycle_count")
        if isinstance(run["sampling_window"]["raw"], dict) else None
        for run in runs.values()
    }
    sampling_identity_sequences = {
        name: tuple(sorted(run["sampling_window"]["identity"].items()))
        for name, run in runs.items()
    }
    sampling_identity_equal = len(set(sampling_identity_sequences.values())) == 1
    digest_sequences = {
        name: [sample.get("snapshot_identity_digest") for sample in run.get("raw_samples", [])]
        for name, run in runs.items()
    }
    dynamic_identity_equal = len({tuple(value) for value in digest_sequences.values()}) == 1
    reasons: list[str] = []
    if not static_equal:
        reasons.append("revision_or_scenario_differs")
    if not runtime_equal:
        reasons.append("effective_runtime_parameter_differs")
    if not effective_parameters_equal:
        reasons.append("effective_qp_parameter_differs")
    if not sampling_windows_valid:
        reasons.append("fixed_sampling_window_incomplete_or_identity_invalid")
    if len(requested_cycle_counts) != 1:
        reasons.append("sampling_window_cycle_count_differs")
    if not sampling_identity_equal:
        reasons.append("execute_reference_or_map_identity_differs")
    if not dynamic_identity_equal:
        reasons.append("per_cycle_snapshot_identity_differs_or_is_unavailable")
    return {
        "static_scenario_comparable": (
            static_equal and runtime_equal and effective_parameters_equal and
            sampling_windows_valid
        ),
        "fixed_sampling_window_comparable": sampling_windows_valid and len(requested_cycle_counts) == 1,
        "execute_reference_map_identity_comparable": sampling_identity_equal,
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
