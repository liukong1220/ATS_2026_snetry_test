#!/usr/bin/env python3
"""Summarize a realtime-profile run into per-stage and per-process evidence.

Reads the structured ``P2 projection end`` lines emitted by ``ats_rog_map`` and
the MPC control-cycle telemetry lines, then reports per-stage p50/p95/p99/max so
the earliest and the largest budget consumers can be named separately.  It does
not decide a root cause: when the accounted stages do not add up to the measured
total, the residual is reported as ``unaccounted`` rather than assigned.
"""

from __future__ import annotations

import argparse
import json
import re
import sys

PROJECTION_END = re.compile(r"P2 projection end ")
KEY_VALUE = re.compile(r"([a-z_0-9]+)=(-?[0-9]+\.?[0-9]*)")

#: ROGMap projection stages, in the order they occur.
PROJECTION_STAGES = (
    "map_lock_wait_ms",
    "map_lock_hold_ms",
    "esdf_refresh_ms",
    "sample_ms",
    "grid_type_query_ms",
    "esdf_query_ms",
    "grid_type_queries",
    "esdf_queries",
    "gradient_ms",
    "serialize_ms",
    "total_ms",
    "accounted_ms",
    "unaccounted_ms",
)

#: MPC control-cycle stages. The iLQR sub-stages are contained inside
#: ilqr_solve_ms; ilqr_jacobian_ms is itself inside ilqr_backward_pass_ms.
MPC_STAGES = (
    "state_trajectory_snapshot_ms",
    "reference_extraction_ms",
    "ilqr_solve_ms",
    "ilqr_warm_start_ms",
    "ilqr_rollout_ms",
    "ilqr_backward_pass_ms",
    "ilqr_jacobian_ms",
    "ilqr_line_search_ms",
    "ilqr_command_publish_ms",
    "candidate_hard_check_ms",
    "telemetry_ring_write_ms",
    "percentile_aggregation_ms",
    "logging_publish_ms",
    "full_callback_ms",
    "timer_interarrival_ms",
)


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def describe(name: str, values: list[float]) -> str:
    if not values:
        return f"  {name}: no samples"
    return (
        f"  {name}: n={len(values)} "
        f"p50={percentile(values, 0.50):.1f} "
        f"p95={percentile(values, 0.95):.1f} "
        f"p99={percentile(values, 0.99):.1f} "
        f"max={max(values):.1f}"
    )


def collect_stage_samples(path: str, matcher, stages: tuple[str, ...]) -> dict:
    samples: dict[str, list[float]] = {stage: [] for stage in stages}
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if not matcher(line):
                continue
            fields = dict(KEY_VALUE.findall(line))
            for stage in stages:
                if stage in fields:
                    try:
                        samples[stage].append(float(fields[stage]))
                    except ValueError:
                        continue
    return samples


def summarize_processes(path: str) -> list[str]:
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            header = handle.readline().rstrip("\n").split("\t")
            rows = [line.rstrip("\n").split("\t") for line in handle if line.strip()]
    except FileNotFoundError:
        return ["  no process samples collected"]
    if not rows:
        return ["  no process samples collected"]
    index = {name: position for position, name in enumerate(header)}
    grouped: dict[str, dict] = {}
    for row in rows:
        if len(row) != len(header):
            continue
        comm = row[index["comm"]]
        entry = grouped.setdefault(
            comm,
            {"pcpu": [], "rss_kb": [], "threads": [], "vol": [], "nonvol": []},
        )
        for key, column in (
            ("pcpu", "pcpu"),
            ("rss_kb", "rss_kb"),
            ("threads", "threads"),
            ("vol", "voluntary_ctxt"),
            ("nonvol", "nonvoluntary_ctxt"),
        ):
            try:
                entry[key].append(float(row[index[column]]))
            except (ValueError, KeyError):
                continue
    lines = []
    for comm in sorted(grouped):
        entry = grouped[comm]
        if not entry["pcpu"]:
            continue
        # 上下文切换是单调累计计数，取窗口内的增量而不是平均值。
        vol_delta = (
            max(entry["vol"]) - min(entry["vol"]) if entry["vol"] else float("nan")
        )
        nonvol_delta = (
            max(entry["nonvol"]) - min(entry["nonvol"])
            if entry["nonvol"]
            else float("nan")
        )
        lines.append(
            f"  {comm}: samples={len(entry['pcpu'])} "
            f"cpu_p50={percentile(entry['pcpu'], 0.50):.1f}% "
            f"cpu_max={max(entry['pcpu']):.1f}% "
            f"rss_max_mb={max(entry['rss_kb']) / 1024.0:.1f} "
            f"threads_max={int(max(entry['threads'])) if entry['threads'] else 0} "
            f"voluntary_ctxt_delta={vol_delta:.0f} "
            f"nonvoluntary_ctxt_delta={nonvol_delta:.0f}"
        )
    return lines or ["  no process samples matched the expected columns"]


def summarize_control_telemetry(path: str) -> list[str]:
    """Read the exported control-telemetry JSON for the MPC per-stage split.

    The MPC stage timings live in the bounded telemetry ring, not in log lines:
    nothing is added to the 20 Hz control timer just to collect them.  When the
    export is absent this reports that plainly instead of inventing samples.
    """
    try:
        with open(path, encoding="utf-8") as handle:
            document = json.load(handle)
    except FileNotFoundError:
        return [f"  no control telemetry export at {path}"]
    except (OSError, json.JSONDecodeError) as error:
        return [f"  control telemetry export unreadable: {error}"]

    window = document.get("sampling_window", {})
    lines = [
        f"  sampling_window status={window.get('status', 'unknown')} "
        f"requested={window.get('requested_cycle_count', 0)} "
        f"collected={window.get('collected_cycle_count', 0)}"
    ]
    summary = document.get("timing_summary", {})
    for stage in MPC_STAGES:
        stats = summary.get(stage)
        if not isinstance(stats, dict) or not stats.get("count"):
            lines.append(f"  {stage}: no samples")
            continue

        def read(key: str) -> float:
            value = stats.get(key)
            return float(value) if isinstance(value, (int, float)) else float("nan")

        lines.append(
            f"  {stage}: n={stats['count']} "
            f"p50={read('p50_ms'):.1f} p95={read('p95_ms'):.1f} "
            f"p99={read('p99_ms'):.1f} max={read('max_ms'):.1f}"
        )
    counters = document.get("deadline_counters", {})
    nonzero = {name: value for name, value in counters.items() if value}
    lines.append(
        f"  deadline_counters(nonzero)={nonzero or 'none'}"
    )

    period_ms = document.get("metadata", {}).get("control_period_ms")
    callbacks = [
        sample["timing_ms"]["full_callback_ms"]
        for sample in document.get("samples", [])
        if isinstance(sample.get("timing_ms", {}).get("full_callback_ms"), (int, float))
    ]
    if callbacks and isinstance(period_ms, (int, float)):
        misses = sum(1 for value in callbacks if value > period_ms)
        lines.append(
            f"  full_callback over reported control_period_ms={period_ms:.1f}: "
            f"{misses}/{len(callbacks)} samples"
        )
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-log", required=True)
    parser.add_argument("--process-samples", required=True)
    parser.add_argument(
        "--control-telemetry",
        default="",
        help="Exported /ats_swerve_mpc/dump_control_telemetry JSON, if collected.",
    )
    parser.add_argument("--experiment", required=True)
    parser.add_argument("--domain", required=True)
    parser.add_argument(
        "--control-period-ms",
        type=float,
        default=50.0,
        help="Reported only for comparison; this script never relaxes it.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print(f"experiment={args.experiment} domain={args.domain} log={args.run_log}")

    projection = collect_stage_samples(
        args.run_log, lambda line: PROJECTION_END.search(line) is not None,
        PROJECTION_STAGES,
    )
    print("ROGMap ground projection stages (ms):")
    for stage in PROJECTION_STAGES:
        print(describe(stage, projection[stage]))

    print("MPC control cycle stages (ms, from the telemetry ring export):")
    if args.control_telemetry:
        for line in summarize_control_telemetry(args.control_telemetry):
            print(line)
    else:
        print("  no --control-telemetry given; MPC stage split not collected")

    print(f"Budget reference: control_period_ms={args.control_period_ms:.1f}")
    totals = projection["total_ms"]
    if totals:
        print(
            f"  projection total p99={percentile(totals, 0.99):.1f} ms "
            f"max={max(totals):.1f} ms"
        )
    print("Per-process resource usage (ps + /proc only):")
    for line in summarize_processes(args.process_samples):
        print(line)
    print(
        "Attribution note: stages are reported separately and the residual is "
        "left as unaccounted_ms. Without a CPU/scheduler trace this output does "
        "not establish a sole root cause."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
