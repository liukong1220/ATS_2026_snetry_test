#!/usr/bin/env python3
"""Summarize numeric timing or robotics metrics from a CSV file."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", help="Input CSV file")
    parser.add_argument(
        "--column", action="append", dest="columns", help="Numeric column; repeat for multiple"
    )
    parser.add_argument("--group-by", help="Optional categorical grouping column")
    parser.add_argument("--deadline", type=float, help="Deadline threshold in the column's unit")
    parser.add_argument("--warmup-rows", type=int, default=0, help="Discard this many initial rows")
    parser.add_argument("--format", choices=("json", "markdown"), default="markdown")
    parser.add_argument("--output")
    return parser.parse_args()


def quantile(values: list[float], probability: float) -> float:
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def summarize(values: list[float], deadline: float | None, invalid: int) -> dict:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "invalid_or_missing": invalid}
    median = quantile(ordered, 0.5)
    absolute_deviations = sorted(abs(value - median) for value in ordered)
    result = {
        "count": len(ordered),
        "invalid_or_missing": invalid,
        "min": ordered[0],
        "max": ordered[-1],
        "mean": statistics.fmean(ordered),
        "stdev": statistics.stdev(ordered) if len(ordered) > 1 else 0.0,
        "p50": median,
        "p90": quantile(ordered, 0.90),
        "p95": quantile(ordered, 0.95),
        "p99": quantile(ordered, 0.99),
        "mad": quantile(absolute_deviations, 0.5),
    }
    if deadline is not None:
        misses = sum(value > deadline for value in ordered)
        result.update(
            {
                "deadline": deadline,
                "deadline_misses": misses,
                "deadline_miss_rate": misses / len(ordered),
            }
        )
    return result


def detect_dialect(path: Path) -> csv.Dialect:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        sample = handle.read(8192)
    try:
        return csv.Sniffer().sniff(sample, delimiters=",;\t|")
    except csv.Error:
        return csv.excel


def load_data(args: argparse.Namespace) -> dict:
    path = Path(args.csv_file).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"CSV file does not exist: {path}")
    if args.warmup_rows < 0:
        raise ValueError("--warmup-rows must be non-negative")

    dialect = detect_dialect(path)
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, dialect=dialect)
        headers = reader.fieldnames or []
        if not headers:
            raise ValueError("CSV has no header")
        if args.group_by and args.group_by not in headers:
            raise ValueError(f"group column not found: {args.group_by}")

        columns = args.columns
        rows = list(reader)
        considered = rows[args.warmup_rows :]
        if not columns:
            columns = []
            for header in headers:
                if header == args.group_by:
                    continue
                for row in considered:
                    raw = (row.get(header) or "").strip()
                    if not raw:
                        continue
                    try:
                        value = float(raw)
                    except ValueError:
                        break
                    if math.isfinite(value):
                        columns.append(header)
                    break
        missing_columns = [column for column in columns if column not in headers]
        if missing_columns:
            raise ValueError(f"columns not found: {', '.join(missing_columns)}")
        if not columns:
            raise ValueError("no numeric columns selected or inferred")

        grouped_values: dict[str, dict[str, list[float]]] = defaultdict(
            lambda: defaultdict(list)
        )
        grouped_invalid: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
        for row in considered:
            group = str(row.get(args.group_by, "all")) if args.group_by else "all"
            for column in columns:
                raw = (row.get(column) or "").strip()
                try:
                    value = float(raw)
                except ValueError:
                    grouped_invalid[group][column] += 1
                    continue
                if not math.isfinite(value):
                    grouped_invalid[group][column] += 1
                    continue
                grouped_values[group][column].append(value)

    groups = {}
    all_group_names = sorted(set(grouped_values) | set(grouped_invalid)) or ["all"]
    for group in all_group_names:
        groups[group] = {}
        for column in columns:
            groups[group][column] = summarize(
                grouped_values[group][column], args.deadline, grouped_invalid[group][column]
            )
    return {
        "schema": "robot-metric-summary/v1",
        "source": str(path),
        "rows_total": len(rows),
        "warmup_rows_discarded": min(args.warmup_rows, len(rows)),
        "rows_considered": len(considered),
        "group_by": args.group_by,
        "columns": columns,
        "groups": groups,
    }


def number(value) -> str:
    if isinstance(value, int):
        return str(value)
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "-"
    return f"{value:.6g}"


def render_markdown(data: dict) -> str:
    lines = [
        "# Metric Summary",
        "",
        f"Source: `{data['source']}`",
        "",
        f"Rows: {data['rows_considered']} considered, "
        f"{data['warmup_rows_discarded']} warm-up rows discarded.",
    ]
    for group, metrics in data["groups"].items():
        lines.extend(["", f"## Group: {group}", ""])
        lines.append("| Metric | Count | Mean | p50 | p95 | p99 | Max | Stdev | Miss rate |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
        for name, summary in metrics.items():
            miss_rate = summary.get("deadline_miss_rate")
            miss_text = f"{100.0 * miss_rate:.3f}%" if miss_rate is not None else "-"
            lines.append(
                f"| `{name}` | {number(summary.get('count'))} | {number(summary.get('mean'))} | "
                f"{number(summary.get('p50'))} | {number(summary.get('p95'))} | "
                f"{number(summary.get('p99'))} | {number(summary.get('max'))} | "
                f"{number(summary.get('stdev'))} | {miss_text} |"
            )
            if summary.get("invalid_or_missing"):
                lines.append(
                    f"\n`{name}` excluded {summary['invalid_or_missing']} missing or non-finite values."
                )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    try:
        data = load_data(args)
    except (OSError, ValueError, csv.Error) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    output = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
    if args.format == "markdown":
        output = render_markdown(data)
    if args.output:
        Path(args.output).expanduser().write_text(output, encoding="utf-8")
    else:
        sys.stdout.write(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
