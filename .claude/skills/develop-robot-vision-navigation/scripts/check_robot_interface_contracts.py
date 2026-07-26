#!/usr/bin/env python3
"""Collect static evidence for robotics interface-contract audits.

The output is intentionally heuristic. It inventories schemas, topic endpoints, frame
assignments, clock use, and message-field references; it does not prove runtime wiring.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import re
import sys


IGNORED_DIRS = {
    ".git",
    ".cache",
    ".pytest_cache",
    "__pycache__",
    "build",
    "dist",
    "install",
    "log",
    "logs",
    "node_modules",
    "target",
    "vendor",
}
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".py"}
SCHEMA_SUFFIXES = {".action", ".msg", ".srv"}
TEXT_LIMIT_BYTES = 2_000_000

CPP_ENDPOINT_PATTERNS = (
    ("publisher", re.compile(r"create_publisher\s*<\s*([^>]+)>\s*\(\s*[\"']([^\"']+)[\"']", re.S)),
    ("subscription", re.compile(r"create_subscription\s*<\s*([^>]+)>\s*\(\s*[\"']([^\"']+)[\"']", re.S)),
    ("publisher", re.compile(r"\.advertise\s*<\s*([^>]+)>\s*\(\s*[\"']([^\"']+)[\"']", re.S)),
    ("subscription", re.compile(r"\.subscribe\s*<\s*([^>]+)>\s*\(\s*[\"']([^\"']+)[\"']", re.S)),
)
PY_ENDPOINT_PATTERNS = (
    ("publisher", re.compile(r"create_publisher\s*\(\s*([^,\n]+),\s*[\"']([^\"']+)[\"']", re.S)),
    ("subscription", re.compile(r"create_subscription\s*\(\s*([^,\n]+),\s*[\"']([^\"']+)[\"']", re.S)),
)
TWO_STRING_CPP_ENDPOINT = re.compile(
    r"(?:\.|->)create_(publisher|subscription)\s*<\s*([^>]+)>\s*\(\s*"
    r"[\"']([^\"']+)[\"']\s*,\s*[\"']([^\"']+)[\"']",
    re.S,
)
SOURCE_FRAME_PATTERNS = (
    re.compile(r"(?:header\.frame_id|child_frame_id)\s*=\s*([^;\n]+)"),
)
CONFIG_FRAME_PATTERNS = (
    re.compile(r"(?:child_frame_id|[A-Za-z0-9_]*frame(?:_id)?)\s*:\s*([^\n#]+)"),
)
CLOCK_TOKENS = {
    "steady_clock": re.compile(r"steady_clock"),
    "system_clock": re.compile(r"system_clock"),
    "ros_or_middleware_now": re.compile(r"(?:get_clock\(\)|\bnow\(\)|Clock\()"),
    "wall_timer": re.compile(r"wall_timer|create_wall_timer"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".", help="Workspace root to inspect")
    parser.add_argument("--format", choices=("json", "markdown"), default="markdown")
    parser.add_argument("--max-depth", type=int, default=8)
    parser.add_argument("--max-files", type=int, default=40000)
    parser.add_argument("--output")
    return parser.parse_args()


def rel(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def walk_text_files(root: Path, max_depth: int, max_files: int):
    seen = 0
    for current, dirs, files in os.walk(root, followlinks=False):
        current_path = Path(current)
        depth = len(current_path.relative_to(root).parts)
        dirs[:] = sorted(
            name
            for name in dirs
            if name not in IGNORED_DIRS and not (current_path / name).is_symlink()
        )
        if depth >= max_depth:
            dirs[:] = []
        for name in sorted(files):
            if seen >= max_files:
                return
            path = current_path / name
            suffix = path.suffix.lower()
            if suffix not in SOURCE_SUFFIXES | SCHEMA_SUFFIXES | {".yaml", ".yml"}:
                continue
            if path.is_symlink():
                continue
            try:
                if path.stat().st_size > TEXT_LIMIT_BYTES:
                    continue
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            seen += 1
            yield path, text


def parse_schema(path: Path, text: str, root: Path) -> dict:
    sections = [[]]
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line == "---":
            sections.append([])
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        field_type, token = parts[0], parts[1]
        constant = "=" in token
        field_name = token.split("=", 1)[0]
        sections[-1].append(
            {"name": field_name, "type": field_type, "constant": constant, "line": number}
        )
    return {
        "path": rel(path, root),
        "name": path.stem,
        "kind": path.suffix.lstrip("."),
        "sections": sections,
    }


def endpoint_evidence(path: Path, text: str, root: Path) -> list[dict]:
    patterns = PY_ENDPOINT_PATTERNS if path.suffix.lower() == ".py" else CPP_ENDPOINT_PATTERNS
    evidence = []
    covered_offsets: set[int] = set()
    if path.suffix.lower() != ".py":
        for match in TWO_STRING_CPP_ENDPOINT.finditer(text):
            evidence.append(
                {
                    "direction": match.group(1),
                    "topic": match.group(4).strip(),
                    "type": " ".join(match.group(2).split()),
                    "path": rel(path, root),
                    "line": line_number(text, match.start()),
                    "note": f"selected second string argument; first was {match.group(3)!r}",
                }
            )
            covered_offsets.add(match.start())
    for direction, pattern in patterns:
        for match in pattern.finditer(text):
            if any(abs(match.start() - offset) < 32 for offset in covered_offsets):
                continue
            evidence.append(
                {
                    "direction": direction,
                    "topic": match.group(2).strip(),
                    "type": " ".join(match.group(1).split()),
                    "path": rel(path, root),
                    "line": line_number(text, match.start()),
                }
            )
    return evidence


def frame_evidence(path: Path, text: str, root: Path) -> list[dict]:
    evidence = []
    patterns = (
        CONFIG_FRAME_PATTERNS
        if path.suffix.lower() in {".yaml", ".yml"}
        else SOURCE_FRAME_PATTERNS
    )
    for pattern in patterns:
        for match in pattern.finditer(text):
            expression = match.group(1).strip().strip("\"'")
            if not expression or len(expression) > 120:
                continue
            evidence.append(
                {
                    "frame": expression,
                    "path": rel(path, root),
                    "line": line_number(text, match.start()),
                }
            )
    return evidence


def collect_member_usage(
    fields: set[str], sources: list[tuple[Path, str]]
) -> dict[str, dict]:
    usage = {field: {"field": field, "references": 0, "samples": []} for field in fields}
    pattern = re.compile(r"(?:\.|->)([A-Za-z_][A-Za-z0-9_]*)\b")
    for path, text in sources:
        for match in pattern.finditer(text):
            field = match.group(1)
            if field not in usage:
                continue
            item = usage[field]
            item["references"] += 1
            if len(item["samples"]) < 5:
                item["samples"].append(
                    {"path": str(path), "line": line_number(text, match.start())}
                )
    return usage


def inspect(root: Path, max_depth: int, max_files: int) -> dict:
    schemas = []
    endpoints = []
    frames = []
    clock_use: dict[str, list[dict]] = defaultdict(list)
    source_texts: list[tuple[Path, str]] = []
    scanned = 0

    for path, text in walk_text_files(root, max_depth, max_files):
        scanned += 1
        if path.suffix.lower() in SCHEMA_SUFFIXES:
            schemas.append(parse_schema(path, text, root))
            continue
        if path.suffix.lower() in SOURCE_SUFFIXES:
            source_texts.append((path, text))
            endpoints.extend(endpoint_evidence(path, text, root))
        frames.extend(frame_evidence(path, text, root))
        for name, pattern in CLOCK_TOKENS.items():
            for match in pattern.finditer(text):
                if len(clock_use[name]) < 50:
                    clock_use[name].append(
                        {"path": rel(path, root), "line": line_number(text, match.start())}
                    )

    unique_fields = {
        field["name"]
        for schema in schemas
        for section in schema["sections"]
        for field in section
        if not field["constant"]
    }
    source_views = [(Path(rel(path, root)), text) for path, text in source_texts]
    indexed_usage = collect_member_usage(unique_fields, source_views)
    field_usage = [indexed_usage[field] for field in sorted(indexed_usage)]

    topic_directions: dict[str, set[str]] = defaultdict(set)
    for endpoint in endpoints:
        topic_directions[endpoint["topic"]].add(endpoint["direction"])
    findings = []
    for topic, directions in sorted(topic_directions.items()):
        if len(directions) == 1:
            direction = next(iter(directions))
            missing = "subscription" if direction == "publisher" else "publisher"
            findings.append(
                {
                    "level": "info",
                    "kind": "one-sided-static-topic",
                    "message": f"{topic!r} has only {direction} evidence; {missing} may be external or dynamic",
                }
            )
    for usage in field_usage:
        if usage["references"] == 0:
            findings.append(
                {
                    "level": "info",
                    "kind": "unreferenced-schema-field",
                    "message": f"schema field {usage['field']!r} has no exact static member reference",
                }
            )

    return {
        "root": str(root),
        "method": "heuristic static inventory; confirm against runtime graph and effective configuration",
        "scanned_text_files": scanned,
        "schemas": sorted(schemas, key=lambda item: item["path"]),
        "endpoints": sorted(
            endpoints, key=lambda item: (item["topic"], item["direction"], item["path"], item["line"])
        ),
        "frames": sorted(frames, key=lambda item: (item["frame"], item["path"], item["line"])),
        "clock_use": dict(sorted(clock_use.items())),
        "field_usage": field_usage,
        "findings": findings,
    }


def render_markdown(data: dict) -> str:
    lines = [
        "# Robot Interface Contract Evidence",
        "",
        f"Root: `{data['root']}`",
        "",
        f"Method: {data['method']}.",
        "",
        "## Schemas",
        "",
    ]
    if not data["schemas"]:
        lines.append("No ROS-style `.msg`, `.srv`, or `.action` schemas found.")
    else:
        for schema in data["schemas"]:
            field_count = sum(
                1 for section in schema["sections"] for field in section if not field["constant"]
            )
            lines.append(f"- `{schema['path']}`: {field_count} data fields")

    lines.extend(["", "## Static Topic Endpoints", ""])
    if data["endpoints"]:
        lines.append("| Topic | Direction | Type | Evidence |")
        lines.append("|---|---|---|---|")
        for endpoint in data["endpoints"][:100]:
            lines.append(
                f"| `{endpoint['topic']}` | {endpoint['direction']} | `{endpoint['type']}` | "
                f"`{endpoint['path']}:{endpoint['line']}` |"
            )
        if len(data["endpoints"]) > 100:
            lines.append(f"\n{len(data['endpoints']) - 100} additional endpoints omitted from Markdown.")
    else:
        lines.append("No literal publisher/subscription endpoints recognized.")

    lines.extend(["", "## Frame Evidence", ""])
    grouped_frames: dict[str, list[str]] = defaultdict(list)
    for item in data["frames"]:
        grouped_frames[item["frame"]].append(f"{item['path']}:{item['line']}")
    if grouped_frames:
        for frame, locations in list(grouped_frames.items())[:50]:
            sample = ", ".join(f"`{location}`" for location in locations[:4])
            lines.append(f"- `{frame}`: {sample}")
    else:
        lines.append("No literal frame assignments recognized.")

    lines.extend(["", "## Clock Evidence", ""])
    if data["clock_use"]:
        for clock, locations in data["clock_use"].items():
            lines.append(f"- {clock}: {len(locations)} sampled occurrences")
    else:
        lines.append("No recognized clock tokens found.")

    lines.extend(["", "## Audit Candidates", ""])
    if data["findings"]:
        lines.extend(f"- [{item['level']}] {item['message']}" for item in data["findings"][:100])
    else:
        lines.append("No static audit candidates produced.")
    lines.extend(
        [
            "",
            "> These candidates are not defects by themselves. Confirm runtime remapping, dynamic names,",
            "> external processes, generated bindings, effective launch configuration, and recorded traffic.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    if not root.is_dir():
        print(f"error: workspace root is not a directory: {root}", file=sys.stderr)
        return 2
    if args.max_depth < 1 or args.max_files < 1:
        print("error: scan limits must be positive", file=sys.stderr)
        return 2
    data = inspect(root, args.max_depth, args.max_files)
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
