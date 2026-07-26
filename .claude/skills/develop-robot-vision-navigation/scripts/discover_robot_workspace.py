#!/usr/bin/env python3
"""Inventory a robotics workspace without assuming ROS or a repository layout."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


IGNORED_DIRS = {
    ".git",
    ".cache",
    ".idea",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".vscode",
    "__pycache__",
    "build",
    "coverage",
    "dist",
    "install",
    "log",
    "logs",
    "node_modules",
    "target",
    "vendor",
}

MANIFEST_NAMES = {
    "BUILD",
    "BUILD.bazel",
    "CMakeLists.txt",
    "Cargo.toml",
    "Dockerfile",
    "Makefile",
    "WORKSPACE",
    "WORKSPACE.bazel",
    "colcon.meta",
    "package.json",
    "package.xml",
    "pyproject.toml",
    "setup.cfg",
    "setup.py",
}

INSTRUCTION_NAMES = {"AGENTS.md", "CLAUDE.md", "CONTRIBUTING.md"}
INTERFACE_SUFFIXES = {".action", ".idl", ".msg", ".proto", ".srv"}
CONFIG_SUFFIXES = {".json", ".toml", ".yaml", ".yml"}
SOURCE_SUFFIXES = {
    ".c": "C",
    ".cc": "C++",
    ".cpp": "C++",
    ".cu": "CUDA",
    ".cuh": "CUDA",
    ".h": "C/C++ header",
    ".hh": "C++ header",
    ".hpp": "C++ header",
    ".java": "Java",
    ".js": "JavaScript",
    ".m": "MATLAB/Objective-C",
    ".py": "Python",
    ".rs": "Rust",
    ".sh": "Shell",
    ".ts": "TypeScript",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".", help="Workspace root to inspect")
    parser.add_argument("--format", choices=("json", "markdown"), default="markdown")
    parser.add_argument("--max-depth", type=int, default=6)
    parser.add_argument("--max-files", type=int, default=30000)
    parser.add_argument("--output", help="Write output to this path instead of stdout")
    return parser.parse_args()


def relative(path: Path, root: Path) -> str:
    try:
        value = path.relative_to(root).as_posix()
        return value or "."
    except ValueError:
        return str(path)


def is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def contains_program_entrypoint(path: Path) -> bool:
    if path.suffix.lower() not in SOURCE_SUFFIXES or path.stat().st_size > 2_000_000:
        return False
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    if path.suffix.lower() in {".c", ".cc", ".cpp", ".cu"}:
        return re.search(r"\b(?:int|auto)\s+main\s*\(", text) is not None
    if path.suffix.lower() == ".py":
        return re.search(r"if\s+__name__\s*==\s*[\"']__main__[\"']", text) is not None
    return False


def walk_files(root: Path, max_depth: int, max_files: int):
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
            if path.is_symlink():
                continue
            seen += 1
            yield path


def git_metadata(repo: Path) -> dict:
    def run(*args: str) -> str:
        result = subprocess.run(
            ["git", "-C", str(repo), *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
        )
        return result.stdout.strip()

    status = run("status", "--porcelain=v1")
    return {
        "path": str(repo),
        "branch": run("branch", "--show-current") or None,
        "head": run("rev-parse", "--short", "HEAD") or None,
        "dirty": bool(status),
        "changed_entries": len(status.splitlines()) if status else 0,
        "remotes": [line for line in run("remote", "-v").splitlines() if line],
    }


def parse_ros_package(path: Path) -> dict:
    result = {"path": str(path), "name": None, "version": None, "dependencies": []}
    try:
        root = ET.parse(path).getroot()
        result["name"] = root.findtext("name")
        result["version"] = root.findtext("version")
        dependency_tags = {
            "depend",
            "build_depend",
            "buildtool_depend",
            "exec_depend",
            "run_depend",
            "test_depend",
        }
        result["dependencies"] = sorted(
            {
                (element.text or "").strip()
                for element in root
                if element.tag in dependency_tags and (element.text or "").strip()
            }
        )
    except (ET.ParseError, OSError) as error:
        result["error"] = str(error)
    return result


def infer_build_systems(manifests: list[str]) -> list[str]:
    names = {Path(path).name for path in manifests}
    systems = []
    for marker, label in (
        ("package.xml", "ROS package manifest"),
        ("CMakeLists.txt", "CMake"),
        ("pyproject.toml", "Python/pyproject"),
        ("setup.py", "Python/setuptools"),
        ("Cargo.toml", "Cargo"),
        ("package.json", "Node.js"),
        ("BUILD", "Bazel"),
        ("BUILD.bazel", "Bazel"),
        ("Makefile", "Make"),
        ("Dockerfile", "Container"),
    ):
        if marker in names and label not in systems:
            systems.append(label)
    return systems


def inventory(root: Path, max_depth: int, max_files: int) -> dict:
    manifests: list[str] = []
    instructions: list[str] = []
    interfaces: list[str] = []
    configs: list[str] = []
    tests: list[str] = []
    entrypoints: list[str] = []
    repositories: set[Path] = set()
    languages: dict[str, int] = {}
    ros_package_paths: list[Path] = []
    scanned = 0

    if (root / ".git").exists():
        repositories.add(root)

    for path in walk_files(root, max_depth, max_files):
        scanned += 1
        rel = relative(path, root)
        if path.name in MANIFEST_NAMES:
            manifests.append(rel)
            if path.name == "package.xml":
                ros_package_paths.append(path)
        if path.name in INSTRUCTION_NAMES:
            instructions.append(rel)
        if path.suffix.lower() in INTERFACE_SUFFIXES:
            interfaces.append(rel)
        if path.suffix.lower() in CONFIG_SUFFIXES:
            configs.append(rel)
        lower_parts = {part.lower() for part in path.parts}
        lower_name = path.name.lower()
        is_test_path = (
            "test" in lower_parts or "tests" in lower_parts or lower_name.startswith("test_")
        )
        if is_test_path:
            tests.append(rel)
        if "launch" in lower_parts or (not is_test_path and contains_program_entrypoint(path)):
            entrypoints.append(rel)
        language = SOURCE_SUFFIXES.get(path.suffix.lower())
        if language:
            languages[language] = languages.get(language, 0) + 1

        parent = path.parent
        while parent != root and is_within(parent, root):
            if (parent / ".git").exists():
                repositories.add(parent)
                break
            parent = parent.parent

    repo_data = []
    for repo in sorted(repositories, key=lambda item: str(item)):
        data = git_metadata(repo)
        data["path"] = relative(repo, root)
        repo_data.append(data)

    ros_packages = []
    for path in sorted(ros_package_paths):
        data = parse_ros_package(path)
        data["path"] = relative(path, root)
        ros_packages.append(data)

    return {
        "root": str(root),
        "scan_limits": {"max_depth": max_depth, "max_files": max_files},
        "scanned_files": scanned,
        "truncated": scanned >= max_files,
        "repositories": repo_data,
        "instructions": sorted(instructions),
        "build_systems": infer_build_systems(manifests),
        "manifests": sorted(manifests),
        "ros_packages": ros_packages,
        "entrypoints": sorted(entrypoints),
        "interfaces": sorted(interfaces),
        "configs": sorted(configs),
        "tests": sorted(tests),
        "languages": dict(sorted(languages.items(), key=lambda item: (-item[1], item[0]))),
    }


def limited(items: list, limit: int = 30) -> list:
    return items[:limit]


def render_markdown(data: dict) -> str:
    lines = ["# Robot Workspace Inventory", "", f"Root: `{data['root']}`", ""]
    lines.append(
        f"Scanned {data['scanned_files']} files"
        + (" (limit reached)" if data["truncated"] else "")
        + "."
    )
    lines.extend(["", "## Source Ownership", ""])
    if data["repositories"]:
        lines.append("| Repository | Branch | HEAD | Dirty | Changes |")
        lines.append("|---|---|---|---:|---:|")
        for repo in data["repositories"]:
            lines.append(
                f"| `{repo['path']}` | {repo['branch'] or '-'} | {repo['head'] or '-'} | "
                f"{str(repo['dirty']).lower()} | {repo['changed_entries']} |"
            )
    else:
        lines.append("No Git repository found within the scan depth.")

    lines.extend(["", "## Build And Runtime", ""])
    lines.append("Build systems: " + (", ".join(data["build_systems"]) or "unknown"))
    for title, key in (
        ("Instructions", "instructions"),
        ("Manifests", "manifests"),
        ("Entrypoints", "entrypoints"),
        ("Interfaces", "interfaces"),
        ("Tests", "tests"),
    ):
        lines.extend(["", f"### {title}", ""])
        values = data[key]
        if not values:
            lines.append("None found within the scan limits.")
        else:
            lines.extend(f"- `{value}`" for value in limited(values))
            if len(values) > 30:
                lines.append(f"- ... {len(values) - 30} more")

    lines.extend(["", "## Languages", ""])
    if data["languages"]:
        lines.extend(f"- {name}: {count}" for name, count in data["languages"].items())
    else:
        lines.append("No recognized source suffixes found.")

    if data["ros_packages"]:
        lines.extend(["", "## ROS Package Manifests", ""])
        lines.append("| Package | Version | Manifest |")
        lines.append("|---|---|---|")
        for package in limited(data["ros_packages"], 50):
            lines.append(
                f"| {package.get('name') or '?'} | {package.get('version') or '?'} | "
                f"`{package['path']}` |"
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

    data = inventory(root, args.max_depth, args.max_files)
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
