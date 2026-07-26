#!/usr/bin/env python3
"""Self-tests for the portable Skill helper scripts."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile


SCRIPT_DIR = Path(__file__).resolve().parent


def run(script: str, *args: str, expected: int = 0) -> subprocess.CompletedProcess:
    result = subprocess.run(
        [sys.executable, str(SCRIPT_DIR / script), *args],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=20,
    )
    if result.returncode != expected:
        raise AssertionError(
            f"{script} returned {result.returncode}, expected {expected}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result


def write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def make_workspace(root: Path) -> None:
    write(root / "AGENTS.md", "# Test instructions\n")
    write(
        root / "package.xml",
        """<package format="3">
  <name>portable_robot</name><version>0.1.0</version>
  <description>fixture</description><maintainer email="a@b.c">A</maintainer>
  <license>Apache-2.0</license><depend>geometry_msgs</depend>
</package>
""",
    )
    write(root / "CMakeLists.txt", "cmake_minimum_required(VERSION 3.16)\n")
    write(
        root / "interfaces" / "TrackedObject.msg",
        """builtin_interfaces/Time timestamp
bool valid
float32 confidence
geometry_msgs/Point position
string frame_id
""",
    )
    write(
        root / "src" / "tracker_node.cpp",
        """void setup() {
  auto p = create_publisher<robot_msgs::msg::TrackedObject>("tracking/object", 10);
  message.valid = true;
  message.confidence = 0.8;
  message.position.x = 1.0;
  message.frame_id = "camera";
  header.frame_id = "camera";
  auto t = std::chrono::steady_clock::now();
}
""",
    )
    write(
        root / "nav" / "consumer.py",
        """node.create_subscription(TrackedObject, "tracking/object", callback, 10)
msg.valid
msg.confidence
msg.position
msg.frame_id
""",
    )
    write(root / "launch" / "robot.launch.py", "def generate_launch_description(): pass\n")
    write(root / "tests" / "test_tracker.py", "def test_contract(): assert True\n")
    write(root / "build" / "generated.cpp", "int ignored = 1;\n")


def test_workspace_discovery(root: Path) -> None:
    result = run(
        "discover_robot_workspace.py", "--root", str(root), "--format", "json", "--max-depth", "8"
    )
    data = json.loads(result.stdout)
    assert "CMake" in data["build_systems"]
    assert data["ros_packages"][0]["name"] == "portable_robot"
    assert "interfaces/TrackedObject.msg" in data["interfaces"]
    assert "launch/robot.launch.py" in data["entrypoints"]
    assert not any("build/generated.cpp" in path for path in data["entrypoints"])


def test_contract_inventory(root: Path) -> None:
    result = run(
        "check_robot_interface_contracts.py", "--root", str(root), "--format", "json"
    )
    data = json.loads(result.stdout)
    assert len(data["schemas"]) == 1
    directions = {(item["topic"], item["direction"]) for item in data["endpoints"]}
    assert ("tracking/object", "publisher") in directions
    assert ("tracking/object", "subscription") in directions
    usage = {item["field"]: item["references"] for item in data["field_usage"]}
    assert usage["valid"] >= 2
    assert any(item["frame"] == "camera" for item in data["frames"])
    assert data["clock_use"]["steady_clock"]


def test_metric_analysis(root: Path) -> tuple[Path, Path]:
    csv_path = root / "timing.csv"
    write(csv_path, "latency_ms,mode\n1,track\n2,track\n3,track\n100,track\nbad,track\n")
    baseline_path = root / "baseline.json"
    result = run(
        "analyze_timing_metrics.py",
        str(csv_path),
        "--column",
        "latency_ms",
        "--deadline",
        "10",
        "--format",
        "json",
        "--output",
        str(baseline_path),
    )
    assert result.stdout == ""
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    summary = baseline["groups"]["all"]["latency_ms"]
    assert summary["count"] == 4
    assert summary["invalid_or_missing"] == 1
    assert summary["deadline_misses"] == 1

    candidate = json.loads(json.dumps(baseline))
    candidate["groups"]["all"]["latency_ms"]["mean"] *= 0.5
    candidate["groups"]["all"]["latency_ms"]["p95"] *= 0.5
    candidate_path = root / "candidate.json"
    candidate_path.write_text(json.dumps(candidate), encoding="utf-8")
    return baseline_path, candidate_path


def test_experiment_comparison(baseline: Path, candidate: Path) -> None:
    prefix = "groups.all.latency_ms"
    result = run(
        "compare_experiments.py",
        str(baseline),
        str(candidate),
        "--prefix",
        prefix,
        "--format",
        "json",
    )
    data = json.loads(result.stdout)
    outcomes = {item["key"]: item["outcome"] for item in data["comparisons"]}
    assert outcomes[f"{prefix}.mean"] == "improvement"
    assert outcomes[f"{prefix}.p95"] == "improvement"

    run(
        "compare_experiments.py",
        str(candidate),
        str(baseline),
        "--prefix",
        prefix,
        "--fail-on-regression",
        "--format",
        "json",
        expected=3,
    )


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="robot-skill-tools-") as temp:
        root = Path(temp)
        workspace = root / "unfamiliar-layout" / "robot_stack"
        make_workspace(workspace)
        test_workspace_discovery(workspace)
        test_contract_inventory(workspace)
        baseline, candidate = test_metric_analysis(root)
        test_experiment_comparison(baseline, candidate)
    print("portable helper script tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
