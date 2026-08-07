#!/usr/bin/env python3
"""Read the MPC telemetry service and atomically preserve one raw experiment artifact."""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def git_revision(path: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--solver-mode", required=True)
    parser.add_argument("--log-level", required=True)
    parser.add_argument("--test-profile", required=True)
    parser.add_argument("--planning-grid-owner", required=True)
    parser.add_argument("--p2-fault-case", required=True)
    parser.add_argument("--p3-fault-case", required=True)
    parser.add_argument("--ros-domain-id", required=True, type=int)
    parser.add_argument("--params-file", required=True)
    parser.add_argument("--start-x", required=True, type=float)
    parser.add_argument("--start-y", required=True, type=float)
    parser.add_argument("--start-z", required=True, type=float)
    parser.add_argument("--start-yaw", required=True, type=float)
    parser.add_argument("--goal-x", required=True, type=float)
    parser.add_argument("--goal-y", required=True, type=float)
    parser.add_argument("--goal-yaw-w", required=True, type=float)
    parser.add_argument("--run-start-epoch-ns", required=True, type=int)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    rclpy.init()
    node = Node("ats_swerve_mpc_telemetry_dump")
    try:
        client = node.create_client(Trigger, "/ats_swerve_mpc/dump_control_telemetry")
        if not client.wait_for_service(timeout_sec=args.timeout_sec):
            raise RuntimeError("telemetry service is unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_sec)
        response = future.result()
        if response is None or not response.success:
            detail = "no response" if response is None else response.message
            raise RuntimeError(f"telemetry service rejected dump: {detail}")
        payload = json.loads(response.message)
        if payload.get("schema_version") != 2 or not isinstance(payload.get("samples"), list):
            raise RuntimeError("telemetry service returned an unsupported schema")
        write_json(args.output, payload)
        manifest = {
            "schema_version": 1,
            "revisions": {
                "root": git_revision(args.workspace),
                "navigation": git_revision(args.workspace / "src/ats_sentry_nav"),
                "mujoco": git_revision(args.workspace / "src/sim/ats_mujoco_sim"),
            },
            "runtime": {
                "ros_domain_id": args.ros_domain_id,
                "solver_mode": args.solver_mode,
                "log_level": args.log_level,
                "use_sim_time": payload.get("metadata", {}).get("use_sim_time"),
                "control_rate_hz": payload.get("metadata", {}).get("control_rate_hz"),
                "control_period_ms": payload.get("metadata", {}).get("control_period_ms"),
                "qp_time_limit_ms": payload.get("metadata", {}).get("qp_time_limit_ms"),
                "params_file": args.params_file,
                "duration_ms_at_dump": (time.time_ns() - args.run_start_epoch_ns) / 1_000_000.0,
            },
            "scenario": {
                "test_profile": args.test_profile,
                "planning_grid_owner": args.planning_grid_owner,
                "p2_fault_case": args.p2_fault_case,
                "p3_fault_case": args.p3_fault_case,
                "initial_pose": [args.start_x, args.start_y, args.start_z, args.start_yaw],
                "first_goal": [args.goal_x, args.goal_y, args.goal_yaw_w],
            },
            "telemetry_file": str(args.output),
        }
        write_json(args.manifest, manifest)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
