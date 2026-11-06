#!/usr/bin/env python3
"""Check 小陀螺 and tunnel yaw alignment on a recorded MuJoCo route.

Reads every goal leg of a nav-tracking evidence directory (samples.jsonl from
scripts/nav_tracking_recorder), puts the chassis pose in the map frame with the
recorded map_from_odom, and reports for each of the 6 tunnels of the 2027 field
(tunnels_2027.yaml + passages_2027.yaml):

- whether the chassis centre entered the corridor and the maximum |yaw rate|
  and yaw error to the axis (modulo pi, the footprint is symmetric) inside it;
- the first time after leaving the corridor at which |yaw rate| reached the
  spin threshold again.

It also reports the share of samples in open space (PGM centre clearance at
least --open-clearance) with |yaw rate| >= --spin-threshold.  The yaw rate is
the MuJoCo telemetry measured_wz (fallback: odometry wz_body), so it does not depend on frames.

Physical contact is not evaluated here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/sim/ats_mujoco_sim/config"
MAP_YAML = ROOT / "src/ats_sentry_bringup/map/rmuc_2027.yaml"


def load_tunnels() -> list[dict]:
    tunnels = []
    for name in ("tunnels_2027.yaml", "passages_2027.yaml"):
        tunnels += yaml.safe_load((CONFIG / name).read_text(encoding="utf-8")).get("tunnels", [])
    return tunnels


def inside(x: float, y: float, polygon: list[list[float]]) -> bool:
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)


def axis_error(yaw: float, axis: float) -> float:
    return abs(0.5 * math.atan2(math.sin(2.0 * (yaw - axis)), math.cos(2.0 * (yaw - axis))))


def load_samples(evidence: Path) -> list[dict]:
    rows = []
    for leg in sorted(evidence.glob("goal_*")):
        path = leg / "samples.jsonl"
        if not path.exists():
            continue
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            actual = row.get("actual")
            transform = row.get("map_from_odom")
            if not actual or not transform:
                continue
            c, s = math.cos(transform["yaw"]), math.sin(transform["yaw"])
            rows.append({
                "leg": leg.name,
                "t": float(actual["stamp_identity"]),
                "x": transform["x"] + c * actual["x"] - s * actual["y"],
                "y": transform["y"] + s * actual["x"] + c * actual["y"],
                "yaw": math.atan2(math.sin(transform["yaw"] + actual["yaw"]),
                                  math.cos(transform["yaw"] + actual["yaw"])),
                # MuJoCo 遥测的 measured_wz 优先：Point-LIO odom 的 wz_body 在 domain 124 出现过
                # ±120 rad/s 的单帧尖峰，而同一时刻 measured_wz 约 1.2 rad/s。
                "wz": float((row.get("telemetry") or {}).get("measured_wz", actual.get("wz_body", 0.0))),
                "authority": (row.get("execution") or {}).get("yaw_authority"),
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence", type=Path, help="/tmp/ats_nav_evidence/nav_tracking_<profile>_<domain>")
    parser.add_argument("--spin-threshold", type=float, default=1.5, help="|wz| counted as spinning, rad/s")
    parser.add_argument("--open-clearance", type=float, default=0.8, help="PGM centre clearance of open space, m")
    parser.add_argument("--resume-window", type=float, default=15.0, help="sim seconds after a corridor exit")
    args = parser.parse_args()

    rows = load_samples(args.evidence)
    if not rows:
        print("no samples")
        return 2
    meta = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
    image = cv2.imread(str(MAP_YAML.parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
    resolution = float(meta["resolution"])
    clearance = cv2.distanceTransform((image >= 200).astype(np.uint8), cv2.DIST_L2, 5) * resolution

    def centre_clearance(x: float, y: float) -> float:
        row = image.shape[0] - 1 - int(y / resolution)
        col = int(x / resolution)
        if 0 <= row < image.shape[0] and 0 <= col < image.shape[1]:
            return float(clearance[row, col])
        return 0.0

    authorities: dict[str, int] = {}
    for r in rows:
        authorities[str(r["authority"])] = authorities.get(str(r["authority"]), 0) + 1
    open_rows = [r for r in rows if centre_clearance(r["x"], r["y"]) >= args.open_clearance]
    spinning_open = sum(abs(r["wz"]) >= args.spin_threshold for r in open_rows)
    result = {
        "samples": len(rows),
        "sim_time_s": [rows[0]["t"], rows[-1]["t"]],
        "yaw_authority_samples": authorities,
        "max_abs_wz": max(abs(r["wz"]) for r in rows),
        "open_samples": len(open_rows),
        "open_spinning_fraction": spinning_open / len(open_rows) if open_rows else None,
        "tunnels": {},
        "physical_contact": "not evaluated here",
    }
    for tunnel in load_tunnels():
        polygon = tunnel["corridor_polygon"]
        flags = [inside(r["x"], r["y"], polygon) for r in rows]
        if not any(flags):
            continue
        inside_rows = [r for r, f in zip(rows, flags) if f]
        last = max(i for i, f in enumerate(flags) if f)
        exit_time = rows[last]["t"]
        resumed = next((r["t"] - exit_time for r in rows[last + 1:]
                        if r["t"] - exit_time <= args.resume_window and abs(r["wz"]) >= args.spin_threshold),
                       None)
        result["tunnels"][tunnel["name"]] = {
            "samples_inside": len(inside_rows),
            "max_abs_wz_inside": max(abs(r["wz"]) for r in inside_rows),
            "max_axis_yaw_error_rad": max(axis_error(r["yaw"], float(tunnel["axis_yaw"])) for r in inside_rows),
            # 0.581 x 0.571 m 的底盘近似正方形，转 90 度后宽度只差 1 cm，所以也给出模 pi/2 的误差。
            "max_axis_yaw_error_mod_half_pi_rad": max(
                axis_error(2.0 * r["yaw"], 2.0 * float(tunnel["axis_yaw"])) / 2.0 for r in inside_rows),
            "spinning_samples_inside": sum(abs(r["wz"]) >= args.spin_threshold for r in inside_rows),
            "spin_resumed_after_exit_s": resumed,
        }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
