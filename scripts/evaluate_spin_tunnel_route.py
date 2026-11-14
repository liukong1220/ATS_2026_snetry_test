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
import bisect
import csv
import json
import math
from pathlib import Path

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


def motion_spin_summary(rows, rate=3.0, tolerance=0.15, min_speed=0.05, max_gap=0.25,
                        eligible=lambda row: True):
    """Report measured motion, never infer sustained spin from a peak or parked yaw."""
    # Wall-clock recorder ticks can contain the same observation many times.
    # Keep the latest received value for each observation in each route leg.
    rows = sorted({(r.get('leg'), r['t']): r for r in rows}.values(), key=lambda r: r['t'])
    moving = [r for r in rows if eligible(r) and math.hypot(r["vx"], r["vy"]) >= min_speed]
    def stable(r):
        return (eligible(r) and math.hypot(r["vx"], r["vy"]) >= min_speed and
                abs(abs(r["wz"]) - rate) <= tolerance)
    intervals, begin, previous = [], None, None
    moving_duration = stable_duration = 0.0
    for row in rows:
        dt = row["t"] - previous["t"] if previous else 0.0
        adjacent = (previous is not None and 0.0 < dt <= max_gap and
                    row.get('leg') == previous.get('leg'))
        if adjacent and all(eligible(r) and math.hypot(r["vx"], r["vy"]) >= min_speed
                            for r in (row, previous)):
            moving_duration += dt
        if stable(row):
            if adjacent and stable(previous):
                stable_duration += dt
            else:
                if begin is not None:
                    intervals.append([begin, previous["t"]])
                begin = row["t"]
        elif begin is not None:
            intervals.append([begin, previous["t"]])
            begin = None
        previous = row
    if begin is not None:
        intervals.append([begin, previous["t"]])
    def distribution(values):
        return dict(zip(("min", "p05", "median", "p95", "max"),
                        map(float, np.percentile(values, [0, 5, 50, 95, 100])))) if values else None
    return {"moving_samples": len(moving), "moving_duration_s": moving_duration,
            "moving_speed_mps": distribution([math.hypot(r["vx"], r["vy"]) for r in moving]),
            "moving_abs_wz_radps": distribution([abs(r["wz"]) for r in moving]),
            "target_rate_radps": rate, "diagnostic_tolerance_radps": tolerance,
            "minimum_translation_mps": min_speed, "maximum_sample_gap_s": max_gap,
            "near_target_duration_s": stable_duration,
            "near_target_intervals_s": intervals,
            "longest_near_target_interval_s": max((b-a for a, b in intervals), default=0.0)}


def body_intersects(row, polygon, length, width):
    """SAT on evaluator polygons; the rear corners must leave before recovery timing."""
    c, s = math.cos(row["yaw"]), math.sin(row["yaw"])
    body = np.array([[row["x"] + c*x-s*y, row["y"] + s*x+c*y]
                     for x, y in [(length/2, width/2), (-length/2, width/2),
                                  (-length/2, -width/2), (length/2, -width/2)]])
    corridor = np.asarray(polygon, dtype=float)
    for poly in (body, corridor):
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            edge = b-a
            normal = np.array([-edge[1], edge[0]])
            p, q = body @ normal, corridor @ normal
            if max(p) < min(q) or max(q) < min(p):
                return False
    return True


def load_samples(evidence: Path, truth_path: Path | None = None) -> list[dict]:
    rows = []
    truth = []
    if truth_path:
        with truth_path.open() as stream:
            truth = list(csv.DictReader(stream))
    truth_stamps = [float(r['sim_s']) for r in truth]
    if truth_path and (not truth or truth_stamps != sorted(truth_stamps)):
        raise ValueError('truth poses must have ordered simulation timestamps')
    for leg in sorted(evidence.glob("goal_*")):
        path = leg / "samples.jsonl"
        if not path.exists():
            continue
        summary = json.loads((leg / 'summary.json').read_text())
        if summary['grid_frame'] != 'map' and not truth:
            raise ValueError('odom-grid recorder transform is not map<-odom; supply --truth-csv')
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            actual = row.get("actual")
            transform = row.get("map_from_odom")
            if not actual or not transform:
                continue
            observed_at = float(actual['stamp_identity'])
            paired = None
            if truth:
                j = bisect.bisect_left(truth_stamps, observed_at)
                k = min({max(0, j-1), min(len(truth)-1, j)},
                        key=lambda i: abs(truth_stamps[i]-observed_at))
                if abs(truth_stamps[k]-observed_at) > .05:
                    raise ValueError('truth pose exceeds 50 ms pairing bound')
                paired = truth[k]
            c, s = math.cos(transform["yaw"]), math.sin(transform["yaw"])
            rows.append({
                "leg": leg.name,
                "t": observed_at,
                "x": float(paired['x']) if paired else transform["x"] + c * actual["x"] - s * actual["y"],
                "y": float(paired['y']) if paired else transform["y"] + s * actual["x"] + c * actual["y"],
                "yaw": float(paired['yaw']) if paired else math.atan2(
                    math.sin(transform["yaw"] + actual["yaw"]),
                    math.cos(transform["yaw"] + actual["yaw"])),
                "pose_pairing_delta_s": abs(truth_stamps[k]-observed_at) if paired else None,
                # MuJoCo 遥测的 measured_wz 优先：Point-LIO odom 的 wz_body 在 domain 124 出现过
                # ±120 rad/s 的单帧尖峰，而同一时刻 measured_wz 约 1.2 rad/s。
                "wz": float((row.get("telemetry") or {}).get("measured_wz", actual.get("wz_body", 0.0))),
                "vx": float((row.get("telemetry") or {}).get("measured_vx", actual.get("vx_body", 0.0))),
                "vy": float((row.get("telemetry") or {}).get("measured_vy", actual.get("vy_body", 0.0))),
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
    parser.add_argument("--spin-rate", type=float, default=3.0)
    parser.add_argument("--spin-tolerance", type=float, default=0.15,
                        help="diagnostic band; raw distribution is always reported, not an acceptance override")
    parser.add_argument("--min-translation", type=float, default=0.05)
    parser.add_argument("--profile-yaml", type=Path, default=CONFIG / "rmuc_2025_navigation.yaml",
                        help="effective footprint source; 2027 inherits these dimensions")
    parser.add_argument("--map-yaml", type=Path, default=MAP_YAML)
    parser.add_argument("--truth-csv", type=Path,
                        help="S3 truth.csv for map geometry; required when recorder grid-frame=odom")
    args = parser.parse_args()

    rows = load_samples(args.evidence, args.truth_csv)
    if not rows:
        print("no samples")
        return 2
    import cv2
    dimensions = yaml.safe_load(args.profile_yaml.read_text())["minco_planner"]["ros__parameters"]
    length, width = dimensions["footprint_length"], dimensions["footprint_width"]
    meta = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    image = cv2.imread(str(args.map_yaml.parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
    resolution = float(meta["resolution"])
    clearance = cv2.distanceTransform((image >= 200).astype(np.uint8), cv2.DIST_L2, 5) * resolution

    def centre_clearance(x: float, y: float) -> float:
        ox, oy, heading = meta["origin"]
        c, s = math.cos(heading), math.sin(heading)
        gx, gy = c*(x-ox)+s*(y-oy), -s*(x-ox)+c*(y-oy)
        row = image.shape[0] - 1 - math.floor(gy / resolution)
        col = math.floor(gx / resolution)
        if 0 <= row < image.shape[0] and 0 <= col < image.shape[1]:
            return float(clearance[row, col])
        return 0.0

    authorities: dict[str, int] = {}
    for r in rows:
        authorities[str(r["authority"])] = authorities.get(str(r["authority"]), 0) + 1
    for r in rows:
        r['open'] = centre_clearance(r["x"], r["y"]) >= args.open_clearance
    open_rows = [r for r in rows if r['open']]
    spinning_open = sum(abs(r["wz"]) >= args.spin_threshold for r in open_rows)
    result = {
        "samples": len(rows),
        "pose_source": str(args.truth_csv) if args.truth_csv else 'recorded map<-odom',
        "max_pose_pairing_delta_s": max((r['pose_pairing_delta_s'] or 0.0 for r in rows)),
        "sim_time_s": [rows[0]["t"], rows[-1]["t"]],
        "yaw_authority_samples": authorities,
        "max_abs_wz": max(abs(r["wz"]) for r in rows),
        "open_samples": len(open_rows),
        "legacy_open_spinning_fraction_including_stationary": spinning_open / len(open_rows) if open_rows else None,
        "open_translation_spin": motion_spin_summary(rows, args.spin_rate,
            args.spin_tolerance, args.min_translation, eligible=lambda r: r['open']),
        "footprint_source": str(args.profile_yaml), "body_dimensions_m": [length, width],
        "tunnels": {},
        "physical_contact": "not evaluated here",
    }
    for tunnel in load_tunnels():
        polygon = tunnel["corridor_polygon"]
        flags = [inside(r["x"], r["y"], polygon) for r in rows]
        if not any(flags):
            continue
        inside_rows = [r for r, f in zip(rows, flags) if f]
        body_flags = [body_intersects(r, polygon, length, width) for r in rows]
        recoveries = []
        for i in range(1, len(rows)):
            if not body_flags[i-1] or body_flags[i]:
                continue
            exit_time = rows[i]["t"]
            following = []
            for j in range(i, len(rows)):
                r = rows[j]
                if r["t"] - exit_time > args.resume_window or body_flags[j]:
                    break
                following.append(r)
            motion = motion_spin_summary(following, args.spin_rate,
                                         args.spin_tolerance, args.min_translation)
            intervals = motion["near_target_intervals_s"]
            recoveries.append({"whole_body_exit_s": exit_time,
                "first_near_target_delay_s": intervals[0][0]-exit_time if intervals else None,
                "post_exit_translation_spin": motion})
        result["tunnels"][tunnel["name"]] = {
            "samples_inside": len(inside_rows),
            "max_abs_wz_inside": max(abs(r["wz"]) for r in inside_rows),
            "max_axis_yaw_error_rad": max(axis_error(r["yaw"], float(tunnel["axis_yaw"])) for r in inside_rows),
            # 0.581 x 0.571 m 的底盘近似正方形，转 90 度后宽度只差 1 cm，所以也给出模 pi/2 的误差。
            "max_axis_yaw_error_mod_half_pi_rad": max(
                axis_error(2.0 * r["yaw"], 2.0 * float(tunnel["axis_yaw"])) / 2.0 for r in inside_rows),
            "spinning_samples_inside": sum(abs(r["wz"]) >= args.spin_threshold for r in inside_rows),
            "whole_body_exit_recoveries": recoveries,
        }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
