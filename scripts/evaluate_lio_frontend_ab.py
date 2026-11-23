#!/usr/bin/env python3
"""Offline Point-LIO / localization frontend A/B evaluation utility (O5d).

Computes Absolute Pose Error (APE), Relative Pose Error (RPE), high-rate spin
drift, odometry publication frequency, and Point-LIO scan processing wall-clock
percentiles from recorded MuJoCo / rosbag2 SQLite3 logs or CSV/JSON trajectories,
without linking or copying any GPL-2.0 localization code.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sqlite3
import struct
from typing import Dict, List, Optional, Sequence, Tuple


PoseRow = Tuple[float, float, float, float, float]  # (t_sec, x, y, z, yaw)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = max(0.0, min(float(len(ordered) - 1), (pct / 100.0) * (len(ordered) - 1)))
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    alpha = rank - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def parse_odometry_cdr(blob: bytes) -> Optional[PoseRow]:
    """Minimal FastCDR decoder for nav_msgs/msg/Odometry header.stamp + pose.pose."""
    if len(blob) < 88:
        return None
    try:
        # 4-byte CDR encapsulation header + int32 sec + uint32 nanosec + string frame_id
        sec, nanosec, frame_len = struct.unpack_from("<iII", blob, 4)
        offset = 16 + frame_len
        offset = (offset + 3) & ~3
        (child_len,) = struct.unpack_from("<I", blob, offset)
        offset += 4 + child_len
        offset = (offset + 7) & ~7
        px, py, pz, qx, qy, qz, qw = struct.unpack_from("<7d", blob, offset)
        t_sec = float(sec) + float(nanosec) * 1e-9
        if not all(math.isfinite(v) for v in (t_sec, px, py, pz, qx, qy, qz, qw)):
            return None
        return (t_sec, px, py, pz, quat_to_yaw(qx, qy, qz, qw))
    except struct.error:
        return None


def load_rosbag_sqlite(db_path: pathlib.Path, est_topic: str, gt_topic: str) -> Tuple[List[PoseRow], List[PoseRow]]:
    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topic_ids = {name: tid for tid, name in cur.fetchall()}
    est_rows: List[PoseRow] = []
    gt_rows: List[PoseRow] = []
    if est_topic in topic_ids:
        for (data,) in cur.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topic_ids[est_topic],),
        ):
            parsed = parse_odometry_cdr(data)
            if parsed is not None:
                est_rows.append(parsed)
    if gt_topic in topic_ids:
        for (data,) in cur.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topic_ids[gt_topic],),
        ):
            parsed = parse_odometry_cdr(data)
            if parsed is not None:
                gt_rows.append(parsed)
    conn.close()
    return est_rows, gt_rows


def load_pose_json_or_csv(path: pathlib.Path) -> List[PoseRow]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        payload = json.loads(text)
        rows = payload.get("poses", payload if isinstance(payload, list) else [])
        return [
            (float(r["t"]), float(r["x"]), float(r["y"]), float(r.get("z", 0.0)), float(r["yaw"]))
            for r in rows
        ]
    out: List[PoseRow] = []
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#") or s[0].isalpha():
            continue
        parts = [p.strip() for p in s.split(",")]
        if len(parts) >= 5:
            out.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])))
        elif len(parts) == 4:
            out.append((float(parts[0]), float(parts[1]), float(parts[2]), 0.0, float(parts[3])))
    return out


def parse_lio_log_latencies(log_path: Optional[pathlib.Path]) -> List[float]:
    if log_path is None or not log_path.exists():
        return []
    pattern = re.compile(r"\[point_lio 帧统计\]\s+wall_ms=([0-9.]+)")
    values: List[float] = []
    for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = pattern.search(line)
        if match:
            values.append(float(match.group(1)))
    return values


def interpolate_gt(gt: Sequence[PoseRow], t: float, max_gap_s: float = 0.10) -> Optional[PoseRow]:
    if len(gt) < 2 or t < gt[0][0] or t > gt[-1][0]:
        return None
    lo, hi = 0, len(gt) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if gt[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = gt[lo], gt[hi]
    dt = b[0] - a[0]
    if dt <= 0.0 or dt > max_gap_s:
        return None
    u = (t - a[0]) / dt
    yaw = wrap_angle(a[4] + u * wrap_angle(b[4] - a[4]))
    return (
        t,
        a[1] + u * (b[1] - a[1]),
        a[2] + u * (b[2] - a[2]),
        a[3] + u * (b[3] - a[3]),
        yaw,
    )


def evaluate_run(
    est: Sequence[PoseRow], gt: Sequence[PoseRow], wall_ms: Sequence[float], align_origin: bool = True
) -> Dict[str, float]:
    paired: List[Tuple[PoseRow, PoseRow]] = []
    for sample in est:
        matched = interpolate_gt(gt, sample[0])
        if matched is not None:
            paired.append((sample, matched))

    dts = [est[i][0] - est[i - 1][0] for i in range(1, len(est)) if est[i][0] > est[i - 1][0]]
    mean_rate_hz = (1.0 / (sum(dts) / len(dts))) if dts else 0.0

    if not paired:
        return {
            "samples": float(len(est)),
            "paired_samples": 0.0,
            "odom_rate_hz": mean_rate_hz,
            "ape_xy_rmse_m": float("nan"),
            "ape_xy_p95_m": float("nan"),
            "ape_xy_max_m": float("nan"),
            "ape_yaw_rmse_rad": float("nan"),
            "rpe_1m_xy_rmse_m": float("nan"),
            "spin_ape_xy_p95_m": float("nan"),
            "lio_wall_p50_ms": percentile(wall_ms, 50.0),
            "lio_wall_p95_ms": percentile(wall_ms, 95.0),
            "lio_wall_max_ms": max(wall_ms) if wall_ms else 0.0,
        }

    # Anchor first paired SE(2) pose when evaluating pure odometry (/odometry vs map GT).
    e0, g0 = paired[0]
    dyaw0 = wrap_angle(g0[4] - e0[4]) if align_origin else 0.0
    c0, s0 = math.cos(dyaw0), math.sin(dyaw0)

    def to_aligned(p: PoseRow) -> Tuple[float, float, float]:
        if not align_origin:
            return (p[1], p[2], p[4])
        dx = p[1] - e0[1]
        dy = p[2] - e0[2]
        return (g0[1] + c0 * dx - s0 * dy, g0[2] + s0 * dx + c0 * dy, wrap_angle(p[4] + dyaw0))

    ape_xy: List[float] = []
    ape_yaw: List[float] = []
    spin_ape_xy: List[float] = []
    for idx, (e, g) in enumerate(paired):
        ax, ay, ayaw = to_aligned(e)
        err_xy = math.hypot(ax - g[1], ay - g[2])
        err_yaw = abs(wrap_angle(ayaw - g[4]))
        ape_xy.append(err_xy)
        ape_yaw.append(err_yaw)
        if idx > 0:
            prev_g = paired[idx - 1][1]
            dt = g[0] - prev_g[0]
            if dt > 1e-3 and abs(wrap_angle(g[4] - prev_g[4])) / dt >= 1.5:
                spin_ape_xy.append(err_xy)

    rpe_xy: List[float] = []
    j = 0
    for i in range(len(paired)):
        while j < len(paired) and math.hypot(
            paired[j][1][1] - paired[i][1][1], paired[j][1][2] - paired[i][1][2]
        ) < 1.0:
            j += 1
        if j < len(paired):
            ei_x, ei_y, ei_yaw = to_aligned(paired[i][0])
            ej_x, ej_y, _ = to_aligned(paired[j][0])
            gi, gj = paired[i][1], paired[j][1]
            ci, si = math.cos(-ei_yaw), math.sin(-ei_yaw)
            rel_e = (ci * (ej_x - ei_x) - si * (ej_y - ei_y), si * (ej_x - ei_x) + ci * (ej_y - ei_y))
            cg, sg = math.cos(-gi[4]), math.sin(-gi[4])
            rel_g = (cg * (gj[1] - gi[1]) - sg * (gj[2] - gi[2]), sg * (gj[1] - gi[1]) + cg * (gj[2] - gi[2]))
            rpe_xy.append(math.hypot(rel_e[0] - rel_g[0], rel_e[1] - rel_g[1]))

    rmse = lambda vals: math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0
    return {
        "samples": float(len(est)),
        "paired_samples": float(len(paired)),
        "odom_rate_hz": mean_rate_hz,
        "ape_xy_rmse_m": rmse(ape_xy),
        "ape_xy_p95_m": percentile(ape_xy, 95.0),
        "ape_xy_max_m": max(ape_xy),
        "ape_yaw_rmse_rad": rmse(ape_yaw),
        "rpe_1m_xy_rmse_m": rmse(rpe_xy),
        "spin_ape_xy_p95_m": percentile(spin_ape_xy, 95.0) if spin_ape_xy else 0.0,
        "lio_wall_p50_ms": percentile(wall_ms, 50.0),
        "lio_wall_p95_ms": percentile(wall_ms, 95.0),
        "lio_wall_max_ms": max(wall_ms) if wall_ms else 0.0,
    }


def load_input(path: pathlib.Path, est_topic: str, gt_topic: str) -> Tuple[List[PoseRow], List[PoseRow]]:
    if path.is_dir():
        db3_files = sorted(path.glob("*.db3"))
        if not db3_files:
            raise FileNotFoundError(f"No .db3 file found in {path}")
        return load_rosbag_sqlite(db3_files[0], est_topic, gt_topic)
    if path.suffix.lower() == ".db3":
        return load_rosbag_sqlite(path, est_topic, gt_topic)
    raise ValueError(f"Unsupported bag input {path}; use --est-csv/--gt-csv for text logs")


def format_markdown(results: Dict[str, Dict[str, float]]) -> str:
    headers = [
        "Run",
        "Rate (Hz)",
        "APE RMSE (m)",
        "APE p95 (m)",
        "APE Max (m)",
        "RPE-1m (m)",
        "Spin p95 (m)",
        "LIO p95 (ms)",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for label, m in results.items():
        lines.append(
            f"| `{label}` | {m['odom_rate_hz']:.1f} | {m['ape_xy_rmse_m']:.4f} | "
            f"{m['ape_xy_p95_m']:.4f} | {m['ape_xy_max_m']:.4f} | "
            f"{m['rpe_1m_xy_rmse_m']:.4f} | {m['spin_ape_xy_p95_m']:.4f} | "
            f"{m['lio_wall_p95_ms']:.2f} |"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-a", type=pathlib.Path, help="Baseline rosbag2 dir or .db3")
    parser.add_argument("--bag-b", type=pathlib.Path, help="Candidate rosbag2 dir or .db3")
    parser.add_argument("--est-a", type=pathlib.Path, help="Baseline estimated pose CSV/JSON")
    parser.add_argument("--gt-a", type=pathlib.Path, help="Baseline ground-truth pose CSV/JSON")
    parser.add_argument("--est-b", type=pathlib.Path, help="Candidate estimated pose CSV/JSON")
    parser.add_argument("--gt-b", type=pathlib.Path, help="Candidate ground-truth pose CSV/JSON")
    parser.add_argument("--log-a", type=pathlib.Path, help="Baseline point_lio console log")
    parser.add_argument("--log-b", type=pathlib.Path, help="Candidate point_lio console log")
    parser.add_argument("--label-a", default="baseline_A")
    parser.add_argument("--label-b", default="candidate_B")
    parser.add_argument("--est-topic", default="/odometry")
    parser.add_argument("--gt-topic", default="/simulation/ground_truth/odometry")
    parser.add_argument("--no-align-origin", action="store_true")
    parser.add_argument("--output-json", type=pathlib.Path)
    args = parser.parse_args()

    results: Dict[str, Dict[str, float]] = {}
    if args.bag_a is not None:
        est_a, gt_a = load_input(args.bag_a, args.est_topic, args.gt_topic)
        results[args.label_a] = evaluate_run(
            est_a, gt_a, parse_lio_log_latencies(args.log_a), not args.no_align_origin
        )
    elif args.est_a is not None and args.gt_a is not None:
        results[args.label_a] = evaluate_run(
            load_pose_json_or_csv(args.est_a),
            load_pose_json_or_csv(args.gt_a),
            parse_lio_log_latencies(args.log_a),
            not args.no_align_origin,
        )

    if args.bag_b is not None:
        est_b, gt_b = load_input(args.bag_b, args.est_topic, args.gt_topic)
        results[args.label_b] = evaluate_run(
            est_b, gt_b, parse_lio_log_latencies(args.log_b), not args.no_align_origin
        )
    elif args.est_b is not None and args.gt_b is not None:
        results[args.label_b] = evaluate_run(
            load_pose_json_or_csv(args.est_b),
            load_pose_json_or_csv(args.gt_b),
            parse_lio_log_latencies(args.log_b),
            not args.no_align_origin,
        )

    md = format_markdown(results)
    if results:
        print(md)
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps({"results": results, "markdown": md}, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
