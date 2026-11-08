#!/usr/bin/env python3
"""Offline connected-floor/ceiling candidates from H raw odom scans.

This diagnostic recomputes ground; it is not the production GroundColumnGrid
state and cannot by itself certify traversability or physical clearance.
"""
import argparse
from collections import defaultdict, deque
import json
from pathlib import Path

import numpy as np


def localization_seed(localizations, stamp, base_height, max_delta_s=0.1):
    """Offline nearest-time pose, bounded explicitly; no latest-pose fallback."""
    if not localizations:
        raise ValueError("localization seed requires recorded poses")
    def time_of(message):
        value = message["header"]["stamp"]
        return value["sec"] + value["nanosec"] * 1e-9
    pose = min(localizations, key=lambda message: abs(time_of(message) - stamp))
    delta = time_of(pose) - stamp
    if abs(delta) > max_delta_s:
        raise ValueError("localization seed pose is outside scan time tolerance")
    if pose["header"]["frame_id"] != "odom" or pose["child_frame_id"] != "gimbal_yaw_odom":
        raise ValueError("unexpected localization seed frames")
    position = pose["pose"]["pose"]["position"]
    seed = np.array([position[key] for key in ("x", "y", "z")], dtype=float)
    seed[2] -= base_height
    if not np.isfinite(seed).all() or not np.isfinite(base_height) or base_height < 0:
        raise ValueError("invalid localization ground seed")
    return seed, delta


def candidates(xyz, resolution=0.1, seed=(0.0, 0.0, 0.0), ground_fit_radius=0.0):
    columns = defaultdict(list)
    for x, y, z in np.asarray(xyz):
        if np.isfinite([x, y, z]).all():
            columns[(int(np.floor(x / resolution)), int(np.floor(y / resolution)))].append(z)
    surfaces = {}
    for key, values in columns.items():
        ordered = np.sort(values)
        clusters = np.split(ordered, np.flatnonzero(np.diff(ordered) > 0.06) + 1)
        # Isolated speckles are not a supported floor or a measured ceiling.
        surfaces[key] = [cluster for cluster in clusters if len(cluster) >= 3]
    floor = {}
    queue = deque()
    for key, clusters in surfaces.items():
        if not clusters:
            continue
        x, y = ((v + 0.5) * resolution for v in key)
        z = float(np.median(clusters[0]))
        if np.hypot(x - seed[0], y - seed[1]) <= 0.65 and abs(z - seed[2]) <= 0.05:
            floor[key] = z
            queue.append(key)
    while queue:
        x, y = queue.popleft()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                key = (x + dx, y + dy)
                if key in floor or key not in surfaces or not surfaces[key]:
                    continue
                z = float(np.median(surfaces[key][0]))
                if abs(z - floor[(x, y)]) <= 0.6 * resolution * np.hypot(dx, dy) + 1e-6:
                    floor[key] = z
                    queue.append(key)
    # Optional diagnostic interpolation across the parked vehicle's own
    # occlusion. It requires observed, connected supports on all four sides,
    # a full-rank plane and <=5 mm residual. It never creates a navigation free
    # cell, nor proves that an unobserved patch actually contains solid ground.
    estimates = {key: (z, "observed", 0.0) for key, z in floor.items()}
    if ground_fit_radius > 0 and floor:
        supports = np.array([[(key[0] + 0.5) * resolution,
                              (key[1] + 0.5) * resolution, z] for key, z in floor.items()])
        for key, clusters in surfaces.items():
            if key in estimates or not clusters:
                continue
            center = (np.array(key) + 0.5) * resolution
            local = supports[np.linalg.norm(supports[:, :2] - center, axis=1) <= ground_fit_radius]
            if len(local) < 6 or not (
                np.any(local[:, 0] < center[0]) and np.any(local[:, 0] > center[0]) and
                np.any(local[:, 1] < center[1]) and np.any(local[:, 1] > center[1])):
                continue
            design = np.column_stack([local[:, :2] - center, np.ones(len(local))])
            coefficients, _, rank, _ = np.linalg.lstsq(design, local[:, 2], rcond=None)
            residual = float(np.max(np.abs(design @ coefficients - local[:, 2])))
            if rank == 3 and residual <= 0.005 and np.linalg.norm(coefficients[:2]) <= 0.6:
                estimates[key] = (float(coefficients[2]), "supported_plane_diagnostic", residual)
    result = []
    for key, (z, ground_source, residual) in estimates.items():
        overhead = [cluster for cluster in surfaces[key]
                    if float(np.quantile(cluster, 0.1)) > z + 0.1]
        if overhead:
            cluster = overhead[0]
            result.append({"column": list(key), "ground_z": z,
                           "ground_source": ground_source, "ground_fit_max_residual_m": residual,
                           "ceiling_min_z": float(cluster.min()),
                           "clearance_min_m": float(cluster.min() - z),
                           "clearance_q10_m": float(np.quantile(cluster, 0.1) - z),
                           "floor_points": len(surfaces[key][0]) if key in floor else 0,
                           "ceiling_points": len(cluster)})
    return result


def summarize(values):
    return {"count": len(values), "min": float(np.min(values)),
            "p05": float(np.percentile(values, 5)), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "max": float(np.max(values))} if values else {"count": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--window-frames", type=int, default=1,
                        help="fixed raw-frame aggregation window; >1 creates correlated estimates")
    parser.add_argument("--ground-fit-radius", type=float, default=0.0,
                        help="optional supported-plane diagnostic for self-occluded ground, metres")
    parser.add_argument("--seed", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--localization-base-height", type=float,
                        help="use nearest recorded odom body pose minus this z offset as seed")
    parser.add_argument("--roi", nargs=4, type=float, metavar=("XMIN", "YMIN", "XMAX", "YMAX"))
    args = parser.parse_args()
    if args.resolution <= 0 or args.window_frames <= 0:
        parser.error("resolution and window-frames must be positive")
    records, temporal = [], defaultdict(list)
    seed_deltas = []
    localizations = None
    if args.localization_base_height is not None:
        localizations = json.loads((args.directory / "localization.json").read_text())
    window = deque(maxlen=args.window_frames)
    metadata = json.loads((args.directory / "frames.json").read_text())
    for index, frame in enumerate(metadata):
        if frame["frame"] != "odom":
            raise ValueError(f"unexpected scan frame {frame['frame']}")
        xyz = np.load(args.directory / f"scan_{index:04d}.npz")["xyz"]
        window.append(xyz)
        if len(window) < args.window_frames:
            continue
        seed = args.seed
        if localizations is not None:
            seed, delta = localization_seed(localizations, frame["stamp"], args.localization_base_height)
            seed_deltas.append(abs(delta))
        rows = candidates(np.concatenate(window), args.resolution, seed, args.ground_fit_radius)
        for row in rows:
            x, y = ((v + 0.5) * args.resolution for v in row["column"])
            if args.roi and not (args.roi[0] <= x <= args.roi[2] and args.roi[1] <= y <= args.roi[3]):
                continue
            row.update(frame_index=index, stamp=frame["stamp"], seed=list(seed))
            records.append(row)
            temporal[tuple(row["column"])].append(row["clearance_min_m"])
    repeated = {str(key): {"samples": len(values), "std_m": float(np.std(values)),
                          "peak_to_peak_m": float(np.ptp(values))}
                for key, values in temporal.items() if len(values) >= 5}
    result = {"scope": "offline connected-ground candidates, not production state or acceptance",
              "frames": len(metadata), "window_frames": args.window_frames,
              "ground_fit_radius": args.ground_fit_radius,
               "resolution": args.resolution, "seed": args.seed, "roi": args.roi,
              "seed_source": "scan_time_localization" if localizations is not None else "fixed_argument",
              "localization_base_height": args.localization_base_height,
              "seed_absolute_time_delta_s": summarize(seed_deltas),
              "clearance_min_m": summarize([r["clearance_min_m"] for r in records]),
              "clearance_q10_m": summarize([r["clearance_q10_m"] for r in records]),
              "repeated_columns": repeated,
              "temporal_std_m": summarize([r["std_m"] for r in repeated.values()]),
              "temporal_peak_to_peak_m": summarize([r["peak_to_peak_m"] for r in repeated.values()]),
              "records": records}
    suffix = f"_fit{args.ground_fit_radius:g}" if args.ground_fit_radius else ""
    if localizations is not None:
        suffix += "_locseed"
    (args.directory / f"clearance_analysis_w{args.window_frames}{suffix}.json").write_text(
        json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k not in ("records", "repeated_columns")}, indent=2))


if __name__ == "__main__":
    main()
