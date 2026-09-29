#!/usr/bin/env python3
"""Generate the RMUC prior PCD by area-uniform sampling of the field STL.

The previous prior was sampled too sparsely on large flat triangles, so most of
the floor around the spawn area had no prior points and GICP matched only ~30%
of a MuJoCo/Gazebo scan. Sampling every triangle proportionally to its area
covers floors, ramps and walls uniformly (scan inlier ratio ~99% at 0.05 m).

Example:
  python3 scripts/generate_rmuc_prior_pcd.py \\
    --stl src/sim/ats_mujoco_sim/models/meshes/rmuc_2025.stl \\
    --offset 10.92 -1.44 0.20 \\
    --output src/ats_sentry_bringup/pcd/rmuc_2025.pcd
"""

from __future__ import annotations

import argparse
import struct
import sys

import numpy as np

from dump_gazebo_prior_pcd import write_pcd


def load_binary_stl(path: str) -> np.ndarray:
    """Return triangles as an (N, 3, 3) float64 array."""
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < 84:
        raise ValueError(f"{path}: too short for a binary STL")
    count = struct.unpack("<I", raw[80:84])[0]
    if len(raw) < 84 + count * 50:
        raise ValueError(f"{path}: truncated or ASCII STL (expected binary)")
    dtype = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw[84 : 84 + count * 50], dtype=dtype)["v"].astype(np.float64)


def sample_triangles(
    tri: np.ndarray, spacing: float, rng: np.random.Generator
) -> np.ndarray:
    """Area-uniform samples, at least one point per triangle."""
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    counts = rng.poisson(area / (spacing * spacing)) + 1
    idx = np.repeat(np.arange(len(tri)), counts)
    uv = rng.random((len(idx), 2))
    flip = uv.sum(axis=1) > 1.0
    uv[flip] = 1.0 - uv[flip]
    return tri[idx, 0] + uv[:, :1] * e1[idx] + uv[:, 1:] * e2[idx]


def voxel_centers(xyz: np.ndarray, leaf: float) -> np.ndarray:
    keys = np.unique(np.floor(xyz / leaf).astype(np.int64), axis=0)
    return (keys + 0.5) * leaf


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stl", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--offset",
        type=float,
        nargs=3,
        default=(10.92, -1.44, 0.20),
        help="p_map = p_stl + offset (Gazebo field pose)",
    )
    parser.add_argument("--spacing", type=float, default=0.02)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=-0.2)
    parser.add_argument("--z-max", type=float, default=3.0)
    parser.add_argument("--seed", type=int, default=2025)
    args = parser.parse_args()

    if args.spacing <= 0.0 or args.voxel <= 0.0:
        print("ERROR: --spacing and --voxel must be positive", file=sys.stderr)
        return 2

    tri = load_binary_stl(args.stl) + np.asarray(args.offset, dtype=np.float64)
    pts = sample_triangles(tri, args.spacing, np.random.default_rng(args.seed))
    pts = pts[(pts[:, 2] >= args.z_min) & (pts[:, 2] <= args.z_max)]
    xyz = voxel_centers(pts, args.voxel)
    if xyz.size == 0:
        print("ERROR: no points inside the height band", file=sys.stderr)
        return 2
    write_pcd(args.output, xyz)
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    print(
        f"OK triangles={len(tri)} points={len(xyz)} output={args.output} "
        f"bounds=({lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f})..({hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
