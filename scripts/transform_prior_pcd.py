#!/usr/bin/env python3
"""Apply a rigid map alignment to a raw SLAM PCD and write a GICP prior.

The raw RMUC 2025 SLAM map is expressed in the tilted MID360 frame at the
mapping start pose. Aligning it to the map frame (same frame as rmuc_2025.yaml
and the Gazebo/MuJoCo field) needs one fixed 4x4 transform; the default below
was solved by floor levelling + yaw search + point-to-point ICP against the
upward-facing surfaces of the field STL (the slab underside at z~0 is excluded
so the floor locks to the 0.20 m field top; inliers 97.1 % @ 0.10 m, median
0.033 m):

  roll 30.201 deg, pitch -0.063 deg, yaw 89.870 deg, t = (0.0200, 0.0875, 0.4780)

Example:
  python3 scripts/transform_prior_pcd.py \\
    --input "src/ats_sentry_bringup/pcd/rmuc_2025 (1).pcd" \\
    --output src/ats_sentry_bringup/pcd/rmuc_2025.pcd
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from dump_gazebo_prior_pcd import write_pcd

RMUC_2025_RAW_TO_MAP = (
    0.002274, -0.864263, 0.503036, 0.019987,
    0.999997, 0.001410, -0.002099, 0.087460,
    0.001105, 0.503039, 0.864263, 0.477995,
)


def read_binary_pcd_xyz(path: str) -> np.ndarray:
    """Read x/y/z from a binary PCD whose fields are all 4-byte floats."""
    with open(path, "rb") as f:
        raw = f.read()
    marker = b"DATA binary\n"
    end = raw.find(marker)
    if end < 0:
        raise ValueError(f"{path}: only 'DATA binary' PCD is supported")
    header = {}
    for line in raw[:end].decode("ascii", "replace").splitlines():
        parts = line.split()
        if parts and not parts[0].startswith("#"):
            header[parts[0]] = parts[1:]
    fields = header.get("FIELDS", [])
    if header.get("SIZE") != ["4"] * len(fields) or header.get("TYPE") != ["F"] * len(fields):
        raise ValueError(f"{path}: expected float32 fields, got {header.get('TYPE')}")
    if any(c != "1" for c in header.get("COUNT", ["1"] * len(fields))):
        raise ValueError(f"{path}: COUNT > 1 fields are not supported")
    count = int(header["POINTS"][0])
    data = np.frombuffer(raw[end + len(marker):], dtype=np.float32, count=count * len(fields))
    data = data.reshape(count, len(fields))
    xyz = data[:, [fields.index("x"), fields.index("y"), fields.index("z")]].astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--transform",
        type=float,
        nargs=12,
        default=RMUC_2025_RAW_TO_MAP,
        help="row-major 3x4 [R|t] mapping raw points into the map frame",
    )
    parser.add_argument("--voxel", type=float, default=0.05)
    # Matches the GICP source height filter; stands/ceiling above never match.
    parser.add_argument("--z-min", type=float, default=-0.5)
    parser.add_argument("--z-max", type=float, default=3.0)
    args = parser.parse_args()

    T = np.asarray(args.transform, dtype=np.float64).reshape(3, 4)
    u, _, vt = np.linalg.svd(T[:, :3])
    rotation = u @ vt  # re-orthonormalize the printed 6-digit matrix
    xyz = read_binary_pcd_xyz(args.input) @ rotation.T + T[:, 3]
    xyz = xyz[(xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max)]
    if args.voxel > 0.0:
        keys, index = np.unique(np.floor(xyz / args.voxel).astype(np.int64), axis=0, return_index=True)
        xyz = xyz[np.sort(index)]
    if xyz.size == 0:
        print("ERROR: no points left after transform/crop", file=sys.stderr)
        return 2
    write_pcd(args.output, xyz)
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    print(
        f"OK points={len(xyz)} output={args.output} "
        f"bounds=({lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f})..({hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
