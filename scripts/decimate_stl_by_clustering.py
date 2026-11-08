#!/usr/bin/env python3
"""Reduce a dense binary STL to a MuJoCo-sized OBJ by vertex clustering.

MuJoCo rejects STL files with more than 200000 faces and the official RMUC field
model has 7.5 million, so the field is reduced here instead of by a quadric
decimator (none is installed on the workstation).  Every vertex is snapped to a
cubic cell of ``--cell`` metres, vertices that share a cell are merged into
their mean position, and triangles that collapse or become duplicates are
dropped.  Nothing larger than one cell disappears, and no surface moves by more
than about one cell, which is the guarantee that a lossy decimator does not give.
Face orientation is kept.

MuJoCo's ray cost grows roughly with the face count (measured on the RMUC 2027 field:
1.1-1.5 s per 12000-ray LiDAR frame at 450k faces, 0.35-0.46 s at 122k), and the
simulated LiDAR is the bottleneck of the ``lio`` mode.  So the field is reduced with a
coarse ``--cell`` and keeps a fine ``--fine-cell`` only inside ``--fine-zone`` boxes
(or around the tunnels of a ``--tunnels-yaml``), where centimetres decide whether a
0.571 m robot fits a 0.795 m passage.  Triangles are assigned to a zone by centroid;
the two parts share no vertices, so a hairline seam of about one coarse cell can
exist along a zone boundary.

Example (official RMUC 2027 V1.0.0 model, millimetres -> metres):
  python3 scripts/decimate_stl_by_clustering.py 2027_map/RMUC2027_V1.0.0.STL \\
    src/sim/ats_mujoco_sim/models/meshes/rmuc_2027.obj --scale 0.001 --cell 0.03 \\
    --fine-cell 0.01 --tunnels-yaml src/sim/ats_mujoco_sim/config/tunnels_2027.yaml --zone-margin 1.2
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np
import yaml

_TRIANGLE = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
_CHUNK = 1_000_000
_AXIS_BITS = 21


def read_binary_stl_vertices(path: Path, scale: float) -> np.ndarray:
    """Return the (N, 3, 3) float32 triangle vertices of a binary STL, scaled."""
    with path.open("rb") as stl:
        header = stl.read(84)
    if len(header) < 84:
        raise ValueError(f"{path}: too short for a binary STL")
    count = struct.unpack_from("<I", header, 80)[0]
    if 84 + count * _TRIANGLE.itemsize != path.stat().st_size:
        raise ValueError(f"{path}: not a binary STL (size does not match {count} triangles)")
    triangles = np.memmap(path, dtype=_TRIANGLE, mode="r", offset=84, shape=(count,))
    vertices = np.empty((count, 3, 3), dtype=np.float32)
    for start in range(0, count, _CHUNK):
        vertices[start:start + _CHUNK] = triangles["v"][start:start + _CHUNK]
    vertices *= np.float32(scale)
    return vertices


def cluster(vertices: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Merge vertices per ``cell``-sized cube; returns (points, oriented faces)."""
    flat = vertices.reshape(-1, 3)
    grid = np.floor(flat / np.float32(cell)).astype(np.int64)
    grid -= grid.min(axis=0)
    if int(grid.max()) >= 1 << _AXIS_BITS:
        raise ValueError("mesh is too large for this cell size")
    key = (grid[:, 0] << (2 * _AXIS_BITS)) | (grid[:, 1] << _AXIS_BITS) | grid[:, 2]
    del grid
    unique_keys, inverse = np.unique(key, return_inverse=True)
    del key
    members = np.bincount(inverse, minlength=len(unique_keys)).astype(np.float64)
    points = np.empty((len(unique_keys), 3), dtype=np.float64)
    for axis in range(3):
        points[:, axis] = np.bincount(
            inverse, weights=flat[:, axis].astype(np.float64), minlength=len(unique_keys)
        ) / members
    faces = inverse.reshape(-1, 3)
    distinct = (
        (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    )
    faces = faces[distinct]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return points, faces[np.sort(first)]


def in_any_zone(vertices: np.ndarray, zones: list[tuple[float, float, float, float]]) -> np.ndarray:
    """True for triangles whose centroid lies in one of the (x0, x1, y0, y1) boxes."""
    centroid = vertices.mean(axis=1)
    inside = np.zeros(len(vertices), dtype=bool)
    for x0, x1, y0, y1 in zones:
        inside |= (
            (centroid[:, 0] >= x0) & (centroid[:, 0] <= x1)
            & (centroid[:, 1] >= y0) & (centroid[:, 1] <= y1)
        )
    return inside


def zones_from_tunnels(path: Path, margin: float) -> list[tuple[float, float, float, float]]:
    """Boxes around every tunnel's roof polygon and corridor polygon, grown by ``margin``."""
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    zones = []
    for tunnel in config.get("tunnels", []):
        polygons = [tunnel["polygon"]] + ([tunnel["corridor_polygon"]] if "corridor_polygon" in tunnel else [])
        xs = [float(p[0]) for polygon in polygons for p in polygon]
        ys = [float(p[1]) for polygon in polygons for p in polygon]
        zones.append((min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin))
    return zones


def zones_from_audit(path: Path, margin: float) -> list[tuple[float, float, float, float]]:
    """All bounded-air clusters, not just hand-labelled tunnels.

    Keeping even low-clearance clusters fine preserves low lips such as D1.
    These bounds affect simulation mesh reduction only, never navigation.
    """
    if margin < .3:
        raise ValueError('audited fine zones need at least 0.3 m margin')
    report = json.loads(path.read_text())
    if report.get('purpose') != 'offline_evaluation_only':
        raise ValueError('expected official-mesh offline audit')
    return [(a - margin, b + margin, c - margin, d + margin)
            for a, b, c, d in (cluster['bounds_xy'] for cluster in report['clusters'])]


def write_obj(path: Path, points: np.ndarray, faces: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as obj:
        obj.write(f"# {len(points)} vertices, {len(faces)} faces, metres\n")
        np.savetxt(obj, points, fmt="v %.5f %.5f %.5f")
        np.savetxt(obj, faces + 1, fmt="f %d %d %d")


def simplify_planar(points, faces, error):
    """Optional topology-preserving reduction of coplanar dense triangles.

    VTK's absolute error is a generator constraint, not independent proof of
    mesh accuracy. Official-layer ray regression is still required afterwards.
    """
    if error <= 0:
        raise ValueError('simplification error must be positive')
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

    vertices = vtk.vtkPoints()
    vertices.SetData(numpy_to_vtk(np.asarray(points, dtype=np.float64), deep=True))
    triangles = vtk.vtkCellArray()
    packed = np.c_[np.full(len(faces), 3, dtype=np.int64), faces.astype(np.int64)]
    triangles.SetCells(len(faces), numpy_to_vtkIdTypeArray(packed.ravel(), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(vertices)
    poly.SetPolys(triangles)
    decimator = vtk.vtkDecimatePro()
    decimator.SetInputData(poly)
    decimator.SetTargetReduction(.95)
    decimator.PreserveTopologyOn()
    decimator.SplittingOff()
    decimator.BoundaryVertexDeletionOff()
    decimator.SetFeatureAngle(30.)
    decimator.SetErrorIsAbsolute(1)
    decimator.SetAbsoluteError(error)
    decimator.Update()
    result = decimator.GetOutput()
    return (vtk_to_numpy(result.GetPoints().GetData()).copy(),
            vtk_to_numpy(result.GetPolys().GetData()).reshape(-1, 4)[:, 1:].copy())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stl", type=Path)
    parser.add_argument("obj", type=Path)
    parser.add_argument("--scale", type=float, default=1.0, help="STL units to metres")
    parser.add_argument("--cell", type=float, default=0.01, help="cluster cell size in metres")
    parser.add_argument("--fine-cell", type=float, default=None, help="cell size inside the fine zones")
    parser.add_argument(
        "--fine-zone", type=float, nargs=4, action="append", default=[],
        metavar=("X0", "X1", "Y0", "Y1"), help="box (output frame, metres) kept at --fine-cell",
    )
    parser.add_argument("--tunnels-yaml", type=Path, default=None, help="add a fine zone around each tunnel")
    parser.add_argument('--overhang-audit', type=Path, help='add all audited bounded-air clusters')
    parser.add_argument('--audit-zone-margin', type=float, default=.3)
    parser.add_argument('--simplify-error', type=float, help='optional VTK topology-preserving absolute error (m)')
    parser.add_argument("--zone-margin", type=float, default=1.2, help="margin around --tunnels-yaml polygons")
    args = parser.parse_args()
    vertices = read_binary_stl_vertices(args.stl, args.scale)
    zones = [tuple(zone) for zone in args.fine_zone]
    if args.tunnels_yaml is not None:
        zones += zones_from_tunnels(args.tunnels_yaml, args.zone_margin)
    if args.overhang_audit is not None:
        zones += zones_from_audit(args.overhang_audit, args.audit_zone_margin)
    if zones and args.fine_cell is None:
        raise SystemExit("--fine-zone / --tunnels-yaml need --fine-cell")
    if zones:
        fine = in_any_zone(vertices, zones)
        coarse_points, coarse_faces = cluster(vertices[~fine], args.cell)
        fine_points, fine_faces = cluster(vertices[fine], args.fine_cell)
        points = np.vstack([coarse_points, fine_points])
        faces = np.vstack([coarse_faces, fine_faces + len(coarse_points)])
        print(f"fine zones: {len(zones)}, fine triangles in: {int(fine.sum())}, "
              f"faces coarse/fine: {len(coarse_faces)}/{len(fine_faces)}")
    else:
        points, faces = cluster(vertices, args.cell)
    if args.simplify_error is not None:
        before = len(faces)
        points, faces = simplify_planar(points, faces, args.simplify_error)
        print(f'topology-preserving reduction: {before} -> {len(faces)} faces; absolute error {args.simplify_error} m')
    args.obj.parent.mkdir(parents=True, exist_ok=True)
    write_obj(args.obj, points, faces)
    print(
        f"{args.stl}: {len(vertices)} triangles -> {args.obj}: {len(points)} vertices, "
        f"{len(faces)} faces (cell {args.cell} m" + (f", fine {args.fine_cell} m" if zones else "") + "); bounds "
        f"{np.round(points.min(axis=0), 3).tolist()}..{np.round(points.max(axis=0), 3).tolist()}"
    )


if __name__ == "__main__":
    main()
