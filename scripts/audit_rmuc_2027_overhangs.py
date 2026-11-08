#!/usr/bin/env python3
"""Offline official-mesh audit. Output is evaluation data, never navigation input."""

from __future__ import annotations

import argparse
import json
import platform
from dataclasses import asdict
from pathlib import Path

import numpy as np
from scipy.ndimage import label

from generate_rmuc_2027_map import (
    REPO_ROOT, Rules, build_layers, iter_triangles, rasterize, sha256,
)


def lowest_air_layers(keys, rules):
    """Lowest exposed floor with a bounded ceiling; no tunnel exemptions.

    The shared map rasterizer preserves face orientation and unions overlapping
    solids. A closed roof must have an exposed upper face after its underside.
    Open/unbalanced columns are reported, not silently treated as trustworthy.
    """
    if not len(keys):
        return np.empty((0, 4)), {"unbalanced_columns": 0.0, "unclosed_roofs": 0}
    layers = build_layers(keys, rules, [])
    col, z = layers["column"], layers["z"]
    # Reject columns whose signed crossings do not close. Their apparent free
    # space depends on a missing face and cannot drive collision generation.
    heads = np.r_[True, col[1:] != col[:-1]]
    starts = np.flatnonzero(heads)
    balances = np.add.reduceat(np.where(layers["up"], 1, -1), starts)
    balanced = np.repeat(balances == 0, np.diff(np.r_[starts, len(col)]))
    bounded = np.flatnonzero(layers["exposed"] & np.isfinite(layers["clearance"])
                             & balanced & (layers["clearance"] > rules.interface_gap))
    # build_layers is ordered by column, then height.
    bounded = bounded[np.r_[True, np.diff(col[bounded]) != 0]] if len(bounded) else bounded
    records, unclosed = [], 0
    for i in bounded:
        end = np.searchsorted(col, col[i], side="right")
        tops = np.flatnonzero(layers["exposed"][i + 2:end]) + i + 2
        if not len(tops):
            unclosed += 1
            continue
        records.append((col[i], z[i], z[i] + layers["clearance"][i], z[tops[0]]))
    return np.asarray(records, dtype=float).reshape(-1, 4), {
        "unbalanced_columns": layers["unbalanced_columns"], "unclosed_roofs": unclosed,
        "excluded_unbalanced_columns": int(np.count_nonzero(balances)),
    }


def fit_plane(x, y, z):
    design = np.c_[np.ones(len(x)), x, y]
    coef, _, rank, _ = np.linalg.lstsq(design, z, rcond=None)
    residual = z - design @ coef
    return {"a": float(coef[0]), "slope_x": float(coef[1]), "slope_y": float(coef[2]),
            "rank": int(rank), "max_abs_error_m": float(np.max(np.abs(residual))),
            "rms_error_m": float(np.sqrt(np.mean(residual ** 2)))}


def probe_surfaces(chunks, points):
    """Independent exact-coordinate barycentric rays (no grid or z quantization)."""
    hits = [[] for _ in points]
    for triangles in chunks:
        low, high = triangles[:, :, :2].min(axis=1), triangles[:, :, :2].max(axis=1)
        for k, point in enumerate(points):
            xy = np.asarray(point)
            tri = triangles[np.all((low <= xy) & (high >= xy), axis=1)]
            if not len(tri):
                continue
            e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
            det = e1[:, 0] * e2[:, 1] - e2[:, 0] * e1[:, 1]
            keep = np.abs(det) > 1e-14
            tri, e1, e2, det = tri[keep], e1[keep], e2[keep], det[keep]
            delta = xy - tri[:, 0, :2]
            u = (delta[:, 0] * e2[:, 1] - e2[:, 0] * delta[:, 1]) / det
            v = (e1[:, 0] * delta[:, 1] - delta[:, 0] * e1[:, 1]) / det
            inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
            z = tri[:, 0, 2] + u * e1[:, 2] + v * e2[:, 2]
            hits[k].extend(zip(z[inside].tolist(), (det[inside] > 0).tolist()))
    result = []
    for point, surfaces in zip(points, hits):
        merged = []
        for z, up in sorted(surfaces):
            if merged and up == merged[-1][1] and abs(z - merged[-1][0]) < 1e-6:
                continue
            merged.append([z, up])
        result.append({"xy": list(point), "surfaces_z_facing_up": merged})
    return result


def cluster_layers(records, rules):
    """8-connectivity is geometric proximity only, not proof of one planar plate."""
    width = rules.width_cells * rules.supersample
    height = rules.height_cells * rules.supersample
    step = rules.resolution / rules.supersample
    ids = records[:, 0].astype(int)
    mask = np.zeros((height, width), dtype=bool)
    mask.ravel()[ids] = True
    labels, count = label(mask, structure=np.ones((3, 3), dtype=int))
    cell_labels = labels.ravel()[ids]
    clusters = []
    for k in range(1, count + 1):
        picked = np.flatnonzero(cell_labels == k)
        cells = records[picked]
        x, y = (cells[:, 0] % width + 0.5) * step, (cells[:, 0] // width + 0.5) * step
        gap = cells[:, 2] - cells[:, 1]
        clusters.append({"id": k, "cells": len(picked), "area_m2": len(picked) * step ** 2,
                         "bounds_xy": [float(x.min() - step / 2), float(x.max() + step / 2),
                                       float(y.min() - step / 2), float(y.max() + step / 2)],
                         "clearance_min_max_m": [float(gap.min()), float(gap.max())],
                         "bottom_plane": fit_plane(x, y, cells[:, 2]),
                         "top_plane": fit_plane(x, y, cells[:, 3])})
    return clusters, cell_labels


def compare_scene(records, rules, model_path, map_yaml):
    """Compare actual compiled MuJoCo group-3 rays, not XML box bounding boxes."""
    import mujoco
    import yaml
    from PIL import Image

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    groups = np.zeros(mujoco.mjNGROUP, dtype=np.uint8)
    groups[3] = 1
    geom_id = np.zeros(1, dtype=np.int32)
    width = rules.width_cells * rules.supersample
    step = rules.resolution / rules.supersample
    x = (records[:, 0] % width + 0.5) * step
    y = (records[:, 0] // width + 0.5) * step
    hgeom = model.geom("rmuc_2027_field").id
    hid = int(model.geom_dataid[hgeom])
    rows, cols = int(model.hfield_nrow[hid]), int(model.hfield_ncol[hid])
    size = model.hfield_size[hid]
    hdata = model.hfield_data[int(model.hfield_adr[hid]):][:rows * cols].reshape(rows, cols)
    center = data.geom_xpos[hgeom]
    ci = np.rint((x - center[0] + size[0]) / (2 * size[0]) * (cols - 1)).astype(int)
    ri = np.rint((y - center[1] + size[1]) / (2 * size[1]) * (rows - 1)).astype(int)
    terrain = hdata[ri, ci] * size[2] + center[2]
    underside = []
    for xx, yy, floor in zip(x, y, records[:, 1]):
        start = floor + 0.01
        distance = mujoco.mj_ray(model, data, np.array([xx, yy, start]),
                                 np.array([0., 0., 1.]), groups, 1, -1, geom_id)
        underside.append(start + distance if distance >= 0 else np.nan)
    underside = np.asarray(underside)
    metadata = yaml.safe_load(map_yaml.read_text())
    if abs(float(metadata["origin"][2])) > 1e-12:
        raise ValueError("audit PGM comparison currently requires zero map yaw")
    image_path = map_yaml.parent / metadata["image"]
    image = np.asarray(Image.open(image_path).convert("L"))
    pc = np.floor((x - metadata["origin"][0]) / metadata["resolution"]).astype(int)
    pr = image.shape[0] - 1 - np.floor((y - metadata["origin"][1]) / metadata["resolution"]).astype(int)
    valid = (pc >= 0) & (pc < image.shape[1]) & (pr >= 0) & (pr < image.shape[0])
    pixels = np.full(len(records), -1, dtype=int)
    pixels[valid] = image[pr[valid], pc[valid]]
    return terrain, underside, pixels


REGIONS = {
    "D1": (14.78, 15.08, 1.48, 1.60),
    "D2": (14.30, 15.46, 2.46, 2.58),
    "D3": (10.02, 10.58, 4.50, 5.62),
    "D4_a": (24.94, 25.50, 5.12, 5.32),
    "D4_b": (2.50, 3.08, 10.68, 10.90),
    "D4_c": (5.74, 6.54, 4.46, 4.66),
    "D4_d": (21.30, 22.26, 11.28, 11.58),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh", type=Path, default=REPO_ROOT / "2027_map/RMUC2027_V1.0.0.STL")
    parser.add_argument("--mesh-scale", type=float, default=0.001)
    parser.add_argument("--resolution", type=float, default=0.02)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=REPO_ROOT / "src/sim/ats_mujoco_sim/models/rmuc_2027_swerve.xml")
    parser.add_argument("--map-yaml", type=Path, default=REPO_ROOT / "src/ats_sentry_bringup/map/rmuc_2027.yaml")
    args = parser.parse_args()
    if args.resolution <= 0 or args.mesh_scale <= 0:
        parser.error("resolution and mesh-scale must be positive")
    rules = Rules(resolution=args.resolution, supersample=1,
                  width_cells=round(28 / args.resolution), height_cells=round(16 / args.resolution),
                  merge_gap=0.001, interface_gap=0.001)
    keys = rasterize(iter_triangles(args.mesh, args.mesh_scale), rules)
    records, diagnostics = lowest_air_layers(keys, rules)
    clusters, cell_labels = cluster_layers(records, rules)
    terrain, underside, pixels = compare_scene(records, rules, args.model, args.map_yaml)
    width = rules.width_cells
    x = (records[:, 0] % width + 0.5) * args.resolution
    y = (records[:, 0] // width + 0.5) * args.resolution
    def summarize(mask):
        cells = records[mask]
        if not len(cells):
            return {"cells": 0}
        return {"cells": len(cells),
                "floor_min_max_m": [float(cells[:, 1].min()), float(cells[:, 1].max())],
                "clearance_min_max_m": [float((cells[:, 2] - cells[:, 1]).min()),
                                        float((cells[:, 2] - cells[:, 1]).max())],
                "bottom_plane": fit_plane(x[mask], y[mask], cells[:, 2]),
                "hfield_error_over_1cm": int(np.count_nonzero(np.abs(terrain[mask] - cells[:, 1]) > 0.01)),
                "underside_error_over_1cm_or_missing": int(np.count_nonzero(
                    ~np.isfinite(underside[mask]) | (np.abs(underside[mask] - cells[:, 2]) > 0.01))),
                "pgm_white_fraction": float(np.mean(pixels[mask] >= 200))}
    report = {"schema_version": 1, "purpose": "offline_evaluation_only",
              "mesh": str(args.mesh), "mesh_sha256": sha256(args.mesh),
              "mesh_scale": args.mesh_scale, "rules": asdict(rules), "diagnostics": diagnostics,
              "exact_probes": probe_surfaces(iter_triangles(args.mesh, args.mesh_scale),
                                               [(14.93, 1.50), (14.50, 2.52), (10.30, 5.00)]),
              "environment": {"python": platform.python_version(), "numpy": np.__version__},
              "source_hashes": {str(path): sha256(path) for path in [
                  Path(__file__), Path(__file__).with_name("generate_rmuc_2027_map.py"),
                  args.model, args.model.parent / "rmuc_2027_height.png",
                  args.model.parent / "rmuc_2027_roof_boxes.xml",
                  args.model.parent / "meshes/rmuc_2027.obj", args.map_yaml,
                  args.map_yaml.with_suffix(".pgm")]},
              "comparison_semantics": "nearest hfield sample; upward group-3 ray; no edge exemptions",
              "g_box": None, "g_box_note": "Not established: no collision-equivalence claim or geometry generation",
              "summary": summarize(np.ones(len(records), dtype=bool)), "clusters": clusters,
              "regions": {name: summarize((x >= a) & (x <= b) & (y >= c) & (y <= d))
                          for name, (a, b, c, d) in REGIONS.items()},
              "regions_vehicle_height_subset": {
                  name: summarize((x >= a) & (x <= b) & (y >= c) & (y <= d)
                                  & (records[:, 2] - records[:, 1] >= rules.vehicle_height))
                  for name, (a, b, c, d) in REGIONS.items()},
              "cell_fields": ["column", "floor_z", "bottom_z", "top_z", "cluster_id",
                              "hfield_z_nearest", "collision_underside_z", "pgm_pixel"],
              "cells": [[int(r[0]), *r[1:].tolist(), int(k), float(h),
                         float(u) if np.isfinite(u) else None, int(p)]
                        for r, k, h, u, p in zip(records, cell_labels, terrain, underside, pixels)]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "summary": report["summary"],
                      "diagnostics": diagnostics, "regions": report["regions"]}, indent=2))


if __name__ == "__main__":
    main()
