#!/usr/bin/env python3
"""Compare saved maps with independent scene box surfaces, including unseen walls."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml


def cell_centres(width, height, resolution, origin):
    x, y = np.meshgrid((np.arange(width) + 0.5) * resolution,
                       (np.arange(height) + 0.5) * resolution)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return np.column_stack((origin[0] + c * x.ravel() - s * y.ravel(),
                            origin[1] + s * x.ravel() + c * y.ravel()))


def compare_masks(prediction, truth):
    prediction, truth = np.asarray(prediction), np.asarray(truth, dtype=bool)
    if prediction.shape != truth.shape:
        raise ValueError("prediction and scene truth have different shapes")
    occupied = prediction >= 50
    missed = truth & ~occupied
    return {"wall_surface_cells": int(truth.sum()),
            "saved_occupied_cells": int(occupied.sum()),
            "true_positive_cells": int((truth & occupied).sum()),
            "missed_wall_cells": int(missed.sum()),
            "missed_unknown_cells": int((missed & (prediction < 0)).sum()),
            "missed_free_cells": int((missed & (prediction == 0)).sum()),
            "false_positive_cells": int((~truth & occupied).sum())}


def box_surface_distances(points, boxes):
    points = np.asarray(points, dtype=float)
    result = np.full(len(points), np.inf)
    for x, y, dx, dy in boxes:
        q = np.abs(points - [x, y]) - [dx, dy]
        signed = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0)
        result = np.minimum(result, np.abs(signed))
    return result


def read_ascii_pcd(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.startswith("DATA "):
                if line.strip() != "DATA ascii":
                    raise ValueError("service PCD must use its declared ASCII encoding")
                return np.loadtxt(stream, ndmin=2)[:, :3]
    raise ValueError("PCD has no DATA header")


def cloud_geometry(points):
    points = np.asarray(points, dtype=float)
    if len(points) < 6 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("PCD has insufficient finite geometric support")
    eigenvalues = np.linalg.eigvalsh(np.cov(points.T))
    if eigenvalues[0] < 1e-10:
        raise ValueError("PCD has insufficient 3D geometric support")
    return {"points": len(points), "minimum": points.min(axis=0).tolist(),
            "maximum": points.max(axis=0).tolist(),
            "covariance_eigenvalues": eigenvalues.tolist()}


def load_map(prefix):
    metadata = yaml.safe_load(Path(str(prefix) + ".yaml").read_text())
    path = Path(str(prefix) + ".yaml").parent / metadata["image"]
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError("expected service binary PGM")
        width, height = map(int, stream.readline().split())
        if stream.readline().strip() != b"255":
            raise ValueError("expected 8-bit map")
        pixels = np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width)[::-1]
    probability = 1.0 - pixels.astype(float) / 255.0
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[probability < metadata["free_thresh"]] = 0
    cells[probability > metadata["occupied_thresh"]] = 100
    centres = cell_centres(width, height, metadata["resolution"], metadata["origin"])
    return metadata, cells.ravel(), centres


def evaluate(prefix, boxes=None, model_path=None):
    metadata, cells, centres = load_map(prefix)
    resolution = metadata["resolution"]
    scene = None
    if model_path is not None:
        from scipy.spatial import cKDTree
        from s3_scene_geometry import scene_wall_samples
        points, scene = scene_wall_samples(model_path, resolution / 5)
        distances = cKDTree(points).query(centres)[0]
    else:
        distances = box_surface_distances(centres, boxes)
    truth = distances < resolution * math.sqrt(2) / 2 - 1e-9
    result = compare_masks(cells, truth)
    saved_distances = distances[cells >= 50]
    result["position_error_m"] = {
        "mean": float(np.mean(saved_distances)) if len(saved_distances) else None,
        "p95": float(np.percentile(saved_distances, 95)) if len(saved_distances) else None,
        "maximum": float(np.max(saved_distances)) if len(saved_distances) else None}
    result["saved_cells_farther_than_one_resolution"] = int((saved_distances > resolution).sum())
    result["definition"] = ("all projected static steep scene-mesh/box faces, including unseen and overhead faces; "
                            "geometric wall comparison, not a traversability ground truth; "
                            "longest projected triangle segments sampled at resolution/5"
                            if scene else "all scene box surfaces; unobserved surfaces remain missed; raw counts unchanged")
    if scene:
        result["scene_geometry"] = scene
    result["resolution_m"] = resolution
    result["cloud_geometry"] = cloud_geometry(read_ascii_pcd(Path(str(prefix) + ".pcd")))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--truth-boxes")
    source.add_argument("--scene-model")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = evaluate(args.prefix, json.loads(Path(args.truth_boxes).read_text()) if args.truth_boxes else None,
                      args.scene_model)
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
