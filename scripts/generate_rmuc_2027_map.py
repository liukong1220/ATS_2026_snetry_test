#!/usr/bin/env python3
"""Generate the RMUC 2027 static navigation map (PGM + YAML) from the official field mesh.

The mesh is cut by vertical rays on a 4 x 4 sub-grid per map cell (the official mesh is a union
of closed solids, so faces buried inside other solids are discarded by a winding-number test).  Every column
yields its surfaces (up- or down-facing) from bottom to top.  The map is then
built with these rules (report 9.1.2, confirmed 2026-10-25):

* Floor: an up-facing surface with at least `vehicle_height` (0.242 m) of free space
  above it and a plane-fit tilt of at most `slope_max_deg`.  Roofs are only an obstacle
  when they leave less than the vehicle height; inside a tunnel `corridor_polygon`
  (tunnels_2027.yaml) ceilings are ignored, because the tunnel lintel leaves only
  0.250 m for a 0.242 m vehicle and is passable by design.
* Reachable floor: floor cells grow from the seed points through 4-neighbours whose
  height differs by at most `step_max` (0.04 m).  Everything else is not drivable.
* Wall: any solid inside the vehicle height band [floor + step_max, floor + vehicle_height]
  of the local floor, or a column that has no surface at the local floor height
  (the top of a wall or platform that cannot be climbed).
* A map cell is occupied when at least `occupied_coverage` (50 %) of its sub-cells are
  not free, which keeps the 0.795 m tunnel 16 cells (0.80 m) wide.
* Cliff edge: when two 4-neighbour free cells stand on floors that differ by more than
  `step_max`, the higher cell is occupied.  The wall rule only looks inside a cell's own
  column, so a step whose face lies on a cell boundary used to leave both sides free; the
  0.19 m slab edge at y 2.5, x 7.7-10 (south of the red-box start) was drawn free and the
  planner routed across it (report 5.1.6).
* Keep-out: the two undulating-road sections (`Rules.keep_out`) are drawn occupied.  The waves are
  free floor in the mesh but too tall for the wheels; delete the entry to let the planner try them.

Only numpy and PyYAML are required, so the output is bit-identical on every Python that
runs the repository (`python3` and `python3 -I`).

    python3 scripts/generate_rmuc_2027_map.py                  # write pgm, yaml, manifest
    python3 scripts/generate_rmuc_2027_map.py --check          # fail if the committed map drifts
    python3 scripts/generate_rmuc_2027_map.py --report         # numbers: PCD IoU, connectivity, tunnels
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MESH = REPO_ROOT / "src/sim/ats_mujoco_sim/models/meshes/rmuc_2027.obj"
DEFAULT_TUNNELS = REPO_ROOT / "src/sim/ats_mujoco_sim/config/tunnels_2027.yaml"
DEFAULT_PCD = REPO_ROOT / "src/ats_sentry_bringup/pcd/rmuc_2027.pcd"
DEFAULT_MAP_DIR = REPO_ROOT / "src/ats_sentry_bringup/map"
GENERATOR_VERSION = 1

FREE_PIXEL = 254
OCCUPIED_PIXEL = 0
Z_PER_METRE = 2000         # vertical quantum 0.5 mm (coarser steps show up as fake tilt)
Z_BITS = 13
Z_LEVELS = 1 << Z_BITS     # supports heights up to 4.09 m (the field top is 3.80 m)
YAML_TEXT = (
    "image: rmuc_2027.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
)



@dataclass(frozen=True)
class Rules:
    resolution: float = 0.05
    width_cells: int = 560
    height_cells: int = 320
    supersample: int = 4            # sub-cells per cell edge
    vehicle_height: float = 0.242   # RMUC vehicle, report 2.1.8 / user 2026-10-25
    step_max: float = 0.04          # about one wheel radius (0.0425 m)
    slope_max_deg: float = 15.0     # highland ramp ~12 deg, tunnel ramp ~9 deg
    slope_window_cells: int = 3     # +-0.15 m plane fit, shorter than the 0.54 m wheel spacing
    smooth_window_cells: int = 1    # 0.15 m mean before the step test
    same_surface_tolerance: float = 0.05   # above the 4.35 cm hop of the waves, below a 6 cm step
    occupied_coverage: float = 0.5
    layer_gap: float = 0.03         # surfaces closer than this are one floor layer in a cell
    merge_gap: float = 0.008        # coincident triangle edges inside one column
    interface_gap: float = 0.012    # up face directly under a down face = two solids touching
    support_tolerance: float = 0.03
    min_node_support: int = 4       # sub-cells a floor layer needs inside a cell
    max_layers: int = 8
    border_cells: int = 1           # the official mesh has no perimeter fence; keep planners inside
    seeds: tuple = ((5.0, 8.0), (23.0, 8.0))
    # Undulating-road sections (terrain reference: 70 mm bumps, 240 mm pitch).  Measured on the mesh the waves are
    # 6.8 cm crest to trough with flanks up to 41 deg, which the 4.25 cm wheels cannot climb, so the vehicle
    # sticks.  The mesh rasterises them as free floor, hence they are drawn here as obstacles.  Boxes are
    # (x0, x1, y0, y1) in metres; the second is the first one reflected through the field centre (14, 8).
    keep_out: tuple = ((5.25, 7.75, 0.50, 2.70), (20.25, 22.75, 13.30, 15.50))


def _read_obj(path):
    vertices, faces = [], []
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if line.startswith("v "):
                vertices.append([float(v) for v in line.split()[1:4]])
            elif line.startswith("f "):
                corners = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                for k in range(1, len(corners) - 1):        # a quad or n-gon is a fan of triangles
                    faces.append([corners[0], corners[k], corners[k + 1]])
    if not vertices or not faces:
        raise SystemExit(f"no triangles parsed from {path}")
    yield np.asarray(vertices, dtype=np.float64)[np.asarray(faces, dtype=np.int64)]


def _read_stl(path, scale, chunk=400_000):
    """Binary STL, streamed so the 376 MB official file never sits in memory twice."""
    with path.open("rb") as handle:
        header = handle.read(84)
    count = struct.unpack_from("<I", header, 80)[0]
    if 84 + 50 * count != path.stat().st_size:
        raise SystemExit(f"{path} is not a binary STL (ASCII STL is not supported)")
    dtype = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
    data = np.memmap(path, dtype=dtype, mode="r", offset=84, shape=(count,))
    for start in range(0, count, chunk):
        yield np.asarray(data["v"][start:start + chunk], dtype=np.float64) * scale


def iter_triangles(path, scale=1.0):
    """Yield (n, 3, 3) metre-valued triangle chunks from an .obj (metres) or .stl file."""
    suffix = path.suffix.lower()
    if suffix == ".obj":
        return (chunk * scale for chunk in _read_obj(path))
    if suffix == ".stl":
        return _read_stl(path, scale)
    raise SystemExit(f"unsupported mesh format {suffix}; use .obj or binary .stl")


def _window_hits(px, py, pz, d, i0, j0, size, width, height):
    """Vertical-ray hits of `len(px)` triangles over a size x size window of sub-cells."""
    offsets = np.arange(size)
    ii = i0[:, None, None] + offsets[None, :, None]
    jj = j0[:, None, None] + offsets[None, None, :]
    dx, dy = ii - px[:, 0, None, None], jj - py[:, 0, None, None]
    e1x, e1y = (px[:, 1] - px[:, 0])[:, None, None], (py[:, 1] - py[:, 0])[:, None, None]
    e2x, e2y = (px[:, 2] - px[:, 0])[:, None, None], (py[:, 2] - py[:, 0])[:, None, None]
    det = d[:, None, None]
    w1 = (dx * e2y - e2x * dy) / det
    w2 = (e1x * dy - dx * e1y) / det
    w0 = 1.0 - w1 - w2
    eps = 1e-9
    inside = (w0 >= -eps) & (w1 >= -eps) & (w2 >= -eps)
    inside &= (ii >= 0) & (ii < width) & (jj >= 0) & (jj < height)
    z = w0 * pz[:, 0, None, None] + w1 * pz[:, 1, None, None] + w2 * pz[:, 2, None, None]
    zq = np.clip(np.rint(z * Z_PER_METRE), 0, Z_LEVELS - 1).astype(np.int64)
    up = np.broadcast_to((det > 0.0).astype(np.int64), inside.shape)
    return (((jj * width + ii) * Z_LEVELS + zq) * 2 + up)[inside]


def _rasterize_chunk(tri, sub, width, height):
    low, high = float(tri[:, :, 2].min()), float(tri[:, :, 2].max())
    if low < -0.001 or high > (Z_LEVELS - 1) / Z_PER_METRE:
        raise SystemExit(f"mesh heights {low:.3f}..{high:.3f} m are outside the supported 0..{(Z_LEVELS - 1) / Z_PER_METRE:.2f} m "
                         "(wrong --mesh-scale?)")
    px, py, pz = tri[:, :, 0] / sub - 0.5, tri[:, :, 1] / sub - 0.5, tri[:, :, 2]
    d = (px[:, 1] - px[:, 0]) * (py[:, 2] - py[:, 0]) - (px[:, 2] - px[:, 0]) * (py[:, 1] - py[:, 0])
    i0 = np.maximum(np.ceil(px.min(axis=1)), 0).astype(np.int64)
    i1 = np.minimum(np.floor(px.max(axis=1)), width - 1).astype(np.int64)
    j0 = np.maximum(np.ceil(py.min(axis=1)), 0).astype(np.int64)
    j1 = np.minimum(np.floor(py.max(axis=1)), height - 1).astype(np.int64)
    span = np.maximum(i1 - i0, j1 - j0) + 1
    ok = (np.abs(d) > 1e-12) & (i1 >= i0) & (j1 >= j0)
    out, lower = [], 0
    for size in (1, 2, 4, 8, 16, 32, 64):
        picked = np.flatnonzero(ok & (span > lower) & (span <= size))
        lower = size
        step = max(1, 2_000_000 // (size * size))
        for start in range(0, len(picked), step):
            t = picked[start:start + step]
            out.append(_window_hits(px[t], py[t], pz[t], d[t], i0[t], j0[t], size, width, height))
    for t in np.flatnonzero(ok & (span > 64)):       # rare: very large triangles, in 128-wide tiles
        for a in range(0, int(span[t]), 128):
            for b in range(0, int(span[t]), 128):
                s = slice(t, t + 1)
                out.append(_window_hits(px[s], py[s], pz[s], d[s], i0[s] + a, j0[s] + b, 128, width, height))
    return np.concatenate(out) if out else np.empty(0, dtype=np.int64)


def rasterize(chunks, rules):
    """Sorted unique hit keys ((sub-cell * Z_LEVELS + z_q) * 2 + up) of the whole mesh."""
    sub = rules.resolution / rules.supersample
    width, height = rules.width_cells * rules.supersample, rules.height_cells * rules.supersample
    parts, held = [], 0
    for tri in chunks:
        parts.append(np.unique(_rasterize_chunk(tri, sub, width, height)))
        held += len(parts[-1])
        if held > 6_000_000:
            parts = [np.unique(np.concatenate(parts))]
            held = len(parts[0])
    return np.unique(np.concatenate(parts))


def load_tunnels(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))["tunnels"]


def corridor_boxes(tunnels):
    """Axis-aligned (x0, x1, y0, y1) of every tunnel `corridor_polygon`."""
    boxes = []
    for tunnel in tunnels:
        xs, ys = zip(*tunnel["corridor_polygon"])
        boxes.append((min(xs), max(xs), min(ys), max(ys)))
    return boxes


def build_layers(keys, rules, boxes):
    """Merge the hits of every sub-cell column into surfaces with the free height above them."""
    sub = rules.resolution / rules.supersample
    width = rules.width_cells * rules.supersample
    up = (keys & 1).astype(bool)
    zq = (keys >> 1) & (Z_LEVELS - 1)
    idx = keys >> (Z_BITS + 1)
    merge_bins = int(round(rules.merge_gap * Z_PER_METRE))
    new = np.ones(len(idx), dtype=bool)
    new[1:] = ~((idx[1:] == idx[:-1]) & (up[1:] == up[:-1]) & (zq[1:] - zq[:-1] <= merge_bins))
    cluster = np.cumsum(new) - 1
    total = int(cluster[-1]) + 1
    count = np.bincount(cluster, minlength=total)
    z = np.bincount(cluster, weights=zq, minlength=total) / count / Z_PER_METRE
    column, facing_up = idx[np.flatnonzero(new)], up[np.flatnonzero(new)]
    # Two solids that touch leave an up-facing face right under a down-facing one; that
    # interface is not a surface, so both faces go (the winding sums are unchanged).
    touching = (facing_up[:-1] & ~facing_up[1:] & (column[:-1] == column[1:])
                & (z[1:] - z[:-1] <= rules.interface_gap))
    drop = np.zeros(total, dtype=bool)
    drop[:-1] |= touching
    drop[1:] |= touching
    column, facing_up, z = column[~drop], facing_up[~drop], z[~drop]
    total = len(z)
    cx, cy = (column % width + 0.5) * sub, (column // width + 0.5) * sub
    corridor = np.zeros(total, dtype=bool)
    for x0, x1, y0, y1 in boxes:
        corridor |= (cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)
    exempt = corridor & ~facing_up      # a ceiling inside a tunnel corridor is not an obstacle
    # Winding number of the space just above each surface: 0 is open air, >0 is inside a solid.
    # A union of overlapping solids leaves up-facing faces buried inside; those are no floor.
    head = np.r_[True, column[1:] != column[:-1]]
    column_id, head_at = np.cumsum(head) - 1, np.flatnonzero(head)
    sign = np.where(facing_up, 1, -1)
    running = np.cumsum(sign)
    inclusive = running - np.r_[0, running][head_at][column_id]
    column_sum = np.add.reduceat(sign, head_at)
    winding_above = column_sum[column_id] - inclusive
    # Free height above a surface = distance to the next surface above that is not exempt.
    candidate = np.where(exempt, total, np.arange(total))
    later = np.r_[np.minimum.accumulate(candidate[::-1])[::-1][1:], total]
    safe = np.minimum(later, total - 1)
    above = np.where((later < total) & (column[safe] == column), z[safe], np.inf)
    return {"column": column, "up": facing_up, "z": z, "clearance": above - z,
            "exposed": facing_up & (winding_above == 0), "exempt": exempt,
            "unbalanced_columns": float(np.mean(column_sum != 0))}   # cracks in the "closed" mesh


def build_nodes(layers, rules):
    """Floor layers per map cell: height, support, clearance above and plane-fit tilt."""
    S, sub = rules.supersample, rules.resolution / rules.supersample
    width = rules.width_cells * S
    pick = layers["exposed"]
    column, z, clearance = layers["column"][pick], layers["z"][pick], layers["clearance"][pick]
    col, row = column % width, column // width
    cell = (row // S) * rules.width_cells + (col // S)
    order = np.lexsort((z, cell))
    cell, z, clearance, col, row = (a[order] for a in (cell, z, clearance, col, row))
    new = np.ones(len(cell), dtype=bool)
    new[1:] = (cell[1:] != cell[:-1]) | (z[1:] - z[:-1] > rules.layer_gap)
    node = np.cumsum(new) - 1
    total = int(node[-1]) + 1
    starts = np.flatnonzero(new)
    support = np.bincount(node, minlength=total)
    dx, dy = ((col % S) - (S - 1) / 2) * sub, ((row % S) - (S - 1) / 2) * sub
    sums = {name: np.bincount(node, weights=w, minlength=total) for name, w in (
        ("x", dx), ("y", dy), ("xx", dx * dx), ("yy", dy * dy), ("xy", dx * dy),
        ("z", z), ("xz", dx * z), ("yz", dy * z))}
    normal = np.empty((total, 3, 3))
    normal[:, 0] = np.stack([support, sums["x"], sums["y"]], axis=1)
    normal[:, 1] = np.stack([sums["x"], sums["xx"], sums["xy"]], axis=1)
    normal[:, 2] = np.stack([sums["y"], sums["xy"], sums["yy"]], axis=1)
    rhs = np.stack([sums["z"], sums["xz"], sums["yz"]], axis=1)
    fit = (support >= 6) & (np.abs(np.linalg.det(normal)) > 1e-10)
    tilt = np.zeros(total)
    solved = np.linalg.solve(normal[fit], rhs[fit][..., None])[..., 0]
    tilt[fit] = np.degrees(np.arctan(np.hypot(solved[:, 1], solved[:, 2])))
    node_cell = cell[starts]
    group_start = np.r_[True, node_cell[1:] != node_cell[:-1]]
    rank = np.arange(total) - np.flatnonzero(group_start)[np.cumsum(group_start) - 1]
    nodes = {"cell": node_cell, "rank": rank, "z": np.bincount(node, weights=z, minlength=total) / support,
             "support": support, "clearance": np.minimum.reduceat(clearance, starts), "tilt": tilt}
    # A floor layer needs vehicle height of free space over the same share of its cell as the
    # occupancy rule asks for (one overhanging sub-cell must not close a whole cell).
    headroom = np.bincount(node, weights=(clearance >= rules.vehicle_height).astype(float), minlength=total)
    need = int((1.0 - rules.occupied_coverage) * S * S + 1e-9) + 1
    # Geometry only; the slope is judged at vehicle scale by vehicle_scale_surface (the per-cell
    # `tilt` above is a 5 cm plane fit that reads 40 deg on the 25 cm undulating-road waves).
    nodes["valid"] = (support >= rules.min_node_support) & (headroom >= need) & (rank < rules.max_layers)
    return nodes


def vehicle_scale_surface(z, valid, rules):
    """Floor height smoothed over 0.15 m and slope of the plane fit over +-0.15 m.

    Only neighbouring floor layers within `same_surface_tolerance` of the centre count, so a wall
    or a platform edge never mixes levels.  The undulating-road waves (6.7 cm peak to valley every
    25 cm, 41 deg at the steepest flank) read about 12 deg over that window, so this rule lets them
    through; `Rules.keep_out` draws them as obstacles.  A ramp steeper than `slope_max_deg` still
    reads steeper.
    """
    K, H, W = z.shape
    R, S, step = rules.slope_window_cells, rules.smooth_window_cells, rules.resolution
    k_idx, r_idx, c_idx = np.nonzero(valid)
    centre = z[k_idx, r_idx, c_idx]
    count = len(centre)
    padded = np.pad(z, ((0, 0), (R, R), (R, R)), constant_values=np.nan)
    padded_valid = np.pad(valid, ((0, 0), (R, R), (R, R)), constant_values=False)
    rows = np.arange(count)
    names = ("n", "x", "y", "xx", "yy", "xy", "z", "xz", "yz")
    acc = {name: np.zeros(count) for name in names}
    smooth_sum, smooth_count = np.zeros(count), np.zeros(count)
    for dy in range(-R, R + 1):
        for dx in range(-R, R + 1):
            candidate = padded[:, r_idx + R + dy, c_idx + R + dx].T
            usable = padded_valid[:, r_idx + R + dy, c_idx + R + dx].T
            difference = np.where(usable, np.abs(candidate - centre[:, None]), np.inf)
            best = difference.argmin(axis=1)
            weight = (difference[rows, best] <= rules.same_surface_tolerance).astype(float)
            height, x, y = candidate[rows, best], dx * step, dy * step
            height = np.where(weight > 0, height, 0.0)
            for name, value in zip(names, (1.0, x, y, x * x, y * y, x * y, height, x * height, y * height)):
                acc[name] += weight * value
            if abs(dx) <= S and abs(dy) <= S:
                smooth_sum += weight * height
                smooth_count += weight
    normal = np.empty((count, 3, 3))
    normal[:, 0] = np.stack([acc["n"], acc["x"], acc["y"]], axis=1)
    normal[:, 1] = np.stack([acc["x"], acc["xx"], acc["xy"]], axis=1)
    normal[:, 2] = np.stack([acc["y"], acc["xy"], acc["yy"]], axis=1)
    rhs = np.stack([acc["z"], acc["xz"], acc["yz"]], axis=1)
    fit = (acc["n"] >= 6) & (np.abs(np.linalg.det(normal)) > 1e-12)
    slope = np.zeros(count)
    solved = np.linalg.solve(normal[fit], rhs[fit][..., None])[..., 0]
    slope[fit] = np.degrees(np.arctan(np.hypot(solved[:, 1], solved[:, 2])))
    smooth = np.full(z.shape, np.nan)
    smooth[k_idx, r_idx, c_idx] = smooth_sum / np.maximum(smooth_count, 1.0)
    slope_map = np.zeros(z.shape)
    slope_map[k_idx, r_idx, c_idx] = slope
    return smooth, slope_map


def dense_layers(nodes, rules):
    K, Hc, Wc = rules.max_layers, rules.height_cells, rules.width_cells
    z = np.full((K, Hc, Wc), np.nan)
    valid = np.zeros((K, Hc, Wc), dtype=bool)
    support = np.zeros((K, Hc, Wc), dtype=np.int32)
    keep = nodes["rank"] < K
    rank, cell = nodes["rank"][keep], nodes["cell"][keep]
    z[rank, cell // Wc, cell % Wc] = nodes["z"][keep]
    valid[rank, cell // Wc, cell % Wc] = nodes["valid"][keep]
    support[rank, cell // Wc, cell % Wc] = nodes["support"][keep]
    return z, valid, support


def _components(total, a, b):
    """Connected-component label of every node (union-find; scipy is not used because the
    system scipy does not import next to the user-site numpy 2)."""
    parent = list(range(total))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for u, v in zip(a.tolist(), b.tolist()):
        ru, rv = find(u), find(v)
        if ru != rv:
            parent[max(ru, rv)] = min(ru, rv)
    return np.array([find(i) for i in range(total)])


def _seed_node(z, valid, support, rules, x, y):
    K, Hc, Wc = z.shape
    row0, col0 = int(y / rules.resolution), int(x / rules.resolution)
    for radius in range(0, 11):
        best = None
        for row in range(max(0, row0 - radius), min(Hc, row0 + radius + 1)):
            for col in range(max(0, col0 - radius), min(Wc, col0 + radius + 1)):
                if max(abs(row - row0), abs(col - col0)) != radius:
                    continue
                for k in range(K):
                    if valid[k, row, col] and (best is None or support[k, row, col] > best[0]):
                        best = (int(support[k, row, col]), k, row, col)
        if best is not None:
            return (best[1] * Hc + best[2]) * Wc + best[3]
    raise SystemExit(f"seed ({x}, {y}) has no floor within 0.5 m")


def reachable_floor(z, valid, support, rules, seeds):
    """Floor layers connected to a seed through 4-neighbours that differ by at most step_max."""
    K, Hc, Wc = z.shape
    ids = np.arange(K * Hc * Wc).reshape(K, Hc, Wc)
    a_parts, b_parts = [], []
    for dy, dx in ((0, 1), (1, 0)):
        for k1 in range(K):
            for k2 in range(K):
                s1 = (k1, slice(0, Hc - dy), slice(0, Wc - dx))
                s2 = (k2, slice(dy, Hc), slice(dx, Wc))
                with np.errstate(invalid="ignore"):
                    ok = valid[s1] & valid[s2] & (np.abs(z[s1] - z[s2]) <= rules.step_max)
                a_parts.append(ids[s1][ok])
                b_parts.append(ids[s2][ok])
    labels = _components(K * Hc * Wc, np.concatenate(a_parts), np.concatenate(b_parts)).reshape(K, Hc, Wc)
    wanted = {int(labels.reshape(-1)[_seed_node(z, valid, support, rules, x, y)]) for x, y in seeds}
    return valid & np.isin(labels, list(wanted))


def occupancy(layers, z, reach, support, rules, coverage=None):
    """Occupied-cell mask (row 0 = y 0) plus the chosen floor height per cell."""
    coverage = rules.occupied_coverage if coverage is None else coverage
    K, Hc, Wc = z.shape
    S = rules.supersample
    score = np.where(reach, support, -1)
    chosen = score.argmax(axis=0)
    floor = np.where(score.max(axis=0) >= 0, np.take_along_axis(z, chosen[None], axis=0)[0], np.nan)
    width, height = Wc * S, Hc * S
    column = layers["column"]
    dz = layers["z"] - floor[(column // width) // S, (column % width) // S]
    with np.errstate(invalid="ignore"):
        standing = layers["exposed"] & (np.abs(dz) <= rules.support_tolerance)
        blocking = (dz > rules.step_max) & (dz < rules.vehicle_height) & ~layers["exempt"]
    free = np.zeros(width * height, dtype=bool)
    free[column[standing]] = True
    free[column[blocking]] = False
    free_count = free.reshape(Hc, S, Wc, S).sum(axis=(1, 3))
    occupied = (S * S - free_count) >= coverage * S * S - 1e-9
    return occupied | cliff_edges(occupied, floor, rules), floor


def cliff_edges(occupied, floor, rules):
    """Free cells that sit more than `step_max` above a free 4-neighbour (the top edge of a step)."""
    cliff = np.zeros_like(occupied)
    open_floor = ~occupied & np.isfinite(floor)
    for axis in (0, 1):
        n = floor.shape[axis]
        lo = [slice(None)] * 2
        hi = [slice(None)] * 2
        lo[axis], hi[axis] = slice(0, n - 1), slice(1, n)
        lo, hi = tuple(lo), tuple(hi)
        both = open_floor[lo] & open_floor[hi]
        with np.errstate(invalid="ignore"):
            dz = floor[hi] - floor[lo]
        cliff[hi] |= both & (dz > rules.step_max)
        cliff[lo] |= both & (-dz > rules.step_max)
    return cliff


def build_map(mesh_path, tunnels_path, rules, mesh_scale=1.0, keys=None):
    tunnels = load_tunnels(tunnels_path)
    if keys is None:
        keys = rasterize(iter_triangles(mesh_path, mesh_scale), rules)
    return build_map_from_keys(keys, tunnels, rules)


def build_map_from_keys(keys, tunnels, rules, extra_seeds=()):
    """Run the whole pipeline; returns the occupied mask and the intermediate products."""
    layers = build_layers(keys, rules, corridor_boxes(tunnels))
    nodes = build_nodes(layers, rules)
    z, valid, support = dense_layers(nodes, rules)
    smooth, slope = vehicle_scale_surface(z, valid, rules)
    valid = valid & (slope <= rules.slope_max_deg)
    seeds = list(rules.seeds) + list(extra_seeds)
    for tunnel in tunnels:
        seeds += [tuple(tunnel["entry"][:2]), tuple(tunnel["exit"][:2])]
    reach = reachable_floor(smooth, valid, support, rules, seeds)
    occupied, floor = occupancy(layers, z, reach, support, rules)
    return {"occupied": apply_border(apply_keep_out(occupied, rules), rules.border_cells), "floor": floor,
            "layers": layers,
            "dropped_layers": int((nodes["rank"] >= rules.max_layers).sum()),
            "nodes": nodes, "z": z, "valid": valid, "support": support, "reach": reach, "keys": keys,
            "tunnels": tunnels}


def apply_keep_out(occupied, rules):
    """Mark every `Rules.keep_out` box occupied."""
    occupied, res = occupied.copy(), rules.resolution
    for x0, x1, y0, y1 in rules.keep_out:
        occupied[int(round(y0 / res)):int(round(y1 / res)), int(round(x0 / res)):int(round(x1 / res))] = True
    return occupied


def apply_border(occupied, cells):
    occupied = occupied.copy()
    if cells > 0:
        occupied[:cells, :] = occupied[-cells:, :] = True
        occupied[:, :cells] = occupied[:, -cells:] = True
    return occupied


def encode_pgm(occupied, rules):
    """P5 image, row 0 = the top of the map (y max), as map_server expects."""
    image = np.where(occupied, OCCUPIED_PIXEL, FREE_PIXEL).astype(np.uint8)[::-1]
    return f"P5\n{rules.width_cells} {rules.height_cells}\n255\n".encode("ascii") + image.tobytes()


def decode_pgm(data):
    """Occupied mask (row 0 = y 0) of a P5 file, using the yaml thresholds of the repository."""
    header = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", data)
    width, height = int(header.group(1)), int(header.group(2))
    pixels = np.frombuffer(data[header.end():], dtype=np.uint8)[:width * height].reshape(height, width)
    return ((255 - pixels) / 255.0 > 0.65)[::-1]


def sha256(path_or_bytes):
    data = path_or_bytes if isinstance(path_or_bytes, bytes) else Path(path_or_bytes).read_bytes()
    return hashlib.sha256(data).hexdigest()


def make_manifest(rules, mesh_path, tunnels_path, pgm_bytes, occupied):
    def relative(path):
        path = Path(path).resolve()
        return str(path.relative_to(REPO_ROOT)) if REPO_ROOT in path.parents else str(path)
    manifest = {
        "generator": "scripts/generate_rmuc_2027_map.py", "generator_version": GENERATOR_VERSION,
        "rules": asdict(rules),
        "inputs": {"mesh": relative(mesh_path), "mesh_sha256": sha256(mesh_path),
                   "tunnels": relative(tunnels_path), "tunnels_sha256": sha256(tunnels_path)},
        "outputs": {"rmuc_2027.pgm": sha256(pgm_bytes), "rmuc_2027.yaml": sha256(YAML_TEXT.encode())},
        "cells": int(occupied.size), "occupied_cells": int(occupied.sum()),
    }
    return json.loads(json.dumps(manifest))     # tuples become lists, as they are on disk


def read_pcd_xyz(path):
    data = Path(path).read_bytes()
    end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = {line.split()[0]: line.split()[1:] for line in data[:end].decode("ascii").split("\n") if line.strip()}
    if header["SIZE"] != ["4"] * len(header["FIELDS"]) or header["TYPE"] != ["F"] * len(header["FIELDS"]):
        raise SystemExit(f"{path}: only all-float32 binary PCD files are supported")
    count, width = int(header["POINTS"][0]), len(header["FIELDS"])
    cloud = np.frombuffer(data[end:], dtype="<f4", count=count * width).reshape(count, width)
    normals = cloud[:, 3:6].astype(np.float64) if width >= 6 else None
    return cloud[:, :3].astype(np.float64), normals


def _shift_reduce(values, radius, reducer):
    """Square window reduce (min or max) of radius cells, separable and numpy-only."""
    out = values.copy()
    for axis in (0, 1):
        base = out.copy()
        for shift in range(1, radius + 1):
            ahead, behind = [slice(None)] * 2, [slice(None)] * 2
            ahead[axis], behind[axis] = slice(shift, None), slice(None, -shift)
            out[tuple(ahead)] = reducer(out[tuple(ahead)], base[tuple(behind)])
            out[tuple(behind)] = reducer(out[tuple(behind)], base[tuple(ahead)])
    return out


def dilate_square(mask, radius):
    return _shift_reduce(mask, radius, np.logical_or) if radius > 0 else mask


def free_labels(occupied, radius_cells):
    """8-connected labels of the free space after growing obstacles by a square radius; -1 = blocked."""
    free = ~dilate_square(occupied, radius_cells)
    H, W = free.shape
    ids = np.arange(H * W).reshape(H, W)
    a, b = [], []
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        s1 = (slice(0, H - dy), slice(max(0, -dx), W - max(0, dx)))
        s2 = (slice(dy, H), slice(max(0, dx), W - max(0, -dx)))
        ok = free[s1] & free[s2]
        a.append(ids[s1][ok])
        b.append(ids[s2][ok])
    labels = _components(H * W, np.concatenate(a), np.concatenate(b)).reshape(H, W)
    return np.where(free, labels, -1)


def pcd_wall_cells(pcd, rules):
    """Cells holding PCD points of height 0.3-0.9 m: all of them (the 5.1.2 definition, which also
    counts raised floors such as the 0.5 m highland) and only those on vertical faces (|nz| <= 0.5),
    which are walls whatever the floor height is."""
    (xyz, normals), res = pcd, rules.resolution
    ix = np.clip((xyz[:, 0] / res).astype(int), 0, rules.width_cells - 1)
    iy = np.clip((xyz[:, 1] / res).astype(int), 0, rules.height_cells - 1)
    band = (xyz[:, 2] >= 0.3) & (xyz[:, 2] <= 0.9)
    masks = {}
    for name, keep in (("absolute", band),
                       ("vertical_faces", band & (np.abs(normals[:, 2]) <= 0.5) if normals is not None else None)):
        if keep is not None:
            masks[name] = np.zeros((rules.height_cells, rules.width_cells), dtype=bool)
            masks[name][iy[keep], ix[keep]] = True
    return masks


def tunnel_widths(occupied, tunnels, rules):
    """Free run through the tunnel axis across every corridor column, in metres."""
    res, out = rules.resolution, {}
    for tunnel in tunnels:
        x0, x1, _, _ = corridor_boxes([tunnel])[0]
        lintel = [p[0] for p in tunnel["polygon"]]
        axis = int(tunnel["entry"][1] / res)
        widths = {}
        for ix in range(int(x0 / res), int(x1 / res) + 1):
            column = occupied[:, ix]
            low = high = axis
            if not column[axis]:
                while low > 0 and not column[low - 1]:
                    low -= 1
                while high < len(column) - 1 and not column[high + 1]:
                    high += 1
            widths[ix] = 0.0 if column[axis] else (high - low + 1) * res
        in_lintel = [w for ix, w in widths.items() if min(lintel) <= (ix + 0.5) * res <= max(lintel)]
        narrow = min(widths, key=widths.get)
        out[tunnel["name"]] = {"min_width_m": round(widths[narrow], 3), "min_at_x": round((narrow + 0.5) * res, 3),
                               "lintel_min_width_m": round(min(in_lintel), 3) if in_lintel else None}
    return out


def evaluate(occupied, tunnels, pcd, rules):
    """Numbers for the report: PCD wall overlap, connectivity of the three regions, tunnel widths."""
    res = rules.resolution
    ev = {"occupied_fraction": round(float(occupied.mean()), 4)}
    if pcd is not None:
        walls = pcd_wall_cells(pcd, rules)
        both = walls["absolute"] & occupied
        ev["wall_iou_absolute"] = round(float(both.sum() / (walls["absolute"] | occupied).sum()), 3)
        ev["wall_cells_in_occupied_absolute"] = round(float(both.sum() / walls["absolute"].sum()), 3)
        ev["occupied_cells_with_wall_band_point"] = round(float(both.sum() / occupied.sum()), 3)
        if "vertical_faces" in walls:
            faces = walls["vertical_faces"]
            for centimetres, radius in ((0, 0), (5, 1), (10, 2)):
                ev[f"vertical_face_cells_within_{centimetres}cm"] = round(
                    float((faces & dilate_square(occupied, radius)).sum() / faces.sum()), 3)
    points = {"west": (5.0, 8.0), "highland": (14.0, 11.6), "east": (23.0, 8.0)}
    closed = occupied.copy()
    for x0, x1, y0, y1 in corridor_boxes(tunnels):
        closed[int(y0 / res):int(y1 / res) + 1, int(x0 / res):int(x1 / res) + 1] = True
    ev["connectivity"] = {}
    for label, grid in (("open", occupied), ("tunnels_closed", closed)):
        for radius in (0.0, 0.2, 0.3):
            labels = free_labels(grid, int(round(radius / res)))
            ids = {name: int(labels[int(y / res), int(x / res)]) for name, (x, y) in points.items()}
            ev["connectivity"][f"{label}_erosion_{radius:.2f}m"] = {
                f"{a}-{b}": bool(ids[a] >= 0 and ids[a] == ids[b])
                for a, b in (("west", "highland"), ("west", "east"), ("highland", "east"))}
    ev["tunnels"] = tunnel_widths(occupied, tunnels, rules)
    return ev


def print_report(title, ev):
    print(f"== {title}")
    print(f"   occupied fraction            {ev['occupied_fraction']:.4f}")
    if "wall_iou_absolute" in ev:
        print(f"   PCD wall IoU (z 0.3-0.9 m)   {ev['wall_iou_absolute']:.3f}   wall cells inside occupied "
              f"{100 * ev['wall_cells_in_occupied_absolute']:.1f} %   occupied cells holding a wall-band point "
              f"{100 * ev['occupied_cells_with_wall_band_point']:.1f} %")
    if "vertical_face_cells_within_0cm" in ev:
        print("   PCD vertical-face cells (|nz|<=0.5, z 0.3-0.9 m) inside / within 5 cm / within 10 cm of occupied: "
              + " / ".join(f"{100 * ev[f'vertical_face_cells_within_{c}cm']:.1f} %" for c in (0, 5, 10)))
    for label, pairs in ev["connectivity"].items():
        print(f"   connectivity {label:26s} " + "  ".join(f"{k}={'Y' if v else 'N'}" for k, v in pairs.items()))
    for name, info in ev["tunnels"].items():
        print(f"   tunnel {name:16s} narrowest {info['min_width_m']:.2f} m at x={info['min_at_x']:.2f}, "
              f"under the lintel {info['lintel_min_width_m']} m")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mesh", type=Path, default=DEFAULT_MESH, help="field mesh (.obj in metres, or binary .stl)")
    parser.add_argument("--mesh-scale", type=float, default=1.0, help="multiply mesh coordinates (0.001 for the mm STL)")
    parser.add_argument("--tunnels", type=Path, default=DEFAULT_TUNNELS)
    parser.add_argument("--pcd", type=Path, default=DEFAULT_PCD)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_MAP_DIR)
    parser.add_argument("--check", action="store_true", help="do not write; exit 1 if the committed map differs")
    parser.add_argument("--report", action="store_true", help="print PCD overlap, connectivity and tunnel widths")
    parser.add_argument("--evaluate", type=Path, help="only report on an existing .pgm (no mesh needed)")
    parser.add_argument("--report-json", type=Path, help="also write the report numbers as JSON")
    args = parser.parse_args(argv)
    rules = Rules()
    tunnels = load_tunnels(args.tunnels)
    pcd = read_pcd_xyz(args.pcd) if args.pcd.exists() and (args.report or args.evaluate) else None
    if args.evaluate:
        ev = evaluate(decode_pgm(args.evaluate.read_bytes()), tunnels, pcd, rules)
        print_report(str(args.evaluate), ev)
        return 0
    result = build_map(args.mesh, args.tunnels, rules, args.mesh_scale)
    occupied = result["occupied"]
    pgm = encode_pgm(occupied, rules)
    manifest = make_manifest(rules, args.mesh, args.tunnels, pgm, occupied)
    pgm_path, yaml_path = args.out_dir / "rmuc_2027.pgm", args.out_dir / "rmuc_2027.yaml"
    manifest_path = args.out_dir / "rmuc_2027.manifest.json"
    status = 0
    if args.check:
        committed = decode_pgm(pgm_path.read_bytes())
        if pgm_path.read_bytes() != pgm or yaml_path.read_text() != YAML_TEXT:
            diff = np.argwhere(committed != occupied)
            print(f"DRIFT: {pgm_path} differs from the generator output in {len(diff)} cells"
                  + (f" (rows {diff[:, 0].min()}-{diff[:, 0].max()}, cols {diff[:, 1].min()}-{diff[:, 1].max()})" if len(diff) else ""))
            status = 1
        elif json.loads(manifest_path.read_text()) != manifest:
            print(f"DRIFT: {manifest_path} does not describe the current inputs or rules")
            status = 1
        else:
            print(f"OK: {pgm_path} is exactly what the generator produces from {manifest['inputs']['mesh']}")
    else:
        args.out_dir.mkdir(parents=True, exist_ok=True)
        pgm_path.write_bytes(pgm)
        yaml_path.write_text(YAML_TEXT)
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        print(f"wrote {pgm_path} ({int(occupied.sum())} of {occupied.size} cells occupied), {yaml_path}, {manifest_path}")
    if args.report:
        ev = evaluate(occupied, tunnels, pcd, rules)
        print_report("generated map", ev)
        print(f"   mesh columns whose up/down faces do not balance (cracks): {100 * result['layers']['unbalanced_columns']:.2f} %;"
              f" floor layers dropped above max_layers={rules.max_layers}: {result['dropped_layers']}")
        any_overlap = apply_border(occupancy(result["layers"], result["z"], result["reach"], result["support"], rules,
                                             coverage=1.0 / rules.supersample ** 2)[0], rules.border_cells)
        for name, info in tunnel_widths(any_overlap, tunnels, rules).items():
            print(f"   (any-overlap rasterisation) tunnel {name}: narrowest {info['min_width_m']:.2f} m, "
                  f"under the lintel {info['lintel_min_width_m']} m")
        if args.report_json:
            args.report_json.write_text(json.dumps(ev, indent=2, sort_keys=True) + "\n")
    return status


if __name__ == "__main__":
    sys.exit(main())
