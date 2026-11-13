#!/usr/bin/env python3
"""Convert the official RMUL 2027 3V3 field mesh into every asset the ATS stack needs.

Input : 2027_map/RMUL2027_3V3_V1.0.0.STL (binary, millimetres; x 0..12600, z 0..8600 horizontal,
        y is the *downward* height axis: the field floor is y = 420, wall tops are y = 0).
Output (map frame, metres, floor z = 0, origin at the outer lower-left corner of the field):

* src/sim/ats_mujoco_sim/models/meshes/rmul_2027.obj    visual / LiDAR mesh
* src/sim/ats_mujoco_sim/models/rmul_2027_height.bin    MuJoCo hfield (physical ground, 1 cm)
* src/sim/ats_mujoco_sim/models/rmul_2027_roof_boxes.xml tunnel lintel collision boxes
* src/sim/ats_mujoco_sim/models/rmul_2027_swerve.xml    MuJoCo scene (robot copied from the 2027 scene)
* src/sim/ats_mujoco_sim/config/tunnels_rmul_2027.yaml  tunnel geometry (single source of truth)
* src/ats_sentry_bringup/map/rmul_2027.{pgm,yaml}       nav2 / ROG-Map static map, 0.05 m
* src/ats_sentry_bringup/pcd/rmul_2027.pcd              GICP prior cloud

Tunnel lintel (rule book 3.2.3): the official lintel leaves only 0.250 m under it for a 0.242 m
vehicle, i.e. 8 mm.  `--roof-bottom` (default 0.32 m) lifts the lintel; the piers on both sides are
lifted with it, so the mesh stays closed.

Orientation: `mirror_y` (default) puts the start zone in the lower-left corner, which is how the STL
looks from the CAD "top" view and what the project asked for.  `official` is the rule-book picture
(start zone top-left); `rotate_ccw` keeps the official handedness but gives a portrait 8.6 x 12.6 m map.

    python3 scripts/convert_rmul_2027_map.py            # write everything
    python3 scripts/convert_rmul_2027_map.py --check    # fail if committed outputs drift
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "src/sim/ats_mujoco_sim"))

import generate_rmuc_2027_map as gen  # noqa: E402  (reuses the proven floor / wall / cliff rules)
from ats_mujoco_sim.rmuc_tunnel_roofs import generate_roof_xml  # noqa: E402

DEFAULT_STL = REPO_ROOT / "2027_map/RMUL2027_3V3_V1.0.0.STL"
SIM = REPO_ROOT / "src/sim/ats_mujoco_sim"
BRINGUP = REPO_ROOT / "src/ats_sentry_bringup"
OUT = {
    "obj": SIM / "models/meshes/rmul_2027.obj",
    "hfield": SIM / "models/rmul_2027_height.bin",
    "roofs": SIM / "models/rmul_2027_roof_boxes.xml",
    "model": SIM / "models/rmul_2027_swerve.xml",
    "tunnels": SIM / "config/tunnels_rmul_2027.yaml",
    "pgm": BRINGUP / "map/rmul_2027.pgm",
    "yaml": BRINGUP / "map/rmul_2027.yaml",
    "pcd": BRINGUP / "pcd/rmul_2027.pcd",
}
TEMPLATE_MODEL = SIM / "models/rmuc_2027_swerve.xml"

FLOOR_Y_MM = 420.0           # STL y of the playing surface
OUTER_X_MM, OUTER_Z_MM = 12600.0, 8600.0
ORIENTATIONS = ("mirror_y", "official", "rotate_ccw")
STOCK_ROOF_BOTTOM = 0.250    # official lintel underside above the floor
STOCK_ROOF_THICK = 0.020
VEHICLE_HEIGHT = 0.242       # report 2.1.8, includes the LiDAR
HFIELD_RES = 0.01
HFIELD_BASE = 0.03
PCD_SPACING = 0.03


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if 84 + 50 * count != len(data):
        raise SystemExit(f"{path} is not a binary STL")
    dtype = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data, dtype=dtype, offset=84, count=count)["v"].astype(np.float64)


def lift_roofs(tri_mm: np.ndarray, roof_bottom: float):
    """Raise both lintels (STL vertices 0.245..0.275 m above the floor) so the underside is `roof_bottom`."""
    height = (FLOOR_Y_MM - tri_mm[:, :, 1]) / 1000.0
    lifted = (height > 0.245) & (height < 0.275)
    unique = np.unique(np.round(tri_mm[lifted], 1), axis=0)
    if len(unique) != 16:
        raise SystemExit(f"expected 16 lintel vertices (2 lintels x 8), found {len(unique)}")
    out = tri_mm.copy()
    out[:, :, 1] = np.where(lifted, tri_mm[:, :, 1] - (roof_bottom - STOCK_ROOF_BOTTOM) * 1000.0, tri_mm[:, :, 1])
    return out, lifted


def to_map_frame(tri_mm: np.ndarray, orientation: str):
    """STL mm -> map metres (x, y horizontal, z up, floor 0).  Returns triangles and the map size (x, y)."""
    x, y, z = (tri_mm[:, :, i] / 1000.0 for i in range(3))
    up = (FLOOR_Y_MM / 1000.0) - y
    width, depth = OUTER_X_MM / 1000.0, OUTER_Z_MM / 1000.0
    if orientation == "official":
        mx, my, size, flip = x, z, (width, depth), False
    elif orientation == "mirror_y":
        mx, my, size, flip = x, depth - z, (width, depth), True
    elif orientation == "rotate_ccw":
        mx, my, size, flip = depth - z, x, (depth, width), False
    else:
        raise SystemExit(f"unknown orientation {orientation}")
    tri = np.stack([mx, my, up], axis=2)
    if flip:                                        # a mirror reverses the winding
        tri = tri[:, [0, 2, 1], :]
    return tri, size


def normals_and_areas(tri: np.ndarray):
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = 0.5 * np.linalg.norm(cross, axis=1)
    return cross / np.maximum(2.0 * area, 1e-12)[:, None], area


def roof_plates(tri: np.ndarray, lifted: np.ndarray, roof_bottom: float):
    """Underside slot (axis aligned rectangle) of each lintel and the lintel thickness."""
    pts = tri[lifted]
    under = pts[np.abs(pts[:, 2] - roof_bottom) < 1e-4]
    top = pts[np.abs(pts[:, 2] - (roof_bottom + STOCK_ROOF_THICK)) < 1e-4]
    order = np.argsort(under[:, 0])                 # the two lintels are far apart in x
    split = int(np.argmax(np.diff(under[order, 0])))
    plates = []
    for group in (order[:split + 1], order[split + 1:]):
        plates.append((float(under[group, 0].min()), float(under[group, 0].max()),
                       float(under[group, 1].min()), float(under[group, 1].max())))
    thick = float(top[:, 2].mean() - roof_bottom) if len(top) else STOCK_ROOF_THICK
    return plates, thick


# ----------------------------------------------------------------------------- hfield ---------------


def column_hits(tri: np.ndarray, nx: int, ny: int, res: float):
    """Vertical-ray hits at cell centres: (cell index, height, facing_up)."""
    cells, heights, ups = [], [], []
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    det = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
    for i in np.flatnonzero(np.abs(det) > 1e-12):
        ax, ay, az = tri[i, 0]
        lo = np.floor(tri[i, :, :2].min(axis=0) / res - 0.5).astype(int) + 1
        hi = np.ceil(tri[i, :, :2].max(axis=0) / res - 0.5).astype(int)
        i0, j0 = max(lo[0], 0), max(lo[1], 0)
        i1, j1 = min(hi[0], nx - 1), min(hi[1], ny - 1)
        if i1 < i0 or j1 < j0:
            continue
        gx, gy = np.meshgrid((np.arange(i0, i1 + 1) + 0.5) * res, (np.arange(j0, j1 + 1) + 0.5) * res)
        dx, dy = gx - ax, gy - ay
        w1 = (dx * (c[i, 1] - ay) - (c[i, 0] - ax) * dy) / det[i]
        w2 = ((b[i, 0] - ax) * dy - dx * (b[i, 1] - ay)) / det[i]
        w0 = 1.0 - w1 - w2
        inside = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
        if not inside.any():
            continue
        z = w0 * az + w1 * b[i, 2] + w2 * c[i, 2]
        jj, ii = np.nonzero(inside)
        cells.append((jj + j0) * nx + (ii + i0))
        heights.append(z[inside])
        ups.append(np.full(len(jj), det[i] > 0.0))
    return np.concatenate(cells), np.concatenate(heights), np.concatenate(ups)


def ground_height(tri: np.ndarray, size, res: float = HFIELD_RES):
    """Top of the lowest solid interval of every column (the lintels hover above it and are excluded)."""
    nx, ny = int(round(size[0] / res)), int(round(size[1] / res))
    cell, h, up = column_hits(tri, nx, ny, res)
    order = np.lexsort((h, cell))
    cell, h, up = cell[order], h[order], up[order]
    keep = np.ones(len(cell), dtype=bool)               # a ray on a shared edge hits both triangles
    keep[1:] = ~((cell[1:] == cell[:-1]) & (up[1:] == up[:-1]) & (np.abs(h[1:] - h[:-1]) < 2e-3))
    cell, h, up = cell[keep], h[keep], up[keep]
    ground = np.full(nx * ny, np.nan)
    start = np.r_[0, np.flatnonzero(cell[1:] != cell[:-1]) + 1]
    stop = np.r_[start[1:], len(cell)]
    for s, e in zip(start.tolist(), stop.tolist()):
        hits_h, hits_up = h[s:e], up[s:e]
        top = None
        k = 0
        while k + 1 < len(hits_h):
            if not hits_up[k] and hits_up[k + 1]:       # solid between a down face and the next up face
                top = hits_h[k + 1]
                while k + 3 < len(hits_h) and not hits_up[k + 2] and hits_up[k + 3] and hits_h[k + 2] - top < 0.01:
                    top, k = hits_h[k + 3], k + 2       # a touching solid continues the same interval
                break
            k += 1
        if top is None and hits_up.any():
            top = hits_h[hits_up].max()
        ground[cell[s]] = np.nan if top is None else top
    ground = ground.reshape(ny, nx)
    if np.isnan(ground).any():
        raise SystemExit(f"{int(np.isnan(ground).sum())} hfield columns have no surface")
    return ground


def encode_hfield(ground: np.ndarray):
    top = float(ground.max())
    data = np.clip(ground, 0.0, None) / top
    return struct.pack("<ii", ground.shape[0], ground.shape[1]) + data.astype("<f4").tobytes(), top


# ------------------------------------------------------------------------------- map -----------------


def build_nav_map(tri: np.ndarray, size, seeds):
    width_cells, height_cells = int(round(size[0] / 0.05)), int(round(size[1] / 0.05))
    rules = gen.Rules(width_cells=width_cells, height_cells=height_cells, seeds=tuple(seeds), keep_out=(),
                      border_cells=1)
    shifted = tri.copy()
    shifted[:, :, 2] += 0.1                         # the rasteriser wants z >= 0; the base slab is 0.1 m thick
    keys = gen.rasterize([shifted], rules)
    result = gen.build_map_from_keys(keys, [], rules)
    return rules, result["occupied"]


def pgm_yaml(rules, occupied):
    yaml_text = ("image: rmul_2027.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
                 "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return gen.encode_pgm(occupied, rules), yaml_text


def free_span(line: np.ndarray, centre: int, res: float = 0.05):
    lo = hi = centre
    while lo > 0 and not line[lo - 1]:
        lo -= 1
    while hi < len(line) - 1 and not line[hi + 1]:
        hi += 1
    return lo * res, (hi + 1) * res


def tunnel_corridor(occupied: np.ndarray, plate, res: float = 0.05, slack: float = 0.10):
    """Free channel under a lintel: the y span at the lintel, extended along x while the free run stays
    inside `slack` of that width (i.e. while there is a wall on both sides)."""
    x0, x1, y0, y1 = plate
    row, col = int(round((y0 + y1) / 2 / res)), int(round((x0 + x1) / 2 / res))
    ylo, yhi = free_span(occupied[:, col], row)
    width = yhi - ylo

    def narrow(c):
        if occupied[row, c]:
            return False
        lo, hi = free_span(occupied[:, c], row)
        return hi - lo <= width + slack

    left = right = col
    while left > 0 and narrow(left - 1):
        left -= 1
    while right < occupied.shape[1] - 1 and narrow(right + 1):
        right += 1
    return left * res, (right + 1) * res, ylo, yhi


# ------------------------------------------------------------------------------- pcd ----------------


def sample_pcd(tri: np.ndarray, spacing: float = PCD_SPACING):
    normal, area = normals_and_areas(tri)
    keep = (tri[:, :, 2].max(axis=1) > 0.0005) & (area > 1e-9)    # drop the base slab underside and sides
    tri, normal, area = tri[keep], normal[keep], area[keep]
    rng = np.random.default_rng(20261110)
    expected = area / (spacing * spacing)
    count = np.floor(expected).astype(int) + (rng.random(len(area)) < expected - np.floor(expected))
    owner = np.repeat(np.arange(len(area)), count)
    r = rng.random((len(owner), 2))
    flip = r.sum(axis=1) > 1.0
    r[flip] = 1.0 - r[flip]
    tri_o = tri[owner]
    xyz = tri_o[:, 0] + r[:, :1] * (tri_o[:, 1] - tri_o[:, 0]) + r[:, 1:] * (tri_o[:, 2] - tri_o[:, 0])
    above = xyz[:, 2] >= -0.001                       # the 0.1 m base slab below the floor is never seen
    return xyz[above].astype(np.float32), normal[owner][above].astype(np.float32)


def encode_pcd(xyz: np.ndarray, normals: np.ndarray) -> bytes:
    n = len(xyz)
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z normal_x normal_y normal_z\n"
              f"SIZE 4 4 4 4 4 4\nTYPE F F F F F F\nCOUNT 1 1 1 1 1 1\nWIDTH {n}\nHEIGHT 1\n"
              f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    return header.encode("ascii") + np.hstack([xyz, normals]).astype("<f4").tobytes()


def encode_obj(tri: np.ndarray) -> bytes:
    flat = np.round(tri.reshape(-1, 3), 6)
    vertices, inverse = np.unique(flat, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3) + 1
    lines = ["# RMUL 2027 3V3 field, metres, map frame, floor z = 0 (scripts/convert_rmul_2027_map.py)"]
    lines += [f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}" for v in vertices]
    lines += [f"f {a} {b} {c}" for a, b, c in faces]
    return ("\n".join(lines) + "\n").encode("ascii")


# ---------------------------------------------------------------------------- scene ---------------


def tunnels_yaml(plates, thickness, roof_bottom, orientation, size, corridors):
    lines = [
        "# RMUL 2027 3V3 隧道（由 scripts/convert_rmul_2027_map.py 生成，勿手改）。坐标与 rmul_2027.yaml 一致，单位 m。",
        f"# 官方顶板下沿离地 {STOCK_ROOF_BOTTOM:.3f} m、车高 {VEHICLE_HEIGHT:.3f} m（含雷达），净空仅 8 mm，车过不去；",
        f"# 这里把两块顶板连同墩子整体抬高，下沿离地 {roof_bottom:.3f} m（净空 {roof_bottom - VEHICLE_HEIGHT:.3f} m）。",
        f"# 布局 orientation={orientation}，地图 {size[0]:.1f} x {size[1]:.1f} m，原点在外框左下角。",
        "# polygon 是顶板下方的槽（顶板投影），corridor_polygon 是该槽所在的 PGM 自由通道（沿轴线两个方向直到障碍）。",
        "frame: map", "tunnels:"]
    for k, ((x0, x1, y0, y1), (cx0, cx1, cy0, cy1)) in enumerate(zip(plates, corridors)):
        mid = (cy0 + cy1) / 2.0
        lines += [
            f"  - name: tunnel_{'AB'[k]}",
            f"    polygon: [[{x0:.3f}, {y0:.3f}], [{x1:.3f}, {y0:.3f}], [{x1:.3f}, {y1:.3f}], [{x0:.3f}, {y1:.3f}]]",
            f"    corridor_polygon: [[{cx0:.3f}, {cy0:.3f}], [{cx1:.3f}, {cy0:.3f}], [{cx1:.3f}, {cy1:.3f}], "
            f"[{cx0:.3f}, {cy1:.3f}]]",
            "    floor_z: 0.0", f"    roof_bottom_z: {roof_bottom:.3f}",
            f"    roof_top_z: {roof_bottom + thickness:.3f}", "    roof_slope_y: 0.0",
            f"    width: {y1 - y0:.3f}", "    axis_yaw: 0.0",
            f"    entry: [{cx0 - 0.4:.3f}, {mid:.3f}, 0.0]",
            f"    exit: [{cx1 + 0.4:.3f}, {mid:.3f}, 0.0]"]
    return "\n".join(lines) + "\n"


def scene_xml(size, top, template: Path):
    """The 2027 robot + actuators, wrapped in the RMUL field assets."""
    text = template.read_text(encoding="utf-8")
    cx, cy = size[0] / 2.0, size[1] / 2.0
    hx, hy = (size[0] - HFIELD_RES) / 2.0, (size[1] - HFIELD_RES) / 2.0

    def swap(pattern, new):
        nonlocal text
        text, n = re.subn(pattern, lambda _m: new, text, count=1, flags=re.S)
        if n != 1:
            raise SystemExit(f"template drifted: {pattern!r} not found")

    swap(r'<mujoco model="rmuc_2027_swerve">', '<mujoco model="rmul_2027_swerve">')
    swap(r"<!--\s*RMUC 2027 V1\.0\.0 field\..*?-->",
         "<!--\n      RMUL 2027 3V3 V1.0.0 field, generated by scripts/convert_rmul_2027_map.py from\n"
         f"      2027_map/RMUL2027_3V3_V1.0.0.STL (metres, map frame x [0, {size[0]:.1f}], y [0, {size[1]:.1f}], floor z = 0).\n"
         "      The visual mesh is exact LiDAR ray geometry and never a contact geom; physical ground is the 1 cm\n"
         "      float hfield plus the tunnel lintel boxes of rmul_2027_roof_boxes.xml.  The geom names keep the\n"
         "      rmuc_2027_ prefix because sim_node / the acceptance tools look the ground up by that name.\n"
         "    -->")
    swap(r'<hfield name="rmuc_2027_field_hfield"[^>]*/>',
         f'<hfield name="rmuc_2027_field_hfield" size="{hx:.4f} {hy:.4f} {top:.4f} {HFIELD_BASE:.3f}"\n'
         '            file="rmul_2027_height.bin"/>')
    swap(r'<mesh name="rmuc_2027_field_stl"[^>]*/>', '<mesh name="rmuc_2027_field_stl" file="meshes/rmul_2027.obj"/>')
    swap(r'<light name="sun" directional="true" pos="[^"]*"',
         f'<light name="sun" directional="true" pos="{cx:.2f} {cy:.2f} 8"')
    swap(r'<geom name="floor" type="plane" pos="[^"]*"\s*size="[^"]*"',
         f'<geom name="floor" type="plane" pos="{cx:.2f} {cy:.2f} -0.030"\n          size="{cx + 1:.1f} {cy + 1:.1f} 0.05"')
    swap(r'<geom name="rmuc_2027_field" type="hfield" hfield="rmuc_2027_field_hfield"\s*pos="[^"]*"',
         f'<geom name="rmuc_2027_field" type="hfield" hfield="rmuc_2027_field_hfield"\n          pos="{cx:.4f} {cy:.4f} 0"')
    swap(r'<include file="rmuc_2027_roof_boxes.xml"/>', '<include file="rmul_2027_roof_boxes.xml"/>')
    return text


def build(args):
    tri_mm = read_stl(args.stl)
    tri_mm, lifted = lift_roofs(tri_mm, args.roof_bottom)
    tri, size = to_map_frame(tri_mm, args.orientation)
    plates, thickness = roof_plates(tri, lifted, args.roof_bottom)

    ground = ground_height(tri, size)
    hfield, top = encode_hfield(ground)

    rules, occupied = build_nav_map(tri, size, [tuple(args.start), (size[0] / 2.0, size[1] / 2.0)])
    pgm, yaml_text = pgm_yaml(rules, occupied)

    corridors = [tunnel_corridor(occupied, plate) for plate in plates]
    tunnels = tunnels_yaml(plates, thickness, args.roof_bottom, args.orientation, size, corridors)
    tmp = OUT["tunnels"].with_suffix(".yaml.tmp")
    tmp.parent.mkdir(parents=True, exist_ok=True)
    tmp.write_text(tunnels, encoding="utf-8")
    try:
        roofs = generate_roof_xml(str(tmp))
    finally:
        tmp.unlink()

    xyz, normals = sample_pcd(tri)
    outputs = {
        "obj": encode_obj(tri), "hfield": hfield, "roofs": roofs.encode(), "pgm": pgm, "yaml": yaml_text.encode(),
        "pcd": encode_pcd(xyz, normals), "tunnels": tunnels.encode(),
        "model": scene_xml(size, top, TEMPLATE_MODEL).encode(),
    }
    info = {"orientation": args.orientation, "size_m": size, "roof_bottom_m": args.roof_bottom,
            "plates": plates, "corridors": corridors, "hfield_top_m": top, "occupied_cells": int(occupied.sum()),
            "cells": int(occupied.size), "pcd_points": len(xyz)}
    return outputs, info, occupied


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stl", type=Path, default=DEFAULT_STL)
    parser.add_argument("--orientation", choices=ORIENTATIONS, default="mirror_y")
    parser.add_argument("--roof-bottom", type=float, default=0.32,
                        help="lintel underside above the floor in metres (official 0.25, vehicle 0.242)")
    parser.add_argument("--start", type=float, nargs=2, default=(1.8, 1.3), metavar=("X", "Y"),
                        help="seed of the reachable floor (inside the lower-left start zone)")
    parser.add_argument("--check", action="store_true", help="write nothing; exit 1 if committed files differ")
    args = parser.parse_args(argv)
    if args.roof_bottom < VEHICLE_HEIGHT + 0.03:
        raise SystemExit(f"--roof-bottom {args.roof_bottom} leaves < 3 cm over the {VEHICLE_HEIGHT} m vehicle")
    outputs, info, _ = build(args)
    if args.check:
        bad = [name for name, data in outputs.items() if not OUT[name].exists() or OUT[name].read_bytes() != data]
        print("DRIFT: " + ", ".join(bad) if bad else "OK: committed RMUL 2027 assets match the generator")
        return 1 if bad else 0
    for name, data in outputs.items():
        OUT[name].parent.mkdir(parents=True, exist_ok=True)
        OUT[name].write_bytes(data)
        print(f"wrote {OUT[name].relative_to(REPO_ROOT)} ({len(data)} bytes, sha256 {hashlib.sha256(data).hexdigest()[:12]})")
    print(json.dumps(info, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
