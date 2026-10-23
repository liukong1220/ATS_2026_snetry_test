#!/usr/bin/env python3
"""Offline Mid360 mount checks for the chassis-mounted lidar (S0 items 8 and 9).

``self``   Switch the simulator's self-exclusion off and report where the
           chassis' own returns land, plus the body-frame crop box that removes
           them.
``tunnel`` Place the vehicle in a 0.80 m wide tunnel (+x along the axis, centred)
           and report what the lidar sees on the left wall, the right wall and
           the ground ahead, with and without two crop strategies:
             sphere : Point-LIO's `blind` ball (0.3 m around the sensor)
             box    : the vehicle bounding box inflated by a 0.1 m ball
No ROS needed: it traces the full 800000-ray Mid360 pattern cycle (about 33 s of
sensor time) against a static MuJoCo scene.

  python3 scripts/mujoco_lidar_visibility_probe.py self   --tilt-direction front
  python3 scripts/mujoco_lidar_visibility_probe.py tunnel --tilt-direction left
"""

import argparse
import json
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "sim" / "ats_mujoco_sim"
sys.path.insert(0, str(PKG))

from mujoco_lidar import scan_gen  # noqa: E402
from mujoco_lidar.multiray_compat import multi_ray  # noqa: E402

from ats_mujoco_sim.lidar_mount import BASE_LINK_HEIGHT, chassis_mount  # noqa: E402
from ats_mujoco_sim.model_patch import patch_lidar_mount  # noqa: E402

MODEL = PKG / "models" / "rmuc_2027_swerve.xml"
BASE_X_HALF, BASE_Y_HALF = 0.2905, 0.2855      # body envelope after the axis correction
BASE_Z_MIN, BASE_Z_MAX = -BASE_LINK_HEIGHT, 0.062   # wheel contact plane .. envelope top
STRIP = [
    r"<hfield\b[^>]*?/>",
    r'<mesh name="rmuc_2025_field_stl"[^>]*?/>',
    r'<geom name="floor"[^>]*?/>',
    r'<geom name="rmuc_2025_field_visual"[^>]*?/>',
    r'<geom name="rmuc_2025_field"[^>]*?/>',
    r'<include file="rmuc_2027_roof_boxes.xml"/>',
]


def build_model(pose, scene_geoms, with_walls=False):
    """Vehicle-only model (field stripped) with ``scene_geoms`` XML added."""
    text = patch_lidar_mount(MODEL.read_text("utf-8"), pose)
    for pattern in STRIP:
        text, count = re.subn(pattern, "", text, flags=re.S)
        if count == 0:
            raise SystemExit(f"model changed, cannot strip {pattern}")
    text = text.replace("</worldbody>", scene_geoms + "\n  </worldbody>", 1)
    path = PKG / "models" / ".visibility_probe_model.xml"
    path.write_text(text, "utf-8")
    try:
        return mujoco.MjModel.from_xml_path(str(path))
    finally:
        path.unlink()


# Visualisation-only geoms of the chassis body: a transparent envelope box and a
# red front strip.  They are not hardware, so a physical-occlusion trace ignores
# them (the sensor sits inside the envelope and just ahead of the strip).
VISUAL_ONLY_GEOMS = ("base_size_envelope", "front_marker")


def trace(model, data, exclude_self, max_range=40.0, min_range=0.1, hide_visual=False):
    """Trace the whole Mid360 cycle. Returns hits (world, geom names, ranges, ray idx).

    ``hide_visual`` moves ``VISUAL_ONLY_GEOMS`` to the ignored geom group while tracing.
    """
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in VISUAL_ONLY_GEOMS]
    ids = [i for i in ids if i >= 0]
    saved = {i: int(model.geom_group[i]) for i in ids}
    if hide_visual:
        for i in ids:
            model.geom_group[i] = 3
    try:
        return _trace(model, data, exclude_self, max_range, min_range)
    finally:
        for i, group in saved.items():
            model.geom_group[i] = group


def _trace(model, data, exclude_self, max_range, min_range):
    gen = scan_gen.LivoxGenerator("mid360")
    angles = np.asarray(gen.ray_angles, dtype=np.float64)
    site = data.site("lidar_raycast_site")
    origin = np.array(site.xpos, dtype=np.float64).reshape(3, 1)
    rot = np.array(site.xmat, dtype=np.float64).reshape(3, 3)
    geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.ubyte)
    geomgroup[3] = 0
    body = model.body("base_link").id if exclude_self else -1
    world_points, geom_ids, ranges, ray_index = [], [], [], []
    chunk = 100000
    for start in range(0, len(angles), chunk):
        theta, phi = angles[start:start + chunk, 0], angles[start:start + chunk, 1]
        local = np.stack(
            [np.cos(phi) * np.cos(theta), np.cos(phi) * np.sin(theta), np.sin(phi)], axis=-1)
        world = local @ rot.T
        world /= np.linalg.norm(world, axis=1, keepdims=True)
        geomid = np.full(len(theta), 0, dtype=np.int32)
        dist = np.full(len(theta), max_range, dtype=np.float64)
        multi_ray(model, data, origin, np.ascontiguousarray(world.reshape(-1)), geomgroup, 1,
                  body, geomid, dist, len(theta), max_range)
        hit = (geomid >= 0) & (dist >= min_range)
        world_points.append(origin.T + world[hit] * dist[hit, None])
        geom_ids.append(geomid[hit])
        ranges.append(dist[hit])
        ray_index.append(np.nonzero(hit)[0] + start)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or f"geom{int(g)}"
             for g in np.concatenate(geom_ids)]
    return (np.concatenate(world_points), np.array(names, dtype=object),
            np.concatenate(ranges), np.concatenate(ray_index), len(angles))


def lidar_frame_origin(data):
    return np.array(data.site("lidar_site").xpos, dtype=np.float64)


def box_sdf(points_body, inflate=0.0):
    """Signed distance of body-frame points to the vehicle envelope box (>0 outside)."""
    lo = np.array([-BASE_X_HALF, -BASE_Y_HALF, BASE_Z_MIN])
    hi = np.array([BASE_X_HALF, BASE_Y_HALF, BASE_Z_MAX])
    centre, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
    q = np.abs(points_body - centre) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(np.max(q, axis=1), 0.0)
    return outside + inside - inflate


def setup(args, scene_geoms):
    pose = chassis_mount(tilt_direction=args.tilt_direction, tilt_deg=args.tilt_deg,
                         x=args.mount_x, y=args.mount_y, height_above_ground=args.mount_height)
    model = build_model(pose, scene_geoms)
    data = mujoco.MjData(model)
    data.qpos[:7] = [0.0, 0.0, BASE_LINK_HEIGHT, 1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    return pose, model, data


def probe_self(args):
    pose, model, data = setup(args, "")
    points, names, ranges, _, total = trace(model, data, exclude_self=False, max_range=5.0)
    base = np.array(data.xpos[model.body("base_link").id])
    body_pts = points - base
    result = {
        "mount": {"tilt_direction": args.tilt_direction, "tilt_deg": args.tilt_deg,
                  "position_in_base_link": list(map(float, pose.position))},
        "rays": int(total), "own_hits": int(len(points)),
        "own_hit_fraction": float(len(points) / total),
    }
    if len(points):
        counts = {}
        for name in names:
            counts[name] = counts.get(name, 0) + 1
        result["own_hits_by_geom"] = dict(sorted(counts.items(), key=lambda kv: -kv[1]))
        result["own_hit_range_m"] = {"min": float(ranges.min()), "p50": float(np.median(ranges)),
                                      "max": float(ranges.max())}
        lo, hi = body_pts.min(axis=0), body_pts.max(axis=0)
        result["own_hit_bbox_body_frame_m"] = {"min": lo.round(4).tolist(), "max": hi.round(4).tolist()}
        centre_dist = np.linalg.norm(points - lidar_frame_origin(data), axis=1)
        result["own_hits_beyond_blind_0p3"] = int((centre_dist > 0.3).sum())
        result["own_hit_fraction_beyond_blind_0p3"] = float((centre_dist > 0.3).mean())
        # Crop box that contains every own return, with a 2 cm margin.
        margin = 0.02
        crop_lo, crop_hi = lo - margin, hi + margin
        result["crop_box_body_frame_m"] = {
            "min": crop_lo.round(3).tolist(), "max": crop_hi.round(3).tolist(),
            "size": (crop_hi - crop_lo).round(3).tolist()}
        result["vehicle_envelope_body_frame_m"] = {
            "min": [-BASE_X_HALF, -BASE_Y_HALF, BASE_Z_MIN],
            "max": [BASE_X_HALF, BASE_Y_HALF, BASE_Z_MAX]}
        sdf = box_sdf(body_pts)
        result["own_hits_outside_envelope"] = int((sdf > 0.0).sum())
        # The envelope box and the front strip are visualisation only, not hardware:
        # echoes from them are an artefact of switching the exclusion off.
        real = np.array([n not in VISUAL_ONLY_GEOMS for n in names])
        if real.any():
            r_lo, r_hi = body_pts[real].min(axis=0), body_pts[real].max(axis=0)
            real_range = ranges[real]
            result["real_geom_hits"] = int(real.sum())
            result["real_geom_hits_beyond_blind_0p3"] = int(
                (np.linalg.norm(points[real] - lidar_frame_origin(data), axis=1) > 0.3).sum())
            result["real_geom_range_m"] = {"min": float(real_range.min()), "max": float(real_range.max())}
            result["real_geom_bbox_body_frame_m"] = {"min": r_lo.round(4).tolist(), "max": r_hi.round(4).tolist()}
            result["real_geom_crop_box_body_frame_m"] = {
                "min": (r_lo - margin).round(3).tolist(), "max": (r_hi + margin).round(3).tolist(),
                "size": (r_hi - r_lo + 2 * margin).round(3).tolist()}
    return result


def scene_xml(args):
    half_w = args.tunnel_width / 2.0
    wall_h = args.roof_bottom if args.roof_bottom > 0 else args.open_wall_height
    parts = [
        # floor top at z = 0
        '<geom name="tunnel_floor" type="box" pos="0 0 -0.05" size="9 3 0.05" contype="0" conaffinity="0"/>',
        f'<geom name="wall_left" type="box" pos="0 {half_w + 0.05} {wall_h / 2}" '
        f'size="9 0.05 {wall_h / 2}" contype="0" conaffinity="0"/>',
        f'<geom name="wall_right" type="box" pos="0 {-half_w - 0.05} {wall_h / 2}" '
        f'size="9 0.05 {wall_h / 2}" contype="0" conaffinity="0"/>',
    ]
    if args.roof_bottom > 0:
        parts.append(f'<geom name="tunnel_roof" type="box" pos="0 0 {args.roof_bottom + 0.025}" '
                     f'size="9 {half_w + 0.1} 0.025" contype="0" conaffinity="0"/>')
    return "\n    ".join(parts)


def wall_summary(z, wall_height):
    if len(z) == 0:
        return {"points": 0}
    bins = np.arange(0.0, wall_height + 0.01, 0.02)
    hist, _ = np.histogram(z, bins=bins)
    covered = hist > 0
    # Longest contiguous run of covered 2 cm bins.
    best = cur = 0
    for flag in covered:
        cur = cur + 1 if flag else 0
        best = max(best, cur)
    return {"points": int(len(z)), "z_min_m": float(z.min()), "z_max_m": float(z.max()),
            "covered_2cm_bins": int(covered.sum()), "total_2cm_bins": int(len(covered)),
            "coverage_fraction": float(covered.mean()), "longest_visible_run_m": float(best * 0.02)}


WINDOWS = {
    # |dx| is measured from the sensor along the tunnel axis (+x ahead).
    "abeam_pm0.15": (-0.15, 0.15),
    "ahead_0.3_to_1.0": (0.3, 1.0),
    "behind_0.3_to_0.5": (-0.5, -0.3),
}


def summarise(points, names, body_pts, sensor, keep, wall_height):
    entry = {}
    for label, wall in (("left_wall", "wall_left"), ("right_wall", "wall_right")):
        per_window = {}
        for window, (lo, hi) in WINDOWS.items():
            dx = points[:, 0] - sensor[0]
            sel = keep & (names == wall) & (dx >= lo) & (dx <= hi)
            per_window[window] = wall_summary(points[sel][:, 2], wall_height)
        per_window["whole_length"] = wall_summary(points[keep & (names == wall)][:, 2], wall_height)
        entry[label] = per_window
    floor = keep & (names == "tunnel_floor")
    ahead = points[floor & (body_pts[:, 0] > 0.0)]
    entry["ground_ahead"] = ({
        "points": int(len(ahead)),
        "nearest_x_from_sensor_m": float(ahead[:, 0].min() - sensor[0]),
        "nearest_x_beyond_front_face_m": float(ahead[:, 0].min() - BASE_X_HALF),
        "farthest_x_m": float(ahead[:, 0].max())} if len(ahead) else {"points": 0})
    entry["ground_under_vehicle_footprint_points"] = int(
        (floor & (np.abs(body_pts[:, 0]) < BASE_X_HALF) & (np.abs(body_pts[:, 1]) < BASE_Y_HALF)).sum())
    entry["roof_points"] = int((keep & (names == "tunnel_roof")).sum())
    entry["points_kept"] = int(keep.sum())
    return entry


def probe_tunnel(args):
    pose, model, data = setup(args, scene_xml(args))
    base = np.array(data.xpos[model.body("base_link").id])
    sensor = lidar_frame_origin(data)
    wall_height = args.roof_bottom if args.roof_bottom > 0 else args.open_wall_height
    result = {
        "mount": {"tilt_direction": args.tilt_direction, "tilt_deg": args.tilt_deg,
                  "position_in_base_link": list(map(float, pose.position)),
                  "sensor_to_left_wall_m": float(args.tunnel_width / 2 - sensor[1]),
                  "sensor_to_right_wall_m": float(args.tunnel_width / 2 + sensor[1])},
        "tunnel": {"width_m": args.tunnel_width, "wall_height_m": wall_height,
                   "roofed": args.roof_bottom > 0},
        "full_cycle_sensor_seconds": 800000 / 24000 * 0.1,
    }
    # "sim_as_is": the simulator excludes the chassis, rays pass through it.
    # "physical": the chassis blocks rays like the real vehicle; own echoes are
    # dropped afterwards (they are the crop's job, counted separately).
    for model_name, exclude in (("sim_as_is", True), ("physical_occlusion", False)):
        points, names, ranges, _, total = trace(
            model, data, exclude_self=exclude, max_range=40.0, hide_visual=not exclude)
        own = np.array([n not in ("wall_left", "wall_right", "tunnel_floor", "tunnel_roof")
                        for n in names], dtype=bool)
        if not exclude:
            result.setdefault(model_name + "_own_hits", int(own.sum()))
        points, names = points[~own], names[~own]
        body_pts = points - base
        crops = {
            "no_crop": np.ones(len(points), dtype=bool),
            "sphere_blind_0p3": np.linalg.norm(points - sensor, axis=1) >= 0.3,
            "box_plus_0p1_ball": box_sdf(body_pts) > 0.1,
        }
        result[model_name] = {
            "rays": int(total), "environment_hits": int(len(points)),
            **{crop: summarise(points, names, body_pts, sensor, keep, wall_height)
               for crop, keep in crops.items()}}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["self", "tunnel"])
    parser.add_argument("--tilt-direction", choices=["front", "left"], default="front")
    parser.add_argument("--tilt-deg", type=float, default=35.0)
    parser.add_argument("--mount-x", type=float, default=0.2405)
    parser.add_argument("--mount-y", type=float, default=0.0)
    parser.add_argument("--mount-height", type=float, default=0.20)
    parser.add_argument("--tunnel-width", type=float, default=0.80)
    parser.add_argument("--roof-bottom", type=float, default=0.25,
                        help="roof height above the floor, m (tunnels_2027.yaml: 0.25); 0 = open top")
    parser.add_argument("--open-wall-height", type=float, default=1.5)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    result = probe_self(args) if args.mode == "self" else probe_tunnel(args)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(text, "utf-8")
    print(text)


if __name__ == "__main__":
    main()
