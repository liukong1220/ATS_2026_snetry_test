#!/usr/bin/env python3
"""Evaluator-only physical fixtures for unmodified RMUC field geometry."""
import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np


def make_fixture_scene(source, target):
    """Copy a full scene to /tmp and append one initially hidden free box."""
    import mujoco

    source, target = Path(source).resolve(), Path(target).resolve()
    original = source.read_bytes()
    original_model = mujoco.MjModel.from_xml_path(str(source))
    # The production XML contains CLI comments with '--'. MuJoCo accepts
    # them; ElementTree requires comments to be removed before parsing.
    root = ET.fromstring(re.sub(r"<!--.*?-->", "", original.decode(), flags=re.S))
    # Appending a free body changes MuJoCo's automatic mean mass/inertia/size.
    # Keep the original solver normalization and rendering bounds so the test
    # fixture changes only the explicit box, including while it is hidden.
    statistic = root.find("statistic")
    if statistic is None:
        statistic = ET.SubElement(root, "statistic")
    for name in ("meanmass", "meaninertia", "meansize", "extent"):
        statistic.set(name, repr(float(getattr(original_model.stat, name))))
    statistic.set("center", " ".join(repr(float(v)) for v in original_model.stat.center))
    for element in root.iter():
        if "file" in element.attrib:
            element.set("file", str((source.parent / element.get("file")).resolve()))
    world = root.find("worldbody")
    if world is None or world.find("body[@name='s3_test_obstacle']") is not None:
        raise ValueError("full field needs a worldbody without an existing S3 fixture")
    body = ET.SubElement(world, "body", name="s3_test_obstacle", pos="2 0 -10")
    ET.SubElement(body, "freejoint", name="s3_test_obstacle_joint")
    ET.SubElement(body, "geom", name="s3_test_obstacle_geom", type="box",
                  size="0.3 0.3 0.3", mass="1", group="2", rgba="1 0.2 0.1 1")
    target.parent.mkdir(parents=True, exist_ok=True)
    # sim_node also checks this compatibility resource relative to the model.
    mesh_link = target.parent / "meshes/mid360.obj"
    mesh_link.parent.mkdir(exist_ok=True)
    if not mesh_link.exists():
        mesh_link.symlink_to((source.parent / "meshes/mid360.obj").resolve())
    ET.ElementTree(root).write(target, encoding="unicode")
    return {"source_model": str(source), "source_sha256": hashlib.sha256(original).hexdigest(),
            "fixture_model": str(target),
            "fixture_sha256": hashlib.sha256(target.read_bytes()).hexdigest()}


def fixture_position_for_pose(model_path, pose, distance=2.0, half_size=0.3):
    """Place the evaluator's box ahead of truth pose, on the real ground.

    Truth is used only to move an experimental physical object. It is never
    published as a localization, map, goal or control input to navigation.
    """
    import mujoco

    x = float(pose[0] + distance * math.cos(pose[2]))
    y = float(pose[1] + distance * math.sin(pose[2]))
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    heights = []
    for geom in range(model.ngeom):
        if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_HFIELD:
            ray = mujoco.mj_rayHfield(
                model, data, geom, np.array([x, y, 5.0]), np.array([0.0, 0.0, -1.0]))
            if ray >= 0:
                heights.append(5.0 - ray)
        elif (model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_PLANE and
              (model.geom_contype[geom] or model.geom_conaffinity[geom])):
            heights.append(float(data.geom_xpos[geom, 2]))
    if not heights or not all(math.isfinite(z) for z in heights):
        raise ValueError("no independently supported fixture ground at the requested site")
    return [x, y, max(heights) + half_size]


def fixture_site(world_position, map_from_odom, half_size=0.3):
    """Express a world-axis box in odom, including its 3-D bounding extent."""
    position = np.asarray(world_position, dtype=float)
    translation = np.array([map_from_odom["translation"][key] for key in ("x", "y", "z")])
    x, y, z, w = [map_from_odom["rotation"][key] for key in ("x", "y", "z", "w")]
    norm = x*x + y*y + z*z + w*w
    if (position.shape != (3,) or not np.isfinite(position).all() or
            not np.isfinite(translation).all() or not math.isfinite(norm) or
            abs(norm - 1.0) > 1e-3 or not math.isfinite(half_size) or half_size <= 0):
        raise ValueError("invalid physical fixture pose or map-to-odom transform")
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])
    return {"world_position": position.tolist(),
            "odom_position": (rotation.T @ (position - translation)).tolist(),
            "odom_half_extent": (np.abs(rotation.T) @ np.full(3, half_size)).tolist(),
            "half_size": half_size}
