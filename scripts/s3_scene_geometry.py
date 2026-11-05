#!/usr/bin/env python3
"""Independent static wall geometry from MuJoCo's actual LiDAR scene mesh."""
from pathlib import Path

import numpy as np


def vertical_surface_samples(vertices, faces, spacing, maximum_normal_z=0.30):
    """Sample projected steep faces; horizontal floor/roofs and ramps are excluded.

    Every selected triangle contributes its longest horizontal segment. The
    normal cutoff describes wall geometry, not a robot traversability verdict.
    Overhead wall faces remain in the denominator and are reported explicitly.
    """
    vertices, faces = np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    selected = triangles[(lengths > 1e-12) & (np.abs(normals[:, 2]) <= maximum_normal_z * lengths)]
    result = []
    pairs = np.array([[0, 1], [0, 2], [1, 2]])
    for triangle in selected:
        segments = triangle[pairs, :2]
        distances = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1)
        i = int(np.argmax(distances))
        if distances[i] < 1e-9:
            continue
        count = max(2, int(np.ceil(distances[i] / spacing)) + 1)
        result.append(np.linspace(segments[i, 0], segments[i, 1], count))
    return np.concatenate(result) if result else np.empty((0, 2))


def scene_wall_samples(model_path, spacing):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    samples, sources = [], []
    box_vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    box_faces = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                          [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                          [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    for geom in range(model.ngeom):
        # Exclude the robot and every moving/free-joint fixture. Static geoms
        # welded to world are the independent, undeformed scene truth.
        if model.body_weldid[model.geom_bodyid[geom]] != 0:
            continue
        kind = model.geom_type[geom]
        if kind == mujoco.mjtGeom.mjGEOM_MESH and model.geom_group[geom] == 2:
            mesh = model.geom_dataid[geom]
            a, n = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            f, k = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            vertices = model.mesh_vert[a:a+n].copy()
            faces = model.mesh_face[f:f+k].copy()
        elif kind == mujoco.mjtGeom.mjGEOM_BOX and (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            vertices = box_vertices * model.geom_size[geom]
            faces = box_faces
        else:
            continue
        vertices = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        points = vertical_surface_samples(vertices, faces, spacing)
        if len(points):
            samples.append(points)
            sources.append({"geom": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom),
                            "mesh_faces": len(faces), "projected_samples": len(points)})
    if not samples:
        raise ValueError("scene has no independently supported static wall surfaces")
    points = np.unique(np.round(np.concatenate(samples), 4), axis=0)
    return points, {"model": str(Path(model_path).resolve()), "sources": sources,
                    "samples": len(points), "sample_spacing_m": spacing,
                    "maximum_normal_z": 0.30, "minimum_xy": points.min(axis=0).tolist(),
                    "maximum_xy": points.max(axis=0).tolist()}
