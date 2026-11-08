#!/usr/bin/env python3
"""Offline scene-cost diagnostic, not navigation or physical-contact acceptance."""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np


def measure(path, frames, rays, steps):
    start = time.perf_counter()
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    data = mujoco.MjData(model)
    load = time.perf_counter() - start
    base = model.body('base_link').id
    qpos = int(model.jnt_qposadr[model.body_jntadr[base]])
    data.qpos[qpos:qpos+3] = [3.37, 9.03, .465]
    mujoco.mj_forward(model, data)
    # Repeatable full-azimuth rays with elevations -7..52 degrees. This is a
    # single-thread microbenchmark, not the simulator's parallel Livox stream.
    index = np.arange(rays)
    yaw = index * np.pi * (3 - np.sqrt(5))
    pitch = np.deg2rad(-7 + 59 * (index + .5) / rays)
    vectors = np.c_[np.cos(yaw)*np.cos(pitch), np.sin(yaw)*np.cos(pitch), np.sin(pitch)]
    groups = np.zeros(6, dtype=np.uint8)
    groups[2] = 1
    ids = np.zeros(rays, dtype=np.int32)
    distances = np.zeros(rays)
    point = np.array([3.37, 9.03, .35])
    ray_times = []
    for _ in range(frames + 1):
        start = time.perf_counter()
        mujoco.mj_multiRay(model, data, point, vectors.ravel(), groups, 1,
                           base, ids, distances, rays, 30.)
        ray_times.append(time.perf_counter() - start)
    step_times = []
    max_contacts = 0
    for _ in range(steps):
        start = time.perf_counter()
        mujoco.mj_step(model, data)
        step_times.append(time.perf_counter() - start)
        max_contacts = max(max_contacts, data.ncon)
    return dict(model=str(path), load_s=load, geoms=model.ngeom, mesh_faces=int(model.mesh_facenum.sum()),
                rays=rays, frames=frames, ray_warmup_s=ray_times[0],
                ray_frame_s=ray_times[1:], ray_frame_p95_s=float(np.percentile(ray_times[1:], 95)),
                step_p95_ms=float(np.percentile(step_times, 95)*1000),
                step_max_ms=max(step_times)*1000, max_contact_count_diagnostic=max_contacts,
                physical_contact_acceptance='unverified', threads=1,
                scope='synthetic offline microbenchmark, not actual Livox or navigation timing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', action='append', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--rays', type=int, default=12000)
    parser.add_argument('--steps', type=int, default=1000)
    args = parser.parse_args()
    results = [measure(path, args.frames, args.rays, args.steps) for path in args.model]
    args.output.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
