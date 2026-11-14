#!/usr/bin/env python3
"""Evaluate saved map cells below audited overhang interiors, never navigation input.

Only complete map-cell footprints inside the roof and its clear corridor are
counted. Boundary/straddling cells are excluded explicitly. This supplements the
all-wall geometric comparison; it does not replace it or assert physical contact.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from s3_map_quality import load_map


def inside_convex(points, polygon, tolerance=1e-10):
    points, polygon = np.asarray(points), np.asarray(polygon, dtype=float)
    if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2:
        raise ValueError('expected convex XY polygon')
    area = np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1) -
                  polygon[:, 1] * np.roll(polygon[:, 0], -1))
    if abs(area) < tolerance:
        raise ValueError('zero-area polygon')
    result = np.ones(points.shape[:-1], dtype=bool)
    orientation = 1 if area > 0 else -1
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        cross = (b[0] - a[0]) * (points[..., 1] - a[1]) - (b[1] - a[1]) * (points[..., 0] - a[0])
        result &= orientation * cross >= -tolerance
    return result


def summarize(metadata, cells, centres, descriptions):
    cells, centres = np.asarray(cells), np.asarray(centres)
    r = float(metadata['resolution'])
    yaw = float(metadata['origin'][2])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    offsets = np.array([[-.5, -.5], [-.5, .5], [.5, -.5], [.5, .5]]) * r
    corners = centres[:, None, :] + offsets @ rotation.T
    result = {}
    for item in descriptions:
        centre_mask = inside_convex(centres, item['polygon'])
        complete = inside_convex(corners, item['polygon']).all(axis=1)
        if item.get('corridor_polygon'):
            centre_mask &= inside_convex(centres, item['corridor_polygon'])
            complete &= inside_convex(corners, item['corridor_polygon']).all(axis=1)
        selected = cells[complete]
        occupied = selected >= 50
        unknown = selected < 0
        free = (selected >= 0) & ~occupied
        result[item['name']] = {
            'full_interior_cells': len(selected),
            'boundary_cells_excluded': int((centre_mask & ~complete).sum()),
            'occupied_cells': int(occupied.sum()), 'free_cells': int(free.sum()),
            'unknown_cells': int(unknown.sum()),
            'occupied_centres_map': centres[complete][occupied].tolist(),
            'all_interiors_known_free': bool(len(selected) and free.all()),
            'scope': 'audited roof/corridor polygon interior; no inference for excluded boundary cells'}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prefix', type=Path, required=True, help='saved map prefix without suffix')
    p.add_argument('--overhang-yaml', type=Path, required=True, help='evaluation-only audited geometry')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('choose a fresh output')
    document = yaml.safe_load(args.overhang_yaml.read_text())
    if document.get('frame') != 'map':
        p.error('evaluation polygons must be in map')
    metadata, cells, centres = load_map(args.prefix)
    descriptions = document.get('tunnels', []) + document.get('overhangs', [])
    if not descriptions:
        p.error('no audited structures')
    result = {'navigation_input': False, 'physical_contact_acceptance': 'unverified',
              'structures': summarize(metadata, cells, centres, descriptions), 'inputs': {}}
    for path in [args.overhang_yaml, Path(str(args.prefix) + '.yaml'),
                 Path(str(args.prefix) + '.yaml').parent / metadata['image']]:
        result['inputs'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
