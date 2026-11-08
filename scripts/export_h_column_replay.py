#!/usr/bin/env python3
"""Export recorded H scans for a cold-start, observe-only component replay.

This is NOT the complete odom-layer integration: no ray clearing, epoch gate,
fusion commit or prior map state. Saved startup rings may omit earlier scans.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from analyze_h_clearance import localization_seed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--base-height', type=float, required=True)
    parser.add_argument('--include-startup-ring', action='store_true')
    parser.add_argument('--roi', nargs=4, type=float, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.json').exists():
        parser.error('choose a fresh output path')
    params = yaml.safe_load(args.params.read_text())['ats_rog_map']['ros__parameters']
    keys = ['width', 'height', 'resolution', 'origin_x', 'origin_y', 'surface_max',
            'robot_height', 'top_margin', 'max_ground_slope']
    config = {key: params[f'odom_layer.{key}'] for key in keys}
    localizations = json.loads((args.directory / 'localization.json').read_text())
    frames = []
    for prefix, metadata in ([('startup_scan', 'startup_scan_frames')] if args.include_startup_ring else []) + [
            ('scan', 'frames')]:
        for i, frame in enumerate(json.loads((args.directory / f'{metadata}.json').read_text())):
            if frame['frame'] != 'odom':
                raise ValueError('component replay requires odom scans')
            frames.append((frame['stamp'], args.directory / f'{prefix}_{i:04d}.npz'))
    frames.sort(key=lambda frame: frame[0])
    if not frames:
        raise ValueError('no recorded scans')
    if len({stamp for stamp, _ in frames}) != len(frames):
        raise ValueError('duplicate frame stamps')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    max_delta = 0.0
    with args.output.open('x') as stream:
        stream.write(' '.join(str(v) for v in [*[config[key] for key in keys], *args.roi]) + '\n')
        for stamp, file in frames:
            seed, delta = localization_seed(localizations, stamp, args.base_height)
            max_delta = max(max_delta, abs(delta))
            xyz = np.load(file)['xyz']
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            stream.write(f'{stamp:.17g} {len(xyz)} ' + ' '.join(f'{v:.17g}' for v in seed) + '\n')
            np.savetxt(stream, xyz, fmt='%.17g')
    metadata = {'scope': __doc__, 'config_source': str(args.params.resolve()),
                'config_sha256': hashlib.sha256(args.params.read_bytes()).hexdigest(),
                'config': config, 'base_height': args.base_height, 'roi': args.roi,
                'first_stamp': frames[0][0], 'last_stamp': frames[-1][0], 'frames': len(frames),
                'max_abs_pose_delta_s': max_delta, 'input': str(args.output.resolve()),
                'input_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest()}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
