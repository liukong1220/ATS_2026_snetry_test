#!/usr/bin/env python3
"""Export recorded H scans for a cold-start, observe-only component replay.

This is NOT the complete odom-layer integration: no ray clearing, epoch gate,
fusion commit or prior map state. Saved startup rings may omit earlier scans.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import yaml

from analyze_h_clearance import localization_seed


def flatten_parameters(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten_parameters(value, name + '.'))
        else:
            result[name] = value
    return result


def filtered_points(xyz, base, radius):
    """Production XY self-filter; equality remains, no ground interpolation."""
    if not np.isfinite(radius) or radius < 0:
        raise ValueError('invalid production self_filter_radius')
    xyz = np.asarray(xyz)
    valid = np.isfinite(xyz).all(axis=1)
    distance = np.hypot(xyz[:, 0].astype(float) - base[0],
                        xyz[:, 1].astype(float) - base[1])
    return xyz[valid & (distance >= radius)]


def committed_sources(text):
    """Audit a complete startup prefix observed by the adapter, not all DDS scans.

    This is deliberately strict: missing generations, an epoch change or an
    ambiguous generation/stamp pair needs different evidence, not interpolation.
    """
    sources = {}
    identities = set()
    for generation, stamp, instance, epoch in re.findall(
            r'projection end[^\n]*source_generation=(\d+) source_stamp_ns=(\d+)'
            r'[^\n]*odom_instance_id=(\d+) odom_epoch=(\d+)', text):
        generation, stamp = int(generation), int(stamp)
        if stamp <= 0:
            continue
        identities.add((int(instance), int(epoch)))
        if generation in sources and sources[generation] != stamp:
            raise ValueError('ambiguous source generation/stamp')
        sources[generation] = stamp
    if not sources or len(identities) != 1 or any(v <= 0 for pair in identities for v in pair):
        raise ValueError('source replay requires one nonzero odom identity')
    ordered = sorted(sources)
    # A fresh ROG instance first commits its epoch reset as generation 1.
    if ordered != list(range(2, max(ordered)+1)):
        raise ValueError('missing startup/source generation; cannot claim complete column history')
    stamps = [sources[g] for g in ordered]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('nonmonotonic committed scan times')
    return sources, next(iter(identities))


def committed_layer_sources(text):
    """Audit unthrottled integration/commit pairs against projection identities.

    A normal mapping launch starts with reset generation 1. The epoch mutex
    serializes each successful layer integration and its following cloud trace.
    Failed updates, incomplete traces, restarts and missing pairs are rejected;
    no absent generation or scan is interpolated from publication timestamps.
    """
    stamps, pending = [], False
    for line in text.splitlines():
        if 'S3 layer_integrate ' in line:
            if pending:
                raise ValueError('unpaired layer integration trace')
            pending = True
        match = re.search(r'S3 cloud_commit stamp_ns=(\d+)', line)
        if match:
            if not pending:
                raise ValueError('unpaired cloud commit trace')
            stamp = int(match[1])
            if stamp <= 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError('nonmonotonic committed scan times')
            stamps.append(stamp)
            pending = False
    if pending:
        raise ValueError('unpaired trailing layer integration trace')
    rows = re.findall(
        r'projection end[^\n]*source_generation=(\d+) source_stamp_ns=(\d+)'
        r'[^\n]*odom_instance_id=(\d+) odom_epoch=(\d+)', text)
    rows = [(int(g), int(s), int(i), int(e)) for g, s, i, e in rows if int(s) > 0]
    identities = {(i, e) for _, _, i, e in rows}
    if not rows or len(identities) != 1 or any(v <= 0 for pair in identities for v in pair):
        raise ValueError('source replay requires one nonzero odom identity')
    sources = dict(enumerate(stamps, 2))
    for generation, stamp, _, _ in rows:
        if sources.get(generation) != stamp:
            raise ValueError('layer commit generation/stamp disagrees with projection')
    # Require a projection anchor after the final integration as well. Otherwise
    # a dropped pair after the last projection could hide an incomplete suffix.
    if max(g for g, _, _, _ in rows) != max(sources, default=0):
        raise ValueError('unpaired trailing commits without a projection anchor')
    return sources, next(iter(identities))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--base-height', type=float, required=True)
    parser.add_argument('--include-startup-ring', action='store_true')
    parser.add_argument('--roi', nargs=4, type=float, required=True)
    parser.add_argument('--end-stamp', type=float,
                        help='explicit diagnostic truncation; recorded in metadata, never implicit')
    parser.add_argument('--start-stamp', type=float,
                        help='explicit cold-start boundary; does not restore earlier map history')
    parser.add_argument('--runtime-base-tf', action='store_true',
                        help='use recorded scan-time base TF and actual self_filter_radius')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--committed-source-log', type=Path,
                           help='select the complete generation/stamp stream observed in adapter log')
    selection.add_argument('--committed-layer-log', type=Path,
                           help='audit all layer/commit trace pairs against adapter generation anchors')
    args = parser.parse_args()
    if not np.isfinite(args.base_height) or args.base_height < 0:
        parser.error('base height must be finite and nonnegative')
    source_log = args.committed_source_log or args.committed_layer_log
    if source_log and not args.runtime_base_tf:
        parser.error('committed source selection requires --runtime-base-tf')
    if args.output.exists() or args.output.with_suffix('.json').exists():
        parser.error('choose a fresh output path')
    document = yaml.safe_load(args.params.read_text())
    node = document.get('ats_rog_map', document.get('/ats_rog_map'))
    params = flatten_parameters(node['ros__parameters'])
    keys = ['width', 'height', 'resolution', 'origin_x', 'origin_y', 'surface_max',
            'robot_height', 'top_margin', 'max_ground_slope']
    config = {key: params[f'odom_layer.{key}'] for key in keys}
    localizations = ([] if args.runtime_base_tf else
                     json.loads((args.directory / 'localization.json').read_text()))
    frames = []
    for prefix, metadata in ([('startup_scan', 'startup_scan_frames')] if args.include_startup_ring else []) + [
            ('scan', 'frames')]:
        for i, frame in enumerate(json.loads((args.directory / f'{metadata}.json').read_text())):
            if frame['frame'] != 'odom':
                raise ValueError('component replay requires odom scans')
            frames.append((frame['stamp'], args.directory / f'{prefix}_{i:04d}.npz'))
    frames.sort(key=lambda frame: frame[0])
    source_audit = None
    if source_log:
        select = committed_layer_sources if args.committed_layer_log else committed_sources
        sources, identity = select(source_log.read_text())
        selected = {stamp for stamp in sources.values()
                    if (args.start_stamp is None or stamp*1e-9 >= args.start_stamp) and
                    (args.end_stamp is None or stamp*1e-9 <= args.end_stamp)}
        available = {round(stamp*1e9) for stamp, _ in frames}
        if selected - available:
            raise ValueError(f'missing {len(selected-available)} committed raw scans')
        frames = [frame for frame in frames if round(frame[0]*1e9) in selected]
        source_audit = {'odom_instance_id': identity[0], 'odom_epoch': identity[1],
                        'selection': select.__name__,
                        'observed_generations': len(sources), 'first_generation': min(sources),
                        'last_generation': max(sources), 'selected_committed_frames': len(selected),
                        'scope': ('layer/commit pairs cross-checked against projection generation anchors'
                                  if args.committed_layer_log else 'adapter-observed contiguous generations')
                                 + '; no fusion/epoch replay'}
    if args.start_stamp is not None:
        frames = [frame for frame in frames if frame[0] >= args.start_stamp]
    if args.end_stamp is not None:
        frames = [frame for frame in frames if frame[0] <= args.end_stamp]
    if not frames:
        raise ValueError('no recorded scans')
    if len({stamp for stamp, _ in frames}) != len(frames):
        raise ValueError('duplicate frame stamps')
    bases = None
    radius = None
    if args.runtime_base_tf:
        from export_h_sensor_origins import sensor_origins
        bases = sensor_origins(args.directory, [stamp for stamp, _ in frames],
                               params['base_frame'])
        radius = params['self_filter_radius']
        if args.base_height != params['odom_layer.base_ground_offset']:
            raise ValueError('base height must equal actual odom_layer.base_ground_offset')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    max_delta = 0.0
    filtered_count = 0
    with args.output.open('x') as stream:
        stream.write(' '.join(str(v) for v in [*[config[key] for key in keys], *args.roi]) + '\n')
        for i, (stamp, file) in enumerate(frames):
            if bases is None:
                seed, delta = localization_seed(localizations, stamp, args.base_height)
            else:
                seed = np.asarray(bases[i][1:]) - [0, 0, args.base_height]
                delta = 0.0
            max_delta = max(max_delta, abs(delta))
            xyz = np.load(file)['xyz']
            count = len(xyz)
            xyz = (filtered_points(xyz, bases[i][1:], radius) if bases is not None
                   else xyz[np.isfinite(xyz).all(axis=1)])
            filtered_count += count - len(xyz)
            stream.write(f'{stamp:.17g} {len(xyz)} ' + ' '.join(f'{v:.17g}' for v in seed) + '\n')
            np.savetxt(stream, xyz, fmt='%.17g')
    metadata = {'scope': __doc__, 'config_source': str(args.params.resolve()),
                'config_sha256': hashlib.sha256(args.params.read_bytes()).hexdigest(),
                'config': config, 'base_height': args.base_height, 'roi': args.roi,
                'first_stamp': frames[0][0], 'last_stamp': frames[-1][0], 'frames': len(frames),
                'max_abs_pose_delta_s': max_delta, 'requested_end_stamp': args.end_stamp,
                'requested_start_stamp': args.start_stamp,
                'base_source': 'scan-time runtime TF' if bases is not None else 'nearest localization',
                'self_filter_radius': radius, 'points_filtered': filtered_count,
                'committed_source_audit': source_audit,
                'input': str(args.output.resolve()),
                'input_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest()}
    recorded_files = [args.directory / 'frames.json', *(file for _, file in frames)]
    recorded_files += ([args.directory / 'tf.json', args.directory / 'tf_static.json']
                       if bases is not None else [args.directory / 'localization.json'])
    if source_log:
        recorded_files.append(source_log)
    metadata['recorded_input_hashes'] = {
        str(file.resolve()): hashlib.sha256(file.read_bytes()).hexdigest() for file in recorded_files}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
