#!/usr/bin/env python3
"""Summarize separate spin-chain streams; no atomic cross-topic causality claim."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path

from evaluate_spin_tunnel_route import motion_spin_summary


def segment_velocity(first, second, dt):
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('a positive measured/configured segment duration is required')
    dx, dy = second[1] - first[1], second[2] - first[2]
    delta = second[3] - first[3]
    return {'vx': dx / dt, 'vy': dy / dt,
            'wz': math.atan2(math.sin(delta), math.cos(delta)) / dt}


def summarize(path, parameters):
    config = parameters['nodes']['ats_swerve_mpc']['values']
    dt = float(config['dt'])
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('MPC dt must come from the actual runtime parameter capture')
    rows = collections.defaultdict(list)
    references = []
    goal = None
    incomplete = 0
    with path.open() as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                incomplete += 1
                continue
            data, kind = event['data'], event['kind']
            t = event['ros_time_ns'] * 1e-9
            if kind == 'execution':
                goal = (data['manager_incarnation'], data['goal_id'], data['localization_epoch'])
                if data['mode'] != 1:
                    continue
                poses = data['reference']['poses']
                segments = []
                for first, second in zip(poses, poses[1:]):
                    span = (second[0] - first[0]) * 1e-9
                    velocity = segment_velocity(first, second, span)
                    velocity['t'] = second[0] * 1e-9
                    segments.append(velocity)
                references.append({'goal_identity': goal, 'sequence': data['command_sequence'],
                                   'stamp_ns': data['reference']['stamp_ns'],
                                   'scope': 'whole submitted path; preemption can leave a suffix unexecuted',
                                   'motion': motion_spin_summary(segments)})
                continue
            if kind in ('autonomy_raw', 'selected'):
                value = {'vx': data['linear']['x'], 'vy': data['linear']['y'],
                         'wz': data['angular']['z']}
            elif kind == 'motion_control':
                value = {'vx': data['linear_x'], 'vy': data['linear_y'], 'wz': data['angular_z']}
            elif kind == 'telemetry':
                stamp = data['header']['stamp']
                t = stamp['sec'] + stamp['nanosec'] * 1e-9
                value = {'vx': data['measured_vx'], 'vy': data['measured_vy'],
                         'wz': data['measured_wz']}
                rows['simulator_command'].append({
                    't': t, 'leg': goal, 'vx': data['command_vx'],
                    'vy': data['command_vy'], 'wz': data['command_wz']})
            elif kind == 'horizon' and len(data['poses']) >= 2:
                # Header stamps repeat for every horizon pose. The actual MPC
                # model dt is the state-spacing clock, not DDS receive spacing.
                value = segment_velocity(data['poses'][0], data['poses'][1], dt)
                t = data['stamp_ns'] * 1e-9
            else:
                continue
            rows[kind].append({'t': t, 'leg': goal, **value})
    return {
        'scope': 'separate subscriber streams; all moving samples, not open-area acceptance; '
                 'horizon derivatives are state secants, not an internal control-vector trace',
        'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'incomplete_json_lines': incomplete,
        'runtime_limits': {k: config[k] for k in (
            'dt', 'max_wz', 'max_awz', 'max_wheel_speed', 'wheel_base_x', 'wheel_base_y')},
        'streams': {key: motion_spin_summary(values) for key, values in rows.items()},
        'missing_streams': sorted(set(('horizon', 'autonomy_raw', 'selected',
                                      'motion_control', 'telemetry')) - rows.keys()),
        'submitted_references': references,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.trace, json.loads(args.parameters.read_text()))
    result['parameters_sha256'] = hashlib.sha256(args.parameters.read_bytes()).hexdigest()
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
