#!/usr/bin/env python3
"""Measure O0 reference handoffs from the read-only navigation chain trace.

Twists have no source stamp: their ROS receive time is the alignment coordinate.
Truth also retains its acquisition stamp. These are subscriber observations, not
an atomic MPC snapshot. No event is deleted for being terminal or safety related.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics


SOURCES = ('reference', 'autonomy_raw', 'selected', 'motion_control', 'truth')


def stamp_seconds(header):
    value = header['stamp']
    return value['sec'] + value['nanosec'] * 1e-9


def measure(records, mpc_dt, window=0.25, drop_limit=0.2, terminal_radius=1.0,
            odometry_timeout=None):
    if not math.isfinite(mpc_dt) or mpc_dt <= 0 or not math.isfinite(window) or window <= 0:
        raise ValueError('positive finite MPC dt and event window required')
    records = sorted(records, key=lambda r: r['ros_time_ns'])
    series = {source: [] for source in SOURCES}
    health = {key: [] for key in ('emergency_stop', 'map_ready', 'localization_status')}
    localizations, executions, rows = [], [], []
    odometry_receipts, control_receipts = [], []
    for record in records:
        kind, data = record['kind'], record['data']
        t = record['ros_time_ns'] * 1e-9
        if kind in health:
            health[kind].append((t, data['state'] if kind == 'localization_status' else data['data']))
        if kind == 'localization':
            localizations.append((t, data))
            if 'received_steady_s' in record:
                odometry_receipts.append(record['received_steady_s'])
        if kind == 'autonomy_raw' and 'received_steady_s' in record:
            control_receipts.append((t, record['received_steady_s']))
        if kind == 'execution':
            executions.append((t, data))
        sample_t, vx, vy, wz = t, None, None, None
        if kind in ('autonomy_raw', 'selected'):
            vx, vy, wz = data['linear']['x'], data['linear']['y'], data['angular']['z']
        elif kind == 'motion_control':
            vx, vy, wz = data['linear_x'], data['linear_y'], data['angular_z']
        elif kind == 'truth':
            v = data['twist']['twist']
            vx, vy, wz = v['linear']['x'], v['linear']['y'], v['angular']['z']
            sample_t = stamp_seconds(data['header'])
        elif kind == 'horizon' and len(data['poses']) >= 2:
            a, b = data['poses'][:2]
            vx, vy = (b[1] - a[1]) / mpc_dt, (b[2] - a[2]) / mpc_dt
            wz = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])) / mpc_dt
            kind = 'reference'
        if kind in series and vx is not None:
            if not all(math.isfinite(v) for v in (t, sample_t, vx, vy, wz)):
                raise ValueError('non-finite speed observation')
            speed = math.hypot(vx, vy)
            series[kind].append((t, speed))
            rows.append((t, sample_t, kind, speed, vx, vy, wz))

    def previous(samples, t):
        index = bisect.bisect_right([x[0] for x in samples], t) - 1
        return samples[index] if index >= 0 else None

    def window_metric(samples, t):
        before = [s for ts, s in samples if t - window <= ts < t]
        after = [s for ts, s in samples if t <= ts <= t + window]
        valid = len(before) >= 2 and len(after) >= 2
        return {'before_samples': len(before), 'after_samples': len(after),
                'before_median_mps': statistics.median(before) if before else None,
                'after_min_mps': min(after) if after else None,
                'drop_mps': max(0.0, statistics.median(before) - min(after)) if valid else None,
                'complete': valid}

    events, active = [], None
    odometry_receipts.sort()
    for t, command in executions:
        if command['mode'] != 1:
            events.append({'sim_s': t, 'category': 'stop', 'command': command})
            active = None
            continue
        if not active:
            events.append({'sim_s': t, 'category': 'initial_install', 'command': command})
            active = command
            continue
        identity = ('manager_incarnation', 'goal_id', 'localization_epoch')
        if any(active[key] != command[key] for key in identity):
            events.append({'sim_s': t, 'category': 'identity_change', 'command': command})
            active = command
            continue
        if active['reference']['stamp_ns'] == command['reference']['stamp_ns']:
            active = command
            continue
        metrics = {source: window_metric(series[source], t) for source in SOURCES}
        classifications = []
        if odometry_timeout is not None:
            ages = []
            for ts, received in control_receipts:
                if t - window <= ts <= t + window:
                    index = bisect.bisect_right(odometry_receipts, received) - 1
                    if index >= 0:
                        ages.append(received - odometry_receipts[index])
            if not ages:
                classifications.append('missing_odometry_receipt_age')
            elif max(ages) > odometry_timeout:
                classifications.append('odometry_receive_timeout')
        for key, expected in (('emergency_stop', False), ('map_ready', True),
                              ('localization_status', 1)):
            baseline = previous(health[key], t - window)
            values = ([baseline[1]] if baseline else []) + [
                v for ts, v in health[key] if t - window <= ts <= t + window]
            if not baseline:
                classifications.append('missing_' + key)
            elif any(v != expected for v in values):
                classifications.append('safety_transition')
        position = previous(localizations, t)
        remaining = None
        if position and t - position[0] <= window:
            pose = position[1]
            path = command['reference']
            if pose['header']['frame_id'] == path['frame'] and path['poses']:
                p = pose['pose']['pose']['position']
                endpoint = path['poses'][-1]
                remaining = math.hypot(p['x'] - endpoint[1], p['y'] - endpoint[2])
        if remaining is None:
            classifications.append('missing_terminal_distance')
        elif remaining <= terminal_radius:
            classifications.append('terminal_braking')
        if metrics['reference']['drop_mps'] is not None and metrics['reference']['drop_mps'] > 0.02:
            classifications.append('reference_deceleration')
        complete = all(m['complete'] for m in metrics.values())
        if not complete:
            classifications.append('incomplete_speed_window')
        eligible = not classifications
        events.append({'sim_s': t, 'category': 'handoff',
                       'classification': sorted(set(classifications)) or ['cruise'],
                       'eligible': eligible, 'terminal_distance_m': remaining,
                       'previous_reference_stamp_ns': active['reference']['stamp_ns'],
                       'command': command, 'metrics': metrics,
                       'observed_drop_limit_passed': (all(m['drop_mps'] < drop_limit for m in metrics.values())
                                                       if complete else None),
                       'passed': (all(m['drop_mps'] < drop_limit for m in metrics.values())
                                  if eligible else None)})
        active = command
    handoffs = [e for e in events if e['category'] == 'handoff']
    eligible = [e for e in handoffs if e['eligible']]
    stats = {}
    for source, samples in series.items():
        gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        stats[source] = {'samples': len(samples), 'peak_mps': max((s for _, s in samples), default=None),
                         'requested_or_observed_gt_2mps': any(s > 2.0 for _, s in samples),
                         'gt_2mps_sample_span_s': sum(
                             max(0.0, min(b[0] - a[0], 0.1))
                             for a, b in zip(samples, samples[1:]) if a[1] > 2.0),
                         'maximum_receive_gap_sim_s': max(gaps, default=None)}
    return {'window_sim_s': window, 'mpc_dt_s': mpc_dt, 'drop_limit_mps_exclusive': drop_limit,
            'odometry_receive_timeout_steady_s': odometry_timeout,
            'terminal_radius_m': terminal_radius, 'handoffs': len(handoffs),
            'eligible_cruise_handoffs': len(eligible),
            'all_classifications_drop_failures': sum(e['observed_drop_limit_passed'] is False for e in handoffs),
            'o0_passed': bool(eligible) and all(e['passed'] for e in eligible),
            'o0_status': ('measured' if eligible else 'not_triggered'),
            'speed': stats, 'events': events,
            'alignment': 'ROS subscriber receive time; truth acquisition time retained in CSV; '
                         'horizon first segment / measured MPC dt; subscriber observations not atomic'}, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mpc-dt', type=float, required=True)
    parser.add_argument('--terminal-radius', type=float, required=True)
    parser.add_argument('--odometry-timeout', type=float, required=True,
                        help='MPC runtime odometry_timeout in steady seconds; do not classify these stops as cruise')
    parser.add_argument('--window', type=float, default=0.25)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.input.read_text().splitlines() if line.strip()]
    if not math.isfinite(args.odometry_timeout) or args.odometry_timeout <= 0:
        parser.error('positive finite runtime odometry timeout required')
    result, rows = measure(records, args.mpc_dt, args.window, terminal_radius=args.terminal_radius,
                           odometry_timeout=args.odometry_timeout)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (args.output / 'speeds.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(('received_sim_s', 'sample_sim_s', 'source', 'speed_mps', 'vx', 'vy', 'wz'))
        writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for source in SOURCES:
        selected = [row for row in rows if row[2] == source]
        axes[0].plot([r[0] for r in selected], [r[3] for r in selected], label=source)
        axes[1].plot([r[0] for r in selected], [r[6] for r in selected], label=source)
    for event in result['events']:
        if event['category'] == 'handoff':
            for ax in axes:
                ax.axvline(event['sim_s'], color='grey', alpha=0.3)
    axes[0].set_ylabel('Planar speed [m/s]')
    axes[1].set_ylabel('Yaw rate [rad/s]')
    axes[1].set_xlabel('ROS receive time [simulation s]')
    axes[0].legend()
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.output / 'speeds.png', dpi=150)
    fig.savefig(args.output / 'speeds.svg')
    plt.close(fig)
    print(json.dumps({k: result[k] for k in ('handoffs', 'eligible_cruise_handoffs', 'o0_passed', 'o0_status', 'speed')}))


if __name__ == '__main__':
    main()
