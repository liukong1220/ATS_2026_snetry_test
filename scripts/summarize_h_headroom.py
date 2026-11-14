#!/usr/bin/env python3
"""Summarise production ceiling headroom from h_ground_column_replay CSV output.

Per column over the last N frames: production headroom from ceilingHeadroom()
(including accepted floor/ceiling planes), its fluctuation and missing evidence,
the support interval width, and the interval-centre diagnostic. Evaluator-side
diagnostic only; scene truth is never an input and the gate is not changed here.
"""
import argparse
import csv
import json
import math
import statistics
import sys
from collections import defaultdict


def load(path):
    rows = defaultdict(dict)
    with open(path, newline='') as stream:
        for row in csv.DictReader(stream):
            key = (int(row['column_x']), int(row['column_y']))
            stamp = float(row['stamp'])
            if not math.isfinite(stamp):
                raise ValueError('non-finite stamp')
            if not row['headroom']:
                rows[key][stamp] = {'state': row['state'], 'headroom': None}
                continue
            lower, upper = float(row['support_lower']), float(row['support_upper'])
            low = float(row['ceiling_low'])
            headroom = float(row['headroom'])
            if not all(math.isfinite(v) for v in (lower, upper, low, headroom)):
                raise ValueError('non-finite clearance evidence')
            rows[key][stamp] = {
                'state': row['state'], 'headroom': headroom,
                'width': upper - lower, 'centre': low - 0.5 * (lower + upper)}
    return rows


def summarise(rows, window, gates):
    if window <= 0:
        raise ValueError('window must be positive')
    stamps = sorted({s for column in rows.values() for s in column})
    tail = stamps[-window:]
    columns = []
    for key, column in sorted(rows.items()):
        values = [column[s] for s in tail if s in column and column[s]['headroom'] is not None]
        states = [column[s]['state'] if s in column else 'missing' for s in tail]
        centre = [v['centre'] for v in values]
        headroom = [v['headroom'] for v in values]
        result = {
            'column': list(key), 'frames': len(values),
            'missing_headroom_frames': len(tail) - len(values),
            'state_transitions': sum(a != b for a, b in zip(states, states[1:])),
            'last_state': states[-1] if states else 'missing'}
        if values:
            result.update({
                'headroom_min': min(headroom),
                'headroom_median': statistics.median(headroom),
                'headroom_frame_ptp': max(headroom) - min(headroom),
                'width_median': statistics.median(v['width'] for v in values),
                'centre_min': min(centre), 'centre_median': statistics.median(centre),
                'centre_max': max(centre), 'centre_frame_ptp': max(centre) - min(centre),
            })
        columns.append(result)
    measured = [c for c in columns if c['frames']]
    complete = [c for c in measured if c['missing_headroom_frames'] == 0]
    centre_all = [c['centre_median'] for c in measured]
    result = {'columns': len(columns), 'window_frames': len(tail), 'per_column': columns}
    result['summary'] = {
        'measured_columns': len(measured), 'complete_columns': len(complete),
        'gate_passes_production': {
            f'{g:.3f}': sum(c['headroom_min'] >= g for c in complete) for g in gates},
        'gate_passes_centre': {
            f'{g:.3f}': sum(c['centre_min'] >= g for c in complete) for g in gates}}
    if measured:
        result['summary'].update({
            'headroom_production_median': statistics.median(
                c['headroom_median'] for c in measured),
            'headroom_frame_ptp_max': max(c['headroom_frame_ptp'] for c in measured),
            'support_width_median': statistics.median(c['width_median'] for c in measured),
            'centre_min': min(centre_all),
            'centre_median': statistics.median(centre_all),
            'centre_max': max(centre_all),
            'centre_frame_ptp_max': max(c['centre_frame_ptp'] for c in measured)})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--window', type=int, default=30)
    parser.add_argument('--gates', type=float, nargs='*',
                        default=[0.305, 0.255, 0.249, 0.247, 0.245, 0.240])
    args = parser.parse_args()
    json.dump(summarise(load(args.csv), args.window, args.gates), sys.stdout, indent=1)
    print()


if __name__ == '__main__':
    main()
