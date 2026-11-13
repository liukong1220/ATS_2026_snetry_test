#!/usr/bin/env python3
"""Summarise production ceiling headroom from h_ground_column_replay CSV output.

Per column over the last N frames: the production headroom (lowest ceiling return
minus the UPPER support bound, the quantity state() compares with the gate), the
support interval width, and the interval-centre estimate. Evaluator-side
diagnostic only; scene truth is never an input and the gate is not changed here.
"""
import argparse
import csv
import json
import statistics
import sys
from collections import defaultdict


def load(path):
    rows = defaultdict(dict)
    with open(path, newline='') as stream:
        for row in csv.DictReader(stream):
            if not row['headroom']:
                continue
            key = (int(row['column_x']), int(row['column_y']))
            lower, upper = float(row['support_lower']), float(row['support_upper'])
            low = float(row['ceiling_low'])
            rows[key][float(row['stamp'])] = {
                'state': row['state'], 'headroom': float(row['headroom']),
                'width': upper - lower, 'centre': low - 0.5 * (lower + upper)}
    return rows


def summarise(rows, window, gates):
    stamps = sorted({s for column in rows.values() for s in column})
    tail = stamps[-window:]
    columns = []
    for key, column in sorted(rows.items()):
        values = [column[s] for s in tail if s in column]
        if len(values) < max(3, window // 6):
            continue
        centre = [v['centre'] for v in values]
        columns.append({
            'column': list(key), 'frames': len(values),
            'headroom_min': min(v['headroom'] for v in values),
            'headroom_median': statistics.median(v['headroom'] for v in values),
            'width_median': statistics.median(v['width'] for v in values),
            'centre_min': min(centre), 'centre_median': statistics.median(centre),
            'centre_max': max(centre), 'centre_frame_ptp': max(centre) - min(centre),
            'last_state': column[max(column)]['state']})
    centre_all = [c['centre_median'] for c in columns]
    result = {'columns': len(columns), 'window_frames': len(tail), 'per_column': columns}
    if columns:
        result['summary'] = {
            'headroom_conservative_median': statistics.median(
                c['headroom_median'] for c in columns),
            'support_width_median': statistics.median(c['width_median'] for c in columns),
            'centre_min': min(centre_all),
            'centre_median': statistics.median(centre_all),
            'centre_max': max(centre_all),
            'centre_frame_ptp_max': max(c['centre_frame_ptp'] for c in columns),
            'gate_passes_conservative': {
                f'{g:.3f}': sum(c['headroom_min'] >= g for c in columns) for g in gates},
            'gate_passes_centre': {
                f'{g:.3f}': sum(c['centre_min'] >= g for c in columns) for g in gates}}
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
