#!/usr/bin/env python3
"""Extract S3 global-layer range counters without inventing absent zero evidence.

Production logs are throttled every five ROS seconds. The latest counter is a
lower bound at the last logged integration, not an exact shutdown total. Logged
XYZ positions are samples of discarded observations, never a complete point set.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re


COUNTER = re.compile(r'(?:S3 global odom layer|odom layer: column_non_obstacle=\d+) out_of_range=(\d+)')
SAMPLE = re.compile(r'frame_count=(\d+) sample_odom=\(([^)]+)\)')


def summarize(text):
    records, segments = [], []
    previous = None
    for number, line in enumerate(text.splitlines(), 1):
        match = COUNTER.search(line)
        if match is None:
            continue
        count = int(match[1])
        if previous is not None and count < previous:
            segments.append(previous)
        record = {'line': number, 'cumulative_discarded_observations': count}
        sample = SAMPLE.search(line)
        if sample:
            position = [float(v) for v in sample[2].split(',')]
            if len(position) != 3:
                raise ValueError(f'invalid position on line {number}')
            record.update(frame_count=int(sample[1]), sample_odom_xyz=position)
        records.append(record)
        previous = count
    if previous is not None:
        segments.append(previous)
    return {'status': 'observed_counter' if records else 'unverified_missing_counter',
            'counter_segments': segments, 'counter_decreases': max(0, len(segments) - 1),
            'latest_logged_counter': previous,
            'sum_segment_last_counts_lower_bound': sum(segments) if records else None,
            'sampled_positions_odom': [r['sample_odom_xyz'] for r in records if 'sample_odom_xyz' in r],
            'records': records, 'scope': __doc__}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('launch_log', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = args.launch_log.read_bytes()
    result = summarize(data.decode())
    result.update(source=str(args.launch_log.resolve()), source_sha256=hashlib.sha256(data).hexdigest())
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('records', 'scope')}, indent=2))


if __name__ == '__main__':
    main()
