#!/usr/bin/env python3
"""Export a complete recorded terrain sequence for the production C++ filter.

The recorder normally deduplicates unchanged payloads. Reject an incomplete
sequence instead of treating content changes as consecutive sensor frames.
"""
import argparse
from decimal import Decimal
import gzip
import hashlib
import json
from pathlib import Path


def frames(directory):
    directory = Path(directory)
    summary = json.loads((directory / 'summary.json').read_text())
    topic = '/traversability_grid'
    rows = [r['payload'] for r in map(json.loads, (directory / 'events.jsonl').read_text().splitlines())
            if r['kind'] == 'layer_payload' and r['payload']['topic'] == topic]
    if summary['layer_payloads_truncated'] or len(rows) != summary['messages_received'].get(topic):
        raise ValueError('terrain sequence is deduplicated, truncated or incomplete; cannot replay frame counts')
    result = []
    for row in rows:
        with gzip.open(directory / row['path'], 'rt') as stream:
            payload = json.load(stream)
        info = payload['info']
        if len(payload['data']) != info['width'] * info['height']:
            raise ValueError('incomplete terrain grid')
        payload['stamp_ns'] = int(Decimal(payload['stamp_identity']) * 1000000000)
        result.append(payload)
    if not result:
        raise ValueError('no terrain frames')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    sequence = frames(args.evidence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        for row in sequence:
            info = row['info']
            stream.write(' '.join(map(str, [row['mono_s'], row['stamp_ns'], row['frame_id'],
                info['width'], info['height'], info['resolution'], info['origin_x'],
                info['origin_y'], info['origin_yaw']])) + '\n')
            stream.write(' '.join(map(str, row['data'])) + '\n')
    metadata = {'frames': len(sequence), 'first_stamp_ns': sequence[0]['stamp_ns'],
                'last_stamp_ns': sequence[-1]['stamp_ns'],
                'sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
                'scope': 'all received terrain frames; filter cold start at recording start',
                'source': str(args.evidence)}
    Path(str(args.output) + '.json').write_text(json.dumps(metadata, indent=2) + '\n')


if __name__ == '__main__':
    main()
