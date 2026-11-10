#!/usr/bin/env python3
"""Strict scan-time runtime-TF origins for cold-start odom integration replay.

No truth, source-default extrinsic, latest-TF fallback or missing-frame skipping.
This does not replay epochs, fusion commits or history before the saved ring.
"""
import argparse
import hashlib
import json
from pathlib import Path


def input_stamps(path):
    stamps = []
    with path.open() as stream:
        if len(stream.readline().split()) != 13:
            raise ValueError('invalid column replay header')
        while True:
            line = stream.readline()
            if not line:
                break
            values = line.split()
            if len(values) != 5:
                raise ValueError('invalid frame header')
            stamp, count = float(values[0]), int(values[1])
            if count < 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError('unordered or invalid frame')
            for _ in range(count):
                if len(stream.readline().split()) != 3:
                    raise ValueError('truncated point stream')
            stamps.append(stamp)
    if not stamps:
        raise ValueError('no frames')
    return stamps


def sensor_origins(directory, stamps):
    from geometry_msgs.msg import TransformStamped
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rosidl_runtime_py.set_message import set_message_fields
    from tf2_ros import Buffer

    buffer = Buffer(cache_time=Duration(seconds=max(stamps) - min(stamps) + 120.0))
    for file, static in [('tf_static.json', True), ('tf.json', False)]:
        rows = json.loads((directory / file).read_text())
        for row in rows:
            transform = TransformStamped()
            set_message_fields(transform, row)
            if static:
                buffer.set_transform_static(transform, 'recorded_runtime')
            else:
                buffer.set_transform(transform, 'recorded_runtime')
    result = []
    for stamp in stamps:
        transform = buffer.lookup_transform('odom', 'front_mid360',
                                            Time(nanoseconds=round(stamp * 1e9)))
        point = transform.transform.translation
        result.append((stamp, point.x, point.y, point.z))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.json').exists():
        parser.error('choose a fresh output')
    stamps = input_stamps(args.input)
    rows = sensor_origins(args.directory, stamps)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        for row in rows:
            stream.write(' '.join(f'{value:.17g}' for value in row) + '\n')
    files = [args.input, args.directory / 'tf.json', args.directory / 'tf_static.json', args.output]
    metadata = {'scope': __doc__, 'frames': len(rows), 'hashes': {
        str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')


if __name__ == '__main__':
    main()
