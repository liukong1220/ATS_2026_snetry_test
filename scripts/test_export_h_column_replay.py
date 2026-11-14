import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml
import pytest

from export_h_column_replay import committed_sources, filtered_points


def test_export_keeps_component_scope_and_refuses_overwrite(tmp_path):
    pose = {'header': {'stamp': {'sec': 1, 'nanosec': 0}, 'frame_id': 'odom'},
            'child_frame_id': 'gimbal_yaw_odom',
            'pose': {'pose': {'position': {'x': 0.1, 'y': 0.02, 'z': 0.2}}}}
    (tmp_path / 'localization.json').write_text(json.dumps([pose]))
    (tmp_path / 'frames.json').write_text(json.dumps([
        {'stamp': 1.0, 'frame': 'odom', 'points': 1}]))
    np.savez(tmp_path / 'scan_0000.npz', xyz=np.array([[0.1, 0.2, 0.0]]))
    values = dict(width=10, height=10, resolution=0.1, origin_x=-0.5, origin_y=-0.5,
                  surface_max=0.1, robot_height=0.255, top_margin=0.05, max_ground_slope=0.6)
    params = tmp_path / 'params.yaml'
    params.write_text(yaml.safe_dump({'ats_rog_map': {'ros__parameters': {
        f'odom_layer.{key}': value for key, value in values.items()}}}))
    output = tmp_path / 'replay.txt'
    script = Path(__file__).with_name('export_h_column_replay.py')
    command = [sys.executable, '-s', str(script), str(tmp_path), '--output', str(output),
               '--params', str(params), '--base-height', '.18', '--roi', '0', '0', '.2', '.2']
    subprocess.run(command, check=True, capture_output=True)
    metadata = json.loads(output.with_suffix('.json').read_text())
    assert metadata['config'] == values
    assert metadata['frames'] == 1
    assert 'NOT the complete odom-layer' in metadata['scope']
    frame = [float(x) for x in output.read_text().splitlines()[1].split()]
    assert np.allclose(frame, [1, 1, .1, .02, .02])
    before = output.read_bytes()
    rejected = subprocess.run(command, capture_output=True)
    assert rejected.returncode != 0
    assert b'fresh output' in rejected.stderr
    assert output.read_bytes() == before

    # A requested shorter interval is explicit, preserves the full record, and
    # is reflected in metadata. A missing-TF frame is never silently skipped.
    (tmp_path / 'frames.json').write_text(json.dumps([
        {'stamp': 1.0, 'frame': 'odom', 'points': 1},
        {'stamp': 2.0, 'frame': 'odom', 'points': 1}]))
    clipped = tmp_path / 'clipped.txt'
    limited = command.copy()
    limited[limited.index(str(output))] = str(clipped)
    subprocess.run([*limited, '--end-stamp', '1.0'], check=True, capture_output=True)
    metadata = json.loads(clipped.with_suffix('.json').read_text())
    assert metadata['frames'] == 1
    assert metadata['requested_end_stamp'] == 1.0
    assert output.read_bytes() == before


def test_self_filter_uses_xy_and_keeps_the_exact_boundary():
    points = np.array([[.449, 0, -20], [.45, 0, 0], [0, -.45, .2],
                       [.6, 0, float('nan')], [.8, 0, .1]])
    actual = filtered_points(points, [0, 0, 10], .45)
    assert np.array_equal(actual, points[[1, 2, 4]])
    with pytest.raises(ValueError, match='self_filter_radius'):
        filtered_points(points, [0, 0, 0], float('nan'))


def test_source_selection_requires_complete_identity_and_generation_evidence():
    def row(generation, stamp, epoch=1):
        return (f'projection end source_generation={generation} source_stamp_ns={stamp} '
                f'ready=1 odom_instance_id=123 odom_epoch={epoch}\n')
    stream = row(1, 0)+row(2, 100)+row(2, 100)+row(3, 200)
    assert committed_sources(stream) == ({2: 100, 3: 200}, (123, 1))
    for text, error in [(row(3, 200), 'missing startup'),
                        (row(2, 100)+row(4, 300), 'missing startup'),
                        (stream+row(3, 201), 'ambiguous'),
                        (stream+row(4, 300, 2), 'one nonzero'),
                        (stream+row(4, 150), 'nonmonotonic')]:
        with pytest.raises(ValueError, match=error):
            committed_sources(text)


def test_layer_commit_trace_recovers_only_independently_verified_generations():
    from export_h_column_replay import committed_layer_sources
    pair = lambda stamp: f'S3 layer_integrate prepare_ms=1\nS3 cloud_commit stamp_ns={stamp}\n'
    projection = ('projection end source_generation=3 source_stamp_ns=200 '
                  'ready=1 odom_instance_id=123 odom_epoch=1\n')
    final_projection = projection.replace('generation=3', 'generation=4').replace('stamp_ns=200', 'stamp_ns=300')
    stream = pair(100) + pair(200) + projection + pair(300) + final_projection
    assert committed_layer_sources(stream) == ({2: 100, 3: 200, 4: 300}, (123, 1))
    for text, error in [
            (pair(200) + projection, 'generation/stamp'),
            (stream + 'S3 layer_integrate prepare_ms=1\n', 'unpaired'),
            ('S3 cloud_commit stamp_ns=100\n' + stream, 'unpaired'),
            ('S3 layer_integrate prepare_ms=1\n' + stream, 'unpaired'),
            (stream + pair(250), 'nonmonotonic'),
            (stream + pair(400), 'unpaired'),
            (stream + final_projection.replace('odom_epoch=1', 'odom_epoch=2'), 'one nonzero'),
            (stream.replace('odom_epoch=1', 'odom_epoch=0'), 'one nonzero'),
            (pair(100), 'one nonzero'),
            (stream + projection.replace('stamp_ns=200', 'stamp_ns=201'), 'generation/stamp')]:
        with pytest.raises(ValueError, match=error):
            committed_layer_sources(text)


def test_export_runtime_base_tf_applies_actual_filter_and_explicit_start(tmp_path):
    pytest.importorskip('tf2_ros')
    def transform(stamp, x):
        return {'header': {'stamp': {'sec': stamp, 'nanosec': 0}, 'frame_id': 'odom'},
                'child_frame_id': 'gimbal_yaw_odom', 'transform': {
                    'translation': {'x': float(x), 'y': 0.0, 'z': 0.2},
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}
    (tmp_path / 'tf_static.json').write_text('[]')
    (tmp_path / 'tf.json').write_text(json.dumps([transform(1, 1), transform(3, 3)]))
    (tmp_path / 'frames.json').write_text(json.dumps([
        {'stamp': 0.0, 'frame': 'odom'}, {'stamp': 2.0, 'frame': 'odom'}]))
    # The unselected first scan has no TF and no payload; selecting an explicit
    # start is allowed, silently dropping it is not.
    np.savez(tmp_path / 'scan_0001.npz', xyz=np.array([[2.1, 0, .1], [2.5, 0, .1]]))
    params = tmp_path / 'params.yaml'
    params.write_text(yaml.safe_dump({'/ats_rog_map': {'ros__parameters': {
        'base_frame': 'gimbal_yaw_odom', 'self_filter_radius': .45,
        'odom_layer': dict(width=10, height=10, resolution=.1, origin_x=0, origin_y=0,
                           surface_max=.1, robot_height=.242, top_margin=.007,
                           max_ground_slope=.6, base_ground_offset=.18)}}}))
    output = tmp_path / 'input.txt'
    command = [sys.executable, '-s', str(Path(__file__).with_name('export_h_column_replay.py')),
               str(tmp_path), '--params', str(params), '--output', str(output),
               '--base-height', '.18', '--roi', '0', '0', '1', '1', '--runtime-base-tf']
    rejected = subprocess.run(command, capture_output=True)
    assert rejected.returncode != 0  # TF extrapolation, before writing output
    assert not output.exists()
    subprocess.run([*command, '--start-stamp', '1.0'], check=True, capture_output=True)
    lines = output.read_text().splitlines()
    assert np.allclose([float(x) for x in lines[1].split()], [2, 1, 2, 0, .02])
    assert np.allclose([float(x) for x in lines[2].split()], [2.5, 0, .1])
    metadata = json.loads(output.with_suffix('.json').read_text())
    assert metadata['base_source'] == 'scan-time runtime TF'
    assert metadata['requested_start_stamp'] == 1.0
    assert metadata['self_filter_radius'] == .45
    assert metadata['points_filtered'] == 1
