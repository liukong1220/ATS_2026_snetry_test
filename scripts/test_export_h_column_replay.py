import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml


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
