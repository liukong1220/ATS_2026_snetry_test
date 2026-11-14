import json

import pytest

from export_h_sensor_origins import input_stamps, sensor_origins


def test_input_stamps_rejects_truncation_and_reordering(tmp_path):
    path = tmp_path / 'input.txt'
    header = '10 10 0.1 -0.5 -0.5 0.1 0.255 0.05 0.6 -1 -1 1 1\n'
    path.write_text(header + '1 1 0 0 0\n0 0 0\n2 0 0 0 0\n')
    assert input_stamps(path) == [1.0, 2.0]
    path.write_text(header + '1 1 0 0 0\n')
    with pytest.raises(ValueError, match='truncated'):
        input_stamps(path)
    path.write_text(header + '2 0 0 0 0\n1 0 0 0 0\n')
    with pytest.raises(ValueError, match='unordered'):
        input_stamps(path)


def test_runtime_tf_origin_is_interpolated_at_scan_not_latest(tmp_path):
    pytest.importorskip('tf2_ros')
    def transform(stamp, parent, child, x):
        return {'header': {'stamp': {'sec': stamp, 'nanosec': 0}, 'frame_id': parent},
                'child_frame_id': child, 'transform': {
                    'translation': {'x': float(x), 'y': 0.0, 'z': 0.0},
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}
    (tmp_path / 'tf_static.json').write_text(json.dumps([
        transform(0, 'gimbal_yaw_odom', 'front_mid360', 0.2405)]))
    (tmp_path / 'tf.json').write_text(json.dumps([
        transform(1, 'odom', 'gimbal_yaw_odom', 1),
        transform(3, 'odom', 'gimbal_yaw_odom', 3)]))
    rows = sensor_origins(tmp_path, [2.0])
    assert rows[0][1:] == pytest.approx([2.2405, 0.0, 0.0])
    from tf2_ros import TransformException
    with pytest.raises(TransformException):
        sensor_origins(tmp_path, [4.0])
    with pytest.raises(ValueError, match='positive finite'):
        sensor_origins(tmp_path, [0.0])
