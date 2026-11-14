import json
from pathlib import Path

import numpy as np

from s3_column_input_recorder import ColumnInputRecorder


def test_duplicate_and_old_stamps_remain_explicit_replay_evidence(tmp_path):
    recorder = ColumnInputRecorder.__new__(ColumnInputRecorder)
    recorder.directory, recorder.frames = tmp_path, []
    recorder.messages = {'tf': [], 'tf_static': [], 'localization': [], 'localization_status': []}
    recorder.closed = False
    xyz = np.array([[.1, .2, .239], [.3, .4, .249]], dtype=np.float32)
    for ns in [2000000001, 2000000001, 1000000000]:
        recorder.write_scan(ns, 'odom', xyz)
    recorder.close()
    recorder.close()
    rows = json.loads((tmp_path/'frames.json').read_text())
    assert [r['stamp_ns'] for r in rows] == [2000000001, 2000000001, 1000000000]
    for i in range(3):
        np.testing.assert_array_equal(np.load(tmp_path/f'scan_{i:04d}.npz')['xyz'], xyz)
    assert json.loads((tmp_path/'scope.json').read_text())['truth_used'] is False


def test_diagnostics_keep_real_message_quaternions_and_each_reference_stamp(tmp_path):
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from ats_navigation_interfaces.msg import ExecutionCommand

    class Node:
        def __init__(self):
            self.callbacks = {}

        def create_subscription(self, kind, topic, callback, qos):
            self.callbacks[topic] = callback
            return callback

    node = Node()
    recorder = ColumnInputRecorder(node, tmp_path / 'inputs')
    truth = Odometry()
    truth.header.stamp.sec, truth.header.stamp.nanosec = 3, 123456789
    truth.pose.pose.orientation.x, truth.pose.pose.orientation.w = 0.3, 0.9
    node.callbacks['/simulation/ground_truth/odometry'](truth)
    command = ExecutionCommand()
    command.goal_id, command.command_sequence = 7, 42
    command.mode = command.MODE_EXECUTE
    command.reference.header.stamp.sec = 4
    for ns in (100000001, 200000002):
        pose = PoseStamped()
        pose.header.stamp.sec, pose.header.stamp.nanosec = 4, ns
        command.reference.poses.append(pose)
    node.callbacks['/planner/execution_command'](command)
    command.command_sequence += 1
    node.callbacks['/planner/execution_command'](command)
    recorder.close()
    rows = [json.loads(line) for line in
            (recorder.directory / 'execution_references.jsonl').read_text().splitlines()]
    assert len(rows) == 1
    assert rows[0]['message']['command_sequence'] == 42
    assert [p['header']['stamp']['nanosec'] for p in
            rows[0]['message']['reference']['poses']] == [100000001, 200000002]
    message = json.loads((recorder.directory / 'truth_pose.jsonl').read_text())['message']
    assert message['pose']['pose']['orientation']['x'] == 0.3
    assert message['header']['stamp']['nanosec'] == 123456789
