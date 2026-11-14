"""Read-only column inputs and separate truth/reference diagnostics; no navigation writes."""
import json
from pathlib import Path
import time

import numpy as np


class ColumnInputRecorder:
    def __init__(self, node, directory):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from tf2_msgs.msg import TFMessage
        from ats_navigation_interfaces.msg import LocalizationStatus, ExecutionCommand
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict

        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.frames = []
        self.messages = {key: [] for key in ('localization', 'localization_status', 'tf', 'tf_static')}
        self.subscriptions = []
        self.closed = False
        self.diagnostic_streams = {
            key: (self.directory / f'{key}.jsonl').open('x')
            for key in ('truth_pose', 'execution_references')}
        self.reference_keys = set()
        durable = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for kind, topic, key, qos in (
                (Odometry, '/localization', 'localization', qos_profile_sensor_data),
                (LocalizationStatus, '/localization/status', 'localization_status', durable),
                (TFMessage, '/tf', 'tf', 100), (TFMessage, '/tf_static', 'tf_static', durable)):
            def record(message, key=key):
                values = message.transforms if key.startswith('tf') else [message]
                self.messages[key].extend(message_to_ordereddict(v) for v in values)
            self.subscriptions.append(node.create_subscription(kind, topic, record, qos))
        # Raw replay inputs need history during bounded evaluation/serialization
        # pauses (d17 lost three committed scans with sensor-data depth 5).
        # This is only an evidence subscriber, never a navigation state input.
        raw_scan_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions.append(node.create_subscription(
            PointCloud2, '/registered_scan', self.on_cloud, raw_scan_qos))

        def truth(message):
            self.write_diagnostic('truth_pose', message_to_ordereddict(message))

        def execution(message):
            path = message.reference
            if not path.poses:
                return
            key = (message.goal_id, path.header.stamp.sec, path.header.stamp.nanosec)
            if key in self.reference_keys:
                return
            self.reference_keys.add(key)
            # Keep the entire first command carrying this reference, including
            # authorization sequence and each PoseStamped time. Pose coordinates
            # alone cannot reconstruct a terminal yaw reversal's physical timing.
            self.write_diagnostic('execution_references', message_to_ordereddict(message))

        self.subscriptions.append(node.create_subscription(
            Odometry, '/simulation/ground_truth/odometry', truth, qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(
            ExecutionCommand, '/planner/execution_command', execution, durable))

    def write_diagnostic(self, key, message):
        self.diagnostic_streams[key].write(json.dumps({
            'received_steady_ns': time.monotonic_ns(), 'message': message}) + '\n')

    def on_cloud(self, message):
        from sensor_msgs_py import point_cloud2
        points = np.array(list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)))
        xyz = (np.column_stack([points[key] for key in ('x', 'y', 'z')])
               if points.dtype.names else points.reshape(-1, 3))
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        self.write_scan(stamp_ns, message.header.frame_id, xyz)

    def write_scan(self, stamp_ns, frame, xyz):
        # Preserve every delivered frame, including duplicate/old epochs. The
        # replay exporter must reject or explicitly select them; never hide them.
        np.savez_compressed(self.directory / f'scan_{len(self.frames):04d}.npz', xyz=xyz)
        self.frames.append({'stamp': stamp_ns * 1e-9, 'stamp_ns': stamp_ns,
                            'frame': frame, 'points': len(xyz),
                            'received_steady_ns': time.monotonic_ns()})

    def close(self):
        if self.closed:
            return
        for key, rows in {'frames': self.frames, **self.messages}.items():
            (self.directory / f'{key}.json').write_text(json.dumps(rows) + '\n')
        (self.directory / 'scope.json').write_text(json.dumps({
            'scope': __doc__, 'frames_received': len(self.frames),
            'filtered_epochs': False, 'truth_used': False,
            'truth_recorded_for_evaluation_only': bool(getattr(self, 'diagnostic_streams', {})),
            'receipt_completeness': 'DDS-delivered frames only; source sequence is unavailable',
            'raw_scan_queue_depth': 200,
            'completeness_check': 'cross-check recorded stamps against production commits; losses still fail replay',
        }, indent=2)+'\n')
        for stream in getattr(self, 'diagnostic_streams', {}).values():
            stream.close()
        self.closed = True
