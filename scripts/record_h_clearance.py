#!/usr/bin/env python3
"""H parking evidence: record live registered scans without publishing velocity/goals.

Optional --park requests the existing X-lock mode through /motion_mode before
settling; it does not pin the simulated pose or change the scene/dynamics.

Coordinates only select the initial simulator pose. No audit/truth geometry is
fed to navigation. Raw clouds remain in their published odom frame.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np


def parking_acknowledged(future):
    if future is None or not future.done():
        return False
    response = future.result()
    if response is None or response.cmd_ack != 0:
        raise RuntimeError('parking mode request was rejected')
    return True


def check_parking_displacement(start, position, limit):
    """Evaluator-only XY drift gate; never published to navigation/control."""
    distance = math.hypot(position[0] - start[0], position[1] - start[1])
    if not math.isfinite(distance) or distance > limit:
        raise RuntimeError(f"parking displacement {distance:.6f} m exceeds {limit:.6f} m")
    return distance


def localization_abort_required(status, has_frontend, tracking_seen, fixed_body):
    return status.odom_identity_latched or (
        status.state == status.STATE_LOST and has_frontend and
        (not fixed_body or tracking_seen))


def settle_origin(previous, stamp, park_ready, tracking_seen, require_tracking):
    """Physical settling begins after parking and the first accepted tracking.

    Later non-TRACKING clouds are skipped separately; a postcommit hold must
    not restart physical settling. LOST/identity/motion still abort the run.
    """
    if not park_ready or (require_tracking and not tracking_seen):
        return None
    if previous is None or stamp < previous:
        return stamp
    return previous


def launch_command(start, output, roll=0.0, pitch=0.0, nav_start_delay_sec=60.0):
    """Start LIO before fusion; never extend production health leases."""
    return ["ros2", "launch", "ats_mujoco_sim", "rmuc_2027_mujoco.launch.py",
            "localization_source:=lio", "slam:=true", "use_identified_odometry:=true",
            "use_viewer:=false", "show_viewer:=false", "use_rviz:=false",
            "launch_mujoco_rviz:=false", "lidar_threads:=8", "lidar_downsample:=1",
            f"nav_start_delay_sec:={nav_start_delay_sec}",
            f"start_roll:={roll}", f"start_pitch:={pitch}",
            f"map_yaml_file:={output}/absent.yaml", f"prior_pcd_file:={output}/absent.pcd",
            *[f"{key}:={value}" for key, value in zip(
                ("start_x", "start_y", "start_z", "start_yaw"), start)]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", nargs=4, type=float, required=True,
                        metavar=("X", "Y", "Z", "YAW"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--park", action="store_true",
                        help="request existing MODE_PARK X-lock before settling; keep drift gate")
    parser.add_argument("--fixed-body", action="store_true",
                        help="static metrology only: generated world-weld fixture, not navigation evidence")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--settle-sim-s", type=float, default=15.0)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--max-parking-displacement-m", type=float, default=0.05,
                        help="abort on XY drift from spawn; truth is evaluator-only")
    parser.add_argument("--nav-start-delay-sec", type=float, default=60.0,
                        help="wall-clock delay before fusion/planning, allowing model and LIO startup")
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")
    if not math.isfinite(args.max_parking_displacement_m) or args.max_parking_displacement_m <= 0:
        parser.error("--max-parking-displacement-m must be finite and positive")
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 1 <= domain <= 232 or domain == 77:
        parser.error("explicit isolated ROS_DOMAIN_ID in [1,232], excluding 77, required")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "command.json").exists():
        parser.error("output already contains a run; choose a fresh directory")
    command = launch_command(args.start, output, args.roll, args.pitch, args.nav_start_delay_sec)
    if args.fixed_body:
        from h_stationary_fixture import make_stationary_fixture
        source = Path(__file__).resolve().parents[1] / 'src/sim/ats_mujoco_sim/models/rmuc_2027_swerve.xml'
        fixture = output / 'stationary_scene.xml'
        provenance = make_stationary_fixture(source, fixture, args.start, args.roll, args.pitch)
        (output / 'fixture-provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
        command.append(f'model_file:={fixture}')
    (output / "command.json").write_text(json.dumps({
        "command": command, "domain": domain, "scope": "stationary H sampling, not navigation acceptance",
        "range_noise_enabled": False, "physical_contact_acceptance": "unverified",
        "max_parking_displacement_m": args.max_parking_displacement_m,
        "park_requested": args.park,
        "fixed_body_metrology": args.fixed_body,
        "wait_for_first_tracking": args.fixed_body,
    }, indent=2) + "\n")
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from rclpy.qos import qos_profile_sensor_data
    from ats_navigation_interfaces.msg import LocalizationStatus, SwerveTelemetry
    from rosidl_runtime_py.convert import message_to_ordereddict
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException

    rclpy.init()
    node = rclpy.create_node("h_clearance_recorder")
    park_client = None
    if args.park:
        from manda_can_control.srv import MotionMode
        park_client = node.create_client(MotionMode, '/motion_mode')
    park_future = None
    park_ready = not args.park
    park_result = {'requested': args.park, 'acknowledged': False}
    frames = []
    truth = []
    telemetry = []
    localization = []
    statuses = []
    sensor_frames = []
    # A failed bootstrap must retain its input, not only the post-settle stream.
    # Bounded rings are diagnostic only and never count as stable sample frames.
    startup_scans = deque(maxlen=60)
    startup_sensor_scans = deque(maxlen=60)
    transforms = []
    static_transforms = []
    tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
    first_stamp = None
    tracking_seen = False
    startup_lost_samples = 0

    def cloud_xyz(message):
        points = np.array(list(point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)))
        if points.dtype.names:
            return np.column_stack([points[key] for key in ("x", "y", "z")])
        return points.reshape(-1, 3)

    def on_sensor_cloud(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        xyz = cloud_xyz(message)
        if first_stamp is None or stamp - first_stamp < args.settle_sim_s:
            startup_sensor_scans.append((stamp, message.header.frame_id, xyz))
            return
        if len(frames) >= args.frames:
            return
        np.savez_compressed(output / f"sensor_{len(sensor_frames):04d}.npz", xyz=xyz)
        sensor_frames.append({"stamp": stamp, "frame": message.header.frame_id, "points": len(xyz)})

    def on_cloud(message):
        nonlocal first_stamp
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        first_stamp = settle_origin(first_stamp, stamp, park_ready, tracking_seen, args.fixed_body)
        if not park_ready or (args.fixed_body and
                (not statuses or statuses[-1]['state'] != LocalizationStatus.STATE_TRACKING)):
            startup_scans.append((stamp, message.header.frame_id, cloud_xyz(message)))
            return
        if first_stamp is None:
            first_stamp = stamp
        if stamp - first_stamp < args.settle_sim_s:
            startup_scans.append((stamp, message.header.frame_id, cloud_xyz(message)))
            return
        if len(frames) >= args.frames:
            return
        if not localization or not statuses:
            return
        xyz = cloud_xyz(message)
        np.savez_compressed(output / f"scan_{len(frames):04d}.npz", xyz=xyz)
        frames.append({"stamp": stamp, "frame": message.header.frame_id, "points": len(xyz),
                       "latest_localization_index": len(localization) - 1,
                       "latest_status_index": len(statuses) - 1})
        print(f"recorded {len(frames)}/{args.frames} stamp={stamp:.3f}", flush=True)

    def on_truth(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        truth.append({"stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                      "position": [p.x, p.y, p.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w]})
        check_parking_displacement(args.start, truth[-1]["position"], args.max_parking_displacement_m)

    def on_telemetry(message):
        telemetry.append({"stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                          "contact_count": message.contact_violation_count,
                          "max_contact_force_n": message.max_contact_force,
                          "command": [message.command_vx, message.command_vy, message.command_wz]})
        if message.contact_violation_count:
            raise RuntimeError("body/obstacle contact diagnostic: abort parking sample")
        if max(abs(v) for v in telemetry[-1]["command"]) > 1e-6:
            raise RuntimeError("unexpected nonzero motion command during parking sample")

    def on_status(message):
        nonlocal tracking_seen, startup_lost_samples
        statuses.append(message_to_ordereddict(message))
        # Before the first frontend sample the production fusion reports LOST
        # while sensors/model load. This is initialization, not a transition
        # from healthy parking. Keep navigation's own inhibit untouched.
        if message.state == LocalizationStatus.STATE_LOST and not tracking_seen:
            startup_lost_samples += 1
        if localization_abort_required(message, bool(localization), tracking_seen, args.fixed_body):
            raise RuntimeError("localization lost or identity latched: abort parking sample")
        if message.state == LocalizationStatus.STATE_TRACKING:
            tracking_seen = True

    def on_tf(message, static=False):
        destination = static_transforms if static else transforms
        for transform in message.transforms:
            destination.append(message_to_ordereddict(transform))
            if static:
                tf_buffer.set_transform_static(transform, 'runtime_recorder')
            else:
                tf_buffer.set_transform(transform, 'runtime_recorder')

    def final_scan_tf_available():
        if len(frames) < args.frames:
            return False
        try:
            tf_buffer.lookup_transform('odom', 'front_mid360',
                                       Time(nanoseconds=round(frames[-1]['stamp'] * 1e9)))
            return True
        except TransformException:
            return False

    subscriptions = [
        # Preserve actual runtime extrinsics and odom transforms. Offline ray
        # replay must query scan time, not assume the source-default mount.
        node.create_subscription(TFMessage, "/tf", on_tf, 100),
        node.create_subscription(TFMessage, "/tf_static",
                                 lambda m: on_tf(m, static=True),
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(PointCloud2, "/mujoco/livox_points", on_sensor_cloud,
                                 qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/registered_scan", on_cloud, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/simulation/ground_truth/odometry", on_truth,
                                 qos_profile_sensor_data),
        node.create_subscription(SwerveTelemetry, "/swerve/telemetry", on_telemetry,
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, "/localization",
                                 lambda message: localization.append(message_to_ordereddict(message)),
                                 qos_profile_sensor_data),
        node.create_subscription(LocalizationStatus, "/localization/status", on_status, 10),
    ]
    process = None
    started = time.monotonic()
    try:
        with (output / "launch.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            while not final_scan_tf_available() and time.monotonic() - started < args.timeout:
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited {process.returncode}")
                if args.park and not park_ready:
                    if park_future is None and park_client.service_is_ready():
                        request = MotionMode.Request()
                        request.cmd_ctl = 16
                        park_future = park_client.call_async(request)
                    if parking_acknowledged(park_future):
                        park_ready = True
                        park_result.update(acknowledged=True,
                                           acknowledged_wall_s=time.monotonic() - started)
                        print('MODE_PARK acknowledged; starting settle window', flush=True)
                rclpy.spin_once(node, timeout_sec=0.2)
            if len(frames) < args.frames:
                raise RuntimeError(f"sampling timeout: {len(frames)}/{args.frames} frames")
            if not final_scan_tf_available():
                raise RuntimeError('sampling timeout: final scan-time TF unavailable')
    finally:
        park_result.update(tracking_seen=tracking_seen, startup_lost_samples=startup_lost_samples,
                           fixed_body_metrology=args.fixed_body)
        (output / 'parking_mode.json').write_text(json.dumps(park_result, indent=2) + '\n')
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=15)
        (output / "frames.json").write_text(json.dumps(frames, indent=2) + "\n")
        (output / "sensor_frames.json").write_text(json.dumps(sensor_frames, indent=2) + "\n")
        for prefix, ring in (("startup_scan", startup_scans),
                             ("startup_sensor", startup_sensor_scans)):
            metadata = []
            for index, (stamp, frame, xyz) in enumerate(ring):
                np.savez_compressed(output / f"{prefix}_{index:04d}.npz", xyz=xyz)
                metadata.append({"stamp": stamp, "frame": frame, "points": len(xyz)})
            (output / f"{prefix}_frames.json").write_text(json.dumps(metadata, indent=2) + "\n")
        (output / "truth.json").write_text(json.dumps(truth) + "\n")
        (output / "telemetry.json").write_text(json.dumps(telemetry) + "\n")
        (output / "localization.json").write_text(json.dumps(localization) + "\n")
        (output / "localization_status.json").write_text(json.dumps(statuses) + "\n")
        (output / "tf.json").write_text(json.dumps(transforms) + "\n")
        (output / "tf_static.json").write_text(json.dumps(static_transforms) + "\n")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
