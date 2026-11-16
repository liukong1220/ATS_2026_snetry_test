#!/usr/bin/env python3
"""Isolated empty-field sensor/actuator check; not full navigation acceptance."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from manda_can_control.msg import MotionCtrl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from ats_mujoco_sim.twist_to_motion_ctrl import TwistToMotionCtrl
from ats_mujoco_sim.model_patch import IMU_SITE_NAME, IMU_GYRO_SENSOR, IMU_ACCEL_SENSOR
from ats_mujoco_sim.lidar_mount import MID360_IMU_IN_LIDAR


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    domain = int(os.environ.get('ROS_DOMAIN_ID', '-1'))
    if not 0 <= domain <= 232 or domain in (5, 77, 91, 92):
        parser.error('choose a checked, isolated ROS_DOMAIN_ID')
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    # The old empty scene predates the LIO IMU. Add only the same production
    # sensor site/types; no estimated pose or synthetic localization is supplied.
    models = root / 'src/sim/ats_mujoco_sim/models'
    scene = ET.parse(models / 'swerve_chassis.xml')
    lidar_body = scene.find(".//body[@name='lidar_link']")
    ET.SubElement(lidar_body, 'site', name=IMU_SITE_NAME,
                  pos=' '.join(str(v) for v in MID360_IMU_IN_LIDAR))
    sensors = ET.SubElement(scene.getroot(), 'sensor')
    ET.SubElement(sensors, 'gyro', name=IMU_GYRO_SENSOR, site=IMU_SITE_NAME)
    ET.SubElement(sensors, 'accelerometer', name=IMU_ACCEL_SENSOR, site=IMU_SITE_NAME)
    for asset in scene.findall('./asset/*'):
        if 'file' in asset.attrib:
            asset.set('file', str(models / asset.get('file')))
    scene_path = (args.output / 'empty_lio_scene.xml').resolve()
    scene.write(scene_path, encoding='unicode')
    (args.output / 'meshes').symlink_to(models / 'meshes', target_is_directory=True)
    import mujoco
    checked_model = mujoco.MjModel.from_xml_path(str(scene_path))
    assert checked_model.sensor(IMU_GYRO_SENSOR).id >= 0
    command = [
        'ros2', 'launch', 'ats_mujoco_sim', 'ats_mujoco_sim.launch.py',
        f'model_path:={scene_path}',
        'localization_source:=lio', 'enable_lidar:=true', 'enable_tof:=false',
        'lidar_backend:=cpu', 'lidar_rate_hz:=10.0', 'lidar_rate_clock:=sim',
        'lidar_downsample:=0', 'lidar_threads:=0', 'lidar_mount_preset:=chassis',
        'sim_rate_hz:=400.0', 'start_z:=0.18', 'max_wheel_speed:=5.0',
        'use_viewer:=false', 'show_viewer:=false', 'launch_mujoco_rviz:=false',
    ]
    (args.output / 'command.json').write_text(json.dumps(command, ensure_ascii=False, indent=2))
    frames, imus, stats, motions, truth = [], [], [], [], []
    simulation_time = [None]
    rclpy.init()
    probe = rclpy.create_node('ats_sensor_speed_probe')
    bridge = TwistToMotionCtrl()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(bridge)
    publisher = probe.create_publisher(Twist, '/cmd_vel/selected', 10)
    subscriptions = [
        probe.create_subscription(PointCloud2, '/mujoco/livox_points',
            lambda msg: frames.append([stamp_seconds(msg.header.stamp), msg.width, msg.header.frame_id]),
            qos_profile_sensor_data),
        probe.create_subscription(Imu, '/livox/imu',
            lambda msg: imus.append(stamp_seconds(msg.header.stamp)), qos_profile_sensor_data),
        probe.create_subscription(Float64MultiArray, '/simulation/lidar_frame_stats',
            lambda msg: stats.append(list(msg.data)), 10),
        probe.create_subscription(MotionCtrl, '/motion_control',
            lambda msg: motions.append([msg.linear_x, msg.linear_y, msg.angular_z]), 10),
    ]

    def on_truth(msg):
        simulation_time[0] = stamp_seconds(msg.header.stamp)
        velocity = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        truth.append([simulation_time[0], math.hypot(velocity.x, velocity.y),
                      math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))])

    subscriptions.append(probe.create_subscription(
        Odometry, '/simulation/ground_truth/odometry', on_truth, qos_profile_sensor_data))
    log = (args.output / 'launch.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    started, origin, passed = time.monotonic(), None, False
    error = None
    try:
        while time.monotonic() - started < args.timeout:
            executor.spin_once(timeout_sec=0.02)
            if process.poll() is not None:
                raise RuntimeError(f'simulator exited {process.returncode}')
            if simulation_time[0] is None:
                continue
            if origin is None:
                origin = simulation_time[0]
            elapsed = simulation_time[0] - origin
            request = Twist()
            if 1.0 <= elapsed < 6.0:
                request.linear.x = 5.0
            elif 9.0 <= elapsed < 14.0:
                request.angular.z = 3.0
            publisher.publish(request)
            if elapsed >= 17.0 and len(frames) >= 30:
                passed = True
                break
        if not passed:
            raise TimeoutError('sensor/physics window did not complete')
    except Exception as exc:
        error = str(exc)
    finally:
        for _ in range(5):
            publisher.publish(Twist())
            executor.spin_once(timeout_sec=0.02)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        executor.shutdown()
        bridge.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
    frames_array = np.array([row[0] for row in frames])
    frame_periods = np.diff(frames_array)
    imu_periods = np.diff(imus)
    speed = max((math.hypot(row[0], row[1]) for row in motions), default=0.0)
    yaw_change = float(np.ptp(np.unwrap([row[2] for row in truth]))) if truth else 0.0
    checks = {
        'completed': passed and error is None,
        'sensor_frame': bool(frames) and all(row[2] == 'front_mid360' for row in frames),
        'lidar_10hz_sim_time': len(frame_periods) > 20 and bool(np.allclose(frame_periods, .1, atol=1e-6)),
        'imu_200hz_sim_time': len(imu_periods) > 100 and abs(float(np.median(imu_periods)) - .005) < 1e-6,
        'motion_control_accepts_5mps': speed > 4.99,
        'body_rotates': yaw_change > math.pi,
        'bounded_ray_budget': bool(stats) and all(0 < row[4] <= 20000 for row in stats),
    }
    summary = {
        'scope': 'isolated sensor/actuator diagnostic; no planner, no navigation acceptance',
        'passed': all(checks.values()), 'checks': checks, 'error': error, 'domain': domain,
        'wall_seconds': time.monotonic() - started, 'frames': len(frames), 'imu_samples': len(imus),
        'lidar_period_median_s': float(np.median(frame_periods)) if len(frame_periods) else None,
        'lidar_period_max_s': float(np.max(frame_periods)) if len(frame_periods) else None,
        'motion_control_max_planar_mps': speed,
        'truth_max_planar_mps': max((row[1] for row in truth), default=0.0),
        'truth_yaw_span_rad': yaw_change, 'lidar_stats_tail': stats[-5:],
        'physical_contact': 'not evaluated', 'rviz': 'not used',
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    (args.output / 'samples.json').write_text(json.dumps({'frames': frames, 'imus': imus, 'motions': motions, 'truth': truth}))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
