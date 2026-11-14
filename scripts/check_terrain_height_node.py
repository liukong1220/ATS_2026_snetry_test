#!/usr/bin/env python3
"""Installed terrain node regression, synthetic scans only; no actuator commands.

Use a fresh isolated ROS_DOMAIN_ID with the workspace WLAN environment. Pass an
effective terrain parameter dump or the production parameter/profile files in
load order. The only override is use_sim_time=false for this node-only fixture.
This is not physical clearance or navigation acceptance.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np


def sample(grid, x, y):
    if abs(grid.info.origin.orientation.z) > 1e-8:
        raise ValueError('fixture expects an axis-aligned terrain grid')
    ix = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    iy = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
        raise ValueError('fixture cell outside the published grid')
    return int(grid.data[iy * grid.info.width + ix])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from ament_index_python.packages import get_package_prefix
    from nav_msgs.msg import Odometry, OccupancyGrid
    from rcl_interfaces.srv import GetParameters
    import rclpy
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    from std_msgs.msg import Header
    from rosidl_runtime_py.convert import message_to_ordereddict

    executable = Path(get_package_prefix('terrain_analysis_ext')) / 'lib/terrain_analysis_ext/terrainAnalysisExt'
    command = [str(executable), '--ros-args', '-r', '__node:=terrain_analysis_ext']
    for path in args.params:
        command += ['--params-file', str(path.resolve())]
    command += ['-p', 'use_sim_time:=false']
    (args.output / 'command.json').write_text(json.dumps(command, indent=2)+'\n')
    inputs = [executable, Path(__file__), *args.params]
    (args.output / 'hashes.json').write_text(json.dumps({
        str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}, indent=2)+'\n')
    rclpy.init()
    node = rclpy.create_node('h2_terrain_height_probe')
    received = []
    clouds = []
    subscriptions = [
        node.create_subscription(OccupancyGrid, '/traversability_grid', received.append, 10),
        node.create_subscription(PointCloud2, '/terrain_map_ext', clouds.append, 10),
    ]
    odom_pub = node.create_publisher(Odometry, '/lidar_odometry', 5)
    scan_pub = node.create_publisher(PointCloud2, '/registered_scan', 5)
    log = (args.output / 'node.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    summary = {'scope': __doc__, 'passed': False}
    try:
        service = node.create_client(GetParameters, '/terrain_analysis_ext/get_parameters')
        if not service.wait_for_service(timeout_sec=20):
            raise RuntimeError('terrain parameter service missing')
        names = ['vehicleHeight', 'traversabilityOverheadHeight', 'planarVoxelSize',
                 'traversabilityMinPointCount', 'localTerrainMapRadius', 'use_sim_time']
        future = service.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('effective parameter query failed')
        values = future.result().values
        effective = {name: message_to_ordereddict(value) for name, value in zip(names, values)}
        (args.output / 'effective-parameters.json').write_text(json.dumps(effective, indent=2)+'\n')
        if (abs(values[0].double_value - .23) > 1e-9 or
                abs(values[1].double_value - .249) > 1e-9 or values[-1].bool_value):
            raise RuntimeError('unexpected height gates or clock; refusing to substitute test values')
        deadline = time.monotonic()+10
        while time.monotonic() < deadline and (
                not scan_pub.get_subscription_count() or not odom_pub.get_subscription_count()):
            rclpy.spin_once(node, timeout_sec=.1)
        if not scan_pub.get_subscription_count() or not odom_pub.get_subscription_count():
            raise RuntimeError('terrain input subscriptions missing')
        cases = [(5.05+i, height) for i, height in enumerate([.23, .235, .239, .242, .249, .259])]
        points = [(x, y, 0., 1.) for x in np.arange(4.4, 10.7, .04)
                  for y in np.arange(-.7, .71, .04)]
        for center, height in cases:
            points.extend((x, y, height, 1.) for x in np.arange(center-.16, center+.161, .04)
                          for y in np.arange(-.16, .161, .04))
        points = np.asarray(points, dtype=np.float32)
        np.save(args.output / 'injected-points.npy', points)
        fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
        for _ in range(20):
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id='odom')
            odom.child_frame_id = 'gimbal_yaw_odom'
            odom.pose.pose.orientation.w = 1.
            odom.pose.pose.position.z = .18
            odom_pub.publish(odom)
            scan_pub.publish(create_cloud(Header(stamp=stamp, frame_id='odom'), fields, points))
            end = time.monotonic()+.10
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            if process.poll() is not None:
                raise RuntimeError('terrain node exited during injected scans')
        if len(received) < 5:
            raise RuntimeError('insufficient processed terrain frames')
        results = []
        for center, height in cases:
            costs = [sample(grid, center, 0.) for grid in received[-5:]]
            expected_blocked = height < .249
            passed = all(cost == 100 if expected_blocked else 0 <= cost < 100 for cost in costs)
            results.append({'height_m': height, 'x': center, 'last_five_costs': costs,
                            'expected_blocked': expected_blocked, 'passed': passed})
        summary.update(cases=results, terrain_frames=len(received), filtered_cloud_frames=len(clouds))
        for index, grid in enumerate(received[-5:]):
            (args.output / f'grid-{index}.json').write_text(json.dumps(message_to_ordereddict(grid))+'\n')
        summary['passed'] = all(case['passed'] for case in results)
        if not summary['passed']:
            raise RuntimeError('height contract failed at installed terrain node')
    except Exception as error:
        summary['error'] = str(error)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)
        summary['node_exit_code'] = process.returncode
        summary['domain'] = __import__('os').environ.get('ROS_DOMAIN_ID')
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
