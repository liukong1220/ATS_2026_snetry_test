#!/usr/bin/env python3
"""Control the live MuJoCo LiDAR source without the ros2cli daemon graph cache."""
import argparse
import json
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LidarOcclusionControl(Node):
    OWNER = '/ats_mujoco_sim'
    PARAMETER = 'lidar_occlusion_enabled'
    TOPIC = '/registered_scan'

    def __init__(self):
        super().__init__('mujoco_lidar_occlusion_control')
        self.samples = []
        self.create_subscription(PointCloud2, self.TOPIC, self.on_scan, qos_profile_sensor_data)
        self.setter = self.create_client(SetParameters, self.OWNER + '/set_parameters')
        self.getter = self.create_client(GetParameters, self.OWNER + '/get_parameters')

    def on_scan(self, message):
        self.samples.append(dict(
            stamp=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            points=int(message.width) * int(message.height),
            valid=(bool(message.header.frame_id) and message.height > 0
                   and message.point_step > 0
                   and message.row_step == message.width * message.point_step
                   and len(message.data) == message.row_step * message.height)))

    def wait_for(self, predicate, deadline, label):
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError('timeout waiting for ' + label)

    def readback(self, deadline):
        request = GetParameters.Request()
        request.names = [self.PARAMETER]
        future = self.getter.call_async(request)
        self.wait_for(future.done, deadline, 'source parameter readback')
        response = future.result()
        if (response is None or len(response.values) != 1
                or response.values[0].type != ParameterType.PARAMETER_BOOL):
            raise RuntimeError('source parameter missing or not boolean')
        return response.values[0].bool_value

    def unique_source(self):
        publishers = self.get_publishers_info_by_topic(self.TOPIC)
        return (len(publishers) == 1
                and publishers[0].node_name == 'swerve_lidar_publisher'
                and publishers[0].node_namespace == '/')

    def fresh_effect(self, enabled, first_sample, stamp_floor):
        stamps = {sample['stamp'] for sample in self.samples[first_sample:]
                  if sample['valid'] and sample['stamp'] > stamp_floor
                  and (sample['points'] == 0) == enabled}
        return len(stamps) >= 2

    def run(self, enabled, timeout):
        # One shared deadline, not a retry or a longer fault injection window.
        deadline = time.monotonic() + timeout
        self.wait_for(lambda: self.setter.service_is_ready() and self.getter.service_is_ready()
                      and self.unique_source() and bool(self.samples), deadline,
                      'live simulator parameter services and unique LiDAR publisher')
        before = self.readback(deadline)
        if before == enabled:
            raise RuntimeError('source already has requested state; no injection transition proven')
        baseline = self.samples[-1]
        if not baseline['valid'] or (baseline['points'] == 0) != before:
            raise RuntimeError('source scan does not match pre-injection parameter state')
        first_sample = len(self.samples)
        stamp_floor = max(baseline['stamp'], self.get_clock().now().nanoseconds)
        request = SetParameters.Request()
        request.parameters = [Parameter(self.PARAMETER, value=enabled).to_parameter_msg()]
        future = self.setter.call_async(request)
        self.wait_for(future.done, deadline, 'source parameter update')
        response = future.result()
        if response is None or len(response.results) != 1 or not response.results[0].successful:
            reason = 'missing response' if response is None else str(response.results)
            raise RuntimeError('simulator rejected LiDAR occlusion: ' + reason)
        if self.readback(deadline) != enabled:
            raise RuntimeError('source parameter readback disagrees with requested state')
        self.wait_for(lambda: self.unique_source()
                      and self.fresh_effect(enabled, first_sample, stamp_floor), deadline,
                      'two fresh empty scans' if enabled else 'two fresh nonempty raycast scans')
        return dict(owner=self.OWNER, parameter=self.PARAMETER, enabled=enabled,
                    topic=self.TOPIC, baseline_stamp=baseline['stamp'],
                    effect_stamp_floor=stamp_floor,
                    effect_samples=self.samples[first_sample:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('value', choices=('true', 'false'))
    parser.add_argument('--timeout', type=float, default=6.0)
    args = parser.parse_args()
    rclpy.init()
    node = LidarOcclusionControl()
    try:
        print(json.dumps(node.run(args.value == 'true', args.timeout), sort_keys=True))
        return 0
    except RuntimeError as error:
        print(json.dumps(dict(passed=False, error=str(error)), sort_keys=True))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
