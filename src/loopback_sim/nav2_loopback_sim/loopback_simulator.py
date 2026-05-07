
import math
from typing import Optional

from geometry_msgs.msg import (PoseWithCovarianceStamped, Quaternion, TransformStamped, Twist,
                               TwistStamped, Vector3)
from nav2_simple_commander.line_iterator import LineIterator
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.timer import Timer
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

# `tf_transformations` still pulls in transforms3d APIs removed in NumPy 2.x.
# Patch the missing symbols before importing it so the loopback simulator can
# run on newer Python environments without pinning an older NumPy version.
if not hasattr(np, 'float'):
    np.float = np.float64  # type: ignore[attr-defined]

if not hasattr(np, 'maximum_sctype'):
    def _maximum_sctype(dtype_like):
        dtype = np.dtype(dtype_like)
        if np.issubdtype(dtype, np.complexfloating):
            return np.complex128
        if np.issubdtype(dtype, np.floating):
            return np.float64
        if np.issubdtype(dtype, np.signedinteger):
            return np.int64
        if np.issubdtype(dtype, np.unsignedinteger):
            return np.uint64
        if np.issubdtype(dtype, np.bool_):
            return np.bool_
        return dtype.type

    np.maximum_sctype = _maximum_sctype  # type: ignore[attr-defined]

from .utils import (addYawToQuat, getMapOccupancy, matrixToTransform, transformStampedToMatrix,
                    worldToMap)
from .tf_compat import tf_transformations

"""
This is a loopback simulator that replaces a physics simulator to create a
frictionless, inertialess, and collisionless simulation environment. It
accepts cmd_vel messages and publishes odometry & TF messages based on the
cumulative velocities received to mimic global localization and simulation.
It also accepts initialpose messages to set the initial pose of the robot
to place anywhere.
"""


class LoopbackSimulator(Node):

    def __init__(self) -> None:
        super().__init__(node_name='loopback_simulator')
        self.curr_cmd_vel = None
        self.curr_cmd_vel_time = self.get_clock().now()
        self.initial_pose: PoseWithCovarianceStamped = None
        self.timer: Optional[Timer] = None
        self.setupTimer = None
        self.map = None
        self.mat_base_to_laser: Optional[np.ndarray[np.float64, np.dtype[np.float64]]] = None
        self.last_tf_stamp = None
        self.last_step_stamp = None
        self.tf_warmup_publish_count = 0

        self.declare_parameter('update_duration', 0.01)
        self.update_dur = self.get_parameter('update_duration').get_parameter_value().double_value

        self.declare_parameter('base_frame_id', 'base_footprint')
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # loopback 导航主底盘坐标默认使用 base_footprint，但上层仍有一部分节点、
        # 可视化与调试工具会继续查询 base_link / base_scan。
        # 因此这里额外维护一条同拍、同时间戳的辅助 TF 链：
        # base_footprint -> base_link -> base_scan
        # 这样能避免外部静态 TF 在 sim_time 下被晚收到或未连通时，
        # Nav2 / RViz 出现“TF 断树”“scan 时间早于缓存”的假故障。
        self.declare_parameter('body_frame_id', 'base_link')
        self.body_frame_id = self.get_parameter('body_frame_id').get_parameter_value().string_value

        # 实车链里不少节点会查询 gimbal_yaw_fake 作为“导航主底盘”坐标系。
        # loopback 没有真实云台链路时，也持续补一条 base_footprint -> gimbal_yaw_fake
        # 的辅助 TF，避免 recovery / behavior / RViz 因 frame 缺失直接失败。
        self.declare_parameter('auxiliary_frame_id', 'gimbal_yaw_fake')
        self.auxiliary_frame_id = self.get_parameter(
            'auxiliary_frame_id').get_parameter_value().string_value

        self.declare_parameter('map_frame_id', 'map')
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value

        self.declare_parameter('odom_frame_id', 'odom')
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value

        self.declare_parameter('scan_frame_id', 'base_scan')
        self.scan_frame_id = self.get_parameter('scan_frame_id').get_parameter_value().string_value

        self.declare_parameter('enable_stamped_cmd_vel', True)
        use_stamped = self.get_parameter('enable_stamped_cmd_vel').get_parameter_value().bool_value

        self.declare_parameter('scan_publish_dur', 0.1)
        self.scan_publish_dur = self.get_parameter(
            'scan_publish_dur').get_parameter_value().double_value

        self.declare_parameter('scan_tf_warmup_cycles', 3)
        self.scan_tf_warmup_cycles = self.get_parameter(
            'scan_tf_warmup_cycles').get_parameter_value().integer_value

        self.declare_parameter('publish_map_odom_tf', True)
        self.publish_map_odom_tf = self.get_parameter(
            'publish_map_odom_tf').get_parameter_value().bool_value

        self.declare_parameter('publish_clock', True)
        self.publish_clock = self.get_parameter('publish_clock').get_parameter_value().bool_value

        self.declare_parameter('scan_range_min',  0.05)
        self.scan_range_min = \
            self.get_parameter('scan_range_min').get_parameter_value().double_value

        self.declare_parameter('scan_range_max',  30.0)
        self.scan_range_max = \
            self.get_parameter('scan_range_max').get_parameter_value().double_value

        self.declare_parameter('scan_angle_min',  -math.pi)
        self.scan_angle_min = \
            self.get_parameter('scan_angle_min').get_parameter_value().double_value

        self.declare_parameter('scan_angle_max',  math.pi)
        self.scan_angle_max = \
            self.get_parameter('scan_angle_max').get_parameter_value().double_value

        self.declare_parameter('scan_angle_increment',  0.0261)  # 0.0261 rad = 1.5 degrees
        self.scan_angle_increment = \
            self.get_parameter('scan_angle_increment').get_parameter_value().double_value

        self.declare_parameter('scan_use_inf', True)
        self.use_inf = \
            self.get_parameter('scan_use_inf').get_parameter_value().bool_value

        self.t_map_to_odom = TransformStamped()
        self.t_map_to_odom.header.frame_id = self.map_frame_id
        self.t_map_to_odom.child_frame_id = self.odom_frame_id
        self.t_map_to_odom.transform.rotation.w = 1.0
        self.t_odom_to_base_link = TransformStamped()
        self.t_odom_to_base_link.header.frame_id = self.odom_frame_id
        self.t_odom_to_base_link.child_frame_id = self.base_frame_id
        self.t_odom_to_base_link.transform.rotation.w = 1.0

        self.t_base_to_body = None
        if self.body_frame_id != self.base_frame_id:
            self.t_base_to_body = TransformStamped()
            self.t_base_to_body.header.frame_id = self.base_frame_id
            self.t_base_to_body.child_frame_id = self.body_frame_id
            self.t_base_to_body.transform.rotation.w = 1.0

        self.t_base_to_aux = None
        if self.auxiliary_frame_id not in ('', self.base_frame_id, self.body_frame_id):
            self.t_base_to_aux = TransformStamped()
            self.t_base_to_aux.header.frame_id = self.base_frame_id
            self.t_base_to_aux.child_frame_id = self.auxiliary_frame_id
            self.t_base_to_aux.transform.rotation.w = 1.0

        self.scan_parent_frame_id = self.body_frame_id \
            if self.t_base_to_body is not None else self.base_frame_id
        self.t_body_to_scan = None
        if self.scan_frame_id != self.scan_parent_frame_id:
            self.t_body_to_scan = TransformStamped()
            self.t_body_to_scan.header.frame_id = self.scan_parent_frame_id
            self.t_body_to_scan.child_frame_id = self.scan_frame_id
            self.t_body_to_scan.transform.rotation.w = 1.0

        self.tf_broadcaster = TransformBroadcaster(self)

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose', self.initialPoseCallback, 10)
        if not use_stamped:
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                'cmd_vel', self.cmdVelCallback, 10)
        else:
            self.cmd_vel_sub = self.create_subscription(
                TwistStamped,
                'cmd_vel', self.cmdVelStampedCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', sensor_qos)

        if self.publish_clock:
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        self.setupTimer = self.create_timer(0.1, self.setupTimerCallback)

        self.map_client = self.create_client(GetMap, '/map_server/map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.getMap()

        self.info('Loopback simulator initialized')

    def getBaseToLaserTf(self) -> None:
        try:
            # Wait for transform and lookup
            transform = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.scan_frame_id, rclpy.time.Time())
            self.mat_base_to_laser = transformStampedToMatrix(transform)

        except Exception as ex:
            # Static TF publishers may not have latched yet during startup.
            # Keep retrying quietly until the transform tree is ready.
            self.debug(f'Waiting for startup transform: {str(ex)}')

    def setupTimerCallback(self) -> None:
        # 在 initialpose 之前也持续发布一套完整的动态 TF/odom，
        # 让 Nav2 / RViz / message filter 提前建立稳定缓存，避免
        # 首次收到 initialpose 或 scan 时出现“frame 不连通 / 时间早于缓存”的抖动。
        stamp = self.makeStepStamp()
        self.publishClock(stamp)
        self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link, stamp)
        self.publishOdometry(self.t_odom_to_base_link, stamp)
        if self.mat_base_to_laser is None:
            self.getBaseToLaserTf()

    def makeStepStamp(self):
        stamp = self.get_clock().now().to_msg()
        self.last_step_stamp = stamp
        return stamp

    def publishClock(self, stamp=None) -> None:
        msg = Clock()
        if stamp is None:
            stamp = self.makeStepStamp()
        msg.clock = stamp
        self.clock_pub.publish(msg)

    def cmdVelCallback(self, msg: Twist) -> None:
        self.debug('Received cmd_vel')
        if self.initial_pose is None:
            # Don't consider velocities before the initial pose is set
            return
        self.curr_cmd_vel = msg
        self.curr_cmd_vel_time = self.get_clock().now()

    def cmdVelStampedCallback(self, msg: TwistStamped) -> None:
        self.debug('Received cmd_vel')
        if self.initial_pose is None:
            # Don't consider velocities before the initial pose is set
            return
        self.curr_cmd_vel = msg.twist
        self.curr_cmd_vel_time = rclpy.time.Time.from_msg(msg.header.stamp)

    def initialPoseCallback(self, msg: PoseWithCovarianceStamped) -> None:
        self.info('Received initial pose!')
        if self.initial_pose is None:
            # Initialize transforms (map->odom as input pose, odom->base_link start from identity)
            self.initial_pose = msg.pose.pose
            self.t_map_to_odom.transform.translation.x = self.initial_pose.position.x
            self.t_map_to_odom.transform.translation.y = self.initial_pose.position.y
            self.t_map_to_odom.transform.translation.z = 0.0
            self.t_map_to_odom.transform.rotation = self.initial_pose.orientation
            self.t_odom_to_base_link.transform.translation = Vector3()
            self.t_odom_to_base_link.transform.rotation = Quaternion()
            self.t_odom_to_base_link.transform.rotation.w = 1.0
            stamp = self.makeStepStamp()
            self.publishClock(stamp)
            self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link, stamp)
            self.publishOdometry(self.t_odom_to_base_link, stamp)

            # Start republication timer and velocity processing
            if self.setupTimer is not None:
                self.setupTimer.cancel()
                self.setupTimer.destroy()
                self.setupTimer = None
            self.timer = self.create_timer(self.update_dur, self.timerCallback)
            self.timer_laser = self.create_timer(self.scan_publish_dur, self.publishLaserScan)
            return

        self.initial_pose = msg.pose.pose

        # Adjust map->odom transform based on new initial pose, keeping odom->base_link the same
        t_map_to_base_link = TransformStamped()
        t_map_to_base_link.header = msg.header
        t_map_to_base_link.child_frame_id = self.base_frame_id
        t_map_to_base_link.transform.translation.x = self.initial_pose.position.x
        t_map_to_base_link.transform.translation.y = self.initial_pose.position.y
        t_map_to_base_link.transform.translation.z = 0.0
        t_map_to_base_link.transform.rotation = self.initial_pose.orientation
        mat_map_to_base_link = transformStampedToMatrix(t_map_to_base_link)
        mat_odom_to_base_link = transformStampedToMatrix(self.t_odom_to_base_link)
        mat_base_link_to_odom = tf_transformations.inverse_matrix(mat_odom_to_base_link)
        mat_map_to_odom = \
            tf_transformations.concatenate_matrices(mat_map_to_base_link, mat_base_link_to_odom)
        self.t_map_to_odom.transform = matrixToTransform(mat_map_to_odom)
        stamp = self.makeStepStamp()
        self.publishClock(stamp)
        self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link, stamp)
        self.publishOdometry(self.t_odom_to_base_link, stamp)

    def timerCallback(self) -> None:
        # If no data, just republish existing transforms without change
        one_sec = Duration(seconds=1)
        now = self.get_clock().now()
        stamp = self.makeStepStamp()
        self.publishClock(stamp)
        if self.curr_cmd_vel is None or now - self.curr_cmd_vel_time > one_sec:
            self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link, stamp)
            # 静止时也持续重发 odom。
            # 否则上层只订阅 odom 的节点会把位姿当成“停更”，
            # 在 loopback 中放大出“视觉最近圆周点不再刷新 / 到点后不再重规划”的假象。
            self.publishOdometry(self.t_odom_to_base_link, stamp)
            self.curr_cmd_vel = None
            return

        # Update odom->base_link from cmd_vel
        dx = self.curr_cmd_vel.linear.x * self.update_dur
        dy = self.curr_cmd_vel.linear.y * self.update_dur
        dth = self.curr_cmd_vel.angular.z * self.update_dur
        q = [self.t_odom_to_base_link.transform.rotation.x,
             self.t_odom_to_base_link.transform.rotation.y,
             self.t_odom_to_base_link.transform.rotation.z,
             self.t_odom_to_base_link.transform.rotation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.t_odom_to_base_link.transform.translation.x += dx * math.cos(yaw) - dy * math.sin(yaw)
        self.t_odom_to_base_link.transform.translation.y += dx * math.sin(yaw) + dy * math.cos(yaw)
        self.t_odom_to_base_link.transform.rotation = \
            addYawToQuat(self.t_odom_to_base_link.transform.rotation, dth)

        self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link, stamp)
        self.publishOdometry(self.t_odom_to_base_link, stamp)

    def publishLaserScan(self, stamp=None) -> None:
        if self.tf_warmup_publish_count < self.scan_tf_warmup_cycles:
            self.debug(
                'Skipping scan publish until TF cache is warmed up: '
                f'{self.tf_warmup_publish_count}/{self.scan_tf_warmup_cycles}'
            )
            return
        # Publish a bogus laser scan for collision monitor
        self.scan_msg = LaserScan()
        if stamp is None:
            stamp = self.last_step_stamp if self.last_step_stamp is not None else self.makeStepStamp()
        # scan 这一拍若没有先发布过 TF，就先补一拍同时间戳 TF/odom，
        # 让 message filter 总能在缓存里找到不晚于 scan 的坐标树。
        if self.last_tf_stamp != stamp:
            self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link, stamp)
            self.publishOdometry(self.t_odom_to_base_link, stamp)
        self.scan_msg.header.stamp = stamp
        self.scan_msg.header.frame_id = self.scan_frame_id
        self.scan_msg.angle_min = self.scan_angle_min
        self.scan_msg.angle_max = self.scan_angle_max
        # 1.5 degrees
        self.scan_msg.angle_increment = self.scan_angle_increment
        self.scan_msg.time_increment = 0.0
        self.scan_msg.scan_time = 0.1
        self.scan_msg.range_min = self.scan_range_min
        self.scan_msg.range_max = self.scan_range_max
        num_samples = int(
            (self.scan_msg.angle_max - self.scan_msg.angle_min) /
            self.scan_msg.angle_increment)
        self.scan_msg.ranges = [0.0] * num_samples
        self.getLaserScan(num_samples)
        self.scan_pub.publish(self.scan_msg)

    def publishTransforms(self, map_to_odom: TransformStamped,
                          odom_to_base_link: TransformStamped, stamp=None) -> None:
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        map_to_odom.header.stamp = stamp
        odom_to_base_link.header.stamp = stamp
        if self.publish_map_odom_tf:
            self.tf_broadcaster.sendTransform(map_to_odom)
        self.tf_broadcaster.sendTransform(odom_to_base_link)
        if self.t_base_to_aux is not None:
            self.t_base_to_aux.header.stamp = stamp
            self.tf_broadcaster.sendTransform(self.t_base_to_aux)
        if self.t_base_to_body is not None:
            self.t_base_to_body.header.stamp = stamp
            self.tf_broadcaster.sendTransform(self.t_base_to_body)
        if self.t_body_to_scan is not None:
            self.t_body_to_scan.header.stamp = stamp
            self.tf_broadcaster.sendTransform(self.t_body_to_scan)
        self.last_tf_stamp = stamp
        self.tf_warmup_publish_count += 1

    def publishOdometry(self, odom_to_base_link: TransformStamped, stamp=None) -> None:
        odom = Odometry()
        if stamp is None:
            stamp = self.last_tf_stamp if self.last_tf_stamp is not None \
                else self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = odom_to_base_link.transform.translation.x
        odom.pose.pose.position.y = odom_to_base_link.transform.translation.y
        odom.pose.pose.orientation = odom_to_base_link.transform.rotation
        # 静止重发 odom 时，curr_cmd_vel 可能已经被清空。
        # 这里统一回填一个零速度，避免把 None 赋给 Twist 子消息导致仿真节点崩溃。
        if self.curr_cmd_vel is None:
            odom.twist.twist = Twist()
        else:
            odom.twist.twist = self.curr_cmd_vel
        self.odom_pub.publish(odom)

    def info(self, msg: str) -> None:
        self.get_logger().info(msg)
        return

    def debug(self, msg: str) -> None:
        self.get_logger().debug(msg)
        return

    def getMap(self) -> None:
        request = GetMap.Request()
        if self.map_client.wait_for_service(timeout_sec=5.0):
            # Request to get map
            future = self.map_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                self.map = result.map
                self.get_logger().info('Laser scan will be populated using map data')
            else:
                self.get_logger().warn(
                    'Failed to get map, '
                    'Laser scan will be populated using max range'
                )
        else:
            self.get_logger().warn(
                'Failed to get map, '
                'Laser scan will be populated using max range'
            )

    def getLaserPose(self) -> tuple[float, float, float]:
        mat_map_to_odom = transformStampedToMatrix(self.t_map_to_odom)
        mat_odom_to_base = transformStampedToMatrix(self.t_odom_to_base_link)

        mat_map_to_laser = tf_transformations.concatenate_matrices(
            mat_map_to_odom,
            mat_odom_to_base,
            self.mat_base_to_laser
        )
        transform = matrixToTransform(mat_map_to_laser)

        x = transform.translation.x
        y = transform.translation.y
        theta = tf_transformations.euler_from_quaternion([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ])[2]

        return x, y, theta

    def getLaserScan(self, num_samples: int) -> None:
        if self.map is None or self.initial_pose is None or self.mat_base_to_laser is None:
            if self.use_inf:
                self.scan_msg.ranges = [float('inf')] * num_samples
            else:
                self.scan_msg.ranges = [self.scan_msg.range_max - 0.1] * num_samples
            return

        x0, y0, theta = self.getLaserPose()

        mx0, my0 = worldToMap(x0, y0, self.map)

        if not 0 < mx0 < self.map.info.width or not 0 < my0 < self.map.info.height:
            # outside map
            if self.use_inf:
                self.scan_msg.ranges = [float('inf')] * num_samples
            else:
                self.scan_msg.ranges = [self.scan_msg.range_max - 0.1] * num_samples
            return

        for i in range(num_samples):
            curr_angle = theta + self.scan_msg.angle_min + i * self.scan_msg.angle_increment
            x1 = x0 + self.scan_msg.range_max * math.cos(curr_angle)
            y1 = y0 + self.scan_msg.range_max * math.sin(curr_angle)

            mx1, my1 = worldToMap(x1, y1, self.map)

            line_iterator = LineIterator(mx0, my0, mx1, my1, 0.5)

            while line_iterator.isValid():
                mx, my = int(line_iterator.getX()), int(line_iterator.getY())

                if not 0 < mx < self.map.info.width or not 0 < my < self.map.info.height:
                    # if outside map then check next ray
                    break

                point_cost = getMapOccupancy(mx, my, self.map)

                if point_cost >= 60:
                    self.scan_msg.ranges[i] = math.sqrt(
                        (int(line_iterator.getX()) - mx0) ** 2 +
                        (int(line_iterator.getY()) - my0) ** 2
                    ) * self.map.info.resolution
                    break

                line_iterator.advance()
            if self.scan_msg.ranges[i] == 0.0 and self.use_inf:
                self.scan_msg.ranges[i] = float('inf')


def main() -> None:
    rclpy.init()
    loopback_simulator = LoopbackSimulator()
    rclpy.spin(loopback_simulator)
    loopback_simulator.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
