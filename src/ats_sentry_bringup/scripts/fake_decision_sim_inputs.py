#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from ats_rm_interfaces.msg import GameStatus, RfidStatus, RobotStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sp_msgs.msg import VisionTargetMsg
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_mode(mode: str) -> str:
    return mode.strip().lower()


class FakeDecisionSimInputs(Node):
    def __init__(self):
        super().__init__(
            "fake_decision_sim_inputs",
            start_parameter_services=True,
        )

        self.declare_parameter("publish_rate", 5.0)
        # use_sim_time 由 launch 统一注入。
        # 这里不要重复 declare，否则在 ROS 已经自动声明该参数时会直接启动失败。
        # loopback 只需要在启动时给一次 initialpose。
        # 若在导航过程中反复发布，会持续改写 map->odom，导致 RViz 里局部路径/rollout 看起来“乱飘”。
        self.declare_parameter("initial_pose_repeats", 1)
        self.declare_parameter("initial_pose_period", 0.5)

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.declare_parameter("publish_decision_mode", True)
        self.declare_parameter("decision_mode", "patrol")
        self.declare_parameter("decision_mode_topic", "decision/sim_mode")
        self.declare_parameter("mode_script", [])

        # loopback 现在默认也发布假裁判数据，
        # 这样资源模式、视觉接管和实机主线的行为语义更一致。
        self.declare_parameter("publish_referee_inputs", True)
        self.declare_parameter("game_progress", int(GameStatus.RUNNING))
        self.declare_parameter("stage_remain_time", 420)

        # 视觉融合测试参数：
        # 通过这一组参数就能在 loopback 环境里伪造“视觉接管导航”的条件，
        # 不需要真的把整套视觉程序一起跑起来。
        self.declare_parameter("publish_vision_target", False)
        self.declare_parameter("vision_topic", "vision/target")
        self.declare_parameter("vision_tracking", False)
        # nav_hold 代表视觉明确建议“导航接管已经成立”。
        # 当前行为树主线会要求 nav_hold=true 才允许进入视觉跟随。
        self.declare_parameter("vision_nav_hold", True)
        self.declare_parameter("vision_fire_permitted", False)
        self.declare_parameter(
            "vision_target_type", int(VisionTargetMsg.TARGET_TYPE_UNKNOWN)
        )
        self.declare_parameter("vision_target_id", 7)
        self.declare_parameter("vision_confidence", 1.0)
        self.declare_parameter("vision_target_distance", 3.0)
        self.declare_parameter("vision_target_yaw", 0.0)
        self.declare_parameter("vision_target_pitch", 0.0)
        self.declare_parameter("vision_target_position_gimbal_x", 1.0)
        self.declare_parameter("vision_target_position_gimbal_y", 0.0)
        self.declare_parameter("vision_target_position_gimbal_z", 0.0)
        self.declare_parameter("vision_target_position_map_x", 0.0)
        self.declare_parameter("vision_target_position_map_y", 0.0)
        self.declare_parameter("vision_target_position_map_z", 0.0)
        self.declare_parameter("vision_has_target_position_map", False)
        self.declare_parameter("vision_target_position_map_frame", "map")

        self.declare_parameter("robot_id", 7)
        self.declare_parameter("current_hp", 400)
        self.declare_parameter("maximum_hp", 400)
        self.declare_parameter("shooter_barrel_cooling_value", 60)
        self.declare_parameter("shooter_barrel_heat_limit", 400)
        self.declare_parameter("shooter_17mm_1_barrel_heat", 0)
        self.declare_parameter("projectile_allowance_17mm", 200)
        self.declare_parameter("remaining_gold_coin", 0)
        self.declare_parameter("armor_id", 0)
        self.declare_parameter("hp_deduction_reason", int(RobotStatus.ARMOR_HIT))
        self.declare_parameter("is_hp_deduced", False)

        self.declare_parameter("friendly_fortress_gain_point", False)
        self.declare_parameter("friendly_supply_zone_non_exchange", False)
        self.declare_parameter("friendly_supply_zone_exchange", False)
        self.declare_parameter("center_gain_point", False)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )
        self.decision_mode_topic = str(
            self.get_parameter("decision_mode_topic").value
        )
        self.decision_mode_pub = self.create_publisher(
            String, self.decision_mode_topic, 10
        )
        self.vision_topic = str(self.get_parameter("vision_topic").value)
        self.vision_target_pub = self.create_publisher(
            VisionTargetMsg, self.vision_topic, 10
        )
        self.vision_target_point_map_pub = self.create_publisher(
            PointStamped, "vision/target_point_map", 10
        )
        self.game_status_pub = self.create_publisher(
            GameStatus, "referee/game_status", 10
        )
        self.robot_status_pub = self.create_publisher(
            RobotStatus, "referee/robot_status", 10
        )
        self.rfid_status_pub = self.create_publisher(
            RfidStatus, "referee/rfid_status", 10
        )

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.initial_pose_repeats = int(
            self.get_parameter("initial_pose_repeats").value
        )
        self.start_time = self.get_clock().now()
        self.last_mode: Optional[str] = None
        self.mode_script_cache_key: Optional[Tuple[str, ...]] = None
        self.mode_script_cache: List[Tuple[float, str]] = []
        self.add_on_set_parameters_callback(self.on_parameters_set)

        self.create_timer(1.0 / publish_rate, self.publish_loop)
        self.initial_pose_timer = self.create_timer(
            float(self.get_parameter("initial_pose_period").value),
            self.publish_initial_pose,
        )

        self.get_logger().info(
            "Fake loopback inputs are active. Default mode=%s, publish_referee_inputs=%s."
            % (
                str(self.get_parameter("decision_mode").value),
                str(self.get_parameter("publish_referee_inputs").value),
            )
        )
        self.get_logger().info(
            "Runtime parameter service is ready on /fake_decision_sim_inputs."
        )

    def on_parameters_set(self, parameters):
        updates = []
        for parameter in parameters:
            if parameter.name == "decision_mode":
                mode = normalize_mode(str(parameter.value))
                if mode not in ("patrol", "anchor", "retreat", "safe"):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "decision_mode must be one of "
                            "patrol/anchor/retreat/safe"
                        ),
                    )

            if parameter.name == "mode_script":
                self.mode_script_cache_key = None
                self.mode_script_cache = []

            if parameter.name in (
                "current_hp",
                "projectile_allowance_17mm",
                "decision_mode",
                "publish_decision_mode",
                "publish_referee_inputs",
                "decision_mode_topic",
                "vision_topic",
                "publish_vision_target",
                "vision_tracking",
                "vision_nav_hold",
            ):
                updates.append(f"{parameter.name}={parameter.value}")

        if updates:
            self.get_logger().info(
                "Accepted parameter update: %s" % ", ".join(updates)
            )

        return SetParametersResult(successful=True)

    def parse_mode_script(self) -> List[Tuple[float, str]]:
        raw_script = tuple(str(item) for item in self.get_parameter("mode_script").value)
        if raw_script == self.mode_script_cache_key:
            return self.mode_script_cache

        parsed: List[Tuple[float, str]] = []
        for entry in raw_script:
            if ":" not in entry:
                self.get_logger().warn(
                    "Ignore invalid mode_script entry '%s', expected '<seconds>:<mode>'"
                    % entry
                )
                continue

            time_text, mode_text = entry.split(":", 1)
            try:
                trigger_time = float(time_text.strip())
            except ValueError:
                self.get_logger().warn(
                    "Ignore invalid mode_script time '%s' in entry '%s'"
                    % (time_text, entry)
                )
                continue

            mode = normalize_mode(mode_text)
            if mode not in ("patrol", "anchor", "retreat", "safe"):
                self.get_logger().warn(
                    "Ignore invalid mode_script mode '%s' in entry '%s'"
                    % (mode_text, entry)
                )
                continue

            parsed.append((max(0.0, trigger_time), mode))

        parsed.sort(key=lambda item: item[0])
        self.mode_script_cache_key = raw_script
        self.mode_script_cache = parsed
        return parsed

    def resolve_decision_mode(self) -> str:
        script = self.parse_mode_script()
        if script:
            elapsed = (
                self.get_clock().now().nanoseconds - self.start_time.nanoseconds
            ) / 1e9
            resolved_mode = script[0][1]
            for trigger_time, scripted_mode in script:
                if elapsed >= trigger_time:
                    resolved_mode = scripted_mode
                else:
                    break
            return resolved_mode

        configured_mode = normalize_mode(str(self.get_parameter("decision_mode").value))
        if configured_mode not in ("patrol", "anchor", "retreat", "safe"):
            self.get_logger().warn(
                "Unsupported decision_mode '%s', fallback to 'patrol'"
                % configured_mode
            )
            return "patrol"
        return configured_mode

    def ensure_decision_mode_publisher(self):
        topic = str(self.get_parameter("decision_mode_topic").value).strip()
        if topic and topic != self.decision_mode_topic:
            self.decision_mode_topic = topic
            self.decision_mode_pub = self.create_publisher(String, topic, 10)

    def ensure_vision_target_publisher(self):
        topic = str(self.get_parameter("vision_topic").value).strip()
        if topic and topic != self.vision_topic:
            self.vision_topic = topic
            self.vision_target_pub = self.create_publisher(VisionTargetMsg, topic, 10)

    def publish_initial_pose(self):
        if self.initial_pose_repeats <= 0:
            self.initial_pose_timer.cancel()
            return

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = float(self.get_parameter("initial_x").value)
        initial_pose.pose.pose.position.y = float(self.get_parameter("initial_y").value)
        qx, qy, qz, qw = yaw_to_quaternion(
            float(self.get_parameter("initial_yaw").value)
        )
        initial_pose.pose.pose.orientation.x = qx
        initial_pose.pose.pose.orientation.y = qy
        initial_pose.pose.pose.orientation.z = qz
        initial_pose.pose.pose.orientation.w = qw
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.0685

        self.initial_pose_pub.publish(initial_pose)
        self.initial_pose_repeats -= 1

    def publish_decision_mode(self):
        if not bool(self.get_parameter("publish_decision_mode").value):
            return

        self.ensure_decision_mode_publisher()
        mode = self.resolve_decision_mode()
        if mode != self.last_mode:
            self.get_logger().info("Decision simulation mode: %s" % mode)
            self.last_mode = mode

        message = String()
        message.data = mode
        self.decision_mode_pub.publish(message)

    def publish_referee_messages(self):
        if not bool(self.get_parameter("publish_referee_inputs").value):
            return

        game_status = GameStatus()
        game_status.game_progress = int(self.get_parameter("game_progress").value)
        game_status.stage_remain_time = int(
            self.get_parameter("stage_remain_time").value
        )
        self.game_status_pub.publish(game_status)

        robot_status = RobotStatus()
        robot_status.robot_id = int(self.get_parameter("robot_id").value)
        robot_status.current_hp = int(self.get_parameter("current_hp").value)
        robot_status.maximum_hp = int(self.get_parameter("maximum_hp").value)
        robot_status.shooter_barrel_cooling_value = int(
            self.get_parameter("shooter_barrel_cooling_value").value
        )
        robot_status.shooter_barrel_heat_limit = int(
            self.get_parameter("shooter_barrel_heat_limit").value
        )
        robot_status.shooter_17mm_1_barrel_heat = int(
            self.get_parameter("shooter_17mm_1_barrel_heat").value
        )
        robot_status.projectile_allowance_17mm = int(
            self.get_parameter("projectile_allowance_17mm").value
        )
        robot_status.remaining_gold_coin = int(
            self.get_parameter("remaining_gold_coin").value
        )
        robot_status.armor_id = int(self.get_parameter("armor_id").value)
        robot_status.hp_deduction_reason = int(
            self.get_parameter("hp_deduction_reason").value
        )
        robot_status.is_hp_deduced = bool(
            self.get_parameter("is_hp_deduced").value
        )
        robot_status.robot_pos.orientation.w = 1.0
        self.robot_status_pub.publish(robot_status)

        rfid_status = RfidStatus()
        rfid_status.friendly_fortress_gain_point = bool(
            self.get_parameter("friendly_fortress_gain_point").value
        )
        rfid_status.friendly_supply_zone_non_exchange = bool(
            self.get_parameter("friendly_supply_zone_non_exchange").value
        )
        rfid_status.friendly_supply_zone_exchange = bool(
            self.get_parameter("friendly_supply_zone_exchange").value
        )
        rfid_status.center_gain_point = bool(
            self.get_parameter("center_gain_point").value
        )
        self.rfid_status_pub.publish(rfid_status)

    def publish_vision_message(self):
        if not bool(self.get_parameter("publish_vision_target").value):
            return

        self.ensure_vision_target_publisher()

        # 这里发布的是行为树真正会消费的融合消息，
        # 所以字段命名和语义与 sp_vision25 -> behavior 的正式链路保持一致。
        msg = VisionTargetMsg()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.tracking = bool(self.get_parameter("vision_tracking").value)
        msg.nav_hold = bool(self.get_parameter("vision_nav_hold").value)
        msg.fire_permitted = bool(self.get_parameter("vision_fire_permitted").value)
        msg.target_id = int(self.get_parameter("vision_target_id").value)
        msg.target_type = int(self.get_parameter("vision_target_type").value)
        msg.confidence = float(self.get_parameter("vision_confidence").value)
        msg.target_distance = float(
            self.get_parameter("vision_target_distance").value
        )
        msg.target_yaw = float(self.get_parameter("vision_target_yaw").value)
        msg.target_pitch = float(self.get_parameter("vision_target_pitch").value)
        msg.target_position_gimbal.x = float(
            self.get_parameter("vision_target_position_gimbal_x").value
        )
        msg.target_position_gimbal.y = float(
            self.get_parameter("vision_target_position_gimbal_y").value
        )
        msg.target_position_gimbal.z = float(
            self.get_parameter("vision_target_position_gimbal_z").value
        )
        msg.target_position_map.x = float(
            self.get_parameter("vision_target_position_map_x").value
        )
        msg.target_position_map.y = float(
            self.get_parameter("vision_target_position_map_y").value
        )
        msg.target_position_map.z = float(
            self.get_parameter("vision_target_position_map_z").value
        )
        msg.has_target_position_map = bool(
            self.get_parameter("vision_has_target_position_map").value
        )
        msg.target_position_map_frame = str(
            self.get_parameter("vision_target_position_map_frame").value
        )
        self.vision_target_pub.publish(msg)

        if msg.has_target_position_map and msg.target_position_map_frame:
            # 单独补发一个 PointStamped，直接喂给 RViz 的 PointStamped display，
            # 这样在 loopback 下可以直观看到视觉目标地图点是否真的更新了。
            target_point = PointStamped()
            target_point.header.stamp = msg.timestamp
            target_point.header.frame_id = msg.target_position_map_frame
            target_point.point = msg.target_position_map
            self.vision_target_point_map_pub.publish(target_point)

    def publish_loop(self):
        self.publish_decision_mode()
        self.publish_referee_messages()
        self.publish_vision_message()


def main():
    rclpy.init()
    node = FakeDecisionSimInputs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
