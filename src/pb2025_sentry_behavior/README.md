# pb2025_sentry_behavior

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

> 开发中，不考虑向前兼容性，仅供参考，请谨慎使用。文档可能不会及时更新以反映代码的最新变化。

基于 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 和 [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) 的行为树框架与插件，用于 [RoboMaster](https://www.robomaster.com) 2025 赛季哨兵机器人。

当前仓库内和“现状”最相关的文档入口见：

- [`../../docs/sentry_bt_decision_checklist.md`](../../docs/sentry_bt_decision_checklist.md)
- [`../../docs/sentry_posture_switch_logic.md`](../../docs/sentry_posture_switch_logic.md)
- [`../../docs/视觉跟随仿真调试.md`](../../docs/视觉跟随仿真调试.md)
- [`../../docs/融合.md`](../../docs/融合.md)
- [`../../docs/实机视觉跟随优化方案.md`](../../docs/实机视觉跟随优化方案.md)

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS2 Humble
- BehaviorTree.CPP (Developed with release [4.6.2](https://github.com/BehaviorTree/BehaviorTree.CPP/releases/tag/4.6.2))

### 2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
pip install vcstool2
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior.git src/pb2025_sentry_behavior
```

```bash
vcs import --recursive src < src/pb2025_sentry_behavior/dependencies.repos
```

### 2.3 Build

```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.4 Running

```bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
```

## 3. Behaviors

### 3.1 Action

#### PubTwist

以 `geometry_msgs/msg/Twist` 的形式发布速度，用于控制底盘运动。

#### SendNav2Goal

创建 Client，以 `nav2_msgs/action/NavigateToPose` 的形式发送 Navigation2 目标点。

> [!CAUTION]
> BehaviorTree.ROS2 中存在 bug，导致继承自 [RosActionNode](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/cc31ea7b97947f1aac6e8c37df6cec379c84a7d9/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp#L80) 的节点无法被正确 halt，最终导致行为树被 shutdown。因此，下面提供了一个以 topic 代替 action 发布 goal_pose 的临时方案 `PubNav2Goal`，坏处是无法实时获取到 action 的 feedback 和 result。
>
> Related Issue:  [#18 Error when canceling action during halt()](https://github.com/BehaviorTree/BehaviorTree.ROS2/issues/18)

#### PubNav2Goal

以 `geometry_msgs/msg/pose_stamped` 的形式发布 Navigation2 目标点。

#### PublishRobotMode

发布哨兵当前姿态模式到 `decision/robot_mode`，供串口节点继续下发给下位机。

当前模式约定：

- `move = 0`
- `attack = 1`
- `defend = 2`

该节点内部会统一处理：

- 姿态切换冷却时间 `decision.mode_limits.switch_cooldown_s`
- 单局累计时长限制 `decision.mode_limits.max_cumulative_s`
- 姿态受限时的回退与保持策略

### 3.2 Condition

#### IsAttacked

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RobotStatus` 类型数据，判断机器人是否受到攻击，并根据裁判系统装甲模块反馈的信息输出敌方可能的角度位置。该条件节点会根据输入端口的配置，检查以下几个条件：

- `key_port`：从 GlobalBlackboard 获取 `RobotStatus` 消息
- `gimbal_pitch`：输出固定的云台俯仰角度（0.0）
- `gimbal_yaw`：输出敌方可能的角度位置

如果检测到装甲板被击中，则返回 `SUCCESS`，并输出相应的云台角度；否则返回 `FAILURE`。

当前版本额外约束为：

- 只要 `is_hp_deduced == true` 就视为本次应触发受击自旋
- 连续未发生新掉血超过 `decision.motion.hit_spin_stop_after_no_hp_drop_s` 后，受击自旋停止

行为树中通常与 `PublishSpinSpeed` 配合使用，受击时发布 `decision.motion.hit_spin_speed`，其余时间发布 `0.0`。

#### IsRobotHpBelow

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RobotStatus`，判断当前血量是否低于阈值。

当前主树已不再把它作为统一资源决策入口，更多适合保留给局部阈值判断或临时实验使用。

#### IsRobotResourceMode

统一根据 `RobotStatus` 中的血量和弹量判断当前资源状态。

当前资源状态分为：

- `engage`
- `resupply`
- `defend`

当前主树中，这个节点已经取代“视觉节点自己读低血量阈值”与“根树分散判断血量”的旧写法，用于统一控制：

- 健康状态时是否允许视觉接管
- 中低资源时是否回补给安全点
- 极低血量时是否立即退防

它内部直接读取以下参数，并且自带迟滞锁存，避免在阈值边缘来回抖动：

- `decision.resource_policy.defend_enter_hp`
- `decision.resource_policy.defend_exit_hp`
- `decision.resource_policy.resupply_enter_hp`
- `decision.resource_policy.resupply_exit_hp`
- `decision.resource_policy.resupply_enter_ammo`
- `decision.resource_policy.resupply_exit_ammo`

#### IsVisionTargetValid

统一判断视觉目标是否“可以接管行为树”。

它当前不仅检查：

- `tracking`
- `nav_hold`
- 时间戳是否过期
- yaw / pitch 是否为有限值

还会做三层平滑：

- `activation_hold_s`
  初次看到目标后，先稳定持续一小段时间再真正接管
- `switch_target_hold_s`
  已经锁定目标 A 时，新目标 B 必须持续稳定满足该时长才允许切换
- `override_hold_s`
  短时掉帧、遮挡或单帧异常时，继续保持旧目标，避免立刻掉回巡逻

因此它的职责不是“只要看见就追”，而是“给行为树一个平滑、可继承的视觉接管判定”。

#### SelectVisionFollowPath

根据视觉提供的敌方地图点，在目标周围生成一个供 Nav2 / MPPI 跟随的局部跟随点。

当前实现重点有三类稳定化逻辑：

- 跟随环采样
  在 `attack_radius` 半径附近采样候选点，并结合全局 costmap 过滤不可通行位置
- 同侧保持
  通过 `prefer_previous_goal_side` 与 `max_target_shift_for_side_hold_m`
  尽量保持机器人继续待在目标的同一侧，减少转角和近终点时突然翻边
- 实时重规划 + 角度平滑
  每个决策周期都会重新基于“当前机器人位置 + 当前敌方地图点”选圆周跟随点，
  再通过 `max_goal_angle_step_deg` 限制单拍角度跃迁，避免目标点瞬间跳边
- 位姿跳变重置
  通过 `pose_jump_reset_distance_m` 在重定位、手动改 `/initialpose` 或实车定位突变时
  直接清空旧缓存，保证尽快切到当前车位对应的最近圆周点
- 微抖死区
  通过 `min_replan_interval_s` 与 `min_goal_shift_m`
  只对极小幅度抖动保留一个短暂输出死区，但不会长期冻结整条视觉跟随路径

#### SendNavThroughPoses

向 Nav2 发送 `NavigateThroughPoses` 目标路径。

当前除了基础的“同路径不重复发送”外，还新增了“近似同目标不急着抢占”的稳定器：

- `decision.decision_config.active_goal_hold_tolerance`
  若新路径和当前已经发给 Nav2 的路径只在很小范围内偏移，则先保持当前目标
- `decision.decision_config.active_goal_min_resend_interval_s`
  即使检测到近似新目标，也要求与上次发目标至少隔一段时间才允许再次重发

这一层很重要，因为视觉和上层行为即便已经做了节流，如果 Nav2 入口仍然每次都
`cancel + resend`，在转角、近终点和贴墙跟随时仍然容易放大成 MPPI 左右试探。

另外当前实现会优先读取行为树黑板里的 `decision_current_pose` 来判断“是否真的还在终点”。
因此 loopback 手动拖动车体、重定位，或者实车定位链更新了当前车位后，即使路径名字没变，
也不会再被“上一次已经成功到点”这个旧状态卡死。

#### IsGameStatus

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::GameStatus` 类型数据，判断当前比赛状态是否在输入的时间范围内且处于预期的比赛阶段。该条件节点会根据输入端口的配置，检查以下几个条件：

- `key_port`：从 GlobalBlackboard 获取 `GameStatus` 消息
- `expected_game_progress`：预期的比赛阶段
- `min_remain_time`：最小剩余时间（秒）
- `max_remain_time`：最大剩余时间（秒）

如果比赛阶段和剩余时间都符合预期，则返回 `SUCCESS`，否则返回 `FAILURE`。

#### IsRfidDetected

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RfidStatus` 类型数据，判断机器人是否检测到指定的 RFID 标签。该条件节点会根据输入端口的配置，检查以下几个位置的 RFID 状态：

- `key_port`：从 GlobalBlackboard 获取 `RfidStatus` 消息
- `friendly_fortress_gain_point`：己方堡垒增益点
- `friendly_supply_zone_non_exchange`：己方与兑换区不重叠的补给区 / RMUL 补给区
- `friendly_supply_zone_exchange`：己方与兑换区重叠的补给区
- `center_gain_point`：中心增益点（仅 RMUL 适用）

如果任意一个配置为 `true` 的位置检测到 RFID 标签，则返回 `SUCCESS`，否则返回 `FAILURE`。

#### IsStatusOK

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RobotStatus` 类型数据，判断机器人的状态是否正常。该条件节点会根据输入端口的配置，检查以下几个条件：

- `key_port`：从 GlobalBlackboard 获取 `RobotStatus` 消息
- `hp_min`：最低血量
- `heat_max`：最大发射机构的射击热量
- `ammo_min`：最小弹丸允许发弹量

如果机器人的 HP、热量和弹药量都在预期范围内，则返回 `SUCCESS`，否则返回 `FAILURE`。
