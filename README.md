# ATS 2026 Sentry Workspace

安徽信息工程学院 Artisans 战队 2026 哨兵机器人工作区。

当前仓库实际主线只保留下面四条链路：

1. 实机总启动链路
2. 行为树决策与姿态切换链路
3. Nav2 + MPPI 导航执行链路
4. loopback 轻量仿真与视觉跟随测试链路

## 当前主线

当前默认架构是：

```text
pb2025_sentry_bringup
  -> pb2025_sentry_behavior
  -> /navigate_through_poses
  -> Nav2 Planner + MPPI Controller
  -> /cmd_vel
  -> standard_robot_pp_ros2 / loopback_sim
```

其中：

- `pb2025_sentry_behavior` 负责决策、姿态切换、视觉接管、受击自旋
- `pb2025_sentry_bringup` 负责实机与 loopback 启动编排
- `pb2025_sentry_nav` 负责 Nav2、定位、地图、传感器链路
- `standard_robot_pp_ros2` 负责上下位机串口与裁判系统接口
- `loopback_sim` 负责无实车条件下的软件闭环仿真

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- GCC / G++ 11
- CMake 3.16+

常用依赖：

```bash
sudo apt update
sudo apt install -y \
  git git-lfs curl wget python3-pip python3-vcstool python3-rosdep \
  build-essential cmake pkg-config libeigen3-dev libomp-dev
```

首次配置：

```bash
sudo rosdep init
rosdep update
```

## 构建

推荐直接使用仓库的一键脚本：

```bash
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
```

如果需要补 ROS 依赖：

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

## 当前常用启动入口

### 1. loopback 通用决策仿真

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

对应入口：

- [src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py](./src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)

### 2. loopback 视觉跟随测试

```bash
export ROS_DOMAIN_ID=90
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py \
  use_rviz:=True \
  publish_referee_inputs:=True \
  current_hp:=400 \
  projectile_allowance_17mm:=200 \
  publish_vision_target:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_has_target_position_map:=True \
  vision_target_position_map_frame:=map \
  vision_target_position_map_x:=5.0 \
  vision_target_position_map_y:=2.0 \
  vision_target_position_map_z:=0.0 \
  vision_target_yaw:=0.30 \
  vision_target_pitch:=-0.06
```

对应入口：

- [src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py](./src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)

### 3. 实机总启动

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup bringup.launch.py world:=<YOUR_WORLD_NAME> use_rviz:=True
```

对应入口：

- [src/pb2025_sentry_bringup/launch/bringup.launch.py](./src/pb2025_sentry_bringup/launch/bringup.launch.py)

## 当前最常改的参数文件

### 行为树参数

- loopback 主树：
  [src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml](./src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)
- 视觉专测：
  [src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml](./src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml)
- 实机主树：
  [src/pb2025_sentry_behavior/params/sentry_behavior.yaml](./src/pb2025_sentry_behavior/params/sentry_behavior.yaml)

### Nav2 参数

- loopback Nav2：
  [src/loopback_sim/params/nav2_params.yaml](./src/loopback_sim/params/nav2_params.yaml)
- 实机总入口 Nav2：
  [src/pb2025_sentry_bringup/params/node_params.yaml](./src/pb2025_sentry_bringup/params/node_params.yaml)
- 导航包 reality 默认参数：
  [src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml](./src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)

### 串口与模式下发参数

- [src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml](./src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml)

关键项：

- `robot_mode_topic`
- `/cmd_vel.angular.z -> speed_vector.wz`
- `/decision/robot_mode -> speed_vector.mode`

## 当前功能结论

### 姿态模式

当前姿态枚举固定为：

- `move = 0`
- `attack = 1`
- `defend = 2`

行为树通过 `decision/robot_mode` 发布姿态，下位机串口层写入：

- `SendRobotCmdData.data.speed_vector.mode`

### 受击自旋

当前受击逻辑使用：

- `RobotStatus.is_hp_deduced == true`

只要检测到新的掉血，就触发自旋；连续一段时间没有新的掉血，则停止自旋。

当前自旋速度由：

- `decision.motion.hit_spin_speed`

控制，最终通过：

- `/cmd_vel.angular.z`

发送给下位机，对应串口字段：

- `SendRobotCmdData.data.speed_vector.wz`

### 视觉跟随

当前视觉跟随使用 `VisionTargetMsg.target_position_map` 作为导航目标基础输入。

行为层每个决策周期都会：

1. 根据当前车位和敌方地图点计算最近攻击圆周点
2. 结合 costmap、边界余量、局部可通行性筛选候选点
3. 对最终点做角度限幅平滑
4. 通过 `SendNavThroughPoses` 发送到 Nav2

这套逻辑在 loopback 和实车共用。

## 当前最常调的关键参数

### 姿态与资源

- `decision.mode_limits.switch_cooldown_s`
- `decision.mode_limits.max_cumulative_s`
- `decision.resource_policy.defend_enter_hp`
- `decision.resource_policy.defend_exit_hp`
- `decision.resource_policy.resupply_enter_hp`
- `decision.resource_policy.resupply_exit_hp`
- `decision.resource_policy.resupply_enter_ammo`
- `decision.resource_policy.resupply_exit_ammo`

### 受击自旋

- `decision.motion.hit_spin_speed`
- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

### 视觉跟随

- `decision.vision.attack_radius`
- `decision.vision.timeout_s`
- `decision.vision.activation_hold_s`
- `decision.vision.switch_target_hold_s`
- `decision.vision.override_hold_s`
- `decision.vision.min_replan_interval_s`
- `decision.vision.min_goal_shift_m`
- `decision.vision.max_goal_angle_step_deg`
- `decision.vision.pose_jump_reset_distance_m`
- `decision.vision.pose_jump_reset_angle_deg`

### 路径重发节流

- `decision.decision_config.path_goal_reached_tolerance`
- `decision.decision_config.active_goal_hold_tolerance`
- `decision.decision_config.active_goal_min_resend_interval_s`
- `decision.decision_config.vision_active_goal_hold_tolerance`
- `decision.decision_config.vision_active_goal_min_resend_interval_s`

## 当前常用调试命令

### 查看姿态

```bash
ros2 topic echo /decision/robot_mode
ros2 topic echo /decision/robot_mode_markers
```

### 查看视觉接管与攻击圆周

```bash
ros2 topic echo /vision/target
ros2 topic echo /decision/vision_follow_markers
```

### 查看受击自旋链

```bash
ros2 topic echo /cmd_spin
ros2 topic echo /cmd_vel
```

### loopback 运行中重定位

```bash
export ROS_DOMAIN_ID=90
source install/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 1.5, y: 4.5, z: 0.0}, orientation: {z: 0.70710678, w: 0.70710678}}}}"
```

## 文档入口

建议按下面顺序阅读：

1. [docs/移植.md](./docs/移植.md)
2. [docs/sentry_bt_decision_checklist.md](./docs/sentry_bt_decision_checklist.md)
3. [docs/sentry_posture_switch_logic.md](./docs/sentry_posture_switch_logic.md)
4. [docs/slim_loopback_refactor.md](./docs/slim_loopback_refactor.md)
5. [docs/融合.md](./docs/融合.md)
6. [docs/视觉跟随仿真调试.md](./docs/视觉跟随仿真调试.md)
7. [docs/实机视觉跟随优化方案.md](./docs/实机视觉跟随优化方案.md)
8. [docs/mppi_parameter_tuning_guide.md](./docs/mppi_parameter_tuning_guide.md)
9. [docs/navigate_through_poses_migration_checklist.md](./docs/navigate_through_poses_migration_checklist.md)

## 仓库结构

```text
src/
├── loopback_sim              # 轻量仿真闭环
├── pb2025_robot_description  # 机器人模型
├── pb2025_sentry_behavior    # 行为树与 BT 插件
├── pb2025_sentry_bringup     # 实机 / loopback 启动入口
├── pb2025_sentry_nav         # 导航、定位、地图、传感器
├── sp_vision25               # 视觉算法工程
├── standard_robot_pp_ros2    # 串口驱动与机器人本体接口
├── interfaces                # 自定义消息
├── dependencies              # 第三方依赖
└── tools                     # 辅助工具
```

## 当前维护原则

1. 以 `pb2025_sentry_bringup` 为系统启动总入口
2. 以 `pb2025_sentry_behavior` 为主决策入口
3. 以 `/navigate_through_poses` 为统一导航执行接口
4. 以 `docs/` 中现状文档为维护说明主入口
5. 不再把旧的 PID / `goal_pose` / 旧决策支链当作当前主线
