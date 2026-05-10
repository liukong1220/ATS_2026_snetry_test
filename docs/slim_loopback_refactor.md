# 轻量 Loopback 仿真说明

这份文档详细说明当前 loopback 仿真链路、运行命令、视觉测试方法、重定位测试方法和关键参数。

## 1. 当前 loopback 的定位

当前 loopback 不是物理仿真器，而是软件闭环仿真层。

它的目标是：

1. 不接实车
2. 不接串口
3. 不跑完整真实定位链
4. 但仍然保留行为树、Nav2、地图、TF、scan、RViz 和视觉接管调试能力

## 2. 当前入口

### 2.1 通用决策仿真入口

- [../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py](../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)

### 2.2 视觉专测入口

- [../src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py](../src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)

### 2.3 仿真器本体

- [../src/loopback_sim/nav2_loopback_sim/loopback_simulator.py](../src/loopback_sim/nav2_loopback_sim/loopback_simulator.py)

### 2.4 假输入节点

- [../src/pb2025_sentry_bringup/scripts/fake_decision_sim_inputs.py](../src/pb2025_sentry_bringup/scripts/fake_decision_sim_inputs.py)

## 3. 当前闭环结构

```text
fake_decision_sim_inputs.py
  -> initialpose
  -> decision/sim_mode
  -> referee/*
  -> vision/target

pb2025_sentry_behavior
  -> decision_path
  -> decision/robot_mode
  -> cmd_spin
  -> cmd_gimbal
  -> /navigate_through_poses

Nav2
  -> SmacPlannerHybrid
  -> Nav2BSplineSmoother
  -> MPPI
  -> trajectory_speed_governor
  -> velocity_smoother
  -> cmd_vel_nav2_result

fake_vel_transform
  -> cmd_spin + cmd_vel_nav2_result
  -> /cmd_vel

loopback_simulator
  -> /odom
  -> TF
  -> /scan
  -> /clock
```

## 4. 当前 loopback 的关键代码与算法

### 4.1 假输入节点发布什么

`fake_decision_sim_inputs.py` 当前会按参数发布：

1. `initialpose`
2. `decision/sim_mode`
3. `referee/game_status`
4. `referee/robot_status`
5. `referee/rfid_status`
6. `vision/target`
7. `vision/target_point_map`

### 4.2 loopback 仿真器做什么

`loopback_simulator.py` 当前负责：

1. 接收 `/cmd_vel`
2. 积分生成 `/odom`
3. 发布 `map -> odom -> base_footprint` 等 TF
4. 结合静态地图生成 `/scan`
5. 发布 `/clock`

### 4.3 当前视觉圆周跟随算法也在 loopback 中完整生效

loopback 并不自己决定攻击点，仍然完全复用：

- [../src/pb2025_sentry_behavior/plugins/action/select_vision_follow_path.cpp](../src/pb2025_sentry_behavior/plugins/action/select_vision_follow_path.cpp)

当前算法流程：

1. 读取 `target_position_map`
2. 根据当前车位计算目标圆周上的最近点
3. 用 costmap 筛选候选点
4. 若当前最近点不可用，则扩大角域回退
5. 对最终点做角度限幅平滑
6. 位姿跳变时清缓存重选

因此 loopback 里看到的攻击圆周点变化，和实车行为层逻辑是一致的。

## 5. 当前参数入口

### 5.1 行为树参数

- loopback 主树：  
  [../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml](../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)
- 视觉专测：  
  [../src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml](../src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml)

### 5.2 Nav2 参数

- [../src/loopback_sim/params/nav2_params.yaml](../src/loopback_sim/params/nav2_params.yaml)

### 5.3 RViz 配置

- [../src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz](../src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz)

当前已包含：

- `decision/robot_mode_markers`
- `decision/vision_follow_markers`

## 6. 当前推荐启动命令

### 6.1 通用决策仿真

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

### 6.2 视觉接管与攻击圆周跟随

```bash
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

当前 workspace 已默认设置 `ROS_DOMAIN_ID=90`，
因此 loopback 常规启动时只需要 `source install/setup.bash`。

## 7. 当前视觉测试参数的详细含义

### 7.1 目标有效性相关

- `publish_vision_target`
  是否发布假视觉消息。
- `vision_tracking`
  是否认为目标已被稳定跟踪。
- `vision_nav_hold`
  是否允许行为树进入视觉接管。
- `vision_has_target_position_map`
  是否声明当前视觉消息包含可用于导航的地图点。
- `vision_target_position_map_frame`
  目标地图点所属坐标系，当前通常为 `map`。

### 7.2 目标位置相关

- `vision_target_position_map_x`
- `vision_target_position_map_y`
- `vision_target_position_map_z`

这三项共同决定视觉跟随使用的目标地图点。

### 7.3 云台观测相关

- `vision_target_yaw`
- `vision_target_pitch`
- `vision_target_position_gimbal_x`
- `vision_target_position_gimbal_y`
- `vision_target_position_gimbal_z`

这些参数主要给云台和消息语义使用。

### 7.4 资源门控相关

- `current_hp`
- `projectile_allowance_17mm`

因为当前视觉接管前面还有资源模式门控，所以这两个参数太低时，即使视觉消息有效，也不会进入 `attack`。

## 8. 当前运行中动态改参数的方法

### 8.1 修改视觉目标地图点

```bash
ros2 param set /fake_decision_sim_inputs vision_target_position_map_x 4.2
ros2 param set /fake_decision_sim_inputs vision_target_position_map_y 1.6
ros2 param set /fake_decision_sim_inputs vision_target_position_map_z 0.0
```

### 8.2 修改云台角度

```bash
ros2 param set /fake_decision_sim_inputs vision_target_yaw 0.10
ros2 param set /fake_decision_sim_inputs vision_target_pitch -0.03
```

### 8.3 修改 simulation 模式

```bash
ros2 param set /fake_decision_sim_inputs decision_mode patrol
ros2 param set /fake_decision_sim_inputs decision_mode anchor
ros2 param set /fake_decision_sim_inputs decision_mode retreat
ros2 param set /fake_decision_sim_inputs decision_mode safe
```

### 8.4 修改资源输入

```bash
ros2 param set /fake_decision_sim_inputs current_hp 280
ros2 param set /fake_decision_sim_inputs projectile_allowance_17mm 60
```

### 8.5 修改行为层视觉跟随参数

```bash
ros2 param set /pb2025_sentry_behavior_server decision.vision.attack_radius 1.8
ros2 param set /pb2025_sentry_behavior_server decision.vision.min_replan_interval_s 0.15
ros2 param set /pb2025_sentry_behavior_server decision.vision.min_goal_shift_m 0.10
ros2 param set /pb2025_sentry_behavior_server decision.vision.max_goal_angle_step_deg 25.0
ros2 param set /pb2025_sentry_behavior_server decision.vision.pose_jump_reset_distance_m 0.8
ros2 param set /pb2025_sentry_behavior_server decision.vision.pose_jump_reset_angle_deg 55.0
```

## 9. 当前重定位测试方法

运行中发送：

```bash
export ROS_DOMAIN_ID=90
source install/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 1.5, y: 4.5, z: 0.0}, orientation: {z: 0.70710678, w: 0.70710678}}}}"
```

当前预期现象：

1. `loopback_simulator` 打印 `Received initial pose!`
2. 行为层打印 `Reset cached vision-follow state after pose jump ...`
3. `Vision follow target=(...) nearest_ring_goal=(...) selected_goal=(...)` 切到新的最近圆周侧
4. 姿态仍保持 `attack`

## 10. 当前调试时应该观察什么

### 10.1 观察姿态

```bash
ros2 topic echo /decision/robot_mode
ros2 topic echo /decision/robot_mode_markers
```

### 10.2 观察视觉目标

```bash
ros2 topic echo /vision/target
ros2 topic echo /vision/target_point_map
```

### 10.3 观察攻击圆周可视化

```bash
ros2 topic echo /decision/vision_follow_markers
```

### 10.4 观察行为层日志

重点看：

- `Decision resource mode=...`
- `Vision override rejected: ...`
- `Vision follow target=(...) nearest_ring_goal=(...) selected_goal=(...)`
- `Send NavigateThroughPoses goal with ...`
- `Robot posture switched: ...`

## 11. 当前最常见问题与原因

### 11.1 看不到视觉接管

优先检查：

1. `publish_vision_target`
2. `vision_tracking`
3. `vision_nav_hold`
4. `vision_has_target_position_map`
5. `current_hp`
6. `projectile_allowance_17mm`

### 11.2 看不到攻击圆周 Marker

优先检查：

1. `decision/vision_follow_markers`
2. RViz 是否加载 `sentry_default_view.rviz`
3. `target_position_map` 是否有效

### 11.3 到点后仍反复发目标

优先检查：

1. `path_goal_reached_tolerance`
2. `active_goal_hold_tolerance`
3. `active_goal_min_resend_interval_s`
4. `vision_active_goal_hold_tolerance`
5. `vision_active_goal_min_resend_interval_s`

### 11.4 TF / 时钟异常

当前最常见原因不是代码，而是同域串线。

如果同一 `ROS_DOMAIN_ID` 下同时存在：

1. 多套 loopback
2. 其他 `/clock`
3. 其他 TF
4. 额外 `/initialpose` 发布源

就容易出现：

- `Detected jump back in time`
- `Message Filter dropping message`
- `Vision override rejected`
- `decision_current_pose is stale`

因此当前建议：

- 每次 loopback 测试固定使用独立 `ROS_DOMAIN_ID`

## 12. 当前 loopback 与实车的共用边界

共用部分：

1. 行为树
2. 姿态切换
3. 资源模式状态机
4. 受击自旋
5. 视觉接管判定
6. 攻击圆周跟随点选择
7. `SendNavThroughPoses` 稳定器
8. `SmacPlannerHybrid -> Nav2BSplineSmoother -> MPPI -> trajectory_speed_governor -> velocity_smoother` 主链结构

不同部分：

1. 输入源
2. Nav2 参数
3. 串口与真实传感器链路

## 13. 相关文档

- [./总览.md](./总览.md)
- [./omni_recovery_smoothing_optimization.md](./omni_recovery_smoothing_optimization.md)
- [./sentry_bt_decision_checklist.md](./sentry_bt_decision_checklist.md)
- [./sentry_posture_switch_logic.md](./sentry_posture_switch_logic.md)
- [./融合.md](./融合.md)
- [./视觉跟随仿真调试.md](./视觉跟随仿真调试.md)
