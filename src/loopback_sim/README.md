# nav2_loopback_sim

当前工作区中的轻量软件闭环仿真层。

本包当前作用不是物理仿真，而是：

1. 接收 `/cmd_vel`
2. 积分生成 `/odom`
3. 维护最小必要 TF 链
4. 发布 `/scan`
5. 发布 `/clock`
6. 让 Nav2 与行为树在无实车条件下仍能形成完整闭环

## 当前入口

通常不直接单独启动本包，而是通过：

- [../ats_sentry_bringup/launch/loopback_decision_sim.launch.py](../ats_sentry_bringup/launch/loopback_decision_sim.launch.py)
- [../ats_sentry_bringup/launch/loopback_vision_test.launch.py](../ats_sentry_bringup/launch/loopback_vision_test.launch.py)
- [../ats_sentry_bringup/launch/loopback_nav_only.launch.py](../ats_sentry_bringup/launch/loopback_nav_only.launch.py)

仿真器本体：

- [nav2_loopback_sim/loopback_simulator.py](./nav2_loopback_sim/loopback_simulator.py)

默认参数：

- [params/nav2_params.yaml](./params/nav2_params.yaml)

## 当前闭环结构

```text
fake_decision_sim_inputs.py
  -> initialpose
  -> decision/sim_mode
  -> referee/*
  -> vision/target

ats_sentry_behavior
  -> /navigate_through_poses
  -> decision/robot_mode
  -> cmd_spin

Nav2
  -> planner / smoother / MPPI / governor / velocity_smoother
  -> cmd_vel_nav2_result

fake_vel_transform
  -> cmd_spin + cmd_vel_nav2_result
  -> /cmd_vel

nav2_loopback_sim
  -> /odom
  -> TF
  -> /scan
  -> /clock
```

## 当前功能

### 1. 位姿闭环

当前 loopback 会维护最小必要坐标链：

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

### 2. 仿真时钟

当前持续发布：

- `/clock`

因此所有 `use_sim_time=True` 的节点都能在 loopback 下正常运行。

### 3. 简化激光

当前会基于静态地图生成：

- `/scan`

它不是高保真物理雷达，但足以驱动 Nav2 的 local/global costmap。

### 4. 重定位测试

支持运行中重发：

- `/initialpose`

用于验证：

1. 行为树当前位姿更新
2. Nav2 重定位后的路径更新
3. 视觉跟随点是否重新选择

## 当前推荐命令

### 通用决策仿真

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

### 视觉接管专测

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup loopback_vision_test.launch.py \
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

### 纯导航观察

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup loopback_nav_only.launch.py use_rviz:=True
```

## 当前注意事项

1. 不要在同一 `ROS_DOMAIN_ID` 中同时运行多套 loopback 或其他 `/clock` 发布者
2. 当前工作区 loopback 常规调试默认使用 `ROS_DOMAIN_ID=90` 的环境 hook；若行为异常，先确认终端环境是否来自当前工作区 `install/setup.bash`
3. 若 `ros2 param set` 卡住，可优先尝试 `--no-daemon`

## 当前维护边界

1. 调仿真器自身 `/odom`、TF、`/scan` 生成逻辑，在本包改
2. 调行为树决策、视觉接管，不在本包改，去 `ats_sentry_behavior`
3. 调 Nav2、MPPI、平滑、恢复行为，不在本包改，去 `ats_sentry_nav`
4. 调假输入参数与组合 launch，不在本包改，去 `ats_sentry_bringup`

## 相关文档

- [../../docs/总览.md](../../docs/总览.md)
- [../../docs/视觉跟随仿真调试.md](../../docs/视觉跟随仿真调试.md)
- [../../docs/slim_loopback_refactor.md](../../docs/slim_loopback_refactor.md)
