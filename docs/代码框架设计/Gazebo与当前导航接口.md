# Gazebo 与当前自研导航接口

## 适用范围

本文记录当前 `develop` 代码中 Gazebo 导航入口与自研导航链的实际接口。Gazebo 负责物理世界、传感器/真值桥和底盘执行；定位、地图、规划与 MPC 由活动导航仓提供。MuJoCo 与 Gazebo 共享同一套导航 topic 和消息语义，因此可以用相同的接口检查脚本比较两种仿真域。

## 启动入口

```bash
ROS_DOMAIN_ID=231 \
  ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
  world:=rmuc_2025 planning_grid_owner:=rog_map \
  use_gazebo_gt_odometry:=true launch_small_gicp_relocalization:=true \
  headless:=true use_viewer:=false use_rviz:=false
```

`planning_grid_owner` 只在启动时选择 `rog_map` 或 `rc_esdf`，当前 P2 验收使用 `rog_map`。Gazebo 默认使用机器人自身的 `gimbal_odometry_gt` 和 LiDAR 真值中继，不会同时启动 Point-LIO 作为第二个 `/odometry` 所有者；设置 `use_gazebo_gt_odometry:=false` 才进入 Gazebo 的 Point-LIO 链。

## 端到端接口账本

| 阶段 | 唯一 producer | topic/接口 | frame 与时序约束 |
| --- | --- | --- | --- |
| Gazebo 真值里程计 | `gazebo_gt_odometry_relay` | `/<robot>/gimbal_odometry_gt` -> `/odometry` | 输入 `world -> <robot>/gimbal_yaw_odom`；输出 `odom -> gimbal_yaw_odom`，沿用仿真时钟 |
| Gazebo 真值点云 | `gazebo_gt_registered_scan_relay` | `/<robot>/livox/lidar` -> `/registered_scan` | 输出点云为 `odom`，查询 `odom <- front_mid360` TF；过期、TF 失败和转换失败均丢帧 |
| 重定位观测 | `small_gicp_relocalization` | `/relocalization_observation` | 先验地图为 `map`，观测 child 为 `gimbal_yaw_odom`；只发布观测，不拥有 `map -> odom` |
| 全局定位 | `localization_fusion` | `/localization`、`/localization/status`、`map -> odom` | `/localization` 保持 `odom -> gimbal_yaw_odom`；融合节点是 `map -> odom` 唯一所有者 |
| 地图投影 | `ats_rog_map` + `ats_rog_map_adapter` | `/rog_map/*`、`/rog_map/get_ground_projection`、`/rc_esdf/planning_grid` | adapter 使用数值 projection 服务，不反解析 `/rog_map/esdf` 点云；ready 是有租约的心跳 |
| 规划 | `minco_planner` | `/minco/raw_path`、`/minco/reference_path_candidate`、`/minco/planning_status` | JPS、MINCO、yaw、足迹门禁共享一次不可变规划快照；地图失效时确定性急停 |
| 目标复核与授权 | `ats_goal_manager` | `/ats_navigate_to_pose`、`/planner/execution_command`、`/planner/emergency_stop` | 目标管理器在提交点复核快照并重定时，取消/超时/失效均撤销执行授权 |
| 全向控制 | `ats_swerve_mpc` | `/cmd_vel/autonomy_raw`、`/ats_swerve_mpc/predicted_path` | 车体系 `[vx, vy, wz]`；定位、参考或急停不健康时输出零速度 |
| 速度仲裁 | `cmd_vel_arbiter` | `/cmd_vel` + `/cmd_vel/autonomy_raw` -> `/cmd_vel/selected` | 只保留一个选中速度发布者；Gazebo profile 不启动 fake/chassis transform |
| Gazebo 底盘执行 | `gz_chassis_cmd_adapter` | `/cmd_vel/selected` -> `/motion_control` + `/<robot>/cmd_vel` | 根据 `gimbal_yaw_odom_joint` 旋转平动速度；大 yaw 反馈缺失、急停或指令过期时两个输出均为精确零 |

## TF、QoS 与所有权边界

- `gimbal_yaw_odom -> front_mid360` 由 Gazebo launch 的静态 TF 发布；`gazebo_robot_tf_relay` 只转发命名机器人状态发布器的其余链，并丢弃会造成双父节点的 `chassis -> gimbal_yaw_odom` 边。
- `/rc_esdf/planning_grid`、`/cmd_vel/selected`、`/motion_control` 和 `map -> odom` 各只有一个权威 producer。启动时用 `ros2 topic info --verbose` 和 `ros2 node info` 检查 publisher/subscriber 数量，不能以 topic 存在代替所有权检查。
- `/rc_esdf/planning_grid` 使用可靠、瞬态本地 QoS；MINCO 必须能接收 late joiner 的首张栅格。传感器点云和里程计保留传感器 QoS，桥接层不得为了“看见消息”而放宽 frame、时间或 stale 门。

## 日志策略

规划器、MPC、GICP 重定位和 Gazebo bridge 的运行日志使用中文说明，保留 `planned generation=`、`jps failed:`、`TRACE execution_command` 等结构化键供回归脚本解析。规划器和 GICP 的重复诊断默认 `log_throttle_ms=2000`，允许通过参数在 `250--60000 ms` 范围内调整；CSV 候选诊断仍逐候选写盘，不把高频文本日志当作遥测通道。急停、无效输入、TF 失败和安全拒绝使用节流后的错误/警告级别，状态恢复仍在状态变化或下一次允许输出时记录。

## 验证命令

```bash
python3 scripts/validate_navigation_config.py
bash scripts/test_gazebo_runner_contract.sh
python3 -m py_compile \
  src/sim/gazebo_simulator/rmu_gazebo_simulator/launch/ats_gazebo_nav.launch.py \
  src/sim/gazebo_simulator/rmu_gazebo_simulator/scripts/ats_bridge/chassis_cmd_adapter.py
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select \
  minco_planner ats_swerve_mpc small_gicp_relocalization rmu_gazebo_simulator \
  --parallel-workers 1
```

闭环验收应使用新 `ROS_DOMAIN_ID` 和新 Gazebo 进程，至少保存 `/odometry`、`/localization`、`/localization/status`、`/rog_map_adapter/ready`、`/rc_esdf/planning_grid`、`/minco/reference_path`、`/planner/emergency_stop`、`/cmd_vel/selected`、`/motion_control` 的消息和所有权证据。无独立 Gazebo contact evaluator 时，日志中的 `footprint_collisions=0` 只能说明离散足迹门禁通过，不能推导物理接触为零。

## 本轮启动证据

在 `ROS_DOMAIN_ID=220`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=1`、无 GUI、
`launch_planning:=false`、`launch_small_gicp_relocalization:=false` 的隔离启动中，已观察到：

- Gazebo 世界加载、机器人生成和 `ros_gz_bridge` 的里程计、关节状态、Mid360 点云/IMU 桥接成功；
- `gazebo_gt_odometry_relay` 正常发布 `/odometry`，日志确认输入为
  `/<robot>/gimbal_odometry_gt`，输出 child 为 `gimbal_yaw_odom`；
- `gazebo_gt_registered_scan_relay` 正常发布 `/registered_scan`，统计达到 `成功=100 丢弃=0`；
- `localization_fusion` 正常创建并声明 `/localization`、`/localization/status` 和
  `/relocalization_observation` 接口。

该运行在 20 s 后由测试超时主动发送 `SIGINT`，只证明 Gazebo 传感器/定位接口能够启动并传输数据；
规划器、MPC、底盘执行和红框终点仍需使用独立进程完成完整闭环验收。

## 当前边界

- Gazebo 接口已按当前自研导航 topic、frame、急停和唯一所有者契约接线；这不等于当前 revision 已通过 nominal/red-box 全闭环。
- P3 Nav2-free 仍需单独证明：`launch_nav2:=false`、无 Nav2 server、目标入口不调用 `NavigateToPose`，并在自研入口下完成扩大矩形和红框验证。
- 本轮只改善接口账本和可读性；物理接触、连续 swept footprint、实车动力学和实车运行仍未验证。
