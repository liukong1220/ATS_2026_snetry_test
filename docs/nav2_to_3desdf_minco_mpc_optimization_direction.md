# ATS 自研导航 V1 状态

更新时间：2026-08-02。本页只记录当前活动源码和本轮运行证据。

## P2

- `planning_grid_owner:=rog_map` 时，`ats_rog_map_adapter` 是
  `/rc_esdf/planning_grid` 的唯一发布者；adapter 直接调用
  `/rog_map/get_ground_projection` 数值服务，不使用 `/rog_map/esdf` 点云作为数值输入。
- MINCO 使用本地不可变 snapshot；RC-ESDF 保留 signed-distance、unknown、梯度、
  map 外、origin/yaw 和保守静态栅格融合语义。
- 已在独立 MuJoCo domain 运行 rectangle、red_box、adapter lease、projection service
  timeout、Point-LIO 输入 stale、all unknown 和 unreachable。每个故障都观察到
  `emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0`。

## P3

- 正式入口为 `ats_sentry_bringup/launch/bringup.launch.py` 和中立命名的
  `real_robot_navigation.launch.py`；自研入口固定 ROGMap、ATS action、Goal Manager、
  MINCO、SE2 MPC 与唯一速度链。
- 运行图无 Nav2 server，MINCO 不订阅 `/plan`；行为树多航点顺序调用
  `/ats_navigate_to_pose`。
- 本轮 MuJoCo 已验证 cancel、preempt、timeout、TF failure：action 分别返回预期结果，
  且急停后的两级速度为零。

## 配置与限制

- 正式节点参数集中于 `src/ats_sentry_bringup/params/node_params.yaml`；launch 仅覆盖
  `use_sim_time`、资产/设备路径和受控 HIL 开关。ROGMap core 从显式 ROS 参数构造配置，
  正式 profile 不接受第二份地图配置源。
- `static_map_publisher.py` 保留 `/map` 的 frame、origin/yaw、resolution、占据语义和
  transient-local QoS。
- 已验证 rectangle 终点误差 `0.004126 m`、red_box 终点误差 `0.003696 m`，两例离散
  footprint 冲突为 `0`、MuJoCo `contact_violation_count=0`。这不替代 P4 的连续 swept
  footprint 和实车动力学验证。
