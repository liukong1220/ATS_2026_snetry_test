# ATS 2026 Sentry Workspace

## 简介

ATS 四驱四转舵轮哨兵 ROS 2 工作区。活动导航链为 Point-LIO 定位、ROGMap 概率地图与
数值投影、RC-ESDF、JPS、MINCO S3、独立 yaw、footprint gate/Local Collision Repair、
全向 SE2 MPC 和底盘速度桥。状态保持世界系 `[x, y, yaw]`，控制保持车体系
`[vx, vy, wz]`。

## 模块

- `src/ats_sentry_bringup`：正式启动、根总 YAML、静态地图和默认 RViz。
- `src/ats_sentry_nav`：Point-LIO、ROGMap、adapter、RC-ESDF、JPS/MINCO、Goal Manager、MPC。
- `src/ats_sentry_behavior`：任务行为树和 ATS action client。
- `src/sim/ats_mujoco_sim`：物理仿真、传感器和唯一 `/motion_control` bridge。
- `src/sim/loopback_sim`：轻量栅格/行为回归，不提供物理接触证据。

## 依赖

- Ubuntu 22.04、ROS 2 Humble、GCC 11、Python 3。
- Eigen3、PCL、OpenCV、yaml-cpp；MuJoCo 仅用于物理仿真。
- `参考/` 只用于溯源，绝不作为活动构建输入。

## 构建

这是 ROS 2 多包工作区，正式构建与测试必须使用 `colcon`：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src --symlink-install --parallel-workers 1
source install/setup.bash
```

因 `参考/` 可能含同名包，所有 `colcon build`、`colcon test` 都必须保留
`--base-paths src`。局部 CMake 仅可用于诊断，不能替代 colcon 构建证据。

## 启动

实机总入口默认使用自研导航：

```bash
ros2 launch ats_sentry_bringup bringup.launch.py \
  world:=rmuc_2026 planning_grid_owner:=rog_map use_rviz:=false
```

中立的 `real_robot_navigation.launch.py` 提供同一自研编排入口。实机默认保留
`launch_fake_vel_transform:=True` 与 `launch_chassis_vel_transform:=True`，以及 fake-yaw
关闭时的兼容 TF；不得新增重复 TF 或 `/cmd_vel`/`/motion_control` owner。

MuJoCo 无界面闭环使用：

```bash
PLANNING_GRID_OWNER=rog_map TEST_PROFILE=red_box GOAL_TIMEOUT=180 \
scripts/test_mujoco_minco_mpc_chain.sh
```

## 接口

| 接口 | 权威 producer | consumer |
| :--- | :--- | :--- |
| `/ats_navigate_to_pose` | `ats_goal_manager` action server | 行为树、测试客户端 |
| `/rc_esdf/planning_grid` | `ats_rog_map_adapter` | MINCO、行为层 |
| `/minco/reference_path` | Goal Manager 提交 | MPC |
| `/planner/emergency_stop` | Goal Manager | MPC、底盘安全链 |
| `/cmd_vel_mpc` | `ats_swerve_mpc` | `twist_to_motion_ctrl` |
| `/motion_control` | `twist_to_motion_ctrl` | MuJoCo/底盘 |

ROGMap 的 `/rog_map/esdf` 是调试点云；数值规划只通过
`/rog_map/get_ground_projection` 服务与 RC-ESDF 接口传递。

## 配置

`src/ats_sentry_bringup/params/node_params.yaml` 是正式节点参数总 YAML，覆盖 ROGMap、
adapter、MINCO、Goal Manager 和 MPC。正式 ROGMap profile 不允许 `map_config_file` 与
显式 ROS 参数同时生效。launch 只覆盖 `use_sim_time`、资产/设备路径和受控 HIL 开关。

`static_map_publisher.py` 发布 `/map`，保留地图 frame、origin/yaw、resolution、占据语义和
transient-local QoS。

## 架构

```text
/localization + /registered_scan
  -> ROGMap -> 数值 ground projection -> ats_rog_map_adapter
  -> /rc_esdf/planning_grid + RC-ESDF
  -> ATS Goal Manager -> JPS -> MINCO S3 + yaw + footprint/repair
  -> 已复核的 reference -> 全向 SE2 MPC
  -> /cmd_vel_mpc -> twist_to_motion_ctrl -> /motion_control
```

ROGMap source generation、adapter publication 和 MINCO local snapshot generation 是不同的
编号域。单次 MINCO 规划内保持不可变 snapshot；不宣称三者端到端编号相同。

## 验证与限制

已验证：11 个受影响 ROS 2 包的 `colcon build --base-paths src --symlink-install`、相关
单测/语法/launch 参数展示、rectangle 和 red_box MuJoCo 闭环，以及 adapter lease、
service timeout、input stale、unknown、unreachable、cancel、preempt、timeout、TF failure
独立故障注入。rectangle 终点位置误差为 `0.004126 m`，red_box 为 `0.003696 m`；两例
离散 footprint 冲突为 `0`，接触 evaluator 均为 `0`。

各故障均观察到 `emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0`，并核对规划
grid、MPC 到 bridge、bridge 到底盘均为唯一 owner。实车、连续 swept footprint 和实车动力学
约束仍未验证，不能由 MuJoCo 结果替代。

根仓、导航仓和 MuJoCo 仓是独立 Git 仓库；状态、提交与普通 push 必须分别完成。
