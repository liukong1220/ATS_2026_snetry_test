# ATS 导航优化当前状态与下一阶段交接

更新时间：2026-07-14

有效更新窗口：2026-07-12 至 2026-07-14。本文只保留当前有效架构、最近三天的验证结果和后续任务；更早的基线、已替代链路和流水账已移除。完整仿真统一使用 MuJoCo，`loopback_sim` 仅用于低成本链路检查。

## 1. V1 当前边界

### 1.1 比赛目标

V1 面向四驱四转舵轮地面哨兵，目标链为：

`2.5D 地形语义 -> 2D 栅格 -> RC-ESDF -> JPS -> MINCO S3 -> 独立 yaw -> footprint safety -> Local Collision Repair -> 全向 SE2 MPC`

舵轮底盘保持世界系位置状态 `[x, y, yaw]` 与车体系控制 `[vx, vy, wz]`。禁止迁入 DDR 差速车的 ICR、曲率转向或 `vy=0` 约束。

### 1.2 两条当前运行链

| 模式 | 启动条件 | 实际控制链 | 状态 |
| --- | --- | --- | --- |
| 默认比赛/稳定基线 | `launch_swerve_mpc:=false` | `SmacPlanner2D -> RC-ESDF local elastic path -> Nav2BSplineSmoother -> trajectory_speed_governor -> MPPI -> cmd_vel_nav2_result -> fake_vel_transform -> 底盘` | 默认保留；回归范围更多。 |
| MuJoCo 全向实验链 | `launch_swerve_mpc:=true` | `NavigateToPose -> Smac /plan -> /rc_esdf/planning_grid -> JPS -> MINCO -> 独立 yaw -> footprint-aware RC-ESDF -> swept-footprint gate -> SE2 MPC -> /cmd_vel_mpc -> twist_to_motion_ctrl -> /motion_control` | 已通过长路线仿真；尚未灰度到实车。 |

实验链不是仅发布可视化路径：该开关会关闭 `fake_vel_transform`，`twist_to_motion_ctrl` 只订阅 `/cmd_vel_mpc`。Nav2 的 planner/action 节点仍运行并提供 `/plan`，但 MPPI 输出不能再驱动 MuJoCo 底盘。

### 1.3 V2 边界

V2 是长期的 `3D Occupancy / 3D ESDF -> 地面可通行语义` 地图后端升级。当前不做三维路径搜索；V1 继续以地面 `2D/2.5D` 路径、安全和控制接口为主。

## 2. 当前数据与控制接口

### 2.1 MuJoCo 与地形输入

| Topic / 数据 | frame / 作用 |
| --- | --- |
| `/localization` | `odom -> gimbal_yaw_odom`，Nav2 与 MPC 位姿输入。 |
| `/local_pointcloud` | `front_mid360`，MID360 局部观察。 |
| `/registered_scan` | `odom`，直接供 `terrain_analysis` 与 `terrain_analysis_ext` 消费。 |
| `/terrain_map_ext` | 2.5D 调试与语义来源。 |
| `/traversability_grid`、`/traversability_slope_grid` | terrain 可通行性和坡度语义。 |
| `/rc_esdf/planning_grid` | `/map` 与 terrain 融合后的 JPS/MINCO/主链 ESDF 输入。 |

`terrain_analysis_ext` 发布栅格使用 ROS 标准 `x + y * width` 索引。`/rc_esdf/planning_grid` 继承 terrain 的 `odom`、origin、时间戳；当前以 `0.10 m` 规划分辨率上采样局部 `0.40 m` terrain 语义，并按静态 PGM 原始分辨率融合墙体。

### 2.2 RC-ESDF 语义

`RcTraversabilityEsdfProvider` 使用精确二维 signed Euclidean Distance Transform，不使用 `fake_costmap_esdf_provider` 作为运行时后端。

| 数据 | 当前定义 |
| --- | --- |
| `/rc_esdf/planning_grid` | `0` free，`1..49` 软风险，`50..100` occupied，`-1` unknown；unknown 对 MINCO/JPS 按障碍处理。 |
| 内部 signed distance | $d=d_{occ}-d_{free}$；$d>0$ free，$d<0$ occupied。 |
| `/rc_esdf/signed_distance_grid` | RViz 运输编码：`-1` unknown，`0..49` 负距离，`50` 零距离，`51..100` 正距离；截断到 `2.0 m`。 |
| `/rc_esdf/footprint_clearance_grid` | 可视化保守外接圆 clearance，尺寸为 `0.60 x 0.50 m + 0.02 m`。不能替代 MINCO runtime safety。 |

运行时 MINCO footprint gate 使用 `0.70 x 0.55 m + 0.05 m` 的定向矩形，尺寸更保守。因此 RViz clearance 数值可能比 runtime gate 乐观，所有发布轨迹仍以 gate 结果为准。

## 3. JPS、MINCO 与舵轮 MPC

### 3.1 规划与安全

1. `minco_planner` 订阅 `/rc_esdf/planning_grid`，默认 JPS，A* 仅作为失败回退。
2. `jps_safe_distance: 0.57 m` 不低于当前矩形任意 yaw 的保守外接半径。
3. JPS/A* 只量化内部搜索索引，输出路径保留连续起终点，避免格心偏移进入 MINCO。
4. MINCO S3 生成连续位置、世界系 `vx/vy/ax/ay` 和时间戳。
5. `yaw_mode: clearance_aware`：开阔区域保持目标朝向；窄区按净空滞回选择正/反路径切向，yaw 不强制等于平移切线。
6. 最终 `FootprintSafetyChecker` 对定向矩形逐点检查；不安全轨迹默认拒绝发布。local repair 默认关闭，开启后必须重新经 MINCO 求导与 gate 检查。

### 3.2 P2.1：footprint-aware RC-ESDF 内点修正

当前采用两阶段，而不是位置-yaw 联合优化：

1. 先按质心 signed-distance gradient 生成中心 MINCO 候选。
2. 独立 yaw planner 为该候选生成 yaw reference。
3. 用与 footprint gate 共用的矩形采样点，在该 yaw 下查询 RC-ESDF；选择最小 signed distance 的正 gradient，仅修正 MINCO 世界系 `x/y` 内点并重新求 S3。
4. 端点固定。footprint 候选不安全而中心候选安全时保留中心候选；二者均不安全时回退原始 JPS-MINCO，再由 local repair 和最终 gate 决定是否发布。

当前参数位于 `src/ats_sentry_nav/minco_planner/config/minco_planner.yaml`：

```yaml
esdf_obstacle_clearance: 0.45
esdf_obstacle_max_iterations: 6
esdf_obstacle_control_point_spacing: 0.30
esdf_obstacle_max_step: 0.10
esdf_footprint_optimization_enabled: true
esdf_footprint_clearance: 0.10
esdf_footprint_sample_spacing: 0.10
```

### 3.3 MPC 与控制权

`ats_swerve_mpc` 使用最近 MINCO 线段投影而非 ROS 墙钟时间构建预测域，限制轨迹进度回跳和未来跳跃；横向误差较大时压缩参考进度与前馈速度。MuJoCo 中唯一底盘控制输入为：

`/cmd_vel_mpc -> twist_to_motion_ctrl -> /motion_control`

实车尚未完成 mux、急停优先级、轮端舵角速度、轮速和反馈延迟约束标定，因此不允许直接把该实验链作为实车默认控制器。

## 4. 最近三天验证记录

### 4.1 2026-07-13

1. 扩大矩形回归 `TEST_PROFILE=rectangle` 五段均完成，south/north 检测到非零 `/cmd_vel_mpc.linear.y`，证明未退化为差速转向。
2. MINCO/JPS/MPC 旁路已验证 `/plan`、`/minco/raw_path`、`/minco/reference_path`、MPC reference/predicted path、`/cmd_vel_mpc`、`/motion_control` 和单一控制发布者/订阅者。

### 4.2 2026-07-14

1. 新增 `UsesYawAwareFootprintToIncreaseEdgeClearance` 单测：构造“质心安全、矩形边缘接近障碍”路线，确认 P2.1 后最小足迹净空大于 `0.39 m`，且较中心候选提高超过 `0.05 m`，连续起终点不变。
2. `test_rc_esdf_map`、`test_grid_jps`、`test_minco_trajectory_optimizer`、`test_yaw_spline_planner` 全部通过。
3. 红框长路线 `TEST_PROFILE=red_box` 两段 `NavigateToPose` 均为 `SUCCEEDED`：中转终点 `(-8.9181, 1.4698)`，误差 `0.038 m`；红框终点 `(-0.0845, -4.0685)`，误差 `0.046 m`。
4. 主段记录 `raw_points=39`、`reference_points=707`、`length=22.28 m`、`collisions=0`；MPC reference horizon、predicted path、`/cmd_vel_mpc`、`/motion_control` 录制均非空。

验证边界：这些结果只证明当前 RMUC2026 静态 MuJoCo 场景与指定路线可运行；不证明窄门、动态障碍、连续 swept volume 或实车安全已完成。

## 5. 构建、启动与回归

低性能主机使用单线程构建：

```bash
MAKEFLAGS=-j1 colcon build --packages-select trajectory_optimizer minco_planner --parallel-workers 1
MAKEFLAGS=-j1 colcon build --packages-select ats_swerve_mpc --parallel-workers 1
source install/setup.bash
```

默认 MPPI 基线：

```bash
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  launch_swerve_mpc:=false \
  use_viewer:=false \
  launch_mujoco_rviz:=false
```

全向实验链：

```bash
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  launch_swerve_mpc:=true \
  use_viewer:=false \
  show_viewer:=false \
  launch_mujoco_rviz:=false \
  lidar_backend:=cpu
```

定向测试与 MuJoCo 回归：

```bash
ctest --test-dir build/trajectory_optimizer -R test_rc_esdf_map --output-on-failure
ctest --test-dir build/minco_planner \
  -R 'test_grid_jps|test_minco_trajectory_optimizer|test_yaw_spline_planner' \
  --output-on-failure

TEST_PROFILE=rectangle scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

实验链运行时应满足：存在 `/minco_planner`、`/ats_swerve_mpc`、`/twist_to_motion_ctrl`；不存在 `/fake_vel_transform`；`/cmd_vel_mpc` 恰好一个 MPC 发布者和一个 bridge 订阅者。

## 6. 下一阶段任务

按以下顺序推进，避免同时重构多包：

1. `P2.2：swept-volume 与窄门矩阵`
   对相邻 MINCO 采样时刻补连续 swept-volume 检查；在窄门、贴边、纯横移、大 yaw 变化和动态障碍场景建立 MuJoCo 自动测试矩阵。
2. `P2.3：position-yaw 交替重算`
   仅在 P2.1 yaw reference 与最终 yaw 差异过大时做有限次交替重算；评估是否需要完整联合优化，始终保持全向 `vy` 可用。
3. `P3：舵轮执行约束标定`
   依据实车轴距、轮距、最大轮速、最大舵角速度和延迟，标定 MPC 的 `max_vx/max_vy/max_wz/max_ax/max_ay/max_awz`。
4. `P4：控制权仲裁与实车灰度`
   先完成实车 mux/急停优先级和控制器健康检查，再评估 `/cmd_vel_mpc` 接入 `sentry_chassis_vel_transform`。
5. `性能问题`
   本轮红框回归中仍有 Nav2 BT/controller tick-rate 与高 footprint-cost 告警。它们未阻止 action 成功，但需在控制权隔离和低性能主机调度优化中消除。

## 7. 维护约束

1. 后续提交统一在各仓库的 `develop` 分支完成并推送 `origin/develop`。
2. 文档只追加最近三天的验证记录；过期状态应合并为当前结论，不保留历史流水账。
3. 涉及 RC-ESDF、MINCO、footprint 或 MPC 的改动，必须至少完成定向构建/测试；修改实际控制路径时还必须重跑 MuJoCo `red_box` 或覆盖同等距离的自动路线。
