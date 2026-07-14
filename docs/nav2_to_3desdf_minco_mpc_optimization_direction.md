# ATS 自研导航 V1 当前状态与下一阶段交接

更新时间：2026-07-14

有效更新窗口：2026-07-12 至 2026-07-14。本文只保留当前有效架构、最近三天验证结果和下一阶段任务。完整仿真统一使用 MuJoCo，`loopback_sim` 仅用于低成本接口检查。

本文中的状态含义：

- `已验证`：已有源码、构建或 MuJoCo 运行证据。
- `迁移中`：已有部分能力，但运行时仍依赖 Nav2 或旧地图入口。
- `目标`：V1 必须实现，不能当作当前已完成能力。

## 0. V1 架构决策

### 0.1 V1 比赛主线

V1 面向四驱四转舵轮地面哨兵，最终比赛主线确定为脱离 Nav2 的自研 ROS2 导航方案：

`传感器 + 独立状态估计 -> ROGMap 概率地图/膨胀地图/3D ESDF -> 地面投影与 2.5D 可通行语义 -> RC-ESDF 规划接口 -> 自研目标管理 -> JPS -> MINCO S3 -> 独立 yaw -> footprint safety -> Local Collision Repair -> 全向 SE2 MPC -> 底盘`

架构约束：

1. ROGMap 是 V1 的建图与 ESDF 地图后端主线。
2. ROGMap 不是完整定位器。它消费外部里程计与点云，不估计机器人位姿；状态估计仍须独立提供 `map -> odom -> base_link`。
3. 自研比赛链不得依赖 Nav2 的 `NavigateToPose`、BT Navigator、planner server、controller server、costmap 或 `/plan` 才能运行。
4. 初期可继续复用 ROS 标准消息、TF2、RViz2 和 lifecycle，但不能把“使用 ROS2 基础设施”误写为“仍使用 Nav2 导航框架”。
5. 舵轮状态保持世界系 `[x, y, yaw]`，控制保持车体系 `[vx, vy, wz]`。禁止迁入 DDR 差速车的 ICR、曲率转向或 `vy=0` 约束。

### 0.2 Nav2 的新定位

Nav2 从 V1 默认比赛主线降级为：

- 已验证功能的回归对照组；
- ROGMap/自研任务管理尚未完成时的临时仿真回退；
- MPPI 与自研 SE2 MPC 的性能比较基线。

后续不得再以“Nav2 接 JPS/MINCO/MPC”作为 V1 终局描述。删除 Nav2 源码或立即破坏现有回归没有收益，因此兼容启动保留到自研链完成同等测试覆盖后再评估移除。

### 0.3 当前真实运行状态

| 模式 | 当前实际控制链 | 状态与用途 |
| --- | --- | --- |
| Nav2 + MPPI | `SmacPlanner2D -> RC-ESDF local elastic path -> Nav2BSplineSmoother -> trajectory_speed_governor -> MPPI -> 底盘` | `已验证`；仅作稳定回归与对照，不再是 V1 比赛目标。 |
| Nav2 上游 + 自研规划控制 | `NavigateToPose -> Smac /plan -> RC-ESDF planning grid -> JPS -> MINCO -> 独立 yaw -> footprint gate -> SE2 MPC -> 底盘` | `迁移中`；JPS/MINCO/MPC 已接管规划优化与控制，但目标管理和全局引导仍依赖 Nav2。 |
| ROGMap + 自研 ROS2 导航 | `ROGMap -> 自研 goal/action -> JPS -> MINCO -> 独立 yaw -> footprint safety/repair -> SE2 MPC` | `目标`；尚未形成可启动、可回归的完整链。 |

当前 `launch_swerve_mpc:=true` 会关闭 `fake_vel_transform`，`twist_to_motion_ctrl` 只订阅 `/cmd_vel_mpc`，因此 MPPI 不能同时驱动 MuJoCo 底盘。但 Nav2 action 和 `/plan` 仍在上游运行，不能据此声称已脱离 Nav2。

## 1. ROGMap 参考实现的能力边界

### 1.1 本地来源与当前状态

参考源码位于 `参考/src/rog_map`，当前不在根仓库版本跟踪中，也没有进入活动 colcon 主线。下一阶段应将确认过许可证和版本的依赖放入受版本控制的位置，不能让比赛构建依赖本机未跟踪目录。

本地参考实现已确认：

1. `rog_map` 的 CMake 当前只构建静态库，没有安装可直接 `ros2 run` 的节点可执行文件。
2. `ROGMapROS` 包装类订阅 `nav_msgs/msg/Odometry` 和 `sensor_msgs/msg/PointCloud2`，默认配置名为 `/lidar_slam/odom` 与 `/cloud_registered`。
3. 内部包含概率占据、unknown、inflation 和真实 3D ESDF 更新；`ESDFMap` 提供 `getDistance()`、`evaluateEDT()` 与梯度相关接口。
4. ROS 包装层当前发布的 `rog_map/occ`、`rog_map/inf_occ`、`rog_map/unk`、`rog_map/esdf` 都是 `PointCloud2` 可视化输出，不是 JPS/MINCO 所需的稳定数值地图契约。
5. 参考包装层硬编码 `world`/`drone` frame 并主动广播 TF。接入 ATS 时必须参数化为统一 frame，且状态估计是 `map -> odom -> base_link` 的唯一所有者，ROGMap 不得重复广播机器人位姿 TF。
6. 当前回调 QoS 为 best-effort、depth 1；实车接入前须分别为点云、里程计和规划地图明确 QoS 与超时策略。

许可证备注：参考文件头与 `package.xml` 的许可证声明不完全一致。正式搬入活动源码前必须确认上游许可证、保留版权声明并记录具体版本。

### 1.2 ATS 侧目标接口

第一阶段不要求同时改写 JPS/MINCO，先用适配层保持现有规划接口：

| 接口 | 类型/语义 | 所有者 |
| --- | --- | --- |
| `/localization` | `nav_msgs/msg/Odometry`；状态估计输出，不由 ROGMap 生成。 | localization |
| `/registered_scan` | 已配准或可按时间戳变换的 `PointCloud2`。 | lidar/localization |
| `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk` | ROGMap 调试点云；只用于 RViz 和诊断。 | ROGMap |
| `/rog_map/esdf` | ROGMap ESDF 调试点云；禁止将可视化点云反解析成优化器距离场。 | ROGMap |
| `/rc_esdf/planning_grid` | `nav_msgs/msg/OccupancyGrid`；ROGMap 地面投影与 terrain 语义融合后的 JPS 入口。 | ATS ROGMap adapter |
| 数值 ESDF 查询 | 保留 signed distance、unknown 和梯度，不经过 `0..100` 显示编码。 | ATS ROGMap adapter/provider |

迁移早期可以让 ROGMap adapter 继续发布 `/rc_esdf/planning_grid`，从而保持 JPS、MINCO 和 footprint gate 的已测接口。中期必须为 MINCO 增加 ROGMap 数值 ESDF provider，直接消费浮点距离与梯度；`/rog_map/esdf` 只保留可视化用途。

### 1.3 frame 与地图语义

目标 frame 约束：

1. `map` 是比赛全局规划 frame。
2. 状态估计发布 `map -> odom`，底盘里程计发布 `odom -> base_link`；传感器外参连接到 `base_link`。
3. ROGMap 必须按点云时间戳查询传感器到 `map` 的位姿，禁止只取“最近一帧 odom”而忽略时间同步。
4. 地面投影必须显式定义高度带、坡度阈值、悬空障碍、地面以下噪点和 unknown 策略。
5. 每份 planning grid 和 ESDF 数据必须携带一致的 frame、resolution、origin、尺寸、时间戳和 map generation/version。

## 2. 当前 RC-ESDF、JPS、MINCO 与 MPC 基线

### 2.1 当前地图输入

当前已验证链仍使用：

| Topic / 数据 | 当前作用 |
| --- | --- |
| `/localization` | `odom -> gimbal_yaw_odom`，Nav2 与 MPC 位姿输入。 |
| `/local_pointcloud` | `front_mid360`，MID360 局部观察。 |
| `/registered_scan` | `odom`，供 `terrain_analysis` 与 `terrain_analysis_ext` 消费。 |
| `/terrain_map_ext` | 2.5D 调试与语义来源。 |
| `/traversability_grid`、`/traversability_slope_grid` | terrain 可通行性和坡度语义。 |
| `/rc_esdf/planning_grid` | 当前由静态 `/map` 与 terrain 融合生成，尚未由 ROGMap 提供。 |

`terrain_analysis_ext` 使用 ROS 标准 `x + y * width` 索引。`/rc_esdf/planning_grid` 当前以 `0.10 m` 规划分辨率上采样局部 `0.40 m` terrain 语义，并融合静态 PGM 墙体。

### 2.2 当前 signed distance 语义

`RcTraversabilityEsdfProvider` 使用精确二维 signed Euclidean Distance Transform，不使用 `fake_costmap_esdf_provider` 作为运行时后端。

| 数据 | 当前定义 |
| --- | --- |
| `/rc_esdf/planning_grid` | `0` free，`1..49` 软风险，`50..100` occupied，`-1` unknown；unknown 对 JPS/MINCO 按障碍处理。 |
| 内部 signed distance | $d=d_{occ}-d_{free}$；$d>0$ free，$d<0$ occupied。 |
| `/rc_esdf/signed_distance_grid` | RViz 编码：`-1` unknown，`0..49` 负距离，`50` 零距离，`51..100` 正距离；截断到 `2.0 m`。 |
| `/rc_esdf/footprint_clearance_grid` | 保守外接圆可视化，尺寸为 `0.60 x 0.50 m + 0.02 m`；不能替代 runtime gate。 |

运行时 MINCO footprint gate 使用 `0.70 x 0.55 m + 0.05 m` 的定向矩形。ROGMap 接入后必须保持相同正负号、unknown 和 footprint 语义，避免地图后端切换导致安全阈值反向或失效。

### 2.3 自研规划控制的已完成能力

1. `minco_planner` 已能直接订阅 `goal_pose`；`global_plan_topic` 设为空后不会订阅 Nav2 `/plan`。
2. `onGoal()` 能从 TF 获取当前位姿，在最新 planning grid 上独立运行 JPS，失败时回退 A*，不需要 Smac 路径内容。
3. MINCO S3 生成连续位置、世界系 `vx/vy/ax/ay` 和时间戳。
4. `yaw_mode: clearance_aware` 保持平移与朝向解耦，支持舵轮横移。
5. yaw-aware footprint RC-ESDF 内点修正、最终矩形 gate 和可选 local repair 已接入。
6. `ats_swerve_mpc` 使用 `[vx, vy, wz]` 跟踪 MINCO reference，输出 `/cmd_vel_mpc`。

当前主要缺口不是重新实现 JPS/MINCO/MPC，而是 ROGMap 活动节点、地图适配、自研 goal/action 状态机、Nav2-free launch 和不依赖 `NavigateToPose` 的回归脚本。

## 3. Nav2-free 目标运行链

### 3.1 最小可运行链

下一阶段首先实现以下最小链：

`MuJoCo sensors/localization -> ats_rog_map_node -> ats_rog_map_adapter -> /rc_esdf/planning_grid + 数值 ESDF -> /goal_pose -> JPS -> MINCO -> yaw/footprint gate -> SE2 MPC -> /cmd_vel_mpc -> twist_to_motion_ctrl -> /motion_control`

该模式必须满足：

1. `launch_nav2:=false` 时仍能接收目标、生成路径并到达目标。
2. 不存在 Nav2 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server` 或 lifecycle manager。
3. 不订阅 `/plan`，不调用 `nav2_msgs/action/NavigateToPose`。
4. `/cmd_vel_mpc` 只有一个控制发布者，底盘 bridge 只有一个对应订阅入口。
5. ROGMap/adapter 未就绪、地图过期、目标不可达、轨迹被 gate 拒绝或 MPC 异常时，任务状态必须明确失败并停止底盘。

### 3.2 自研目标管理

迁移顺序：

1. 首先使用现有 `/goal_pose` 贯通 Nav2-free 仿真，验证地图、JPS、MINCO 和 MPC 的因果链。
2. 随后在 ATS 自有接口包中新增 action，提供 goal、feedback、result、cancel、preempt 和 timeout；不能仅用无状态 topic 作为比赛任务接口。
3. 自研 action server 负责任务状态与安全停止，规划器只负责地图上的路径生成，MPC 只负责跟踪，避免职责重新耦合成单节点。
4. 决策层最终只调用 ATS action，不直接依赖 Nav2 action 类型。

`src/interfaces` 当前没有 `.action` 定义，因此 action 接口属于未实现任务，不能在 launch 或文档中写成已有能力。

## 4. 最近三天验证记录

### 4.1 2026-07-13

1. 扩大矩形回归 `TEST_PROFILE=rectangle` 五段均完成，south/north 检测到非零 `/cmd_vel_mpc.linear.y`，证明控制未退化为差速转向。
2. 当前实验链已验证 `/plan`、`/minco/raw_path`、`/minco/reference_path`、MPC reference/predicted path、`/cmd_vel_mpc`、`/motion_control` 和单一控制发布者/订阅者。

### 4.2 2026-07-14

1. `UsesYawAwareFootprintToIncreaseEdgeClearance` 单测确认 P2.1 后最小足迹净空大于 `0.39 m`，较中心候选提高超过 `0.05 m`，连续起终点不变。
2. `test_rc_esdf_map`、`test_grid_jps`、`test_minco_trajectory_optimizer`、`test_yaw_spline_planner` 全部通过。
3. 红框长路线 `TEST_PROFILE=red_box` 两段 Nav2 `NavigateToPose` 均为 `SUCCEEDED`：中转终点 `(-8.9181, 1.4698)`，误差 `0.038 m`；红框终点 `(-0.0845, -4.0685)`，误差 `0.046 m`。
4. 主段记录 `raw_points=39`、`reference_points=707`、`length=22.28 m`、`collisions=0`；MPC reference horizon、predicted path、`/cmd_vel_mpc`、`/motion_control` 录制均非空。

验证边界：以上结果证明当前静态 MuJoCo 场景中的 Nav2 上游 + 自研规划控制链可运行，不证明 ROGMap、自研 action、Nav2-free 启动、动态障碍、连续 swept volume 或实车安全已完成。

## 5. 下一阶段实施顺序

### P0：架构与交接文档

状态：`已完成`。

- V1 已明确为 ROGMap + 自研 ROS2 导航主线。
- Nav2 已降为对照和回退。
- 当前能力与目标能力已分开描述。

### P1：将 ROGMap 纳入活动构建

这是下一次对话应首先完成的任务，范围只覆盖地图节点和接口，不同时切换底盘控制。

1. 确认上游版本与许可证，将 ROGMap 依赖放入受版本控制的工作区位置。
2. 增加可执行的 `ats_rog_map_node`，加载 YAML，订阅 MuJoCo `/localization` 与 `/registered_scan`。
3. 参数化 `map_frame`、`odom_frame`、`base_frame`、cloud/odom topic、QoS、超时和更新频率；移除 `world`/`drone` 硬编码与重复 TF 广播。
4. 在 MuJoCo 中确认概率占据、inflation、unknown 和 3D ESDF 输出非空，frame、时间戳和分辨率一致。
5. 运行 ROGMap 自带概率融合/衰减测试，并增加 ATS wrapper 的启动与接口测试。

P1 完成标准：干净工作区可独立 colcon 构建；节点不依赖 `参考/`；输入停止后能报告 stale；RViz 可稳定看到 occupied/unknown/inflated/ESDF；TF 树无重复发布者。

### P2：ROGMap 到地面规划接口

1. 新增 ATS adapter，从 ROGMap 概率/ESDF 生成地面规划层，不从可视化点云反解析距离。
2. 融合 `traversability_grid`、`traversability_slope_grid` 与高度带语义，明确 free/occupied/unknown 和速度限制。
3. 首先兼容发布 `/rc_esdf/planning_grid`，保持 JPS/MINCO 已测入口。
4. 增加无损浮点 signed-distance/gradient provider，逐步替换 MINCO 内部二维重算 ESDF。
5. 对中心、矩形 footprint 和 swept footprint 使用同一 map generation，地图更新过程中不得混用新旧快照。

P2 完成标准：墙体、坡度和 unknown 均能阻断 JPS；MINCO clearance 与 ROGMap 数值查询一致；RViz 显示与 runtime gate 结论一致。

### P3：Nav2-free 启动与目标状态机

1. 新增自研导航 launch 模式，关闭 Nav2，`minco_planner.global_plan_topic` 设为空。
2. 先用 `/goal_pose` 完成长路线，再增加 ATS 自定义 Navigate action 与任务状态机。
3. 改造回归脚本，不调用 `nav2_msgs/action/NavigateToPose`，直接验证 ATS goal/action。
4. 增加地图未就绪、目标不可达、规划失败、轨迹不安全、取消和抢占测试。

P3 完成标准：进程图中无 Nav2 节点和 `/plan` 依赖，仍能完成扩大矩形与红框长路线，终点误差、碰撞数、横移量和唯一控制权均自动判定。

### P4：连续安全与舵轮执行约束

1. 为相邻 MINCO 时刻补连续 swept-volume 检查，覆盖窄门、贴边、纯横移和大 yaw 变化。
2. 依据实车轴距、轮距、最大轮速、最大舵角速度和反馈延迟标定 MPC 约束。
3. 完成控制 mux、急停优先级、地图/规划/MPC 健康检查后再灰度到实车。

## 6. 当前构建与回归入口

当前以下命令仍是 Nav2 上游实验链的回归入口，不能用于证明 Nav2-free 目标已完成：

```bash
MAKEFLAGS=-j1 colcon build --packages-select trajectory_optimizer minco_planner --parallel-workers 1
MAKEFLAGS=-j1 colcon build --packages-select ats_swerve_mpc --parallel-workers 1
source install/setup.bash

ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  launch_nav2:=true \
  launch_swerve_mpc:=true \
  use_viewer:=false \
  show_viewer:=false \
  launch_mujoco_rviz:=false \
  lidar_backend:=cpu

TEST_PROFILE=rectangle scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

Nav2-free 的正式命令只能在 P1-P3 对应 launch 和回归脚本落地后补入，避免文档提供当前不存在的参数或伪启动方式。

## 7. 维护约束

1. 后续提交统一在各仓库 `develop` 分支完成并推送 `origin/develop`。
2. 文档只保留最近三天验证记录；过期状态合并为当前结论，不保留历史流水账。
3. 参考目录不是运行时依赖。进入比赛链的源码、配置、消息定义和许可证信息必须受版本控制。
4. 修改地图语义时必须验证 frame、时间戳、分辨率、origin、unknown 和 signed distance；修改控制路径时必须重跑 Nav2-free 红框或同等距离路线。
5. 未完成 ROGMap Nav2-free 回归前，保留当前 Nav2 模式作为对照，但新功能优先落在自研链。
