# ATS 自研导航 V1 当前状态与下一阶段交接

更新时间：2026-07-15

有效更新窗口：2026-07-12 至 2026-07-15。本文只保留当前有效架构、最近三天验证结果和下一阶段任务。完整仿真统一使用 MuJoCo，`loopback_sim` 仅用于低成本接口检查。

本文中的状态含义：

- `已验证`：已有源码、构建或 MuJoCo 运行证据。
- `本阶段完成`：源码、构建、针对性测试和当前 MuJoCo 运行验收均已完成；不代表后续阶段已切换到该能力。
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
| ROGMap + 自研 ROS2 导航 | `ROGMap -> 自研 goal/action -> JPS -> MINCO -> 独立 yaw -> footprint safety/repair -> SE2 MPC` | `迁移中`；P1 ROGMap 地图节点已完成，但 P2 地面规划适配、目标状态机和 Nav2-free 完整回归仍未实现。 |

当前 `launch_swerve_mpc:=true` 会关闭 `fake_vel_transform`，`twist_to_motion_ctrl` 只订阅 `/cmd_vel_mpc`，因此 MPPI 不能同时驱动 MuJoCo 底盘。但 Nav2 action 和 `/plan` 仍在上游运行，不能据此声称已脱离 Nav2。

## 1. ROGMap 参考来源与活动实现

### 1.1 本地来源与当前状态

上游参考源码保留在 `参考/src/rog_map`，只用于溯源与比较。活动实现位于 `src/ats_sentry_nav/ats_rog_map`，属于 `ats_sentry_nav` 独立 Git 仓库；构建和运行均不依赖 `参考/`。

活动包为 `ats_rog_map_node`，使用 LGPL-3.0-or-later，并随包保留 `LICENSE` 与 `NOTICE`。节点默认订阅 `/localization` 与 `/registered_scan`，默认运行 frame 为 `map_frame=odom`、`base_frame=gimbal_yaw_odom`、`sensor_frame=front_mid360`。`base_frame` 仅控制 Sliding Map 中心，`sensor_frame` 仅作为 raycasting 起点，不能再将云台传感器原点误当作机器人中心。

活动实现与参考源码已确认：

1. 参考 `rog_map` 原本只构建静态库；活动包已提供可直接 `ros2 run ats_rog_map ats_rog_map_node` 的 ROS 2 节点。
2. 内部保留概率占据、unknown、inflation 与真实 3D ESDF；`ESDFMap` 提供距离查询，P1 只用于地图和调试输出。
3. 节点按点云时间戳查询 TF；已在 `odom` 的 `/registered_scan` 不会被再次按里程计变换。
4. 节点不发布机器人位姿 TF，只发布 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 与 `/rog_map/stale`。
5. 四类点云均是诊断/RViz 数据，禁止从 `/rog_map/esdf` 点云反解析为 MINCO 距离场；P1 有意不发布 `/rc_esdf/planning_grid`。
6. 输入与调试点云默认 best-effort、depth 1；输入过期通过 `cloud_timeout_sec` 与 `odom_timeout_sec` 报告 `/rog_map/stale`。MuJoCo 因 ESDF 计算开销使用 `2.0 s` 超时，实车仍需按目标机负载复核。

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

P1 只建立 ROGMap 地图节点和调试接口，不切换 `/rc_esdf/planning_grid` 所有者。P2 才新增 ATS adapter 以保持 JPS、MINCO 和 footprint gate 的已测接口；之后再为 MINCO 增加直接消费浮点距离与梯度的 ROGMap 数值 ESDF provider。

### 1.3 frame 与地图语义

目标 frame 约束：

1. `map` 是比赛全局规划 frame。
2. 状态估计发布 `map -> odom`，底盘里程计发布 `odom -> base_link`；传感器外参连接到 `base_link`。
3. ROGMap 必须按点云时间戳查询传感器到 `map` 的位姿，禁止只取“最近一帧 odom”而忽略时间同步。
4. 地面投影必须显式定义高度带、坡度阈值、悬空障碍、地面以下噪点和 unknown 策略。
5. 每份 planning grid 和 ESDF 数据必须携带一致的 frame、resolution、origin、尺寸、时间戳和 map generation/version。

P1 当前的 `odom` 仅是局部滑动调试地图 frame，与 Point-LIO `/registered_scan` 契约一致；全局 `map` 规划语义、地面投影与 `map`/`odom` 对齐都属于 P2，不能把当前 `/rog_map/*` 写成已完成的全局规划地图。

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

当前主要缺口不是重新实现 JPS/MINCO/MPC，而是 ROGMap 到地面规划接口的 P2 适配、自研 goal/action 状态机、Nav2-free launch 和不依赖 `NavigateToPose` 的回归脚本。

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

### 4.3 2026-07-15：ROGMap P1 与云台/底盘兼容链

1. `ats_rog_map`、`ats_nav_bringup`、`ats_sentry_bringup` 与 `ats_mujoco_sim` 单线程构建通过；`prob_map_log_odds_fusion_test`、`prob_map_stale_decay_test`、`test_split_robot_sensor_pose` 全部通过。
2. 在隔离 `ROS_DOMAIN_ID=88`、无 viewer/RViz、`launch_nav2:=false`、`launch_rog_map:=true` 的 MuJoCo 验收中，`/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 的宽度分别为 `2`、`239`、`9797`、`2601`，均为 `odom` frame；持续输入时 `/rog_map/stale=false`。
3. `ros2 node info /ats_rog_map` 只列出地图诊断话题，不含 `/tf` 或 `/tf_static` 发布者；ROGMap 不参与机器人 TF 所有权。
4. `fake_yaw` 与底盘坐标转换在实机主入口默认启用。四种开关组合均有明确速度出口：双开时维持 `cmd_vel_nav2_result -> fake -> cmd_vel_gimbal_yaw_odom -> chassis -> /cmd_vel`；仅底盘开时 chassis 直接订阅 `cmd_vel_nav2_result`；双关时 Nav2 直接输出 `/cmd_vel`；仅 fake 开时 fake 直接输出 `/cmd_vel`。实机不得在下游仍要求底盘坐标速度时仅关闭 chassis；该组合只适用于显式接管 gimbal-yaw 速度的外部执行器。
5. fake 关闭时启动 `gimbal_yaw_odom -> gimbal_yaw_fake` 的零旋转兼容 TF，使现有 Nav2 frame 参数不失效，但不会执行 fake-yaw 动态旋转。`base_footprint -> base_link` 静态 TF 改由内层 navigation launch 单一所有；启用 `robot_state_publisher` 时不再额外发布该静态 TF。

验证边界：本节只验证了当前稀疏 MuJoCo 点云、调试地图和开关路由。`rog_map_report_025.yaml` 尚未在目标机完成 50 Hz、约 6 ms、峰值 RSS 或完整 2.5 cm ESDF 性能验收；不得将技术报告数字写成 ATS 实测结果。

## 5. 下一阶段实施顺序

### P0：架构与交接文档

状态：`已完成`。

- V1 已明确为 ROGMap + 自研 ROS2 导航主线。
- Nav2 已降为对照和回退。
- 当前能力与目标能力已分开描述。

### P1：将 ROGMap 纳入活动构建

状态：`本阶段完成`。范围只覆盖三维 Sliding Map、诊断接口和现有云台/底盘启动兼容，不切换 JPS/MINCO 的地图所有者。

已完成范围：

1. `ats_rog_map` 已进入 `ats_sentry_nav` 独立版本库，活动构建不依赖 `参考/`；许可证为 LGPL-3.0-or-later，版权与通知文件随包安装。
2. `ats_rog_map_node` 已接收 `/localization`、`/registered_scan` 和点云时间戳 TF，机器人位姿与传感器 ray origin 已拆分；输入停止会发布 `/rog_map/stale`。
3. 已提供 occupied、inflated、unknown、ESDF 四类调试点云，并在 MuJoCo 验收为非空；节点不发布机器人 TF，也不发布 `/rc_esdf/planning_grid`。
4. 概率融合、stale 衰减与机器人/传感器位姿拆分均有针对性测试；启动参数允许保留或关闭 fake-yaw 与底盘转换，并为关闭 fake-yaw 提供静态兼容 frame。

| 配置 | 占据/膨胀分辨率 | ESDF 分辨率 | ESDF 刷新 | `p_occ` | 用途 |
| --- | ---: | ---: | ---: | ---: | --- |
| `rog_map.yaml` | `0.10 m` | `0.10 m` | 每 10 次地图更新 | `0.80` | 默认启动档 |
| `rog_map_050.yaml` | `0.05 m` | `0.10 m` | 每 5 次地图更新 | `0.80` | 中间精度档 |
| `rog_map_report_025.yaml` | `0.025 m` | `0.10 m` | 每 5 次地图更新 | `0.80` | 技术报告导向的占据/膨胀参数档 |
| `rog_map_mujoco.yaml` | `0.10 m` | `0.10 m` | 每 10 次地图更新 | `0.70` | MuJoCo 稀疏扫描档 |

四档的地图尺寸均为 `10 x 10 x 1 m`，虚拟地面/顶面为 `-0.50/1.00 m`。`update_interval_updates` 是地图更新计数，不是 Hz。`rog_map_report_025.yaml` 仅复现 `2.5 cm` 占据/膨胀分辨率，ESDF 仍为 `0.10 m`；在目标机记录 CPU、峰值 RSS、输入点数、地图更新时间和 ESDF 耗时前，不能引用技术报告的 `50 Hz` 或约 `6 ms` 作为 ATS 性能结果。

后续加固项：补充自动化 ROS launch/接口测试以替代本轮手工 MuJoCo smoke test；该项不改变 P2 的功能边界。

### P2：ROGMap 到地面规划接口

状态：`未实现`。`ats_rog_map_node` 有意不发布 `/rc_esdf/planning_grid`；当前 RC-ESDF 节点仍是该规划契约的唯一所有者。

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

## 6. 下一对话接续入口

下一对话从 P2 开始，不能重复实现 ROGMap 节点，也不能把 `/rog_map/esdf` 调试点云作为规划距离场。第一步应设计 `ats_rog_map_adapter` 的数值接口与地面投影快照，再决定 `/rc_esdf/planning_grid` 的切换时机。

必须保持以下边界：

1. Point-LIO 继续提供 `/localization` 与 `/registered_scan`；不得用 ROGMap 替换里程计。
2. RC-ESDF、JPS、MINCO 与全向 SE2 MPC 继续保留；P2 期间 `/rc_esdf/planning_grid` 只能有一个发布者。
3. 默认保留 `launch_fake_vel_transform:=True` 与 `launch_chassis_vel_transform:=True`。固定雷达迁移时可关闭开关；关闭 fake-yaw 仍保留零旋转兼容 TF，待所有 Nav2/行为参数改为固定 frame 后再删除该兼容层。
4. P2 的最小 DoD 是：ROGMap 地面适配层能处理墙体、坡度、unknown 和高度带，JPS/MINCO 查询同一 generation 的数值 ESDF，且不经 PointCloud2 反解析。

本阶段复验命令：

```bash
MAKEFLAGS=-j1 colcon build \
  --packages-select ats_rog_map ats_nav_bringup ats_sentry_bringup ats_mujoco_sim \
  --parallel-workers 1
colcon test --packages-select ats_rog_map
colcon test-result --test-result-base build/ats_rog_map --verbose
```

MuJoCo P1 smoke test 使用 `launch_nav2:=false launch_rog_map:=true`，并关闭 viewer、RViz、MPC 与速度 bridge；验收 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 非空、`frame_id=odom`、`/rog_map/stale=false`，且 `ats_rog_map` 不发布 TF。

## 7. 当前构建与回归入口

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

Nav2-free 的正式命令只能在 P2-P3 对应 launch 和回归脚本落地后补入，避免文档提供当前不存在的参数或伪启动方式。

## 8. 维护约束

1. 后续提交统一在各仓库 `develop` 分支完成并推送 `origin/develop`。
2. 文档只保留最近三天验证记录；过期状态合并为当前结论，不保留历史流水账。
3. 参考目录不是运行时依赖。进入比赛链的源码、配置、消息定义和许可证信息必须受版本控制。
4. 修改地图语义时必须验证 frame、时间戳、分辨率、origin、unknown 和 signed distance；修改控制路径时必须重跑 Nav2-free 红框或同等距离路线。
5. 未完成 ROGMap Nav2-free 回归前，保留当前 Nav2 模式作为对照，但新功能优先落在自研链。
