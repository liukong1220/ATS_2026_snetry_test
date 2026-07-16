# ATS 自研导航 V1 当前状态与下一阶段交接

更新时间：2026-07-16

有效更新窗口：2026-07-13 至 2026-07-16。本文只保留当前有效架构、最近验证结果和下一阶段任务。完整仿真统一使用 MuJoCo，`loopback_sim` 仅用于低成本接口检查。

本文中的状态含义：

- `已实现`：已有活动源码与可加载配置，但不等于运行通过。
- `已测试`：已有构建和针对性单元/组件测试证据。
- `已验证`：已有本轮实际 MuJoCo 闭环运行证据。
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
- P3 自研任务管理尚未完成时的临时仿真回退；
- MPPI 与自研 SE2 MPC 的性能比较基线。

后续不得再以“Nav2 接 JPS/MINCO/MPC”作为 V1 终局描述。删除 Nav2 源码或立即破坏现有回归没有收益，因此兼容启动保留到自研链完成同等测试覆盖后再评估移除。

### 0.3 当前真实运行状态

| 模式 | 当前实际控制链 | 状态与用途 |
| --- | --- | --- |
| Nav2 + MPPI | `SmacPlanner2D -> RC-ESDF local elastic path -> Nav2BSplineSmoother -> trajectory_speed_governor -> MPPI -> 底盘` | `已验证`；仅作稳定回归与对照，不再是 V1 比赛目标。 |
| ROGMap owner + Nav2 上游 + 自研规划控制 | `NavigateToPose -> /plan 目标触发 -> ROGMap 数值投影/2.5D 融合 -> JPS -> MINCO -> 独立 yaw -> footprint gate/repair -> SE2 MPC -> 底盘` | `P2 最小实现范围本阶段完成`；ROGMap adapter 是 planning grid 唯一所有者，但目标管理和全局触发仍依赖 Nav2。 |
| ROGMap + 自研 ROS2 导航 | `/goal_pose` 或 ATS `NavigateToPose` action -> 目标管理 -> ROGMap/adapter -> JPS -> MINCO -> 独立 yaw -> footprint safety/repair -> SE2 MPC -> 底盘 | `P3 主线已验证`；正式 MuJoCo 入口显式 `launch_nav2:=false`，不依赖 `/plan` 或 Nav2 action。 |

在 P3 中，`launch_swerve_mpc:=true` 会关闭 `fake_vel_transform`，`twist_to_motion_ctrl` 只订阅 `/cmd_vel_mpc`；目标管理器是 `/planner/emergency_stop` 与正式 `/minco/reference_path` 的唯一权威。Nav2 action 和 `/plan` 只保留在独立的对照模式，不能与 P3 运行图混用。

## 1. ROGMap 参考来源与活动实现

### 1.1 本地来源与当前状态

上游参考源码保留在 `参考/src/rog_map`，只用于溯源与比较。活动实现位于 `src/ats_sentry_nav/ats_rog_map`，属于 `ats_sentry_nav` 独立 Git 仓库；构建和运行均不依赖 `参考/`。

活动包为 `ats_rog_map_node`，使用 LGPL-3.0-or-later，并随包保留 `LICENSE` 与 `NOTICE`。节点默认订阅 `/localization` 与 `/registered_scan`，默认运行 frame 为 `map_frame=odom`、`base_frame=gimbal_yaw_odom`、`sensor_frame=front_mid360`。`base_frame` 仅控制 Sliding Map 中心，`sensor_frame` 仅作为 raycasting 起点，不能再将云台传感器原点误当作机器人中心。

活动实现与参考源码已确认：

1. 参考 `rog_map` 原本只构建静态库；活动包已提供可直接 `ros2 run ats_rog_map ats_rog_map_node` 的 ROS 2 节点。
2. 内部保留概率占据、unknown、inflation 与真实 3D ESDF；P2 数值 service 直接查询 raw occupancy 与 3D ESDF，不经过调试点云。
3. 节点按点云时间戳查询 TF；已在 `odom` 的 `/registered_scan` 不会被再次按里程计变换。
4. 节点不发布机器人位姿 TF，只发布地图诊断、`/rog_map/stale` 并提供 `/rog_map/get_ground_projection`；`/rc_esdf/planning_grid` 由 adapter 发布。
5. 四类点云均是诊断/RViz 数据，禁止从 `/rog_map/esdf` 点云反解析为 MINCO 距离场。
6. 输入与调试点云默认 best-effort、depth 1；输入过期通过 `cloud_timeout_sec` 与 `odom_timeout_sec` 报告 `/rog_map/stale`。MuJoCo 因 ESDF 计算开销使用 `2.0 s` 超时，实车仍需按目标机负载复核。

### 1.2 ATS 侧目标接口

P2 用适配层保持现有 JPS/MINCO 规划接口：

| 接口 | 类型/语义 | 所有者 |
| --- | --- | --- |
| `/localization` | `nav_msgs/msg/Odometry`；状态估计输出，不由 ROGMap 生成。 | localization |
| `/registered_scan` | 已配准或可按时间戳变换的 `PointCloud2`。 | lidar/localization |
| `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk` | ROGMap 调试点云；只用于 RViz 和诊断。 | ROGMap |
| `/rog_map/esdf` | ROGMap ESDF 调试点云；禁止将可视化点云反解析成优化器距离场。 | ROGMap |
| `/rog_map/get_ground_projection` | 同一 ROG source generation、来自概率地图且未膨胀的 raw 三态占据分类；正 free/负 occupied/unknown=`NaN` 的 signed distance；`occupancy_grid.header.frame_id` 坐标系梯度；ready/stale。 | ROGMap |
| `/rc_esdf/planning_grid` | `nav_msgs/msg/OccupancyGrid`；ROGMap 地面投影与 terrain 语义融合后的 JPS 入口。 | ATS ROGMap adapter |
| adapter 数值 snapshot | 保留 service 的 signed distance、unknown、梯度与 ROG generation，已提供查询接口；当前尚未跨进程提供给 MINCO。 | ATS ROGMap adapter |

P2 已新增 ATS adapter 并可通过 `planning_grid_owner:=rog_map` 切换 `/rc_esdf/planning_grid` 唯一所有者。MINCO 直接消费浮点距离与梯度的 provider 仍是后续优化，不能把 adapter 内部 snapshot 写成已经贯通到 MINCO。

### 1.3 frame 与地图语义

目标 frame 约束：

1. `map` 是比赛全局规划 frame。
2. 状态估计发布 `map -> odom`，底盘里程计发布 `odom -> base_link`；传感器外参连接到 `base_link`。
3. ROGMap 必须按点云时间戳查询传感器到 `map` 的位姿，禁止只取“最近一帧 odom”而忽略时间同步。
4. 地面投影必须显式定义高度带、坡度阈值、悬空障碍、地面以下噪点和 unknown 策略。
5. ROG 数值 service 内 occupancy、distance、gradient 与 source generation 属于同一快照；adapter 发布的 `OccupancyGrid` 不携带 source generation，MINCO 当前以 callback 本地编号构造不可变 grid + 二维 RC-ESDF snapshot。

ROGMap 当前仍以 `odom` 维护局部滑动三维地图；adapter 将投影变换并融合到静态 `map` planning grid。MINCO 在该 grid 上规划后把 reference 通过 TF 转到 MPC 所需的 `odom`，TF 失败时拒绝发布并急停。

## 2. 当前 RC-ESDF、JPS、MINCO 与 MPC 基线

### 2.1 当前地图输入

当前 P2 已验证链使用：

| Topic / 数据 | 当前作用 |
| --- | --- |
| `/localization` | `odom -> gimbal_yaw_odom`，Nav2 与 MPC 位姿输入。 |
| `/local_pointcloud` | `front_mid360`，MID360 局部观察。 |
| `/registered_scan` | `odom`，供 `terrain_analysis` 与 `terrain_analysis_ext` 消费。 |
| `/terrain_map_ext` | 2.5D 调试与语义来源。 |
| `/traversability_grid`、`/traversability_slope_grid` | terrain 可通行性和坡度语义。 |
| `/rc_esdf/planning_grid` | P2 由 ROGMap adapter 融合 raw occupancy、静态 `/map`、terrain 与 slope 后唯一发布；Nav2 对照模式仍可由原 RC-ESDF 发布。 |

`terrain_analysis_ext` 使用 ROS 标准 `x + y * width` 索引。`/rc_esdf/planning_grid` 当前以 `0.10 m` 规划分辨率上采样局部 `0.40 m` terrain 语义，并融合静态 PGM 墙体。静态图分辨率约为 `0.02847 m`，adapter 不再只采样输出单元中心，而是聚合每个输出 footprint 有面积重叠的所有静态源单元；任一 occupied 源单元都会保留。聚焦 GTest 已覆盖非整除分辨率、平移 origin 与非零 yaw。

### 2.2 当前 signed distance 语义

`RcTraversabilityEsdfProvider` 使用精确二维 signed Euclidean Distance Transform，不使用 `fake_costmap_esdf_provider` 作为运行时后端。

| 数据 | 当前定义 |
| --- | --- |
| `/rc_esdf/planning_grid` | `0` free，`1..49` 软风险，`50..100` occupied，`-1` unknown；unknown 对 JPS/MINCO 按障碍处理。 |
| 内部 signed distance | $d=d_{occ}-d_{free}$；$d>0$ free，$d<0$ occupied。 |
| `/rc_esdf/signed_distance_grid` | RViz 编码：`-1` unknown，`0..49` 负距离，`50` 零距离，`51..100` 正距离；截断到 `2.0 m`。 |
| `/rc_esdf/footprint_clearance_grid` | P2 按 `0.70 x 0.55 m + 0.05 m` 外接圆生成的保守可视化；不能替代 runtime 定向矩形 gate。 |

运行时 MINCO footprint gate 使用 `0.70 x 0.55 m + 0.05 m` 的定向矩形。adapter 的 ego unknown 外接圆清理已默认设为 `0.0`；在实现带 yaw 的矩形栅格化前不得启用，避免把 footprint 外 unknown 改成 free。

### 2.3 自研规划控制的已完成能力

1. `minco_planner` 已能直接订阅 `goal_pose`；`global_plan_topic` 设为空后不会订阅 Nav2 `/plan`。
2. `onGoal()` 能从 TF 获取当前位姿，在最新 planning grid 上独立运行 JPS，失败时回退 A*，不需要 Smac 路径内容。
3. MINCO S3 生成连续位置、世界系 `vx/vy/ax/ay` 和时间戳。
4. `yaw_mode: clearance_aware` 保持平移与朝向解耦，支持舵轮横移。
5. yaw-aware footprint RC-ESDF 内点修正、最终矩形 gate 和可选 local repair 已接入。
6. `ats_swerve_mpc` 使用 `[vx, vy, wz]` 跟踪 MINCO reference，输出 `/cmd_vel_mpc`。

P3 已补齐自研 goal/action 状态机、Nav2-free launch 和不依赖 `NavigateToPose`/`/plan` 的回归。P2 仍可继续做 MINCO 直接数值 ESDF provider 与 source generation 结构化传播；这些优化不应替换当前 RC-ESDF 规划语义。

## 3. Nav2-free 目标运行链

### 3.1 最小可运行链

已实现的最小链为：

`MuJoCo sensors/localization -> ats_rog_map_node -> ats_rog_map_adapter -> /rc_esdf/planning_grid -> /goal_pose 或 /ats_navigate_to_pose -> ats_goal_manager -> /ats_goal_manager/planner_goal -> JPS/MINCO -> candidate reference/status -> ats_goal_manager -> 正式 reference + emergency_stop -> SE2 MPC -> /cmd_vel_mpc -> twist_to_motion_ctrl -> /motion_control`

运行时 `launch_nav2:=false` 会启动非 lifecycle 的 `static_map_publisher` 和 `ats_goal_manager`，而不会启动 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`、`velocity_smoother`、`map_server` 或 Nav2 lifecycle manager。launch 覆盖 MINCO 为 `goal_topic=""`、`global_plan_topic=""`、`goal_request_topic=/ats_goal_manager/planner_goal`、`planner_status_topic=/minco/planning_status`、`candidate_reference_path_topic=/minco/reference_path_candidate`、`planner_manages_emergency_stop=false`；故不订阅 `/plan`，也不由 MINCO 发布正式 reference 或急停。

`ROS_DOMAIN_ID=162/164/165` 的实际进程图同时验证 ATS action 存在、`/plan` 不存在，`/rc_esdf/planning_grid` 仅由 `ats_rog_map_adapter` 发布，`/cmd_vel_mpc` 为 `ats_swerve_mpc -> twist_to_motion_ctrl`，`/motion_control` 为 `twist_to_motion_ctrl -> ats_mujoco_sim` 的一对一链路。

### 3.2 自研目标管理

1. `/goal_pose` 已先通过目标管理器打通 Nav2-free 最小闭环；`ROS_DOMAIN_ID=129` 到达 `(-9.089719, 1.463896)`，误差 `0.089926 m`。它仅是兼容入口，正式任务入口为 `ats_navigation_interfaces/action/NavigateToPose`。
2. ATS action 支持 goal、feedback、result、cancel、preempt、timeout；结果码显式区分成功、取消、抢占、超时、地图未就绪、规划失败和 TF 失败。取消、抢占、超时、到达、TF/地图/规划失败均清空 active task、candidate reference 与旧授权，并发布急停。
3. `ats_goal_manager` 只负责任务生命周期、map heartbeat steady-clock lease、goal_id/candidate stamp 复核、reference 重定时和急停；JPS/MINCO 只规划，MPC 只跟踪。提交点在同一互斥区先发布 `emergency_stop=false`，再发布新的正式 reference；恢复 ready 本身不能复活旧 reference。
4. action 和 `/goal_pose` 都不调用 `nav2_msgs/action/NavigateToPose`；决策层应只调用 ATS action。

## 4. 最近验证记录

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

### 4.4 2026-07-16：ROGMap P2 地面规划闭环

源码与组件证据：

1. 新增 `ats_rog_map_interfaces/GetRogMapProjection` 与 `ats_rog_map_adapter`。adapter 直接调用数值 service，不订阅 `/rog_map/esdf`；service 输出来自概率地图、未膨胀的 raw 三态占据分类，避免与下游二维 RC-ESDF/footprint clearance 重复膨胀。
2. ROGMap wrapper 使用独立 source generation；概率地图 mutation、滑窗移动/reset 会推进 generation 并立即失效旧 ESDF generation。查询前只为当前 generation 重算 ESDF，数值结果仅在本次 ESDF updated bbox 内有效，bbox 外 distance/gradient 保持 `NaN`。地图健康由最后一次成功 mutation 与 odom freshness 判定，不能由原始 cloud 到达单独续命。
3. 有效 MuJoCo 参数位于受版本控制且由 launch 加载的 `ats_rog_map/config/rog_map_ground_planning_mujoco.yaml` 与 `ats_rog_map_adapter/config/rog_map_ground_planning.yaml`。`projection_min_height=0.10 m` 排除了当前场景地面回波；低矮障碍仍依赖 terrain/slope 语义，属于后续场景覆盖风险。
4. 融合真值表已由 GTest 固化：任一有效来源 occupied 保持 occupied；任一新鲜来源 known-free 可消解其他来源 unknown；只有所有来源都缺少 free/occupied 证据时输出 unknown，且 unknown 默认按障碍处理。静态细栅格使用输出 footprint 保守聚合，测试覆盖非整除分辨率、平移 origin 与 yaw；ego unknown 圆形清理默认禁用。
5. `planning_grid_owner:=rog_map` 会启动 adapter 并抑制 `rc_esdf_map`。运行时 `/rc_esdf/planning_grid` 只有 adapter 一个 publisher；`/cmd_vel_mpc` 与 `/motion_control` 也分别只有一个 publisher 和一个下游 subscriber。
6. MINCO 每次 planning-grid callback 构造不可变本地 snapshot，单次 JPS、二维 RC-ESDF、clearance、footprint gate 与 repair 不混用更新。最终 reference 在 snapshot/heartbeat 复核成功的提交点统一重定时，先发布 `emergency_stop=false`、再发布 reference；MPC 继续拒绝急停前的旧 reference。
7. `/planner/emergency_stop` 使用 reliable + transient-local QoS 和 `10 Hz` heartbeat；MPC 默认 fail-stop、使用 `0.5 s` steady-clock lease，并在首次建立权威、显式 stop 或 lease timeout 时清 tracker、deadline 与 warm start。ready 恢复本身不能重新授权旧轨迹。

最终统一 Release 构建为 `7 packages finished`。功能 CTest 为 `ats_rog_map 4/4`、`ats_rog_map_adapter 1/1`（测试进程内部 `7/7` GTest）、`minco_planner 5/5`、`ats_swerve_mpc 3/3`。`minco_planner` 全包测试仍有既有 `copyright`、`cpplint`、`clang_format` 失败，不能写成全包 lint 通过；本轮新增 reference timing 文件没有格式偏差。

最终 MuJoCo 名义路线均使用隔离 `ROS_DOMAIN_ID`、无 viewer/RViz 与 `PLANNING_GRID_OWNER=rog_map`。rectangle/red_box 使用 `P2_FAULT_CASE=none`；single 数据来自最终源码 `adapter_lease` 用例在注入故障前完成的健康名义段：

| 场景 | 终点与误差 | 路径/控制证据 | 离散 footprint 冲突 |
| --- | --- | --- | ---: |
| single | `(-9.076145, 1.464456)`，误差 `0.076347 m` | `/plan=59`，raw/reference `3/80`，MPC reference/predicted `21/21`，两级速度非零 | `0` |
| rectangle | 五段误差 `0.017822/0.019986/0.026784/0.028911/0.020937 m` | 五段 raw/reference 为 `3/56, 2/25, 2/13, 2/25, 3/17`；south/north 均有真实 `linear.y`；generation `298 -> 817` | 每段 `0` |
| red_box 中转 | `(-8.919291, 1.467177)`，误差 `0.039392 m` | `/plan=64`，raw/reference `3/86` | `0` |
| red_box 终点 | `(-0.069555, -4.092861)`，误差 `0.032232 m` | `/plan=128`，首次捕获 raw/reference `37/128`，最终有效规划日志 `16/572`，MPC reference/predicted `21/21`，两级速度非零，generation `304 -> 1133` | `0` |

最后一次源码修改后又完整运行 `ROS_DOMAIN_ID=203 scripts/test_mujoco_nav_chain.sh`，Nav2 action、lifecycle、TF、terrain/slope、RC-ESDF local elastic path 与速度桥均通过。该脚本不计算终点误差，只是 Nav2 回归对照，不能证明 P2 红框或 Nav2-free。

失效安全使用相互独立的 MuJoCo launch 与 ROS domain 执行；禁止将多个 fault 串在同一机器人状态中：

| `P2_FAULT_CASE` | 实际触发 | 结果 |
| --- | --- | --- |
| `adapter_lease` | 最终源码下 `ROS_DOMAIN_ID=204`，运动中 `SIGSTOP` adapter | heartbeat lease 触发急停与两级零速度；恢复后 generation `588 -> 701`，无新目标时旧 reference 不恢复运动 |
| `service_timeout` | `ROS_DOMAIN_ID=189`，运动中 `SIGSTOP` ROGMap | 记录 `projection request timed out`、ready=false、急停与双零；恢复后 generation `628 -> 675` 且仍双零 |
| `input_stale` | `ROS_DOMAIN_ID=191`，运动中 `SIGSTOP` MuJoCo sensor/localization 输入 | `/rog_map/stale=true`、ready=false、急停与双零；恢复后 generation `502 -> 540` 且仍双零 |
| `unknown` | `ROS_DOMAIN_ID=182`，从最终 planning grid 选取真实 unknown 单元 `(-14.525, -7.975)` | 日志 `goal is occupied`、急停与双零 |
| `unreachable` | `ROS_DOMAIN_ID=186`，动态选取 clearance-valid、值小于 `50` 且与起点不连通的 free 单元 `(10.175, -6.875)` | 日志 `no path` 而非 occupied-goal，急停与双零 |

验证边界：

1. `footprint_collisions=0` 只表示 MINCO 离散定向矩形 gate 未发现冲突采样。项目尚无独立 MuJoCo contact evaluator，因此物理接触次数为“未验证”，不能写成零碰撞。
2. 本节的 P2 红框脚本固定 `launch_nav2:=true`，因此只证明 Nav2 上游 + 自研规划控制；P3 Nav2-free 的独立证据见 4.6，二者不能互相替代。
3. ROG source generation 尚未通过 `OccupancyGrid` 结构化传播到 MINCO；MINCO 也尚未直接消费 adapter 数值 ESDF，而是从融合 planning grid 重建二维 RC-ESDF。
4. `/planner/emergency_stop` 与 `/minco/reference_path` 仍是两个独立 topic，不具备 DDS 跨 topic 原子事务；当前通过源端互斥、提交点重定时、MPC 旧 reference 拒绝和 lease fail-stop 限制风险。
5. 本轮没有在目标机复现技术报告的 `50 Hz`、约 `6 ms`、CPU 或峰值 RSS，也没有完成受控尾延迟基准；运行日志中的单次耗时不能替代性能验收。

### 4.5 2026-07-16：MuJoCo 入口与回归脚本复验

1. `ats_mujoco_sim.launch.py` 是裸底盘/传感器入口，默认回退到通用 `swerve_chassis.xml`，不启动 `map_server`、Nav2、ROGMap adapter、MINCO 或 MPC；其轻量 RViz 也没有 `/map` display。固定 RMUC 场地与导航验证必须使用 `ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py`。P2 手工闭环还需设置 `planning_grid_owner:=rog_map launch_swerve_mpc:=true`；在 Bash 中统一 source `install/setup.bash`。
2. 用户附件中的 single 回归实际上已经得到 `NavigateToPose=SUCCEEDED`、`/plan=59`、raw/reference `3/80` 和 `collisions=0`。原最终失败来自测试脚本调用环境中不存在的 `rg`，不是地图、规划或控制失败。两个 MuJoCo 回归脚本现已使用系统基础 `grep` 完成等价日志匹配，不再把 ripgrep 作为隐式运行依赖。
3. Nav2 对照脚本原先在 action 完成后才创建 `/local_elastic_path` 临时订阅，可能错过运动期唯一一次实质变化发布。脚本现于发送目标前预置 `reliable + transient_local` 捕获，并在 action 完成后验证 `poses` 非空。定向 `info` 运行记录 optimizer 输出 `35` 点、`published=true`、耗时 `0.007 s`；该单次日志只用于验证采样时序，不能作为性能指标。
4. 修复后在不含 `rg` 的精简 `PATH` 中运行 `ROS_DOMAIN_ID=211 TEST_PROFILE=single PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none`：终点 `(-9.062361, 1.466486)`，误差 `0.062460 m`；`/plan=59`、raw/reference `3/80`、MPC reference/predicted `21/21`、两级速度非零，generation `294 -> 466`，离散 footprint 冲突 `0`。
5. 同一最终脚本以 `ROS_DOMAIN_ID=212 TEST_PROFILE=red_box GOAL_TIMEOUT=180` 完成红框长路线：中转终点 `(-8.892180, 1.472841)`、误差 `0.012507 m`；红框终点 `(-0.114627, -4.070920)`、误差 `0.075178 m`。两段 action 均 `SUCCEEDED`，最终段 `/plan=128`、首次捕获 raw/reference `37/128`、最终有效规划 `16/512`、MPC reference/predicted `21/21`、两级速度非零，generation `296 -> 991`，离散 footprint 冲突 `0`。
6. `ROS_DOMAIN_ID=216 scripts/test_mujoco_nav_chain.sh` 完整通过固定地图、TF、terrain/slope、RC-ESDF 三类栅格、Nav2 lifecycle、非空 `35` 点 local elastic path、事件驱动 `/plan`、无空 FollowPath 和无 controller abort。该脚本仍是 Nav2 对照，不计算终点误差；物理接触仍未验证，以上结果也不证明 P3 Nav2-free。

### 4.6 2026-07-16：P3 Nav2-free 目标管理与启动链

源码、构建与接口证据：

1. 新增 `ats_navigation_interfaces` 的 ATS `NavigateToPose` action、`PlannerGoal`、`PlannerStatus`，以及 `ats_goal_manager`。前者不依赖 `nav2_msgs`；后者处理 `/goal_pose` 与 `/ats_navigate_to_pose`，将单调 `goal_id` 发给 MINCO，并独占正式 `/minco/reference_path` 与 `/planner/emergency_stop`。
2. MINCO 仅向 `/minco/reference_path_candidate` 与 `/minco/planning_status` 反馈候选；每个 planning-grid callback 仍构造一个不可变本地 snapshot，JPS、二维 RC-ESDF、MINCO clearance、footprint gate 和 repair 不跨 generation 混用。`clearance_aware` yaw 已对末端原地 yaw 过渡逐点复核 footprint gate；MPC 也能在零平移、yaw 变化的 tail segment 按 yaw 推进跟踪时间。
3. `launch_nav2:=false` 时 MuJoCo 使用 `static_map_publisher.py` 读取 RMUC YAML/PGM 并保持三态 OccupancyGrid、origin、分辨率、`map` frame、reliable/transient-local QoS；没有 `map_server` 或 lifecycle manager。RViz 配置已按当前 ROGMap/adapter、raw/reference/MPC 路径框架更新。
4. 最终 Release 构建为 `ats_navigation_interfaces`、`ats_goal_manager`、`ats_rog_map_adapter`、`minco_planner`、`ats_swerve_mpc`、`ats_mujoco_sim` 共 `6 packages finished`。`test_goal_lifecycle`、`test_ground_projection_fusion`、`test_yaw_spline_planner`、`test_trajectory_tracker` 与静态地图 Python 单测均通过；launch `--show-args`、改动 Python `py_compile`、两个回归脚本 `bash -n` 通过。`minco_planner` 全包 lint 的既有 `copyright`、`cpplint`、`clang_format` 失败仍不能写成全包测试通过。

所有 P3 名义路线均使用独立 ROS domain、`use_viewer:=false`、`show_viewer:=false`、`launch_mujoco_rviz:=false`、`NAVIGATION_MODE=p3`、`P3_GOAL_ENTRY=action`、`PLANNING_GRID_OWNER=rog_map` 与 `launch_nav2:=false`：

| 场景 | 终点与二维误差 | JPS/MINCO/MPC/底盘证据 | 离散 footprint 冲突 |
| --- | --- | --- | ---: |
| single（`ROS_DOMAIN_ID=162` 名义段） | `(-9.017683, 1.463644)`，`0.018791 m` | action feedback + `SUCCEEDED`；raw/reference `3/82`；MPC reference/predicted `21/21`；两级速度非零；adapter generation 名义段 `326 -> 493` | `0` |
| rectangle stage/east/south/west/north（`ROS_DOMAIN_ID=164`） | `(-9.502665,1.472583)`/`0.003711 m`；`(-8.852614,1.476912)`/`0.033339 m`；`(-8.817343,1.146728)`/`0.004215 m`；`(-9.500722,1.142227)`/`0.007806 m`；`(-9.500829,1.468654)`/`0.001581 m` | raw/reference 依次 `3/58`、`3/42`、`2/26`、`2/27`、`2/25`；south/north 实测非零 `linear.y`；MPC reference/predicted `21/21`、两级速度非零；generation `178 -> 696` | 每段 `0` |
| red_box 中转（`ROS_DOMAIN_ID=165`） | `(-8.947054,1.467880)`，`0.067088 m` | raw/reference `3/88`，action `SUCCEEDED` | `0` |
| red_box 最终目标 `(-0.04,-4.08)`（`ROS_DOMAIN_ID=165`） | `(-0.038637,-4.087151)`，`0.007280 m` | 首次捕获 raw/reference `37/128`，最终 MINCO reference `37/708`；MPC reference/predicted `21/21`、两级速度非零；generation `311 -> 989` | `0` |

故障均从新的 MuJoCo launch 和新的 ROS domain 注入，动作前若需要运动证据，先以已验证自由走廊 action 进入 tracking；故障恢复后均不发送新目标并再次采样双零输出：

| 故障 | 实际结果 |
| --- | --- |
| adapter lease（`155`） | 暂停 adapter 后 ready heartbeat lease 触发急停，`/cmd_vel_mpc` 与 `/motion_control` 为零；恢复 ready/generation 后无新目标仍为零。 |
| projection service timeout（`156`） | 暂停 ROGMap 后记录 `projection request timed out`、adapter not-ready、急停与双零；恢复后无旧 reference 复活。 |
| Point-LIO-compatible input stale（`157`） | 暂停 MuJoCo 输入后 `/rog_map/stale=true`、adapter not-ready、急停与双零；恢复后 generation `584 -> 616`，无新目标仍双零。 |
| all-unknown planning grid（`162`） | 仅隔离测试参数 `test_force_all_unknown=true`（默认 false）使 adapter 作为唯一 publisher 输出 all-unknown blocked grid；ready=false，正在 tracking 的 ATS action 返回 `RESULT_MAP_UNREADY=4`，双零；关闭参数后 generation `556 -> 726`，无新目标仍双零。 |
| free-unreachable（`163`） | 动态选取 `(10.175000,-6.875000)` 的 clearance-valid free 且不连通单元；ATS action 返回 `RESULT_PLANNING_FAILED=5`，MINCO 记录 no-path，双零。 |
| cancel（`148`） | Humble 通过 action `cancel_goal` service 精确 UUID 取消，结果码 `1`，有非空 cancel acknowledgement，急停与双零；恢复后仍双零。 |
| preempt（`151`） | 第一个 action 结果码 `2`，第二个 action 以超时结果码 `3` 结束，最终急停与双零；恢复后仍双零。 |
| timeout（`149`） | action 结果码 `3`，急停与双零；恢复后仍双零。 |
| TF failure（`152`） | 目标 frame 不存在，action 结果码 `6`，急停与双零。 |

`ROS_DOMAIN_ID=166 scripts/test_mujoco_nav_chain.sh` 也完整通过 Nav2 lifecycle、TF、terrain/slope、RC-ESDF、`NavigateToPose`、local elastic path 与速度桥；它是 Nav2 回归对照，不能作为 P3 通过证据。

验证边界：

1. `footprint_collisions=0` 仅表示 MINCO 离散定向矩形 gate 无冲突采样，项目仍没有独立 MuJoCo contact evaluator；物理 contact 为“未验证”，不能推导为零碰撞。
2. 专用 unsafe-trajectory 运行注入尚未实现；`publish_unsafe_trajectory=false`、MINCO failed status 到目标管理器急停已在源码中保留，但本轮没有把它单独作为 MuJoCo 故障证据。连续 swept footprint、实车动力学约束和实机验证属于 P4。
3. ROG source generation 尚未以结构化 OccupancyGrid/数值 ESDF 消息端到端传给 MINCO；当前只证明一次 MINCO 局部不可变 snapshot 内的一致性。MINCO 也尚未直接消费 adapter 数值 ESDF。
4. 本轮未在目标机测量技术报告的 `50 Hz`、约 `6 ms`、CPU、内存或尾延迟；不得写为 ATS 实测性能。

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

状态：`用户规定的 P2 最小实现范围本阶段完成`。已满足 ROGMap 数值查询、地面高度带、terrain/slope/static/unknown 融合、planning grid 唯一所有权、MINCO 本地不可变 snapshot、最终红框与指定失效安全回归，可以开始 P3 Nav2-free 目标管理；以下后续项仍不得写成已完成。

已完成范围：

1. `ats_rog_map_node` 提供同 source generation 的 raw occupancy、signed distance、unknown 与梯度 service；adapter 不反解析可视化点云。
2. adapter 融合 ROGMap、`traversability_grid`、`traversability_slope_grid` 与静态墙，unknown 默认按障碍处理；静态细栅格到 planning grid 使用保守 footprint 聚合并保留 origin/yaw。
3. `planning_grid_owner` 在 `rc_esdf|rog_map` 两种启动模式间选择唯一 publisher；不支持运行中热切换。
4. JPS、二维 RC-ESDF、MINCO clearance、footprint gate 与 Local Collision Repair 在单次规划中使用同一 MINCO immutable snapshot。
5. map ready heartbeat、projection request deadline/epoch、input stale、规划失败与 MPC tracker clear 已进入确定性急停链；最终 reference 使用提交点重定时，修复连续目标间被误判为急停前旧轨迹的问题。

P2 后续优化但不阻塞 P3 的范围：

1. 将 ROG source generation 与融合 grid/数值 ESDF 放入同一结构化消息并传播到 MINCO，替代当前本地 generation。
2. 实现 MINCO 直接数值 ESDF provider；当前仍从融合 planning grid 重建二维 RC-ESDF。
3. 将急停状态与 reference epoch 收敛为结构化原子安全契约；当前两个 topic 只有源端顺序与 lease 保护。
4. 增加 unsafe-trajectory 专用运行注入、连续 swept footprint 与独立 MuJoCo contact evaluator。
5. 在目标机验证完整 `2.5 cm` ESDF、频率、尾延迟、CPU 与峰值 RSS。

### P3：Nav2-free 启动与目标状态机

状态：`Nav2-free 主线已实现并在当前静态 MuJoCo 场景验证`；专用 unsafe-trajectory 运行注入仍未完成，不能把它写成已覆盖的安全验收。

1. 已新增正式 launch 模式并显式使用 `launch_nav2:=false`；MINCO 运行时 `global_plan_topic=""`，P3 图中不出现禁止的 Nav2 节点、Nav2 lifecycle manager 或 `/plan`。
2. 已先以 `/goal_pose` 贯通最小闭环，再以 ATS 自定义 action 完成正式 single、rectangle、red_box。action 覆盖 feedback、result、cancel、preempt、timeout 与 TF/map/planning 失败，并对所有终止状态执行安全停止。
3. 已把任务生命周期和正式 reference/急停交给目标管理器，JPS/MINCO 与 MPC 保持职责分离；P3 graph、planning grid、急停与速度/底盘输入的唯一所有权已实际检查。
4. 已独立注入 adapter lease、projection timeout、input stale、all-unknown、free-unreachable、cancel、preempt、timeout、TF failure，并在恢复后验证无新目标时双零。unsafe trajectory、连续 swept footprint、实车约束和独立 contact evaluator 转入 P4。

### P4：连续安全与舵轮执行约束

1. 为相邻 MINCO 时刻补连续 swept-volume 检查，覆盖窄门、贴边、纯横移和大 yaw 变化。
2. 依据实车轴距、轮距、最大轮速、最大舵角速度和反馈延迟标定 MPC 约束。
3. 完成控制 mux、急停优先级、地图/规划/MPC 健康检查后再灰度到实车。

## 6. 下一对话接续入口

下一对话从 P4 连续 swept footprint、实车舵轮执行约束和 P2 的结构化 generation/直接数值 ESDF provider 加固开始；不得重复实现 ROGMap/adapter，也不得把 `/rog_map/esdf` 调试点云作为规划距离场。不得通过放宽 unknown、frame、footprint 或 stale 安全门禁换取路线通过。

必须保持以下边界：

1. Point-LIO 继续提供 `/localization` 与 `/registered_scan`；不得用 ROGMap 替换里程计。
2. RC-ESDF、JPS、MINCO 与全向 SE2 MPC 继续保留；P2、P3 及后续运行中 `/rc_esdf/planning_grid` 始终只能有一个发布者。
3. 默认保留 `launch_fake_vel_transform:=True` 与 `launch_chassis_vel_transform:=True`。固定雷达迁移时可关闭开关；关闭 fake-yaw 仍保留零旋转兼容 TF，待所有 Nav2/行为参数改为固定 frame 后再删除该兼容层。
4. P3 已保留 P2 owner、heartbeat、snapshot 和急停契约；后续 P3 正式回归必须继续显式 `launch_nav2:=false`、使用 ATS action、拒绝 `/plan`，Nav2 `NavigateToPose` 只能作为独立对照。

本阶段复验命令：

```bash
MAKEFLAGS=-j1 colcon build \
  --base-paths src \
  --packages-select ats_rog_map_interfaces ats_rog_map ats_rog_map_adapter \
    minco_planner ats_swerve_mpc ats_nav_bringup ats_mujoco_sim \
  --parallel-workers 1
colcon test --base-paths src \
  --packages-select ats_rog_map ats_rog_map_adapter minco_planner ats_swerve_mpc
```

MuJoCo P1 smoke test 使用 `launch_nav2:=false launch_rog_map:=true`，并关闭 viewer、RViz、MPC 与速度 bridge；验收 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 非空、`frame_id=odom`、`/rog_map/stale=false`，且 `ats_rog_map` 不发布 TF。

## 7. 当前构建与回归入口

Nav2 对照和 P3 正式入口必须分开运行：

```bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select ats_navigation_interfaces ats_goal_manager ats_rog_map_adapter \
    minco_planner ats_swerve_mpc ats_mujoco_sim --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 仅 Nav2 对照：允许 NavigateToPose、/plan 与 lifecycle 节点。
ROS_DOMAIN_ID=166 scripts/test_mujoco_nav_chain.sh

# P3 正式：action 入口，无 Nav2 与 /plan。
ROS_DOMAIN_ID=165 NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action \
  PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
ROS_DOMAIN_ID=164 NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action \
  PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=none \
  TEST_PROFILE=rectangle GOAL_TIMEOUT=90 scripts/test_mujoco_minco_mpc_chain.sh

# P2/Nav2 兼容红框，仅作为地图与规划控制回归，不能替代上面的 P3 命令。
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  launch_nav2:=true \
  launch_swerve_mpc:=true \
  use_viewer:=false \
  show_viewer:=false \
  launch_mujoco_rviz:=false \
  lidar_backend:=cpu

PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=rectangle scripts/test_mujoco_minco_mpc_chain.sh
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh

domain=140
for fault in adapter_lease service_timeout input_stale unknown unreachable; do
  # 每次使用新的 ROS_DOMAIN_ID 与新的 MuJoCo launch，不能在同一进程内串行注入。
  ROS_DOMAIN_ID="${domain}" PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE="${fault}" \
    TEST_PROFILE=single scripts/test_mujoco_minco_mpc_chain.sh
  domain=$((domain + 1))
done
```

## 8. 维护约束

1. 后续提交统一在各仓库 `develop` 分支完成并推送 `origin/develop`。
2. 文档保留最近一周内当前决策所需的验证记录，方便回溯和阅读；超过一周的过期状态合并为当前结论，不保留无关历史流水账。
3. 参考目录不是运行时依赖。进入比赛链的源码、配置、消息定义和许可证信息必须受版本控制。
4. 修改地图语义时必须验证 frame、时间戳、分辨率、origin、unknown 和 signed distance；P3 的正式门禁是 Nav2-free rectangle 与 red_box，Nav2 红框只保留为对照。
5. 保留 Nav2 模式作为对照，直到自研链具备同等或更高覆盖；新功能优先落在自研链，禁止重新引入 Nav2 action 或 `/plan` 依赖。
