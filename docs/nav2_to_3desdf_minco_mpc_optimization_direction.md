# ATS 自研导航 V1 当前状态与下一阶段交接

更新时间：2026-07-21

有效更新窗口：2026-07-15 至 2026-07-21（滚动保留最近一周）。本文只保留当前有效架构、最近验证结果和下一阶段任务，方便回溯和连续阅读。MuJoCo 继续承担规划、控制、四舵轮动力学和 contact 的完整闭环；`loopback_sim` 只承担低成本行为决策、action 生命周期和任务时间线检查，不能替代 MuJoCo 验收。

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

`传感器时间同步 -> Point-LIO 局部连续里程计 -> small_gicp 全局重定位观测 -> 定位融合/健康状态/localization epoch -> ROGMap 概率地图/膨胀地图/3D ESDF -> 地面投影与 2.5D 可通行语义 -> RC-ESDF 规划接口 -> ATS 目标管理/action -> JPS -> MINCO S3 -> 独立 yaw -> footprint safety -> Local Collision Repair -> 全向 SE2 MPC -> twist_to_motion_ctrl -> 四舵轮底盘`

该箭头表示数据依赖和安全授权关系，不表示所有模块按目标串行调用。ATS Action Server 是 Goal Manager 的正式接口组成，不应再画成 Goal Manager 之前的第二个任务节点；ROGMap 与 adapter 持续异步更新地图，目标到达时 JPS/MINCO 只读取已 ready 的单次规划本地不可变 snapshot，Goal Manager 不负责同步调用建图。

架构约束：

1. ROGMap 是 V1 的建图与 ESDF 地图后端主线。
2. ROGMap 不是完整定位器。它消费外部里程计与点云，不估计机器人位姿；Point-LIO、重定位和后续定位融合必须独立提供 `map -> odom -> robot base`。
3. 自研比赛链不得依赖 Nav2 的 `NavigateToPose`、BT Navigator、planner server、controller server、costmap 或 `/plan` 才能运行。
4. 初期可继续复用 ROS 标准消息、TF2、RViz2 和 lifecycle，但不能把“使用 ROS2 基础设施”误写为“仍使用 Nav2 导航框架”。
5. 舵轮状态保持世界系 `[x, y, yaw]`，控制保持车体系 `[vx, vy, wz]`。禁止迁入 DDR 差速车的 ICR、曲率转向或 `vy=0` 约束。
6. `odom -> robot base` 必须保持局部连续，禁止用重定位结果直接覆盖；全局修正只允许作用于 `map -> odom`。任一时刻 `map -> odom` 只能有一个权威发布者。

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

## 1. 状态估计、里程计与重定位

### 1.1 当前实机活动链

当前实机链不能简化为“Point-LIO 直接发布 `odom -> base`”。活动源码与实机 launch 的实际数据流为：

```text
MID360 + IMU
  -> point_lio
     aft_mapped_to_init: camera_init -> body
     cloud_registered: camera_init
  -> loam_interface
     lidar_odometry: odom -> front_mid360
     registered_scan: odom
  -> sensor_scan_generation
     odometry: odom -> gimbal_yaw_odom
     TF: odom -> gimbal_yaw_odom
     TF: odom -> base_footprint

registered_scan + prior PCD + /initialpose
  -> small_gicp_relocalization
     relocalization_observation: map -> gimbal_yaw_odom
  -> localization_fusion
     /odometry -> /localization
     TF: map -> odom
     localization health + epoch
```

Point-LIO 负责局部连续激光惯性里程计；`loam_interface` 将 Point-LIO 的 `camera_init/body` 契约转换为导航使用的 `odom/front_mid360`；`sensor_scan_generation` 再结合雷达外参生成机器人基座里程计和 TF。`small_gicp_relocalization` 位于 `src/ats_sentry_nav/small_gicp_relocalization`，是活动导航仓中的正式重定位包，不属于 `参考/`，实机主入口默认 `launch_small_gicp_relocalization:=True`。

当前接口与唯一所有权如下：

| 契约 | 当前生产者 | frame / 时间语义 | 当前边界 |
| --- | --- | --- | --- |
| `aft_mapped_to_init`、`cloud_registered` | Point-LIO | `camera_init -> body`；使用雷达测量时间 | Point-LIO 不直接拥有 `map -> odom`。 |
| `lidar_odometry`、`registered_scan` | `loam_interface` | 转换到 `odom`；点云发布时选择时间最接近的里程计样本 | 最近样本匹配不是插值，实车高速运动下仍需验证时间误差。 |
| `/odometry` | 实机为 `sensor_scan_generation`；MuJoCo P4 为 `ats_mujoco_sim` | `odom -> gimbal_yaw_odom`；pose 连续，twist 为 child frame 车体系 | 定位融合的唯一局部里程计输入。 |
| `odom -> gimbal_yaw_odom`、`odom -> base_footprint` | `sensor_scan_generation` | 跟随 Point-LIO 里程计时间戳 | TF 查询失败不再回退单位变换；该帧不发布错误里程计。 |
| `relocalization_observation` | `small_gicp_relocalization` | 扫描原始时间的 `map -> gimbal_yaw_odom`，含协方差近似、质量、内点、误差与序号 | small_gicp 在 fusion 模式不再发布 `map -> odom`。 |
| `map -> odom` | `localization_fusion` | 用观测时刻 odom 历史插值计算；接受几何跳变时推进 epoch | 实机主入口默认启用 fusion；small_gicp TF 与静态 fallback 由 launch 互斥。 |
| `/localization` | `localization_fusion` | 保持连续的 `odom -> gimbal_yaw_odom` | ROGMap、Goal Manager 与 MPC 的统一状态入口。 |

因此，V1 实机 Nav2-free 入口已经由单一 fusion 节点显式统一 `/odometry -> /localization`。MuJoCo P4 运行图实际检查了两者的唯一 publisher，且 `ats_mujoco_sim.publish_map_to_odom_tf=false`、`localization_fusion.publish_tf=true`；实车接线仍需在目标机复核 topic 频率、时延与协方差统计。

### 1.2 `small_gicp_relocalization` 当前行为与限制

当前实现订阅 `registered_scan` 和 `/initialpose`，加载先验 PCD。GICP 结果先经过收敛、归一化误差、内点和两帧一致性确认，再发布 `RelocalizationObservation`；消息携带扫描原始时间、`map -> robot base`、信息矩阵逆与残差尺度得到的协方差近似、质量、内点数、源点数、归一化配准误差和单调序号。fusion 模式下 `publish_tf=false`，由 `localization_fusion` 独占 `map -> odom`。注册 timer、累计窗口和运动触发阈值仍只是配置上限，不是目标机实测频率。

实机配置使用 `min_inliers=200`、归一化 `max_registration_error=5.0` 和两帧确认；fusion 再复核质量、协方差有限性、观测序号、历史覆盖和最大创新。协方差与这些阈值尚未经过实车 rosbag 统计标定；当前也没有把修正渐进分摊到固定时间窗口。附件中的 Point-LIO `100 Hz~1 kHz`、small_gicp `0.5~1 Hz`、yaw `0.5 deg` 死区、位移 `0.3 m` 重规划阈值和 `0.5 s` 渐进修正均未在 ATS 目标机测量，不能写成实测结果。

fusion 保留按时间有序的 `/odometry` 历史，在观测时间插值 `odom -> robot base` 后计算 `map -> odom`；超出历史、间隔过大、迟到或重复的观测会被拒绝。TF 定时发布使用当前 ROS 时间加小幅 future offset，观测原始时间仍保留在状态消息中。

### 1.3 目标优化架构与 TF 所有权

最终优化方向是在保持局部里程计连续的前提下，将 small_gicp 从“直接改 TF 的唯一节点”演进为“带质量的全局重定位观测源”：

```text
Point-LIO / loam_interface / sensor_scan_generation
  -> 连续 odom -> robot base + 带时间戳的里程计历史
                                      \
                                       -> localization fusion
small_gicp + prior PCD                /      -> 唯一 map -> odom
  -> relocalization observation             -> localization health
     {stamp, pose, covariance,               -> localization epoch
      inliers, error, sequence}

localization health/epoch
  -> ROGMap adapter / Goal Manager / MINCO / MPC 安全门禁
```

目标契约如下：

1. small_gicp 观测必须携带原始测量时间、全局位姿、协方差或可解释质量、内点数、配准误差和单调序号；不得只靠一个无状态 TF 表示“重定位成功”。
2. 定位融合节点必须使用观测时刻的 Point-LIO 历史位姿计算 `map -> odom`，再外推到当前时刻；禁止把迟到观测直接套到最新 odom。
3. 当前 small_gicp 直接拥有 `map -> odom`；引入融合节点后，必须关闭 small_gicp TF 和静态 fallback，由融合节点成为唯一发布者。禁止三个来源同时发布同一 TF。
4. 不新增会破坏局部连续性的“平滑 odom”替换 Point-LIO。MPC 继续使用连续 `odom` 状态；如需全局平滑位姿，只作为诊断或任务层输出，不得形成第二条底盘状态权威。
5. 每次接受会改变规划几何关系的全局修正都推进 localization epoch。adapter 发布、Goal Manager 目标、MINCO snapshot/reference 和 MPC tracker 必须绑定该 epoch；旧 epoch 的地图、候选轨迹与 reference 一律拒绝。
6. 小修正是否低通、大修正阈值和修正时间常数必须通过 rosbag replay、MuJoCo 注入和目标机实测标定。未标定前采用保守策略：停止当前执行、失效旧 reference，等待新 epoch 地图 ready 后重规划。

localization epoch 与 ROG source generation、adapter publication epoch、MINCO local snapshot generation 是不同维度，禁止复用同一个数字或宣称天然一致。规划提交至少要同时校验“定位 epoch 未变”和“本次地图/轨迹 snapshot 仍有效”；只有结构化接口贯通并由 consumer 校验后，才能声称定位修正到轨迹执行的端到端版本一致。

### 1.4 重定位健康与安全状态机

定位健康至少区分 `UNINITIALIZED`、`TRACKING`、`RELOCALIZING`、`DEGRADED`、`LOST`。安全行为必须由 Goal Manager 统一收敛：

| 事件/状态 | 规划与执行行为 |
| --- | --- |
| `UNINITIALIZED`、`LOST`、定位 stale、TF 失败 | `emergency_stop=true`，清空 candidate/reference 和 MPC tracker，保证 `/cmd_vel_mpc`、`/motion_control` 为零。 |
| 正在验证重定位或接受显著全局修正 | 先急停并推进 localization epoch；旧地图 snapshot/reference 禁止继续执行。 |
| 单帧 GICP 失败或质量不足 | 拒绝该观测，不直接跳变 TF；连续失败或超时后进入 `DEGRADED/LOST`。 |
| 新 epoch 定位与地图恢复 ready，仍有活动目标 | 只允许 Goal Manager 重新触发 JPS/MINCO，收到同 epoch 安全 reference 后解除急停。 |
| 新 epoch 恢复但没有活动目标 | 保持急停和双零；不得恢复急停前 reference。 |

该状态机、结构化观测、定位融合、localization epoch 和重定位触发的自动重规划均已实现并通过节点测试与 MuJoCo 故障注入。[Confidence: High] 证据为 fusion/Goal Manager 节点测试和 5.7 的六个独立故障闭环；实车 rosbag、错误接受率、恢复时延分布和阈值标定仍未验证。

### 1.5 后续重定位验收门禁

1. 使用 rosbag 或可控仿真分别注入正常修正、迟到观测、乱序观测、单帧假匹配、连续配准失败、Point-LIO stale、TF 缺失、小修正和大跳变；每个用例使用独立进程或明确清理全部状态。
2. 验证 `map -> odom` publisher 始终唯一，`odom -> robot base` 在重定位前后不跳变；记录观测时间、处理时间、修正量、内点数、误差、定位状态和 localization epoch。
3. 大修正期间必须观测 `emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0`，旧 planning-grid snapshot、候选 reference、正式 reference 和 MPC warm-start 均失效。
4. 恢复后有活动目标时必须生成新 epoch 的 JPS raw path、MINCO reference 和 MPC 预测后才能运动；没有活动目标时保持双零。
5. 记录重定位接受率、错误接受率、恢复时间和端到端延迟，但在目标机完成受控测量前不得给出固定运行频率、滤波时间常数或跳变阈值。

## 2. ROGMap 参考来源与活动实现

### 2.1 本地来源与当前状态

上游参考源码保留在 `参考/src/rog_map`，只用于溯源与比较。活动实现位于 `src/ats_sentry_nav/ats_rog_map`，属于 `ats_sentry_nav` 独立 Git 仓库；构建和运行均不依赖 `参考/`。

活动包为 `ats_rog_map_node`，使用 LGPL-3.0-or-later，并随包保留 `LICENSE` 与 `NOTICE`。节点默认订阅 `/localization` 与 `/registered_scan`，默认运行 frame 为 `map_frame=odom`、`base_frame=gimbal_yaw_odom`、`sensor_frame=front_mid360`。`base_frame` 仅控制 Sliding Map 中心，`sensor_frame` 仅作为 raycasting 起点，不能再将云台传感器原点误当作机器人中心。

活动实现与参考源码已确认：

1. 参考 `rog_map` 原本只构建静态库；活动包已提供可直接 `ros2 run ats_rog_map ats_rog_map_node` 的 ROS 2 节点。
2. 内部保留概率占据、unknown、inflation 与真实 3D ESDF；P2 数值 service 直接查询 raw occupancy 与 3D ESDF，不经过调试点云。
3. 节点按点云时间戳查询 TF；已在 `odom` 的 `/registered_scan` 不会被再次按里程计变换。
4. 节点不发布机器人位姿 TF，只发布地图诊断、`/rog_map/stale` 并提供 `/rog_map/get_ground_projection`；`/rc_esdf/planning_grid` 由 adapter 发布。
5. 四类点云均是诊断/RViz 数据，禁止从 `/rog_map/esdf` 点云反解析为 MINCO 距离场。
6. 输入与调试点云默认 best-effort、depth 1；输入过期通过 `cloud_timeout_sec` 与 `odom_timeout_sec` 报告 `/rog_map/stale`。MuJoCo 因 ESDF 计算开销使用 `2.0 s` 超时，实车仍需按目标机负载复核。

### 2.2 ATS 侧目标接口

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

### 2.3 frame 与地图语义

目标 frame 约束：

1. `map` 是比赛全局规划 frame。
2. 当前实机由 `sensor_scan_generation` 发布 `/odometry` 及局部连续 TF，`small_gicp_relocalization` 发布结构化全局观测，`localization_fusion` 独占 `/localization` 与 `map -> odom`；不得增加第二个发布者。
3. ROGMap 必须按点云时间戳查询传感器到 `map` 的位姿，禁止只取“最近一帧 odom”而忽略时间同步。
4. 地面投影必须显式定义高度带、坡度阈值、悬空障碍、地面以下噪点和 unknown 策略。
5. ROG 数值 service 内 occupancy、distance、gradient 与 source generation 属于同一快照；adapter 发布的 `OccupancyGrid` 不携带 source generation，MINCO 当前以 callback 本地编号构造不可变 grid + 二维 RC-ESDF snapshot。

ROGMap 当前仍以 `odom` 维护局部滑动三维地图；adapter 将投影变换并融合到静态 `map` planning grid。MINCO 在该 grid 上规划后把 reference 通过 TF 转到 MPC 所需的 `odom`，TF 失败时拒绝发布并急停。

## 3. 当前 RC-ESDF、JPS、MINCO 与 MPC 基线

### 3.1 当前地图输入

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

### 3.2 当前 signed distance 语义

`RcTraversabilityEsdfProvider` 使用精确二维 signed Euclidean Distance Transform，不使用 `fake_costmap_esdf_provider` 作为运行时后端。

| 数据 | 当前定义 |
| --- | --- |
| `/rc_esdf/planning_grid` | `0` free，`1..49` 软风险，`50..100` occupied，`-1` unknown；unknown 对 JPS/MINCO 按障碍处理。 |
| 内部 signed distance | $d=d_{occ}-d_{free}$；$d>0$ free，$d<0$ occupied。 |
| `/rc_esdf/signed_distance_grid` | RViz 编码：`-1` unknown，`0..49` 负距离，`50` 零距离，`51..100` 正距离；截断到 `2.0 m`。 |
| `/rc_esdf/footprint_clearance_grid` | P2 按 `0.70 x 0.55 m + 0.05 m` 外接圆生成的保守可视化；不能替代 runtime 定向矩形 gate。 |

运行时 MINCO footprint gate 使用 `0.70 x 0.55 m + 0.05 m` 的定向矩形。adapter 的 ego unknown 外接圆清理已默认设为 `0.0`；在实现带 yaw 的矩形栅格化前不得启用，避免把 footprint 外 unknown 改成 free。

### 3.3 自研规划控制的已完成能力

1. `minco_planner` 已能直接订阅 `goal_pose`；`global_plan_topic` 设为空后不会订阅 Nav2 `/plan`。
2. `onGoal()` 能从 TF 获取当前位姿，在最新 planning grid 上独立运行 JPS，失败时回退 A*，不需要 Smac 路径内容。
3. MINCO S3 生成连续位置、世界系 `vx/vy/ax/ay` 和时间戳。
4. `yaw_mode: clearance_aware` 保持平移与朝向解耦，支持舵轮横移。
5. yaw-aware footprint RC-ESDF 内点修正、最终矩形 gate 和可选 local repair 已接入。
6. `ats_swerve_mpc` 使用 `[vx, vy, wz]` 跟踪 MINCO reference，逐轮约束合速度、轮速向量增量/加速度和运动中舵向变化率，输出 `/cmd_vel_mpc`。

P3 已补齐自研 goal/action 状态机、Nav2-free launch 和不依赖 `NavigateToPose`/`/plan` 的回归。P2 仍可继续做 MINCO 直接数值 ESDF provider 与 source generation 结构化传播；这些优化不应替换当前 RC-ESDF 规划语义。

## 4. Nav2-free 目标运行链

### 4.1 最小可运行链

已实现的最小链为：

`MuJoCo sensors/localization -> ats_rog_map_node -> ats_rog_map_adapter -> /rc_esdf/planning_grid -> /goal_pose 或 /ats_navigate_to_pose -> ats_goal_manager -> /ats_goal_manager/planner_goal -> JPS/MINCO -> candidate reference/status -> ats_goal_manager -> 正式 reference + emergency_stop -> SE2 MPC -> /cmd_vel_mpc -> twist_to_motion_ctrl -> /motion_control`

运行时 `launch_nav2:=false` 会启动非 lifecycle 的 `static_map_publisher` 和 `ats_goal_manager`，而不会启动 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`、`velocity_smoother`、`map_server` 或 Nav2 lifecycle manager。launch 覆盖 MINCO 为 `goal_topic=""`、`global_plan_topic=""`、`goal_request_topic=/ats_goal_manager/planner_goal`、`planner_status_topic=/minco/planning_status`、`candidate_reference_path_topic=/minco/reference_path_candidate`、`planner_manages_emergency_stop=false`；故不订阅 `/plan`，也不由 MINCO 发布正式 reference 或急停。

`ROS_DOMAIN_ID=162/164/165` 的实际进程图同时验证 ATS action 存在、`/plan` 不存在，`/rc_esdf/planning_grid` 仅由 `ats_rog_map_adapter` 发布，`/cmd_vel_mpc` 为 `ats_swerve_mpc -> twist_to_motion_ctrl`，`/motion_control` 为 `twist_to_motion_ctrl -> ats_mujoco_sim` 的一对一链路。

### 4.2 自研目标管理

1. `/goal_pose` 已先通过目标管理器打通 Nav2-free 最小闭环；`ROS_DOMAIN_ID=129` 到达 `(-9.089719, 1.463896)`，误差 `0.089926 m`。它仅是兼容入口，正式任务入口为 `ats_navigation_interfaces/action/NavigateToPose`。
2. ATS action 支持 goal、feedback、result、cancel、preempt、timeout；结果码显式区分成功、取消、抢占、超时、地图未就绪、规划失败和 TF 失败。取消、抢占、超时、到达、TF/地图/规划失败均清空 active task、candidate reference 与旧授权，并发布急停。
3. `ats_goal_manager` 只负责任务生命周期、map heartbeat steady-clock lease、goal_id/candidate stamp 复核、reference 重定时和急停；JPS/MINCO 只规划，MPC 只跟踪。提交点在同一互斥区先发布 `emergency_stop=false`，再发布新的正式 reference；恢复 ready 本身不能复活旧 reference。
4. action 和 `/goal_pose` 都不调用 `nav2_msgs/action/NavigateToPose`；决策层应只调用 ATS action。

## 5. 最近验证记录

### 5.1 2026-07-13

1. 扩大矩形回归 `TEST_PROFILE=rectangle` 五段均完成，south/north 检测到非零 `/cmd_vel_mpc.linear.y`，证明控制未退化为差速转向。
2. 当前实验链已验证 `/plan`、`/minco/raw_path`、`/minco/reference_path`、MPC reference/predicted path、`/cmd_vel_mpc`、`/motion_control` 和单一控制发布者/订阅者。

### 5.2 2026-07-14

1. `UsesYawAwareFootprintToIncreaseEdgeClearance` 单测确认 P2.1 后最小足迹净空大于 `0.39 m`，较中心候选提高超过 `0.05 m`，连续起终点不变。
2. `test_rc_esdf_map`、`test_grid_jps`、`test_minco_trajectory_optimizer`、`test_yaw_spline_planner` 全部通过。
3. 红框长路线 `TEST_PROFILE=red_box` 两段 Nav2 `NavigateToPose` 均为 `SUCCEEDED`：中转终点 `(-8.9181, 1.4698)`，误差 `0.038 m`；红框终点 `(-0.0845, -4.0685)`，误差 `0.046 m`。
4. 主段记录 `raw_points=39`、`reference_points=707`、`length=22.28 m`、`collisions=0`；MPC reference horizon、predicted path、`/cmd_vel_mpc`、`/motion_control` 录制均非空。

验证边界：以上结果证明当前静态 MuJoCo 场景中的 Nav2 上游 + 自研规划控制链可运行，不证明 ROGMap、自研 action、Nav2-free 启动、动态障碍、连续 swept volume 或实车安全已完成。

### 5.3 2026-07-15：ROGMap P1 与云台/底盘兼容链

1. `ats_rog_map`、`ats_nav_bringup`、`ats_sentry_bringup` 与 `ats_mujoco_sim` 单线程构建通过；`prob_map_log_odds_fusion_test`、`prob_map_stale_decay_test`、`test_split_robot_sensor_pose` 全部通过。
2. 在隔离 `ROS_DOMAIN_ID=88`、无 viewer/RViz、`launch_nav2:=false`、`launch_rog_map:=true` 的 MuJoCo 验收中，`/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 的宽度分别为 `2`、`239`、`9797`、`2601`，均为 `odom` frame；持续输入时 `/rog_map/stale=false`。
3. `ros2 node info /ats_rog_map` 只列出地图诊断话题，不含 `/tf` 或 `/tf_static` 发布者；ROGMap 不参与机器人 TF 所有权。
4. `fake_yaw` 与底盘坐标转换在实机主入口默认启用。四种开关组合均有明确速度出口：双开时维持 `cmd_vel_nav2_result -> fake -> cmd_vel_gimbal_yaw_odom -> chassis -> /cmd_vel`；仅底盘开时 chassis 直接订阅 `cmd_vel_nav2_result`；双关时 Nav2 直接输出 `/cmd_vel`；仅 fake 开时 fake 直接输出 `/cmd_vel`。实机不得在下游仍要求底盘坐标速度时仅关闭 chassis；该组合只适用于显式接管 gimbal-yaw 速度的外部执行器。
5. fake 关闭时启动 `gimbal_yaw_odom -> gimbal_yaw_fake` 的零旋转兼容 TF，使现有 Nav2 frame 参数不失效，但不会执行 fake-yaw 动态旋转。`base_footprint -> base_link` 静态 TF 改由内层 navigation launch 单一所有；启用 `robot_state_publisher` 时不再额外发布该静态 TF。

验证边界：本节只验证了当前稀疏 MuJoCo 点云、调试地图和开关路由。`rog_map_report_025.yaml` 尚未在目标机完成 50 Hz、约 6 ms、峰值 RSS 或完整 2.5 cm ESDF 性能验收；不得将技术报告数字写成 ATS 实测结果。

### 5.4 2026-07-16：ROGMap P2 地面规划闭环

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

1. 该批 2026-07-16 P2 结果运行时尚无独立 MuJoCo contact evaluator，`footprint_collisions=0` 只表示 MINCO 离散定向矩形 gate 未发现冲突采样；P4 evaluator 不能倒推这批历史结果的物理 contact。
2. 本节的 P2 红框脚本固定 `launch_nav2:=true`，因此只证明 Nav2 上游 + 自研规划控制；P3 Nav2-free 的独立证据见 4.6，二者不能互相替代。
3. ROG source generation 尚未通过 `OccupancyGrid` 结构化传播到 MINCO；MINCO 也尚未直接消费 adapter 数值 ESDF，而是从融合 planning grid 重建二维 RC-ESDF。
4. `/planner/emergency_stop` 与 `/minco/reference_path` 仍是两个独立 topic，不具备 DDS 跨 topic 原子事务；当前通过源端互斥、提交点重定时、MPC 旧 reference 拒绝和 lease fail-stop 限制风险。
5. 本轮没有在目标机复现技术报告的 `50 Hz`、约 `6 ms`、CPU 或峰值 RSS，也没有完成受控尾延迟基准；运行日志中的单次耗时不能替代性能验收。

### 5.5 2026-07-16：MuJoCo 入口与回归脚本复验

1. `ats_mujoco_sim.launch.py` 是裸底盘/传感器入口，默认回退到通用 `swerve_chassis.xml`，不启动 `map_server`、Nav2、ROGMap adapter、MINCO 或 MPC；其轻量 RViz 也没有 `/map` display。固定 RMUC 场地与导航验证必须使用 `ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py`。P2 手工闭环还需设置 `planning_grid_owner:=rog_map launch_swerve_mpc:=true`；在 Bash 中统一 source `install/setup.bash`。
2. 用户附件中的 single 回归实际上已经得到 `NavigateToPose=SUCCEEDED`、`/plan=59`、raw/reference `3/80` 和 `collisions=0`。原最终失败来自测试脚本调用环境中不存在的 `rg`，不是地图、规划或控制失败。两个 MuJoCo 回归脚本现已使用系统基础 `grep` 完成等价日志匹配，不再把 ripgrep 作为隐式运行依赖。
3. Nav2 对照脚本原先在 action 完成后才创建 `/local_elastic_path` 临时订阅，可能错过运动期唯一一次实质变化发布。脚本现于发送目标前预置 `reliable + transient_local` 捕获，并在 action 完成后验证 `poses` 非空。定向 `info` 运行记录 optimizer 输出 `35` 点、`published=true`、耗时 `0.007 s`；该单次日志只用于验证采样时序，不能作为性能指标。
4. 修复后在不含 `rg` 的精简 `PATH` 中运行 `ROS_DOMAIN_ID=211 TEST_PROFILE=single PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none`：终点 `(-9.062361, 1.466486)`，误差 `0.062460 m`；`/plan=59`、raw/reference `3/80`、MPC reference/predicted `21/21`、两级速度非零，generation `294 -> 466`，离散 footprint 冲突 `0`。
5. 同一最终脚本以 `ROS_DOMAIN_ID=212 TEST_PROFILE=red_box GOAL_TIMEOUT=180` 完成红框长路线：中转终点 `(-8.892180, 1.472841)`、误差 `0.012507 m`；红框终点 `(-0.114627, -4.070920)`、误差 `0.075178 m`。两段 action 均 `SUCCEEDED`，最终段 `/plan=128`、首次捕获 raw/reference `37/128`、最终有效规划 `16/512`、MPC reference/predicted `21/21`、两级速度非零，generation `296 -> 991`，离散 footprint 冲突 `0`。
6. `ROS_DOMAIN_ID=216 scripts/test_mujoco_nav_chain.sh` 完整通过固定地图、TF、terrain/slope、RC-ESDF 三类栅格、Nav2 lifecycle、非空 `35` 点 local elastic path、事件驱动 `/plan`、无空 FollowPath 和无 controller abort。该脚本仍是 Nav2 对照，不计算终点误差；物理接触仍未验证，以上结果也不证明 P3 Nav2-free。

### 5.6 2026-07-16：P3 Nav2-free 目标管理与启动链

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

1. 该批 2026-07-16 P3 结果运行时尚未接入独立 MuJoCo contact evaluator；`footprint_collisions=0` 只能表示 MINCO 离散定向矩形 gate 无冲突采样。P4 新 evaluator 与最终闭环结果见 5.7，不能倒推旧结果的物理 contact。
2. 专用 unsafe-trajectory 运行注入尚未实现；`publish_unsafe_trajectory=false`、MINCO failed status 到目标管理器急停已在源码中保留，但本轮没有把它单独作为 MuJoCo 故障证据。连续 swept footprint、实车动力学标定和实机验证继续属于 P4。
3. ROG source generation 尚未以结构化 OccupancyGrid/数值 ESDF 消息端到端传给 MINCO；当前只证明一次 MINCO 局部不可变 snapshot 内的一致性。MINCO 也尚未直接消费 adapter 数值 ESDF。
4. 本轮未在目标机测量技术报告的 `50 Hz`、约 `6 ms`、CPU、内存或尾延迟；不得写为 ATS 实测性能。

### 5.7 2026-07-17：P4 第一阶段定位融合与四舵轮执行约束

源码与接口：

1. `RelocalizationObservation` 已携带扫描时间、`map -> robot base` 位姿、协方差近似、内点/源点数、归一化配准误差、质量、状态与单调序号。small_gicp 增加两帧一致性确认；协方差来自最终信息矩阵逆与残差尺度，尚未经过实车统计标定。
2. `localization_fusion` 明确执行 `/odometry -> /localization`，保持 `odom -> gimbal_yaw_odom` 局部连续；用观测时刻 odom 历史插值计算 `map -> odom`，拒绝迟到、重复、历史越界、质量不足和超创新观测，并发布 `UNINITIALIZED/TRACKING/RELOCALIZING/DEGRADED/LOST` 与 localization epoch。
3. adapter、Goal Manager、MINCO 与 MPC 已校验 localization epoch。定位失效或 epoch 变化会失效 planning snapshot、候选/正式 reference、MPC tracker 与 warm start；恢复时必须先得到同 epoch 新地图，再派发同 epoch planner goal 和新 reference。MPC 还会拒绝定位非 `TRACKING` 期间到达的新 Path，单独的 `emergency_stop=false` 不能解除定位 fail-stop。
4. P4 MuJoCo 图中 `/odometry` 唯一链为 `ats_mujoco_sim -> localization_fusion`，`/localization`、`/localization/status` 和 `map -> odom` 由 fusion 独占；MuJoCo 的 `publish_map_to_odom_tf=false`。`/planner/emergency_stop` 只有 Goal Manager 一个 publisher，MPC 与 MuJoCo 分别执行 tracker 清理和执行器硬零速。

四舵轮几何与执行器语义：

1. `wheel_base_x/y=270 mm` 按底盘中心到轮心的半轴偏置解释，因此完整前后和左右轮心跨度均为 `0.540 m`。这是由现有碰撞包络和模型轮位交叉确认的源码语义，不再二次除以二。
2. `wheel_radius=42.5 mm` 是包含 `10 mm` 包胶后的最终滚动半径，模型未叠加到 `52.5 mm`。Shore A 60 未无依据换算为 `solref/solimp`。
3. 三个 swerve 模型均为根部上装 `23 kg` 加四个 `0.5 kg` 模块，MuJoCo 编译模型总质量 `25 kg`、整车总质心高度 `0.100 m`。轮胎切向摩擦为 `0.8`，扭转/滚动分量保持独立的 `0.02/0.005`，没有把 `0.8` 复制到三个摩擦分量。
4. 减速比按“电机转速/轮端转速”解释。`450 rpm`、`42.5 mm`、`i_drive=1` 的原始轮缘速度为 `2.002765 m/s`，除以 `1.2` 冗余后为 `1.668971 m/s`，对应保守轮端 `375 rpm`。`120 rpm`、`i_steer=1` 的原始舵速为 `12.566371 rad/s`，保守舵速为 `10.471976 rad/s`。
5. MPC 与 MuJoCo 均按每轮 $v_i=[v_x-\omega_z y_i,\ v_y+\omega_z x_i]$ 计算，约束每轮合速度、驱动 RPM、轮速向量增量/加速度和运动中舵向变化率；状态仍为世界系 `[x,y,yaw]`，控制仍为车体系 `[vx,vy,wz]`。

构建与测试：

1. Release 构建 `ats_navigation_interfaces`、`small_gicp_relocalization`、`sensor_scan_generation`、`ats_goal_manager`、`ats_rog_map_adapter`、`minco_planner`、`ats_swerve_mpc`、`ats_nav_bringup`、`ats_sentry_bringup`、`ats_mujoco_sim` 共十包通过。
2. `small_gicp_relocalization 42/42`、Goal Manager `5/5`、MPC `18/18`、adapter `8/8`、MINCO 五个功能 GTest 和 MuJoCo physics `6/6` 通过。四个包仍有既有全包 copyright/Black/cpplint 债务；MINCO 功能 GTest 已从 lint 中单独重跑，不能写成四个包 lint 全通过。
3. 改动 Python `py_compile`、两个 evaluator 的 Black/`ament_flake8`、YAML/XML 解析、脚本 `bash -n`、四个 launch `--show-args` 与三仓 `git diff --check` 均通过。
4. `ROS_DOMAIN_ID=215 scripts/test_mujoco_nav_chain.sh` 完整通过 Nav2 lifecycle、`NavigateToPose=SUCCEEDED`、RC-ESDF local elastic path 和底盘转发；该结果只作为 Nav2 对照。

`ROS_DOMAIN_ID=214` 四舵轮动力学矩阵覆盖前进、横移、斜向、纯旋转、组合、加减速、急停和舵向反转，`failures=[]`、违规 contact sample 累计为 `0`。以下 saturation 为该阶段结束时的累计计数，不是单阶段增量：

| 阶段 | 实测底盘峰值 | 轮/舵与滑移证据 | 累计饱和 |
| --- | --- | --- | --- |
| 前进 `[0.5,0,0]` | `vx=0.506308 m/s` | `111.255 rpm`；稳定纵滑 P95 `0.011191 m/s` | drive accel `200`，steer `0` |
| 横移 `[0,0.5,0]` | `vy=0.506533 m/s` | `111.295 rpm`；稳定纵/侧滑 P95 `0.011191/0.000000 m/s` | drive accel `228`，steer `96` |
| 斜向 `[0.35,0.35,0]` | `vx/vy=0.354632/0.354615 m/s` | `110.186 rpm`；稳定纵滑 P95 `0.011079 m/s` | drive accel `228`，steer `20` |
| 纯旋转 `[0,0,0.8]` | `wz=0.796517 rad/s` | `67.379 rpm`；舵速达到保守上限 `10.471976 rad/s` | drive accel `132`，steer `60` |
| 组合 `[0.3,0.2,0.5]` | `0.307571/0.201065/0.510110` | `122.009 rpm`；稳定纵/侧滑 P95 `0.019475/0.003797 m/s` | drive accel `184`，steer `40` |
| 加速 `[1.5,0,0]` | `vx=1.519071 m/s` | `333.766 rpm`；稳定纵滑 P95 `0.033628 m/s` | drive accel `864`，steer `35` |
| 舵向反转 | `vy: +0.404548 -> -0.365481 m/s` | 舵速限幅 `10.471976 rad/s`；瞬态侧滑峰值 `0.471973 m/s` | drive accel `670`，steer `649` |
| 急停 | 最终 `[0,0,0]` | 最终四轮 `[0,0,0,0] rpm` | contact `0` |

六类定位故障均使用新的 MuJoCo launch/domain，故障期观测急停、`/cmd_vel_mpc=0`、`/motion_control=0` 和四轮归零；恢复后必须先发布新 map/planner goal/reference，action 才继续：

| 故障/domain | epoch | 最终二维误差 | 恢复与物理结果 |
| --- | ---: | ---: | --- |
| Point-LIO-compatible odometry stale（`207`） | `1 -> 1` | `0.061098 m` | reference `1 -> 2`，四轮 `0 rpm`，contact `0` |
| 迟到/乱序观测（`208`） | `1 -> 1` | `0.056659 m` | reference `1 -> 2`，四轮 `0 rpm`，contact `0` |
| GICP 拒绝（`209`） | `1 -> 1` | `0.054846 m` | reference `1 -> 2`，四轮 `0 rpm`，contact `0` |
| 3 m 假匹配（`210`） | `1 -> 1` | `0.070018 m` | 创新门限拒绝；reference `1 -> 2`，contact `0` |
| `map -> odom` 跳变（`211`） | `1 -> 2` | `0.065180 m` | planner goal `2 -> 3`、reference `1 -> 2`，contact `0` |
| TF 丢失/恢复（`212`） | `1 -> 1` | `0.059394 m` | planner goal `2 -> 3`、reference `1 -> 2`，contact `0` |

最终 P4 fusion + Nav2-free 名义闭环均使用 `launch_nav2:=false`、ATS action、ROGMap owner、无 viewer/RViz，且检查了 planning grid、急停、reference、速度和底盘输入的唯一 publisher：

| 场景/domain | 最终结果 | 规划与执行证据 | 物理 contact |
| --- | --- | --- | ---: |
| single（`218`） | action `(-9.078995,1.465139)`/`0.079144 m`；独立采样 `0.060029 m` | raw/reference `3/81`，footprint 冲突 `0`，generation `380 -> 612` | `0` |
| rectangle（`219`） | stage/east/south/west/north action 误差依次 `0.069776/0.046826/0.001796/0.065216/0.000433 m`；最终独立误差 `0.002587 m` | south/north 最大 $|linear.y|$ 为 `0.471399/0.186684 m/s`，每段 footprint 冲突 `0`，generation `451 -> 1094` | `0` |
| red_box（`220`） | 中转误差 `0.062494 m`；目标 `(-0.04,-4.08)` action `(-0.033339,-4.080146)`/`0.006663 m`，独立采样 `0.006405 m` | 最终 raw `37` 点、reference `708` 点、footprint 冲突 `0`，generation `445 -> 1124` | `0` |

当前边界：连续 swept footprint、unsafe-trajectory 专用注入、small_gicp 协方差/质量的实车统计标定、轮端电流/反馈延迟和 `2.0 m/s^2` 加速度约束的实测标定、Shore A 材料接触模型、控制 mux 与实车灰度均未完成。MuJoCo contact evaluator 把正常轮地接触排除，只累计机器人与非地面几何体或底盘触地的 contact sample；它不是实车碰撞安全证明。

### 5.8 2026-07-17：P4 第二阶段连续安全与执行契约（组件证据）

已实现并通过组件测试的范围：

1. `FootprintSafetyChecker` 保持既有 `0.70 x 0.55 m + margin` 定向矩形栅格语义，在相邻 reference SE(2) 段上新增 swept 检查。分段数由四个最远角点的实际位移除以 `planning_grid.resolution * swept_max_corner_step_cells` 向上取整；默认上限为半个栅格。yaw 使用最短角插值，故跨 `-pi/pi` 不会绕完整圆周。此实现是保守离散 swept 采样，不是解析连续多边形 Minkowski 和；其保守性依赖于栅格分辨率、矩形采样和半格步长。
2. checker 与 Local Collision Repair 已按 `OccupancyGrid.info.origin` 的平移和 yaw 在世界/栅格坐标间变换。`test_footprint_swept_safety` 覆盖细墙、纯横移、斜切、纯旋转角点、yaw wrap、unknown/outside、非整除分辨率/平移/旋转 origin，以及 repair 后最终 swept 复核；`test_planning_map_snapshot` 继续锁定 immutable snapshot 基础契约。
3. MINCO 的 center、footprint-aware、fallback、repair 后 candidate 和最终提交前轨迹均使用同一 snapshot 的 checker。提交后，MINCO 按短视域重新检查剩余 reference 对最新 immutable snapshot 的 swept 安全性；发现 unsafe 时只发布结构化 `PlannerStatus::FAILURE_RUNTIME_UNSAFE`，由 Goal Manager 停止并重规划，不由 MINCO 越权发布 P3 正式急停/reference。
4. 新增 `ExecutionCommand` 作为 Goal Manager 到 MPC 的唯一执行授权，原子携带 `STOP/EXECUTE`、reference、goal id、localization epoch、MINCO local snapshot generation、adapter publication sequence、failure enum 和严格单调命令序号。MPC 对执行命令作 lease、序号、mode、epoch 和 frame/path 校验；旧 `Path` 与 `emergency_stop=false` 不能重新授权 tracker。`/planner/emergency_stop` 与 `/minco/reference_path` 仅保留为诊断兼容输出，不再消除 DDS 跨 topic 顺序风险的依据。
5. `PlannerStatus.reason` 已替换为稳定的 `failure_reason` 枚举。Goal Manager 的节点 pytest 覆盖旧 localization epoch 和旧 adapter publication sequence reference 拒绝；MPC GTest 覆盖结构化 stop 后旧 sequence 与旧 epoch execute 不得复活。adapter 增加默认关闭的运行期障碍覆盖参数，供“地图提交后变障碍”注入，不使用 `/rog_map/esdf` 可视化点云。

本轮实际通过的命令为 Release 构建 `ats_navigation_interfaces`、`ats_rog_map_adapter`、`minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`、`ats_mujoco_sim`，以及上述 MINCO、Goal Manager、MPC、adapter 聚焦 CTest。`minco_planner` 全包 `copyright/cpplint/clang_format` 仍有既有失败，不能称全包 lint 通过。

本轮运行验证（均关闭 viewer/RViz、显式 `launch_nav2:=false`、`P4_LOCALIZATION_FUSION=true`、`PLANNING_GRID_OWNER=rog_map`）：

1. `ROS_DOMAIN_ID=226` single action 到达 `(-9.032020,1.465134)`，二维误差 `0.032388 m`，MINCO `raw/reference=3/81`、离散+swept footprint 冲突 `0`、MPC reference/predicted path 非空、终态四轮低于 `2 rpm`、MuJoCo contact `0`。过程图验证 `/planner/execution_command` 为 `ats_goal_manager -> ats_swerve_mpc` 的唯一发布/订阅授权链，`/cmd_vel_mpc` 和 `/motion_control` 仍各自一对一。
2. `ROS_DOMAIN_ID=214` rectangle 五段 action 误差依次为 `0.041995/0.031296/0.005092/0.064109/0.005942 m`；south/north 都观察到真实 `linear.y`，每段 footprint 冲突 `0`，最终四轮低于 `2 rpm`、contact `0`。
3. `ROS_DOMAIN_ID=217` red_box 中转误差 `0.015724 m`；终点 `(-0.04,-4.08)` 实测 `(-0.037221,-4.076565)`、误差 `0.004419 m`。最终段 MINCO `raw/reference=37/708`、冲突 `0`，两条 MPC debug path、两级速度均非空，终态四轮低于 `2 rpm`、contact `0`。
4. 新增 unsafe evaluator 各用独立 launch/domain。`map_after_commit`（`229`）、`pure_rotation`（`230`）、`unknown`（`231`）、`outside`（`232`）、`mid_segment`（`219`）、`old_generation`（`220`）、`repair_after_unsafe`（`222`）均进入确定性停止；前六个 runtime/map 用例记录旧 `ExecutionCommand` 序号和 map generation/publication sequence，最终四轮均 `0 rpm`、contact `0`。动态障碍用例要求 `FAILURE_RUNTIME_UNSAFE` 后 Goal Manager 发送 stop；unknown 由 adapter 唯一发布 all-unknown blocked grid；outside-map action 返回 `RESULT_PLANNING_FAILED=5`。`repair_after_unsafe` 的 Local Collision Repair 几何重算和最终 swept gate 由 `test_footprint_swept_safety` 覆盖，MuJoCo 用例验证提交后变障碍时最终 reference 不继续执行，不把它写成解析连续碰撞证明。
5. 结构化旧版本恢复：`epoch_jump`（`211`）从 epoch `1 -> 2`，reference `1 -> 2`、planner goal `1 -> 2`，终点误差 `0.058543 m`；`tf_loss`（`212`）reference `1 -> 2`、planner goal `2 -> 3`，终点误差 `0.056650 m`。两项均急停、两级零速度、四轮 `0 rpm`、contact `0`，恢复只接受新版本 reference。`odometry_stale` 的本轮补跑在初始 `TRACKING` 前超时，未进入故障注入；迟到/乱序、GICP 拒绝和假匹配保留 5.7 的第一阶段证据，未在本轮最终脚本环境重跑。[Confidence: Medium]。
6. `ROS_DOMAIN_ID=218 scripts/test_mujoco_nav_chain.sh` 仍通过 Nav2 `NavigateToPose=SUCCEEDED`、RC-ESDF local elastic path 与速度桥，只是对照组；`ROS_DOMAIN_ID=221 scripts/test_mujoco_swerve_dynamics.sh` 的 `failures=[]`，紧急停止终态四轮为 `0 rpm`、contact `0`。

验证边界：swept 实现仍是半栅格最远角点位移约束下的保守栅格采样，不是解析连续 Minkowski sweep；MuJoCo contact evaluator 不是实车碰撞证明。实车只新增 `docs/p4_real_robot_calibration_preflight.md` 的采集与停止门禁，未进行通电或运动。

### 5.9 2026-07-20：P4 第三阶段转弯/yaw authority/fake-yaw（部分实现与验证）

本节只记录本轮源码、测试和独立 MuJoCo 运行证据；它不是 P4 第三阶段的完整验收结论。

**已实现和已测试的契约**

1. `ExecutionCommand` 增加 `yaw_authority`、`requires_gimbal_lock`、`gimbal_request_sequence` 与 `gimbal_feedback_sequence`；新增 `YawAuthorityRequest` 和 `GimbalYawStatus`。三个 authority 常量为 `HOLD_SAFE_STOP=0`、`GIMBAL_COMPENSATED=1`、`BODY_YAW_FOLLOW=2`。Goal Manager 仍是唯一 `STOP/EXECUTE` 授权者；MPC 不接受 legacy `Path` 或 `emergency_stop=false` 重新授权。
2. 每一个 candidate reference 都先由 Goal Manager 发布包含 `goal_id`、localization epoch、MINCO local snapshot generation、adapter publication sequence、authority 和 lock 要求的 `YawAuthorityRequest`。只有收到同一 request sequence 的新鲜、TF 健康反馈后才提交 execute。MPC 连续复核 feedback lease、authority、lock、request sequence 和 feedback sequence；任一失配清 tracker/warm start 并输出零速度。
3. authority 改变不允许运动中热切换：Goal Manager 先发布结构化 `STOP`、清 active command/pending request，并回到 planning；新 candidate 经新的 request/ack 后才回到 tracking。节点 pytest 已复现并修复“STOP 后 lifecycle 仍为 tracking、正确锁定确认却不能重新 execute”的状态机错误。
4. MINCO 将 immutable reference 的最小有限定向 footprint clearance 用于整段保守 authority 选择：`force_body_yaw_follow=true` 必选 `BODY_YAW_FOLLOW`；否则任一点 clearance 不高于 `body_yaw_follow_clearance`（默认 `0.55 m`）即选 body。当前 planning grid 不携带坡道来源标签，因此坡道/接触敏感 route 只能由 profile 显式 force body，不能写成已自动识别坡道。
5. `fake_vel_transform` 修正为首个有效 odometry yaw 初始化 compatibility frame：令 $\psi_0$ 为初始 gimbal/body yaw、$\psi$ 为当前 yaw，则 `gimbal_yaw_odom -> gimbal_yaw_fake` 为 $\psi_0-\psi$，fake velocity 回 gimbal velocity 为 $R(\psi-\psi_0)$。这保留非零初始摆放 yaw，且速度旋转为 TF 的逆变换；fake frame 不覆盖真实 world-state body yaw。P3/MPC 主链仍关闭 legacy fake velocity adapter，避免二次旋转 `/cmd_vel_mpc`；兼容 TF 未删除。
6. MuJoCo 订阅 yaw request 并以 transient-local/reliable `/gimbal/yaw_status` 回显 authority、lock、request sequence、body/gimbal/fake yaw。其 status QoS 已与 Goal Manager/MPC 匹配。当前模型的 MID360 固定在 `base_link`，所以只能验证 request/ack、lease 和 fail-stop，**不能**验证真实云台旋转的点云位移、地图匹配漂移或 fake-yaw 传感器补偿保真。

**独立证据**

1. Release 构建通过：`ats_navigation_interfaces`、`fake_vel_transform`、`minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`、`ats_mujoco_sim`。聚焦测试通过：`test_fake_yaw_math`（非零初始 yaw、跨 $\pm\pi$、TF/速度逆变换）、`test_yaw_authority_policy`（开阔/窄道/force profile）、Goal Manager lifecycle/epoch pytest（错误 request、未锁定 body、authority STOP/re-ack、旧 epoch/reference 不复活）、MPC `test_se2_mpc_controller`、`test_trajectory_tracker` 和 `test_mpc_localization_gate`（world/body 变换、yaw wrap、旧 sequence/epoch、gimbal stale/错误 request fail-stop），以及 MuJoCo `test_swerve_physics` 6 项。
2. 静态审计和 tracker 单测共同证明 SE2 模型仍按 $R(yaw)[v_x,v_y]^T$ 将车体系控制投影到世界系状态，MINCO 世界速度按参考 yaw 变为 body control，yaw error 使用 $\operatorname{atan2}(\sin\Delta\psi,\cos\Delta\psi)$。故“world velocity 被直接当作 body control”与“yaw wrap 缺失”不是本轮首个已证实根因。[Confidence: High，源码加针对性测试；尚非实车动力学证据]
3. `ROS_DOMAIN_ID=36`、`launch_nav2=false`、`P4_LOCALIZATION_FUSION=true`、`planning_grid_owner=rog_map`、`force_body_yaw_follow=true` 的 single action 已通过。execute capture 为 `yaw_authority=2`、`requires_gimbal_lock=true`、`request_sequence=1`、`feedback_sequence=834`；终点 `(-9.017584,1.464537)` 对 `(-9.0,1.47)` 位置误差 `0.018413 m`，MINCO `raw/reference=3/81`、离散 footprint 冲突 `0`、contact violation `0`、最终四轮 `0 rpm`。终端 action 成功由 position/yaw、body linear/angular velocity 与 `0.30 s` dwell 共同门控，不再仅凭 pose。
4. `ROS_DOMAIN_ID=38` 自动策略 single action 通过，终点误差 `0.002574 m`、最终四轮 `0 rpm`、contact `0`。MINCO 记录 `minimum_clearance=0.160 m < 0.55 m`，因而 execute 合理为 body（`yaw_authority=2`、locked）。此前 `ROS_DOMAIN_ID=37` 强制期待 gimbal 的断言失败，但 action 自身成功；capture 同样显示 body。这是实测 clearance 分类，不是调低 MPC 权重或云台状态机故障。
5. `ROS_DOMAIN_ID=34` 的自动 single（authority capture 加入前）闭环成功：终点误差 `0.009832 m`、`raw/reference=3/81`、离散 footprint `0`、contact `0`、最终四轮 `0 rpm`。`ROS_DOMAIN_ID=35` 在 action 前因一次 ros2cli pose discovery 未返回而退出，未产生 execute；脚本已把 pose capture 改为三次有界重试。`ROS_DOMAIN_ID=39` 在 fusion 后的无 timeout `ros2 topic info` 查询挂起且未进入 action，已精确终止；脚本已改为 `timeout 5`，该次不作为控制失败或 gimbal 结果。
6. 回归脚本随后将所有 event-driven topic capture 改为显式 ROS message type、启动前存活检查和有界 `TERM/KILL/reap`，避免 `ros2 topic echo` 的 graph type discovery 或退出后的无界 `wait` 被误判为路径/控制故障。最终脚本下 `ROS_DOMAIN_ID=50` single action 到达 `(-9.003661,1.467067)`，误差 `0.004691 m`；`raw/reference=3/81`，execute 为 body、`request/feedback=1/979`，MPC reference/predicted 与两级速度非空，终态四轮 `0 rpm`、contact `0`。
7. `ROS_DOMAIN_ID=48/49` red_box 的目标 action 均实际到达 `(-0.04,-4.08)` 邻域，最终 action 位姿分别为 `(-0.038428,-4.081950)`/`0.002504 m` 和 `(-0.041480,-4.078687)`/`0.001979 m`，第二段均生成 `37/708` raw/reference。但这两次在 action 成功后的脚本 debug/telemetry type discovery 处退出；因此它们是终端到达证据，不是本轮“完整 red_box 回归通过”证据。最后一次脚本修复后仅完成 single，rectangle/red_box 仍须完整重跑。
8. 最终脚本下 `ROS_DOMAIN_ID=51` red_box 完整通过：最终 action 位姿 `(-0.038022,-4.080458)`、位置误差 `0.002031 m`，第二段 `raw/reference=37/708`、reference duration `53.08 s`、minimum clearance `0.094 m`、footprint collision `0`；终态四轮 `0 rpm`、实测 body velocity `(-0.00306,-0.00067,0.00298)`、contact `0`。累计 drive-acceleration/steer-rate saturation 为 `29234/12840`，该累计量证明约束在运行中频繁介入，但当前没有分段时序和占空比，不能直接归因于转弯误差或据此放宽限值。

**未完成、不可宣称的范围**

1. 尚未在实测 clearance 大于默认 `0.55 m` 的开阔 route 完成 `GIMBAL_COMPENSATED` MuJoCo action；`0.10 m` 阈值敏感性运行因 ros2cli graph 查询挂起而未进入 action。单元测试覆盖开阔 clearance 的 gimbal 分支，但不是旋转云台物理闭环。
2. 本轮未完成 90 度转弯、S 弯、保持 yaw 横移、窄道、坡道/起伏专用 route、mode switch while moving、gimbal feedback stale/TF/map stale/旧 generation 故障矩阵；最终脚本已完整重跑 single 与 red_box，但最后一次 Goal Manager/map-sequence 源码修改后仍缺完整 rectangle。既有 P4 第二阶段 rectangle 证据不能代替本轮第三阶段最终验收。
3. 未记录整条路线的 cross-track p50/p95/max、along-track、wrapped yaw error、MPC solve time、wheel/steer saturation 时序和完整 physical contact 历史；现有 final telemetry 只证明终端 RPM、末尾 slip/steer sample 和 contact evaluator 计数。不得由离散 `footprint_collisions=0` 推导实车物理零碰撞。
4. 包级 `minco_planner` 既有 copyright/cpplint/clang-format 债务仍未处理；本轮仅报告聚焦 CTest 通过。真实电控尚未发布 gimbal feedback 时必须保持 `require_gimbal_status=true` 的 fail-stop，不得为兼容旧路径关闭它。

### 5.10 P4 第三阶段下一步门禁

1. 先选择已测 minimum clearance 大于默认阈值的开阔 route，再验证 `GIMBAL_COMPENSATED` execute/ack；不得降低默认 clearance 阈值来伪造开阔区。
2. 用独立 domain/launch 完成 90 度、S 弯、横移转弯、窄道、坡道和 mode-switch-running；每条保存 reference/localization/body command、cross/along/yaw error、reference age、tracker progress、MPC time、wheel/steer saturation、slip 和 contact CSV。
3. 完成 gimbal stale、错误/迟到 ack、TF loss、map stale、epoch/generation fault 的 `STOP -> emergency_stop -> cmd_vel_mpc=0 -> motion_control=0 -> four-wheel 0 rpm` 运行证据；恢复时禁止旧 request/reference/warm start 复活。
4. 上述结束后必须重新运行 P3 rectangle 和 red_box，red_box 仍要求到达 `(-0.04,-4.08)`，并记录 terminal yaw、速度/dwell 与 contact；再更新本节而非沿用本轮 single 证据。

### 5.11 P4 第四阶段：稳定跟踪、抑制漂移与上场灰度

目标不是继续追求单次更小终点误差，而是在固定地图、固定初始条件、固定配置和固定路线下，获得可重复的定位、reference、跟踪、停止与故障恢复。当前 red_box 已证明可到达，但尚未证明转弯尾误差、定位漂移、执行器饱和和控制时延在比赛包络内可重复受控。

#### 5.11.1 优化顺序与归属

下一阶段严格按以下顺序推进；前一层不满足门禁时不得通过后一层权重掩盖：

1. **测量与真值**：建立统一 telemetry CSV/rosbag，记录 reference、localization、body command、MPC timing、四轮目标/反馈、舵角、饱和、slip、contact、gimbal/body/fake yaw 和全部版本序列。没有外部真值或测量不确定度时，不得把 localization 与 tracking error 混为一项。
2. **定位与传感器几何**：标定雷达/云台/车体外参、云台编码器零位和时间偏移；按点/包测量时间查询云台 TF，验证 deskew。`fake_yaw` 只维持兼容观测 frame，不能修正真实 body yaw，也不能掩盖错误外参或时间同步。
3. **reference 可跟踪性**：MINCO 输出在提交前通过现有四轮几何和执行器约束做离线前视；若 wheel RPM、wheel acceleration、steer rate、yaw rate 或 clearance 预算不可行，优先整体/分段 time scaling，必要时重新优化，禁止把不可行 reference 直接交给 MPC。
4. **控制与时延**：在 reference 已可行后，测量 localization age、reference age、solver time、command-to-wheel delay 与控制周期 jitter；再决定是否加入有界状态前推、执行器延迟模型、reference acceleration feedforward 和 terminal controller。最后才按单一参数组调整 MPC stage/terminal cost。
5. **安全与灰度**：所有速度提升都受制动距离、定位不确定度、tracking 尾误差、地图余量和 operator stop 条件约束；未通过 HIL 与低速实车重复试验前，不进入代表性比赛速度。

#### 5.11.2 净空与稳定性误差预算

每个 trajectory sample 的可执行条件至少满足：

$$
C_{\min}(t) > e_{\mathrm{track},99}(v,\omega,\kappa)
+ e_{\mathrm{loc},99}(v,\omega,\text{scene})
+ v(t)\tau_{99}
+ d_{\mathrm{brake}}(v,\text{slope})
+ m_{\mathrm{map}}
$$

其中 $C_{\min}$ 使用已有 yaw-aware footprint/RC-ESDF clearance；$e_{\mathrm{track},99}$ 和 $e_{\mathrm{loc},99}$ 必须分开测量；$\tau_{99}$ 是 sensor-to-actuator 尾时延；$d_{\mathrm{brake}}$ 来自实车制动测试；$m_{\mathrm{map}}$ 包含分辨率、外参和场景变化余量。预算不成立时只允许降速、延长轨迹、重规划或停止，不能通过减小 footprint、放宽 unknown、扩大执行器限值或降低 stale 门禁获得通过。

#### 5.11.3 yaw policy 与速度调度

1. 开阔区 `GIMBAL_COMPENSATED`：允许 crab/横移，body yaw 不被云台角度驱动；但仍对 body yaw rate、轮速、舵速和 footprint sweep 施加物理约束。
2. 窄道/坡道/接触敏感段 `BODY_YAW_FOLLOW`：必须先 STOP、确认云台锁定，再执行新 sequence reference；yaw 由 footprint clearance、路径切线、坡向和任务姿态共同决定，不固定为所有路径切线。
3. 在 curvature、yaw-rate、clearance 或预计 steer saturation 增大时降低 $v_x/v_y$；在横移段保留真实 $v_y$，不得退化为 DDR。
4. mode switch 只允许 `STOP -> 清 tracker/warm start -> gimbal ack -> fresh reference -> EXECUTE`，运动中直接热切换视为安全失败。

#### 5.11.4 固定基线与准入门禁

所有阈值必须在候选优化前冻结。建议先以当前最终 revision 对每类路线至少运行 `10` 次建立 baseline，再冻结正式阈值；以下是进入低速实车前的初始工程门禁，不是已经达到的实测结论：

| 类别 | 初始门禁 |
| --- | --- |
| 安全契约 | `10/10` 无 contact、无错误 owner、无旧 reference 复活；任一 stale/TF/map/gimbal/epoch 故障在 deadline 内完成五级归零链。 |
| 终端 | 位置误差 p95 `<=0.08 m`、yaw error p95 `<=0.10 rad`、线速度 `<=0.05 m/s`、角速度 `<=0.10 rad/s` 并 dwell `>=0.30 s`。 |
| 跟踪 | cross-track/yaw 的 p50/p95/max 均被记录；正式上限由上式 clearance budget 决定，任何 sample 的剩余安全预算不得为负。 |
| 定位 | 外部真值下分别报告 ATE/RPE、yaw drift、跳变次数和 relocalization false accept；无外部真值时该门禁保持未验证。 |
| 时序 | localization/reference/command age 与 MPC solve time 报 p50/p95/p99；p99 超过各自 lease/deadline 的 run 直接失败。 |
| 执行器 | 报 wheel/steer saturation 次数、持续时间和占空比；不得只有累计计数。稳态持续饱和或饱和与误差峰值一致时，reference 必须降速/延时。 |
| 重复性 | 固定配置下 single、90 度、S 弯、横移、窄道、坡道、rectangle、red_box 各 `10/10` 完成，且不得只选择最优一次。 |

#### 5.11.5 实验矩阵与消融

1. Baseline A：当前代码和参数，不改权重；完成直线、90 度、S 弯、保持 yaw 横移、窄道、坡道、rectangle、red_box。
2. Experiment B：只启用 reference feasibility/time scaling；比较 tracking、饱和、总时间和最小净空。
3. Experiment C：在 B 上只加入测得时延的状态前推/执行器模型；比较 p95/p99 跟踪误差和控制 jitter。
4. Experiment D：在 C 上一次只改变一个 MPC 参数组；位置、yaw、速度、terminal cost 分开做消融，拒绝同时扫全部权重。
5. Localization E：固定控制/reference，分别测试云台静止、旋转、锁定切换、退化几何和重定位；用外部真值区分定位漂移与控制误差。
6. Fault F：gimbal stale、TF loss、map stale、epoch/generation、solver overrun、wheel feedback stale、进程重启，各自使用新 launch/domain。

每次实验保存 revision、配置 hash、地图、模型、随机种子、初始状态、原始 rosbag/CSV、summary JSON 和失败日志。候选只有在主要指标相对 baseline 有预先定义的最小改善、全部 guardrail 不回退且独立重跑可复现时才可保留。

#### 5.11.6 分级上车

1. Gate 0：Release build、单测、replay 和 MuJoCo 全矩阵。
2. Gate 1：执行器禁用/抬轮 HIL，验证真实时钟、gimbal ack、RPM/舵角符号、watchdog 和五级归零链。
3. Gate 2：低能台架，分别测 $v_x$、$v_y$、$\omega_z$ 阶跃、延迟、加减速、制动和电流/温度。
4. Gate 3：封闭低速地面，先直线/横移/停止，再 90 度/S 弯/窄道；每次 run 后检查误差预算与饱和。
5. Gate 4：在上一 gate 连续 `10/10` 通过后，逐级增加速度、路线长度、坡度和云台运动。任一未解释漂移、异常饱和、定位跳变、contact 或 stop 链失败立即回退上一 gate。

### 5.12 行为树决策与 loopback/MuJoCo 双仿真（下一阶段，尚未接入正式主线）

导航链贯通后可以开始行为树决策测试，但“已有行为树包”不等于“行为树已经接入 ATS Nav2-free 主线”。下一阶段允许稳定跟踪和行为决策并行开发，前提是共享场景输入和验收语义，而不是让两个仿真互相替代。

#### 5.12.1 当前静态审计事实

1. `src/ats_sentry_behavior` 已有 BehaviorTree.CPP/BehaviorTree.ROS2、`rmul_2026.xml`、`rmuc_2026_mapping.xml`、巡逻/补给/防守/视觉接管节点和 loopback 参数；该目录本身是独立 Git 仓库，当前 `develop` 为 `24fc53b` 并与 `origin/develop` 一致。
2. 正式 RMUC/RMUL 树仍使用 `SendNavThroughPoses`。该节点直接依赖 `nav2_msgs/action/NavigateThroughPoses` 和 `nav2_msgs/action/NavigateToPose`，默认 action 为 `/navigate_through_poses` 与 `/navigate_to_pose`；它不是 ATS `/ats_navigate_to_pose` client。
3. `SendNavThroughPoses` 当前继承 `BT::SyncActionNode`，发出异步 Nav2 goal 后立即向树返回 `SUCCESS`，且没有 BT halt 回调。它只在后续发送不同 goal 时调用 `cancelCurrentGoal()`。因此，Reactive branch 切换本身不能静态证明旧导航 goal 已被取消；接入正式链前必须以实现和测试收敛 halt/cancel/preempt 语义。[Confidence: High，类定义与完整实现交叉核对；尚未运行专用 BT halt 测试]
4. 当前树使用 `IsPathGoalReached` 的位置容差和本地 `goal_succeeded` 参与路径完成判断，而 ATS Goal Manager 的成功还要求位置、wrapped yaw、终端线/角速度和 dwell。正式树不得以行为层位置判断提前推进 waypoint 或宣告任务成功；ATS action result 必须是终端成功权威。
5. `rmul_2026.xml` 与 RMUC 树在未开赛分支保留 `PublishTwist`；受击/默认自旋通过 `cmd_spin` 发布。`fake_vel_transform` 当前会将 `cmd_spin` 直接加到输出 `angular.z`，所以非零 `cmd_spin` 是绕过 MPC 的车体角速度入口，不只是诊断或云台命令。正式 ATS profile 在解决该旁路前不得启用此输出。
6. 行为 server 当前硬编码订阅 `global_costmap/costmap`、`odom` 和 `odometry`；ATS 正式链的权威输入是 `/rc_esdf/planning_grid` 与 `/localization`。视觉候选点选择可使用 planning grid 作为任务候选证据，但不能把自己的点/圆半径检查写成最终碰撞安全证明，最终安全仍由 RC-ESDF、yaw-aware footprint、continuous swept checker 和 Local Collision Repair 决定。
7. `loopback_decision_sim.launch.py` 当前启动 `nav2_map_server`、完整 Nav2 navigation lifecycle 和旧行为树 action；`src/sim/loopback_sim` 仅按车体系 Twist 积分位姿并生成低保真 odom/TF/scan，没有轮端、舵向、接触、滑移、执行器饱和或云台雷达物理。`loopback_sim` 也是独立 Git 仓库，当前有用户未提交的 `params/nav2_params.yaml` 修改，后续必须保留并合并。
8. 行为仓当前 `BUILD_TESTING` 只配置 ament lint，未发现针对主树优先级、action halt/cancel、迟到 result 或 waypoint 状态机的聚焦功能测试；README 仍把 `/navigate_through_poses` 写为统一执行接口。两项都必须随 ATS action 迁移修正，但在迁移完成前不能先改文档声称已接入。

以上只证明迁移缺口，不是 BT 运行失败结论。当前旧 Nav2 loopback 可继续作为对照，但不能被写成 ATS 决策闭环通过。

#### 5.12.2 目标职责链与唯一所有权

正式比赛职责链固定为：

```text
裁判/视觉/任务场景输入
  -> ats_sentry_behavior BT（任务优先级、目标选择、取消/抢占策略）
  -> ats_navigation_interfaces/action/NavigateToPose
  -> ats_goal_manager（目标生命周期与唯一 STOP/EXECUTE 授权）
  -> JPS/MINCO/ExecutionCommand/MPC
  -> twist bridge/四舵轮底盘
```

1. BT 只拥有任务选择、目标选择和 action cancel/preempt 意图；Goal Manager 继续独占 `ExecutionCommand`、正式 reference 和 stop/execute 授权。
2. 正式树禁止发布 `/cmd_vel_mpc`、`/motion_control`、`/minco/reference_path`、`/planner/emergency_stop` 或 `/planner/execution_command`，也禁止通过 `PublishTwist`、`cmd_spin` 或兼容速度链在 MPC 后叠加车体运动。
3. 树被 halt、任务优先级切换、比赛结束、视觉接管结束或进程关闭时，活动 ATS action 必须显式取消；只有 Goal Manager 接受 cancel 后发布的结构化 STOP 才是运动停止授权。BT 不得自行伪造 `ExecutionCommand`。
4. 单点视觉目标直接使用 ATS action。CSV/path 若只表达目标选择提示，提交最终目标并由 JPS/MINCO 决定几何路线；若中间 waypoint 具有任务语义，则在行为层按顺序发送多个单点 ATS action，每点等待正式 result。禁止为了兼容旧树重新引入 Nav2 `NavigateThroughPoses`。
5. BT 的云台/扫描/攻击意图不得形成第二个 yaw authority。`BODY_YAW_FOLLOW` 的安全锁定请求和实际 feedback acknowledgement 优先级高于行为层云台动作；`GIMBAL_COMPENSATED`、`BODY_YAW_FOLLOW` 与 `HOLD_SAFE_STOP` 仍随 execution reference 验证。

#### 5.12.3 双仿真分层而非相互替代

| 层级 | 应验证内容 | 明确不能证明 |
| --- | --- | --- |
| BT 单测/离线 tick | 黑板字段、优先级、迟滞、branch halt、goal 去抖、输入 stale、确定性 trace | ROS action、TF、导航闭环 |
| ATS loopback 决策仿真 | 同一场景时间线、ATS action goal/cancel/preempt/timeout/result、任务序列、server restart、无孤儿 goal | JPS/MINCO 安全、四舵轮动力学、转弯误差、饱和、slip、contact |
| MuJoCo 完整闭环 | 同一 BT/XML/参数接真实 Goal Manager、JPS/MINCO/MPC/yaw authority/四舵轮，验证路线、终端、饱和、slip、contact 和五级归零链 | 真实云台编码器、点云 deskew、实车通信/制动/热特性 |
| HIL/实车 | 真实时钟、外参、编码器、电控 ack、watchdog、制动、电流/温度和独立急停 | 只有前三级门禁通过后才允许进入 |

“同步”定义为两套仿真消费相同 `scenario_id`、输入事件时间线和期望决策 trace，并在独立 `ROS_DOMAIN_ID`、独立 launch、独立日志目录运行。允许资源足够时并行执行，但禁止共享 ROS graph、临时文件或用 loopback 的成功覆盖 MuJoCo 失败。

#### 5.12.4 迁移与场景门禁

1. 先新增 ATS action BT 节点并锁定 goal、feedback、result、cancel、halt、preempt、timeout、server unavailable/restart 和 action result-code 映射；不得在 sync tick 中无界等待 action server。
2. 给行为层地图、定位、action 和决策输入增加显式 topic/QoS/freshness 参数；正式 profile 使用 `/rc_esdf/planning_grid`、`/localization` 和 `/ats_navigate_to_pose`，旧 Nav2 topic 只留在命名清楚的对照 profile。
3. 建立版本化 scenario runner，至少携带 `scenario_id`、相对事件时间、裁判/视觉输入、期望 branch/task、期望 action 事件、允许的 transition deadline 和故障注入。loopback 与 MuJoCo 生成可比较的有序 trace，不能只解析自由文本日志判断通过。
4. 决策矩阵至少覆盖：未开赛保持 STOP、开赛巡逻、补给、极低 HP 退防/防守策略、关键时间、视觉接管、视觉 stale 返回、受击策略、补给途中视觉优先级、目标输入抖动、goal reject/timeout/cancel/preempt、map/localization/gimbal stale、authority switch while moving、行为 server/Goal Manager restart。
5. 任一优先级切换必须证明旧 action 被取消或明确完成，新 action 具有新 UUID/goal_id；不允许旧 result 修改新任务，不允许重复 tick 造成无界 preempt storm，也不允许重启后旧目标复活。
6. loopback 先通过 decision/action 门禁，再用完全相同的树、参数和场景输入进入 MuJoCo。MuJoCo 还必须复核 `ExecutionCommand STOP -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0 -> four-wheel 0 rpm`、contact、饱和和 terminal position/yaw/velocity/dwell。
7. 固定 revision/config/seed 后，决策场景至少重复 `20` 次无非确定性 branch/action 序列；完整 MuJoCo 关键场景至少 `10/10` 通过。阈值在候选优化前冻结，失败样本全部保留。

该阶段涉及的独立仓库不再只有原三仓。实际修改前至少检查根仓、导航仓、MuJoCo 仓、`src/ats_sentry_behavior` 和 `src/sim/loopback_sim` 五个仓库；只在实际修改的仓库创建中文分内容提交并普通 push，未修改仓库不得制造空提交。

## 6. 下一阶段实施顺序

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
4. 连续 swept footprint、结构化执行授权与 unsafe-trajectory 专用运行注入已在 P4 第二阶段实现；ROG source generation 仍未端到端结构化传播到 MINCO。
5. 在目标机验证完整 `2.5 cm` ESDF、频率、尾延迟、CPU 与峰值 RSS。

### P3：Nav2-free 启动与目标状态机

状态：`Nav2-free 主线已实现并在当前静态 MuJoCo 场景验证`；专用 unsafe-trajectory 运行注入仍未完成，不能把它写成已覆盖的安全验收。

1. 已新增正式 launch 模式并显式使用 `launch_nav2:=false`；MINCO 运行时 `global_plan_topic=""`，P3 图中不出现禁止的 Nav2 节点、Nav2 lifecycle manager 或 `/plan`。
2. 已先以 `/goal_pose` 贯通最小闭环，再以 ATS 自定义 action 完成正式 single、rectangle、red_box。action 覆盖 feedback、result、cancel、preempt、timeout 与 TF/map/planning 失败，并对所有终止状态执行安全停止。
3. 已把任务生命周期和正式 reference/急停交给目标管理器，JPS/MINCO 与 MPC 保持职责分离；P3 graph、planning grid、急停与速度/底盘输入的唯一所有权已实际检查。
4. 已独立注入 adapter lease、projection timeout、input stale、all-unknown、free-unreachable、cancel、preempt、timeout、TF failure，并在恢复后验证无新目标时双零。P4 已补定位故障、舵轮约束、contact evaluator、连续 swept footprint 与 unsafe trajectory 运行注入。
5. 上述 P3 状态只覆盖导航 action 到底盘链，不代表 `ats_sentry_behavior` 已接入。当前正式行为树仍使用 Nav2 action、Nav2 costmap/odom 名称和速度旁路；其 ATS action 迁移与双仿真验收属于 5.12 的未完成范围。

### P4：定位融合、连续安全与舵轮执行约束

状态：`第二阶段已实现并完成当前 MuJoCo 组件与指定运行验证`。定位融合、epoch 安全链、四舵轮几何/执行器约束、contact evaluator、连续 swept footprint、结构化执行授权和 unsafe trajectory 注入均已实现；实车统计标定、控制 mux 与灰度仍未完成。

1. 已为 small_gicp 增加结构化重定位观测，并由 `localization_fusion` 独占 `map -> odom`、定位健康状态和 localization epoch。
2. 已补齐 `/odometry -> /localization` 显式契约，按观测时间对齐 odom 历史；已验证 stale、迟到/乱序、GICP 拒绝、假匹配、跳变和 TF 丢失恢复时旧 reference 不复活。
3. 已按四轮位置落实轮速、轮速增量/加速度与舵速约束，并在三个 MuJoCo 模型落实总质量、总质心、轮位、最终轮径和分离摩擦语义。
4. swept checker 已覆盖细墙、横移、斜切、旋转角扫、yaw wrap、unknown/outside 和带 yaw 的 grid origin；运行期覆盖提交后变障碍、纯旋转、unknown、outside、旧 generation/epoch、TF 丢失和最终停止。剩余工作是目标机上测量真实 footprint margin、传感器时延与制动距离。
5. 使用实车电流、轮端阶跃、反馈延迟、rosbag 重定位统计标定协方差、质量门限与加速度约束；完成控制 mux 和灰度门禁后再上车。

## 7. 下一对话接续入口

下一对话从 P4 第四阶段的统一 telemetry/baseline 与 5.12 的 ATS 行为树 action 迁移开始：先用同一 scenario 进行 BT 离线/loopback 决策验证，再进入 MuJoCo 的真实 Goal Manager/JPS/MINCO/MPC 闭环；reference feasibility/time scaling、实车 rosbag/轮端与时延标定、控制 mux 和受控灰度按门禁后续推进。不得重复实现定位融合、ROGMap/adapter、JPS、MINCO 或 MPC，也不得把 `/rog_map/esdf` 调试点云作为规划距离场。不得通过放宽 unknown、frame、footprint、执行器物理限值或 stale 安全门禁换取路线通过。

必须保持以下边界：

1. Point-LIO 链继续提供局部连续 `/odometry` 与 `/registered_scan`，不得用 ROGMap 替换里程计；`localization_fusion` 继续独占 `/localization` 与 `map -> odom`。
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

## 8. 当前构建与回归入口

Nav2 对照和 P3 正式入口必须分开运行：

```bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select ats_navigation_interfaces ats_goal_manager ats_rog_map_adapter \
    minco_planner ats_swerve_mpc ats_mujoco_sim --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 仅 Nav2 对照：允许 NavigateToPose、/plan 与 lifecycle 节点。
ROS_DOMAIN_ID=215 scripts/test_mujoco_nav_chain.sh

# P4 fusion + Nav2-free 正式入口：ATS action，无 Nav2 与 /plan。
ROS_DOMAIN_ID=220 NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action \
  P4_LOCALIZATION_FUSION=true PLANNING_GRID_OWNER=rog_map \
  P2_FAULT_CASE=none P3_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
ROS_DOMAIN_ID=219 NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action \
  P4_LOCALIZATION_FUSION=true PLANNING_GRID_OWNER=rog_map \
  P2_FAULT_CASE=none P3_FAULT_CASE=none \
  TEST_PROFILE=rectangle GOAL_TIMEOUT=120 scripts/test_mujoco_minco_mpc_chain.sh

# 四舵轮几何、执行器、滑移、饱和、急停与 contact evaluator。
ROS_DOMAIN_ID=214 scripts/test_mujoco_swerve_dynamics.sh

# 每项必须使用新的 domain 和新的 MuJoCo launch。
domain=207
for fault in odometry_stale delayed gicp_rejected false_match epoch_jump tf_loss; do
  ROS_DOMAIN_ID="${domain}" P4_FAULT_CASE="${fault}" \
    scripts/test_mujoco_localization_fault.sh
  domain=$((domain + 1))
done

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

## 9. 维护约束

1. 后续提交统一在各仓库 `develop` 分支完成并推送 `origin/develop`。
2. 文档滚动保留最近七个自然日内当前决策所需的验证记录，方便回溯和阅读；超过一周的过期状态合并为当前结论，不保留无关历史流水账。
3. 参考目录不是运行时依赖。进入比赛链的源码、配置、消息定义和许可证信息必须受版本控制。
4. 修改地图语义时必须验证 frame、时间戳、分辨率、origin、unknown 和 signed distance；P3 的正式门禁是 Nav2-free rectangle 与 red_box，Nav2 红框只保留为对照。
5. 保留 Nav2 模式作为对照，直到自研链具备同等或更高覆盖；新功能优先落在自研链，禁止重新引入 Nav2 action 或 `/plan` 依赖。
