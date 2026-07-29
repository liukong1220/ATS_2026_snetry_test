# ATS 自研导航 V1 当前状态与下一阶段交接

更新时间：2026-07-28

有效更新窗口：2026-07-22 至 2026-07-28（滚动保留最近一周）。窗口外的逐日验证流水（原 5.1~5.8，覆盖 2026-07-13 至 2026-07-17）已按第 9 节维护约束第 2 条合并为 5.1 的当前结论，不再保留分日记录。本文只保留当前有效架构、最近验证结果和下一阶段任务，方便回溯和连续阅读。MuJoCo 继续承担规划、控制、四舵轮动力学和 contact 的完整闭环；`loopback_sim` 只承担低成本行为决策、action 生命周期和任务时间线检查，不能替代 MuJoCo 验收。

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

该状态机、结构化观测、定位融合、localization epoch 和重定位触发的自动重规划均已实现并通过节点测试与 MuJoCo 故障注入。[Confidence: High] 证据为 fusion/Goal Manager 节点测试和 5.1 合并结论中的六个独立定位故障闭环；实车 rosbag、错误接受率、恢复时延分布和阈值标定仍未验证。

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

### 5.1 2026-07-13 至 2026-07-17 结论合并（P1/P2/P3 与 P4 第一、二阶段）

按第 9 节维护约束第 2 条，原 5.1~5.8 的分日流水已合并为下列当前结论，只保留仍在支撑当前决策的结论与验证边界，不再保留逐次运行明细。后续小节保留原编号 5.9~5.13，避免破坏第 6、7 节的交叉引用。

**地图与规划控制链（原 P1/P2）**

1. ROGMap 三维滑窗、四类调试点云、概率对数几率融合与 stale 衰减已在 MuJoCo 验收；`ats_rog_map` 不发布任何机器人 TF，也不发布 `/rc_esdf/planning_grid`。四档配置见 6 节 P1 表格。
2. `ats_rog_map_adapter` 只调用数值 `GetRogMapProjection` service，融合 ROGMap、`traversability_grid`、`traversability_slope_grid` 与静态墙，unknown 默认按障碍处理；`planning_grid_owner` 在 `rc_esdf|rog_map` 间选定 `/rc_esdf/planning_grid` 的唯一 publisher，不支持运行中热切换。禁止从 `/rog_map/esdf` 可视化点云反解析距离场。
3. MINCO 每次 planning-grid 回调构造不可变 snapshot，单次 JPS、二维 RC-ESDF、clearance、footprint gate 与 Local Collision Repair 不混用地图更新；最终 reference 在提交点统一重定时，先发布 `emergency_stop=false` 再发布 reference。
4. map ready heartbeat（`10 Hz`）、`0.5 s` steady-clock lease、projection deadline/epoch、input stale、规划失败与 MPC tracker clear 已构成确定性急停链；ready 恢复本身不能重新授权旧轨迹。
5. Nav2 上游加自研规划控制的红框回归（rectangle 五段、red_box 中转与终点）终点误差量级为 `0.02~0.08 m`、离散 footprint 冲突 `0`；该批只作为地图与规划控制回归，不能替代 P3 Nav2-free 证据。

**Nav2-free 目标管理（原 P3）**

1. `ats_navigation_interfaces` action、`PlannerGoal`/`PlannerStatus`、`ats_goal_manager` 与 `static_map_publisher.py` 构成 Nav2-free 主线；MINCO 只发布 `/minco/reference_path_candidate` 与 `/minco/planning_status`，不越权发布正式 reference 或急停。
2. 正式入口显式 `launch_nav2:=false`、`global_plan_topic=""`，运行图中不出现 Nav2 节点、lifecycle manager 或 `/plan`；single、rectangle、red_box 均由 ATS action 完成，并覆盖 feedback、result、cancel、preempt、timeout。
3. 已独立注入并验证的失效安全用例：adapter lease、projection timeout、input stale、all-unknown（`RESULT_MAP_UNREADY=4`）、free-unreachable（`RESULT_PLANNING_FAILED=5`）、cancel、preempt、timeout、TF failure；每例均确定性急停、两级零速度，恢复后无新目标时不复活旧 reference。
4. `rmuc_2026_mujoco.launch.py` 是唯一 MuJoCo 入口；回归脚本不得依赖 `rg` 等隐式工具，`/local_elastic_path` 一类 transient-local 话题必须先订阅再启动。

**定位融合与四舵轮执行约束（原 P4 第一阶段）**

1. `localization_fusion` 独占 `map -> odom`、`/localization` 与定位健康/epoch；small_gicp 提供结构化 `RelocalizationObservation`，按观测时间对齐 odom 历史。
2. 六个独立定位故障闭环（`odometry_stale`、`delayed`、`gicp_rejected`、`false_match`、`epoch_jump`、`tf_loss`）均验证急停、两级零速度与恢复后旧 reference 不复活；该批即 1.4 引用的故障证据。其中 `odometry_stale` 在 P4 第二阶段补跑时于初始 `TRACKING` 前超时，未重新注入。[Confidence: Medium]
3. 四舵轮几何与执行器语义已在三个 MuJoCo 模型与 MPC 配置间对齐：`wheel_base_x/y=0.270 m` 是半轴偏置（轮心距 `0.540 m`）、`wheel_radius=0.0425 m` 是含包胶最终滚动半径、总质量 `25 kg`、质心高 `0.100 m`、摩擦 `0.8/0.02/0.005`；`450 rpm` 与 `120 rpm` 经减速比和 `1.2` 冗余折算为 `max_wheel_speed=1.6689711 m/s`、`max_steer_rate=10.4719755 rad/s`。
4. 动力学矩阵运行已记录各阶段峰值与累计饱和次数，证明执行器约束在运行中频繁介入；该累计量不足以支撑放宽任何执行器限值。

**连续安全与执行契约（原 P4 第二阶段）**

1. `FootprintSafetyChecker` 在相邻 reference SE(2) 段间做 swept 检查，分段数按四个最远角点位移除以 `planning_grid.resolution * swept_max_corner_step_cells`（默认半格）向上取整，yaw 用最短角插值；这是保守离散采样，不是解析连续 Minkowski sweep。
2. `ExecutionCommand` 是 Goal Manager 到 MPC 的唯一执行授权，原子携带 `STOP/EXECUTE`、reference、goal id、localization epoch、snapshot generation、adapter publication sequence、`failure_reason` 枚举与严格单调命令序号；legacy `Path` 与 `emergency_stop=false` 不能重新授权 tracker。
3. 运行期 unsafe 用例（提交后变障碍、纯旋转、unknown、outside、旧 generation/epoch、mid-segment、repair 后复核）均进入确定性停止；`FAILURE_RUNTIME_UNSAFE` 由 Goal Manager 决策停止与重规划，MINCO 不越权急停。

**该批结论的验证边界（继续有效）**

1. `footprint_collisions=0` 只表示离散定向矩形 gate 未发现冲突采样；P4 contact evaluator 也不是实车碰撞证明，不得由此推导实车物理零碰撞。
2. `rog_map_report_025.yaml` 的 `50 Hz`、约 `6 ms`、CPU 与峰值 RSS 未在目标机测量，技术报告数字不得写成 ATS 实测结果。
3. ROG source generation 仍未结构化传播到 MINCO；MINCO 仍从融合 planning grid 重建二维 RC-ESDF，未直接消费 adapter 数值 ESDF。
4. `/planner/emergency_stop` 与 `/minco/reference_path` 只保留为诊断兼容输出，不具备 DDS 跨 topic 原子事务。
5. `minco_planner` 包级 `copyright/cpplint/clang_format` 既有债务未处理，历次只报告聚焦 CTest 通过。
6. 实车仅完成 `docs/p4_real_robot_calibration_preflight.md` 的采集与停止门禁定义，未通电、未运动。

### 5.9 P4 第三阶段 yaw authority/fake-yaw 结论合并（原 2026-07-20，已按第 9 节第 2 条合并）

本节的逐次 MuJoCo 运行流水（原 `ROS_DOMAIN_ID=34~51` 共 8 条记录）已过滚动窗口，按第 9 节维护约束第 2 条合并为下面的当前结论，不再保留分次记录。它不是 P4 第三阶段的完整验收结论。

**已实现和已测试的契约**

1. `ExecutionCommand` 增加 `yaw_authority`、`requires_gimbal_lock`、`gimbal_request_sequence` 与 `gimbal_feedback_sequence`；新增 `YawAuthorityRequest` 和 `GimbalYawStatus`。三个 authority 常量为 `HOLD_SAFE_STOP=0`、`GIMBAL_COMPENSATED=1`、`BODY_YAW_FOLLOW=2`。Goal Manager 仍是唯一 `STOP/EXECUTE` 授权者；MPC 不接受 legacy `Path` 或 `emergency_stop=false` 重新授权。
2. 每一个 candidate reference 都先由 Goal Manager 发布包含 `goal_id`、localization epoch、MINCO local snapshot generation、adapter publication sequence、authority 和 lock 要求的 `YawAuthorityRequest`。只有收到同一 request sequence 的新鲜、TF 健康反馈后才提交 execute。MPC 连续复核 feedback lease、authority、lock、request sequence 和 feedback sequence；任一失配清 tracker/warm start 并输出零速度。
3. authority 改变不允许运动中热切换：Goal Manager 先发布结构化 `STOP`、清 active command/pending request，并回到 planning；新 candidate 经新的 request/ack 后才回到 tracking。节点 pytest 已复现并修复“STOP 后 lifecycle 仍为 tracking、正确锁定确认却不能重新 execute”的状态机错误。
4. MINCO 将 immutable reference 的最小有限定向 footprint clearance 用于整段保守 authority 选择：`force_body_yaw_follow=true` 必选 `BODY_YAW_FOLLOW`；否则任一点 clearance 不高于 `body_yaw_follow_clearance`（默认 `0.55 m`）即选 body。当前 planning grid 不携带坡道来源标签，因此坡道/接触敏感 route 只能由 profile 显式 force body，不能写成已自动识别坡道。
5. `fake_vel_transform` 修正为首个有效 odometry yaw 初始化 compatibility frame：令 $\psi_0$ 为初始 gimbal/body yaw、$\psi$ 为当前 yaw，则 `gimbal_yaw_odom -> gimbal_yaw_fake` 为 $\psi_0-\psi$，fake velocity 回 gimbal velocity 为 $R(\psi-\psi_0)$。这保留非零初始摆放 yaw，且速度旋转为 TF 的逆变换；fake frame 不覆盖真实 world-state body yaw。P3/MPC 主链仍关闭 legacy fake velocity adapter，避免二次旋转 `/cmd_vel_mpc`；兼容 TF 未删除。
6. MuJoCo 订阅 yaw request 并以 transient-local/reliable `/gimbal/yaw_status` 回显 authority、lock、request sequence、body/gimbal/fake yaw。其 status QoS 已与 Goal Manager/MPC 匹配。当前模型的 MID360 固定在 `base_link`，所以只能验证 request/ack、lease 和 fail-stop，**不能**验证真实云台旋转的点云位移、地图匹配漂移或 fake-yaw 传感器补偿保真。

**独立证据（合并后的当前结论）**

1. Release 构建通过：`ats_navigation_interfaces`、`fake_vel_transform`、`minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`、`ats_mujoco_sim`。聚焦测试通过：`test_fake_yaw_math`（非零初始 yaw、跨 $\pm\pi$、TF/速度逆变换）、`test_yaw_authority_policy`（开阔/窄道/force profile）、Goal Manager lifecycle/epoch pytest（错误 request、未锁定 body、authority STOP/re-ack、旧 epoch/reference 不复活）、MPC `test_se2_mpc_controller`、`test_trajectory_tracker` 和 `test_mpc_localization_gate`（world/body 变换、yaw wrap、旧 sequence/epoch、gimbal stale/错误 request fail-stop），以及 MuJoCo `test_swerve_physics` 6 项。
2. 静态审计和 tracker 单测共同证明 SE2 模型仍按 $R(yaw)[v_x,v_y]^T$ 将车体系控制投影到世界系状态，MINCO 世界速度按参考 yaw 变为 body control，yaw error 使用 $\operatorname{atan2}(\sin\Delta\psi,\cos\Delta\psi)$。故“world velocity 被直接当作 body control”与“yaw wrap 缺失”不是已证实根因。[Confidence: High，源码加针对性测试；尚非实车动力学证据]
3. MuJoCo single 与 red_box 均已完整通过，终端位置误差落在 `0.002 m` 至 `0.019 m` 区间，离散 footprint 冲突与 contact violation 全为 `0`，终态四轮 `0 rpm`。终端 action 成功由 position/yaw、body linear/angular velocity 与 `0.30 s` dwell 共同门控，不再仅凭 pose。
4. authority 选择由实测 clearance 决定，不是权重或状态机故障：`minimum_clearance` 实测 `0.094` 至 `0.160 m`，均低于 `body_yaw_follow_clearance=0.55 m`，因此 execute 合理落在 `BODY_YAW_FOLLOW`（locked）。开阔 route 的 `GIMBAL_COMPENSATED` 只有单测覆盖。
5. 累计 drive-acceleration/steer-rate saturation 曾记录为 `29234/12840`，证明约束在运行中频繁介入；但缺分段时序和占空比，不得据此归因转弯误差或放宽限值。
6. 回归脚本的所有 event-driven topic capture 已改为显式 ROS message type、启动前存活检查和有界 `TERM/KILL/reap`，并给所有 `ros2 topic info` 加 `timeout`，避免 ros2cli graph discovery 挂起被误判为路径/控制故障。

**未完成、不可宣称的范围**

1. 尚未在实测 clearance 大于默认 `0.55 m` 的开阔 route 完成 `GIMBAL_COMPENSATED` MuJoCo action；`0.10 m` 阈值敏感性运行因 ros2cli graph 查询挂起而未进入 action。单元测试覆盖开阔 clearance 的 gimbal 分支，但不是旋转云台物理闭环。
2. 仍未完成 90 度转弯、S 弯、保持 yaw 横移、窄道、坡道/起伏专用 route、mode switch while moving 专项 route。rectangle 已在 5.15 补跑通过（Nav2 对照 profile），故障矩阵见 5.15。
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

### 5.13 2026-07-26 MINCO+MPC 缺陷审查与实车化整改

本节记录本轮按"MINCO 稀疏轨迹表示 / MPC 预测模型 / 二者协同"三条线做的静态审查结论与已落地整改。所有结论都给出源码锚点；未运行的部分显式标注，不得当作闭环证据。

**高危：已修复并有测试证据（`已验证`）**

1. 轮速执行器边界整体失效。[se2_mpc_controller.cpp:122-128](../src/ats_sentry_nav/ats_swerve_mpc/src/se2_mpc_controller.cpp#L122-L128) 原 `maxModuleSpeed()` 直接返回 `0.0`，使 `clampControl` 的逐轮速度判据永远不成立，等于把四舵轮的轮缘速度上限关掉。仿真中电机是理想速度源，看不出异常；实车会直接超出 `450 rpm` 能力并进入失速。修复后按 `wheel_base_x/y`、`wheel_radius` 与 `max_wheel_speed` 推导逐轮上限，并对整个车体 Twist 做等比缩放而不是逐轮独立截断，保持四轮速度向量仍来自同一车体 `[vx, vy, wz]`（否则破坏舵轮瞬心一致性）。测试：`ClampsBodyCommandByPerWheelSpeed`、`KeepsWheelSpeedLimitWhenWheelBaseUnset`。
2. 模型时间轴与执行时间轴不一致。[ats_swerve_mpc.yaml:54-60](../src/ats_sentry_nav/ats_swerve_mpc/config/ats_swerve_mpc.yaml#L54-L60) 原 `dt=0.1` 而 `control_rate_hz=20.0`（周期 `0.05 s`），预测的"一步"实际只执行半步，等价把加速度与舵角速率约束放大 `2` 倍。已改为 `dt=0.05`，并把 `horizon` 定为 `30`（预测时长 `1.5 s`，覆盖 `max_vx=1.5 m/s` 下 $v^2/(2a)\approx0.56\ \mathrm{m}$ 的完整制动过程）。
3. 舵角 `180°` 反向翻转被判成零变化。[se2_mpc_controller.cpp:159-190](../src/ats_sentry_nav/ats_swerve_mpc/src/se2_mpc_controller.cpp#L159-L190) 原实现对舵角差取绝对值，瞬时反向的实际 $\pi$ 变化被读成 `0`，舵角速率门禁形同失效。改为带符号点积求角差，并在 [se2_mpc_controller.cpp:203-236](../src/ats_sentry_nav/ats_swerve_mpc/src/se2_mpc_controller.cpp#L203-L236) 用 40 次二分把不可行增量收缩到可行域。测试：`RejectsInstantModuleFlipUnderSteerRateLimit`、`ClampsMovingWheelSteerRate`、`ClampsEveryWheelVelocityIncrement`。
4. 求解成功判据过严与饱和诊断结构性失效。[se2_mpc_controller.cpp:377-487](../src/ats_sentry_nav/ats_swerve_mpc/src/se2_mpc_controller.cpp#L377-L487) 现以"至少一次反向递推成功"置 `solution_certified`，并要求序列长度、代价有限同时满足才算成功；首步饱和在前向生成时捕获，避免原实现里 saturation 报告永远为空。测试：`FailsWhenNoIterationIsCertified`、`FailsWhenReferenceHorizonTooShort`、`SucceedsAtStationaryWarmStart`。
5. `ats_swerve_mpc` 本轮 `colcon test` 为 `28` 项全通过、`0 error / 0 failure / 0 skip`（本机隔离 domain 运行）。[Confidence: High]

**高危：已实现未运行（`已实现未运行`）**

1. 重规划恒按"车辆静止"播种 MINCO 首端边界。原 `solveMinco` 的 head 恒为零，`p(0)=c_0,\ \dot p(0)=c_1,\ \ddot p(0)=2c_2` 中的一、二阶导数被强制为零，车辆以 `1.5 m/s` 行进时重规划瞬间参考速度从 `0` 起算，MPC 前馈与实际状态出现阶跃（仿真里定位无噪声且重规划稀疏，不易暴露）。已在 [minco_trajectory_optimizer.hpp:56-99](../src/ats_sentry_nav/minco_planner/include/minco_planner/trajectory/minco_trajectory_optimizer.hpp#L56-L99) 引入 `InitialKinematicState`（`valid` 开关 + 默认 `nullptr`，保持既有四个单测与调用点语义不变），在 [minco_trajectory_optimizer.cpp:137-162](../src/ats_sentry_nav/minco_planner/src/trajectory/minco_trajectory_optimizer.cpp#L137-L162) 用 `initial_state_max_speed`/`initial_state_max_acceleration` 与轨迹级 `max_velocity`/`max_acceleration` 双重裁剪（首端速度若本身超限，时间缩放不改变边界条件，永远压不回可行域），并在 ESDF 净空修正 [minco_trajectory_optimizer.cpp:262-272](../src/ats_sentry_nav/minco_planner/src/trajectory/minco_trajectory_optimizer.cpp#L262-L272)、首次求解与时间缩放回代中使用同一 head，保证修正量落在实际会被执行的曲线上。当前 `minco_planner` 已编译通过，但节点侧尚未接线，闭环未运行。
2. 尚未接线项（`未实现`，属下一阶段）：`minco_planner_node` 未声明/读取上述两个参数、四处 `optimizer_.optimize(...)` 未传入初值、无 `/localization` 里程计订阅、无周期重规划定时器（`onRuntimeSafetyRecheck` 目前只能停不能重规划）、`toPath` 丢弃解析得到的 `vx/vy/ax/ay` 前馈、约 25 处规划失败日志仍为英文。

**中危：已确认未修复（`未实现`）**

1. 无安全走廊约束。MINCO 侧只有航点空间的 ESDF 启发式修正（`maximum_step`/`maximum_deviation` 夹紧、端点锁定）加事后 footprint swept gate，没有把净空写成优化约束，也没有解析梯度回传。窄道下表现为"多次修正仍不可行 → 直接判失败"，而不是收敛到贴边可行解。
2. 时间缩放是全局均匀的。局部一处超限会把整条轨迹按同一 `scale` 拉长（[minco_trajectory_optimizer.cpp:412-439](../src/ats_sentry_nav/minco_planner/src/trajectory/minco_trajectory_optimizer.cpp#L412-L439)），全程变慢；需要按段自适应 time scaling，属 5.11.1 的 reference feasibility 环节。
3. MINCO 输出到 MPC 的前馈信息在 `Path` 转换处被截断，MPC 只能靠数值差分恢复参考速度，等于放弃了 MINCO 的解析可微优势。

**协同层结论**

1. 数据格式已对齐：世界系状态 `[x, y, yaw]`、车体系控制 `[vx, vy, wz]`，`ExecutionCommand` 原子携带 reference、goal id、localization epoch、snapshot generation、adapter publication sequence 与单调命令序号，是唯一执行授权。
2. 频率匹配当前依赖 `command_latency_compensation=0.01 s` 与投影窗口（`projection_backward_window=0.20`、`projection_forward_window=2.00`）吸收规划周期与控制周期差；端到端时延尚未逐段测量，`tau_99` 未知，因此 5.11.2 的净空预算目前不可闭合。
3. 时间戳同步：reference 在提交点统一重定时，但缺少 route-wide 的 reference age 时序记录，无法给出 p50/p95/p99，属 5.10 的第二条门禁。

**MuJoCo 场地模型冲突（`已验证` 为静态度量，整改 `未实现`）**

1. 碰撞表示重复：hfield 地形与 `1621` 个 group-3 墙体 box 同时为实体，墙体因 `geomgroup[3]=0` 对 LiDAR 不可见，形成"能撞不能看"的不一致。
2. 高度压缩 `10` 倍：`--z-scale 0.1` 使 PNG `255` 映射为 `0.292571 m`，而 `RM2026场地.pdf` 对应真实高度约 `2.926 m`。
3. 覆盖不一致：pgm occupied `163545`，hfield`>0` 且 occupied 仅 `104461`，有 `59084` 个 occupied 栅格 hfield 为 `0`（其中 `31014` 贴边界）。
4. 原点不一致：`8.027637` 与 `8.025` 并存，产生约 `2.6 mm` 的 y 偏移；`clear_nav_free_space` 还会抹平坡道。
5. 生成顺序存在环形依赖：`rmuc_corridor_patch.py` → `rmuc_hfield.py` → `rmuc_nav_map.py`；墙体只能经 `rmuc_wall_collisions.py` 与 `rmuc_2026_swerve.xml:65` 的 `<include>` 再生。
6. 图纸目标值（供整改对齐）：场地 `28 m × 15 m`、围挡 `2.4 m`、横向倾角 `1°~2°`、梯形高地 `200~400 mm`（`43°`/`23°`）、中央高地 `10.5°`、装配区 `12°/14°/15°/45°`、公路区 `11°/15°`、飞坡 `17°`、堡垒 `20°`；公差 `<100 mm` 为 `±5 mm`、`≥100 mm` 为 `±5%`、结构 `±3°`、道具 `±1°`。

**本轮不可声明项**

1. 上述 MINCO 首端播种未做 MuJoCo 闭环，也未新增针对性单测，不得写成跟踪精度改善证据。
2. MPC 三处高危修复只有单测与构建证据，本轮未重跑 rectangle/red_box 闭环，原 5.9 的运行数字仍是当前最新闭环证据。
3. MuJoCo 场地冲突只有静态度量，未修改模型，因此既有闭环结论仍建立在旧场地模型上。

### 5.14 2026-07-27 实车接口连通性审计（串口、行为树、实车 launch、MID360）

本节回答"现在能否上车（MID360）、串口与行为树及各方面接口能否与当前自研导航框架连通"。结论：**当前不能上车。** 自研 MINCO+MPC 链没有出现在实车运行图中，命令通路与授权通路都断在实车侧。以下全部是静态源码/配置审计结论（`已验证` 仅指静态事实），未通电、未运动、未运行本轮闭环。[Confidence: High]

**结论一：实车运行图里没有自研导航链（阻塞级）**

1. `minco_planner_node`、`ats_goal_manager_node`、`ats_swerve_mpc_node` 只出现在 `src/sim/ats_mujoco_sim/launch/mujoco_navigation.launch.py`、`rmuc_2026_mujoco.launch.py`、各包自带 launch 与 `ats_goal_manager` 测试中；`src/ats_sentry_nav/ats_nav_bringup/launch/` 下八个 launch 文件零引用。
2. 实车入口链 [bringup.launch.py](../src/ats_sentry_bringup/launch/bringup.launch.py) -> [rm_navigation_reality_launch.py](../src/ats_sentry_nav/ats_nav_bringup/launch/rm_navigation_reality_launch.py) -> `bringup_launch.py` -> [navigation_launch.py](../src/ats_sentry_nav/ats_nav_bringup/launch/navigation_launch.py) 仍启动 Nav2 `controller_server`/`planner_server`/`bt_navigator`/`lifecycle_manager` 与 `trajectory_optimizer_node`、`trajectory_speed_governor_node`。
3. 参数缺失：`src/ats_sentry_bringup/params/node_params.yaml` 与 `ats_nav_bringup/config/reality/nav2_params.yaml` 均无 `minco_planner`/`ats_swerve_mpc`/`ats_goal_manager` 段；[ats_swerve_mpc.yaml:3](../src/ats_sentry_nav/ats_swerve_mpc/config/ats_swerve_mpc.yaml#L3) 仍为 `use_sim_time: true`。

**结论二：命令通路与授权通路断裂（阻塞级）**

1. MPC 输出 `/cmd_vel_mpc` 在 `src/ats_sentry_bringup` 与 `ats_nav_bringup` 内没有任何订阅者；实车实际通路是 `cmd_vel_nav2_result -> fake_vel_transform -> cmd_vel_gimbal_yaw_odom -> chassis_vel_transform -> /cmd_vel -> 串口 speed_vector`。
2. `GimbalYawStatus` 仅由 [sim_node.py](../src/sim/ats_mujoco_sim/ats_mujoco_sim/sim_node.py) 发布，实车无发布者；而 Goal Manager 与 MPC 都是 `require_gimbal_status: true`、超时 `0.5 s`，实车上电后整链会直接停在确定性停止态。
3. `fake_vel_transform` 把 `cmd_spin` 直接叠加到输出 `angular.z`，是 MPC 之后的第二个车体角速度入口，正式 profile 未解决前不得启用。

**结论三：串口层协议兼容，但保护逻辑与执行器上限冲突**

1. 兼容（无需改造）：[packet_typedef.hpp:216-219](../src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp#L216-L219) 的 packed `speed_vector {float vx; float vy; float wz;}` 与 [standard_robot_pp_ros2.cpp:962-967](../src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp#L962-L967) 的 `linear.x/linear.y/angular.z -> vx/vy/wz` 本身就是车体系全向命令，与四舵轮语义一致，不需要迁入差速或 `vy=0`。
2. 高危：`enable_transient_zero_cmd_hold: true` 与 `transient_zero_cmd_hold_timeout_ms: 50`（[standard_robot_pp_ros2.yaml:24-26](../src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml#L24-L26)、实现见 [standard_robot_pp_ros2.cpp:932-960](../src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp#L932-L960)）在收到零命令时继续下发上一条非零 twist，最多抑制 `50 ms` 的五级归零链。
3. 高危：出口级比授权级宽松。[node_params.yaml:248-261](../src/ats_sentry_bringup/params/node_params.yaml#L248-L261) 的 `chassis_vel_transform` 允许 `4.6 m/s`、`3.6 m/s^2`、`4.2 rad/s`，远高于 MPC 可行域 `max_vx/vy=1.5`、`max_ax/ay=2.0`、`max_wz=2.0`、`max_awz=3.0`。
4. 高危：`pass_through_without_yaw: true` 在缺 `serial/gimbal_joint_state` 时不旋转直通命令，属静默降级放行。该目录 `sentry_chassis_vel_transform/` 是用户未跟踪文件且自带嵌套 `.git`，归属用户。
5. `serialPortProtect()`（[standard_robot_pp_ros2.cpp:306-322](../src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp#L306-L322)）保持连接、断开重连、异常处理三项仍为 `@TODO`；`cmd_vel_watchdog_timeout_ms: 300` 与上层 `0.5 s` lease 的时序关系未论证。

**结论四：行为树用的是 Nav2 接口，不是 ATS 接口（阻塞级）**

1. [send_nav2_goal.cpp:12-53](../src/ats_sentry_behavior/plugins/action/send_nav2_goal.cpp#L12-L53) 与 `send_nav_through_poses.cpp` 依赖 `nav2_msgs/action/NavigateToPose`、`NavigateThroughPoses`，默认名 `/navigate_to_pose`（[send_nav_through_poses.hpp:80](../src/ats_sentry_behavior/include/ats_sentry_behavior/plugins/action/send_nav_through_poses.hpp#L80)、[sentry_behavior.yaml:184](../src/ats_sentry_behavior/params/sentry_behavior.yaml#L184)）；Goal Manager 提供的是 `ats_navigation_interfaces/action/NavigateToPose`、服务名 `/ats_navigate_to_pose`（[ats_goal_manager_node.cpp:154-155](../src/ats_sentry_nav/ats_goal_manager/src/ats_goal_manager_node.cpp#L154-L155)）。消息定义不同，不存在自动兼容。
2. `decision.topics.cmd_vel`（[ats_sentry_behavior_server.cpp:218](../src/ats_sentry_behavior/src/ats_sentry_behavior_server.cpp#L218)）仍暴露行为层直发底盘通路，被 `docs/p4_real_robot_calibration_preflight.md` 行为决策准入禁止。其余 5.12.1 已记录的缺口（`SyncActionNode` 无 halt、`IsPathGoalReached` 位置判断、硬编码 costmap/odom 输入、无聚焦功能测试）仍全部有效。

**结论五：MID360 配置内部一致，但外参未标定、有效参数双份**

1. 一致：[MID360_config.json](../src/ats_sentry_nav/livox_ros_driver2/config/MID360_config.json) 与 [mid360_user_config.json](../src/ats_sentry_nav/ats_nav_bringup/config/reality/mid360_user_config.json) 的主机 `192.168.1.50`、雷达 `192.168.1.177` 与端口对齐；`node_params.yaml:59-73` 使用 `xfer_format: 4`、`frame_id: front_mid360`。
2. 高危：两份配置的 `extrinsic_parameter` 全为零，云台上安装的 MID360 到 `base_link` 外参未标定，Point-LIO 输出与 footprint/净空判据不在同一几何基准上。
3. Point-LIO 有效参数双份不一致：[point_lio/config/mid360.yaml](../src/ats_sentry_nav/point_lio/config/mid360.yaml) 与 `node_params.yaml:85-110` 在 `filter_size_map`（`0.5` vs `0.15`）、`ivox_nearby_type`（`6` vs `18`）、`blind`（`0.5` vs `0.3`）、`cut_frame_time_interval`（`0.1` vs `0.05`）上冲突，实车生效值必须固定并在运行时打印确认。

**本节不可声明项**

1. 以上全部为静态审计，未通电、未运动、未采集实车 rosbag，不得作为任何实车能力证据。
2. 本轮未修改任何代码或 launch，因此实车不可上车结论在 P6 完成前保持有效。
3. 本轮未重跑 MuJoCo 闭环，5.9 的运行数字仍是当前最新闭环证据。

### 5.15 2026-07-28 P6 实车接口连通与抬轮 HIL 准备（部分实现，实车未通电）

本节记录 P6 本轮改动与实测证据。**结论边界：本轮只做到"具备不通电检查与抬轮 HIL 的软件条件"，实车未通电、未抬轮、未落地行走。** 所有实车数值项均为 `未实测`。[Confidence: High]

**已实现（有构建与单测证据）**

1. 实车 Nav2-free 入口 [real_robot_nav2_free.launch.py](../src/ats_sentry_bringup/launch/real_robot_nav2_free.launch.py)：钉死 `launch_nav2:=false`、`launch_swerve_mpc:=true`、`launch_fake_vel_transform:=false`、`launch_chassis_vel_transform:=false`、`mpc_cmd_vel_topic:=/cmd_vel`、`use_sim_time:=False`，与 Nav2 对照 profile 互斥。
2. 唯一命令通路：[navigation_launch.py](../src/ats_sentry_nav/ats_nav_bringup/launch/navigation_launch.py) 的 `fake_vel_transform_enabled`/`chassis_vel_transform_enabled` 都追加 `and launch_nav2 == 'true'`，因此 Nav2-free 下 MPC 之后不存在任何旋转级或增益级；`gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF 由 `UnlessCondition(fake_vel_transform_enabled)` 提供，每个 profile 各有唯一所有者。
3. 实车 `GimbalYawStatus` 发布者 `gimbal_yaw_status_bridge_node`（[standard_robot_pp_ros2](../src/standard_robot_pp_ros2/src/gimbal_yaw_status_bridge.cpp)）由串口云台关节反馈驱动，`locked`/`tf_healthy` 均为实测量；`require_gimbal_status` 为假时禁止 `BODY_YAW_FOLLOW`。未伪造任何 ack。
4. ATS action 行为树节点 `SendAtsNavGoal`（[send_ats_nav_goal.cpp](../src/ats_sentry_behavior/plugins/action/send_ats_nav_goal.cpp)）：正式 RMUC/RMUL 树切到 `/ats_navigate_to_pose` 与 `ats_navigation_interfaces/action/NavigateToPose`，可 halt/cancel，行为层直发 `cmd_vel` 通路已移除，输入改为 `/rc_esdf/planning_grid` 与 `/localization`。
5. 实车 `gimbal_yaw_odom -> front_mid360` 静态 TF（[bringup.launch.py](../src/ats_sentry_bringup/launch/bringup.launch.py) 的 `static_tf_gimbal_yaw_odom_to_front_mid360`）：此前实车侧零发布者，而 `sensor_scan_generation::odometryHandler` 在该查询失败时整帧 return，连带 `odom->gimbal_yaw_odom`、`odom->base_footprint` 一起消失。条件为 `use_sim_time == false and launch_lidar_static_tf == true`，不与仿真发布者共存。
6. Point-LIO 生效值可验证：新增 `logEffectiveParameters()`（[parameters.cpp:268-344](../src/ats_sentry_nav/point_lio/src/parameters.cpp#L268-L344)）在启动时打印七行 `[point_lio 生效参数]`；`point_lio/config/mid360.yaml` 经 launch 图追踪确认不生效，已加 `[Dead Code Suggestion]` 头（未删除）。
7. 对照 profile 的 `SendNavThroughPoses` 由 `BT::SyncActionNode` 改为 `BT::StatefulActionNode`（[send_nav_through_poses.cpp](../src/ats_sentry_behavior/plugins/action/send_nav_through_poses.cpp)）。原实现有两个与「授权 STOP -> 串口零速度」直接冲突的缺陷：`SyncActionNode::halt()` 是 `final`（只做 `resetStatus()`），派生类无法插入 `async_cancel_goal()`，且 `tick()` 在 `async_send_goal()` 之后立刻返回 `SUCCESS`，节点从不停留在 RUNNING——两者叠加使主树 halt 掉导航分支后 Nav2 侧目标仍在执行，对照 profile 下停车链整条失效；另有 `tick()` 内 `wait_for_action_server(2.0s)` 的阻塞等待，最坏每拍阻塞 `2 s`。现改为目标飞行中返回 `RUNNING`、`onHalted()` 真正 cancel，server 就绪判断改为 `action_server_is_ready()` + 跨拍累计等待。顺带修两处判据错误：单点路径句柄在 `current_goal_to_pose_handle_` 而原实现只看 `current_goal_handle_`（单点目标飞行中被判成「无活动目标」反复重发）；`ABORTED` 收尾原本直接重发同一路径使失败永不上报，现返回 `FAILURE`（主动 cancel 不置该标志）。

**已测试（本轮实测数字）**

1. 构建：`MAKEFLAGS=-j1 colcon build --base-paths src --packages-select point_lio` → `Finished <<< point_lio [51.7s]`；`ats_sentry_bringup standard_robot_pp_ros2 ats_sentry_behavior` 与 `ats_mujoco_sim` 均 exit 0。
2. 功能单测 `54/54` 全通过：`test_ats_nav_goal_logic` 11、`test_official_tree_priority` 5、`test_send_ats_nav_goal_action` 10、`test_send_nav_through_poses_halt` 4、`test_cmd_vel_authorization` 14、`test_gimbal_yaw_status_logic` 10，failures/errors 均为 0。这是**功能 gtest**口径，不是全包测试口径：`minco_planner` 的 `clang_format`/`copyright`/`cpplint` 既有债务本轮未修，也未加重（详见本节末尾的债务基线）。
3. MuJoCo 四舵轮动力学 `scripts/test_mujoco_swerve_dynamics.sh` 通过，`failures: []`，17 个相位 `drive_speed_saturations=0`、`contact_violations=0`；`vy` 全程可控（`lateral` 相位 `max_measured_vy=0.5004`），确认车体系全向语义未退化为差速。
4. MuJoCo 闭环三条全部 `PASS`（每条独立 `ROS_DOMAIN_ID`，viewer/RViz 关闭）：

   | 用例 | 命令要点 | 终端位置误差 | 结果 |
   | --- | --- | --- | --- |
   | Nav2 对照 `red_box` | `ROS_DOMAIN_ID=63 TEST_PROFILE=red_box GOAL_TIMEOUT=180` | `0.0073 / 0.0072 m` | PASS，`EXIT=0` |
   | Nav2 对照 `rectangle` | `ROS_DOMAIN_ID=64 TEST_PROFILE=rectangle GOAL_TIMEOUT=120` | `0.034 / 0.011 / 0.053 / 0.010 m` | PASS，`EXIT=0`，`south`/`north` 段测得横向 MPC 命令 |
   | **Nav2-free 自研链 `red_box`** | `ROS_DOMAIN_ID=65 NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action TEST_PROFILE=red_box GOAL_TIMEOUT=180` | `0.003081 / 0.005662 m` | PASS，`EXIT=0` |

   Nav2-free 用例是本轮零/二两节的直接证据，实测项：ATS action 返回 `result_code: 0` 且有 feedback；
   `/planner/execution_command`、`/minco/reference_path`、`/ats_goal_manager/planner_goal`、`/cmd_vel_mpc`、
   `/motion_control` 各自唯一所有者；`/planner/emergency_stop` 唯一发布者为 `ats_goal_manager`；
   `footprint_collisions=0`；`execute_yaw_authority=2 requires_gimbal_lock=true request_sequence=2
   feedback_sequence=748`（云台 ack 由仿真真实反馈驱动，未伪造）；
   `contact_violation_count=0` 且终局四轮驱动转速低于 `2 rpm`。
5. 门禁顺序（先确定性、后 MuJoCo）第一关通过：固定 revision/config 下把五个功能测试二进制连跑 `20` 轮，
   `20/20` 通过，剥掉每例耗时后的判定签名 `distinct_signatures=1`，即 `50` 个用例的顺序与结论完全一致，
   无非确定性分支或动作序列。
6. 门禁顺序第二关通过：同一 revision/config/seed 下把 Nav2-free `red_box` 闭环重复 `10` 次
   （`ROS_DOMAIN_ID` 101–110，每次独立 MuJoCo launch，viewer/RViz 关闭，
   `NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action PLANNING_GRID_OWNER=rog_map GOAL_TIMEOUT=180`）：
   `10/10` 全部 `rc=0`，`footprint_collisions=0`，安全契约断言 `10/10`。
   `20` 个终端位置误差样本 `min=0.002725 m`、`max=0.028782 m`、`p95=0.017402 m`，
   均在 `<= 0.08 m` 判据内。阈值在候选优化之前已冻结，全部样本保留在
   `/tmp/p6_mujoco_10x_summary.txt`。
7. 九类故障注入矩阵 `9/9` 通过（每例独立 `ROS_DOMAIN_ID` 130–138、独立 MuJoCo launch、
   viewer/RViz 关闭，统一 `NAVIGATION_MODE=p3 P3_GOAL_ENTRY=action PLANNING_GRID_OWNER=rog_map
   TEST_PROFILE=red_box GOAL_TIMEOUT=180`）。全部 `rc=0`、`FAIL` 行数为 `0`，
   每例都先断言 `/cmd_vel_mpc` 与 `/motion_control` 载过非零命令，再在故障期与恢复期
   断言两条通路同时为零（判据：`|value| <= 0.001` 且必须是有限数）：

   | 用例 | 注入 | action `result_code` | 归零证据 |
   | --- | --- | --- | --- |
   | `P2 adapter_lease` | SIGSTOP adapter 使心跳租约超时 | 不涉及 | `adapter_lease` 与 `adapter_lease_recovery` 两段 `/cmd_vel_mpc`、`/motion_control` 均为零 |
   | `P2 service_timeout` | SIGSTOP ROGMap 使数值投影服务超时 | 不涉及 | 同上两段共 4 条零断言，且 adapter 发布 not-ready |
   | `P2 input_stale` | SIGSTOP MuJoCo 使 Point-LIO 兼容输入 stale | 不涉及 | 同上两段共 4 条零断言 |
   | `P2 unknown` | 规划网格整体置 unknown | `4`（`MAP_UNREADY`） | 同上两段共 4 条零断言 |
   | `P2 unreachable` | 空闲但不可达目标 | `5`（`PLANNING_FAILED`） | 故障期 2 条零断言（该例无恢复段） |
   | `P3 cancel` | `action_msgs/srv/CancelGoal` 真实取消 | `1`（`CANCELED`） | `cancel` 与 `cancel_recovery` 共 4 条零断言 |
   | `P3 preempt` | 第二个目标抢占仍在 tracking 的第一个 | 首个 `2`（`PREEMPTED`）、次个 `3` | `preempt` 与 `preempt_recovery` 共 4 条零断言 |
   | `P3 timeout` | goal `timeout = 1 s` | `3`（`TIMEOUT`） | `timeout` 与 `timeout_recovery` 共 4 条零断言 |
   | `P3 tf_failure` | 目标 frame 为不存在的 `p3_missing_goal_frame` | `6`（`TF_FAILED`） | 故障期 2 条零断言 |

   关键点：五个 P2 用例与三个有恢复段的 P3 用例都额外断言了**恢复段仍然为零**，
   即故障消失本身不会让运动自动恢复——恢复必须经由新的授权序号。
   全部原始日志保留在 `/tmp/p6_fault_p2_*.log` 与 `/tmp/p6_fault_p3_*.log`，
   汇总在 `/tmp/p6_fault_matrix_summary.txt`。
   本矩阵是**仿真**口径：验证的是 `ready=false -> emergency_stop=true -> /cmd_vel_mpc=0
   -> /motion_control=0` 这一段，串口那一级（`/motion_control=0 -> 串口输出零速度`）
   在仿真里由 `twist_to_motion_ctrl -> ats_mujoco_sim` 承接，实车串口段仍为 `未实测`。
8. 抬轮 HIL 需要的行为树开关：`bringup.launch.py` 与 `real_robot_nav2_free.launch.py`
   新增 `launch_behavior`（默认 `True`）。此前 `start_behavior_launch_cmd` 无条件加入
   `LaunchDescription`，行为树会自己下发导航目标，五级归零的实测时延就无法归因到
   某一次授权跳变。抬轮 HIL 必须用 `launch_behavior:=False`。
   `python3 -m py_compile` 两个 launch 均通过，`ament_pep257` `No problems found`。
9. 注意：`scripts/test_mujoco_minco_mpc_chain.sh` 的 `NAVIGATION_MODE` 默认值是 `nav2`，
   因此提示词第五节给出的两条命令实际跑的是 Nav2 对照 profile。自研链必须显式加
   `NAVIGATION_MODE=p3`，本轮已补跑并单列在上表第三行。

**本轮修复的自身缺陷（重复运行才暴露）**

`test_send_ats_nav_goal_action.cpp` 的脚本化服务端把 `execute()` 放在 `detach()` 的线程里并持有裸 `this`，
而 `TearDown()` 在 executor cancel 之后直接销毁该对象，构成 use-after-free。
单次运行看不出来，重复 30 次时约 `4/30` 出现 `free(): invalid pointer` 或段错误（`rc=134`/`rc=139`），
且总是发生在最后一个用例的 fixture 析构处，已跑过的用例仍报 OK——**这类缺陷单跑一次会被完全掩盖**。
修复：线程登记到 `workers_` 由析构函数统一 join，hold 循环增加 `shutting_down_` 退出条件避免 join 死锁，
并把 `server_.reset()` 移到 `executor->cancel()` 之前，使收尾的 `succeed()`/`abort()` 仍打在活着的 executor 上。
修复后 `30/30` 全部 `rc=0` 且每次 10 个用例齐全。

另外，本轮在写文档时先声称已把 `SendNavThroughPoses` 改成可 halt 的异步节点，
实际读代码才发现它仍是原始的 `BT::SyncActionNode`（`git log` 显示自
`24fc53b Initial split` 以来未改动）。已按上面已实现第 7 条真正改造并补测试
（`test_send_nav_through_poses_halt.cpp` 4 项：单点 halt cancel、多点 halt cancel、
空路径不下发、server 缺失按超时判失败且首拍耗时 `< 200 ms`）。
教训：文档条目必须以代码为准回读确认，不能以计划为准书写。

**既有 lint 债务基线（本轮未加重）**

`send_nav_through_poses.cpp` 与 `.hpp` 的 `ament_cpplint` 错误数
HEAD `7` → 工作区 `7`（`legal/copyright` 2、`build/header_guard` 2、
`whitespace/line_length` 3，全部为既有项）；`ament_clang_format` divergences
HEAD `46` → 工作区 `40`。本轮新增文件 `test_send_nav_through_poses_halt.cpp`
的 `ament_clang_format` 与 `ament_cpplint` 均为 `No problems found`。

**本轮修复的两项阻塞（否则闭环无法运行）**

1. CycloneDDS 参与者索引上限：默认 `MaxAutoParticipantIndex=9`，而 MuJoCo 闭环图有 25 个以上参与者，第 10 个之后的节点在 `rmw_create_node` 阶段直接抛 `Failed to find a free participant index`。本轮用测试专用配置 `/tmp/ats_cyclonedds_loopback.xml`（`lo`、`MaxAutoParticipantIndex=120`）运行，未修改用户的 `~/.ros/cyclonedds.xml`。属环境债务，需要长期方案。
2. MuJoCo CPU 雷达后端与 `mujoco 3.4.0` 不兼容：[mjlidar_cpu.py](../src/sim/ats_mujoco_sim/mujoco_lidar/core_cpu/mjlidar_cpu.py) 传入已被移除的 `normal=` 形参，且 `pnt`/`vec` 为 `float32`，`mj_multiRay` 抛 `TypeError` 使雷达子进程退出，`/local_pointcloud` 永不发布，连带 ROGMap `odom` 与 `/traversability_grid` 缺失。已修为 `float64` 并去掉 `normal=`；单独验证 RMUC 模型 `2000` 条射线中 `806` 条命中，距离区间 `[1.381, 24.860] m`。

**本节不可声明项**

1. 实车未通电、未抬轮、未落地行走：串口五级归零实测时延、`serialPortProtect` 重连实测行为、端到端 `tau_99` 分解全部 `未实测`，不得代入 5.11.2 净空预算。
2. MID360 `gimbal_yaw_odom -> front_mid360` 静态 TF 的六个数值取自仿真模型 `ats_sentry_robot.sdf.xmacro:45`，`未实测`；Point-LIO `extrinsic_T` 亦未实测（启动日志会打印全零告警）。落地行走前必须替换为实测标定值，属停止条件。
3. MuJoCo `max_contact_force` 各相位均为 `0.0`，即本轮没有测得任何接触力，物理接触评估为 `未验证`，不得写成"零碰撞"。
4. P3 Nav2-free、P5 均未完成；实车定位精度、跟踪性能未验证。

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
6. 本轮（5.13）已修复 MPC 侧四项高危实车缺陷（轮速边界失效、模型/执行时间轴不一致、舵角翻转判据、求解成功判据与饱和诊断），并在 optimizer 层实现带初速的 MINCO 首端播种。MPC 有单测证据；MINCO 播种属 `已实现未运行`，节点接线、周期重规划、前馈传递与中文分级日志仍未完成。

### P5：实车化整改（未完成）

状态：`迁移中`。范围与门禁见 5.13。

1. `minco_planner_node` 接线带初速播种：两个新参数的声明/读取、四处 `optimize` 调用点传值、`/localization` 里程计订阅、周期重规划定时器（使 `onRuntimeSafetyRecheck` 可触发重规划而非只停止）、`toPath` 保留 `vx/vy/ax/ay` 前馈、规划失败日志改中文分级，并补带初速头状态单测。
2. 按段自适应 time scaling 替代全局均匀缩放；净空进入优化约束（安全走廊）而非仅事后 gate。
3. MuJoCo 场地模型单一权威碰撞表示、按图纸校正高度与坡角、统一 `8.025` 原点、固定 pgm→hfield→墙体生成顺序。
4. 上述任一改动完成后必须重跑 rectangle 与 red_box 闭环，且不得放宽 unknown、footprint、执行器或 stale 门禁换取通过。

### P6：实车接口连通（软件侧部分完成，实车侧未开始）

状态：`部分实现`。软件接线与仿真验证见 5.15；实车实测项全部 `未实测`。范围与门禁见 5.14 与 `docs/p6_real_robot_interface_integration_prompt.md`。

1. 实车 Nav2-free profile：`已实现`。`real_robot_nav2_free.launch.py` 启动 `minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`，钉死 `launch_nav2:=false` 与 `use_sim_time:=False`，与 Nav2 对照 profile 互斥。`已测试`（MuJoCo `NAVIGATION_MODE=p3` 闭环通过），实车 `未验证`。
2. 唯一命令通路：`已实现`。MPC 以 `mpc_cmd_vel_topic:=/cmd_vel` 直接产出车体系命令，`fake_vel_transform` 与 `chassis_vel_transform` 在 Nav2-free 下不启动，MPC 之后无任何旋转级或增益级。`已测试`（闭环断言 `fake_vel_transform absent` 与 `/cmd_vel_mpc` 唯一所有者），实车 `未验证`。
3. 串口层：`部分实现`。授权归零绕过瞬时零保持、`serialPortProtect` 重连与重连期零速度、新 epoch 才恢复授权、符号/单位/轮位口径表均已落地并有单测（`test_cmd_vel_authorization` 14 例通过）；**实测归零时延与实测重连行为 `未实测`**，必须抬轮 HIL 补齐。出口限幅问题在正式 profile 中因该节点停用而消解，对照 profile 仍需收紧。
4. 行为树：`已实现`。`SendAtsNavGoal` 切到 `/ats_navigate_to_pose`，行为层直发底盘通路已移除，输入改用 `/rc_esdf/planning_grid` 与 `/localization`；对照 profile 的 `SendNavThroughPoses` 由 `SyncActionNode` 改为可 halt 的 `StatefulActionNode`。`已测试`（halt/cancel/迟到 result/主树优先级/对照节点 halt 共 30 例通过）。抬轮 HIL 用 `launch_behavior:=False` 关掉整棵树，使授权只由测试脚本触发。
5. 实车 `GimbalYawStatus` 发布者：`已实现`（`gimbal_yaw_status_bridge_node`，由串口云台关节反馈驱动，`已测试` 10 例）。未伪造 ack。实车反馈链路 `未验证`。
6. MID360：`部分实现`。Point-LIO 实车生效参数已唯一化并在启动时打印，实车 `gimbal_yaw_odom -> front_mid360` 静态 TF 已补齐；**外参仍为未实测值（仿真模型取值），静止 rosbag 采集与漂移统计 `未实现`**，属落地行走前的停止条件。
7. 故障判据：`已测试`。九类故障注入 `9/9` 通过，逐例证明 `ready=false -> emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0` 成立，且八例额外证明故障消失本身不恢复运动。第五级（串口输出零速度 -> 四轮 0 rpm）实车 `未实测`。
8. 分级执行：本轮只推进到「具备不通电检查与抬轮 HIL 的软件条件」。第 1 级不通电检查与第 2 级抬轮 HIL 均 `未执行`（实车未通电），第 3、4 级不在本轮范围。5.11.2 净空预算与实测 `tau_99` 仍未闭合。

## 7. 下一对话接续入口

下一对话按"实车不通电检查与抬轮 HIL 优先、P5 并行"的顺序推进。P6 软件侧接线已完成（见 5.15），下一轮的第一件事是**在实车上执行第 1 级不通电检查与第 2 级抬轮 HIL**，按 `docs/p4_real_robot_calibration_preflight.md` 逐项记录，并补齐本轮标记为 `未实测` 的三类数值：

1. 串口五级归零实测时延（授权 STOP、急停、定位 stale、串口断连、缺云台 ack 五条路径各自的实测值）。
2. `serialPortProtect` 断连/重连实测行为，含重连期零速度保持与新 epoch 恢复授权。
3. 端到端 $\tau_{99}$ 分解（传感器 → 定位 → 规划 → 授权 → MPC → 底盘），不得用估计值代入 5.11.2。

同时必须完成 MID360 外参实测标定（当前写入值取自仿真模型，是落地行走的停止条件）与静止 rosbag 采集/漂移统计。

抬轮 HIL 的启动方式已就位，直接用：

```bash
ros2 launch ats_sentry_bringup real_robot_nav2_free.launch.py \
  launch_behavior:=False require_gimbal_status:=False
```

`launch_behavior:=False` 是必需的：行为树自己下发目标会让归零时延无法归因到单次授权跳变。
`require_gimbal_status` 按云台是否通电取值，置 `False` 时 `BODY_YAW_FOLLOW` 被禁止，且不得伪造 ack。
仿真侧的九类故障判据（`ready=false -> emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0`）
已 `9/9` 通过，抬轮 HIL 要补的是第五级，即这四级之后串口是否真的输出零速度、四轮是否真的 `0 rpm`。

`minco_planner` 节点接线与 MuJoCo 场地模型冲突（P5，提示词见 `docs/p5_real_robot_hardening_prompt.md`）继续作为仿真侧主线，随后继续 P4 第四阶段的统一 telemetry/baseline 与 5.12 的 ATS 行为树 action 迁移：先用同一 scenario 进行 BT 离线/loopback 决策验证，再进入 MuJoCo 的真实 Goal Manager/JPS/MINCO/MPC 闭环；reference feasibility/time scaling、实车 rosbag/轮端与时延标定、控制 mux 和受控灰度按门禁后续推进。不得重复实现定位融合、ROGMap/adapter、JPS、MINCO 或 MPC，也不得把 `/rog_map/esdf` 调试点云作为规划距离场。不得通过放宽 unknown、frame、footprint、执行器物理限值或 stale 安全门禁换取路线通过。

必须保持以下边界：

1. Point-LIO 链继续提供局部连续 `/odometry` 与 `/registered_scan`，不得用 ROGMap 替换里程计；`localization_fusion` 继续独占 `/localization` 与 `map -> odom`。
2. RC-ESDF、JPS、MINCO 与全向 SE2 MPC 继续保留；P2、P3 及后续运行中 `/rc_esdf/planning_grid` 始终只能有一个发布者。
3. 两级速度变换按 profile 分离（P6 零.2 起）：Nav2 对照 profile 默认保留 `launch_fake_vel_transform:=True` 与 `launch_chassis_vel_transform:=True`；Nav2-free 正式 profile 二者强制关闭，MPC 之后不允许存在任何改变 `[vx, vy, wz]` 数值或方向的环节。关闭 fake-yaw 时仍由零旋转兼容静态 TF 提供 `gimbal_yaw_odom -> gimbal_yaw_fake`，两个 profile 各有唯一发布者，待所有 Nav2/行为参数改为固定 frame 后再删除该兼容层。
4. P3 已保留 P2 owner、heartbeat、snapshot 和急停契约；后续 P3 正式回归必须继续显式 `launch_nav2:=false`、使用 ATS action、拒绝 `/plan`，Nav2 `NavigateToPose` 只能作为独立对照。
5. 行为树正式树只允许 `/ats_navigate_to_pose`；`/navigate_to_pose` 与 `/navigate_through_poses` 只能出现在明确命名的对照 profile，且两者不得同时启用。

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
