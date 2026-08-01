# ROGMap 与 RViz 可视化升级方案

## 1. 目标

新版 `src/ats_sentry_bringup/rviz/sentry_default_view.rviz` 必须直接服务于 ATS 自研运行图，满足三类需求：

1. 地图开发：观察 ROGMap raw occupancy、inflation、unknown、ESDF 和地图边界；
2. 规划安全：观察 planning grid、JPS raw path、MINCO candidate/committed reference、footprint collision/repair；
3. 执行诊断：观察 localization、MPC predicted path、command、telemetry 和 health/stop 状态。

可视化是诊断旁路，绝不成为 planning 输入。`/rog_map/esdf` 只能用于 RViz，不得由 adapter、MINCO 或 MPC 反解析数值距离。

## 2. 当前状态

`ats_rog_map_node.cpp` 当前发布：

| topic | 类型 | 内容 | 默认 QoS |
| --- | --- | --- | --- |
| `/rog_map/occ` | `PointCloud2 XYZ` | raw occupied voxels | best effort、volatile、depth 1 |
| `/rog_map/inf_occ` | `PointCloud2 XYZ` | ROG inflation occupied voxels | best effort、volatile、depth 1 |
| `/rog_map/unk` | `PointCloud2 XYZ` | unknown voxels | best effort、volatile、depth 1 |
| `/rog_map/esdf` | `PointCloud2 XYZI` | 单一高度切片，intensity 为 signed distance | best effort、volatile、depth 1 |
| `/rog_map/stale` | `Bool` | map update 或 odometry stale | reliable、depth 1 |

节点只在有订阅者时构建对应 debug cloud，这一优化应保留。当前主 RViz 没有完整 ROGMap display 组，仍显示 `plan`、global/local costmap、MPPI rollout、transformed plan，并使用 `nav2_rviz_plugins/GoalTool`。

## 3. 外部参考与许可

参考文章：

- 标题：ROG-Map: 一种高效的以机器人为中心的大场景高分辨率 LiDAR 运动规划网格地图（论文阅读）
- 作者：CUN_CUI
- URL：<https://blog.csdn.net/CUN_CUI/article/details/158610185>
- 发布：2026-03-03；页面标注修改日期：2026-07-11
- 本次访问：2026-07-31
- 许可：页面声明 CC BY-SA 4.0，转载需署名、链接原文并保留相同许可

本文件不整篇复制文章和图片，只保存与 ATS 实施直接相关的技术摘要与映射。文章中的公开数据是参考项目/论文结论，不是 ATS 实测性能。

文章与当前实现相关的要点：

- ROGMap 使用机器人中心的固定内存滑动局部地图；`map_size` 与 resolution 决定容器，滑动复用内存；
- incremental inflation 通过占据/未知邻居计数更新派生膨胀层；
- ProbMap 是概率地图，InfMap 是膨胀地图，ESDFMap 为规划/优化提供距离；
- RViz 中用不同颜色的边界框区分局部地图、可视化裁剪、单帧更新和规划搜索范围。

## 4. 原项目风格的四色边界映射

| 参考语义 | 颜色 | ATS topic | owner | 当前状态 |
| --- | --- | --- | --- | --- |
| Local Map Range | 橙色 | `/rog_map/debug_bounds` namespace `local_map_range` | `ats_rog_map` | 未实现 |
| Visualization Range | 紫色 | `/rog_map/debug_bounds` namespace `visualization_range` | `ats_rog_map` | 未实现 |
| Current Update Range | 绿色 | `/rog_map/debug_bounds` namespace `update_range` | `ats_rog_map` | 未实现 |
| A*/JPS Search Range | 淡蓝色 | `/minco/debug_markers` namespace `search_range` | `minco_planner` | 未实现 |

边界 Marker 设计：

- 使用 `visualization_msgs/MarkerArray`，每个边界为 `LINE_LIST` 或 `CUBE` wireframe；
- `header.frame_id` 必须与边界数值所在 frame 相同，通常为 `odom`；
- `header.stamp` 使用产生该范围的数据时间，不能一律写 `now()` 掩盖 stale；
- 固定 namespace + id，更新时覆盖旧 Marker；地图 reset 时发送 `DELETE`；
- Marker 只描述范围，不承担 occupancy、ESDF 或 collision 语义。

owner 边界很重要：A*/JPS 搜索范围属于 planner，不应由 ROGMap 猜测；ROGMap 只发布地图内部可证明的 local、visualization 和 update bounds。

## 5. ROGMap PointCloud2 显示参数

### 5.1 Raw Occupancy

```text
Class: rviz_default_plugins/PointCloud2
Topic: /rog_map/occ
Style: Boxes
Size: 与 rog_map.resolution 相同，初始 0.10 m
Color Transformer: FlatColor
Color: 220, 55, 47
Alpha: 0.90
Enabled: true
Reliability: Best Effort
```

### 5.2 Inflated Occupancy

```text
Topic: /rog_map/inf_occ
Style: Boxes
Size: 与 inflation_resolution 相同，初始 0.10 m
Color: 245, 145, 35
Alpha: 0.45
Enabled: true
```

raw occupancy 与 inflation 必须同时可见但颜色/透明度不同，避免把物理障碍和规划 margin 混为一层。

### 5.3 Unknown

```text
Topic: /rog_map/unk
Style: Boxes
Color: 125, 132, 140
Alpha: 0.15
Enabled: false
```

unknown 点数量大，默认关闭。需要观察 unknown safety 时显式开启，并限制 `visualization_range`，禁止为显示效果修改 planning 的 unknown 语义。

### 5.4 ESDF Slice

```text
Topic: /rog_map/esdf
Style: Flat Squares 或 Points
Channel: intensity
Color Transformer: Intensity
Use rainbow: true
Autocompute Intensity Bounds: false
Min Value: -0.20
Max Value: 2.00
Enabled: false
```

ESDF cloud 的 z 为 `esdf_visualization_height`，intensity 为 signed distance。负值是 occupied 内部、正值是 free clearance、NaN 不发布。显示上下界必须与 `signed_distance_max_m` 和实际截断语义一致。

## 6. 新主 RViz 分组

```text
Displays
├── 01 Robot & TF
│   ├── RobotModel
│   ├── TF
│   └── Localization Odom/Path
├── 02 Sensor
│   ├── Registered Scan
│   └── LiDAR Frame
├── 03 ROGMap 3D
│   ├── Raw Occupancy
│   ├── Inflated Occupancy
│   ├── Unknown
│   ├── ESDF Slice
│   └── ROGMap Bounds
├── 04 Ground Planning
│   ├── Static Map
│   ├── Planning Grid
│   ├── Signed Distance Grid
│   ├── Footprint Clearance Grid
│   └── Terrain/Slope
├── 05 JPS & MINCO
│   ├── Raw Path
│   ├── Candidate Reference
│   ├── Committed Reference
│   ├── Search/Footprint/Repair Markers
│   └── Goal Pose
├── 06 MPC & Chassis
│   ├── MPC Reference
│   ├── MPC Predicted Path
│   ├── Odom/GT Path
│   └── Swerve Telemetry Markers
└── 07 Health & Safety
    ├── Localization State
    ├── Map Lease/Generation
    ├── Planner State/Failure
    ├── Execution Command/Stop
    └── Topic Diagnostics
```

删除或替换的旧 display：

- 删除 `Global Planner` 下 `plan`、global costmap 和 Nav2 footprint；
- 删除 `Local Planner (MPPI)` 下 local costmap、transformed global plan、MPPI rollouts/lookahead；
- 保留仍真实发布的 odom/GT path，但移入 MPC/Chassis；
- 将 `trajectory_esdf_debug` 仅在其 owner 仍是活动自研链时保留；
- 删除 `nav2_rviz_plugins/GoalTool`。

## 7. 目标交互

### 阶段 V1

使用 RViz 默认 goal tool 发布 `/goal_pose`。Goal Manager 的 `input_goal_topic` 是唯一 consumer，并由 Goal Manager 创建内部 goal identity、执行规划、发布 stop/command。不得让 RViz 直接发布 `/ats_goal_manager/planner_goal`。

### 阶段 V2

新增 `ats_rviz_plugins/NavigateToPoseTool`：

- action client 指向 `/ats_navigate_to_pose`；
- 支持点击位置 + 拖动 yaw；
- 新目标发送前明确 cancel/preempt 当前目标；
- 状态只在 RViz panel 显示，不用 marker 伪造 action result；
- 提供 cancel 按钮；
- server 不可用或 result failure 时不自动重试。

## 8. 诊断可视化缺口

建议新增结构化诊断，不把所有状态塞进 Marker 文本：

| 数据 | 推荐接口 | RViz/CLI |
| --- | --- | --- |
| ROG source generation、update bounds | debug Marker + snapshot status | RViz + topic echo |
| adapter publication/lease | `PlanningMapSnapshot`/DiagnosticStatus | panel/diagnostics |
| MINCO failure reason、snapshot generation | `PlannerCandidate`/status | panel |
| footprint collision count/repair | MarkerArray + numeric status | RViz + evaluator |
| MPC solve time/saturation | `SwerveTelemetry` + diagnostics | plot/panel |
| emergency stop reason | structured enum/status | panel，不只显示 Bool |

## 9. 实施文件

| 仓库 | 文件 | 修改 |
| --- | --- | --- |
| nav | `ats_rog_map/src/ats_rog_map_node.cpp` | 发布 ROGMap bounds MarkerArray |
| nav | `ats_rog_map/include/ats_rog_map/rog_map_engine.hpp` | 暴露 local/visualization/update bounds 的只读 snapshot API |
| nav | `minco_planner/src/nodes/minco_planner_node.cpp` | 发布 JPS search range 与 repair markers |
| root | `ats_sentry_bringup/rviz/sentry_default_view.rviz` | 重建 displays/tools/views |
| root/nav | package manifests/CMake | 仅在需要新 RViz plugin 时增加依赖；删除 nav2_rviz_plugins |
| root | `docs/启动入口与运行链路.md`、`docs/仿真域说明.md` | 更新启动与截图说明 |

## 10. 验收

### 静态

- RViz 文件中无 `nav2_rviz_plugins`、`global_costmap`、`local_costmap`、`transformed_global_plan`；
- 所有显示 topic 在活动源码有 producer；
- PointCloud2 display reliability 与 publisher 匹配；
- Fixed Frame 为实际存在的 `odom` 或 `map`，不依赖缺失 TF。

### 运行

1. 新 domain 启动无 RViz MuJoCo，先确认主链闭环不依赖可视化订阅。
2. 再启动 RViz，确认四类 cloud 非空且加入 RViz 后不改变 planning generation 行为。
3. 检查 bounds 位置、尺寸和滑动；update range 必须随帧变化，search range 必须随 goal 变化。
4. 在 desktop 和目标显示器截图，检查 display 非空、颜色可区分、文本不重叠。
5. 关闭 RViz，确认 debug cloud 构建随 subscription count 降为零。

### 性能

记录 RViz 关闭/开启时 ROGMap update p50/p95/p99、CPU 和内存。未在 ATS 目标机测量前，不引用参考文章中的 50 Hz、约 6 ms 作为验收阈值。

## 11. 停止条件

- 为实现显示而改变 occupancy、unknown、inflation 或 ESDF 数值语义；
- adapter/规划器开始订阅 `/rog_map/esdf`；
- bounds marker 的 frame/stamp 无法对应其数据快照；
- RViz 开启导致 projection deadline、adapter lease 或控制周期超时；
- 新 goal tool 绕过 Goal Manager 或不能 cancel 当前 action。
