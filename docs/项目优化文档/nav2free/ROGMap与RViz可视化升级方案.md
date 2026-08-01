# ROGMap 与 RViz 自研导航可视化方案

更新时间：2026-08-01。

RViz 是自研导航的只读诊断旁路，不得成为 planning、ESDF 数值或安全授权的输入。
规划器只能消费 ROGMap 数值 service/结构化 snapshot，禁止从 `/rog_map/esdf` 的
`PointCloud2` 反解析距离。

## 1. 当前实现

活动节点 `src/ats_sentry_nav/ats_rog_map/src/ats_rog_map_node.cpp` 当前发布：

| topic | 类型 | 内容 | 当前状态 |
| --- | --- | --- | --- |
| `/rog_map/occ` | `sensor_msgs/msg/PointCloud2` | raw occupied voxels | 已接入 RViz，红色 boxes |
| `/rog_map/inf_occ` | `sensor_msgs/msg/PointCloud2` | ROG inflation occupied | 已接入 RViz，橙色半透明 boxes |
| `/rog_map/unk` | `sensor_msgs/msg/PointCloud2` | unknown voxels | 已接入 RViz，灰色透明 boxes |
| `/rog_map/esdf` | `sensor_msgs/msg/PointCloud2` | 高度切片，intensity 为 signed distance | 已接入 RViz，仅 debug |
| `/rog_map/bounds` | `visualization_msgs/msg/MarkerArray` | 当前 local map AABB wireframe | 已接入，namespace `rog_map_bounds` |
| `/rog_map/stale` | `std_msgs/msg/Bool` | map/odom stale | 已接入 health 链 |

节点只在有订阅者时构建 debug cloud/Marker。当前已经有四类点云和单个 local bounds；
ROGMap 的 visualization/update/search 多边界、MINCO search/repair marker、结构化
health panel 和 live RViz 截图仍未完成。

## 2. 外部参考边界

参考文章：<https://blog.csdn.net/CUN_CUI/article/details/158610185>。

文章中关于机器人中心滑动地图、概率 occupancy、inflation、ESDF 和多彩边界框的
描述用于设计映射；文章中的频率、耗时、内存或其他项目结果不是 ATS 实测，不能写入
验收阈值。实现必须遵守仓内许可证和活动源码边界，不复制参考项目节点。

## 3. 目标显示层

```text
01 Robot & TF
  RobotModel / TF / localization odom path
02 Sensor
  registered scan / LiDAR frame
03 ROGMap 3D
  raw occupancy / inflation / unknown / ESDF slice / local bounds
04 Ground Planning
  static map / planning grid / signed distance / footprint clearance / terrain/slope
05 JPS & MINCO
  goal / raw path / candidate reference / committed reference / search/repair markers
06 MPC & Chassis
  MPC reference / predicted path / odom-GT / swerve telemetry
07 Health & Safety
  localization / map lease-generation / planner status / execution stop / diagnostics
```

旧 costmap、MPPI rollout、transformed global plan、`/plan` 和 Nav2 GoalTool 不属于目标
显示层。当前 RViz 文件仍保留部分旧 display 作为 disabled/legacy 资产，必须在自研
入口和 headless 配置验收通过后删除，而不是继续扩大其使用范围。

## 4. bounds 设计

### 4.1 已实现

`ats_rog_map` 当前只发布一个由 Sliding Map 实际边界生成的 `LINE_LIST` Marker：

- frame 使用地图数据的 `map_frame`，当前正式参数为 `odom`；
- stamp 使用最近地图 stamp，不用 `now()` 掩盖 stale；
- namespace `rog_map_bounds`、id `0` 固定覆盖；
- 只表示 local map 数据范围，不代表 occupancy、clearance 或可通行性。

### 4.2 目标扩展

| 边界 | owner | 颜色 | 状态 |
| --- | --- | --- | --- |
| local map range | `ats_rog_map` | 橙色 | 已实现一个 AABB |
| visualization range | `ats_rog_map` | 紫色 | 未实现 |
| current update range | `ats_rog_map` | 绿色 | 未实现 |
| JPS/MINCO search range | `minco_planner` | 淡蓝色 | 未实现 |

每个边界都必须有自己的 snapshot stamp、frame、namespace 和 id；地图 reset 时发布
DELETE 或完整替换。ROGMap 不得猜测 JPS search range，planner 不得发布 ROGMap local
range。

## 5. 显示参数约定

### 5.1 occupancy/inflation/unknown

- `/rog_map/occ`：Boxes，尺寸跟随 `rog_map.resolution`，红色，alpha 0.90；
- `/rog_map/inf_occ`：Boxes，尺寸跟随 inflation resolution，橙色，alpha 0.40；
- `/rog_map/unk`：Boxes，灰色，alpha 0.15，默认可关闭以减少视觉负担；
- 三层必须颜色和透明度不同，不能把物理障碍、规划 margin 和 unknown 混成一层。

### 5.2 ESDF debug

`/rog_map/esdf` 使用 XYZI 的 intensity 显示 signed distance：负值 occupied、正值
free clearance、unknown 为 NaN；显示范围必须与实际截断参数一致。ESDF cloud 只做
诊断，不可作为 adapter/MINCO/MPC 数值输入。

### 5.3 planning/control/health

规划显示应优先使用结构化 topic：`/rc_esdf/planning_grid`、
`/rc_esdf/signed_distance_grid`、`/rc_esdf/footprint_clearance_grid`、
`minco/raw_path`、`minco/reference_path`、candidate/reference status、MPC predicted
path 和 `SwerveTelemetry`。对于尚未存在的 `PlanningMapSnapshot`、`PlannerCandidate`
和 health panel，RViz 只能标记未实现，不能伪造 topic。

## 6. 目标交互

当前 `SetGoal` 发布 `/goal_pose`，Goal Manager 的 `input_goal_topic` 是唯一 consumer，
负责 transform、identity、规划、cancel/preempt、stop 和 execution command。RViz 不得
直发 `/ats_goal_manager/planner_goal`。

后续 `ats_rviz_plugins/NavigateToPoseTool` 若实现，必须：

1. action client 只指向 `/ats_navigate_to_pose`；
2. 点击位置和拖动 yaw 组成 `PoseStamped`；
3. 新目标前显式 cancel/preempt 当前 goal；
4. 显示 feedback/result，不用 Marker 冒充 action 状态；
5. server 不可用或失败时不自动重试。

## 7. 实施文件与验收

| 仓库 | 文件/范围 | 任务 |
| --- | --- | --- |
| 导航仓 | `ats_rog_map/src/ats_rog_map_node.cpp` | 扩展 visualization/update bounds snapshot |
| 导航仓 | `ats_rog_map/include/rog_map/rog_map_engine.hpp` | 暴露只读边界 API，保留 frame/stamp |
| 导航仓 | `minco_planner/src/nodes/minco_planner_node.cpp` | search、footprint、repair Marker |
| 根仓 | `ats_sentry_bringup/rviz/sentry_default_view.rviz` | 删除旧 display，增加自研 health/规划组 |
| 导航/接口仓 | message/diagnostics | 只有 schema 落地后才增加结构化 display |

静态验收：

- RViz 文件无活动 `global_costmap`、`local_costmap`、`transformed_global_plan`、
  `/plan` 和 `nav2_rviz_plugins`；
- 每个 enabled topic 在活动源码有 producer，frame、QoS、depth 一致；
- bounds namespace/id/frame/stamp 与 owner 一致；
- Fixed Frame 使用当前存在的 `map`/`odom` TF，不依赖缺失 frame。

运行验收：

1. 无 RViz 启动 MuJoCo，证明规划 generation、action 和控制不依赖可视化；
2. 启动 RViz，确认 cloud 非空且不改变 planning generation、deadline 和 MPC 周期；
3. 检查 bounds 随地图滑动、update range 随输入变化、search range 随 goal 变化；
4. 关闭 RViz，确认 debug 发布按 subscription count 降低；
5. 保存 desktop/目标显示器截图，检查颜色、alpha、文字和显示边界无重叠。

## 8. 停止条件

- 为显示效果修改 occupancy、unknown、inflation、ESDF 或 footprint 语义；
- adapter、MINCO 或 MPC 开始订阅 `/rog_map/esdf` 点云；
- bounds frame/stamp 无法对应同一 snapshot；
- RViz 造成 projection deadline、adapter lease 或 MPC deadline 失败；
- goal tool 绕过 Goal Manager，或失败后自动重试旧目标；
- 没有独立 contact evaluator 却把离散 collision/Marker 结果写成物理零碰撞。
