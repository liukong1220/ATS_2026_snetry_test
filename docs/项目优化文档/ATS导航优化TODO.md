# ATS Sentry 导航优化 TODO

更新时间：2026-08-04

本文是 ATS 四驱四转舵轮哨兵的执行清单，不把“源码对比”“编译通过”或“topic 存在”写成闭环通过。每一项优化必须在根仓、导航仓、MuJoCo 仓分别检查归属；只提交本轮显式列出的文件，并在 `develop` 上完成对应提交和 `origin/develop` 推送。三个仓库当前统一使用 `git@github.com:liukong1220/<repo>.git` SSH remote；禁止 force push、空提交和将未验证性能写成实测结论。

## Definition of Done

- ROGMap：活动 ROS 2 wrapper 与官方 ROG-Map 的滑窗、机器人中心可视化范围、raycast 更新框、frame、时间和 occupancy/ESDF 语义有逐项对照；仿真和实机使用同一正式参数契约；RViz 能区分橙色 Local Map Range、紫色 Visualization Range、绿色 Raycast Update Range；淡蓝色 JPS 搜索框由 JPS/MINCO 所有者发布，不写入 ROGMap 数值服务。
- MINCO：轨迹至少满足离散 footprint 安全、段间位置/速度/加速度连续、有效 reference 时间单调；规划存在但机器人位姿在地图内长期无进展时触发有界重规划，不能因为一个不可行点永久停死；地图 stale、目标不可达、优化/修复失败仍 fail-closed 零速度。
- RViz：全局控制路径、局部控制/reference、MPC prediction 使用独立 topic/display 名称、颜色、线宽和 QoS；不将调试 Marker 当数值规划输入。
- 验收：最窄单测、包构建、launch Python/`--show-args`、隔离 DDS domain 的 MuJoCo nominal/red-box/fault case、话题唯一所有权和安全停机证据齐全；实车 Gate 0--3 未完成前不声称实车通过。

## 下一阶段执行地图（P2.1--P2.4）

| 批次 | 直接行为所有者与文件范围 | 必须保持的契约 | 验收与停止条件 |
| --- | --- | --- | --- |
| P2.1 ROGMap 显示对齐 | 导航仓 `ats_rog_map/src/ats_rog_map_node.cpp`、`rog_map/prob_map.*`、两份正式参数；根仓/仿真仓 RViz 配置 | 橙色是滑窗存储边界、紫色是机器人中心可视范围、绿色是量化后的 raycast 更新范围；occupancy、unknown、signed distance、gradient 和 adapter 数值服务绝不从 Marker/PointCloud2 反解析 | 代码/engine 单测、launch 参数、MuJoCo `/rog_map/bounds` 三 namespace 和一张同帧截图；若 frame、时间戳或数值 projection 变更，停止并先补接口测试 |
| P2.2 JPS/MINCO/MPC RViz 语义 | 根仓 `sentry_default_view.rviz`、MuJoCo 仓 `mujoco_navigation.rviz`；producer 只读检查 `minco_planner_node.cpp`、`ats_swerve_mpc` | `/minco/raw_path` 是 JPS/A* 全局搜索引导线，淡蓝；`/minco/reference_path` 是局部时间 reference，绿；MPC reference horizon 黄、predicted rollout 品红；显示层不得新增控制/规划 writer | RViz 配置语法、topic/QoS 账本、MuJoCo 截图；topic 不存在、frame 不一致或 publisher 非唯一时不宣称显示验收通过 |
| P2.3 平滑与 reference 恢复 | 导航仓 `minco_planner`、`ats_goal_manager`、`ats_swerve_mpc` 及最窄 GTest | 仅在同一 immutable map snapshot、目标 epoch、localization identity 和 heartbeat 有效时重定时并发布新 reference；急停前旧 reference 永不复活 | 先为 JPS corner、S3 continuity、reference 时间单调、旧 reference 拒绝/恢复添加确定性单测；红框前必须独立复核离散 footprint |
| P2.4 无进展有界重规划 | 导航仓新增/扩展 `PlanProgressWatchdog`，由 Goal Manager 拥有任务级状态机，MINCO 只拥有单次规划 | steady-clock stall deadline、inside-map/free/footprint 验证、goal epoch、snapshot generation、最小重规划间隔、最大尝试次数；stale/unknown/TF failure/no-path 均 fail-closed 零速度 | 单点不可过、机器人冻结、map stale、lease stale、unreachable 分别使用新 DDS domain 和新 MuJoCo；超过最大恢复次数必须保持急停，不允许循环重规划 |

### P2.1/P2.2 当前状态

- [x] P2.1 核心实现已推送：ROS 2 wrapper 已按官方可视范围/滑窗裁剪语义发布三种 ROGMap bounds，并保留数值 projection 契约。
- [x] P2.2 display 语义重命名：`/minco/raw_path` 的 producer 是 `GridJps::plan()` 返回的 `search_result.path`，因此淡蓝色 display 必须名为 `Global Planning / JPS Search Path`，不能误称 `MINCO Raw Path`。
- [ ] P2.1/P2.2 运行期截图：尚缺当前 revision 的同帧 bounds 三色框与四层路径截图；这不是完成闭环的替代品。

## 已完成的第一步：ROGMap 分层可视化

### 证据与差异

| 项目 | 官方 ROG-Map 证据 | ATS 活动实现证据 | 决策 |
| --- | --- | --- | --- |
| 可视化范围 | `rog_map/src/rog_map/rog_map.cpp` 用 `robot_state_.p +/- visualization_range / 2`，随后 `boundBoxByLocalMap` | `ats_rog_map_node.cpp` 原来直接用整块 `getLocalMapOrigin/getLocalMapSize` | debug 点云改用机器人中心紫色范围，保留橙色整块滑窗框 |
| 局部更新框 | 上游 `ProbMap::updateLocalBox` 量化并裁剪到滑窗 | 活动 wrapper 原来未发布 `raycast_data_.local_update_box_*` | 新增只读诊断 getter，绿色框只表示实际 raycast 更新范围 |
| 淡蓝搜索框 | 官方 A* `rog_astar.hpp` 在每次 `pathSearch` 依据 start/goal 构造临时搜索框 | ATS JPS 目前只返回 `GridAstarResult`，无搜索框 debug 输出 | 下一小项由 `minco_planner` 发布，ROGMap 不拥有它 |
| 数值 ESDF | 上游/ATS 核心保留 signed distance | adapter 直接调用 `/rog_map/get_ground_projection` | 不从 `/rog_map/esdf` `PointCloud2` 反解析距离 |
| 参数源 | ROS 2 wrapper 从 `core.*` 构造 `rog_map::Config` | 正式 root/MuJoCo profile 原来 range 为 `[0,0,0]` | 正式 profile 设 `[8,8,1]`，并保留 `map_size=[10,10,1]` 为橙色存储边界 |

### 本轮实现范围

- `ats_rog_map::ProbMap::getRaycastLocalUpdateBox()` 只读返回核心已经计算好的量化、裁剪后更新框；增加 engine 回归断言。
- `/rog_map/bounds` 发布稳定 namespace/id 的六个 Marker：橙色 `Local Map Range`、紫色 `Visualization Range`、绿色 `Raycast Update Range`，各自带可删除的文本标签，避免旧框残留。
- occupied/inflated/unknown/ESDF debug cloud 与紫色可视化框使用同一 frame 和范围；projection service 的 map bounds、occupancy、signed distance、gradient 数组不改变。
- `src/ats_sentry_bringup/params/node_params.yaml` 与 `ats_rog_map/config/rog_map_ground_planning_mujoco.yaml` 共同声明有效 debug 参数；实机默认仍保留 `launch_fake_vel_transform:=True` 与 `launch_chassis_vel_transform:=True`。

### 第一阶段验证清单

- [x] `MAKEFLAGS=-j1 colcon build --base-paths src --packages-select ats_rog_map --parallel-workers 1`；构建通过。
- [x] `colcon test --base-paths src --packages-select ats_rog_map` 与
  `colcon test-result --test-result-base build/ats_rog_map --verbose`；7 tests、0 errors、0 failures、0 skipped。
- [x] `python3 -m py_compile` 受影响 launch；`ros2 launch ats_rog_map ats_rog_map.launch.py --show-args`；语法与参数展开通过。
- [x] MuJoCo 新 DDS domain `156` 运行期检查：`/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 非空，adapter `ready=true` 且 generation 递增，`/rc_esdf/planning_grid` 唯一 publisher 为 `ats_rog_map_adapter`。`/rog_map/bounds` 的三组 namespace 由实现固定发布；本轮未保存独立 RViz bounds 三色截图，标记为待补证据。
- [x] ROGMap projection 快照时间修复后，日志连续出现 `generation=164..206`、`cells=10000`、`stale=false`；实测单次 projection 约 `1.15--1.56 s`，配置 deadline 为 `4.0 s`，adapter generation 持续递增。该证据证明数值快照与 adapter 链路新鲜，不证明端到端编号一致。
- [ ] MuJoCo 红框命令：
  `PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh`。
  本轮已生成候选 `generation=50 raw_points=3 reference_points=88 length=1.89 time=6.28 collisions=0 expanded=3 minimum_clearance=0.157`，但目标未到达：机器人约从 `(-10.59, 1.55)` 移到 `(-10.34, 1.05)`，最终 `distance_remaining≈1.52 m`，MPC 多次保持零速度。日志显示急停后旧 reference 被正确拒绝（`Ignoring a trajectory older than the latest emergency stop`），随后没有新的有效 reference；action 在 `180 s` 超时。因此红框不通过，不能声称 P2 闭环通过。MuJoCo 独立 contact evaluator 未接入，物理接触为“未验证”。
- [x] 实机 Gate 0 的源码/参数/构建边界已记录；Gate 2 HIL、Gate 3 低速实车和真实 RViz/运动均未运行，禁止声称实机通过。

### 第一阶段运行证据与遗留项

- 已验证：ROGMap local map、robot-centred visualization range、raycast update range 的实现与官方源码语义对照；projection 数值服务保持 occupancy、signed-distance、gradient、unknown 语义；MuJoCo CPU LiDAR 在 MuJoCo 3.x 下可正常调用。
- 已验证：adapter 不订阅 `/rog_map/esdf` 点云，直接消费 `/rog_map/get_ground_projection`；`planning_grid_owner:=rog_map` 时规划栅格只有一个 publisher。
- 已实现未运行：淡蓝色 display 已按 `/minco/raw_path` 的真实 JPS producer 重新命名；根仓与 MuJoCo RViz 配置分别保持同一语义。截图回归、MINCO 轨迹平滑、reference 恢复和 `PlanProgressWatchdog` 仍未完成。

## 第二步：MINCO 轨迹平滑与有界重规划

### 根因假设（待实验区分）

1. **已知风险**：JPS 离散折线的角点直接成为 MINCO 引导点，若时间分配/导数约束尺度不一致，会出现速度突变或角点摆动。
2. **已知风险**：footprint gate/Local Collision Repair 失败后当前路径可能进入 fail-closed 停止，但没有“地图内长期无进展”的任务级重规划触发器。
3. **待区分假设**：停止来自地图 stale、控制 reference 被拒绝、MPC solver failure、真实碰撞还是机器人位姿没有推进；不能只提高 planner frequency 或 MPC gain。

### 实施顺序

- [ ] 在 `minco_planner` 建立 `PlanProgressWatchdog`：用 `map_snapshot` frame/时间、机器人当前位姿、目标 epoch、reference generation、最后有效命令和进展距离建立状态；机器人在规划地图内且距离目标下降小于阈值持续 `replan_stall_timeout_sec` 才触发重规划。
- [ ] 触发器使用 steady clock deadline，带最小重规划间隔、最大连续重规划次数和 goal epoch；旧 goal、旧 generation、急停前 reference 不得复活。
- [ ] 重规划入口先读取机器人在当前 `PlanningMapSnapshot` 的位置并验证 inside-map/free/footprint；地图 stale、unknown、TF 失败或目标不可达直接零速度。
- [ ] MINCO 优化先锁定端点和 yaw，再用尺度一致的 segment duration、位置/速度/加速度连续性和 clearance；对最终轨迹做独立离散 footprint 复核，不能用 solver success 代替安全。
- [ ] 以简单 JPS polyline + 固定速度 baseline 做消融：记录 path length、最小 clearance、曲率/加加速度 proxy、tracking error、replan count、solver wall time p50/p95/p99。
- [ ] 分别注入“一个中间点不可过”“机器人不动但地图新鲜”“Point-LIO stale”“adapter lease stale”，每个故障使用新 DDS domain 和新 MuJoCo launch，验证 `ready=false -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0`。

## 第三步：RViz 全局/局部/MPC 路径分层

- [x] 全局控制路径：`/minco/raw_path` 已标记为 `Global Planning / JPS Search Path`，淡蓝色；只表达 JPS/A* 的任务级拓扑搜索结果。
- [x] 局部控制路径：`/minco/reference_path` 已标记为 `Local Control / MINCO Timed Reference`，绿色；不与全局路径共用 display 名称。
- [x] MPC：`/ats_swerve_mpc/reference_horizon` 已标记为 `MPC Follow / Reference Horizon`（黄），`/ats_swerve_mpc/predicted_path` 为 `MPC Follow / Predicted Rollout`（品红）；使用独立 display。
- [x] MuJoCo 与实机 RViz 配置使用同一 topic、QoS、fixed frame 语义；本轮只调整显示名称，未改变 planner/control topic ownership。
- [ ] 增加截图回归和 `ros2 topic info --verbose` 唯一 publisher/subscriber 记录；截图非黑不等价于路径跟随通过。

## P2/P3/P4 边界

- P2 当前目标是 ROGMap ground projection、terrain/static wall/unknown 融合、唯一 planning-grid owner、单次 MINCO immutable snapshot 和安全停机；ROS 2 可视化框不改变这些数值语义。
- P3 只有 `launch_nav2:=false`、无 Nav2 servers、MINCO 不订阅 `/plan`、自研 action feedback/result/cancel/preempt/timeout 全部运行验证后才能标记 Nav2-free。
- P4 连续 swept footprint、真实动力学/制动、HIL 和实车验证未完成；MuJoCo `contact_violation_count=0` 不能推出实车物理碰撞为零。

## 提交与作者约束

- 每一项功能提交正文写明“为什么改、frame/time/map/ESDF/generation/ownership 契约、验证结果、未覆盖范围”。
- 根仓只提交 `docs/`、`scripts/`、`src/ats_sentry_bringup`；导航仓只提交 `src/ats_sentry_nav` 归属文件；MuJoCo 仓只提交 `src/sim/ats_mujoco_sim` 归属文件。
- 禁止 `git add .`、`git add -A`、历史重写和 force push；只允许 `develop`，提交作者固定为 `liukong1220 <1625038134@qq.com>`。
- 每次改动后先以 SSH `git push origin develop` 推送有改动仓库，再记录本地 HEAD 与 `origin/develop` 一致性；无改动仓库不得制造空提交。全历史 `git shortlog -sne --all` 必须只显示 `liukong1220 <1625038134@qq.com>`。

## 下一方执行提示词

```text
继续 ATS Sentry P2.3/P2.4，先读取 AGENTS.md 和 docs/项目优化文档/ATS导航优化TODO.md。
目标：修复“JPS/MINCO candidate 已生成、急停后旧 reference 被 MPC 拒绝、机器人长期零速且不重规划”。
只修改拥有行为的 minco_planner、ats_goal_manager、ats_swerve_mpc 与最窄测试；不要改 Point-LIO、ROGMap 数值 projection、RC-ESDF signed-distance/unknown 语义，也不要把 /rog_map/esdf PointCloud2 当数值输入。
实现 PlanProgressWatchdog：以 steady clock、同一 goal epoch、immutable snapshot generation、localization identity、inside-map/free/footprint、最小重规划间隔、最大连续尝试次数为条件。地图 stale/unknown、TF 失败、无路、unsafe trajectory 或次数耗尽必须保持 emergency_stop=true、/cmd_vel_mpc=0、/motion_control=0；恢复时只能发布急停之后重新定时且经提交点复核的新 reference。
先补 deterministic 单测（reference 时间单调、旧轨迹不得复活、冻结无进展触发一次有界重规划、次数耗尽停机），再构建、运行 MuJoCo 新 DDS domain 的 nominal/red_box/单点不可过/冻结/stale/unreachable。每个有改动仓库显式 stage、中文详细提交、SSH push；最终报告实际 terminal 坐标、位置误差、replan 次数、最小 clearance、离散 footprint 冲突、MPC reference/predicted、两级速度、contact evaluator 与未运行实车 Gate。
```
