# ATS Sentry 导航优化 TODO

更新时间：2026-08-03

本文是 ATS 四驱四转舵轮哨兵的执行清单，不把“源码对比”“编译通过”或“topic 存在”写成闭环通过。每一项优化必须在根仓、导航仓、MuJoCo 仓分别检查归属；只提交本轮显式列出的文件，并在 `develop` 上完成对应提交和 `origin/develop` 推送。当前推送凭据在本机不可用，已提交内容必须保留本地并在凭据恢复后重试，禁止 force push。

## Definition of Done

- ROGMap：活动 ROS 2 wrapper 与官方 ROG-Map 的滑窗、机器人中心可视化范围、raycast 更新框、frame、时间和 occupancy/ESDF 语义有逐项对照；仿真和实机使用同一正式参数契约；RViz 能区分橙色 Local Map Range、紫色 Visualization Range、绿色 Raycast Update Range；淡蓝色 JPS 搜索框由 JPS/MINCO 所有者发布，不写入 ROGMap 数值服务。
- MINCO：轨迹至少满足离散 footprint 安全、段间位置/速度/加速度连续、有效 reference 时间单调；规划存在但机器人位姿在地图内长期无进展时触发有界重规划，不能因为一个不可行点永久停死；地图 stale、目标不可达、优化/修复失败仍 fail-closed 零速度。
- RViz：全局控制路径、局部控制/reference、MPC prediction 使用独立 topic/display 名称、颜色、线宽和 QoS；不将调试 Marker 当数值规划输入。
- 验收：最窄单测、包构建、launch Python/`--show-args`、隔离 DDS domain 的 MuJoCo nominal/red-box/fault case、话题唯一所有权和安全停机证据齐全；实车 Gate 0--3 未完成前不声称实车通过。

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
- 未完成：淡蓝色 JPS 搜索框、全局/JPS 与局部 MINCO/MPC 路径的 RViz 独立 topic/display 尚未实现；MINCO 轨迹平滑、reference 恢复和 `PlanProgressWatchdog` 尚未实现。

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

- [ ] 全局控制路径：保留 JPS/MINCO raw path，标记为 `Global Planning / JPS`，淡蓝色；只表达任务级拓扑搜索结果。
- [ ] 局部控制路径：`/minco/reference_path` 使用绿色或黄色，标记为 `Local Control / MINCO Reference`；不与全局路径共用 display 名称。
- [ ] MPC：`/mpc/reference` 与 `/mpc/predicted_path` 分别标为 `MPC Follow Reference`、`MPC Predicted`，使用高对比颜色/线宽和独立 group；显示 timestamp/TF frame 一致性。
- [ ] MuJoCo 与实机 RViz 配置保持同一 topic、QoS、fixed frame；只在 RViz 配置层调整颜色/名称，禁止改 planner/control topic ownership。
- [ ] 增加截图回归和 `ros2 topic info --verbose` 唯一 publisher/subscriber 记录；截图非黑不等价于路径跟随通过。

## P2/P3/P4 边界

- P2 当前目标是 ROGMap ground projection、terrain/static wall/unknown 融合、唯一 planning-grid owner、单次 MINCO immutable snapshot 和安全停机；ROS 2 可视化框不改变这些数值语义。
- P3 只有 `launch_nav2:=false`、无 Nav2 servers、MINCO 不订阅 `/plan`、自研 action feedback/result/cancel/preempt/timeout 全部运行验证后才能标记 Nav2-free。
- P4 连续 swept footprint、真实动力学/制动、HIL 和实车验证未完成；MuJoCo `contact_violation_count=0` 不能推出实车物理碰撞为零。

## 提交与作者约束

- 每一项功能提交正文写明“为什么改、frame/time/map/ESDF/generation/ownership 契约、验证结果、未覆盖范围”。
- 根仓只提交 `docs/`、`scripts/`、`src/ats_sentry_bringup`；导航仓只提交 `src/ats_sentry_nav` 归属文件；MuJoCo 仓只提交 `src/sim/ats_mujoco_sim` 归属文件。
- 禁止 `git add .`、`git add -A`、历史重写和 force push；只允许 `develop`，提交作者固定为 `liukong1220 <1625038134@qq.com>`。
- 当前本机 GitHub HTTPS push 因缺少用户名/凭据失败；本地提交保留，凭据恢复后逐仓执行 `git push origin develop`，并记录本地 HEAD 与 `origin/develop` 一致性。
