# ATS Sentry 导航优化 TODO

更新时间：2026-08-09

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

### P2.3/P2.4 本轮实现与运行证据（2026-08-04）

- [x] 已实现：`PlannerGoal` 与 `PlannerStatus` 增加单调
  `plan_request_sequence`；Goal Manager 只接受相同 `goal_id`、
  `localization_epoch`、request sequence 的 candidate/status。急停或恢复后必须重新发起
  request，不能让迟到的 candidate、旧 generation 或急停前 reference 恢复执行。
- [x] 已实现：adapter 发布可靠、transient-local 的数值 `PlanningMapSnapshot`；其中包含
  fused OccupancyGrid、signed distance、世界系 gradient、source generation、localization
  epoch、publication sequence 与 origin/yaw。Goal Manager 提交 reference 前用当前 snapshot
  再复核 freshness、frame、inside-map、free cell 与 `0.70 x 0.55 m + margin` 定向 footprint；
  unknown/outside/occupied 均 fail-closed。
- [x] 参数证据：ROGMap 输入 `input_sync_tolerance_sec` 从 `1.8` 调整到 `2.0`，对齐 181
  域日志实测 projection 与 terrain/slope stamp delta `1.83--1.94 s`；没有放宽
  `input_timeout_sec`、projection deadline、unknown/occupied 语义或 MPC 旧 reference 拒绝。
- [x] 已实现并通过确定性单测：`PlanProgressWatchdog` 使用 steady clock、
  `progress_min_delta_m=0.10`、`replan_stall_timeout_sec=4.0`、
  `replan_min_interval_sec=2.0`、`max_consecutive_replans=2`。只有 health、TF、inside-map、
  free cell、footprint 与有效 reference 全部成立时才统计无进展；超限返回
  `RESULT_PLANNING_FAILED` 并保持急停。health/TF 暂态进入 map-wait/recovery，不把跨 DDS
  topic 乱序错误归类为 task failure。
- [x] 最窄验证：构建
  `ats_navigation_interfaces ats_rog_map_adapter ats_goal_manager minco_planner ats_sentry_bringup ats_mujoco_sim`
  成功；Goal Manager 的 lifecycle/watchdog/snapshot/epoch 共 4 项、adapter 的 fusion/snapshot
  共 2 项、MINCO 的 JPS/optimizer/reference/atomic/footprint 共 5 项通过。完整
  `colcon test` 中 `minco_planner` 的 8 项 GTest 通过；包级 lint 对既有 28 个文件报告
  `copyright`、`cpplint`、`clang_format` 共 635 处风格偏差，不能写成全包通过。
- [x] MuJoCo domain `185` headless nominal：ROGMap owner、JPS/MINCO、Goal Manager、SE2
  MPC、twist bridge 与 MuJoCo 形成一条成功闭环。终点采样为
  `(-8.999925, 1.490465)`，到 `(-9.0, 1.47)` 的脚本误差 `0.020465 m`；
  `/minco/raw_path` 18 poses、`/minco/reference_path` 312 poses，末次 MINCO record 为
  `generation=52 raw_points=2 reference_points=5 minimum_clearance=0.397
  footprint_collisions=0`；MPC reference/predicted 各 3782 poses，adapter generation 从
  `35` 增至 `76`，`/cmd_vel_mpc` 与 `/motion_control` 均观察到非零流且最终为零。
  MuJoCo telemetry 的 `contact_violation_count=0` 与四轮 RPM 为零，只是该仿真 evaluator
  的结果，实车/HIL 物理接触仍未验证。[Confidence: High，脚本、topic ownership 与日志交叉证据]
- [ ] red-box domain `186` 未通过：`stage_red_box` 成功到
  `(-8.881671, 1.453885)`，误差 `0.016202 m`；第二段在约 `74.2 s` 以
  `RESULT_MAP_UNREADY=4` 中止，最终 pose `(-9.215931, -0.076150)`、
  `final_distance=10.011419 m`。日志有 projection `2--3 s`、输入 stale、MPC odometry
  timeout 与多次 health-induced recovery；长路线 MINCO candidate 的
  `minimum_clearance` 约 `0.067--0.102`，均为离散 `footprint_collisions=0`，但没有完整
  到达或最终 contact telemetry，红框不得判为通过。
- [ ] 冻结 fault：MuJoCo 已新增运行时 `freeze_motion`，它只拒绝底盘执行并保持仿真、LiDAR、
  odometry、localization 与 ROGMap 发布；脚本在 nominal 后设置该参数，要求健康状态、
  1--2 次 watchdog replan、`RESULT_PLANNING_FAILED=5`、两级零速度和 generation 继续增长。
  domain `188` 暴露了“启动即冻结”的脚本设计错误，已改为运行时切换；domain `189` 因已有用户
  `rviz2` 占用单核、load/swap 升高而在 `/localization` 首次发现前超时。最终冻结闭环尚未运行，
  不能将接口实现写为 fault 通过。
- [ ] adapter lease、projection service timeout、Point-LIO input stale、unknown、unreachable
  与中间点不可过仍需在无 viewer、低负载环境中各用独立新 DDS domain 重跑；每例必须记录
  `ready=false -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0`，恢复时还须确认无
  新目标不会复活旧 response/reference。

### 实施顺序

- [x] `PlanProgressWatchdog` 由 Goal Manager 拥有；它记录 goal/localization/request identity、
  snapshot publication/source generation、距离、steady-clock 进展时间、重规划间隔与连续次数。
- [x] 重规划提交遵守新 request、新 snapshot 复核、同一互斥区内重定时后
  `emergency_stop=false` 再发布 reference 的顺序；旧 goal、旧 generation、急停前 reference
  不得复活。
- [x] 重规划入口在当前 `PlanningMapSnapshot` 上验证 inside-map/free/footprint；地图 stale、
  unknown、TF 失败或目标不可达保持确定性零速度。故障运行闭环仍按上文待补。
- [ ] MINCO 优化先锁定端点和 yaw，再用尺度一致的 segment duration、位置/速度/加速度连续性和 clearance；对最终轨迹做独立离散 footprint 复核，不能用 solver success 代替安全。
- [ ] 以简单 JPS polyline + 固定速度 baseline 做消融：记录 path length、最小 clearance、曲率/加加速度 proxy、tracking error、replan count、solver wall time p50/p95/p99。
- [ ] 分别注入“一个中间点不可过”“机器人不动但地图新鲜”“Point-LIO stale”“adapter lease stale”，每个故障使用新 DDS domain 和新 MuJoCo launch，验证 `ready=false -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0`。

### MPC LTV-QP 迁移（第一阶段已开始）

- [x] 提取共享 `Se2Model`：iLQR 与未来 QP 共用同一 SE(2) dynamics、Jacobian 和 rollout，避免 A/B 对比时混入两套运动学。
- [x] 增加带滞回的 `ZeroSpeedGuard`：轮速向量接近零时不使用舵角方向线性化；当前 Twist-only 链路不实现原地独立舵角控制。
- [x] 增加 solver-independent `LtvQpBuilder`：生成线性化动力学、状态/控制/控制增量二次代价、车体速度和增量边界，并拒绝 malformed/non-finite horizon。
- [x] 已将 QP 公开头隔离到 `ats_swerve_mpc/include/ats_swerve_mpc/qp/`；共享 `Se2Model`、
  iLQR controller、tracker 和 ROS node 保持在 MPC 层，避免 solver API 与主控制算法混放。
- [x] 接入已批准的 OSQP v1.0.0 源码快照（固定 tag/commit/archive SHA-256、LICENSE/NOTICE），
  构造固定 LTV CSC pattern，首次 setup 后只更新数值并支持 primal/dual warm-start；当前仅 `ilqr`
  和 `qp_shadow`，未切换生产控制器。
- [ ] 只对跟踪误差使用有界 slack；轮速、舵角速率、碰撞、急停和输入健康约束保持 hard constraint；QP timeout/infeasible/slack 超限必须进入现有零速度 fail-stop。
- [ ] 在 MuJoCo 中加入 yaw ±π、速度阶跃、轮速过零、QP timeout/infeasible 和舵角限位 telemetry 回归，再进行 iLQR/QP A/B；OSQP 组件测试已通过，但未完成这些证据前不得宣称 QP 实时性或实车收益。

#### QP-0：后端准入与接口冻结

- [x] **历史接口冻结（2026-08-06，已由后续 OSQP/Shadow 实现替代）**：当时新增 backend-neutral `LtvQpSolver`、固定
  CSC pattern、primal/dual warm-start payload、完整 result 状态字段及
  `LtvQpCandidateValidator`。独立复核会拒绝 non-finite/维度、deadline、residual、非 `solved`
  status、输入健康/急停/collision、body 速度/加速度、真实轮速、轮速度向量增量、有效舵角速率和
  slack/hard-bound 违规；方向未定义时走 `ZeroSpeedGuard`，不生成伪舵角。该历史状态中尚未接入
  ROS node/`solver_mode`；后续条目已完成 OSQP 与 `qp_shadow` 接线，不能再将本段作为当前事实。
- [x] **OSQP v1.0.0 后端准入（2026-08-06）**：官方 tag `236713ce9a56c182ac3230d52108f952afce1523`、
  archive SHA-256 `dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48`、Apache-2.0
  `LICENSE`/`NOTICE`、QDLDL/AMD 等第三方声明已核验；导航仓使用版本控制源码快照和
  `osqp::osqpstatic`，不使用 apt/未知系统库/运行时下载。详见
  `docs/ats_swerve_mpc_ltv_qp_backend_admission.md`。

- [x] OSQP 版本、许可证、来源、CMake 依赖、目标环境、CSC 格式和 warm-start 能力已记录；固定
  `z=[delta_x_0...delta_x_N, delta_u_0...delta_u_N-1]`，当前不含 slack 列。
- [x] `LtvQpSolver` 结果契约包含 `solved`、`solved_inaccurate`、`max_iterations`、`time_limit`、
  infeasible/numerical 状态、primal/dual residual、iteration、solve/update time、slack 和 hard
  violation；deterministic GTest 覆盖固定模式、primal/dual warm-start、全部非 `solved` 状态拒绝、
  nonzero `delta_u` 重建、非线性 rollout、`ZeroSpeedGuard` 和外部健康 gate。
- [x] DoD（组件范围）：固定结构二次求解、pattern 漂移拒绝、warm-start 和真实 OSQP status 已通过；
  `qp` 主链仍禁止启用。
- [ ] 停止条件：后端不能在目标 Ubuntu/ROS 构建链以可复现方式链接，或许可证/依赖来源无法审计，则保持 `iLQR` 主链，不以自写未验证 QP 求解器替代。

#### QP-1：约束层级与低速语义

- [ ] 继续保持硬约束：`vx/vy/wz`、车体加速度、单轮最大速度、单轮速度增量、有效舵角速率、emergency stop、ExecutionCommand lease、localization/map/reference freshness 和已验证 footprint/collision gate。硬约束不得通过 slack 放松。
- [ ] 只为 tracking state/reference speed/terminal error 设置有界 slack；记录每个 slack 的上界、二次/一范数惩罚和触发次数。任何 slack 超过门限均不得下发控制，应进入现有零速度 fail-stop 或上层 recovery。
- [ ] 将四轮轮速圆以保守多边形或可信线性化写入 QP，并由独立真实轮速检查复核；不得用外接矩形放宽 `max_wheel_speed`。
- [ ] 将轮速增量作为线性约束加入 QP；舵角速率只在 previous/candidate wheel-vector 都高于 `ZeroSpeedGuard` 退出阈值时线性化。任一向量低速时只允许向量增量约束，不能创建伪方向角约束。
- [ ] 当前 Twist-only 接口无独立舵角命令。若需要静止预转向，必须另行定义下层 steer authority、反馈、限位与急停契约；在该接口完成前，不把“零速舵角优化”列为 MPC 已实现能力。
- [ ] DoD：含 yaw 跨 ±pi、前后反向、横纵切换和四轮过零的 GTest/property test 证明所有 QP hard constraints 与独立真实检查一致。

#### QP-2：Shadow 后端与结果复核

- [x] `qp_shadow` 已接入真实 OSQP v1.0.0 producer：控制周期先冻结 bounded
  `ControlCycleSnapshot`，iLQR 与 QP 只使用其中相同的 `current/reference/solve 前 last_control`、
  ExecutionCommand identity、localization epoch、reference frame/time；QP 使用 iLQR 名义 rollout
  构造固定 buffer/CSC、primal/dual warm-start 和真实 status/残差/solve/update 计时。QP primal 的
  `delta_u` 会重建完整控制并经共享 `Se2Model` 非线性 rollout 后再复核；只记录诊断，不发布 QP
  command。当前 collision/footprint 与 map freshness producer 缺失，candidate 保守 hard reject。

- [x] 新增 `solver_mode:=ilqr|qp_shadow|qp`，默认 `ilqr`；`qp_shadow` 不发布 QP command、不改变
  tracker/iLQR warm-start/急停/topic ownership；`qp` 参数显式拒绝启动。
- [x] 每次 QP 返回由 `LtvQpCandidateValidator` 复核维度、finite、状态码、deadline、residual、
  body/真实四轮速度、轮速度增量、ZeroSpeedGuard 有效舵角速率、slack/hard bounds 及输入 gate；
  非 `solved`、超时、残差、slack 或 hard gate 失败均不可行。
- [x] `qp_shadow` 用固定 128 槽环形 telemetry 记录 iLQR baseline 与 QP shadow 的 cycle sequence、
  same-snapshot identity、status、iteration、warm-start、OSQP reported/C API wall update+solve time、
  residual、hard margin、slack、两者首控及 delta、candidate/reject 与 collision/map gate；13 个
  steady-clock 阶段均可导出 count/p50/p95/p99/max。十类饱和 deadline root cause 分别覆盖 OSQP
  time-limit、OSQP solve、full callback、iLQR、QP build/update/audit、aggregation、logging 和 timer
  interarrival，不再保留会混淆归因的 merged miss。JSON 在 timer 外经只读
  `/ats_swerve_mpc/dump_control_telemetry` service 导出，日志不包含完整路径数组。`ControlCycleSnapshot` digest 已按固定小端字节序、
  字符串长度前缀和 FNV-1a-64 覆盖 current/reference time+frame+state/control、solve 前
  last_control 与 ExecutionCommand identity；不使用 DDS CDR。
- [x] DoD（组件/ROS gate）：`qp_shadow` 下输出仍由 iLQR 唯一发布，测试确认命令 publisher 数为 1；
  shadow 从单一 `ControlCycleSnapshot` 构造并且无 backend、reconstruction、residual 或硬门拒绝时
  不改写 iLQR command。
- [x] **MuJoCo shadow 已运行但停止条件触发（domain 229）**：在修复 CPU LiDAR
  `mj_multiRay(..., dist, None, nray, cutoff)` 后，raw cloud/`/registered_scan`、ROGMap、
  `/traversability_grid`、adapter heartbeat 和 generation `55 -> 138` 恢复；默认 iLQR action
  达到 `0.011173 m` 终点误差且两级 topic owner 唯一。该 nominal 不代表 QP 通过。8 条节流后的
  OSQP record 全部为 `max_iterations/400`、`solver_status_not_solved`、`warm=false`、
  `candidate_feasible=false`，各条 `same_snapshot=true`。最后 telemetry 为 solve
  `p50/p95/p99=3.829/5.424/5.874 ms`，但完整 callback 为 `57.611/131.704/160.563 ms`；该次有效
  参数为 `20 Hz/50 ms`，此前 `50 Hz/20 ms` 是错误口径，旧的 `61` 次 merged miss 无法归因。
  保持默认
  `solver_mode=ilqr`，禁止 `qp` 发布、deadline/iteration/residual 放宽和未 solved warm-start。
- [x] **QP-2.5 性能归因与收敛诊断（2026-08-07）**：新增 OSQP C API wall-time、矩阵尺度
  telemetry、128 槽固定阶段环、root-cause counter、只读 dump service、A/B/C 运行器和离线分析器。
  `ats_swerve_mpc` 的 `70 tests`、窄构建、affected launch `py_compile`、`--show-args` 和三仓
  `git diff --check` 通过。MuJoCo raw artifact 位于
  `/tmp/ats_qp25_profiles_20260807`：A `ilqr,warn`、B `qp_shadow,warn`、C `qp_shadow,info` 都在
  action/两级 owner/非零速度检查后成功导出 128 槽 JSON；外层 600 s 会话在 C 收尾时终止，因此
  runner 最终 PASS 未取得。B status 为 `128 max_iterations`，C 为 `126 max_iterations + 2 time_limit`；
  全部 candidate `solver_status_not_solved`、zero feasible、zero warm-start。B p50 为 callback
  `52.044 ms`、iLQR `31.350 ms`、QP build `12.084 ms`、OSQP wall solve `3.727 ms`、hard-check
  `4.100 ms`；C p50 分别 `114.200/87.984/17.269/5.067/5.740 ms`。A/B/C 的 scenario/revision/params
  一致，但实际 duration 与逐周期 digest 不同，分析 verdict `not_comparable`，所有 cross-run
  Shadow/INFO 增量成本结论保持 withheld。CPU/allocation 未验证；P2 未通过，P3 不得标记 Nav2-free。
- [x] 停止条件已执行：shadow 出现非有限矩阵、结构尺寸变化、超过采样/内存预算、修改现有 iLQR
  输出或破坏 emergency stop 时，必须保持 iLQR 主链并停止 QP 主链迁移。本轮符合“超过采样预算”和
  “non-solved status”两项，已停止在 shadow 证据边界；collision/footprint/map-health producer、
  P2/P3、HIL 和实车均未通过或未验证。

- [x] **QP-2.6 采样 owner 与离线数值归因（已实现）**：`ControlCycleTelemetryRing` 新增显式
  `telemetry_sampling_window_cycles` 固定窗口；首个有效 Execute lease/reference/map identity 冻结后，
  只接受同一 `manager_incarnation + goal_id + localization_epoch + map_generation +
  map_publication_sequence + reference stamp/deadline/frame` 的精确周期数。Execute heartbeat 的
  `command_sequence` 每拍递增，只记录并检查单调性，不错误地作为 lease 恒等字段。身份变化、lease/reference
  失效、localization/map generation 为零或窗口未收满时保留 raw fragment/manifest，schema 3 离线分析器固定输出 `not_comparable`，禁止
  任何 Shadow/INFO delta；旧 schema 2 artifact 仍可读取但因缺少窗口契约自动 withheld。
- [x] QP-2.6 raw sample 继续记录 20 Hz/50 ms、有效 OSQP 参数、domain/revision/scenario、status/iteration、
  residual/slack、十类 root cause、OSQP reported/C API wall time、candidate reject、两级 owner 所需外部
  证据字段和 CPU/allocation 未验证标记；builder metrics 新增 Hessian/row/bound/dynamic residual 尺度。
  当前 operational fixture 的可复现尺度断言为 Hessian `0.66..56.0`、非零 finite bound `0.1..2.15`、
  row L2 `1.0..1.42`、zero-delta dynamic residual `0`。该 proxy 不足以提出 scaling/preconditioning，
  不实施任何矩阵、准入参数或主链修改。manifest 另保存每仓 `HEAD`、tracked diff SHA-256 和关键有效
  QP 参数，不能把尚未提交的运行构建误标为纯 revision。
- [ ] QP-2.6 运行门禁：Python/Bash、source launch `--show-args`、三仓 `git diff --check` 已通过；本地
  `/tmp/ats_qdldl_v0_1_8` 注入后，窄构建、package GTest 12/12 和 `colcon test-result` 73 tests/0 failure
  均通过。临时 schema-3 fixture 验证逐周期 digest 不同稳定得到 `not_comparable`/withheld。新、空 domain
  `210` 的 headless A profile 仍在 `/traversability_grid` 前失败：`mujoco==3.4.0` CPU LiDAR 的
  `mj_multiRay()` `vec` 参数形状不兼容，`/registered_scan` 缺失使 ROGMap stale fail-closed；没有 raw
  telemetry/manifest，也未运行到 `/cmd_vel_mpc` 或 `/motion_control` runtime ownership、终点或 contact
  检查。本轮不修复该无关 MuJoCo binding，不能伪造 B/C。P2 不得标记通过，P3 不得标记 Nav2-free，
  HIL/实车/物理接触继续未验证。

- [x] **QP-2.6 后续输入链复核（2026-08-09，domain 213）**：当前环境 `mujoco==3.10.0` 下，
  headless MuJoCo LiDAR 子进程正常启动；`/local_pointcloud` 为 `front_mid360`、宽度 `787`，
  `/registered_scan` 为 `odom`、宽度 `104`，未复现 `mj_multiRay()` 崩溃。ROGMap 日志中的
  `cloud_age` 为有限值，source generation 从 `76` 增至 `90`，adapter `ready=1` 且
  `publication_sequence` 持续递增；脚本已确认 `/rog_map/occ`、`/rog_map/inf_occ` 非空，随后在
  `/rog_map/unk` gate 停止，因此 `/rog_map/esdf`、planning grid、action 和 ownership 尚未在该 profile
  中取得运行期证据。
- [ ] **QP-2.6 运行门禁仍未完成**：同一 domain 的 nominal profile 在 action、两级速度 owner、
  telemetry dump 前停止于 `/rog_map/unk` 非空检查。有效配置为
  `core.visualization.publish_unknown=false`，ROGMap/adapter 日志的运行 unknown cell 计数为 `0`；
  这不是 cloud stale、source generation 停止或 QP reject 证据。没有 raw schema-3 profile，不能计算
  A/B/C 配对成本，也不能宣称 QP shadow、终点、ownership 或 contact 通过。

#### QP-2.7：真实 unknown 场景与可配对 shadow 复核

- [ ] 先由实际 owner 设计受控 unknown 区域：优先使用 MuJoCo 地图/传感器遮挡或版本控制的独立
  `P2_FAULT_CASE=unknown` fixture，使 `/rog_map/unk` 产生真实非空 `PointCloud2`，并核对 frame、stamp、
  QoS、source generation 与 adapter publication sequence；不得发布伪点、把 `cloud_age=inf` 改成 fresh、
  关闭 stale/lease 或以静态假地图替代运行时 ROGMap。
- [ ] nominal、unknown fault、QP timeout/infeasible 各使用新的空闲 `ROS_DOMAIN_ID` 和独立 launch；
  nominal 不得被 fault 注入污染。unknown case 必须验证 all-unknown planning snapshot、
  `ready=false -> emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0`，恢复后 generation
  继续递增且急停前 reference 不复活。
- [ ] 只有 nominal 上游门禁、action、唯一 ownership 和固定 schema-3 telemetry 均完成后，才重跑
  A/B/C 配对窗口；窗口不完整、lease/reference/map identity 变化或 digest 不一致时 analyzer 必须保持
  `not_comparable`/`withheld`。不得调高 `qp_max_iterations`、放宽 `qp_time_limit_ms`/residual、保存
  non-solved warm-start 或启用 `solver_mode=qp`。
- [ ] 当前有效控制预算继续记录为 `20 Hz / 50 ms`；OSQP reported time、C API wall time、完整 callback、
  iLQR、hard-check、status、iteration、residual、slack 和十类 root cause 分开记录。CPU/allocation、
  P2 red-box、HIL、实车和物理接触继续未验证。

#### QP-2.7 本轮实现与停止记录（2026-08-09）

- **已实现且聚焦构建/单测通过**：`P2_FAULT_CASE=unknown` 不再通过 adapter fusion 后覆写栅格或伪造
  `/rog_map/unk`。MuJoCo `lidar_occlusion_enabled` 让 LiDAR worker 继续按原频率发布带有效
  header/stamp 的零点云；ROGMap 的边沿触发 `test_reset_to_unknown` 清空概率、inflation、frontier 和
  ESDF 表，并只递增 source generation；adapter 的 `test_mask_secondary_evidence` 只在确认 ROGMap
  数值 projection 已含 `-1` 时把 static/terrain/slope 置为 unknown，再交给既有融合真值表。全
  unknown 只会发布 `ready=false` 的 blocked unavailable snapshot，保留 audit occupancy、NaN ESDF/
  gradient、source generation、publication sequence 与 localization epoch，绝不从调试 PointCloud2
  反解析规划数据。
- **已实现未运行**：unknown runner 先要求 action 已产生非零 `/cmd_vel_mpc`，再记录 fault、首次数值
  unknown、`ready=false`、emergency stop、两级归零、source generation、adapter publication sequence、
  localization epoch 和 request identity 到独立 timeline；恢复后只允许 generation/sequence 继续递增、
  旧 `/minco/reference_path` 不复活、两级速度继续为零，并仅用新目标恢复运动。nominal 不再强制
  `/rog_map/unk` 非空；unknown case 才审计该可视化 payload 的 `odom` frame、非零 stamp、唯一
  `/ats_rog_map` publisher 和 `BEST_EFFORT` QoS。此路径尚无 runtime 证据，不能写为 P2/unknown 通过。
- **已验证（最窄）**：`ats_rog_map`、`ats_rog_map_adapter`、`ats_mujoco_sim` 聚焦 CTest 分别为
  `7/0/0`、`14/0/0`、`0/0/0`（tests/errors/failures）；ROGMap engine test 锁定 reset 后 generation
  单调与 unknown 保留，adapter snapshot test 锁定 blocked all-unknown payload/NaN，融合既有测试锁定
  occupied 不被 unknown 覆盖、任一 free 消解其他 unknown、全来源无证据才为 unknown、map 外 fail-closed。
  指定八包 `--base-paths src` 单 worker build、脚本 `bash -n`、Python `py_compile`、三个 launch
  `--show-args` 和三仓 `git diff --check` 均通过。MuJoCo Python pytest 因当前 install 不暴露
  `carstatemsgs` 可 import module 而 collection 失败，不能记为通过。
- **运行失败，已观察到的预算违反（Confidence: High；唯一根因 Unconfirmed）**：独立 headless
  nominal `ROS_DOMAIN_ID=181`（`LOG_LEVEL=info`）和 `182`（`LOG_LEVEL=warn`）均使用
  `PLANNING_GRID_OWNER=rog_map`、`P2_FAULT_CASE=none`、`SOLVER_MODE=ilqr`。两轮都在 action
  accepted/tracking 前后验证了 LiDAR、ROGMap numeric projection、adapter heartbeat/planning-grid
  owner 及两级 topic ownership。
  - 时间上**最早**观察到的预算违反是 ROGMap ground projection：domain `192` 的 72 个 projection
    样本为 `p50=2386.1 ms`、`p95=3304.5 ms`、`p99=3595.8 ms`，对应
    `cloud_timeout_sec=2.0 s`，且早于 tracking 建立。
  - tracking 建立**之后**，iLQR solve/callback 同样严重超过 `20 Hz / 50 ms`：domain `181` 最后
    样本为 `155.387/281.713/326.962 ms`，domain `182` 为 `94.671/268.061/392.287 ms`
    （p50/p95/p99），中途出现 `MPC solve 482.97 ms`。
  - 随后出现 odometry stale、ROGMap stale、projection 延迟和 planning-map heartbeat lease
    timeout，action 最终 `ABORTED/result_code=4`。日志为
    `/tmp/ats_minco_mpc_test_launch_181.log` 与 `/tmp/ats_minco_mpc_test_launch_182.log`。
  - **缺少 CPU/scheduler trace，不能确认唯一根因**：现有证据无法区分「iLQR 超时是 map 链阻塞的
    下游后果」与「iLQR 求解本身是独立的开销来源」。因此不写「iLQR 是唯一 first violation」。
    这证明 fail-closed 在健康链失效后生效，不证明 nominal action/schema-3 固定窗口通过。
  - **本轮新增：整仓以 `-O0` 编译（Confidence: High）**。`build/*/CMakeCache.txt` 全部 36 个包
    `CMAKE_BUILD_TYPE` 为空，`flags.make` 里没有任何 `-O` 选项；`ats_rog_map` 与 `ats_swerve_mpc`
    的 `CMakeLists.txt` 只设了 `-Wall -Wextra -Wpedantic`，从未设过优化等级。已在两个包内补
    `if(NOT CMAKE_BUILD_TYPE ...) set(CMAKE_BUILD_TYPE Release)`，重建后 `flags.make` 出现 `-O3`。
  - **分阶段证据把投影成本定位到 `getGridType`（Confidence: High）**。实验 A（domain 202，`-O0`，
    不做动作跟踪）：`sample_ms` p50=520.2 ms 中 `grid_type_query_ms` p50=420.3 ms / 60000 次调用，
    而 `esdf_query_ms` p50=8.9 ms / 2293 次；即成本在逐格占用类型查询，不在 ESDF 距离查询。
    `grid_type_queries` 在 A/B/C 三个实验里恒为 60000，与 LiDAR 采样密度（downsample 2→8）无关，
    说明投影成本由投影栅格几何决定，不由点云密度决定。
  - **`-O3` 后投影耗时下降 71×（Confidence: Medium，受环境污染影响）**。domain 205 实验 A：
    projection `total_ms` p50 由 1515.9 ms 降到 21.2 ms，`grid_type_query_ms` p50 420.3→10.9 ms，
    `map_lock_wait_ms` p50 853.0→0.0 ms。倍数本身可信（同一固定条件、同一脚本、同一起点），
    但该轮运行因下述环境污染未通过 admission，因此不作为 nominal 通过证据。
  - **测得的 nominal 运行全部被残留进程污染（Confidence: High），故本轮不宣称 first violation 已解决**：
    发现一个 20.7 小时前遗留的整套节点进程组（PGID 42520，`ROS_DOMAIN_ID=179`），其中
    `ats_rog_map_node` 常驻 100% CPU、一个 `python3` 82%，8 核机器 loadavg 达到 12。该进程组
    与本轮使用的 domain 201–207 无 DDS 交叉，但持续占用约 2 个核，且在本会话每一次采集期间都在运行。
    `-O3` 运行中出现 761–982 次 `cur_pose out of map range, reset the map`、机体 z 发散到 −22 km，
    而 `-O0` 基线只有 1 次 reset；把 `ats_rog_map` 单独退回 `-O0`（MPC 保持 `-O3`）后仍发散 133 次，
    因此**发散与优化等级无因果关系**，指向 CPU 争用下的仿真步进失稳。清理该进程组需要属主确认，
    本轮未执行，故 nominal 两次独立通过、unknown 故障注入与 paired Shadow A/B/C 均未运行。
- **停止条件已执行**：nominal/action/ownership/schema-3 固定窗口未完整通过，故本轮未运行真实
  unknown source fault、two-stage zero/recovery/old-reference runtime 验收，也未运行 paired Shadow
  A/B/C；未启动 `solver_mode=qp`，未提高 OSQP iteration、未接受 `solved_inaccurate`/`max_iterations`，
  未放宽 deadline、stale、lease、unknown、collision、footprint、localization 或 gimbal gate。P2、P3、
  QP-2.7、QP-3、P4/HIL/实车均未通过；MuJoCo physical contact 亦未取得本轮可用证据。

#### QP-2.7 review 修正与下一轮环境门禁（2026-08-10）

- **已实现且已验证（组件层）**：review 后，实时采样器不再按全系统进程名聚合，而是从本轮
  `setsid` launch 取得 PGID 后仅采集该进程组；报告保留 PGID 列，旧 PGID `42520` 不会再被写入本轮
  CPU/RSS/线程/上下文切换样本。实验 B 仅改变 LiDAR downsample，用于验证投影成本是否随点云密度变化；
  它不是固定 reference 或 MPC-only 实验，禁止据此单独归因 iLQR/MPC。
- **已实现且已验证（故障判据）**：unknown observer 的 all-unknown snapshot 现在必须同时满足正
  width/height、`occupancy_len == width*height`、全部 occupancy 为 `-1`、三组数值数组同长且全 NaN；
  它还必须和同一 `publication_sequence` 的 `ready=false` status 配对，并复核 localization epoch 与
  source generation。恢复 status/snapshot 也必须配对、ready/epoch/generation 一致，避免两个独立消息
  分别满足条件时产生假阳性。
- **已实现且已验证（仿真故障原子性）**：MuJoCo `freeze_motion` 与
  `lidar_occlusion_enabled` 的单个 `SetParameters` 请求先完整校验再应用；合法参数后跟非法参数时不得
  留下半生效的故障状态。遮挡仍发布有 header/stamp 的空 `PointCloud2`，并保持输入 fresh 与 input-stale
  故障语义不同。
- **本轮验证**：`ats_rog_map`/adapter/MPC 的 `colcon test-result` 分别为
  `13/0/0`、`29/0/0`、`73/0/0`（tests/errors/failures）；MuJoCo ROS-free fault pytest 为
  `16 passed`。四包单 worker 构建、Bash/Python 语法、两个受影响 launch `--show-args` 和三仓
  `git diff --check` 通过。`ats_rog_map` 与 `ats_swerve_mpc` 目标 `flags.make` 均实际含 `-O3`。
  全仓 `test-result --all` 的 710 failures 仍是未修改 `minco_planner` 的历史 lint 产物，不能计入本轮。
- **提交证据**：根仓 `469ae5d`、`e60af36`，导航仓 `8fb2b74`、`1c7bfbd`，MuJoCo 仓 `9e91304` 已
  SSH push 到各自 `origin/develop`。新提交作者均为 `liukong1220 <1625038134@qq.com>`，没有新增
  Claude trailer。历史 commit 正文中仍存在旧 Claude co-author trailer；删除它需要改写已推送历史和
  force-push，未获单独授权前不得执行。
- **仍未通过/未运行**：旧 PGID `42520` 的归属没有得到确认，故未终止。没有干净环境下的两次 nominal、
  unknown runtime、paired Shadow A/B/C、P2 red-box、HIL、实车或物理 contact 证据；P2/P3/QP-2.7/QP-3/
  P4 均不得标记通过。

#### QP-2.7 干净环境执行结果与当前停止点（2026-08-10）

- **A profile 已通过**：在用户授权精确终止遗留 `PGID=42520` 后，无 ATS 导航进程残留。实时采样器将
  `START_Z` 默认值从错误的 `0.12 m` 与主回归/MuJoCo launch 对齐为 `0.42 m`。干净的 domain `226`/`227`
  各完成一次 map-only A，均 `run_status=0`，且 PGID TSV 仅含本轮 launch（`372618`/`376017`）；adapter
  generation 分别 `74 -> 164`、`82 -> 178`，尾端 fresh/ready 均为真，`/cmd_vel_mpc` 与
  `/motion_control` 全程为零。投影 total A1 `p50/p95/p99=13.0/23.5/35.8 ms`，A2
  `14.1/24.8/43.1 ms`；map-only 无 tracking/MPC cycle sample，不构成 MPC 或 QP 性能证据。
- **iLQR nominal 已通过**：domain `228`、headless、`solver_mode=ilqr`、`planning_grid_owner=rog_map`
  的 single action `SUCCEEDED`。终点 `(-9.058255,1.470335)` 相对目标误差 `0.058256 m`；adapter generation
  `908 -> 1697`；MINCO raw/reference `20/357`、离散 `footprint_collisions=0`；MPC
  reference/predicted 各 `4526` poses；两级速度有非零跟踪并在收尾为零。MuJoCo
  `contact_violation_count=0` 只是仿真字段，不能推导物理或实车无碰撞。
- **真实 unknown 的部分核心证据已取得一次**：domain `225` 使用持续 LiDAR 空回波、ROGMap 数值 reset 和
  adapter 融合前 secondary-evidence mask；没有伪造 planning grid。数值 projection 严格 all-unknown，新的
  fault-only `/rog_map/unk` audit cloud 只由同一权威 numerical grid 的 all-`-1` payload 生成。observer
  已记录 blocked snapshot/status 同 publication sequence、epoch/source generation、
  `ready=false -> emergency_stop=true -> /cmd_vel_mpc=0 + /motion_control=0` 与恢复 sequence 推进。实测
  fault-to-all-unknown=`10.675 s`、ready-false=`10.635 s`、emergency-stop=`0.294 s`、两级首个零命令
  `0.429/0.385 s`。但当时 observer 对 `/minco/reference_path` 请求 `TRANSIENT_LOCAL`，而 Goal Manager
  publisher 为 `RELIABLE + VOLATILE`，DDS 已报告 durability 不兼容；故“旧 reference 不复活”是空观察，
  必须降级为未验证。脚本现以兼容 QoS 在故障前订阅，并强制先看到非空 baseline reference 与 recovery，
  否则该 gate 失败。
- **停止结论**：这不是 P2 全部通过。domain `225` 的固定向西 recovery goal 被 footprint gate 正确拒绝
  `trajectory footprint is unsafe`；runner 已改用同一 fresh launch 已成功的 nominal map-frame goal。之后
  domain `224` 在 fault 前置动作中反复记录 `Progress watchdog unsafe gate: map_fresh=0, tf=0,
  pose=(nan,nan)`，12 s 内未出现当前 action 的非零 MPC 命令。不得通过延长等待、放宽 map/TF/footprint/
  unknown/lease 或使用旧 reference 绕过。完整 unknown runner、P2 red-box、freeze/其他 fault、paired
  `qp_shadow` A/B/C、HIL、实车和物理 contact 仍未通过或未运行。
- **最窄验证**：ROGMap pure helper GTest 覆盖 all-unknown、mixed、malformed 与 origin yaw；
  `ats_rog_map` 单 worker build 通过，`colcon test-result=16 tests, 0 errors, 0 failures, 0 skipped`。
  新 Python audit capture 的 `py_compile`、两脚本 `bash -n` 与 diff check 通过；既有核心头文件 warning 未修改。

#### 下一阶段新对话提示词：QP-2.7 unknown 前置 unsafe-gate 根因与稳定回归

```text
继续 ATS Sentry QP-2.7，但只解决干净环境 P2 unknown 完整 runner 的 fault 前置动作偶发
Progress watchdog unsafe gate（map_fresh=0、tf=0、pose=(nan,nan)），禁止切换 solver_mode=qp。
先完整阅读 AGENTS.md、三份 QP 文档、本轮运行工件，以及 collect_realtime_profile、
capture_rog_unknown_audit、p2_fault_observer、test_mujoco_minco_mpc_chain、Goal Manager watchdog/
safety gate、ROGMap adapter 和 MuJoCo localization producer。保留导航仓未跟踪 ats_swerve_mpc/求解器.md。

起点：A1/A2（226/227）和 iLQR nominal（228）通过；unknown domain 225 的 all-unknown、blocked
status/snapshot 同 sequence、急停、共同零速度和恢复 generation 已观察一次；旧 reference 不复活因 QoS
不兼容已降级为未验证。domain 224 在 fault 前置无故障动作发生 unsafe gate，完整 runner 未通过。

先 git pull --ff-only 和只读环境审计；用源码与日志分辨 localization 是否真的 NaN、TF 的时间/frame
是否失配、watchdog 判定点的 heartbeat/snapshot 是否过期、是否存在跨 topic 非原子顺序。禁止用 sleep、
放宽 timeout、跳过 footprint、忽略 NaN 或使用旧 reference 修复。若要修改，先给 DoD、文件范围、
frame/time/generation/QoS 契约、最窄测试和停止条件；之后新 domain 依次 A/A、nominal、unknown，要求
recovery new goal 成功。仅在完整 unknown 稳定通过后才恢复 qp_shadow A/B/C；P2、P3、HIL、实车不提前通过。
最终更新三份文档，分仓中文提交和 SSH push，作者仅 liukong1220。
```

#### [历史提示词，已被 2026-08-10 执行结果取代] QP-2.7 干净环境 nominal 与 unknown 闭环

```text
继续 ATS Sentry `ats_swerve_mpc` 的 QP-2.7，但本轮目标先限于“干净环境下 P2 nominal 与真实 unknown
安全闭环”，不是直接切 QP 主控制链。完整阅读 `AGENTS.md`、ATS导航优化TODO、backend admission、导航
方向文档，及 `scripts/collect_realtime_profile.sh`、`scripts/p2_fault_observer.py`、
`scripts/test_mujoco_minco_mpc_chain.sh`、ROGMap/adapter/MuJoCo runtime fault 源码和相关测试。

先在根仓、导航仓、MuJoCo 仓执行 `git status --short --branch`、`git pull --ff-only origin develop`、
`git rev-parse HEAD origin/develop` 与 remote 核对。保留导航仓未跟踪 `ats_swerve_mpc/求解器.md`，禁止
`git add .`、`git add -A`、reset/checkout --、删除用户文件、force push。默认 `solver_mode=ilqr`；
`qp_shadow` 只诊断，`solver_mode=qp` 继续拒绝，不能改变 `/cmd_vel_mpc` 唯一发布、tracker、last_control、
急停、unknown/stale/lease/collision/footprint/localization/gimbal gate。

第一步只读环境审计：定位残留 PGID/ROS domain/父子进程、loadavg、CPU、内存和 swap。对未知归属的进程
绝不能 kill；只有用户明确确认具体 PID/PGID 可清理，才用精确 PGID 终止并记录命令。若无法取得无遗留
导航进程、无持续 CPU 饱和和足够资源的环境，停止并报告，禁止运行 nominal 或把受污染样本当准入证据。

干净环境后按严格顺序：
1. 用两个新 `PROFILE_DOMAIN` 独立运行 `scripts/collect_realtime_profile.sh A`，确认样本只来自本轮
   PGID；A 是 map-only，B 仅用于 LiDAR downsample 敏感性，不能称 MPC-only。
2. 用一个新的 `ROS_DOMAIN_ID` 运行 headless `P2_FAULT_CASE=none`、`PLANNING_GRID_OWNER=rog_map`、
   `SOLVER_MODE=ilqr` 的 single nominal；必须取得 action、终点误差、ROGMap/adapter generation 递增、
   `/cmd_vel_mpc` 与 `/motion_control` 唯一 owner、非零跟踪后收尾零速、MINCO footprint 与 MuJoCo
   contact telemetry。任何 z 发散、out-of-map reset、stale/lease、deadline 或 action 失败都停止并保留日志。
3. 只有 nominal 在干净环境完整通过，才用另一个新的 domain 运行 `P2_FAULT_CASE=unknown`。必须从真实
   LiDAR 遮挡 + ROGMap 数值 reset 获得 source projection all-unknown；observer 必须证明同一 publication
   sequence 的 blocked snapshot/status 配对、`ready=false -> emergency_stop=true -> 两级零速度`、恢复后
   generation/sequence 继续递增且无新 goal 时旧 reference 不复活。

先给 DoD、精确文件范围、QoS/frame/time/generation 契约、可执行验证命令、假设和停止条件。完成后再考虑
`qp_shadow` A/B/C；若任一窗口 identity/digest 不一致，必须输出 `not_comparable`/`withheld`，不得调高
OSQP iteration、放宽 deadline/residual 或接受 non-solved warm-start。P2/P3/HIL/实车/物理 contact 不得提前
标记通过。每次实际修改后跑最窄构建/测试/launch/diff check，显式暂存、中文提交、SSH push，并报告三个
仓库 HEAD 与 origin/develop 一致性和作者约束。
```

#### QP-3：受控主链切换与回退

- [ ] `solver_mode=qp` 只能在 QP candidate 已通过全部 hard check 后发布 `controls.front()`；任何 `timeout`、`infeasible`、`numerical_failure`、residual 不合格、slack 超限或输入不健康都必须调用现有 `publishZeroCommandForFailure()`/`engageFailStop()` 语义。
- [ ] 不得无条件沿用 `last_control_`。只有 command/localization/map/reference 全部新鲜、前一可行序列仍被真实约束复核、且处于一个明确且极短的 fallback window 时，才可执行受限减速；其余情形一律零速度。
- [ ] 第一版不要求双求解器每周期同时运行。iLQR 保留为 runtime 可选 baseline 和受限 fallback，不能因 QP 接入删除；若启用 fallback，必须记录原因、次数、持续时间和最终零速结果。
- [ ] DoD：QP 成功、QP infeasible、QP deadline、QP solved-inaccurate、QP residual reject 都有 deterministic 单测和 ROS 节点级零速度证据。

#### QP-4：性能、MuJoCo 与故障验收

- [ ] 建立**可配对**的固定硬件/编译选项/参数基线：每组都要保持同一受控窗口长度和逐周期
  `snapshot_identity_digest`，否则离线脚本必须输出 `not_comparable` 并禁止计算 Shadow 增量。记录
  iLQR 与 QP build+solve 的 p50/p95/p99、allocation/CPU、deadline root-cause、iterations、residual、
  slack、saturation 与 fallback count。有效配置是 `20 Hz/50 ms`；未在目标机测量前，不得写成 50 Hz、
  6 ms 或内存性能结论。
- [ ] 每个 case 使用新的 `ROS_DOMAIN_ID` 和新的 MuJoCo launch，禁止串行污染机器人状态：nominal、rectangle、red-box、yaw `+pi/-pi` 跳变、reference 速度阶跃、正反向切换、横纵切换、轮速过零、QP time limit、QP infeasible、localization stale、adapter lease stale、unknown、unreachable、runtime freeze。
- [ ] 每例记录 terminal pose/error、MPC reference/predicted、`/cmd_vel_mpc` 与 `/motion_control` 唯一 ownership、minimum clearance、离散 footprint collision sample、QP status/残差/solve time、replan/fallback 次数及 MuJoCo contact telemetry。`contact_violation_count=0` 不得推导实车物理无碰撞。
- [ ] 完成 MuJoCo 后才进入抬轮 HIL：先验证四模块 drive/steer 符号、零速过渡、速率/限位、物理急停和 watchdog，再受限低速实车。P2/P3 门禁与 Nav2-free 结论不因 QP 工作改变。

#### [历史提示词，已被 QP-2.7 干净环境 nominal 与 unknown 闭环取代] QP-2.6 可配对采样与数值归因

```text
继续 ATS Sentry `ats_swerve_mpc` 的 QP-2.6 可配对性能采样与数值归因。先完整阅读 `AGENTS.md`、
QP TODO、backend admission、导航方向文档，以及当前 `ControlCycleTelemetryRing`、
`LtvQpOsqpSolver`、`ats_swerve_mpc_node`、MuJoCo chain 脚本和相关 GTest。先对根仓、导航仓、
MuJoCo 仓执行 `git status --short --branch`、`git pull --ff-only origin develop`、HEAD/origin HEAD
和 remote 核对；保留导航仓未跟踪 `ats_swerve_mpc/求解器.md`，禁止 `git add .`、`git add -A`、
`reset --hard`、`checkout --`、force push 或改动无关模块。

当前事实：OSQP v1.0.0、固定 CSC、primal/dual warm-start 接口、same-snapshot shadow、C API wall-time、
十类 root cause、只读 dump service 和离线分析器已经存在；默认 `solver_mode=ilqr`，`qp` 继续显式拒绝。
最近 A/B/C raw telemetry 的有效配置为 `20 Hz/50 ms`、`qp_time_limit_ms=10`，B 是 128
`max_iterations`，C 是 126 `max_iterations` 加 2 `time_limit`，零 feasible/warm-start；map/collision
gate 仍 hard false。A/B/C 的 source/scenario/params 一致但 duration 和逐周期 digest 不同，故成本结论
已被正确 withheld，**不得**拿 iLQR nominal、非配对差值或旧 merged deadline 充当 QP 通过证据。

目标仅限于建立可配对的采样窗口和复现 max-iterations 的离线诊断，不调 OSQP 准入参数：不得增加
`qp_max_iterations`、放宽 `qp_time_limit_ms` 或 residual、保存 non-solved warm-start、启用 `qp`、
放开 collision/map/emergency/ExecutionCommand/localization/gimbal/reference gate，或改变 iLQR 的
`/cmd_vel_mpc` 唯一 owner。先给 DoD、精确文件范围、采样 identity 契约、测试命令、假设和停止条件。

优先修改 telemetry collection 的实际 owner，使每个 profile 在相同的 execute lease、reference epoch、
map generation、固定采样 cycle count/window 内导出可比 raw records；若跨独立 MuJoCo launch 无法证明
逐周期 identity，相应脚本必须稳定输出 `not_comparable`，不得计算 delta。保留 raw artifact/manifest，
记录 20 Hz/50 ms、effective params、domain、revisions、scenario、status/residual、root causes、
matrix scale、CPU/allocation 的可信或未验证来源。再用当前 QP builder fixture 检查 Hessian/row/bound/dynamic
residual 的尺度；只在可复现证据支持时提出矩阵 scaling/preconditioning 建议，不实施放宽或主链切换。

按顺序运行窄构建、相关 GTest、`colcon test-result`、受影响 launch `py_compile`、`--show-args`、
三仓 `git diff --check`；随后每组新且空的 `ROS_DOMAIN_ID` headless MuJoCo，保存 raw telemetry，确认
`/cmd_vel_mpc` 仍仅 `ats_swerve_mpc -> twist_to_motion_ctrl`、`/motion_control` 仍仅
`twist_to_motion_ctrl -> ats_mujoco_sim`。P2 不得标记通过，P3 不得标记 Nav2-free；HIL、实车和物理接触
必须保留为未验证。完成后更新三份 QP 文档，只显式 stage 本轮文件，分仓中文详细提交，SSH push，并报告
HEAD/origin 一致性、shortlog 作者约束与未验证边界。
```

#### 历史提示词：QP-0/QP-2 后端接入与 Shadow 验证

```text
继续 ATS Sentry `ats_swerve_mpc` 的 LTV-QP 迁移第二阶段。先完整阅读 AGENTS.md、
docs/项目优化文档/ATS导航优化TODO.md、
docs/nav2_to_3desdf_minco_mpc_optimization_direction.md，以及现有
ats_swerve_mpc 的 Se2Model、ZeroSpeedGuard、LtvQpBuilder、Se2MpcController 和 ROS node。

目标：在不改变现有 iLQR 默认控制链的前提下，接入一个可审计、固定稀疏结构、支持 warm-start 的
C++ QP 后端，并实现 `solver_mode=qp_shadow`。Shadow 模式只能基于与 iLQR 完全相同的
current_state/reference/last_control 构造和求解 QP，发布 `/cmd_vel_mpc` 的唯一 owner 仍必须是
现有 iLQR；不得改变 emergency stop、ExecutionCommand、localization、gimbal、map/reference
freshness、topic、frame 或底盘所有权。

先给出 DoD、精确文件范围、后端版本/许可证/依赖来源、测试命令、假设和停止条件。先执行三个仓库
的 `git pull --ff-only origin develop` 与 status。发现用户未跟踪文件必须保留，禁止 git add .、
git add -A、reset --hard、checkout -- 和 force push。

QP 结果必须有明确 status、iteration、solve time、primal/dual residual、slack maximum、hard
constraint maximum violation。QP candidate 只有同时通过 finite、矩阵尺寸、deadline、residual、
body velocity、body acceleration、真实四轮速度、轮速增量、有效舵角速率和 slack upper-bound
复核后才能标为 feasible。低速向量方向未定义时必须使用 ZeroSpeedGuard；禁止线性化伪舵角方向。
轮速、舵角、碰撞、急停和输入健康约束保持 hard，只有 tracking/terminal 类约束可用有界 slack。

没有已批准且可复现的 QP 后端时，不得手写未经验证的生产求解器，也不得切换主链；完成共享接口、
后端准入文档和测试后停止并报告阻塞。不得无条件沿用 last_control；timeout/infeasible/residual
reject/slack 超限或输入不健康必须保持现有确定性零速度语义。

实现后按顺序运行：窄构建、相关 GTest、colcon test-result、launch Python syntax、
ros2 launch --show-args、git diff --check；随后在新的 ROS_DOMAIN_ID 运行 headless MuJoCo
shadow 观察。记录 QP/iLQR 同周期诊断、p50/p95/p99、status、residual、非零控制和唯一 ownership。
MuJoCo、HIL、实车未实际执行时必须明确列为未验证，P2 不得标记通过，P3 不得标记 Nav2-free。

只显式 stage 本轮文件；提交信息使用详细中文，导航代码提交到 ats_sentry_nav，文档/脚本提交到
根仓，MuJoCo 仅在实际修改时提交。提交前检查 cached stat/check，SSH push 各改动仓的
develop，并报告本地 HEAD 与 origin/develop 是否一致及 shortlog 作者约束。
```

## 第三步：RViz 全局/局部/MPC 路径分层

- [x] 全局控制路径：`/minco/raw_path` 已标记为 `Global Planning / JPS Search Path`，淡蓝色；只表达 JPS/A* 的任务级拓扑搜索结果。
- [x] 局部控制路径：`/minco/reference_path` 已标记为 `Local Control / MINCO Timed Reference`，绿色；不与全局路径共用 display 名称。
- [x] MPC：`/ats_swerve_mpc/reference_horizon` 已标记为 `MPC Follow / Reference Horizon`（黄），`/ats_swerve_mpc/predicted_path` 为 `MPC Follow / Predicted Rollout`（品红）；使用独立 display。
- [x] MuJoCo 与实机 RViz 配置使用同一 topic、QoS、fixed frame 语义；本轮只调整显示名称，未改变 planner/control topic ownership。
- [x] 增加 `/rog_map/viz` 的 RViz-only RGB 体素诊断层：全局 `/map`/planning grid 保持底图，局部层按 `Visualization Range` 裁剪，`/rog_map/bounds` 保持三色范围框；`/minco/raw_path` 淡蓝 JPS、`/minco/reference_path` 绿色 MINCO、MPC reference/predicted 分别为黄/品红。
- [x] 以静态配置校验和 domain `195` ROS payload/QoS 观察验证 `/rog_map/viz` 的 `frame_id=odom`、`PointCloud2.rgb` 与 RViz Best Effort subscriber；该次未保留可复查截图，不能将其写成截图回归通过。
- [x] 已完成 RViz-only 独立 profile 与最小实现：`publishDebug()` 在锁内只收集有界、不可变的
  debug snapshot，锁外进行 RGB 序列化与 DDS publish；无 `/rog_map/viz` subscriber 时不构建
  cloud。`collectVoxelDebugInBox()` 仅 reserve 当前范围的采样上界，不复制整张地图。新增
  `viz_collect_ms`、`viz_serialize_ms`、`viz_publish_ms`、`map_lock_wait_ms`、
  `map_lock_hold_ms`，保留汇总 `viz_build_ms`。
- [x] 已完成三场景基线（nearest-rank，ms）：domain `203`（RViz disabled）29 个 projection 的
  compute `P50/P95/P99=1.2/2.1/2.2`、sample `1.2/2.0/2.1`，lock wait/ESDF refresh/gradient
  均为 `0/0/0`；domain `206`（RViz 不显示 `/rog_map/viz`）32 个 projection 的 compute
  `1.5/2.8/3.0`、sample `1.5/2.7/2.9`，常态无 debug build（临时 CLI payload 核验只产生一次
  `1.0 ms`）；domain `207`（RViz 真正订阅）380 个 projection 的 compute
  `1.8/2.8/5.0`、sample `1.7/2.7/4.8`、lock wait/ESDF refresh `0/0/0`，334 个 RGB debug 的
  build `1.1/1.7/4.6`、collect `0.9/1.3/3.5`、serialize `0.1/0.1/0.2`、publish
  `0.1/0.2/0.3`、lock hold `2.0/2.7/5.7`。domain `207` 的 build 最大 `23.8` 来自锁外 publish
  最大 `23.1`；debug lock hold 最大仅 `6.4`。无 scheduler trace 时，不把该尾峰断言为唯一系统
  级根因。
- [x] 组件与接口验证：`ats_rog_map` 增量构建通过，`colcon test-result` 为 `7 tests, 0 errors,
  0 failures, 0 skipped`；debug 测试锁定 visualization/local-map 边界、非整除范围、stride、
  unknown 不导出但保留统计、RGB 分类、只读 update generation 与零 subscriber gate。配置校验
  `4/4`、受影响 launch 的 Python 语法和 `ROS_LOG_DIR=/tmp/ats_roslogs ros2 launch ... --show-args`
  通过；MuJoCo CPU LiDAR Python binding 窄回归 `1 passed`。
- [x] RViz 运行期观察：domain `207` 的 `/rog_map/viz` publisher 仅为 `/ats_rog_map`，subscriber
  为 `/mujoco_navigation_rviz2`，QoS 为 `BEST_EFFORT`；抓包 `frame_id=odom`、`width=8710`、fields
  含 `rgb`。ROGMap source generation `19 -> 7599`（380 samples），adapter numeric snapshot
  generation `19 -> 7579`（246 records），所有 projection 均 `ready=1, stale=0`。这不证明
  MINCO local snapshot 与前两者编号端到端一致，也不是 P2 闭环通过。
- [ ] 必须在不受外层会话时限影响的环境中完整重跑 headless nominal、freeze 和 red-box。domain
  `208`/`210` 在 action 前被执行会话回收，只有基础 graph/ROGMap fresh 证据；没有当前 revision
  的 RViz 截图，且本机虽具备 `ffmpeg`，本轮没有运行中的窗口可捕获。不得将它们记为通过。
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
