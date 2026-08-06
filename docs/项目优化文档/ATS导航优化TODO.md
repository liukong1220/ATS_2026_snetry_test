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
- [x] 新增 8 个窄回归测试，导航包构建通过，40 个测试用例全部通过；QP 描述仍为 shadow-only。
- [x] 接入已批准的 OSQP v1.0.0 源码快照（固定 tag/commit/archive SHA-256、LICENSE/NOTICE），
  构造固定 LTV CSC pattern，首次 setup 后只更新数值并支持 primal/dual warm-start；当前仅 `ilqr`
  和 `qp_shadow`，未切换生产控制器。
- [ ] 只对跟踪误差使用有界 slack；轮速、舵角速率、碰撞、急停和输入健康约束保持 hard constraint；QP timeout/infeasible/slack 超限必须进入现有零速度 fail-stop。
- [ ] 在 MuJoCo 中加入 yaw ±π、速度阶跃、轮速过零、QP timeout/infeasible 和舵角限位 telemetry 回归，再进行 iLQR/QP A/B；OSQP 组件测试已通过，但未完成这些证据前不得宣称 QP 实时性或实车收益。

#### QP-0：后端准入与接口冻结

- [x] **接口冻结，不等于后端准入（2026-08-06）**：新增 backend-neutral `LtvQpSolver`、固定
  CSC pattern、primal/dual warm-start payload、完整 result 状态字段及
  `LtvQpCandidateValidator`。独立复核会拒绝 non-finite/维度、deadline、residual、非 `solved`
  status、输入健康/急停/collision、body 速度/加速度、真实轮速、轮速度向量增量、有效舵角速率和
  slack/hard-bound 违规；方向未定义时走 `ZeroSpeedGuard`，不生成伪舵角。该代码未接入 ROS
  node、未添加 `solver_mode`，因此 iLQR、`/cmd_vel_mpc` 和全部紧急停机所有权不变。
  窄构建、source 工作区后的 8/8 CTest（`test_ltv_qp_problem` 与 `test_ltv_qp_solver` 已单独
  CTest）、`colcon test-result` 的 `49 tests, 0 errors, 0 failures, 0 skipped`、launch Python
  syntax 和 `ros2 launch ... --show-args` 均通过；这不是 QP/MuJoCo/HIL/实车验收。
- [x] **OSQP v1.0.0 后端准入（2026-08-06）**：官方 tag `236713ce9a56c182ac3230d52108f952afce1523`、
  archive SHA-256 `dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48`、Apache-2.0
  `LICENSE`/`NOTICE`、QDLDL/AMD 等第三方声明已核验；导航仓使用版本控制源码快照和
  `osqp::osqpstatic`，不使用 apt/未知系统库/运行时下载。详见
  `docs/ats_swerve_mpc_ltv_qp_backend_admission.md`。

- [x] OSQP 版本、许可证、来源、CMake 依赖、目标环境、CSC 格式和 warm-start 能力已记录；固定
  `z=[delta_x_0...delta_x_N, delta_u_0...delta_u_N-1]`，当前不含 slack 列。
- [x] `LtvQpSolver` 结果契约包含 `solved`、`solved_inaccurate`、`max_iterations`、`time_limit`、
  infeasible/numerical 状态、primal/dual residual、iteration、solve time、slack 和 hard violation；
  deterministic GTest 覆盖固定模式/warm-start/状态拒绝。
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

- [x] `qp_shadow` 已接入真实 OSQP v1.0.0 producer：使用同一 control timer 的
  `current/reference/solve 前 last_control`，固定 buffer/CSC、warm-start 和真实 status/残差/计时；
  只记录诊断，不发布 QP command。当前碰撞健康 producer 缺失，candidate 保守 hard reject。

- [x] 新增 `solver_mode:=ilqr|qp_shadow|qp`，默认 `ilqr`；`qp_shadow` 不发布 QP command、不改变
  tracker/iLQR warm-start/急停/topic ownership；`qp` 参数显式拒绝启动。
- [x] 每次 QP 返回由 `LtvQpCandidateValidator` 复核维度、finite、状态码、deadline、residual、
  body/真实四轮速度、轮速度增量、ZeroSpeedGuard 有效舵角速率、slack/hard bounds 及输入 gate；
  非 `solved`、超时、残差、slack 或 hard gate 失败均不可行。
- [ ] 记录 iLQR 与 QP 的同周期比较：cost、first control、预测状态、控制 delta、hard-constraint margin、slack、iteration、solve time、QP status；日志不能包含完整路径数组或无限增长数据。
- [x] DoD（组件/ROS gate）：`qp_shadow` 下输出仍由 iLQR 唯一发布，测试确认命令 publisher 数为 1；
  shadow 从同一 current/reference/last_control 快照构造。长期同周期 identity/p50/p95/p99 尚未完成。
- [ ] 停止条件：shadow 产生非有限矩阵、结构尺寸变化、超过采样/内存预算、修改现有 iLQR 输出或破坏 emergency stop，立即退回仅构造问题层。

#### QP-3：受控主链切换与回退

- [ ] `solver_mode=qp` 只能在 QP candidate 已通过全部 hard check 后发布 `controls.front()`；任何 `timeout`、`infeasible`、`numerical_failure`、residual 不合格、slack 超限或输入不健康都必须调用现有 `publishZeroCommandForFailure()`/`engageFailStop()` 语义。
- [ ] 不得无条件沿用 `last_control_`。只有 command/localization/map/reference 全部新鲜、前一可行序列仍被真实约束复核、且处于一个明确且极短的 fallback window 时，才可执行受限减速；其余情形一律零速度。
- [ ] 第一版不要求双求解器每周期同时运行。iLQR 保留为 runtime 可选 baseline 和受限 fallback，不能因 QP 接入删除；若启用 fallback，必须记录原因、次数、持续时间和最终零速结果。
- [ ] DoD：QP 成功、QP infeasible、QP deadline、QP solved-inaccurate、QP residual reject 都有 deterministic 单测和 ROS 节点级零速度证据。

#### QP-4：性能、MuJoCo 与故障验收

- [ ] 建立固定硬件/编译选项/参数基线，记录 iLQR 与 QP build+solve 的 p50/p95/p99、allocation/CPU、deadline miss、iterations、residual、slack、saturation 与 fallback count。未在目标机测量前，不得写成 50 Hz、6 ms 或内存性能结论。
- [ ] 每个 case 使用新的 `ROS_DOMAIN_ID` 和新的 MuJoCo launch，禁止串行污染机器人状态：nominal、rectangle、red-box、yaw `+pi/-pi` 跳变、reference 速度阶跃、正反向切换、横纵切换、轮速过零、QP time limit、QP infeasible、localization stale、adapter lease stale、unknown、unreachable、runtime freeze。
- [ ] 每例记录 terminal pose/error、MPC reference/predicted、`/cmd_vel_mpc` 与 `/motion_control` 唯一 ownership、minimum clearance、离散 footprint collision sample、QP status/残差/solve time、replan/fallback 次数及 MuJoCo contact telemetry。`contact_violation_count=0` 不得推导实车物理无碰撞。
- [ ] 完成 MuJoCo 后才进入抬轮 HIL：先验证四模块 drive/steer 符号、零速过渡、速率/限位、物理急停和 watchdog，再受限低速实车。P2/P3 门禁与 Nav2-free 结论不因 QP 工作改变。

#### 下一阶段新对话提示词：QP-0/QP-2 后端接入与 Shadow 验证

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
