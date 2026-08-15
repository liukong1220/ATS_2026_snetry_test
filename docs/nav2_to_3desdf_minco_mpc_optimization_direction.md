# ATS 自研导航 V1 状态

更新时间：2026-08-09。本页只记录当前活动源码和已保存的本轮运行证据。

## P2

- `ROGMap` 可视化层现已拆成两个互不替代的产品：全局 `/map`/planning grid 继续表达
  静态导航底图；`/rog_map/viz` 只在有 RViz 订阅者时生成机器人中心
  `Visualization Range` 内的 RGB 体素诊断。`/rog_map/bounds` 继续发布橙色
  `Local Map Range`、紫色 `Visualization Range` 和绿色 `Raycast Update Range`，数值
  projection service、occupancy、signed-distance、unknown 和 adapter owner 未改变。
  诊断颜色为 raw occupied=`(245,70,70)`、known free=`(92,220,235)`，unknown 只有
  `debug_viz_include_unknown=true` 才导出；淡蓝色 JPS 仍由独立的 `/minco/raw_path`
  display 表达，不能把局部 free 体素误认成 JPS 路径。
- **已验证（静态 + 组件）**：`ats_rog_map` 最终源码后的 CTest 为 `4/4` CTest、`7/7`
  测试通过；根 RViz validator 为 `4/4`，`ats_sentry_bringup` 与 `ats_mujoco_sim`
  构建、YAML、launch Python、Bash 语法和三仓 `git diff --check` 通过。
- **已验证（MuJoCo/RViz 观察链，非 P2 闭环）**：历史 domain `195` 首次确认了
  `/rog_map/viz` 的 `PointCloud2`/`odom`/`rgb` 契约，但该 revision 的
  `viz_build_ms=416.7--554.0 ms`、projection `compute_ms=1663.9--2252.9 ms` 只能作为
  优化前基线，不能与下述实现后的分项测量混用。
- **已验证（RViz-only 性能优化，2026-08-06）**：`publishDebug()` 现在只在
  `map_mutex_` 内收集有界的不可变 `DebugSnapshot`，锁外完成 RGB `PointCloud2` 序列化和
  DDS publish。`collectVoxelDebugInBox()` 只扫描 `Visualization Range` 与 local-map 的交集，
  按采样上界 reserve，绝不复制整张地图；无 `/rog_map/viz` 订阅者时不采集或构建 RGB cloud。
  snapshot 捕获 source generation、stamp、bounds 和已分类 voxel，不会再在锁外访问 `map_`。
  projection service、`PlanningMapSnapshot`、JPS、MINCO、MPC 与 emergency-stop 的输入/输出
  均未改动。[Confidence: High，源码与 7 项 CTest]
- **三场景性能基线**：以下分位数使用 nearest-rank，单位均为 ms。`map_lock_wait_ms` 指
  projection 的锁等待；`debug hold` 是 `publishDebug()` 在锁内的采集时间。`esdf_refresh_ms`
  三个场景均为 `0`，表示当前 immutable snapshot 未重复刷新 ESDF，而不代表 ESDF 链路被移除。

  | domain / 场景 | projection（n）compute P50/P95/P99 | sample P50/P95/P99 | map lock wait / ESDF / gradient P50/P95/P99 | RGB debug |
  | --- | --- | --- | --- | --- |
  | `203`，RViz disabled | `29`: `1.2/2.1/2.2` | `1.2/2.0/2.1` | `0/0/0`, `0/0/0`, `0/0/0` | 无 subscriber，无 debug build record |
  | `206`，RViz 启动但不显示 `/rog_map/viz` | `32`: `1.5/2.8/3.0` | `1.5/2.7/2.9` | `0/0/0`, `0/0/0`, `0/0/0` | 常态无构建；仅为 payload 核验临时 `echo` 产生 1 次 `build=1.0` |
  | `207`，RViz 订阅 `/rog_map/viz` | `380`: `1.8/2.8/5.0` | `1.7/2.7/4.8` | `0/0/0`, `0/0/0`, `0/0/0`（gradient max=`0.1`） | `334`: build `1.1/1.7/4.6`，collect `0.9/1.3/3.5`，serialize `0.1/0.1/0.2`，publish `0.1/0.2/0.3`，debug hold `2.0/2.7/5.7` |

  domain `207` 的 debug `build` 最大值为 `23.8`，其中 publish 最大值为 `23.1`，但
  `debug hold` 最大值仅为 `6.4`、projection lock wait 全部为 `0.0`。因此当前尾峰位于锁外
  publish，而不是 `map_mutex_` 内；没有 scheduler trace，不能把它断言为唯一系统级延迟根因。
  优化前没有这五段计时，不能反推旧 `416.7--554.0 ms` 的精确比例。[Confidence: High，源码
  临界区与固定运行日志；唯一根因结论为 Medium]
- **generation 与 RViz 契约**：domain `207` 中 380 个 projection 均为
  `ready=1, stale=0`，ROGMap source generation `19 -> 7599`；adapter 记录 246 个数值
  snapshot，generation `19 -> 7579`。这是 source/adapter 新鲜度的独立证据，不意味着其编号与
  MINCO local snapshot generation 端到端相等。运行期 `/rog_map/viz` 只有
  `/ats_rog_map` publisher 和 `/mujoco_navigation_rviz2` subscriber，双方为 `BEST_EFFORT`；
  抓取包为 `frame_id=odom`、`width=8710`，fields 含 `x/y/z/rgb`。adapter 不订阅
  `/rog_map/viz` 或 `/rog_map/esdf`，数值输入仍是 `/rog_map/get_ground_projection`。
- **本轮未完成**：domain `208` 与 `210` 的 headless runner 分别在完整 action 前被外层执行
  会话回收；其日志只证明启动后 `/localization`、planning-grid owner、`/rog_map/occ`（以及
  domain `210` 的 `/rog_map/inf_occ`）和连续 `ready=1/stale=0` 的 source/adapter 已建立，
  不构成 nominal 通过。freeze 与 red-box 本轮未能完整运行，沿用此前 red-box 未通过状态。没有
  生成当前 revision 的 RViz 截图；本机有 `ffmpeg`，但没有运行中的 RViz 窗口可捕获，故不能写成
  截图验收通过。P2/P3 状态均不变。
- 淡蓝色 JPS 搜索框不属于 ROGMap owner；官方 A* 每次 start/goal 生成临时搜索框，ATS
  后续应由 `minco_planner` 发布同一 frame 的 JPS debug marker，不能混入 ROGMap 数值服务。
- [已实现未运行] 仿真 `mujoco_navigation.rviz` 与实车默认 `sentry_default_view.rviz` 共享四层路径
  可视化契约：`/minco/raw_path` 是淡蓝色、`Z=0.02 m` 的 JPS 离散搜索路径；
  `/minco/reference_path` 是绿色、`Z=0.04 m` 的 MINCO 时间化局部参考；
  `/ats_swerve_mpc/reference_horizon` 是琥珀色细线、`Z=0.08 m` 的当前 MPC 跟随 horizon；
  `/ats_swerve_mpc/predicted_path` 是品红色、`Z=0.12 m`、`Line Style: Billboards` 的 iLQR 预测跟随 rollout。
  通过不同颜色、线宽、样式和高度避免局部重合时被同一条线覆盖。`minco_planner_node.cpp` 直接发布
  `GridJps::plan()` 的 `search_result.path` 到 `/minco/raw_path`；MPC node 对同周期 reference 与
  iLQR states 分别发布 horizon/predicted，因此该命名反映实际 producer，而非只按 topic 名称推断。
  这四个 `Path` 均为诊断输出，不改变规划、安全或 `/cmd_vel_mpc` 所有权。当前 revision 尚未保存同帧截图
  或做实车 RViz 验收。[Confidence: High，源码、配置与静态回归交叉证据；运行截图未验证]

- `planning_grid_owner:=rog_map` 时，`ats_rog_map_adapter` 是
  `/rc_esdf/planning_grid` 的唯一发布者；adapter 直接调用
  `/rog_map/get_ground_projection` 数值服务，不使用 `/rog_map/esdf` 点云作为数值输入。
- MINCO 使用本地不可变 snapshot；RC-ESDF 保留 signed-distance、unknown、梯度、
  map 外、origin/yaw 和保守静态栅格融合语义。
- 已在独立 MuJoCo domain 验证 ROGMap topic、adapter lease/generation、projection 数值
  服务与部分故障停机链路；故障注入日志需按用例分别保存，不能合并成一次红框通过结论。
  最终 red-box 运行未通过：虽然生成了 `generation=50`、`raw_points=3`、
  `reference_points=88`、`collisions=0`、`minimum_clearance=0.157` 的 MINCO candidate，
  但机器人约移动 `0.8 m` 后，MPC 日志多次报告“尚未收到有效参考轨迹，保持零速度”。旧
  reference 因急停时间戳被正确拒绝（`Ignoring a trajectory older than the latest emergency stop`），
  没有新的 reference 恢复，action 在 `180 s` 超时，最终距离约 `1.52 m`。因此 P2 红框闭环
  尚未通过；MuJoCo 独立 contact evaluator 未接入，物理接触为“未验证”。

### P2.3/P2.4 当前事实（2026-08-05）

- **已实现且已做最窄验证**：规划请求使用
  `goal_id + localization_epoch + plan_request_sequence` 身份；adapter 的
  `PlanningMapSnapshot` 提供数值 occupancy/ESDF/gradient、frame/origin/yaw 与三类时序字段；
  Goal Manager 在最终提交点复核当前 snapshot、heartbeat、frame、inside-map/free/footprint，
  并在提交点重定时后先清 emergency stop、再发布新的 reference。`ROGMap source generation`、
  adapter publication sequence、MINCO local snapshot generation 不端到端等同。
- **参数证据**：`input_sync_tolerance_sec` 由 `1.8` 调整为 `2.0`，仅对齐 181 域实测
  `1.83--1.94 s` 的 projection/terrain/slope stamp delta；`input_timeout_sec`、projection
  deadline、unknown/occupied 和 MPC 旧 reference 拒绝规则均未放宽。
- **已实现且已做最窄验证**：Goal Manager 的 `PlanProgressWatchdog` 使用 steady clock；
  `0.10 m / 4.0 s / 2.0 s / 2` 分别为最小进展、stall timeout、最小重规划间隔和最多连续
  replan。它只在 map/localization/TF/free/footprint/reference 全部健康时执行 bounded replan；
  任务级超限返回 `RESULT_PLANNING_FAILED` 并持续急停，health/TF 暂态则 fail-closed 后进入
  map-wait/recovery。相关 Goal Manager 4 项、adapter 2 项、MINCO 5 项聚焦测试均通过。
- **已验证**：headless MuJoCo domain `185` nominal 达成一条完整成功闭环，终点
  `(-8.999925, 1.490465)` 到 `(-9.0, 1.47)` 的误差 `0.020465 m`，末次 MINCO 为
  `generation=52 raw_points=2 reference_points=5 minimum_clearance=0.397
  footprint_collisions=0`，MPC reference/predicted 各 3782 poses，最终
  `contact_violation_count=0` 与四轮 RPM 为零。该 contact telemetry 不是实车/HIL 物理无碰撞
  证明。[Confidence: High，脚本和日志；实车物理结论未验证]
- **未通过**：headless domain `186` red-box 第一段成功（误差 `0.016202 m`），第二段因
  ROGMap projection/input stale 与 MPC odometry timeout 进入 map-wait，最终返回
  `RESULT_MAP_UNREADY=4`，`final_distance=10.011419 m`。此 revision 的红框 P2 门禁不通过；
  不得将单段 nominal、MINCO candidate 或 topic 存在写成 red-box 通过。
- **已实现未运行**：MuJoCo `freeze_motion` 运行时故障入口保持 sensors/localization/map
  fresh 而阻断底盘执行；脚本验收 watchdog 的 1--2 次 bounded replan、耗尽后两级零速度和旧
  reference 不复活。原启动期冻结用例已修正为运行时切换，但最终闭环仍需无 viewer/低负载重跑。
- **已验证（诊断，不是 freeze 门禁）**：domain `190` 首次运行暴露动态参数回调的
  `RcutilsLogger.warn()` printf 风格调用崩溃；MuJoCo 仓已修正为单条格式化日志，并以直接执行的
  确定性回归覆盖 `freeze_motion=true/false` 两次切换。该 `ament_python` 包未把该文件注册给
  standalone pytest，`colcon test --pytest-args` 会显示 `Ran 0 tests`，故不得写成包级 pytest
  通过。domain `191` 修复后没有此崩溃，但 nominal action 先以
  `RESULT_MAP_UNREADY=4: map ready heartbeat lease expired` 终止，冻结注入未执行。
- **已验证（带 RViz 诊断，不是低负载门禁）**：domain `192` 使用
  `use_rviz:=true`、`launch_mujoco_rviz:=false`、`use_viewer:=false`，只启动
  `mujoco_navigation_rviz2`，由用户可见地观察 ROGMap、planning grid 与导航显示；这不能替代
  无 viewer/RViz 的 freeze 验收。自研 action 已接受 `goal_id=1`，但在任何
  `/cmd_vel_mpc`/`/motion_control` 输出前返回 `RESULT_MAP_UNREADY=4`，原因仍为
  `map ready heartbeat lease expired`；最终 pose `(-10.594281, 1.541168)`，距
  `(-9.0, 1.47)` 的距离 `1.595869 m`。因此没有 runtime freeze、watchdog bounded replan、
  `RESULT_PLANNING_FAILED`、两级零速度或旧 reference 拒绝的新增运行证据。
- **已验证（首次违反点观测）**：domain `192` 的 72 个 ROGMap projection 样本为
  `p50=2386.1 ms`、`p95=3304.5 ms`、`p99=3595.8 ms`、max `3595.8 ms`；当前
  `cloud_timeout_sec=2.0 s`。同一请求中 `odom_age` 约 `0.04--0.10 s`，而
  `map_age/cloud_age` 可在 projection 前后从不足 `1 s` 增至超过 `2 s`，随即出现
  `map_update=true, odom=false, raw_cloud=true`、adapter `ready=false` 与 Goal Manager
  fail-closed。新增结构化日志保存 projection 起止/计算与 round-trip、terrain/slope stamp/age/delta、
  heartbeat sequence/source generation/localization epoch、reference identity、map lease 与
  emergency-stop 边沿。当前没有 profiler 或 scheduler trace，不能断言 map mutex 是唯一根因；
  不得据此放宽 sync tolerance、projection deadline、lease、unknown/occupied 或 MPC 旧 reference
  拒绝规则。[Confidence: High，运行日志和源码观测点；唯一根因仍为 Medium]
- **已实现且已做最窄验证（待 MuJoCo 对比）**：源码确认 core 在
  `esdf_update_interval_updates=1` 的 map update 内已构建 ESDF，而 engine 原先仍令同一
  immutable snapshot 的 `ensureCurrentEsdf()` 再建一次。现将 `esdf_generation_` 仅在
  `map_update_index_` 实际递增且命中 core interval 时对齐到 snapshot generation；滑动窗口但
  未更新概率地图时继续使旧 ESDF 失效。`RogMapEngineSnapshot.ReusesEsdfRebuiltByEveryMapUpdate`
  已通过，并锁定“update 后可直接读取当前 ESDF、sliding 后必须重建”两条分支。尚未在该修正后
  重跑 MuJoCo nominal、freeze 或 red-box，不得声称 projection deadline 或 P2 门禁已恢复。
- **已验证（静态）**：`scripts/validate_navigation_config.py` 已从过时的
  `ROGMap Local Bounds` 显示名迁移到三色语义名
  `ROGMap Bounds: Orange Local / Purple Visualization / Green Update`。它对两份 RViz
  配置验证 `/rog_map/bounds`、`/minco/raw_path`、`/minco/reference_path`、MPC
  reference/predicted topic 的唯一 display、class、QoS 和 `odom` fixed frame，并以正式参数及
  producer 源码锚点核对 ROGMap、MINCO、Goal Manager、MPC 的发布/订阅归属。
  `python3 scripts/test_validate_navigation_config.py`（4/4）和
  `python3 scripts/validate_navigation_config.py` 已通过。该检查不替代运行期 ROS graph
  ownership 或任何 P2/P3 闭环验收；本轮只做显示观察，未重跑 freeze、red-box 或故障矩阵。
  [Confidence: High，受版本控制的配置、源码锚点、确定性测试与运行期 payload/QoS 交叉证据]

### P2.7 真实 unknown 安全闭环前置复核（2026-08-09）

- **已实现且最窄验证通过**：`/rog_map/unk` 继续只是 ROGMap 的可视化/审计 `PointCloud2`，实际
  planner 输入仍为 `/rog_map/get_ground_projection` 的数值 response；unknown 为
  `occupancy_grid.data=-1`，对应 signed-distance/gradient 的 `NaN`。P2 fault 不订阅或反解析
  `/rog_map/unk`、`/rog_map/esdf`。source fixture 由持续运行的 MuJoCo LiDAR worker 发布真实空回波和
  ROGMap owner 清空现有观察构成，adapter 只在融合**前**mask secondary evidence，并只接受已由数值
  ROGMap response 证明存在 unknown 的请求；既有融合语义仍保证 occupied 优先、明确 free 可消解其他
  unknown、全来源无 free/occupied 才为最终 unknown、outside-map fail-closed。
- **数值/时序契约已实现未运行**：all-unknown fusion 发布保留 occupancy audit payload 的
  `PlanningMapSnapshot(ready=false)`，其 ESDF/gradient 为 `NaN`；adapter ready heartbeat 使用收到的
  最新 source generation。脚本为 unknown 建立独立 timeline，审计 `odom` frame、非零 stamp、
  `BEST_EFFORT` QoS、source generation、publication sequence、localization epoch 与 plan request
  identity，随后要求 `ready=false -> emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0`；
  恢复后不带新目标观察旧 reference 不复活。该 runtime 链尚未执行，不能作为实际 fail-closed 证据。
- **nominal 实际失败，停止 unknown 升级（Confidence: High；唯一根因 Unconfirmed）**：在新的无
  viewer/RViz MuJoCo domain `181` 与 `182` 下，`planning_grid_owner=rog_map` 的 single nominal 已越过
  旧 `/rog_map/unk` gate，实际观察到 ROGMap numeric service、adapter fresh heartbeat、唯一
  planning-grid owner、ATS action accepted 和 tracking 前的
  `/cmd_vel_mpc`/`/motion_control` ownership。实时性归因按观测顺序分两段，两段都是已观察到的
  违反，不能压缩成单一 owner：
  - **最早观察到的预算违反是 ROGMap projection**：domain `192` 的 72 个样本为 `p50=2386.1 ms`、
    `p95=3304.5 ms`、`p99=3595.8 ms`，对应 `cloud_timeout_sec=2.0 s`，发生在 tracking 建立之前。
  - **tracking 之后 iLQR solve/callback 同样严重超过 `50 ms`**：domain `181`
    p50/p95/p99=`155.387/281.713/326.962 ms`，domain `182` 为 `94.671/268.061/392.287 ms`，并伴随
    odometry/ROGMap stale、projection/lease timeout，最终 `ABORTED/RESULT_MAP_UNREADY=4`。
  - **缺少 CPU/scheduler trace（无 perf、无 runqueue 采样、无 mutex contention profile），不能
    确认唯一根因**。iLQR 超时既可能是 map 链阻塞的下游后果，也可能是独立求解开销；在按阶段
    插桩定位 owner 之前不做单一归因，也不放宽 cloud timeout、projection deadline、map lease、
    `20 Hz` 控制周期、MPC 约束或安全门禁。
  日志路径为 `/tmp/ats_minco_mpc_test_launch_181.log` 和
  `/tmp/ats_minco_mpc_test_launch_182.log`。本轮没有冒充运行中 unknown、two-stage zero、恢复后旧轨迹
  拒绝、终点、schema-3 固定窗口或 contact 验收。
- **状态边界**：P2 nominal 未闭环，真实 unknown runtime、P2 红框与完整故障矩阵未通过；P3 仍未满足
  `launch_nav2:=false` 的 Nav2-free 验收。`solver_mode=ilqr` 保持唯一控制发布链，`qp_shadow` 未在本轮
  paired A/B/C 运行，`solver_mode=qp`、HIL 与实车自主运动均未启动。

### P2.7 review 后的证据修正（2026-08-10）

- **性能采样证据收紧**：实时采样器已改为以本轮 `setsid` launch 的 PGID 为边界，仅收集该进程组的
  target node 资源数据，并在 TSV 中保留 PGID。此前机器上的 PGID `42520` 仍因归属未知而未终止；它不再
  污染采样归属，但其资源竞争仍使所有旧 nominal 结果不具备准入资格。实验 B 只改变 LiDAR downsample，
  不能再称为固定 reference/MPC-only 实验。[Confidence: High，脚本实现与 Bash 静态检查]
- **unknown 判据收紧**：observer 的 blocked snapshot 必须具有有效维度、精确数组长度、全 `-1` occupancy
  和全 NaN 数值 payload，并与相同 publication sequence 的 `ready=false` status 配对；fault/recovery 两端
  均校验 ready、localization epoch 和 source generation。MuJoCo fault 参数批次先验证后应用，拒绝请求
  不会留下半生效 freeze/occlusion 状态。[Confidence: High，源码与 ROS-free fault 测试]
- **组件验证更新**：`ats_rog_map`、`ats_rog_map_adapter`、`ats_swerve_mpc` test-result 分别为
  `13/0/0`、`29/0/0`、`73/0/0`；MuJoCo ROS-free pytest 为 `16 passed`，四包单 worker 构建、Bash/Python
  语法、受影响 launch 参数和三仓 diff check 通过。两目标包编译 flags 已实测为 `-O3`。这不是闭环
  nominal/unknown、P2、P3、HIL 或实车通过证据。
- **下一步门禁**：先得到用户确认或自然不存在的干净环境，再用两个独立 map-only A run 验证采样 PGID
  边界；随后依次重跑 headless nominal 与独立 unknown。任何 `cur_pose out of map range`、z 发散、stale/
  lease、deadline、action failure 或环境污染都中止本阶段并保存日志。只有两者完整通过，才重新开始 paired
  `qp_shadow` A/B/C；`solver_mode=qp` 继续禁止。

## P3

- 正式入口为 `ats_sentry_bringup/launch/bringup.launch.py` 和中立命名的
  `real_robot_navigation.launch.py`；自研入口固定 ROGMap、ATS action、Goal Manager、
  MINCO、SE2 MPC 与唯一速度链。
- 运行图无 Nav2 server，MINCO 不订阅 `/plan`；行为树多航点顺序调用
  `/ats_navigate_to_pose`。
- 本轮 MuJoCo 已验证 cancel、preempt、timeout、TF failure：action 分别返回预期结果，
  且急停后的两级速度为零。
- P3 **未完成且不得标记 Nav2-free**：即使当前 MuJoCo graph 未见 Nav2 server 且 MINCO
  不订阅 `/plan`，尚未按 P3 门禁以 `launch_nav2:=false` 完整验证自研 action 的
  feedback/result/cancel/preempt/timeout 与扩大矩形、red-box；本页 P2.3/P2.4 运行证据不能升级
  为 P3 验收，也未复现或宣称实机 `50 Hz`、约 `6 ms` 等性能。

## S1 收口

- 正式 behavior server/client 现在同样只由
  `src/ats_sentry_bringup/params/node_params.yaml` 提供 `ros__parameters`。
  `bringup.launch.py` 不再声明或传递 `behavior_params_file`；behavior 子 launch 要求
  caller 显式给出 `params_file`。`sentry_behavior.example.yaml` 仅用于 standalone 调试，
  `sentry_behavior_decision_vision_test.yaml` 仅由测试 launch 显式传入。
- MuJoCo 默认 `mujoco_navigation.rviz` 与根默认视图对齐：显示 ROGMap occupied/inflated、
  bounds、planning grid、MINCO raw/reference 与 MPC predicted；unknown 和 ESDF debug
  默认关闭。SetGoal 固定发布 `/goal_pose`，不存在 Nav2 display。
- 根正式 profile 的实机外部资源启动在当前工作站被缺失 xmacro 资源阻断；因此迁移前后的
  behavior effective dump 在独立 behavior 子启动中比对。该限制不降低正式 caller 的
  单一参数源约束。[Confidence: Medium，待具备完整实机资产的根入口复验]

### S1 本轮运行证据

- 迁移前后保存了 behavior server/client 的 `ros2 param dump`：
  `/tmp/ats_s1_pre_behavior_params/` 与 `/tmp/ats_s1_post_behavior_params/`。两节点的顶层
  参数名和 YAML 值/类型一致；action graph 为一个 server、一个 client。正式根入口的完整
  实机资产启动仍未在本机复验。[Confidence: Medium]
- 历史 rectangle 基线在新 DDS domain `186`/`184` 曾观察到 generation 递增、非零 `vy`
  横移、MPC reference/predicted 非空和离散 footprint collision sample 为 0；这些结果用于
  对比，不替代本轮 red-box 最终验收。无独立 contact evaluator 时，MuJoCo 接触只能写为
  “未验证”。
- RViz `184` 已将 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/bounds` 和两个
  `/localization` display 的运行期 subscriber 与 producer 都核验为 `BEST_EFFORT`；端点工件为
  `/tmp/ats_minco_mpc_rectangle_184_rviz_qos.out`，窗口级非黑截图为
  `/tmp/ats_minco_mpc_rectangle_184_rviz.png`。启动时 Rviz 内部 QoS property 初始化仍在 source
  日志留下短暂 incompatible warning，但最终端点表和可见画面均为匹配状态；此初始化顺序未证明
  改变闭环结果，仍应在后续 Rviz 版本升级时复验。[Confidence: Medium]
- 两例 MuJoCo 遥测的 `contact_violation_count=0` 且最终 drive RPM 为零；这只证明该仿真
  evaluator 未报告接触，实车物理接触、HIL 和低速实车仍未验证。

## P4 准备

### MuJoCo 底盘复位接口（2026-08-07）

- **已实现且已验证（sim-core）**：`ats_mujoco_sim` 新增可禁用的
  `/simulation/reset_pose`（`std_srvs/srv/Trigger`）。它在 MuJoCo 锁内恢复本次启动的
  `start_x/y/z/yaw`，清空 freejoint 和四轮转向/驱动关节速度、旧底盘命令、执行 target 与
  actuator control，然后执行 `mj_forward`。仿真时间、动态障碍物、地图/定位/规划状态和累计
  `contact_violation_count`/`max_contact_force` 均保留，不会通过复位隐藏已发生的接触诊断。
- **已验证（domain `215`，headless、无 LiDAR/ToF/RViz）**：服务返回
  `success=True`；复位后 `/swerve/telemetry` 的 `command_vx/vy/wz=[0,0,0]`、四轮最大
  `drive_rpm=0.000000`，`/localization=(1.250000,-0.750000,0.181922)`。该用例配置
  `start_x/y/z/yaw=(1.25,-0.75,0.18,0.6)`；`z` 为 MuJoCo 在地面接触后的稳定高度，平面坐标在
  测量精度内恢复。`test_pose_reset.py` 同时以真实 freejoint 模型锁定位置/航向、速度/control
  清零、旧命令清除及接触历史保留，2 项通过；连同既有 focused 物理/参数测试为 9 项通过。
- **边界**：该接口用于单个 sim-core 实例的可重复初态恢复，不能在运行中的任务里替代 cancel、
  emergency-stop、新 goal、新 domain 或新的 MuJoCo launch；它不重置导航侧状态，未运行 P2
  nominal、红框、stale/unknown/unreachable、安全停机或 P3 action 验收，也不能证明连续 swept
  footprint、物理零碰撞、HIL 或实车动力学。[Confidence: High，源码、focused test、独立 domain
  runtime service/telemetry/odometry；导航闭环结论未覆盖]

## LTV-QP 迁移第一阶段（2026-08-06，历史基线）

本轮按“共享模型、低速保护、求解器后端隔离”的顺序开始 MPC QP 化，源码范围限定在
`src/ats_sentry_nav/ats_swerve_mpc`。已完成：

- `Se2Model` 提取当前 SE(2) dynamics、Jacobian 和 rollout，现有 iLQR 通过同一模型调用；
  状态仍为世界系 `[x,y,yaw]`，控制仍为车体系 `[vx,vy,wz]`。
- `ZeroSpeedGuard` 增加 `0.01/0.02 m/s` 默认滞回阈值。轮速向量接近零时不宣称舵角方向
  可线性化，避免用未定义的角度梯度制造零速转向抖动。当前 Twist-only 链路没有独立舵角
  命令，因此该保护只提供约束契约，不实现原地舵角动作。
- `LtvQpBuilder` 生成 solver-independent 的凸 LTV-QP 描述：状态偏差/控制偏差/控制增量
  二次代价、线性化 SE(2) 等式动力学、车体速度边界和车体增量不等式；问题包含有限性、
  horizon 和输入尺寸校验。
- QP 矩阵暴露每个模块的低速角度约束有效性，尚未把轮速圆、轮速增量和舵角速率近似偷偷
  写成未经验证的硬约束；此处“尚未接入 ROS 控制计时器”是第一阶段历史状态，已由下文
  OSQP `qp_shadow` 接线替代。

验证结果：该历史基线中 `ats_swerve_mpc` 窄构建通过；当时没有 OSQP、HPIPM、qpOASES
或其他 QP 后端依赖。该限制已被下文锁定版本的 OSQP v1.0.0 vendor 接入替代，但不构成
QP 闭环、实时性或实车通过证据。

下一阶段门禁：先接入一个固定稀疏结构、warm-start、最大迭代/时间预算和求解后硬约束复核的
QP 后端；只对跟踪类约束使用有界 slack，轮速/舵角/碰撞/急停保持硬约束。QP timeout、
infeasible、slack 超限或残差不合格必须沿现有 fail-stop 链输出零速度，不能盲目保持上一拍速度。

### LTV-QP 后端准入与第二阶段边界（2026-08-06）

- **已验证（OSQP 组件）**：OSQP `v1.0.0` 官方 tag `236713ce9a56c182ac3230d52108f952afce1523`、
  archive SHA-256 `dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48`、Apache-2.0
  `LICENSE`/`NOTICE` 及 QDLDL/AMD 等第三方声明已核验；源码快照位于导航仓 `third_party/osqp`，
  CMake 静态目标为 `osqp::osqpstatic`，不使用 apt/未知系统库/运行时下载。
- **已验证（固定结构/结果）**：`LtvQpOsqpSolver` 构造期 setup 一次，timer 路径只更新固定 LTV
  CSC 数值、`q/l/u` 和 primal/dual warm-start；映射 `solved`、`solved_inaccurate`、max-iteration/
  time-limit、primal/dual infeasible 和 numerical 状态，并记录 iteration、solve time、residual、
  slack、hard violation 与 update time。QP primal 先从 `delta_u` 重建完整控制，再用共享
  `Se2Model` 做非线性 rollout 并由真实四轮 hard-check 复核。GTest 覆盖真解、primal/dual
  warm-start、pattern 漂移拒绝、全部非 `solved` 状态、ZeroSpeedGuard 和候选 hard-check；窄构建
  和包级测试通过。
- **已验证（ROS gate）**：`solver_mode` 支持 `ilqr|qp_shadow|qp`，默认 `ilqr`；控制周期建立
  immutable `ControlCycleSnapshot`，`qp_shadow` 只使用其中同一 `current/reference/solve 前
  last_control`、ExecutionCommand identity、localization epoch 与 reference 时间。它记录固定容量
  status/residual/首控差/p50-p95-p99 telemetry，命令仍由 iLQR 唯一发布；gate 测试确认 command
  publisher 数为 1。`qp` 本轮显式拒绝启动。
- **安全边界**：当前节点没有碰撞/footprint 和 map freshness 健康 producer，两项 shadow gate 保守
  hard reject；timeout、max-iteration、infeasible、solved-inaccurate、residual/slack/hard-check
  失败均不得标记 feasible，不能切换主链或无条件复用 `last_control`。当前 QP layout 不含 slack 列。
- **已运行但未通过 QP 准入（domain 229）**：MuJoCo 3.10 CPU LiDAR 的 `mj_multiRay` 补齐
  `normal=None` 槽位后，raw cloud/`/registered_scan -> ROGMap -> /traversability_grid` 恢复；
  ROGMap `cloud_age` 有限、adapter generation `55 -> 138`、iLQR 保持两级速度链唯一 owner，
  single action 终点误差 `0.011173 m`。这只是 iLQR nominal。8 条节流后的 QP record 均
  `max_iterations/400` 和 `solver_status_not_solved`；最后 telemetry 为 OSQP solve
  `p50/p95/p99=3.829/5.424/5.874 ms`、完整 callback `57.611/131.704/160.563 ms`。实际
  `control_rate_hz=20.0`，deadline 为 `50 ms`，此前 `50 Hz/20 ms` 是错误口径；旧 `61` 次
  merged miss 不能区分 status、OSQP budget 与 callback。因此停止 QP 主链迁移，
  默认保持 `solver_mode=ilqr`，不得启用 `qp`、放宽 iteration/deadline/residual 或复用未 solved
  warm-start。
- **已验证（输入同一性审计，非可行性）**：`ControlCycleSnapshot` 现在以固定小端字节序、字符串
  长度前缀和 FNV-1a-64 digest 覆盖 current state、reference stamp/deadline/frame/state/control、
  solve 前 last control 及 ExecutionCommand identity，不使用 DDS CDR。domain `229` 的所有
  节流记录 `same_snapshot=true`；该结果不证明 OSQP 可行、candidate feasible、实时性或安全 gate。
  collision/footprint/map-health producer 仍不存在，candidate 继续 fail-closed。
- **未验证**：CPU/allocation profile、稳定 QP p99、完整故障矩阵、真实 collision/footprint/map-health
  输入、P2 red-box、HIL、实车和物理接触。P2 红框仍未通过，P3 仍不得标记 Nav2-free；本轮不能
  把组件通过或 iLQR nominal 写成 QP 实时性或实车收益。后端准入清单、复现方法和停止条件见
  `docs/ats_swerve_mpc_ltv_qp_backend_admission.md`。

- **QP-2.5 已实现并运行（domain 200/201/202，非主链准入）**：控制 timer 以固定 128 槽记录
  iLQR baseline 与 qp_shadow 的 13 个 steady-clock 阶段，OSQP reported 与 C API wall-time 分离，
  同时累计十类 deadline root cause；JSON 由 timer 外的只读 service 导出，A/B/C raw telemetry 再由
  `scripts/analyze_qp_shadow_telemetry.py` 离线汇总。B `qp_shadow,warn` 的最后 128 槽为 full callback
  `52.044/78.478/92.001 ms`、iLQR `31.350/58.473/71.815 ms`、QP build `12.084/17.378/18.144 ms`、
  OSQP wall solve `3.727/4.790/5.322 ms`、hard-check `4.100/5.620/6.445 ms`，128 条均
  `max_iterations`。C `qp_shadow,info` 为 full callback `114.200/258.097/283.488 ms`，126 条
  `max_iterations`、2 条 `time_limit`。所有 candidate 均因 non-solved 拒绝，zero feasible/warm-start；
  map/collision gate 仍 fail-closed。A/B/C source/scenario/params 相同，但实际运行时长和逐周期
  snapshot digest 不同，离线 verdict 为 `not_comparable`，不得把任何差值解释为 Shadow 或 INFO 的
  因果成本。CPU、allocation、P2 red-box、HIL、实车与物理接触仍未验证。

- **QP-2.6 已实现，仿真输入链阻塞**：采样 owner 改为 `ControlCycleTelemetryRing` 的显式固定窗口，profile 通过
  `telemetry_sampling_window_cycles` 请求固定周期数（默认 `0` 不改变滚动诊断）。首个有效周期冻结
  Execute lease/reference/map identity：`manager_incarnation`、`goal_id`、`localization_epoch`、
  `map_generation`、`map_publication_sequence`、reference stamp/deadline/frame，并要求每条记录
  `execution_lease_valid=true`、`reference_fresh=true` 与非零 localization/map generation。Goal Manager 的递增 `command_sequence` 是
  heartbeat 续租审计字段，不是恒等字段。身份变化或窗口不完整会保留 raw fragment/manifest，schema 3
  离线 analyzer 固定输出 `not_comparable`，不计算 delta；跨独立 MuJoCo launch 还必须满足逐周期
  `snapshot_identity_digest` 完全一致。
- **QP-2.6 数值范围**：raw/fixture 新增 finite bound magnitude 最小/最大值，和 Hessian diagonal、
  constraint row L2、zero-delta dynamic residual 一起导出。当前 operational fixture 锁定
  Hessian `0.66..56.0`、bound `0.1..2.15`、row L2 `1.0..1.42`、dynamic residual `0`；这只是
  归因 proxy，不能单凭尺度提出 preconditioning，更不能实施 scaling 或放宽准入。CPU/allocation
  仍为未验证。
- **QP-2.6 验证边界**：本地 QDLDL source injection 后窄构建通过；`ats_swerve_mpc` package GTest
  12/12 通过，`colcon test-result` 为 73 tests、0 failure，Python/Bash、launch `--show-args` 均通过。
  临时 schema-3 fixture 已验证逐周期 digest 不同会严格输出 `not_comparable`/withheld，但不是运行性能数据。
  新空 domain `210` 的 headless A profile 在 `/traversability_grid` 前被 `mujoco==3.4.0` CPU LiDAR
  的 `mj_multiRay()` `vec` 参数形状错误中断，`/registered_scan` 缺失导致 ROGMap stale；没有 raw artifact，
  未到 command ownership/终点/contact 检查。故 P2 仍未通过，P3 仍不得标记 Nav2-free，HIL、实车和物理
  接触继续未验证。

- **QP-2.6 后续 domain 213 输入链复核（2026-08-09）**：在当前 `mujoco==3.10.0` 运行时，headless
  MuJoCo LiDAR 子进程正常启动并发布非空 `/local_pointcloud`（`frame_id=front_mid360`、宽度 `787`）
  与 `/registered_scan`（`frame_id=odom`、宽度 `104`）。ROGMap 日志的 `cloud_age` 为有限值，source
  generation `76 -> 90`，adapter `ready=1` 且 publication sequence 持续递增；脚本确认 occ/inf_occ
  非空后在 unk gate 停止，因此 esdf、planning grid、action 和 ownership 尚未取得该 profile 的运行期证据。
- 同一运行在 action、ownership 和 telemetry dump 前停止于 `/rog_map/unk` 非空 gate。有效 profile 的
  `core.visualization.publish_unknown=false`，ROGMap/adapter unknown cell count 为 `0`。这是 nominal
  debug/场景证据缺口，不是 LiDAR stale 或 QP solver 证据；没有 schema-3 raw profile，A/B/C 增量结论
  继续 withheld。下一步只允许由 MuJoCo/ROGMap 实际 owner 提供真实、受控且独立的 unknown 场景，
  保持 unknown/occupied/stale/lease fail-closed 语义；P2 仍未通过，P3 不得标记 Nav2-free。

QP 迁移实施顺序固定为：后端准入和结果状态契约 -> 低速/硬软约束测试 -> `qp_shadow` 同输入
诊断 -> 受控 `qp` 发布和有界 fallback -> 新 DDS domain 的 MuJoCo 故障验收 -> 抬轮 HIL/实车。
不能将“构造矩阵”“QP 返回 solved”或“topic 存在”替代硬约束复核、两级零速度、红框、物理接触
或实车门禁。详细 task、DoD、停止条件和下一方提示词以
`docs/项目优化文档/ATS导航优化TODO.md` 的 `MPC LTV-QP 迁移` 为准。

- [已实现未端到端迁移] `PlanningMapSnapshot` 将 adapter 的 `ready`、source/publication
  generation、localization epoch、frame、origin/yaw、occupancy、signed distance 和梯度
  收进一个不可变消息。`ready=false` 时 payload 必须为空；`ready=true` 时所有数组长度均为
  `width * height`，unknown 使用 `-1 + NaN`，已知 free/occupied 的 ESDF 符号分别非负/非正。
- [已实现未端到端迁移] `PlannerCandidate` 将 MINCO candidate、三类 generation、目标和定位
  identity、yaw authority、footprint/dynamics 判定、reference、lease 与 SHA-256 内容摘要
  置于一个 DDS 样本。`STATE_READY` 必须有非零 planner incarnation/sequence/digest、零
  footprint collision sample、非负 clearance、有效 lease 和时间单调的同 frame path。
  现有 `ExecutionCommand` 已预留 candidate incarnation/sequence/digest 字段；P3 consumer
  仍按现行授权规则运行，只有 adapter -> MINCO -> Goal Manager -> MPC -> serial 全部迁移并
  以非零字段验证后才启用 P4 fail-closed gate。
- 内容摘要的规范输入是 candidate immutable identity、三类 generation、yaw policy、raw/reference
  path 的 frame、时间戳、pose 和 lease；不含 transport heartbeat 的 `header.stamp`，也不含摘要
  字段本身。实现时必须固定字节序、浮点编码、字符串长度前缀和 schema version，禁止把 DDS
  实现相关的 CDR 输出当成跨实现 canonical digest。
- [未完成] 当前 footprint checker 的角点位移上界采样与 property test 只能提供离散保守证据，
  不是连续 swept-volume 证明；连续 swept footprint、实车动力学/制动约束、serial digest
  enforcement、HIL 与实车标定均未验收。

### P4 实车门禁

1. Gate 0：冻结 revision、参数、地图、固件和标定；运行 atomic schema/property test，验证
   `map -> odom -> base`、世界系状态 `[x,y,yaw]` 与车体系命令 `[vx,vy,wz]`，以及每个
   stale/no-path/solver/serial failure 都到达零速度。
2. Gate 1：用记录数据和故障回放注入 source/adapter/MINCO/Goal Manager/MPC/serial 的 restart、
   延迟、乱序、摘要不匹配和时钟跳变；记录 p50/p95/p99 sensor-to-command age、候选拒绝原因、
   停车上界和全部非零命令。
3. Gate 2：抬轮 HIL，先断开或禁用驱动，再逐轴确认 `vx`、`vy`、`wz` 与四个 steer/drive 的
   符号、幅值、rate limit、watchdog、物理急停和掉线后零命令；任何 unexpected motion、热/流/压
   或 telemetry 缺失立即保持禁能。
4. Gate 3：受限低速实车，空旷隔离区、独立安全员、物理急停、限制速度/加速度/扭矩/路径长度，
   先直行和制动测量，再横移、转向、定位、规划和受控故障。根据实测 braking distance、感知到
   执行延迟、坡度与定位误差更新 exclusion zone。
5. Gate 4：只有 Gate 0--3 按固定路线重复通过且无 contact、near miss、非预期命令或人工干预，
   才能逐一扩大速度、区域、时长和障碍复杂度。MuJoCo 结果不能替代 Gate 2--4。

## 配置与限制

- 正式节点参数集中于 `src/ats_sentry_bringup/params/node_params.yaml`；launch 仅覆盖
  `use_sim_time`、资产/设备路径和受控 HIL 开关。ROGMap core 从显式 ROS 参数构造配置，
  正式 profile 不接受第二份地图或 behavior 参数源。
- `static_map_publisher.py` 保留 `/map` 的 frame、origin/yaw、resolution、占据语义和
  transient-local QoS。
- 当前 S1 记录的最新 rectangle 运行结果见上文：domain `184`（RViz）为 `0.038681 m`，
  domain `186`（headless）为 `0.041613 m`，均为五段路线的最大终点误差。此前记录的
  单路线 rectangle `0.004126 m` 和 red_box `0.003696 m` 属于较早 revision 的独立运行，
  不与当前 S1 结果混合比较。本轮 ROGMap 时间/owner 修改后的 red-box 因 reference 恢复
  缺陷未完成，详见 P2。离散 footprint 冲突为 `0` 不替代连续 swept footprint、MuJoCo
  物理接触评估和实车动力学验证。

## README 同步说明（2026-08-03）

- 根仓、导航仓和 MuJoCo 仓的 README 已按当前正式源码、P2/P3 状态与上述已保存运行证据
  更新，并补充了部署、接口所有权、回归入口、致谢和许可证指引。
- 此次是文档同步，不是新的构建、MuJoCo 或实车验收；README 中的“已验证”均指向本页先前
  记录的运行证据。P4 的原子链、连续 swept footprint、HIL 与实车门禁状态不因 README 更新
  而改变。

## 2026-08-10 干净环境复跑与 unknown 证据修正

用户授权后，精确终止遗留 ATS launch `PGID=42520` 及其子进程；后续环境审计未发现该导航树残留。本轮把
实时 profile 的默认 `START_Z` 从错误的 `-0.12 m` 对齐到主回归/MuJoCo launch 的 `0.42 m`，没有修改
timeout、lease、unknown/occupied、footprint、MPC 或 QP 门禁。

- **已验证，map-only A**：独立 domain `226`/`227` 各通过一次，采样仅归属本轮 launch PGID。adapter
  generation 分别 `74 -> 164`、`82 -> 178`，两级速度全程零；projection total 的
  p50/p95/p99 分别为 `13.0/23.5/35.8 ms` 与 `14.1/24.8/43.1 ms`。A 没有 tracking/MPC cycle，不能
  推导 iLQR、QP 或控制实时性。
- **已验证，iLQR nominal**：domain `228` 使用 headless、`planning_grid_owner=rog_map`、
  `solver_mode=ilqr`，action 成功，终点 `(-9.058255, 1.470335)` 到目标误差 `0.058256 m`。adapter
  generation `908 -> 1697`；MINCO raw/reference `20/357`、离散 `footprint_collisions=0`；MPC
  reference/predicted 各 `4526`，两级速度曾非零跟踪并在结束归零。`contact_violation_count=0` 只是一项
  MuJoCo telemetry，物理 contact/HIL/实车仍未验证。
- **部分验证，真实 unknown**：domain `225` 实际取得 strict all-unknown numeric projection、fault-only
  audit cloud、blocked status/snapshot 同 publication sequence、`ready=false -> emergency_stop=true`、共同
  两级零速度窗口和 recovery sequence 推进。此前 observer 对 `/minco/reference_path` 的 durability 与
  Goal Manager publisher 不兼容，日志明确报告 QoS warning；因此旧 reference 不复活不能被视为通过证据。
  observer 已改为 `RELIABLE + VOLATILE`，且后续 gate 要求 fault 前收到非空 reference baseline 和真实
  recovery；尚未在运行时重验。
- **停止条件仍生效**：更新 runner 的 domain `224` 在 fault 前置动作中出现
  `Progress watchdog unsafe gate: map_fresh=0, tf=0, pose=(nan,nan)`，12 s 未出现当前 action 的非零 MPC
  command。不得通过延长等待、放宽 map/TF/unknown/lease/footprint 或复用旧 reference 绕过。完整 unknown
  runner、P2 red-box、其它故障、paired `qp_shadow` A/B/C、P2/P3/Nav2-free、HIL、实车与物理接触均未通过或
  未运行；`solver_mode=qp` 继续拒绝。

## 2026-08-11 QP 数值防御与本机运行门禁

`ats_swerve_mpc` 的 LTV-QP shadow 保持在执行跟踪层，未替代 Point-LIO、ROGMap、ground projection、
PlanningMapSnapshot、Goal Manager、JPS、MINCO、footprint safety 或 Local Collision Repair。本轮把 QP
candidate 的 dense LTV layout、双边 bounds、dual payload 与 reported/wall deadline 复核收紧为
fail-closed，并在 builder 阶段拒绝非有限/非凸权重和非有限动力学限制；iLQR 仍是
`/cmd_vel_mpc` 唯一 publisher，`solver_mode=qp_shadow` 不发布 Twist，`solver_mode=qp` 继续拒绝。
`ats_swerve_mpc` 的实际组件结果为 12 个 CTest target、`77 tests, 0 errors, 0 failures, 0 skipped`；
这只证明算法接口和安全拒绝路径，不能证明完整规控系统已运行。

本机再次准备 headless MuJoCo 前的资源审计显示约 `2.5 GiB` available memory、`9.8 GiB` 已用 swap，
且用户 `rviz2` 有持续约 `18% CPU` 负载。依据 CPU/swap 争用停止条件，本轮没有启动新的 ROS domain，
故没有 nominal terminal、all-unknown 同 sequence、`ready=false -> emergency_stop=true ->` 两级零速度、
recovery old-reference 拒绝、QP p50/p95/p99、P2 red-box 或 contact runtime 证据。不得把先前 iLQR nominal
或组件测试外推为 QP shadow/P2 通过；P2 仍未通过，P3 仍不得标记 Nav2-free，P4/HIL/实车和连续 swept
footprint 继续未验证。

## 2026-08-11 QP-2.8.1 数值准入与运行停止边界

本轮仍只修改执行跟踪层的 LTV-QP adapter，不替代或改动 Point-LIO、ROGMap、ground projection、
PlanningMapSnapshot、Goal Manager、JPS、MINCO、footprint safety、Local Collision Repair、ExecutionCommand、
速度 frame 或底盘 ownership。`solver_mode=ilqr` 保持默认和唯一 `/cmd_vel_mpc` publisher；`qp_shadow` 只在
iLQR 已发布后读取同周期 snapshot，`solver_mode=qp` 仍在构造期拒绝。

为消除 QP-2.8 review 中的资源与数值访问缺口，shared `checkedLtvQpDimensions()` 在 node/controller、dense
builder 与 OSQP CSC setup 之前统一拒绝 `horizon > 64` 或超过 `3 MiB` 的 dense payload；正式 horizon 30
的 payload 是 `543144 bytes`。完整 layout 损坏不再在 timer 内重分配，NaN/Inf dense 数值在 OSQP copy、
primal reconstruction 与 candidate hard-check 的索引前拒绝。OSQP update 的墙钟已覆盖 settings update、
numeric update 和 warm-start，并新增覆盖 **backend solve phase** 的 deadline、telemetry 和 p50/p95/p99 数据入口。
这保持 all-unknown、lease、emergency stop、localization/TF、gimbal、map/reference freshness、footprint 和
四轮 hard-check 的原 fail-closed 语义，不会把 QP 结果发布给底盘。

**已验证的是组件边界**：单 worker build、12/12 CTest target、`83 tests, 0 errors, 0 failures, 0 skipped`、
`p2_fault_observer` `3/0/0`、导航配置 `4/0/0`、Bash/Python launch 静态检查。**未验证的是运行闭环**：审计到
未知用户 `ros2 topic echo /ats_swerve_mpc/reference_horizon`、约 `1.6 GiB` available memory、`9.3 GiB` swap
used、约 `3.6` load average；未杀该进程且未启动新 domain。因而没有新的 nominal terminal、all-unknown
publication-sequence 配对、两级零速/recovery、QP phase p50/p95/p99、paired shadow A/B/C、red-box、HIL、
实车或 physical contact 结论。P2 不通过，P3 不得称 Nav2-free，P4 未通过，`solver_mode=qp` 不得启用。

**后续 review 阻塞（QP-2.8.2）**：`copyLtvNumericalValues()` 在 backend phase steady-clock 起点前执行，故目前
`qp_backend_phase_ms` 不含 dense-to-CSC copy，不能写成 complete QP adapter/solver timing，也不能单独作为未来
QP 主链的 deadline 准入。携带 reconstructed candidate 的 validator overload 也仍需在 identity 重建前重复执行
layout/bounds/finite gate。下一轮先用连续 complete phase wall clock、独立 telemetry/deadline、corrupted-object
GTest 关闭这两个缺口；再按资源门禁、iLQR nominal、真实 all-unknown、paired qp_shadow A/B/C 的顺序恢复运行。

## 2026-08-12 QP-2.8.2 实现边界与证据

QP adapter 现在区分两个 steady-clock 口径：`qp_backend_phase_ms` 仅覆盖 settings/data update、warm-start 与
`osqp_solve()`；`qp_complete_phase_ms` 从 dense-to-CSC 数值拷贝开始连续覆盖到 solve 返回。两者均进入 JSON
schema `4`、ring 汇总 p50/p95/p99 和独立 deadline counter；QP build、primal reconstruction、hard-check 不混入
complete phase，但仍保留 full callback telemetry。candidate deadline 同时拒绝 backend-only 与 complete phase 超期。

validator reconstructed-candidate overload 已在 `decisionSize()`、`controlOffset()` 与 Eigen segment 之前执行完整
layout/bounds/finite、horizon、nominal 与 primal exact-size gate，并检查 offset 非负且三维块不越界。腐坏但 `valid=true`
的 state/control dimension、矩阵/向量 layout、NaN/Inf fixture 组件测试通过，保持 fail-closed。

本 revision 的已验证证据仅为 `ats_swerve_mpc` 单 worker build、12/12 CTest、`85 tests, 0 errors, 0 failures, 0 skipped`、
P2 observer/config Python tests、Bash/Python/launch 静态校验。未进行 MuJoCo nominal、真实 unknown 安全停机/recovery、
paired shadow A/B/C、red-box、P2/P3、物理接触、HIL 或实车；不得将组件 phase 样本外推为 20 Hz 周期预算或主链准入，
也不得启用 `solver_mode=qp`。

## 2026-08-14 Gazebo 导航后端接入证据

本节只记录本轮实际运行的 Gazebo 结果，不回填或覆盖前述 MuJoCo/QP 结论。新后端由用户 fork
`liukong1220/rmu_gazebo_simulator` 提供，root `dependencies.repos` 以 SSH 固定到其 `main`；
嵌套 Gazebo manifest 已退役，Gazebo-domain rmoss 依赖统一由 root manifest 导入
`src/dependencies`。`pb2025_robot_description` 的活动引用已清除，机器人资源唯一来自
`ats_robot_description`。

### 配置和闭环契约

- 默认 `world=rmuc_2025`，`map_yaml` 为 root-owned
  `src/ats_sentry_bringup/map/rmuc_2025.yaml`；PGM 始终从 YAML 的 `image` 字段解析，不复制到
  simulator。默认 `planning_grid_owner=rog_map` 与 `solver_mode=ilqr`；`qp_shadow` 只诊断，`qp`
  继续拒绝。
- 闭环为 `Gazebo Mid360/IMU -> C++ PointCloud2-to-Livox adapter -> Point-LIO ->
  /localization,/registered_scan -> ROGMap -> adapter/RC-ESDF -> JPS -> MINCO ->
  MPC -> /cmd_vel_mpc -> chassis adapter -> Gazebo 4WD4WS`。Gazebo ground truth 只用于观测，
  不是 localization 输入。
- 车体系命令是 `[vx,vy,wz]`。四个模块各用
  $[vx-wz\,y_i,vy+wz\,x_i]$ 求轮心速度；因而横移存在、非差速、无 `vy=0`/ICR 假设。短转向
  翻转和 command timeout 都在 Gazebo actuator 内 fail-closed。
- `sensor_scan_generation.base_frame=""` 只作用于 Gazebo profile，消除
  `base_footprint` 无输入时的 localization 自举死锁；实车默认入口未改。observer 对 sensor
  stream 使用 `BEST_EFFORT`，不会因 QoS 不兼容产生空 executed path。

### 已验证的运行结果

- domain `181` 的 headless nominal action 成功：终点误差 `0.0502185 m`、JPS/MINCO/
  predicted/executed path `3/67/31/19`、GT 位移 `2.1284 m`、ROG source generation
  `220->332`、adapter sequence `56->86`。运行期 evidence recorder 中
  `/cmd_vel_mpc`、`/motion_control`、Gazebo chassis command 的 publisher max 都是 `1`。
- domain `183` 的最终 RViz 配置实际显示 static map、planning grid、registered scan，以及蓝色
  JPS、橙色 MINCO、洋红 predicted 和绿色真实 executed path；图像为
  `log/gazebo_minco_mpc_chain/20260814_212305_nominal_none_domain183/rviz_navigation_active.png`。
  同次 action 被 unsafe footprint 拒绝，故它不是 nominal success 证据。
- domain `206` 的 Gazebo RViz 操作层新增 `2D Goal Pose -> /goal_pose`；现有
  `ats_goal_manager` 仍独占 frame/map/localization/急停校验和 JPS/MINCO/MPC 任务生命周期，未引入
  Nav2 action。显示分为 `Global Map and Planning`（`/map`、`/traversability_grid`、唯一
  `/rc_esdf/planning_grid`）与 `ROGMap Local Quality`（默认 `/rog_map/viz`、
  `/rog_map/bounds`，可选 occ/inf_occ/unk/esdf）。`/registered_scan` 仍可用但默认关闭，避免软件
  渲染与 Gazebo/Point-LIO 争用；四条真实 Path 继续为独立 display，predicted/executed 使用
  `Billboards`。最终界面工件为
  `log/gazebo_minco_mpc_chain/20260815_102643_rviz_ui_final_readable_none_domain206/rviz_navigation_active.png`；
  同次 metrics 为 JPS/MINCO/predicted `3/101/31` 点且三段命令非零。action 未成功，故该工件只证明
  可视化和诊断 payload，不是 Gazebo nominal 或 P2 通过。
- domain `184` 至 `191` 分别运行 all-unknown、map-unready、map-stale、input-stale、
  goal-unreachable、adapter-lease、projection-timeout 和 emergency-stop-recovery，均为独立 ROS
  domain、`failures: 0`。所有故障用例观测到 emergency stop 与两级精确零速度。recovery run 在
  cancel 后取得 3 个零命令样本、generation/sequence `203->436`/`50->103` 持续前进，只有新的
  recovery goal 才重新成功。

### 边界

这些结果说明 Gazebo 后端已能以真实导航主链、地图 owner、路径、控制和安全退化行为回归算法修改；
它们不证明 P2 完整通过，也不证明 P3 Nav2-free。最终可视化代码版本尚无成功 nominal；minimum
clearance、MINCO 离散 footprint collision、Gazebo physical contact telemetry、红框、HIL、实车和连续
swept footprint 均未验证。不得通过放宽 TF wait、input/map timeout、unknown、lease、old reference
或 emergency stop 来处理 domain `183` 的安全拒绝。

## 2026-08-15 ROGMap 显示层与 MINCO 几何/时间后端实施

`/rog_map/esdf` 继续是 `10 x 10 x 1 m` 滑动窗口的局部 debug cloud，未被累积或扩大。两份实际 RViz
配置现直接显示 adapter 的全局 `/rc_esdf/signed_distance_grid`，名称为
`Global Fused RC-ESDF (ROGMap + Static + Terrain)`，并保留局部 RGB voxel/ESDF/bounds 覆盖层。它的
`0..100` 仅为 display encoding，规划仍消费数值 `PlanningMapSnapshot`/RC-ESDF，adapter 也未订阅 debug
point cloud。配置 validator 和 9 个 Python 聚焦测试已通过；尚无本 revision 的 RViz 截图或三米滑窗跟随证据。

MINCO 现以同一 immutable snapshot 保留 `raw -> preprocessed guide -> ESDF-refined guide -> MINCO ->
predicted -> executed`。几何预处理、footprint-aware fail-closed shortcut、曲率感知 time allocation、局部
相邻段 scaling、ESDF trigger/target、法向 projection、offset smoothing/backtracking 与独立 v/a/j/footprint
quality gate 已实现，十个指定 GTest 与完整 `minco_planner` CTest `11/11` 通过。实现保持 MINCO S3、独立
yaw 和全向 `[v_x,v_y,w_z]`；没有 Ackermann、ICR 或 `vy=0` 约束。

未修改源码 Gazebo baseline 已先保存在 domain `221--223`。随后 domain `226` 证明无条件 ESDF densify 会把
二点直线变为 9 个硬控制点并导致 `peak_a=2.659>2.5`；现已修复为仅在真实 clearance trigger 下插点。修复后
直线候选实际为二点 guide、`length_ratio=1.000`、横向偏差/曲率符号变化为零，轨迹、MPC、执行和两级非零速度
均曾观测到。C++ recorder 进一步确认动作期 planning grid publisher max 为 `1`、adapter 已见且没有命名的
非 adapter publisher；匿名 DDS endpoint 只作为诊断保存。

但 P2 不得更新为通过：domain `230` 的 localization interval 为 `0.371/0.994/1.612 s` (p50/p95/p99)，adapter
反复 `ready=false`，goal 在距目标 `0.802 m` 时 fail-closed `ABORTED`。此 freshness 停止条件也使新样本不能
作为低负载性能结论；Gazebo corner/S/narrow/nominal/red-box、RViz 截图、故障矩阵、MuJoCo、HIL、实车、
physical contact 与完整 clearance/swept collision 仍未验证。P3 仍不是 Nav2-free，P4 仍未进入。
