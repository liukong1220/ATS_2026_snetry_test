# ATS 自研导航 V1 状态

更新时间：2026-08-05。本页只记录当前活动源码和已保存的本轮运行证据。

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
- [已实现未运行] 两份正式 RViz 配置已把 `/minco/raw_path` 标记为淡蓝色
  `Global Planning / JPS Search Path`，把 `/minco/reference_path` 标记为绿色
  `Local Control / MINCO Timed Reference`，并独立显示 MPC 的黄色 reference horizon 与
  品红 predicted rollout。`minco_planner_node.cpp` 直接发布 `GridJps::plan()` 的
  `search_result.path` 到 `/minco/raw_path`，因此该命名反映实际 producer，而非只按 topic
  名称推断。当前 revision 尚未保存同帧截图或做实车 RViz 验收。[Confidence: High，源码与
  RViz 配置交叉证据；运行截图未验证]

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
