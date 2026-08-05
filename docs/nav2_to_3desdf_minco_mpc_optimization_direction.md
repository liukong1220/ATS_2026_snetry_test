# ATS 自研导航 V1 状态

更新时间：2026-08-05。本页只记录当前活动源码和已保存的本轮运行证据。

## P2

- `ROGMap` 可视化第一步已实现并完成构建、单测和 MuJoCo 运行期复验：活动 ROS 2
  wrapper 按官方 `robot_state +/- visualization_range / 2` 语义裁剪 debug cloud，并在
  `/rog_map/bounds` 以稳定 namespace 发布橙色 `Local Map Range`、紫色
  `Visualization Range` 和绿色 `Raycast Update Range`。绿色框来自核心量化并裁剪后的
  raycast update box；数值 projection service、occupancy、signed-distance、unknown 和
  adapter owner 未改变。运行期 domain `156` 观察到四类 ROGMap topic 非空、adapter
  `ready=true`、generation 递增，以及 `cells=10000/stale=false` 的连续 projection。
  `/rog_map/bounds` 三色框尚未保存独立截图，故只对消息实现与运行链路给出高置信结论，RViz
  视觉效果仍待截图回归。[Confidence: High，三色几何视觉为实现证据，截图为未验证项]
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
- **已验证（静态）**：`scripts/validate_navigation_config.py` 已从过时的
  `ROGMap Local Bounds` 显示名迁移到三色语义名
  `ROGMap Bounds: Orange Local / Purple Visualization / Green Update`。它对两份 RViz
  配置验证 `/rog_map/bounds`、`/minco/raw_path`、`/minco/reference_path`、MPC
  reference/predicted topic 的唯一 display、class、QoS 和 `odom` fixed frame，并以正式参数及
  producer 源码锚点核对 ROGMap、MINCO、Goal Manager、MPC 的发布/订阅归属。
  `python3 scripts/test_validate_navigation_config.py`（3/3）和
  `python3 scripts/validate_navigation_config.py` 已通过。该检查不替代运行期 ROS graph
  ownership 或任何 P2/P3 闭环验收；本轮因用户 `rviz2` 占用而未重跑 freeze、red-box 或故障矩阵。
  [Confidence: High，受版本控制的配置、源码锚点与确定性测试交叉证据；运行期证据未新增]

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
