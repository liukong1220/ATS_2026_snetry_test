# ATS 自研导航 V1 状态

更新时间：2026-08-02。本页只记录当前活动源码和本轮运行证据。

## P2

- `planning_grid_owner:=rog_map` 时，`ats_rog_map_adapter` 是
  `/rc_esdf/planning_grid` 的唯一发布者；adapter 直接调用
  `/rog_map/get_ground_projection` 数值服务，不使用 `/rog_map/esdf` 点云作为数值输入。
- MINCO 使用本地不可变 snapshot；RC-ESDF 保留 signed-distance、unknown、梯度、
  map 外、origin/yaw 和保守静态栅格融合语义。
- 已在独立 MuJoCo domain 运行 rectangle、red_box、adapter lease、projection service
  timeout、Point-LIO 输入 stale、all unknown 和 unreachable。每个故障都观察到
  `emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0`。

## P3

- 正式入口为 `ats_sentry_bringup/launch/bringup.launch.py` 和中立命名的
  `real_robot_navigation.launch.py`；自研入口固定 ROGMap、ATS action、Goal Manager、
  MINCO、SE2 MPC 与唯一速度链。
- 运行图无 Nav2 server，MINCO 不订阅 `/plan`；行为树多航点顺序调用
  `/ats_navigate_to_pose`。
- 本轮 MuJoCo 已验证 cancel、preempt、timeout、TF failure：action 分别返回预期结果，
  且急停后的两级速度为零。

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
- 相同 `planning_grid_owner=rog_map` 的 rectangle 在新 DDS domain 分别完成：无界面
  `186` 的 generation `309 -> 1264`，最大五段终点误差 `0.041613 m`；RViz `184` 的
  generation `313 -> 1328`，最大五段终点误差 `0.038681 m`。两例的每段 MINCO 离散
  footprint collision sample 都是 0，south/north 均观测到非零 `vy` 横移，MPC reference /
  predicted 均非空，最终四轮 RPM、`/cmd_vel_mpc` 与 `/motion_control` 均为零。
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
- 已验证 rectangle 终点误差 `0.004126 m`、red_box 终点误差 `0.003696 m`，两例离散
  footprint 冲突为 `0`、MuJoCo `contact_violation_count=0`。这不替代 P4 的连续 swept
  footprint 和实车动力学验证。
