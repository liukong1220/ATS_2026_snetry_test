# ATS 自研导航 V1 当前状态与优化方向

> 更新时间：2026-09-20
> 本页只记录当前准入状态、稳定架构边界和下一执行入口。历史阶段流水账已从活动文档移除，
> 仍可由 Git 历史和专项准入记录追溯。

> 本轮新增修复、失败记录、闭环接口账本和实车/HIL 门禁见
> [2026-09-20 fail-closed 审计](navigation_fail_closed_audit_20260920.md)。
> 下文带 domain 的已有运行仅证明各自当时 revision，不自动成为后续修改后的验收证据。

**2026-09-20 planning-grid 安全内容身份（已实现，单测已验证）**：MINCO 本地
`PlanningMapSnapshot.generation` 不再因同一安全栅格的 publication timestamp 或 adapter heartbeat
而递增。其安全 digest 覆盖 `frame_id`、resolution、width/height、完整 origin pose、全部
occupied/free/unknown cell，以及 RC-ESDF 的 `obstacle_value_threshold`/`unknown_is_obstacle`
输入；任一字段变化仍创建新 immutable snapshot、撤销旧 reference 并保持 emergency stop，直到新
generation 的计划通过 swept-footprint 复核。`ROGMap source generation`、adapter
`publication_sequence` 与 MINCO local snapshot generation 仍是三个独立字段：前两者续租数据源，
后者只标识实际用于 JPS、RC-ESDF、MINCO、footprint gate 与 repair 的本地不可变安全内容。此策略不将
source generation 伪装成端到端同号，也不放宽 snapshot exact-generation 或 stale-map fail-stop；GTest
覆盖 heartbeat 不变、occupied/unknown、几何和 unknown policy 改变。隔离 MuJoCo `single` 运行中只观测
到安全内容实际改变后的 invalidation，尚未命中相同 digest 的 runtime retain 分支；该分支仍是**已实现、
未在动态仿真数据中命中**，不能写为该运行的实测收益。

**2026-09-20 MuJoCo current-revision `single`（已验证，domain `226`）**：
`PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=single GOAL_TIMEOUT=60` 的 runner
退出 `0`。ATS action accepted 且 `SUCCEEDED`，目标 `(1.0, 0.06)`，最终 `(0.979175,
0.059443)`，误差 `0.020832 m`。`/rc_esdf/planning_grid`、`/cmd_vel/autonomy_raw`、
`/cmd_vel/selected`、`/motion_control`、planner request、reference 与 `ExecutionCommand` 的
runner 所检 publisher/subscriber 所有权均唯一；MINCO 的三次提交均为 `footprint_collisions=0`，
recorder 对 3 条 reference 得到 Q1=yes、134 个有效实际跟踪 tick 得到 Q2=no、Q3=no。

同一动作期间，map generation `55 -> 56`、`57 -> 58` 的真实安全内容变化均先使旧 reference
invalid，再由 Goal Manager 使用更高 coherent publication 进入 stop/replan/commit；最终的 stop
heartbeat 在 action 结束时生效。runner 观察到非零 `/cmd_vel/selected`，结束后为零；MuJoCo
`contact_violation_delta=0`，但这只是在该仿真计数器下没有非地面违规接触，不替代实车物理接触、HIL
或独立 contact evaluator。recorder 另报告 layer payload 截断与 1 个无 `map<-odom` TF tick，故该
结果是单条 nominal 回归，不升级为 red-box、故障矩阵或长期稳定性结论。

**2026-09-20 ROGMap `PointCloud2` 输入契约（已实现，构建与单测已验证）**：Gazebo GT
`/registered_scan` producer 输出仅含 `x/y/z` 的 `PointCloud2`，而 ROGMap 的内部点类型带
intensity。消费端现先校验 `x/y/z`；缺少几何字段时不调用 PCL 并拒绝。未启用 intensity filter 时，
XYZ-only 消息以 `pcl::PointXYZ` 解码，内部 intensity 显式写为 `NaN`，不把未观测强度伪造为零；启用
filter 时，缺 intensity 的消息被拒绝，且不提交 map update，因此既有 map-stale 到 emergency-stop 的
fail-closed 路径仍然生效。`test_point_cloud_input` 覆盖 XYZ-only 接受、filter 启用时拒绝、保留实际
intensity、以及 PCL 转换前拒绝缺失几何字段四种情形；`ats_rog_map` 构建和该 GTest 均通过。此修复不改变
ROGMap source generation、adapter publication sequence、MINCO local immutable snapshot generation、
exact-generation、swept footprint 或 emergency-stop 契约。

**2026-09-20 Gazebo terminal frame 与 current-revision nominal（已验证为未通过，domain `220`）**：
action result 的 final pose 是 `map`，而 Gazebo GT `/localization` 的原始 frame 是 `odom`；runner 现复用
`sample_localization_xy()` 的 `map<-odom` 转换，并显式记录 `terminal_localization_frame=map`。此前把二者
直接相减得到的约 `1.25 m` 不能作为滑移证据。修复后的本次 action 仍 timeout：Goal Manager 记录
`final_distance_m=1.742`、`final_pose=(2.849,-3.406,-2.517)`，归一后的 terminal goal error 为
`1.7512 m`，且 `accepted=1`、`succeeded=0`、最终 `planner_emergency_stop=true` 和 selected command 为零。

同一 artifact 的 P1 admission 为 false；首个 freshness violation 是 `gazebo_lidar`，其 328 个样本的
wall interval p99/max 为 `1.402925/1.784857 s`，高于 `0.25 s` 门限，`clock_rtf_p50=0.461077`。本次
launch log 中不再出现 `Failed to find match for field 'intensity'`，与 decoder 实现及单测共同证明缺失
intensity 的确定性 PCL schema 错误已消除；这不证明 LiDAR cadence 已恢复，因 raw Gazebo LiDAR 和
livox input 都有相似长尾。Gazebo contact 文件为空，物理接触评估为**未验证**。RTF、Gazebo Transport、DDS
或调度负载目前只构成相关候选，尚无受控单因素实验可将其写为根因。[Confidence: High] action/P1 失败和
schema-error 消失由 artifact 加运行源码/测试支持；[Confidence: Medium] 长尾的行为 owner 尚未确定。

**2026-09-20 MuJoCo current-revision `single` 补充运行证据（已验证，domain `221`）**：
`PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=single GOAL_TIMEOUT=60` 中，ATS action
`SUCCEEDED`；Goal Manager 记录 `final_distance_m=0.017`、`final_pose=(1.017,0.061,0.008)`。runner 物理
pose 从 `(-0.192545,0.075999)` 到 `(1.023685,0.044804)`，相对目标 `(1.0,0.06)` 的几何误差约 `0.028 m`；
MuJoCo telemetry 为 `contact_violation_before=0`、`after=0`、`delta=0`、`max_contact_force_n=0.0`。本次
工具记录未单独保留 runner 终止退出码，故结论是 action、终点和安全 artifact 满足，而不是“runner exit 0”。
action success 后约 16 s，runner 发出 `SIGINT/SIGTERM`，核心 ROGMap、Goal Manager、MINCO、MPC、arbiter、
fusion 和 GICP cleanly finished；但 `terrainAnalysis`/`terrainAnalysisExt` 在 context 已 shutdown 后抛出
`RCLError` 并以 `-6` 退出，MuJoCo、static-map 与 twist bridge 的 Python 进程还因
`ExternalShutdownException` 或二次 `rclpy.shutdown()` 以 `1` 退出。因此 action-time evidence 仍有效，
但本次 launch 的 graceful teardown 是**未通过**，必须在以 runner 退出码作为门禁前修复并添加最窄回归。

recorder 对 3 条 reference 给出 Q1（发布时无碰撞）=`yes`，164 个 actual tracking tick 给出 Q2（离开
reference envelope）=`no`、Q3（同 pose free/occupied 翻转）=`no`；未观察到 swept colliding segment，
minimum clearance=`0.400000 m`。运行中 map/source publication 发生 `110 -> 111 -> 112` 的变化，每次旧
reference 先 invalid，再按新的 coherent snapshot 重规划/提交，完成后 emergency stop 再置真。recorder
明确报告 layer payload 截断，且有 1 个未绑定 snapshot 的 tick 被排除；MuJoCo contact counter 又是独立
证据。因此这里只能记录“仿真 telemetry 未见非地面违规接触、recorder 未见 reference/actual swept-footprint
冲突”，不能推导实车 physical contact 为零。

本组证据不证明 Gazebo P1、Gazebo red-box、P3/Nav2-free、dynamic same-digest retain 的运行收益、HIL 或
实车稳定性。Gazebo 红框的风险转入条件仍是先在新的隔离 domain 中满足 P1 freshness 与 nominal action
success；实车转入仍需要真实 bag、持续运行、accepted observation、资源与独立 physical-contact 证据。

**2026-09-18 重定位输入门（已实现、未做仿真/实车准入）**：
`small_gicp_relocalization` 对 `registered_scan` 使用 `SensorDataQoS.keep_last(1)`，在累积前
拒绝空/错误 `frame_id`、零或回退时间戳、过期/未来帧、非有限或越界点，并按有效点比例拒绝污染帧。
累积窗口受 `max_accumulated_points`/`max_accumulated_frames` 限制，丢弃、stale、invalid、accepted
和 trimmed-window 计数以节流日志输出。该修改只保护 GICP 输入，不改变 overlap/inlier/information/
confirmation/jump 门，也没有将它升级为 Gazebo、MuJoCo 或实车稳定性证据；对应包级构建与 13 项测试
（含 3 个 GTest、2 个 fusion Python 测试和 lint）功能结果为 `0`；包级 `xmllint` 曾受远端 ROS
schema 可用性影响，随后对 `package.xml` 单独校验通过。fusion 另增加观测 stamp 的接收时刻年龄/未来
门（`observation_stamp_max_age_s`/`observation_stamp_max_future_s`），避免旧观测仅因 odom history 尚未
淘汰而改写 `map->odom`。

## 0. 2026-09-09 P2 红框与代码闭环

**已验证（domain `186`，`PLANNING_GRID_OWNER=rog_map`，`P2_FAULT_CASE=none`，`TEST_PROFILE=red_box`）**：目标 1--9 全部到达，终点误差 `0.049/0.009/0.036/0.034/0.010/0.061/0.023/0.024/0.033 m`。MINCO 提交 `collisions=0`。analyzer Q1 yes / Q2 no / Q3 no。历史 domain `176` 目标 5 Q2 越界与 domain `183/184` 目标 9 规划拒绝均未复现。提交点日志带 occupancy/content digest。

**MINCO 后端优化**（对照 `参考/navi_minco_bit` 但未照搬）：内角 fillet（`path_fillet_radius=0.35`）+ guide densify（`guide_control_point_spacing=0.30`）+ 连续侧向加速度限速 + 提前窄通道 yaw 切线（预扫整条轨迹）。MPC yaw 权重 4→10、min_progress_scale 0.25→0.10。诊断 topic 门控为空。

**未通过**：目标 9 `highland_ramp` 的 sim contact gate 拒绝（`delta=3`，`max_force=234 N`）。这是 sim `contact_is_violation()` 把底盘与高地 hfield `rmuc_2025_field` 的坡面接触判为违规，不是导航链失败。目标 10 未下发。

## 0. 2026-08-31 Codex review 结论

### Gazebo P1

**已验证通过**：隔离 domain `127/129/131` 连续三次得到
`p1_admission_evidence=true`、`all_p1_gates_passed`、`failures=0`。三次 action 终态误差分别为
`0.07443/0.06013/0.06239 m`；Gazebo Transport LiDAR p99 wall interval 为
`0.103479/0.103899/0.108009 s`，ROS LiDAR 边界为 `0.104534/0.107123/0.109608 s`。
定位状态均为 `TRACKING 600/600`，`tf_chain_established=yes`，首次建立后的 TF 查询失败为 `0`。

Point-LIO Z 发散的行为所有者是仿真 IMU 单位 profile：Gazebo IMU 静止输出约
`(0,0,9.8) m/s^2`，而共享实机配置使用 `acc_norm=1.0`、`satu_acc=6.0`。Gazebo launch 现局部覆盖为
`acc_norm=9.81`、`satu_acc=30.0`，实机 profile 未改；通过运行中的 map reset 从 `677` 降为 `0`。

证据边界：Transport 与 ROS 两侧到达间隔相近只支持“这三次运行没有可测的频率退化”，不等价于
端到端传输延迟为零。TF recorder 使用缓存中的最新变换，支持“链已建立且可查询”，尚未独立证明动态
TF 持续更新或 age 始终在阈值内。[Confidence: High] P1 准入结果由三次独立运行支持；
[Confidence: Medium] bridge 延迟与动态 TF age 仍需要专用时间戳证据。

### MuJoCo P2

**已验证（review 前候选 revision）**：`adapter_lease/service_timeout/input_stale/unknown/unreachable/freeze`
六个独立故障用例均通过。freeze 的真实缺陷是 `kExhausted` 终止原因只进入 action result、没有进入日志；
unknown 的失败来自前置 `farthest-free` 无距离上限，选到约 `14 m` 的不可提交目标。两处已分别由终止日志
和 `FARTHEST_GOAL_MAX_DISTANCE=4.0` 修复，并补入聚焦回归。

**review 阻塞**：候选实现曾允许提交 `FootprintSafetyChecker` 已判碰撞的 escape prefix。长度、点数与 yaw
上界不能排除在 `0.4 m` 内穿过薄实体障碍，因此当前代码和 RMUC profile 已把 planner/goal-manager 两层
escape 默认关闭，runner 恢复为任一 `footprint_collisions>0` 即失败。上述 6/6 artifact 早于这项安全修正；
在新 revision 重跑前记为历史候选证据，不升级为当前 P2 总体通过。

**已验证（Codex 安全 revision）**：domain `132/133/134/136/137` 的
`adapter_lease/service_timeout/input_stale/unreachable/freeze` 为 `5/5` 通过，名义前段均为
`footprint_collisions=0`、`escape_prefix_end=0`，故障与恢复等待阶段的 `/cmd_vel/selected` 和
`/motion_control` 均为零。domain `135` 的 unknown 故障主体门禁通过，包括真实 all-unknown、identity 配对、
两级零速和旧 reference 不复活；但恢复新目标在起点附近被 `8–9` 个 footprint 冲突安全拒绝，整项因此失败。
当前故障矩阵结论为 `5/6`，不是 `6/6`。

**已验证（当前 red_box，domain `138`）**：目标 1–4 以零碰撞成功，终点误差为
`0.0456/0.0069/0.0277/0.0492 m`。目标 5 行驶到约 `(1.18,-7.62,yaw=0.50)` 后，实际足迹西缘进入墙侧
占据带；后续重规划在 index `0/1` 持续得到 `2–4` 个碰撞并被拒，最终
`progress watchdog exhausted the bounded suspended-replan wait`。该运行说明最先需要收敛的是跟踪/制动后
停入接触区的行为，而不是直接从目标 9 开始调参。

`red_box` 仍未通过。目标 9 的绑定约束位于终端接近路径：实测冲突中心最坏东向极值为 `9.832 m`，仅缩小
terrain 过报仍不足；现有 6 个 terminal yaw relocation 候选也全部构造后被拒。候选间距还会漏过冲突带，
`12` 个候选可覆盖 index `43/48`，`24` 个可覆盖 `41/43/45/48`。目标 10 从目标 9 停车位起步时，被同一
terrain 过报挡在起点，尚未证伪其自身可达性。

下一轮优先级：先定位目标 5 与 unknown 恢复的 reference/actual tracking error、急停制动距离和地图来源，
让车辆在需要重规划或故障停车时仍留在零碰撞可重启域；随后再让目标 9 候选显式覆盖实际冲突带，并约束
终端 suffix 的东向摆动。所有约束从 snapshot、footprint、跟踪误差与碰撞样本推导，避免写入 RMUC 坐标特例。
若后续恢复 escape 能力，优先增加结构化、snapshot-bound 授权和薄墙/unknown/地图变化负例，而不是放宽 runner。

## 1. 当前架构

```text
Point-LIO /localization + /registered_scan
-> ROGMap 概率占据/膨胀/3D ESDF
-> ats_rog_map_adapter 地面投影与 static/terrain/slope/unknown 融合
-> PlanningMapSnapshot + RC-ESDF
-> ATS Goal Manager
-> JPS/A* fallback
-> MINCO S3 + independent yaw
-> oriented footprint + sampled swept safety + optional local repair
-> holonomic SE(2) iLQR MPC
-> /cmd_vel/autonomy_raw -> fake/chassis velocity transform -> /cmd_vel/autonomy
-> cmd_vel arbiter -> /cmd_vel/selected
-> lower-controller velocity interface
```

活动实现建议避免替换 Point-LIO、ROGMap、RC-ESDF、JPS、MINCO S3、独立 yaw、footprint gate、
Local Collision Repair 或全向 SE(2) MPC。控制保持车体系 `[vx,vy,wz]`，建议避免差速、Ackermann、
ICR 或 `vy=0`。

### 导航与下位机边界

实机的自主源由 `ats_swerve_mpc` 唯一发布到 `/cmd_vel/autonomy_raw`，经既有
`fake_vel_transform` 与 `chassis_vel_transform` 到 `/cmd_vel/autonomy`；`cmd_vel_arbiter`
将其与保持 ROS 默认的手动 `/cmd_vel` 仲裁为唯一的 `/cmd_vel/selected`，串口只订阅 selected。
自动源依赖新鲜 `ExecutionCommand`，手动源可不依赖该授权；两者都受 emergency stop、串口链路
和各自超时的归零约束。下位机负责 CAN、电机、舵轮、
电流、电压、温度、轮速、硬件 watchdog、制动和底盘反馈；ATS 导航不订阅这些信号，也不以其
作为 action、仿真或导航准入条件。

`standard_robot_pp_ros2` 保持原有决策与自瞄相关内容。`serial/gimbal_joint_state` 建议保留为
云台 yaw、速度坐标变换和自瞄-导航协调的输入，`GimbalYawStatus` 与
`YawAuthorityRequest` 也继续属于导航协调契约，而非底盘硬件健康门禁。

本轮已将实机、MuJoCo、Gazebo 与 loopback 统一到 selected 边界：实机的根总 YAML、单包串口 YAML 和
串口节点参数默认值均为 `/cmd_vel/selected`，自动授权只在 arbiter，串口不订阅空 `ExecutionCommand`。
MuJoCo/Gazebo 有源码和静态 launch 契约，loopback 也启动同一 `cmd_vel_arbiter`。`ats_cmd_vel_arbiter` 的
19 条 GTest 已通过。重装当前 launch 后的隔离 `ROS_DOMAIN_ID=222` 中，
`/cmd_vel -> cmd_vel_arbiter -> /cmd_vel/selected -> loopback_simulator` 实际闭合：selected 的
publisher/subscriber 为 `1/1`，分别是 arbiter 与 loopback；持续 `vx=0.3` 后观察到非零 selected 与
`/odom.pose.pose.position.x=0.825`（artifact：`/tmp/ats_loopback_arbiter_domain222.fEjFyT`）。该试验仅验证手动源的
仲裁出口接线，不验证 MPC、主动授权、碰撞或物理动力学。键鼠到串口或 `/motion_control` 的运行闭环、当前 revision 的 MuJoCo
nominal/red_box/fault matrix 和物理接触仍为**未验证**。当前 selected 链的最新 Gazebo P1 已在新 domain `228`
完成 `60.001345 s` recorder，但首违仍是 `/lidar_odometry`：wall p99/max=`0.650163/0.743549 s`，
`/localization`=`0.650164/0.743591 s`、status `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`，故
`p1_admission_evidence=false`、action 未成功。该窗口中 JPS/MINCO/MPC 路径与非零 selected 均未出现，属于
定位/地图 fail-closed，不是速度仲裁失败；旧 `/cmd_vel_mpc` P1 artifact 仍不能证明当前链通过。

`rmu_gazebo_simulator` 的 `test_evidence_statistics` 已补齐 arrival/stamp 的独立语义：四次 arrival
建议记录三段 wall 间隔，即使其中 ROS stamp 重复或倒退；更新后的 focused CTest 已通过。该修复不放宽
freshness 判据，也不改变速度或统计实现。

四环境的目标出口账本如下。除 loopback 的最终 consumer 实测外，其余行仍是源码/静态契约，不是闭环通过声明：

| 环境 | 自主输入 | arbiter 输出 | 唯一最终 consumer |
| --- | --- | --- | --- |
| 实机 | `raw -> fake/chassis -> /cmd_vel/autonomy` | `/cmd_vel/selected` | `standard_robot_pp_ros2` 串口 |
| MuJoCo | `/cmd_vel/autonomy_raw` | `/cmd_vel/selected` | `twist_to_motion_ctrl` |
| Gazebo | `/cmd_vel/autonomy_raw` | `/cmd_vel/selected` | `gz_chassis_cmd_adapter` |
| loopback | 手动 `/cmd_vel`；不启动 MPC | `/cmd_vel/selected` | `loopback_simulator` |

Gazebo adapter 继续负责它既有的 big-yaw 车体系旋转和 `/motion_control`/Gazebo chassis 的唯一发布；
统一输入不等于删除该 frame 变换。若要求 big-yaw feedback 而样本缺失，adapter 令
`/motion_control` 与 Gazebo chassis 两个最终输出同时为零；该 pure logic focused CTest 已通过，尚未重跑
Gazebo 物理闭环。loopback 是轻量 Twist 执行端，只订阅 selected；它启动 arbiter 保证 selected 不会被
直接注入，也不能把 `/motion_control` 作为绕过仲裁的入口。

### 2026-08-21 MuJoCo 交接证据

- **已验证（组件/场景）**：`ats_mujoco_sim` 的 `test_rmuc_2025_scene.py` 为 `9 passed`；
  `ats_goal_manager` 的 `test_goal_manager_epoch` 和 `minco_planner` 的
  `test_minco_trajectory_optimizer` 均为 `1/1 passed`；三包定向构建通过。
- **已验证（历史隔离运行）**：在 `ROS_DOMAIN_ID=222`、`planning_grid_owner=rog_map` 的
  `south_corridor` 五段路线完成，末段终点 `(6.54853, -7.65519)` 对 `(6.50, -7.65)` 的误差为
  `0.04881 m`，离散 footprint 冲突数为 `0`。ROGMap adapter generation 从 `788` 增至 `3641`。
- **未验证**：本提交 revision 的默认单点、完整十段 `red_box`、adapter lease/service timeout/
  input stale/unknown/unreachable P2 故障矩阵、`/cmd_vel` 仲裁与键鼠-串口闭环、MuJoCo 物理接触。
  `footprint_collisions=0` 只表示规划器离散采样无冲突；**MuJoCo 物理接触未验证**。
- **测试基础设施限制**：`minco_planner` 整包历史 `clang_format`、`copyright` 与 `cpplint` 检查
  仍有大量既存失败；本轮未格式化或改写无关文件，不能称整包 lint 通过。

## 2. 当前准入结论

| 阶段 | 已完成 | 当前缺口 | 状态 |
| --- | --- | --- | --- |
| P2 | ROGMap/adapter 数值链、唯一 planning owner、immutable snapshot、fail-stop、直线 Gazebo 运行 | freshness、最终 nominal/red-box、当前 revision fault matrix、clearance/footprint 采样、RViz 滑窗验收 | **未通过** |
| P3 | ATS action、feedback、cancel/preempt/timeout、Gazebo 默认无 Nav2 启动路径 | 完整 action 生命周期、扩大路线、red-box、无 Nav2 server graph 运行证据 | **未通过** |
| P4 | 四舵轮导航模型、速度 owner、矩形 footprint、自适应 sampled sweep | 连续 swept 误差上界、导航故障矩阵与实机导航结果 | **未通过** |
| QP | OSQP v1.0.0、固定 CSC、warm-start ABI、same-snapshot shadow、数值防御 | 真实 map/collision gate、稳定 solved、paired runtime、主链 fallback 和切换准入 | **仅 Shadow** |

## 3. 最新有效证据

- P0 已通过：`dependencies.repos` 将不存在的 `rmoss_gz_resources@main` 修正为可验证的
  `humble`，并将曾经传输不稳定的 `ats_mujoco_sim` 和 `teleop_gimbal_keyboard` 切换到可达的
  用户 SSH URL；`dependencies.lock.repos` 由最终干净目录的 `vcs export --exact -n` 生成，锁定
  22 个实际 checkout SHA。
- 最终干净目录 `/tmp/ats_p0_repro_final.6xNq8i` 的 `vcs import` 于 `152.2 s`、`rc=0` 完成；
  Gazebo fork `a28ccd20428ffc4bdd7fbbc22fee884fa1db72eb` 还修复了 CMake 引用 ignored 测试源的
  clean-build 缺陷。该目录的最窄 Gazebo 依赖闭包构建 17 包通过，Gazebo CTest `4/4` 和
  `ats_gazebo_nav.launch.py --show-args` 通过。该证据只覆盖复建和资源解析，不覆盖运行期导航。
- 独立目录 `/tmp/ats_p0_remote_final.Lgyalq` 又以 SSH 对 root `origin/develop` 做 depth-1 clone，
  得到 `f2d049cbf245b6fdfbad0d4870e53dc3ab09cbeb` 后直接导入 exact lock；全流程
  `250.2 s`、`rc=0`，22 个依赖均从远端 checkout。至此 root 传输、manifest、锁定与最窄构建具有
  相互独立的复建证据。
- 两份 RViz 已配置全局 `/rc_esdf/signed_distance_grid` 和局部 ROGMap debug，但当前 revision 尚无
  全局 ESDF/三米滑窗运行截图；
- MINCO 已有 geometry preprocessor、curvature-aware time allocation、ESDF refinement 和 quality
  telemetry，聚焦 CTest 已通过；
- Gazebo domain `228/229` 的短直线 action 成功，终点误差约 `0.060/0.045 m`；
- domain `230` 的直线 candidate 长度比 `1.000`、曲率为零，但 `/localization` wall interval
  `p50/p95/p99=0.371/0.994/1.612 s`，adapter 反复 `ready=false`，action fail-closed；
- P1 的单 recorder 已补齐 `/clock`、三段 odometry、`/localization/status` 与 adapter 的 wall/stamp/age、
  duplicate/backward、RTF、TRACKING、TF lookup、本 session 进程 telemetry 与 recorder callback duration
  字段；callback 仅量化观测器自身开销，不代表上游 executor 或 DDS queue。新统计的 focused CTest、
  Release build、runner/launch 静态检查通过；完整包 CTest 的 `ament_black` 仍因 sandbox 无本地 socket 权限
  所限，尚未取得主机复跑结果。DDS queue/drop 计数明确为 `unverified_no_portable_rmw_counter`，不能解释为
  零丢包；
- 根仓 runner 新增纯函数 `scripts/gazebo_freshness_classifier.sh` 和确定性回归，按
  `/clock -> /lidar_odometry -> /odometry -> /localization -> status` 顺序输出
  `p1_first_freshness_violation`；只在完整 P1 条件满足时置 `p1_admission_evidence=true`，不改变
  timeout、lease、QoS 或控制行为。
- **已验证（历史诊断运行，2026-08-19）**：合法 domain `220` headless 30 s 已运行完整 Gazebo
  链。`/clock` p99/max=`0.122538/0.135193 s`，`/lidar_odometry`=`1.671385/1.671385 s`，
  `/odometry`=`1.659046/1.659046 s`，`/localization`=`1.663433/1.663433 s`；RTF p50/p95/p99=
  `0.199802/0.409690/0.596497`，status `178/119`（TRACKING/non-TRACKING），TF failure `18/300`，
  action 未成功。分类器把 `/lidar_odometry` 标为首个可见 timing 违反者；这不是 P1/P2、性能或安全通过证据。
- **已验证（P1 baseline，2026-08-19）**：新 domain `224` headless 的实际 recorder 窗口为
  `60.003753 s` 并正常完成，action 成功、终点误差 `0.146 m`、路径/reference/MPC 有输出，
  terminal `emergency_stop=true` 且 `/cmd_vel_mpc` 为零。`/rc_esdf/planning_grid` 与
  `/cmd_vel_mpc` 都观测为单一 publisher。
- **未通过（P1 freshness）**：`/lidar_odometry` p99/max wall interval=`1.144617/1.488598 s`，
  `/odometry`=`1.143652/1.490561 s`、`/localization`=`1.142361/1.490312 s`；RTF p50/p95/p99=
  `0.331409/0.502033/0.560295`，status `TRACKING/non-TRACKING=535/56`，TF failure=`16/600`。
  分类器将 `/lidar_odometry` 识别为首个可见违反者，`p1_admission_evidence=false`。该记录确立了
  freshness 缺口，不能标记 P1/P2、性能或安全通过。
- **已验证（最终 revision P1 baseline，2026-08-19）**：domain `225`、显式
  `ENABLE_CAMERA_SENSORS=false` 的 recorder 正常完成 `60.010472 s`，启动前无残留进程，planning grid 与
  `/cmd_vel_mpc` 均为单一 active publisher，终态 `emergency_stop=true`、`/cmd_vel_mpc` 为零。action 被接受，但 90 s 内没有终态，runner 返回
  `nominal action did not succeed`。
- **未通过（最终 P1 freshness）**：`/lidar_odometry` p99/max=`2.759540/2.942285 s`，下游
  `/odometry`=`2.764472/2.946450 s`、`/localization`=`2.764479/2.944695 s`；RTF p50/p95/p99=
  `0.303726/0.803858/1.017098`，status `419/158`（TRACKING/non-TRACKING），TF failure `27/600`。
  该结果与 domain `224` 的 action 成功但 freshness 不通过共同表明 P1 action 尚不具备重复性；建议避免把
  任一单次运行标记为 P1/P2、性能或安全通过。
- **推断 [Confidence: Medium]**：`loam_interface` 只在 `cloud_registered` callback 中发布
  `/lidar_odometry`；其 ROS stamp p99 为 `0.299990 s`，而 wall p99 为 `1.144617 s`，下游两段保持同量级，
  callback p99 为微秒级。因此优先调查 Gazebo 传感器/RTF、Point-LIO publisher cadence 与 DDS 丢包；
  尚不能把行为 owner 归因到其中任一单独组件。DDS counter 仍为
  `unverified_no_portable_rmw_counter`。
- **已验证（raw LiDAR 分层与 A/B，2026-08-20）**：最终 revision 的 recorder 已在新 domain 实际记录
  `/<robot>/livox/lidar -> /livox/lidar -> /cloud_registered -> /lidar_odometry`。domain `215` 的 `4 ms`
  physics candidate（默认 `10 Hz / 625 x 32`）raw/lidar-odometry/localization wall p99 为
  `1.610141/1.435872/1.430092 s`，action unsafe ABORTED；domain `214` 的 `5 Hz / 625 x 32` 保持
  SDF、bridge offset 与 Point-LIO 三处 `0.2 s` 周期一致，但 p99 恶化为
  `3.666797/3.312480/3.308619 s`；两者均不成为默认。domain `213` 仅移除 headless GUI state 的
  `SceneBroadcaster`，保留默认 physics 和 `10 Hz / 625 x 32`，但 p99 仍为
  `1.409273/1.322408/1.319348 s`，status `TRACKING/non-TRACKING=448/125`、TF failure=`35/601`，
  action unsafe ABORTED。三个 artifact 都完整 observer `>=60 s`，均为 `freshness_lidar_odometry`，
  不能标记 P1/P2、性能或安全通过。
- **已验证（时间契约与 runner）**：`LIVOX_UPDATE_RATE_HZ` 现在同步驱动 Gazebo SDF update rate、C++ bridge
  `scan_period_sec` 与 Point-LIO `mapping.lidar_time_inte`，默认仍为 `10.0 Hz`；`WORLD_SDF_PATH` 只在非空
  时转发，避免空 launch 参数阻断默认 world。`rmu_gazebo_simulator` 在本轮为 `32 tests, 0 errors,
  0 failures`，runner contract 和 freshness classifier 均通过。
- **推断 [Confidence: Medium]**：raw sensor、bridge、Point-LIO output 与 loam output 的 wall gap 仍同阶，
  而各 recorder callback p99 均为微秒级。现有证据否定了三项候选的收益，但仍不能在 Gazebo sensor publisher
  调度与 DDS 接收之间指定唯一 owner；下一步观测范围聚焦这两者的独立计数，freshness、安全和动作门限保持不变。
- **未通过（最新 P1 默认正式基线，2026-08-20）**：全新合法 domain `208` 使用默认
  `10 Hz / 625 x 32`、headless off-screen rendering、关闭相机、关闭 Transport 诊断订阅、关闭 Direct
  bridge、generic bridge `RELIABLE/KeepLast(10)`、`planning_grid_owner=rog_map`。完整 recorder 正常完成
  `60.016403 s`，启动前和结束后均无导航/仿真残留进程；`/clock` wall p99/max 为
  `0.078741/0.351209 s`，而 `/<robot>/livox/lidar`、`/livox/lidar`、`/cloud_registered`、
  `/lidar_odometry`、`/localization` 的 wall p99 依次为
  `2.954985/2.958803/3.327495/3.327198/3.324217 s`。分类器仍输出
  `first_violation=lidar_odometry`，status `TRACKING/non-TRACKING=362/210`，TF lookup failure 为
  `35/600`，故 `p1_admission_evidence=false`。
- **已验证（同一 domain 的安全收尾）**：domain `208` 曾观察到 JPS/reference/MPC、单一
  planning grid 与 `/cmd_vel_mpc` owner；但 action 最终 `ABORTED`，最终位置误差 `3.1606 m`，
  终态 `emergency_stop=true` 且 `/cmd_vel_mpc` 为零。地图持续因新鲜度失效而拒绝规划，这证明
  fail-closed 仍生效，不构成活跃导航成功证据。
- **未通过（当前 Gazebo 名义基线，2026-08-21）**：新 domain `208`、`planning_grid_owner=rog_map`、
  无 viewer/RViz 的 `TEST_PROFILE=nominal` 完成了 90.008 s recorder。JPS、MINCO、MPC、ROGMap adapter
  和 `/cmd_vel_mpc` 单一发布者均有实际观测，action 仍在 `90 s` 内未成功。`/lidar_odometry`、
  `/odometry`、`/localization` 的 wall interval p95/max 分别为 `1.427/2.744 s`、`1.424/2.750 s`、
  `1.419/2.757 s`；`/clock` RTF p50/p95/p99=`0.330/0.585/0.984`，分类结果为
  `p1_admission_evidence=false`、`freshness_lidar_odometry`。这是定位建图链的新鲜度失败，不是
  下位机、CAN、轮速、接触或底盘反馈门禁。
- **未通过（当前 selected P1，2026-08-23）**：domain `218`、默认 `10 Hz / 625`、headless 与
  `planning_grid_owner=rog_map` 的 recorder 正常完成 `60.008622 s`。`/lidar_odometry` p99/max=
  `0.818336/0.877152 s`，`/localization`=`0.818289/0.877242 s`，status
  `TRACKING/non-TRACKING=575/25`、TF failure=`4/601`；action `ABORTED`，最终
  `emergency_stop=true` 与 `/cmd_vel/selected=0`。运行期 selected owner 为 `1/2`，terminal 为 `1/1`；
  因定位/地图 fail-closed，JPS/MINCO/MPC path 与非零 selected 均未出现。这是安全行为，不是仲裁回归。
- **未通过（最新 Gazebo P1 对照，2026-08-30 domain `107/113`）**：domain `107` 的 7 级 freshness 全部通过，
  但 Point-LIO Z 轴发散至约 `292.56 m`、ROGMap 多次 reset，目标 `ABORTED`；domain `113` 目标成功、
  终点误差 `0.1628 m`，但 `gazebo_lidar` p99=`0.323823 s`，freshness 未通过。旧 domain `228` 的
  `lidar_odometry` 首违来自旧分类器阶段遗漏，不能继续作为当前 owner 结论。当前单次运行不足以确认
  freshness owner；Direct bridge 与 Transport observer 在本工作区尚无实际运行 artifact。
- **已验证（P1 owner 审计，2026-08-24；部署 + upstream 源码）**：运行入口实际解析到系统安装的
  `/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge`，Debian 包版本为
  `0.244.25-1jammy.20260608.160002`，workspace 中没有 `ros_gz_bridge` 源包。对应 upstream
  `0.244.25` 的 `Factory::create_gz_subscriber()` 在 Gazebo Transport 回调中同步转换并发布 ROS 消息，
  且未使用 `BridgeConfig::subscriber_queue_size`；项目 fork 只拥有 YAML/launch 的 topic、方向和 ROS
  publisher QoS 配置。`rmoss_gz_bridge` 仅提供 pose/RFID bridge，不是 generic LiDAR bridge owner。
  这触发了“owner 不在项目可修改范围”的风险转入条件：本轮未修改源码、未增加 publisher/DDS 计数、未运行
  新 domain，domain `228` 继续是最新运行失败证据，P1 DoD 仍未通过。现有 artifact 仍不能区分 Gazebo
  publisher 调度、generic bridge 回调处理与 Fast DDS 接收丢样。
- **已修复（验收产物一致性）**：domain `217` 一度执行了比源码旧的 `sensor_scan_generation` 与
  `localization_fusion` executable，导致空 frame/无 `map->odom` 的健康门禁失败。runner 现检查 arbiter、MPC、
  sensor generation、localization fusion 与 Gazebo recorder 的源码/二进制新旧，并在
  `runtime_preflight.txt` 中记录、失配即 fail-fast；该 domain 不作为 P1 算法证据。
- **未通过（P1 Transport 分层，2026-08-23 domain `219`）**：在默认 `10 Hz / 625`、headless、
  `planning_grid_owner=rog_map` 上开启只读 Gazebo Transport observer 后，60.009199 s recorder 记录了
  `599` 个 Transport PointCloudPacked 样本，wall interval p99=`0.108672 s`；同窗口 ROS
  `/<robot>/livox/lidar` 仅 `168` 个样本，wall/stamp p99=`0.639515/1.800000 s`，
  `/lidar_odometry`/`/localization` wall p99=`0.662703/0.662656 s`。因此首个可见退化位于
  Transport 到 ROS 原始 PointCloud2 边界；该 observer 自身是额外 Transport subscriber，对默认无 observer
  因果归因为 **[Confidence: Medium]**。action `ABORTED`、`p1_admission_evidence=false`，selected
  active/terminal owner 为 `1/2` 与 `1/1`，终态零速。
- **未通过（P1 DDS 元数据分层，2026-08-23 domain `220`）**：60.009184 s recorder 记录了 `600`
  个 Transport 样本（wall p99=`0.105195 s`），ROS 原始点云仅 `150` 个（wall/stamp
  p99=`0.802505/1.900000 s`），`/lidar_odometry`/`/localization` wall p99=`0.881395/0.881366 s`。
  Fast DDS 当前 RMW 不提供 reception `publication_sequence_number`，recorder 报
  `gazebo_lidar_dds_publication_sequence_supported=no`，全部 sequence 缺口计数保持 `0`，不可推导为零丢包。
  这排除了“用该字段区分 generic bridge 未发布与 DDS 接收丢样”的可能；仍是
  `freshness_lidar_odometry`、action `ABORTED`、`p1_admission_evidence=false`，没有非零 selected。
- **未通过（当前 MuJoCo red_box，2026-08-21）**：新 domain `207`、`planning_grid_owner=rog_map` 的
  首个 red_box 目标已接受，ROGMap adapter generation 从 `756` 增至 `769`，规划图 owner、JPS/MINCO/MPC
  节点与 `/cmd_vel_mpc` 单发布者均通过运行期检查；随后 Goal Manager 记录 `pose=(nan, nan)`，按既有
  算法 fail-stop 将 `emergency_stop` 置真，未完成首个目标。无效 pose 的源头尚未定位，故该运行不构成
  red_box、P2 或 P3 通过证据。
- **推断 [Confidence: Medium]**：在 domain `208` 中，raw ROS PointCloud2 已先于 Point-LIO/Loam 下游
  输出失去 cadence，故当前可观测边界收敛到 Gazebo sensor/Transport 与 generic `ros_gz_bridge` 的
  GZ-to-ROS 输出之间。此结论不能唯一归因 generic bridge：domain `212` 的 Transport 诊断曾显示
  Transport 端健康，但诊断订阅本身会改变该边界；下一台性能更高的机器建议以全新 domain 重跑默认基线，
  再用不增加长期 PointCloudPacked Transport subscriber 的计数或 trace 分开 publisher 慢与 ROS/DDS
  接收缺口。
- P1 runner 在 ROS graph 创建前检查新 ROS domain 的合法范围和残留导航/仿真进程，并把 candidate domain
  与仓库 SHA 写入 raw artifact；它不以主机资源统计决定是否启动。无故障运行中，只有至少 60 s observer、
  无 freshness 首违、status 全部 TRACKING、无 TF lookup failure 且 straight action 成功，才写入
  `p1_admission_evidence=true`。
- domain `233` 的尝试首先触发 Fast DDS 合法 domain 上限（`Calculated port number is too high`），不能作为
  Gazebo/导航运行证据。合法 domain `231` 曾启动 Gazebo 和导航链，health gate 通过并观察到
  JPS/MINCO/MPC 与非零 `/cmd_vel_mpc`；但 `/clock` RTF p50/p95=`0.2518/0.4815`、`/localization` wall interval
  p50/p95/p99=`0.484/1.506/2.185 s`，status `TRACKING/non-TRACKING=189/111`，TF lookup failure=`17/300`，
  action 未成功并最终 fail-closed。该历史结果不足以确定首个行为 owner，也不是 P1/P2 通过证据；
- production MINCO node 尚未把实时 `InitialKinematicState` 传入 optimizer；几何质量指标主要用于
  telemetry，尚未形成完整候选接受门禁；
- Gazebo runner 的 `TEST_PROFILE` 尚未拥有实际 corner/S/narrow/red-box 场景逻辑；
- QP node 仍固定 `map_fresh=false`、`collision_free=false`，`solver_mode=qp` 显式拒绝。

组件行为由源码与聚焦测试支持；domain 数值来自已保存运行记录。freshness 的唯一根因仍未确定，建议
逐级测量 `/lidar_odometry -> /odometry -> /localization -> status -> adapter`，不能仅凭相关性归因
Point-LIO、DDS、仿真 RTF 或 CPU 争用中的任一项。

## 4. 2026-09-18 P2 corridor 对抗前置诊断

### 已验证

- `scripts/test_gazebo_reloc_cell.sh` 现在在 `NAVIGATE_BEFORE_INJECTION=true` 时调用只读
  `ats_navigation_health_probe`，要求 localization 为 `TRACKING`、adapter `ready/status` 新鲜且
  epoch 一致、planning grid 有 payload，并连续满足 `3/3` 个样本；探针失败时 evaluator
  fail-closed，不发送导航 action 后的错误 `/initialpose`，也不把导航阶段 observation 计为 fault 后恢复。
- relocation harness 显式传入 `projection_rate_hz=0.2` 和 `livox_update_rate_hz=10.0`，并把参数及健康
  探针原始结果写入运行目录。Gazebo launch 的 `projection_rate_hz` 默认也保持 `0.2`，用于给 MINCO
  candidate 留出单一 immutable snapshot 的提交窗口；snapshot、swept-footprint、急停与 future-command
  拒绝逻辑没有放宽。
- 合法新 domain `221` 的单目标对照（`4.20,-4.30,0.0`）健康门通过：
  `stable=3/3`、`localization_state=1`、`localization_epoch=2`、`map_ready=true`、
  `map_status_ready=true`、`grid_payload=yes`。运行中 `/registered_scan` 为 `726` 条、`sim_starved=false`，
  action 仍在 `90 s` 超时；GT 终点距目标 `4.7634 m`、odom travel `0.1513 m`。
  日志同时记录 localization lease 中断、ROGMap candidate stale、`pose=(nan,nan)` watchdog 与 MPC
  reference timeout。该运行未发布错误 `/initialpose`，`post_fault_obs_accepted=0`，不是有效对抗样本。
- relocation harness 现在拒绝非法 `ROS_DOMAIN_ID>232`，避免把 Fast DDS 端口计算错误误记为算法失败。

### 已实现未运行

- corridor 完整多目标路线、`north_pocket` 路线和至少 `100` 次重复结构样本尚未在健康门改动后完成。
- health gate 的专用 node 级回归尚未新增；当前由脚本 `bash -n`、Python `py_compile`、Gazebo 构建和
  `--show-args` 覆盖静态/接口检查，运行证据来自 domain `221`。

### 推断

- [Confidence: High] 健康门不是 domain `221` action timeout 的根因，因为它在 action dispatch 前已连续通过，
  且 scan/adapter payload 持续存在。
- [Confidence: High] `projection_rate_hz=0.2` 比历史 `2.0/0.5 Hz` 减少了 snapshot stale 竞争，但仍不能
  保证执行期地图更新不会触发 runtime swept-footprint 拒绝；不能因此宣称主链闭环或导航能力通过。
- [Confidence: Medium] 当前失败由 localization lease、地图 snapshot 更新、runtime footprint gate 和
  watchdog/MPC freshness 共同影响，单一行为 owner 尚未锁定；不应通过降低 GICP overlap/information/inlier
  或放宽 snapshot/footprint/急停门限来换取到点。

### 未实现

- corridor 至少一个满足“action 成功、GT 到点、静止 hold、fault 注入、fault 后 accepted、recover hold
  3 s、错误接受 0、非有限接受 0”的有效样本仍未获得；因此不得开始 north_pocket 或宣称重复结构错误解
  拒绝已运行验证，也不得把 `<=0.30 m` 写作已验证能力。

## 5. 安全边界

- unknown、occupied、outside-map、ESDF sign/gradient、snapshot freshness 和 lease 继续 fail-closed；
- `/rog_map/esdf` 只是调试点云，不能作为数值规划输入；
- `/cmd_vel/autonomy_raw` 的唯一发布者为 `ats_swerve_mpc`，`/cmd_vel/selected` 的唯一发布者为
  `cmd_vel_arbiter`，最终执行端经 selected 接入；
- 地图、定位、TF、reference、ExecutionCommand、gimbal 任一不健康都触发零运动状态；
- 急停清空 tracker，恢复后旧 reference 保持失效；
- freshness timeout、QP iteration/deadline/residual 保持现值，不作为绕过失败的手段；
- planner collision/footprint 采样只用于导航算法安全复核；
- 未在目标机测量前建议避免引用报告中的 `50 Hz`、`6 ms` 或内存数据。

### 5.1 2026-09-18 MPC 授权时间门

- `ats_cmd_vel_arbiter` 与 `ats_swerve_mpc` 现在都以 `ExecutionCommand.header.stamp` 对接收时刻做
  producer-stamp lease 校验：未来样本、超过 `execution_command_timeout` 的旧样本，以及缺失身份/序列的样本
  不得获得自动执行权；MPC 侧拒绝时直接清空 tracker/warm-start 并发布确定性零速度。
- 该双侧门修复了“arbiter 已防重放但 MPC 被旁路时仍可装载未来/旧 reference”的边界。可直接复现的聚焦回归命令为：
  `source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/ats_swerve_mpc/test_mpc_localization_gate --gtest_filter='MpcLocalizationGateTest.RejectsFutureAndStaleExecutionCommandTimestamps:MpcLocalizationGateTest.RejectsReferenceReceivedWhileLocalizationIsUnhealthy:MpcLocalizationGateTest.RejectsOldSequenceAndOldEpochExecutionCommands'`
  ；本轮聚焦测试 `3/3` 通过，日志同时出现 timestamp reject、STOP 和零速路径。
- Goal Manager 的节点回归还验证了 accepted `ExecutionCommand` heartbeat 会递增 sequence 并刷新 producer stamp；运行该类节点测试时必须使用独立 `ROS_DOMAIN_ID`，否则外部 `/tf` 发布者可能污染“无 TF 应拒绝派发”的负向断言。
- 这只是组件级闭环证据，不等价于 Gazebo、MuJoCo 或实车主链通过；仍需在最后一次安全源码修改后重跑对应主链，
  并分别记录 action、GT、静止 hold、fault/recover、unique publisher/subscriber 与资源指标。
- `minco_planner` 收到新的 planning-grid generation 时，会在同一 map mutex 内清除旧的 active safety
  reference、置 `plan_safe=false` 并重新发布急停；若旧 reference 绑定了活动 goal，还会发送
  `FAILURE_SNAPSHOT_CHANGED` 触发 Goal Manager 瞬时重规划。地图 heartbeat 保持有效，但旧 generation
  不再被 runtime swept-footprint 复核或复活。新增 `PlannerSafetyState.NewMapInvalidatesPlanButKeepsHealthyMapLease`
  回归覆盖了“新 generation 可用、旧 generation 不可用、急停保持”的状态契约。
- Goal Manager 的 `ExecutionCommand` producer lease 默认回退 `20 ms` 再写入 `header.stamp`，用于吸收独立
  `/clock` 回调在仿真/多进程中的 tick 顺序差异；这是 producer 侧保守时间戳，不是放宽 MPC/arbiter 的未来样本
  拒绝门。`execution_command_stamp_backdate_sec=0` 可用于时钟已严格同步的 profile，负值和超过 `250 ms`
  的配置被约束；Goal Manager heartbeat 回归同时检查 sequence/stamp 单调刷新且不超前本地 clock。

## 6. 下一优化顺序

1. generic `ros_gz_bridge` owner 审计已确认活动实现属于系统安装包，当前四仓不拥有其转换回调或
   Gazebo Transport 接收代码。下一步优先由用户决定：把对应 `0.244.25` 源码纳入可维护的
   workspace/fork，或采用不修改项目默认链的项目外 trace；形成选择前暂缓源码修改，并继续只读审查与证据整理，
   下游计数不作为 generic publisher 的替代证据。获得该边界能力后，再以关闭诊断 observer、关闭 Direct bridge、
   generic `RELIABLE/KeepLast(10)` 的 `60 s` 默认 P1 基线复验。随后才以
   `OBSERVE_GAZEBO_TRANSPORT_LIDAR=true` 打开诊断进程寿命内的 Transport source observer，验证 Gazebo
   publisher 与 Point-LIO/loam cadence 的边界。已拒绝的
   `4 ms`、`5 Hz`、无 SceneBroadcaster、
   Direct bridge 与 `BEST_EFFORT/KeepLast(1)` candidate 保持非默认，仅作为历史诊断记录；
2. 本轮首违分类器已把 `/lidar_odometry` 标为首个可见边界，但建议避免根据单次 domain `224` 运行直接
   修改 timeout 或指定唯一算法 owner；
3. 将实际运动状态接入 MINCO 四条生产优化路径；
4. 把几何质量 telemetry 升级为按路径类别生效的候选门禁；
5. 实现真实 Gazebo straight/corner/S/narrow/nominal/red-box runner；
6. 重跑当前 revision 的 P2 名义、边界和故障矩阵；
7. 完成 RViz 全局/局部滑窗、clearance 和 continuous swept；
8. 完成 P3 Nav2-free action 生命周期；
9. 完成长时间性能和 MuJoCo 跨后端导航回归；
10. P2/P3/P4 通过后再推进 QP 主链和低速实车。

详细任务、DoD、验证命令和风险转入条件见：

- [ATS 导航剩余优化总 TODO](项目优化文档/ATS导航剩余优化总TODO.md)
- [下一阶段新对话提示词](项目优化文档/下一阶段提示词_ATS单雷达导航闭环与速度仲裁.md)
- [LTV-QP 后端准入记录](ats_swerve_mpc_ltv_qp_backend_admission.md)

## 7. 文档职责

- 本页：只记录当前阶段结论和架构边界；
- 总 TODO：只记录未完成任务、依赖、DoD 和状态，不累计完整日志；
- QP backend admission：记录后端来源、数值准入和 QP runtime 边界；
- `log/` artifact：保存原始运行结果，不提交大体积生成物；
- Git 历史：保留已退役阶段文档和过去实验的可追溯性。

任何新源码行为修改后，建议用该 revision 重跑相应仿真，建议避免复用修改前结果作为最终证据。
