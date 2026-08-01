# ATS P4 第四阶段稳定跟踪、行为决策与双仿真提示词

下面内容可直接作为下一轮 Codex 任务提示词。它以当前 P4 第三阶段导航代码为基线，同时推进稳定跟踪和行为决策 ATS action 迁移；loopback 用于低成本决策/action 验证，MuJoCo 用于真实规划控制与四舵轮物理闭环。不得再次从头实现导航链，也不得用 loopback 的成功替代 MuJoCo 或实车证据。

```text
请继续在工作区 `/home/kong/ATS_2026_snetry_test` 开展 ATS 2026 四驱四转哨兵导航研发。

本轮目标是 P4 第四阶段与行为决策并行准入：稳定跟踪、定位漂移分离、reference 动力学可行性、ATS 行为树 action 迁移、loopback/MuJoCo 同场景分层验证、端到端时延补偿和受控上场灰度。

必须完整阅读并遵守：
1. `/home/kong/ATS_2026_snetry_test/AGENTS.md`
2. `docs/项目优化文档/nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md`，重点是 5.9~5.12
3. `docs/项目优化文档/nav2移植/p4_real_robot_calibration_preflight.md`
4. `docs/项目优化文档/nav2移植/p4_stage4_stable_tracking_prompt.md`
5. 五个独立仓库的 branch、status、HEAD、origin/develop 和最近提交
6. 本轮直接相关的 producer/consumer、launch、配置和测试完整函数/类作用域，特别是 `ats_sentry_behavior` 的 action/条件节点、主树、server、参数，Goal Manager ATS action，以及两套仿真入口
7. 不得读取或修改 `参考/`

五个独立仓库：
- 根仓 `/home/kong/ATS_2026_snetry_test`
- 导航仓 `src/ats_sentry_nav`
- MuJoCo 仓 `src/sim/ats_mujoco_sim`
- 行为决策仓 `src/ats_sentry_behavior`
- loopback 仿真仓 `src/sim/loopback_sim`

当前用户修改必须保留：loopback 仓 `params/nav2_params.yaml` 已有未提交修改；其余未知修改和未跟踪文件也一律按用户文件处理。

当前推送前静态审计记录的待验证项，不得未经运行证据直接改成新设计：
- `test_mujoco_minco_mpc_chain.sh` 的全程 debug/速度/ExecutionCommand capture 使用单个 `GOAL_TIMEOUT`；多目标总时长超过单目标超时时可能提前结束捕获。下一阶段先以长时 rectangle/red_box 复现，再决定是否把全程 capture deadline 按目标数计算。
- `fake_vel_transform` 的 current/initial yaw、spin speed 和 controller activity time 在多个 callback/timer 间共享；若实机容器使用多线程 executor，需要用线程检测或并发测试证明现有 callback group 串行性，否则再补明确同步。
- Goal Manager/MPC 正式配置固定 `require_gimbal_status=true`；`false` 兼容路径的 request/feedback 语义尚未作为支持模式验收，不得在实机用它绕过 gimbal acknowledgement。
- MuJoCo 的 yaw-authority request subscriber 当前未声明 transient-local durability；名义启动已通过，但 gimbal simulator/process 在 pending request 中途重启后的恢复语义需要专用测试。
- MINCO authority policy 对非有限 clearance 当前忽略并继续评估其他点；正式安全 checker 会阻断 unknown/outside，但仍需单测锁定“不可用 clearance 应保守 BODY 还是直接 HOLD/STOP”的策略。

必须保留的架构边界：
- Point-LIO/sensor_scan_generation 提供局部连续 odometry；localization_fusion 独占 `/localization` 与 `map -> odom`。
- small_gicp 只提供带质量和原始时间的全局重定位观测，不直接覆盖局部 odom。
- ROGMap/adapter、RC-ESDF、JPS、MINCO S3、独立 yaw、continuous swept footprint、Local Collision Repair、SE2 MPC 和四舵轮约束必须保留。
- state 为世界系 `[x,y,yaw]`，control 为车体系 `[vx,vy,wz]`；禁止 DDR/ICR/`vy=0`。
- Goal Manager 的 `ExecutionCommand` 是唯一执行授权；legacy Path、`emergency_stop=false` 或 ready heartbeat 不得重新授权 MPC。
- `/cmd_vel_mpc` 只能由 MPC 发布，`/motion_control` 只能由 twist bridge 发布。
- BT 只能拥有任务优先级、目标选择和 ATS action 的 cancel/preempt 意图；不得发布正式 reference、急停、ExecutionCommand 或任何底盘 Twist。
- ATS action result 是任务成功权威；行为层不得以位置距离单独替代 Goal Manager 的 position/yaw/terminal velocity/dwell 成功条件。
- `fake_yaw` 是云台雷达兼容/观测 frame，不覆盖真实 body yaw；不得删除兼容 TF 或新增重复 `base_footprint -> base_link` publisher。
- `GIMBAL_COMPENSATED`、`BODY_YAW_FOLLOW`、`HOLD_SAFE_STOP` 及 stop/re-ack/fresh-reference 切换协议必须保留。
- 禁止从 `/rog_map/esdf` PointCloud2 反解析距离场，禁止放宽 unknown、footprint、stale、TF、epoch/generation 或执行器物理门禁换取路线通过。

开始修改前必须输出：
1. Definition of Done；
2. 精确文件范围；
3. 可执行验证清单；
4. 当前假设、未验证项、停止条件；
5. 五仓 baseline commit 与用户已有改动清单。

总原则：
稳定跟踪按“先测量定位 -> 分离误差来源 -> 固化 baseline -> reference 可行性 -> 时延/模型补偿 -> 最后有限调权重”推进。行为决策按“接口审计 -> ATS action 迁移 -> BT 单测 -> loopback 场景 -> MuJoCo 同场景 -> HIL”推进。两条工作流共享 scenario、revision 和验收指标，但不得互相掩盖失败；不得先写“权重小、轮胎滑、云台导致漂移”等结论。

零、先修复行为树接入契约

当前已确认的静态事实必须作为迁移起点，不得写成已完成：
- `src/ats_sentry_behavior` 已有 RMUL/RMUC 主树、巡逻、补给、防守、视觉接管、受击、自旋和云台节点；但正式树仍使用 `nav2_msgs` 的 `/navigate_through_poses` 与 `/navigate_to_pose`。
- 当前 `SendNavThroughPoses` 是 `BT::SyncActionNode`，发出异步 goal 后立即返回 SUCCESS，branch halt 没有对应 cancel 回调；只在后续发送不同 goal 时取消旧 goal。
- 当前树用 `IsPathGoalReached` 的位置容差参与完成判定，而 Goal Manager 的正式成功还要求 yaw、终端线/角速度和 dwell。
- 当前树包含 `PublishTwist`；`cmd_spin` 会在 `fake_vel_transform` 中直接叠加到输出 `angular.z`，会绕过 MPC/ExecutionCommand。
- behavior server 当前硬编码订阅 `global_costmap/costmap`、`odom` 和 `odometry`；ATS 正式输入应为 `/rc_esdf/planning_grid` 与 `/localization`。
- 当前 `loopback_decision_sim.launch.py` 启动完整 Nav2，只能作为旧对照，不能证明 ATS Nav2-free 决策链。
- 行为仓当前 `BUILD_TESTING` 只有 ament lint，未发现主树优先级、action halt/cancel、迟到 result 或 waypoint 状态机的聚焦功能测试；README 仍把 `/navigate_through_poses` 写成统一执行接口。

当前提交前静态审计记录了以下待复核项。本轮不扩大修改范围；下一轮必须先用针对性测试确认，再决定是否修复，不能直接写成运行故障：
- `[Potential Bug]` Goal Manager 的 `require_gimbal_status=false` 分支不创建 pending yaw request，但 reference commit 与 MPC 仍要求非零 request sequence；兼容关闭模式可能无法进入 EXECUTE。正式配置当前为 true，未影响已有名义运行。
- `[Potential Bug]` 运行中 gimbal status 失配时 Goal Manager 会清 active execution 并 fail-stop，但当前静态路径未看到它同步回到 waiting/planning 并派发 fresh reference；可能只停机直到 action timeout。必须注入 stale/错误 ack 验证恢复状态机。
- `[Potential Bug]` MINCO yaw authority policy 忽略非有限 clearance；当整段 clearance 不可用时可能选择 `GIMBAL_COMPENSATED`。需要明确选择 `HOLD_SAFE_STOP` 或保守 `BODY_YAW_FOLLOW`，并补全 NaN/全 NaN 测试。
- `[Potential Bug]` MuJoCo 的 gimbal status yaw 来自模型 world/odom 朝向，但 `header.frame_id` 当前填 robot base frame；需要统一消息字段的参考 frame，并做非零初始 yaw round-trip 测试。
- `[Test Harness Risk]` 全局 topic capture 的存活时间当前按单个 `GOAL_TIMEOUT`，多段路线可能提前退出；`YAW_AUTHORITY_EXPECTED=auto` 目前偏向记录而不是逐条验证全部 EXECUTE 的非零 request/feedback。下一轮先补脚本自测，不得由单次 red_box 通过推断所有场景均受保护。

目标职责链固定为：
`裁判/视觉/任务输入 -> ats_sentry_behavior BT -> /ats_navigate_to_pose -> ats_goal_manager -> JPS/MINCO -> ExecutionCommand -> MPC -> 底盘`

必须完成：
1. 新增 ATS `NavigateToPose` BT action client，优先复用 BehaviorTree.ROS2 的异步 action 基类；支持 feedback、result、cancel、halt、preempt、timeout、server unavailable/restart 和 ATS result-code 映射。BT tick 不得无界阻塞等待 server。
2. branch halt、比赛结束、视觉失效、任务优先级切换和 server shutdown 必须取消活动 ATS goal；旧 callback/result 用 request/UUID/generation 隔离，不能修改新任务。
3. 正式成功只接受 ATS action `RESULT_SUCCEEDED`。行为层位置检查最多用于候选去抖或诊断，不得提前推进巡逻 cursor、waypoint 或任务状态，也不得把目标 pose 写回伪装成观测 pose。
4. 单点视觉任务直接发送单点 ATS action。对 CSV/Path，先逐类确认中间点是“任务语义 waypoint”还是“旧 Nav2 几何路径”：前者按顺序执行多个单点 ATS action 并等待每点 result，后者只提交任务终点并由 JPS/MINCO 重新规划。禁止重新引入 Nav2 `NavigateThroughPoses`。
5. 正式 XML/profile 移除或隔离 `PublishTwist`。重构 `cmd_spin`：若是车体自旋意图，必须进入 Goal Manager/MINCO/yaw reference 的授权链并受 MPC/footprint/执行器约束；若是纯云台意图，必须改用不会叠加 body `wz` 的云台接口。不得继续在 MPC 后加角速度。
6. 云台命令必须与 yaw authority 仲裁：`BODY_YAW_FOLLOW` 的锁定请求和实际 feedback ack 优先于视觉/扫描云台动作；BT 不得发布第二套 yaw authority 或自行伪造 lock ack。
7. 将 behavior 的地图、定位和 action topic 配置化。ATS profile 使用 `/rc_esdf/planning_grid`、`/localization`、`/ats_navigate_to_pose`；旧 Nav2 topic 只允许存在于名称明确、不能与正式链同时启动的对照 profile。
8. 视觉候选点选择可以读取 planning grid 进行任务级筛选，但不能成为最终安全判据；unknown/outside 必须保守处理，最终碰撞安全继续复用 RC-ESDF、yaw-aware footprint、continuous swept checker 和 Local Collision Repair。
9. 新增聚焦 BT/action 测试：正常 success、ATS result-code 映射、halt 触发 cancel、迟到 goal response/result 丢弃、server unavailable/restart、目标去抖、语义 waypoint 顺序、位置已近但 terminal yaw/velocity 未满足时不提前成功，以及正式 profile 不加载 `PublishTwist`/Nav2 action。
10. 迁移实际运行通过后再更新行为仓 README；文档必须同时保留旧 Nav2 对照 profile 的边界，不能先写“ATS 行为树已接入”。

一、统一 telemetry 与真值

新增或补齐结构化 telemetry/CSV/rosbag，至少记录：
- reference `[x_ref,y_ref,yaw_ref]`、world velocity/acceleration 和时间戳；
- localization `[x,y,yaw]`、twist、状态、covariance、观测时间、publication time、age；
- body command `[vx,vy,wz]` 与 world/body conversion；
- cross-track、along-track、wrapped yaw、velocity error；
- tracker progress、reference age、trajectory deadline、MPC solve p50/p95/p99、fallback reason；
- 每轮 target/actual RPM、wheel acceleration、steer target/actual/rate/error；
- saturation 起止时间、持续时间、占空比，不得只有累计 count；
- longitudinal/lateral slip、contact、command-to-wheel delay；
- gimbal/body/fake yaw、authority、lock request/ack sequence 和 feedback age；
- localization epoch、ROG source generation、adapter publication sequence、MINCO local generation、ExecutionCommand sequence；
- planner/MPC/recovery failure reason。

必须提供版本化的汇总工具，输出 JSON/CSV：
- cross-track、along-track、yaw error 的 p50/p95/p99/max；
- localization ATE/RPE/yaw drift（只有存在外部真值时）；
- reference/localization/command age 与 solve time p50/p95/p99；
- terminal position/yaw/linear/angular velocity 和 dwell；
- saturation 占空比、slip、contact、replan/recovery 次数。

没有外部真值时，只能报告“localization 与 reference/仿真真值的差异未独立验证”，不得把控制 tracking error 当作定位漂移。

二、固定 baseline 与拒绝规则

在任何算法/参数优化前，用当前 revision 和固定配置对以下路线各运行至少 10 次：
- 直线；
- 90 度转弯；
- S 弯；
- 保持 yaw 的横移；
- 横移+转弯；
- 窄道 BODY_YAW_FOLLOW；
- 坡道/起伏 BODY_YAW_FOLLOW；
- 开阔区 GIMBAL_COMPENSATED；
- rectangle；
- red_box `(-0.04,-4.08)`。

每个 run 固定：五仓 revision、Release build、地图、参数、行为树 XML、scenario、模型、seed、初始位姿、目标、ROS_DOMAIN_ID 规则、硬件/仿真版本。保留所有失败，不得只选择最优 run。

候选优化前冻结主要指标、最小改善和 guardrail。若没有足够 baseline 样本，只能记录“门槛待冻结”，不得事后按候选结果改阈值。

三、定位与云台雷达漂移分离

必须分别证明或排除：
- lidar/gimbal/body 外参方向、平移、零位或单位错误；
- 云台 encoder 时间、点云时间、odometry 时间和 TF 查询时间不一致；
- 旋转雷达点云未按点/包 deskew；
- fake_yaw 初始非零 yaw、TF 查询方向和逆速度变换错误；
- map->odom 更新、local odom 连续性或 relocalization observation 延迟；
- 低几何/重复结构导致 GICP 假匹配或不可观；
- 定位/地图短暂异常被错误归为 MPC 跟踪误差。

至少使用两类证据：实现+单测、TF graph+rosbag、外部真值+telemetry、故障注入+状态机。

真实旋转云台模型或实机 encoder 未接入前，不得宣称 GIMBAL_COMPENSATED 已验证物理地图匹配保真。

四、reference 动力学可行性与自适应 time scaling

在 MINCO candidate 提交 Goal Manager 前，复用现有四轮位置、轮径、传动比和 MPC 执行器限制进行前视检查：
- 将 world reference velocity 正确转换到 body frame；
- 预测每轮 velocity vector、RPM、wheel acceleration、steer angle/rate；
- 检查 body yaw rate、速度、加速度和 terminal stop；
- 保持 yaw-aware footprint、RC-ESDF、continuous swept checker 与 repair；
- 不重复实现 collision geometry。

若 reference 不可跟踪，优先：
1. 全局或分段延长时间；
2. 在曲率、yaw rate、低 clearance、预计 steer saturation 和坡道段降速；
3. 必要时重新运行 MINCO；
4. 若仍不可行，结构化拒绝并 STOP。

禁止只放宽 RPM、轮加速度、舵速或 MPC 输入限值。必须新增测试：直线、90 度、横移、纯 yaw、yaw wrap、急转弯饱和、terminal stop、time scaling 后物理约束全部满足。

五、端到端时延与控制优化

reference 可行后再测量并处理：
- sensor acquisition -> localization；
- localization -> MPC callback；
- MPC solve -> `/cmd_vel_mpc`；
- bridge -> `/motion_control`；
- command -> wheel/steer feedback。

只有测得 p95/p99 delay 后，才允许实现有界状态前推或 actuator delay model；必须对最大预测时域、stale/future stamp、clock jump 和 solver overrun fail-stop。

再评估：
- reference velocity/acceleration feedforward；
- terminal controller/terminal set；
- steering actual state 或 wheel feedback 进入预测约束；
- warm-start reset 与 solver fallback；
- MPC horizon duration，而非只看 sample count。

权重优化一次只改一个因果参数组：position、yaw、velocity/control effort、terminal。每组必须有 baseline/candidate/ablation；改善主要指标但导致 clearance、饱和、solve deadline、slip 或 stop guardrail 回退时拒绝候选。

六、净空误差预算

每个可执行 sample 必须满足：

`C_min > e_track_99 + e_loc_99 + v*tau_99 + d_brake + m_map`

其中 C_min 来自现有 yaw-aware footprint clearance；e_track/e_loc 必须独立测量；tau 是 sensor-to-actuator p99；d_brake 来自实测速度/坡度制动；m_map 包含地图分辨率、外参和环境变化余量。

预算为负时必须降速、延时、重规划或 STOP。不得缩 footprint、把 unknown 改 free 或忽略定位不确定度。

七、yaw authority 与分区策略

- 开阔区：允许 crab，GIMBAL_COMPENSATED，不因云台角度驱动 body yaw。
- 窄道/坡道/接触敏感段：BODY_YAW_FOLLOW，必须有云台锁定反馈；yaw 同时考虑 footprint clearance、切线、坡向和任务姿态。
- planning grid 当前没有 terrain/slope 来源标签时，用显式 route/profile 选择并记录限制；不得假装已自动识别。
- mode switch while moving 必须验证 STOP -> 清 tracker/warm start -> ack -> fresh sequence/reference -> EXECUTE。

八、故障与恢复矩阵

每项使用新 ROS_DOMAIN_ID 和新 launch：
- gimbal feedback stale、错误/迟到 ack、lock 丢失；
- TF loss、map stale、localization stale/lost；
- old epoch/generation/publication/reference/command sequence；
- solver timeout/infeasible/NaN；
- wheel/steer feedback stale、bridge/controller process restart；
- authority switch while moving。

必须观察：
`ExecutionCommand STOP -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0 -> four-wheel 0 rpm`

恢复只能接受新 request、ack、map/localization version 和 fresh reference；旧 tracker/reference/warm-start 不得复活。

九、BT 决策与双仿真场景矩阵

建立一个版本化 scenario schema 和 runner，两套仿真必须消费同一份输入，而不是分别手工发 topic。每个 scenario 至少包含：
- `scenario_id`、schema version、seed、相对仿真时间；
- 裁判 game/HP/ammo/outpost/RFID 输入；
- 视觉 tracking/nav_hold/target_id/pose/confidence/freshness 输入；
- 期望 BT branch、任务类型、目标/waypoint、robot/gimbal mode；
- 期望 ATS action send/feedback/cancel/preempt/result 序列与 transition deadline；
- loopback mock action outcome 或 MuJoCo 实际 fault injection；
- 明确 pass/fail/stop 条件。

必须记录结构化 decision trace，至少包含：`scenario_id`、event sequence、输入 stamp/age、BT tick、selected branch/task、goal fingerprint、action UUID、ATS goal_id、feedback state、result code、cancel/preempt reason、yaw authority、ExecutionCommand sequence。某层不产生的字段必须明确写为 `N/A`；loopback 不得伪造 Goal Manager 的 `ExecutionCommand`。不得只用自由文本日志或 topic 存在判断通过。

场景至少覆盖：
1. 未开赛、比赛结束和裁判输入 stale：不得发送运动 goal，活动 goal 必须 cancel，MuJoCo 完成五级归零；
2. 开赛进入巡逻，waypoint 顺序和 cursor 只在 ATS success 后推进；
3. HP/弹量进入和退出补给阈值，验证迟滞且不在边界抖动；
4. 极低 HP 的 defend/retreat 优先级；当前树若只切 robot mode 而继续旧导航，必须先由测试暴露再按任务策略修复；
5. 关键时间/前哨站状态切换；
6. 视觉目标稳定后接管、目标切换 hold、短遮挡保持、vision stale 后 cancel 并返回巡逻；
7. 补给途中出现视觉目标、视觉过程中资源降级、受击策略与 yaw authority 的优先级；
8. 不变输入、阈值噪声和高频目标抖动：不得形成无界 action preempt storm；
9. ATS goal reject、planning failure、map unready、TF failure、timeout、cancel、preempt，验证有界 retry/recovery；
10. BT server、mock server、Goal Manager、planner/MPC restart，确认旧 action/result/reference 不复活；
11. authority switch while moving、gimbal feedback stale、lock 丢失、localization/map stale；
12. rectangle、red_box 和比赛巡逻/补给固定路线的完整任务序列。

双仿真分工：
- BT 单测/离线 tick：验证黑板、优先级、迟滞、halt/cancel 和确定性状态迁移；
- ATS loopback：使用独立 `loopback_decision_ats.launch.py` 或等价入口，运行同一 BT/XML/scenario、相同决策参数和受控 ATS mock action server，只验证任务与 action 生命周期；不得启动正式 profile 的 Nav2 action，也不得声称验证 JPS/MINCO/MPC/四舵轮；
- MuJoCo：使用独立 behavior launch 将同一 BT/XML/scenario 和相同决策参数接到真实 `/ats_navigate_to_pose`、Goal Manager、JPS/MINCO/MPC 和 yaw authority；验证路线、终端、饱和、slip、contact 和 stop 链；
- 每套使用不同 `ROS_DOMAIN_ID`、独立临时目录和独立日志。资源足够时可并行运行，但禁止共享 ROS graph、状态文件或通过 bridge 把一套结果伪装成另一套结果。

决策重复性门禁：固定输入下，离线/loopback 每个场景至少连续 20 次得到相同的 branch/action 有序 trace，且无孤儿 goal、重复 success、迟到 result 污染或无界 retry；MuJoCo 关键决策场景至少 10/10 完成并满足原有安全、tracking 和 terminal 门禁。

十、初始准入门禁

在 baseline 后冻结最终阈值。进入低速实车前至少满足：
- 每个目标场景 10/10 完成，无 contact、无错误 owner、无旧 reference 复活；
- terminal position p95 <= 0.08 m；terminal yaw p95 <= 0.10 rad；
- terminal linear <= 0.05 m/s、angular <= 0.10 rad/s、dwell >= 0.30 s；
- tracking/localization/time/braking 组成的 clearance budget 始终为正；
- MPC/reference/localization p99 不超过 lease/deadline；
- 饱和报告持续时间和占空比，不允许稳态持续饱和；
- rectangle south/north 保持真实非零 `linear.y`；
- red_box 到达 `(-0.04,-4.08)` 并满足 terminal yaw/velocity/dwell；
- 全部故障完成五级归零链。

十一、HIL 与实车灰度

严格按 Gate 0 offline/loopback/MuJoCo -> Gate 1 执行器禁用 HIL -> Gate 2 低能台架 -> Gate 3 封闭低速地面 -> Gate 4 代表性路线推进。每级固定 speed/acceleration/torque/workspace、operator、独立物理急停、停止条件和回滚 commit。

任何未命令运动、方向/幅值异常、定位跳变、TF/epoch 异常、负 clearance budget、持续饱和、contact、通信丢失、watchdog 未归零或缺少关键 telemetry 都立即停止并回退上一 gate。

十二、实施与交付

必须按：
`接口审计/测量 -> baseline -> 最小实现 -> Release build -> BT/组件单测 -> scenario replay -> ATS loopback -> MuJoCo -> HIL/实车门禁 -> 文档 -> 分内容中文提交 -> 普通 push`

执行五仓 `git diff --check`；只显式 stage 本轮文件；禁止 `git add .`、`git add -A`、force push。提交按“行为接口、BT 状态机、loopback 场景、导航/控制、MuJoCo、文档”拆分，只 push 实际修改仓库，未修改仓库不制造空提交。最后一次影响 BT action、planning/yaw/MPC/control 的源码修改后必须重跑对应 loopback 决策矩阵、完整 MuJoCo rectangle 和 red_box。

最终报告必须列出：
- 首个被证实根因与两类证据；
- 修改文件与唯一所有权；
- frame/time/QoS/version/fallback 契约；
- BT branch/goal/action/cancel/preempt/result 有序 trace 与孤儿 goal 检查；
- loopback 与 MuJoCo 同 scenario 的对应结果及两者证据边界；
- baseline 与 candidate 各场景重复统计；
- cross-track/along-track/yaw/terminal/latency/saturation/slip/contact；
- clearance budget；
- 故障矩阵与恢复；
- 未运行项、残余风险和实车授权边界；
- 五仓 baseline/final commit、实际修改仓库与 push 结果。
```
