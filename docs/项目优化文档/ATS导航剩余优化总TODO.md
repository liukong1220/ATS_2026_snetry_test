# ATS 导航剩余优化总 TODO

> 状态：唯一活动导航优化清单
> 更新时间：2026-09-01
> 适用范围：Gazebo、MuJoCo 与实机导航软件侧的 ATS 四驱四转哨兵导航链
> 历史说明：旧阶段 TODO 已退役；历史实现与运行证据通过 Git 历史、
> `docs/ats_swerve_mpc_ltv_qp_backend_admission.md` 和状态文档追溯。

## 0. 2026-09-01 当前执行清单

- [x] **2026-09-01：Gazebo P1 在完整门禁集（含双动态 TF 门禁）下连续三次通过 `147/149/151`**，
  固定默认 profile，退出码均为 0，三次都是 `all_p1_gates_passed`、`failures: 0`。桥侧 ~2 s 延迟缺陷
  **未修复**，只是三次未复现（`143` 同 profile、同产物却失败），因此这是“当前门禁集下的连续通过”；
- [x] **2026-09-01：MuJoCo `single` 在独立新 domain `178` 通过**（exit 0，终点 0.0343 m，owner 逐条唯一，
  50 Hz 离线取证 q1=yes/q2=no/q3=no）；
- [x] **2026-09-01：arbiter 专项 GTest 已运行**：24 tests, 0 errors, 0 failures, 0 skipped，覆盖优先项 2/3/6；
- [x] **2026-09-01：`red_box` 根因改判**——由 `176` 的 50 Hz 离线取证定位为执行/跟踪偏差（q2=yes、q3=no），
  推翻此前“栅格过报”的解释；修法属 MPC 跟踪/限速侧，`escape_from_contact_enabled` 不得打开；
- [x] Gazebo 仿真 profile 按 SI 单位覆盖 Point-LIO `acc_norm=9.81`、`satu_acc=30.0`，实机参数保持不变；
- [x] TF warm-up 计数拆为建立前失败、首次建立和建立后失败，准入同时要求链实际建立；
- [x] Codex 安全 review：planner 与 goal manager 的实验 escape 默认关闭，RMUC profile 显式关闭；runner 对
  任一 `footprint_collisions>0` 保持失败；
- [x] 运行产物预检改为明确比较实际载入 artifact，源码目录、artifact 或扫描缺失时 fail-closed；
- [x] P2 六故障矩阵已在独立新 domain 全部执行并通过，当前为 `6/6`：`adapter_lease` 154、
  `service_timeout` 156、`input_stale` 158、`unreachable` 166、`freeze` 168、`unknown` 170，
  退出码均为 0。`unknown` 恢复新目标已通过（action result code 0，终点 0.012154 m，
  observer 20 项检查全通过，恢复延迟 2.218 s，`reference_path_after_recovery=[]`）；
- [x] 产物审计在本轮真实拦截了一次污染：`unreachable` 162 与 `freeze` 164 以退出码 3 在导航门禁之前
  被拒，原因是一次并发的 `colcon build --packages-select ... ats_rc_esdf minco_planner ...`
  在 13:15:56 重链了 `libats_rc_esdf.a` 却没有包含 `ats_rog_map_adapter`。重链 adapter 后两例
  重跑通过。同一次并发构建与 `unknown` 160 的运行窗口重叠（13:15:50–13:18:33），因此 160 的产物
  来源不干净、已作废并在 170 重跑。**验收运行期间不得并发构建**；
- [x] `unknown` 恢复新目标的停车与重规划起点已修复并在 domain `170` 独立确认（result code 0）。
  **归因边界**：与上一次失败之间没有导航代码提交（根仓 `git log --since=2026-08-30` 只有文档与脚本，
  `src/ats_sentry_nav` 最后一次提交为 `efd68e1`，工作区唯一 nav 代码改动是 `ats_rog_map_adapter_node.cpp`
  里纯诊断的 `tf_failure_detail`），所以这是安全修正后的复跑结果，不能声称由某次定点修改导致；
- [ ] `red_box` 仍为**未通过**，失败目标编号在三次运行间漂移：domain `152` 失败于目标 9
  `highland_ramp`（`final_distance=0.548`），domain `174` 失败于目标 8 `east_mid`，domain `176`
  失败于目标 5 `south_lane_entry`——而目标 5 是前两次都零碰撞通过的一段。三次终局形态一致：
  车停住后每条 MINCO 轨迹都在 `first_index=0` 被拒、`escape_allowed=0`，goal manager 最终
  `no-executable-plan budget exhausted`（`map_ready=0`）。**`152` 与 `174` 不构成同条件对比**：
  `scripts/test_mujoco_minco_mpc_chain.sh` 的 mtime 为 13:26:48，晚于 `152`（13:09）、早于 `174`（13:33），
  `174` 起才带每段 50 Hz 的 `nav_tracking_recorder` 进程，`152` 全程没有它（日志 0 行）。
  可做同条件对比的是 `174` 与 `176`（同 runner、同产物、同 profile），二者失败目标仍不同；
- [x] `red_box` 根因已用 domain `176` 的离线取证定位，并**推翻**了此前“停车位姿落入 footprint
  冲突带（栅格过报）”的解释。`python3 scripts/analyze_nav_tracking.py --input-dir <leg>
  --length 0.60 --width 0.50 --safety-margin 0.02` 对 `176` 每段独立判定：
  q1（发布时参考轨迹无碰撞）=yes，q3（同一位姿被地图更新从 free 翻成 occupied）=**no**，
  即冲突是真实几何而不是地图翻转或量化过报；q2（实际位姿离开参考包络）在目标 1–4 为 no、
  目标 5 为 **yes**：最大偏航误差按段为 0.0089 / 0.1927 / 0.3961 / 0.0940 / **1.1073 rad**，
  最大横向误差 0.053 / 0.179 / 0.037 / 0.047 / **0.275 m**，目标 5 有 337/360 个评估 tick 在碰撞。
  首个冲突在该段第 2.367 s、`pose=(1.1495, -7.0767, yaw=-1.7926)`、footprint 最小间隙 **−0.100 m**
  （已嵌入障碍 0.1 m），当时 `localization_age=0.0098 s`、`map_ready=true`、`emergency_stop=false`，
  MPC 请求 `vx=1.0283` 而 `motion_control.linear_x` 恰好被夹到 `1.0`，实测 `wz=-0.691` 对指令 `-0.526`，
  `drive_acceleration_saturation_count=11731`。因此顺序是**先执行/跟踪偏差把车开进真实障碍**，
  随后 fail-closed 提交门（`escape_from_contact_enabled` 默认 `false`，理由见
  `src/ats_sentry_nav/minco_planner/include/minco_planner/safety/escape_prefix.hpp`）使其再也无法提交轨迹而死锁。
  证据边界：q2 只覆盖有 reference+snapshot 配对的 tick（各段 93–360，对应总样本 446–1733），
  analyzer 不评估物理接触；
- [ ] `red_box` 修复方向（未实施，留给 Codex/用户决策）：按 analyzer 自带 routing
  「audit MPC tracking error, execution latency, velocity/acceleration limiting and the stopping envelope」，
  先核对 MPC 指令与底盘可达加速度/限幅的一致性，而不是放宽 footprint margin 或 obstacle threshold；
  `escape_from_contact_enabled` 不得为了通过而打开——`escape_prefix.hpp` 已写明它缺少 snapshot 绑定的
  逃逸授权，打开即属于“用放宽换通过”；
- [ ] **失败段同时丢掉 analyzer 与物理接触两份证据**，根因是 `fail()` 直接 `exit 1`
  （`scripts/test_mujoco_minco_mpc_chain.sh:338-343`），而 leg 收尾里的
  `capture_contact_telemetry after` → `assert_no_physical_contact` → `stop_nav_tracking_recorder`
  → `analyze_nav_tracking_leg` 全部排在动作成功判定之后（约 2010–2019 行）。后果分两种：
  - analyzer 可离线补跑，`176` 目标 5 的结论就是这样取得的（recorder 虽然在 SIGINT 时以
    `RCLError: failed to initialize wait set` 崩退，数据在崩退前已落盘）；
  - **物理接触读数不可补**：`contact_violation_count` / `max_contact_force` 只能从运行中的
    `/swerve/telemetry` 取，仿真拆掉后就没有了。所以 `176` 只有目标 1–4 的
    `CONTACT: ... contact_violation_delta=0 ... max_contact_force_n=0.0` 四行，**失败的目标 5 没有任何
    接触读数**，`assert_no_physical_contact` 在该段从未执行。`174` 同理只有 7 行（覆盖成功段）。
    因此“footprint 最小间隙 −0.100 m”是几何结论，**目标 5 是否真的发生刚体接触仍未测量**——
    既不能说发生了，也不能说没发生。修法是把 after-capture 与 analyzer 移到失败路径也会经过的
    收尾里（trap/EXIT 或先采集后判定），未实现；
- [ ] domain `174` 全程 `/localization` 零消息（RMW 报 `incompatible QoS ... RELIABILITY`，
  recorder 侧请求的是 BEST_EFFORT，`localization_fusion_node` 用 `SensorDataQoS()`），
  该运行的 q2/q3 因此不可判；`176` 用同一脚本同一 QoS 正常收到（134 条/段），
  故这是 `174` 运行期一次性的发现顺序问题，不是工具与 profile 不匹配；
- [ ] 原“修复目标 9 候选索引覆盖冲突带、抑制终端东向过冲”与“从目标 9 安全终态独立验证目标 10”
  两项的前提（栅格过报）已被 q3=no 推翻，改为在跟踪偏差修好后重新评估是否仍需要；
- [x] 增加动态 TF stamp/age 门禁：`tf_dynamic_age_p99_s` 与 `tf_dynamic_stamp_staleness_p99_s`
  双门禁（各 0.5 s），配 distinct-update 速率下限、`update_gap_max` 与 stamp 异常计数；
  两个百分位及其样本数缺失时 fail-closed。`scripts/test_gazebo_dynamic_tf_gate.sh` 19 例，
  两个门禁与两处 fail-closed 均经变异测试；
- [x] Gazebo P1 阻塞点已定位到 `ros_gz_bridge parameter_bridge`：同一次运行内对比雷达在桥两侧的
  证据（domain `143`，`OBSERVE_GAZEBO_TRANSPORT_LIDAR=true`）——gz-transport 侧 898 帧、
  间隔 0.099994 s、stamp age p50 0.002 s；ROS 侧只剩 659 帧（丢 26.6%）、间隔 0.136471 s、
  stamp age p50 1.982 s。通过的 domain `131` 在同一默认 profile 下两侧均为 600 帧、
  age 0.012 s。Gazebo 与传感器在通过/失败两次运行中完全一致，唯一变量是桥；
- [x] `parameter_bridge` 已补入 `capture_launch_process_resources` 的进程过滤：此前该过滤只覆盖
  `gz_livox_bridge_node`、`pointlio_mapping` 等，唯独漏掉证据所指向的桥进程，导致桥的 CPU、线程和
  非自愿上下文切换在全部九次运行里都没有被测量。`scripts/test_gazebo_runner_contract.sh` 新增断言
  只匹配 `ps` 过滤表达式本身（第一版被我自己写的注释满足、变异存活，已修正后变异被杀）；
- [x] 一个候选根因已被证据否证：`OBSERVE_GAZEBO_TRANSPORT_LIDAR` 不是通过/失败的区分变量——
  domain `127/129/131` 该开关为 `true` 且通过，`133/135/137/139/141` 为 `false` 且全部失败。
  录制器的 gz-transport 订阅不是桥延迟的成因；
- [x] 用补齐后的进程测量重跑 Gazebo P1（domain `147`，默认 profile + `OBSERVE_GAZEBO_TRANSPORT_LIDAR=true`），
  该次**通过**，并首次拿到 `parameter_bridge` 的资源基线。`147` 与 `143` 的 active 窗口长度相同
  （90.43 s 对 90.40 s），因此 tick 增量可直接比较（每秒 tick）：`ign gazebo` 85.10 对 77.32、
  `pointlio_mapping` 32.59 对 15.60、`gz_livox_bridge_node` 12.82 对 8.14——**通过的那次在三个进程上
  CPU 都更高**。这否证了“主机 CPU 争用导致桥延迟”：若是争用，失败运行应当压力更大，而实测是失败运行
  下游做的功更少，正是输入被拖慢后的表现。`parameter_bridge` 在通过运行为 21.48 tick/s、
  非自愿上下文切换 90 s 内仅 98 次（健康时几乎不被抢占）；
- [ ] 仍缺一次**带该进程测量的失败运行**才能判定桥自身每帧成本：`143` 及更早九次运行都在补测之前，
  `147` 又通过了，因此“桥内部成本 vs 外部饿死”目前无法定论，不能推断；
- [x] 同一次运行内的分级证据再次确认插入点就是桥：`147` 为 gz-transport 0.002 s → ROS 侧
  `gazebo_lidar` 0.022 s（+0.020）→ `livox_input` 0.042 → `cloud_registered` 0.042 →
  `localization` 0.042 → `tf_dynamic_age_p99` 0.122 s；`143` 为 0.002 → **1.982**（+1.980）→ 2.042
  → 2.232 → 2.232 → 2.302 s。两次同一默认 profile，链路结构一致，唯一插入 ~2 s 的环节相同；
- [x] Gazebo P1 三次连续通过：**已达成 3/3**，全部使用固定默认 profile（`TEST_PROFILE=nominal`、
  `P2_FAULT_CASE=none`、`PLANNING_GRID_OWNER=rog_map`、`USE_DIRECT_GAZEBO_LIDAR_BRIDGE=false`、
  `LIDAR_BRIDGE_PUBLISHER_RELIABILITY=reliable`、`LIDAR_BRIDGE_PUBLISHER_DEPTH=10`），
  三次都是 `p1 admission evidence: true`、`reason=all_p1_gates_passed`、`failures: 0`：
  - domain `147`（`log/gazebo_minco_mpc_chain/20260901_135358_nominal_none_domain147`）：
    `tf_dynamic_age_p99_s=0.122043`、`tf_dynamic_stamp_staleness_p99_s=0.200000`、
    `tf_dynamic_update_gap_max_s=0.200108`；
  - domain `149`（`log/gazebo_minco_mpc_chain/20260901_135756_nominal_none_domain149`）：
    `tf_dynamic_age_p99_s=0.122037`、`tf_dynamic_stamp_staleness_p99_s=0.100000`、
    `tf_dynamic_update_gap_max_s=0.200008`、`tf_lookup_failures=2`（全部在 established 之前，
    `failures_after_establishment=0`）、`goal_action_final_distance_m=0.0656`、
    `terminal_localization_goal_error_m=0.1163`、`post_goal_localization_delta_m=0.1792`；
  - domain `151`（`log/gazebo_minco_mpc_chain/20260901_140013_nominal_none_domain151`）：
    `tf_dynamic_age_p99_s=0.112042`、`tf_dynamic_stamp_staleness_p99_s=0.100000`、
    `tf_dynamic_update_gap_max_s=0.199670`、`tf_lookup_failures_after_establishment=0`、
    `goal_action_final_distance_m=0.0334`、`terminal_localization_goal_error_m=0.1646`、
    `post_goal_localization_delta_m=0.1942`。
  三次的 owner 计数一致：`selected_cmd_vel_publisher_max=1`、`selected_cmd_vel_subscriber_max=2`、
  `planning_grid_publisher_max=1`、`planning_grid_publisher_names=/ats_rog_map_adapter`、
  `planning_grid_named_non_adapter_seen=no`、`planning_grid_anonymous_endpoint_seen=no`。
  分级 stamp age 在三次中都停在桥前：gz-transport 0.002 s → `gazebo_lidar` 0.022 s（+0.020），
  与 `143` 的 0.002 → 1.982（+1.980）形成对照；
- [ ] 该 3/3 的证据边界：桥延迟缺陷**未被修复**，只是三次未复现。`143` 与 `147/149/151` 使用同一
  默认 profile 与同一批产物，因此缺陷是**间歇性**的，三次连续通过不等于该缺陷已消除；后续任何
  Gazebo 运行仍可能重现 ~2 s 插入。三次运行的 `minimum_clearance_m`、`minco_footprint_collisions`、
  `gazebo_contact_telemetry` 均为 `unverified`，物理接触仍**未验证**；
- [x] MuJoCo 独立 contact evaluator **已实现并已在运行中产出读数**：runner 的
  `capture_contact_telemetry` / `capture_contact_force` / `assert_no_physical_contact` 从
  `/swerve/telemetry` 读 `contact_violation_count` 与 `max_contact_force`，按单个目标窗口取**增量**
  （计数自仿真启动累计、按物理步累加，绝对值无意义），telemetry 读不到时 **fail-closed**，计数回退时
  判仿真已重启。它与离散 `footprint_collisions` 并列、互不替代：前者是刚体求解器算出的接触
  （`contact_is_violation()` 只计机器人与非地面几何体的接触，四轮正常接地不计入），后者只是 MINCO
  轨迹采样点的几何自检。单元级门禁 `bash scripts/test_mujoco_contact_gate.sh` **exit 0**
  （`RESULT: MuJoCo contact gate test PASSED`，含 `force_reported_on_reject` 的拒绝分支）。
  domain `178` 的实测读数：`CONTACT: single leg contact_violation_delta=0
  contact_violation_before=0 contact_violation_after=0 max_contact_force_n=0.0`，
  因此 MuJoCo `single` 的物理接触是**已验证为零**，不再只依赖 `footprint_collisions=0`。
- [ ] Gazebo 侧仍无对应物理接触证据：`147/149/151` 三次的 `minimum_clearance_m`、
  `minco_footprint_collisions`、`gazebo_contact_telemetry` 均为 `unverified`，
  `物理接触评估 未验证`。Gazebo 接触遥测未实现，`footprint_collisions=0` 在 Gazebo 侧继续不替代
  物理接触结论。

当前状态：Gazebo P1 在固定默认 profile 下**三次连续通过（3/3）**：domain `147`、`149`、`151` 均为
`p1 admission evidence: true`、`all_p1_gates_passed`、`failures: 0`，双动态 TF 门禁读数分别为
`tf_dynamic_age_p99_s` 0.122043 / 0.122037 / 0.112042 与 `tf_dynamic_stamp_staleness_p99_s`
0.200000 / 0.100000 / 0.100000（限值各 0.5 s）。此前 domain `133/135/137/139/141/143` 六次全部失败，
更早通过的 domain `131` 无该门禁。桥延迟缺陷本身**未修复**、只是未复现：`143` 与这三次同 profile、同一批产物，
失败模式是桥侧 ~2.0 s 雷达延迟使 `ats_rog_map_adapter` 把 `map <- gimbal_yaw_odom` 判为未来外插
（`143` 在 sim t≈21.4 s 跨过 `[0,0.1]` 接受窗），planning grid never ready、action abort。
因此该 3/3 是“当前门禁集下的连续通过”，不是“缺陷已消除”。此前 domain `127/129/131` 的通过结论只在
“无动态 TF 新鲜度门禁”的证据边界内成立，不再作为 P1 通过依据；三次运行的物理接触仍为未验证。
MuJoCo P2 六故障矩阵已在独立新 domain 全部执行并通过，为**6/6**（154/156/158/166/168/170）。
`red_box` 仍为**未通过**：三次运行分别失败于目标 9（`152`）、目标 8（`174`）、目标 5（`176`），
失败目标编号漂移。根因已由 `176` 的离线取证定位为**执行/跟踪偏差**（目标 5 最大偏航误差 1.107 rad、
footprint 最小间隙 −0.100 m），而不是此前认为的栅格过报——同一位姿的 free→occupied 翻转判定为 no；
车进入真实障碍后，默认 fail-closed 的提交门使其无法再提交任何轨迹而死锁。
MuJoCo 侧物理接触已有独立读数：`154/156/158/166/168/170/178` 与 `174/176` 的成功段均为
`contact_violation_delta=0`、`max_contact_force_n=0.0`；但**失败段没有接触读数且不可补测**，
Gazebo 侧接触遥测仍未实现（`147/149/151` 的 `gazebo_contact_telemetry=unverified`）。
`red_box` 未通过与 Gazebo 侧接触证据缺失，使 P2 总体准入仍为**未通过**。

## 1. 目标与完成定义

本清单用于收敛当前仍未完成的导航生产能力和准入证据。它不把组件测试、单次仿真成功与实际导航
任务成功混为同一状态。底盘 CAN、电机、轮速、电流、电压、温度、底盘反馈和硬件 watchdog 由下位机
或 HIL 诊断维护，不属于本导航清单的检测、门禁或通过条件。最终目标链保持不变：

```text
传感器 + 独立状态估计
-> ROGMap 概率占据/膨胀/3D ESDF
-> 地面投影与 2.5D 可通行语义
-> RC-ESDF 规划接口
-> ATS Goal Manager -> JPS -> MINCO S3 + 独立 yaw
-> footprint safety + Local Collision Repair
-> 全向 SE(2) MPC -> 自主速度源 -> 云台 yaw 速度变换 -> 速度源仲裁 -> 下位机速度接口
```

总体验收建议同时满足：

- 干净主机可从远端仓库复建全部依赖和仿真资源；
- 地图、定位、规划、控制和导航速度输出各有唯一 owner；
- nominal、边界场景和故障恢复均有独立 ROS domain 的证据；
- stale、unknown、无路、unsafe、solver failure 或 lease failure 都确定性零速度；
- P2、P3、P4 和 QP 主链分别通过自己的门禁，不相互替代；
- 仿真与实机导航侧分别保留独立 artifact；下位机/HIL 诊断不构成导航准入；
- 性能结论来自固定 revision、配置、硬件和原始 artifact。

### 1.1 2026-08-21 交接更新

- MuJoCo 默认场景已切换为与 Gazebo 同源的 RMUC 2025 模型、world 几何与高度场；`red_box`
  的最终目标固定为 map 坐标 `(10.45, 0.35)`，即原图像标注的中央高地位置；
- RMUC 2025 profile 将 terrain 连续风险 `0..99` 与硬障碍 `100` 区分开。adapter 与 MINCO
  的 `terrain_obstacle_value_threshold`/`obstacle_value_threshold` 均为 `100`，不再把风险值 `63`
  误判为墙体；
- 独立 physics/navigation launch、RMUC 场景契约测试与南侧通道分段回归已有组件或运行证据；
  完整 `red_box`、默认单点和 P2 故障矩阵建议在本次提交 revision 的全新 ROS domain 重跑后才能验收；
- **已实现并经静态验收**：实机、MuJoCo 与 Gazebo 的自主源均为
  `/cmd_vel/autonomy_raw`；实机经既有 fake/chassis 变换到 `/cmd_vel/autonomy`，四端的最终执行速度
  都以 `/cmd_vel/selected` 为唯一允许 Twist 输入。`cmd_vel_arbiter` 是 selected 的唯一 publisher；自动源
  由新鲜 `ExecutionCommand` 租约放行，手动源可不依赖该授权，但两源均受 emergency stop、链路失效和各自
  timeout 归零。Gazebo adapter 保留 big-yaw frame 变换和 `/motion_control`/Gazebo chassis 的唯一 owner；
  loopback 不再以 `/motion_control` 绕过仲裁。
- **已验证（loopback arbiter 出口，2026-08-23 domain `222`）**：重装当前 launch 后实际启动 arbiter，
  `/cmd_vel -> /cmd_vel/selected -> loopback_simulator` 有非零 `vx=0.3` 输出，selected 的
  publisher/subscriber 为 `cmd_vel_arbiter/loopback_simulator=1/1`，`/odom.x=0.825`（artifact：
  `/tmp/ats_loopback_arbiter_domain222.fEjFyT`）。该试验仅验证
  手动源仲裁出口，不覆盖 MPC 授权、完整导航或物理仿真。
- **未验证**：无需 remap 的键鼠到串口或 `/motion_control`、MuJoCo nominal/red_box/P2 fault matrix 和物理
  接触。历史 `/cmd_vel_mpc` artifact 只用于迁移前的 freshness 诊断；当前 selected 链最新默认 P1 基线
  为 domain `228`，仍未通过。
- **未通过（Gazebo P1，2026-08-23 domain `218`）**：默认 `10 Hz / 625`、headless、`rog_map` owner 的
  recorder 正常完成 `60.008622 s`。`/lidar_odometry` 是 first violation，wall p99/max=
  `0.818336/0.877152 s`；`/localization`=`0.818289/0.877242 s`，status
  `TRACKING/non-TRACKING=575/25`、TF failure=`4/601`，action `ABORTED`。运行期
  `/cmd_vel/selected` owner=`1/2`、terminal=`1/1` 且零速；定位/地图 fail-closed 后没有 JPS/MINCO/MPC
  path 或非零 selected。这不构成速度仲裁回归，也不满足 P1/P2。
- **未通过（最新 Gazebo P1，2026-08-23 domain `228`）**：默认 `10 Hz / 625`、headless、`rog_map` owner、
  关闭 Transport observer/Direct bridge 的 recorder 正常完成 `60.001345 s`。`/lidar_odometry` 是 first
  violation，wall p99/max=`0.650163/0.743549 s`；`/localization`=`0.650164/0.743591 s`，raw Gazebo LiDAR
  wall p99/max=`0.708598/0.714906 s`，`/clock` p99=`0.010387 s`、RTF p99=`1.032094`，status
  `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`。active `/cmd_vel/selected` owner=`1/2`，无非零
  selected，JPS/MINCO/MPC path 为空，故 `p1_admission_evidence=false`、原因为
  `freshness_lidar_odometry`。这是定位/地图 fail-closed，不是仲裁回归；artifact：
  `log/gazebo_minco_mpc_chain/20260823_211350_nominal_none_domain228/`。
- **已修复（P1 预检）**：domain `217` 暴露 install executable 早于
  `sensor_scan_generation`/`small_gicp_relocalization` 源码。Gazebo runner 现检查 arbiter、MPC、两定位节点与
  Gazebo recorder 的源码/可执行文件新旧，失配时写入 `runtime_preflight.txt` 并 fail-fast；217 artifact
  不作为算法验收。
- **未通过（P1 上游分层，2026-08-23 domain `219/220`）**：两个 60 s 窗口内 Gazebo Transport
  PointCloudPacked 分别记录 `599/600` 个样本，wall p99=`0.108672/0.105195 s`；ROS raw
  `/<robot>/livox/lidar` 仅 `168/150` 个样本，wall p99=`0.639515/0.802505 s`，下游
  `/lidar_odometry` p99=`0.662703/0.881395 s`。domain `220` 还确认 Fast DDS RMW 不支持
  reception publication sequence（`supported=no`），sequence 计数 `0` 不能解释为零丢包。两次均
  `freshness_lidar_odometry`、action `ABORTED`、`p1_admission_evidence=false`；Transport observer 为额外
  subscriber，对默认链的因果归因仍是 **[Confidence: Medium]**。
- **已验证（2026-08-30 交付复核）**：MuJoCo runner 增加关键运行产物新鲜度检查，arbiter 增加断链恢复与
  车体系分量回归，MuJoCo LiDAR 增加 MuJoCo 3.4/3.10 `mj_multiRay` 参数兼容层。MuJoCo single 与
  `adapter_lease/service_timeout/input_stale` 独立故障用例通过；`red_box`、`unknown`、`unreachable` 的
  失败分别记录为真实规划缺陷、注入时序竞态和故障前提未成立，物理接触仍未验证。
- **已验证（arbiter 专项 GTest，2026-09-01）**：`colcon build --base-paths src --packages-select
  ats_cmd_vel_arbiter --cmake-args -DCMAKE_BUILD_TYPE=Release`（exit 0）+ `colcon test --base-paths src
  --packages-select ats_cmd_vel_arbiter`（exit 0）+ `colcon test-result --test-result-base
  build/ats_cmd_vel_arbiter --all`（exit 0）：`test_cmd_vel_arbiter.gtest.xml` **24 tests, 0 errors,
  0 failures, 0 skipped**（合计 25 tests，含包级 1 项）。24 例覆盖优先项 2/3/6：手动新鲜优先与手动
  超时归零（`FreshManualPreemptsAuthorizedAuto`、`ManualTimeoutDoesNotResurrectOldCommand`、
  `ManualTimeoutRecoversToAuthorizedAuto`、`ManualTimeoutThenNewManualIsAccepted`）、自动源需要有新鲜
  `ExecutionCommand`（`AutoWithoutAuthorizationStaysZero`、`StaleOrReplayExecutionCommandIsRejected`、
  `AutoTimeoutDoesNotResurrectOldCommand`、`ExecutionLeaseExpiresWithoutNewCommand`）、STOP/新化身/急停/
  断链使旧自动失效（`StopRevokesAutoAndRequiresNewAutoSample`、`StopDoesNotClearFreshManual`、
  `NewIncarnationRequiresStopBeforeExecute`、`EmergencyStopZerosBothSources`、
  `ExecutionCommandDuringEmergencyStopIsNotHonored`、`LinkDownZerosBothSources`、
  `LinkHeartbeatTimeoutZerosOutput`、`MissingLinkHeartbeatZerosOutput`、
  `AuthorizationDuringLinkDownIsNotHonored`）、DOWN→UP 只接受恢复后新到命令
  （`LinkDownToUpDiscardsCommandsBufferedWhileDown`、`HeartbeatTimeoutThenLinkUpDiscardsBufferedCommands`、
  `ManualReceivedBeforeInitialLinkUpDoesNotRevive`）、以及车体系 `[vx,vy,wz]` 分量保持
  （`BodyFrameHolonomicComponentsArePreserved`）。该结论是单元级的，不替代整链 owner 与零速链实测。
- **已验证（MuJoCo `single` 独立新 domain，2026-09-01）**：`ROS_DOMAIN_ID=178 TEST_PROFILE=single
  bash scripts/test_mujoco_minco_mpc_chain.sh`，**exit 0**，`PASS: MuJoCo JPS/MINCO/clearance-aware yaw/
  SE2 MPC 'single' profile completed.`。运行前产物新鲜度审计全部通过（`ats_cmd_vel_arbiter`、
  `ats_swerve_mpc`、`ats_goal_manager`、`minco_planner`、`ats_rog_map`、`ats_rog_map_adapter` 为
  `fresh`，`ats_rc_esdf` 为 `propagated` 到 `libminco_planner.so` 与 `ats_rog_map_adapter_node`）。
  owner 唯一性逐条通过：`/cmd_vel/autonomy_raw` `ats_swerve_mpc -> cmd_vel_arbiter`、
  `/cmd_vel/selected` `cmd_vel_arbiter -> twist_to_motion_ctrl`、`/motion_control`
  `twist_to_motion_ctrl -> ats_mujoco_sim`、`/rc_esdf/planning_grid` 单一 publisher 属
  `ats_rog_map_adapter`、`/planner/execution_command` `ats_goal_manager -> ats_swerve_mpc` 且
  `cmd_vel_arbiter` 为只读 observer。终点 `final_distance=0.0343 m`，
  `RESULT: single generation=26 raw_points=3 reference_points=24 footprint_collisions=0
  escape_prefix_end=0`，零速链 `stop from last_nonzero_selected_command: 0.1132 m over 0.518 s`。
  50 Hz 离线取证（`/tmp/ats_nav_evidence/nav_tracking_single_178/goal_1_single/verdict.json`）：
  q1=`yes`、q2=`no`、q3=`no`，`ticks_evaluated=104`、`ticks_unpaired_frame=0`、`colliding_ticks=0`、
  `swept_colliding_segments=0`、`max_abs_yaw_error_rad=0.0082`、`max_abs_lateral_error_m=0.0033`、
  `min_clearance_m=+0.400`。该组读数与 `red_box` 目标 5 的 1.107 rad / −0.100 m 形成直接对照，
  说明缺陷是 profile 相关的执行偏差而非全局跟踪能力缺失。`footprint_collisions=0` 与
  `colliding_ticks=0` 仍不替代物理接触结论（verdict 自带 `physical contact is not evaluated here`）。
- [ ] 该离线取证工具的一个证据边界（新记录）：`verdict.json` 的 `recorder_summary.publishers` 在
  d178 中每个 topic 都是 `count: 0` 且带 `RCLError: ... rcl node's context is invalid`——枚举发生在
  SIGINT 关停之后，节点 context 已失效。因此 recorder 的 publisher 枚举**不携带 owner 结论**（既不
  支持也不反驳），owner 数量只能取自 runner 自身的 owner 审计与 Gazebo `active_ownership.log`；
  修法是把枚举移到关停之前，未实现。
- **已验证（离线取证层与门禁的单元测试，2026-09-01，全部 exit 0）**：
  `python3 scripts/test_analyze_nav_tracking.py`（`RESULT: nav tracking analyzer test PASSED`）、
  `python3 scripts/test_footprint_evaluator.py`（`RESULT: footprint evaluator test PASSED`）、
  `bash scripts/test_footprint_evaluator_parity.sh`（`PARITY: compared 1280 cases against the linked
  C++ FootprintSafetyChecker`，`RESULT: footprint evaluator parity PASSED`——Python 评估器与实际链接的
  C++ `FootprintSafetyChecker` 逐例对齐，这是离线结论可以引用在线语义的依据）、
  `bash scripts/test_gazebo_dynamic_tf_gate.sh`（`RESULT: dynamic TF gate test PASSED`，含
  `no_staleness_samples -> evidence=false reason=tf_dynamic_staleness_samples_missing` 与
  `aborted_action_outranks_evidence_gates -> evidence=false reason=straight_action_not_succeeded`，
  即门禁顺序本身被当作正确性属性测试）、`bash scripts/test_mujoco_contact_gate.sh`
  （`RESULT: MuJoCo contact gate test PASSED`）。
  `python3 scripts/test_nav_tracking_recorder.py` 需要先 source 工作区：未 source 时以
  `ModuleNotFoundError: No module named 'ats_navigation_interfaces'` **exit 1**；
  `source /opt/ros/humble/setup.bash && source install/setup.bash` 后 **exit 0**
  （`RESULT: nav tracking recorder test PASSED`）。这些都是单元级结论，不替代整链实测。
- **已验证（四环境 launch/config 契约与优先项 4/5，2026-09-01）**：
  `python3 scripts/test_validate_navigation_config.py` **exit 0**（`Ran 11 tests ... OK`）、
  `python3 scripts/validate_navigation_config.py` **exit 0**（`PASS: formal single-source behavior,
  navigation configuration, and ROGMap visualization contract`）、
  `bash scripts/test_gazebo_runner_contract.sh` **exit 0**（`PASS: Gazebo runner runtime contract`）。
  该校验器同时断言四个环境的速度链拓扑：实机（`bringup.launch.py` +
  `rm_navigation_reality_launch.py` + `navigation_launch.py`，MPC `command_topic=/cmd_vel/autonomy_raw`、
  `IfElseSubstitution(launch_fake_vel_transform, '/cmd_vel/autonomy_gimbal', ...)`、串口节点
  `cmd_vel_topic` 默认 `/cmd_vel/selected`）、MuJoCo（`rmuc_2025_mujoco.launch.py`：
  `"input_topic": "/cmd_vel/selected"`、`"require_serial_link": False`）、Gazebo
  （`"selected_cmd_vel_topic": "/cmd_vel/selected"`、adapter `input_topic` 同）、loopback/HIL
  （`'command_topic': '/cmd_vel/selected'`）。
  - 优先项 4（避免重复 `base_footprint -> base_link`、fake yaw 关闭时保留零旋转兼容 TF）在
    `src/ats_sentry_nav/ats_nav_bringup/launch/navigation_launch.py:79-88` 由条件互斥保证：
    `static_transform_publisher_base_footprint_to_base_link` 带
    `UnlessCondition(use_robot_state_pub)`，`static_transform_publisher_fake_yaw_compat`
    （`gimbal_yaw_odom -> gimbal_yaw_fake`，无 rpy 参数即恒等旋转）带
    `UnlessCondition(launch_fake_vel_transform)`。Gazebo profile 另有单 owner 说明：
    `odom -> base_footprint` / `odom -> gimbal_yaw_odom` 唯一发布者是 `sensor_scan_generation`，
    且该 profile 有意不启动 `fake_vel_transform`/`chassis_vel_transform`。**结论是静态/结构级的**；
  - 优先项 5（缺必要大 yaw 反馈时 Gazebo 底盘与 `/motion_control` 同时归零）在
    `chassis_command_logic.py:46-49` 是 fail-closed 的单点：`transform_with_big_yaw and big_yaw is None`
    且 `require_big_yaw` 时返回 `ChassisCommandOutputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True)`——六个分量
    全零，因此 `/motion_control` 与底盘 `Twist` 由同一输出驱动、不可能只归零一路。
    `python3 src/sim/gazebo_simulator/rmu_gazebo_simulator/tests/test_chassis_command_logic.py`
    **exit 0**（`Ran 2 tests ... OK`）。该结论是单元级的，未在 Gazebo 运行中注入过缺失 yaw 反馈；
- **已修复（Gazebo freshness 分类器）**：阶段表覆盖 Transport、raw LiDAR、Livox、registered scan、
  lidar odometry、odometry、localization 与 status；旧 domain `228` 的 `lidar_odometry` 首违属于阶段遗漏。
  domain `107/113` 出现 freshness 与动作结果反转，单次 run 不足以确认 owner；Direct bridge/Transport
  observer 在当前工作区尚无实测 artifact。

## 2. 当前冻结基线

### 2.1 仓库基线

| 仓库 | 当前 revision | 远端状态 | 说明 |
| --- | --- | --- | --- |
| 根仓 | `011338b` | `origin/develop` 待本轮同步 | 文档、runner 与离线证据工具 |
| 导航仓 | `efd68e1` | `origin/develop` 已同步 | ROGMap、JPS/MINCO、Goal Manager、MPC |
| MuJoCo | `54a7c01` | `origin/develop` 已同步 | 仿真模型与 launch |

受保护的用户内容继续保留：

- `src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py`；
- `src/ats_sentry_nav/ats_swerve_mpc/求解器.md`；
- Gazebo fork `scripts/ats_bridge/gz_livox_bridge.py`。

建议避免读取其内容作为设计依据，建议避免删除、覆盖、暂存或提交。

### 2.2 已实现并有组件证据

- ROGMap 数值 projection、adapter 融合、全局 RC-ESDF display 与局部滑窗 debug 已接线；
- 两份 RViz 已分层显示全局融合 RC-ESDF、局部 ROGMap、JPS、MINCO、MPC predicted 和 executed；
- `PathGeometryPreprocessor`、`MincoTimeAllocator`、`TrajectoryQualityEvaluator` 已实现；
- 直线不增弯、曲率感知时间分配、ESDF backtracking、动态限制和安全回退有聚焦 GTest；
- Gazebo 四舵轮 `[v_x,v_y,w_z]` 运动学与命令唯一 ownership 已实现；
- ATS Goal Manager 已有 action、feedback、cancel、preempt、timeout 和有界重规划代码；
- OSQP v1.0.0、固定 CSC、warm-start ABI、complete-phase 计时和 `qp_shadow` 已实现；
- 默认仍为 `solver_mode=ilqr`，`solver_mode=qp` 显式拒绝。

### 2.3 当前证据边界

- Gazebo `147/149/151` 只代表当前动态 TF 门禁集下的三次连续通过，bridge 延迟故障仍可能复现；
- MuJoCo `154/156/158/166/168/170` 的 P2 故障矩阵为 `6/6`，`red_box` 仍未通过；
- `single` domain `178` 有 reference/actual 零碰撞与 contact telemetry 零增量证据，不能外推到失败段或 Gazebo；
- QP 仍为 shadow/显式拒绝状态，未形成主链准入证据。

## 3. 稳定契约

- Point-LIO 继续拥有 `/localization` 和 `/registered_scan` 的状态估计输入链；
- ROGMap 不是定位器，建议避免用 ground truth 替换正式定位；
- adapter 仅消费 ROGMap 数值 projection，建议避免反解析 `/rog_map/esdf`；
- unknown、occupied、outside-map、signed-distance 正负号和 gradient 语义建议避免放宽；
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 SE(2) MPC 建议保留；
- 四舵轮控制保持车体系 `[vx,vy,wz]`，建议避免差速、Ackermann、ICR 或 `vy=0`；
- 实机/MuJoCo 中 `/cmd_vel/autonomy_raw` 的唯一 publisher 为 `ats_swerve_mpc`，
  `/cmd_vel/selected` 的唯一 publisher 为 `cmd_vel_arbiter`，最终出口订阅 selected；
  Gazebo、MuJoCo、loopback 与实机均适用该判据；历史 Gazebo `/cmd_vel_mpc` 记录建议避免误作当前
  owner 证据；
- 急停建议清空 tracker，急停前 reference 建议避免在恢复后复活；
- 不能通过增大 freshness timeout、QP iteration、residual 或 deadline 掩盖失败；
- `solver_mode=qp` 在 QP-3 门禁通过前继续拒绝启动。

## 4. 优先级与依赖关系

```text
P0 远端复建
  -> P1 Gazebo localization freshness
    -> P2 MINCO 生产契约补齐
      -> P3 Gazebo 场景 runner
        -> P4 P2 完整仿真验收
          -> P5 RViz/ROGMap 运行验收
          -> P6 P3 Nav2-free 验收
          -> P7 P4 仿真算法安全

P1/P4 通过 -> QP-2 Shadow 可配对复核 -> QP-3 受控主链切换
所有分支 -> 长时间导航稳定性 -> 受限低速实机导航
```

建议避免跳过 P0/P1 直接调 MINCO 或 QP；污染环境中的 timing 仅保存为无效样本。

## 5. P0：远端复建与 revision 冻结

### 工作项

- [x] 将 `ats_robot_description` 本地提交 `dea591e53fa0` 推送到用户远端；
- [x] 将 push remote 改为可非交互使用的 SSH 地址；
- [x] 确认活动引用只来自 `ats_robot_description`，没有 `pb2025_robot_description`；
- [x] 在临时干净目录执行 `vcs import dependencies.repos`；
- [x] 记录每个仓库实际 checkout SHA，而不是只记录分支名；
- [x] 生成验收用 locked manifest，固定关键依赖 commit；
- [x] 在干净工作区完成最窄 Gazebo 导航包构建、CTest 与 `--show-args`；
- [x] 确认不依赖本机未跟踪 bridge、旧 install 或旧 build。

### 2026-08-15 P0 完成证据

- 首次审计揭示 `rmoss_gz_resources@main` 不存在，以及 `ats_mujoco_sim` 与
  `teleop_gimbal_keyboard` 的 HTTPS 传输不稳定。远端双重核验确认
  `rmoss_gz_resources@humble=b5c759f08844dfda19c79aa870866ace8d4c7b3a`，并确认两个用户仓库
  的 SSH URL 和目标分支可达，故 `dependencies.repos` 切换到该 branch/URL 组合。
- 完整编译又揭示 Gazebo `CMakeLists.txt` 直接引用被忽略、未跟踪的运动学测试源。该问题在用户
  Gazebo fork `a28ccd20428ffc4bdd7fbbc22fee884fa1db72eb` 中修复：测试源迁入受版本控制的
  `tests/`；未触碰用户 ignored bridge/header。
- 最终全新目录 `/tmp/ats_p0_repro_final.6xNq8i` 仅由根仓 archive 与远端 manifest 建立，执行
  `vcs import --recursive --shallow --skip-existing <root>`，耗时 `152.2 s`、`rc=0`，完整取得
  22 个仓库。`dependencies.lock.repos` 由 `vcs export --exact -n` 生成，记录全部实际 SHA。
- 同一干净目录执行 `colcon build --base-paths src --packages-up-to rmu_gazebo_simulator
  --parallel-workers 1`，17 包在 `11 min 13 s` 内通过；唯一 stderr 是上游 `rmoss_base` 既有
  `pipe()` 返回值 warning。`ctest --test-dir build/rmu_gazebo_simulator --output-on-failure` 为
  `4/4` 通过，`ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py --show-args` 通过。
- 为排除 archive 不能证明 root 远端传输的边界，又在独立目录
  `/tmp/ats_p0_remote_final.Lgyalq` 以 SSH 对 `origin/develop` 执行 depth-1 root clone，得到
  `f2d049cbf245b6fdfbad0d4870e53dc3ab09cbeb`；随后直接使用受版本控制的
  `dependencies.lock.repos` 执行 exact SHA import。全流程耗时 `250.2 s`、`rc=0`，22 个依赖均
  从远端 detached checkout，未使用本机 archive、build、install 或未跟踪文件。
- 该 P0 只证明远端复建、锁定与启动前资源解析；未启动 Gazebo，不构成 freshness、P2、安全或性能结论。

### DoD

- 新电脑仅凭远端仓库和 manifest 能取得 ATS 四舵轮模型、Mid360 和 xmacro；
- 所有活动仓库 HEAD 与记录一致；
- 构建不从旧 install 解析缺失包；
- Gazebo headless 能完成启动前资源解析。

### 风险转入条件

- 机器人描述 push 失败；
- manifest 指向不存在的 commit；
- 干净目录依赖未授权本机文件；
- 发现向 Gazebo `upstream` 写入的风险。

## 6. P1：定位 Gazebo localization freshness 根因

### 最短链路

```text
/clock + Gazebo real-time factor
-> /lidar_odometry
-> sensor_scan_generation /odometry
-> localization_fusion /localization
-> /localization/status
-> /rog_map_adapter/ready
```

### 插桩与指标

- [x] 单一 C++ recorder 同时订阅 `/clock`、`/cloud_registered`、`/lidar_odometry`、`/odometry`、
  `/localization` 和 `/localization/status`；
- [x] 每级记录 steady wall arrival、ROS stamp interval、`/clock` 相对 stamp age、重复/倒退 stamp、
  消息数和最大 gap；
- [x] 同一 recorder 记录其对 `/clock`、三段 odometry、`/localization/status` 和 adapter status 的
  callback 执行时长分布；它只量化观测器自身开销，不可替代行为 owner 的 executor/queue trace；
- [x] 记录 `/clock` wall interval、sim-time interval 与 RTF 分位数；
- [x] 以 `map -> gimbal_yaw_odom` 的实际零超时查询记录 TF lookup attempt/success/failure/max duration；
- [x] runner 只对本 launch session 内的 bridge、Point-LIO、loam、sensor generation、fusion、ROGMap
  与 adapter 写入 CPU tick、RSS、线程和 voluntary/nonvoluntary context-switch 两次原始快照；
- [x] DDS queue/drop 无可移植 RMW counter 时显式写入
  `unverified_no_portable_rmw_counter`，建议避免当作零丢包；
- [x] 可选 Gazebo Transport observer 实际记录 source cadence；domain `219/220` 分别为
  `599/600` 个样本且 wall p99 `0.108672/0.105195 s`；
- [x] 原始 ROS PointCloud2 reception publication-sequence 字段在当前 Fast DDS 实际探测为
  `supported=no`；不以零值推断零丢包；
- [x] runner 按 `/clock -> /lidar_odometry -> /odometry -> /localization -> status` 顺序输出
  `p1_first_freshness_violation`；分类器只消费单行 recorder witness，不修改运行时 timeout；
- [x] 审计 generic `ros_gz_bridge` 的实际部署 owner：活动 `parameter_bridge` 来自系统安装包
  `ros-humble-ros-gz-bridge 0.244.25-1jammy.20260608.160002`，workspace 没有该包源码；项目只拥有
  topic/YAML 与 ROS publisher QoS 配置面。对应 upstream `0.244.25` 的 GZ-to-ROS 回调同步完成
  `PointCloudPacked -> PointCloud2 -> publish()`，且 `create_gz_subscriber()` 未使用传入的
  `subscriber_queue_size`；`rmoss_gz_bridge` 只构建 pose/RFID bridge，不是该 LiDAR owner；
- [ ] 以 generic bridge 内部发布计数或本机 Fast DDS Statistics 区分 publisher 未发布与 DDS
  subscriber 丢样；当前 RMW sequence 字段不足以完成此归因；
- [ ] A/B 每次只改变一个因素：headless、recorder、RViz、相机、LiDAR profile、日志；
- [ ] 所有 profile 使用新 domain、相同 revision、相同起点和固定窗口。

### 2026-08-15 至 2026-08-19 P1 插桩、运行前审计与诊断

- **已验证（组件）**：Gazebo fork 的 `EvidenceStatistics` 确定性 CTest、`rmu_gazebo_simulator`
  单 worker Release build、完整包级 CTest（`32 tests, 0 errors, 0 failures`）、recorder callback
  duration 统计、runner `bash -n`、`ats_gazebo_nav.launch.py --show-args` 与相关 diff check 已通过。
  sandbox 下完整包 CTest 的 `ament_black` 会因 Python `SyncManager` 无法创建本地 socket 失败；这不是
  本轮 C++ 统计测试失败，仍需要主机环境复跑。
- **已验证（runner/回归）**：`scripts/gazebo_freshness_classifier.sh` 按固定阈值 p99 `<0.25 s`、最大
  wall gap `<=0.5 s`，在 `/clock -> /lidar_odometry -> /odometry -> /localization -> status` 顺序中输出
  首个可见违反者；`scripts/test_gazebo_freshness_classifier.sh` 覆盖上游首违、健康与缺字段输入。
  `scripts/test_gazebo_runner_contract.sh` 锁定 runner 仅保留合法 ROS domain 与残留导航/仿真进程的启动前
  审计。`p1_admission_evidence=true` 仅要求无故障注入、至少 60 s observer、无
  freshness 首违、status 全部 TRACKING、TF 无失败和 straight action 成功；分类器不改变安全 timeout 或行为参数。
- **已验证（domain 合法性）**：domain `233` 在当前 Fast DDS portBase 下报 `Calculated port number is too high`，
  多个 ROS 节点立即退出且 `/clock` 不推进。因此后续运行仅使用 `0..232` 的新 domain。
- **已验证（历史诊断运行，2026-08-19）**：合法 domain `220`、headless、`RUN_DURATION_SEC=30` 已启动
  Gazebo、传感器、localization、ROGMap/adapter、MINCO 与 iLQR MPC。recorder 记录 `/clock` wall
  p99/max=`0.122538/0.135193 s`，`/lidar_odometry`=`1.671385/1.671385 s`，`/odometry`=
  `1.659046/1.659046 s`，`/localization`=`1.663433/1.663433 s`；RTF p50/p95/p99=
  `0.199802/0.409690/0.596497`，status `TRACKING/non-TRACKING=178/119`，TF lookup `18/300` 失败。
  action 在 30 s 内未成功，但 `/cmd_vel_mpc` 曾有非零导航速度输出。
  分类器输出首个可见违反者为 `/lidar_odometry`。
- **推断 [Confidence: Medium]**：`/lidar_odometry` 是该窗口中最早违反 wall cadence 的可观测边界；下游
  `/odometry` 与 `/localization` 具有同量级 gap，recorder callback p99 为微秒级，因此现有证据不支持将
  首因归给 recorder 或 `localization_fusion` callback 阻塞。仍无法在 Gazebo 传感器负载/RTF、
  `loam_interface` publisher cadence 与 DDS subscriber 丢包之间唯一归因；DDS queue/drop 为
  `unverified_no_portable_rmw_counter`。
- **已验证（P1 60 s baseline，2026-08-19）**：新合法 domain `224`、headless、`P2_FAULT_CASE=none`、
  `planning_grid_owner=rog_map` 的 runner 返回 `0`。recorder 实际 `completed=yes`、`duration_s=60.003753`，
  修复了短 action 截断观察的旧缺口：`p1_admission_evidence` 现在还要求 recorder 正常完成且实际 duration
  不短于请求窗口。action 成功，最终误差 `0.146 m`；JPS/reference/predicted/executed 点数为
  `3/32/31/10`，`/cmd_vel_mpc` 曾有非零导航速度输出。运行期
  `/rc_esdf/planning_grid` 与 `/cmd_vel_mpc` 均为单一 publisher，terminal `emergency_stop=true`，
  `/cmd_vel_mpc` 采样为零。无残留进程，启动前审计通过。
- **未通过（P1 freshness）**：该实际 60 s 窗口中 `/lidar_odometry` p99/max wall interval 为
  `1.144617/1.488598 s`，下游 `/odometry`=`1.143652/1.490561 s`、`/localization`=
  `1.142361/1.490312 s`，故首违仍是 `/lidar_odometry`；RTF p50/p95/p99=
  `0.331409/0.502033/0.560295`，status `TRACKING/non-TRACKING=535/56`，TF lookup failure=`16/600`。
  `p1_admission_evidence=false`，原因为 `freshness_lidar_odometry`。该结果不能由单次 action 成功或
  终态 `/cmd_vel_mpc=0` 升级为当前 revision 的导航准入。
- **推断 [Confidence: Medium]**：`loam_interface` 仅在 `cloud_registered` callback 中发布
  `/lidar_odometry`；其 ROS stamp p99=`0.299990 s` 而 wall p99=`1.144617 s`，同时 `/clock` RTF
  p50=`0.331409`。证据优先支持检查 Gazebo 传感器/RTF、Point-LIO publisher cadence 和 DDS 接收边界，
  不支持把首因唯一归给 loam callback。下一实验建议以新 domain 只改变一个因素并补足 upstream publisher
  与 subscriber/drop 的独立计数。
- **已验证（最终 revision P1 60 s baseline，2026-08-19）**：显式 `ENABLE_CAMERA_SENSORS=false` 的新
  domain `225` recorder 实际 `completed=yes`、`duration_s=60.010472`，启动前无残留进程，planning grid、
  `/cmd_vel_mpc` 为单一 active publisher 且曾有非零导航速度输出；terminal `emergency_stop=true`，
  `/cmd_vel_mpc` 采样为零。此 final revision action
  被接受但在 `90 s` 内未给出终态，runner 以 `nominal action did not succeed` 返回失败。action 不成功不被
  隐藏为环境条件，也不影响完整 60 s recorder 的有效性。
- **未通过（最终 P1 freshness）**：domain `225` `/lidar_odometry` p99/max wall interval=
  `2.759540/2.942285 s`，下游 `/odometry`=`2.764472/2.946450 s`、`/localization`=
  `2.764479/2.944695 s`；RTF p50/p95/p99=`0.303726/0.803858/1.017098`，status
  `TRACKING/non-TRACKING=419/158`，TF lookup failure=`27/600`。首违仍为 `/lidar_odometry`，
  `p1_admission_evidence=false`。domain `224` action 成功与 domain `225` action 超时共同表明当前 P1
  不具有可重复的 action 成功证据。
- **未验证**：adapter ready=false 计数与持续 lease、publisher/subscriber/DDS 分层、重复 action 的统计
  稳定性、camera true/false 等单因素 A/B、P2 red-box 和实机导航。P1 建议避免因单次 domain `224` 成功 action
  标记通过。
- **已验证（raw Gazebo LiDAR 分层，2026-08-20）**：recorder 已在独立 `60 s` domain 实际订阅
  `/<robot>/livox/lidar`、`/livox/lidar`、`/cloud_registered` 与 `/lidar_odometry`。domain `215` 的
  `4 ms` physics candidate 在默认 `10 Hz / 625 x 32`、off-screen rendering 下得到 raw/lidar-odometry/
  localization wall p99=`1.610141/1.435872/1.430092 s`，action 因 `trajectory footprint is unsafe`
  ABORTED；该 world 不能替代 P4 的默认物理精度，也不能成为 P1 默认。raw 与 bridge 的 callback p99
  分别仅 `8/16 us`，wall cadence 同阶，不能通过下游 Point-LIO、loam、MINCO、MPC 或 timeout 调整修复。
- **未通过（LiDAR timing candidate，2026-08-20）**：runner 新增显式 `LIVOX_UPDATE_RATE_HZ`，使 SDF
  `update_rate`、bridge `scan_period_sec` 与 Point-LIO `mapping.lidar_time_inte` 同时由同一频率推导。
  domain `214` 的 `5 Hz / 625 x 32` 使用 `0.2 s` 三处一致周期，但 raw/lidar-odometry/localization wall
  p99 恶化为 `3.666797/3.312480/3.308619 s`，action ABORTED；`5 Hz` 建议避免成为默认。
- **未通过（headless world candidate，2026-08-20）**：`rmuc_2025_navigation_headless_world.sdf` 仅移除
  GUI state 的 `SceneBroadcaster`，保留默认物理步长、Physics、Sensors、IMU、用户命令和场景几何。
  短时 server 可推进 `/clock`，但 domain `213` 的 `10 Hz / 625 x 32` raw/lidar-odometry/localization wall
  p99=`1.409273/1.322408/1.319348 s`，status `TRACKING/non-TRACKING=448/125`、TF failure=`35/601`，
  action 在位移 `0.7409 m` 后仍以 unsafe trajectory ABORTED。该 world 仅保留为可复现的 rejected A/B，
  不是活跃导航或 P1/P2 通过证据。
- **已修复（runner world override）**：空 `WORLD_SDF_PATH` 不再生成无效的 `world_sdf_path:=` 参数；只有
  非空候选 SDF 路径才转发至 launch，默认 world-name 解析保持不变。`LIVOX_UPDATE_RATE_HZ=10.0` 与
  `LIVOX_HORIZONTAL_SAMPLES=625` 仍是默认配置。
- **下一定位边界 [Confidence: Medium]**：三个候选均未使 raw LiDAR cadence 达到 P1，且 raw、bridge、
  Point-LIO output 和 `/lidar_odometry` 的 wall gaps 仍同阶。下一项应区分 Gazebo sensor publisher 变慢与
  DDS subscriber 接收缺口；建议避免重复降低频率、改变 physics step 或移除 SceneBroadcaster 来宣称活跃导航。
- **未通过（默认配置复核，2026-08-20 domain 208）**：在关闭 Transport observer、关闭 Direct bridge、
  generic `RELIABLE/KeepLast(10)`、`10 Hz / 625 x 32`、关闭相机、headless off-screen rendering 与
  `planning_grid_owner=rog_map` 下，recorder 完整覆盖 `60.016403 s`。`/clock` wall p99/max 为
  `0.078741/0.351209 s`，但 `/<robot>/livox/lidar`、`/livox/lidar`、`/cloud_registered`、
  `/lidar_odometry`、`/localization` 的 wall p99 分别为
  `2.954985/2.958803/3.327495/3.327198/3.324217 s`；首违仍是 `lidar_odometry`，status
  `TRACKING/non-TRACKING=362/210`、TF failure=`35/600`、`p1_admission_evidence=false`。该 result
  排除了“仅 Transport 诊断 observer 导致默认配置失效”的简单解释，但单次运行仍不能唯一归因 sensor、
  generic bridge 或 DDS。
- **已验证（domain 208 安全行为）**：JPS/reference/MPC/轮关节和三段命令均曾非零，运行期 planning grid、
  `/cmd_vel_mpc` 仍有唯一 active publisher；动作最终 `ABORTED`、终态 `emergency_stop=true` 且
  `/cmd_vel_mpc` 为零。连续 swept 与 MINCO 离散 footprint 冲突采样仍为 `未验证`，建议避免由此次
  fail-closed 推导。
- **未通过（最新默认 P1，2026-08-23 domain `228`）**：在默认 `10 Hz / 625`、headless、
  `planning_grid_owner=rog_map`、`OBSERVE_GAZEBO_TRANSPORT_LIDAR=false`、
  `USE_DIRECT_GAZEBO_LIDAR_BRIDGE=false` 与 generic `RELIABLE/KeepLast(10)` 下，recorder 实际完成
  `60.001345 s`。`/clock` wall p99=`0.010387 s`、RTF p99=`1.032094`，但 raw Gazebo LiDAR、
  `/lidar_odometry`、`/localization` 的 wall p99/max 分别为 `0.708598/0.714906 s`、
  `0.650163/0.743549 s`、`0.650164/0.743591 s`。首违仍是 `lidar_odometry`，status
  `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`，`p1_admission_evidence=false`。JPS/MINCO/MPC
  path 为空且 selected 没有非零样本，是定位/地图 fail-closed；不支持将根因唯一归为 Gazebo sensor、
  generic bridge、DDS 或 Point-LIO。下一步仅审计 generic bridge 的可修改 owner，或增加不改变默认链的
  publisher/DDS 分层计数。
- **已验证（P1 owner 审计，2026-08-24；部署 + upstream 源码）**：domain `228` 的第一个已测 ROS
  边界 `/<robot>/livox/lidar` 由 `/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge` 发布；其
  转换回调、Gazebo Transport 接收线程和内部发布计数均不在四个项目仓库的可修改源码内。
  `ros_gz_bridge.yaml`/launch 可配置 topic、方向和 ROS publisher `RELIABLE/KeepLast(10)`，但不能在
  当前 workspace 内修改或观测 generic bridge 的实际 GZ 接收回调；`rmoss_gz_bridge` 也不拥有这条映射。
  因此触发“owner 不在项目可修改范围”的风险转入条件，本轮没有修改源码、没有新增计数，也没有占用新
  `ROS_DOMAIN_ID`。最新运行证据仍仅为上述 domain `228` 原始 artifact，P1 DoD 仍未通过；在显式纳入并
  授权维护 `ros_gz_bridge` 对应源码，或批准项目外 trace 方案前，建议避免继续用下游 timeout/QoS/Point-LIO
  改动替代该边界诊断。

### 修复原则

- 只修改最早违反 freshness 的行为 owner；
- 不提高 `odom_timeout_s`、localization timeout、adapter lease 或 map timeout；
- 不把 ground truth 接入正式 `/localization`；
- 不用 ros2cli 高频 observer 干扰被测链，优先单一 C++ recorder；
- 降低传感器负载优先结合感知质量与闭环指标共同评估。

### DoD

- [ ] 低负载 headless 连续至少 `60 s`，`/localization` p99 interval `< 0.25 s`；
- [ ] 同一窗口不存在 `> 0.5 s` 的 localization gap；
- [ ] status 持续 TRACKING，adapter 不因 localization 反复 `ready=false`；
- [ ] stamp 不倒退，sensor-to-localization age 有 p50/p95/p99；
- [ ] straight action 两次成功，终点误差、owner 和收尾零速均通过；
- [ ] 修复有聚焦单测或 deterministic fault test。

### 首违分类设计

分类器只处理 recorder 的结构化单行结果，契约顺序固定为：

```text
/clock(max wall gap <= 0.5 s)
-> /lidar_odometry(p99 < 0.25 s, max gap <= 0.5 s)
-> /odometry(p99 < 0.25 s, max gap <= 0.5 s)
-> /localization(p99 < 0.25 s, max gap <= 0.5 s)
-> /localization/status(p99 < 0.25 s, max gap <= 0.5 s)
```

缺字段输出 `first_violation=unverified` 并阻止 admission 结论；首违输出只作为定位证据，不能
替代 publisher/DDS/RTF 独立实验。正式 P1 仍要求新 domain、固定 revision、60 s headless 和两次
straight action。

### 风险转入条件

- 残留导航/仿真进程、非法 ROS domain、Gazebo z 发散、RTF 异常、TF 冲突或多个 localization publisher；
- unknown/lease/emergency stop 异常、关键 telemetry 缺失或系统失稳；
- 需要放宽安全 timeout 才能通过；
- 当前机器的非侵入式计数/trace 仍无法区分上游发布慢和下游丢包；转移到性能更高机器后建议先用全新
  ROS domain 重跑默认 `60 s` 基线，再比较同 revision 的独立边界证据。

## 7. P2：补齐 MINCO 生产契约

P1 Gazebo freshness 是运行准入门，不再阻塞不依赖 Gazebo 的算法实现、组件测试和 MuJoCo 验证。
在 P1 通过前可以完成本节的接口、数学与 fail-closed 行为，但建议避免把组件结果写成 Gazebo/P2 闭环通过，
也建议避免用算法改动掩盖 generic bridge 的 freshness 失败。

### 7.1 当前运动状态接入

- **已验证缺口（2026-08-24）**：`InitialKinematicState`、首端裁剪和初速度参与时间分配已经存在于
  `MincoTrajectoryOptimizer`，但 production node 的 center、footprint、JPS fallback、repair 四次
  `optimize()` 调用仍都传 `nullptr`；node 本身也没有与 localization epoch 原子绑定的运动状态。
- **接口决策**：由已经订阅 `/localization` 并拥有 localization epoch 的 `ats_goal_manager`，在同一互斥区内
  将规划系线速度、观测 stamp、epoch、goal/request 和 map publication sequence 一起冻结到
  `PlannerGoal`。MINCO 不再另建一个无法与 epoch 原子绑定的 odometry cache。
- **参考证据边界**：`参考/navi_minco_bit` 只证明“实时起点状态、上一轨迹剩余段、曲率/制动时间分配、
  独立 yaw、最终轨迹复核”是可行机制。建议避免复制其 Nav2 plugin/FSM、communication、双雷达或协议；其
  `determinePlanningState()` 还存在日志声称 COLD_START、代码却返回 HOT_START 的反例，不能照搬状态机。

- [ ] `MincoPlannerNode` 获取与 goal/snapshot 同一 localization epoch 的新鲜状态；
- [ ] 明确 twist frame、单位和时间，不假定速度已经是世界系；
- [ ] 将车体系速度正确旋转到规划世界系；
- [ ] 无可靠加速度时只播种速度，加速度保持零；
- [ ] stale、epoch 不匹配、非 finite 或 TF 失败时不用该状态；
- [ ] center、footprint、fallback、repair 使用同一个冻结初始状态；
- [ ] telemetry 记录原值、裁剪值、stamp age 和拒绝原因。
- [ ] production 请求缺少新鲜状态时保持急停并返回结构化失败；legacy 直连目标若保留零初值，建议明确
  标为兼容路径且不能作为 P2 准入证据；
- [ ] center、footprint、fallback、repair 分别记录候选类别和同一个冻结状态 identity，不能只记录最终
  `selected_trace` 后丢失被拒候选证据。

测试至少覆盖：

- [ ] 非零 yaw 下横移速度转换；
- [ ] 速度/加速度 finite 与上限裁剪；
- [ ] localization epoch 变化拒绝旧状态；
- [ ] 四条 optimizer 调用路径不再直接传 `nullptr`；
- [ ] 重规划首端速度连续，终端速度/加速度仍为零。

### 7.2 把质量 telemetry 升级为生产门禁

质量门禁建议区分直线与一般曲线，不能用起终点直线偏差拒绝合法 S 弯。

- [ ] 直线类限制 length ratio、横向偏差、曲率峰值/TV 和符号变化；
- [ ] 一般曲线相对 preprocessed guide/baseline 比较长度、偏差、曲率 TV 和净空；
- [ ] 所有类检查 v/a/j、时间单调、footprint/swept collision 和 snapshot freshness；
- [ ] ESDF candidate 同时满足净空不下降、碰撞不增加、长度和曲率变化不过门；
- [ ] 阈值进入唯一实际加载配置，并有参数范围校验；
- [ ] 记录结构化首个拒绝原因；
- [ ] quality 失败只回退到同 snapshot 上安全的 baseline；
- [ ] baseline 也失败时不发布 reference，保持急停、`/cmd_vel/selected=0` 与最终执行端为零。

### 7.3 净空与连续性

- [ ] center clearance 与 oriented-footprint clearance 分开记录；
- [ ] ESDF backtracking 使用同一 immutable snapshot；
- [ ] unknown、outside、非 finite gradient 和 snapshot 变化立即拒绝；
- [ ] noisy gradient 不产生交替法向偏移；
- [ ] 单拐角不会让无关直线段一起减速；
- [ ] 非零 initial-state 时重新检查首段连续曲线净空和动态极值；
- [ ] repair 输出重新求解 MINCO、yaw、时间和完整安全门。

### 7.4 参考对照后的计算量与时延优化

- [ ] 修复 `MincoTimeAllocator` 把非零首端速度静默压到 `reference_speed` 以下的问题；首点速度建议等于
  裁剪后的实际边界速度。若剩余路径在 `max_acceleration` 下无法降到终端零速，应在时间分配阶段结构化
  拒绝，不能先用不一致速度求解再消耗多轮 dynamic scaling；
- [ ] 将 `PathGeometryPreprocessor::preprocess()` 从每个候选重复执行改为每个 `planGoal()` 只生成一次
  immutable prepared seed，center/footprint/fallback/repair 共用同一 seed 与 map snapshot；
- [ ] 保留兼容 `optimize(raw_path, ...)` 包装，但 production 走 `prepare + optimizePrepared`，并用数值等价
  单测证明重构不改变路径、时间、yaw 或安全语义；
- [ ] 分候选记录 preprocessing、ESDF refinement、MINCO solve、safety recheck 和总 wall time 的
  p50/p95/max；只有测得 dominant stage 后才继续做上一轨迹热启动或缓存；
- [ ] 不照搬参考工程的 `1.5 x` severe dynamics 容忍、invalid duration 视为 safe、tracking error 仍返回
  HOT_START、旧轨迹无 generation 复用等行为；这些都比 ATS 当前 fail-closed 契约更弱；
- [ ] 若 prepared seed 后仍考虑热启动，先证明当前 closed-form MINCO S3/ESDF refinement 存在可复用的
  优化变量和至少 `20%` p95 wall-time 收益，再单独设计；建议避免只为对齐参考工程引入共享可变轨迹状态。

### DoD

- [ ] 库级、node 级和旧 reference 竞态测试通过；
- [ ] 直线、冗余共线、短首尾段、单角、S/U 弯、窄通道、noisy ESDF fixture 通过；
- [ ] 门禁能拒绝“finite 但无意义多弯”的候选；
- [ ] initial-state 在实际 node 路径生效；
- [ ] 不改变 JPS、MINCO S3、独立 yaw、地图和速度 owner。

## 8. P3：实现 Gazebo 确定性场景 runner

当前 `TEST_PROFILE` 只用于日志命名，建议升级为实际行为 owner。

| profile | 固定输入 | 核心判据 |
| --- | --- | --- |
| `straight` | 两点自由空间、零/非零 yaw | 不增弯、终点与停止 |
| `single_corner` | 一个必要拐角 | 拐角减速、无 overshoot |
| `s_turn` | 两次相反转弯 | 无多余摆动、曲率符号正确 |
| `narrow_corridor` | yaw-aware footprint 可通过窄通道 | clearance、无错误 shortcut |
| `nominal` | 固定任务路线 | 端到端成功和 owner |
| `red_box` | 固定多段红框 | 每段 action 生命周期和恢复 |

### 工作项

- [ ] `case "$TEST_PROFILE"` 拒绝未知 profile；
- [ ] 每个 profile 固定 world、起点、目标序列、yaw、timeout 和路径类别；
- [ ] action 根据 `GOAL_YAW` 生成规范化 quaternion；
- [ ] 保存实际目标 payload 和 scenario manifest；
- [ ] 场景建议避免依赖人工 RViz 点击；
- [ ] 每个 goal 独立记录 accepted/result/cancel/preempt/timeout；
- [ ] 保存 raw/preprocessed/refined/reference/predicted/executed；
- [ ] 保存地图 identity、owner、clearance、碰撞采样、v/a/j 和 terminal error；
- [ ] 未验证 clearance 或碰撞采样不写默认通过值；
- [ ] shell/Python/C++ 测试锁定 profile、yaw 和未知 profile 拒绝。

### DoD

- 每个 profile 至少两个新 ROS domain；
- 结果与固定 manifest 可配对；
- profile 名称确实改变输入和验收逻辑；
- 失败保留首因和 artifact，不污染后续场景。

## 9. P4：完成 P2 仿真验收

### 名义与边界场景

- [ ] `straight`、`single_corner`、`s_turn`、`narrow_corridor`、`nominal`、`red_box`
  各两个独立 domain。

每次建议记录：

- terminal pose、位置/yaw 误差、总耗时；
- JPS/MINCO/MPC/executed 点数和五层 payload；
- length ratio、横向偏差、曲率 max/p95/TV/符号变化；
- v/a/j peak/p95、segment duration、time-scaling 次数；
- center/footprint minimum clearance；
- discrete/swept collision samples；
- replan、fallback、repair、失败和恢复次数；
- localization/map/reference/command age；
- planning grid、`/cmd_vel/autonomy_raw` 与 `/cmd_vel/selected` 唯一 owner；

### 当前 revision 故障矩阵

每例使用独立 domain 和全新 launch：

- [ ] all-unknown、map-unready、map-stale、input stale、unreachable；
- [ ] adapter lease、projection timeout、emergency-stop recovery；
- [ ] localization jump/epoch、TF loss、runtime unsafe、process restart/late response。

共同 DoD：

```text
failure detected within configured deadline
-> ready=false or planner failure
-> emergency_stop=true
-> /cmd_vel/selected=0
-> final actuator input=0
-> old reference cannot revive
```

恢复建议满足 generation/sequence 继续推进，且只有新目标或新 request identity 才恢复运动。

只有唯一 owner、immutable snapshot、fail-stop、名义/红框、故障矩阵、clearance 与碰撞证据全部完成，
才能标记 P2。单次直线成功或旧 revision fault 不能替代。

## 10. P5：ROGMap/RViz 运行验收

- [ ] 静止时保存全局融合 RC-ESDF、局部 RGB voxel 和三色 bounds 同帧截图；
- [ ] 机器人直线移动 `3 m`，记录 visualization/local/update bounds center；
- [ ] 跨 `1.0 m` sliding threshold 时 local-map center 正确更新；
- [ ] visualization center 相对机器人误差不超过一个 ROGMap cell；
- [ ] `Decay Time=0` 下旧局部点不残留；
- [ ] 全局 grid origin/尺寸不随机器人漂移；
- [ ] A/B/C：headless、global-only、global+local，各至少两次；
- [ ] 比较 projection、adapter、map-lock、debug p50/p95/p99；
- [ ] RViz 退化超过 `20%` 时停止定位，不放宽 deadline；
- [ ] 实车 RViz 配置做 YAML/QoS/fixed-frame 静态验证。

## 11. P6：P3 Nav2-free 运行准入

- [ ] `launch_nav2:=false`；
- [ ] graph 中无 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`；
- [ ] MINCO 不订阅 `/plan`；
- [ ] 目标入口只使用 ATS action 或受控 `/goal_pose`；
- [ ] feedback、success、abort、cancel、preempt、timeout 全部运行验证；
- [ ] cancel/preempt 后连续零速度；
- [ ] timeout 后旧 planner result 不能重新授权；
- [ ] 扩大矩形和 red-box 均由 ATS action 完成；
- [ ] Goal Manager restart、late joiner、map/localization wait 有确定性结果；
- [ ] action result、emergency stop 和 ExecutionCommand identity 可审计。

在上述完成前仅写“Nav2-free 代码路径存在”，不能写“P3 已通过”。

## 12. P7：P4 仿真算法安全

### 仿真阶段

- [ ] 将自适应 sampled sweep 升级为具有明确误差上界的连续 swept 契约；
- [ ] 覆盖纯旋转、横移、对角、`+pi/-pi`、高曲率和 map 边界；
- [ ] 验证 planner collision、footprint gate、Local Collision Repair 与 unsafe trajectory
  都不会提交不安全 reference；
- [ ] 注入 map stale、unknown、localization stale、无路、目标取消和 solver failure，验证
  `emergency_stop=true -> /cmd_vel/selected=0 -> final actuator input=0`；
- [ ] 验证 map snapshot、generation、reference timestamp 和 goal identity 不会让旧轨迹复活；
- [ ] 长时间运行无 queue/RSS/thread/generation 异常增长。

### 下位机边界

CAN、电机、轮速、电流、电压、温度、底盘反馈、硬件 watchdog、接触和制动诊断由下位机/HIL
链独立维护。本导航仓只保证向 `/cmd_vel` 下位机速度接口提交经过云台 yaw 变换的速度，不订阅、
不记录、也不以这些硬件信号决定导航 action 成功或失败。

## 13. QP：Shadow 到受控主链

### QP-2 真实 Shadow

- [ ] 接入与 iLQR 同一 snapshot 的真实 map-health 和 footprint/collision producer；
- [ ] 仅在真实输入成立后移除临时 `map_fresh=false`、`collision_free=false`；
- [ ] freshness/P2 通过后运行 paired A/B/C；
- [ ] identity digest 不可比时 analyzer 输出 `not_comparable`；
- [ ] 定位 OSQP `max_iterations` 的矩阵尺度、conditioning、active bounds 和 warm-start；
- [ ] 建议避免提高 iteration、放宽 residual/time limit 或接受 `solved_inaccurate`；
- [ ] 得到稳定 `solved`、residual、hard margin、slack 和 warm-start 分布；
- [ ] 记录 complete phase、full callback、CPU/allocation p50/p95/p99；
- [ ] 覆盖 nominal、yaw jump、速度阶跃、反向、横移和 navigation fault matrix。

### QP-3 受控主链

- [ ] candidate 全部 hard-check 通过时才能发布 `controls.front()`；
- [ ] infeasible、deadline、residual、slack、map、collision、lease 失败均零速度；
- [ ] iLQR 保留为可选 baseline；
- [ ] fallback 建议限定窗口并重新验证旧序列；
- [ ] 建议避免直接沿用 `last_control`；
- [ ] 节点级覆盖 solved/infeasible/time-limit/solved-inaccurate/residual reject；
- [ ] MuJoCo navigation matrix 通过后，QP 主链候选再进入导航评审；
- [ ] 默认切换需要单独评审和回滚点。

## 14. 长时间稳定性与性能

- [ ] 固定 `Release/-O3`、线程、CPU governor、middleware 和 power mode；
- [ ] 关键 profile 预热后至少运行 `10 min`，另做 `30--60 min` soak；
- [ ] 记录 RTF、CPU/RSS、threads、context switches、DDS drops、queue depth；
- [ ] 记录 localization、projection、adapter、MINCO、iLQR/QP、callback p50/p95/p99/max；
- [ ] 记录 control jitter、deadline miss、command age 和 tracking error；
- [ ] 检查 generation、sequence、goal ID 和 telemetry ring 是否倒退或增长；
- [ ] RViz、INFO logging、recorder 分别做 A/B/C 消融；
- [ ] 性能结论附测试机、revision、配置、样本数和 artifact；
- [ ] 未在目标机测量前建议避免引用 `50 Hz`、`6 ms` 或报告内存数值。

## 15. 导航侧 Gate 0--2

### Gate 0：静态与数学

- [ ] frame/time/QoS/map/ESDF/generation/ownership 账本完整；
- [ ] 状态、矩阵、参数和 finite 检查通过；
- [ ] 速度、加速度和 jerk 约束来自当前导航模型与固定配置，并保留可审计单位；
- [ ] 启动、部分初始化、reset 和 shutdown 都输出安全状态。

### Gate 1：离线/回放

- [ ] nominal、边界、stale、乱序、clock jump、NaN、overload 回放；
- [ ] 与冻结 iLQR/JPS-MINCO baseline 成对比较；
- [ ] 关键故障在 deadline 内零速度。

### Gate 2：仿真

- [ ] P2/P3/P4 仿真条目通过；
- [ ] Gazebo 与 MuJoCo 结论一致或差异已解释；
- [ ] clearance、tracking、deadline 和 recovery 有导航侧 artifact。

### 受限低速实机导航观察

- [ ] 独立物理急停与安全观察员就位；
- [ ] 先直线停止，再横移、原地旋转、单拐角；
- [ ] 再进入窄通道、重规划和边界场景；
- [ ] 每次只提升一个能量或复杂度维度；
- [ ] 异常时优先安全停车，并保留冻结 revision 作为回滚点。

## 16. 通用验证命令

```bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select <changed_packages> --parallel-workers 1

colcon test --base-paths src --packages-select <changed_packages> \
  --event-handlers console_direct+ --parallel-workers 1

colcon test-result --test-result-base build/<package> --verbose
python3 -m py_compile <changed_python_files>
bash -n <changed_shell_files>
ros2 launch <package> <launch_file> --show-args
git diff --check
```

仿真建议使用新 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1`、`ROS2CLI_DAEMON=false`、headless
性能基线、`planning_grid_owner=rog_map` 和 `solver_mode=ilqr`（QP-3 前）。

## 17. Artifact 最小字段

- 仓库 SHA、dirty state、effective params、world/map、domain、时间；
- 目标序列/yaw、action feedback/result；
- topic type/frame/QoS 和 ownership；
- localization/map/reference/command age；
- source generation、adapter publication、MINCO snapshot identity；
- raw/preprocessed/refined/reference/predicted/executed payload；
- clearance、碰撞采样、v/a/j、tracking 和 terminal error；
- CPU/RSS/RTF、p50/p95/p99/max 和 deadline misses；
- first failure、stop/recovery、旧 reference 拒绝和最终零速度；
- 未执行项和不能得出的结论。

## 18. 提交与文档规则

- 每轮开始与结束检查根、导航、Gazebo、MuJoCo 和机器人描述仓；
- 建议避免 `git add .`、`git add -A`、force push、历史重写和破坏性恢复；
- 只显式 stage 本轮文件；
- 根、导航、MuJoCo 分别提交自己的 `develop`；
- Gazebo 只推送用户 `origin/main`，建议避免向 `upstream` 写入；
- 无修改仓库建议避免制造空提交；
- 作者固定为 `liukong1220 <1625038134@qq.com>`；
- 本清单只更新状态，不再次追加完整运行流水账；
- 原始结果放 artifact，准入结论放状态文档，QP 证据放 backend admission。

## 19. 下一阶段执行顺序

1. P1 基础设施线：形成维护 `ros_gz_bridge 0.244.25` 源码或项目外 trace 的明确边界后，再继续
   generic bridge/Gazebo freshness 修复；下游 timeout 与 Point-LIO 保持现状，不作为绕过手段；
2. P2 算法线：现在即可完成 `PlannerGoal` 原子运动状态、MINCO 四候选初值连续性、制动可行时间分配、
   prepared seed 复用、候选分类 telemetry、组件测试和 MuJoCo 非零速度重规划；
3. P1 通过后，再用最终算法 revision 执行两个 Gazebo straight 和 P2 Gate 2，建议避免复用算法修改前 artifact；
4. 完成 P3 确定性 profile runner；
5. 再进入 P4 场景和故障矩阵。

P1 未通过时，Gazebo/P2 准入声明、MINCO 参数搜索、QP 主链和性能结论暂缓；带有确定性单测和
独立 MuJoCo 证据的算法源码实现仍可继续。下一算法会话使用
`docs/项目优化文档/下一阶段提示词_ATS_MINCO动力学连续重规划.md`。
