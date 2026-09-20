# 2026-09-20 导航 fail-closed 修复、边界审计与部署门禁

本文记录本轮实际证据，不把动作成功、topic 存在或编译通过等同于系统准入。既有 domain 220/221 记录保持原义；`docs/nav2_to_3desdf_minco_mpc_optimization_direction.md` 中的历史运行不是本轮修改后证据。

## 一、已验证

### 基线与修改所有者

初始三个仓库均在 `develop`，HEAD 分别为根仓 `b62ffecdc89e0449f5e2cd32621733ac9e679ca6`、导航仓 `9c7de7ce386e1ba26b5176670315cca025dcf045`、MuJoCo 仓 `6d67d72a7dfafde420479b98d096e0142df3ce91`。Gazebo 仓初始为 `main/e632f4d513680121437d76fd5b8ec65e75b5cffc`，前期只读，后经用户明确授权修改相关定位/导航所有者。两个参考工程和无关 HTML 保持只读。初始已有用户修改，未进行破坏性恢复。

|所有者|本轮行为修改|直接证据|
|---|---|---|
|`ats_cmd_vel_arbiter`|无效 mode、future/stale、replay、旧 incarnation 撤销自动租约并清旧速度；节点立即发布零，timer 与拒绝输出串行；防重放水位不回退|core 42 项、DDS 节点 5 项 GTest 通过|
|`ats_swerve_mpc`|旧 incarnation/replay 退出 trajectory mutex 后调用既有 fail-stop；旧 tracker 不继续执行|`test_mpc_localization_gate` 12 项通过|
|`small_gicp_relocalization`|异步 request/result 失效代次；initialpose/recovery 后旧结果不能修改 seed/发布 observation；fine 调度与结果提交复核 steady deadline；启动缺 TF 可响应关停|candidate core 24 项；cold-seed 中真实缺 TF/暂停 clock 的 SIGINT 进程测试通过|
|fusion / GICP status 输入|odom history 时间窗外另有 2000 条默认样本上限与丢弃计数；status frame/stamp/age/future/顺序/epoch gate，拒绝不续租|fusion core 10 项、scan core 3 项；fusion node/cold-seed CTest 通过|
|`terrain_analysis` / `terrain_analysis_ext`|只在 context 已关闭时把 `spin_some` 的 RCLError 视为正常退出，活跃 context 错误继续抛出|两个真实节点各 20 次 SIGINT 均退出 0|
|MuJoCo sim/static-map/twist bridge|处理 ExternalShutdownException，使用 context 的幂等 shutdown；传感器子进程有界回收，异常子进程仍失败|三节点 owner/group SIGINT 共 6 项 pytest 通过|
|MuJoCo runner|action、safety、recorder process、recorder evidence、teardown、overall 独立结果；默认 `NAV_TRACKING_GATE=1`；证据不完整不得 overall=0；重置后台 launch 的 SIGINT disposition，SIGINT 只发 launch leader|18 类 shutdown/evidence 回归通过，包含真实 spawn、单次 child SIGINT、analysis 1/2/not_run 拒绝|
|tracking recorder / analyzer|payload 按 digest 复用，publication 身份逐条保存；Q1 精确匹配 EXECUTE 与 `(epoch, publication_sequence)`，拒绝缺失/歧义/错误 frame/content；unknown 返回 2，不伪造成 collision 或 safe|recorder 离线测试、analyzer 冲突/缺证据/乱序回归通过；实际 SIGINT 两路径已通过，新增证据行为待新仿真复验|
|目标集与遥测|完整 yaw quaternion、独立 domain/manifest/结果、真实源所有者故障入口；恢复场景显式指定 real GICP|目标集 quaternion/dispatch 7 项通过；软件定位失效急停不等价于物理 E-stop|

构建命令均在 source Humble 和工作区后执行，`MAKEFLAGS=-j1`，均退出 0：

```bash
colcon build --base-paths src --packages-select ats_cmd_vel_arbiter ats_swerve_mpc --parallel-workers 1
colcon build --base-paths src --packages-select terrain_analysis terrain_analysis_ext ats_mujoco_sim --parallel-workers 1
colcon build --base-paths src --packages-select small_gicp_relocalization --parallel-workers 1
```

`python3 scripts/validate_navigation_config.py`、`bash scripts/test_gazebo_runner_contract.sh`、所改 shell 的 `bash -n` 和 Python `py_compile` 通过。指定 ROGMap 四组、MINCO 三组、Goal Manager epoch、fusion node、MPC gate 共 10 个 CTest 基线目标全部通过；修改后的授权与定位目标另行重跑通过。GTest XML 位于各 `build/<package>/test_results/<package>/`；不要把 CTest target 数写成内部 test case 数。构建有既有 CMake/overlay 警告，不是无警告构建。

### 后续几何、生命周期与真实恢复验证

- GICP：`registered_scan` 按 `odom` 验证，map-frame PCD 不再套用机械雷达外参，部署入口预检 prior PCD。定位包 6 个聚焦 CTest target、launch/PCD 契约 10 项通过；real/MuJoCo/Gazebo 当前 `--show-args` 均退出 0。
- MINCO：既有 solve 次数中为全局耦合重求解保留一次，不放宽动态门。0.9 m 直线、初速度 0.7 m/s 的重现由拒绝转为接受；峰值 jerk 从 12.8168 降至 10.8706 m/s³，限值仍为 12。对应聚焦 CTest 通过，不能由此推导所有场景通过。
- 导航 static-map：构造与 spin 同处正常 SIGINT 捕获范围；真实 `/map` 发布后 owner/group SIGINT 两次均退出 0（domain178/179）。
- 实际命令 `ROS_DOMAIN_ID=216 P4_FAULT_CASE=odometry_stale P4_RELOCALIZATION_MODE=real P4_RESULT_FILE=/tmp/ats_recovery_real_216.json P4_LAUNCH_LOG=/tmp/ats_recovery_real_216.log bash scripts/test_mujoco_localization_fault.sh` 退出 0，无合成 observation 辅助。GICP accepted sequence 11→14，故障时 LOST、odom silence 0.5661 s、软件停止延迟 0.4997 s；STOP command86 后以新 request2（原1）、map generation6/publication9 的 command118 恢复。action result=0，终点 `(1.035841,0.059779)`，误差 `0.035842 m`。这是实际 GICP、自研重规划和新授权闭环，不是物理 E-stop/实车/全矩阵证明。
- 恢复评估器 10 项测试通过。domain211 的发现阶段失败与 domain212 的误判产物均保留：后者要求保留字段 `planner_candidate_sequence` 递增，而生产者始终写零。修复改用实际 request/REFERENCE_READY/digest/map tuple，保留 STOP 后新时间戳要求；domain212 虽日志记录 action success（误差0.072 m），仍不改写原失败结果，以216重跑为通过证据。
- 构建/测试日志在 `log/fail_closed_audit_20260920/`：`minco_moving_retime.log`、`static_map_shutdown_final.log`、`reloc_frame_contract.log`、`launch_recovery_contract.log`、`gazebo_reloc_contract.log`；三个入口参数输出为 `*_launch_args_frame_contract.log`。
- fusion stale 契约复跑：`log/fail_closed_audit_20260920/fusion_stale_contract_repeat.log` 记录同一 `test_localization_fusion_node` CTest target 连续 10 次 Passed，不是 10 个不同 target；任务中提到的 `fusion_stale_repeat.log` 在本次读取时不存在，采用实际日志路径。recorder readiness 诊断确认无订阅、错误 frame、延迟 TF、未 ready snapshot 均不能产生首个可审计 tick；匹配 actual/TF/ready snapshot 后才出现 `RECORDER_READY`，启动后缺 TF 仍保留为不完整 tick。证据：`log/fail_closed_audit_20260920/recorder_ready_and_fusion_diagnostic.log`。
- recorder 最终 readiness/shutdown 日志 `log/fail_closed_audit_20260920/recorder_ready_shutdown_final.log` 记录 `Ran 5 tests ... OK` 和 `MuJoCo runner shutdown PASSED`；前置 `FAIL: mandatory recorder evidence ...` 是负向门禁用例的拒绝输出，不能与末尾套件结果混写。原 diagnostic 中 SIGINT 用例曾因 `RuntimeError: Unable to convert call argument to Python object` 退出 1，`Ran 4 tests ... FAILED (failures=1)` 仍保留，不用最终 5 项通过覆盖失败历史。这些是聚焦 readiness/shutdown 证据，不是新仿真完整 recorder evidence 通过。
- MuJoCo matrix100 的 straight 单场景现已完成：`/tmp/ats_goal_set_next_100/straight/runner_status.env` 独立记录 action=succeeded、navigation safety=passed、recorder process/evidence=passed、analysis=0、teardown=passed、launch wait=0、无升级、runner exit=0。`runner.log` 记录 action SUCCEEDED，终点 `(0.952763,0.048079)`、目标 `(1.0,0.06)`、误差 `0.048718 m`，MuJoCo contact delta=0/max force=0；recorder 首 tick ready，关停写出300 samples。原 `0.60×0.50 m + 0.02 m` footprint 的分析为 COMPLETE、Q1=yes/Q2=no/Q3=no、最小 clearance `0.4000 m`。部分 layer provenance 仍因 payload 截断不可用，analyzer 不独立验证物理接触；日志还有重名节点警告及一次 `/ats_goal_manager` CLI 查询失败，不能称为无告警运行或全量 graph 审计通过。该结果只证明本次 straight，不覆盖其余矩阵、其他 footprint、实车或 red-box；不改写旧172严格重放的 incomplete。
- 七状态与 DualMap 地图授权（本轮源码+聚焦单测，非全矩阵动作通过）：
  - `LocalizationStatus` 保留 0..4，新增 `BOOTSTRAP=5`、`CONFIRMED=6`；seed≠TRACKING；LOST 闩锁；有界 RECOVERING；`confirmation_timeout_s` 进入 bringup `node_params.yaml`。`test_localization_fusion_core`、`test_relocalization_candidate_core`、`test_relocalization_frame_contract`（含异步 stamp / TF 零等待）、`test_localization_fusion_node`、`test_localization_fusion_cold_seed`（含重复 pending 不得续开 recovery episode）共 5 个 CTest target 全部 Passed。日志：`log/fail_closed_audit_20260920/seven_state_tests3.log`。
  - MPC / arbiter / MINCO 要求已知非零 MINCO-local epoch/generation（来自 `PlannerStatus`）与新鲜 ready lease；未知/旧/未来 EXECUTE fail-closed；ACCEPTED 忽略；FAILED 撤销 AUTO；`MAP_UNREADY` 退役 generation；status/ready alone 不能恢复。ready lease 过期只退役当时租约绑定的 generation，恢复心跳不得把租约期外的更新 generation 永久抬高退役地板。`test_mpc_localization_gate`、`ats_cmd_vel_arbiter` 2 个 target（含 `ReadyLeaseExpiryAndLateHeartbeatRetireGeneration`）、`test_planning_map_snapshot` 全部 Passed。日志：`dual_map_*_tests3.log`、`dual_map_fix_rebuild4.log`。
  - 仍缺：端到端 source generation 与 grid/ESDF 同号、跨进程 content lineage 原子快照。上述单测通过不冒充 MuJoCo/Gazebo 动作矩阵通过。


### 本轮失败也属于证据

- Gazebo domain `223`：命令与 domain 220 相同，只开启 `OBSERVE_GAZEBO_TRANSPORT_LIDAR=true`；nominal action accepted=1、succeeded=0，terminal map pose `(4.473041,-4.202148)`，目标 `(1.17,-2.94)`，误差 `3.536 m`，最后急停和 selected 零。Transport LiDAR 904 帧、p99/max wall interval `0.264500/0.335093 s`；ROS LiDAR 445 帧、p99 `0.863108 s`；RTF p50=`0.563493`。P1 false。action log 最后为 `elapsed=58.711 / waiting_for_map`，没有 terminal result，不能称该运行已确认 Goal Manager timeout。contact 未验证。证据：`log/gazebo_minco_mpc_chain/20260920_122816_nominal_none_domain223/`。这是后续授权/生命周期修改前的诊断，不冒充最终 revision 验收。
- domain `233` 被 runner 的 domain preflight 拒绝，退出 3，未启动仿真。
- MuJoCo domain `199`：运行中的 shell 脚本被并发修改，出现错误命令和重复 preflight；已停止，退出 143，随后该 domain ROS graph 为空。该运行作废，不能计入导航或 teardown 验收。
- 冻结后的 MuJoCo domain `170`：action SUCCEEDED、物理终点 `(1.0539828,0.0360112)`，误差 `0.059073 m`，contact delta=0/max force=0；但 recorder ExternalShutdownException、launch TERM 升级，runner=1。独立状态为 action succeeded / safety passed / recorder failed / teardown failed。launch 的 13 个子节点实际均 clean finish，launch 本身继承忽略 SIGINT 导致需要 TERM；随后修复 signal disposition 和 recorder。证据：`/tmp/ats_goal_set_straight_170_final/straight/`。不能用该动作成功证明 runner 通过。
- MuJoCo domain `171`：action/safety/recorder 通过、误差 `0.071556 m`，但 group SIGINT 与 launch fan-out 重复传信号，使子进程退出 `-2`；runner=1。随后改为只向 launch leader 发 SIGINT。证据：`/tmp/ats_goal_set_straight_171_final/straight/`。
- MuJoCo domain `172`：action/safety/recorder/teardown 均通过，launch wait=0、无升级，误差 `0.056394 m`、contact delta=0；旧 runner 在 `NAV_TRACKING_GATE=0` 下忽略 analyzer=1，因此 runner=0 不等于完整证据通过。原始 verdict 保留。先前单独重放 `/tmp/nav_tracking_172_identity_bound_v2.json` 曾返回 exit=0、Q1=yes/Q2=no/Q3=no、最小 footprint clearance `0.4000 m`；这是历史分析结果，不是最新 strict final pass。最新严格重放 `/tmp/ats_straight172_q3_complete_verdict.json` 返回 incomplete/exit=2：Q1=yes、Q2=no、Q3=unknown，初始 1 tick 缺 `map←odom` TF 被排除，`unevaluated_actual_ticks=1`，不能把“已评估部分无冲突”升级为证据完整。原误报把 epoch2/pub58 的 reference 配给了后来的 epoch3/pub59 无效图；重放使用原 `0.60×0.50 m + 0.02 m` footprint，不能替代其他尺寸的验收。部分 layer payload 截断；TF 为同授权身份的采样值，不是提交点原子 TF；analyzer 不评估物理接触。证据：`/tmp/ats_goal_set_straight_172_final/straight/`，原运行和旧重放均不改写。
- Gazebo domain `213`：最终运行源码、direct bridge=false；runner=1、accepted=1/succeeded=0，终点 `(3.338529,-2.798606)`，误差 `2.1731 m`。Transport LiDAR p99/max `0.113711/0.148435 s`，ROS LiDAR p99/max `0.472034/1.143934 s`；RTF p50=`0.692174`，P1 false，最后急停 true/selected 全零，contact 未验证。证据：`log/gazebo_minco_mpc_chain/20260920_130407_nominal_none_domain213/`。
- Gazebo domain `214`：相同配置仅切换 direct bridge=true。发布端 BEST_EFFORT、registered-scan relay 请求 RELIABLE，双方日志明确报告 QoS 不兼容；`/registered_scan` 无消息，健康门未开、未派发 action，runner=1。不能用此运行比较导航性能。GT relay 两节点、导航 static-map 节点退出 1，Gazebo SIGTERM 升级退出 -15；后续只修复可写导航仓的 static-map 生命周期，Gazebo 源码保持只读。证据：`log/gazebo_minco_mpc_chain/20260920_130748_nominal_none_domain214/`。
- 上述213/214均为后续 frame/GICP 修改前结果，不是当前 revision 验收。用户后来授权 Gazebo 修改；已修正 relay QoS 配对、正常关停，并由 native plugin 发布真实 gimbal link GT，构建和聚焦测试通过，运行结论需新证据。
- 旧 MuJoCo matrix200–210 完整运行11场景，仅 occupied206通过，其余10项失败，原始 `/tmp/ats_goal_set_200_final_all/summary.json` 不改写。clearance204 第二腿的最新 probe 修复后重放为 `/tmp/ats_clearance204_q3_conflict_probe_verdict.json`：Q1=yes（8 条 committed reference 发布时无冲突）、Q2=yes（202 个已评估 tick 中 23 个实际足迹离散冲突）、Q3=yes，翻转标签为 `actual_pose_at_first_discrete_conflict`，即首个实际离散冲突处的同一位姿有 free→occupied 证据。不能归因为被拒绝的候选，也不能由 contact计数0掩盖此失败。此结果支持继续审计地图变化与旧 reference 撤销时序，但不证明单一根因；仍有 4 tick 缺 `map←odom` TF 被排除、部分 layer payload 截断，物理接触不由该 analyzer 验证，地图/位姿/TF 完整时间配对的限制保留。
- Gazebo215未启动仿真：freshness将整个package的CMake时间与未修改的recorder比较，误报stale。已改为按recorder/plugin各自产物、直接源码和构建配置检查，解析installed symlink，metadata独立检查生成Makefile；freshness/runner contract/shutdown三组回归通过。此检查不覆盖所有传递外部header/library依赖，不能替代构建。原始preflight在 `log/gazebo_minco_mpc_chain/20260920_142642_nominal_none_domain215/runtime_preflight.txt`。
- Gazebo domain `218`：`log/gazebo_minco_mpc_chain/20260920_150013_nominal_none_domain218/runner_status.env` 与 `summary.txt` 一致记录 `runtime_gate_status=failed`、`action_status=not_started`、accepted=0/succeeded=0、`runner_exit=1`；健康门 `localization_state=4`、map ready=false、source generation=0，未派发 action。急停 true、selected 全零仅是本次安全停机观测，不是 nominal action/safety 全场景通过；recorder 未启动，recorder evidence=unverified，P1 admission=false（not_evaluated）。关停单独通过：`teardown_status=passed`、`gazebo_stop_status=passed`、`launch_wait_status=0`、`teardown_escalation=none`，`gazebo_server_stop.log` 返回 `data: true`。native stop 成功不能覆盖健康门失败。
- 同一 Gazebo218 `launch.log` 实际出现 GICP `minimum information eigenvalue below threshold` 拒绝，即使 `converged=true`；multi-guess 记录例如 `source_points=1351/1354/1350`（约1350）。这是观测到的拒绝原因与点数，不是“点数少导致全部失败”的唯一根因证明，也不据此宣告待实施的性能修复。日志同时确认 `/registered_scan` relay 输出 `frame=odom`。
- MuJoCo 单轮 matrix100–103 已全部结束，`/tmp/ats_goal_set_next_100/summary.json` 记录 straight100 通过，lateral101/yaw102/nearby103 失败；四场 teardown 均 passed、launch wait=0、无升级。101–103 各自 `runner_status.env` 一致为 action=not_started、navigation safety=failed、recorder=not_started、recorder evidence=unverified、analysis=not_run、runner exit=1；不能写成三次动作执行失败，也不能以 clean teardown 覆盖准入失败。
  - lateral101：`/tmp/ats_goal_set_next_100/lateral/runner.log` 的 `/cmd_vel/autonomy_raw` 所有权门未收敛；观测为1个 publisher、2个 subscriber，除预期 `cmd_vel_arbiter` 外还有 `_NODE_NAME_UNKNOWN_` subscriber。后者 GID 前缀 `01.0f.7a.ab`，预期 MPC/arbiter endpoint 前缀为 `01.0f.1e.d8`。
  - yaw102：`/tmp/ats_goal_set_next_100/yaw/runner.log` 的 `/cmd_vel/selected` 所有权门未收敛；观测为2个 publisher，节点名均为 `cmd_vel_arbiter`，但 GID 分别以 `01.0f.1e.d8` 与 `01.0f.7a.ab` 开头。不同 GID 仅证实不同 endpoint 身份，不能单凭此前缀断言来自外部主机或确定发现异常根因。
  - nearby103：`/tmp/ats_goal_set_next_100/nearby/runner.log` 记录 `timeout waiting for localization tracking`，同时 GICP multi-guess 多次报告 `produced no valid candidate`，adapter ready=0、source generation=0；这说明准入时未得到所需定位健康状态，不是 nearby 动作性能结果，也不把单条拒绝理由当成完整根因。
- MuJoCo matrix220–223（`ROS_LOCALHOST_ONLY=1`，`/tmp/ats_goal_set_next_220/summary.json`）：straight/lateral/yaw/nearby 四场全部失败；各自 `runner_status.env` 为 action=not_started、navigation safety=failed、recorder=not_started、recorder evidence=unverified、analysis=not_run、teardown=passed、launch wait=0、无升级、runner exit=1。各 `runner.log` 首条失败为 `FAIL: timeout waiting for localization tracking`；运行中 GICP multi-guess 多次 `budget_exhausted` / `produced no valid candidate`，adapter `ready=0`、`reason=localization is not tracking`。这是定位 TRACKING 准入超时，不是 DualMap 授权误拒动作，也不因 clean teardown 改写为通过。按用户要求不继续对本机 GICP 性能做反复调参；此前 matrix100 straight 通过与 recovery216 通过仍保留为既有证据，不与本轮 220 结果互相覆盖。


## 二、已实现未运行：闭环边界账本与部署计划

以下是源码契约，不是所有边均经本轮故障注入确认。参数值如注明配置默认，不代表已读取每次 launch 的有效参数。

|边界|producer → consumer / type|frame、time、QoS、身份|lease / fallback / 可观测性|
|---|---|---|---|
|scan/odom/clock → 输入门|sensor/Point-LIO 或 sim → GICP、ROGMap；PointCloud2/Odometry/TF|GICP 的 `/registered_scan` 必须为 odom frame，不是原始 lidar frame；正、递增 stamp；SensorDataQoS keep_last(1)；ROS sample time 与 steady timeout 分开|GICP stale/future/schema/finite/range/height 拒绝与计数；ROGMap MuJoCo 配置 cloud/odom timeout 2s、TF 0.1s，失败不提交 update|
|候选 → confirmation|GICP callback → 有界 worker request/result|scan stamp、request generation；质量/overlap/information/inlier/ambiguity；cancel 不代替 result generation fence|旧结果丢弃；deadline 限制调度/接纳，不保证底层 align 可中断；confirmation 不直接创建第二 TF 权威|
|observation → fusion|GICP → fusion；RelocalizationObservation|map→base pose；scan stamp、sequence；reliable depth10；odom history 插值|future/stale/quality/frame/history/plausibility 拒绝；fusion 独占 map→odom；拒绝不推进修正|
|fusion → planner/map/control|fusion → status、Odometry、TF consumers|localization 为 odom→gimbal_yaw_odom；status reliable/transient-local depth1；epoch 仅进程内单调|非 TRACKING 下游停；GICP status 拒绝不续租；odom steady freshness、observation silence degraded/lost；完整 map identity/incarnation 尚缺|
|ROGMap → projection|ROGMap → adapter；GetRogMapProjection|numeric occupancy/ESDF/gradient、source generation、source stamp、map frame；service 默认 QoS|map_mutex 内一致投影；unknown/invalid/stale 不伪造距离；不从可视化点云反解析|
|projection → fused map|adapter + static/terrain/slope → planning grid/snapshot/status|static reliable/transient-local；terrain depth10；输出 reliable/transient-local depth1；publication sequence 与 source generation 分开|steady request deadline，超时 remove_pending_request、清 pending；epoch 丢迟到 callback；配置 request/snapshot timeout 4s、input timeout 2s；失败 blocked unknown/ready=false|
|planning grid → immutable snapshot|adapter → MINCO；OccupancyGrid|digest 含 frame、resolution、origin、geometry、cells、unknown/threshold policy，不含 stamp/heartbeat；MINCO local generation 独立|同安全内容不 churn；新安全内容撤销 active reference，发布 FAILURE_SNAPSHOT_CHANGED/stop|
|snapshot → JPS/MINCO/safety|MINCO 内部 JPS/A*/RC-ESDF/MINCO/yaw/footprint/repair|一次 plan 捕获同一不可变 local snapshot；全向 SE2，不添加差速约束|unknown/outside/occupied/terrain/slope 保守处理；swept 和运行期复核拒绝 unsafe；repair 后重新求解、重新 gate|
|candidate → execution commit|MINCO → Goal Manager → ExecutionCommand|goal id、epoch、request、publication sequence、snapshot、stamp、yaw lease 复核；重定时 reference；execution reliable/transient-local|失效 stop/wait/replan；单条 ExecutionCommand 才是自动授权，legacy Path 不能安装 tracker；Bool 与 Path 跨 DDS topic 非原子|
|authorization → MPC|Goal Manager → MPC；ExecutionCommand|producer stamp + steady receipt lease、incarnation/sequence、epoch；body [vx,vy,wz]，world [x,y,yaw]|无效授权清 tracker/warm-start 并零；ready 或 legacy reference 不恢复旧授权；MPC 对 authoritative map generation 的独立订阅验证仍缺|
|raw → selected → actuator|MPC → arbiter → bridge/serial/sim|raw/selected/motion_control 各应唯一权威；实机 fake/chassis transform，Gazebo body→chassis yaw rotation|arbiter 拒绝清自动租约/旧速度并立即零；serial link/e-stop 门；实机执行器独立停机延迟尚未 HIL 测量|
|feedback → health/recovery|sim/底盘反馈 → localization/control/action/evidence|pose 必须归一到 map 再算目标误差；ROS stamp 用作身份，观测持续时间用 steady clock|contact telemetry 与离散/swept footprint 独立；失效后必须新鲜地图/定位和新授权，旧 reference 不复活|

源码所有者：`small_gicp_relocalization`、`ats_rog_map`、`ats_rog_map_adapter`、`minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`、`ats_cmd_vel_arbiter`；实机 `src/ats_sentry_bringup/launch/real_robot_navigation.launch.py`，仿真 `src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py`。配置与运行 graph 应分别审计，不把上述静态表当成全场景唯一 publisher 的实测。

### 实车/HIL 计划（未运行）

实机 `use_sim_time=false`，serial link required；sim relaxation 不能带入 real。fake/chassis velocity transforms 保留，必须核对唯一 publisher 与最终底盘 frame；`gimbal_yaw_odom→front_mid360` 的实机外参尚有仿真来源，必须先标定。串口硬件 watchdog、独立物理 E-stop、host/sensor 时钟偏移均需测量，不能由 topic 存在替代。

1. Bag A：30 分钟无自动执行的定位/地图/TF soak，至少 100 次真正 accepted observation（不是 callback 数）；逐条关联 scan/odom stamp、TF、map、sequence/epoch，记录拒绝 reason 与恢复。
2. Bag B：低速人工控制、仲裁与执行器验证；先架空/约束执行器做 HIL，再做受控地面试验；注入 link 中断、producer 停止、物理 E-stop，测 selected→actuator→实际静止的延迟和停止距离。
3. Bag C：前两门通过后，30 分钟低速自主分级目标与隔离故障；直线、横移、yaw、近点、clearance，再做 unknown/occupied/unreachable、map change、定位恢复和急停后新授权。不得共用污染状态串行证明不同故障。

每包记录 `/tf,/tf_static,/registered_scan,/localization,/localization/status`、实际 observation topic、ROG/adapter status+snapshot、planner status/ExecutionCommand/reference/e-stop、raw/selected/motion_control、wheel/base feedback、serial link、物理 E-stop 与 contact/bumper；确认实际 topic 名后录制。补采 CPU/RSS/内存、温度/电流/电压、真实点云频率与 p50/p95/p99/max age/gap、controller deadline miss 和恢复次数。现有只录 lidar/IMU/serial 的短 bag 配置不足以完成此计划。

## 三、推断

- [INFERENCE] [Confidence: Medium] Gazebo Transport→ROS 链可能存在接收/调度长尾；domain213 Transport p99 合格而 ROS p99 不合格，但 `dds_receive` 仍只是分类标签，不是因果证明。domain214 被 QoS 不兼容阻断，不能作为有效 bridge 性能对照。
- [INFERENCE] [Confidence: High] `ExecutionCommand` 的 atomic sample 优于 Bool/Path 的跨 topic 顺序；本轮拒绝/恢复测试支持授权边界，但不能据此声称端到端 map identity 已完整。
- [INFERENCE] [Confidence: Medium] 旧 Gazebo 的 chassis-as-gimbal frame 错误可能参与横向漂移，但213缺少同步 joint yaw/command/twist，因果未证明。native实际gimbal GT修复已构建并通过聚焦测试，运行几何与导航效果待新证据。模型支持全向四舵轮，不能归因为差速模型。

## 四、未实现与未验证边界

完整端到端 map identity/producer incarnation、跨进程重启后的 epoch 认证仍未实现。低 epoch producer 重启仍会 fail-closed，需要 consumer 重启，不能声称自动恢复完整。CSV 仍缺完整 identity/epoch/scan 关联。ATS 离线 replay 未实现。GICP 底层 align 不可中断，没有硬 worker shutdown latency 证明；默认 ambiguity margin=0 的门尚未通过真实候选分布标定。七状态枚举与 DualMap MINCO-local 验权已在源码与聚焦单测落地（见第一节），但尚无本轮 MuJoCo/Gazebo 动作级闭环证明。

端到端 source generation 同号、跨 ABI canonical digest、独立物理 contact evaluator、performance budget 尚未完成。测试 helper 覆盖不等于完整节点故障矩阵；新 generation/source/publication 三者不得混写。MPC/arbiter 对 MINCO-local map generation 的双侧验权已由聚焦 GTest 覆盖；对 ROG adapter source generation 的权威同号校验仍缺。


Gazebo P1/nominal/red-box、MuJoCo 全目标及故障矩阵、corridor_mouth、north_pocket、100 次重复结构、3 个真实 bag、30 分钟连续运行、100 次 accepted observation、HIL、CPU/内存/温度、真实点云频率、实车低速验证，均须有各自本轮证据后才能升级状态。MuJoCo 自身 contact telemetry 不等于实车物理接触为零。

风险转入条件：任一定位/TF/地图/授权失效、旧命令恢复运动、不能在配置 lease 内停止、未知接触/异常运动、E-stop 不可用、温度电流超限或关键证据缺失，立即退出高风险运行，继续只读分析和低风险修复。Gazebo 未同时通过 P1 freshness 与 nominal action 前，不运行 red-box；不放宽安全门、超大 timeout 或禁用 footprint 来换成功。

最新推进决定：按用户要求停止反复 Gazebo 性能调参与重试；七状态与 DualMap 地图授权已实现并完成聚焦单测；MuJoCo localhost 四场景 matrix220 因定位 TRACKING 超时全部 fail-closed。不宣称四环境稳定运行或 MuJoCo/Gazebo 矩阵通过。Gazebo red-box 不宣称通过，既有 P1/nominal 前置门与所有未验证边界不因推进决定而放宽。
