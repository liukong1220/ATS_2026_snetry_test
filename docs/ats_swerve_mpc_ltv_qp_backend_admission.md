# ATS Swerve MPC LTV-QP Backend Admission

更新时间：2026-08-09。本文是 `ats_swerve_mpc` LTV-QP 后端的准入记录，不是 QP 控制链、
`qp_shadow`、P2 或 P3 的验收声明。

## 当前结论

已批准并纳入导航仓 vendor 的活动后端为 **OSQP v1.0.0**。本记录批准的是可审计的
`qp_shadow` 组件接入，不是 QP 主链、实车实时性或 P2/P3 验收声明。`solver_mode` 默认仍为
`ilqr`；`qp_shadow` 只记录诊断，`qp` 在本阶段显式拒绝启动；iLQR 仍是唯一控制器和
`/cmd_vel_mpc` 唯一发布者。

| 准入字段 | 当前值 | 证据与边界 |
| --- | --- | --- |
| C++ QP backend | OSQP | 导航仓 `third_party/osqp` 原始源码快照，CMake 静态目标 `osqp::osqpstatic` |
| backend version | `v1.0.0`, tag `236713ce9a56c182ac3230d52108f952afce1523` | 官方 tag 核验 |
| official source | `git@github.com:osqp/osqp.git`; `https://codeload.github.com/osqp/osqp/tar.gz/refs/tags/v1.0.0` | 上游源码与 archive |
| source archive SHA-256 | `dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48` | `sha256sum` 与批准记录一致 |
| license | Apache License 2.0 | `third_party/osqp/LICENSE`；必须随源码保留 `NOTICE` |
| third-party notice | QDLDL、AMD、Stanford University、University of Oxford | `third_party/osqp/NOTICE` |
| ATS compatibility | 技术判断为 Apache-2.0 兼容，不替代组织法务意见 | 保留 LICENSE/NOTICE/版权声明 |
| dependency source | 受版本控制第三方源码快照，不使用 apt、未知系统 `libosqp` 或运行时下载 | 导航仓 `third_party/osqp` |
| CMake/package.xml | CMake `add_subdirectory(.../third_party/osqp)`，关闭 shared/demo/unit/codegen，link `osqp::osqpstatic`；无独立 ROS binary package 依赖 | `ats_swerve_mpc/CMakeLists.txt`、`package.xml` |
| target environment | Ubuntu 22.04.5、ROS 2 Humble、GCC 11.4.0、CMake 3.22.1、x86_64 | 当前目标环境 |

导航仓 `.gitattributes` 仅对 `third_party/osqp/**` 关闭 whitespace check 并提高 conflict marker
识别长度：OSQP 上游文档中保存有历史尾随空白和七字符冲突示例，vendor 内容保持原样，ATS 自有
源码仍使用默认 `git diff --check` 规则。这一 Git 属性不影响 CMake 构建、OSQP 源码或运行时行为。

本轮只读准入审计使用以下可复现命令，均未发现 OSQP、qpOASES、ProxSuite、HPIPM 或 Clarabel：

```bash
git grep -n -E '(find_package\((OSQP|osqp|qpOASES|ProxSuite|proxsuite|HPIPM|Clarabel)|target_.*(OSQP|osqp|qpOASES|ProxSuite|proxsuite|HPIPM|Clarabel))'
dpkg-query -W 'libosqp*' 'osqp*' 'libqpoases*' 'qpoases*' 'libproxsuite*' 'proxsuite*' 'libhpipm*' 'hpipm*' 'libclarabel*' 'clarabel*'
pkg-config --modversion osqp
ros2 pkg list | rg '^(osqp|osqp_vendor|qpoases|qpoases_vendor|proxsuite|hpipm|clarabel)'
```

这些查询仍可用于发现意外系统依赖；本轮实际构建使用仓内固定源码。OSQP archive、tag、LICENSE、
NOTICE 和 CMake target 已独立核验。[Confidence: High；来源、哈希、构建输入和 GTest 一致，
尚无目标机实时 benchmark]

## 冻结接口

`LtvQpSolver` 已由 `LtvQpOsqpSolver`（OSQP v1.0.0）实现并由 ROS node 的 `qp_shadow` 调用；
它不改变 iLQR 的求解、tracker、急停或唯一速度发布者。当前公开 QP 头位于
`ats_swerve_mpc/include/ats_swerve_mpc/qp/`，与 MPC 算法头隔离。实现必须持续满足以下契约：

- QP 输入使用不可变 CSC pattern；`column_offsets` 和 `row_indices` 是 setup contract，timer
  周期只能更新 value，模式变化必须显式重建，不得隐式分配。
- warm-start 必须同时按决策变量和约束 dual 的精确维度检查 finite；后端必须显式报告是否使用。
- 每个 result 必须返回 status、iteration、solve/update time、primal/dual residual、slack maximum、
  hard-constraint maximum violation、primal 和 dual payload。仅 `solved` 能进入候选复核；
  `solved_inaccurate` 只作诊断，不能下发控制。
- QP primal 必须先恢复 `u_qp[k]=u_nominal[k]+delta_u[k]`，并用同一个 `Se2Model` 做非线性
  rollout；`LtvQpCandidateValidator` 再独立复核每步 body `[vx,vy,wz]` 与四轮真实速度向量，核查 body
  velocity/acceleration、轮速、轮速度向量增量、有效舵角速率和 slack/hard-bound。方向未定义
  时使用 `ZeroSpeedGuard` 跳过方向角差，而非捏造舵角；轮速度向量增量仍为 hard check。
- `inputs_healthy`、`emergency_stop_active`、`collision_free`、localization/reference freshness、
  ExecutionCommand lease、gimbal 和 map freshness 是不可 slack 的外部硬 gate。当前
  `LtvQpBuilder` 的决策布局没有 tracking/terminal slack 列，因此非空 slack payload 一律拒绝，
  直到 slack variable、上下界和 penalty 经单独评审加入固定结构。

该接口、OSQP adapter 和单测已产生实际 QP 求解结果，`qp_shadow` 已实现但仅用于审计：运行期 node
尚无 collision/footprint 与 map freshness producer，故两项 gate 明确为 false，candidate 必须拒绝。
它不构造完整 wheel/collision QP 约束，也不能称为 `qp` 主链、MuJoCo/HIL/实车或实时性通过。

## 准入门槛与本轮核对

准入记录要求至少包含以下项目；本轮已完成来源、哈希、许可证和 API 能力核对，性能与全链
benchmark 仍是后续门禁：

1. 上游项目、精确 version/tag、官方发布来源、SHA-256、许可证原文与兼容性审查。
2. 目标 Ubuntu/ROS、CPU 架构、编译器和 CMake target；离线或 clean workspace 重建命令。
3. CSC API 能力：固定 pattern update、primal/dual warm-start、iteration/time-limit、primal/dual
   residual、infeasible/numerical status 的一一映射。
4. 独立 benchmark 和 deterministic GTest：同 pattern 二次求解、结构漂移拒绝、time limit、
   infeasible、non-finite、residual/slack reject，以及四轮低速/反向/横纵切换约束复核。
5. `qp_shadow` 已使用单一 `ControlCycleSnapshot` 冻结同周期的
   `current_state/reference/last_control`、ExecutionCommand identity、定位 epoch 与 reference 时间；
   iLQR 保持 `/cmd_vel_mpc` 唯一 owner。受控 `solver_mode=qp` 仍被显式拒绝。

本轮 GTest 已覆盖同 pattern 二次求解、结构漂移拒绝、primal/dual warm-start、全部非 `solved`
状态拒绝、solve/update time 字段、nonzero `delta_u` 重建、非线性 rollout、ZeroSpeedGuard、
外部硬 gate 和 `qp_shadow` 单 publisher；真实 collision/footprint 输入、长期 deadline 分布和
MuJoCo runtime benchmark 尚未完成，不能升级为 QP 主链准入。

## 2026-08-07 MuJoCo Shadow 观察：已运行，未通过准入

本轮先修复 MuJoCo CPU LiDAR 对当前 `mujoco==3.10.0` 的 ABI 调用：`mj_multiRay()` 的 Python
绑定要求在 `dist` 与 `nray` 之间传入 `normal` 槽位，bridge 不使用命中法线时显式传 `None`。修复前
LiDAR 子进程在首次 raycast 因参数左移抛出 `TypeError`，从而没有 `/registered_scan`，ROGMap 正确报
`cloud_age=inf`、`source_generation=0` 和 stale；本轮没有改变 cloud timeout、projection deadline、
unknown/occupied、adapter lease 或 MPC fail-closed 语义。

在无 viewer、隔离 `ROS_DOMAIN_ID=229` 下，以下同一 single profile 命令实际完成：

```bash
PLANNING_GRID_OWNER=rog_map SOLVER_MODE=qp_shadow LOG_LEVEL=info \
TEST_PROFILE=single P2_FAULT_CASE=none ROS_DOMAIN_ID=229 \
scripts/test_mujoco_minco_mpc_chain.sh
```

已验证的上游与 iLQR ownership 证据为：LiDAR 子进程启动；ROGMap 首次 projection 观察到
`cloud_age=0.084 s` 并返回 `ready=true, stale=false`；adapter source generation 从 `55` 增至 `138`；
`/traversability_grid` 和四个 ROGMap debug topic 非空；`/cmd_vel_mpc` 仍只有
`ats_swerve_mpc -> twist_to_motion_ctrl`，`/motion_control` 仍只有
`twist_to_motion_ctrl -> ats_mujoco_sim`。ATS action 到达 `(-9.011029, 1.468209)`，相对
`(-9.0, 1.47)` 的脚本误差为 `0.011173 m`，这只是保持默认 iLQR 主链的 nominal 结果，不能作为
QP candidate、P2 红框、P3 Nav2-free、HIL 或实车证据。

Shadow 运行**未通过 QP 准入停止条件**，不得启用 `solver_mode=qp`：

- 记录到的 8 条节流后 OSQP telemetry 均为 `status=max_iterations`、`iter=400`、
  `warm=false`、`candidate_feasible=false` 和 `reject=solver_status_not_solved`；即使个别
  primal/dual residual 已低于配置阈值，非 `solved` status 仍按契约拒绝，不能保存 warm-start
  或构造可下发 candidate。
- 固定 telemetry 窗口最后一次报告 OSQP solve `p50/p95/p99=3.829/5.424/5.874 ms`，但完整 control
  callback 为 `57.611/131.704/160.563 ms`。该次 launch 的有效参数是 `control_rate_hz=20.0`，即
  `50 ms` 周期，而非此前误写的 `50 Hz/20 ms`；即使按正确周期，callback p50 仍超期。旧的合并
  `deadline_miss_count=61` 混合了 OSQP status、OSQP time budget 和完整 callback，不能用来归因。
  因此这不是实时 QP shadow 通过证据。
- 8 条观测都显示 `same_snapshot=true`，并记录固定字节序 digest；摘要覆盖 current state、reference
  stamp/deadline/frame/state/control、solve 前 last control 及 ExecutionCommand identity。该 digest
  只证明 iLQR/QP 的输入同一性，不证明求解结果可行或实时。
- 当前 collision/footprint 与 map-health producer 仍未接入节点，两个 gate 继续 hard false；本次
  reject 首因是 solver status，不能把它误称为 collision/map gate 已通过或已执行的可行候选审计。

本轮窄验证为 `test_mujoco_lidar_cpu.py` 的 `2 passed`、`ats_mujoco_sim` 窄构建、
`ats_swerve_mpc` 窄构建和 `colcon test-result` 的 `62 tests, 0 errors, 0 failures`；MuJoCo
`ament_python` 的现有包级 test 注册仍显示 `0 tests`，故不将它写为包级 pytest 覆盖。启动清理阶段
还有 terrain/Python 节点在 SIGINT 后的已知 context-shutdown traceback；它出现在 action、ROS graph
和上游检查结束之后，不是 LiDAR 子进程运行期崩溃，但 clean shutdown 仍属独立未解决项。
最终摘要有效位源码修正后还在新 domain `228` 启动了一次同配置观察：上游、adapter 和两级 owner
检查均完成，但外层 `180 s` 时限在 action/QP telemetry 前中止；该进程组已用 `SIGINT` 后的
`SIGTERM` 正常清理，domain `228` 不计入上述 action、status 或分位数证据。

## 2026-08-07 QP-2.5 分阶段归因：已实现并运行，主链仍停止

`ControlCycleTelemetryRing` 现在同时记录 `ilqr` 与 `qp_shadow` 的最后 128 个正常控制周期：使用
`steady_clock` 区分 snapshot、iLQR solve/发布、LTV build、OSQP C API numeric update/solve 墙钟、
primal reconstruction、hard-check、telemetry、aggregation、logging、完整 callback 和 timer
interarrival。OSQP `OSQPInfo` reported update/solve time 与 C API 墙钟值分开保存。十类饱和根因计数
分别为 OSQP time-limit status、OSQP solve budget、完整 callback、iLQR solve、QP build、QP update、
candidate audit、aggregation、logging 和 timer interarrival；不再用单个 merged miss 代替根因分布。
`/ats_swerve_mpc/dump_control_telemetry` 是只读 `std_srvs/Trigger` service，JSON/文件 I/O 在 timer 外；
它不创建 QP Twist publisher，也不访问 tracker、warm-start、急停或安全 gate。

本轮运行 `scripts/test_mujoco_qp_shadow_profiles.sh` 的 A/B/C profile，域 `200/201/202`，均为
`planning_grid_owner=rog_map`、single 起点/目标、`20 Hz/50 ms`、`qp_time_limit_ms=10`、
`use_sim_time=false`。每个 raw artifact 在 action、两级速度唯一 owner、非零 `/cmd_vel_mpc` 和
`/motion_control` 检查后导出至 `/tmp/ats_qp25_profiles_20260807/{A_ilqr_warn,B_qp_shadow_warn,C_qp_shadow_info}`；
launcher 总进程受外层 600 s 限制在 C 导出后的收尾阶段终止，故不能把 runner 的最终 PASS 写为通过。
三份 JSON 和 manifest 均通过 `python3 -m json.tool`，随后由版本控制的离线分析器重建摘要。

| profile | full callback p50/p95/p99 (ms) | iLQR solve p50/p95/p99 (ms) | QP build / OSQP wall solve / hard-check p50 (ms) | status / candidate |
| --- | --- | --- | --- | --- |
| A `ilqr,warn` | 29.209 / 89.615 / 136.142 | 28.822 / 88.960 / 135.487 | not run | iLQR baseline，QP 未尝试 |
| B `qp_shadow,warn` | 52.044 / 78.478 / 92.001 | 31.350 / 58.473 / 71.815 | 12.084 / 3.727 / 4.100 | 128 `max_iterations`；0 feasible；0 warm-start |
| C `qp_shadow,info` | 114.200 / 258.097 / 283.488 | 87.984 / 221.718 / 249.684 | 17.269 / 5.067 / 5.740 | 126 `max_iterations` + 2 `time_limit`；0 feasible；0 warm-start |

B 的 Hessian diagonal 范围为 `0.66..56.0`、constraint row L2 范围约 `1.0..1.41510`，zero-delta
dynamic equality residual 为 `0`；C 同项为 `0.66..56.0`、`1.0..1.41594`、`0`。这描述实际矩阵的
尺度，不证明病态或构成参数放宽理由。B 的累计 root-cause events 为 callback `298`、iLQR `136`、
timer interarrival `307`，C 为 callback `128`、iLQR `103`、timer interarrival `114`、OSQP
time-limit status `2`、OSQP solve budget `1`；build/update/audit/aggregation/logging 根因均为 `0`。
根因累计覆盖 node 本次生命周期，分位数覆盖最后 128 槽，二者不可互换。

离线分析器判定 A/B/C `not_comparable`：source revision、scenario 与有效参数相同，但实际
`duration_ms_at_dump` 不同，且逐周期 `snapshot_identity_digest` 序列不同。因此**不得**从上述数值
计算或声明 qp_shadow 的配对增量成本、INFO 日志因果成本或 OSQP 的唯一超期责任。logging 阶段本身
的 p99 仅 `0.103/0.138 ms`，但这只是各自运行内的计时，不足以解释跨运行 iLQR 尾延迟。CPU 与
allocation 没有可信 profile，明确为未验证。实际 status 仍非 `solved`，collision/footprint 与
map-health producer 仍为 hard false；`solver_mode=qp`、iteration/deadline/residual 放宽和非 solved
warm-start 继续禁止。P2 未通过，P3 不得标记 Nav2-free，HIL、实车与物理接触未验证。

## 复现命令与证据边界

## QP-2.6 可配对采样与数值归因（已实现；MuJoCo 输入链阻塞）

本轮把采样窗口下沉到 `AtsSwerveMpcNode::finalizeControlTelemetry()` 所拥有的
`ControlCycleTelemetryRing`，不再由脚本从“最后 128 槽”猜测窗口。profile 通过
`telemetry_sampling_window_cycles` 显式请求固定周期数（当前上限仍为 128，默认 `0` 保持滚动诊断）；
首个满足 `execution_lease_valid=true`、`reference_fresh=true`，且 localization/map generation 均非零的
周期冻结窗口。窗口恒等
字段为 `manager_incarnation`、`goal_id`、`localization_epoch`、`map_generation`、
`map_publication_sequence`、`reference_stamp_ns`、`reference_deadline_ns` 和 `reference_frame`。
Goal Manager 的 `command_sequence` 是每次 Execute heartbeat 的续租序号，会逐拍记录并检查单调性，
但不作为 lease 恒等字段。窗口身份变化、lease/reference 失效、周期数未收满或 schema 缺失都保留
raw fragment/manifest，并由离线分析器稳定判为 `not_comparable`；不计算任何 delta。

遥测 schema 已升级到 `3`，每个 sample 同时保存上述 identity、`snapshot_identity_digest`、status、
iterations、warm-start、reported/C API wall update+solve、primal/dual residual、slack、十类 root cause、
candidate rejection 和 collision/map gate。QP builder metrics 新增 finite nonzero bound 的最小绝对值与
最大绝对值，和已有 Hessian 对角、constraint-row L2、zero-delta dynamic residual 一起作为只读数值证据。
manifest 额外保存 raw `sampling_window`、关键有效 QP 参数，以及每仓 `HEAD`、tracked diff 是否存在和
对应 SHA-256，避免未提交构建被误记成纯 `HEAD`；CPU/allocation 仍明确为
`unverified_no_trusted_profiler`/`unverified_no_trusted_allocator_profiler`。

`scripts/test_mujoco_qp_shadow_profiles.sh` 对 A/B/C 每个新 domain 请求同一 `128` 周期窗口；
`scripts/analyze_qp_shadow_telemetry.py` 只在三份 raw 的窗口完整、内部 lease/reference/map identity 稳定、
heartbeat 单调、snapshot digest 有效且逐周期 digest 完全一致时给出 `comparable`，否则保留
`shadow_increment_cost_conclusion=withheld`。即使窗口身份字段相同，FNV-1a digest 仍是审计摘要而非
密码学签名；跨独立 MuJoCo launch 的动态相等性证据不足时不能升级 QP 准入。

当前 fixture 对 operational builder 锁定的诊断范围为 Hessian diagonal `0.66..56.0`、非零 finite
bound magnitude `0.1..2.15`、constraint row L2 最小 `1.0` 且最大小于 `1.42`、zero-delta dynamic
residual `0`；这些范围只用于复现/排查，不改变 `qp_max_iterations=400`、`qp_time_limit_ms=10`、residual
或任何 safety gate。仅凭这组尺度 proxy 不能证明病态或支持 scaling/preconditioning；只有固定窗口中
重复复现 `max_iterations` 且有独立条件数/缩放实验时，才可提出后续建议，本轮不实施 scaling。

**本轮已验证**：以本地 `/tmp/ats_qdldl_v0_1_8` 作为 QDLDL `FetchContent` 源完成
`ats_swerve_mpc` 窄构建；`colcon test --base-paths src --packages-select ats_swerve_mpc` 为 12/12
通过，随后 `colcon test-result --test-result-base build/ats_swerve_mpc --verbose` 为 73 tests、0 failure。
受影响 Python/Bash 语法和 source MuJoCo launch `--show-args` 通过，后者确认
`telemetry_sampling_window_cycles`。临时 schema-3 fixture 的三组完整窗口只有 C 的逐周期 digest 不同，
analyzer 确认输出 `not_comparable` 和 `shadow_increment_cost_conclusion=withheld`；这只验证 fail-closed
分析逻辑，不是 MuJoCo 性能样本。

**MuJoCo 停止条件**：新、空 domain `210` 的 A profile 在输入链就失败，未进入
`/cmd_vel_mpc`/`/motion_control` runtime ownership 检查，也没有 raw telemetry 或 manifest。运行时
`mujoco==3.4.0` 的 CPU LiDAR 子进程在
`src/sim/ats_mujoco_sim/mujoco_lidar/core_cpu/mjlidar_cpu.py:56` 调用 `mj_multiRay()` 时因 `vec` 参数
形状不兼容退出，继而 `/registered_scan` 缺失、ROGMap 以 stale fail-closed；launch log
`/tmp/ats_minco_mpc_test_launch_210.log` 的 SHA-256 为
`cc36ba626f83ce2018ae69060351dbe8a8d3a5669d3508d381ba8aae91772bfc`。这是 MuJoCo binding/输入链阻塞，
本轮不改动该无关模块，也不伪造 B/C profile、window、ownership、terminal/contact 或 QP 统计。
不得把旧构建树的 29 项结果、旧 A/B/C artifact 或本轮临时 fixture 写成 QP-2.6 通过；P2/P3、HIL、
实车和物理接触仍未验证。

从干净目标环境复现 vendor 快照和构建：

```bash
curl --fail --location --http1.1 \
  https://codeload.github.com/osqp/osqp/tar.gz/refs/tags/v1.0.0 \
  -o osqp-v1.0.0.tar.gz
sha256sum osqp-v1.0.0.tar.gz
tar -xf osqp-v1.0.0.tar.gz
cp -a osqp-1.0.0/. src/ats_sentry_nav/third_party/osqp/
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select ats_swerve_mpc --parallel-workers 1
```

仓内 CMake 不联网、不查找系统 `libosqp`；`osqp_setup()` 只在 `LtvQpOsqpSolver` 构造期调用一次，
控制 timer 只更新固定 CSC 数值、`q/l/u` 和 primal/dual warm-start；结果再由 snapshot 驱动的
nonlinear reconstruction/hard-check 与固定 128 槽 telemetry 审计。LTV conversion 的 rows/columns
为固定动力学块、控制增量块和 bounds identity，不引入 slack 列；当前 `qp_max_tracking_slack=0`，
所有非空 slack payload 拒绝。

**已验证（组件）**：

- `MAKEFLAGS=-j1 colcon build --base-paths src --packages-select ats_swerve_mpc --parallel-workers 1`；
- OSQP adapter、固定 CSC、primal/dual warm-start、状态/残差/solve-update time、非线性重建、
  ZeroSpeedGuard、hard-check 和 `qp_shadow` 单 publisher GTest；
- `source install/setup.bash && colcon test --base-paths src --packages-select ats_swerve_mpc`；
- 结果文件由 `colcon test-result` 汇总为 9 个测试目标全部通过（测试总数以本机构建输出为准）；
- launch Python syntax、`ros2 launch ... --show-args` 和三仓 `git diff --check`。

**已验证（ROS node gate）**：`qp_shadow` 的 node gate 在 iLQR 可发布非零控制时确认 command topic
publisher 数保持 1；该路径只读同周期 snapshot，OSQP 不创建 QP Twist publisher。代码记录
status/iteration/solve-update time/primal-dual residual/slack/hard margin、首控 delta 与固定窗口
p50/p95/p99。当前节点没有 collision/footprint 或 map freshness 健康 producer，因此两项 gate
保守 hard reject，不能宣称 runtime QP candidate feasible。

**未验证或未通过**：MuJoCo headless nominal 的 upstream/iLQR 链和 QP input digest 已实际运行，
但 QP status/iteration 与 callback deadline 未通过；分配/CPU profile、collision/footprint/map-health
真实 producer、故障注入、P2 红框、HIL、实车和物理接触仍未验证。P2 仍未通过，P3 不得标记
Nav2-free；不得引用本轮单测或该次 iLQR nominal 日志宣称 QP 50 Hz、6 ms、p99 实时性或生产准入。

## 2026-08-09 domain 213 后续复核与 QP-2.7 阻塞

本轮从三个仓库的 `origin/develop` 快进确认后，使用新的空闲 `ROS_DOMAIN_ID=213` 检查上游输入链。
当前 Python 环境为 `mujoco==3.10.0`。headless LiDAR bridge 的实际消息证据为：

- `/local_pointcloud`：`sensor_msgs/PointCloud2`，`frame_id=front_mid360`，`width=787`；
- `/registered_scan`：`sensor_msgs/PointCloud2`，`frame_id=odom`，`width=104`；
- ROGMap 日志 `cloud_age` 有限，source generation 从 `76` 增至 `90`；
- adapter heartbeat `ready=1`，publication sequence 持续递增；脚本确认 `/rog_map/occ`、`/rog_map/inf_occ`
  非空后在 `/rog_map/unk` gate 停止，因此 `/rog_map/esdf`、planning grid、action 和 ownership 尚未
  取得该 profile 的运行期证据。

该 profile 在 action、`/cmd_vel_mpc`/`/motion_control` ownership 和 telemetry dump 前停止于
`/rog_map/unk` 非空 gate。有效参数为 `core.visualization.publish_unknown=false`，ROGMap/adapter 日志的
unknown cell count 为 `0`。因此本轮没有 schema-3 raw profile，也没有 A/B/C 配对、QP status/残差分布、
终点或 contact 证据；不能把 debug publisher 存在或上游 generation 递增写成 unknown 语义通过。

后续 QP-2.7 的准入前置条件是由真实行为 owner 提供受版本控制的 unknown 场景或独立 fault fixture，并
分别验证 raw unknown payload、frame/stamp/QoS、source generation、adapter heartbeat、all-unknown planning
snapshot 与两级确定性零速度。不得伪造 unknown 点、修改 `cloud_age=inf`、关闭 stale/lease、使用静态假地图、
或放宽规划/控制 fail-closed 逻辑。只有 nominal upstream gate、action、唯一 ownership 和完整 schema-3
固定窗口全部满足后，才允许重新运行 A/B/C；窗口不完整或 digest 不一致时 analyzer 必须保持
`not_comparable`/`withheld`。

P2 仍未通过；P3 不得标记 Nav2-free；QP `solver_mode=qp`、iteration/deadline/residual 放宽、non-solved
warm-start、HIL、实车、物理接触和可信 CPU/allocation profile 继续未验证。

## QP-2.7 unknown fixture 与 nominal realtime 阻塞（2026-08-09）

**已实现且聚焦验证通过**：本轮把 P2 unknown fault 固定为真实 ROGMap 数值 source 路径。MuJoCo
LiDAR worker 的 `lidar_occlusion_enabled` 在保持正常 message cadence、frame 和 stamp 的前提下发布空回波；
ROGMap 的 test-only edge trigger 清空概率/inflation/frontier/ESDF 表并保持 source generation 单调；adapter
只有在 numeric projection 已包含 unknown 时才在 fusion 前 mask static/terrain/slope。all-unknown 由既有
fusion 真值表得出，blocked unavailable snapshot 保留 `-1` audit occupancy 与 NaN distance/gradient，且
`ready=false`。这不修改 `ats_swerve_mpc` 的 tracker、last control、iLQR warm start、emergency-stop 或
命令 publisher，也不读取 debug PointCloud2 作为地图。

`ats_rog_map`（7）、`ats_rog_map_adapter`（14）聚焦 CTest 均为 0 failures；指定八包单 worker
`colcon build --base-paths src`、脚本/Python syntax、三个相关 launch `--show-args` 和三仓 diff check
通过。`test_freeze_motion_parameter.py` 的直接 pytest collection 被当前 install 的
`ModuleNotFoundError: carstatemsgs` 阻断，故没有把它记为已通过。

**运行失败，停止条件生效**：headless domain `181`（info）与 `182`（warn）都以
`SOLVER_MODE=ilqr`、`P2_FAULT_CASE=none` 运行。ROGMap numeric projection、adapter heartbeat、唯一
`/rc_esdf/planning_grid` owner、action accepted 和两级 command topic owner 已在运行期观察到；但 full
callback 长期越过 `20 Hz/50 ms`。domain `181` 终样本 p50/p95/p99 为
`155.387/281.713/326.962 ms`；domain `182` 为 `94.671/268.061/392.287 ms`，中途达到
`482.97 ms` iLQR solve。随后 odometry/ROGMap stale、projection 延迟和 map heartbeat lease timeout 导致
action `ABORTED/result_code=4`。这是 iLQR nominal realtime/health 输入链的 first violation；domain `182`
显示的 `qp_status=backend_unavailable, iter=0` 仅表示 mode 为 ilqr，没有提供任何 OSQP solved、iteration、
residual 或 QP timing 证据。

因此本轮**未运行**真实 unknown 的 fault 前非零运动、unknown payload、all-unknown、two-stage zero、恢复后
generation/sequence 递增、旧 reference 拒绝和新目标恢复；也**未运行** paired Shadow A/B/C。未启用
`solver_mode=qp`，没有提高 OSQP iteration、接受 `solved_inaccurate`/`max_iterations`、放宽 deadline/residual，
也没有削弱 map/unknown/collision/footprint/localization/gimbal/lease gate。QP-2.7 未通过；P2、P3、QP-3、
P4/HIL/实车和 MuJoCo physical contact 继续未通过或未验证。运行日志：
`/tmp/ats_minco_mpc_test_launch_181.log`、`/tmp/ats_minco_mpc_test_launch_182.log`。
