# ATS Swerve MPC LTV-QP Backend Admission

更新时间：2026-08-07。本文是 `ats_swerve_mpc` LTV-QP 后端的准入记录，不是 QP 控制链、
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

## 复现命令与证据边界

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

**未验证**：稳定运行时 QP/iLQR 同周期 p50/p95/p99、分配/CPU、headless MuJoCo 全链、故障注入、
HIL、实车和物理接触。P2 仍未通过，P3 不得标记 Nav2-free；不得引用本轮单测日志宣称 50 Hz、
6 ms 或 p99 实时性。
