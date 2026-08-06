# ATS Swerve MPC LTV-QP Backend Admission

更新时间：2026-08-06。本文是 `ats_swerve_mpc` LTV-QP 后端的准入记录，不是 QP 控制链、
`qp_shadow`、P2 或 P3 的验收声明。

## 当前结论

当前活动后端为**无**。因此不存在可填写的已准入后端版本、许可证、依赖来源、CMake target
或 ROS package；这些字段当前均为“不适用（未选择）”。在后端完成准入前，
`solver_mode=qp_shadow` 不实现，`solver_mode=qp` 更不得实现；iLQR 仍是唯一控制器和
`/cmd_vel_mpc` 唯一发布者。

| 准入字段 | 当前值 | 证据与边界 |
| --- | --- | --- |
| C++ QP backend | 无 | 活动导航仓没有 QP `find_package`、link target 或 vendor source |
| backend version | 不适用 | 未选择软件包，不能虚构 version pin |
| license | 不适用 | 未选择上游或二进制，不能声明许可证已审核 |
| dependency source | 不适用 | 当前工作区没有锁定源码、系统包或 ROS vendor package |
| CMake/package.xml dependency | 无 | `ats_swerve_mpc` 仅声明现有 ROS/Eigen 依赖 |

本轮只读准入审计使用以下可复现命令，均未发现 OSQP、qpOASES、ProxSuite、HPIPM 或 Clarabel：

```bash
git grep -n -E '(find_package\((OSQP|osqp|qpOASES|ProxSuite|proxsuite|HPIPM|Clarabel)|target_.*(OSQP|osqp|qpOASES|ProxSuite|proxsuite|HPIPM|Clarabel))'
dpkg-query -W 'libosqp*' 'osqp*' 'libqpoases*' 'qpoases*' 'libproxsuite*' 'proxsuite*' 'libhpipm*' 'hpipm*' 'libclarabel*' 'clarabel*'
pkg-config --modversion osqp
ros2 pkg list | rg '^(osqp|osqp_vendor|qpoases|qpoases_vendor|proxsuite|hpipm|clarabel)'
```

这四类结果只证明本工作区和构建环境当前无可复现后端；它们不证明任何候选后端不适合 ATS。
[Confidence: High，受版本控制构建文件、本机系统包、pkg-config 与 ROS package index 一致；
未对外部候选进行 benchmark]

## 冻结接口

导航仓新增的 `LtvQpSolver` 是纯 C++ 抽象，未被 ROS node 或 iLQR 调用。它要求未来已批准
后端逐项实现以下契约：

- QP 输入使用不可变 CSC pattern；`column_offsets` 和 `row_indices` 是 setup contract，timer
  周期只能更新 value，模式变化必须显式重建，不得隐式分配。
- warm-start 必须同时按决策变量和约束 dual 的精确维度检查 finite；后端必须显式报告是否使用。
- 每个 result 必须返回 status、iteration、solve time、primal/dual residual、slack maximum、
  hard-constraint maximum violation、primal 和 dual payload。仅 `solved` 能进入候选复核；
  `solved_inaccurate` 只作诊断，不能下发控制。
- `LtvQpCandidateValidator` 独立重建每步 body `[vx,vy,wz]` 与四轮真实速度向量，核查 body
  velocity/acceleration、轮速、轮速度向量增量、有效舵角速率和 slack/hard-bound。方向未定义
  时使用 `ZeroSpeedGuard` 跳过方向角差，而非捏造舵角；轮速度向量增量仍为 hard check。
- `inputs_healthy`、`emergency_stop_active` 和 `collision_free` 是不可 slack 的外部硬 gate。当前
  `LtvQpBuilder` 的决策布局没有 tracking/terminal slack 列，因此非空 slack payload 一律拒绝，
  直到 slack variable、上下界和 penalty 经单独评审加入固定结构。

该接口和单测仅固定 future backend 的输入、诊断和拒绝语义；它不构造完整 wheel/collision QP
约束，也不产生实际求解结果，不能称为 QP backend 或 `qp_shadow` 已实现。

## 准入门槛

在任何依赖安装、下载、vendor 导入或 CMake/package.xml 改动前，必须有单独的审批记录，至少包含：

1. 上游项目、精确 version/tag、官方发布来源、SHA-256、许可证原文与兼容性审查。
2. 目标 Ubuntu/ROS、CPU 架构、编译器和 CMake target；离线或 clean workspace 重建命令。
3. CSC API 能力：固定 pattern update、primal/dual warm-start、iteration/time-limit、primal/dual
   residual、infeasible/numerical status 的一一映射。
4. 独立 benchmark 和 deterministic GTest：同 pattern 二次求解、结构漂移拒绝、time limit、
   infeasible、non-finite、residual/slack reject，以及四轮低速/反向/横纵切换约束复核。
5. 完成上项后，才可实现 `solver_mode=qp_shadow`。该模式必须与 iLQR 共享同一
   `current_state/reference/last_control` snapshot，iLQR 保持 `/cmd_vel_mpc` 唯一 owner；再后才可
   讨论受控 `solver_mode=qp`。

## 停止条件与未验证项

本轮命中停止条件：“无已批准且可复现的 C++ QP backend”。未使用 `apt`、网络下载或未知 vendor
源码改变构建环境，也没有手写生产 QP solver。**已验证（组件）**：
`MAKEFLAGS=-j1 colcon build --base-paths src --packages-select ats_swerve_mpc --parallel-workers 1`、
source 工作区后的 8/8 CTest（含新增 `test_ltv_qp_solver`）和
`colcon test-result --test-result-base build/ats_swerve_mpc --verbose` 的 `49 tests, 0 errors,
0 failures, 0 skipped` 通过；launch Python syntax 与 `ros2 launch ats_swerve_mpc
ats_swerve_mpc.launch.py --show-args` 通过。`qp_shadow`、QP/iLQR 同周期诊断、p50/p95/p99、
headless MuJoCo、HIL、实车以及 P2/P3 门禁均未执行。P2 仍未通过，P3 不得标记 Nav2-free。
