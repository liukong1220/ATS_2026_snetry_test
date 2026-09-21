# Gazebo 全局重定位 / 健康门审计（domain192–199）

日期：2026-09-21  
工作区：`/home/kong/ATS_2026_snetry_test`  
范围：Gazebo nominal 单次回归（`TEST_PROFILE=nominal`，`PLANNING_GRID_OWNER=rog_map`，`NAV_TRACKING_GATE=1`）

## 结论摘要

- **重定位确认链已打通**：domain199 达到 `localization_state=1`（TRACKING）、health gate 稳定 3/3、`GICP alignment accepted` 持续刷新、fusion epoch 稳定（`localization epoch changed` = 0）。
- **Gazebo nominal 仍未闭环**：domain199 action 被接受后因 **目标栅格占用**（`jps failed: goal is occupied`）与 `map_ready=0` 挂起重规划，最终 `ABORTED`；`auto_authorized=1` / `emit mode=1` 未出现。
- **不宣称四环境 DoD 已满足**。MuJoCo recovery185 / straight191 既往通过证据保持有效；实机仍为部署审计，无 HIL。

## 域结果表

| Domain | 结果 | 关键证据 |
|--------|------|----------|
| 192 | FAIL health | `min_information_eigenvalue=1e4` 密度耦合拒识；`localization_state=4` |
| 193 | FAIL action not accepted | `gated=1`×N 零 accept；pending 被 BOOTSTRAP/stale status 清掉 |
| 194 | FAIL action not succeed | health 过（DEGRADED）；confirmation_recheck `not converged`；`sim_relax=0`（force window only） |
| 195 | FAIL | preferMultiGuess LOST-only → DEGRADED 下 coarse+fine 刷屏 |
| 196 | FAIL | 50× recheck gated 零 accept；`epoch_changed` 清 pending |
| 197 | FAIL | **GICP accept 通**（88×）；每次 accept 推 epoch → adapter `ready=0`；DualMap 无授权 |
| 198 | FAIL health | epoch 阻尼生效；但 `min_registration_delta=0.40` 饿死观测租约 → LOST |
| 199 | FAIL plan | health **TRACKING+map_ready**；accept 持续；epoch_inv=0；**goal occupied**；watchdog `map_ready=0` |

## 已落地修复（本轮）

### `ats_sentry_nav` / `small_gicp_relocalization`

1. pre-accept pending 在 BOOTSTRAP / DEGRADED / LOST 抖动下保留；status stale 不再 `invalidateRecovery` 清 pending。
2. `simRelaxAllowed()`：冷启动至首次 accept（含 confirmation_recheck）可放过 optimizer `converged` 标志。
3. `preferMultiGuess()`：首次 accept 前在 DEGRADED/BOOTSTRAP 仍走 multi_guess。
4. `epoch_changed` 在 pre-accept pending 打开时不 `invalidateRecovery`（修 domain196）。
5. NO_ODOM：保留 pending；精确 stamp 失败时回退最新 TF。
6. 可观测：`relax_sim=` ready 行、`Confirmation pending opened`、`GICP alignment accepted`。
7. 单测：`PreAcceptPendingSurvivesBootstrapAndStaleStatus`、`SimRelaxCoversPreAcceptOutsideForceWindow`、`ColdStartDegradedPrefersMultiGuess`、`PreAcceptPendingSurvivesEpochChange` 等 PASS。

### `gazebo_simulator` / `rmu_gazebo_simulator`

1. `min_information_eigenvalue: 0.0`；`observation_lost_timeout_s` 默认 30；cold-start prior；`max_fine_per_scan=4`（去重）。
2. health probe：`TRACKING|CONFIRMED|DEGRADED`。
3. `relax_convergence_for_sim: ParameterValue(True)`；确认容差放宽；`fine_max_iterations: 32`。
4. fusion：`epoch_translation_threshold=0.50`、`epoch_yaw_threshold=0.35`、`max_odom_history_samples=20000`。
5. GICP 租约：`min_registration_translation/yaw_delta=0.12`，`registration_interval_s=0.35`（避免 198 饿死观测）。

## domain199 残余失败（规划，非确认）

```text
jps failed: goal is occupied expanded=0 clearance=0.180 m
goal=(1.170, -2.940, yaw=-1.571)
Progress watchdog ... map_ready=0 localization=1 ...
action_result ... progress watchdog exhausted ... final_distance_m=2.500
final_pose≈(1.170,-0.440)
```

推断：GICP accept 后 map→odom 与 Gazebo 规划栅格对齐仍使名义目标落在占用单元；属地图/对齐/投影问题，不是 confirmation pending 逻辑回归。

## 产物路径

- `log/gazebo_minco_mpc_chain/20260921_*_nominal_none_domain19{2-9}/`
- `/tmp/ats_gazebo_nominal_{192-199}_driver.log`
- MuJoCo 既证：`/tmp/ats_recovery_real_185.json`、`/tmp/ats_goal_set_straight_191_final`

## 未覆盖

- Gazebo nominal 全程成功 / DualMap `auto_authorized=1`
- 目标占用与 map→odom 数值对齐标定
- 实机 HIL；ATS 离线 replay；corridor 矩阵；100 次重复
