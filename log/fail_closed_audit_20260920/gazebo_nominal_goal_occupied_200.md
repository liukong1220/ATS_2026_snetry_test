# Gazebo nominal「goal occupied」修复审计（domain199 → 200）

日期：2026-09-21  
工作区：`/home/kong/ATS_2026_snetry_test`  
范围：Gazebo `TEST_PROFILE=nominal`，`PLANNING_GRID_OWNER=rog_map`，`NAV_TRACKING_GATE=1`

## 结论

- **domain199 FAIL**：health/TRACKING + GICP accept 已通，但 `Goal pose admission` 165/165 失败且 `jps failed: goal is occupied`；DualMap 无 `auto_authorized=1`；路径全空。
- **根因**：静态-only 规划栅格上目标 `(1.17,-2.94)` 自由且净空约 `1.3 m`；live 融合相对静态约多 `~185` 个 occupied 格。Gazebo Mid360 ROG 投影高度带 `[0.10,0.80]` + `core.inflation_step=1` 把近场/甲板回波膨胀进开阔南向走廊，使目标格 `!isTraversable`。
- **domain200 PASS**：`inflation_step=0` + `projection_min_height=0.20` 后，`action_status=succeeded`，`runner_exit=0`，`final_distance≈0.023 m`，`auto_authorized=1` / `emit mode=1` 出现，JPS/MINCO/MPC/cmd_vel 非空，零 `goal is occupied` 命中。

## 证据对照

| 项 | domain199 | domain200 |
|----|-----------|-----------|
| adapter height | `[0.10, 0.80]` | `[0.20, 0.80]` |
| `core.inflation_step` | 1 | 0 |
| goal occupied hits | 多次 | **0** |
| `auto_authorized=1` | 0 | ≥1（日志计数 18 含续租） |
| `action_status` | failed | **succeeded** |
| `final_distance_m` | ~2.5（未动） | **0.0226** |
| `runner_exit` | 1 | **0** |
| health | TRACKING(1) | CONFIRMED(6) |
| planning ownership | adapter 1/1 | adapter 1/1 |

产物：

- `log/gazebo_minco_mpc_chain/20260921_163724_nominal_none_domain200/`
- `/tmp/ats_gazebo_nominal_200_driver.log`
- 既证保留：MuJoCo `/tmp/ats_recovery_real_185.json`、`/tmp/ats_goal_set_straight_191_final`

## 代码变更（仅 Gazebo launch）

`src/sim/gazebo_simulator/rmu_gazebo_simulator/launch/ats_gazebo_nav.launch.py`：

1. `core.inflation_step: 0`（GT Mid360；静态墙仍由 `/map` 融合保留）。
2. adapter `projection_min_height: 0.20` / `projection_max_height: 0.80`。

未削弱：free-space 0.42 m、DualMap 唯一性、非 sim 的 occupied 合取语义；west corridor red_box 几何仍可能需单独回归。

## 未覆盖

- 实机 HIL；Gazebo red_box 全腿；MuJoCo 对本轮 launch 参数的复跑（MuJoCo 不吃该 Gazebo-only 覆盖）。
- `p1_admission_evidence=false`（`freshness_gazebo_lidar`）——独立于 nominal action 成功，不改写 runner PASS。
- 100 次重复 / soak / 独立接触评估。
