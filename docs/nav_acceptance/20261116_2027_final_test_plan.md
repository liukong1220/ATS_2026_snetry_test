# 2027 导航验证方案：基地到基地、隧道与对称点速度

更新日期：2026-11-24。本文件是 RMUC 2027 仿真导航验收的唯一任务坐标、执行入口与当前状态账本。

- 默认仿真入口为有图模式：`rmuc_2027_mujoco.launch.py` 默认 `slam:=false`，加载 `src/ats_sentry_bringup/map/rmuc_2027.yaml` 与 `pcd/rmuc_2027.pcd`；唯一 MPC 求解器为 `OSQP v1.0.0 SQP-RTI`（`solver_mode:=qp`）。
- 无先验建图测试显式传 `slam:=true`（`scripts/s3_mujoco_acceptance.py` 加 `--mapless`），PGM 和 PCD 输入置空，两种模式分开记录。
- 当前默认无窗口运行（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`）；未带窗口时画面项记为未验证。自 Wave 6A 起已落地独立 MuJoCo `mujoco_geom_normal_v1` geom-pair 物理接触评估器，按轮地/桥面合法接触与车体/轮侧/顶板/障碍非法碰撞精确判定 `physical_contact_acceptance`。
- 后续优化路线与新会话提示词见 [主报告第 5 节](../调研/无先验地图钻洞高速导航优化调研报告.md#51-执行顺序与当前停点) 与 [提示词选择](../调研/无先验地图钻洞高速导航优化调研报告.md#97-提示词选择新会话可直接复制)。

---

## 1. 固定场景与坐标

| 项 | 现行约定 | 说明 |
| --- | --- | --- |
| 场景与定位 | `rmuc_2027`，`localization_source:=lio`，`use_identified_odometry:=true` | 不用真值定位替代 Point-LIO |
| 实际模型范围 | $x \in [0, 28]$、$y \in [0, 16]\ \mathrm{m}$，场地中心 `map (14, 8)` | 与 560×320、0.05 m PGM 同框（不套用 28×15 m 概念尺寸） |
| 基地起点（B1/T1–T3） | `(3.37, 9.03, 0.465, 0)` | [s3_mapping_routes.yaml](../../src/sim/ats_mujoco_sim/config/s3_mapping_routes.yaml) 显式起点 |
| 对面基地目标（B1） | `map (24.63, 6.97, yaw=0)` | 中心对称点，XY 位移 `(21.26, -2.06) m`，直线距离 `21.36 m` |
| S3 `odom` 全局层覆盖 | `40 × 30 m`（`0.1 m`），2027 偏置原点 `(-8, -15)` | `odom` 覆盖 $x \in [-8, 32]$、$y \in [-15, 15]\ \mathrm{m}$，包含整场与远目标 |
| V1 对称点速度起点/目标 | `(13.40, 2.00, 0.550, 0)` -> `map (14.60, 14.00, yaw=0)` | 直线距离 `12.060 m`，静止到静止 `2 m/s²` 理想峰值约 `4.911 m/s` |

中心对称目标公式（场地原点 $(x_0, y_0)$、尺寸 $(W, H)$、起点 $S=(x_s, y_s)$）：

$$
C = \left(x_0 + \frac{W}{2},\; y_0 + \frac{H}{2}\right), \qquad G = 2C - S, \qquad D = \lVert G - S \rVert_2.
$$

---

## 2. 验收任务矩阵

| ID | 任务 | 正式入口（`scripts/s3_mujoco_acceptance.py`） | 必报指标 |
| --- | --- | --- | --- |
| **B1** | 基地到基地 | `--profile rmuc_2027_tunnels --route base_to_base --case route`（无先验加 `--mapless`） | 单 action 终点真值（$\le 0.08\ \mathrm{m}$）、实走里程、q1/q2、MINCO/OSQP 墙钟、`0` QP 告警 |
| **B2** | 动态障碍绕行接回 | `--profile rmuc_2027_tunnels --route base_to_base --case b2_detour`（无先验加 `--mapless`） | 在线实体箱投放、触发 `local_rejoin` 绕行、终点真值与 `0` 碰撞；独立全堵目标验证有界失败与三路零速 |
| **T1** | 真实 `tunnel_B` 往返 | `--profile rmuc_2027_tunnels --route tunnel_b_roundtrip --case route --waypoint-control-tolerance-m 0.08` | 全部 13 点到达、`tunnel_ramp_bridge`（正 1）与 `tunnel_B`（正/反各 1）侧偏/航向误差、末点 $\le 0.08\ \mathrm{m}$、q1/q2 |
| **T2** | 南坡/南侧通道往返 | `--profile rmuc_2027_tunnels --route ramp_south --case route --waypoint-control-tolerance-m 0.08` | 全部 13 点、`tunnel_south_plate` 正/反各 1、坡道支撑与顶板净空、独立物理接触归因 |
| **T3** | 六隧道完整路线 | `--profile rmuc_2027_tunnels --route six_tunnels --case route --waypoint-control-tolerance-m 0.08` | 全部 42 点与 6 声明通道正反向计数；运行前声明重复次数 $N$ |
| **V1** | 对称点导航速度 | `--profile rmuc_2027_tunnels --route symmetry_speed --case route` | 经 `/ats_navigate_to_pose` 到达，记录参考/MPC/仲裁/底盘/真值速度曲线与终点 $\le 0.08\ \mathrm{m}$ |
| **F** | 独立安全故障矩阵 | `--case {unknown,no_path,unsafe,adapter_lease,point_lio_restart}`（一项一独立 domain、一新 launch） | 故障触发急停、三路持续零速、恢复后 epoch/seq 递增且旧 `ExecutionCommand` 拒绝不复活 |
| **S3** | 无先验建图与保存/重载 | `--profile rmuc_2027 --mapless --case route --post-route-case obstacle_and_dynamic` 及 `--reload <prefix>` | 建图到达精度、障碍可见延迟（`p95 < 250 ms`）与射线清除、保存漏检/误检、动态不保存、重载 `TRACKING` 与首目标 $\le 0.08\ \mathrm{m}$ |

> T1 第 7 点为前置 `ramp_bridge`，不计作 `tunnel_B` 通过（真实 B 位于第 10–13 点）；禁止用 `--max-goals` 片段替代完整 T1/T2/T3 通过。

---

## 3. 速度与安全门口径

- 暂定平移上限 `5.0 m/s`、加速度 `2.0 m/s²`、yaw 上限 `3.0 rad/s`、角加速度 `2.0 rad/s²`；`5 m/s` 为允许上限，未在当前几何下触发高速段时如实标注“未触发”，严禁直接给 `/motion_control` 发速度替代导航 action。
- 终点独立真值门：中间点 $\le 0.15\ \mathrm{m}$，终点与重载首目标 $\le 0.08\ \mathrm{m}$；当前规划/控制终端容差收紧为 `0.05 m` 以留足裕量。
- 跟踪门：`q1`（参考发布无冲突）必过；`q2`（实际足迹进入后续占据格）持续 $\ge 3$ tick 且跨 $\ge 2$ 快照判失败；`q3` 仅诊断。所有批次断言单一发布者所有权与零 Nav2（无 `bt_navigator`/`planner_server`/`controller_server`/`/plan`）。

---

## 4. 正式执行模板

运行前完成 `./build.sh`（Release）与 `python3 scripts/validate_navigation_config.py`，按本机网络加载 ROS/DDS 环境并导出未占用的 `ROS_DOMAIN_ID`（运行前用 `pgrep` 确认无残留进程）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID="${ATS_DOMAIN:?选择未占用的隔离 domain}"
export USE_RVIZ=false USE_MUJOCO_VIEWER=false

# 1) Stage A 有图核心路线（T1/T2/T3/B1/V1）
python3 scripts/s3_mujoco_acceptance.py \
  --profile rmuc_2027_tunnels --route tunnel_b_roundtrip \
  --case route --waypoint-control-tolerance-m 0.08 --registration-details \
  --output "${ATS_OUT:?独立输出目录}"

# 2) Stage A 动态绕行与 5 项独立故障（每项换新 domain 与新输出目录）
python3 scripts/s3_mujoco_acceptance.py \
  --profile rmuc_2027_tunnels --route base_to_base \
  --case b2_detour --output "${ATS_OUT:?独立输出目录}"
# --case 可换为：unknown / no_path / unsafe / adapter_lease / point_lio_restart

# 3) Stage B 无先验建图（空 PGM/PCD）与保存/重载
python3 scripts/s3_mujoco_acceptance.py \
  --profile rmuc_2027 --mapless --case route \
  --post-route-case obstacle_and_dynamic --output "${ATS_OUT:?独立输出目录}"
python3 scripts/s3_mujoco_acceptance.py \
  --profile rmuc_2027 --reload "${SAVED_PREFIX:?保存地图前缀}" \
  --output "${ATS_OUT:?独立输出目录}"
```

需要现场观察时（桌面未锁屏且有可用 `DISPLAY`），将 `USE_RVIZ=true USE_MUJOCO_VIEWER=true` 或直接运行 `ros2 launch ats_mujoco_sim rmuc_2027_mujoco.launch.py localization_source:=lio use_identified_odometry:=true use_viewer:=true show_viewer:=true use_rviz:=true`（无先验追加 `slam:=true`）。

---

## 5. 当前验收状态（2026-11-24 Wave 6A 最终二进制无窗口实测）

Stage A 全套 11 项（5 条路线 + B2 绕行 + 5 项独立故障）与 `small_10x10` S3 连续修正 3 项回归均基于 Wave 6A 最终二进制与唯一参数源，在独立隔离 `ROS_DOMAIN_ID` 下无窗口运行（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`；因本机桌面锁屏，开窗 RViz/MuJoCo 画面项记为未验证；实车硬件记为未验证）。全部通过批次均满足单一发布者所有权、零 Nav2 断言与 `footprint_gate_collisions = 0`。Stage B 无先验基线沿用 2026-11-23 Waves 0–5 实测并将在 Wave 6B 集中收口。

### 5.1 Stage A：有图定位模式（`slam:=false`，`rmuc_2027`）— 11/11 全过

| 批次 | 入口与路线 | 结果 | 关键实测指标与边界 |
| --- | --- | --- | --- |
| **T1** (`stageA_T1_d151`) | `rmuc_2027_tunnels / tunnel_b_roundtrip` | **通过** | `13/13` 到达，里程 `20.85 m`，末点真值误差 `0.0423 m`，足迹冲突 `0`，`13/13` q1/q2 全过；`tunnel_ramp_bridge`（正 1、侧偏 `0.0103 m`、`yaw_err=0.0345 rad`）与 `tunnel_B`（正/反各 1、侧偏 `0.0296 m`、`yaw_err=0.0252 rad`）全过；`physical_contact_acceptance=passed`（`illegal=0`）；`qp_backend p50/p95=0.311/0.998 ms`，MINCO `p50/p95=0.39/0.99 ms`，`retarget=0`（注：1 拍调度抖动 `time_limit` 确定性零速后恢复，`0` 硬约束拒绝） |
| **T2** (`stageA_T2_d152`) | `rmuc_2027_tunnels / ramp_south` | **通过** | `13/13` 到达，里程 `21.02 m`，末点真值误差 `0.0262 m`，足迹冲突 `0`，`13/13` q1/q2 全过；`tunnel_ramp_bridge`（正 1、侧偏 `0.0108 m`、`yaw_err=0.0283 rad`）与 `tunnel_south_plate`（正/反各 1、侧偏 `0.0327 m`、`yaw_err=0.0041 rad`）全过；`physical_contact_acceptance=passed`（`illegal=0`，原 `46940` 接触归因为 `legal_wheel_drivable_deck=23425` 与 `legal_wheel_ground=420145`）；`0` QP 告警，`qp_backend p50/p95=0.321/0.965 ms`，MINCO `p50/p95=0.42/0.86 ms`，`retarget=0` |
| **T3** (`stageA_T3_d150`) | `rmuc_2027_tunnels / six_tunnels` | **通过**（`42/42`） | `42/42` 到达，里程 `74.04 m`，末点真值误差 `0.0025 m`（`post_stop=0.0053 m`），足迹冲突 `0`，`42/42` q1/q2 全过；**6/6 隧道全过**（`tunnel_ramp_bridge` 侧偏 `0.0162 m`、`tunnel_B` `0.0336 m`、`tunnel_south_plate` `0.0363 m`、`tunnel_south_plate_mirror` `0.0624 m`、`tunnel_B_mirror` `0.0295 m`、`tunnel_ramp_bridge_mirror` `0.0180 m`，全部接触增量 `0`）；`physical_contact_acceptance=passed`（`illegal=0`）；`0` QP 告警，`qp_backend p50/p95=0.285/0.801 ms`，MINCO `p50/p95/max=0.32/6.48/8.58 ms`，全程仅 `3` 次 retarget |
| **B1** (`stageA_B1_d153`) | `rmuc_2027_tunnels / base_to_base` | **通过** | 单 action `1/1` 到达，里程 `27.21 m`，末点真值误差 `0.0225 m`，足迹冲突 `0`，q1/q2 通过；峰值指令/真值速度 **`3.423 / 3.404 m/s`**（较 Wave 5 `1.389 m/s` 提升 `+145%`；`5 m/s` 上限未触发），峰值自旋 `1.556 / 1.515 rad/s`；`physical_contact_acceptance=passed`（`illegal=0`，消除原 `267` 未归因接触）；`0` QP 告警，`qp_backend p50/p95=0.336/0.709 ms`；**长段 MINCO `p50/p95/max=5.72/6.48/7.84 ms`**（原 `34.01/184.43 ms`，远低于 `< 20 ms` 预算），`retarget=0` |
| **V1** (`stageA_V1_d154`) | `rmuc_2027_tunnels / symmetry_speed` | **通过** | 单 action `1/1` 到达，里程 `15.91 m`，末点真值误差 `0.0294 m`，足迹冲突 `0`，q1/q2 通过；峰值指令/真值速度 **`3.164 / 3.053 m/s`**（较 Wave 5 `2.223 m/s` 提升 `+37%`；`5 m/s` 高速段未触发）；`physical_contact_acceptance=passed`（`illegal=0`）；`0` QP 告警，`qp_backend p50/p95=0.427/1.613 ms`，MINCO `p50/p95/max=5.06/5.06/6.89 ms` |
| **B2** (`stageA_B2_d156`) | `rmuc_2027_tunnels --prior rmuc_2027 --case b2_detour` | **通过** | 在线投放实体箱触发 `local_rejoin` 绕行到达，真值误差 `0.0058 m`，`physical_contact_acceptance=passed`（`illegal=0`），`qp_backend p50/p95=0.297/0.577 ms` |
| **F（5 项）** (`d157`/`d158`/`d169`/`d160`/`d161`) | `unknown` / `no_path` / `unsafe` / `adapter_lease` / `point_lio_restart` | **5/5 通过** | `unknown`（`stop=3.25 s`）、`no_path`（`2.13 s`）、`unsafe`（目标阻断，`3.57 s`）、OS `SIGSTOP`/`SIGCONT` `adapter_lease`（`stop=4.67 s`、`recovery=2.38 s`、`pub_seq 871->876`、旧授权拒绝）与 `point_lio_restart`（`death_to_stop=2.06 s`、`recovery=6.73 s`、`epoch 1->2`、旧授权拒绝）全过，且 5 项均 `physical_contact_acceptance=passed`（`illegal=0`） |
| **S3 连续修正回归** (`d175`/`d176`/`d177`) | `small_10x10 --case correction_0.05 / 0.15 / 0.30` | **3/3 通过** | `0.05 m`（`epoch 1->1`、`err=0.0137 m`）、`0.15 m`（`epoch 1->1`、`err=0.0020 m`）与 `0.30 m`（`epoch 2->3`、`err=0.0011 m`）均在 `10 s` 内 `< 0.002 m` 收敛并保持 `S3 endpoint retarget` 与 `odom` 连续性契约（注：`small_10x10` 合成地板名 `s3_floor` 未纳入 `rmuc_2027` 地面白名单，物理接触门以 2027 全套 11/11 实测为准） |

### 5.2 Stage B：无先验建图模式（`slam:=true`，`--mapless`，空先验）与 Wave 6B（统一 `0.05 m` + 模块化拆分）实测

2026-11-26 Wave 6B 完成全链统一 `0.05 m` 像素分辨率收敛（`node_params.yaml` 单一参数源）、实测 TF 雷达观测方向（`resolveLidarFovYawOffsetRad()`）接入、`ats_rog_map` 按需 3D ESDF 与增量平面缓存优化、死代码清理以及全部 17 个超长第一方非驱动源码文件按功能拆分为 `<= 1000` 行（`453` 个文件审校 `0` 超长）。以下为 Wave 6B 无窗口（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`）隔离域实测结果：

| 批次 | 入口与用例 | 结果 | 关键实测指标与边界 |
| --- | --- | --- | --- |
| **Stage A B1（最终二进制）** (`vfinal_stageA_b1_d194`) | `rmuc_2027_tunnels / base_to_base` | **通过** | `1/1` 到达，里程 `27.41 m`，末点真值误差 `0.0153 m`（停后 `0.0309 m`），`footprint_gate_collisions=0`，`nav_tracking=1/1`，`physical_contact_acceptance=passed`（`illegal=0`） |
| **Stage A B2（最终二进制）** (`vfinal_stageA_b2_d195`) | `rmuc_2027_tunnels --prior rmuc_2027 --case b2_detour` | **Action 通过；物理接触未过** | 绕行到达（真值误差 `0.0267 m`），但在 `5 cm` 全静态图 2D ESDF 投影周期（`~180 ms`）下于 `1.2 m` 投放距离出现 `illegal=528` 轮碰箱体（归入下一轮 Wave 6B-2 优化） |
| **Stage A T3（5 cm 拆分后）** (`final_stageA_t3_d232`) | `rmuc_2027_tunnels / six_tunnels --waypoint-control-tolerance-m 0.08` | **42/42 路点与 q1/q2 通过；物理接触未过** | `42/42` 到达（里程 `74.33 m`，末点 `0.0096 m`，`42/42` `nav_tracking`），左右侧壁独立 PCA 修复 `tunnel_B_mirror` 顶板接触至 `0`（`illegal_chassis=0`、`illegal_wheel_ceiling=0`、`illegal_max_penetration_m=0.0`）；仅 `south_plate`/`south_plate_mirror` 处 `18` tick 底板拼接缝余量触碰（`d=+0.79..+1.00 mm`，`nz=+0.38..+0.77`，归入下一轮 Wave 6B-2） |
| **Stage B 障碍延迟与保存** (`final_obs_dyn_d181` / `vfinal_obs_dyn_d196`) | `rmuc_2027 --mapless --max-goals 3 --post-route-case obstacle_and_dynamic` | **延迟与 3 点建图通过；`dynamic_save` 在 `d181` 全过、`d196` 清除等待略短** | `d181` 全套通过：`3/3`（末点 `0.0054 m`）、`contact=0`、`nav_tracking=3/3`、障碍可见延迟 **`p50=187.8 ms / p95=201.9 ms (< 250 ms) / max=202.8 ms`**、`dynamic_save passed=True`、`s3_map_quality`（`586/756` 占格命中，`8684` 持久点）；最终二进制 `d196` `3/3`（`0.00033 m`）、`contact=0`、`nav_tracking=3/3`、延迟 `p50=190.0 / p95=208.9 / max=209.3 ms`，但 `dynamic_save` 前等待 `3.5 s` 略短于 `dynamic_hold_seconds=3.0 s` 清除窗口 |
| **Stage B 重载首目标** (`final_reload_d182`) | `rmuc_2027 --reload d181/map --max-goals 1` | **通过** | 重载 `d181/map` 后 `1/1` 到达，里程 `4.33 m`，末点真值误差 **`0.0412 m <= 0.08 m`**，`contact=0`（`passed`），`nav_tracking=1/1` |
| **Stage B 建图 `no_path`（最终二进制）** (`vfinal_nopath_d199`) | `small_10x10 --mapless --case no_path` | **通过** | 有界无路失败与三路速度确定性归零通过（注：`small_10x10` 合成地板 `s3_floor` 未纳入 2027 地面白名单） |
| **Stage B 无先验 `B1_mapless`** (`s3_b1_mapless_d193` / `vfinal_b1_mapless_d198`) | `rmuc_2027_tunnels --route base_to_base --mapless` | **拆分前 `d193` 通过；最终二进制 `d198` 未通过** | 拆分前 5 cm 基线 `d193` 单 action `1/1` 到达（`79.84 m`，`0.0144 m`，`contact=0`，`nav_tracking=1/1`）；重构后最终二进制 `d198` 在 `(8.81, 9.00)` 触发 `suspended-replan wait`（`contact=30`），归入下一轮 Wave 6B-2 稳定收口 |

### 5.3 离线 Point-LIO A/B 评估（`scripts/evaluate_lio_frontend_ab.py`）

跨 `stageA_T1_d205`、`stageA_B1_d208`、`stageA_V1_d207`、`stageB_rmuc2027_save_d215` 实测：`APE RMSE = 0.0202–0.0784 m`，`APE p95 = 0.0288–0.1732 m`，`RPE-1m = 0.0088–0.0718 m`，`Spin p95 = 0.0294–0.1068 m`，`LIO p95 = 6.32–7.05 ms`。

### 5.4 未通过与未验证总界

- **未通过（归入下一轮 Wave 6B-2）**：
  1. `5 cm` 最终二进制下 Stage B `B1_mapless` 稳定通过（消除中台路缘/颠簸带与 `suspended-replan wait` 波动）及 `obstacle_and_dynamic` 中 `dynamic_save` 前等待/射线清除稳定性；
  2. `5 cm` 最终二进制下 Stage A `B2 detour` 零物理接触（消除 `~180 ms` 5 cm 全图 ESDF 投影导致的 `1.2 m` 投放距离擦碰）与 `T3 six_tunnels` 南侧悬空板通道底板拼接缝 `18` tick 余量触碰归零（并复验 Stage A 全套 11/11）；
  3. 完整 64 点 S3 `A–K`。
- **未验证**：`5 m/s` 高速段与高速换轨（当前 B1/V1 真值峰值已达 `3.404 / 3.053 m/s`，`5 m/s` 上限未触发）、开窗 RViz/MuJoCo 视觉画面（桌面锁屏未带窗口）、P3 行为树全链集成及实车硬件（含 H2/H3 钻洞）。
- 历史基线保留原则：2025 场景 `q2`、早期 2027 `south` 漂移与未复验的历史兼容项不因场景切换或局部通过自动标为已修复。
