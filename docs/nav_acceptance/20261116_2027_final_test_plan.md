# 2027 导航验证方案：基地到基地、隧道与对称点速度

更新日期：2026-11-24。本文件是 RMUC 2027 仿真导航验收的唯一任务坐标、执行入口与当前状态账本。

- 默认仿真入口为有图模式：`rmuc_2027_mujoco.launch.py` 默认 `slam:=false`，加载 `src/ats_sentry_bringup/map/rmuc_2027.yaml` 与 `pcd/rmuc_2027.pcd`；唯一 MPC 求解器为 `OSQP v1.0.0 SQP-RTI`（`solver_mode:=qp`）。
- 无先验建图测试显式传 `slam:=true`（`scripts/s3_mujoco_acceptance.py` 加 `--mapless`），PGM 和 PCD 输入置空，两种模式分开记录。
- 当前默认无窗口运行（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`）；未带窗口时画面项记为未验证，无独立接触评估器时 `physical_contact_acceptance` 记为 `unverified`。
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

## 5. 当前验收状态（2026-11-23 Waves 0–5 无窗口实测）

全部批次基于三仓最终提交（根仓 `241dc08`、导航仓 `db1b4e2`、MuJoCo 仓 `47b9194`），在独立隔离域下无窗口运行（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`；画面项、独立物理接触评估 `physical_contact_acceptance: unverified` 及实车硬件均记为未验证）。全部通过批次均满足单一发布者所有权、零 Nav2 断言与 `0` QP 告警。

### 5.1 Stage A：有图定位模式（`slam:=false`，`rmuc_2027`）

| 批次 | 入口与路线 | 结果 | 关键实测指标与边界 |
| --- | --- | --- | --- |
| **T1** (`stageA_T1_d205`) | `rmuc_2027_tunnels / tunnel_b_roundtrip` | **通过** | `13/13` 到达，里程 `21.03 m`，末点真值误差 `0.0792 m`，足迹冲突 `0`，`13/13` q1/q2 全过，接触增量 `0`；`tunnel_ramp_bridge`（正 1、侧偏 `0.0131 m`）与 `tunnel_B`（正/反各 1、侧偏 `0.0336 m`）全过；`qp_backend p50/p95/p99=0.662/1.482/1.759 ms`，MINCO `p50/p95=3.62/8.31 ms` |
| **T2** (`stageA_T2_d192`) | `rmuc_2027_tunnels / ramp_south` | **通过** | `13/13` 到达，里程 `21.58 m`，末点真值误差 `0.0756 m`，足迹冲突 `0`，`13/13` q1/q2 全过；`tunnel_ramp_bridge`（正 1、侧偏 `0.0190 m`）与 `tunnel_south_plate`（正/反各 1、侧偏 `0.0412 m`）全过；`qp_backend p50/p95=0.679/1.436 ms`；注：南侧顶板几何接触诊断增量 `46940`（独立接触待归因） |
| **T3** (`stageA_T3_d212`) | `rmuc_2027_tunnels / six_tunnels` | **未通过**（`16/42`） | 通过 `tunnel_ramp_bridge`（正 1）、`tunnel_B`（正/反各 1）、`tunnel_south_plate`（正 1），里程 `29.71 m`；第 17 点 `[16.0, 1.08, 3.1416]` action 成功（估计误差 `0.009 m`）但独立真值误差 `0.181 m > 0.15 m`，根因为南侧退化区 GICP Y 漂移（同期 Point-LIO odom Y 漂移仅 `+0.005 m`，且 `weak_translation_eigenvalue=25.0` 未触发退化抑制） |
| **B1** (`stageA_B1_d208`) | `rmuc_2027_tunnels / base_to_base` | **通过** | 单 action `1/1` 到达，里程 `28.26 m`，末点真值误差 `0.0206 m`，足迹冲突 `0`，q1/q2 通过；峰值参考/指令/真值 `1.751 / 1.392 / 1.389 m/s`，峰值自旋 `2.504 / 2.469 rad/s`；`qp_backend p50/p95=0.881/1.826 ms`；接触诊断增量 `267`（独立接触待归因）；长段 MINCO `p50/p95=34.01/184.43 ms`（超 `20 ms` 预算，待 P0-2 滚动裁剪） |
| **V1** (`stageA_V1_d207`) | `rmuc_2027_tunnels / symmetry_speed` | **通过** | 单 action `1/1` 到达，里程 `16.41 m`，末点真值误差 `0.0358 m`，足迹冲突 `0`，q1/q2 通过，接触增量 `0`；峰值参考/指令/真值 `2.549 / 2.236 / 2.223 m/s`（`5 m/s` 高速段未触发）；`qp_backend p50/p95=0.586/1.245 ms` |
| **B2** (`stageA_B2_detour_d209`) | `rmuc_2027_tunnels --case b2_detour` | **通过** | 在线投放实体箱触发 `local_rejoin` 绕行到达，真值误差 `0.0130 m`，接触增量 `0`，`qp_backend p50/p95=0.499/1.099 ms` |
| **F（5 项）** | `unknown` / `no_path` / `unsafe` / `adapter_lease` / `point_lio_restart` | **5/5 通过** | `unknown`（`stop=3.50 s`）、`no_path`（`2.11 s`）、`unsafe`（目标阻断，`7.02 s`）、OS `SIGSTOP`/`SIGCONT` `adapter_lease`（`stop=4.75 s`、`recovery=2.95 s`、`pub_seq 503->508`、旧授权拒绝）与最终二进制 `point_lio_restart`（`death_to_stop=2.02 s`、`recovery=7.92 s`、`epoch 1->2`、旧授权拒绝）全过且接触增量 `0` |

### 5.2 Stage B：无先验建图模式（`slam:=true`，`--mapless`，空先验）

| 批次 | 入口与用例 | 结果 | 关键实测指标与边界 |
| --- | --- | --- | --- |
| 3 点建图与保存 (`stageB_rmuc2027_save_d215`) | `rmuc_2027 --mapless --max-goals 3 --post-route-case obstacle_and_dynamic` | **3/3 路点与保存通过；后置延迟超时** | 3/3 到达（真值误差 `0.0213 / 0.0352 / 0.0270 m`，里程 `8.04 m`，`3/3` q1/q2，足迹冲突 `0`，接触 `0`，`qp_backend p95=1.461 ms`），显式保存 `167` 占据格 / `3858` 持久体素；后置射线清除超时导致整脚本失败 |
| 无先验 B2 与故障 (`d217`/`d219`/`d220`) | `b2_detour` / `unknown` / `point_lio_restart` | **3/3 通过** | `b2_detour` 真值误差 `0.0143 m`（`qp_backend p95=0.887 ms`）；`unknown` `stop=3.36 s`；`point_lio_restart` `death_to_stop=1.90 s`、`recovery=6.08 s`、`epoch 1->2`、旧授权拒绝；三项接触均为 `0` |
| 其余 S3 开放门（6 项） | `reload` / `B1_mapless` / `obstacle_latency` / `dynamic_save` / `no_path` / `route_save` | **未通过** | `reload`（锁定 `TRACKING`/`ready` 但部分图无路）；`B1_mapless`（A3 探索前进 `11.39 m`、定位误差 `1.8 cm`，北侧凹槽看门狗超时 `code=5`）；`obstacle_latency`（墙钟 `476–910 ms`，第 7 样本射线清除超时）；`dynamic_save`（静态持久占据证据 `0`）；`no_path`（建图前沿推进未立即停车）；`route_save`（首点孤立 `2×2` 未知格卡滞） |

### 5.3 离线 Point-LIO A/B 评估（`scripts/evaluate_lio_frontend_ab.py`）

跨 `stageA_T1_d205`、`stageA_B1_d208`、`stageA_V1_d207`、`stageB_rmuc2027_save_d215` 实测：`APE RMSE = 0.0202–0.0784 m`，`APE p95 = 0.0288–0.1732 m`，`RPE-1m = 0.0088–0.0718 m`，`Spin p95 = 0.0294–0.1068 m`，`LIO p95 = 6.32–7.05 ms`。

### 5.4 未通过与未验证总界

- **未通过**：有图 **T3 `six_tunnels`**、B1 长段 MINCO `< 20 ms` 墙钟门、Stage B 无先验 6 项 S3 开放门及完整 64 点 S3 `A–K`。
- **未验证**：`5 m/s` 高速段与高速换轨、独立物理接触评估（`physical_contact_acceptance: unverified`）、开窗 RViz/MuJoCo 视觉画面、P3 行为树全链集成及实车硬件（含 H2/H3 钻洞）。
- 历史基线保留原则：2025 场景 `q2`、早期 2027 `south` 漂移与未复验的历史兼容项不因场景切换或局部通过自动标为已修复。
