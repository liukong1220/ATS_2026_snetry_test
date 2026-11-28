# 2027 导航验证方案：基地到基地、隧道与对称点速度

更新日期：2026-11-28。本文件是 RMUC 2027 仿真导航验收的唯一任务坐标、执行入口与当前状态账本。

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

### 5.2 Wave 6B-2（统一 `0.05 m` + 启动 YAML 参数化 + 双阶段收口）终态二进制无窗口实测（2026-11-28）

2026-11-28 Wave 6B-2 在统一 `0.05 m` 分辨率与全部第一方非驱动源码 `<= 1000` 行（最高 `999` 行，0 超长）基础上，完成事件驱动局部 2D ESDF 投影（消除 Stage A 5 cm `~180 ms` 全静态图重算瓶颈，压降至 `p95=20.55 ms`）、`ground_column` / `terrain_analysis_ext` 路缘与颠簸带判据修复、12 份启动文件自动建图与定位默认值 YAML 参数化（`launch_defaults.yaml` / `mujoco_launch_defaults.yaml`）、MPC 剩余平移终段限速与三轴 `kBrake`、MuJoCo 舵角有限界控制。以下均为同一冻结终态二进制在独立隔离域下无窗口（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`）实测结果（每轮前后均校验零残留进程）：

| 批次 | 入口与用例 | 结果 | 关键实测指标与边界 |
| --- | --- | --- | --- |
| **Stage A B2** (`vfinal_stageA_b2_d171`) | `rmuc_2027_tunnels --prior rmuc_2027 --route base_to_base --case b2_detour` | **通过** | 动态绕行到达，真值误差 **`0.0037 m`**，最小箱距 `0.753 m`，绕行墙钟 `14.55 s`，`A=True`，**`physical_contact_acceptance=passed`（`illegal=0`）**（较 6B-1 `contact=528` 彻底归零） |
| **Stage A B1** (`vfinal_stageA_b1_d172`) | `rmuc_2027_tunnels --prior rmuc_2027 --route base_to_base --case route` | **通过** | `1/1` 到达，里程 `27.00 m`，末点真值误差 `0.0249 m`（停后 `0.0309 m`），真值峰值速度 **`3.561 m/s`**，`footprint_gate_collisions=0`，`nav_tracking=1/1`，`contact=0`（`passed`），`hd/qp=0/0` |
| **Stage A V1** (`vfinal_stageA_v1_d173`) | `rmuc_2027_tunnels --prior rmuc_2027 --route symmetry_speed --case route` | **通过** | `1/1` 到达，里程 `15.42 m`，末点真值误差 `0.0151 m`（停后 `0.0474 m`），真值峰值速度 **`3.148 m/s`**，`footprint_gate_collisions=0`，`nav_tracking=1/1`，`contact=0`（`passed`），`hd/qp=0/0` |
| **Stage A T1** (`vfinal_stageA_t1_d174`) | `rmuc_2027_tunnels --prior rmuc_2027 --route tunnel_b_roundtrip --waypoint-control-tolerance-m 0.08` | **通过** | `13/13` 到达，里程 `20.79 m`，末点真值误差 `0.0118 m`（停后 `0.0115 m`），`tunnel_ramp_bridge`/`tunnel_B` 接触增量均为 `0`，`13/13` `nav_tracking`，`contact=0`（`passed`），`hd/qp=0/0` |
| **Stage A T2** (`vfinal_stageA_t2_d175`) | `rmuc_2027_tunnels --prior rmuc_2027 --route ramp_south --waypoint-control-tolerance-m 0.08` | **路点与 q1/q2 通过；物理接触未过** | `13/13` 到达（`21.11 m`，`0.0105 m`，`13/13` `nav_tracking`，`footprint_gate=0`），`tunnel_ramp_bridge=0`；仅 `tunnel_south_plate` 在 `y=1.0787..1.0792` 触发 `10` tick 零侵入底板接缝余量触碰（`audited_roof_8737`，`d=+0.52..+0.85 mm`，`nz=0.506..0.717`，`penetration=0.0`） |
| **Stage A T3** (`vfinal_stageA_t3_d184`) | `rmuc_2027_tunnels --prior rmuc_2027 --route six_tunnels --waypoint-control-tolerance-m 0.08` | **42/42 路点与 q1/q2 通过；物理接触未过** | `42/42` 到达（`73.78 m`，`0.0057 m`，`42/42` `nav_tracking`，`footprint_gate=0`，MINCO `p95=5.90 ms`）；`ramp_bridge`/`south_plate`/`B_mirror`/`ramp_bridge_mirror` 4 洞为 `0`；`tunnel_B` `28` tick（`y=1.988`，`d=-3.1 mm`）与 `south_plate_mirror` `19` tick（注：22 点收紧参数改动前同轮 `stageA_t3_d161` 曾 `42/42` `contact=0` 全过，归入 Wave 6B-3 统一收敛） |
| **Stage A 故障 `unknown` / `adapter_lease`** (`d179` / `d182`) | `rmuc_2027_tunnels --prior rmuc_2027 --route base_to_base --case {unknown,adapter_lease}` | **2/2 通过** | `unknown`（`stop_wall_s=3.124 s`，`contact=0`）与 `adapter_lease`（`stop=5.369 s`，`recovery=0.259 s`，`old_execution_rejected=True`，`contact=0`）均满足急停与三路零速 |
| **Stage A 故障 `no_path` / `unsafe` / `point_lio_restart`** (`d180` / `d181` / `d183`) | `rmuc_2027_tunnels --prior rmuc_2027 --route base_to_base --case {no_path,unsafe,point_lio_restart}` | **未通过（根因已定位，归入 6B-3）** | `no_path`：搜索已耗尽 `95771` 节点并停车零接触，但 5 cm 全图搜索（`~110 ms`）慢于 `~100 ms` 快照发布导致上报 `FAILURE_SNAPSHOT_CHANGED (11)` 而非 `FAILURE_NO_PATH (6)`；`unsafe`：`+0.28 s` 已急停，但快速投影在 `+0.58 s`（早于 `0.40 s` 持续零速窗结束）即完成开阔地绕行重规划并擦箱；`point_lio_restart`：击杀后急停零接触通过，但重启的 `pointlio_mapping` 在 `sim_s=4.63 s` 仅打印参数后 30 s 无里程计输出（`epoch` 未递增） |
| **Stage B `B1_mapless`** (`vfinal_stageB_b1_mapless_d176`) | `rmuc_2027_tunnels --route base_to_base --mapless --case route` | **通过** | 单 action `1/1` 到达，里程 `36.85 m`，末点真值误差 **`0.0128 m`**（停后 `0.0432 m`），峰值 `2.416 m/s`，`footprint_gate=0`，`nav_tracking=1/1`，**`contact=0`（`passed`）**，`hd/qp=0/0` |
| **Stage B 障碍延迟与动态保存** (`vfinal_stageB_obs_dyn_d177`) | `rmuc_2027 --mapless --max-goals 3 --post-route-case obstacle_and_dynamic` | **通过** | `3/3` 到达（`7.72 m`，末点 `0.0079 m`，`3/3` `nav_tracking`，`contact=0`），障碍可见延迟 **`p50=186.9 / p95=196.3 (< 250 ms) / max=197.2 ms`**，**`dynamic_save passed=True`**（移除前 `166 pts / 61 cells` -> 移除后 `0 pts / 0 cells`） |
| **Stage B 重载首目标** (`vfinal_stageB_reload_d178`) | `rmuc_2027 --reload d177/map --max-goals 1` | **通过** | 重载后 `1/1` 到达，里程 `4.20 m`，末点真值误差 **`0.0196 m`**（停后 `0.0079 m`），`nav_tracking=1/1`，`contact=0`（`passed`） |
| **Stage B 建图 `no_path`** (`final_nopath_mapless_d170`) | `small_10x10 --mapless --case no_path` | **通过（耗时待压回）** | `passed=True`、`A=True`、有界失败与三路零速通过；但 `stop_wall_s=8.404 s`（受 `isLocalDirectNoPathSealed` 放宽 `lat<=3.50 m` 影响，较改动前 `1.034 s` 变慢，归入 6B-3 压回） |
| **Stage B 22 点建图巡航 + 重载** (`final_mapping_22pt_d168` / `d169`) | 22 点无先验建图（Phase 1 17 点 + 保存 + Phase 2 5 点）及 `--reload` | **Phase 1、质量与重载通过；Phase 2 未全过** | Phase 1 `17/17` 到达（`210.1 s` sim / `262.9 s` wall `< 420 s`，前 18 点 `contact=0`），保存成功（`6067` 栅格占据格 / `30428` 持久体素），`s3_map_quality`（`tp=4532`、`fp=1535`、位置误差均值/p95=`0.0412/0.0814 m`），`reload` `d169` 通过（`0.0143 m`，`contact=0`，`1/1` `nav_tracking`）；总计 `21/22`（Phase 2 WP 19 因 `tunnel_south_plate_mirror` 内参考失效引发 `known_only=0/1` 振荡超时 `121 s` 且 `6` tick 轮侧触碰；WP 20 在 `y=14.948` 出现 `10` tick 零侵入接缝触碰） |

### 5.3 离线 Point-LIO A/B 评估（`scripts/evaluate_lio_frontend_ab.py`）

跨 `stageA_T1_d205`、`stageA_B1_d208`、`stageA_V1_d207`、`stageB_rmuc2027_save_d215` 实测：`APE RMSE = 0.0202–0.0784 m`，`APE p95 = 0.0288–0.1732 m`，`RPE-1m = 0.0088–0.0718 m`，`Spin p95 = 0.0294–0.1068 m`，`LIO p95 = 6.32–7.05 ms`。

### 5.4 未通过与未验证总界

- **未通过（归入下一轮 Wave 6B-3）**：
  1. Stage A 5 cm `T2`（`10` tick 零侵入接缝）与 `T3`（`47` tick，`tunnel_B` 侧偏 + `south_plate_mirror` 接缝）稳定零接触复验（合并恢复 `d161` `42/42` 零接触与 22 点收紧参数）；
  2. Stage A 3 项独立故障：`no_path`（5 cm 搜索跨快照被覆盖为 `FAILURE_SNAPSHOT_CHANGED`）、`unsafe`（快投影 `0.58 s` 绕行打断 `0.40 s` 持续零速窗与 `0.80 m` 擦箱）、`point_lio_restart`（中途重启 `pointlio_mapping` 后 30 s 无里程计输出）；
  3. Stage B 22 点建图巡航 Phase 2（WP 19 `known_only` 振荡与 WP 20 接缝 `10` tick）及 `small_10x10 --mapless --case no_path` 停车耗时压回 `~1 s`；
  4. 完整 64 点 S3 `A–K`。
- **未验证**：`5 m/s` 高速段与高速换轨（当前 5 cm 终态二进制 B1/V1 真值峰值已达 `3.561 / 3.148 m/s`，`5 m/s` 上限未触发）、开窗 RViz/MuJoCo 视觉画面（桌面锁屏未带窗口）、P3 行为树全链集成及实车硬件（含 H2/H3 钻洞）。
- 历史基线保留原则：2025 场景 `q2`、早期 2027 `south` 漂移与未复验的历史兼容项不因场景切换或局部通过自动标为已修复。
