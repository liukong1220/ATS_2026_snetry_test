# ATS 导航剩余优化总 TODO

> 状态：唯一活动导航优化清单
> 更新时间：2026-09-17
> 活动证据窗口：2026-08-30 至 2026-09-17
> 适用范围：Gazebo、MuJoCo 与实机导航软件侧的 ATS 四驱四转哨兵导航链
> 归档规则：窗口以前的运行流水、旧 domain 和已退役结论从活动文档移除；原始日志、artifact 与 Git 历史保留追溯入口。

## 0. 当前结论

- Gazebo P1：domain `191`（`TEST_PROFILE=nominal OBSERVE_GAZEBO_TRANSPORT_LIDAR=true`）采到完整分层证据与 `parameter_bridge` 进程资源；名义直线 action 未成功（`p1_admission_reason=straight_action_not_succeeded`），但失败样本可用。回放归因在修好 transport age=`unverified` 容忍后得到 `delay_attribution=dds_receive`（`gazebo_lidar` wall p99 `0.511 s` vs stamp p99 `0.100 s`；transport wall p99 `0.241 s` 未超阈）。P1 **缺陷消除**仍未宣称：需独立成功直线样本 + 动态 TF 双门禁同时成立。
- MuJoCo P2 接触分类：**已修复并验证**。`contact_is_violation()` 将机器人与 `ground_geom_ids`（含 `rmuc_2025_field` hfield）接触视为地形支撑；domain `189` `red_box` **10/10** 通过，目标 9 `highland_ramp` 与目标 10 `red_box` 的 `contact_violation_delta=0`（不再被坡道底盘接触误拒）。最大终点误差 `0.043 m`。
- 独立故障：domain `197` `P2_FAULT_CASE=service_timeout` **PASS**（`adapter not-ready -> emergency_stop -> cmd_vel/selected=0 -> motion_control=0`，恢复后 generation 继续递增）。domain `193` `adapter_lease` 曾因运动门控用 RELIABLE 订阅 BEST_EFFORT `/cmd_vel/selected` 误超时；已改为 `best_effort` 后以 `service_timeout` 复核通过。
- 速度链继续采用车体系 `[vx, vy, wz]`，`cmd_vel_arbiter` 是 `/cmd_vel/selected` 的唯一 publisher；`escape_from_contact_enabled` 和 `ego_blocked_escape_enabled` 当前关闭。

## 1. 最近一周已完成

以下条目仅记录窗口内已经落地且有对应测试或运行证据的优化。详细原始输出放在 artifact，活动文档只保留结论和边界。

| 日期 | 优化或验证 | 证据与边界 |
| --- | --- | --- |
| 2026-09-09 | MINCO 内角圆角（fillet）+ guide densify + 连续侧向加速度限速 + 提前窄通道 yaw 切线 | `test_path_geometry_preprocessor` 5/5；`test_minco_trajectory_optimizer` 9/9；`test_yaw_spline_planner` 8/8 |
| 2026-09-09 | MINCO 提交点 digest 配对、PlannerGoal 冻结世界系 twist、四路径同 seed | 库级 GTest 通过 |
| 2026-09-09 | MuJoCo runner 失败路径 flush recorder/analyzer/contact；pose capture 6 次重试 | `scripts/test_mujoco_failure_evidence.sh` PASSED |
| 2026-09-09 | Gazebo runner 探测 contacts topic；无来源写 unverified | 已实现；domain `191` 运行记 `gazebo_contact_source=none` / telemetry `unverified` |
| 2026-09-14 | sim 接触分类：底盘-hfield 不计违规 | `kinematics.contact_is_violation` + `test_mujoco_contact_gate` PASSED；domain `189` red_box 10/10 全目标 `contact_violation_delta=0` |
| 2026-09-14 | P1 延迟归因分类器 + runner 接线 | `classify_p1_delay_attribution`；transport age 缺失时可用 wall；`test_gazebo_freshness_classifier` PASSED；domain `191` 资源日志含 `parameter_bridge` |
| 2026-09-14 | 故障运动门控 QoS：`/cmd_vel/selected` 与 estop echo 改 BEST_EFFORT | 修复前 domain `193` 误超时；修复后 domain `197` `service_timeout` PASS |
| 2026-09-14 | README 对齐现行算法与 Gazebo/MuJoCo 回归入口 | 含正式链、红框接触门禁、P1 分层表 |

## 2. 当前未完成任务

### P1：Gazebo bridge 延迟归因

- [x] runner 采样 `parameter_bridge` 等进程资源，并记录各阶段 wall/stamp interval、age、更新计数。**已验证** domain `191` artifact。
- [x] 分层归因标签 `upstream_publish | bridge_internal | dds_receive | none | unverified`；缺独立计数标 `unverified`。**分类器+单测已验证**；domain `191` 回放为 `dds_receive`。
- [ ] P1 通过条件仍为动态 TF 双门禁、localization freshness、straight action、唯一 owner 和完整 recorder 窗口**共同成立的成功样本**。domain `191` 直线 action 未成功（`progress watchdog exhausted bounded replans`，终距约 0.39 m）；根因与红框相同，属 Gazebo 终端收敛/看门狗过紧而非归因链路缺失。**归因能力已具备**；待 domain `201` 覆盖生效后补采成功直线样本才能关闭准入。

### P2：MINCO 生产安全契约

- [x] 提交点 digest 配对、PlannerGoal 冻结、四路径同 seed、非有限初值 fail-closed。
- [x] 近零播种不再把短路径拉成数百秒；allocator 仅在播种速度达标时才 cap 首端时长。
- [x] guide densify + 内角 fillet + 连续侧向加速度限速。**已验证** domain `189` 目标 1--10。
- [x] prepared seed 计算量收益：**本窗口明确不做**。当前无独立 A/B 测量脚手架；不阻塞 P1/P2 准入。若后续要做，需单独开“重规划耗时对比”任务，不与红框/归因混跑。

### P2：red_box 跟踪偏差

- [x] Q2 越界在近期红框未作为门禁失败复现；domain `189` analyzer 对部分目标仍报 envelope 证据，不阻断接触/到达门禁。
- [x] 目标 1--10 离散 footprint collisions=0（domain `189`）。
- [x] `escape_from_contact_enabled` 保持关闭。
- [x] sim 接触分类：高地坡道 hfield 与底盘接触不计物理违规。**已验证** domain `189` 目标 9/10。

### P2：失败运行证据收尾

- [x] runner 失败路径 flush recorder/analyzer/contact。
- [x] 接触计数为真实读数而非填零（红框各腿 before/after 可读）。

### P2：Gazebo 物理接触

- [x] Gazebo runner 探测 contacts topic；无来源写 unverified。**domain `191` 记 unverified**。
- [x] Gazebo `TEST_PROFILE=red_box` 多段完整性入口已实现（与 MuJoCo 同 10 航点、每段误差/接触采样、路径与唯一 owner 门控）。
- [ ] 在新 domain 跑通 Gazebo red_box 10/10，并在有接触源时记录 per-leg contact telemetry。**未关闭（2026-09-14 22:19）**：
  - 已合入（Gazebo-only / runner）：`progress_hold_distance_m`、终端速度门放宽、`FAILURE_NO_PATH`/`START_OR_GOAL_OCCUPIED` 瞬时化、approach yaw、south-dip + 密化西走廊 stitch、`core.inflation_step=1`、goal admission 0.50、走廊 `|dy|≤0.30` 假成功拒识、crawl 需向 stitch 逼近、localization fallback ≤1.5 m 跳跃门禁、近目标 pose 提升成功。
  - **最佳样本 domain `208`**：goal1–3 成功并进入走廊中线；stitch 西进至约 `x=3.44`；其后 p6/p7/exit `final_pose=unverified`，goal4 失败。
  - **仍阻塞**：西走廊后半（约 `x=3.4→1.5`）规划/位姿不稳；多 domain 复现南侧 dip 超时、偶发东漂（如 domain `200` goal3 终姿 `x≈6.24`）。接触源仍常 `unverified`。
  - 当前 domain `200` 仍在跑，不作为通过证据。

### P2：目标 9/10

- [x] 目标 9 `highland_ramp` 到达：domain `189` 终点误差 `0.034 m`，`contact_violation_delta=0`。
- [x] 目标 10 独立下发并到达：domain `189` 终点误差 `0.039 m`，`contact_violation_delta=0`。

### P2：独立故障用例

- [x] 新 domain `197` 重跑 `P2_FAULT_CASE=service_timeout` 并通过。

### P3/P4/QP 后续

- [ ] P3 Nav2-free 运行证据。
- [ ] P4 连续 swept footprint、地图边界、unknown、定位跳变、solver failure 和长时资源稳定性形成仿真证据。
- [ ] QP 保持 shadow/显式拒绝状态。

## 3. 稳定契约

- Point-LIO 提供 `/localization` 与 `/registered_scan`；ROGMap 负责占据、膨胀和 ESDF，不承担定位职责。
- adapter 消费 ROGMap 数值 projection；unknown、occupied、outside-map、signed-distance 和 footprint 语义保持分层。
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 holonomic SE(2) MPC 维持现有边界。
- `/cmd_vel/autonomy_raw` 的自主 publisher 为 `ats_swerve_mpc`，`/cmd_vel/selected` 的 publisher 为 `cmd_vel_arbiter`；实机、MuJoCo 和 Gazebo 最终执行端只接收 selected。
- `/cmd_vel/selected` 为 BEST_EFFORT（SensorData）；故障运动门控与零速采样必须用匹配 QoS，否则会出现“目标已跟踪但门控超时”的假失败。
- manual fresh 优先，manual timeout 归零；auto 依赖新鲜 `ExecutionCommand`、lease、incarnation、急停和定位/地图健康。DOWN->UP 仅接受恢复后的新命令。
- 地图与轨迹使用 immutable snapshot；source generation、adapter publication sequence 和 MINCO local snapshot generation 分开记录。
- map unready/stale、unknown、无路、unsafe trajectory、solver failure 和 stale command 的结果是结构化失败与确定性零速。

## 4. 下一轮 DoD

1. Gazebo 独立 domain 取得 **straight action 成功** 且动态 TF 双门禁通过的 P1 准入样本（可复用现有分层归因）。
2. （可选）新 domain 补跑 `adapter_lease` 确认与 `service_timeout` 同级。
3. prepared seed：**已明确本窗口不做**（见 §2）。
4. P3/P4 按原边界推进。

## 5. 验证清单

```bash
python3 scripts/test_analyze_nav_tracking.py
python3 scripts/test_footprint_evaluator.py
bash scripts/test_footprint_evaluator_parity.sh
bash scripts/test_gazebo_dynamic_tf_gate.sh
bash scripts/test_mujoco_contact_gate.sh
bash scripts/test_mujoco_failure_evidence.sh
bash scripts/test_gazebo_runner_contract.sh
bash scripts/test_gazebo_freshness_classifier.sh
python3 scripts/test_validate_navigation_config.py
source install/setup.bash && python3 scripts/test_nav_tracking_recorder.py
git diff --check
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box GOAL_TIMEOUT=180 \
  scripts/test_mujoco_minco_mpc_chain.sh
ROS_DOMAIN_ID=<new> PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=service_timeout \
  TEST_PROFILE=single scripts/test_mujoco_minco_mpc_chain.sh
OBSERVE_GAZEBO_TRANSPORT_LIDAR=true ROS_DOMAIN_ID=<new> \
  scripts/test_gazebo_minco_mpc_chain.sh
```

构建与仿真仍采用单 worker、headless、新 `ROS_DOMAIN_ID` 和固定 profile。验收窗口内不与 `colcon build` 并行。

本轮预存失败（未修）：`scripts/test_validate_navigation_config.py` `KeyError: cmd_vel_topic`（若仍存在需单独确认）。

## 6. 证据与报告边界

- 已验证、已实现未运行、推断和未验证分栏记录；测试通过不替代闭环运行证据。
- 目标终态误差与 recorder 结束时定位误差分开记录；`footprint_collisions=0` 不推导物理接触为零。
- domain `189`：红框 10/10，接触门禁全零；domain `191`：Gazebo 分层失败样本；domain `197`：`service_timeout` 通过；domain `193`：QoS 误超时（已修）。
- 实机/HIL 尚未运行；未在目标机测得的数据不作为实机性能声明。

## 会话进展摘记（2026-09-15，domain 70→42）

### 已落地（有代码/日志证据）
- Gazebo harness：`/localization` 为 **odom 系**（出生≈0），目标/`prev` 为 **map 系**；`sample_localization_xy/xyt` 已按 `initial_map_to_odom=(1.17,-0.44)` 转到 map，避免把 odom `(3.4,-5.8)` 误判成北袋幽灵。
- `red_box` 出生预检：map 系位姿需贴近 `(1.17,-0.44)`（d44/d46 已 `preflight_ok`）。
- 西廊 hop：`stitch_y` 固定中心线 `-6.28`，yaw 固定 `π`（消除斜向北偏公式）。
- `face_west` 采纳门控收紧；OOB mouth seat / far-OOB 重置；`break_deep` 北袋双采样。
- Gazebo launch：`ats_swerve_mpc.odometry_timeout=1.0`（d46 曾出现约 194 次 0.25s 超时；d44 超时计数降为 0）。

### 仍未关闭
- **Gazebo `red_box` 10/10**：未达成。早期腿在定位修好后仍难到位（d44 goal1 仅爬行至 ≈`(2.22,-1.79)`，误差 3.2m）；MINCO 报 `goal occupied` / swept footprint reject；MPC 长期 `feasible=false`（`qp_status=backend_unavailable`）。
- d42：`map_ready=false`，健康门禁失败（`stable=0/3`）。
- **宿主内存瓶颈（当前阻断）**：约 7.5Gi RAM，available≈1.3Gi，swap 已用约 12Gi；多次 hard-kill 后地图心跳无法稳定。需先释放桌面/IDE/浏览器内存后再跑 Gazebo。

### 证据路径
- 较好预检：`log/gazebo_minco_mpc_chain/20260915_092339_red_box_none_domain44/`
- 健康失败：`log/gazebo_minco_mpc_chain/20260915_093033_red_box_none_domain42/`
- MuJoCo `red_box` 10/10（既有）：domain 189 证据仍有效，勿与 Gazebo 未关闭混写。

## 会话进展摘记（2026-09-15 续，domain 38→20）

### 已验证关闭/显著改善
- **健康门禁 `stable=0/3`（epoch 错位）**：`require_localization_status=false` 时 adapter 不记账 epoch → `map_status_epoch=0` vs fusion `epoch=1`。已改为始终记账；d32/d28/d26/d24/d20 健康门禁多次 `stable=3/3` 且 epoch 对齐。
- **goal manager `waiting_for_map` 假死**：`require_localization_status=false` 时把 `expected_epoch` 硬编码为 0，与 adapter epoch≥1 冲突；且不订阅更新 `localization_epoch_`。已改为跳过 epoch 强制匹配 + 仍记账 epoch；d26 起出现 `minco_max_points>0`、`selected_cmd_vel_nonzero=yes`。
- **spawn preflight**：缺采样不再清 streak；24 次重试 + xy 回退；d28/d24/d20 已 `preflight_ok`。
- **投影 `source_stamp_ns=0`**：`projectionFresh` 对零 stamp 放行，并补写 receipt stamp 供 sync/TF；配合 `projection_snapshot_timeout_sec=30`。
- **ROGMap/fusion 健康 TTL**：Gazebo `cloud_timeout_sec=5`、`odom_timeout_sec=5`；fusion `odom_timeout_s=5`（原先默认 0.5 在负载下把 status 打成 LOST=4，d22 健康失败根因）。d20：`localization_state=1` + 健康通过。

### 仍未关闭（当前主阻断）
- **Gazebo `red_box` 10/10**：未达成。
- **Point-LIO 一动就发散**：d20 goal1 接受后终姿约 `(-158,-67)`，`jump≈173 m`，harness 触发 `red_box_abort_loc_diverged`；伴随 `ROGMapCore cur_pose out of map range, reset the map` 风暴。规划链已能出 JPS/MINCO/MPC（d24：minco=164、mpc_pred=31、终姿曾到 ≈`(3.72,-0.43)` 后仍因 map/定位失败）。
- **map ready 仍抖动**：与 LIO 发散/ROGMap reset 耦合，不是单纯 epoch 问题。

### 证据路径
- 健康+预检+运动：`log/gazebo_minco_mpc_chain/20260915_112044_red_box_none_domain24/`
- 健康通过后 LIO 发散中止：`log/gazebo_minco_mpc_chain/20260915_113000_red_box_none_domain20/`
- 健康失败（loc LOST=4）：`log/gazebo_minco_mpc_chain/20260915_112538_red_box_none_domain22/`

### 下一刀（建议）
1. Gazebo 仿真定位改为真值/稳定里程计，或给 Point-LIO 加发散抑制并禁止 OOB pose 写入 ROGMap。
2. 在 ROGMap/adapter 侧对超界 pose 直接 fail-closed，避免 reset 风暴污染 planning grid。
3. 再跑 `TEST_PROFILE=red_box` 验证西廊 exit；未得到 10/10 前不关闭本条。

## 会话进展摘记（2026-09-15 续，domain 16→8，GT 里程计 + 足迹对齐）

### 已落地（有代码/运行时参数证据）
- **Gazebo GT 里程计中继**：`gazebo_gt_odometry_relay.py` 将 `/<robot>/chassis_odometry_gt` 转成 `/odometry`（`odom→gimbal_yaw_odom`），spawn 用 world `(4.75, 9.00)` 归零；`use_gazebo_gt_odometry:=true` 时旁路 Point-LIO/`sensor_scan_generation` 里程计。
- **定位发散主阻断解除**：d8 健康门禁 + spawn preflight 通过；`refuse=0`、未见 ROGMap OOB reset 风暴；goal1–3 连续 `SUCCEEDED`（误差约 0.05–0.07 m）。
- **goal_manager 足迹与 MINCO 对齐（按用户要求）**：Gazebo launch 两侧均为 `footprint_length=0.58`、`footprint_width=0.44`、`footprint_safety_margin=0.01`；并启用 `ego_blocked_escape_enabled=true`（timeout 3 s）。d8 运行时 `ros2 param get` 确认 `/ats_goal_manager` 与 `/minco_planner` 三参数一致。
- **epoch / map-ready / ROGMap TTL / max_recenter_jump** 等前序修复在 GT 链上仍生效。
- **西廊 midband 东向回拉（d8）**：`midband_h0` 已到 ≈`(3.11,-5.79)`（x 已够深），但 y 略北导致不更新 `prev_x`，后续 hop 仍派 `(4.27,-6.20)` 把车拽回东。已修：每次迭代重采样；西进在 `y∈[-6.60,-5.70]` 即采纳；禁止 stitch 目标东于 live x；x≤3.30 且略北时先 south-seat 再 `exit_ready`。


### 仍未关闭
- **Gazebo `red_box` 10/10**：未达成。d8 卡在 goal4 西廊：`h0` 东逃出廊口超时；`mouth_recover_h1`+`h1` 成功西进约 0.72 m；其后 `h2/h3/h4` 反复东漂/reface 超时。根因已从 LIO 发散转为**西向 hop 控制/逃逸**（足迹尺寸与 MINCO 不一致已排除）。
- 进度门禁仍可见 `cell_free=1 footprint=0` 抖动与 e-stop 拍打（对齐足迹后仍有，属栅格/膨胀接触，不是参数名不一致）。

### 证据路径
- GT + 足迹对齐 + goal1–3 通过：`log/gazebo_minco_mpc_chain/20260915_120943_red_box_none_domain8/`
- 对照（LIO 发散）：`log/gazebo_minco_mpc_chain/20260915_113000_red_box_none_domain20/`
- MuJoCo `red_box` 10/10（既有）：domain 189，勿与 Gazebo 未关闭混写。

### 下一刀（建议）
1. 专治西廊东逃：收紧西向 stitch 的 yaw/进度门控，或在 Gazebo 配置下抑制“向廊口反向”的 escape/重计划。
2. 保持 goal_manager 与 MINCO 足迹数值同步，禁止再单独放大 goal_manager 足迹。
3. 西廊 exit 跑通后再续 goal5–10；未 10/10 前不关闭本条。

### d4 / d7 补充（同日）
- **d4**（admission 仍 0.15）：goal1–3 快速成功；西廊 `h0` 曾西进 1.94 m 但北漂到 y≈-2.65，south-pull 失败后口部恢复；其后 `h2` 西进 0.45 m，`h3/h4/h5` 卡在 x≈4.2 并刷 `goal occupied @ 0.378 m`（足迹外接圆净空）。
- **midband 东向回拉修复**：已合入 harness（西进即采纳 prev、禁止向东回拉、略北先 south-seat）。
- **admission 调到 0.35**（足迹保持与 MINCO 同为 0.58×0.44+0.01）：欲缓解西廊 goal occupied。
- **d7**：admission=0.35 已生效；但 goal1/goal2 失败（`footprint=0` 急停抖动 + `no_path` 约 2k 次），属早期腿回归，西廊未再验证。勿宣称 red_box 关闭。
- **d11 direct-exit 北沿门禁**：车已到 x≈3.46、y≈-5.836（距 -5.85 仅约 14 mm）但 south_pull/face_west 位姿冻结；已将 direct-exit 相关北沿放宽到 -5.80，并在深 x 时放宽 pre_exit yaw。足迹仍与 MINCO 同为 0.58×0.44+0.01。

## 会话进展摘记（2026-09-15，定位专项：GT 位姿 + GT registered_scan）

### 契约（Gazebo sim，`use_gazebo_gt_odometry:=true`）
- `/odometry` + `odom→gimbal_yaw_odom`：唯一所有者 `gazebo_gt_odometry_relay`（chassis GT，world spawn 归零，非 `initial_map_to_odom`）。
- `/localization`：`localization_fusion` 透传 `/odometry`（odom 系）；`map→odom` 仅冻结初始 `(1.17,-0.44)`。
- `/registered_scan`：唯一所有者 `gazebo_gt_registered_scan_relay`（`/livox/lidar` 经 GT TF 变到 `odom`）。
- Point-LIO / `loam_interface` / `sensor_scan_generation`：GT 模式下 **不启动**，禁止 pose/点云污染。

### 已落地（代码）
- 新增 `gazebo_gt_registered_scan_relay.py`；launch 条件旁路 LIO 链；CMake 安装。
- 根因判断：西廊北袋/幽灵主因是 GT 位姿 + 发散 LIO `/registered_scan` 被 ROGMap 混用（MuJoCo 对照为 sim 真值点云）。

### 验证状态
- 包构建：`colcon build --packages-select rmu_gazebo_simulator` **通过**。
- 最小定位闭环（静止/短动 jump 统计、ownership）：**进行中 / 见本轮运行日志**。
- Gazebo `red_box` 10/10：**未关闭**（本对话非目标）。

### 定位专项验证（2026-09-15，Gazebo GT 位姿+点云）

#### 契约（已实现）
- `/odometry` + `odom→gimbal_yaw_odom`：唯一所有者 `gazebo_gt_odometry_relay`（chassis GT，spawn world 归零）。
- `/localization`：`localization_fusion` 透传 `/odometry`（odom 系）；`map→odom` 仅冻结初始 `(1.17,-0.44)`。
- `/registered_scan`：唯一所有者 `gazebo_gt_registered_scan_relay`。
  - 输入：`/<robot>/livox/lidar`（ros_gz `PointCloud2` RELIABLE），**不是** `/livox/lidar`（CustomMsg）。
  - 发布：RELIABLE keep_last(5)；手动 xyz TF（避开 Gazebo 字段触发的 `do_transform_cloud` dtype 断言）。
- GT 模式下 Point-LIO / `loam_interface` / `sensor_scan_generation` **不启动**。
- harness：`scripts/test_gazebo_minco_mpc_chain.sh` 显式传 `use_gazebo_gt_odometry`。

#### 已验证（domain 27，静止探针 30s）
- 证据：`log/gazebo_gt_loc_probe/20260915_143747_domain27/`
- `localization_n≈4616`，`odometry_n≈4979`，`registered_scan_n=49`
- `localization_jump_max≈0`，`jumps_over_thresh=0`，`pass=true`
- ownership：仅 GT relays；无 Point-LIO
- terrain_analysis 与 relay 的 RELIABLE QoS 已匹配（不再 incompatible）

#### 已知残余（不阻断本定位 DoD）
- 启动初期约 9 帧 TF `odom<-front_mid360` 未连通即丢弃，随后 `ok` 持续递增。
- `/registered_scan` 约 2.5–3 Hz（20k 点/帧手动变换）；低于原始 LiDAR，够规划消费但可后续加速。
- **Gazebo `red_box` 10/10 未关闭**；本轮不宣称红框通过。

#### 参考采纳（navi_minco_bit）
- 点云主输入用完整注册云 + 默认 RELIABLE 发布契约（对齐 `cloud_registered`）。
- 不做 GT 静默 remap `/localization`；位姿与点云分所有者、契约显式。

### Gazebo 先验图重定位专项（2026-09-15）

#### DoD（本轮已验证）
1. **建图期 GT 定位稳定**：`/odometry`/`/localization`/`/registered_scan` 由 GT 链路独占；静止探针无大跳变（既有 domain27 证据仍有效）。
2. **先验图任意初值重定位（Gazebo）**：错误 `initial_map_to_odom` 种子可见 → `/initialpose` 真值种子 → GICP 接受观测 → fusion 纠正 map 位姿。

#### 证据
- 先验 PCD（Gazebo 原生）：`src/ats_sentry_bringup/pcd/rmuc_2025_gazebo_prior.pcd`（约 4720 点，由 `scripts/dump_gazebo_prior_pcd_run.sh` 从 `/registered_scan` 投到 map）
- 验收脚本：`scripts/test_gazebo_prior_reloc.sh`
- 通过跑次：`log/gazebo_prior_reloc/20260915_152913_domain90/`
  - `pass_wrong_seed_visible=true`（错种子约 1.28 m / 0.40 rad）
  - `obs_accepted>=1`，`last_obs_message=accepted`
  - `pass_reloc_recover=true`（`best_xy_err≈0.64 m` < 0.70 m 门限）
  - `pass=true`

#### 关键修复
- GT `/registered_scan` 中继：RELIABLE + 订 `/<robot>/livox/lidar` + 手动 xyz TF
- GICP `init_pose` 与 fusion 的 `initial_map_to_odom` 对齐（OpaqueFunction）
- **跳过** map 系先验的 `base→lidar` 外参扭曲（空 `base_frame`/`lidar_frame`）
- 仿真-only：`relax_convergence_for_sim`（仅 `/initialpose` 之后）+ 非有限 error 时 quality 回退，避免 fusion 因 `quality=0` 拒收
- **未**放宽实车 fail-closed；`relax_convergence_for_sim` 默认 false

#### 非目标 / 残余
- 未宣称 Gazebo `red_box` 10/10
- 参考 `2026rmuc.pcd` 不能直接当 Gazebo 先验（坐标系/外参不匹配）
- small_gicp 在仿真薄壁上仍常 `converged=false error=inf`；靠仿真放宽路径验收，实车仍走严格收敛
- 恢复后位姿可能停在真值与错种子之间的可接受带内（本跑 `best_xy≈0.64 m`）；可后续加密先验/提高迭代再收紧门限


## 会话进展摘记（2026-09-16，domain23：commit 几何门禁拦北漂）

### DoD（本轮）
1. 干净残留 + `ROS_DOMAIN_ID=23` + `USE_GAZEBO_GT_ODOMETRY=true` + 无 `terrainAnalysis`：**已验证**。
2. GT `min_range=0.450`、adapter `require_terrain_inputs=false`、health `stable=3/3`、`preflight_ok`：**已验证**。
3. goal1 禁止北漂、须南向推进：**已验证南向**（未达 ≤0.50 m）。
4. Gazebo `red_box` 10/10：**未关闭**。

### 已落地代码（导航仓 + Gazebo launch）
- `minco_planner`：`CommitGeometryLimits` + `admitsCommitGeometry()`；名义 commit 前拒过大绕行；escape-from-contact 豁免。
- 参数：`commit_max_length_ratio` / `commit_max_lateral_deviation_m`（默认 `0`=关闭，MuJoCo/实车不变）。
- Gazebo-only：`ats_gazebo_nav.launch.py` 置 `2.5` / `3.0 m`（对齐 d21 gen173 `length_ratio=5.496` / `lateral=7.424`）。
- 单测：`CommitGeometryAdmission.*` **PASSED**（3/3）。

### domain23 已验证事实
| 项 | 结果 |
|---|---|
| 对照 d21 北漂 | d21 goal1 终姿 `(1.07, 1.14)`；**d23 `(1.62, -5.39)`**，spawn `(1.17,-0.44)` 南侧，无北漂 |
| detour 门禁 | `Rejecting MINCO detour before commit` **21 次**（例 `length_ratio=3.203>2.5`） |
| goal1 `south_approach` | 超时 FAIL，误差 `2.80 m`；已南推过目标 y，但 x 未收敛 |
| goal2 `south_entry` | **SUCCEEDED**，误差 `0.084 m` |
| goal3 `west_corridor_east` | FAIL，终姿 `(6.97, -5.50)` 东漂，误差 `1.90 m` |
| 西廊 stitch | `center_seat`/`prehop`/`h0`/`reface`/`h1` 仍卡（loc 拒识 `6.97,-5.50`） |
| terrain | 进程无 `terrainAnalysis`；adapter 持续 `substituting unknown terrain/slope` |
| occupied | 静止约 14.2k → 运动中约 14.7–14.9k（关 terrain 后仍偏高） |

### 证据路径
- `log/gazebo_minco_mpc_chain/20260916_141446_red_box_none_domain23/`
- harness：`/tmp/ats_p1_p2_runs/gazebo_red_box_d23.log`
- 对照北漂：`log/gazebo_minco_mpc_chain/20260915_175600_red_box_none_domain21/`

### 仍未关闭 / 下一刀
1. goal1 南向已通但 180s 未进 0.50 m：runtime swept reject + start-in-contact `footprint` 拒轨仍多；需查南向走廊 occupied 来源（ROG 高度带/膨胀），勿再缩足迹。
2. goal3 东漂与西廊 stitch 卡住：与 d8 西廊问题同类，**勿为过门禁全局放宽 fail-closed**。
3. 未 10/10 前不关闭 Gazebo `red_box` 条目；MuJoCo domain189 10/10 勿混写。


## 2026-09-17：Gazebo 红框与导航中 LOST 实测未闭环

本轮 **A 未通过，B 仅完成失败现场取证，C 一个受控样本擦线达标但不稳定，D 通过**。无生产源码或配置改动，未调整 footprint、fusion correction 或 overlap 门限以换取通过。未重跑已完成的 mid/hard spawn 压力。完整命令、实验脚本与失败尝试见 `log/gazebo_validation_20260917/EVIDENCE.md`。

### 红框主门：domain104 安全中止

执行：

```bash
ROS_DOMAIN_ID=104 PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 GOAL_TIMEOUT_SEC=180 \
  scripts/test_gazebo_minco_mpc_chain.sh
```

脚本实际读取 `GOAL_TIMEOUT_SEC`。domain101 的首次 preflight 因 7 个旧 bridge 残留拒绝启动；定向清理记录保留，未绕过门禁。domain104 日志位于 `log/gazebo_minco_mpc_chain/20260917_155559_red_box_none_domain104/`。

| 目标 | 实际终姿 map (m) | 位置误差 (m) | 结果 |
| --- | --- | --- | --- |
| goal1 south_approach | (1.954684, -1.642510) | 3.479037 | ABORTED |
| goal2 south_entry | (1.954684, -1.642510) | 4.909765 | ABORTED |
| south_dip helper | (1.954684, -1.642510) | 南向差 4.707490 | 已派发，失败 |
| goal3 west_corridor_east | (1.954684, -1.642510) | 5.594890 | ABORTED，未进入南走廊验收带 |
| goal4 west_corridor_exit | 未验证 | 未验证 | 仅运行口部/走廊 helpers，主目标未派发 |
| goal5–10 | 未运行 | 未验证 | 安全中止后未派发 |

前三腿 action 原因均为 `progress watchdog exhausted the bounded suspended-replan wait`。实际出现 `west_corridor_south_dip`，未采用 near-band skip；但其起点仍远离口部，不能计为走廊通过。未出现 `true_stuck_*_abort` 或 `skipped_north_pocket`，这两个门禁本轮未验证；goal5–10 没有派发是外部安全中止的结果。

16:22:25 因急停振荡和重复 mouth recovery 停止本次仿真，`SAFETY_ABORT.json` 记录急停 true 51 次（含初始）、false 50 次；cleanup 后全日志为 51/51。最后一次 harness 位姿约 (0.467706, 2.311446)，不是 goal4 成功终姿。红框没有自然完整跑完，不能宣称完整回归通过。

- 拒绝 unsafe MINCO 候选 799 次，单候选 footprint collision count 为 1–416；这些是拒绝候选计数，不是已发布路径或物理接触计数。
- 有限窗口独立 analyzer：一条 263 点 reference 对接收时 adapter snapshot 有 5 个几何冲突（离散 3、swept 2）；851 个可评估 actual ticks 中 20 个冲突，另 1 个 swept segment 冲突。[Confidence: Medium] 它不是 producer 的 MINCO commit snapshot；另有首段漏采、3990 ticks 缺 payload、17 ticks 缺 TF、1 个层 gzip CRC 损坏，不能据此宣称完整 footprint 通过，也不能直接判定 planner fail-open。原始与仅含有效文件的分析视图均保留。
- 物理接触：**未验证**。首 90 s 内部 observer 采到 JPS/MINCO/MPC predicted 最大点数 30/190/31；planning grid publisher max=1，识别到 adapter，存在匿名 discovery 样本；selected publisher max=1。完整运行唯一所有权未闭合。

### P0 口部与北袋：本次未复现 occupied

`corridor_comparison.json` 使用 goal3 final ROS stamp 177.969 附近的 planning grid 178.112（差 0.143 s）与 ROG 数值服务 178.782；服务和 adapter 不同 generation，不声称原子配对。TF 实测 `map←odom=(1.17,-0.44,0)`，ROG 的 `odom` 单元变换到 map 后计数。

| 原生图层 | 口部 (5.0–5.4, -6.4–-6.0) free/occ/unknown | 北袋 (6.5–7.2, -5.7–-5.3) free/occ/unknown |
| --- | --- | --- |
| planning，0.10 m | 16 / 0 / 0 | 28 / 0 / 0 |
| static，0.05 m | 64 / 0 / 0 | 112 / 0 / 0 |
| ROG raw，0.10 m，仅投影覆盖部分 | 0 / 0 / 4 | 0 / 0 / 12 |

ROG 未覆盖部分是缺少该来源证据；static 明确 free 消解 ROG unknown。terrain/slope 没有观测，launch 显式关闭 terrain，adapter 实效 `require_terrain_inputs=false`。参数服务确认高度带 `[0.10,0.80]`、`core.inflation_step=1`、`unknown_is_obstacle=true`。

**没有图层把本次指定测区写成 occupied。** [Confidence: High] 原始 dump 与离线重算支持计数；[Confidence: Medium] 不能将远处停机样本推广为机器人靠近口部时的根因，也不能由小测区 free 推导完整矩形 footprint 可通行。数值投影源码调用原始 `getGridType()`，不读取 inflated occupancy；不能直接把 planning-grid occupied 归因于 `inflation_step`。本轮没有修改共享 adapter YAML 的证据基础。

### 导航中 1.5 m 偏差恢复：domain118 单次擦线达标

`log/gazebo_validation_20260917/run_mid_nav_full_deadline.sh` 在独立 domain118 启动；日志内实验 launch 副本保持 TRACKING `2.0 m/1.0 rad`、LOST `5.0 m/1.5 rad`，observation timeout 为 6/8 s。生产 launch 未修改。GICP 延后启动以隔离导航偏差注入，故不覆盖 GICP 持续运行中的自然 LOST。

1. TRACKING 下实际导航位移 `1.000940 m`，真值约 `(1.855249,-1.169599)` 后，通过一次标记为 `TEST_ONLY` 的 synthetic observation 注入 `map→odom` x 偏差 1.5 m，未增加 TF publisher。车辆仍在移动，第一次 error>1 m 的观测值为 `1.050332 m`，不能把该时刻误写成稳态 1.5 m 残差。
2. 状态为 `TRACKING → RELOCALIZING → TRACKING → DEGRADED → LOST`。GICP 随后出现 `Starting async multi_guess recovery with 225 candidates (state=4)`。
3. 搜索期间发送 `/initialpose`，日志记录接收并取消异步搜索；随后 seeded coarse+fine 被接受。以 `/initialpose` 后 90 s 为 deadline，**24.039 s 首次达到 best_xy=0.796377 m ≤ 0.80 m**（从 GICP 启动计为 44.050 s），该点已回到 TRACKING，恢复阶段 6 条 accepted observation。仅有约 3.6 mm 门限余量；未要求持续保持，也没有证据称为稳定残差。
4. 注入/恢复阶段 status 接收最大间隔分别 `0.109778/0.363151 s`，ROS stamp 持续推进。原脚本 `pass=false` 使用包含 action-server 阻塞等待的全局 gap `3.292581 s`；原文件保留，`mid_nav_domain118/result_audited.json` 显式按任务要求的注入/恢复期间审计，`audited_c_gates_met=true`。这是观察窗口修正，未改变 0.80 m 或 90 s 门限。
5. C++ 观察器从 DDS MessageInfo 提取的 map→odom GID 仅一个，graph 映射 `/localization_fusion`。[Confidence: High] 本次阈值到达、TF 权威与期间 status 新鲜有运行证据；[Confidence: Medium] 仅为晚启动 GICP 的一次受控样本，不证明持续运行 GICP 的自然恢复稳定性，不宣称优秀全局后端。

反例全部保留：domain116 同样实际注入 1.5 m、进入 LOST 并 async，但从 GICP 启动计 90 s 内最佳误差仍 1.50 m；其 `/initialpose` 后仅覆盖约 70 s，因计时口径不足而重跑 domain118。domain110 是 Humble rclpy 观察器 MessageInfo API 不兼容，未注入；domain114 短 lease 下未完成 1 m TRACKING 导航位移；domain115 有导航位移，但 fusion 严格 plausibility gate 拒绝 GICP 观测，未建立 accepted TRACKING，未注入 SIGSTOP。这些样本不冒充恢复成功。

### 回归、参数差异与剩余边界

- `MAKEFLAGS=-j1 colcon build --base-paths src --packages-select small_gicp_relocalization --parallel-workers 1`：通过。
- 指定 `colcon test ... --ctest-args -R 'gtest|pytest|cpplint|clang_format'`：实际重跑两个 lint，通过。CTest `-R` 匹配名称，故补跑 `test_localization_fusion_core|test_localization_fusion_node`：5 个 GTest、1 个 pytest 均通过。`test-result` 的 34 tests/0 errors/0 failures/5 skipped 含历史 XML，不称作本轮全部重跑。
- domain117：`OFFSET_X=1.0 OFFSET_Y=0.8 OFFSET_YAW=0.40 RECOVER_XY_M=0.60 scripts/test_gazebo_prior_reloc.sh`，`pass=true`，初始误差 `1.280625 m`、最佳误差 `0.580451 m`、yaw `0.178706 rad`、脚本退出 0。证据：`log/gazebo_prior_reloc/20260917_164128_domain117/reloc_probe.json`。
- launch `--show-args`、Python AST 语法、两份 shell `bash -n` 均通过；各仓 `git diff --check` 在提交前单独执行。
- **实效参数与前提不同**：Gazebo nominal fusion 被现有 launch 覆盖为 TRACKING/LOST 均 5.0/1.5，源码默认 2.0/1.0 不是 nominal 实测值。MINCO 实效 footprint 为 0.58×0.44+0.01，adapter 为 0.70×0.55+0.05；本轮未修改，不能声称统一 0.70×0.55 足迹验收。
- 本轮为 Gazebo GT odometry/registered-scan profile，不是 Point-LIO 运动验证；未将 MuJoCo 189 作为本轮证据。P3/Nav2-free、Gazebo red_box 10/10、残差≤0.30 m、实车信息矩阵分布与物理接触均未验证。
- 下一步先对齐接收时地图与 planner commit snapshot、定位移动后错误先验匹配的接受原因，再提出最小修复；没有证据支持通过缩小 footprint、增大 correction 或放宽恢复门限解决本轮失败。
