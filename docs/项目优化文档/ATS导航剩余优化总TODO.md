# ATS 导航剩余优化总 TODO

> 状态：唯一活动导航优化清单
> 更新时间：2026-09-09
> 活动证据窗口：2026-08-30 至 2026-09-09
> 适用范围：Gazebo、MuJoCo 与实机导航软件侧的 ATS 四驱四转哨兵导航链
> 归档规则：窗口以前的运行流水、旧 domain 和已退役结论从活动文档移除；原始日志、artifact 与 Git 历史保留追溯入口。

## 0. 当前结论

- Gazebo P1 在当前动态 TF age/staleness 门禁下，domain `147/149/151` 连续 `3/3` 通过；domain `143` 的约 `2 s` bridge 延迟仍可复现，P1 的缺陷消除结论尚未形成。
- MuJoCo P2 六故障矩阵在独立 domain `154/156/158/166/168/170` 为 `6/6`；`single` domain `178` 的 reference、actual 和物理接触增量均为零。本轮未用新 domain 重跑该矩阵。
- 有效红框样本为 domain `186`（`PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box GOAL_TIMEOUT=180`）。目标 1--9 全部到达，终点误差 `0.049/0.009/0.036/0.034/0.010/0.061/0.023/0.024/0.033 m`，离散 footprint collisions=0，analyzer Q1 yes / Q2 no / Q3 no。目标 10 未下发（goal 9 的 sim contact gate 先拒绝）。
- `red_box` 未通过的剩余阻塞是 **sim 物理接触分类**：高地坡道 hfield `rmuc_2025_field` 被正确放入 `ground_geom_ids`，但底盘（非轮）与 hfield 接触在 `contact_is_violation()` 中被判违规。3 次 contact_violation 来自底盘爬坡时与坡面接触，最大力 `234 N`。规划器已提交零碰撞轨迹，机器人到达目标。
- P2 总体准入仍为未通过：sim 接触分类需修或 runner 需对坡道段豁免。
- 速度链继续采用车体系 `[vx, vy, wz]`，`cmd_vel_arbiter` 是 `/cmd_vel/selected` 的唯一 publisher；`escape_from_contact_enabled` 和 `ego_blocked_escape_enabled` 当前关闭。

## 1. 最近一周已完成

以下条目仅记录窗口内已经落地且有对应测试或运行证据的优化。详细原始输出放在 artifact，活动文档只保留结论和边界。

| 日期 | 优化或验证 | 证据与边界 |
| --- | --- | --- |
| 2026-09-09 | MINCO 内角圆角（fillet）+ guide densify + 连续侧向加速度限速 + 提前窄通道 yaw 切线 | `test_path_geometry_preprocessor` 5/5；`test_minco_trajectory_optimizer` 9/9；`test_yaw_spline_planner` 8/8。domain `186` 目标 1--9 footprint collisions=0 |
| 2026-09-09 | MINCO 提交点 digest 配对、PlannerGoal 冻结世界系 twist、四路径同 seed | 库级 GTest 通过；domain `186` commit 日志带 digest |
| 2026-09-09 | MuJoCo runner 失败路径 flush recorder/analyzer/contact；pose capture 6 次重试 | `scripts/test_mujoco_failure_evidence.sh` PASSED；domain `186` 目标 9 flush 写出 verdict.json |
| 2026-09-09 | Gazebo runner 探测 contacts topic；无来源写 unverified | 已实现未运行 |
| 2026-09-09 | 诊断 topic 门控：空 topic 名不创建 publisher | `preprocessed_guide_topic=""`，`esdf_refined_guide_topic=""`，`debug_marker_topic=""` |
| 2026-09-09 | MPC yaw 权重 4→10、min_reference_progress_scale 0.25→0.10 | `ats_swerve_mpc_reality.yaml` 已更新 |

## 2. 当前未完成任务

### P1：Gazebo bridge 延迟归因

- [ ] 在带 `parameter_bridge` 进程资源采样的失败运行中，区分 bridge 内部处理成本、上游发布变慢和 DDS 接收缺口；保持同一 revision、profile、起点、窗口和新 domain。
- [ ] 运行结果同时保存 gz-transport、ROS raw LiDAR、Point-LIO、localization 的 source stamp、wall gap、age、更新计数和进程资源；缺少独立计数的字段标记为 `unverified`。
- [ ] P1 通过条件仍为动态 TF 双门禁、localization freshness、straight action、唯一 owner 和完整 recorder 窗口共同成立。

### P2：MINCO 生产安全契约

- [x] 提交点 digest 配对、PlannerGoal 冻结、四路径同 seed、非有限初值 fail-closed。**已验证** domain `186`。
- [x] 近零播种不再把短路径拉成数百秒；allocator 仅在播种速度达标时才 cap 首端时长。
- [x] guide densify（`guide_control_point_spacing=0.30`）+ 内角 fillet（`path_fillet_radius=0.35`）+ 连续侧向加速度限速。**已验证** domain `186` 目标 1--9。
- [ ] prepared seed 计算量收益仍待专门重规划 A/B。

### P2：red_box 跟踪偏差

- [x] domain `176` Q2 越界在 domain `186` 未复现：Q1 yes / Q2 no / Q3 no。
- [x] 目标 1--9 每次提交 reference 的离散 footprint collisions=0。
- [x] `escape_from_contact_enabled` 保持关闭。
- [ ] sim 接触分类：高地坡道 hfield 与底盘接触不应计为物理违规。

### P2：失败运行证据收尾

- [x] runner 失败路径 flush recorder/analyzer/contact；`scripts/test_mujoco_failure_evidence.sh` PASSED。
- [x] domain `186` 目标 9：contact 为真实读数 `delta=3`，不是填零；analyzer verdict.json 存在。

### P2：Gazebo 物理接触

- [x] Gazebo runner 探测 contacts topic；无来源写 unverified。**已实现未运行**。
- [ ] 在 P1 资源归因完成后，再记录 Gazebo nominal 与 red-box 的 contact delta。

### P2：目标 9/10

- [x] 目标 9 `highland_ramp` 到达：终点误差 `0.033 m`，footprint collisions=0，Q1 yes / Q2 no。
- [x] terminal yaw relocation 现输出 tail/window、全部候选 index、冲突区间和拒绝原因。
- [ ] 目标 10 未下发：goal 9 的 sim contact gate 先拒绝。修复接触分类后需用新 domain 独立发送目标 10。

### P3/P4/QP 后续

- [ ] P3 Nav2-free 运行证据。
- [ ] P4 连续 swept footprint、地图边界、unknown、定位跳变、solver failure 和长时资源稳定性形成仿真证据。
- [ ] QP 保持 shadow/显式拒绝状态。

## 3. 稳定契约

- Point-LIO 提供 `/localization` 与 `/registered_scan`；ROGMap 负责占据、膨胀和 ESDF，不承担定位职责。
- adapter 消费 ROGMap 数值 projection；unknown、occupied、outside-map、signed-distance 和 footprint 语义保持分层。
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 holonomic SE(2) MPC 维持现有边界。
- `/cmd_vel/autonomy_raw` 的自主 publisher 为 `ats_swerve_mpc`，`/cmd_vel/selected` 的 publisher 为 `cmd_vel_arbiter`；实机、MuJoCo 和 Gazebo 最终执行端只接收 selected。
- manual fresh 优先，manual timeout 归零；auto 依赖新鲜 `ExecutionCommand`、lease、incarnation、急停和定位/地图健康。DOWN->UP 仅接受恢复后的新命令。
- 地图与轨迹使用 immutable snapshot；source generation、adapter publication sequence 和 MINCO local snapshot generation 分开记录。
- map unready/stale、unknown、无路、unsafe trajectory、solver failure 和 stale command 的结果是结构化失败与确定性零速。

## 4. 下一轮 DoD

1. 修 sim `contact_is_violation()`：高地 hfield 与底盘（非轮）接触不判为违规，或 runner 对坡道段豁免。
2. 新独立 domain 完整 `red_box` 10/10，目标 10 独立发送。
3. 一例独立 `P2_FAULT_CASE` 用新 domain 重跑。
4. Gazebo bridge 失败样本完成资源/发布/DDS 分层。

## 5. 验证清单

```bash
python3 scripts/test_analyze_nav_tracking.py
python3 scripts/test_footprint_evaluator.py
bash scripts/test_footprint_evaluator_parity.sh
bash scripts/test_gazebo_dynamic_tf_gate.sh
bash scripts/test_mujoco_contact_gate.sh
bash scripts/test_mujoco_failure_evidence.sh
bash scripts/test_gazebo_runner_contract.sh
python3 scripts/test_validate_navigation_config.py
source install/setup.bash && python3 scripts/test_nav_tracking_recorder.py
git diff --check
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

构建与仿真仍采用单 worker、headless、新 `ROS_DOMAIN_ID` 和固定 profile。验收窗口内不与 `colcon build` 并行。

本轮预存失败（未修）：`scripts/test_validate_navigation_config.py` `KeyError: cmd_vel_topic`；`scripts/test_gazebo_runner_contract.sh` `recorder does not use the dynamic TF freshness witness`。

## 6. 证据与报告边界

- 已验证、已实现未运行、推断和未验证分栏记录；测试通过不替代闭环运行证据。
- 目标终态误差与 recorder 结束时定位误差分开记录；`footprint_collisions=0` 不推导物理接触为零。
- domain `183` 目标 9 规划拒绝（MINCO 切内角）、domain `184` 目标 9 起点碰撞、domain `186` 目标 9 到达但 sim 接触拒绝：三者分别记录。
- 实机/HIL 尚未运行；未在目标机测得的数据不作为实机性能声明。
