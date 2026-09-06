# ATS 导航剩余优化总 TODO

> 状态：唯一活动导航优化清单
> 更新时间：2026-09-06
> 活动证据窗口：2026-08-30 至 2026-09-06
> 适用范围：Gazebo、MuJoCo 与实机导航软件侧的 ATS 四驱四转哨兵导航链
> 归档规则：窗口以前的运行流水、旧 domain 和已退役结论从活动文档移除；原始日志、artifact 与 Git 历史保留追溯入口。

## 0. 当前结论

- Gazebo P1 在当前动态 TF age/staleness 门禁下，domain `147/149/151` 连续 `3/3` 通过；domain `143` 的约 `2 s` bridge 延迟仍可复现，P1 的缺陷消除结论尚未形成。
- MuJoCo P2 六故障矩阵在独立 domain `154/156/158/166/168/170` 为 `6/6`；`single` domain `178` 的 reference、actual 和物理接触增量均为零。
- `red_box` 仍未通过。domain `176` 目标 5 的同钟分析为 Q1 reference 安全、Q2 实际跟踪越界、Q3 未发现地图翻转；最大偏航误差 `1.107 rad`、横向误差 `0.275 m`、最小足迹间隙 `-0.100 m`。
- P2 总体准入仍为未通过：`red_box` 的 MPC/执行偏差尚未修复，Gazebo 侧没有独立物理接触遥测，失败 leg 的运行期接触收尾也未闭合。
- 速度链继续采用车体系 `[vx, vy, wz]`，`cmd_vel_arbiter` 是 `/cmd_vel/selected` 的唯一 publisher；`escape_from_contact_enabled` 和 `ego_blocked_escape_enabled` 当前关闭。

## 1. 最近一周已完成

以下条目仅记录窗口内已经落地且有对应测试或运行证据的优化。详细原始输出放在 artifact，活动文档只保留结论和边界。

| 日期 | 优化或验证 | 证据与边界 |
| --- | --- | --- |
| 2026-08-30 至 2026-08-31 | MuJoCo 运行前产物/域审计、P2 故障观察器、查询工具回归；`farthest-free` 增加可选距离上限，runner 默认 `FARTHEST_GOAL_MAX_DISTANCE=4.0` | `scripts/test_query_occupancy_grid.py`、`scripts/test_p2_fault_observer.py`、`scripts/test_runtime_binary_freshness.sh` 通过；默认 `0.0` 保持旧查询语义 |
| 2026-08-31 | MINCO 足迹门禁、Goal Manager 有界重规划与故障状态收敛；`freeze` 终止分支补齐结构化错误日志 | 导航仓 revision `efd68e1`；相关 focused GTest 已纳入当前构建；日志文本来自 action result 的 `progress_failure` |
| 2026-09-01 | P2 六故障矩阵独立 domain 全部通过；`unknown` 恢复后新目标独立到达 | `154/156/158/166/168/170` 为 `6/6`；`unknown` 恢复结果 code `0`，旧 reference 未重新发布；故障运行仍需保留失败证据收尾回归 |
| 2026-09-01 | Gazebo P1 动态 TF 双门禁下连续三次通过 | `147/149/151` 均 `all_p1_gates_passed`、`failures=0`；该结果不消除 domain `143` 的间歇 bridge 延迟，也不提供 Gazebo 物理接触结论 |
| 2026-09-01 | arbiter 与四环境 launch/config 契约回归 | arbiter GTest `24 tests, 0 failures`；配置校验 `11 tests` 通过；`/cmd_vel/selected`、规划栅格和 TF 的 owner 结论以运行图和源码双证据为准 |
| 2026-09-01 | MuJoCo `single` 闭环与接触 evaluator 运行验证 | domain `178` 终点误差 `0.0343 m`，reference/actual footprint collisions 为 `0`，`contact_violation_delta=0`、`max_contact_force_n=0.0`；不能外推到失败 leg 或 Gazebo |
| 2026-09-06 | 跟踪取证、footprint evaluator、动态 TF gate、物理接触 gate 和 reference/snapshot pairing 回归补齐 | footprint C++ parity `1280/1280`；相关 Python/shell 测试通过；不同 stamp 的同几何 pairing 已有回归；失败 leg 仍受 runner 提前退出影响 |

## 2. 当前未完成任务

### P1：Gazebo bridge 延迟归因

- [ ] 在带 `parameter_bridge` 进程资源采样的失败运行中，区分 bridge 内部处理成本、上游发布变慢和 DDS 接收缺口；保持同一 revision、profile、起点、窗口和新 domain。
- [ ] 运行结果同时保存 gz-transport、ROS raw LiDAR、Point-LIO、localization 的 source stamp、wall gap、age、更新计数和进程资源；缺少独立计数的字段标记为 `unverified`。
- [ ] P1 通过条件仍为动态 TF 双门禁、localization freshness、straight action、唯一 owner 和完整 recorder 窗口共同成立。

### P2：MINCO 生产安全契约

- [ ] 在 planner commit site 记录 published digest、point count、gate collision count/indices、grid topic、publication sequence、occupancy digest 和 validation frame，闭合“提交引用被自身 checker 拒绝”的审查链。
- [ ] `PlannerGoal` 冻结与 goal、map snapshot 同一 localization epoch 的速度、stamp、frame 和 request identity；center、footprint、fallback、repair 共用该 immutable initial state。
- [ ] production 的四条 optimizer 路径接入该状态，stale、epoch mismatch、TF failure 和非 finite 输入返回结构化失败并维持零速。
- [ ] prepared seed、时间分配和候选 telemetry 经过库级与 node 级回归后，再评估重规划首端连续性和计算量收益。

### P2：red_box 跟踪偏差

- [ ] 先以 domain `176` 的 Q2 证据为基线，核对 MPC 请求、执行延迟、速度/加速度限幅、轮端反馈和停车包络的一致性。
- [ ] 以新的独立 domain 复跑目标 1--5，保存 reference、actual、selected command、motion control、急停和停止距离的同钟配对；每次提交 reference 都通过离散与 swept footprint gate。
- [ ] `escape_from_contact_enabled` 保持关闭；安全阈值和 footprint margin 不作为通过手段。

### P2：失败运行证据收尾

- [ ] 将 `scripts/test_mujoco_minco_mpc_chain.sh` 的 contact capture、recorder stop、analyzer 和 artifact flush 统一放入失败也会经过的清理路径，同时保留原始退出码。
- [ ] 增加 focused shell regression：动作失败时仍产生 analyzer 输出和运行期 contact 读数；读数缺失时结果为 `unverified`，不是零。
- [ ] runner 收尾修复后重新执行至少一个 red-box 失败段和一例 fault，不能用修复前失败段宣称接触结论。

### P2：Gazebo 物理接触

- [ ] 为 Gazebo 增加独立 contact telemetry 或明确的替代测量；`footprint_collisions=0` 只代表几何采样安全。
- [ ] 在 P1 资源归因和 red-box 跟踪修复完成后，再记录 Gazebo nominal 与 red-box 的 contact delta、最大接触力和最小净空。

### P2：目标 9/10

- [ ] 在跟踪偏差收敛后重新评估 terminal yaw relocation 的东向摆动和候选间距；输出 tail/window、全部候选 index、冲突区间和拒绝原因。
- [ ] 目标 9 达到零碰撞安全终态后，用新 domain 独立发送目标 10，并保存起点 footprint、snapshot identity 和停止状态。

### P3/P4/QP 后续

- [ ] P3 Nav2-free 运行证据覆盖 `launch_nav2=false`、无 Nav2 server、ATS action 生命周期和旧结果不复活。
- [ ] P4 连续 swept footprint、地图边界、unknown、定位跳变、solver failure 和长时资源稳定性形成仿真证据。
- [ ] QP 保持 shadow/显式拒绝状态；paired A/B、hard-check、residual、slack、deadline 和 fallback 证据完成后再评估主链切换。

## 3. 稳定契约

- Point-LIO 提供 `/localization` 与 `/registered_scan`；ROGMap 负责占据、膨胀和 ESDF，不承担定位职责。
- adapter 消费 ROGMap 数值 projection；unknown、occupied、outside-map、signed-distance 和 footprint 语义保持分层。
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 holonomic SE(2) MPC 维持现有边界。
- `/cmd_vel/autonomy_raw` 的自主 publisher 为 `ats_swerve_mpc`，`/cmd_vel/selected` 的 publisher 为 `cmd_vel_arbiter`；实机、MuJoCo 和 Gazebo 最终执行端只接收 selected。
- manual fresh 优先，manual timeout 归零；auto 依赖新鲜 `ExecutionCommand`、lease、incarnation、急停和定位/地图健康。DOWN->UP 仅接受恢复后的新命令。
- 地图与轨迹使用 immutable snapshot；source generation、adapter publication sequence 和 MINCO local snapshot generation 分开记录，当前没有编号端到端一致证据。
- map unready/stale、unknown、无路、unsafe trajectory、solver failure 和 stale command 的结果是结构化失败与确定性零速。

## 4. 下一轮 DoD

1. planner commit log 能将 published digest 与 gate verdict、grid identity 和 validation frame 一行配对。
2. red-box 的新运行显示跟踪误差、执行限幅和停止包络满足 footprint/swept 安全门；失败原因仍可审计。
3. 动作失败的 MuJoCo leg 保留 analyzer、contact、recorder 和原始退出码，缺失字段不被填成通过值。
4. Gazebo bridge 失败样本完成资源/发布/DDS 分层；P1 结论与 P2 算法证据分别记录。
5. 目标 9/10 在跟踪修复后的新 domain 独立验证，或保留结构化未通过原因。

## 5. 验证清单

```bash
python3 scripts/test_analyze_nav_tracking.py
python3 scripts/test_footprint_evaluator.py
bash scripts/test_footprint_evaluator_parity.sh
bash scripts/test_gazebo_dynamic_tf_gate.sh
bash scripts/test_mujoco_contact_gate.sh
bash scripts/test_gazebo_runner_contract.sh
python3 scripts/test_validate_navigation_config.py
source install/setup.bash && python3 scripts/test_nav_tracking_recorder.py
git diff --check
```

构建与仿真仍采用单 worker、headless、新 `ROS_DOMAIN_ID` 和固定 profile。验收窗口内不与 `colcon build` 并行；产物审计发现 revision、artifact 或运行环境污染时，该样本标记为无效并重新运行。

## 6. 证据与报告边界

- 已验证、已实现未运行、推断和未验证分栏记录；测试通过不替代闭环运行证据。
- 目标终态误差与 recorder 结束时定位误差分开记录；`footprint_collisions=0` 不推导物理接触为零。
- 失败段若缺少接触 evaluator、reference/snapshot pairing 或完整 recorder 窗口，结论保留为 `unverified`，同时保留失败原因和可复现 artifact。
- 实机/HIL 尚未运行；未在目标机测得的数据不作为实机性能声明。
- 下一阶段提示词位于：
  - `docs/项目优化文档/下一阶段提示词_ATS单雷达导航闭环与速度仲裁.md`
  - `docs/项目优化文档/下一阶段提示词_ATS导航仿真闭环与准入.md`
