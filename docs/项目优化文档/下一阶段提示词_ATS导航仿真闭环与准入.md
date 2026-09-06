# 下一阶段提示词：ATS 导航仿真闭环与准入

下面内容用于 Claude 的 P2 接触区、red-box 和故障矩阵工作。Claude 负责实现、测试、仿真和证据报告；Codex 负责 review、文档清理、提交和 push。

```text
[$develop-robot-vision-navigation]

工作目录：/home/ats/ATS_2026_snetry_test

先阅读 AGENTS.md、docs/nav2_to_3desdf_minco_mpc_optimization_direction.md 的最新 review 结论、
docs/项目优化文档/ATS导航剩余优化总TODO.md，以及 minco_planner 的 terminal yaw relocation、goal admission、
footprint safety、local repair 和测试。开始前列出完成条件、精确文件范围、验证命令、假设和风险转入条件，
保留工作区既有修改并保持 Git 无提交。

可信基线（2026-09-01）：

- Gazebo P1 在动态 TF 双门禁下 domain 147/149/151 连续通过；bridge 延迟在 143 复现过，问题仍开放。
- MuJoCo P2 六故障 domain 154/156/158/166/168/170 为 6/6，unknown 恢复后新目标已独立确认。
- single domain 178 的 reference/actual 足迹碰撞和 contact telemetry 均为零。
- red_box domain 176 目标 5 的离线结果为 Q1=yes、Q2=yes、Q3=no：实际跟踪越界先于安全提交门拒绝，
  最大偏航误差 1.107 rad、横向误差 0.275 m、足迹最小间隙 -0.100 m。escape_from_contact_enabled 和
  ego_blocked_escape_enabled 保持关闭。
- 失败 leg 的 contact/analyzer 收尾仍在成功断言之后，失败段的物理接触无法补测；Gazebo 物理接触也没有独立遥测。

完成条件：

1. 对目标 5 和 unknown 恢复建立 reference -> actual -> selected command -> stop pose 的同一时钟证据，
   记录横向/纵向/yaw 误差、命令年龄、急停时刻和停止距离。
2. 首冲突样本关联 immutable snapshot identity、publication sequence、地图来源和 footprint evaluator 结果，
   区分发布时不安全、跟踪越界和地图更新翻转。
3. 修复放在首个违反安全不变量的行为 owner，任何提交 reference 都满足离散与 swept 碰撞为零。
4. 每个 fault 使用新 ROS_DOMAIN_ID；故障链路记录 ready=false -> emergency_stop=true -> selected=0 ->
   motion_control=0，恢复后旧 reference 不恢复执行。
5. 失败 leg 也保存 analyzer 和 contact 证据；若证据缺失，结果写为未验证并保留失败原因。

任务顺序：

一、目标 5 取证和修复

- 不改安全阈值先复现目标 1--5。围绕首个低净空冲突采集 localization、reference、MPC predicted/executed、
  /cmd_vel/selected、/motion_control、急停和地图 generation。
- 对 reference 与 actual 运行同一 footprint evaluator，保存最小 clearance、首冲突 index/cell/center/yaw、
  实际速度和急停到静止的位移。
- 若 Q1 失败，审查最终 snapshot、采样连续性和提交时序；若 Q2 失败，审查 MPC 跟踪误差、执行延迟、速度/加速度
  限幅和停车包络；若 Q3 为 yes，审查 generation、heartbeat 和旧 reference 撤销。

二、unknown 恢复

- 记录故障前、故障中、ready 恢复和新目标提交四个时刻的 localization、起点 footprint、地图来源、source generation、
  adapter publication 和 MINCO local snapshot。
- 逐格说明恢复起点的 occupied/unknown 来源和值。健康条件恢复后才重新准入；起点不安全时保持零速并返回结构化失败。

三、目标 9/10

- 仅在目标 5 跟踪问题收敛后重新评估 relocation。输出 tail/window、全部候选 index、冲突区间和拒绝原因。
- 候选按冲突带边界和弧长确定性补点；不依赖目标编号、场景旁路或降低安全阈值。
- 目标 9 达到零碰撞安全终态后，用新 domain 独立发送目标 10，并保存起点 footprint 和 snapshot identity。

四、验证与报告

- 先运行 interfaces、RC-ESDF、minco_planner、goal_manager、terrain_analysis_ext 和 MuJoCo 的窄构建与聚焦测试。
- 运行 `python3 scripts/test_analyze_nav_tracking.py`、`python3 scripts/test_footprint_evaluator.py`、
  `bash scripts/test_footprint_evaluator_parity.sh`、`bash scripts/test_mujoco_contact_gate.sh`。
- `farthest-free --max-distance` 默认 0.0 保持兼容；runner 的 `FARTHEST_GOAL_MAX_DISTANCE` 默认 4.0，
  对应查询工具回归一并保存。
- 报告区分已验证、已实现未运行、推断和未实现；列出根因、行为 owner、精确文件、命令/退出码、fault matrix、
  red_box 每段结果、碰撞和接触证据、未覆盖项。完成后输出 READY_FOR_CODEX_REVIEW。
```
