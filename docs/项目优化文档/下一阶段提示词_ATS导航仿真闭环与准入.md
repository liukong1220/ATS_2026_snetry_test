# 下一阶段提示词：ATS P2 接触区停机根因与 red-box 收敛

下面内容可直接作为 Claude 新对话的首条提示词。

```text
[$develop-robot-vision-navigation]

继续 /home/ats/ATS_2026_snetry_test 的 ATS 四驱四转哨兵导航优化。本轮从 Codex review 后的
P2 安全基线开始，优先定位 MuJoCo red_box 目标 5 与 unknown 恢复后的接触区问题，不再重复 P1 bridge
归因实验，也不先调整目标 9/10 的 terminal relocation。

开始前请完整读取：

1. AGENTS.md
2. docs/nav2_to_3desdf_minco_mpc_optimization_direction.md 的 2026-08-31 review 结论
3. docs/项目优化文档/ATS导航剩余优化总TODO.md 的 2026-08-31 当前执行清单
4. minco_planner 的 terminal yaw relocation、goal admission、footprint safety、local repair 与测试
5. ats_goal_manager 的 progress watchdog 与 RMUC 2025 navigation profile

工作边界：Claude 负责定位、实现、构建、测试、仿真和交付报告；Git 暂存、提交和 push 交给 Codex。
建议保留用户已有修改，显式列出每个准备编辑的文件。每次源码修改前先给出 DoD、文件范围、验证清单、
当前假设和风险转入条件。

当前可信基线：

- Gazebo P1 已在 domain 127/129/131 连续三次通过；本轮不通过放宽 freshness、TF 或安全阈值换结果。
- Point-LIO Z 发散已由 Gazebo launch 的 SI 单位覆盖修复：acc_norm=9.81、satu_acc=30.0；实机配置保持不变。
- MuJoCo 最终安全 revision 已在独立 domain 132--137 重跑：adapter_lease、service_timeout、input_stale、
  unreachable、freeze 通过；unknown 的故障注入和两级零速通过，但恢复后的新目标因起点附近
  `8--9` 个 footprint collisions 被安全拒绝，当前矩阵为 5/6。
- planner escape_from_contact_enabled=false，goal manager ego_blocked_escape_enabled=false；RMUC profile
  显式保持两者关闭。runner 对任一 footprint_collisions>0 判失败。
- 有界碰撞前缀不能区分量化接触与薄实体障碍。本轮避免重新打开 escape，也避免让 runner 接受碰撞轨迹。
- 最终安全 revision 的 red_box domain 138 中目标 1--4 以零碰撞成功，终点误差分别为
  `0.0456/0.0069/0.0277/0.0492 m`；目标 5 在约 `(1.18,-7.62,yaw=0.50)` 停入墙侧占据带，
  后续重规划从 index 0/1 持续发现 `2--4` 个碰撞并由 watchdog 安全终止。
- red_box 仍未通过，P2 不能写成总体通过；MuJoCo 物理接触仍未由独立 evaluator 验证。

Definition of Done：

1. 对目标 5 和 unknown 恢复分别建立 `reference pose -> localization actual pose -> selected command -> stop pose`
   的同一时钟证据，记录横向/纵向/yaw 跟踪误差、速度、命令年龄、急停时刻和停止距离。
2. 对每个首冲突样本记录 immutable snapshot identity，以及 terrain、static、unknown、ROG inflation 的来源，
   区分轨迹发布时已碰撞、执行跟踪后越界和地图更新后变为碰撞三种情况。
3. 修复放在首个违反安全不变量的行为 owner，不写 RMUC 坐标、目标编号或场景专用旁路；任一最终 reference
   继续满足离散与 swept footprint collisions=0。
4. 最终 revision 的 P2 六故障独立 domain 达到 6/6，完整 red_box 至少越过目标 5；若未达到，保存首违证据
   并维持确定性零速，不用放宽 footprint、地图或 runner 门禁换取通过。
5. 修改后的 focused GTest、构建、故障矩阵与 red_box 都有原始 artifact；未执行项如实标注。

第一项任务：目标 5 的 reference/actual/stop 证据

- 先在不改算法参数的独立 domain 复现目标 1--5。围绕目标 5 首次进入低净空区域的前后窗口，按同一
  steady/ROS time 基准保存 localization、reference、MPC predicted/executed、`/cmd_vel/selected`、
  `/motion_control`、急停状态和地图 generation。
- 对 reference 与 actual 分别运行同一个离散和 swept footprint evaluator，记录最小 clearance、首冲突
  center/yaw、碰撞 index、相对墙法向误差、实际速度和从首次制动/急停到静止的位移。
- 先回答三个可证伪问题：发布时 reference 是否为零碰撞；actual 是否在跟踪中越过 reference 的安全包络；
  相同 pose 是否因 snapshot/source 更新从 free 变为 occupied。缺少时间配对时先补只读 telemetry 与测试。
- 若 reference 安全而 actual 越界，优先审查 MPC tracking error、执行延迟、速度/加速度限幅和停机包络；
  若 reference 本身不安全，优先审查最终 revalidation 的 snapshot、采样连续性和提交时序；若地图发生变化，
  优先审查 generation/heartbeat 与旧 reference 撤销时序。
- 避免只通过加大 watchdog 时间、缩小 footprint/margin、降低障碍阈值或允许碰撞前缀来绕过首违。

第二项任务：unknown 恢复后的起点冲突

- 使用全新 domain 复现 all-unknown 注入、零速和恢复，不复用 red_box 的机器人状态。保存故障前、故障中、
  ready 恢复和新目标提交四个时刻的 localization pose、footprint cells、各地图来源、source generation、
  adapter publication 与 MINCO local snapshot identity。
- 对恢复后 `8--9` 个碰撞逐格列出来源和值，确认是机器人确实停在 occupied/unknown 边界、ego unknown
  清理范围/方向不足、地图 origin/yaw 转换偏差，还是恢复后的 snapshot 时序问题。
- 恢复时只允许健康条件重新准入；ready 本身不复活旧 reference。若当前 pose 已不具备零碰撞起步条件，
  保持零速并给出结构化失败，不启用未经同等碰撞验证的 escape motion。
- 为确认的 owner 增加最窄回归，包括恢复前后 generation、起点 footprint、旧 reference 失效和新目标准入。

第三项任务：目标 9/10 后续门禁

- 目标 5 与 unknown 恢复首违收敛后，再继续目标 9。先打印 relocation 的 tail/window、全部候选 index、
  collision interval 和拒绝原因；候选优先覆盖窗口边界、冲突带首末与 tail_start，再按弧长确定性补点。
- 历史目标 9 的候选间距会漏掉窄冲突带，且已记录接近路径东向极值 `9.832 m`；单纯把候选数从 6
  调高或缩小 terrain 过报只作为 A/B 诊断。最终方案仍需从 snapshot 与 footprint 几何推导。
- 目标 9 达到零碰撞安全终态后，再用新 domain 独立发送目标 10，并保存起点 footprint、首冲突样本、
  terrain/static/unknown 来源和 snapshot identity。

第四项任务：回归与证据

- 先运行最窄包：ats_navigation_interfaces、ats_rc_esdf、minco_planner、ats_goal_manager、
  terrain_analysis_ext、ats_mujoco_sim。
- 功能测试与既有 lint 分开报告。当前 minco_planner 和 terrain_analysis_ext 有历史 copyright/cpplint/
  format 欠债，建议避免顺带格式化无关文件。
- query_occupancy_grid.py 的 farthest-free --max-distance 默认 0.0 保留旧行为；runner 默认
  FARTHEST_GOAL_MAX_DISTANCE=4.0。保留对应 deterministic regression。
- P2 六个故障分别使用新 ROS_DOMAIN_ID，避免在同一机器人状态中串行注入。
- 每个 fault 保存 ready=false -> emergency_stop=true -> selected=0 -> motion_control=0，恢复后没有新目标时
  旧 reference 不复活。
- red_box 每段记录终点误差、raw/reference 点数、reference/actual footprint collisions、跟踪误差、停止距离、
  失败原因和恢复次数。
- 没有独立 contact evaluator 时，物理接触写为未验证。

交付报告请包含：根因、行为所有者、精确修改文件、关键 diff、实际命令与退出码、focused tests、
六故障结果、red_box 每段结果、未验证项和残余风险。完成后输出 READY_FOR_CODEX_REVIEW，保持 Git 无写入。
```
