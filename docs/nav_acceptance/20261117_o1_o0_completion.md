# O1 / O1b / O0 优化与验证账本

本次授权为完整 O1（含 O1b）与 O0。验收状态只随实际证据更新，不能由构建或局部修复推导全场通过。历史失败见 [11 月 17 日前一轮记录](20261117_o1a_o0.md)。

## 基线与完成门

本机 `ats-OMEN-by-HP-Gaming-Laptop-16-wf0xxx`，工作区 `/home/ats/ATS_2026_snetry_test`。起始三个仓库均为干净的 `develop`：根仓 `9404ee2a93a315804f49da15a0923c8b3d150753`，导航仓 `a51ce3a273dfa5ea7916f11405774bae28215a44`，仿真仓 `0b0d5df449ccff3c1e0145f1b33301657bd34541`。其他嵌套仓的未知修改不纳入。

- O1-A：V1 参考安全与完整证据、不可达终止契约、T1 known-free 与重规划活性。
- O1-B：2027 完整 64 点与 A–K，包括保存质量、动态排除、重载、20 次实体障碍延迟和独立故障。
- O1 完整设计：增量 EDT、比赛先验与障碍层分离及 geofence、动态对象跟踪与前向膨胀；按报告要求先收敛地图/安全失败。
- O1b：保持唯一快照 owner 和数值语义，核验不可变版本、epoch/租约、按需显示及最终 Release 性能。
- O0：同任务连续 EXECUTE 换轨，前后各 0.25 s 仿真时间，参考/MPC/仲裁/底盘/真值分别统计跌速，合格巡航严格 <0.2 m/s；无事件为未触发。

## 第一轮范围与验证

导航仓：`ats_navigation_interfaces/msg/ExecutionCommand.msg`、`ats_goal_manager/src/ats_goal_manager_node.cpp`、`ats_goal_manager/test/test_goal_manager_fast_maps.py` 及必要直接测试。保留原规划 publication，不把重新验证的地图序号写成 MINCO 原始来源；单独携带最终验证地图身份和完整三维 TF。

根仓：`scripts/nav_tracking_recorder.py`、`scripts/analyze_nav_tracking.py`、`scripts/test_mujoco_minco_mpc_chain.sh`、相应跟踪/故障测试；本账本、主报告和 2027 最终方案。先补旧实现能失败的离线反例，再修所有者。

可执行验证：`python3 -s scripts/test_analyze_nav_tracking.py`、新证据契约测试、`python3 -s scripts/test_mujoco_fault_contracts.py`、`bash -n scripts/test_mujoco_minco_mpc_chain.sh`、`./build.sh`、直接 CTest 和三个仓库的 `git diff --check`。创建 ROS 节点的测试与仿真均受下述隔离前置约束。

## 假设、限制与风险转入

V1 的坐标问题尚未定因：旧记录器丢失 z/roll/pitch，并使用 tick TF；同时执行权威可在更新地图复检候选，却只发布原规划地图序号。需要把最终验证身份、参考和同一次验证的 TF 绑定后再复核，不回写历史失败为通过。

当前 domain 审计覆盖 0..232，排除项不变。根据 AGENTS.md 6.5，已向用户请求本轮历史编号复用的隔离安排；明确答复前不启动 ROS 图、DDS 单测或仿真。WLAN 已核验为 `wlo1`，只用工作区 DDS 文件，不改全局。

运行出现实际足迹冲突、身份/时间失效、LOST 或真值超差时，安全退出该次运行并保留失败，继续审查修复。known-free、footprint、健康期限、STOP 屏障和旧参考拒绝门保持有效。未带 RViz 的画面项和无独立评估的物理接触均不能写通过。
