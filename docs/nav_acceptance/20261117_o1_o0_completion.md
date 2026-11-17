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

当前 domain 历史审计覆盖 0..232，排除项不变。用户已明确授权本轮复用历史编号，执行[本轮清场规则](20261117_domain_reuse_rules.md)。WLAN 已核验为 `wlo1`，只用工作区 DDS 文件，不改全局。每次 guard 同时核验进程和 UDP 端口，退出只清理本轮身份匹配的子进程。

运行出现实际足迹冲突、身份/时间失效、LOST 或真值超差时，安全退出该次运行并保留失败，继续审查修复。known-free、footprint、健康期限、STOP 屏障和旧参考拒绝门保持有效。未带 RViz 的画面项和无独立评估的物理接触均不能写通过。

## 已执行与当前结论

证据根目录为 `log/nav_acceptance/20261117_o1_o0_completion`。两次 `./build.sh` 均完成 40 包 Release 构建；d20 的 Goal Manager 13 组测试全部通过，d21 的 MINCO 8 组、ROG 10 组、RC-ESDF 1 组、MPC 6 组窄回归通过，两套 launch `--show-args` 通过。上述为选定功能回归，不代表全包 lint 通过。新增证据契约 7 例、domain guard 3 例、故障夹具 11 例和原跟踪分析/O0 离线回归通过。

本轮修复把最终验证地图 publication/source 与完整三维 TF 绑定到 EXECUTE，同时保留 MINCO 原始规划来源；记录器保留 SE3、几何/距离载荷身份及全程载荷，分析器按该证据复核。不可达夹具绑定同一个目标，接受实际 JPS/A* 后端，并严格区分先到期的 goal timeout 与有界重规划预算终止，继续核验零速及旧参考不复活。该夹具仍需最终版本独立运行验证。

V1 d22（N=1，lio、有先验、未带 RViz）运行失败，不能计对称点速度通过：ATS action 返回 code 6，`localization did not recover before deadline`，定位终态距离 10.635 m。16 条已执行参考 q1 全部通过；q2 共 30 个冲突 tick，最长 24 tick / 12 个快照，持续冲突段 2 个；没有缺少快照载荷或 frame 配对的 tick。892 个快照、1161 个来源层载荷均未截断。MuJoCo 接触计数增量为 0，独立物理接触评估仍未验证。

O0 记录器在目标下发前就绪，完整保存 d22 换轨前后窗口：8 次换轨，合格巡航事件 0，主要分类为里程计接收超时、参考减速和安全状态切换；跌速指标未触发。参考 horizon 峰值 0.730416 m/s，自主/仲裁命令峰值 0.492536 m/s，底盘指令峰值 0.492536 m/s，独立真值峰值 0.449983 m/s，高速请求未触发。曲线和逐事件分类在 `v1_d22/o0/`。

T1 历史 d8 的离线组件回放只用于定位，不能代替闭环。最初误把保存的根参数 0.07 m 当作运行覆盖，产生全部 unknown 的错误回放；保留该产物并废止其根因结论。按 launch 和静态 TF 的 0.18 m 重放后，目标列 `(90,143)` 早期 20 帧 free、107 帧 unknown，2.699995002 s 收到低于已连接地面约 0.29 m 的回波后持续触发不连续下层支撑拒绝。离线场景射线确认同一位置存在上层平台和下层地面。完整 epoch/fusion 状态未重放，不能声称在线等价或据此清除 unknown。

下一组先复现 d22 的参考平面问题：规划点 z=0 经非平面 TF 变换后，控制参考的高度与实际机体高度不同，可能造成厘米级 XY 偏移。仅修改参考转换及运行期复检，保留原安全门；倾斜 TF、往返投影、缺失/非法 TF 反例通过后重新构建和运行 V1。O1-A、O1-B、O1 完整设计及 O0 总体仍未完成。
