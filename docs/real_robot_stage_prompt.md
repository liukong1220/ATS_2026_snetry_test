# ATS 下一阶段研发提示词：仿真收尾 → 实车

你是 ATS 2026 四驱四转舵轮哨兵导航链的证据驱动代码修改者。工作目录
`/home/kong/ATS_2026_snetry_test`，先读根目录 `AGENTS.md`、`docs/minco_joint_optimization_prompt.md`
与本文件。根仓库、`src/ats_sentry_nav`、`src/sim/ats_mujoco_sim`、`src/standard_robot_pp_ros2`、
`src/sim/gazebo_simulator`、`src/sim/loopback_sim`、`src/sp_vision25` 都是独立 Git 仓库，分别提交。
本阶段分两段：A 段把仿真验收做完，B 段进入实车。A 段未通过不得进入 B 段。

## 当前状态（2026-09-29，domain 87，红点目标 map (10.36, 1.49)）

- 链路：goal_manager → minco_planner（JPS → 整轨迹 MINCO 联合优化 → planYaw → 矩形足迹门禁，
  失败时足迹 yaw 补解 / 局部修复）→ ats_swerve_mpc → cmd_vel_arbiter → MuJoCo。
- 车体 0.58 x 0.58 m（中心到边 0.29，到角 0.41）。仿真 safety_margin 0.02、jps_safe_distance 0.44；
  实车 0.05 / 0.54。引用净空数字时要注明是否已含 safety_margin。
- 地图：MuJoCo 场地与 28 x 15 m 实际场地 1:1。规划栅格 `/rc_esdf/planning_grid`（0.1 m，
  origin (-3.58, -9.44)，map 系）；pgm 的 yaml origin 不是 map 系，不要用 pgm 坐标判断路线。
  规划栅格上到红点最宽的路线是南侧路线（瓶颈 0.50 m），不存在更宽的直连路线。
- 已提交：minco_planner f239715（足迹 yaw 不动点补解，`footprint_yaw_refinement_rounds`）；
  ats_mujoco_sim 7788fed（joint_footprint_clearance 0.08、edge_samples 4、time_budget 50 ms）。
- 仿真已验证：修改前 0 条规划下发；修改后 opt_3 下发 4 条规划，首条参考 k95 1.70、
  曲率符号翻转 8、curvature_tv 25.9（基线 base_2：k95 4.02、翻转 48、tv 223.7），
  已进入南侧墙尖下方约 1.05 m 宽的通道。gtest 20/20，validate_navigation_config PASS。
- 未解决：opt_3 在 (3.55~3.93, -6.30) 浅接触后被门禁拒绝，停在 (3.23, -6.17)，未到达。
  bag 分析：进入通道时实际 yaw 0.5~0.9 rad，参考 yaw -0.11~0.30 rad，yaw 跟踪滞后
  0.4~0.6 rad；进通道前几次重规划使跟踪误差达 0.2~1.0 m。单次规划 solver_wall 约 225 ms，
  joint 优化 51.8 ms 以 time_budget 退出。日志中的 kmax 是按时间采样的，存在尖峰假象。
- 实车 profile `src/ats_sentry_bringup/params/node_params.yaml` 未同步：joint_footprint_clearance
  0.03、edge_samples 1、time_budget 15 ms；没有写 `footprint_yaw_refinement_rounds`，
  代码默认值 2 会生效。

## A 段：仿真收尾（每完成一步提交并 push）

1. 通过南侧墙尖通道。先查 yaw_spline_planner / planYaw 在窄通道内的 yaw 需求与 MPC yaw
   权限，优先在规划侧降低 yaw 变化（例如窄通道段保持 yaw 与通道方向对齐或不变），
   再看减少进通道前的重规划扰动。不得通过放宽门禁、调高 `ego_contact_max_depth`、关急停
   或提高速度上限来解决。
2. 指标脚本改为按弧长重采样计算曲率（k95、kmax、curvature_tv、符号翻转），基线与新方案
   用同一脚本重算。
3. 验收（每次冷启动、结束后杀净进程并用 pgrep 确认 leftover=0，仿真时 `use_rviz:=true`、
   `DISPLAY=:1` 供用户观看）：基线 ≥3 次、新方案 ≥5 次；要求 k95 ≤ 1.5、kmax ≤ 3.0、
   符号翻转 ≤ 基线、curvature_tv < 基线、南侧走廊不变差，并附 RViz Orbit 截图。
   记录到达率、终点误差、接触次数、solver_wall/joint_wall、time_budget 退出比例。
4. 实车 profile 同步评估：逐项说明 joint_* 与 `footprint_yaw_refinement_rounds` 是否迁入
   node_params.yaml，实车 margin 0.05 下重新核对净空目标；每迁一项都要有仿真证据，
   默认 fail-closed。仿真专用开关（retain_safe_reference_on_snapshot_change、
   progress_along_reference、escape_from_contact_enabled、local_repair_enabled、
   goal_pose_admission_enabled、endpoint_clearance_relaxation_enabled、ego_contact_max_depth、
   terminal_yaw_relocation_enabled）在实车 profile 中保持关闭或更保守，除非另有证据。
5. 回归：Gazebo 链（新 gazebo_robot_tf_relay、雷达外参 (-0.1, 0.245, 0.325, roll 75°, yaw -161°)）
   与 loopback 链（cmd_vel_arbiter → /cmd_vel/selected）各跑一次冒烟，确认 TF 树唯一 owner、
   执行端只消费 /cmd_vel/selected。

## B 段：实车（按顺序，前一步未通过不得进入下一步）

1. 静态检查：Mid360 外参与 `front_mid360` 静态 TF、串口 `serial/link_up` 与
   `serial/gimbal_joint_state`、先验 PCD 路径、`base_link`/`odom`/`map` TF 树唯一 owner；
   串口桥 `cmd_vel_topic` 为 `/cmd_vel/selected`（standard_robot_pp_ros2 0dbf055 起生效）。
   用 `ros2 run tf2_tools view_frames`、`ros2 topic hz` 记录频率与延迟。
2. 定位：静止与推行两种工况验证 Point-LIO + small_gicp 重定位，记录跳变次数、重定位耗时、
   与场地标志点偏差；按实车 LiDAR 高度核对 ROG-Map z 窗口与 virtual ceil。
3. 地图：比对实车 ROG-Map / terrain_analysis 与静态图在墙、立柱、坡沿处的过报/漏报，
   量化足迹门禁误拒率，重点看南侧薄墙尖。
4. 开环规划：底盘断电或架空，只发目标不执行，检查参考轨迹、足迹门禁、yaw 补解轮数、
   单次规划耗时（实车 CPU 上 joint time_budget 是否够用），与仿真对比曲率指标。
5. 低速闭环：`max_velocity` 先限 0.5 m/s（改速度上限前先征得确认），遥控急停随时可用，
   逐档放开到 1.0、2.0 m/s；每档记录横向误差、yaw 误差、MPC 求解耗时、cmd_vel 与实测轮速。
6. 单目标 (10.36, 1.49) 实车 ≥5 次连续成功，再讨论提速或多目标。

## 强制工作方式

- 修改前先给出 DoD、文件范围、验证命令与风险；安全门禁、急停、速度上限相关改动先征得确认。
- 不得为了到达率放宽足迹门禁、关闭急停或调高 `ego_contact_max_depth`。map unready/stale、
  定位跳变、串口断链、unsafe trajectory、MPC failure 必须确定性零速。
- 构建：`colcon build --base-paths src --packages-select minco_planner ats_mujoco_sim ats_sentry_bringup`
  （按需加包）；不得用 `-UFETCHCONTENT_SOURCE_DIR_QDLDL` 重建 `ats_swerve_mpc`。
- 验证：gtest（`build/minco_planner` 下 ctest，排除 lint 项）、`python3 scripts/validate_navigation_config.py`、
  `git diff --check`。包级 cpplint/clang_format 是历史失败，不能写成"全量测试通过"。
- 每次测试使用隔离 `ROS_DOMAIN_ID`，保存 rosbag（`/odometry`、`/cmd_vel`、`/minco/raw_path`、
  `/minco/reference_path`、`/rc_esdf/planning_grid`、TF）与完整日志；测试后杀净进程。
- Git：每完成一阶段提交并 push；只暂存明确的文件，不用 `git add -A/.`、`reset --hard`、
  `checkout --`、force push；作者只用 liukong1220 <1625038134@qq.com>，不加任何 Co-Authored-By；
  nav/sim 仓库推 `develop`。提交信息用中文详写，带 [接口]/[安全]/[仿真]/[文档] 标签。
- 结束时清理进程与 /tmp 临时文件。

## 报告要求

中文报告，区分"实车已验证""仿真已验证""已实现未运行""推断"；给出每次运行的日期、
速度上限、成功/失败与原因、bag 路径、指标表；列出各仓库 commit 与 push 结果及未覆盖范围。
