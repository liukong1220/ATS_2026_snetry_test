# ATS P6 实车接口连通提示词（串口、行为树、实车 launch 接线、MID360）

本文件是 5.14 实车接口连通性审计之后的下一阶段提示词，直接复制下面代码块作为新对话的第一条消息。它承接 `docs/项目优化文档/nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md` 的 5.12、5.14 与第 6 节 P6，不重复 P1~P4 已完成范围，也不替代 P5 的 MINCO 接线与场地模型整改。

结论前置：**当前不能直接上车。** 自研 MINCO+MPC 链只存在于 MuJoCo 入口与各包自带 launch，实车入口 `bringup.launch.py` 仍走 Nav2 + `trajectory_optimizer`，`/cmd_vel_mpc` 在实车侧没有任何消费者，行为树用的是 `nav2_msgs` action，`GimbalYawStatus` 在实车没有发布者。串口协议本身是车体系 `[vx, vy, wz]`，与四舵轮语义天然兼容，是唯一无需改造的一层。

```text
请继续在工作区 `/home/ats/ATS_2026_snetry_test` 开展 ATS 2026 四驱四转哨兵导航研发。

本轮目标：打通"自研 MINCO+MPC 导航链 ↔ 实车串口底盘 ↔ 行为树决策"的接口连通性，把 MID360 实车链从"不可上车"推进到"可做不通电与抬轮 HIL 验证"。本轮不得进行任何落地行走测试。

必须完整阅读并遵守：
1. `AGENTS.md` 与 `CLAUDE.md` 的全部约束（提交、分支、colcon、破坏性操作、用户文件归属）。
2. `docs/项目优化文档/nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md`，重点 5.9~5.14、第 6 节 P5/P6、第 7 节边界、第 8 节回归入口、第 9 节维护约束。
3. `docs/项目优化文档/nav2移植/p4_real_robot_calibration_preflight.md`（不通电检查、待标定量、分级执行、稳定跟踪准入、行为决策准入、立即停止条件）。
4. `docs/项目优化文档/nav2移植/p5_real_robot_hardening_prompt.md`（P5 仍未完成，本轮不得覆盖或声称已完成）。
5. `docs/视觉与串口桥说明.md`、`docs/行为树决策链路.md`、`docs/启动入口与运行链路.md`、`docs/接口消息与话题约定.md`。
6. 本文件。
7. 五个独立仓库的 git 状态与用户已有改动。
8. 不得读取或修改 `参考/` 与 `minco+mpc_reference/` 下的任何内容，它们不是运行时依赖。

五个独立仓库（各自都在 `develop`，禁止新开分支）：
- 根仓库 `/home/ats/ATS_2026_snetry_test`（`docs/`、`scripts/`、`src/ats_sentry_bringup`、`src/standard_robot_pp_ros2`、顶层规范）
- 导航仓库 `src/ats_sentry_nav`
- MuJoCo 仓库 `src/sim/ats_mujoco_sim`
- 行为树仓库 `src/ats_sentry_behavior`
- loopback 仓库 `src/sim/loopback_sim`

必须保留的用户改动（不得 stage、不得回滚、不得覆盖）：
- 根仓库 ` M .gitignore`
- 导航仓库 `?? sentry_chassis_vel_transform/` 与 `ats_swerve_mpc`/`minco_planner` 下尚未提交的修改
- MuJoCo 仓库 `?? **/__pycache__/`
其余未知修改与未跟踪文件默认属于用户，发现重叠修改先理解并合并，无法安全处理再询问。

开始修改前必须输出：
1. Definition of Done；
2. 精确文件范围（逐个文件路径）；
3. 可执行验证清单（构建、单测、闭环命令与判据）；
4. 当前假设、未验证项、停止条件；
5. 五仓 baseline commit 与用户已有改动清单。
不得只给方案后停止。

总原则：
- 一切以"仿真通过但实车失控/损坏"为最高优先级判据；任何门禁不得通过放宽 unknown、frame、footprint、执行器物理限值或 stale 安全语义来通过。
- 区分 `已实现` / `已测试` / `已验证` / `未实现`，未验证假设标注 `[Confidence: High/Medium/Low]`。
- 用户可见输出使用中文，数学使用严格 LaTeX。
- 不得替换：Point-LIO 定位、`src/ats_sentry_nav/ats_rog_map`、RC-ESDF 语义、JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair、`ats_swerve_mpc`。
- 四舵轮控制量恒为车体系 `[vx, vy, wz]`，状态恒为世界系 `[x, y, yaw]`；禁止迁入差速、ICR 或 `vy=0` 约束。
- 不得从 `/rog_map/esdf` 可视化 `PointCloud2` 反解析数值距离场。
- `localization_fusion` 继续独占 `map -> odom`、`/localization` 与定位健康/epoch；`/rc_esdf/planning_grid` 始终只能有一个发布者。
- `ExecutionCommand` 继续是 MPC 的唯一执行授权，legacy `Path` 与 `emergency_stop=false` 不得重新授权。
- 本轮全程禁止落地行走：只允许不通电检查、抬轮/断执行器 HIL 与台架单自由度，任何"上车跑一圈"都必须先满足第五节门禁。

零. 实车 launch 与参数接线（最高优先级，当前完全缺失）
1. 现状：`minco_planner_node`、`ats_goal_manager_node`、`ats_swerve_mpc_node` 只出现在 `src/sim/ats_mujoco_sim/launch/mujoco_navigation.launch.py`、`rmuc_2026_mujoco.launch.py` 与各包自带 launch 中；`src/ats_sentry_nav/ats_nav_bringup/launch/` 下八个 launch 文件零引用。实车入口链 `src/ats_sentry_bringup/launch/bringup.launch.py` -> `rm_navigation_reality_launch.py` -> `bringup_launch.py` -> `navigation_launch.py` 仍启动 Nav2 controller/planner/bt_navigator/lifecycle 与 `trajectory_optimizer_node`、`trajectory_speed_governor_node`。必须新增实车 Nav2-free profile，使 `minco_planner`、`ats_goal_manager`、`ats_swerve_mpc` 在实车入口可被启动，且与 Nav2 对照 profile 互斥（显式 `launch_nav2:=false`，运行图中不得出现 Nav2 节点、lifecycle manager 或 `/plan`）。
2. 参数缺失：`src/ats_sentry_bringup/params/node_params.yaml` 与 `src/ats_sentry_nav/ats_nav_bringup/config/reality/nav2_params.yaml` 都没有 `minco_planner` / `ats_swerve_mpc` / `ats_goal_manager` 段。必须补齐实车参数段，并明确单一权威来源（包内 yaml 还是 bringup yaml），禁止两处同时给出不同数值。
3. `src/ats_sentry_nav/ats_swerve_mpc/config/ats_swerve_mpc.yaml:3` 当前 `use_sim_time: true`（注释已写"实车应改为 false"）。实车 profile 必须 `use_sim_time: false`，并对全链节点做一次 `use_sim_time` 一致性审计（Point-LIO、localization_fusion、rog_map、adapter、minco、goal manager、mpc、行为树、串口）；混用会让 `0.5 s` lease 与 stale 判据失效。
4. 命令通路断裂：MPC 输出 `/cmd_vel_mpc`，实车链却是 `cmd_vel_nav2_result -> fake_vel_transform -> cmd_vel_gimbal_yaw_odom -> chassis_vel_transform -> /cmd_vel -> 串口 speed_vector`，`/cmd_vel_mpc` 在 `src/ats_sentry_bringup` 与 `src/ats_sentry_nav/ats_nav_bringup` 内没有任何订阅者。必须给出唯一的实车命令通路并落实：优先让 MPC 直接产出车体系 `/cmd_vel`（或经一个只做限幅/看门狗、不做坐标旋转与增益放大的薄 bridge），并显式说明 `fake_vel_transform` 与 `chassis_vel_transform` 在 Nav2-free profile 中的去留。任何保留方案都必须证明不会在 MPC 之后改变 `[vx, vy, wz]` 的数值或方向。
5. `GimbalYawStatus` 实车无发布者（仅 `src/sim/ats_mujoco_sim/ats_mujoco_sim/sim_node.py` 发布），而 `ats_goal_manager` 与 `ats_swerve_mpc` 都是 `require_gimbal_status: true`、超时 `0.5 s`。实车必须新增由串口云台关节反馈驱动的真实 `GimbalYawStatus` 发布者（含 ack 语义与 stale 判据），或在受控 HIL profile 中显式关闭该要求并写明关闭期间禁止 `BODY_YAW_FOLLOW`。禁止伪造 ack。
6. 交付一张实车 topic/action/TF 连通表：每个话题的唯一发布者、订阅者、frame、QoS、freshness 阈值，以及 `map->odom`、`odom->gimbal_yaw_odom`、`base_footprint->base_link` 的唯一 TF 发布者。

一. 串口链（`src/standard_robot_pp_ros2`）
1. 已确认兼容项，保持不动：`include/standard_robot_pp_ros2/packet_typedef.hpp:216-219` 的 packed `speed_vector {float vx; float vy; float wz;}` 与 `src/standard_robot_pp_ros2.cpp:962-967` 的 `linear.x/linear.y/angular.z -> vx/vy/wz` 映射天生是车体系全向命令，与四舵轮语义一致，不需要引入差速或 `vy=0`。
2. 零命令保持与五级归零冲突（高危）：`config/standard_robot_pp_ros2.yaml:24-26` 的 `enable_transient_zero_cmd_hold: true`、`transient_zero_cmd_hold_timeout_ms: 50` 会在收到零命令时继续下发上一条非零 twist（`src/standard_robot_pp_ros2.cpp:932-960`），最多抑制 `50 ms` 的确定性归零，与 `ExecutionCommand STOP -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0 -> 四轮 0 rpm` 直接矛盾。必须让急停/STOP 路径绕过或立即失效该保持逻辑（例如区分"通信抖动导致的瞬时零"与"授权归零"，后者不可保持），并给出台架实测的归零延迟。
3. `cmd_vel_watchdog_timeout_ms: 300` 与 MPC `control_rate_hz: 20.0`（周期 `0.05 s`）、`emergency_stop_timeout: 0.5`、`0.5 s` lease 之间必须给出显式时序关系：底盘看门狗必须严于或等于上层 lease，不能出现"上层已判超时、底盘仍在执行"的窗口。
4. `src/standard_robot_pp_ros2.cpp:306-322` 的 `serialPortProtect()` 仍是 `@TODO`（保持连接、断开重连、异常处理均未实现）。必须实现断开检测、重连与重连期间的确定性零速度，并在重连成功前禁止恢复执行授权（恢复必须走新 epoch/generation/序号）。
5. 符号、单位与量纲审计：`[vx, vy, wz]` 的正方向、单位（`m/s`、`rad/s`）、轮位 `(±0.270, ±0.270) m`、滚动半径 `0.0425 m`、`450 rpm`/`120 rpm` 折算得到的 `max_wheel_speed=1.6689711 m/s`、`max_steer_rate=10.4719755 rad/s` 必须与下位机固件口径逐项对照并记录在文档，任何不一致按停止条件处理。
6. 出口限幅冲突（高危）：`src/ats_sentry_bringup/params/node_params.yaml:248-261` 的 `chassis_vel_transform` 允许 `max_linear_speed=4.6 m/s`、`max_linear_accel=3.6 m/s^2`、`max_angular_speed=4.2 rad/s`，远高于 MPC 可行域（`max_vx/vy=1.5`、`max_ax/ay=2.0`、`max_wz=2.0`、`max_awz=3.0`）。出口级不得比授权级宽松：必须收紧到不宽于 MPC 限值，或在 Nav2-free profile 中移除该级。
7. 静默降级（高危）：`sentry_chassis_vel_transform/src/chassis_vel_transform.cpp` 的 `pass_through_without_yaw: true` 在缺少 `serial/gimbal_joint_state` 时不做旋转直通命令，等于在定位/云台反馈缺失时仍然放行运动。实车必须改为缺反馈即零速度并输出中文 WARN，或证明 Nav2-free profile 下该节点已不在通路上。注意该目录是用户未跟踪文件且自带嵌套 `.git`，不得 stage、不得覆盖，如需修改先与用户确认归属。
8. `fake_vel_transform` 的 `cmd_spin` 会把行为层角速度直接叠加到输出 `angular.z`，是绕过 MPC 的车体角速度入口。正式 profile 必须关闭该叠加或移除该级。

二. 行为树接口迁移（`src/ats_sentry_behavior`）
1. 接口不匹配（高危）：`plugins/action/send_nav2_goal.cpp:12-53` 与 `plugins/action/send_nav_through_poses.cpp` 使用 `nav2_msgs/action/NavigateToPose`、`NavigateThroughPoses`，默认 action 名 `"/navigate_to_pose"`（`include/.../send_nav_through_poses.hpp:80`、`params/sentry_behavior.yaml:184`、`behavior_trees/dev_rm.xml`）；而 Goal Manager 提供的是 `ats_navigation_interfaces/action/NavigateToPose`，服务名 `/ats_navigate_to_pose`（`src/ats_sentry_nav/ats_goal_manager/src/ats_goal_manager_node.cpp:154-155, 191-218`）。两者消息定义不同（ATS 版 result 携带 `result_code` 0~7、`final_pose`、`final_distance`；feedback 携带 `state` 0~5、`goal_id`、`distance_remaining`、`elapsed_sec`）。必须新增 ATS action BT 节点并把正式 RMUC/RMUL 树切到 `/ats_navigate_to_pose`，旧 Nav2 节点只保留在命名清楚的对照 profile。
2. 按 5.12.4 锁定 goal、feedback、result、cancel、halt、preempt、timeout、server unavailable/restart 与 result-code 映射；`SendNavThroughPoses` 当前是 `BT::SyncActionNode`、发出 goal 立即返回 `SUCCESS`、无 halt 回调，必须收敛为可 halt/可取消的异步语义，且不得在 sync tick 中无界等待 action server。
3. 按 `docs/项目优化文档/nav2移植/p4_real_robot_calibration_preflight.md` 行为决策准入：正式 profile 只能通过 ATS action 驱动 Goal Manager；禁止 `PublishTwist`、禁止 `cmd_spin` 在 MPC 之后叠加车体 `wz`、禁止用 `IsPathGoalReached` 的位置容差替代 ATS action 的终端成功（位置、wrapped yaw、终端线/角速度、dwell 全部由 Goal Manager 判定）、禁止伪造云台 ack。`src/ats_sentry_behavior_server.cpp` 中 `decision.topics.cmd_vel` 这条直发底盘通路必须从正式 profile 移除。
4. 行为层输入改为正式权威源：`/rc_esdf/planning_grid` 与 `/localization`，替换硬编码的 `global_costmap/costmap`、`odom`、`odometry`；并为地图、定位、action 输入补显式 topic/QoS/freshness 参数。
5. 补聚焦功能测试：主树优先级、action halt/cancel、迟到 result、waypoint 状态机；当前行为仓 `BUILD_TESTING` 只有 ament lint。README 里"`/navigate_through_poses` 是统一执行接口"必须随迁移修正，不得先改文档声称已接入。
6. 门禁顺序不可颠倒：loopback 决策场景先固定 revision/config/seed 重复 `20` 次无非确定性 branch/action 序列，再用完全相同的树、参数与场景输入进入 MuJoCo 关键场景 `10/10`。

三. MID360 与定位链实车化
1. 配置一致性已确认：`src/ats_sentry_nav/livox_ros_driver2/config/MID360_config.json` 与 `src/ats_sentry_nav/ats_nav_bringup/config/reality/mid360_user_config.json` 的主机 `192.168.1.50`、雷达 `192.168.1.177` 与端口对齐；`src/ats_sentry_bringup/params/node_params.yaml:59-73` 使用 `xfer_format: 4`、`frame_id: front_mid360`。必须实测确认网口静态 IP、`cmdline_input_bd_code` 与实物序列号一致。
2. 外参未标定（高危）：两份 MID360 配置的 `extrinsic_parameter` 全为零。云台上安装的 MID360 到 `base_link` 的平移与旋转必须实测标定并写入唯一权威位置（URDF/静态 TF 与驱动外参不得同时给出不同值），否则 Point-LIO 输出与 footprint/净空判据不在同一几何基准上。
3. Point-LIO 参数双份：`src/ats_sentry_nav/point_lio/config/mid360.yaml` 与 `node_params.yaml:85-110` 的覆盖值不一致（`filter_size_map` `0.5` vs `0.15`、`ivox_nearby_type` `6` vs `18`、`blind` `0.5` vs `0.3`、`cut_frame_time_interval` `0.1` vs `0.05`）。必须固定实车生效值并在运行时打印确认，禁止靠加载顺序隐式决定。
4. 实车静态采集（不通电、不运动）：固定放置采集 rosbag，给出点云频率、每帧点数、IMU 频率、时间戳单调性与 `timestamp_unit` 正确性、`map->odom` 与 `/localization` 的静止漂移与 `LocalizationStatus` 状态分布；确认 `localization_fusion` 是 `map->odom` 与 `/localization` 的唯一发布者。
5. 云台旋转时的定位鲁棒性必须单独评估（MID360 装在云台上，`odom->gimbal_yaw_odom` 与 `fake_vel_transform` 的 `\psi_0-\psi` 约定必须与实车关节零位一致）；该项在抬轮 HIL 阶段完成，不得推迟到落地。

四. 分级上车计划（本轮只允许执行第 1、2 级）
1. 不通电检查（复用 `docs/项目优化文档/nav2移植/p4_real_robot_calibration_preflight.md`）：TF 唯一发布者、`/planner/execution_command` 为唯一授权、符号/单位/轮位/滚动半径审计、物理急停与远程急停与 Goal Manager/lease/watchdog 急停演练（执行器断开）。
2. 抬轮 HIL：车轮离地或执行器断开，跑通"MID360 -> Point-LIO -> localization_fusion -> rog_map/adapter -> Goal Manager -> JPS/MINCO -> ExecutionCommand -> MPC -> 串口"整链，验证五级归零实测延迟、断串口重连、定位 stale/丢失、云台 ack 缺失、地图未就绪五类注入下均确定性零速度。
3. 台架单自由度（下一轮，需第 1、2 级全绿）：依次 `vx`、`vy`、`wz`，每次只放开一个自由度，标定轮 RPM↔车体速度、轮/舵动力学、制动距离与命令时延。
4. 受控低速地面（更后一轮）：必须先满足稳定跟踪准入的净空预算
   `C_min(t) > e_track_99 + e_loc_99 + v(t)*tau_99 + d_brake(v, slope) + m_map`
   其中 `tau_99` 必须来自实测端到端时延分解（传感器→定位→规划→授权→MPC→底盘），不得用估计值代入。
5. 任一级出现下列情况立即断电并停止本级：命令方向与实际运动不符、归零后仍有残余运动、定位跳变、串口断连未归零、出现第二个 yaw authority。

五. 验证与准入
1. 构建：`MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1`。
2. 单测：`colcon test --base-paths src --packages-select <targets>`；`colcon test-result --test-result-base build/<package> --verbose`。注意 `minco_planner` 存在既有 `clang_format/copyright/cpplint` 债务，只以功能 gtest 与本轮新增测试为判据，并显式报告债务未变差。
3. launch 语法：`python3 -m py_compile <changed_launch_files>`；空白检查 `git diff --check`。
4. 仿真闭环（每例独立 `ROS_DOMAIN_ID` 与独立 MuJoCo 启动，关闭 viewer/RViz）：
   `PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh`
   以及 `TEST_PROFILE=rectangle GOAL_TIMEOUT=120` 与 `scripts/test_mujoco_swerve_dynamics.sh`。任何实车 profile 改动都必须先在 MuJoCo 复跑，证明未破坏既有安全契约。
5. 准入判据沿用 5.11.4：安全契约 `10/10`；终端 p95 位置 `<=0.08 m`、yaw `<=0.10 rad`、线速度 `<=0.05 m/s`、角速度 `<=0.10 rad/s`，停稳 dwell `>=0.30 s`；固定 revision/config/seed 后关键场景重复 `10/10`。阈值在候选优化前冻结，失败样本全部保留。
6. 实车部分只允许声明"不通电检查完成"与"抬轮 HIL 完成"，不得由 HIL 结果推导落地跟踪精度或零碰撞。

六. 提交与交付
1. 按内容拆分提交，使用详细中文标签（`[安全]`、`[接口]`、`[行为]`、`[控制]`、`[仿真]`、`[文档]`、`[规范]`），只显式 stage 本轮列出的文件；禁止 `git add -A`、`git add .`。
2. 只在实际修改的仓库提交并普通 push 到 `origin/develop`；推送失败保留本地提交并报告远端错误，不做 force push。
3. 必须更新 `docs/项目优化文档/nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md`：滚动窗口、5.14 之后的新验证记录、第 6 节 P6 状态、第 7 节接续入口；同时更新 `docs/启动入口与运行链路.md`、`docs/接口消息与话题约定.md`、`docs/行为树决策链路.md`、`docs/视觉与串口桥说明.md` 中受本轮改动影响的部分。
4. 最终报告必须列出：改动文件清单；构建/单测/闭环实测数字；实车侧每项检查的实测值；每项结论的 `已实现`/`已测试`/`已验证`/`未实现` 标注与 Confidence；本轮不可声明项；下一阶段建议顺序。
```
