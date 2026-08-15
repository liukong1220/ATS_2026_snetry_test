# 下一阶段新对话提示词：Gazebo 定位新鲜度与规划闭环准入

下面内容可直接作为新对话的首条提示词。

[$develop-robot-vision-navigation]

继续 /home/kong/ATS_2026_snetry_test 的 ATS 四驱四转哨兵导航优化。P0 已完成，本轮从
P1 Gazebo localization freshness 开始。完整读取：

1. AGENTS.md
2. docs/项目优化文档/ATS导航剩余优化总TODO.md
3. docs/nav2_to_3desdf_minco_mpc_optimization_direction.md
4. docs/ats_swerve_mpc_ltv_qp_backend_admission.md
5. dependencies.repos 与 dependencies.lock.repos

按门禁持续执行：定位 -> 最小修改 -> 构建 -> 聚焦单测 -> 隔离 Gazebo 闭环 -> 文档 ->
分仓中文提交 -> SSH push。停止条件触发后保存 raw artifact 和 first violation，停止后续高风险阶段。

当前已验证基线：

- P0 已完成：最终 clean workspace 的 vcs import 成功取得 22 个仓库；锁文件由
  vcs export --exact -n 生成。
- rmoss_gz_resources 使用 humble=b5c759f08844dfda19c79aa870866ace8d4c7b3a；
  ats_mujoco_sim、teleop_gimbal_keyboard 使用已验证的用户 SSH URL。
- Gazebo fork a28ccd20428ffc4bdd7fbbc22fee884fa1db72eb 在 clean checkout 中通过依赖闭包构建，
  rmu_gazebo_simulator CTest 4/4 与 ats_gazebo_nav.launch.py --show-args 均通过。
- 历史 domain 230 的 /localization interval p50/p95/p99 为 0.371/0.994/1.612 s，
  adapter 反复 ready=false，action fail-closed。
- 尚未找到该 freshness 首个违反者；不能直接归因 LiDAR、Point-LIO、DDS、Gazebo RTF、TF、
  localization_fusion、recorder 或资源争用。
- MINCO production node 仍未将 InitialKinematicState 传到 center、footprint、fallback、
  repair 四条路径；geometry telemetry 还不是完整 production gate。
- TEST_PROFILE 尚未真正控制 straight/corner/S/narrow/red-box 场景，GOAL_YAW 未进入 action payload。
- 默认保持 solver_mode=ilqr；qp_shadow 只诊断，solver_mode=qp 继续拒绝。

开始前先报告 DoD、精确文件范围、验证清单、假设/未验证项/停止条件，并核对根仓、导航仓、
MuJoCo、Gazebo fork、机器人描述仓的 branch/HEAD/upstream/remote/status。

保护用户内容，禁止读取为设计依据、修改、删除、暂存或提交：

- src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py
- src/ats_sentry_nav/ats_swerve_mpc/求解器.md
- src/sim/gazebo_simulator/rmu_gazebo_simulator/include/rmu_gazebo_simulator/evidence_statistics.hpp
- src/sim/gazebo_simulator/rmu_gazebo_simulator/scripts/ats_bridge/gz_livox_bridge.py

禁止 git add .、git add -A、破坏性恢复或 force push。Gazebo 只能写用户 origin/main，
禁止写 upstream。未知改动均视为用户内容；若与必要 owner 重叠，停止并报告。

阶段 P1：定位 Gazebo localization freshness 的首个违反者。

1. 在已有 Gazebo C++ evidence recorder 中低开销订阅并测量：
   /clock、/lidar_odometry、/odometry、/localization、/localization/status、
   /rog_map_adapter/ready。
2. 每级记录 steady_clock wall arrival interval p50/p95/p99/max、ROS stamp interval、
   stamp age、重复/倒退、最长 gap、消息数；记录 RTF、TF lookup failure、关键进程
   CPU/RSS/thread/context switch、DDS queue/drop 与 callback blocking。
3. 建立字段级 contract table：
   /clock -> /lidar_odometry -> /odometry -> /localization -> status -> adapter
   必须涵盖 frame、clock、QoS、producer、consumer、timeout、health gate、fallback。
4. 每次只改变一个因素，使用全新 ROS domain 和固定 60 s 窗口做 A/B：
   headless、RViz、viewer、camera sensor、LiDAR profile、recorder/logging。
   禁止高频 ros2 topic echo 干扰被测链。
5. 找到最早违反 freshness 的行为 owner 后，只修改该 owner，并补最窄 deterministic regression。
   禁止提高 odom/localization/map/reference timeout、adapter lease 或 projection deadline；
   禁止 Ground Truth 接管正式 /localization。

P1 DoD：低负载 headless 连续至少 60 s，/localization p99 interval < 0.25 s，
无 >0.5 s gap，stamp 不倒退，status 持续 TRACKING，adapter 不因 localization 抖动
变为 ready=false；随后两个独立 ROS domain 的 straight action 成功，并记录终点误差、
规划/速度唯一 owner 与收尾零速。

每个 domain 前审计残留导航进程、ROS domain、load、CPU、内存、swap、RTF、TF owner、
速度 owner。未知归属进程不得终止。高 swap、低可用内存、CPU 饱和、Gazebo z 发散、
RTF 异常、TF/速度多 owner 或关键 telemetry 缺失时立即停止，不输出性能通过结论。

仅当 P1 通过，才依总 TODO 顺序推进：

1. P2：InitialKinematicState 接入 MINCO 四条路径，geometry telemetry 升级为按路径类别门禁。
2. P3：实现真正改变 world/start/goal/yaw/验收逻辑的 straight、single_corner、s_turn、
   narrow_corridor、nominal、red_box profile。
3. P4：重跑当前 revision 的 nominal、unknown、stale、unreachable、lease、timeout、recovery、
   TF/localization epoch 与 runtime unsafe。
4. 再推进 RViz 滑窗、P3 Nav2-free、P4 swept footprint/contact、MuJoCo 跨后端、HIL、
   QP paired shadow 和受控实车门禁。不得跳级。

每次变更后按风险递增执行：

  MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1
  colcon test --base-paths src --packages-select <targets> --parallel-workers 1
  colcon test-result --test-result-base build/<package> --verbose
  python3 -m py_compile <changed_python_files>
  bash -n <changed_shell_files>
  ros2 launch <package> <launch> --show-args
  git diff --check

影响地图、定位、规划、安全或控制行为后，必须以最终 revision 在隔离 ROS domain 重跑闭环；
适用时运行 headless MuJoCo 跨后端回归。禁止复用修改前运行结果。

最后更新总 TODO 与当前方向文档；未改 QP 时不要改 backend admission。只显式 stage 本轮文件，
按接口/算法/安全/仿真/文档拆分中文提交，作者固定 liukong1220 <1625038134@qq.com>，
只 push 有改动的用户仓库。最终报告列出每仓 baseline/final SHA、push、测试、ROS domain、
指标、first violation、未验证项和回滚 revision。
