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

- P0 已完成：从用户 SSH `origin/develop` 的 depth-1 root clone 后，使用 exact lock import
  成功取得 22 个仓库；锁文件由 vcs export --exact -n 生成。
- rmoss_gz_resources 使用 humble=b5c759f08844dfda19c79aa870866ace8d4c7b3a；
  ats_mujoco_sim、teleop_gimbal_keyboard 使用已验证的用户 SSH URL。
- Gazebo fork ac2085fcf5e109f9f53d80e5d0661facbe588a6a 已加入单一 C++ recorder、
  EvidenceStatistics 与 runner 进程 telemetry；Release build、包级 CTest 32/32、
  runner bash -n 和 launch --show-args 已通过。
- 历史 domain 230 的 /localization interval p50/p95/p99 为 0.371/0.994/1.612 s，
  adapter 反复 ready=false，action fail-closed。
- `/lidar_odometry` 已是当前 recorder 链中的首个可见 freshness 违反者；仍不能直接归因 LiDAR、
  Point-LIO、DDS、Gazebo RTF、TF、localization_fusion 或 recorder 中的任一唯一行为 owner。
- P1 的最终 60 s headless 已在 domain `225` 完成：`ENABLE_CAMERA_SENSORS=false`、终态急停与两级
  零速均成立，但 action 在 90 s 内未终止；`/lidar_odometry` p99/max wall interval 为
  `2.759540/2.942285 s`，因此 freshness 不通过，P1 和 P2 仍未通过。独立 domain `224` 曾 action 成功，
  但同样不满足 freshness。
- recorder 已在新 `60 s` domain 实际覆盖 raw `/<robot>/livox/lidar`、bridge `/livox/lidar`、
  `/cloud_registered` 与 `/lidar_odometry`。domain `215` 的 `4 ms` physics candidate（`10 Hz / 625 x 32`）
  p99=`1.610141/1.435872/1.430092 s`；domain `214` 的 `5 Hz / 625 x 32`、SDF/bridge/Point-LIO 三处
  `0.2 s` 同期 candidate 恶化为 `3.666797/3.312480/3.308619 s`；domain `213` 的无 GUI state broadcaster
  headless world 在 `10 Hz / 625 x 32` 下为 `1.409273/1.322408/1.319348 s`。三次 action 均 unsafe
  ABORTED，均为 `freshness_lidar_odometry`，不得作为默认或 P1/P2 通过。
- 默认仍是 `LIVOX_UPDATE_RATE_HZ=10.0`、`LIVOX_HORIZONTAL_SAMPLES=625`。频率参数会同步驱动 SDF
  update rate、C++ bridge `scan_period_sec` 和 Point-LIO `mapping.lidar_time_inte`；`WORLD_SDF_PATH` 只在
  非空时传入 launch。`rmu_gazebo_simulator` 本轮为 `32 tests, 0 errors, 0 failures`。
- domain `233` 不合法（Fast DDS domain 上限/port 计算错误），不得再使用大于 `232` 的 domain。
  合法 domain `231` 曾在 `ROS_LOG_DIR=/tmp` 下启动完整导航链，但 RTF 偏低、localization wall gap
  最大约 `2.185 s`、status 有非 TRACKING 样本，action 未完成并最终急停。该结果是历史诊断运行，
  不能作为 P1 通过。
- MINCO production node 仍未将 InitialKinematicState 传到 center、footprint、fallback、
  repair 四条路径；geometry telemetry 还不是完整 production gate。
- TEST_PROFILE 尚未真正控制 straight/corner/S/narrow/red-box 场景，GOAL_YAW 未进入 action payload。
- 默认保持 solver_mode=ilqr；qp_shadow 只诊断，solver_mode=qp 继续拒绝。
- runner 在启动前只审计合法 ROS domain 与残留导航/仿真进程。

开始前先报告 DoD、精确文件范围、验证清单、假设/未验证项/停止条件，并核对根仓、导航仓、
MuJoCo、Gazebo fork、机器人描述仓的 branch/HEAD/upstream/remote/status。

保护用户内容，禁止读取为设计依据、修改、删除、暂存或提交：

- src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py
- src/ats_sentry_nav/ats_swerve_mpc/求解器.md
- src/sim/gazebo_simulator/rmu_gazebo_simulator/scripts/ats_bridge/gz_livox_bridge.py

禁止 git add .、git add -A、破坏性恢复或 force push。Gazebo 只能写用户 origin/main，
禁止写 upstream。未知改动均视为用户内容；若与必要 owner 重叠，停止并报告。

阶段 P1：定位 Gazebo localization freshness 的首个违反者。

运行开始前审计残留进程、Gazebo z/RTF 和关键 telemetry；一旦系统失稳、残留进程、TF/速度多 owner、
unknown/lease/急停门异常或 callback/话题证据缺失，立即保存 raw artifact 并停止。不要通过提高
localization/adapter/MPC timeout 或关闭 fail-closed 来“通过”。

最终 revision 的 C++ recorder 已在独立 domain 实际记录 raw Gazebo LiDAR、`/livox/lidar`、
`/cloud_registered`、`/lidar_odometry`、`/odometry` 和 `/localization` 的同构 steady-wall、ROS stamp 与
`/clock` age。raw 至 loam 的 wall gap 仍同阶、recorder callback p99 为微秒级；下一步补 Gazebo publisher
与 DDS subscriber 的独立计数，不能凭相关性归因 Point-LIO、loam 或 DDS。


1. 当前 recorder 已低开销订阅并测量：
   /clock、/lidar_odometry、/odometry、/localization、/localization/status、
   /rog_map_adapter/ready；先审计实现和现有 CTest，只有字段缺失时才修改它。
2. 每级记录 steady_clock wall arrival interval p50/p95/p99/max、ROS stamp interval、
   stamp age、重复/倒退、最长 gap、消息数；记录 RTF、TF lookup failure、关键进程
   CPU/RSS/thread/context switch、DDS queue/drop 与 callback blocking。
3. 建立字段级 contract table：
   /clock -> /lidar_odometry -> /odometry -> /localization -> status -> adapter
   必须涵盖 frame、clock、QoS、producer、consumer、timeout、health gate、fallback。
4. 当前 domain `213/214/215/224/225` 都把 `/lidar_odometry` 标为首个可见违反者。下一轮使用新 domain、
   固定 revision，且只补 publisher/DDS 分层所需的单因素观测；不得将已拒绝的 `4 ms`、`5 Hz` 或无 GUI
   state broadcaster world 作为默认或重复用于通过声明。设置 `ROS_LOG_DIR=/tmp/<run>`，避免用户 home
   的只读日志路径干扰；禁止高频 ros2 topic echo 干扰被测链。
5. 找到最早违反 freshness 的行为 owner 后，只修改该 owner，并补最窄 deterministic regression。
   禁止提高 odom/localization/map/reference timeout、adapter lease 或 projection deadline；
   禁止 Ground Truth 接管正式 /localization。

P1 DoD：低负载 headless 连续至少 60 s，/localization p99 interval < 0.25 s，
无 >0.5 s gap，stamp 不倒退，status 持续 TRACKING，adapter 不因 localization 抖动
变为 ready=false；随后两个独立 ROS domain 的 straight action 成功，并记录终点误差、
规划/速度唯一 owner 与收尾零速。

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

## 2026-08-20 最新交接（优先于历史诊断条目）

当前源码 revision 在本次提交后以各仓库 `origin` 分支 HEAD 为准。最后一个完整 P1 默认基线为：

```bash
ROS_DOMAIN_ID=208 \
ROS_LOG_DIR=/tmp/ats_p1_runtime_208 \
LOG_ROOT=/tmp/ats_p1_runtime_208 \
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=nominal \
HEADLESS=true HEADLESS_RENDERING=true ENABLE_CAMERA_SENSORS=false \
USE_RVIZ=false USE_VIEWER=false \
LIVOX_UPDATE_RATE_HZ=10.0 LIVOX_HORIZONTAL_SAMPLES=625 \
OBSERVE_GAZEBO_TRANSPORT_LIDAR=false \
USE_DIRECT_GAZEBO_LIDAR_BRIDGE=false \
LIDAR_BRIDGE_PUBLISHER_DEPTH=10 \
LIDAR_BRIDGE_PUBLISHER_RELIABILITY=reliable \
RUN_DURATION_SEC=60 ACTIVE_OBSERVER_WINDOW_SEC=60 \
GOAL_TIMEOUT_SEC=90 GOAL_RESULT_WAIT_SEC=90 \
scripts/test_gazebo_minco_mpc_chain.sh
```

该命令的 artifact 是
`/tmp/ats_p1_runtime_208/20260820_175758_nominal_none_domain208/`，退出码为 `1`。这是有效的
失败证据，不是运行环境污染或通过记录：recorder 完成 `60.016403 s`，启动前和结束后无残留进程，
`/clock` wall p99=`0.078741 s`，但 raw ROS `/<robot>/livox/lidar` p99=`2.954985 s`、
`/livox/lidar`=`2.958803 s`、`/cloud_registered`=`3.327495 s`、`/lidar_odometry`=`3.327198 s`、
`/localization`=`3.324217 s`。分类器首违仍为 `lidar_odometry`，status
`TRACKING/non-TRACKING=362/210`，TF failure=`35/600`，故 `p1_admission_evidence=false`。
动作有 JPS/MINCO/MPC/轮转动证据，但最终 `ABORTED`；terminal `emergency_stop=true` 且
`/cmd_vel_mpc`、`/motion_control` 均为零。物理 contact、连续 swept 与离散 footprint 冲突未验证。

在性能更高的新电脑上必须先用**全新且合法的** `ROS_DOMAIN_ID`（`0..232`，不得复用旧 domain）和
同一默认参数重跑上面的 60 秒 baseline。先运行 `bash -n scripts/test_gazebo_minco_mpc_chain.sh`、
`scripts/test_gazebo_runner_contract.sh`、定向 `rmu_gazebo_simulator` build/CTest 与两个 launch 的
`--show-args`，并保存完整 artifact。只有 P1 的全部条件都通过，才可进入 P2：

- `/localization` p99 `<0.25 s`、max gap `<=0.5 s`，且 stamp 无倒退；
- `/localization/status` 持续 TRACKING，TF lookup 没有 failure；
- adapter heartbeat 不因定位而失效；
- 两个独立 domain 的真实 straight action 成功，且唯一 owner、终点误差和终态零速都有证据。

禁止把本机 domain `208` 的绝对 wall 时间外推到新电脑，也禁止基于硬件更快而直接跳到 P2。若新的默认
baseline 仍失败，下一步只做不增加长期 `PointCloudPacked` Transport subscriber 的分层计数/trace，区分
Gazebo publisher 调度与 generic `ros_gz_bridge`/ROS-DDS 接收边界；不要重新启用已拒绝的 `4 ms` physics、
`5 Hz`、无 SceneBroadcaster、Direct bridge 或 `BEST_EFFORT/KeepLast(1)` candidate 作为默认。继续禁止
swap/内存/CPU 等主机资源准入设计、timeout 放宽、Ground Truth 接管定位和关闭 fail-closed。
