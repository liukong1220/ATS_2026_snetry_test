# 下一阶段新对话提示词：Gazebo 定位新鲜度与规划闭环准入

下面内容可直接作为新对话的首条提示词。

[$develop-robot-vision-navigation]

继续 ~/ATS_2026_snetry_test 的 ATS 四驱四转哨兵导航优化。P0 已完成，本轮从
P1 Gazebo localization freshness 开始。完整读取：

1. AGENTS.md
2. docs/项目优化文档/ATS导航剩余优化总TODO.md
3. docs/nav2_to_3desdf_minco_mpc_optimization_direction.md
4. docs/ats_swerve_mpc_ltv_qp_backend_admission.md
5. dependencies.repos 与 dependencies.lock.repos

按门禁持续执行：定位 -> 最小修改 -> 构建 -> 聚焦单测 -> 隔离 Gazebo 闭环 -> 文档。停止条件触发后保存
raw artifact 和 first violation，停止后续高风险阶段。Codex 经用户明确授权后可以分仓提交并普通 push；
Claude 禁止任何 Git 写操作。提交只使用用户既有个人身份，不得添加其他作者或 `Co-authored-by`。

当前已验证基线：

- P0 已完成：从用户 SSH `origin/develop` 的 depth-1 root clone 后，使用 exact lock import
  成功取得 22 个仓库；锁文件由 vcs export --exact -n 生成。
- rmoss_gz_resources 使用 humble=b5c759f08844dfda19c79aa870866ace8d4c7b3a；
  ats_mujoco_sim、teleop_gimbal_keyboard 使用已验证的用户 SSH URL。
- Gazebo fork ac2085fcf5e109f9f53d80e5d0661facbe588a6a 已加入单一 C++ recorder、
  EvidenceStatistics 与 runner 进程 telemetry；“包级 CTest 32/32”是历史记录，不能当作当前状态。
  当前 focused `test_evidence_statistics` 已验证四次 arrival 记录三段 wall 间隔，即使 ROS stamp 重复或
  倒退；该 fixture 修正不放宽任何 freshness 判据。
- **当前速度接口迁移（2026-08-23）**：Gazebo 已使用
  `ats_swerve_mpc -> /cmd_vel/autonomy_raw -> cmd_vel_arbiter -> /cmd_vel/selected ->
  gz_chassis_cmd_adapter`；adapter 保留 big-yaw 旋转和 `/motion_control`/Gazebo chassis 唯一发布。
  `validate_navigation_config.py` 和 arbiter 19 条 GTest 已通过。重装当前 launch 后，loopback 在隔离 domain
  `222` 实测 `/cmd_vel -> selected -> /odom`；selected publisher/subscriber=`1/1`，`vx=0.3` 对应
  `/odom.x=0.825`（artifact：`/tmp/ats_loopback_arbiter_domain222.fEjFyT`）。这只是手动源仲裁出口证据，
  不是 P1 或完整速度链闭环。
- **已实现且聚焦测试通过**：当 `gz_chassis_cmd_adapter` 要求 big-yaw feedback 而样本缺失时，
  `/motion_control` 与 Gazebo chassis 输出必须同时精确归零；不得只让 chassis 输出归零而保留非零
  `/motion_control`。这不影响 P1 freshness 判据，尚不是 Gazebo 物理闭环证据。
- **历史 selected P1（2026-08-23，domain `218`）**：默认 `10 Hz / 625`、headless、`rog_map` owner 的
  recorder 实际完成 `60.008622 s`。`/lidar_odometry` 是首个 freshness 违反者，wall p99/max=
  `0.818336/0.877152 s`；`/localization`=`0.818289/0.877242 s`，status
  `TRACKING/non-TRACKING=575/25`、TF failure=`4/601`。action `ABORTED`，JPS/MINCO/MPC 路径均为空，
  `/cmd_vel/selected` 无非零样本并在 terminal 保持唯一 publisher/零速。`p1_admission_evidence=false`，
  原始 artifact：`/tmp/ats_p1_selected_domain218/20260823_194049_nominal_none_domain218/`。
- **最新默认 P1（2026-08-23，domain `228`）**：默认 `10 Hz / 625`、headless、`rog_map` owner、
  `OBSERVE_GAZEBO_TRANSPORT_LIDAR=false`、`USE_DIRECT_GAZEBO_LIDAR_BRIDGE=false` 与 generic
  `RELIABLE/KeepLast(10)` 的正式运行，recorder 完成 `60.001345 s`。`/lidar_odometry` 是首个 freshness
  违反者，wall p99/max=`0.650163/0.743549 s`；`/localization`=`0.650164/0.743591 s`，raw Gazebo LiDAR
  wall p99/max=`0.708598/0.714906 s`。`/clock` wall p99=`0.010387 s`、RTF p99=`1.032094`，status
  `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`。JPS/MINCO/MPC path 均为空，selected 无非零样本，
  active owner=`1/2`；这是定位/地图 fail-closed。`p1_admission_evidence=false`，原因为
  `freshness_lidar_odometry`，artifact：
  `log/gazebo_minco_mpc_chain/20260823_211350_nominal_none_domain228/`。RTF 正常不能证明 generic bridge、DDS
  或 Point-LIO 中任一方是唯一根因。
- **最新 owner 审计（2026-08-24，无新运行）**：活动 generic LiDAR publisher 是系统安装的
  `/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge`，包版本
  `0.244.25-1jammy.20260608.160002`；workspace 没有 `ros_gz_bridge` 源包。对应 upstream
  `0.244.25` 的 GZ-to-ROS 路径在 Gazebo Transport 回调内同步执行
  `PointCloudPacked -> PointCloud2 -> publish()`，且没有使用 YAML 传入的 subscriber queue size。
  项目只拥有 generic bridge 的 topic/方向/ROS publisher QoS 配置；`rmoss_gz_bridge` 只有 pose/RFID
  bridge，不是该 LiDAR owner。因此已触发“owner 不在项目可修改范围”的停止条件：本轮没有源码修改、
  没有添加计数、没有占用新 ROS domain，domain `228` 仍是最新运行失败 artifact，P1 仍未通过。
- domain `217` 的健康门禁失败来自过期 install 二进制，而不是源码参数：源码的
  `sensor_scan_generation`/`localization_fusion` 已含 Gazebo frame 与固定 map 注册，实际执行文件却早于源码。
  runner 现把 arbiter、MPC、两定位节点与 Gazebo recorder 五个关键 executable 的源码新旧检查写入 `runtime_preflight.txt`，失配即拒绝启动，不得将此类
  artifact 当作 P1 失败或通过。
- **P1 Transport 分层（domain `219`，未通过）**：60.009199 s 内 Gazebo Transport PointCloudPacked
  记录 `599` 个样本，wall p99=`0.108672 s`；ROS `/<robot>/livox/lidar` 仅 `168`
  个，wall/stamp p99=`0.639515/1.800000 s`，`/lidar_odometry` p99=`0.662703 s`。该运行增加了
  一个只读 Transport subscriber，能证明该运行的源头稳定，不能单独证明默认无 observer 链的因果。
- **P1 DDS 元数据分层（domain `220`，未通过）**：60.009184 s 内 Transport `600`
  个样本且 wall p99=`0.105195 s`，ROS raw `150` 个且 wall/stamp p99=`0.802505/1.900000 s`，
  `/lidar_odometry`/`/localization` p99=`0.881395/0.881366 s`。Fast DDS RMW 报
  `gazebo_lidar_dds_publication_sequence_supported=no`，故 sequence 的 `0` 不是零丢包证据；不得用它区分
  generic bridge 未发布与 DDS 接收丢样。这两次均为 action `ABORTED`、`p1_admission_evidence=false`。
- 历史 domain 230 的 /localization interval p50/p95/p99 为 0.371/0.994/1.612 s，
  adapter 反复 ready=false，action fail-closed。
- `/lidar_odometry` 已是当前 recorder 链中的首个可见 freshness 违反者；仍不能直接归因 LiDAR、
  Point-LIO、DDS、Gazebo RTF、TF、localization_fusion 或 recorder 中的任一唯一行为 owner。
- P1 的最终 60 s headless 已在迁移前的 domain `225` 完成：`ENABLE_CAMERA_SENSORS=false`、终态急停与
  旧 `/cmd_vel_mpc=0` 均成立，但 action 在 90 s 内未终止；`/lidar_odometry` p99/max wall interval 为
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
- runner 在启动前审计合法 ROS domain、残留导航/仿真进程及关键运行二进制新鲜度。

开始前先报告 DoD、精确文件范围、验证清单、假设/未验证项/停止条件，并核对根仓、导航仓、
MuJoCo、Gazebo fork、机器人描述仓的 branch/HEAD/upstream/remote/status。

保护用户内容，禁止读取为设计依据、修改、删除、暂存或提交：

- src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py
- src/ats_sentry_nav/ats_swerve_mpc/求解器.md
- src/sim/gazebo_simulator/rmu_gazebo_simulator/scripts/ats_bridge/gz_livox_bridge.py

Codex 经用户明确授权后可执行显式 `add`、分仓 commit 与普通 push；Claude 禁止所有 Git 写操作。
两者均禁止 `git add .`、`git add -A`、破坏性恢复和 force push。Gazebo fork 只允许 Codex 将当前 `main`
普通 push 到用户 `origin/main`，禁止写 upstream；未知改动均视为用户内容，若与必要 owner 重叠，停止并报告。

阶段 P1：定位 Gazebo localization freshness 的首个违反者。

运行开始前审计残留进程、Gazebo z/RTF 和关键 telemetry；一旦系统失稳、残留进程、TF/速度多 owner、
unknown/lease/急停门异常或 callback/话题证据缺失，立即保存 raw artifact 并停止。不要通过提高
localization/adapter/MPC timeout 或关闭 fail-closed 来“通过”。

最终 revision 的 C++ recorder 已在独立 domain 实际记录 raw Gazebo LiDAR、`/livox/lidar`、
`/cloud_registered`、`/lidar_odometry`、`/odometry` 和 `/localization` 的同构 steady-wall、ROS stamp 与
`/clock` age。domain `219/220` 已表明 raw 至 loam 的 wall gap 仍同阶、recorder callback p99 为微秒级，
而 Transport source 保持 10 Hz。Transport observer 是 `OBSERVE_GAZEBO_TRANSPORT_LIDAR=true` 的可选诊断，
不改默认链但会新增 subscriber。DDS sequence 元数据当前不可用；最新 domain `228` 在无 Transport observer
下仍失败，且 `/clock`/RTF 正常。generic `ros_gz_bridge` owner 已确认位于系统安装包、超出当前四仓
可修改范围；下一步必须先由用户授权将对应源码纳入 workspace/fork，或批准项目外 trace。授权前停止源码
修改，不在 Point-LIO、loam、recorder 或 DDS consumer 侧增加无法证明 generic 发布次数的替代计数，也不能
凭相关性归因 Point-LIO、loam 或 DDS。


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
4. 再推进 RViz 滑窗、P3 Nav2-free、P4 swept footprint、MuJoCo 跨后端、QP paired shadow 和
   受限实机导航。不得跳级。

Gazebo 与 loopback 的迁移已完成：Gazebo 使用
`/cmd_vel/autonomy_raw -> /cmd_vel/selected -> gz_chassis_cmd_adapter`，loopback 使用
`/cmd_vel -> cmd_vel_arbiter -> /cmd_vel/selected -> loopback_simulator`；
不启动实机专属 fake/chassis yaw transform 时仍保持 Gazebo adapter 的既有 frame 语义。现在必须以新
合法 domain 重跑 P1，验证定位、建图、规划、轨迹、MPC、云台协调和 selected 导航安全链。此前
`/cmd_vel_mpc` artifact 是历史失败诊断，不能作为新链通过或失败的直接证据。不得新增或恢复
CAN、电机、轮速、电流、电压、温度、底盘反馈、硬件 watchdog、`/motion_control` 或接触 telemetry
作为仿真或 action 通过条件。下位机/HIL 诊断不属于该提示词的执行范围。必须保留
`standard_robot_pp_ros2` 的决策/自瞄相关内容，以及 `serial/gimbal_joint_state`、`GimbalYawStatus` 和
`YawAuthorityRequest` 的云台与速度变换契约。

当前会话直接实施、验证并依据实际 artifact 更新 `docs/`。Codex 经用户授权后负责 review、显式暂存、
分仓提交和普通 push；Claude 不得执行 Git 写操作。提交只使用用户本机既有 Git 身份，不添加任何其他作者或
`Co-authored-by`。Gazebo 的
`gz_chassis_cmd_adapter` 保留 big-yaw 变换、`/motion_control` 与 Gazebo chassis 的唯一发布，只将它的
  输入切到 selected；loopback 也启动 arbiter 且只订阅 selected。同步更新 Gazebo evidence recorder、cancel client 和
runner，使 active/terminal ownership、非零动作和故障归零都观察 `/cmd_vel/selected`，不保留旧字段名作为
当前话题判据。

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
按接口/算法/安全/仿真/文档拆分中文提交，作者仅保留用户本人既有身份，
只 push 有改动的用户仓库。最终报告列出每仓 baseline/final SHA、push、测试、ROS domain、
指标、first violation、未验证项和回滚 revision。

## 2026-08-21 最新交接（优先于历史诊断条目）

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
动作有 JPS/MINCO/MPC 与 `/cmd_vel_mpc` 证据，但最终 `ABORTED`；terminal
`emergency_stop=true` 且 `/cmd_vel_mpc` 为零。连续 swept 与离散 footprint 冲突未验证。

本轮随后在新 domain `208` 重跑无 viewer/RViz 的 `TEST_PROFILE=nominal`，recorder 实际完成
`90.007728 s`，JPS、MINCO、MPC、ROGMap adapter 与 `/cmd_vel_mpc` 单一发布者均有运行证据，但 action
在 `90 s` 内未成功。`/lidar_odometry`、`/odometry`、`/localization` 的 wall interval p95/max 为
`1.427/2.744 s`、`1.424/2.750 s`、`1.419/2.757 s`，`/clock` RTF p50/p95/p99 为
`0.330/0.585/0.984`，结果仍是 `freshness_lidar_odometry`。artifact 位于
`log/gazebo_minco_mpc_chain/20260821_101956_nominal_none_domain208/`；它是当前 revision 的失败证据，
不是 P1/P2 通过记录。

本轮 MuJoCo red_box 使用新 domain `207`；首个 action 目标已接受，ROGMap adapter generation 从
`756` 前进至 `769`，但 Goal Manager 后续记录 `pose=(nan, nan)` 并 fail-stop，故首个目标未完成。
无效 pose 的首次来源未定位，不得用仿真内部速度适配、轮速、接触、CAN 或底盘反馈替代该导航侧诊断。

在性能更高的新电脑上必须先用**全新且合法的** `ROS_DOMAIN_ID`（`0..232`，不得复用旧 domain）和
同一默认参数重跑上面的 60 秒 baseline。先运行 `bash -n scripts/test_gazebo_minco_mpc_chain.sh`、
`scripts/test_gazebo_runner_contract.sh`、定向 `rmu_gazebo_simulator` build/CTest 与两个 launch 的
`--show-args`，并保存完整 artifact。只有 P1 的全部条件都通过，才可进入 P2：

- `/localization` p99 `<0.25 s`、max gap `<=0.5 s`，且 stamp 无倒退；
- `/localization/status` 持续 TRACKING，TF lookup 没有 failure；
- adapter heartbeat 不因定位而失效；
- 两个独立 domain 的真实 straight action 成功，且唯一 owner、终点误差和终态零速都有证据。

禁止把本机 domain `208` 的绝对 wall 时间外推到新电脑，也禁止基于硬件更快而直接跳到 P2。若新的默认
baseline 仍失败，先在独立新 domain 以 `OBSERVE_GAZEBO_TRANSPORT_LIDAR=true` 运行同一窗口；该 recorder
订阅只在诊断进程寿命内存在，用于区分 Gazebo publisher 调度与 generic `ros_gz_bridge`/ROS-DDS 接收边界。
若 source 边界仍不足以归因，再补 DDS 接收计数；不要重新启用已拒绝的 `4 ms` physics、
`5 Hz`、无 SceneBroadcaster、Direct bridge 或 `BEST_EFFORT/KeepLast(1)` candidate 作为默认。继续禁止
swap/内存/CPU 等主机资源准入设计、timeout 放宽、Ground Truth 接管定位和关闭 fail-closed。
