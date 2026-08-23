# ATS 单雷达导航闭环、速度仲裁与 RMUC 2025 验收

请在 `~/ATS_2026_snetry_test` 继续 ATS 四驱四转哨兵导航优化。先完整阅读根目录
`AGENTS.md`，严格遵守三独立 Git 仓库、最小修改、分仓提交与推送规范。用户可见输出使用中文。

## 0. 当前交接基线

先执行并报告三个仓库的状态、分支、`HEAD` 与 `origin/develop`，不得假设工作树干净：

```bash
git -C ~/ATS_2026_snetry_test status --short --branch
git -C ~/ATS_2026_snetry_test/src/ats_sentry_nav status --short --branch
git -C ~/ATS_2026_snetry_test/src/sim/ats_mujoco_sim status --short --branch
```

以下内容属于用户，除非用户再次明确授权，否则不得删除、覆盖、暂存或提交：

- 根仓 `docs/ats_swerve_mpc_ltv_qp_backend_admission.md`；
- 导航仓 `ats_nav_bringup/scripts/static_map_publisher.py`；
- 导航仓 `ats_swerve_mpc/求解器.md`。

当前 V1 架构必须保持：

```text
单 LiDAR 传感器 + Point-LIO 独立状态估计
-> ROGMap 概率占据/膨胀/3D ESDF
-> 地面投影与 2.5D 可通行语义
-> RC-ESDF 规划接口
-> ATS Goal Manager -> JPS -> MINCO S3 + 独立 yaw
-> footprint safety + Local Collision Repair
-> 全向 SE(2) MPC
-> 云台 yaw 速度变换 -> 速度源仲裁 -> 下位机速度接口
```

不得替换或删除 Point-LIO、ROGMap、RC-ESDF、JPS、MINCO S3、独立 yaw、footprint gate、
Local Collision Repair、`ats_swerve_mpc`、`standard_robot_pp_ros2`、自瞄相关代码或
`serial/gimbal_joint_state`。四舵轮命令为车体系 `[vx, vy, wz]`，禁止引入差速、Ackermann、
ICR 或 `vy=0` 约束。只使用一个 LiDAR，不迁移参考工程的双雷达方案。

CAN、电机、轮速、温度、电压、底盘反馈、硬件 watchdog、HIL 诊断与资源 admission 不属于导航
或仿真验收；不要重新引入。仿真只验证定位、建图、规划、控制与算法闭环。

## 1. 已完成但必须如实标注边界

- MuJoCo 默认场景已配置为与 Gazebo 同源的 RMUC 2025：
  `src/sim/gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2025` 与
  `src/sim/gazebo_simulator/rmu_gazebo_simulator/resource/worlds/rmuc_2025_world.sdf`；
- 最终 `red_box` 目标固定为 map `(10.45, 0.35)`，为 RMUC 2025 中央高地标注位置；
- terrain 语义为 `0..99` 连续风险、`100` 硬障碍。MuJoCo 的 RMUC profile 中 adapter 与 MINCO
  阈值均为 `100`，不可再次把连续风险 `>=50` 当硬障碍；
- 已有 `south_corridor` 历史隔离回归：末段误差 `0.04881 m`，规划器离散 footprint 冲突数 `0`；
  该结果不替代当前提交 revision 的默认、完整 `red_box` 或故障矩阵；
- `test_rmuc_2025_scene.py` 已通过 `9 passed`，`ats_goal_manager` 与 `minco_planner` 本轮聚焦
  CTest 各 `1/1 passed`；
- `footprint_collisions=0` 不能证明物理未碰撞。没有独立 contact evaluator 时必须写：
  **MuJoCo 物理接触未验证**；
- **已实现未运行（待本轮最终复验）**：实机自主链为
  `/cmd_vel/autonomy_raw -> fake_vel_transform -> chassis_vel_transform -> /cmd_vel/autonomy`，再与手动
  `/cmd_vel` 由 arbiter 选择为 `/cmd_vel/selected`；MuJoCo 不启动两层速度变换，直接把
  `/cmd_vel/autonomy_raw` 送入 arbiter，再由 `twist_to_motion_ctrl` 消费 selected。键鼠到实机出口、MuJoCo
  `single`/`red_box`/fault matrix 和物理接触均未运行，禁止写为通过。
- **已实现（独立实机入口防绕过）**：`standard_robot_pp_ros2` 的成员默认值和
  `cmd_vel_topic` 参数默认值均为 `/cmd_vel/selected`，根总 YAML 和单包 YAML 也显式传入同一值；代码参数
  默认、根总 YAML 与单包 YAML 的 `execution_command_topic` 均为空，且
  `require_execution_authorization=false`，由 arbiter 处理自动授权而不误伤手动源。直接单包启动不再默认为
  `/cmd_vel`，也不会订阅旧的 latched `MODE_STOP`。这只收紧速度输入所有权，不是实车/HIL 闭环证据。
- **已实现且已运行但未通过 P1（2026-08-23，domain `218`，历史 selected 基线）**：Gazebo 使用
  `/cmd_vel/autonomy_raw -> cmd_vel_arbiter -> /cmd_vel/selected -> gz_chassis_cmd_adapter`，并保留 adapter
  的 big-yaw 旋转、`/motion_control` 与 Gazebo chassis 的唯一发布。完整 recorder 实际覆盖 `60.008622 s`，
  但 `/lidar_odometry` wall p99/max 为 `0.818336/0.877152 s`，`/localization` 为
  `0.818289/0.877242 s`，status `TRACKING/non-TRACKING=575/25`、TF failure=`4/601`，故
  `p1_admission_evidence=false`、action `ABORTED`。运行期 selected publisher/subscriber 为 `1/2`，
  terminal 为 `1/1` 且零速；这证明 fail-closed，不能把未观察到非零 selected 当作仲裁回归。
- **已运行但未通过 P1（最新 2026-08-23，domain `228`）**：默认 `10 Hz / 625`、headless、`rog_map` owner、
  关闭 Transport observer/Direct bridge 的正式 60 s 窗口完成 `60.001345 s`。`/lidar_odometry` 是 first
  violation，wall p99/max=`0.650163/0.743549 s`；`/localization`=`0.650164/0.743591 s`，raw Gazebo LiDAR
  wall p99/max=`0.708598/0.714906 s`，`/clock` p99=`0.010387 s`、RTF p99=`1.032094`，status
  `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`。active selected owner=`1/2`，无非零 selected，
  JPS/MINCO/MPC path 为空，`p1_admission_evidence=false`、原因为 `freshness_lidar_odometry`；这是
  定位/地图 fail-closed，而非仲裁失败。artifact：
  `log/gazebo_minco_mpc_chain/20260823_211350_nominal_none_domain228/`。
- **已验证（loopback arbiter 闭环，2026-08-23 domain `222`）**：重装当前 launch 后正式入口启动
  `cmd_vel_arbiter`，`/cmd_vel -> /cmd_vel/selected -> loopback_simulator` 实际观察到非零
  `vx=0.3`；selected publisher/subscriber 为 `cmd_vel_arbiter/loopback_simulator=1/1`，`/odom.x=0.825`
  （artifact：`/tmp/ats_loopback_arbiter_domain222.fEjFyT`）。该试验仅验证手动源仲裁出口，不验证 MPC
  授权、碰撞或物理动力学。
- 历史 `/cmd_vel_mpc` artifact 只记录迁移前的 freshness/安全收尾；迁移后必须以新 revision、新 ROS domain
  重跑 P1，不能混作当前 selected 链的通过证据。
- domain `217` 曾在健康门禁前使用过期的 `sensor_scan_generation` 与 `localization_fusion` install 二进制，
  因而出现空 frame 和缺少 `map->odom`。该 artifact 只证明部署产物失配，不是 P1 算法结果；runner 现已在
  启动前检查关键源码是否比对应 executable 新，发现失配即 fail-fast。
- **已验证（P1 分层诊断，2026-08-23 domain `219/220`）**：Transport source 在两个 60 s
  窗口各记录 `599/600` 个样本，wall p99=`0.108672/0.105195 s`；对应 ROS
  `/<robot>/livox/lidar` 仅 `168/150` 个样本，wall p99=`0.639515/0.802505 s`，下游
  `/lidar_odometry` 为 `0.662703/0.881395 s`。这两次都不通过 P1；domain `220` 还证明当前
  Fast DDS RMW 不提供 reception publication sequence（`supported=no`），因而不能用该字段推断 DDS
  零丢包或唯一归因。Transport observer 是额外 subscriber，对默认链的因果归因仍是
  **[Confidence: Medium]**。

Claude 只实施、测试和报告修改，禁止任何 Git 写操作（包括 `add`、`commit`、`push`、`pull`、
`stash`、分支或 rebase）。Codex 负责独立 review，并已获本项目持续授权，无需逐轮重复确认即可显式暂存、分仓提交和普通 push；
文档由 Codex 按已验证证据维护。提交只使用用户既有个人身份，不得加入其他作者或 `Co-authored-by`。

P3 尚未验收，不得声称 Nav2-free；目标/action 现有功能不等于 P3 已通过。

## 2. 第一优先级：复核速度源仲裁与键鼠实际闭环

用户必须可以直接运行，无 remap：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

但不能让键鼠和 MPC 直接共同发布同一个底盘话题，因为 `Twist` 没有来源字段且会造成不可判定竞争。
先审查已存在的 arbiter、fake/chassis 变换、launch、`standard_robot_pp_ros2` 串口消费者与
`CmdVelAuthorizationGate`；只修复违反以下契约的行为：

```text
ats_swerve_mpc
-> /cmd_vel/autonomy_raw
-> fake yaw transform
-> /cmd_vel/autonomy_gimbal
-> chassis yaw transform
-> /cmd_vel/autonomy

teleop_twist_keyboard -> /cmd_vel

/cmd_vel + /cmd_vel/autonomy
-> cmd_vel arbiter
-> /cmd_vel/selected
-> standard_robot_pp_ros2 serial -> lower controller
```

硬性契约：

- Gazebo、MuJoCo、loopback 与实机都不再以 `/cmd_vel_mpc` 作为最终执行入口；MPC 自主输出必须为
  `/cmd_vel/autonomy_raw`，arbiter 的唯一输出必须是 `/cmd_vel/selected`；
- 键鼠原始 `/cmd_vel` 保持 ROS 默认，不能要求用户 remap；
- `/cmd_vel/selected` 只能由 arbiter 发布，串口只能订阅 `/cmd_vel/selected`；
- manual fresh 时优先，manual timeout 后必须输出零或按明确状态回收，绝不能复活旧手动命令；
- auto source 仍需 `ExecutionCommand`，manual source 不要求该授权；两者都必须受 emergency stop、
  下位机链路失效与各自 timeout 归零；
- 无链路心跳期间缓存的 manual/auto 命令不得在首个 DOWN->UP 心跳后复活；新命令必须在
  UP 之后到达。该契约由 arbiter 纯逻辑 GTest 锁定；
- 不得删除 fake/chassis 速度变换，实机 launch 默认仍为
  `launch_fake_vel_transform:=True` 和 `launch_chassis_vel_transform:=True`；
- 需要覆盖 fresh manual 优先、manual 超时、自动授权、`MODE_STOP`、新 manager incarnation、
  急停/断链/lease 过期两源归零与 selected 唯一 publisher；并在 MuJoCo 或可控 bridge 中实测键鼠
  `/cmd_vel` 到串口出口/`/motion_control` 的闭环。

实现细节必须满足：

- arbiter 以 `(manager_incarnation, command_sequence)` 识别授权；新 incarnation 必须先接受新鲜
  `MODE_STOP`、撤销旧租约并作废自动缓存，再允许它的 `MODE_EXECUTE`；旧 incarnation 的 replay 必须拒绝；
- 接受任意 `MODE_STOP` 时不得清空手动缓存，但必须清空自动缓存，避免随后新 EXECUTE 复活停机前的 Twist；
- 串口 `require_execution_authorization=false` 后必须把 `execution_command_topic` 设为空，避免串口层
  `execution_stop_` 吞掉手动命令；串口仍保留 emergency stop、链路失效和 watchdog；
- Gazebo 启动 arbiter（无串口场景 `require_serial_link=False`），MPC 发 raw、adapter 订 selected；同步
  evidence recorder、cancel client、runner ownership/zero checks、manifest 和注释。required big-yaw feedback
  缺失时，adapter 的 `/motion_control` 与 Gazebo chassis 输出必须同时为精确零；
- loopback 的 `command_topic` 默认及 launch 参数都为 selected，且 launch 必须启动
  `cmd_vel_arbiter` 以保持 selected 的唯一 publisher。它不启动 MPC，因此仍只是速度出口回归，不是
  完整导航闭环。

修改前必须按 `AGENTS.md` 给出 DoD、精确文件范围、可执行验证清单、假设和停止条件。不要为了让
键鼠运行而放宽 unknown、地图、footprint、lease、急停或自动导航安全语义。

## 3. 第二优先级：RMUC 2025 完整闭环与 P2 故障矩阵

每次运行使用全新合法 `ROS_DOMAIN_ID`，启动前清理同 domain 的残留 launch 进程，但不可影响其他用户任务。
所有 colcon 命令必须显式使用 `--base-paths src`，参考目录不是构建输入。

先运行定向检查：

```bash
cd ~/ATS_2026_snetry_test
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select ats_cmd_vel_arbiter standard_robot_pp_ros2 ats_goal_manager minco_planner ats_mujoco_sim --parallel-workers 1
source install/setup.bash
ctest --test-dir build/ats_cmd_vel_arbiter -R '^test_cmd_vel_arbiter$' --output-on-failure
ctest --test-dir build/standard_robot_pp_ros2 -R '^test_cmd_vel_authorization$' --output-on-failure
ctest --test-dir build/ats_goal_manager -R '^test_goal_manager_epoch$' --output-on-failure
ctest --test-dir build/minco_planner -R '^test_minco_trajectory_optimizer$' --output-on-failure
cd src/sim/ats_mujoco_sim
python3 -m pytest -q test/test_rmuc_2025_scene.py
```

再以当前 revision 完整运行：

```bash
cd ~/ATS_2026_snetry_test
ROS_DOMAIN_ID=<new_valid_domain> PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=single scripts/test_mujoco_minco_mpc_chain.sh

ROS_DOMAIN_ID=<another_new_valid_domain> PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

`red_box` 十段目标固定为：

```text
(4.20,-4.30) -> (4.40,-5.90) -> (5.20,-6.20) -> (1.50,-6.20)
-> (1.50,-7.65) -> (2.20,-7.65) -> (6.50,-7.65) -> (8.70,-4.90)
-> (9.25,-2.25) -> (10.45,0.35)
```

对每个 P2 故障分别启动全新 domain 与 MuJoCo：`adapter_lease`、`service_timeout`、`input_stale`、
`unknown`、`unreachable`。每项必须观察并保存：

```text
ready=false -> emergency_stop=true -> 最终速度=0 -> /motion_control=0
```

恢复后 source/adapter generation 必须递增，未发送新目标时不能由迟到 response、旧 snapshot 或急停前
reference 恢复运动。成功路线必须记录最终坐标、误差、raw/reference 点数、离散 footprint 冲突采样数、
失败/恢复次数；物理接触单独标注为未验证，除非加入独立 contact evaluator。

## 4. 第三优先级：以 `参考/navi_minco_bit` 做证据型审阅

参考项目路径为 `~/ATS_2026_snetry_test/参考/navi_minco_bit`，只能作为算法和实现证据来源，
不得加入 colcon、不得整包复制、不得迁入双雷达、其 Nav2 依赖或通信协议。优先审阅：

- `src/navigation/minco_controller/include/minco_controller/minco_mpc_controller.hpp`；
- `src/navigation/minco_controller/include/minco_controller/mpc_solver.hpp`；
- `src/navigation/minco_controller/include/minco_controller/performance/mpc_performance_monitor.hpp`；
- `src/navigation/communication/src/com_interface_ros.cpp` 与 `com.cpp`；
- `src/navigation/navi2_bringup/launch/navigation_launch.py` 与 `params/sentry1.yaml`。

围绕四条线逐项建立表格，每一项必须有“参考源码证据 -> ATS 当前实现 -> 实际缺口 -> 最小改动 ->
可运行测试”，不得把参考工程中存在的代码当作 ATS 性能结果：

1. Point-LIO 到 ROGMap：时间戳、队列/背压、地图 snapshot、unknown/膨胀/ESDF、单雷达输入链；
2. MINCO：初始运动状态、时间分配、动态约束、轨迹生命周期、与 immutable snapshot 的一致性；
3. MPC：warm start、solver residual/deadline、分配与锁、reference freshness、`[vx,vy,wz]` 和云台 yaw 坐标变换；
4. 性能：在真实 ATS revision/固定场景记录 p50/p95/p99、CPU、内存、频率与时间预算，不引用参考项目的
   `50 Hz`、`6 ms` 或内存数据为 ATS 结论。

## 5. 文档、提交与报告要求

每轮源码行为变更后必须更新：

- `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`；
- `docs/项目优化文档/ATS导航剩余优化总TODO.md`；
- 下一阶段提示词文档。

最后分别检查根仓、导航仓、MuJoCo 仓的 `git diff --check`。Claude 不得暂存、提交或推送，只向 Codex
报告精确文件、diff 要点和原始测试输出；Codex 已获本项目持续授权，无需逐轮重复确认，按接口/算法/仿真/文档拆分中文提交并普通 push。
Codex 提交前需核对 `git diff --cached --stat` 与 `git diff --cached --check`，仅显式暂存本轮文件，不使用
`git add .`、`git add -A`、force push，且不添加其他作者。最终报告必须区分“已验证”“已实现未运行”“推断”“未实现”，
列出命令、终点结果、唯一 publisher/subscriber、故障矩阵、MuJoCo 物理接触状态、未运行测试与建议提交拆分。
