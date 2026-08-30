# ATS 导航剩余优化总 TODO

> 状态：唯一活动导航优化清单
> 更新时间：2026-08-24
> 适用范围：Gazebo、MuJoCo 与实机导航软件侧的 ATS 四驱四转哨兵导航链
> 历史说明：旧阶段 TODO 已退役；历史实现与运行证据通过 Git 历史、
> `docs/ats_swerve_mpc_ltv_qp_backend_admission.md` 和状态文档追溯。

## 1. 目标与完成定义

本清单用于收敛当前仍未完成的导航生产能力和准入证据。它不把组件测试、单次仿真成功与实际导航
任务成功混为同一状态。底盘 CAN、电机、轮速、电流、电压、温度、底盘反馈和硬件 watchdog 由下位机
或 HIL 诊断维护，不属于本导航清单的检测、门禁或通过条件。最终目标链保持不变：

```text
传感器 + 独立状态估计
-> ROGMap 概率占据/膨胀/3D ESDF
-> 地面投影与 2.5D 可通行语义
-> RC-ESDF 规划接口
-> ATS Goal Manager -> JPS -> MINCO S3 + 独立 yaw
-> footprint safety + Local Collision Repair
-> 全向 SE(2) MPC -> 自主速度源 -> 云台 yaw 速度变换 -> 速度源仲裁 -> 下位机速度接口
```

总体验收建议同时满足：

- 干净主机可从远端仓库复建全部依赖和仿真资源；
- 地图、定位、规划、控制和导航速度输出各有唯一 owner；
- nominal、边界场景和故障恢复均有独立 ROS domain 的证据；
- stale、unknown、无路、unsafe、solver failure 或 lease failure 都确定性零速度；
- P2、P3、P4 和 QP 主链分别通过自己的门禁，不相互替代；
- 仿真与实机导航侧分别保留独立 artifact；下位机/HIL 诊断不构成导航准入；
- 性能结论来自固定 revision、配置、硬件和原始 artifact。

### 1.1 2026-08-21 交接更新

- MuJoCo 默认场景已切换为与 Gazebo 同源的 RMUC 2025 模型、world 几何与高度场；`red_box`
  的最终目标固定为 map 坐标 `(10.45, 0.35)`，即原图像标注的中央高地位置；
- RMUC 2025 profile 将 terrain 连续风险 `0..99` 与硬障碍 `100` 区分开。adapter 与 MINCO
  的 `terrain_obstacle_value_threshold`/`obstacle_value_threshold` 均为 `100`，不再把风险值 `63`
  误判为墙体；
- 独立 physics/navigation launch、RMUC 场景契约测试与南侧通道分段回归已有组件或运行证据；
  完整 `red_box`、默认单点和 P2 故障矩阵建议在本次提交 revision 的全新 ROS domain 重跑后才能验收；
- **已实现并经静态验收**：实机、MuJoCo 与 Gazebo 的自主源均为
  `/cmd_vel/autonomy_raw`；实机经既有 fake/chassis 变换到 `/cmd_vel/autonomy`，四端的最终执行速度
  都以 `/cmd_vel/selected` 为唯一允许 Twist 输入。`cmd_vel_arbiter` 是 selected 的唯一 publisher；自动源
  由新鲜 `ExecutionCommand` 租约放行，手动源可不依赖该授权，但两源均受 emergency stop、链路失效和各自
  timeout 归零。Gazebo adapter 保留 big-yaw frame 变换和 `/motion_control`/Gazebo chassis 的唯一 owner；
  loopback 不再以 `/motion_control` 绕过仲裁。
- **已验证（loopback arbiter 出口，2026-08-23 domain `222`）**：重装当前 launch 后实际启动 arbiter，
  `/cmd_vel -> /cmd_vel/selected -> loopback_simulator` 有非零 `vx=0.3` 输出，selected 的
  publisher/subscriber 为 `cmd_vel_arbiter/loopback_simulator=1/1`，`/odom.x=0.825`（artifact：
  `/tmp/ats_loopback_arbiter_domain222.fEjFyT`）。该试验仅验证
  手动源仲裁出口，不覆盖 MPC 授权、完整导航或物理仿真。
- **未验证**：无需 remap 的键鼠到串口或 `/motion_control`、MuJoCo nominal/red_box/P2 fault matrix 和物理
  接触。历史 `/cmd_vel_mpc` artifact 只用于迁移前的 freshness 诊断；当前 selected 链最新默认 P1 基线
  为 domain `228`，仍未通过。
- **未通过（Gazebo P1，2026-08-23 domain `218`）**：默认 `10 Hz / 625`、headless、`rog_map` owner 的
  recorder 正常完成 `60.008622 s`。`/lidar_odometry` 是 first violation，wall p99/max=
  `0.818336/0.877152 s`；`/localization`=`0.818289/0.877242 s`，status
  `TRACKING/non-TRACKING=575/25`、TF failure=`4/601`，action `ABORTED`。运行期
  `/cmd_vel/selected` owner=`1/2`、terminal=`1/1` 且零速；定位/地图 fail-closed 后没有 JPS/MINCO/MPC
  path 或非零 selected。这不构成速度仲裁回归，也不满足 P1/P2。
- **未通过（最新 Gazebo P1，2026-08-23 domain `228`）**：默认 `10 Hz / 625`、headless、`rog_map` owner、
  关闭 Transport observer/Direct bridge 的 recorder 正常完成 `60.001345 s`。`/lidar_odometry` 是 first
  violation，wall p99/max=`0.650163/0.743549 s`；`/localization`=`0.650164/0.743591 s`，raw Gazebo LiDAR
  wall p99/max=`0.708598/0.714906 s`，`/clock` p99=`0.010387 s`、RTF p99=`1.032094`，status
  `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`。active `/cmd_vel/selected` owner=`1/2`，无非零
  selected，JPS/MINCO/MPC path 为空，故 `p1_admission_evidence=false`、原因为
  `freshness_lidar_odometry`。这是定位/地图 fail-closed，不是仲裁回归；artifact：
  `log/gazebo_minco_mpc_chain/20260823_211350_nominal_none_domain228/`。
- **已修复（P1 预检）**：domain `217` 暴露 install executable 早于
  `sensor_scan_generation`/`small_gicp_relocalization` 源码。Gazebo runner 现检查 arbiter、MPC、两定位节点与
  Gazebo recorder 的源码/可执行文件新旧，失配时写入 `runtime_preflight.txt` 并 fail-fast；217 artifact
  不作为算法验收。
- **未通过（P1 上游分层，2026-08-23 domain `219/220`）**：两个 60 s 窗口内 Gazebo Transport
  PointCloudPacked 分别记录 `599/600` 个样本，wall p99=`0.108672/0.105195 s`；ROS raw
  `/<robot>/livox/lidar` 仅 `168/150` 个样本，wall p99=`0.639515/0.802505 s`，下游
  `/lidar_odometry` p99=`0.662703/0.881395 s`。domain `220` 还确认 Fast DDS RMW 不支持
  reception publication sequence（`supported=no`），sequence 计数 `0` 不能解释为零丢包。两次均
  `freshness_lidar_odometry`、action `ABORTED`、`p1_admission_evidence=false`；Transport observer 为额外
  subscriber，对默认链的因果归因仍是 **[Confidence: Medium]**。
- **已验证（2026-08-30 交付复核）**：MuJoCo runner 增加关键运行产物新鲜度检查，arbiter 增加断链恢复与
  车体系分量回归，MuJoCo LiDAR 增加 MuJoCo 3.4/3.10 `mj_multiRay` 参数兼容层。MuJoCo single 与
  `adapter_lease/service_timeout/input_stale` 独立故障用例通过；`red_box`、`unknown`、`unreachable` 的
  失败分别记录为真实规划缺陷、注入时序竞态和故障前提未成立，物理接触仍未验证。
- **已修复（Gazebo freshness 分类器）**：阶段表覆盖 Transport、raw LiDAR、Livox、registered scan、
  lidar odometry、odometry、localization 与 status；旧 domain `228` 的 `lidar_odometry` 首违属于阶段遗漏。
  domain `107/113` 出现 freshness 与动作结果反转，单次 run 不足以确认 owner；Direct bridge/Transport
  observer 在当前工作区尚无实测 artifact。

## 2. 当前冻结基线

### 2.1 仓库基线

| 仓库 | P0 起始 revision | 远端状态 | 说明 |
| --- | --- | --- | --- |
| 根仓 | `c7cc0e54cc7d` | `origin/develop` 已同步 | P0 阻塞记录后的文档基线 |
| 导航仓 | `5ea786eb2e70` | `origin/develop` 已同步 | ROGMap、JPS/MINCO、Goal Manager、MPC |
| Gazebo 用户 fork | `a28ccd20428f` | `origin/main` 已同步 | clean checkout 的运动学测试源已固化；建议避免向 `upstream` 写入 |
| MuJoCo | `e3d6ea7a5e61` | `origin/develop` 已同步 | 当前轮未修改 |
| `ats_robot_description` | `dea591e53fa0` | `origin/develop` 已同步 | 该提交已于 2026-08-15 经 SSH 推送；干净 Git/vcs 复建仍受本机传输失败阻塞 |

受保护的用户内容继续保留：

- `src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py`；
- `src/ats_sentry_nav/ats_swerve_mpc/求解器.md`；
- Gazebo fork `scripts/ats_bridge/gz_livox_bridge.py`。

建议避免读取其内容作为设计依据，建议避免删除、覆盖、暂存或提交。

### 2.2 已实现并有组件证据

- ROGMap 数值 projection、adapter 融合、全局 RC-ESDF display 与局部滑窗 debug 已接线；
- 两份 RViz 已分层显示全局融合 RC-ESDF、局部 ROGMap、JPS、MINCO、MPC predicted 和 executed；
- `PathGeometryPreprocessor`、`MincoTimeAllocator`、`TrajectoryQualityEvaluator` 已实现；
- 直线不增弯、曲率感知时间分配、ESDF backtracking、动态限制和安全回退有聚焦 GTest；
- Gazebo 四舵轮 `[v_x,v_y,w_z]` 运动学与命令唯一 ownership 已实现；
- ATS Goal Manager 已有 action、feedback、cancel、preempt、timeout 和有界重规划代码；
- OSQP v1.0.0、固定 CSC、warm-start ABI、complete-phase 计时和 `qp_shadow` 已实现；
- 默认仍为 `solver_mode=ilqr`，`solver_mode=qp` 显式拒绝。

### 2.3 已有运行证据但不能升级准入

- Gazebo domain `228/229` 的短直线 action 成功，终点误差约 `0.060/0.045 m`；
- domain `230` 观测到直线长度比 `1.000`、横向偏差和曲率为零；
- domain `230` 因 `/localization` wall interval
  `p50/p95/p99=0.371/0.994/1.612 s` 触发 freshness fail-closed；
- 旧 revision 的八个 Gazebo fault domain 曾通过，但 MINCO 行为修改后建议重跑；
- 历史 `qp_shadow` 仍以 `max_iterations`、零 feasible 和零 warm-start 为主，不能启用 QP 主链。

## 3. 稳定契约

- Point-LIO 继续拥有 `/localization` 和 `/registered_scan` 的状态估计输入链；
- ROGMap 不是定位器，建议避免用 ground truth 替换正式定位；
- adapter 仅消费 ROGMap 数值 projection，建议避免反解析 `/rog_map/esdf`；
- unknown、occupied、outside-map、signed-distance 正负号和 gradient 语义建议避免放宽；
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 SE(2) MPC 建议保留；
- 四舵轮控制保持车体系 `[vx,vy,wz]`，建议避免差速、Ackermann、ICR 或 `vy=0`；
- 实机/MuJoCo 中 `/cmd_vel/autonomy_raw` 的唯一 publisher 为 `ats_swerve_mpc`，
  `/cmd_vel/selected` 的唯一 publisher 为 `cmd_vel_arbiter`，最终出口订阅 selected；
  Gazebo、MuJoCo、loopback 与实机均适用该判据；历史 Gazebo `/cmd_vel_mpc` 记录建议避免误作当前
  owner 证据；
- 急停建议清空 tracker，急停前 reference 建议避免在恢复后复活；
- 不能通过增大 freshness timeout、QP iteration、residual 或 deadline 掩盖失败；
- `solver_mode=qp` 在 QP-3 门禁通过前继续拒绝启动。

## 4. 优先级与依赖关系

```text
P0 远端复建
  -> P1 Gazebo localization freshness
    -> P2 MINCO 生产契约补齐
      -> P3 Gazebo 场景 runner
        -> P4 P2 完整仿真验收
          -> P5 RViz/ROGMap 运行验收
          -> P6 P3 Nav2-free 验收
          -> P7 P4 仿真算法安全

P1/P4 通过 -> QP-2 Shadow 可配对复核 -> QP-3 受控主链切换
所有分支 -> 长时间导航稳定性 -> 受限低速实机导航
```

建议避免跳过 P0/P1 直接调 MINCO 或 QP；污染环境中的 timing 仅保存为无效样本。

## 5. P0：远端复建与 revision 冻结

### 工作项

- [x] 将 `ats_robot_description` 本地提交 `dea591e53fa0` 推送到用户远端；
- [x] 将 push remote 改为可非交互使用的 SSH 地址；
- [x] 确认活动引用只来自 `ats_robot_description`，没有 `pb2025_robot_description`；
- [x] 在临时干净目录执行 `vcs import dependencies.repos`；
- [x] 记录每个仓库实际 checkout SHA，而不是只记录分支名；
- [x] 生成验收用 locked manifest，固定关键依赖 commit；
- [x] 在干净工作区完成最窄 Gazebo 导航包构建、CTest 与 `--show-args`；
- [x] 确认不依赖本机未跟踪 bridge、旧 install 或旧 build。

### 2026-08-15 P0 完成证据

- 首次审计揭示 `rmoss_gz_resources@main` 不存在，以及 `ats_mujoco_sim` 与
  `teleop_gimbal_keyboard` 的 HTTPS 传输不稳定。远端双重核验确认
  `rmoss_gz_resources@humble=b5c759f08844dfda19c79aa870866ace8d4c7b3a`，并确认两个用户仓库
  的 SSH URL 和目标分支可达，故 `dependencies.repos` 切换到该 branch/URL 组合。
- 完整编译又揭示 Gazebo `CMakeLists.txt` 直接引用被忽略、未跟踪的运动学测试源。该问题在用户
  Gazebo fork `a28ccd20428ffc4bdd7fbbc22fee884fa1db72eb` 中修复：测试源迁入受版本控制的
  `tests/`；未触碰用户 ignored bridge/header。
- 最终全新目录 `/tmp/ats_p0_repro_final.6xNq8i` 仅由根仓 archive 与远端 manifest 建立，执行
  `vcs import --recursive --shallow --skip-existing <root>`，耗时 `152.2 s`、`rc=0`，完整取得
  22 个仓库。`dependencies.lock.repos` 由 `vcs export --exact -n` 生成，记录全部实际 SHA。
- 同一干净目录执行 `colcon build --base-paths src --packages-up-to rmu_gazebo_simulator
  --parallel-workers 1`，17 包在 `11 min 13 s` 内通过；唯一 stderr 是上游 `rmoss_base` 既有
  `pipe()` 返回值 warning。`ctest --test-dir build/rmu_gazebo_simulator --output-on-failure` 为
  `4/4` 通过，`ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py --show-args` 通过。
- 为排除 archive 不能证明 root 远端传输的边界，又在独立目录
  `/tmp/ats_p0_remote_final.Lgyalq` 以 SSH 对 `origin/develop` 执行 depth-1 root clone，得到
  `f2d049cbf245b6fdfbad0d4870e53dc3ab09cbeb`；随后直接使用受版本控制的
  `dependencies.lock.repos` 执行 exact SHA import。全流程耗时 `250.2 s`、`rc=0`，22 个依赖均
  从远端 detached checkout，未使用本机 archive、build、install 或未跟踪文件。
- 该 P0 只证明远端复建、锁定与启动前资源解析；未启动 Gazebo，不构成 freshness、P2、安全或性能结论。

### DoD

- 新电脑仅凭远端仓库和 manifest 能取得 ATS 四舵轮模型、Mid360 和 xmacro；
- 所有活动仓库 HEAD 与记录一致；
- 构建不从旧 install 解析缺失包；
- Gazebo headless 能完成启动前资源解析。

### 风险转入条件

- 机器人描述 push 失败；
- manifest 指向不存在的 commit；
- 干净目录依赖未授权本机文件；
- 发现向 Gazebo `upstream` 写入的风险。

## 6. P1：定位 Gazebo localization freshness 根因

### 最短链路

```text
/clock + Gazebo real-time factor
-> /lidar_odometry
-> sensor_scan_generation /odometry
-> localization_fusion /localization
-> /localization/status
-> /rog_map_adapter/ready
```

### 插桩与指标

- [x] 单一 C++ recorder 同时订阅 `/clock`、`/cloud_registered`、`/lidar_odometry`、`/odometry`、
  `/localization` 和 `/localization/status`；
- [x] 每级记录 steady wall arrival、ROS stamp interval、`/clock` 相对 stamp age、重复/倒退 stamp、
  消息数和最大 gap；
- [x] 同一 recorder 记录其对 `/clock`、三段 odometry、`/localization/status` 和 adapter status 的
  callback 执行时长分布；它只量化观测器自身开销，不可替代行为 owner 的 executor/queue trace；
- [x] 记录 `/clock` wall interval、sim-time interval 与 RTF 分位数；
- [x] 以 `map -> gimbal_yaw_odom` 的实际零超时查询记录 TF lookup attempt/success/failure/max duration；
- [x] runner 只对本 launch session 内的 bridge、Point-LIO、loam、sensor generation、fusion、ROGMap
  与 adapter 写入 CPU tick、RSS、线程和 voluntary/nonvoluntary context-switch 两次原始快照；
- [x] DDS queue/drop 无可移植 RMW counter 时显式写入
  `unverified_no_portable_rmw_counter`，建议避免当作零丢包；
- [x] 可选 Gazebo Transport observer 实际记录 source cadence；domain `219/220` 分别为
  `599/600` 个样本且 wall p99 `0.108672/0.105195 s`；
- [x] 原始 ROS PointCloud2 reception publication-sequence 字段在当前 Fast DDS 实际探测为
  `supported=no`；不以零值推断零丢包；
- [x] runner 按 `/clock -> /lidar_odometry -> /odometry -> /localization -> status` 顺序输出
  `p1_first_freshness_violation`；分类器只消费单行 recorder witness，不修改运行时 timeout；
- [x] 审计 generic `ros_gz_bridge` 的实际部署 owner：活动 `parameter_bridge` 来自系统安装包
  `ros-humble-ros-gz-bridge 0.244.25-1jammy.20260608.160002`，workspace 没有该包源码；项目只拥有
  topic/YAML 与 ROS publisher QoS 配置面。对应 upstream `0.244.25` 的 GZ-to-ROS 回调同步完成
  `PointCloudPacked -> PointCloud2 -> publish()`，且 `create_gz_subscriber()` 未使用传入的
  `subscriber_queue_size`；`rmoss_gz_bridge` 只构建 pose/RFID bridge，不是该 LiDAR owner；
- [ ] 以 generic bridge 内部发布计数或本机 Fast DDS Statistics 区分 publisher 未发布与 DDS
  subscriber 丢样；当前 RMW sequence 字段不足以完成此归因；
- [ ] A/B 每次只改变一个因素：headless、recorder、RViz、相机、LiDAR profile、日志；
- [ ] 所有 profile 使用新 domain、相同 revision、相同起点和固定窗口。

### 2026-08-15 至 2026-08-19 P1 插桩、运行前审计与诊断

- **已验证（组件）**：Gazebo fork 的 `EvidenceStatistics` 确定性 CTest、`rmu_gazebo_simulator`
  单 worker Release build、完整包级 CTest（`32 tests, 0 errors, 0 failures`）、recorder callback
  duration 统计、runner `bash -n`、`ats_gazebo_nav.launch.py --show-args` 与相关 diff check 已通过。
  sandbox 下完整包 CTest 的 `ament_black` 会因 Python `SyncManager` 无法创建本地 socket 失败；这不是
  本轮 C++ 统计测试失败，仍需要主机环境复跑。
- **已验证（runner/回归）**：`scripts/gazebo_freshness_classifier.sh` 按固定阈值 p99 `<0.25 s`、最大
  wall gap `<=0.5 s`，在 `/clock -> /lidar_odometry -> /odometry -> /localization -> status` 顺序中输出
  首个可见违反者；`scripts/test_gazebo_freshness_classifier.sh` 覆盖上游首违、健康与缺字段输入。
  `scripts/test_gazebo_runner_contract.sh` 锁定 runner 仅保留合法 ROS domain 与残留导航/仿真进程的启动前
  审计。`p1_admission_evidence=true` 仅要求无故障注入、至少 60 s observer、无
  freshness 首违、status 全部 TRACKING、TF 无失败和 straight action 成功；分类器不改变安全 timeout 或行为参数。
- **已验证（domain 合法性）**：domain `233` 在当前 Fast DDS portBase 下报 `Calculated port number is too high`，
  多个 ROS 节点立即退出且 `/clock` 不推进。因此后续运行仅使用 `0..232` 的新 domain。
- **已验证（历史诊断运行，2026-08-19）**：合法 domain `220`、headless、`RUN_DURATION_SEC=30` 已启动
  Gazebo、传感器、localization、ROGMap/adapter、MINCO 与 iLQR MPC。recorder 记录 `/clock` wall
  p99/max=`0.122538/0.135193 s`，`/lidar_odometry`=`1.671385/1.671385 s`，`/odometry`=
  `1.659046/1.659046 s`，`/localization`=`1.663433/1.663433 s`；RTF p50/p95/p99=
  `0.199802/0.409690/0.596497`，status `TRACKING/non-TRACKING=178/119`，TF lookup `18/300` 失败。
  action 在 30 s 内未成功，但 `/cmd_vel_mpc` 曾有非零导航速度输出。
  分类器输出首个可见违反者为 `/lidar_odometry`。
- **推断 [Confidence: Medium]**：`/lidar_odometry` 是该窗口中最早违反 wall cadence 的可观测边界；下游
  `/odometry` 与 `/localization` 具有同量级 gap，recorder callback p99 为微秒级，因此现有证据不支持将
  首因归给 recorder 或 `localization_fusion` callback 阻塞。仍无法在 Gazebo 传感器负载/RTF、
  `loam_interface` publisher cadence 与 DDS subscriber 丢包之间唯一归因；DDS queue/drop 为
  `unverified_no_portable_rmw_counter`。
- **已验证（P1 60 s baseline，2026-08-19）**：新合法 domain `224`、headless、`P2_FAULT_CASE=none`、
  `planning_grid_owner=rog_map` 的 runner 返回 `0`。recorder 实际 `completed=yes`、`duration_s=60.003753`，
  修复了短 action 截断观察的旧缺口：`p1_admission_evidence` 现在还要求 recorder 正常完成且实际 duration
  不短于请求窗口。action 成功，最终误差 `0.146 m`；JPS/reference/predicted/executed 点数为
  `3/32/31/10`，`/cmd_vel_mpc` 曾有非零导航速度输出。运行期
  `/rc_esdf/planning_grid` 与 `/cmd_vel_mpc` 均为单一 publisher，terminal `emergency_stop=true`，
  `/cmd_vel_mpc` 采样为零。无残留进程，启动前审计通过。
- **未通过（P1 freshness）**：该实际 60 s 窗口中 `/lidar_odometry` p99/max wall interval 为
  `1.144617/1.488598 s`，下游 `/odometry`=`1.143652/1.490561 s`、`/localization`=
  `1.142361/1.490312 s`，故首违仍是 `/lidar_odometry`；RTF p50/p95/p99=
  `0.331409/0.502033/0.560295`，status `TRACKING/non-TRACKING=535/56`，TF lookup failure=`16/600`。
  `p1_admission_evidence=false`，原因为 `freshness_lidar_odometry`。该结果不能由单次 action 成功或
  终态 `/cmd_vel_mpc=0` 升级为当前 revision 的导航准入。
- **推断 [Confidence: Medium]**：`loam_interface` 仅在 `cloud_registered` callback 中发布
  `/lidar_odometry`；其 ROS stamp p99=`0.299990 s` 而 wall p99=`1.144617 s`，同时 `/clock` RTF
  p50=`0.331409`。证据优先支持检查 Gazebo 传感器/RTF、Point-LIO publisher cadence 和 DDS 接收边界，
  不支持把首因唯一归给 loam callback。下一实验建议以新 domain 只改变一个因素并补足 upstream publisher
  与 subscriber/drop 的独立计数。
- **已验证（最终 revision P1 60 s baseline，2026-08-19）**：显式 `ENABLE_CAMERA_SENSORS=false` 的新
  domain `225` recorder 实际 `completed=yes`、`duration_s=60.010472`，启动前无残留进程，planning grid、
  `/cmd_vel_mpc` 为单一 active publisher 且曾有非零导航速度输出；terminal `emergency_stop=true`，
  `/cmd_vel_mpc` 采样为零。此 final revision action
  被接受但在 `90 s` 内未给出终态，runner 以 `nominal action did not succeed` 返回失败。action 不成功不被
  隐藏为环境条件，也不影响完整 60 s recorder 的有效性。
- **未通过（最终 P1 freshness）**：domain `225` `/lidar_odometry` p99/max wall interval=
  `2.759540/2.942285 s`，下游 `/odometry`=`2.764472/2.946450 s`、`/localization`=
  `2.764479/2.944695 s`；RTF p50/p95/p99=`0.303726/0.803858/1.017098`，status
  `TRACKING/non-TRACKING=419/158`，TF lookup failure=`27/600`。首违仍为 `/lidar_odometry`，
  `p1_admission_evidence=false`。domain `224` action 成功与 domain `225` action 超时共同表明当前 P1
  不具有可重复的 action 成功证据。
- **未验证**：adapter ready=false 计数与持续 lease、publisher/subscriber/DDS 分层、重复 action 的统计
  稳定性、camera true/false 等单因素 A/B、P2 red-box 和实机导航。P1 建议避免因单次 domain `224` 成功 action
  标记通过。
- **已验证（raw Gazebo LiDAR 分层，2026-08-20）**：recorder 已在独立 `60 s` domain 实际订阅
  `/<robot>/livox/lidar`、`/livox/lidar`、`/cloud_registered` 与 `/lidar_odometry`。domain `215` 的
  `4 ms` physics candidate 在默认 `10 Hz / 625 x 32`、off-screen rendering 下得到 raw/lidar-odometry/
  localization wall p99=`1.610141/1.435872/1.430092 s`，action 因 `trajectory footprint is unsafe`
  ABORTED；该 world 不能替代 P4 的默认物理精度，也不能成为 P1 默认。raw 与 bridge 的 callback p99
  分别仅 `8/16 us`，wall cadence 同阶，不能通过下游 Point-LIO、loam、MINCO、MPC 或 timeout 调整修复。
- **未通过（LiDAR timing candidate，2026-08-20）**：runner 新增显式 `LIVOX_UPDATE_RATE_HZ`，使 SDF
  `update_rate`、bridge `scan_period_sec` 与 Point-LIO `mapping.lidar_time_inte` 同时由同一频率推导。
  domain `214` 的 `5 Hz / 625 x 32` 使用 `0.2 s` 三处一致周期，但 raw/lidar-odometry/localization wall
  p99 恶化为 `3.666797/3.312480/3.308619 s`，action ABORTED；`5 Hz` 建议避免成为默认。
- **未通过（headless world candidate，2026-08-20）**：`rmuc_2025_navigation_headless_world.sdf` 仅移除
  GUI state 的 `SceneBroadcaster`，保留默认物理步长、Physics、Sensors、IMU、用户命令和场景几何。
  短时 server 可推进 `/clock`，但 domain `213` 的 `10 Hz / 625 x 32` raw/lidar-odometry/localization wall
  p99=`1.409273/1.322408/1.319348 s`，status `TRACKING/non-TRACKING=448/125`、TF failure=`35/601`，
  action 在位移 `0.7409 m` 后仍以 unsafe trajectory ABORTED。该 world 仅保留为可复现的 rejected A/B，
  不是活跃导航或 P1/P2 通过证据。
- **已修复（runner world override）**：空 `WORLD_SDF_PATH` 不再生成无效的 `world_sdf_path:=` 参数；只有
  非空候选 SDF 路径才转发至 launch，默认 world-name 解析保持不变。`LIVOX_UPDATE_RATE_HZ=10.0` 与
  `LIVOX_HORIZONTAL_SAMPLES=625` 仍是默认配置。
- **下一定位边界 [Confidence: Medium]**：三个候选均未使 raw LiDAR cadence 达到 P1，且 raw、bridge、
  Point-LIO output 和 `/lidar_odometry` 的 wall gaps 仍同阶。下一项应区分 Gazebo sensor publisher 变慢与
  DDS subscriber 接收缺口；建议避免重复降低频率、改变 physics step 或移除 SceneBroadcaster 来宣称活跃导航。
- **未通过（默认配置复核，2026-08-20 domain 208）**：在关闭 Transport observer、关闭 Direct bridge、
  generic `RELIABLE/KeepLast(10)`、`10 Hz / 625 x 32`、关闭相机、headless off-screen rendering 与
  `planning_grid_owner=rog_map` 下，recorder 完整覆盖 `60.016403 s`。`/clock` wall p99/max 为
  `0.078741/0.351209 s`，但 `/<robot>/livox/lidar`、`/livox/lidar`、`/cloud_registered`、
  `/lidar_odometry`、`/localization` 的 wall p99 分别为
  `2.954985/2.958803/3.327495/3.327198/3.324217 s`；首违仍是 `lidar_odometry`，status
  `TRACKING/non-TRACKING=362/210`、TF failure=`35/600`、`p1_admission_evidence=false`。该 result
  排除了“仅 Transport 诊断 observer 导致默认配置失效”的简单解释，但单次运行仍不能唯一归因 sensor、
  generic bridge 或 DDS。
- **已验证（domain 208 安全行为）**：JPS/reference/MPC/轮关节和三段命令均曾非零，运行期 planning grid、
  `/cmd_vel_mpc` 仍有唯一 active publisher；动作最终 `ABORTED`、终态 `emergency_stop=true` 且
  `/cmd_vel_mpc` 为零。连续 swept 与 MINCO 离散 footprint 冲突采样仍为 `未验证`，建议避免由此次
  fail-closed 推导。
- **未通过（最新默认 P1，2026-08-23 domain `228`）**：在默认 `10 Hz / 625`、headless、
  `planning_grid_owner=rog_map`、`OBSERVE_GAZEBO_TRANSPORT_LIDAR=false`、
  `USE_DIRECT_GAZEBO_LIDAR_BRIDGE=false` 与 generic `RELIABLE/KeepLast(10)` 下，recorder 实际完成
  `60.001345 s`。`/clock` wall p99=`0.010387 s`、RTF p99=`1.032094`，但 raw Gazebo LiDAR、
  `/lidar_odometry`、`/localization` 的 wall p99/max 分别为 `0.708598/0.714906 s`、
  `0.650163/0.743549 s`、`0.650164/0.743591 s`。首违仍是 `lidar_odometry`，status
  `TRACKING/non-TRACKING=585/15`、TF failure=`7/600`，`p1_admission_evidence=false`。JPS/MINCO/MPC
  path 为空且 selected 没有非零样本，是定位/地图 fail-closed；不支持将根因唯一归为 Gazebo sensor、
  generic bridge、DDS 或 Point-LIO。下一步仅审计 generic bridge 的可修改 owner，或增加不改变默认链的
  publisher/DDS 分层计数。
- **已验证（P1 owner 审计，2026-08-24；部署 + upstream 源码）**：domain `228` 的第一个已测 ROS
  边界 `/<robot>/livox/lidar` 由 `/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge` 发布；其
  转换回调、Gazebo Transport 接收线程和内部发布计数均不在四个项目仓库的可修改源码内。
  `ros_gz_bridge.yaml`/launch 可配置 topic、方向和 ROS publisher `RELIABLE/KeepLast(10)`，但不能在
  当前 workspace 内修改或观测 generic bridge 的实际 GZ 接收回调；`rmoss_gz_bridge` 也不拥有这条映射。
  因此触发“owner 不在项目可修改范围”的风险转入条件，本轮没有修改源码、没有新增计数，也没有占用新
  `ROS_DOMAIN_ID`。最新运行证据仍仅为上述 domain `228` 原始 artifact，P1 DoD 仍未通过；在显式纳入并
  授权维护 `ros_gz_bridge` 对应源码，或批准项目外 trace 方案前，建议避免继续用下游 timeout/QoS/Point-LIO
  改动替代该边界诊断。

### 修复原则

- 只修改最早违反 freshness 的行为 owner；
- 不提高 `odom_timeout_s`、localization timeout、adapter lease 或 map timeout；
- 不把 ground truth 接入正式 `/localization`；
- 不用 ros2cli 高频 observer 干扰被测链，优先单一 C++ recorder；
- 降低传感器负载优先结合感知质量与闭环指标共同评估。

### DoD

- [ ] 低负载 headless 连续至少 `60 s`，`/localization` p99 interval `< 0.25 s`；
- [ ] 同一窗口不存在 `> 0.5 s` 的 localization gap；
- [ ] status 持续 TRACKING，adapter 不因 localization 反复 `ready=false`；
- [ ] stamp 不倒退，sensor-to-localization age 有 p50/p95/p99；
- [ ] straight action 两次成功，终点误差、owner 和收尾零速均通过；
- [ ] 修复有聚焦单测或 deterministic fault test。

### 首违分类设计

分类器只处理 recorder 的结构化单行结果，契约顺序固定为：

```text
/clock(max wall gap <= 0.5 s)
-> /lidar_odometry(p99 < 0.25 s, max gap <= 0.5 s)
-> /odometry(p99 < 0.25 s, max gap <= 0.5 s)
-> /localization(p99 < 0.25 s, max gap <= 0.5 s)
-> /localization/status(p99 < 0.25 s, max gap <= 0.5 s)
```

缺字段输出 `first_violation=unverified` 并阻止 admission 结论；首违输出只作为定位证据，不能
替代 publisher/DDS/RTF 独立实验。正式 P1 仍要求新 domain、固定 revision、60 s headless 和两次
straight action。

### 风险转入条件

- 残留导航/仿真进程、非法 ROS domain、Gazebo z 发散、RTF 异常、TF 冲突或多个 localization publisher；
- unknown/lease/emergency stop 异常、关键 telemetry 缺失或系统失稳；
- 需要放宽安全 timeout 才能通过；
- 当前机器的非侵入式计数/trace 仍无法区分上游发布慢和下游丢包；转移到性能更高机器后建议先用全新
  ROS domain 重跑默认 `60 s` 基线，再比较同 revision 的独立边界证据。

## 7. P2：补齐 MINCO 生产契约

P1 Gazebo freshness 是运行准入门，不再阻塞不依赖 Gazebo 的算法实现、组件测试和 MuJoCo 验证。
在 P1 通过前可以完成本节的接口、数学与 fail-closed 行为，但建议避免把组件结果写成 Gazebo/P2 闭环通过，
也建议避免用算法改动掩盖 generic bridge 的 freshness 失败。

### 7.1 当前运动状态接入

- **已验证缺口（2026-08-24）**：`InitialKinematicState`、首端裁剪和初速度参与时间分配已经存在于
  `MincoTrajectoryOptimizer`，但 production node 的 center、footprint、JPS fallback、repair 四次
  `optimize()` 调用仍都传 `nullptr`；node 本身也没有与 localization epoch 原子绑定的运动状态。
- **接口决策**：由已经订阅 `/localization` 并拥有 localization epoch 的 `ats_goal_manager`，在同一互斥区内
  将规划系线速度、观测 stamp、epoch、goal/request 和 map publication sequence 一起冻结到
  `PlannerGoal`。MINCO 不再另建一个无法与 epoch 原子绑定的 odometry cache。
- **参考证据边界**：`参考/navi_minco_bit` 只证明“实时起点状态、上一轨迹剩余段、曲率/制动时间分配、
  独立 yaw、最终轨迹复核”是可行机制。建议避免复制其 Nav2 plugin/FSM、communication、双雷达或协议；其
  `determinePlanningState()` 还存在日志声称 COLD_START、代码却返回 HOT_START 的反例，不能照搬状态机。

- [ ] `MincoPlannerNode` 获取与 goal/snapshot 同一 localization epoch 的新鲜状态；
- [ ] 明确 twist frame、单位和时间，不假定速度已经是世界系；
- [ ] 将车体系速度正确旋转到规划世界系；
- [ ] 无可靠加速度时只播种速度，加速度保持零；
- [ ] stale、epoch 不匹配、非 finite 或 TF 失败时不用该状态；
- [ ] center、footprint、fallback、repair 使用同一个冻结初始状态；
- [ ] telemetry 记录原值、裁剪值、stamp age 和拒绝原因。
- [ ] production 请求缺少新鲜状态时保持急停并返回结构化失败；legacy 直连目标若保留零初值，建议明确
  标为兼容路径且不能作为 P2 准入证据；
- [ ] center、footprint、fallback、repair 分别记录候选类别和同一个冻结状态 identity，不能只记录最终
  `selected_trace` 后丢失被拒候选证据。

测试至少覆盖：

- [ ] 非零 yaw 下横移速度转换；
- [ ] 速度/加速度 finite 与上限裁剪；
- [ ] localization epoch 变化拒绝旧状态；
- [ ] 四条 optimizer 调用路径不再直接传 `nullptr`；
- [ ] 重规划首端速度连续，终端速度/加速度仍为零。

### 7.2 把质量 telemetry 升级为生产门禁

质量门禁建议区分直线与一般曲线，不能用起终点直线偏差拒绝合法 S 弯。

- [ ] 直线类限制 length ratio、横向偏差、曲率峰值/TV 和符号变化；
- [ ] 一般曲线相对 preprocessed guide/baseline 比较长度、偏差、曲率 TV 和净空；
- [ ] 所有类检查 v/a/j、时间单调、footprint/swept collision 和 snapshot freshness；
- [ ] ESDF candidate 同时满足净空不下降、碰撞不增加、长度和曲率变化不过门；
- [ ] 阈值进入唯一实际加载配置，并有参数范围校验；
- [ ] 记录结构化首个拒绝原因；
- [ ] quality 失败只回退到同 snapshot 上安全的 baseline；
- [ ] baseline 也失败时不发布 reference，保持急停、`/cmd_vel/selected=0` 与最终执行端为零。

### 7.3 净空与连续性

- [ ] center clearance 与 oriented-footprint clearance 分开记录；
- [ ] ESDF backtracking 使用同一 immutable snapshot；
- [ ] unknown、outside、非 finite gradient 和 snapshot 变化立即拒绝；
- [ ] noisy gradient 不产生交替法向偏移；
- [ ] 单拐角不会让无关直线段一起减速；
- [ ] 非零 initial-state 时重新检查首段连续曲线净空和动态极值；
- [ ] repair 输出重新求解 MINCO、yaw、时间和完整安全门。

### 7.4 参考对照后的计算量与时延优化

- [ ] 修复 `MincoTimeAllocator` 把非零首端速度静默压到 `reference_speed` 以下的问题；首点速度建议等于
  裁剪后的实际边界速度。若剩余路径在 `max_acceleration` 下无法降到终端零速，应在时间分配阶段结构化
  拒绝，不能先用不一致速度求解再消耗多轮 dynamic scaling；
- [ ] 将 `PathGeometryPreprocessor::preprocess()` 从每个候选重复执行改为每个 `planGoal()` 只生成一次
  immutable prepared seed，center/footprint/fallback/repair 共用同一 seed 与 map snapshot；
- [ ] 保留兼容 `optimize(raw_path, ...)` 包装，但 production 走 `prepare + optimizePrepared`，并用数值等价
  单测证明重构不改变路径、时间、yaw 或安全语义；
- [ ] 分候选记录 preprocessing、ESDF refinement、MINCO solve、safety recheck 和总 wall time 的
  p50/p95/max；只有测得 dominant stage 后才继续做上一轨迹热启动或缓存；
- [ ] 不照搬参考工程的 `1.5 x` severe dynamics 容忍、invalid duration 视为 safe、tracking error 仍返回
  HOT_START、旧轨迹无 generation 复用等行为；这些都比 ATS 当前 fail-closed 契约更弱；
- [ ] 若 prepared seed 后仍考虑热启动，先证明当前 closed-form MINCO S3/ESDF refinement 存在可复用的
  优化变量和至少 `20%` p95 wall-time 收益，再单独设计；建议避免只为对齐参考工程引入共享可变轨迹状态。

### DoD

- [ ] 库级、node 级和旧 reference 竞态测试通过；
- [ ] 直线、冗余共线、短首尾段、单角、S/U 弯、窄通道、noisy ESDF fixture 通过；
- [ ] 门禁能拒绝“finite 但无意义多弯”的候选；
- [ ] initial-state 在实际 node 路径生效；
- [ ] 不改变 JPS、MINCO S3、独立 yaw、地图和速度 owner。

## 8. P3：实现 Gazebo 确定性场景 runner

当前 `TEST_PROFILE` 只用于日志命名，建议升级为实际行为 owner。

| profile | 固定输入 | 核心判据 |
| --- | --- | --- |
| `straight` | 两点自由空间、零/非零 yaw | 不增弯、终点与停止 |
| `single_corner` | 一个必要拐角 | 拐角减速、无 overshoot |
| `s_turn` | 两次相反转弯 | 无多余摆动、曲率符号正确 |
| `narrow_corridor` | yaw-aware footprint 可通过窄通道 | clearance、无错误 shortcut |
| `nominal` | 固定任务路线 | 端到端成功和 owner |
| `red_box` | 固定多段红框 | 每段 action 生命周期和恢复 |

### 工作项

- [ ] `case "$TEST_PROFILE"` 拒绝未知 profile；
- [ ] 每个 profile 固定 world、起点、目标序列、yaw、timeout 和路径类别；
- [ ] action 根据 `GOAL_YAW` 生成规范化 quaternion；
- [ ] 保存实际目标 payload 和 scenario manifest；
- [ ] 场景建议避免依赖人工 RViz 点击；
- [ ] 每个 goal 独立记录 accepted/result/cancel/preempt/timeout；
- [ ] 保存 raw/preprocessed/refined/reference/predicted/executed；
- [ ] 保存地图 identity、owner、clearance、碰撞采样、v/a/j 和 terminal error；
- [ ] 未验证 clearance 或碰撞采样不写默认通过值；
- [ ] shell/Python/C++ 测试锁定 profile、yaw 和未知 profile 拒绝。

### DoD

- 每个 profile 至少两个新 ROS domain；
- 结果与固定 manifest 可配对；
- profile 名称确实改变输入和验收逻辑；
- 失败保留首因和 artifact，不污染后续场景。

## 9. P4：完成 P2 仿真验收

### 名义与边界场景

- [ ] `straight`、`single_corner`、`s_turn`、`narrow_corridor`、`nominal`、`red_box`
  各两个独立 domain。

每次建议记录：

- terminal pose、位置/yaw 误差、总耗时；
- JPS/MINCO/MPC/executed 点数和五层 payload；
- length ratio、横向偏差、曲率 max/p95/TV/符号变化；
- v/a/j peak/p95、segment duration、time-scaling 次数；
- center/footprint minimum clearance；
- discrete/swept collision samples；
- replan、fallback、repair、失败和恢复次数；
- localization/map/reference/command age；
- planning grid、`/cmd_vel/autonomy_raw` 与 `/cmd_vel/selected` 唯一 owner；

### 当前 revision 故障矩阵

每例使用独立 domain 和全新 launch：

- [ ] all-unknown、map-unready、map-stale、input stale、unreachable；
- [ ] adapter lease、projection timeout、emergency-stop recovery；
- [ ] localization jump/epoch、TF loss、runtime unsafe、process restart/late response。

共同 DoD：

```text
failure detected within configured deadline
-> ready=false or planner failure
-> emergency_stop=true
-> /cmd_vel/selected=0
-> final actuator input=0
-> old reference cannot revive
```

恢复建议满足 generation/sequence 继续推进，且只有新目标或新 request identity 才恢复运动。

只有唯一 owner、immutable snapshot、fail-stop、名义/红框、故障矩阵、clearance 与碰撞证据全部完成，
才能标记 P2。单次直线成功或旧 revision fault 不能替代。

## 10. P5：ROGMap/RViz 运行验收

- [ ] 静止时保存全局融合 RC-ESDF、局部 RGB voxel 和三色 bounds 同帧截图；
- [ ] 机器人直线移动 `3 m`，记录 visualization/local/update bounds center；
- [ ] 跨 `1.0 m` sliding threshold 时 local-map center 正确更新；
- [ ] visualization center 相对机器人误差不超过一个 ROGMap cell；
- [ ] `Decay Time=0` 下旧局部点不残留；
- [ ] 全局 grid origin/尺寸不随机器人漂移；
- [ ] A/B/C：headless、global-only、global+local，各至少两次；
- [ ] 比较 projection、adapter、map-lock、debug p50/p95/p99；
- [ ] RViz 退化超过 `20%` 时停止定位，不放宽 deadline；
- [ ] 实车 RViz 配置做 YAML/QoS/fixed-frame 静态验证。

## 11. P6：P3 Nav2-free 运行准入

- [ ] `launch_nav2:=false`；
- [ ] graph 中无 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`；
- [ ] MINCO 不订阅 `/plan`；
- [ ] 目标入口只使用 ATS action 或受控 `/goal_pose`；
- [ ] feedback、success、abort、cancel、preempt、timeout 全部运行验证；
- [ ] cancel/preempt 后连续零速度；
- [ ] timeout 后旧 planner result 不能重新授权；
- [ ] 扩大矩形和 red-box 均由 ATS action 完成；
- [ ] Goal Manager restart、late joiner、map/localization wait 有确定性结果；
- [ ] action result、emergency stop 和 ExecutionCommand identity 可审计。

在上述完成前仅写“Nav2-free 代码路径存在”，不能写“P3 已通过”。

## 12. P7：P4 仿真算法安全

### 仿真阶段

- [ ] 将自适应 sampled sweep 升级为具有明确误差上界的连续 swept 契约；
- [ ] 覆盖纯旋转、横移、对角、`+pi/-pi`、高曲率和 map 边界；
- [ ] 验证 planner collision、footprint gate、Local Collision Repair 与 unsafe trajectory
  都不会提交不安全 reference；
- [ ] 注入 map stale、unknown、localization stale、无路、目标取消和 solver failure，验证
  `emergency_stop=true -> /cmd_vel/selected=0 -> final actuator input=0`；
- [ ] 验证 map snapshot、generation、reference timestamp 和 goal identity 不会让旧轨迹复活；
- [ ] 长时间运行无 queue/RSS/thread/generation 异常增长。

### 下位机边界

CAN、电机、轮速、电流、电压、温度、底盘反馈、硬件 watchdog、接触和制动诊断由下位机/HIL
链独立维护。本导航仓只保证向 `/cmd_vel` 下位机速度接口提交经过云台 yaw 变换的速度，不订阅、
不记录、也不以这些硬件信号决定导航 action 成功或失败。

## 13. QP：Shadow 到受控主链

### QP-2 真实 Shadow

- [ ] 接入与 iLQR 同一 snapshot 的真实 map-health 和 footprint/collision producer；
- [ ] 仅在真实输入成立后移除临时 `map_fresh=false`、`collision_free=false`；
- [ ] freshness/P2 通过后运行 paired A/B/C；
- [ ] identity digest 不可比时 analyzer 输出 `not_comparable`；
- [ ] 定位 OSQP `max_iterations` 的矩阵尺度、conditioning、active bounds 和 warm-start；
- [ ] 建议避免提高 iteration、放宽 residual/time limit 或接受 `solved_inaccurate`；
- [ ] 得到稳定 `solved`、residual、hard margin、slack 和 warm-start 分布；
- [ ] 记录 complete phase、full callback、CPU/allocation p50/p95/p99；
- [ ] 覆盖 nominal、yaw jump、速度阶跃、反向、横移和 navigation fault matrix。

### QP-3 受控主链

- [ ] candidate 全部 hard-check 通过时才能发布 `controls.front()`；
- [ ] infeasible、deadline、residual、slack、map、collision、lease 失败均零速度；
- [ ] iLQR 保留为可选 baseline；
- [ ] fallback 建议限定窗口并重新验证旧序列；
- [ ] 建议避免直接沿用 `last_control`；
- [ ] 节点级覆盖 solved/infeasible/time-limit/solved-inaccurate/residual reject；
- [ ] MuJoCo navigation matrix 通过后，QP 主链候选再进入导航评审；
- [ ] 默认切换需要单独评审和回滚点。

## 14. 长时间稳定性与性能

- [ ] 固定 `Release/-O3`、线程、CPU governor、middleware 和 power mode；
- [ ] 关键 profile 预热后至少运行 `10 min`，另做 `30--60 min` soak；
- [ ] 记录 RTF、CPU/RSS、threads、context switches、DDS drops、queue depth；
- [ ] 记录 localization、projection、adapter、MINCO、iLQR/QP、callback p50/p95/p99/max；
- [ ] 记录 control jitter、deadline miss、command age 和 tracking error；
- [ ] 检查 generation、sequence、goal ID 和 telemetry ring 是否倒退或增长；
- [ ] RViz、INFO logging、recorder 分别做 A/B/C 消融；
- [ ] 性能结论附测试机、revision、配置、样本数和 artifact；
- [ ] 未在目标机测量前建议避免引用 `50 Hz`、`6 ms` 或报告内存数值。

## 15. 导航侧 Gate 0--2

### Gate 0：静态与数学

- [ ] frame/time/QoS/map/ESDF/generation/ownership 账本完整；
- [ ] 状态、矩阵、参数和 finite 检查通过；
- [ ] 速度、加速度和 jerk 约束来自当前导航模型与固定配置，并保留可审计单位；
- [ ] 启动、部分初始化、reset 和 shutdown 都输出安全状态。

### Gate 1：离线/回放

- [ ] nominal、边界、stale、乱序、clock jump、NaN、overload 回放；
- [ ] 与冻结 iLQR/JPS-MINCO baseline 成对比较；
- [ ] 关键故障在 deadline 内零速度。

### Gate 2：仿真

- [ ] P2/P3/P4 仿真条目通过；
- [ ] Gazebo 与 MuJoCo 结论一致或差异已解释；
- [ ] clearance、tracking、deadline 和 recovery 有导航侧 artifact。

### 受限低速实机导航观察

- [ ] 独立物理急停与安全观察员就位；
- [ ] 先直线停止，再横移、原地旋转、单拐角；
- [ ] 再进入窄通道、重规划和边界场景；
- [ ] 每次只提升一个能量或复杂度维度；
- [ ] 异常时优先安全停车，并保留冻结 revision 作为回滚点。

## 16. 通用验证命令

```bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select <changed_packages> --parallel-workers 1

colcon test --base-paths src --packages-select <changed_packages> \
  --event-handlers console_direct+ --parallel-workers 1

colcon test-result --test-result-base build/<package> --verbose
python3 -m py_compile <changed_python_files>
bash -n <changed_shell_files>
ros2 launch <package> <launch_file> --show-args
git diff --check
```

仿真建议使用新 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1`、`ROS2CLI_DAEMON=false`、headless
性能基线、`planning_grid_owner=rog_map` 和 `solver_mode=ilqr`（QP-3 前）。

## 17. Artifact 最小字段

- 仓库 SHA、dirty state、effective params、world/map、domain、时间；
- 目标序列/yaw、action feedback/result；
- topic type/frame/QoS 和 ownership；
- localization/map/reference/command age；
- source generation、adapter publication、MINCO snapshot identity；
- raw/preprocessed/refined/reference/predicted/executed payload；
- clearance、碰撞采样、v/a/j、tracking 和 terminal error；
- CPU/RSS/RTF、p50/p95/p99/max 和 deadline misses；
- first failure、stop/recovery、旧 reference 拒绝和最终零速度；
- 未执行项和不能得出的结论。

## 18. 提交与文档规则

- 每轮开始与结束检查根、导航、Gazebo、MuJoCo 和机器人描述仓；
- 建议避免 `git add .`、`git add -A`、force push、历史重写和破坏性恢复；
- 只显式 stage 本轮文件；
- 根、导航、MuJoCo 分别提交自己的 `develop`；
- Gazebo 只推送用户 `origin/main`，建议避免向 `upstream` 写入；
- 无修改仓库建议避免制造空提交；
- 作者固定为 `liukong1220 <1625038134@qq.com>`；
- 本清单只更新状态，不再次追加完整运行流水账；
- 原始结果放 artifact，准入结论放状态文档，QP 证据放 backend admission。

## 19. 下一阶段执行顺序

1. P1 基础设施线：形成维护 `ros_gz_bridge 0.244.25` 源码或项目外 trace 的明确边界后，再继续
   generic bridge/Gazebo freshness 修复；下游 timeout 与 Point-LIO 保持现状，不作为绕过手段；
2. P2 算法线：现在即可完成 `PlannerGoal` 原子运动状态、MINCO 四候选初值连续性、制动可行时间分配、
   prepared seed 复用、候选分类 telemetry、组件测试和 MuJoCo 非零速度重规划；
3. P1 通过后，再用最终算法 revision 执行两个 Gazebo straight 和 P2 Gate 2，建议避免复用算法修改前 artifact；
4. 完成 P3 确定性 profile runner；
5. 再进入 P4 场景和故障矩阵。

P1 未通过时，Gazebo/P2 准入声明、MINCO 参数搜索、QP 主链和性能结论暂缓；带有确定性单测和
独立 MuJoCo 证据的算法源码实现仍可继续。下一算法会话使用
`docs/项目优化文档/下一阶段提示词_ATS_MINCO动力学连续重规划.md`。
