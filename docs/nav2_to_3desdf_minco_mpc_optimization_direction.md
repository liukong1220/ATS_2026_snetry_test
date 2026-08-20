# ATS 自研导航 V1 当前状态与优化方向

> 更新时间：2026-08-20
> 本页只记录当前准入状态、不可破坏的架构边界和下一执行入口。历史阶段流水账已从活动文档移除，
> 仍可由 Git 历史和专项准入记录追溯。

## 1. 当前架构

```text
Point-LIO /localization + /registered_scan
-> ROGMap 概率占据/膨胀/3D ESDF
-> ats_rog_map_adapter 地面投影与 static/terrain/slope/unknown 融合
-> PlanningMapSnapshot + RC-ESDF
-> ATS Goal Manager
-> JPS/A* fallback
-> MINCO S3 + independent yaw
-> oriented footprint + sampled swept safety + optional local repair
-> holonomic SE(2) iLQR MPC
-> /cmd_vel_mpc -> /motion_control
-> four-wheel independent steer/drive chassis
```

活动实现不得替换 Point-LIO、ROGMap、RC-ESDF、JPS、MINCO S3、独立 yaw、footprint gate、
Local Collision Repair 或全向 SE(2) MPC。控制保持车体系 `[vx,vy,wz]`，禁止差速、Ackermann、
ICR 或 `vy=0`。

## 2. 当前准入结论

| 阶段 | 已完成 | 当前缺口 | 状态 |
| --- | --- | --- | --- |
| P2 | ROGMap/adapter 数值链、唯一 planning owner、immutable snapshot、fail-stop、直线 Gazebo 运行 | freshness、最终 nominal/red-box、当前 revision fault matrix、clearance/contact、RViz 滑窗验收 | **未通过** |
| P3 | ATS action、feedback、cancel/preempt/timeout、Goal Manager watchdog、Gazebo 默认无 Nav2 启动路径 | 完整 action 生命周期、扩大路线、red-box、无 Nav2 server graph 运行证据 | **未通过** |
| P4 | 四舵轮仿真、速度 owner、矩形 footprint、自适应 sampled sweep | 连续 swept 误差上界、独立 contact evaluator、制动/延迟、HIL、实车 | **未通过** |
| QP | OSQP v1.0.0、固定 CSC、warm-start ABI、same-snapshot shadow、数值防御 | 真实 map/collision gate、稳定 solved、paired runtime、主链 fallback 和切换准入 | **仅 Shadow** |

## 3. 最新有效证据

- P0 已通过：`dependencies.repos` 将不存在的 `rmoss_gz_resources@main` 修正为可验证的
  `humble`，并将曾经传输不稳定的 `ats_mujoco_sim` 和 `teleop_gimbal_keyboard` 切换到可达的
  用户 SSH URL；`dependencies.lock.repos` 由最终干净目录的 `vcs export --exact -n` 生成，锁定
  22 个实际 checkout SHA。
- 最终干净目录 `/tmp/ats_p0_repro_final.6xNq8i` 的 `vcs import` 于 `152.2 s`、`rc=0` 完成；
  Gazebo fork `a28ccd20428ffc4bdd7fbbc22fee884fa1db72eb` 还修复了 CMake 引用 ignored 测试源的
  clean-build 缺陷。该目录的最窄 Gazebo 依赖闭包构建 17 包通过，Gazebo CTest `4/4` 和
  `ats_gazebo_nav.launch.py --show-args` 通过。该证据只覆盖复建和资源解析，不覆盖运行期导航。
- 独立目录 `/tmp/ats_p0_remote_final.Lgyalq` 又以 SSH 对 root `origin/develop` 做 depth-1 clone，
  得到 `f2d049cbf245b6fdfbad0d4870e53dc3ab09cbeb` 后直接导入 exact lock；全流程
  `250.2 s`、`rc=0`，22 个依赖均从远端 checkout。至此 root 传输、manifest、锁定与最窄构建具有
  相互独立的复建证据。
- 两份 RViz 已配置全局 `/rc_esdf/signed_distance_grid` 和局部 ROGMap debug，但当前 revision 尚无
  全局 ESDF/三米滑窗运行截图；
- MINCO 已有 geometry preprocessor、curvature-aware time allocation、ESDF refinement 和 quality
  telemetry，聚焦 CTest 已通过；
- Gazebo domain `228/229` 的短直线 action 成功，终点误差约 `0.060/0.045 m`；
- domain `230` 的直线 candidate 长度比 `1.000`、曲率为零，但 `/localization` wall interval
  `p50/p95/p99=0.371/0.994/1.612 s`，adapter 反复 `ready=false`，action fail-closed；
- P1 的单 recorder 已补齐 `/clock`、三段 odometry、`/localization/status` 与 adapter 的 wall/stamp/age、
  duplicate/backward、RTF、TRACKING、TF lookup、本 session 进程 telemetry 与 recorder callback duration
  字段；callback 仅量化观测器自身开销，不代表上游 executor 或 DDS queue。新统计的 focused CTest、
  Release build、runner/launch 静态检查通过；完整包 CTest 的 `ament_black` 仍受 sandbox 禁止本地 socket
  所限，尚未取得主机复跑结果。DDS queue/drop 计数明确为 `unverified_no_portable_rmw_counter`，不能解释为
  零丢包；
- 根仓 runner 新增纯函数 `scripts/gazebo_freshness_classifier.sh` 和确定性回归，按
  `/clock -> /lidar_odometry -> /odometry -> /localization -> status` 顺序输出
  `p1_first_freshness_violation`；只在完整 P1 条件满足时置 `p1_admission_evidence=true`，不改变
  timeout、lease、QoS 或控制行为。
- **已验证（历史诊断运行，2026-08-19）**：合法 domain `220` headless 30 s 已运行完整 Gazebo
  链。`/clock` p99/max=`0.122538/0.135193 s`，`/lidar_odometry`=`1.671385/1.671385 s`，
  `/odometry`=`1.659046/1.659046 s`，`/localization`=`1.663433/1.663433 s`；RTF p50/p95/p99=
  `0.199802/0.409690/0.596497`，status `178/119`（TRACKING/non-TRACKING），TF failure `18/300`，
  action 未成功。分类器把 `/lidar_odometry` 标为首个可见 timing 违反者；这不是 P1/P2、性能或安全通过证据。
- **已验证（P1 baseline，2026-08-19）**：新 domain `224` headless 的实际 recorder 窗口为
  `60.003753 s` 并正常完成，action 成功、终点误差 `0.146 m`、路径/reference/MPC/底盘链均有输出，
  terminal `emergency_stop=true` 且两级速度为零。`/rc_esdf/planning_grid`、`/cmd_vel_mpc`、
  `/motion_control` 和 chassis command 都观测为单一 publisher。
- **未通过（P1 freshness）**：`/lidar_odometry` p99/max wall interval=`1.144617/1.488598 s`，
  `/odometry`=`1.143652/1.490561 s`、`/localization`=`1.142361/1.490312 s`；RTF p50/p95/p99=
  `0.331409/0.502033/0.560295`，status `TRACKING/non-TRACKING=535/56`，TF failure=`16/600`。
  分类器将 `/lidar_odometry` 识别为首个可见违反者，`p1_admission_evidence=false`。该记录确立了
  freshness 缺口，不能标记 P1/P2、性能或安全通过。
- **已验证（最终 revision P1 baseline，2026-08-19）**：domain `225`、显式
  `ENABLE_CAMERA_SENSORS=false` 的 recorder 正常完成 `60.010472 s`，启动前无残留进程，planning grid、
  `/cmd_vel_mpc`、`/motion_control` 与 chassis command 均为单一 active publisher，终态
  `emergency_stop=true`、两级速度为零。action 被接受，但 90 s 内没有终态，runner 返回
  `nominal action did not succeed`。
- **未通过（最终 P1 freshness）**：`/lidar_odometry` p99/max=`2.759540/2.942285 s`，下游
  `/odometry`=`2.764472/2.946450 s`、`/localization`=`2.764479/2.944695 s`；RTF p50/p95/p99=
  `0.303726/0.803858/1.017098`，status `419/158`（TRACKING/non-TRACKING），TF failure `27/600`。
  该结果与 domain `224` 的 action 成功但 freshness 不通过共同表明 P1 action 尚不具备重复性；不得把
  任一单次运行标记为 P1/P2、性能或安全通过。
- **推断 [Confidence: Medium]**：`loam_interface` 只在 `cloud_registered` callback 中发布
  `/lidar_odometry`；其 ROS stamp p99 为 `0.299990 s`，而 wall p99 为 `1.144617 s`，下游两段保持同量级，
  callback p99 为微秒级。因此优先调查 Gazebo 传感器/RTF、Point-LIO publisher cadence 与 DDS 丢包；
  尚不能把行为 owner 归因到其中任一单独组件。DDS counter 仍为
  `unverified_no_portable_rmw_counter`。
- **已验证（raw LiDAR 分层与 A/B，2026-08-20）**：最终 revision 的 recorder 已在新 domain 实际记录
  `/<robot>/livox/lidar -> /livox/lidar -> /cloud_registered -> /lidar_odometry`。domain `215` 的 `4 ms`
  physics candidate（默认 `10 Hz / 625 x 32`）raw/lidar-odometry/localization wall p99 为
  `1.610141/1.435872/1.430092 s`，action unsafe ABORTED；domain `214` 的 `5 Hz / 625 x 32` 保持
  SDF、bridge offset 与 Point-LIO 三处 `0.2 s` 周期一致，但 p99 恶化为
  `3.666797/3.312480/3.308619 s`；两者均不成为默认。domain `213` 仅移除 headless GUI state 的
  `SceneBroadcaster`，保留默认 physics 和 `10 Hz / 625 x 32`，但 p99 仍为
  `1.409273/1.322408/1.319348 s`，status `TRACKING/non-TRACKING=448/125`、TF failure=`35/601`，
  action unsafe ABORTED。三个 artifact 都完整 observer `>=60 s`，均为 `freshness_lidar_odometry`，
  不能标记 P1/P2、性能或安全通过。
- **已验证（时间契约与 runner）**：`LIVOX_UPDATE_RATE_HZ` 现在同步驱动 Gazebo SDF update rate、C++ bridge
  `scan_period_sec` 与 Point-LIO `mapping.lidar_time_inte`，默认仍为 `10.0 Hz`；`WORLD_SDF_PATH` 只在非空
  时转发，避免空 launch 参数阻断默认 world。`rmu_gazebo_simulator` 在本轮为 `32 tests, 0 errors,
  0 failures`，runner contract 和 freshness classifier 均通过。
- **推断 [Confidence: Medium]**：raw sensor、bridge、Point-LIO output 与 loam output 的 wall gap 仍同阶，
  而各 recorder callback p99 均为微秒级。现有证据否定了三项候选的收益，但仍不能在 Gazebo sensor publisher
  调度与 DDS 接收之间指定唯一 owner；下一步只应补这两者的独立计数，不得放宽 freshness、安全或动作门限。
- **未通过（最新 P1 默认正式基线，2026-08-20）**：全新合法 domain `208` 使用默认
  `10 Hz / 625 x 32`、headless off-screen rendering、关闭相机、关闭 Transport 诊断订阅、关闭 Direct
  bridge、generic bridge `RELIABLE/KeepLast(10)`、`planning_grid_owner=rog_map`。完整 recorder 正常完成
  `60.016403 s`，启动前和结束后均无导航/仿真残留进程；`/clock` wall p99/max 为
  `0.078741/0.351209 s`，而 `/<robot>/livox/lidar`、`/livox/lidar`、`/cloud_registered`、
  `/lidar_odometry`、`/localization` 的 wall p99 依次为
  `2.954985/2.958803/3.327495/3.327198/3.324217 s`。分类器仍输出
  `first_violation=lidar_odometry`，status `TRACKING/non-TRACKING=362/210`，TF lookup failure 为
  `35/600`，故 `p1_admission_evidence=false`。
- **已验证（同一 domain 的安全收尾）**：domain `208` 曾观察到 JPS/reference/MPC、单一
  planning grid、`/cmd_vel_mpc`、`/motion_control` 与 chassis command owner 及非零轮速；但 action 最终
  `ABORTED`，最终位置误差 `3.1606 m`，终态 `emergency_stop=true`，两级速度均为零。地图持续因新鲜度
  失效而拒绝规划，这证明 fail-closed 仍生效，绝不构成活跃导航成功或物理接触为零的证据。
- **推断 [Confidence: Medium]**：在 domain `208` 中，raw ROS PointCloud2 已先于 Point-LIO/Loam 下游
  输出失去 cadence，故当前可观测边界收敛到 Gazebo sensor/Transport 与 generic `ros_gz_bridge` 的
  GZ-to-ROS 输出之间。此结论不能唯一归因 generic bridge：domain `212` 的 Transport 诊断曾显示
  Transport 端健康，但诊断订阅本身会改变该边界；下一台性能更高的机器必须以全新 domain 重跑默认基线，
  再用不增加长期 PointCloudPacked Transport subscriber 的计数或 trace 分开 publisher 慢与 ROS/DDS
  接收缺口。
- P1 runner 在 ROS graph 创建前检查新 ROS domain 的合法范围和残留导航/仿真进程，并把 candidate domain
  与仓库 SHA 写入 raw artifact；它不以主机资源统计决定是否启动。无故障运行中，只有至少 60 s observer、
  无 freshness 首违、status 全部 TRACKING、无 TF lookup failure 且 straight action 成功，才写入
  `p1_admission_evidence=true`。
- domain `233` 的尝试首先触发 Fast DDS 合法 domain 上限（`Calculated port number is too high`），不能作为
  Gazebo/导航运行证据。合法 domain `231` 曾启动 Gazebo 和导航链，health gate 通过并观察到
  JPS/MINCO/MPC/底盘非零动作；但 `/clock` RTF p50/p95=`0.2518/0.4815`、`/localization` wall interval
  p50/p95/p99=`0.484/1.506/2.185 s`，status `TRACKING/non-TRACKING=189/111`，TF lookup failure=`17/300`，
  action 未成功并最终 fail-closed。该历史结果不足以确定首个行为 owner，也不是 P1/P2 通过证据；
- production MINCO node 尚未把实时 `InitialKinematicState` 传入 optimizer；几何质量指标主要用于
  telemetry，尚未形成完整候选接受门禁；
- Gazebo runner 的 `TEST_PROFILE` 尚未拥有实际 corner/S/narrow/red-box 场景逻辑；
- QP node 仍固定 `map_fresh=false`、`collision_free=false`，`solver_mode=qp` 显式拒绝。

组件行为由源码与聚焦测试支持；domain 数值来自已保存运行记录。freshness 的唯一根因仍未确定，必须
逐级测量 `/lidar_odometry -> /odometry -> /localization -> status -> adapter`，不能仅凭相关性归因
Point-LIO、DDS、仿真 RTF 或 CPU 争用中的任一项。

## 4. 不可放宽的安全边界

- unknown、occupied、outside-map、ESDF sign/gradient、snapshot freshness 和 lease 继续 fail-closed；
- `/rog_map/esdf` 只是调试点云，不能作为数值规划输入；
- `/cmd_vel_mpc` 和 `/motion_control` 必须各自保持唯一发布者；
- 地图、定位、TF、reference、ExecutionCommand、gimbal 任一不健康都不得继续运动；
- 急停清空 tracker，恢复后旧 reference 不得复活；
- 不能提高 freshness timeout、QP iteration/deadline/residual 来绕过失败；
- planner collision 为零不能推出 Gazebo physical contact 或实车碰撞为零；
- 未在目标机测量前不得引用报告中的 `50 Hz`、`6 ms` 或内存数据。

## 5. 下一优化顺序

1. 在性能更高的目标机以新合法 ROS domain、固定 revision 首先重跑关闭诊断 observer、关闭 Direct bridge、
   generic `RELIABLE/KeepLast(10)` 的 `60 s` 默认 P1 基线；随后补 Gazebo LiDAR publisher 与 DDS subscriber
   的独立计数，验证其与 Point-LIO/loam cadence 的边界。已拒绝的 `4 ms`、`5 Hz`、无 SceneBroadcaster、
   Direct bridge 与 `BEST_EFFORT/KeepLast(1)` candidate 均不得成为默认或重复用于通过声明；
2. 本轮首违分类器已把 `/lidar_odometry` 标为首个可见边界，但不得根据单次 domain `224` 运行直接
   修改 timeout 或指定唯一算法 owner；
3. 将实际运动状态接入 MINCO 四条生产优化路径；
4. 把几何质量 telemetry 升级为按路径类别生效的候选门禁；
5. 实现真实 Gazebo straight/corner/S/narrow/nominal/red-box runner；
6. 重跑当前 revision 的 P2 名义、边界和故障矩阵；
7. 完成 RViz 全局/局部滑窗、clearance、continuous swept 和 contact；
8. 完成 P3 Nav2-free action 生命周期；
9. 完成长时间性能、MuJoCo 跨后端和 HIL；
10. P2/P3/P4 通过后再推进 QP 主链和低速实车。

详细任务、DoD、验证命令和停止条件见：

- [ATS 导航剩余优化总 TODO](项目优化文档/ATS导航剩余优化总TODO.md)
- [下一阶段新对话提示词](项目优化文档/下一阶段提示词_ATS导航仿真闭环与准入.md)
- [LTV-QP 后端准入记录](ats_swerve_mpc_ltv_qp_backend_admission.md)

## 6. 文档职责

- 本页：只记录当前阶段结论和架构边界；
- 总 TODO：只记录未完成任务、依赖、DoD 和状态，不累计完整日志；
- QP backend admission：记录后端来源、数值准入和 QP runtime 边界；
- `log/` artifact：保存原始运行结果，不提交大体积生成物；
- Git 历史：保留已退役阶段文档和过去实验的可追溯性。

任何新源码行为修改后，必须用该 revision 重跑相应仿真，不得复用修改前结果作为最终证据。
