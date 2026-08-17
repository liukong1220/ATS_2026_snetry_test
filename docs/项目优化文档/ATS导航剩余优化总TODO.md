# ATS 导航剩余优化总 TODO

> 状态：唯一活动导航优化清单
> 更新时间：2026-08-15
> 适用范围：Gazebo、MuJoCo、HIL 与实车共用的 ATS 四驱四转哨兵导航链
> 历史说明：旧阶段 TODO 已退役；历史实现与运行证据通过 Git 历史、
> `docs/ats_swerve_mpc_ltv_qp_backend_admission.md` 和状态文档追溯。

## 1. 目标与完成定义

本清单用于收敛当前仍未完成的生产能力和准入证据。它不把组件测试、单次仿真成功、HIL 和实车通过
混为同一状态。最终目标链保持不变：

```text
传感器 + 独立状态估计
-> ROGMap 概率占据/膨胀/3D ESDF
-> 地面投影与 2.5D 可通行语义
-> RC-ESDF 规划接口
-> ATS Goal Manager -> JPS -> MINCO S3 + 独立 yaw
-> footprint safety + Local Collision Repair
-> 全向 SE(2) MPC -> 四驱四转底盘
```

总体验收必须同时满足：

- 干净主机可从远端仓库复建全部依赖和仿真资源；
- 地图、定位、规划、控制和底盘命令各有唯一 owner；
- nominal、边界场景和故障恢复均有独立 ROS domain 的证据；
- stale、unknown、无路、unsafe、solver failure 或 lease failure 都确定性零速度；
- P2、P3、P4 和 QP 主链分别通过自己的门禁，不相互替代；
- HIL 前完成 Gate 0--2，实车前完成 Gate 0--3；
- 性能结论来自固定 revision、配置、硬件和原始 artifact。

## 2. 当前冻结基线

### 2.1 仓库基线

| 仓库 | P0 起始 revision | 远端状态 | 说明 |
| --- | --- | --- | --- |
| 根仓 | `c7cc0e54cc7d` | `origin/develop` 已同步 | P0 阻塞记录后的文档基线 |
| 导航仓 | `5ea786eb2e70` | `origin/develop` 已同步 | ROGMap、JPS/MINCO、Goal Manager、MPC |
| Gazebo 用户 fork | `a28ccd20428f` | `origin/main` 已同步 | clean checkout 的运动学测试源已固化；禁止向 `upstream` 写入 |
| MuJoCo | `e3d6ea7a5e61` | `origin/develop` 已同步 | 当前轮未修改 |
| `ats_robot_description` | `dea591e53fa0` | `origin/develop` 已同步 | 该提交已于 2026-08-15 经 SSH 推送；干净 Git/vcs 复建仍受本机传输失败阻塞 |

受保护的用户内容继续保留：

- `src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py`；
- `src/ats_sentry_nav/ats_swerve_mpc/求解器.md`；
- Gazebo fork `scripts/ats_bridge/gz_livox_bridge.py`。

不得读取其内容作为设计依据，不得删除、覆盖、暂存或提交。

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
- 旧 revision 的八个 Gazebo fault domain 曾通过，但 MINCO 行为修改后必须重跑；
- 历史 `qp_shadow` 仍以 `max_iterations`、零 feasible 和零 warm-start 为主，不能启用 QP 主链。

## 3. 不可破坏的契约

- Point-LIO 继续拥有 `/localization` 和 `/registered_scan` 的状态估计输入链；
- ROGMap 不是定位器，不得用 ground truth 替换正式定位；
- adapter 只能消费 ROGMap 数值 projection，不得反解析 `/rog_map/esdf`；
- unknown、occupied、outside-map、signed-distance 正负号和 gradient 语义不得放宽；
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 SE(2) MPC 必须保留；
- 四舵轮控制保持车体系 `[vx,vy,wz]`，禁止差速、Ackermann、ICR 或 `vy=0`；
- `/cmd_vel_mpc` 和 `/motion_control` 必须各自只有一个发布 owner；
- 急停必须清空 tracker，急停前 reference 不得在恢复后复活；
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
          -> P7 P4 仿真安全与 HIL

P1/P4 通过 -> QP-2 Shadow 可配对复核 -> QP-3 受控主链切换
所有分支 -> 长时间稳定性 -> HIL -> 低速实车
```

禁止跳过 P0/P1 直接调 MINCO 或 QP；污染环境中的 timing 只能保存为无效样本。

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
- 完整编译又揭示 Gazebo `CMakeLists.txt` 无条件引用被忽略、未跟踪的运动学测试源。该问题在用户
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

### 停止条件

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

- [x] 单一 C++ recorder 同时订阅 `/clock`、`/lidar_odometry`、`/odometry`、`/localization`
  和 `/localization/status`；
- [x] 每级记录 steady wall arrival、ROS stamp interval、`/clock` 相对 stamp age、重复/倒退 stamp、
  消息数和最大 gap；
- [x] 同一 recorder 记录其对 `/clock`、三段 odometry、`/localization/status` 和 adapter status 的
  callback 执行时长分布；它只量化观测器自身开销，不可替代行为 owner 的 executor/queue trace；
- [x] 记录 `/clock` wall interval、sim-time interval 与 RTF 分位数；
- [x] 以 `map -> gimbal_yaw_odom` 的实际零超时查询记录 TF lookup attempt/success/failure/max duration；
- [x] runner 只对本 launch session 内的 bridge、Point-LIO、loam、sensor generation、fusion、ROGMap
  与 adapter 写入 CPU tick、RSS、线程和 voluntary/nonvoluntary context-switch 两次原始快照；
- [x] DDS queue/drop 无可移植 RMW counter 时显式写入
  `unverified_no_portable_rmw_counter`，不得当作零丢包；
- [ ] 区分 publisher 慢、subscriber 丢包、sim 慢和 wall watchdog 四类根因；
- [ ] A/B 每次只改变一个因素：headless、recorder、RViz、相机、LiDAR profile、日志；
- [ ] 所有 profile 使用新 domain、相同 revision、相同起点和固定窗口。

### 2026-08-15 P1 插桩与 preflight

- **已验证（组件）**：Gazebo fork 的 `EvidenceStatistics` 确定性 CTest、`rmu_gazebo_simulator`
  单 worker Release build、完整包级 CTest（`32 tests, 0 errors, 0 failures`）、runner `bash -n`、
  当前 revision `ats_gazebo_nav.launch.py --show-args` 和五仓 `git diff --check` 均通过。
- **已验证（组件，2026-08-17）**：recorder 的 callback duration 统计已进入每级 timing 输出与
  runner artifact；单 worker `rmu_gazebo_simulator` Release build、focused
  `test_evidence_statistics`、runner `bash -n`、launch Python 编译、`--show-args` 与相关 diff check
  均通过。当前 sandbox 下完整包 CTest 的 `ament_black` 因禁止 Python `SyncManager` 创建本地 socket
  而失败（`33 tests, 1 error, 1 failure`）；该限制不是本轮 C++ 测试失败，仍待可执行的主机环境复跑。
- **已实现未运行（闭环）**：新 recorder/runner 现已输出上述链路、状态、RTF、TF 与资源字段；尚未在
  新 ROS domain 启动，不存在新的 timing 分布、action、owner、终点、两级零速度或 P2 结论。callback
  duration 只用于证明 recorder 的观测开销，DDS queue/drop 与其他节点 callback blocking 仍未验证。
- **已验证（停止）**：运行前 artifact
  `log/gazebo_minco_mpc_chain/20260815_2150_stage1_preflight_resource_stop/preflight_resource_stop.txt`
  记录 first violation 为 `swap_used=5.7 GiB`。审计未发现残留导航/Gazebo/MuJoCo 进程，但该高 swap
  已满足停止条件，故未分配 ROS domain、未启动 Gazebo。
- **已验证（资源门，2026-08-17）**：runner 在任何 ROS graph 或 Gazebo 进程创建前执行正式
  `P1_RESOURCE_MODE=admission` 资源门，默认 `P1_MAX_SWAP_USED_GIB=4.0 GiB`；artifact 同时保存
  `SwapTotal/SwapFree/MemAvailable`、候选 domain、仓库 SHA 和按进程名匹配的残留导航/仿真进程。
  `P1_RESOURCE_PREFLIGHT_ONLY=true` 不会启动 ROS 或 Gazebo。候选 domain `313` 的 raw artifact
  `log/gazebo_minco_mpc_chain/20260817_114122_nominal_none_domain313/preflight_resource.txt`
  记录 `swap_used=5.158 GiB > 4.0 GiB`、`ros_domain=not_allocated`、无残留进程，runner 返回 `3`。
- **已实现（探索资源模式，尚未形成闭环证据）**：显式 `P1_RESOURCE_MODE=exploratory` 允许开发者
  在 swap 超过正式阈值时继续启动观察，但只将资源质量标为 `degraded`，并固定写入
  `p1_admission_evidence=false`、`timing_valid_for_admission=false`。该模式仍拒绝非法模式、缺失
  `/proc` 字段和残留导航/仿真进程，且不改变任何 TF、planning owner、unknown、lease、急停、零速度
  或 action 安全门。探索结果只能用于算法调试，不能写成 P1/P2、性能、实时性或安全通过。
- **已验证（探索资源门回归）**：`scripts/test_gazebo_resource_gate.sh` 覆盖非法模式、正式模式 swap
  超限 fail-closed、探索模式 degraded 标记和 `P1_RESOURCE_PREFLIGHT_ONLY` 不分配 ROS domain；
  该回归不启动 ROS 或 Gazebo。
- **已验证（当前主机探索 preflight，2026-08-17）**：domain `324` 以
  `P1_RESOURCE_MODE=exploratory P1_RESOURCE_PREFLIGHT_ONLY=true` 返回 `0`，实测
  `swap_used=5.333 GiB`、`MemAvailable=1.836 GiB`，artifact 标记
  `resource_quality=degraded`、`p1_admission_evidence=false`、`timing_valid_for_admission=false`、
  `ros_domain=not_allocated`。未启动 ROS/Gazebo，不能作为 P1 runtime 或性能证据。
- **推断 [Confidence: Medium]**：旧 domain `230` 三个下游 topic 的相近 wall gap 可能共同受上游 cadence、
  仿真 RTF 或资源争用影响；新增观测尚未运行，不能归因任何行为 owner。
- **未验证**：DDS 中间件可报告的队列/丢包计数、各进程 CPU rate/context-switch delta、`/clock` 与各级
  stamp/age 分布、持续 TRACKING、TF failure、first violating publisher/subscriber 以及所有 A/B 因子。

### 修复原则

- 只修改最早违反 freshness 的行为 owner；
- 不提高 `odom_timeout_s`、localization timeout、adapter lease 或 map timeout；
- 不把 ground truth 接入正式 `/localization`；
- 不用 ros2cli 高频 observer 干扰被测链，优先单一 C++ recorder；
- 降低传感器负载必须有感知质量与闭环指标共同批准。

### DoD

- [ ] 低负载 headless 连续至少 `60 s`，`/localization` p99 interval `< 0.25 s`；
- [ ] 同一窗口不存在 `> 0.5 s` 的 localization gap；
- [ ] status 持续 TRACKING，adapter 不因 localization 反复 `ready=false`；
- [ ] stamp 不倒退，sensor-to-localization age 有 p50/p95/p99；
- [ ] straight action 两次成功，终点误差、owner 和收尾零速均通过；
- [ ] 修复有聚焦单测或 deterministic fault test。

### 停止条件

- 正式模式下 `P1_MAX_SWAP_USED_GIB=4.0` 的高 swap、低可用内存、持续 CPU 饱和或残留导航进程；
- 探索模式不得把 degraded 运行写成正式 P1/P2/性能证据，且仍须在 Gazebo z 发散、RTF 异常、关键
  telemetry 缺失或系统失稳时停止；
- Gazebo z 发散、RTF 异常、TF 冲突或多个 localization publisher；
- 需要放宽安全 timeout 才能通过；
- 无法区分上游发布慢和下游丢包。

## 7. P2：补齐 MINCO 生产契约

### 7.1 当前运动状态接入

- [ ] `MincoPlannerNode` 获取与 goal/snapshot 同一 localization epoch 的新鲜状态；
- [ ] 明确 twist frame、单位和时间，不假定速度已经是世界系；
- [ ] 将车体系速度正确旋转到规划世界系；
- [ ] 无可靠加速度时只播种速度，加速度保持零；
- [ ] stale、epoch 不匹配、非 finite 或 TF 失败时不用该状态；
- [ ] center、footprint、fallback、repair 使用同一个冻结初始状态；
- [ ] telemetry 记录原值、裁剪值、stamp age 和拒绝原因。

测试至少覆盖：

- [ ] 非零 yaw 下横移速度转换；
- [ ] 速度/加速度 finite 与上限裁剪；
- [ ] localization epoch 变化拒绝旧状态；
- [ ] 四条 optimizer 调用路径不再无条件传 `nullptr`；
- [ ] 重规划首端速度连续，终端速度/加速度仍为零。

### 7.2 把质量 telemetry 升级为生产门禁

质量门禁必须区分直线与一般曲线，不能用起终点直线偏差拒绝合法 S 弯。

- [ ] 直线类限制 length ratio、横向偏差、曲率峰值/TV 和符号变化；
- [ ] 一般曲线相对 preprocessed guide/baseline 比较长度、偏差、曲率 TV 和净空；
- [ ] 所有类检查 v/a/j、时间单调、footprint/swept collision 和 snapshot freshness；
- [ ] ESDF candidate 同时满足净空不下降、碰撞不增加、长度和曲率变化不过门；
- [ ] 阈值进入唯一实际加载配置，并有参数范围校验；
- [ ] 记录结构化首个拒绝原因；
- [ ] quality 失败只回退到同 snapshot 上安全的 baseline；
- [ ] baseline 也失败时不发布 reference，保持急停与两级零速度。

### 7.3 净空与连续性

- [ ] center clearance 与 oriented-footprint clearance 分开记录；
- [ ] ESDF backtracking 使用同一 immutable snapshot；
- [ ] unknown、outside、非 finite gradient 和 snapshot 变化立即拒绝；
- [ ] noisy gradient 不产生交替法向偏移；
- [ ] 单拐角不会让无关直线段一起减速；
- [ ] 非零 initial-state 时重新检查首段连续曲线净空和动态极值；
- [ ] repair 输出重新求解 MINCO、yaw、时间和完整安全门。

### DoD

- [ ] 库级、node 级和旧 reference 竞态测试通过；
- [ ] 直线、冗余共线、短首尾段、单角、S/U 弯、窄通道、noisy ESDF fixture 通过；
- [ ] 门禁能拒绝“finite 但无意义多弯”的候选；
- [ ] initial-state 在实际 node 路径生效；
- [ ] 不改变 JPS、MINCO S3、独立 yaw、地图和速度 owner。

## 8. P3：实现 Gazebo 确定性场景 runner

当前 `TEST_PROFILE` 只用于日志命名，必须升级为实际行为 owner。

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
- [ ] 场景不得依赖人工 RViz 点击；
- [ ] 每个 goal 独立记录 accepted/result/cancel/preempt/timeout；
- [ ] 保存 raw/preprocessed/refined/reference/predicted/executed；
- [ ] 保存地图 identity、owner、clearance、collision/contact、v/a/j 和 terminal error；
- [ ] 未验证 contact/clearance 不写默认通过值；
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

每次必须记录：

- terminal pose、位置/yaw 误差、总耗时；
- JPS/MINCO/MPC/executed 点数和五层 payload；
- length ratio、横向偏差、曲率 max/p95/TV/符号变化；
- v/a/j peak/p95、segment duration、time-scaling 次数；
- center/footprint minimum clearance；
- discrete/swept collision samples；
- replan、fallback、repair、失败和恢复次数；
- localization/map/reference/command age；
- planning grid、`/cmd_vel_mpc`、`/motion_control` 唯一 owner；
- Gazebo contact evaluator 结果或明确 `unverified`。

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
-> /cmd_vel_mpc=0
-> /motion_control=0
-> old reference cannot revive
```

恢复必须满足 generation/sequence 继续推进，且只有新目标或新 request identity 才恢复运动。

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

在上述完成前只能写“Nav2-free 代码路径存在”，不能写“P3 已通过”。

## 12. P7：P4 仿真安全与 HIL 前置

### 仿真阶段

- [ ] 将自适应 sampled sweep 升级为具有明确误差上界的连续 swept 契约；
- [ ] 覆盖纯旋转、横移、对角、`+pi/-pi`、高曲率和 map 边界；
- [ ] 增加独立 Gazebo physical contact evaluator；
- [ ] 区分 planner collision、Gazebo contact 和 near-miss；
- [ ] 测量急停到 `/motion_control=0`、wheel speed=0 的延迟；
- [ ] 测量制动距离并纳入动态安全 margin；
- [ ] 注入 saturation、steer/wheel rate、命令延迟和丢包；
- [ ] 验证轮速过零、正反/横纵切换和舵角机械限位；
- [ ] 长时间运行无 queue/RSS/thread/generation 异常增长。

### HIL Gate 3

- [ ] 抬轮或断开动力负载；
- [ ] 四个 steer/drive 模块逐轴验证符号和单位；
- [ ] 物理、遥控、软件急停和断电路径分别演练；
- [ ] 停止 producer、断网、断传感器、杀进程并验证硬件 watchdog；
- [ ] 测量 sensor-to-command、command-to-actuator p50/p95/p99；
- [ ] 监控电流、电压、温度、CAN 和 actuator fault；
- [ ] 形成回滚 commit 和安全观察员清单。

## 13. QP：Shadow 到受控主链

### QP-2 真实 Shadow

- [ ] 接入与 iLQR 同一 snapshot 的真实 map-health 和 footprint/collision producer；
- [ ] 仅在真实输入成立后移除临时 `map_fresh=false`、`collision_free=false`；
- [ ] freshness/P2 通过后运行 paired A/B/C；
- [ ] identity digest 不可比时 analyzer 输出 `not_comparable`；
- [ ] 定位 OSQP `max_iterations` 的矩阵尺度、conditioning、active bounds 和 warm-start；
- [ ] 禁止提高 iteration、放宽 residual/time limit 或接受 `solved_inaccurate`；
- [ ] 得到稳定 `solved`、residual、hard margin、slack 和 warm-start 分布；
- [ ] 记录 complete phase、full callback、CPU/allocation p50/p95/p99；
- [ ] 覆盖 nominal、yaw jump、速度阶跃、反向、横移、轮速过零和 fault matrix。

### QP-3 受控主链

- [ ] candidate 全部 hard-check 通过时才能发布 `controls.front()`；
- [ ] infeasible、deadline、residual、slack、map、collision、lease 失败均零速度；
- [ ] iLQR 保留为可选 baseline；
- [ ] fallback 必须限定窗口并重新验证旧序列；
- [ ] 不得无条件沿用 `last_control`；
- [ ] 节点级覆盖 solved/infeasible/time-limit/solved-inaccurate/residual reject；
- [ ] MuJoCo 完整通过后才允许 HIL QP；
- [ ] 默认切换需要单独评审和回滚点。

## 14. 长时间稳定性与性能

- [ ] 固定 `Release/-O3`、线程、CPU governor、middleware 和 power mode；
- [ ] 关键 profile 预热后至少运行 `10 min`，另做 `30--60 min` soak；
- [ ] 记录 RTF、CPU/RSS/swap、threads、context switches、DDS drops、queue depth；
- [ ] 记录 localization、projection、adapter、MINCO、iLQR/QP、callback p50/p95/p99/max；
- [ ] 记录 control jitter、deadline miss、command age 和 tracking error；
- [ ] 检查 generation、sequence、goal ID 和 telemetry ring 是否倒退或增长；
- [ ] RViz、INFO logging、recorder 分别做 A/B/C 消融；
- [ ] 性能结论附硬件、revision、配置、样本数和 artifact；
- [ ] 未在目标机测量前禁止引用 `50 Hz`、`6 ms` 或报告内存数值。

## 15. 实车前 Gate 0--4

### Gate 0：静态与数学

- [ ] frame/time/QoS/map/ESDF/generation/ownership 账本完整；
- [ ] 状态、矩阵、参数和 finite 检查通过；
- [ ] 速度、加速度、jerk、轮速、舵角速率和机械限位有物理来源；
- [ ] 启动、部分初始化、reset 和 shutdown 都输出安全状态。

### Gate 1：离线/回放

- [ ] nominal、边界、stale、乱序、clock jump、NaN、overload 回放；
- [ ] 与冻结 iLQR/JPS-MINCO baseline 成对比较；
- [ ] 关键故障在 deadline 内零速度。

### Gate 2：仿真

- [ ] P2/P3/P4 仿真条目通过；
- [ ] Gazebo 与 MuJoCo 结论一致或差异已解释；
- [ ] contact、clearance、tracking、deadline 和 recovery 无未知项。

### Gate 3：HIL

- [ ] 抬轮 HIL、硬件 watchdog、急停、符号、延迟和热稳定通过。

### Gate 4：低速实车

- [ ] 独立物理急停与安全观察员就位；
- [ ] 先直线停止，再横移、原地旋转、单拐角；
- [ ] 再进入窄通道、重规划和边界场景；
- [ ] 每次只提升一个能量或复杂度维度；
- [ ] 异常立即停止并回滚冻结 revision。

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

仿真必须使用新 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1`、`ROS2CLI_DAEMON=false`、headless
性能基线、`planning_grid_owner=rog_map` 和 `solver_mode=ilqr`（QP-3 前）。

## 17. Artifact 最小字段

- 仓库 SHA、dirty state、effective params、world/map、domain、时间；
- 目标序列/yaw、action feedback/result；
- topic type/frame/QoS 和 ownership；
- localization/map/reference/command age；
- source generation、adapter publication、MINCO snapshot identity；
- raw/preprocessed/refined/reference/predicted/executed payload；
- clearance、collision/contact、v/a/j、tracking 和 terminal error；
- CPU/RSS/swap/RTF、p50/p95/p99/max 和 deadline misses；
- first failure、stop/recovery、旧 reference 拒绝和最终零速度；
- 未执行项和不能得出的结论。

## 18. 提交与文档规则

- 每轮开始与结束检查根、导航、Gazebo、MuJoCo 和机器人描述仓；
- 禁止 `git add .`、`git add -A`、force push、历史重写和破坏性恢复；
- 只显式 stage 本轮文件；
- 根、导航、MuJoCo 分别提交自己的 `develop`；
- Gazebo 只推送用户 `origin/main`，不得向 `upstream` 写入；
- 无修改仓库不得制造空提交；
- 作者固定为 `liukong1220 <1625038134@qq.com>`；
- 本清单只更新状态，不再次追加完整运行流水账；
- 原始结果放 artifact，准入结论放状态文档，QP 证据放 backend admission。

## 19. 下一阶段执行顺序

1. P0 推送并复建机器人描述；
2. P1 定位并修复 Gazebo localization freshness；
3. 两次 straight 通过后，执行 P2 initial-state 与真实质量门禁；
4. 完成 P3 确定性 profile runner；
5. 再进入 P4 场景和故障矩阵。

P1 未通过时不得继续 MINCO 参数搜索、QP 主链或性能结论。
