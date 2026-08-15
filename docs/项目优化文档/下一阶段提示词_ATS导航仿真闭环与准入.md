# 下一阶段新对话提示词：ATS 导航仿真闭环与准入

下面内容可直接作为新对话的首条提示词。

```text
[$develop-robot-vision-navigation]

继续 `/home/kong/ATS_2026_snetry_test` 的 ATS 四驱四转哨兵导航优化。先完整读取：

1. `AGENTS.md`
2. `docs/项目优化文档/ATS导航剩余优化总TODO.md`
3. `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`
4. `docs/ats_swerve_mpc_ltv_qp_backend_admission.md`

本轮不是只做分析。按门禁持续执行“定位 -> 最小修改 -> 构建 -> 单测 -> Gazebo -> 文档 ->
分仓提交 -> SSH push”，但任何停止条件触发后必须保存证据并停止后续高风险阶段。

当前冻结事实：

- 根仓 `90b5bfbceb88`，导航仓 `5ea786eb2e70`，Gazebo 用户 fork `9ed6c41650e6`，
  MuJoCo `e3d6ea7a5e61`。
- `ats_robot_description` 本地 `dea591e53fa0`，远端仍是 `ed293ca0613e`，阻塞新电脑复建。
- Gazebo domain 230 的 `/localization` wall interval p50/p95/p99 为
  `0.371/0.994/1.612 s`，adapter 反复 ready=false，action fail-closed。
- `localization_fusion` 每个新 `/odometry` 回调立即发布 `/localization`；先审计
  `/lidar_odometry -> /odometry -> /localization -> status -> adapter`，不能直接归因 MINCO。
- MINCO optimizer 支持 `InitialKinematicState`，但 production node 四条路径仍传 `nullptr`。
- QualityEvaluator 已计算几何指标，但 production node 主要只用 finite/时间单调作质量拒绝；
  ESDF backtracking 主要只检查最小净空不下降。
- Gazebo runner 的 `TEST_PROFILE` 只用于日志命名；没有实际 corner/S/narrow/red-box 分支，
  action payload 也没有使用 `GOAL_YAW`。
- 默认 `solver_mode=ilqr`；`qp_shadow` 只诊断；`solver_mode=qp` 必须继续拒绝。

修改前先报告 DoD、精确文件范围、验证清单、假设/未验证项/停止条件，以及五个仓库的分支、
HEAD、upstream、remote 与工作区状态。

保护用户内容：导航仓 `ats_nav_bringup/scripts/static_map_publisher.py`、
`ats_swerve_mpc/求解器.md`，Gazebo `scripts/ats_bridge/gz_livox_bridge.py`。不得读取为设计依据，
不得修改、删除、暂存或提交。禁止 `git add .`、`git add -A`、破坏性恢复、force push。
Gazebo 只能推送用户 `origin/main`，禁止写 upstream。

阶段 A：远端复建

1. 核对 `ats_robot_description` 本地提交 `dea591e53fa0` 的内容、作者和远端。
2. 将 push remote 切换为用户 SSH remote，只推送现有本地提交，禁止改写历史。
3. 在临时干净目录执行 `vcs import dependencies.repos`，记录实际 SHA；确认四舵轮模型、Mid360、
   xmacro 能从远端取得，不依赖旧 build/install 或未跟踪文件。
4. push 或干净复建失败时停止，不进入运行性能测试。

阶段 B：Gazebo localization freshness

1. 在 Gazebo C++ evidence recorder 中只读测量 `/lidar_odometry`、`/odometry`、`/localization`、
   `/localization/status`、adapter heartbeat 和 `/clock`。
2. 每级记录 wall interval p50/p95/p99/max、ROS stamp interval/age、倒退/重复、消息数；记录 RTF、
   关键进程 CPU/RSS/thread/context switch 和 TF lookup failure。
3. 用全新 ROS_DOMAIN_ID 做受控 A/B：headless、recorder、RViz/相机/日志消融；每次只改一个因素，
   禁止 ros2cli 高频采样干扰被测链。
4. 找到最早违反 freshness 的 owner 后，只修改该 owner，并补最窄回归测试。
5. 禁止提高 odom/localization/map/lease timeout，禁止 ground truth 接管 `/localization`。
6. DoD：连续至少 60 s，`/localization` p99 < 0.25 s、无 >0.5 s gap、status 持续 TRACKING、
   adapter 不因 localization 抖动 ready=false；随后两个独立 straight domain action 成功。

运行前审计残留进程、ROS domain、load、CPU、内存和 swap。未知归属进程不得终止；高 swap、低内存、
CPU 饱和、Gazebo z 发散、RTF 异常或 TF owner 冲突均立即停止，不产生性能通过结论。

阶段 C：只有 B 通过后补齐 MINCO 生产契约

1. MincoPlannerNode 获取与 goal、snapshot、localization epoch 匹配的新鲜运动状态；核对 twist frame，
   转换到规划世界系，将同一冻结 InitialKinematicState 传给 center、footprint、fallback、repair 四条路径。
2. 增加 node 测试：非零 yaw 横移速度转换、裁剪、stale/epoch 拒绝、首端速度连续、终端零状态。
3. 把质量 telemetry 升级为门禁：直线使用 length ratio/横向偏差/曲率 TV/符号变化；一般曲线相对
   guide/baseline 比较；全部候选检查 v/a/j、净空、footprint/swept collision、时间和 snapshot。
4. ESDF backtracking 仅在净空不下降、碰撞不增加、长度与曲率变化不过门时接受。
5. 失败只回退到同 snapshot 上安全 baseline；否则急停和两级零速度。
6. 补 finite 但无意义多弯被拒绝、合法 S 弯不被误拒绝、旧 reference 不复活的 node/integration 测试。

阶段 D：实现真实 Gazebo profile runner

1. `TEST_PROFILE` 实际支持 `straight`、`single_corner`、`s_turn`、`narrow_corridor`、
   `nominal`、`red_box`，未知值失败。
2. 每个 profile 固定 world、起点、目标序列/yaw、timeout 和期望几何；不得只改日志目录。
3. action payload 将 `GOAL_YAW` 转成规范化 quaternion，保存 payload 和 scenario manifest。
4. 保存 raw/preprocessed/refined/reference/predicted/executed、地图 identity、owner、clearance、
   collision/contact、v/a/j、terminal error、replan/fallback 和两级速度。
5. 先每项一个开发 domain，稳定后每项两个独立验收 domain；失败保留首因，不复用机器人状态。

阶段 E：回归边界

- 重跑当前 revision 的 all-unknown、map-unready、map-stale、input-stale、unreachable、adapter lease、
  projection timeout、emergency-stop recovery；旧 revision 不能替代。
- 获取全局融合 ESDF、局部 RGB voxel、三色 bounds 同帧截图，验证移动 3 m 的滑窗跟随。
- P2 未完成 nominal/red-box/fault/clearance/contact 前不得通过。
- P3 未完成无 Nav2 servers 和 action 生命周期前不得标记 Nav2-free。
- sampled sweep 不能冒充严格连续证明；无独立 contact evaluator 时 physical contact 写 unverified。
- 禁止启用 `solver_mode=qp`，不要运行不可配对的 qp_shadow 性能比较。

验证顺序：

MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1
colcon test --base-paths src --packages-select <targets> --parallel-workers 1
colcon test-result --test-result-base build/<package> --verbose
python3 -m py_compile <changed_python_files>
bash -n <changed_shell_files>
ros2 launch <package> <launch> --show-args
git diff --check

最后更新总 TODO 的状态和当前方向文档；QP 未修改时不要改 backend admission。按仓库显式 stage，
中文详细提交，作者固定 `liukong1220 <1625038134@qq.com>`，分别 SSH push 有修改的用户仓库。
无修改仓库不制造空提交。报告每仓 baseline/final SHA、HEAD/upstream、测试、domain、指标、first
violation、未验证项和回滚 revision。
```
