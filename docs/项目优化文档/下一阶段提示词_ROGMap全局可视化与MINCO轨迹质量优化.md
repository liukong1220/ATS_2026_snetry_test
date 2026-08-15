# 下一阶段提示词：ROGMap 全局可视化与 MINCO 轨迹质量优化

将下面整段内容复制到新对话。新电脑必须先拉取所有用户仓库的远程最新 revision，再按实际源码和运行证据继续。

````text
你是 ATS 2026 四驱四转哨兵导航项目的精确代码修改者和闭环验证者。请完成“ROGMap 全局/局部可视化 + JPS/MINCO 轨迹质量优化”阶段。不要只给方案；必须依次完成审计、最小实现、聚焦测试、Gazebo 闭环、文档、分仓提交和推送。若满足停止条件，保留证据并停止，不得放宽安全门禁凑通过。

第一步必须完整读取：

1. `/home/kong/ATS_2026_snetry_test/AGENTS.md`
2. `docs/项目优化文档/ROGMap全局可视化与MINCO轨迹质量优化TODO.md`
3. `docs/项目优化文档/ATS导航优化TODO.md`
4. `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`
5. ROGMap、adapter、JPS、MINCO、RViz 和相关测试的当前活动源码。

不要读取 `build/`、`install/`、`log/`、`参考/` 或媒体作为设计依据；需要运行工件时只读取本轮新生成且明确相关的日志。

一、修改前必须先报告

在任何编辑前先输出：

- Definition of Done；
- 精确文件范围；
- 可执行验证清单；
- 当前假设、未验证项和停止条件；
- 五个仓库的 branch、HEAD、upstream、remote 和工作区状态。

至少检查：

```bash
git status --short --branch
git rev-parse HEAD
git rev-parse '@{upstream}'
git remote -v
```

仓库边界：

- 根仓：`/home/kong/ATS_2026_snetry_test`，`develop -> origin/develop`；
- 导航仓：`src/ats_sentry_nav`，`develop -> origin/develop`；
- Gazebo 用户 fork：`src/sim/gazebo_simulator/rmu_gazebo_simulator`，`main -> origin/main`；
- MuJoCo：`src/sim/ats_mujoco_sim`，`develop -> origin/develop`；
- 机器人描述：`src/ats_robot_description`，存在本地未推送提交时只记录，不得混入本阶段。

保留并不得暂存、修改或删除的用户内容：

- `src/ats_sentry_nav/ats_swerve_mpc/求解器.md`；
- `src/ats_sentry_nav/ats_nav_bringup/scripts/static_map_publisher.py` 的既有模式位变化；
- `src/sim/gazebo_simulator/rmu_gazebo_simulator/scripts/ats_bridge/gz_livox_bridge.py` 未跟踪文件；
- 其他开始时发现的未知修改。

Gazebo 的 `upstream` 只读，严禁 commit、push、PR 或任何写操作；只能推送用户的
`git@github.com:liukong1220/rmu_gazebo_simulator.git`。禁止 `git add .`、`git add -A`、force push、历史重写和 Claude/Anthropic co-author。提交作者必须只有 `liukong1220 <1625038134@qq.com>`。

二、先确认问题归属，不要立即调参

建立两张证据表：

1. 地图显示表：`/map`、`/rc_esdf/planning_grid`、`/rc_esdf/signed_distance_grid`、`/rog_map/viz`、`/rog_map/esdf`、`/rog_map/bounds` 的 producer、consumer、frame、stamp、QoS、rate、origin/yaw、width/height、generation 和用途；
2. 轨迹阶段表：JPS raw、preprocessed guide、ESDF-refined guide、MINCO final、MPC predicted、executed path 的 frame、时间、点数、长度、曲率、clearance、publisher 和 snapshot identity。

当前源码事实必须复核：

- `/rog_map/esdf` 只采样 ROGMap 滑动窗口和机器人中心 visualization range 的交集，所以它是局部 debug topic；
- `core.map_sliding.enable=true`，局部窗口应随机器人移动；
- adapter 已按静态图全尺寸构建融合 planning grid 和 RC-ESDF，并发布 `/rc_esdf/signed_distance_grid`；
- `/rc_esdf/signed_distance_grid` 是 `0..100` 的有损显示编码，真正规划数据仍在不可变 `PlanningMapSnapshot` 中；
- MINCO 当前按段长/参考速度分配时间，ESDF 修正会加密并独立移动控制点，动力学超限时整体缩放所有段；
- 当前测试没有直线不增弯、曲率变化、局部时间分配或 noisy-gradient 防锯齿门禁。

如果任一事实与当前 revision 不一致，以源码为准，先更新 TODO 再实施。

三、ROGMap/RC-ESDF 可视化实施

先在低负载、全新 `ROS_DOMAIN_ID` 下采集 headless baseline。不要仅因 RViz 没显示就修改 producer。

第一优先级是复用现有全局 topic：

- 在实车 `src/ats_sentry_bringup/rviz/sentry_default_view.rviz` 和 Gazebo
  `rmu_gazebo_simulator/rviz/ats_gazebo_nav.rviz` 中启用 `/rc_esdf/signed_distance_grid`；
- display 命名为 `Global Fused RC-ESDF (ROGMap + Static + Terrain)`；
- 使用 `rviz_default_plugins/Map`、`costmap` color scheme、`Reliable + Transient Local + Keep Last 1`；
- 先用 `Alpha 0.55--0.70`、`Draw Behind=true`，降低 static PGM alpha，保留低 alpha planning grid；
- 局部 `/rog_map/viz` 和 `/rog_map/bounds` 覆盖在全局层上，形成与用户参考图相同的“全场距离背景 + 机器人附近滑窗”层次；
- `/rog_map/esdf` 保持可选局部诊断，不能重命名成全局图；
- 在 RViz 注释中明确显示编码不能被反解析成米制 ESDF。

只有运行证据表明 `/rc_esdf/signed_distance_grid` 为空、frame/origin/yaw 错误或不随 adapter generation 更新，才修改 adapter producer。禁止：

- 把历史 `/rog_map/esdf` 点云永久累积成伪全局地图；
- 放大 ROGMap `map_size` 覆盖整场以换取截图效果；
- 从任何 RViz/debug `PointCloud2` 反解析规划距离；
- 改变 unknown、occupied、outside、distance sign、gradient 或规划 snapshot 语义。

验证局部跟随：

- 机器人直线移动至少 3 m；
- 记录 localization、紫色 visualization bounds、橙色 local-map bounds、绿色 raycast bounds 的中心；
- visualization center 相对机器人误差不得超过一个 ROGMap cell；
- local-map center 应在跨越 sliding threshold 后移动；
- `/rog_map/viz`、occ/inf_occ/unk/esdf 使用同一 frame/stamp/bounds；
- RViz `Decay Time=0`，旧局部点不得残留成伪全局图。

如果实现确有错误，再把 box 计算抽成纯函数并补边界、resolution、滑动阈值和裁剪 GTest。保留 subscriber-gated、锁内 snapshot、锁外序列化和 publish。

更新 `scripts/validate_navigation_config.py` 和聚焦测试，锁定两份 RViz 的 display 名称、topic、QoS、启用状态、颜色方案、alpha 和分组。使用 Playwright 不适用于 RViz；必须用实际 RViz 截图、窗口像素非空检查和 ROS payload/QoS 证据。

四、先建立 MINCO 可复现基线

不要先改权重。新增只读 evaluator/telemetry，针对以下固定 fixture 和 Gazebo route 保存：

- 两点直线；
- 带冗余共线点的直线；
- 极短首段/尾段；
- 90 度单拐角；
- S 弯；
- U 弯；
- 窄通道；
- 交替 noisy ESDF gradient；
- nominal 和 red-box。

每次保存 `JPS raw -> preprocessed guide -> ESDF-refined guide -> MINCO final -> MPC predicted -> executed`，记录：

- 点数、path length 和长度比；
- 直线最大横向偏差；
- 最大/ p95 离散几何曲率、总转角、曲率 total variation、曲率符号变化次数；
- minimum center/oriented-footprint clearance；
- discrete footprint 和 continuous swept collision（若 continuous 尚未实现则明确未验证）；
- 每段 duration、peak velocity/acceleration/jerk；
- solver wall time p50/p95/p99、迭代数、fallback 和失败首因；
- 最终 tracking error、终点误差、replan/recovery 和 Gazebo contact telemetry。

离散三点几何曲率使用：

$$
\kappa_i = \frac{2\left| (p_i-p_{i-1}) \times (p_{i+1}-p_i) \right|}
{\|p_i-p_{i-1}\|\,\|p_{i+1}-p_i\|\,\|p_{i+1}-p_{i-1}\|}.
$$

低速端点不得使用除以速度三次方的解析曲率。四舵轮保持全向模型；曲率用于几何质量和速度分配，不得引入 Ackermann/ICR/`vy=0` 约束。

五、JPS 几何预处理

新增独立、可单测的 `PathGeometryPreprocessor`，由 MINCO 前端调用，JPS 搜索本身保持不变。

按顺序实现：

1. 去除重复点；
2. 基于 planning resolution、横向误差和角度的近共线删除；
3. 合并小于配置最短长度的短段；
4. 标记真实转角；
5. 使用同一 immutable planning snapshot 做保守 line-of-sight shortcut；
6. 仅为数值稳定做弧长重采样，不能把每个采样点都变成不必要的硬插值点。

shortcut 必须检查带 yaw 的 `0.70 x 0.55 m + margin` 定向 footprint 和 swept motion；unknown、outside、occupied、map generation 变化全部拒绝。精确 start/goal 必须保留。发布 raw JPS 和 preprocessed guide 的独立 debug marker/topic，禁止用最终 MINCO 路径覆盖输入证据。

六、曲率感知时间分配

新增独立 `MincoTimeAllocator`，不要把逻辑继续堆进 `optimize()`。

候选 waypoint 速度：

$$
v_i^*=\min\left(v_{ref},v_{max},\sqrt{a_{lat,max}/\max(|\kappa_i|,\epsilon)}\right).
$$

执行前向/后向加速度传播：

$$
v_{i+1}\le\sqrt{v_i^2+2a_{max}L_i},\qquad
v_i\le\sqrt{v_{i+1}^2+2a_{max}L_i}.
$$

初始 duration：

$$
T_i=\max\left(T_{min},2L_i/(v_i+v_{i+1}+\epsilon)\right).
$$

要求：

- 首端使用裁剪后的当前世界系速度/加速度，终端保持零速度/零加速度；
- duration finite、严格为正并有上限；
- MINCO 求解后只放大违规段及相邻连续性影响段，不再默认让全部段统一变慢；
- 每轮重新检查 velocity/acceleration/jerk；
- 达到迭代或 wall deadline 后返回结构化失败，不执行未通过复核的最后 iterate；
- 记录每段 curvature、target speed、duration、scale 和 violation reason。

七、ESDF 几何修正防锯齿

保留 MINCO S3 和独立 yaw，不另写替代轨迹器。重构现有 `refineWaypointsWithEsdf()`：

- 分离 `trigger_clearance` 与 `target_clearance`，加入滞回；
- 对已满足 hard footprint safety 的直线，far-obstacle/free-space case 不允许启动几何修正；
- ESDF gradient 只取 guide 法向分量，禁止沿切向制造点密度变化；
- 对相邻 offset 增加一阶/二阶平滑，抑制符号交替；
- 约束相对原 guide 的 trust region、单调进度和最大偏移；
- 每次 candidate 使用 backtracking 接受；只有 clearance 改善、collision 不增加、长度与曲率变化不越门禁时才能提交；
- ESDF unknown/outside/NaN/Inf/零梯度或 snapshot 改变时不移动点；
- center 和 yaw-aware footprint 两次求解必须共享同一 guide、time allocation 和 snapshot identity。

在 `MincoS3` 中增加可审计 jerk sample 或等价精确 cost，但不得改变 S3 连续性阶次或许可证声明。最终 candidate 必须由独立 quality evaluator 和 footprint checker 复核。

八、强制聚焦回归

至少新增：

- `StraightPathRemainsStraightWithRedundantCollinearPoints`；
- `SafeStraightPathDoesNotTriggerEsdfRefinement`；
- `BlockedShortcutPreservesCollisionFreeCorner`；
- `ShortcutRejectsUnknownOutsideAndSweptFootprintCollision`；
- `CurvatureAwareAllocationSlowsCornerWithoutSlowingUnrelatedStraightSegments`；
- `ShortEndpointSegmentsRemainFiniteAndMonotonic`；
- `NoisyAlternatingGradientDoesNotCreateZigzag`；
- `RefinementBacktracksWhenClearanceOrCurvatureRegresses`；
- `FinalCandidatePreservesEndpointsInitialStateAndTerminalStop`；
- `UnsafeCandidateFailsClosedAndOldReferenceCannotRevive`。

首轮直线门禁：

- `max_lateral_deviation <= max(0.02 m, 0.25 * planning_resolution)`；
- `MINCO length / direct length <= 1.01`；
- far-obstacle/free-space 不触发 ESDF refine；
- 除首尾低速数值区外，不出现曲率符号反复变化；
- 端点精确、时间严格单调、所有导数 finite、速度/加速度/jerk 不越限。

如果实际基线说明阈值不合理，必须用同输入成对数据说明后再调整；禁止直接放宽测试。

九、构建与仿真顺序

先执行最窄检查：

```bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select ats_rog_map ats_rog_map_adapter minco_planner ats_sentry_bringup \
  --parallel-workers 1

colcon test --base-paths src \
  --packages-select ats_rog_map ats_rog_map_adapter minco_planner \
  --ctest-args --output-on-failure

python3 scripts/test_validate_navigation_config.py
python3 -m py_compile <changed launch/scripts>
bash -n <changed shell scripts>
ros2 launch <affected package> <launch> --show-args
git diff --check
```

然后使用低负载、新 domain 依次执行：

1. Gazebo headless straight；
2. Gazebo headless single-corner、S-turn、narrow-corridor；
3. Gazebo nominal 两次；
4. 单 RViz 的全局 ESDF + 局部滑窗移动截图；
5. red-box；
6. all-unknown、map-unready、map-stale、input-stale、goal-unreachable、adapter-lease、projection-timeout、emergency-stop-recovery；
7. MuJoCo 同输入跨后端回归。

每个 fault 使用独立 domain，禁止串行污染机器人状态。必须验证：

```text
ready=false
-> emergency_stop=true
-> /cmd_vel_mpc=0
-> /motion_control=0
```

恢复后 generation/sequence 必须继续推进，且没有新目标时急停前 reference 不得复活。

十、停止条件

遇到任一情况立即停止并保留日志：

- raw/guide/ref 配对表明弯折来自 MPC、定位或显示，而不是 MINCO；
- 当前 revision 无法确定性复现问题；
- global grid frame/origin/yaw 错误或 generation 不推进；
- RViz 使 projection/adapter p95/p99 相对 headless 退化超过 20%；
- shortcut 或 refinement 穿越 unknown/outside/occupied、增加 collision 或降低 hard clearance；
- duration 非正、NaN/Inf、动力学超限、solver 超时或曲率摆动更严重；
- 地图、localization、TF、heartbeat、reference freshness 不满足；
- 出现重复 planning grid、TF、急停、`/cmd_vel_mpc` 或 `/motion_control` owner；
- 资源审计发现未知长跑进程、高 swap、低内存或 CPU 争用；
- 实车缺少独立急停、限速、隔离区域和安全观察员。

不得通过放宽 timeout、unknown、occupied、lease、old-reference、footprint、MPC freshness 或急停规则继续。

十一、文档、提交与推送

必须更新：

- `docs/项目优化文档/ROGMap全局可视化与MINCO轨迹质量优化TODO.md`；
- `docs/项目优化文档/ATS导航优化TODO.md`；
- `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`。

按内容拆分：

- 导航仓：`[规划] 规范JPS几何预处理与安全shortcut`；
- 导航仓：`[优化] 增加MINCO曲率感知时间分配与质量门禁`；
- 导航仓：`[安全] 收紧ESDF轨迹修正与回退复核`；
- 根仓/Gazebo fork：`[可视化] 分层显示全局融合ESDF与ROGMap局部滑窗`；
- 根仓：`[仿真] 增加轨迹质量与滑窗跟随回归`；
- 根仓：`[文档] 记录ROGMap与MINCO实测边界`。

每次只显式 stage 本轮文件，先检查：

```bash
git diff --cached --stat
git diff --cached --check
```

分别推送用户仓库对应分支；无修改仓库不制造空提交。最终报告：

- 修改文件；
- baseline/final commit；
- 实际命令和结果；
- 直线/拐角/S 弯/窄通道/nominal/red-box 指标；
- 全局 ESDF 与局部 bounds 跟随证据和截图；
- source generation、adapter sequence、snapshot identity；
- minimum clearance、footprint/swept collision、contact telemetry；
- 两级速度、唯一 ownership、故障停机与恢复；
- p50/p95/p99 和资源情况；
- 未执行的 HIL/实车/P2/P3/P4 门禁；
- 每仓 `HEAD == upstream` 与 shortlog 作者。

当前 P2 不因 Gazebo 已接入或本阶段局部通过而自动完成；P3 仍不得仅因 `launch_nav2=false` 称为 Nav2-free；continuous swept footprint、独立 physical contact evaluator、HIL 和实车未通过前不得标记 P4。
````
