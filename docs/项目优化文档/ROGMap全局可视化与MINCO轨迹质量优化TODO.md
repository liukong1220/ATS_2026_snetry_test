# ROGMap 全局可视化与 MINCO 轨迹质量优化 TODO

> 状态：设计基线，尚未实施算法或 RViz 修改
> 日期：2026-08-15
> 适用链路：Gazebo、MuJoCo 与实车共用的 `ROGMap -> adapter/RC-ESDF -> JPS -> MINCO -> MPC` 规控链

## 1. 本阶段目标与非目标

本阶段解决两个可独立验收、但必须在同一地图快照和规控链中观察的问题：

1. 在 RViz 中同时得到类似参考截图的全局距离场背景和随机器人移动的 ROGMap 局部滑窗诊断；
2. 消除 MINCO 对直线或低曲率 JPS 引导线引入的不必要弯折，并建立曲率感知的局部时间分配与轨迹质量门禁。

本阶段不是把局部 ROGMap 改造成无限增长的全局地图，也不是用 RViz 点云代替数值 ESDF。它不替换
Point-LIO、JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 或全向 SE(2) MPC；不会引入
差速、Ackermann、ICR 或 `vy=0` 约束。

## 2. 源码基线与已确认事实

### 2.1 ROGMap 与全局距离场

| 结论 | 当前源码证据 | 设计含义 |
| :--- | :--- | :--- |
| `/rog_map/esdf` 是局部调试点云 | `ats_rog_map_node.cpp::publishDebug()` 将 ESDF bounds 与机器人中心 `visualization_range` 相交后采样 | 不得把该 topic 扩展后冒充全局规划 ESDF |
| ROGMap 是滑动窗口 | `core.map_sliding.enable=true`、`core.map_size=[10,10,1]`、`core.visualization.range=[8,8,1]` | 局部窗口应随机器人移动，旧区域会离开活动数值地图 |
| 局部可视范围按机器人位置计算 | `visualization_box = robot_position +/- 0.5 * visualization_range`，再裁到 local map | 紫色可视框应连续跟随机器人；橙色 local-map 框按滑动阈值分段移动 |
| adapter 已生成全局融合距离场 | `GroundProjectionFusion::fuse()` 以静态图全尺寸生成 planning grid，随后构建 RC-ESDF | 无需新建第二套全局数值地图 |
| 全局显示 topic 已存在 | adapter 发布 `/rc_esdf/signed_distance_grid`，将 signed distance 映射到 `OccupancyGrid` 的 `0..100`，unknown 为 `-1` | 第一版应启用该 display，并明确它是有损、仅用于显示的全局融合 RC-ESDF |
| 数值规划契约另有权威 | `PlanningMapSnapshot` 携带 occupancy、signed distance、gradient、generation 与 publication sequence | 规划继续消费不可变数值快照，绝不从显示栅格或 `/rog_map/esdf` 反解析 |

因此，本阶段的术语必须固定为：

- **全局融合 RC-ESDF**：`/rc_esdf/signed_distance_grid`，覆盖静态图全尺寸，融合 static/terrain/slope 与当前 ROGMap projection；
- **ROGMap 局部数值 ESDF**：projection service 和不可变数值快照中的 signed distance/gradient；
- **ROGMap 局部显示**：`/rog_map/viz`、`/rog_map/esdf`、`/rog_map/bounds`，只用于诊断滑动窗口；
- **禁止的伪全局图**：把历史 `/rog_map/esdf` 点云永久累积、把 debug cloud 当规划输入，或单纯放大 `core.map_size` 覆盖整场。

### 2.2 JPS 到 MINCO 的当前几何与时间流程

当前 `MincoTrajectoryOptimizer` 的主要步骤为：

```text
JPS jump points
-> 删除重复点和几乎严格共线点
-> 每 0.30 m 加密控制点
-> 对低净空采样逐点施加 ESDF 梯度位移
-> Ti = max(Tmin, segment_length / reference_speed)
-> MINCO S3 五次分段求解
-> 速度/加速度超限时对全部 Ti 统一乘相同 scale
-> 离散采样并执行最终 footprint safety gate
```

已确认的结构缺口：

- 近共线删除阈值固定为 `1e-3`，没有按 planning resolution、横向偏差或最短段长归一化；
- 没有使用同一地图快照进行保守 line-of-sight shortcut，JPS 的短首尾段和相邻转折都成为硬插值点；
- ESDF 修正会先在每条线段上加密控制点，再让相邻点各自响应局部梯度；没有法向投影、offset 平滑、曲率变化或回溯接受准则；
- 只要距离小于 soft clearance 就可能改变原本无碰撞的直线，而不是先判断几何是否真的需要偏移；
- 时间分配只看段长，不看转角、几何曲率、初始速度、终端速度和局部加速度需求；
- 动力学超限时统一拉长所有段，局部拐点可能让整条直线段同时变慢；
- 现有 GTest 验证端点、finite derivative 和 ESDF 净空提升，但没有直线不增弯、曲率/拐点数量、时间分配或 noisy-gradient 防锯齿回归。

这些事实可以解释用户观察到的现象，但尚无当前 revision 的 raw/final 轨迹配对指标，因此“ESDF 密集点独立位移是唯一根因”仍是待实验验证的假设。

## 3. Definition of Done

### 3.1 ROGMap/RViz DoD

- [ ] Gazebo 与实车 RViz 都默认显示全局融合 RC-ESDF，显示名称不得暗示它是纯 ROGMap 全局历史；
- [ ] 全局 grid 的 `frame/origin/yaw/resolution/width/height` 与 `/map` 和 adapter 输出契约一致；
- [ ] `/rog_map/viz`、`/rog_map/esdf` 和三类 bounds 仍为局部窗口，机器人移动时局部内容与框同步移动；
- [ ] 紫色 visualization center 相对机器人位置误差不超过一个 ROGMap cell；橙色 local-map center 只在跨越 sliding threshold 后移动；
- [ ] 全局显示更新时 adapter `source_generation` 和 `publication_sequence` 持续递增，局部 topic stamp 不倒退；
- [ ] RViz 订阅不改变 planning-grid owner、projection 数值语义、地图 stale/unknown/lease 或急停行为；
- [ ] 开启全局/局部显示相对 headless baseline 不得让 projection、adapter 或 map lock 的 p95/p99 出现未解释退化；
- [ ] 保存静止、直线移动、跨滑窗阈值和故障恢复四类截图/结构化日志。

### 3.2 MINCO DoD

- [ ] 自由空间直线、含冗余共线点的直线和 JPS 两点直线均满足“直线不增弯”；
- [ ] 保留精确 start/goal、世界系首端速度播种、终端零速度/零加速度和独立 yaw；
- [ ] JPS 几何预处理只移除经同一 immutable snapshot 证明 swept footprint 安全的冗余点，不穿越 unknown、outside 或 occupied；
- [ ] 时间分配显式考虑段长、初末速度、转角/几何曲率、最大速度和最大加速度，所有 duration finite 且严格为正；
- [ ] ESDF 几何修正只在净空门禁触发时运行，控制点偏移以法向为主、沿路径平滑，并通过 trust-region/backtracking 接受；
- [ ] 优化后不得增加 footprint collision，最终安全仍由独立 footprint checker 决定，solver success 不构成安全授权；
- [ ] 失败时只允许回退到同 snapshot 上通过独立安全复核的基线轨迹，否则保持急停和两级零速度；
- [ ] Gazebo 直线、单拐角、S 弯、窄通道、red-box 至少各两次独立 domain，指标优于或不劣于冻结基线；
- [ ] 不改变 `/cmd_vel_mpc`、`/motion_control`、goal、地图、急停和底盘唯一 ownership；
- [ ] P2、P3、P4 只按原门禁判定，本阶段通过不能自动标记 P2 完整通过或 Nav2-free。

## 4. 轨迹质量指标与判定公式

对每次规划同时保存 `JPS raw -> preprocessed guide -> ESDF-refined guide -> MINCO final -> executed path`。

### 4.1 几何指标

对三点 $p_{i-1},p_i,p_{i+1}$，使用不依赖车体模型的离散几何曲率：

$$
\kappa_i = \frac{2\left| (p_i-p_{i-1}) \times (p_{i+1}-p_i) \right|}
{\|p_i-p_{i-1}\|\,\|p_{i+1}-p_i\|\,\|p_{i+1}-p_{i-1}\|}.
$$

必须记录：

- path length 与 `MINCO/JPS` 长度比；
- 直线最大横向偏差 $d_{\perp,\max}$；
- 最大 $|\kappa|$、曲率 p95、总转角、曲率 total variation；
- 曲率符号变化次数和小幅连续摆动次数；
- shortcut 前后 waypoint 数、短段数量和最小段长；
- minimum center clearance、minimum oriented-footprint clearance、离散/连续 swept collision 数。

直线初始验收阈值：

- `max_lateral_deviation <= max(0.02 m, 0.25 * planning_resolution)`；
- `MINCO length / direct length <= 1.01`；
- 除首尾低速数值区外，不得出现曲率符号反复变化；
- far-obstacle/free-space case 中不得触发 ESDF geometry refinement。

上述数值是首轮工程门禁，不是实车动力学极限；若基线数据证明地图离散误差更大，只能通过成对实验和文档审批调整，禁止为通过测试直接放宽。

### 4.2 动力学与时间指标

四舵轮仍按全向平面轨迹评估。曲率只用于路径方向变化和局部速度分配，不作为 Ackermann 转向硬约束。

候选 waypoint 速度上限：

$$
v_i^* = \min\left(v_{\mathrm{ref}},v_{\max},
\sqrt{\frac{a_{\perp,\max}}{\max(|\kappa_i|,\epsilon)}}\right).
$$

前向/后向加速度传播：

$$
v_{i+1} \le \sqrt{v_i^2+2a_{\max}L_i},\qquad
v_i \le \sqrt{v_{i+1}^2+2a_{\max}L_i}.
$$

初始段时长：

$$
T_i=\max\left(T_{\min},\frac{2L_i}{v_i+v_{i+1}+\epsilon}\right).
$$

还必须记录 peak/p95 velocity、acceleration、jerk、duration、solver wall time p50/p95/p99、迭代次数和局部/全局 time-scaling 次数。低速点的解析曲率奇异时使用三点几何曲率，不除以接近零的速度。

## 5. 分阶段实施 TODO

### P2.9-0：冻结当前可视化基线

- [ ] 在低负载、全新 `ROS_DOMAIN_ID` 下分别运行 Gazebo headless、Gazebo 单 RViz 和实车配置静态解析；
- [ ] 采集 `/map`、`/rc_esdf/planning_grid`、`/rc_esdf/signed_distance_grid`、`/rog_map/viz`、`/rog_map/esdf`、`/rog_map/bounds` 的 type/frame/QoS/rate/width/height；
- [ ] 保存机器人静止、直线移动 3 m、跨越 `1.0 m` sliding threshold 的 bounds center 序列；
- [ ] 记录 projection、adapter fuse/ESDF、debug collect/serialize/publish、map-lock p50/p95/p99；
- [ ] 如果全局 signed-distance grid 本身为空、frame 错误或不随 generation 更新，先修 producer，再改 RViz。

### P2.9-1：启用真实的全局融合 ESDF display

- [ ] 同步修改 `src/ats_sentry_bringup/rviz/sentry_default_view.rviz` 和 Gazebo fork 的 `rviz/ats_gazebo_nav.rviz`；
- [ ] 新 display 命名为 `Global Fused RC-ESDF (ROGMap + Static + Terrain)`；
- [ ] topic 固定 `/rc_esdf/signed_distance_grid`，`Reliable + Transient Local + Keep Last 1`；
- [ ] 使用 `Map/costmap` 颜色方案，先以 `Alpha 0.55--0.70`、`Draw Behind=true` 为视觉基线；
- [ ] static PGM 降低 alpha，planning grid 保留较低 alpha；局部 `/rog_map/viz` 覆盖在全局层上；
- [ ] 在 RViz 注释和文档说明 `0..100` 是 display encoding，不是可逆的米制 signed-distance；
- [ ] 更新 `scripts/validate_navigation_config.py` 及聚焦测试，锁定两份配置的名称、topic、QoS、启用状态和层级。

### P2.9-2：局部滑窗跟随与更新契约

- [ ] 为 `publishDebug()` 的 box 计算抽出纯函数，仅在运行证据证明现有跟随错误时实施；
- [ ] 纯函数测试覆盖机器人连续移动、local-map 阈值滑动、地图边界裁剪、非整数 resolution 和 yaw-free map frame；
- [ ] `visualization range` 中心应跟随机器人，local-map range 中心按 ROGMap sliding origin 更新；
- [ ] `/rog_map/viz`、occ/inf_occ/unk/esdf 必须使用同一 debug snapshot、frame、stamp 和 bounds；
- [ ] RViz `Decay Time=0`，防止旧窗口残留形成伪全局轨迹；
- [ ] debug 保持 subscriber-gated、锁内采集不可变 snapshot、锁外序列化和 publish；
- [ ] 不改变 projection service、source generation、ESDF sign/gradient、unknown 或 planning snapshot。

### P2.9-3：可视化性能与运行验收

- [ ] A/B/C 成对运行：headless、全局 ESDF only、全局 + 局部 ROGMap；每项至少两次新 domain；
- [ ] 保存 RViz canvas 像素非空检查和截图，确认全场彩色距离场、局部 RGB voxel、三色 bounds 同时可见；
- [ ] 局部窗口移动后旧点不残留，全局 grid origin/尺寸不随机器人漂移；
- [ ] source generation、adapter sequence 持续递增，唯一 planning-grid owner 不变；
- [ ] projection 与 adapter p95/p99 若相对 headless 退化超过 20%，停止并定位渲染、DDS、锁或序列化 owner，不放宽 deadline。

### P2.10-0：冻结 MINCO 现状与增加只读 telemetry

- [ ] 构造确定性直线、冗余共线点、短首尾段、90 度、S 弯、U 弯、窄通道和 noisy ESDF fixture；
- [ ] 输出每一阶段 waypoint、duration、轨迹质量指标和失败首因；
- [ ] 在任何参数调整前保存当前 revision 的单测/离线/Gazebo基线；
- [ ] 把“直线弯折”分为 JPS 输入折线、ESDF guide 位移、MINCO interpolation overshoot、MPC tracking 四层，不用 RViz 目测直接归因。

### P2.10-1：JPS 几何预处理器

- [ ] 新建独立 `PathGeometryPreprocessor`，由 MINCO 调用，不改变 JPS 搜索成功/失败语义；
- [ ] 去重、按 resolution 的近共线删除、短段合并、角点标记和弧长重采样分开实现；
- [ ] shortcut 必须在同一 immutable planning snapshot 上执行完整定向 footprint/swept 检查；
- [ ] unknown、outside、occupied 和 map generation 变化均 fail-closed，不得只检查中心点；
- [ ] 精确 start/goal 永远保留，不能被栅格中心替换；
- [ ] 为 raw JPS 与 preprocessed guide 分别保留 debug topic/marker，避免优化后无法定位误差来源。

### P2.10-2：曲率感知时间分配

- [ ] 新建可单测的 `MincoTimeAllocator`，输入 guide、初始速度、终端速度和动力学参数；
- [ ] 以段长基础时长、离散几何曲率限速、前向/后向加速度传播生成初值；
- [ ] 终端速度/加速度保持零，重规划首端使用经过裁剪的当前世界系速度/加速度；
- [ ] MINCO 求解后只放大违规段及相邻连续性影响段，不再默认把全部 duration 同比放大；
- [ ] 每轮重新检查所有段的 velocity/acceleration/jerk，超过迭代上限即返回结构化失败；
- [ ] duration、scale、curvature 和 violation index 进入 telemetry；
- [ ] 参数进入唯一实际加载配置，仿真只覆盖 `use_sim_time` 等 profile 量。

### P2.10-3：抑制 ESDF 修正引入的锯齿

- [ ] 把 `trigger clearance` 与 `target clearance` 分离并带滞回；足迹已满足 hard gate 的直线不因微小梯度噪声变形；
- [ ] 梯度只取相对 guide tangent 的法向分量，禁止沿切向制造点密度/速度突变；
- [ ] 对 offset 加一阶/二阶平滑正则，限制相邻控制点符号频繁翻转；
- [ ] 保持相对 guide 的 trust region、单调进度和最大偏移；
- [ ] 每次 candidate 通过 backtracking 接受：净空必须改善，collision 不增加，长度/曲率变化不越门禁；
- [ ] 无 finite ESDF、unknown、outside、零梯度或 snapshot 变化时不得移动点；
- [ ] 保留 center 与 yaw-aware footprint 两阶段，但二者必须共享相同预处理 guide、时间分配和快照 identity。

### P2.10-4：MINCO S3 质量复核与安全回退

- [ ] 在 `MincoS3` 中暴露可审计 jerk sample 或等价精确积分，不修改 S3 连续性阶次；
- [ ] 记录 jerk cost、guide deviation、length、clearance、curvature variation 和 duration，不只看总 cost；
- [ ] final candidate 独立复核 finite、时间单调、速度/加速度/jerk、footprint 和 snapshot freshness；
- [ ] quality gate 失败时回退到同 snapshot 上安全的 preprocessed JPS-MINCO baseline；
- [ ] baseline 也不安全时不得发布 reference，保持 `emergency_stop=true`、`/cmd_vel_mpc=0`、`/motion_control=0`；
- [ ] 急停恢复仍需新目标/新请求 identity，旧 reference 不得复活。

### P2.10-5：聚焦测试

- [ ] `StraightPathRemainsStraightWithRedundantCollinearPoints`；
- [ ] `SafeStraightPathDoesNotTriggerEsdfRefinement`；
- [ ] `BlockedShortcutPreservesCollisionFreeCorner`；
- [ ] `ShortcutRejectsUnknownOutsideAndSweptFootprintCollision`；
- [ ] `CurvatureAwareAllocationSlowsCornerWithoutSlowingUnrelatedStraightSegments`；
- [ ] `ShortEndpointSegmentsRemainFiniteAndMonotonic`；
- [ ] `NoisyAlternatingGradientDoesNotCreateZigzag`；
- [ ] `RefinementBacktracksWhenClearanceOrCurvatureRegresses`；
- [ ] `FinalCandidatePreservesEndpointsInitialStateAndTerminalStop`；
- [ ] `UnsafeCandidateFailsClosedAndOldReferenceCannotRevive`。

### P2.10-6：Gazebo/MuJoCo/实车分级验收

- [ ] Gazebo 先跑 straight、single-corner、S-turn、narrow-corridor、nominal、red-box，每项两次独立 domain；
- [ ] 保存 raw/guide/ref/predicted/executed 五层轨迹、地图 generation、最小 clearance、collision/contact、终点误差和重规划次数；
- [ ] 重跑 all-unknown、map-unready、map-stale、input-stale、unreachable、adapter-lease、projection-timeout 和 recovery；
- [ ] Gazebo 通过后以 MuJoCo 相同输入做跨后端回归，分离算法改进与仿真动力学差异；
- [ ] HIL 前完成 Gate 0--2；实车先低速直线和停止，再单拐角，再狭窄通道；
- [ ] 实车具备独立物理急停、速度/加速度限幅、命令 lease、安全观察员和明确回滚 commit；
- [ ] 未完成 continuous swept footprint 与独立 contact evaluator 前不得标记 P4。

## 6. 预计文件范围

### 根仓

- `src/ats_sentry_bringup/rviz/sentry_default_view.rviz`
- `src/ats_sentry_bringup/params/node_params.yaml`
- `scripts/validate_navigation_config.py`
- `scripts/test_validate_navigation_config.py`
- 新增只读轨迹/ROGMap 指标采集脚本与本文档

### 导航仓 `src/ats_sentry_nav`

- `ats_rog_map/src/ats_rog_map_node.cpp` 及最窄测试（只在跟随审计证明实现错误时修改）
- `ats_rog_map_adapter` 的 display encoding/测试（只在全局 grid producer 证据不成立时修改）
- `minco_planner/include/minco_planner/trajectory/*`
- `minco_planner/src/trajectory/*`
- 新增 path preprocessor、time allocator、quality evaluator 及对应 GTest
- `minco_planner/src/nodes/minco_planner_node.cpp`
- `minco_planner/config/minco_planner_reality.yaml`

### Gazebo 用户 fork

- `rmu_gazebo_simulator/rviz/ats_gazebo_nav.rviz`
- 必要时仅修改现有回归 recorder/launch 参数，不改变上游 remote

MuJoCo、机器人描述和 MPC 在没有直接行为缺陷证据时不修改；无改动仓库不得制造空提交。

## 7. 验证顺序与停止条件

验证必须按 `静态/单测 -> 离线 fixture -> Gazebo headless -> Gazebo RViz -> MuJoCo -> HIL -> 低速实车` 前进。

立即停止并保留日志的条件：

- 当前 revision 无法复现直线弯折，或 raw/final 配对证明问题属于 MPC/定位而不是 MINCO；
- 全局 display 与 `/map` frame/origin/yaw 不一致，或 adapter generation 不推进；
- 可视化让 projection/adapter deadline、map lock 或控制周期出现显著尾延迟；
- shortcut/ESDF refinement 穿过 unknown、outside、occupied 或增加 footprint collision；
- duration 非正、NaN/Inf、速度/加速度/jerk 超限、solver 超时或曲率摆动加重；
- `/cmd_vel_mpc`、`/motion_control`、planning grid、TF 或急停出现重复 owner；
- 地图、localization、TF、reference 或 heartbeat 不新鲜；
- 资源审计发现未知长跑进程、高 swap/低可用内存，无法形成可信性能证据；
- 实车独立急停、限速、测试区域或安全观察员任一不具备。

禁止通过放宽 unknown、occupied、projection deadline、adapter lease、old-reference、footprint、MPC freshness 或急停规则绕过停止条件。

## 8. Git 与证据要求

- 开始与结束分别检查根仓、导航仓、Gazebo fork、MuJoCo 和机器人描述仓状态；
- 保留导航仓 `ats_swerve_mpc/求解器.md`、`static_map_publisher.py` 模式位和 Gazebo 未跟踪 bridge 文件；
- 机器人描述仓本地未推送提交不是本阶段内容，未取得 SSH remote 前不得混入本轮；
- 禁止向 Gazebo `upstream` 写入，Gazebo 只允许推送用户 `origin/main`；
- 禁止 `git add .`、`git add -A`、force push 和历史重写；
- 提交按 `[可视化]`、`[规划]`、`[优化]`、`[安全]`、`[仿真]`、`[文档]` 分拆；
- 作者固定为 `liukong1220 <1625038134@qq.com>`，不得包含 Claude/Anthropic co-author；
- 每项报告 baseline/final commit、实际命令、指标、失败首因、未运行测试和 `HEAD == upstream branch`。

## 9. 2026-08-15 实施与证据边界

### 已实现并通过静态/组件验证

- [x] 两份实际 RViz 配置均启用 `Global Fused RC-ESDF (ROGMap + Static + Terrain)`：
  `rviz_default_plugins/Map` 订阅 `/rc_esdf/signed_distance_grid`，使用 `Reliable + Transient Local
  + Keep Last 1`、`costmap`、`Alpha=0.62`、`Draw Behind=true`；static PGM alpha 降至 `0.30`。
  局部 `/rog_map/viz`、`/rog_map/esdf`、`/rog_map/bounds` 仍叠加在全局层上，`/rog_map/viz`
  `Decay Time=0`。`0..100` 只作为 display encoding，未被反解析为米制 signed distance。
- [x] `scripts/validate_navigation_config.py` 和其 9 个 Python 聚焦测试锁定两份 RViz 的全局层、
  QoS、层级、MINCO 两个中间 guide display 与局部 cloud 的无累积契约；配置 validator、Python
  syntax、runner shell syntax 均通过。
- [x] 新增 `PathGeometryPreprocessor`、`MincoTimeAllocator` 和 `TrajectoryQualityEvaluator`。前者在
  同一 immutable planning snapshot 上完成去重、近共线/短段处理、角点标记和 footprint-aware shortcut；
  unknown/outside/occupied 仍 fail-closed，并精确保留 start/goal。后两者分别完成曲率限速、
  前后向加速度传播、局部相邻段 time scaling，以及长度、横向偏差、曲率、clearance、collision、
  v/a/j、duration、finite/严格时间单调的独立复核。
- [x] TODO 列出的十个聚焦 GTest 已落到三个目标：`StraightPathRemainsStraightWithRedundantCollinearPoints`、
  `BlockedShortcutPreservesCollisionFreeCorner`、`ShortcutRejectsUnknownOutsideAndSweptFootprintCollision`、
  `CurvatureAwareAllocationSlowsCornerWithoutSlowingUnrelatedStraightSegments`、
  `ShortEndpointSegmentsRemainFiniteAndMonotonic`、`SafeStraightPathDoesNotTriggerEsdfRefinement`、
  `NoisyAlternatingGradientDoesNotCreateZigzag`、`RefinementBacktracksWhenClearanceOrCurvatureRegresses`、
  `FinalCandidatePreservesEndpointsInitialStateAndTerminalStop`、
  `UnsafeCandidateFailsClosedAndOldReferenceCannotRevive`。完整 `minco_planner` CTest 为 `11/11`，
  三个新增目标的再运行也为 `3/3`。
- [x] `MincoPlannerNode` 发布 `/minco/raw_path`、`/minco/preprocessed_guide`、
  `/minco/esdf_refined_guide`、candidate/final reference；运行日志包含 raw/guide 点数、snapshot
  generation/publication sequence、质量指标、各段 duration、失败首因和 optimizer wall time。
  `MincoS3` 已公开解析 jerk sample；原有 S3、独立 yaw 与全向 `[v_x,v_y,w_z]` 契约未改变。

### 已保存的 Gazebo 证据

- [x] **未修改源码 baseline**：headless domain `221`、`222` 与五层 payload capture domain `223`
  已在任何源码编辑前保存。domain `223` rosbag 为 `38.3 MiB / 26.737 s / 605 messages`，包含
  `/map`、planning/signed-distance grid、三类 ROGMap local debug、`/localization`、JPS raw、
  MINCO reference、MPC predicted/executed。其最大点数为 `19/111/31/13`，ROG source generation
  `128->194`、adapter sequence `79->122`。这些 baseline 最终均因后续 watchdog/freshness 终止，
  只能证明链路曾实际生成五层轨迹，不能归因 MINCO 为唯一根因。
- [x] domain `226` 给出可复现的 MINCO 回归：安全二点直线被 ESDF 阶段无条件加密成 9 个硬控制点，
  `peak_a=2.659 > 2.5 m/s^2`。实现已改为先评估连续曲线，只有 clearance trigger 才插入内部控制点；
  同时局部 time scaling 覆盖违反段及一阶相邻段，远端无关直线段不被整体放慢。
- [x] 修复后的 domain `227` 和 `228/229/230` 都实际观测到 `raw/preprocessed/refined=2/2/2`
  的自由空间直线以及 MINCO/MPC/执行路径和两级非零速度。domain `230` 的 candidate 例子为
  `length_ratio=1.000`、`lateral=0`、`curvature_max/p95/TV=0/0/0`、`curvature_sign_changes=0`、
  `collisions=0`、`peak_v<=2.0`、`peak_a<=2.5`、`peak_j<=12.0`；该 `collisions=0` 仅是规划
  离散检查，不能推出 Gazebo physical contact 为零。
- [x] runner 改为由 C++ 只读 recorder 在 action 期间记录 planning-grid graph ownership；启动期
  `ros2cli` graph cache 仅作诊断，不再把 health gate 前的 `unverified` 误判为 owner 失败。domain
  `230` 记录 `/rc_esdf/planning_grid` publisher max `1`、adapter 已见、无命名非 adapter；
  `_NODE_*_UNKNOWN_` 端点保留为 CycloneDDS 匿名诊断，未被计入第二个实际 publisher。速度链仍为
  `/cmd_vel_mpc=1`、`/motion_control=1`、Gazebo chassis command=`1` 个动作期 publisher。

### 当前停止条件与未完成门禁

- [x] domain `230` 在 action 中触发 freshness 停止条件：`localization` wall interval
  `p50/p95/p99=0.371/0.994/1.612 s`，adapter 多次发布 `ready=false`，目标在 `0.802 m` 处
  `ABORTED`，随后两级速度为零。资源快照仍显示历史高 swap，因此这些 samples 不能作为低负载
  p50/p95/p99、nominal 通过或性能对比证据；没有修改 localization/TF、lease、unknown、footprint、
  old-reference 或 emergency-stop 门禁。
- [ ] 因上述停止条件，尚未运行单拐角、S 弯、窄通道、nominal 两次、red-box、单 RViz 截图与三米
  sliding-window 跟随、八个独立 fault domain、MuJoCo 跨后端、HIL 或实车。未采集全局 ESDF 与局部
  bounds 同帧截图，也未验证 visualization/local/update center 或旧点残留。
- [ ] runner 仍没有独立 Gazebo contact evaluator，`minimum_clearance_m`、全轨迹 footprint/swept
  collision 和 physical contact 必须保持 `unverified`，不得由 MINCO telemetry 代替。
- [ ] P2 未通过；P3 不得标记 Nav2-free；P4 连续 swept footprint、实车动力学、HIL/实车均未进入。

下一次低负载复验必须从新的 `ROS_DOMAIN_ID` 开始，先取得一次不触发 localization/TF/heartbeat
freshness gate 的 headless straight，再按本文件第 7 节推进；不得复用 domain `227--230` 的 timing
样本替代该门禁。
