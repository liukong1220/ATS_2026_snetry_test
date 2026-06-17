# 从当前 2.5D ESDF 过渡到 3D ESDF + A* / MINCO + SE2 MPC 的优化方向

更新时间：2026-06-17

本文档基于三部分内容整理：

1. `docs/中科大哨兵2025技术报告.pdf`
2. 当前仓库已经落地的 `terrain_analysis / terrain_analysis_ext / trajectory_optimizer / Gazebo` 代码
3. 现有过渡文档与仿真接入状态

目标不是继续把当前工程表述为“已经完成 PDF 同款导航链”，而是准确回答下面两个问题：

1. 现在这套代码到底已经做到哪一步了。
2. 如果要彻底从 `Nav2` 迁移到 `3D/2.5D ESDF + A* / MINCO + SE2 MPC`，下一步应该怎么做。

## 1. 先说结论

当前项目已经完成的部分是：

`点云/里程计 -> terrain_analysis -> terrain_analysis_ext -> traversability_grid + 地形语义调试栅格 -> signed Traversability ESDF -> Nav2 smoother / trajectory visual optimizer -> MPPI`

也就是说，现在仓库已经不再只是“纯 2D costmap + simple smoother”：

1. 已经把 `terrain_map_ext`、`traversability_grid` 和地形语义调试栅格接入到了 `trajectory_optimizer` 和 `Nav2BSplineSmoother`。
2. 已经支持 `esdf_source: traversability_grid`，说明 2.5D 地形分析结果开始直接参与路径回拉与近障碍代价。
3. `TraversabilityEsdfProvider` 已从单纯二值距离场升级为 signed ESDF，并融合 `height_diff / occupancy_ratio / ground_confidence` 三类语义输入。
4. 已经有 Gazebo 入口验证这条过渡链。

但当前项目还没有完成的关键部分同样需要明确：

1. 还没有自有 `A*` 前端搜索器。
2. 还没有 `MINCO` 轨迹表示与两阶段优化实现。
3. 还没有 `SE2 MPC` 控制器实现。
4. `Nav2` 仍然承担着 planner、controller、behavior lifecycle 的主链职责。

所以更准确的判断是：

`当前工程已经完成了“从 Nav2 纯 2D 代价地图向 2.5D ESDF 过渡”的前半段，但尚未进入自研 A* + MINCO + MPC 主链阶段。`

## 2. 结合当前代码，现状到底是什么

### 2.1 已落地的过渡链

当前仿真/实车主链仍以 `Nav2` 为骨架：

`SmacPlannerHybrid -> Nav2BSplineSmoother -> MPPI -> trajectory_speed_governor -> velocity_smoother`

但和最初版本相比，已经发生了两个实质变化：

1. `terrain_analysis_ext` 发布 `terrain_map_ext` 与 `traversability_grid`，不再只有 costmap 视角。
2. `trajectory_optimizer` 与 `Nav2BSplineSmoother` 都支持 `traversability_grid` ESDF 源。

从代码看，这个过渡已经落实在下面几处：

1. `terrain_analysis_ext` 发布 `terrain_map_ext`
2. `terrain_analysis_ext` 发布 `traversability_grid`
3. `terrain_analysis_ext` 发布 `traversability_height_diff_grid`
4. `terrain_analysis_ext` 发布 `traversability_occupancy_ratio_grid`
5. `terrain_analysis_ext` 发布 `traversability_ground_confidence_grid`
6. `trajectory_optimizer/src/traversability_esdf_provider.cpp` 会把 traversability 与三类语义栅格融合成 signed ESDF
7. `trajectory_optimizer_node` 和 `Nav2BSplineSmoother` 都支持 `esdf_source: traversability_grid`
8. Gazebo 仿真参数已经默认切到这套 traversability ESDF 过渡链

### 2.2 当前这套 “traversability ESDF” 的真实定位

它的价值很明确：

1. 已经摆脱了完全依赖 `global_costmap` 后处理构造 fake ESDF 的状态。
2. 已经把三维点云投影/筛选后的可通行分析结果接到了平滑器和可视化优化器上。
3. 已经为后续替换成真正的导航前端保留了 `EsdfProvider` 接口。

但它还不能等价于 PDF 里那套完整方案，原因也很明确：

1. 当前 `traversability_esdf_provider` 已经消费 `traversability_grid`、`height_diff`、`occupancy_ratio`、`ground_confidence`，并输出 signed distance。
2. 它目前仍然服务于 `B 样条平滑器` 和旁路 trajectory visual optimizer，不是 `MINCO` 两阶段优化器。
3. 它目前服务的控制器仍然是 `MPPI`，不是“牢牢贴轨迹”的 `SE2 MPC`。
4. 当前 ESDF 仍是二维 / 2.5D 过渡后端，不是完整 3D voxel ESDF。

因此它更适合被定义为：

`通向 3D/2.5D ESDF 主链的过渡型 Traversability ESDF 后端`

而不是最终形态。

### 2.3 既然已经切到 2.5D ESDF，现在能不能“看高度并做有效避障”

答案是：

`可以开始利用高度相关信息做更有效的地面避障，但还不能把它等同于完整 3D 体素避障。`

更具体地说，当前已经具备的能力是：

1. `terrain_map_ext` 仍然保留了点云层面的 `z` 信息，所以在 RViz 里已经可以直接观察局部障碍和地形高低。
2. `terrain_analysis_ext` 会把每个 `(x, y)` 栅格上的 `height_diff / max_rel_height / occupancy_ratio / ground_confidence` 折算成 `traversability_grid`。
3. `TraversabilityEsdfProvider` 会继续把这些 2.5D 语义输入融合成 signed ESDF，因此平滑器已经不只是“看二维黑白障碍”，而是在看“这个平面格子从地形语义上到底有多危险”。

所以对地面机器人来说，当前已经可以有效改善下面这些场景：

1. 台阶边缘、矮墙、坡坎、碎障碍堆积带来的高度差风险
2. 狭窄通道里“虽然平面看起来能过，但两侧高度/密度风险很高”的情况
3. 低置信度地面、稀疏点云区域导致的保守绕行

但必须明确当前还做不到的事情：

1. 还不能正确表达“同一个 `(x, y)` 上方有悬挑、下方可通过”这一类真正的 3D 结构
2. 还不能表达多层空间、桥下穿行、桌下可通行这类需要 `z` 方向拓扑推理的情况
3. `slam:=True` 时最终给 `slam_toolbox` 的仍然是 `terrain_map_ext -> obstacle_scan` 的二维投影结果，保存出来的地图资产本身不是带高度语义的地图

因此当前更准确的表述应该是：

`你已经能观察并利用高度差相关语义来改善地面避障，但现在的避障本质仍是“带高度语义的 2D / 2.5D 可通行判断”，不是完整 3D 规划。`

### 2.4 现在是否值得立刻更新 RViz

答案也是：

`值得，而且应该现在就做，不必等 A* / MINCO / MPC。`

原因很简单：

1. 现在最需要验证的是 `terrain_analysis_ext -> traversability_grid -> signed ESDF` 这条过渡链到底稳不稳。
2. 如果 RViz 里只能看到 `plan / smoothed_path / local_costmap`，你很难区分“路径贴边”到底是规划器问题、平滑器问题，还是地形语义本身把那里判成了高风险。
3. 在 `A* / MINCO` 尚未接入前，最值得提升的不是“轨迹炫酷程度”，而是“地图语义解释能力”。

对于当前仓库，RViz 建议分成三层来看：

1. 地图前端层：`terrain_map_ext`
2. 可通行语义层：`traversability_grid`
3. 解释层：`traversability_height_diff_grid / traversability_occupancy_ratio_grid / traversability_ground_confidence_grid`
4. 优化反馈层：`trajectory_esdf_debug / trajectory_profile_markers`
5. 控制跟踪层：`transformed_global_plan / trajectories / lookahead_point / footprint`

如果参考 `fast_planner` 前端或者 `~/rose_navigation`，建议借鉴的是：

1. `分层显示习惯`，不是 `MINCO` 本身
2. `原始路径 / 当前优化路径 / 历史路径` 同屏对照
3. `障碍层 / 风险层 / 轨迹层 / 起终点层` 同时存在
4. 用 `MarkerArray` 明确展示“为什么这里危险、轨迹为什么被推开”

但由于当前主线还是 `B 样条` 而不是 `MINCO`，所以短期不要追求复制 `MINCO` 特有的可视化语义，而应该优先把下面这几层看清楚：

1. `terrain_map_ext` 的高度分布
2. `traversability_grid` 的占障/可通行分区
3. `height_diff / occupancy_ratio / ground_confidence` 到最终 risk 的映射
4. `trajectory_esdf_debug` 的红绿点和梯度箭头是否真的与上述语义一致

## 3. PDF 方案里最值得照搬的部分

根据 `docs/中科大哨兵2025技术报告.pdf`，最应该直接继承的是下面这条分层逻辑：

`3D Occupancy Grid -> 高程/占据率可通行分析 -> 2D ESDF -> A* -> 时间重采样 -> MINCO 两阶段优化 -> SE2 MPC`

其中最关键的不是算法名，而是工程组织方式：

1. 地图前端主导规划，而不是 costmap 主导规划。
2. 前端搜索和后端优化解耦。
3. 轨迹表示天然支持重规划、前缀保留和时间优化。
4. 控制器只负责贴轨迹，不再自己“发明局部轨迹”。

这也是为什么后续方向不应该继续放在：

1. 给 `SmacPlannerHybrid` 再打更多补丁。
2. 把 `Nav2BSplineSmoother` 继续越堆越复杂。
3. 长期把 `MPPI critic` 调参当成主线。

## 4. 参考 EGO-Planner 时，应该借什么

建议借的是工程组织，而不是照搬其地图假设。

建议借鉴：

1. 高频重规划组织方式。
2. 上一条轨迹前缀保留。
3. 热启动与局部续接。
4. 让“搜索、优化、重规划决策”形成扁平直连链路。

不建议照搬：

1. `ESDF-free` 假设。
2. 无人机 3D 自由空间模型。
3. 无 footprint、无地形约束的碰撞模型。

一句话概括：

`借 EGO-Planner 的重规划组织，借技术报告的地图建模、MINCO 和 SE2 MPC 主链。`

## 5. 面向当前仓库的推荐目标架构

推荐最终目标链如下：

`Batch-LIWO / 里程计 -> 3D Occupancy -> 2.5D Traversability -> 2D ESDF -> A* -> MINCO(PRE + FINELY) -> SE2 MPC -> chassis`

其中每一层在当前仓库里的对应关系如下。

### 5.1 状态估计层

当前基础：

1. 已有 `point_lio / lidar_odometry` 相关链路
2. `terrain_analysis_ext` 已经直接订阅 `lidar_odometry`

这层短期不是重构重点，只要接口稳定即可。

### 5.2 3D 地图与 2.5D 可通行分析层

当前基础：

1. `terrain_analysis`
2. `terrain_analysis_ext`
3. `terrain_map`
4. `terrain_map_ext`
5. `traversability_grid`

下一阶段不要只把它当成“发点云给 costmap/ESDF”的模块，而要升级成真正的导航前端。

建议为每个 `(x, y)` 栅格显式维护：

1. `z_min`
2. `z_max`
3. `height_diff`
4. `occupancy_ratio`
5. `ground_confidence`
6. `dynamic_obstacle_confidence`
7. `traversable / occupied / unknown`

当前已经开始落地的第一步是：

1. `terrain_analysis_ext` 除了 `terrain_map_ext` 之外，新增输出 `traversability_grid`
2. `trajectory_optimizer` 与 `Nav2BSplineSmoother` 已经可以直接消费 `traversability_grid`
3. `TraversabilityEsdfProvider` 已经订阅并融合 `height_diff / occupancy_ratio / ground_confidence`
4. 当前阶段先保证 Gazebo 和现有 Nav2 主链能稳定消费 signed traversability ESDF，再逐步上 `A* / MINCO`

这一步做完之后，ESDF 才真正有“来自地形语义”的基础。

### 5.3 2D ESDF 层

当前基础：

1. `EsdfProvider` 抽象已经存在
2. `FakeCostmapEsdfProvider` 已存在
3. `TerrainPointCloudEsdfProvider` 已存在

下一步不该直接推翻，而应该新增第三类提供器，例如：

`TraversabilityEsdfProvider`

职责：

1. 输入不再是裸点云，而是“已判定可通行/不可通行/未知”的二维栅格和地形语义栅格
2. 输出 signed distance，负值代表已进入障碍 / 风险区，正值代表 free space clearance
3. 给后续 `A*`、`MINCO`、`SE2 MPC` 统一提供 `d(x,y)` 和 `grad d(x,y)`

### 5.4 前端搜索层

当前状态：

1. 仍由 `SmacPlannerHybrid` 提供 `plan`

目标状态：

1. 新增自有 `A*` 前端
2. 直接读取 signed `Traversability ESDF`
3. 输出离散路径点和初始时间分配信息
4. 支持目标点占障时的外推拉回
5. 支持沿旧轨迹前缀局部续接

建议做法：

1. 第一版直接上 `A*`，不要一开始追求 `Hybrid A*` 或 kinodynamic 搜索
2. 对当前四驱舵轮 / 全向底盘，位置路径和底盘朝向本来就应该解耦；全局搜索层没必要先把朝向约束硬塞进去
3. `JPS` 可以保留为后续性能优化项，而不是第一阶段必做项
4. 先完成“摆脱 Nav2 planner”，再考虑更复杂前端

这里要特别说明为什么当前更推荐 `A*` 而不是 `Hybrid A*`：

1. 你的底盘不是典型 Dubins / Ackermann 小车，`Hybrid A*` 的核心价值没有那么大。
2. 你已经明确希望把 `Yaw` 单独规划，因此搜索层更适合只管 `(x, y)` 位置可达性。
3. 当前 `traversability_grid + signed ESDF` 本身带有风险代价和 unknown 语义，`A*` 更容易直接把 clearance / risk 项加进代价函数。
4. `JPS` 在无权重均匀栅格上优势最明显，但你这里的搜索代价并不是纯均匀格，第一阶段优先实现清晰稳定的 `A*` 更合适。

### 5.5 轨迹优化层

当前状态：

1. 已有 `BSplinePathOptimizer`
2. 已有曲率代价、近障碍代价、速度剖面
3. 已有可视化与旁路验证节点

目标状态：

1. 新建独立 `minco_planner` 包
2. 轨迹表示改为 `MINCO`
3. 实现 `PRE_OPTIMIZATION + FINELY_OPTIMIZATION`
4. 实现二次插值/平滑梯度接口
5. 输出时参数轨迹与参考采样序列

建议演进方式：

1. 保留现有 `BSplinePathOptimizer` 作为 baseline
2. 新建并行 `MINCO` 实验链
3. 先实现“输入路标点 -> MINCO 优化 -> RViz 可视化”
4. 再接动作执行和重规划

如果按当前项目的真实节奏推进，还需要再加一个更务实的补充判断：

1. `minco_planner` 这个包现在就值得新建。
2. 但第一阶段不要强迫自己“新包一建好就必须用 MINCO 接管全部位置轨迹”。
3. 更稳的路线是：`minco_planner` 先承载 `A* + 独立 Yaw + 局部 B-spline 碰撞修补 + 调试消息`。
4. 等 signed ESDF 和窄门场景稳定以后，再把 `MINCO PRE/FINELY` 接进同一个包。

也就是说：

`包边界现在就应该按 MINCO 时代来设计，但第一阶段的主任务仍然是把 ESDF 驱动的前端搜索和局部轻量修补跑通。`

### 5.6 控制层

当前状态：

1. 仍由 `nav2_mppi_controller::MPPIController` 承担主控制
2. `trajectory_speed_governor` 只是给 MPPI 输出限速辅助

目标状态：

1. 自研 `SE2 MPC` 控制器替换 MPPI 主链
2. 直接跟踪时参数轨迹
3. 输入包含 `x y yaw vx vy wz` 与参考状态序列
4. 支持“底盘是否需要强跟随轨迹朝向”的控制模式切换

这一步完成后，`velocity_smoother` 和大量 MPPI critic 的职责会显著收缩。

## 6. 推荐迁移顺序

不建议一次性推翻。当前应把已经完成的过渡成果冻结成可验证基线，再继续替换 Nav2 的 planner、smoother 和 controller。

### 阶段 A：已完成的 2.5D ESDF 过渡基线

当前状态：

1. 保留 Nav2 主链。
2. `terrain_map_ext`、`traversability_grid` 与三类地形语义栅格已经接入轨迹优化链。
3. `TraversabilityEsdfProvider` 已经替代 fake costmap ESDF 成为当前主线后端。
4. fake costmap ESDF 与 terrain pointcloud ESDF 仍保留为 fallback / 对照路径。

保留价值：

1. 在不推翻 Nav2 的前提下，验证地形语义是否能稳定影响平滑路径。
2. 给后续 `A* / MINCO / MPC` 提供统一的 `EsdfProvider` 抽象。
3. 保留 loopback、Gazebo 和实车之间可对比的调试入口。

### 阶段 B：当前应优先完成的 traversability ESDF 稳定性验证

目标：

1. 在 Gazebo、loopback 和实车中验证 signed distance、梯度方向和风险点分布是否一致。
2. 固化 `d_min / d_avg / |g|avg / risk` 与 `height_diff / occupancy_ratio / ground_confidence` 的对应关系。
3. 确认可通行、不可通行和未知区域在 ESDF 中的符号与安全距离表现符合预期。
4. 保持 obstacle 权重保守，不在验证阶段同时大改 MPPI critic 和 governor。

产出：

1. 可重复的仿真测试场景。
2. 更稳定的 `terrain_analysis_ext` 参数。
3. signed Traversability ESDF 的上车观察清单。
4. 进入 `A* / MINCO` 前的地图前端验收基线。

### 阶段 C：替换 Nav2 planner/smoother

目标：

1. 新增 `A*` 前端搜索
2. 新增独立 `Yaw` 规划与局部 B-spline repair
3. 后续接入 `MINCO` 两阶段优化
4. 先并联输出调试，不抢控制权

产出：

1. `goal -> A* -> oriented path -> local repair` 的第一阶段规划链
2. 后续 `goal -> A* -> MINCO trajectory` 的完整规划链
3. 重规划、前缀保留、热启动逻辑

### 阶段 D：替换 Nav2 controller

目标：

1. 新增 `SE2 MPC`
2. 从 MPPI 切换到“严格贴轨迹”的控制器
3. 最后再逐步剥离 Nav2 action/lifecycle 依赖

产出：

1. 自研导航主链闭环
2. Nav2 从主系统退为对照组和兜底链

## 7. 面向当前仓库的具体开发任务建议

### 7.1 近期待办

建议优先做下面几项，而不是直接开写 MPC：

1. 在 Gazebo / loopback / 实车中验证 signed Traversability ESDF 的方向、距离和风险点是否一致
2. 固化 ESDF 观测指标，包括 `d_min / d_avg / risk_count` 与三类地形语义栅格的对应关系
3. 新建 `minco_planner` 包，先落 `A* + 独立 Yaw + 局部 B-spline repair`
4. 设计统一的 `ReferenceTrajectory` 消息或内部结构，避免后面 `MINCO` 与 `MPC` 接口再次重写

### 7.2 MINCO 接入建议

推荐单独建包，而不是塞进现有 `trajectory_optimizer`：

1. `minco_core`
2. `minco_planner_node`
3. `trajectory_sampler`

原因：

1. 当前 `trajectory_optimizer` 强绑定 B 样条语义
2. 继续在同一个类里揉会让过渡逻辑越来越难维护

### 7.3 MPC 接入建议

推荐单独建控制包，例如：

1. `se2_mpc_controller`

输入建议统一为：

1. 当前底盘状态
2. 参考点序列
3. 参考速度/加速度/朝向
4. 轨迹跟随模式位

不要一开始就继续挂在 Nav2 controller plugin 接口里实现全部逻辑；先做独立节点验证会更快。

不过对“下一阶段马上要不要上 MPC”这件事，需要再说得更细：

1. 当前位置最适合先保留 `MPPI` 做执行器。
2. 但不要再把 `MPPI` 当成“自己生成局部轨迹”的主体，而是把它降级成“沿上游给定 path / orientation 尽量稳定跟踪”的执行层。
3. 等 `A* + 独立 Yaw + 局部修补` 这一层稳定后，再判断 `MPPI` 是否已经足够通过狭窄地形。
4. 如果那时仍然出现“路径很好但底盘贴不住”的问题，再上 `SE2 MPC` 才是合理顺序。

### 7.4 RViz 继续升级时应该补什么

当前已经值得补齐的显示项是：

1. `terrain_map_ext` 按 `Z` 着色显示
2. `traversability_grid`
3. `traversability_height_diff_grid`
4. `traversability_occupancy_ratio_grid`
5. `traversability_ground_confidence_grid`
6. `trajectory_profile_markers`
7. `trajectory_esdf_debug`

后续如果要继续向 `fast_planner` / `rose_navigation` 的前端效果靠拢，建议再补：

1. 原始 planner path
2. smoothed / optimized path
3. 历史 path 或前缀保留 path
4. 局部 corridor / tunnel / safe band marker
5. 起点、目标点、当前参考点 marker

其中第 1 到第 4 项需要新增调试发布器，当前仓库不是没有思路，而是还缺稳定的发布话题。

### 7.5 针对当前项目，更合适的下一阶段计划

如果完全按当前仓库状态、底盘构型和你的目标来排优先级，我认为下面这条路线更合适：

#### 阶段 P1：不碰 `trajectory_optimizer`，新建 `minco_planner`

包内第一阶段建议只放这些模块：

1. `esdf_frontend_adapter`
2. `grid_astar`
3. `yaw_spline_planner`
4. `local_collision_repair`
5. `planner_debug_visualizer`

这个阶段的目标不是“立刻 MINCO 化”，而是：

`先把 signed ESDF 驱动的自有规划链跑起来，并能替代掉 Nav2 planner + 部分 smoother 职责。`

#### 阶段 P2：前端位置规划先用 `A*`

建议输入：

1. `TraversabilityEsdfProvider` 或等价的独立 ESDF 采样接口
2. 当前起点、目标点
3. clearance / risk / unknown 代价参数

建议输出：

1. 离散 `path points`
2. 每个点的累计弧长
3. 初始时间分配
4. 调试用 `raw_path`

这里先不上 `Hybrid A*` 的原因前面已经说了，再补一句最现实的：

`你当前真正缺的是“能稳定穿窄门的可解释路径”，不是“在搜索阶段就把朝向约束建得很复杂”。`

#### 阶段 P3：独立 `Yaw` 规划可以做，但代价函数不能只有平滑项

你提出的“5次均匀 B 样条拟合 `Yaw`，只惩罚角速度和角加速度”方向基本对，但如果真的只有这两项，会有一个问题：

1. 最优解很容易退化成“几乎常值 yaw”
2. 它未必会在进入窄门前主动对齐通道切向
3. 这样 MPPI 最后拿到的 orientation 参考不一定真的有用

因此更推荐的 `Yaw` 代价应当是：

1. 主项：角速度惩罚
2. 主项：角加速度惩罚
3. 约束项：转向电机转速上限
4. 轻量参考项：在狭窄区域或低 clearance 区域，对路径切向 `yaw_tangent` 施加弱到中等权重
5. 边界项：起点 `yaw` 必须贴当前车体实际朝向

一句话：

`Yaw 可以独立规划，但不能完全无参考；至少在狭窄环境里要被“通道切向”轻度牵引。`

#### 阶段 P4：局部安全修补值得做，但要按“局部支撑”语义来实现

你说的这一步我认为是当前方案里最有价值的部分之一，而且确实很接近 `EGO-Planner` 最值得借的工程思想：

1. 先对轨迹按固定时间或弧长采样
2. 对每个采样点做车体矩形 footprint collision check
3. 发现碰撞后，不做全局重算
4. 只取碰撞段影响到的 `3~4` 个局部控制点
5. 沿安全方向拖动一小步
6. 重新局部拟合并再次校验

但这里有一个实现细节必须说清：

1. 这套“拖动 3~4 个控制点”的轻量修补，与 `uniform B-spline` 的局部支撑性天然契合。
2. 它和 `MINCO` 的“经过路标点”特性不是同一种局部修改语义。
3. 所以第一阶段你完全可以在 `minco_planner` 包里使用一个“局部 B-spline repair layer”，并不矛盾。

也就是说，第一阶段最合理的形式不是：

`A* -> MINCO -> 再硬改 MINCO 控制点`

而更像是：

`A* -> 初始轨迹/采样点 -> 局部 B-spline repair -> oriented path -> MPPI`

等这一层稳定后，再考虑把中间的“初始轨迹表示”替换成真正的 `MINCO`。

#### 阶段 P5：先下发给 `MPPI`，但要明确它能跟什么、不能跟什么

这一步可以做，而且对当前项目最务实。

但需要明确边界：

1. 当前 Nav2 `MPPI` 主接口消费的是 `nav_msgs/Path`，不是完整时参数轨迹。
2. 它能比较好地做的是“路径 + orientation” 跟踪。
3. 它做不了真正意义上的“严格按你给定的时间参数和 yaw 速度边界执行”。

因此当前更准确的目标应该写成：

1. 把 `minco_planner` 输出成带 orientation 的 path
2. 在 MPPI 侧打开对 path orientation 更敏感的跟踪配置
3. 把 MPPI 先用成“参考路径跟踪器”，不是“最终版轨迹控制器”

这一步特别适合先在 `rmuc_2025` 的窄门场景验证：

1. 是否更早对齐通道切向
2. 是否减少门前左右试探
3. 是否减少局部贴障时的抖动
4. 是否比当前 `Smac + bspline` 更可解释

#### 阶段 P6：ESDF 稳定后再上 MINCO

这一步我赞成，而且比“现在立刻把 MINCO 和控制器一起接上”更稳。

推荐接入顺序：

1. 先实现 `A* -> 局部修补 -> MPPI`
2. 跑通 `rmuc_2025` 窄门、贴边、S 弯
3. 固化 signed ESDF 采样接口与调试指标
4. 再把 `MINCO PRE + FINELY` 接到 `minco_planner`
5. 最后才决定是否替换 `MPPI`

原因很简单：

1. 现在系统里最大的未知量仍然是 `2.5D traversability -> signed ESDF` 的稳定性
2. 如果现在同时引入 `MINCO + Yaw + 局部修补 + MPC`，变量会过多
3. 先让 `A* + repair + MPPI` 过窄门，能够更快判断 ESDF 和规划表示到底谁在拖后腿

## 8. 这份文档对应的现实判断

如果以“是否已经彻底改成技术报告里的方案”为标准，答案是：

`还没有。`

如果以“是否已经朝那个方向完成了关键过渡基础设施”为标准，答案是：

`已经完成了一部分，而且方向是对的。`

最重要的不是继续把当前链路包装成最终形态，而是承认它现在所处的位置：

`当前项目处于“2.5D signed Traversability ESDF 过渡阶段”，下一步应先验证 ESDF 稳定性，再补齐 A*、MINCO，并最终替换 MPPI 为 SE2 MPC。`

## 9. 推荐的下一版里程碑

建议把后续工作拆成下面三个里程碑：

1. `M1`：验证 signed Traversability ESDF 与 `traversability_grid / height_diff / occupancy_ratio / ground_confidence` 的一致性
2. `M2`：新建 `minco_planner`，实现 `A* + 独立 Yaw + 局部 B-spline repair`，先输出给 MPPI
3. `M3`：在 `minco_planner` 中接入 `MINCO PRE/FINELY`
4. `M4`：若 MPPI 仍无法稳定贴轨，再实现 `SE2 MPC`

按这个顺序推进，风险和返工都会比“一口气全改”小很多。

## 10. Gazebo 仿真地图的直接建议

既然你现在最需要验证的是 `2.5D traversability -> signed ESDF -> bspline smoothing / MPPI`，那么场景选择应该优先服务于“高度差、狭窄通道、近障碍贴边、局部重规划观察”，而不是一开始就追求超大场景。

### 10.1 第一优先级：先把你自己的 Gazebo 世界改造成“语义压力测试场”

最推荐的第一步不是立刻换掉整套世界，而是基于当前仓库已有的：

1. `rmuc_2025`
2. `rmul_2025`

直接加几类障碍构型：

1. 高低台阶
2. 斜向窄门
3. 两侧贴边通道
4. 低矮障碍簇
5. 一侧可通、一侧高度差明显的 S 弯

原因：

1. 你已经有现成机器人、桥接、出生点、导航地图资产和 TF 链
2. 这样最容易看出 `traversability_grid` 与 `trajectory_esdf_debug` 的变化
3. 比一开始迁移外部大世界更省时间

### 10.2 第二优先级：优先参考 Gazebo Sim / Ignition 路线的现成世界

更适合直接参考或迁移的是 `Open-RMF` 的 Gazebo Sim 世界：

1. `rmf_demos_gz office`
2. `rmf_demos_gz clinic`
3. `rmf_demos_gz airport_terminal`

推荐理由：

1. `office` 是单层室内环境，带走廊、门、补给点，最适合先压测局部避障和转角贴边
2. `clinic` 有更复杂的房间组织和多层结构，适合后面验证更长路径与复杂拓扑
3. `airport_terminal` 场景更大，而且官方示例里明确包含 crowd sim 和只读车辆避让，更适合后期压力测试重规划

对当前阶段的优先顺序建议是：

1. 先 `office`
2. 再 `airport_terminal`
3. 最后再考虑 `clinic / hotel` 这类多层世界

原因不是这些世界不好，而是你当前链路仍然是单层地面导航 + 2.5D 语义前端，多层世界暂时不会把你的核心问题暴露得更清楚。

### 10.3 第三优先级：Classic 世界只作为参考资源

网上有两类很经典的 Gazebo Classic 世界仍然值得拿来参考布局：

1. `aws-robomaker-small-warehouse-world`
2. `aws-robomaker-bookstore-world`

它们的价值主要在于：

1. 仓储窄通道、货架夹缝、遮挡关系清晰
2. 书店货架和桌椅混排，适合看局部路径回拉

但当前不建议把它们当第一优先级直接深度接入，原因是：

1. Gazebo Classic 已在 `2025-01` 进入 EOL
2. 你当前仓库主线是 `Gazebo Sim / Ignition`
3. 直接接 Classic 资产，往往会把时间花在迁移材质、插件和 world 兼容性上

因此更合理的用法是：

1. 参考其世界布局
2. 按同样的“货架-窄通道-拐角-半开区域”结构，在你自己的 `Gazebo Sim` 世界里重建轻量版本

### 10.4 对你现在最有效的落地建议

如果只做一轮最务实的推进，我建议按这个顺序：

1. 先更新 RViz，把 `terrain_map_ext + traversability_grid + 三类语义栅格 + trajectory_esdf_debug` 看清楚
2. 在当前 `rmuc_2025 / rmul_2025` 世界里人工加几个“高度差 + 贴边 + 狭窄通道”测试构型
3. 跑 `Gazebo + Nav2 + bspline + MPPI`，先把 `2.5D` 语义链验证透
4. 然后再挑 `rmf_demos_gz office` 这类 Gazebo Sim 场景做第二轮外部世界验证

这样推进的收益最高，因为你现在最缺的不是“更大的世界”，而是：

`一个能稳定解释为什么轨迹被推开、为什么某处被判成 risk、以及这些判定是否真的来自高度语义的观察闭环。`
