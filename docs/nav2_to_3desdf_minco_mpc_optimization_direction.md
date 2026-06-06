# 从当前 2.5D ESDF 过渡到 3D ESDF + JPS + MINCO + SE2 MPC 的优化方向

更新时间：2026-06-06

本文档基于三部分内容整理：

1. `docs/中科大哨兵2025技术报告.pdf`
2. 当前仓库已经落地的 `terrain_analysis / terrain_analysis_ext / trajectory_optimizer / Gazebo` 代码
3. 现有过渡文档与仿真接入状态

目标不是继续把当前工程表述为“已经完成 PDF 同款导航链”，而是准确回答下面两个问题：

1. 现在这套代码到底已经做到哪一步了。
2. 如果要彻底从 `Nav2` 迁移到 `3D/2.5D ESDF + JPS + MINCO + SE2 MPC`，下一步应该怎么做。

## 1. 先说结论

当前项目已经完成的部分是：

`点云/里程计 -> terrain_analysis -> terrain_analysis_ext -> terrain_pointcloud ESDF -> Nav2 smoother / trajectory visual optimizer -> MPPI`

也就是说，现在仓库已经不再只是“纯 2D costmap + simple smoother”：

1. 已经把 `terrain_map_ext` 接入到了 `trajectory_optimizer` 和 `Nav2BSplineSmoother`。
2. 已经支持 `esdf_source: terrain_pointcloud`，说明 2.5D 地形点云已经开始参与路径回拉与近障碍代价。
3. 已经有 Gazebo 入口验证这条过渡链。

但当前项目还没有完成的关键部分同样需要明确：

1. 还没有自有 `JPS / A*` 前端搜索器。
2. 还没有 `MINCO` 轨迹表示与两阶段优化实现。
3. 还没有 `SE2 MPC` 控制器实现。
4. `Nav2` 仍然承担着 planner、controller、behavior lifecycle 的主链职责。

所以更准确的判断是：

`当前工程已经完成了“从 Nav2 纯 2D 代价地图向 2.5D ESDF 过渡”的前半段，但尚未进入自研 JPS + MINCO + MPC 主链阶段。`

## 2. 结合当前代码，现状到底是什么

### 2.1 已落地的过渡链

当前仿真/实车主链仍以 `Nav2` 为骨架：

`SmacPlannerHybrid -> Nav2BSplineSmoother -> MPPI -> trajectory_speed_governor -> velocity_smoother`

但和最初版本相比，已经发生了两个实质变化：

1. `terrain_analysis_ext` 发布 `terrain_map_ext`，不再只有 costmap 视角。
2. `trajectory_optimizer` 与 `Nav2BSplineSmoother` 都支持 `terrain_pointcloud` ESDF 源。

从代码看，这个过渡已经落实在下面几处：

1. `terrain_analysis_ext` 发布 `terrain_map_ext`
2. `trajectory_optimizer/src/terrain_pointcloud_esdf_provider.cpp` 会把 `terrain_map_ext` 转成二维距离场
3. `trajectory_optimizer_node` 和 `Nav2BSplineSmoother` 都支持 `esdf_source: terrain_pointcloud`
4. Gazebo 仿真参数已经默认切到这套点云 ESDF 过渡链

### 2.2 当前这套 “terrain_pointcloud ESDF” 的真实定位

它的价值很明确：

1. 已经摆脱了完全依赖 `global_costmap` 后处理构造 fake ESDF 的状态。
2. 已经把三维点云投影/筛选后的结果接到了平滑器和可视化优化器上。
3. 已经为后续替换成真正的导航前端保留了 `EsdfProvider` 接口。

但它还不能等价于 PDF 里那套完整方案，原因也很明确：

1. 当前 `terrain_pointcloud_esdf_provider` 本质上还是“把点云直接栅格化后做二维距离传播”。
2. 它没有显式维护 PDF 里强调的 `occupancy / traversability / height_diff / occupancy_ratio / ground_confidence` 这些中间层语义。
3. 它目前服务的仍然是 `B 样条平滑器`，不是 `MINCO` 两阶段优化器。
4. 它目前服务的控制器仍然是 `MPPI`，不是“牢牢贴轨迹”的 `SE2 MPC`。

因此它更适合被定义为：

`通向 3D/2.5D ESDF 主链的过渡型 ESDF 后端`

而不是最终形态。

## 3. PDF 方案里最值得照搬的部分

根据 `docs/中科大哨兵2025技术报告.pdf`，最应该直接继承的是下面这条分层逻辑：

`3D Occupancy Grid -> 高程/占据率可通行分析 -> 2D ESDF -> JPS -> 时间重采样 -> MINCO 两阶段优化 -> SE2 MPC`

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

`Batch-LIWO / 里程计 -> 3D Occupancy -> 2.5D Traversability -> 2D ESDF -> JPS -> MINCO(PRE + FINELY) -> SE2 MPC -> chassis`

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
2. 第一版语义先收敛为 `unknown / traversable / occupied`
3. 先保证 Gazebo 和现有导航链能稳定消费，再逐步补 `height_diff / occupancy_ratio / ground_confidence`

这一步做完之后，ESDF 才真正有“来自地形语义”的基础。

### 5.3 2D ESDF 层

当前基础：

1. `EsdfProvider` 抽象已经存在
2. `FakeCostmapEsdfProvider` 已存在
3. `TerrainPointCloudEsdfProvider` 已存在

下一步不该直接推翻，而应该新增第三类提供器，例如：

`TraversabilityEsdfProvider`

职责：

1. 输入不再是裸点云，而是“已判定可通行/不可通行/未知”的二维栅格
2. 显式支持静态障碍与动态障碍融合
3. 给后续 `JPS`、`MINCO`、`SE2 MPC` 统一提供 `d(x,y)` 和 `grad d(x,y)`

### 5.4 前端搜索层

当前状态：

1. 仍由 `SmacPlannerHybrid` 提供 `plan`

目标状态：

1. 新增自有 `JPS/A*` 前端
2. 输出离散路标点和初始时间分配信息
3. 支持目标点占障时的外推拉回
4. 支持沿旧轨迹前缀局部续接

建议做法：

1. 第一版直接上 `A* / JPS`，不要一开始追求 kinodynamic 搜索
2. 先完成“摆脱 Nav2 planner”，再考虑更复杂前端

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

不建议一次性推翻。推荐分四阶段推进。

### 阶段 A：把 2.5D ESDF 过渡链做扎实

目标：

1. 保留 Nav2 主链
2. 继续使用 `terrain_map_ext -> terrain_pointcloud ESDF`
3. 在 Gazebo 和 loopback 中验证“贴边回拉、狭窄通道、安全侧偏移”是否稳定

产出：

1. 可重复的仿真测试场景
2. 更稳定的 `terrain_analysis_ext` 参数
3. 更准确的 ESDF 调试指标

### 阶段 B：从点云 ESDF 过渡到 Traversability ESDF

目标：

1. 把 `terrain_analysis_ext` 升级为可通行分析前端
2. 输出二维 traversability grid
3. 基于 traversability 构建新的 ESDF provider

产出：

1. `traversability_grid` 第一版
2. `TraversabilityEsdfProvider`
3. 独立于 Nav2 costmap 的地图前端

### 阶段 C：替换 Nav2 planner/smoother

目标：

1. 新增 `JPS` 前端搜索
2. 新增 `MINCO` 两阶段优化
3. 先并联输出调试，不抢控制权

产出：

1. `goal -> JPS -> MINCO trajectory` 的完整规划链
2. 重规划、前缀保留、热启动逻辑

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

1. 给 `terrain_analysis_ext` 增加可通行栅格输出，而不是只发点云
2. 抽象新的 `TraversabilityEsdfProvider`
3. 新建 `planner_core` 包，先落 `JPS/A*`
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

## 8. 这份文档对应的现实判断

如果以“是否已经彻底改成技术报告里的方案”为标准，答案是：

`还没有。`

如果以“是否已经朝那个方向完成了关键过渡基础设施”为标准，答案是：

`已经完成了一部分，而且方向是对的。`

最重要的不是继续把当前链路包装成最终形态，而是承认它现在所处的位置：

`当前项目处于“2.5D ESDF 过渡阶段”，下一步应优先补齐 traversability、JPS、MINCO，再替换 MPPI 为 SE2 MPC。`

## 9. 推荐的下一版里程碑

建议把后续工作拆成下面三个里程碑：

1. `M1`：完善 `terrain_analysis_ext`，输出 traversability grid，并在 Gazebo 中验证 ESDF 与地形分析一致
2. `M2`：实现 `JPS + MINCO` 并联规划链，只做可视化和轨迹发布，不接控制权
3. `M3`：实现 `SE2 MPC`，在 Gazebo 中完成闭环跟踪，最后再逐步摘除 Nav2 主链

按这个顺序推进，风险和返工都会比“一口气全改”小很多。
