# 从 2.5D 语义 ESDF 到稳定比赛版与长期最终版导航主链

更新时间：2026-06-17

本文档只保留两条主线：

1. `V1 稳定比赛版`
   `2.5D 语义地图 + 2D 栅格导航主链 + JPS / MINCO / SE2 MPC`
2. `V2 长期最终版`
   `3D ESDF + JPS / MINCO / SE2 MPC`

当前优化工作优先服务于 `V1`。`V2` 是在 `V1` 稳定后再推进的长期目标。

## 0. 目标流程图

下面两张图对应 `V1` 的目标工程形态，核心思想与 `docs/中科大哨兵2025技术报告.pdf` 一致，但更明确地区分了：

1. 2.5D 语义建图
2. 2D 栅格主链
3. 后端轨迹优化与控制

简版主流程图：

![JPS MINCO MPC Simple](./jps_minco_mpc_pipeline_simple.png)

分层版流程图：

![JPS MINCO MPC Layered](./jps_minco_mpc_pipeline_layered.png)

这两张图表达的核心结论是：

1. 地面机器人总体导航仍然是 `2D` 栅格主链。
2. `2.5D` 前端负责高程、坡度、占有率和地形语义判断。
3. signed `Traversability ESDF` 负责 clearance、gradient 和 slope cost 支撑。
4. `JPS` 负责主搜索。
5. `MINCO + 独立 Yaw + 局部 B-spline repair` 负责把离散路径变成可执行轨迹。
6. 终局控制目标是 `SE2 MPC`。

## 1. 先说结论

2.5D `ESDF` 可以做两件事：

1. 识别高程变化、坡道、坎边、堆叠障碍等地形语义。
2. 将这些语义转成速度、加速度、清障距离和局部重拟合的约束。

它不需要把整个系统升级成真正的 3D 导航。

对地面机器人来说，更合理的选择是：

1. 总体导航仍坚持 `2D` 栅格主链。
2. `2.5D` 前端负责把坡度、占有率和语义风险变成速度规划输入。
3. 只有在长期版本 `V2` 中，才把地图后端升级成真正的 `3D ESDF`。

一句话概括：

`V1` 里，2.5D 语义前端服务 2D 地面导航主链。
`V2` 里，才考虑真正 3D ESDF。

## 2. V1 稳定比赛版

### 2.1 V1 的目标链

`Batch-LIWO / 里程计 -> 3D 点云投影 -> 2.5D Traversability + Slope -> 2D 语义 ESDF -> JPS -> MINCO(PRE + FINELY) -> 独立 Yaw -> 安全校验与局部重拟合 -> SE2 MPC -> chassis`

这条链是 `V1` 的比赛可用版本目标。

### 2.2 V1 中各层的职责

#### 状态估计层

1. 维持 `point_lio / lidar_odometry` 的稳定输出。
2. 保证时间戳和 TF 链稳定。

#### 2.5D 地形语义层

建议每个 `(x, y)` 栅格显式维护：

1. `z_min`
2. `z_max`
3. `height_diff`
4. `occupancy_ratio`
5. `ground_confidence`
6. `slope`
7. `slope_direction`
8. `roughness`
9. `dynamic_obstacle_confidence`
10. `traversable / occupied / unknown`

这层的任务不是“能不能走”的唯一判断，而是把地形语义转换成可规划、可控速、可解释的代价。

#### 2D 语义 ESDF 层

`TraversabilityEsdfProvider` 的职责是：

1. 输入二维可通行栅格和地形语义栅格。
2. 输出 signed distance。
3. 旁路输出 `slope_grid` 或等价坡度代价。
4. 给 `JPS / MINCO / MPC` 提供 `d(x,y)`、`grad d(x,y)` 和坡度相关代价。

这里要明确：

1. 主导航拓扑仍由 `2D` 搜索器在栅格图上完成。
2. ESDF 不负责把问题变成真正 3D 导航。
3. ESDF 负责的是在 2D 平面路径上叠加更丰富的地形语义。

#### 前端搜索层

`V1` 的最终目标前端是 `JPS`。
在进入 `JPS` 之前，可以先用 `A*` 把接口跑通，验证 ESDF、坡度和可通行定义是否稳定。

建议职责分工如下：

1. `traversability_grid` / 二值通行栅格：给 `JPS` 做主搜索。
2. `slope_grid`: 给速度/加速度自适应和 soft penalty 使用。
3. signed ESDF: 给目标点拉回、tie-break、局部修补、后端优化和控制器提供 clearance / gradient 信息。
4. 如果后续需要风险加权搜索，优先做 `JPS` 主搜索 + ESDF / slope 后验筛选与修补，不要一开始把 `JPS` 改造成重权图搜索器。

#### 轨迹优化层

`V1` 中建议新建独立 `minco_planner` 包，轨迹优化职责如下：

1. 路径表示改为 `MINCO`。
2. 实现 `PRE_OPTIMIZATION + FINELY_OPTIMIZATION`。
3. 在位置轨迹优化中接入 `slope` 软惩罚。
4. 保留独立 `Yaw` 规划。
5. 保留局部 B-spline repair 作为轻量碰撞修补层。

#### 控制层

`V1` 里可以先用 `MPPI` 做执行器，但目标控制层仍然是 `SE2 MPC`。

### 2.3 为什么 V1 仍然坚持 2D 栅格主链

对当前地面机器人，`2D` 栅格主链是最合理的选择：

1. 任务目标是地面可通行拓扑，不是空中或多层空间拓扑。
2. 主决策变量仍然是平面路径和沿路径的 `yaw / v / a` 分配。
3. 2.5D 高程/坡度/占有率分析已经足以表达坡道、矮墙、坎边、障碍堆和低置信区域。
4. 真正 3D 导航引入的复杂度，当前不会给 RMUC / 哨兵地面场景带来等比例收益。

所以 `V1` 的定位是：

`做一个由 2.5D 语义地图驱动的 2D 栅格导航系统`

### 2.4 2.5D ESDF 如何服务坡道速度优化

2.5D `ESDF` 不仅可以识别坡道，还应该参与加减速优化。

建议的分工如下：

1. `traversability_grid` 决定“这里能不能走”。
2. `slope_grid` 决定“这里该以多快的速度走”。
3. signed ESDF 决定“离障碍多近、梯度往哪边推”。

推荐最小实现：

1. 平坡：保持 nominal `v_max / a_max`。
2. 中等坡度：压低 `v_max`。
3. 大坡度或坡顶/坡底突变：同时压低 `v_max` 和 `a_max`。

这样做不需要把导航变成 3D，但会显著提升地面机器人在坡道环境下的轨迹可执行性。

### 2.5 V1 的明确边界

`V1` 明确不是 `3D` 导航版本。

它要做的是：

1. 用 `2.5D` 语义地图理解地形。
2. 用 `2D` 栅格主链完成导航。
3. 用 `JPS / MINCO / 独立 Yaw / 局部修补 / SE2 MPC` 提高比赛可用性和稳定性。

它不要做的是：

1. 把整个系统升级成真正 3D 导航。
2. 把悬挑、多层空间、桥下穿行当成当前主线问题。
3. 在 `V1` 阶段同时把地图后端和控制器一起大改。

## 3. V2 长期最终版

`V2` 的目标是把 `V1` 稳定下来的规划控制接口保留下来，再把地图后端升级成真正的 `3D ESDF`。

### 3.1 V2 的目标链

`Batch-LIWO / 里程计 -> 3D Occupancy / 3D ESDF -> JPS / 搜索前端 -> MINCO(PRE + FINELY) -> 独立 Yaw -> 安全校验与局部重拟合 -> SE2 MPC -> chassis`

### 3.2 V2 的核心变化

1. 地图后端从 2.5D 语义 ESDF 升级到真正 3D ESDF。
2. `JPS / MINCO / Yaw / MPC` 的上层接口尽量保持不变。
3. `V1` 中已经稳定的坡度、速度、修补和控制逻辑尽量复用。

### 3.3 V2 不是当前主线

`V2` 不是现在立刻开工的主线。
当前主线仍然是：

1. 先把 `V1` 做稳定。
2. 再考虑 `V2` 的 3D ESDF 后端升级。

## 4. 当前仓库的真实状态

当前仓库已经具备继续推进 `V1` 的基础：

1. `terrain_analysis_ext` 已经输出 `traversability_grid` 和多类地形语义栅格。
2. `TraversabilityEsdfProvider` 已经接入 signed ESDF。
3. `trajectory_optimizer` 和 `Nav2BSplineSmoother` 已经能消费 traversability ESDF。
4. 当前主链仍然是 Nav2，因此可以平滑过渡到 `V1`。

当前还缺的关键项：

1. `slope_grid` 和坡度阈值定义。
2. `minco_planner` 包。
3. `A* -> JPS` 的前端演进。
4. `MINCO + 独立 Yaw + 局部 B-spline repair` 的稳定实现。
5. `SE2 MPC` 控制器。

## 5. 近期实现顺序

### 5.1 V1 近期待办

1. 在 `terrain_analysis_ext` 侧补出 `slope_grid`。
2. 定义坡度如何影响 `v_max / a_max`。
3. 新建 `minco_planner` 包，先落 `A* + 独立 Yaw + 局部 B-spline repair`。
4. 在 `A*` 稳定后，把前端替换成 `JPS`。
5. 设计统一的 `ReferenceTrajectory` 消息或内部结构。

### 5.2 新对话起手任务

如果后续要开新对话继续项目优化，建议直接从下面 5 个任务开工：

1. `任务 1`
   在 `terrain_analysis_ext` 侧补出 `slope_grid`，并明确数据格式、阈值和可视化方式。
2. `任务 2`
   定义坡度如何影响 `v_max / a_max`，先把坡道速度自适应做成可配置规则。
3. `任务 3`
   新建 `minco_planner` 包骨架，只建目录、消息结构、配置和最小 launch。
4. `任务 4`
   在 `minco_planner` 中先实现 `grid_astar`，输出 `raw_path` 和调试 marker。
5. `任务 5`
   再实现 `yaw_spline_planner` 与 `local_collision_repair`，最后先接 `MPPI` 验证窄门和坡道。

### 5.3 V1 的阶段划分

#### 阶段 P0：补齐 2.5D 地形语义

目标：

1. 输出 `slope_grid`。
2. 明确坡度阈值和坡度分级。
3. 明确坡度如何影响 `traversability` 二值化。
4. 明确坡度如何影响 `v_max / a_max`。

#### 阶段 P1：新建 `minco_planner`

包内第一阶段建议只放这些模块：

1. `esdf_frontend_adapter`
2. `grid_astar`
3. `yaw_spline_planner`
4. `local_collision_repair`
5. `planner_debug_visualizer`
6. `trajectory_speed_adapter`

这个阶段的目标不是“立刻 MINCO 化”，而是先把 signed ESDF 驱动的自有规划链跑起来。

#### 阶段 P2：前端位置规划先用 `A*`

输入：

1. `TraversabilityEsdfProvider` 或等价的独立 ESDF 采样接口。
2. 当前起点、目标点。
3. clearance / risk / unknown 代价参数。

输出：

1. 离散 `path points`。
2. 每个点的累计弧长。
3. 初始时间分配。
4. 调试用 `raw_path`。

#### 阶段 P2.5：把前端替换成 `JPS`

切换条件建议是：

1. `traversability_grid` 的二值可通行定义已经稳定。
2. 目标点拉回和 unknown 策略已经稳定。
3. `A*` 版本已经能稳定穿过 `rmuc_2025` 窄门。
4. 你已经确认真正想让 `JPS` 吃的是哪一层搜索图，而不是临时混合各种 risk 权重。

切换后的职责：

1. `JPS` 负责全局或局部快速搜索。
2. signed ESDF 继续负责 clearance 判断、局部修补和后端优化。
3. `slope_grid` 继续负责速度/加速度自适应。
4. `Yaw` 规划、局部 B-spline repair、控制器接口保持不变。

#### 阶段 P3：独立 `Yaw` 规划

`Yaw` 只惩罚角速度和角加速度还不够，建议增加：

1. 转向电机转速上限。
2. 狭窄区域内的路径切向轻量参考。
3. 起点 yaw 边界项。

#### 阶段 P4：局部安全修补

建议按局部支撑语义实现：

1. 采样车体矩形 footprint。
2. 发现碰撞后，不做全局重算。
3. 只动碰撞段影响到的 3 到 4 个局部控制点。
4. 沿安全方向拖动一小步。
5. 重新局部拟合并再次校验。

#### 阶段 P5：先下发给 `MPPI`

`MPPI` 先作为执行器，目标仍然是过渡到 `SE2 MPC`。

#### 阶段 P6：V1 稳定后再上 `MINCO` 和 `SE2 MPC`

推荐顺序：

1. 先补齐 `slope_grid`。
2. 实现 `A* -> 局部修补 -> MPPI`。
3. 跑通窄门、贴边、S 弯和坡道。
4. 切到 `JPS`。
5. 固化 signed ESDF 与 `slope_grid` 的接口和调试指标。
6. 再把 `MINCO PRE + FINELY` 接到 `minco_planner`。
7. 最后再用 `SE2 MPC` 替换执行器。

## 6. 旧补丁和旧叙述的处理原则

为了让文档真正服务于当前项目优化，下面这些内容不再作为主线叙述：

1. 把 fake costmap ESDF 当主线的旧描述。
2. 把 Nav2 的旧参数调优写成长期主方向的叙述。
3. 把过渡链和历史对照写成与当前目标同等重要的内容。

文档只保留两类内容：

1. `V1` 现在应该做什么。
2. `V2` 未来应该怎么升级。

## 7. 结论

当前最合理的推进方式是：

1. 先按 `V1 稳定比赛版` 做。
2. 把 `2.5D` 语义地图、坡度、速度自适应、`JPS`、`MINCO`、`SE2 MPC` 跑稳定。
3. 等比赛版稳定后，再升级到 `V2 长期最终版` 的真正 `3D ESDF` 后端。

如果一句话总结：

`V1 解决比赛能用，V2 解决体系最终形态。`
