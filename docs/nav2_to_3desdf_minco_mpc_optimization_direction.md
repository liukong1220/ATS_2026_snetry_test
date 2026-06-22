# 从 2.5D 语义 ESDF 到稳定比赛版与长期最终版导航主链

更新时间：2026-06-22

本文档只保留两条主线：

1. `V1 稳定比赛版`
   `2.5D 语义地图 + 2D 栅格导航主链 + RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 局部修补 + SE2 MPC`
2. `V2 长期最终版`
   `3D ESDF + 2D / 2.5D 地面导航主链 + JPS + MINCO + 独立 Yaw + 轮廓安全校验 + 局部重拟合 + SE2 MPC`

当前优化工作优先服务于 `V1`。`V2` 是在 `V1` 稳定后再推进的长期目标。

## 0. 目标流程图

下面两张图对应 `V1` 的目标工程形态，核心思想与 `docs/中科大哨兵2025技术报告.pdf` 一致，但这里进一步明确：

1. 2.5D 语义建图仍然服务 `2D` 地面导航主拓扑
2. RC-ESDF 是局部滚动的语义距离场，而不是把系统升级成真正 3D 导航
3. 轨迹侧的终局不再是“B 样条平滑 + MPPI”本身，而是 `A* / JPS -> MINCO -> 独立 Yaw -> 轮廓安全校验 -> 局部重拟合 -> SE2 MPC`

简版主流程图：

![JPS MINCO MPC Simple](./jps_minco_mpc_pipeline_simple.png)

分层版流程图：

![JPS MINCO MPC Layered](./jps_minco_mpc_pipeline_layered.png)

这两张图表达的核心结论是：

1. 地面机器人总体导航仍然是 `2D` 栅格主链。
2. `2.5D` 前端负责高程、坡度、占有率和地形语义判断。
3. signed `RC-ESDF` 负责 clearance、gradient 和局部本体安全查询支撑。
4. `A* / JPS` 负责主搜索。
5. `MINCO + 独立 Yaw + 轮廓安全校验 + 局部重拟合` 负责把离散路径变成可执行轨迹。
6. 终局控制目标是 `SE2 MPC`。

## 1. 先说结论

对当前仓库，最合理的路线不是一步跳到“所有模块同时重写”，而是分成两个清晰层次：

1. 近期比赛优化目标：
   `LBFGS-RC-ESDF + MPPI`
2. `V1` 最终比赛版目标：
   `RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 轮廓安全校验 + 局部重拟合 + SE2 MPC`

原因很明确：

1. 当前仓库已经有 `TraversabilityEsdfProvider + Nav2BSplineSmoother + MPPI` 的稳定过渡主链。
2. 先把当前 signed Traversability ESDF 演进为更强的 `RC-ESDF-lite`，可以最快获得窄门、贴边、高速转角收益。
3. 直接同时替换搜索器、轨迹优化器和执行器，会让比赛期变量过多，调试成本过高。

一句话概括：

`近期先做 LBFGS-RC-ESDF + MPPI，最终收敛到 RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 局部修补 + SE2 MPC。`

## 2. V1 稳定比赛版

### 2.1 V1 的最终目标链

`Batch-LIWO / 里程计 -> 3D 点云投影 -> 2.5D Traversability + Slope -> 2D 语义 RC-ESDF -> A* / JPS 搜索 -> MINCO 优化 2D 质心位置 + 时间 -> 独立 Yaw 规划（5次 B-spline，限制转向速率） -> 采样车体轮廓并反算 B-spline 控制点做凸包安全校验 -> 若碰撞则仅局部拖动控制点重拟合 -> 输出可执行参考轨迹 -> 近端执行器（过渡期 MPPI，最终 SE2 MPC） -> chassis`

这条链是 `V1` 的比赛可用版本最终目标。

### 2.2 V1 中各层的职责

#### 状态估计层

1. 维持 `point_lio / lidar_odometry` 的稳定输出。
2. 保证时间戳和 TF 链稳定。
3. 为局部滚动 RC-ESDF、搜索器与控制器提供一致的姿态参考。

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

#### 2D 语义 RC-ESDF 层

`V1` 中的 ESDF 层应从当前 `TraversabilityEsdfProvider` 逐步演进为：

`RC-ESDF-lite`

这里的 `RC` 指的是：

1. 局部滚动窗口
2. 更强调围绕机器人当前位置的高频查询
3. 优先服务轨迹优化、轮廓校验与控制器，而不是做全局 3D 拓扑表达

这一层的职责是：

1. 输入二维可通行栅格和地形语义栅格。
2. 输出 signed distance。
3. 输出稳定 `grad d(x, y)`。
4. 旁路输出 `slope_grid` 或等价坡度代价。
5. 为 `A* / JPS`、`MINCO`、轮廓安全校验和 `SE2 MPC` 提供统一 clearance 接口。

这里要明确：

1. 主导航拓扑仍由 `2D` 搜索器在栅格图上完成。
2. RC-ESDF 不负责把问题变成真正 3D 导航。
3. RC-ESDF 负责的是在 `2D` 平面路径上叠加更丰富的地形语义，并强化局部本体可执行性建模。

#### 前端搜索层

`V1` 的前端搜索最终目标是：

`A* / JPS`

建议分两步推进：

1. 先用 `A*` 把接口跑通。
2. 再切换到 `JPS` 作为最终比赛版主搜索器。

建议职责分工如下：

1. `traversability_grid` / 二值通行栅格：给 `A* / JPS` 做主搜索。
2. `slope_grid`：给速度/加速度自适应和软惩罚使用。
3. signed `RC-ESDF`：给 tie-break、后验筛选、局部修补、后端优化和控制器提供 clearance / gradient 信息。
4. 如果后续要做风险加权搜索，优先做 `JPS` 主搜索 + ESDF / slope 后验筛选与修补，不要一开始把 `JPS` 改造成重权图搜索器。

#### 轨迹优化层

`V1` 中建议新建独立 `minco_planner` 包，轨迹优化职责如下：

1. 搜索输出先变成离散 `raw_path`。
2. 用 `MINCO` 优化 `2D` 质心位置轨迹。
3. 在 `MINCO` 中同时完成时间分配。
4. 在位置轨迹优化中接入 clearance、曲率和 `slope` 软惩罚。
5. 输出可给 `MPPI / SE2 MPC` 直接消费的参考轨迹结构。

这一层的目标不是单纯“让路径更圆”，而是：

1. 提高高速下的可跟踪性。
2. 提高大角度转向时的轨迹几何质量。
3. 提高极狭窄通道中的轮廓通过成功率。

#### 独立 Yaw 规划层

`Yaw` 规划建议保持独立层，不与位置轨迹一次性强耦合成一个大优化问题。

推荐形式：

`5次 B-spline Yaw Planner`

这一层至少需要考虑：

1. `yaw rate` 限制
2. 必要时的 `yaw acceleration` 限制
3. 起点 yaw 边界项
4. 狭窄区域内的路径切向轻量参考

保留独立 `Yaw` 的原因是：

1. 更适合比赛期分层调试。
2. 更利于和 `MINCO`、轮廓安全校验分开定位问题。
3. 更便于后续与 `SE2 MPC` 对接。

#### 车体轮廓安全校验层

在 `MINCO + 独立 Yaw` 之后，应新增明确的：

`车体轮廓采样 + 凸包安全校验`

推荐流程：

1. 沿轨迹采样姿态。
2. 根据机器人车体轮廓生成离散 footprint。
3. 将轨迹与 footprint 映射回局部控制点区段。
4. 基于 RC-ESDF 做 clearance 与碰撞检查。
5. 做凸包安全性判断，而不是只看中心点。

这层直接服务于：

1. 极窄通道
2. 贴边过门
3. 大角度转向时的轮廓扫掠安全

#### 局部碰撞重拟合层

若安全校验发现碰撞，不建议整条轨迹整体重算，建议采用：

`Local Collision Repair`

它的职责应明确为：

1. 只在碰撞段附近调整控制点。
2. 尽量保持未碰撞区段不动。
3. 优先修补 clearance 问题。
4. 用轻量局部重拟合代替全局重优化。

推荐最小实现：

1. 采样车体矩形 footprint。
2. 发现碰撞后，不做全局重算。
3. 只动碰撞段影响到的局部控制点。
4. 沿安全方向拖动一小步。
5. 重新局部拟合并再次校验。

#### 控制层

控制层需要明确区分：

1. 过渡执行器：`MPPI`
2. 最终执行器：`SE2 MPC`

`V1` 中可以继续保留 `MPPI` 作为验证执行器，但最终比赛目标控制层仍应收敛到 `SE2 MPC`。

### 2.3 为什么 V1 仍然坚持 2D 栅格主链

对当前地面机器人，`2D` 栅格主链仍然是最合理的选择：

1. 任务目标是地面可通行拓扑，不是空中或多层空间拓扑。
2. 主决策变量仍然是平面路径与沿路径的 `yaw / v / a` 分配。
3. `2.5D` 高程、坡度、占有率分析已经足以表达坡道、矮墙、坎边、障碍堆和低置信区域。
4. RC-ESDF、MINCO、独立 Yaw、轮廓安全校验和局部修补都可以在 `2D` 主链上有效工作。
5. 真正 3D 导航引入的复杂度，当前不会给 RMUC / 哨兵地面场景带来等比例收益。

所以 `V1` 的定位是：

`做一个由 2.5D 语义地图驱动的 2D 栅格导航系统`

### 2.4 RC-ESDF 如何服务高速、大角度转向和极窄通道

RC-ESDF 相比当前“仅给平滑器提供点式 clearance 代价”的做法，更适合服务下面三类比赛问题：

1. 高速场景
   clearance、gradient 和坡度代价可以更直接地进入时间分配与控制器。
2. 大角度转向
   独立 `Yaw` 与轮廓扫掠安全校验可以避免仅看质心路径导致的姿态风险。
3. 极窄通道
   footprint-aware 的局部安全检查比仅看中心线 clearance 更关键。

推荐分工如下：

1. `traversability_grid` 决定“这里能不能走”。
2. `slope_grid` 决定“这里该以多快的速度走”。
3. RC-ESDF 决定“离障碍多近、梯度往哪边推、局部轮廓是否还能过”。

### 2.5 V1 的明确边界

`V1` 明确不是 `3D` 导航版本。

它要做的是：

1. 用 `2.5D` 语义地图理解地形。
2. 用 `2D` 栅格主链完成导航。
3. 用 `RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 局部修补 + SE2 MPC` 提高比赛可用性和稳定性。

它不要做的是：

1. 把整个系统升级成真正 3D 导航。
2. 把悬挑、多层空间、桥下穿行当成当前主线问题。
3. 在 `V1` 阶段同时把地图后端和控制器做成一个难以回退的大一统重写工程。

## 3. V2 长期最终版

`V2` 的目标不是推翻 `V1` 的比赛链路，而是在保留 `V1` 全部规划控制主链结构的前提下，把地图与安全查询后端升级成真正的 `3D ESDF`。

也就是说，`V2` 仍然沿用下面这些关键设计：

1. 主导航仍然服务地面机器人任务
2. 搜索器仍然是 `A* / JPS` 这一类前端
3. 轨迹优化仍然是 `MINCO`
4. 姿态规划仍然保留独立 `Yaw`
5. 仍然保留车体轮廓安全校验与局部重拟合
6. 最终执行器仍然是 `SE2 MPC`

`V2` 真正变化的核心，是把 `V1` 中的 `2.5D` 语义 RC-ESDF 后端，升级为更完整的 `3D Occupancy / 3D ESDF` 后端。

### 3.1 V2 的目标链

`Batch-LIWO / 里程计 -> 3D Occupancy / 3D ESDF -> 2D / 2.5D 可通行投影与地面语义抽取 -> A* / JPS 搜索 -> MINCO 优化 2D 质心位置 + 时间 -> 独立 Yaw 规划（5次 B-spline，限制转向速率） -> 采样车体轮廓并反算控制点做凸包安全校验 -> 若碰撞则仅局部拖动控制点重拟合 -> 输出可执行参考轨迹 -> SE2 MPC -> chassis`

这条链与 `V1` 的关系应理解为：

1. `V1` 解决的是 `2.5D` 语义地图驱动下的比赛可用主链
2. `V2` 解决的是在相同规划控制结构下，把后端地图能力升级为真正 `3D ESDF`
3. `V2` 的上层搜索、轨迹优化、姿态规划、安全校验和控制器接口尽量不要与 `V1` 分裂成两套体系

### 3.2 V2 中各层的职责

#### 3D 地图后端层

`V2` 与 `V1` 最大的差异在这里。

这一层的职责是：

1. 维护真正的 `3D Occupancy`
2. 构建真正的 `3D ESDF`
3. 为上层提供比 `V1` 更准确的 clearance 与几何关系
4. 在存在堆叠障碍、悬挑、复杂坡体和多高度结构时，提供比 `2.5D` 更稳定的后端几何支撑

但要明确：

1. `V2` 的 3D ESDF 后端不等于把整个导航任务变成空中机器人式 3D 路径规划
2. 对当前地面机器人，主决策变量仍然主要是平面质心轨迹、沿路径姿态与速度分配

#### 地面语义抽取层

即使进入 `V2`，仍然建议保留一个显式的地面任务抽取层，把 `3D Occupancy / 3D ESDF` 转换成：

1. 地面可通行区域
2. 地形风险
3. 坡度语义
4. 可投影到 `2D / 2.5D` 搜索图上的通行定义

原因是：

1. 当前比赛任务本质仍然是地面机器人导航
2. 上层搜索器与轨迹优化器继续围绕地面任务定义即可
3. 这样可以最大限度复用 `V1` 已稳定的接口

#### 搜索前端层

`V2` 仍然建议沿用 `A* / JPS` 这一类前端搜索框架。

区别在于：

1. 搜索图来自 `3D ESDF` 支撑下的更强地面语义抽取结果
2. tie-break、后验筛选与局部风险评估可以利用更准确的后端 clearance
3. 上层搜索接口尽量与 `V1` 保持一致

#### 轨迹优化层

`V2` 中的轨迹优化层仍然建议保持：

`MINCO 优化 2D 质心位置 + 时间`

原因是：

1. 即便地图后端升级为 `3D ESDF`，当前地面机器人比赛中的主轨迹变量仍然主要是平面轨迹
2. `MINCO`、独立 `Yaw`、轮廓安全校验、局部重拟合这套结构在 `V1` 中若已稳定，没有必要在 `V2` 中重新改成另一套上层轨迹体系

#### 独立 Yaw 与轮廓安全层

`V2` 中仍应保留：

1. 独立 `Yaw` 规划
2. 车体轮廓采样
3. 凸包安全校验
4. 局部碰撞重拟合

与 `V1` 的差异主要在于：

1. clearance 查询会来自更强的 `3D ESDF` 后端
2. 某些复杂结构附近的轮廓风险评估会更稳定

#### 控制层

`V2` 的最终执行器仍然保持 `SE2 MPC`。

原因是：

1. 当前比赛任务仍然是地面机器人执行问题
2. 即使地图后端升级为 `3D ESDF`，控制层的主状态仍可保持在 `SE2`
3. 这有助于保持 `V1` 与 `V2` 的执行接口连续
### 3.3 V2 的核心变化

1. 地图后端从 `2.5D` 语义 RC-ESDF 升级到真正 `3D ESDF`。
2. 在 `3D ESDF` 后端之上继续抽取适合地面机器人的 `2D / 2.5D` 搜索与优化语义。
3. `A* / JPS / MINCO / 独立 Yaw / 轮廓安全校验 / 局部修补 / SE2 MPC` 的上层接口尽量保持不变。
4. `V1` 中已经稳定的坡度、速度、修补和控制逻辑尽量复用。

### 3.4 V2 不是当前主线

`V2` 不是现在立刻开工的主线。
当前主线仍然是：

1. 先把 `V1` 做稳定。
2. 先把当前过渡主链收敛为 `LBFGS-RC-ESDF + MPPI`。
3. 再把它推进到 `RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 局部修补 + SE2 MPC`。
4. 最后再考虑 `V2` 的 `3D ESDF` 后端升级。

## 4. 当前仓库的真实状态

当前仓库已经具备继续推进 `V1` 的基础：

1. `terrain_analysis_ext` 已经输出 `traversability_grid` 与多类地形语义栅格。
2. `slope_grid`、`slope_band_grid`、坡度阈值与可视化已经补齐。
3. `TraversabilityEsdfProvider` 已经能输出 signed distance 与 gradient。
4. `trajectory_optimizer` 与 `Nav2BSplineSmoother` 已经能消费 traversability ESDF。
5. 当前主链仍然是 `SmacPlannerHybrid -> Nav2BSplineSmoother -> MPPI`。
6. 当前平滑器已在用连续优化 / `LBFGS` 风格参数链，具备继续向 `LBFGS-RC-ESDF + MPPI` 收敛的基础。

当前还缺的关键项：

1. `slope_grid` 对 `v_max / a_max` 的正式规则化接入。
2. 当前 `TraversabilityEsdfProvider` 向 `RC-ESDF-lite` 的演进。
3. `minco_planner` 包骨架。
4. `A* -> JPS` 的前端演进。
5. `MINCO + 独立 Yaw + 轮廓安全校验 + Local Collision Repair` 的稳定实现。
6. `ReferenceTrajectory` 消息或等价内部结构。
7. `SE2 MPC` 控制器。

## 5. 推荐实现顺序

### 5.1 路线选择

当前阶段推荐同时保留两个层级的目标：

1. 近期比赛优化目标：
   `LBFGS-RC-ESDF + MPPI`
2. `V1` 最终比赛目标：
   `RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 轮廓安全校验 + Local Collision Repair + SE2 MPC`

推荐原因：

1. 先改 ESDF 表达和局部可执行性建模，收益最快。
2. 先保留 `MPPI`，可以降低控制层同时更换带来的调试风险。
3. 等 RC-ESDF、搜索器、轨迹和安全校验都跑稳后，再切 `SE2 MPC` 最合理。

### 5.2 新对话起手任务

如果后续要开新对话继续项目优化，建议直接从下面 8 个任务开工：

1. `任务 1`
   将当前 `TraversabilityEsdfProvider` 演进为 `RC-ESDF-lite`，明确 rolling window、查询接口、`slope_grid` 输入和 footprint-aware 扩展方向。
2. `任务 2`
   定义 `slope_grid` 如何影响 `v_max / a_max`，把坡道速度自适应做成可配置规则，并接入现有 profile / governor 链。
3. `任务 3`
   新建 `minco_planner` 包骨架，先建目录、配置、最小 launch、调试 marker 和 `ReferenceTrajectory` 结构。
4. `任务 4`
   在 `minco_planner` 中先实现 `grid_astar`，输出 `raw_path`、累计弧长、初始时间分配和调试 marker。
5. `任务 5`
   实现 `minco_trajectory_optimizer`，先完成 `2D` 质心位置 + 时间分配的最小 `MINCO` 闭环。
6. `任务 6`
   实现 `yaw_spline_planner`，使用 `5次 B-spline` 规划独立 `Yaw`，并限制 `yaw rate`。
7. `任务 7`
   实现 `footprint_safety_checker` 与 `local_collision_repair`，完成轮廓安全校验和局部控制点重拟合。
8. `任务 8`
   先接 `MPPI` 验证窄门、贴边、S 弯、坡道和高速转角，再规划 `SE2 MPC` 替换。

### 5.3 V1 的阶段划分

#### 阶段 P0：固化 2.5D 地形语义与坡度速度规则

目标：

1. 固化 `slope_grid` 与 `slope_band_grid`。
2. 明确坡度阈值与坡度分级。
3. 明确坡度如何影响 `traversability` 二值化。
4. 明确坡度如何影响 `v_max / a_max`。

#### 阶段 P1：将当前 ESDF 演进为 `RC-ESDF-lite`

目标：

1. 保留当前 `TraversabilityEsdfProvider` 的基础接口。
2. 强化局部滚动特性。
3. 稳定 `d(x, y)` 与 `grad d(x, y)` 查询。
4. 为后续 footprint-aware 校验预留接口。
5. 保持当前 `LBFGS + MPPI` 过渡主链可继续运行。

#### 阶段 P2：新建 `minco_planner`

包内第一阶段建议只放这些模块：

1. `rc_esdf_adapter`
2. `grid_astar`
3. `minco_trajectory_optimizer`
4. `yaw_spline_planner`
5. `footprint_safety_checker`
6. `local_collision_repair`
7. `planner_debug_visualizer`
8. `trajectory_bridge_mppi`

这个阶段的目标不是“立刻全部终局化”，而是先把自有规划链骨架和接口跑起来。

#### 阶段 P3：前端位置规划先用 `A*`

输入：

1. `RC-ESDF-lite` 或等价独立 clearance 采样接口。
2. 当前起点、目标点。
3. `traversability_grid`。
4. clearance / risk / unknown 代价参数。

输出：

1. 离散 `raw_path`。
2. 每个点的累计弧长。
3. 初始时间分配。
4. 调试用 `raw_path` marker。

#### 阶段 P4：`MINCO` 优化 2D 质心位置 + 时间

目标：

1. 输入 `raw_path`。
2. 输出连续 `2D` 质心轨迹。
3. 同步完成时间分配。
4. 接入 clearance、曲率和 `slope` 软惩罚。
5. 形成后续 `Yaw` 规划与执行器都能消费的参考轨迹。

#### 阶段 P5：独立 `Yaw` 规划

目标：

1. 基于位置轨迹生成独立 `Yaw` 参考。
2. 使用 `5次 B-spline`。
3. 限制 `yaw rate`。
4. 必要时限制 `yaw acceleration`。
5. 在狭窄区域对路径切向引入轻量参考。

#### 阶段 P6：轮廓安全校验与局部碰撞重拟合

目标：

1. 沿轨迹采样车体 footprint。
2. 做凸包安全校验。
3. 若碰撞，只在局部控制点区段重拟合。
4. 避免一旦发现碰撞就整条轨迹全局重算。

#### 阶段 P7：把前端替换成 `JPS`

切换条件建议是：

1. `traversability_grid` 的二值可通行定义已经稳定。
2. 目标点拉回和 unknown 策略已经稳定。
3. `A*` 版本已经能稳定穿过 `rmuc_2025` 窄门。
4. RC-ESDF、`MINCO`、`Yaw` 和局部修补接口已经定型。

切换后的职责：

1. `JPS` 负责快速主搜索。
2. signed `RC-ESDF` 继续负责 clearance 判断、局部修补和后端优化。
3. `slope_grid` 继续负责速度/加速度自适应。
4. `Yaw` 规划、局部修补、控制器接口保持不变。

#### 阶段 P8：先下发给 `MPPI`

`MPPI` 先作为执行器，目标仍然是过渡到 `SE2 MPC`。

这个阶段要重点验证：

1. 窄门通过
2. 贴边走廊
3. 高速直道转急弯
4. `S` 弯连续性
5. 坡道速度自适应

#### 阶段 P9：V1 稳定后再上 `SE2 MPC`

推荐顺序：

1. 先把 `RC-ESDF-lite + A* / JPS + MINCO + 独立 Yaw + Local Collision Repair` 跑稳定。
2. 再把参考轨迹接口固化为 `SE2 MPC` 可直接消费的结构。
3. 最后再用 `SE2 MPC` 替换 `MPPI` 作为最终执行器。

## 6. 旧叙述的处理原则

为了让文档真正服务于当前项目优化，下面这些内容不再作为主线叙述：

1. 把 fake costmap ESDF 当主线的旧描述。
2. 把单纯 Nav2 参数调优写成长期主方向的叙述。
3. 把历史对照链与当前最终目标写成同等重要。
4. 把“B 样条平滑 + MPPI”误写成终局架构。

文档只保留三类内容：

1. 当前项目的真实过渡状态是什么。
2. `V1` 现在应该做什么。
3. `V2` 未来应该怎么升级。

## 7. 结论

当前最合理的推进方式是：

1. 先按 `V1 稳定比赛版` 做。
2. 先把当前主链收敛为 `LBFGS-RC-ESDF + MPPI`。
3. 再推进到 `RC-ESDF + A* / JPS + MINCO + 独立 Yaw + 轮廓安全校验 + Local Collision Repair + SE2 MPC`。
4. 等比赛版稳定后，再升级到 `V2` 的真正 `3D ESDF` 后端。

如果一句话总结：

`V1 解决比赛能用与高速窄通道可执行性，V2 解决体系最终形态。`
