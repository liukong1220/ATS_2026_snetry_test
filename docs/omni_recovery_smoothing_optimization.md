# 全向导航优化接力文档

更新时间：2026-05-06

这份文档记录当前三阶段优化的实际落地状态，避免后续对话把“仅可视化验证”和“已接入 controller 主链”混在一起。

## 1. 当前阶段状态

### 第一阶段：`/plan -> 平滑参考路径 -> 高密度采样`

状态：已完成

已经落地的内容：

1. 独立 B 样条风格路径优化算法
2. 高密度路径重采样
3. 保形约束，避免平滑后过度偏离原始走廊
4. 可视化旁路输出

### 第二阶段：MPPI 真正跟踪平滑后的参考路径

状态：已完成第一版接入

已经落地的内容：

1. `trajectory_optimizer` 不再只是旁路节点
2. 同一套平滑算法已经封装为 Nav2 smoother plugin
3. BT 主链已恢复为：
   `ComputePath -> SmoothPath -> FollowPath`
4. `FollowPath` 现在会真正吃到 `bspline_smoother` 输出后的 path
5. MPPI 参数已做一轮偏保守的拐角/障碍收敛调节

### 第三阶段：ESDF obstacle cost

状态：未开始

计划仍然是：

1. 在轨迹优化中加入连续障碍代价
2. 优先形式：
   `max(0, safe_dist - distance)^2`
3. 让第一层参考路径本身远离障碍，而不是主要靠 MPPI 在离散 costmap 上补救

## 2. 当前真实链路

现在已经分成两条并行链：

### 主控制链

`ComputePath -> SmoothPath(bspline_smoother) -> FollowPath(MPPI)`

这条链会真正影响 MPPI 控制结果。

### 旁路可视化链

`plan_raw_visual -> smoothed_path_visual`

这条链只是为了在 RViz 中继续对比：

1. planner 原始路径
2. B 样条平滑后的高密度路径

它不再参与 controller 输入。

## 3. 已落地代码位置

### 3.1 B 样条算法核心

- [bspline_path_optimizer.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/bspline_path_optimizer.hpp)
- [bspline_path_optimizer.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/bspline_path_optimizer.cpp)

当前能力：

1. 清洗过密路径点
2. 稀疏控制点提取
3. cubic B-spline 风格插值
4. 高密度等弧长输出
5. 横向偏差夹紧

### 3.2 可视化旁路节点

- [trajectory_optimizer_node.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/trajectory_optimizer_node.hpp)
- [trajectory_optimizer_node.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/trajectory_optimizer_node.cpp)

当前行为：

1. 订阅 `plan_raw_visual`
2. 发布 `smoothed_path_visual`

### 3.3 Nav2 smoother plugin

- [nav2_bspline_smoother.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/nav2_bspline_smoother.hpp)
- [nav2_bspline_smoother.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/nav2_bspline_smoother.cpp)
- [trajectory_optimizer_plugins.xml](../src/pb2025_sentry_nav/trajectory_optimizer/trajectory_optimizer_plugins.xml)

当前行为：

1. 被 `smoother_server` 动态加载
2. 在 `SmoothPath` action 中直接处理 planner 输出路径
3. 把平滑后的 path 返回给 BT blackboard，再交给 `FollowPath`

## 4. 第二阶段具体接法

### 4.1 BT 已接回 SmoothPath 主链

已更新：

- [navigate_to_pose_w_replanning_and_recovery.xml](../src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)
- [navigate_through_poses_w_replanning_and_recovery.xml](../src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml)

现在：

1. `ComputePath*` 输出到 `{path_raw}`
2. `SmoothPath` 用 `smoother_id="bspline_smoother"`
3. 平滑结果写回 `{path}`
4. `FollowPath` 追踪 `{path}`

### 4.2 loopback 的 smoother_server 已切到我们自己的插件

已更新：

- [loopback_sim/nav2_params.yaml](../src/loopback_sim/params/nav2_params.yaml)

当前配置：

1. `bspline_smoother`
2. `fallback_smoother`

这样即使后续我们继续试 ESDF，也只需要在 smoother 层继续扩展，不用再改 controller 接口。

### 4.3 loopback launch 已保留可视化旁路

已更新：

- [loopback_navigation.launch.py](../src/pb2025_sentry_bringup/launch/loopback_navigation.launch.py)

当前行为：

1. `trajectory_optimizer_node` 仍然启动
2. 但它被 remap 到：
   - `plan_raw_visual`
   - `smoothed_path_visual`
3. 因此不会和 Nav2 `SmoothPath` action 混 topic

## 5. 这次发现并修掉的问题

### 5.1 之前看不到青色路径的真正原因

之前你在：

`ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py use_rviz:=True publish_referee_inputs:=True`

看不到青线，不是 RViz 配置坏了，而是：

1. `loopback_vision_test`
2. -> `loopback_decision_sim`
3. -> `loopback_navigation`

这条链原本根本没起 `trajectory_optimizer`

所以：

1. RViz 里有显示项
2. 但没有 `/smoothed_path` 发布者

这个问题已经修掉。

### 5.2 第二阶段初次接入时的 lifecycle 卡死

一开始在 `loopback_navigation.launch.py` 里把 `trajectory_optimizer` 错误地加进了 `lifecycle_nodes`。

但它是普通 node，不是 lifecycle node，所以会卡在：

`Waiting for service trajectory_optimizer/get_state...`

这个问题也已经修掉。

## 6. 本轮 MPPI 参数调整

当前只先动了 loopback 仿真链，目的是减轻：

1. 转角 shortcut
2. 穿进 inflation layer
3. 角点附近过于激进的“切弯”

主要方向：

1. `batch_size` 略增
2. `temperature` 略增
3. `gamma` 略增
4. `vx_std / vy_std / wz_std` 降低
5. `PathAlignCritic` 降权并缩短前视
6. `PathFollowCritic` 略增强
7. `PathAngleCritic` 增强并收紧最大允许夹角
8. `ObstaclesCritic` 的 `repulsion_weight / critical_weight / collision_margin_distance` 都提高

这组改动的意图不是让 MPPI “更聪明”，而是先让它在平滑参考路径接入后，不要太爱沿对角 shortcut 去切膨胀层。

## 7. 当前观察重点

现在最应该观察的是：

1. MPPI 角点处是否比以前更少切进 inflation layer
2. `transformed_global_plan` 是否比以前更贴近平滑参考线
3. 是否出现新的副作用：
   - 转弯变钝
   - 速度下降过多
   - 终点附近犹豫

## 8. 已完成验证

已完成：

1. `trajectory_optimizer` 单包重新编译通过
2. `bspline_smoother` 被 `smoother_server` 正常加载
3. loopback 主链能够正常进入 active
4. `controller_server` 持续收到新 path
5. `Goal succeeded` 正常出现
6. 旁路 topic `plan_raw_visual` / `smoothed_path_visual` 存在

说明：

第二阶段已经不是“只改了配置”，而是已经真实跑通。

## 9. 现在还没做的事

1. 没有对 reality 参数做同样级别的第二阶段切换
2. 没有系统性 sweep MPPI 参数
3. 没有引入 ESDF obstacle cost
4. 没有把平滑器做成“按局部代价场自适应收缩偏差”的版本

## 10. 下一步建议

最推荐的下一步是：

1. 继续在 loopback 下观察第二阶段效果
2. 再做一轮 MPPI 参数收敛
3. 等“不会明显切进 inflation layer”之后，再开第三阶段 ESDF obstacle cost

如果下一轮继续，我建议直接做：

1. 对 `PathAlign / PathAngle / ObstaclesCritic` 做更细一轮 sweep
2. 或者开始给 `bspline_smoother` 加第二阶段 ESDF obstacle term
