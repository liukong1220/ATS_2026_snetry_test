# 全向导航优化接力文档

更新时间：2026-05-06

## 1. 当前阶段状态

### 第一阶段：B 样条平滑 + 高密度路径点

状态：已完成

已落地：

1. 独立 B 样条风格路径优化算法
2. 高密度路径重采样
3. 可视化旁路输出 `smoothed_path_visual`

### 第二阶段：把平滑后的参考路径真正接入 MPPI

状态：已完成第一版，并已重新修复接入 loopback

已落地：

1. `trajectory_optimizer` 已封装成 Nav2 smoother plugin
2. BT 主链已恢复为 `ComputePath -> SmoothPath -> FollowPath`
3. loopback 当前已真正跟踪平滑后的 path
4. `trajectory_profile` 已作为正式接口发布
5. `trajectory_speed_governor` 已基于 profile 对 controller 输出做二次限速
6. 实车 `reality` 参数和 `navigation_launch.py` 已同步到同一套完整链

### 第三阶段：ESDF obstacle cost

状态：未开始

## 2. 当前核心结论

“第二个弯进入膨胀层”不单单是 MPPI 的问题。

当前排查结论是：

1. planner、smoother、controller 三层都会共同影响
2. loopback 原来使用 `NavfnPlanner` 时，第一层路径更容易贴边
3. 纯几何 B 样条会在角点进一步往内抹
4. MPPI 会在第二个弯继续沿 shortcut 倾向切弯

所以这不是单层问题，而是：

1. planner 可能先贴边
2. smoother 可能再抹角
3. controller 最后把它放大

## 3. 当前 `trajectory_optimizer` 能力

### 3.1 规范化三次 B 样条表示

现在已经有：

- `CubicBSpline2D`
- `getPoint(s)`
- `getFirstDerivative(s)`
- `getSecondDerivative(s)`
- `getCurvature(s)`

位置：

- [bspline_path_optimizer.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/bspline_path_optimizer.hpp)
- [bspline_path_optimizer.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/bspline_path_optimizer.cpp)

### 3.2 曲率约束

当前已加：

1. `curvature_limit`
2. `curvature_weight`
3. `curvature_refinement_iterations`
4. `curvature_refinement_gain`

目前做法是：

1. 先生成 dense B 样条 path
2. 计算离散一阶/二阶导
3. 用曲率公式计算 `kappa`
4. 对超出 `kappa_max` 的点做 refinement

这已经是曲率约束雏形。

### 3.3 时间参数化

当前已加：

1. 曲率限速
2. forward/backward acceleration limiting
3. velocity smoothing
4. `TrajectoryProfile2D`

profile 每个采样点包含：

1. `s`
2. `t`
3. `point`
4. `first_derivative`
5. `second_derivative`
6. `curvature`
7. `speed_limit`
8. `speed`
9. `acceleration`

## 4. `trajectory_profile` 已成为正式接口

当前已定义消息：

- [TrajectoryProfileMsg.msg](../src/pb2025_sentry_nav/sp_msgs/msg/TrajectoryProfileMsg.msg)
- [TrajectoryProfilePoint.msg](../src/pb2025_sentry_nav/sp_msgs/msg/TrajectoryProfilePoint.msg)

当前存在两条 profile 输出链：

1. `trajectory_optimizer_node -> /trajectory_profile_visual`
2. `nav2_bspline_smoother -> /trajectory_profile`

这意味着：

1. 在 BT / smoother server 外已经有正式 profile 接口
2. 后续 ESDF 接入时不需要再重新定义一套轨迹结构

## 5. 时间参数化已经开始反哺 controller

现在 loopback 中存在：

- `trajectory_speed_governor`

它会：

1. 订阅 `/trajectory_profile`
2. 订阅 `cmd_vel_controller`
3. 输出 `cmd_vel_controller_governed`

然后再由：

- `velocity_smoother`

继续处理并输出 `cmd_vel_nav2_result`

所以“时间参数化反哺 controller”这件事在 loopback 里已经不是内部 profile 变量，而是已经进入执行链。

## 6. 本轮重新修复的内容

### 6.1 修复了回退后 `trajectory_optimizer` 的编译断点

这次误回退后，主要断点是：

1. `trajectory_optimizer` 源码残留了未使用函数
2. 在 `-Werror` 下直接导致构建失败

目前：

- `sp_msgs`
- `trajectory_optimizer`

都已经重新编译通过。

### 6.2 loopback 链已重新接通

当前 loopback 里已经存在：

1. `/smoothed_path_visual`
2. `/trajectory_profile_visual`
3. `/trajectory_profile`
4. `/cmd_vel_controller_governed`

说明：

1. path 侧可视化正常
2. profile 接口正常
3. speed governor 链路正常

### 6.3 loopback planner 已升级为 `SmacPlannerHybrid`

当前 loopback 已从：

- `NavfnPlanner`

升级为：

- `SmacPlannerHybrid`

这样与实车链更接近，也更适合分析“为什么第二个弯 still 切膨胀层”。

### 6.4 实车链已同步到完整版本

当前已同步到实车链的内容：

1. `reality/nav2_params.yaml` 的 `smoother_server` 已切到 `bspline_smoother`
2. 曲率 / 速度 / 障碍联合优化参数已同步到实车 `trajectory_optimizer`
3. `trajectory_speed_governor` 已接入实车 `navigation_launch.py`
4. `velocity_smoother` 已改为吃 `cmd_vel_controller_governed`
5. 实车旁路可视化仍保留：
   - `smoothed_path_visual`
   - `trajectory_profile_visual`

这意味着“最新这套曲线 + 规划 + 速度约束”已经不再只停留在 loopback，而是已经完整接入实车链的配置和 launch 层。

### 6.5 `bringup.launch.py` 已是最终实车入口

当前最终实车入口是：

- [bringup.launch.py](../src/pb2025_sentry_bringup/launch/bringup.launch.py)

它会继续包含：

1. `rm_navigation_reality_launch.py`
2. `navigation_launch.py`
3. 当前这套：
   - `trajectory_optimizer`
   - `bspline_smoother`
   - `trajectory_speed_governor`
   - `velocity_smoother`

所以后续所有“上车前检查”和“RViz 观察”都应以这条入口为准，而不是以 loopback 或单独 nav bringup 为准。

## 7. 当前 loopback 初始参数

### `trajectory_optimizer`

1. `control_point_spacing: 0.36`
2. `output_path_spacing: 0.05`
3. `max_lateral_deviation: 0.28`
4. `curvature_limit: 0.85`
5. `curvature_weight: 40.0`
6. `curvature_refinement_iterations: 10`
7. `curvature_refinement_gain: 0.05`
8. `global_speed_limit: 1.20`
9. `lateral_accel_limit: 1.0`
10. `longitudinal_accel_limit: 0.5`
11. `velocity_smoothing_gain: 0.3`
12. `derivative_step: 0.02`
13. `obstacle_safe_cost: 64`
14. `obstacle_weight: 35.0`
15. `obstacle_refinement_iterations: 3`
16. `obstacle_refinement_gain: 0.04`

### `bspline_smoother`

1. `max_path_cost: 64`
2. `pullback_samples: 8`

### `trajectory_speed_governor`

1. `min_speed_scale: 0.18`
2. `curvature_brake_gain: 1.35`

## 8. 当前仍然存在的限制

虽然现在已经有：

1. `J_curvature`
2. `J_velocity`
3. 时间参数化
4. profile 接口
5. speed governor

虽然现在已经有：

1. `J_curvature`
2. `J_velocity`
3. `J_obs`
4. 时间参数化
5. profile 接口
6. speed governor

并且它们已经进入同一套 `BSplinePathOptimizer::optimizeDetailed()` 框架，

但还没有做到：

1. 用 ESDF / distance field 作为 obstacle term 的连续梯度来源
2. 做真正的连续优化器求解（当前仍是 refinement-based unified optimizer）
3. 让 obstacle cost 在狭窄通道里更智能地区分“可贴边但可通行”和“必然卡死”

## 9. 当前对“为什么规划会靠近膨胀层”的理解

当前理解比之前更清楚：

1. 并不是每次 `/plan` 静态就直接踩进高 cost
2. 更常见的是第二个弯时：
   - planner 给的走廊已经不够保守
   - smoother 有内抹倾向
   - controller 再沿 shortcut 切进去
3. 一旦切进 inflation layer，就容易造成卡死或抖动

## 10. 最近一轮曲率收紧结果

针对“第二个弯还是太急”的问题，最近一轮主要做了两类调整：

1. 给样条更多几何自由度去把弯圆开：
   - 更大的 `control_point_spacing`
   - 更大的 `max_lateral_deviation`
   - 更多的 `curvature_refinement_iterations`
   - 更大的 `curvature_refinement_gain`
2. 让高曲率段更早减速：
   - 更低的 `global_speed_limit`
   - 更低的 `longitudinal_accel_limit`
   - 更强的 `curvature_brake_gain`

这轮抓到的 profile 指标变化：

1. `max_abs_curvature` 已从约 `4.89` 降到约 `2.83`
2. `curvature_penalty` 已从约 `912` 降到约 `598`
3. `velocity_smoothness_cost` 进一步下降
4. `obstacle_cost` 仍为 `0.0`

当前含义：

1. 第二个弯的几何形状已经明显变圆
2. 速度 profile 也更平顺
3. 当前主导问题仍然更像“高曲率段 + shortcut 跟踪”，而不是 obstacle term 先触发

## 11. bringup.launch.py 上车前检查清单

在真正上车前，建议按下面顺序确认：

1. **参数入口确认**
   - `params_file` 指向 [node_params.yaml](../src/pb2025_sentry_bringup/params/node_params.yaml)
   - `trajectory_optimizer` 段存在
   - `smoother_server.bspline_smoother` 段存在
   - `trajectory_speed_governor` 段存在

2. **launch 链确认**
   - `bringup.launch.py` 会包含 `rm_navigation_reality_launch.py`
   - `rm_navigation_reality_launch.py` 会包含 `navigation_launch.py`
   - `navigation_launch.py` 会启动：
     - `trajectory_optimizer_node`
     - `trajectory_speed_governor_node`
     - `controller_server`
     - `smoother_server`
     - `planner_server`
     - `velocity_smoother`

3. **关键 topic 确认**
   - `/plan`
   - `/smoothed_path_visual`
   - `/trajectory_profile_visual`
   - `/trajectory_profile`
   - `/cmd_vel_controller`
   - `/cmd_vel_controller_governed`
   - `/cmd_vel_nav2_result`

4. **RViz 观测确认**
   当前 [sentry_default_view.rviz](../src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz) 里应能看到：
   - 红色 `Global Plan (planner)`
   - 青色 `Smoothed Path (viz)`
   - `TrajectoryProfileMarkers`
   - 绿色/橙色 MPPI 局部链

5. **实车现象确认**
   - 第一个弯不应明显变差
   - 第二个弯应优先观察：
     - 是否还会主动切进 inflation layer
     - `TrajectoryProfileMarkers` 是否在高曲率段明显变色
     - `cmd_vel_controller_governed` 是否比 `cmd_vel_controller` 更平顺

所以“真正问题”不是一个孤立参数，而是：

- 第一层参考路径离障安全裕量不够
- 第二层在高曲率段的速度和 shortcut 倾向还过强

## 10. 下一步建议

现在最合理的下一步是继续推进真正的联合优化器：

1. 在 `TrajectoryProfile2D` / B 样条表示上加入 `J_obs`
2. 用连续 obstacle cost 把第一层路径本身从膨胀层外推开
3. 再让 `trajectory_speed_governor` 利用新的 profile 限速

具体优先级：

1. 先把 `J_obs` 作为统一 optimizer 的第三项接入
2. 再观察第二个弯的 `max_abs_curvature`、`speed_limit`、`obstacle_cost`
3. 再决定是否继续调 MPPI critic
