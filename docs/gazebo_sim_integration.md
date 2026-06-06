# Gazebo 仿真接入与当前验证范围

更新时间：2026-06-06

本文档专门回答两个问题：

1. 当前仓库的 Gazebo 仿真链到底接入到了哪里。
2. 以现在的代码状态，仿真能验证哪些内容，不能验证哪些内容。

## 1. 当前已接入的 Gazebo 组件

当前项目已接入：

1. `src/rmu_gazebo_simulator`
2. `src/dependencies/rmoss_gazebo`
3. 项目级入口 `ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py`

职责分工：

1. `rmu_gazebo_simulator` 负责 Gazebo 世界、机器人生成、桥接与仿真基础设施
2. `rmoss_gazebo` 负责机器人底盘/云台/里程计等 Gazebo 侧接口
3. `pb2025_nav_bringup` 负责把当前导航链挂到 Gazebo 机器人上

## 2. 默认启动入口

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py
```

默认行为：

1. `sim_world:=rmuc_2025`
2. `nav_world:=rmul`
3. `namespace:=red_standard_robot1`
4. `use_sim_time:=True`
5. 默认启动当前导航主链
6. 默认不启动行为层

## 3. 常用参数

切换 Gazebo 世界：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py sim_world:=rmul_2025
```

切换导航地图资产：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py nav_world:=rmul
```

启动行为层：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py launch_behavior:=True
```

关闭 RViz：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py use_rviz:=False
```

关闭旁路 `trajectory_optimizer` 可视化节点：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py launch_trajectory_optimizer:=False
```

## 4. 当前实际仿真链路

当前 Gazebo 仿真验证的并不是“最终版 JPS + MINCO + MPC”，而是下面这条过渡主链：

`Gazebo robot -> 点云/里程计 -> terrain_analysis -> terrain_analysis_ext -> terrain_map_ext + traversability_grid -> terrain_pointcloud ESDF -> Nav2(Smac + bspline smoother + MPPI)`

更细一点是：

1. `bringup_sim.launch.py`
2. `rm_navigation_simulation_launch.py`
3. `ign_sim_pointcloud_tool`
4. `terrain_analysis`
5. `terrain_analysis_ext`
6. `terrain_map_ext`
7. `traversability_grid`
8. `planner_server: SmacPlannerHybrid`
9. `smoother_server: Nav2BSplineSmoother`
10. `controller_server: MPPIController`
11. 可选 `trajectory_optimizer_node` 作为旁路可视化与剖面调试

## 5.1 当前完整仿真模式

当前建议区分两种入口使用。

### A. Gazebo 导航模式

入口：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py use_rviz:=False
```

用途：

1. 启动完整 Gazebo 世界和机器人
2. 验证 `point_lio -> terrain_analysis_ext -> terrain_map_ext`
3. 验证 `Smac -> bspline smoother -> MPPI`

### B. 导航栈单独仿真模式

入口：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py
```

用途：

1. 已有 Gazebo/传感器环境时，只拉起导航相关 ROS 链
2. 便于单独调导航参数、RViz 和轨迹话题

### C. 建图模式

入口：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py slam:=True
```

这条链会切到：

1. `point_lio`
2. `terrain_analysis`
3. `terrain_analysis_ext`
4. `pointcloud_to_laserscan`
5. `slam_toolbox`
6. `map_saver_server`

当前建图模式的关键链路是：

`registered_scan -> terrain_map_ext -> obstacle_scan -> slam_toolbox`

也就是说，当前 `slam:=True` 不是直接用原始激光 scan 建图，而是先经过：

1. `point_lio` 输出 `cloud_registered`
2. `loam_interface` 输出 `registered_scan / lidar_odometry`
3. `terrain_analysis_ext` 输出 `terrain_map_ext`
4. `pointcloud_to_laserscan` 把 `terrain_map_ext` 转成 `obstacle_scan`
5. `slam_toolbox` 基于 `obstacle_scan` 建图

适合用途：

1. 检查当前 2.5D 地形前端在建图模式下是否稳定
2. 生成与当前 terrain 前端一致的地图资产

## 6. 已经同步到仿真的 2.5D ESDF 改动

当前仿真参数已明确切到过渡版点云 ESDF 链：

1. `trajectory_optimizer.esdf_source: terrain_pointcloud`
2. `trajectory_optimizer.terrain_pointcloud_topic: terrain_map_ext`
3. `smoother_server.bspline_smoother.esdf_source: terrain_pointcloud`
4. `smoother_server.bspline_smoother.terrain_pointcloud_topic: terrain_map_ext`
5. `terrain_analysis_ext` 新增 `traversability_grid`

这意味着 Gazebo 现在验证的不是旧版 `costmap fake ESDF` 单一路线，而是：

1. `terrain_analysis_ext` 输出的扩展地形点云
2. `terrain_pointcloud_esdf_provider` 构造出的二维 ESDF
3. `bspline smoother` 利用该 ESDF 做近障碍回拉

## 7. 这套仿真目前能验证什么

当前可以验证：

1. Gazebo 点云是否能正确进入 `terrain_analysis` 与 `terrain_analysis_ext`
2. `terrain_map_ext` 是否与地图障碍位置基本一致
3. `traversability_grid` 是否已经把“可通行 / 不可通行 / 未知”分出来
4. `terrain_pointcloud` ESDF 是否能在 RViz 中表现出合理的近障碍风险分布
5. `Nav2BSplineSmoother` 是否会把贴边路径往安全侧回拉
6. `trajectory_profile` 的曲率、速度限制和近障碍代价是否合理
7. MPPI 在上述过渡路径上的跟踪是否连续稳定

尤其适合验证的场景：

1. 贴墙通过
2. 狭窄通道
3. 斜向隧道入口
4. 转角前后靠障碍的路径段

## 8. 这套仿真目前不能验证什么

当前还不能直接验证：

1. 自有 `JPS/A*` 前端搜索
2. `MINCO` 两阶段轨迹优化
3. `SE2 MPC` 控制器
4. 技术报告中完整的 `3D Occupancy -> Traversability -> ESDF -> MINCO -> MPC` 闭环

原因很简单：

1. 当前仓库里这些模块尚未落地
2. Gazebo 现在验证的是“2.5D ESDF 过渡链”，不是最终自研导航主链

所以仿真结论必须表述准确：

`当前 Gazebo 仿真验证的是过渡方案，不是最终方案。`

## 9. 当前建议的 Gazebo 测试目标

在 `MINCO/MPC` 尚未接入前，建议把 Gazebo 先用于完成下面三类验证。

### 8.1 地图前端一致性

看这些 topic：

1. `terrain_map`
2. `terrain_map_ext`
3. `traversability_grid`
4. `global_costmap/costmap_raw`
5. `smoothed_path_visual`
6. `trajectory_profile_visual`

目标：

1. 确认 `terrain_map_ext` 真正覆盖到狭窄通道边界
2. 确认 `traversability_grid` 中可通行区域没有被大面积误杀
3. 确认 ESDF 风险高的区域和障碍位置一致

### 8.2 路径回拉效果

目标：

1. 确认贴边的 `plan` 能被 smoother 回拉
2. 确认回拉后路径没有明显振荡或反复跨障碍侧切换

重点看：

1. `plan`
2. `smoothed_path`
3. `smoothed_path_visual`
4. `trajectory_esdf_debug`

### 8.3 控制跟踪稳定性

目标：

1. 确认 MPPI 在这套过渡路径上不会频繁抖动
2. 确认曲率限速和近障碍减速没有把速度链打断

重点看：

1. `/cmd_vel_nav2_result`
2. `/cmd_vel`
3. `trajectory_profile`

## 10. 接下来怎么把 Gazebo 用到最终迁移中

推荐把仿真也按阶段推进。

### 阶段 A：当前阶段

验证：

1. `terrain_map_ext -> traversability_grid -> terrain_pointcloud ESDF -> bspline smoother -> MPPI`

### 阶段 B：下一阶段

验证：

1. `traversability_grid -> Traversability ESDF`
2. 先不抢控制权，只做可视化对照

### 阶段 C：再下一阶段

验证：

1. `JPS -> MINCO` 并联规划
2. 和 `Smac + bspline` 做轨迹形状、时长、通过率对照

### 阶段 D：最终阶段

验证：

1. `SE2 MPC` 闭环跟踪
2. 逐步摘掉 MPPI 和 Nav2 planner/smoother 主链

## 11. 当前限制

1. `nav_world` 仍依赖仓内现有地图资产，当前主要使用 `rmul`
2. Gazebo 世界名与导航地图资产名仍是分开的
3. 当前 `traversability_grid` 还是第一版语义输出，仅包含 `unknown / traversable / occupied`
4. 当前过渡 ESDF 仍是从 `terrain_map_ext` 直接构图，尚未切换成 `TraversabilityEsdfProvider`
5. 行为层、规划层和控制层仍然深度依赖 Nav2 生命周期

## 12. 本文档的使用方式

如果你现在要做的是：

1. 验证 2.5D ESDF 过渡链是否靠谱

就按本文档直接跑 Gazebo。

如果你现在要做的是：

1. 验证技术报告里的最终版 JPS + MINCO + MPC

那当前仓库还不具备直接验证条件，需要先把这些模块实现出来，再扩展仿真入口。
