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
2. `nav_world:=rmuc_2025`
3. `namespace:=red_standard_robot1`
4. `use_sim_time:=True`
5. 默认启动当前导航主链
6. 默认不启动行为层
7. 默认启动 `small_gicp` 重定位与底盘速度坐标变换，优先验证与实车一致的定位/控制链

## 3. 常用参数

切换 Gazebo 世界：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py sim_world:=rmul_2025
```

切换导航地图资产：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py nav_world:=rmuc_2025
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

关闭 `small_gicp` 重定位：

```bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py launch_small_gicp_relocalization:=False
```

## 4. 当前实际仿真链路

当前 Gazebo 仿真验证的并不是“最终版 JPS + MINCO + MPC”，而是下面这条过渡主链：

`Gazebo robot -> 点云/里程计 -> terrain_analysis -> terrain_analysis_ext -> terrain_map_ext + traversability_grid + 地形语义调试栅格 -> signed Traversability ESDF -> Nav2(Smac + bspline smoother + MPPI)`

更细一点是：

1. `bringup_sim.launch.py`
2. `rm_navigation_simulation_launch.py`
3. `ign_sim_pointcloud_tool`
4. `terrain_analysis`
5. `terrain_analysis_ext`
6. `terrain_map_ext`
7. `traversability_grid`
8. `traversability_height_diff_grid`
9. `traversability_occupancy_ratio_grid`
10. `traversability_ground_confidence_grid`
11. `TraversabilityEsdfProvider`
12. `planner_server: SmacPlannerHybrid`
13. `smoother_server: Nav2BSplineSmoother`
14. `controller_server: MPPIController`
15. 可选 `trajectory_optimizer_node` 作为旁路可视化与剖面调试

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

推荐两终端“实车同构”测试方式：

终端 1 启动 Gazebo：

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc_2025
```

终端 2 启动导航：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  use_rviz:=True \
  launch_small_gicp_relocalization:=True \
  launch_chassis_vel_transform:=True
```

这一路径默认从 `pb2025_nav_bringup/map/simulation` 和 `pb2025_nav_bringup/pcd/simulation` 读取同名资产，并保留与实车一致的主链：

1. `small_gicp_relocalization` 持续发布 `map -> odom`
2. `gimbal_yaw_odom` 作为定位与底盘速度参考主轴
3. `fake_vel_transform + chassis_vel_transform` 保留实车同构速度链
4. 更适合直接验证“仿真与实车是否吻合”
5. `small_gicp.init_pose` 必须与 `rmu_gazebo_simulator/config/gz_world.yaml` 中该机器人的出生点一致

推荐优先用这条链做算法验证；只有在隔离 TF/地图问题时，才暂时关闭 `small_gicp`。

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

当前仿真参数已明确切到过渡版 traversability ESDF 链：

1. `trajectory_optimizer.esdf_source: traversability_grid`
2. `trajectory_optimizer.traversability_grid_topic: traversability_grid`
3. `trajectory_optimizer.traversability_height_diff_topic: traversability_height_diff_grid`
4. `trajectory_optimizer.traversability_occupancy_ratio_topic: traversability_occupancy_ratio_grid`
5. `trajectory_optimizer.traversability_ground_confidence_topic: traversability_ground_confidence_grid`
6. `smoother_server.bspline_smoother.esdf_source: traversability_grid`
7. `smoother_server.bspline_smoother.traversability_grid_topic: traversability_grid`
8. `terrain_analysis_ext` 发布 traversability 与三类地形语义调试栅格

这意味着 Gazebo 现在验证的不是旧版 `costmap fake ESDF` 单一路线，而是：

1. `terrain_analysis_ext` 输出的扩展地形点云和可通行栅格
2. `traversability_esdf_provider` 构造出的 signed 2D ESDF
3. `bspline smoother` 利用该 ESDF 做近障碍回拉

## 7. 这套仿真目前能验证什么

当前可以验证：

1. Gazebo 点云是否能正确进入 `terrain_analysis` 与 `terrain_analysis_ext`
2. `terrain_map_ext` 是否与地图障碍位置基本一致
3. `traversability_grid` 是否已经把“可通行 / 不可通行 / 未知”分出来
4. `traversability_height_diff_grid / traversability_occupancy_ratio_grid / traversability_ground_confidence_grid` 是否能解释 risk 区域来源
5. signed `traversability` ESDF 是否能在 RViz 中表现出合理的近障碍风险分布
6. `Nav2BSplineSmoother` 是否会把贴边路径往安全侧回拉
7. `trajectory_profile` 的曲率、速度限制和近障碍代价是否合理
8. MPPI 在上述过渡路径上的跟踪是否连续稳定

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
4. `traversability_height_diff_grid`
5. `traversability_occupancy_ratio_grid`
6. `traversability_ground_confidence_grid`
7. `global_costmap/costmap_raw`
8. `smoothed_path_visual`
9. `trajectory_profile_visual`

目标：

1. 确认 `terrain_map_ext` 真正覆盖到狭窄通道边界
2. 确认 `traversability_grid` 中可通行区域没有被大面积误杀
3. 确认 height / occupancy / ground confidence 能解释 ESDF 风险高的区域
4. 确认 ESDF 风险高的区域和障碍位置一致

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

## 9. 仿真 TF 约定

Gazebo 导航模式保留 `gimbal_yaw_fake` 作为 Nav2 的 `robot_base_frame`，同时继续以 `gimbal_yaw_odom` 作为定位与底盘速度参考主轴。

当前仿真中，TF 主链应为：

`map -> odom -> gimbal_yaw_odom -> gimbal_yaw_fake`

其中：

1. `small_gicp_relocalization` 发布 `map -> odom`
2. `sensor_scan_generation` 根据点云里程计发布 `odom -> gimbal_yaw_odom`
3. `fake_vel_transform` 发布 `gimbal_yaw_odom -> gimbal_yaw_fake`
4. `chassis_vel_transform` 把 `cmd_vel_gimbal_yaw_odom` 转成最终 `cmd_vel`
5. Nav2 的 `bt_navigator / local_costmap / global_costmap / behavior_server` 继续使用 `gimbal_yaw_fake`

如果 local costmap 报：

`Timed out waiting for transform from gimbal_yaw_fake to odom`

优先检查：

1. `small_gicp_relocalization` 是否正常发布 `map -> odom`
2. `sensor_scan_generation` 是否收到 `lidar_odometry` 和 `registered_scan`
3. `fake_vel_transform` 与 `chassis_vel_transform` 是否在机器人命名空间内启动
4. `/red_standard_robot1/tf` 中是否存在 `odom -> gimbal_yaw_odom -> gimbal_yaw_fake`

## 10. 接下来怎么把 Gazebo 用到最终迁移中

推荐把仿真也按阶段推进。

### 阶段 A：当前阶段

验证：

1. `terrain_map_ext -> traversability_grid + 地形语义栅格 -> signed traversability ESDF -> bspline smoother -> MPPI`

### 阶段 B：下一阶段

验证：

1. `signed Traversability ESDF -> JPS/A*`
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

1. `nav_world` 当前已可直接使用仓内同步过来的 `rmuc_2025 / rmul_2025 / rmuc_2024 / rmul_2024`
2. Gazebo 世界名与导航地图资产名仍是分开的
3. 当前过渡 ESDF 已切换到 `TraversabilityEsdfProvider`，但仍属于 2D / 2.5D ESDF
4. 当前尚未实现自有 `JPS/A*`、`MINCO` 和 `SE2 MPC`
5. 行为层、规划层和控制层仍然深度依赖 Nav2 生命周期

## 12. 本文档的使用方式

如果你现在要做的是：

1. 验证 2.5D ESDF 过渡链是否靠谱

就按本文档直接跑 Gazebo。

如果你现在要做的是：

1. 验证技术报告里的最终版 JPS + MINCO + MPC

那当前仓库还不具备直接验证条件，需要先把这些模块实现出来，再扩展仿真入口。
