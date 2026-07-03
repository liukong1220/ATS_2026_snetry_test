# ats_sentry_nav

当前哨兵项目中的导航、定位、点云处理与恢复行为子系统。

本包系在当前工作区中的职责不是“整车总入口”，而是提供被总入口调用的导航能力，包括：

1. `ats_nav_bringup`：Nav2、定位、RViz、传感器链路启动
2. `trajectory_optimizer`：B 样条平滑、trajectory profile、signed Traversability ESDF、governor
3. `ats_nav2_plugins`：恢复行为与 costmap 插件
4. `fake_vel_transform`：速度坐标系变换与自旋叠加
5. `small_gicp_relocalization`、`point_lio`、`loam_interface`、`sensor_scan_generation`：定位与点云接口

## 当前主链

当前整车使用的导航执行主链为：

```text
SmacPlannerHybrid
  -> Nav2BSplineSmoother
  -> MPPI Controller
  -> trajectory_speed_governor
  -> velocity_smoother
  -> fake_vel_transform
  -> /cmd_vel
```

当前恢复链为：

```text
FollowPath 失败
  -> behavior_server
  -> BackUpFreeSpace
  -> 主走廊搜索
  -> 必要时 centroid fallback
  -> 平均走廊代价高时自动降速
```

当前地形语义到 ESDF 的过渡链为：

```text
terrain_analysis_ext
  -> terrain_map_ext
  -> traversability_grid
  -> traversability_height_diff_grid
  -> traversability_occupancy_ratio_grid
  -> traversability_ground_confidence_grid
  -> traversability_slope_grid
  -> traversability_slope_band_grid
  -> TraversabilityEsdfProvider
  -> Nav2BSplineSmoother / trajectory_optimizer_node
```

## 当前入口

### 实机导航子系统入口

- [ats_nav_bringup/launch/rm_navigation_reality_launch.py](./ats_nav_bringup/launch/rm_navigation_reality_launch.py)

说明：

- 当前整车维护优先使用工作区总入口 `ats_sentry_bringup/bringup.launch.py`
- 本入口更适合单独排查导航、定位、点云和 RViz 相关问题

### loopback 导航入口

- [../ats_sentry_bringup/launch/loopback_nav_only.launch.py](../ats_sentry_bringup/launch/loopback_nav_only.launch.py)
- [../ats_sentry_bringup/launch/loopback_decision_sim.launch.py](../ats_sentry_bringup/launch/loopback_decision_sim.launch.py)
- [../ats_sentry_bringup/launch/loopback_vision_test.launch.py](../ats_sentry_bringup/launch/loopback_vision_test.launch.py)

## 当前关键目录

```text
ats_sentry_nav/
├── fake_vel_transform/            # 速度坐标系转换与自旋叠加
├── ign_sim_pointcloud_tool/       # Ignition/Gazebo 点云格式补全
├── livox_ros_driver2/             # Livox mid360 驱动
├── loam_interface/                # point_lio 输出转换到导航 odom
├── ats_nav_bringup/            # 导航 launch、RViz、simulation/reality 参数
├── ats_nav2_plugins/               # BackUpFreeSpace、IntensityVoxelLayer 等插件
├── ats_teleop_twist_joy/           # 手柄速度/云台控制
├── pointcloud_to_laserscan/       # 建图模式下的点云转 LaserScan
├── point_lio/                     # 点云里程计
├── sensor_scan_generation/        # 点云 / odom / TF 相关速度与扫描生成
├── small_gicp_relocalization/     # 全局重定位
├── terrain_analysis/              # 近场地形分析
├── terrain_analysis_ext/          # 远场地形分析
└── trajectory_optimizer/          # B 样条平滑、profile、governor、ESDF 调试
```

## 当前最重要的参数入口

### 实机主参数

实际整车主入口默认读取：

- [../ats_sentry_bringup/params/node_params.yaml](../ats_sentry_bringup/params/node_params.yaml)

这份文件比 `ats_nav_bringup/config/reality/nav2_params.yaml` 更重要，因为它是当前工作区总入口真实加载的参数。

### reality 默认参数

- [ats_nav_bringup/config/reality/nav2_params.yaml](./ats_nav_bringup/config/reality/nav2_params.yaml)

当前主要用于：

1. 导航子系统单独启动
2. 与 `node_params.yaml` 对照
3. 保留 `ats_nav_bringup` 包内默认配置

### loopback 参数

- [../loopback_sim/params/nav2_params.yaml](../loopback_sim/params/nav2_params.yaml)

## 当前关键包说明

### `trajectory_optimizer`

关键文件：

- [trajectory_optimizer/src/bspline_path_optimizer.cpp](./trajectory_optimizer/src/bspline_path_optimizer.cpp)
- [trajectory_optimizer/src/nav2_bspline_smoother.cpp](./trajectory_optimizer/src/nav2_bspline_smoother.cpp)
- [trajectory_optimizer/src/trajectory_speed_governor.cpp](./trajectory_optimizer/src/trajectory_speed_governor.cpp)

当前已实现：

1. B 样条平滑
2. 曲率限速
3. 障碍距离限速
4. trajectory profile 发布
5. trajectory profile marker
6. FakeCostmap / TerrainPointCloud / Traversability 三类 ESDF provider
7. signed Traversability ESDF 调试 marker

当前 ESDF 主线说明：

1. `trajectory_optimizer_node` 与 `Nav2BSplineSmoother` 当前都支持 `esdf_source: traversability_grid`。
2. `TraversabilityEsdfProvider` 会融合 `traversability_grid`、`traversability_height_diff_grid`、`traversability_occupancy_ratio_grid`、`traversability_ground_confidence_grid`。
3. fake costmap ESDF 与 terrain pointcloud ESDF 仍保留为 fallback / 历史对照路径。

当前坡度语义说明：

1. `traversability_slope_grid` 使用 `nav_msgs/OccupancyGrid` 编码，`-1` 表示 unknown，`0~100` 线性对应 `0 ~ slopeGridMaxDeg` 的坡度角。
2. `traversability_slope_band_grid` 同样使用 `nav_msgs/OccupancyGrid` 编码，按 `slopeGentleDegThre / slopeModerateDegThre / slopeSteepDegThre` 分成平缓、中坡、陡坡三档。
3. 默认这两张图优先用于任务 2 的坡道速度规则与 RViz 观测，不直接改写当前 `traversability_grid` 的二值通行逻辑。
4. 如果希望“坡太陡就直接绕开”，把 `terrain_analysis_ext.useSlopeAsObstacle` 设为 `true`，再用 `slopeObstacleDegThre` 调整坡度障碍阈值。

### `ats_nav2_plugins`

关键文件：

- [ats_nav2_plugins/src/behaviors/back_up_free_space.cpp](./ats_nav2_plugins/src/behaviors/back_up_free_space.cpp)
- [ats_nav2_plugins/include/ats_nav2_plugins/behaviors/back_up_free_space.hpp](./ats_nav2_plugins/include/ats_nav2_plugins/behaviors/back_up_free_space.hpp)

当前恢复行为已实现：

1. 主走廊恢复搜索
2. lookahead 前缀安全检测
3. 连续阻塞滞回 + 冷却重规划
4. centroid fallback
5. 高 cost 自动降速
6. RViz marker 区分 `corridor` / `centroid_fallback`

### `small_gicp_relocalization`

关键文件：

- [small_gicp_relocalization/src/small_gicp_relocalization.cpp](./small_gicp_relocalization/src/small_gicp_relocalization.cpp)

当前重定位逻辑已实现：

1. 条件触发配准
2. 短窗口点云累积
3. 手动 `initialpose` 后强制注册窗口
4. 成功结果接受时的 inlier / error 判据

### `fake_vel_transform`

关键文件：

- [fake_vel_transform/src/fake_vel_transform.cpp](./fake_vel_transform/src/fake_vel_transform.cpp)

当前作用：

1. 将 `cmd_vel_nav2_result` 从 fake reference frame 变换到机器人底盘参考系
2. 叠加 `cmd_spin`
3. 输出最终 `/cmd_vel`

## 当前常用话题

### 规划 / 平滑 / 控制

- `plan`
- `smoothed_path_visual`
- `trajectory_profile`
- `trajectory_profile_visual`
- `trajectory_profile_markers`
- `trajectory_esdf_debug`
- `traversability_grid`
- `traversability_height_diff_grid`
- `traversability_occupancy_ratio_grid`
- `traversability_ground_confidence_grid`
- `traversability_slope_grid`
- `traversability_slope_band_grid`
- `cmd_vel_controller`
- `cmd_vel_controller_governed`
- `cmd_vel_nav2_result`
- `cmd_vel`

### 恢复与局部可视化

- `back_up_free_space_markers`
- `local_costmap/costmap`
- `global_costmap/costmap`
- `transformed_global_plan`
- `trajectories`

## 当前维护建议

1. 如果问题是“速度慢、弯前过保守、贴障限速异常”，优先看 `trajectory_optimizer`
2. 如果问题是“规划能过但恢复动作不自然”，优先看 `ats_nav2_plugins`
3. 如果问题是“姿态/视觉/目标点异常”，不要先改本包，优先看 `ats_sentry_behavior`
4. 如果问题是“最终底盘速度和 Nav2 输出不一致”，同时看 `fake_vel_transform` 与 `standard_robot_pp_ros2`
5. 若只改了 `ats_nav_bringup/config/reality/nav2_params.yaml` 却发现总入口没变化，先确认当前是不是从 `node_params.yaml` 启动的

## 相关文档

- [../../docs/总览.md](../../docs/总览.md)
- [../../docs/mppi_parameter_tuning_guide.md](../../docs/mppi_parameter_tuning_guide.md)
- [../../docs/omni_recovery_smoothing_optimization.md](../../docs/omni_recovery_smoothing_optimization.md)
- [../../docs/上车测试清单.md](../../docs/上车测试清单.md)
- [../../docs/gazebo_sim_integration.md](../../docs/gazebo_sim_integration.md)
