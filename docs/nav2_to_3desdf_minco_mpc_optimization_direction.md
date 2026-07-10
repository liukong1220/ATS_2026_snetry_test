# ATS 导航优化当前框架与下一阶段交接

更新时间：2026-07-08

本文档已按当前 `src` 目录重新检索后整理，并补充 2026-07-07 MuJoCo/MID360/ESDF/RViz 导航闭环修正结果。它的用途是让新对话先理解当前工程真实结构，再继续推进 RC-ESDF、MINCO、MuJoCo、MPC 等优化。本文不再记录历史流水账；Gazebo 入口和依赖已清理，后续完整仿真统一使用 MuJoCo，loopback 保留用于快速决策和导航链路测试。

## 0. V1 / V2 版本目标

本文档所有优化都围绕两个版本边界展开，后续新对话必须先按这个边界判断任务属于当前比赛主线还是长期升级。

### 0.1 V1：当前比赛可用主线

`V1` 的目标是先把当前地面哨兵导航做稳定、可观察、可比赛使用：

`2.5D 地形语义 + 2D 栅格主链 + RC-ESDF-lite/RC-ESDF + JPS + MINCO + 独立 Yaw + footprint safety + Local Collision Repair + SE2 MPC`

当前代码还没有完全到达这条终局链。现阶段真实运行链是：

`SmacPlanner2D + stable global reference + RC-ESDF local elastic path + Nav2BSplineSmoother + trajectory_speed_governor + MPPI`

因此 `V1` 当前阶段不是立即推翻 Nav2，而是先把 MuJoCo/RViz2 中的现有过渡主链跑稳定，再逐步替换为 `JPS + MINCO + footprint safety + SE2 MPC`。

### 0.2 V2：长期 3D ESDF 后端升级

`V2` 的目标是在 `V1` 稳定后升级地图后端能力：

`3D Occupancy / 3D ESDF -> 地面任务语义抽取 -> 复用 V1 的搜索、轨迹、安全和控制接口`

`V2` 不是当前立刻开工的主线，也不是把哨兵导航改成空中机器人式三维路径搜索。它的重点是让地图后端更强，但上层仍围绕地面机器人输出 `2D / 2.5D` 可通行语义。

当前结论：先做 `V1`，先稳定 MuJoCo 完整闭环，再推进 RC-ESDF、MINCO、footprint safety、SE2 MPC；`V2` 等 `V1` 稳定后再做。

### 0.3 2026-07-07 MuJoCo 导航基线状态

当前 MuJoCo 导航基线已经完成一轮传感器、地形、ESDF 输入、Nav2 action、控制桥接的闭环验证：

1. `ros2 launch ats_mujoco_sim mujoco_navigation.launch.py` 是当前完整仿真入口。
2. MuJoCo LiDAR 使用 MID360 扫描模式，`/local_pointcloud` 在 `front_mid360`，`/registered_scan` 在 `odom`。
3. `terrain_analysis` / `terrain_analysis_ext` 输出的 `/terrain_map`、`/terrain_map_ext`、`/traversability_grid`、`/traversability_slope_grid` 均按 `odom` 语义进入 Nav2 / smoother / RC-ESDF-lite。
4. `terrain_analysis_ext` 发布 `OccupancyGrid` 时已经把内部 `planarVoxelWidth * indX + indY` 索引转换为 ROS 标准 `x + y * width` 索引，避免 ESDF / traversability grid 出现 X/Y 转置。
5. RViz2 的 `mujoco_navigation.rviz` 已使用 `nav2_rviz_plugins/GoalTool`，目标点会进入 `/navigate_to_pose` action，而不是只发布旧的 `/goal_pose` topic。
6. 回归脚本 `scripts/test_mujoco_nav_chain.sh` 已覆盖 `/localization`、TF、MID360 点云、注册点云、terrain map、traversability grid、Nav2 lifecycle、`/navigate_to_pose`、`/cmd_vel_nav2_result`、`/motion_control`。
7. 已接入用户导出的 RMUC2026 OBJ/STL，新增 `rmuc_2026_swerve.xml` 和 `rmuc_2026_mujoco.launch.py`，可直接用真实 mesh 作为 MuJoCo 场地。

## 1. 当前结论

当前主线是：

`实车/仿真统一 ROS 接口 -> Nav2 过渡主链 -> 2.5D traversability -> RC-ESDF-lite -> Nav2BSplineSmoother + trajectory_speed_governor + MPPI -> 后续 minco_planner + SE2 MPC`

当前仿真主线是：

`MuJoCo + RViz2 + Nav2 + trajectory_optimizer`

Gazebo 不再作为后续仿真方案；`loopback_sim` 适合低成本验证 Nav2 参数、话题链和行为树；MuJoCo 用于后续完整底盘动力学、传感器、控制器和实车接口对齐测试。

当前执行链还不是最终 `JPS + MINCO + SE2 MPC`。真实状态是：

1. 全局规划：`nav2_smac_planner/SmacPlanner2D`
2. 主链平滑：`trajectory_optimizer/Nav2BSplineSmoother`
3. 控制器：`nav2_mppi_controller::MPPIController`
4. 曲率/坡度/障碍限速：`trajectory_speed_governor`
5. 目标规划骨架：`minco_planner`，但真实 MINCO/JPS/footprint SDF/SE2 MPC 尚未完成替换

因此下一阶段最稳妥的推进方式是：先把 MuJoCo 中的当前主链稳定跑通和观察清楚，再逐层把 RC-ESDF、footprint safety、MINCO、JPS、SE2 MPC 替换进去。

当前 MuJoCo 基线已经可以作为后续替换链路前的回归标准。任何 RC-ESDF、MINCO、JPS、MPC 改动后，都应至少重新跑通：

```bash
scripts/test_mujoco_nav_chain.sh
```

该脚本不替代人工 RViz 检查，但能快速确认 Nav2 action 和控制输出没有被破坏。

## 2. 当前 src 分层

### 2.1 顶层目录

当前 `src` 里和导航相关的主目录如下：

1. `src/ats_sentry_bringup`
   实车和综合启动顶层入口。包含 `bringup.launch.py`、loopback 启动、实车默认 `node_params.yaml`、地图、PCD、RViz 配置。
2. `src/ats_sentry_nav`
   导航子工作区。包含 Nav2 bringup、Nav2 插件、定位/点云转换、地形分析、轨迹优化、MINCO 规划骨架、速度坐标转换等。
3. `src/sim`
   仿真工作区。包含 `ats_mujoco_sim` 和 `loopback_sim`。MuJoCo 是完整仿真主线，loopback 用于快速测试。
4. `src/interfaces`
   消息与服务接口。当前包含 `ats_rm_interfaces`、`manda_can_control`、`carstatemsgs`、`sp_msgs`。
5. `src/dependencies`
   第三方或外部依赖，包括 BehaviorTree.ROS2、rmoss core/interfaces、sdformat_tools 等。Gazebo 相关依赖已从当前清单移除。
6. `src/tools`
   辅助工具，如 `pcd2pgm`、rosbag recorder、键盘遥控等。
7. `src/sp_vision25`
   视觉工程，当前不是本导航优化文档的主线，但会影响实车系统整体 bringup。

后续新代码统一使用 `ats_` 命名。历史 `pb` 前缀只允许出现在维护检查命令中，不应作为当前包名、topic、参数或文档主结构继续出现。

### 2.2 实车顶层 bringup

实车主入口是：

```bash
ros2 launch ats_sentry_bringup bringup.launch.py
```

该入口当前负责组织：

1. `standard_robot_pp_ros2`
   实车串口/底盘/机器人基础通信入口。
2. `sentry_chassis_vel_transform`
   将 `cmd_vel_gimbal_yaw_odom` 转换成底盘实际 `/cmd_vel`，并处理大 yaw 坐标系。
3. `ats_nav_bringup/rm_navigation_reality_launch.py`
   实车 Nav2、定位、地形分析、trajectory optimizer、small_gicp 等导航子系统。
4. `ats_sentry_behavior`
   行为树系统。
5. `rviz_launch.py`
   可选 RViz2。
6. `rosbag2_composable_recorder`
   可选轻量 rosbag 记录。

实车默认参数主要在：

`src/ats_sentry_bringup/params/node_params.yaml`

这份参数当前是实车综合入口的主要事实来源，包含 Livox、Point-LIO、loam_interface、small_gicp、fake_vel_transform、chassis_vel_transform、Nav2、smoother、trajectory optimizer、speed governor 等配置。

### 2.3 Nav2 子 bringup

Nav2 子入口位于：

`src/ats_sentry_nav/ats_nav_bringup/launch`

关键 launch：

1. `rm_navigation_reality_launch.py`
   实车导航入口。会启动 `livox_ros_driver2`，过滤可能冲突的 MVS `LD_LIBRARY_PATH`，再拉起 `bringup_launch.py`、RViz、可选 joy teleop。
2. `navigation_launch.py`
   Nav2 核心节点入口。当前会启动 `terrain_analysis`、`terrain_analysis_ext`、静态 TF、可选 `sentry_chassis_vel_transform`、`loam_interface`、`sensor_scan_generation`、`fake_vel_transform`、可选 `trajectory_optimizer_node`、固定 `trajectory_speed_governor_node`、Nav2 controller/smoother/planner/behavior/bt/waypoint/velocity_smoother/lifecycle。
3. `bringup_launch.py`
   更上层的 Nav2 组合入口。

需要注意：`trajectory_optimizer_node` 现在维护稳定全局参考并发布 `local_elastic_path`；BT 的 `FollowElasticPath` 仅在路径有实质变化时向 controller 更新 FollowPath goal。`smoother_server` 的 `trajectory_optimizer/Nav2BSplineSmoother` 和其 `trajectory_profile` 仍负责既有主链的平滑与速度治理。恢复行为继续使用自研 `ats_nav2_behaviors/BackUpFreeSpace`。

### 2.4 当前仿真入口

当前仿真分三类：

1. `src/sim/ats_mujoco_sim`
   当前主线仿真。负责 MuJoCo 底盘、随机地图、RM2026 窄路坡道地图、scene 生成、MID360-pattern LiDAR、ToF、Twist 到 `/motion_control` 桥接、RViz2 配置、完整 Nav2 联调入口。
2. `src/sim/loopback_sim`
   轻量 loopback 仿真。适合在低性能电脑上快速验证 Nav2 行为、参数和 topic 链路。
MuJoCo 主要入口：

```bash
ros2 launch ats_mujoco_sim mujoco_navigation.launch.py
```

只看 MuJoCo、地图、传感器、RViz2：

```bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py
```

Gazebo 综合入口已经从当前主线清理，不再新增或维护 Gazebo 调试路径。

MuJoCo RViz2 默认配置：

`src/sim/ats_mujoco_sim/rviz/mujoco_navigation.rviz`

这份配置用于完整导航观察，当前应包含：

1. `Navigation 2` panel。
2. `nav2_rviz_plugins/GoalTool` 目标工具。
3. `/map`、TF、`/local_pointcloud`、`/registered_scan`、`/terrain_map_ext`、`/traversability_grid`、global/local costmap、`/plan` 等显示项。

注意：不要再依赖 RViz 默认 `SetGoal -> /goal_pose` 工具测试 Nav2 主链。当前 Nav2 主链目标入口是 `/navigate_to_pose` action。

## 3. 当前数据与控制链路

### 3.1 实车/常规 Nav2 过渡链

当前实车参数中，速度链路大致是：

`Nav2 controller_server -> cmd_vel_controller -> trajectory_speed_governor -> cmd_vel_controller_governed -> velocity_smoother -> cmd_vel_nav2_result -> fake_vel_transform -> cmd_vel_gimbal_yaw_odom -> sentry_chassis_vel_transform -> /cmd_vel`

关键参数位置：

1. `fake_vel_transform`
   `input_cmd_vel_topic: cmd_vel_nav2_result`
   `output_cmd_vel_topic: cmd_vel_gimbal_yaw_odom`
2. `chassis_vel_transform`
   `input_cmd_vel_topic: cmd_vel_gimbal_yaw_odom`
   `output_cmd_vel_topic: /cmd_vel`
3. `trajectory_speed_governor`
   `profile_topic: trajectory_profile`
   `input_cmd_vel_topic: cmd_vel_controller`
   `output_cmd_vel_topic: cmd_vel_controller_governed`

这一链路说明：当前真正参与控制闭环的是 `Nav2BSplineSmoother` 生成的 `trajectory_profile` 和 `trajectory_speed_governor`，不是旁路 `trajectory_optimizer_node` 的 `trajectory_profile_visual`。

### 3.2 MuJoCo 导航链

MuJoCo 完整导航入口的目标链路是：

`ats_mujoco_sim -> /localization + /lidar_odometry + /local_pointcloud + /registered_scan + /perception/tof/points_merged -> Nav2 / trajectory_optimizer -> cmd_vel_gimbal_yaw_odom -> twist_to_motion_ctrl -> /motion_control -> MuJoCo chassis`

当前 `ats_mujoco_sim` 提供的 console scripts：

1. `ats_mujoco_sim`
2. `twist_to_motion_ctrl`
3. `generate_ats_mujoco_map`
4. `generate_ats_mujoco_scene`

关键点：

1. MuJoCo 包位于 `src/sim/ats_mujoco_sim`，不是 `src/ats_mujoco_sim`。
2. LiDAR 默认应使用 `lidar_backend:=cpu`，避免低性能电脑缺少 `taichi` 后进程崩溃。
3. 默认 LiDAR 模式固定为 `mid360`，可用 `lidar_downsample` 降低负载。
4. RViz2 配置包括 `mujoco_navigation.rviz` 和 `mujoco_sim_observe.rviz`。
5. MuJoCo 与实车应继续通过统一 topic 抽象隔离，规划层不应直接依赖 MuJoCo 内部实现。

当前 MuJoCo MID360 / 注册点云约定：

1. `/local_pointcloud`
   frame 为 `front_mid360`，表示仿真 MID360 传感器局部坐标系点云，用于 RViz 观察和传感器链路对齐。
2. `/registered_scan`
   frame 为 `odom`，由 MuJoCo 根据 LiDAR raycast 命中点转换到世界/里程计语义后发布，供 `terrain_analysis` 和 `terrain_analysis_ext` 直接消费。
3. `/lidar_odometry`
   header frame 为 `odom`，child frame 为 `front_mid360`，描述当前 MID360 位姿。
4. `/localization`
   header frame 为 `odom`，child frame 为 `gimbal_yaw_odom`，供 Nav2 和 costmap 获取机器人基准位姿。
5. `gimbal_yaw_odom -> front_mid360` 静态外参应与 `ats_robot_description` 中 MID360 模型一致，MuJoCo 可视 MID360 模型应只服务于仿真和观察，不应重新引入额外旧 LiDAR 旋转。

这意味着 MuJoCo 模式下不要再把 `/registered_scan` 当作 Point-LIO 的 `cloud_registered` 旧语义重复转换；它已经是 `odom` 下的注册点云。

### 3.3 地图、点云与地形链

当前导航主链仍是地面机器人 `2D / 2.5D` 主链：

1. `livox_ros_driver2` 或 MuJoCo LiDAR 发布点云。
2. `point_lio` / 仿真定位链提供里程计与注册点云。
3. `loam_interface` 对接定位输出、注册点云、frame。
4. `sensor_scan_generation` 生成导航需要的 scan/terrain 输入。
5. `terrain_analysis` 与 `terrain_analysis_ext` 输出地形语义。
6. `traversability_grid`、`traversability_slope_grid` 等进入 smoother、optimizer 和后续 minco_planner。

必须继续保持的原则：

1. 主导航拓扑是地面 `2D` 栅格主链，不是三维路径搜索。
2. `2.5D` 层负责高程、坡度、占有率、roughness、unknown、ground confidence 等地形语义。
3. RC-ESDF 层负责 signed distance、gradient、clearance 和局部本体安全查询。
4. footprint safety 不能长期只看质心点 clearance，必须逐步接入车体轮廓扫掠检查。

当前 MuJoCo 地形 / ESDF 输入链路的关键事实：

1. `terrain_analysis` 订阅 `registered_scan` 和 `lidar_odometry`，发布 `/terrain_map`，frame 为 `odom`。
2. `terrain_analysis_ext` 订阅 `registered_scan`、`lidar_odometry` 和 `/terrain_map`，发布 `/terrain_map_ext`、`/traversability_grid`、`/traversability_slope_grid` 等，frame 为 `odom`。
3. `trajectory_optimizer/Nav2BSplineSmoother` 当前主链 `esdf_source` 为 `traversability_grid`，因此 ESDF 方向首先取决于 `traversability_grid` 的 ROS `OccupancyGrid` 索引是否符合 `x + y * width`。
4. `terrain_analysis_ext` 内部平面 voxel 使用 `planarVoxelWidth * indX + indY`，发布 ROS `OccupancyGrid` 时必须转换为 `indX + indY * width`。这是防止 ESDF 方向相对点云发生转置的关键约束。
5. 如果后续重构 `terrain_analysis_ext` 或更换 ESDF 后端，需要保留上述转换或整体改为 ROS 标准索引，不可只改一半。

## 4. 当前核心包职责

### 4.1 trajectory_optimizer

路径：

`src/ats_sentry_nav/trajectory_optimizer`

当前构建目标：

1. `trajectory_optimizer_node`
2. `trajectory_speed_governor_node`
3. `trajectory_optimizer/Nav2BSplineSmoother` Nav2 smoother 插件

当前源码分层：

1. `src/bspline`
   B-spline path optimizer。
2. `src/esdf`
   ESDF provider。
   `rc_traversability_esdf_provider` 是当前 RC-ESDF-lite 主入口；
   `terrain_pointcloud_esdf_provider` 是点云 ESDF 入口；
   `fake_costmap_esdf_provider` 只是 costmap 近似 ESDF 的 fallback/debug adapter。
3. `src/nav2`
   Nav2 smoother 插件 `Nav2BSplineSmoother`。
4. `src/control`
   `trajectory_speed_governor`，根据 `trajectory_profile` 对 controller 输出限速。
5. `src/nodes`
   ROS2 节点胶水。

当前参数事实：

1. 主链 smoother 使用 `esdf_source: traversability_grid`。
2. 主链 smoother 订阅 `terrain_map_ext`、`traversability_grid`、`traversability_slope_grid`。
3. 主链 smoother 发布 `trajectory_profile`。
4. `trajectory_optimizer_node` 发布 `local_elastic_path` 和 `trajectory_profile_visual`；前者经 `FollowElasticPath` 进入 controller，后者仍用于观察。
5. 当前已启用 `use_esdf_obstacle_cost`、坡度速度/加速度限制、footprint cost 采样与局部退化逻辑，但这仍不是最终 MINCO/footprint SDF 实现。

### 4.2 minco_planner

路径：

`src/ats_sentry_nav/minco_planner`

当前构建目标：

`minco_planner_node`

当前源码分层：

1. `src/planning`
   `grid_astar` 前端搜索骨架。
2. `src/trajectory`
   `minco_trajectory_optimizer`、`reference_trajectory`、`yaw_spline_planner`。
3. `src/safety`
   `footprint_safety_checker`、`local_collision_repair`。
4. `src/debug`
   `planner_debug_visualizer`。
5. `src/nodes`
   `minco_planner_node`。

当前默认 topic：

1. `grid_topic: traversability_grid`
2. `goal_topic: goal_pose`
3. `raw_path_topic: minco/raw_path`
4. `reference_path_topic: minco/reference_path`
5. `debug_marker_topic: minco/debug_markers`
6. `global_frame: map`
7. `robot_frame: base_link`

当前状态判断：

1. `minco_planner` 已经是合理分包骨架。
2. 真实 MINCO 后端仍未接入。
3. JPS 尚未替代当前 A* 骨架。
4. footprint safety 和 local collision repair 还需要接真实 RC-ESDF/footprint SDF，并在 MuJoCo 场景中验证。

后续参考迁移重点：

1. MINCO：`~/参考/src/DDR-opt/back_end/include/gcopter/minco.hpp`
2. footprint SDF：`~/参考/src/DDR-opt/utils/plan_env/src/rc_footprint_collision.cpp`
3. MPC / MuJoCo：`~/参考/src/nullspace_mpc`、`~/参考/src/swerve_drive`、`~/参考/src/MuJoCo-LiDAR`

### 4.3 ats_nav2_plugins

路径：

`src/ats_sentry_nav/ats_nav2_plugins`

当前用于 Nav2 自定义插件，包括 costmap layer 和 behavior。参数中可以看到：

1. `ats_nav2_costmap_2d::IntensityVoxelLayer`
2. `ats_nav2_behaviors/BackUpFreeSpace`

这些插件仍属于当前 Nav2 过渡主链的一部分。后续引入 MINCO/MPC 时，不应直接删除现有插件，而应先明确哪些功能被新链路替代，哪些仍作为安全 fallback 保留。

### 4.4 sentry_chassis_vel_transform 与 fake_vel_transform

当前存在两级速度坐标变换：

1. `fake_vel_transform`
   把 Nav2 输出从 `cmd_vel_nav2_result` 转到大 yaw 导航参考系 `cmd_vel_gimbal_yaw_odom`，并维护 `gimbal_yaw_fake`。
2. `sentry_chassis_vel_transform`
   结合实车 gimbal joint state，把 `cmd_vel_gimbal_yaw_odom` 转成底盘 `/cmd_vel`。

MuJoCo 中则通过 `twist_to_motion_ctrl` 把 `cmd_vel_gimbal_yaw_odom` 转为 `/motion_control`。

因此后续 SE2 MPC 接入时必须先决定输出接在哪一层：

1. 若输出 `cmd_vel_gimbal_yaw_odom`，可复用实车/MuJoCo 后级转换。
2. 若直接输出 `/motion_control`，需要明确实车 CAN 与 MuJoCo 的接口一致性。
3. 不建议让 MPC 同时绕过多条链路，否则实车与仿真会难以对比。

## 5. 当前已完成

1. 主线命名已迁移到 ATS 前缀，当前代码包以 `ats_` 为主。
2. `trajectory_optimizer` 已按 B-spline、ESDF、Nav2、control、nodes 拆分。
3. `Nav2BSplineSmoother` 已进入 Nav2 主链，并和 `trajectory_speed_governor` 联动。
4. `rc_traversability_esdf_provider` 已作为当前 RC-ESDF-lite provider，主链和旁路 optimizer 都可以使用 `traversability_grid`。
5. `traversability_slope_grid` 已进入 smoother / governor 的速度、加速度约束逻辑。
6. `minco_planner` 已建立 planning、trajectory、safety、debug、nodes 分层骨架。
7. `ats_mujoco_sim` 已迁移进 `src/sim`，具备地图/scene 生成、MuJoCo 底盘、LiDAR、ToF、RViz2 和 Nav2 联调入口。
8. `mujoco_navigation.launch.py` 已可组织 MuJoCo、map_server、Nav2、trajectory optimizer 选项、Twist bridge 和 RViz2。
9. 实车 bringup、MuJoCo bringup、loopback bringup 是后续保留入口；Gazebo bringup 已从当前主线清理。
10. MuJoCo MID360 点云链路已对齐：`/local_pointcloud` 使用 `front_mid360`，`/registered_scan` 使用 `odom`，地形分析输出与 ESDF 输入保持同一方向。
11. `terrain_analysis_ext` 的 traversability / slope / debug `OccupancyGrid` 发布索引已按 ROS 标准修正，避免 ESDF 相对点云转置。
12. `mujoco_navigation.rviz` 已切换为 `nav2_rviz_plugins/GoalTool`，RViz 点目标可进入 Nav2 `/navigate_to_pose` action。
13. `scripts/test_mujoco_nav_chain.sh` 已作为 MuJoCo 导航回归脚本，覆盖传感器、terrain、traversability、Nav2 action 和 `/motion_control`。
14. `ats_mujoco_sim` 已新增 RMUC2026 mesh 场景：`models/rmuc_2026_swerve.xml` 引用 `models/meshes/rmuc2026_v1_2_0.obj`，并保留现有舵轮底盘、MID360、ToF、执行器和传感器。

## 6. 当前未完成与风险

1. `minco_planner` 仍是骨架包，真实 MINCO 后端未接入。
2. 当前前端搜索仍是 `SmacPlanner2D` 和 `grid_astar` 骨架，JPS 尚未成为主链。
3. footprint safety 还没有完全替代当前的中心线/局部 footprint cost 采样，窄门和大角度转向仍需重点验证。
4. SE2 MPC 尚未替换 MPPI，当前控制器仍是 Nav2 MPPI 过渡方案。
5. `fake_costmap_esdf_provider` 只能作为 fallback/debug，不应当作最终 RC-ESDF。
6. MuJoCo 已有入口，但底盘参数、传感器外参、真实 footprint、速度/加速度/角速度约束仍需继续和实车对齐。
7. RMUC2026 mesh 场景已验证可加载和启动，但完整 Nav2 仍需要与 2026 mesh 对齐的 2D/2.5D 导航地图；不能长期用 2025 PGM 代替 2026 场地。
8. `src` 下存在未跟踪的 `__pycache__` 运行缓存。它们未被 git 跟踪，但后续可清理工作区，避免检索噪声。
9. 当前电脑性能较弱，不要使用全工作区高并发构建。
10. `src/*` 目前被 `.gitignore` 忽略，很多源码改动不会出现在 `git diff` 中。交接时必须直接核查文件内容，不能只依赖 git 状态。
11. RViz 人工点目标依赖 `nav2_rviz_plugins`，若某台机器缺少该插件，会表现为工具加载失败；此时应先确认 Nav2 RViz 插件安装和 overlay 环境，而不是回退到 `/goal_pose`。

## 7. 阶段目标与下一阶段任务

### 7.1 当前阶段目标

当前阶段不是立即完成全部 `V1` 终局，而是把 `V1` 过渡链打成稳定基线：

1. 使用 MuJoCo 作为完整仿真主线，形成可复现的导航测试入口。
2. 在 RViz2 中稳定观察 map、TF、点云、traversability、RC-ESDF、Nav2 plan、B-spline path、trajectory profile、限速 marker。
3. 固化实车和 MuJoCo 的统一控制接口，避免规划层直接绑定仿真内部实现。
4. 证明当前 `SmacPlanner2D + Nav2BSplineSmoother + trajectory_speed_governor + MPPI` 在 MuJoCo 中能闭环跑通。
5. 找出当前 RC-ESDF-lite 是否仍存在原理性问题，尤其是 unknown、坡度、障碍膨胀、signed distance、footprint clearance 的定义。

当前阶段验收标准：

1. `mujoco_navigation.launch.py` 能在低性能电脑上用 CPU LiDAR 后端启动。
2. RViz2 能看到定位、地图、点云、路径、profile 和关键调试 marker。
3. `/motion_control` 能驱动 MuJoCo 底盘响应 Nav2 输出。
4. `trajectory_profile` 能实际进入 `trajectory_speed_governor`，而不是只作为旁路可视化。
5. 发现问题时能判断属于仿真、TF/时间戳、地形语义、ESDF、规划、平滑、限速或控制哪一层。
6. RViz2 使用 `GoalTool` 发布 Nav2 action 目标后，`/navigate_to_pose` 能 accepted 并产生 `/cmd_vel_nav2_result` 与 `/motion_control`。
7. `/traversability_grid` 和 `/traversability_slope_grid` 在 RViz 中与 `/registered_scan`、`/terrain_map_ext` 方向一致。

### 7.2 下一阶段任务

下一阶段建议按下面顺序执行，不要同时大范围重构多包：

1. `任务 1：MuJoCo 闭环稳定化`
   低性能默认参数、LiDAR/ToF topic、TF、`/motion_control`、RViz2 显示全部跑通。
2. `任务 2：RC-ESDF-lite 观测与修正`
   对齐 `traversability_grid`、`traversability_slope_grid`、`terrain_map_ext`，检查 signed distance、unknown、坡度阈值、障碍膨胀和 gradient。
3. `任务 3：footprint-aware 安全检查`
   先在 `trajectory_optimizer` / smoother 侧验证 footprint clearance，再迁移到 `minco_planner/safety`。
4. `任务 4：minco_planner 骨架接真实数据`
   让 `grid_astar -> raw_path -> reference_path -> debug_marker` 基于当前 `traversability_grid` 跑通。
5. `任务 5：接入真实 MINCO`
   参考 `~/参考/src/DDR-opt/back_end/include/gcopter/minco.hpp`，替换当前 placeholder optimizer。
6. `任务 6：独立 yaw + local collision repair`
   让 yaw、footprint safety 和局部修补形成可重复验证链。
7. `任务 7：JPS 替换 A* / grid search`
   在接口稳定后替换前端搜索，不要过早和 MINCO 同时改。
8. `任务 8：SE2 MPC 接入 MuJoCo 与实车接口`
   先在 MuJoCo 验证 MPC 轨迹跟踪，再决定实车输出接 `cmd_vel_gimbal_yaw_odom` 还是更底层控制接口。

### 7.3 P0：固定当前可观察闭环

目标：先保证每次调试都能复现同一条链路。

优先验证：

1. MuJoCo 能发布 `/localization`、`/lidar_odometry`、`/local_pointcloud`、`/registered_scan`。
2. RViz2 能看到 map、TF、机器人、点云、Nav2 plan、smoothed path、trajectory profile marker。
3. `cmd_vel_gimbal_yaw_odom` 能通过 `twist_to_motion_ctrl` 驱动 `/motion_control`。
4. 当前主链的 `trajectory_profile` 能被 `trajectory_speed_governor` 使用。
5. RViz2 中使用 `Navigation 2 GoalTool` 点目标，确认目标进入 `/navigate_to_pose` action，而不是只在 `/goal_pose` topic 上出现。
6. 每次改 MuJoCo、terrain、ESDF、RViz、Nav2 参数后，至少跑一次 `scripts/test_mujoco_nav_chain.sh`。
7. 涉及 footprint、坡度、ESDF 或控制器的改动，后续应在 RMUC2026 mesh 场景上复查 MID360 点云、terrain map、traversability 和控制输出。

### 7.4 P1：强化 RC-ESDF-lite 的真实性

目标：让 ESDF 不只是能跑，而是符合当前地面机器人任务。

重点：

1. 检查 `traversability_grid`、`traversability_slope_grid` 和 `terrain_map_ext` 的 frame、分辨率、时间戳。
2. 明确 free/occupied/unknown 的 signed distance 编码。
3. 检查坡度障碍阈值和速度限制是否与 `terrain_analysis_ext` 对齐。
4. 让 RViz2 能稳定观察 clearance、slope、profile、限速 marker。
5. 开始补 footprint-aware 查询，不再只依赖质心 clearance。

### 7.5 P2：让 minco_planner 从骨架变成可验证节点

推荐顺序：

1. 先让 `grid_astar -> reference_path -> debug_marker` 在当前 `traversability_grid` 上稳定跑通。
2. 接入真实 MINCO 位置轨迹优化。
3. 接入独立 yaw planner，输出可用于 footprint 检查和控制器的姿态参考。
4. 接入 footprint safety checker 和 local collision repair。
5. 在 MuJoCo 中验证窄门、贴边、大角度转向。
6. 最后再把 A* / grid search 替换成 JPS。

### 7.6 P3：从 MPPI 过渡到 SE2 MPC

目标：最终让控制器消费 `minco_planner` 输出的轨迹，而不是长期依赖 Nav2 smoother + MPPI。

需要先明确：

1. MPC 输入轨迹格式：位置、yaw、速度、加速度、时间戳。
2. MPC 输出 topic：优先复用 `cmd_vel_gimbal_yaw_odom` 或统一后的控制抽象。
3. MuJoCo `/motion_control` 与实车 `/cmd_vel` 或 CAN 控制接口的边界。
4. RViz2 和日志中能区分规划失败、ESDF 不可信、footprint 碰撞、控制跟踪失败。

## 8. 低性能电脑构建与启动命令

不要全工作区并行构建。默认使用单包、单 worker、单 job：

```bash
MAKEFLAGS=-j1 colcon build --packages-select ats_mujoco_sim --parallel-workers 1
MAKEFLAGS=-j1 colcon build --packages-select trajectory_optimizer --parallel-workers 1
MAKEFLAGS=-j1 colcon build --packages-select minco_planner --parallel-workers 1
```

加载环境：

```bash
source install/setup.bash
```

随机地图基线入口：

```bash
ros2 launch ats_mujoco_sim mujoco_navigation.launch.py \
  use_rviz:=true \
  use_viewer:=false \
  show_viewer:=false \
  enable_lidar:=true \
  lidar_backend:=cpu \
  lidar_downsample:=24 \
  enable_tof:=true \
  launch_nav2:=true \
  launch_twist_bridge:=true
```

RMUC2026 完整导航与回归入口：

```bash
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  use_viewer:=false \
  show_viewer:=false \
  launch_mujoco_rviz:=false \
  launch_trajectory_optimizer:=true \
  enable_lidar:=true \
  lidar_backend:=cpu \
  lidar_downsample:=24 \
  enable_tof:=false

scripts/test_mujoco_nav_chain.sh
```

RMUC2026 mesh 场景加载测试：

```bash
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  use_viewer:=false \
  show_viewer:=false \
  launch_mujoco_rviz:=false \
  enable_lidar:=false \
  enable_tof:=false
```

该脚本会自动使用无 RViz / 无 viewer 模式启动 MuJoCo 导航链，并检查：

1. `/localization`
2. TF `odom -> gimbal_yaw_odom`
3. TF `gimbal_yaw_odom -> front_mid360`
4. `/local_pointcloud`
5. `/registered_scan`
6. `/terrain_map`
7. `/terrain_map_ext`
8. `/traversability_grid`
9. `/traversability_slope_grid`
10. Nav2 lifecycle active
11. `/navigate_to_pose` action accepted / succeeded
12. `/cmd_vel_nav2_result`
13. `/motion_control`

RViz2 手动导航测试：

```bash
ros2 launch ats_mujoco_sim mujoco_navigation.launch.py \
  use_rviz:=true \
  use_viewer:=false \
  show_viewer:=false \
  enable_lidar:=true \
  lidar_backend:=cpu \
  lidar_downsample:=24 \
  enable_tof:=true \
  launch_nav2:=true \
  launch_twist_bridge:=true
```

启动后在 RViz2 工具栏选择 `Nav2 Goal` / `GoalTool` 类工具点目标。若只看到旧的 `2D Goal Pose` 并发布 `/goal_pose`，说明 RViz 配置或插件环境没有加载到当前 `mujoco_navigation.rviz`。

RMUC2026 mesh 场景手动 MuJoCo/RViz2 测试：

```bash
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  use_viewer:=true \
  launch_mujoco_rviz:=true \
  start_x:=-12.0 \
  start_y:=-6.0 \
  start_z:=0.20 \
  start_yaw:=0.0 \
  enable_lidar:=true \
  lidar_backend:=cpu \
  lidar_downsample:=24 \
  enable_tof:=false
```

该入口使用 `rmuc_2026_swerve.xml`、`rmuc_2026.yaml` 和完整 Nav2 map server / planner / controller 链。回归脚本默认使用该入口，并验证 `local_elastic_path` 与事件触发的全局路径行为。

只看 MuJoCo 传感器和 RViz2：

```bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py \
  use_rviz:=true \
  use_viewer:=false \
  show_viewer:=false \
  enable_lidar:=true \
  lidar_backend:=cpu \
  lidar_downsample:=24 \
  enable_tof:=true
```

实车主入口：

```bash
ros2 launch ats_sentry_bringup bringup.launch.py \
  world:=rmul \
  use_rviz:=true \
  launch_trajectory_optimizer:=false
```

检查关键 topic：

```bash
ros2 topic list | rg 'localization|lidar_odometry|local_pointcloud|registered_scan|terrain_map|traversability|trajectory_profile|cmd_vel|motion_control'
ros2 topic hz /localization
ros2 topic hz /local_pointcloud
ros2 topic hz /trajectory_profile
```

检查旧命名残留：

```bash
rg -n "pb2025|pb_rm_interfaces|pb_nav2_plugins|pb_teleop_twist_joy" src docs --glob '!build/**' --glob '!install/**' --glob '!log/**' || true
```

## 9. 新对话起手清单

新对话继续优化时，建议按这个顺序：

1. 读本文档，先确认当前主链不是最终 MINCO/MPC，而是 Nav2 + B-spline + MPPI 过渡链。
2. 读 `src/ats_sentry_bringup/params/node_params.yaml`，确认当前实车参数和 topic 事实。
3. 读 `src/ats_sentry_nav/ats_nav_bringup/launch/navigation_launch.py`，确认 Nav2 实际启动节点。
4. 读 `src/sim/ats_mujoco_sim/launch/mujoco_navigation.launch.py`，确认 MuJoCo 完整测试入口。
5. 读 `src/ats_sentry_nav/trajectory_optimizer/README.md` 和源码分层，确认当前 smoother/ESDF/governor。
6. 读 `src/ats_sentry_nav/minco_planner/README.md` 和 `config/minco_planner.yaml`，确认下一阶段目标骨架。
7. 先单包构建和启动 MuJoCo/RViz2，再进入 RC-ESDF、MINCO、footprint、MPC 的具体实现。
