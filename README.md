# ATS 2026 Sentry Workspace

安徽信息工程学院 Artisans 战队 2026 哨兵机器人 ROS 2 工作区。

当前仓库以实机总启动、行为树决策、Nav2 导航执行、轻量 loopback 仿真和上下位机串口桥接为主线，所有说明以当前工作区代码、launch 文件和参数文件为准。

## 项目概览

当前主线由 5 层组成：

1. 启动编排层：`src/ats_sentry_bringup`
2. 决策层：`src/ats_sentry_behavior`
3. 导航与定位层：`src/ats_sentry_nav`
4. 轻量闭环仿真层：`src/loopback_sim`
5. 串口与裁判系统接口层：`src/standard_robot_pp_ros2`

当前默认执行链为：

```text
ats_sentry_bringup/bringup.launch.py
  -> standard_robot_pp_ros2
  -> ats_nav_bringup
  -> ats_sentry_behavior
  -> /navigate_through_poses
  -> SmacPlannerHybrid
  -> Nav2BSplineSmoother
  -> MPPI Controller
  -> trajectory_speed_governor
  -> velocity_smoother
  -> fake_vel_transform
  -> /cmd_vel
  -> 下位机底盘 / loopback_sim
```

当前导航地图/轨迹优化过渡链为：

```text
registered_scan / lidar_odometry
  -> terrain_analysis
  -> terrain_analysis_ext
  -> terrain_map_ext
  -> traversability_grid
  -> traversability_height_diff_grid / traversability_occupancy_ratio_grid / traversability_ground_confidence_grid
  -> signed Traversability ESDF
  -> Nav2BSplineSmoother / trajectory_optimizer_node
  -> trajectory_profile / trajectory_esdf_debug
```

当前 `trajectory_optimizer` 主线分工是：

1. `Nav2BSplineSmoother` 和 `trajectory_optimizer_node` 都支持 `esdf_source: traversability_grid`
2. `TraversabilityEsdfProvider` 会融合 `traversability_grid` 与三类地形语义调试栅格
3. `fake_costmap` 和 `terrain_pointcloud` ESDF 仍保留为 fallback / 对照后端

当前导航恢复链为：

```text
FollowPath 失败
  -> behavior_server / BackUpFreeSpace
  -> 局部走廊搜索
  -> 必要时 centroid fallback
  -> 速度平滑与高 cost 自动降速
```

## 工作区结构

当前根目录中与维护直接相关的内容：

```text
.
├── build.sh                            # 推荐构建脚本
├── mapping.sh                          # 建图 + 保存地图/PCD 辅助脚本
├── NAV2.sh                             # 实机导航辅助脚本
├── docs/                               # 项目专项文档
├── src/
│   ├── ats_sentry_bringup/         # 实机与 loopback 总入口、参数、地图、RViz
│   ├── ats_sentry_behavior/        # 行为树、视觉接管、姿态切换、路径输出
│   ├── ats_sentry_nav/             # Nav2、平滑、定位、点云、恢复插件
│   ├── loopback_sim/                  # 轻量软件闭环仿真
│   ├── standard_robot_pp_ros2/        # 串口桥、裁判系统、底盘命令接口
│   ├── interfaces/                    # ats_rm_interfaces / sp_msgs
│   └── tools/                         # pcd2pgm、rosbag recorder、键盘云台控制等
├── install/
└── log/
```

## 环境要求

根据当前代码和构建脚本，推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- GCC / G++ 11
- CMake 3.16+
- Python 3

至少需要的常用系统依赖：

```bash
sudo apt update
sudo apt install -y \
  git git-lfs curl wget \
  python3-pip python3-vcstool python3-rosdep \
  build-essential cmake pkg-config \
  libeigen3-dev libomp-dev
```

首次使用 ROS 2 工作区时：

```bash
sudo rosdep init
rosdep update
```

## 仓库清单与异地部署

当前工作区支持用根目录的 [dependencies.repos](./dependencies.repos) 作为 vcstool 清单重建 `src/`。新机器上可以先 clone 这个工作区壳仓，再导入各功能仓和第三方依赖：

```bash
git clone git@github.com:liukong1220/ATS_2026_snetry_test.git
cd ATS_2026_snetry_test
tools/import_workspace_repos.sh --shallow
```

`--shallow` 会避免拉取完整历史，适合只部署不开发的机器；开发机可以去掉 `--shallow` 保留完整提交历史。

如果要把当前大仓拆成独立仓库，先在 GitHub 的 `liukong1220` 命名空间创建 `dependencies.repos` 中列出的自有仓库，然后执行：

```bash
# 保留每个目录自己的相关历史
tools/export_workspace_repos.sh --mode subtree --push

# 或者只保留当前快照，历史最轻
tools/export_workspace_repos.sh --mode snapshot --push
```

等这些独立仓库都能被 `vcs import` 正常拉取后，再把根仓中已拆出去的 `src/...` 目录从索引移除，只保留清单、脚本和文档。这样根仓后续 clone 的历史会明显变小；已经写进旧大仓的历史不会因为新增 `.repos` 自动消失，若要彻底缩小旧仓包体，需要另建干净壳仓或重写历史。

## 构建

### 推荐方式

当前推荐直接使用根目录脚本：

```bash
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
```

`build.sh` 当前会：

1. 强制切回工作区根目录
2. 自动 `source /opt/ros/${ROS_DISTRO}/setup.bash`
3. 清理旧 overlay 环境变量
4. 先单独编译重包：
   `livox_ros_driver2`、`point_lio`、`small_gicp_relocalization`、`terrain_analysis`、`terrain_analysis_ext`
5. 再编译剩余包

### 手动构建

如果要手动安装依赖并编译：

```bash
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 低性能机器单包构建

如果电脑内存或 CPU 余量较小，不建议直接全工作区并行构建。可以按依赖顺序单包构建，并同时限制 colcon worker 和 CMake 底层并行度：

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --packages-select sp_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --packages-select trajectory_optimizer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash
```

注意：`--parallel-workers 1` 只限制 colcon 同时构建几个包，`CMAKE_BUILD_PARALLEL_LEVEL=1` 才会限制单个包内部的 `cmake --build` 并行度。低性能机器上两者都建议设置。

## 当前主要参数入口

当前最重要的参数文件如下：

- 实机总入口参数：
  [src/ats_sentry_bringup/params/node_params.yaml](./src/ats_sentry_bringup/params/node_params.yaml)
- 实机行为树参数：
  [src/ats_sentry_behavior/params/sentry_behavior.yaml](./src/ats_sentry_behavior/params/sentry_behavior.yaml)
- loopback 行为树参数：
  [src/ats_sentry_behavior/params/sentry_behavior_loopback.yaml](./src/ats_sentry_behavior/params/sentry_behavior_loopback.yaml)
- 视觉专测行为树参数：
  [src/ats_sentry_behavior/params/sentry_behavior_vision_test.yaml](./src/ats_sentry_behavior/params/sentry_behavior_vision_test.yaml)
- loopback Nav2 参数：
  [src/loopback_sim/params/nav2_params.yaml](./src/loopback_sim/params/nav2_params.yaml)
- `ats_nav_bringup` reality 默认参数：
  [src/ats_sentry_nav/ats_nav_bringup/config/reality/nav2_params.yaml](./src/ats_sentry_nav/ats_nav_bringup/config/reality/nav2_params.yaml)
- 串口桥默认参数：
  [src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml](./src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml)

## 运行

### 实机总入口

当前推荐的实机总入口是：

- [src/ats_sentry_bringup/launch/bringup.launch.py](./src/ats_sentry_bringup/launch/bringup.launch.py)

示例：

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  slam:=False \
  use_rviz:=True
```

这个入口会同时启动：

1. `standard_robot_pp_ros2`
2. `ats_nav_bringup` 实机导航链
3. `ats_sentry_behavior`
4. RViz（可选）
5. `rosbag2_composable_recorder`（由 `node_params.yaml` 控制）

### 建图

当前推荐使用根脚本：

```bash
./mapping.sh <MAP_NAME>
```

它会调用：

```bash
ros2 launch ats_sentry_bringup bringup.launch.py \
  world:=<MAP_NAME> \
  slam:=True \
  use_rviz:=True
```

退出时脚本会提示是否：

1. 保存栅格地图到 `src/ats_sentry_bringup/map/<MAP_NAME>.{yaml,pgm}`
2. 复制最新 Point-LIO PCD 到 `src/ats_sentry_bringup/pcd/<MAP_NAME>.pcd`

### loopback 通用决策仿真

入口：

- [src/ats_sentry_bringup/launch/loopback_decision_sim.launch.py](./src/ats_sentry_bringup/launch/loopback_decision_sim.launch.py)

示例：

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

### loopback 视觉专测

入口：

- [src/ats_sentry_bringup/launch/loopback_vision_test.launch.py](./src/ats_sentry_bringup/launch/loopback_vision_test.launch.py)

示例：

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup loopback_vision_test.launch.py \
  use_rviz:=True \
  publish_referee_inputs:=True \
  current_hp:=400 \
  projectile_allowance_17mm:=200 \
  publish_vision_target:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_has_target_position_map:=True \
  vision_target_position_map_frame:=map \
  vision_target_position_map_x:=5.0 \
  vision_target_position_map_y:=2.0 \
  vision_target_position_map_z:=0.0 \
  vision_target_yaw:=0.30 \
  vision_target_pitch:=-0.06
```

### loopback 纯导航观察

入口：

- [src/ats_sentry_bringup/launch/loopback_nav_only.launch.py](./src/ats_sentry_bringup/launch/loopback_nav_only.launch.py)

示例：

```bash
source install/setup.bash
ros2 launch ats_sentry_bringup loopback_nav_only.launch.py use_rviz:=True
```

这个入口更适合单独观察：

- `plan`
- `smoothed_path_visual`
- `trajectory_profile_markers`
- `trajectory_esdf_debug`
- `traversability_grid`
- `traversability_height_diff_grid`
- `traversability_occupancy_ratio_grid`
- `traversability_ground_confidence_grid`
- `back_up_free_space_markers`
- `/cmd_vel_controller`
- `/cmd_vel_controller_governed`
- `/cmd_vel_nav2_result`

## 常见链路说明

### 姿态模式

当前行为树通过 `decision/robot_mode` 发布姿态模式，下位机串口桥最终写入：

- `move = 3`
- `attack = 1`
- `defend = 2`

对应协议字段在：

- [src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp](./src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp)

### 视觉融合

当前视觉侧与行为树、导航的消息契约使用：

- [src/interfaces/sp_msgs](./src/interfaces/sp_msgs)

核心消息为：

- [src/interfaces/sp_msgs/msg/VisionTargetMsg.msg](./src/interfaces/sp_msgs/msg/VisionTargetMsg.msg)

### 恢复行为

当前 `BackUpFreeSpace` 已支持：

1. 主走廊搜索
2. centroid fallback
3. 平均走廊代价高时自动降速
4. RViz marker 区分主方案与 fallback 方案

如果 `behavior_server.visualize: true`，可在 RViz 观察：

- `back_up_free_space_markers`

## 文档导航

当前建议阅读顺序：

1. [docs/总览.md](./docs/总览.md)
2. [docs/nav2_to_3desdf_minco_mpc_optimization_direction.md](./docs/nav2_to_3desdf_minco_mpc_optimization_direction.md)
3. [docs/gazebo_sim_integration.md](./docs/gazebo_sim_integration.md)
4. [docs/esdf_special_sim_observation_plan.md](./docs/esdf_special_sim_observation_plan.md)
5. [docs/mppi_parameter_tuning_guide.md](./docs/mppi_parameter_tuning_guide.md)
6. [docs/omni_recovery_smoothing_optimization.md](./docs/omni_recovery_smoothing_optimization.md)
7. [docs/融合.md](./docs/融合.md)
8. [docs/sentry_bt_decision_checklist.md](./docs/sentry_bt_decision_checklist.md)
9. [docs/sentry_posture_switch_logic.md](./docs/sentry_posture_switch_logic.md)
10. [docs/视觉跟随仿真调试.md](./docs/视觉跟随仿真调试.md)
11. [docs/实机视觉跟随优化方案.md](./docs/实机视觉跟随优化方案.md)
12. [docs/上车测试清单.md](./docs/上车测试清单.md)

说明：

- `docs/interview_prep.md` 体量较大，当前更像内部资料，不作为主线运维文档入口。
- `docs/navigate_through_poses_migration_checklist.md`、`docs/slim_loopback_refactor.md` 更偏迁移/重构说明，适合作为背景资料。

## 维护约定

1. 修改主启动逻辑、参数入口或地图/PCD 目录时，优先同步本 README 与 `docs/总览.md`
2. 修改行为树决策、视觉接管、姿态切换时，优先同步 `ats_sentry_behavior/README.md` 与 `docs/融合.md`、`docs/sentry_posture_switch_logic.md`
3. 修改 Nav2 参数、恢复行为、轨迹优化时，优先同步 `ats_sentry_nav/README.md` 与 `docs/mppi_parameter_tuning_guide.md`、`docs/omni_recovery_smoothing_optimization.md`
4. 若文档内容无法从当前仓库代码、参数或 launch 中直接确认，应明确标注“待补充”或“需要人工确认”

## 待人工确认

以下内容当前无法仅从本仓库直接严格确认，后续如需写入正式对外文档，应由维护者补充：

1. 实车底盘、电控和传感器的最终硬件型号清单
2. 现场网络、交换机、串口适配器和供电拓扑
3. 比赛现场使用的固定地图名与 PCD 文件命名规范
4. 真实视觉算法包 `sp_vision25` 的完整运行依赖、构建方式和部署步骤
