# ATS 2026 Sentry Workspace

安徽信息工程学院 Artisans 战队 2026 哨兵机器人 ROS 2 工作区。

当前仓库以实机总启动、行为树决策、Nav2 导航执行、轻量 loopback 仿真和上下位机串口桥接为主线，所有说明以当前工作区代码、launch 文件和参数文件为准。

## 项目概览

当前主线由 5 层组成：

1. 启动编排层：`src/ats_sentry_bringup`
2. 决策层：`src/ats_sentry_behavior`
3. 导航与定位层：`src/ats_sentry_nav`
4. 轻量闭环仿真层：`src/sim/loopback_sim`
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
│   ├── ats_sentry_bringup/           # 根仓保留：实机与 loopback 总入口、参数、地图、RViz
│   ├── ats_sentry_behavior/          # 行为树、视觉接管、姿态切换、路径输出
│   ├── ats_sentry_nav/               # Nav2、平滑、定位、点云、恢复插件、底盘速度坐标转换
│   │   └── sentry_chassis_vel_transform/
│   ├── sim/                          # 仿真域
│   │   ├── ats_mujoco_sim/
│   │   ├── loopback_sim/
│   │   └── rmu_gazebo_simulator/
│   ├── standard_robot_pp_ros2/        # 串口桥、裁判系统、底盘命令接口
│   ├── interfaces/                    # ats_rm_interfaces / sp_msgs / carstatemsgs / manda_can_control
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

## 仓库组织与异地部署

当前仓库采用“根仓保留总启动 + vcstool 清单 + 功能完整分包”的方式组织。

根仓 `ATS_2026_snetry_test` 直接保留：

- 工作区说明、部署脚本、构建脚本和 [dependencies.repos](./dependencies.repos)
- `src/ats_sentry_bringup`

`src/ats_sentry_bringup` 不再拆成独立仓库。原因是它不是普通算法包，而是实机和仿真的总入口，集中维护：

- `bringup.launch.py`、loopback、Gazebo、视觉专测等 launch 入口
- 实机 `node_params.yaml`、MID360 配置、RViz 视图
- 比赛/测试地图资产，以及 PCD 的本地目录约定
- `mapping.sh`、`NAV2.sh` 等根脚本实际依赖的路径约定

这部分留在根仓后，新机器克隆根仓即可获得可启动的部署入口；再通过 `dependencies.repos` 拉取行为、导航、接口、仿真和第三方依赖，工作区才完整。

`src/ats_sentry_bringup` 对应的独立远端仓库/分支不再维护，也不应重新加入 `dependencies.repos`。它随根仓 `ATS_2026_snetry_test` 一起被 `colcon` 发现和构建。

`dependencies.repos` 中按功能域维护以下路径：

- 主线功能域：`src/ats_sentry_nav`、`src/ats_sentry_behavior`、`src/standard_robot_pp_ros2`
- 仿真域：`src/sim/ats_mujoco_sim`、`src/sim/loopback_sim`、`src/sim/rmu_gazebo_simulator`
- 接口域：`src/interfaces`、`src/interfaces/carstatemsgs`、`src/interfaces/manda_can_control`
- 导航辅助域：`src/ats_sentry_nav/sentry_chassis_vel_transform`
- 机器人描述：`src/ats_robot_description`
- 第三方依赖仓库：继续指向原始上游，避免把外部代码重复塞进根仓

开源部署约定：

- 根仓和自研分包仓库面向开源使用，`dependencies.repos` 统一使用 `https://github.com/...` URL。
- 新机器不需要配置 GitHub SSH key 就能执行 `tools/import_workspace_repos.sh --shallow`。
- 自研分包仓库应保持 public；如果需要批量创建或修正可见性，使用 `GH_TOKEN=<YOUR_TOKEN> tools/create_github_repos.sh`。
- `src/ats_sentry_bringup/pcd/*.pcd` 只作为本机运行资产，不提交、不上传。

分包边界按“功能完整性”确定，而不是按每个 ROS package 机械拆分。例如 `src/ats_sentry_nav` 内部同时包含 Nav2 bringup、定位、点云转换、地形分析、轨迹优化、恢复插件和底盘速度转换相关工具；`sentry_chassis_vel_transform` 负责底盘速度坐标转换和 fake yaw 相关逻辑，归入导航域后更便于和 `fake_vel_transform`、控制器输出链路一起维护。

### 快速部署

新机器推荐按下面流程创建工作区：

```bash
git clone --depth=1 -b develop https://github.com/liukong1220/ATS_2026_snetry_test.git
cd ATS_2026_snetry_test
git lfs install
tools/import_workspace_repos.sh --shallow
```

说明：

- `git clone --depth=1` 只拉根仓最近一次提交，避免下载旧大仓历史
- `tools/import_workspace_repos.sh --shallow` 会按 `dependencies.repos` 浅克隆除 `ats_sentry_bringup` 之外的功能仓和第三方依赖
- `src/ats_sentry_bringup/pcd/*.pcd` 不提交到 Git；需要实机建图或从队内离线介质拷贝到本地
- 如果你要在部署机器上长期开发，可以去掉 `--shallow`，保留各子仓库完整历史

导入完成后，目录结构会变成：

```text
.
├── dependencies.repos
├── build.sh
├── docs/
├── tools/
└── src/
    ├── ats_sentry_bringup/
    ├── ats_sentry_behavior/
    ├── ats_sentry_nav/
    │   └── sentry_chassis_vel_transform/
    ├── sim/
    │   ├── ats_mujoco_sim/
    │   ├── loopback_sim/
    │   └── rmu_gazebo_simulator/
    ├── standard_robot_pp_ros2/
    ├── dependencies/
    ├── interfaces/
    │   ├── ats_rm_interfaces/
    │   ├── sp_msgs/
    │   ├── carstatemsgs/
    │   └── manda_can_control/
    └── tools/
```

### 开发流程

根仓和拆分功能包是不同 Git 仓库，提交时需要区分：

```bash
# 查看所有子仓状态
vcs status src

# 更新所有子仓
vcs pull src

# 在某个功能包内提交代码
cd src/ats_sentry_nav
git status
git add <files>
git commit -m "<message>"
git push origin develop
```

如果修改的是已经归入导航域的底盘速度坐标转换包，路径是：

```bash
cd src/ats_sentry_nav/sentry_chassis_vel_transform
```

根仓提交这些内容：

- `dependencies.repos`
- 根目录脚本，例如 `build.sh`、`mapping.sh`、`NAV2.sh`
- `tools/` 下的工作区维护脚本
- `docs/` 和 README
- `.gitignore`、`.gitattributes` 等根仓配置
- `src/ats_sentry_bringup` 下的总启动入口、参数、地图和 RViz 配置

拆分功能包源码、第三方依赖、构建产物都不应直接提交到根仓。

如果新增正式地图或 PCD：

- 地图文件放入 `src/ats_sentry_bringup/map`
- PCD 文件放入 `src/ats_sentry_bringup/pcd` 供本机运行使用，但不提交、不上传
- 临时建图结果不要直接提交，先确认命名、场地版本和是否确实要作为部署资产

### 清单维护

新增一个功能域仓库时，先确认它是否应独立于根仓维护。

不应加入 `dependencies.repos` 的内容：

- `src/ats_sentry_bringup`
- 只服务于根仓部署脚本的临时文件
- build/install/log 等构建产物

应加入 `dependencies.repos` 的内容：

- 能独立表达一个功能域的自研仓库
- 需要跟随上游更新的第三方依赖
- 与 bringup 松耦合、可以单独开发和复用的工具仓库

确认需要新增后，创建并推送独立仓库，然后在 [dependencies.repos](./dependencies.repos) 中添加条目：

```yaml
repositories:
  src/example_package:
    type: git
    url: https://github.com/liukong1220/example_package.git
    version: develop
```

如果某个功能包要锁定到确定版本，可以把 `version` 从分支名改成 commit hash：

```yaml
version: 0123456789abcdef0123456789abcdef01234567
```

部署机器要复现固定版本时，优先使用 commit hash；日常开发可以继续使用 `develop`。

### 拆仓维护脚本

仓库内保留了三个维护脚本：

```bash
# 按 dependencies.repos 导入 src/
tools/import_workspace_repos.sh --shallow

# 创建 GitHub 缺失功能仓，需要本地提供 GH_TOKEN 或 GITHUB_TOKEN
GH_TOKEN=<YOUR_TOKEN> tools/create_github_repos.sh

# 若仓库已存在，该脚本会在 token 权限允许时把自研分包仓库修正为 public
GH_TOKEN=<YOUR_TOKEN> MAKE_PUBLIC=1 tools/create_github_repos.sh

# 从一个仍包含 src 源码的旧大仓 checkout 导出独立仓库
tools/export_workspace_repos.sh --mode snapshot --push
```

`tools/create_github_repos.sh` 和 `tools/export_workspace_repos.sh` 都不会再处理 `ats_sentry_bringup`。该包是根仓部署入口，后续直接随根仓提交。

`tools/create_github_repos.sh` 默认创建 public 仓库，并会在 token 权限允许时把已存在的自研分包仓库设置为 public。`tools/export_workspace_repos.sh` 默认使用 `https://github.com/liukong1220/<repo>.git` 作为推送目标。

`tools/export_workspace_repos.sh` 主要用于历史迁移。当前这些自研功能包已经按 snapshot 方式推送到 GitHub，后续日常开发不需要重复执行。

### 旧仓历史说明

根仓当前 HEAD 只跟踪 `src/ats_sentry_bringup`，其余 `src/` 功能域由 `dependencies.repos` 拉取。旧提交里曾经包含过更多源码和依赖，所以普通 clone 仍可能下载旧历史。异地部署时请使用：

```bash
git clone --depth=1 -b develop https://github.com/liukong1220/ATS_2026_snetry_test.git
```

如果要让根仓普通 clone 也彻底变小，需要重写 Git 历史或新建一个全新的壳仓。这会影响所有已有 clone 的同步方式，因此没有在本次迁移中自动执行。

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
6. 默认使用低性能机器策略：`--parallel-workers 1`，单包内部 `CMAKE_BUILD_PARALLEL_LEVEL=2` / `MAKEFLAGS=-j2`

如需临时调整构建强度：

```bash
COLCON_WORKERS=1 BUILD_THREADS=2 ./build.sh
```

如果整理目录或迁移工作区后遇到 CMake cache 记录旧源码路径，可清一次缓存：

```bash
CMAKE_CLEAN_CACHE=1 COLCON_WORKERS=1 BUILD_THREADS=2 ./build.sh
```

### 手动构建

如果要手动安装依赖并编译：

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 低性能机器单包构建

如果电脑内存或 CPU 余量较小，不建议直接全工作区并行构建。当前默认构建强度就是单包双核，也可以按依赖顺序单包构建，并同时限制 colcon worker 和 CMake 底层并行度：

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --symlink-install --packages-select sp_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --symlink-install --packages-select trajectory_optimizer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash
```

注意：`--parallel-workers 1` 只限制 colcon 同时构建几个包，`CMAKE_BUILD_PARALLEL_LEVEL=2` 和 `MAKEFLAGS=-j2` 限制单个包内部最多使用 2 个编译任务。

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
  [src/sim/loopback_sim/params/nav2_params.yaml](./src/sim/loopback_sim/params/nav2_params.yaml)
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
2. [docs/代码范围与目录结构.md](./docs/代码范围与目录结构.md)
3. [docs/启动入口与运行链路.md](./docs/启动入口与运行链路.md)
4. [docs/导航定位与轨迹链路.md](./docs/导航定位与轨迹链路.md)
5. [docs/行为树决策链路.md](./docs/行为树决策链路.md)
6. [docs/接口消息与话题约定.md](./docs/接口消息与话题约定.md)
7. [docs/仿真域说明.md](./docs/仿真域说明.md)
8. [docs/视觉与串口桥说明.md](./docs/视觉与串口桥说明.md)
9. [docs/构建与维护说明.md](./docs/构建与维护说明.md)
10. [docs/nav2_to_3desdf_minco_mpc_optimization_direction.md](./docs/nav2_to_3desdf_minco_mpc_optimization_direction.md)

说明：

- `docs` 已按当前 `src` 目录和功能域重新整理，除 PDF 与 `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md` 外，旧文档不再作为维护入口。
- `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md` 是保留的专项规划文档，用于承接 3D ESDF、MINCO、MPC 等后续方向。

## 维护约定

1. 修改主启动逻辑、参数入口或地图/PCD 目录时，优先同步本 README 与 `docs/总览.md`
2. 修改行为树决策、视觉接管、姿态切换时，优先同步 `ats_sentry_behavior/README.md` 与 `docs/行为树决策链路.md`
3. 修改 Nav2 参数、恢复行为、轨迹优化、底盘速度坐标转换时，优先同步 `ats_sentry_nav/README.md` 与 `docs/导航定位与轨迹链路.md`
4. 修改消息、服务、串口桥或视觉桥接时，优先同步 `docs/接口消息与话题约定.md` 与 `docs/视觉与串口桥说明.md`
5. 修改仿真入口或新增仿真包时，优先同步 `docs/仿真域说明.md`
6. 若文档内容无法从当前仓库代码、参数或 launch 中直接确认，应明确标注“待补充”或“需要人工确认”

## 待人工确认

以下内容当前无法仅从本仓库直接严格确认，后续如需写入正式对外文档，应由维护者补充：

1. 实车底盘、电控和传感器的最终硬件型号清单
2. 现场网络、交换机、串口适配器和供电拓扑
3. 比赛现场使用的固定地图名与 PCD 文件命名规范
4. 真实视觉算法包 `sp_vision25` 的完整运行依赖、构建方式和部署步骤
