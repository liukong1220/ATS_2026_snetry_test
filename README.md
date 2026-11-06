<div align="center">

# 🤖 ATS 2026 SENTRY NAVIGATION

**面向 ATS 四驱四转舵轮哨兵的 ROS 2 自研导航工作区**

<p>
  <img src="https://img.shields.io/badge/C%2B%2B-17%2B-00599C.svg?style=for-the-badge&logo=cplusplus">
  <img src="https://img.shields.io/badge/CMake-3.8%2B-064F8C.svg?style=for-the-badge&logo=cmake">
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E.svg?style=for-the-badge&logo=ros">
  <img src="https://img.shields.io/badge/Linux-Ubuntu%2022.04-E95420.svg?style=for-the-badge&logo=ubuntu">
</p>

<p>
  <img src="https://img.shields.io/badge/Chassis-4WD%20%2B%204WS-5C2D91.svg?style=for-the-badge">
  <img src="https://img.shields.io/badge/Mapping-ROGMap%20%2F%20RC--ESDF-2E7D32.svg?style=for-the-badge">
  <img src="https://img.shields.io/badge/Planning-JPS%20%2B%20MINCO%20%2B%20MPC-B00020.svg?style=for-the-badge">
</p>

<p>
  <img src="https://img.shields.io/github/stars/liukong1220/ATS_2026_snetry_test?style=for-the-badge">
  <img src="https://img.shields.io/github/license/liukong1220/ATS_2026_snetry_test?style=for-the-badge">
  <img src="https://img.shields.io/github/last-commit/liukong1220/ATS_2026_snetry_test?style=for-the-badge">
</p>

</div>

---

# 📌 项目简介

ATS 2026 Sentry Workspace 是 ATS 四驱四转舵轮哨兵的 ROS 2 Humble 工作区。正式链路以
LiDAR-Inertial 定位为状态来源，以 ROGMap 和 RC-ESDF 提供规划环境，以 JPS、MINCO S3、
独立 yaw、footprint gate/Local Collision Repair 产生安全 reference，再由全向 SE2 MPC
输出底盘车体系速度。它保留四舵轮横移能力，不将底盘退化为差速模型。

正式自研导航链：

```text
/localization + /registered_scan
  -> ROGMap -> numeric ground projection -> ROGMap adapter
  -> /rog_map_adapter/planning_snapshot + RC-ESDF
  -> ATS NavigateToPose action / Goal Manager
  -> JPS -> MINCO S3 + independent yaw + footprint safety / repair
  -> committed reference -> omnidirectional SE2 MPC
  -> /cmd_vel/autonomy_raw
  -> (real robot: fake/chassis velocity transform -> /cmd_vel/autonomy)
  -> cmd_vel_arbiter -> /cmd_vel/selected -> lower-controller velocity interface
```

`point_lio` 持续提供 `/localization` 与 `/registered_scan`；ROGMap 不是定位器。
`/rog_map/esdf` 是可视化点云。adapter 在 `ats_rog_map` 进程内直接读取数值状态，通过
`/rog_map_adapter/planning_snapshot` 发布规划快照，不能从点云反解析距离场。

## 📑 目录

- [工作区与仓库边界](#工作区与仓库边界)
- [技术亮点](#技术亮点)
- [系统依赖](#系统依赖)
- [Quick Start](#quick-start)
- [实机部署](#实机部署)
- [MuJoCo 导航仿真](#mujoco-导航仿真)
- [Gazebo 导航仿真](#gazebo-导航仿真)
- [关键接口与所有权](#关键接口与所有权)
- [统一配置](#统一配置)
- [常见问题](#常见问题)
- [目录结构](#目录结构)
- [致谢与许可证](#致谢与许可证)

## 工作区与仓库边界

本目录是 ROS 2 工作区根，同时是编排与文档仓；`src/` 下的主要源码目录各自是独立 Git 仓库，
提交、状态检查和推送建议逐仓执行（清单与分支见 `dependencies.repos`）。

| 仓库 | 路径 | 分支 | 主要职责 |
| :--- | :--- | :--- | :--- |
| 工作区根仓 | `.` | `develop` | `docs/`、`scripts/` 工程工具、`src/ats_sentry_bringup`（实机 bringup、总参数、地图、默认 RViz） |
| 导航仓 | `src/ats_sentry_nav` | `develop` | 定位接入、ROGMap、adapter、RC-ESDF、JPS/MINCO、action、MPC 与 `cmd_vel_arbiter` |
| MuJoCo 仓 | `src/sim/ats_mujoco_sim` | `develop` | 四舵轮物理、传感器、场地、底盘 bridge、仿真 profile 与 launch |
| Gazebo 仓 | `src/sim/gazebo_simulator` | `main` | `rmu_gazebo_simulator` 场地与 `ats_gazebo_nav.launch.py` |
| loopback 仓 | `src/sim/loopback_sim` | `develop` | 轻量运动学/栅格/行为仿真执行端 |
| 串口桥仓 | `src/standard_robot_pp_ros2` | `develop` | 下位机串口、云台状态与最终速度入口 |
| 视觉仓 | `src/sp_vision25` | `de` | 自瞄视觉 |
| 其他 | `src/ats_sentry_behavior`、`src/ats_robot_description`、`src/interfaces`、`src/tools/*` | 见 `dependencies.repos` | 行为树、机器人描述、接口与工具 |

`参考/` 仅用于算法/许可证溯源，不是活动构建输入。所有 `colcon` 命令都建议保留
`--base-paths src`，避免同名参考包进入构建图。

## ✨ 技术亮点

### 概率地图、数值投影与 2.5D 语义

`ats_rog_map` 维护概率占据、膨胀占据和 3D ESDF，由进程内 adapter 读取数值投影，输出二维
occupancy、signed distance 和梯度。adapter 将 ROGMap、terrain/slope 与静态图
保守融合，是 `/rog_map_adapter/planning_snapshot` 的唯一发布者。地图外、unknown、障碍和不可通行地形
都不能被错误地解释为自由空间。

### 同一地图快照上的搜索、轨迹与安全复核

单次规划内，JPS、二维 RC-ESDF、MINCO clearance、footprint gate 与 Local Collision Repair
使用同一 MINCO 本地不可变 snapshot。ROGMap source generation、adapter publication sequence
与 MINCO local snapshot generation 是不同编号域；当前实现不会将它们声明为端到端同号。

### 面向四舵轮的全向控制与失效安全

系统状态为世界系 `[x, y, yaw]`，MPC 命令为车体系 `[vx, vy, wz]`。地图未就绪或过期、定位异常、
无路、轨迹不安全、执行授权失效或 MPC 失败时，安全链输出确定性零速度。急停会清空 tracker，
急停前 reference 不会因 ready 恢复而自动驱动车辆。

### 自研目标 action 与 Nav2-free 正式入口

正式入口使用 `/ats_navigate_to_pose` 自研 action，支持 feedback、result、cancel、preempt 与
timeout。`real_robot_navigation.launch.py` 和 MuJoCo 正式 launch 固定编排自研链，不启动
Nav2 server、costmap、BT navigator，也不消费 `/plan`；这不改变 `Nav2` 可作为历史基线或
参考工程存在的事实。

## 📦 系统依赖

### 基础环境

- Ubuntu 22.04；
- ROS 2 Humble；
- C++17、CMake、Python 3、`colcon`、`rosdep` 与 `vcstool`；
- Eigen3、PCL、OpenCV、yaml-cpp、glog、libunwind；
- LiDAR/IMU、底盘串口与云台的真实设备依赖见根参数和各上游驱动说明；
- MuJoCo、NumPy、SciPy、Pillow、PyYAML（仅物理仿真）。

### 获取完整源码依赖

首次创建工作区时，先克隆根仓，再用根 `dependencies.repos` 导入所有独立源码仓：

```bash
git clone -b develop \
  git@github.com:liukong1220/ATS_2026_snetry_test.git \
  /home/ats/ATS_2026_snetry_test
cd /home/ats/ATS_2026_snetry_test
vcs import --recursive --skip-existing . < dependencies.repos
```

清单包含根 bringup 之外的导航、行为、接口、实机驱动、MuJoCo、loopback、工具，以及优先从
源码构建的依赖。`src/sim/loopback_sim` 被明确保留为自研导航框架的轻量运动学/栅格/行为
仿真域；正式导航运行图仍不启动 Nav2 server、plugin 或 costmap。

导入后先检查分支和缺失仓库：

```bash
vcs status --repos
colcon list --base-paths src --names-only
```

优先让 `rosdep` 从活动源码解析可安装依赖：

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
sudo rosdep init  # 仅首次使用 rosdep 时执行
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep init` 已完成时可直接跳过。Livox SDK、small_gicp、MuJoCo 与可选 LiDAR backend
可能需要按各项目上游说明额外安装，并保持版本兼容。

## ⚡ Quick Start

### 构建整个活动工作区

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
```

若仅构建仿真及其上游依赖，可使用：

```bash
flock /tmp/ats27_build.lock env MAKEFLAGS=-j8 colcon build --base-paths src \
  --packages-up-to ats_mujoco_sim --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

日常改动只重建受影响包：

```bash
flock /tmp/ats27_build.lock env MAKEFLAGS=-j8 colcon build --base-paths src \
  --packages-select minco_planner ats_mujoco_sim --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 不要用 `-UFETCHCONTENT_SOURCE_DIR_QDLDL` 重建 `ats_swerve_mpc`。
- 修改源码、launch 或 YAML 后，先重新编译对应包，再重新启动节点。
- `source install/setup.bash` 时若未设置 `ROS_DOMAIN_ID`，`ats_sentry_bringup` 的 env-hook 会将其设为 `90`。

检查正式入口的参数，不会驱动车辆：

```bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py --show-args
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py --show-args
ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py --show-args
```

## 🚀 实机部署

### 部署前清单

在允许执行任何运动命令前，建议确认：

- `world` 对应的静态地图与 prior PCD 文件存在且坐标系匹配；
- LiDAR/IMU、底盘串口、云台连接、波特率与 `node_params.yaml` 一致；串口桥 `cmd_vel_topic` 为 `/cmd_vel/selected`；
- LiDAR 外参、`map -> odom`、`odom -> gimbal_yaw_odom` 与底盘 frame 已实测标定；
- 物理急停、独立安全员和受限低速区域可用；
- 没有其他节点发布竞争的关键 TF、`/cmd_vel/autonomy_raw`、`/cmd_vel/selected` 或底盘最终输入。

### 正式实机入口

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py \
  world:=rmuc_2026 use_rviz:=true
```

实机默认保留：

```text
launch_fake_vel_transform:=True
launch_chassis_vel_transform:=True
require_gimbal_status:=True
```

这两层速度转换是既有云台/底盘 topic 与 frame 契约的一部分。固定雷达迁移仅通过 launch
参数关闭兼容层，并同时验证下游速度坐标系、TF 和 topic；不能仅关闭 transform 后继续假定
底盘接收到相同 frame 的命令。fake-yaw 关闭时仍须保留
`gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF，且建议避免增加重复的
`base_footprint -> base_link` 发布者。

仅调试导航 action、暂不启动行为树：

```bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py \
  world:=rmuc_2026 launch_behavior:=false use_rviz:=true
```

下列命令会驱动车辆，仅在完成上方安全清单后使用：

```bash
ros2 action send_goal --feedback \
  /ats_navigate_to_pose \
  ats_navigation_interfaces/action/NavigateToPose \
  "{goal_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, timeout: {sec: 60, nanosec: 0}}"
```

## 🧪 MuJoCo 导航仿真

先启动完整导航仿真（物理 + 定位/地图 + 规划/控制），再用 action 下发目标。正式算法链：

```text
Point-LIO (/localization, /registered_scan)
  -> ROGMap / in-process adapter -> /rog_map_adapter/planning_snapshot
  -> JPS + MINCO S3 + independent yaw + footprint gate / Local Collision Repair
  -> omnidirectional SE2 MPC -> /cmd_vel/autonomy_raw
  -> cmd_vel_arbiter -> /cmd_vel/selected
```

### 直接启动（推荐）

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
source install/setup.bash

# 带 MuJoCo viewer（本机可视化，默认 RMUC 2027 场景）
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  use_viewer:=true \
  show_viewer:=true \
  use_rviz:=false

# 无 viewer，仅开 RViz
# ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
#   use_viewer:=false show_viewer:=false use_rviz:=true
```

`rmuc_2026_mujoco.launch.py` 是保留旧文件名的兼容入口，现转发到 `rmuc_2027_mujoco.launch.py`：
默认模型 `rmuc_2027_swerve.xml`、静态图 `rmuc_2027.yaml`、先验 `rmuc_2027.pcd`，出生点为基地
`map (3.37, 9.03, z=0.465)`、yaw `0`。启动参数仍可覆盖这些默认值。
`mujoco_navigation.launch.py` 仍转发到 **2025**；需要旧场地时请显式使用
`rmuc_2025_mujoco.launch.py`，不要混用两个入口。

2026 兼容入口默认：`use_viewer:=false`、`show_viewer:=false`、`use_rviz:=false`、
`launch_physics:=true`、`launch_navigation:=true`、`planning_grid_owner:=rog_map`、`solver_mode:=ilqr`。
上面的命令显式打开 viewer。定位默认是仿真真值 `localization_source:=truth`；需要 Point-LIO + GICP
时追加 `localization_source:=lio lidar_threads:=8`，运行速度会更慢。

另开终端下发目标（会驱动仿真车，确认急停可用后再执行）：

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 action send_goal --feedback \
  /ats_navigate_to_pose \
  ats_navigation_interfaces/action/NavigateToPose \
  "{goal_pose: {header: {frame_id: map}, pose: {position: {x: 9.8, y: 11.7, z: 0.0}, orientation: {w: 1.0}}}, timeout: {sec: 300, nanosec: 0}}"
```

两个终端必须使用同一 `ROS_DOMAIN_ID`；不要与其他仿真实例共用 domain，不使用 `77`。
上例从默认基地前往 `map (9.8, 11.7)`，可按场地中的实际可通行区域修改目标。

### 小陀螺：为什么只启动后车和雷达不旋转？

- **启动不会自动发运动命令**。先等定位、地图健康，再通过上面的 action 或 RViz `2D Goal Pose`
  下发目标。闲置、完成目标或急停状态下不应持续自旋。
- 仿真默认 `spin_enabled:=true spin_rate:=3.0`。2027 入口速度桥默认 `max_angular_z:=3.0`，
  不再被旧的 `2.0 rad/s` 桥限幅截断；这不是提高实车轮速上限，也不保证任何路线都能达到该转速。
- 自旋只在足迹净空允许的开阔段进行；窄段前对齐、隧道内不自旋，出口后的安全开阔段才恢复。
  平移到达终点后需停止自旋并对齐目标 yaw，停稳后 action 才能成功。短路线、净空不足、强制跟随 yaw 或
  自旋参考未通过足迹门禁时可能全程不自旋，不能通过放宽安全门来强制旋转。
- Mid360 **固定在底盘上**，应随车身一起转；没有独立的雷达旋转关节。点云已变换到世界/odom
  坐标时，静态环境点云保持稳定是正常现象，不能靠点云是否转动判断车身是否自旋。
- 可在启动命令后追加 `spin_enabled:=false` 对照；`force_body_yaw_follow:=true` 也会抑制自旋。

另开同一环境的终端，只读观察：

```bash
ros2 param get /minco_planner spin_enabled
ros2 param get /minco_planner spin_rate
ros2 topic echo /simulation/ground_truth/odometry --field twist.twist.angular.z
ros2 topic echo /cmd_vel/selected --field angular.z
```

最后两条各占一个终端，`Ctrl-C` 停止观察，不要向速度话题直接发布命令绕过执行授权。
viewer 应看到车身和固定雷达共同改变朝向；RViz 固定坐标系的静态地图不旋转。
通过 RViz 下发新目标会抢占正在执行的 action，这是正常的交互式接管行为。

更细的依赖、资产与 telemetry 见 [`src/sim/ats_mujoco_sim/README.md`](src/sim/ats_mujoco_sim/README.md)。

## 🧪 Gazebo 导航仿真

同样先 `ros2 launch` 起完整导航链，再手动下发目标。Gazebo 默认 `use_gazebo_gt_odometry:=true`（底盘真值中继到 `/odometry`，并旁路 Point-LIO 位姿）；需要练 LIO 时再显式关掉。

### 直接启动（推荐）

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
source install/setup.bash

# RMUC 2027：只启动 RViz，Gazebo 使用 headless rendering
ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
  world:=rmuc_2027 \
  use_gazebo_gt_odometry:=true \
  use_rviz:=true

# 需要 Gazebo GUI 时显式关闭 headless，并保留 RViz
# ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
#   world:=rmuc_2027 use_gazebo_gt_odometry:=true \
#   headless:=false use_viewer:=true use_rviz:=true
```

服务器或只需显示点云/定位时，先设置可用的 EGL 渲染环境，再使用独立 domain：

```bash
export DISPLAY=:0
export EGL_PLATFORM=surfaceless
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

ROS_DOMAIN_ID=232 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1 \
ros2 launch rmu_gazebo_simulator ats_gazebo_nav.launch.py \
  world:=rmuc_2027 use_sim_time:=true \
  headless:=true headless_rendering:=true use_viewer:=false use_rviz:=false \
  enable_camera_sensors:=false use_gazebo_gt_odometry:=true \
  launch_planning:=false launch_small_gicp_relocalization:=true \
  prior_pcd_file:=src/ats_sentry_bringup/pcd/rmuc_2027.pcd
```

该命令用于确认 Gazebo world、仿真 Mid360、`/registered_scan` 和 GICP 重定位，不会启动规划器。启动后另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=232 ros2 topic hz /registered_scan
ROS_DOMAIN_ID=232 ros2 topic echo --once /relocalization_observation
ROS_DOMAIN_ID=232 ros2 topic echo --once /localization
```

日常完整导航使用第一条命令；下发目标前确认 `/planner/emergency_stop`、`/cmd_vel/selected` 和 RViz 中的地图/定位状态正常。常用参数：`world`、`map_yaml`、`params_file`、`robot_name`、`use_gazebo_gt_odometry`、`launch_small_gicp_relocalization`、`prior_pcd_file`、`launch_planning`、`headless`、`headless_rendering`、`use_viewer`、`use_rviz`。先用 `--show-args` 查看完整列表。

另开终端下发目标（命令与 MuJoCo 相同，仍是 `/ats_navigate_to_pose`）。

## 📡 关键接口与所有权

| 接口 | 唯一权威 producer | 主要 consumer | 契约 |
| :--- | :--- | :--- | :--- |
| `/ats_navigate_to_pose` | `ats_goal_manager` action server | 行为树、目标客户端 | 目标生命周期与失败码 |
| `/rog_map_adapter/planning_snapshot` | `ats_rog_map` 进程内 adapter | MINCO、目标管理、可视化适配 | 唯一规划快照 owner |
| `/minco/reference_path` | Goal Manager 提交点 | MPC、RViz | 复核后统一重定时的 reference |
| `/planner/emergency_stop` | Goal Manager | MPC | heartbeat 急停状态 |
| `/cmd_vel/autonomy_raw` | `ats_swerve_mpc` | 速度变换或 arbiter | 车体系 `[vx, vy, wz]` 的唯一自主源 |
| `/cmd_vel` | 键鼠等手动源 | `cmd_vel_arbiter` | 保持 ROS 默认输入；新鲜时优先 |
| `/cmd_vel/autonomy` | `chassis_vel_transform`（实机） | `cmd_vel_arbiter` | 实机自主源：`autonomy_raw -> fake_vel_transform -> /cmd_vel/autonomy_gimbal -> chassis_vel_transform` |
| `/cmd_vel/selected` | `cmd_vel_arbiter` | 串口桥（`cmd_vel_topic`）、MuJoCo `twist_to_motion_ctrl`、Gazebo/loopback 执行端 | 唯一最终 Twist；自动源需要新鲜执行租约 |
| `/serial/link_up` | `standard_robot_pp_ros2` | `cmd_vel_arbiter`、监控 | 50 ms 心跳；串口断开时为 `false` |
| `/serial/gimbal_state` | `standard_robot_pp_ros2` | ROS 2 视觉 `GimbalTopicBridge` | 11 元数组，四元数顺序 `wxyz`，角度单位 rad |
| `/serial/gimbal_cmd` | ROS 2 视觉 `GimbalTopicBridge` | `standard_robot_pp_ros2` | 7 元数组；桥接层校验长度、有限值和开火权限 |

`/planner/emergency_stop` 与 `/minco/reference_path` 是独立 DDS topic，不具备跨 topic
原子顺序。Goal Manager 的提交点会重新校验地图 snapshot/heartbeat，并在同一临界区内先发布
`emergency_stop=false`、再发布重定时 reference；MPC 建议拒绝急停前或无有效时间戳的旧轨迹。

`/cmd_vel/selected` 是仲裁后的导航速度边界。实机串口、MuJoCo bridge 与 Gazebo chassis adapter
仅订阅它；实机保留 `fake_vel_transform` 和 `chassis_vel_transform`，MuJoCo/Gazebo 建议避免订空的
`/cmd_vel/autonomy`。下位机负责 CAN、电机、轮速、电流、电压、温度、底盘反馈、
硬件 watchdog、制动和物理急停；这些信号由对应硬件接口处理，不由导航 action 裁决。
`standard_robot_pp_ros2` 的决策与自瞄内容保持不变，`serial/gimbal_joint_state` 仍提供云台 yaw、
速度变换和自瞄-导航协调所需输入。

## ⚙️ 统一配置

根仓的正式参数权威为：

```text
src/ats_sentry_bringup/params/node_params.yaml
```

它是实车 profile。正式 launch 将同一 `params_file` 传给定位、地图、规划、控制、串口与行为节点；
实机 launch 仅覆盖 `use_sim_time` 与资产/设备路径。

MuJoCo 在同一 `node_params.yaml` 之上叠加仿真 profile
`src/sim/ats_mujoco_sim/config/rmuc_2025_navigation.yaml`（后加载者覆盖），覆盖范围为
`ats_rog_map`、`ats_rog_map_adapter`、`minco_planner`、`ats_goal_manager` 与 `terrain_analysis_ext`；
仿真 launch 另外以内联参数指定 `cmd_vel_arbiter` 等节点的 topic。

当前 2027 车体足迹为 `0.571 x 0.581 m`；隧道模式只允许 `0.571 m` 面正对隧道，实车和仿真仍使用不同安全余量：

| 参数 | 实车 `node_params.yaml` | 仿真 `rmuc_2025_navigation.yaml` |
| :--- | :--- | :--- |
| `footprint_length/width` | `0.571 / 0.581` | `0.571 / 0.581` |
| `footprint_safety_margin` | `0.05` | `0.02` |
| `minco_planner.jps_safe_distance` | `0.54` | `0.44` |

仿真 profile 的放宽项和仿真专用开关不能直接迁入实车，每迁一项都要有仿真证据；已迁入实车的项见
`docs/调研/无先验地图钻洞高速导航优化调研报告.md` 5.1.6 F 与 5.1.7 E。ROGMap 的正式 profile 不接受
`map_config_file` 与显式 ROS 参数同时生效。`static_map_publisher.py` 建议保留 `/map` 的
frame、origin/yaw、resolution、占据语义和 transient-local QoS。

导航模块的细化配置、接口语义和 action 用法见
[`src/ats_sentry_nav/README.md`](src/ats_sentry_nav/README.md)。

## 常见问题

### CycloneDDS 报 `wlo1: does not match an available interface` 或 `rmw_create_node: failed to create domain`

`$CYCLONEDDS_URI`（通常为 `~/.ros/cyclonedds.xml`）绑定的网卡当前不在线，节点无法创建 DDS
domain。在该文件中把网卡标为非必需，并加回环兜底（CycloneDDS 0.10.5 支持）：

```xml
<Interfaces>
  <NetworkInterface name="wlo1" multicast="true" presence_required="false"/>
  <NetworkInterface name="lo" multicast="true"/>
</Interfaces>
```

多实例运行时选择空闲且互不相同的 `ROS_DOMAIN_ID`（本机使用 `0–232`，不要使用 `77`），
同一实例的 launch、RViz 与 action 客户端必须保持一致。

## 📂 目录结构

```text
ATS_2026_snetry_test/
├── src/
│   ├── ats_sentry_bringup/          # 实机总入口、总参数、地图、PCD、RViz（根仓）
│   ├── ats_sentry_nav/              # 独立导航 Git 仓库
│   ├── ats_sentry_behavior/         # 行为树与 ATS action client
│   ├── ats_robot_description/       # 机器人描述与模型资源
│   ├── standard_robot_pp_ros2/      # 下位机串口桥
│   ├── sp_vision25/                 # 自瞄视觉
│   ├── interfaces/                  # 底盘/业务 ROS 接口
│   ├── tools/                       # pcd2pgm、rosbag 录制、云台键盘遥控
│   ├── dependencies/                # 源码构建的第三方依赖
│   └── sim/
│       ├── ats_mujoco_sim/          # 独立 MuJoCo Git 仓库
│       ├── gazebo_simulator/        # Gazebo 场地与 ATS 导航入口
│       └── loopback_sim/            # 轻量仿真执行端
├── scripts/                          # 工程工具与分析脚本
└── docs/                             # 研发与工程文档
```

## 🙏 致谢与许可证

本工作区的自研链路建立在以下开源项目与社区的贡献之上：

| 技术/项目 | 在 ATS 中的用途 | 上游 |
| :--- | :--- | :--- |
| ROS 2 | 节点、Topic、Service、Action、TF 与 launch 工具链 | [ros2/ros2](https://github.com/ros2/ros2) |
| Point-LIO | LiDAR-Inertial 定位与配准点云基础 | [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) |
| ROG-Map | 概率占据、滑窗地图与 ESDF 基础 | [hku-mars/ROG-Map](https://github.com/hku-mars/ROG-Map) |
| GCOPTER / MINCO | 非均匀时间轨迹表示与优化基础 | [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) |
| small_gicp | 点云配准与重定位基础 | [koide3/small_gicp](https://github.com/koide3/small_gicp) |
| MuJoCo | 四舵轮刚体、接触与传感器物理仿真 | [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) |
| Livox ROS Driver 2 | Livox 设备接入与带点时间戳的消息 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |

### 🙏技术交流与开源贡献致谢

除上述直接使用的开源项目外，本项目的学习、设计和工程实践也受到了许多高校、战队和个人的帮助。在此特别致谢：

| 致谢对象 | 交流与贡献 | 相关链接 |
| :--- | :--- | :--- |
| 中国科学技术大学技术开源报告、张昊鹏、邵宜阳等同学 | 感谢技术开源报告，以及在导航、算法思路和工程问题交流中的帮助与贡献 | [张昊鹏 GitHub](https://github.com/ZhangHaopeng-Dino) |
| 武汉科技大学武晓健（hyheiyue） | 感谢在导航实践与开源交流中的帮助，以及 `rose_navigation` 项目的开源贡献 | [rose_navigation](https://github.com/hyheiyue/rose_navigation) |
| 哈尔滨工业大学（威海）HERO 战队 | 感谢在 RoboMaster 导航与工程实践方面的开源贡献 | [HERO_2026_Sentry_NAV](https://github.com/LiuJinbo1027/HERO_2026_Sentry_NAV) |
| 同济大学 SuperPower 战队 | 感谢在 RoboMaster 视觉方向的开源项目与技术贡献 | [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) |
| 昆明理工大学津桥学院王从亦同学 | 感谢在项目进入导航方向初期给予的指导、交流与大力帮助 | — |
| 深圳北理莫斯科大学陈力瀚、北极熊战队 | 特别感谢在进入 RoboMaster 视觉与导航领域初期给予的引导、经验分享和技术帮助 | [SMBU-PolarBear-Robotics-Team](https://github.com/SMBU-PolarBear-Robotics-Team) |

以上致谢名单不分先后。因篇幅所限，未能尽列所有曾经给予支持的院校、战队、个人与开源项目，在此谨表歉意，并致以同样诚挚的谢意。衷心感谢 RoboMaster 各高校在算法研究、工程实践和开源生态建设方面的贡献与无私帮助。最后，特别感谢所有 RoboMaster 参赛队伍，你们的实践与探索让我获得了更深层次的学习与成长。

工作区聚合了多个独立 Git 仓库和不同许可证的 ROS 包。每个包、模型和第三方资产的最终许可与
再分发要求，以其自身的 `LICENSE`、`NOTICE`、`package.xml` 和上游声明为准。
