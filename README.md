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
  -> /rc_esdf/planning_grid + RC-ESDF
  -> ATS NavigateToPose action / Goal Manager
  -> JPS -> MINCO S3 + independent yaw + footprint safety / repair
  -> committed reference -> omnidirectional SE2 MPC
  -> /cmd_vel_mpc -> velocity bridge -> /motion_control -> swerve chassis
```

`point_lio` 持续提供 `/localization` 与 `/registered_scan`；ROGMap 不是定位器。
`/rog_map/esdf` 是可视化点云，规划数值只通过
`/rog_map/get_ground_projection` 与 RC-ESDF 接口传递，不能从点云反解析距离场。

## 📑 目录

- [工作区与仓库边界](#工作区与仓库边界)
- [技术亮点](#技术亮点)
- [系统依赖](#系统依赖)
- [Quick Start](#quick-start)
- [实机部署](#实机部署)
- [MuJoCo 仿真与回归](#mujoco-仿真与回归)
- [关键接口与所有权](#关键接口与所有权)
- [统一配置](#统一配置)
- [验证状态与限制](#验证状态与限制)
- [目录结构](#目录结构)
- [致谢与许可证](#致谢与许可证)

## 工作区与仓库边界

本目录是 ROS 2 工作区根，也是三个独立 Git 仓库的编排与文档仓；提交、状态检查和推送必须
逐仓执行。

| 仓库 | 路径 | 主要职责 |
| :--- | :--- | :--- |
| 工作区根仓 | `.` | `docs/`、回归脚本、实机 bringup、总参数、地图与默认 RViz |
| 导航仓 | `src/ats_sentry_nav` | 定位接入、ROGMap、adapter、RC-ESDF、JPS/MINCO、action 与 MPC |
| MuJoCo 仓 | `src/sim/ats_mujoco_sim` | 四舵轮物理、传感器、场地、底盘 bridge 与仿真 launch |

其他 `src/` 包提供机器人描述、行为、接口或第三方依赖。`minco+mpc_reference/` 与
`参考/` 仅用于算法/许可证溯源；它们不是活动构建输入。所有 `colcon` 命令都必须保留
`--base-paths src`，避免同名参考包进入构建图。

## ✨ 技术亮点

### 概率地图、数值投影与 2.5D 语义

`ats_rog_map` 维护概率占据、膨胀占据和 3D ESDF，并通过数值 projection service 输出二维
occupancy、signed distance 和梯度。`ats_rog_map_adapter` 将 ROGMap、terrain/slope 与静态图
保守融合，是 `/rc_esdf/planning_grid` 的唯一发布者。地图外、unknown、障碍和不可通行地形
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

优先让 `rosdep` 从活动源码解析可安装依赖：

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
sudo rosdep init  # 仅首次使用 rosdep 时执行
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep init` 已完成时不要重复执行。Livox SDK、small_gicp、MuJoCo 与可选 LiDAR backend
可能需按各项目上游说明额外安装；不要把未验证的版本组合写成已部署基线。

## ⚡ Quick Start

### 构建整个活动工作区

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src --symlink-install --parallel-workers 1
source install/setup.bash
```

只需构建仿真及其上游依赖时：

```bash
MAKEFLAGS=-j1 colcon build --base-paths src --symlink-install \
  --packages-up-to ats_mujoco_sim --parallel-workers 1
source install/setup.bash
```

检查正式入口的参数，不会驱动车辆：

```bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py --show-args
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py --show-args
```

## 🚀 实机部署

### 部署前清单

在允许执行任何运动命令前，必须确认：

- `world` 对应的静态地图与 prior PCD 文件存在且坐标系匹配；
- LiDAR/IMU、底盘串口、云台连接、波特率与 `node_params.yaml` 一致；
- LiDAR 外参、`map -> odom`、`odom -> gimbal_yaw_odom` 与底盘 frame 已实测标定；
- 物理急停、独立安全员和受限低速区域可用；
- 没有其他节点发布竞争的关键 TF、`/cmd_vel_mpc` 或底盘最终输入。

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

这两层速度转换是既有云台/底盘 topic 与 frame 契约的一部分。固定雷达迁移只能通过 launch
参数关闭兼容层，并同时验证下游速度坐标系、TF 和 topic；不能仅关闭 transform 后继续假定
底盘接收到相同 frame 的命令。fake-yaw 关闭时仍须保留
`gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF，且不得增加重复的
`base_footprint -> base_link` 发布者。

仅调试导航 action、暂不启动行为树：

```bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py \
  world:=rmuc_2026 launch_behavior:=false use_rviz:=true
```

下列命令会驱动车辆，只能在完成上方安全清单后使用：

```bash
ros2 action send_goal --feedback \
  /ats_navigate_to_pose \
  ats_navigation_interfaces/action/NavigateToPose \
  "{goal_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, timeout: {sec: 60, nanosec: 0}}"
```

## 🧪 MuJoCo 仿真与回归

无界面闭环是自动化回归推荐入口：

```bash
cd /home/ats/ATS_2026_snetry_test
ROS_DOMAIN_ID=187 \
PLANNING_GRID_OWNER=rog_map \
P2_FAULT_CASE=none \
TEST_PROFILE=red_box \
GOAL_TIMEOUT=180 \
scripts/test_mujoco_minco_mpc_chain.sh
```

横移回归使用 `TEST_PROFILE=rectangle`；south/north 段应观察到非零 `linear.y`，用于防止
四舵轮控制链静默退化为差速运动。每个 P2/P3 故障场景都必须在新的 `ROS_DOMAIN_ID` 和新的
MuJoCo launch 中运行，不能在同一进程内串行注入后声称独立通过。

可选故障入口：

```text
P2_FAULT_CASE: adapter_lease | service_timeout | input_stale | unknown | unreachable
P3_FAULT_CASE: cancel | preempt | timeout | tf_failure
```

故障验收至少要观察：

```text
emergency_stop=true -> /cmd_vel_mpc=0 -> /motion_control=0
```

恢复时还要确认 generation 继续前进，且未提交新目标时旧 reference/执行授权不复活。详细的
MuJoCo 依赖、launch、资产和 telemetry 说明见
[`src/sim/ats_mujoco_sim/README.md`](src/sim/ats_mujoco_sim/README.md)。

## 📡 关键接口与所有权

| 接口 | 唯一权威 producer | 主要 consumer | 契约 |
| :--- | :--- | :--- | :--- |
| `/ats_navigate_to_pose` | `ats_goal_manager` action server | 行为树、测试客户端 | 目标生命周期与失败码 |
| `/rc_esdf/planning_grid` | `ats_rog_map_adapter` | MINCO、行为层、RViz | 规划地图唯一 owner |
| `/minco/reference_path` | Goal Manager 提交点 | MPC、RViz | 复核后统一重定时的 reference |
| `/planner/emergency_stop` | Goal Manager | MPC、底盘安全链 | heartbeat 急停状态 |
| `/cmd_vel_mpc` | `ats_swerve_mpc` | 唯一速度 bridge | 车体系 `[vx, vy, wz]` |
| `/motion_control` | `twist_to_motion_ctrl` | MuJoCo/底盘 | 唯一最终底盘输入 |

`/planner/emergency_stop` 与 `/minco/reference_path` 是独立 DDS topic，不具备跨 topic
原子顺序。Goal Manager 的提交点会重新校验地图 snapshot/heartbeat，并在同一临界区内先发布
`emergency_stop=false`、再发布重定时 reference；MPC 必须拒绝急停前或无有效时间戳的旧轨迹。

## ⚙️ 统一配置

根仓的正式参数权威为：

```text
src/ats_sentry_bringup/params/node_params.yaml
```

正式 launch 将同一 `params_file` 传给定位、地图、规划、控制、串口与行为节点；launch 仅覆盖
`use_sim_time`、资产/设备路径和明确的 HIL 开关。ROGMap 的正式 profile 不允许
`map_config_file` 与显式 ROS 参数同时生效。`static_map_publisher.py` 必须保留 `/map` 的
frame、origin/yaw、resolution、占据语义和 transient-local QoS。

导航模块的细化配置、接口语义和 action 用法见
[`src/ats_sentry_nav/README.md`](src/ats_sentry_nav/README.md)。

## ✅ 验证状态与限制

**已记录的运行证据（本次 README 更新未重新执行）：** 2026-08-02 的 S1 记录中，
`planning_grid_owner=rog_map` 的 rectangle 场景在 domain `184`（RViz）和 `186`（headless）
完成五段 action；最大终点误差分别为 `0.038681 m` 与 `0.041613 m`，generation 分别为
`313 -> 1328` 与 `309 -> 1264`。两例均记录了 south/north 非零 `vy`、唯一速度/底盘 owner、
MINCO 离散 footprint collision sample `0`、最终四轮 RPM 和两级命令为零；现有 MuJoCo
evaluator 的 `contact_violation_count=0`。

P2 的 adapter lease、projection timeout、Point-LIO input stale、unknown、unreachable，以及
P3 的 cancel、preempt、timeout、TF failure 已有独立故障运行记录；它们均记录到急停和两级
零速度。完整证据边界、P4 接口进度和实车门禁见
[`docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`](docs/nav2_to_3desdf_minco_mpc_optimization_direction.md)。

以下仍是**未完成或未验证**项：连续 swept footprint、`PlanningMapSnapshot`/
`PlannerCandidate` 原子契约的全链运行迁移、serial digest enforcement、HIL、实车动力学/制动
标定与受限低速实车。离散 footprint sample 或 MuJoCo contact 计数为零不能推导为物理零碰撞，
也不能替代实车性能数据。

## 📂 目录结构

```text
ATS_2026_snetry_test/
├── src/
│   ├── ats_sentry_bringup/          # 实机总入口、总参数、地图、PCD、RViz
│   ├── ats_sentry_nav/              # 独立导航 Git 仓库
│   ├── ats_sentry_behavior/         # 行为树与 ATS action client
│   ├── ats_robot_description/       # 机器人描述与模型资源
│   ├── sim/ats_mujoco_sim/          # 独立 MuJoCo Git 仓库
│   └── interfaces/                  # 底盘/业务 ROS 接口
├── scripts/                          # 构建、配置与 MuJoCo 回归脚本
├── docs/                             # 研发状态、验收边界与工程文档
└── minco+mpc_reference/              # 参考工程，不参与活动构建
```

## 🙏 致谢与许可证

本工作区的自研链路建立在以下开源项目与社区的贡献之上：

| 技术/项目 | 在 ATS 中的用途 | 上游 |
| :--- | :--- | :--- |
| ROS 2 | 节点、Topic、Service、Action、TF、launch 与测试工具链 | [ros2/ros2](https://github.com/ros2/ros2) |
| Point-LIO | LiDAR-Inertial 定位与配准点云基础 | [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) |
| ROG-Map | 概率占据、滑窗地图与 ESDF 基础 | [hku-mars/ROG-Map](https://github.com/hku-mars/ROG-Map) |
| GCOPTER / MINCO | 非均匀时间轨迹表示与优化基础 | [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) |
| small_gicp | 点云配准与重定位基础 | [koide3/small_gicp](https://github.com/koide3/small_gicp) |
| MuJoCo | 四舵轮刚体、接触与传感器物理仿真 | [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) |
| Livox ROS Driver 2 | Livox 设备接入与带点时间戳的消息 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |

工作区聚合了多个独立 Git 仓库和不同许可证的 ROS 包。每个包、模型和第三方资产的最终许可与
再分发要求，以其自身的 `LICENSE`、`NOTICE`、`package.xml` 和上游声明为准。
