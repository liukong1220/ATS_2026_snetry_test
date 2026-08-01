# ATS 2026 Sentry Workspace

安徽信息工程学院 Artisans 战队 2026 四驱四转舵轮哨兵 ROS 2 工作区。

本仓库组织实机启动、行为决策、定位与地图、自研规划控制、MuJoCo 仿真、
loopback 快速回归和上下位机通信。当前同时保留 Nav2 对照链与专用
Nav2-free 实机入口；“全仓已移除 Nav2”尚不成立。

> 文档状态：2026-07-31，依据当前源码、launch、参数与接口静态核对。
> 本轮仅更新文档，未重跑 colcon、MuJoCo 或实车验收。

## 目录

- [项目简介](#项目简介)
- [当前状态](#当前状态)
- [核心模块](#核心模块)
- [依赖环境](#依赖环境)
- [Quick Start](#quick-start)
- [启动入口](#启动入口)
- [接口与所有权](#接口与所有权)
- [配置文件](#配置文件)
- [数据流](#数据流)
- [软件架构](#软件架构)
- [目录结构](#目录结构)
- [测试与验收](#测试与验收)
- [文档导航](#文档导航)
- [验证边界](#验证边界)
- [参考与致谢](#参考与致谢)

## 项目简介

V1 目标链为：

```text
传感器 + 独立状态估计
  -> ROGMap 概率占据 / 膨胀 / 3D ESDF
  -> 地面投影与 2.5D 可通行语义
  -> RC-ESDF 规划接口
  -> 自研目标管理
  -> JPS
  -> MINCO S3 + 独立 yaw
  -> footprint safety + Local Collision Repair
  -> 全向 SE2 MPC
  -> 四舵轮底盘
```

Point-LIO 继续拥有 `/localization` 与 `/registered_scan` 定位输出；ROGMap
只负责概率占据、3D ESDF 和地面投影，不替代定位器。底盘命令是车体系
`[vx, vy, wz]`，状态是世界系 `[x, y, yaw]`，不能引入差速底盘的
`vy=0` 约束。

## 当前状态

| 范围 | 当前事实 | 结论 |
| :--- | :--- | :--- |
| 普通实机总入口 | `bringup.launch.py` 默认 `launch_nav2:=true`、`planning_grid_owner:=rc_esdf` | Nav2 对照 profile 仍是默认值 |
| 专用实机入口 | `real_robot_nav2_free.launch.py` 固定 `launch_nav2:=false`、`launch_swerve_mpc:=true` | 自研 action/MINCO/MPC 入口已实现 |
| MuJoCo | `mujoco_navigation.launch.py` 默认 `launch_nav2:=true`，MPC 模式仍可依赖 `/plan` | 不能据此声明 P3 通过 |
| 规划地图 | `planning_grid_owner` 只允许 `rc_esdf|rog_map` | launch 负责抑制非 owner，禁止热切换 |
| 自研接口 | `/ats_navigate_to_pose`、`PlannerGoal`、`PlannerStatus`、`ExecutionCommand` 已定义 | schema 权威在 `.msg/.action/.srv` |
| 参数 | 实机感知/串口在总 YAML，自研三节点仍各有 `*_reality.yaml` | “单一总 YAML”仍是待实施目标 |
| ROGMap 可视化 | 支持占据、膨胀、unknown、ESDF 调试输出；P2 配置默认关闭 visualization | RViz 方案已规划，未在本轮运行确认 |

阶段边界：

- P2：ROGMap 地面适配、唯一地图 owner、单次 MINCO 不可变 snapshot 与失效安全。
- P3：自研 goal/action 生命周期和所有运行入口 Nav2-free。
- P4：连续 swept footprint、实车动力学约束与实车验证。

## 核心模块

| 模块 | 路径 | 主要职责 |
| :--- | :--- | :--- |
| 总启动 | `src/ats_sentry_bringup` | 实机编排、总参数、地图/PCD、RViz、rosbag |
| 行为决策 | `src/ats_sentry_behavior` | 行为树、巡逻/补给/视觉目标、自研与对照 action client |
| 导航定位 | `src/ats_sentry_nav` | Point-LIO、ROGMap、adapter、RC-ESDF、JPS/MINCO、Goal Manager、MPC、Nav2 对照资源 |
| MuJoCo | `src/sim/ats_mujoco_sim` | 四舵轮动力学、LiDAR/ToF、真值、速度桥和闭环回归 |
| loopback | `src/sim/loopback_sim` | 不含物理动力学的轻量 Nav2/行为回归 |
| 串口桥 | `src/standard_robot_pp_ros2` | `/cmd_vel` 到下位机、裁判系统、云台状态和 watchdog |
| 接口域 | `src/interfaces` 与 `ats_navigation_interfaces` | 业务消息、导航 action、地图/规划状态 schema |

## 依赖环境

- Ubuntu 22.04
- ROS 2 Humble
- GCC/G++ 11
- CMake 3.16+
- Python 3
- Eigen3、PCL、OpenCV、yaml-cpp
- MuJoCo Python 运行环境，仅仿真需要

常用基础依赖：

```bash
sudo apt update
sudo apt install -y \
  git git-lfs build-essential cmake pkg-config \
  python3-pip python3-vcstool python3-rosdep \
  libeigen3-dev libomp-dev
```

## Quick Start

### 拉取工作区

```bash
git clone --depth=1 -b develop \
  https://github.com/liukong1220/ATS_2026_snetry_test.git
cd ATS_2026_snetry_test
git lfs install
./import_workspace_repos.sh --shallow
```

`src/ats_sentry_bringup/pcd/*.pcd` 是本地实机资产，不随 Git 分发。缺少目标
场地 PCD 时不得把定位链降级为可用状态。

### 构建

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build \
  --base-paths src \
  --symlink-install \
  --parallel-workers 1
source install/setup.bash
```

活动源码与 `参考/` 可能存在同名包，因此所有 `colcon build/test` 必须显式
使用 `--base-paths src`。

## 启动入口

### 实机 Nav2-free 专用入口

```bash
ros2 launch ats_sentry_bringup real_robot_nav2_free.launch.py \
  world:=rmuc_2026 \
  planning_grid_owner:=rog_map \
  use_rviz:=false
```

该入口固定关闭 Nav2 和两级 Nav2 速度变换，MPC 直接向 `/cmd_vel` 输出
车体系速度。当前源码注释将它限制在不通电检查与抬轮 HIL；落地实车闭环
尚需按 P4 门禁执行。

### 普通实机总入口/对照 profile

```bash
ros2 launch ats_sentry_bringup bringup.launch.py \
  world:=rmuc_2026 \
  launch_nav2:=true
```

普通入口默认仍启动 Nav2。其 `launch_fake_vel_transform:=True` 和
`launch_chassis_vel_transform:=True` 是云台雷达兼容链默认值，不能仅因移除
Nav2 就静默删除；固定雷达迁移必须同时保持 topic、TF 和速度 frame 契约。

### MuJoCo 导航

```bash
ros2 launch ats_mujoco_sim mujoco_navigation.launch.py \
  use_viewer:=false \
  show_viewer:=false \
  use_rviz:=false
```

MuJoCo 当前默认是 Nav2 基线。P2 红框回归必须通过脚本显式选择 ROGMap owner：

```bash
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 \
  scripts/test_mujoco_minco_mpc_chain.sh
```

### loopback 快速回归

```bash
ros2 launch ats_sentry_bringup loopback_decision_sim.launch.py
```

loopback 用于低成本验证行为和 Nav2 话题，不提供真实接触、传感器噪声或舵轮
动力学证据。

## 接口与所有权

### 自研导航主接口

| 名称 | 类型 | Producer | Consumer/用途 |
| :--- | :--- | :--- | :--- |
| `/ats_navigate_to_pose` | `ats_navigation_interfaces/action/NavigateToPose` | `ats_goal_manager` server | 行为树/测试客户端 |
| `/ats_goal_manager/planner_goal` | `PlannerGoal` | Goal Manager | MINCO planner |
| `/minco/planning_status` | `PlannerStatus` | MINCO planner | Goal Manager |
| `/minco/reference_path_candidate` | `nav_msgs/Path` | MINCO planner | Goal Manager 复核 |
| `/planner/execution_command` | `ExecutionCommand` | Goal Manager 唯一 owner | MPC 唯一执行授权 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2-free 下 MPC 唯一 owner | 串口底盘 |

### 地图接口

| 名称 | 类型 | 约定 |
| :--- | :--- | :--- |
| `/rog_map/get_ground_projection` | `GetRogMapProjection` | 同一 response 原子携带 grid、signed distance、gradient 与 ROG generation |
| `/rc_esdf/planning_grid` | `nav_msgs/OccupancyGrid` | `rc_esdf_map` 或 adapter 二选一 owner |
| `/rog_map_adapter/status` | `PlanningMapStatus` | 携带 adapter publication sequence，不等于 MINCO snapshot generation |
| `/rog_map_adapter/ready` | `std_msgs/Bool` | 持续 heartbeat/lease，不是永久 ready |

ROS schema 只能由 `.msg/.srv/.action` 定义。总 YAML 未来只统一参数、topic、
frame、QoS、timeout 和 owner，不能复制消息字段成为第二份接口定义。

## 配置文件

### 当前有效配置

| 文件 | 当前所有权 |
| :--- | :--- |
| `src/ats_sentry_bringup/params/node_params.yaml` | 实机传感器、定位、串口、Nav2 对照和兼容速度链 |
| `src/ats_sentry_nav/minco_planner/config/minco_planner_reality.yaml` | 实机 MINCO/JPS/footprint 参数 |
| `src/ats_sentry_nav/ats_goal_manager/config/ats_goal_manager_reality.yaml` | 实机 action、lease、提交和终点判据 |
| `src/ats_sentry_nav/ats_swerve_mpc/config/ats_swerve_mpc_reality.yaml` | 实机 MPC 与舵轮约束 |
| `src/ats_sentry_nav/ats_rog_map/config/rog_map_ground_planning_mujoco.yaml` | P2 ROGMap 地图参数 |
| `src/ats_sentry_nav/ats_rog_map_adapter/config/rog_map_ground_planning.yaml` | P2 投影、融合、snapshot、heartbeat 参数 |

目标状态是将正式实机参数收敛到 `node_params.yaml` 一个总 YAML，并保持原参数
键可直接修改/移除。迁移必须先让 launch 真正加载新段，再删除分散配置；当前还未
完成，详见 [导航参数与接口统一配置方案](./docs/项目优化文档/nav2free/导航参数与接口统一配置方案.md)。

## 数据流

### Nav2-free 目标链

```text
/livox/lidar + /livox/imu
  -> Point-LIO
  -> /localization + /registered_scan
  -> ROGMap + terrain/static map
  -> ats_rog_map_adapter
  -> /rc_esdf/planning_grid + numeric signed distance
  -> ats_goal_manager -> PlannerGoal
  -> JPS -> MINCO S3 -> yaw -> footprint/repair
  -> candidate reference
  -> Goal Manager atomic ExecutionCommand
  -> ats_swerve_mpc
  -> /cmd_vel
  -> standard_robot_pp_ros2
  -> 四舵轮底盘
```

### generation 边界

```text
ROGMap source generation
  != adapter publication sequence
  != MINCO local immutable snapshot generation
```

当前 `OccupancyGrid` 不携带 ROG source generation。只能证明单次规划内部的
JPS、二维 RC-ESDF、MINCO clearance、footprint gate 和 repair 共用一份
MINCO snapshot；不能声称 generation 编号端到端一致。

## 软件架构

```text
任务/视觉/裁判系统
        |
        v
ats_sentry_behavior
        |
        v
ats_navigation_interfaces/action/NavigateToPose
        |
        v
ats_goal_manager <---- localization/map heartbeat
        |                              ^
        v                              |
minco_planner <---- ats_rog_map_adapter <---- ROGMap/terrain/static map
        |
        v
atomic ExecutionCommand
        |
        v
ats_swerve_mpc ----> /cmd_vel ----> standard_robot_pp_ros2
```

Nav2 对照资源在 P3 完成前保留，用于基线比较；它们不应再成为新自研接口的
依赖。公共算法解耦、构建依赖移除和旧资源归档必须分阶段完成。

## 目录结构

```text
.
├── README.md
├── AGENTS.md
├── dependencies.repos
├── docs/
│   └── 项目优化文档/
├── scripts/
├── src/
│   ├── ats_sentry_bringup/
│   ├── ats_sentry_behavior/
│   ├── ats_sentry_nav/
│   ├── interfaces/
│   ├── sim/
│   │   ├── ats_mujoco_sim/
│   │   └── loopback_sim/
│   └── standard_robot_pp_ros2/
└── 参考/                    # 许可证/算法溯源，不是活动构建输入
```

根仓、导航仓、行为仓、MuJoCo 仓、loopback 仓和串口仓都是独立 Git 仓库。
提交、状态检查和推送必须分别进行。

## 测试与验收

最窄静态与包级检查：

```bash
python3 -m py_compile <changed_launch_files>
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-select <targets> --parallel-workers 1
colcon test --base-paths src --packages-select <targets>
colcon test-result --test-result-base build/<package> --verbose
git diff --check
```

运行门禁不能由 topic 存在替代。P2/P3 报告至少应包含：

- 规划地图、`/cmd_vel_mpc`/`/cmd_vel`、`/motion_control` 的唯一 producer/consumer；
- stale、unknown、unreachable、projection timeout、heartbeat 中断的确定性零速度；
- 路径点数、reference 点数、离散 footprint 冲突采样数；
- 终点坐标与位置误差；
- MuJoCo 物理接触 evaluator 结果；没有 evaluator 时必须写“未验证”；
- 恢复后旧 reference、迟到 response 和旧 generation 不复活运动。

## 文档导航

- [工作区总览](./docs/总览.md)
- [代码范围与目录结构](./docs/代码范围与目录结构.md)
- [启动入口与运行链路](./docs/启动入口与运行链路.md)
- [导航定位与轨迹链路](./docs/导航定位与轨迹链路.md)
- [接口消息与话题约定](./docs/接口消息与话题约定.md)
- [行为树决策链路](./docs/行为树决策链路.md)
- [仿真域说明](./docs/仿真域说明.md)
- [视觉与串口桥说明](./docs/视觉与串口桥说明.md)
- [构建与维护说明](./docs/构建与维护说明.md)
- [Nav2-free 优化文档索引](./docs/项目优化文档/nav2free/README.md)
- [下一阶段实施提示词](./docs/项目优化文档/nav2free/下一阶段Nav2移除与统一配置实施提示词.md)

## 验证边界

- **已验证**：本 README 中的文件、launch 默认值、接口 schema 和参数文件归属已做静态交叉核对。
- **已实现未运行**：专用实机 Nav2-free launch、ATS action、Goal Manager、原子 `ExecutionCommand` 链。
- **未验证**：本轮没有执行 colcon、MuJoCo、红框、故障注入或实车测试。
- **未完成**：全仓 Nav2 构建依赖移除、单一总 YAML、ROGMap 新 RViz profile、P3 两场景闭环和 P4 实车验收。

[Confidence: High] 静态架构结论由 launch 与接口/参数源码交叉支持；运行性能和
安全闭环没有本轮证据，禁止引用参考项目的频率、耗时或内存数据作为 ATS 实测值。

## 参考与致谢

项目使用或参考 ROS 2、Nav2 baseline、Point-LIO、ROGMap、MINCO、MuJoCo、
BehaviorTree.CPP/BehaviorTree.ROS2 等开源项目。具体许可证、版权和修改说明以各包
源码与许可证文件为准；参考实现不自动构成本仓活动构建输入。
