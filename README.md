# ATS 2026 Sentry Workspace

安徽信息工程学院 Artisans 战队哨兵机器人工作区。当前仓库整合了：

1. 实机总启动链路
2. 导航与定位模块
3. 行为树决策模块
4. 视觉接入与视觉跟随调试链路
5. loopback 轻量仿真链路

---

## 1. 环境要求

推荐基础环境：

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.16+
- GCC / G++ 11

常用系统依赖建议至少包含：

```bash
sudo apt update
sudo apt install -y \
  git git-lfs curl wget python3-pip python3-vcstool python3-rosdep \
  build-essential cmake pkg-config libeigen3-dev libomp-dev
```

如果还没有初始化 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

### 1.1 可选外部依赖

以下依赖不是每个人一上来都必须装，但在实机或视觉模块调试时经常会用到：

- HikRobot MVS SDK
- MindVision SDK
- Livox SDK2
- OpenVINO
- `small_gicp`

建议做法：

1. 先完成 ROS 2 和工作区基础构建
2. 再按自己当前任务补对应 SDK
3. 不要一开始把所有外部依赖都堆上去，排错会更困难

### 1.2 WSL 使用建议

如果你在 WSL 下工作，建议：

1. 不要在 `~/.bashrc` 长期 source 其它旧工作区
2. 初次构建时使用低并发
3. 优先先跑 loopback，再去接实机链路

---

## 2. 获取代码

```bash
git clone -b develop https://github.com/liukong1220/ATS_2026_snetry_test.git
cd ATS_2026_snetry_test
```

如果你们的依赖是通过 `vcs` 或其它内部方式管理，请按团队当前规则补齐依赖源；本 README 不重复维护所有外部仓库地址。

---

## 3. 构建流程

### 3.1 基础构建

先只加载 ROS 官方环境：

```bash
source /opt/ros/humble/setup.bash
```

安装 ROS 依赖：

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

推荐构建：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

构建完成后：

```bash
source install/setup.bash
```

### 3.2 WSL 推荐构建

如果你在 WSL 下，推荐先用保守配置：

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### 3.3 视觉 / 传感器相关可选环境变量

如果你当前需要接真实视觉或雷达，再补这些环境变量：

```bash
export HIKROBOT_SDK_ROOT=<YOUR_HIK_SDK_PATH>
export MINDVISION_SDK_ROOT=<YOUR_MINDVISION_SDK_PATH>
export LIVOX_SDK2_ROOT=<YOUR_LIVOX_SDK2_PATH>
```

只有在你实际使用对应设备时，这些变量才有必要配置。

---

## 4. 配置流程

建议按下面顺序配置，而不是一上来同时改很多文件。

### 4.1 先确定你当前要跑哪条链路

常见有三种：

1. loopback 通用仿真
2. loopback 视觉跟随测试
3. 实机总启动

### 4.2 再准备地图与点云资源

当前默认资源入口在：

- [`src/pb2025_sentry_bringup/map`](./src/pb2025_sentry_bringup/map)
- [`src/pb2025_sentry_bringup/pcd`](./src/pb2025_sentry_bringup/pcd)

如果你新增地图，通常需要保证：

1. 栅格地图在 `map/`
2. 先验点云在 `pcd/`
3. `world` 参数与文件名一致

### 4.3 再确认行为树参数文件

当前最常用的是这三份：

1. loopback 通用参数  
   [`src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml`](./src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)
2. 实机主线参数  
   [`src/pb2025_sentry_behavior/params/sentry_behavior.yaml`](./src/pb2025_sentry_behavior/params/sentry_behavior.yaml)
3. 视觉测试参数  
   [`src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml`](./src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml)

当前哨兵姿态与受击自旋逻辑也已经接入这三份参数文件，推荐同步阅读：

- [`docs/sentry_posture_switch_logic.md`](./docs/sentry_posture_switch_logic.md)

最常需要调的相关参数有：

- `decision.mode_thresholds.defend_hp`
- `decision.mode_limits.switch_cooldown_s`
- `decision.mode_limits.max_cumulative_s`
- `decision.motion.hit_spin_speed`
- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

### 4.4 再确认导航参数文件

这里最容易改错，建议直接记住：

1. loopback Nav2 参数  
   [`src/loopback_sim/params/nav2_params.yaml`](./src/loopback_sim/params/nav2_params.yaml)
2. 实机总入口默认参数  
   [`src/pb2025_sentry_bringup/params/node_params.yaml`](./src/pb2025_sentry_bringup/params/node_params.yaml)
3. 独立导航包默认参数  
   [`src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`](./src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)

### 4.5 推荐的实际配置顺序

建议新同学或新机器按这个顺序推进：

1. 先编过工作区
2. 先跑 loopback 通用仿真
3. 再跑 loopback 视觉测试
4. 再调整行为树与 MPPI 参数
5. 最后接实机总启动

---

## 5. 常用启动入口

### 5.1 loopback 通用仿真

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

入口文件：

- [`src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py`](./src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)

### 5.2 loopback 视觉跟随测试

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py use_rviz:=True
```

入口文件：

- [`src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py`](./src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)

### 5.3 实机总启动

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup bringup.launch.py world:=<YOUR_WORLD_NAME> use_rviz:=True
```

入口文件：

- [`src/pb2025_sentry_bringup/launch/bringup.launch.py`](./src/pb2025_sentry_bringup/launch/bringup.launch.py)

---

## 6. 工程目录说明

### 6.1 工作区顶层

```text
.
├── docs/                       # 面向维护者的说明文档
├── src/                        # 工作区源码
├── build/ install/ log/        # colcon 产物
├── README.md                   # 当前总说明
├── nav_README.md               # 历史导航说明
└── ws_README.md                # 历史工作区说明
```

### 6.2 `src/` 主目录

```text
src/
├── dependencies/              # 第三方依赖与公共库
├── interfaces/                # 自定义 ROS 2 接口
├── loopback_sim/              # 轻量 loopback 仿真与其 Nav2 参数
├── pb2025_robot_description/  # 机器人模型与描述
├── pb2025_sentry_behavior/    # 行为树、决策节点、BT 插件
├── pb2025_sentry_bringup/     # 全系统启动入口、地图、pcd、脚本
├── pb2025_sentry_nav/         # 导航、定位、传感器相关包
├── sp_vision25/               # 视觉算法工程
├── standard_robot_pp_ros2/    # 串口与机器人本体接口
└── tools/                     # 辅助工具
```

### 6.3 你最常会改到的地方

| 需求 | 优先看哪里 |
| --- | --- |
| 改启动链路 | `src/pb2025_sentry_bringup/launch/` |
| 改行为树策略 | `src/pb2025_sentry_behavior/` |
| 改 loopback 仿真参数 | `src/loopback_sim/params/nav2_params.yaml` |
| 改实机导航参数 | `src/pb2025_sentry_bringup/params/node_params.yaml` |
| 改地图 / pcd | `src/pb2025_sentry_bringup/map/`、`pcd/` |
| 改视觉消息或视觉接管 | `src/pb2025_sentry_nav/sp_msgs/`、`src/pb2025_sentry_behavior/` |

---

## 7. 推荐的文档阅读顺序

`docs/` 里现在已经按“现状优先”整理过。建议不要随便跳着读，先按下面顺序。

### 7.1 第一次接手项目

1. [`docs/移植.md`](./docs/移植.md)  
   先建立“哪些包负责什么、哪些参数文件会生效”的整体概念
2. [`docs/sentry_bt_decision_checklist.md`](./docs/sentry_bt_decision_checklist.md)  
   再理解当前行为树主线、黑板、分支优先级
3. [`docs/sentry_posture_switch_logic.md`](./docs/sentry_posture_switch_logic.md)  
   再确认姿态切换、血量阈值、受击自旋和下位机模式发送规则

### 7.2 只想快速跑通仿真

1. [`docs/slim_loopback_refactor.md`](./docs/slim_loopback_refactor.md)  
   先知道 loopback 是什么、从哪里启动
2. [`docs/mppi_parameter_tuning_guide.md`](./docs/mppi_parameter_tuning_guide.md)  
   再看 MPPI 参数和观测指标

### 7.3 要调视觉接管

1. [`docs/融合.md`](./docs/融合.md)
2. [`docs/视觉跟随仿真调试.md`](./docs/视觉跟随仿真调试.md)
3. [`docs/实机视觉跟随优化方案.md`](./docs/实机视觉跟随优化方案.md)

### 7.4 要调 MPPI 局部控制

1. [`docs/mppi_parameter_tuning_guide.md`](./docs/mppi_parameter_tuning_guide.md)
2. [`docs/mppi_local_plan_fix.md`](./docs/mppi_local_plan_fix.md)

### 7.5 哪些文档是“现状主文档”

优先级最高的是：

1. [`docs/移植.md`](./docs/移植.md)
2. [`docs/sentry_bt_decision_checklist.md`](./docs/sentry_bt_decision_checklist.md)
3. [`docs/sentry_posture_switch_logic.md`](./docs/sentry_posture_switch_logic.md)
4. [`docs/融合.md`](./docs/融合.md)
5. [`docs/视觉跟随仿真调试.md`](./docs/视觉跟随仿真调试.md)

历史长文和阶段性记录仍然保留，但应作为补充材料看，不要替代上面几份现状文档。

---

## 8. 推荐的上手路径

如果你是新加入项目，推荐按这个流程上手：

1. 先读本 README
2. 再读 `docs/移植.md`
3. 跑一次 `loopback_decision_sim.launch.py`
4. 再读 `docs/sentry_bt_decision_checklist.md`
5. 如需视觉，再跑 `loopback_vision_test.launch.py`
6. 最后再接实机 `bringup.launch.py`

这样能把问题拆开：

1. 先确认工作区能构建
2. 再确认导航与行为树链路能闭环
3. 再确认视觉接管能闭环
4. 最后再处理真实硬件问题

---

## 9. 额外说明

1. 根目录的 [`nav_README.md`](./nav_README.md) 和 [`ws_README.md`](./ws_README.md) 仍保留历史信息，但不再作为当前主索引。
2. 各子包自己的 README 仍然有价值，尤其是：
   - [`src/pb2025_sentry_behavior/README.md`](./src/pb2025_sentry_behavior/README.md)
   - [`src/pb2025_sentry_nav/README.md`](./src/pb2025_sentry_nav/README.md)
   - [`src/loopback_sim/README.md`](./src/loopback_sim/README.md)
   - [`src/standard_robot_pp_ros2/README.md`](./src/standard_robot_pp_ros2/README.md)
3. 如果你发现 README 与 `docs/` 有冲突，请以 `docs/` 中已经明确标注为“现状”的文档为准。

---

## 10. 一句话总结

这个仓库当前最推荐的理解方式是：

> 根 README 负责告诉你“怎么配、去哪改、先读什么”，`docs/` 负责告诉你“当前系统怎么组织、怎么调试”，子包 README 再负责各模块自己的细节。
