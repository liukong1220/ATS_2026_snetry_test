# ATS 2026 Sentry Workspace

安徽信息工程学院Artisans战队哨兵机器人工作区，集成了视觉、串口、导航、行为树、机器人描述和若干第三方依赖。工作区基于 ROS 2 Humble，默认运行在 Ubuntu 22.04 系统上，可实现wsl仿真

## 1. 工程目录

工作区顶层目录:

```text
.
├── src/
│   ├── dependencies/             # 第三方依赖与本队公共库
│   ├── interfaces/               # 自定义 ROS 2 接口
│   ├── pb2025_robot_description/ # 机器人模型与资源
│   ├── pb2025_sentry_behavior/   # 哨兵行为树
│   ├── pb2025_sentry_bringup/    # 总启动入口
│   ├── pb2025_sentry_nav/        # 导航与定位相关功能包
│   ├── sp_vision25/              # 视觉算法工程，现已整理为工作区内 ament 包
│   ├── standard_robot_pp_ros2/   # 串口与裁判系统通信
│   └── tools/                    # 辅助工具包
├── build/                        # colcon 构建产物
├── install/                      # colcon 安装产物
├── log/                          # colcon 日志
├── NAV2.sh                       # 导航相关脚本
├── mapping.sh                    # 建图相关脚本
├── pp_ros2.sh                    # 串口相关脚本
├── rosbag.sh                     # rosbag 相关脚本
├── nav_README.md                 # 导航模块原始说明
└── ws_README.md                  # 工作区原始说明
```

`src/sp_vision25/` 关键子目录:

```text
src/sp_vision25/
├── calibration/      # 标定程序
├── configs/          # 视觉配置文件
├── docs/             # 视觉模块文档
├── io/               # 硬件抽象层，相机/串口/ROS2 接口
├── src/              # 主程序入口
├── tasks/            # 自瞄、打符、全向感知
├── tests/            # 各模块独立测试程序
└── tools/            # 工具函数与基础组件
```

## 2. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Fortress
- CMake >= 3.16
- GCC / G++ 11
- OpenVINO 2024.6 或与你模型兼容的版本
- Ceres Solver
- small_gicp
- HikRobot MVS SDK
- MindVision SDK（仅在你需要使用 MindVision 相机时）

建议的系统依赖:

```bash
sudo apt update
sudo apt install -y \
  git git-lfs curl wget python3-pip python3-vcstool python3-rosdep \
  build-essential cmake pkg-config \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev \
  libusb-1.0-0-dev nlohmann-json3-dev can-utils screen \
  libomp-dev
```

如果你还没初始化 `rosdep`:

```bash
sudo rosdep init
rosdep update
```

安装 `small_gicp`:

```bash
git clone https://github.com/koide3/small_gicp.git
cd small_gicp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
sudo cmake --install build
```

## 3. HikRobot SDK 说明

本工作区里有两个地方会用到海康 SDK:

- `src/dependencies/hik_camera_ros2_driver`
- `sp_vision25`

现在 `sp_vision25` 的构建逻辑已经改成如下优先级:

1. 优先读取系统安装的 HikRobot SDK
2. 其次读取环境变量 `HIKROBOT_SDK_ROOT`
3. 再回退到工作区中的 `src/dependencies/hik_camera_ros2_driver/hikSDK`
4. 最后回退到 `src/sp_vision25/io/hikrobot`

推荐显式设置:

```bash
export HIKROBOT_SDK_ROOT=/opt/MVS
```

如果你的 SDK 实际安装在别的位置，把上面的路径替换成真实安装目录。目录下应至少能找到:

- `include/MvCameraControl.h`
- `lib/64/libMvCameraControl.so` 或 `lib/amd64/libMvCameraControl.so`

如果你使用 WSL，请不要在 `~/.bashrc` 里长期 `source` 其它旧工作区的 `install/setup.bash`，否则很容易出现 overlay 污染。

## 3.1 MindVision SDK 说明

`sp_vision25` 现在会在找到 MindVision SDK 时自动启用 MindVision 相机支持；如果找不到，则只禁用 MindVision 相机，不影响 HikRobot 方案构建。

如需启用 MindVision，相同地可显式设置:

```bash
export MINDVISION_SDK_ROOT=/path/to/MindVisionSDK
```

目录下应至少包含:

- `include/CameraApi.h`
- `lib/amd64/libMVSDK.so` 或 `lib/arm64/libMVSDK.so`

## 3.2 Livox SDK2 说明

`src/pb2025_sentry_nav/livox_ros_driver2` 现在会按下面顺序查找 Livox SDK2:

1. 环境变量 `LIVOX_SDK2_ROOT`
2. 包内目录 `src/pb2025_sentry_nav/livox_ros_driver2/Livox-SDK2`
3. 系统目录 `/usr/local` 与 `/usr`

推荐在新电脑上先执行仓库内脚本，把 SDK 安装到包目录里，这样不依赖全局 `/usr/local`:

```bash
./src/pb2025_sentry_nav/livox_ros_driver2/scripts/setup_livox_sdk2.sh
```

如果你已经自行安装到别的路径，则显式导出:

```bash
export LIVOX_SDK2_ROOT=/path/to/Livox-SDK2
```

目录下至少应包含:

- `include/livox_lidar_api.h`
- `lib/liblivox_lidar_sdk_shared.so`

## 4. 获取代码

```bash
git clone https://github.com/liukong1220/ATS_2026_snetry_test.git
cd ATS_2026_snetry_test
```

如需拉取外部仓库依赖，可按你们项目的 `dependencies.repos` / 各子模块 README 补齐。

## 5. 构建方式

先只加载 ROS 官方环境:

```bash
source /opt/ros/humble/setup.bash
```

安装 ROS 依赖:

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

如果你需要编译 Livox 驱动，建议在 `colcon build` 前先准备 SDK:

```bash
./src/pb2025_sentry_nav/livox_ros_driver2/scripts/setup_livox_sdk2.sh
```

### 5.1 推荐的 WSL 构建方式

WSL 下不要使用过高并发，尤其这个工程同时包含 PCL、OpenVINO、导航和视觉模块，内存峰值较高。推荐:

```bash
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

如果机器内存充足，可尝试稍微提高并发:

```bash
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
```

### 5.2 单独编译 `sp_vision25`

如果只调视觉模块，推荐优先直接走工作区内的 `colcon` 构建：

```bash
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS
colcon build --packages-select sp_vision25 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

这样做的好处是：

1. `sp_vision25` 会和 `sp_msgs`、行为树、bringup 保持同一套依赖解析
2. 运行时资源统一走 `install/sp_vision25/share/sp_vision25`
3. 后续直接 `ros2 run sp_vision25 ...` 或 launch，不容易再出现“自己单独 build 能跑，colcon 版路径错乱”的问题

如果只是临时做纯视觉算法调试，也仍然保留进入目录单独用 CMake 的方式：

```bash
cd src/sp_vision25
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j1
```

## 6. 启动方式

构建完成后:

```bash
source install/setup.bash
```

### 6.1 总启动

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  use_rviz:=True
```

### 6.2 常用单模块启动

相机:

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py \
  params_file:=<ABSOLUTE_PARAMS_FILE>
```

串口:

```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
  use_rviz:=True \
  params_file:=<ABSOLUTE_PARAMS_FILE>
```

导航:

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=<YOUR_WORLD_NAME> \
  slam:=False
```

行为树:

```bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
  params_file:=<ABSOLUTE_PARAMS_FILE>
```

### 6.3 `sp_vision25` 常用程序

下面这些程序会在 `sp_vision25` 包构建后生成:

- `standard`
- `mt_standard`
- `sentry`
- `sentry_debug`
- `camera_test`
- `auto_aim_test`

如果你用 `colcon` 构建，执行文件会安装到 `install/sp_vision25/lib/sp_vision25/`，也可以直接用：

```bash
ros2 run sp_vision25 <executable_name>
```

例如：

```bash
ros2 run sp_vision25 sentry --help
ros2 run sp_vision25 publish_test
ros2 run sp_vision25 auto_aim_test assets/demo/demo
```

现在 `sp_vision25` 已经补了统一路径解析，因此：

1. 默认 `configs/*.yaml`
2. YAML 里的 `assets/*.xml / *.onnx`
3. 离线测试里常用的 demo 录像

都不再强依赖“当前目录必须正好在 `src/sp_vision25` 里”。

### 6.4 最新视觉跟随仿真调试

当前最新版 loopback 视觉调试已经完成第二轮瘦身：`vision_suggested_goal_index` 冗余链路已删除，路径规划直接使用 `sp_vision` 提供的 `target_position_map` 生成跟随点。

推荐入口：

- 详细调试手册见 [docs/视觉跟随仿真调试.md](./docs/视觉跟随仿真调试.md)
- 架构与接口说明见 [docs/融合.md](./docs/融合.md)
- 实机落地方案见 [docs/实机视觉跟随优化方案.md](./docs/实机视觉跟随优化方案.md)

最常用启动命令：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py \
  use_rviz:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_target_yaw:=0.30 \
  vision_target_pitch:=-0.06 \
  vision_target_position_map_x:=5.0 \
  vision_target_position_map_y:=2.0 \
  vision_target_position_map_z:=0.0
```

运行中动态切换视觉目标位置：

```bash
ros2 param set /fake_decision_sim_inputs vision_target_position_map_x 2.5
ros2 param set /fake_decision_sim_inputs vision_target_position_map_y 4.2
```

调跟随半径：

```bash
ros2 param set /pb2025_sentry_behavior_server decision.vision.attack_radius 2.5
```

关闭视觉接管，验证是否回退到普通导航：

```bash
ros2 param set /fake_decision_sim_inputs vision_tracking false
```

## 7. 局部控制器：MPPI 配置与调优

本项目已从 PID 纯追踪控制器迁移到 MPPI 模型预测路径积分控制器 (`nav2_mppi_controller::MPPIController`)。当前默认工作流仅保留 MPPI 这一条局部控制链路，用于后续统一迭代与调参。

### 7.1 MPPI 概述

```
MPPI 工作流程:
  1. 在当前速度周围采样 K 条控制序列（vx, vy, omega）
  2. 用运动模型将每条序列前向推演 T 步
  3. 对每条轨迹评估代价（路径偏离 + 障碍物 + 目标 + 平滑性）
  4. 通过 softmax 加权平均得到最优控制
  5. 将最优序列平移一步，作为下一周期的初始猜测（warm-start）
```

相比 PID 纯追踪的主要优势：

| 特性 | PID 纯追踪 | MPPI |
|------|-----------|------|
| 预测能力 | 仅看一个前瞻点 | 前向推演 T 步（1.5s） |
| 转弯处理 | 曲率限速（被动减速） | 采样优化（主动选择最优轨迹） |
| 全向运动 | 解耦控制 v/ω | 联合优化 vx, vy, ω |
| 超调 | 高速急转易超调 | 预测性减速，大幅减少超调 |
| 多目标优化 | 单一误差最小化 | 多代价函数加权（路径 + 障碍物 + 目标 + 平滑） |
| 调参 | PID 三参数 × 2 | 代价权重（直观可解释） |

### 7.2 在新电脑上部署

MPPI 控制器是 ROS 2 Humble 官方包，无需额外编译：

```bash
# 确认已安装
sudo apt install ros-humble-nav2-mppi-controller

# 验证插件可用
ls /opt/ros/humble/lib/libmppi_controller.so
ls /opt/ros/humble/lib/libmppi_critics.so
```

如果你的系统已经完整安装了 ROS 2 Humble 导航栈（`ros-humble-navigation2`），MPPI 包已经包含在内。构建工作区时不需要额外步骤——只有 YAML 配置文件被修改，会随 `colcon build --symlink-install` 自动生效。

如果在 `controller_server` 启动时遇到找不到 MPPI 插件的错误：

```bash
# 确认插件注册
cat /opt/ros/humble/share/nav2_mppi_controller/mppic.xml

# 如果缺失，手动安装
sudo apt install -y ros-humble-nav2-mppi-controller
```

### 7.3 配置文件位置

- **仿真环境**: `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml`
- **实车环境**: `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`
- **Loopback 仿真**: `src/loopback_sim/params/nav2_params.yaml`

关键配置段落（以实车为例）：

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"              # 全向底盘
  time_steps: 30                    # 前向推演步数 (1.5s @ 20Hz)
  batch_size: 750                   # 采样轨迹数（实车）
  temperature: 0.15                 # softmax 温度：越小越倾向于低成本轨迹
  vx_std: 0.5 / vy_std: 0.5 / wz_std: 0.8   # 采样噪声标准差
  critics: ["PathAlignCritic", ...] # 启用的代价函数列表
```

### 7.4 调参指南

#### 7.4.1 调参顺序

按以下顺序逐步调优，每步验证后再进入下一步：

1. **先跑通基本路径跟踪** — 只启用 `PathAlignCritic` + `GoalCritic` + `ObstaclesCritic`，确认机器人能跟踪路径并避开障碍物
2. **调路径对齐** — 增大 `PathAlignCritic.weight` 如果机器人偏离路径；减小如果路径跟踪过于僵硬
3. **调转弯平滑性** — 增大 `wz_std` 如果在转弯处不够灵活；减小如果转弯时抖动
4. **调全向行为** — 启用 `TwirlingCritic`（weight: 10-20）抑制不必要的原地旋转
5. **调终点收敛** — 调 `GoalCritic.weight` 和 `GoalAngleCritic.weight` 控制最终停靠精度
6. **调计算性能** — 如果 controller_server 掉频，减小 `batch_size` 或 `time_steps`

#### 7.4.2 核心参数说明

| 参数 | 效果 | 调大 | 调小 |
|------|------|------|------|
| `temperature` | Softmax 选择锐度 | 更激进，接近最优轨迹 | 更保守，融合更多采样 |
| `batch_size` | 采样数量 | 更平滑的控制 | 更快的计算 |
| `time_steps` | 前向视野 | 更早规划转弯 | 响应更快 |
| `vx_std / vy_std / wz_std` | 探索范围 | 更多样化的轨迹 | 更稳定的控制 |
| `gamma` | 远视折扣 | 更多关注远期目标 | 更多关注近期路径 |

#### 7.4.3 代价权重调优

权重越大，该代价项的优先级越高：

- `PathAlignCritic.weight: 15` — 如果机器人走偏，增大此值
- `GoalCritic.weight: 10` — 如果不及时停靠，增大此值
- `ObstaclesCritic.weight: 50` — 安全第一！不应大幅下调
- `TwirlingCritic.weight: 15` — 如果机器人原地打转，增大此值
- `ConstraintCritic.weight: 5` — 如果速度指令超限频繁，增大此值

#### 7.4.4 运行时动态调参

无需重启即可调整大部分参数：

```bash
# 调整采样数量
ros2 param set /controller_server FollowPath.batch_size 500

# 调整温度
ros2 param set /controller_server FollowPath.temperature 0.2

# 调整路径跟踪权重
ros2 param set /controller_server FollowPath.PathAlignCritic.weight 20.0

# 查看当前参数
ros2 param dump /controller_server | grep FollowPath
```

#### 7.4.5 实车部署建议

1. **首次部署降低速度上限**：将 `vx_max/vy_max` 临时设为 2.0，验证无误后再恢复到 4.5
2. **逐步增加 batch_size**：从 500 开始，确认 controller_server 能稳定 20Hz 后再提升到 750
3. **保持单轨配置**：不要在当前工作流里再混用 PID 旧参数，后续调参统一围绕 MPPI 展开

### 7.5 单轨维护建议

当前仓库的默认启动、默认依赖和默认构建流程都已收敛到 MPPI。后续维护建议保持：

- 仅维护 `nav2_mppi_controller::MPPIController` 的参数链路
- 仅观察 `transformed_global_plan` 与 `trajectories` 两类 MPPI 可视化输出
- 不再把 PID 配置片段作为运行时回退方案保留在主工作流中

### 7.6 Loopback 仿真中的 MPPI 对比

若要在 loopback 中对比 MPPI 和 DWB 控制器的表现：

```bash
# 默认 loopback 使用 MPPI
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py

# 观察 MPPI 输出的 cmd_vel
ros2 topic echo /cmd_vel

# 查看采样的轨迹可视化（如启用 visualize: true）
# 在 RViz 中添加 MarkerArray 话题
```

## 8. 常见问题

### 7.1 `cannot find -lMvCameraControl`

说明 `sp_vision25` 或 `hik_camera_ros2_driver` 找不到 HikRobot SDK 动态库。排查顺序:

1. 确认 SDK 已安装
2. 确认 `HIKROBOT_SDK_ROOT` 指向 SDK 根目录
3. 确认目录下存在 `MvCameraControl.h` 和 `libMvCameraControl.so`
4. 确认不要混入旧工作区环境

快速检查:

```bash
echo "$HIKROBOT_SDK_ROOT"
find "$HIKROBOT_SDK_ROOT" -name 'MvCameraControl.h' -o -name 'libMvCameraControl.so'
```

### 7.1.2 `liblivox_lidar_sdk_shared.so` 缺失或 `livox_ros_driver2` 链接失败

这说明 `livox_ros_driver2` 没找到 Livox SDK2 动态库。排查顺序:

1. 确认是否已执行 `./src/pb2025_sentry_nav/livox_ros_driver2/scripts/setup_livox_sdk2.sh`
2. 确认 `LIVOX_SDK2_ROOT` 是否指向正确 SDK 根目录
3. 确认目录下存在 `include/livox_lidar_api.h` 和 `lib/liblivox_lidar_sdk_shared.so`
4. 重新执行 `source /opt/ros/humble/setup.bash`，避免旧 overlay 污染

### 7.2 为什么现在更建议把 `sp_vision25` 放进 `src/`

结论是：**是的，更好。**

原因主要有 4 个：

1. 视觉、自定义消息、行为树、bringup 都在同一个 ROS 2 工作区里，依赖关系最清晰
2. `ros2 run`、launch、参数文件、接口包引用方式能统一，不需要维护两套启动习惯
3. `colcon install` 后资源会进入 `share/sp_vision25`，比“手动切目录跑 build 下二进制”更稳定
4. 后面你继续做导航融合、loopback 联调、真机替换 fake vision 时，不需要再额外写一层外部包装

保留 `src/sp_vision25/build` 的独立 CMake 方式仍然有价值，但它更适合：

1. 临时算法调试
2. 不依赖 ROS 接口的纯视觉单体验证

如果你现在目标是“完整哨兵融合系统”，那就应该以工作区内的 `src/sp_vision25` 包版本为主。

### 7.1.1 `cannot find -lMVSDK`

这说明构建时启用了 MindVision 相机支持，但没有找到 MindVision SDK。你有两种选择:

1. 安装并设置 `MINDVISION_SDK_ROOT`
2. 如果只使用 HikRobot，相比安装无关 SDK，更推荐保持当前代码逻辑，让 MindVision 支持自动禁用

### 7.2 WSL 下 `cc1plus` 被 Killed

这是典型的内存不足。把下面两个参数一起降下来:

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### 7.3 环境污染

不要在当前工作区构建前自动 `source` 其它工作区，例如:

```bash
source ~/old_ws/install/setup.bash
```

建议只保留:

```bash
source /opt/ros/humble/setup.bash
```

## 9. 参考文档

- [docs/视觉跟随仿真调试.md](./docs/视觉跟随仿真调试.md)
- [docs/融合.md](./docs/融合.md)
- [ws_README.md](./ws_README.md)
- [nav_README.md](./nav_README.md)
- [sp_vision25/readme.md](./src/sp_vision25/readme.md)
