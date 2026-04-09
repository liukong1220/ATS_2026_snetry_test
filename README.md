# ATS 2026 Sentry Workspace

安徽信息工程学院Artisans战队哨兵机器人工作区，集成了视觉、串口、导航、行为树、机器人描述和若干第三方依赖。工作区基于 ROS 2 Humble，默认运行在 Ubuntu 22.04 系统上，可实现wsl仿真

## 1. 工程目录

工作区顶层目录:

```text
.
├── sp_vision25/                  # 视觉算法工程，普通 CMake 包，会被 colcon 当作 cmake package 构建
├── src/
│   ├── dependencies/             # 第三方依赖与本队公共库
│   ├── interfaces/               # 自定义 ROS 2 接口
│   ├── pb2025_robot_description/ # 机器人模型与资源
│   ├── pb2025_sentry_behavior/   # 哨兵行为树
│   ├── pb2025_sentry_bringup/    # 总启动入口
│   ├── pb2025_sentry_nav/        # 导航与定位相关功能包
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

`sp_vision25/` 关键子目录:

```text
sp_vision25/
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
4. 最后回退到 `sp_vision25/io/hikrobot`

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

如果只调视觉模块:

```bash
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS
colcon build --packages-select sp_vision --cmake-args -DCMAKE_BUILD_TYPE=Release
```

或者进入目录单独用 CMake:

```bash
cd sp_vision25
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

下面这些程序会在 `sp_vision` 包构建后生成:

- `standard`
- `mt_standard`
- `sentry`
- `sentry_debug`
- `camera_test`
- `auto_aim_test`

如果你用 `colcon` 构建，执行文件位于 `build/sp_vision25/` 对应构建目录中，或按你们现有脚本启动。

## 7. 常见问题

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

## 8. 参考文档

- [ws_README.md](./ws_README.md)
- [nav_README.md](./nav_README.md)
- [sp_vision25/readme.md](./sp_vision25/readme.md)
