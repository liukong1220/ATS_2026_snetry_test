# ATS MuJoCo 仿真接入说明

更新时间：2026-07-03

本文档记录从 `~/参考/src/swerve_drive` 和 `~/参考/src/MuJoCo-LiDAR`
迁移到当前仓库的 MuJoCo 仿真入口。

## 1. 当前定位

`ats_mujoco_sim` 是后续验证底盘动力学、传感器点云、ToF、控制器和
`SE2 MPC` 的仿真入口，不再依赖 Gazebo 作为主阻塞项。

当前包来源：

1. `~/参考/src/swerve_drive`
2. `~/参考/src/MuJoCo-LiDAR`
3. `~/参考/src/manda_can_control`
4. `~/参考/src/DDR-opt/utils/carstatemsgs`

## 2. 新增包

1. `src/ats_mujoco_sim`
   MuJoCo 仿真节点、地图生成、场景生成、模型和内嵌 `mujoco_lidar`。
2. `src/manda_can_control`
   仿真沿用的 WL100 / CAN 控制接口消息。
3. `src/carstatemsgs`
   仿真沿用的车辆状态消息。

## 3. 启动方式

先构建相关包。低性能机器建议单包顺序构建：

```bash
MAKEFLAGS=-j1 colcon build --packages-select carstatemsgs --parallel-workers 1
MAKEFLAGS=-j1 colcon build --packages-select manda_can_control --parallel-workers 1
MAKEFLAGS=-j1 colcon build --packages-select ats_mujoco_sim --parallel-workers 1
```

启动随机地图 + MuJoCo 仿真：

```bash
source install/setup.bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py use_viewer:=true
```

无界面轻量启动：

```bash
source install/setup.bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py \
  use_viewer:=false \
  show_viewer:=false \
  enable_lidar:=false \
  enable_tof:=false
```

默认随机地图输出到：

```text
/tmp/ats_mujoco_sim_maps
```

如需保留某个场景，可指定：

```bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py \
  output_root:=$PWD/mujoco_maps \
  map_name:=debug_case \
  seed:=1
```

## 4. 关键话题

1. `/localization`
   仿真里程计。
2. `/local_pointcloud`
   可选 lidar 点云，`enable_lidar:=true` 时发布。
3. `/perception/tof/points_merged`
   可选双侧 ToF 合并点云，`enable_tof:=true` 时发布。
4. `/simulation/PoseSub`
   位姿重置输入。

## 5. 当前验证结果

已完成：

1. `python3 -m py_compile` 检查 `ats_mujoco_sim` Python 文件。
2. 单包低并发构建：
   `carstatemsgs`、`manda_can_control`、`ats_mujoco_sim`。
3. `ros2 launch ats_mujoco_sim planner_mujoco.launch.py --show-args`。
4. 随机地图与 MuJoCo scene 生成。
5. 无 viewer、无 lidar/tof 的 8 秒短启动烟测，节点能加载 MuJoCo 模型。

运行环境已检查存在：

1. `mujoco 3.10.0`
2. `cv2 4.5.4`
3. `numpy 1.21.5`
