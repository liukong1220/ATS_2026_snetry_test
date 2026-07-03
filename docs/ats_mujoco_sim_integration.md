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

启动 MuJoCo 仿真并同时打开 RViz2：

```bash
source install/setup.bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py \
  use_viewer:=false \
  show_viewer:=false \
  use_rviz:=true \
  rviz_delay_sec:=4.0 \
  enable_lidar:=true \
  enable_tof:=true \
  lidar_backend:=cpu
```

默认 RViz2 配置为：

```text
src/ats_mujoco_sim/rviz/mujoco_sim_observe.rviz
```

该视图默认显示：

1. `TF`
2. `/localization`
3. `/local_pointcloud`
4. `/perception/tof/points_merged`

默认启动顺序是：

1. 生成随机地图和 MuJoCo scene。
2. 启动 `ats_mujoco_sim` 控制器、里程计、反馈和传感器进程。
3. 延迟 `rviz_delay_sec` 秒后启动 RViz2。

这样可以避免 RViz2 比 MuJoCo 控制器更早启动时，看不到 TF、里程计或点云而误判为仿真失败。

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

## 4. 与实车链路的连接方式

`ats_mujoco_sim` 的目标不是做一套脱离实车的独立玩具仿真，而是让仿真端尽量复用实车控制和感知接口。
这样后续调 `MPPI / SE2 MPC / 底盘限速 / 高带宽控制器` 时，可以在仿真和实车之间少改 launch 与参数。

当前对齐关系如下：

| 方向 | MuJoCo 仿真话题 / 服务 | 类型 | 对齐目标 |
| --- | --- | --- | --- |
| 控制输入 | `/motion_control` | `manda_can_control/msg/MotionCtrl` | 实车底盘速度控制入口 |
| 控制输入 | `/speed_ctrl` | `manda_can_control/msg/SpeedCtrl` | 单轮速度控制调试入口 |
| 控制输入 | `/steer_ctrl` | `manda_can_control/msg/SteerCtrl` | 单轮转向控制调试入口 |
| 模式切换 | `/motion_mode` | `manda_can_control/srv/MotionMode` | 实车运动模式切换 |
| 模式切换 | `/control_mode` | `manda_can_control/srv/ControlMode` | 实车控制模式切换 |
| 反馈输出 | `/motion_fb` | `manda_can_control/msg/MotionFb` | 底盘运动反馈 |
| 反馈输出 | `/speed_fb` | `manda_can_control/msg/SpeedFb` | 单轮速度反馈 |
| 反馈输出 | `/steer_fb` | `manda_can_control/msg/SteerFb` | 单轮转向反馈 |
| 反馈输出 | `/system_state_fb` | `manda_can_control/msg/SystemstateFb` | 系统状态反馈 |
| 反馈输出 | `/battery_fb` | `manda_can_control/msg/BatteryFb` | 电池状态反馈 |
| 定位输出 | `/localization` | `nav_msgs/msg/Odometry` | 导航链定位输入 |
| 点云输出 | `/local_pointcloud` | `sensor_msgs/msg/PointCloud2` | Mid360 / lidar 等价观察入口 |
| ToF 输出 | `/perception/tof/points_merged` | `sensor_msgs/msg/PointCloud2` | 近距离侧向避障观察入口 |
| 位姿重置 | `/simulation/PoseSub` | `carstatemsgs/msg/CarState` | 仿真调试复位入口 |

实车连接建议分两层推进：

1. `接口同名层`
   保持仿真话题、服务和消息类型尽量与实车一致。
   上层规划控制节点只依赖 `/motion_control`、`/localization`、点云和反馈话题，
   不直接关心当前后端是 MuJoCo 还是实车 CAN。
2. `桥接适配层`
   如果实车底盘实际入口不是 `manda_can_control`，则单独写桥接节点，
   只在桥接层做消息转换。
   不要让 `minco_planner`、`trajectory_optimizer` 或未来 `SE2 MPC`
   直接依赖某个硬件驱动的私有字段。

推荐实车 / 仿真切换方式：

1. 仿真：
   启动 `ats_mujoco_sim`，由 MuJoCo 发布 `/localization`、点云和底盘反馈。
2. 实车：
   不启动 `ats_mujoco_sim`，由真实定位、雷达、ToF、CAN 驱动发布同名或经 remap 后同名的话题。
3. 上层：
   `Nav2 / trajectory_optimizer / minco_planner / MPC` 使用同一套输入输出话题。

## 5. RViz2 观察

当前已经提供 MuJoCo 专用 RViz2 配置：

```text
src/ats_mujoco_sim/rviz/mujoco_sim_observe.rviz
```

推荐启动：

```bash
source install/setup.bash
ros2 launch ats_mujoco_sim planner_mujoco.launch.py \
  use_rviz:=true \
  use_viewer:=false \
  show_viewer:=false \
  enable_lidar:=true \
  enable_tof:=true \
  lidar_backend:=cpu
```

如果只想单独打开 RViz2：

```bash
source install/setup.bash
rviz2 -d install/ats_mujoco_sim/share/ats_mujoco_sim/rviz/mujoco_sim_observe.rviz
```

RViz2 中优先确认：

1. Fixed Frame 为 `map`。
2. `map -> odom -> base_link` TF 是否连续。
3. `/localization` 的机器人位姿是否跟 MuJoCo 中运动一致。
4. `/local_pointcloud` 是否跟随 lidar frame。
5. `/perception/tof/points_merged` 是否贴近车体两侧并能反映近距离障碍。

LiDAR 后端说明：

1. 默认 `lidar_backend:=cpu`，低性能电脑优先使用这个配置。
2. `lidar_backend:=gpu` 当前会尝试 Taichi 后端，需要本机安装 `taichi`。
3. 如果用户手动传入 `gpu` / `taichi` 但环境没有 Taichi，节点会自动回退到 CPU 后端并输出 warning。
4. 后续做高频点云或大规模场景时，再单独评估 Taichi / GPU 后端，不要让 GPU 依赖阻塞基础仿真观察。

## 6. 关键话题

1. `/localization`
   仿真里程计。
2. `/local_pointcloud`
   可选 lidar 点云，`enable_lidar:=true` 时发布。
3. `/perception/tof/points_merged`
   可选双侧 ToF 合并点云，`enable_tof:=true` 时发布。
4. `/simulation/PoseSub`
   位姿重置输入。

## 7. 当前验证结果

已完成：

1. `python3 -m py_compile` 检查 `ats_mujoco_sim` Python 文件。
2. 单包低并发构建：
   `carstatemsgs`、`manda_can_control`、`ats_mujoco_sim`。
3. `ros2 launch ats_mujoco_sim planner_mujoco.launch.py --show-args`。
4. 随机地图与 MuJoCo scene 生成。
5. 无 viewer、无 lidar/tof 的 8 秒短启动烟测，节点能加载 MuJoCo 模型。
6. 新增 `use_rviz` / `rviz_config_file` launch 参数和 MuJoCo 专用 RViz2 观察配置。
7. `lidar_backend` 默认改为 `cpu`，并增加非 CPU 后端不可用时的自动 CPU 降级。
8. RViz2 增加 `rviz_delay_sec` 延迟启动，默认先启动 MuJoCo 控制器和传感器，再打开观察界面。

运行环境已检查存在：

1. `mujoco 3.10.0`
2. `cv2 4.5.4`
3. `numpy 1.21.5`
