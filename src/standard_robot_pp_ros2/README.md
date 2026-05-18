# standard_robot_pp_ros2

当前工作区中的上下位机串口桥接与裁判系统接口层。

本包当前职责：

1. 把下位机串口数据解析成 ROS 话题
2. 接收导航与行为树输出并写回串口发送结构
3. 作为 `/cmd_vel`、姿态模式、裁判系统和云台关节状态的实机桥梁

## 当前入口

启动文件：

- [launch/standard_robot_pp_ros2.launch.py](./launch/standard_robot_pp_ros2.launch.py)

默认参数：

- [config/standard_robot_pp_ros2.yaml](./config/standard_robot_pp_ros2.yaml)

当前通常由整车总入口拉起：

- [../pb2025_sentry_bringup/launch/bringup.launch.py](../pb2025_sentry_bringup/launch/bringup.launch.py)

## 当前与上层系统的对接

### 1. 底盘速度

当前订阅：

- `/cmd_vel`

并写入串口发送结构：

- `SendRobotCmdData.data.speed_vector.vx`
- `SendRobotCmdData.data.speed_vector.vy`
- `SendRobotCmdData.data.speed_vector.wz`

当前还额外做了两层保护：

1. 瞬时零速短时保持
2. `cmd_vel` 断流 watchdog
3. 串口发送前的速度倍率适配

对应代码：

- [src/standard_robot_pp_ros2.cpp](./src/standard_robot_pp_ros2.cpp)

### 2. 姿态模式

当前行为树通过：

- `decision/robot_mode`

发布姿态模式，本包订阅后写入：

- `SendRobotCmdData.data.speed_vector.mode`

当前固定约定：

- `move = 3`
- `attack = 1`
- `defend = 2`

协议定义：

- [include/standard_robot_pp_ros2/packet_typedef.hpp](./include/standard_robot_pp_ros2/packet_typedef.hpp)

### 3. 裁判系统数据

当前会把下位机上传的裁判系统字段发布为：

- `referee/game_status`
- `referee/robot_status`
- `referee/rfid_status`
- 以及其他裁判相关话题

这些话题是行为树资源门控、受击检测和比赛状态判断的直接数据源。

### 4. 云台关节与 IMU

当前会发布：

- `serial/gimbal_joint_state`
- `serial/imu`

其中 `gimbal_joint_state` 供 `joint_state_publisher` / `robot_state_publisher` 维护整车 TF 链。

## 当前关键参数

最常动的参数在：

- [config/standard_robot_pp_ros2.yaml](./config/standard_robot_pp_ros2.yaml)

尤其是：

- `device_name`
- `baud_rate`
- `robot_mode_topic`
- `enable_transient_zero_cmd_hold`
- `transient_zero_cmd_hold_timeout_ms`
- `cmd_vel_watchdog_timeout_ms`
- `publish_imu_as_gimbal_joint_state`
- `accept_legacy_two_axis_joint_state`
- `small_yaw_is_relative`

## 当前常见维护问题

### 1. `/cmd_vel` 有值但底盘卡顿

先看：

1. `enable_transient_zero_cmd_hold`
2. `transient_zero_cmd_hold_timeout_ms`
3. `cmd_vel_watchdog_timeout_ms`
4. 上游 `/cmd_vel` 是否夹杂零速帧
5. 上游 `/cmd_vel` 是否已经在速度转换节点中限幅和限加速度

### 2. 姿态模式不生效

先看：

1. `decision/robot_mode`
2. `robot_mode_topic`
3. `packet_typedef.hpp` 中的模式枚举
4. 下位机协议是否与当前约定一致

### 3. TF 不完整或云台投影不对

先看：

1. `serial/gimbal_joint_state`
2. `publish_imu_as_gimbal_joint_state`
3. `accept_legacy_two_axis_joint_state`
4. `small_yaw_is_relative`

## 当前维护边界

1. 改串口协议、模式字段映射、瞬时零速保护，在本包改
2. 改姿态切换规则、视觉接管、受击自旋，不在本包改，去 `pb2025_sentry_behavior`
3. 改 `/cmd_vel` 生成链，不在本包改，去 `pb2025_sentry_nav`
4. 改整车启动和总参数入口，不在本包改，去 `pb2025_sentry_bringup`

## 相关文档

- [../../docs/总览.md](../../docs/总览.md)
- [../../docs/sentry_posture_switch_logic.md](../../docs/sentry_posture_switch_logic.md)
- [../../docs/实机视觉跟随优化方案.md](../../docs/实机视觉跟随优化方案.md)
