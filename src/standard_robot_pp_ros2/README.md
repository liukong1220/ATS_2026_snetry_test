# standard_robot_pp_ros2

当前项目中的上下位机串口接口层。

这个包在当前仓库里的职责是：

1. 接收下位机串口数据并发布成 ROS topic
2. 接收上层行为树和导航输出并写回串口发送结构
3. 作为实机链路中 `cmd_vel`、姿态模式、裁判系统信息的接口桥梁

## 当前入口

启动文件：

- [launch/standard_robot_pp_ros2.launch.py](./launch/standard_robot_pp_ros2.launch.py)

默认参数文件：

- [config/standard_robot_pp_ros2.yaml](./config/standard_robot_pp_ros2.yaml)

在整车主线中，通常由：

- [../pb2025_sentry_bringup/launch/bringup.launch.py](../pb2025_sentry_bringup/launch/bringup.launch.py)

统一拉起。

## 当前与上层决策的对接关系

### 1. 姿态模式

当前行为树通过：

- `decision/robot_mode`

发布姿态模式。

本包订阅该话题后，写入串口发送结构中的：

- `SendRobotCmdData.data.speed_vector.mode`

当前模式约定固定为：

- `move = 0`
- `attack = 1`
- `defend = 2`

对应代码位置：

- [src/standard_robot_pp_ros2.cpp](./src/standard_robot_pp_ros2.cpp)
- [include/standard_robot_pp_ros2/packet_typedef.hpp](./include/standard_robot_pp_ros2/packet_typedef.hpp)

### 2. 底盘速度

当前本包订阅：

- `/cmd_vel`

并把速度写入：

- `SendRobotCmdData.data.speed_vector.vx`
- `SendRobotCmdData.data.speed_vector.vy`
- `SendRobotCmdData.data.speed_vector.wz`

因此：

- 上层受击自旋最终就是通过 `/cmd_vel.angular.z`
- 再映射到串口结构体的 `speed_vector.wz`

### 3. 裁判系统数据

当前本包负责把串口中的裁判系统数据转成 ROS topic，供行为树直接消费：

- `referee/game_status`
- `referee/robot_status`
- `referee/rfid_status`

这也是姿态切换、低血量防御、受击自旋等逻辑的数据来源。

## 当前关键参数

参数文件：

- [config/standard_robot_pp_ros2.yaml](./config/standard_robot_pp_ros2.yaml)

当前和行为树主线最相关的参数：

- `device_name`
- `baud_rate`
- `robot_mode_topic`

其中：

- `robot_mode_topic` 默认就是 `decision/robot_mode`

## 当前运行方式

### 单独启动串口层

```bash
source install/setup.bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
```

### 在整车主线中启动

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup bringup.launch.py world:=<YOUR_WORLD_NAME> use_rviz:=True
```

## 当前维护建议

1. 改姿态切换规则、攻击/防御触发逻辑，不在本包改，去 `pb2025_sentry_behavior`
2. 改模式枚举和串口协议字段映射，要同时检查本包和行为层是否一致
3. 改自旋速度时，不在本包直接写死，优先调行为树参数 `decision.motion.hit_spin_speed`
4. 改模式话题名时，要同步检查：
   - `pb2025_sentry_behavior` 的 `decision.topics.robot_mode`
   - 本包的 `robot_mode_topic`

## 相关文档

- [../../docs/sentry_posture_switch_logic.md](../../docs/sentry_posture_switch_logic.md)
- [../../docs/sentry_bt_decision_checklist.md](../../docs/sentry_bt_decision_checklist.md)
- [../../docs/实机视觉跟随优化方案.md](../../docs/实机视觉跟随优化方案.md)
