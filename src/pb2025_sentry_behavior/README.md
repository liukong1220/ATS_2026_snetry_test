# pb2025_sentry_behavior

当前哨兵项目中的行为树决策包。

本包的当前职责是：

1. 订阅裁判系统、视觉、位姿、costmap 和仿真输入
2. 在黑板中维护统一决策上下文
3. 每个 tick 重新评估主树
4. 输出路径、姿态模式、自旋速度和云台控制

## 当前入口

启动文件：

- [launch/pb2025_sentry_behavior_launch.py](./launch/pb2025_sentry_behavior_launch.py)

当前主树：

- [behavior_trees/rmul_2026.xml](./behavior_trees/rmul_2026.xml)

当前视觉专测树：

- [behavior_trees/vision_test.xml](./behavior_trees/vision_test.xml)

当前参数文件：

- 实机主树：
  [params/sentry_behavior.yaml](./params/sentry_behavior.yaml)
- loopback 主树：
  [params/sentry_behavior_loopback.yaml](./params/sentry_behavior_loopback.yaml)
- 视觉专测：
  [params/sentry_behavior_vision_test.yaml](./params/sentry_behavior_vision_test.yaml)

## 当前主线

```text
裁判系统 / 视觉 / 当前位姿 / costmap / 仿真输入
  -> pb2025_sentry_behavior_server
  -> 行为树黑板
  -> rmul_2026
  -> 决定当前路径 / 姿态模式 / 自旋 / 云台
  -> /navigate_through_poses + decision/robot_mode + cmd_spin + cmd_gimbal
```

## 当前关键功能

### 1. 路径输出

当前统一执行接口是：

- `/navigate_through_poses`

对应动作节点：

- [plugins/action/send_nav_through_poses.cpp](./plugins/action/send_nav_through_poses.cpp)

当前不再把旧 `NavigateToPose` 作为主链接口。

### 2. 姿态切换

当前姿态模式固定映射为：

- `move = 3`
- `attack = 1`
- `defend = 2`

发布节点：

- [plugins/action/pub_robot_mode.cpp](./plugins/action/pub_robot_mode.cpp)

最终输出：

- `decision/robot_mode`

### 3. 视觉接管

当前视觉接管核心文件：

- [plugins/condition/is_vision_target_valid.cpp](./plugins/condition/is_vision_target_valid.cpp)
- [plugins/action/select_vision_follow_path.cpp](./plugins/action/select_vision_follow_path.cpp)

当前视觉接管成立需要同时满足：

1. `tracking = true`
2. `nav_hold = true`（若配置要求）
3. 消息未超时
4. 资源状态允许 `engage`
5. `target_position_map` 可用

### 4. 资源门控

当前统一资源状态机：

- [plugins/condition/is_robot_resource_mode.cpp](./plugins/condition/is_robot_resource_mode.cpp)

当前资源状态：

- `engage`
- `resupply`
- `defend`

输入主要来自：

- `referee/robot_status`

### 5. 受击自旋

当前受击检测：

- [plugins/condition/is_attacked.cpp](./plugins/condition/is_attacked.cpp)

当前自旋速度发布：

- [plugins/action/pub_spin_speed.cpp](./plugins/action/pub_spin_speed.cpp)

最终输出：

- `cmd_spin`

## 当前关键输入 / 输出

### 核心输入

- `referee/game_status`
- `referee/robot_status`
- `referee/rfid_status`
- `vision/target`
- `nav_globalCostmap`
- `decision_current_pose`

### 核心输出

- `/navigate_through_poses`
- `decision/robot_mode`
- `cmd_spin`
- `cmd_gimbal`
- `decision/vision_follow_markers`
- `decision/robot_mode_markers`

## 当前维护边界

1. 改决策优先级、视觉接管、资源门控、姿态切换，优先改本包
2. 改 planner / MPPI / smoother / 恢复行为，不在本包改，去 `pb2025_sentry_nav`
3. 改实机总启动、参数整合、loopback 假输入，不在本包改，去 `pb2025_sentry_bringup`
4. 改串口协议和模式字段映射，不在本包改，去 `standard_robot_pp_ros2`

## 相关文档

- [../../docs/总览.md](../../docs/总览.md)
- [../../docs/融合.md](../../docs/融合.md)
- [../../docs/sentry_bt_decision_checklist.md](../../docs/sentry_bt_decision_checklist.md)
- [../../docs/sentry_posture_switch_logic.md](../../docs/sentry_posture_switch_logic.md)
- [../../docs/视觉跟随仿真调试.md](../../docs/视觉跟随仿真调试.md)
- [../../docs/实机视觉跟随优化方案.md](../../docs/实机视觉跟随优化方案.md)
