# pb2025_sentry_behavior

当前哨兵行为树决策包。

这个包在当前项目中的职责只有一条主线：

```text
订阅 裁判 / 视觉 / 当前位姿 / costmap
  -> 写入行为树黑板
  -> 每个 tick 重新评估 rmul_2026
  -> 先决定当前目标点，再发布姿态模式 / 自旋速度 / 云台指令
  -> 通过 /navigate_through_poses 和 topic 输出给下游
```

## 当前入口

行为树服务端与客户端启动入口：

- [launch/pb2025_sentry_behavior_launch.py](./launch/pb2025_sentry_behavior_launch.py)

当前行为树：

- 主树：
  [behavior_trees/rmul_2026.xml](./behavior_trees/rmul_2026.xml)
- 视觉专测树文件仍保留：
  [behavior_trees/vision_test.xml](./behavior_trees/vision_test.xml)
  但当前 loopback / vision_test 启动入口默认也走 `rmul_2026`

当前参数文件：

- 实机主树：
  [params/sentry_behavior.yaml](./params/sentry_behavior.yaml)
- loopback 主树：
  [params/sentry_behavior_loopback.yaml](./params/sentry_behavior_loopback.yaml)
- 视觉专测：
  [params/sentry_behavior_vision_test.yaml](./params/sentry_behavior_vision_test.yaml)

## 当前主功能

### 1. 路径决策

行为树统一生成 `nav_msgs/Path`，再由：

- [plugins/action/send_nav_through_poses.cpp](./plugins/action/send_nav_through_poses.cpp)

发送到：

- `/navigate_through_poses`

当前已经不再把 `NavigateToPose` 当作主线执行接口。

当前目标点决策语义：

1. 先根据血量、弹量、视觉接管条件、巡航状态决定“应该去哪里”
2. 再根据这个目标点所在分支发布姿态模式
3. 姿态模式是结果，不反向决定导航目标点

### 2. 姿态切换

当前姿态模式固定为：

- `move = 0`
- `attack = 1`
- `defend = 2`

由：

- [plugins/action/pub_robot_mode.cpp](./plugins/action/pub_robot_mode.cpp)

统一处理：

1. 姿态切换冷却时间
2. 单局累计时长限制
3. 不可用姿态时的回退
4. `decision/robot_mode_markers` 可视化

最终输出：

- `decision/robot_mode`

### 3. 受击自旋

由：

- [plugins/condition/is_attacked.cpp](./plugins/condition/is_attacked.cpp)

负责检测是否发生新的掉血。

当前规则：

1. 只要 `RobotStatus.is_hp_deduced == true` 就视为应触发自旋
2. 不再依赖 `hp_deduction_reason == ARMOR_HIT`
3. 连续一段时间没有新的掉血后，停止自旋

相关参数：

- `decision.motion.hit_spin_speed`
- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

### 4. 视觉接管与视觉跟随

由：

- [plugins/condition/is_vision_target_valid.cpp](./plugins/condition/is_vision_target_valid.cpp)
- [plugins/action/select_vision_follow_path.cpp](./plugins/action/select_vision_follow_path.cpp)

负责。

当前规则：

1. 视觉接管基于 `VisionTargetMsg`
2. 导航目标优先使用 `target_position_map`
3. 每个决策周期都会重新按“当前车位 + 当前敌方地图点”计算攻击圆周点
4. 结合 costmap、边界余量、线段可通行性筛选候选点
5. 再用角度限幅平滑，避免目标点突跳

可视化输出：

- `decision/vision_follow_markers`

### 5. 统一资源模式

由：

- [plugins/condition/is_robot_resource_mode.cpp](./plugins/condition/is_robot_resource_mode.cpp)

统一根据血量和弹量判断：

- `engage`
- `resupply`
- `defend`

当前视觉接管和主树资源分支都走这套统一判定。

不过当前主树已经不再把 `defend / resupply` 直接当作“目标点枚举”来使用。
当前实际规则是：

1. 若 `current_hp <= resupply_enter_hp`，优先回补给安全点
2. 若弹量或资源状态进入 `resupply`，也回补给安全点
3. 只有资源健康时，才允许视觉接管、关键时间点或普通巡逻决定目标点
4. `move / attack / defend` 姿态只在具体子树里按结果发布给下位机

## 当前最常改的参数

### 姿态相关

- `decision.mode_limits.switch_cooldown_s`
- `decision.mode_limits.max_cumulative_s`
- `decision.resource_policy.defend_enter_hp`
- `decision.resource_policy.defend_exit_hp`
- `decision.motion.hit_spin_speed`
- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

### 视觉相关

- `decision.vision.attack_radius`
- `decision.vision.min_replan_interval_s`
- `decision.vision.min_goal_shift_m`
- `decision.vision.max_goal_angle_step_deg`
- `decision.vision.pose_jump_reset_distance_m`
- `decision.decision_config.vision_active_goal_hold_tolerance`
- `decision.decision_config.vision_active_goal_min_resend_interval_s`

### 导航接口相关

- `decision.decision_config.path_goal_reached_tolerance`
- `decision.decision_config.active_goal_hold_tolerance`
- `decision.decision_config.active_goal_min_resend_interval_s`

## 当前对外话题

核心输出：

- `decision/robot_mode`
- `cmd_spin`
- `cmd_gimbal`
- `/navigate_through_poses`

核心输入：

- `referee/game_status`
- `referee/robot_status`
- `referee/rfid_status`
- `vision/target`
- `nav_globalCostmap`
- `decision_current_pose`

## 当前维护建议

1. 改决策优先级、姿态切换、视觉接管，优先看 XML 和本包插件
2. 改 loopback 或实机启动参数，不要在本包 README 找 launch，直接去 `pb2025_sentry_bringup`
3. 改 MPPI、goal checker、planner，不在本包内改，去 Nav2 参数文件
4. 姿态模式和下位机串口协议的映射必须与 `standard_robot_pp_ros2` 保持一致

## 相关文档

- [../../docs/sentry_bt_decision_checklist.md](../../docs/sentry_bt_decision_checklist.md)
- [../../docs/sentry_posture_switch_logic.md](../../docs/sentry_posture_switch_logic.md)
- [../../docs/融合.md](../../docs/融合.md)
- [../../docs/视觉跟随仿真调试.md](../../docs/视觉跟随仿真调试.md)
- [../../docs/slim_loopback_refactor.md](../../docs/slim_loopback_refactor.md)
