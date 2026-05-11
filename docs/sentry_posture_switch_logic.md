# 哨兵姿态切换与受击自旋说明

这份文档详细说明当前仓库中姿态模式、资源状态机、受击自旋和下位机下发链路。

涉及代码主要在：

- `src/pb2025_sentry_behavior`
- `src/pb2025_sentry_nav/fake_vel_transform`
- `src/standard_robot_pp_ros2`

## 1. 当前这套逻辑要解决什么

当前实现同时解决四件事：

1. 给下位机发送当前姿态模式
2. 避免姿态在高频 tick 中来回抖动
3. 限制单局比赛中某一姿态累计占用时间
4. 检测到新的掉血时触发自旋，并在一段时间没有新掉血后自动停转

## 2. 当前两条主链

### 2.1 姿态模式链

```text
行为树分支
  -> PublishRobotMode(mode=move/attack/defend)
  -> resolveModeWithConstraints()
  -> 发布 decision/robot_mode
  -> standard_robot_pp_ros2 订阅
  -> 写入 SendRobotCmdData.data.speed_vector.mode
  -> 串口发送给下位机
```

### 2.2 受击自旋链

```text
referee/robot_status
  -> IsAttacked
  -> PublishSpinSpeed
  -> 发布 cmd_spin
  -> fake_vel_transform 把 cmd_spin 叠加到 /cmd_vel
  -> standard_robot_pp_ros2 订阅 /cmd_vel
  -> 把 angular.z 写入 speed_vector.wz
  -> 串口发送给下位机
```

## 3. 当前姿态定义

当前姿态固定为：

- `move = 3`
- `attack = 1`
- `defend = 2`

行为树发布：

- `decision/robot_mode`

串口层最终写入：

```cpp
send_robot_cmd_data_.data.speed_vector.mode
```

协议定义位置：

- [../src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp](../src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp)

## 4. 当前三种姿态分别何时触发

### 4.1 `move`

当前普通机动分支会发布 `move`：

1. 巡逻
2. 锚点移动
3. 关键时间点位移动

### 4.2 `attack`

当前只有视觉接管成立时才会发布 `attack`。

当前必须同时满足：

1. 资源模式是 `engage`
2. 视觉目标有效
3. `nav_hold=true`
4. 视觉消息未超时

### 4.3 `defend`

当前 `defend` 更像“当前任务结果对应的姿态发布”：

1. 回补给安全点时会发布 `defend`
2. 视觉失效、资源不健康、不适合继续追击时也可能发布 `defend`

当前姿态不再反向决定目标点，目标点先由血量 / 弹量 / 视觉 / 巡航条件决定，
随后由具体子树发布 `move / attack / defend` 给下位机。

## 5. 当前姿态裁决算法

对应实现：

- [../src/pb2025_sentry_behavior/plugins/action/pub_robot_mode.cpp](../src/pb2025_sentry_behavior/plugins/action/pub_robot_mode.cpp)

当前内部做了三步：

### 5.1 解析请求姿态

行为树 XML 中写的是：

- `move`
- `attack`
- `defend`

`PublishRobotMode` 会先统一解析成数值枚举。

### 5.2 根据冷却时间决定是否允许切换

当前参数：

- `decision.mode_limits.switch_cooldown_s`

算法含义：

1. 当前分支请求了一个新姿态
2. 如果距离上次真正切换姿态还没到冷却时间
3. 则继续保持旧姿态，不立即切换

### 5.3 根据单局累计时长决定该姿态是否还能继续使用

当前参数：

- `decision.mode_limits.max_cumulative_s`

算法含义：

1. 只在比赛 `RUNNING` 阶段累计姿态使用时长
2. 如果当前姿态累计时间已经超限
3. 则从允许的姿态里选择一个合法回退姿态

## 6. 当前资源状态机算法

对应实现：

- [../src/pb2025_sentry_behavior/plugins/condition/is_robot_resource_mode.cpp](../src/pb2025_sentry_behavior/plugins/condition/is_robot_resource_mode.cpp)

当前资源模式只有三种：

- `engage`
- `resupply`
- `defend`

当前输入字段：

- `RobotStatus.current_hp`
- `RobotStatus.projectile_allowance_17mm`

当前算法仍是迟滞锁存状态机，但当前主树对“目标点”的实际使用方式已经调整为：

1. 当前处于 `engage` 时
   - 若血量低于补给进入阈值，主树优先回补给安全点
   - 若弹量不足，也优先回补给安全点
   - 否则保持 `engage`，允许视觉接管或普通巡逻决定目标点
2. 当前处于 `resupply` 时
   - 若血量与弹量都恢复到退出阈值以上，回到 `engage`
   - 否则保持 `resupply`
3. 当前 `defend` 资源状态仍保留给运行时观测与姿态发布，
   但当前主树不再让它单独决定“去最近退防点还是安全区”

当前参数：

- `decision.resource_policy.defend_enter_hp`
- `decision.resource_policy.defend_exit_hp`
- `decision.resource_policy.resupply_enter_hp`
- `decision.resource_policy.resupply_exit_hp`
- `decision.resource_policy.resupply_enter_ammo`
- `decision.resource_policy.resupply_exit_ammo`

当前推荐理解：

1. `resupply_enter_hp / resupply_exit_hp` 控制“血量低到要不要回安全区”
2. `resupply_enter_ammo / resupply_exit_ammo` 控制“弹量低到要不要回安全区”
3. `defend_*` 现在更偏向资源状态观测与姿态语义，不再单独主导目标点选择

## 7. 当前受击自旋检测算法

对应实现：

- [../src/pb2025_sentry_behavior/plugins/condition/is_attacked.cpp](../src/pb2025_sentry_behavior/plugins/condition/is_attacked.cpp)

当前触发条件：

```cpp
const bool is_attacked = msg->is_hp_deduced;
```

也就是说：

1. 只要检测到新的掉血，就触发自旋
2. 不再额外要求 `hp_deduction_reason == ARMOR_HIT`

当前这样设计的目的，是在高频掉血或裁判反馈存在延迟时，也能更快进入保护性自旋。

### 7.1 当前锁存逻辑

检测到掉血后，节点会：

1. 记录最近一次掉血时间
2. 记录最近一次受击装甲方向
3. 在一段时间内持续返回 `SUCCESS`

当前参数：

- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

语义不是“固定自旋总时长”，而是：

- 最近一次掉血后，如果连续这段时间没有新的掉血，就停转

## 8. 当前自旋速度是如何进入下位机的

对应实现链路：

### 8.1 行为树发布 `cmd_spin`

对应节点：

- [../src/pb2025_sentry_behavior/plugins/action/pub_spin_speed.cpp](../src/pb2025_sentry_behavior/plugins/action/pub_spin_speed.cpp)

它只负责发布一个标量：

- `spin_speed`

### 8.2 `fake_vel_transform` 把自旋速度叠加到 `/cmd_vel`

对应实现：

- [../src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp](../src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp)

关键代码语义：

```cpp
aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
```

也就是说：

1. Nav2 正常输出线速度与角速度
2. 行为树额外输出 `cmd_spin`
3. `fake_vel_transform` 把两者合成为最终 `/cmd_vel`

### 8.3 `standard_robot_pp_ros2` 把 `/cmd_vel.angular.z` 写入 `wz`

对应实现：

- [../src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp](../src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp)

最终写入：

```cpp
send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
```

因此当前可以明确认为：

- 自旋速度最终就是通过 `wz` 发给下位机的

## 9. 当前与下位机模式位的关系

当前上位机发布的是枚举语义，但下位机完全可以按整数位接收。

也就是说，下面两种理解在作用上是等价的：

1. 上位机使用 `enum` 表达 `move / attack / defend`
2. 下位机使用 `int32_t mode`，按 `0 / 1 / 2` 判断

只要双方约定一致即可：

- `0 -> move`
- `1 -> attack`
- `2 -> defend`

## 10. 当前最常调的参数

### 10.1 姿态相关

- `decision.mode_limits.switch_cooldown_s`
- `decision.mode_limits.max_cumulative_s`
- `decision.resource_policy.defend_enter_hp`
- `decision.resource_policy.defend_exit_hp`
- `decision.resource_policy.resupply_enter_hp`
- `decision.resource_policy.resupply_exit_hp`

### 10.2 自旋相关

- `decision.motion.hit_spin_speed`
- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

### 10.3 视觉接管相关

- `decision.vision.timeout_s`
- `decision.vision.activation_hold_s`
- `decision.vision.switch_target_hold_s`
- `decision.vision.override_hold_s`
- `decision.vision.attack_radius`

## 11. 当前调试命令

### 11.1 查看姿态模式

```bash
ros2 topic echo /decision/robot_mode
```

### 11.2 查看姿态 Marker

```bash
ros2 topic echo /decision/robot_mode_markers
```

### 11.3 查看自旋速度链

```bash
ros2 topic echo /cmd_spin
ros2 topic echo /cmd_vel
```

### 11.4 loopback 下测试受击自旋

```bash
ros2 param set /fake_decision_sim_inputs current_hp 280
ros2 param set /fake_decision_sim_inputs is_hp_deduced true
```

### 11.5 修改姿态和自旋参数

```bash
ros2 param set /pb2025_sentry_behavior_server decision.mode_limits.switch_cooldown_s 3.0
ros2 param set /pb2025_sentry_behavior_server decision.mode_limits.max_cumulative_s 120.0
ros2 param set /pb2025_sentry_behavior_server decision.motion.hit_spin_speed 5.5
ros2 param set /pb2025_sentry_behavior_server decision.motion.hit_spin_stop_after_no_hp_drop_s 1.5
```

## 12. 当前实车与 loopback 是否共用这套逻辑

共用部分：

1. 姿态切换算法
2. 冷却时间与累计时长限制
3. 资源模式状态机
4. 受击自旋检测
5. 自旋速度叠加逻辑
6. 视觉接管触发 `attack`

差异主要只在输入源：

- loopback 由 `fake_decision_sim_inputs.py` 伪造输入
- 实机由串口与真实视觉链路提供输入

## 13. 相关代码入口

- 姿态裁决：  
  `src/pb2025_sentry_behavior/plugins/action/pub_robot_mode.cpp`
- 受击检测：  
  `src/pb2025_sentry_behavior/plugins/condition/is_attacked.cpp`
- 自旋速度发布：  
  `src/pb2025_sentry_behavior/plugins/action/pub_spin_speed.cpp`
- 速度合成：  
  `src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp`
- 串口模式下发：  
  `src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp`
- 串口协议定义：  
  `src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp`
