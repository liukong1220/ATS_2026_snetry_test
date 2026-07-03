# 哨兵行为树决策现状说明

这份文档详细说明当前主树的执行顺序、关键插件算法、参数入口和调试方法。

## 1. 当前主树入口

当前主树：

- [../src/ats_sentry_behavior/behavior_trees/rmul_2026.xml](../src/ats_sentry_behavior/behavior_trees/rmul_2026.xml)

当前视觉专测树：

- [../src/ats_sentry_behavior/behavior_trees/vision_test.xml](../src/ats_sentry_behavior/behavior_trees/vision_test.xml)

当前行为树服务端：

- [../src/ats_sentry_behavior/src/ats_sentry_behavior_server.cpp](../src/ats_sentry_behavior/src/ats_sentry_behavior_server.cpp)

## 2. 当前主线数据流

```text
裁判 / 视觉 / 当前位姿 / costmap
  -> 行为树黑板
  -> rmul_2026 每个 tick 重新评估优先级
  -> 输出姿态模式 / 自旋速度 / 云台指令 / decision_path
  -> SendNavThroughPoses
  -> /navigate_through_poses
  -> SmacPlannerHybrid
  -> Nav2BSplineSmoother
  -> MPPI
  -> trajectory_speed_governor
  -> velocity_smoother
  -> cmd_vel_nav2_result
  -> fake_vel_transform
  -> /cmd_vel
```

## 3. 当前 `rmul_2026` 的根层执行顺序

当前 `rmul_2026` 根层是：

1. `ReactiveFallback`
   先处理受击自旋
2. `ReactiveFallback`
   再处理视觉接管或普通决策分支

### 3.1 第一优先级：受击自旋

根树最前面总会执行：

```xml
<ReactiveFallback>
  <Sequence>
    <IsAttacked .../>
    <PublishSpinSpeed .../>
  </Sequence>
  <PublishSpinSpeed spin_speed="0.0" .../>
</ReactiveFallback>
```

语义：

1. 当前帧检测到新掉血，则发布受击自旋速度
2. 没检测到，则发布 `0.0`

这部分只控制旋转速度，不控制姿态模式。

### 3.2 第二优先级：视觉接管优先于普通决策

当前分支优先级为：

1. `vision_override_realtime`
2. `decision_simulation`
3. `decision_referee`

也就是说：

- 视觉接管一旦成立，会优先抢占普通巡逻/退防分支

## 4. 当前视觉接管分支

当前实现：

- `vision_override_realtime`

执行顺序：

1. `IsRobotResourceMode(state="engage")`
2. `IsVisionTargetValid(...)`
3. `PublishRobotMode(mode="attack")`
4. `PublishGimbalAbsolute`
5. `SelectVisionFollowPath`
6. `IsPathGoalReached / SendNavThroughPoses`

### 4.1 为什么先判资源模式

当前视觉接管不是无条件成立的，必须先满足：

- 当前资源模式为 `engage`

这样可以保证：

1. 低血量时不会继续追击
2. 弹量不足时不会继续攻击
3. 主树和视觉分支使用同一套资源策略

### 4.2 `IsVisionTargetValid` 的算法

对应实现：

- [../src/ats_sentry_behavior/plugins/condition/is_vision_target_valid.cpp](../src/ats_sentry_behavior/plugins/condition/is_vision_target_valid.cpp)

当前判定条件：

1. `tracking=true`
2. `require_nav_hold=true` 时要求 `nav_hold=true`
3. `target_yaw` 与 `target_pitch` 为有限值
4. `timestamp` 有效
5. `timestamp` 未超时

当前还带三层平滑：

1. `activation_hold_s`
   初次出现目标后，不会立即接管，而是先稳定一段时间
2. `switch_target_hold_s`
   已锁定目标 A 后，目标 B 也要稳定一段时间才能切换
3. `override_hold_s`
   短时掉帧、遮挡或单帧异常时，继续保持旧目标

相关参数：

- `decision.vision.timeout_s`
- `decision.vision.activation_hold_s`
- `decision.vision.switch_target_hold_s`
- `decision.vision.override_hold_s`

### 4.3 `SelectVisionFollowPath` 的算法

对应实现：

- [../src/ats_sentry_behavior/plugins/action/select_vision_follow_path.cpp](../src/ats_sentry_behavior/plugins/action/select_vision_follow_path.cpp)

当前核心流程：

1. 读取 `VisionTargetMsg.target_position_map`
2. 确定规划坐标系 `planning_frame`
3. 将目标点和当前车位变换到同一坐标系
4. 根据当前车位计算攻击圆周上的最近角度
5. 生成原始最近圆周点 `nearest_ring_goal`
6. 基于 costmap 筛选候选点：
   - 候选栅格可通行
   - 与地图边界保留最小余量
   - 从当前车位到候选点的连线尽量可通行
7. 若局部候选点都不可用，则扩大搜索范围到更大角域甚至全环
8. 对选中点做角度限幅平滑
9. 若检测到位姿跳变且相对敌方方位也明显变化，则清空缓存重选
10. 最终生成一个单点 `nav_msgs/Path`

相关参数：

- `decision.vision.attack_radius`
- `decision.vision.follow_occupied_threshold`
- `decision.vision.follow_sample_count`
- `decision.vision.follow_arc_half_angle_deg`
- `decision.vision.min_replan_interval_s`
- `decision.vision.min_goal_shift_m`
- `decision.vision.max_goal_angle_step_deg`
- `decision.vision.pose_jump_reset_distance_m`
- `decision.vision.pose_jump_reset_angle_deg`

### 4.4 视觉跟随为什么不是固定点

当前不是追一个写死点，而是每个决策周期都重新依据：

1. 当前车位
2. 当前敌方地图点
3. 当前 costmap

来重新选攻击圆周点。

因此当前预期是：

- 机器人始终尽量去攻击半径上“离自己更近且更安全”的点

## 5. 当前 simulation 与 referee 分支

### 5.1 simulation 分支

当前通过：

- `decision/sim_mode`

决定四个模式：

- `patrol`
- `anchor`
- `retreat`
- `safe`

对应子树：

- `decision_patrol`
- `decision_anchor_target`
- `decision_retreat`
- `decision_safe_point`

### 5.2 referee 分支

当前通过血量、资源模式和比赛时间做分流：

1. `current_hp <= resupply_enter_hp -> decision_safe_point`
2. `resupply -> decision_safe_point`
3. `engage + critical time -> decision_critical_time_target`
4. `engage -> decision_patrol`

## 6. 当前资源模式状态机

对应实现：

- [../src/ats_sentry_behavior/plugins/condition/is_robot_resource_mode.cpp](../src/ats_sentry_behavior/plugins/condition/is_robot_resource_mode.cpp)

当前只使用两类输入：

- `RobotStatus.current_hp`
- `RobotStatus.projectile_allowance_17mm`

当前状态：

- `engage`
- `resupply`
- `defend`

当前算法仍保留迟滞锁存状态机，但主树对目标点的使用方式已经调整为：

1. 若当前血量低于 `resupply_enter_hp`，直接回补给安全点
2. 若当前已在 `resupply`，继续回补给安全点，直到血量和弹量恢复
3. 若当前是 `engage`，才允许关键时间点、视觉接管和普通巡逻决定目标点

相关参数：

- `decision.resource_policy.defend_enter_hp`
- `decision.resource_policy.defend_exit_hp`
- `decision.resource_policy.resupply_enter_hp`
- `decision.resource_policy.resupply_exit_hp`
- `decision.resource_policy.resupply_enter_ammo`
- `decision.resource_policy.resupply_exit_ammo`

## 7. 当前姿态发布位置

当前姿态不是根节点统一发布，而是跟随子树语义发布：

### 7.1 `attack`

由：

- `vision_override_realtime`

发布。

### 7.2 `defend`

由：

- `decision_safe_point`

发布。

当前 `defend` 更像“当前目标点决策结果对应的姿态发布”，
不再反向决定目标点必须去退防点。

### 7.3 `move`

由：

- `decision_patrol`
- `decision_anchor_target`
- `decision_critical_time_target`

发布。

统一姿态裁决节点：

- [../src/ats_sentry_behavior/plugins/action/pub_robot_mode.cpp](../src/ats_sentry_behavior/plugins/action/pub_robot_mode.cpp)

## 8. 当前路径执行稳定器

当前 `SendNavThroughPoses` 的核心算法：

1. 读取当前路径
2. 检查 action server 是否可用
3. 读取黑板或 TF 中的当前位姿
4. 判断是否与当前活动路径相同或近似相同
5. 若当前路径已成功且当前位姿仍在终点附近，则不重发
6. 若当前仍有活动 goal 且新路径只是微小变化，则在一段时间内不抢占
7. 只有必要时才 cancel + resend

特别针对视觉单点路径，又单独给了一组更积极的保持阈值：

- `vision_active_goal_hold_tolerance`
- `vision_active_goal_min_resend_interval_s`

## 9. 当前 loopback 与实车共用边界

共用：

- 主树结构
- 姿态切换
- 资源模式状态机
- 视觉接管判定
- 攻击圆周点选择
- `SendNavThroughPoses` 稳定器

不同：

- 输入源
- Nav2 参数
- 串口与真实传感器链路

## 10. 当前最常改的文件

| 需求 | 优先修改位置 |
| --- | --- |
| 改姿态切换 | `pub_robot_mode.cpp` 与参数文件 |
| 改资源门控 | `is_robot_resource_mode.cpp` 与参数文件 |
| 改视觉接管判定 | `is_vision_target_valid.cpp` |
| 改视觉圆周跟随点选择 | `select_vision_follow_path.cpp` |
| 改路径重发节流 | `send_nav_through_poses.cpp` |
| 改普通巡逻/退防路径 | 路径选择插件与 `decision.goal_points` |

## 11. 调试建议

### 11.1 看主树当前在干什么

优先看日志：

- `Decision resource mode=...`
- `Vision override rejected: ...`
- `Vision follow target=(...) nearest_ring_goal=(...) selected_goal=(...)`
- `Send NavigateThroughPoses goal with ...`
- `Robot posture switched: ...`

### 11.2 验证视觉跟随

优先看：

- `/vision/target`
- `/decision/vision_follow_markers`
- `/decision/robot_mode`

### 11.3 验证重发节流

优先看：

- 是否反复出现同一个 `Send NavigateThroughPoses goal`
- `Goal succeeded` 后是否仍继续重发相同终点

## 12. 相关文档

- [./sentry_posture_switch_logic.md](./sentry_posture_switch_logic.md)
- [./融合.md](./融合.md)
- [./视觉跟随仿真调试.md](./视觉跟随仿真调试.md)
- [./slim_loopback_refactor.md](./slim_loopback_refactor.md)
