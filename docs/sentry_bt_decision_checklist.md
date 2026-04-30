# Sentry 行为树决策学习手册

## 1. 文档定位

这份文档以当前仓库代码为准，面向三类读者：

1. 想理解“哨兵现在到底怎么决策、怎么导航”的新同学。
2. 想修改巡逻点、安全点、退防点、视觉接管逻辑的维护者。
3. 想从历史 `goal_pose / PID` 思路迁移到当前 `BT + NavigateThroughPoses + MPPI` 架构的开发者。

当前主决策树是：

- [`../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml`](../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml)

当前行为树服务端入口是：

- [`../src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp`](../src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp)

当前姿态切换、模式冷却、单局累计时长限制，以及受击自旋的最新规则请同时参考：

- [`./sentry_posture_switch_logic.md`](./sentry_posture_switch_logic.md)

如果你只想先抓住主线，可以先记住下面这一张链路图。

```text
launch 文件
  -> pb2025_sentry_behavior_server
  -> 订阅裁判 / 视觉 / 里程计 / costmap
  -> 写入 BehaviorTree 黑板
  -> rmul_2026.xml 每个 tick 重新评估优先级
  -> 选择一条 decision_path
  -> SendNavThroughPoses 发送 /navigate_through_poses
  -> Nav2 Planner + MPPI Controller 执行
  -> 输出 cmd_vel / 底盘运动
```

当前导航恢复链也建议一起记住：

```text
行为树生成 decision_path
  -> SendNavThroughPoses
  -> Nav2 Planner + MPPI FollowPath
  -> 若局部长期无有效进展
  -> progress_checker 判定失败
  -> BT RecoveryFallback
  -> 清空 costmap
  -> BackUpFreeSpace 选择低代价退让方向
  -> 重新回到 FollowPath
```

这意味着：

1. 行为树负责“该去哪”。
2. Nav2 负责“如何规划、如何控制、何时认定卡住”。
3. 恢复动作已经是当前主导航链的一部分，而不是旧 PID 时代的旁路补丁。

---

## 2. 当前工程里谁负责什么

| 包或文件 | 作用 | 维护时什么时候看 |
| --- | --- | --- |
| [`../src/pb2025_sentry_bringup`](../src/pb2025_sentry_bringup) | 全系统启动入口，负责把串口、导航、行为树、RViz 串起来 | 你要改启动链路、实机入口、loopback 入口时 |
| [`../src/pb2025_sentry_behavior`](../src/pb2025_sentry_behavior) | 决策核心，包含行为树 XML、BT 插件、行为树服务端/客户端 | 你要改决策逻辑、路径选择、视觉接管时 |
| [`../src/pb2025_sentry_nav/pb2025_nav_bringup`](../src/pb2025_sentry_nav/pb2025_nav_bringup) | Nav2、定位、地图、点云、RViz 等导航启动链路 | 你要改导航栈、实车 Nav2 参数时 |
| [`../src/loopback_sim`](../src/loopback_sim) | `nav2_loopback_sim`，用于无物理仿真的轻量 loopback | 你要做快速仿真、调 MPPI 局部控制时 |
| [`../src/pb2025_sentry_nav/sp_msgs`](../src/pb2025_sentry_nav/sp_msgs) | 视觉与行为树之间的消息契约 | 你要改 `VisionTargetMsg` 结构时 |
| [`../src/standard_robot_pp_ros2`](../src/standard_robot_pp_ros2) | 串口驱动、机器人本体接口 | 你要接实车底盘/云台/裁判系统时 |

最常用的启动入口有两个：

1. loopback 学习/调参入口  
   [`../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py`](../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)
2. 实机总入口  
   [`../src/pb2025_sentry_bringup/launch/bringup.launch.py`](../src/pb2025_sentry_bringup/launch/bringup.launch.py)

---

## 3. 先理解行为树，不然看 XML 会很痛苦

### 3.1 什么是行为树

行为树可以把它理解成“每个周期都重新问自己一遍，现在最应该做什么”的决策图。

和传统 `if-else` 大串判断相比，它有三个优点：

1. 优先级关系直接画在树结构里，可读性高。
2. 条件节点和动作节点可以复用。
3. 高优先级事件可以在下一个 tick 立刻抢占低优先级行为。

当前项目里，行为树每个 tick 做的事情并不是“直接开车”，而是：

1. 看黑板上现在有什么输入。
2. 选出当前应该走的分支。
3. 生成一条 `decision_path`。
4. 把路径交给 Nav2。

也就是说：

- 行为树负责“决策和派单”。
- Nav2 负责“真正规划与控制”。
- MPPI 负责“局部控制与轨迹采样优化”。

### 3.2 什么是黑板

黑板就是行为树共享内存。

当前项目中，服务端会把订阅到的数据写进根黑板，例如：

- `referee_gameStatus`
- `referee_robotStatus`
- `sp_vision_target`
- `decision_current_pose`
- `nav_globalCostmap`
- `decision_input_source`
- `decision_sim_mode`

在 XML 里你会看到两种写法：

- `{@xxx}`：表示从根黑板直接取值，适合全局参数或全局状态。
- `{xxx}`：表示普通端口变量，通常用于在同一分支里传递中间结果。

比如：

- `{@decision_input_source}` 是全树共享的“当前输入源”。
- `{decision_path}` 是当前分支刚刚生成的路径。

### 3.3 当前主树里最常见的控制节点含义

#### `Sequence`

按顺序执行子节点，前面任何一个失败，整个分支就失败。

适合理解为：

- “先满足条件，再做动作。”

#### `Fallback`

按顺序尝试子节点，前面任何一个成功，整个分支就成功。

适合理解为：

- “先试方案 A，不行再试方案 B。”

#### `ReactiveSequence`

每个 tick 都会从第一个子节点重新检查。

适合理解为：

- “只要前置条件一变，后面的动作立刻失效。”

当前项目里，`decision_referee` 用它来保证：

- 比赛开始条件不满足时，不会继续跑后面的血量/时间决策。

#### `ReactiveFallback`

每个 tick 都从第一个子节点重新尝试，前面的高优先级分支一旦成功，就直接抢占后面的分支。

当前项目里最关键的一层就是这个：

- 视觉接管分支排在最前面。
- simulation 分支排第二。
- referee 分支排第三。

但这里要特别注意：

- 现在“排在最前面”不再等于“视觉无条件最高优先级”。
- 真正决定视觉是否能成功接管的，是 `IsVisionTargetValid` 条件节点内部的门控逻辑。
- 它已经把“目标是否稳定”“目标是否正在切换”“消息是否只是短时丢失”“当前血量是否已经该回防”都纳入判断。

所以更准确的理解是：

- 视觉分支拥有第一抢占权。
- 但只有在战术上允许、目标也足够稳定时，它才真的会抢占导航。

#### `KeepRunningUntilFailure`

只要子树不失败，就一直反复 tick。

它把整棵树变成持续运行的决策循环。

#### `ForceSuccess`

无论子树返回什么，外层都把它当作成功。

当前根节点外面包了一层 `ForceSuccess`，目的不是“掩盖错误”，而是：

1. 让整棵树持续运行，不因为某一帧的条件失败直接退出。
2. 把“这一 tick 没选中任何分支”视为正常运行中的一个状态。

#### `ForceFailure`

无论子树返回什么，外层都把它当作失败。

当前它用在“比赛未开始时停车”场景：

1. 先发 `spin=0`。
2. 再发 `cmd_vel=0`。
3. 然后故意返回失败，让更外层的逻辑知道“比赛还没开始，不要继续进入正式决策分支”。

---

## 4. 当前主树 `rmul_2026.xml` 是怎么工作的

### 4.1 顶层结构

`rmul_2026` 的主干可以读成：

1. 先对“是否受击”做一次高优先级判断。
2. 如果检测到新的掉血，则发布 `decision.motion.hit_spin_speed`；如果在 `decision.motion.hit_spin_stop_after_no_hp_drop_s` 这段时间内没有新的掉血，则回到 `0.0`。
3. 再在三个总分支里做优先级仲裁：
   - `vision_override_realtime`
   - `decision_simulation`
   - `decision_referee`

当前版本新增的关键变化是：

1. 视觉分支仍然放在最前面，保证允许接管时响应足够快。
2. 低血量时不会因为“视觉排第一”就继续攻击跟随，因为 `IsVisionTargetValid` 会直接阻止该分支成功。
3. 也就是说，现在不是靠“把视觉分支挪到后面”实现保命优先，而是靠“视觉分支自己在不该抢占时失败”实现平滑让权。

姿态模式不是单独在根节点统一发，而是跟随具体分支发送：

- 视觉接管分支发送 `attack`
- 巡逻 / 锚点 / 关键时间移动分支发送 `move`
- 低血量退防 / 安全点分支发送 `defend`

这些姿态发布都经过 `PublishRobotMode` 统一裁决，包含：

- `5s` 姿态切换冷却
- 每局单姿态累计 `180s` 上限
- 姿态超限后的保持或回退策略

这层结构写在：

- [`../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml`](../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml)

### 4.2 为什么视觉分支排第一

因为它是顶层 `ReactiveFallback` 的第一个子树：

1. 视觉消息存在且时间戳未超时。
2. `tracking == true`。
3. 如果 XML 要求，则 `nav_hold == true`。
4. `target_yaw / target_pitch` 是有限值。
5. 初次进入视觉接管前，目标已连续稳定满足 `decision.vision.activation_hold_s`。
6. 若当前锁定目标 A，新目标 B 只有连续稳定满足 `decision.vision.switch_target_hold_s` 才允许切换。
7. 若只是短时掉帧或短时遮挡，则在 `decision.vision.override_hold_s` 内仍允许保持原目标。
8. 视觉分支前先经过 `IsRobotResourceMode state="engage"`，只有资源状态是 `engage` 时才允许视觉接管。
9. 若血量/弹量触发 `resupply` 或 `defend`，视觉分支会主动失败，主树回到补给或退防分支。
10. 资源模式内部通过 `decision.resource_policy.*` 的 enter / exit 阈值做迟滞，避免在边界附近来回横跳。

只有这些条件都满足后，它才会成功吃掉本 tick，后面的 simulation/referee 不会再参与。

这代表当前设计哲学是：

- 视觉接管属于高优先级实时 override。
- 但它不再是“只要看到目标就绝对压制”的硬抢占。
- 现在的语义更接近“满足战术约束和稳定性约束后，才允许插队”。

### 4.3 simulation 分支是什么意思

simulation 分支主要服务于 loopback 仿真和调试，它不依赖真实裁判系统。

它只关心：

- `decision.input_source == simulation`
- `decision/sim_mode` 当前是什么

支持四种模式：

1. `safe`
2. `retreat`
3. `anchor`
4. `patrol`

因此在 loopback 里，你可以单独验证：

- 安全点逻辑是否正确
- 退防点选择是否正确
- 锚点驻守是否正确
- 巡逻状态机是否正确

而不必先把整套裁判系统输入都伪造完整。

### 4.4 referee 分支是什么意思

referee 分支面向实机正式比赛输入。

它的优先级从高到低是：

1. 比赛未开始：停转、停车
2. `resource_mode = defend`：去最近退防点
3. `resource_mode = resupply`：去补给安全点
4. `resource_mode = engage` 且比赛时间 `critical`：去关键时刻目标点
5. `resource_mode = engage`：巡逻

这里的“血量区间”不是写死的常量，而是和比赛时间阶段一起判定：

- 时间越紧，血量判断阈值可以不同。

对应逻辑主要在：

- [`../src/pb2025_sentry_behavior/plugins/condition/is_game_time_stage.cpp`](../src/pb2025_sentry_behavior/plugins/condition/is_game_time_stage.cpp)
- [`../src/pb2025_sentry_behavior/plugins/condition/is_hp_band.cpp`](../src/pb2025_sentry_behavior/plugins/condition/is_hp_band.cpp)
- [`../src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/decision_utils.hpp`](../src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/decision_utils.hpp)

---

## 5. 每个业务子树到底在做什么

### 5.1 `decision_safe_point`

逻辑很直接：

1. 先重置低血量粘滞目标。
2. 选出固定安全点。
3. 如果已经到达，就不再重发。
4. 如果还没到达，就发送 `NavigateThroughPoses`。

关键节点：

- `ResetLowHpTarget`
- `SelectFixedPath`
- `IsPathGoalReached`
- `SendNavThroughPoses`

### 5.2 `decision_retreat`

退防分支比安全点多了一层“最近点选择”和“粘滞保持”：

1. 第一次进入时，根据当前位姿在候选退防点中选最近点。
2. 选中后，把这个索引写回 `decision_low_hp_target_index`。
3. 后续 tick 会优先沿用这个索引，不会在多个退防点之间来回跳。
4. 到点后再重置这个索引。

这样做的目的，是防止机器人低血量时因为位姿抖动而反复换退防目标。

关键文件：

- [`../src/pb2025_sentry_behavior/plugins/action/select_nearest_retreat_path.cpp`](../src/pb2025_sentry_behavior/plugins/action/select_nearest_retreat_path.cpp)
- [`../src/pb2025_sentry_behavior/plugins/action/reset_low_hp_target.cpp`](../src/pb2025_sentry_behavior/plugins/action/reset_low_hp_target.cpp)

### 5.3 `decision_anchor_target`

本质上和安全点一样，差别只是目标索引不同：

- 安全点用 `decision_supply_safe_point_index`
- 锚点用 `decision_anchor_target_index`

### 5.4 `decision_critical_time_target`

本质上也是固定点路径，只是使用：

- `decision_critical_time_target_index`

它表达的是：

- 在关键比赛时间阶段，把哨兵拉到一个固定高价值位置。

### 5.5 `decision_patrol`

巡逻是当前项目里最容易误解的一段。

它现在不是“把整条巡逻链路一次性全发给 Nav2”，而是：

1. 根据 `patrol_cursor` 和 `patrol_direction` 选出“当前目标巡逻点”。
2. 只生成一段单点 path。
3. 到点后再把游标推进到下一个巡逻点。
4. 方向到边界后会自动反向，形成往返巡逻。

换句话说，当前巡逻状态机是：

- `cursor`：当前在巡逻点列表中的位置
- `direction`：当前往前走还是往后走
- `next_cursor / next_direction`：到点后应切换成的下一个状态

状态推进逻辑在：

- [`../src/pb2025_sentry_behavior/plugins/action/select_patrol_path.cpp`](../src/pb2025_sentry_behavior/plugins/action/select_patrol_path.cpp)
- [`../src/pb2025_sentry_behavior/plugins/action/advance_patrol_cursor.cpp`](../src/pb2025_sentry_behavior/plugins/action/advance_patrol_cursor.cpp)
- [`../src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/decision_utils.hpp`](../src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/decision_utils.hpp)

当前实现还有一个非常重要的细节：

- `patrol_preview_points` 参数虽然保留着，
- 但 `SelectPatrolPath` 会把巡逻预览强制收敛到“单个目标点”，并在参数大于 1 时打印警告。

这样做的原因是为了避免：

- 多巡逻点预发送造成高频抢占
- 在行为树高频 tick 下不断取消旧 waypoint goal
- 导航和决策互相打架，出现巡逻切点抖动

所以如果你发现当前巡逻没有一次性发多点，这是设计使然，不是 bug。

---

## 6. 为什么现在不用旧的 `goal_pose / PID` 思路

当前主链已经是：

```text
行为树选路径
  -> SendNavThroughPoses
  -> Nav2
  -> MPPI controller
```

而不是：

```text
行为树发布单个 goal_pose
  -> 自研局部 PID 跟踪
```

这意味着：

1. 局部控制主力已经是 MPPI。
2. 原来的 `pb_omni_pid_pursuit_controller` 不再是主链依赖。
3. 当前行为树关注的是“何时发什么路径”，不是“如何逐周期做底盘 PID”。

如果后续有人还在旧文档里看到 `goal_pose`、`PID 局部控制器`、`局部跟踪节点` 等说法，要以当前代码为准。

---

## 7. 为什么要自己实现 `SendNavThroughPoses`

这是当前架构最关键的一个设计点。

实现文件：

- [`../src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp`](../src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp)

### 7.1 它不是普通的 BT ROS Action 节点

它继承的是：

- `BT::SyncActionNode`

不是：

- `BT::RosActionNode`
- `BT::StatefulActionNode`

含义是：

1. BT 节点本身不会长时间卡在 `RUNNING`。
2. 真正的 Nav2 action 在节点内部异步发送。
3. BT 生命周期和 ROS action 生命周期被主动解耦。

### 7.2 它解决了什么历史问题

历史上如果直接把导航 action 长时间挂在行为树里，容易遇到：

1. `halt()` 触发 cancel 的生命周期问题。
2. 高优先级分支抢占时 action 状态混乱。
3. 每个 tick 重复发同一路径。

而当前节点做了四件非常重要的事：

1. 同一路径不重发。
2. 只有路径真的变化时才主动 cancel 旧 goal。
3. 用 `goal_request_id_` 屏蔽旧回调串线。
4. 把 `goal_succeeded` 单独输出给行为树。

### 7.3 `IsPathGoalReached` 和它是怎么配合的

行为树并不是“等 action 自己结束后再继续”，而是每个 tick 做一次判断：

1. 当前 path 是否已经到达。
2. 如果没到，就让 `SendNavThroughPoses` 保持/发送这条路径。
3. 如果到了，就执行后续状态推进，例如巡逻游标更新。

这也是为什么 `IsPathGoalReached` 现在会优先参考：

- `goal_succeeded`

对应文件：

- [`../src/pb2025_sentry_behavior/plugins/condition/is_path_goal_reached.cpp`](../src/pb2025_sentry_behavior/plugins/condition/is_path_goal_reached.cpp)

---

## 8. 视觉接管分支是怎么工作的

视觉接管分支在当前主树里叫：

- `vision_override_realtime`

执行顺序是：

1. `IsVisionTargetValid`
2. `PublishGimbalAbsolute`
3. `SelectVisionFollowPath`
4. `IsPathGoalReached` 或 `SendNavThroughPoses`

可以理解成：

1. 先确认视觉目标可靠不可靠。
2. 可靠的话先发云台绝对角。
3. 再在目标周围选一个适合导航接近的跟随点。
4. 最后把这条跟随 path 交给 Nav2。

关键文件：

- [`../src/pb2025_sentry_behavior/plugins/condition/is_vision_target_valid.cpp`](../src/pb2025_sentry_behavior/plugins/condition/is_vision_target_valid.cpp)
- [`../src/pb2025_sentry_behavior/plugins/action/pub_gimbal_absolute.cpp`](../src/pb2025_sentry_behavior/plugins/action/pub_gimbal_absolute.cpp)
- [`../src/pb2025_sentry_behavior/plugins/action/select_vision_follow_path.cpp`](../src/pb2025_sentry_behavior/plugins/action/select_vision_follow_path.cpp)

更详细的视觉链路说明见：

- [`./融合.md`](./融合.md)

---

## 9. 参数从哪里来

### 9.1 loopback 行为树参数

- [`../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)

特点：

- `decision.input_source = simulation`
- 默认用于 `loopback_decision_sim.launch.py`

### 9.2 实机行为树参数

- [`../src/pb2025_sentry_behavior/params/sentry_behavior.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior.yaml)

特点：

- `decision.input_source = referee`
- 默认用于 `bringup.launch.py`

### 9.3 行为树服务端会声明和写入黑板的关键参数

入口：

- [`../src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp`](../src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp)

重点关注这些参数组：

- `decision.goal_points.*`
- `decision.point_roles.*`
- `decision.simulation.*`
- `decision.referee.start_gate.*`
- `decision.motion.*`
- `decision.vision.*`
- `decision.time_thresholds.*`
- `decision.hp_thresholds.*`
- `decision.decision_config.*`

这些参数决定了：

1. 点位坐标是什么。
2. 哪个点承担什么角色。
3. simulation/referee 分支怎么切。
4. 血量和时间阈值怎么判。
5. 视觉接管如何工作。
6. 巡逻切点、路径到达判定、action server 等细节怎么工作。

---

## 10. 推荐阅读顺序

如果你是第一次接手这套代码，建议按这个顺序看：

1. 先看 [`../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py`](../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)  
   目的：知道调试时到底起了哪些节点。
2. 再看 [`../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)  
   目的：知道行为树输入源、目标点和阈值是怎么配置的。
3. 再看 [`../src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp`](../src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp)  
   目的：知道黑板里到底有哪些键。
4. 再看 [`../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml`](../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml)  
   目的：把整体优先级关系读明白。
5. 最后按需看具体插件：
   - 巡逻：`select_patrol_path.cpp`、`advance_patrol_cursor.cpp`
   - 退防：`select_nearest_retreat_path.cpp`
   - 导航派单：`send_nav_through_poses.cpp`
   - 视觉跟随：`select_vision_follow_path.cpp`

---

## 11. 常用排查清单

### 11.1 树是否起的是当前主树

检查：

- `pb2025_sentry_behavior_client` 的 `target_tree` 是否为 `rmul_2026`

参数文件：

- [`../src/pb2025_sentry_behavior/params/sentry_behavior.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior.yaml)
- [`../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)

### 11.2 为什么分支没有切换

先看三件事：

1. `decision.input_source` 当前是 `simulation` 还是 `referee`
2. `decision/sim_mode` 当前是什么
3. 裁判系统消息或视觉消息是否真的进入黑板对应话题

### 11.3 为什么一直重发导航

先排查：

1. `decision_path` 是否每个 tick 都在变化
2. `goal_position_tolerance` 是否过小，导致路径被判成“不同路径”
3. 巡逻点是否配置过密，导致刚切点就被下一帧切回

### 11.4 为什么到点后不切巡逻点

先排查：

1. `IsPathGoalReached` 是否成功
2. `goal_succeeded` 是否被正确置位
3. `waypoint_stop_duration_s` 是否让 `AdvancePatrolCursor` 还在等待

### 11.5 为什么视觉分支不接管

先排查：

1. `vision/target` 是否真的在发
2. `tracking` 是否为 `true`
3. `nav_hold` 是否为 `true`
4. `timestamp` 是否新鲜
5. `target_position_map` 是否有效且 frame 可转换

更细的视觉排查见：

- [`./融合.md`](./融合.md)

---

## 12. 当前版本最重要的结论

如果你只记住一句话，请记这句：

> 当前哨兵主线已经是“行为树做优先级决策，Nav2 负责全局/局部规划，MPPI 负责局部控制”，行为树本身不再承担旧式 PID 局部控制职责。

因此后续优化的主战场通常是三处：

1. 行为树路径选择是否合理。
2. Nav2 / MPPI 参数是否合理。
3. 视觉输入、位姿、costmap 是否稳定。
