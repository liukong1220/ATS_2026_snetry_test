# `NavigateThroughPoses` 迁移说明

## 1. 这份文档现在的定位

这份文档不再表示“当前主线系统的实时状态”，而是用于说明两件事：

1. 当前主线为什么已经选择 `NavigateThroughPoses`
2. 仓库里保留的旧 `standard_robot_pp_ros2::robot_decision` 节点，和主线之间是什么关系

如果你要看当前真正在线运行的主线，请优先看：

- [`./总览.md`](./总览.md)
- [`./sentry_bt_decision_checklist.md`](./sentry_bt_decision_checklist.md)
- [`./omni_recovery_smoothing_optimization.md`](./omni_recovery_smoothing_optimization.md)

---

## 2. 当前主线的真实情况

当前默认主线不是 `standard_robot_pp_ros2::robot_decision`，而是：

```text
pb2025_sentry_bringup/bringup.launch.py
  -> pb2025_sentry_behavior
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

关键事实：

1. 默认总入口是 [`../src/pb2025_sentry_bringup/launch/bringup.launch.py`](../src/pb2025_sentry_bringup/launch/bringup.launch.py)
2. 它调用 `standard_robot_pp_ros2.launch.py` 时显式传入：
   - `launch_robot_decision := False`
3. 当前主决策树在 [`../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml`](../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml)
4. 当前真正负责给 Nav2 发 action 的是：
   - [`../src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp`](../src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp)

所以如果你在排查当前哨兵主线，请不要先去改 `standard_robot_pp_ros2::robot_decision`。

---

## 3. 为什么主线选择 `NavigateThroughPoses`

相对旧的单点 `NavigateToPose` 思路，主线选择 `NavigateThroughPoses` 的原因是：

1. 行为树可以统一输出 `nav_msgs/Path`
2. 固定点、退防点、巡逻点、视觉跟随点都能走同一接口
3. 行为树可以继续保留“是否重发、何时 cancel、何时切分支”的控制权
4. Nav2 仍然复用同一条 planner/controller/MPPI 执行链

但需要强调的是：

> 当前主线虽然走 `NavigateThroughPoses` 接口，巡逻并不等于一次性预发整串 waypoint。

当前巡逻策略是：

1. 行为树每次只选当前巡逻目标点
2. `SelectPatrolPath` 实际生成单点 path
3. 到点后由 `AdvancePatrolCursor` 推进游标

这和早期“当前点 + 下一点”的段式预发送思路已经不一样了。

---
### 4.1 现在为什么不会和行为树 `halt()` 冲突

这次方案里，`NavigateThroughPoses` 能稳定工作，还有一个很重要的原因：

- 我们不是直接用 `BehaviorTree.ROS2` 的 `RosActionNode`
- 而是自己实现了一个自定义节点 `SendNavThroughPoses`

对应实现位置：

- `src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.hpp`
- `src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp`

这样做的核心目的，是绕开 `RosActionNode` 在 `cancel during halt()` 这条链路上的已知问题。

你可以把当前方案理解成：

- 行为树负责“决定什么时候应该发一条路径”
- `SendNavThroughPoses` 负责“把这条路径异步送给 Nav2，并自己维护 goal 状态”
- 行为树本身不去长期持有一个 `RUNNING` 的 action 节点等待它结束

当前规避冲突的设计有 4 个关键点。

#### 4.1.1 `SendNavThroughPoses` 是 `SyncActionNode`

`SendNavThroughPosesAction` 继承的是：

- `BT::SyncActionNode`

而不是：

- `BT::RosActionNode`
- `BT::StatefulActionNode`

这意味着它在一次 `tick()` 里就会返回，不会长时间停留在 `RUNNING` 状态等待 Nav2 action 完成。

直接结果是：

- 上层行为树通常不需要对它执行 `halt()`
- 自然也就不会落入“halt 时顺带 cancel action”那条容易冲突的路径

#### 4.1.2 真正的 Nav2 action 由节点内部异步管理

在 `tick()` 里，这个节点会自己创建并使用：

- `rclcpp_action::Client<NavigateThroughPoses>`

然后调用：

- `async_send_goal(...)`

也就是说：

- BT 节点同步返回
- Nav2 action 在节点内部异步继续跑
- feedback 和 result 通过回调更新内部状态

这相当于把“BT 生命周期”和“ROS action 生命周期”解耦了。

#### 4.1.3 同一路径不重发，新路径才手动 cancel

当前实现不会每个 tick 都重发目标。

它先比较当前输入路径和正在执行的 `active_path_` 是否等价：

- 如果是同一路径，而且目标还在执行，就直接返回成功，不重发
- 如果是同一路径，而且已经成功完成，就直接设置 `goal_succeeded`
- 只有路径真的变了，才会先 `cancelCurrentGoal()` 再发新 goal

这点非常关键，因为它避免了两类常见问题：

- 行为树高频 tick 导致的重复发目标
- 每次重 tick 都 cancel 一次，最终把 Nav2 和 BT 状态搅乱

所以现在的取消逻辑不是由 `halt()` 触发的，而是由“路径切换”主动触发的。

#### 4.1.4 用 `goal_request_id_` 屏蔽旧回调串线

当前实现里还有一个小但很重要的保护：

- 每次发新 goal 都会递增 `goal_request_id_`
- feedback / goal response / result 回调都会先检查 request id

这样做的效果是：

- 旧 goal 即使晚到反馈，也不会污染新 goal 的状态
- 取消旧目标后，不容易发生“旧回调把当前状态写乱”的问题

这对 Reactive 行为树场景尤其重要，因为分支切换时目标替换会比较频繁。

### 4.2 从行为树视角看它的执行方式

当前 `rmul_2026.xml` 里，对每种决策路径基本都用了同一个模式：

1. 先生成 `decision_path`
2. 先用 `IsPathGoalReached` 判断当前路径是不是已经完成
3. 只有未完成时，才执行 `SendNavThroughPoses`

也就是说行为树不是“卡在导航 action 上等它返回”，而是每次 tick 都做一次：

- 这条路径还要不要继续
- 如果要继续，当前 goal 是否已经在跑
- 如果已经在跑且路径没变，就什么都不做

因此整个模式更像：

- “路径命令派发器”

而不是：

- “BT 内部阻塞等待的 action 节点”

这也是它和 `halt()` 不容易冲突的根本原因。

### 4.3 一句话总结

当前项目避免 `NavigateThroughPoses` 与行为树 `halt()` 冲突的方法，不是去修 `halt()`，而是从架构上绕开它：

- 用自定义 `SyncActionNode`
- 在节点内部异步维护 Nav2 action
- 同路径不重发
- 仅在路径切换时主动 cancel
- 用 request id 防止旧回调串线

所以你现在这套方案的本质是：

- `NavigateThroughPoses` 仍然是真执行接口
- 但它不再受 `BehaviorTree.ROS2` 默认 action/halt 生命周期的直接约束

## 4. 仓库里保留的旧 `robot_decision` 节点是什么状态

旧节点代码仍在仓库里：

- [`../src/standard_robot_pp_ros2/src/2025_robot_decision.cpp`](../src/standard_robot_pp_ros2/src/2025_robot_decision.cpp)
- [`../src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml`](../src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml)

但它当前有三个重要事实：

1. 默认不参与总入口运行
2. 代码里当前 action client 仍然是 `NavigateToPose`
3. 参数默认值当前仍然是：
   - `decision_config.nav2_action_server = "/navigate_to_pose"`

因此：

- 它是保留在仓库中的 legacy 节点
- 它不是当前主线 ThroughPoses 决策链的一部分

---

## 5. 当前哪些地方确实具备 `NavigateThroughPoses` 能力

### 5.1 行为树主线

已经具备，并且正在使用：

- [`../src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp`](../src/pb2025_sentry_behavior/plugins/action/send_nav_through_poses.cpp)
- [`../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml`](../src/pb2025_sentry_behavior/behavior_trees/rmul_2026.xml)
- [`../src/pb2025_sentry_behavior/behavior_trees/vision_test.xml`](../src/pb2025_sentry_behavior/behavior_trees/vision_test.xml)

### 5.2 Nav2 配置侧

当前 Nav2 配置已经包含 ThroughPoses 相关能力，主线可以直接调用：

- [`../src/loopback_sim/params/nav2_params.yaml`](../src/loopback_sim/params/nav2_params.yaml)
- [`../src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`](../src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)
- [`../src/pb2025_sentry_bringup/params/node_params.yaml`](../src/pb2025_sentry_bringup/params/node_params.yaml)

### 5.3 RViz 观察

无论是 `NavigateToPose` 还是 `NavigateThroughPoses`，只要走的是同一条 Nav2 planner/controller 链：

1. 全局路径可视化仍然可看
2. 局部轨迹 / trajectories 仍然可看
3. 问题重点不在 RViz 能不能显示，而在“谁在发 goal、goal 是否稳定、局部控制是否合理”

---

## 6. 如果你要维护 legacy `robot_decision`，应该怎么理解这份文档

这时它可以被当作“迁移待办清单”，而不是“已完成事实”。

也就是说，如果后续你真的想把旧 `standard_robot_pp_ros2::robot_decision` 继续迁到 ThroughPoses，需要至少完成下面这些工作：

1. 把 action client 从 `NavigateToPose` 改成 `NavigateThroughPoses`
2. 把参数默认值从 `"/navigate_to_pose"` 改成 `"/navigate_through_poses"`
3. 重新定义旧节点中的巡逻发送逻辑
4. 明确它与当前行为树主线谁是主入口，避免双重决策

但在当前项目里，更推荐的方向不是继续强化这个 legacy 节点，而是直接维护行为树主线。

---

## 7. 当前最推荐的判断方法

如果你想确认自己现在调的是哪条链，可以直接看下面两个点：

### 7.1 看总入口是否启用了旧节点

当前默认总入口里：

- `launch_robot_decision := False`

这表示：

- 旧 `robot_decision` 节点默认不开

### 7.2 看日志里是谁在发导航 goal

如果你看到类似：

- `Send NavigateThroughPoses goal with ...`

那基本说明你正在走当前行为树主线。

如果你看到的是旧 `RobotDecisionNode` 自己的决策日志，那说明你是单独启用了 legacy 节点。

---

## 8. 一句话结论

这份文档现在最重要的结论是：

> `NavigateThroughPoses` 在当前项目里已经是行为树主线正在使用的接口，但仓库中保留的 `standard_robot_pp_ros2::robot_decision` 仍是 legacy 节点，默认不参与主线运行，也不应再被误认为当前主决策入口。
