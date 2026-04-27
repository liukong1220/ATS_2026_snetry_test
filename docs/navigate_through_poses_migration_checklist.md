# `NavigateThroughPoses` 迁移说明

## 1. 这份文档现在的定位

这份文档不再表示“当前主线系统的实时状态”，而是用于说明两件事：

1. 当前主线为什么已经选择 `NavigateThroughPoses`
2. 仓库里保留的旧 `standard_robot_pp_ros2::robot_decision` 节点，和主线之间是什么关系

如果你要看当前真正在线运行的主线，请优先看：

- [`./sentry_bt_decision_checklist.md`](./sentry_bt_decision_checklist.md)
- [`./移植.md`](./移植.md)

---

## 2. 当前主线的真实情况

当前默认主线不是 `standard_robot_pp_ros2::robot_decision`，而是：

```text
pb2025_sentry_bringup/bringup.launch.py
  -> pb2025_sentry_behavior
  -> SendNavThroughPoses
  -> /navigate_through_poses
  -> Nav2 + MPPI
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
