# RobotDecision `NavigateThroughPoses` 迁移清单

## 1. 迁移收益（相对 `NavigateToPose`）

- 多点连续执行：一次 action 可包含多个 waypoint，避免“到点 success 后再发下一点”的停顿窗口。
- 降低切点时延：减少决策节点与 BT Navigator 之间的 action 建连/结束抖动。
- 更贴合巡逻场景：巡逻本质是路径段执行，不是离散单点任务。

## 2. 与原方案最大区别

- 原方案：每次只发 1 个 `Pose`，到达后再发下一个点。
- 新方案：巡逻时发送“当前点 + 下一点”的 `NavigateThroughPoses` 段式目标；低血/危急血仍可单点发送（但也通过 ThroughPoses 接口，保持接口统一）。

## 3. 本项目实际改动点

- 决策节点 action 类型切换：
  - `src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/2025_robot_decision.hpp`
  - `src/standard_robot_pp_ros2/src/2025_robot_decision.cpp`
- 动作服务参数默认值切换为 `"/navigate_through_poses"` 且修复参数命名：
  - `src/standard_robot_pp_ros2/src/2025_robot_decision.cpp`
  - `src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml`
- 新增巡逻段逻辑：
  - `sendPatrolSegmentGoal(start_index, end_index)`
  - `computeNextPatrolIndex(...)`
  - `isSamePatrolSegmentGoal(...)`

## 4. 全项目兼容性检查结论

- Nav2 BT Navigator 已具备 ThroughPoses 支持：
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml`
  - 两处均包含：
    - `default_nav_through_poses_bt_xml`
    - `nav2_navigate_through_poses_action_bt_node`
- `pb2025_sentry_bringup` 的参数中同样具备 ThroughPoses 节点插件配置。

## 5. RViz 全局/局部路径可视化是否受影响

- 结论：不受影响，仍可看到全局与局部路径。
- 原因：`NavigateToPose` 与 `NavigateThroughPoses` 都走 planner/controller 管线，`planner_server` 与 `controller_server` 未改。
- 现有 RViz 配置已包含 `Path` / `local_plan` 显示：
  - `src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz`
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz`

## 6. 上线前验证清单

1. 编译：
   - `colcon build --packages-select standard_robot_pp_ros2`
2. 动作服务确认：
   - `ros2 action list | grep navigate_through_poses`
3. 节点参数确认：
   - `ros2 param get /robot_decision decision_config.nav2_action_server`
4. 巡逻实测：
   - 观察日志是否连续出现 `巡逻段目标已接受 [i -> j]` 与 `巡逻段导航完成 [i -> j]`
5. RViz 检查：
   - `Path`（全局）和 `local_plan`（局部）均持续刷新。

## 7. 故障回退点

- 仅回退 `decision_config.nav2_action_server` 到 `"/navigate_to_pose"` 不足以恢复旧行为，因为决策代码 action 类型已切换。
- 若需完整回退到旧单点方案，需同时回退 `2025_robot_decision.hpp/.cpp` 的 action 类型与巡逻发送逻辑。
