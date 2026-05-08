# 全向导航、平滑与 ESDF 优化交接文档

更新时间：2026-05-08

本文档用于交接当前 `loopback_sim` 与实车导航链路的轨迹优化进度。下一轮对话可以直接从本文档继续，不需要再重新确认 ESDF 是否接通、MPPI 是否跟踪平滑路径、RViz 是否能观察到 fake ESDF。

## 1. 当前结论

当前系统已经从“能规划但跟踪不稳定”推进到“平滑路径进入 Nav2 主链，fake ESDF 可调用、可观测，loopback 中效果方向正确”的阶段。

阶段状态：

1. `SmacPlannerHybrid -> Nav2BSplineSmoother -> MPPI -> trajectory_speed_governor -> velocity_smoother` 主链已接通。
2. MPPI 当前跟随的是 `SmoothPath` 后输出给 controller 的路径，不再只是 RViz 中旁路青色线。
3. `trajectory_profile` 已成为正式接口，`trajectory_speed_governor` 已基于该 profile 对 controller 输出做二次限速。
4. fake ESDF 已经不是单纯 stub，当前基于 costmap 做 2D distance transform，并接入 optimizer 与 Nav2 smoother。
5. loopback 中 fake ESDF 已达到当前目标：RViz 可稳定看到贴墙/弯角处红色近障碍采样点，梯度箭头方向可解释，青色路径在贴边段比之前更早回拉。
6. 按当前观察，暂时不增强 fake ESDF 作用强度，避免在已可用状态下继续堆参数导致行为不可控。

当前需要保留的判断：

1. 现阶段主要问题已经不是“链路没接通”，而是“轨迹连续性、控制耦合、实车调试一致性”。
2. 后续不要优先继续把 B 样条磨圆；过度平滑会让弯角内切、终端段拉直、MPPI 跟踪变慢。
3. fake ESDF 的作用已经可见，下一步应先稳定接口和文档，再进入更高质量 ESDF-lite，而不是马上接复杂后端库。

## 2. 关键代码与配置索引

轨迹优化核心：

1. [bspline_path_optimizer.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/bspline_path_optimizer.hpp)
2. [bspline_path_optimizer.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/bspline_path_optimizer.cpp)
3. [nav2_bspline_smoother.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/nav2_bspline_smoother.hpp)
4. [nav2_bspline_smoother.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/nav2_bspline_smoother.cpp)
5. [trajectory_optimizer_node.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/trajectory_optimizer_node.cpp)
6. [trajectory_speed_governor.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/trajectory_speed_governor.cpp)

ESDF 接口与 fake provider：

1. [esdf_provider.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/esdf_provider.hpp)
2. [fake_costmap_esdf_provider.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/fake_costmap_esdf_provider.hpp)
3. [fake_costmap_esdf_provider.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/fake_costmap_esdf_provider.cpp)

loopback 配置：

1. [loopback_sim/params/nav2_params.yaml](../src/loopback_sim/params/nav2_params.yaml)
2. [loopback_navigation.launch.py](../src/pb2025_sentry_bringup/launch/loopback_navigation.launch.py)
3. [loopback_nav_only.launch.py](../src/pb2025_sentry_bringup/launch/loopback_nav_only.launch.py)
4. [loopback_decision_sim.launch.py](../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)
5. [loopback_vision_test.launch.py](../src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)

实车链路配置：

1. [node_params.yaml](../src/pb2025_sentry_bringup/params/node_params.yaml)
2. [reality/nav2_params.yaml](../src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)
3. [bringup.launch.py](../src/pb2025_sentry_bringup/launch/bringup.launch.py)
4. [navigation_launch.py](../src/pb2025_sentry_nav/pb2025_nav_bringup/launch/navigation_launch.py)

行为层与视觉跟随：

1. [select_vision_follow_path.cpp](../src/pb2025_sentry_behavior/plugins/action/select_vision_follow_path.cpp)
2. [sentry_behavior.yaml](../src/pb2025_sentry_behavior/params/sentry_behavior.yaml)
3. [sentry_behavior_loopback.yaml](../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)
4. [sentry_behavior_vision_test.yaml](../src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml)

RViz：

1. [loopback_nav2_view.rviz](../src/loopback_sim/rviz/loopback_nav2_view.rviz)
2. [sentry_default_view.rviz](../src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz)

参考文档：

1. [中科大哨兵2025技术报告.pdf](../sentry_doc/中科大哨兵2025技术报告.pdf)
2. [Batch-LIWO.pdf](../sentry_doc/Batch-LIWO.pdf)

## 3. loopback 链路状态

推荐用于纯导航观察的入口：

```bash
ros2 launch pb2025_sentry_bringup loopback_nav_only.launch.py use_rviz:=True
```

不打开 RViz 的 smoke test 入口：

```bash
ros2 launch pb2025_sentry_bringup loopback_nav_only.launch.py use_rviz:=False
```

带决策仿真的入口：

```bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

视觉链路测试入口：

```bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py use_rviz:=True
```

注意：

1. `loopback_vision_test.launch.py` 不是纯导航入口，它默认包含视觉目标发布、视觉 hold 和行为树测试逻辑。
2. 如果只想看路径规划、平滑、MPPI 和 ESDF，不要优先用 `loopback_vision_test.launch.py`。
3. `src/loopback_sim` 的 ROS 包名是 `nav2_loopback_sim`，不是 `loopback_sim`。
4. 现在包级入口可用：

```bash
ros2 launch nav2_loopback_sim tb3_loopback_simulation_launch.py use_rviz:=True
```

loopback 当前执行链：

1. `loopback_navigation.launch.py` 启动 Nav2 与轨迹优化旁路。
2. planner 使用 `SmacPlannerHybrid`。
3. BT 中路径链路为 `ComputePath -> SmoothPath(bspline_smoother) -> FollowPath(MPPI)`。
4. `Nav2BSplineSmoother` 发布正式 `/trajectory_profile`。
5. `trajectory_speed_governor` 订阅 `/trajectory_profile` 与 `cmd_vel_controller`，输出 `cmd_vel_controller_governed`。
6. `velocity_smoother` 输出 `cmd_vel_nav2_result`。
7. `loopback_simulator` 当前订阅 `cmd_vel_nav2_result`，仿真车体会真实运动。

loopback 已修复过的关键问题：

1. `loopback_simulator` 订阅旧 `cmd_vel` 导致 Nav2 有速度但仿真车不动。
2. 缺少 `base_footprint -> gimbal_yaw_fake` TF 导致 recovery / behavior 检查失败。
3. smoother 只看中心点 cost，但 Nav2 最终按 footprint 校验导致 `Smoothed path leads to a collision`。
4. smoother 碰撞后整条 raw fallback 过于粗暴，现已加入局部退化模式。
5. RViz 旧 ESDF 箭头残留，现已使用 `DELETEALL` 清理旧 marker。

## 4. 实车链路状态

实车默认入口：

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py
```

实车默认参数入口：

1. 主入口使用 [node_params.yaml](../src/pb2025_sentry_bringup/params/node_params.yaml)。
2. Nav bringup reality 备份参数同步在 [reality/nav2_params.yaml](../src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)。

实车当前主链：

1. `bringup.launch.py`
2. `rm_navigation_reality_launch.py`
3. `navigation_launch.py`
4. `trajectory_optimizer_node`
5. `planner_server.GridBased(SmacPlannerHybrid)`
6. `smoother_server.bspline_smoother`
7. `controller_server.FollowPath(MPPI)`
8. `trajectory_speed_governor`
9. `velocity_smoother`

当前实车链路已同步的内容：

1. `SmacPlannerHybrid` 终端直连参数已调低激进程度。
2. `smooth_path: False`，避免 Smac 自带 smoother 与 B 样条重复平滑。
3. `bspline_smoother` 与旁路 `trajectory_optimizer` 使用同一组保形参数。
4. `trajectory_speed_governor` 已接入 controller 与 velocity smoother 中间。
5. `BackUpFreeSpace` recovery 搜索半径与走廊参数已收紧。
6. 视觉跟随候选点筛选已同步 loopback 与实车行为参数。

当前实车链路没有开启的内容：

1. fake ESDF 当前只在 loopback `src/loopback_sim/params/nav2_params.yaml` 中开启。
2. `node_params.yaml` 与 `reality/nav2_params.yaml` 目前仍保持 costmap obstacle penalty 基线。
3. 这个取舍是为了避免在实车调试前同时引入 ESDF 作用强度、传感器噪声、定位误差三个变量。

实车调试建议：

1. 先验证 `SmacPlannerHybrid + bspline_smoother + MPPI + speed_governor` 的稳定性。
2. 再观察 `smoothed_path_visual` 和 `trajectory_profile_markers` 是否与 loopback 趋势一致。
3. 最后才考虑把 fake ESDF 分支迁入实车参数。

## 5. 当前参数基线

loopback 中 fake ESDF 已开启：

```yaml
trajectory_optimizer:
  use_esdf_obstacle_cost: true
  obstacle_safe_distance: 0.30

smoother_server:
  bspline_smoother:
    use_esdf_obstacle_cost: true
    obstacle_safe_distance: 0.30
```

实车当前仍未开启 fake ESDF：

```yaml
trajectory_optimizer:
  obstacle_safe_cost: 48
  obstacle_weight: 40.0
  obstacle_refinement_iterations: 3
  obstacle_refinement_gain: 0.02

smoother_server:
  bspline_smoother:
    obstacle_safe_cost: 48
    obstacle_weight: 40.0
    obstacle_refinement_iterations: 3
    obstacle_refinement_gain: 0.02
```

B 样条与速度 profile 当前方向：

1. `control_point_spacing: 0.20`
2. `output_path_spacing: 0.05`
3. `max_lateral_deviation: 0.08`
4. `curvature_limit: 1.60`
5. `curvature_weight: 10.0`
6. `curvature_refinement_iterations: 2`
7. `curvature_refinement_gain: 0.010`
8. `lateral_accel_limit: 0.75`
9. `longitudinal_accel_limit: 0.55`
10. `velocity_smoothing_gain: 0.18`

参数含义：

1. 当前不是追求极致平滑，而是优先保形、少内切、少贴障。
2. `max_lateral_deviation` 已压到 `0.08`，用于限制 B 样条离原始全局路径太远。
3. `control_point_spacing` 已收小到 `0.20`，用于减少转角被大步控制点抹成直线。
4. `curvature_refinement_gain` 已保守，避免曲率修正本身把线推进膨胀层。

SmacPlannerHybrid 当前方向：

1. loopback `tolerance: 0.20`
2. real `tolerance: 0.18`
3. loopback `analytic_expansion_ratio: 2.0`
4. real `analytic_expansion_ratio: 2.2`
5. loopback `analytic_expansion_max_length: 1.2`
6. real `analytic_expansion_max_length: 1.8`
7. loopback `minimum_turning_radius: 0.16`
8. real `minimum_turning_radius: 0.22`
9. `cost_travel_multiplier: 2.9`
10. `cost_penalty: 3.0`
11. `smooth_path: False`

实车 footprint / 安全边界方向：

1. footprint 仍为约 `0.60m x 0.60m` 的方形近似。
2. `footprint_padding: 0.04`
3. `inflation_radius: 0.50`
4. MPPI `collision_margin_distance: 0.24`
5. `xy_goal_tolerance: 0.20`

Recovery 当前同步参数：

1. `max_radius: 1.6`
2. `search_half_span_deg: 120.0`
3. `trajectory_sample_step: 0.06`
4. `near_sample_step: 0.04`
5. `far_sample_step: 0.08`
6. `layered_sampling_split_distance: 0.40`
7. `corridor_half_width: 0.18`
8. `corridor_lateral_step: 0.06`
9. `far_corridor_lateral_step: 0.10`
10. `minimum_release_distance: 0.14`

## 6. fake ESDF 当前实现

抽象接口：

```cpp
double getDistance(double x, double y);
Eigen::Vector2d getGradient(double x, double y);
```

设计约束：

1. 不绑定具体库。
2. 不绑定具体 ROS topic 或 msg。
3. 距离单位为米。
4. 梯度在世界坐标系下表达。
5. optimizer 内部会对梯度方向做归一化使用。

当前 fake provider：

1. 输入 `global_costmap/costmap_raw`。
2. 将高代价值栅格视为障碍。
3. 在二维 costmap 上做 distance transform。
4. `getDistance(x, y)` 返回到最近障碍的近似欧氏距离。
5. `getGradient(x, y)` 返回距离场中心差分梯度。

optimizer 当前分支逻辑：

1. 若 `use_esdf_obstacle_cost=true` 且 provider 可用，优先使用 `d(x)` 与 `grad d(x)`。
2. 若 ESDF 不可用，退回 costmap-cost obstacle penalty。
3. 当前实现保留 costmap fallback，因此 fake ESDF 不会破坏原主链。

当前 obstacle cost 目标形式：

```text
J_obstacle = sum(max(0, d_safe - d(x))^2)
```

当前观察结论：

1. fake ESDF 已经能达到阶段目标。
2. 红色近障碍点主要集中在弯角内侧或贴墙段。
3. 黄色梯度箭头方向基本可解释，用于判断路径应被推向哪侧。
4. 青色 `smoothed_path_visual` 在贴边段相较之前更早回拉。
5. 暂时不增强作用强度，避免把一个已可观察、可调试的基线打乱。

## 7. RViz 观察说明

当前轨迹规划相关可视化没有删除视觉目标、模式转换、瞄准和 tracker 层，只统一了路径规划相关显示风格。

重点观察 topic：

1. `/plan`
2. `/smoothed_path_visual`
3. `/trajectory_profile_visual`
4. `/trajectory_profile_markers`
5. `/trajectory_esdf_debug`
6. `/global_costmap/costmap`
7. `/local_costmap/costmap`

读取规则：

1. `/plan`：看 Smac 原始路径是否终端直线化、是否贴墙。
2. `/smoothed_path_visual`：看 B 样条与 obstacle refinement 后是否仍内切。
3. `/trajectory_profile_markers`：看速度 profile 是否在弯角处过度收缩。
4. `/trajectory_esdf_debug` 红/绿点：红点表示离障碍近，绿点表示离障碍远。
5. `/trajectory_esdf_debug` 黄色箭头：表示 `grad d(x)`，也就是局部远离障碍的方向。
6. costmap：确认红点是否确实对应膨胀层、墙角或局部高代价区域。

当前 RViz 已修复：

1. ESDF debug marker 每次发布前先 `DELETEALL`，旧箭头不会在路径更新后残留。
2. 梯度从无方向线段改为小尺寸箭头。
3. 颜色、透明度、线宽已统一成更克制的调试风格。
4. 不关闭、不删除非轨迹优化相关显示。

## 8. 已解决问题清单

导航执行：

1. 修复 Nav2 有路径但仿真车不运动的问题。
2. 修复 `cmd_vel_controller -> cmd_vel_controller_governed -> cmd_vel_nav2_result` 链路对接。
3. 修复 loopback fake TF 缺失导致 behavior / recovery 失败。

平滑与碰撞：

1. 关闭 Smac 自带 `smooth_path`，避免重复平滑。
2. `Nav2BSplineSmoother` 读取 obstacle 参数。
3. smoother 支持 footprint-aware collision / pullback。
4. smoother 支持局部退化为 raw polyline 段。
5. fallback 后重新评估最终路径 profile。

视觉跟随：

1. `SelectVisionFollowPath` 增加候选点 clearance。
2. 增加从当前位姿到候选点的 segment / corridor clearance。
3. 增加 planner-friendly 的目标点可达性兜底。
4. 行为层减少贴边目标和频繁重发目标。

ESDF：

1. `EsdfProvider` 抽象接口完成。
2. fake costmap ESDF provider 完成。
3. optimizer 与 smoother 都可调用 fake ESDF。
4. loopback 已验证 fake ESDF 可观测。

## 9. 仍需关注的问题

当前仍需实车和 loopback 继续观察的问题：

1. 某些终端段仍可能被 Smac analytic expansion 拉直。
2. B 样条过度平滑时仍可能把真实转角抹成近似直线。
3. MPPI 在弯前可能因为速度 profile、path alignment 和障碍 critic 耦合表现为转弯慢。
4. fake ESDF 当前来自二维 costmap，不是严格连续 signed distance field。
5. 实车定位、底盘延迟、轮速反馈和云台/底盘坐标链误差可能放大 loopback 中不明显的问题。

当前不要优先做的事：

1. 不要继续单纯提高 ESDF obstacle weight。
2. 不要继续无节制减小 `output_path_spacing`。
3. 不要同时修改 planner、smoother、MPPI 和 governor 多组参数。
4. 不要在实车上直接开启 fake ESDF 后再同时调 MPPI，否则变量过多。

## 10. 参考技术报告后的优化路线

### 10.1 从 `中科大哨兵2025技术报告.pdf` 得到的直接启发

报告中导航部分的核心经验是：

1. 轨迹优化应直接消费距离与梯度，而不只是离散 occupancy / inflation cost。
2. ESDF 相比安全走廊的优势在于持续提供障碍距离与梯度，狭窄区域中更容易做连续优化。
3. 狭窄地形的主要风险不是“有没有梯度”，而是梯度震荡、梯度无效化和优化后动力学不可用。
4. 两阶段优化有价值：先得到可行形状，再做更细的动力学与障碍联合优化。
5. 控制器应充分信任上游轨迹，因此上游轨迹必须连续、无碰撞，并含有可用的速度/加速度信息。

映射到当前项目：

1. 当前 B 样条 + profile 已经承担了“轨迹表示 + 时间参数化”的角色。
2. fake ESDF 已经承担了“距离与梯度观测”的入口角色。
3. 下一步最重要的是让 fake ESDF 更连续，而不是马上换库。
4. 后续 optimizer 应逐步从单点 obstacle push，演进到更稳定的二阶段 refinement。

### 10.2 从 `Batch-LIWO.pdf` 得到的实车链路启发

Batch-LIWO 主要不是轨迹优化文档，但对实车调试有直接意义：

1. 稳定高带宽里程计是后端规划与控制可信的前提。
2. 轮速协方差和退化检测会影响控制器看到的状态质量。
3. 实车上如果出现路径正确但运动抽动，不能只看 planner/smoother，也要看定位延迟、速度反馈、轮速退化和 TF 时间。
4. loopback 的路径优化结论上车前必须经过状态估计链路检查。

实车排查顺序建议：

1. 先确认 TF、odom、base_link、base_footprint 时间戳稳定。
2. 再确认 Nav2 输出速度与底盘实际响应一致。
3. 再看 `trajectory_profile` 是否与实际转弯能力匹配。
4. 最后再讨论 ESDF 强度或 MPPI critic 权重。

## 11. 下一阶段建议

建议按以下顺序推进，避免重新引入过多变量。

第一优先级：文档与基线冻结

1. 保留当前 loopback fake ESDF 参数，不增强作用强度。
2. 以 `loopback_nav_only.launch.py` 作为纯导航观察入口。
3. 以 `bringup.launch.py + node_params.yaml` 作为实车链路唯一主入口。
4. 新对话开始后先读本文档，不重新翻旧日志。

第二优先级：实车链路一致性检查

1. 检查实车 `smoothed_path_visual` 与 controller 实际跟随路径是否一致。
2. 检查 `trajectory_profile_markers` 是否跟随 `/trajectory_profile` 正常刷新。
3. 检查实车终端转角是否仍被拉直。
4. 若实车比 loopback 抖，优先排查 odom / TF / 下位机响应延迟。

第三优先级：trajectory-grade fake ESDF-lite

1. `已完成` 给 fake ESDF provider 增加 bilinear interpolation。
2. `已完成` 对 `getGradient(x, y)` 做平滑距离场上的插值梯度，减少中心差分跳变。
3. `已完成` 增加 `d_min`、`d_avg`、平均梯度、危险采样点数量等统计输出。
4. `已完成` 保持 `EsdfProvider` 抽象不变，只替换 provider 内部质量。

第四优先级：优化器二阶段化

1. `已完成第一版` 第一阶段继续做保形、清碰撞和基础曲率限制。
2. `已完成第一版` 第二阶段补充 ESDF 导向 refinement，当前先实现“切向抑制 + 法向推离 + 轻量 shape pullback”。
3. `已完成第一版` 对沿轨迹切向的梯度做抑制，优先使用法向推离障碍，参考技术报告中对狭窄区域优化的处理思路。
4. `当前状态` 还没有把 velocity objective 显式并入第二阶段 cost，目前重点仍是避免 obstacle refinement 把控制点沿路径方向推开，导致速度 profile 异常。

第五优先级：真实 ESDF 后端

1. 在 fake ESDF-lite 调顺后再考虑真实后端。
2. 可选方向包括自建 2D ESDF、ROG-Map 输出适配、FIESTA / voxel ESDF 等。
3. 真实后端接入时仍只实现 `getDistance(x, y)` 和 `getGradient(x, y)`，不要让 optimizer 绑定具体库。
4. 先做只读 provider，再考虑增量更新、线程缓存和动态障碍。

## 12. 新对话建议提示词

下一轮如果继续推进，可以直接给模型以下上下文：

```text
请先阅读 docs/omni_recovery_smoothing_optimization.md。
当前 fake ESDF 在 loopback 已达到阶段目标，先不要增强 obstacle 强度。
实车主参数是 src/pb2025_sentry_bringup/params/node_params.yaml。
目前 loopback 与实车主链一致性、fake ESDF-lite 第一版、二阶段 optimizer refinement 第一版都已经完成。
请下一步优先做：
1. loopback 运行时观察是否稳定
2. 实车链路一致性验证
3. 结合 RViz 观察 profile / ESDF / controller 耦合
4. 再决定是否继续细化二阶段 velocity objective
```

当前工程判断：

1. `ESDF interface`: 已完成。
2. `fake ESDF provider`: 已完成并可观测。
3. `loopback fake ESDF`: 当前效果达到阶段目标，暂不加权。
4. `real chain ESDF`: 暂未开启，应先做实车主链稳定性验证。
5. `fake ESDF-lite`: 已完成第一轮连续性升级，包括 bilinear distance、平滑梯度、debug statistics。
6. `optimizer refinement`: 已完成第一轮二阶段化，当前版本重点是抑制沿轨迹切向的 ESDF 推动。
7. `RViz debug`: 已修正 ESDF 统计文本与 `trajectory_profile_markers` 的尾部文字重叠，改为沿终点局部法向偏移显示。
8. `next core`: loopback 运行时观察、实车一致性、控制耦合。
