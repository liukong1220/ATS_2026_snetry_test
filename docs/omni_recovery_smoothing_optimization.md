# 全向导航优化接力文档

更新时间：2026-05-08

## 1. 当前阶段状态

### 第一阶段：B 样条平滑 + 高密度路径点

状态：已完成

已落地：

1. 独立 B 样条风格路径优化算法
2. 高密度路径重采样
3. 可视化旁路输出 `smoothed_path_visual`

### 第二阶段：把平滑后的参考路径真正接入 MPPI

状态：已完成第一版，并已重新修复接入 loopback

已落地：

1. `trajectory_optimizer` 已封装成 Nav2 smoother plugin
2. BT 主链已恢复为 `ComputePath -> SmoothPath -> FollowPath`
3. loopback 当前已真正跟踪平滑后的 path
4. `trajectory_profile` 已作为正式接口发布
5. `trajectory_speed_governor` 已基于 profile 对 controller 输出做二次限速
6. 实车 `reality` 参数和 `navigation_launch.py` 已同步到同一套完整链

### 第三阶段：ESDF obstacle cost

状态：未开始

## 2. 当前进度总览

当前进度可以直接概括成三句话：

1. 第一阶段 B 样条平滑已经落地，并且已经能在 RViz 里看到青色 `smoothed_path_visual`
2. 第二阶段已经接入真实导航链，`trajectory_profile` 和 `trajectory_speed_governor` 已经反哺 controller
3. 第三阶段 ESDF 还没开始，现在的重点是先把“贴线但抖、转弯偏慢、折线残余”压稳

当前状态标签：

1. `trajectory_optimizer` 第一阶段和第二阶段第一版：已完成
2. `SmacPlannerHybrid + bspline_smoother + MPPI` 链路：已接通
3. `smooth_path` 与 B 样条重复平滑：已解决
4. `Nav2BSplineSmoother` 的 `J_obs` 接口读取：已解决
5. `trajectory_profile` 正式发布与 governor 反哺：已解决
6. costmap 初期未到导致旁路节点崩溃：已解决
7. 当前弯角仍有轻微折线感和抖动：待继续优化
8. 当前转弯速度偏慢但可接受：待继续优化

## 3. 当前核心结论

“第二个弯进入膨胀层”不单单是 MPPI 的问题。

当前排查结论是：

1. planner、smoother、controller 三层都会共同影响
2. loopback 原来使用 `NavfnPlanner` 时，第一层路径更容易贴边
3. 纯几何 B 样条会在角点进一步往内抹
4. MPPI 会在第二个弯继续沿 shortcut 倾向切弯

所以这不是单层问题，而是：

1. planner 可能先贴边
2. smoother 可能再抹角
3. controller 最后把它放大

## 4. 当前 `trajectory_optimizer` 能力

### 4.1 规范化三次 B 样条表示

现在已经有：

- `CubicBSpline2D`
- `getPoint(s)`
- `getFirstDerivative(s)`
- `getSecondDerivative(s)`
- `getCurvature(s)`

位置：

- [bspline_path_optimizer.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/bspline_path_optimizer.hpp)
- [bspline_path_optimizer.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/bspline_path_optimizer.cpp)

### 4.2 曲率约束

当前已加：

1. `curvature_limit`
2. `curvature_weight`
3. `curvature_refinement_iterations`
4. `curvature_refinement_gain`

目前做法是：

1. 先生成 dense B 样条 path
2. 计算离散一阶/二阶导
3. 用曲率公式计算 `kappa`
4. 对超出 `kappa_max` 的点做 refinement

这已经是曲率约束雏形。

### 4.3 时间参数化

当前已加：

1. 曲率限速
2. forward/backward acceleration limiting
3. velocity smoothing
4. `TrajectoryProfile2D`

profile 每个采样点包含：

1. `s`
2. `t`
3. `point`
4. `first_derivative`
5. `second_derivative`
6. `curvature`
7. `speed_limit`
8. `speed`
9. `acceleration`

## 5. `trajectory_profile` 已成为正式接口

当前已定义消息：

- [TrajectoryProfileMsg.msg](../src/pb2025_sentry_nav/sp_msgs/msg/TrajectoryProfileMsg.msg)
- [TrajectoryProfilePoint.msg](../src/pb2025_sentry_nav/sp_msgs/msg/TrajectoryProfilePoint.msg)

当前存在两条 profile 输出链：

1. `trajectory_optimizer_node -> /trajectory_profile_visual`
2. `nav2_bspline_smoother -> /trajectory_profile`

这意味着：

1. 在 BT / smoother server 外已经有正式 profile 接口
2. 后续 ESDF 接入时不需要再重新定义一套轨迹结构

## 6. 时间参数化已经开始反哺 controller

现在 loopback 中存在：

- `trajectory_speed_governor`

它会：

1. 订阅 `/trajectory_profile`
2. 订阅 `cmd_vel_controller`
3. 输出 `cmd_vel_controller_governed`

然后再由：

- `velocity_smoother`

继续处理并输出 `cmd_vel_nav2_result`

所以“时间参数化反哺 controller”这件事在 loopback 里已经不是内部 profile 变量，而是已经进入执行链。

## 7. 本轮重新修复的内容

### 7.1 修复了回退后 `trajectory_optimizer` 的编译断点

这次误回退后，主要断点是：

1. `trajectory_optimizer` 源码残留了未使用函数
2. 在 `-Werror` 下直接导致构建失败

目前：

- `sp_msgs`
- `trajectory_optimizer`

都已经重新编译通过。

### 7.2 loopback 链已重新接通

当前 loopback 里已经存在：

1. `/smoothed_path_visual`
2. `/trajectory_profile_visual`
3. `/trajectory_profile`
4. `/cmd_vel_controller_governed`

说明：

1. path 侧可视化正常
2. profile 接口正常
3. speed governor 链路正常

### 7.3 loopback planner 已升级为 `SmacPlannerHybrid`

当前 loopback 已从：

- `NavfnPlanner`

升级为：

- `SmacPlannerHybrid`

这样与实车链更接近，也更适合分析“为什么第二个弯 still 切膨胀层”。

### 7.4 实车链已同步到完整版本

当前已同步到实车链的内容：

1. `reality/nav2_params.yaml` 的 `smoother_server` 已切到 `bspline_smoother`
2. 曲率 / 速度 / 障碍联合优化参数已同步到实车 `trajectory_optimizer`
3. `trajectory_speed_governor` 已接入实车 `navigation_launch.py`
4. `velocity_smoother` 已改为吃 `cmd_vel_controller_governed`
5. 实车旁路可视化仍保留：
   - `smoothed_path_visual`
   - `trajectory_profile_visual`

这意味着“最新这套曲线 + 规划 + 速度约束”已经不再只停留在 loopback，而是已经完整接入实车链的配置和 launch 层。

### 7.5 `bringup.launch.py` 已是最终实车入口

当前最终实车入口是：

- [bringup.launch.py](../src/pb2025_sentry_bringup/launch/bringup.launch.py)

它会继续包含：

1. `rm_navigation_reality_launch.py`
2. `navigation_launch.py`
3. 当前这套：
   - `trajectory_optimizer`
   - `bspline_smoother`
   - `trajectory_speed_governor`
   - `velocity_smoother`

所以后续所有“上车前检查”和“RViz 观察”都应以这条入口为准，而不是以 loopback 或单独 nav bringup 为准。

## 8. 已解决诊断

以下问题已经确认完成并解决：

1. [已解决] `SmacPlannerHybrid` 自带 `smooth_path` 与 B 样条重复平滑，已关闭 `smooth_path: False`
2. [已解决] `Nav2BSplineSmoother` 之前没有读取 `obstacle_*` 参数，现已接入 `J_obs`
3. [已解决] `trajectory_profile` 之前未正确激活发布，现已在 smoother `activate()` 中激活
4. [已解决] costmap 未就绪时 `trajectory_optimizer_node` 会崩溃，现已改为 fallback 到 geometry-only
5. [已解决] smoother 做 costmap 回拉后没有重算 profile，现已重新评估最终路径 profile
6. [已解决] 实车和 loopback 参数不同步，现已同步到同一组保形、离障、限速基线
7. [已解决] `trajectory_speed_governor` 未稳定反哺 controller 的链路问题，现已接入正式 profile

## 9. 2026-05-07 新一轮排查结论

当前现象已经从“第二个弯局部问题”扩大为“任何弯角进入前都可能不贴曲线、MPPI 轨迹挤在一起、然后切进膨胀层”。

这轮排查后的判断是：

1. 不是地图显示错误，`/plan` 和地图本身大体正常
2. 新增 B 样条后，第一层参考路径存在“几何内切”风险
3. SmacPlannerHybrid 自带 `smooth_path` 已关闭，重复平滑链这次已经解决
4. 当前仍能看到“贴线但抖、转弯偏慢”，更像是折线路径残余 + MPPI 弯角策略的耦合，而不是 Smac 自带平滑还在偷偷二次内切
5. `Nav2BSplineSmoother` 之前没有读取 `obstacle_*` 参数，导致 YAML 里的 `J_obs` 调参没有真正进入主链
6. `trajectory_profile` 之前在 smoother 里未激活 publisher，speed governor 可能拿不到正式 profile
7. smoother 做 costmap 回拉后没有重算 profile，governor 用到的可能是回拉前速度/曲率
8. MPPI 实车参数前视过远、采样扰动过大、障碍 critic 偏弱，会放大弯角 shortcut

所以本轮不是继续把曲线磨圆，而是改成：

1. 第一层更保形
2. 规划更远离高 cost
3. MPPI 缩短前视、增强贴线和避障
4. 速度治理降低过度刹车，避免弯前控制量挤成一团

## 10. 本轮代码修复

已修复位置：

1. [bspline_path_optimizer.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/bspline_path_optimizer.hpp)
2. [bspline_path_optimizer.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/bspline_path_optimizer.cpp)
3. [nav2_bspline_smoother.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/nav2_bspline_smoother.cpp)
4. [trajectory_optimizer_node.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/trajectory_optimizer_node.cpp)

具体变化：

1. `Nav2BSplineSmoother` 现在读取 `obstacle_safe_cost / obstacle_weight / obstacle_refinement_*`
2. smoother 在 costmap 回拉后会重新计算路径姿态和 `TrajectoryProfile2D`
3. smoother 的 `trajectory_profile` lifecycle publisher 已在 `activate()` 里激活
4. `trajectory_optimizer_node` 旁路可视化也支持订阅 `global_costmap/costmap_raw`
5. costmap 暂时不可用时不会崩溃，会退回 geometry-only smoothing 并打印节流 warning
6. obstacle penalty 已改成归一化平方，避免高 cost 格子导致 refinement 步长突然过大
7. 新增 `BSplinePathOptimizer::evaluateProfile()`，用于对最终路径重新评估 profile

代码优化索引：

1. `bspline_path_optimizer.*`：负责三次 B 样条、高密度采样、曲率计算、速度 profile、`J_curvature / J_velocity / J_obs` 统一 refinement
2. `nav2_bspline_smoother.*`：负责把优化器接入 Nav2 `SmoothPath` 主链，并发布正式 `/trajectory_profile`
3. `trajectory_optimizer_node.*`：负责 RViz 旁路可视化，发布 `/smoothed_path_visual` 和 `/trajectory_profile_visual`
4. `trajectory_speed_governor.*`：负责读取 `/trajectory_profile`，对 `cmd_vel_controller` 做曲率相关限速
5. `loopback_sim/params/nav2_params.yaml`：loopback 当前调参基线
6. `pb2025_nav_bringup/config/reality/nav2_params.yaml`：Nav bringup reality 备份参数
7. `pb2025_sentry_bringup/params/node_params.yaml`：最终实车入口 `bringup.launch.py` 默认使用的参数

## 11. 当前新参数基线

loopback、`reality/nav2_params.yaml`、实车入口 `pb2025_sentry_bringup/params/node_params.yaml` 已同步到同一方向。

### `trajectory_optimizer` / `bspline_smoother`

1. `control_point_spacing: 0.24`
2. `output_path_spacing: 0.05`
3. `max_lateral_deviation: 0.12`
4. `curvature_limit: 1.60`
5. `curvature_weight: 10.0`
6. `curvature_refinement_iterations: 2`
7. `curvature_refinement_gain: 0.015`
8. `global_speed_limit: 1.15` real, `1.20` loopback
9. `lateral_accel_limit: 0.75`
10. `longitudinal_accel_limit: 0.55`
11. `velocity_smoothing_gain: 0.18`
12. `obstacle_safe_cost: 48`
13. `obstacle_weight: 55.0`
14. `obstacle_refinement_iterations: 5`
15. `max_path_cost: 48`
16. `pullback_samples: 12`

### Planner

1. `SmacPlannerHybrid` 保持启用
2. `cost_travel_multiplier: 2.9`
3. `cost_penalty: 3.0`
4. `smooth_path: False`

关闭 Smac 自带平滑的原因：避免 Smac smoother 和 B 样条在弯角重复内切。

### MPPI

1. `batch_size: 1400` loopback, `1000` real
2. `temperature: 0.22`
3. `gamma: 0.035`
4. `vx_std / vy_std / wz_std: 0.09 / 0.09 / 0.16` loopback
5. `vx_std / vy_std / wz_std: 0.12 / 0.12 / 0.22` real
6. `PathAlignCritic.cost_weight: 12.0`
7. `PathAlignCritic.offset_from_furthest: 6`
8. `PathFollowCritic.cost_weight: 3.2`
9. `PathFollowCritic.offset_from_furthest: 1`
10. `PathAngleCritic.cost_weight: 6.0`
11. `PathAngleCritic.max_angle_to_furthest: 0.55`
12. `ObstaclesCritic.repulsion_weight: 7.0`
13. `ObstaclesCritic.critical_weight: 90.0`
14. `ObstaclesCritic.collision_margin_distance: 0.20`

### Speed Chain

1. `trajectory_speed_governor.min_speed_scale: 0.40`
2. `trajectory_speed_governor.curvature_brake_gain: 0.60`
3. real `velocity_smoother.max_velocity: [2.0, 2.0, 3.0]`
4. real `velocity_smoother.max_accel: [1.4, 1.4, 2.5]`

这组参数的目标不是追求最快，而是先把“弯前抽动 + 切膨胀层”压下去。

## 12. 当前实车链状态

最终实车入口仍是：

- [bringup.launch.py](../src/pb2025_sentry_bringup/launch/bringup.launch.py)

实际链路：

1. `bringup.launch.py`
2. `rm_navigation_reality_launch.py`
3. `navigation_launch.py`
4. `trajectory_optimizer_node`
5. `planner_server`
6. `smoother_server.bspline_smoother`
7. `controller_server.FollowPath`
8. `trajectory_speed_governor`
9. `velocity_smoother`

参数入口：

1. 实车默认使用 [node_params.yaml](../src/pb2025_sentry_bringup/params/node_params.yaml)
2. Nav bringup reality 备份同步在 [reality/nav2_params.yaml](../src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)
3. loopback 使用 [loopback nav2_params.yaml](../src/loopback_sim/params/nav2_params.yaml)

## 13. 验证记录

本轮已验证：

1. `colcon build --packages-select trajectory_optimizer` 通过
2. `loopback_vision_test.launch.py use_rviz:=False publish_referee_inputs:=True` 35 秒 smoke test 通过
3. 启动日志确认：
   - `Trajectory optimizer active: plan -> smoothed_path_visual`
   - `Configured Nav2BSplineSmoother plugin: bspline_smoother`
   - `Configured MPPI Controller: FollowPath`
   - `SmacPlannerHybrid` 正常加载
4. 旁路可视化在 costmap 未到时会打印 warning，但不会崩溃

仍需 RViz 人工确认：

1. 青色 `smoothed_path_visual` 是否比之前少切弯
2. `trajectory_profile_markers` 是否随主链 `trajectory_profile` 更新
3. MPPI rollout 在转弯前是否还挤成一团
4. 机器人 footprint 是否仍进入 inflation layer

## 14. 当前 loopback 观测结果

你这轮测试后的实际现象可以总结成三句：

1. 已经能贴线，说明第一层路径和 MPPI 跟随链比之前更对齐
2. 仍然有抖动，而且抖动里还能看到折线感，说明参考路径还不是完全“连续顺滑到 controller 级别”
3. 转弯速度偏慢，但仍在可接受范围，说明现在更像是安全性偏保守，而不是继续激进切弯

这意味着当前问题已经不是“完全跟不上曲线”，而是：

1. 路径几何还有折线残余
2. MPPI 在弯角还会做保守补偿
3. 速度治理让弯前更稳，但也让转弯没有那么利落

所以下一轮优化的方向应当是：

1. 先把曲线进一步去折线，但不要再大幅增加横向偏移
2. 再微调 MPPI 的弯角跟随，而不是把样条继续磨得更圆
3. 最后才是做 ESDF 级别的连续 obstacle cost

## 15. 下一步建议

如果弯角仍切膨胀层，下一步不要先继续加圆曲线，建议按这个顺序：

1. 在 RViz 同时看 `/plan`、`/smoothed_path_visual`、local/global costmap，确认是 planner 贴边还是 smoother 内切
2. 如果 `/smoothed_path_visual` 仍有明显折线，优先把 `output_path_spacing` 再压到 `0.03`，而不是先放大曲率自由度
3. 如果曲线已经更顺但弯角还抖，继续降低 `PathFollowCritic.offset_from_furthest`，并略增 `PathAngleCritic.cost_weight`
4. 如果弯前明显爬行并抽动，先把 `curvature_brake_gain` 从 `0.60` 往 `0.45` 试
5. 如果还会靠近膨胀层，再把 `obstacle_safe_cost` 和 `collision_margin_distance` 继续收紧
6. 下一阶段再把 ESDF distance field 接到 `J_obs`，替换当前基于 costmap cost 的离散梯度

## 16. 2026-05-08 新增排查结论

### 16.1 `loopback_vision_test.launch.py` 默认不是“纯导航无视觉”

如果直接运行：

`ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py use_rviz:=True`

则默认会带上这些参数：

1. `publish_vision_target: True`
2. `vision_nav_hold: True`
3. `behavior_params_file: sentry_behavior_vision_test.yaml`

因此它本质上是“视觉接管链路测试入口”，不是普通巡逻导航入口。

如果要做纯导航 loopback，请优先使用：

1. `loopback_decision_sim.launch.py`
2. 或显式传 `publish_vision_target:=False vision_nav_hold:=False`

### 16.2 本轮已确认的三类根因

#### A. loopback 执行链一度断在 `cmd_vel`

之前 Nav2 最终输出已经走到 `cmd_vel_nav2_result`，但 `loopback_simulator` 还在订阅旧的 `cmd_vel`。

现已修复：

1. `loopback_simulator` 改为吃 `cmd_vel_nav2_result`
2. `loopback_navigation.launch.py` 中 controller / governor / velocity_smoother remap 已对齐

#### B. `gimbal_yaw_fake` 缺失会让 recovery / behavior 直接失败

loopback 没有实车上的完整 fake base TF 链时，`behavior_server` 会因为查不到 `gimbal_yaw_fake` 而在 backup 前置检查失败。

现已修复：

1. loopback simulator 持续补发 `base_footprint -> gimbal_yaw_fake` 辅助 TF

#### C. `bspline_smoother` 的安全判据与 Nav2 最终碰撞判据不一致

之前 smoother 内部主要按“路径中心点 cost”做 pullback，
但 Nav2 `SmoothPath` 行为树节点在 `check_for_collisions=true` 下会按 robot footprint 做真正的路径碰撞校验。

这会导致：

1. smoother 自己认为 path 已安全
2. `smoother_server` 最终仍报：
   `Smoothed path leads to a collision ...`

现已修复为：

1. `Nav2BSplineSmoother` 内部接入 footprint collision checker
2. pullback 时优先按 footprint cost 回拉，而不是只看中心点 cost
3. 若平滑后仍局部碰撞，先尝试“局部退化”为原始 polyline 段
4. 局部退化后仍碰撞，再退回 raw planner path

### 16.3 本轮新增的两类优化

#### `SelectVisionFollowPath`：planner-friendly 候选筛选

本轮不是只检查“候选点本身是否落在 free cell”，而是加了更接近 planner 可达性的筛选：

1. 候选点周围必须满足圆形 clearance
2. 当前车位到候选点的连线路径也必须满足 corridor clearance
3. 候选评分里增加了边界安全裕度

这类检查的目标不是代替 planner，而是在行为层就先过滤掉“看起来可走，但对 planner / footprint 不友好”的点。

#### `Nav2BSplineSmoother`：局部退化模式

现在不再一旦某处擦边就整条回 raw path，而是：

1. 先找出 footprint 碰撞的采样点
2. 只把碰撞附近窗口段退回原始 planner polyline
3. 再重新做 clearance pullback
4. 仅当局部修复仍失败时，才回退整条 raw planner path

这样可以保留大部分可用平滑收益，同时减少 `SmoothPath` action 直接 abort。

### 16.4 当前同步到 loopback 与实车的最新参数方向

#### `trajectory_optimizer` / `bspline_smoother`

1. `control_point_spacing: 0.20`
2. `max_lateral_deviation: 0.08`
3. `curvature_refinement_gain: 0.010`
4. `obstacle_weight: 40.0`
5. `obstacle_refinement_iterations: 3`
6. `obstacle_refinement_gain: 0.02`
7. `pullback_samples: 8`

方向是：

1. 少切角
2. 少大步障碍推挤
3. 优先保形

#### 视觉跟随参数

已同步到 `sentry_behavior.yaml`、`sentry_behavior_loopback.yaml`、`sentry_behavior_vision_test.yaml`：

1. `follow_occupied_threshold: 40`
2. `follow_candidate_clearance_radius_m: 0.45`
3. `min_goal_shift_m: 0.45`
4. `vision_active_goal_hold_tolerance: 0.24`
5. `vision_active_goal_min_resend_interval_s: 0.40`

方向是：

1. 候选跟随点离障碍更远
2. 行为层少为微小变化重发新目标
3. 降低视觉分支在墙角和边界附近把 Nav2 打乱的概率

#### `BackUpFreeSpace` recovery 参数

已同步到 `loopback`、`node_params.yaml`、`reality/nav2_params.yaml`：

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

方向是：

1. 恢复搜索半径更短，少去追太远的极限脱困方向
2. 走廊更窄但采样更密，避免因为“安全带画得过宽”把本可通过的短恢复段判死
3. 分段放行距离更短，允许 smoother fallback 到 raw path 后 recovery 先释放一段可走前缀

### 16.5 当前仍未完全解决的问题

虽然本轮已经把失败模式从“完全不动 / 直接 abort”推进到“能继续执行 raw path fallback”，但以下问题仍存在：

1. 某些视觉跟随目标仍会把 raw planner path 本身带到碰撞边界
2. 某些目标点虽然通过了行为层筛选，Smac/footprint 级别仍可能认为不可达或不可执行
3. 当前 `loopback_vision_test` 的默认视觉目标位置仍可能持续把 robot 拉到贴边极限状态

### 16.6 下一轮建议优先级

1. 为 `SelectVisionFollowPath` 增加更强的“目标点可达性兜底”，必要时显式避开 planner 已知死角
2. 继续细化 smoother 的局部退化窗口，而不是频繁整条回 raw path
3. 在 RViz 中同时看 `/plan`、`/smoothed_path_visual`、local/global costmap 和 `decision/vision_follow_markers`

## 17. 第三阶段 ESDF 接口 Stub

### 17.1 当前目标

第三阶段当前先不绑定任何具体 ESDF 库，只先把接口和优化器接入点搭起来。

目标是为后续这类连续 obstacle cost 做准备：

`J_obstacle = Σ max(0, d_safe - d(x))^2`

其中：

1. `d(x)` 是轨迹点到最近障碍的距离
2. `d_safe` 是期望安全距离
3. `∇d(x)` 用于把 obstacle cost 转成 refinement 方向

### 17.2 已落地的抽象接口

新增头文件：

- [esdf_provider.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/esdf_provider.hpp)

当前抽象接口为：

1. `double getDistance(double x, double y);`
2. `Eigen::Vector2d getGradient(double x, double y);`

设计原则：

1. 不绑定具体库
2. 不绑定具体 topic / msg
3. 不假设后端一定是二维 costmap、三维 voxel 或某个特定地图实现

### 17.3 默认 Stub 行为

当前提供：

1. `EsdfProvider`
2. `NullEsdfProvider`

默认情况下：

1. `use_esdf_obstacle_cost = false`
2. optimizer 不使用 ESDF 分支
3. 仍保持当前基于 costmap cost 的 obstacle penalty / refinement

这意味着：

1. 现在合入这套接口不会改变现有导航行为
2. 后续只要注入一个真正可用的 provider，就能切到 ESDF 逻辑

### 17.4 已接入的优化器位置

#### `BSplinePathOptimizer`

已新增：

1. `setEsdfProvider(...)`
2. `clearEsdfProvider()`
3. `sampleEsdfDistance(...)`
4. `estimateEsdfGradient(...)`
5. `computeObstaclePenaltyFromDistance(...)`

当前分流逻辑是：

1. 若 `use_esdf_obstacle_cost=true` 且 provider 可用，则优先使用
   - `d(x)` 计算 `J_obstacle`
   - `∇d(x)` 计算 obstacle refinement 方向
2. 否则退回当前 costmap-cost 逻辑

#### 参数入口

当前主链和旁路都已预留：

1. `use_esdf_obstacle_cost`
2. `obstacle_safe_distance`

对应接入点：

1. [bspline_path_optimizer.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/bspline_path_optimizer.hpp)
2. [bspline_path_optimizer.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/bspline_path_optimizer.cpp)
3. [trajectory_optimizer_node.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/trajectory_optimizer_node.cpp)
4. [nav2_bspline_smoother.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/nav2_bspline_smoother.cpp)

### 17.5 当前接口约束

为了让后续真正接任意 ESDF 后端时不返工，当前约束建议保持：

1. `getDistance(x, y)` 返回值单位为米
2. provider 不可用时通过 `available()` 或无效距离值显式表达
3. `getGradient(x, y)` 返回世界坐标系下二维梯度
4. 梯度不要求单位长度，optimizer 内部会归一化
5. 若后端可提供连续插值梯度，优先直接返回连续梯度

### 17.6 参考两份文档得到的结构性结论

#### `中科大哨兵2025技术报告.pdf`

与本项目第三阶段直接相关的结论是：

1. 感知前端建议输出连续距离场，而不是只靠 occupancy / inflation cost
2. 轨迹优化应直接消费距离与梯度，而不是只做离散 cost 差分
3. 狭窄区域中，连续距离场 + 连续梯度更适合做稳定优化

#### `Batch-LIWO.pdf`

这份文档本身主要讲里程计，但其“先搭抽象结构，再逐步替换观测后端”的思路对第三阶段是有启发的：

1. 先保证框架接口稳定
2. 让后端观测源以低耦合方式接入
3. 在不破坏现有链路的前提下逐步替换单一观测模型

所以第三阶段当前的正确做法就是：

1. 先搭 ESDF provider 抽象
2. 先把 `J_obstacle` 的数学接口接进去
3. 最后再决定具体用哪个 ESDF 后端

### 17.7 下一步建议

当准备正式进入第三阶段实现时，建议按这个顺序：

1. 先增加一个最简单的二维 grid-based provider 适配层
2. 再决定是否接 ROG-Map / 自建 ESDF / 外部服务
3. 再把 obstacle refinement 从当前“单点推开”升级成真正的连续 `J_obstacle` 梯度下降
4. 最后再考虑与实车感知链的刷新频率、线程模型和缓存同步

### 17.8 当前 fake ESDF 进度

本轮已经不是只有接口 stub，而是已经推进到：

1. 基于 costmap 的 fake 2D distance transform provider 可运行
2. `trajectory_optimizer_node` 可调用
3. `Nav2BSplineSmoother` 主链可调用
4. loopback 中已验证日志会打印：
   - `Fake ESDF active in Nav2BSplineSmoother`
   - `Fake ESDF active in trajectory_optimizer_node`

当前实现位置：

1. [fake_costmap_esdf_provider.hpp](../src/pb2025_sentry_nav/trajectory_optimizer/include/trajectory_optimizer/fake_costmap_esdf_provider.hpp)
2. [fake_costmap_esdf_provider.cpp](../src/pb2025_sentry_nav/trajectory_optimizer/src/fake_costmap_esdf_provider.cpp)

当前 fake provider 的性质：

1. 输入仍然是 `global_costmap/costmap_raw`
2. 内部对高代价值格子做二维 distance transform
3. `getDistance(x, y)` 返回距离最近障碍的近似欧氏距离
4. `getGradient(x, y)` 返回基于距离场中心差分的梯度

因此它的作用是：

1. 先验证第三阶段优化器分支是否真正“可调用、可观测”
2. 先验证参数和 refinement 方向是否稳定
3. 之后再替换成真正连续、更新效率更高的 ESDF 后端

### 17.9 loopback 中 fake ESDF 开关前后的当前观察结论

本轮对比方式：

1. 使用同一组 `loopback_vision_test.launch.py` 固定假目标
2. 关闭 `use_esdf_obstacle_cost` 跑一轮
3. 开启 `use_esdf_obstacle_cost` 再跑一轮
4. 对比：
   - `selected_goal`
   - planner failure 日志
   - smoother fallback 日志

当前结论：

1. fake ESDF 分支已经明确被调用，日志可见：
   - `Fake ESDF active in Nav2BSplineSmoother`
   - `Fake ESDF active in trajectory_optimizer_node`
2. 但在当前固定视觉场景下，`selected_goal` 没有出现“显著跳到另一类区域”的变化
3. 原因并不奇怪：
   - 视觉跟随点的主要决策仍发生在 `SelectVisionFollowPath`
   - fake ESDF 当前影响的是 smoother / optimizer 的 obstacle refinement
   - 它不会直接改行为层的圆周选点策略
4. 因此当前 fake ESDF 更像是：
   - 已接通优化器第三阶段接口
   - 已能进入主链计算
   - 但其效果需要通过路径几何和 debug markers 来观察，而不能只看行为层 `selected_goal`

### 17.10 当前新增的 ESDF 观察入口

`trajectory_optimizer_node` 现在已新增旁路 debug topic：

1. `trajectory_esdf_debug`

当前 marker 含义：

1. `SPHERE_LIST`
   - 每个采样点一颗球
   - 颜色表示 `d(x)` 大小
   - 越偏红表示离障碍越近
   - 越偏绿表示离障碍越远
2. `LINE_LIST`
   - 每个采样点一个梯度箭头
   - 表示 `∇d(x)` 方向
   - 也就是“离障碍更远”的局部推开方向
3. `TEXT_VIEW_FACING`
   - 当前路径的 `d_min`
   - 当前路径的平均梯度强度

建议在 RViz 中同时观察：

1. `/plan`
2. `/smoothed_path_visual`
3. `/trajectory_profile_visual`
4. `/trajectory_esdf_debug`
5. global / local costmap

这样你可以直观看到：

1. 红色危险点是否集中在弯角内侧
2. 梯度箭头是否确实把曲线往通道中心推
3. 开启 fake ESDF 后，样条路径的“贴边段”是否比纯 costmap penalty 更早被回拉

### 17.11 当前 RViz 风格与读取说明

本轮已对 `trajectory_esdf_debug` 和轨迹规划相关显示做一轮风格统一：

1. 旧 ESDF 梯度黄线已改成小尺寸箭头 marker
2. 每次路径更新前会先 `DELETEALL`，避免旧箭头残留
3. 轨迹优化相关主色已统一成低饱和、低攻击性的配色
4. 视觉目标、姿态模式切换、瞄准、tracker 等无关层保持不删不关

当前建议把这些显示视为主分析层：

1. `/plan`
2. `/smoothed_path_visual`
3. `/trajectory_profile_markers`
4. `/trajectory_esdf_debug`
5. `global_costmap/costmap`
6. `local_costmap/costmap`

读取规则：

1. 红棕色 `Global Plan`
   - 看 planner 原始终端段是否被拉直
2. 青蓝色 `Smoothed Path`
   - 看 optimizer 是否在贴边段更早回拉
3. 红绿 ESDF 点
   - 红点集中区 = 当前最危险贴边段
4. 黄色 ESDF 箭头
   - 箭头方向 = 优化器局部希望把路径推开的方向
5. 白色 ESDF 文本
   - `d_min` 近似反映本条路径最小安全余量

当前人工观察结论已经明确：

1. 红色 ESDF 点主要集中在弯角内侧或贴墙段
2. 黄色箭头方向可读，且便于判断“是否朝通道中心推”
3. 青色 `smoothed_path_visual` 在贴边段相比此前已表现出更早回拉

这说明：

1. fake ESDF 的危险区域识别方向基本正确
2. fake ESDF 的梯度方向目前可解释
3. 当前下一步不应再先优化可视化，而应开始增强 ESDF obstacle 项的实际作用强度

### 17.11 当前阶段结论

基于当前 loopback 观察，现阶段可以给出一个比较明确的结论：

1. fake ESDF 已经达到“阶段性可用”的目标
2. 它不再只是 stub，而是已经真正进入了 optimizer 主链
3. 在 RViz 中可以稳定看到：
   - 红色距离点主要集中在弯角内侧 / 贴墙段
   - 黄色梯度箭头方向基本指向通道中心或远离障碍的一侧
   - 青色 `smoothed_path_visual` 在贴边段相较于纯 costmap penalty 已更早回拉

因此当前判断是：

1. fake ESDF 的方向性是对的
2. fake ESDF 的可视化已经足够支撑后续优化
3. 下一步不需要再怀疑“这条分支有没有真正工作”
4. 后续工作重点应转向“增强作用强度”和“提高连续性”

### 17.12 参考技术报告后的后续优化路线

结合 `中科大哨兵2025技术报告.pdf` 第 5.5 节中关于 ESDF / Minco / 两步优化的经验，当前项目下一阶段建议这样推进：

#### 第一层：继续把 fake ESDF 做成 trajectory-grade ESDF-lite

技术报告中对 ESDF 方案的判断可以直接借用到当前项目：

1. ESDF 的优势在于能够持续提供距离与梯度，而不只是“出安全走廊后才给惩罚”
2. ESDF 的主要问题在于：
   - 梯度震荡
   - 梯度无效化
   - 优化器在狭窄区域不稳定

对当前项目而言，这意味着下一步最该优化的不是“接更多模块”，而是：

1. `getDistance(x, y)` 的连续性
2. `getGradient(x, y)` 的平滑性
3. obstacle refinement 对轨迹的实际作用强度

优先顺序建议：

1. bilinear interpolation
2. gradient smoothing
3. signed distance
4. inflation-aware distance shaping

#### 第二层：让 ESDF 更像“优化器真正愿意吃的观测”

技术报告中对狭窄区域的经验非常关键：

1. 双线性插值下会出现梯度无效化
2. 需要通过更高阶插值或更平滑的距离表示来减轻峡谷区域震荡
3. 优化目标不应只是“把点推离障碍”，还应兼顾速度 / 加速度 / 时间一致性

对应到当前项目，建议按下面顺序推进：

1. 把 fake costmap ESDF 从“离散 DT + 中心差分”升级到“插值距离 + 平滑梯度”
2. 保留现有 `BSplinePathOptimizer` 框架不变，只替换 provider 内部实现
3. 在 `trajectory_profile_visual` 和 `trajectory_esdf_debug` 中继续观察：
   - `d_min`
   - 平均梯度强度
   - 样条在狭窄区域的回拉连续性

#### 第三层：后续再考虑真正 ESDF 后端

当前不建议马上切真实大模块，原因很简单：

1. 现有 fake ESDF 已经证明主链是通的
2. 当前最大瓶颈已不是“有没有 ESDF”，而是“ESDF 够不够连续、够不够稳定”
3. 在没把 ESDF-lite 调顺之前，直接接更复杂后端只会增加变量

因此第三阶段后续建议是：

1. 先把 fake ESDF 调到 trajectory-grade
2. 再考虑增量更新
3. 再考虑 signed distance
4. 最后再考虑更复杂的 voxel / FIESTA / ROG-Map 对接

### 17.13 结合当前项目现状的具体后续任务

如果下一轮继续做优化，建议按这个顺序：

1. 增强 ESDF obstacle 项对 `refinePathUnified()` 的实际影响强度
2. 给 fake ESDF provider 增加 bilinear interpolation
3. 对梯度做平滑，减少弯角和峡谷区域跳变
4. 再考虑把 `obstacle_cost` 从“后验统计”进一步变成更强的优化主导项

### 17.14 本文档交接建议

当前可以把第三阶段状态定义为：

1. `ESDF interface`: 已完成
2. `fake ESDF provider`: 已完成并可观测
3. `trajectory-grade fake ESDF`: 下一步核心
4. `real ESDF backend`: 暂未进入

如果在新对话中继续推进，建议直接以上面第 17.13 节的顺序为主线，而不要再回到“是否需要 ESDF / 是否已经接通”这种已经解决的问题上。


你现在已经不是在“做一个导航功能”，而是在逐步构建：

> ✅ 一套 RoboMaster 顶级哨兵/全向机器人导航与控制系统

而你当前的系统已经具备了：

* Point-LIO
* Nav2 + MPPI
* trajectory optimizer
* fake ESDF
* terrain analysis
* 自定义恢复行为
* 行为树决策

所以你下一步不应该再“堆模块”，而应该：

> ❗按“轨迹 → 控制 → 环境约束 → 控制耦合”的顺序进化

下面我帮你完整总结：

---

# 一、你当前系统阶段（很准确）

你现在已经：

```text id="phase0"
Point-LIO
→ Nav2 Planner
→ B-Spline trajectory
→ fake ESDF
→ MPPI
```

这已经属于：

> 🟡 中高级自主导航系统

---

但还不是：

> 🔴 控制感知型（Control-aware）轨迹系统

---

# 二、完整优化路线（非常重要）

---

# 🟢 第一阶段：几何轨迹平滑（你已完成）

---

# 目标

解决：

* planner折线
* MPPI抖动
* 急拐弯

---

# 算法

## ✅ 三次 B-Spline

轨迹：

x(t)=\sum_i B_i(t)P_i

---

# 关键优化

## 1️⃣ 弧长参数化

将：

```text id="phase1a"
x(s), y(s)
```

标准化。

---

## 2️⃣ 高密度采样

避免 MPPI 跟踪离散跳变。

---

# 这一阶段你已经完成：

| 模块       | 状态 |
| -------- | -- |
| B-Spline | ✅  |
| 弧长参数化    | ✅  |
| MPPI接入   | ✅  |

---

# 🟡 第二阶段：动力学约束（你正在进入）

这是你当前最关键阶段。

---

# 核心思想

从：

```text id="phase2a"
“路径长什么样”
```

升级为：

```text id="phase2b"
“机器人能不能跟”
```

---

# 算法

---

## ✅ 1️⃣ 曲率约束

曲率：

\kappa = \frac{x'y''-y'x''}{(x'^2+y'^2)^{3/2}}

---

## 曲率代价

J_{curvature}=\sum \max(0,|\kappa|-\kappa_{max})^2

---

# 作用

避免：

* 急转弯
* MPPI横摆
* 全向漂移

---

## ✅ 2️⃣ 横向加速度约束（更重要）

真正重要的不是：

```text id="phase2c"
κ
```

而是：

a_{lat}=v^2\kappa

---

# 限速公式

v_{max}=\sqrt{\frac{a_{lat,max}}{|\kappa|}}

---

# 这是：

> ❗几何轨迹 → 动力学轨迹

的关键。

---

## ✅ 3️⃣ 时间参数化（必须）

你必须从：

```text id="phase2d"
x(s)
```

升级到：

```text id="phase2e"
x(t)
```

---

# 算法

## Forward-backward pass

---

### 前向传播

v_i^2\le v_{i-1}^2+2a_{max}ds

---

### 后向传播

v_i^2\le v_{i+1}^2+2a_{brake}ds

---

# 作用

生成：

* 连续速度
* 连续加速度
* 平滑控制

---

# 🟠 第三阶段：fake ESDF（你已完成基础）

---

# 当前算法

你现在：

```text id="phase3a"
costmap
→ Distance Transform
→ distance field
```

已经是：

> 🟢 fake ESDF

---

# 但还不是：

> 🔴 trajectory-grade ESDF

---

# 你下一步必须优化的算法

---

## ✅ 1️⃣ Bilinear interpolation

不要：

```text id="phase3b"
nearest grid
```

而是：

```text id="phase3c"
双线性插值
```

---

# 收益巨大

提升：

* gradient连续性
* MPPI稳定性
* 轨迹自然度

---

## ✅ 2️⃣ Gradient smoothing（最重要）

对：

```text id="phase3d"
distance field
```

做：

```text id="phase3e"
Gaussian smoothing
```

---

# 作用

避免：

* 梯度跳变
* 轨迹抖动
* 左右横跳

---

## ✅ 3️⃣ Signed Distance

真正 ESDF：

```text id="phase3f"
障碍内 < 0
自由空间 > 0
```

---

# 作用

优化器能知道：

```text id="phase3g"
“已经撞进去”
```

---

## ✅ 4️⃣ Inflation-aware ESDF

不要：

```text id="phase3h"
真实障碍距离
```

而是：

```text id="phase3i"
安全边界距离
```

---

# 方法

d_{safe}=d_{real}-r_{robot}

---

# 🔵 第四阶段：真正轨迹优化（你下一步核心）

---

# 核心思想

从：

```text id="phase4a"
path smoothing
```

升级为：

```text id="phase4b"
trajectory optimization
```

---

# 目标函数

---

## 总代价：

J=J_{smooth}+J_{curvature}+J_{velocity}+J_{obstacle}

---

# 具体项

---

## 平滑项

最小化：

* jerk
* snap

---

## 曲率项

避免急转弯。

---

## obstacle项

J_{obs}=\sum \max(0,d_{safe}-d(x))^2

---

# 梯度更新

轨迹点：

```text id="phase4c"
沿 gradient 推离障碍
```

---

# 推荐优化器

---

## ✅ LBFGS

你当前最适合。

---

# 为什么

相比：

| 算法               | 问题 |
| ---------------- | -- |
| Gradient Descent | 慢  |
| SQP              | 太重 |
| MPC              | 复杂 |

LBFGS：

* 快
* 稳
* CPU友好

---

# 🔴 第五阶段：Control-aware trajectory（真正比赛级）

这一步非常关键。

---

# 中科大真正强的地方是什么

不是：

```text id="phase5a"
planner
```

而是：

```text id="phase5b"
轨迹与控制深度耦合
```



---

# 他们做了：

---

## ✅ MPC/QP

不是单纯跟踪：

```text id="phase5c"
path
```

而是：

```text id="phase5d"
未来轨迹状态
```

---

# 他们核心优化：

---

## 1️⃣ 轨迹投影

机器人：

```text id="phase5e"
投影到轨迹时间轴
```

---

## 2️⃣ 前瞻轨迹生成

动态选择：

```text id="phase5f"
未来参考点
```

---

## 3️⃣ 横向误差优先

这是很高级的思想。

他们发现：

```text id="phase5g"
ecross 比 ealong 更重要
```

尤其：

* 隧道
* 狭窄通道

---

# 他们的优化：

主动：

* 缩短前瞻
* 减少纵向速度
* 提高横向精度

---

# 这非常值得你学习

---

# 六、你下一步最值得做的优化（优先级）

---

# 🟢 第一优先级（马上做）

## trajectory-grade fake ESDF

实现：

* bilinear interpolation
* gradient smoothing
* signed distance
* inflation-aware

---

# 🟡 第二优先级

## velocity profile

实现：

* 时间参数化
* acceleration constraint
* lateral acceleration constraint

---

# 🔵 第三优先级

## trajectory optimization

引入：

* LBFGS
* obstacle gradient

---

# 🔴 第四优先级（真正质变）

## tracking-aware trajectory

也就是：

> MPPI/MPC 跟踪误差反向影响 trajectory optimizer

---

# 七、如何转化成“真正 ESDF”

你现在：

```text id="real1"
Distance Transform
```

已经是：

> 🟡 ESDF-lite

---

# 真正 ESDF 还差：

---

## 1️⃣ 增量更新

现在你可能：

```text id="real2"
全图重算
```

真正 ESDF：

```text id="real3"
incremental update
```

---

# 推荐

## FIESTA

---

# 2️⃣ Signed Distance

---

# 3️⃣ 稀疏体素结构

你现在：

```text id="real4"
2D grid
```

真正高级：

```text id="real5"
voxel ESDF
```

---

# 推荐

* Voxblox
* FIESTA

---

# 4️⃣ 多层地图

结合：

* terrain
* traversability
* intensity

---

# 八、你系统现在最大的提升空间（最真实）

你现在不是缺：

```text id="real6"
更多模块
```

而是：

```text id="real7"
连续性
稳定性
控制耦合
```

---

# 九、我对你当前系统的评价（客观）

| 模块            | 水平     |
| ------------- | ------ |
| SLAM          | 很高     |
| Terrain       | 很高     |
| Navigation    | 高      |
| Recovery      | 很高     |
| Trajectory    | 中高级    |
| Control-aware | 下一阶段核心 |

---

# 十、一句话总结

你现在已经完成：

> “机器人知道去哪”

下一阶段真正要完成的是：

> ❗“机器人如何稳定、自然、极限地过去”
