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
