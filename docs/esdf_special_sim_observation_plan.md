# 2.5D ESDF 专项仿真观察方案

更新时间：2026-06-24

本文档的目标不是讨论“最终仿真器选型”，而是解决当前一个更直接的问题：

1. 现在的 `RC-ESDF-lite + slope_grid + trajectory_profile + governor` 到底有没有生效。
2. 在当前仓库基础上，怎样用最少改动把效果看清楚。
3. 为什么当前阶段优先继续用 `Gazebo + loopback`，而不是立刻切到 `MuJoCo`。

## 1. 先说结论

当前阶段不建议为了观察 `2.5D ESDF` 效果而立刻切换到 `MuJoCo`。

原因：

1. 当前仓库的导航、点云、地形语义、ESDF、Nav2 和 RViz 观测链已经围绕 `Gazebo / loopback` 搭好。
2. 当前要验证的重点不是“更真实的接触动力学”，而是：
   `terrain_analysis_ext -> traversability_*_grid -> TraversabilityEsdfProvider -> smoother / profile / governor`
   这条链是否按预期工作。
3. `MuJoCo` 真正更有价值的阶段，通常是在后续验证：
   `底盘动力学`、
   `SE2 MPC`、
   `轮地接触`、
   `高带宽控制器`
   这些问题时。
4. 对当前项目，真正不明显的往往不是“Gazebo 看不出来”，而是：
   没有把该看的语义图层、ESDF debug、profile marker 和速度链一起摆到同一个观察面里。

所以当前最优先的事情不是换仿真器，而是：

1. 强化 Gazebo / loopback 下的可观测性。
2. 固化一套标准观察视图。
3. 固化一套标准测试场景和对比流程。

## 2. 当前推荐的验证分工

建议把现有验证拆成两条线：

### A. Gazebo：系统级验证

适合验证：

1. 点云、里程计、`terrain_analysis_ext`、`traversability_*_grid` 是否连通。
2. `TraversabilityEsdfProvider` 是否正确构造 signed distance 与 slope 查询。
3. `Nav2BSplineSmoother` 是否会对贴边路径做安全回拉。
4. `trajectory_profile` 与 `trajectory_speed_governor` 是否把坡度规则真正落实到速度输出上。

当前入口：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py \
  use_rviz:=True \
  launch_trajectory_optimizer:=True
```

参考：

1. [docs/gazebo_sim_integration.md](./gazebo_sim_integration.md)
2. [src/pb2025_sentry_bringup/launch/gazebo_bringup.launch.py](../src/pb2025_sentry_bringup/launch/gazebo_bringup.launch.py)

### B. loopback：观测与回归验证

适合验证：

1. `plan -> smoothed_path_visual` 的几何变化是否符合预期。
2. `trajectory_esdf_debug`、`trajectory_profile_markers` 是否能稳定解释回拉和限速结果。
3. 对比参数开关前后，ESDF / 坡度规则是否真的影响 `cmd_vel_controller_governed`。

当前入口：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_nav_only.launch.py use_rviz:=True
```

参考：

1. [README.md](../README.md)
2. [src/pb2025_sentry_bringup/launch/loopback_nav_only.launch.py](../src/pb2025_sentry_bringup/launch/loopback_nav_only.launch.py)

## 3. 推荐使用的 RViz 视图

为了避免默认视图里的信息太多、太杂，当前新增了一份更聚焦的 ESDF 观察视图：

1. [src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_esdf_observe_view.rviz](../src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_esdf_observe_view.rviz)

建议启动方式：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py \
  use_rviz:=True \
  launch_trajectory_optimizer:=True \
  rviz_config_file:=$(pwd)/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_esdf_observe_view.rviz
```

或者 loopback：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_nav_only.launch.py \
  use_rviz:=True \
  rviz_config_file:=$(pwd)/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_esdf_observe_view.rviz
```

这份视图默认强调：

1. `traversability_height_diff_grid`
2. `traversability_occupancy_ratio_grid`
3. `traversability_slope_grid`
4. `traversability_slope_band_grid`
5. `traversability_ground_confidence_grid`
6. `plan`
7. `smoothed_path_visual`
8. `trajectory_profile_markers`
9. `trajectory_esdf_debug`

同时默认弱化或关闭：

1. `MPPI Rollouts`
2. `Lookahead Point`
3. 局部 footprint polygon

原因很简单：

1. 当前要看的不是控制器采样细节，而是 ESDF 与坡度语义是否真正改变了轨迹和速度链。

## 4. 必看话题

当前专项观察建议固定盯下面这些 topic：

### 4.1 地形语义层

1. `traversability_grid`
2. `traversability_height_diff_grid`
3. `traversability_occupancy_ratio_grid`
4. `traversability_ground_confidence_grid`
5. `traversability_slope_grid`
6. `traversability_slope_band_grid`

观察目标：

1. ESDF 风险高的位置，是否能被其中一张或多张语义图解释。
2. 坡度较小区域是否真的被识别成“可加速”的候选区。
3. 接近坡度障碍阈值的区域，是否能在 `slope_grid` 里清楚看到分界。

### 4.2 轨迹几何层

1. `plan`
2. `smoothed_path_visual`
3. `transformed_global_plan`
4. `trajectory_esdf_debug`

观察目标：

1. 贴边段是否发生明显回拉。
2. 回拉是否朝着 ESDF gradient 的安全侧发生。
3. 回拉后轨迹是否仍保持连续、不过度抖动。

### 4.3 速度剖面层

1. `trajectory_profile`
2. `trajectory_profile_markers`
3. `cmd_vel_controller`
4. `cmd_vel_controller_governed`
5. `cmd_vel_nav2_result`

观察目标：

1. 低于坡度障碍阈值时，是否能看到 profile 中更高的近端速度上限。
2. 超过阈值时，是否能看到 speed_limit 和纵向加速度变保守。
3. governor 是否真正把 profile 的目标速度落实到了输出速度。

## 5. 推荐测试场景

为了让 ESDF 和坡度规则“看起来明显”，建议优先做下面 4 类场景，而不是随便找一段平地跑。

### 场景 1：贴边直走廊

目标：

1. 观察 `plan` 是否贴近障碍边界。
2. 观察 `smoothed_path_visual` 是否被 ESDF 稳定回拉。
3. 观察 `trajectory_esdf_debug` 是否在贴边段给出明显的危险点与 gradient。

重点看：

1. `plan`
2. `smoothed_path_visual`
3. `trajectory_esdf_debug`

### 场景 2：窄门 / 门框进入

目标：

1. 观察 ESDF 是否帮助 smoother 避免“中心线能过、车体轮廓其实擦边”的风险。
2. 观察回拉是否保持门中心通过，而不是左右来回抖。

重点看：

1. `traversability_grid`
2. `trajectory_esdf_debug`
3. `smoothed_path_visual`

### 场景 3：缓坡到陡坡过渡

目标：

1. 验证低于坡度障碍阈值时是否真的存在加速增益。
2. 验证接近阈值时增益是否回落到 `1.0`。
3. 验证超过阈值后是否开始限速和压缩纵向加速度。

重点看：

1. `traversability_slope_grid`
2. `trajectory_profile_markers`
3. `cmd_vel_controller`
4. `cmd_vel_controller_governed`

### 场景 4：弯道叠加坡度

目标：

1. 验证“曲率限速”和“坡度速度规则”叠加后是否还可解释。
2. 验证 governor 是不是同时尊重曲率与 profile 的绝对目标速度。

重点看：

1. `trajectory_profile_markers`
2. `cmd_vel_controller`
3. `cmd_vel_controller_governed`

## 6. 建议的对比方法

建议不要只看“开了以后怎么样”，而要固定做 A/B 对比。

### 对比 A：ESDF 回拉效果

对比项：

1. `launch_trajectory_optimizer:=True`
2. `trajectory_optimizer.use_esdf_obstacle_cost := true / false`

目的：

1. 直接看 signed traversability ESDF 对路径几何回拉的影响。

### 对比 B：坡度速度规则

对比项：

1. `use_slope_speed_limits := true / false`
2. `use_slope_accel_limits := true / false`

目的：

1. 直接看 `trajectory_profile` 和 `cmd_vel_controller_governed` 是否产生变化。

### 对比 C：加速阈值位置

对比项：

1. `slope_speed_obstacle_deg`
2. `slope_accel_obstacle_deg`

目的：

1. 验证“低于坡度障碍阈值时加速、超过阈值后保守”的语义边界是否合理。

## 7. 推荐的最小测试流程

如果只想先做一轮最小验证，建议按下面顺序：

1. 启动 Gazebo：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup gazebo_bringup.launch.py \
  use_rviz:=True \
  launch_trajectory_optimizer:=True \
  rviz_config_file:=$(pwd)/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_esdf_observe_view.rviz
```

2. 先观察地形语义是否稳定：
   `traversability_grid`
   `height_diff`
   `occupancy_ratio`
   `ground_confidence`
   `slope`
3. 在 RViz 里给一个贴边或窄门目标点。
4. 观察：
   `plan`
   `smoothed_path_visual`
   `trajectory_esdf_debug`
5. 再跑一段坡度变化明显的区域，观察：
   `trajectory_profile_markers`
   `cmd_vel_controller`
   `cmd_vel_controller_governed`
6. 记录一组截图和 rosbag：
   至少保留“地形语义层”、“轨迹层”、“速度链层”三张图。

## 8. 什么时候再考虑 MuJoCo

当下面这些问题成为主矛盾时，再认真考虑 MuJoCo 更合适：

1. 需要更真实地验证底盘纵向 / 横向加减速极限。
2. 需要验证 `SE2 MPC` 的高带宽控制效果。
3. 需要验证轮地接触、打滑、冲坡和接触恢复。
4. 当前 Gazebo 的接触动力学已经成为主要瓶颈，而不是 ESDF / 轨迹 / 速度规则本身。

在那之前，当前项目更应该优先做的是：

1. 把 Gazebo / loopback 的 ESDF 可观测性做强。
2. 把对比测试流程固化。
3. 用同一套场景持续回归任务 1 / 任务 2 的效果。
