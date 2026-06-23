# MPPI 参数调试说明

## 适用范围

本文主要说明当前项目中 MPPI 局部控制器的核心参数含义、常见现象与调参方向，优先针对：

- `ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True`
- `ros2 launch pb2025_sentry_bringup loopback_nav_only.launch.py use_rviz:=True`
- 参数文件：`src/loopback_sim/params/nav2_params.yaml`

这套说明也可以迁移到：

- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`
- `src/pb2025_sentry_bringup/params/node_params.yaml`

但建议先在 loopback 中把现象调顺，再同步到正式仿真和实车。

另外一定要记住：

- `loopback_nav_only.launch.py`
  - 适合只看导航、平滑、ESDF、trajectory profile 和速度链
- `loopback_decision_sim.launch.py`
  - 默认读取 `src/loopback_sim/params/nav2_params.yaml`
- `bringup.launch.py`
  - 默认读取 `src/pb2025_sentry_bringup/params/node_params.yaml`

如果你在 loopback 里观察局部路径和 trajectories，却去改 `node_params.yaml`，那么大概率不会看到你预期的变化。

## 官方参考

本文关于参数定义、默认值和调参原则，优先参考 Nav2 官方 MPPI 文档：

- Nav2 MPPI Controller 配置总览：
  - https://docs.nav2.org/configuration/packages/configuring-mppic.html

根据官方文档，下面几条尤其值得优先记住：

1. `model_dt` 一般应与控制周期一致，通常不要大于控制周期。
2. `iteration_count` 官方通常建议保持为 `1`，优先增加 `batch_size`。
3. `batch_size` 的官方经验值大致是：
   - 50Hz 时约 `1000`
   - 30Hz 时约 `2000`
4. `visualize` 很适合调试，但官方明确提醒它会增加控制器计算开销。
5. `ObstaclesCritic.inflation_radius` 与 `cost_scaling_factor` 在 Humble 下应与 costmap inflation layer 保持一致。
6. 官方特别提醒：很多实际问题，最先该检查的是运动模型、速度边界和 obstacle critic 与 inflation layer 的匹配关系，而不是先猛调 critic 权重。

另外两份本次项目中非常常用的官方参考：

- Nav2 Inflation Layer：
  - https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
- Nav2 Smac Hybrid-A* Planner：
  - https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html

## 为什么 MPPI 调参容易痛苦

MPPI 不是单一 PID 参数，而是：

1. 在一个有限时域里采样很多条未来轨迹
2. 用多组 critic 给每条轨迹打代价
3. 选出综合代价最低的一批轨迹进行控制输出

所以它的现象往往不是某一个参数单独决定，而是：

- 预测时域
- 采样扰动
- 速度上下限
- 路径类 critic
- 目标类 critic
- 障碍物 critic

一起耦合出来的。

调 MPPI 时，最容易犯的错误不是“参数调错了”，而是“一次改太多，不知道哪项在起作用”。

## 权重调大到底是数值调大还是调小

结论很简单：

- 对 `cost_weight`、`repulsion_weight`、`critical_weight` 这类真正的权重参数，调大权重就是把数值调大。
- 数值越大，该 critic 对总 cost 的影响越强，MPPI 越不愿选择被它惩罚的轨迹。
- `cost_power` 不是普通权重，它会改变代价曲线形状；通常先保持 `1`，不要把它当作第一调参旋钮。

需要特别区分的是，很多 MPPI 参数名字里没有 `weight`，它们不是“权重”：

- `threshold_to_consider` 是距离目标点的生效窗口。
  - `PathAlignCritic`、`PathFollowCritic`、`PathAngleCritic`、`PreferForwardCritic`：进入该距离后关闭，把终点段交给 goal 类 critic。
  - `GoalCritic`、`GoalAngleCritic`：进入该距离后开启，开始主导终点收敛。
  - `ObstaclesCritic.near_goal_distance`：进入该距离后关闭普通 repulsion 斥力，只保留碰撞/近碰撞惩罚。
- `offset_from_furthest` 是沿路径向前看的 point 偏移，调大通常更积极、更快，但也更容易 shortcut。
- `temperature` 是 MPPI softmax 温度，调小更偏向最低代价样本，调大更像对多条样本平均。
- `vx_std / vy_std / wz_std` 是采样扰动标准差，不是速度上限；调大探索更强，但 rollout 更容易散。

## 先看什么可视化

调 MPPI 时，推荐同时观察：

- `/plan`
  - 全局路径
- `/transformed_global_plan`
  - MPPI 使用的局部参考路径
- `/trajectories`
  - MPPI 每周期采样出的 rollout

经验上：

- `transformed_global_plan` 形状正常，但 `trajectories` 乱飘
  - 先查 MPPI 参数
- `transformed_global_plan` 本身就奇怪
  - 先查行为树、路径点切换、全局规划输入

- `transformed_global_plan` 正常，但机器人一边走一边像“地图坐标被轻微重置”
  - 先查 loopback 是否在反复发布 `initialpose`
- `scan`/costmap 偶发抖动，且日志里出现 message filter 早于 TF cache
  - 先查 loopback 时间戳链，而不是先怀疑 MPPI critic

如果你现在正在调“贴墙、狭窄通道、终点抖动”，建议再同时观察：

- `local_costmap/costmap_raw`
- `global_costmap/costmap_raw`
- `/smoothed_path_visual`
- `/trajectory_profile`
- `/trajectory_profile_markers`
- `/trajectory_esdf_debug`
- `/cmd_vel_controller`
- `/cmd_vel_controller_governed`
- `/cmd_vel_nav2_result`
- `back_up_free_space_markers`（如果恢复可视化打开）

## 调参基本顺序

建议按下面顺序调，而不是乱试：

1. 先定预测范围
2. 再定速度采样范围
3. 再定路径相关 critic
4. 再定目标相关 critic
5. 最后处理终点附近的小抖动

原因是：

- 预测时域和速度上限不合理时，后面 critic 再精调也会被“更大的错误”掩盖

## 参数分组说明

说明：

- 每个参数先尽量按“官方定义/官方建议”理解
- 再结合本项目中出现的现象做调参解释

这样可以避免把项目经验误当成通用规律。

### 1. Goal Checker

#### `xy_goal_tolerance`

作用：

- 终点位置容差，决定 Nav2 什么时候判定“到点”

官方建议：

- 官方文档没有给固定推荐值，应根据任务是否强调最终贴点精度决定。

调大后的现象：

- 更容易结束
- 终点附近左右试探会减少
- 但可能还没完全贴到目标点就提前结束

调小后的现象：

- 更追求精确停点
- 但终点附近更容易反复修正

当前 loopback 建议：

- 巡逻类任务通常 `0.15 ~ 0.25`

### 2. 优化器参数

#### `time_steps`

作用：

- 预测步数
- 与 `model_dt` 相乘后得到总预测时域

官方定义：

- `time_steps * model_dt` 就是 prediction horizon。

公式：

```text
prediction_horizon = time_steps * model_dt
```

例如：

- `time_steps=20`
- `model_dt=0.05`
- 总预测时域约 `1.0s`

调大后的现象：

- 看得更远
- 高速直线可能更平顺
- 转角和终点附近更容易“想太远”，rollout 扇形发散

调小后的现象：

- 更贴近当前局部
- 转角和终点通常更稳
- 但可能变得短视，提前量不足

经验：

- 如果 `trajectories` 明显超出局部地图边界，先减 `time_steps`

#### `model_dt`

作用：

- 每个预测步长的时间间隔

通常建议：

- 与控制频率一致
- 如果 `controller_frequency = 20Hz`，则 `model_dt = 0.05`

官方建议：

- 一般不要比控制周期更大。

一般不优先动它，除非整套控制频率都要重构。

#### `batch_size`

作用：

- 每周期采样多少条候选轨迹

官方建议：

- `iteration_count` 保持 `1`
- 优先通过增大 `batch_size` 改善效果
- 经验值：
  - `1000 @ 50Hz`
  - `2000 @ 30Hz`

调大后的现象：

- 最优轨迹更稳定
- 对随机抖动更不敏感
- CPU 占用更高

调小后的现象：

- 算得快
- 但最优轨迹更容易跳来跳去

经验：

- 如果控制频率掉不下来，再考虑减它
- 否则先别动

#### `temperature`

作用：

- 决定对“最优样本”的偏执程度

官方定义：

- 越接近 `0` 越偏向最低代价控制
- 很大时会逐渐接近对所有轨迹取平均

调小后的现象：

- 更激进
- 只盯最优样本
- 容易突然换轨、抖动

调大后的现象：

- 更平滑
- 更温和
- 但过大可能显得拖沓

经验：

- 看到 `trajectories` 中“赢家”经常突然跳边，优先略增 `temperature`

#### `gamma`

作用：

- 对控制变化和控制能量的平滑正则

官方建议：

- 这是一个较复杂的参数，官方认为通常不需要偏离默认值太多。

调小后的现象：

- 更敢猛打控制
- 转向和侧移更激进
- 终点附近容易抽动

调大后的现象：

- 更顺滑
- 不容易急拐
- 但过大可能显得跟不上弯

经验：

- 如果轨迹不是“偏”，而是“抖”，优先考虑调 `gamma`

### 3. 速度与采样范围

#### `vx_max / vy_max / wz_max`

作用：

- MPPI 采样空间的硬边界

官方建议：

- 这组参数经常应作为最优先检查对象之一。

重要提醒：

- 这不只是“车能跑多快”
- 还决定 MPPI 会不会去考虑很多非常激进的轨迹

调大后的现象：

- rollout 更分散
- 转角可能更爱 shortcut
- 终点附近更容易 overshoot

调小后的现象：

- 更稳
- 更收敛
- 但极限机动会下降

经验：

- loopback 里若可视化发散，先别急着怪 critic，先看速度上限是不是设得像实车一样大

#### `vx_std / vy_std / wz_std`

作用：

- 每周期在当前控制序列附近的采样扰动大小

官方定义：

- 对应 Vx / Vy / Wz 的高斯采样标准差。

调大后的现象：

- rollout 看起来像扇形散开
- 容易跳出局部最优
- 但也更容易左右试探

调小后的现象：

- rollout 更集中
- 可视化更好看
- 但可能在复杂场景过于保守

经验：

- `trajectories` 乱飘时，优先降低 std
- 真要提速度，先提 `GoalCritic` 或 `PathFollowCritic`，不要只提 std

### 4. 路径相关 critic

#### `PathAlignCritic`

作用：

- 惩罚轨迹偏离路径中心线

关键参数：

- `cost_weight`
- `threshold_to_consider`
- `offset_from_furthest`

官方定义：

- 这是“路径对齐” critic，不是“路径跟随” critic。

`cost_weight` 调大：

- 更贴着局部路径走
- 但转角处会更僵，像被路径拽住

`threshold_to_consider` 调大：

- 更早停止强制贴线，终点附近更容易由目标 critic 接管

`offset_from_furthest` 调大：

- 看得更远
- 更爱切弯

经验：

- 如果路径明明很顺，但 `trajectories` 老是左右纠偏，先减 `PathAlignCritic.cost_weight`

#### `PathFollowCritic`

作用：

- 鼓励沿路径整体向前推进
- 比 PathAlign 更柔和，不是死贴中心线

官方定义：

- 这是“路径跟随” critic，PathAlign 则偏向“路径对齐”。

调大后的现象：

- 走得更果断
- 速度感更强
- 但可能开始 shortcut

经验：

- 想让车“顺着路往前走得更积极”，优先调它
- 想让车“贴线更准”，优先调 PathAlign

#### `PathAngleCritic`

作用：

- 惩罚轨迹朝向与路径前进方向严重不一致

官方定义：

- 用于极端失配或转向情形下的路径朝向一致性约束。

关键参数：

- `cost_weight`
- `max_angle_to_furthest`
- `forward_preference`

调大后的现象：

- 转角处更不容易横着走或反向试探
- 但过大时拐角会发僵

`max_angle_to_furthest` 调小：

- 对朝向偏差更严格

经验：

- 如果你看到的是“在角点前后总想反着来一下”，这项通常值得先调

### 5. 目标相关 critic

#### `GoalCritic`

作用：

- 惩罚轨迹末端距离目标点太远

官方定义：

- 这是“朝目标位置收敛”的核心 critic。

这项是终点收敛的核心参数。

调大后的现象：

- 更快把注意力从“贴路径”切到“贴终点”
- 终点附近更不容易蛇形

调小后的现象：

- 更可能一路死跟路径
- 终点附近容易继续被路径项拉着左右修正

`threshold_to_consider` 调大后的现象：

- 更早进入“目标主导”阶段

经验：

- 临近终点还在左右摆，优先看它

#### `GoalAngleCritic`

作用：

- 惩罚终点附近朝向误差

官方定义：

- 用于在接近目标时鼓励达到目标姿态。

如果任务不关心最终朝向：

- 权重可适中
- `symmetric_yaw_tolerance: true` 更合适

如果终点附近老转圈：

- 先看它和 `TwirlingCritic`

### 6. 障碍物与约束相关 critic

#### `ObstaclesCritic`

作用：

- 让 MPPI 躲开代价地图中的高代价区域

官方建议：

- `repulsion_weight` 的调节应结合 inflation layer 半径一起考虑。
- `inflation_radius` 与 `cost_scaling_factor` 在 Humble 下应与 inflation layer 保持一致。

常看参数：

- `repulsion_weight`
- `critical_weight`
- `collision_margin_distance`
- `near_goal_distance`

`repulsion_weight` 调大后的现象：

- 更早躲障
- 也更容易被障碍代价场顶离路径

`near_goal_distance` 调大后的现象：

- 终点附近更容易忽略轻微障碍代价影响
- 有助于“最后贴点”

经验：

- 如果终点旁边没有真实障碍，但车像被“看不见的墙”推开，优先看这组参数和 inflation 配置是否匹配

## 全局层和局部层不要用同一种调参思路

很多人调窄路问题时会把 global / local inflation 一起往下压，这很容易把问题从“过不去”变成“能过去但贴墙撞边”。

建议把这两层分开理解：

### global costmap inflation

职责：

- 决定全局规划更愿意走通道中心还是沿墙走

优先调这些参数：

- `global_costmap.inflation_layer.inflation_radius`
- `global_costmap.inflation_layer.cost_scaling_factor`
- `planner_server.GridBased.cost_travel_multiplier`
- `planner_server.GridBased.cost_penalty`

经验：

- 全局层通常应该比局部层更保守
- 如果全局路径太激进、容易贴墙，先提 `cost_travel_multiplier`
- 再提 `cost_penalty`
- 如果还不够，再把 global inflation 略放大

### local costmap + MPPI obstacle critic

职责：

- 决定局部控制敢不敢从狭窄通道通过
- 决定 MPPI 是被代价场推回去，还是能稳定贴着中心穿过去

优先调这些参数：

- `local_costmap.inflation_layer.inflation_radius`
- `local_costmap.inflation_layer.cost_scaling_factor`
- `FollowPath.ObstaclesCritic.repulsion_weight`
- `FollowPath.ObstaclesCritic.inflation_radius`
- `FollowPath.ObstaclesCritic.cost_scaling_factor`

经验：

- `ObstaclesCritic.inflation_radius / cost_scaling_factor` 在 Humble 下应和 local inflation 保持一致
- 局部层缩得太小，虽然过窄路会更容易，但机器人会更喜欢贴边
- 如果已经能过窄路，但出现刮墙趋势，优先略增 `repulsion_weight`

## 现象 6：全局路径能过窄路，但越来越贴墙

优先检查：

1. `planner_server.GridBased.cost_travel_multiplier`
2. `planner_server.GridBased.cost_penalty`
3. `global_costmap.inflation_layer.*`

推荐方向：

- 提高 `cost_travel_multiplier`
- 再提高 `cost_penalty`
- 保持 global inflation 稍微大于 local inflation

## 现象 7：转角前后反向试探，切换巡逻点时更明显

优先检查：

1. `PathAngleCritic.cost_weight`
2. `PathAngleCritic.max_angle_to_furthest`
3. `PathAngleCritic.forward_preference`
4. `PathAlignCritic.cost_weight`
5. `PathAlignCritic.offset_from_furthest`

推荐方向：

- 增大 `PathAngleCritic.cost_weight`
- 适当减小 `max_angle_to_furthest`
- 对全向底盘也可打开 `forward_preference`，减少“先反一下再转”的试探
- 如果还被路径强拽着走，再降低 `PathAlignCritic.cost_weight`
- 再缩短 `offset_from_furthest`

## 现象 8：快到终点或刚切到下一个巡逻点时局部预测偏离

优先检查：

1. `GoalCritic.cost_weight`
2. `GoalCritic.threshold_to_consider`
3. `xy_goal_tolerance`
4. `VelocityDeadbandCritic`
5. `time_steps * model_dt`

推荐方向：

- 让 `GoalCritic` 更早接管
- 提高 `GoalCritic.cost_weight`
- 略放宽 `xy_goal_tolerance`
- 如果预测范围明显长过头，优先缩短 horizon，而不是只改 critic

## 卡在膨胀层里的脱困如何理解

这类问题不只靠 MPPI 正常跟踪能解决，恢复链也要一起看。

当前项目已经额外做了一层处理：

- 恢复插件使用 `pb_nav2_behaviors/BackUpFreeSpace`
- 它会读取 costmap，搜索低代价退让方向
- 新增了 `max_allowed_cost`
- 高于该阈值的膨胀层区域，不再被当作“可退空间”

这样做的原因是：

- 如果恢复动作把高 cost 的膨胀区也当安全区
- 机器人会在“看似能退、其实越退越贴墙”的方向上来回试探

所以调脱困时要一起看：

- `behavior_server.max_allowed_cost`
- BT 中 `BackUp.backup_dist`
- BT 中 `BackUp.backup_speed`
- `progress_checker.required_movement_radius`
- `progress_checker.movement_time_allowance`

## 外参错了时，看起来很像 MPPI 参数没调好

如果你遇到这些现象：

- 局部障碍整体像斜着摆
- `trajectories` 在空旷处也被无形推开
- 仿真和实车同一套参数表现差异特别大

优先检查传感器外参，不要急着继续改 MPPI。

当前项目中要特别注意：

- 实车模型 `pb2025_sentry_robot.sdf.xmacro`
  - Livox 位姿：`0.1 0.245 0.3 ${68*pi/180} 0 -${161*pi/180}`
- 仿真模型 `simulation_robot.sdf.xmacro`
  - 应与实车保持同一位姿
- 导航链路默认仍使用：
  - `lidar_frame: front_mid360`
  - `robot_base_frame: gimbal_yaw_odom`

而 `sensor_scan_generation`、`loam_interface`、costmap `sensor_frame` 依赖的是运行时 TF 结果。

所以真正要确认的是：

1. 机器人描述里的外参是否改了
2. 仿真模型是否同步改了
3. 运行时 TF 是否真的是新的安装位姿

## loopback 特有的两个“假故障源”

这两项不是 MPPI 参数本身，但它们会非常像 MPPI 没调好：

### 1. 反复发布 initialpose

如果 loopback 在导航进行中不断重发 `initialpose`，`nav2_loopback_sim` 会持续重算 `map->odom`。

表现出来就是：

- RViz 里局部路径、rollout、costmap 参考关系像在轻微漂移
- 你以为是 MPPI rollout 发散
- 实际上是仿真定位原点在反复被改写

当前项目已把假输入节点默认 `initial_pose_repeats` 调整为 `1`，只在启动时给一次初始位姿。

### 2. Omni 控制器和速度平滑器侧移约束不一致

如果 MPPI `motion_model=Omni`，但 `velocity_smoother` 又把 `linear.y` 限成 `0`，就会出现：

- 控制器采样认为自己可以侧移
- 底层执行却不允许侧移
- 转角、终点、切换巡逻点时更容易出现反向试探和预测偏离

所以 loopback 里必须保证：

- `FollowPath.motion_model: Omni`
- `velocity_smoother.max_velocity[1] / min_velocity[1]`
  - 不要被锁成 `0`

#### `ConstraintCritic`

作用：

- 惩罚超出运动学约束、加速度约束的轨迹

官方定义：

- 用于惩罚超出动态或运动学约束的轨迹。

通常：

- 保持中等权重即可
- 不建议作为第一调参入口

#### `TwirlingCritic`

作用：

- 抑制原地乱转

官方定义：

- 用于惩罚不必要的旋转行为。

现象：

- 终点附近打圈明显时，可以适当增大

#### `VelocityDeadbandCritic`

作用：

- 惩罚特别小但不为零的控制量

官方定义：

- 用于惩罚速度死区附近的小幅控制输出。

现象：

- 它很适合解决“明明快停了还在轻微抽搐”

## 官方建议优先级

结合 Nav2 官方文档，建议优先检查顺序可以总结为：

1. `motion_model` 是否与底盘一致
2. `vx_max / vy_max / wz_max / vx_min` 等速度边界是否合理
3. `model_dt` 是否与控制频率匹配
4. `time_steps * model_dt` 的预测时域是否过长
5. `ObstaclesCritic` 是否与 inflation layer 参数一致
6. 在以上合理后，再细调路径类和目标类 critic

## 现象到参数的快速映射

### 现象 1：`trajectories` 扇形乱飘

优先检查：

1. `vx_std / vy_std / wz_std` 是否过大
2. `vx_max / vy_max / wz_max` 是否设得过激
3. `time_steps * model_dt` 是否明显大于 local costmap 可覆盖范围

推荐方向：

- 降低 std
- 缩短 horizon
- 减小最大速度
- 适当放大 local costmap

### 现象 2：转角前后左右试探，甚至反向抽一下

优先检查：

1. `PathAngleCritic.cost_weight`
2. `PathAngleCritic.max_angle_to_furthest`
3. `PathAlignCritic.cost_weight`
4. 巡逻点切换逻辑本身是否在频繁 preempt

推荐方向：

- 略增 `PathAngleCritic`
- 略减 `PathAlignCritic`
- 避免一次发多点 through poses 导致角点切换抖动

### 现象 3：快到终点时蛇形摆动

优先检查：

1. `GoalCritic.cost_weight`
2. `GoalCritic.threshold_to_consider`
3. `xy_goal_tolerance`
4. `VelocityDeadbandCritic`

推荐方向：

- 增大 `GoalCritic`
- 让 GoalCritic 更早接管
- 略放宽 `xy_goal_tolerance`
- 保持适度 deadband 惩罚

### 现象 4：贴路径太死，遇到拐角很僵

优先检查：

1. `PathAlignCritic.cost_weight`
2. `PathAlignCritic.offset_from_furthest`
3. `temperature`

推荐方向：

- 降低 PathAlign 权重
- 缩短对前方路径的执念
- 略增 temperature

### 现象 5：速度太保守，虽然稳但很慢

优先检查：

1. `/trajectory_profile.max_abs_curvature`
2. `/trajectory_profile.curvature_penalty`
3. `/cmd_vel_controller`
4. `/cmd_vel_controller_governed`
5. `/cmd_vel_nav2_result`
6. `PathFollowCritic.cost_weight`
7. `GoalCritic.cost_weight`
8. `vx_max / vy_max`

推荐方向：

- 如果 `max_abs_curvature` 很大且 `curvature_penalty` 主导，先处理 planner / smoother 的轨迹形状。
- 如果 `/cmd_vel_controller` 已经很小，优先看 MPPI critic、目标距离、局部 costmap 和 TF / odom。
- 如果 `/cmd_vel_controller` 正常但 `/cmd_vel_controller_governed` 明显变小，看 `trajectory_speed_governor.curvature_window_points` 和 `curvature_brake_gain`。
- 如果 governed 正常但 `/cmd_vel_nav2_result` 变小，看 `velocity_smoother` 的速度、加速度、timeout 和 deadband。
- 确认前面都正常后，再考虑提高 `PathFollowCritic`、`GoalCritic` 或扩大速度上限和 std。

## 推荐调参流程

每次只改一组，跑一轮 loopback：

1. 固定地图与巡逻点
2. 只改 1~2 个参数
3. 观察 `trajectories` 形状是否变化
4. 记录“变好还是变坏”
5. 再决定下一轮

建议记录格式：

```text
轮次:
改动:
现象:
结论:
```

## 当前 loopback 参数的调参意图

当前 `src/loopback_sim/params/nav2_params.yaml` 的思路是：

- 缩短预测时域
  - 减少“想太远”导致的终点和转角发散
- 降低采样扰动
  - 让 rollout 更集中，更容易看清真实趋势
- 提前让 GoalCritic 接管
  - 避免末端继续被路径项拽着左右修正
- 保证 local costmap 覆盖主要预测范围
  - 防止 rollout 总顶到局部地图边缘

这套参数不是“最终答案”，而是一套更适合继续迭代观察的基线。

当前实车 `src/pb2025_sentry_bringup/params/node_params.yaml` 的速度基线更激进：

- `lateral_accel_limit` / `longitudinal_accel_limit` 已提高到 `1.8`
- `velocity_smoother.max_velocity` 为 `[4.5, 4.5, 5.0]`
- `trajectory_speed_governor` 使用近端曲率窗口，而不是整条路径最大曲率
- signed Traversability ESDF 已接入实车 optimizer 和 smoother，但暂不建议继续增强 obstacle 强度

## 与其他文档的关系

可配合阅读：

- `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`
  - 记录从当前 Nav2 主链迁移到 3D/2.5D ESDF + JPS + MINCO + MPC 的路线
- `docs/omni_recovery_smoothing_optimization.md`
  - 记录当前平滑、ESDF、trajectory profile 和速度链基线
- `docs/navigate_through_poses_migration_checklist.md`
  - 记录 `NavigateThroughPoses` 迁移和旧决策节点关系
- `docs/slim_loopback_refactor.md`
  - 记录 loopback 精简和重构过程

## 一句话经验

MPPI 调参时：

- 先调 horizon 和采样范围
- 再调路径 critic 和目标 critic
- 不要上来只改 std
- 也不要上来只改某个 weight

看 `trajectories` 的形状，往往比单看机器人跑得快不快更有信息量。
