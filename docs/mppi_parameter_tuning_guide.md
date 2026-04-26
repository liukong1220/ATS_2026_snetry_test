# MPPI 参数调试说明

## 适用范围

本文主要说明当前项目中 MPPI 局部控制器的核心参数含义、常见现象与调参方向，优先针对：

- `ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True`
- 参数文件：`src/loopback_sim/params/nav2_params.yaml`

这套说明也可以迁移到：

- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`
- `src/pb2025_sentry_bringup/params/node_params.yaml`

但建议先在 loopback 中把现象调顺，再同步到正式仿真和实车。

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

### 1. Goal Checker

#### `xy_goal_tolerance`

作用：

- 终点位置容差，决定 Nav2 什么时候判定“到点”

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

一般不优先动它，除非整套控制频率都要重构。

#### `batch_size`

作用：

- 每周期采样多少条候选轨迹

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

如果任务不关心最终朝向：

- 权重可适中
- `symmetric_yaw_tolerance: true` 更合适

如果终点附近老转圈：

- 先看它和 `TwirlingCritic`

### 6. 障碍物与约束相关 critic

#### `ObstaclesCritic`

作用：

- 让 MPPI 躲开代价地图中的高代价区域

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

#### `ConstraintCritic`

作用：

- 惩罚超出运动学约束、加速度约束的轨迹

通常：

- 保持中等权重即可
- 不建议作为第一调参入口

#### `TwirlingCritic`

作用：

- 抑制原地乱转

现象：

- 终点附近打圈明显时，可以适当增大

#### `VelocityDeadbandCritic`

作用：

- 惩罚特别小但不为零的控制量

现象：

- 它很适合解决“明明快停了还在轻微抽搐”

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

1. `PathFollowCritic.cost_weight`
2. `GoalCritic.cost_weight`
3. `vx_max / vy_max`

推荐方向：

- 先略增 `PathFollowCritic`
- 再看是否需要提高 `GoalCritic`
- 最后才考虑扩大速度上限和 std

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

## 与其他文档的关系

可配合阅读：

- `docs/mppi_local_plan_fix.md`
  - 记录 MPPI 可视化话题和迁移问题
- `docs/slim_loopback_refactor.md`
  - 记录 loopback 精简和重构过程

## 一句话经验

MPPI 调参时：

- 先调 horizon 和采样范围
- 再调路径 critic 和目标 critic
- 不要上来只改 std
- 也不要上来只改某个 weight

看 `trajectories` 的形状，往往比单看机器人跑得快不快更有信息量。
