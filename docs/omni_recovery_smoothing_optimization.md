# 全向舵轮脱困与避障平滑优化说明

## 1. 问题背景

当前项目中的恢复行为主要由：

- [src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp](../src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp)

负责。

原实现的核心问题是：

1. 每个控制周期只发一个固定方向的小步速度
2. 紧接着立刻用单步前向碰撞检测判断是否还能继续
3. 一旦局部 costmap 或膨胀层边界有轻微变化，就立即停下 / 失败 / 切恢复状态

这在全向舵轮底盘上会放大成三个问题：

1. 底盘表现为“挪一点、停一下、再挪一点”
2. 电机与舵轮不断经历高频启停和方向突变，容易抖动、发热、磨损
3. 正常导航 / 避障 / 脱困状态在边界附近频繁跳变，整体控制观感卡顿

---

## 2. 优化目标

本次优化不是简单调参数，而是重新设计恢复动作的局部控制策略，目标是：

1. 提高脱困成功率
2. 让全向舵轮底盘在恢复和避障时保持连续、平滑
3. 降低电机与舵轮机构的高频冲击
4. 避免正常 / 避障 / 脱困之间的抖动切换

---

## 3. 新算法设计

### 3.1 从“逐小步试探”改为“恢复轨迹规划 + 批量采样检测”

原逻辑是：

1. 找一个方向
2. 每拍沿这个方向发一个速度
3. 每拍只检查很近的一点是否碰撞

新逻辑改为：

1. 先基于当前 costmap 规划一条短时恢复轨迹
2. 轨迹不是单点，而是一条带宽度的“通行走廊”
3. 对整条走廊做批量采样检测
4. 选择平均代价更低、贴近车尾主后退方向、且与上一次方向更连续的恢复方向

这样做的好处：

1. 不再因为单个局部采样点抖动而频繁停启
2. 恢复动作对膨胀层边界和动态障碍扰动更稳
3. 更符合全向舵轮“可以向斜后方 / 侧后方柔性退让”的运动优势

对应代码：

- [back_up_free_space.cpp](../src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp)
  中的 `planEscapeTrajectory()`
- `evaluateCandidateTrajectory()`
- `sampleCost()`

### 3.2 面向全向舵轮的恢复方向搜索

全向舵轮不应该只会“纯 x 轴后退”。

本次恢复方向搜索以机器人车尾方向为中心，在一个可配置的角域内搜索：

- `search_half_span_deg`
- `search_angle_increment_deg`

优先搜索后向和后侧向的连续可行方向，必要时再放开到全角域：

- `enable_full_circle_fallback`

这样做的原因：

1. 全向舵轮在狭窄区域常常“正后退不通，但斜后退可通”
2. 若仍坚持刚性纯后退，会反复卡在膨胀层边缘
3. 斜后退对脱离动态障碍遮挡也更高效

### 3.3 速度输出改为一阶低通 + 加减速限幅

原逻辑直接把目标速度一步跳到输出：

1. 本拍 `0`
2. 下一拍 `0.25`
3. 再下一拍可能又变成 `0`

这正是电机抖动和舵轮机械冲击的重要来源。

新逻辑在恢复动作内部加入两层平滑：

1. 一阶低通滤波
2. 每轴加减速限幅

对应参数：

- `speed_filter_tau`
- `translational_acc_limit`
- `translational_decel_limit`
- `minimum_speed_xy`

对应代码：

- `buildDesiredCommand()`
- `smoothCommand()`

这样做的好处：

1. 速度不再突变
2. 斜向恢复时 `vx / vy` 会连续过渡
3. 对舵轮转向执行器和驱动电机更友好
4. 明显减少“卡顿 + 抖一下”的观感

### 3.4 加入状态滞回，抑制频繁切状态

原实现只有“当前可走 / 当前不可走”的瞬时判断，没有滞回。

所以只要障碍边界或膨胀层有一点抖动，就可能出现：

1. 一拍可走
2. 下一拍不可走
3. 再下一拍又可走

这会直接导致：

1. 恢复行为反复停启
2. BT 在恢复分支里表现得像“抽搐”
3. 上层看起来像导航 / 脱困状态在抢控制权

新逻辑引入恢复内部状态机：

- `PLANNING`
- `EXECUTING`
- `BLOCKED`

并加入滞回参数：

- `blocked_enter_cycles`
- `clear_exit_cycles`
- `replanning_cooldown_s`
- `max_replan_attempts`

语义：

1. 连续若干拍都检测到轨迹前缀被阻挡，才进入 `BLOCKED`
2. 连续若干拍都恢复清空，才退出 `BLOCKED`
3. 进入 `BLOCKED` 后不会立刻每拍都重规划，而是有冷却时间

对应代码：

- `onCycleUpdate()`
- `replanFromCurrentPose()`

这样做的好处：

1. 避免边界噪声触发高频切状态
2. 让恢复动作更像“连续控制过程”而不是“离散开关”
3. 对动态障碍物穿行和局部 costmap 抖动更稳

### 3.5 前向批量 lookahead 监控，而不是每步立刻失败

本次新增：

- `monitor_lookahead_distance`

在恢复执行阶段，不是只检查“眼前那一步”，而是对当前恢复方向前方一小段距离做连续批量检查。

对应代码：

- `isTrajectoryPrefixSafe()`

这样做的好处：

1. 更早感知恢复轨迹前缀是否被动态障碍重新封堵
2. 不会因为脚下单个格点 cost 抖动而立即失败
3. 更接近连续运动控制，而不是栅格级振荡

---

## 4. 为什么这样改能解决卡顿

卡顿的本质，不是“底盘动力不够”，而是控制策略过于离散：

1. 检测离散
2. 状态切换离散
3. 速度输出离散

### 4.1 旧实现为什么卡

旧实现中：

1. 每拍输出固定速度
2. 每拍立即做短距离碰撞检测
3. 单次检测失败就立刻停

于是控制链会变成：

```text
发速度 -> 移动一点 -> 检测 -> 停 -> 再发速度 -> 再停
```

这在全向舵轮底盘上会表现为：

1. `vx / vy` 一直跳变
2. 舵轮转角不断修正
3. 电机频繁启停

### 4.2 新实现为什么更稳

新实现把整个恢复链改成：

```text
先选一条短时恢复轨迹
-> 连续监控轨迹前缀是否仍可走
-> 在可走时持续执行
-> 速度经过低通滤波和加减速约束
-> 只有连续阻塞后才重规划
```

因此：

1. 恢复方向不会每拍乱跳
2. 输出速度不会每拍突变
3. 状态不会在阈值边界附近来回抽动

最终效果就是：

1. 脱困动作更连续
2. 避障动作更平滑
3. 舵轮电机寿命更友好

---

## 5. 本次具体修改的代码文件

### 5.1 核心恢复行为重构

- [src/pb2025_sentry_nav/pb_nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp](../src/pb2025_sentry_nav/pb_nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp)
- [src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp](../src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp)

新增的核心能力：

1. 恢复轨迹规划 `planEscapeTrajectory`
2. 候选轨迹批量采样 `evaluateCandidateTrajectory`
3. 速度平滑 `smoothCommand`
4. 轨迹前缀监控 `isTrajectoryPrefixSafe`
5. 恢复状态滞回 `PLANNING / EXECUTING / BLOCKED`
6. 被阻挡后的冷却重规划 `replanFromCurrentPose`

### 5.2 参数扩展

已同步更新：

- [src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml](../src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)
- [src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml](../src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml)
- [src/pb2025_sentry_bringup/params/node_params.yaml](../src/pb2025_sentry_bringup/params/node_params.yaml)

新增参数分组：

1. 方向搜索参数
2. 轨迹采样参数
3. 状态滞回参数
4. 速度滤波参数
5. 轨迹监控参数

---

## 6. 关键参数理解

### 6.1 搜索与轨迹参数

- `search_half_span_deg`
  以车尾为中心搜索恢复方向的半角范围

- `search_angle_increment_deg`
  候选方向离散步长

- `trajectory_sample_step`
  沿恢复轨迹前进方向的采样步长

- `corridor_half_width`
  轨迹走廊半宽，反映底盘横向占用

- `corridor_lateral_step`
  轨迹走廊横向采样分辨率

### 6.2 平滑与执行参数

- `speed_filter_tau`
  一阶低通时间常数，越大越平滑，响应越慢

- `translational_acc_limit`
  平移加速度上限

- `translational_decel_limit`
  平移减速度上限

- `minimum_speed_xy`
  为避免恢复末段由于限速太小导致底盘发抖，保留一个最小平移速度

### 6.3 状态滞回参数

- `blocked_enter_cycles`
  连续多少拍阻挡才认定真正 blocked

- `clear_exit_cycles`
  连续多少拍通畅才从 blocked 恢复执行

- `replanning_cooldown_s`
  两次重规划之间的最小间隔

- `max_replan_attempts`
  允许的最大重规划次数

---

## 7. 建议的实车调参顺序

建议按这个顺序调：

1. 先固定 `backup_dist` 和 `backup_speed`
2. 调 `max_allowed_cost`
3. 调 `corridor_half_width`
4. 调 `speed_filter_tau`
5. 调 `blocked_enter_cycles / clear_exit_cycles`
6. 最后再调 `search_half_span_deg`

推荐经验：

1. 如果还是抖：
   - 先增大 `speed_filter_tau`
   - 再增大 `blocked_enter_cycles`
2. 如果恢复太慢：
   - 先减小 `speed_filter_tau`
   - 再减小 `replanning_cooldown_s`
3. 如果仍然容易贴墙来回磨：
   - 适当降低 `max_allowed_cost`
   - 适当增大 `corridor_half_width`

---

## 8. 本次优化的适用边界

这次优化重点作用于：

1. 膨胀层边界卡住
2. 动态障碍导致恢复方向短时被封
3. 全向舵轮在狭窄空间中的侧后退让

它不是用来替代：

1. 全局路径规划
2. MPPI 主控制器
3. 正常导航过程中的高层策略判断

换句话说：

这次改的是“恢复动作的局部执行质量”，不是把整个 Nav2 控制器替换掉。

---

## 9. 已完成的验证

已完成：

1. `pb_nav2_plugins` 定点编译通过

建议后续实车重点观察：

1. `/cmd_vel`
2. `back_up_free_space_markers`
3. 恢复过程中底盘是否仍有高频启停
4. 卡在膨胀层边界时是否能连续斜后退脱离

---

## 10. 实车调参速查表

这一节专门给现场调试使用。

建议调参顺序始终遵守一条原则：

1. 先解决“抖不抖”
2. 再解决“脱不脱得出来”
3. 最后再解决“脱困是否足够快”

不要一开始就只追求恢复动作更快，否则很容易重新把电机抖动和边界抽动带回来。

### 10.1 现象：底盘恢复时还是一顿一顿，电机有高频抖动

优先调整：

- `speed_filter_tau`
- `translational_acc_limit`
- `blocked_enter_cycles`

建议方向：

1. 先增大 `speed_filter_tau`
   - 例如从 `0.18 -> 0.22 / 0.26`
   - 效果：速度更平滑，但响应会慢一点
2. 再减小 `translational_acc_limit`
   - 例如从 `0.8 -> 0.6`
   - 效果：起步更柔和，电机负担更小
3. 若仍有“走一下停一下”的感觉，再增大 `blocked_enter_cycles`
   - 例如从 `3 -> 4`
   - 效果：不会因为 1~2 拍的局部障碍抖动就立刻进入阻塞态

不建议先动：

- `max_allowed_cost`
- `search_half_span_deg`

因为这两个主要影响“往哪退”，不是“退得顺不顺”。

### 10.2 现象：恢复方向左右来回换，像在犹豫

优先调整：

- `heading_stickiness_weight`
- `replanning_cooldown_s`
- `clear_exit_cycles`

建议方向：

1. 增大 `heading_stickiness_weight`
   - 例如 `6.0 -> 8.0`
   - 效果：更愿意保持上一条恢复方向
2. 增大 `replanning_cooldown_s`
   - 例如 `0.35 -> 0.45`
   - 效果：减少短时间内连续重规划
3. 增大 `clear_exit_cycles`
   - 例如 `2 -> 3`
   - 效果：从 `BLOCKED` 回到 `EXECUTING` 更稳，不会刚清一点又马上切回来

### 10.3 现象：恢复很稳，但是脱困太慢

优先调整：

- `speed_filter_tau`
- `translational_acc_limit`
- `monitor_lookahead_distance`

建议方向：

1. 适当减小 `speed_filter_tau`
   - 例如 `0.18 -> 0.14`
   - 效果：响应更快，但不要一次减太多
2. 适当增大 `translational_acc_limit`
   - 例如 `0.8 -> 1.0`
   - 效果：起步更果断
3. 若感觉太早因为前方风险停下，可以小幅减小 `monitor_lookahead_distance`
   - 例如 `0.35 -> 0.28`
   - 效果：动作更激进

注意：

1. 一次只改一个主参数
2. 每次调整后至少重复测试 3 次
3. 如果变快的同时又开始抖，就回退上一档

### 10.4 现象：底盘总是贴着墙边或膨胀层边缘磨，不愿意真正退开

优先调整：

- `max_allowed_cost`
- `corridor_half_width`
- `trajectory_sample_step`

建议方向：

1. 先减小 `max_allowed_cost`
   - 例如 `96 -> 88`
   - 效果：恢复动作更不愿意走高代价边缘区域
2. 增大 `corridor_half_width`
   - 例如 `0.22 -> 0.26`
   - 效果：把底盘看得更“胖”，轨迹会自动更保守
3. 若地图分辨率足够高，可减小 `trajectory_sample_step`
   - 例如 `0.08 -> 0.06`
   - 效果：更早发现贴边风险

### 10.5 现象：正后方不通时，机器人还是不愿意侧后退

优先调整：

- `search_half_span_deg`
- `search_angle_increment_deg`
- `enable_full_circle_fallback`

建议方向：

1. 增大 `search_half_span_deg`
   - 例如 `140 -> 160`
   - 效果：允许搜索更靠近侧向的恢复方向
2. 减小 `search_angle_increment_deg`
   - 例如 `10 -> 6`
   - 效果：更容易找到狭窄但真实可走的方向
3. 确认 `enable_full_circle_fallback=true`
   - 若后向和侧后向都找不到，允许全角域兜底搜索

### 10.6 现象：动态障碍一靠近，恢复动作就频繁停掉

优先调整：

- `blocked_enter_cycles`
- `replanning_cooldown_s`
- `monitor_lookahead_distance`

建议方向：

1. 增大 `blocked_enter_cycles`
   - 例如 `3 -> 5`
   - 效果：短时动态遮挡不会立刻触发阻塞
2. 增大 `replanning_cooldown_s`
   - 例如 `0.35 -> 0.5`
   - 效果：不给动态障碍抖动每拍触发一次重规划
3. 适当减小 `monitor_lookahead_distance`
   - 例如 `0.35 -> 0.25`
   - 效果：降低“过早把远处短时动态障碍当成眼前阻挡”的概率

### 10.7 现象：恢复动作经常直接失败，重规划次数很快耗尽

优先调整：

- `max_replan_attempts`
- `search_half_span_deg`
- `max_allowed_cost`

建议方向：

1. 先增大 `max_replan_attempts`
   - 例如 `6 -> 8`
   - 效果：给恢复动作更多重新尝试空间
2. 再增大 `search_half_span_deg`
   - 效果：扩大候选方向搜索范围
3. 若环境确实比较挤，再略微增大 `max_allowed_cost`
   - 例如 `96 -> 104`
   - 效果：允许通过更高代价的膨胀边缘

但注意：

1. `max_allowed_cost` 调太大后，可能重新出现贴墙和磨边
2. 这个参数宁可小步加，也不要一次加很多

### 10.8 现象：快到脱困终点时反复抖，不像真正停住

优先调整：

- `goal_tolerance`
- `minimum_speed_xy`
- `translational_decel_limit`

建议方向：

1. 增大 `goal_tolerance`
   - 例如 `0.04 -> 0.06`
   - 效果：更早判定恢复到位，减少末端反复修正
2. 若末端还是有“将停未停”的粘滞感，可略增 `translational_decel_limit`
   - 例如 `1.2 -> 1.4`
3. 如果是极低速时来回抖，适当减小 `minimum_speed_xy`
   - 例如 `0.08 -> 0.06`

### 10.9 一组推荐调试流程

现场建议这样做：

1. 第一轮只看是否抖动
   - 重点改：`speed_filter_tau`、`translational_acc_limit`
2. 第二轮看是否能稳定脱离膨胀层
   - 重点改：`max_allowed_cost`、`corridor_half_width`
3. 第三轮看是否能灵活利用全向底盘斜退
   - 重点改：`search_half_span_deg`、`search_angle_increment_deg`
4. 第四轮看动态障碍下是否还会抽动
   - 重点改：`blocked_enter_cycles`、`replanning_cooldown_s`

---

## 11. 推荐记录方式

每次实车调试时建议记录四项：

1. 当前参数改了什么
2. 触发恢复的场景是什么
3. 现象变好了还是变坏了
4. 是否出现新的副作用

推荐用下面这种格式：

```md
日期：
场景：
修改参数：
现象改善：
副作用：
最终是否保留：
```

这样你后面回看时，不会只记得“好像那天调过”，但不知道到底是哪一个参数起作用。
