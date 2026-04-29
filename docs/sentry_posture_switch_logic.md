# 哨兵姿态转换逻辑说明

本文档面向后续维护者，详细说明当前仓库中“姿态切换 + 受击自旋 + 下位机模式发送”的完整链路。  
涉及代码主要在：

- `src/pb2025_sentry_behavior`
- `src/standard_robot_pp_ros2`
- `src/pb2025_sentry_bringup`

如果后续行为树、串口协议或裁判系统接线发生调整，请优先同步更新本文档。

## 1. 这套逻辑解决什么问题

当前实现要同时满足四个目标：

1. 让哨兵在不同决策阶段明确告诉下位机“现在是什么姿态”。
2. 避免姿态在高频 tick 的行为树里来回抖动。
3. 避免单局比赛里某一种姿态无限占用，违反比赛规则。
4. 只在“真实受击”时触发底盘自旋，并在短时间未继续掉血后自动停转。

因此，代码里把这个需求拆成了两条独立但协同的链路：

1. `姿态模式链路`
   行为树分支决定想要的姿态，`PublishRobotMode` 统一裁决后发布给下位机。
2. `受击自旋链路`
   `IsAttacked` 负责识别是否发生了有效受击，行为树再决定是否发布自旋角速度。

## 2. 总体数据流

### 2.1 姿态模式数据流

```text
行为树分支
  -> PublishRobotMode(mode=move/attack/defend)
  -> 读取黑板中的比赛状态、冷却时间、累计时长限制
  -> resolveModeWithConstraints() 做最终姿态裁决
  -> 发布到 decision/robot_mode
  -> standard_robot_pp_ros2 订阅该话题
  -> 写入 SendRobotCmdData.data.speed_vector.mode
  -> 串口发送给下位机
```

### 2.2 受击自旋数据流

```text
裁判系统 RobotStatus
  -> IsAttacked
  -> 判断是否是 ARMOR_HIT 且本次确实掉血
  -> 若命中则刷新最近一次受击时间
  -> 在一段可配置的持续时间内返回 SUCCESS
  -> 行为树发布 decision.motion.hit_spin_speed
  -> 若超时没有新掉血则回到 0.0
```

## 3. 姿态定义与下发

当前姿态枚举定义如下：

- `move = 0`
- `attack = 1`
- `defend = 2`

上层行为树统一发布 `example_interfaces/msg/UInt8` 到 `decision/robot_mode`。  
下位机接口层订阅这个话题后，将其写入：

```cpp
send_robot_cmd_data_.data.speed_vector.mode
```

对应的下位机串口结构体定义位于：

- `src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp`

其中枚举如下：

```cpp
enum mode
{
  Move = 0,
  Attack = 1,
  Defend = 2,
} mode;
```

为了避免后续维护者把“姿态”和“动作”混在一起，可以直接按下面理解：

| 姿态 | 数值 | 谁来触发 | 下位机应如何理解 |
| --- | --- | --- | --- |
| `move` | `0` | 常规巡逻、转点、锚点移动等普通机动分支 | 机器人处于正常机动态，不带防守或进攻姿态语义 |
| `attack` | `1` | 识别到敌方装甲板且视觉接管分支成立 | 机器人处于主动进攻态，可配合下位机做更激进的底盘/上装策略 |
| `defend` | `2` | 血量低于防御阈值，进入撤退/保守分支 | 机器人处于防御保命态，下位机可切换到保守机动/防御姿态 |

这里要特别强调两点：

1. `defend` 不是“被打了就立刻切换”，而是“当前血量已经低于防御阈值”。
2. 自旋不是姿态本身的一部分，而是受击事件触发的额外运动行为，所以文档后面把它单独拆出来说明。

## 4. 行为树里哪些分支会发布什么姿态

当前主树 `rmul_2026.xml` 中，姿态不是在根节点统一发布，而是跟随分支语义分别发布。

### 4.1 `attack`

视觉接管分支 `vision_override_realtime` 中会先判断目标是否合法，若合法则立即发布：

```xml
<PublishRobotMode mode="attack"
                  duration="0.0"
                  topic_name="{@decision_robot_mode_topic}"/>
```

语义是：

- 当前已经进入视觉接管
- 导航/云台都在围绕敌方目标工作
- 需要下位机切到进攻姿态

### 4.2 `move`

以下常规移动分支会发布 `move`：

- `decision_patrol`
- `decision_anchor_target`
- `decision_critical_time_target`

它们的共同语义是：

- 当前在执行正常巡逻或移动任务
- 不是低血量防守态
- 也不是视觉接管的主动进攻态

### 4.3 `defend`

以下分支会发布 `defend`：

- `decision_safe_point`
- `decision_retreat`

其中最关键的触发入口是：

```xml
<IsRobotHpBelow threshold="{@decision_defend_mode_hp}"/>
```

也就是当：

```text
current_hp <= decision.mode_thresholds.defend_hp
```

时，行为树会进入低血量分支，并切换为防御姿态。

如果后续你要新增“半防御”“警戒”“补给”等姿态，建议先确认三件事再动代码：

1. 这是不是一个真正需要发给下位机的离散模式。
2. 它是否也需要冷却时间与单局累计时长限制。
3. 它是新增行为树分支，还是只是现有分支里的另一种运动参数。

## 5. 姿态切换为什么还要经过统一裁决

行为树每个 tick 都可能重新评估分支。  
如果每次分支一变就直接向下位机发模式，会有三个明显问题：

1. 模式可能在 `attack / move / defend` 之间频繁抖动。
2. 某姿态可能累计时间超规则后还在继续使用。
3. 不同分支之间可能同时“想切换”，导致下位机感知不稳定。

所以项目里新增了 `PublishRobotModeAction`，专门把“分支想要的模式”转成“最终允许下发的模式”。

核心函数是：

```cpp
uint8_t PublishRobotModeAction::resolveModeWithConstraints(uint8_t requested_mode)
```

它做的事情不是“盲发 requested_mode”，而是按下面顺序进行判断：

1. 读取比赛状态，判断是否处于一局比赛的 `RUNNING` 阶段。
2. 维护本局三种姿态的累计时长。
3. 判断目标姿态是否已经达到累计上限。
4. 判断距离上一次切姿态是否还处于冷却时间。
5. 如果目标姿态不可用，则选择一个当前仍合法的回退姿态。
6. 最后才把最终结果发布到 `decision/robot_mode`。

如果你想快速理解这个函数，可以直接把它看成下面这段伪代码：

```cpp
resolved_mode = requested_mode;

更新当前局内累计时长;

if (当前 active_mode 已超单局累计上限) {
  resolved_mode = 从 requested_mode / active_mode / move / attack / defend 中选一个仍合法的;
} else {
  if (requested_mode 已超上限) {
    resolved_mode = active_mode;
  }

  if (resolved_mode != active_mode && 距离上次成功切换 < cooldown_s) {
    resolved_mode = active_mode;
  }
}

if (resolved_mode != active_mode) {
  active_mode = resolved_mode;
  刷新 last_switch_ns;
}

return active_mode;
```

这也是本文档最核心的一条维护原则：

- 行为树负责“提出请求”
- `PublishRobotMode` 负责“决定是否允许切换”
- 串口节点只负责“把最终结果发给下位机”

## 6. 姿态切换约束

### 6.1 切换冷却

参数：

- `decision.mode_limits.switch_cooldown_s`

默认值：

- `5.0`

含义：

- 不是“5 秒后再发一次同样的模式”
- 而是“距离上一次成功切换姿态不足 5 秒时，不允许切到另一个新姿态”

例如：

1. 当前是 `move`
2. 这一刻视觉目标出现，请求切 `attack`
3. 若此时距离上一次姿态切换不足 `5s`
4. 则本次保持原姿态，不立即切换

这样做是为了避免：

- 视觉目标一闪而过导致来回切换
- 血量阈值边界抖动导致 `move/defend` 反复横跳

一个典型时序例如下：

```text
t = 0.0s   当前 active_mode = move
t = 1.0s   视觉目标出现，请求 attack，允许切换，active_mode = attack
t = 3.0s   目标丢失，请求 move，但距离上次切换仅 2s < cooldown(5s)
           因此保持 attack，不切回 move
t = 6.2s   仍然请求 move，且已经超过 5s 冷却
           此时才真正切回 move
```

所以冷却限制的是“姿态之间的切换频率”，不是“消息发布频率”。

### 6.2 单局累计时长限制

参数：

- `decision.mode_limits.max_cumulative_s`

默认值：

- `180.0`

含义：

- `move`、`attack`、`defend` 每一种姿态，各自独立累计时长
- 任意一种姿态在单局中累计达到上限后，就不再允许重新切入该姿态

当前累计状态存放在 `PublishRobotModeAction` 内部维护的运行时状态中：

```cpp
struct RobotModeRuntimeState
{
  bool initialized = false;
  bool match_running = false;
  uint8_t last_game_progress = pb_rm_interfaces::msg::GameStatus::NOT_START;
  uint8_t active_mode = kMoveMode;
  int64_t last_update_ns = 0;
  int64_t last_switch_ns = 0;
  std::array<double, 3> cumulative_s{0.0, 0.0, 0.0};
};
```

其中：

- `active_mode` 表示当前生效姿态
- `last_switch_ns` 用来判断冷却时间
- `cumulative_s[0/1/2]` 分别记录 `move/attack/defend` 的单局累计时长

建议按下面方式理解这个限制：

1. 它限制的是“累计使用时长”，不是“连续使用时长”。
2. 每种姿态各记各的，不会互相抵扣。
3. 到达上限后，不是强制停机，而是“不再允许重新选择该姿态”。
4. 若当前正在使用的姿态在累计更新后碰到上限，下一次裁决时会尝试回退到其他合法姿态。

### 6.3 单局计时什么时候清零

当前逻辑不是节点重启就清零，而是：

- 当比赛从“非 RUNNING”进入 `RUNNING` 时
- 视为新的一局开始
- 三种姿态累计时间全部重置为 `0`

代码中对应逻辑在：

```cpp
if (!state.match_running && match_running) {
  initializeRuntimeState(state, now_ns, true, current_game_progress, cooldown_s);
}
```

这样做的好处是：

- 平时调试不容易误把上一局比赛时长带进来
- 也不需要依赖人工手动清理状态

### 6.4 超限后的回退策略

如果请求姿态已经到达累计上限，系统不会直接报错退出，而是尝试回退到仍然可用的姿态。

当前回退优先级为：

1. `requested_mode`
2. `state.active_mode`
3. `move`
4. `attack`
5. `defend`

对应代码：

```cpp
const std::array<uint8_t, 5> candidates = {
  preferred_mode, state.active_mode, kMoveMode, kAttackMode, kDefendMode};
```

这样设计的原因是：

1. 先尽量满足当前分支想要的姿态。
2. 如果不行，优先保持当前姿态，降低抖动。
3. 如果当前姿态也超限，再回退到其他仍合法的姿态。

可以把这个回退逻辑理解为一句话：

> 先满足当前分支，再尽量保持稳定，最后才全局兜底。

这也是为什么不会简单地“某个姿态超限就永远强制 move”，而是会把其余合法姿态也纳入选择。

## 7. 受击旋转逻辑

### 7.1 为什么受击自旋和防御姿态分开处理

“进入防御姿态”与“是否自旋”不是一个概念。

- 防御姿态看的是血量是否低于阈值。
- 受击自旋看的是最近是否发生过有效装甲受击。

因此当前实现中：

- 低血量时可以进入 `defend`
- 但只有在最近发生真实掉血时才自旋
- 若后续一段可配置时间内没有继续掉血，则停止自旋

这样更符合你的需求：

- 不是低血量就一直转
- 也不是所有扣血原因都触发自旋

### 7.2 触发条件

只有同时满足以下两个条件，才认定为有效受击：

1. `is_hp_deduced == true`
2. `hp_deduction_reason == ARMOR_HIT`

对应代码：

```cpp
const bool is_attacked = msg->is_hp_deduced && msg->hp_deduction_reason == msg->ARMOR_HIT;
```

这意味着下列情况都不会触发自旋：

- 并未实际掉血
- 掉血原因不是装甲板受击
- 当前没有新的裁判系统受击信息

### 7.3 受击方向怎么得到

当前根据裁判系统给出的 `armor_id` 推算云台/底盘应面对的受击方向：

- `0 -> 0`
- `1 -> +pi/2`
- `2 -> +pi`
- `3 -> -pi/2`

代码中用 `last_attack_yaw_` 保存最近一次受击方向。  
如果 `armor_id` 非法，会打告警，但不会让节点崩溃。

### 7.4 为什么要做“锁存”

受击消息通常只会在某一帧出现一次。  
如果条件节点只在“当前这一帧”返回成功，那么行为树下一帧就可能立刻停转，实际效果太短。

所以 `IsAttackedCondition` 额外维护了：

- `attack_latched_`
- `last_attack_time_`
- `last_attack_yaw_`

逻辑是：

1. 一旦收到一次有效受击，先记住“最近被打过”。
2. 后续即使下一帧没有新的受击消息，只要没超过超时时间，仍然返回 `SUCCESS`。
3. 一旦超过超时时间还没有新掉血，就清掉锁存，返回 `FAILURE`。

也就是说，`IsAttacked` 不是一个只看“当前帧”的瞬时条件，而是一个“短时记忆条件”。
这点非常关键，因为行为树是周期 tick 的，如果没有这个短时锁存，自旋会因为消息不是每帧都带受击信息而显得断断续续。

### 7.5 停止条件

参数：

- `decision.motion.hit_spin_stop_after_no_hp_drop_s`

默认值：

- `2.0`

含义：

- 不是固定自旋 2 秒
- 而是“最近一次有效掉血之后，若连续 2 秒没有再次掉血，则停止自旋”

这和“固定定时器”相比更合理，因为：

1. 若敌人持续命中，旋转会持续刷新，不会过早停下。
2. 若只是偶发受击，很快就能自动停转，不会一直空转。

一个更直观的例子：

```text
t = 10.0s  第一次有效装甲掉血，开始自旋
t = 10.8s  再次有效掉血，刷新最近受击时间
t = 11.6s  再次有效掉血，继续刷新
t = 13.4s  仍未超过 stop_after_s，自旋继续
t = 13.7s  距离最近一次掉血已超过 stop_after_s，停止自旋
```

因此该参数本质上描述的是：

- “没有继续掉血时，受击自旋还能保留多久”

而不是：

- “每次受击固定转几秒”

### 7.6 自旋速度

参数：

- `decision.motion.hit_spin_speed`

默认值：

- `7.0 rad/s`

该参数只在 `IsAttacked` 返回 `SUCCESS` 的那段时间内被发布。  
其余情况行为树会发送 `0.0`。

### 7.7 自旋速度最终是不是通过 `wz` 发给下位机

是的，当前实现里受击自旋速度最终就是通过底盘速度指令里的 `angular.z`，也就是下位机串口结构体中的 `speed_vector.wz` 发下去的。

完整链路如下：

```text
IsAttacked 返回 SUCCESS
  -> PublishSpinSpeed 发布 Float32 到 cmd_spin
  -> fake_vel_transform 订阅 cmd_spin，保存 spin_speed_
  -> fake_vel_transform 在速度变换时执行：
     aft_tf_vel.angular.z = twist->angular.z + spin_speed_
  -> 输出新的 cmd_vel
  -> standard_robot_pp_ros2 订阅 /cmd_vel
  -> send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z
  -> 串口发送给下位机
```

也就是说，`decision.motion.hit_spin_speed` 并不是单独通过一个“姿态字段”或者“专用自旋字段”发下去，而是叠加到底盘角速度命令中。

对应代码位置如下：

- `PublishSpinSpeed` 将自旋速度发布到 `cmd_spin`
  - `src/pb2025_sentry_behavior/plugins/action/pub_spin_speed.cpp`
- `fake_vel_transform` 将 `cmd_spin` 叠加到 `cmd_vel.angular.z`
  - `src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp`
- `standard_robot_pp_ros2` 将 `cmd_vel.angular.z` 写入 `speed_vector.wz`
  - `src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp`

其中最关键的一行是：

```cpp
aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
```

它表示：

1. 导航或上层原本给出的角速度是 `twist->angular.z`
2. 受击自旋附加角速度是 `spin_speed_`
3. 最终发给下位机的 `wz` 是两者叠加后的结果

所以如果后续你发现“机器人在移动时受击会边走边转”，这是当前设计的正常结果，因为自旋速度本来就是作为额外 `wz` 叠加进去的。

## 8. 参数总表

| 参数名 | 默认值 | 作用 | 影响模块 |
| --- | --- | --- | --- |
| `decision.topics.robot_mode` | `decision/robot_mode` | 姿态模式发布话题 | 行为树 / 串口 |
| `decision.mode_thresholds.defend_hp` | `300` | 低于该血量进入防御分支 | 行为树 |
| `decision.mode_limits.switch_cooldown_s` | `5.0` | 姿态切换冷却时间 | `PublishRobotMode` |
| `decision.mode_limits.max_cumulative_s` | `180.0` | 单局单姿态累计时长上限 | `PublishRobotMode` |
| `decision.motion.hit_spin_speed` | `7.0` | 受击时发布的自旋角速度 | 行为树 |
| `decision.motion.hit_spin_stop_after_no_hp_drop_s` | `2.0` | 没有继续掉血多久后停转 | `IsAttacked` |
| `standard_robot_pp_ros2.robot_mode_topic` | `decision/robot_mode` | 串口节点订阅的姿态话题 | 下位机接口 |

## 9. 参数是如何从 YAML 流到代码里的

很多后续维护问题其实不是“逻辑错了”，而是“不知道参数最终被谁用了”。  
当前这套姿态系统的参数流转顺序如下：

```text
params/*.yaml
  -> pb2025_sentry_behavior_server declare/get_parameter
  -> globalBlackboard()->set(...)
  -> behavior tree XML 通过 {@...} 取值
  -> PublishRobotMode / IsAttacked / IsRobotHpBelow 读取端口
  -> 发布 robot_mode 或 spin 速度
```

例如防御阈值和姿态冷却是这样接上的：

```cpp
// pb2025_sentry_behavior_server.cpp
globalBlackboard()->set("decision_defend_mode_hp", defend_mode_hp);
globalBlackboard()->set("decision_mode_switch_cooldown_s", mode_switch_cooldown_s);
globalBlackboard()->set("decision_mode_max_cumulative_s", mode_max_cumulative_s);
```

```xml
<!-- rmul_2026.xml -->
<IsRobotHpBelow threshold="{@decision_defend_mode_hp}"/>
<PublishRobotMode mode="attack"
                  cooldown_s="{@decision_mode_switch_cooldown_s}"
                  max_cumulative_s="{@decision_mode_max_cumulative_s}"/>
```

这意味着：

1. 如果你只想调阈值，优先改 YAML。
2. 如果你改了黑板 key，XML 和插件端口默认值都要同步。
3. 如果你改了 topic 名称，行为树发布端和串口订阅端要一起改。

## 10. 关键代码落点

### 10.1 姿态统一裁决

- `src/pb2025_sentry_behavior/plugins/action/pub_robot_mode.cpp`
- `src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/plugins/action/pub_robot_mode.hpp`

职责：

- 将字符串姿态 `move / attack / defend` 转成枚举值
- 维护单局运行时状态
- 处理冷却时间
- 处理累计时长限制
- 计算最终允许发布的姿态

### 10.2 受击检测与停转

- `src/pb2025_sentry_behavior/plugins/condition/is_attacked.cpp`
- `src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/plugins/condition/is_attacked.hpp`

职责：

- 判断是否是有效装甲受击
- 锁存最近一次受击信息
- 在可配置的超时时间内维持 `SUCCESS`

### 10.3 低血量防御判断

- `src/pb2025_sentry_behavior/plugins/condition/is_robot_hp_below.cpp`
- `src/pb2025_sentry_behavior/include/pb2025_sentry_behavior/plugins/condition/is_robot_hp_below.hpp`

职责：

- 从 `RobotStatus` 读取当前血量
- 与 `decision.mode_thresholds.defend_hp` 进行比较
- 为主树是否进入防御分支提供条件判断

### 10.4 参数声明与黑板注入

- `src/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp`

职责：

- 声明姿态模式和受击自旋相关参数
- 将参数放入黑板
- 让 XML 可以通过 `{@...}` 直接引用这些值

### 10.5 下位机模式发送

- `src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp`
- `src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/packet_typedef.hpp`

职责：

- 订阅 `decision/robot_mode`
- 校验模式值是否合法
- 将模式写入 `SendRobotCmdData.data.speed_vector.mode`
- 经串口发给下位机

## 11. 维护时最常看的几个函数

如果你是第一次接手这部分代码，建议按下面顺序看：

1. `PublishRobotModeAction::setMessage`
   入口函数，负责把 XML 里的字符串姿态转成枚举，并调用统一裁决函数。
2. `PublishRobotModeAction::resolveModeWithConstraints`
   真正执行冷却时间、累计时长限制、回退策略的核心函数。
3. `IsRobotHpBelowCondition::tickCondition`
   低血量进入防御分支的最直接入口。
4. `IsAttackedCondition::checkIsAttacked`
   受击锁存、自旋保持、停止条件的核心函数。
5. `StandardRobotPpRos2Node::cmdRobotModeCallback`
   上层姿态最终写入串口发送结构体的位置。

如果调试时发现“行为树看起来进入了某个分支，但下位机收到的模式不是预期值”，优先检查第 2 个函数，因为很可能是被冷却时间或累计时长限制拦住了。

## 12. 当前默认参数

- 防御姿态血量阈值：`300`
- 姿态切换冷却：`5.0 s`
- 单姿态单局累计上限：`180.0 s`
- 受击自旋速度：`7.0 rad/s`
- 无继续掉血后的停转时间：`2.0 s`

## 13. 维护建议

1. 如果只改了阈值、冷却时间或停转时间，优先改 yaml 参数，不要先改代码常量。
2. 如果新增第四种姿态，必须同步修改：
   - `PublishRobotModeAction` 的枚举和累计数组
   - 下位机 `packet_typedef.hpp` 的 `mode` 枚举
   - 相关 README 和本文档
3. 如果比赛规则再次变化，先确认“限制的是切换次数、连续时间还是累计时间”，不要直接在当前逻辑上硬改。
