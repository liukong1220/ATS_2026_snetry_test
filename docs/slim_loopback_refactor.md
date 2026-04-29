# 轻量 Loopback 仿真说明

> 说明  
> 本文保留了较多阶段性调试记录与历史命令。若文中某些绝对路径、旧命名或阶段性结论与当前代码不一致，请以这几份现状文档为准：[`./移植.md`](./移植.md)、[`./sentry_bt_decision_checklist.md`](./sentry_bt_decision_checklist.md)、[`./融合.md`](./融合.md)。

如果你当前在看哨兵姿态切换、防御阈值、受击自旋或下位机模式发送，请额外以 [`./sentry_posture_switch_logic.md`](./sentry_posture_switch_logic.md) 为准。

## 0. 现状速览

如果你现在只是想按当前代码启动和调试，请先记住这几个现状入口：

1. 通用 loopback 入口  
   [`../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py`](../src/pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)
2. 视觉跟随快捷入口  
   [`../src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py`](../src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)
3. 通用 loopback 行为树参数  
   [`../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml)
4. 视觉测试行为树参数  
   [`../src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml`](../src/pb2025_sentry_behavior/params/sentry_behavior_vision_test.yaml)
5. loopback Nav2 与 MPPI 参数  
   [`../src/loopback_sim/params/nav2_params.yaml`](../src/loopback_sim/params/nav2_params.yaml)

当前建议直接使用：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

如果要测视觉接管，则使用：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py use_rviz:=True
```

本文后续如果出现：

1. `/home/ats/...` 旧绝对路径
2. 阶段性临时参数文件名
3. 已被后续文档细化或纠正的中间结论

请把它们视为开发存档，而不是当前唯一标准。

## 1. 文档目标

这份文档解释的是当前项目里的“轻量 loopback 仿真”到底是怎么打通的。

它不是 Gazebo，也不是 Ignition，也不是实车替身的完整物理仿真。它的目标非常明确：

- 不接串口
- 不接底盘
- 不接 Livox 雷达
- 不接实机裁判系统
- 不跑 Point-LIO、重定位、地形分析这一整套真实导航前端

但仍然保留下面这些真正有价值的上层链路：

- 保留行为树决策
- 保留 Nav2 规划与控制
- 保留地图与代价地图
- 保留 `NavigateThroughPoses` 行为接口
- 保留话题级闭环验证能力

一句话概括：

`loopback` 的本质，是用一个极简的软件节点把 `cmd_vel` 再“回灌”为 `odom / tf / scan`，从而让行为树和 Nav2 在没有硬件、没有物理引擎的情况下仍然跑成闭环。

## 2. 先看整体闭环

当前默认启动入口：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py
```

这条链路的核心闭环可以先记成下面这张图：

```text
fake_decision_sim_inputs
  |- 发布 /initialpose ------------------------------> loopback_simulator
  |- 发布 /decision/sim_mode ------------------------> pb2025_sentry_behavior_server
  `- 可选发布 /referee/* ----------------------------> pb2025_sentry_behavior_server

pb2025_sentry_behavior_server
  `- 通过 BT 生成 decision_path
     并调用 /navigate_through_poses -----------------> Nav2

Nav2
  `- 规划、控制，输出 /cmd_vel ----------------------> loopback_simulator

loopback_simulator
  |- 根据 /cmd_vel 积分出 /odom
  |- 发布 TF: map->odom, odom->base_footprint
  |- 基于静态地图射线生成 /scan
  `- 发布 /clock

/odom, /tf, /scan
  |- 反馈给 Nav2 的 local/global costmap
  `- 反馈给 pb2025_sentry_behavior 的位姿相关逻辑
```

这就是“loopback 链接成功”的关键：

- 决策侧有输入
- Nav2 侧有地图、位姿、激光
- 控制侧能输出 `cmd_vel`
- 仿真侧能把 `cmd_vel` 再变回导航系统可消费的状态

于是整个系统就形成了一个软件闭环。

## 3. 当前轻量链路的特点

当前这套轻量 loopback 有这些特征：

- 默认使用 `sentry_behavior_loopback.yaml`
- 默认 `decision.input_source=simulation`
- 默认用 `decision/sim_mode` 控制行为树模式
- 决策输出已经统一走 `/navigate_through_poses`
- 可以按需切换到“手动裁判仿真输入”模式
- 可以完全关闭假输入节点，改为你自己手发 topic

它不是在模拟真实底盘动力学，而是在模拟“导航与决策系统眼中的世界”。

## 4. 启动后到底起了哪些东西

`pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py` 是整条轻量链路的总入口。它做的事情可以拆成 7 个部分。

### 4.1 地图服务

启动：

- `nav2_map_server/map_server`
- `nav2_lifecycle_manager/lifecycle_manager_map_server`

作用：

- 读取地图，默认地图来自 `src/pb2025_sentry_bringup/map/rmul.yaml`
- 给 Nav2 提供静态地图
- 给 `loopback_simulator` 提供 `/map_server/map` 服务

这里的地图不仅给全局规划器用，还被 `loopback_simulator` 拿去“反向生成”假激光。

### 4.2 最小 TF 树

启动了两个静态 TF：

- `base_footprint -> base_link`
- `base_link -> base_scan`

作用：

- 补齐 Nav2 需要的基础坐标关系
- 让 `loopback_simulator` 能把激光发布在 `base_scan` 上

这也是为什么当前轻量 loopback 不需要完整的 URDF 和 `robot_state_publisher`。它只保留最小必要 TF，而不是整个机器人模型。

### 4.3 Loopback 仿真器

通过 `src/loopback_sim/launch/loopback_simulation.launch.py` 启动：

- `nav2_loopback_sim/loopback_simulator`

它订阅：

- `initialpose`
- `cmd_vel`

它发布：

- `/clock`
- `/odom`
- `/tf`
- `/scan`

它是整套轻量仿真的核心，因为它承担了“底盘 + 定位 + 简化传感器”的替身角色。

这里还有一个很重要的坐标系细节：

- `loopback_simulator` 自己发布的是 `odom -> base_footprint`
- Nav2 的主要配置使用的是 `base_link`
- 两者之间靠静态 TF `base_footprint -> base_link` 衔接
- 假激光则挂在 `base_scan`

也就是说，当前最小 TF 链实际是：

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

### 4.4 最小 Nav2 栈

通过 `src/pb2025_sentry_bringup/launch/loopback_navigation.launch.py` 启动：

- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `lifecycle_manager_navigation`

这里没有走 `pb2025_nav_bringup` 的完整 reality/simulation bringup，而是直接拉起一套足够支撑 `NavigateThroughPoses` 的“最小 Nav2”。

默认用的参数文件也不是 `pb2025_nav_bringup/config/simulation/nav2_params.yaml`，而是：

- `src/loopback_sim/params/nav2_params.yaml`

这份 loopback 专用的参数文件使用 `nav2_mppi_controller::MPPIController` 作为局部控制器（`DiffDrive` 运动模型），与实车和仿真环境的 `Omni` 全向运动模型区分开。

这说明当前 slim loopback 的思路是：

- 保留 Nav2 核心能力
- 但不把真实点云、定位、速度适配那一整层依赖重新背进来

### 4.5 假输入节点

启动：

- `pb2025_sentry_bringup/scripts/fake_decision_sim_inputs.py`

它负责：

- 重复发布 `initialpose`
- 发布 `decision/sim_mode`
- 可选发布 `referee/game_status`
- 可选发布 `referee/robot_status`
- 可选发布 `referee/rfid_status`

这个节点是“行为树入口信号发生器”。

### 4.6 行为树系统

延迟 3 秒启动：

- `pb2025_sentry_behavior_server`
- `pb2025_sentry_behavior_client`

这里特意延迟启动，是为了尽量让地图、TF、Nav2 action server 先就绪，减少行为树刚启动时因为 action server 尚未可用而报错。

### 4.7 RViz

按需启动 RViz，默认复用 `pb2025_nav_bringup` 里的 RViz 配置。

## 5. Loopback 是如何真正形成“仿真闭环”的

这一节是最关键的部分。

### 5.1 `initialpose` 先把机器人“放进地图”

`fake_decision_sim_inputs.py` 在启动后会重复多次发布 `initialpose`。

这样做不是多余，而是为了保证在系统刚启动、订阅关系还没完全建立时，`loopback_simulator` 仍然能收到至少一次初始位姿。

`loopback_simulator` 收到第一次 `initialpose` 后会做两件事：

1. 把 `map -> odom` 设为这次初始位姿
2. 把 `odom -> base_footprint` 初始化为单位变换

于是机器人在地图中的“出生点”就建立起来了。

这一步非常重要，因为在 `initialpose` 到来之前，`loopback_simulator` 对 `cmd_vel` 是忽略的。也就是说：

- 没有初始位姿
- 就没有有效仿真运动

### 5.2 `cmd_vel` 被积分成 `odom`

一旦初始位姿建立完成，`loopback_simulator` 就开始周期性执行更新：

- 从 `cmd_vel` 取当前速度
- 按 `update_duration` 积分出 `dx / dy / dtheta`
- 更新 `odom -> base_footprint`
- 发布 `nav_msgs/Odometry`

它使用的是极简二维运动学积分，而不是动力学仿真。

这意味着：

- 没有轮滑
- 没有加速度限制导致的迟滞
- 没有摩擦
- 没有碰撞反弹

但对于验证“行为树是否能驱动 Nav2 形成正确路径执行”来说，这已经足够。

### 5.3 假激光不是乱发的，而是从地图里“射线投射”出来的

`loopback_simulator` 并不是简单发一圈固定距离。

它会：

- 先调用 `/map_server/map` 服务拿到 OccupancyGrid
- 读取当前激光位姿
- 对每条扫描角度做射线遍历
- 在地图栅格里找到第一个占据值较高的位置
- 把那个距离写进 `LaserScan.ranges`

所以这里的 `/scan` 不是纯占位，而是“由静态地图反演出的二维激光”。

这带来两个直接好处：

- Nav2 的 local/global costmap 能正常工作
- 基于激光的碰撞监控、局部避障至少有最基础的环境反馈

需要注意的是，这个激光只反映静态地图，不包含真实动态障碍物，也不包含传感器噪声模型。

### 5.4 行为树为什么能切换到仿真模式

`sentry_behavior_loopback.yaml` 里最关键的一项是：

```yaml
decision:
  input_source: simulation
```

这会让 `pb2025_sentry_behavior_server` 在初始化时把根黑板上的：

- `decision_input_source`

设置成 `simulation`。

随后行为树 `rmul_2026.xml` 在顶层通过 `IsDecisionInputSource` 做分流：

- `simulation` 走 `decision_simulation`
- `referee` 走 `decision_referee`

因此 loopback 默认不会依赖完整裁判系统输入，它直接进入仿真专用的行为分支。

### 5.5 `decision/sim_mode` 是如何驱动路径选择的

`fake_decision_sim_inputs.py` 默认持续发布：

- `decision/sim_mode`

可选值：

- `patrol`
- `anchor`
- `retreat`
- `safe`

`pb2025_sentry_behavior_server` 订阅这个 topic 后，会把最新模式写进根黑板：

- `decision_sim_mode`

然后 `decision_simulation` 这棵子树通过 `IsDecisionMode` 选择不同路径生成逻辑：

- `safe` -> `decision_safe_point`
- `retreat` -> `decision_retreat`
- `anchor` -> `decision_anchor_target`
- `patrol` -> `decision_patrol`

这一步就是“仿真输入如何驱动真实行为树”的关键桥梁。

### 5.6 路径不是写死发给底盘，而是先生成 `nav_msgs/Path`

行为树里的路径生成插件主要有三类：

- `SelectFixedPath`
- `SelectNearestRetreatPath`
- `SelectPatrolPath`

这些插件都会从参数里的：

- `decision.goal_points`
- `decision.point_roles.*`

构造出 `nav_msgs/Path`。

其中：

- `safe` 和 `anchor` 一般会落到固定点
- `retreat` 会根据当前位姿在候选点里选最近点
- `patrol` 会根据 `patrol_cursor`、`patrol_direction` 和 `patrol_preview_points` 生成一段预览路径

所以 loopback 下测到的不是“假动作”，而是你当前行为树参数配置下的真实路径生成结果。

### 5.7 路径是如何送进 Nav2 的

真正把行为树和 Nav2 接起来的是插件：

- `SendNavThroughPoses`

它会把行为树生成的 `decision_path` 转成 `NavigateThroughPoses::Goal`，发送到：

- `/navigate_through_poses`

也就是说，loopback 模式下并没有绕开 Nav2，而是明确复用了项目现在的导航执行接口。

这非常关键，因为这保证了以下东西在 loopback 中仍然真实有效：

- action server 是否能接到目标
- planner 能不能出路径
- controller 能不能收敛
- 路径切换时行为树会不会频繁重发目标

### 5.8 Nav2 输出 `cmd_vel`，loopback 再把它喂回系统

Nav2 的控制器输出 `cmd_vel` 后：

- `loopback_simulator` 订阅它
- 把速度积分成里程计
- 发布 `/odom`
- 发布 `/tf`

与此同时，行为树服务端也在订阅：

- `odom`

用来更新当前位姿黑板项 `decision_current_pose`。

于是就形成了完整反馈回路：

```text
行为树选路径
-> Nav2 接收 NavigateThroughPoses
-> Nav2 输出 cmd_vel
-> loopback_simulator 生成 odom/tf/scan
-> Nav2 和行为树再次看到新的机器人状态
```

这就是当前工程里“轻量 loopback 仿真能跑起来”的真正原因。

## 6. 默认轻量仿真模式

默认启动：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py
```

默认配置等价于：

- `behavior_params_file=sentry_behavior_loopback.yaml`
- `decision.input_source=simulation`
- `decision_mode=patrol`
- `publish_decision_mode=True`
- `publish_referee_inputs=False`

运行中切模式：

```bash
ros2 param set /fake_decision_sim_inputs decision_mode patrol
ros2 param set /fake_decision_sim_inputs decision_mode anchor
ros2 param set /fake_decision_sim_inputs decision_mode retreat
ros2 param set /fake_decision_sim_inputs decision_mode safe
```

如果你要做脚本化切换，`fake_decision_sim_inputs.py` 还支持 `mode_script` 参数，格式是：

```text
<触发秒数>:<模式>
```

例如：

```text
0:patrol
15:anchor
30:retreat
```

## 7. 手动裁判仿真

如果你想测试血量、比赛时间、RFID 等裁判逻辑，就不要用 loopback 专用参数，而是切回正式行为树参数：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py \
  use_rviz:=true \
  publish_referee_inputs:=True \
  publish_decision_mode:=False \
  behavior_params_file:=/home/ats/ats_sentry_ws/install/pb2025_sentry_behavior/share/pb2025_sentry_behavior/params/sentry_behavior.yaml
```

这里的含义是：

- 行为树重新按 `referee` 输入源工作
- 假输入节点不再发 `decision/sim_mode`
- 假输入节点改为构造裁判话题

运行后常用调参：

```bash
ros2 param set /fake_decision_sim_inputs current_hp 100
ros2 param set /fake_decision_sim_inputs stage_remain_time 30
ros2 param set /fake_decision_sim_inputs game_progress 4
```

## 8. 不想让假输入节点接管话题时

如果你希望完全自己控制话题输入：

```bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py \
  use_rviz:=False \
  publish_referee_inputs:=False \
  publish_decision_mode:=False
```

然后手动发 topic。

例如只控制仿真模式：

```bash
ros2 topic pub /decision/sim_mode std_msgs/msg/String "{data: anchor}" -1
```

或者手动构造裁判状态：

```bash
ros2 topic pub /referee/game_status pb_rm_interfaces/msg/GameStatus "{game_progress: 4, stage_remain_time: 30}" -1
```

## 9. 平滑性调节

loopback 下巡逻是否“丝滑”，最关键的两个参数是：

- `decision.point_roles.patrol_indices`
- `decision.decision_config.patrol_preview_points`

原理是：

- 巡逻点越密，生成的 Through Poses 越连续
- `patrol_preview_points` 越大，一次送给 Nav2 的未来点越多

但要注意：

- 如果巡逻点总共只有 2 个，预览点再大提升也有限
- 真要让路径切换平顺，优先把巡逻路线拆得更细

运行时可直接调：

```bash
ros2 param set /pb2025_sentry_behavior_server decision.decision_config.patrol_preview_points 3
```

## 10. Loopback 与其他文件夹的逻辑关系

这一节直接对应“这个 loopback 和仓库里其他部分到底是什么关系”。

### 10.1 直接参与当前轻量闭环的目录

`src/loopback_sim`

- 提供 `nav2_loopback_sim` 包
- 核心文件是 `nav2_loopback_sim/loopback_simulator.py`
- 负责把 `cmd_vel` 回灌成 `odom / tf / scan / clock`
- 自带一份适合 loopback 的 `nav2_params.yaml`

`src/pb2025_sentry_bringup`

- 是当前轻量仿真的总装层
- `launch/loopback_decision_sim.launch.py` 负责把地图、loopback、Nav2、行为树、RViz 串起来
- `launch/loopback_navigation.launch.py` 负责拉起最小 Nav2 栈
- `scripts/fake_decision_sim_inputs.py` 负责提供初始位姿、仿真模式和可选裁判输入
- `map/` 提供默认地图资源

`src/pb2025_sentry_behavior`

- 是当前 loopback 的真实决策主体，不是占位包
- `params/sentry_behavior_loopback.yaml` 把输入源切到 `simulation`
- `behavior_trees/rmul_2026.xml` 定义仿真/裁判双入口树
- `plugins/action/*` 和 `plugins/condition/*` 决定如何从仿真模式生成路径并发送给 Nav2

`src/pb2025_sentry_nav/pb2025_nav_bringup`

- 在当前 slim loopback 中没有直接承担定位、感知、点云处理等重任务
- 主要被复用于 RViz 配置与导航工程资源
- 它代表的是项目正式导航栈的标准组织方式，但当前轻量 loopback 为了减负，没有走它的完整 bringup

`docs`

- 当前文档就在这里
- 这里适合放 loopback 架构说明、启动方式、调试方法与设计约束

### 10.2 被 loopback 有意绕开的目录

`src/pb2025_sentry_nav/point_lio`

- 实车或完整导航里负责 LiDAR-IMU 里程计
- 轻量 loopback 中不需要，因为 `loopback_simulator` 直接生成 `odom`

`src/pb2025_sentry_nav/livox_ros_driver2`

- 实车雷达驱动
- 轻量 loopback 中不需要，因为没有真实雷达

`src/pb2025_sentry_nav/small_gicp_relocalization`

- 实际地图重定位模块
- 轻量 loopback 中不需要，因为 `initialpose + map->odom` 已经足够建立全局位姿

`src/pb2025_sentry_nav/terrain_analysis` 与 `terrain_analysis_ext`

- 完整导航链路里的地形代价分析
- 轻量 loopback 中不参与

`src/pb2025_sentry_nav/fake_vel_transform`

- 完整导航里用于速度话题适配、底层接口转换
- 轻量 loopback 中被简化掉，因为 Nav2 直接输出 `cmd_vel` 给 `loopback_simulator`

`src/pb2025_robot_description`

- 当前轻量 loopback 没有启动完整机器人模型
- 只用两个静态 TF 补出 `base_footprint -> base_link -> base_scan`
- 所以它在当前链路中是“间接相关但不直接参与”

`src/dependencies`、`standard_robot_pp_ros2`、`rmoss_*`

- 这些目录更偏底层驱动、串口、机器人基类或外部依赖
- 当前轻量 loopback 明确绕过了这部分硬件通信层

## 11. 它和实车 bringup 的关系

可以把两条链路理解成：

实车链路：

```text
串口/底盘/雷达/定位前端/重定位
-> Nav2
-> 行为树
```

轻量 loopback 链路：

```text
fake_decision_sim_inputs + loopback_simulator
-> Nav2
-> 行为树
```

也就是说，轻量 loopback 并没有替掉：

- 行为树决策策略
- 路径点配置
- `NavigateThroughPoses` 接口
- Nav2 的 planner/controller 行为

它替掉的是：

- 实际硬件驱动
- 实际定位链路
- 实际传感器采集
- 裁判系统实连

因此它特别适合：

- 验证行为树模式切换
- 验证巡逻点配置
- 验证 `NavigateThroughPoses` 闭环
- 验证地图与 Nav2 参数
- 做快速回归测试

但它不适合：

- 调底盘控制器动态性能
- 调真实定位精度
- 调雷达外参或点云链路
- 调实车通信延迟

## 12. 常见验证方法

如果你想确认 loopback 已经真正连通，可以重点看下面几项。

看初始位姿有没有发出去：

```bash
ros2 topic echo /initialpose
```

看 loopback 有没有产生里程计：

```bash
ros2 topic echo /odom
```

看假激光有没有生成：

```bash
ros2 topic echo /scan
```

看行为树有没有收到仿真模式：

```bash
ros2 topic echo /decision/sim_mode
```

看 Nav2 action server 是否存在：

```bash
ros2 action info /navigate_through_poses
```

看系统是否真的在用仿真时钟：

```bash
ros2 topic echo /clock
```

## 13. 常见故障点

机器人完全不动：

- 大概率是 `initialpose` 没有被 `loopback_simulator` 收到
- 因为它在收到初始位姿前会忽略 `cmd_vel`

有地图但 local costmap 没反应：

- 先看 `base_link -> base_scan` 静态 TF 是否存在
- 再看 `/scan` 是否真的有数据

行为树不响应 `decision/sim_mode`：

- 先确认是否还在使用 `sentry_behavior_loopback.yaml`
- 再确认 `decision.input_source` 是否为 `simulation`

路径切换不平顺：

- 优先检查 `patrol_indices`
- 再检查 `patrol_preview_points`

仿真里一切都好，实车不工作：

- 这通常说明上层逻辑没问题，问题更可能在驱动、定位、传感器、时序或硬件接口层
- 这正是轻量 loopback 的价值：帮助你把“上层问题”和“底层问题”拆开

## 14. 当前设计结论

当前这套 slim loopback 的设计已经具备很清晰的工程定位：

- `pb2025_sentry_bringup` 负责总装
- `loopback_sim` 负责替代底层状态闭环
- `pb2025_sentry_behavior` 负责真实决策逻辑
- `Nav2` 负责真实路径执行
- 其他真实硬件与感知定位目录被有意识地裁掉

因此它不是“玩具仿真”，而是一套为哨兵导航与决策回归测试服务的最小可用系统。

## 15. 当前模式下不需要的东西

使用当前 loopback 时，不需要：

- Gazebo
- 串口
- 实机底盘
- 实机雷达
- 实机裁判系统
- Point-LIO
- 重定位
- 地形分析

只有你主动切到“手动裁判仿真模式”时，才会在仿真里构造裁判话题。

## 16. 视觉融合接入后的 loopback 拓扑补充

在当前 slim loopback 结构上，又新增了一条专门给视觉融合测试使用的支路，但它没有破坏默认闭环。

如果你现在主要想调“视觉跟随仿真”，建议同时参考：

- [/home/ats/ats_sentry_ws/docs/视觉跟随仿真调试.md](/home/ats/ats_sentry_ws/docs/视觉跟随仿真调试.md)
- [/home/ats/ats_sentry_ws/docs/融合.md](/home/ats/ats_sentry_ws/docs/融合.md)

### 16.1 默认 loopback 链仍然不变

默认启动：

```bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py
```

依然是：

1. `fake_decision_sim_inputs.py` 发布 `initialpose` 和 `decision/sim_mode`
2. `pb2025_sentry_behavior` 执行 `rmul_2026` 或其他指定树
3. Nav2 输出 `cmd_vel`
4. `loopback_simulator` 回灌 `odom / tf / scan`

也就是说，视觉融合是**增量接入**，不是把原有 slim loopback 改坏。

### 16.2 新增了 `/vision/target` 假输入能力

现在：

- [/home/ats/ats_sentry_ws/src/pb2025_sentry_bringup/scripts/fake_decision_sim_inputs.py](/home/ats/ats_sentry_ws/src/pb2025_sentry_bringup/scripts/fake_decision_sim_inputs.py)

除了能发 `decision/sim_mode` 和可选裁判话题，还能按参数发布 `/vision/target`，包括：

1. `tracking`
2. `nav_hold`
3. `target_id`
4. `target_yaw`
5. `target_pitch`
6. `target_position_map`

这里需要特别说明：

1. 第二轮瘦身后，`suggested_goal_index` 已经从消息和 fake 输入中移除
2. 当前 `vision_test.xml` 直接依赖 `target_position_map`
3. 现在真正驱动视觉导航接管的是 `target_position_map`

这意味着在没有真视觉程序时，loopback 已经可以先验证“视觉接管行为树 + 基于地图目标位置做跟随规划”的链路。

### 16.3 新增了专用快捷入口

新增 launch：

- [/home/ats/ats_sentry_ws/src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py](/home/ats/ats_sentry_ws/src/pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)

推荐启动方式：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py \
  use_rviz:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_target_yaw:=0.30 \
  vision_target_pitch:=-0.06 \
  vision_target_position_map_x:=5.0 \
  vision_target_position_map_y:=2.0 \
  vision_target_position_map_z:=0.0
```

这个入口会自动：

1. 切到 `sentry_behavior_vision_test.yaml`
2. 让行为树执行 `vision_test`
3. 开启假视觉目标发布
4. 让视觉分支优先尝试接管导航

如果你后面要把这个 fake 视觉入口替换成真视觉入口，也建议直接使用已经整理进工作区的：

- [/home/ats/ats_sentry_ws/src/sp_vision25](/home/ats/ats_sentry_ws/src/sp_vision25)

原因是现在这份包已经能通过 `colcon` 安装后直接 `ros2 run sp_vision25 ...`，比继续依赖它自己目录里的独立 `build/` 更适合和 loopback / 行为树联调。

### 16.3.1 运行中如何改假视觉参数

如果 launch 已经跑起来了，最直接的做法是动态改：

```bash
ros2 param set /fake_decision_sim_inputs vision_tracking true
ros2 param set /fake_decision_sim_inputs vision_nav_hold true
ros2 param set /fake_decision_sim_inputs vision_target_yaw 0.25
ros2 param set /fake_decision_sim_inputs vision_target_pitch -0.08
ros2 param set /fake_decision_sim_inputs vision_target_position_map_x 2.5
ros2 param set /fake_decision_sim_inputs vision_target_position_map_y 4.2
ros2 param set /fake_decision_sim_inputs vision_target_position_map_z 0.0
```

常用用途：

1. `vision_tracking=false`
   用来验证视觉分支会不会退出。
2. `vision_nav_hold=false`
   用来验证“看到目标但不建议接管”时是否会回退。
3. `vision_target_yaw / vision_target_pitch`
   用来观察 `/cmd_gimbal` 是否跟着变化。
4. `vision_target_position_map_x/y/z`
   用来观察导航是否围绕新的敌方位置重建跟随点。
5. `publish_vision_target=false`
   用来模拟视觉话题中断。

当前视觉跟随的主调参项则是：

```bash
ros2 param set /pb2025_sentry_behavior_server decision.vision.attack_radius 2.5
```

### 16.4 视觉融合测试时的闭环关系

新增后的测试拓扑可以记成：

```text
fake_decision_sim_inputs
  -> /vision/target
  -> pb2025_sentry_behavior_server
  -> 黑板 {@sp_vision_target}
  -> vision_test.xml
  -> PublishGimbalAbsolute + SelectVisionFollowPath + SendNavThroughPoses
  -> Nav2
  -> loopback_simulator
```

这里最重要的变化有两点：

1. 行为树不再只吃 `decision/sim_mode`，也能吃视觉消息
2. slim loopback 现在不仅能测“导航巡逻是否闭环”，还能测“视觉接管导航/云台是否闭环”

### 16.5 当前视觉 loopback 的边界

这条视觉支路当前主要用于验证：

1. 视觉消息字段是否接对
2. 行为树是否会进入视觉分支
3. `target_position_map` 是否会被转换成跟随路径
4. 云台绝对角话题是否会被发布
5. 视觉失效后是否会回退到普通导航分支

它还不等价于真视觉算法联调，因为：

1. fake 输入里的 `target_position_map` 是**直接注入**
2. 它验证的是“行为树是否正确消费视觉地图目标”
3. 它**不验证** 真实视觉的地图坐标转换和 TF 链路
4. 当前代码默认假设 `target_position_map` 已与 global costmap 处于同一全局坐标系
5. 真实图像、跟踪抖动、时延和置信度波动还需要真机再看

### 16.6 当前已经验证通过的视觉 loopback 现象

目前 loopback 已经确认：

1. 视觉分支可以真正到达 `SelectVisionFollowPath -> SendNavThroughPoses`
2. 运行中动态修改 `vision_target_position_map_x/y` 会触发新的视觉跟随目标
3. 关闭 `vision_tracking` 后，行为树会回退到普通 `decision_simulation` 分支

同时还定位并修复了两个关键卡点：

1. `duration=0` 的 BT 发布节点以前会先返回 `RUNNING`，导致后续规划节点被饿死；现在已改为立即返回 `SUCCESS`
2. `IsPathGoalReached` 以前会被旧路径残留的 `goal_succeeded` 短路，导致新视觉目标来了也被误判成“已经到达”；现在已改成按当前 pose 与当前路径终点距离判断

## 17. 推荐的全面测试顺序

如果你想在 slim loopback 里把视觉融合测得更完整，推荐按下面顺序来。

### 17.1 先确认消息链

启动：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py \
  use_rviz:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_target_position_map_x:=5.0 \
  vision_target_position_map_y:=2.0
```

再看：

```bash
ros2 topic echo --once /vision/target
```

先确认视觉消息在稳定发布，重点看：

1. `tracking`
2. `nav_hold`
3. `target_yaw / target_pitch`
4. `target_position_map`

### 17.2 再确认云台接管链

看：

```bash
ros2 topic echo /cmd_gimbal
```

同时修改：

```bash
ros2 param set /fake_decision_sim_inputs vision_target_yaw 0.35
ros2 param set /fake_decision_sim_inputs vision_target_pitch -0.10
```

如果 `/cmd_gimbal` 跟着变，说明“假视觉 -> 行为树 -> 云台”这半条链通了。

### 17.3 再确认导航接管链

推荐开 RViz，并同时观察行为树日志。依次执行：

```bash
ros2 param set /fake_decision_sim_inputs vision_target_position_map_x 2.5
ros2 param set /fake_decision_sim_inputs vision_target_position_map_y 4.2
ros2 param set /fake_decision_sim_inputs vision_target_position_map_x -2.0
ros2 param set /fake_decision_sim_inputs vision_target_position_map_y -2.0
```

预期应该是：

1. `/vision/target` 里的 `target_position_map` 会改变
2. 日志里会出现新的 `Vision follow target=(...) selected_goal=(...)`
3. 如果当前未到达新跟随点，会再次出现 `Send NavigateThroughPoses goal with 1 poses`

你也可以同时确认 action 服务：

```bash
ros2 action info /navigate_through_poses
```

预期应该看到：

1. client 包含 `/pb2025_sentry_behavior_server`
2. server 包含 `/bt_navigator`

### 17.3.1 再确认半径调参链

执行：

```bash
ros2 param set /pb2025_sentry_behavior_server decision.vision.attack_radius 2.5
```

预期应该是：

1. 日志里的 `attack_radius` 跟着变化
2. `selected_goal` 会在目标周围更远或更近的位置重新选取

### 17.4 再确认回退链

执行：

```bash
ros2 param set /fake_decision_sim_inputs vision_tracking false
```

或者：

```bash
ros2 param set /fake_decision_sim_inputs publish_vision_target false
```

看机器人是否重新受 `decision/sim_mode` 控制。

比较明确的现象通常是：

1. `IsVisionTargetValid` 失败
2. 日志里重新出现普通 patrol 路径，例如 `Send NavigateThroughPoses goal with 2 poses`

### 17.5 最后把“假消息验证”和“真视觉验证”区分开

这里一定要分清：

1. fake 节点里的 `target_position_map` 是直接注入，测的是行为树消费链
2. 当前 loopback 更适合验证“视觉消息接入后，行为树和 Nav2 会不会做出正确动作”
3. 真视觉程序联调时，重点要验证的是 `sp_vision25 -> /vision/target -> target_position_map`

所以如果你要测真视觉地图级目标输出，还需要启动真视觉程序，再去观察：

```bash
ros2 topic echo /vision/target
```

重点看：

1. `target_position_map`
2. `target_position_gimbal`
3. `target_distance`

是不是随着真实目标和机器人位姿变化而自动变化。

### 17.6 如何确认你现在用的是 colcon 版 `sp_vision25`

如果你已经把 `sp_vision25` 并进工作区，推荐先做下面这组确认：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix sp_vision25
ros2 run sp_vision25 sentry --help
```

预期应该看到：

1. `ros2 pkg prefix sp_vision25` 指向当前工作区的 `install/sp_vision25`
2. `ros2 run sp_vision25 sentry --help` 能正常输出帮助，而不是报包找不到或配置找不到

这一步的意义是先确认”你现在调用的是工作区内安装版视觉包”，再去做 loopback 或真机联调，这样最不容易和旧的独立 build 产物串线。

## 18. 局部控制器迁移：PID → MPPI

### 18.1 迁移背景

原有的局部控制器 `pb_omni_pid_pursuit_controller::OmniPidPursuitController` 采用纯追踪（Pure Pursuit）+ 双 PID 的架构：

```
前瞻点 → PID(xy距离) → v_linear
       → PID(角度差) → omega
       → 曲率限速（三点圆拟合减速）
```

这套方案在低速和简单路径上表现稳定，但在以下场景存在固有限制：

- **高速急转弯**：PID 只能”看到”一个前瞻点，无法预判前方弯道，导致入弯减速不及时、出弯超调
- **全向运动耦合**：vx/vy 与 omega 分别由两个独立的 PID 控制，缺乏联合优化
- **多目标权衡**：路径跟踪、障碍物避让、运动平滑性之间无法灵活平衡

MPPI（Model Predictive Path Integral Control）通过**采样 + 前向推演 + 软最大化加权**的方式天然解决这些问题。

### 18.2 MPPI 原理简述

```
每个控制周期 (50ms @ 20Hz):

1. 在当前最优控制序列附近，用高斯噪声生成 K 条候选控制序列
     每条序列长度 T 步，覆盖 1.5s 的前向时域
     采样空间: vx, vy, omega（全向三自由度联合采样）

2. 对每条序列，用运动模型前向推演 T 步，得到一条候选轨迹

3. 对每条轨迹计算加权代价:
     cost = w1 * 路径偏离 + w2 * 路径角度偏差
          + w3 * 目标距离 + w4 * 目标角度偏差
          + w5 * 障碍物碰撞 + w6 * 速度约束违反
          + w7 * 原地旋转 + w8 * 低速死区

4. 用 softmax 将代价转换为权重:
     weight_i = exp(-cost_i / temperature)

5. 加权平均得到最优控制:
     u* = sum(weight_i * u_i) / sum(weight_i)

6. 将最优序列平移一步，作为下一周期的初始猜测 (warm-start)
```

关键公式：

```
u* = ∫ u · exp(-S(u)/λ) du  /  ∫ exp(-S(u)/λ) du

其中 S(u) 是控制序列 u 对应的轨迹总代价，λ 是温度参数
```

### 18.3 修改的文件

| 文件 | 变更 | 说明 |
|------|------|------|
| `pb2025_nav_bringup/config/simulation/nav2_params.yaml` | `FollowPath` 段替换为 MPPI 配置 | 仿真环境，Omni 运动模型，batch_size=500 |
| `pb2025_nav_bringup/config/reality/nav2_params.yaml` | `FollowPath` 段替换为 MPPI 配置 | 实车环境，Omni 运动模型，batch_size=750 |
| `loopback_sim/params/nav2_params.yaml` | `FollowPath` 段从 DWB 替换为 MPPI 配置 | Slim loopback，DiffDrive 运动模型，batch_size=400 |
| `nav_README.md` | 更新控制器描述 | 将 pb_omni_pid_pursuit_controller 改为 nav2_mppi_controller |
| `README.md` | 新增第 7 章 MPPI 配置与调优 | 完整的部署指南和调参手册 |

以下文件**未做任何修改**：
- `pb_omni_pid_pursuit_controller` 包已从仓库中删除，不再作为回退控制器保留
- 所有 launch 文件
- 行为树 XML
- planner_server、smoother_server、behavior_server、velocity_smoother 配置
- 代价地图层配置

### 18.4 三种环境的 MPPI 配置差异

| 参数 | 仿真 (Omni) | 实车 (Omni) | Loopback (DiffDrive) |
|------|------------|------------|---------------------|
| `motion_model` | Omni | Omni | DiffDrive |
| `batch_size` | 500 | 750 | 400 |
| `vx_max/min` | ±2.5 | ±4.5 | ±0.5 |
| `vy_max/min` | ±2.5 | ±4.5 | 0 |
| `wz_max/min` | ±3.0 | ±3.0 | ±1.5 |
| `vx_std` | 0.5 | 0.5 | 0.1 |
| `vy_std` | 0.5 | 0.5 | 0 |
| `wz_std` | 0.8 | 0.8 | 0.3 |
| `critics` | 8 个（含 TwirlingCritic） | 8 个（含 TwirlingCritic） | 8 个（含 PreferForwardCritic） |
| `visualize` | true | true | false |

### 18.5 MPPI 调参方法

#### 18.5.1 调参顺序

按以下顺序逐步调优，每步验证后再进入下一步：

1. **先跑通基本路径跟踪**
   - 只启用 `PathAlignCritic` + `GoalCritic` + `ObstaclesCritic`
   - 确认机器人能跟踪 A* 规划的全局路径并避开障碍物
   - 在 RViz 中开启 `visualize: true` 观察采样轨迹

2. **调路径对齐**
   - 机器人偏离路径：增大 `PathAlignCritic.weight`（当前 15.0）
   - 路径跟踪过于僵硬、频繁修正：减小 weight
   - 机器人横着走（侧移过多）：增大 `PathAngleCritic.weight`（当前 8.0）

3. **调转弯平滑性**
   - 转弯处切角严重：增大 `wz_std`，让采样探索更大的角速度范围
   - 转弯时抖动：减小 `wz_std` 或增大 `ConstraintCritic.weight`
   - 入弯减速不足：增大 `time_steps` 让 MPPI 看到更远的弯道

4. **调全向行为**
   - 机器人原地打转：增大 `TwirlingCritic.weight`（当前 15.0）
   - 需要侧移但不够灵活：减小 `TwirlingCritic.weight`，增大 `vy_std`

5. **调终点收敛**
   - 停不到目标点：增大 `GoalCritic.weight`
   - 停靠角度不对：增大 `GoalAngleCritic.weight`
   - 终点附近震荡：增大 `VelocityDeadbandCritic.weight`

6. **调计算性能**
   - 如果 `controller_server` 掉频（低于 20Hz）：
     - 首先减小 `batch_size`（500 → 400 → 300）
     - 其次减小 `time_steps`（30 → 25 → 20）
     - 关闭可视化 `visualize: false`
   - 如果 CPU 有余量但控制不够平滑，增大 `batch_size`

#### 18.5.2 核心参数速查表

| 参数 | 作用 | 上调效果 | 下调效果 |
|------|------|---------|---------|
| `temperature` (0.05-0.3) | Softmax 锐度 | 更激进，接近单一最优轨迹 | 更保守，融合更多采样，控制更平滑 |
| `batch_size` (300-1000) | 每周期采样数 | 控制更平滑，更好的统计覆盖 | 计算更快 |
| `time_steps` (20-50) | 前向视野 | 更早预判弯道和障碍物 | 响应更快，延迟更低 |
| `vx_std / vy_std` (0.1-1.0) | 线速度探索 | 更多样的速度选择 | 更稳定，减小速度抖动 |
| `wz_std` (0.2-1.5) | 角速度探索 | 转弯更灵活 | 减小方向抖动 |
| `gamma` (0.005-0.05) | 远期折扣 | 更关注远期目标 | 更关注近期路径精度 |

#### 18.5.3 代价权重调优参考

| 代价函数 | 默认权重 | 何时调大 | 何时调小 |
|---------|---------|---------|---------|
| `PathAlignCritic` | 15 | 偏离路径 | 跟踪僵硬 |
| `PathAngleCritic` | 8 | 横着走/航向不对 | 需要侧移灵活性 |
| `GoalCritic` | 10 | 不停靠/停靠慢 | 终点附近震荡 |
| `GoalAngleCritic` | 5 | 终点朝向不对 | 不关心终点朝向 |
| `ObstaclesCritic` | 50 | — | **不应大幅下调** (安全) |
| `ConstraintCritic` | 5 | 速度超限频繁 | 需要更快的加减速 |
| `TwirlingCritic` | 15 | 原地打转 | 需要原地旋转 |
| `VelocityDeadbandCritic` | 3 | 停止时抖动 | 低速指令被过度抑制 |

#### 18.5.4 运行时动态调参

MPPI 支持运行时通过 `ros2 param set` 实时调整所有参数，无需重启：

```bash
# 调温度（最常用的快速调节）
ros2 param set /controller_server FollowPath.temperature 0.1   # 更激进
ros2 param set /controller_server FollowPath.temperature 0.25  # 更保守

# 调采样数
ros2 param set /controller_server FollowPath.batch_size 500

# 调路径跟踪
ros2 param set /controller_server FollowPath.PathAlignCritic.weight 20.0

# 调转弯
ros2 param set /controller_server FollowPath.wz_std 1.2

# 调全向行为
ros2 param set /controller_server FollowPath.TwirlingCritic.weight 25.0

# 快速查看当前所有 MPPI 参数
ros2 param dump /controller_server | grep FollowPath
```

推荐调参工作流：

```bash
# 1. 启动仿真
source install/setup.sh
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True

# 2. 在 RViz 中发布 Nav2 Goal

# 3. 观察机器人跟踪表现，实时调参
ros2 param set /controller_server FollowPath.PathAlignCritic.weight 20.0

# 4. 把满意的参数写回 YAML 文件固化
```

### 18.6 MPPI vs PID 性能对比预期

| 指标 | PID 纯追踪 | MPPI | 改善 |
|------|-----------|------|------|
| 直线跟踪误差 | < 0.1m | < 0.05m | 路径更贴近 |
| 急转弯超调 | 0.3-0.5m | < 0.1m | 大幅减少 |
| 入弯减速 | 曲率限速（被动） | 预测性减速（主动） | 更自然 |
| 全向侧移 | vx/vy 解耦 | vx/vy/ω 联合优化 | 更协调 |
| 障碍物绕行 | 碰撞检测→抛异常 | 代价函数软约束 | 更安全 |
| 终点停靠 | 速度缩放 | GoalCritic + 预测 | 更精确 |
| CPU 开销 | ~1ms | ~10-15ms (batch_size=750) | 可接受 |

### 18.7 回退方案

如需回退到 PID 控制器，只需修改 YAML 中的 `FollowPath.plugin`：

```yaml
# 实车/仿真回退到 PID：
FollowPath:
  plugin: “pb_omni_pid_pursuit_controller::OmniPidPursuitController”  # 旧配置示例，仅作迁移历史说明
  # ... 恢复原 PID 参数

# Loopback 回退到 DWB：
FollowPath:
  plugin: “dwb_core::DWBLocalPlanner”
  # ... 恢复原 DWB 参数
```

PID 控制器包 (`pb_omni_pid_pursuit_controller`) 已从仓库、默认启动、依赖声明和工作区默认构建流程中删除，后续迭代以 MPPI 为唯一局部控制器。

### 18.8 常见 MPPI 问题排查

**Q: 机器人完全不动？**
- 检查 `controller_server` 是否成功加载 MPPI 插件：`ros2 component list`
- 查看 `controller_server` 日志：`ros2 run rqt_console rqt_console`
- 确认 `nav2_mppi_controller` 包已安装：`ros2 pkg list | grep mppi`

**Q: 路径跟踪抖动严重？**
- 减小 `vx_std / vy_std / wz_std`（采样噪声太大）
- 增大 `ConstraintCritic.weight`（抑制大幅速度变化）
- 增大 `temperature`（让控制更保守）

**Q: 转弯时切角严重？**
- 增大 `time_steps`（让 MPPI 看到更远的弯道）
- 增大 `PathAlignCritic.weight`
- 增大 `wz_std`（探索更大的角速度范围）

**Q: controller_server 掉频？**
- 减小 `batch_size`（500 → 400 → 300）
- 减小 `time_steps`（30 → 25 → 20）
- 关闭可视化：`visualize: false`
- 检查是否有其他高 CPU 进程

**Q: 在实车上行为与仿真差异大？**
- 确认 `motion_model` 是否正确（实车用 `Omni`，非全向用 `DiffDrive`）
- 确认速度限制 (`vx_max/min`, `wz_max/min`) 与实际底盘能力匹配
- 实车首次部署建议降低速度上限到 2.0 m/s 验证无误后再恢复
