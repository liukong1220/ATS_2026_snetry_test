# ATS 自研导航一体化架构与 Nav2 清理计划

更新时间：2026-08-02。

本文件定义唯一目标架构、当前源码边界和删除顺序。Nav2 不再是目标运行图的一部分；
源码中尚存的 Nav2 兼容分支只能作为清理对象，不能被新功能继续依赖。

## 1. 最终 Definition of Done

只有同时满足以下条件，才能写“仓库级 Nav2-free 已完成”：

1. 实机、MuJoCo、loopback 和行为正式入口均由 ATS action/Goal Manager 接收目标，
   不使用 `nav2_msgs` action、`/plan` 或 Nav2 lifecycle。
2. 默认运行图没有 `bt_navigator`、`planner_server`、`controller_server`、
   `behavior_server`、`map_server`、Nav2 lifecycle manager 或 Nav2 composition。
3. 活动 package manifest、CMake、launch、YAML、RViz、行为树和测试不再依赖
   `navigation2`、`nav2_common`、`nav2_msgs`、`nav2_core`、`nav2_costmap_2d`。
4. `node_params.yaml` 是正式 profile 的唯一参数文件；behavior、ROGMap core 和
   package 私有 reality YAML 不再作为第二权威加载。
5. `/rc_esdf/planning_grid`、map snapshot、goal、candidate、execution authorization、
   `/cmd_vel` 和 `/motion_control` 各只有一个活动 owner。
6. map/localization/gimbal/reference/command 任一 stale、unknown、unreachable、unsafe
   或 MPC failure 都在 deadline 内发布 STOP 并令底盘输入为零；恢复不得复活旧数据。
7. default、rectangle、red_box、cancel、preempt、timeout、restart、unknown、
   unreachable、projection timeout、heartbeat stale 和 unsafe trajectory 均有独立证据。
8. P4 实车静态、抬轮 HIL、低速准入和独立 contact evaluator 完成前，不得声称实车安全。

## 2. 唯一目标运行图

```mermaid
flowchart LR
  S[LiDAR + IMU] --> L[Point-LIO + localization fusion]
  L -->|/localization| R[ROGMap]
  L -->|/registered_scan| R
  L -->|/localization/status| A[Ground Adapter]
  R -->|GetRogMapProjection: grid + SDF + gradient + generation| A
  A -->|planning grid + status/lease| P[Immutable planning snapshot]
  G[ATS NavigateToPose action] --> M[Goal Manager]
  M -->|PlannerGoal| J[JPS + MINCO S3 + yaw]
  P --> J
  J -->|candidate + PlannerStatus| M
  M -->|ExecutionCommand + reference + stop| C[全向 SE2 MPC]
  C -->|/cmd_vel_mpc body [vx,vy,wz]| F[fake-yaw/chassis compatibility]
  F -->|/cmd_vel body [vx,vy,wz]| X[Serial or MuJoCo bridge]
  X --> W[四舵轮底盘]
  W --> L
  R -. debug only .-> V[RViz]
  J -. debug only .-> V
  C -. telemetry .-> V
```

### 2.1 责任分层

| 层 | 唯一责任 | 不得承担的责任 |
| --- | --- | --- |
| Point-LIO/定位融合 | 局部连续 odom、定位健康、`map -> odom` | 不由 ROGMap 估计位姿 |
| ROGMap | 概率 occupancy、inflation、unknown、3D ESDF、数值 projection | 不发布规划 grid，不从 RViz 点云提供数值距离 |
| Ground Adapter | ground projection、terrain/static/slope/unknown 融合、planning grid lease | 不订阅 `/rog_map/esdf` 反解析 |
| Goal Manager | ATS action 生命周期、目标 identity、PlannerGoal、candidate 验证、ExecutionCommand 和急停 | 不执行 JPS/MINCO 数值优化 |
| JPS/MINCO | 路径搜索、S3 trajectory、独立 yaw、footprint gate/repair | 不直接向底盘发布速度 |
| SE2 MPC | 世界系状态 `[x,y,yaw]` 跟踪 reference，输出车体系 `[vx,vy,wz]` | 不改变规划 frame 或绕过授权 |
| fake-yaw/chassis transform | 实机云台/底盘 frame 兼容、限幅和失效归零 | 不生成第二份命令源 |
| Serial/MuJoCo bridge | 唯一执行出口、watchdog、协议/动力学转换 | 不接受未授权旧 command |
| RViz/diagnostics | 只读显示、健康和调试 | 不成为规划输入或安全判定源 |

### 2.2 当前接口账本

| 接口 | 当前 producer -> consumer | 当前状态 |
| --- | --- | --- |
| `/localization` | localization/Point-LIO -> ROGMap、Goal Manager、MPC | 已接；时间戳和 stale 仍需目标机测量 |
| `/registered_scan` | Point-LIO 链 -> ROGMap | 已接；stale 由 ROGMap health 传播 |
| `/rog_map/get_ground_projection` | ROGMap -> adapter | 已接；response 含 occupancy、signed distance、gradient、generation、ready/stale |
| `/rc_esdf/planning_grid` | adapter 或旧 RC-ESDF 二选一 -> JPS/行为 | 运行时可选 owner；最终必须固定 `rog_map` |
| `/ats_navigate_to_pose` | Goal Manager action server -> behavior/RViz/test | 已接；支持 feedback/result/cancel/preempt/timeout |
| `/ats_goal_manager/planner_goal` | Goal Manager -> MINCO | 已接；当前 `goal_id` 只在进程内单调 |
| `/minco/planning_status` + `/minco/reference_path_candidate` | MINCO -> Goal Manager | 已接但 candidate/status 不是同一结构化样本 |
| `/planner/execution_command` | Goal Manager -> MPC/serial gate | 已接；reference、授权和 `manager_incarnation` 在同一消息。MPC 已要求新 incarnation 先 `MODE_STOP`；serial gate 与 content digest 未覆盖 |
| `/cmd_vel_mpc` | MPC -> fake-yaw/chassis transform | 已接；body frame `[vx,vy,wz]` |
| `/cmd_vel` | chassis transform -> serial bridge | 已接；必须唯一 publisher |
| `/motion_control` | MuJoCo bridge -> MuJoCo | 已接；必须唯一 publisher/subscriber |

`PlanningMapSnapshot`、`PlannerCandidate` 和全链 `authority_incarnation` 仍是目标接口。
当前只有 `ExecutionCommand.manager_incarnation` 已在 Goal Manager -> MPC 链落地，不能
把该局部字段写成 goal/candidate/serial 的端到端版本契约。

## 3. 地图、时间和安全语义

### 3.1 地图

- planning frame 当前为 `odom`；全局 goal 从 `map` 变换到 `odom`。
- ROG projection 的 occupancy：`0=free`、`100=occupied`、`-1=unknown`。
- signed distance：正值为 free clearance，负值为 occupied，unknown 为 NaN。
- 任一来源 occupied 必须保持 occupied；明确新鲜 free 才能消解另一来源 unknown；
  所有来源无证据时输出 unknown，并按障碍处理。
- 静态细图到 planning grid 必须按输出 footprint 覆盖面积保守聚合，不做中心点采样。
- ROG source generation、adapter publication sequence、MINCO local snapshot generation
  和 localization epoch 不能复用同一个编号。

### 3.2 时间与恢复

- 观测 stamp 保留 ROS/sim time；projection deadline、lease 和超时使用 steady clock。
- `ready=true` 是持续 heartbeat，不是永久授权；lease 过期必须同时使规划和执行失效。
- 最终 reference 在安全提交点统一重定时，再由同一互斥区发布 STOP 状态和
  `ExecutionCommand`；旧 reference 不得在恢复后复活。
- map/localization/gimbal stale、投影超时、目标不可达、unsafe trajectory、MPC 失败和
  serial watchdog 都输出确定性零速度。

### 3.3 运动契约

- 状态为世界系 `[x,y,yaw]`，控制为车体系 `[vx,vy,wz]`，单位为 m/s、rad/s。
- 四舵轮 footprint 为 `0.70 m x 0.55 m + margin`，必须进行带 yaw 的 footprint 和
  swept motion 检查。
- `contact_violation_count=0` 或离散 footprint collision 为零，不等价于物理接触为零。

## 4. 当前残余与删除顺序

### N1：运行入口自研化

目标是让正式入口不再有 `launch_nav2` 条件分支，而不是继续增加一个 free wrapper。

| 范围 | 当前残余 | 动作 |
| --- | --- | --- |
| 根 `bringup.launch.py` | `nav2_common.RewrittenYaml`、`launch_nav2`、Nav2 topic 选择 | 固定自研节点和 `rog_map` owner，保留速度兼容层独立开关 |
| 导航 `navigation_launch.py` | server/lifecycle/composable Nav2 分支 | 删除 Nav2 分支，只保留 ROGMap、adapter、MINCO、Goal Manager、MPC 和传感器 |
| `localization_launch.py` | map_server/lifecycle 分支 | 固定 `static_map_publisher.py`，保留 `/map` transient-local 和 origin/yaw |
| `rm_navigation_reality_launch.py` | 私有 params file 和 Nav2 参数 | 所有正式节点只接收根 `node_params.yaml` |
| `joy_teleop_launch.py`、`slam_launch.py` | `nav2_common` 兼容导入 | 迁移为中立 ROS 参数处理或移出正式安装 |

### N2：参数单一权威

当前 `node_params.yaml` 已包含 ROGMap、adapter、MINCO、Goal Manager 和 MPC 段，但仍
包含 Nav2 段；behavior 默认仍使用 `sentry_behavior.yaml`；ROGMap core 仍可从
`map_config_file` 读取自定义 YAML。实施顺序必须是：

1. 生成每个活动节点的 effective `ros2 param dump`；
2. 镜像字段到 `node_params.yaml`，并增加重复 key/topic/frame/timeout 校验；
3. 让 ROGMap core 从 ROS parameter struct 构造 Config，禁止双来源；
4. 让 behavior、serial 和所有正式自研节点只加载根文件；
5. 删除 Nav2 段和正式 launch 对 package 私有 YAML 的引用；
6. 只允许 launch 覆盖 `use_sim_time`、地图/PCD/设备路径和受控 HIL 开关。

### N3：行为、MuJoCo 和 loopback

- 删除 `send_nav2_goal.*`、Nav2 `send_nav_through_poses` 类型和正式构建注册；多航点
  由 behavior 逐点调用 ATS action，保留 cancel/preempt/timeout 测试。
- MuJoCo 官方入口固定 ATS action、静态地图 publisher、ROGMap owner 和 MPC；删除
  `mujoco_navigation.launch.py`/`rmuc_2026_mujoco.launch.py` 中的 Nav2 condition、
  `nav2_common` import、`/plan` remap 和 Nav2 map/lifecycle。
- loopback 若继续保留，只能作为无物理动力学的自研 action/behavior 回归；删除
  Nav2 server、`nav2_params.yaml` 和 Nav2 action 依赖，包名变更必须先审计所有 include。

### N4：公共算法和构建依赖解耦

`minco_planner` 当前 include 并链接 `trajectory_optimizer` 的 RC-ESDF provider。先
迁移并回归以下公共能力，再删除旧包中的 Nav2 专属部分：

- `trajectory_optimizer/esdf/rc_traversability_esdf_provider.*`；
- `trajectory_optimizer/esdf/esdf_provider.hpp`；
- `trajectory_optimizer/esdf/static_map_fusion.*`；
- 必要的 terrain/static ESDF provider 和测试。

不得在迁移前删除整个 `trajectory_optimizer`，不得把 `ats_nav2_plugins` 中无法证明
为 Nav2-only 的能力一并删除。迁移完成后再删除 `nav2_core`、`nav2_costmap_2d`、
`nav2_msgs`、`navigation2`、`nav2_common` manifest/CMake 依赖和 Nav2-only 资源。

### N5：RViz 和文档收口

- 保留 ROGMap 四类点云和 `/rog_map/bounds`，补齐 update/search bounds、health、
  candidate/reference、footprint/repair 和 MPC telemetry。
- 删除 costmap、MPPI、`/plan`、Nav2 GoalTool 等活动 display；`SetGoal` 只能通过
  Goal Manager 的 `/goal_pose` consumer 进入规划。
- 本目录成为自研设计唯一文档源；其它 README/docs 只同步已确认的活动接口。

## 5. 验收矩阵

### 静态门禁

```bash
rg -n 'nav2_common|nav2_msgs|nav2_core|nav2_costmap_2d|navigation2|/plan|bt_navigator|planner_server|controller_server|behavior_server' \
  src/ats_sentry_bringup src/ats_sentry_nav src/ats_sentry_behavior \
  src/sim/ats_mujoco_sim src/sim/loopback_sim
```

结果只允许出现在明确的清理任务、第三方目录或测试 fixture；活动正式 launch、
manifest、YAML、行为树和 RViz 不得出现。

### 运行门禁

每个 case 使用新的 `ROS_DOMAIN_ID` 和新的 MuJoCo launch：

```bash
scripts/test_mujoco_nav_chain.sh                 # 仅历史基线，不是目标架构
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 \
  scripts/test_mujoco_minco_mpc_chain.sh
```

必须记录 action result、终点误差、raw/reference 点数、footprint 冲突采样、owner 数量、
generation/epoch、stop 时间、恢复次数和物理 contact evaluator 状态。

### 停止条件

- planner 需要订阅 `/rog_map/esdf` 点云或混合时间的地图层；
- planning grid、`/cmd_vel`、`/motion_control` 出现两个活动 owner；
- 新旧 YAML 同时生效且 effective param 未闭合；
- stale/unknown/unreachable/unsafe/MPC failure 后仍有非零底盘命令；
- RC-ESDF 迁移前删除 `trajectory_optimizer` 导致公共能力无 owner；
- 修改地图、规划、安全或控制源码后沿用旧闭环结果；
- 目标机尚未测量却把参考项目性能写成 ATS 实测。

每个阶段使用阶段开始时各仓 HEAD 作为回滚点；不得 reset 用户修改，不得 force push。
