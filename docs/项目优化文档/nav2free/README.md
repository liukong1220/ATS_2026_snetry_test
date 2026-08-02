# ATS Nav2-free 自研导航一体化设计

更新时间：2026-08-02。

本目录是 ATS 2026 四驱四转哨兵自研导航的唯一设计、实施和验收入口。正式架构只
包含 Point-LIO/定位融合、ROGMap、地面适配、RC-ESDF、JPS、MINCO S3、独立 yaw、
footprint safety、Local Collision Repair、Goal Manager、全向 SE2 MPC、速度 frame
兼容层和四舵轮执行链。Nav2 只作为待清理的源码残余出现，不再作为目标架构、回退
架构或长期对照架构。

## 证据范围

后续 review、实施和验收只使用以下证据：

1. `/home/ats/ATS_2026_snetry_test/AGENTS.md`；
2. 当前活动源码、interface、launch、参数、manifest 和测试；
3. 本目录中的设计与任务账本；
4. 本轮实际执行的构建、单测、ROS graph、MuJoCo、HIL 和实车结果。

`docs/项目优化文档/nav2移植/**` 是此前迁移框架资料，不属于本设计。禁止读取其内容
补全当前事实，禁止引用其中状态或性能作为验收证据，也不再从本目录建立到该目录的
链接。

## 当前结论

| 范围 | 当前状态 | 证据边界 |
| --- | --- | --- |
| 专用实机自研入口 | `已实现-静态确认` | `real_robot_nav2_free.launch.py` 固定关闭 Nav2，启动 ATS action/MINCO/MPC，默认保留两级速度兼容层 |
| 自研 MuJoCo 回归入口 | `已实现-静态确认` | `test_mujoco_minco_mpc_chain.sh` 固定 ATS action，并拒绝 Nav2 节点和 `/plan` |
| 通用实机/导航 launch | `待清理` | 仍导入 `nav2_common`，保留 `launch_nav2` 与完整 server/lifecycle 分支 |
| 正式总参数 | `部分统一` | `node_params.yaml` 已有 ROGMap、adapter、MINCO、Goal Manager、MPC 段，但仍含 Nav2 段；behavior 和 ROGMap core 仍有第二来源 |
| 行为正式 profile | `部分完成` | 正式参数使用 `/ats_navigate_to_pose`，但 Nav2 action plugin、测试和构建依赖仍存在 |
| ROGMap/RViz | `部分完成` | 四类点云、`/rog_map/bounds` 和 `/goal_pose` 已接入；旧 costmap/MPPI display 与更多 bounds/health 仍待清理 |
| 执行授权重启交接 | `部分完成，已验证-单测` | `ExecutionCommand` 已携带 `manager_incarnation`；MPC 只在新实例先收到 `MODE_STOP` 后接收其 `MODE_EXECUTE`。Goal/candidate/serial 与内容 digest 尚未结构化贯通，未做进程重启注入 |
| 仓库级 Nav2-free | `未完成` | manifest、launch、YAML、行为、MuJoCo、loopback、`ats_nav2_plugins` 和 `trajectory_optimizer` 仍有活动依赖 |

上述结论来自当前源码静态交叉核对，不等价于本轮重新运行闭环。

## 1. 已完成内容

以下记录对应根仓 `df96ad7` 与导航仓 `718ceb6`，均已推送到各自的
`origin/develop`：

1. `ExecutionCommand` 增加 `manager_incarnation`。Goal Manager 以
   `steady_clock` 生成实例 token，并在启动时发布 `MODE_STOP`；MPC 以
   `(manager_incarnation, command_sequence)` 接收授权。新实例未经该实例
   `MODE_STOP` 的 `MODE_EXECUTE` 会归零，旧实例的延迟命令不能复活 tracker。
   该局部契约已由 Goal Manager pytest 和 MPC gtest 验证。
   `[已验证-单测, Confidence: High]`
2. 自研 MuJoCo 回归脚本接受稳定的 `TEST_PROFILE=default`。`default` 与
   `red_box` 名义路线均已在 `PLANNING_GRID_OWNER=rog_map` 下运行：终点误差分别为
   `0.003553 m` 与 `0.003347 m`，离散 footprint 冲突采样均为 `0`；运行图检查没有
   Nav2 server 或 `/plan`，并确认 planning grid、执行授权、MPC 和底盘输出的指定
   owner 唯一。`contact_violation_count=0` 只表示 MuJoCo telemetry 计数，不是独立
   physical contact evaluator。`[已验证-运行, Confidence: High]`
3. 回归脚本已实现但尚无成功运行证据的独立门禁包括 P2 的 `adapter_lease`、
   `service_timeout`、`input_stale`、`unknown`、`unreachable`，以及 P3 action 的
   `cancel`、`preempt`、`timeout`、`tf_failure`。`[已实现-静态确认, Confidence: High]`

### 运行期检查记录（2026-08-02）

已对 `adapter_lease` 进行独立启动尝试：

```text
ROS_DOMAIN_ID=241 P2_FAULT_CASE=adapter_lease P3_FAULT_CASE=none
ROS_DOMAIN_ID=243 ROS_LOCALHOST_ONLY=1 P2_FAULT_CASE=adapter_lease P3_FAULT_CASE=none
```

两次均在 ROS graph 建立前失败，CycloneDDS 报 `failed to enumerate interfaces for
"udp": -1`，节点随后报 `rcl node's rmw handle is invalid`；脚本最终为
`timeout waiting for node graph`。因此本轮没有触达 `adapter_lease` 注入，也没有获得
`emergency_stop -> /cmd_vel_mpc=0 -> /motion_control=0` 或恢复 generation 证据。
原始日志保存在 `/tmp/ats_minco_mpc_test_launch_241.log` 和
`/tmp/ats_minco_mpc_test_launch_243.log`，状态为
`[未验证, Confidence: High；环境阻塞]`，不能写成 fault 通过。

## 2. 当前下一阶段

下一阶段是 **P2/P3 运行期安全故障注入与恢复闭环**，而不是立即清理 Nav2 或重构总
YAML。每个 fault case 必须用新的 `ROS_DOMAIN_ID` 和新的无 viewer MuJoCo launch
独立运行，验证其完整链路：

```text
故障注入 -> ready/stale 或 action result 的预期状态
         -> /planner/emergency_stop=true
         -> /cmd_vel_mpc=0
         -> /motion_control=0
         -> 恢复后 generation 前进，且无新目标时旧 reference/旧授权不复活
```

只有这 9 个运行门禁全部通过，才进入 Goal Manager/MPC/serial 的真实进程重启注入；
只有安全和恢复门禁闭合，才允许开始正式入口、总 YAML 或 Nav2 依赖清理。
`[未验证, Confidence: High]`

## 唯一目标架构

```text
LiDAR + IMU
  -> Point-LIO + localization fusion
  -> /localization + /localization/status + /registered_scan
  -> ROGMap 概率占据/膨胀/3D ESDF
  -> ROGMap ground projection + terrain/static/unknown 融合
  -> /rc_esdf/planning_grid + 数值 signed distance/clearance
  -> ATS NavigateToPose + Goal Manager
  -> PlannerGoal
  -> JPS -> MINCO S3 -> 独立 yaw -> footprint gate/repair
  -> Goal Manager 提交安全 reference 与 ExecutionCommand
  -> 全向 SE2 MPC
  -> /cmd_vel_mpc -> fake-yaw -> chassis transform -> /cmd_vel
  -> serial/MuJoCo 唯一执行桥 -> 四舵轮底盘
```

Nav2 planner/controller/BT/costmap/lifecycle、`nav2_msgs` action 和 `/plan` 不属于该图。

## 阅读顺序

1. [项目优化总览与问题评分](./项目优化总览与问题评分.md)：当前事实、已完成项、剩余风险和优先级。
2. [Nav2移除与导航架构重构计划](./Nav2移除与导航架构重构计划.md)：唯一目标运行图、owner、接口和分仓清理顺序。
3. [导航参数与接口统一配置方案](./导航参数与接口统一配置方案.md)：`node_params.yaml` 单一权威与结构化接口账本。
4. [ROGMap与RViz可视化升级方案](./ROGMap与RViz可视化升级方案.md)：已接显示、剩余显示和数值接口隔离。
5. [分阶段任务清单与验收矩阵](./分阶段任务清单与验收矩阵.md)：可执行 TODO、DoD、测试、停止条件和提交拆分。
6. [下一阶段 P2/P3 安全故障注入与恢复实施提示词](./下一阶段Nav2移除与统一配置实施提示词.md)：下一会话可直接执行的提示词。

## 状态标签

| 标签 | 含义 |
| --- | --- |
| `已验证-运行` | 当前 revision 有本轮运行输出、终点或故障链证据 |
| `已验证-单测` | 当前 revision 的聚焦测试已通过 |
| `已实现-静态确认` | producer、consumer、launch 和参数已从源码确认，尚未在本轮运行 |
| `部分完成` | 主体已接入，但仍有第二权威、兼容分支或未闭合门禁 |
| `未实现` | 活动源码中不存在目标能力 |
| `未验证` | 缺少当前 revision 的运行、HIL、实车或独立 evaluator 证据 |

## 不可突破的边界

- Point-LIO/定位融合继续拥有 `/localization`、`/localization/status` 和
  `/registered_scan`；ROGMap 不承担定位。
- 活动 ROGMap 只有 `src/ats_sentry_nav/ats_rog_map`，不得复制参考实现。
- adapter、MINCO 和 MPC 禁止从 `/rog_map/esdf` 可视化点云反解析数值距离。
- 物理 occupancy、概率证据、ROG inflation、JPS clearance 和 footprint margin 分层。
- 静态细栅格按输出 footprint 覆盖面积保守聚合，保留 origin 与 yaw。
- 四舵轮状态为世界系 `[x, y, yaw]`，控制为车体系 `[vx, vy, wz]`；禁止差速、
  ICR 或 `vy=0` 约束。
- 实机默认保持 fake-yaw 和 chassis transform；固定雷达 profile 才能显式关闭。
- fake-yaw 关闭时保留 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转 TF；不得新增
  `base_footprint -> base_link` 第二发布者。
- source generation、adapter publication、MINCO local snapshot 和 localization epoch
  是不同版本域，未结构化贯通前不得宣称端到端编号一致。
- map/localization/gimbal/reference/command stale、unknown、unreachable、unsafe 或 MPC
  失败必须确定性归零；恢复不得复活旧目标、旧 reference 或旧授权。
- 未在 ATS 目标机测量前，不得把外部项目的频率、耗时和内存数据写成 ATS 实测。

## 文档维护规则

1. 当前事实必须由活动源码与测试/运行证据交叉确认。
2. 目标接口必须标记 `待实现`，不能与已有 `.msg/.srv/.action` 混写。
3. Nav2 残余只进入清理清单，不进入目标运行图。
4. 源码阶段完成后同步更新本目录 7 份文档；其它 README/docs 只在其实际接口受影响时更新。
5. 仓库级 Nav2-free 只有在 graph、manifest、launch、YAML、RViz、行为与测试均无
   活动 Nav2 依赖后才能声明完成。
