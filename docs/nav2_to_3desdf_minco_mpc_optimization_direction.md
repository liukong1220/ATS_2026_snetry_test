# ATS 自研导航 V1 当前状态与优化方向

> 更新时间：2026-08-15
> 本页只记录当前准入状态、不可破坏的架构边界和下一执行入口。历史阶段流水账已从活动文档移除，
> 仍可由 Git 历史和专项准入记录追溯。

## 1. 当前架构

```text
Point-LIO /localization + /registered_scan
-> ROGMap 概率占据/膨胀/3D ESDF
-> ats_rog_map_adapter 地面投影与 static/terrain/slope/unknown 融合
-> PlanningMapSnapshot + RC-ESDF
-> ATS Goal Manager
-> JPS/A* fallback
-> MINCO S3 + independent yaw
-> oriented footprint + sampled swept safety + optional local repair
-> holonomic SE(2) iLQR MPC
-> /cmd_vel_mpc -> /motion_control
-> four-wheel independent steer/drive chassis
```

活动实现不得替换 Point-LIO、ROGMap、RC-ESDF、JPS、MINCO S3、独立 yaw、footprint gate、
Local Collision Repair 或全向 SE(2) MPC。控制保持车体系 `[vx,vy,wz]`，禁止差速、Ackermann、
ICR 或 `vy=0`。

## 2. 当前准入结论

| 阶段 | 已完成 | 当前缺口 | 状态 |
| --- | --- | --- | --- |
| P2 | ROGMap/adapter 数值链、唯一 planning owner、immutable snapshot、fail-stop、直线 Gazebo 运行 | freshness、最终 nominal/red-box、当前 revision fault matrix、clearance/contact、RViz 滑窗验收 | **未通过** |
| P3 | ATS action、feedback、cancel/preempt/timeout、Goal Manager watchdog、Gazebo 默认无 Nav2 启动路径 | 完整 action 生命周期、扩大路线、red-box、无 Nav2 server graph 运行证据 | **未通过** |
| P4 | 四舵轮仿真、速度 owner、矩形 footprint、自适应 sampled sweep | 连续 swept 误差上界、独立 contact evaluator、制动/延迟、HIL、实车 | **未通过** |
| QP | OSQP v1.0.0、固定 CSC、warm-start ABI、same-snapshot shadow、数值防御 | 真实 map/collision gate、稳定 solved、paired runtime、主链 fallback 和切换准入 | **仅 Shadow** |

## 3. 最新有效证据

- 根仓 `90b5bfbceb88`、导航仓 `5ea786eb2e70`、Gazebo fork `9ed6c41650e6`、MuJoCo
  `e3d6ea7a5e61` 已同步各自远端；
- `ats_robot_description` 本地 `dea591e53fa0` 仍领先远端 `ed293ca0613e` 一个提交，干净复建未通过；
- 两份 RViz 已配置全局 `/rc_esdf/signed_distance_grid` 和局部 ROGMap debug，但当前 revision 尚无
  全局 ESDF/三米滑窗运行截图；
- MINCO 已有 geometry preprocessor、curvature-aware time allocation、ESDF refinement 和 quality
  telemetry，聚焦 CTest 已通过；
- Gazebo domain `228/229` 的短直线 action 成功，终点误差约 `0.060/0.045 m`；
- domain `230` 的直线 candidate 长度比 `1.000`、曲率为零，但 `/localization` wall interval
  `p50/p95/p99=0.371/0.994/1.612 s`，adapter 反复 `ready=false`，action fail-closed；
- production MINCO node 尚未把实时 `InitialKinematicState` 传入 optimizer；几何质量指标主要用于
  telemetry，尚未形成完整候选接受门禁；
- Gazebo runner 的 `TEST_PROFILE` 尚未拥有实际 corner/S/narrow/red-box 场景逻辑；
- QP node 仍固定 `map_fresh=false`、`collision_free=false`，`solver_mode=qp` 显式拒绝。

组件行为由源码与聚焦测试支持；domain 数值来自已保存运行记录。freshness 的唯一根因仍未确定，必须
逐级测量 `/lidar_odometry -> /odometry -> /localization -> status -> adapter`，不能仅凭相关性归因
Point-LIO、DDS、仿真 RTF 或 CPU 争用中的任一项。

## 4. 不可放宽的安全边界

- unknown、occupied、outside-map、ESDF sign/gradient、snapshot freshness 和 lease 继续 fail-closed；
- `/rog_map/esdf` 只是调试点云，不能作为数值规划输入；
- `/cmd_vel_mpc` 和 `/motion_control` 必须各自保持唯一发布者；
- 地图、定位、TF、reference、ExecutionCommand、gimbal 任一不健康都不得继续运动；
- 急停清空 tracker，恢复后旧 reference 不得复活；
- 不能提高 freshness timeout、QP iteration/deadline/residual 来绕过失败；
- planner collision 为零不能推出 Gazebo physical contact 或实车碰撞为零；
- 未在目标机测量前不得引用报告中的 `50 Hz`、`6 ms` 或内存数据。

## 5. 下一优化顺序

1. 推送并干净复建 `ats_robot_description`；
2. 定位并修复 Gazebo localization freshness 首个违反者；
3. 将实际运动状态接入 MINCO 四条生产优化路径；
4. 把几何质量 telemetry 升级为按路径类别生效的候选门禁；
5. 实现真实 Gazebo straight/corner/S/narrow/nominal/red-box runner；
6. 重跑当前 revision 的 P2 名义、边界和故障矩阵；
7. 完成 RViz 全局/局部滑窗、clearance、continuous swept 和 contact；
8. 完成 P3 Nav2-free action 生命周期；
9. 完成长时间性能、MuJoCo 跨后端和 HIL；
10. P2/P3/P4 通过后再推进 QP 主链和低速实车。

详细任务、DoD、验证命令和停止条件见：

- [ATS 导航剩余优化总 TODO](项目优化文档/ATS导航剩余优化总TODO.md)
- [下一阶段新对话提示词](项目优化文档/下一阶段提示词_ATS导航仿真闭环与准入.md)
- [LTV-QP 后端准入记录](ats_swerve_mpc_ltv_qp_backend_admission.md)

## 6. 文档职责

- 本页：只记录当前阶段结论和架构边界；
- 总 TODO：只记录未完成任务、依赖、DoD 和状态，不累计完整日志；
- QP backend admission：记录后端来源、数值准入和 QP runtime 边界；
- `log/` artifact：保存原始运行结果，不提交大体积生成物；
- Git 历史：保留已退役阶段文档和过去实验的可追溯性。

任何新源码行为修改后，必须用该 revision 重跑相应仿真，不得复用修改前结果作为最终证据。
