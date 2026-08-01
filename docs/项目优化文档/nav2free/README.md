# ATS 导航项目优化文档

本目录集中保存 ATS 2026 四驱四转哨兵从 Nav2 双栈过渡到自研 ROGMap、JPS、MINCO S3、全向 SE2 MPC 导航链的审查、迁移和验收资料。

截至 2026-08-01，本目录同时保存“架构冻结与实施账本”和本轮实施证据。正式
`node_params.yaml`、Nav2-free launch、ATS action、ROGMap RViz profile 与 MuJoCo
action-only 脚本已经进入当前工作树；静态检查和 Fast DDS 本机 MuJoCo 回归见
`../nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md` 第 5.16 节。该证据
不等价于 CycloneDDS、实车或完整 P3 验收通过。

## 阅读顺序

1. [项目优化总览与问题评分](./项目优化总览与问题评分.md)：当前缺陷、风险分数、优先级和未接接口。
2. [Nav2移除与导航架构重构计划](./Nav2移除与导航架构重构计划.md)：目标运行图、删除清单、分仓修改顺序。
3. [导航参数与接口统一配置方案](./导航参数与接口统一配置方案.md)：以 `node_params.yaml` 为唯一总参数文件的迁移规则。
4. [ROGMap与RViz可视化升级方案](./ROGMap与RViz可视化升级方案.md)：ROGMap 原项目风格图层、RViz 面板和数值/可视化边界。
5. [分阶段任务清单与验收矩阵](./分阶段任务清单与验收矩阵.md)：可执行 TODO、DoD、测试命令、停止条件和提交拆分。
6. [下一阶段Nav2移除与统一配置实施提示词](./下一阶段Nav2移除与统一配置实施提示词.md)：下一次新对话可直接使用的 Goal、DoD、范围、约束、验证和交接事实。

## 历史与专项资料

- [ATS 自研导航 V1 当前状态与下一阶段交接](../nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md)
- [P4 实车标定准备与安全门禁](../nav2移植/p4_real_robot_calibration_preflight.md)
- [P4 第四阶段稳定跟踪、行为决策与双仿真提示词](../nav2移植/p4_stage4_stable_tracking_prompt.md)
- [P5 实车化整改提示词](../nav2移植/p5_real_robot_hardening_prompt.md)
- [P6 实车接口连通提示词](../nav2移植/p6_real_robot_interface_integration_prompt.md)

## 证据标签

| 标签 | 含义 |
| --- | --- |
| `已验证-静态` | 已从活动源码、launch、参数或 interface 定义确认，不等价于运行通过 |
| `已验证-单测` | 已有针对性单元测试证据；仍不替代跨进程闭环 |
| `已实现未运行` | 源码存在，但本轮没有运行构建、仿真或实车验证 |
| `未实现` | 活动源码中不存在目标能力 |
| `待决` | 会影响删除范围或接口设计，需要在对应阶段开始前冻结 |

重要结论必须至少有两类独立证据才能升级为闭环结论。例如源码与单测、launch 与 ROS graph、日志与终点测量。只有静态证据时统一标注 `[Confidence: Medium]` 或明确写出证据边界。

## 不可突破的边界

- Point-LIO 继续提供 `/localization` 与 `/registered_scan`，ROGMap 不承担定位。
- ROGMap 活动实现仅为 `src/ats_sentry_nav/ats_rog_map`。
- 规划数值距离只能来自结构化服务/消息，禁止反解析 `/rog_map/esdf` 可视化点云。
- 四舵轮状态为世界系 `[x, y, yaw]`，控制为车体系 `[vx, vy, wz]`，不引入 `vy=0`。
- 实机主入口默认保持 `launch_fake_vel_transform:=True` 和 `launch_chassis_vel_transform:=True`；Nav2 移除不能隐式关闭速度 frame 兼容层。
- fake-yaw 关闭时保留 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转 TF；不得新增重复 `base_footprint -> base_link` 发布者。
- 固定雷达迁移只能通过显式 profile 关闭兼容层，并在关闭前证明 topic、TF 和下游车体系速度契约完整。
- `ROGMap source generation`、adapter publication 和 `MINCO local snapshot generation` 是三个不同版本域。
- 未完成 P3 graph 门禁前，不得声称项目已彻底 Nav2-free；未在目标机测量前，不得把参考文章的 50 Hz、约 6 ms 写成 ATS 实测性能。

## 当前文件范围

`nav2free/` 汇总架构审查、迁移计划、统一配置、ROGMap/RViz、任务矩阵和下一阶段
提示词；`../nav2移植/` 保存此前 P4/P5/P6 与 V1 状态资料。旧文档由用户从
`docs/` 移入分类目录，不恢复到根层。

2026-08-01 同步更新了根 README、相关分仓 README 和外层专题 docs：普通入口与
MuJoCo 都保留显式 Nav2 baseline；专用正式实机入口和 P3 MuJoCo 回归入口固定使用
ATS action、`launch_nav2:=false` 与 ROGMap planning-grid owner。当前工作树尚未完成
分仓提交；CycloneDDS 双进程 discovery、Nav2 baseline、RViz live、有效参数 dump 与
实机门禁仍需独立验证。
