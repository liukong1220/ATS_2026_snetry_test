# Gazebo 历史接口与恢复边界

## 适用状态

2026-11-24 审查：Gazebo 自用户 2026-10-31 决定起冻结未启用。现存 `ats_gazebo_nav.launch.py` 尚引用已删除的独立 adapter、`get_ground_projection` 服务和 `/rc_esdf/planning_grid`。旧入口不能直接作为当前导航启动步骤；本文仅保留恢复迁移契约与边界。

当前活动链见 [导航定位与轨迹链路](导航定位与轨迹链路.md)，最终验收使用 [MuJoCo 2027 方案](../nav_acceptance/20261116_2027_final_test_plan.md)。本阶段不修改或启动 Gazebo。

## 重新启用前的接口迁移

| 旧接口/假设 | 当前导航契约 | 待验证项 |
| --- | --- | --- |
| 独立`ats_rog_map_adapter`包与投影service | adapter位于`ats_rog_map`同一进程，直接读取数值投影 | 删除旧启动依赖，核对地图唯一owner |
| `/rc_esdf/planning_grid` | `/rog_map_adapter/planning_snapshot`单消息含占据、距离、梯度、来源/发布/epoch | producer与consumer同源，陈旧/迟到数据拒绝 |
| owner二选一 | 当前规划地图唯一由进程内adapter发布 | 旧`planning_grid_owner`不能证明兼容 |
| 旧低速、旧雷达频率与显示说明 | 共享profile暂定5 m/s、3 rad/s、2 rad/s²；固定底盘雷达 | Gazebo传感器和执行器须另行核验，不能由MuJoCo更新推导已同步 |
| 红框作为最终必过项 | 当前为2027真实隧道与对称点导航速度 | 旧场景记录只归档，不替代当前套件 |

## 保留的边界

恢复时必须保持GT/LIO producer互斥、`map->odom`唯一、点云/里程计frame和时间一致；速度沿MPC、arbiter、最终bridge传递。底盘固定雷达的外参和旋转语义须核对实际模型。地图unready/stale、无路、unsafe、授权或控制失败均确定性零速，ready恢复不得复活旧reference。

未来Gazebo验收须使用新domain和新进程，按实际launch核对`/odometry`、`/registered_scan`、`/localization/status`、规划快照、执行授权、reference、自主/selected速度及底盘输出。无独立contact evaluator时物理接触保持未验证。

## 历史证据（本次未复现）

旧d220无GUI、关闭planning/GICP的启动仅证明world、机器人、传感器bridge和relay曾启动，20 s后超时退出。旧d232的2027 GT/direct-LiDAR运行曾记录持续`/registered_scan`及GICP accepted/CONFIRMED。这些不证明当前revision的完整导航兼容，也不证明隧道或对称点速度通过。

P3 还需无 Nav2 服务进程、MINCO 不订阅 `/plan`、目标入口不调用 `nav2_msgs/action/NavigateToPose`、自研 action 生命周期与行为树全链共同通过。物理接触（独立评估器）与实车硬件联调目前仍未验证（自适应连续扫掠足迹门已在 Wave 3 落地）。
