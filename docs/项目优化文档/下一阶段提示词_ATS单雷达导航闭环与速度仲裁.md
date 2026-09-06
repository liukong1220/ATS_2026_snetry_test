# 下一阶段提示词：ATS 单雷达导航闭环与速度仲裁

下面内容用于 Claude 的 P2 规划修复后回归。Claude 负责定位、实现、测试和证据整理；Codex 负责代码审查、文档收口以及 Git 提交和推送。

```text
[$develop-robot-vision-navigation]

工作目录：/home/ats/ATS_2026_snetry_test

开始前阅读 AGENTS.md、docs/nav2_to_3desdf_minco_mpc_optimization_direction.md 和
docs/项目优化文档/ATS导航剩余优化总TODO.md 的最新清单。先列出完成条件、精确文件范围、验证命令、
当前假设和风险转入条件，并保留工作区已有修改。

稳定架构边界：

单 LiDAR + Point-LIO
-> ROGMap 概率占据/膨胀/3D ESDF
-> terrain/static/unknown 地面融合 + RC-ESDF immutable snapshot
-> ATS Goal Manager -> JPS/A* -> MINCO S3 + independent yaw
-> footprint safety + Local Collision Repair
-> holonomic SE(2) MPC
-> fake/chassis yaw velocity transform
-> cmd_vel arbiter
-> lower-controller velocity interface

保持车体系 [vx,vy,wz]。不迁入差速、Ackermann、ICR 或 vy=0 约束。实机入口保持
launch_fake_vel_transform:=True、launch_chassis_vel_transform:=True；fake yaw 关闭时保留
gimbal_yaw_odom -> gimbal_yaw_fake 零旋转兼容 TF。单雷达 profile 不引入双雷达链。

当前可信状态（2026-09-06，活动证据窗口为 2026-08-30 至 2026-09-06）：

- Gazebo P1 在动态 TF age/staleness 门禁下 domain 147/149/151 连续三次通过；bridge 约 2 s 延迟在
  domain 143 复现过，尚无带失败运行资源测量的结论。
- MuJoCo P2 六故障 domain 154/156/158/166/168/170 为 6/6；single domain 178 的 reference、实际轨迹
  和 contact telemetry 均为零碰撞证据。
- red_box domain 176 目标 5 的同钟分析为 Q1 reference 安全、Q2 实际跟踪越界、Q3 未发现地图翻转；
  最大偏航误差 1.107 rad、横向误差 0.275 m、最小足迹间隙 -0.100 m。修复 owner 在 MPC 跟踪、执行限幅和
  停车包络，escape_from_contact_enabled 保持关闭。
- 实机/HIL 尚未运行；Gazebo 物理接触遥测仍未形成独立证据。
- 最近一周已补齐 `freeze` 终止日志和 `farthest-free --max-distance` 夹具参数，并完成
  reference/snapshot pairing、footprint parity 和 contact gate 回归；这些结果不改变 red_box 与失败 leg 的未完成边界。

速度链契约：

ats_swerve_mpc -> /cmd_vel/autonomy_raw
-> fake yaw transform -> /cmd_vel/autonomy_gimbal
-> chassis yaw transform -> /cmd_vel/autonomy

teleop_twist_keyboard -> /cmd_vel

/cmd_vel + /cmd_vel/autonomy
-> cmd_vel_arbiter -> /cmd_vel/selected
-> serial or simulation final consumer

检查重点：

1. /cmd_vel/selected 只保留 arbiter publisher；实机串口、MuJoCo bridge、Gazebo adapter 只订阅 selected。
2. manual fresh 优先；manual timeout 后归零。auto 依赖新鲜 ExecutionCommand，STOP、manager incarnation、
   lease、急停、定位/地图失效和链路断开都会作废旧 auto。
3. DOWN->UP 只接受恢复后到达的新命令，断链前缓存不恢复执行。
4. fake/chassis transform 的 topic、frame 和速度分量保持一致，避免重复 base_footprint -> base_link TF。
5. 缺少 required big-yaw 反馈时，Gazebo chassis 与 /motion_control 使用同一份全零输出。
6. recorder 保存动态 edge source stamp、age、更新计数、reference/snapshot identity 和 tracking error；
   TimePointZero 查询仅用于链路存在性信息；动态新鲜度由 source stamp、age 和更新计数门禁判断。

回归安排：

- 运行 arbiter focused GTest，覆盖优先级、timeout、STOP、incarnation、断链恢复和 [vx,vy,wz]。
- 运行四环境 launch/config contract，并核对 publisher/subscriber、frame、QoS 和串口 topic。
- Gazebo P1 使用新 domain、固定默认 profile 和动态 TF 双门禁，保存每次 artifact。
- MuJoCo 独立运行 single、red_box 和六个 P2 fault；每例分离机器人状态。
- `python3 scripts/test_footprint_evaluator.py`、`bash scripts/test_footprint_evaluator_parity.sh`、
  `python3 scripts/test_analyze_nav_tracking.py`、`bash scripts/test_gazebo_dynamic_tf_gate.sh` 和
  `bash scripts/test_mujoco_contact_gate.sh` 均纳入离线验证。
- footprint_collisions=0 只代表几何采样结果，物理接触单独记录；缺少接触 evaluator 时结论写为未验证。

降低 freshness、footprint、障碍阈值或开启 escape 会改变安全边界；topic 存在、编译成功或单次运动不构成
闭环准入。目标终态误差与 recorder 结束时定位误差分别记录。

交付报告区分已验证、已实现未运行、推断和未实现，列出精确文件、命令与退出码、domain artifact、终点误差、
owner 数量、零速链、reference/actual 碰撞、接触证据和残余风险。完成后输出 READY_FOR_CODEX_REVIEW，交付时 Git
工作区保持无新增提交。
```
