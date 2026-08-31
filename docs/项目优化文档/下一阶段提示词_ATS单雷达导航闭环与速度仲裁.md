# 下一阶段提示词：ATS 单雷达导航闭环与速度仲裁

下面内容可直接交给 Claude，用于 P2 规划修复完成后的速度链与单雷达闭环回归。

```text
[$develop-robot-vision-navigation]

继续 /home/ats/ATS_2026_snetry_test 的 ATS 单 LiDAR、四驱四转导航优化。先读取 AGENTS.md、
docs/nav2_to_3desdf_minco_mpc_optimization_direction.md 和
docs/项目优化文档/ATS导航剩余优化总TODO.md 的 2026-08-31 置顶结论。

Claude 负责实现、测试和交付报告；Git 暂存、提交与 push 交给 Codex。建议先列出 DoD、精确文件范围、
验证命令、假设和风险转入条件，并保留用户已有修改。

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

保持车体系 [vx,vy,wz]，避免引入差速、Ackermann、ICR 或 vy=0。实机默认保留
launch_fake_vel_transform:=True、launch_chassis_vel_transform:=True；fake yaw 关闭时保留
gimbal_yaw_odom -> gimbal_yaw_fake 零旋转兼容 TF。单 LiDAR profile 不迁入参考工程的双雷达链。

当前可信状态：

- Gazebo P1 已在 domain 127/129/131 连续三次通过。P1 action 误差为 0.07443/0.06013/0.06239 m；
  TRACKING 均为 600/600，TF 链建立后的查询失败为 0。
- Gazebo IMU 使用 m/s^2，sim launch 已覆盖 Point-LIO acc_norm=9.81、satu_acc=30.0；实机配置未改。
- Transport 与 ROS LiDAR cadence 相近只支持这三次运行没有频率退化，不代表端到端延迟为零。
- 当前 TF 证据支持链建立后可查询，动态 TF stamp/age 尚无独立门禁。
- MuJoCo P2 在安全修正后为 5/6：unknown 故障主体通过，但恢复新目标因起点 footprint 冲突失败。
  red_box domain 138 的目标 1–4 零碰撞成功，目标 5 因停车后进入墙侧接触区失败。
- escape_from_contact_enabled=false、ego_blocked_escape_enabled=false；任何 footprint_collisions>0 不纳入准入。

速度链契约：

ats_swerve_mpc -> /cmd_vel/autonomy_raw
-> fake yaw transform -> /cmd_vel/autonomy_gimbal
-> chassis yaw transform -> /cmd_vel/autonomy

teleop_twist_keyboard -> /cmd_vel

/cmd_vel + /cmd_vel/autonomy
-> cmd_vel_arbiter -> /cmd_vel/selected
-> standard_robot_pp_ros2 serial 或仿真 final consumer

优先检查：

1. /cmd_vel/selected 只有 arbiter 一个 publisher；实机串口、MuJoCo twist_to_motion_ctrl、Gazebo adapter
   各自只订阅 selected，避免从 /cmd_vel_mpc 或 /motion_control 绕过仲裁。
2. manual fresh 时优先；manual timeout 后归零。auto 依赖新鲜 ExecutionCommand，STOP、新 manager incarnation、
   lease 过期、急停、定位/地图失效和串口断链都会作废旧 auto。
3. DOWN->UP 之后只接受恢复后到达的新命令，断链前缓存不复活。
4. fake/chassis transform 保持 topic、frame 和速度坐标契约；避免重复发布 base_footprint -> base_link。
5. required big-yaw feedback 缺失时，Gazebo chassis 与 /motion_control 同时为零。
6. TF recorder 增加动态 edge 的 source stamp、age、连续更新计数；warm-up 仍以首次建立分界，链从未建立时
   保持 fail-closed。TimePointZero 查询不单独作为动态新鲜度结论。

回归建议：

- arbiter focused GTest：manual/auto 优先级、timeout、STOP、incarnation replay、断链恢复、两源归零；
- launch/config contract：四环境的 publisher/subscriber 唯一性与 frame 账本；
- Gazebo P1：新 domain 三次，固定默认 profile，记录 dynamic TF age 与既有 cadence/action 门禁；
- MuJoCo：single、red_box 和六个 P2 fault 使用独立新 domain；
- 实机/HIL 尚未运行时，键鼠到串口只标为已实现未运行；
- footprint_collisions=0 不替代物理接触结论。

建议避免再做以下工作：通过 Direct bridge、QoS、freshness timeout、footprint margin 或 obstacle threshold
放宽换通过；改实机 IMU 标定以适配 Gazebo；把 action 终态误差与 60 秒 recorder 结束时定位误差混写；
把 topic 存在、编译成功或单次运动写成闭环准入。

交付报告请区分已验证、已实现未运行、推断与未实现，列出精确文件、命令、退出码、每个 domain artifact、
终点误差、owner 数量、zero-speed 链、未运行项和残余风险。完成后输出 READY_FOR_CODEX_REVIEW，保持 Git 无写入。
```
