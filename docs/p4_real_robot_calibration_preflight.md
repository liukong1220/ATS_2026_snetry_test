# P4 实车标定准备与安全门禁

本文只定义 P4 第二阶段进入台架、HIL 和低速实车前必须采集的证据，不授权通电、自动运动或场地运行。MuJoCo 的 contact sample、四轮遥测和 swept footprint 仅用于缩小待标定范围，不能替代实车测量。

## 固定输入

每次采集前记录并冻结：根仓、导航仓、MuJoCo 仓以及实际参与运行的行为决策仓和 loopback 仓 commit；参数 YAML、行为树 XML 和 scenario version；机器人质量、载荷、电池状态、轮胎和场地；控制器固件；地图版本；时间同步状态；安全操作员和独立物理急停状态。采集文件必须保留原始 rosbag、参数转储、控制器反馈和执行命令序号。

## 不通电检查

1. 复核 `map -> odom`、`odom -> gimbal_yaw_odom` 和 `base_footprint -> base_link` 的唯一发布者、方向、时间戳与 frame 名称。
2. 验证 `/planner/execution_command` 是 Goal Manager 到 MPC 的唯一执行授权；`MODE_STOP`、命令序号回退、epoch 不一致和空 reference 必须使 MPC tracker 清空且输出零速度。
3. 对照车体状态世界系 `[x,y,yaw]`、控制车体系 `[vx,vy,wz]`，逐项检查正负方向、单位、轮位 `(+/-0.270,+/-0.270) m`、最终滚动半径 `0.0425 m` 和减速比定义。
4. 在执行器断开、抬轮或受约束状态下演练物理急停、远程急停、Goal Manager stop、MPC lease timeout 和底盘 watchdog；任一路径异常均停止后续测试。

## 待标定量与数据

| 项目 | 采集方法 | 产物 | 用途 |
| --- | --- | --- | --- |
| 轮端 RPM 与车体速度 | 单轴低速阶跃，分别前进、横移、旋转 | wheel RPM、舵角、IMU/外部位姿、命令时间 | 校验轮径、减速比、速度符号和轮速上限 |
| 轮速/舵速动态 | 多幅值正反阶跃，禁止人员进入区域 | 电流、电压、RPM、舵角、命令和反馈时间 | 拟合逐轮加速度、舵速、延迟和饱和模型 |
| 制动与指令时延 | 低速直线和横移 stop，重复采样 | 执行命令序号、`cmd_vel_mpc`、`motion_control`、RPM、位置 | 计算 sensor-to-actuator 年龄、制动距离和 safety margin |
| 定位质量 | 静态、受控平移/旋转和重定位重复试验 | `/odometry`、observation、协方差、质量、内点、epoch | 标定 covariance/quality 门限与 epoch 阈值 |
| footprint margin | 低速、隔离环境的已知墙体和窄门，不做贴边自动绕障 | 地图、真值距离、reference、接触/安全观察记录 | 校准矩形尺寸、tracking error 和 swept margin |

## 分级执行

1. HIL：执行器禁用或抬轮，先验证 topic、时间、序号、零速、RPM 和急停链，不能出现任何未命令运动。
2. 低能台架：一次只开启一个自由度，先验证 `vx`、再 `vy`、最后 `wz`；每个样本间停机检查电流、温度和反馈延迟。
3. 受控低速地面：定义封闭区域、最大速度/加速度/扭矩、观察员、物理急停和停止条件。先直线/横移/停止，再加入 yaw、规划和重定位。
4. 只有重复运行满足停止距离、无异常接触、无未解释饱和、定位/地图健康与命令唯一性后，才可扩大速度、路线或障碍复杂度。

## 稳定跟踪准入

实车路线不以“单次到达”为准入依据。进入每一级速度前，必须在前一级包络内固定 revision、参数、地图、载荷、电池和轮胎状态，按直线、横移、90 度、S 弯、窄道、坡道、rectangle 和 red_box 分层重复测量。

1. 使用外部真值或经标定的场地测量把 localization error 与 MPC tracking error 分开；无外部真值时不得宣称“定位不漂移”。
2. 每次运行都计算 `C_min > e_track_99 + e_loc_99 + v*tau_99 + d_brake + m_map`。任一分项缺失或预算为负时，不得提高速度。
3. 终端必须同时满足位置、wrapped yaw、线速度、角速度和 dwell；不能只以 action `SUCCEEDED` 或位置误差判断稳定。
4. wheel/steer saturation 必须记录起止时间、持续时间和占空比。累计 count 只能用于发现约束介入，不能证明 reference 不可行或执行器需要放宽。
5. 每个场景至少连续 `10/10` 通过且无人工接管、contact、定位跳变、错误 owner 或旧 reference 复活，才允许进入下一灰度级。
6. 任何硬件、固件、外参、轮胎、地图或安全相关参数变化都会失效当前准入记录，必须从相应低风险 gate 重新验证。

## 行为决策准入

行为树接入不能扩大底盘命令所有权。进入 HIL 前，正式行为 profile 必须只通过 `ats_navigation_interfaces/action/NavigateToPose` 驱动 Goal Manager，并在离线 tick、ATS loopback 和 MuJoCo 依次通过同一 scenario 矩阵。

1. branch halt、比赛结束、视觉 stale、补给/防守优先级切换和进程关闭均须取消活动 ATS action；Goal Manager 发布的新 `ExecutionCommand STOP` 才是停止运动的权威。
2. 正式 profile 不得包含直接底盘 `PublishTwist`，不得让 `cmd_spin` 在 MPC 后叠加 body `wz`；受击自旋若需要车体运动，必须成为受规划、footprint、执行器约束和 yaw authority 保护的正式 reference。
3. 行为层不得用位置距离替代 ATS action 成功。巡逻 cursor、任务 waypoint 和补给/退防完成只在 Goal Manager 已验证 position、yaw、terminal velocity 与 dwell 后推进。
4. 云台视觉/扫描请求不得覆盖 `BODY_YAW_FOLLOW` 的锁定要求；没有实际 gimbal feedback acknowledgement 时保持 `HOLD_SAFE_STOP`，不能以 BT 黑板状态伪造确认。
5. 固定 scenario 下，离线/loopback 至少连续 `20` 次得到一致的 branch/action 序列；MuJoCo 关键决策场景至少 `10/10` 无孤儿 goal、无抢占风暴、无旧 result/reference 复活并完成五级归零故障链。

立即停止条件包括：任何未命令运动、方向/幅值异常、无效状态被接受、TF/epoch 跳变、swept clearance 失效、执行命令租约失效未归零、轮端电流/温度异常、通信丢失或独立急停不可用。发生后保留日志并回到 HIL 或离线复现，禁止用放宽门限恢复测试。
