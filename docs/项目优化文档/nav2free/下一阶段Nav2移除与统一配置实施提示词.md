# 下一阶段 Nav2 移除与统一配置实施提示词

更新时间：2026-07-31。

本文件用于下一次新对话直接启动实施。它不是完成报告；开始时必须重新读取当前
工作树、`AGENTS.md`、launch、参数与测试结果，不能沿用本次静态审查作为运行证据。

## 可直接使用的提示词

```text
新线程。除下面的 handoff 外，忽略旧线程假设；所有事实以当前工作区源码、Git
状态和本轮实际运行结果为准。

工作区：/home/ats/ATS_2026_snetry_test

Goal:
在不破坏 ATS 四驱四转舵轮、Point-LIO、ROGMap 数值 projection、RC-ESDF、
JPS、MINCO S3、独立 yaw、footprint safety、Local Collision Repair 和全向 SE2
MPC 的前提下，实施下一阶段 Nav2 移除与正式实机参数统一：

1. 将正式实机 MINCO、Goal Manager、MPC 以及相关 ROGMap adapter 参数迁入
   src/ats_sentry_bringup/params/node_params.yaml，使它成为正式实机唯一总 YAML；
2. 保留 .msg/.srv/.action 作为 schema 唯一权威，总 YAML 只统一参数、topic、
   frame、QoS、timeout、owner 和 profile；
3. 让正式入口和自研链完全不依赖 Nav2 运行节点、/plan 或 nav2_msgs action；
4. 保留独立且明确命名的 Nav2 baseline profile，直至 P3 回归通过，再分阶段移除
   公共算法的 Nav2 继承、CMake/package 依赖和旧资源；
5. 新增/更新 ROGMap RViz profile，复用原项目的占据、膨胀、unknown、ESDF 和
   bounds 显示方式；规划继续只消费数值 projection service，禁止反解析点云；
6. 完成最窄构建、单测、launch 检查、MuJoCo 默认/红框/fault injection、文档和
   分仓提交推送。

Definition of Done:
- 每轮修改前先输出 DoD、精确文件范围、验证清单、假设/未验证项/停止条件；
- 普通正式实机入口选择 Nav2-free profile 时 launch_nav2=false，运行图没有
  bt_navigator、planner_server、controller_server、behavior_server；
- MINCO 正式参数中 goal_topic/global_plan_topic 为空，只消费
  /ats_goal_manager/planner_goal；
- 行为正式树只调用 /ats_navigate_to_pose，feedback/result/cancel/preempt/timeout
  都有单测或运行证据；Nav2 action 只能出现在显式 baseline profile；
- node_params.yaml 包含正式实机 minco_planner、ats_goal_manager、ats_swerve_mpc
  和需要统一的 adapter 段，launch 实际只加载这一份正式配置；
- 原 *_reality.yaml 若删除或降级为示例，必须先通过 effective param dump/测试证明
  没有第二权威；不得先删后接；
- planning_grid_owner 只允许 rc_esdf|rog_map，rog_map profile 下
  /rc_esdf/planning_grid publisher 数为 1 且 owner 为 adapter；
- ROG projection response 同一 sample 携带 grid、signed distance、gradient 和
  ROG generation；adapter 不订阅 /rog_map/esdf；
- 单次规划内 JPS、二维 RC-ESDF、MINCO clearance、footprint gate/repair 共用同一
  MINCO immutable snapshot；不宣称 ROG/adapter/MINCO 编号端到端相同；
- map unready/stale、全 unknown、unreachable、projection timeout、Point-LIO stale、
  adapter heartbeat 中断和 unsafe trajectory 都使 emergency_stop=true、控制和底盘
  在 deadline 内归零；恢复后旧 response/generation/reference/command 不复活运动；
- ROGMap RViz 能显示 /rog_map/occ、/rog_map/inf_occ、/rog_map/unk、/rog_map/esdf
  及 bounds Marker，并明确 display 颜色、尺寸、alpha、frame 和开关；
- P3 扩大矩形和红框都从 ATS action 入口通过；若仍使用 NavigateToPose 或 /plan，
  只能标 P2/对照通过；
- 更新所有受影响 README、docs/项目优化文档以及
  docs/项目优化文档/nav2移植/nav2_to_3desdf_minco_mpc_optimization_direction.md 的新位置/内容；
- 每个独立仓库按内容拆分中文提交并推送 develop -> origin/develop，报告 commit ID。

Scope:
优先定位并仅在证据要求时修改：
- src/ats_sentry_bringup/launch/bringup.launch.py
- src/ats_sentry_bringup/launch/real_robot_nav2_free.launch.py
- src/ats_sentry_bringup/params/node_params.yaml
- src/ats_sentry_bringup/rviz/*.rviz
- src/ats_sentry_nav/ats_nav_bringup/launch/*.py
- src/ats_sentry_nav/minco_planner/{config,launch,src,include,test}
- src/ats_sentry_nav/ats_goal_manager/{config,src,include,test}
- src/ats_sentry_nav/ats_swerve_mpc/{config,launch,src,include,test}
- src/ats_sentry_nav/ats_rog_map/{config,launch,src,include,test}
- src/ats_sentry_nav/ats_rog_map_adapter/{config,launch,src,include,test}
- src/ats_sentry_nav/ats_navigation_interfaces
- src/ats_sentry_behavior/{params,behavior_trees,plugins,include,test,CMakeLists.txt,package.xml}
- src/sim/ats_mujoco_sim/launch
- scripts/test_mujoco_minco_mpc_chain.sh 及其直接 helper
- 受影响 README 与 docs

Constraints:
- 开始前读取 /home/ats/ATS_2026_snetry_test/AGENTS.md 并逐条遵守；
- 先运行根、导航、行为、MuJoCo、loopback、串口仓的 git status --short --branch；
- 未知修改/未跟踪文件属于用户，不删除、不覆盖、不提交；禁止 git add . / -A；
- 活动 ROGMap 只在 src/ats_sentry_nav/ats_rog_map，不复制参考实现；
- Point-LIO 保持 /localization 和 /registered_scan owner；
- 不从 /rog_map/esdf PointCloud2 反解析 signed distance；
- signed distance、unknown、gradient、outside-map 和 footprint 语义必须保留；
- 静态细图到 planning grid 采用 footprint overlap 保守聚合，保留 origin/yaw；
- 车体控制保留 [vx,vy,wz]，禁止 vy=0、差速或 ICR 假设；
- 普通实机兼容 profile 的 launch_fake_vel_transform 和
  launch_chassis_vel_transform 默认仍为 True，固定雷达迁移只能用显式 profile；
- fake-yaw 关闭时保留 gimbal_yaw_odom -> gimbal_yaw_fake 零旋转 TF；
- 不新增第二个 base_footprint -> base_link TF publisher；
- callback 并发地图/轨迹使用 immutable snapshot 或明确同步；
- deadline/lease 用 steady clock，观测 stamp 保留 ROS/sim time；
- ready=true 是 heartbeat，不是永久状态；
- 最终安全 reference 在提交点统一重定时，并通过 atomic ExecutionCommand 授权；
- dead code 只标记 [Dead Code Suggestion]，没有明确授权不删除；
- 不引用参考项目 50 Hz、约 6 ms 或内存数字作为 ATS 实测。

Implementation order:
1. 重新审计当前 Git 状态、HEAD、分支、有效 launch 与参数加载图；为每条重要结论
   建立“claim -> 源码证据 -> 测试证据 -> confidence”账本。
2. 在将被修改且 tracked tree 稳定的仓库创建中文基线提交；不得纳入用户文件。
3. 先补参数 schema/重复 key/effective-load 测试，再迁移正式实机参数到总 YAML。
4. 修改 launch，使正式 profile 只加载总 YAML；逐节点 ros2 param dump 对照。
5. 将行为正式 profile 与 ATS action 固化；补 cancel/preempt/timeout 和 server restart
   incarnation 风险测试。不要因插件文件存在就直接删除 Nav2 构建依赖。
6. 修复 MuJoCo 自研入口，使 launch_nav2=false 时不依赖 /plan，并让测试脚本从
   ATS action 发目标；保留单独 Nav2 baseline 脚本。
7. 接入 ROGMap RViz displays 和 bounds Marker；可视化 topic 与规划数值接口分离。
8. 先最窄构建/单测，再 launch --show-args，再无 viewer MuJoCo，再红框与分故障注入。
9. 最后一次地图/规划/安全/控制源码修改后，重跑对应闭环，不复用旧结果。
10. 更新文档，只写本轮实际结果；按仓、按接口/算法/安全/仿真/文档拆分提交并推送。

Validation:
静态与构建：
source /opt/ros/humble/setup.bash
python3 -m py_compile <changed_launch_files>
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1
colcon test --base-paths src --packages-select <targets>
colcon test-result --test-result-base build/<package> --verbose
ros2 launch <package> <launch> --show-args
git diff --check

参数/运行图：
- 在隔离 ROS_DOMAIN_ID 下启动，不使用旧 daemon 结果；
- ros2 node list --no-daemon 确认 Nav2 节点不存在；
- ros2 topic info -v 确认 planning grid、cmd_vel、motion_control 唯一 owner；
- ros2 param dump 分别核对 MINCO、Goal Manager、MPC、adapter 的 effective values；
- rg 检查正式 launch/参数/树不再引用 /plan、navigate_to_pose、Nav2 server。

MuJoCo：
- scripts/test_mujoco_nav_chain.sh 只作为 Nav2 baseline，必须完整执行；
- PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box
  GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh；
- 将上述脚本改为 ATS action 后，再执行扩大矩形和红框 P3 回归；
- projection timeout、Point-LIO stale、adapter heartbeat 中断、unknown、unreachable
  分别使用新 ROS_DOMAIN_ID 与新 MuJoCo launch 注入；
- 记录 ready=false -> emergency_stop=true -> command=0 -> motion_control=0 的实测时间；
- 成功场景记录终点坐标/误差、路径/reference 点数、footprint 冲突采样、重规划/
  恢复次数和物理 contact evaluator；无 evaluator 写“未验证”。

RViz：
- 先用 ros2 topic echo/info 验证非空、frame、QoS；
- GUI 环境截图检查 occ/inf_occ/unk/esdf/bounds 不重叠且颜色可区分；
- headless 下至少验证 RViz config 可解析、topic 存在且 PointCloud2 非空；
- 不把 RViz 显示成功当作数值 ESDF 规划正确。

Signals:
每 30-60 秒用中文简短报告：正在定位的 owner、已经确认的证据、即将修改的文件、
当前测试阶段。遇到失败先保留日志并定位，不要跳过后直接扩大范围。

Required final report:
- 修改文件按仓分组；
- 实际命令与逐项结果；
- 默认/红框终点、误差、路径/reference 点数、footprint/contact 指标；
- topic/frame/QoS/owner/generation/effective-param 核对；
- stale/unknown/unreachable/timeout/heartbeat 归零与恢复结果；
- 已验证、已实现未运行、未实现、残余风险；
- 未运行项及原因；
- 每仓基线/最终 commit ID、origin/develop 一致性和 push 结果。

Handoff facts to verify, not blindly trust:
- 普通 bringup 在 2026-07-31 静态审查时默认 launch_nav2=true、
  planning_grid_owner=rc_esdf、两级速度 transform=true；
- real_robot_nav2_free 当时固定 launch_nav2=false、launch_swerve_mpc=true、
  planning_grid_owner=rog_map，并关闭两级 transform；
- MuJoCo navigation 当时默认 launch_nav2=true，MPC 模式说明仍可能依赖 /plan；
- 正式自研三节点当时分别加载自己的 *_reality.yaml，总 YAML 尚未统一；
- P2 ROGMap YAML 当时 visualization.enable=false；
- 当前最高风险包括跨 topic/地图快照一致性、候选与正式 reference 时间轴、
  Goal Manager 重启后的 ID incarnation、定位跳变后的封锁持续时间。

现在先输出本轮 DoD、精确文件范围、验证命令、假设/停止条件，然后立即开始
“定位 -> 修改 -> 构建 -> 单测 -> 仿真 -> 文档 -> 分仓提交”，不要只给方案。
```

## 使用说明

1. 新会话第一条消息粘贴上面代码块。
2. 若只希望 review，不实施，把 `Goal` 第一行改为“只做 review，不修改代码”，并
   删除 Implementation order 中的编辑/提交步骤。
3. 若只做参数统一，显式把 MuJoCo/Nav2 删除/RViz 标为 out of scope，避免一次变更
   同时跨越过多安全边界。
4. 新会话必须重新读取 `AGENTS.md` 和仓库状态；本文件中的 2026-07-31 事实会过期。

## 推荐拆分

为降低一次变更的 blast radius，推荐按以下提交序列实施：

1. `[接口]` 参数加载/重复 key/effective param 测试；
2. `[配置]` 正式实机参数迁入总 YAML；
3. `[接口]` launch 与行为正式 profile 断开 Nav2；
4. `[安全]` ATS action/Goal Manager/MPC 重启、时序和旧授权回归；
5. `[仿真]` MuJoCo ATS action 入口与故障注入；
6. `[可视化]` ROGMap RViz profile；
7. `[文档]` 实际证据与边界；
8. `[清理]` 只有 P3 通过后再移除 Nav2 构建依赖/旧资源。
