# 下一阶段 P2/P3 安全故障注入与恢复实施提示词

更新时间：2026-08-02。

本文件是下一次新对话的可执行 handoff。下一阶段先关闭已经存在的运行期安全门禁；
不得把名义路线通过、静态源码检查或 topic 存在写成故障恢复、仓库级 Nav2-free 或
实车安全通过。

## 1. 已完成内容

1. 导航仓 `718ceb6`：`ExecutionCommand` 新增 `manager_incarnation`；Goal Manager
   以 `steady_clock` 为每次进程启动生成 token，启动发布 `MODE_STOP`；MPC 按
   `(manager_incarnation, command_sequence)` 拒绝旧实例，且新实例必须先 STOP 再
   EXECUTE。Goal Manager pytest 和 MPC gtest 已通过。
   `[已验证-单测, Confidence: High]`
2. 根仓 `70af093` 与 `df96ad7`：MuJoCo 自研脚本接受 `TEST_PROFILE=default`，并记录
   default/red_box 的名义运行。两次运行检查没有 Nav2 server、没有 `/plan`，确认
   `/rc_esdf/planning_grid`、`/planner/execution_command`、`/cmd_vel_mpc`、
   `/motion_control` 的指定 owner 唯一；终点误差分别为 `0.003553 m`、`0.003347 m`。
   `[已验证-运行, Confidence: High]`
3. 脚本已经实现 P2 `adapter_lease`、`service_timeout`、`input_stale`、`unknown`、
   `unreachable` 和 P3 `cancel`、`preempt`、`timeout`、`tf_failure`，但尚无这些用例
   的当前 revision 运行结果。`[已实现未运行, Confidence: High]`

未完成：上述 9 个故障门禁、Goal Manager/MPC/serial 的真实进程重启注入、serial
incarnation/digest 契约、独立 physical contact evaluator、P3 仓库级 Nav2-free 和实车/HIL。

## 2. 直接复制的下一轮提示词

```text
新线程。请从当前工作区重新开始证据优先的实施，不沿用旧会话的未验证结论。

工作区：/home/ats/ATS_2026_snetry_test

一、绝对范围

1. 本轮只完成 P2/P3 运行期安全故障注入与恢复闭环。优先运行已有脚本入口；任一
   用例失败时只修改拥有该失败状态转换的模块并补最窄回归。
2. 禁止删除 Nav2、重构总 YAML、迁移 RC-ESDF、修改 RViz 或扩展目标功能。这些工作
   必须等待本轮所有故障门禁通过。
3. `docs/项目优化文档/nav2移植/**` 是排除目录。禁止读取、搜索、引用、修改或从中
   推断事实；证据只来自 `AGENTS.md`、活动源码、接口、launch、测试、日志和本目录。
4. 不得把编译通过、topic 存在或一次名义运动写成故障闭环通过。用户要求当前执行者
   完成定位、修改、构建、单测、仿真、文档、分仓提交和推送，不转交给其他代理。

二、本轮已完成上下文（只作定位线索，不能代替当前 revision 验证）

- `ExecutionCommand.manager_incarnation` 已从 Goal Manager 传至 MPC；新实例先 STOP
  后 EXECUTE 的聚焦测试已通过。serial gate 和 content digest 尚未覆盖。
- `TEST_PROFILE=default` 和 `red_box` 名义 MuJoCo 路线曾通过；P2/P3 fault case 已实现
  但未运行。
- 目标链固定为 Point-LIO/定位 -> ROGMap -> adapter -> RC-ESDF -> JPS/MINCO ->
  Goal Manager -> 全向 SE2 MPC -> 速度兼容层 -> 四舵轮/MuJoCo。状态为世界系
  `[x, y, yaw]`，控制为车体系 `[vx, vy, wz]`，禁止引入 `vy=0`、差速或 ICR 约束。

三、开始前必须输出

- DoD、根仓/导航仓/MuJoCo 仓的精确文件范围、命令清单和通过判据；
- 三仓 branch、HEAD、origin/develop、未知用户修改；
- 假设、未验证项、风险和停止条件。

先用 rg 定位 P2_FAULT_CASE、P3_FAULT_CASE、run_p2_fault_injection、
run_p3_action_fault_injection、emergency_stop、ready、generation、ExecutionCommand 的
producer/consumer。读取完整函数、直接 launch 和测试；默认忽略 build/install/log/cache，
且绝不读取排除目录。

四、DoD

1. 9 个 fault case 均在各自新的 ROS_DOMAIN_ID 和各自新的无 viewer MuJoCo launch 中
   执行，不能在同一次 launch 中串行污染状态。
2. P2 每例必须在配置 deadline/lease 内观测：预期 ready/stale 状态 ->
   `/planner/emergency_stop=true` -> `/cmd_vel_mpc=0` -> `/motion_control=0`。对可恢复
   用例，恢复后 adapter generation 必须递增；没有新目标时不得复活旧 reference 或旧
   command。
3. P3 每例必须得到正确 action result（cancel/preempt/timeout/tf_failure）和同样的
   双零输出；停止后等待至少一个 watchdog 周期，确认不自行恢复运动。
4. 每例保存 launch log、action 输出、命令流和 telemetry。没有独立 physical contact
   evaluator 时只记录 telemetry 的 contact_violation_count，不声称物理碰撞为零。
5. 任一失败先定位 owner，添加最窄单测/脚本断言，重跑该例；所有通过后再更新文档、
   显式 stage、按仓提交和普通 push。

五、运行命令

每条命令的 N 必须不同，且在一条新 launch 中只启用一个故障。先运行默认名义路线，
确认基础图和 owner，再运行对应 fault case：

```bash
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=none \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh

TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=adapter_lease P3_FAULT_CASE=none \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=service_timeout P3_FAULT_CASE=none \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=input_stale P3_FAULT_CASE=none \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=unknown P3_FAULT_CASE=none \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=unreachable P3_FAULT_CASE=none \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh

TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=cancel \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=preempt \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=timeout \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none P3_FAULT_CASE=tf_failure \
  ROS_DOMAIN_ID=N GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

六、不可突破的契约

- ROGMap adapter 只能消费数值 projection service，不得反解析 `/rog_map/esdf`。
- `/rc_esdf/planning_grid`、`/cmd_vel_mpc`、`/motion_control`、急停和 TF 各有唯一 owner。
- unknown 默认按障碍处理；occupied 不得被 free/unknown 覆盖；不得以 ego 外接圆清除
  footprint 外 unknown。
- 只可在现有 `ExecutionCommand` 契约中验证 Goal Manager -> MPC；没有读取 serial
  producer/consumer 之前，不得把 `manager_incarnation` 宣称为端到端 serial 契约。
- 实机入口的 fake-yaw/chassis transform 默认保持启用；不得新增重复 TF publisher。

七、停止条件

出现任一情况即保留日志和工作树，停止扩大修改并报告：急停链未在 deadline/lease 内
双零、恢复时旧 reference/command 自动复活、存在第二个 owner、故障注入未实际触达目标
状态、或只能用静态推断替代运行证据。
```

## 3. 本轮交付规则

1. 修改前后运行三个仓库各自的 `git status --short --branch`；未知文件归用户，禁止
   `git add .`、`git add -A`、破坏性恢复或 force push。
2. 有源码修改时先运行最窄构建与单测，再运行受影响 launch 的 `--show-args`，最后才运行
   对应的单一 fault case。
3. 文档必须分别标注 `已验证-运行`、`已验证-单测`、`已实现未运行`、`未验证` 及
   `[Confidence: High/Medium/Low]`。不得把 `contact_violation_count=0` 写为物理接触
   已验证。
