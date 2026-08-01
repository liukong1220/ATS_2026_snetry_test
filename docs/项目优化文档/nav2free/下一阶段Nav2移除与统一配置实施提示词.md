# 下一阶段 Nav2-free 移除与统一配置实施提示词

更新时间：2026-08-01。

本文件是下一次新对话的可执行 handoff。它描述实施边界和验收门禁，不把本轮静态审查当作运行证据。新对话必须重新读取当前工作树、`AGENTS.md`、活动源码和测试结果。

## 直接复制的下一轮提示词

```text
新线程。请从当前工作区重新开始证据优先的实施，不沿用旧会话的未验证结论。

工作区：/home/ats/ATS_2026_snetry_test

一、绝对范围

1. 当前目标是 `nav2free`：把 ATS 自研导航链做成正式运行架构，并逐阶段移除活动代码中的 Nav2 依赖。
2. `docs/项目优化文档/nav2移植/**` 是历史迁移资料，不属于本设计。禁止读取、搜索、引用、修改、链接或从该目录推断任何事实、状态和性能；所有证据只来自 `AGENTS.md`、当前活动源码、接口定义、实际 launch、测试和本目录文档。
3. 不要把“存在 Nav2 兼容源码”写成“目标架构使用 Nav2”，也不要把“编译通过、topic 存在、单次局部运动”写成闭环通过。
4. 用户要求由当前执行者完成定位、修改、构建、单测、仿真、文档、提交和推送；不要把任务转交给其他代理。

二、目标架构与不可替换契约

目标链固定为：

`Point-LIO/定位融合 -> ROGMap -> ROGMap ground adapter -> RC-ESDF -> JPS -> MINCO S3 + 独立 yaw -> footprint gate/Local Collision Repair -> ATS Goal Manager -> 全向 SE2 MPC -> 速度兼容层 -> 四舵轮底盘或 MuJoCo`

- Point-LIO 继续拥有 `/localization`、`/localization/status`、`/registered_scan`；ROGMap 不承担定位。
- ROGMap 活动实现只有 `src/ats_sentry_nav/ats_rog_map`，adapter 必须消费数值 projection service，禁止从 `/rog_map/esdf` 的 `PointCloud2` 反解析 signed distance。
- RC-ESDF 必须保留 signed distance 的符号、梯度方向、截断、unknown、outside-map 和 footprint 语义。
- JPS、MINCO S3、独立 yaw、footprint safety、Local Collision Repair、`ats_swerve_mpc` 均属于目标链，不得用差速、ICR 或 `vy=0` 约束替换四舵轮车体系 `[vx, vy, wz]`。
- 状态使用世界系 `[x, y, yaw]`，底盘控制使用车体系 `[vx, vy, wz]`；所有 producer/consumer 必须同时核对 frame、单位、时间戳、QoS、超时和 owner。
- 正式实机默认保持 `launch_fake_vel_transform:=True` 与 `launch_chassis_vel_transform:=True`。fake-yaw 关闭时仍发布 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF；不得增加第二个 `base_footprint -> base_link` publisher。
- `/rc_esdf/planning_grid`、`/cmd_vel_mpc`、`/motion_control`、急停和 TF 各自只能有一个权威 owner。安全状态必须是输入健康与规划安全的合取。
- `ROGMap source generation`、adapter publication、`MINCO local snapshot generation` 和 localization epoch 是不同版本域。除非同一不可变接口显式携带并由 consumer 校验，否则禁止声称端到端编号一致。

三、先输出再实施

开始修改前，在回复中明确：

- Definition of Done（DoD）；
- 精确文件范围，按根仓、导航仓、MuJoCo 仓分组；
- 可执行验证清单和预期判据；
- 当前假设、未验证项、风险和停止条件；
- 三个仓库当前 branch、HEAD、`origin/develop` 和未知用户修改。

任何与用户已有修改重叠的文件都必须先读取并合并理解；不得删除、覆盖或恢复未知修改。禁止 `git add .`、`git add -A`、`git reset --hard`、`git checkout --`。

四、精确证据范围

先用 `rg` 定位 symbol、topic、参数和调用点，再读取完整函数/类作用域及直接 producer、consumer、launch、manifest 和测试。默认忽略 `build/`、`install/`、`log/`、缓存、二进制、媒体、生成物和参考资料。

优先核对这些活动路径（按实际存在情况取证，不要盲目修改）：

- 根仓：`src/ats_sentry_bringup/launch/bringup.launch.py`、`src/ats_sentry_bringup/launch/real_robot_nav2_free.launch.py`、`src/ats_sentry_bringup/params/node_params.yaml`、`src/ats_sentry_bringup/rviz/*.rviz`、`scripts/test_mujoco_minco_mpc_chain.sh`、相关 README/docs；
- 导航仓：`ats_nav_bringup/launch`、`ats_rog_map`、`ats_rog_map_adapter`、`minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`、`ats_navigation_interfaces`、behavior、`trajectory_optimizer` 的直接接口和测试；
- MuJoCo 仓：`src/sim/ats_mujoco_sim/launch`、传感器桥、执行桥和直接测试；
- manifest/CMake/package.xml 中的 `nav2_common`、`nav2_msgs`、Nav2 plugin/server/lifecycle 依赖；
- 当前实际加载的 YAML、behavior tree、RViz profile 和测试脚本，而不是同名示例文件。

对每条重要结论建立账本：`claim | confirmed/inference/hypothesis/unknown | evidence A | evidence B | counterevidence | validation`。

五、分阶段实施顺序

P2：运行入口和 ROGMap 唯一规划地图

1. 明确 `planning_grid_owner` 仅接受 `rc_esdf|rog_map`；`rog_map` 启动 adapter 并抑制 `rc_esdf_map`，`rc_esdf` 执行反向选择；没有显式 arbiter 前禁止运行中热切换。
2. 核对并修复 ROGMap ground planning 参数：高度带必须排除有运行证据的地面回波，同时保留墙体、低矮障碍、terrain、slope、unknown 语义。
3. 验证 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf`、`/rog_map/bounds` 非空；adapter 不订阅 ESDF 点云。
4. 对 source occupied/free/unknown、全来源 unknown、范围外、非整除分辨率、平移 origin、地图 yaw 和 footprint overlap 补最窄单测；occupied 永不被 free 覆盖，unknown 默认按障碍处理。
5. projection 使用 steady-clock deadline；timeout 必须移除 pending client、清除 pending 状态并允许重试，迟到回调按单调 epoch 丢弃。

P3：Nav2-free 正式入口和统一总 YAML

1. 将正式实机 MINCO、Goal Manager、MPC、ROGMap adapter 及 owner/topic/frame/QoS/timeout 参数迁入 `src/ats_sentry_bringup/params/node_params.yaml`，使其成为正式实机唯一总 YAML。`.msg/.srv/.action` 仍是 schema 唯一权威，不把消息字段伪装成参数。
2. 逐个 launch 追踪实际参数加载路径，先增加重复 key/effective parameter 检查，再切换入口；不能先删除 `*_reality.yaml` 再补连接。
3. 正式自研入口必须 `launch_nav2:=false`，无 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`，MINCO 不订阅 `/plan`，目标入口不调用 `nav2_msgs/action/NavigateToPose`。
4. ATS action/goal manager 必须覆盖 feedback、result、cancel、preempt、timeout、server restart incarnation；无新目标时恢复不能复活旧 response、旧 generation、急停前 reference 或 command。
5. 保留单独且明确命名的 Nav2 baseline 仅用于对照，不能让 baseline 参数或 topic 泄漏到正式 profile；在 P3 扩大矩形和红框均通过前，不删除公共算法的兼容依赖。

P4：行为、RViz 与 MuJoCo 闭环

1. 正式行为树只通过 `/ats_navigate_to_pose` 进入自研 action；Nav2 plugin、测试和构建残余列入清理清单，逐项确认无 owner 后再删除。
2. RViz 复用原项目可视化方式，显示 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 和 bounds Marker；明确各 display 的 frame、颜色、点尺寸、alpha、QoS、开关。可视化只消费显示 topic，规划只消费数值 projection。
3. MuJoCo 自研入口从 ATS action 发目标，验证无 Nav2 process、无 `/plan`、`/cmd_vel_mpc` 和 `/motion_control` 唯一 owner；默认路线与红框分开运行。
4. projection timeout、Point-LIO stale、adapter heartbeat lease 超时、真实 unknown、unreachable 和 unsafe trajectory 分别使用新的 `ROS_DOMAIN_ID` 与新 launch 注入，观察 `ready=false -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0` 的 deadline 内闭环；恢复后 generation 继续递增且旧授权不复活。
5. 成功场景必须记录终点坐标/误差、JPS/MINCO/reference 点数、离散 footprint 冲突采样数、失败/恢复次数；没有独立 contact evaluator 时物理接触写为“未验证”。

P5：RC-ESDF 和公共依赖解耦

1. 只有在 P2-P4 门禁通过后，才清理 `trajectory_optimizer` 中不再属于目标链的 Nav2 继承、plugin、CMake/package 依赖和旧资源。
2. 每次删除前用 `rg` 证明无活动 producer/consumer；保留 `[Dead Code Suggestion]` 标记，不因整洁而删除未授权代码。
3. 清理后重新运行最窄构建、聚焦单测、launch `--show-args`、无 viewer MuJoCo、P3 扩大矩形和红框。

六、验证命令与门禁

静态与构建：

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile <changed_launch_files>
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1
colcon test --base-paths src --packages-select <targets>
colcon test-result --test-result-base build/<package> --verbose
ros2 launch <package> <launch> --show-args
git diff --check
```

运行图和参数：

- 使用新的 `ROS_DOMAIN_ID`、新的 daemon 状态和无 viewer/RViz MuJoCo；
- `ros2 node list --no-daemon` 确认正式 profile 没有 Nav2 server；
- `ros2 topic info -v` 核对 planning grid、`/cmd_vel_mpc`、`/motion_control`、急停和 TF 的唯一 owner、QoS、frame；
- `ros2 param dump` 核对 MINCO、Goal Manager、MPC、adapter 的 effective values 确实来自总 YAML；
- `rg` 检查正式入口、参数和行为树不再引用 `/plan`、Nav2 action 或 Nav2 server；
- 用接口定义和 producer/consumer 两侧核对 frame、时间、单位、generation、unknown、outside-map、QoS、timeout 和 fallback。

闭环场景：

```bash
TEST_PROFILE=default PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh

TEST_PROFILE=red_box PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

`scripts/test_mujoco_nav_chain.sh` 只能作为显式 Nav2 baseline 对照，不能作为 P3/Nav2-free 通过证据。故障注入必须分场景、分 domain 运行，不能串行污染同一机器人状态。不得用旧 revision 的 MuJoCo 结果覆盖最后一次地图、规划、安全或控制源码修改后的证据。

七、文档与交付

1. 只更新活动源码对应的 README 和 `docs/项目优化文档/nav2free/` 文档；不得读取或修改被排除的历史迁移目录。
2. 每条结论标注 `已验证-运行`、`已验证-单测`、`已实现-静态确认`、`已实现未运行`、`未实现` 或 `未验证`，并注明 `[Confidence: High/Medium/Low]` 和证据边界。
3. 根仓、导航仓、MuJoCo 仓分别显式 stage 本轮文件，禁止 `git add .`/`-A`；提交标题使用中文并按接口、配置、算法、安全、仿真、可视化、文档拆分。
4. 每个仓库提交前运行 `git diff --cached --stat`、`git diff --cached --check`，确认未混入用户文件、测试 log、编译残留、模型或缓存；推送 `develop -> origin/develop`，不 force push。
5. 最终报告按仓列出修改文件、命令及结果、默认/红框终点与误差、路径/reference/MPC/底盘 owner、generation/effective-param、stale/unknown/unreachable/timeout/heartbeat 归零与恢复、未运行项、残余风险、commit ID 与远端一致性。

停止条件：发现用户修改无法安全合并、接口/参数语义冲突未能由源码和测试判定、急停链不能在 deadline 内归零、或实际运行证据缺失而只能靠推断时，保留日志和工作树，停止扩大改动并报告阻塞点。

现在立即执行“定位 -> 最小修改 -> 构建 -> 单测 -> launch 检查 -> MuJoCo -> 文档 -> 分仓提交/推送”，不要只输出计划。
```

## 使用说明

1. 新会话第一条消息粘贴上面的代码块。
2. 新会话必须重新读取 `AGENTS.md`、三个仓库状态和活动源码；本文件中的事实只作定位线索，不能替代当前 revision 证据。
3. 若只做 review，明确写“只做 review，不修改代码”，并删除提示词中的实施、提交和推送授权。
4. 若只做参数统一，应明确把 MuJoCo、RViz 和 Nav2 依赖清理标为 out of scope，避免跨越多个安全边界。

## 推荐提交拆分

1. `[接口]` 固化参数 schema、重复 key 和 effective-load 检查；
2. `[配置]` 将正式实机参数迁入总 YAML；
3. `[接口]` 让 launch 与行为正式 profile 使用 ATS action 并断开 Nav2；
4. `[安全]` 固化 Goal Manager/MPC 重启、时序、快照和旧授权回归；
5. `[仿真]` 接入 MuJoCo ATS action 入口和分场景故障注入；
6. `[可视化]` 更新 ROGMap RViz profile；
7. `[文档]` 记录实际证据、边界和残余风险；
8. `[清理]` 仅在 P3/P4 通过后移除 Nav2 构建依赖与旧资源。
