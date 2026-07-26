# ATS P5 实车化整改提示词（MINCO 接线、场地模型、统一 telemetry）

本文件是 5.13 缺陷审查之后的下一阶段提示词，直接复制下面代码块作为新对话的第一条消息。它承接 `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md` 的 5.13 与第 6 节 P5，不重复 P1~P4 已完成范围。

```text
请继续在工作区 `/home/ats/ATS_2026_snetry_test` 开展 ATS 2026 四驱四转哨兵导航研发。

本轮目标：完成 5.13 列出的 P5 实车化整改，把已实现未运行的 MINCO 首端播种接入运行链，消除 MuJoCo 场地模型冲突，并建立统一 telemetry 以便后续 baseline 与灰度门禁可度量。

必须完整阅读并遵守：
1. `AGENTS.md` 与 `CLAUDE.md` 的全部约束（提交、分支、colcon、破坏性操作、用户文件归属）。
2. `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`，重点 5.9~5.13、第 6 节 P5、第 7 节边界、第 8 节回归入口、第 9 节维护约束。
3. `docs/p4_real_robot_calibration_preflight.md`。
4. `docs/p4_stage4_stable_tracking_prompt.md`（telemetry/baseline/门禁定义仍然有效）。
5. 本文件。
6. 五个独立仓库的 git 状态与用户已有改动。
7. 不得读取或修改 `参考/` 与 `minco+mpc_reference/` 下的任何内容，它们不是运行时依赖。

五个独立仓库（各自都在 `develop`，禁止新开分支）：
- 根仓库 `/home/ats/ATS_2026_snetry_test`（`docs/`、`scripts/`、`src/ats_sentry_bringup`、顶层规范）
- 导航仓库 `src/ats_sentry_nav`
- MuJoCo 仓库 `src/sim/ats_mujoco_sim`
- 行为树仓库 `src/ats_sentry_behavior`
- loopback 仓库 `src/sim/loopback_sim`

必须保留的用户改动（不得 stage、不得回滚、不得覆盖）：
- 根仓库 ` M .gitignore`
- 导航仓库 `?? sentry_chassis_vel_transform/`
- MuJoCo 仓库 `?? **/__pycache__/`
其余未知修改与未跟踪文件默认属于用户，发现重叠修改先理解并合并，无法安全处理再询问。

开始修改前必须输出：
1. Definition of Done；
2. 精确文件范围（逐个文件路径）；
3. 可执行验证清单（构建、单测、闭环命令与判据）；
4. 当前假设、未验证项、停止条件；
5. 五仓 baseline commit 与用户已有改动清单。
不得只给方案后停止。

总原则：
- 一切以"仿真通过但实车失控/损坏"为最高优先级判据；任何门禁不得通过放宽 unknown、frame、footprint、执行器物理限值或 stale 安全语义来通过。
- 区分 `已实现` / `已测试` / `已验证` / `未实现`，未验证假设标注 `[Confidence: High/Medium/Low]`。
- 用户可见输出使用中文，数学使用严格 LaTeX。
- 不得替换：Point-LIO 定位、`src/ats_sentry_nav/ats_rog_map`、RC-ESDF 语义、JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair、`ats_swerve_mpc`。
- 四舵轮控制量恒为车体系 `[vx, vy, wz]`，状态恒为世界系 `[x, y, yaw]`；禁止迁入差速、ICR 或 `vy=0` 约束。
- 不得从 `/rog_map/esdf` 可视化 `PointCloud2` 反解析数值距离场。
- `localization_fusion` 继续独占 `map -> odom`、`/localization` 与定位健康/epoch；`/rc_esdf/planning_grid` 始终只能有一个发布者。
- `ExecutionCommand` 继续是 MPC 的唯一执行授权，legacy `Path` 与 `emergency_stop=false` 不得重新授权。

零. 先完成 MINCO 首端播种的运行链接线（最高优先级，已实现未运行）
1. 在 `minco_planner_node` 的参数声明与读取两处补齐 `initial_state_max_speed`、`initial_state_max_acceleration`，默认值与 `MincoTrajectoryOptimizerParams` 一致，并加中文注释说明它们对应实车里程计噪声保护而非动力学上限。
2. 新增 `/localization` 订阅，把 twist 旋转到世界系后填入 `InitialKinematicState`；无有效里程计、数据 stale 或非有限值时 `valid=false`，保持原零初值行为，并输出中文 WARN。
3. 四处 `optimizer_.optimize(...)` 调用点全部传入初值；首次规划（车辆静止）与单测路径必须保持行为不变。
4. 新增周期重规划定时器，使 `onRuntimeSafetyRecheck` 判定不安全后可以触发重规划而不是只能停止；重规划仍必须经 Goal Manager 授权，MINCO 不得越权发布正式 reference 或急停。
5. `toPath` 保留 MINCO 解析得到的 `vx/vy/ax/ay`，让 MPC 使用解析前馈而不是数值差分；若消息类型无法承载，则改走结构化 reference 消息并在文档记录。
6. 新增单测：带初速播种时首端一阶/二阶导数等于裁剪后的输入；超限初速被裁剪到 `min(initial_state_max_*, max_*)`；非有限输入退化为零初值。
7. 把 `minco_planner` 剩余英文规划失败日志改为中文分级日志，覆盖：地图未就绪/过期、JPS 无路径、MINCO 求解失败、时间缩放次数耗尽、footprint gate 拒绝、运行期 unsafe、里程计缺失或跳变，全部使用 throttle 避免刷屏。

一. reference 动力学可行性
1. 用按段自适应 time scaling 替代当前全局均匀缩放，只拉长真正超限的段，并给出缩放前后峰值速度/加速度对照。
2. 评估把净空写成优化约束（安全走廊）的最小改动方案；若本轮不落地，必须在文档写明理由和保留的事后 gate 边界。
3. 每次规划记录：`raw_points`、`reference_points`、峰值速度/加速度、时间缩放迭代次数、最小 footprint 净空、求解耗时。

二. MuJoCo 场地模型冲突
1. 确定单一权威碰撞表示：hfield 与墙体 box 不得对同一实体重复建模；对 LiDAR 不可见的实体碰撞必须消除或显式在文档标注为已知偏差。
2. 按 `docs/RM2026场地.pdf` 校正高度与坡角：场地 `28 m × 15 m`、围挡 `2.4 m`、梯形高地 `200~400 mm`（`43°`/`23°`）、中央高地 `10.5°`、装配区 `12°/14°/15°/45°`、公路区 `11°/15°`、飞坡 `17°`、堡垒 `20°`；公差 `<100 mm` 为 `±5 mm`、`≥100 mm` 为 `±5%`、结构 `±3°`、道具 `±1°`。修正当前 `--z-scale 0.1` 造成的 10 倍高度压缩。
3. 统一 origin 为单一取值，消除 `8.027637` 与 `8.025` 并存的约 `2.6 mm` y 偏移。
4. 固定生成顺序，消除 `rmuc_corridor_patch.py` → `rmuc_hfield.py` → `rmuc_nav_map.py` 的环形依赖；墙体只能经 `rmuc_wall_collisions.py` 与 `rmuc_2026_swerve.xml` 的 `<include>` 再生。`clear_nav_free_space` 不得抹平坡道。
5. 给出修正前后覆盖度对照（occupied 栅格数、hfield`>0` 且 occupied 数、越界数），并说明既有闭环结论是在旧模型下取得。

三. 统一 telemetry 与真值
1. 每条路线导出一份 CSV：时间戳、reference 位姿/速度、定位位姿/速度、车体命令、cross-track/along-track/yaw 误差、reference age、tracker progress、MPC 求解耗时与迭代数、逐轮速度/舵角饱和标志、滑移、contact。
2. 给出 p50/p95/p99 统计，使 5.11.2 的净空预算
   `C_min(t) > e_track_99 + e_loc_99 + v(t)*tau_99 + d_brake(v, slope) + m_map`
   可以真正闭合；`tau_99` 必须来自实测端到端时延分解（传感器→定位→规划→授权→MPC→底盘）。
3. telemetry 只做旁路观测，不得进入控制回路，也不得改变任何安全判据。

四. 故障与恢复复核
1. 任一改动完成后复核五级归零：`ExecutionCommand STOP -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0 -> 四轮 0 rpm`。
2. 复核带初速播种后仍满足：急停后不得因残留初值继续生成非零参考；恢复只接受新 epoch/generation/序号的授权。
3. 复核周期重规划不会产生 preempt storm，也不会让旧 reference 复活。

五. 验证与准入
1. 构建：`MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1`。
2. 单测：`colcon test --base-paths src --packages-select <targets>`；`colcon test-result --test-result-base build/<package> --verbose`。
3. launch 语法：`python3 -m py_compile <changed_launch_files>`；空白检查 `git diff --check`。
4. 闭环（每例独立 `ROS_DOMAIN_ID` 与独立 MuJoCo 启动，关闭 viewer/RViz）：
   `PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh`
   以及 `TEST_PROFILE=rectangle GOAL_TIMEOUT=120` 与 `scripts/test_mujoco_swerve_dynamics.sh`。
5. 准入判据沿用 5.11.4：安全契约 `10/10`；终端 p95 位置 `<=0.08 m`、yaw `<=0.10 rad`、线速度 `<=0.05 m/s`、角速度 `<=0.10 rad/s`，停稳 dwell `>=0.30 s`；固定 revision/config/seed 后关键场景重复 `10/10`。阈值在候选优化前冻结，失败样本全部保留。

六. 提交与交付
1. 按内容拆分提交，使用详细中文标签（`[安全]`、`[仿真]`、`[规划]`、`[控制]`、`[文档]`、`[规范]`），只显式 stage 本轮列出的文件；禁止 `git add -A`、`git add .`。
2. 只在实际修改的仓库提交并普通 push 到 `origin/develop`；推送失败保留本地提交并报告远端错误，不做 force push。
3. 必须更新 `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`：滚动窗口、5.13 之后的新验证记录、第 6 节 P5 状态、第 7 节接续入口。
4. 最终报告必须列出：改动文件清单；构建/单测/闭环实测数字；每项结论的 `已实现`/`已测试`/`已验证`/`未实现` 标注与 Confidence；本轮不可声明项；下一阶段建议顺序。
```
