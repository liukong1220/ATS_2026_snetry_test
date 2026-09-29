# MINCO 整条轨迹联合优化（GCOPTER / EGO-Planner 思路）实现提示词

你是 ATS 2026 四驱四转舵轮哨兵导航链的证据驱动代码修改者。工作目录 `/home/kong/ATS_2026_snetry_test`，
先读根目录 `AGENTS.md`、`docs/next_stage_prompt.md`、`docs/real_robot_stage_prompt.md`。根仓库、
`src/ats_sentry_nav`、`src/sim/ats_mujoco_sim` 是三个独立 Git 仓库，分别提交。

## 1. 问题与根因（已核实）

- 现象：RViz 中红色 MINCO 参考呈"长直线 + 小圆角"，不是连续大弧。
- 现链路：JPS → PathGeometryPreprocessor → 0.30 m 加密 → `smoothGuideWithEsdf` 弹性带 →
  `refineWaypointsWithEsdf` 逐点法向修正 → 再次 0.30 m 加密 → MINCO S3（全部点为硬插值约束）→
  `MincoTimeAllocator` 启发式时长 + 局部/整体时间缩放。
- 根因：几何与时间都不是在整条轨迹上优化出来的。每 0.30 m 一个硬约束点、每段约 0.2 s，
  S3 只能逐点穿过引导折线，每个折点被单独磨圆；时长是按段长/折角限速的启发式值。
- 已试过的"稀疏航点 + 分段二分插点"（`guide_sparse_*`，未提交）只减少了硬点，点位和时长
  仍固定：开阔处首条参考 k95 由约 2.6 降到 1.4~1.8，但窄处退化回密点并在密点两侧甩钩
  （kmax 5~6），红点目标多次在南侧走廊 (1.8~4.1, -6.9~-5.5) 贴墙 ABORTED。
  关闭稀疏化的对照基线还没重跑，"成功率下降全因稀疏化"尚无对照证据。
- 现有 `MincoS3`（`include/minco_planner/trajectory/minco_s3.hpp`）只有
  `solve / sample / valid / pieceCount / pieceDuration`，内部 `BandedSystem` 只有 `factorizeLu` 和
  `solve`：没有能量、没有 ∂/∂系数、没有伴随求解（Aᵀ），也就没有 ∂C/∂q、∂C/∂T。
  仓库内没有 L-BFGS。`RcTraversabilityEsdfProvider` 已有 `getDistance` 与 `getGradient`（双线性）。

## 2. 第 0 步：处理稀疏航点实验与基线

1. `git status` / `git diff` 核对三个仓库的未提交改动（optimizer、node、hpp、单测、两个 yaml 中的
   `guide_sparse_*`、node 里的 `dense_optimizer_` 兜底）。
2. 默认处理：仿真 yaml 置 `guide_sparse_spacing: 0.0`，代码保留但不作为本次方案的一部分；
   如需回滚，逐文件用 Edit 撤销，不使用 `git checkout --` / `reset --hard`。
3. 稀疏关闭状态下跑红点目标 (10.36, 1.49) ≥3 次，记录成功率、首条参考 kmax/k95/符号翻转/
   `curvature_tv`、卡住位置。这是后续所有对比的基线。

## 3. 设计

### 3.1 决策变量
- 内点 q ∈ R^{2×(N-1)}：在现有加密引导（`refineWaypointsWithEsdf` + 二次加密之后）上按弧长约
  每 `joint_waypoint_spacing`（默认 1.0 m）重采样；首尾点固定，不是变量。
- 时长 τ ∈ R^N，T_i = exp(τ_i)，保证 T_i > 0。τ 初值取 `MincoTimeAllocator::allocate` 的 log。
  τ 做区间保护（对应 T ∈ [min_segment_time, joint_max_piece_time]），防止 exp 溢出；若线搜索
  因 exp 增长卡住，可换 GCOPTER 的 C2 微分同胚映射，但必须附证据。
- 边界：头部 (p, v, a) 来自 `makeHeadState`（与现有一致，含反向速度分量投影），尾部 v = a = 0。

### 3.2 代价
J = ∫ ||p⁽³⁾(t)||² dt + ρ_T·ΣT_i + Σ_i Σ_j (T_i/K)·ω_j·[ρ_obs·P_obs + ρ_v·P_v + ρ_a·P_a + ρ_lat·P_lat]
- 每段 K 个采样（默认 16），ω_j 为梯形权重，t_ij = (j/K)·T_i；对 T 的梯度要同时包含权重项
  T_i/K 和采样时刻 j/K 通过 p', p'', p''' 的链式项。
- 罚函数用光滑三次铰链 max(0, x)³（GCOPTER 做法），x 为违例量。
- P_obs：矩形足迹采样点（长宽 = `footprint_length/width` + `footprint_safety_margin`，四角 + 各边
  中点，可配置加密）按 yaw 旋转到世界系，x = d_safe − ESDF(pt)，梯度用 `getGradient`。
  yaw 取 `footprint_orientation`（无则取中心参考）按归一化时间插值，视为常量不对 yaw 求导；
  最终 yaw 仍由节点 `planYaw` 决定，门禁以它为准。另加中心点净空项
  x = d_center − ESDF(p)，d_center 与 `jps_safe_distance` 对齐。
- P_v：||v||² − v_max²；P_a：||a||² − a_max²；
  P_lat：|v × a|/||v|| − a_lat_max（||v|| 小于阈值时跳过，避免除零）。
  上限取现有 `max_velocity / max_acceleration / max_lateral_acceleration`。

### 3.3 MINCO 梯度（主要工作量）
在 `minco_s3.hpp` 中补齐（参考 GCOPTER `MINCO_S3NU`，自己实现，保持现有代码风格与注释密度）：
- `BandedSystem::solveAdj`（Aᵀx = b，复用已有 LU 分解）。
- `getEnergy`、`getEnergyPartialGradByCoeffs`、`getEnergyPartialGradByTimes`。
- `propagateGrad(∂J/∂c, ∂J/∂T_partial) → (∂J/∂q, ∂J/∂T)`：伴随法，每次迭代只解一次 Aᵀ。
- 罚项先对系数与时刻求偏导（由 β(t) 基函数链式展开），再统一经 `propagateGrad` 反传。

### 3.4 求解器
- L-BFGS：自实现（two-loop recursion + Lewis-Overton 或 More-Thuente 弱 Wolfe 线搜索），放
  `include/minco_planner/trajectory/lbfgs.hpp`。若改为引入第三方代码，先确认许可证、保留版权头，
  并在报告中说明；不得新增网络依赖或未固定版本的依赖。
- 终止：梯度相对范数 < `joint_g_epsilon`、迭代上限 `joint_max_iterations`（默认 200）、
  墙钟预算 `joint_time_budget_ms`（默认 15 ms，按实测调整），任一触发即停。

### 3.5 集成与兜底（fail-closed，保证不比现在差）
- 新文件 `trajectory/minco_joint_optimizer.{hpp,cpp}`，由 `MincoTrajectoryOptimizer::optimize`
  在现有加密引导生成后调用；参数 `joint_optimization_enabled`（默认 false）及 `joint_*` 权重与终止项
  写入 optimizer params、node 的 declare/get_parameter、仿真 yaml（带中文注释）；实车
  `node_params.yaml` 默认 false。
- 联合优化结果仍需经过现有动力学检查/时间缩放与采样，再进入节点矩形足迹门禁（最终裁决）。
- 以下任一情况回退到现有加密引导 MINCO 结果：L-BFGS 返回错误/NaN、终止时罚项残差超阈值、
  动力学超限、门禁不通过。节点候选顺序：联合优化候选 → 现有加密候选 → 现有 JPS-MINCO 兜底 →
  局部修复；兜底与局部修复不走联合优化。
- trace/日志增加：是否启用、迭代数、终止原因、初末代价、各罚项残差、墙钟、是否回退及原因。

## 4. 测试（gtest，`test/test_minco_joint_optimizer.cpp`）
1. 梯度校验：能量与每个罚项分别对 q、τ 做中心差分，相对误差 < 1e-5。
2. `solveAdj` 与稠密 Aᵀ 求解一致。
3. 直线路径：结果横向偏差 < 1e-6，端点精确。
4. L 形路径（沿用 `makeLPath`、`maxArcCurvature`）：kmax、k95 低于现有加密引导结果，
   端点误差 < 1e-8，满足速度/加速度/横向加速度上限。
5. 拐角内侧障碍：足迹采样点 ESDF 最小净空 ≥ 目标值 − 容差。
6. 不可行约束（例如走廊比足迹窄）：返回回退标记，输出与关闭联合优化时一致。
7. 26 m 规模路径的墙钟 < 预算。
现有 `test_minco_trajectory_optimizer` 必须全部通过。

## 5. 仿真验收（ROS_DOMAIN_ID=87，对齐后的 `rmuc_2025.pcd`）
- 每次运行前检查并清理残留 ROS/仿真/bag/rviz 进程。
- 单目标 (10.36, 1.49) 连续 ≥5 次：成功率不低于第 0 步基线；首条参考 k95 ≤ 1.5、kmax ≤ 3.0、
  符号翻转不高于基线、`curvature_tv` 低于基线；规划墙钟 p95 在预算内；回退次数与原因逐条列出。
- 南侧走廊 (1.8~4.1, -6.9~-5.5) 是已知贴墙卡死区，联合优化不得使其更差。
- 提供 RViz 3D（Orbit）截图，证明参考为连续大弧。

## 6. 强制规则
- 修改前先给出 DoD、文件范围、验证命令与风险；安全门禁、急停、速度上限相关改动先征得确认。
- 不得为了到达率放宽足迹门禁、关闭急停或调高 `ego_contact_max_depth`。
- 构建：`colcon build --base-paths src --packages-select minco_planner ats_mujoco_sim ats_sentry_bringup`；
  不得用 `-UFETCHCONTENT_SOURCE_DIR_QDLDL` 重建 `ats_swerve_mpc`（qdldl 源目录缺失是已知历史问题）。
- 验证：gtest、`python3 scripts/validate_navigation_config.py`、`git diff --check`。
- Git：只暂存明确的文件；不用 `reset --hard`、`checkout --`、force push；作者只用仓库已配置身份，
  不加任何 Co-Authored-By；推送 `develop` 分支。
- 结束时清理进程与 /tmp 临时文件。

## 7. 报告
区分"仿真已验证""已实现未运行""推断"；给出基线与联合优化的逐次成功/失败、曲率指标、墙钟、
回退统计、bag 路径和截图；列出三个仓库的 commit/push 结果与未覆盖范围。
