# ATS 下一阶段研发提示词：南侧走廊收尾验收 → 实车

你负责修改 ATS 2026 四驱四转舵轮哨兵的导航链，每项改动都要有证据支撑。工作目录是仓库根目录
（本机为 `/home/ats/ATS_2026_snetry_test`）。开始前先读根目录 `AGENTS.md` 和本文件。
以下都是独立 Git 仓库，要分别提交：根仓库、`src/ats_sentry_nav`、`src/sim/ats_mujoco_sim`、
`src/standard_robot_pp_ros2`、`src/sim/gazebo_simulator`、`src/sim/loopback_sim`、`src/sp_vision25`。
`参考/` 目录不读也不改。本阶段分 A、B 两段，A 段没有通过就不能进入 B 段。

## 当前状态（2026-10-01，红点目标 map (10.36, 1.49)）

### 已提交

- ats_sentry_nav c41d922：minco_planner 新增 5 个参数，代码默认值都等于关闭或保持原行为：
  - `yaw_tangent_symmetry_order`：正方形车取 4，窄通道切线的 ±π/2 也视为对齐。
  - `yaw_narrow_gap_bridge_time`：两段窄通道之间的短开阔间隙按窄通道处理。
  - `yaw_acceleration_limit`：参考 yaw 用可刹停的二阶跟踪生成，角加速度不超过上限。
  - `wheel_speed_time_scaling_*`：四轮轮速超限的位置局部放慢。
  - `narrow_turn_speed_limit`：窄通道内一边平移一边转向的位置限速。
  - 后两项只改时间参数化，位置和 yaw 序列不变。
- ats_mujoco_sim b2ea802：仿真 profile 打开上述参数：
  - 对称阶数 4，间隙桥接 1.0 s。
  - 参考 yaw 角速度 2.0、角加速度 3.0，与 MPC 的 max_wz、max_awz 一致。
  - 轮速上限 1.45，窄通道转向限速 0.8。
- 实车 profile `src/ats_sentry_bringup/params/node_params.yaml` 还没有同步任何一项。

### 仿真已验证（domain 71–77、84–86，测试工具在 `~/ats_stageA/`）

- **基线**（关闭上述全部开关）：3/3 冷启动都没有下发任何参考，车停在起点。
  - 每次都有 16 条轨迹被门禁拒绝，首个冲突在 (4.11,-5.44) 墙尖上方。
  - 因为没有参考，基线的曲率指标无法用 `scripts/analyze_reference_curvature.py` 计算。
- **新方案**：6 次有效运行中 3 次到达，终点误差 0.009–0.059 m。
  - 到达的 d71、d72 物理接触为 0，d73 有 496 次接触。
  - 首条参考 k95 1.24–1.37、kmax 2.43–2.72、曲率符号翻转 2–4、curvature_tv 11.6–12.2。
  - 参考 |wz| ≤ 2.0、|awz| ≤ 3.0、轮速 ≤ 1.45，都在 MPC 能力内。
  - solver_wall 中位数 27–79 ms，最大 109 ms；joint_wall 中位数 19–51 ms，最大 69 ms。
  - joint 优化以 time_budget 退出的比例约 1/5–2/3。
- **新方案失败的 3 次**都在南侧走廊被门禁拦停，最终未到达：
  - d74、d77 停在 (6.0~6.2, -7.4)，yaw 约 0。
    - 车体西北角压进 (5.85, -7.19) 处的占据格，起点位姿本身就冲突。
    - escape 前缀的长度和扫掠角都在上界内（0.22 m / 0.37 rad），但没有通过 0.05 m 深度检查。
    - d77 的 MuJoCo 物理接触为 0。
  - d76 停在 (2.30, -6.49)，yaw -1.45，31 次接触。
- **试过但已撤回**：窄通道整段保持恒定 yaw（d64、d65）。在墙尖处更差，d65 有 9677 次接触，代码未保留。
- **nav_tracking_recorder**：每次都报实际足迹与栅格冲突（含 safety_margin 0.02）。
  冲突集中在南侧墙尖 (3.4~3.6, -6.2) 和走廊 (6.0~6.9, -7.4)，所以 NAV_TRACKING_GATE 都为 1。

### 推断（未证实）

走廊 (6.0~6.2, -7.4) 的拦停很可能是规划栅格比 MuJoCo 实体更厚，原因可能是 0.1 m 量化或
静态图聚合：物理接触为 0，栅格冲突深度却超过 5 cm。参考通过门禁时只剩 1–3 cm 余量，
MPC 的 2–6 cm 横向误差就足以让起点位姿进入冲突。

### 环境注意

- 本机 Wi-Fi 网卡 wlo1 常掉线，而 `~/.ros/cyclonedds.xml` 绑定了 wlo1，
  仿真时用 `~/ats_stageA/cyclonedds_lo.xml`（只走 lo）。
- ROS_DOMAIN_ID 用 101 以下，120 曾创建 DDS domain 失败。
- 重启会清空 /tmp，证据目录放在 `~/ats_stageA/`。

## A 段：南侧走廊收尾（每完成一步就提交并 push）

1. **量化走廊 (5.8~6.9, -7.6~-7.0) 的栅格与 MuJoCo 实体偏差。**
   - 用 MuJoCo 模型的 geom 与 `/rc_esdf/planning_grid`（包括静态图聚合层）逐格对比，
     给出墙面实际位置与占据格边界的差值。
   - 偏差来自地图一侧时，修地图生成或聚合，不能放宽门禁或 safety_margin。
2. **加大规划侧余量。** 让参考在该走廊和南侧墙尖离墙更远，例如：
   - 评估 `joint_footprint_clearance` 0.08 → 更大值、`joint_footprint_edge_samples`，
     以及窄通道内对横向误差敏感的速度。
   - 目标：nav_tracking_recorder 的实际足迹冲突 tick 降到 0。
3. **定因 d76 的 (2.30, -6.49) 拦停**：墙尖西侧、yaw -1.45 时的接触。
4. **重新验收。** 每次冷启动，`use_rviz:=true`、`DISPLAY=:1`。
   - 基线 ≥3 次、新方案 ≥5 次（基线只临时改 install 副本，结束后重建 ats_mujoco_sim 恢复）。
   - 每次结束后用 pgrep 确认 leftover=0。
   - 通过标准：
     - 新方案到达率 5/5，物理接触 0。
     - 首条与全部参考 k95 ≤ 1.5、kmax ≤ 3.0。
     - 曲率符号翻转、curvature_tv 不劣于基线；基线无参考时注明"基线无下发"。
     - 南侧走廊不变差。
   - 记录到达率、终点误差、接触次数、solver_wall/joint_wall、time_budget 退出比例，
     并附 RViz 截图（`~/ats_stageA/<run>/ats_minco_mpc_red_box_<dom>_rviz.png`）。
5. **评估实车 profile 同步。** 逐项说明本阶段新参数，以及 joint_*、`footprint_yaw_refinement_rounds`
   是否迁入 node_params.yaml。
   - 实车 margin 是 0.05，要重新核对净空目标。
   - 轮速偏置要按实车 wheel_base 核对。
   - 每迁一项都要有仿真证据，默认 fail-closed。
   - 以下仿真专用开关在实车保持关闭或更保守：escape_from_contact、local_repair、
     retain_safe_reference、progress_along_reference、goal_pose_admission、
     endpoint_clearance_relaxation、ego_contact_max_depth、terminal_yaw_relocation。
6. **回归冒烟。** Gazebo 链与 loopback 链各跑一次。
   - 确认 TF 树每个 frame 只有一个 owner，执行端只消费 `/cmd_vel/selected`。

## B 段：实车（按顺序，前一步没有通过就不能进入下一步）

1. **静态检查**：
   - Mid360 外参与 `front_mid360` 静态 TF。
   - 串口 `serial/link_up`、`serial/gimbal_joint_state`。
   - 先验 PCD 路径。
   - `base_link`/`odom`/`map` 每个 frame 只有一个 owner。
   - 串口桥 `cmd_vel_topic` 为 `/cmd_vel/selected`。
   - 用 `view_frames`、`ros2 topic hz` 记录频率与延迟。
2. **定位**：静止和推行两种工况验证 Point-LIO + small_gicp 重定位，记录跳变次数、耗时、与标志点的偏差。
3. **地图**：比对实车 ROG-Map 与静态图在墙、立柱、坡沿处的过报和漏报，重点看南侧墙尖和走廊，量化门禁误拒率。
4. **开环规划**：架空底盘或断电，只发目标不执行。
   - 检查参考轨迹、门禁、yaw 补解轮数。
   - 检查实车 CPU 上的单次规划耗时和 time_budget 退出比例。
5. **低速闭环**：
   - `max_velocity` 先限 0.5 m/s，改速度上限前先征得确认；遥控急停随时可用。
   - 逐档放开到 1.0、1.5 m/s。
   - 每档记录横向误差、yaw 误差、MPC 求解耗时、cmd_vel 与实测轮速。
6. **单目标验收**：单目标 (10.36, 1.49) 实车连续成功 ≥5 次之后，再讨论提速或多目标。

## 强制工作方式

- **改动前**：先给出 DoD、文件范围、验证命令与风险。涉及安全门禁、急停、速度上限的改动先征得确认。
- **安全底线**：不能为了到达率放宽足迹门禁、关闭急停、调高 `ego_contact_max_depth` 或
  escape 深度上界。map unready/stale、定位跳变、串口断链、unsafe trajectory、MPC failure
  都必须确定性零速。
- **构建**：`colcon build --base-paths src --packages-select <包>`；不能用
  `-UFETCHCONTENT_SOURCE_DIR_QDLDL` 重建 `ats_swerve_mpc`。
  - 测试脚本会检查运行产物是否比源码新，改源码后必须重建再跑。
- **验证**：
  - 在 `build/minco_planner` 下跑 ctest，排除 lint/clang_format 等历史失败项。
  - 跑 `python3 scripts/validate_navigation_config.py` 和 `git diff --check`。
  - 不能写成"全量测试通过"。
- **测试运行**：
  - 每次使用隔离的 `ROS_DOMAIN_ID`。
  - 保存 rosbag 与完整日志。bag 必须在全部 topic 所有权检查通过后才开始录，否则所有权检查会失败。
  - 测试后杀净进程。
- **Git**：
  - 每完成一阶段提交并 push。
  - 只暂存明确的文件，不用 `git add -A/.`、`reset --hard`、`checkout --`、force push。
  - 作者只用 liukong1220 <1625038134@qq.com>，不加 Co-Authored-By。
  - 推送到 `develop`。
  - 提交信息用中文详写，带 [接口]/[安全]/[仿真]/[文档] 标签。
- **收尾**：清理进程与 /tmp 临时文件。

## 报告要求

- 用中文，分"实车已验证""仿真已验证""已实现未运行""推断"四类。
- 给出每次运行的日期、domain、速度上限、成功或失败及原因、bag 路径、指标表。
- 列出各仓库 commit 与 push 结果，以及未覆盖的范围。
