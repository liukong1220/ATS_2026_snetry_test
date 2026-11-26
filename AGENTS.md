# ATS 2026 哨兵导航研发规范

## 1. 角色与目标

你是本项目的精确代码修改者与闭环验证者。改动优先服务于 ATS 四驱四转舵轮哨兵的实际 ROS 2 导航链：先定位行为所有者，再做最小修改，并以源码、静态校验和运行证据共同证明结果。

每轮行为修改前先给出：

1. Definition of Done（DoD）；
2. 精确文件范围；
3. 可执行的验证清单；
4. 当前假设、未验证项和风险转入条件。

本规范使用“建议、优先、默认”描述工作方式。现场证据与本文不一致时，先记录差异、影响和验证方案，再选择更合适的路径。“风险转入条件”仅表示当前高风险运行安全退出；会话继续进行只读审查、证据整理和修复复盘。

实现、修复或优化类请求的默认交付链为“定位 -> 修改 -> Release 构建 -> 配置与语法校验 -> 仿真闭环 -> 文档 -> 分仓提交与推送”。

### 1.1 当前状态与文档维护

当前模块进度与后续优化优先级见 [主报告第 5 节](docs/调研/无先验地图钻洞高速导航优化调研报告.md#51-执行顺序与当前停点)，2027 验收口径与最新实测见 [2027 导航验证方案](docs/nav_acceptance/20261116_2027_final_test_plan.md)。

- 仿真默认使用有图模式：`rmuc_2027` 地图，`slam:=false`，加载 `rmuc_2027` PGM/PCD。测试无先验时显式传 `slam:=true`（S3 runner 用 `--mapless`），PGM 和 PCD 输入为空。
- **已完成（Waves 0–6B-1，2026-11-26）**：唯一 `OSQP v1.0.0 SQP-RTI` 求解器（移除 `ilqr`/`qp_shadow`，含旋转系加速度盒可行投影与延迟增广/rollout 前瞻收缩）、统一混合搜索（移除 `astar_fallback`）与 `bicubic` 距离场、O2 真局部滚动地平线裁剪/拼接（B1 MINCO `p95` 从 `184.43 ms` 降至 `6.48 ms`）、`TUNNEL_FREE`/自适应连续扫掠/左右独立侧壁 PCA/隧道状态机与起点侧移掉头、Point-LIO 批量更新与 `point_filter_num=2` 窄洞特征保留、Schur SE(2) 条件数/单位内点双判据 O5b 退化门控 + 跟踪期死区/LPF/运动一致性滤波 + 分段重投影防卡顿、`unix_global_esdf` 风格 5 场零厚度 ESDF 与 RViz 局部规划默认显示、导航域 11 份零引用旧配置清理、独立 MuJoCo `mujoco_geom_normal_v1` 物理接触评估器；**Wave 6B-1（2026-11-26）**完成全链统一 `0.05 m` 分辨率与 `node_params.yaml` 单一参数源收敛、实时 TF 雷达观测方向（`resolveLidarFovYawOffsetRad()`）接入、`ats_rog_map` 按需 3D ESDF（无订阅时 `map_update_ms` 由 `142 ms` 降至 `3.3–7.5 ms`）与增量平面缓存、死代码清理以及全部 17 个超长第一方非驱动文件按功能拆分至 `<= 1000` 行（`453` 文件 `0` 超长），静态门全过，Stage A `B1`（`0.0153 m`，`contact=0`）、Stage B 障碍延迟（`p95=201.9–208.9 ms < 250 ms`）、`--reload`（`0.0412 m`，`contact=0`）与 `no_path` 通过。
- **尚未通过 / 未验证**：**Wave 6B-2 统一 `0.05 m` 后双阶段闭环收口**（Stage A 5 cm `B2 detour` 零接触与 `T3` 南侧底板拼接缝 `18` tick 归零并复验 11/11、Stage B `B1_mapless` 最终二进制稳定通过与 `dynamic_save` 前等待鲁棒性）、**`5 m/s` 高速段与高速换轨**（当前 V1/B1 真值峰值 `3.05–3.40 m/s`）、**开窗 RViz/MuJoCo GUI 核验**（当前桌面锁屏未开窗）、**P3 行为树全链集成**与**实车硬件联调**。

纯文档整理与代码审查只运行引用、链接、配置校验和 Git 差异等必要检查，不自动启动新的算法修改、构建或仿真；行为修改仍执行本规范的完整验证链。主报告只保留现行架构、契约、完成状态、剩余优化门与最新提示词，不追加重复流水账。

本阶段不修改系统文件或全局 ROS/DDS/SSH 配置，不处理与项目优化无关的内容，不修改 `src/dependencies/*`、`src/ats_sentry_nav/third_party/{osqp,qdldl}` 与物理场景 XML。**本项目已清理内部测试目录与 `scripts/test_*`，严禁新建、恢复或运行 `test/`、`gtest`、`pytest`、`colcon test` 或 `scripts/test_*`。**

## 2. 项目事实与架构边界

### 2.1 活动导航链

```text
传感器 + Point-LIO（批量更新 + IMU 饱和降级 + SO(3) 前向传播）
-> small_gicp_relocalization（Schur SE(2) 方向信息 + LOST-only Scan Context）-> localization_fusion
-> ROGMap 概率占据/膨胀/3D ESDF + 地面相对列与 TUNNEL_FREE + 进程内 adapter
-> /rog_map_adapter/planning_snapshot（唯一规划地图接口）
-> ATS Goal Manager (/ats_navigate_to_pose)
-> 统一混合 JPS/A* 搜索 -> MINCO S3 (bicubic) + 独立 yaw + 自适应连续扫掠足迹门 + Local Collision Repair
-> 全向 SE(2) 唯一 OSQP v1.0.0 SQP-RTI MPC（延迟增广 + 沿迹横迹 + rollout 安全门）
-> cmd_vel_arbiter -> 四舵轮底盘
```

- P3（Nav2-free）完成前仍须补齐行为树全链集成与最终套件通过；未在目标机实测前不引用参考项目性能作为 ATS 实测性能。

### 2.2 稳定模块边界

- Point-LIO 提供 `/odometry_identified`、`/localization` 与 `/registered_scan`；ROGMap 不是定位器。
- ROGMap 活动实现位于 `src/ats_sentry_nav/ats_rog_map`，进程内集成 adapter；原独立 `ats_rog_map_adapter` 与 `/rc_esdf/*` 已移除。
- MPC 唯一求解器为 `OSQP v1.0.0 SQP-RTI`（`solver_mode: qp`、`schema_version=5`），不恢复 `ilqr` 或 `qp_shadow`；图搜索不恢复 `astar_fallback`。
- 四舵轮控制为车体系 `[vx, vy, wz]`，状态为世界系 `[x, y, yaw]`；不迁入差速/ICR/`vy=0` 约束。
- 不从 `/rog_map/esdf` 可视化 `PointCloud2` 反解析数值距离场。
- Point-LIO 当前无回环检测或位姿图，建图漂移校正采用增长地图匹配，不引入回环后端（除非用户另行决定）。
- ZLIO（z-localization）为 GPL-2.0，只借鉴思路，不拷贝代码，不整套接入。

### 2.3 雷达安装与速度链

统一通过 `lidar_mount_mode`（`chassis` / `gimbal`）管理雷达安装与速度链契约，MCU 采用方案 A（`SendRobotCmdData.data.speed_vector.{vx, vy, wz}` 直接消费底盘系速度），由 `scripts/validate_navigation_config.py` 锁定：

- `ats_sentry_bringup/launch/real_robot_navigation.launch.py` 默认 `lidar_mount_mode:="gimbal"`、`gimbal_use_base_footprint_odom:="True"`、`launch_fake_vel_transform:="False"`、`launch_chassis_vel_transform` 随模式自动联动（`gimbal` 默认 `True`、`chassis` 默认 `False`）。双 yaw 云台大 yaw 上装雷达时，默认启用 `base_footprint` 里程计解耦（`active_nav_frame:=base_footprint`、`sensor_scan_generation` 仅发 `odom -> base_footprint` 且关闭 `publish_robot_base_tf`、`ats_rog_map` `base_ground_offset:=0.0`、`loam_interface` `base_frame_height:=0.15`、`point_lio` 同步雷达安装倾角重力向量、`sentry_chassis_vel_transform` 保持启用且 `transform_linear_with_big_yaw:=False`）；显式设 `gimbal_use_base_footprint_odom:=False` 回退至 `gimbal_yaw_odom` + `transform_linear_with_big_yaw:=True`（含大 yaw 前馈延迟补偿）。
- `ats_sentry_bringup/launch/bringup.launch.py` 默认 `lidar_mount_mode:="chassis"`、`launch_fake_vel_transform:="False"`、`launch_chassis_vel_transform:="False"`。
- `fake_vel_transform` 关闭时保留 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF；不引入重复的 `base_footprint -> base_link` 或双父 `odom -> gimbal_yaw_odom` TF 发布者。
- 全链参数统一收敛至 `src/ats_sentry_bringup/params/node_params.yaml`；小陀螺平移启用 `allow_body_yaw_follow:=true`、`enforce_gimbal_lock:=false`，MPC 与速度变换均含旋转系速度搬运 $R(-\omega_z \Delta t)$。
- 三套 profile 统一为实车长宽 `0.581 × 0.571 m`、余量 `0.02 m`、ROG/terrain 车高门 `0.249 m`、`jps_safe_distance=0.36`、终端位置容差 `0.05 m`、隧道 yaw/自旋启用；暂定平移上限 `5 m/s`、yaw `3 rad/s`、角加速度 `2 rad/s²`，MINCO 起点保护与 MuJoCo 轮速范围同步。实车硬件联调与双 yaw 云台现场标定仍未执行，配置与仿真通过不代表实车验证通过。

## 3. 多仓库与文件所有权

三个主要独立 Git 仓库分别检查、提交和推送：

| 仓库 | 路径 | 主要所有权 |
| --- | --- | --- |
| 根仓库 | 实际总工作区（由 `pwd`、`git rev-parse --show-toplevel` 核验） | `docs/`、`scripts/`、`src/ats_sentry_bringup`、顶层规范 |
| 导航仓库 | `src/ats_sentry_nav` | ROGMap、RC-ESDF、MINCO、Goal Manager、OSQP MPC、Point-LIO、GICP、导航 launch |
| MuJoCo 仓库 | `src/sim/ats_mujoco_sim` | 仿真模型、传感器桥、CPU/GPU LiDAR、MuJoCo launch、RViz 配置 |

修改前后均运行各仓 `git status --short --branch`。未知修改和未跟踪文件默认属于用户（如用户保留的 `README.md`、`scripts/install_build_dependencies.sh`、`src/sim/ats_mujoco_sim/models/.ats_mujoco_no_walls_r9bi8608.xml`）：

- 避免 `git add -A`、`git add .`；使用显式文件清单 stage；
- 不删除、不覆盖、不提交无关用户文件及 `log/`；
- 避免 `git reset --hard`、`git checkout --` 等破坏性恢复；发现重叠修改先理解合并。

## 4. 精确导航协议与接口账本

### 4.1 定位方式

1. 用 `rg`/grep 搜索目标 symbol、topic、参数、错误或调用点；
2. 读取目标函数/类的完整作用域；
3. 追踪直接 producer、consumer、launch 与参数配置；
4. 默认忽略 `build/`、`install/`、`log/`、缓存、二进制与 `参考/`。

`参考/` 仅用于算法与许可证溯源，不是活动构建输入。构建统一使用根目录 `./build.sh` 或带 `--base-paths src` 的 Release `colcon build`（带 `-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`）。

### 4.2 闭环接口账本

跨模块修改必须核对：

| 契约 | 必查内容 |
| --- | --- |
| frame | producer frame、TF 查询方向、consumer target frame |
| time | 观测时间、ROS/sim/steady clock、timeout、stale 行为 |
| map | resolution、origin/yaw、width/height、unknown、occupied、outside-map；细静态图降采样保留所有面积重叠 occupied 单元 |
| ESDF | signed distance 正负号、unknown、梯度方向、截断、`bicubic` 同源插值 |
| generation | 单次规划内 JPS/MINCO/footprint/repair 绑定同一不可变本地快照；跨进程按 `publication_sequence`/`source_generation`/`localization_epoch` 校验 |
| QoS | reliability、durability、depth、late joiner 行为 |
| ownership | `/rog_map_adapter/planning_snapshot`、TF、`/cmd_vel/autonomy_raw`、`/cmd_vel/selected`、`/motion_control` 与急停均有唯一权威 |
| fallback | map unready/stale、全 unknown、无路、unsafe 轨迹、MPC 求解失败或硬碰撞门禁均输出确定性零速；恢复后旧执行授权不复活 |
| map（建图模式） | 全局静态层 `40 × 30 m`、`0.1 m`，原点按 profile 偏置；`12 × 12 m` 仅为计算窗口；出界观测计数记录 |

## 5. 修改工作流

### 5.1 修改前

1. 定义 DoD、精确文件范围、可执行验证清单与风险转入条件；
2. 记录当前分支、HEAD 与回滚点。

### 5.2 实施

- 修改范围聚焦拥有目标行为的模块，不做无关重构；
- 物理 occupancy、概率证据、ROG inflation、JPS clearance 与 footprint margin 分层，避免重复膨胀；
- 回调并发共享地图/轨迹时使用不可变 snapshot 或明确同步；
- `/rog_map_adapter/ready=true` 是持续续租心跳，租约超时确定性触发急停零速；
- MPC 收到急停后清空 tracker；Goal Manager 在急停上升沿强制 `stop_to_reference_guard_sec`（默认 `0.01 s`）提交屏障，MPC 旧 reference/epoch 拒绝门不放宽；
- dead code 只标记 `[Dead Code Suggestion]`，未经用户授权不删除。

### 5.3 修改后

1. 运行最窄 Release 构建（`./build.sh --packages-select <targets>`）；
2. 运行 `python3 scripts/validate_navigation_config.py`、`python3 -m py_compile`、`ros2 launch ... --show-args` 与各仓 `git diff --check`；
3. 在隔离 `ROS_DOMAIN_ID` 下运行与改动直接相关的 MuJoCo 闭环（默认无窗口 `USE_RVIZ=false USE_MUJOCO_VIEWER=false`）；
4. 最后一次影响地图、定位、规划、安全或控制行为的源码修改后重跑对应闭环，不沿用旧二进制结果；
5. 按实际运行证据更新主报告与 `docs/nav_acceptance/20261116_2027_final_test_plan.md`。

## 6. 验收门禁摘要

详细坐标、命令与逐项状态以 [2027 导航验证方案](docs/nav_acceptance/20261116_2027_final_test_plan.md) 为准，核心硬门如下：

- **到达精度**：中间点独立真值 $\le 0.15\ \mathrm{m}$，终点与重载首目标 $\le 0.08\ \mathrm{m}$；
- **参考与跟踪安全**：实际执行参考离散矩形足迹冲突 `0`；`q1` 必过；`q2` 持续 $\ge 3$ tick 且跨 $\ge 2$ 快照判失败；
- **唯一所有权与零 Nav2**：`/rog_map_adapter/planning_snapshot`、`autonomy_raw`、`selected`、`/motion_control` 发布者均为 `1`，无 `/rc_esdf/*`、无 Nav2 节点与 `/plan`；
- **故障隔离**：`unknown`、`no_path`、`unsafe`、`adapter_lease`（OS 级 `SIGSTOP`/`SIGCONT`）、`point_lio_restart` 每项使用独立新 `ROS_DOMAIN_ID` 与独立 launch，验证急停、三路持续零速与恢复后旧参考不复活；
- **无先验 S3 门**：障碍可见延迟 `p95 < 250 ms`（同时报告最大值与射线清除恢复）、静态持久化与动态不保存、保存地图漏检/误检率、重载首目标 $\le 0.08\ \mathrm{m}$；
- **不可替代项**：物理接触在无独立评估器前记为 `unverified`；无窗口运行记为“未带 RViz/MuJoCo 窗口”；`5 m/s` 未触发时单列“未触发”；实车硬件未联调记为“未验证”。

## 7. RViz 与 MuJoCo 窗口约定

- 默认无窗口运行（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`），由代理核验日志、话题与录制器，并在报告中注明“未带 RViz/MuJoCo 窗口”。
- 需要现场观察时（桌面未锁屏且 `DISPLAY` 可用），显式启动 `ros2 launch ats_mujoco_sim rmuc_2027_mujoco.launch.py localization_source:=lio use_identified_odometry:=true use_viewer:=true show_viewer:=true use_rviz:=true`（无先验追加 `slam:=true`）。`mujoco_navigation.rviz` 已含车模、定位/真值箭头、`/goal_pose` 箭头、当前帧点云与局部规划窗口组。
- `ROS_DOMAIN_ID` 与 DDS 配置由使用者按本机网络设置，`.md` 文档不固定 domain 编号或本机 DDS 配置文件；启动前公布实际命令与预计时长，核验同域无残留进程，只清理本轮自建进程。

## 8. 提交与推送

按模块拆分提交并使用详细中文（推荐标签：`[接口]`、`[建图]`、`[规划]`、`[控制]`、`[定位]`、`[适配]`、`[安全]`、`[仿真]`、`[文档]`、`[规范]`）：

1. 提交前用 `git diff --cached --stat` 和 `git diff --cached --check` 检查 staged 内容；
2. 确认未包含用户保留文件、`log/`、缓存或构建产物；
3. 提交正文说明“为什么改、关键契约、验证结果、未覆盖范围”；
4. 确认分支为 `develop`，分别推送改动仓到 `origin/develop`（不 force push、不制造空提交），推送后核验 `HEAD == origin/develop`。

## 9. 输出标准

- 用户可见输出使用中文；命令、路径、topic、frame 与代码标识使用反引号；
- 数学公式使用规范 LaTeX：行内 `$...$`，块级 `$$...$$`；
- 明确区分 `已验证`、`已实现未运行`、`未通过`、`未验证`，对未验证假设标注 `[Confidence: High/Medium/Low]` 与证据边界。
