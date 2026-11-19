# ATS 2026 哨兵导航研发规范

## 1. 角色与目标

你是本项目的精确代码修改者与闭环验证者。改动优先服务于 ATS 四驱四转舵轮哨兵的实际 ROS 2 导航链：先定位行为所有者，再做最小修改，并以源码、测试和运行证据共同证明结果。

每轮修改前先给出：

1. Definition of Done（DoD）；
2. 精确文件范围；
3. 可执行的验证清单；
4. 当前假设、未验证项和风险转入条件。

本规范使用“建议、优先、默认”描述工作方式。现场证据与本文不一致时，先记录差异、影响和验证方案，再选择更合适的路径。“风险转入条件”仅表示当前高风险运行安全退出；会话继续进行只读审查、证据整理和修复复盘。

实现、修复或优化类请求的默认交付链为“定位 -> 修改 -> 构建 -> 单测 -> 仿真 -> 文档 -> 分仓提交”。

### 1.1 当前状态与文档维护

当前模块进度以 `docs/调研/无先验地图钻洞高速导航优化调研报告.md` 第 5 节为准，顺序见 5.1，实施入口见 9.7。下一次实施优先接通 **无先验基地到基地的在线建图与局部重规划**，遇到窄通道时在洞外将车头对向入口。有图后沿用现有全局规划，局部规划绕障并接回路线。d40 无先验首段与保存、d34 V1 路线与足迹、d35 unreachable 局部结果继续按 9.8.11–9.8.12 和原始来源记账，当前主场景尚未运行。

O1-A全部门、S3和2027最终套件仍未通过。O1-B完整S3/A–K与O3新增算法分别选任务，不因回归失败自动扩大实现范围；本修复引入的失败须处理，历史或独立欠账继续记未通过。历史两场见9.5，11月16日专项/文档审查见9.8.6–9.8.9。S0/S1/S2a/S2b已有阶段交付，S3主干、O1b及部分O3已有实现；完整O2/O4、O5数值优化、方向后端、描述子恢复与前端A/B未完成。实车未验证，不按提交数或函数数估算总体完成百分比。

纯文档整理与代码审查只运行引用、JSON、证据 hash 和 Git 差异等必要检查，不自动启动新的算法修改、构建或仿真；行为修改仍执行本规范的完整验证链。主报告保留结论、契约、完成状态和剩余门，逐次 domain/PID/命令/日志放专门账本与证据索引，不追加重复流水账。

本阶段不修改系统文件或全局 ROS/DDS/SSH 配置，不处理与项目优化无关的内容。备份只写用户指定位置，保留当前工作树。10 月 22 日基线备份在 `/home/ats/Documents/ATS_20261022_git_baseline`，来源和逐文件校验见其 `manifest.json`；它不包含全部独立仓和忽略资产，不能直接视为完整可运行环境。

## 2. 项目事实与架构边界

### 2.1 目标主线

V1 目标链为：

```text
传感器 + 独立状态估计
-> ROGMap 概率占据/膨胀/3D ESDF
-> 地面投影与 2.5D 可通行语义
-> RC-ESDF 规划接口
-> 自研目标管理
-> JPS
-> MINCO S3 + 独立 yaw
-> footprint safety + Local Collision Repair
-> 全向 SE2 MPC
-> 四舵轮底盘
```

当前阶段建议明确区分：

- P2：ROGMap 地面适配、terrain/static wall/unknown 融合、规划地图唯一所有权、MINCO 单次规划本地不可变 snapshot 和安全停机；
- S3（P2 之后，部分实现、验收未通过）：建图模式已接入 odom 全局层、地面相对列语义、显式保存与增长地图匹配；无先验入口已能发布规划栅格。全场安全覆盖、真值精度和延迟门仍有失败，要求见 6.4；
- P3：自研 goal/action 状态机与 Nav2-free 启动；
- P4：连续 swept footprint、实车动力学约束与实车验证。

P3 完成前建议避免声称 Nav2-free。未在目标机测量前建议避免引用技术报告的 50 Hz、约 6 ms 或内存数据作为 ATS 实测性能。

### 2.2 稳定模块边界

- Point-LIO 继续提供 `/localization` 与 `/registered_scan`；ROGMap 不是定位器。
- ROGMap 活动实现位于 `src/ats_sentry_nav/ats_rog_map`，因此优先复用现有节点。
- RC-ESDF 保留 signed distance、unknown 与 footprint 语义。
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 `ats_swerve_mpc` 属于稳定架构边界。
- 四舵轮控制为车体系 `[vx, vy, wz]`，状态为世界系 `[x, y, yaw]`；建议避免迁入差速/ICR/`vy=0` 约束。
- 建议避免从 `/rog_map/esdf` 可视化 `PointCloud2` 反解析数值距离场。
- Point-LIO 当前是激光惯性里程计，代码中没有回环检测或位姿图。建图模式的漂移校正采用地图匹配（见 6.4），不引入回环后端，除非用户另行决定。
- ZLIO（z-localization）为 GPL-2.0，只借鉴思路，不拷贝代码，不整套接入，除非用户先确认许可与 A/B 决策。

### 2.3 雷达安装与速度链

2026-10-23 起雷达固定在底盘上（报告 2.1.8）：`gimbal_yaw_odom` 保留名字，语义是底盘系（原点取底盘底面中心）。导航不再依赖云台 yaw，因此：

- 两个速度兼容层的默认值按入口不同：
  - `ats_sentry_bringup/launch/bringup.launch.py` 默认 `launch_fake_vel_transform:=False`、`launch_chassis_vel_transform:=False`；
  - `ats_sentry_bringup/launch/real_robot_navigation.launch.py` 默认两个都是 `True`，`scripts/validate_navigation_config.py` 锁定这一默认。小陀螺继续用 fake yaw 稳定导航（用户 2026-10-30），所以这个入口没有关掉它们。
- 云台仍然存在（自瞄用）；需要云台坐标系速度的旧链路可以显式把两个参数设回 `True`。
- fake-yaw 关闭时保留 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF。
- 建议避免引入重复的 `base_footprint -> base_link` TF 发布者。
- 下游仍要求底盘坐标速度时，建议避免只关闭 chassis transform。
- 固定雷达迁移优先通过启动参数关闭兼容层，同时保持 topic、TF 和速度 frame 契约完整。

2026-11-16 用户要求三套 profile 统一为实车长宽 `0.581 x 0.571 m`、2027 仿真余量 `0.02 m`、ROG/terrain 车高门 `0.249 m`、`jps_safe_distance=0.36`、隧道 yaw/自旋启用；暂定平移上限 `5 m/s`、yaw `3 rad/s`、角加速度 `2 rad/s²`，MINCO 起点保护与 MuJoCo 轮速范围同步。此条取代旧的实车余量/车高配置保留要求；同步配置不代表 H2/M/H3 或实车验证通过。

## 3. 多仓库与文件所有权

以下是三个独立 Git 仓库，建议分别检查、提交和推送：

| 仓库 | 路径 | 主要所有权 |
| --- | --- | --- |
| 根仓库 | 实际总工作区，由`pwd`、`git rev-parse --show-toplevel`与主报告位置核验 | `docs/`、`scripts/`、`src/ats_sentry_bringup`、顶层规范 |
| 导航仓库 | `src/ats_sentry_nav` | ROGMap、adapter、RC-ESDF、JPS/MINCO、MPC、导航 launch |
| MuJoCo 仓库 | `src/sim/ats_mujoco_sim` | 仿真模型、传感器桥、MuJoCo launch |

双主机区分：11月17–19日行为验证记录来自 `/home/ats/ATS_2026_snetry_test`（`ats-OMEN-by-HP-Gaming-Laptop-16-wf0xxx`）；11月19日同步/文档审查实际在 `/home/kongliu/文档/27`（`kongliu-B760M-H-DDR4`）。两个路径按日期和主机记录，不能把其中一个硬编码为另一主机的当前路径。进入主机后先核验实际总工作区、HEAD、安装与在线WLAN，不跨主机套用依赖或运行结论。整仓行为构建使用实际总工作区的 `./build.sh` 做 Release；本轮纯同步/文档审查不启动构建或仿真。

修改前后均运行三个仓库各自的 `git status --short --branch`。未知修改和未跟踪文件默认属于用户：

- 建议避免 `git add -A`、`git add .`；
- stage 范围采用本轮列出的显式文件清单；
- 不删除、不覆盖、不提交无关用户文件；
- 建议避免 `git reset --hard`、`git checkout --` 等破坏性恢复；
- 发现重叠修改时先理解并合并，无法安全处理再询问用户。

## 4. 精确导航协议

### 4.1 定位方式

建议避免无目标地递归读取仓库。按以下顺序定位：

1. 用 `rg` 搜索目标 symbol、topic、参数、错误或调用点；
2. 读取目标函数/类的完整作用域；
3. 优先追踪直接 producer、consumer、launch 和测试；
4. 未知类型优先读取消息、service、action、头文件或 package manifest；
5. 默认忽略 `build/`、`install/`、`log/`、缓存、二进制、媒体和 `参考/`。

`参考/` 仅用于算法与许可证溯源，不是活动构建输入。仓库存在参考项目与活动源码同名 ROS 包，所有 colcon 命令建议显式使用：

```bash
colcon build --base-paths src ...
colcon test --base-paths src ...
```

### 4.2 闭环接口账本

跨模块修改建议核对以下字段，建议避免只看消息类型一致：

| 契约 | 必查内容 |
| --- | --- |
| frame | producer frame、TF 查询方向、consumer target frame |
| time | 观测时间、ROS/sim/steady clock、timeout、stale 行为 |
| map | resolution、origin/yaw、width/height、unknown、occupied、outside-map；细静态图降采样时保留所有有面积重叠的 occupied 单元 |
| ESDF | signed distance 正负号、unknown、梯度方向、截断、插值 |
| generation | JPS、MINCO clearance、footprint gate、repair 使用同一不可变快照 |
| QoS | reliability、durability、depth、late joiner 行为 |
| ownership | planning grid、TF、速度、急停和底盘输入均有唯一权威 |
| request | adapter 已并入 `ats_rog_map` 进程内（M3/M4），不再有投影 service 与 client pending request；规划地图只由 `/rog_map_adapter/planning_snapshot` 单消息发布，消费者按 `publication_sequence`、`source_generation`、`localization_epoch` 与 steady-clock 租约判断新鲜度，旧 epoch 与迟到数据丢弃 |
| fallback | map unready/stale、无路、unsafe trajectory、MPC 失败均输出确定性零速度 |
| map（建图模式，目标要求） | 全局静态层固定 40×30 m、0.1 m、原点为建图起点；12×12 m 计算窗口不是地图存储；超出全局范围的观测计数丢弃并记录，不静默丢弃 |

重要结论优先用两类独立证据交叉验证，例如“实现 + 单测”“launch + ROS graph”“日志 + 终点测量”。只有单一来源时明确标记 `[Confidence: Medium/Low]` 与限制。

generation 的证据建议区分 `ROGMap source generation`、adapter `publication_sequence`、`localization_epoch`、`MINCO local snapshot generation` 和执行参考的 stamp。当前活动契约是 `PlanningMapSnapshot`，已在单消息中携带 occupancy、signed distance、梯度和来源/发布/epoch 字段，消费者装载该数值场；不要继续按旧 `OccupancyGrid` 缺字段描述当前链。单次规划中的 JPS、MINCO clearance、footprint gate 与 repair 绑定同一不可变本地快照。跨进程结论仍须核对 producer、consumer 与 EXECUTE 的字段传递和拒绝证据；这些编号含义不同，不能要求它们数值相等，也不能只凭消息定义就声称端到端闭环验收通过。

## 5. 修改工作流

### 5.1 修改前

1. 定义 DoD、文件范围与测试命令。
2. 定位现有测试；无覆盖时明确说明并补最窄回归测试。
3. 重大功能或架构修改前，在每个将被修改且 tracked tree 已稳定的仓库创建中文基线提交；建议避免为此纳入用户未跟踪文件。
4. 记录当前分支、HEAD 和回滚点。

### 5.2 实施

- 修改范围聚焦拥有目标行为的模块，不做无关重构。
- 优先复用现有接口；新增跨进程数值数据时使用明确的 ROS interface，不用调试点云替代。
- 物理 occupancy、概率证据、ROG inflation、JPS clearance 与 footprint margin 建议分层，建议避免重复膨胀。
- 静态图比 planning grid 更细或分辨率不整除时，建议避免只采样输出单元中心；建议对输出 footprint 覆盖的源单元保守聚合，并保留静态图 origin 与 yaw。
- callback 并发共享地图/轨迹时使用不可变 snapshot 或明确同步。
- 失效安全状态定义为输入健康与规划安全的合取；单一 ready 消息不改变规划失败急停状态。
- `/rog_map_adapter/ready=true` 是持续续租的 heartbeat，不是永久状态；lease 超时后的确定性结果是地图与规划失效并触发急停。
- MPC 收到急停后清空 tracker；ready 恢复本身不会让急停前或无有效时间戳的旧 reference 重新驱动车辆。
- 最终安全 reference 优先在 snapshot/heartbeat 复核成功的提交点统一重定时，保持各 pose 相对时间，并在同一互斥区内先发布 `emergency_stop=false`、再发布 reference；时序竞争优先在提交点解决，而非放宽 MPC 的旧 reference 拒绝规则。Goal Manager 在每次急停上升沿记录 `stop_to_reference_guard_sec`（默认 0.01 s、范围 0.002–0.1 s）的提交屏障；急停后的候选 reference 必须等本节点时钟越过屏障后再提交，MPC 的旧 reference 拒绝门不放宽。该屏障用于跨 DDS `/clock` 回调竞态，不是永久 ready 状态，也不改变失败零速。
- dead code 只标记 `[Dead Code Suggestion]`，除非用户明确授权，否则不删除。

### 5.3 修改后

1. 先运行最窄构建与单测；失败先定位再扩大范围。
2. 运行 launch Python 语法和 `ros2 launch ... --show-args`。
3. 运行三个仓库各自的 `git diff --check`。
4. 在隔离 `ROS_DOMAIN_ID` 下运行 MuJoCo 闭环；是否带 RViz 见 7.1。
5. 保存失败日志并修复后重跑；建议避免把 topic 存在写成闭环通过。
6. 最后一次影响地图、规划、安全或控制行为的源码修改后，建议重跑对应闭环；建议避免沿用该修改前的 MuJoCo 结果作为最终证据。
7. 文档内容以已有源码与本轮实际运行证据为依据。
8. 更新主报告及 `docs/nav_acceptance/20261116_2027_final_test_plan.md`，分别记录 P2 已完成/未完成、2027 隧道与对称点速度实际结果、P3 边界和未复现性能。旧优化方向文件在本机不存在，不为此新建重复报告。

## 6. P2/P3 验收门禁

### 6.1 P2 ROGMap owner

P2 回归使用 `ats_rog_map` 进程内的规划地图所有者，并验证；MuJoCo 兼容 launch 保留的 `planning_grid_owner:=rog_map` 不能推广为所有入口都接受的参数：

- 正式参数由根仓`src/ats_sentry_bringup/params/node_params.yaml`拥有；MuJoCo按实际launch叠加2025公共、2027场景及LIO覆盖。包内`ats_rog_map/config/rog_map_ground_planning_mujoco.yaml`和`rog_map_ground_planning.yaml`是独立`ats_rog_map.launch.py`的默认输入，不能写成正式入口的第二份参数源。adapter已并入`ats_rog_map`，原独立包不再是活动构建输入；高度带排除有证据的地面回波并保留墙体和低矮障碍语义；
- 规划地图唯一所有者是 `ats_rog_map` 进程内的 adapter；`planning_grid_owner` 与 `rc_esdf_map` 的二选一入口已随 M3/M4 移除，`PLANNING_GRID_OWNER=rog_map` 仅作为测试脚本的兼容取值保留；
- `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 非空；
- adapter 在进程内直接读取 ROGMap 数值状态，不订阅 `/rog_map/esdf`；
- `/rog_map_adapter/planning_snapshot` publisher 数为 1 且 owner 为 adapter，`/rc_esdf/*` 话题不存在；
- 任一有效来源 occupied 保持 occupied；任一新鲜来源的明确 free 可消解其他来源 unknown；所有来源均无 free/occupied 证据时输出最终 unknown，并默认按障碍处理（建图模式例外见 6.4）；静态细栅格到 planning grid 优先按输出 footprint 保守聚合；真值表、非整除分辨率、平移 origin 与 yaw 建议由单测锁定；
- ROGMap 投影几何范围只定义其数据范围；范围外和 terrain/slope 的 `-1` 都表示缺少该来源证据，建议避免凭空覆盖另一来源的明确 free；
- static wall、height band、terrain、slope 和全来源 unknown 可阻断规划；ego unknown 清理默认值为 `0.0`。只有完成对应 profile 的定向矩形栅格化和边界测试后，才评估启用；栅格化和验收读取对应 profile 的长/宽/margin（当前车体配置为 `0.581 x 0.571 m`，三套 margin 均为 `0.02`），不混用旧固定尺寸或余量；外接圆不作为 footprint 外 unknown 的清理依据，occupied 始终保留；
- `ROGMap source generation`、adapter publication 与 epoch 可追溯、发布保持新鲜、单次规划固定 `MINCO local snapshot generation`；按 4.2 核对其与执行参考的绑定，不将字段存在当作端到端验证完成；
- `/minco/raw_path`、`/minco/reference_path`、MPC reference/predicted、`/cmd_vel/autonomy_raw`、`/cmd_vel/selected`、`/motion_control` 非空；
- MuJoCo 的 `/cmd_vel/autonomy_raw` 只有 MPC 一个 publisher，arbiter 读取该自主源；`/cmd_vel/selected` 只有 arbiter 一个 publisher，`twist_to_motion_ctrl` 是唯一底盘 Twist consumer。其他入口按实际 launch 推导速度 topic；历史 `/cmd_vel_mpc` 不作为现行验收话题；
- `/motion_control` 只有一个 publisher，MuJoCo/底盘只有一个 subscriber；
- map unready/stale、全 unknown、goal unreachable 时 `/planner/emergency_stop=true`，速度为零；unsafe trajectory 不进入发布链并保持或触发急停，未做专用运行注入时标为未验证门禁；
- 名义路线固定 `P2_FAULT_CASE=none`；adapter lease、进程内 projection 超时（service_timeout）、Point-LIO input stale、真实 unknown cell 和 unreachable 目标建议使用新 `ROS_DOMAIN_ID` 与新 MuJoCo launch 分别注入，建议避免在一次仿真中串行污染机器人状态；
- 使用 `SIGSTOP` 或输入中断分别注入进程内 projection 停滞、Point-LIO 输入 stale 和 adapter ready heartbeat 中断；建议在配置 deadline/lease 内观察 `ready=false -> emergency_stop=true -> autonomy_raw=0 -> selected=0 -> motion_control=0`；恢复后 generation 继续递增，没有新目标时迟到 response、旧 generation 和急停前 reference 保持失效；
- `/planner/emergency_stop` 与 `/minco/reference_path` 仍是两个独立 topic，不具备 DDS 跨 topic 原子顺序；P2 建议验证提交点重定时与恢复后旧轨迹不复活，并把结构化原子安全契约保留为后续加固项；
- 成功到达时记录终点坐标、位置误差、MINCO 离散 footprint 冲突采样数、MuJoCo 物理接触评估结果、规划点数、reference 点数和失败/恢复次数；无独立 contact evaluator 时建议把物理接触写为“未验证”，建议避免由 `footprint_collisions=0` 推导物理碰撞为零。

### 6.2 2027 实际导航任务与专项验收

下一次实际导航以无先验基地到基地为主，使用场地中心对称和当前起点确定一个远目标。T1/T2/T3、V1、F 按需要判断的具体问题选用，原结果继续保留，2025/2027 红框归档。 [2027 导航验证方案](docs/nav_acceptance/20261116_2027_final_test_plan.md)

- 基地到基地使用 `rmuc_2027`、`lio`、`slam:=true` 和空先验，从 `(3.37, 9.03, 0.465, 0)` 向 `map (24.63, 6.97)` 导航。直线距离 `21.36 m`，实际里程从轨迹测量。窗口内局部目标随在线建图更新，远目标接续和洞口正向对齐待实现。
- 隧道专项使用 `scripts/s3_mujoco_acceptance.py --profile rmuc_2027_tunnels --route tunnel_b_roundtrip` 或相应的 `ramp_south`、`six_tunnels`。T1 第 7 点为 `ramp_bridge`，真实 B 位于第 10–13 点。运行前声明完整尝试次数 N。
- 速度：`SCENE=rmuc_2027 TEST_PROFILE=single`，起点采用当前2027 launch默认`(13.40,2.00,0.550,0)`，以当前模型28×16m中心`(14,8)`得到目标`map (14.60,14.00)`，`GOAL_YAW=0`、`GOAL_TOLERANCE=0.08`。不依赖runner的默认2025起点，也不使用`red_box_2027`别名。
- 速度必须经ATS导航action到达；记录参考、MPC/仲裁/底盘命令和独立真值速度曲线。5m/s只是暂定上限，实际受距离、制动、曲率和净空约束；没有高速请求段时记未触发，不以直接发底盘命令或空场d108代替导航速度通过。
- 实际测试使用新domain、`lio`、工作区WLAN；有先验使用同源2027 PGM/PCD，无先验使用显式建图profile。2025缺先验不作为当前2027套件的前置条件。
- 最后行为修改后运行与改动直接相关的实际导航任务。选择 P2 故障专项时逐项核对真实注入锚点和前置等待。

旧`scripts/test_mujoco_nav_chain.sh`已删除，不执行不存在的入口。正式runner使用`/ats_navigate_to_pose`、`ats_navigation_interfaces/action/NavigateToPose`，断言没有Nav2进程和`/plan`。`PLANNING_GRID_OWNER=rog_map`仅是兼容取值，实际核验进程内adapter和唯一快照发布者；不能以变量取值或topic存在代替闭环通过。

历史Nav2隔离入口及d57原默认真值失败、d15严格0.05m变体通过继续归档，变体不替代原基线。2025 q2、2027 south及旧两场A–K未通过事实保留，不能因切换最终场景而标为已修复；历史兼容不自动加入当前2027最终套件。

### 6.3 P3 Nav2-free

自研 action、MINCO 直接目标入口和无 Nav2 graph 断言已有源码实现；实现与验收分别报告，不由入口存在推导阶段全部完成。

P3 标记条件为同时满足以下各项：

- 正式入口不启动Nav2。当前正式launch未声明`launch_nav2`，不传不存在的参数；兼容入口若仍声明该开关则显式设为`false`，并以下述源码与运行graph核验，不能只看开关值；
- 无 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`；
- MINCO 不订阅 `/plan`；
- 目标入口不调用 `nav2_msgs/action/NavigateToPose`；
- 自研 goal/action 支持 feedback、result、cancel、preempt、timeout；
- 2027真实隧道与对称点速度导航均在自研入口下通过，并满足本节其他条件；历史矩形/红框不替代当前套件。

### 6.4 建图模式与无先验导航（S3，部分实现，验收未通过）

完整路线历史基线（2026-11-14行为、11-15审查）：`slam:=true`已接入身份链、建图健康、射线清除、odom全局层、增长图匹配及显式保存。导航`e7bf500`/仿真`c42576a`的两场均失败：2025 d17到37/54点，第37段q2最长5 tick跨2快照；2027 d14前7/64点真值通过，第8点south误差0.194165m超差，action成功仍判失败。两场未完成最终保存/动态排除/重载及完整A–K，d13在注入前超时，不能计故障通过，来源见报告9.5/9.8.5。

11月16日另一主机的d103/d107启动失败与空场d108传感器/执行器结果继续归档。11月17日本机已修复初始锁存，原首目标d16正常退出0、真值0.005460892m且q1/q2通过；T1、V1和独立unreachable仍失败，其他四项P2故障通过，完整2027套件尚未通过。最新局部证据见报告9.8.10，不能据此清零历史全场失败或宣称完整A–K通过。

- 所有无先验运行与测试统一使用建图模式 `slam:=true`。从空地图开始，先验 PGM 和 PCD 输入留空，Point-LIO 持续提供位姿并累积在线扫描。
- 漂移校正：采用地图匹配（方式 3）。当前扫描与增长中的全局点云地图配准，修正写入 `map->odom`，经 `localization_fusion` 门控接受或拒绝。本阶段不做回环检测、位姿图和关键帧存储。
- 全局静态层：固定大数组，建图模式 40×30 m、0.1 m 分辨率，原点为建图起点，留出外延和漂移余量。超出范围的观测计数并记录。
- 计算窗口：12×12 m，跟随机器人，只服务 ESDF、JPS、MINCO。窗口移出时全局层保留，不随窗口丢弃。
- 场地尺寸：比赛概念尺寸28×15m与当前2027仿真模型/PGM的28×16m分开记录；本轮对称点计算使用实际2027模型中心(14,8)。10×10m只用于小测试，所有尺寸参数化，不据此修改比赛配置。
- 建图任务的远目标可以位于尚未观测区域，局部推进使用实时观测到的连通可通行区域。已提交段保持 known-free，超出窗口时的局部目标接续待实现。定位模式继续按 6.1 的 unknown 规则执行。
- 保存：只写静态层中未被射线清除、且观测占比达到阈值的格，输出 PGM+YAML（符合 map_server 契约）和 PCD。动态层不保存。
- 比赛（定位）模式：加载建图保存的 PCD 与静态层，`small_gicp_relocalization` 做先验重定位，输出 `map->odom`。比赛起点固定，重定位初值为建图起点。当前比赛入口仍保留 `use_static_map_obstacles=true` 和缺图 fail closed；“先验静态层只作引导/边界、局部障碍由实时观测决定”是 O1 的目标设计，不能写成全部模式已完成。
- 无先验测试须显式选择建图 profile。S3 runner 使用 `--profile`，省略 `--prior` 和 `--reload`。有先验定位使用对应 profile 所要求的 PGM 和 PCD。
- 验收（MuJoCo `lio` 模式，无 PGM、无 PCD）：
  - 建图中导航到目标（2026-11-06 用户授权放宽，数值为 `scripts/s3_mujoco_acceptance.py` 的默认参数，旧值可用参数复现）：中间路线点真值误差 ≤ 0.15 m（`--waypoint-tolerance-m`），终点与重载首目标 ≤ 0.08 m（`--final-tolerance-m`）；离散 footprint 冲突只统计被提交并由 MPC 执行（goal manager 下发 EXECUTE 且 `reference_stamp` 匹配）的 reference，必须为 0，READY 瞬态冲突只记录在 `summary.json` 的 `footprint_gate_collisions_transient_ready`。物理接触单独记为未验证，除非有独立接触评估；
  - `/rog_map_adapter/planning_snapshot` 发布者数为 1，owner 为 adapter；
  - 新障碍到规划器可见的延迟 p95 < 250 ms（`--latency-p95-limit-ms`，150 可复现旧门），同时报告最大值；
  - 导航跟踪记录器：q1（参考发布时无冲突）必过；q2（实际足迹进入后续快照占据格）仅当冲突持续 ≥ 3 个 tick 且跨 ≥ 2 个快照发布时判失败，其余记为诊断；q3（同一位姿跨快照翻转）为诊断，不判失败。参数 `NAV_TRACKING_PERSIST_MIN_TICKS`、`NAV_TRACKING_PERSIST_MIN_SNAPSHOTS`，`NAV_TRACKING_STRICT=1` 复现旧门；记录器判定用的足迹长宽与 margin 读自 `NAV_TRACKING_PROFILE_YAML` 的 `minco_planner` 段（默认是 2025 仿真 profile，margin 0.02；当前实车配置已同步为 0.02，仍须显式选择与本次运行一致的 profile）；
  - 地图修正前后不出现无法解释的跳变；
  - 保存地图与场景真值比对，报告漏检和误检格数；动态障碍不写入保存地图；
  - 地图失效、无路、unsafe 轨迹时 `/planner/emergency_stop=true`，速度为零。
- 6.1 的静态墙默认规则在比赛模式继续有效：`use_static_map_obstacles=true`、`allow_missing_static_map=false`。只有显式建图模式在 odom 全局层参与规划后才设置前者为 `false`、后者为 `true`；occupied 合并与已提交段 known-free 规则保持，不能用空闲占位图替代缺图。对应模式与融合规则由单测覆盖。

已定事项（用户 2026-11-04 确认）：
- 静态层坐标系采用混合方案：内部仍按 odom 系存储，修正 `map->odom` 时不搬移，障碍不跳动；保存时一次性变换到 map 系，输出 PGM、YAML 和三维 PCD，并记录保存时的 `map->odom`。规划链的坐标系在门 1 中单独决定。
- 规划链坐标系采用 odom 系。ESDF、JPS、MINCO、MPC 在 odom 中完成，控制器位姿使用 Point-LIO 的 odom。目标点和全局引导路径在每次重规划时用当前 `map->odom` 转换到 odom。修正只改变目标在 odom 中的位置，障碍和位姿同源，不跳动。
- 比赛起点固定。同一起点的下一场可直接复用建图地图。

### 6.5 当前专项与后续路线

H2 的 0.249 m 候选车高门尚未获充分净空证据；TUNNEL_FREE 未实现，真实 tunnel_B 往返、ramp_south、six_tunnels、持续 3 rad/s 平移与洞口切换、south 定位及有先验六隧道均未通过完整验收。隧道路线第 7 个前置 ramp_bridge 不算 tunnel_B 通过，实际 tunnel_B 位于第 10–13 点。

2026-11-16 用户直接授权将实车、2025/2027 仿真的 ROG 门和 terrain 头顶门同步为 `0.242+0.007=0.249 m`。旧的“等待 M 后再迁移参数”约定被本次指令取代；H2 净空证据、M 路线和 H3 实车验收仍未通过，不操作实车，也不把配置同步写成验收通过。

本专项的后续滚动规划、Point-LIO 数值优化、方向观测后端、机械升级不自动纳入当前任务。用户只要求审查/文档时，以完成审查/文档为交付；恢复优化时以当次授权范围推进，保留现有安全门与失败证据。

domain 已用编号见 `docs/nav_acceptance/20261115_domain_ledger.md`；分配前同时核验宿主占用。使用工作区 `log/nav_acceptance/dds/wlan_env.sh` / `wlan.xml` 和 `ROS_LOCALHOST_ONLY=0`。11月17日本机实际接口为 `wlo1`；另一台主机11月16日使用 `wlx500ff5500d30`，两者分开记录；每次启动前核验实际在线 WLAN、XML 与环境一致，不把本机接口名硬编码为其他机器的要求。配置缺失按 `20261114_runtime_inputs.json` 在工作区恢复并记录路径/接口差异，不改全局 DDS，不退回 lo。每次新 domain / 新 launch，范围 0..232，排除 5、77、91/92 及已尝试编号。

截至11月17日d16收尾，账本与排除项已覆盖全部0..232编号，核验见`log/nav_acceptance/20261117_o1a_o0/domain_allocation_final_audit.json`。后续ROS运行须先明确满足本段规则的隔离分配安排；端口空闲不能替代“未使用”条件。不擅自复用历史编号或越界分配，源码与已有证据的只读审查可以继续。

11月17日当前O1/O1b/O0轮次，用户已明确授权“允许本轮复用历史编号”，并要求制定清场规则。本轮按[历史domain复用规则](docs/nav_acceptance/20261117_domain_reuse_rules.md)执行：仍排除5、77、91/92，使用工作区WLAN，启动前核验同域进程和端口、持有本机锁；仅清理本轮记录且PID/start-time/domain匹配的进程，退出后再次核验。每次仿真/故障建立独立批次，不将历史编号标成首次使用，不删除旧证据。此授权不自动扩展到后续其他轮次。

## 7. 测试与结果报告

推荐次序：

```bash
# 并发按当前机器计算：总任务数 = min(nproc, MemAvailable / 1.5 GiB)；整仓构建用 ./build.sh（自动计算）
JOBS=$(awk -v n="$(nproc)" '/^MemAvailable:/ {j=int($2/1024/1536); if (j>n) j=n; if (j<1) j=1; print j}' /proc/meminfo)
# 单包构建必须带 Release；未显式设置可能生成 -O0 节点（历史实测），当前验证口径见报告第 6 节；./build.sh 已带。
MAKEFLAGS=-j${JOBS} colcon build --base-paths src --packages-select <targets> --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src --packages-select <targets>
colcon test-result --test-result-base build/<package> --verbose
python3 -m py_compile <changed_launch_files>
git diff --check
```

包级 lint 若被既有无关文件拖红，建议同时：

1. 报告既有失败及其文件；
2. 单独运行本轮相关 GTest/CTest；
3. 不借机格式化或重写无关模块；
4. 不把“功能测试通过”写成“全包测试通过”。

最终报告建议列出：

- 修改文件；
- 实际执行的命令与结果；
- 2027隧道穿越结果与对称点速度曲线/终点真值；历史红框另列归档；
- 路径/reference/MPC/底盘话题与唯一所有权；
- MINCO 离散 footprint 冲突采样数、MuJoCo 物理接触评估结果和每个失败原因；
- stale/unknown/unreachable 安全停机结果；
- 未执行测试与残余风险；
- 三个仓库的基线/最终 commit ID 和 push 结果。

### 7.1 RViz 与 MuJoCo 窗口

2026-10-29 起用户允许闭环测试不开 RViz 和 MuJoCo 窗口（`USE_RVIZ=false USE_MUJOCO_VIEWER=false`），由代理在后台看日志、话题和录制器；
报告里写明"未带 RViz"，车模颜色、画面这类只能看屏幕的项仍记为未验证，不用日志代替。启动前仍告诉用户命令、`ROS_DOMAIN_ID` 和预计时长。
用户要现场观察时按下面的方式带 RViz：

- 用 `USE_RVIZ=true`（`RVIZ_DELAY_SEC` 默认 18 s）运行 `scripts/test_mujoco_minco_mpc_chain.sh`；runner 会拉起
  `src/sim/ats_mujoco_sim/rviz/mujoco_navigation.rviz` 并把窗口截图存到 `/tmp/ats_minco_mpc_<profile>_<domain>_rviz.png`。
  需要可用的`DISPLAY`，启动前读取当前桌面环境，不沿用旧宿主的固定值。该配置显示`/localization`（红箭头）、`/odometry`（Point-LIO，绿箭头）、
  `/simulation/ground_truth/odometry`（仿真真值，蓝箭头）、`/registered_scan`、车辆、传感器坐标轴和TF；静态图、规划栅格、ESDF、ROGMap范围、路径及安全调试组默认关闭，可手动打开。新增话题时同步维护该`.rviz`。
- `USE_RVIZ=true` 时 runner 同时打开 MuJoCo 自带的 3D 窗口（`USE_MUJOCO_VIEWER` 默认等于 `USE_RVIZ`，可显式设为 `false`）。
  关闭这个窗口会让仿真停止；Wayland 会话里它是原生 Wayland 窗口，`xwd` 抓不到，runner 只在 X11 窗口存在时才保存
  `/tmp/ats_minco_mpc_<profile>_<domain>_mujoco.png`，否则打印 NOTE。`mujoco_navigation.rviz` 现在还显示车辆模型（RobotModel，
  由 `ats_robot_description/urdf/ats_sentry.urdf.xacro` 生成，只在 `/tf_static` 发布 `gimbal_yaw_odom -> part_*` 恒等边）；ROGMap原生occ/inf_occ/unk/esdf体素默认关闭，当前点云采用Points、Decay Time=0，不能再描述为默认显示灰色体素。
- 桌面锁屏（`loginctl` 的 `LockedHint=yes`）时 MuJoCo 的原生 Wayland 窗口出不来（窗口线程一直在等合成器，日志只有"正在后台打开窗口"，没有"已启动"），RViz 的 X11 窗口虽然还能被 `xwd` 截图，但没有人能现场观察；
  这样的运行要在报告里写明"无人现场观察"，窗口的画面和稳定性记为未验证。runner 的"当前位姿"（终点判据、故障注入的起点查询）在 `lio` 模式读 map 系的 `/simulation/ground_truth/odometry`，
  不读 `/localization`（`lio` 下它是 Point-LIO 的 odom 系直通，原点在出生点下方的地面）；truth 模式仍读 `/localization`。
- 启动前先告诉用户命令、`ROS_DOMAIN_ID` 和预计时长；实时率须按本机、场景和本次射线预算测量，旧硬件的 0.06–0.4 倍实时记录不能作为当前 MID360 配置的性能承诺。测试进行时不修改正在被脚本读取的文件。
- 结束后读取截图，在报告里分开写「RViz 上应该看到什么」和「实际看到什么」；截图只是单帧，不能替代终点测量与话题证据。
- 用户已允许无头闭环；每次均在报告里写明哪些结果没有经过 RViz 观察，画面项保持未验证。

## 8. 提交与推送

提交建议按内容拆分，使用详细中文，建议避免把接口、算法、安全、仿真和文档混成一个含糊提交。推荐标签：

```text
[接口] 定义ROGMap地面数值快照与generation契约
[适配] 融合ROGMap地面投影与2.5D可通行语义
[安全] 固化MINCO地图快照并收敛规划急停状态
[仿真] 使用2027隧道与对称点速度验收
[文档] 记录P2已验证边界与未完成范围
[建图] 增加建图模式全局静态层与地图匹配校正
[规范] 建立ATS多仓导航闭环研发与验收规则
```

每次提交前：

1. 用 `git diff --cached --stat` 和 `git diff --cached --check` 检查 staged 内容；
2. 确认未包含用户文件、缓存、模型或生成物；
3. 提交正文说明“为什么改、关键契约、验证结果、未覆盖范围”；
4. 分别推送三个仓库的 `develop` 到 `origin/develop`；
5. 推送失败时保留本地提交并报告远端错误，不做 force push。
6. 推送前确认当前分支为 `develop`；推送后记录本地 HEAD 与 `origin/develop` 一致性；未修改的仓库建议避免制造空提交。

## 9. 输出标准

- 用户可见输出使用中文；命令、路径、topic、frame 与代码标识使用反引号。
- 数学公式使用规范 LaTeX：行内 `$...$`，块级 `$$...$$`。
- 区分 `已验证`、`已实现未运行`、`推断`、`未实现`。
- 对未验证假设标注 `[Confidence: High/Medium/Low]` 并说明证据边界。
- 建议避免用编译通过、topic存在、端点地图预检或单次局部运动替代2027隧道/导航速度闭环验收。
