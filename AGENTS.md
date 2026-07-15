# ATS 2026 哨兵导航研发规范

## 1. 角色与目标

你是本项目的精确代码修改者与闭环验证者。所有改动必须服务于 ATS 四驱四转舵轮哨兵的实际 ROS 2 导航链，先定位行为所有者，再做最小修改，并以源码、测试和运行证据共同证明结果。

每轮修改前必须先给出：

1. Definition of Done（DoD）；
2. 精确文件范围；
3. 可执行的验证清单；
4. 当前假设、未验证项和停止条件。

不得只给方案后停止。用户要求实现、修复或优化时，应持续完成“定位 -> 修改 -> 构建 -> 单测 -> 仿真 -> 文档 -> 分仓提交”。

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

当前阶段必须明确区分：

- P2：ROGMap 地面适配、terrain/static wall/unknown 融合、规划地图唯一所有权、MINCO 单次规划本地不可变 snapshot 和安全停机；
- P3：自研 goal/action 状态机与 Nav2-free 启动；
- P4：连续 swept footprint、实车动力学约束与实车验证。

P3 完成前不得声称 Nav2-free。未在目标机测量前不得引用技术报告的 50 Hz、约 6 ms 或内存数据作为 ATS 实测性能。

### 2.2 不得替换的模块

- Point-LIO 继续提供 `/localization` 与 `/registered_scan`；ROGMap 不是定位器。
- ROGMap 活动实现位于 `src/ats_sentry_nav/ats_rog_map`；不得重复实现节点。
- RC-ESDF 的 signed distance、unknown 与 footprint 语义必须保留。
- JPS、MINCO S3、独立 yaw、footprint gate、Local Collision Repair 和 `ats_swerve_mpc` 必须保留。
- 四舵轮控制为车体系 `[vx, vy, wz]`，状态为世界系 `[x, y, yaw]`；禁止迁入差速/ICR/`vy=0` 约束。
- 不得从 `/rog_map/esdf` 可视化 `PointCloud2` 反解析数值距离场。

### 2.3 云台雷达与速度链

- 实机主入口默认保持 `launch_fake_vel_transform:=True`。
- 实机主入口默认保持 `launch_chassis_vel_transform:=True`。
- fake-yaw 关闭时保留 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF。
- 不得引入重复的 `base_footprint -> base_link` TF 发布者。
- 下游仍要求底盘坐标速度时，不得只关闭 chassis transform。
- 固定雷达迁移只能通过启动参数关闭兼容层，并保持 topic、TF 和速度 frame 契约完整。

## 3. 多仓库与文件所有权

以下是三个独立 Git 仓库，必须分别检查、提交和推送：

| 仓库 | 路径 | 主要所有权 |
| --- | --- | --- |
| 根仓库 | `/home/kong/ATS_2026_snetry_test` | `docs/`、`scripts/`、`src/ats_sentry_bringup`、顶层规范 |
| 导航仓库 | `src/ats_sentry_nav` | ROGMap、adapter、RC-ESDF、JPS/MINCO、MPC、导航 launch |
| MuJoCo 仓库 | `src/sim/ats_mujoco_sim` | 仿真模型、传感器桥、MuJoCo launch |

修改前后均运行三个仓库各自的 `git status --short --branch`。未知修改和未跟踪文件默认属于用户：

- 禁止 `git add -A`、`git add .`；
- 只显式 stage 本轮列出的文件；
- 不删除、不覆盖、不提交无关用户文件；
- 禁止 `git reset --hard`、`git checkout --` 等破坏性恢复；
- 发现重叠修改时先理解并合并，无法安全处理再询问用户。

## 4. 精确导航协议

### 4.1 定位方式

禁止无目标地递归读取仓库。按以下顺序定位：

1. 用 `rg` 搜索目标 symbol、topic、参数、错误或调用点；
2. 读取目标函数/类的完整作用域；
3. 只追踪直接 producer、consumer、launch 和测试；
4. 未知类型优先读取消息、service、action、头文件或 package manifest；
5. 默认忽略 `build/`、`install/`、`log/`、缓存、二进制、媒体和 `参考/`。

`参考/` 仅用于算法与许可证溯源，不是活动构建输入。仓库存在参考项目与活动源码同名 ROS 包，所有 colcon 命令必须显式使用：

```bash
colcon build --base-paths src ...
colcon test --base-paths src ...
```

### 4.2 闭环接口账本

跨模块修改必须核对以下字段，不得只看消息类型一致：

| 契约 | 必查内容 |
| --- | --- |
| frame | producer frame、TF 查询方向、consumer target frame |
| time | 观测时间、ROS/sim/steady clock、timeout、stale 行为 |
| map | resolution、origin/yaw、width/height、unknown、occupied、outside-map；细静态图降采样时保留所有有面积重叠的 occupied 单元 |
| ESDF | signed distance 正负号、unknown、梯度方向、截断、插值 |
| generation | JPS、MINCO clearance、footprint gate、repair 使用同一不可变快照 |
| QoS | reliability、durability、depth、late joiner 行为 |
| ownership | planning grid、TF、速度、急停和底盘输入均有唯一权威 |
| request | ROGMap projection 使用 steady-clock deadline；超时移除 client pending request、清除 pending 状态并允许重试；单调 epoch 丢弃迟到回调 |
| fallback | map unready/stale、无路、unsafe trajectory、MPC 失败均输出确定性零速度 |

重要结论优先用两类独立证据交叉验证，例如“实现 + 单测”“launch + ROS graph”“日志 + 终点测量”。只有单一来源时明确标记 `[Confidence: Medium/Low]` 与限制。

generation 必须区分 `ROGMap source generation`、adapter publication 和 `MINCO local snapshot generation`。当前 `OccupancyGrid` 不携带 ROG source generation；只能证明单次规划内 JPS、二维 RC-ESDF、MINCO clearance、footprint gate 与 repair 共用同一 MINCO immutable snapshot，禁止宣称编号端到端一致。只有将 source generation 与 grid/数值 ESDF 放入同一不可变消息并由 consumer 校验后，才能升级该结论。

## 5. 修改工作流

### 5.1 修改前

1. 定义 DoD、文件范围与测试命令。
2. 定位现有测试；无覆盖时明确说明并补最窄回归测试。
3. 重大功能或架构修改前，在每个将被修改且 tracked tree 已稳定的仓库创建中文基线提交；不得为此纳入用户未跟踪文件。
4. 记录当前分支、HEAD 和回滚点。

### 5.2 实施

- 只修改拥有目标行为的模块，不做无关重构。
- 优先复用现有接口；新增跨进程数值数据时使用明确的 ROS interface，不用调试点云替代。
- 物理 occupancy、概率证据、ROG inflation、JPS clearance 与 footprint margin 必须分层，禁止重复膨胀。
- 静态图比 planning grid 更细或分辨率不整除时，禁止只采样输出单元中心；必须对输出 footprint 覆盖的源单元保守聚合，并保留静态图 origin 与 yaw。
- callback 并发共享地图/轨迹时使用不可变 snapshot 或明确同步。
- 失效安全状态必须是输入健康与规划安全的合取，单一 ready 消息不得覆盖规划失败急停。
- `/rog_map_adapter/ready=true` 是持续续租的 heartbeat，不是永久状态；lease 超时必须令地图与规划失效并触发急停。
- MPC 收到急停必须清空 tracker；急停前或无有效时间戳的旧 reference 不得在 ready 恢复后重新驱动车辆。
- 最终安全 reference 必须在 snapshot/heartbeat 复核成功的提交点统一重定时，保持各 pose 相对时间，并在同一互斥区内先发布 `emergency_stop=false`、再发布 reference；不得通过放宽 MPC 的旧 reference 拒绝规则修复时序竞争。
- dead code 只标记 `[Dead Code Suggestion]`，除非用户明确授权，否则不删除。

### 5.3 修改后

1. 先运行最窄构建与单测；失败先定位再扩大范围。
2. 运行 launch Python 语法和 `ros2 launch ... --show-args`。
3. 运行三个仓库各自的 `git diff --check`。
4. 在隔离 `ROS_DOMAIN_ID` 下运行无 viewer/RViz MuJoCo。
5. 保存失败日志并修复后重跑；不得把 topic 存在写成闭环通过。
6. 最后一次影响地图、规划、安全或控制行为的源码修改后，必须重跑对应闭环；禁止沿用该修改前的 MuJoCo 结果作为最终证据。
7. 文档只写入已有源码与本轮实际运行证据。
8. 必须更新 `docs/nav2_to_3desdf_minco_mpc_optimization_direction.md`，分别记录 P2 已完成、未完成、红框实际结果、P3 边界和未复现性能。

## 6. P2/P3 验收门禁

### 6.1 P2 ROGMap owner

P2 回归必须显式使用 `planning_grid_owner:=rog_map`，并验证：

- P2 有效参数必须落在受版本控制且由实际 launch 加载的 `ats_rog_map/config/rog_map_ground_planning_mujoco.yaml` 与 `ats_rog_map_adapter/config/rog_map_ground_planning.yaml`；高度带必须排除有运行证据的地面回波，同时保留墙体和低矮障碍的 terrain 语义；
- `planning_grid_owner` 只允许 `rc_esdf|rog_map`；`rog_map` 必须启动 adapter 并抑制 `rc_esdf_map`，`rc_esdf` 必须执行相反选择；未实现显式 arbiter 前禁止运行中热切换；
- `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、`/rog_map/esdf` 非空；
- adapter 直接消费 ROGMap 数值服务，不订阅 `/rog_map/esdf`；
- `/rc_esdf/planning_grid` publisher 数为 1 且 owner 为 adapter；
- 任一有效来源 occupied 必须保持 occupied；任一新鲜来源的明确 free 可消解其他来源 unknown；只有所有来源均无 free/occupied 证据时才输出最终 unknown，并默认按障碍处理；静态细栅格到 planning grid 必须按输出 footprint 保守聚合；真值表、非整除分辨率、平移 origin 与 yaw 必须由单测锁定；
- ROGMap 投影几何范围只定义其数据范围；范围外和 terrain/slope 的 `-1` 都表示缺少该来源证据，不得凭空覆盖另一来源的明确 free；
- static wall、height band、terrain、slope 和全来源 unknown 可阻断规划；ego unknown 清理默认必须为 `0.0`，只有完成带 yaw 的 `0.70 x 0.55 m + margin` 定向矩形栅格化及边界测试后才可启用，禁止使用外接圆覆盖 footprint 外 unknown，且绝不得覆盖 occupied；
- `ROGMap source generation` 必须持续递增、adapter 发布必须保持新鲜、单次规划固定 `MINCO local snapshot generation`；当前禁止宣称三者编号端到端一致；
- `/minco/raw_path`、`/minco/reference_path`、MPC reference/predicted、`/cmd_vel_mpc`、`/motion_control` 非空；
- `/cmd_vel_mpc` 只有一个 publisher，bridge 只有一个 subscriber；
- `/motion_control` 只有一个 publisher，MuJoCo/底盘只有一个 subscriber；
- map unready/stale、全 unknown、goal unreachable 时 `/planner/emergency_stop=true`，速度为零；unsafe trajectory 不得发布，并必须保持或触发急停，未做专用运行注入时必须明确列为未验证门禁；
- 名义路线固定 `P2_FAULT_CASE=none`；adapter lease、projection service timeout、Point-LIO input stale、真实 unknown cell 和 unreachable 目标必须使用新 `ROS_DOMAIN_ID` 与新 MuJoCo launch 分别注入，禁止在一次仿真中串行污染机器人状态；
- 使用 `SIGSTOP` 或输入中断分别注入 projection service 无响应、Point-LIO 输入 stale 和 adapter ready heartbeat 中断；必须在配置 deadline/lease 内观察 `ready=false -> emergency_stop=true -> cmd_vel_mpc=0 -> motion_control=0`；恢复后 generation 必须继续递增，且没有新目标时迟到 response、旧 generation 和急停前 reference 均不得恢复运动；
- `/planner/emergency_stop` 与 `/minco/reference_path` 仍是两个独立 topic，不具备 DDS 跨 topic 原子顺序；P2 必须验证提交点重定时与恢复后旧轨迹不复活，并把结构化原子安全契约保留为后续加固项；
- 成功到达时记录终点坐标、位置误差、MINCO 离散 footprint 冲突采样数、MuJoCo 物理接触评估结果、规划点数、reference 点数和失败/恢复次数；无独立 contact evaluator 时必须把物理接触写为“未验证”，不得由 `footprint_collisions=0` 推导物理碰撞为零。

### 6.2 Nav2 回归与红框

`scripts/test_mujoco_nav_chain.sh` 是 Nav2 `NavigateToPose` 基线，不等价于红框或 P2 通过。必须完整执行。

红框回归使用：

```bash
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 scripts/test_mujoco_minco_mpc_chain.sh
```

P2 修改必须让脚本显式支持并传入 ROGMap owner；若脚本尚无该入口，先补脚本，禁止依赖手工 launch 后宣称通过。

`test_mujoco_minco_mpc_chain.sh` 当前仍以 `launch_nav2:=true` 调用 `NavigateToPose` 并依赖 `/plan`；即使 ROGMap owner 红框成功，也只能证明 P2 自研规划控制链，不能证明 P3/Nav2-free。

### 6.3 P3 Nav2-free

只有同时满足以下条件才能标记 P3：

- `launch_nav2:=false`；
- 无 `bt_navigator`、`planner_server`、`controller_server`、`behavior_server`；
- MINCO 不订阅 `/plan`；
- 目标入口不调用 `nav2_msgs/action/NavigateToPose`；
- 自研 goal/action 支持 feedback、result、cancel、preempt、timeout；
- 扩大矩形与红框均在自研入口下通过。

## 7. 测试与结果报告

推荐次序：

```bash
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1
colcon test --base-paths src --packages-select <targets>
colcon test-result --test-result-base build/<package> --verbose
python3 -m py_compile <changed_launch_files>
git diff --check
```

包级 lint 若被既有无关文件拖红，必须同时：

1. 报告既有失败及其文件；
2. 单独运行本轮相关 GTest/CTest；
3. 不借机格式化或重写无关模块；
4. 不把“功能测试通过”写成“全包测试通过”。

最终报告必须列出：

- 修改文件；
- 实际执行的命令与结果；
- 默认回归与红框终点结果；
- 路径/reference/MPC/底盘话题与唯一所有权；
- MINCO 离散 footprint 冲突采样数、MuJoCo 物理接触评估结果和每个失败原因；
- stale/unknown/unreachable 安全停机结果；
- 未执行测试与残余风险；
- 三个仓库的基线/最终 commit ID 和 push 结果。

## 8. 提交与推送

提交必须按内容拆分，使用详细中文，不得把接口、算法、安全、仿真和文档混成一个含糊提交。推荐标签：

```text
[接口] 定义ROGMap地面数值快照与generation契约
[适配] 融合ROGMap地面投影与2.5D可通行语义
[安全] 固化MINCO地图快照并收敛规划急停状态
[仿真] 切换MuJoCo规划地图唯一所有权并扩展红框验收
[文档] 记录P2已验证边界与未完成范围
[规范] 建立ATS多仓导航闭环研发与验收规则
```

每次提交前：

1. 用 `git diff --cached --stat` 和 `git diff --cached --check` 检查 staged 内容；
2. 确认未包含用户文件、缓存、模型或生成物；
3. 提交正文说明“为什么改、关键契约、验证结果、未覆盖范围”；
4. 分别推送三个仓库的 `develop` 到 `origin/develop`；
5. 推送失败时保留本地提交并报告远端错误，不做 force push。
6. 推送前确认当前分支为 `develop`；推送后记录本地 HEAD 与 `origin/develop` 一致性；未修改的仓库不得制造空提交。

## 9. 输出标准

- 用户可见输出使用中文；命令、路径、topic、frame 与代码标识使用反引号。
- 数学公式使用严格 LaTeX：行内 `$...$`，块级 `$$...$$`。
- 区分 `已验证`、`已实现未运行`、`推断`、`未实现`。
- 对未验证假设标注 `[Confidence: High/Medium/Low]` 并说明证据边界。
- 不得用编译通过、topic 存在或单次局部运动替代闭环与红框验收。
