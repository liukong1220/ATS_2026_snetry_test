# O1 / O1b / O0 优化与验证账本

本次授权为完整 O1（含 O1b）与 O0。验收状态只随实际证据更新，不能由构建或局部修复推导全场通过。历史失败见 [11 月 17 日前一轮记录](20261117_o1a_o0.md)。

## 基线与完成门

本机 `ats-OMEN-by-HP-Gaming-Laptop-16-wf0xxx`，工作区 `/home/ats/ATS_2026_snetry_test`。起始三个仓库均为干净的 `develop`：根仓 `9404ee2a93a315804f49da15a0923c8b3d150753`，导航仓 `a51ce3a273dfa5ea7916f11405774bae28215a44`，仿真仓 `0b0d5df449ccff3c1e0145f1b33301657bd34541`。其他嵌套仓的未知修改不纳入。

- O1-A：V1 参考安全与完整证据、不可达终止契约、T1 known-free 与重规划活性。
- O1-B：2027 完整 64 点与 A–K，包括保存质量、动态排除、重载、20 次实体障碍延迟和独立故障。
- O1 完整设计：增量 EDT、比赛先验与障碍层分离及 geofence、动态对象跟踪与前向膨胀；按报告要求先收敛地图/安全失败。
- O1b：保持唯一快照 owner 和数值语义，核验不可变版本、epoch/租约、按需显示及最终 Release 性能。
- O0：同任务连续 EXECUTE 换轨，前后各 0.25 s 仿真时间，参考/MPC/仲裁/底盘/真值分别统计跌速，合格巡航严格 <0.2 m/s；无事件为未触发。

## 第一轮范围与验证

导航仓：`ats_navigation_interfaces/msg/ExecutionCommand.msg`、`ats_goal_manager/src/ats_goal_manager_node.cpp`、`ats_goal_manager/test/test_goal_manager_fast_maps.py` 及必要直接测试。保留原规划 publication，不把重新验证的地图序号写成 MINCO 原始来源；单独携带最终验证地图身份和完整三维 TF。

根仓：`scripts/nav_tracking_recorder.py`、`scripts/analyze_nav_tracking.py`、`scripts/test_mujoco_minco_mpc_chain.sh`、相应跟踪/故障测试；本账本、主报告和 2027 最终方案。先补旧实现能失败的离线反例，再修所有者。

可执行验证：`python3 -s scripts/test_analyze_nav_tracking.py`、新证据契约测试、`python3 -s scripts/test_mujoco_fault_contracts.py`、`bash -n scripts/test_mujoco_minco_mpc_chain.sh`、`./build.sh`、直接 CTest 和三个仓库的 `git diff --check`。创建 ROS 节点的测试与仿真均受下述隔离前置约束。

## 假设、限制与风险转入

V1 的坐标问题尚未定因：旧记录器丢失 z/roll/pitch，并使用 tick TF；同时执行权威可在更新地图复检候选，却只发布原规划地图序号。需要把最终验证身份、参考和同一次验证的 TF 绑定后再复核，不回写历史失败为通过。

当前 domain 历史审计覆盖 0..232，排除项不变。用户已明确授权本轮复用历史编号，执行[本轮清场规则](20261117_domain_reuse_rules.md)。WLAN 已核验为 `wlo1`，只用工作区 DDS 文件，不改全局。每次 guard 同时核验进程和 UDP 端口，退出只清理本轮身份匹配的子进程。

运行出现实际足迹冲突、身份/时间失效、LOST 或真值超差时，安全退出该次运行并保留失败，继续审查修复。known-free、footprint、健康期限、STOP 屏障和旧参考拒绝门保持有效。未带 RViz 的画面项和无独立评估的物理接触均不能写通过。

## 已执行与当前结论

证据根目录为 `log/nav_acceptance/20261117_o1_o0_completion`。两次 `./build.sh` 均完成 40 包 Release 构建；d20 的 Goal Manager 13 组测试全部通过，d21 的 MINCO 8 组、ROG 10 组、RC-ESDF 1 组、MPC 6 组窄回归通过，两套 launch `--show-args` 通过。上述为选定功能回归，不代表全包 lint 通过。新增证据契约 7 例、domain guard 3 例、故障夹具 11 例和原跟踪分析/O0 离线回归通过。

本轮修复把最终验证地图 publication/source 与完整三维 TF 绑定到 EXECUTE，同时保留 MINCO 原始规划来源；记录器保留 SE3、几何/距离载荷身份及全程载荷，分析器按该证据复核。不可达夹具绑定同一个目标，接受实际 JPS/A* 后端，并严格区分先到期的 goal timeout 与有界重规划预算终止，继续核验零速及旧参考不复活。该夹具仍需最终版本独立运行验证。

V1 d22（N=1，lio、有先验、未带 RViz）运行失败，不能计对称点速度通过：ATS action 返回 code 6，`localization did not recover before deadline`，定位终态距离 10.635 m。16 条已执行参考 q1 全部通过；q2 共 30 个冲突 tick，最长 24 tick / 12 个快照，持续冲突段 2 个；没有缺少快照载荷或 frame 配对的 tick。892 个快照、1161 个来源层载荷均未截断。MuJoCo 接触计数增量为 0，独立物理接触评估仍未验证。

O0 记录器在目标下发前就绪，完整保存 d22 换轨前后窗口：8 次换轨，合格巡航事件 0，主要分类为里程计接收超时、参考减速和安全状态切换；跌速指标未触发。参考 horizon 峰值 0.730416 m/s，自主/仲裁命令峰值 0.492536 m/s，底盘指令峰值 0.492536 m/s，独立真值峰值 0.449983 m/s，高速请求未触发。曲线和逐事件分类在 `v1_d22/o0/`。

T1 历史 d8 的离线组件回放只用于定位，不能代替闭环。最初误把保存的根参数 0.07 m 当作运行覆盖，产生全部 unknown 的错误回放；保留该产物并废止其根因结论。按 launch 和静态 TF 的 0.18 m 重放后，目标列 `(90,143)` 早期 20 帧 free、107 帧 unknown，2.699995002 s 收到低于已连接地面约 0.29 m 的回波后持续触发不连续下层支撑拒绝。离线场景射线确认同一位置存在上层平台和下层地面。完整 epoch/fusion 状态未重放，不能声称在线等价或据此清除 unknown。

参考平面修复保持规划 XY，转换到机体当前控制高度；运行期按实际提交的三维控制路径复检，缺 TF 撤销参考。d23 旧实现 3 个反例失败；第三次 40 包 Release 构建通过，d24 MINCO 8 组及 5 个相关节点用例通过。独立 V1 d25（N=1、lio、未带 RViz）因重复 steady 里程计接收超时按风险条件提前取消，返回 code 1，V1 未通过；18 条执行参考 q1 全部通过，1388 个实际位姿中 2 个诊断冲突 tick、仅跨 1 快照、无持续冲突，载荷/配对无缺口。接触计数增量为 0，独立物理接触未验证。guard 退出 1、进程/端口清空。

d25 O0 记录覆盖目标前后及全部退出过程：2 次连续换轨、合格巡航 0 次，均含里程计接收超时/参考减速，其中一项还含安全切换，保留全部分类与跌速。参考 horizon 峰值 1.972639 m/s，自主/仲裁命令 0.494309 m/s，底盘 0.494309 m/s，独立真值 0.439118 m/s；高速仍未触发。曲线在 `v1_plane_d25/o0/`。

O1b 证据工具另修复重复整条参考写盘：以完整几何/SE3/每 pose frame 与时间戳做内容 hash，先写载荷定义、每个授权保留序号/身份/验证地图。分析器拒绝缺失、迟到、损坏或矛盾载荷，同时兼容旧内联档。runner 的 YAML 授权流保留所有授权字段和参考头/原点数，完整参考由跟踪记录器保留。旧 d22 的离线存储测量保留 9020 次授权和 17 份内容，事件文件从 541854747 字节降到 9034159 字节；仅存储基准，不能推导在线性能提升。d26 新证据契约 13 例、捕获器 11 例含 ROS 收发及旧跟踪/故障/O0 回归通过并清场；尚需在线复验。

发现旧捕获器端到端测试会按 PID 自选 domain，已改为只继承 guard 分配。未受 guard 管理的负对照调用不计验收，事后审计范围及未清空编号限制见 domain 账本；没有清理未知进程。O1-A、O1-B、O1 完整设计及 O0 总体仍未完成，继续处理输入健康与 T1 准入契约。

## 每目标容差与运行输入收尾（进行中）

精确范围：导航仓 `NavigateToPose.action`、`PlannerGoal.msg`、Goal Manager 节点/CMake/新组件测试、MINCO 节点/头文件/直接节点测试；仿真仓 `rmuc_2025_mujoco.launch.py`、`parallel_lidar.py`、`sim_node.py` 及直接测试；根仓 `s3_mujoco_acceptance.py` 和容差/启动清理测试。仿真仓修改前基线为 `3f27996`；根仓、导航仓分别以 `1713f71`、`46b6145` 为本组回滚点。

DoD：每目标显式控制容差从 action 经同目标重规划进入 MINCO 准入，0 保持默认；非法/越界拒绝，抢占及后续默认目标不串用。建图仿真的默认收敛半径仍为 0.03m，显式请求上限 0.10m；本轮中间点请求 0.08m，最后点/重载首点请求 0。独立真值门继续中间 0.15m、最终 0.08m。MINCO 用请求内参数副本，footprint/unknown/known-free 原判定继续约束实际候选。T1 用完整 13 点 N=1，不以片段替代真实 B 第 10–13 点。

旧接口 d28 负对照及 runner 5 个反例失败，修复后第四次 `./build.sh` 的 40 包 Release 构建通过；d29 Goal Manager 14 组、MINCO 9 组、MPC 6 组、证据契约/捕获器和两套 launch 参数检查通过并清场。根仓 runner/场景工具 40 项通过；后续初始化清理窄回归 9 项通过。MuJoCo 依赖位于用户 site，验证工具先装载系统 NumPy/SciPy 再追加该 site；没有改动安装依赖。早期错误 Python 环境的导入失败保留，不计功能失败或通过。

同位姿离线 20k Livox 射线、20 段、8 线程的 CPU 亲和性微基准：原末尾 8 核约 0.79s/帧，独立 P 核约 0.66–0.68s，全部距离和命中点 hash 一致。仿真子进程现从继承的可用 CPU 中优先选独立高性能物理核，拓扑不可读时沿用旧分配；线程/射线/时间语义不变，17 个直接测试通过。该微基准不证明在线健康门已通过；d31 日志仍有少量 steady 接收超时，继续保留失败零速。

d27 不可达独立门已通过，后续行为改动后的最终故障矩阵仍待重跑。d30 临时入口遗漏 `scripts` 导入路径，Probe 初始化失败、未下发目标；发现并修复 runner 初始化异常未进入 launch 清理的问题。其完整命令/独立参数路径/PID/start-time 归属核验及补清保存在该批次，最终 clear=true，原启动失败保留。

d31独立T1全路线尝试N=1已结束：到达前6/13点，真值误差依次为0.0492002359/0.0105410347/0.0117540457/0.0116919939/0.0113221558/0.0326669434m。第7点ramp_bridge搜索/准入失败，有界挂起重规划预算code5，真值误差1.7183642842m，保持急停零速；真实B未进入。10条已执行参考q1无冲突、1475个有效实际位姿q2无冲突，配对/载荷无缺口；第7点没有执行参考，q1为无执行参考可评价而非“通过”。接触计数增量0、力0，独立物理接触未验证。guard command_exit=1、clear=true。原始结果不改写。

活动进程的16份有序参数输入已复制并hash，避免把根参数误当作launch最终值。30s墙钟绝对采样记录在`runtime_capture/cpu_rss_30s.json`：ROGMap约66.3%单核/167.1MiB、MINCO14.2%/55.6MiB、Goal Manager4.5%/26.5MiB、LiDAR工作进程543.1%/1373.8MiB；这是建图40Hz发布、planning 0.05m和记录器开启条件下的一次采样，不能与d22比赛模式直接算优化比例。

O1b补验范围仅为导航仓`ats_rog_map/CMakeLists.txt`和新增`ats_rog_map/test/test_planning_publisher_visualization.cpp`，不修改运行所有者。DoD为无订阅→三种视图晚加入→移除订阅期间，唯一规划快照生产者、占据/距离/梯度/geometry/policy保持相同且publication严格递增，三种视图确有非空输出；这不代替RViz现场画面验收。d31结束后Release单包构建成功（既有colcon underlay/CMP0074警告保留），独立d32的`ctest --test-dir build/ats_rog_map -R '^test_planning_publisher_visualization$' --output-on-failure`通过，command_exit=0、clear=true。

验收计数器另发现独立缺陷：真实B的entry/exit为x=12.90/16.10，而墙间corridor为x=13.30..15.57；旧计数要求端区距entry/exit≤0.15m且同时inside corridor，两者无交集。六个真实通道配置的完整往返反例均复现0次，绕行/中途离开通道的反例不误计，负对照在`tunnel_counter_negative.log`。本次先仅增加`scripts/test_s3_acceptance_tools.py`反例，d31退出后再修`s3_mujoco_acceptance.py`的端区计数：沿portal方向，取该轴与墙间通道重叠区端部，持续在通道内才计穿越；相关6项回归通过，输出计数轴区间供审查。d31没有实际通道采样，仍判失败；不改路线13点、footprint、q1/q2或物理接触门。

## 11 月 18 日 V1 与 O0

d33 源码时间戳预检将 ROGMap 测试目标的 CMake 更新视为运行产物失配，退出码 3，未启动仿真，清场完成。清理 ROGMap 构建目标后运行 `./build.sh`，40 包 Release 构建通过。d34 以同源 2027 先验、`lio`、同一尺寸和限幅运行对称点测试，`N=1`，未开 RViz 或 MuJoCo 窗口。 [构建记录](../../log/nav_acceptance/20261117_o1_o0_completion/build_release_fresh_d33.log) · [执行命令与源码哈希](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34.orchestration.json)

d34 action 成功，真值终点 `(14.628394925, 14.008269660)`，误差 `0.029575 m`，action 估计 `0.043816 m`。规划快照、参考、执行命令、急停、三路速度的所有权及无 Nav2 运行图通过。15 条 EXECUTE 参考 q1 零冲突，1642 个实际足迹样本 q2 零冲突，扫掠冲突为零，地图载荷与变换配对完整。保留 936 张快照及 3285 份来源层载荷。接触计数增量及记录力均为零，独立物理接触与实车未验。退出码 0，`after_cleanup.clear=true`。 [运行记录](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34.console.log) · [足迹结果](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34/tracking/goal_1_single/verdict.json) · [清场](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34/domain_guard.json)

O0 用 `scripts/analyze_o0_handoffs.py` 按 `--mpc-dt .05 --terminal-radius 1 --odometry-timeout 1` 分析。换轨前后各 `0.25 s`，9 次中 1 次符合现行统计口径，发生在 `15.180 s` 仿真时间。参考与真值跌速为零，MPC、仲裁与底盘均约 `0.0404 m/s`。该事件之前真值中位 `0.0327 m/s`。全程参考峰值 `2.148274 m/s`，三路指令峰值 `1.067871 m/s`，独立真值峰值 `0.892115 m/s`。参考 `>2 m/s` 累计采样跨度 `10.0 s`，指令与真值未超过 `2 m/s`。其余 8 次含主动减速、里程计接收超时或安全状态变化，高速换轨仍待验。 [事件与各窗口](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34/o0/summary.json) · [速度曲线](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34/o0/speeds.png) · [速度 CSV](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34/o0/speeds.csv)

## 测试执行简化

V1 runner 去掉产物时间戳、PGM 生成/安装副本和额外净空的启动拦截。S3 runner 去掉安装副本及重复容差检查，地图唯一发布者写入验收结果。S3 跟踪使用实际 MINCO 参数，采集首帧和跟踪判定在结果中保留。O0 仍提前启动记录，首帧缺失会留下证据失败状态，导航 action 继续执行。d34 保留原运行脚本与文件身份，简化版独立批次从 d35 开始。

将隧道计数的现行回归收敛到 B 洞往返和中途离开两条路径，移除重复的容差/启动 mock 与 d32 专用显示夹具。d32 的原始执行结果和测试源码保存于该批次，SHA-256 `c25396f0fdbe8bef9840577cbdc96ae0c9aa23b3ce221cf413331285e14e5b8d`。此次调整复用先验/重载、目标 yaw、B 洞计数与失败证据的现有用例，运行时安全状态机、数值场和几何参数沿用 d34 已测版本。

17 项现有关联用例、B 洞计数与失败证据两个用例、runner 退出回归通过。d35 以简化版 runner 完成一次独立 `unreachable` 仿真。名义目标通过后，故障目标 3 的 A* 扩展 98477 个节点后返回无路，净空 `0.341 m`。同一目标以 `progress watchdog exhausted the bounded suspended-replan wait` 终止，code 5，急停有效，MPC/仲裁/底盘三路零速，旧参考未恢复。runner 退出 0，domain 35 清空，无残留进程。 [运行日志](../../log/nav_acceptance/20261117_o1_o0_completion/unreachable_simplified_d35.console.log) · [源码与命令](../../log/nav_acceptance/20261117_o1_o0_completion/unreachable_simplified_d35.orchestration.json) · [清场](../../log/nav_acceptance/20261117_o1_o0_completion/unreachable_simplified_d35/domain_guard.json)

分仓本地提交为根仓 `bdba984`、导航仓 `ce4fa3f`、仿真仓 `9d30878`。另见各仓基线和第 5.3 节记录的 [SCAN-Planner 空间判定参考](../调研/无先验地图钻洞高速导航优化调研报告.md#无先验隧道的空间判定参考)。

## 精确增量 EDT

导航仓 `88994ca` 让 ROGMap 的单一发布器复用 RC-ESDF provider。阻塞与自由种子分开维护，变化行的中间距离列变化才重算该列。增删种子使用相同的精确欧氏变换，尺寸和分辨率改变时全量计算。缓存按当前栅格索引与种子掩码工作，地图发布、frame、epoch、lease 与消费者的不可变快照仍由原有链路判定。

现有 RC-ESDF 暴力解回归改用同一 provider 连续更新，包括增删阻塞、两种 unknown 策略、分辨率与尺寸变化。`ctest --test-dir build/ats_rc_esdf -R '^test_rc_esdf_map$' --output-on-failure` 通过，整仓 `./build.sh` 40 包 Release 通过，保留既有 ROGMap 编译告警。T1 d31 第 7 段 1253 张输入栅格，以 CPU 8 固定核 Release 交替回放，逐张有符号距离位模式哈希一致。p50 从 `2.13 ms` 降到 `0.98 ms`，p95 从 `2.23–2.28 ms` 降到 `1.23–1.25 ms`，计时含 provider 更新和距离复制。融合、DDS、规划控制及实车不在该测量内。 [回放输入哈希与结果](../../log/nav_acceptance/20261117_o1_o0_completion/edt_replay/summary.json) · [Release 构建](../../log/nav_acceptance/20261117_o1_o0_completion/build_release_incremental_edt.log)
## 首目标与实际场地净空

d36 的无窗口 T1 全路线尝试在第一点 `map (4.475, 7.925, -0.7854)` 结束。先后执行两条参考，两条 q1 以及 166 个实际位姿的 q2 和扫掠冲突均为零。后续候选反复发生足迹冲突，action 返回有界挂起重试耗尽 code 5，真值误差 `1.281819 m`。真实 B 未进入，domain 36 已清空。 [结果](../../log/nav_acceptance/20261117_o1_o0_completion/t1_incremental_edt_d36/run/summary.json)

d36 线上 231 个去重有效栅格用全量与增量 provider 重新计算，有符号距离逐位哈希一致。发布消息的 13,305,600 项 float32 距离与独立 SciPy EDT 计算完全相等。该结果排除了同一栅格输入下距离场数值变化。 [发布字段对比](../../log/nav_acceptance/20261117_o1_o0_completion/edt_replay/d36_published_field.json)

MuJoCo 实际碰撞模型使用按官方场地网格生成的 `audited_roof_*` 盒及高度场。离线评估固定在路线轴向，按约 0.02 m 间距采样直立矩形扫掠区域的竖直截面。`ramp_bridge` 的 576 个有顶板样本最小净高约 `0.26145 m`。真实 B 的 544 个有顶板样本最小约 `0.24889 m`，其中 20 个低于配置的 `0.249 m`。评估未考虑俯仰、悬挂、动态跟踪和接触。d31 晚期的 terrain 硬占据相关冲突中也有位于顶板之外的格，需结合实际融合输入定位原因。 [碰撞模型截面与 d31 关联](../../log/nav_acceptance/20261117_o1_o0_completion/t1_audited_collision_geometry.json)

## 带窗口复验与测试简化

d37 已开始下发 V1 目标。按用户现场观察要求中止该无窗口批次，清场后另起 domain。保留 9 条 q1 无冲突执行参考、404 个实际位姿及一次未达到持续条件的扫掠冲突。两个换轨窗口中无合格巡航事件。 [跟踪](../../log/nav_acceptance/20261117_o1_o0_completion/v1_incremental_edt_d37/tracking/goal_1_single/verdict.json) · [速度](../../log/nav_acceptance/20261117_o1_o0_completion/v1_incremental_edt_d37/o0_interrupted/summary.json)

d38 在解锁桌面启动 RViz2 和 MuJoCo 原生窗口。旧 runner 要求 RViz 订阅默认关闭的参考路径，导致未发目标。已清场。 [d38 日志](../../log/nav_acceptance/20261117_o1_o0_completion/v1_viewer_d38.console.log)

同一测试脚本允许 RViz 作为可选订阅端。删除了仅用于显示的 QoS、体素内容及截图成功检查，两个窗口共用截图采集函数。脚本语法、现有 shutdown 回归通过。d39 在相同 V1 参数下通过 ownership 检查并下发目标，RViz 截图显示网格、车模和点云，MuJoCo 原生窗口在解锁后启动。为切换无先验建图而中止 d39。已提交的 10 条参考、928 个实际位姿未发现跟踪冲突，domain 已清空。期间 30 秒绝对采样中 LiDAR worker 使用约 5.39 核，ROGMap 使用约 0.17 核。该段存在重复里程计接收超时。 [d39 截图](../../log/nav_acceptance/20261117_o1_o0_completion/v1_viewer_simple_d39/rviz.png) · [跟踪证据](../../log/nav_acceptance/20261117_o1_o0_completion/v1_viewer_simple_d39/tracking/goal_1_single/verdict.json)

## 无先验建图、在线 ESDF 与地图保存

d40 使用 `rmuc_2027` 建图 profile 和 `--max-goals 1`，从 `map (3.37, 9.03)` 出发，目标为 `map (5.975, 6.225, -0.785398163)`。生效的 `slam:=true`，未提供 `--prior` 或 `--reload`。Point-LIO 实时注册扫描，ROGMap 和 terrain 在 odom 中持续生成规划快照。RViz 使用本轮独立配置，打开 odom 累积层、实际供规划使用的有符号距离场显示 `/rog_map_adapter/viz/esdf_cloud` 和路径。ESDF 彩色图由现有数值快照编码。 [d40 命令](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/run/command.json) · [在线 ESDF 截图](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/mapping_esdf.png)

通过 `/rog_map/save_map` 在导航中保存。从 odom 累积静态层转到 map 系写出 `400×300` 的 PGM、0.10 m 分辨率和保存变换的 YAML、1,770 点的三维 PCD。40 个 PGM 黑色占据格与服务返回计数一致。在线 ESDF 随扫描更新，保存文件对应当次静态层快照。 [PGM](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/interim_map.pgm) · [YAML](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/interim_map.yaml) · [PCD](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/interim_map.pcd)

首目标 action 返回 code 0，真值 `(5.975170476, 6.229656357)`，误差 `0.004659477 m`。2 条执行参考和 386 个实际位姿无足迹冲突，扫掠冲突为 0。规划快照在 odom 系，记录覆盖 387 个来源代次。独立显示订阅收到了 109 个不同的 odom 时间戳。两张 RViz 截图可见 ESDF 随行驶扩展。原生 MuJoCo 查看器在锁屏期间等待，未取得该批次画面。独立物理接触未验证。退出码为 0，domain 40 已清空。 [首段结果](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/run/summary.json) · [行驶截图](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/mapping_route.png) · [在线 ESDF 时间戳](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/esdf_headers.yaml)

到达后的保存仍为 `400×300`、0.10 m 分辨率。PGM 有 92 个占据格，三维 PCD 有 3,227 点，另存的格中心 PCD 有 92 点，计数与服务返回一致。当前结论覆盖路线首段及保存接口，全场覆盖、保存质量、动态排除和重载仍待验。 [终点 PGM](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/run/map.pgm) · [YAML](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/run/map.yaml) · [PCD](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/run/map.pcd) · [保存校验](../../log/nav_acceptance/20261117_o1_o0_completion/mapping_viewer_d40/saved_map.sha256)
