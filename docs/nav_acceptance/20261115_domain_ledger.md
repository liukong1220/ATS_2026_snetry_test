# ATS导航domain历史账本

整理日期：2026-11-15。来源：根仓88b7c428的报告9.0.1。本轮仅文档审查，没有分配新domain或启动ROS。以下编号与状态均按原记录保留；分配前还必须核验宿主当前占用，不能从旧表或沙箱/proc推断宿主空闲。排除5、77及已尝试的91/92；新运行分配后立即更新本文件。

#### 9.0.1 domain 账本（截至 2026-11-14 收尾）

| 编号 | 用途 | 详情 |
| --- | --- | --- |
| 120–146 | 11-07 及以前的闭环与测试（含六隧道 132–142、GICP 对照 145/146） | 0.10 |
| 147–157 | P 对照（147/148 配置无效）、F 组件测试（157） | 9.8 |
| 158–166 | H 停车采样（158 工具超时无效；161、162、165 LOST；164 夹具滑坡） | 9.8 |
| 167–177 | H2 候选 ctest、DDS 冒烟（169 无效）、停车 171/173、首目标 174–177 | 9.8 |
| 178–199 | H2 修复：179–182 启动期被工具超时杀死（各有 `INVALID.json`，不计结果）；183、199 2027 首目标失败；184–198 ctest 与竞态复现 | 9.8 |
| 200–221 | 11-06 S3 取证（0.9）；其中 206、209–219 在 11-11/12 又被重号使用：206 2025 首目标、209 首帧记录、210 loam ctest、211 无产物、212/213 锚定修复首目标、215/218/219 最终首目标 | 0.9、9.8 |
| 222–232 | 11-07 terrain 方案 A 闭环等；226 在 11-12 又用于 MPC ctest | 0.10、9.8 |
| 100 | 11-12 负对照：2027 无图 `tunnel_b_roundtrip`，门 0.255+0.05 原值 | 9.8 |
| 99 | 11-12 新门 0.249 + 顶板平面语义：2027 无图 `tunnel_b_roundtrip`，目标 3 终点未宣布到达后定位超时失败 | 9.8 |
| 98 | 11-12 同 d99 代码重跑：目标 1–3 通过，目标 4 在 0.096 m 停住看门狗耗尽失败 | 9.8 |
| 97 | 11-12 同代码 + 探针 5 Hz `localization_pose` 事件，重跑 `tunnel_b_roundtrip`：目标 1、2 通过，目标 3 看门狗耗尽（真值 0.058 m），`EXIT=1` | 9.8 |
| 96 | 11-13 同代码（当时 overlay 的 goal_manager 段实际未生效），重跑 `tunnel_b_roundtrip`：目标 1 在 sim 13 s 因 odom 层一块 unknown 洞挡住路线耗尽挂起预算而失败（真值 1.05 m），未测到目标 3 | 9.8 |
| 95 | 11-13 同 d96 代码重跑：sim 3.7 s 启动期 `RELOCALIZING` 超 `observation_lost_timeout_s 10`（墙钟）变 `LOST`，runner 中止，未进入导航 | 9.8 |
| 94 | 11-13 同 d96 代码，`lidar_threads:=24`（LiDAR 追踪 297 ms/帧，8 线程为 512–610 ms/帧）：目标 1 在 sim 12 s 同样挂起预算耗尽（真值 0.89 m） | 9.8 |
| 93 | 11-13 同代码（overlay 的 `ats_goal_manager` 段实际未生效，已删），重跑 `tunnel_b_roundtrip`：目标 1–3 通过（0.035/0.0115/0.0259 m），目标 4 在 0.082–0.094 m、yaw 误差 1.46 rad 处 MPC 指令恒为 0，看门狗耗尽（真值 0.1007 m），`EXIT=1` | 9.8 |
| 73 | 11-14 同源码tunnel_B前6点到达，第6段记录器JSON截断，退出1 | `log/nav_acceptance/20261114_stage/tunnel_b_d73*` |
| 72 | 11-14 terrain实际节点车高范围悬挂物检查；已分配 | `log/nav_acceptance/20261114_stage/terrain_height_d72*` |
| 71 | 11-14 记录器SIGINT写中断旧实现回归；已分配 | `log/nav_acceptance/20261114_stage/recorder_json_red*` |
| 70 | 11-14 terrain节点正确名称加载参数后重测；已分配 | `log/nav_acceptance/20261114_stage/terrain_height_d70*` |
| 69 | 11-14 记录器信号收尾完整ROS节点回归；已分配 | `log/nav_acceptance/20261114_stage/recorder_json_green*` |
| 68 | 11-14 记录器完整回归，补现行runner fixture grid-frame；已分配 | `log/nav_acceptance/20261114_stage/recorder_json_d68*` |
| 67 | 11-14 记录器信号修复后tunnel_B原路线重跑；已分配 | `log/nav_acceptance/20261114_stage/tunnel_b_d67*` |
| 66 | 11-14 odom层采样边界修复Release构建/ROG包测试；已分配 | `log/nav_acceptance/20261114_stage/layer_boundary_*` |
| 65 | 11-14 odom层边界与unknown聚合修复后tunnel_B；已分配 | `log/nav_acceptance/20261114_stage/tunnel_b_d65*` |
| 64 | 11-14 列支撑XY小区间修复Release构建与ROG包测试；宿主socket检查空闲后分配 | `log/nav_acceptance/20261114_stage/dense_support_*` |
| 63 | 11-14 局部支撑身份保护与拟合缓存Release构建/ROG测试；ss端口23150–23399空闲后分配 | `log/nav_acceptance/20261114_stage/local_support_*` |
| 62 | 11-14 支撑XY小区间、拟合缓存与自身支撑保护修复后tunnel_B；退出1，第7点goal occupied | `log/nav_acceptance/20261114_stage/tunnel_b_d62*` |
| 50 | 11-14 真实体素密度计数Release构建、ROG包测试与d62提交序列回放；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d50_manifest.json` |
| 49 | 11-14 真实体素密度修复后原无图tunnel_b_roundtrip，完整扫描/真值/执行参考取证；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d49_manifest.json` |
| 48 | 11-14 密度修复后P2 service_timeout完整故障与恢复零速时间线；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d48_manifest.json` |
| 47 | 11-14 d87f83b原tunnel_B重跑；LiDAR24线程和既有GICP明细日志，不改确认门；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d47_manifest.json` |
| 46 | 11-14 11-14 d87f83b P2 input_stale独立故障、完整恢复零速及旧reference失效取证；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d46_manifest.json` |
| 45 | 11-14 11-14 d87f83b P2真实unknown独立故障及恢复零速验收；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d45_manifest.json` |
| 44 | 11-14 11-14修复lio/unknown夹具模式后新launch复跑，保留严格数值全unknown与恢复零速门；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d44_manifest.json` |
| 43 | 11-14 11-14 d87f83b P2 unreachable独立分类与三层零速验收；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d43_manifest.json` |
| 42 | 11-14 11-14 H2实测XY顶板分片与全范围局部平面Release构建、175项预计包回归和d62组件复验；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d42_manifest.json` |
| 41 | 11-14 11-14局部完整支撑平面与XY顶板分片175项回归后tunnel_B新launch闭环；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d41_manifest.json` |
| 40 | 11-14 顶板存在性限定的局部支撑平面及缓存撤回44项原生回归后Release构建；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d40_manifest.json` |
| 39 | 11-14 顶板存在性限定与移除缓存回退176项回归后tunnel_B新launch；24LiDAR线程；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d39_manifest.json` |
| 38 | 11-14 不连续低层回波不替换既有连接支撑的最窄修复Release及包测试；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d38_manifest.json` |
| 37 | 11-14 连接支撑身份不被不连续低层回波替换177项回归后tunnel_B新launch；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d37_manifest.json` |
| 36 | 11-14 上支撑面拟合与跨Z体素支撑保持Release和ROS包回归；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d36_manifest.json` |
| 35 | 11-14 上支撑面拟合与跨Z保留180项回归后tunnel_B完整新launch；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d35_manifest.json` |
| 34 | 11-14 S3 2025完整54点加20次障碍延迟与动态不保存取证；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d34_manifest.json` |
| 33 | 11-14 记录器回调积压导致旧位姿回放的先红ROS反例；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d33_manifest.json` |
| 32 | 11-14 记录器最新状态队列和完整快速载荷写入ROS回归；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d32_manifest.json` |
| 31 | 11-14 记录器连续状态最新值契约与原有摘要载荷离线回归；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d31_manifest.json` |
| 30 | 11-14 记录器积压修复后S3 2025完整54点及障碍动态保存复验；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d30_manifest.json` |
| 29 | 11-14 S3 2027完整路线与障碍动态保存；south和持续自旋同步只读取证；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d29_manifest.json` |
| 28 | 11-14 局部矛盾支撑隔离修复Release构建与ROG全包测试；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d28_manifest.json` |
| 27 | 11-14 支撑可用性改变同步通知积分层；最终Release与全包回归；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d27_manifest.json` |
| 26 | 11-14 局部矛盾支撑隔离最终源码2025完整54点及保存延迟动态复验；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d26_manifest.json` |
| 25 | 11-14 S3 2025 Point-LIO重启独立故障与身份急停恢复验证；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d25_manifest.json` |
| 24 | 11-14 S3 2025 Point-LIO重启故障有界LOST观察夹具复验；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d24_manifest.json` |
| 23 | 11-14 MPC终端原地yaw制动ROS反例构建与红测试；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d23_manifest.json` |
| 22 | 11-14 MPC终端yaw制动Release构建及全包测试；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d22_manifest.json` |
| 21 | 11-14 终端yaw制动修复后2025完整54点保存延迟动态闭环；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d21_manifest.json` |
| 20 | 11-14 两场仿真近障参考限速0.15构建配置回归及launch参数核对；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/d20_manifest.json` |
| 18 | 11-14 宿主端口12001占用，预检拒绝，未启动仿真 | `log/nav_acceptance/20261114_stage/d18_preflight_rejection.txt` |
| 19 | 11-14 端口预检拒绝后编排误启动；已中断自有session31125（130），证据作废，不复用 | `log/nav_acceptance/20261114_stage/d19_invalid_abort.json` |
| 17 | 11-14 2025完整54点尝试：37点到达，第37段q2持续冲突退出1 | `log/nav_acceptance/20261114_stage/d17_result_summary.json` |
| 15 | 11-14 隔离Nav2严格控制器0.05变体通过，真值0.049105m；不替代原默认历史基线 | `log/nav_acceptance/20261114_stage/nav2_d15/summary.json` |
| 14 | 11-14 2027完整64点尝试：前7点真值通过，第8点0.194165m超差，退出1；8段q1/q2过 | `log/nav_acceptance/20261114_stage/s3_2027_d14/summary.json` |
| 13 | 11-14 2027 point_lio_restart：注入前motion/EXECUTE等待超时，退出1，kill/restart/replay均0 | `log/nav_acceptance/20261114_stage/s3_2027_restart_d13/summary.json` |
| 51 | 11-14 当前4311d73 adapter_lease，lio/WLAN/prior，新增完整时间线与取消后不复活门；宿主socket空闲后分配 | `log/nav_acceptance/20261114_stage/adapter_lease_d51_*` |
| 52 | 11-14 当前4311d73的2025红框lio/WLAN/prior，退出0，真值0.027305m，q1/q2通过 | `log/nav_acceptance/20261114_stage/redbox_d52_*` |
| 53 | 11-14 当前ATS默认lio/2025 prior/WLAN；退出0，真值0.052991m，q1/q2通过 | `log/nav_acceptance/20261114_stage/default_d53_*` |
| 54 | 11-14 撤回未提交的支撑约束候选回归，重建已提交4311d73并跑ROG包测试；173项0失败0跳过 | `log/nav_acceptance/20261114_stage/restore_4311_*` |
| 55 | 11-14 支撑局部XY约束修复后原tunnel_B闭环；首目标等待重规划超时，退出1，真值1.367121m | `log/nav_acceptance/20261114_stage/tunnel_b_d55/` |
| 56 | 11-14 ROG支撑局部XY约束Release构建与包测试；174项0失败0跳过 | `log/nav_acceptance/20261114_stage/support_patch_*` |
| 57 | 11-14 补齐历史base_footprint到base_link TF后Nav2复跑；旧协议基线通过，当前0.08m真值门失败(0.167386m) | `log/nav_acceptance/20261114_stage/nav2_d57/` |
| 58 | 11-14 修正兼容runner等待全部Nav2 lifecycle active后复跑；退出1，SUCCEEDED但真值0.171670m，兼容TF缺base_link | `log/nav_acceptance/20261114_stage/nav2_d58/` |
| 59 | 11-14 隔离历史RMUC2026 truth/Nav2，原场景与起终点；退出1，兼容runner在lifecycle active前过早发目标被拒绝 | `log/nav_acceptance/20261114_stage/nav2_d59/` |
| 60 | 11-14 隔离历史Nav2 NavigateToPose + 当前lio/WLAN，2025误套历史负坐标起点导致场外下落；目标未发，人工中止130 | `log/nav_acceptance/20261114_stage/nav2_d60/` |
| 61 | 11-14 Nav2历史兼容夹具独立Release构建；ss端口22650–22899空闲后分配 | `log/nav_acceptance/20261114_stage/nav2_compat_build*` |
| 74 | 11-14 H2修复后tunnel_B退出1；首点0.0211m通过，第二点unknown足迹触发安全停车；两段q1/q2通过 | `log/nav_acceptance/20261114_stage/tunnel_b_d74*` |
| 75 | 11-14 H2坡道固定补采退出0，60有效帧；近域无支撑种子，生产无净空测量，非H2门通过 | `log/nav_acceptance/20261114_stage/ramp_fixed_d75*` |
| 76 | 11-14 H2两包Release构建通过；ROG167项零失败，terrain功能通过、包级既有lint失败 | `log/nav_acceptance/20261114_stage/h2_packages_*` |
| 78 | 11-14 固定位姿补采超时退出1，0/60稳定帧；CONFIRMED/DEGRADED重置计时的夹具缺陷，启动LOST=0 | `log/nav_acceptance/20261114_stage/ramp_fixed_d78*` |
| 79 | 11-14 H2 world-weld位姿稳定，首次GICP not converged后1/2确认中LOST，退出1、0稳定帧 | `log/nav_acceptance/20261114_stage/ramp_fixed_d79*` |
| 80 | 11-14 H2 ramp_bridge X-lock补采失败，12.14 s漂移0.050017 m，0稳定帧 | `log/nav_acceptance/20261114_stage/ramp_park_d80*` |
| 81 | 11-14 运动时钟/终端yaw修复后 tunnel_B；前6点通过，第7点目标occupied，无参考预算退出1，未带RViz | `log/nav_acceptance/20261114_stage/tunnel_b_d81*` |
| 82 | 11-14 MPC稠密末段投影构建/全包测试，126项0失败 | `log/nav_acceptance/20261114_stage/tracker_dense_mpc_*` |
| 83 | 11-14 MPC终端投影初版构建/全包测试，124项0失败；稠密轨迹尚有缺口 | `log/nav_acceptance/20261114_stage/tracker_mpc_*` |
| 84 | 11-14 运动时钟修复构建/Goal Manager全包测试，90项0失败 | `log/nav_acceptance/20261114_stage/progress_*` |
| 85 | 11-14 tunnel_B复跑，首目标真值0.02190 m通过；第二目标终端投影停滞，指定action取消，诊断中止退出1，未带RViz | `log/nav_acceptance/20261114_stage/tunnel_b_d85` |
| 86 | 11-14 P 无图 tunnel_b_roundtrip，lio；退出 1，首目标 watchdog 耗尽，真值 0.052743 m，不计通过 | `log/nav_acceptance/20261114_stage/tunnel_b_d86` |
| 87 | 11-14 P 2025 红框，lio，未带 RViz；退出 0，终点误差 0.064133 m，q1/q2 通过 | `log/nav_acceptance/20261114_stage/redbox_d87*` |
| 88 | 11-14 MPC Release 重建与全包测试 | `log/nav_acceptance/20261114_stage/mpc_*` |
| 89 | 11-14 P 2025 红框；启动前 stale MPC binary 拦截，退出 3，无仿真 | `log/nav_acceptance/20261114_stage/redbox*` |
| 90 | 11-14 P Release 构建与 Goal Manager 全套 CTest（含 frozen-clock）；13/13 通过 | `log/nav_acceptance/20261114_stage/`；WLAN 可用，宿主网络 socket 表无对应端口；PID namespace 受限，不能由 `/proc` 声称宿主无进程 |
| 91 | 11-13 STOP/reference 节点回归尝试；DDS 创建失败，未进入测试行为 | `20261113_stop_guard/node_before.log`、`ctest_all.log`；受限环境禁止 UDP socket |
| 92 | 11-13 WLAN DDS 初始化检查：wlo1、组播、localhost-only=0；未启动仿真 | `log/nav_acceptance/dds/wlan_probe_d92.log`；网络接口枚举受沙箱权限阻塞，未建立节点 |

120–232 已基本用尽并有重号；100–93 已用于上述运行，91 已尝试测试。92 已尝试 WLAN DDS 初始化，未启动闭环；下一次必须在宿主机重新查占用（当前沙箱 `/proc` 只能看到自身命名空间，不能证明宿主空闲）。禁用 5、77。

## 2026-11-16 profile/速度/雷达专项

宿主WLAN实际为`wlx500ff5500d30`（192.168.11.116），已以只读`ip/ss`核验；旧`wlo1`不存在。DDS只写工作区配置，保持`ROS_LOCALHOST_ONLY=0`，不使用lo。新编号在启动前再次核验端口。

| 编号 | 用途 | 状态 |
| --- | --- | --- |
| 101 | MINCO/MPC相关Release窄回归 | 已分配；运行状态见`20261116_profile_sensor_update.json` |
| 102 | MID360 10Hz、旋转与5m/s链仿真冒烟 | 已分配；运行状态见`20261116_profile_sensor_update.json` |
| 103 | 2027 S3新配置短路线检查 | 已分配；运行状态见`20261116_profile_sensor_update.json` |
| 104 | 空场LIO诊断补齐夹具IMU后重跑 | 已分配；102因旧空场模型无IMU在初始化失败，不计行为通过 |
| 105 | 空场传感器/速度链诊断 | 已分配；104为夹具mesh目录检查失败，未启动传感器；保留失败产物 |

- 2026-11-16 domain 103：2027 单目标 LIO 回归在首批 odom 前进入 LOST，目标执行 0；保留启动失败，不计导航通过。
- 2026-11-16 domain 106：宿主端口核验后用于 2027 单目标重跑；显式延后导航启动至 70 s、ROG 至 65 s，并恢复自动 LiDAR 预算；不更改定位超时/质量门。
- 2026-11-16 domain 106：安装文件一致性预检拒绝，未启动 DDS/仿真；重建 ats_mujoco_sim 后保留此尝试号。
- 2026-11-16 domain 107：2027 单目标 LIO 重跑，自动雷达预算、导航/ROG 延迟 70/65 s；与默认启动分列。
- 2026-11-16 domain 107：延后导航后仍在首批有效身份化 odom 前 LOST，目标执行 0，不计导航通过。
- 2026-11-16 domain 108：宿主 DDS 端口核验空闲；最后行为修改后的传感器/5 m/s 专项，底盘固定雷达、自动 CPU 预算，未带 RViz。
- 2026-11-16 domain 105/108：传感器/执行器专项通过；d108为最后行为版本，169帧/3400 IMU，10 Hz/200 Hz、指令5 m/s；无规划器，未带RViz，不计导航/物理接触验收。

## 2026-11-17 本机 O1-A / O0

本机 `ats-OMEN-by-HP-Gaming-Laptop-16-wf0xxx`、工作区 `/home/ats/ATS_2026_snetry_test`、WLAN `wlo1`；与11月16日 `/home/kongliu/文档/27` 主机分开记录。用户11月17日确认在本机实施。每次运行前核验宿主 UDP 端口，`ROS_LOCALHOST_ONLY=0`。

- domain 109：Release 基线定位身份/MPC窄测试；端口核验见 `log/nav_acceptance/20261117_o1a_o0/domain109_preflight.json`。
- domain 110：本机O1-A首次首目标复现，默认runner延迟、自动雷达预算、lio/WLAN、未带RViz；输出 `log/nav_acceptance/20261117_o1a_o0/baseline_d110`。
- domain 111：O1-A首帧超时/跨topic配对回归的旧实现负对照；日志 `log/nav_acceptance/20261117_o1a_o0/startup_red_*`。
- domain 112：O1-A修复后fusion全部直接节点/冷启动/身份及core窄回归；独立WLAN domain，结果见本轮索引。
- domain 113：初始超时分类/runner跨topic配对修复后的2027首目标，原始延迟、自动雷达预算、lio/WLAN；独立速度链与身份bag，输出 `log/nav_acceptance/20261117_o1a_o0/startup_fix_d113`。
- domain 114：本机lio/WLAN独立运行 `template_fix_d114`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/template_fix_d114*`。
- domain 115：本机lio/WLAN独立运行 `candidate_diag_d115`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/candidate_diag_d115*`。
- domain 116：矩形覆盖修复后 MINCO/Goal Manager/MPC 窄 CTest，WLAN 独立 domain。
- domain 117：本机lio/WLAN独立运行 `footprint_fix_d117`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/footprint_fix_d117*`。
- domain 118：O0 慢仿真时钟控制周期负对照与冻结时钟安全门，旧实现红测试。
- domain 119：O0 时基修复最终窄 CTest，慢时钟/冻结租约/回拨旧参考及原有安全换轨门，独立WLAN。
- domain 12：本机lio/WLAN独立运行 `final_first_goal_d12`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/final_first_goal_d12*`。
- domain 11：同 epoch DEGRADED 续证确认的旧实现负对照，WLAN 独立节点窄测试。
- domain 10：续证修复后 small_gicp 全部功能窄测试，含确认期限、身份、地图提交、运行期失效门。
- domain 9：本机lio/WLAN独立运行 `final_first_goal_d9`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/final_first_goal_d9*`。
- domain 8：本机lio/WLAN独立运行 `tunnel_b_d8`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/tunnel_b_d8*`。
- domain 7：2027 lio/WLAN 独立 `v1_d7`，P2=none；完整命令/锚点/结果见本轮索引。
- domain 6：2027 lio/WLAN 独立 `adapter_lease_d6`，P2=adapter_lease；完整命令/锚点/结果见本轮索引。
- domain 4：2027 lio/WLAN 独立 `adapter_lease_d4`，P2=adapter_lease；完整命令/锚点/结果见本轮索引。
- domain 3：2027 lio/WLAN 独立 `service_timeout_d3`，P2=service_timeout；完整命令/锚点/结果见本轮索引。
- domain 2：2027 lio/WLAN 独立 `input_stale_d2`，P2=input_stale；完整命令/锚点/结果见本轮索引。
- domain 1：2027 lio/WLAN 独立 `unknown_d1`，P2=unknown；完整命令/锚点/结果见本轮索引。
- domain 0：2027 lio/WLAN 独立 `unreachable_d0`，P2=unreachable；完整命令/锚点/结果见本轮索引。

11月17日逐号核验发现16号没有历史分配记录，不能推断编号已耗尽。端口预检后将其保留给正常编排的原首目标补验，实际分配条目在下方追加。独立F四项通过、unreachable严格失败；d6前置失败未注入，结果与证据见`20261117_o1a_o0.md/json`。
- domain 16：本机lio/WLAN独立运行 `final_first_goal_d16`，命令与结果见 `log/nav_acceptance/20261117_o1a_o0/final_first_goal_d16*`。

## 2026-11-17 本轮获授权的历史编号复用

用户明确允许本轮 O1/O1b/O0 复用历史编号，按[清场规则](20261117_domain_reuse_rules.md)执行；本节不是首次分配记录，不覆盖上方历史。排除 5、77、91、92，WLAN 为本机 `wlo1`。

| 复用编号 | 独立批次 | 结果与清场 |
| --- | --- | --- |
| 20 | `20261117_o1_o0_completion/unit_d20` | Goal Manager 13 组通过；guard command_exit=0，进程/端口清空 |
| 21 | `20261117_o1_o0_completion/unit_d21` | 记录器、MINCO/ROG/RC-ESDF/MPC 窄回归与 launch 参数检查通过；command_exit=0，清空 |
| 22 | `20261117_o1_o0_completion/v1_d22` | V1 action 定位恢复超时，q1 通过、q2 失败、O0 未触发；command_exit=1，清空 |

完整命令、PID/kernel start time、运行前检查和运行后检查分别保存在每个批次的 `domain_guard.json`，不把本机清场推导为其他 WLAN 主机参与者已排除。
