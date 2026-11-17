# 2027 最终验收：隧道通过与对称点速度测试

更新：2026-11-17。当前最终场景采用2027隧道和起点到中心对称点的速度测试。2025及2027红框仅作历史记录，不是本套件的执行入口或通过条件。11月16日制定计划；11月17日在 `/home/ats/ATS_2026_snetry_test` 实施O1-A/O0，实际结果见[本机证据说明](20261117_o1a_o0.md)和[索引](20261117_o1a_o0.json)。

算法进度和分模块提示词见[主报告第5节](../调研/无先验地图钻洞高速导航优化调研报告.md)。历史失败不因切换测试场景而消失；S3完整设计与建图保存/重载要求继续单独记账。

本文件拥有测试坐标、门禁、命令和实际结果；开发优先级见[主报告5.1](../调研/无先验地图钻洞高速导航优化调研报告.md#51-执行顺序与当前停点)，可复制任务见[9.7](../调研/无先验地图钻洞高速导航优化调研报告.md#97-提示词选择)。当前默认实施任务为O1-A回归收尾，启动/首目标局部证据保留；O1-B完整S3与O3新增算法另选。文档整理不启动运行，行为修改后按本方案复验；前置失败阻断项记未执行，局部通过不代表S3或本套件通过。正式入口须源码与graph共同确认无Nav2，不向未声明该开关的launch传`launch_nav2`。

## 1. 固定场景与坐标

| 项 | 当前约定 | 来源/限制 |
| --- | --- | --- |
| 场景 | `rmuc_2027`，`localization_source:=lio` | 不用truth定位替代Point-LIO |
| 实际模型范围 | $x\in[0,28]$、$y\in[0,16]$ m | MJCF注释与560×320、0.05m的PGM交叉核对；不套用28×15m比赛概念尺寸 |
| 对称中心 | $C=(14,8)$ m | $C=origin+(width,height)\times resolution/2$ |
| 速度起点 | $S=(13.40,2.00)$，`z=0.550`、`yaw=0` | 当前2027 launch默认值；运行前仍需检查落地与接触 |
| 速度目标 | $G=2C-S=(14.60,14.00)$ m | 目标坐标系`map`，终点yaw固定0以便比较平移速度 |
| 隧道路线起点 | `(3.37,9.03,0.465,0)` | `rmuc_2027_tunnels`显式路线起点；与速度试验分开，不混用 |

若另行指定实际出生点，按同一公式重新计算对称点并记录两者，不能保留旧目标而只改起点。当前坐标选择取自实际launch默认值；没有把隧道路线起点静默替换成速度起点。

文档预检：速度两端在当前PGM上，yaw=0、`0.581×0.571 m + 每边0.02 m`的保守覆盖单元均为free，中心单元所在原始free连通域相同。圆形膨胀连通诊断没有通过；它含圆形/栅格取整近似，不能单独证明矩形SE2有路或无路。完整路径、转弯足迹、三维净空、落地、动态障碍与定位仍需实际导航验证，不由端点预检推导通过。原始计算及hash见[计划检查索引](20261116_2027_final_test_plan.json)。

## 2. 当前最终测试矩阵

| ID | 用例 | 入口 | 必须报告 |
| --- | --- | --- | --- |
| T1 | 真实`tunnel_B`往返 | `rmuc_2027_tunnels / tunnel_b_roundtrip` | 所有13个目标、B正反向穿越次数、入口/出口yaw及横向误差、真值终点和执行足迹 |
| T2 | 南坡/南侧通道往返 | `rmuc_2027_tunnels / ramp_south` | 坡道支撑、顶板净空、正反向通过、失败点 |
| T3 | 六隧道完整路线 | `rmuc_2027_tunnels / six_tunnels` | 全部配置目标与每个声明通道的穿越计数；运行前声明重复次数N并逐次报告 |
| V1 | 初始点到中心对称点导航速度 | `SCENE=rmuc_2027 TEST_PROFILE=single` | `(13.40,2.00)→(14.60,14.00)`，参考/指令/实际速度、终点真值及停车 |
| F | 2027独立安全故障 | 同一2027基线，逐项新domain/launch | ready失效→急停→控制零速→底盘零速；恢复后旧参考不复活 |

T1中的第7个前置`ramp_bridge`不算B通过；真正B位于第10–13个目标。不得使用`--max-goals`片段替代完整T1/T2/T3。现有路线和几何评估输入只给评估器，不回流成导航真值。

本方案没有给定重复稳定性门的N；运行前记录所选N和依据，不能事后按成功次数选N。探索阶段N=1只代表单次结果，不能宣称重复稳定性。O1-A允许的原64点路线首目标截断仅作启动诊断，不计入T1/T2/T3完整路线通过。

T1→T2→T3先按显式无先验模式验证；有先验复验使用经核验的同源2027 PGM/PCD或本场保存地图，通过`--prior PREFIX`单独启动。两种模式分列结果，不能将无先验通过当作有先验通过。

## 3. 速度验收的含义

暂定平移上限5m/s，yaw上限3rad/s、角加速度2rad/s²；保留逐轮速度/加速度、曲率、近障、终点和失效零速门。V1使用导航action，不能直接给`/motion_control`发5m/s来代替导航测试。

逐段记录同一仿真时间下的参考速度、MPC出口、仲裁出口、`/motion_control`及独立真值平面速度；报告峰值、持续时间、速度曲线、加减速、路程、目标误差、停车时间，以及每次限速的来源。只有安全参考实际请求高速时，才能判定下游是否仍有固定2/1.5m/s截断；未触发高速段标“未触发”，不伪报5m/s导航能力已验证。

本对称点直线距离约12.060m。在从静止到静止、加减速度均为2m/s²的理想条件下，达到5m/s所需最短距离为12.5m，短于该距离的理论峰值约4.911m/s。实际绕路、转弯、近障和净空会改变可达速度。因此V1必须报告实测结果，不能要求每次强行到5m/s，也不能用路径受限为理由隐瞒软件额外限幅。完整5m/s平台段未触发时，该能力项继续未验证。

终点独立真值误差采用现有最终门≤0.08m；路径/执行足迹、q1/q2和物理接触证据分列。`single`脚本默认终点门为0.30m，V1必须显式传入`GOAL_TOLERANCE=0.08`。

## 4. 执行模板：先完成前置检查再运行

每次选择一个未使用且宿主端口空闲的0..232 domain，排除5、77、91/92和[账本](20261115_domain_ledger.md)已尝试编号。使用工作区WLAN配置，保持`ROS_LOCALHOST_ONLY=0`，不退回lo。先核验Release构建、安装一致性、2027资产和健康。d103/d107来自另一主机，本机d110复现初始无odom超时卡死，当前修复与首目标证据见11月17日索引；不能放宽定位/地图安全超时绕过。

截至d16收尾，账本与排除项已覆盖全部0..232编号。以下为条件满足后的模板；后续ROS运行须先明确满足AGENTS规则的隔离分配安排，端口空闲不等于可复用历史编号。未满足时可继续源码和已有证据审查。

### 4.1 隧道

```bash
cd /home/ats/ATS_2026_snetry_test
source install/setup.bash
source log/nav_acceptance/dds/wlan_env.sh
ROS_DOMAIN_ID="${ATS_DOMAIN:?先核验新domain}" \
  python3 scripts/s3_mujoco_acceptance.py \
  --profile rmuc_2027_tunnels --route tunnel_b_roundtrip \
  --case route --output "${ATS_OUT:?全新的证据目录}" \
  --record-column-inputs --registration-details \
  --launch-arg lidar_threads:=0 --launch-arg lidar_downsample:=0
```

T2/T3分别改为`ramp_south`和`six_tunnels`，每次另起domain和证据目录。无先验入口不加载PGM/PCD；有先验入口添加已核验的`--prior "${ATS_2027_PRIOR_PREFIX:?同源地图前缀}"`，不要编造不存在的路线或CLI选项。

### 4.2 对称点速度

```bash
cd /home/ats/ATS_2026_snetry_test
source install/setup.bash
source log/nav_acceptance/dds/wlan_env.sh
SCENE=rmuc_2027 TEST_PROFILE=single \
  START_X=13.40 START_Y=2.00 START_Z=0.550 START_YAW=0.0 \
  GOAL_X=14.60 GOAL_Y=14.00 GOAL_YAW=0.0 P3_GOAL_FRAME=map \
  GOAL_TOLERANCE=0.08 GOAL_TIMEOUT=300 GOAL_WALL_PER_SIM=20 \
  PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  LIDAR_DOWNSAMPLE=0 NAV_TRACKING_MAX_LAYER_PAYLOADS=3000 \
  NAV_TRACKING_PROFILE_YAML="$PWD/src/ats_sentry_bringup/params/node_params.yaml" \
  USE_RVIZ=false USE_MUJOCO_VIEWER=false \
  ROS_DOMAIN_ID="${ATS_DOMAIN:?先核验新domain}" \
  LOG_DIR="${ATS_OUT:?全新的runner日志目录}" \
  EXTRA_LAUNCH_ARGS="localization_source:=lio slam:=false lidar_threads:=0 lidar_rate_hz:=10.0 lidar_rate_clock:=sim prior_pcd_file:=$PWD/src/ats_sentry_bringup/pcd/rmuc_2027.pcd" \
  scripts/test_mujoco_minco_mpc_chain.sh
```

命令显式指定`single`和2027场景，不依赖runner的默认2025值。记录器只从`NAV_TRACKING_PROFILE_YAML`取足迹长/宽/margin；三套已同步，因此这里用含完整字段的根参数，不能直接指向缺这些键的2027稀疏overlay。其余运行参数仍按正式2027覆盖链加载。

**记录完整性前置尚待修复。** 当前runner只将`NAV_TRACKING_MAX_LAYER_PAYLOADS`传给`--max-layer-payloads`；`nav_tracking_recorder.py`的`--max-snapshot-payloads`仍用默认400。上面的3000只影响来源层，d7的snapshot与layer均出现截断、q2缺863 tick。下一阶段需分别设计两类载荷的保留预算/策略及完整性检查，再更新此模板；本文件不编造尚未接入的环境变量。保留V1的q1失败，补证后再判断其原因。

本机11月17日实测实时率约0.08–0.10，客户端墙钟兜底用20倍；32逻辑核默认6倍的历史速度假设不适用。该值只防止客户端早于ATS目标的300s仿真时间截止，不改变目标本身、odom/证据/地图租约或失效零速门。长任务使用独立后台进程并保存PID/退出码，避免执行工具约15分钟会话中断；不能把进程仍在运行当作验收成功。

先启动V1；在runner完成所有唯一所有权检查（日志出现`OK: /planner/emergency_stop has one publisher owned by ats_goal_manager`）后、下发目标前，用另一个已加载相同ROS、WLAN及domain的终端录制速度证据，输出目录不得已存在。只读记录器增加subscriber，若提前启动会破坏runner的前置精确计数；不得为此放宽所有权断言：

```bash
ros2 bag record --use-sim-time -o "${ATS_BAG_OUT:?独立新目录}" \
  /clock /simulation/ground_truth/odometry /localization /odometry \
  /cmd_vel/autonomy_raw /cmd_vel/selected /motion_control \
  /minco/reference_path /planner/execution_command /planner/emergency_stop \
  /rog_map_adapter/planning_snapshot
```

bag的domain须与V1相同；它是同一测试的只读记录器，不另起场景。结束后导出对齐的速度CSV/曲线与统计；11月17日新增只读`record_navigation_spin_chain.py`扩展及`analyze_o0_handoffs.py`，同样需在所有权检查后、目标前启动；按实际运行参数传入`--mpc-dt`、`--terminal-radius`与`--odometry-timeout`。缺原始曲线时不能把runner退出0写成速度验收通过。`/cmd_vel_mpc`等兼容话题如存在可追加，正式出口以实际graph为准。

### 4.3 故障及建图能力

P2故障用`adapter_lease/service_timeout/input_stale/unknown/unreachable`，在同一2027基线上每项新domain、新launch；替换V1的`P2_FAULT_CASE`，并先核对注入锚点/前置等待适配2027。不能仅凭`SCENE`可切换就声称每项已经通过。

11月17日F首次前置采用 `(13.40,2.00)→map (13.40,3.00,0)`、真值门0.08m；d6因真值0.159192m及q1/q2失败而未注入。后续独立F改用原路线起点 `(3.37,9.03,0.465,0)→map (4.00,9.03,0)`，经静态PGM足迹检查，prior资产/profile/场景不变，在线门继续保持；两个配置与结果分别保留，均不冒充V1。故障在飞目标从实际快照和估计 `/localization` 选取；frame不同时显式用一条父子frame一致且与odom/快照均≤0.5s差的完整TF转换，不把map真值标为odom。未到在飞运动门或未实际注入须记“未注入”。

S3的`point_lio_restart/unsafe/service_timeout/unknown/no_path`及地图修正仍按现有CLI，在2027显式profile下独立运行；与P2矩阵分列。保存质量、动态不保存、重载、实体障碍延迟及完整2027 A–K继续验证，不用隧道或速度单次结果替代。

## 5. 报告与当前状态

统一报告T1/T2/T3/V1/F每项：未运行、启动失败、执行失败、已通过及其限制。保留domain、三仓commit、实际参数/命令、原始日志与hash、每个目标结果、穿越计数、q1/q2、接触评估和终点/速度曲线。未带RViz时明确写明，不能由配置验证推导实际画面通过。

以下是11月17日运行结果，本次文档整理没有新增运行通过项；完整原始记录见[本机说明](20261117_o1a_o0.md)与[冻结索引](20261117_o1a_o0.json)。

| 项目 | 结果 | 结论边界 |
| --- | --- | --- |
| 原64点路线首目标d16 | **局部通过**：runner退出0、action成功、独立真值0.005460892m；3条执行参考q1和524有效位姿q2零冲突，无payload缺失 | 仅首点；小范围保存成功不计全场保存质量/重载。d9的143中断及原UUID成功接续证据保留，不作重复稳定性结论 |
| T1 d8 | **失败**：首目标unknown足迹/重规划预算阻断，0/13到达、真实B穿越0 | 该段q1/q2及失败零速通过，路线仍失败 |
| T2 / T3 | **未执行** | T1前置未通过 |
| V1 d7 | **失败**：action成功、真值0.069451m达标；q1为7/9可评价参考冲突、另1条缺证，q2缺863 tick；运行期LOST后epoch1→2 | 真值峰值0.826715m/s；snapshot/layer截断及首次EXECUTE前位移待归因，未证明精确对称起点或高速能力 |
| F：adapter lease / service_timeout / input_stale / unknown | **各独立通过**：d4/d3/d2/d1，新锚点名义前置及三路零速、恢复后旧参考不复活获证 | 新锚点见4.3；d6前置失败未注入、非零接触计数保留。四项不能代表整个F通过 |
| F：unreachable d0 | **失败**：自由断连目标实际触发A*无路及零速，约3.02s仿真时返回挂起预算code 5，夹具预期code 3 | 结果码优先级与JPS/A*断言需先明确契约再修复；专用恢复门未执行 |
| O0 / 5m/s能力 | **未触发**：d9/d16/V1合格巡航换轨为0；导航运行未触发5m/s平台 | 已修时基问题，原跌速门未获证；空场d108只验证传感器/执行器，不替代V1 |

**O1-A全部门与本套件仍未通过。** 后续实施顺序由主报告5.1统一维护；该表只随新的实际运行证据更新，不由文档整理、端点预检或源码存在改变判定。

历史2025 q2、2027 south和H2/M/H3的失败/未验证状态保留。2025默认/红框本轮未重跑；2025缺先验PCD属于历史兼容项，当前2027套件不以补齐2025资产作为前置条件。所有本轮运行未带RViz，物理接触仍未获独立验收。
