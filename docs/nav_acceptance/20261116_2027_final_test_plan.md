# 2027 导航验证方案：无先验基地到基地、隧道与对称点速度

2026-11-19 文档更新将无先验基地到基地设为下一次实际导航的主场景。目标由场地几何和起始位姿计算，沿途在线建图和重规划。T1/T2/T3、V1、F 保留各自的专项判定，按实际要验证的问题选用。2025 和 2027 红框归档。11 月 17 日及后续运行来自另一验证主机。 [验证主机说明](20261117_o1a_o0.md) · [证据索引](20261117_o1a_o0.json) · [续接账本](20261117_o1_o0_completion.md)

本文件维护任务坐标、运行入口和验收结果。新主场景的远目标接续和洞口正向对齐处于设计阶段。d34、d35、d40 的局部通过和其余历史失败保留原判定及来源边界。 [实施顺序](../调研/无先验地图钻洞高速导航优化调研报告.md#51-执行顺序与当前停点) · [实施入口](../调研/无先验地图钻洞高速导航优化调研报告.md#97-提示词选择)

## 1. 固定场景与坐标

| 项 | 当前约定 | 来源/限制 |
| --- | --- | --- |
| 场景 | `rmuc_2027`，`localization_source:=lio` | 不用truth定位替代Point-LIO |
| 实际模型范围 | $x\in[0,28]$、$y\in[0,16]$ m | MJCF注释与560×320、0.05m的PGM交叉核对；不套用28×15m比赛概念尺寸 |
| 场地中心 | `map (14, 8)` | 场地原点 `(0, 0)` 加场地长宽的一半 |
| 无先验基地起点 | `(3.37, 9.03, 0.465, 0)` | `rmuc_2027`、`rmuc_2027_tunnels` 的显式起点，最后两项为高度和 yaw |
| 无先验对面基地对应点 | `map (24.63, 6.97)` | XY 位移 `(21.26, −2.06) m`，直线距离 `21.36 m` |
| V1 速度起点 | `(13.40, 2.00, 0.550, 0)` | 当前 2027 launch 默认起点 |
| V1 速度目标 | `map (14.60, 14.00)` | 原对称点专项，末端 yaw 为 `0` |

场地关于中心对称且 `map` XY 与场地轴对齐时，设场地原点为 $(x_0,y_0)$、尺寸为 $(W,H)$、起始 XY 为 $S=(x_s,y_s)$。对面对应点和直线距离按下式计算。

$$
C=(x_0+W/2,\,y_0+H/2),\qquad G=2C-S,\qquad D=\lVert G-S\rVert_2.
$$

2027 的场地尺寸由模型与 PGM 头交叉核对，基地起始位姿取自现有路线配置。无先验运行只输入场地几何和起始位姿，Point-LIO 与在线地图提供行进中的定位和障碍证据。场地尺寸、原点或出生点变化时重算目标。非中心对称场地使用实测对应点。实际里程按仿真真值轨迹累加，单独报告绕行增加的距离。 [基地起点](../../src/sim/ats_mujoco_sim/config/s3_mapping_routes.yaml#L80) · [地图分辨率与原点](../../src/ats_sentry_bringup/map/rmuc_2027.yaml)

文档预检：速度两端在当前PGM上，yaw=0、`0.581×0.571 m + 每边0.02 m`的保守覆盖单元均为free，中心单元所在原始free连通域相同。圆形膨胀连通诊断没有通过；它含圆形/栅格取整近似，不能单独证明矩形SE2有路或无路。完整路径、转弯足迹、三维净空、落地、动态障碍与定位仍需实际导航验证，不由端点预检推导通过。原始计算及hash见[计划检查索引](20261116_2027_final_test_plan.json)。

## 2. 实际任务与专项矩阵

| ID | 用例 | 入口 | 必须报告 |
| --- | --- | --- | --- |
| 主场景 | 无先验基地到基地 | `rmuc_2027`，`lio`、`slam:=true`，单一 ATS action 到 `map (24.63, 6.97)` | 终点真值、实际里程、局部重规划、经过地形及静态地图保存 |
| T1 | 真实`tunnel_B`往返 | `rmuc_2027_tunnels / tunnel_b_roundtrip` | 所有13个目标、B正反向穿越次数、入口/出口yaw及横向误差、真值终点和执行足迹 |
| T2 | 南坡/南侧通道往返 | `rmuc_2027_tunnels / ramp_south` | 坡道支撑、顶板净空、正反向通过、失败点 |
| T3 | 六隧道完整路线 | `rmuc_2027_tunnels / six_tunnels` | 全部配置目标与每个声明通道的穿越计数；运行前声明重复次数N并逐次报告 |
| V1 | 初始点到中心对称点导航速度 | `SCENE=rmuc_2027 TEST_PROFILE=single` | `(13.40,2.00)→(14.60,14.00)`，参考/指令/实际速度、终点真值及停车 |
| F | 2027独立安全故障 | 同一2027基线，逐项新domain/launch | ready失效→急停→控制零速→底盘零速；恢复后旧参考不复活 |

T1中的第7个前置`ramp_bridge`不算B通过；真正B位于第10–13个目标。不得使用`--max-goals`片段替代完整T1/T2/T3。现有路线和几何评估输入只给评估器，不回流成导航真值。

本方案没有给定重复稳定性门的N；运行前记录所选N和依据，不能事后按成功次数选N。探索阶段N=1只代表单次结果，不能宣称重复稳定性。O1-A允许的原64点路线首目标截断仅作启动诊断，不计入T1/T2/T3完整路线通过。

主场景只按实际遇到的地形记录结果。需要单独判定指定通道时，再选择相应的 T1/T2/T3 完整路线。无先验用例使用显式建图模式，有先验复验使用经核验的同源 2027 PGM/PCD 或本场保存地图，通过 `--prior PREFIX` 单独启动。

## 3. 速度验收的含义

暂定平移上限5m/s，yaw上限3rad/s、角加速度2rad/s²；保留逐轮速度/加速度、曲率、近障、终点和失效零速门。V1使用导航action，不能直接给`/motion_control`发5m/s来代替导航测试。

逐段记录同一仿真时间下的参考速度、MPC出口、仲裁出口、`/motion_control`及独立真值平面速度；报告峰值、持续时间、速度曲线、加减速、路程、目标误差、停车时间，以及每次限速的来源。只有安全参考实际请求高速时，才能判定下游是否仍有固定2/1.5m/s截断；未触发高速段标“未触发”，不伪报5m/s导航能力已验证。

本对称点直线距离约12.060m。在从静止到静止、加减速度均为2m/s²的理想条件下，达到5m/s所需最短距离为12.5m，短于该距离的理论峰值约4.911m/s。实际绕路、转弯、近障和净空会改变可达速度。因此V1必须报告实测结果，不能要求每次强行到5m/s，也不能用路径受限为理由隐瞒软件额外限幅。完整5m/s平台段未触发时，该能力项继续未验证。

终点独立真值误差采用现有最终门≤0.08m；路径/执行足迹、q1/q2和物理接触证据分列。`single`脚本默认终点门为0.30m，V1必须显式传入`GOAL_TOLERANCE=0.08`。

## 4. 执行模板：先完成前置检查再运行

每次选择一个未使用且宿主端口空闲的0..232 domain，排除5、77、91/92和[账本](20261115_domain_ledger.md)已尝试编号。使用工作区WLAN配置，保持`ROS_LOCALHOST_ONLY=0`，不退回lo。先核验Release构建、安装一致性、2027资产和健康。d103/d107来自另一主机，本机d110复现初始无odom超时卡死，当前修复与首目标证据见11月17日索引；不能放宽定位/地图安全超时绕过。

截至 d16 收尾，账本和排除项已覆盖全部 `0..232` 编号。以下带 `--reuse-authorized` 的专项模板源自此前获授权的 O1/O1b/O0 批次。后续运行按本次实际授权分配 domain，并按工作区规则核验进程和端口。 [历史清场规则](20261117_domain_reuse_rules.md)

### 4.0 无先验基地到基地

1. 用 `rmuc_2027_mujoco.launch.py`，设置 `localization_source:=lio`、`slam:=true`、`start_x:=3.37 start_y:=9.03 start_z:=0.465 start_yaw:=0.0`。`map_yaml_file` 和 `prior_pcd_file` 按现有无先验 runner 指向本次输出目录下不存在的文件。保留 MID360 `10 Hz` 与 IMU `200 Hz` 口径。 [2027 launch](../../src/sim/ats_mujoco_sim/launch/rmuc_2027_mujoco.launch.py) · [无先验参数](../../scripts/s3_mujoco_acceptance.py#L748)
2. 向 `/ats_navigate_to_pose` 下发一个 `map` 目标 `(24.63, 6.97)`，`position_tolerance=0` 使用现行服务端默认。仅核验平移到达时可沿用终点 `yaw=0`。远目标在随车规划窗口外时，由后续实现选择窗口内可达局部目标，随着地图增长继续同一 action。局部规划输入来自在线扫描及地图，洞口先在洞外对齐车体 `+x`，随后正向通过。现有 `rmuc_2027` 多点 runner 保留原路线，单目标滚动验收在接通对应行为后执行。 [局部规划设计](../调研/无先验地图钻洞高速导航优化调研报告.md#无先验基地到基地与局部重规划)
3. 核对最终 action 状态、真值误差 `≤0.08 m`、实际里程与末端停车。报告实时绕障和地形引起的重规划、求解墙钟耗时、已执行参考与地图身份。遇到窄通道时记录入口前的实际 yaw、通过方向及整车驶出，未遇到则记未触发。保存 PGM/YAML/PCD 并记录观测范围，沿用现有足迹和执行判定。

有先验时加载建图产物并使用 `slam:=false`。现有全局路径提供引导，局部规划根据实时地图绕过受阻段并接回，独立报告本次选定任务的实际结果。

### 4.1 隧道

```bash
cd "${ATS_WORKSPACE:?先设置并核验实际总工作区路径}"
source install/setup.bash
source log/nav_acceptance/dds/wlan_env.sh
python3 -s scripts/nav_domain_guard.py run \
  --domain "${ATS_DOMAIN:?先核验本轮复用domain}" \
  --run-dir "${ATS_GUARD_OUT:?独立guard批次目录}" --reuse-authorized -- \
  python3 -s scripts/s3_mujoco_acceptance.py \
  --profile rmuc_2027_tunnels --route tunnel_b_roundtrip \
  --case route --output "${ATS_OUT:?全新的证据目录}" \
  --waypoint-control-tolerance-m 0.08 \
  --record-column-inputs --registration-details \
  --launch-arg use_rviz:=true --launch-arg show_viewer:=true \
  --launch-arg lidar_threads:=0 --launch-arg lidar_downsample:=0
```

T2/T3分别改为`ramp_south`和`six_tunnels`，每次另起domain和证据目录。无先验入口不加载PGM/PCD；有先验入口添加已核验的`--prior "${ATS_2027_PRIOR_PREFIX:?同源地图前缀}"`，不要编造不存在的路线或CLI选项。

无先验建图显示和保存使用 `--profile rmuc_2027 --case route --max-goals 1`，省略 `--prior` 与 `--reload`，runner 会启动 `slam:=true`。两窗口参数与上方相同。RViz 的「距离场」组显示 `/rog_map_adapter/viz/esdf_cloud`，「局部规划窗口」组可打开 `/rog_map/odom_layer`。正常完成后调用 `/rog_map/save_map`，在输出目录生成 `map.pgm`、`map.yaml`、`map.pcd`。 [首段建图与保存过程](20261117_o1_o0_completion.md#无先验建图在线-esdf-与地图保存)

本轮中间目标显式请求0.08m的控制/准入半径，最后目标及重载首目标仍请求0、使用服务器原默认；独立真值验收仍为中间≤0.15m、最后/重载≤0.08m。该请求随同目标重规划传递，不改坐标或unknown/footprint规则。为复现历史默认收敛契约，可省略`--waypoint-control-tolerance-m`或传0。d29已验证非法/越界拒绝、默认不变及抢占后不串用。

### 4.2 对称点速度

```bash
cd "${ATS_WORKSPACE:?先设置并核验实际总工作区路径}"
source install/setup.bash
source log/nav_acceptance/dds/wlan_env.sh
SCENE=rmuc_2027 TEST_PROFILE=single \
  START_X=13.40 START_Y=2.00 START_Z=0.550 START_YAW=0.0 \
  GOAL_X=14.60 GOAL_Y=14.00 GOAL_YAW=0.0 P3_GOAL_FRAME=map \
  GOAL_TOLERANCE=0.08 GOAL_TIMEOUT=300 GOAL_WALL_PER_SIM=20 \
  PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  LIDAR_DOWNSAMPLE=0 NAV_TRACKING_MAX_SNAPSHOT_PAYLOADS=-1 \
  NAV_TRACKING_MAX_LAYER_PAYLOADS=-1 O0_CHAIN_RECORDER=1 \
  NAV_TRACKING_PROFILE_YAML="$PWD/src/ats_sentry_bringup/params/node_params.yaml" \
  USE_RVIZ=true USE_MUJOCO_VIEWER=true \
  LOG_DIR="${ATS_OUT:?全新的runner日志目录}" \
  EXTRA_LAUNCH_ARGS="localization_source:=lio slam:=false lidar_threads:=0 lidar_rate_hz:=10.0 lidar_rate_clock:=sim prior_pcd_file:=$PWD/src/ats_sentry_bringup/pcd/rmuc_2027.pcd" \
  python3 -s scripts/nav_domain_guard.py run \
    --domain "${ATS_DOMAIN:?先核验本轮复用domain}" \
    --run-dir "${ATS_GUARD_OUT:?独立guard批次目录}" --reuse-authorized -- \
    bash scripts/test_mujoco_minco_mpc_chain.sh
```

命令显式指定`single`和2027场景，不依赖runner的默认2025值。记录器只从`NAV_TRACKING_PROFILE_YAML`取足迹长/宽/margin；三套已同步，因此这里用含完整字段的根参数，不能直接指向缺这些键的2027稀疏overlay。其余运行参数仍按正式2027覆盖链加载。

**本轮已修复记录完整性链。** runner分别传入两种载荷预算，默认`-1`全程保留，`0`关闭、正数显式限额仍记录缺证；分析器按LRU只缓存8份解压载荷。记录器保存完整SE3及几何/距离场身份，EXECUTE单独绑定最终验证快照与TF。d22的892个快照、1161个来源层载荷未截断，16条执行参考q1通过；q2仍失败。d7的历史失败和缺证保留，不回写为通过。

`O0_CHAIN_RECORDER=1` 由 runner 在目标下发前自动启动。初始证据缺失记入记录状态，导航仍可下发。输出位于该目标跟踪目录的 `o0_chain.jsonl`。分析时使用该次运行的控制周期、终点半径和里程计接收超时。

本机11月17日实测实时率约0.08–0.10，客户端墙钟兜底用20倍；32逻辑核默认6倍的历史速度假设不适用。该值只防止客户端早于ATS目标的300s仿真时间截止，不改变目标本身、odom/证据/地图租约或失效零速门。长任务使用独立后台进程并保存PID/退出码，避免执行工具约15分钟会话中断；不能把进程仍在运行当作验收成功。

若需要留存 ROS 原始消息，可在 runner 完成所有权检查后用同一 ROS、WLAN 及 domain 另录 bag：

```bash
ros2 bag record --use-sim-time -o "${ATS_BAG_OUT:?独立新目录}" \
  /clock /simulation/ground_truth/odometry /localization /odometry \
  /cmd_vel/autonomy_raw /cmd_vel/selected /motion_control \
  /minco/reference_path /planner/execution_command /planner/emergency_stop \
  /rog_map_adapter/planning_snapshot
```

用 `scripts/analyze_o0_handoffs.py` 分析 runner 自动保存的记录，按运行值传入 `--mpc-dt`、`--terminal-radius` 和 `--odometry-timeout`。保存对齐的速度 CSV、曲线与各窗分类。两个窗口用于现场观察，锁屏会延迟原生 Wayland 窗口出现，截图按实际结果记载。

### 4.3 故障及建图能力

P2故障用`adapter_lease/service_timeout/input_stale/unknown/unreachable`，在同一2027基线上每项新domain、新launch；替换V1的`P2_FAULT_CASE`，并先核对注入锚点/前置等待适配2027。不能仅凭`SCENE`可切换就声称每项已经通过。

11月17日F首次前置采用 `(13.40,2.00)→map (13.40,3.00,0)`、真值门0.08m；d6因真值0.159192m及q1/q2失败而未注入。后续独立F改用原路线起点 `(3.37,9.03,0.465,0)→map (4.00,9.03,0)`，经静态PGM足迹检查，prior资产/profile/场景不变，在线门继续保持；两个配置与结果分别保留，均不冒充V1。故障在飞目标从实际快照和估计 `/localization` 选取；frame不同时显式用一条父子frame一致且与odom/快照均≤0.5s差的完整TF转换，不把map真值标为odom。未到在飞运动门或未实际注入须记“未注入”。

S3的`point_lio_restart/unsafe/service_timeout/unknown/no_path`及地图修正仍按现有CLI，在2027显式profile下独立运行；与P2矩阵分列。保存质量、动态不保存、重载、实体障碍延迟及完整2027 A–K继续验证，不用隧道或速度单次结果替代。

## 5. 报告与当前状态

主场景和各专项按实际运行状态记录。保留 domain、三仓 commit、参数、原始日志、轨迹、q1/q2、接触评估及终点真值。基地到基地额外记录直线基准与实走里程，以及在线重规划和建图覆盖。未触发的地形只记录当次路线事实。

下表保留 11 月 17–19 日已归档的实际结果。 [验证主机说明](20261117_o1a_o0.md) · [冻结索引](20261117_o1a_o0.json) · [优化账本](20261117_o1_o0_completion.md)

| 项目 | 结果 | 结论边界 |
| --- | --- | --- |
| 无先验基地到基地 | **设计完成，未运行** | 已复算 `(3.37, 9.03) → (24.63, 6.97)` 的直线距离 `21.36 m`，远目标滚动推进和实际到达仍待实现与验证 |
| 原64点路线首目标d16 | **局部通过**：runner退出0、action成功、独立真值0.005460892m；3条执行参考q1和524有效位姿q2零冲突，无payload缺失 | 仅首点；小范围保存成功不计全场保存质量/重载。d9的143中断及原UUID成功接续证据保留，不作重复稳定性结论 |
| 原64点路线首目标 d40 | **建图首段与保存通过**。无 PGM/PCD 输入，真值误差 `0.004659 m`，2 条执行参考和 386 个实际位姿无冲突，清场完成 | RViz 规划 ESDF 实时更新；终点 PGM 为 `400×300`，PCD 为 3,227 点。MuJoCo 原生画面等待解锁。 [记录与地图](20261117_o1_o0_completion.md#无先验建图在线-esdf-与地图保存) |
| T1 d8 | **失败**：首目标unknown足迹/重规划预算阻断，0/13到达、真实B穿越0 | 该段q1/q2及失败零速通过，路线仍失败 |
| T1 d31（本轮） | **失败**：全路线尝试N=1，前6/13点到达，最大真值误差0.049200236m；第7点搜索/准入失败后有界预算code5，真值误差1.718364284m | 10条已执行参考q1、1475有效实际位姿q2零冲突，载荷/配对无缺口；第7点无执行参考，真实B未进入；接触计数增量0仍不是独立物理评估；清场完成 |
| T1 d36（本轮） | **失败**。首目标调用返回有界重试 code 5，真值误差 `1.281819 m`，真实 B 未进入 | 两条已提交参考及 166 个实际位姿无冲突。231 张线上快照的数值场与旧 EDT 回放一致。 [独立结果](../../log/nav_acceptance/20261117_o1_o0_completion/t1_incremental_edt_d36/run/summary.json) |
| T2 / T3 | **未执行** | T1前置未通过 |
| V1 d7 | **失败**：action成功、真值0.069451m达标；q1为7/9可评价参考冲突、另1条缺证，q2缺863 tick；运行期LOST后epoch1→2 | 真值峰值0.826715m/s；snapshot/layer截断及首次EXECUTE前位移待归因，未证明精确对称起点或高速能力 |
| V1 d22（本轮） | **失败**：action code 6定位恢复超时；16条执行参考q1通过；q2共30冲突tick，最长24 tick跨12快照，持续段2个 | 892快照与1161来源层未截断、无frame配对缺失；真值峰值0.449983m/s；接触计数增量0不替代独立物理评估 |
| V1 d25（本轮） | **提前终止、未通过**：重复里程计 steady 接收超时，绑定本次目标 UUID 取消，action code 1；18条执行参考q1通过；1388有效实际位姿中2个诊断冲突tick、无持续段 | 参考控制平面修复后独立运行；载荷/配对完整，接触计数增量0仍不是独立物理评估；O0为2次换轨、合格巡航0次，真值峰值0.439118m/s；清场完成 |
| V1 d34（本轮） | **路线和执行足迹通过**。action 成功，真值误差 `0.029575 m`，15 条执行参考、1642 个实际足迹样本均无冲突，退出码 0，清场完成 | 参考、MPC/仲裁/底盘、真值峰值分别 `2.148`、`1.068`、`0.892 m/s`。接触计数增量 0，独立接触评估未执行。 [速度曲线](../../log/nav_acceptance/20261117_o1_o0_completion/v1_tolerance_d34/o0/speeds.png) |
| V1 d37–d39（本轮） | d37 为切换窗口而中止。d38 双窗口已打开，旧 runner 要求可选 RViz 路径订阅，目标未发。d39 用简化 runner 运行，按用户要求切换无先验建图后中止 | 三轮均清场。d39 RViz 截图含车模和点云，MuJoCo 原生查看器在解锁后启动。 [RViz 截图](../../log/nav_acceptance/20261117_o1_o0_completion/v1_viewer_simple_d39/rviz.png) |
| F：adapter lease / service_timeout / input_stale / unknown | **各独立通过**：d4/d3/d2/d1，新锚点名义前置及三路零速、恢复后旧参考不复活获证 | 新锚点见4.3；d6前置失败未注入、非零接触计数保留。四项不能代表整个F通过 |
| F：unreachable d0 | **失败**：自由断连目标实际触发A*无路及零速，约3.02s仿真时返回挂起预算code 5，夹具预期code 3 | 结果码优先级与JPS/A*断言需先明确契约再修复；专用恢复门未执行 |
| F：unreachable d27（本轮） | **独立门通过**：夹具绑定同目标搜索及严格终止原因，三路零速、恢复后旧参考不复活 | 此后新增每目标容差和仿真CPU亲和性修改；最终行为版本仍须逐项重跑整个F，不沿用d27作最终矩阵 |
| F：unreachable d35（本轮） | **独立通过**。同一目标 A* 无路，有界等待 code 5，急停、三路零速、旧参考保持失效，退出码 0，清场完成 | 简化版 runner，含每目标容差和仿真 CPU 亲和性版本。其余四项仍需复验 |
| O0 / 高速能力 | **低速窗口达标**。d34 共 9 次换轨，1 次合格窗口中三路指令跌速最大 `0.040405 m/s`，真值无跌速 | 该事件前真值中位 `0.0327 m/s`，其余窗口含参考主动减速或健康变化。参考出现 `>2 m/s`，三路指令与真值均未达 `2 m/s`，`5 m/s` 和高速换轨仍待验 |

**O1-A全部门与本套件仍未通过。** 后续实施顺序由主报告5.1统一维护；该表只随新的实际运行证据更新，不由文档整理、端点预检或源码存在改变判定。

隧道计数器另修复portal与墙间通道端区无交集的问题：计数沿portal定义的方向，使用该轴与墙间corridor重叠区的两个端区，要求中间采样持续留在通道内；路线全部目标及真值/足迹门不变。六个真实配置的旧实现均把完整往返记为0次，修复后的6项相关回归通过，绕行/中途离开通道不计数。d31实际尚未进入任何声明通道，因此修复不改变该次失败结论。O1b三种可视化订阅开/关的数值及唯一owner集成回归在d32通过，不代替RViz画面观察。

历史2025 q2、2027 south和H2/M/H3的失败/未验证状态保留。2025默认/红框本轮未重跑；2025缺先验PCD属于历史兼容项，当前2027套件不以补齐2025资产作为前置条件。早期无窗口批次与d39/d40带RViz记录分列，MuJoCo原生窗口/现场解锁按批次实际记录。11月19日本次只有同步/审查，没有新窗口观察或仿真；物理接触仍未获独立验收。
