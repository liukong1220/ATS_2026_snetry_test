# Gazebo red_box domain201 + 实车 BT 比赛部署门槛

日期：2026-09-22  
工作区：`/home/kong/ATS_2026_snetry_test`  
运行：`TEST_PROFILE=red_box` `ROS_DOMAIN_ID=201` `PLANNING_GRID_OWNER=rog_map` `NAV_TRACKING_GATE=1`  
产物：`log/gazebo_minco_mpc_chain/20260922_092806_red_box_none_domain201/`、`/tmp/ats_gazebo_red_box_201_driver.log`

## 结论摘要

- **red_box 未通过**（已验证失败，非超时误判）。
- 腿1 `south_approach` **SUCCEEDED**（误差 `0.361 m` ≤ Gazebo `0.50 m`）。
- 腿1 成功后机器人 **xy 永久冻结**在 `(3.851, -4.207)`；后续腿/stitch 全部 timeout，仅 yaw 变化。
- 因此 **不能** 用本轮 red_box 证明「多航点比赛可闭环」；nominal domain200 仍证明单目标链可用。
- 实车比赛「行为树快速发任务」：**接口路径已具备**（BT → `/ats_navigate_to_pose` → Goal Manager），但默认实车门控仍偏多；可按比赛剖面关掉非必要门，**不可**关掉地图/定位失效急停。

## red_box201 腿结果

| 腿 | 目标 | 结果 | 终姿/误差 |
|----|------|------|-----------|
| 1 south_approach | (4.20,-4.30) | SUCCEEDED | (3.851,-4.207) / 0.361 m |
| 2 south_entry | (4.40,-5.90) | FAIL timeout | **同 xy** / 1.780 m |
| stitch south_dip | (3.851,-6.35) | FAIL timeout | xy 未动 |
| 3 west_corridor_east | (5.20,-6.20) | FAIL | **同 xy** / 2.407 m |
| stitch mouth_seat | (5.10,-6.28) | 运行中被中止 | — |
| 4–10 | … | 未继续有意义验证 | 中止：腿1后冻结已充分 |

启动参数已生效：`height=[0.20,0.80]`（domain200 修复保留）。

## 冻结证据（已验证）

`action_result` 终姿 xy 四次相同：

```text
goal=1 SUCCEEDED final_pose=(3.851,-4.207,-2.022)
goal=2 timeout    final_pose=(3.851,-4.207,2.597)
goal=3 timeout    final_pose=(3.851,-4.207,1.357)
goal=4 timeout    final_pose=(3.851,-4.207,2.520)
```

腿1 期间有 MPC 饱和与 `emit mode=1`/`auto_authorized=1`（能走）。腿1 后仍有规划输出（path length≈1.7–2.4 m），但 xy 不前进 → **执行/底盘或后继 EXECUTE 未形成平移**，不是「目标 occupied」类失败。

中止理由：继续 180s×剩余腿不会增加新信息；runner 未写出完整 `runner_status.env`（脚本被 SIGINT 后 Gazebo 曾孤儿化，已清理）。

## 推断

- 冻结更像「首目标成功后控制/授权/底盘平移丢失」，而非 ROG inflation=0 单独导致。`[Confidence: Medium]`（终姿冻结 + 仍有规划；缺腿2窗口 cmd_vel 数值袋）。
- `footprint=0` 曾出现但全日志仅约 4 次，不足以单独解释 180s 静止。`[Confidence: Medium]`。

## 实车 BT 比赛部署门槛（对照代码，未跑 HIL）

### 已具备的发任务路径

```text
referee/mission → ats_sentry_behavior → /ats_navigate_to_pose
  → ats_goal_manager → /ats_goal_manager/planner_goal → MINCO → MPC → arbiter → serial
```

- BT 配置：`decision_config.ats_action_server: /ats_navigate_to_pose`（`node_params.yaml`）。
- 根入口：`bringup.launch.py` 默认 `launch_behavior:=True`，实车走 `rm_navigation_reality_launch.py`。
- 行为树 **不拥有** 地图/速度；多航点按序调 ATS action（cancel/preempt/timeout 语义在 GM）。

### 默认实车门控（偏多，比赛可裁）

| 门控 | 默认 | 比赛建议 |
|------|------|----------|
| BT referee `start_gate`（`game_progress==4`） | 有 | **保留**作开赛闸；调试可改 simulation input |
| `require_gimbal_status` | true | 无可靠云台 ACK 时 launch 置 **false**（否则任务进不了执行） |
| `require_localization_status` | true | **保留**（LOST 时 fail-closed） |
| `require_map_status` / snapshot / map_wait 30s | true | **保留**最短地图健康；不要为「快发」关掉 ready=false→急停 |
| serial `require_execution_authorization` | false | 已偏比赛友好 |
| DualMap / emergency_stop | 有 | **保留** |

### 比赛最小可用剖面（部署清单，未 HIL）

1. `ros2 launch ats_sentry_bringup bringup.launch.py`（或现用实车入口）+ 地图/先验 PCD 就绪。
2. 确认 `/ats_navigate_to_pose` server、BT client `target_tree` 指向比赛树。
3. 裁判开赛后 BT 自动 `SendGoal`；或临时用 `ros2 action send_goal` 验证单点。
4. 不要依赖 Gazebo red_box 多腿作为实车放行条件；以 **单点到达 + 连续第二次 SendGoal 仍能平移** 为最低实车冒烟。

## 未覆盖

- red_box 全 10 腿成功；腿1后冻结根因修复与复跑。
- 实机 HIL；裁判联动实测。
- MuJoCo red_box 本轮未跑。
