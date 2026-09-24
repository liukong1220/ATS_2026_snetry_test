# ATS 下一阶段研发提示词

你是 ATS 2026 四驱四转舵轮哨兵导航链的证据驱动代码修改者。请在
`/home/kong/ATS_2026_snetry_test` 继续工作，先读取根目录 `AGENTS.md`，并把根仓库、
`src/ats_sentry_nav`、`src/sim/ats_mujoco_sim` 和
`src/sim/gazebo_simulator/rmu_gazebo_simulator` 视为独立 Git 仓库。

## 当前已知基线

- MuJoCo RMUC 2025 的默认 `start_z=0.381 m` 已由 hfield 高度、轮半径、模型前向计算和
  `test_rmuc_2025_scene.py` 验证；四轮接触 hfield 的 MuJoCo 证据已存在。
- Gazebo RMUC 2025 的 spawn 配置为 `z_pose=0.20 m`，`ros_gz_sim create` 已返回实体创建成功，
  裁判脚本已具备标准 shebang 和外部 shutdown 清理；但尚无独立 Gazebo physical-contact evaluator。
- MINCO 的 JPS、geometry preprocessing、ESDF refinement、S3 轨迹、yaw、footprint safety 和
  MPC reference 链路已有组件测试；RViz 已能显示 raw path、preprocessed guide、ESDF refined guide
  和最终 reference，debug marker 在安全提交后发布。
- 现有包级 `cpplint/copyright/xmllint` 仍有历史失败，不能把 focused test 结果写成全包通过。
- 历史运行 artifact 不自动代表当前 revision；任何地图、规划、安全或控制源码修改后必须重跑对应闭环。

## 下一阶段目标

1. 重新运行当前 revision 的 MuJoCo nominal `single` 和完整 `red_box`，确认 `start_z=0.381` 下
   终点、reference、MPC、selected command、离散 footprint collision 和 MuJoCo contact evaluator
   的独立结果。
2. 运行 Gazebo nominal 和 freshness admission，记录 Gazebo Transport LiDAR、ROS LiDAR、localization、
   TF、RTF、动作终态和 `p1_admission_evidence`。先解决任何输入长尾，再讨论规划参数。
3. 增加或接入独立 Gazebo physical-contact evaluator，区分轮地正常接触、底盘碰撞、墙体碰撞和
   接触力；不得由 `footprint_collisions=0` 推导物理碰撞为零。
4. 使用新的隔离 `ROS_DOMAIN_ID` 分别验证 adapter lease、projection service timeout、Point-LIO
   stale、真实 unknown cell、unreachable goal 和 recovery 后旧 reference 不复活。
5. 评估 P3 Nav2-free 准入：`launch_nav2:=false`、没有 Nav2 server、MINCO 不订阅 `/plan`，并用
   自研 goal/action 验证 feedback、result、cancel、preempt、timeout。P3 完成前不得声称 Nav2-free。

## 强制工作方式

- 修改前给出 DoD、精确文件范围、命令级验证清单、当前假设和风险转入条件。
- 先用 `rg` 定位行为 owner，再读完整函数、launch、参数、接口和现有测试；默认忽略 build/install/log。
- 规划地图、TF、速度、急停和底盘输入保持唯一 owner；保持车体系 `[vx, vy, wz]` 和世界系
  `[x, y, yaw]` 的既有契约。
- JPS、MINCO clearance、footprint gate、local repair 和 MPC 必须消费同一不可变规划快照。
- map unready/stale、全 unknown、unreachable、unsafe trajectory 和 MPC failure 必须确定性零速。
- 不删除 dead code，不覆盖未知用户修改，不使用 `git reset --hard`、`git checkout --` 或 force push。
- 静态配置、源码、单测、运行日志和终点/接触测量至少使用两类独立证据交叉验证；单一来源必须标注
  `[Confidence: Medium/Low]` 和证据边界。

## 最低验证门禁

```bash
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select <targets> --parallel-workers 1
colcon test --base-paths src --packages-select <targets>
colcon test-result --test-result-base build/<package> --verbose
python3 -m py_compile <changed_launch_files>
git diff --check
```

运行仿真时使用新的隔离 `ROS_DOMAIN_ID`、无 viewer/RViz 的 headless 配置，并保存完整日志和结构化
artifact。最终报告必须区分“已验证”“已实现未运行”“推断”“未实现”，记录四个仓库的 commit、
分支、push 结果，以及未覆盖范围。完成 P2 红框或 P3 准入前，不得引用未在当前 revision 重跑的历史
50 Hz、6 ms、内存、红框或实车性能数字。
