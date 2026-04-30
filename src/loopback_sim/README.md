# loopback_sim

当前项目中的轻量软件闭环仿真层。

这个包在当前仓库里的定位非常明确：

```text
接收 /cmd_vel
  -> 积分生成 /odom 和 TF
  -> 发布 /clock
  -> 基于静态地图生成 /scan
  -> 让 Nav2 和行为树在没有实车时仍然形成闭环
```

## 当前入口

loopback 仿真本体：

- [nav2_loopback_sim/loopback_simulator.py](./nav2_loopback_sim/loopback_simulator.py)

当前通常不直接单独启动本包，而是由：

- [../pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py](../pb2025_sentry_bringup/launch/loopback_decision_sim.launch.py)
- [../pb2025_sentry_bringup/launch/loopback_vision_test.launch.py](../pb2025_sentry_bringup/launch/loopback_vision_test.launch.py)

统一编排。

## 当前功能

### 1. 仿真位姿闭环

当前 loopback 会发布最小必要 TF 链：

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

这样 Nav2、行为树、RViz 都能使用统一坐标系。

### 2. 仿真时钟

当前 loopback 会持续发布：

- `/clock`

这对 `use_sim_time=True` 的行为树、Nav2、RViz 非常关键。

### 3. 假激光

当前 loopback 会基于静态地图做简化射线投射，生成：

- `/scan`

因此它不是纯空壳仿真，而是能给 local/global costmap 提供最基础环境反馈。

### 4. 重定位测试

当前 loopback 支持在运行中重新发送：

- `/initialpose`

用于测试：

1. Nav2 重定位后的路径更新
2. 行为树 `decision_current_pose` 刷新
3. 视觉跟随是否重新选择新的最近圆周点

## 当前参数文件

当前 loopback Nav2 参数入口：

- [params/nav2_params.yaml](./params/nav2_params.yaml)

这份参数文件控制：

1. planner / controller / behavior server
2. MPPI 局部控制器
3. goal checker / progress checker
4. local/global costmap
5. 恢复行为

## 当前注意事项

### 1. 不要同域起两套 loopback

当前实测结论：

如果同一个 `ROS_DOMAIN_ID` 里同时存在两套 loopback、或还有其他节点在发 `/clock`、TF、`/initialpose`，很容易出现：

- `Detected jump back in time`
- `Message Filter dropping message`
- `Vision override rejected`
- `decision_current_pose is stale`

因此推荐：

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py use_rviz:=True
```

当前 workspace 已通过环境 hook 默认设置 `ROS_DOMAIN_ID=90`，
因此日常只需要 `source install/setup.bash` 即可。

### 2. 当前视觉专测推荐命令

```bash
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py \
  use_rviz:=True \
  publish_referee_inputs:=True \
  current_hp:=400 \
  projectile_allowance_17mm:=200 \
  publish_vision_target:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_has_target_position_map:=True \
  vision_target_position_map_frame:=map \
  vision_target_position_map_x:=5.0 \
  vision_target_position_map_y:=2.0 \
  vision_target_position_map_z:=0.0 \
  vision_target_yaw:=0.30 \
  vision_target_pitch:=-0.06
```

### 3. 当前重定位测试命令

```bash
source install/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 1.5, y: 4.5, z: 0.0}, orientation: {z: 0.70710678, w: 0.70710678}}}}"
```

### 4. 运行时修改假输入参数

如果 `ros2 param set` 长时间没有返回 `Set parameter successful`，优先绕过
`ros2 daemon`：

```bash
source install/setup.bash
ros2 param set --no-daemon /fake_decision_sim_inputs current_hp 50
ros2 param set --no-daemon /fake_decision_sim_inputs decision_mode retreat
ros2 param get --no-daemon /fake_decision_sim_inputs current_hp
ros2 topic echo --no-daemon /referee/robot_status --once
```

普通 `ros2 param set` 会依赖本机 `ros2 daemon` 的图缓存。若 daemon 是在旧的
`ROS_DOMAIN_ID`、旧 overlay，或异常状态下启动的，CLI 可能卡住，但节点本身仍然正常。
这种情况下也可以重启 daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

## 当前维护建议

1. 调局部控制和轨迹抖动，优先改 `params/nav2_params.yaml`
2. 调姿态切换、视觉跟随、受击自旋，不在本包改，去 `pb2025_sentry_behavior`
3. 调假输入、视觉测试参数，不在本包改，去 `pb2025_sentry_bringup/scripts/fake_decision_sim_inputs.py`
4. 当前 loopback 和实车共用行为层视觉跟随算法，因此 loopback 问题优先先检查行为层，再判断是不是仿真器本身
