# ATS 2026 哨兵机器人工程优化路线图

> 基于全工程代码分析，结合当前已经实际落地的恢复链优化，重构后的工程优化路线图。
>
> 这份文档的目标不是一次性把所有方向都推进，而是明确：
>
> 1. 哪些方向适合当前阶段继续做
> 2. 哪些方向值得后续排队推进
> 3. 哪些方向当前不建议立即投入，但保留细节供未来参考

---

## 0. 当前结论

### 0.1 已完成并验证通过的优化

当前已经实际完成并通过完整 `./build.sh` 编译验证的内容：

1. 恢复链平滑化基础重构
   - 恢复轨迹规划替代逐小步试探
   - 一阶低通 + 加减速限幅
   - 恢复内部状态滞回
2. 第一阶段
   - 轨迹走廊分层采样
   - 分段放行
3. 第二阶段
   - 动态障碍简单速度预测

详细实现说明见：

- [omni_recovery_smoothing_optimization.md](./omni_recovery_smoothing_optimization.md)

### 0.2 现阶段适合继续做的方向

按当前工程状态和上车收益排序，建议优先级如下：

1. 视觉-导航融合优化
   - 视觉目标预测与插值
   - 攻击圆采样优化
2. 恢复链第三阶段
   - 圆弧 / 样条恢复轨迹
3. 局部层动态障碍预测扩展
   - 把当前恢复前缀预测扩展到局部参考轨迹层
4. MPPI 精细调优
   - 在恢复链明显稳定后再做

### 0.3 当前不建议立即投入的大项

下面这些方向不是没有价值，而是当前阶段不建议优先做：

1. 多分辨率点云地图
2. 增量式地图更新
3. 基于特征的快速重定位
4. 端到端延迟补偿
5. 大范围全局规划器替换

原因：

1. 工程改动面大
2. 验证链更长
3. 对当前“上车即可见效”的帮助不如恢复链和视觉-局部融合直接

### 0.4 这份路线图的阅读方式

从现在开始，本文中的条目分成三类：

1. `A 类：现阶段适合继续做`
   - 建议直接进入开发队列
2. `B 类：后续可取，建议保留`
   - 先保留详细方案，等当前阶段稳定后再推进
3. `C 类：当前暂不建议立即投入`
   - 仅保留思路和细节，不作为最近几轮对话的主任务

---

## 一、项目现状总览

### 1.1 系统架构

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                           决策层 (pb2025_sentry_behavior)                    │
│  行为树 rmul_2026.xml                                                       │
│  ├── 受击旋转 (IsAttacked → PublishSpinSpeed)                               │
│  ├── 视觉覆盖 (vision_override_realtime)                                    │
│  ├── 仿真模式 (decision_simulation)                                         │
│  └── 裁判模式 (decision_referee)                                            │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           导航层 (pb2025_sentry_nav)                        │
│  Nav2 Stack: SmacPlannerHybrid + MPPIController + BackUpFreeSpace           │
│  定位: Point-LIO → loam_interface → small_gicp_relocalization              │
│  感知: terrain_analysis → IntensityVoxelLayer → costmap                     │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           执行层 (standard_robot_pp_ros2)                   │
│  串口通信 → STM32下位机 → 麦克纳姆底盘 + 云台 + 发射机构                      │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 核心数据流

```text
Livox Mid-360 → livox_ros_driver2 → Point-LIO → loam_interface
    → sensor_scan_generation → terrain_analysis → IntensityVoxelLayer → Nav2
    → fake_vel_transform → standard_robot_pp_ros2 → 串口 → 底盘

sp_vision25 → vision/target → pb2025_sentry_behavior → SendNavThroughPoses → Nav2
```

---

## 二、轨迹优化方案

> 分类结论：
>
> - `A 类：现阶段适合继续做`
>   - MPPI 参数精细化调优
>   - 轨迹预测可视化增强
> - `B 类：后续可取`
>   - 全局路径平滑改进
>   - 速度规划优化
> - `C 类：当前暂不建议立即投入`
>   - 从全局规划器层大改轨迹生成逻辑

### 2.1 当前状态分析

**现有控制器**: MPPI (Model Predictive Path Integral)
- 运动模型: Omni (全向)
- 预测步长: 56步 × 0.02s = 1.12s
- 采样批次: 750
- 速度限制: vx/vy ±3.5 m/s, wz ±3.0 rad/s
- 9个Critic: PathAlign, PathFollow, PathAngle, Goal, GoalAngle, Obstacles, Constraint, Twirling, VelocityDeadband

**现有规划器**: SmacPlannerHybrid (Hybrid A*)
- 运动模型: Dubin
- 最小转弯半径: 0.07m
- 角度量化: 64 bins
- 路径平滑: w_smooth=0.3, w_data=0.2, 1000次迭代

### 2.2 优化方向

#### 2.2.1 MPPI参数精细化调优

类别：`A 类：现阶段适合继续做`

**问题**: 当前参数为经验值，缺乏系统性调优依据。

**优化方案**:

| 参数 | 当前值 | 建议值 | 理由 |
|------|--------|--------|------|
| `batch_size` | 750 | 1000-1500 | 增加采样多样性，提高最优轨迹质量 |
| `time_steps` | 56 | 40-48 | 缩短预测时域，减少累积误差 |
| `vx_std/vy_std` | 0.30 | 0.20-0.25 | 减小采样方差，提高轨迹一致性 |
| `temperature` | 0.3 | 0.15-0.25 | 降低温度使最优轨迹权重更集中 |
| `PathAlignCritic.cost_weight` | 8.0 | 10.0-12.0 | 增强路径跟踪精度 |
| `ObstaclesCritic.critical_weight` | 3.0 | 5.0 | 增强近距离障碍物避让 |

**现阶段如何使用这部分建议**:

当前更建议：

1. 只做小步、可回退的参数微调
2. 重点围绕“局部轨迹稳定性”和“全向侧移一致性”来调
3. 不建议现在同时大改 5 个以上 MPPI 参数

**调优流程**:
1. 使用 `loopback_decision_sim.launch.py` 进行仿真测试
2. 通过 RViz 观察 `/trajectories` topic 的采样轨迹分布
3. 监控 `local_costmap/costmap_raw` 确认障碍物感知正确
4. 逐个参数调整，每次只改一个，观察效果

#### 2.2.2 全局路径平滑改进

类别：`B 类：后续可取`

**问题**: SmacPlannerHybrid 的内置平滑器迭代次数过多(1000次)，可能导致过度平滑。

**优化方案**:
```yaml
planner_server:
  ros__parameters:
    SmacPlannerHybrid:
      smoother:
        w_smooth: 0.25
        w_data: 0.3
        num_iterations: 500
        do_refinement: true
```

#### 2.2.3 速度规划优化

类别：`B 类：后续可取`

**问题**: 当前速度平滑器参数较为保守，可能限制了机器人的机动性能。

**优化方案**:
```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [4.0, 4.0, 5.5]
    min_velocity: [-4.0, -4.0, -5.5]
    velocity_timeout: 0.5
```

#### 2.2.4 轨迹预测可视化增强

类别：`A 类：现阶段适合继续做`

**新增功能**:

1. 发布 MPPI 预测的最优轨迹到 `/predicted_trajectory`
2. 发布各 Critic 的代价分布到 `/critic_costs`
3. 便于调试和参数调优

---

## 三、点云匹配重定位优化

> 分类结论：
>
> - `A 类：现阶段适合继续做`
>   - 动态障碍物滤波
>   - 重定位失败检测与恢复
> - `B 类：后续可取`
>   - 多分辨率点云地图
> - `C 类：当前暂不建议立即投入`
>   - 基于特征的快速重定位
>   - 增量式地图更新

### 3.1 当前状态分析

**现有方案**: small_gicp_relocalization
- 使用 GICP (Generalized Iterative Closest Point) 算法
- 匹配当前扫描与先验 PCD 地图
- 发布 `map → odom` TF 变换
- 依赖 Point-LIO 提供的初始位姿估计

**已知问题**:
1. 重定位精度受初始位姿估计影响大
2. 在特征稀疏区域（如空旷场地）重定位容易失败
3. 动态障碍物（如其他机器人）会干扰匹配
4. 计算开销较大，可能影响实时性

### 3.2 优化方向

#### 3.2.1 多分辨率点云地图

类别：`B 类：后续可取`

**方案**: 构建多分辨率 PCD 地图，提高匹配效率和鲁棒性

```cpp
class MultiResolutionPCDMap {
    std::vector<pcl::PointCloud> maps;
    std::vector<float> resolutions;

    Eigen::Matrix4f coarseAlign(pcl::PointCloud& input);
    Eigen::Matrix4f fineAlign(pcl::PointCloud& input, Eigen::Matrix4f initial);
};
```

**实现步骤**:
1. 使用 `pcd2pgm` 工具的点云处理逻辑，添加多分辨率下采样
2. 实现金字塔式匹配: 粗→细
3. 集成到 `small_gicp_relocalization` 节点

#### 3.2.2 动态障碍物滤波

类别：`A 类：现阶段适合继续做`

**问题**: 比赛中其他机器人、裁判等动态物体会干扰点云匹配。

**优化方案**:
```cpp
class DynamicObjectFilter {
    pcl::PointCloud filter(const pcl::PointCloud& input,
                           const pcl::PointCloud& map);
};
```

**现阶段建议**:

1. 优先做轻量动态障碍滤波
2. 尽量复用现有 `terrain_analysis` 的动态障碍判定信息
3. 不建议现在直接开重型点云目标跟踪

**集成位置**: 在 `small_gicp_relocalization` 的输入端添加滤波器

#### 3.2.3 重定位失败检测与恢复

类别：`A 类：现阶段适合继续做`

**当前问题**: 重定位失败时没有有效的恢复机制。

**优化方案**:
```cpp
class RelocalizationQualityEstimator {
    float fitness_score;
    float correspondence_ratio;
    float transformation_delta;

    bool isRelocalizationValid();
    void triggerRecovery();
};
```

#### 3.2.4 基于特征的快速重定位

类别：`C 类：当前暂不建议立即投入`

**方案**: 提取点云的几何特征（平面、边缘、角点），用于快速初始对齐

```cpp
class FeatureExtractor {
    std::vector<Plane> extractPlanes(pcl::PointCloud& cloud);
    std::vector<Edge> extractEdges(pcl::PointCloud& cloud);
    Eigen::Matrix4f featureBasedAlignment(
        const std::vector<Feature>& source,
        const std::vector<Feature>& target);
};
```

#### 3.2.5 增量式地图更新

类别：`C 类：当前暂不建议立即投入`

**方案**: 在比赛中动态更新地图，适应场地变化

```cpp
class IncrementalMapUpdate {
    void updateMap(const pcl::PointCloud& new_scan,
                   const Eigen::Matrix4f& pose);
    void decayOldPoints(float decay_time);
    void saveUpdatedMap(const std::string& path);
};
```

---

## 四、视觉-导航融合优化

> 分类结论：
>
> - `A 类：现阶段适合继续做`
>   - 视觉目标预测与插值
>   - 攻击圆采样优化
> - `B 类：后续可取`
>   - 多目标优先级管理
>   - 视觉丢失恢复增强
> - `C 类：当前暂不建议立即投入`
>   - 端到端延迟补偿

### 4.1 当前状态分析

**现有架构**:

```text
sp_vision25 (OpenVINO推理 + EKF跟踪 + MPC轨迹规划)
    → vision/target (VisionTargetMsg)
    → IsVisionTargetValid (9项检查 + 3层平滑)
    → SelectVisionFollowPath (攻击圆采样 + costmap筛选 + 角度平滑)
    → SendNavThroughPoses → Nav2执行
```

**已知问题**:
1. 视觉目标丢失时的恢复策略不够平滑
2. 攻击圆采样点可能落在不可通行区域
3. 多目标切换时的姿态抖动
4. 视觉延迟（推理+传输）导致的跟踪滞后

### 4.2 优化方向

#### 4.2.1 视觉目标预测与插值

类别：`A 类：现阶段适合继续做`

**问题**: 视觉推理频率（约30Hz）低于控制频率（50Hz），导致跟踪滞后。

**优化方案**:
```cpp
class VisionTargetPredictor {
    struct TargetState {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Vector3f acceleration;
        ros::Time timestamp;
    };

    TargetState predict(ros::Time future_time);
    VisionTargetMsg interpolate(ros::Time query_time);
};
```

**现阶段建议**:

1. 先做轻量级位置预测和帧间插值
2. 先保证单目标稳定跟随
3. 不建议现在就把视觉预测和全局导航大范围耦合

**集成位置**: `IsVisionTargetValid` 节点中添加预测逻辑

#### 4.2.2 攻击圆采样优化

类别：`A 类：现阶段适合继续做`

**问题**: 当前攻击圆采样点可能落在障碍物内或不可通行区域。

**优化方案**:
```cpp
class AttackCircleSampler {
    std::vector<Pose> sampleAdaptive(
        const Pose& enemy_pose,
        float attack_radius,
        const Costmap2D& costmap,
        const Pose& robot_pose);
};
```

优先选择：

1. 可通行区域
2. 朝向敌人的方向
3. 距离当前位置较近的点
4. 有良好射击角度的点

#### 4.2.3 多目标优先级管理（sp_vision25）中有多目标切换的写法且合理（忽略这个优化）

类别：`B 类：后续可取`

**问题**: 当前仅跟踪单个目标，缺乏多目标切换策略。

**优化方案**:
```cpp
class MultiTargetManager {
    struct TargetPriority {
        float distance;
        float threat_level;
        float visibility;
        float shooting_angle;
    };

    float calculatePriority(const VisionTargetMsg& target);
    bool shouldSwitchTarget(const VisionTargetMsg& current,
                            const VisionTargetMsg& candidate);
    Pose computeTransitionPath(const Pose& from, const Pose& to);
};
```

#### 4.2.4 视觉丢失恢复增强

类别：`B 类：后续可取`

**问题**: 视觉目标丢失时，机器人行为不够智能。

**优化方案**:
```cpp
class VisionLossRecovery {
    enum RecoveryStrategy {
        HOLD_POSITION,
        LAST_KNOWN_SEARCH,
        PATROL_SEARCH,
        RETREAT_TO_SAFE
    };

    RecoveryStrategy selectStrategy(
        float time_since_loss,
        float last_confidence,
        const Pose& robot_pose,
        const Pose& last_target_pose);

    void executeRecovery(RecoveryStrategy strategy);
};
```

#### 4.2.5 视觉-导航延迟补偿

类别：`C 类：当前暂不建议立即投入`

**方案**: 端到端延迟测量与补偿

```cpp
class LatencyCompensator {
    struct LatencyBreakdown {
        float vision_inference;
        float message_transport;
        float bt_decision;
        float nav_planning;
        float control_execution;
    };

    float totalLatency();
    Pose compensateForLatency(const Pose& target, float latency);
};
```

---

## 五、其他优化建议

> 分类结论：
>
> - `A 类：现阶段适合继续做`
>   - TF 查找优化
>   - 实时性能监控
>   - 比赛数据记录
> - `B 类：后续可取`
>   - 行为树执行效率
>   - Costmap 更新优化
>   - 通信超时处理
> - `C 类：当前暂不建议立即投入`
>   - 完整传感器健康管理系统
>   - 完整异常重启框架

### 5.1 性能优化

#### 5.1.1 行为树执行效率

类别：`B 类：后续可取`

- 当前行为树每 tick 都会遍历所有节点，建议添加条件缓存
- 对于 `IsRobotResourceMode` 等状态节点，仅在状态变化时重新评估

#### 5.1.2 Costmap 更新优化

类别：`B 类：后续可取`

- `IntensityVoxelLayer` 的体素大小可以动态调整
- 近距离使用高分辨率，远距离使用低分辨率

#### 5.1.3 TF 查找优化

类别：`A 类：现阶段适合继续做`

- 当前多处使用 `tf_buffer_->lookupTransform()`，建议缓存常用变换
- 特别是 `odom → base_footprint` 等高频变换

### 5.2 鲁棒性增强

#### 5.2.1 传感器故障检测

类别：`C 类：当前暂不建议立即投入`

```cpp
class SensorHealthMonitor {
    bool isLidarHealthy();
    bool isIMUHealthy();
    bool isVisionHealthy();
    bool isRefereeHealthy();
    void degradeToMinimalMode();
};
```

#### 5.2.2 通信超时处理

类别：`B 类：后续可取`

- 添加各 topic 的超时检测
- 超时时触发安全行为（停止或撤退）

#### 5.2.3 异常状态恢复

类别：`C 类：当前暂不建议立即投入`

- 行为树卡死检测与重启
- Nav2 节点崩溃的自动恢复
- 串口通信断开的重连机制

### 5.3 调试与监控

#### 5.3.1 增强日志系统

类别：`B 类：后续可取`

```cpp
RCLCPP_INFO_STREAM(logger,
    "Decision: mode=" << mode <<
    " target=" << target <<
    " confidence=" << confidence <<
    " latency=" << latency << "ms");
```

#### 5.3.2 实时性能监控

类别：`A 类：现阶段适合继续做`

- 添加各环节耗时统计
- 发布到 `/diagnostics` topic
- 集成到 PlotJuggler 进行实时可视化

#### 5.3.3 比赛数据记录

类别：`A 类：现阶段适合继续做`

- 记录完整的决策过程
- 便于赛后分析和复盘

---

## 六、重构后的实施优先级

### 6.1 现阶段适合继续做（建议最近几轮对话优先推进）

1. 视觉目标预测与插值
2. 攻击圆采样优化
3. MPPI 小步精细调优
4. 动态障碍物滤波（面向重定位）
5. 重定位失败检测与恢复
6. TF 查找优化
7. 实时性能监控与比赛数据记录

### 6.2 后续可取优化（建议在现阶段稳定后排队推进）

1. 全局路径平滑改进
2. 速度规划优化
3. 多分辨率点云地图
4. 多目标优先级管理
5. 视觉丢失恢复增强
6. 行为树执行效率优化
7. Costmap 更新优化
8. 通信超时处理

### 6.3 当前暂不建议立即投入（保留方案，未来再做）

1. 基于特征的快速重定位
2. 增量式地图更新
3. 端到端延迟补偿
4. 完整传感器健康管理系统
5. 完整异常重启框架

---

## 七、测试验证方案

### 7.1 仿真测试

- 使用 `loopback_decision_sim.launch.py` 进行功能验证
- 使用 `loopback_vision_test.launch.py` 进行视觉跟随测试
- 修改 `fake_decision_sim_inputs.py` 模拟各种场景

### 7.2 实机测试

- 使用 `mapping.sh` 建立测试地图
- 使用 `NAV2.sh` 进行导航测试
- 通过 RViz 实时监控各项指标

### 7.3 性能指标

| 指标 | 目标值 | 测量方法 |
|------|--------|----------|
| 重定位精度 | < 0.1m | 对比 ground truth |
| 重定位时间 | < 500ms | 计时器 |
| 视觉跟踪延迟 | < 100ms | 端到端测量 |
| 路径跟踪误差 | < 0.2m | 对比规划路径 |
| 避障成功率 | > 95% | 统计碰撞次数 |

---

## 八、恢复链专项阶段推进记录

### 阶段 1：恢复轨迹分层采样 + 分段放行

状态：`已完成并编译通过`

已落地内容：

1. 近距离密采样、远距离疏采样
2. 整条恢复轨迹不全通时，先释放最远可通安全段
3. 新增参数：
   - `near_sample_step`
   - `far_sample_step`
   - `layered_sampling_split_distance`
   - `far_corridor_lateral_step`
   - `minimum_release_distance`

主要收益：

1. 近处安全性更高
2. 远处计算量更低
3. 避免“远处不通就原地完全不动”的卡死

### 阶段 2：动态障碍简单速度预测

状态：`已完成并编译通过`

已落地内容：

1. 计算恢复轨迹前方的安全前缀距离
2. 估计安全前缀长度变化速度
3. 对安全前缀做短时匀速外推
4. 若预测到恢复走廊将很快被重新封堵，则提前进入 `BLOCKED`

新增参数：

1. `dynamic_obstacle_prediction_enabled`
2. `prediction_horizon_s`
3. `prefix_velocity_alpha`
4. `predictive_block_margin`

主要收益：

1. 不再只看当前帧是否可走
2. 对动态障碍重新逼近恢复走廊的情况反应更早
3. 比“撞到边界再急停”更平滑

### 下一步推荐阶段

建议继续做：

1. 将动态障碍预测从“恢复前缀层”扩展到“局部参考轨迹层”
2. 把当前恢复轨迹由直线中心线进一步升级为圆弧 / 样条恢复轨迹
3. 对 MPPI 的局部参考路径增加走廊安全校验

---

## 九、参考资源

### 9.1 官方文档

- [Nav2 官方文档](https://docs.nav2.org/)
- [BehaviorTree.CPP 文档](https://www.behaviortree.dev/)
- [MPPI Controller 文档](https://docs.nav2.org/configuration/packages/configuring-mpc.html)

### 9.2 论文参考

- Point-LIO: "Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry"
- GICP: "Generalized-ICP"
- MPPI: "Information Theoretic Model Predictive Control"

### 9.3 工程文档

- `docs/mppi_parameter_tuning_guide.md` - MPPI 调参指南
- `docs/sentry_bt_decision_checklist.md` - 决策树清单
- `docs/sentry_posture_switch_logic.md` - 姿态切换逻辑
- `docs/实机视觉跟随优化方案.md` - 视觉跟随优化
- `docs/omni_recovery_smoothing_optimization.md` - 恢复链平滑化专项说明

---

> 本文档基于 2026-05-03 之后的工程实际优化进展重构，后续建议继续按阶段更新，而不是重新生成一份全新路线图。
