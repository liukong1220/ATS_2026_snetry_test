# ATS 2026 哨兵机器人工程优化路线图

> 基于全工程代码分析，针对轨迹优化、点云匹配重定位、视觉-导航融合等方向的系统性优化方案

> 当前对话已实际落地完成：
>
> - 阶段 1：恢复轨迹分层采样 + 分段放行
> - 阶段 2：恢复执行中的动态障碍简单速度预测
>
> 相关实现与细节文档见：
>
> - [omni_recovery_smoothing_optimization.md](./omni_recovery_smoothing_optimization.md)

---

## 一、项目现状总览

### 1.1 系统架构

```
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

```
Livox Mid-360 → livox_ros_driver2 → Point-LIO → loam_interface
    → sensor_scan_generation → terrain_analysis → IntensityVoxelLayer → Nav2
    → fake_vel_transform → standard_robot_pp_ros2 → 串口 → 底盘

sp_vision25 → vision/target → pb2025_sentry_behavior → SendNavThroughPoses → Nav2
```

---

## 二、轨迹优化方案

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

**调优流程**:
1. 使用 `loopback_decision_sim.launch.py` 进行仿真测试
2. 通过 RViz 观察 `/trajectories` topic 的采样轨迹分布
3. 监控 `local_costmap/costmap_raw` 确认障碍物感知正确
4. 逐个参数调整，每次只改一个，观察效果

#### 2.2.2 全局路径平滑改进

**问题**: SmacPlannerHybrid 的内置平滑器迭代次数过多(1000次)，可能导致过度平滑。

**优化方案**:
```yaml
# 在 nav2_params.yaml 中调整
planner_server:
  ros__parameters:
    SmacPlannerHybrid:
      smoother:
        w_smooth: 0.25      # 降低平滑权重，保留更多原始路径特征
        w_data: 0.3          # 增加数据权重，保持路径接近原始规划
        num_iterations: 500  # 减少迭代次数，降低计算开销
        do_refinement: true  # 保留细化步骤
```

#### 2.2.3 速度规划优化

**问题**: 当前速度平滑器参数较为保守，可能限制了机器人的机动性能。

**优化方案**:
```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [4.0, 4.0, 5.5]      # 从 [3.5, 3.5, 5.0] 提升
    min_velocity: [-4.0, -4.0, -5.5]   # 允许更大反向速度
    velocity_timeout: 0.5               # 缩短超时，提高响应性
```

#### 2.2.4 轨迹预测可视化增强

**新增功能**: 添加轨迹预测可视化节点
- 发布 MPPI 预测的最优轨迹到 `/predicted_trajectory`
- 发布各 Critic 的代价分布到 `/critic_costs`
- 便于调试和参数调优

---

## 三、点云匹配重定位优化

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

**方案**: 构建多分辨率 PCD 地图，提高匹配效率和鲁棒性

```cpp
// 伪代码示意
class MultiResolutionPCDMap {
    std::vector<pcl::PointCloud> maps;  // 不同分辨率
    std::vector<float> resolutions;      // [0.1, 0.2, 0.5, 1.0]
    
    // 粗匹配: 使用低分辨率地图快速定位
    Eigen::Matrix4f coarseAlign(pcl::PointCloud& input);
    
    // 精匹配: 使用高分辨率地图精确对齐
    Eigen::Matrix4f fineAlign(pcl::PointCloud& input, Eigen::Matrix4f initial);
};
```

**实现步骤**:
1. 使用 `pcd2pgm` 工具的点云处理逻辑，添加多分辨率下采样
2. 实现金字塔式匹配: 粗→细
3. 集成到 `small_gicp_relocalization` 节点

#### 3.2.2 动态障碍物滤波

**问题**: 比赛中其他机器人、裁判等动态物体会干扰点云匹配。

**优化方案**:
```cpp
// 在 terrain_analysis 中已有动态障碍物检测
// 可以复用该逻辑进行点云预处理

class DynamicObjectFilter {
    // 基于时间的点云差异检测
    // 移除与历史地图不一致的点
    pcl::PointCloud filter(const pcl::PointCloud& input, 
                          const pcl::PointCloud& map);
};
```

**集成位置**: 在 `small_gicp_relocalization` 的输入端添加滤波器

#### 3.2.3 重定位失败检测与恢复

**当前问题**: 重定位失败时没有有效的恢复机制。

**优化方案**:
```cpp
// 添加重定位质量评估
class RelocalizationQualityEstimator {
    float fitness_score;           // GICP 匹配分数
    float correspondence_ratio;    // 对应点比例
    float transformation_delta;    // 变换增量
    
    bool isRelocalizationValid();
    
    // 失败时的恢复策略
    void triggerRecovery();
    // 1. 增大搜索范围
    // 2. 降低匹配阈值
    // 3. 使用多假设跟踪
};
```

#### 3.2.4 基于特征的快速重定位

**方案**: 提取点云的几何特征（平面、边缘、角点），用于快速初始对齐

```cpp
// 特征提取
class FeatureExtractor {
    // 平面特征: 用于地面、墙面
    std::vector<Plane> extractPlanes(pcl::PointCloud& cloud);
    
    // 边缘特征: 用于墙角、台阶
    std::vector<Edge> extractEdges(pcl::PointCloud& cloud);
    
    // 基于特征的初始对齐
    Eigen::Matrix4f featureBasedAlignment(
        const std::vector<Feature>& source,
        const std::vector<Feature>& target);
};
```

#### 3.2.5 增量式地图更新

**方案**: 在比赛中动态更新地图，适应场地变化

```cpp
class IncrementalMapUpdate {
    // 将新的扫描合并到地图中
    void updateMap(const pcl::PointCloud& new_scan, 
                  const Eigen::Matrix4f& pose);
    
    // 移除过时的点（基于时间衰减）
    void decayOldPoints(float decay_time);
    
    // 保存更新后的地图
    void saveUpdatedMap(const std::string& path);
};
```

---

## 四、视觉-导航融合优化

### 4.1 当前状态分析

**现有架构**:
```
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

**问题**: 视觉推理频率（约30Hz）低于控制频率（50Hz），导致跟踪滞后。

**优化方案**:
```cpp
class VisionTargetPredictor {
    // 基于卡尔曼滤波的目标状态预测
    struct TargetState {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Vector3f acceleration;
        ros::Time timestamp;
    };
    
    // 预测未来时刻的目标位置
    TargetState predict(ros::Time future_time);
    
    // 在视觉帧间进行插值
    VisionTargetMsg interpolate(ros::Time query_time);
};
```

**集成位置**: `IsVisionTargetValid` 节点中添加预测逻辑

#### 4.2.2 攻击圆采样优化

**问题**: 当前攻击圆采样点可能落在障碍物内或不可通行区域。

**优化方案**:
```cpp
class AttackCircleSampler {
    // 当前实现: 在攻击圆上均匀采样
    // 优化: 基于 costmap 的自适应采样
    
    std::vector<Pose> sampleAdaptive(
        const Pose& enemy_pose,
        float attack_radius,
        const Costmap2D& costmap,
        const Pose& robot_pose);
    
    // 优先选择:
    // 1. 可通行区域
    // 2. 朝向敌人的方向
    // 3. 距离当前位置较近的点
    // 4. 有良好射击角度的点
};
```

#### 4.2.3 多目标优先级管理

**问题**: 当前仅跟踪单个目标，缺乏多目标切换策略。

**优化方案**:
```cpp
class MultiTargetManager {
    struct TargetPriority {
        float distance;           // 距离权重
        float threat_level;       // 威胁等级（基于敌人类型）
        float visibility;         // 可见性（置信度）
        float shooting_angle;     // 射击角度优势
    };
    
    // 计算综合优先级
    float calculatePriority(const VisionTargetMsg& target);
    
    // 平滑切换策略
    bool shouldSwitchTarget(const VisionTargetMsg& current,
                           const VisionTargetMsg& candidate);
    
    // 切换时的平滑过渡
    Pose computeTransitionPath(const Pose& from, const Pose& to);
};
```

#### 4.2.4 视觉丢失恢复增强

**问题**: 视觉目标丢失时，机器人行为不够智能。

**优化方案**:
```cpp
class VisionLossRecovery {
    enum RecoveryStrategy {
        HOLD_POSITION,      // 保持当前位置
        LAST_KNOWN_SEARCH,  // 在最后已知位置附近搜索
        PATROL_SEARCH,      // 切换到巡逻模式搜索
        RETREAT_TO_SAFE     // 撤退到安全位置
    };
    
    // 根据上下文选择恢复策略
    RecoveryStrategy selectStrategy(
        float time_since_loss,
        float last_confidence,
        const Pose& robot_pose,
        const Pose& last_target_pose);
    
    // 执行恢复行为
    void executeRecovery(RecoveryStrategy strategy);
};
```

#### 4.2.5 视觉-导航延迟补偿

**方案**: 端到端延迟测量与补偿

```cpp
class LatencyCompensator {
    // 测量各环节延迟
    struct LatencyBreakdown {
        float vision_inference;    // 视觉推理延迟
        float message_transport;   // 消息传输延迟
        float bt_decision;         // 行为树决策延迟
        float nav_planning;        // 导航规划延迟
        float control_execution;   // 控制执行延迟
    };
    
    // 总延迟
    float totalLatency();
    
    // 基于延迟的目标位置补偿
    Pose compensateForLatency(const Pose& target, float latency);
};
```

---

## 五、其他优化建议

### 5.1 性能优化

#### 5.1.1 行为树执行效率
- 当前行为树每 tick 都会遍历所有节点，建议添加条件缓存
- 对于 `IsRobotResourceMode` 等状态节点，仅在状态变化时重新评估

#### 5.1.2 Costmap 更新优化
- `IntensityVoxelLayer` 的体素大小可以动态调整
- 近距离使用高分辨率，远距离使用低分辨率

#### 5.1.3 TF 查找优化
- 当前多处使用 `tf_buffer_->lookupTransform()`，建议缓存常用变换
- 特别是 `odom → base_footprint` 等高频变换

### 5.2 鲁棒性增强

#### 5.2.1 传感器故障检测
```cpp
class SensorHealthMonitor {
    // 检测各传感器状态
    bool isLidarHealthy();
    bool isIMUHealthy();
    bool isVisionHealthy();
    bool isRefereeHealthy();
    
    // 故障时的降级策略
    void degradeToMinimalMode();
};
```

#### 5.2.2 通信超时处理
- 添加各 topic 的超时检测
- 超时时触发安全行为（停止或撤退）

#### 5.2.3 异常状态恢复
- 行为树卡死检测与重启
- Nav2 节点崩溃的自动恢复
- 串口通信断开的重连机制

### 5.3 调试与监控

#### 5.3.1 增强日志系统
```cpp
// 结构化日志，便于分析
RCLCPP_INFO_STREAM(logger, 
    "Decision: mode=" << mode << 
    " target=" << target <<
    " confidence=" << confidence <<
    " latency=" << latency << "ms");
```

#### 5.3.2 实时性能监控
- 添加各环节耗时统计
- 发布到 `/diagnostics` topic
- 集成到 PlotJuggler 进行实时可视化

#### 5.3.3 比赛数据记录
- 记录完整的决策过程
- 便于赛后分析和复盘

---

## 六、实施优先级

### P0 (立即实施)
1. **恢复链平滑化与脱困增强** - 已开始并已落地前两阶段
2. **MPPI 参数调优** - 直接影响导航性能
3. **视觉目标预测** - 减少跟踪滞后

### P1 (短期实施)
4. **攻击圆采样优化** - 提高视觉跟随质量
5. **动态障碍物滤波 / 动态障碍预测** - 提高局部恢复与重定位质量
6. **速度规划优化** - 提高机动性能

### P2 (中期实施)
7. **多分辨率点云地图** - 提高重定位效率
8. **多目标优先级管理** - 增强战术能力
9. **视觉丢失恢复增强** - 提高系统鲁棒性

### P3 (长期实施)
10. **增量式地图更新** - 适应场地变化
11. **基于特征的快速重定位** - 提高重定位速度
12. **端到端延迟补偿** - 提高跟踪精度

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

## 九、恢复链专项阶段推进记录

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

## 八、参考资源

### 8.1 官方文档
- [Nav2 官方文档](https://docs.nav2.org/)
- [BehaviorTree.CPP 文档](https://www.behaviortree.dev/)
- [MPPI Controller 文档](https://docs.nav2.org/configuration/packages/configuring-mpc.html)

### 8.2 论文参考
- Point-LIO: "Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry"
- GICP: "Generalized-ICP"
- MPPI: "Information Theoretic Model Predictive Control"

### 8.3 工程文档
- `docs/mppi_parameter_tuning_guide.md` - MPPI 调参指南
- `docs/sentry_bt_decision_checklist.md` - 决策树清单
- `docs/sentry_posture_switch_logic.md` - 姿态切换逻辑
- `docs/实机视觉跟随优化方案.md` - 视觉跟随优化

---

> 本文档基于 2026-05-03 的代码分析生成，建议定期更新以反映最新进展。
