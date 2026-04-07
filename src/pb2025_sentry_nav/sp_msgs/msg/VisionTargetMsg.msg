builtin_interfaces/Time timestamp        # 视觉结果时间戳，行为树用它做新鲜度判断

bool tracking                            # 视觉是否已经稳定追踪到目标
bool nav_hold                            # 视觉是否建议导航进入接管/压制状态
bool fire_permitted                      # 当前视觉链是否认为可以发弹

int32 target_id                          # 当前目标编号，延续视觉侧 +1 的编号约定
int32 suggested_goal_index               # 视觉建议前往的支援点编号，-1 表示未提供

float32 confidence                       # 视觉置信度，当前阶段主要用于调试和观测
float32 target_distance                  # 目标距离，单位 m
float32 target_yaw                       # 视觉给出的云台目标 yaw，单位 rad
float32 target_pitch                     # 视觉给出的云台目标 pitch，单位 rad

geometry_msgs/Point target_position_gimbal  # 目标在云台坐标系下的位置
geometry_msgs/Point target_position_map     # 目标在地图坐标系下的位置，暂未实装时保持零值
