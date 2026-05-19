builtin_interfaces/Time timestamp        # 视觉结果时间戳，行为树用它做新鲜度判断

uint8 TARGET_TYPE_UNKNOWN=0
uint8 TARGET_TYPE_HERO=1
uint8 TARGET_TYPE_ENGINEER=2
uint8 TARGET_TYPE_INFANTRY_3=3
uint8 TARGET_TYPE_INFANTRY_4=4
uint8 TARGET_TYPE_INFANTRY_5=5
uint8 TARGET_TYPE_SENTRY=6
uint8 TARGET_TYPE_OUTPOST=7
uint8 TARGET_TYPE_BASE=8

bool tracking                            # 视觉是否已经稳定追踪到目标
bool nav_hold                            # 视觉是否建议导航进入接管/压制状态
bool fire_permitted                      # 当前视觉链是否认为可以发弹

int32 target_id                          # 当前目标编号，延续视觉侧 +1 的编号约定
uint8 target_type                        # 当前目标类型，使用上面的 TARGET_TYPE_* 枚举

float32 confidence                       # 视觉置信度，当前阶段主要用于调试和观测
float32 target_distance                  # 目标距离，单位 m
float32 target_yaw                       # 视觉给出的云台目标 yaw，单位 rad
float32 target_pitch                     # 视觉给出的云台目标 pitch，单位 rad

geometry_msgs/Point target_position_gimbal  # 目标在云台坐标系下的位置
geometry_msgs/Point target_position_map     # 目标在地图/规划坐标系下的位置
bool has_target_position_map                # target_position_map 是否可被导航侧直接消费
string target_position_map_frame            # target_position_map 所属 frame，例如 map / odom
