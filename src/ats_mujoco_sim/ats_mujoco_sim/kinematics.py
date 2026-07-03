"""Simple four-swerve-drive kinematics for the WL100-style chassis."""

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, pi, radians, sin


MODE_SWERVE = 0
MODE_CRAB = 2
MODE_SPIN = 4
MODE_USER_CTRL = 8
MODE_PARK = 16

MODE_NAMES = {
    MODE_SWERVE: "swerve",
    MODE_CRAB: "crab",
    MODE_SPIN: "spin",
    MODE_USER_CTRL: "user_ctrl",
    MODE_PARK: "park",
}

WHEEL_ORDER = ("lf", "lr", "rf", "rr")

# Keep these positions in sync with models/swerve_chassis.xml.
WHEEL_POSITIONS = {
    "lf": (0.285, 0.215),
    "lr": (-0.285, 0.215),
    "rf": (0.285, -0.215),
    "rr": (-0.285, -0.215),
}

PARK_ANGLES_RAD = {
    "lf": radians(45.0),
    "lr": radians(-45.0),
    "rf": radians(-45.0),
    "rr": radians(45.0),
}


@dataclass
class ChassisCommand:
    """Body-frame chassis command, using meters and radians."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass
class WheelTarget:
    """Target for one swerve module."""

    name: str
    steer_angle: float
    wheel_speed: float

    @property
    def steer_angle_deg(self):
        """Return the steering target in degrees."""
        return degrees(self.steer_angle)


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


def optimize_steer_angle(angle, speed, previous_angle):
    """Flip wheel speed if that avoids a long steering rotation."""
    angle = normalize_angle(angle)
    previous_angle = normalize_angle(previous_angle)
    delta = normalize_angle(angle - previous_angle)

    if delta > pi / 2.0:
        return normalize_angle(angle - pi), -speed
    if delta < -pi / 2.0:
        return normalize_angle(angle + pi), -speed
    return angle, speed


# BUG 设置舵轮目标
def chassis_to_wheel_targets(command, previous_angles=None):
    """Convert a chassis velocity command to four wheel targets."""
    previous_angles = previous_angles or {}
    targets = []

    # for name in WHEEL_ORDER:
    #     x_pos, y_pos = WHEEL_POSITIONS[name]
    #     wheel_vx = command.linear_x - command.angular_z * y_pos
    #     wheel_vy = command.linear_y + command.angular_z * x_pos
    #     speed = hypot(wheel_vx, wheel_vy)

    #     if speed < 1e-6:
    #         angle = previous_angles.get(name, 0.0)
    #         speed = 0.0
    #     else:
    #         angle = atan2(wheel_vy, wheel_vx)
    #         angle, speed = optimize_steer_angle(
    #             angle,
    #             speed,
    #             previous_angles.get(name, 0.0),
    #         )

    #     targets.append(WheelTarget(name, angle, speed))



    # 阿克曼逆解算 R+左转 R-右转 command.linear_x必须正 command.angular_z+左转 command.angular_z-右转
    """
    x_pos, y_pos = WHEEL_POSITIONS['lf']
    if command.linear_x<0:
        targets.append(WheelTarget('lf', 0, 0))
        targets.append(WheelTarget('lr', 0, 0))
        targets.append(WheelTarget('rf', 0, 0))
        targets.append(WheelTarget('rr', 0, 0))
    elif abs(command.angular_z) > 1e-6:
        R = command.linear_x / command.angular_z
        alpha=atan2(2.0*x_pos, R + y_pos)
        if alpha>pi/2.0:
            alpha=alpha-pi
        beta=atan2(2.0*x_pos, R - y_pos)
        if beta>pi/2.0:
            beta=beta-pi

        targets.append(WheelTarget('lf', beta, abs(command.angular_z)*hypot(R-y_pos, 2.0*x_pos)))
        targets.append(WheelTarget('lr', 0, abs(command.angular_z)*(abs(R)-y_pos)))
        targets.append(WheelTarget('rf', alpha, abs(command.angular_z)*hypot(R+y_pos, 2.0*x_pos)))
        targets.append(WheelTarget('rr', 0, abs(command.angular_z)*(abs(R)+y_pos)))
        # targets.append(WheelTarget('lf', beta, 0))
        # targets.append(WheelTarget('lr', 0, 0))
        # targets.append(WheelTarget('rf', alpha, 0))
        # targets.append(WheelTarget('rr', 0, 0))
    else:
        targets.append(WheelTarget('lf', 0, command.linear_x))
        targets.append(WheelTarget('lr', 0, command.linear_x))
        targets.append(WheelTarget('rf', 0, command.linear_x))
        targets.append(WheelTarget('rr', 0, command.linear_x))
    """
    # 4WS逆解算 R+左转 R-右转 command.linear_x必须正 command.angular_z+左转 command.angular_z-右转
    # command.linear_x=0.5
    # command.angular_z=-0.5/1
    x_pos, y_pos = WHEEL_POSITIONS['lf']
    if command.linear_x<0:
        targets.append(WheelTarget('lf', 0, 0))
        targets.append(WheelTarget('lr', 0, 0))
        targets.append(WheelTarget('rf', 0, 0))
        targets.append(WheelTarget('rr', 0, 0))
    elif abs(command.angular_z) > 1e-6:
        R = command.linear_x / command.angular_z
        alpha=atan2(x_pos, R + y_pos)
        if alpha>pi/2.0:
            alpha=alpha-pi
        beta=atan2(x_pos, R - y_pos)
        if beta>pi/2.0:
            beta=beta-pi

        v_l=abs(command.angular_z)*hypot(R-y_pos, x_pos)
        v_r=abs(command.angular_z)*hypot(R+y_pos, x_pos)
        targets.append(WheelTarget('lf', beta, v_l))
        targets.append(WheelTarget('lr', -beta, v_l))
        targets.append(WheelTarget('rf', alpha, v_r))
        targets.append(WheelTarget('rr', -alpha, v_r))
        # targets.append(WheelTarget('lf', beta, 0))
        # targets.append(WheelTarget('lr', -beta, 0))
        # targets.append(WheelTarget('rf', alpha, 0))
        # targets.append(WheelTarget('rr', -alpha, 0))
    else:
        targets.append(WheelTarget('lf', 0, command.linear_x))
        targets.append(WheelTarget('lr', 0, command.linear_x))
        targets.append(WheelTarget('rf', 0, command.linear_x))
        targets.append(WheelTarget('rr', 0, command.linear_x))

    # targets.append(WheelTarget('lf', 0, 0))
    # targets.append(WheelTarget('lr', 0, 0))
    # targets.append(WheelTarget('rf', 0, 0))
    # targets.append(WheelTarget('rr', 0, 0))
    return targets


def park_wheel_targets():
    """Return a simple X-lock wheel pattern for parking."""
    return [
        WheelTarget(name, PARK_ANGLES_RAD[name], 0.0)
        for name in WHEEL_ORDER
    ]

# BUG 根据当前模式和命令计算每个轮子的目标转向角和车轮速度，返回给调用者并保存到 self.effective_motion_cmd 以供显示和记录日志用
def mode_to_wheel_targets(mode, command, previous_angles=None):
    """Apply WL100 motion-mode rules and return wheel targets."""
    previous_angles = previous_angles or {}

    # if mode == MODE_SWERVE:# 直接使用线速度和角速度计算每个轮子的目标转向角和车轮速度
    effective = ChassisCommand(command.linear_x, command.linear_y, command.angular_z)
    return effective, chassis_to_wheel_targets(effective, previous_angles)

    # if mode == MODE_CRAB:# 只使用线速度计算每个轮子的目标转向角和车轮速度，忽略角速度
    #     effective = ChassisCommand(command.linear_x, command.linear_y, 0.0)
    #     return effective, chassis_to_wheel_targets(effective, previous_angles)

    # if mode == MODE_SPIN:# 只使用角速度计算每个轮子的目标转向角和车轮速度，忽略线速度
    #     effective = ChassisCommand(0.0, 0.0, command.angular_z)
    #     return effective, chassis_to_wheel_targets(effective, previous_angles)

    # if mode == MODE_PARK:# 不使用线速度和角速度，直接设置每个轮子的目标转向角为 45 度或 -45 度，车轮速度为 0 来实现停车
    #     effective = ChassisCommand()
    #     return effective, park_wheel_targets()

    # effective = ChassisCommand()
    # # 对于其他模式，直接设置每个轮子的目标转向角为当前角度，车轮速度为 0 来保持原地不动
    # zero_targets = [
    #     WheelTarget(name, previous_angles.get(name, 0.0), 0.0)
    #     for name in WHEEL_ORDER
    # ]
    # return effective, zero_targets


def estimate_chassis_command(targets):
    """Estimate chassis motion from four wheel speed and steering targets."""
    if not targets:
        return ChassisCommand()

    linear_x = 0.0
    linear_y = 0.0
    angular_z = 0.0

    for target in targets:
        x_pos, y_pos = WHEEL_POSITIONS[target.name]
        wheel_vx = target.wheel_speed * cos(target.steer_angle)
        wheel_vy = target.wheel_speed * sin(target.steer_angle)
        linear_x += wheel_vx
        linear_y += wheel_vy

        radius_sq = x_pos * x_pos + y_pos * y_pos
        if radius_sq > 1e-9:
            angular_z += (-wheel_vx * y_pos + wheel_vy * x_pos) / radius_sq

    count = float(len(targets))
    return ChassisCommand(
        linear_x / count,
        linear_y / count,
        angular_z / count,
    )
