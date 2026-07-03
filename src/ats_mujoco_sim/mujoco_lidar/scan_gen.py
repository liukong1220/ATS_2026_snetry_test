"""
激光雷达扫描模式生成函数

此模块提供各种激光雷达的扫描模式生成函数，不依赖 taichi。
如需使用 Livox LiDAR，请从 scan_gen_livox 导入 LivoxGenerator。
"""

import os
from functools import lru_cache
from typing import Any

import numpy as np

AIRY96_VERTICAL_ANGLES_DEG = np.array(
    [
        -0.07,
        0.88,
        1.81,
        2.76,
        3.69,
        4.62,
        5.54,
        6.48,
        7.41,
        8.34,
        9.27,
        10.21,
        11.15,
        12.09,
        13.03,
        13.98,
        14.92,
        15.87,
        16.82,
        17.77,
        18.72,
        19.67,
        20.62,
        21.57,
        22.51,
        23.45,
        24.40,
        25.33,
        26.28,
        27.21,
        28.15,
        29.08,
        30.02,
        30.95,
        31.88,
        32.82,
        33.74,
        34.68,
        35.62,
        36.55,
        37.50,
        38.43,
        39.37,
        40.31,
        41.25,
        42.21,
        43.16,
        44.09,
        45.05,
        46.00,
        46.95,
        47.90,
        48.85,
        49.80,
        50.73,
        51.69,
        52.62,
        53.56,
        54.50,
        55.45,
        56.37,
        57.30,
        58.24,
        59.18,
        60.12,
        61.05,
        61.99,
        62.93,
        63.86,
        64.81,
        65.76,
        66.69,
        67.65,
        68.60,
        69.56,
        70.51,
        71.46,
        72.42,
        73.37,
        74.33,
        75.29,
        76.24,
        77.19,
        78.14,
        79.07,
        80.02,
        80.96,
        81.90,
        82.84,
        83.78,
        84.70,
        85.64,
        86.57,
        87.52,
        88.46,
        89.40,
    ],
    dtype=np.float32,
)
AIRY96_CHANNEL_DELAY_US = np.repeat(
    np.array(
        [
            0.0,
            5.712,
            12.376,
            19.040,
            25.704,
            33.320,
            41.888,
            50.456,
            59.024,
            70.448,
            81.872,
            93.296,
        ],
        dtype=np.float32,
    ),
    8,
)
AIRY96_MIN_RANGE = 0.1
AIRY96_MAX_RANGE = 60.0
AIRY96_FRAME_RATE_HZ = 10.0
AIRY96_ROTATION_RATE_HZ = 10.0
AIRY96_HORIZONTAL_RESOLUTION_DEG = 0.4
AIRY96_GAP_PERIOD_FRAMES = 10
AIRY96_GAP_ANGLE_DEG = 32.0
AIRY_LINE_MODES = (48, 96)


def get_airy_channel_indices(line_mode: int = 96) -> np.ndarray:
    if line_mode == 96:
        return np.arange(96, dtype=np.int32)
    if line_mode == 48:
        return np.arange(0, 96, 2, dtype=np.int32)
    raise ValueError(f"Unsupported Airy line_mode: {line_mode}, choose 48 or 96")


class LivoxGenerator:
    """生成 Livox 激光雷达的扫描模式"""

    livox_lidar_params: dict[str, dict[str, Any]] = {
        "avia": {
            "laser_min_range": 0.1,
            "laser_max_range": 200.0,
            "horizontal_fov": 70.4,
            "vertical_fov": 77.2,
            "samples": 24000,
        },
        "HAP": {
            "laser_min_range": 0.1,
            "laser_max_range": 200.0,
            "samples": 45300,
            "downsample": 1,
        },
        "horizon": {
            "laser_min_range": 0.1,
            "laser_max_range": 200.0,
            "horizontal_fov": 81.7,
            "vertical_fov": 25.1,
            "samples": 24000,
        },
        "mid40": {
            "laser_min_range": 0.1,
            "laser_max_range": 200.0,
            "horizontal_fov": 81.7,
            "vertical_fov": 25.1,
            "samples": 24000,
        },
        "mid70": {
            "laser_min_range": 0.1,
            "laser_max_range": 200.0,
            "horizontal_fov": 70.4,
            "vertical_fov": 70.4,
            "samples": 10000,
        },
        "mid360": {"laser_min_range": 0.1, "laser_max_range": 200.0, "samples": 24000},
        "tele": {
            "laser_min_range": 0.1,
            "laser_max_range": 200.0,
            "horizontal_fov": 14.5,
            "vertical_fov": 16.1,
            "samples": 24000,
        },
    }

    def __init__(self, name: str):
        if name in self.livox_lidar_params:
            self.laser_min_range = self.livox_lidar_params[name]["laser_min_range"]
            self.laser_max_range = self.livox_lidar_params[name]["laser_max_range"]
            self.samples = self.livox_lidar_params[name]["samples"]
            try:
                pattern_npy_path = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "scan_mode", f"{name}.npy"
                )
                self.ray_angles = np.load(pattern_npy_path)
            except FileNotFoundError:
                raise FileNotFoundError(
                    f"Scan mode file not found for {name}, file should be saved in {pattern_npy_path}"
                ) from None
            self.n_rays = len(self.ray_angles)
        else:
            raise ValueError(f"Invalid LiDAR name: {name}")
        self.currStartIndex = 0

    def sample_ray_angles(self, downsample: int = 1) -> tuple[np.ndarray, np.ndarray]:
        if self.currStartIndex + self.samples > self.n_rays:
            self.ray_part1 = self.ray_angles[self.currStartIndex :]
            self.ray_part2 = self.ray_angles[: self.samples - len(self.ray_part1)]
            self.currStartIndex = self.samples - len(self.ray_part1)
            self.ray_out = np.concatenate([self.ray_part1, self.ray_part2], axis=0)
        else:
            self.ray_part1 = self.ray_angles[
                self.currStartIndex : self.currStartIndex + self.samples
            ]
            self.currStartIndex += self.samples
            self.ray_out = self.ray_part1
        if downsample > 1:
            self.ray_out = self.ray_out[::downsample]
        return self.ray_out[:, 0], self.ray_out[:, 1]


# =======================================================================
# 生成网格状扫描模式
# =======================================================================
def generate_grid_scan_pattern(
    num_ray_cols: int,
    num_ray_rows: int,
    theta_range: tuple[float, float] = (-np.pi, np.pi),
    phi_range: tuple[float, float] = (-np.pi / 3, np.pi / 3),
) -> tuple[np.ndarray, np.ndarray]:
    """
    生成网格状扫描模式

    参数:
        num_ray_cols: 水平方向射线数
        num_ray_rows: 垂直方向射线数

    返回:
        (ray_theta, ray_phi): 水平角和垂直角数组
    """
    # 创建网格扫描模式
    theta_grid, phi_grid = np.meshgrid(
        np.linspace(theta_range[0], theta_range[1], num_ray_cols),  # 水平角
        np.linspace(phi_range[0], phi_range[1], num_ray_rows),  # 垂直角
    )

    # 展平网格为一维数组
    ray_phi = phi_grid.flatten()
    ray_theta = theta_grid.flatten()
    return ray_theta, ray_phi


# =======================================================================
# 创建激光雷达扫描线的角度数组，仅包含水平方向
# =======================================================================
def create_lidar_single_line(
    horizontal_resolution: int = 360, horizontal_fov: float = 2 * np.pi
) -> tuple[np.ndarray, np.ndarray]:
    """创建激光雷达扫描线的角度数组，仅包含水平方向"""
    h_angles = np.linspace(-horizontal_fov / 2, horizontal_fov / 2, horizontal_resolution)
    v_angles = np.zeros_like(h_angles)
    return h_angles, v_angles


# =======================================================================
# 1. Velodyne HDL-64 (任意 360° 旋转式激光雷达)
# =======================================================================
def generate_HDL64(  # |参数            | Velodyne HDL-64
    f_rot: float = 10.0,  # |转速 (Hz)       |  5-20Hz
    sample_rate: float = 1.1e6,  # |采样率 (Hz)     | 2.2MHz(双返回模式)
    n_channels: int = 64,  # |垂直通道数       | 64 (Vertical Angular Resolution : 0.4°)
    phi_fov: tuple[float, float] = (-24.9, 2.0),  # |垂直视场角 (度)  | (-24.9°, 2.°)
) -> tuple[np.ndarray, np.ndarray]:
    # 转换为弧度
    phi_min, phi_max = np.deg2rad(phi_fov)

    # 时间序列（列向量）
    t = np.arange(0, 1.0 / f_rot, n_channels / sample_rate)[:, None]  # shape: (n_times, 1)

    # 水平角计算（广播机制）
    theta = (2 * np.pi * f_rot * t) % (2 * np.pi)  # shape: (n_times, 1)

    # 垂直角（行向量）
    phi = np.linspace(phi_min, phi_max, n_channels)  # shape: (1, n_channels)

    # 生成网格（无需显式使用meshgrid）
    theta_grid = theta + np.zeros((1, n_channels))  # 广播至 (n_times, n_channels)
    phi_grid = np.zeros_like(theta) + phi  # 广播至 (n_times, n_channels)

    return theta_grid.flatten(), phi_grid.flatten()


# =======================================================================
# 2. Velodyne VLP-32 模式
# https://www.mapix.com/lidar-scanner-sensors/velodyne/velodyne-vlp-32c/
# =======================================================================
@lru_cache(maxsize=8)
def _get_vlp32_angles() -> np.ndarray:
    """使用缓存获取VLP-32的角度分布，避免重复计算，返回弧度值"""
    vlp32_angles = np.array(
        [
            -25.0,
            -22.5,
            -20.0,
            -15.0,
            -13.0,
            -10.0,
            -5.0,
            -3.0,
            -2.333,
            -1.0,
            -0.667,
            -0.333,
            0.0,
            0.0,
            0.333,
            0.667,
            1.0,
            1.333,
            1.667,
            2.0,
            2.333,
            2.667,
            3.0,
            3.333,
            3.667,
            4.0,
            5.0,
            7.0,
            10.0,
            15.0,
            17.0,
            20.0,
        ]
    )
    # 转换为弧度并裁剪
    vlp32_angles = np.deg2rad(vlp32_angles)
    return vlp32_angles


def generate_vlp32(
    f_rot: float = 10.0,  # 转速 (Hz)
    sample_rate: float = 1.2e6,  # 采样率 (Hz)
) -> tuple[np.ndarray, np.ndarray]:
    # 垂直角参数
    phi = _get_vlp32_angles()  # shape: (n_channels,)

    # 时间序列（列向量）
    t = np.arange(0, 1 / f_rot, 32 / sample_rate)[:, None]  # shape: (n_times, 1)

    # 水平角计算
    theta = (2 * np.pi * f_rot * t) % (2 * np.pi)  # shape: (n_times, 1)

    # 广播生成网格
    theta_grid = theta + np.zeros_like(phi)  # shape: (n_times, n_channels)
    phi_grid = np.zeros_like(theta) + phi  # shape: (n_times, n_channels)

    return theta_grid.flatten(), phi_grid.flatten()


# =======================================================================
# 3. Ouster OS-128 模式
# https://www.general-laser.at/en/shop-en/ouster-os0-128-lidar-sensor-en
# =======================================================================
def generate_os128(
    f_rot: float = 20.0,  # 转速 (Hz)
    sample_rate: float = 5.2e6,  # 采样率 (Hz)
) -> tuple[np.ndarray, np.ndarray]:
    # 垂直角参数（均匀分布）
    n_channels = 128
    phi = np.deg2rad(np.linspace(-22.5, 22.5, n_channels))  # shape: (n_channels,)

    # 时间序列（列向量）
    t = np.arange(0, 1 / f_rot, n_channels / sample_rate)[:, None]  # shape: (n_times, 1)

    # 水平角计算
    theta = (2 * np.pi * f_rot * t) % (2 * np.pi)  # shape: (n_times, 1)

    # 广播生成网格
    theta_grid = theta + np.zeros_like(phi)  # shape: (n_times, n_channels)
    phi_grid = np.zeros_like(theta) + phi  # shape: (n_times, n_channels)

    return theta_grid.flatten(), phi_grid.flatten()


# =======================================================================
# 4. Robosense Airy-96 模式
# 数据来自 Airy 产品手册 V1.2：96 线、0.4°水平分辨率、600 RPM、每 10 帧约 32°缺口。
# =======================================================================
def generate_airy96(
    frame_index: int | None = None,
    gap_start_angle_deg: float = 0.0,
    horizontal_resolution_deg: float = AIRY96_HORIZONTAL_RESOLUTION_DEG,
    rotation_rate_hz: float = AIRY96_ROTATION_RATE_HZ,
    line_mode: int = 96,
) -> tuple[np.ndarray, np.ndarray]:
    """
    生成 RoboSense Airy-96 单帧扫描模式。

    默认返回完整 360° 96 线帧，共 900 * 96 = 86400 条射线。line_mode=48 时
    返回 48 线帧；传入 frame_index 后，每 10 帧中的第 10 帧会按手册移除约
    32°扫描缺口，96 线 10 Hz 下平均点频为 856320 pts/s。
    """
    channel_indices = get_airy_channel_indices(line_mode)
    base_theta_deg = np.arange(0.0, 360.0, horizontal_resolution_deg, dtype=np.float32)
    if frame_index is not None and frame_index % AIRY96_GAP_PERIOD_FRAMES == 9:
        gap_end_angle_deg = gap_start_angle_deg + AIRY96_GAP_ANGLE_DEG
        theta_from_gap_start = (base_theta_deg - gap_start_angle_deg) % 360.0
        gap_width = (gap_end_angle_deg - gap_start_angle_deg) % 360.0
        base_theta_deg = base_theta_deg[theta_from_gap_start >= gap_width]

    theta_per_channel_deg = (
        AIRY96_CHANNEL_DELAY_US[channel_indices] * 1e-6 * rotation_rate_hz * 360.0
    )
    theta_grid_deg = base_theta_deg[:, np.newaxis] + theta_per_channel_deg[np.newaxis, :]
    phi_grid_deg = np.broadcast_to(
        AIRY96_VERTICAL_ANGLES_DEG[channel_indices][np.newaxis, :], theta_grid_deg.shape
    )

    theta = np.deg2rad(theta_grid_deg % 360.0).astype(np.float32, copy=False)
    phi = np.deg2rad(phi_grid_deg).astype(np.float32, copy=False)
    return theta.reshape(-1), phi.reshape(-1)


class AiryGenerator:
    """逐帧生成 Airy-96 扫描模式。"""

    laser_min_range = AIRY96_MIN_RANGE
    laser_max_range = AIRY96_MAX_RANGE
    frame_rate_hz = AIRY96_FRAME_RATE_HZ

    def __init__(
        self,
        gap_start_angle_deg: float = 0.0,
        horizontal_resolution_deg: float = AIRY96_HORIZONTAL_RESOLUTION_DEG,
        rotation_rate_hz: float = AIRY96_ROTATION_RATE_HZ,
        line_mode: int = 96,
    ) -> None:
        self.gap_start_angle_deg = gap_start_angle_deg
        self.horizontal_resolution_deg = horizontal_resolution_deg
        self.rotation_rate_hz = rotation_rate_hz
        self.line_mode = line_mode
        self.frame_index = 0
        self.channel_indices = get_airy_channel_indices(line_mode)

    def sample_ray_angles(self) -> tuple[np.ndarray, np.ndarray]:
        theta, phi = generate_airy96(
            frame_index=self.frame_index,
            gap_start_angle_deg=self.gap_start_angle_deg,
            horizontal_resolution_deg=self.horizontal_resolution_deg,
            rotation_rate_hz=self.rotation_rate_hz,
            line_mode=self.line_mode,
        )
        self.frame_index += 1
        return theta, phi
