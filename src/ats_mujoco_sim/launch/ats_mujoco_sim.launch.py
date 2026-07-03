"""Launch the MuJoCo swerve chassis simulation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description."""
    model_path = LaunchConfiguration("model_path")
    scene_file = LaunchConfiguration("scene_file")
    map_dir = LaunchConfiguration("map_dir")
    map_manifest_path = LaunchConfiguration("map_manifest_path")
    map_ready_file = LaunchConfiguration("map_ready_file")
    map_ready_token = LaunchConfiguration("map_ready_token")
    map_wait_timeout_sec = LaunchConfiguration("map_wait_timeout_sec")
    sim_rate_hz = LaunchConfiguration("sim_rate_hz")
    feedback_rate_hz = LaunchConfiguration("feedback_rate_hz")
    truth_rate_hz = LaunchConfiguration("truth_rate_hz")
    command_timeout = LaunchConfiguration("command_timeout")
    show_viewer = LaunchConfiguration("show_viewer")
    use_viewer = LaunchConfiguration("use_viewer")
    viewer_rate_hz = LaunchConfiguration("viewer_rate_hz")
    enable_lidar = LaunchConfiguration("enable_lidar")
    lidar_backend = LaunchConfiguration("lidar_backend")
    lidar_line_mode = LaunchConfiguration("lidar_line_mode")
    lidar_rate_hz = LaunchConfiguration("lidar_rate_hz")
    lidar_rate_clock = LaunchConfiguration("lidar_rate_clock")
    lidar_state_rate_hz = LaunchConfiguration("lidar_state_rate_hz")
    lidar_horizontal_resolution_deg = LaunchConfiguration(
        "lidar_horizontal_resolution_deg"
    )
    lidar_topic = LaunchConfiguration("lidar_topic")
    enable_tof = LaunchConfiguration("enable_tof")
    tof_backend = LaunchConfiguration("tof_backend")
    tof_range = LaunchConfiguration("tof_range")
    tof_min_range = LaunchConfiguration("tof_min_range")
    tof_rate_hz = LaunchConfiguration("tof_rate_hz")
    tof_width = LaunchConfiguration("tof_width")
    tof_height = LaunchConfiguration("tof_height")
    tof_horizontal_fov_deg = LaunchConfiguration("tof_horizontal_fov_deg")
    tof_vertical_fov_deg = LaunchConfiguration("tof_vertical_fov_deg")
    tof_footprint_length = LaunchConfiguration("tof_footprint_length")
    tof_footprint_width = LaunchConfiguration("tof_footprint_width")
    tof_footprint_expand = LaunchConfiguration("tof_footprint_expand")
    tof_footprint_resolution = LaunchConfiguration("tof_footprint_resolution")
    tof_footprint_z_min = LaunchConfiguration("tof_footprint_z_min")
    tof_footprint_z_max = LaunchConfiguration("tof_footprint_z_max")
    merged_tof_topic = LaunchConfiguration("merged_tof_topic")
    left_tof_topic = LaunchConfiguration("left_tof_topic")
    right_tof_topic = LaunchConfiguration("right_tof_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    pose_cmd_topic = LaunchConfiguration("pose_cmd_topic")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_z = LaunchConfiguration("start_z")
    start_yaw = LaunchConfiguration("start_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("scene_file", default_value=""),
        DeclareLaunchArgument("map_dir", default_value=""),
        DeclareLaunchArgument("map_manifest_path", default_value=""),
        DeclareLaunchArgument("map_ready_file", default_value=""),
        DeclareLaunchArgument("map_ready_token", default_value=""),
        DeclareLaunchArgument("map_wait_timeout_sec", default_value="0.0"),
        DeclareLaunchArgument("sim_rate_hz", default_value="300.0"),
        DeclareLaunchArgument("feedback_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("truth_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("command_timeout", default_value="0.5"),
        DeclareLaunchArgument("show_viewer", default_value="true"),
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("viewer_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("enable_lidar", default_value="false"),
        DeclareLaunchArgument("lidar_backend", default_value="gpu"),
        DeclareLaunchArgument("lidar_line_mode", default_value="96"),
        DeclareLaunchArgument("lidar_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lidar_rate_clock", default_value="wall"),
        DeclareLaunchArgument("lidar_state_rate_hz", default_value="30.0"),
        DeclareLaunchArgument(
            "lidar_horizontal_resolution_deg",
            default_value="0.4",
        ),
        DeclareLaunchArgument("lidar_topic", default_value="/local_pointcloud"),
        DeclareLaunchArgument("enable_tof", default_value="true"),
        DeclareLaunchArgument("tof_backend", default_value="cpu"),
        DeclareLaunchArgument("tof_range", default_value="1.0"),
        DeclareLaunchArgument("tof_min_range", default_value="0.03"),
        DeclareLaunchArgument("tof_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("tof_width", default_value="248"),
        DeclareLaunchArgument("tof_height", default_value="180"),
        DeclareLaunchArgument("tof_horizontal_fov_deg", default_value="98.0"),
        DeclareLaunchArgument("tof_vertical_fov_deg", default_value="72.0"),
        DeclareLaunchArgument("tof_footprint_length", default_value="0.73"),
        DeclareLaunchArgument("tof_footprint_width", default_value="0.54"),
        DeclareLaunchArgument("tof_footprint_expand", default_value="1.0"),
        DeclareLaunchArgument("tof_footprint_resolution", default_value="0.05"),
        DeclareLaunchArgument("tof_footprint_z_min", default_value="0.1"),
        DeclareLaunchArgument("tof_footprint_z_max", default_value="0.65"),
        DeclareLaunchArgument(
            "merged_tof_topic",
            default_value="/perception/tof/points_merged",
        ),
        DeclareLaunchArgument("left_tof_topic", default_value="/left_tof/points"),
        DeclareLaunchArgument("right_tof_topic", default_value="/right_tof/points"),
        DeclareLaunchArgument("odom_topic", default_value="/localization"),
        DeclareLaunchArgument("pose_cmd_topic", default_value="/simulation/PoseSub"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_z", default_value="0.18"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        Node(
            package="ats_mujoco_sim",
            executable="ats_mujoco_sim",
            name="ats_mujoco_sim",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "scene_file": scene_file,
                "map_dir": map_dir,
                "map_manifest_path": map_manifest_path,
                "map_ready_file": map_ready_file,
                "map_ready_token": map_ready_token,
                "map_wait_timeout_sec": map_wait_timeout_sec,
                "sim_rate_hz": sim_rate_hz,
                "feedback_rate_hz": feedback_rate_hz,
                "truth_rate_hz": truth_rate_hz,
                "command_timeout": command_timeout,
                "show_viewer": show_viewer,
                "use_viewer": use_viewer,
                "viewer_rate_hz": viewer_rate_hz,
                "enable_lidar": enable_lidar,
                "lidar_backend": lidar_backend,
                "lidar_line_mode": lidar_line_mode,
                "lidar_rate_hz": lidar_rate_hz,
                "lidar_rate_clock": lidar_rate_clock,
                "lidar_state_rate_hz": lidar_state_rate_hz,
                "lidar_horizontal_resolution_deg": (
                    lidar_horizontal_resolution_deg
                ),
                "lidar_topic": lidar_topic,
                "enable_tof": enable_tof,
                "tof_backend": tof_backend,
                "tof_range": tof_range,
                "tof_min_range": tof_min_range,
                "tof_rate_hz": tof_rate_hz,
                "tof_width": tof_width,
                "tof_height": tof_height,
                "tof_horizontal_fov_deg": tof_horizontal_fov_deg,
                "tof_vertical_fov_deg": tof_vertical_fov_deg,
                "tof_footprint_length": tof_footprint_length,
                "tof_footprint_width": tof_footprint_width,
                "tof_footprint_expand": tof_footprint_expand,
                "tof_footprint_resolution": tof_footprint_resolution,
                "tof_footprint_z_min": tof_footprint_z_min,
                "tof_footprint_z_max": tof_footprint_z_max,
                "merged_tof_topic": merged_tof_topic,
                "left_tof_topic": left_tof_topic,
                "right_tof_topic": right_tof_topic,
                "odom_topic": odom_topic,
                "pose_cmd_topic": pose_cmd_topic,
                "start_x": start_x,
                "start_y": start_y,
                "start_z": start_z,
                "start_yaw": start_yaw,
            }],
        ),
    ])
