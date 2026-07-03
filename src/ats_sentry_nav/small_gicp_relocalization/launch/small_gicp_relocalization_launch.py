
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "",
                "lidar_frame": "",
                "prior_pcd_file": "",
            }
        ],
    )

    return LaunchDescription([node])
