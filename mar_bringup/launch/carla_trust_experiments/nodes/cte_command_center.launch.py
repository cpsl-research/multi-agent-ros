from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    n_agents = LaunchConfiguration("n_agents")

    cc_config = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "command_center",
            "track_fusion.yaml",
        ]
    )

    # sensor fusion
    cc_fusion_node = Node(
        package="avstack_bridge",
        executable="multi_platform_boxtracker3d",
        namespace="command_center",
        name="tracker",
        parameters=[
            cc_config,
            {
                "n_agents": n_agents,
            },
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([cc_fusion_node])
