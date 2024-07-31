from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    n_agents = LaunchConfiguration("n_agents")

    mate_config = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "trust",
            "mate.yaml",
        ]
    )

    cc_trust_node = Node(
        package="trust",
        executable="mate",
        namespace="mate",
        name="trust_estimator",
        parameters=[
            mate_config,
            {
                "n_agents": n_agents,
            },
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )


    return LaunchDescription([cc_trust_node])
