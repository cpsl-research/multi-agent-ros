from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    n_agents = LaunchConfiguration("n_agents")

    trust_estimator_config = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "trust",
            "trust_estimator.yaml",
        ]
    )

    trust_estimator_node = Node(
        package="trust",
        executable="estimator",
        namespace="mate",
        name="trust_estimator",
        parameters=[
            trust_estimator_config,
            {
                "n_agents": n_agents,
            },
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([trust_estimator_node])
