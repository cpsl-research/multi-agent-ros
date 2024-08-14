from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    n_agents = LaunchConfiguration("n_agents")

    trust_measurement_config = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "trust",
            "trust_updater.yaml",
        ]
    )

    trust_updater_config = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "trust",
            "trust_updater.yaml",
        ]
    )

    trust_measurement_node = Node(
        package="trust",
        executable="measurement",
        namespace="mate",
        name="trust_measurement",
        parameters=[
            trust_measurement_config,
            {
                "n_agents": n_agents,
            },
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    trust_updater_node = Node(
        package="trust",
        executable="updater",
        namespace="mate",
        name="trust_updater",
        parameters=[
            trust_updater_config,
            {
                "n_agents": n_agents,
            },
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([trust_measurement_node, trust_updater_node])