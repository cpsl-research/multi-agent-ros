import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    coordinator_name = LaunchConfiguration("adversary_name")
    attack_coord_topic = LaunchConfiguration("attack_coord_topic")

    coordinator_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "adversary",
        "coordinator.yml",
    )

    coordinator_node = Node(
        package="mar_adv",
        executable="coordinator",
        namespace=coordinator_name,
        name="coordinator",
        parameters=[
            coordinator_config,
            {
                "attack_coord_topic": attack_coord_topic,
            },
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([coordinator_node])
