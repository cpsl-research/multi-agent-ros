import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    coordinator_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "adversary",
        "coordinator.yml",
    )

    coordinator_node = Node(
        package="mar_adv",
        executable="coordinator",
        namespace="adversary",
        name="adversary_coordinator",
        parameters=[coordinator_config],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([coordinator_node])
