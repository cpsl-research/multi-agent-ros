import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("mar_bringup"),
                        "config",
                        "visualizer",
                        "multi_agent_config.rviz",
                    ),
                ],
            ),
        ]
    )
