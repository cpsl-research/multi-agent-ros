import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    viz_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "visualizer",
        "visualizer.yml",
    )

    return LaunchDescription(
        [
            Node(
                package="mar_viz",
                executable="visualizer",
                name="visualizer",
                parameters=[viz_config],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("mar_bringup"),
                        "config",
                        "visualizer",
                        "rviz_configuration.rviz",
                    ),
                ],
            ),
        ]
    )
