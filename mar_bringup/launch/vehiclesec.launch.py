import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch"
                ),
                "/carla_point_simulator.launch.py",
            ]
        )
    )

    visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch"
                ),
                "/visualizer.launch.py",
            ]
        )
    )
        
    return LaunchDescription(
        [
            simulator,
            visualizer,
        ]
    )