import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "simulator",
        "carla_point_simulator.yml",
    )

    return LaunchDescription(
        [
            Node(
                package="mar_carla_sim",
                executable="point_simulator",
                name="simulator",
                parameters=[sim_config],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ]
    )
