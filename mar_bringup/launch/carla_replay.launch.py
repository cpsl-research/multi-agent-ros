import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "simulator",
        "carla_point_simulator.yml",
    )

    output_folder = LaunchConfiguration("output_folder")

    return LaunchDescription(
        [
            Node(
                package="mar_carla_sim",
                executable="carla_simulator",
                name="simulator",
                parameters=[
                    sim_config,
                    {"output_folder": output_folder},
                ],
                arguments=["--ros-args", "--log-level", "INFO"],
                # remappings=[
                #     *[
                #         (f"/agent{i}/detections", remap)
                #         for i, remap in enumerate(agent_remaps_configs)
                #     ]
                # ],
                on_exit=Shutdown(),
            ),
        ]
    )
