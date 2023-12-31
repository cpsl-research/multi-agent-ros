import os
import sys
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_infra_agents, get_simulator


def generate_launch_description():
    output_folder = LaunchConfiguration("output_folder")
    n_infrastructure_agents_launch_arg = DeclareLaunchArgument(
        "n_infrastructure_agents", default_value="4"
    )

    simulator = OpaqueFunction(function=partial(get_simulator, output_folder))

    ego_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch", "nodes"
                ),
                "/agent_passive.launch.py",
            ]
        ),
        launch_arguments={
            "agent_name": "ego",
            "output_folder": output_folder,
        }.items(),
    )

    inf_agents = OpaqueFunction(function=partial(get_infra_agents, output_folder))

    command_center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch", "nodes"
                ),
                "/command_center.launch.py",
            ]
        ),
        launch_arguments={
            "cc_pipeline": "command_center.py",
            "output_folder": output_folder,
        }.items(),
    )

    visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch", "nodes"
                ),
                "/visualizer.launch.py",
            ]
        )
    )

    return LaunchDescription(
        [
            n_infrastructure_agents_launch_arg,
            simulator,
            visualizer,
            ego_agent,
            inf_agents,
            command_center,
        ]
    )
