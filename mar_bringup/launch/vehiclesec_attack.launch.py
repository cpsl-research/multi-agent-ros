import os
import sys
from datetime import datetime
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_adversaries, log_launch_metadata


def generate_launch_description():
    run_name = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    output_folder = os.path.join("outputs", run_name)
    os.makedirs(output_folder)
    symlink = os.path.join("outputs", "last_run")
    if os.path.exists(symlink):
        os.remove(symlink)
    os.symlink(src=run_name, dst=symlink, target_is_directory=True)

    attack_is_coordinated = LaunchConfiguration("attack_is_coordinated")

    n_adversaries_launch_arg = DeclareLaunchArgument("n_adversaries")
    is_coordinated_launch_arg = DeclareLaunchArgument("attack_is_coordinated")

    base_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("mar_bringup"), "launch"),
                "/_vehiclesec_base.launch.py",
            ]
        ),
        launch_arguments={
            "output_folder": output_folder,
        }.items(),
    )

    adversaries = OpaqueFunction(function=partial(get_adversaries, output_folder))

    adversary_coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch", "nodes"
                ),
                "/adversary_coordinator.launch.py",
            ]
        ),
        launch_arguments={
            "adversary_name": "adversary_coordinator",
            "attack_coord_topic": "attack_directive",
        }.items(),
        condition=IfCondition(attack_is_coordinated),  # only on this condition
    )

    launch_metadata = OpaqueFunction(
        function=partial(log_launch_metadata, output_folder)
    )

    return LaunchDescription(
        [
            launch_metadata,
            base_nodes,
            n_adversaries_launch_arg,
            adversaries,
            is_coordinated_launch_arg,
            adversary_coordinator,
        ]
    )
